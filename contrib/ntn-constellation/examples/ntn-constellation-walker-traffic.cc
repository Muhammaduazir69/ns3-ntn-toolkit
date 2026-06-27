/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only
//
// ntn-constellation-walker-traffic — a Walker-Delta constellation provides the
// ephemeris + ISL/GSL contact-graph context (ContactGraphScheduler), while ONE
// serving satellite carries a REAL mmwave NR NTN cell (NtnRealStackHelper) to a
// ground terminal placed at its sub-point. This mirrors the ns-O-RAN approach:
// the wider constellation is ephemeris-predicted context and the access link is
// a real measured radio. The radio KPIs (SINR/TBLER/throughput) are MEASURED off
// the mmwave PHY trace; the constellation-scale connectivity is reported from the
// contact graph. No closed-form SINR, no P2P star.
//
// Quick test:  --simSeconds=20 --numPlanes=4 --satsPerPlane=11

#include "ns3/command-line.h"
#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/ntn-real-stack-helper.h"
#include "ns3/ntn-tr38811-mobility-model.h"

#include "ns3/contact-graph-scheduler.h"
#include "ns3/sgp4-mobility-model.h"
#include "ns3/walker-constellation.h"

#include <cmath>
#include <cstdio>
#include <vector>

#include "ns3/ntn-scene-helper.h"

using namespace ns3;
using ns3::ntncon::ContactGraphScheduler;
using ns3::ntncon::Sgp4MobilityModel;
using ns3::ntncon::WalkerConfig;
using ns3::ntncon::WalkerConstellation;

namespace
{
Vector
GeodeticToEcef(double latDeg, double lonDeg, double altM)
{
    constexpr double kA = 6378137.0;
    constexpr double kF = 1.0 / 298.257223563;
    constexpr double kE2 = kF * (2.0 - kF);
    const double latR = latDeg * M_PI / 180.0;
    const double lonR = lonDeg * M_PI / 180.0;
    const double s = std::sin(latR), c = std::cos(latR);
    const double N = kA / std::sqrt(1.0 - kE2 * s * s);
    return Vector((N + altM) * c * std::cos(lonR),
                  (N + altM) * c * std::sin(lonR),
                  (N * (1.0 - kE2) + altM) * s);
}
} // namespace

int
main(int argc, char* argv[])
{
    double simSeconds = 20.0;
    uint32_t numPlanes = 4;
    uint32_t satsPerPlane = 11;
    double altKm = 550.0;
    double inclinationDeg = 53.0;
    double islRangeCapKm = 5000.0;
    uint32_t numUes = 4;
    double satEirpDbm = -1.0; // sentinel: backend-appropriate default chosen below
    std::string radio = "nr"; // radio backend: "nr" (5G-LENA FR1, 30 kHz SCS) | "mmwave" (FR2)
    std::string outputDir = "ntn-constellation-walker-output";

    CommandLine cmd(__FILE__);
    cmd.AddValue("simSeconds", "Simulation duration (s)", simSeconds);
    cmd.AddValue("numPlanes", "Walker orbital planes", numPlanes);
    cmd.AddValue("satsPerPlane", "Satellites per plane", satsPerPlane);
    cmd.AddValue("altKm", "Satellite altitude (km)", altKm);
    cmd.AddValue("inclinationDeg", "Orbital inclination (deg)", inclinationDeg);
    cmd.AddValue("islRangeCapKm", "Max ISL range for contact graph (km)", islRangeCapKm);
    cmd.AddValue("numUes", "Number of ground UEs on the serving cell", numUes);
    cmd.AddValue("satEirpDbm", "Satellite EIRP / gNB Tx power (dBm); -1 = backend default", satEirpDbm);
    cmd.AddValue("radio", "Radio backend: nr (5G-LENA FR1, 30 kHz SCS) | mmwave (FR2)", radio);
    cmd.AddValue("outputDir", "Output directory", outputDir);
    std::string netSimOut;
    std::string czmlOut;
    cmd.AddValue("netSim", "NetSimulyzer 3D JSON output (empty=off)", netSimOut);
    cmd.AddValue("czml", "Cesium CZML 3D output (empty=off)", czmlOut);
    cmd.Parse(argc, argv);

    const bool useNr = (radio != "mmwave");
    // Backend-appropriate EIRP default: nr's Friis LEO link needs ~70 dBm for a
    // healthy SINR; mmwave keeps its historical 55 dBm (zero regression).
    if (satEirpDbm < 0.0)
    {
        satEirpDbm = useNr ? 70.0 : 55.0;
    }

    // ---- Build the Walker constellation (ephemeris context) ----
    WalkerConfig wcfg;
    wcfg.num_planes = numPlanes;
    wcfg.total_sats = numPlanes * satsPerPlane;
    wcfg.altitude_km = altKm;
    wcfg.inclination_deg = inclinationDeg;
    wcfg.epoch_unix_s = 1735689600.0; // 2025-01-01
    const auto elements = WalkerConstellation::BuildDelta(wcfg);

    std::vector<Ptr<Sgp4MobilityModel>> sats;
    sats.reserve(elements.size());
    for (const auto& el : elements)
    {
        auto m = CreateObject<Sgp4MobilityModel>();
        m->SetElements(el);
        sats.push_back(m);
    }

    auto scheduler = CreateObject<ContactGraphScheduler>();
    scheduler->SetMinElevationDeg(10.0);
    scheduler->SetMaxIslRangeM(islRangeCapKm * 1e3);
    scheduler->RegisterGroundStation(0, 35.0, -75.0);
    scheduler->RegisterGroundStation(1, 35.0, 15.0);
    for (size_t i = 0; i < sats.size(); ++i)
    {
        scheduler->RegisterSatellite(static_cast<uint32_t>(i + 100), sats[i]);
    }

    // ---- Pick a serving satellite and place the ground UE at its sub-point so
    //      the real mmwave access link is in view (ns-O-RAN serving-cell pattern).
    Ptr<Sgp4MobilityModel> servSat = sats.front();
    double subLat, subLon, subAlt;
    servSat->GetGeodetic(subLat, subLon, subAlt);
    const Vector gsEcef = GeodeticToEcef(subLat, subLon, 540.0);

    NodeContainer satNodes;
    satNodes.Create(1);
    satNodes.Get(0)->AggregateObject(servSat);
    NodeContainer ueNodes;
    ueNodes.Create(numUes);
    // TR 38.811 class UEs (real MobilityModel) under the serving sub-point.
    NtnTr38811MobilityHelper ueMobility(1);
    auto mobProfile = NtnMobilityScenarios::MixedContinental();
    ueMobility.Install(ueNodes, mobProfile, subLat - 0.03, subLat + 0.03,
                       subLon - 0.03, subLon + 0.03);

    // ---- real NR serving cell + measured traffic (mmwave FR2 or nr FR1) ----
    NtnRealStackHelper rs;
    rs.SetRadioBackend(useNr ? NtnRealStackHelper::RadioBackend::Nr
                             : NtnRealStackHelper::RadioBackend::Mmwave);
    if (useNr)
    {
        rs.SetNumerology(1); // FR1 30 kHz SCS
    }
    rs.SetSimTime(Seconds(simSeconds));
    rs.SetOutputDir(outputDir);
    rs.SetRunTag("ntn-constellation-walker-traffic");
    rs.SetSatEirpDbm(satEirpDbm);
    rs.Build(satNodes, ueNodes);
    rs.InstallTraffic(NtnRealStackHelper::TrafficProfile::MixedBouquet,
                      Seconds(1.0), Seconds(simSeconds - 0.5));
    rs.EnableAiFlowMonitor("ntn-constellation-walker-traffic"); // WS2 KPM series (TS 28.552 names)

    scheduler->Start();
    Simulator::Stop(Seconds(simSeconds));
    ns3::ntnobs::NtnSceneHelper ntnScene;
    if (!netSimOut.empty()) ntnScene.SetNetSimulyzer(netSimOut);
    if (!czmlOut.empty()) ntnScene.SetCzml(czmlOut);
    Ptr<ns3::ntnobs::NtnSceneRecorder> ntnSceneRec = ntnScene.Build(satNodes, ueNodes);

    Simulator::Run();
    if (ntnSceneRec) ntnSceneRec->Stop();
    scheduler->Stop();
    rs.Collect();
    rs.WriteHealthReport();

    const uint64_t gslUp = scheduler->GslEventsUp();
    const uint64_t gslDown = scheduler->GslEventsDown();
    const uint64_t islUp = scheduler->IslEventsUp();
    const uint64_t islDown = scheduler->IslEventsDown();

    std::printf("# === ntn-constellation-walker-traffic summary ===\n"
                "#   Walker: planes=%u sats/plane=%u total=%u altKm=%.0f inc=%.1f deg\n"
                "#   serving cell on sat 0 (sub-point lat %.2f lon %.2f)\n"
                "#   measured SINR=%.2f dB  measured throughput=%.3f Mbps\n"
                "#   GSL up=%lu down=%lu  ISL up=%lu down=%lu\n",
                numPlanes, satsPerPlane,
                static_cast<unsigned>(numPlanes * satsPerPlane), altKm, inclinationDeg,
                subLat, subLon, rs.GetMeanDlSinrDb(), rs.GetRxThroughputMbps(),
                static_cast<unsigned long>(gslUp), static_cast<unsigned long>(gslDown),
                static_cast<unsigned long>(islUp), static_cast<unsigned long>(islDown));

    Simulator::Destroy();
    return 0;
}
