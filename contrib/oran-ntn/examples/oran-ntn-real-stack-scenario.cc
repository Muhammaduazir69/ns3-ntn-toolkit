/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026  Muhammad Uzair
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * O-RAN NTN — REAL-STACK scenario on the REAL NTN mobility architecture.
 *
 * The KPM that drives the Near-RT RIC + xApps is built from MEASURED per-UE
 * SINR/TBLER of a REAL mmwave NR NTN access link (NtnRealStackHelper:
 * SpectrumPhy + MAC + HARQ/AMC + RLC/PDCP + RRC + EPC), and the satellite
 * enrichment data the O-RAN NTN architecture feeds the RIC (elevation, Doppler,
 * time-to-exit) is computed from REAL SGP4 Walker orbits — the serving LEO
 * passes zenith and recedes while in-plane neighbours approach; nothing is
 * scripted or teleported. UEs move under 3GPP TR 38.811 §6.1.1.1 class
 * mobility (real ns-3 MobilityModel).
 *
 *   serving LEO gNB (SGP4) --(real mmwave radio, measured SINR)--> TR 38.811 UEs
 *        |                                                            |
 *        +--> E2 node --> Near-RT RIC + xApps  <-- measured KPM + real-ephemeris
 *   candidate sats (SGP4): ephemeris-predicted handover targets    enrichment
 *
 * mmwave does not scale to 66 sats x 100 UEs, so the real radio runs on the
 * serving cell over a tractable UE set; the wider Walker constellation provides
 * the real orbital/handover context (the accepted ns-O-RAN pattern).
 *
 * Usage:
 *   ./ns3 run "oran-ntn-real-stack-scenario --duration=30 --numUes=6"
 */

#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"

#include "ns3/ntn-real-stack-helper.h"
#include "ns3/ntn-tr38811-mobility-model.h"
#include "ns3/oran-ntn-a1-interface.h"
#include "ns3/oran-ntn-e2-interface.h"
#include "ns3/oran-ntn-helper.h"
#include "ns3/oran-ntn-near-rt-ric.h"
#include "ns3/oran-ntn-types.h"

#include "ns3/sgp4-mobility-model.h"
#include "ns3/walker-constellation.h"

#include <cmath>
#include <iomanip>
#include <iostream>
#include <map>
#include <vector>

#include "ns3/ntn-scene-helper.h"

using namespace ns3;
using ns3::ntncon::Sgp4MobilityModel;
using ns3::ntncon::WalkerConfig;
using ns3::ntncon::WalkerConstellation;

NS_LOG_COMPONENT_DEFINE("OranNtnRealStackScenario");

int
main(int argc, char* argv[])
{
    double duration = 30.0;
    uint32_t numUes = 6;
    uint32_t numSats = 6;        // 1 serving + candidates, all on real SGP4 orbits
    double altitudeKm = 550.0;
    double satEirpDbm = -1.0; // sentinel: backend-appropriate default chosen below
    double freqGhz = 2.0;
    double bwMhz = 50.0;
    double minElevDeg = 10.0;
    std::string radio = "nr"; // radio backend: nr (FR1) or mmwave
    std::string outputDir = "oran-ntn-real-stack-output";
    std::string conflictStrategy = "priority";

    CommandLine cmd(__FILE__);
    cmd.AddValue("duration", "Simulation duration (s)", duration);
    cmd.AddValue("numUes", "Number of TR 38.811 UEs on the serving cell", numUes);
    cmd.AddValue("numSats", "Number of satellites (1 serving + candidates)", numSats);
    cmd.AddValue("altitude", "Constellation altitude (km)", altitudeKm);
    cmd.AddValue("satEirpDbm", "Satellite EIRP / gNB Tx power (dBm); -1 = backend default", satEirpDbm);
    cmd.AddValue("radio", "Radio backend: nr (FR1) or mmwave", radio);
    cmd.AddValue("freqGhz", "Carrier frequency (GHz)", freqGhz);
    cmd.AddValue("bwMhz", "Bandwidth (MHz)", bwMhz);
    cmd.AddValue("minElev", "Min service elevation (deg)", minElevDeg);
    cmd.AddValue("outputDir", "Output directory", outputDir);
    cmd.AddValue("conflictStrategy", "priority|temporal|merge", conflictStrategy);
    std::string netSimOut;
    std::string czmlOut;
    cmd.AddValue("netSim", "NetSimulyzer 3D JSON output (empty=off)", netSimOut);
    cmd.AddValue("czml", "Cesium CZML 3D output (empty=off)", czmlOut);
    cmd.Parse(argc, argv);

    // nr's Friis LEO link needs ~70 dBm for a healthy SINR; mmwave keeps 55.
    if (satEirpDbm < 0.0)
    {
        satEirpDbm = (radio == "mmwave") ? 55.0 : 70.0;
    }

    std::cout << "\n=== O-RAN NTN REAL-STACK scenario (real SGP4 orbits + TR 38.811 UEs) ===\n"
              << "  serving cell: real mmwave NR link, " << numUes << " UEs\n"
              << "  constellation: " << numSats << " SGP4 Walker sats @ " << altitudeKm << " km\n"
              << "  KPM: MEASURED PHY SINR + real-ephemeris enrichment (elev/Doppler/TTE)\n"
              << "  duration: " << duration << " s\n\n";

    // ---- Real Walker-Delta constellation (SGP4) ----
    WalkerConfig wcfg;
    wcfg.num_planes = 1;
    wcfg.total_sats = 80; // 4.5 deg in-plane spacing; we instantiate numSats of them
    wcfg.altitude_km = altitudeKm;
    wcfg.inclination_deg = 53.0;
    wcfg.epoch_unix_s = 1735689600.0;
    const auto elements = WalkerConstellation::BuildDelta(wcfg);

    NodeContainer satNodes;
    satNodes.Create(numSats);
    std::vector<Ptr<Sgp4MobilityModel>> satMobs(numSats);
    for (uint32_t s = 0; s < numSats; ++s)
    {
        satMobs[s] = CreateObject<Sgp4MobilityModel>();
        // Serving = element 0; candidates = in-plane neighbours both ways.
        const uint32_t idx =
            (s == 0) ? 0 : ((s % 2 == 1) ? s / 2 + 1 : wcfg.total_sats - s / 2);
        satMobs[s]->SetElements(elements[idx % wcfg.total_sats]);
        satNodes.Get(s)->AggregateObject(satMobs[s]);
    }

    // ---- TR 38.811 UEs under the serving sat's t=0 sub-point ----
    double subLat, subLon, subAlt;
    satMobs[0]->GetGeodetic(subLat, subLon, subAlt);
    NodeContainer ueNodes;
    ueNodes.Create(numUes);
    NtnTr38811MobilityHelper ueMobility(1);
    auto profile = NtnMobilityScenarios::MixedContinental();
    auto ueModels = ueMobility.Install(ueNodes, profile, subLat - 0.03, subLat + 0.03,
                                       subLon - 0.03, subLon + 0.03);

    // ---- Real mmwave NTN access link on the serving satellite ----
    NodeContainer servingGnb(satNodes.Get(0));
    NtnRealStackHelper rs;
    rs.SetRadioBackend(radio == "mmwave" ? NtnRealStackHelper::RadioBackend::Mmwave
                                         : NtnRealStackHelper::RadioBackend::Nr);
    if (radio != "mmwave")
    {
        rs.SetNumerology(1); // FR1 30 kHz SCS
    }
    rs.SetSimTime(Seconds(duration));
    rs.SetOutputDir(outputDir);
    rs.SetRunTag("oran-ntn-real-stack");
    rs.SetCarrierFrequencyHz(freqGhz * 1e9);
    rs.SetBandwidthHz(bwMhz * 1e6);
    // NT-02: declared as CONDUCTED power at the array input. This carrier has
    // no TR 38.821 Set-1 reference in the toolkit, so the EIRP health gate
    // reports "not asserted" rather than certifying an uncalibrated budget.
    rs.SetSatConductedPowerDbm(satEirpDbm);
    rs.Build(servingGnb, ueNodes);
    rs.InstallTraffic(NtnRealStackHelper::TrafficProfile::EmbbStreaming,
                      Seconds(1.0), Seconds(duration - 0.5));

    // ---- O-RAN architecture ----
    auto helper = CreateObject<OranNtnHelper>();
    helper->SetOutputDirectory(outputDir);
    auto nonRtRic = helper->CreateNonRtRic();
    auto nearRtRic = helper->CreateNearRtRic();
    helper->ConnectA1Interface(nearRtRic, nonRtRic);

    auto cm = nearRtRic->GetConflictManager();
    if (conflictStrategy == "temporal")
        cm->SetResolutionStrategy(ConflictResolutionStrategy::TEMPORAL);
    else if (conflictStrategy == "merge")
        cm->SetResolutionStrategy(ConflictResolutionStrategy::MERGE);
    else
        cm->SetResolutionStrategy(ConflictResolutionStrategy::PRIORITY_BASED);

    auto satE2Nodes = helper->CreateSatelliteE2Nodes(satNodes, nearRtRic);
    helper->GenerateConstellationPolicies(nonRtRic, 1, numSats, 53.0, altitudeKm);
    auto xapps = helper->CreateAllXapps(nearRtRic);
    helper->StartAllXapps(nearRtRic);

    // ---- KPM feed: MEASURED PHY SINR + REAL-ephemeris enrichment ----
    // Every 100 ms: per UE, the serving SINR is read off the real radio; the
    // O-RAN NTN enrichment (elevation, Doppler, TTE) comes from the live SGP4
    // geometry. TTE = time until the sat sets below minElev, estimated from the
    // real elevation rate. Candidates report real elevation + a Friis-ratio
    // SINR prediction off the measured baseline (flagged prediction by design).
    std::map<uint32_t, double> lastElev; // satIdx -> last elevation (for d/dt)
    std::map<uint32_t, uint64_t> lastRxBytes; // ue -> last measured DL bytes
    const double kpmIntervalS = 0.1; // matches RegisterPeriodicCallback below
    auto kpmTick = [&, minElevDeg](Time) {
        for (uint32_t ue = 0; ue < numUes; ++ue)
        {
            const double sinr = rs.GetUeRecentSinrDb(ue);
            if (std::isnan(sinr))
            {
                continue;
            }
            const Vector uePos = ueModels[ue]->GetPosition();
            const Vector ueVel = ueModels[ue]->GetVelocity();

            // Serving: real ephemeris geometry.
            const Vector sp = satMobs[0]->GetPosition();
            const Vector sv = satMobs[0]->GetVelocity();
            const double elev = ntngeo::ElevationDeg(uePos, sp);
            const double servSlant = ntngeo::SlantRangeM(uePos, sp);
            const double doppler = ntngeo::DopplerHz(uePos, ueVel, sp, sv, freqGhz * 1e9);
            double elevRate = 0.0;
            if (auto it = lastElev.find(0); it != lastElev.end())
            {
                elevRate = (elev - it->second) / 0.1; // deg/s over the KPM period
            }
            lastElev[0] = elev;
            const double tte = (elevRate < -1e-3)
                                   ? std::max(1.0, (elev - minElevDeg) / -elevRate)
                                   : 600.0; // rising/flat: bounded "long" TTE
            // Serving cell rides the REAL radio: thread measured
            // bytes/TBLER/goodput into the KPM report (Global invariant 2).
            const uint64_t rxBytes = rs.GetUeRxBytes(ue);
            double measTbler = rs.GetUeRecentTbler(ue);
            if (std::isnan(measTbler))
            {
                measTbler = -1.0;
            }
            double measThp = -1.0;
            if (auto bit = lastRxBytes.find(ue);
                bit != lastRxBytes.end() && rxBytes >= bit->second)
            {
                measThp = (rxBytes - bit->second) * 8.0 / (kpmIntervalS * 1e6);
            }
            lastRxBytes[ue] = rxBytes;
            helper->InjectKpmReport(1, ue, sinr, sinr - 95.0, tte, elev, doppler,
                                    measThp, rxBytes, measTbler);

            // One ephemeris-predicted candidate per UE (round-robin).
            const uint32_t candIdx = 1 + (ue % std::max(1u, numSats - 1));
            const Vector cp = satMobs[candIdx]->GetPosition();
            const Vector cv = satMobs[candIdx]->GetVelocity();
            const double cElev = ntngeo::ElevationDeg(uePos, cp);
            const double cSlant = ntngeo::SlantRangeM(uePos, cp);
            const double cDoppler = ntngeo::DopplerHz(uePos, ueVel, cp, cv, freqGhz * 1e9);
            const double cSinr =
                sinr + 20.0 * std::log10(servSlant / std::max(1.0, cSlant)); // Friis ratio
            double cElevRate = 0.0;
            if (auto it = lastElev.find(candIdx); it != lastElev.end())
            {
                cElevRate = (cElev - it->second) / 0.1;
            }
            lastElev[candIdx] = cElev;
            const double cTte = (cElevRate < -1e-3)
                                    ? std::max(1.0, (cElev - minElevDeg) / -cElevRate)
                                    : 600.0;
            helper->InjectKpmReport(candIdx + 1, ue, cSinr, cSinr - 95.0, cTte, cElev, cDoppler);
        }
    };
    rs.RegisterPeriodicCallback(MilliSeconds(100), kpmTick);

    // ---- Run (the degradation is the REAL pass: serving sets, neighbours rise) --
    std::cout << "Running real-stack O-RAN simulation on real orbits...\n";
    Simulator::Stop(Seconds(duration));
    ns3::ntnobs::NtnSceneHelper ntnScene;
    if (!netSimOut.empty()) ntnScene.SetNetSimulyzer(netSimOut);
    if (!czmlOut.empty()) ntnScene.SetCzml(czmlOut);
    Ptr<ns3::ntnobs::NtnSceneRecorder> ntnSceneRec = ntnScene.Build(satNodes, ueNodes);

    Simulator::Run();
    if (ntnSceneRec) ntnSceneRec->Stop();
    rs.Collect();
    rs.WriteHealthReport();
    helper->WriteAllMetrics(nearRtRic);

    // ---- Summary ----
    auto m = nearRtRic->GetMetrics();
    std::cout << "\n--- RIC Summary (MEASURED KPM + real-ephemeris enrichment) ---\n"
              << "  measured serving SINR (mean): " << rs.GetMeanDlSinrDb() << " dB\n"
              << "  measured DL throughput:       " << rs.GetRxThroughputMbps() << " Mbps\n"
              << "  measured TBLER:               " << rs.GetMeanDlTbler() << "\n"
              << "  Active xApps:                 " << m.activeXapps << "\n"
              << "  E2 Nodes:                     " << m.totalE2Nodes << "\n"
              << "  Actions Processed:            " << m.totalActionsProcessed << "\n"
              << "  Conflicts:                    " << m.totalConflicts << "\n";
    for (const auto& [name, xapp] : xapps)
    {
        auto xm = xapp->GetMetrics();
        std::cout << "  xApp " << std::left << std::setw(18) << name
                  << " decisions=" << xm.totalDecisions
                  << " actions=" << xm.successfulActions << "/"
                  << (xm.successfulActions + xm.failedActions) << "\n";
    }
    std::cout << "\nResults in: " << outputDir << "/\n";

    Simulator::Destroy();
    return 0;
}
