/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only
//
// thz-ntn-leo-ground-downlink-traffic — END-TO-END packet transmission over a
// sub-THz LEO downlink on a REAL mmwave NR NTN cell (NtnRealStackHelper:
// SpectrumPhy + MAC + HARQ + RLC/PDCP + RRC + EPC).
//
// Audit fix (2026-06 protocol-fidelity audit, channel-plugin recipe):
// the old version computed FSPL + molecular absorption in a probe loop and
// drove a P2P RateErrorModel through a sigmoid SnrToPer() — packets never saw
// the THz channel. Here ThzNtnMolecularAbsorption, re-homed as
// ThzNtnPropagationLossModel (pure atmospheric EXCESS), is chained onto the
// real spectrum channel via AddExtraPropagationLoss(); FSPL comes from the
// stack's own Friis model over the live SGP4 geometry. As the satellite
// recedes the slant path lengthens, FSPL and the O2/H2O absorption grow, and
// the decline shows up in the MEASURED SINR / TBLER / goodput — driven by the
// channel, nothing closed-form in the packet path.
//
// The carrier is capped at 100 GHz by the 3GPP spectrum model (sub-THz /
// W-band); the 183 GHz water-vapour-line study stays in the analytic
// thz-ntn-leo-ground link-budget tool.
//
// Quick test:  --simSeconds=40
#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/ntn-real-stack-helper.h"
#include "ns3/ntn-tr38811-mobility-model.h"
#include "ns3/sgp4-mobility-model.h"
#include "ns3/thz-ntn-pointing-loss-model.h"
#include "ns3/thz-ntn-propagation-loss-model.h"
#include "ns3/walker-constellation.h"

#include <cmath>
#include <cstdio>
#include <string>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("ThzNtnLeoGroundDownlinkTraffic");

namespace
{

double
ElevDegEnu(const Vector& gnd, const Vector& sat)
{
    const double dx = sat.x - gnd.x;
    const double dy = sat.y - gnd.y;
    const double dz = sat.z - gnd.z;
    const double horiz = std::max(std::sqrt(dx * dx + dy * dy), 1e-3);
    return std::atan2(dz, horiz) * 180.0 / M_PI;
}

double
DistM(const Vector& a, const Vector& b)
{
    const double dx = a.x - b.x, dy = a.y - b.y, dz = a.z - b.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

} // namespace

int
main(int argc, char* argv[])
{
    double simSeconds = 60.0;
    double freqGHz = 100.0;    // sub-THz / W-band (3GPP spectrum model upper bound)
    double satEirpDbm = 115.0; // high-gain THz dishes both ends
    std::string outputDir = "thz-ntn-leo-ground-downlink-output";
    std::string radio = "nr"; // radio backend: nr (FR1) or mmwave

    CommandLine cmd(__FILE__);
    cmd.AddValue("simSeconds", "Simulation duration (s)", simSeconds);
    cmd.AddValue("freqGHz", "Carrier frequency (GHz), capped at 100", freqGHz);
    cmd.AddValue("satEirpDbm", "Satellite EIRP / gNB Tx power (dBm)", satEirpDbm);
    cmd.AddValue("radio", "Radio backend: nr (FR1) or mmwave", radio);
    cmd.AddValue("outputDir", "Output directory", outputDir);
    cmd.Parse(argc, argv);

    if (freqGHz > 100.0)
    {
        std::printf("# NOTE: 3GPP spectrum model caps the carrier at 100 GHz; "
                    "clamping %.0f -> 100 GHz\n",
                    freqGHz);
        freqGHz = 100.0;
    }

    std::printf("# thz-ntn-leo-ground-downlink-traffic (REAL radio, THz absorption in "
                "the packet path)\n");
    std::printf("#   sim=%.0fs freq=%.0fGHz EIRP=%.1fdBm\n", simSeconds, freqGHz,
                satEirpDbm);

    NodeContainer satNodes;
    satNodes.Create(1);
    NodeContainer gndNodes;
    gndNodes.Create(1);

    // Real SGP4 orbit projected into the local ENU frame: the serving Walker
    // element is at zenith at t=0 and recedes with genuine orbital dynamics.
    ns3::ntncon::WalkerConfig wcfg;
    wcfg.num_planes = 1;
    wcfg.total_sats = 80;
    wcfg.altitude_km = 550.0;
    wcfg.inclination_deg = 53.0;
    wcfg.epoch_unix_s = 1735689600.0;
    const auto elements = ns3::ntncon::WalkerConstellation::BuildDelta(wcfg);
    Ptr<ns3::ntncon::Sgp4MobilityModel> satSgp4 =
        CreateObject<ns3::ntncon::Sgp4MobilityModel>();
    satSgp4->SetElements(elements[0]);
    double subLat, subLon, subAlt;
    satSgp4->GetGeodetic(subLat, subLon, subAlt);
    Ptr<NtnEnuProjectionMobilityModel> satEnu = CreateObject<NtnEnuProjectionMobilityModel>();
    satEnu->SetSource(satSgp4);
    satEnu->SetReference(subLat, subLon, 0.0);
    satNodes.Get(0)->AggregateObject(satEnu);

    // The ground station is a fixed high-gain THz dish site at the sub-point.
    MobilityHelper mob;
    mob.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    Ptr<ListPositionAllocator> gndPos = CreateObject<ListPositionAllocator>();
    gndPos->Add(Vector(0.0, 0.0, 50.0));
    mob.SetPositionAllocator(gndPos);
    mob.Install(gndNodes);

    NtnRealStackHelper rs;
    rs.SetRadioBackend(radio == "mmwave" ? NtnRealStackHelper::RadioBackend::Mmwave
                                         : NtnRealStackHelper::RadioBackend::Nr);
    if (radio != "mmwave")
    {
        rs.SetNumerology(1); // FR1 30 kHz SCS
    }
    rs.SetSimTime(Seconds(simSeconds));
    rs.SetOutputDir(outputDir);
    rs.SetRunTag("thz-ntn-leo-ground-downlink-traffic");
    rs.SetCarrierFrequencyHz(freqGHz * 1e9);
    rs.SetSatEirpDbm(satEirpDbm);
    rs.Build(satNodes, gndNodes);

    // Channel plug-in: O2/H2O molecular slant-path absorption as pure EXCESS
    // chained AFTER the built-in Friis loss (no FSPL double-count).
    Ptr<ThzNtnPropagationLossModel> thz = CreateObject<ThzNtnPropagationLossModel>();
    thz->SetFrequency(freqGHz * 1e9);
    rs.AddExtraPropagationLoss(thz);

    // gap A2 — THz beam pointing impairment now in the measured packet path.
    Ptr<ThzNtnPointingLossModel> ptg = CreateObject<ThzNtnPointingLossModel>();
    rs.AddExtraPropagationLoss(ptg);

    rs.InstallTraffic(NtnRealStackHelper::TrafficProfile::EmbbStreaming,
                      Seconds(1.0), Seconds(simSeconds - 0.5));
    rs.EnableAiFlowMonitor("thz-ntn-leo-ground-downlink-traffic");

    std::printf("# %5s  %7s  %9s  %8s  %8s  %8s  %9s\n",
                "t_s", "elev", "slant_km", "molAbs", "sinr_dB", "tbler", "goodput");

    // 1 Hz probe: MEASURED SINR/TBLER off the PHY trace, goodput off the UE
    // PacketSink, molecular absorption off the live plug-in.
    Ptr<MobilityModel> gndMob = gndNodes.Get(0)->GetObject<MobilityModel>();
    uint64_t lastRx = 0;
    rs.RegisterPeriodicCallback(
        Seconds(1.0),
        [&rs, thz, gndMob, satEnu, &lastRx](Time now) {
            const Vector g = gndMob->GetPosition();
            const Vector s = satEnu->GetPosition();
            const double elev = ElevDegEnu(g, s);
            const double slantKm = DistM(g, s) / 1000.0;
            const double sinr = rs.GetUeRecentSinrDb(0);
            const double tbler = rs.GetUeRecentTbler(0);
            const uint64_t rx = rs.GetUeRxBytes(0);
            const double mbps = (rx - lastRx) * 8.0 / 1e6;
            lastRx = rx;
            std::printf("  %5.1f  %7.2f  %9.1f  %8.2f  %8.2f  %8.3f  %9.3f\n",
                        now.GetSeconds(), elev, slantKm, thz->GetLastLossDb(), sinr,
                        tbler, mbps);
        });

    Simulator::Stop(Seconds(simSeconds));
    Simulator::Run();
    rs.Collect();
    rs.WriteHealthReport();

    std::printf("# === summary ===  measured cell SINR=%.2f dB TBLER=%.4f "
                "throughput=%.3f Mbps (THz absorption applied to real packets)\n",
                rs.GetMeanDlSinrDb(), rs.GetMeanDlTbler(), rs.GetRxThroughputMbps());

    Simulator::Destroy();
    return 0;
}
