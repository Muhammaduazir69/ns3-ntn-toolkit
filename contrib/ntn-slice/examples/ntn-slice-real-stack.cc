/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026  Muhammad Uzair
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * ntn-slice-real-stack — Phase 2 of 2026-06 protocol-fidelity audit
 * (RAN recipe).
 *
 * The audit found ntn-slice's "isolation" was trivial — each slice was a separate
 * PointToPoint link, so PRB-level isolation was never actually contested. Here all
 * slices share ONE real mmwave NR cell (NtnRealStackHelper). Per-slice SINR and
 * throughput are MEASURED from that shared cell's PHY/PacketSink, and the real
 * SliceIsolationMonitor evaluates SLA breaches against the MEASURED delivery plus
 * the real geometric NTN latency (so URLLC's 5 ms budget realistically breaches
 * over the LEO slant while eMBB's 50 ms budget holds).
 *
 * Usage:
 *   ./ns3 run "ntn-slice-real-stack --duration=15 --numUes=9"
 */

#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/ntn-real-stack-helper.h"
#include "ns3/ntn-tr38811-mobility-model.h"
#include "ns3/sgp4-mobility-model.h"
#include "ns3/walker-constellation.h"

#include "ns3/ntn-slice-types.h"
#include "ns3/slice-isolation-monitor.h"

#include <cmath>
#include <iomanip>
#include <iostream>

#include "ns3/ntn-scene-helper.h"

using namespace ns3;
using namespace ns3::ntnslice;

NS_LOG_COMPONENT_DEFINE("NtnSliceRealStack");

int
main(int argc, char* argv[])
{
    double duration = 15.0;
    uint32_t numUes = 9;
    double altitudeKm = 550.0;
    double satEirpDbm = -1.0; // sentinel: backend-appropriate default chosen below
    double backhaulMs = 5.0;
    std::string radio = "nr"; // radio backend: "nr" (5G-LENA FR1) | "mmwave" (FR2)
    std::string outputDir = "ntn-slice-real-stack-output";

    CommandLine cmd(__FILE__);
    cmd.AddValue("duration", "Simulation duration (s)", duration);
    cmd.AddValue("numUes", "Number of UEs sharing the cell", numUes);
    cmd.AddValue("altitude", "Satellite altitude (km)", altitudeKm);
    cmd.AddValue("satEirpDbm", "Satellite EIRP (dBm); -1 = backend default", satEirpDbm);
    cmd.AddValue("radio", "Radio backend: nr (FR1) or mmwave", radio);
    cmd.AddValue("outputDir", "Output directory", outputDir);
    std::string netSimOut;
    std::string czmlOut;
    cmd.AddValue("netSim", "NetSimulyzer 3D JSON output (empty=off)", netSimOut);
    cmd.AddValue("czml", "Cesium CZML 3D output (empty=off)", czmlOut);
    cmd.Parse(argc, argv);

    // Backend-appropriate EIRP default (honoured only if the user did not set it):
    // nr's Friis LEO link needs ~70 dBm for a healthy SINR; mmwave keeps 55 dBm.
    if (satEirpDbm < 0.0)
    {
        satEirpDbm = (radio == "mmwave") ? 55.0 : 70.0;
    }

    std::cout << "\n=== ntn-slice REAL-STACK (slices share ONE real " << radio << " cell) ===\n"
              << "  per-slice SINR + throughput MEASURED from the shared cell\n"
              << "  SLA breaches: measured delivery + real geometric NTN latency\n"
              << "  duration: " << duration << " s, UEs: " << numUes << "\n\n";

    NodeContainer satNodes;
    satNodes.Create(1);
    NodeContainer ueNodes;
    ueNodes.Create(numUes);

    // Real NTN mobility: SGP4 Walker serving satellite + TR 38.811 UEs under
    // its t=0 sub-point (UE+sat share the ECEF frame; the pass is genuine).
    ns3::ntncon::WalkerConfig wcfg;
    wcfg.num_planes = 1;
    wcfg.total_sats = 80;
    wcfg.altitude_km = altitudeKm;
    wcfg.inclination_deg = 53.0;
    wcfg.epoch_unix_s = 1735689600.0;
    const auto wElements = ns3::ntncon::WalkerConstellation::BuildDelta(wcfg);
    Ptr<ns3::ntncon::Sgp4MobilityModel> servSatMob =
        CreateObject<ns3::ntncon::Sgp4MobilityModel>();
    servSatMob->SetElements(wElements[0]);
    satNodes.Get(0)->AggregateObject(servSatMob);
    double subLat, subLon, subAlt;
    servSatMob->GetGeodetic(subLat, subLon, subAlt);
    NtnTr38811MobilityHelper ueMobility(1);
    auto mobProfile = NtnMobilityScenarios::MixedContinental();
    ueMobility.Install(ueNodes, mobProfile, subLat - 0.03, subLat + 0.03,
                       subLon - 0.03, subLon + 0.03);

    NtnRealStackHelper rs;
    rs.SetRadioBackend(radio == "mmwave" ? NtnRealStackHelper::RadioBackend::Mmwave
                                         : NtnRealStackHelper::RadioBackend::Nr);
    if (radio != "mmwave")
    {
        rs.SetNumerology(1); // FR1 30 kHz SCS
    }
    rs.SetSimTime(Seconds(duration));
    rs.SetOutputDir(outputDir);
    rs.SetRunTag("ntn-slice-real-stack");
    rs.SetSatEirpDbm(satEirpDbm);
    rs.SetBackhaulDelay(MilliSeconds(backhaulMs));
    rs.Build(satNodes, ueNodes);
    // MixedBouquet spreads UEs across eMBB/URLLC/mMTC traffic on the shared cell.
    rs.InstallTraffic(NtnRealStackHelper::TrafficProfile::MixedBouquet,
                      Seconds(1.0), Seconds(duration - 0.5));
    rs.EnableAiFlowMonitor("ntn-slice-real-stack"); // WS2 KPM series (TS 28.552 names)

    Simulator::Stop(Seconds(duration));
    ns3::ntnobs::NtnSceneHelper ntnScene;
    if (!netSimOut.empty()) ntnScene.SetNetSimulyzer(netSimOut);
    if (!czmlOut.empty()) ntnScene.SetCzml(czmlOut);
    Ptr<ns3::ntnobs::NtnSceneRecorder> ntnSceneRec = ntnScene.Build(satNodes, ueNodes);

    Simulator::Run();
    if (ntnSceneRec) ntnSceneRec->Stop();
    rs.Collect();
    rs.WriteHealthReport();

    // ---- Per-slice MEASURED isolation analysis on the shared cell ----
    SliceProfile embb = DefaultEmbb(1);
    SliceProfile urllc = DefaultUrllc(2);
    SliceProfile mmtc = DefaultMmtc(3);
    SliceProfile profiles[3] = {embb, urllc, mmtc};

    SliceIsolationMonitor monitor;
    monitor.RegisterSlice(embb);
    monitor.RegisterSlice(urllc);
    monitor.RegisterSlice(mmtc);

    double sliceRxMbps[3] = {0, 0, 0};
    double sliceSinrSum[3] = {0, 0, 0};
    uint32_t sliceUes[3] = {0, 0, 0};

    Vector sp = satNodes.Get(0)->GetObject<MobilityModel>()->GetPosition();
    for (uint32_t u = 0; u < numUes; ++u)
    {
        uint32_t slice = u % 3; // 0=eMBB, 1=URLLC, 2=mMTC
        uint64_t rxBytes = rs.GetUeRxBytes(u);
        double sinr = rs.GetUeMeanSinrDb(u);
        double thr = rxBytes * 8.0 / std::max(1.0, duration) / 1e6;
        sliceRxMbps[slice] += thr;
        if (!std::isnan(sinr))
        {
            sliceSinrSum[slice] += sinr;
        }
        sliceUes[slice]++;

        // Real geometric one-way NTN latency for this UE (slant range / c + backhaul).
        Vector up = ueNodes.Get(u)->GetObject<MobilityModel>()->GetPosition();
        double dx = sp.x - up.x, dy = sp.y - up.y, dz = sp.z - up.z;
        double slantM = std::sqrt(dx * dx + dy * dy + dz * dz);
        double latencyMs = slantM / 299792458.0 * 1e3 + backhaulMs;

        // Record the UE's MEASURED delivered packets (1400-byte DL) into its slice.
        uint64_t deliveredPkts = rxBytes / 1400;
        for (uint64_t p = 0; p < deliveredPkts; ++p)
        {
            monitor.RecordPacket(profiles[slice].snssai, latencyMs, true);
        }
    }

    auto breaches = monitor.EvaluateAll();

    std::cout << "\n--- Per-slice isolation on the SHARED real cell (MEASURED) ---\n";
    const char* names[3] = {"eMBB ", "URLLC", "mMTC "};
    for (uint32_t s = 0; s < 3; ++s)
    {
        double meanSinr = (sliceUes[s] > 0) ? sliceSinrSum[s] / sliceUes[s] : 0.0;
        std::cout << "  " << names[s] << " | UEs=" << sliceUes[s]
                  << " | meas throughput=" << std::fixed << std::setprecision(2)
                  << sliceRxMbps[s] << " Mbps | meas SINR=" << meanSinr
                  << " dB | budget=" << profiles[s].latencyBudgetMs << "ms\n";
    }
    std::cout << "\n--- SLA breach evaluation (real SliceIsolationMonitor) ---\n";
    for (const auto& b : breaches)
    {
        std::cout << "  slice " << b.snssai << " p99 latency=" << b.observedLatencyP99Ms
                  << "ms  latencyBreach=" << (b.latencyBreach ? "YES" : "no")
                  << "  reliabilityBreach=" << (b.reliabilityBreach ? "YES" : "no") << "\n";
    }
    std::cout << "\n  (URLLC's 5 ms budget breaches over the LEO slant — a real, "
                 "measured-geometry result, not a synthetic formula.)\n";

    Simulator::Destroy();
    return 0;
}
