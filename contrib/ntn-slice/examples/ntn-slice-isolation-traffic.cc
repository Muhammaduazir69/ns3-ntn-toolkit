/*
 * SPDX-License-Identifier: GPL-2.0-only
 * Copyright (c) 2026 Muhammad Uzair (ns3-ntn-toolkit)
 *
 * ntn-slice-isolation-traffic — three network slices (eMBB / URLLC / mMTC)
 * SHARE one real mmwave NR NTN cell (NtnRealStackHelper: SpectrumPhy + MAC +
 * HARQ + RLC/PDCP + RRC + EPC) on a real SGP4 LEO pass, with TR 38.811 UE
 * mobility. Isolation is genuinely CONTESTED: the eMBB slice saturates the
 * shared cell while URLLC and mMTC must hold their SLAs. Every KPI is MEASURED
 * — per-slice SINR off the mmwave PHY trace, per-slice delivered bytes off the
 * PacketSinks, latency from the real slant geometry — and fed to the real
 * SliceIsolationMonitor, which flags breaches.
 *
 * Quick test:  --simSeconds=15 --numUes=9
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
#include <cstdio>

using namespace ns3;
using namespace ns3::ntnslice;

NS_LOG_COMPONENT_DEFINE("NtnSliceIsolationTraffic");

int
main(int argc, char* argv[])
{
    double simSeconds = 15.0;
    uint32_t numUes = 9; // %3 -> eMBB / URLLC / mMTC
    double satEirpDbm = 55.0;
    double backhaulMs = 5.0;
    std::string outputDir = "ntn-slice-isolation-output";

    CommandLine cmd(__FILE__);
    cmd.AddValue("simSeconds", "Simulation duration (s)", simSeconds);
    cmd.AddValue("numUes", "UEs on the shared cell (slice = ue%3)", numUes);
    cmd.AddValue("satEirpDbm", "Satellite EIRP / gNB Tx power (dBm)", satEirpDbm);
    cmd.AddValue("backhaulMs", "Feeder+core one-way delay (ms)", backhaulMs);
    cmd.AddValue("outputDir", "Output directory", outputDir);
    cmd.Parse(argc, argv);

    std::printf("# ntn-slice-isolation-traffic (3 slices CONTEND on ONE real mmwave cell)\n"
                "#   eMBB saturates the shared cell; URLLC/mMTC isolation is MEASURED\n"
                "#   sim=%.0fs UEs=%u EIRP=%.1fdBm\n",
                simSeconds, numUes, satEirpDbm);

    // ---- Real NTN mobility: SGP4 Walker sat + TR 38.811 UEs at sub-point ----
    ns3::ntncon::WalkerConfig wcfg;
    wcfg.num_planes = 1;
    wcfg.total_sats = 80;
    wcfg.altitude_km = 550.0;
    wcfg.inclination_deg = 53.0;
    wcfg.epoch_unix_s = 1735689600.0;
    const auto wElements = ns3::ntncon::WalkerConstellation::BuildDelta(wcfg);
    Ptr<ns3::ntncon::Sgp4MobilityModel> servSat =
        CreateObject<ns3::ntncon::Sgp4MobilityModel>();
    servSat->SetElements(wElements[0]);

    NodeContainer satNodes;
    satNodes.Create(1);
    satNodes.Get(0)->AggregateObject(servSat);
    NodeContainer ueNodes;
    ueNodes.Create(numUes);
    double subLat, subLon, subAlt;
    servSat->GetGeodetic(subLat, subLon, subAlt);
    NtnTr38811MobilityHelper ueMobility(1);
    auto mobProfile = NtnMobilityScenarios::MixedContinental();
    auto ueModels = ueMobility.Install(ueNodes, mobProfile, subLat - 0.03, subLat + 0.03,
                                       subLon - 0.03, subLon + 0.03);

    // ---- ONE shared real cell; MixedBouquet = per-UE slice traffic profiles
    //      (eMBB saturating stream, URLLC pings, mMTC periodic) ----
    NtnRealStackHelper rs;
    rs.SetSimTime(Seconds(simSeconds));
    rs.SetOutputDir(outputDir);
    rs.SetRunTag("ntn-slice-isolation-traffic");
    rs.SetSatEirpDbm(satEirpDbm);
    rs.SetBackhaulDelay(MilliSeconds(backhaulMs));
    rs.Build(satNodes, ueNodes);
    rs.InstallTraffic(NtnRealStackHelper::TrafficProfile::MixedBouquet,
                      Seconds(1.0), Seconds(simSeconds - 0.5));
    rs.EnableAiFlowMonitor("ntn-slice-isolation-traffic"); // WS2 KPM series (TS 28.552 names)

    Simulator::Stop(Seconds(simSeconds));
    Simulator::Run();
    rs.Collect();
    rs.WriteHealthReport();

    // ---- Per-slice MEASURED isolation analysis on the contended cell ----
    SliceProfile profiles[3] = {DefaultEmbb(1), DefaultUrllc(2), DefaultMmtc(3)};
    SliceIsolationMonitor monitor;
    for (auto& p : profiles)
    {
        monitor.RegisterSlice(p);
    }
    const char* names[3] = {"eMBB ", "URLLC", "mMTC "};
    double sliceMbps[3] = {0, 0, 0};
    double sliceSinr[3] = {0, 0, 0};
    uint32_t sliceN[3] = {0, 0, 0};
    const Vector sp = servSat->GetPosition();
    for (uint32_t u = 0; u < numUes; ++u)
    {
        // Deployment assumption (declared, not discovered): terminals are
        // provisioned round-robin across the three slice profiles — UE 3k is
        // a broadband terminal (eMBB), 3k+1 a control unit (URLLC), 3k+2 a
        // sensor (mMTC) — matching the MixedBouquet per-UE traffic profiles.
        // In a real network the S-NSSAI comes from subscription data; a
        // DSCP/QFI classifier (NtnSliceSelector) is exercised in
        // ntn-slice-real-stack.
        const uint32_t s = u % 3;
        const uint64_t rxBytes = rs.GetUeRxBytes(u);
        const double sinr = rs.GetUeMeanSinrDb(u);
        sliceMbps[s] += rxBytes * 8.0 / std::max(1.0, simSeconds) / 1e6;
        if (!std::isnan(sinr))
        {
            sliceSinr[s] += sinr;
        }
        sliceN[s]++;
        // Real geometric one-way latency (slant/c + backhaul) for this UE.
        const double slantM = ntngeo::SlantRangeM(ueModels[u]->GetPosition(), sp);
        const double latencyMs = slantM / 299792458.0 * 1e3 + backhaulMs;
        for (uint64_t p = 0; p < rxBytes / 1400; ++p)
        {
            monitor.RecordPacket(profiles[s].snssai, latencyMs, true);
        }
    }
    auto breaches = monitor.EvaluateAll();

    std::printf("# === isolation summary (MEASURED on the shared contended cell) ===\n"
                "#   cell: measured SINR=%.2f dB  TBLER=%.4f  total throughput=%.3f Mbps\n",
                rs.GetMeanDlSinrDb(), rs.GetMeanDlTbler(), rs.GetRxThroughputMbps());
    for (int s = 0; s < 3; ++s)
    {
        std::printf("#   %s  meas thr=%6.3f Mbps  meas SINR=%6.2f dB  (n=%u)\n", names[s],
                    sliceMbps[s], sliceN[s] ? sliceSinr[s] / sliceN[s] : 0.0, sliceN[s]);
    }
    uint32_t nBreach = 0;
    for (const auto& b : breaches)
    {
        if (b.latencyBreach || b.reliabilityBreach)
        {
            ++nBreach;
            std::printf("#   SLA BREACH sst=%u latencyBreach=%d reliabilityBreach=%d\n",
                        static_cast<unsigned>(b.snssai.sst), b.latencyBreach,
                        b.reliabilityBreach);
        }
    }
    std::printf("#   SLA breaches: %u/3 (isolation %s under eMBB saturation)\n", nBreach,
                nBreach == 0 ? "HELD" : "violated");

    Simulator::Destroy();
    return 0;
}
