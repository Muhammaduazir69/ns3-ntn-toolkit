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
#include <filesystem>
#include <fstream>
#include <system_error>

using namespace ns3;
using namespace ns3::ntnslice;

NS_LOG_COMPONENT_DEFINE("NtnSliceIsolationTraffic");

int
main(int argc, char* argv[])
{
    double simSeconds = 15.0;
    uint32_t numUes = 9; // %3 -> mMTC / eMBB / URLLC (MixedBouquet order)
    double satEirpDbm = -1.0; // sentinel: backend-appropriate default chosen below
    double backhaulMs = 5.0;
    std::string radio = "nr"; // radio backend: "nr" (5G-LENA FR1) | "mmwave" (FR2)
    std::string outputDir = "ntn-slice-isolation-output";

    CommandLine cmd(__FILE__);
    cmd.AddValue("simSeconds", "Simulation duration (s)", simSeconds);
    cmd.AddValue("numUes", "UEs on the shared cell (slice = ue%3)", numUes);
    cmd.AddValue("satEirpDbm", "Satellite EIRP / gNB Tx power (dBm); -1 = backend default", satEirpDbm);
    cmd.AddValue("radio", "Radio backend: nr (FR1) or mmwave", radio);
    cmd.AddValue("backhaulMs", "Feeder+core one-way delay (ms)", backhaulMs);
    cmd.AddValue("outputDir", "Output directory", outputDir);
    cmd.Parse(argc, argv);

    // Backend-appropriate EIRP default (honoured only if the user did not set it):
    // nr's Friis LEO link needs ~70 dBm for a healthy SINR; mmwave keeps 55 dBm.
    if (satEirpDbm < 0.0)
    {
        satEirpDbm = (radio == "mmwave") ? 55.0 : 70.0;
    }

    std::printf("# ntn-slice-isolation-traffic (3 slices CONTEND on ONE real %s cell)\n"
                "#   eMBB saturates the shared cell; URLLC/mMTC isolation is MEASURED\n"
                "#   sim=%.0fs UEs=%u EIRP=%.1fdBm\n",
                radio.c_str(), simSeconds, numUes, satEirpDbm);

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
    //      (u%3 -> mMTC periodic, eMBB saturating stream, URLLC pings) ----
    NtnRealStackHelper rs;
    rs.SetRadioBackend(radio == "mmwave" ? NtnRealStackHelper::RadioBackend::Mmwave
                                         : NtnRealStackHelper::RadioBackend::Nr);
    if (radio != "mmwave")
    {
        rs.SetNumerology(1); // FR1 30 kHz SCS
    }
    rs.SetSimTime(Seconds(simSeconds));
    rs.SetOutputDir(outputDir);
    rs.SetRunTag("ntn-slice-isolation-traffic");
    // NT-02: declared as CONDUCTED power at the array input. This carrier has
    // no TR 38.821 Set-1 reference in the toolkit, so the EIRP health gate
    // reports "not asserted" rather than certifying an uncalibrated budget.
    rs.SetSatConductedPowerDbm(satEirpDbm);
    rs.SetBackhaulDelay(MilliSeconds(backhaulMs));
    // GAP L1 FIX: real per-slice BWP isolation (see ntn-slice-real-stack). The
    // three 5QIs each get their own BWP + QoS scheduler instead of contending on
    // one default BWP.
    if (radio != "mmwave")
    {
        rs.SetSlices({{"eMBB", 2}, {"URLLC", 82}, {"mMTC", 9}});
    }
    rs.Build(satNodes, ueNodes);
    rs.InstallTraffic(NtnRealStackHelper::TrafficProfile::MixedBouquet,
                      Seconds(1.0), Seconds(simSeconds - 0.5));
    rs.EnableAiFlowMonitor("ntn-slice-isolation-traffic"); // WS2 KPM series (TS 28.552 names)

    Simulator::Stop(Seconds(simSeconds));
    Simulator::Run();
    rs.Collect();
    rs.WriteHealthReport();

    // ---- Per-slice MEASURED isolation analysis on the contended cell ----
    // Index order MUST match NtnRealStackHelper's MixedBouquet per-UE assignment
    // (u%3 -> 0:mMTC/NB-IoT 5QI9, 1:eMBB 5QI2, 2:URLLC 5QI82), else the per-slice
    // KPIs below are reported under the wrong slice labels.
    SliceProfile profiles[3] = {DefaultMmtc(3), DefaultEmbb(1), DefaultUrllc(2)};
    SliceIsolationMonitor monitor;
    for (auto& p : profiles)
    {
        monitor.RegisterSlice(p);
    }
    const char* names[3] = {"mMTC ", "eMBB ", "URLLC"};
    // GAP L2/L3/M5 FIX: feed the monitor from MEASURED per-slice in-band stats
    // (real delivered COUNT + measured OWD + sequence-gap losses), not
    // rxBytes/1400 with a closed-form geometric latency stamped delivered=true.
    const uint8_t sliceFiveQi[3] = {9, 2, 82}; // {mMTC, eMBB, URLLC}
    const uint8_t sliceBwp[3] = {2, 0, 1};     // SetSlices order eMBB=0,URLLC=1,mMTC=2
    double sliceMbps[3] = {0, 0, 0};
    double sliceSinr[3] = {0, 0, 0};
    uint64_t sliceRxPkts[3] = {0, 0, 0};
    double sliceOwd[3] = {0, 0, 0};
    uint64_t sliceLost[3] = {0, 0, 0};
    for (uint32_t s = 0; s < 3; ++s)
    {
        const auto st = rs.GetSliceMeasuredStats(sliceFiveQi[s]);
        sliceMbps[s] = st.thrMbps;
        sliceRxPkts[s] = st.rxPackets;
        sliceOwd[s] = st.meanOwdMs;
        sliceLost[s] = st.lostPackets;
        const double bwpSinr = rs.GetBwpMeanSinrDb(sliceBwp[s]);
        sliceSinr[s] = std::isnan(bwpSinr) ? 0.0 : bwpSinr;
        // SLICE-1 FIX (2026-08-24): replay the MEASURED delay DISTRIBUTION.
        // Stamping every packet with st.meanOwdMs made the monitor's p99 the
        // p99 of a constant, so a percentile SLA could never breach however
        // bad the tail was.
        {
            uint64_t replayed = 0;
            for (const auto& [binMs, count] : rs.GetSliceDelayHistogram(sliceFiveQi[s]))
            {
                for (uint64_t k = 0; k < count; ++k)
                {
                    monitor.RecordPacket(profiles[s].snssai, binMs + 0.5, /*delivered=*/true);
                    ++replayed;
                }
            }
            for (uint64_t k = replayed; k < st.rxPackets; ++k)
            {
                monitor.RecordPacket(profiles[s].snssai, st.maxOwdMs, /*delivered=*/true);
            }
        }
        for (uint64_t p = 0; p < st.lostPackets; ++p)
        {
            monitor.RecordPacket(profiles[s].snssai, st.meanOwdMs, /*delivered=*/false);
        }
    }
    auto breaches = monitor.EvaluateAll();

    std::printf("# === isolation summary (MEASURED on the shared contended cell) ===\n"
                "#   cell: measured SINR=%.2f dB  TBLER=%.4f  total throughput=%.3f Mbps\n",
                rs.GetMeanDlSinrDb(), rs.GetMeanDlTbler(), rs.GetRxThroughputMbps());
    for (int s = 0; s < 3; ++s)
    {
        std::printf("#   %s  meas thr=%6.3f Mbps  BWP SINR=%6.2f dB  (rxPkts=%lu)\n", names[s],
                    sliceMbps[s], sliceSinr[s], static_cast<unsigned long>(sliceRxPkts[s]));
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

    // Persist the PER-SLICE breakdown (the signature of this example): per-BWP
    // SINR, measured throughput, measured one-way delay and delivered/lost counts
    // per slice, plus each slice's SLA breach verdict. URLLC latency conformance
    // is judged against its configured budget (TS 23.501 5QI PDB) — over a LEO
    // slant a 5 ms URLLC budget is expected to BREACH, which the file now shows.
    {
        using SstT = decltype(profiles[0].snssai.sst);
        auto breachFor = [&](SstT sst, bool& lat, bool& rel) {
            lat = rel = false;
            for (const auto& b : breaches)
            {
                if (b.snssai.sst == sst)
                {
                    lat = b.latencyBreach;
                    rel = b.reliabilityBreach;
                }
            }
        };
        const SstT sliceSst[3] = {profiles[0].snssai.sst, profiles[1].snssai.sst,
                                  profiles[2].snssai.sst};
        std::error_code ec;
        std::filesystem::create_directories(outputDir, ec);
        std::ofstream f(outputDir + "/slices.csv");
        f << "slice,five_qi,sst,bwp,sinr_db,throughput_mbps,mean_owd_ms,rx_pkts,lost_pkts,"
             "latency_breach,reliability_breach,provenance\n";
        for (int s = 0; s < 3; ++s)
        {
            bool lat = false;
            bool rel = false;
            breachFor(sliceSst[s], lat, rel);
            f << names[s] << "," << static_cast<unsigned>(sliceFiveQi[s]) << ","
              << static_cast<unsigned>(sliceSst[s]) << "," << static_cast<unsigned>(sliceBwp[s])
              << "," << sliceSinr[s] << "," << sliceMbps[s] << "," << sliceOwd[s] << ","
              << static_cast<unsigned long>(sliceRxPkts[s]) << ","
              << static_cast<unsigned long>(sliceLost[s]) << "," << (lat ? 1 : 0) << ","
              << (rel ? 1 : 0) << ",measured-per-bwp\n";
        }
        f.close();
        std::printf("# wrote %s/slices.csv (per-slice measured KPIs + SLA verdicts)\n",
                    outputDir.c_str());
    }

    Simulator::Destroy();
    return 0;
}
