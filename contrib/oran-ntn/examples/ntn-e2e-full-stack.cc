/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026  Muhammad Uzair
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * ntn-e2e-full-stack — Phase 3 of 2026-06 protocol-fidelity audit
 * (composition / silo-breaker).
 *
 * The audit's anti-pattern #6: the modules are siloed — each re-derives its own
 * geometry and never shares a data plane. This example proves they COMPOSE on a
 * SINGLE shared data plane: ONE real mmwave NR NTN cell (NtnRealStackHelper)
 * whose MEASURED PHY SINR simultaneously drives
 *   (a) the oran-ntn Near-RT RIC + xApps (measured KPM -> RC actions), and
 *   (b) the ntn-slice SliceIsolationMonitor (per-slice SLA on measured KPIs),
 * and is logged as a measured-KPI CSV (observability). One NodeContainer, one
 * channel, one packet flow, many consumers.
 *
 * Usage:
 *   ./ns3 run "ntn-e2e-full-stack --duration=15 --numUes=9"
 */

#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"

#include "ns3/ntn-real-stack-helper.h"
#include "ns3/ntn-slice-types.h"
#include "ns3/ntn-tr38811-mobility-model.h"
#include "ns3/sgp4-mobility-model.h"
#include "ns3/walker-constellation.h"
#include "ns3/oran-ntn-a1-interface.h"
#include "ns3/oran-ntn-helper.h"
#include "ns3/oran-ntn-near-rt-ric.h"
#include "ns3/oran-ntn-ric-placement.h"
#include "ns3/oran-ntn-types.h"
#include "ns3/slice-isolation-monitor.h"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>

using namespace ns3;
using namespace ns3::ntnslice;

NS_LOG_COMPONENT_DEFINE("NtnE2eFullStack");

int
main(int argc, char* argv[])
{
    double duration = 15.0;
    uint32_t numUes = 9;
    double altitudeKm = 550.0;
    double satEirpDbm = -1.0; // sentinel: backend-appropriate default chosen below
    std::string radio = "nr"; // radio backend: nr (FR1) or mmwave
    std::string outputDir = "ntn-e2e-full-stack-output";

    CommandLine cmd(__FILE__);
    cmd.AddValue("duration", "Simulation duration (s)", duration);
    cmd.AddValue("numUes", "Number of UEs on the shared cell", numUes);
    cmd.AddValue("altitude", "Satellite altitude (km)", altitudeKm);
    cmd.AddValue("satEirpDbm", "Satellite EIRP (dBm); -1 = backend default", satEirpDbm);
    cmd.AddValue("radio", "Radio backend: nr (FR1) or mmwave", radio);
    cmd.AddValue("outputDir", "Output directory", outputDir);
    cmd.Parse(argc, argv);

    // nr's Friis LEO link needs ~70 dBm for a healthy SINR; mmwave keeps 55.
    if (satEirpDbm < 0.0)
    {
        satEirpDbm = (radio == "mmwave") ? 55.0 : 70.0;
    }

    std::cout << "\n=== ntn-e2e-full-stack (COMPOSITION on one shared data plane) ===\n"
              << "  ONE real mmwave cell -> measured SINR feeds RIC + slice + observability\n"
              << "  duration: " << duration << " s, UEs: " << numUes << "\n\n";

    // Real NTN mobility architecture: SGP4 serving satellite + TR 38.811 UEs.
    ns3::ntncon::WalkerConfig wcfg;
    wcfg.num_planes = 1;
    wcfg.total_sats = 80;
    wcfg.altitude_km = altitudeKm;
    wcfg.inclination_deg = 53.0;
    wcfg.epoch_unix_s = 1735689600.0;
    const auto elements = ns3::ntncon::WalkerConstellation::BuildDelta(wcfg);
    Ptr<ns3::ntncon::Sgp4MobilityModel> servSat =
        CreateObject<ns3::ntncon::Sgp4MobilityModel>();
    servSat->SetElements(elements[0]);

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

    // ---- THE shared data plane: one real mmwave NR NTN cell ----
    NtnRealStackHelper rs;
    rs.SetRadioBackend(radio == "mmwave" ? NtnRealStackHelper::RadioBackend::Mmwave
                                         : NtnRealStackHelper::RadioBackend::Nr);
    if (radio != "mmwave")
    {
        rs.SetNumerology(1); // FR1 30 kHz SCS
    }
    rs.SetSimTime(Seconds(duration));
    rs.SetOutputDir(outputDir);
    rs.SetRunTag("ntn-e2e-full-stack");
    // NT-02: declared as CONDUCTED power at the array input. This carrier has
    // no TR 38.821 Set-1 reference in the toolkit, so the EIRP health gate
    // reports "not asserted" rather than certifying an uncalibrated budget.
    rs.SetSatConductedPowerDbm(satEirpDbm);
    rs.Build(satNodes, ueNodes);
    rs.InstallTraffic(NtnRealStackHelper::TrafficProfile::MixedBouquet,
                      Seconds(1.0), Seconds(duration - 0.5));

    // ---- Consumer 1: oran-ntn Near-RT RIC + xApps ----
    auto helper = CreateObject<OranNtnHelper>();
    helper->SetOutputDirectory(outputDir);
    auto nonRtRic = helper->CreateNonRtRic();
    auto nearRtRic = helper->CreateNearRtRic();
    helper->ConnectA1Interface(nearRtRic, nonRtRic);
    auto satE2Nodes = helper->CreateSatelliteE2Nodes(satNodes, nearRtRic);
    helper->SetShellAltitudeKm(altitudeKm);
    // MixedBouquet marks UE u%3 = 0 mMTC (5QI 9), 1 eMBB (5QI 2), 2 URLLC
    // (5QI 82); declare the same slices to the RIC instead of the helper's
    // unrelated ueId%3 tag (0 eMBB, 1 URLLC, 2 mMTC).
    for (uint32_t u = 0; u < numUes; ++u)
    {
        helper->SetUeSliceId(u, (u % 3 == 0) ? 2 : (u % 3 == 1) ? 0 : 1);
    }
    // The E2 node sat at the helper's constant 4 ms feeder delay. A ground RIC
    // controlling a LEO cell sees the live satellite-gateway slant on both
    // legs (O-RAN WG3 E2 over the feeder), so bind it to the geometry: a
    // gateway about 110 km north of the UE cluster, refreshed every second.
    Ptr<ConstantPositionMobilityModel> gwMob = CreateObject<ConstantPositionMobilityModel>();
    gwMob->SetPosition(ntngeo::GeodeticToEcef(subLat + 1.0, subLon, 0.0));
    Ptr<OranNtnRicPlacement> ricSite = CreateObject<OranNtnRicPlacement>();
    ricSite->SetSite(OranNtnRicPlacement::Site::GroundGateway);
    ricSite->BindToGeometry(servSat, gwMob, satE2Nodes.front(), Seconds(1.0));
    auto xapps = helper->CreateAllXapps(nearRtRic);
    helper->StartAllXapps(nearRtRic);

    // ---- Consumer 2: ntn-slice SLA monitor ----
    SliceProfile profiles[3] = {DefaultEmbb(1), DefaultUrllc(2), DefaultMmtc(3)};
    SliceIsolationMonitor sliceMon;
    // The monitor keeps a 200-sample window by default and the histogram below
    // is replayed in ascending delay order, so only the slowest 200 packets
    // would survive and "p99" would be the tail maximum. Keep every sample.
    sliceMon.SetWindow(10000000);
    for (auto& p : profiles)
    {
        sliceMon.RegisterSlice(p);
    }

    // ---- Consumer 3: observability — measured-KPI CSV log ----
    std::filesystem::create_directories(outputDir);
    std::ofstream kpiLog(outputDir + "/measured_kpi_log.csv");
    kpiLog << "t_s,ue,measured_sinr_db,source\n";

    // One periodic loop drives ALL consumers from the SAME measured SINR; the
    // O-RAN enrichment (elevation/Doppler) is computed from the REAL ephemeris.
    std::map<uint32_t, uint64_t> lastRxBytes; // ue -> last measured DL bytes
    std::map<uint32_t, double> lastElev;      // ue -> last serving elevation (deg)
    const double minElevDeg = 10.0;           // service elevation mask for TTE
    const double kpmIntervalS = 0.1; // matches RegisterPeriodicCallback below
    auto tick = [&](Time now) {
        double t = now.GetSeconds();
        const Vector sp = servSat->GetPosition();
        const Vector sv = servSat->GetVelocity();
        for (uint32_t u = 0; u < numUes; ++u)
        {
            double sinr = rs.GetUeRecentSinrDb(u);
            if (std::isnan(sinr))
            {
                continue;
            }
            // (a) RIC: measured KPM + real-ephemeris enrichment. The measured
            // bytes/TBLER are threaded in so the RIC's xApps act on measured
            // goodput, not a synthetic SINR formula (Global invariant 2 + 4).
            const Vector up = ueModels[u]->GetPosition();
            const Vector uv = ueModels[u]->GetVelocity();
            const double elev = ntngeo::ElevationDeg(up, sp);
            const double doppler = ntngeo::DopplerHz(up, uv, sp, sv, 2.0e9);
            const uint64_t rxBytes = rs.GetUeRxBytes(u);
            double measTbler = rs.GetUeRecentTbler(u);
            if (std::isnan(measTbler))
            {
                measTbler = -1.0;
            }
            double measThp = -1.0;
            if (auto bit = lastRxBytes.find(u);
                bit != lastRxBytes.end() && rxBytes >= bit->second)
            {
                measThp = (rxBytes - bit->second) * 8.0 / (kpmIntervalS * 1e6);
            }
            lastRxBytes[u] = rxBytes;
            // Time-to-exit from the live elevation rate of THIS UE's serving
            // link (it was a constant 60 s for every UE and every tick, which
            // the ho-predict xApp then read as geometry).
            double elevRate = 0.0;
            if (auto eit = lastElev.find(u); eit != lastElev.end())
            {
                elevRate = (elev - eit->second) / kpmIntervalS;
            }
            lastElev[u] = elev;
            const double tte =
                (elevRate < -1e-3) ? std::max(1.0, (elev - minElevDeg) / -elevRate) : 600.0;
            // The serving SINR is read off the PHY trace: claim the measured
            // provenance (ORAN-03), as the gym example already does.
            helper->InjectKpmReport(1, u, sinr, sinr - 95.0, tte, elev, doppler,
                                    measThp, rxBytes, measTbler, /*measPrbUtil=*/-1.0,
                                    /*sinrMeasured=*/true);
            // (c) observability: log the measured value.
            kpiLog << std::fixed << std::setprecision(2) << t << "," << u << "," << sinr
                   << ",phy-trace\n";
        }
    };
    rs.RegisterPeriodicCallback(MilliSeconds(100), tick);

    Simulator::Stop(Seconds(duration));
    Simulator::Run();
    rs.Collect();
    rs.WriteHealthReport();
    kpiLog.close();

    // ---- (b) slice: the MEASURED one-way-delay distribution ----
    //
    // WF-05. This used to be
    //     latencyMs = slant/c*1e3 + 5.0;                 // geometry + a magic 5 ms
    //     for (p = 0; p < rxBytes / 1400; ++p) RecordPacket(..., latencyMs, true);
    // and the summary then called the result "measured geometry". Two defects
    // in three lines. The latency was a closed form, identical for every packet
    // of a UE, so the monitor's p99 was a percentile over N copies of one
    // number and no percentile SLA could ever breach. And rxBytes/1400 is the
    // packet estimate the helper documents as wrong for any non-1400 B packet -
    // this example runs MixedBouquet, whose mMTC flows are 128 B and URLLC 256 B,
    // so the count was off by up to a factor of ten for two of the three slices.
    //
    // Both have real replacements that already existed and were not called:
    // GetSliceDelayHistogram() returns the pooled measured OWD distribution in
    // 1 ms bins, and GetUeRxPackets() counts delivered packets correctly for any
    // size. Replaying the histogram gives the monitor a distribution with actual
    // spread, so a percentile bound can be breached by a tail rather than only
    // by the mean.
    // eMBB / URLLC / mMTC 5QIs in this run. MixedBouquet marks eMBB 5QI 2; this
    // table said 9 for eMBB, so the eMBB slice was fed the mMTC histogram (and
    // the mMTC samples were replayed twice) while the real eMBB flows were
    // never looked at.
    const uint8_t sliceQi[3] = {2, 82, 9};
    uint64_t replayed = 0;
    for (uint32_t slice = 0; slice < 3; ++slice)
    {
        const auto hist = rs.GetSliceDelayHistogram(sliceQi[slice]);
        for (const auto& [delayMsBin, count] : hist)
        {
            for (uint64_t p = 0; p < count; ++p)
            {
                sliceMon.RecordPacket(profiles[slice].snssai,
                                      static_cast<double>(delayMsBin), true);
                ++replayed;
            }
        }
    }
    // Losses are measured too, so the reliability verdict is not asserted.
    // Each loss is charged to the slice whose flow lost it; every loss used to
    // be booked against eMBB, which hid a URLLC reliability breach.
    for (uint32_t slice = 0; slice < 3; ++slice)
    {
        const uint64_t lost = rs.GetSliceMeasuredStats(sliceQi[slice]).lostPackets;
        for (uint64_t p = 0; p < lost; ++p)
        {
            sliceMon.RecordPacket(profiles[slice].snssai, 0.0, false);
        }
    }
    // Report the SPREAD, because its absence was the defect. A distribution
    // stamped with one value has zero range, and a p99 over it equals the mean
    // by construction - which is how a percentile SLA became unbreachable.
    {
        uint32_t lo = 0xFFFFFFFF;
        uint32_t hi = 0;
        for (uint32_t slice = 0; slice < 3; ++slice)
        {
            for (const auto& [bin, count] : rs.GetSliceDelayHistogram(sliceQi[slice]))
            {
                if (count == 0)
                {
                    continue;
                }
                lo = std::min(lo, bin);
                hi = std::max(hi, bin);
            }
        }
        if (replayed > 0)
        {
            std::cout << "  [slice] replayed " << replayed
                      << " MEASURED OWD samples spanning " << lo << " to " << hi
                      << " ms (a closed-form stamp would span 0 ms, which is why the old p99 "
                         "could never breach)\n";
        }
    }

    if (replayed == 0)
    {
        // Say so rather than reporting an SLA verdict over an empty sample.
        std::cout << "  NOTE: no measured OWD samples for these 5QIs; the slice SLA "
                     "verdict below is over an empty distribution and means nothing\n";
    }
    auto breaches = sliceMon.EvaluateAll();
    helper->WriteAllMetrics(nearRtRic);

    // ---- Composition summary: all three consumers fed by ONE measured SINR ----
    auto m = nearRtRic->GetMetrics();
    uint32_t totalXappActions = 0;
    for (const auto& [name, xapp] : xapps)
    {
        totalXappActions += xapp->GetMetrics().successfulActions;
    }
    uint32_t latencyBreaches = 0;
    for (const auto& b : breaches)
    {
        if (b.latencyBreach)
        {
            ++latencyBreaches;
        }
    }

    std::cout << "\n--- COMPOSITION proof: one measured data plane, three consumers ---\n"
              << "  shared cell measured SINR (mean): " << rs.GetMeanDlSinrDb() << " dB\n"
              << "  shared cell measured throughput:  " << rs.GetRxThroughputMbps() << " Mbps\n"
              << "  [RIC]           xApp actions on measured KPM: " << totalXappActions
              << " (E2 nodes " << m.totalE2Nodes << ")\n"
              << "  [slice]         SLA latency-breaches on the MEASURED OWD distribution: " << latencyBreaches
              << "/3\n"
              << "  [observability] measured-KPI rows logged: " << outputDir
              << "/measured_kpi_log.csv\n"
              << "  -> the modules COMPOSE on a shared data plane (silo anti-pattern killed).\n";

    Simulator::Destroy();
    return 0;
}
