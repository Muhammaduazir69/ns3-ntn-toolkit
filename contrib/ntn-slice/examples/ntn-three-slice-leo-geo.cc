/*
 * SPDX-License-Identifier: GPL-2.0-only
 * Copyright (c) 2026 Muhammad Uzair (ns3-ntn-toolkit, W6)
 *
 * ntn-three-slice-leo-geo — three slices (eMBB / URLLC / mMTC) coexisting on a
 * LEO+GEO NTN, with URLLC enforcing GEO mode-skip routing. The slice
 * orchestrator runs at 1 Hz; its PRB allocations and satisfaction ratios are
 * its real control outputs. The radio truth is MEASURED: a real mmwave NR cell
 * (NtnRealStackHelper) on a real SGP4 LEO pass with TR 38.811 UEs provides the
 * measured TBLER that gates packet delivery, and per-slice latency is the REAL
 * geometry — LEO slant/c for LEO-served slices, the real ~36 000 km GEO slant/c
 * (~120 ms one-way) for GEO-served ones — so the URLLC GEO-skip decision is
 * validated against real physics, not a hardcoded 250 ms constant.
 *
 * Provenance note: the per-slice KPI sample stream fed to the isolation
 * monitor is synthesized at 5 samples/s/slice FROM the measured cell TBLER
 * plus the orchestrator's own PRB-starvation ratio (so the allocation
 * decision acts on the per-slice KPIs); packet-level slice CONTENTION on one
 * shared real cell is covered by ntn-slice-isolation-traffic and
 * ntn-slice-real-stack.
 *
 * Demonstrates W2 (RRC) -> W4 (RL env API) -> W6 (orchestrator) -> W3
 * (observability) integration; same (observation, action) shape as SliceEnv.
 */
#include "ns3/command-line.h"
#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/ntn-real-stack-helper.h"
#include "ns3/ntn-tr38811-mobility-model.h"
#include "ns3/sgp4-mobility-model.h"
#include "ns3/walker-constellation.h"

#include "ns3/ntn-slice-helper.h"
#include "ns3/ntn-slice-types.h"
#include "ns3/slice-isolation-monitor.h"
#include "ns3/slice-orchestrator-xapp.h"

#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>

using namespace ns3;
using namespace ns3::ntnslice;

int
main(int argc, char* argv[])
{
    double simTimeSec = 30.0;
    uint32_t numUes = 3;
    bool routeUrllcViaGeo = false;
    double satEirpDbm = 55.0;
    double backhaulMs = 5.0;
    std::string csvPath = "ntn-three-slice.csv";
    uint32_t totalPrb = 273;
    std::string outputDir = "ntn-three-slice-output";

    CommandLine cmd(__FILE__);
    cmd.AddValue("simTime", "Simulation duration (s)", simTimeSec);
    cmd.AddValue("numUes", "UEs on the measured LEO cell", numUes);
    cmd.AddValue("urllcViaGeo", "Force URLLC over GEO (gates the GEO-skip test)", routeUrllcViaGeo);
    cmd.AddValue("totalPrb", "Total PRB budget", totalPrb);
    cmd.AddValue("satEirpDbm", "LEO EIRP / gNB Tx power (dBm)", satEirpDbm);
    cmd.AddValue("csv", "Per-slice KPI CSV path", csvPath);
    cmd.AddValue("outputDir", "Output directory", outputDir);
    cmd.Parse(argc, argv);

    // ---- Real LEO cell: SGP4 Walker sat + TR 38.811 UEs (measured radio) ----
    ns3::ntncon::WalkerConfig wcfg;
    wcfg.num_planes = 1;
    wcfg.total_sats = 80;
    wcfg.altitude_km = 550.0;
    wcfg.inclination_deg = 53.0;
    wcfg.epoch_unix_s = 1735689600.0;
    const auto wElements = ns3::ntncon::WalkerConstellation::BuildDelta(wcfg);
    Ptr<ns3::ntncon::Sgp4MobilityModel> leoSat =
        CreateObject<ns3::ntncon::Sgp4MobilityModel>();
    leoSat->SetElements(wElements[0]);
    NodeContainer satNodes;
    satNodes.Create(1);
    satNodes.Get(0)->AggregateObject(leoSat);
    NodeContainer ueNodes;
    ueNodes.Create(numUes);
    double subLat, subLon, subAlt;
    leoSat->GetGeodetic(subLat, subLon, subAlt);
    NtnTr38811MobilityHelper ueMobility(1);
    auto mobProfile = NtnMobilityScenarios::MixedContinental();
    auto ueModels = ueMobility.Install(ueNodes, mobProfile, subLat - 0.03, subLat + 0.03,
                                       subLon - 0.03, subLon + 0.03);
    // The REAL GEO position serving this region (equatorial, same longitude).
    const Vector geoEcef = ntngeo::GeodeticToEcef(0.0, subLon, 35786000.0);

    NtnRealStackHelper rs;
    rs.SetSimTime(Seconds(simTimeSec));
    rs.SetOutputDir(outputDir);
    rs.SetRunTag("ntn-three-slice-leo-geo");
    rs.SetSatEirpDbm(satEirpDbm);
    rs.SetBackhaulDelay(MilliSeconds(backhaulMs));
    rs.Build(satNodes, ueNodes);
    rs.InstallTraffic(NtnRealStackHelper::TrafficProfile::MixedBouquet,
                      Seconds(1.0), Seconds(simTimeSec - 0.5));
    rs.EnableAiFlowMonitor("ntn-three-slice-leo-geo"); // WS2 KPM series (TS 28.552 names)

    auto stack = NtnSliceHelper::ThreeSliceDefault();
    stack.orchestrator->SetTotalPrb(totalPrb);

    Ptr<UniformRandomVariable> u = CreateObject<UniformRandomVariable>();
    Ptr<NormalRandomVariable> demandJitter = CreateObject<NormalRandomVariable>();
    demandJitter->SetAttribute("Mean", DoubleValue(0.0));
    demandJitter->SetAttribute("Variance", DoubleValue(1.0));

    std::ofstream out(csvPath);
    out << "time_s,slice,demand_mbps,served_mbps,prb,satisfaction,p99_latency_ms,"
           "latency_breach,reliability_breach\n";

    int totalLatencyBreach = 0;
    int totalReliabilityBreach = 0;
    double maxUrllcP99 = 0.0;

    rs.RegisterPeriodicCallback(Seconds(1.0), [&](Time now) {
        const double t = now.GetSeconds();
        // Offered load per TS 22.261 reference KPIs (operator demand input).
        double embbDemand = std::max(50.0, 400.0 + 200.0 * demandJitter->GetValue());
        double urllcDemand = std::max(15.0, 80.0 + 25.0 * demandJitter->GetValue());
        double mmtcDemand = std::max(2.0, 20.0 + 5.0 * demandJitter->GetValue());
        stack.orchestrator->SetDemand(stack.embb.snssai, embbDemand);
        stack.orchestrator->SetDemand(stack.urllc.snssai, urllcDemand);
        stack.orchestrator->SetDemand(stack.mmtc.snssai, mmtcDemand);

        // MEASURED radio truth from the real LEO cell.
        const double measTbler = rs.GetUeRecentTbler(0);
        const double tbler = std::isnan(measTbler) ? 0.0 : measTbler;
        const Vector uePos = ueModels[0]->GetPosition();
        const double leoLatMs =
            ntngeo::SlantRangeM(uePos, leoSat->GetPosition()) / 299792458.0 * 1e3 + backhaulMs;
        const double geoLatMs =
            ntngeo::SlantRangeM(uePos, geoEcef) / 299792458.0 * 1e3 + backhaulMs;

        auto ticks = stack.orchestrator->Step();
        for (auto& tk : ticks)
        {
            bool servedByGeo = false;
            if (tk.snssai.sst == SliceSst::Urllc)
            {
                servedByGeo = routeUrllcViaGeo; // GEO-skip when false
            }
            else if (tk.snssai.sst == SliceSst::Embb)
            {
                servedByGeo = (u->GetValue() < 0.3); // some eMBB offloaded to GEO
            }
            // Real path latency + a queueing term from the orchestrator's own
            // PRB starvation (1 - satisfaction).
            const double queueMs =
                (tk.satisfactionRatio < 0.99) ? (1.0 - tk.satisfactionRatio) * 60.0 : 0.0;
            const double latency = (servedByGeo ? geoLatMs : leoLatMs) + queueMs;
            // Delivery gated by the MEASURED cell TBLER plus PRB starvation.
            const double unserved = std::max(0.0, 1.0 - tk.satisfactionRatio) * 0.05;
            for (int i = 0; i < 5; ++i)
            {
                const bool delivered = (u->GetValue() > tbler) && (u->GetValue() > unserved);
                stack.monitor->RecordPacket(tk.snssai, latency, delivered);
            }
        }

        auto breaches = stack.monitor->EvaluateAll();
        for (auto& tk : ticks)
        {
            BreachEvent br{};
            for (auto& candidate : breaches)
            {
                if (candidate.snssai == tk.snssai)
                {
                    br = candidate;
                    break;
                }
            }
            out << std::fixed << std::setprecision(3) << t << "," << SstName(tk.snssai.sst)
                << "," << tk.demandMbps << "," << tk.servedMbps << "," << tk.prbAllocated << ","
                << tk.satisfactionRatio << "," << br.observedLatencyP99Ms << ","
                << (br.latencyBreach ? 1 : 0) << "," << (br.reliabilityBreach ? 1 : 0) << "\n";
            if (br.latencyBreach) totalLatencyBreach += 1;
            if (br.reliabilityBreach) totalReliabilityBreach += 1;
            if (tk.snssai.sst == SliceSst::Urllc && br.observedLatencyP99Ms > maxUrllcP99)
            {
                maxUrllcP99 = br.observedLatencyP99Ms;
            }
        }
    });

    Simulator::Stop(Seconds(simTimeSec));
    Simulator::Run();
    rs.Collect();
    rs.WriteHealthReport();
    out.close();

    std::cout << "ntn-three-slice-leo-geo complete (orchestrator on MEASURED radio).\n"
              << "  measured LEO SINR  : " << rs.GetMeanDlSinrDb() << " dB\n"
              << "  measured LEO TBLER : " << rs.GetMeanDlTbler() << "\n"
              << "  urllcViaGeo=" << (routeUrllcViaGeo ? "true" : "false")
              << "  max URLLC p99 latency: " << maxUrllcP99 << " ms (real geometry)\n"
              << "  latency breaches=" << totalLatencyBreach
              << "  reliability breaches=" << totalReliabilityBreach << "\n"
              << "  csv: " << csvPath << "\n";
    Simulator::Destroy();
    return 0;
}
