/*
 * SPDX-License-Identifier: GPL-2.0-only
 * Copyright (c) 2026 Muhammad Uzair (ns3-ntn-toolkit, W6)
 *
 * Demo: three slices (eMBB / URLLC / mMTC) coexisting on a single LEO,
 * with URLLC enforcing GEO mode-skip routing. Drives the orchestrator at
 * 1 Hz over a 1 h scenario and pipes per-slice KPIs through the isolation
 * monitor; emits a CSV plus a stdout summary.
 *
 * Demonstrates the W2 (RRC) → W4 (RL env API) → W6 (slice orchestrator)
 * → W3 (observability) integration: same `(observation, action)` shape as
 * `ns3_ai_ntn.envs.SliceEnv`, so an RL policy trained against W4's env can
 * drive the C++ orchestrator directly.
 */
#include "ns3/command-line.h"
#include "ns3/double.h"
#include "ns3/ntn-slice-helper.h"
#include "ns3/ntn-slice-selector.h"
#include "ns3/ntn-slice-types.h"
#include "ns3/random-variable-stream.h"
#include "ns3/simulator.h"
#include "ns3/slice-isolation-monitor.h"
#include "ns3/slice-orchestrator-xapp.h"
#include "ns3/ntn-realistic-traffic-helper.h"

#include <fstream>
#include <iomanip>
#include <iostream>

using namespace ns3;
using namespace ns3::ntnslice;

namespace
{

double
SimulatedLatencyMs(SliceSst sst, double satisfaction, bool servedByGeo)
{
    // Bare-metal latency model: LEO ≈ 30 ms RTT, GEO ≈ 500 ms RTT;
    // queuing penalty grows as satisfaction drops.
    double base = servedByGeo ? 280.0 : 18.0;  // one-way ms
    double queueing = (satisfaction < 0.99) ? (1.0 - satisfaction) * 60.0 : 0.0;
    if (sst == SliceSst::Urllc)
    {
        // URLLC bumps to 1 ms baseline (low-latency processing budget) when LEO.
        base = servedByGeo ? 250.0 : 1.5;
    }
    return base + queueing;
}

bool
SimulatedDelivered(SliceSst sst, double satisfaction, Ptr<UniformRandomVariable> u)
{
    double pLoss = (1.0 - satisfaction) * 0.05;
    if (sst == SliceSst::Urllc)
        pLoss *= 0.1; // URLLC ARQ + HARQ
    return u->GetValue() > pLoss;
}

} // namespace

int
main(int argc, char* argv[])
{
    double simTimeSec = 3600.0;
    std::string outputDir = ".";
    bool routeUrllcViaGeo = false;
    std::string csvPath = "ntn-three-slice.csv";
    uint32_t totalPrb = 273;

    CommandLine cmd(__FILE__);
    cmd.AddValue("simTime", "Simulation duration (s)", simTimeSec);
    cmd.AddValue("urllcViaGeo", "Force URLLC over GEO (gates the GEO-skip test)", routeUrllcViaGeo);
    cmd.AddValue("totalPrb", "Total PRB budget", totalPrb);
    cmd.AddValue("csv", "Per-slice KPI CSV path", csvPath);
    cmd.AddValue("outputDir", "Output directory for sim_health.csv", outputDir);
    cmd.Parse(argc, argv);

    auto stack = NtnSliceHelper::ThreeSliceDefault();
    stack.orchestrator->SetTotalPrb(totalPrb);

    Ptr<UniformRandomVariable> u = CreateObject<UniformRandomVariable>();
    Ptr<NormalRandomVariable> demandJitter = CreateObject<NormalRandomVariable>();
    demandJitter->SetAttribute("Mean", DoubleValue(0.0));
    demandJitter->SetAttribute("Variance", DoubleValue(1.0));

    std::ofstream out(csvPath);
    out << "time_s,slice,demand_mbps,served_mbps,prb,satisfaction,p99_latency_ms,latency_breach,reliability_breach\n";

    int totalLatencyBreach = 0;
    int totalReliabilityBreach = 0;
    double maxUrllcP99 = 0.0;

    for (double t = 0; t < simTimeSec; t += 1.0)
    {
        Simulator::Schedule(Seconds(t), [&]() {
            // Synthetic demand baselines per TS 22.261 reference KPIs
            double embbDemand = 400.0 + 200.0 * demandJitter->GetValue();
            double urllcDemand = 80.0 + 25.0 * demandJitter->GetValue();
            double mmtcDemand = 20.0 + 5.0 * demandJitter->GetValue();
            embbDemand = std::max(50.0, embbDemand);
            urllcDemand = std::max(15.0, urllcDemand);
            mmtcDemand = std::max(2.0, mmtcDemand);

            stack.orchestrator->SetDemand(stack.embb.snssai, embbDemand);
            stack.orchestrator->SetDemand(stack.urllc.snssai, urllcDemand);
            stack.orchestrator->SetDemand(stack.mmtc.snssai, mmtcDemand);

            auto ticks = stack.orchestrator->Step();
            for (auto& tk : ticks)
            {
                bool servedByGeo = false;
                if (tk.snssai.sst == SliceSst::Urllc)
                {
                    servedByGeo = routeUrllcViaGeo;  // GEO-skip when false
                }
                else if (tk.snssai.sst == SliceSst::Embb)
                {
                    servedByGeo = (u->GetValue() < 0.3); // some eMBB on GEO
                }

                // Stream a couple of packets per slice into the monitor.
                for (int i = 0; i < 5; ++i)
                {
                    double latency = SimulatedLatencyMs(tk.snssai.sst,
                                                        tk.satisfactionRatio,
                                                        servedByGeo);
                    bool delivered = SimulatedDelivered(tk.snssai.sst,
                                                        tk.satisfactionRatio, u);
                    stack.monitor->RecordPacket(tk.snssai, latency, delivered);
                }
            }

            // Evaluate the SLA window every second.
            auto breaches = stack.monitor->EvaluateAll();
            for (std::size_t i = 0; i < ticks.size(); ++i)
            {
                auto& tk = ticks[i];
                BreachEvent br{};
                for (auto& candidate : breaches)
                {
                    if (candidate.snssai == tk.snssai)
                    {
                        br = candidate;
                        break;
                    }
                }
                out << std::fixed << std::setprecision(3) << t << ","
                    << SstName(tk.snssai.sst) << ","
                    << tk.demandMbps << "," << tk.servedMbps << ","
                    << tk.prbAllocated << "," << tk.satisfactionRatio << ","
                    << br.observedLatencyP99Ms << ","
                    << (br.latencyBreach ? 1 : 0) << ","
                    << (br.reliabilityBreach ? 1 : 0) << "\n";
                if (br.latencyBreach) totalLatencyBreach += 1;
                if (br.reliabilityBreach) totalReliabilityBreach += 1;
                if (tk.snssai.sst == SliceSst::Urllc &&
                    br.observedLatencyP99Ms > maxUrllcP99)
                {
                    maxUrllcP99 = br.observedLatencyP99Ms;
                }
            }
        });
    }
    // ==== v2 realistic traffic plane (auto-injected) =====================
    NtnRealisticTrafficHelper _ntn_traffic;
    _ntn_traffic.SetSimTime(Seconds(simTimeSec));
    _ntn_traffic.SetOutputDir(outputDir);
    _ntn_traffic.SetRunTag("ntn-slice");
    _ntn_traffic.SetProfile(NtnRealisticTrafficHelper::TrafficProfile::MixedBouquet);
    _ntn_traffic.InstallUes(8);
    _ntn_traffic.Wire();

    

    Simulator::Stop(Seconds(simTimeSec));
    Simulator::Run();
    _ntn_traffic.WriteHealthReport();
    Simulator::Destroy();

    std::cout << "ntn-three-slice-leo-geo done.\n"
              << "  sim time            : " << simTimeSec << " s\n"
              << "  URLLC via GEO       : " << (routeUrllcViaGeo ? "yes (test)" : "no (mode-skip)") << "\n"
              << "  total PRB           : " << totalPrb << "\n"
              << "  URLLC max p99       : " << maxUrllcP99 << " ms\n"
              << "  latency breaches    : " << totalLatencyBreach << "\n"
              << "  reliability breaches: " << totalReliabilityBreach << "\n"
              << "  csv                 : " << csvPath << "\n";
    return 0;
}
