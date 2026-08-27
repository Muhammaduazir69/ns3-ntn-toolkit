/*
 * SPDX-License-Identifier: GPL-2.0-only
 * Copyright (c) 2026 Muhammad Uzair (ns3-ntn-toolkit, W5)
 *
 * Demo: 8-UAV swarm — 2 random-waypoint, 4 patrol, 2 search-pattern — served by
 * a ground gNB over a REAL mmwave NR cell (NtnRealStackHelper) carrying the
 * 3GPP TR 36.777 RMa-AV air-to-ground channel (SaginA2gPropagationLossModel).
 * Each UAV is a UE; the per-UAV SINR/TBLER/throughput are MEASURED off the
 * mmwave PHY trace as the swarm manoeuvres — no closed-form path-loss CSV, no
 * P2P star. The UAV mobility (waypoint/patrol/search) is the real value kept.
 */
#include "ns3/box.h"
#include "ns3/command-line.h"
#include "ns3/constant-position-mobility-model.h"
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/ntn-real-stack-helper.h"
#include "ns3/sagin-a2g-propagation-loss-model.h"
#include "ns3/sagin-helper.h"
#include "ns3/uav-mobility-models.h"

#include <cstdio>
#include <iostream>
#include <vector>

using namespace ns3;

namespace
{
NtnRealStackHelper* g_rs = nullptr;
uint32_t g_numUavs = 0;
double g_simTime = 30.0;

void
SwarmProbe()
{
    if (Simulator::Now().GetSeconds() >= g_simTime)
    {
        return;
    }
    std::printf("  %6.1f  per-UAV measSINR:", Simulator::Now().GetSeconds());
    for (uint32_t i = 0; i < g_numUavs; ++i)
    {
        const double s = g_rs->GetUeRecentSinrDb(i);
        std::printf(" %5.1f", std::isnan(s) ? 0.0 : s);
    }
    std::printf(" dB\n");
    Simulator::Schedule(Seconds(2.0), &SwarmProbe);
}
} // namespace

int
main(int argc, char* argv[])
{
    double simTimeSec = 30.0;
    double fcGHz = 2.0;
    double gnbTxDbm = 0.0; // ground mast to UAVs at short range -> low Tx
    std::string outputDir = "sagin-uav-swarm-output";
    std::string radio = "nr"; // radio backend: "nr" (5G-LENA FR1) | "mmwave" (FR2)

    CommandLine cmd(__FILE__);
    cmd.AddValue("simTime", "Simulation duration (s)", simTimeSec);
    cmd.AddValue("fcGHz", "Carrier frequency (GHz)", fcGHz);
    cmd.AddValue("gnbTxDbm", "Ground gNB Tx power (dBm)", gnbTxDbm);
    cmd.AddValue("outputDir", "Output directory", outputDir);
    cmd.AddValue("radio", "Radio backend: nr (FR1) or mmwave", radio);
    cmd.Parse(argc, argv);
    g_simTime = simTimeSec;

    SaginHelper helper;

    // 8-UAV swarm (real mobility): 2 random-waypoint, 4 patrol, 2 search.
    std::vector<Ptr<MobilityModel>> uavs;
    for (int i = 0; i < 2; ++i)
    {
        Box box(-1500, 1500, -1500, 1500, 50, 250);
        uavs.push_back(helper.CreateUavWaypoint(box, 22.0));
    }
    for (int i = 0; i < 4; ++i)
    {
        Vector a{-1500.0 + 750.0 * i, 0.0, 100.0};
        Vector b{1500.0 - 750.0 * i, 0.0, 100.0};
        uavs.push_back(helper.CreateUavPatrol(a, b, 18.0));
    }
    uavs.push_back(helper.CreateUavSearch(Vector{-1000, -500, 0}, 2000.0, 100.0, 8, 120.0));
    uavs.push_back(helper.CreateUavSearch(Vector{500, -500, 0}, 1500.0, 80.0, 6, 80.0));
    g_numUavs = static_cast<uint32_t>(uavs.size());

    // Each UAV is a UE; the ground gNB is the "satellite" endpoint.
    NodeContainer ueNodes;
    ueNodes.Create(g_numUavs);
    for (uint32_t i = 0; i < g_numUavs; ++i)
    {
        ueNodes.Get(i)->AggregateObject(uavs[i]);
    }
    NodeContainer gnbNodes;
    gnbNodes.Create(1);
    Ptr<ConstantPositionMobilityModel> gnb = CreateObject<ConstantPositionMobilityModel>();
    gnb->SetPosition(Vector(0.0, 0.0, 35.0)); // RMa-AV ground mast
    gnbNodes.Get(0)->AggregateObject(gnb);

    NtnRealStackHelper rs;
    rs.SetSimTime(Seconds(simTimeSec));
    rs.SetOutputDir(outputDir);
    rs.SetRunTag("sagin-uav-swarm");
    rs.SetCarrierFrequencyHz(fcGHz * 1e9);
    // NT-02: declared as CONDUCTED power at the array input. This carrier has
    // no TR 38.821 Set-1 reference in the toolkit, so the EIRP health gate
    // reports "not asserted" rather than certifying an uncalibrated budget.
    rs.SetSatConductedPowerDbm(gnbTxDbm); // short-range A2G gNB Tx (not a LEO link) — kept as-is
    rs.SetRadioBackend(radio == "mmwave" ? NtnRealStackHelper::RadioBackend::Mmwave
                                         : NtnRealStackHelper::RadioBackend::Nr);
    if (radio != "mmwave")
    {
        rs.SetNumerology(1); // FR1 30 kHz SCS
    }
    rs.Build(gnbNodes, ueNodes);
    Ptr<SaginA2gPropagationLossModel> a2g = CreateObject<SaginA2gPropagationLossModel>();
    a2g->SetFrequencyGHz(fcGHz);
    a2g->SetScenario(A2gScenario::RMa_AV);
    a2g->SetLink(A2gLink::LOS);
    rs.AddExtraPropagationLoss(a2g);
    rs.InstallTraffic(NtnRealStackHelper::TrafficProfile::MixedBouquet,
                      Seconds(1.0), Seconds(simTimeSec - 0.5));
    rs.EnableAiFlowMonitor("sagin-uav-swarm"); // WS2 KPM series (TS 28.552 names)
    g_rs = &rs;

    std::printf("# sagin-uav-swarm (%u UAVs on a real mmwave NR cell, TR 36.777 RMa-AV A2G)\n",
                g_numUavs);
    Simulator::Schedule(Seconds(2.0), &SwarmProbe);
    Simulator::Stop(Seconds(simTimeSec));
    Simulator::Run();
    rs.Collect();
    rs.WriteHealthReport();

    std::printf("# === summary ===  %u UAVs  measured SINR=%.2f dB  measured TBLER=%.4f  "
                "measured throughput=%.3f Mbps\n",
                g_numUavs, rs.GetMeanDlSinrDb(), rs.GetMeanDlTbler(), rs.GetRxThroughputMbps());
    Simulator::Destroy();
    return 0;
}
