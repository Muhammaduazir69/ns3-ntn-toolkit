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
 *
 * Wave 7B, one flow through the toolkit (space-air-ground):
 *  - ground terminals next to the swarm (--groundUes, default 2) are TR 38.811
 *    UE-class terminals (ntn-cho NtnTr38811MobilityModel, the IoT-field class
 *    mix: fixed IoT, static handheld, vehicular) placed around the site and
 *    projected into the cell's local ENU frame (NtnEnuProjectionMobilityModel);
 *  - the gNB site reaches its core over a bent-pipe LEO hop of a Walker shell on
 *    Vallado SGP4 (ntn-constellation; sagin-leo-backhaul.h): live one-way delay
 *    from the exact SGP4 state every 100 ms, satellite switches, outages, and
 *    loss from two NtnSatLinkErrorModel Ka legs. --leoBackhaul=0 restores the
 *    terrestrial backhaul of the earlier runs;
 *  - traffic is the ntn-traffic standards bouquet with the WS2 KPM series
 *    (TS 28.552 names) from the FlowMonitor-based NtnOranAiFlowMonitor.
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
#include "ns3/ntn-tr38811-mobility-model.h"
#include "ns3/system-path.h"

#include "sagin-leo-backhaul.h"

#include <cstdio>
#include <iostream>
#include <memory>
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
    double fcGHz = 2.185; // TS 38.101-5 n256 DOWNLINK block (2170-2200 MHz).
    double gnbTxDbm = 0.0; // ground mast to UAVs at short range -> low Tx
    std::string outputDir = "sagin-uav-swarm-output";
    std::string radio = "nr"; // radio backend: "nr" (5G-LENA FR1) | "mmwave" (FR2)

    CommandLine cmd(__FILE__);
    cmd.AddValue("simTime", "Simulation duration (s)", simTimeSec);
    cmd.AddValue("fcGHz", "Carrier frequency (GHz)", fcGHz);
    cmd.AddValue("gnbTxDbm", "Ground gNB Tx power (dBm)", gnbTxDbm);
    cmd.AddValue("outputDir", "Output directory", outputDir);
    cmd.AddValue("radio", "Radio backend: nr (FR1) or mmwave", radio);
    uint32_t groundUes = 2;
    bool leoBackhaul = true;
    SaginLeoBackhaulConfig bh; // rural site, gateway about 250 km away
    double bhMbps = 100.0;
    cmd.AddValue("groundUes", "TR 38.811 ground terminals served next to the swarm", groundUes);
    cmd.AddValue("leoBackhaul", "gNB backhaul over a bent-pipe LEO hop (SGP4 Walker shell)", leoBackhaul);
    cmd.AddValue("siteLat", "gNB site latitude (deg)", bh.siteLatDeg);
    cmd.AddValue("siteLon", "gNB site longitude (deg)", bh.siteLonDeg);
    cmd.AddValue("gwLat", "LEO gateway latitude (deg)", bh.gwLatDeg);
    cmd.AddValue("gwLon", "LEO gateway longitude (deg)", bh.gwLonDeg);
    cmd.AddValue("bhMaskDeg", "backhaul elevation mask at site and gateway (deg)", bh.siteMaskDeg);
    cmd.AddValue("bhMbps", "backhaul rate (Mb/s), sets the Ka noise bandwidth", bhMbps);
    cmd.Parse(argc, argv);
    bh.gwMaskDeg = bh.siteMaskDeg;
    bh.rateBps = bhMbps * 1e6;
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
    ueNodes.Create(g_numUavs + groundUes);
    for (uint32_t i = 0; i < g_numUavs; ++i)
    {
        ueNodes.Get(i)->AggregateObject(uavs[i]);
    }
    // TR 38.811 ground terminals within ~1 km of the mast, in the cell's ENU frame
    // (UE height 1.5 m: the frame origin sits 1.5 m below the terrain).
    if (groundUes > 0)
    {
        auto engine = std::make_shared<NtnRealisticMobilityHelper>(7);
        const double dLat = 0.009, dLon = 0.009 / std::cos(bh.siteLatDeg * M_PI / 180.0);
        auto ues = engine->GenerateUes(groundUes, NtnMobilityScenarios::IotField(),
                                       bh.siteLatDeg - dLat, bh.siteLatDeg + dLat,
                                       bh.siteLonDeg - dLon, bh.siteLonDeg + dLon);
        for (uint32_t j = 0; j < groundUes; ++j)
        {
            Ptr<NtnTr38811MobilityModel> src = CreateObject<NtnTr38811MobilityModel>();
            src->SetUeState(ues[j], engine);
            Ptr<NtnEnuProjectionMobilityModel> enu = CreateObject<NtnEnuProjectionMobilityModel>();
            enu->SetSource(src);
            enu->SetReference(bh.siteLatDeg, bh.siteLonDeg, -1.5);
            ueNodes.Get(g_numUavs + j)->AggregateObject(enu);
            std::printf("# ground UE %u: TR 38.811 class %s at (%.4f, %.4f)\n", j,
                        ues[j].className.c_str(), ues[j].lat, ues[j].lon);
        }
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
    std::unique_ptr<SaginLeoBackhaul> leo;
    if (leoBackhaul)
    {
        leo = std::make_unique<SaginLeoBackhaul>(bh);
        leo->Configure(rs);
    }
    rs.Build(gnbNodes, ueNodes);
    if (leo)
    {
        SystemPath::MakeDirectories(outputDir);
        leo->Attach(rs, outputDir + "/leo_backhaul.csv");
    }
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
    if (leo)
    {
        leo->Stop();
    }
    rs.Collect();
    rs.WriteHealthReport();
    if (leo)
    {
        leo->PrintSummary("sagin-uav-swarm");
    }
    std::printf("# measured app one-way delay %.3f ms, jitter %.3f ms, app loss %.5f; the cell "
                "means below include %u TR 38.811 ground UEs\n",
                rs.GetMeanDelayMs(), rs.GetMeanJitterMs(), rs.GetAppLossRatio(), groundUes);

    std::printf("# === summary ===  %u UAVs  measured SINR=%.2f dB  measured TBLER=%.4f  "
                "measured throughput=%.3f Mbps\n",
                g_numUavs, rs.GetMeanDlSinrDb(), rs.GetMeanDlTbler(), rs.GetRxThroughputMbps());
    Simulator::Destroy();
    return 0;
}
