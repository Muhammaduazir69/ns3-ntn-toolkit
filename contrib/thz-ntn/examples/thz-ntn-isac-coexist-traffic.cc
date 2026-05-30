/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only
//
// thz-ntn-isac-coexist-traffic — joint communication + sensing coexistence.
// A ThzNtnIsacScheduler partitions the THz resource grid between comm and
// sensing per the active ThzNtnIsac mode; the comm share drives the capacity
// of a PointToPoint link carrying REAL UDP traffic. As the ISAC mode is
// stepped over sim time (COMM_ONLY → COMM_CENTRIC → JOINT → SENSING_CENTRIC
// → SENSING_ONLY), the comm link capacity — and therefore the delivered
// goodput — drops because more sub-bands are reallocated to radar sensing.
//
// The comm sub-band count comes from ThzNtnIsacScheduler::Schedule() over a
// real sub-band grid, so the throughput ceiling is computed by the module,
// not hardcoded.
//
// Quick test:  --simSeconds=100 --offeredMbps=200 --numSubBands=20
#include "ns3/applications-module.h"
#include "ns3/command-line.h"
#include "ns3/core-module.h"
#include "ns3/flow-monitor-helper.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/point-to-point-helper.h"
#include "ns3/point-to-point-net-device.h"

#include "ns3/thz-ntn-isac-scheduler.h"
#include "ns3/thz-ntn-isac.h"
#include "ns3/thz-ntn-mac-scheduler.h"

#include <cstdio>
#include <vector>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("ThzNtnIsacCoexistTraffic");

namespace
{
Ptr<ThzNtnIsac> g_isac;
Ptr<ThzNtnIsacScheduler> g_sched;
Ptr<PointToPointNetDevice> g_devTx;
Ptr<PointToPointNetDevice> g_devRx;
Ptr<PacketSink> g_sink;
std::vector<ThzNtnSubBand> g_subBands;
std::vector<ThzNtnUeContext> g_ues;
double g_perSubBandMbps = 0.0;
uint64_t g_lastRx = 0;
const char* g_modeName = "?";

const char*
ModeName(IsacMode m)
{
    switch (m)
    {
    case COMMUNICATION_ONLY: return "COMM_ONLY";
    case COMMUNICATION_CENTRIC: return "COMM_CENTRIC";
    case JOINT_ISAC: return "JOINT_ISAC";
    case SENSING_CENTRIC: return "SENSING_CENTRIC";
    case SENSING_ONLY: return "SENSING_ONLY";
    }
    return "?";
}

void
SetMode(IsacMode m)
{
    g_isac->SetIsacMode(m);
    g_modeName = ModeName(m);
    const auto decision = g_sched->Schedule(g_ues, g_subBands);
    const double commMbps = decision.commSubBandIndices.size() * g_perSubBandMbps;
    // Throttle the comm link capacity to the comm sub-band allocation.
    const uint64_t bps = static_cast<uint64_t>(std::max(0.1, commMbps) * 1e6);
    g_devTx->SetAttribute("DataRate", DataRateValue(DataRate(bps)));
    g_devRx->SetAttribute("DataRate", DataRateValue(DataRate(bps)));
}

void
IsacProbe()
{
    const uint64_t tot = g_sink ? g_sink->GetTotalRx() : 0;
    const double mbps = (tot - g_lastRx) * 8.0 / 1e6;
    g_lastRx = tot;
    const auto d = g_sched->Schedule(g_ues, g_subBands);
    std::printf("  %6.1f  %-16s  %3zu/%-3u  %8.1f  %9.3f\n",
                Simulator::Now().GetSeconds(), g_modeName,
                d.commSubBandIndices.size(), d.numSubBandsTotal,
                d.commSubBandIndices.size() * g_perSubBandMbps, mbps);
    Simulator::Schedule(Seconds(1.0), &IsacProbe);
}
} // namespace

int
main(int argc, char* argv[])
{
    double simSeconds = 100.0;
    double offeredMbps = 200.0;
    uint32_t numSubBands = 20;
    uint32_t numUes = 4;
    uint32_t packetBytes = 1200;

    CommandLine cmd(__FILE__);
    cmd.AddValue("simSeconds", "Simulation duration (s)", simSeconds);
    cmd.AddValue("offeredMbps", "Offered comm load (Mbps)", offeredMbps);
    cmd.AddValue("numSubBands", "Number of THz sub-bands in the grid", numSubBands);
    cmd.AddValue("numUes", "Number of comm UEs", numUes);
    cmd.AddValue("packetBytes", "UDP payload size (bytes)", packetBytes);
    cmd.Parse(argc, argv);

    // Total comm capacity if ALL sub-bands were comm = offeredMbps; so each
    // sub-band carries offeredMbps/numSubBands. As sensing takes sub-bands,
    // the comm ceiling drops proportionally.
    g_perSubBandMbps = offeredMbps / numSubBands;

    g_isac = CreateObject<ThzNtnIsac>();
    Ptr<ThzNtnMacScheduler> comm = CreateObject<ThzNtnMacScheduler>();
    g_sched = CreateObject<ThzNtnIsacScheduler>();
    g_sched->SetIsac(g_isac);
    g_sched->SetCommScheduler(comm);

    // Build the sub-band grid + UE contexts the scheduler partitions.
    for (uint32_t i = 0; i < numSubBands; ++i)
    {
        ThzNtnSubBand sb;
        sb.index = i;
        sb.centerFreqHz = 140e9 + i * 1e9;
        sb.bandwidthHz = 1e9;
        sb.isAvailable = true;
        sb.absorptionLoss_dB = 0.0;
        sb.assignedUeId = 0;
        sb.carrierIndex = i;
        g_subBands.push_back(sb);
    }
    for (uint32_t i = 0; i < numUes; ++i)
    {
        ThzNtnUeContext u;
        u.ueId = 100 + i;
        u.rnti = 1 + static_cast<uint16_t>(i);
        u.qosClass = ThzNtnQosClass::EMBB;
        u.sinr_dB = 15.0;
        u.throughput_Mbps = 100.0;
        u.dopplerHz = 0.0;
        u.mcsIndex = 10;
        u.bufferSize_bytes = 200000;
        g_ues.push_back(u);
    }

    NodeContainer nodes;
    nodes.Create(2);
    PointToPointHelper p2p;
    p2p.SetDeviceAttribute("DataRate", DataRateValue(DataRate(uint64_t(offeredMbps * 1e6))));
    p2p.SetChannelAttribute("Delay", TimeValue(MilliSeconds(2)));
    NetDeviceContainer devices = p2p.Install(nodes);
    g_devTx = DynamicCast<PointToPointNetDevice>(devices.Get(1));
    g_devRx = DynamicCast<PointToPointNetDevice>(devices.Get(0));

    InternetStackHelper internet;
    internet.Install(nodes);
    Ipv4AddressHelper ipv4;
    ipv4.SetBase("10.13.1.0", "255.255.255.0");
    Ipv4InterfaceContainer ifaces = ipv4.Assign(devices);

    const uint16_t port = 9800;
    PacketSinkHelper sinkHelper(
        "ns3::UdpSocketFactory",
        InetSocketAddress(Ipv4Address::GetAny(), port));
    ApplicationContainer sinkApp = sinkHelper.Install(nodes.Get(0));
    sinkApp.Start(Seconds(0.0));
    sinkApp.Stop(Seconds(simSeconds));
    g_sink = DynamicCast<PacketSink>(sinkApp.Get(0));

    OnOffHelper onoff("ns3::UdpSocketFactory",
                      InetSocketAddress(ifaces.GetAddress(0), port));
    onoff.SetAttribute("DataRate", DataRateValue(DataRate(uint64_t(offeredMbps * 1e6))));
    onoff.SetAttribute("PacketSize", UintegerValue(packetBytes));
    onoff.SetAttribute("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=1]"));
    onoff.SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0]"));
    ApplicationContainer srcApp = onoff.Install(nodes.Get(1));
    srcApp.Start(Seconds(1.0));
    srcApp.Stop(Seconds(simSeconds));

    // Step the ISAC mode across the sim: comm share shrinks over time.
    const IsacMode seq[] = {COMMUNICATION_ONLY, COMMUNICATION_CENTRIC,
                            JOINT_ISAC, SENSING_CENTRIC, SENSING_ONLY};
    for (int i = 0; i < 5; ++i)
    {
        Simulator::Schedule(Seconds(0.5 + i * (simSeconds / 5.0)), &SetMode,
                            seq[i]);
    }

    FlowMonitorHelper fmHelper;
    Ptr<FlowMonitor> monitor = fmHelper.InstallAll();

    std::printf("# thz-ntn-isac-coexist-traffic\n");
    std::printf("#   sim=%.0fs offered=%.0fMbps subBands=%u UEs=%u "
                "perSubBand=%.1fMbps\n",
                simSeconds, offeredMbps, numSubBands, numUes, g_perSubBandMbps);
    std::printf("#   ISAC mode steps every %.0fs: COMM_ONLY→COMM_CENTRIC→"
                "JOINT→SENSING_CENTRIC→SENSING_ONLY\n", simSeconds / 5.0);
    std::printf("# %5s  %-16s  %7s  %8s  %9s\n",
                "t_s", "isac_mode", "comm_sb", "commCap", "goodput");

    Simulator::Schedule(Seconds(2.0), &IsacProbe);
    Simulator::Stop(Seconds(simSeconds + 0.1));
    Simulator::Run();

    const uint64_t totalRx = g_sink ? g_sink->GetTotalRx() : 0;
    std::printf("# === summary ===  totalRxBytes=%lu avgGoodput=%.3f Mbps "
                "(comm ceiling shrinks as sensing share grows)\n",
                (unsigned long)totalRx, totalRx * 8.0 / simSeconds / 1e6);
    Simulator::Destroy();
    return 0;
}
