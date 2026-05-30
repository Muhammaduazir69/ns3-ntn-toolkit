/*
 * SPDX-License-Identifier: GPL-2.0-only
 * Copyright (c) 2026 Muhammad Uzair (ns3-ntn-toolkit)
 *
 * sagin-slice-traffic — cross-layer network-slice steering with REAL traffic.
 * A ground gateway carries three concurrent UDP flows (one per 5G QFI band):
 *   QFI 1  → URLLC  (low latency, no GEO)
 *   QFI 7  → eMBB   (broadband, GEO too slow)
 *   QFI 50 → mMTC   (delay-tolerant, GEO allowed)
 * A SaginSliceRouter maps each QFI to its slice profile and chooses a serving
 * layer (HAPS / LEO / GEO). The example wires one PointToPoint link per layer
 * with that layer's realistic propagation delay, routes each slice's flow to
 * its chosen layer, and uses FlowMonitor to show the per-slice mean delay:
 * URLLC lands on the low-delay layer, mMTC accepts the high-delay GEO, eMBB
 * sits on LEO — the layer choice comes from SaginSliceRouter, not hardcoded.
 *
 * Quick test:  --simSeconds=60 --dataRateMbps=5
 */
#include "ns3/applications-module.h"
#include "ns3/command-line.h"
#include "ns3/constant-position-mobility-model.h"
#include "ns3/core-module.h"
#include "ns3/flow-monitor-helper.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/ipv4-flow-classifier.h"
#include "ns3/ipv4-global-routing-helper.h"
#include "ns3/point-to-point-helper.h"

#include "ns3/multi-layer-router.h"
#include "ns3/sagin-slice-router.h"

#include <cmath>
#include <cstdio>
#include <map>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("SaginSliceTraffic");

int
main(int argc, char* argv[])
{
    double simSeconds = 60.0;
    double dataRateMbps = 5.0;
    uint32_t packetBytes = 1000;
    double hapsAltKm = 20.0;
    double leoAltKm = 550.0;
    double geoAltKm = 35786.0;

    CommandLine cmd(__FILE__);
    cmd.AddValue("simSeconds", "Simulation duration (s)", simSeconds);
    cmd.AddValue("dataRateMbps", "Per-slice offered load (Mbps)", dataRateMbps);
    cmd.AddValue("packetBytes", "UDP payload size (bytes)", packetBytes);
    cmd.AddValue("hapsAltKm", "HAPS altitude (km)", hapsAltKm);
    cmd.AddValue("leoAltKm", "LEO altitude (km)", leoAltKm);
    cmd.AddValue("geoAltKm", "GEO altitude (km)", geoAltKm);
    cmd.Parse(argc, argv);

    constexpr double kC = 299792458.0;

    // --- Router topology: ground + HAPS + LEO + GEO candidates ---
    Ptr<MultiLayerRouter> mlr = CreateObject<MultiLayerRouter>();
    Ptr<ConstantPositionMobilityModel> gMob =
        CreateObject<ConstantPositionMobilityModel>();
    gMob->SetPosition(Vector(0, 0, 0));
    Ptr<ConstantPositionMobilityModel> hapsMob =
        CreateObject<ConstantPositionMobilityModel>();
    hapsMob->SetPosition(Vector(0, 0, hapsAltKm * 1000.0));
    Ptr<ConstantPositionMobilityModel> leoMob =
        CreateObject<ConstantPositionMobilityModel>();
    leoMob->SetPosition(Vector(300e3, 0, leoAltKm * 1000.0)); // off-axis
    Ptr<ConstantPositionMobilityModel> geoMob =
        CreateObject<ConstantPositionMobilityModel>();
    geoMob->SetPosition(Vector(0, 0, geoAltKm * 1000.0)); // overhead
    mlr->AddNode(SaginLayer::Haps, hapsMob);
    mlr->AddNode(SaginLayer::Leo, leoMob);
    mlr->AddNode(SaginLayer::Leo, geoMob);

    Ptr<SaginSliceRouter> sr = CreateObject<SaginSliceRouter>();
    sr->SetRouter(mlr);
    // A tight URLLC variant (1 ms) so it is forced to the HAPS layer.
    ntnslice::SliceProfile tightUrllc = ntnslice::DefaultUrllc();
    tightUrllc.latencyBudgetMs = 1.0;
    sr->AddSlice(tightUrllc);

    // Decide each slice's serving-layer altitude from the router.
    auto topAltKm = [&](uint8_t qfi) {
        auto path = sr->RouteForQfi(gMob, qfi);
        return path.empty() ? 0.0 : path.back().node->GetPosition().z / 1000.0;
    };
    const double urllcAltKm = topAltKm(1);
    const double embbAltKm = topAltKm(7);
    const double mmtcAltKm = topAltKm(50);

    // --- Data plane: ground gateway + one relay node per slice layer ---
    NodeContainer gnd;
    gnd.Create(1);
    NodeContainer relays;
    relays.Create(3); // 0=URLLC layer, 1=eMBB layer, 2=mMTC layer
    InternetStackHelper internet;
    internet.Install(gnd);
    internet.Install(relays);

    struct Slice
    {
        const char* name;
        uint8_t qfi;
        double altKm;
        uint16_t port;
    };
    Slice slices[3] = {{"URLLC", 1, urllcAltKm, 7100},
                       {"eMBB", 7, embbAltKm, 7101},
                       {"mMTC", 50, mmtcAltKm, 7102}};

    Ipv4AddressHelper ipv4;
    std::map<int, Ipv4Address> dstAddr;
    for (int i = 0; i < 3; ++i)
    {
        PointToPointHelper p2p;
        p2p.SetDeviceAttribute("DataRate", DataRateValue(DataRate(uint64_t(100e6))));
        // Realistic one-way propagation delay for this slice's layer.
        p2p.SetChannelAttribute(
            "Delay", TimeValue(Seconds(slices[i].altKm * 1000.0 / kC)));
        NetDeviceContainer dev =
            p2p.Install(NodeContainer(gnd.Get(0), relays.Get(i)));
        char net[20];
        std::snprintf(net, sizeof(net), "10.30.%d.0", i + 1);
        ipv4.SetBase(net, "255.255.255.0");
        Ipv4InterfaceContainer ifc = ipv4.Assign(dev);
        dstAddr[i] = ifc.GetAddress(1);
    }
    Ipv4GlobalRoutingHelper::PopulateRoutingTables();

    // One sink per slice on its relay; one source per slice on the gateway.
    for (int i = 0; i < 3; ++i)
    {
        PacketSinkHelper sink(
            "ns3::UdpSocketFactory",
            InetSocketAddress(Ipv4Address::GetAny(), slices[i].port));
        ApplicationContainer sa = sink.Install(relays.Get(i));
        sa.Start(Seconds(0.0));
        sa.Stop(Seconds(simSeconds));

        OnOffHelper onoff("ns3::UdpSocketFactory",
                          InetSocketAddress(dstAddr[i], slices[i].port));
        onoff.SetAttribute("DataRate",
                           DataRateValue(DataRate(uint64_t(dataRateMbps * 1e6))));
        onoff.SetAttribute("PacketSize", UintegerValue(packetBytes));
        onoff.SetAttribute(
            "OnTime", StringValue("ns3::ConstantRandomVariable[Constant=1]"));
        onoff.SetAttribute(
            "OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0]"));
        ApplicationContainer src = onoff.Install(gnd.Get(0));
        src.Start(Seconds(1.0));
        src.Stop(Seconds(simSeconds));
    }

    FlowMonitorHelper fmHelper;
    Ptr<FlowMonitor> monitor = fmHelper.InstallAll();

    std::printf("# sagin-slice-traffic (SaginSliceRouter cross-layer steering)\n");
    std::printf("#   sim=%.0fs perSliceLoad=%.1fMbps\n", simSeconds, dataRateMbps);
    std::printf("#   slice→layer (chosen by SaginSliceRouter):\n");
    for (int i = 0; i < 3; ++i)
    {
        std::printf("#     %-6s QFI %-2u → layer alt %.0f km "
                    "(one-way delay %.2f ms)\n",
                    slices[i].name, slices[i].qfi, slices[i].altKm,
                    slices[i].altKm * 1000.0 / kC * 1000.0);
    }

    Simulator::Stop(Seconds(simSeconds + 0.1));
    Simulator::Run();

    monitor->CheckForLostPackets();
    Ptr<Ipv4FlowClassifier> classifier =
        DynamicCast<Ipv4FlowClassifier>(fmHelper.GetClassifier());
    const auto stats = monitor->GetFlowStats();
    std::printf("# === per-slice FlowMonitor results ===\n");
    std::printf("# %-6s  %-8s  %10s  %10s  %8s\n",
                "slice", "dstPort", "rxPackets", "meanDelay", "Mbps");
    for (const auto& kv : stats)
    {
        const auto ft = classifier->FindFlow(kv.first);
        const auto& s = kv.second;
        // Identify the slice by destination port.
        const char* name = "?";
        for (int i = 0; i < 3; ++i)
        {
            if (ft.destinationPort == slices[i].port)
            {
                name = slices[i].name;
            }
        }
        const double durationS =
            (s.timeLastRxPacket - s.timeFirstTxPacket).GetSeconds();
        const double mbps = durationS > 0 ? s.rxBytes * 8.0 / durationS / 1e6 : 0.0;
        const double meanDelayMs =
            s.rxPackets ? (s.delaySum.GetSeconds() / s.rxPackets) * 1000.0 : 0.0;
        std::printf("  %-6s  %-8u  %10lu  %8.2fms  %8.3f\n",
                    name, ft.destinationPort, (unsigned long)s.rxPackets,
                    meanDelayMs, mbps);
    }
    Simulator::Destroy();
    return 0;
}
