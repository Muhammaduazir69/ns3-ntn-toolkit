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
 * its chosen layer; per-slice delay/jitter/loss are MEASURED by NtnOranSink
 * from in-band NtnOranPayloadHeader bytes (WS1 application suite):
 * URLLC lands on the low-delay layer, mMTC accepts the high-delay GEO, eMBB
 * sits on LEO — the layer choice comes from SaginSliceRouter, not hardcoded.
 *
 * Quick test:  --simSeconds=60 --dataRateMbps=5
 */
#include "ns3/applications-module.h"
#include "ns3/command-line.h"
#include "ns3/constant-position-mobility-model.h"
#include "ns3/core-module.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/ntn-oran-application.h"
#include "ns3/ntn-oran-sink.h"
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
    // 5QI per slice class: URLLC 82, eMBB 2, mMTC 9; the S-NSSAI SST encodes
    // the slice (TS 23.501: 1 = eMBB, 2 = URLLC, 3 = mMTC).
    Ptr<NtnOranSink> sliceSinks[3];
    const uint8_t sliceQi[3] = {82, 2, 9};
    const uint8_t sliceSst[3] = {2, 1, 3};
    for (int i = 0; i < 3; ++i)
    {
        sliceSinks[i] = CreateObject<NtnOranSink>();
        sliceSinks[i]->SetAttribute(
            "Local",
            AddressValue(InetSocketAddress(Ipv4Address::GetAny(), slices[i].port)));
        relays.Get(i)->AddApplication(sliceSinks[i]);
        sliceSinks[i]->SetStartTime(Seconds(0.0));
        sliceSinks[i]->SetStopTime(Seconds(simSeconds));

        Ptr<NtnOranApplication> src = CreateObject<NtnOranApplication>();
        src->SetRemote(InetSocketAddress(dstAddr[i], slices[i].port));
        src->SetProfile(NtnOranApplication::CBR_SATURATING);
        src->SetAttribute("DataRate",
                          DataRateValue(DataRate(uint64_t(dataRateMbps * 1e6))));
        src->SetAttribute("PacketSize", UintegerValue(packetBytes));
        src->SetFlowIdentity(sliceQi[i], sliceSst[i], 0x000001, slices[i].qfi, i);
        gnd.Get(0)->AddApplication(src);
        src->SetStartTime(Seconds(1.0));
        src->SetStopTime(Seconds(simSeconds));
    }

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

    std::printf("# === per-slice measured results (in-band header) ===\n");
    std::printf("# %-6s  %4s  %10s  %10s  %9s  %8s  %8s\n",
                "slice", "5QI", "rxPackets", "meanDelay", "jitter", "loss", "Mbps");
    for (int i = 0; i < 3; ++i)
    {
        for (const auto& kv : sliceSinks[i]->GetFlowStats())
        {
            const auto& fs = kv.second;
            std::printf("  %-6s  %4u  %10lu  %8.2fms  %7.3fms  %8.4f  %8.3f\n",
                        slices[i].name, fs.fiveQi, (unsigned long)fs.rxPackets,
                        fs.MeanDelayMs(), fs.jitterMs, fs.LossRatio(),
                        fs.ThroughputMbps());
        }
    }
    Simulator::Destroy();
    return 0;
}
