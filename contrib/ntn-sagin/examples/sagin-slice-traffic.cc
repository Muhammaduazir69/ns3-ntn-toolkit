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
 * layer (HAPS / LEO / GEO). Layer mobility is real: the LEO flies a genuine
 * SGP4 Walker pass (overhead near t=0, receding through the run), the HAPS
 * station-keeps on its figure-8, and the GEO is quasi-static at the true
 * geostationary slant range. The slice router is re-run every second on the
 * LIVE geometry and each slice's serving link delay is retuned to the chosen
 * layer's current slant range — so the layer choice changes DURING the run
 * and acts on real packets. Per-slice delay/jitter/loss are MEASURED by
 * NtnOranSink from in-band NtnOranPayloadHeader bytes (WS1 application
 * suite): URLLC stays on the low-delay layer, mMTC accepts the high-delay
 * GEO, eMBB rides the LEO pass — chosen by SaginSliceRouter, not hardcoded.
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
#include "ns3/ntn-tr38811-mobility-model.h"
#include "ns3/ipv4-global-routing-helper.h"
#include "ns3/point-to-point-channel.h"
#include "ns3/point-to-point-helper.h"
#include "ns3/sgp4-mobility-model.h"
#include "ns3/walker-constellation.h"

#include "ns3/haps-mobility-model.h"
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

    // HAPS: real station-keeping (figure-8 drift inside its keep-radius).
    Ptr<HapsMobilityModel> hapsMob = CreateObject<HapsMobilityModel>();
    hapsMob->SetCenter(Vector(0, 0, hapsAltKm * 1000.0));

    // LEO: real SGP4 Walker element projected into the scenario's local ENU
    // frame — overhead near t=0 and receding with genuine orbital dynamics,
    // so the slice router's layer choice can change during the run.
    ns3::ntncon::WalkerConfig wcfgSat;
    wcfgSat.num_planes = 1;
    wcfgSat.total_sats = 80;
    wcfgSat.altitude_km = leoAltKm;
    wcfgSat.inclination_deg = 53.0;
    wcfgSat.epoch_unix_s = 1735689600.0;
    const auto satElements = ns3::ntncon::WalkerConstellation::BuildDelta(wcfgSat);
    Ptr<ns3::ntncon::Sgp4MobilityModel> leoSgp4 =
        CreateObject<ns3::ntncon::Sgp4MobilityModel>();
    leoSgp4->SetElements(satElements[0]);
    double leoSubLat, leoSubLon, leoSubAlt;
    leoSgp4->GetGeodetic(leoSubLat, leoSubLon, leoSubAlt);
    Ptr<NtnEnuProjectionMobilityModel> leoMob =
        CreateObject<NtnEnuProjectionMobilityModel>();
    leoMob->SetSource(leoSgp4);
    leoMob->SetReference(leoSubLat, leoSubLon, 0.0);

    // GEO: quasi-static IS the physically correct behaviour for a
    // geostationary satellite. The ground gateway is taken to sit on the
    // equator at the GEO's longitude, so in this local ENU frame the GEO is
    // at zenith with a slant range equal to the geostationary altitude
    // (35 786 km above the sub-satellite point, TR 38.821 GEO geometry).
    Ptr<ConstantPositionMobilityModel> geoMob =
        CreateObject<ConstantPositionMobilityModel>();
    geoMob->SetPosition(Vector(0, 0, geoAltKm * 1000.0));

    mlr->AddNode(SaginLayer::Haps, hapsMob);
    mlr->AddNode(SaginLayer::Leo, leoMob);
    mlr->AddNode(SaginLayer::Leo, geoMob); // GEO candidate in the top layer

    Ptr<SaginSliceRouter> sr = CreateObject<SaginSliceRouter>();
    sr->SetRouter(mlr);
    // A tight URLLC variant (1 ms) so it is forced to the HAPS layer.
    ntnslice::SliceProfile tightUrllc = ntnslice::DefaultUrllc();
    tightUrllc.latencyBudgetMs = 1.0;
    sr->AddSlice(tightUrllc);

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
        uint16_t port;
        Ptr<PointToPointChannel> chan;       // delay retuned per routing tick
        Ptr<MobilityModel> servingNode;      // last chosen top-layer node
    };
    Slice slices[3] = {{"URLLC", 1, 7100, nullptr, nullptr},
                       {"eMBB", 7, 7101, nullptr, nullptr},
                       {"mMTC", 50, 7102, nullptr, nullptr}};

    Ipv4AddressHelper ipv4;
    std::map<int, Ipv4Address> dstAddr;
    for (int i = 0; i < 3; ++i)
    {
        PointToPointHelper p2p;
        p2p.SetDeviceAttribute("DataRate", DataRateValue(DataRate(uint64_t(100e6))));
        p2p.SetChannelAttribute("Delay", TimeValue(MilliSeconds(1))); // retuned below
        NetDeviceContainer dev =
            p2p.Install(NodeContainer(gnd.Get(0), relays.Get(i)));
        slices[i].chan = DynamicCast<PointToPointChannel>(dev.Get(0)->GetChannel());
        char net[20];
        std::snprintf(net, sizeof(net), "10.30.%d.0", i + 1);
        ipv4.SetBase(net, "255.255.255.0");
        Ipv4InterfaceContainer ifc = ipv4.Assign(dev);
        dstAddr[i] = ifc.GetAddress(1);
    }
    Ipv4GlobalRoutingHelper::PopulateRoutingTables();

    // Re-run the slice router on the LIVE geometry and retune each slice's
    // serving-link delay to the chosen layer's current slant range. As the
    // SGP4 LEO recedes its elevation falls and the eMBB choice migrates
    // between layers mid-run; the decision acts on the real packet delay.
    auto layerTag = [&](Ptr<MobilityModel> n) {
        return (n == geoMob) ? "GEO" : (n == leoMob) ? "LEO"
                             : (n == hapsMob)        ? "HAPS"
                                                     : "?";
    };
    auto applyRoutes = [&]() {
        const double t = Simulator::Now().GetSeconds();
        for (auto& s : slices)
        {
            auto path = sr->RouteForQfi(gMob, s.qfi);
            if (path.empty())
            {
                continue;
            }
            const SaginHop& top = path.back();
            s.chan->SetAttribute("Delay", TimeValue(Seconds(top.rangeM / kC)));
            if (top.node != s.servingNode)
            {
                std::printf("#   t=%5.1f  %-6s -> %-4s (alt %.0f km, one-way %.2f ms)\n",
                            t, s.name, layerTag(top.node),
                            top.node->GetPosition().z / 1000.0,
                            top.rangeM / kC * 1e3);
                s.servingNode = top.node;
            }
        }
    };
    for (double t = 0.0; t < simSeconds; t += 1.0)
    {
        Simulator::Schedule(Seconds(t), [&applyRoutes]() { applyRoutes(); });
    }

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
    std::printf("#   sim=%.0fs perSliceLoad=%.1fMbps  (SGP4 LEO pass; HAPS "
                "station-keeping; GEO quasi-static at %.0f km)\n",
                simSeconds, dataRateMbps, geoAltKm);
    std::printf("#   slice→layer transitions (chosen by SaginSliceRouter on live geometry):\n");

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
