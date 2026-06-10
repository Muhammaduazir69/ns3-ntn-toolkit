/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only
//
// oran-ntn-cross-domain-slice — the paper's representative CROSS-DOMAIN
// closed loop (Deng 2026 Sec. IV-B enabler 4; adoption plan WS3), end to end
// on a real routed data plane:
//
//   space segment:  TWO real SGP4 Walker satellites (ENU-projected). Every
//                   second each transport path's P2P delay is re-set from
//                   the LIVE slant geometry — satA serves at t=0 and
//                   recedes; satB trails it on the same plane.
//   slices:         three NtnOranApplication QoS flows GW -> edge server
//                   (URLLC 5QI82/SST2, eMBB 5QI2/SST1, mMTC 5QI9/SST3),
//                   every packet carrying its identity as real bytes.
//   measurement:    NtnOranAiFlowMonitor -> per-slice KPM (TS 28.552 names),
//                   consumed by OranNtnNwdaf (CN-domain analytics).
//   decision:       OranNtnOnnxXapp (.onnx if built with ONNX Runtime, else
//                   the registered heuristic) flags URLLC degradation from
//                   the measured feature window; OranNtnCrossDomainSmo then
//                   coordinates THREE domains: RAN quota (live eMBB offered-
//                   rate cap), TPN path (OranNtnTpnController re-routes the
//                   GW static route to satB), CN compute (edge-link service
//                   rate). All actuations are real packet-level changes.
//
// Watch the measured URLLC one-way delay climb as satA recedes, then snap
// back when the TPN switches the route to the rising satB.
//
// Quick test:  --simSeconds=120
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/ntn-oran-ai-flow-monitor.h"
#include "ns3/ntn-oran-application.h"
#include "ns3/ntn-oran-sink.h"
#include "ns3/ntn-tr38811-mobility-model.h"
#include "ns3/oran-ntn-cross-domain.h"
#include "ns3/oran-ntn-onnx-xapp.h"
#include "ns3/point-to-point-module.h"
#include "ns3/sgp4-mobility-model.h"
#include "ns3/walker-constellation.h"

#include <cmath>
#include <cstdio>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("OranNtnCrossDomainSlice");

namespace
{
constexpr double kC = 299792458.0;

Ptr<MobilityModel> g_gw, g_srv, g_satA, g_satB;
Ptr<PointToPointChannel> g_chGwA, g_chASrv, g_chGwB, g_chBSrv;

double
PathLatencyMs(Ptr<MobilityModel> sat)
{
    return (g_gw->GetDistanceFrom(sat) + g_srv->GetDistanceFrom(sat)) / kC * 1e3;
}

// Re-set the four P2P delays from the live SGP4 geometry every second.
void
GeometryTick()
{
    g_chGwA->SetAttribute("Delay", TimeValue(Seconds(g_gw->GetDistanceFrom(g_satA) / kC)));
    g_chASrv->SetAttribute("Delay", TimeValue(Seconds(g_srv->GetDistanceFrom(g_satA) / kC)));
    g_chGwB->SetAttribute("Delay", TimeValue(Seconds(g_gw->GetDistanceFrom(g_satB) / kC)));
    g_chBSrv->SetAttribute("Delay", TimeValue(Seconds(g_srv->GetDistanceFrom(g_satB) / kC)));
    Simulator::Schedule(Seconds(1.0), &GeometryTick);
}

} // namespace

int
main(int argc, char* argv[])
{
    double simSeconds = 120.0;
    double leoAltKm = 550.0;

    CommandLine cmd(__FILE__);
    cmd.AddValue("simSeconds", "Simulation duration (s)", simSeconds);
    cmd.AddValue("leoAltKm", "Constellation altitude (km)", leoAltKm);
    cmd.Parse(argc, argv);

    std::printf("# oran-ntn-cross-domain-slice (NWDAF + SMO + TPN + xApp, real SGP4 paths)\n");

    // ---- nodes: GW(0) satA(1) satB(2) SRV(3) ----------------------------
    NodeContainer nodes;
    nodes.Create(4);

    ns3::ntncon::WalkerConfig wcfg;
    wcfg.num_planes = 1;
    wcfg.total_sats = 80;
    wcfg.altitude_km = leoAltKm;
    wcfg.inclination_deg = 53.0;
    wcfg.epoch_unix_s = 1735689600.0;
    const auto elements = ns3::ntncon::WalkerConstellation::BuildDelta(wcfg);
    double subLat, subLon, subAlt;
    for (int i = 0; i < 2; ++i)
    {
        Ptr<ns3::ntncon::Sgp4MobilityModel> sgp4 =
            CreateObject<ns3::ntncon::Sgp4MobilityModel>();
        // satA = serving element at zenith; satB = the trailing element two
        // slots behind on the same plane (rises as satA recedes).
        sgp4->SetElements(elements[i == 0 ? 0 : 78]);
        if (i == 0)
        {
            sgp4->GetGeodetic(subLat, subLon, subAlt);
        }
        Ptr<NtnEnuProjectionMobilityModel> enu = CreateObject<NtnEnuProjectionMobilityModel>();
        enu->SetSource(sgp4);
        enu->SetReference(subLat, subLon, 0.0);
        nodes.Get(1 + i)->AggregateObject(enu);
    }
    MobilityHelper mob;
    mob.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    Ptr<ListPositionAllocator> pos = CreateObject<ListPositionAllocator>();
    pos->Add(Vector(0.0, 0.0, 10.0));       // GW
    pos->Add(Vector(50000.0, 0.0, 10.0));   // SRV (edge site 50 km away)
    mob.SetPositionAllocator(pos);
    mob.Install(NodeContainer(nodes.Get(0), nodes.Get(3)));
    g_gw = nodes.Get(0)->GetObject<MobilityModel>();
    g_srv = nodes.Get(3)->GetObject<MobilityModel>();
    g_satA = nodes.Get(1)->GetObject<MobilityModel>();
    g_satB = nodes.Get(2)->GetObject<MobilityModel>();

    // ---- two transport paths over real geometry -------------------------
    InternetStackHelper internet;
    internet.Install(nodes);
    PointToPointHelper p2p;
    p2p.SetDeviceAttribute("DataRate", StringValue("20Mbps"));
    Ipv4AddressHelper ipv4;

    NetDeviceContainer dGwA = p2p.Install(nodes.Get(0), nodes.Get(1));
    NetDeviceContainer dASrv = p2p.Install(nodes.Get(1), nodes.Get(3));
    NetDeviceContainer dGwB = p2p.Install(nodes.Get(0), nodes.Get(2));
    NetDeviceContainer dBSrv = p2p.Install(nodes.Get(2), nodes.Get(3));
    g_chGwA = DynamicCast<PointToPointChannel>(dGwA.Get(0)->GetChannel());
    g_chASrv = DynamicCast<PointToPointChannel>(dASrv.Get(0)->GetChannel());
    g_chGwB = DynamicCast<PointToPointChannel>(dGwB.Get(0)->GetChannel());
    g_chBSrv = DynamicCast<PointToPointChannel>(dBSrv.Get(0)->GetChannel());

    ipv4.SetBase("10.1.1.0", "255.255.255.0");
    ipv4.Assign(dGwA);
    ipv4.SetBase("10.1.2.0", "255.255.255.0");
    Ipv4InterfaceContainer iASrv = ipv4.Assign(dASrv);
    ipv4.SetBase("10.1.3.0", "255.255.255.0");
    ipv4.Assign(dGwB);
    ipv4.SetBase("10.1.4.0", "255.255.255.0");
    Ipv4InterfaceContainer iBSrv = ipv4.Assign(dBSrv);

    // Static forwarding: each sat forwards BOTH server subnets onward (so
    // either sat can carry either destination address); the GW's choice of
    // sat IS the TPN path decision.
    Ipv4StaticRoutingHelper srh;
    const Ipv4Address srvViaA = iASrv.GetAddress(1);
    const Ipv4Address srvViaB = iBSrv.GetAddress(1);
    Ptr<Ipv4StaticRouting> satARouting =
        srh.GetStaticRouting(nodes.Get(1)->GetObject<Ipv4>());
    satARouting->AddNetworkRouteTo(Ipv4Address("10.1.4.0"), Ipv4Mask("255.255.255.0"), 2);
    Ptr<Ipv4StaticRouting> satBRouting =
        srh.GetStaticRouting(nodes.Get(2)->GetObject<Ipv4>());
    satBRouting->AddNetworkRouteTo(Ipv4Address("10.1.2.0"), Ipv4Mask("255.255.255.0"), 2);

    Ptr<Ipv4StaticRouting> gwRouting = srh.GetStaticRouting(nodes.Get(0)->GetObject<Ipv4>());
    const uint32_t gwBaseRoutes = gwRouting->GetNRoutes();
    auto routeVia = [gwRouting, srvViaA, srvViaB, gwBaseRoutes](bool viaA) {
        // Re-point both server-side addresses through the chosen sat
        // (GW Ipv4 interface 1 = satA link, 2 = satB link).
        while (gwRouting->GetNRoutes() > gwBaseRoutes)
        {
            gwRouting->RemoveRoute(gwBaseRoutes);
        }
        gwRouting->AddHostRouteTo(srvViaA, viaA ? 1 : 2);
        gwRouting->AddHostRouteTo(srvViaB, viaA ? 1 : 2);
    };
    routeVia(true); // start on satA

    GeometryTick();

    // ---- three slices as real ORAN QoS flows ----------------------------
    const double simStop = simSeconds - 0.5;
    struct SliceDef
    {
        const char* name;
        uint8_t fiveQi, sst;
        NtnOranApplication::Profile profile;
        uint16_t port;
    };
    const SliceDef defs[3] = {
        {"urllc", 82, 2, NtnOranApplication::URLLC_PERIODIC, 7001},
        {"embb", 2, 1, NtnOranApplication::CBR_SATURATING, 7002},
        {"mmtc", 9, 3, NtnOranApplication::MMTC_PERIODIC, 7003},
    };
    Ptr<NtnOranAiFlowMonitor> mon = CreateObject<NtnOranAiFlowMonitor>();
    Ptr<NtnOranApplication> apps[3];
    Ptr<NtnOranSink> sinks[3];
    for (int i = 0; i < 3; ++i)
    {
        sinks[i] = CreateObject<NtnOranSink>();
        sinks[i]->SetAttribute(
            "Local", AddressValue(InetSocketAddress(Ipv4Address::GetAny(), defs[i].port)));
        nodes.Get(3)->AddApplication(sinks[i]);
        sinks[i]->SetStartTime(Seconds(0.0));

        apps[i] = CreateObject<NtnOranApplication>();
        apps[i]->SetRemote(InetSocketAddress(i == 1 ? srvViaB : srvViaA, defs[i].port));
        apps[i]->SetProfile(defs[i].profile);
        if (defs[i].profile == NtnOranApplication::CBR_SATURATING)
        {
            apps[i]->SetAttribute("DataRate", DataRateValue(DataRate("8Mbps")));
        }
        apps[i]->SetFlowIdentity(defs[i].fiveQi, defs[i].sst, 0x000001, 100 + i, 3);
        nodes.Get(0)->AddApplication(apps[i]);
        apps[i]->SetStartTime(Seconds(1.0));
        apps[i]->SetStopTime(Seconds(simStop));

        mon->AddSource(apps[i]);
        mon->AddSink(sinks[i]);
    }
    mon->Start();

    // ---- the cross-domain control plane ----------------------------------
    Ptr<OranNtnNwdaf> nwdaf = CreateObject<OranNtnNwdaf>();
    mon->RegisterE2Consumer(
        [nwdaf](const NtnOranAiFlowMonitor::KpmIndication& ind) { nwdaf->ConsumeKpm(ind); });

    Ptr<OranNtnTpnController> tpn = CreateObject<OranNtnTpnController>();
    tpn->RegisterPath("sat-A", [] { return PathLatencyMs(g_satA); },
                      [&routeVia](uint8_t) { routeVia(true); });
    tpn->RegisterPath("sat-B", [] { return PathLatencyMs(g_satB); },
                      [&routeVia](uint8_t) { routeVia(false); });

    Ptr<OranNtnCrossDomainSmo> smo = CreateObject<OranNtnCrossDomainSmo>();
    smo->SetNwdaf(nwdaf);
    smo->SetTpnController(tpn);
    // RAN-domain knob: the eMBB offered-rate cap (live DataRate actuation).
    smo->SetRadioQuotaSetter([&apps](uint8_t sst, double share) {
        if (sst == 2) // URLLC at risk -> throttle the eMBB neighbour
        {
            const double embbMbps = 8.0 * std::max(0.2, 1.5 - share);
            apps[1]->SetAttribute("DataRate",
                                  DataRateValue(DataRate(uint64_t(embbMbps * 1e6))));
        }
    });
    // CN/edge knob: edge-link service rate (real queueing effect).
    smo->SetComputeAllocSetter([&dASrv, &dBSrv](uint8_t, double share) {
        const uint64_t bps = uint64_t(20e6 * std::clamp(0.5 + share, 0.5, 1.5));
        dASrv.Get(0)->SetAttribute("DataRate", DataRateValue(DataRate(bps)));
        dBSrv.Get(0)->SetAttribute("DataRate", DataRateValue(DataRate(bps)));
    });
    smo->AddSlice(/*sst*/ 2, /*budgetMs*/ 8.0, /*radio*/ 0.3, /*compute*/ 0.3);
    smo->SetCoordinationPeriod(Seconds(2.0));
    smo->Start();

    // AI-native decision: the xApp consumes the MEASURED URLLC feature window
    // (ONNX model when available, registered heuristic otherwise).
    Ptr<OranNtnOnnxXapp> xapp = CreateObject<OranNtnOnnxXapp>();
    xapp->RegisterHeuristic([](const std::vector<double>& f) {
        // f = {delayMeanMs, delaySlopeMsPerPeriod, lossMean}
        return std::vector<double>{(f[0] > 8.0 || (f[1] > 0.1 && f[0] > 5.0) || f[2] > 0.05)
                                       ? 1.0
                                       : 0.0};
    });
    std::printf("# inference engine: %s\n",
                OranNtnOnnxXapp::IsOnnxAvailable() ? "ONNX Runtime" : "heuristic fallback");

    Simulator::Schedule(Seconds(2.0), [&]() {
        static std::function<void()> tick = [&]() {
            // URLLC flow features from the measured KPM window.
            FlowId urllcId = 0;
            for (const auto& [id, series] : mon->GetKpmSeries())
            {
                OranFlowKey key;
                if (mon->GetClassifier()->FindFlow(id, key) && key.fiveQi == 82)
                {
                    urllcId = id;
                }
            }
            if (urllcId != 0)
            {
                const auto f = mon->GetFeatures(urllcId);
                const auto act = xapp->Infer({f.delayMeanMs, f.delaySlope, f.lossMean});
                if (!act.empty() && act[0] > 0.5)
                {
                    tpn->SelectPathForSlice(2, 8.0);
                }
            }
            Simulator::Schedule(Seconds(1.0), tick);
        };
        tick();
    });

    std::printf("# %5s  %8s  %8s  %7s  %10s  %8s  %6s\n", "t_s", "latA_ms", "latB_ms",
                "path", "urllc_owd", "risk", "embb");
    Simulator::Schedule(Seconds(5.0), [&]() {
        static std::function<void()> rep = [&]() {
            const auto a = nwdaf->GetAnalytics(2);
            double urllcOwd = 0;
            for (const auto& [k, fs] : sinks[0]->GetFlowStats())
            {
                urllcOwd = fs.MeanDelayMs();
            }
            std::printf("  %5.1f  %8.2f  %8.2f  %7s  %8.2f  %8.2f  %6.1f\n",
                        Simulator::Now().GetSeconds(), PathLatencyMs(g_satA),
                        PathLatencyMs(g_satB), tpn->GetActivePath(2).c_str(), urllcOwd,
                        a.slaRisk, nwdaf->GetAnalytics(1).loadMbps);
            Simulator::Schedule(Seconds(5.0), rep);
        };
        rep();
    });

    Simulator::Stop(Seconds(simSeconds));
    Simulator::Run();

    std::printf("# === summary ===  tpnSwitches=%u smoDecisions=%zu inferences=%lu\n",
                tpn->GetSwitchCount(), smo->GetDecisions().size(),
                static_cast<unsigned long>(xapp->GetInferenceCount()));
    for (int i = 0; i < 3; ++i)
    {
        for (const auto& [k, fs] : sinks[i]->GetFlowStats())
        {
            std::printf("#   %-6s 5QI=%-3u owd=%.2f ms jitter=%.3f ms loss=%.4f thr=%.3f Mbps\n",
                        defs[i].name, fs.fiveQi, fs.MeanDelayMs(), fs.jitterMs,
                        fs.LossRatio(), fs.ThroughputMbps());
        }
    }
    Simulator::Destroy();
    return 0;
}
