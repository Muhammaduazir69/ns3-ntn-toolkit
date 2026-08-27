/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only
//
// sagin-sgp4-routed-traffic — Roadmap §4.4.4 + §4.4.5 (SGP4-driven SAGIN with
// graph shortest-path ISL routing over the live contact graph).
//
// This example COMPOSES two contrib modules into one real ns-3 data plane:
//   * ntn-constellation : Sgp4MobilityModel (LEO geometry), ContactGraphScheduler
//                         (GSL/ISL visibility events), ContactGraphRouter
//                         (Dijkstra weighted shortest path over the contact graph)
//   * ntn-sagin         : the air/ground SAGIN access segment
//
// Topology (a genuine Space-Air-Ground path, ground -> UAV -> HAPS -> LEO -> GW):
//
//     UE --- UAV --- HAPS =====[ GSL ]=====> satA --[ISL]-- satB
//                      ^                        \             /
//                  (gs id 1)                     \===[GSL]===/====> GW --- SRV
//                                                              (gs id 2)
//
// The five SPACE links (HAPS-satA, HAPS-satB, satA-satB ISL, GW-satA, GW-satB)
// are gated by the contact graph: when the scheduler reports a contact UP we
// bring the corresponding ns-3 Ipv4 interface UP, set its channel delay from the
// real slant range and gate usability on an honest binary link budget (closes /
// does not close — no sigmoid PER); on a contact DOWN we bring the interface
// DOWN. After every transition we call
// Ipv4GlobalRoutingHelper::RecomputeRoutingTables(), so ns-3's own global
// routing re-routes the live UDP flow over whatever space path currently exists.
//
// Because the LEO satellites move under SGP4, the contact graph changes during
// the pass: satA hands the gs1 (HAPS) link to satB, the route flips, and the
// NtnOranSink goodput/delay (in-band header) track the actual end-to-end connectivity. The example
// also prints, each second, the ContactGraphRouter's Dijkstra path so the
// model-level decision can be cross-checked against the measured data plane.
//
// Everything is sim-time driven and parameter-dynamic (--simSeconds, --handover
// gap via --leadSeconds, --minElevDeg, --dataRateMbps ...). Nothing is hardcoded.
//
// Quick test:  --simSeconds=360 --dataRateMbps=4
#include "ns3/applications-module.h"
#include "ns3/command-line.h"
#include "ns3/constant-position-mobility-model.h"
#include "ns3/core-module.h"
#include "ns3/error-model.h"
#include "ns3/ntn-oran-application.h"
#include "ns3/ntn-oran-sink.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/ipv4-global-routing-helper.h"
#include "ns3/ipv4.h"
#include "ns3/point-to-point-channel.h"
#include "ns3/point-to-point-helper.h"

#include "ns3/contact-graph-router.h"
#include "ns3/contact-graph-scheduler.h"
#include "ns3/orbital-elements.h"
#include "ns3/sgp4-mobility-model.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <map>
#include <utility>
#include <vector>

using namespace ns3;
using ns3::ntncon::ContactEvent;
using ns3::ntncon::ContactGraphRouter;
using ns3::ntncon::ContactGraphScheduler;
using ns3::ntncon::KeplerianElements;
using ns3::ntncon::Sgp4MobilityModel;

NS_LOG_COMPONENT_DEFINE("SaginSgp4RoutedTraffic");

namespace
{
constexpr double kC = 299792458.0;
constexpr double kMu = 3.986004418e14; // Earth GM, m^3/s^2
constexpr double kRe = 6371000.0;      // mean Earth radius, m

// Contact-graph node IDs (shared by scheduler, router and our link map).
constexpr uint32_t kGs1 = 1;  // HAPS feeder point
constexpr uint32_t kGs2 = 2;  // ground gateway
constexpr uint32_t kSatA = 10;
constexpr uint32_t kSatB = 11;

// One gated space link in the ns-3 data plane.
struct SpaceLink
{
    Ptr<Ipv4> ipA, ipB;            // the two endpoints' Ipv4
    uint32_t ifA = 0, ifB = 0;     // interface indices of this link
    Ptr<RateErrorModel> emA, emB;  // receive error models (both directions)
    Ptr<PointToPointChannel> chan; // for dynamic propagation delay
    // SAGIN-1: the endpoints' mobility, so the link can re-read its own
    // geometry instead of keeping whatever the contact-up event carried.
    Ptr<MobilityModel> mobA, mobB;
    bool up = false;
};

std::map<std::pair<uint32_t, uint32_t>, SpaceLink> g_links;
Ptr<ContactGraphRouter> g_router;
Ptr<NtnOranSink> g_sink;
uint64_t g_lastRx = 0;
double g_eirpDbm = 95.0; // Ka-band GSL with high-gain dishes (txPwr + ant gain)
double g_freqHz = 20e9;
double g_noiseDbm = -100.0;

std::pair<uint32_t, uint32_t>
Canon(uint32_t a, uint32_t b)
{
    return (a <= b) ? std::make_pair(a, b) : std::make_pair(b, a);
}

double
FsplDb(double dM, double fHz)
{
    return 20.0 * std::log10(std::max(dM, 1.0)) + 20.0 * std::log10(fHz / 1e9) + 32.45;
}

double g_minSnrDb = 6.0; // decode threshold for the binary link-budget gate

// Bring an ns-3 Ipv4 link up/down to mirror a contact-graph transition, set
// the channel delay from the true slant range, and gate usability on an
// HONEST binary link budget: a Ka GSL/ISL with high-gain dishes either closes
// its budget (clean decode at these SNRs) or is unusable. No sigmoid PER —
// partial-loss radio behaviour belongs to the real-stack (mmwave) examples;
// this example's contribution is real contact-driven ROUTING.
// SAGIN-1: set a link's propagation delay and binary budget from a range.
// Both the contact-up event and the periodic refresh go through here so the
// two cannot drift apart.
void
SetLinkFromRange(SpaceLink& l, double rangeM)
{
    const double sinr = g_eirpDbm - FsplDb(rangeM, g_freqHz) - g_noiseDbm;
    const bool budgetCloses = sinr >= g_minSnrDb;
    l.chan->SetAttribute("Delay", TimeValue(Seconds(rangeM / kC)));
    l.emA->SetRate(budgetCloses ? 0.0 : 1.0);
    l.emB->SetRate(budgetCloses ? 0.0 : 1.0);
}

// SAGIN-1: follow the geometry of a contact that is already up.
//
// ContactGraphScheduler used to emit on visibility TRANSITIONS only, so the
// delay and the link budget were taken from the range at contact-up and stayed
// pinned for the whole pass. At the shipped defaults (550 km, 20 degree
// elevation floor) the GSL slant sweeps from about 550 km at zenith to about
// 1075 km at the floor, so the frozen figure is wrong by up to ~1.8 ms one way,
// and this example's headline output is a measured one-way delay. The same
// frozen range fed the binary budget, so a link that closed at zenith stayed
// closed all the way down to the horizon and never degraded.
//
// The scheduler now republishes the live range on every tick, so this takes its
// geometry from exactly the same source the transition events use rather than
// recomputing distances here. The sibling sagin-multihop-traffic.cc already
// refreshed correctly, so before this the two examples silently disagreed about
// the same physics.
void
OnContactUpdate(ContactEvent ev)
{
    auto it = g_links.find(Canon(ev.node_a, ev.node_b));
    if (it == g_links.end() || !it->second.up)
    {
        return;
    }
    SetLinkFromRange(it->second, ev.range_m);
}

void
ApplyContact(ContactEvent ev)
{
    const auto key = Canon(ev.node_a, ev.node_b);
    auto it = g_links.find(key);
    if (it == g_links.end())
    {
        return; // not a data-plane-mapped edge
    }
    SpaceLink& l = it->second;
    l.up = ev.up;

    if (ev.up)
    {
        l.ipA->SetUp(l.ifA);
        l.ipB->SetUp(l.ifB);
        SetLinkFromRange(l, ev.range_m);
    }
    else
    {
        l.ipA->SetDown(l.ifA);
        l.ipB->SetDown(l.ifB);
    }
    // ns-3 global routing follows the contact graph.
    Ipv4GlobalRoutingHelper::RecomputeRoutingTables();
}

// Per-second reporting: print the Dijkstra path the contact-graph router picks
// (model decision) alongside the measured goodput (data-plane truth).
void
Report(double simSeconds)
{
    const double now = Simulator::Now().GetSeconds();
    ContactGraphRouter::WeightedPath wp = g_router->ShortestPathWeighted(kGs1, kGs2);
    // SAGIN-7: the path packets ACTUALLY take.
    //
    // Forwarding here is Ipv4GlobalRoutingHelper::RecomputeRoutingTables(),
    // which minimises HOP COUNT over whichever interfaces ApplyContact brought
    // up. Nothing feeds the Dijkstra result into the forwarding tables, so the
    // "path=" column below was a model recommendation printed beside a goodput
    // the data plane produced by a different rule, with no way to tell whether
    // the two agreed.
    ContactGraphRouter::WeightedPath hp = g_router->ShortestPathHops(kGs1, kGs2);

    std::string pathStr;
    if (wp.path.empty())
    {
        pathStr = "(no path)";
    }
    else
    {
        for (size_t i = 0; i < wp.path.size(); ++i)
        {
            uint32_t n = wp.path[i];
            const char* tag = (n == kGs1)   ? "HAPS"
                              : (n == kGs2)  ? "GW"
                              : (n == kSatA) ? "satA"
                              : (n == kSatB) ? "satB"
                                             : "?";
            pathStr += tag;
            if (i + 1 < wp.path.size())
            {
                pathStr += "->";
            }
        }
    }

    const uint64_t tot = g_sink ? g_sink->GetTotalRx() : 0;
    const double mbps = (tot - g_lastRx) * 8.0 / 1e6;
    g_lastRx = tot;

    const double latMs =
        wp.path.empty() ? 0.0 : (wp.total_weight / kC) * 1e3; // range-sum -> 1-way ms

    // SAGIN-7: do the two rules pick the same route?
    const bool sameRoute = (wp.path == hp.path);
    const double hopLatMs =
        hp.path.empty() ? 0.0 : (hp.total_weight / kC) * 1e3;
    std::printf("  t=%6.1f  edges=%2zu  path=%-18s  pathLatency=%6.2f ms  goodput=%7.3f Mbps"
                "  forwarding=%s\n",
                now, g_router->NumEdges(), pathStr.c_str(), latMs, mbps,
                wp.path.empty()  ? "n/a"
                : sameRoute      ? "AGREES (hop-count route is the same)"
                                 : "DIFFERS from the printed path");
    if (!wp.path.empty() && !sameRoute)
    {
        std::printf("    NOTE (SAGIN-7): packets follow the minimum-HOP route "
                    "(%zu hops, %.2f ms) while the contact-graph model recommends "
                    "%zu hops at %.2f ms. The goodput above was produced by the hop-count "
                    "route, not by the path printed beside it.\n",
                    hp.path.size() ? hp.path.size() - 1 : 0, hopLatMs,
                    wp.path.size() ? wp.path.size() - 1 : 0, latMs);
    }

    if (now + 1.0 < simSeconds)
    {
        Simulator::Schedule(Seconds(1.0), &Report, simSeconds);
    }
}

// ECEF -> spherical sub-satellite lat/lon (deg). Matches the scheduler's model.
void
SubPoint(const Vector& ecef, double& latDeg, double& lonDeg)
{
    const double r = std::sqrt(ecef.x * ecef.x + ecef.y * ecef.y + ecef.z * ecef.z);
    latDeg = std::asin(std::max(-1.0, std::min(1.0, ecef.z / std::max(r, 1.0)))) * 180.0 / M_PI;
    lonDeg = std::atan2(ecef.y, ecef.x) * 180.0 / M_PI;
}

// Build one gated space P2P link between two already-created nodes and record it.
void
MakeSpaceLink(uint32_t idA,
              uint32_t idB,
              Ptr<Node> a,
              Ptr<Node> b,
              double capacityMbps,
              Ipv4AddressHelper& ipv4,
              const std::string& subnet)
{
    PointToPointHelper p2p;
    p2p.SetDeviceAttribute(
        "DataRate", DataRateValue(DataRate(static_cast<uint64_t>(capacityMbps * 1e6))));
    p2p.SetChannelAttribute("Delay", TimeValue(MilliSeconds(3))); // overwritten per contact
    NetDeviceContainer dev = p2p.Install(a, b);

    SpaceLink l;
    l.emA = CreateObject<RateErrorModel>();
    l.emA->SetUnit(RateErrorModel::ERROR_UNIT_PACKET);
    l.emA->SetRate(1.0);
    l.emB = CreateObject<RateErrorModel>();
    l.emB->SetUnit(RateErrorModel::ERROR_UNIT_PACKET);
    l.emB->SetRate(1.0);
    dev.Get(0)->SetAttribute("ReceiveErrorModel", PointerValue(l.emA));
    dev.Get(1)->SetAttribute("ReceiveErrorModel", PointerValue(l.emB));
    l.chan = DynamicCast<PointToPointChannel>(dev.Get(0)->GetChannel());

    ipv4.SetBase(Ipv4Address(subnet.c_str()), "255.255.255.0");
    Ipv4InterfaceContainer ifc = ipv4.Assign(dev);

    l.ipA = a->GetObject<Ipv4>();
    l.ipB = b->GetObject<Ipv4>();
    l.mobA = a->GetObject<MobilityModel>();
    l.mobB = b->GetObject<MobilityModel>();
    l.ifA = ifc.Get(0).second;
    l.ifB = ifc.Get(1).second;
    // Start DOWN; the first contact-up event brings the link in.
    l.ipA->SetDown(l.ifA);
    l.ipB->SetDown(l.ifB);

    g_links[Canon(idA, idB)] = l;
}
} // namespace

int
main(int argc, char* argv[])
{
    double simSeconds = 360.0;
    double altKm = 550.0;
    double inclDeg = 53.0;
    double leadSeconds = 180.0; // satA->gs2 / satB->gs1 phasing
    double minElevDeg = 20.0;
    double dataRateMbps = 4.0;
    uint32_t packetBytes = 1200;
    double spaceCapMbps = 50.0;
    double accessCapMbps = 100.0;
    double eirpDbm = 95.0;

    CommandLine cmd(__FILE__);
    cmd.AddValue("simSeconds", "Simulation duration (s)", simSeconds);
    cmd.AddValue("altKm", "LEO altitude (km)", altKm);
    cmd.AddValue("inclDeg", "Orbital inclination (deg)", inclDeg);
    cmd.AddValue("leadSeconds", "Ground-track lead between gs1 and gs2 (s)", leadSeconds);
    cmd.AddValue("minElevDeg", "GSL minimum elevation (deg)", minElevDeg);
    cmd.AddValue("dataRateMbps", "Offered UDP load (Mbps)", dataRateMbps);
    cmd.AddValue("packetBytes", "UDP payload (bytes)", packetBytes);
    cmd.AddValue("spaceCapMbps", "Space-link capacity (Mbps)", spaceCapMbps);
    cmd.AddValue("eirpDbm", "Space-link EIRP (dBm, tx power + antenna gain)", eirpDbm);
    cmd.AddValue("minSnrDb", "Decode threshold for the binary link-budget gate (dB)",
                 g_minSnrDb);
    cmd.Parse(argc, argv);

    const double a = kRe + altKm * 1000.0;
    const double n = std::sqrt(kMu / (a * a * a)); // mean motion, rad/s

    // --- LEO satellites under SGP4 ---------------------------------------
    // satA over gs1 at t=0; satB trails by leadSeconds so it reaches gs1 at
    // t=leadSeconds (a real make-before-break GSL handover under SGP4).
    auto mkElem = [&](uint32_t norad, double m0) {
        KeplerianElements e;
        e.semi_major_axis_m = a;
        e.eccentricity = 0.0;
        e.inclination_rad = inclDeg * M_PI / 180.0;
        e.raan_rad = 0.0;
        e.arg_perigee_rad = 0.0;
        e.mean_anomaly_rad = m0;
        e.epoch_unix_s = 0.0;
        e.norad_id = norad;
        return e;
    };

    Ptr<Sgp4MobilityModel> satA = CreateObject<Sgp4MobilityModel>();
    satA->SetElements(mkElem(kSatA, 0.0));
    Ptr<Sgp4MobilityModel> satB = CreateObject<Sgp4MobilityModel>();
    satB->SetElements(mkElem(kSatB, -n * leadSeconds));

    // gs1 = satA sub-point at t=0.
    double gs1Lat, gs1Lon, gs2Lat, gs2Lon;
    SubPoint(satA->GetEcefPosition(), gs1Lat, gs1Lon);
    // gs2 = satA sub-point leadSeconds later (temp model at advanced anomaly).
    Ptr<Sgp4MobilityModel> tmp = CreateObject<Sgp4MobilityModel>();
    tmp->SetElements(mkElem(99, n * leadSeconds));
    SubPoint(tmp->GetEcefPosition(), gs2Lat, gs2Lon);

    g_freqHz = 20e9;
    g_eirpDbm = eirpDbm;

    // --- contact graph ---------------------------------------------------
    Ptr<ContactGraphScheduler> sched = CreateObject<ContactGraphScheduler>();
    sched->SetSamplingInterval(Seconds(1.0));
    sched->SetMinElevationDeg(minElevDeg);
    sched->SetMaxIslRangeM(5'000'000.0);
    sched->RegisterSatellite(kSatA, satA);
    sched->RegisterSatellite(kSatB, satB);
    sched->RegisterGroundStation(kGs1, gs1Lat, gs1Lon);
    sched->RegisterGroundStation(kGs2, gs2Lat, gs2Lon);

    g_router = CreateObject<ContactGraphRouter>();
    g_router->Attach(sched);

    // --- ns-3 data plane -------------------------------------------------
    NodeContainer ue, uav, haps, nSatA, nSatB, gw, srv;
    ue.Create(1);
    uav.Create(1);
    haps.Create(1);
    nSatA.Create(1);
    nSatB.Create(1);
    gw.Create(1);
    srv.Create(1);

    InternetStackHelper internet;
    internet.Install(ue);
    internet.Install(uav);
    internet.Install(haps);
    internet.Install(nSatA);
    internet.Install(nSatB);
    internet.Install(gw);
    internet.Install(srv);

    Ipv4AddressHelper ipv4;

    // Access + egress segments (always up): UE-UAV-HAPS and GW-SRV.
    auto mkFixed = [&](Ptr<Node> x, Ptr<Node> y, double capMbps, double delayMs,
                       const std::string& subnet) -> Ipv4InterfaceContainer {
        PointToPointHelper p2p;
        p2p.SetDeviceAttribute(
            "DataRate", DataRateValue(DataRate(static_cast<uint64_t>(capMbps * 1e6))));
        p2p.SetChannelAttribute("Delay", TimeValue(MilliSeconds(delayMs)));
        NetDeviceContainer d = p2p.Install(x, y);
        ipv4.SetBase(Ipv4Address(subnet.c_str()), "255.255.255.0");
        return ipv4.Assign(d);
    };
    mkFixed(ue.Get(0), uav.Get(0), accessCapMbps, 1.0, "10.0.1.0");   // ground->UAV
    mkFixed(uav.Get(0), haps.Get(0), accessCapMbps, 2.0, "10.0.2.0"); // UAV->HAPS (A2G)
    Ipv4InterfaceContainer srvIf =
        mkFixed(gw.Get(0), srv.Get(0), accessCapMbps, 1.0, "10.0.9.0"); // GW->server

    // Five gated SPACE links.
    MakeSpaceLink(kSatA, kGs1, nSatA.Get(0), haps.Get(0), spaceCapMbps, ipv4, "10.1.1.0");
    MakeSpaceLink(kSatB, kGs1, nSatB.Get(0), haps.Get(0), spaceCapMbps, ipv4, "10.1.2.0");
    MakeSpaceLink(kSatA, kSatB, nSatA.Get(0), nSatB.Get(0), spaceCapMbps, ipv4, "10.1.3.0");
    MakeSpaceLink(kSatA, kGs2, nSatA.Get(0), gw.Get(0), spaceCapMbps, ipv4, "10.1.4.0");
    MakeSpaceLink(kSatB, kGs2, nSatB.Get(0), gw.Get(0), spaceCapMbps, ipv4, "10.1.5.0");

    Ipv4GlobalRoutingHelper::PopulateRoutingTables();

    // --- traffic: UE -> server (over whatever space path is up) ----------
    const uint16_t port = 7000;
    Ipv4Address srvAddr = srvIf.GetAddress(1); // server side of GW-SRV link
    g_sink = CreateObject<NtnOranSink>();
    g_sink->SetAttribute("Local",
                         AddressValue(InetSocketAddress(Ipv4Address::GetAny(), port)));
    srv.Get(0)->AddApplication(g_sink);
    g_sink->SetStartTime(Seconds(0.0));
    g_sink->SetStopTime(Seconds(simSeconds));

    Ptr<NtnOranApplication> src = CreateObject<NtnOranApplication>();
    src->SetRemote(InetSocketAddress(srvAddr, port));
    src->SetProfile(NtnOranApplication::CBR_SATURATING);
    src->SetAttribute("DataRate", DataRateValue(DataRate(uint64_t(dataRateMbps * 1e6))));
    src->SetAttribute("PacketSize", UintegerValue(packetBytes));
    src->SetFlowIdentity(/*5qi*/ 9, /*sst*/ 1, /*sd*/ 0x000001, /*src*/ 1, /*dst*/ 2);
    ue.Get(0)->AddApplication(src);
    src->SetStartTime(Seconds(1.0));
    src->SetStopTime(Seconds(simSeconds));

    // Drive the data plane from the contact graph.
    sched->m_contactUp.ConnectWithoutContext(MakeCallback(&ApplyContact));
    sched->m_contactDown.ConnectWithoutContext(MakeCallback(&ApplyContact));
    // SAGIN-1: transitions bring links in and out; updates keep the delay and
    // the budget of an established link tracking the pass.
    sched->m_contactUpdate.ConnectWithoutContext(MakeCallback(&OnContactUpdate));

    std::printf("# sagin-sgp4-routed-traffic (Roadmap §4.4.4/§4.4.5)\n");
    std::printf("#   sim=%.0fs alt=%.0fkm incl=%.0f lead=%.0fs minElev=%.0f load=%.1fMbps\n",
                simSeconds, altKm, inclDeg, leadSeconds, minElevDeg, dataRateMbps);
    std::printf("#   gs1(HAPS)=(%.2f,%.2f)  gs2(GW)=(%.2f,%.2f)  orbit n=%.4e rad/s\n",
                gs1Lat, gs1Lon, gs2Lat, gs2Lon, n);

    sched->Start();
    Simulator::Schedule(Seconds(2.0), &Report, simSeconds);
    Simulator::Stop(Seconds(simSeconds + 0.1));
    Simulator::Run();
    sched->Stop();

    const uint64_t rx = g_sink ? g_sink->GetTotalRx() : 0;
    std::printf("# === summary ===  GSL up/down=%lu/%lu  ISL up/down=%lu/%lu  "
                "rxBytes=%lu  avgGoodput=%.3f Mbps\n",
                (unsigned long)sched->GslEventsUp(), (unsigned long)sched->GslEventsDown(),
                (unsigned long)sched->IslEventsUp(), (unsigned long)sched->IslEventsDown(),
                (unsigned long)rx, rx * 8.0 / simSeconds / 1e6);
    std::printf("#   in-band measured: owd=%.3f ms jitter=%.3f ms loss=%.4f\n",
                g_sink->GetMeanDelayMs(), g_sink->GetMeanJitterMs(),
                g_sink->GetLossRatio());
    std::printf("#   router: edges added=%lu removed=%lu route-queries=%lu\n",
                (unsigned long)g_router->EdgesAddedTotal(),
                (unsigned long)g_router->EdgesRemovedTotal(),
                (unsigned long)g_router->RouteQueries());
    Simulator::Destroy();
    return 0;
}
