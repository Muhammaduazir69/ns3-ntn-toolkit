/*
 * SPDX-License-Identifier: GPL-2.0-only
 * Copyright (c) 2026 Muhammad Uzair (ns3-ntn-toolkit)
 *
 * sagin-multihop-traffic — REAL end-to-end packet forwarding up the SAGIN
 * stack Ground -> UAV -> HAPS -> {LEO-A | LEO-B} -> GW -> SRV, in which the
 * MultiLayerRouter's decision ACTUATES the data plane.
 *
 *   UE --- UAV --- HAPS ====[ space ]====> LEO-A ====[ space ]====> GW --- SRV
 *                    \\====[ space ]====> LEO-B ====[ space ]====//
 *
 * Two LEO satellites move under real SGP4 (projected into the scenario's local
 * ENU frame, phase-shifted by --leadSeconds), so the elevation each presents to
 * the HAPS crosses over during the pass. Every second the router is asked which
 * LEO to relay through (greedy max-elevation, with an honest below-horizon
 * feasibility gate). The chosen LEO's two space interfaces (HAPS-LEO and
 * LEO-GW) are brought UP — channel delay set from the true slant range, gated
 * on a binary link budget — while the OTHER LEO's interfaces are brought DOWN,
 * then Ipv4GlobalRoutingHelper::RecomputeRoutingTables() lets ns-3's own global
 * routing forward the live UDP flow over exactly the path the router picked.
 *
 * The router's decision therefore DETERMINES which physical path carries the
 * packets: when the SGP4 geometry flips the max-elevation LEO, the data plane
 * reroutes. The per-second log prints the GW-side bytes carried by EACH LEO
 * (from the P2P MacRx trace) next to the router's choice, so you can watch the
 * throughput migrate from LEO-A to LEO-B as the route switches — a hop that is
 * OFF the chosen route delivers ~0.
 *
 * MODELLING SCOPE / CAVEATS (read before interpreting PDR numbers):
 *   1. The per-link error is a BINARY in-contact gate (snr >= --minSnrDb =>
 *      PER 0, else PER 1), NOT a soft SNR->BLER decode curve. A link either
 *      closes its budget or drops every packet; there is no graceful
 *      degradation region. Partial-loss radio behaviour belongs to the
 *      real-stack (mmwave) examples; this example's contribution is real
 *      router-driven ROUTING actuation.
 *   2. The space-link budget uses FIXED EIRP (--eirpDbm) and FIXED noise, with
 *      only the +20 dB/decade free-space-path-loss frequency term (TR 38.811
 *      Eq. 6.6-2). There is NO rain/atmospheric/gaseous-absorption fade, so any
 *      frequency-driven outage is pure free-space FSPL, a worst-case
 *      illustration rather than a realistic Ka-band link-budget study.
 *
 * Quick test:  --simSeconds=60          (a route flip occurs mid-run)
 */
#include "ns3/applications-module.h"
#include "ns3/command-line.h"
#include "ns3/constant-position-mobility-model.h"
#include "ns3/core-module.h"
#include "ns3/error-model.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/ipv4-global-routing-helper.h"
#include "ns3/ipv4.h"
#include "ns3/ntn-oran-application.h"
#include "ns3/ntn-oran-sink.h"
#include "ns3/point-to-point-channel.h"
#include "ns3/point-to-point-helper.h"

#include "ns3/haps-mobility-model.h"
#include "ns3/multi-layer-router.h"
#include "ns3/ntn-tr38811-mobility-model.h"
#include "ns3/orbital-elements.h"
#include "ns3/sgp4-mobility-model.h"
#include "ns3/uav-mobility-models.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <vector>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("SaginMultihopTraffic");

namespace
{
constexpr double kC = 299792458.0;

// One ns-3 P2P link whose usability is driven from geometry. `gated` links are
// brought UP/DOWN by the router's route choice; non-gated (access/egress) links
// stay up but still have their delay + binary budget refreshed each tick.
struct GatedLink
{
    Ptr<Ipv4> ipA, ipB;
    uint32_t ifA = 0, ifB = 0;
    Ptr<RateErrorModel> emA, emB;
    Ptr<PointToPointChannel> chan;
    Ptr<MobilityModel> mobA, mobB;
    const char* tag = "";
};

// The two gated links a given LEO relay needs to carry the flow end to end.
struct LeoPath
{
    GatedLink hapsLeo; // HAPS <-> this LEO
    GatedLink leoGw;   // this LEO <-> GW
    Ptr<MobilityModel> leoMob;
    const char* tag = "";
};

std::vector<LeoPath> g_leoPaths;
std::vector<GatedLink> g_accessLinks; // UE-UAV, UAV-HAPS, GW-SRV (always up)
std::vector<uint64_t> g_carried;      // GW-side MacRx bytes per LEO path

Ptr<MultiLayerRouter> g_router;
Ptr<MobilityModel> g_ground;
Ptr<NtnOranSink> g_sink;
uint64_t g_lastRx = 0;

double g_eirpDbm = 70.0;
double g_noiseDbm = -95.0;
double g_minSnrDb = 3.0;
double g_freqHz = 2e9;

double
Dist(const Vector& a, const Vector& b)
{
    const double dx = a.x - b.x, dy = a.y - b.y, dz = a.z - b.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double
FsplDb(double dM, double fHz)
{
    return 20.0 * std::log10(std::max(dM, 1.0)) +
           20.0 * std::log10(fHz / 1e9) + 32.45;
}

// Honest binary link budget: the link either closes (clean decode) or drops
// every packet. No sigmoid PER.
bool
BudgetCloses(double rangeM)
{
    const double snr = g_eirpDbm - FsplDb(rangeM, g_freqHz) - g_noiseDbm;
    return snr >= g_minSnrDb;
}

// Refresh a link's channel delay + binary PER from the current geometry.
void
RefreshBudget(GatedLink& l)
{
    const double rng = Dist(l.mobA->GetPosition(), l.mobB->GetPosition());
    l.chan->SetAttribute("Delay", TimeValue(Seconds(rng / kC)));
    const double per = BudgetCloses(rng) ? 0.0 : 1.0;
    l.emA->SetRate(per);
    l.emB->SetRate(per);
}

// Bring a gated link UP (delay + budget from geometry) or DOWN.
void
SetGatedLink(GatedLink& l, bool on)
{
    if (on)
    {
        l.ipA->SetUp(l.ifA);
        l.ipB->SetUp(l.ifB);
        RefreshBudget(l);
    }
    else
    {
        l.ipA->SetDown(l.ifA);
        l.ipB->SetDown(l.ifB);
    }
}

// Actuate the router's chosen path onto the data plane: the LEO the router
// selected has its two space links brought UP, every other LEO's links DOWN,
// then global routing is recomputed so ns-3 forwards over the chosen path.
void
ApplyRoute(const std::vector<SaginHop>& path)
{
    Ptr<MobilityModel> chosenLeo;
    for (const auto& h : path)
    {
        if (h.layer == SaginLayer::Leo)
        {
            chosenLeo = h.node;
        }
    }
    for (auto& lp : g_leoPaths)
    {
        const bool on = (chosenLeo && lp.leoMob == chosenLeo);
        SetGatedLink(lp.hapsLeo, on);
        SetGatedLink(lp.leoGw, on);
    }
    Ipv4GlobalRoutingHelper::RecomputeRoutingTables();
}

void
SaginProbe(double simSeconds)
{
    const double now = Simulator::Now().GetSeconds();

    // Keep the always-up access/egress hops' delay + budget current.
    for (auto& l : g_accessLinks)
    {
        RefreshBudget(l);
    }

    // Ask the router which LEO to relay through, then actuate that decision.
    const auto path = g_router->Route(g_ground);
    Ptr<MobilityModel> chosenLeo;
    for (const auto& h : path)
    {
        if (h.layer == SaginLayer::Leo)
        {
            chosenLeo = h.node;
        }
    }
    int chosenIdx = -1;
    for (std::size_t i = 0; i < g_leoPaths.size(); ++i)
    {
        if (g_leoPaths[i].leoMob == chosenLeo)
        {
            chosenIdx = static_cast<int>(i);
        }
    }
    ApplyRoute(path);

    // Goodput this tick + per-LEO bytes actually carried at the GW (data-plane
    // truth measured from the P2P MacRx trace).
    const uint64_t tot = g_sink ? g_sink->GetTotalRx() : 0;
    const double mbps = (tot - g_lastRx) * 8.0 / 1e6;
    g_lastRx = tot;

    static std::vector<uint64_t> lastCarried(g_carried.size(), 0);
    double carA = (g_carried[0] - lastCarried[0]) * 8.0 / 1e6;
    double carB = (g_carried[1] - lastCarried[1]) * 8.0 / 1e6;
    lastCarried[0] = g_carried[0];
    lastCarried[1] = g_carried[1];

    const char* chosenTag =
        (chosenIdx < 0) ? "(none)" : g_leoPaths[chosenIdx].tag;

    std::printf("  %6.1f   route=%-7s   viaA=%7.3f   viaB=%7.3f   e2e=%7.3f Mbps\n",
                now, chosenTag, carA, carB, mbps);

    if (now + 1.0 < simSeconds)
    {
        Simulator::Schedule(Seconds(1.0), &SaginProbe, simSeconds);
    }
}

void
CountCarried(uint32_t idx, Ptr<const Packet> p)
{
    g_carried[idx] += p->GetSize();
}

// Build one geometry-driven P2P link between two nodes and record its handles.
// `startDown` leaves the interfaces down (gated space links start down; the
// first route decision brings the chosen one up).
GatedLink
MakeLink(Ptr<Node> a,
         Ptr<Node> b,
         Ptr<MobilityModel> mobA,
         Ptr<MobilityModel> mobB,
         double capMbps,
         const std::string& subnet,
         bool startDown,
         const char* tag,
         Ipv4AddressHelper& ipv4,
         NetDeviceContainer& outDev)
{
    PointToPointHelper p2p;
    p2p.SetDeviceAttribute(
        "DataRate", DataRateValue(DataRate(static_cast<uint64_t>(capMbps * 1e6))));
    p2p.SetChannelAttribute("Delay", TimeValue(MicroSeconds(100))); // per-tick
    NetDeviceContainer dev = p2p.Install(NodeContainer(a, b));
    outDev = dev;

    GatedLink l;
    l.tag = tag;
    l.mobA = mobA;
    l.mobB = mobB;
    l.emA = CreateObject<RateErrorModel>();
    l.emA->SetUnit(RateErrorModel::ERROR_UNIT_PACKET);
    l.emA->SetRate(startDown ? 1.0 : 0.0);
    l.emB = CreateObject<RateErrorModel>();
    l.emB->SetUnit(RateErrorModel::ERROR_UNIT_PACKET);
    l.emB->SetRate(startDown ? 1.0 : 0.0);
    dev.Get(0)->SetAttribute("ReceiveErrorModel", PointerValue(l.emA));
    dev.Get(1)->SetAttribute("ReceiveErrorModel", PointerValue(l.emB));
    l.chan = DynamicCast<PointToPointChannel>(dev.Get(0)->GetChannel());

    ipv4.SetBase(Ipv4Address(subnet.c_str()), "255.255.255.0");
    Ipv4InterfaceContainer ifc = ipv4.Assign(dev);
    l.ipA = a->GetObject<Ipv4>();
    l.ipB = b->GetObject<Ipv4>();
    l.ifA = ifc.Get(0).second;
    l.ifB = ifc.Get(1).second;
    if (startDown)
    {
        l.ipA->SetDown(l.ifA);
        l.ipB->SetDown(l.ifB);
    }
    return l;
}
} // namespace

int
main(int argc, char* argv[])
{
    double simSeconds = 120.0;
    double freqGHz = 2.0;
    double dataRateMbps = 5.0;
    uint32_t packetBytes = 1200;
    double uavAltM = 120.0;
    double hapsAltKm = 20.0;
    double leoAltKm = 550.0;
    double leadSeconds = 45.0; // ground-track phasing between LEO-A and LEO-B
    double inclDeg = 53.0;
    double minElevDeg = 5.0;
    double linkCapacityMbps = 50.0;
    double eirpDbm = 70.0;

    CommandLine cmd(__FILE__);
    cmd.AddValue("simSeconds", "Simulation duration (s)", simSeconds);
    cmd.AddValue("freqGHz", "Carrier frequency (GHz)", freqGHz);
    cmd.AddValue("dataRateMbps", "Offered end-to-end load (Mbps)", dataRateMbps);
    cmd.AddValue("packetBytes", "UDP payload size (bytes)", packetBytes);
    cmd.AddValue("uavAltM", "UAV altitude AGL (m)", uavAltM);
    cmd.AddValue("hapsAltKm", "HAPS altitude (km)", hapsAltKm);
    cmd.AddValue("leoAltKm", "LEO altitude (km)", leoAltKm);
    cmd.AddValue("leadSeconds", "Ground-track phasing between LEO-A/LEO-B (s)",
                 leadSeconds);
    cmd.AddValue("inclDeg", "Orbital inclination (deg)", inclDeg);
    cmd.AddValue("minElevDeg", "Router min elevation for a feasible LEO (deg)",
                 minElevDeg);
    cmd.AddValue("linkCapacityMbps", "Per-hop P2P capacity (Mbps)",
                 linkCapacityMbps);
    cmd.AddValue("eirpDbm", "Per-hop EIRP (dBm)", eirpDbm);
    cmd.AddValue("minSnrDb", "Decode threshold for the binary link budget (dB)",
                 g_minSnrDb);
    cmd.Parse(argc, argv);

    g_eirpDbm = eirpDbm;
    g_freqHz = freqGHz * 1e9;

    // --- Two LEO satellites under real SGP4, projected into local ENU --------
    // LEO-A is overhead the ENU reference at t=0; LEO-B trails by leadSeconds
    // and rises to overhead at t=leadSeconds, so the elevation each presents to
    // the HAPS crosses over mid-pass and the router flips its choice.
    const double kMu = 3.986004418e14;
    const double kRe = 6371000.0;
    const double aSma = kRe + leoAltKm * 1000.0;
    const double meanMotion = std::sqrt(kMu / (aSma * aSma * aSma)); // rad/s

    auto mkElem = [&](uint32_t norad, double m0) {
        ns3::ntncon::KeplerianElements e;
        e.semi_major_axis_m = aSma;
        e.eccentricity = 0.0;
        e.inclination_rad = inclDeg * M_PI / 180.0;
        e.raan_rad = 0.0;
        e.arg_perigee_rad = 0.0;
        e.mean_anomaly_rad = m0;
        e.epoch_unix_s = 0.0;
        e.norad_id = norad;
        return e;
    };

    Ptr<ns3::ntncon::Sgp4MobilityModel> satA =
        CreateObject<ns3::ntncon::Sgp4MobilityModel>();
    satA->SetElements(mkElem(10, 0.0));
    Ptr<ns3::ntncon::Sgp4MobilityModel> satB =
        CreateObject<ns3::ntncon::Sgp4MobilityModel>();
    satB->SetElements(mkElem(11, -meanMotion * leadSeconds));

    double refLat, refLon, refAlt;
    satA->GetGeodetic(refLat, refLon, refAlt); // ENU origin = LEO-A subpoint @ t=0

    Ptr<NtnEnuProjectionMobilityModel> leoA =
        CreateObject<NtnEnuProjectionMobilityModel>();
    leoA->SetSource(satA);
    leoA->SetReference(refLat, refLon, 0.0);
    Ptr<NtnEnuProjectionMobilityModel> leoB =
        CreateObject<NtnEnuProjectionMobilityModel>();
    leoB->SetSource(satB);
    leoB->SetReference(refLat, refLon, 0.0);

    // --- ns-3 nodes ----------------------------------------------------------
    // 0=UE 1=UAV 2=HAPS 3=LEO-A 4=LEO-B 5=GW 6=SRV
    NodeContainer nodes;
    nodes.Create(7);

    Ptr<ConstantPositionMobilityModel> ground =
        CreateObject<ConstantPositionMobilityModel>();
    ground->SetPosition(Vector(0, 0, 0));
    nodes.Get(0)->AggregateObject(ground);
    g_ground = ground;

    Ptr<UavPatrolMobilityModel> uav = CreateObject<UavPatrolMobilityModel>();
    uav->SetAttribute("Speed", DoubleValue(30.0));
    // Keep the ground->UAV elevation above the router's feasibility floor
    // across the whole patrol (atan2(120, 700) ~ 9.7 deg > minElevDeg) so the
    // access hop never truncates the path; the LEO layer is what reroutes.
    uav->SetEndpoints(Vector(-700, 0, uavAltM), Vector(700, 0, uavAltM));
    nodes.Get(1)->AggregateObject(uav);

    Ptr<HapsMobilityModel> haps = CreateObject<HapsMobilityModel>();
    haps->SetCenter(Vector(0, 0, hapsAltKm * 1000.0));
    nodes.Get(2)->AggregateObject(haps);

    nodes.Get(3)->AggregateObject(leoA);
    nodes.Get(4)->AggregateObject(leoB);

    Ptr<ConstantPositionMobilityModel> gw =
        CreateObject<ConstantPositionMobilityModel>();
    gw->SetPosition(Vector(0, 0, 0)); // ground gateway at the ENU origin
    nodes.Get(5)->AggregateObject(gw);
    Ptr<ConstantPositionMobilityModel> srvMob =
        CreateObject<ConstantPositionMobilityModel>();
    srvMob->SetPosition(Vector(10, 0, 0));
    nodes.Get(6)->AggregateObject(srvMob);

    // --- routing engine: two LEO candidates the router chooses between -------
    g_router = CreateObject<MultiLayerRouter>();
    g_router->SetMinElevationDeg(minElevDeg); // below-horizon LEOs are infeasible
    g_router->AddNode(SaginLayer::Uav, uav);
    g_router->AddNode(SaginLayer::Haps, haps);
    g_router->AddNode(SaginLayer::Leo, leoA);
    g_router->AddNode(SaginLayer::Leo, leoB);

    InternetStackHelper internet;
    internet.Install(nodes);

    Ipv4AddressHelper ipv4;
    NetDeviceContainer d;

    // Access + egress (always up): UE-UAV, UAV-HAPS, GW-SRV.
    g_accessLinks.push_back(MakeLink(nodes.Get(0), nodes.Get(1), ground, uav,
                                     linkCapacityMbps, "10.20.1.0", false, "ue-uav",
                                     ipv4, d));
    g_accessLinks.push_back(MakeLink(nodes.Get(1), nodes.Get(2), uav, haps,
                                     linkCapacityMbps, "10.20.2.0", false, "uav-haps",
                                     ipv4, d));
    GatedLink gwSrv = MakeLink(nodes.Get(5), nodes.Get(6), gw, srvMob,
                               linkCapacityMbps, "10.20.9.0", false, "gw-srv", ipv4, d);
    g_accessLinks.push_back(gwSrv);
    // Server address (stable dst reached over whichever LEO is chosen).
    Ipv4Address srvAddr = nodes.Get(6)->GetObject<Ipv4>()->GetAddress(
                                    gwSrv.ifB, 0).GetLocal();

    // Gated space links, two per LEO relay (start down).
    g_carried.assign(2, 0);
    const char* leoTags[2] = {"LEO-A", "LEO-B"};
    Ptr<MobilityModel> leoMobs[2] = {leoA, leoB};
    Ptr<Node> leoNodes[2] = {nodes.Get(3), nodes.Get(4)};
    const char* hlSubnet[2] = {"10.30.1.0", "10.30.2.0"};
    const char* lgSubnet[2] = {"10.31.1.0", "10.31.2.0"};
    for (int i = 0; i < 2; ++i)
    {
        LeoPath lp;
        lp.tag = leoTags[i];
        lp.leoMob = leoMobs[i];
        lp.hapsLeo = MakeLink(nodes.Get(2), leoNodes[i], haps, leoMobs[i],
                              linkCapacityMbps, hlSubnet[i], true, "haps-leo", ipv4, d);
        NetDeviceContainer lgDev;
        lp.leoGw = MakeLink(leoNodes[i], nodes.Get(5), leoMobs[i], gw,
                            linkCapacityMbps, lgSubnet[i], true, "leo-gw", ipv4, lgDev);
        // Count bytes the GW actually receives from THIS LEO (GW is side 1).
        lgDev.Get(1)->TraceConnectWithoutContext(
            "MacRx", MakeBoundCallback(&CountCarried, static_cast<uint32_t>(i)));
        g_leoPaths.push_back(lp);
    }

    Ipv4GlobalRoutingHelper::PopulateRoutingTables();

    // --- traffic: UE -> SRV over whichever LEO the router selects ------------
    const uint16_t port = 7000;
    g_sink = CreateObject<NtnOranSink>();
    g_sink->SetAttribute("Local",
                         AddressValue(InetSocketAddress(Ipv4Address::GetAny(), port)));
    nodes.Get(6)->AddApplication(g_sink);
    g_sink->SetStartTime(Seconds(0.0));
    g_sink->SetStopTime(Seconds(simSeconds));

    Ptr<NtnOranApplication> client = CreateObject<NtnOranApplication>();
    client->SetRemote(InetSocketAddress(srvAddr, port));
    client->SetProfile(NtnOranApplication::CBR_SATURATING);
    client->SetAttribute(
        "DataRate",
        DataRateValue(DataRate(static_cast<uint64_t>(dataRateMbps * 1e6))));
    client->SetAttribute("PacketSize", UintegerValue(packetBytes));
    client->SetFlowIdentity(/*5qi*/ 9, /*sst*/ 1, /*sd*/ 0x000001, /*src*/ 0, /*dst*/ 6);
    nodes.Get(0)->AddApplication(client);
    client->SetStartTime(Seconds(2.0));
    client->SetStopTime(Seconds(simSeconds));

    std::printf("# sagin-multihop-traffic (Ground->UAV->HAPS->{LEO-A|LEO-B}->GW->SRV)\n");
    std::printf("#   sim=%.0fs freq=%.1fGHz load=%.1fMbps lead=%.0fs minElev=%.0f "
                "eirp=%.0fdBm\n",
                simSeconds, freqGHz, dataRateMbps, leadSeconds, minElevDeg, eirpDbm);
    std::printf("#   ENU ref (LEO-A subpoint @ t=0) = (%.2f, %.2f)  orbit n=%.4e rad/s\n",
                refLat, refLon, meanMotion);
    std::printf("#   'route' = router's chosen LEO; viaA/viaB = Mbps the GW "
                "actually receives from each LEO (data-plane truth)\n");
    std::printf("# %5s   %-13s %-13s %-13s %s\n",
                "t_s", "route", "viaA(GW)", "viaB(GW)", "e2e");

    Simulator::Schedule(Seconds(1.0), &SaginProbe, simSeconds);
    Simulator::Stop(Seconds(simSeconds + 0.1));
    Simulator::Run();

    const uint64_t txP = client->GetTxPackets();
    const uint64_t rxP = g_sink->GetRxPackets();
    const uint64_t totalRx = g_sink->GetTotalRx();
    std::printf("# === summary ===  txPackets=%lu rxPackets=%lu PDR=%.2f%% "
                "meanDelay=%.3fms jitter=%.3fms avgGoodput=%.3f Mbps\n",
                (unsigned long)txP, (unsigned long)rxP,
                txP ? 100.0 * rxP / txP : 0.0, g_sink->GetMeanDelayMs(),
                g_sink->GetMeanJitterMs(), totalRx * 8.0 / simSeconds / 1e6);
    std::printf("#   carried at GW:  via LEO-A=%lu bytes   via LEO-B=%lu bytes "
                "(sum should track the route history)\n",
                (unsigned long)g_carried[0], (unsigned long)g_carried[1]);
    Simulator::Destroy();
    return 0;
}
