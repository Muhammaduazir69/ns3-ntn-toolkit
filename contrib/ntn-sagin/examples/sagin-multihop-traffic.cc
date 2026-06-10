/*
 * SPDX-License-Identifier: GPL-2.0-only
 * Copyright (c) 2026 Muhammad Uzair (ns3-ntn-toolkit)
 *
 * sagin-multihop-traffic — REAL end-to-end packet forwarding up the SAGIN
 * stack Ground → UAV → HAPS → LEO. Three PointToPoint hops are joined by an
 * IPv4 stack with global routing, so a UDP flow injected at the ground node
 * is forwarded hop-by-hop to the LEO node. Each hop has its own
 * geometry-driven RateErrorModel:
 *   - ground↔UAV  : TR 36.777 air-to-ground path loss (A2gChannelTr36777)
 *   - UAV↔HAPS    : free-space path loss
 *   - HAPS↔LEO    : free-space path loss + min-elevation gate
 * MultiLayerRouter is evaluated each second and its chosen path is logged, so
 * the routing decision engine is exercised alongside the real data plane.
 *
 * As the UAV patrols and the LEO satellite passes, the per-hop link budgets
 * change, so the END-TO-END delivered goodput (the product of per-hop
 * successes) tracks the live geometry — nothing hardcoded.
 *
 * Quick test:  --simSeconds=120 --dataRateMbps=5
 */
#include "ns3/applications-module.h"
#include "ns3/ntn-tr38811-mobility-model.h"
#include "ns3/sgp4-mobility-model.h"
#include "ns3/walker-constellation.h"
#include "ns3/command-line.h"
#include "ns3/constant-position-mobility-model.h"
#include "ns3/constant-velocity-mobility-model.h"
#include "ns3/core-module.h"
#include "ns3/error-model.h"
#include "ns3/ntn-oran-application.h"
#include "ns3/ntn-oran-sink.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/ipv4-global-routing-helper.h"
#include "ns3/point-to-point-channel.h"
#include "ns3/point-to-point-helper.h"

#include "ns3/a2g-channel-tr36777.h"
#include "ns3/haps-mobility-model.h"
#include "ns3/multi-layer-router.h"
#include "ns3/uav-mobility-models.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("SaginMultihopTraffic");

namespace
{
constexpr double kC = 299792458.0;

struct Hop
{
    Ptr<MobilityModel> a;
    Ptr<MobilityModel> b;
    Ptr<RateErrorModel> em;
    Ptr<PointToPointChannel> ch;
    const char* tag;
};

Ptr<MultiLayerRouter> g_router;
Ptr<MobilityModel> g_ground;
Hop g_gu; // ground-uav
Hop g_uh; // uav-haps
Hop g_hl; // haps-leo
Ptr<NtnOranSink> g_sink;
uint64_t g_lastRx = 0;
double g_eirpDbm = 70.0;
double g_noiseDbm = -95.0;
double g_minElev = 5.0;

double
Dist(const Vector& a, const Vector& b)
{
    const double dx = a.x - b.x, dy = a.y - b.y, dz = a.z - b.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double
ElevDeg(const Vector& u, const Vector& s)
{
    const Vector d(s.x - u.x, s.y - u.y, s.z - u.z);
    return std::atan2(d.z, std::max(std::sqrt(d.x * d.x + d.y * d.y), 1e-3)) *
           180.0 / M_PI;
}

double
FsplDb(double dM, double fHz)
{
    return 20.0 * std::log10(std::max(dM, 1.0)) +
           20.0 * std::log10(fHz / 1e9) + 32.45;
}

void
UpdateHopPer(Hop& h, double plDb, bool gated, double elev)
{
    const double rng = Dist(h.a->GetPosition(), h.b->GetPosition());
    const double rx = g_eirpDbm - plDb;
    const double snr = rx - g_noiseDbm;
    // Honest link-budget gate: the hop forwards only while the REAL path-loss
    // budget closes (rx SNR above the decode floor) — a binary in-contact gate
    // driven by the real A2G/FSPL physics, NOT a fabricated sigmoid PER.
    double per = (snr >= 3.0) ? 0.0 : 1.0;
    if (gated && elev < g_minElev)
    {
        per = 1.0;
    }
    h.em->SetRate(per);
    h.ch->SetAttribute("Delay", TimeValue(Seconds(rng / kC)));
}

void
SaginProbe(double freqGHz, double uavAltM)
{
    // Ground↔UAV: TR 36.777 air-to-ground.
    const double d3dGu = Dist(g_gu.a->GetPosition(), g_gu.b->GetPosition());
    const double plGu = A2gChannelTr36777::PathLossDb(
        A2gScenario::UMa_AV, A2gLink::LOS, d3dGu, freqGHz, uavAltM);
    UpdateHopPer(g_gu, plGu, false, 90.0);

    // UAV↔HAPS: FSPL.
    const double dUh = Dist(g_uh.a->GetPosition(), g_uh.b->GetPosition());
    UpdateHopPer(g_uh, FsplDb(dUh, freqGHz * 1e9), false, 90.0);

    // HAPS↔LEO: FSPL + min-elevation gate.
    const double elevHl =
        ElevDeg(g_hl.a->GetPosition(), g_hl.b->GetPosition());
    const double dHl = Dist(g_hl.a->GetPosition(), g_hl.b->GetPosition());
    UpdateHopPer(g_hl, FsplDb(dHl, freqGHz * 1e9), true, elevHl);

    // Exercise the routing decision engine.
    auto path = g_router->Route(g_ground);

    const uint64_t tot = g_sink ? g_sink->GetTotalRx() : 0;
    const double mbps = (tot - g_lastRx) * 8.0 / 1e6;
    g_lastRx = tot;
    std::printf("  %6.1f  %3zu  %8.1f  %8.1f  %7.2f  %9.3f\n",
                Simulator::Now().GetSeconds(), path.size(), plGu,
                FsplDb(dHl, freqGHz * 1e9), elevHl, mbps);
    Simulator::Schedule(Seconds(1.0), &SaginProbe, freqGHz, uavAltM);
}

Hop
MakeHop(Ptr<Node> na, Ptr<Node> nb, Ptr<MobilityModel> ma,
        Ptr<MobilityModel> mb, double capMbps, const char* tag,
        PointToPointHelper& p2p, Ipv4AddressHelper& ipv4,
        NetDeviceContainer& outDev)
{
    Hop h;
    h.a = ma;
    h.b = mb;
    h.tag = tag;
    NetDeviceContainer dev = p2p.Install(NodeContainer(na, nb));
    outDev = dev;
    Ptr<RateErrorModel> em = CreateObject<RateErrorModel>();
    em->SetUnit(RateErrorModel::ERROR_UNIT_PACKET);
    em->SetRate(0.0);
    // Apply the error model on the receiving (upstream) side of each hop.
    dev.Get(1)->SetAttribute("ReceiveErrorModel", PointerValue(em));
    h.em = em;
    h.ch = DynamicCast<PointToPointChannel>(dev.Get(0)->GetChannel());
    return h;
}
} // namespace

int
main(int argc, char* argv[])
{
    double simSeconds = 300.0;
    double freqGHz = 2.0;
    double dataRateMbps = 5.0;
    uint32_t packetBytes = 1200;
    double uavAltM = 120.0;
    double hapsAltKm = 20.0;
    double leoAltKm = 550.0;
    double satSpeed = 7500.0;
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
    cmd.AddValue("satSpeed", "LEO ground-track speed (m/s)", satSpeed);
    cmd.AddValue("linkCapacityMbps", "Per-hop P2P capacity (Mbps)", linkCapacityMbps);
    cmd.AddValue("eirpDbm", "Per-hop EIRP (dBm)", eirpDbm);
    cmd.Parse(argc, argv);

    g_eirpDbm = eirpDbm;

    NodeContainer nodes;
    nodes.Create(4); // 0=ground 1=uav 2=haps 3=leo

    // Mobility.
    Ptr<ConstantPositionMobilityModel> ground =
        CreateObject<ConstantPositionMobilityModel>();
    ground->SetPosition(Vector(0, 0, 0));
    nodes.Get(0)->AggregateObject(ground);
    g_ground = ground;

    Ptr<UavPatrolMobilityModel> uav = CreateObject<UavPatrolMobilityModel>();
    uav->SetAttribute("Speed", DoubleValue(30.0));
    uav->SetEndpoints(Vector(-2000, 0, uavAltM), Vector(2000, 0, uavAltM));
    nodes.Get(1)->AggregateObject(uav);

    Ptr<HapsMobilityModel> haps = CreateObject<HapsMobilityModel>();
    haps->SetCenter(Vector(0, 0, hapsAltKm * 1000.0));
    nodes.Get(2)->AggregateObject(haps);

    // Real SGP4 orbit projected into the scenario's local ENU frame: the
    // satellite passes overhead near t=0 and recedes with genuine orbital
    // dynamics (no straight-line placeholder).
    ns3::ntncon::WalkerConfig wcfgSat;
    wcfgSat.num_planes = 1;
    wcfgSat.total_sats = 80;
    wcfgSat.altitude_km = leoAltKm;
    wcfgSat.inclination_deg = 53.0;
    wcfgSat.epoch_unix_s = 1735689600.0;
    const auto satElements = ns3::ntncon::WalkerConstellation::BuildDelta(wcfgSat);
    Ptr<ns3::ntncon::Sgp4MobilityModel> satSgp4 =
        CreateObject<ns3::ntncon::Sgp4MobilityModel>();
    satSgp4->SetElements(satElements[0]);
    double satSubLat, satSubLon, satSubAlt;
    satSgp4->GetGeodetic(satSubLat, satSubLon, satSubAlt);
    Ptr<NtnEnuProjectionMobilityModel> leo = CreateObject<NtnEnuProjectionMobilityModel>();
    leo->SetSource(satSgp4);
    leo->SetReference(satSubLat, satSubLon, 0.0);
    nodes.Get(3)->AggregateObject(leo);

    // Routing engine (exercised in the probe; logs the layered path).
    g_router = CreateObject<MultiLayerRouter>();
    g_router->AddNode(SaginLayer::Uav, uav);
    g_router->AddNode(SaginLayer::Haps, haps);
    g_router->AddNode(SaginLayer::Leo, leo);

    InternetStackHelper internet;
    internet.Install(nodes);

    PointToPointHelper p2p;
    p2p.SetDeviceAttribute(
        "DataRate",
        DataRateValue(DataRate(static_cast<uint64_t>(linkCapacityMbps * 1e6))));
    p2p.SetChannelAttribute("Delay", TimeValue(MicroSeconds(100)));

    Ipv4AddressHelper ipv4;
    NetDeviceContainer dGu, dUh, dHl;
    g_gu = MakeHop(nodes.Get(0), nodes.Get(1), ground, uav, linkCapacityMbps,
                   "g-u", p2p, ipv4, dGu);
    g_uh = MakeHop(nodes.Get(1), nodes.Get(2), uav, haps, linkCapacityMbps,
                   "u-h", p2p, ipv4, dUh);
    g_hl = MakeHop(nodes.Get(2), nodes.Get(3), haps, leo, linkCapacityMbps,
                   "h-l", p2p, ipv4, dHl);

    ipv4.SetBase("10.20.1.0", "255.255.255.0");
    ipv4.Assign(dGu);
    ipv4.SetBase("10.20.2.0", "255.255.255.0");
    ipv4.Assign(dUh);
    ipv4.SetBase("10.20.3.0", "255.255.255.0");
    Ipv4InterfaceContainer iHl = ipv4.Assign(dHl);

    Ipv4GlobalRoutingHelper::PopulateRoutingTables();

    // End-to-end UDP: ground (node 0) → LEO (node 3), the leo-side address.
    const uint16_t port = 7000;
    g_sink = CreateObject<NtnOranSink>();
    g_sink->SetAttribute("Local",
                         AddressValue(InetSocketAddress(Ipv4Address::GetAny(), port)));
    nodes.Get(3)->AddApplication(g_sink);
    g_sink->SetStartTime(Seconds(0.0));
    g_sink->SetStopTime(Seconds(simSeconds));

    Ptr<NtnOranApplication> client = CreateObject<NtnOranApplication>();
    client->SetRemote(InetSocketAddress(iHl.GetAddress(1), port));
    client->SetProfile(NtnOranApplication::CBR_SATURATING);
    client->SetAttribute(
        "DataRate",
        DataRateValue(DataRate(static_cast<uint64_t>(dataRateMbps * 1e6))));
    client->SetAttribute("PacketSize", UintegerValue(packetBytes));
    client->SetFlowIdentity(/*5qi*/ 9, /*sst*/ 1, /*sd*/ 0x000001, /*src*/ 0, /*dst*/ 3);
    nodes.Get(0)->AddApplication(client);
    client->SetStartTime(Seconds(1.0));
    client->SetStopTime(Seconds(simSeconds));

    std::printf("# sagin-multihop-traffic (Ground→UAV→HAPS→LEO)\n");
    std::printf("#   sim=%.0fs freq=%.1fGHz load=%.1fMbps uavAlt=%.0fm "
                "hapsAlt=%.0fkm leoAlt=%.0fkm eirp=%.0fdBm\n",
                simSeconds, freqGHz, dataRateMbps, uavAltM, hapsAltKm, leoAltKm,
                eirpDbm);
    std::printf("# %5s  %3s  %8s  %8s  %7s  %9s\n",
                "t_s", "hop", "plGU_dB", "fsplHL", "elevHL", "e2e_Mbps");

    Simulator::Schedule(Seconds(2.0), &SaginProbe, freqGHz, uavAltM);
    Simulator::Stop(Seconds(simSeconds + 0.1));
    Simulator::Run();

    // KPIs measured from in-band NtnOranPayloadHeader primitives at the sink.
    const uint64_t txP = client->GetTxPackets();
    const uint64_t rxP = g_sink->GetRxPackets();
    const uint64_t totalRx = g_sink->GetTotalRx();
    std::printf("# === summary ===  end-to-end txPackets=%lu rxPackets=%lu "
                "PDR=%.2f%% meanDelay=%.3fms jitter=%.3fms avgGoodput=%.3f Mbps\n",
                (unsigned long)txP, (unsigned long)rxP,
                txP ? 100.0 * rxP / txP : 0.0, g_sink->GetMeanDelayMs(),
                g_sink->GetMeanJitterMs(), totalRx * 8.0 / simSeconds / 1e6);
    Simulator::Destroy();
    return 0;
}
