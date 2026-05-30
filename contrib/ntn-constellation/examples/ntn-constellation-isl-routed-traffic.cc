/*
 * SPDX-License-Identifier: GPL-2.0-only
 * Copyright (c) 2026 Muhammad Uzair (ns3-ntn-toolkit)
 *
 * ntn-constellation-isl-routed-traffic — REAL end-to-end packet forwarding
 * between two ground stations across a LEO inter-satellite-link path:
 *
 *     GS1 --uplink--> satA --ISL--> satB --downlink--> GS2
 *
 * The two satellites move on SGP4-propagated orbits (Sgp4MobilityModel from
 * Keplerian elements). A ContactGraphScheduler samples GS↔sat visibility and
 * sat↔sat ISL ranges; a ContactGraphRouter computes the shortest path through
 * the contact graph each second (logged). The data plane is the matching
 * 3-hop PointToPoint chain with IPv4 global routing; each hop's RateErrorModel
 * is gated by the live SGP4 geometry (GSL elevation, ISL range), so when a
 * satellite drops below the horizon or the ISL stretches past range the
 * end-to-end goodput collapses. Everything tracks the orbital geometry —
 * nothing hardcoded.
 *
 * Quick test:  --simSeconds=120 --dataRateMbps=5
 */
#include "ns3/applications-module.h"
#include "ns3/command-line.h"
#include "ns3/constant-position-mobility-model.h"
#include "ns3/core-module.h"
#include "ns3/flow-monitor-helper.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/ipv4-global-routing-helper.h"
#include "ns3/error-model.h"
#include "ns3/point-to-point-channel.h"
#include "ns3/point-to-point-helper.h"

#include "ns3/contact-graph-router.h"
#include "ns3/contact-graph-scheduler.h"
#include "ns3/orbital-elements.h"
#include "ns3/sgp4-mobility-model.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

using namespace ns3;
using namespace ns3::ntncon;

NS_LOG_COMPONENT_DEFINE("NtnConstellationIslRoutedTraffic");

namespace
{
constexpr double kC = 299792458.0;
constexpr double kReM = 6371000.0;
constexpr double kMu = 3.986004418e14; // GM (m^3/s^2)

Ptr<ContactGraphRouter> g_router;
Ptr<MobilityModel> g_gs1, g_satA, g_satB, g_gs2;
Ptr<RateErrorModel> g_emUp, g_emIsl, g_emDown;
Ptr<PointToPointChannel> g_chUp, g_chIsl, g_chDown;
Ptr<PacketSink> g_sink;
uint64_t g_lastRx = 0;
double g_eirpDbm = 90.0;
double g_freqHz = 2.0e9;
double g_noiseDbm = -95.0;
double g_minElev = 10.0;
double g_maxIslRangeM = 5.0e6;
uint32_t g_gs1Id = 100, g_gs2Id = 101;

double
Dist(const Vector& a, const Vector& b)
{
    const double dx = a.x - b.x, dy = a.y - b.y, dz = a.z - b.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

// Elevation of `sat` from a ground point `gs`, both in ECEF; up = radial.
double
ElevEcefDeg(const Vector& gs, const Vector& sat)
{
    const double r = std::sqrt(gs.x * gs.x + gs.y * gs.y + gs.z * gs.z);
    const double ux = gs.x / r, uy = gs.y / r, uz = gs.z / r;
    const double lx = sat.x - gs.x, ly = sat.y - gs.y, lz = sat.z - gs.z;
    const double ln = std::max(1e-3, std::sqrt(lx * lx + ly * ly + lz * lz));
    double cosZ = (lx * ux + ly * uy + lz * uz) / ln;
    cosZ = std::clamp(cosZ, -1.0, 1.0);
    return 90.0 - std::acos(cosZ) * 180.0 / M_PI;
}

double
FsplDb(double dM, double fHz)
{
    return 20.0 * std::log10(std::max(dM, 1.0)) +
           20.0 * std::log10(fHz / 1e9) + 32.45;
}

double
SnrToPer(double snrDb)
{
    return 1.0 / (1.0 + std::exp(0.8 * (snrDb - 6.0)));
}

void
GateHop(Ptr<RateErrorModel> em, Ptr<PointToPointChannel> ch, double rng,
        bool usable)
{
    const double rx = g_eirpDbm - FsplDb(rng, g_freqHz);
    const double snr = rx - g_noiseDbm;
    em->SetRate(usable ? SnrToPer(snr) : 1.0);
    ch->SetAttribute("Delay", TimeValue(Seconds(rng / kC)));
}

void
Tick()
{
    const Vector g1 = g_gs1->GetPosition();
    const Vector sa = g_satA->GetPosition();
    const Vector sb = g_satB->GetPosition();
    const Vector g2 = g_gs2->GetPosition();

    const double elUp = ElevEcefDeg(g1, sa);
    const double elDn = ElevEcefDeg(g2, sb);
    const double dUp = Dist(g1, sa);
    const double dIsl = Dist(sa, sb);
    const double dDn = Dist(g2, sb);

    GateHop(g_emUp, g_chUp, dUp, elUp >= g_minElev);
    GateHop(g_emIsl, g_chIsl, dIsl, dIsl <= g_maxIslRangeM);
    GateHop(g_emDown, g_chDown, dDn, elDn >= g_minElev);

    // Exercise the contact-graph router: shortest path GS1 -> GS2.
    auto path = g_router->ShortestPath(g_gs1Id, g_gs2Id);

    const uint64_t tot = g_sink ? g_sink->GetTotalRx() : 0;
    const double mbps = (tot - g_lastRx) * 8.0 / 1e6;
    g_lastRx = tot;
    std::printf("  %6.1f  elUp=%6.1f  islRange=%8.0fkm  elDn=%6.1f  "
                "cgPathHops=%2zu  goodput=%8.3f\n",
                Simulator::Now().GetSeconds(), elUp, dIsl / 1000.0, elDn,
                path.size(), mbps);
    Simulator::Schedule(Seconds(1.0), &Tick);
}

KeplerianElements
MakeLeo(double meanAnomalyDeg, double altKm)
{
    KeplerianElements k;
    k.semi_major_axis_m = kReM + altKm * 1000.0;
    k.eccentricity = 0.0;
    k.inclination_rad = 0.0; // equatorial for a clean east-bound pass
    k.raan_rad = 0.0;
    k.arg_perigee_rad = 0.0;
    k.mean_anomaly_rad = meanAnomalyDeg * M_PI / 180.0;
    k.epoch_unix_s = 0.0;
    return k;
}

} // namespace

int
main(int argc, char* argv[])
{
    double simSeconds = 120.0;
    double dataRateMbps = 5.0;
    uint32_t packetBytes = 1200;
    double altKm = 550.0;
    double gsLonSepDeg = 20.0; // angular separation of the two ground stations
    double linkCapacityMbps = 50.0;

    CommandLine cmd(__FILE__);
    cmd.AddValue("simSeconds", "Simulation duration (s)", simSeconds);
    cmd.AddValue("dataRateMbps", "Offered end-to-end load (Mbps)", dataRateMbps);
    cmd.AddValue("packetBytes", "UDP payload size (bytes)", packetBytes);
    cmd.AddValue("altKm", "Satellite altitude (km)", altKm);
    cmd.AddValue("gsLonSepDeg", "Longitude separation of the two GS (deg)", gsLonSepDeg);
    cmd.AddValue("linkCapacityMbps", "Per-hop P2P capacity (Mbps)", linkCapacityMbps);
    cmd.Parse(argc, argv);

    NodeContainer nodes;
    nodes.Create(4); // 0=GS1 1=satA 2=satB 3=GS2

    // Two LEO satellites: satA leads, satB trails by gsLonSepDeg in the orbit.
    Ptr<Sgp4MobilityModel> satA = CreateObject<Sgp4MobilityModel>();
    satA->SetElements(MakeLeo(0.0, altKm));
    nodes.Get(1)->AggregateObject(satA);
    g_satA = satA;
    Ptr<Sgp4MobilityModel> satB = CreateObject<Sgp4MobilityModel>();
    satB->SetElements(MakeLeo(gsLonSepDeg, altKm));
    nodes.Get(2)->AggregateObject(satB);
    g_satB = satB;

    // Place each ground station at the t=0 sub-satellite point of its serving
    // satellite (read the SGP4 ECEF directly, avoiding any frame-convention
    // guesswork). The satellite starts overhead and the GSL elevation then
    // falls as it orbits away — a realistic setting pass.
    auto subPoint = [](const Vector& satEcef) {
        const double r =
            std::sqrt(satEcef.x * satEcef.x + satEcef.y * satEcef.y +
                      satEcef.z * satEcef.z);
        const double s = kReM / std::max(1.0, r);
        return Vector(satEcef.x * s, satEcef.y * s, satEcef.z * s);
    };
    const Vector saEcef0 = satA->GetPosition();
    const Vector sbEcef0 = satB->GetPosition();
    const Vector gs1Ecef = subPoint(saEcef0);
    const Vector gs2Ecef = subPoint(sbEcef0);
    const double gs1LonDeg = std::atan2(gs1Ecef.y, gs1Ecef.x) * 180.0 / M_PI;
    const double gs2LonDeg = std::atan2(gs2Ecef.y, gs2Ecef.x) * 180.0 / M_PI;

    Ptr<ConstantPositionMobilityModel> gs1 =
        CreateObject<ConstantPositionMobilityModel>();
    gs1->SetPosition(gs1Ecef);
    nodes.Get(0)->AggregateObject(gs1);
    g_gs1 = gs1;
    Ptr<ConstantPositionMobilityModel> gs2 =
        CreateObject<ConstantPositionMobilityModel>();
    gs2->SetPosition(gs2Ecef);
    nodes.Get(3)->AggregateObject(gs2);
    g_gs2 = gs2;

    // Contact-graph scheduler + router.
    Ptr<ContactGraphScheduler> sched = CreateObject<ContactGraphScheduler>();
    sched->SetSamplingInterval(Seconds(1.0));
    sched->SetMinElevationDeg(g_minElev);
    sched->SetMaxIslRangeM(g_maxIslRangeM);
    sched->RegisterSatellite(1, satA);
    sched->RegisterSatellite(2, satB);
    sched->RegisterGroundStation(g_gs1Id, 0.0, gs1LonDeg);
    sched->RegisterGroundStation(g_gs2Id, 0.0, gs2LonDeg);
    g_router = CreateObject<ContactGraphRouter>();
    g_router->Attach(sched);
    sched->Start();

    InternetStackHelper internet;
    internet.Install(nodes);

    PointToPointHelper p2p;
    p2p.SetDeviceAttribute(
        "DataRate",
        DataRateValue(DataRate(static_cast<uint64_t>(linkCapacityMbps * 1e6))));
    p2p.SetChannelAttribute("Delay", TimeValue(MicroSeconds(100)));

    Ipv4AddressHelper ipv4;
    // GS1 <-> satA (uplink)
    NetDeviceContainer dUp = p2p.Install(NodeContainer(nodes.Get(0), nodes.Get(1)));
    g_emUp = CreateObject<RateErrorModel>();
    g_emUp->SetUnit(RateErrorModel::ERROR_UNIT_PACKET);
    dUp.Get(1)->SetAttribute("ReceiveErrorModel", PointerValue(g_emUp));
    g_chUp = DynamicCast<PointToPointChannel>(dUp.Get(0)->GetChannel());
    ipv4.SetBase("10.70.1.0", "255.255.255.0");
    ipv4.Assign(dUp);
    // satA <-> satB (ISL)
    NetDeviceContainer dIsl = p2p.Install(NodeContainer(nodes.Get(1), nodes.Get(2)));
    g_emIsl = CreateObject<RateErrorModel>();
    g_emIsl->SetUnit(RateErrorModel::ERROR_UNIT_PACKET);
    dIsl.Get(1)->SetAttribute("ReceiveErrorModel", PointerValue(g_emIsl));
    g_chIsl = DynamicCast<PointToPointChannel>(dIsl.Get(0)->GetChannel());
    ipv4.SetBase("10.70.2.0", "255.255.255.0");
    ipv4.Assign(dIsl);
    // satB <-> GS2 (downlink)
    NetDeviceContainer dDn = p2p.Install(NodeContainer(nodes.Get(2), nodes.Get(3)));
    g_emDown = CreateObject<RateErrorModel>();
    g_emDown->SetUnit(RateErrorModel::ERROR_UNIT_PACKET);
    dDn.Get(0)->SetAttribute("ReceiveErrorModel", PointerValue(g_emDown));
    g_chDown = DynamicCast<PointToPointChannel>(dDn.Get(0)->GetChannel());
    ipv4.SetBase("10.70.3.0", "255.255.255.0");
    Ipv4InterfaceContainer iDn = ipv4.Assign(dDn);

    Ipv4GlobalRoutingHelper::PopulateRoutingTables();

    const uint16_t port = 7700;
    PacketSinkHelper sinkHelper(
        "ns3::UdpSocketFactory",
        InetSocketAddress(Ipv4Address::GetAny(), port));
    ApplicationContainer sinkApp = sinkHelper.Install(nodes.Get(3));
    sinkApp.Start(Seconds(0.0));
    sinkApp.Stop(Seconds(simSeconds));
    g_sink = DynamicCast<PacketSink>(sinkApp.Get(0));

    OnOffHelper onoff("ns3::UdpSocketFactory",
                      InetSocketAddress(iDn.GetAddress(1), port));
    onoff.SetAttribute("DataRate", DataRateValue(DataRate(uint64_t(dataRateMbps * 1e6))));
    onoff.SetAttribute("PacketSize", UintegerValue(packetBytes));
    onoff.SetAttribute("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=1]"));
    onoff.SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0]"));
    ApplicationContainer src = onoff.Install(nodes.Get(0));
    src.Start(Seconds(1.0));
    src.Stop(Seconds(simSeconds));

    FlowMonitorHelper fmHelper;
    Ptr<FlowMonitor> monitor = fmHelper.InstallAll();

    std::printf("# ntn-constellation-isl-routed-traffic "
                "(GS1->satA->[ISL]->satB->GS2)\n");
    std::printf("#   sim=%.0fs alt=%.0fkm gsSep=%.0fdeg load=%.1fMbps "
                "minElev=%.0f maxIsl=%.0fkm\n",
                simSeconds, altKm, gsLonSepDeg, dataRateMbps, g_minElev,
                g_maxIslRangeM / 1000.0);

    Simulator::Schedule(Seconds(2.0), &Tick);
    Simulator::Stop(Seconds(simSeconds + 0.1));
    Simulator::Run();

    monitor->CheckForLostPackets();
    const auto stats = monitor->GetFlowStats();
    uint64_t txP = 0, rxP = 0;
    double sumDelay = 0.0;
    uint64_t rxForDelay = 0;
    for (const auto& kv : stats)
    {
        txP += kv.second.txPackets;
        rxP += kv.second.rxPackets;
        sumDelay += kv.second.delaySum.GetSeconds();
        rxForDelay += kv.second.rxPackets;
    }
    const uint64_t totalRx = g_sink ? g_sink->GetTotalRx() : 0;
    std::printf("# === summary ===  end-to-end txPackets=%lu rxPackets=%lu "
                "PDR=%.2f%% meanDelay=%.2fms avgGoodput=%.3f Mbps\n",
                (unsigned long)txP, (unsigned long)rxP,
                txP ? 100.0 * rxP / txP : 0.0,
                rxForDelay ? (sumDelay / rxForDelay) * 1000.0 : 0.0,
                totalRx * 8.0 / simSeconds / 1e6);
    Simulator::Destroy();
    return 0;
}
