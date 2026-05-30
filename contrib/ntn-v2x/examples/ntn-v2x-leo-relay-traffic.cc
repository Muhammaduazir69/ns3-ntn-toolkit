/*
 * SPDX-License-Identifier: GPL-2.0-only
 * Copyright (c) 2026 Muhammad Uzair (ns3-ntn-toolkit)
 *
 * ntn-v2x-leo-relay-traffic — REAL V2X safety-message traffic from a vehicle
 * with a poor direct view of the LEO, relayed through a neighbour vehicle
 * that has a clear sky. The data plane is a genuine 2-hop forward:
 *
 *     veh0 (shadowed) --V2V--> veh1 (relay) --uplink--> LEO --feeder--> server
 *
 * IPv4 global routing forwards veh0's UDP basic-safety messages (BSMs) hop by
 * hop to the ground server. The V2V link's RateErrorModel degrades with the
 * inter-vehicle distance (5.9 GHz FSPL) and the veh1->LEO uplink with the
 * pass geometry, so when the relay vehicle drifts out of V2V range the
 * delivered BSM rate collapses. A V2xLeoRelay object is evaluated each second
 * and its decision (direct-to-LEO vs relay-via-peer, with the direct/relay
 * SNRs) is logged alongside the real delivery — so the routing choice and the
 * goodput both come from the geometry, nothing hardcoded.
 *
 * Quick test:  --simSeconds=120 --bsmHz=10
 */
#include "ns3/applications-module.h"
#include "ns3/command-line.h"
#include "ns3/constant-position-mobility-model.h"
#include "ns3/constant-velocity-mobility-model.h"
#include "ns3/core-module.h"
#include "ns3/error-model.h"
#include "ns3/flow-monitor-helper.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/ipv4-global-routing-helper.h"
#include "ns3/point-to-point-channel.h"
#include "ns3/point-to-point-helper.h"

#include "ns3/v2x-leo-relay.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("NtnV2xLeoRelayTraffic");

namespace
{
constexpr double kC = 299792458.0;

Ptr<ntnv2x::V2xLeoRelay> g_relay;
Ptr<MobilityModel> g_veh0; // shadowed vehicle (source)
Ptr<MobilityModel> g_veh1; // relay vehicle
Ptr<MobilityModel> g_sat;
Ptr<RateErrorModel> g_emV2v;    // veh0 -> veh1 link
Ptr<RateErrorModel> g_emUplink; // veh1 -> sat link
Ptr<PointToPointChannel> g_chV2v;
Ptr<PointToPointChannel> g_chUplink;
Ptr<PacketSink> g_sink;
uint64_t g_lastRx = 0;
double g_v2vEirpDbm = 23.0;  // realistic 5.9 GHz C-V2X (≈23 dBm, ~1 km range)
double g_v2vFreqHz = 5.9e9;
double g_upEirpDbm = 90.0;   // vehicle-roof terminal + LEO antenna
double g_upFreqHz = 2.0e9;
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

double
SnrToPer(double snrDb)
{
    return 1.0 / (1.0 + std::exp(0.8 * (snrDb - 6.0)));
}

void
Tick()
{
    // V2V hop quality (5.9 GHz, degrades with inter-vehicle distance).
    const double dV2v = Dist(g_veh0->GetPosition(), g_veh1->GetPosition());
    const double snrV2v = g_v2vEirpDbm - FsplDb(dV2v, g_v2vFreqHz) - g_noiseDbm;
    g_emV2v->SetRate(SnrToPer(snrV2v));
    g_chV2v->SetAttribute("Delay", TimeValue(Seconds(dV2v / kC)));

    // Uplink hop quality (relay vehicle -> LEO).
    const double dUp = Dist(g_veh1->GetPosition(), g_sat->GetPosition());
    const double elevUp = ElevDeg(g_veh1->GetPosition(), g_sat->GetPosition());
    const double snrUp = g_upEirpDbm - FsplDb(dUp, g_upFreqHz) - g_noiseDbm;
    g_emUplink->SetRate(elevUp < g_minElev ? 1.0 : SnrToPer(snrUp));
    g_chUplink->SetAttribute("Delay", TimeValue(Seconds(dUp / kC)));

    // Exercise the V2xLeoRelay decision engine and log veh0's choice.
    auto decisions = g_relay->EvaluateAll();
    const ntnv2x::RelayDecision* d0 = nullptr;
    for (const auto& d : decisions)
    {
        if (d.vehId == "veh0")
        {
            d0 = &d;
        }
    }

    const uint64_t tot = g_sink ? g_sink->GetTotalRx() : 0;
    const double bsmRx = double(tot - g_lastRx);
    g_lastRx = tot;
    std::printf("  %6.1f  v2vRange=%7.0fm  v2vSnr=%6.1f  upSnr=%6.1f  "
                "%-5s peer=%-5s  rx_Bps=%6.0f\n",
                Simulator::Now().GetSeconds(), dV2v, snrV2v, snrUp,
                d0 ? (d0->directToLeo ? "DIR" : "RELAY") : "?",
                (d0 && !d0->relayPeerId.empty()) ? d0->relayPeerId.c_str() : "-",
                bsmRx);
    Simulator::Schedule(Seconds(1.0), &Tick);
}
} // namespace

int
main(int argc, char* argv[])
{
    double simSeconds = 120.0;
    double bsmHz = 10.0;          // basic-safety-message rate (10 Hz typical)
    uint32_t bsmBytes = 300;      // BSM payload size
    double leoAltKm = 550.0;
    double satSpeed = 7500.0;
    double relayDriftMps = 25.0;  // relay vehicle pulls ahead at 25 m/s
    double maxV2vRangeM = 1500.0;
    double minDirectSnrDb = 6.0;
    double linkCapacityMbps = 20.0;

    CommandLine cmd(__FILE__);
    cmd.AddValue("simSeconds", "Simulation duration (s)", simSeconds);
    cmd.AddValue("bsmHz", "Basic-safety-message rate (Hz)", bsmHz);
    cmd.AddValue("bsmBytes", "BSM payload size (bytes)", bsmBytes);
    cmd.AddValue("leoAltKm", "LEO altitude (km)", leoAltKm);
    cmd.AddValue("satSpeed", "LEO ground-track speed (m/s)", satSpeed);
    cmd.AddValue("relayDriftMps", "Relay vehicle relative speed (m/s)", relayDriftMps);
    cmd.AddValue("maxV2vRange", "Max V2V range for relay (m)", maxV2vRangeM);
    cmd.AddValue("minDirectSnr", "Min direct SNR before relaying (dB)", minDirectSnrDb);
    cmd.AddValue("linkCapacityMbps", "Per-hop P2P capacity (Mbps)", linkCapacityMbps);
    cmd.Parse(argc, argv);

    NodeContainer nodes;
    nodes.Create(4); // 0=veh0 1=veh1 2=sat 3=server

    // Mobility.
    Ptr<ConstantPositionMobilityModel> veh0 =
        CreateObject<ConstantPositionMobilityModel>();
    veh0->SetPosition(Vector(0, 0, 1.5)); // shadowed (e.g. urban canyon)
    nodes.Get(0)->AggregateObject(veh0);
    g_veh0 = veh0;

    Ptr<ConstantVelocityMobilityModel> veh1 =
        CreateObject<ConstantVelocityMobilityModel>();
    veh1->SetPosition(Vector(50, 0, 1.5)); // starts 50 m ahead
    veh1->SetVelocity(Vector(relayDriftMps, 0, 0)); // pulls ahead, V2V range grows
    nodes.Get(1)->AggregateObject(veh1);
    g_veh1 = veh1;

    Ptr<ConstantVelocityMobilityModel> sat =
        CreateObject<ConstantVelocityMobilityModel>();
    sat->SetPosition(Vector(-0.5 * satSpeed * simSeconds, 0, leoAltKm * 1000.0));
    sat->SetVelocity(Vector(satSpeed, 0, 0));
    nodes.Get(2)->AggregateObject(sat);
    g_sat = sat;

    // V2xLeoRelay decision engine.
    g_relay = CreateObject<ntnv2x::V2xLeoRelay>();
    g_relay->SetSatellite(sat);
    g_relay->SetMaxV2vRangeM(maxV2vRangeM);
    g_relay->SetMinDirectSnrDb(minDirectSnrDb);
    g_relay->RegisterVehicle("veh0", veh0);
    g_relay->RegisterVehicle("veh1", veh1);

    InternetStackHelper internet;
    internet.Install(nodes);

    // 2-hop relay topology: veh0 -- veh1 -- sat -- server.
    PointToPointHelper p2p;
    p2p.SetDeviceAttribute(
        "DataRate",
        DataRateValue(DataRate(static_cast<uint64_t>(linkCapacityMbps * 1e6))));
    p2p.SetChannelAttribute("Delay", TimeValue(MicroSeconds(10)));

    Ipv4AddressHelper ipv4;
    // veh0 <-> veh1 (V2V)
    NetDeviceContainer dV2v = p2p.Install(NodeContainer(nodes.Get(0), nodes.Get(1)));
    g_emV2v = CreateObject<RateErrorModel>();
    g_emV2v->SetUnit(RateErrorModel::ERROR_UNIT_PACKET);
    g_emV2v->SetRate(0.0);
    dV2v.Get(1)->SetAttribute("ReceiveErrorModel", PointerValue(g_emV2v));
    g_chV2v = DynamicCast<PointToPointChannel>(dV2v.Get(0)->GetChannel());
    ipv4.SetBase("10.60.1.0", "255.255.255.0");
    ipv4.Assign(dV2v);

    // veh1 <-> sat (uplink)
    NetDeviceContainer dUp = p2p.Install(NodeContainer(nodes.Get(1), nodes.Get(2)));
    g_emUplink = CreateObject<RateErrorModel>();
    g_emUplink->SetUnit(RateErrorModel::ERROR_UNIT_PACKET);
    g_emUplink->SetRate(0.0);
    dUp.Get(1)->SetAttribute("ReceiveErrorModel", PointerValue(g_emUplink));
    g_chUplink = DynamicCast<PointToPointChannel>(dUp.Get(0)->GetChannel());
    ipv4.SetBase("10.60.2.0", "255.255.255.0");
    ipv4.Assign(dUp);

    // sat <-> server (feeder)
    NetDeviceContainer dFeeder =
        p2p.Install(NodeContainer(nodes.Get(2), nodes.Get(3)));
    ipv4.SetBase("10.60.3.0", "255.255.255.0");
    Ipv4InterfaceContainer iFeeder = ipv4.Assign(dFeeder);

    Ipv4GlobalRoutingHelper::PopulateRoutingTables();

    // veh0 streams BSMs to the ground server (via veh1 + sat).
    const uint16_t port = 7600;
    PacketSinkHelper sinkHelper(
        "ns3::UdpSocketFactory",
        InetSocketAddress(Ipv4Address::GetAny(), port));
    ApplicationContainer sinkApp = sinkHelper.Install(nodes.Get(3));
    sinkApp.Start(Seconds(0.0));
    sinkApp.Stop(Seconds(simSeconds));
    g_sink = DynamicCast<PacketSink>(sinkApp.Get(0));

    const double bsmRateBps = bsmHz * bsmBytes * 8.0;
    OnOffHelper onoff("ns3::UdpSocketFactory",
                      InetSocketAddress(iFeeder.GetAddress(1), port));
    onoff.SetAttribute("DataRate", DataRateValue(DataRate(uint64_t(bsmRateBps))));
    onoff.SetAttribute("PacketSize", UintegerValue(bsmBytes));
    onoff.SetAttribute("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=1]"));
    onoff.SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0]"));
    ApplicationContainer src = onoff.Install(nodes.Get(0));
    src.Start(Seconds(1.0));
    src.Stop(Seconds(simSeconds));

    FlowMonitorHelper fmHelper;
    Ptr<FlowMonitor> monitor = fmHelper.InstallAll();

    std::printf("# ntn-v2x-leo-relay-traffic (veh0 --V2V--> veh1 --uplink--> LEO --> server)\n");
    std::printf("#   sim=%.0fs bsm=%.0fHz x %uB leoAlt=%.0fkm relayDrift=%.0fm/s "
                "maxV2vRange=%.0fm\n",
                simSeconds, bsmHz, bsmBytes, leoAltKm, relayDriftMps, maxV2vRangeM);

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
    std::printf("# === summary ===  BSMs sent=%lu delivered=%lu PDR=%.2f%% "
                "meanDelay=%.2fms (end-to-end veh0->server via relay)\n",
                (unsigned long)txP, (unsigned long)rxP,
                txP ? 100.0 * rxP / txP : 0.0,
                rxForDelay ? (sumDelay / rxForDelay) * 1000.0 : 0.0);
    Simulator::Destroy();
    return 0;
}
