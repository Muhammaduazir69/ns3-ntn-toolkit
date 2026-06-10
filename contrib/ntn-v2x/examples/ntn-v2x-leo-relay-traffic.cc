/*
 * SPDX-License-Identifier: GPL-2.0-only
 * Copyright (c) 2026 Muhammad Uzair (ns3-ntn-toolkit, W7)
 *
 * ntn-v2x-leo-relay-traffic — a shadowed vehicle (veh0) relays its basic-safety
 * messages to a ground server through a peer vehicle (veh1) and a LEO satellite:
 * veh0 --V2V--> veh1 --uplink--> LEO --feeder--> server. Real UDP BSMs are
 * forwarded THROUGH the relay nodes (real Ipv4 routing); each hop's propagation
 * delay is the real slant range / c, and a hop carries traffic only while it is
 * in contact — the V2V hop within range, the uplink above the minimum elevation.
 * The contact gate is driven by the live geometry, NOT a closed-form SINR /
 * sigmoid; the V2xLeoRelay engine logs when veh0 must relay vs go direct.
 * Delivered BSM rate / latency are MEASURED end-to-end (PacketSink + FlowMonitor).
 *
 * Quick test:  --simSeconds=60 --bsmHz=10
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
#include "ns3/flow-monitor-helper.h"
#include "ns3/internet-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/v2x-leo-relay.h"

#include <cmath>
#include <cstdio>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("NtnV2xLeoRelayTraffic");

namespace
{
constexpr double kC = 299792458.0;
Ptr<ntnv2x::V2xLeoRelay> g_relay;
Ptr<MobilityModel> g_veh0, g_veh1, g_sat;
Ptr<RateErrorModel> g_emV2v, g_emUplink;
Ptr<PointToPointChannel> g_chV2v, g_chUplink;
Ptr<PacketSink> g_sink;
uint64_t g_lastRx = 0;
double g_maxV2vRangeM = 1500.0;
double g_minElev = 10.0;
double g_simTime = 60.0;

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
    return std::atan2(d.z, std::max(std::sqrt(d.x * d.x + d.y * d.y), 1e-3)) * 180.0 / M_PI;
}

void
Tick()
{
    if (Simulator::Now().GetSeconds() >= g_simTime)
    {
        return;
    }
    const double dV2v = Dist(g_veh0->GetPosition(), g_veh1->GetPosition());
    const double dUp = Dist(g_veh1->GetPosition(), g_sat->GetPosition());
    const double elevUp = ElevDeg(g_veh1->GetPosition(), g_sat->GetPosition());

    // Geometry contact gates (NOT a fabricated SINR): real range delays + a
    // binary in-contact/out-of-contact gate on each hop.
    g_chV2v->SetAttribute("Delay", TimeValue(Seconds(dV2v / kC)));
    g_chUplink->SetAttribute("Delay", TimeValue(Seconds(dUp / kC)));
    g_emV2v->SetRate(dV2v <= g_maxV2vRangeM ? 0.0 : 1.0);
    g_emUplink->SetRate(elevUp >= g_minElev ? 0.0 : 1.0);

    // V2xLeoRelay decision (which path veh0 should use) — logged.
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
    std::printf("  %6.1f  v2vRange=%7.0fm  upElev=%5.1f  %-5s peer=%-5s  rx_Bps=%6.0f\n",
                Simulator::Now().GetSeconds(), dV2v, elevUp,
                d0 ? (d0->directToLeo ? "DIR" : "RELAY") : "?",
                (d0 && !d0->relayPeerId.empty()) ? d0->relayPeerId.c_str() : "-", bsmRx);
    Simulator::Schedule(Seconds(1.0), &Tick);
}
} // namespace

int
main(int argc, char* argv[])
{
    double simSeconds = 60.0;
    double bsmHz = 10.0;
    uint32_t bsmBytes = 300;
    double leoAltKm = 550.0;
    double satSpeed = 7500.0;
    double relayDriftMps = 25.0;
    double maxV2vRangeM = 1500.0;
    double minDirectSnrDb = 6.0;
    double linkCapacityMbps = 20.0;
    std::string outputDir = "ntn-v2x-leo-relay-output";

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
    cmd.AddValue("outputDir", "Output directory", outputDir);
    cmd.Parse(argc, argv);
    g_maxV2vRangeM = maxV2vRangeM;
    g_simTime = simSeconds;

    NodeContainer nodes;
    nodes.Create(4); // 0=veh0 1=veh1 2=sat 3=server

    Ptr<ConstantPositionMobilityModel> veh0 = CreateObject<ConstantPositionMobilityModel>();
    veh0->SetPosition(Vector(0, 0, 1.5));
    nodes.Get(0)->AggregateObject(veh0);
    g_veh0 = veh0;
    Ptr<ConstantVelocityMobilityModel> veh1 = CreateObject<ConstantVelocityMobilityModel>();
    veh1->SetPosition(Vector(50, 0, 1.5));
    veh1->SetVelocity(Vector(relayDriftMps, 0, 0));
    nodes.Get(1)->AggregateObject(veh1);
    g_veh1 = veh1;
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
    Ptr<NtnEnuProjectionMobilityModel> sat = CreateObject<NtnEnuProjectionMobilityModel>();
    sat->SetSource(satSgp4);
    sat->SetReference(satSubLat, satSubLon, 0.0);
    nodes.Get(2)->AggregateObject(sat);
    g_sat = sat;

    g_relay = CreateObject<ntnv2x::V2xLeoRelay>();
    g_relay->SetSatellite(sat);
    g_relay->SetMaxV2vRangeM(maxV2vRangeM);
    g_relay->SetMinDirectSnrDb(minDirectSnrDb);
    g_relay->RegisterVehicle("veh0", veh0);
    g_relay->RegisterVehicle("veh1", veh1);

    InternetStackHelper internet;
    internet.Install(nodes);

    PointToPointHelper p2p;
    p2p.SetDeviceAttribute("DataRate",
                           DataRateValue(DataRate(static_cast<uint64_t>(linkCapacityMbps * 1e6))));
    p2p.SetChannelAttribute("Delay", TimeValue(MicroSeconds(10)));
    Ipv4AddressHelper ipv4;

    NetDeviceContainer dV2v = p2p.Install(NodeContainer(nodes.Get(0), nodes.Get(1)));
    g_emV2v = CreateObject<RateErrorModel>();
    g_emV2v->SetUnit(RateErrorModel::ERROR_UNIT_PACKET);
    g_emV2v->SetRate(0.0);
    dV2v.Get(1)->SetAttribute("ReceiveErrorModel", PointerValue(g_emV2v));
    g_chV2v = DynamicCast<PointToPointChannel>(dV2v.Get(0)->GetChannel());
    ipv4.SetBase("10.60.1.0", "255.255.255.0");
    ipv4.Assign(dV2v);

    NetDeviceContainer dUp = p2p.Install(NodeContainer(nodes.Get(1), nodes.Get(2)));
    g_emUplink = CreateObject<RateErrorModel>();
    g_emUplink->SetUnit(RateErrorModel::ERROR_UNIT_PACKET);
    g_emUplink->SetRate(0.0);
    dUp.Get(1)->SetAttribute("ReceiveErrorModel", PointerValue(g_emUplink));
    g_chUplink = DynamicCast<PointToPointChannel>(dUp.Get(0)->GetChannel());
    ipv4.SetBase("10.60.2.0", "255.255.255.0");
    ipv4.Assign(dUp);

    NetDeviceContainer dFeeder = p2p.Install(NodeContainer(nodes.Get(2), nodes.Get(3)));
    ipv4.SetBase("10.60.3.0", "255.255.255.0");
    Ipv4InterfaceContainer iFeeder = ipv4.Assign(dFeeder);

    Ipv4GlobalRoutingHelper::PopulateRoutingTables();

    const uint16_t port = 7600;
    PacketSinkHelper sinkHelper("ns3::UdpSocketFactory",
                                InetSocketAddress(Ipv4Address::GetAny(), port));
    ApplicationContainer sinkApp = sinkHelper.Install(nodes.Get(3));
    sinkApp.Start(Seconds(0.0));
    sinkApp.Stop(Seconds(simSeconds));
    g_sink = DynamicCast<PacketSink>(sinkApp.Get(0));

    const double bsmRateBps = bsmHz * bsmBytes * 8.0;
    OnOffHelper onoff("ns3::UdpSocketFactory", InetSocketAddress(iFeeder.GetAddress(1), port));
    onoff.SetAttribute("DataRate", DataRateValue(DataRate(uint64_t(bsmRateBps))));
    onoff.SetAttribute("PacketSize", UintegerValue(bsmBytes));
    onoff.SetAttribute("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=1]"));
    onoff.SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0]"));
    ApplicationContainer src = onoff.Install(nodes.Get(0));
    src.Start(Seconds(1.0));
    src.Stop(Seconds(simSeconds));

    FlowMonitorHelper fmHelper;
    Ptr<FlowMonitor> monitor = fmHelper.InstallAll();

    std::printf("# ntn-v2x-leo-relay-traffic (veh0 --V2V--> veh1 --uplink--> LEO --> server)\n"
                "#   sim=%.0fs bsm=%.0fHz x %uB leoAlt=%.0fkm relayDrift=%.0fm/s maxV2vRange=%.0fm\n",
                simSeconds, bsmHz, bsmBytes, leoAltKm, relayDriftMps, maxV2vRangeM);

    Simulator::Schedule(Seconds(2.0), &Tick);
    Simulator::Stop(Seconds(simSeconds + 1));
    Simulator::Run();

    monitor->CheckForLostPackets();
    uint64_t txP = 0, rxP = 0;
    double delaySum = 0;
    for (const auto& kv : monitor->GetFlowStats())
    {
        txP += kv.second.txPackets;
        rxP += kv.second.rxPackets;
        delaySum += kv.second.delaySum.GetSeconds();
    }
    const double meanDelayMs = (rxP > 0) ? (delaySum / rxP) * 1e3 : 0.0;
    std::printf("# === summary ===  BSM txPackets=%lu rxPackets=%lu PDR=%.2f%%  "
                "mean e2e delay=%.2f ms (real range delays through the relay)\n",
                (unsigned long)txP, (unsigned long)rxP, txP ? 100.0 * rxP / txP : 0.0, meanDelayMs);
    Simulator::Destroy();
    return 0;
}
