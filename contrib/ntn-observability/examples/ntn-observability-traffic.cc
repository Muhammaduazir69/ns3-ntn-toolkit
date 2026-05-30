/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only
//
// ntn-observability-traffic — instruments a REAL LEO downlink packet
// simulation with the toolkit's observability stack. A UDP flow runs over a
// PointToPoint link gated by the live satellite geometry (RateErrorModel);
// every second an NtnInfluxSink exports the measured KPIs — delivered
// goodput, SINR, elevation, slant range, cumulative Rx bytes — as InfluxDB
// line protocol (to a file by default, or UDP to a Telegraf/InfluxDB
// endpoint). The metrics therefore come from the real data plane
// (PacketSink byte counter + FlowMonitor), not synthetic values, so the
// exported time series is a faithful telemetry trace of the pass.
//
// Quick test:  --simSeconds=120 --dataRateMbps=5 --out=/tmp/ntn-obs.lp
#include "ns3/applications-module.h"
#include "ns3/command-line.h"
#include "ns3/constant-position-mobility-model.h"
#include "ns3/constant-velocity-mobility-model.h"
#include "ns3/core-module.h"
#include "ns3/error-model.h"
#include "ns3/flow-monitor-helper.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/point-to-point-channel.h"
#include "ns3/point-to-point-helper.h"

#include "ns3/ntn-influx-sink.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

using namespace ns3;
using ns3::ntnobs::NtnInfluxSink;
using ns3::ntnobs::Point;

NS_LOG_COMPONENT_DEFINE("NtnObservabilityTraffic");

namespace
{
constexpr double kC = 299792458.0;
Ptr<MobilityModel> g_ue, g_sat;
Ptr<RateErrorModel> g_em;
Ptr<PointToPointChannel> g_channel;
Ptr<PacketSink> g_sink;
Ptr<NtnInfluxSink> g_influx;
uint64_t g_lastRx = 0;
double g_eirpDbm = 88.0;
double g_freqHz = 12e9;
double g_noiseDbm = -95.0;
double g_minElev = 10.0;

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
    const Vector u = g_ue->GetPosition();
    const Vector s = g_sat->GetPosition();
    const double elev = ElevDeg(u, s);
    const double range = Dist(u, s);
    const double sinr = (g_eirpDbm - FsplDb(range, g_freqHz)) - g_noiseDbm;
    g_em->SetRate(elev < g_minElev ? 1.0 : SnrToPer(sinr));
    g_channel->SetAttribute("Delay", TimeValue(Seconds(range / kC)));

    const uint64_t tot = g_sink ? g_sink->GetTotalRx() : 0;
    const double mbps = (tot - g_lastRx) * 8.0 / 1e6;
    g_lastRx = tot;

    // Export the REAL measured KPIs as one InfluxDB point.
    Point p;
    p.measurement = "ntn_downlink";
    p.tags["link"] = "leo-gnd";
    p.tags["band"] = "Ku";
    p.fieldsFloat["goodput_mbps"] = mbps;
    p.fieldsFloat["sinr_db"] = sinr;
    p.fieldsFloat["elevation_deg"] = elev;
    p.fieldsFloat["slant_range_km"] = range / 1000.0;
    p.fieldsInt["rx_bytes_total"] = static_cast<long long>(tot);
    p.timestamp = Simulator::Now();
    g_influx->Push(p);

    std::printf("  %6.1f  elev=%6.1f  sinr=%6.1f  goodput=%8.3f  rxBytes=%lu\n",
                Simulator::Now().GetSeconds(), elev, sinr, mbps,
                (unsigned long)tot);
    Simulator::Schedule(Seconds(1.0), &Tick);
}
} // namespace

int
main(int argc, char* argv[])
{
    double simSeconds = 120.0;
    double leoAltKm = 550.0;
    double satSpeed = 7500.0;
    double freqGHz = 12.0;
    double dataRateMbps = 5.0;
    uint32_t packetBytes = 1200;
    double txPowerDbm = 33.0;
    double antennaGainDb = 55.0;
    std::string outPath = "ntn-observability-traffic.lp";
    std::string influxHost = ""; // empty = file transport
    uint16_t influxPort = 8089;
    double linkCapacityMbps = 50.0;

    CommandLine cmd(__FILE__);
    cmd.AddValue("simSeconds", "Simulation duration (s)", simSeconds);
    cmd.AddValue("leoAltKm", "Satellite altitude (km)", leoAltKm);
    cmd.AddValue("satSpeed", "LEO ground-track speed (m/s)", satSpeed);
    cmd.AddValue("freqGHz", "Carrier frequency (GHz)", freqGHz);
    cmd.AddValue("dataRateMbps", "Offered downlink load (Mbps)", dataRateMbps);
    cmd.AddValue("packetBytes", "UDP payload size (bytes)", packetBytes);
    cmd.AddValue("txPowerDbm", "Satellite HPA output power (dBm)", txPowerDbm);
    cmd.AddValue("antennaGainDb", "Combined antenna gain (dB)", antennaGainDb);
    cmd.AddValue("out", "InfluxDB line-protocol output file", outPath);
    cmd.AddValue("influxHost", "InfluxDB/Telegraf UDP host (empty=file)", influxHost);
    cmd.AddValue("influxPort", "InfluxDB/Telegraf UDP port", influxPort);
    cmd.AddValue("linkCapacityMbps", "P2P link capacity (Mbps)", linkCapacityMbps);
    cmd.Parse(argc, argv);

    g_eirpDbm = txPowerDbm + antennaGainDb;
    g_freqHz = freqGHz * 1e9;

    NodeContainer nodes;
    nodes.Create(2);
    Ptr<ConstantPositionMobilityModel> ue =
        CreateObject<ConstantPositionMobilityModel>();
    ue->SetPosition(Vector(0, 0, 0));
    nodes.Get(0)->AggregateObject(ue);
    g_ue = ue;
    Ptr<ConstantVelocityMobilityModel> sat =
        CreateObject<ConstantVelocityMobilityModel>();
    sat->SetPosition(Vector(-0.5 * satSpeed * simSeconds, 0, leoAltKm * 1000.0));
    sat->SetVelocity(Vector(satSpeed, 0, 0));
    nodes.Get(1)->AggregateObject(sat);
    g_sat = sat;

    // Observability sink.
    g_influx = CreateObject<NtnInfluxSink>();
    g_influx->SetUseSimulationTime(true);
    if (influxHost.empty())
    {
        g_influx->SetTransport(NtnInfluxSink::Transport::File);
        g_influx->SetFilePath(outPath);
    }
    else
    {
        g_influx->SetTransport(NtnInfluxSink::Transport::Udp);
        g_influx->SetUdpEndpoint(influxHost, influxPort);
    }
    g_influx->SetFlushPeriod(Seconds(5.0));

    InternetStackHelper internet;
    internet.Install(nodes);
    PointToPointHelper p2p;
    p2p.SetDeviceAttribute(
        "DataRate",
        DataRateValue(DataRate(static_cast<uint64_t>(linkCapacityMbps * 1e6))));
    p2p.SetChannelAttribute("Delay", TimeValue(Seconds(leoAltKm * 1000.0 / kC)));
    NetDeviceContainer devices = p2p.Install(nodes);
    g_em = CreateObject<RateErrorModel>();
    g_em->SetUnit(RateErrorModel::ERROR_UNIT_PACKET);
    g_em->SetRate(1.0);
    devices.Get(0)->SetAttribute("ReceiveErrorModel", PointerValue(g_em));
    g_channel = DynamicCast<PointToPointChannel>(devices.Get(0)->GetChannel());

    Ipv4AddressHelper ipv4;
    ipv4.SetBase("10.99.1.0", "255.255.255.0");
    Ipv4InterfaceContainer ifaces = ipv4.Assign(devices);

    const uint16_t port = 8000;
    PacketSinkHelper sinkHelper(
        "ns3::UdpSocketFactory",
        InetSocketAddress(Ipv4Address::GetAny(), port));
    ApplicationContainer sinkApp = sinkHelper.Install(nodes.Get(0));
    sinkApp.Start(Seconds(0.0));
    sinkApp.Stop(Seconds(simSeconds));
    g_sink = DynamicCast<PacketSink>(sinkApp.Get(0));

    OnOffHelper onoff("ns3::UdpSocketFactory",
                      InetSocketAddress(ifaces.GetAddress(0), port));
    onoff.SetAttribute("DataRate", DataRateValue(DataRate(uint64_t(dataRateMbps * 1e6))));
    onoff.SetAttribute("PacketSize", UintegerValue(packetBytes));
    onoff.SetAttribute("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=1]"));
    onoff.SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0]"));
    ApplicationContainer src = onoff.Install(nodes.Get(1));
    src.Start(Seconds(1.0));
    src.Stop(Seconds(simSeconds));

    FlowMonitorHelper fmHelper;
    Ptr<FlowMonitor> monitor = fmHelper.InstallAll();

    std::printf("# ntn-observability-traffic (real downlink + InfluxDB KPI export)\n");
    std::printf("#   sim=%.0fs alt=%.0fkm freq=%.0fGHz load=%.1fMbps export=%s\n",
                simSeconds, leoAltKm, freqGHz, dataRateMbps,
                influxHost.empty() ? outPath.c_str() : "UDP");

    g_influx->Start();
    Simulator::Schedule(Seconds(2.0), &Tick);
    Simulator::Stop(Seconds(simSeconds + 0.1));
    Simulator::Run();
    g_influx->Flush();
    g_influx->Stop();

    monitor->CheckForLostPackets();
    const uint64_t totalRx = g_sink ? g_sink->GetTotalRx() : 0;
    std::printf("# === summary ===  totalRxBytes=%lu avgGoodput=%.3f Mbps  "
                "KPIs exported to %s\n",
                (unsigned long)totalRx, totalRx * 8.0 / simSeconds / 1e6,
                influxHost.empty() ? outPath.c_str() : "UDP endpoint");
    Simulator::Destroy();
    return 0;
}
