/*
 * SPDX-License-Identifier: GPL-2.0-only
 * Copyright (c) 2026 Muhammad Uzair (ns3-ntn-toolkit)
 *
 * sagin-hst-leo-traffic — a high-speed-train terminal (HstMobilityModel,
 * TR 38.901 HST-A 500 km/h preset) receives REAL UDP downlink traffic from a
 * passing LEO satellite. The train races along its track while the satellite
 * crosses overhead; a per-second probe forms the satellite→train SNR from
 * FSPL at the live geometry, maps it to a packet-error rate, and updates the
 * propagation delay. Delivered goodput tracks the LEO pass.
 *
 * Covers the HstMobilityModel / HstTraceGenerator classes with a real data
 * plane. Quick test:  --simSeconds=120 --dataRateMbps=5
 */
#include "ns3/applications-module.h"
#include "ns3/command-line.h"
#include "ns3/constant-velocity-mobility-model.h"
#include "ns3/core-module.h"
#include "ns3/error-model.h"
#include "ns3/flow-monitor-helper.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/point-to-point-channel.h"
#include "ns3/point-to-point-helper.h"

#include "ns3/hst-mobility-model.h"
#include "ns3/hst-trace.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("SaginHstLeoTraffic");

namespace
{
constexpr double kC = 299792458.0;
Ptr<MobilityModel> g_train;
Ptr<MobilityModel> g_sat;
Ptr<RateErrorModel> g_em;
Ptr<PointToPointChannel> g_channel;
Ptr<PacketSink> g_sink;
uint64_t g_lastRx = 0;
double g_eirpDbm = 80.0;
double g_freqHz = 2e9;
double g_noiseDbm = -95.0;
double g_minElev = 5.0;

double
ElevDeg(const Vector& u, const Vector& s)
{
    const Vector d(s.x - u.x, s.y - u.y, s.z - u.z);
    return std::atan2(d.z, std::max(std::sqrt(d.x * d.x + d.y * d.y), 1e-3)) *
           180.0 / M_PI;
}

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

double
SnrToPer(double snrDb)
{
    return 1.0 / (1.0 + std::exp(0.8 * (snrDb - 6.0)));
}

void
LinkProbe()
{
    const Vector u = g_train->GetPosition();
    const Vector s = g_sat->GetPosition();
    const double elev = ElevDeg(u, s);
    const double range = Dist(u, s);
    const double rx = g_eirpDbm - FsplDb(range, g_freqHz);
    const double snr = rx - g_noiseDbm;
    const double per = (elev < g_minElev) ? 1.0 : SnrToPer(snr);
    g_em->SetRate(per);
    g_channel->SetAttribute("Delay", TimeValue(Seconds(range / kC)));

    const uint64_t tot = g_sink ? g_sink->GetTotalRx() : 0;
    const double mbps = (tot - g_lastRx) * 8.0 / 1e6;
    g_lastRx = tot;
    std::printf("  %6.1f  %9.0f  %7.2f  %7.2f  %9.3f\n",
                Simulator::Now().GetSeconds(), u.x, elev, snr, mbps);
    Simulator::Schedule(Seconds(1.0), &LinkProbe);
}
} // namespace

int
main(int argc, char* argv[])
{
    double simSeconds = 600.0;
    double leoAltKm = 550.0;
    double satSpeed = 7500.0;
    double freqGHz = 2.0;
    double dataRateMbps = 20.0;
    uint32_t packetBytes = 1200;
    double txPowerDbm = 33.0;
    double antennaGainDb = 47.0;
    double trainSpeedKmh = 500.0;
    double linkCapacityMbps = 50.0;

    CommandLine cmd(__FILE__);
    cmd.AddValue("simSeconds", "Simulation duration (s)", simSeconds);
    cmd.AddValue("leoAltKm", "LEO altitude (km)", leoAltKm);
    cmd.AddValue("satSpeed", "LEO ground-track speed (m/s)", satSpeed);
    cmd.AddValue("freqGHz", "Carrier frequency (GHz)", freqGHz);
    cmd.AddValue("dataRateMbps", "Offered downlink load (Mbps)", dataRateMbps);
    cmd.AddValue("packetBytes", "UDP payload size (bytes)", packetBytes);
    cmd.AddValue("txPowerDbm", "Satellite HPA output power (dBm)", txPowerDbm);
    cmd.AddValue("antennaGainDb", "Combined antenna gain (dB)", antennaGainDb);
    cmd.AddValue("trainSpeedKmh", "Train speed (km/h)", trainSpeedKmh);
    cmd.AddValue("linkCapacityMbps", "P2P link capacity (Mbps)", linkCapacityMbps);
    cmd.Parse(argc, argv);

    g_eirpDbm = txPowerDbm + antennaGainDb;
    g_freqHz = freqGHz * 1e9;

    // Generate a TR 38.901-style HST track at the requested speed.
    sagin::HstTrace trace =
        sagin::HstTraceGenerator::Generate(trainSpeedKmh, 150.0,
                                           simSeconds + 60.0, 200);

    NodeContainer nodes;
    nodes.Create(2); // 0=train terminal, 1=satellite
    Ptr<sagin::HstMobilityModel> train =
        CreateObject<sagin::HstMobilityModel>();
    train->SetCarrierFrequencyHz(g_freqHz);
    train->SetTrace(trace);
    nodes.Get(0)->AggregateObject(train);
    g_train = train;

    Ptr<ConstantVelocityMobilityModel> sat =
        CreateObject<ConstantVelocityMobilityModel>();
    sat->SetPosition(Vector(-0.5 * satSpeed * simSeconds, 0, leoAltKm * 1000.0));
    sat->SetVelocity(Vector(satSpeed, 0, 0));
    nodes.Get(1)->AggregateObject(sat);
    g_sat = sat;

    PointToPointHelper p2p;
    p2p.SetDeviceAttribute(
        "DataRate",
        DataRateValue(DataRate(static_cast<uint64_t>(linkCapacityMbps * 1e6))));
    p2p.SetChannelAttribute("Delay", TimeValue(Seconds(leoAltKm * 1000.0 / kC)));
    NetDeviceContainer devices = p2p.Install(nodes);
    Ptr<RateErrorModel> em = CreateObject<RateErrorModel>();
    em->SetUnit(RateErrorModel::ERROR_UNIT_PACKET);
    em->SetRate(1.0);
    devices.Get(0)->SetAttribute("ReceiveErrorModel", PointerValue(em));
    g_em = em;
    g_channel = DynamicCast<PointToPointChannel>(devices.Get(0)->GetChannel());

    InternetStackHelper internet;
    internet.Install(nodes);
    Ipv4AddressHelper ipv4;
    ipv4.SetBase("10.32.1.0", "255.255.255.0");
    Ipv4InterfaceContainer ifaces = ipv4.Assign(devices);

    const uint16_t port = 7300;
    PacketSinkHelper sinkHelper(
        "ns3::UdpSocketFactory",
        InetSocketAddress(Ipv4Address::GetAny(), port));
    ApplicationContainer sinkApp = sinkHelper.Install(nodes.Get(0));
    sinkApp.Start(Seconds(0.0));
    sinkApp.Stop(Seconds(simSeconds));
    g_sink = DynamicCast<PacketSink>(sinkApp.Get(0));

    OnOffHelper onoff("ns3::UdpSocketFactory",
                      InetSocketAddress(ifaces.GetAddress(0), port));
    onoff.SetAttribute("DataRate",
                       DataRateValue(DataRate(static_cast<uint64_t>(
                           dataRateMbps * 1e6))));
    onoff.SetAttribute("PacketSize", UintegerValue(packetBytes));
    onoff.SetAttribute("OnTime",
                       StringValue("ns3::ConstantRandomVariable[Constant=1]"));
    onoff.SetAttribute("OffTime",
                       StringValue("ns3::ConstantRandomVariable[Constant=0]"));
    ApplicationContainer srcApp = onoff.Install(nodes.Get(1));
    srcApp.Start(Seconds(1.0));
    srcApp.Stop(Seconds(simSeconds));

    FlowMonitorHelper fmHelper;
    Ptr<FlowMonitor> monitor = fmHelper.InstallAll();

    std::printf("# sagin-hst-leo-traffic (HST terminal + LEO)\n");
    std::printf("#   sim=%.0fs leoAlt=%.0fkm freq=%.0fGHz load=%.1fMbps "
                "EIRP=%.1fdBm train=%.0fkm/h\n",
                simSeconds, leoAltKm, freqGHz, dataRateMbps, g_eirpDbm,
                trainSpeedKmh);
    std::printf("# %5s  %9s  %7s  %7s  %9s\n",
                "t_s", "train_x", "elev", "snr_dB", "goodput");

    Simulator::Schedule(Seconds(2.0), &LinkProbe);
    Simulator::Stop(Seconds(simSeconds + 0.1));
    Simulator::Run();

    monitor->CheckForLostPackets();
    const auto stats = monitor->GetFlowStats();
    uint64_t txP = 0, rxP = 0;
    for (const auto& kv : stats)
    {
        txP += kv.second.txPackets;
        rxP += kv.second.rxPackets;
    }
    const uint64_t totalRx = g_sink ? g_sink->GetTotalRx() : 0;
    std::printf("# === summary ===  txPackets=%lu rxPackets=%lu PDR=%.2f%% "
                "avgGoodput=%.3f Mbps\n",
                (unsigned long)txP, (unsigned long)rxP,
                txP ? 100.0 * rxP / txP : 0.0,
                totalRx * 8.0 / simSeconds / 1e6);
    Simulator::Destroy();
    return 0;
}
