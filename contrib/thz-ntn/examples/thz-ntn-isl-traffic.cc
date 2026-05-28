/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only
//
// thz-ntn-isl-traffic — REAL packet transmission over a 300 GHz inter-
// satellite link whose SNR is computed by ThzNtnIslChannel from the live
// inter-satellite geometry (vacuum FSPL + space noise temperature, no
// atmosphere). Two satellites drift apart and back together; the ISL SNR
// and the delivered goodput track the changing inter-satellite range.
//
// The ISL channel's ComputeIslSnr_dB(satA, satB) is evaluated every second
// and mapped to a packet-error rate on the receiver's RateErrorModel, while
// the P2P channel delay is set from the inter-satellite slant range. So when
// the satellites are close the ISL carries full rate; as they separate the
// SNR falls and (beyond the link's max range) the link drops — all from the
// thz-ntn ISL physics, nothing hardcoded.
//
// Quick test:  --simSeconds=120 --dataRateMbps=20
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

#include "ns3/thz-ntn-isl-channel.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("ThzNtnIslTraffic");

namespace
{
constexpr double kC = 299792458.0;
Ptr<ThzNtnIslChannel> g_isl;
Ptr<MobilityModel> g_satA;
Ptr<MobilityModel> g_satB;
Ptr<RateErrorModel> g_em;
Ptr<PointToPointChannel> g_channel;
Ptr<PacketSink> g_sink;
uint64_t g_lastRx = 0;
double g_minSnrDb = 3.0;

double
SnrToPer(double snrDb)
{
    return 1.0 / (1.0 + std::exp(0.8 * (snrDb - 6.0)));
}

void
LinkProbe()
{
    const double snr = g_isl->ComputeIslSnr_dB(g_satA, g_satB);
    const double cap = g_isl->ComputeIslCapacity_Gbps(snr);
    const double dist = g_satA->GetDistanceFrom(g_satB);
    const double per = (snr < g_minSnrDb) ? 1.0 : SnrToPer(snr);
    g_em->SetRate(per);
    g_channel->SetAttribute("Delay", TimeValue(Seconds(dist / kC)));

    const uint64_t tot = g_sink ? g_sink->GetTotalRx() : 0;
    const double mbps = (tot - g_lastRx) * 8.0 / 1e6;
    g_lastRx = tot;
    std::printf("  %6.1f  %10.1f  %8.2f  %10.3f  %9.3f\n",
                Simulator::Now().GetSeconds(), dist / 1000.0, snr, cap, mbps);
    Simulator::Schedule(Seconds(1.0), &LinkProbe);
}
} // namespace

int
main(int argc, char* argv[])
{
    double simSeconds = 600.0;
    double freqGHz = 300.0;
    double txPowerDbm = 30.0;
    double txGainDb = 55.0;
    double rxGainDb = 55.0;
    double bandwidthGHz = 10.0;
    double startSepKm = 200.0;
    double maxSepKm = 5000.0;
    double dataRateMbps = 100.0;
    uint32_t packetBytes = 1200;
    double minSnrDb = 3.0;
    double linkCapacityMbps = 500.0;

    CommandLine cmd(__FILE__);
    cmd.AddValue("simSeconds", "Simulation duration (s)", simSeconds);
    cmd.AddValue("freqGHz", "ISL carrier frequency (GHz)", freqGHz);
    cmd.AddValue("txPowerDbm", "ISL Tx power (dBm)", txPowerDbm);
    cmd.AddValue("txGainDb", "ISL Tx antenna gain (dBi)", txGainDb);
    cmd.AddValue("rxGainDb", "ISL Rx antenna gain (dBi)", rxGainDb);
    cmd.AddValue("bandwidthGHz", "ISL bandwidth (GHz)", bandwidthGHz);
    cmd.AddValue("startSepKm", "Initial inter-satellite separation (km)", startSepKm);
    cmd.AddValue("maxSepKm", "Max separation at mid-sim (km)", maxSepKm);
    cmd.AddValue("dataRateMbps", "Offered ISL load (Mbps)", dataRateMbps);
    cmd.AddValue("packetBytes", "UDP payload size (bytes)", packetBytes);
    cmd.AddValue("minSnrDb", "Min SNR for a usable ISL (dB)", minSnrDb);
    cmd.AddValue("linkCapacityMbps", "P2P link capacity (Mbps)", linkCapacityMbps);
    cmd.Parse(argc, argv);

    g_minSnrDb = minSnrDb;

    NodeContainer nodes;
    nodes.Create(2);
    // Sat A fixed; Sat B drifts away to maxSep at mid-sim then returns.
    Ptr<ConstantPositionMobilityModel> a =
        CreateObject<ConstantPositionMobilityModel>();
    a->SetPosition(Vector(0, 0, 600000.0));
    nodes.Get(0)->AggregateObject(a);
    Ptr<ConstantVelocityMobilityModel> b =
        CreateObject<ConstantVelocityMobilityModel>();
    b->SetPosition(Vector(startSepKm * 1000.0, 0, 600000.0));
    // Velocity so B reaches maxSep at mid-sim (relative drift).
    const double driftMps = (maxSepKm - startSepKm) * 1000.0 / (0.5 * simSeconds);
    b->SetVelocity(Vector(driftMps, 0, 0));
    nodes.Get(1)->AggregateObject(b);
    // At mid-sim reverse the drift so they close again.
    Simulator::Schedule(Seconds(0.5 * simSeconds), [b, driftMps]() {
        b->SetVelocity(Vector(-driftMps, 0, 0));
    });
    g_satA = a;
    g_satB = b;

    g_isl = CreateObject<ThzNtnIslChannel>();
    g_isl->SetAttribute("Frequency", DoubleValue(freqGHz * 1e9));
    g_isl->SetAttribute("TxPower", DoubleValue(txPowerDbm));
    g_isl->SetAttribute("TxGain", DoubleValue(txGainDb));
    g_isl->SetAttribute("RxGain", DoubleValue(rxGainDb));
    g_isl->SetAttribute("Bandwidth", DoubleValue(bandwidthGHz * 1e9));

    PointToPointHelper p2p;
    p2p.SetDeviceAttribute(
        "DataRate",
        DataRateValue(DataRate(static_cast<uint64_t>(linkCapacityMbps * 1e6))));
    p2p.SetChannelAttribute("Delay",
                            TimeValue(Seconds(startSepKm * 1000.0 / kC)));
    NetDeviceContainer devices = p2p.Install(nodes);
    Ptr<RateErrorModel> em = CreateObject<RateErrorModel>();
    em->SetUnit(RateErrorModel::ERROR_UNIT_PACKET);
    em->SetRate(0.0);
    devices.Get(0)->SetAttribute("ReceiveErrorModel", PointerValue(em));
    g_em = em;
    g_channel = DynamicCast<PointToPointChannel>(devices.Get(0)->GetChannel());

    InternetStackHelper internet;
    internet.Install(nodes);
    Ipv4AddressHelper ipv4;
    ipv4.SetBase("10.11.1.0", "255.255.255.0");
    Ipv4InterfaceContainer ifaces = ipv4.Assign(devices);

    const uint16_t port = 9600;
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

    std::printf("# thz-ntn-isl-traffic\n");
    std::printf("#   sim=%.0fs freq=%.0fGHz P=%.0fdBm G=%.0f+%.0fdBi BW=%.0fGHz "
                "sep=%.0f→%.0f→%.0fkm load=%.1fMbps\n",
                simSeconds, freqGHz, txPowerDbm, txGainDb, rxGainDb,
                bandwidthGHz, startSepKm, maxSepKm, startSepKm, dataRateMbps);
    std::printf("# %5s  %10s  %8s  %10s  %9s\n",
                "t_s", "sep_km", "snr_dB", "cap_Gbps", "goodput");

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
