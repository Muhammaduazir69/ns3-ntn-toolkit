/*
 * Copyright (c) 2018 Piotr Gawlowicz
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: Piotr Gawlowicz <gawlowicz.p@gmail.com>
 * Based on script: ./examples/tcp/tcp-variants-comparison.cc
 * Modify: Pengyu Liu <eic_lpy@hust.edu.cn>
 *         Hao Yin <haoyin@uw.edu>
 *         Muyuan Shen <muyuan_shen@hust.edu.cn>
 * Topology (the bottleneck is a LEO bent-pipe hop, see ../leo-bottleneck.h):
 *
 *   Left Leafs (Clients)                       Right Leafs (Sinks)
 *           |            \                    /        |
 *           |             \    bottleneck    /         |
 *           |              R0--------------R1          |
 *           |             /                  \         |
 *           |   access   /                    \ access |
 *
 */

#include "../leo-bottleneck.h"

#include "ns3/ntn-dataset-writer.h"
#include <filesystem>
#include "ns3/ns3-ai-ntn-module.h"
#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/enum.h"
#include "ns3/error-model.h"
#include "ns3/event-id.h"
#include "ns3/flow-monitor-helper.h"
#include "ns3/internet-module.h"
#include "ns3/ipv4-global-routing-helper.h"
#include "ns3/network-module.h"
#include "ns3/point-to-point-layout-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/tcp-header.h"
#include "ns3/traffic-control-module.h"

#include <iostream>
#include <string>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("rl-tcp-example");

static std::vector<uint32_t> rxPkts;

static void
CountRxPkts(uint32_t sinkId, Ptr<const Packet> packet, const Address& srcAddr)
{
    rxPkts[sinkId]++;
}

static uint64_t g_retransmissions = 0;
static double g_rttSumS = 0.0;
static uint64_t g_rttSamples = 0;

static void
CountRetransmission(Ptr<const Packet>,
                    const TcpHeader&,
                    const Address&,
                    const Address&,
                    Ptr<const TcpSocketBase>)
{
    ++g_retransmissions;
}

static void
RecordRtt(Time, Time rtt)
{
    if (rtt.IsStrictlyPositive())
    {
        g_rttSumS += rtt.GetSeconds();
        ++g_rttSamples;
    }
}

/// Connect the RTT and retransmission traces of the sender sockets of one left leaf.
static void
ConnectSenderTraces(uint32_t nodeId)
{
    const std::string base =
        "/NodeList/" + std::to_string(nodeId) + "/$ns3::TcpL4Protocol/SocketList/*/";
    Config::ConnectWithoutContextFailSafe(base + "LastRTT", MakeCallback(&RecordRtt));
    Config::ConnectWithoutContextFailSafe(base + "Retransmission",
                                          MakeCallback(&CountRetransmission));
}

static void
PrintRxCount()
{
    uint32_t size = rxPkts.size();
    NS_LOG_UNCOND("RxPkts:");
    for (uint32_t i = 0; i < size; i++)
    {
        NS_LOG_UNCOND("---SinkId: " << i << " RxPkts: " << rxPkts.at(i));
    }
}

int
main(int argc, char* argv[])
{
    double tcpEnvTimeStep = 0.1;
    uint32_t nLeaf = 1;
    std::string transport_prot = "TcpRlTimeBased";
    double error_p = 0.0;
    std::string bottleneck_bandwidth = "2Mbps";
    std::string bottleneck_delay = "0.01ms";
    std::string access_bandwidth = "10Mbps";
    std::string access_delay = "20ms";
    std::string prefix_file_name = "TcpVariantsComparison";
    uint64_t data_mbytes = 0;
    uint32_t mtu_bytes = 400;
    double duration = 1000.0;
    uint32_t run = 0;
    bool flow_monitor = false;
    bool sack = true;
    std::string queue_disc_type = "ns3::PfifoFastQueueDisc";
    std::string recovery = "ns3::TcpClassicRecovery";
    bool leo = true;
    rltcpleo::LeoPathConfig leoCfg;
    double leoUpdateMs = 100.0;

    CommandLine cmd;
    // seed related
    cmd.AddValue("simSeed", "Seed for random generator. Default: 0", run);
    // other
    cmd.AddValue("envTimeStep",
                 "Time step interval for TcpRlTimeBased. Default: 0.1s",
                 tcpEnvTimeStep);
    cmd.AddValue("nLeaf", "Number of left and right side leaf nodes", nLeaf);
    cmd.AddValue("transport_prot",
                 "Transport protocol to use: TcpNewReno, TcpHybla, TcpHighSpeed, TcpHtcp, "
                 "TcpVegas, TcpScalable, TcpVeno, TcpBic, TcpYeah, TcpIllinois, TcpWestwood, "
                 "TcpWestwoodPlus, TcpLedbat, TcpLp, TcpRlTimeBased, TcpRlEventBased",
                 transport_prot);
    cmd.AddValue("error_p", "Packet error rate", error_p);
    cmd.AddValue("bottleneck_bandwidth", "Bottleneck bandwidth", bottleneck_bandwidth);
    cmd.AddValue("bottleneck_delay", "Bottleneck delay", bottleneck_delay);
    cmd.AddValue("access_bandwidth", "Access link bandwidth", access_bandwidth);
    cmd.AddValue("access_delay", "Access link delay", access_delay);
    cmd.AddValue("prefix_name", "Prefix of output trace file", prefix_file_name);
    cmd.AddValue("data", "Number of Megabytes of data to transmit", data_mbytes);
    cmd.AddValue("mtu", "Size of IP packets to send in bytes", mtu_bytes);
    cmd.AddValue("duration", "Time to allow flows to run in seconds", duration);
    cmd.AddValue("flow_monitor", "Enable flow monitor", flow_monitor);
    cmd.AddValue("queue_disc_type",
                 "Queue disc type for gateway (e.g. ns3::CoDelQueueDisc)",
                 queue_disc_type);
    cmd.AddValue("sack", "Enable or disable SACK option", sack);
    cmd.AddValue("recovery", "Recovery algorithm type to use (e.g., ns3::TcpPrrRecovery", recovery);
    cmd.AddValue("leo",
                 "Bottleneck is a LEO bent-pipe hop whose delay follows the SGP4 slant range "
                 "(false: the fixed bottleneck_delay of the original example)",
                 leo);
    cmd.AddValue("utLat", "User terminal (left router) latitude [deg]", leoCfg.utLatDeg);
    cmd.AddValue("utLon", "User terminal (left router) longitude [deg]", leoCfg.utLonDeg);
    cmd.AddValue("gwLat", "Feeder gateway (right router) latitude [deg]", leoCfg.gwLatDeg);
    cmd.AddValue("gwLon", "Feeder gateway (right router) longitude [deg]", leoCfg.gwLonDeg);
    cmd.AddValue("altKm", "Walker shell altitude [km]", leoCfg.altKm);
    cmd.AddValue("inclinationDeg", "Walker shell inclination [deg]", leoCfg.inclinationDeg);
    cmd.AddValue("numPlanes", "Walker shell orbital planes", leoCfg.numPlanes);
    cmd.AddValue("satsPerPlane", "Walker shell satellites per plane", leoCfg.satsPerPlane);
    cmd.AddValue("minElevationDeg",
                 "Minimum elevation at the terminal and at the gateway [deg]",
                 leoCfg.minElevationDeg);
    cmd.AddValue("tleFile",
                 "Real TLE set (CelesTrak 3-line format); replaces the Walker shell",
                 leoCfg.tleFile);
    cmd.AddValue("leoUpdateMs", "Refresh period of the bottleneck delay [ms]", leoUpdateMs);
    cmd.Parse(argc, argv);

    // There are two kinds of Tcp congestion control algorithm using RL:
    // 1. TcpRlTimeBased
    // 2. TcpRlEventBased
    // The only difference is when interaction occurs (at fixed interval or at event).
    if (transport_prot == "TcpRlTimeBased")
    {
        Config::SetDefault("ns3::TcpTimeStepEnv::StepTime", TimeValue(Seconds(tcpEnvTimeStep)));
    }

    transport_prot = std::string("ns3::") + transport_prot;
    Config::SetDefault("ns3::TcpL4Protocol::SocketType",
                       TypeIdValue(TypeId::LookupByName(transport_prot)));

    // OpenGym Env --- has to be created before any other thing
    Ptr<OpenGymInterface> openGymInterface;
    if (transport_prot == "ns3::TcpRlTimeBased" or transport_prot == "ns3::TcpRlEventBased")
    {
        openGymInterface = OpenGymInterface::Get();
    }

    SeedManager::SetSeed(1);
    SeedManager::SetRun(run);

    NS_LOG_UNCOND("C++ side random seed: " << run);
    NS_LOG_UNCOND("Tcp version: " << transport_prot);

    // Calculate the ADU size
    Header* temp_header = new Ipv4Header();
    uint32_t ip_header = temp_header->GetSerializedSize();
    NS_LOG_LOGIC("IP Header size is: " << ip_header);
    delete temp_header;
    temp_header = new TcpHeader();
    uint32_t tcp_header = temp_header->GetSerializedSize();
    NS_LOG_LOGIC("TCP Header size is: " << tcp_header);
    delete temp_header;
    uint32_t tcp_adu_size = mtu_bytes - 20 - (ip_header + tcp_header);
    NS_LOG_LOGIC("TCP ADU size is: " << tcp_adu_size);

    // Set the simulation start and stop time
    double start_time = 0.1;
    double stop_time = start_time + duration;

    // 4 MB of TCP buffer
    Config::SetDefault("ns3::TcpSocket::RcvBufSize", UintegerValue(1 << 21));
    Config::SetDefault("ns3::TcpSocket::SndBufSize", UintegerValue(1 << 21));
    Config::SetDefault("ns3::TcpSocketBase::Sack", BooleanValue(sack));
    Config::SetDefault("ns3::TcpSocket::DelAckCount", UintegerValue(2));

    Config::SetDefault("ns3::TcpL4Protocol::RecoveryType",
                       TypeIdValue(TypeId::LookupByName(recovery)));

    // Configure the error model
    // Here we use RateErrorModel with packet error rate
    Ptr<UniformRandomVariable> uv = CreateObject<UniformRandomVariable>();
    uv->SetStream(50);
    RateErrorModel error_model;
    error_model.SetRandomVariable(uv);
    error_model.SetUnit(RateErrorModel::ERROR_UNIT_PACKET);
    error_model.SetRate(error_p);

    // Create the point-to-point link helpers
    PointToPointHelper bottleNeckLink;
    bottleNeckLink.SetDeviceAttribute("DataRate", StringValue(bottleneck_bandwidth));
    bottleNeckLink.SetChannelAttribute("Delay", StringValue(bottleneck_delay));
    // bottleNeckLink.SetDeviceAttribute  ("ReceiveErrorModel", PointerValue (&error_model));

    PointToPointHelper pointToPointLeaf;
    pointToPointLeaf.SetDeviceAttribute("DataRate", StringValue(access_bandwidth));
    pointToPointLeaf.SetChannelAttribute("Delay", StringValue(access_delay));

    PointToPointDumbbellHelper d(nLeaf, pointToPointLeaf, nLeaf, pointToPointLeaf, bottleNeckLink);

    // Install IP stack
    InternetStackHelper stack;
    stack.InstallAll();

    // Traffic Control
    TrafficControlHelper tchPfifo;
    tchPfifo.SetRootQueueDisc("ns3::PfifoFastQueueDisc");

    TrafficControlHelper tchCoDel;
    tchCoDel.SetRootQueueDisc("ns3::CoDelQueueDisc");

    DataRate access_b(access_bandwidth);
    DataRate bottle_b(bottleneck_bandwidth);
    Time access_d(access_delay);
    Time bottle_d(bottleneck_delay);

    // LEO bottleneck: R0 is the user terminal, R1 the feeder gateway. The channel delay is
    // driven by the serving satellite from here on; the queue is sized from the initial
    // satellite delay instead of bottleneck_delay.
    rltcpleo::LeoBentPipe leoPath;
    if (leo)
    {
        leoCfg.updateInterval = MilliSeconds(leoUpdateMs);
        Ptr<PointToPointChannel> bottleneckChannel =
            DynamicCast<PointToPointChannel>(d.GetLeft()->GetDevice(0)->GetChannel());
        NS_ABORT_MSG_IF(!bottleneckChannel, "bottleneck channel not found");
        const double d0 =
            leoPath.Install(leoCfg, d.GetLeft(), d.GetRight(), bottleneckChannel, stop_time);
        NS_ABORT_MSG_IF(d0 < 0, "no satellite above " << leoCfg.minElevationDeg
                                                     << " deg at both ground ends at t = 0");
        bottle_d = Seconds(d0);
        NS_LOG_UNCOND("LEO bottleneck: " << leoPath.GetNumSatellites() << " satellites ("
                                         << leoPath.GetPropagatorName()
                                         << "), initial one-way delay " << d0 * 1e3 << " ms");
    }

    uint32_t size = static_cast<uint32_t>((std::min(access_b, bottle_b).GetBitRate() / 8) *
                                          ((access_d + bottle_d + access_d) * 2).GetSeconds());

    Config::SetDefault("ns3::PfifoFastQueueDisc::MaxSize",
                       QueueSizeValue(QueueSize(QueueSizeUnit::PACKETS, size / mtu_bytes)));
    Config::SetDefault("ns3::CoDelQueueDisc::MaxSize",
                       QueueSizeValue(QueueSize(QueueSizeUnit::BYTES, size)));

    if (queue_disc_type == "ns3::PfifoFastQueueDisc")
    {
        tchPfifo.Install(d.GetLeft()->GetDevice(1));
        tchPfifo.Install(d.GetRight()->GetDevice(1));
    }
    else if (queue_disc_type == "ns3::CoDelQueueDisc")
    {
        tchCoDel.Install(d.GetLeft()->GetDevice(1));
        tchCoDel.Install(d.GetRight()->GetDevice(1));
    }
    else
    {
        NS_FATAL_ERROR("Queue not recognized. Allowed values are ns3::CoDelQueueDisc or "
                       "ns3::PfifoFastQueueDisc");
    }

    // Assign IP Addresses
    d.AssignIpv4Addresses(Ipv4AddressHelper("10.1.1.0", "255.255.255.0"),
                          Ipv4AddressHelper("10.2.1.0", "255.255.255.0"),
                          Ipv4AddressHelper("10.3.1.0", "255.255.255.0"));

    NS_LOG_INFO("Initialize Global Routing.");
    Ipv4GlobalRoutingHelper::PopulateRoutingTables();

    // Install apps in left and right nodes
    uint16_t port = 50000;
    Address sinkLocalAddress(InetSocketAddress(Ipv4Address::GetAny(), port));
    PacketSinkHelper sinkHelper("ns3::TcpSocketFactory", sinkLocalAddress);
    ApplicationContainer sinkApps;
    for (uint32_t i = 0; i < d.RightCount(); ++i)
    {
        sinkHelper.SetAttribute("Protocol", TypeIdValue(TcpSocketFactory::GetTypeId()));
        sinkApps.Add(sinkHelper.Install(d.GetRight(i)));
    }
    sinkApps.Start(Seconds(0.0));
    sinkApps.Stop(Seconds(stop_time));

    for (uint32_t i = 0; i < d.LeftCount(); ++i)
    {
        // Create an on/off app sending packets to the left side
        AddressValue remoteAddress(InetSocketAddress(d.GetRightIpv4Address(i), port));
        Config::SetDefault("ns3::TcpSocket::SegmentSize", UintegerValue(tcp_adu_size));
        BulkSendHelper ftp("ns3::TcpSocketFactory", Address());
        ftp.SetAttribute("Remote", remoteAddress);
        ftp.SetAttribute("SendSize", UintegerValue(tcp_adu_size));
        ftp.SetAttribute("MaxBytes", UintegerValue(data_mbytes * 1000000));

        ApplicationContainer clientApp = ftp.Install(d.GetLeft(i));
        clientApp.Start(Seconds(start_time * i)); // Start after sink
        clientApp.Stop(Seconds(stop_time - 3));   // Stop before the sink
    }

    // Flow monitor
    FlowMonitorHelper flowHelper;
    if (flow_monitor)
    {
        flowHelper.InstallAll();
    }

    // Count RX packets
    for (uint32_t i = 0; i < d.RightCount(); ++i)
    {
        rxPkts.push_back(0);
        Ptr<PacketSink> pktSink = DynamicCast<PacketSink>(sinkApps.Get(i));
        pktSink->TraceConnectWithoutContext("Rx", MakeBoundCallback(&CountRxPkts, i));
    }

    for (uint32_t i = 0; i < d.LeftCount(); ++i)
    {
        Simulator::Schedule(Seconds(start_time * i) + MilliSeconds(1),
                            &ConnectSenderTraces,
                            d.GetLeft(i)->GetId());
    }

    // Globe scene: the two ground ends and the serving satellites carry ECEF mobility.
    ns3::ntnobs::StartSceneFromNodeList(".");

    Simulator::Stop(Seconds(stop_time));
    Simulator::Run();

    if (flow_monitor)
    {
        flowHelper.SerializeToXmlFile(prefix_file_name + ".flowmonitor", true, true);
    }

    if (transport_prot == "ns3::TcpRlTimeBased" or transport_prot == "ns3::TcpRlEventBased")
    {
        openGymInterface->NotifySimulationEnd();
    }

    PrintRxCount();
    // ntn-dataset@1: standard manifest + sim_health (plan/MASTER_PLAN_GLOBE_DATASET_E2E_2026-11.md)
    uint64_t rxBytes = 0;
    for (uint32_t i = 0; i < sinkApps.GetN(); ++i)
    {
        rxBytes += DynamicCast<PacketSink>(sinkApps.Get(i))->GetTotalRx();
    }
    const double activeS = std::max(1e-9, stop_time - 3 - start_time * (d.LeftCount() - 1));
    const double thrMbps = rxBytes * 8.0 / activeS / 1e6;
    const double meanRttMs = g_rttSamples > 0 ? 1e3 * g_rttSumS / g_rttSamples : 0.0;
    NS_LOG_UNCOND("Goodput: " << thrMbps << " Mbps, mean RTT: " << meanRttMs
                              << " ms, retransmissions: " << g_retransmissions);
    std::vector<ns3::ntnobs::DatasetMetric> metrics = {
        {"throughput_mbps", std::to_string(thrMbps), "measured"},
        {"rx_bytes", std::to_string(rxBytes), "measured"},
        {"mean_rtt_ms", std::to_string(meanRttMs), "measured"},
        {"rtt_samples", std::to_string(g_rttSamples), "measured"},
        {"retransmissions", std::to_string(g_retransmissions), "measured"}};
    if (leo)
    {
        NS_LOG_UNCOND("LEO bottleneck one-way delay: mean "
                      << leoPath.GetMeanDelayMs() << " ms, range " << leoPath.GetMinDelayMs()
                      << " to " << leoPath.GetMaxDelayMs() << " ms, " << leoPath.GetHandovers()
                      << " satellite changes");
        metrics.push_back(
            {"leo_mean_one_way_delay_ms", std::to_string(leoPath.GetMeanDelayMs()), "geometry"});
        metrics.push_back(
            {"leo_min_one_way_delay_ms", std::to_string(leoPath.GetMinDelayMs()), "geometry"});
        metrics.push_back(
            {"leo_max_one_way_delay_ms", std::to_string(leoPath.GetMaxDelayMs()), "geometry"});
        metrics.push_back(
            {"leo_satellite_changes", std::to_string(leoPath.GetHandovers()), "geometry"});
        metrics.push_back(
            {"leo_outage_s", std::to_string(leoPath.GetOutageSeconds()), "geometry"});
        metrics.push_back({"leo_propagator", leoPath.GetPropagatorName(), "config"});
    }
    ns3::ntnobs::WriteDatasetRecord(".",
                                    std::filesystem::path(argv[0]).filename().string(),
                                    "measured",
                                    argc,
                                    argv,
                                    metrics);
    Simulator::Destroy();
    return 0;
}
