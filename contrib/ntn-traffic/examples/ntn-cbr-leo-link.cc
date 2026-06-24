/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * Author: Muhammad Uzair, Independent Researcher
 */

/**
 * @file
 *
 * @ingroup nrtv
 * @brief Drives the module's own CbrApplication / CbrHelper over a
 *        satellite-representative point-to-point link (a LEO-600 gNB <-> ground
 *        UE feeder/access hop). The link carries a RateErrorModel on the
 *        receive NetDevice so that packet loss is produced by a real ns-3
 *        error model, not a formula. Every KPI reported below is MEASURED:
 *
 *        - cbr_sent_bytes     : CbrApplication::GetSent() on the sender
 *        - cbr_rx_bytes       : PacketSink::GetTotalRx() on the receiver
 *        - cbr_loss_ratio     : 1 - (rx_bytes / sent_bytes)
 *        - throughput / delay / jitter : from FlowMonitor over the live flow
 *
 *        The one-way propagation delay defaults to ~12.9 ms which gives an
 *        end-to-end RTD of ~25.77 ms, the LEO-600 round-trip-delay reference
 *        in 3GPP TR 38.821 (NTN reference scenarios).
 */

#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/error-model.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/internet-module.h"
#include "ns3/network-module.h"
#include "ns3/ntn-traffic-module.h"
#include "ns3/point-to-point-module.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("NtnCbrLeoLink");

int
main(int argc, char* argv[])
{
    double simTime = 10.0;     // seconds
    double intervalMs = 5.0;   // CBR inter-packet interval (ms)
    uint32_t pktSize = 1024;   // CBR payload bytes
    double errorRate = 0.01;   // per-packet loss probability on the rx device
    double delayMs = 12.885;   // one-way prop delay -> ~25.77 ms RTD (LEO-600, TR 38.821)
    double dataRateMbps = 50.0;

    CommandLine cmd;
    cmd.AddValue("simTime", "Simulation time in seconds", simTime);
    cmd.AddValue("intervalMs", "CBR inter-packet interval in milliseconds", intervalMs);
    cmd.AddValue("pktSize", "CBR payload size in bytes", pktSize);
    cmd.AddValue("errorRate", "Per-packet error rate on the receive NetDevice", errorRate);
    cmd.AddValue("delayMs", "One-way link propagation delay in milliseconds", delayMs);
    cmd.AddValue("dataRateMbps", "Link data rate in Mbps", dataRateMbps);
    cmd.Parse(argc, argv);

    // node 0 = satellite gNB (sender), node 1 = ground UE (receiver)
    NodeContainer nodes;
    nodes.Create(2);

    PointToPointHelper p2p;
    std::ostringstream rateStr;
    rateStr << dataRateMbps << "Mbps";
    p2p.SetDeviceAttribute("DataRate", StringValue(rateStr.str()));
    std::ostringstream delayStr;
    delayStr << delayMs << "ms";
    p2p.SetChannelAttribute("Delay", StringValue(delayStr.str()));

    NetDeviceContainer devices = p2p.Install(nodes);

    // Real ns-3 error model on the UE (receive) device -> measured packet loss.
    Ptr<RateErrorModel> em = CreateObject<RateErrorModel>();
    em->SetUnit(RateErrorModel::ERROR_UNIT_PACKET);
    em->SetRate(errorRate);
    em->SetRandomVariable(CreateObject<UniformRandomVariable>());
    devices.Get(1)->SetAttribute("ReceiveErrorModel", PointerValue(em));

    InternetStackHelper stack;
    stack.Install(nodes);

    Ipv4AddressHelper address;
    address.SetBase("10.1.1.0", "255.255.255.0");
    Ipv4InterfaceContainer interfaces = address.Assign(devices);

    uint16_t port = 4000;
    Ipv4Address ueAddr = interfaces.GetAddress(1);

    // Receiver: PacketSink on the ground UE.
    PacketSinkHelper sinkHelper("ns3::UdpSocketFactory",
                                InetSocketAddress(Ipv4Address::GetAny(), port));
    ApplicationContainer sinkApps = sinkHelper.Install(nodes.Get(1));
    sinkApps.Start(Seconds(0.5));
    sinkApps.Stop(Seconds(simTime));

    // Sender: the module's own CbrApplication via CbrHelper.
    CbrHelper cbrHelper("ns3::UdpSocketFactory", InetSocketAddress(ueAddr, port));
    cbrHelper.SetConstantTraffic(MilliSeconds(intervalMs), pktSize);
    cbrHelper.SetAttribute("EnableStatisticsTags", BooleanValue(true));
    ApplicationContainer cbrApps = cbrHelper.Install(nodes.Get(0));
    cbrApps.Start(Seconds(1.0));
    cbrApps.Stop(Seconds(simTime));

    // FlowMonitor over the live data plane.
    FlowMonitorHelper flowmonHelper;
    Ptr<FlowMonitor> monitor = flowmonHelper.InstallAll();

    Simulator::Stop(Seconds(simTime + 0.5));
    Simulator::Run();

    // ---- MEASURED KPIs ----
    Ptr<CbrApplication> sender = DynamicCast<CbrApplication>(cbrApps.Get(0));
    Ptr<PacketSink> sink = DynamicCast<PacketSink>(sinkApps.Get(0));

    // CbrApplication::GetSent() and PacketSink::GetTotalRx() both return BYTES
    // (this is the invariant asserted in test/cbr-test.cc).
    uint64_t sentBytes = sender ? sender->GetSent() : 0;
    uint64_t rxBytes = sink ? sink->GetTotalRx() : 0;
    uint64_t sentPkts = (pktSize > 0) ? (sentBytes / pktSize) : 0;
    uint64_t rxPkts = (pktSize > 0) ? (rxBytes / pktSize) : 0;
    double lossRatio = (sentBytes > 0)
                           ? (1.0 - static_cast<double>(rxBytes) / static_cast<double>(sentBytes))
                           : 0.0;

    monitor->CheckForLostPackets();
    Ptr<Ipv4FlowClassifier> classifier =
        DynamicCast<Ipv4FlowClassifier>(flowmonHelper.GetClassifier());
    double fmThroughputMbps = 0.0;
    double fmMeanDelayMs = 0.0;
    double fmJitterMs = 0.0;
    uint64_t fmRxPackets = 0;
    for (auto& kv : monitor->GetFlowStats())
    {
        const FlowMonitor::FlowStats& st = kv.second;
        if (st.rxPackets > 0)
        {
            double durSec =
                (st.timeLastRxPacket - st.timeFirstTxPacket).GetSeconds();
            if (durSec > 0)
            {
                fmThroughputMbps += st.rxBytes * 8.0 / durSec / 1e6;
            }
            fmMeanDelayMs += st.delaySum.GetMilliSeconds();
            fmJitterMs += st.jitterSum.GetMilliSeconds();
            fmRxPackets += st.rxPackets;
        }
    }
    if (fmRxPackets > 0)
    {
        fmMeanDelayMs /= fmRxPackets;
        fmJitterMs /= fmRxPackets;
    }

    std::cout << "==== ntn-cbr-leo-link MEASURED KPIs ====" << std::endl;
    std::cout << "cbr_sent_bytes         = " << sentBytes << std::endl;
    std::cout << "cbr_sent_packets       = " << sentPkts << std::endl;
    std::cout << "cbr_rx_bytes           = " << rxBytes << std::endl;
    std::cout << "cbr_rx_packets         = " << rxPkts << std::endl;
    std::cout << "cbr_loss_ratio         = " << lossRatio << std::endl;
    std::cout << "flowmon_throughput_Mbps= " << fmThroughputMbps << std::endl;
    std::cout << "flowmon_mean_delay_ms  = " << fmMeanDelayMs << std::endl;
    std::cout << "flowmon_jitter_ms      = " << fmJitterMs << std::endl;
    std::cout << "========================================" << std::endl;

    Simulator::Destroy();
    return 0;
}
