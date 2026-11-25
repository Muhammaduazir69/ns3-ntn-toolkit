/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2011, 2012 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
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
 * Author:Gaurav Sathe <gaurav.sathe@tcs.com>
 */

#include "lte-test-deactivate-bearer.h"

#include "ns3/bulk-send-helper.h"
#include "ns3/flent-helper.h"
#include "ns3/on-off-helper.h"
#include "ns3/packet-sink-helper.h"
#include "ns3/three-gpp-http-helper.h"
#include "ns3/udp-client-server-helper.h"
#include "ns3/udp-echo-helper.h"
#include "ns3/application-packet-probe.h"
#include "ns3/bulk-send-application.h"
#include "ns3/flent-application.h"
#include "ns3/onoff-application.h"
#include "ns3/packet-loss-counter.h"
#include "ns3/packet-sink.h"
#include "ns3/rta-tig-mobile-gaming.h"
#include "ns3/seq-ts-echo-header.h"
#include "ns3/seq-ts-header.h"
#include "ns3/seq-ts-size-header.h"
#include "ns3/sink-application.h"
#include "ns3/source-application.h"
#include "ns3/tgax-video-traffic.h"
#include "ns3/tgax-virtual-desktop.h"
#include "ns3/tgax-voip-traffic.h"
#include "ns3/three-gpp-http-client.h"
#include "ns3/three-gpp-http-header.h"
#include "ns3/three-gpp-http-server.h"
#include "ns3/three-gpp-http-variables.h"
#include "ns3/udp-client.h"
#include "ns3/udp-echo-client.h"
#include "ns3/udp-echo-server.h"
#include "ns3/udp-server.h"
#include "ns3/udp-trace-client.h"
#include "ns3/double.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/internet-trace-helper.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/ipv4-global-routing-helper.h"
#include "ns3/ipv6-global-routing-helper.h"
#include "ns3/ipv4-interface-container.h"
#include "ns3/ipv4-list-routing-helper.h"
#include "ns3/ipv4-routing-helper.h"
#include "ns3/ipv4-static-routing-helper.h"
#include "ns3/ipv6-address-helper.h"
#include "ns3/ipv6-interface-container.h"
#include "ns3/ipv6-list-routing-helper.h"
#include "ns3/ipv6-routing-helper.h"
#include "ns3/ipv6-static-routing-helper.h"
#include "ns3/neighbor-cache-helper.h"
#include "ns3/rip-helper.h"
#include "ns3/ripng-helper.h"
#include "ns3/arp-cache.h"
#include "ns3/arp-header.h"
#include "ns3/arp-l3-protocol.h"
#include "ns3/arp-queue-disc-item.h"
#include "ns3/candidate-queue.h"
#include "ns3/global-route-manager-impl.h"
#include "ns3/global-route-manager.h"
#include "ns3/global-router-interface.h"
#include "ns3/icmpv4-l4-protocol.h"
#include "ns3/icmpv4.h"
#include "ns3/icmpv6-header.h"
#include "ns3/icmpv6-l4-protocol.h"
#include "ns3/ip-l4-protocol.h"
#include "ns3/ipv4-address-generator.h"
#include "ns3/ipv4-end-point-demux.h"
#include "ns3/ipv4-end-point.h"
#include "ns3/global-routing.h"
#include "ns3/ipv4-header.h"
#include "ns3/ipv4-interface-address.h"
#include "ns3/ipv4-interface.h"
#include "ns3/ipv4-l3-protocol.h"
#include "ns3/ipv4-list-routing.h"
#include "ns3/ipv4-packet-filter.h"
#include "ns3/ipv4-packet-info-tag.h"
#include "ns3/ipv4-packet-probe.h"
#include "ns3/ipv4-queue-disc-item.h"
#include "ns3/ipv4-raw-socket-factory.h"
#include "ns3/ipv4-raw-socket-impl.h"
#include "ns3/ipv4-route.h"
#include "ns3/ipv4-routing-protocol.h"
#include "ns3/ipv4-routing-table-entry.h"
#include "ns3/ipv4-static-routing.h"
#include "ns3/ipv4.h"
#include "ns3/ipv6-address-generator.h"
#include "ns3/ipv6-end-point-demux.h"
#include "ns3/ipv6-end-point.h"
#include "ns3/ipv6-extension-demux.h"
#include "ns3/ipv6-extension-header.h"
#include "ns3/ipv6-extension.h"
#include "ns3/ipv6-header.h"
#include "ns3/ipv6-interface-address.h"
#include "ns3/ipv6-interface.h"
#include "ns3/ipv6-l3-protocol.h"
#include "ns3/ipv6-list-routing.h"
#include "ns3/ipv6-option-header.h"
#include "ns3/ipv6-option.h"
#include "ns3/ipv6-packet-filter.h"
#include "ns3/ipv6-packet-info-tag.h"
#include "ns3/ipv6-packet-probe.h"
#include "ns3/ipv6-pmtu-cache.h"
#include "ns3/ipv6-queue-disc-item.h"
#include "ns3/ipv6-raw-socket-factory.h"
#include "ns3/ipv6-route.h"
#include "ns3/ipv6-routing-protocol.h"
#include "ns3/ipv6-routing-table-entry.h"
#include "ns3/ipv6-static-routing.h"
#include "ns3/ipv6.h"
#include "ns3/loopback-net-device.h"
#include "ns3/ndisc-cache.h"
#include "ns3/rip-header.h"
#include "ns3/rip.h"
#include "ns3/ripng-header.h"
#include "ns3/ripng.h"
#include "ns3/rtt-estimator.h"
#include "ns3/tcp-bbr.h"
#include "ns3/tcp-bic.h"
#include "ns3/tcp-congestion-ops.h"
#include "ns3/tcp-cubic.h"
#include "ns3/tcp-dctcp.h"
#include "ns3/tcp-header.h"
#include "ns3/tcp-highspeed.h"
#include "ns3/tcp-htcp.h"
#include "ns3/tcp-hybla.h"
#include "ns3/tcp-illinois.h"
#include "ns3/tcp-l4-protocol.h"
#include "ns3/tcp-ledbat.h"
#include "ns3/tcp-linux-reno.h"
#include "ns3/tcp-lp.h"
#include "ns3/tcp-option-rfc793.h"
#include "ns3/tcp-option-sack-permitted.h"
#include "ns3/tcp-option-sack.h"
#include "ns3/tcp-option-ts.h"
#include "ns3/tcp-option-winscale.h"
#include "ns3/tcp-option.h"
#include "ns3/tcp-prr-recovery.h"
#include "ns3/tcp-rate-ops.h"
#include "ns3/tcp-recovery-ops.h"
#include "ns3/tcp-rx-buffer.h"
#include "ns3/tcp-scalable.h"
#include "ns3/tcp-socket-base.h"
#include "ns3/tcp-socket-factory.h"
#include "ns3/tcp-socket-state.h"
#include "ns3/tcp-socket.h"
#include "ns3/tcp-tx-buffer.h"
#include "ns3/tcp-tx-item.h"
#include "ns3/tcp-vegas.h"
#include "ns3/tcp-veno.h"
#include "ns3/tcp-westwood-plus.h"
#include "ns3/tcp-yeah.h"
#include "ns3/udp-header.h"
#include "ns3/udp-l4-protocol.h"
#include "ns3/udp-socket-factory.h"
#include "ns3/udp-socket.h"
#include "ns3/windowed-filter.h"
#include "ns3/ipv4-global-routing-helper.h"
#include "ns3/application-container.h"
#include "ns3/application-helper.h"
#include "ns3/delay-jitter-estimation.h"
#include "ns3/net-device-container.h"
#include "ns3/node-container.h"
#include "ns3/packet-socket-helper.h"
#include "ns3/simple-net-device-helper.h"
#include "ns3/trace-helper.h"
#include "ns3/address.h"
#include "ns3/application.h"
#include "ns3/buffer.h"
#include "ns3/byte-tag-list.h"
#include "ns3/channel-list.h"
#include "ns3/channel.h"
#include "ns3/chunk.h"
#include "ns3/header.h"
#include "ns3/net-device.h"
#include "ns3/nix-vector.h"
#include "ns3/node-list.h"
#include "ns3/node.h"
#include "ns3/packet-metadata.h"
#include "ns3/packet-tag-list.h"
#include "ns3/packet.h"
#include "ns3/socket-factory.h"
#include "ns3/socket.h"
#include "ns3/tag-buffer.h"
#include "ns3/tag.h"
#include "ns3/trailer.h"
#include "ns3/address-utils.h"
#include "ns3/bit-deserializer.h"
#include "ns3/bit-serializer.h"
#include "ns3/crc32.h"
#include "ns3/data-rate.h"
#include "ns3/drop-tail-queue.h"
#include "ns3/dynamic-queue-limits.h"
#include "ns3/error-channel.h"
#include "ns3/error-model.h"
#include "ns3/ethernet-header.h"
#include "ns3/ethernet-trailer.h"
#include "ns3/flow-id-tag.h"
#include "ns3/generic-phy.h"
#include "ns3/header-serialization-test.h"
#include "ns3/iana-link-type-numbers.h"
#include "ns3/iana-ieee802-numbers.h"
#include "ns3/iana-ppp-numbers.h"
#include "ns3/iana-internet-protocol-numbers.h"
#include "ns3/inet-socket-address.h"
#include "ns3/inet6-socket-address.h"
#include "ns3/ipv4-address.h"
#include "ns3/ipv6-address.h"
#include "ns3/llc-snap-header.h"
#include "ns3/lollipop-counter.h"
#include "ns3/mac16-address.h"
#include "ns3/mac48-address.h"
#include "ns3/mac64-address.h"
#include "ns3/mac8-address.h"
#include "ns3/net-device-queue-interface.h"
#include "ns3/output-stream-wrapper.h"
#include "ns3/packet-burst.h"
#include "ns3/packet-data-calculators.h"
#include "ns3/packet-probe.h"
#include "ns3/packet-socket-address.h"
#include "ns3/packet-socket-client.h"
#include "ns3/packet-socket-factory.h"
#include "ns3/packet-socket-server.h"
#include "ns3/packet-socket.h"
#include "ns3/packetbb.h"
#include "ns3/pcap-file-wrapper.h"
#include "ns3/pcap-file.h"
#include "ns3/pcap-test.h"
#include "ns3/queue-fwd.h"
#include "ns3/queue-item.h"
#include "ns3/queue-limits.h"
#include "ns3/queue-size.h"
#include "ns3/queue.h"
#include "ns3/radiotap-header.h"
#include "ns3/sequence-number.h"
#include "ns3/simple-channel.h"
#include "ns3/simple-net-device.h"
#include "ns3/sll-header.h"
#include "ns3/timestamp-tag.h"
#include "ns3/point-to-point-epc-helper.h"
#include "ns3/point-to-point-helper.h"
#include "ns3/radio-bearer-stats-calculator.h"
#include "ns3/string.h"
#include <ns3/boolean.h>
#include <ns3/constant-position-mobility-model.h>
#include <ns3/enum.h>
#include <ns3/eps-bearer.h>
#include <ns3/log.h>
#include <ns3/lte-enb-net-device.h>
#include <ns3/lte-enb-phy.h>
#include <ns3/lte-helper.h>
#include <ns3/lte-ue-net-device.h>
#include <ns3/lte-ue-phy.h>
#include <ns3/lte-ue-rrc.h>
#include <ns3/mobility-helper.h>
#include <ns3/net-device-container.h>
#include <ns3/node-container.h>
#include <ns3/object.h>
#include <ns3/packet.h>
#include <ns3/ptr.h>
#include <ns3/rng-seed-manager.h>
#include <ns3/simulator.h>
#include <ns3/spectrum-error-model.h>
#include <ns3/spectrum-interference.h>
#include <ns3/test.h>

#include <iostream>
#include <sstream>
#include <string>

NS_LOG_COMPONENT_DEFINE("LenaTestDeactivateBearer");

namespace ns3
{

LenaTestBearerDeactivateSuite::LenaTestBearerDeactivateSuite()
    : TestSuite("lte-test-deactivate-bearer", Type::SYSTEM)
{
    NS_LOG_INFO("creating LenaTestPssFfMacSchedulerSuite");

    bool errorModel = false;

    // Test Case: homogeneous flow test in PSS (different distance)
    // Traffic1 info
    //   UDP traffic: payload size = 100 bytes, interval = 1 ms
    //   UDP rate in scheduler: (payload + RLC header + PDCP header + IP header + UDP header) * 1000
    //   byte/sec -> 132000 byte/rate
    // Maximum throughput = 3 / ( 1/2196000 + 1/1191000 + 1/1383000) = 1486569 byte/s
    // 132000 * 3 = 396000 < 1209046 -> estimated throughput in downlink = 132000 byte/sec
    std::vector<uint16_t> dist_1;

    dist_1.push_back(0); // User 0 distance --> MCS 28
    dist_1.push_back(0); // User 1 distance --> MCS 22
    dist_1.push_back(0); // User 2 distance --> MCS 20

    std::vector<uint16_t> packetSize_1;

    packetSize_1.push_back(100); // 1
    packetSize_1.push_back(100); // 2
    packetSize_1.push_back(100); // 3

    std::vector<uint32_t> estThrPssDl_1;

    estThrPssDl_1.push_back(132000); // User 0 estimated TTI throughput from PSS
    estThrPssDl_1.push_back(132000); // User 1 estimated TTI throughput from PSS
    estThrPssDl_1.push_back(132000); // User 2 estimated TTI throughput from PSS

    AddTestCase(
        new LenaDeactivateBearerTestCase(dist_1, estThrPssDl_1, packetSize_1, 1, errorModel, true),
        Duration::QUICK);
}

static LenaTestBearerDeactivateSuite lenaTestBearerDeactivateSuite; ///< the test suite

std::string
LenaDeactivateBearerTestCase::BuildNameString(uint16_t nUser, std::vector<uint16_t> dist)
{
    std::ostringstream oss;
    oss << "distances (m) = [ ";
    for (std::vector<uint16_t>::iterator it = dist.begin(); it != dist.end(); ++it)
    {
        oss << *it << " ";
    }
    oss << "]";
    return oss.str();
}

LenaDeactivateBearerTestCase::LenaDeactivateBearerTestCase(std::vector<uint16_t> dist,
                                                           std::vector<uint32_t> estThrPssDl,
                                                           std::vector<uint16_t> packetSize,
                                                           uint16_t interval,
                                                           bool errorModelEnabled,
                                                           bool useIdealRrc)
    : TestCase(BuildNameString(dist.size(), dist)),
      m_nUser(dist.size()),
      m_dist(dist),
      m_packetSize(packetSize),
      m_interval(interval),
      m_estThrPssDl(estThrPssDl),
      m_errorModelEnabled(errorModelEnabled)
{
}

LenaDeactivateBearerTestCase::~LenaDeactivateBearerTestCase()
{
}

void
LenaDeactivateBearerTestCase::DoRun(void)
{
    uint32_t originalSeed = RngSeedManager::GetSeed();
    uint32_t originalRun = RngSeedManager::GetRun();
    RngSeedManager::SetSeed(1);
    RngSeedManager::SetRun(1);

    if (!m_errorModelEnabled)
    {
        Config::SetDefault("ns3::LteSpectrumPhy::CtrlErrorModelEnabled", BooleanValue(false));
        Config::SetDefault("ns3::LteSpectrumPhy::DataErrorModelEnabled", BooleanValue(false));
    }

    Config::SetDefault("ns3::LteHelper::UseIdealRrc", BooleanValue(true));
    Config::SetDefault("ns3::RadioBearerStatsCalculator::DlRlcOutputFilename",
                       StringValue(CreateTempDirFilename("DlRlcStats.txt")));
    Config::SetDefault("ns3::RadioBearerStatsCalculator::UlRlcOutputFilename",
                       StringValue(CreateTempDirFilename("UlRlcStats.txt")));

    Ptr<LteHelper> lteHelper = CreateObject<LteHelper>();
    Ptr<PointToPointEpcHelper> epcHelper = CreateObject<PointToPointEpcHelper>();
    lteHelper->SetEpcHelper(epcHelper);

    Ptr<Node> pgw = epcHelper->GetPgwNode();

    // Create a single RemoteHost
    NodeContainer remoteHostContainer;
    remoteHostContainer.Create(1);
    Ptr<Node> remoteHost = remoteHostContainer.Get(0);
    InternetStackHelper internet;
    internet.Install(remoteHostContainer);

    // Create the Internet
    PointToPointHelper p2ph;
    p2ph.SetDeviceAttribute("DataRate", DataRateValue(DataRate("100Gb/s")));
    p2ph.SetDeviceAttribute("Mtu", UintegerValue(1500));
    p2ph.SetChannelAttribute("Delay", TimeValue(Seconds(0.001)));
    NetDeviceContainer internetDevices = p2ph.Install(pgw, remoteHost);
    Ipv4AddressHelper ipv4h;
    ipv4h.SetBase("1.0.0.0", "255.0.0.0");
    Ipv4InterfaceContainer internetIpIfaces = ipv4h.Assign(internetDevices);
    // interface 0 is localhost, 1 is the p2p device
    Ipv4Address remoteHostAddr = internetIpIfaces.GetAddress(1);

    Ipv4StaticRoutingHelper ipv4RoutingHelper;
    Ptr<Ipv4StaticRouting> remoteHostStaticRouting =
        ipv4RoutingHelper.GetStaticRouting(remoteHost->GetObject<Ipv4>());
    remoteHostStaticRouting->AddNetworkRouteTo(Ipv4Address("7.0.0.0"), Ipv4Mask("255.0.0.0"), 1);

    // LogLevel logLevel = (LogLevel)(LOG_PREFIX_FUNC | LOG_PREFIX_TIME | LOG_LEVEL_ALL);

    // LogComponentEnable ("LenaTestDeactivateBearer", LOG_LEVEL_ALL);
    // LogComponentEnable ("LteHelper", logLevel);
    // LogComponentEnable ("EpcHelper", logLevel);
    // LogComponentEnable ("EpcEnbApplication", logLevel);
    // LogComponentEnable ("EpcSgwPgwApplication", logLevel);
    // LogComponentEnable ("EpcMme", logLevel);
    // LogComponentEnable ("LteEnbRrc", logLevel);

    lteHelper->SetAttribute("PathlossModel", StringValue("ns3::FriisSpectrumPropagationLossModel"));

    // Create Nodes: eNodeB and UE
    NodeContainer enbNodes;
    NodeContainer ueNodes;
    enbNodes.Create(1);
    ueNodes.Create(m_nUser);

    // Install Mobility Model
    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(enbNodes);
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(ueNodes);

    // Create Devices and install them in the Nodes (eNB and UE)
    NetDeviceContainer enbDevs;
    NetDeviceContainer ueDevs;
    int64_t stream = 1;

    lteHelper->SetSchedulerType("ns3::PssFfMacScheduler");
    enbDevs = lteHelper->InstallEnbDevice(enbNodes);
    stream += lteHelper->AssignStreams(enbDevs, stream);

    ueDevs = lteHelper->InstallUeDevice(ueNodes);
    stream += lteHelper->AssignStreams(ueDevs, stream);

    Ptr<LteEnbNetDevice> lteEnbDev = enbDevs.Get(0)->GetObject<LteEnbNetDevice>();
    Ptr<LteEnbPhy> enbPhy = lteEnbDev->GetPhy();
    enbPhy->SetAttribute("TxPower", DoubleValue(30.0));
    enbPhy->SetAttribute("NoiseFigure", DoubleValue(5.0));

    // Set UEs' position and power
    for (int i = 0; i < m_nUser; i++)
    {
        Ptr<ConstantPositionMobilityModel> mm =
            ueNodes.Get(i)->GetObject<ConstantPositionMobilityModel>();
        mm->SetPosition(Vector(m_dist.at(i), 0.0, 0.0));
        Ptr<LteUeNetDevice> lteUeDev = ueDevs.Get(i)->GetObject<LteUeNetDevice>();
        Ptr<LteUePhy> uePhy = lteUeDev->GetPhy();
        uePhy->SetAttribute("TxPower", DoubleValue(23.0));
        uePhy->SetAttribute("NoiseFigure", DoubleValue(9.0));
    }

    // Install the IP stack on the UEs
    internet.Install(ueNodes);
    Ipv4InterfaceContainer ueIpIface;
    ueIpIface = epcHelper->AssignUeIpv4Address(NetDeviceContainer(ueDevs));

    // Assign IP address to UEs
    for (uint32_t u = 0; u < ueNodes.GetN(); ++u)
    {
        Ptr<Node> ueNode = ueNodes.Get(u);
        // Set the default gateway for the UE
        Ptr<Ipv4StaticRouting> ueStaticRouting =
            ipv4RoutingHelper.GetStaticRouting(ueNode->GetObject<Ipv4>());
        ueStaticRouting->SetDefaultRoute(epcHelper->GetUeDefaultGatewayAddress(), 1);
    }

    // Attach a UE to a eNB
    lteHelper->Attach(ueDevs, enbDevs.Get(0));

    // Activate an EPS bearer on all UEs

    for (uint32_t u = 0; u < ueNodes.GetN(); ++u)
    {
        Ptr<NetDevice> ueDevice = ueDevs.Get(u);
        GbrQosInformation qos;
        qos.gbrDl = (m_packetSize.at(u) + 32) * (1000 / m_interval) *
                    8; // bit/s, considering IP, UDP, RLC, PDCP header size
        qos.gbrUl = (m_packetSize.at(u) + 32) * (1000 / m_interval) * 8;
        qos.mbrDl = qos.gbrDl;
        qos.mbrUl = qos.gbrUl;

        enum EpsBearer::Qci q = EpsBearer::GBR_CONV_VOICE;
        EpsBearer bearer(q, qos);
        bearer.arp.priorityLevel = 15 - (u + 1);
        bearer.arp.preemptionCapability = true;
        bearer.arp.preemptionVulnerability = true;
        lteHelper->ActivateDedicatedEpsBearer(ueDevice, bearer, EpcTft::Default());
    }

    // Install downlink and uplink applications
    uint16_t dlPort = 1234;
    uint16_t ulPort = 2000;
    PacketSinkHelper dlPacketSinkHelper("ns3::UdpSocketFactory",
                                        InetSocketAddress(Ipv4Address::GetAny(), dlPort));
    ApplicationContainer clientApps;
    ApplicationContainer serverApps;

    for (uint32_t u = 0; u < ueNodes.GetN(); ++u)
    {
        ++ulPort;
        serverApps.Add(
            dlPacketSinkHelper.Install(ueNodes.Get(u))); // receive packets from remotehost
        PacketSinkHelper ulPacketSinkHelper("ns3::UdpSocketFactory",
                                            InetSocketAddress(Ipv4Address::GetAny(), ulPort));
        serverApps.Add(ulPacketSinkHelper.Install(remoteHost)); // receive packets from UEs

        UdpClientHelper dlClient(ueIpIface.GetAddress(u), dlPort); // uplink packets generator
        dlClient.SetAttribute("Interval", TimeValue(MilliSeconds(m_interval)));
        dlClient.SetAttribute("MaxPackets", UintegerValue(1000000));
        dlClient.SetAttribute("PacketSize", UintegerValue(m_packetSize.at(u)));

        UdpClientHelper ulClient(remoteHostAddr, ulPort); // downlink packets generator
        ulClient.SetAttribute("Interval", TimeValue(MilliSeconds(m_interval)));
        ulClient.SetAttribute("MaxPackets", UintegerValue(1000000));
        ulClient.SetAttribute("PacketSize", UintegerValue(m_packetSize.at(u)));

        clientApps.Add(dlClient.Install(remoteHost));
        clientApps.Add(ulClient.Install(ueNodes.Get(u)));
    }

    serverApps.Start(Seconds(0.030));
    clientApps.Start(Seconds(0.030));

    double statsStartTime = 0.04; // need to allow for RRC connection establishment + SRS
    double statsDuration = 1.0;
    double tolerance = 0.1;

    lteHelper->EnableRlcTraces();
    Ptr<RadioBearerStatsCalculator> rlcStats = lteHelper->GetRlcStats();
    rlcStats->SetAttribute("StartTime", TimeValue(Seconds(statsStartTime)));
    rlcStats->SetAttribute("EpochDuration", TimeValue(Seconds(statsDuration)));

    // get ue device pointer for UE-ID 0 IMSI 1 and enb device pointer
    Ptr<NetDevice> ueDevice = ueDevs.Get(0);
    Ptr<NetDevice> enbDevice = enbDevs.Get(0);

    /*
     *   Instantiate De-activation using Simulator::Schedule() method which will initiate bearer
     * de-activation after deActivateTime Instantiate De-activation in sequence (Time const &time,
     * MEM mem_ptr, OBJ obj, T1 a1, T2 a2, T3 a3)
     */
    Time deActivateTime(Seconds(1.5));
    Simulator::Schedule(deActivateTime,
                        &LteHelper::DeActivateDedicatedEpsBearer,
                        lteHelper,
                        ueDevice,
                        enbDevice,
                        2);

    // stop simulation after 3 seconds
    Simulator::Stop(Seconds(3.0));

    Simulator::Run();

    NS_LOG_INFO("DL - Test with " << m_nUser << " user(s)");
    std::vector<uint64_t> dlDataRxed;
    std::vector<uint64_t> dlDataTxed;
    for (int i = 0; i < m_nUser; i++)
    {
        // get the imsi
        uint64_t imsi = ueDevs.Get(i)->GetObject<LteUeNetDevice>()->GetImsi();
        // get the lcId
        // lcId is hard-coded, since only one dedicated bearer is added
        uint8_t lcId = 4;
        dlDataRxed.push_back(rlcStats->GetDlRxData(imsi, lcId));
        dlDataTxed.push_back(rlcStats->GetDlTxData(imsi, lcId));
        NS_LOG_INFO("\tUser " << i << " dist " << m_dist.at(i) << " imsi " << imsi << " bytes rxed "
                              << (double)dlDataRxed.at(i) << "  thr "
                              << (double)dlDataRxed.at(i) / statsDuration << " ref "
                              << m_estThrPssDl.at(i));
        NS_LOG_INFO("\tUser " << i << " imsi " << imsi << " bytes txed " << (double)dlDataTxed.at(i)
                              << "  thr " << (double)dlDataTxed.at(i) / statsDuration);
    }

    for (int i = 0; i < m_nUser; i++)
    {
        uint64_t imsi = ueDevs.Get(i)->GetObject<LteUeNetDevice>()->GetImsi();

        /*
         * For UE ID-0 IMSI 1, LCID=4 is deactivated hence If traffic seen on it, test case should
         * fail Else For other UE's, test case should validate throughput
         */
        if (imsi == 1)
        {
            NS_TEST_ASSERT_MSG_EQ((double)dlDataTxed.at(i), 0, "Invalid LCID in Statistics ");
        }
        else
        {
            NS_TEST_ASSERT_MSG_EQ_TOL((double)dlDataTxed.at(i) / statsDuration,
                                      m_estThrPssDl.at(i),
                                      m_estThrPssDl.at(i) * tolerance,
                                      " Unfair Throughput!");
        }
    }

    Simulator::Destroy();

    RngSeedManager::SetSeed(originalSeed);
    RngSeedManager::SetRun(originalRun);
}
} // namespace ns3
