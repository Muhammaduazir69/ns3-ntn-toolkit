// Copyright (c) 2026 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only

/**
 * @ingroup test
 * @file nr-test-unstructured-session.cc
 *
 * @brief Unstructured PDU session test suite (nr-unstructured-session). An unstructured
 * session (3GPP TS 23.501, Section 5.6.1) carries a single network layer protocol whose
 * payload the network does not parse, so these cases drive a protocol the model knows
 * nothing about, to show that the data plane never reads the packets it carries.
 *
 * The cases assert that a UE with no IP stack attaches and activates such a session, that
 * a packet crosses the whole core in both directions with its payload and its protocol
 * number intact, that an unstructured session and an IP session run on the same UE
 * without either disturbing the other, that the non-IP traffic of the PacketSocketClient and
 * PacketSocketServer applications crosses the session both ways, that payloads looking like
 * IP headers, or shorter than one, are carried as any other, and that the session follows a UE
 * roaming with home-routed roaming and with local breakout.
 */

#include "ns3/boolean.h"
#include "ns3/config.h"
#include "ns3/data-rate.h"
#include "ns3/double.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/ipv4-header.h"
#include "ns3/ipv4-static-routing-helper.h"
#include "ns3/ipv4-static-routing.h"
#include "ns3/ipv6-header.h"
#include "ns3/log.h"
#include "ns3/mobility-helper.h"
#include "ns3/nr-helper.h"
#include "ns3/nr-point-to-point-epc-helper.h"
#include "ns3/nr-ue-net-device.h"
#include "ns3/packet-socket-address.h"
#include "ns3/packet-socket-client.h"
#include "ns3/packet-socket-helper.h"
#include "ns3/packet-socket-server.h"
#include "ns3/point-to-point-helper.h"
#include "ns3/simulator.h"
#include "ns3/test.h"
#include "ns3/udp-echo-helper.h"
#include "ns3/uinteger.h"
#include "ns3/virtual-net-device.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("NrUnstructuredSessionTest");

namespace
{

/// A network layer protocol the model has no knowledge of, to prove the session is unstructured.
constexpr uint16_t DUMMY_PROTOCOL{0x88B7};

/// Records the packets a protocol handler receives, with the protocol they arrived with.
struct ProtocolSink
{
    /// One received packet
    struct Received
    {
        std::string payload;     ///< the payload the packet carried
        uint16_t protocolNumber; ///< the protocol the packet was delivered with
    };

    std::vector<Received> received; ///< every packet received so far

    /**
     * Handler to be registered on a node with Node::RegisterProtocolHandler().
     *
     * @param device the receiving device
     * @param packet the packet
     * @param protocolNumber the protocol the packet was delivered with
     * @param from the sender address
     * @param to the destination address
     * @param packetType the type of the packet
     */
    void Receive(Ptr<NetDevice> device,
                 Ptr<const Packet> packet,
                 uint16_t protocolNumber,
                 const Address& from,
                 const Address& to,
                 NetDevice::PacketType packetType)
    {
        std::string payload(packet->GetSize(), '\0');
        packet->CopyData(reinterpret_cast<uint8_t*>(payload.data()), packet->GetSize());
        received.push_back({payload, protocolNumber});
    }
};

/**
 * Build a packet carrying a text payload.
 *
 * @param payload the text to be carried
 * @return the packet
 */
Ptr<Packet>
MakePacket(const std::string& payload)
{
    return Create<Packet>(reinterpret_cast<const uint8_t*>(payload.data()), payload.size());
}

/**
 * A minimal NR topology with one gNB, one UE and one remote host behind the PGW, on which
 * the cases of this suite activate the sessions they need.
 *
 * The pieces are held by the fixture rather than by the test case so that the helpers
 * outlive the simulation.
 */
struct NrFixture
{
    Ptr<NrHelper> nrHelper;                   ///< the NR helper
    Ptr<NrPointToPointEpcHelper> nrEpcHelper; ///< the EPC helper
    Ptr<Node> pgw;                            ///< the PGW node
    NodeContainer ues;                        ///< the UE node
    NetDeviceContainer ueDevs;                ///< the UE device
    NetDeviceContainer gnbDevs;               ///< the gNB device
    BandwidthPartInfoPtrVector bwps;          ///< the bandwidth parts

    /**
     * Build the topology. The UE gets no IP stack: a case that wants one installs it
     * before attaching.
     */
    NrFixture()
    {
        nrHelper = CreateObject<NrHelper>();
        nrEpcHelper = CreateObject<NrPointToPointEpcHelper>();
        nrHelper->SetEpcHelper(nrEpcHelper);
        nrHelper->SetGnbPhyAttribute("NoiseFigure", DoubleValue(0.0));
        nrHelper->SetUePhyAttribute("NoiseFigure", DoubleValue(0.0));

        auto bandwidthAndBwpPair = nrHelper->CreateBandwidthParts({{2.8e9, 20e6, 1}}, "UMa");
        bwps = bandwidthAndBwpPair.second;

        pgw = nrEpcHelper->GetPgwNode();

        NodeContainer gnbs;
        gnbs.Create(1);
        ues.Create(1);

        MobilityHelper mobility;
        mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
        mobility.Install(gnbs);
        mobility.Install(ues);
        gnbs.Get(0)->GetObject<MobilityModel>()->SetPosition(Vector(0.0, 0.0, 10.0));
        ues.Get(0)->GetObject<MobilityModel>()->SetPosition(Vector(20.0, 0.0, 1.5));

        gnbDevs = nrHelper->InstallGnbDevice(gnbs, bwps);
        ueDevs = nrHelper->InstallUeDevice(ues, bwps);
    }

    /**
     * @return the IMSI of the UE
     */
    uint64_t GetImsi() const
    {
        return ueDevs.Get(0)->GetObject<NrUeNetDevice>()->GetImsi();
    }
};

} // namespace

/**
 * @ingroup tests
 *
 * @brief A UE with no IP stack attaches and activates an unstructured PDU session.
 *
 * The session carries unstructured payload and is reached through a device of its own, so it
 * identifies the UE by itself and needs no address on it. An IP session, whose packets
 * the core routes by address, is what requires the UE to have one.
 */
class NrUnstructuredActivationTestCase : public TestCase
{
  public:
    NrUnstructuredActivationTestCase();

  private:
    void DoRun() override;
};

NrUnstructuredActivationTestCase::NrUnstructuredActivationTestCase()
    : TestCase("A UE with no IP stack activates an unstructured PDU session")
{
}

void
NrUnstructuredActivationTestCase::DoRun()
{
    NrFixture fx;

    uint8_t qfi =
        fx.nrHelper->ActivateUnstructuredQosFlow(fx.ueDevs.Get(0),
                                                 DUMMY_PROTOCOL,
                                                 NrQosFlow(NrQosFlow::NGBR_VIDEO_TCP_DEFAULT));
    NS_TEST_ASSERT_MSG_NE(qfi, 0, "the session must be given a valid QFI");

    fx.nrHelper->AttachToGnb(fx.ueDevs.Get(0), fx.gnbDevs.Get(0));

    auto device = fx.nrEpcHelper->GetUnstructuredSessionDevice(fx.GetImsi(), qfi);
    NS_TEST_ASSERT_MSG_NE(device, nullptr, "the session must have an egress device");
    NS_TEST_ASSERT_MSG_EQ(device->GetNode(), fx.pgw, "the egress device must live on the PGW node");

    Simulator::Stop(Seconds(0.5));
    Simulator::Run();
    Simulator::Destroy();
}

/**
 * @ingroup tests
 *
 * @brief A packet of an unstructured session crosses the core in both directions.
 *
 * The protocol driven here is unknown to the model, so nothing along the path can
 * recognize the payload: uplink is dispatched by the session the protocol was activated
 * for, and downlink recovers the protocol from the session the packet arrived on. The case
 * asserts both the payload and the protocol number at each end.
 */
class NrUnstructuredEndToEndTestCase : public TestCase
{
  public:
    NrUnstructuredEndToEndTestCase();

  private:
    void DoRun() override;
};

NrUnstructuredEndToEndTestCase::NrUnstructuredEndToEndTestCase()
    : TestCase("An unstructured PDU session carries a packet across the core both ways")
{
}

void
NrUnstructuredEndToEndTestCase::DoRun()
{
    NrFixture fx;

    uint8_t qfi =
        fx.nrHelper->ActivateUnstructuredQosFlow(fx.ueDevs.Get(0),
                                                 DUMMY_PROTOCOL,
                                                 NrQosFlow(NrQosFlow::NGBR_VIDEO_TCP_DEFAULT));
    fx.nrHelper->AttachToGnb(fx.ueDevs.Get(0), fx.gnbDevs.Get(0));

    auto sessionDevice = fx.nrEpcHelper->GetUnstructuredSessionDevice(fx.GetImsi(), qfi);
    NS_TEST_ASSERT_MSG_NE(sessionDevice, nullptr, "the session must have an egress device");

    ProtocolSink ueSink;
    ProtocolSink pgwSink;
    fx.ues.Get(0)->RegisterProtocolHandler(MakeCallback(&ProtocolSink::Receive, &ueSink),
                                           DUMMY_PROTOCOL,
                                           fx.ueDevs.Get(0));
    fx.pgw->RegisterProtocolHandler(MakeCallback(&ProtocolSink::Receive, &pgwSink),
                                    DUMMY_PROTOCOL,
                                    sessionDevice);

    const std::string uplinkPayload = "an unstructured payload going up";
    const std::string downlinkPayload = "an unstructured payload coming down";

    Simulator::Schedule(Seconds(0.3), [&]() {
        fx.ueDevs.Get(0)->Send(MakePacket(uplinkPayload),
                               fx.ueDevs.Get(0)->GetBroadcast(),
                               DUMMY_PROTOCOL);
    });
    Simulator::Schedule(Seconds(0.4), [&]() {
        sessionDevice->Send(MakePacket(downlinkPayload),
                            sessionDevice->GetBroadcast(),
                            DUMMY_PROTOCOL);
    });

    Simulator::Stop(Seconds(0.8));
    Simulator::Run();
    Simulator::Destroy();

    NS_TEST_ASSERT_MSG_EQ(pgwSink.received.size(),
                          1,
                          "the uplink packet must come out of the session device");
    NS_TEST_ASSERT_MSG_EQ(pgwSink.received.at(0).payload,
                          uplinkPayload,
                          "the uplink payload must cross the core unchanged");
    NS_TEST_ASSERT_MSG_EQ(pgwSink.received.at(0).protocolNumber,
                          DUMMY_PROTOCOL,
                          "the uplink packet must keep its protocol number");

    NS_TEST_ASSERT_MSG_EQ(ueSink.received.size(), 1, "the downlink packet must reach the UE");
    NS_TEST_ASSERT_MSG_EQ(ueSink.received.at(0).payload,
                          downlinkPayload,
                          "the downlink payload must cross the core unchanged");
    NS_TEST_ASSERT_MSG_EQ(ueSink.received.at(0).protocolNumber,
                          DUMMY_PROTOCOL,
                          "the downlink protocol must be recovered from the session");
}

/**
 * @ingroup tests
 *
 * @brief A UE runs an IP session and an unstructured session at the same time.
 *
 * The UE has an IP stack, so it gets the default IP flow on attach, and it also activates
 * an unstructured session. UDP traffic and traffic of the unknown protocol run together,
 * and the case asserts that both arrive: the uplink dispatch tells them apart by protocol,
 * and the downlink by the session each arrived on.
 */
class NrUnstructuredDualStackTestCase : public TestCase
{
  public:
    NrUnstructuredDualStackTestCase();

  private:
    void DoRun() override;
};

NrUnstructuredDualStackTestCase::NrUnstructuredDualStackTestCase()
    : TestCase("An IP session and an unstructured session run on the same UE")
{
}

void
NrUnstructuredDualStackTestCase::DoRun()
{
    NrFixture fx;

    // The remote host of the IP session, behind the PGW
    NodeContainer remoteHostContainer;
    remoteHostContainer.Create(1);
    Ptr<Node> remoteHost = remoteHostContainer.Get(0);
    InternetStackHelper internet;
    internet.Install(remoteHostContainer);

    PointToPointHelper p2ph;
    p2ph.SetDeviceAttribute("DataRate", DataRateValue(DataRate("100Gb/s")));
    p2ph.SetDeviceAttribute("Mtu", UintegerValue(1500));
    p2ph.SetChannelAttribute("Delay", TimeValue(MilliSeconds(1)));
    NetDeviceContainer internetDevices = p2ph.Install(fx.pgw, remoteHost);
    Ipv4AddressHelper ipv4h;
    ipv4h.SetBase("1.0.0.0", "255.0.0.0");
    Ipv4InterfaceContainer internetIpIfaces = ipv4h.Assign(internetDevices);

    Ipv4StaticRoutingHelper ipv4RoutingHelper;
    Ptr<Ipv4StaticRouting> remoteHostStaticRouting =
        ipv4RoutingHelper.GetStaticRouting(remoteHost->GetObject<Ipv4>());
    remoteHostStaticRouting->AddNetworkRouteTo(Ipv4Address("7.0.0.0"), Ipv4Mask("255.0.0.0"), 1);

    // The UE has an IP stack, so it also gets the default IP flow when it attaches
    internet.Install(fx.ues);
    Ipv4InterfaceContainer ueIpIface = fx.nrEpcHelper->AssignUeIpv4Address(fx.ueDevs);
    Ptr<Ipv4StaticRouting> ueStaticRouting =
        ipv4RoutingHelper.GetStaticRouting(fx.ues.Get(0)->GetObject<Ipv4>());
    ueStaticRouting->SetDefaultRoute(fx.nrEpcHelper->GetUeDefaultGatewayAddress(), 1);

    uint8_t qfi =
        fx.nrHelper->ActivateUnstructuredQosFlow(fx.ueDevs.Get(0),
                                                 DUMMY_PROTOCOL,
                                                 NrQosFlow(NrQosFlow::NGBR_VIDEO_TCP_DEFAULT));
    fx.nrHelper->AttachToGnb(fx.ueDevs.Get(0), fx.gnbDevs.Get(0));

    auto sessionDevice = fx.nrEpcHelper->GetUnstructuredSessionDevice(fx.GetImsi(), qfi);
    NS_TEST_ASSERT_MSG_NE(sessionDevice, nullptr, "the session must have an egress device");

    ProtocolSink ueSink;
    ProtocolSink pgwSink;
    fx.ues.Get(0)->RegisterProtocolHandler(MakeCallback(&ProtocolSink::Receive, &ueSink),
                                           DUMMY_PROTOCOL,
                                           fx.ueDevs.Get(0));
    fx.pgw->RegisterProtocolHandler(MakeCallback(&ProtocolSink::Receive, &pgwSink),
                                    DUMMY_PROTOCOL,
                                    sessionDevice);

    // The IP session: a UDP echo between the UE and the remote host
    uint16_t echoPort = 4000;
    UdpEchoServerHelper echoServer(echoPort);
    ApplicationContainer serverApps = echoServer.Install(remoteHost);
    serverApps.Start(Seconds(0.1));

    UdpEchoClientHelper echoClient(internetIpIfaces.GetAddress(1), echoPort);
    echoClient.SetAttribute("MaxPackets", UintegerValue(4));
    echoClient.SetAttribute("Interval", TimeValue(MilliSeconds(50)));
    echoClient.SetAttribute("PacketSize", UintegerValue(100));
    ApplicationContainer clientApps = echoClient.Install(fx.ues.Get(0));
    clientApps.Start(Seconds(0.3));

    uint32_t echoReplies = 0;
    clientApps.Get(0)->TraceConnectWithoutContext(
        "Rx",
        MakeCallback(+[](uint32_t* count, Ptr<const Packet>) { ++(*count); }).Bind(&echoReplies));

    // The unstructured session, running at the same time as the UDP echo
    const std::string uplinkPayload = "unstructured traffic beside IP";
    const std::string downlinkPayload = "unstructured answer beside IP";
    Simulator::Schedule(Seconds(0.32), [&]() {
        fx.ueDevs.Get(0)->Send(MakePacket(uplinkPayload),
                               fx.ueDevs.Get(0)->GetBroadcast(),
                               DUMMY_PROTOCOL);
    });
    Simulator::Schedule(Seconds(0.42), [&]() {
        sessionDevice->Send(MakePacket(downlinkPayload),
                            sessionDevice->GetBroadcast(),
                            DUMMY_PROTOCOL);
    });

    Simulator::Stop(Seconds(1.0));
    Simulator::Run();
    Simulator::Destroy();

    NS_TEST_ASSERT_MSG_GT(echoReplies, 0, "the IP session must carry the UDP echo");
    NS_TEST_ASSERT_MSG_EQ(pgwSink.received.size(),
                          1,
                          "the unstructured uplink must arrive while IP traffic runs");
    NS_TEST_ASSERT_MSG_EQ(pgwSink.received.at(0).payload,
                          uplinkPayload,
                          "the unstructured uplink payload must be unchanged");
    NS_TEST_ASSERT_MSG_EQ(ueSink.received.size(),
                          1,
                          "the unstructured downlink must arrive while IP traffic runs");
    NS_TEST_ASSERT_MSG_EQ(ueSink.received.at(0).payload,
                          downlinkPayload,
                          "the unstructured downlink payload must be unchanged");
    NS_TEST_ASSERT_MSG_EQ(ueSink.received.at(0).protocolNumber,
                          DUMMY_PROTOCOL,
                          "the IP session must not disturb the unstructured one");
}

/**
 * @ingroup tests
 *
 * @brief The non-IP traffic of the ns-3 packet socket applications crosses an unstructured
 * PDU session both ways.
 *
 * A PacketSocketClient on the UE, bound to its NR device and the protocol of the session, sends
 * to a PacketSocketServer bound to the egress device of the session on the PGW, and another
 * client on the PGW sends to a server on the UE. The UE has no IP stack. The case asserts that
 * every packet arrives, with its size, in each direction.
 */
class NrUnstructuredPacketSocketTestCase : public TestCase
{
  public:
    NrUnstructuredPacketSocketTestCase();

  private:
    void DoRun() override;
};

NrUnstructuredPacketSocketTestCase::NrUnstructuredPacketSocketTestCase()
    : TestCase("Packet socket applications exchange non-IP traffic over an unstructured session")
{
}

void
NrUnstructuredPacketSocketTestCase::DoRun()
{
    NrFixture fx;

    // A UE with no IP stack, attached with an unstructured session for DUMMY_PROTOCOL, whose
    // egress device on the PGW is the far end of the session.
    uint8_t qfi =
        fx.nrHelper->ActivateUnstructuredQosFlow(fx.ueDevs.Get(0),
                                                 DUMMY_PROTOCOL,
                                                 NrQosFlow(NrQosFlow::NGBR_VIDEO_TCP_DEFAULT));
    fx.nrHelper->AttachToGnb(fx.ueDevs.Get(0), fx.gnbDevs.Get(0));

    auto sessionDevice = fx.nrEpcHelper->GetUnstructuredSessionDevice(fx.GetImsi(), qfi);
    NS_TEST_ASSERT_MSG_NE(sessionDevice, nullptr, "the session must have an egress device");

    // The applications use packet sockets. The PGW has them with its IP stack; the UE, which
    // has no IP stack, needs them installed.
    PacketSocketHelper packetSocket;
    packetSocket.Install(fx.ues.Get(0));

    // Each application binds to one device and DUMMY_PROTOCOL: the NR device of the UE at one
    // end, the egress device of the session at the other. The broadcast destination is all a
    // device needs to send on the session, which identifies its peer by itself.
    const uint32_t packets = 20;
    const uint32_t size = 300;
    auto address = [](Ptr<NetDevice> device) {
        PacketSocketAddress address;
        address.SetSingleDevice(device->GetIfIndex());
        address.SetPhysicalAddress(device->GetBroadcast());
        address.SetProtocol(DUMMY_PROTOCOL);
        return address;
    };

    // One client and one server per direction: uplink from the UE to the PGW, downlink from
    // the PGW to the UE. The servers start first, and record the size of each packet received.
    std::map<std::string, std::vector<uint32_t>> received;
    for (const auto& [direction, from, to] :
         {std::tuple{"uplink", fx.ueDevs.Get(0), Ptr<NetDevice>(sessionDevice)},
          std::tuple{"downlink", Ptr<NetDevice>(sessionDevice), fx.ueDevs.Get(0)}})
    {
        auto server = CreateObject<PacketSocketServer>();
        server->SetLocal(address(to));
        server->SetStartTime(Seconds(0.2));
        to->GetNode()->AddApplication(server);
        server->TraceConnectWithoutContext(
            "Rx",
            Callback<void, Ptr<const Packet>, const Address&>(
                [&received, direction = std::string(direction)](Ptr<const Packet> packet,
                                                                const Address&) {
                    received[direction].push_back(packet->GetSize());
                }));

        auto client = CreateObject<PacketSocketClient>();
        client->SetRemote(address(from));
        client->SetAttribute("MaxPackets", UintegerValue(packets));
        client->SetAttribute("Interval", TimeValue(MilliSeconds(10)));
        client->SetAttribute("PacketSize", UintegerValue(size));
        client->SetStartTime(Seconds(0.3));
        from->GetNode()->AddApplication(client);
    }

    Simulator::Stop(Seconds(1));
    Simulator::Run();
    Simulator::Destroy();

    // Every packet of each client reached the server of its direction, at its full size: none
    // was lost, dropped as non-IP, or delivered to the other direction.
    for (const auto& direction : {"uplink", "downlink"})
    {
        const auto& sizes = received[direction];
        NS_TEST_ASSERT_MSG_EQ(sizes.size(),
                              packets,
                              "every " << direction << " packet must cross the session");
        for (auto packetSize : sizes)
        {
            NS_TEST_ASSERT_MSG_EQ(packetSize,
                                  size,
                                  "the " << direction << " packets must keep their size");
        }
    }
}

/**
 * @ingroup tests
 *
 * @brief Payloads that look like IP headers, or are shorter than one, cross an unstructured
 * PDU session both ways as any other.
 *
 * The gNB tells IP from other traffic by the first bytes of a packet, and steers it to the socket
 * of its type; the payload of an unstructured session can begin with the same bytes. The case
 * sends payloads beginning with a valid IPv4 header and a valid IPv6 header, and a payload of a
 * single byte, and asserts that each arrives intact, with the protocol of the session.
 */
class NrUnstructuredIpLikePayloadTestCase : public TestCase
{
  public:
    NrUnstructuredIpLikePayloadTestCase();

  private:
    void DoRun() override;
};

NrUnstructuredIpLikePayloadTestCase::NrUnstructuredIpLikePayloadTestCase()
    : TestCase("An unstructured session carries payloads looking like IP or shorter than it")
{
}

void
NrUnstructuredIpLikePayloadTestCase::DoRun()
{
    NrFixture fx;

    // A UE with no IP stack, attached with an unstructured session for DUMMY_PROTOCOL.
    uint8_t qfi =
        fx.nrHelper->ActivateUnstructuredQosFlow(fx.ueDevs.Get(0),
                                                 DUMMY_PROTOCOL,
                                                 NrQosFlow(NrQosFlow::NGBR_VIDEO_TCP_DEFAULT));
    fx.nrHelper->AttachToGnb(fx.ueDevs.Get(0), fx.gnbDevs.Get(0));

    auto sessionDevice = fx.nrEpcHelper->GetUnstructuredSessionDevice(fx.GetImsi(), qfi);
    NS_TEST_ASSERT_MSG_NE(sessionDevice, nullptr, "the session must have an egress device");

    // Payloads that a classifier sniffing the first bytes could take for IP: a valid IPv4
    // header with its UDP-sized payload, a valid IPv6 header likewise, and a single byte with the
    // IPv4 version nibble, shorter than any IP header.
    std::vector<std::string> payloads;
    {
        Ipv4Header ipv4;
        ipv4.SetSource(Ipv4Address("7.0.0.2"));
        ipv4.SetDestination(Ipv4Address("1.0.0.2"));
        ipv4.SetProtocol(17);
        ipv4.SetPayloadSize(8);
        auto packet = Create<Packet>(8);
        packet->AddHeader(ipv4);
        std::string payload(packet->GetSize(), '\0');
        packet->CopyData(reinterpret_cast<uint8_t*>(payload.data()), payload.size());
        payloads.push_back(payload);
    }
    {
        Ipv6Header ipv6;
        ipv6.SetSource(Ipv6Address("7777:f00d::2"));
        ipv6.SetDestination(Ipv6Address("6001:db80::2"));
        ipv6.SetNextHeader(17);
        ipv6.SetPayloadLength(8);
        auto packet = Create<Packet>(8);
        packet->AddHeader(ipv6);
        std::string payload(packet->GetSize(), '\0');
        packet->CopyData(reinterpret_cast<uint8_t*>(payload.data()), payload.size());
        payloads.push_back(payload);
    }
    payloads.emplace_back("\x45");

    // Sinks for DUMMY_PROTOCOL at both ends of the session: the uplink arrives at the egress
    // device on the PGW, the downlink at the NR device of the UE.
    ProtocolSink ueSink;
    ProtocolSink pgwSink;
    fx.ues.Get(0)->RegisterProtocolHandler(MakeCallback(&ProtocolSink::Receive, &ueSink),
                                           DUMMY_PROTOCOL,
                                           fx.ueDevs.Get(0));
    fx.pgw->RegisterProtocolHandler(MakeCallback(&ProtocolSink::Receive, &pgwSink),
                                    DUMMY_PROTOCOL,
                                    sessionDevice);

    // Each payload goes up from the UE, then down from the egress device, one at a time so that
    // they arrive in order.
    for (size_t i = 0; i < payloads.size(); i++)
    {
        const auto& payload = payloads[i];
        Simulator::Schedule(Seconds(0.3) + MilliSeconds(20 * i), [&fx, payload]() {
            fx.ueDevs.Get(0)->Send(MakePacket(payload),
                                   fx.ueDevs.Get(0)->GetBroadcast(),
                                   DUMMY_PROTOCOL);
        });
        Simulator::Schedule(Seconds(0.5) + MilliSeconds(20 * i), [sessionDevice, payload]() {
            sessionDevice->Send(MakePacket(payload), sessionDevice->GetBroadcast(), DUMMY_PROTOCOL);
        });
    }

    Simulator::Stop(Seconds(1));
    Simulator::Run();
    Simulator::Destroy();

    // Every payload arrived in each direction, byte for byte and with the protocol of the session:
    // none was taken for IP, delivered to an IP socket, or dropped.
    for (const auto& [direction, sink] :
         {std::pair{"uplink", &pgwSink}, std::pair{"downlink", &ueSink}})
    {
        NS_TEST_ASSERT_MSG_EQ(sink->received.size(),
                              payloads.size(),
                              "every " << direction << " payload must cross the session");
        for (size_t i = 0; i < std::min(payloads.size(), sink->received.size()); i++)
        {
            NS_TEST_ASSERT_MSG_EQ((sink->received[i].payload == payloads[i]),
                                  true,
                                  "the " << direction << " payload " << i << " must be intact");
            NS_TEST_ASSERT_MSG_EQ(sink->received[i].protocolNumber,
                                  DUMMY_PROTOCOL,
                                  "the " << direction << " payload " << i
                                         << " must keep the protocol of the session");
        }
    }
}

/**
 * @ingroup tests
 *
 * @brief An unstructured PDU session follows its UE to a visited network, and back.
 *
 * With home-routed roaming, the session stays in the PGW of the home network of the UE, also
 * while the visited network serves the UE. With local breakout, it is in the PGW of the serving
 * network: the visited PGW gives the session an egress device of its own when the UE attaches
 * there. The session is activated while the EPC helper of the visited network is the one set
 * in NrHelper, so that it must be activated in the home network of the UE anyway. The UE sends
 * packets of the session up, and the egress devices of the session send packets down, at home,
 * in the visited network, and at home again; the case asserts that some arrive in each period,
 * both ways, and that the uplink comes out of the PGW of the network holding the session.
 */
class NrUnstructuredRoamingTestCase : public TestCase
{
  public:
    /**
     * Constructor.
     *
     * @param homeRouted whether the roaming is home-routed, else with local breakout
     */
    NrUnstructuredRoamingTestCase(bool homeRouted);

  private:
    void DoRun() override;

    bool m_homeRouted; ///< whether the roaming is home-routed, else with local breakout
};

NrUnstructuredRoamingTestCase::NrUnstructuredRoamingTestCase(bool homeRouted)
    : TestCase(std::string("An unstructured session follows its UE with ") +
               (homeRouted ? "home-routed roaming" : "local breakout")),
      m_homeRouted(homeRouted)
{
}

void
NrUnstructuredRoamingTestCase::DoRun()
{
    // As in the nr-core-roaming suite: a radio link failure detected within a second of leaving
    // the coverage of a gNB, so that the UE attaches to the other one soon after it moves.
    Config::SetDefault("ns3::NrHelper::UseIdealRrc", BooleanValue(true));
    Config::SetDefault("ns3::NrGnbPhy::TxPower", DoubleValue(40));
    Config::SetDefault("ns3::ThreeGppPropagationLossModel::ShadowingEnabled", BooleanValue(false));
    Config::SetDefault("ns3::NrUeRrc::N310", UintegerValue(1));
    Config::SetDefault("ns3::NrUeRrc::N311", UintegerValue(1));
    Config::SetDefault("ns3::NrUeRrc::T310", TimeValue(Seconds(1)));

    // Two core networks, roaming partners: with home-routed roaming, the SGW of the visited
    // network reaches the PGW of the home network over S8.
    auto home = CreateObject<NrPointToPointEpcHelper>();
    auto visited = CreateObjectWithAttributes<NrPointToPointEpcHelper>("UeIpv4Network",
                                                                       Ipv4AddressValue("8.0.0.0"));
    home->AddRoamingPartner(visited, m_homeRouted);

    // A gNB of each network, 15 km apart, and the UE next to the home gNB.
    NodeContainer gnbNodes(2);
    NodeContainer ueNodes(1);
    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    auto positions = CreateObject<ListPositionAllocator>();
    positions->Add(Vector(0, 0, 10));
    positions->Add(Vector(15020, 0, 10));
    positions->Add(Vector(10, 0, 1.5));
    mobility.SetPositionAllocator(positions);
    mobility.Install(gnbNodes);
    mobility.Install(ueNodes);

    // The home gNB and the UE are installed with the home network set, which makes the UE its
    // subscriber, and the visited gNB with the visited network set, which stays set.
    auto nrHelper = CreateObject<NrHelper>();
    nrHelper->SetSchedulerTypeId(TypeId::LookupByName("ns3::NrMacSchedulerTdmaRR"));
    auto bandwidthParts = nrHelper->CreateBandwidthParts({{1.93e9, 10e6, 1}}, "UMa", "LOS");
    nrHelper->SetEpcHelper(home);
    auto homeGnb = nrHelper->InstallGnbDevice(gnbNodes.Get(0), bandwidthParts.second);
    auto ueDevices = nrHelper->InstallUeDevice(ueNodes, bandwidthParts.second);
    nrHelper->SetEpcHelper(visited);
    nrHelper->InstallGnbDevice(gnbNodes.Get(1), bandwidthParts.second);

    // The session is activated while the visited network is set in NrHelper; it must be
    // activated in the home network of the UE anyway, where its egress device is.
    // The session is activated while the visited network is set in NrHelper; it must be
    // activated in the home network of the UE anyway, where its first egress device is.
    auto ue = ueDevices.Get(0);
    const uint64_t imsi = ue->GetObject<NrUeNetDevice>()->GetImsi();
    uint8_t qfi =
        nrHelper->ActivateUnstructuredQosFlow(ue,
                                              DUMMY_PROTOCOL,
                                              NrQosFlow(NrQosFlow::NGBR_VIDEO_TCP_DEFAULT));
    nrHelper->AttachToGnb(ue, homeGnb.Get(0));
    NS_TEST_ASSERT_MSG_NE(home->GetUnstructuredSessionDevice(imsi, qfi),
                          nullptr,
                          "the session must be in the home network of the UE");

    // Sinks for DUMMY_PROTOCOL at both ends of the session, counting the packets received in
    // each period (at home, in the visited network, and at home again), and the uplink ones by
    // the network whose PGW they come out of.
    std::string period = "home";
    std::map<std::string, std::map<std::string, uint32_t>> uplink;
    std::map<std::string, uint32_t> downlink;
    using Handler = Callback<void,
                             Ptr<NetDevice>,
                             Ptr<const Packet>,
                             uint16_t,
                             const Address&,
                             const Address&,
                             NetDevice::PacketType>;
    for (const auto& [network, helper] : {std::pair{"home", home}, std::pair{"visited", visited}})
    {
        helper->GetPgwNode()->RegisterProtocolHandler(
            Handler([&uplink, &period, network = std::string(network)](Ptr<NetDevice>,
                                                                       Ptr<const Packet>,
                                                                       uint16_t,
                                                                       const Address&,
                                                                       const Address&,
                                                                       NetDevice::PacketType) {
                uplink[period][network]++;
            }),
            DUMMY_PROTOCOL,
            nullptr);
    }
    ueNodes.Get(0)->RegisterProtocolHandler(
        Handler([&downlink, &period](Ptr<NetDevice>,
                                     Ptr<const Packet>,
                                     uint16_t,
                                     const Address&,
                                     const Address&,
                                     NetDevice::PacketType) { downlink[period]++; }),
        DUMMY_PROTOCOL,
        ue);

    // A packet up every 100 ms, from 1 s to the end of the simulation, and one down from each
    // egress device of the session at the same time: the one of the home network, and the one
    // of the visited network once it exists. A device whose network does not hold the session
    // of the UE has no tunnel to send on.
    for (int i = 0; i < 140; i++)
    {
        Simulator::Schedule(Seconds(1) + MilliSeconds(100 * i), [=]() {
            ue->Send(MakePacket("up"), ue->GetBroadcast(), DUMMY_PROTOCOL);
            for (const auto& helper : {home, visited})
            {
                if (auto device = helper->GetUnstructuredSessionDevice(imsi, qfi))
                {
                    device->Send(MakePacket("down"), device->GetBroadcast(), DUMMY_PROTOCOL);
                }
            }
        });
    }

    // to the visited network, and back home
    auto ueMobility = ueNodes.Get(0)->GetObject<MobilityModel>();
    Simulator::Schedule(Seconds(3), [ueMobility, &period]() {
        period = "visited";
        ueMobility->SetPosition({15000, 0, 1.5});
    });
    Simulator::Schedule(Seconds(9), [ueMobility, &period]() {
        period = "home again";
        ueMobility->SetPosition({10, 0, 1.5});
    });
    Simulator::Stop(Seconds(15));
    Simulator::Run();

    // With local breakout, the visited network gave the session an egress device of its own.
    NS_TEST_ASSERT_MSG_EQ((visited->GetUnstructuredSessionDevice(imsi, qfi) != nullptr),
                          !m_homeRouted,
                          "the visited network must have an egress device only with local "
                          "breakout");
    Simulator::Destroy();

    // The session carried packets both ways in each period, the visited one included, and the
    // uplink came out of the PGW holding the session: the home one, except while the visited
    // network served the UE with local breakout.
    for (const auto& p : {"home", "visited", "home again"})
    {
        const std::string holder =
            (std::string(p) == "visited" && !m_homeRouted) ? "visited" : "home";
        const std::string other = holder == "home" ? "visited" : "home";
        NS_TEST_ASSERT_MSG_GT(uplink[p][holder],
                              0,
                              "the uplink packets must come out of the " << holder
                                                                         << " PGW in period " << p);
        NS_TEST_ASSERT_MSG_EQ(uplink[p][other],
                              0,
                              "no uplink packet must come out of the " << other << " PGW in period "
                                                                       << p);
        NS_TEST_ASSERT_MSG_GT(downlink[p], 0, "the downlink packets must arrive in period " << p);
    }
}

/**
 * @ingroup tests
 *
 * @brief Unstructured PDU session test suite
 */
class NrUnstructuredSessionTestSuite : public TestSuite
{
  public:
    NrUnstructuredSessionTestSuite();
};

NrUnstructuredSessionTestSuite::NrUnstructuredSessionTestSuite()
    : TestSuite("nr-unstructured-session", Type::SYSTEM)
{
    AddTestCase(new NrUnstructuredActivationTestCase(), Duration::QUICK);
    AddTestCase(new NrUnstructuredEndToEndTestCase(), Duration::QUICK);
    AddTestCase(new NrUnstructuredDualStackTestCase(), Duration::QUICK);
    AddTestCase(new NrUnstructuredPacketSocketTestCase(), Duration::QUICK);
    AddTestCase(new NrUnstructuredIpLikePayloadTestCase(), Duration::QUICK);
    AddTestCase(new NrUnstructuredRoamingTestCase(true), Duration::QUICK);
    AddTestCase(new NrUnstructuredRoamingTestCase(false), Duration::QUICK);
}

/// Static variable for test initialization
static NrUnstructuredSessionTestSuite g_nrUnstructuredSessionTestSuite;
