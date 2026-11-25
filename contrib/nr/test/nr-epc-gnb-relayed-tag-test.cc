// Copyright (c) 2026 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only

#include "ns3/internet-stack-helper.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/nr-epc-gnb-application.h"
#include "ns3/nr-epc-gnb-s1-sap.h"
#include "ns3/nr-epc-s1ap-sap.h"
#include "ns3/nr-qos-flow-tag.h"
#include "ns3/packet-socket-address.h"
#include "ns3/packet-socket-helper.h"
#include "ns3/packet-socket.h"
#include "ns3/packet.h"
#include "ns3/simple-net-device-helper.h"
#include "ns3/simulator.h"
#include "ns3/test.h"

#include <list>

/**
 * @file nr-epc-gnb-relayed-tag-test.cc
 * @ingroup tests
 * @brief Packet socket tags on packets relayed by the gNB EPC application
 */

namespace ns3
{

/**
 * @ingroup tests
 * @brief Collects the DataRadioBearerSetupRequest the gNB application issues.
 *
 * The application asks the RRC to set up the radio bearer before it records the
 * TEID mapping that the uplink relay path needs; the test only has to accept the
 * request, not act on it.
 */
class RelayedTagS1SapUser : public NrEpcGnbS1SapUser
{
  public:
    void InitialContextSetupRequest(InitialContextSetupRequestParameters params) override
    {
    }

    void DataRadioBearerSetupRequest(DataRadioBearerSetupRequestParameters params) override
    {
        m_setupRnti = params.rnti;
    }

    void PathSwitchRequestAcknowledge(PathSwitchRequestAcknowledgeParameters params) override
    {
    }

    void UeContextReleaseCommand(uint64_t imsi, uint16_t rnti) override
    {
    }

    uint16_t m_setupRnti{0}; ///< RNTI of the last bearer setup request
};

/**
 * @ingroup tests
 * @brief Absorbs the S1-AP messages the gNB application sends towards the MME.
 *
 * Registering the UE goes out over S1-AP; nothing in this test depends on the
 * answer, only on the application having somewhere to send it.
 */
class RelayedTagS1apSapMme : public NrEpcS1apSapMme
{
  public:
    void InitialUeMessage(uint64_t mmeUeS1Id,
                          uint16_t gnbUeS1Id,
                          uint64_t stmsi,
                          uint16_t ecgi) override
    {
    }

    void ErabReleaseIndication(
        uint64_t mmeUeS1Id,
        uint16_t gnbUeS1Id,
        std::list<ErabToBeReleasedIndication> erabToBeReleaseIndication) override
    {
    }

    void InitialContextSetupResponse(uint64_t mmeUeS1Id,
                                     uint16_t gnbUeS1Id,
                                     std::list<ErabSetupItem> erabSetupList) override
    {
    }

    void PathSwitchRequest(
        uint64_t gnbUeS1Id,
        uint64_t mmeUeS1Id,
        uint16_t gci,
        std::list<ErabSwitchedInDownlinkItem> erabToBeSwitchedInDownlinkList) override
    {
    }

    void UeContextReleaseComplete(uint64_t mmeUeS1Id, uint16_t gnbUeS1Id) override
    {
    }
};

/**
 * @ingroup tests
 * @brief Reproduces the chain that used to abort the simulation with
 *        "cannot add the same kind of tag twice".
 *
 * A packet socket stamps a PacketSocketTag on every packet it delivers. The gNB
 * EPC application relays such a packet to the S1-U interface, and the tag used to
 * travel on with it: RLC segmentation copies packet tags onto every fragment, and
 * reassembly delivers the held first segment itself, keeping its packet tags. A
 * packet that came back to a packet socket therefore arrived already stamped, and
 * the second stamp aborted the run.
 *
 * The test walks that chain in order and checks the property the fix establishes:
 * what the gNB application relays carries no packet socket tag.
 */
class NrEpcGnbRelayedTagTestCase : public TestCase
{
  public:
    NrEpcGnbRelayedTagTestCase()
        : TestCase("A relayed packet carries no packet socket tag")
    {
    }

  private:
    void DoRun() override;

    /**
     * Record what the gNB application relayed towards S1-U.
     *
     * @param packet the relayed packet, as reported by the RxFromGnb trace
     */
    void RelayedToS1u(Ptr<Packet> packet);

    Ptr<Packet> m_relayed; ///< the packet the application relayed, if any
};

void
NrEpcGnbRelayedTagTestCase::RelayedToS1u(Ptr<Packet> packet)
{
    m_relayed = packet;
}

void
NrEpcGnbRelayedTagTestCase::DoRun()
{
    constexpr uint16_t rnti = 1;
    constexpr uint64_t imsi = 1;
    constexpr uint8_t qfi = 1;
    constexpr uint32_t teid = 100;

    // A sender and the gNB, joined by a plain link. The gNB end stands in for the
    // NR net device: what matters is that a real packet socket delivers the packet,
    // because that delivery is what attaches the tag in the first place.
    NodeContainer nodes;
    nodes.Create(2);
    SimpleNetDeviceHelper linkHelper;
    NetDeviceContainer devices = linkHelper.Install(nodes);
    PacketSocketHelper packetSocketHelper;
    packetSocketHelper.Install(nodes);
    // The S1-U side of the application is a UDP socket, so the gNB needs an IP stack.
    InternetStackHelper internet;
    internet.Install(nodes);
    Ipv4AddressHelper addresses;
    addresses.SetBase("10.0.0.0", "255.255.255.0");
    Ipv4InterfaceContainer interfaces = addresses.Assign(devices);

    PacketSocketAddress gnbAddress;
    gnbAddress.SetSingleDevice(devices.Get(1)->GetIfIndex());
    gnbAddress.SetPhysicalAddress(devices.Get(1)->GetAddress());
    gnbAddress.SetProtocol(1);

    Ptr<Socket> gnbSocket =
        Socket::CreateSocket(nodes.Get(1), TypeId::LookupByName("ns3::PacketSocketFactory"));
    NS_TEST_ASSERT_MSG_EQ(gnbSocket->Bind(gnbAddress), 0, "the gNB packet socket did not bind");

    Ptr<Socket> ueSocket =
        Socket::CreateSocket(nodes.Get(0), TypeId::LookupByName("ns3::PacketSocketFactory"));
    NS_TEST_ASSERT_MSG_EQ(ueSocket->Bind(), 0, "the sending packet socket did not bind");
    NS_TEST_ASSERT_MSG_EQ(ueSocket->Connect(gnbAddress), 0, "the sending socket did not connect");

    // The gNB application relays what arrives on that socket towards S1-U. Give it an
    // S1-U socket and a UE context, so the uplink path is complete and the relayed
    // packet is reported through the RxFromGnb trace.
    Ptr<Socket> s1uSocket =
        Socket::CreateSocket(nodes.Get(1), TypeId::LookupByName("ns3::UdpSocketFactory"));
    // The application binds a receive callback on all its NR sockets, so the IPv6 and the
    // unstructured ones have to exist even though this test only drives the IPv4 path.
    PacketSocketAddress gnbAddress6 = gnbAddress;
    gnbAddress6.SetProtocol(2);
    Ptr<Socket> gnbSocket6 =
        Socket::CreateSocket(nodes.Get(1), TypeId::LookupByName("ns3::PacketSocketFactory"));
    NS_TEST_ASSERT_MSG_EQ(gnbSocket6->Bind(gnbAddress6), 0, "the IPv6 packet socket did not bind");
    PacketSocketAddress gnbAddressUnstructured = gnbAddress;
    gnbAddressUnstructured.SetProtocol(NrEpcGnbApplication::UNSTRUCTURED_SOCKET_PROTOCOL);
    Ptr<Socket> gnbSocketUnstructured =
        Socket::CreateSocket(nodes.Get(1), TypeId::LookupByName("ns3::PacketSocketFactory"));
    NS_TEST_ASSERT_MSG_EQ(gnbSocketUnstructured->Bind(gnbAddressUnstructured),
                          0,
                          "the unstructured packet socket did not bind");

    Ptr<NrEpcGnbApplication> gnbApp =
        CreateObject<NrEpcGnbApplication>(gnbSocket, gnbSocket6, gnbSocketUnstructured, 1);
    nodes.Get(1)->AddApplication(gnbApp);

    RelayedTagS1SapUser s1SapUser;
    RelayedTagS1apSapMme s1apSapMme;
    gnbApp->SetS1SapUser(&s1SapUser);
    gnbApp->SetS1apSapMme(&s1apSapMme);
    gnbApp->AddS1Interface(s1uSocket, interfaces.GetAddress(1), interfaces.GetAddress(0));
    gnbApp->GetS1SapProvider()->InitialUeMessage(imsi, rnti);

    NrEpcS1apSapGnb::ErabToBeSetupItem erab;
    erab.erabId = qfi;
    erab.sgwTeid = teid;
    erab.transportLayerAddress = interfaces.GetAddress(0);
    std::list<NrEpcS1apSapGnb::ErabToBeSetupItem> erabs{erab};
    gnbApp->GetS1apSapGnb()->InitialContextSetupRequest(imsi, rnti, erabs);

    gnbApp->TraceConnectWithoutContext(
        "RxFromGnb",
        MakeCallback(&NrEpcGnbRelayedTagTestCase::RelayedToS1u, this));

    // Step 1 of the chain. The uplink packet carries the QoS flow tag the application
    // expects; the packet socket adds the delivery tag of its own accord.
    Ptr<Packet> uplink = Create<Packet>(600);
    uplink->AddPacketTag(NrQosFlowTag(rnti, qfi));
    NS_TEST_ASSERT_MSG_GT(ueSocket->Send(uplink), 0, "the uplink packet was not sent");

    Simulator::Run();

    // Step 2. The application relayed the packet onwards. Before the fix the delivery
    // tag was still on it here, which is where the leak began.
    NS_TEST_ASSERT_MSG_NE(m_relayed, nullptr, "the application relayed nothing to S1-U");
    PacketSocketTag relayedSocketTag;
    NS_TEST_ASSERT_MSG_EQ(m_relayed->PeekPacketTag(relayedSocketTag),
                          false,
                          "the relayed packet still carries the tag of a delivery that is over");
    DeviceNameTag relayedDeviceTag;
    NS_TEST_ASSERT_MSG_EQ(m_relayed->PeekPacketTag(relayedDeviceTag),
                          false,
                          "the relayed packet still carries the device name tag of that delivery");

    // Steps 3 and 4 are what made the leak reach far beyond the relay, and hold whether
    // or not the packet is tagged. Take a packet that is still stamped, as the relayed
    // one used to be, and follow it through segmentation and reassembly.
    Ptr<Packet> stamped = Create<Packet>(600);
    PacketSocketTag stamp;
    stamp.SetPacketType(NetDevice::PACKET_HOST);
    stamped->AddPacketTag(stamp);

    // Step 3. RLC segments the packet, and every fragment inherits its packet tags.
    Ptr<Packet> firstSegment = stamped->CreateFragment(0, 300);
    Ptr<Packet> secondSegment = stamped->CreateFragment(300, 300);
    PacketSocketTag segmentTag;
    NS_TEST_ASSERT_MSG_EQ(firstSegment->PeekPacketTag(segmentTag),
                          true,
                          "CreateFragment no longer copies packet tags onto the fragment");

    // Step 4. Reassembly concatenates the continuation onto the held first segment and
    // delivers that same object, exactly as NrRlc::ReassembleSdus does. AddAtEnd merges
    // buffers and byte tags only, so the held segment keeps its own packet tags and the
    // reassembled SDU is delivered still stamped.
    Ptr<Packet> heldSegment = firstSegment;
    heldSegment->AddAtEnd(secondSegment);
    PacketSocketTag reassembledTag;
    NS_TEST_ASSERT_MSG_EQ(heldSegment->PeekPacketTag(reassembledTag),
                          true,
                          "the reassembled SDU no longer inherits the tags of the held segment");

    // Step 5. A packet in that state is what reached a packet socket a second time, where
    // AddPacketTag found a tag of the same type already present and aborted the run. The
    // fix keeps a relayed packet from ever entering the chain in this state.

    Simulator::Destroy();
}

/**
 * @ingroup tests
 * @brief Test suite for packet socket tags on relayed packets.
 */
class NrEpcGnbRelayedTagTestSuite : public TestSuite
{
  public:
    NrEpcGnbRelayedTagTestSuite();
};

NrEpcGnbRelayedTagTestSuite::NrEpcGnbRelayedTagTestSuite()
    : TestSuite("nr-epc-gnb-relayed-tag", Type::UNIT)
{
    AddTestCase(new NrEpcGnbRelayedTagTestCase(), Duration::QUICK);
}

static NrEpcGnbRelayedTagTestSuite g_nrEpcGnbRelayedTagTestSuite; //!< Relayed tag test suite

} // namespace ns3
