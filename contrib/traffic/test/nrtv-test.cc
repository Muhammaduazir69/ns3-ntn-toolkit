/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2013 Magister Solutions Oy
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
 * Author: Budiarto Herman <budiarto.herman@magister.fi>
 *
 * Modified by: Patrice Raveneau <Patrice.raveneau@cnes.fr>
 *
 */

/**
 * @file nrtv-test.cc
 * @ingroup nrtv
 * @brief Test cases for NRTV traffic models, grouped in `nrtv` test suite.
 */

#include "ns3/application.h"
#include "ns3/config.h"
#include "ns3/data-rate.h"
#include "ns3/integer.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/ipv4-interface-container.h"
#include "ns3/log.h"
#include "ns3/net-device-container.h"
#include "ns3/node-container.h"
#include "ns3/nrtv-header.h"
#include "ns3/nrtv-helper.h"
#include "ns3/nstime.h"
#include "ns3/point-to-point-helper.h"
#include "ns3/simulator.h"
#include "ns3/string.h"
#include "ns3/tcp-socket-factory.h"
#include "ns3/test.h"
#include "ns3/type-id.h"
#include "ns3/udp-socket-factory.h"
#include "ns3/uinteger.h"

#include <list>
#include <sstream>

NS_LOG_COMPONENT_DEFINE("NrtvTest");

using namespace ns3; //{

/**
 * @ingroup applications
 * @brief Verifies whether the NRTV client Rx buffer properly re-assemble
 *        packets into video slices.
 *
 * Runs a simulation of an NRTV client connected to an NRTV server through a
 * simple point-to-point interface. The test case verifies that every video
 * slice sent by the server is received in the same size and order by the
 * client.
 */
class NrtvClientRxBufferTestCase : public TestCase
{
  public:
    /**
     * @brief Construct a new test case.
     * @param name the test case name, which will be printed on the report
     * @param rngRun the number of run to be used by the random number generator
     * @param protocolTypeId determines the socket type (TCP or UDP)
     * @param channelDelay fixed transmission delay to be set on the
     *                     point-to-point channel
     * @param duration length of simulation
     */
    NrtvClientRxBufferTestCase(std::string name,
                               uint32_t rngRun,
                               TypeId protocolTypeId,
                               Time channelDelay,
                               Time duration);

  private:
    virtual void DoRun();
    virtual void DoTeardown();

    // CALLBACK FUNCTIONS
    void TxCallback(std::string context, Ptr<const Packet> packet);
    void RxCallback(std::string context, Ptr<const Packet> packet, const Address& from);
    void RxSliceCallback(std::string context, Ptr<const Packet> packet);

    /// Size of packets currently in transit in the channel.
    std::list<uint32_t> m_packetsInTransit;
    uint32_t m_rngRun;
    TypeId m_protocolTypeId;
    Time m_channelDelay;
    Time m_duration;

}; // end of `class NrtvClientRxBufferTestCase`

NrtvClientRxBufferTestCase::NrtvClientRxBufferTestCase(std::string name,
                                                       uint32_t rngRun,
                                                       TypeId protocolTypeId,
                                                       Time channelDelay,
                                                       Time duration)
    : TestCase(name),
      m_rngRun(rngRun),
      m_protocolTypeId(protocolTypeId),
      m_channelDelay(channelDelay),
      m_duration(duration)
{
    NS_LOG_FUNCTION(this << name << rngRun);
}

void
NrtvClientRxBufferTestCase::DoRun()
{
    NS_LOG_FUNCTION(this << GetName() << m_rngRun);

    Config::SetGlobal("RngRun", UintegerValue(m_rngRun));
    Config::SetDefault("ns3::TcpL4Protocol::SocketType", StringValue("ns3::TcpNewReno"));

    NodeContainer nodes;
    nodes.Create(2);

    PointToPointHelper pointToPoint;
    pointToPoint.SetDeviceAttribute("DataRate", DataRateValue(DataRate("5Mbps")));
    pointToPoint.SetChannelAttribute("Delay", TimeValue(m_channelDelay));

    NetDeviceContainer devices;
    devices = pointToPoint.Install(nodes);

    InternetStackHelper stack;
    stack.Install(nodes);

    Ipv4AddressHelper address;
    address.SetBase("10.1.1.0", "255.255.255.0");
    Ipv4InterfaceContainer interfaces = address.Assign(devices);

    // bool udp = m_protocolTypeId == TypeId::LookupByName ("ns3::UdpSocketFactory");
    // NS_UNUSED (udp);

    NrtvHelper helper(m_protocolTypeId);
    helper.InstallUsingIpv4(nodes.Get(0), nodes.Get(1));
    Ptr<Application> server = helper.GetServer().Get(0);
    Ptr<Application> client = helper.GetClients().Get(0);
    server->SetStartTime(MilliSeconds(1));
    client->SetStartTime(MilliSeconds(2));
    server->TraceConnect("Tx",
                         m_protocolTypeId.GetName(),
                         MakeCallback(&NrtvClientRxBufferTestCase::TxCallback, this));
    client->TraceConnect("Rx",
                         m_protocolTypeId.GetName(),
                         MakeCallback(&NrtvClientRxBufferTestCase::RxCallback, this));
    client->TraceConnect("RxSlice",
                         m_protocolTypeId.GetName(),
                         MakeCallback(&NrtvClientRxBufferTestCase::RxSliceCallback, this));

    Simulator::Stop(m_duration);
    Simulator::Run();
    Simulator::Destroy();

    // return default values to their default
    Config::SetGlobal("RngRun", UintegerValue(1));

} // end of `void DoRun ()`

void
NrtvClientRxBufferTestCase::DoTeardown()
{
    NS_LOG_FUNCTION(this << GetName() << m_rngRun);
}

void
NrtvClientRxBufferTestCase::TxCallback(std::string context, Ptr<const Packet> packet)
{
    const uint32_t packetSize = packet->GetSize();
    NS_LOG_FUNCTION(this << packet << packetSize);
    m_packetsInTransit.push_back(packetSize);
}

void
NrtvClientRxBufferTestCase::RxCallback(std::string context,
                                       Ptr<const Packet> packet,
                                       const Address& from)
{
    const uint32_t packetSize = packet->GetSize();
    NS_LOG_FUNCTION(this << packet << packetSize);

    if (context == "ns3::TcpSocketFactory")
    {
        // TCP may deliver either compiled or split packets
        NS_ASSERT_MSG(m_packetsInTransit.size() > 0,
                      "Received a packet before any packet was transmitted before");

        if (m_packetsInTransit.front() != packetSize)
        {
            NS_LOG_INFO(this << " [" << GetName() << "]" << " some splitting had occurred,"
                             << " expected " << m_packetsInTransit.front() << " bytes"
                             << " but received " << packetSize << " bytes instead");
        }
    }
    else if (context == "ns3::UdpSocketFactory")
    {
        // UDP must deliver the entire slice in one packet, thus we handle slice check in
        // RxCallback.
        Ptr<Packet> copy = packet->Copy();
        NrtvHeader nrtvHeader;
        copy->RemoveHeader(nrtvHeader);
        const uint32_t sliceSize = nrtvHeader.GetSliceSize();
        NS_TEST_ASSERT_MSG_EQ(sliceSize,
                              copy->GetSize(),
                              "Inconsistent packet size at " << Simulator::Now().GetSeconds());
        NS_TEST_ASSERT_MSG_EQ(packetSize,
                              m_packetsInTransit.front(),
                              "Unexpected packet size at " << Simulator::Now().GetSeconds());
        m_packetsInTransit.pop_front();
    }
    else
    {
        NS_FATAL_ERROR("RxCallback: Unexpected context " << context);
    }
}

void
NrtvClientRxBufferTestCase::RxSliceCallback(std::string context, Ptr<const Packet> slice)
{
    const uint32_t packetSize = slice->GetSize();
    NS_LOG_FUNCTION(this << slice << packetSize);
    NS_ASSERT_MSG(m_packetsInTransit.size() > 0,
                  "Received a slice before any packet was transmitted before");

    Ptr<Packet> copy = slice->Copy();
    NrtvHeader nrtvHeader;
    copy->RemoveHeader(nrtvHeader);
    const uint32_t sliceSize = nrtvHeader.GetSliceSize();
    NS_TEST_ASSERT_MSG_EQ(sliceSize,
                          copy->GetSize(),
                          "Inconsistent packet size at " << Simulator::Now().GetSeconds());
    NS_TEST_ASSERT_MSG_EQ(packetSize,
                          m_packetsInTransit.front(),
                          "Unexpected packet size at " << Simulator::Now().GetSeconds());
    m_packetsInTransit.pop_front();
}

/**
 * @brief Test suite `nrtv`, verifying the NRTV traffic model.
 */
class NrtvTestSuite : public TestSuite
{
  public:
    NrtvTestSuite();
};

NrtvTestSuite::NrtvTestSuite()
    : TestSuite("nrtv", Type::SYSTEM)
{
    // LogComponentEnable ("NrtvTest", LOG_INFO);
    // LogComponentEnable ("NrtvTest", LOG_PREFIX_ALL);

    TypeId tcp = TcpSocketFactory::GetTypeId();
    TypeId udp = UdpSocketFactory::GetTypeId();
    const TypeId protocols[2] = {tcp, udp};
    const uint64_t delayMs[3] = {3, 30, 300};
    const uint32_t rngRun[4] = {1, 22, 333};

    for (uint8_t i = 0; i < 2; i++)
    {
        for (uint8_t j = 0; j < 3; j++)
        {
            for (uint8_t k = 0; k < 3; k++)
            {
                std::ostringstream oss;
                oss << protocols[i].GetName() << ", " << "delay=" << delayMs[j] << "ms, "
                    << "run=" << rngRun[k];
                AddTestCase(new NrtvClientRxBufferTestCase(oss.str(),
                                                           rngRun[k],
                                                           protocols[i],
                                                           MilliSeconds(delayMs[j]),
                                                           Seconds(5)),
                            TestCase::Duration::QUICK);
            }
        }
    }

} // end of `NrtvTestSuite ()`

static NrtvTestSuite g_nrtvTestSuiteInstance;

//} // end of namespace ns3
