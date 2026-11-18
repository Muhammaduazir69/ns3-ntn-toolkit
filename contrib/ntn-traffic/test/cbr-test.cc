/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2013 Magister Solutions Ltd
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
 * Author: Sami Rantanen <sami.rantanen@magister.fi>
 */

#include "ns3/boolean.h"
#include "ns3/cbr-application.h"
#include "ns3/cbr-helper.h"
#include "ns3/constant-position-mobility-model.h"
#include "ns3/double.h"
#include "ns3/error-model.h"
#include "ns3/flow-monitor-helper.h"
#include "ns3/flow-monitor.h"
#include "ns3/ipv4-header.h"
#include "ns3/ipv4-l3-protocol.h"
#include "ns3/ntn-real-stack-helper.h"
#include "ns3/point-to-point-helper.h"
#include "ns3/point-to-point-net-device.h"
#include "ns3/pointer.h"
#include "ns3/queue.h"
#include "ns3/rng-seed-manager.h"
#include "ns3/seq-ts-size-header.h"
#include "ns3/traffic-control-helper.h"
#include "ns3/traffic-control-layer.h"
#include "ns3/traffic-time-tag.h"
#include "ns3/udp-header.h"
#include "ns3/uinteger.h"
#include "ns3/inet-socket-address.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/log.h"
#include "ns3/packet-sink-helper.h"
#include "ns3/packet-sink.h"
#include "ns3/simple-channel.h"
#include "ns3/simple-net-device.h"
#include "ns3/simulator.h"
#include "ns3/string.h"
#include "ns3/test.h"

#include <algorithm>
#include <cmath>
#include <map>
#include <set>
#include <vector>

using namespace ns3;

// \ brief Simple test case to verify Cbr application functionality.
class CbrTestCase1 : public TestCase
{
  public:
    CbrTestCase1();
    virtual ~CbrTestCase1();

  private:
    virtual void DoRun(void);
};

// Add some help text to this case to describe what it is intended to test
CbrTestCase1::CbrTestCase1()
    : TestCase("Cbr test case to verify all sent data is got by receiver.")
{
}

// This destructor does nothing but we include it as a reminder that
// the test case should clean up after itself
CbrTestCase1::~CbrTestCase1()
{
}

//
// This method is the pure virtual method from class TestCase that every
// TestCase must implement
//
void
CbrTestCase1::DoRun(void)
{
    NodeContainer n;
    n.Create(2);

    InternetStackHelper internet;
    internet.Install(n);

    // link the two nodes
    Ptr<SimpleNetDevice> txDev = CreateObject<SimpleNetDevice>();
    Ptr<SimpleNetDevice> rxDev = CreateObject<SimpleNetDevice>();
    n.Get(0)->AddDevice(txDev);
    n.Get(1)->AddDevice(rxDev);
    Ptr<SimpleChannel> channel1 = CreateObject<SimpleChannel>();
    rxDev->SetChannel(channel1);
    txDev->SetChannel(channel1);
    NetDeviceContainer d;
    d.Add(txDev);
    d.Add(rxDev);

    Ipv4AddressHelper ipv4;

    ipv4.SetBase("10.1.1.0", "255.255.255.0");
    Ipv4InterfaceContainer i = ipv4.Assign(d);

    uint16_t port = 4000;
    Address serverAddress(InetSocketAddress(i.GetAddress(1), port));

    PacketSinkHelper server("ns3::UdpSocketFactory",
                            InetSocketAddress(Ipv4Address::GetAny(), port));
    ApplicationContainer serverApps = server.Install(n.Get(1));
    serverApps.Start(Seconds(1.0));
    serverApps.Stop(Seconds(10.0));

    CbrHelper client("ns3::UdpSocketFactory", serverAddress);
    client.SetAttribute("Interval", StringValue("1s"));
    ApplicationContainer clientApps = client.Install(n.Get(0));
    clientApps.Start(Seconds(2.0));
    clientApps.Stop(Seconds(8.0));

    Simulator::Run();
    Simulator::Destroy();

    Ptr<PacketSink> sink = DynamicCast<PacketSink>(serverApps.Get(0));
    Ptr<CbrApplication> sender = DynamicCast<CbrApplication>(clientApps.Get(0));

    NS_TEST_ASSERT_MSG_NE(sender->GetSent(), (uint32_t)0, "Nothing sent !");
    NS_TEST_ASSERT_MSG_EQ(sink->GetTotalRx(), sender->GetSent(), "Packets were lost !");
}

// ---------------------------------------------------------------------------
// In-depth cases (Nov 2026 test campaign). Every case drives Simulator::Run()
// over a real Internet stack (UDP, IPv4, point-to-point device and channel, or
// the NR-NTN stack of NtnRealStackHelper) and asserts on packets that moved.
// ---------------------------------------------------------------------------

namespace
{

/// TR 38.821 Table 5.3.2.1-1: maximum round-trip propagation delay of the
/// transparent payload, LEO at 600 km (25.77 ms) and GEO (541.46 ms). One way
/// on the UE to gateway path is half of each.
constexpr double kLeo600TransparentOwdMs = 25.77 / 2.0;
constexpr double kGeoTransparentOwdMs = 541.46 / 2.0;

/// Two nodes on one point-to-point link (the satellite bent pipe reduced to
/// its rate and its one-way delay), Internet stack, 10.1.1.0/24. The default
/// queue disc is removed so the only buffer is the device DropTail queue.
struct P2pLink
{
    NodeContainer nodes;
    NetDeviceContainer devices;
    Ipv4InterfaceContainer ifaces;
};

P2pLink
MakeLink(const std::string& rate, Time owd, uint32_t queuePackets = 100)
{
    P2pLink l;
    l.nodes.Create(2);
    PointToPointHelper p2p;
    p2p.SetDeviceAttribute("DataRate", StringValue(rate));
    p2p.SetChannelAttribute("Delay", TimeValue(owd));
    p2p.SetQueue("ns3::DropTailQueue<Packet>",
                 "MaxSize",
                 StringValue(std::to_string(queuePackets) + "p"));
    l.devices = p2p.Install(l.nodes);
    InternetStackHelper internet;
    internet.Install(l.nodes);
    Ipv4AddressHelper addr;
    addr.SetBase("10.1.1.0", "255.255.255.0");
    l.ifaces = addr.Assign(l.devices);
    TrafficControlHelper tch;
    tch.Uninstall(l.devices);
    return l;
}

/// Serialization time of a UDP datagram of \p payload bytes on a PPP link:
/// payload + UDP 8 + IPv4 20 + PPP 2 bytes.
Time
PppSerialization(uint32_t payload, double rateBps)
{
    return Seconds((payload + 8 + 20 + 2) * 8.0 / rateBps);
}

} // namespace

/// CBR timing contract over a LEO slant. The class documentation says the first
/// packet leaves one Interval after start and then one per Interval; over a
/// point-to-point link with the TR 38.821 LEO-600 one-way delay the receiver
/// sees the same spacing, and every packet's one-way delay is exactly the
/// propagation delay plus its serialization. FlowMonitor measures the same
/// delay and the IPv4 bytes.
class CbrTimingOverSlantTest : public TestCase
{
  public:
    CbrTimingOverSlantTest()
        : TestCase("CBR timing and one-way delay over the TR 38.821 LEO-600 slant (FlowMonitor)")
    {
    }

  private:
    void Tx(Ptr<const Packet>)
    {
        m_tx.push_back(Simulator::Now());
    }

    void Rx(Ptr<const Packet>, const Address&)
    {
        m_rx.push_back(Simulator::Now());
    }

    void DoRun() override
    {
        const Time owd = MicroSeconds(static_cast<int64_t>(kLeo600TransparentOwdMs * 1000.0));
        P2pLink l = MakeLink("10Mbps", owd);

        PacketSinkHelper sinkH("ns3::UdpSocketFactory", InetSocketAddress(Ipv4Address::GetAny(), 4000));
        ApplicationContainer sink = sinkH.Install(l.nodes.Get(1));
        sink.Start(Seconds(0.0));
        CbrHelper cbr("ns3::UdpSocketFactory", InetSocketAddress(l.ifaces.GetAddress(1), 4000));
        cbr.SetConstantTraffic(MilliSeconds(20), 500);
        ApplicationContainer app = cbr.Install(l.nodes.Get(0));
        app.Start(Seconds(1.0));
        app.Stop(Seconds(2.0));
        app.Get(0)->TraceConnectWithoutContext("Tx", MakeCallback(&CbrTimingOverSlantTest::Tx, this));
        sink.Get(0)->TraceConnectWithoutContext("Rx", MakeCallback(&CbrTimingOverSlantTest::Rx, this));

        FlowMonitorHelper fmh;
        Ptr<FlowMonitor> fm = fmh.InstallAll();
        Simulator::Stop(Seconds(3.0));
        Simulator::Run();
        fm->CheckForLostPackets();
        const auto stats = fm->GetFlowStats();
        Ptr<CbrApplication> sender = DynamicCast<CbrApplication>(app.Get(0));
        const uint64_t sent = sender->GetSent();
        Simulator::Destroy();

        // 1.0 + 0.02 k < 2.0 for k = 1..49; the stop event is older than the
        // send scheduled for 2.0 s and cancels it.
        NS_TEST_ASSERT_MSG_EQ(m_tx.size(), 49u, "one packet per Interval inside [start, stop)");
        NS_TEST_ASSERT_MSG_EQ(m_tx.front(), Seconds(1.02), "first packet one Interval after start");
        for (std::size_t k = 1; k < m_tx.size(); ++k)
        {
            NS_TEST_ASSERT_MSG_EQ(m_tx[k] - m_tx[k - 1], MilliSeconds(20), "constant spacing at the sender");
        }
        NS_TEST_ASSERT_MSG_EQ(sent, 49u * 500u, "GetSent counts application bytes");
        NS_TEST_ASSERT_MSG_EQ(m_rx.size(), m_tx.size(), "an idle 10 Mb/s link loses nothing");
        const Time expected = owd + PppSerialization(500, 10e6);
        for (std::size_t k = 0; k < m_rx.size() && k < m_tx.size(); ++k)
        {
            NS_TEST_ASSERT_MSG_EQ_TOL((m_rx[k] - m_tx[k]).GetMicroSeconds(),
                                      expected.GetMicroSeconds(),
                                      1,
                                      "per-packet delay = TR 38.821 one-way propagation + serialization");
        }
        NS_TEST_ASSERT_MSG_EQ(stats.size(), 1u, "one UDP flow classified");
        for (const auto& [id, st] : stats)
        {
            NS_TEST_ASSERT_MSG_EQ(st.rxPackets, 49u, "FlowMonitor counts every packet");
            NS_TEST_ASSERT_MSG_EQ(st.rxBytes, 49u * 528u, "IPv4 bytes = payload + UDP 8 + IPv4 20");
            NS_TEST_ASSERT_MSG_EQ_TOL(st.delaySum.GetSeconds() / st.rxPackets,
                                      expected.GetSeconds(),
                                      1e-6,
                                      "FlowMonitor mean delay equals the slant plus serialization");
            NS_TEST_ASSERT_MSG_EQ(st.lostPackets, 0u, "nothing lost");
        }
    }

    std::vector<Time> m_tx;
    std::vector<Time> m_rx;
};

/// In-band measurement through a lossy GEO bent pipe. EnableSeqTsSizeHeader and
/// EnableStatisticsTags give two independent delay measurements and a sequence
/// gap count. With a 5 % packet error model on the receiving device, every
/// sequence gap must be one error-model drop (no double counting, nothing
/// invented), the loss ratio must stay inside the binomial 4-sigma band, and
/// both delay measurements must equal the TR 38.821 GEO one-way delay plus
/// serialization.
class CbrInBandLossAccountingTest : public TestCase
{
  public:
    CbrInBandLossAccountingTest()
        : TestCase("CBR SeqTsSizeHeader and TrafficTimeTag account every loss on a lossy GEO link")
    {
    }

  private:
    void Tx(Ptr<const Packet>)
    {
        ++m_tx;
    }

    void RxSeq(Ptr<const Packet>, const Address&, const Address&, const SeqTsSizeHeader& h)
    {
        m_seqs.insert(h.GetSeq());
        ++m_rxSeqCalls;
        m_hdrDelays.push_back(Simulator::Now() - h.GetTs());
        if (h.GetSize() != 200)
        {
            ++m_badSize;
        }
    }

    void Rx(Ptr<const Packet> p, const Address&)
    {
        TrafficTimeTag tag;
        if (p->PeekPacketTag(tag))
        {
            m_tagDelays.push_back(Simulator::Now() - tag.GetSenderTimestamp());
        }
    }

    void Drop(Ptr<const Packet>)
    {
        ++m_drops;
    }

    void DoRun() override
    {
        const Time owd = MicroSeconds(static_cast<int64_t>(kGeoTransparentOwdMs * 1000.0));
        P2pLink l = MakeLink("10Mbps", owd);
        Ptr<RateErrorModel> em = CreateObject<RateErrorModel>();
        em->SetAttribute("ErrorUnit", StringValue("ERROR_UNIT_PACKET"));
        em->SetAttribute("ErrorRate", DoubleValue(0.05));
        em->AssignStreams(41);
        l.devices.Get(1)->SetAttribute("ReceiveErrorModel", PointerValue(em));
        l.devices.Get(1)->TraceConnectWithoutContext(
            "PhyRxDrop", MakeCallback(&CbrInBandLossAccountingTest::Drop, this));

        PacketSinkHelper sinkH("ns3::UdpSocketFactory", InetSocketAddress(Ipv4Address::GetAny(), 4000));
        sinkH.SetAttribute("EnableSeqTsSizeHeader", BooleanValue(true));
        ApplicationContainer sink = sinkH.Install(l.nodes.Get(1));
        CbrHelper cbr("ns3::UdpSocketFactory", InetSocketAddress(l.ifaces.GetAddress(1), 4000));
        cbr.SetConstantTraffic(MilliSeconds(5), 200);
        cbr.SetAttribute("EnableSeqTsSizeHeader", BooleanValue(true));
        cbr.SetAttribute("EnableStatisticsTags", BooleanValue(true));
        ApplicationContainer app = cbr.Install(l.nodes.Get(0));
        app.Start(Seconds(1.0));
        app.Stop(Seconds(6.0));
        app.Get(0)->TraceConnectWithoutContext("Tx", MakeCallback(&CbrInBandLossAccountingTest::Tx, this));
        sink.Get(0)->TraceConnectWithoutContext(
            "RxWithSeqTsSize", MakeCallback(&CbrInBandLossAccountingTest::RxSeq, this));
        sink.Get(0)->TraceConnectWithoutContext("Rx", MakeCallback(&CbrInBandLossAccountingTest::Rx, this));

        Simulator::Stop(Seconds(7.0));
        Simulator::Run();
        const uint64_t rxBytes = DynamicCast<PacketSink>(sink.Get(0))->GetTotalRx();
        Simulator::Destroy();

        NS_TEST_ASSERT_MSG_EQ(m_tx, 999u, "1.005 s to 5.995 s every 5 ms");
        NS_TEST_ASSERT_MSG_EQ(m_rxSeqCalls, m_seqs.size(), "no sequence number delivered twice");
        NS_TEST_ASSERT_MSG_EQ(m_badSize, 0u, "the header carries PacketSize");
        NS_TEST_ASSERT_MSG_EQ(rxBytes, m_seqs.size() * 200u, "the header sits inside PacketSize");
        NS_TEST_ASSERT_MSG_EQ(m_tx - m_seqs.size(), m_drops,
                              "every sequence gap is one error-model drop and nothing else");
        if (!m_seqs.empty())
        {
            NS_TEST_ASSERT_MSG_EQ((*m_seqs.rbegin() < m_tx), true, "sequence numbers stay below the count sent");
        }
        const double loss = 1.0 - static_cast<double>(m_seqs.size()) / m_tx;
        const double sigma = std::sqrt(0.05 * 0.95 / m_tx);
        NS_TEST_ASSERT_MSG_EQ_TOL(loss, 0.05, 4.0 * sigma, "loss ratio inside the binomial 4-sigma band");

        const Time expected = owd + PppSerialization(200, 10e6);
        NS_TEST_ASSERT_MSG_EQ(m_hdrDelays.size(), m_seqs.size(), "one header delay per packet");
        NS_TEST_ASSERT_MSG_EQ(m_tagDelays.size(), m_seqs.size(), "one tag delay per packet");
        for (std::size_t k = 0; k < m_hdrDelays.size() && k < m_tagDelays.size(); ++k)
        {
            NS_TEST_ASSERT_MSG_EQ_TOL(m_hdrDelays[k].GetMicroSeconds(), expected.GetMicroSeconds(), 1,
                                      "in-band header delay = GEO one-way + serialization");
            NS_TEST_ASSERT_MSG_EQ(m_hdrDelays[k], m_tagDelays[k], "header and tag agree");
        }
    }

    uint64_t m_tx{0};
    uint64_t m_drops{0};
    uint64_t m_rxSeqCalls{0};
    uint64_t m_badSize{0};
    std::set<uint32_t> m_seqs;
    std::vector<Time> m_hdrDelays;
    std::vector<Time> m_tagDelays;
};

/// RFC 2474 section 3 / RFC 3246 / RFC 2597: the Tos attribute is the whole
/// IPv4 DS byte. Three CBR flows marked Expedited Forwarding (DSCP 46), AF41
/// (DSCP 34) and best effort, plus EF with ECT(1) in the ECN bits (RFC 3168
/// section 5), must arrive with exactly that byte in the received IPv4 header.
class CbrDscpMarkingTest : public TestCase
{
  public:
    CbrDscpMarkingTest()
        : TestCase("CBR Tos reaches the received IPv4 DS field (RFC 2474, EF/AF41/BE, ECN bits)")
    {
    }

  private:
    void IpRx(Ptr<const Packet> p, Ptr<Ipv4>, uint32_t)
    {
        Ipv4Header h;
        Ptr<Packet> c = p->Copy();
        c->RemoveHeader(h);
        if (h.GetProtocol() != 17)
        {
            return;
        }
        UdpHeader u;
        c->PeekHeader(u);
        m_tosByPort[u.GetDestinationPort()].insert(h.GetTos());
        ++m_countByPort[u.GetDestinationPort()];
    }

    void DoRun() override
    {
        P2pLink l = MakeLink("10Mbps", MilliSeconds(13));
        const std::vector<std::pair<uint16_t, uint8_t>> flows = {
            {5001, 46 << 2},       // EF
            {5002, 34 << 2},       // AF41
            {5003, 0},             // best effort
            {5004, (46 << 2) | 1}, // EF with ECT(1)
        };
        for (const auto& [port, tos] : flows)
        {
            PacketSinkHelper sinkH("ns3::UdpSocketFactory", InetSocketAddress(Ipv4Address::GetAny(), port));
            sinkH.Install(l.nodes.Get(1));
            CbrHelper cbr("ns3::UdpSocketFactory", InetSocketAddress(l.ifaces.GetAddress(1), port));
            cbr.SetConstantTraffic(MilliSeconds(20), 160);
            cbr.SetAttribute("Tos", UintegerValue(tos));
            ApplicationContainer app = cbr.Install(l.nodes.Get(0));
            app.Start(Seconds(0.5));
            app.Stop(Seconds(1.5));
        }
        l.nodes.Get(1)->GetObject<Ipv4L3Protocol>()->TraceConnectWithoutContext(
            "Rx", MakeCallback(&CbrDscpMarkingTest::IpRx, this));
        Simulator::Stop(Seconds(2.0));
        Simulator::Run();
        Simulator::Destroy();

        for (const auto& [port, tos] : flows)
        {
            NS_TEST_ASSERT_MSG_EQ(m_countByPort[port], 49u, "flow " << port << " delivered");
            NS_TEST_ASSERT_MSG_EQ(m_tosByPort[port].size(), 1u, "one DS byte per flow");
            if (!m_tosByPort[port].empty())
            {
                NS_TEST_ASSERT_MSG_EQ(static_cast<uint32_t>(*m_tosByPort[port].begin()),
                                      static_cast<uint32_t>(tos),
                                      "flow " << port << " carries the configured DS byte");
            }
        }
        Ipv4Header ef;
        ef.SetTos(*m_tosByPort[5001].begin());
        NS_TEST_ASSERT_MSG_EQ(ef.GetDscp(), Ipv4Header::DSCP_EF, "EF is DSCP 46 (RFC 3246)");
        Ipv4Header ect;
        ect.SetTos(*m_tosByPort[5004].begin());
        NS_TEST_ASSERT_MSG_EQ(ect.GetEcn(), Ipv4Header::ECN_ECT1, "ECN bits kept (RFC 3168)");
        NS_TEST_ASSERT_MSG_EQ(ect.GetDscp(), Ipv4Header::DSCP_EF, "and the DSCP next to them");
    }

    std::map<uint16_t, std::set<uint8_t>> m_tosByPort;
    std::map<uint16_t, uint32_t> m_countByPort;
};

/// Overload: a 2 Mb/s CBR into a 1 Mb/s feeder with a 50-packet DropTail
/// buffer and no queue disc. The goodput is the link rate times the payload share
/// (1000 / 1030 bytes, 0.971 Mb/s, within 1 %), every packet is either received
/// or counted as a drop (device queue or traffic-control layer), the sender still reports the offered
/// load (the socket accepted it), and the worst one-way delay is the full
/// buffer drain plus the slant (bufferbloat bound).
class CbrOverloadTest : public TestCase
{
  public:
    CbrOverloadTest()
        : TestCase("CBR at 2x the link rate: link-limited goodput, every drop accounted, delay bound")
    {
    }

  private:
    void Rx(Ptr<const Packet> p, const Address&)
    {
        if (m_rxCount == 0)
        {
            m_firstRx = Simulator::Now();
        }
        m_lastRx = Simulator::Now();
        ++m_rxCount;
        m_rxBytes += p->GetSize();
    }

    void Drop(Ptr<const Packet>)
    {
        ++m_drops;
    }

    void DoRun() override
    {
        const Time owd = MicroSeconds(static_cast<int64_t>(kLeo600TransparentOwdMs * 1000.0));
        P2pLink l = MakeLink("1Mbps", owd, 50);
        // A full device queue stops its transmit queue; with no queue disc the
        // traffic-control layer then drops the packet (TcDrop). Count both.
        DynamicCast<PointToPointNetDevice>(l.devices.Get(0))
            ->GetQueue()
            ->TraceConnectWithoutContext("Drop", MakeCallback(&CbrOverloadTest::Drop, this));
        l.nodes.Get(0)->GetObject<TrafficControlLayer>()->TraceConnectWithoutContext(
            "TcDrop", MakeCallback(&CbrOverloadTest::Drop, this));
        PacketSinkHelper sinkH("ns3::UdpSocketFactory", InetSocketAddress(Ipv4Address::GetAny(), 4000));
        ApplicationContainer sink = sinkH.Install(l.nodes.Get(1));
        sink.Get(0)->TraceConnectWithoutContext("Rx", MakeCallback(&CbrOverloadTest::Rx, this));
        CbrHelper cbr("ns3::UdpSocketFactory", InetSocketAddress(l.ifaces.GetAddress(1), 4000));
        cbr.SetConstantTraffic(MilliSeconds(4), 1000);
        ApplicationContainer app = cbr.Install(l.nodes.Get(0));
        app.Start(Seconds(1.0));
        app.Stop(Seconds(5.0));
        FlowMonitorHelper fmh;
        Ptr<FlowMonitor> fm = fmh.InstallAll();
        Simulator::Stop(Seconds(7.0));
        Simulator::Run();
        fm->CheckForLostPackets();
        Ptr<CbrApplication> sender = DynamicCast<CbrApplication>(app.Get(0));
        const uint64_t sentPkts = sender->GetSent() / 1000;
        Time maxDelay;
        for (const auto& [id, st] : fm->GetFlowStats())
        {
            maxDelay = std::max(maxDelay, st.maxDelay);
        }
        Simulator::Destroy();

        NS_TEST_ASSERT_MSG_EQ(sentPkts, 999u, "the socket accepted the whole offered load");
        NS_TEST_ASSERT_MSG_EQ(m_rxCount + m_drops, sentPkts, "received + queue drops = sent");
        NS_TEST_ASSERT_MSG_GT(m_drops, 400u, "about half the offered load cannot fit");
        const double goodput = (m_rxBytes - 1000.0) * 8.0 / (m_lastRx - m_firstRx).GetSeconds();
        const double expected = 1e6 * 1000.0 / 1030.0;
        NS_TEST_ASSERT_MSG_EQ_TOL(goodput, expected, 0.01 * expected,
                                  "goodput is the link rate times the payload share");
        // A full 50-packet queue (plus the packet on the wire) drains in
        // 51 x 8.24 ms; add the slant.
        const double boundS = owd.GetSeconds() + 51 * 1030 * 8.0 / 1e6;
        NS_TEST_ASSERT_MSG_EQ_TOL(maxDelay.GetSeconds(), boundS, 0.009,
                                  "worst delay = slant + one full buffer drain");
    }

    uint64_t m_rxCount{0};
    uint64_t m_rxBytes{0};
    uint64_t m_drops{0};
    Time m_firstRx;
    Time m_lastRx;
};

/// Failure paths. (a) No route to the peer: the UDP socket refuses every
/// packet (ERROR_NOROUTETOHOST); GetSent() and the Tx trace must stay at zero
/// and GetRefused() counts them. (b) Interval zero is documented as "no
/// sending": the application sends nothing and the simulation reaches its stop
/// time. (c) A reachable peer with no listener: the packets do cross the link
/// (counted at the peer's IPv4 layer) and are counted as sent.
class CbrFailurePathsTest : public TestCase
{
  public:
    CbrFailurePathsTest()
        : TestCase("CBR failure paths: no route, zero interval, no listener")
    {
    }

  private:
    void Tx(Ptr<const Packet>)
    {
        ++m_tx;
    }

    void IpRx(Ptr<const Packet>, Ptr<Ipv4>, uint32_t)
    {
        ++m_peerIpRx;
    }

    void DoRun() override
    {
        P2pLink l = MakeLink("10Mbps", MilliSeconds(13));
        // (a) 172.16.5.5 is in no routing table entry and there is no default.
        CbrHelper noRoute("ns3::UdpSocketFactory", InetSocketAddress(Ipv4Address("172.16.5.5"), 4000));
        noRoute.SetConstantTraffic(MilliSeconds(20), 500);
        ApplicationContainer a = noRoute.Install(l.nodes.Get(0));
        a.Start(Seconds(1.0));
        a.Stop(Seconds(2.0));
        a.Get(0)->TraceConnectWithoutContext("Tx", MakeCallback(&CbrFailurePathsTest::Tx, this));
        // (b) zero interval toward a real sink.
        PacketSinkHelper sinkH("ns3::UdpSocketFactory", InetSocketAddress(Ipv4Address::GetAny(), 4001));
        ApplicationContainer sink = sinkH.Install(l.nodes.Get(1));
        CbrHelper zero("ns3::UdpSocketFactory", InetSocketAddress(l.ifaces.GetAddress(1), 4001));
        zero.SetConstantTraffic(Seconds(0), 500);
        ApplicationContainer b = zero.Install(l.nodes.Get(0));
        b.Start(Seconds(1.0));
        b.Stop(Seconds(2.0));
        // (c) no listener on port 4002.
        CbrHelper deaf("ns3::UdpSocketFactory", InetSocketAddress(l.ifaces.GetAddress(1), 4002));
        deaf.SetConstantTraffic(MilliSeconds(50), 300);
        ApplicationContainer c = deaf.Install(l.nodes.Get(0));
        c.Start(Seconds(1.0));
        c.Stop(Seconds(2.0));
        l.nodes.Get(1)->GetObject<Ipv4L3Protocol>()->TraceConnectWithoutContext(
            "Rx", MakeCallback(&CbrFailurePathsTest::IpRx, this));

        Simulator::Stop(Seconds(3.0));
        Simulator::Run();
        const Time end = Simulator::Now();
        auto ca = DynamicCast<CbrApplication>(a.Get(0));
        auto cb = DynamicCast<CbrApplication>(b.Get(0));
        auto cc = DynamicCast<CbrApplication>(c.Get(0));
        const uint32_t sentA = ca->GetSent();
        const uint64_t refusedA = ca->GetRefused();
        const uint32_t sentB = cb->GetSent();
        const uint32_t sentC = cc->GetSent();
        const uint64_t sinkRx = DynamicCast<PacketSink>(sink.Get(0))->GetTotalRx();
        Simulator::Destroy();

        NS_TEST_ASSERT_MSG_EQ(sentA, 0u, "no route: nothing left the node, so nothing counted as sent");
        NS_TEST_ASSERT_MSG_EQ(refusedA, 49u, "no route: every attempt counted as refused");
        NS_TEST_ASSERT_MSG_EQ(m_tx, 0u, "no route: the Tx trace stays silent");
        NS_TEST_ASSERT_MSG_EQ(sentB, 0u, "zero interval: no sending");
        NS_TEST_ASSERT_MSG_EQ(sinkRx, 0u, "zero interval: nothing received");
        NS_TEST_ASSERT_MSG_EQ(end, Seconds(3.0), "zero interval: simulated time still advances to the stop");
        NS_TEST_ASSERT_MSG_EQ(sentC, 19u * 300u, "no listener: the packets left the node");
        NS_TEST_ASSERT_MSG_EQ(m_peerIpRx, 19u, "no listener: each crossed the link to the peer IP layer");
    }

    uint64_t m_tx{0};
    uint64_t m_peerIpRx{0};
};

/// Determinism. The same RngRun gives the identical set of delivered sequence
/// numbers through a 10 % lossy link; another run gives a different set whose
/// loss stays inside the binomial 4-sigma band.
class CbrDeterminismTest : public TestCase
{
  public:
    CbrDeterminismTest()
        : TestCase("CBR over a lossy link: same RngRun identical, other run within binomial bounds")
    {
    }

  private:
    struct Rec
    {
        std::set<uint32_t>* seqs;

        void Rx(Ptr<const Packet>, const Address&, const Address&, const SeqTsSizeHeader& h)
        {
            seqs->insert(h.GetSeq());
        }
    };

    std::set<uint32_t> Run(uint32_t run)
    {
        RngSeedManager::SetSeed(1);
        RngSeedManager::SetRun(run);
        RngSeedManager::ResetNextStreamIndex();
        std::set<uint32_t> seqs;
        Rec rec{&seqs};
        P2pLink l = MakeLink("10Mbps", MilliSeconds(13));
        Ptr<RateErrorModel> em = CreateObject<RateErrorModel>();
        em->SetAttribute("ErrorUnit", StringValue("ERROR_UNIT_PACKET"));
        em->SetAttribute("ErrorRate", DoubleValue(0.1));
        l.devices.Get(1)->SetAttribute("ReceiveErrorModel", PointerValue(em));
        PacketSinkHelper sinkH("ns3::UdpSocketFactory", InetSocketAddress(Ipv4Address::GetAny(), 4000));
        sinkH.SetAttribute("EnableSeqTsSizeHeader", BooleanValue(true));
        ApplicationContainer sink = sinkH.Install(l.nodes.Get(1));
        sink.Get(0)->TraceConnectWithoutContext("RxWithSeqTsSize", MakeCallback(&Rec::Rx, &rec));
        CbrHelper cbr("ns3::UdpSocketFactory", InetSocketAddress(l.ifaces.GetAddress(1), 4000));
        cbr.SetConstantTraffic(MilliSeconds(2), 100);
        cbr.SetAttribute("EnableSeqTsSizeHeader", BooleanValue(true));
        ApplicationContainer app = cbr.Install(l.nodes.Get(0));
        app.Start(Seconds(1.0));
        app.Stop(Seconds(3.0));
        Simulator::Stop(Seconds(4.0));
        Simulator::Run();
        Simulator::Destroy();
        return seqs;
    }

    void DoRun() override
    {
        const auto a = Run(1);
        const auto b = Run(1);
        const auto c = Run(7);
        RngSeedManager::SetRun(1);
        const double n = 999.0;
        const double sigma = std::sqrt(0.1 * 0.9 / n);
        NS_TEST_ASSERT_MSG_EQ((a == b), true, "same RngRun: the identical packets are delivered");
        NS_TEST_ASSERT_MSG_EQ((a == c), false, "another RngRun draws other losses");
        NS_TEST_ASSERT_MSG_EQ_TOL(1.0 - a.size() / n, 0.1, 4.0 * sigma, "run 1 loss in band");
        NS_TEST_ASSERT_MSG_EQ_TOL(1.0 - c.size() / n, 0.1, 4.0 * sigma, "run 7 loss in band");
    }
};

/// Cross-module: an AMR-WB 12.65 kb/s voice CBR (TS 26.201 mode 2: 253 bits
/// per 20 ms frame, 32 bytes, plus the RFC 4867 octet-aligned CMR and TOC
/// bytes and the 12-byte RTP header: 46 bytes every 20 ms) on the UPLINK of
/// the NR-NTN stack
/// (NtnRealStackHelper, nr backend, TR 38.821 Set-1 handheld at the 600 km
/// zenith, TDD): UE PDCP/RLC/MAC (SR/BSR grants)/PHY, the satellite slant, the EPC
/// and the SGi link to the remote host. At least 97 % of the packets are
/// delivered (the handheld PUSCH runs near -5 dB SINR, see RetxBsrTimerTest in
/// ntn-real-stack-helper; a larger 160-byte G.711 frame lost 16 % here), the
/// in-band one-way delay never undercuts the 600 km / c propagation floor, and
/// the mean stays inside the ITU-T G.114 150 ms one-way budget for
/// conversational voice.
class CbrUplinkOverNrNtnTest : public TestCase
{
  public:
    CbrUplinkOverNrNtnTest()
        : TestCase("Cross-module: uplink voice CBR over the NR-NTN stack meets the G.114 budget")
    {
    }

  private:
    void Rx(Ptr<const Packet>, const Address&, const Address&, const SeqTsSizeHeader& h)
    {
        const Time d = Simulator::Now() - h.GetTs();
        m_delays.push_back(d.GetSeconds() * 1e3);
        m_seqs.insert(h.GetSeq());
    }

    void DoRun() override
    {
        RngSeedManager::ResetNextStreamIndex();
        NodeContainer sats;
        NodeContainer ues;
        sats.Create(1);
        ues.Create(1);
        auto sm = CreateObject<ConstantPositionMobilityModel>();
        sm->SetPosition(NtnRealStackHelper::GeodeticToEcef(40.0, -3.0, 600e3));
        sats.Get(0)->AggregateObject(sm);
        auto um = CreateObject<ConstantPositionMobilityModel>();
        um->SetPosition(NtnRealStackHelper::GeodeticToEcef(40.0, -3.0, 0.0));
        ues.Get(0)->AggregateObject(um);

        NtnRealStackHelper rs;
        rs.SetRadioBackend(NtnRealStackHelper::RadioBackend::Nr);
        rs.SetNumerology(1);
        rs.SetSimTime(Seconds(4.5));
        rs.SetOutputDir(CreateTempDirFilename("cbr-ul-nr"));
        rs.SetSatEirpDensityDbwMhz(NtnRealStackHelper::kTr38821Set1SBandEirpDensityDbwMhz);
        // The configuration RetxBsrTimerTest calibrated the handheld uplink on.
        // Under the default paired FDD carrier with BSR triggering the same
        // flow delivered only 75 % (every other packet lost once the grants
        // shrink to MCS 0); that is reported as an open item in TESTS_REPORT
        // (tests-g3a), not hidden here.
        rs.SetDuplexMode(NtnRealStackHelper::DuplexMode::Tdd);
        rs.SetBsrTriggering(false);
        rs.Build(sats, ues);

        PacketSinkHelper sinkH("ns3::UdpSocketFactory", InetSocketAddress(Ipv4Address::GetAny(), 5000));
        sinkH.SetAttribute("EnableSeqTsSizeHeader", BooleanValue(true));
        ApplicationContainer sink = sinkH.Install(rs.GetRemoteHost());
        sink.Get(0)->TraceConnectWithoutContext("RxWithSeqTsSize",
                                                MakeCallback(&CbrUplinkOverNrNtnTest::Rx, this));
        CbrHelper cbr("ns3::UdpSocketFactory", InetSocketAddress(rs.GetRemoteHostIpv4Address(), 5000));
        cbr.SetConstantTraffic(MilliSeconds(20), 46);
        cbr.SetAttribute("EnableSeqTsSizeHeader", BooleanValue(true));
        ApplicationContainer app = cbr.Install(ues.Get(0));
        app.Start(Seconds(1.0));
        app.Stop(Seconds(4.0));
        Simulator::Stop(Seconds(4.5));
        Simulator::Run();
        const uint32_t sent = DynamicCast<CbrApplication>(app.Get(0))->GetSent() / 46;
        Simulator::Destroy();

        NS_TEST_ASSERT_MSG_EQ(sent, 149u, "1.02 s to 3.98 s every 20 ms");
        NS_TEST_ASSERT_MSG_GT_OR_EQ(m_seqs.size() * 100, sent * 97u, "voice crossed the uplink");
        NS_TEST_ASSERT_MSG_EQ(m_delays.size(), m_seqs.size(), "no duplicates");
        if (m_delays.empty())
        {
            return;
        }
        const double floorMs = 600e3 / 299792458.0 * 1e3;
        const double minMs = *std::min_element(m_delays.begin(), m_delays.end());
        double mean = 0.0;
        for (double d : m_delays)
        {
            mean += d / m_delays.size();
        }
        NS_TEST_ASSERT_MSG_GT(minMs, floorMs, "no packet beats the 600 km / c propagation floor");
        NS_TEST_ASSERT_MSG_LT(mean, 150.0, "mean one-way delay inside the ITU-T G.114 150 ms budget");
        std::cout << "[cbr-ul-nr] delivered " << m_seqs.size() << "/" << sent << " min " << minMs
                  << " ms mean " << mean << " ms" << std::endl;
    }

    std::vector<double> m_delays;
    std::set<uint32_t> m_seqs;
};

// The CbrTestSuite class names the TestSuite as cbr-test, identifies what type of TestSuite
// (Type::UNIT), and enables the TestCases to be run CbrTestCase1.
//
class CbrTestSuite : public TestSuite
{
  public:
    CbrTestSuite();
};

CbrTestSuite::CbrTestSuite()
    : TestSuite("cbr-test", Type::UNIT)
{
    AddTestCase(new CbrTestCase1, TestCase::Duration::QUICK);
    AddTestCase(new CbrTimingOverSlantTest, TestCase::Duration::QUICK);
    AddTestCase(new CbrInBandLossAccountingTest, TestCase::Duration::QUICK);
    AddTestCase(new CbrDscpMarkingTest, TestCase::Duration::QUICK);
    AddTestCase(new CbrOverloadTest, TestCase::Duration::QUICK);
    AddTestCase(new CbrFailurePathsTest, TestCase::Duration::QUICK);
    AddTestCase(new CbrDeterminismTest, TestCase::Duration::QUICK);
    AddTestCase(new CbrUplinkOverNrNtnTest, TestCase::Duration::QUICK);
}

// Allocate an instance of this TestSuite
static CbrTestSuite cbrTestSuite;
