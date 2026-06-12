// SPDX-License-Identifier: GPL-2.0-only
//
// e2e tests for the AI-Native ORAN-NTN application suite (WS1). Every test
// runs a real Simulator::Run() data plane (NetDevice + IP + apps); nothing
// is asserted on statically computed values:
//   1. header wire round-trip through a real Packet,
//   2. one-way delay / jitter / loss measured by NtnOranSink over a P2P link
//      with KNOWN delay must match that delay (and zero loss/jitter for CBR),
//   3. a real RateErrorModel loss window must show up in the seq-gap loss,
//   4. NtnCommandAndControlApp telemetry must round-trip the REAL mobility
//      state (position/velocity) and a draining battery through the network,
//   5. a packet whose in-band header carries a foreign wire-format version is
//      counted by GetVersionErrors() and excluded from every flow KPI.

#include "ns3/boolean.h"
#include "ns3/constant-velocity-mobility-model.h"
#include "ns3/double.h"
#include "ns3/enum.h"
#include "ns3/error-model.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/log.h"
#include "ns3/ntn-command-and-control-app.h"
#include "ns3/ntn-oran-application.h"
#include "ns3/ntn-oran-payload-header.h"
#include "ns3/ntn-oran-sink.h"
#include "ns3/packet.h"
#include "ns3/point-to-point-helper.h"
#include "ns3/pointer.h"
#include "ns3/rng-seed-manager.h"
#include "ns3/simulator.h"
#include "ns3/socket.h"
#include "ns3/string.h"
#include "ns3/test.h"
#include "ns3/udp-socket-factory.h"
#include "ns3/uinteger.h"

#include <vector>

using namespace ns3;

namespace
{

/// Build a 2-node P2P link with internet; returns {client node, sink node}.
struct P2pRig
{
    NodeContainer nodes;
    Ipv4InterfaceContainer ifaces;
    NetDeviceContainer devs;

    P2pRig(std::string delay, std::string rate)
    {
        nodes.Create(2);
        PointToPointHelper p2p;
        p2p.SetDeviceAttribute("DataRate", StringValue(rate));
        p2p.SetChannelAttribute("Delay", StringValue(delay));
        devs = p2p.Install(nodes);
        InternetStackHelper internet;
        internet.Install(nodes);
        Ipv4AddressHelper ipv4;
        ipv4.SetBase("10.1.1.0", "255.255.255.0");
        ifaces = ipv4.Assign(devs);
    }
};

} // namespace

class NtnOranHeaderRoundTripTest : public TestCase
{
  public:
    NtnOranHeaderRoundTripTest()
        : TestCase("NtnOranPayloadHeader survives a real Packet round-trip")
    {
    }

  private:
    void DoRun() override
    {
        NtnOranPayloadHeader tx;
        tx.SetPayloadType(NtnOranPayloadHeader::URLLC_CMD);
        tx.SetSeq(0xDEADBEEF);
        tx.SetTxTimestampNs(123456789012345ULL);
        tx.SetFiveQi(82);
        tx.SetSnssai(2, 0xABCDEF);
        tx.SetQfi(61);
        tx.SetSrcId(4321);
        tx.SetDstId(8765);

        Ptr<Packet> p = Create<Packet>(100);
        p->AddHeader(tx);
        NS_TEST_ASSERT_MSG_EQ(p->GetSize(),
                              100 + NtnOranPayloadHeader::SERIALIZED_SIZE,
                              "serialized size");

        NtnOranPayloadHeader rx;
        p->RemoveHeader(rx);
        NS_TEST_ASSERT_MSG_EQ(rx.GetPayloadType(), NtnOranPayloadHeader::URLLC_CMD, "type");
        NS_TEST_ASSERT_MSG_EQ(rx.GetSeq(), 0xDEADBEEF, "seq");
        NS_TEST_ASSERT_MSG_EQ(rx.GetTxTimestampNs(), 123456789012345ULL, "timestamp");
        NS_TEST_ASSERT_MSG_EQ(+rx.GetFiveQi(), 82, "5qi");
        NS_TEST_ASSERT_MSG_EQ(+rx.GetSst(), 2, "sst");
        NS_TEST_ASSERT_MSG_EQ(rx.GetSd(), 0xABCDEFu, "sd");
        NS_TEST_ASSERT_MSG_EQ(+rx.GetQfi(), 61, "qfi");
        NS_TEST_ASSERT_MSG_EQ(rx.GetSrcId(), 4321, "srcId");
        NS_TEST_ASSERT_MSG_EQ(rx.GetDstId(), 8765, "dstId");
    }
};

class NtnOranKnownDelayTest : public TestCase
{
  public:
    NtnOranKnownDelayTest()
        : TestCase("NtnOranSink measures the real 25 ms link delay in-band")
    {
    }

  private:
    void DoRun() override
    {
        P2pRig rig("25ms", "100Mbps");
        const uint16_t port = 4000;

        Ptr<NtnOranSink> sink = CreateObject<NtnOranSink>();
        sink->SetAttribute("Local", AddressValue(InetSocketAddress(Ipv4Address::GetAny(), port)));
        rig.nodes.Get(1)->AddApplication(sink);
        sink->SetStartTime(Seconds(0.0));

        Ptr<NtnOranApplication> app = CreateObject<NtnOranApplication>();
        app->SetRemote(InetSocketAddress(rig.ifaces.GetAddress(1), port));
        app->SetProfile(NtnOranApplication::URLLC_PERIODIC);
        app->SetFlowIdentity(82, 1, 0x000001, 7, 1);
        rig.nodes.Get(0)->AddApplication(app);
        app->SetStartTime(Seconds(1.0));
        app->SetStopTime(Seconds(11.0));

        Simulator::Stop(Seconds(12.0));
        Simulator::Run();

        NS_TEST_ASSERT_MSG_GT(sink->GetRxPackets(), 900, "10 s of 10 ms URLLC packets");
        // 25 ms propagation + serialization (≈0.02 ms at 100 Mb/s for 256 B).
        NS_TEST_ASSERT_MSG_EQ_TOL(sink->GetMeanDelayMs(), 25.0, 0.5,
                                  "in-band one-way delay = real link delay");
        NS_TEST_ASSERT_MSG_LT(sink->GetMeanJitterMs(), 0.1,
                              "deterministic periodic flow has ~0 jitter");
        NS_TEST_ASSERT_MSG_EQ_TOL(sink->GetLossRatio(), 0.0, 1e-9,
                                  "clean link: zero seq-gap loss");
        // Flow identity survived the wire.
        const auto& flows = sink->GetFlowStats();
        NS_TEST_ASSERT_MSG_EQ(flows.size(), 1, "single QoS flow");
        const auto& fs = flows.begin()->second;
        NS_TEST_ASSERT_MSG_EQ(+fs.fiveQi, 82, "5QI in flow key");
        NS_TEST_ASSERT_MSG_EQ(fs.srcId, 7, "srcId in flow key");

        Simulator::Destroy();
    }
};

class NtnOranRealLossTest : public TestCase
{
  public:
    NtnOranRealLossTest()
        : TestCase("seq-gap loss tracks a REAL RateErrorModel on the device")
    {
    }

  private:
    void DoRun() override
    {
        RngSeedManager::SetSeed(7);
        P2pRig rig("10ms", "100Mbps");
        const uint16_t port = 4001;

        // Real per-packet error model on the receiving device.
        Ptr<RateErrorModel> em = CreateObject<RateErrorModel>();
        em->SetUnit(RateErrorModel::ERROR_UNIT_PACKET);
        em->SetRate(0.2);
        rig.devs.Get(1)->SetAttribute("ReceiveErrorModel", PointerValue(em));

        Ptr<NtnOranSink> sink = CreateObject<NtnOranSink>();
        sink->SetAttribute("Local", AddressValue(InetSocketAddress(Ipv4Address::GetAny(), port)));
        rig.nodes.Get(1)->AddApplication(sink);
        sink->SetStartTime(Seconds(0.0));

        Ptr<NtnOranApplication> app = CreateObject<NtnOranApplication>();
        app->SetRemote(InetSocketAddress(rig.ifaces.GetAddress(1), port));
        app->SetProfile(NtnOranApplication::URLLC_PERIODIC);
        app->SetFlowIdentity(82, 1, 0x000001, 9, 1);
        rig.nodes.Get(0)->AddApplication(app);
        app->SetStartTime(Seconds(1.0));
        app->SetStopTime(Seconds(21.0));

        Simulator::Stop(Seconds(22.0));
        Simulator::Run();

        // ~2000 packets at 20% drop: measured seq-gap loss must track the
        // error model's rate (binomial 3-sigma ≈ 0.027).
        NS_TEST_ASSERT_MSG_EQ_TOL(sink->GetLossRatio(), 0.2, 0.04,
                                  "measured loss == real error-model rate");
        Simulator::Destroy();
    }
};

class NtnCncTelemetryTest : public TestCase
{
  public:
    NtnCncTelemetryTest()
        : TestCase("C&C telemetry round-trips real mobility + battery state")
    {
    }

  private:
    void DoRun() override
    {
        P2pRig rig("5ms", "10Mbps");
        const uint16_t port = 4002;

        // Platform node moves with a REAL velocity; telemetry must report it.
        Ptr<ConstantVelocityMobilityModel> mob = CreateObject<ConstantVelocityMobilityModel>();
        mob->SetPosition(Vector(1000.0, 2000.0, 500000.0));
        mob->SetVelocity(Vector(7000.0, 1000.0, -50.0));
        rig.nodes.Get(0)->AggregateObject(mob);

        Ptr<NtnOranSink> smo = CreateObject<NtnOranSink>();
        smo->SetAttribute("Local", AddressValue(InetSocketAddress(Ipv4Address::GetAny(), port)));
        rig.nodes.Get(1)->AddApplication(smo);
        smo->SetStartTime(Seconds(0.0));

        Ptr<NtnCommandAndControlApp> cnc = CreateObject<NtnCommandAndControlApp>();
        cnc->SetRemote(InetSocketAddress(rig.ifaces.GetAddress(1), port));
        cnc->SetAttribute("Period", TimeValue(MilliSeconds(500)));
        cnc->SetAttribute("SrcId", UintegerValue(42));
        // 100 Wh at 3600 W drains 1%/3.6 s — visible inside a 20 s run.
        cnc->SetAttribute("BatteryCapacityWh", DoubleValue(100.0));
        cnc->SetAttribute("PowerDrawW", DoubleValue(3600.0));
        rig.nodes.Get(0)->AddApplication(cnc);
        cnc->SetStartTime(Seconds(0.0));
        cnc->SetStopTime(Seconds(20.0));

        Simulator::Stop(Seconds(20.5));
        Simulator::Run();

        NtnCncTelemetry t;
        NS_TEST_ASSERT_MSG_EQ(smo->GetLatestTelemetry(42, t), true, "telemetry arrived");
        // Last report at ~19.5 s: position = start + v*t from the REAL model.
        NS_TEST_ASSERT_MSG_EQ_TOL(t.velX, 7000.0, 1e-6, "velocity X from mobility model");
        NS_TEST_ASSERT_MSG_EQ_TOL(t.posX, 1000.0 + 7000.0 * 19.5, 7000.0 * 0.51,
                                  "position X tracks the real trajectory");
        // Yaw from the velocity frame: atan2(1000, 7000) = 8.13 deg.
        NS_TEST_ASSERT_MSG_EQ_TOL(t.yawDeg, 8.13, 0.1, "yaw from velocity frame");
        // Battery after 19.5 s at 1 Wh/s on 100 Wh: ~80.5% remain.
        NS_TEST_ASSERT_MSG_EQ_TOL(t.batteryFraction, 1.0 - 19.5 / 100.0, 0.011,
                                  "battery drained by the energy model");
        Simulator::Destroy();
    }
};

class NtnOranVersionErrorTest : public TestCase
{
  public:
    NtnOranVersionErrorTest()
        : TestCase("sink discards a foreign payload-header version and counts it")
    {
    }

  private:
    /// Build an ORAN payload packet; optionally corrupt the version byte
    /// (byte 0 on the wire) to fabricate an incompatible wire format.
    static Ptr<Packet> MakePacket(uint32_t seq, bool corruptVersion)
    {
        NtnOranPayloadHeader hdr;
        hdr.SetPayloadType(NtnOranPayloadHeader::URLLC_CMD);
        hdr.SetSeq(seq);
        hdr.SetTxTimestampNs(Simulator::Now().GetNanoSeconds());
        hdr.SetFiveQi(82);
        hdr.SetSnssai(1, 0x000001);
        hdr.SetSrcId(7);
        hdr.SetDstId(1);
        Ptr<Packet> p = Create<Packet>(100);
        p->AddHeader(hdr);
        if (!corruptVersion)
        {
            return p;
        }
        std::vector<uint8_t> buf(p->GetSize());
        p->CopyData(buf.data(), buf.size());
        buf[0] = NtnOranPayloadHeader::NTN_ORAN_PAYLOAD_VERSION + 1;
        return Create<Packet>(buf.data(), buf.size());
    }

    void DoRun() override
    {
        P2pRig rig("10ms", "100Mbps");
        const uint16_t port = 4003;
        constexpr uint32_t kPktSize = 100 + NtnOranPayloadHeader::SERIALIZED_SIZE;

        Ptr<NtnOranSink> sink = CreateObject<NtnOranSink>();
        sink->SetAttribute("Local", AddressValue(InetSocketAddress(Ipv4Address::GetAny(), port)));
        rig.nodes.Get(1)->AddApplication(sink);
        sink->SetStartTime(Seconds(0.0));

        // Real UDP sends over the P2P link: one valid packet, then one whose
        // version byte was corrupted in the serialized bytes.
        Ptr<Socket> sock = Socket::CreateSocket(rig.nodes.Get(0), UdpSocketFactory::GetTypeId());
        sock->Connect(InetSocketAddress(rig.ifaces.GetAddress(1), port));
        Simulator::Schedule(Seconds(1.0), [sock] { sock->Send(MakePacket(0, false)); });
        Simulator::Schedule(Seconds(2.0), [sock] { sock->Send(MakePacket(1, true)); });

        Simulator::Stop(Seconds(3.0));
        Simulator::Run();

        // Both packets physically arrived (raw counters see them) ...
        NS_TEST_ASSERT_MSG_EQ(sink->GetRxPackets(), 2, "both packets delivered");
        NS_TEST_ASSERT_MSG_EQ(sink->GetTotalRx(), 2 * kPktSize, "raw byte counter sees both");
        // ... but only the valid-version packet is accounted as a flow.
        NS_TEST_ASSERT_MSG_EQ(sink->GetVersionErrors(), 1, "corrupted version counted once");
        const auto& flows = sink->GetFlowStats();
        NS_TEST_ASSERT_MSG_EQ(flows.size(), 1, "only the valid-version flow exists");
        const auto& fs = flows.begin()->second;
        NS_TEST_ASSERT_MSG_EQ(fs.rxPackets, 1, "one valid packet in the flow KPIs");
        NS_TEST_ASSERT_MSG_EQ(fs.rxBytes, kPktSize, "flow bytes == valid packet bytes only");
        NS_TEST_ASSERT_MSG_EQ(+fs.fiveQi, 82, "flow identity from the valid packet");

        Simulator::Destroy();
    }
};

class NtnOranApplicationTestSuite : public TestSuite
{
  public:
    NtnOranApplicationTestSuite()
        : TestSuite("ntn-oran-application", Type::UNIT)
    {
        AddTestCase(new NtnOranHeaderRoundTripTest, Duration::QUICK);
        AddTestCase(new NtnOranKnownDelayTest, Duration::QUICK);
        AddTestCase(new NtnOranRealLossTest, Duration::QUICK);
        AddTestCase(new NtnCncTelemetryTest, Duration::QUICK);
        AddTestCase(new NtnOranVersionErrorTest, Duration::QUICK);
    }
};

static NtnOranApplicationTestSuite g_ntnOranApplicationTestSuite;
