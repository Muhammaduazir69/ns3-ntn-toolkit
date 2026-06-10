// SPDX-License-Identifier: GPL-2.0-only
//
// e2e tests for NtnOranAiFlowMonitor (AI_NATIVE_ORAN_NTN plan WS2). All on a
// real Simulator::Run() data plane:
//   1. ground truth: FlowMonitor bytes == NtnOranSink received bytes EXACTLY,
//      and the KPM throughput series integrates back to the same volume;
//   2. known-loss: a REAL RateErrorModel switched on mid-run must appear in
//      DRB.PacketLossRateDl and raise an EWMA anomaly event;
//   3. multi-flow: 3 slices x 2 flows classify into 6 distinct ORAN flows;
//   4. exporters: XML / CSV / Influx line protocol round-trip with content.

#include "ns3/address.h"
#include "ns3/data-rate.h"
#include "ns3/double.h"
#include "ns3/error-model.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/ntn-oran-ai-flow-monitor.h"
#include "ns3/ntn-oran-application.h"
#include "ns3/ntn-oran-sink.h"
#include "ns3/packet.h"
#include "ns3/point-to-point-helper.h"
#include "ns3/pointer.h"
#include "ns3/rng-seed-manager.h"
#include "ns3/simulator.h"
#include "ns3/string.h"
#include "ns3/test.h"
#include "ns3/uinteger.h"

#include <fstream>
#include <sstream>

using namespace ns3;

namespace
{

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
        ipv4.SetBase("10.2.1.0", "255.255.255.0");
        ifaces = ipv4.Assign(devs);
    }
};

/// Install one ORAN flow node0 -> node1 and register it with the monitor.
std::pair<Ptr<NtnOranApplication>, Ptr<NtnOranSink>>
MakeFlow(P2pRig& rig,
         Ptr<NtnOranAiFlowMonitor> mon,
         uint16_t port,
         uint8_t fiveQi,
         uint8_t sst,
         uint16_t srcId,
         double stopS)
{
    Ptr<NtnOranSink> sink = CreateObject<NtnOranSink>();
    sink->SetAttribute("Local", AddressValue(InetSocketAddress(Ipv4Address::GetAny(), port)));
    rig.nodes.Get(1)->AddApplication(sink);
    sink->SetStartTime(Seconds(0.0));

    Ptr<NtnOranApplication> app = CreateObject<NtnOranApplication>();
    app->SetRemote(InetSocketAddress(rig.ifaces.GetAddress(1), port));
    app->SetProfile(NtnOranApplication::URLLC_PERIODIC);
    app->SetFlowIdentity(fiveQi, sst, 0x000001, srcId, 1);
    rig.nodes.Get(0)->AddApplication(app);
    app->SetStartTime(Seconds(1.0));
    app->SetStopTime(Seconds(stopS));

    mon->AddSource(app);
    mon->AddSink(sink);
    return {app, sink};
}

} // namespace

class OranMonitorGroundTruthTest : public TestCase
{
  public:
    OranMonitorGroundTruthTest()
        : TestCase("FlowMonitor + KPM series match the sink byte-for-byte")
    {
    }

  private:
    void DoRun() override
    {
        P2pRig rig("10ms", "100Mbps");
        Ptr<NtnOranAiFlowMonitor> mon = CreateObject<NtnOranAiFlowMonitor>();
        // Stop the app between ticks so the final 1 s KPM window (tick at
        // t=21) captures every delivered packet before Simulator::Stop.
        auto [app, sink] = MakeFlow(rig, mon, 5000, 82, 2, 11, 20.5);
        mon->Start();

        Simulator::Stop(Seconds(22.0));
        Simulator::Run();

        // One ORAN flow classified.
        const auto& series = mon->GetKpmSeries();
        NS_TEST_ASSERT_MSG_EQ(series.size(), 1, "one ORAN flow");
        const FlowId id = series.begin()->first;
        OranFlowKey key;
        NS_TEST_ASSERT_MSG_EQ(mon->GetClassifier()->FindFlow(id, key), true, "key known");
        NS_TEST_ASSERT_MSG_EQ(+key.fiveQi, 82, "5QI key");
        NS_TEST_ASSERT_MSG_EQ(key.srcId, 11, "srcId key");

        // FlowMonitor (probe pipeline) bytes == sink ground truth EXACTLY.
        const auto fmStats = mon->GetFlowMonitor()->GetFlowStats();
        auto it = fmStats.find(id);
        NS_TEST_ASSERT_MSG_EQ((it != fmStats.end()), true, "FlowMonitor saw the flow");
        NS_TEST_ASSERT_MSG_EQ(it->second.rxBytes, sink->GetTotalRx(),
                              "FlowMonitor rxBytes == sink bytes");
        NS_TEST_ASSERT_MSG_EQ(it->second.rxPackets, sink->GetRxPackets(),
                              "FlowMonitor rxPackets == sink packets");
        // FlowMonitor's own delay agrees with the in-band measurement (~10 ms).
        const double fmDelayMs =
            it->second.rxPackets
                ? it->second.delaySum.GetSeconds() / it->second.rxPackets * 1e3
                : 0.0;
        NS_TEST_ASSERT_MSG_EQ_TOL(fmDelayMs, sink->GetMeanDelayMs(), 0.1,
                                  "probe-pipeline delay == in-band delay");

        // KPM series integrates back to the sink volume.
        double kbitSum = 0;
        for (const auto& s : series.begin()->second)
        {
            kbitSum += s.metrics.at("DRB.PdcpSduVolumeDl");
        }
        NS_TEST_ASSERT_MSG_EQ_TOL(kbitSum, sink->GetTotalRx() * 8.0 / 1e3, 1.0,
                                  "sum(DRB.PdcpSduVolumeDl) == sink volume");
        Simulator::Destroy();
    }
};

class OranMonitorKnownLossTest : public TestCase
{
  public:
    OranMonitorKnownLossTest()
        : TestCase("mid-run REAL error burst shows in KPM loss + anomaly event")
    {
    }

  private:
    void DoRun() override
    {
        RngSeedManager::SetSeed(11);
        P2pRig rig("10ms", "100Mbps");
        Ptr<RateErrorModel> em = CreateObject<RateErrorModel>();
        em->SetUnit(RateErrorModel::ERROR_UNIT_PACKET);
        em->SetRate(0.0);
        rig.devs.Get(1)->SetAttribute("ReceiveErrorModel", PointerValue(em));

        Ptr<NtnOranAiFlowMonitor> mon = CreateObject<NtnOranAiFlowMonitor>();
        auto [app, sink] = MakeFlow(rig, mon, 5001, 82, 2, 21, 40.0);
        mon->Start();

        // Loss burst starts at t=25 s: the device REALLY drops 40% after that.
        Simulator::Schedule(Seconds(25.0), [em] { em->SetRate(0.4); });

        Simulator::Stop(Seconds(41.0));
        Simulator::Run();

        const auto& series = mon->GetKpmSeries().begin()->second;
        double lossBefore = 0, lossAfter = 0;
        uint32_t nBefore = 0, nAfter = 0;
        for (const auto& s : series)
        {
            if (s.time.GetSeconds() <= 24.0)
            {
                lossBefore += s.metrics.at("DRB.PacketLossRateDl");
                ++nBefore;
            }
            else if (s.time.GetSeconds() >= 27.0)
            {
                lossAfter += s.metrics.at("DRB.PacketLossRateDl");
                ++nAfter;
            }
        }
        NS_TEST_ASSERT_MSG_LT(lossBefore / nBefore, 0.01, "clean before the burst");
        NS_TEST_ASSERT_MSG_EQ_TOL(lossAfter / nAfter, 0.4, 0.08,
                                  "measured KPM loss tracks the real 40% drop rate");

        // The EWMA detector must have flagged the loss jump.
        bool lossAnomaly = false;
        for (const auto& ev : mon->GetAnomalies())
        {
            if (ev.metric == "DRB.PacketLossRateDl" && ev.time.GetSeconds() >= 25.0)
            {
                lossAnomaly = true;
            }
        }
        NS_TEST_ASSERT_MSG_EQ(lossAnomaly, true, "anomaly event raised on the burst");
        Simulator::Destroy();
    }
};

class OranMonitorMultiFlowTest : public TestCase
{
  public:
    OranMonitorMultiFlowTest()
        : TestCase("3 slices x 2 flows classify into 6 distinct ORAN flows")
    {
    }

  private:
    void DoRun() override
    {
        P2pRig rig("5ms", "100Mbps");
        Ptr<NtnOranAiFlowMonitor> mon = CreateObject<NtnOranAiFlowMonitor>();
        uint16_t port = 6000;
        uint16_t srcId = 100;
        for (uint8_t sst = 1; sst <= 3; ++sst)
        {
            for (int f = 0; f < 2; ++f)
            {
                MakeFlow(rig, mon, port++, /*5qi*/ uint8_t(80 + sst), sst, srcId++, 12.0);
            }
        }
        mon->Start();
        Simulator::Stop(Seconds(13.0));
        Simulator::Run();

        NS_TEST_ASSERT_MSG_EQ(mon->GetKpmSeries().size(), 6, "6 distinct flows");
        // Every flow has its own series with non-zero throughput.
        for (const auto& [id, series] : mon->GetKpmSeries())
        {
            double thp = 0;
            for (const auto& s : series)
            {
                thp += s.metrics.at("DRB.UEThpDl");
            }
            NS_TEST_ASSERT_MSG_GT(thp, 0.0, "flow carried measured traffic");
            // AI features computable per flow.
            auto feat = mon->GetFeatures(id);
            NS_TEST_ASSERT_MSG_GT(feat.windowLen, 0, "feature window populated");
            NS_TEST_ASSERT_MSG_GT(feat.thpMeanMbps, 0.0, "feature thp from data");
        }
        Simulator::Destroy();
    }
};

class OranMonitorExportTest : public TestCase
{
  public:
    OranMonitorExportTest()
        : TestCase("XML / CSV / Influx exporters round-trip with content")
    {
    }

  private:
    void DoRun() override
    {
        P2pRig rig("5ms", "100Mbps");
        Ptr<NtnOranAiFlowMonitor> mon = CreateObject<NtnOranAiFlowMonitor>();
        MakeFlow(rig, mon, 7000, 9, 1, 51, 11.0);
        mon->Start();
        Simulator::Stop(Seconds(12.0));
        Simulator::Run();

        const std::string base = "ntn-oran-monitor-test";
        mon->SerializeToXmlFile(base + ".xml");
        mon->WriteCsv(base + ".csv");
        mon->WriteInfluxLp(base + ".lp");

        auto slurp = [](const std::string& p) {
            std::ifstream f(p);
            std::stringstream ss;
            ss << f.rdbuf();
            return ss.str();
        };
        const std::string xml = slurp(base + ".xml");
        NS_TEST_ASSERT_MSG_EQ((xml.find("<NtnOranFlowClassifier>") != std::string::npos),
                              true, "XML has classifier records");
        NS_TEST_ASSERT_MSG_EQ((xml.find("<FlowMonitor>") != std::string::npos), true,
                              "XML has FlowMonitor stats");
        const std::string csv = slurp(base + ".csv");
        NS_TEST_ASSERT_MSG_EQ((csv.find("DRB.UEThpDl") != std::string::npos), true,
                              "CSV has KPM header");
        NS_TEST_ASSERT_MSG_GT(std::count(csv.begin(), csv.end(), '\n'), 5,
                              "CSV has series rows");
        const std::string lp = slurp(base + ".lp");
        NS_TEST_ASSERT_MSG_EQ((lp.find("ntn_oran_kpm,flow_id=") != std::string::npos),
                              true, "Influx lp has measurement rows");
        Simulator::Destroy();
    }
};

class NtnOranAiFlowMonitorTestSuite : public TestSuite
{
  public:
    NtnOranAiFlowMonitorTestSuite()
        : TestSuite("ntn-oran-ai-flow-monitor", Type::UNIT)
    {
        AddTestCase(new OranMonitorGroundTruthTest, Duration::QUICK);
        AddTestCase(new OranMonitorKnownLossTest, Duration::QUICK);
        AddTestCase(new OranMonitorMultiFlowTest, Duration::QUICK);
        AddTestCase(new OranMonitorExportTest, Duration::QUICK);
    }
};

static NtnOranAiFlowMonitorTestSuite g_ntnOranAiFlowMonitorTestSuite;
