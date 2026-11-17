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
        // DRB.PdcpSduVolumeDl is in Mbit (TS 28.552 5.1.2.1.1.1).
        double mbitSum = 0;
        for (const auto& s : series.begin()->second)
        {
            mbitSum += s.metrics.at("DRB.PdcpSduVolumeDl");
        }
        NS_TEST_ASSERT_MSG_EQ_TOL(mbitSum, sink->GetTotalRx() * 8.0 / 1e6, 1e-3,
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
        // DRB.PacketLossRateDl is now exported in its TS 28.552 §5.1.3.1 unit:
        // an integer in units of 1e-6 (the 0..1 fraction scaled by 1e6). This
        // test asserts the KPM export directly, so it asserts the spec unit:
        // "clean" is < 0.01 * 1e6 = 1e4, and the 40% burst is 0.4 * 1e6 = 4e5.
        NS_TEST_ASSERT_MSG_LT(lossBefore / nBefore, 0.01e6, "clean before the burst");
        NS_TEST_ASSERT_MSG_EQ_TOL(lossAfter / nAfter, 0.4e6, 0.08e6,
                                  "measured KPM loss tracks the real 40% drop rate (per-1e6 units)");

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

/// Wave 7A: an uplink flow is reported under the TS 28.552 UL names
/// (DRB.UEThpUl, DRB.RlcDelayUl, DRB.PacketLossRateUl) with no DL metric, a
/// flow registered without a direction keeps the DL names (failure path of the
/// direction logic), and the first period's throughput is volume over the time
/// the flow was transferring (TS 28.552 5.1.1.3), so it matches the steady state
/// instead of being diluted by the idle part of the period.
class OranMonitorUplinkNamesTest : public TestCase
{
  public:
    OranMonitorUplinkNamesTest()
        : TestCase("Wave 7A: UL flows use the UL KPM names; first-period throughput")
    {
    }

  private:
    void DoRun() override
    {
        P2pRig rig("10ms", "100Mbps");
        Ptr<NtnOranAiFlowMonitor> mon = CreateObject<NtnOranAiFlowMonitor>();
        // Uplink flow: explicit direction.
        Ptr<NtnOranSink> ulSink = CreateObject<NtnOranSink>();
        ulSink->SetAttribute("Local", AddressValue(InetSocketAddress(Ipv4Address::GetAny(), 7100)));
        rig.nodes.Get(1)->AddApplication(ulSink);
        ulSink->SetStartTime(Seconds(0.0));
        Ptr<NtnOranApplication> ulApp = CreateObject<NtnOranApplication>();
        ulApp->SetRemote(InetSocketAddress(rig.ifaces.GetAddress(1), 7100));
        ulApp->SetProfile(NtnOranApplication::URLLC_PERIODIC);
        ulApp->SetFlowIdentity(82, 2, 0x000001, 61, 1);
        rig.nodes.Get(0)->AddApplication(ulApp);
        ulApp->SetStartTime(Seconds(1.5)); // mid-period start
        ulApp->SetStopTime(Seconds(8.0));
        mon->AddSource(ulApp);
        mon->AddSink(ulSink, 0, NtnOranAiFlowMonitor::Direction::Uplink);
        // Downlink-named flow: no direction given, no PHY source.
        MakeFlow(rig, mon, 7200, 9, 1, 62, 8.0);
        mon->Start();
        Simulator::Stop(Seconds(9.0));
        Simulator::Run();

        uint32_t ulSeries = 0;
        uint32_t dlSeries = 0;
        for (const auto& [id, series] : mon->GetKpmSeries())
        {
            OranFlowKey key;
            mon->GetClassifier()->FindFlow(id, key);
            bool hasUl = false;
            bool hasDl = false;
            for (const auto& smp : series)
            {
                hasUl = hasUl || smp.metrics.count("DRB.UEThpUl") || smp.metrics.count("DRB.RlcDelayUl") ||
                        smp.metrics.count("DRB.PacketLossRateUl");
                hasDl = hasDl || smp.metrics.count("DRB.UEThpDl") ||
                        smp.metrics.count("DRB.RlcSduDelayDl") ||
                        smp.metrics.count("DRB.PacketLossRateDl");
            }
            if (key.srcId == 61)
            {
                ++ulSeries;
                NS_TEST_ASSERT_MSG_EQ(hasUl, true, "uplink flow carries the UL names");
                NS_TEST_ASSERT_MSG_EQ(hasDl, false, "uplink flow carries no DL name");
                // First non-empty period (data from 1.51 s, tick at 2 s) against
                // the steady state of the next one.
                double first = -1.0;
                double steady = -1.0;
                for (const auto& smp : series)
                {
                    const double v = smp.metrics.at("DRB.UEThpUl");
                    if (v > 0.0 && first < 0.0)
                    {
                        first = v;
                    }
                    else if (v > 0.0 && first > 0.0)
                    {
                        steady = v;
                        break;
                    }
                }
                NS_TEST_ASSERT_MSG_GT(first, 0.0, "first period measured");
                NS_TEST_ASSERT_MSG_EQ_TOL(first / steady, 1.0, 0.05,
                                          "first-period throughput is over the active time");
                // 0.1 ms units: 10 ms link + serialization -> about 100.
                double d = 0.0;
                for (const auto& smp : series)
                {
                    d = std::max(d, smp.metrics.at("DRB.RlcDelayUl"));
                }
                NS_TEST_ASSERT_MSG_EQ_TOL(d, 100.0, 2.0, "DRB.RlcDelayUl in 0.1 ms units");
            }
            else
            {
                ++dlSeries;
                NS_TEST_ASSERT_MSG_EQ(hasDl, true, "undirected flow keeps the DL names");
                NS_TEST_ASSERT_MSG_EQ(hasUl, false, "undirected flow has no UL name");
            }
        }
        NS_TEST_ASSERT_MSG_EQ(ulSeries, 1u, "one uplink series");
        NS_TEST_ASSERT_MSG_EQ(dlSeries, 1u, "one downlink series");

        // The CSV carries the UL columns and the units sidecar.
        mon->WriteCsv("ntn-oran-monitor-ul-test.csv");
        std::ifstream units("ntn-oran-monitor-ul-test_units.csv");
        std::stringstream ss;
        ss << units.rdbuf();
        NS_TEST_ASSERT_MSG_EQ((ss.str().find("DRB.RlcSduDelayDl,0.1 ms") != std::string::npos),
                              true,
                              "units file documents the 0.1 ms delay unit");
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
        AddTestCase(new OranMonitorUplinkNamesTest, Duration::QUICK);
    }
};

static NtnOranAiFlowMonitorTestSuite g_ntnOranAiFlowMonitorTestSuite;
