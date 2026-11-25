// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only
//
// In-depth oran-ntn E2 validation (18 Nov 2026 test campaign).
//
// The E2 node runs inside Simulator::Run() with a feeder link between the
// satellite gNB and the RIC. Procedures checked against O-RAN WG3:
//   - E2AP RIC Subscription (clause 8.2.1): an unsupported RAN function is
//     refused; a request for an active id replaces it (modification); RIC
//     Subscription Delete (8.2.2) stops the reports;
//   - E2SM-KPM REPORT service, periodic style: one indication per UE per
//     reporting period, delivered one feeder-link delay after the measurement;
//   - E2SM-KPM event trigger: only measured (radio-valid) reports below the
//     threshold are reported;
//   - NTN store-and-forward during a feeder outage: buffered reports carry
//     their original timestamp, the on-board buffer drops the oldest on
//     overflow, and maxBufferAge discards stale reports.
// The cross-module case measures a real UDP flow with FlowMonitor and checks
// the KPM throughput the RIC receives is the one measured at report time.

#include "ns3/applications-module.h"
#include "ns3/flow-monitor-helper.h"
#include "ns3/internet-module.h"
#include "ns3/oran-ntn-e2-interface.h"
#include "ns3/ns3-ai-gym-interface.h"
#include "ns3/oran-ntn-gym-handover.h"
#include "ns3/oran-ntn-phy-kpm-extractor.h"
#include "ns3/point-to-point-module.h"
#include "ns3/simulator.h"
#include "ns3/string.h"
#include "ns3/test.h"

#include <map>
#include <vector>

using namespace ns3;

namespace
{

struct Rx
{
    std::vector<E2Indication> ind;
    std::vector<double> at;

    static void On(Rx* r, E2Indication i)
    {
        r->ind.push_back(i);
        r->at.push_back(Simulator::Now().GetSeconds());
    }
};

E2Subscription
KpmSub(uint32_t id, Time period, bool event = false, double thr = 0.0)
{
    E2Subscription s{};
    s.subscriptionId = id;
    s.ricRequestorId = 1;
    s.ranFunctionId = 2;
    s.reportingPeriod = period;
    s.eventTrigger = event;
    s.eventThreshold = thr;
    s.batchOnVisibility = true;
    s.maxBufferAge = Time(0);
    s.useIslRelay = false;
    return s;
}

E2KpmReport
Report(uint32_t ue, double sinr, bool valid = true)
{
    E2KpmReport r{};
    r.timestamp = Simulator::Now().GetSeconds();
    r.gnbId = 7;
    r.isNtn = true;
    r.ueId = ue;
    r.sinr_dB = sinr;
    r.radioValid = valid;
    return r;
}

Ptr<OranNtnE2Node>
MakeNode(Rx* rx, Time feeder)
{
    Ptr<OranNtnE2Node> n = CreateObject<OranNtnE2Node>();
    n->SetNodeId(7);
    n->SetIsNtn(true);
    n->SetFeederLinkDelay(feeder);
    n->RegisterRanFunction(2, "E2SM-KPM");
    n->RegisterRanFunction(3, "E2SM-RC");
    n->SetPeriodicReporting(true);
    n->SetIndicationCallback(MakeBoundCallback(&Rx::On, rx));
    return n;
}

} // namespace

// ---------------------------------------------------------------------------
/// Periodic KPM REPORT: 3 UEs, 100 ms period, 20 ms feeder link, 5 s. Exactly
/// one indication per UE per period, each delivered 20 ms after it was taken,
/// carrying the UE's latest measurement.
class OranE2PeriodicKpmTest : public TestCase
{
  public:
    OranE2PeriodicKpmTest()
        : TestCase("E2SM-KPM periodic report: one indication per UE per period, feeder-delayed")
    {
    }

  private:
    void DoRun() override
    {
        Rx rx;
        Ptr<OranNtnE2Node> n = MakeNode(&rx, MilliSeconds(20));
        for (uint32_t ue = 1; ue <= 3; ++ue)
        {
            for (int k = 0; k < 100; ++k)
            {
                Simulator::Schedule(MilliSeconds(50 * k + 5), [n, ue, k]() {
                    n->SubmitKpmMeasurement(Report(ue, 10.0 + ue + 0.01 * k));
                });
            }
        }
        // event-free subscription to the RC function: accepted, never reports KPM
        Simulator::Schedule(Seconds(0), [n]() {
            NS_ABORT_IF(!n->HandleSubscriptionRequest(KpmSub(1, MilliSeconds(100))));
            E2Subscription rc = KpmSub(2, MilliSeconds(100));
            rc.ranFunctionId = 3;
            NS_ABORT_IF(!n->HandleSubscriptionRequest(rc));
        });
        Simulator::Stop(Seconds(5.0));
        Simulator::Run();
        const uint64_t periodic = n->GetPeriodicIndicationCount();
        Simulator::Destroy();

        // the first period with a report is 0.1 s; 49 or 50 periods end before 5 s
        NS_TEST_ASSERT_MSG_GT_OR_EQ(rx.ind.size(), 3u * 49u, "one indication per UE per 100 ms");
        NS_TEST_ASSERT_MSG_LT_OR_EQ(rx.ind.size(), 3u * 50u, "and no more");
        NS_TEST_ASSERT_MSG_EQ(periodic, rx.ind.size(),
                              "every indication is a periodic one: measurements alone do not report");
        uint32_t badDelay = 0;
        uint32_t stale = 0;
        for (size_t i = 0; i < rx.ind.size(); ++i)
        {
            const auto& d = rx.ind[i];
            badDelay += std::abs(rx.at[i] - d.originalTimestamp.GetSeconds() - 0.020) > 1e-9;
            NS_TEST_ASSERT_MSG_EQ(d.subscriptionId, 1u, "only the KPM subscription reports");
            // latest report was taken 45 ms (or 95 ms) before the period tick
            const double age = d.originalTimestamp.GetSeconds() - d.kpmReport.timestamp;
            stale += age > 0.050 + 1e-9;
        }
        NS_TEST_ASSERT_MSG_EQ(badDelay, 0u, "delivered exactly one feeder delay after the report");
        NS_TEST_ASSERT_MSG_EQ(stale, 0u, "each indication carries the UE's latest measurement");
    }
};

// ---------------------------------------------------------------------------
/// RIC Subscription lifecycle: an unsupported RAN function is refused
/// (Subscription Failure); re-subscribing an active id with a 200 ms period
/// replaces the 100 ms one instead of adding to it; Subscription Delete stops
/// all further indications.
class OranE2SubscriptionLifecycleTest : public TestCase
{
  public:
    OranE2SubscriptionLifecycleTest()
        : TestCase("RIC Subscription refusal, modification and delete (E2AP 8.2.1 - 8.2.2)")
    {
    }

  private:
    void DoRun() override
    {
        Rx rx;
        Ptr<OranNtnE2Node> n = MakeNode(&rx, MilliSeconds(10));
        E2Subscription bad = KpmSub(9, MilliSeconds(100));
        bad.ranFunctionId = 42;
        bool badAccepted = true;
        Simulator::Schedule(Seconds(0), [&]() {
            badAccepted = n->HandleSubscriptionRequest(bad);
            n->SubmitKpmMeasurement(Report(1, 12.0));
            n->HandleSubscriptionRequest(KpmSub(1, MilliSeconds(100)));
        });
        // 100 ms period until 1.001 s, then 200 ms (reports at 1.201 ... 3.001); delete at 3.002 s
        Simulator::Schedule(MilliSeconds(1001), [n]() { n->HandleSubscriptionRequest(KpmSub(1, MilliSeconds(200))); });
        Simulator::Schedule(MilliSeconds(3002), [n]() { n->HandleSubscriptionDelete(1); });
        Simulator::Stop(Seconds(5.0));
        Simulator::Run();
        const size_t active = n->GetActiveSubscriptions().size();
        Simulator::Destroy();

        NS_TEST_ASSERT_MSG_EQ(badAccepted, false, "unsupported RAN function refused");
        uint32_t first = 0;
        uint32_t second = 0;
        uint32_t after = 0;
        for (const auto& i : rx.ind)
        {
            const double t = i.originalTimestamp.GetSeconds();
            first += t < 1.001;
            second += t > 1.001 && t < 3.0015;
            after += t > 3.0015;
        }
        NS_TEST_ASSERT_MSG_EQ(first, 10u, "10 reports at 100 ms in the first second");
        NS_TEST_ASSERT_MSG_EQ(second, 10u, "the modified 200 ms period alone: 10 in 2 s");
        NS_TEST_ASSERT_MSG_EQ(after, 0u, "delete stops reporting");
        NS_TEST_ASSERT_MSG_EQ(active, 0u, "no active subscription left");
    }
};

// ---------------------------------------------------------------------------
/// Event-triggered KPM: only radio-valid reports with SINR below the threshold
/// are reported; a report without a radio measurement never triggers.
class OranE2EventTriggerTest : public TestCase
{
  public:
    OranE2EventTriggerTest()
        : TestCase("E2SM-KPM event trigger fires only on measured SINR below threshold")
    {
    }

  private:
    void DoRun() override
    {
        Rx rx;
        Ptr<OranNtnE2Node> n = MakeNode(&rx, MilliSeconds(15));
        Simulator::Schedule(Seconds(0), [n]() { n->HandleSubscriptionRequest(KpmSub(5, Time(0), true, 3.0)); });
        // SINR sweeps 10 -> -10 dB and back; every 5th report has no radio value
        uint32_t expected = 0;
        for (int k = 1; k <= 200; ++k)
        {
            const double sinr = 10.0 - 20.0 * std::abs(std::sin(M_PI * k / 100.0));
            const bool valid = k % 5 != 0;
            expected += valid && sinr < 3.0;
            Simulator::Schedule(MilliSeconds(10 * k), [n, sinr, valid]() {
                n->SubmitKpmMeasurement(Report(1, sinr, valid));
            });
        }
        Simulator::Run();
        Simulator::Destroy();
        NS_TEST_ASSERT_MSG_EQ(rx.ind.size(), expected, "exactly the qualifying reports");
        for (const auto& i : rx.ind)
        {
            NS_TEST_ASSERT_MSG_LT(i.kpmReport.sinr_dB, 3.0, "below the threshold");
            NS_TEST_ASSERT_MSG_EQ(i.kpmReport.radioValid, true, "only measured reports trigger");
        }
        NS_TEST_ASSERT_MSG_GT(expected, 20u, "the sweep crosses the threshold");
    }
};

// ---------------------------------------------------------------------------
/// Store-and-forward over a 2 s feeder outage with a 100 ms periodic report:
/// (a) a large buffer delivers all 20 buffered reports at restoration with
/// their original timestamps; (b) a 15-report buffer keeps the newest 15 and
/// counts 5 drops; (c) maxBufferAge = 1 s keeps only the last second; (d) a
/// subscription that does not batch on visibility drops everything during the
/// outage. Two identical runs give identical delivery sequences.
class OranE2FeederOutageBufferTest : public TestCase
{
  public:
    OranE2FeederOutageBufferTest()
        : TestCase("Feeder outage store-and-forward: original timestamps, overflow, maxBufferAge")
    {
    }

  private:
    struct Out
    {
        std::vector<double> orig;
        std::vector<double> at;
        uint32_t dropped{0};
    };

    static void Drop(uint32_t* d, uint32_t, uint32_t)
    {
        ++*d;
    }

    Out RunOnce(uint32_t bufSize, Time maxAge, bool batch)
    {
        Rx rx;
        Out o;
        Ptr<OranNtnE2Node> n = MakeNode(&rx, MilliSeconds(30));
        n->SetOnBoardBufferSize(bufSize);
        n->TraceConnectWithoutContext("ReportDropped", MakeBoundCallback(&Drop, &o.dropped));
        E2Subscription s = KpmSub(1, MilliSeconds(100));
        s.maxBufferAge = maxAge;
        s.batchOnVisibility = batch;
        Simulator::Schedule(Seconds(0), [n, s]() {
            n->SubmitKpmMeasurement(Report(1, 9.0));
            n->HandleSubscriptionRequest(s);
        });
        Simulator::Schedule(MilliSeconds(1050), [n]() { n->SetFeederLinkAvailable(false); });
        Simulator::Schedule(MilliSeconds(3050), [n]() { n->SetFeederLinkAvailable(true); });
        Simulator::Stop(Seconds(4.0));
        Simulator::Run();
        Simulator::Destroy();
        for (size_t i = 0; i < rx.ind.size(); ++i)
        {
            o.orig.push_back(rx.ind[i].originalTimestamp.GetSeconds());
            o.at.push_back(rx.at[i]);
        }
        return o;
    }

    static uint32_t CountIn(const Out& o, double lo, double hi)
    {
        uint32_t c = 0;
        for (double t : o.orig)
        {
            c += t > lo && t < hi;
        }
        return c;
    }

    void DoRun() override
    {
        const Out big = RunOnce(1000, Time(0), true);
        NS_TEST_ASSERT_MSG_EQ(CountIn(big, 1.05, 3.05), 20u, "all 20 outage reports delivered");
        NS_TEST_ASSERT_MSG_EQ(big.dropped, 0u, "nothing dropped");
        uint32_t lateOk = 0;
        for (size_t i = 0; i < big.orig.size(); ++i)
        {
            if (big.orig[i] > 1.05 && big.orig[i] < 3.05)
            {
                lateOk += std::abs(big.at[i] - (3.05 + 0.030)) < 1e-9;
            }
        }
        NS_TEST_ASSERT_MSG_EQ(lateOk, 20u, "flushed one feeder delay after restoration, timestamps kept");

        const Out small = RunOnce(15, Time(0), true);
        NS_TEST_ASSERT_MSG_EQ(CountIn(small, 1.05, 3.05), 15u, "a 15-report buffer keeps 15");
        NS_TEST_ASSERT_MSG_EQ(small.dropped, 5u, "and drops 5");
        NS_TEST_ASSERT_MSG_EQ(CountIn(small, 1.05, 1.55), 0u, "the oldest are the ones dropped");

        const Out aged = RunOnce(1000, Seconds(1), true);
        NS_TEST_ASSERT_MSG_EQ(CountIn(aged, 1.05, 3.05), 10u, "maxBufferAge 1 s keeps the last second");
        NS_TEST_ASSERT_MSG_EQ(CountIn(aged, 1.05, 2.04), 0u, "older reports are discarded");
        NS_TEST_ASSERT_MSG_EQ(aged.dropped, 10u, "and counted");

        const Out noBatch = RunOnce(1000, Time(0), false);
        NS_TEST_ASSERT_MSG_EQ(CountIn(noBatch, 1.05, 3.05), 0u, "no batching: outage reports lost");
        NS_TEST_ASSERT_MSG_EQ(noBatch.dropped, 20u, "and counted as drops");
        NS_TEST_ASSERT_MSG_EQ(CountIn(noBatch, 3.05, 4.0), 9u, "reporting resumes after restoration");

        const Out again = RunOnce(15, Time(0), true);
        NS_TEST_ASSERT_MSG_EQ((again.orig == small.orig), true, "deterministic delivery sequence");
        NS_TEST_ASSERT_MSG_EQ((again.at == small.at), true, "deterministic delivery times");
    }
};

// ---------------------------------------------------------------------------
/// Cross-layer, packet level: a 2 Mb/s UDP flow over a P2P link is measured
/// by FlowMonitor every 100 ms; the per-window throughput goes into the E2
/// node's KPM report, through a 25 ms feeder link and a 0.5 s feeder outage.
/// Every indication the RIC receives carries exactly the throughput measured
/// in its window, including the buffered ones, and the mean equals the
/// offered IP-layer rate (2.056 Mb/s for 1000-byte payloads).
class OranE2FlowMonitorKpmTest : public TestCase
{
  public:
    OranE2FlowMonitorKpmTest()
        : TestCase("FlowMonitor-measured throughput reaches the RIC intact through outage buffering")
    {
    }

  private:
    void DoRun() override
    {
        NodeContainer nodes;
        nodes.Create(2);
        InternetStackHelper inet;
        inet.Install(nodes);
        PointToPointHelper p2p;
        p2p.SetDeviceAttribute("DataRate", StringValue("20Mbps"));
        p2p.SetChannelAttribute("Delay", StringValue("10ms"));
        NetDeviceContainer d = p2p.Install(nodes);
        Ipv4AddressHelper ip;
        ip.SetBase("10.28.0.0", "255.255.255.252");
        Ipv4InterfaceContainer ifs = ip.Assign(d);
        OnOffHelper onoff("ns3::UdpSocketFactory", InetSocketAddress(ifs.GetAddress(1), 9));
        onoff.SetConstantRate(DataRate("2Mbps"), 1000);
        ApplicationContainer src = onoff.Install(nodes.Get(0));
        src.Start(Seconds(0.0));
        src.Stop(Seconds(4.0));
        PacketSinkHelper ps("ns3::UdpSocketFactory", InetSocketAddress(Ipv4Address::GetAny(), 9));
        ps.Install(nodes.Get(1));
        FlowMonitorHelper fmh;
        Ptr<FlowMonitor> fm = fmh.InstallAll();

        Rx rx;
        Ptr<OranNtnE2Node> n = MakeNode(&rx, MilliSeconds(25));
        n->SetPeriodicReporting(false); // event-free: report per submitted measurement
        E2Subscription s = KpmSub(3, Time(0), true, 1e9); // every radio-valid report qualifies
        Simulator::Schedule(Seconds(0), [n, s]() { n->HandleSubscriptionRequest(s); });
        std::map<double, double> measured; // window end -> Mb/s
        uint64_t lastBytes = 0;
        for (int k = 1; k <= 40; ++k)
        {
            Simulator::Schedule(MilliSeconds(100 * k), [&, n]() {
                uint64_t bytes = 0;
                for (const auto& [id, st] : fm->GetFlowStats())
                {
                    bytes += st.rxBytes;
                }
                const double mbps = (bytes - lastBytes) * 8.0 / 0.1 / 1e6;
                lastBytes = bytes;
                E2KpmReport r = Report(1, 15.0);
                r.throughput_Mbps = mbps;
                measured[r.timestamp] = mbps;
                n->SubmitKpmMeasurement(r);
            });
        }
        Simulator::Schedule(MilliSeconds(1550), [n]() { n->SetFeederLinkAvailable(false); });
        Simulator::Schedule(MilliSeconds(2050), [n]() { n->SetFeederLinkAvailable(true); });
        Simulator::Stop(Seconds(4.5));
        Simulator::Run();
        Simulator::Destroy();

        NS_TEST_ASSERT_MSG_EQ(rx.ind.size(), 40u, "every window reported, outage ones buffered");
        uint32_t mismatched = 0;
        uint32_t buffered = 0;
        double sum = 0;
        for (const auto& i : rx.ind)
        {
            auto it = measured.find(i.kpmReport.timestamp);
            mismatched += it == measured.end() || it->second != i.kpmReport.throughput_Mbps;
            buffered += i.isBuffered;
            if (i.kpmReport.timestamp > 0.15 && i.kpmReport.timestamp < 3.95)
            {
                sum += i.kpmReport.throughput_Mbps;
            }
        }
        NS_TEST_ASSERT_MSG_EQ(mismatched, 0u, "the RIC sees the throughput measured in each window");
        NS_TEST_ASSERT_MSG_EQ(buffered, 5u, "the 5 windows inside the outage arrive buffered");
        NS_TEST_ASSERT_MSG_EQ_TOL(sum / 38.0, 2.056, 0.03, "mean KPM throughput = offered IP-layer rate");
    }
};

/**
 * The gym env must expose the action an external agent chose (the Python path
 * steps it through Notify(), so the scenario never sees the container), and
 * the PHY-KPM extractor must report exactly the SINR it was fed for a
 * registered RNTI while an unregistered UE reads as "no data" (sinrMeasured
 * false). Both were silently wrong in the gym handover example until 18 Nov.
 */
class OranGymLastActionAndExtractorFeedTest : public TestCase
{
  public:
    OranGymLastActionAndExtractorFeedTest()
        : TestCase("gym env reports the last action; extractor reports the fed SINR")
    {
    }

  private:
    void DoRun() override
    {
        Ptr<OranNtnGymHandover> env = CreateObject<OranNtnGymHandover>();
        NS_TEST_ASSERT_MSG_EQ(env->GetLastAction(), 0u, "no action yet reads as stay");
        for (uint32_t a : {2u, 0u, 1u})
        {
            Ptr<OpenGymDiscreteContainer> act = CreateObject<OpenGymDiscreteContainer>(4);
            act->SetValue(a);
            env->ExecuteActions(act); // no xApp: nothing is routed, the action is still recorded
            NS_TEST_ASSERT_MSG_EQ(env->GetLastAction(), a, "GetLastAction returns the executed action");
        }

        Ptr<OranNtnPhyKpmExtractor> ex = CreateObject<OranNtnPhyKpmExtractor>();
        ex->SetRadioGeometry(20e6, 7.0);
        const E2KpmReport none = ex->GetRealKpmReport(0);
        NS_TEST_ASSERT_MSG_EQ(none.sinrMeasured, false, "an unfed UE is not labelled measured");
        ex->RegisterRnti(7, 0, 1);
        ex->IngestMeasuredSample(7, 18.25, 125000, 0.08);
        ex->IngestMeasuredSample(99, 3.0, 1000, 0.5); // unknown RNTI: dropped
        const E2KpmReport rep = ex->GetRealKpmReport(0);
        NS_TEST_ASSERT_MSG_EQ(rep.sinrMeasured, true, "a fed UE is labelled measured");
        NS_TEST_ASSERT_MSG_EQ_TOL(rep.sinr_dB, 18.25, 1e-9, "report carries the ingested SINR");
        NS_TEST_ASSERT_MSG_EQ(rep.gnbId, 1u, "serving cell from RegisterRnti");
        // RSRP = SINR + kTB + NF at 20 MHz: -174 + 73.01 + 7 = -93.99 dBm.
        NS_TEST_ASSERT_MSG_EQ_TOL(rep.rsrp_dBm, 18.25 - 93.9897, 1e-3, "RSRP against the configured carrier");
    }
};

class OranNtnInDepthTestSuite : public TestSuite
{
  public:
    OranNtnInDepthTestSuite()
        : TestSuite("oran-ntn-indepth", Type::UNIT)
    {
        AddTestCase(new OranE2PeriodicKpmTest, Duration::QUICK);
        AddTestCase(new OranE2SubscriptionLifecycleTest, Duration::QUICK);
        AddTestCase(new OranE2EventTriggerTest, Duration::QUICK);
        AddTestCase(new OranE2FeederOutageBufferTest, Duration::QUICK);
        AddTestCase(new OranE2FlowMonitorKpmTest, Duration::QUICK);
        AddTestCase(new OranGymLastActionAndExtractorFeedTest, Duration::QUICK);
    }
};

static OranNtnInDepthTestSuite g_oranNtnInDepthTestSuite;
