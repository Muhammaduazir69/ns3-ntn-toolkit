/*
 * SPDX-License-Identifier: GPL-2.0-only
 * Copyright (c) 2026 Muhammad Uzair (ns3-ntn-toolkit, W6)
 */
#include "ns3/ntn-slice-helper.h"
#include "ns3/ntn-slice-selector.h"
#include "ns3/ntn-slice-types.h"
#include "ns3/slice-isolation-monitor.h"
#include "ns3/slice-orchestrator-xapp.h"
#include "ns3/simulator.h"
#include "ns3/nstime.h"
#include "ns3/test.h"

using namespace ns3;
using namespace ns3::ntnslice;

namespace
{

// 1. SST/SD pack/unpack round-trip.
class SnssaiPackRoundTripTest : public TestCase
{
  public:
    SnssaiPackRoundTripTest()
        : TestCase("S-NSSAI 32-bit pack-unpack round-trips for all SSTs")
    {
    }

    void DoRun() override
    {
        for (auto sst : {SliceSst::Embb, SliceSst::Urllc, SliceSst::Mmtc, SliceSst::V2x})
        {
            for (uint32_t sd : {0u, 1u, 0xABCDEFu, kSdUnset})
            {
                Snssai s{sst, sd};
                uint32_t packed = PackSnssai(s);
                Snssai roundTrip = UnpackSnssai(packed);
                NS_TEST_ASSERT_MSG_EQ(PackSnssai(roundTrip), PackSnssai(s),
                                      "round-trip failed for sst=" << static_cast<int>(sst)
                                      << " sd=" << sd);
            }
        }
    }
};

// 2. Selector first-match-wins on rule priority.
class SelectorFirstMatchTest : public TestCase
{
  public:
    SelectorFirstMatchTest()
        : TestCase("Selector returns first matching rule, default if none")
    {
    }

    void DoRun() override
    {
        auto sel = CreateObject<NtnSliceSelector>();
        Snssai embb{SliceSst::Embb, kSdUnset};
        Snssai urllc{SliceSst::Urllc, kSdUnset};
        Snssai mmtc{SliceSst::Mmtc, kSdUnset};
        sel->SetDefault(embb);
        sel->AddDscpRule(46, urllc);
        sel->AddPortRangeRule(5683, 5684, mmtc);
        sel->AddAppLabelRule("urllc-app", urllc);

        FlowDescriptor f1{};
        f1.dscp = 46;
        NS_TEST_ASSERT_MSG_EQ(PackSnssai(sel->Match(f1)), PackSnssai(urllc), "DSCP 46 → URLLC");

        FlowDescriptor f2{};
        f2.dstPort = 5683;
        NS_TEST_ASSERT_MSG_EQ(PackSnssai(sel->Match(f2)), PackSnssai(mmtc), "CoAP port → mMTC");

        FlowDescriptor f3{};
        f3.appLabel = "urllc-app";
        NS_TEST_ASSERT_MSG_EQ(PackSnssai(sel->Match(f3)), PackSnssai(urllc), "label → URLLC");

        FlowDescriptor f4{};
        f4.dstPort = 80;
        NS_TEST_ASSERT_MSG_EQ(PackSnssai(sel->Match(f4)), PackSnssai(embb), "default → eMBB");
    }
};

// 3. Orchestrator must guarantee minThroughputMbps even under congestion.
class OrchestratorMinThroughputTest : public TestCase
{
  public:
    OrchestratorMinThroughputTest()
        : TestCase("Orchestrator preserves URLLC min throughput under heavy eMBB load")
    {
    }

    void DoRun() override
    {
        auto stack = NtnSliceHelper::ThreeSliceDefault();
        stack.orchestrator->SetTotalPrb(100); // tight budget = 60 Mbps capacity
        stack.orchestrator->SetPrbCapacityMbps(0.6);

        // Heavy eMBB load that would hog all PRBs without min reservation.
        stack.orchestrator->SetDemand(stack.embb.snssai, 1000.0);
        stack.orchestrator->SetDemand(stack.urllc.snssai, 5.0);
        stack.orchestrator->SetDemand(stack.mmtc.snssai, 1.0);

        auto ticks = stack.orchestrator->Step();
        for (auto& t : ticks)
        {
            if (t.snssai.sst == SliceSst::Urllc)
            {
                NS_TEST_ASSERT_MSG_GT_OR_EQ(t.servedMbps, 5.0,
                                            "URLLC under-served (got "
                                            << t.servedMbps << " Mbps)");
            }
        }
    }
};

// 4. Isolation monitor flags URLLC latency breach when budget exceeded.
class IsolationMonitorBreachTest : public TestCase
{
  public:
    IsolationMonitorBreachTest()
        : TestCase("Monitor flags URLLC p99 > 5 ms as latency breach")
    {
    }

    void DoRun() override
    {
        auto stack = NtnSliceHelper::ThreeSliceDefault();
        // 200 packets — 5 % at 200 ms (above URLLC's 5 ms budget) → p99 is in the tail.
        for (int i = 0; i < 200; ++i)
        {
            double lat = (i % 20 == 0) ? 200.0 : 1.0;  // 10/200 = 5 % above budget
            stack.monitor->RecordPacket(stack.urllc.snssai, lat, true);
        }
        auto evs = stack.monitor->EvaluateAll();
        bool sawUrllcBreach = false;
        for (auto& ev : evs)
        {
            if (PackSnssai(ev.snssai) == PackSnssai(stack.urllc.snssai))
            {
                sawUrllcBreach = ev.latencyBreach;
            }
        }
        NS_TEST_ASSERT_MSG_EQ(sawUrllcBreach, true,
                              "URLLC latency breach not flagged");
    }
};

// 5. URLLC profile forbids GEO; helper exposes that policy unambiguously.
class UrllcGeoSkipTest : public TestCase
{
  public:
    UrllcGeoSkipTest()
        : TestCase("URLLC slice mandates GEO mode-skip (allowGeo=false)")
    {
    }

    void DoRun() override
    {
        SliceProfile urllc = DefaultUrllc();
        SliceProfile embb = DefaultEmbb();
        SliceProfile mmtc = DefaultMmtc();
        NS_TEST_ASSERT_MSG_EQ(ShouldSkipGeo(urllc), true, "URLLC must skip GEO");
        NS_TEST_ASSERT_MSG_EQ(ShouldSkipGeo(embb), false, "eMBB allows GEO");
        NS_TEST_ASSERT_MSG_EQ(ShouldSkipGeo(mmtc), false, "mMTC allows GEO");

        // TEST-1. The three assertions above cannot distinguish the two terms
        // of the predicate. ShouldSkipGeo is `sst == Urllc || !allowGeo`, and
        // DefaultUrllc() sets BOTH - sst to Urllc and allowGeo to false - so
        // deleting either term leaves all three passing. The test agreed with
        // the definition on the module's own defaults and nothing more.
        //
        // Isolate the terms. Each of these fails if the corresponding half of
        // the predicate is dropped.
        SliceProfile urllcAllowingGeo = DefaultUrllc();
        urllcAllowingGeo.allowGeo = true;
        NS_TEST_ASSERT_MSG_EQ(ShouldSkipGeo(urllcAllowingGeo), true,
                              "a URLLC slice must skip GEO on its service class alone, even if "
                              "its allowGeo flag says otherwise: the propagation floor is a "
                              "property of the orbit, not of a configuration bit. Dropping the "
                              "sst term leaves the original three assertions passing");

        SliceProfile embbForbiddingGeo = DefaultEmbb();
        embbForbiddingGeo.allowGeo = false;
        NS_TEST_ASSERT_MSG_EQ(ShouldSkipGeo(embbForbiddingGeo), true,
                              "a non-URLLC slice that explicitly forbids GEO must skip it too; "
                              "dropping the allowGeo term leaves the original three passing");
        NS_TEST_ASSERT_MSG_EQ(IsGeoSatellite(35786.0), true, "GEO altitude classification");
        NS_TEST_ASSERT_MSG_EQ(IsGeoSatellite(550.0), false, "LEO not GEO");
    }
};

// 6. Three-slice co-existence: all three must hit min under contention.
class ThreeSliceCoexistenceTest : public TestCase
{
  public:
    ThreeSliceCoexistenceTest()
        : TestCase("eMBB plus URLLC plus mMTC co-exist on one cell without isolation breach")
    {
    }

    void DoRun() override
    {
        auto stack = NtnSliceHelper::ThreeSliceDefault();
        stack.orchestrator->SetTotalPrb(150);  // tight, but enough for all mins (50+5+1)
        stack.orchestrator->SetPrbCapacityMbps(0.6);
        stack.orchestrator->SetDemand(stack.embb.snssai, 200.0);
        stack.orchestrator->SetDemand(stack.urllc.snssai, 80.0);
        stack.orchestrator->SetDemand(stack.mmtc.snssai, 10.0);

        auto ticks = stack.orchestrator->Step();
        bool allMet = true;
        for (auto& t : ticks)
        {
            if (!t.minThroughputMet)
            {
                allMet = false;
            }
        }
        NS_TEST_ASSERT_MSG_EQ(allMet, true, "at least one slice missed its min throughput");
    }
};

// 7. RL hook: external shares are honoured.
class OrchestratorExternalSharesTest : public TestCase
{
  public:
    OrchestratorExternalSharesTest()
        : TestCase("StepWithShares uses external action vector")
    {
    }

    void DoRun() override
    {
        auto stack = NtnSliceHelper::ThreeSliceDefault();
        stack.orchestrator->SetTotalPrb(273);
        stack.orchestrator->SetDemand(stack.embb.snssai, 100.0);
        stack.orchestrator->SetDemand(stack.urllc.snssai, 100.0);
        stack.orchestrator->SetDemand(stack.mmtc.snssai, 100.0);

        std::map<Snssai, double> shares = {
            {stack.embb.snssai, 0.6},
            {stack.urllc.snssai, 0.3},
            {stack.mmtc.snssai, 0.1},
        };
        auto ticks = stack.orchestrator->StepWithShares(shares);
        // eMBB should outserve mMTC by ~6×
        double embbServed = 0, mmtcServed = 0;
        for (auto& t : ticks)
        {
            if (t.snssai.sst == SliceSst::Embb) embbServed = t.servedMbps;
            if (t.snssai.sst == SliceSst::Mmtc) mmtcServed = t.servedMbps;
        }
        NS_TEST_ASSERT_MSG_GT(embbServed, mmtcServed * 2.0,
                              "external 0.6 vs 0.1 share not honoured");
    }
};

/// SLICE-2: the reported loss rate must be a loss rate, not an insertion order.
///
/// observedLossRate was computed over deliveredWindow, a 200-sample FIFO that
/// pops from the front. Both real-stack examples record every DELIVERED packet
/// first and every LOST packet afterwards, so on any run with more than 200
/// deliveries the window held only the tail of the input. With 200 or more
/// losses it read 100 percent loss whatever the true rate; with fewer it read
/// lost/200, a number set by the window size rather than by the traffic. The
/// URLLC 1e-5 reliability target could therefore breach on a slice that
/// delivered 99.9 percent of its packets, and pass on one that did not.
///
/// This reproduces the examples' recording order exactly and asserts the
/// reported rate is the real one.
class SliceIsolationLossRateIsCumulativeTest : public TestCase
{
  public:
    SliceIsolationLossRateIsCumulativeTest()
        : TestCase("SLICE-2: loss rate is cumulative, not the tail of a 200-sample window")
    {
    }

  private:
    void DoRun() override
    {
        SliceIsolationMonitor mon;
        SliceProfile p;
        p.snssai = Snssai{SliceSst::Embb, 1};
        p.latencyBudgetMs = 50.0;
        p.reliability = 0.99;
        mon.RegisterSlice(p);

        // The order both examples use: all deliveries, then all losses.
        // 1000 delivered, 10 lost -> a true loss rate of 10/1010 = 0.99 percent,
        // comfortably inside a 0.99 reliability target.
        for (int i = 0; i < 1000; ++i)
        {
            mon.RecordPacket(p.snssai, 5.0, /*delivered=*/true);
        }
        for (int i = 0; i < 10; ++i)
        {
            mon.RecordPacket(p.snssai, 5.0, /*delivered=*/false);
        }

        auto evs = mon.EvaluateAll();
        NS_TEST_ASSERT_MSG_EQ(evs.empty(), false, "the monitor must evaluate the slice");
        const auto& ev = evs.front();

        NS_TEST_ASSERT_MSG_EQ_TOL(ev.observedLossRate, 10.0 / 1010.0, 1e-6,
                                  "the loss rate must be lost/(delivered+lost) over everything "
                                  "recorded; a windowed value would report 10/200 = 5 percent "
                                  "here, five times the truth, purely because the losses were "
                                  "appended last");
        NS_TEST_ASSERT_MSG_EQ(ev.reliabilityBreach, false,
                              "a slice delivering 99.01 percent must not breach a 0.99 "
                              "reliability target");

        // The pathological case the window made unreachable: enough losses to
        // fill the window entirely. The true rate is still under 20 percent.
        SliceIsolationMonitor mon2;
        mon2.RegisterSlice(p);
        for (int i = 0; i < 1000; ++i)
        {
            mon2.RecordPacket(p.snssai, 5.0, true);
        }
        for (int i = 0; i < 220; ++i)
        {
            mon2.RecordPacket(p.snssai, 5.0, false);
        }
        const auto ev2 = mon2.EvaluateAll().front();
        NS_TEST_ASSERT_MSG_EQ_TOL(ev2.observedLossRate, 220.0 / 1220.0, 1e-6,
                                  "with more losses than the window holds, a windowed rate reads "
                                  "100 percent loss on a slice that delivered 82 percent of its "
                                  "packets");
    }
};

/// TEST-1: the slice suite must exercise the simulator at least once.
///
/// Every test in this file was static: seven cases, zero Simulator::Run(), no
/// NetDevice, no bandwidth part, no scheduler. The suite checked that helper
/// predicates agree with their own definitions, which is documentation with an
/// assert around it.
///
/// This drives the isolation monitor through scheduled simulation events, so
/// the module is exercised in the event loop it actually runs in, and asserts
/// the SLA verdict tracks what was recorded over simulated time rather than
/// over insertion order (the defect SLICE-2 fixed).
class SliceMonitorUnderSimulatorTest : public TestCase
{
  public:
    SliceMonitorUnderSimulatorTest()
        : TestCase("TEST-1: the isolation monitor evaluates under Simulator::Run")
    {
    }

  private:
    SliceIsolationMonitor m_mon;

    void FeedGood(Snssai s)
    {
        m_mon.RecordPacket(s, 3.0, true);
    }

    void FeedLate(Snssai s)
    {
        m_mon.RecordPacket(s, 400.0, true);
    }

    void DoRun() override
    {
        SliceProfile p = DefaultUrllc();
        m_mon.RegisterSlice(p);

        // 200 on-time packets spread across the first second of simulated time,
        // then a burst of very late ones. Scheduling them rather than looping
        // means the monitor is driven the way a scenario drives it.
        for (uint32_t i = 0; i < 200; ++i)
        {
            Simulator::Schedule(MilliSeconds(i * 5),
                                &SliceMonitorUnderSimulatorTest::FeedGood, this, p.snssai);
        }
        for (uint32_t i = 0; i < 40; ++i)
        {
            Simulator::Schedule(Seconds(1.0) + MilliSeconds(i * 5),
                                &SliceMonitorUnderSimulatorTest::FeedLate, this, p.snssai);
        }

        Simulator::Stop(Seconds(2.0));
        Simulator::Run();

        const auto evs = m_mon.EvaluateAll();
        NS_TEST_ASSERT_MSG_EQ(evs.empty(), false, "the monitor must evaluate the slice");
        const auto& ev = evs.front();

        // 40 packets at 400 ms out of 240 total puts the p99 firmly in the late
        // tail, so a URLLC slice with a 5 ms budget must breach.
        NS_TEST_ASSERT_MSG_GT(ev.observedLatencyP99Ms, 100.0,
                              "the p99 must reflect the late burst; a value near the on-time "
                              "figure means the tail is not reaching the percentile");
        NS_TEST_ASSERT_MSG_EQ(ev.latencyBreach, true,
                              "a URLLC slice whose p99 is hundreds of ms must breach its latency "
                              "budget");

        // Everything was delivered, so the reliability side must NOT breach -
        // the two verdicts have to move independently, which is what SLICE-2's
        // cumulative loss rate makes possible.
        NS_TEST_ASSERT_MSG_EQ(ev.reliabilityBreach, false,
                              "no packet was lost, so a latency breach must not drag the "
                              "reliability verdict with it");

        Simulator::Destroy();
    }
};

class NtnSliceTestSuite : public TestSuite
{
  public:
    NtnSliceTestSuite()
        : TestSuite("ntn-slice", Type::UNIT)
    {
        AddTestCase(new SnssaiPackRoundTripTest, TestCase::Duration::QUICK);
        AddTestCase(new SelectorFirstMatchTest, TestCase::Duration::QUICK);
        AddTestCase(new OrchestratorMinThroughputTest, TestCase::Duration::QUICK);
        AddTestCase(new IsolationMonitorBreachTest, TestCase::Duration::QUICK);
        AddTestCase(new UrllcGeoSkipTest, TestCase::Duration::QUICK);
        AddTestCase(new ThreeSliceCoexistenceTest, TestCase::Duration::QUICK);
        AddTestCase(new OrchestratorExternalSharesTest, TestCase::Duration::QUICK);
        AddTestCase(new SliceMonitorUnderSimulatorTest, TestCase::Duration::QUICK);
        AddTestCase(new SliceIsolationLossRateIsCumulativeTest,
                    TestCase::Duration::QUICK);
    }
};

static NtnSliceTestSuite g_ntnSliceTestSuite;

} // namespace
