/*
 * SPDX-License-Identifier: GPL-2.0-only
 * Copyright (c) 2026 Muhammad Uzair (ns3-ntn-toolkit, W6)
 */
#include "ns3/ntn-slice-helper.h"
#include "ns3/ntn-slice-selector.h"
#include "ns3/ntn-slice-types.h"
#include "ns3/slice-isolation-monitor.h"
#include "ns3/slice-orchestrator-xapp.h"
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
    }
};

static NtnSliceTestSuite g_ntnSliceTestSuite;

} // namespace
