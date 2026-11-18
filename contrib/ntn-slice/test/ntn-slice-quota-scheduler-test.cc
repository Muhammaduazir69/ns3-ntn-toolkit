/*
 * SPDX-License-Identifier: GPL-2.0-only
 * Copyright (c) 2026 Muhammad Uzair (ns3-ntn-toolkit)
 *
 * End-to-end tests of NtnSliceQuotaScheduler on a real NR cell: one SGP4 satellite at 550 km, TR 38.811 UEs, the
 * 5G-LENA stack through NtnRealStackHelper, UDP flows from the remote host. The scheduler's own per-slot trace and RBG
 * counters are checked against the quotas, and the packets the UEs receive are checked against the RBGs their slice
 * was given. One case drives the quotas through the O-RAN E2SM-RC slice actuator (oran-ntn).
 */
#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/nr-module.h"
#include "ns3/ntn-oran-application.h"
#include "ns3/ntn-real-stack-helper.h"
#include "ns3/ntn-slice-contract-trim.h"
#include "ns3/ntn-slice-quota-scheduler.h"
#include "ns3/ntn-tr38811-mobility-model.h"
#include "ns3/oran-ntn-e2-interface.h"
#include "ns3/oran-ntn-helper.h"
#include "ns3/oran-ntn-near-rt-ric.h"
#include "ns3/oran-ntn-real-stack-binding.h"
#include "ns3/sgp4-mobility-model.h"
#include "ns3/test.h"
#include "ns3/walker-constellation.h"

#include <array>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <numeric>

using namespace ns3;
using namespace ns3::ntnslice;

namespace
{

// MixedBouquet gives UE u the profile u % 3: 0 mMTC, 1 eMBB, 2 URLLC; scheduler slices are 0 eMBB, 1 URLLC, 2 mMTC.
constexpr uint32_t kSliceOfProfile[3] = {2, 0, 1};
constexpr uint32_t kUes = 6;

/// What one run of the cell measured in a window.
struct Window
{
    std::vector<uint64_t> rbg;          //!< RBGs per scheduler slice (TakeRbgCounts)
    uint64_t slots{0};                  //!< DL slots the scheduler counted
    std::vector<double> bytesPerRbg;    //!< TB bytes per RBG per slice (TakeBytesPerRbg)
    std::array<uint64_t, 3> appBytes{}; //!< application bytes the UEs received, per profile slice
};

struct RigResult
{
    Window w1; //!< 1.0 to 2.0 s
    Window w2; //!< 2.0 to 2.9 s
    uint32_t rbgPerSlot{0};
    uint32_t rbPerRbg{0};
    uint32_t phyRbs{0};
    uint32_t bwpPrbs{0};
    uint64_t phySlots{0};          //!< gNB PHY DL slots (SlotDataStats) in 1.0 to 2.0 s
    uint64_t phyDataSlots{0};      //!< of which carried PDSCH symbols
    double slotPeriodMs{0.0};
    uint64_t tracedSlots{0};       //!< slots seen on the SliceAllocation trace in 1.0 to 2.9 s
    uint64_t overflowSlots{0};     //!< slots that gave more RBGs than the slot has
    std::vector<uint64_t> atLimit; //!< per slice, slots in which it got at least floor(q N)
    std::array<uint64_t, 3> rxPkts{};
    std::array<uint64_t, 3> txPkts{};
    uint64_t totalAppBytes{0};
    std::vector<double> quotasAtEnd;
    uint32_t urllcSliceOfRnti{0};
};

struct RigOptions
{
    std::vector<double> quotas{0.6, 0.3, 0.1};
    double embbMbps{60.0};
    double urllcMbps{40.0};
    bool workConserving{true};
    std::string scheduler{"ns3::ntnslice::NtnSliceQuotaScheduler"};
    bool assignSlices{true};
    uint32_t urllcSlice{1}; //!< scheduler slice the URLLC UEs are put in
    uint32_t run{1};
    /// Called after Build with the satellite nodes, the stack and the scheduler (may be null).
    std::function<void(NodeContainer&, NtnRealStackHelper&, Ptr<NtnSliceQuotaScheduler>)> afterBuild;
};

/// Application bytes received so far, summed per scheduler slice of the UE's profile.
std::array<uint64_t, 3>
AppBytes(const NtnRealStackHelper& rs)
{
    std::array<uint64_t, 3> b{};
    for (uint32_t u = 0; u < kUes; ++u)
    {
        b[kSliceOfProfile[u % 3]] += rs.GetUeRxBytes(u);
    }
    return b;
}

std::array<uint64_t, 3>
Diff(const std::array<uint64_t, 3>& a, const std::array<uint64_t, 3>& b)
{
    return {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
}

/**
 * Run a 3 s NR cell with six UEs (two per profile) from 0.5 to 2.9 s of traffic. eMBB UEs offer opt.embbMbps each,
 * URLLC UEs opt.urllcMbps each (0 makes the slice idle), mMTC keeps its light MixedBouquet load.
 */
RigResult
RunRig(const RigOptions& opt)
{
    RngSeedManager::SetSeed(1);
    RngSeedManager::SetRun(opt.run);
    RngSeedManager::ResetNextStreamIndex();
    Config::SetDefault("ns3::NrHelper::UseIdealRrc", BooleanValue(true));
    NodeContainer sat;
    sat.Create(1);
    NodeContainer ues;
    ues.Create(kUes);
    ns3::ntncon::WalkerConfig w;
    w.num_planes = 1;
    w.total_sats = 80;
    w.altitude_km = 550.0;
    w.inclination_deg = 53.0;
    w.epoch_unix_s = 1735689600.0;
    auto mob = CreateObject<ns3::ntncon::Sgp4MobilityModel>();
    mob->SetElements(ns3::ntncon::WalkerConstellation::BuildDelta(w)[0]);
    sat.Get(0)->AggregateObject(mob);
    double lat, lon, alt;
    mob->GetGeodetic(lat, lon, alt);
    NtnTr38811MobilityHelper um(1);
    um.Install(ues, NtnMobilityScenarios::MixedContinental(), lat - 0.03, lat + 0.03, lon - 0.03, lon + 0.03);

    NtnRealStackHelper rs;
    rs.SetRadioBackend(NtnRealStackHelper::RadioBackend::Nr);
    rs.SetNumerology(1);
    rs.SetSimTime(Seconds(3));
    rs.SetSatConductedPowerDbm(55);
    rs.SetBackhaulDelay(MilliSeconds(5));
    rs.SetSchedulerTypeName(opt.scheduler);
    rs.Build(sat, ues);
    rs.InstallTraffic(NtnRealStackHelper::TrafficProfile::MixedBouquet, Seconds(0.5), Seconds(2.9));
    Ptr<Node> rh = rs.GetRemoteHost();
    std::vector<Ptr<NtnOranApplication>> apps;
    uint32_t k = 0;
    for (uint32_t i = 0; i < rh->GetNApplications(); ++i)
    {
        auto app = DynamicCast<NtnOranApplication>(rh->GetApplication(i));
        if (!app)
        {
            continue;
        }
        if (k % 3 == 1)
        {
            app->SetAttribute("DataRate", DataRateValue(DataRate(opt.embbMbps * 1e6)));
        }
        else if (k % 3 == 2)
        {
            app->SetAttribute("PacketSize", UintegerValue(1000));
            app->SetAttribute("Period",
                              TimeValue(opt.urllcMbps > 0 ? Seconds(8000.0 / (opt.urllcMbps * 1e6))
                                                          : Seconds(100)));
        }
        apps.push_back(app);
        ++k;
    }
    auto gnb = DynamicCast<NrGnbNetDevice>(rs.GetEnbDevices().Get(0));
    NS_ABORT_MSG_IF(!gnb, "no NR gNB");
    Ptr<NtnSliceQuotaScheduler> sched = DynamicCast<NtnSliceQuotaScheduler>(gnb->GetScheduler(0));
    NS_ABORT_MSG_IF(opt.scheduler == "ns3::ntnslice::NtnSliceQuotaScheduler" && !sched,
                    "the gNB does not run NtnSliceQuotaScheduler");
    RigResult out;
    out.rbPerRbg = gnb->GetMac(0)->GetNumRbPerRbg();
    out.phyRbs = gnb->GetPhy(0)->GetRbNum();
    out.bwpPrbs = rs.GetBwpPrbs();
    out.slotPeriodMs = gnb->GetPhy(0)->GetSlotPeriod().GetSeconds() * 1e3;
    gnb->GetPhy(0)->TraceConnectWithoutContext(
        "SlotDataStats",
        MakeBoundCallback(
            +[](RigResult* o,
                const SfnSf&,
                uint32_t,
                uint32_t,
                uint32_t dataSym,
                uint32_t,
                uint32_t,
                uint16_t,
                uint16_t) {
                const double now = Simulator::Now().GetSeconds();
                if (now >= 1.0 && now < 2.0)
                {
                    ++o->phySlots;
                    o->phyDataSlots += dataSym > 0 ? 1 : 0;
                }
            },
            &out));
    if (sched)
    {
        sched->SetAttribute("WorkConserving", BooleanValue(opt.workConserving));
        sched->SetQuotas(opt.quotas);
        out.atLimit.assign(std::max<size_t>(opt.quotas.size(), 1), 0);

        // Per-slot accounting from the scheduler's own trace: one call per slice per slot, slice 0 first.
        struct SlotAcc
        {
            std::vector<uint32_t> given;
            uint32_t total{0};
        };
        auto acc = std::make_shared<SlotAcc>();
        NtnSliceQuotaScheduler* raw = PeekPointer(sched);
        auto finalize = [acc, raw, &out]() {
            if (acc->given.empty())
            {
                return;
            }
            const double now = Simulator::Now().GetSeconds();
            if (now >= 1.0 && now < 2.9)
            {
                ++out.tracedSlots;
                const uint32_t sum = std::accumulate(acc->given.begin(), acc->given.end(), 0U);
                out.overflowSlots += sum > acc->total ? 1 : 0;
                const auto q = raw->GetQuotas();
                for (size_t s = 0; s < acc->given.size() && s < q.size() && s < out.atLimit.size(); ++s)
                {
                    const auto lim = static_cast<uint32_t>(std::floor(q[s] * acc->total + 1e-9));
                    out.atLimit[s] += acc->given[s] >= lim ? 1 : 0;
                }
            }
            acc->given.clear();
        };
        auto cb = std::make_shared<std::function<void(uint32_t, uint32_t, uint32_t)>>(
            [acc, finalize, &out](uint32_t slice, uint32_t given, uint32_t total) {
                if (slice == 0)
                {
                    finalize();
                }
                acc->given.push_back(given);
                acc->total = total;
                out.rbgPerSlot = total;
            });
        sched->TraceConnectWithoutContext(
            "SliceAllocation",
            MakeBoundCallback(
                +[](std::function<void(uint32_t, uint32_t, uint32_t)>* f, uint32_t s, uint32_t g, uint32_t t) {
                    (*f)(s, g, t);
                },
                cb.get()));
        // Keep the closures alive for the run.
        Simulator::ScheduleDestroy([cb, finalize]() { finalize(); });
    }
    if (opt.afterBuild)
    {
        opt.afterBuild(sat, rs, sched);
    }

    if (opt.assignSlices && sched)
    {
        Simulator::Schedule(Seconds(0.4), [&rs, sched, &opt]() {
            for (uint32_t u = 0; u < kUes; ++u)
            {
                const uint32_t p = u % 3;
                sched->SetSliceOfRnti(rs.GetUeRnti(u), p == 2 ? opt.urllcSlice : kSliceOfProfile[p]);
            }
        });
    }
    std::array<uint64_t, 3> b1{}, b2{};
    Simulator::Schedule(Seconds(1.0), [sched, &rs, &b1]() {
        if (sched)
        {
            sched->TakeRbgCounts(nullptr);
            sched->TakeBytesPerRbg();
            sched->TakeBacklog();
        }
        b1 = AppBytes(rs);
    });
    Simulator::Schedule(Seconds(2.0), [sched, &rs, &out, &b1, &b2]() {
        // TakeRbgCounts first on purpose: TakeBytesPerRbg must not depend on the order.
        if (sched)
        {
            out.w1.rbg = sched->TakeRbgCounts(&out.w1.slots);
            out.w1.bytesPerRbg = sched->TakeBytesPerRbg();
        }
        b2 = AppBytes(rs);
        out.w1.appBytes = Diff(b2, b1);
    });
    Simulator::Schedule(Seconds(2.9), [sched, &rs, &out, &b2]() {
        if (sched)
        {
            out.w2.rbg = sched->TakeRbgCounts(&out.w2.slots);
            out.w2.bytesPerRbg = sched->TakeBytesPerRbg();
        }
        out.w2.appBytes = Diff(AppBytes(rs), b2);
    });
    Simulator::Stop(Seconds(3));
    Simulator::Run();
    for (uint32_t u = 0; u < kUes; ++u)
    {
        const uint32_t s = kSliceOfProfile[u % 3];
        out.rxPkts[s] += rs.GetUeRxPackets(u);
        out.totalAppBytes += rs.GetUeRxBytes(u);
    }
    for (uint32_t a = 0; a < apps.size(); ++a)
    {
        out.txPkts[kSliceOfProfile[a % 3]] += apps[a]->GetTxPackets();
    }
    if (sched)
    {
        out.quotasAtEnd = sched->GetQuotas();
        out.urllcSliceOfRnti = sched->GetSliceOfRnti(rs.GetUeRnti(2));
    }
    Simulator::Destroy();
    return out;
}

double
Share(const std::vector<uint64_t>& c, size_t s)
{
    const double sum = std::accumulate(c.begin(), c.end(), 0.0);
    return (sum > 0 && s < c.size()) ? c[s] / sum : 0.0;
}

/// With eMBB and URLLC both saturating, each gets at least its quota (less one RBG of rounding per slot).
class QuotaRespectedUnderSaturationTest : public TestCase
{
  public:
    QuotaRespectedUnderSaturationTest()
        : TestCase("slice-quota scheduler: each saturated slice gets at least its quota")
    {
    }

    void DoRun() override
    {
        const auto r = RunRig({});
        const auto& c = r.w2.rbg;
        NS_TEST_ASSERT_MSG_GT(Share(c, 0) + Share(c, 1), 0.9, "the two saturated slices must use nearly every RBG");
        NS_TEST_ASSERT_MSG_GT_OR_EQ(Share(c, 1), 0.27, "URLLC is saturated and must get its 0.3 quota");
        NS_TEST_ASSERT_MSG_GT_OR_EQ(Share(c, 0), 0.57, "eMBB is saturated and must get its 0.6 quota");
        NS_TEST_ASSERT_MSG_LT(Share(c, 1), 0.45, "URLLC must not take eMBB's quota while eMBB has data");
    }
};

/// With the URLLC slice idle, eMBB takes the RBGs URLLC does not use (work conservation).
class WorkConservationTest : public TestCase
{
  public:
    WorkConservationTest()
        : TestCase("slice-quota scheduler: an idle slice's quota goes to a slice with data")
    {
    }

    void DoRun() override
    {
        RigOptions o;
        o.quotas = {0.3, 0.6, 0.1};
        o.urllcMbps = 0.0;
        const auto r = RunRig(o);
        NS_TEST_ASSERT_MSG_GT(Share(r.w2.rbg, 0), 0.85, "eMBB must use the RBGs that the idle URLLC slice leaves");
        NS_TEST_ASSERT_MSG_LT(Share(r.w2.rbg, 1), 0.05, "an idle URLLC slice must take almost no RBGs");
        NS_TEST_ASSERT_MSG_EQ(r.overflowSlots, 0u, "never more RBGs than the slot has");
    }
};

/**
 * The trim against a saturated tenant that wins leftovers, Y = c (floor(q N) / N + l): it must reach the target from
 * both sides, at the fixed point the Lemma gives when there is no rounding, and on average when there is, respect its
 * clipping, and recover after a period in which the tenant had the whole cell.
 */
class ContractTrimConvergenceTest : public TestCase
{
  public:
    ContractTrimConvergenceTest()
        : TestCase("Contract trim converges to the target despite leftovers and rounding")
    {
    }

  private:
    void DoRun() override
    {
        const double c = 1e6;
        const double l = 0.1;
        const double target = 0.4 * c; // fixed point q = 0.3
        for (double q0 : {0.1, 0.8})
        {
            double q = q0;
            for (int k = 0; k < 30; ++k)
            {
                q = TrimmedQuota(q, target, c * (q + l), 0.5, 0.1, 0.85);
            }
            NS_TEST_ASSERT_MSG_EQ_TOL(q, 0.3, 1e-3, "no rounding: fixed point of the Lemma");
        }
        // Rounding to RBGs: the average over the dither meets the target
        const double n = 53;
        double q = 0.1;
        double sum = 0;
        for (int k = 0; k < 400; ++k)
        {
            const double y = c * (std::floor(q * n) / n + l);
            if (k >= 100)
            {
                sum += y;
            }
            q = TrimmedQuota(q, target, y, 0.5, 0.1, 0.85);
        }
        NS_TEST_ASSERT_MSG_EQ_TOL(sum / 300, target, 0.02 * target, "rounding: average at the target");
        // Clipping, and the return of the other tenant: a whole cell drives q to its floor, and it recovers within 15 periods
        q = 0.3;
        for (int k = 0; k < 20; ++k)
        {
            q = TrimmedQuota(q, target, c, 0.5, 0.1, 0.85);
        }
        NS_TEST_ASSERT_MSG_EQ_TOL(q, 0.1, 1e-9, "clipped at the floor");
        for (int k = 0; k < 15; ++k)
        {
            q = TrimmedQuota(q, target, c * (q + l), 0.5, 0.1, 0.85);
        }
        NS_TEST_ASSERT_MSG_EQ_TOL(q, 0.3, 0.015, "recovered after the pause");
        NS_TEST_ASSERT_MSG_EQ_TOL(TrimmedQuota(0.8, 10 * c, c, 1.0, 0.1, 0.85), 0.85, 1e-12, "clipped at the cap");
    }
};

/**
 * Slot grid and per-slot guarantee. TS 38.211 Table 4.3.2-1: numerology 1 has 2 slots per 1 ms subframe (0.5 ms
 * slots), so the gNB PHY must run 2000 DL slots in a 1 s window of the FDD DL BWP (tolerance 0.5 %), and a saturated
 * cell must carry PDSCH in at least 99 % of them. The scheduler allocates new data in a subset of those slots (slots
 * taken whole by HARQ retransmissions are not offered to AssignDLRBG). The RBGs of a slot must equal the PHY's
 * resource blocks over the MAC's RBG size (5G-LENA NumRbPerRbg, the RBG of TS 38.214 5.1.2.2.1 type 0), and the PHY
 * RBs must equal the PRB count the helper reports as RRU.PrbAvailDl. Then, slot by slot, each saturated slice gets at
 * least floor(q N) RBGs in at least 95 % of the slots: the quota is enforced per slot, not only on average.
 */
class SlotGridAndPerSlotGuaranteeTest : public TestCase
{
  public:
    SlotGridAndPerSlotGuaranteeTest()
        : TestCase("TS 38.211 slot grid at mu=1, RBG count per slot, per-slot floor(q N) guarantee")
    {
    }

  private:
    void DoRun() override
    {
        const auto r = RunRig({});
        NS_TEST_ASSERT_MSG_EQ_TOL(r.slotPeriodMs, 0.5, 1e-9, "TS 38.211 Table 4.3.2-1: 0.5 ms slot at mu = 1");
        NS_TEST_ASSERT_MSG_EQ_TOL(static_cast<double>(r.phySlots), 2000.0, 10.0,
                                  "TS 38.211 Table 4.3.2-1: 2000 slots per second at mu = 1");
        NS_TEST_ASSERT_MSG_GT_OR_EQ(static_cast<double>(r.phyDataSlots), 0.99 * r.phySlots,
                                    "a saturated cell carries PDSCH in every slot");
        NS_TEST_ASSERT_MSG_LT_OR_EQ(r.w1.slots, r.phyDataSlots, "new-data slots are a subset of the data slots");
        NS_TEST_ASSERT_MSG_GT(static_cast<double>(r.w1.slots), 0.6 * r.phySlots,
                              "most slots carry new data under saturation");
        NS_TEST_ASSERT_MSG_GT(r.rbPerRbg, 0u, "the MAC has an RBG size");
        NS_TEST_ASSERT_MSG_EQ(r.rbgPerSlot, r.phyRbs / r.rbPerRbg, "RBGs per slot = PHY RBs / RBG size");
        NS_TEST_ASSERT_MSG_EQ(r.phyRbs, r.bwpPrbs, "RRU.PrbAvailDl is the PRB count the PHY schedules");
        NS_TEST_ASSERT_MSG_GT(r.tracedSlots, 2500u, "the trace covers the 1.0 to 2.9 s window");
        NS_TEST_ASSERT_MSG_EQ(r.overflowSlots, 0u, "never more RBGs than the slot has");
        const double eMbbOk = static_cast<double>(r.atLimit[0]) / r.tracedSlots;
        const double urllcOk = static_cast<double>(r.atLimit[1]) / r.tracedSlots;
        NS_TEST_ASSERT_MSG_GT_OR_EQ(eMbbOk, 0.95, "eMBB gets floor(0.6 N) RBGs in at least 95 % of slots");
        NS_TEST_ASSERT_MSG_GT_OR_EQ(urllcOk, 0.95, "URLLC gets floor(0.3 N) RBGs in at least 95 % of slots");
    }
};

/**
 * Packet-level KPI tracks the quota. Two saturated runs swap the eMBB and URLLC quotas (0.6/0.3 and 0.3/0.6). The
 * application bytes each slice's UEs receive (UDP sinks, not scheduler counters) must follow: URLLC goodput rises by
 * its RBG ratio within 10 %, eMBB falls. The RBG ratio is below 2 because work conservation hands the rounding
 * leftovers, the idle mMTC share and the slots in which the other slice has no free HARQ process to whoever has data
 * (measured 0.59/0.41 and 0.41/0.59). Each slice's goodput must also stay below the TB bytes the MAC gave it
 * (RBGs x measured bytes per RBG), since the TB carries the application bytes plus headers.
 */
class GoodputTracksQuotaTest : public TestCase
{
  public:
    GoodputTracksQuotaTest()
        : TestCase("Measured UDP goodput per slice follows the quota swap, bounded by the MAC TB bytes")
    {
    }

  private:
    void DoRun() override
    {
        RigOptions o;
        o.quotas = {0.6, 0.3, 0.1};
        const auto a = RunRig(o);
        o.quotas = {0.3, 0.6, 0.1};
        const auto b = RunRig(o);
        for (const auto* r : {&a, &b})
        {
            for (size_t s = 0; s < 2; ++s)
            {
                const double tb = r->w2.rbg[s] * r->w2.bytesPerRbg[s];
                NS_TEST_ASSERT_MSG_GT(r->w2.bytesPerRbg[s], 0.0,
                                      "bytes per RBG measured even after TakeRbgCounts ran first");
                NS_TEST_ASSERT_MSG_GT(static_cast<double>(r->w2.appBytes[s]), 0.3 * tb,
                                      "the slice's TBs carry its application bytes");
                NS_TEST_ASSERT_MSG_LT_OR_EQ(static_cast<double>(r->w2.appBytes[s]), 1.02 * tb,
                                            "goodput cannot exceed the TB bytes the MAC gave the slice");
            }
        }
        const double rbgRatio = static_cast<double>(b.w2.rbg[1]) / a.w2.rbg[1];
        const double goodRatio = static_cast<double>(b.w2.appBytes[1]) / a.w2.appBytes[1];
        NS_TEST_ASSERT_MSG_GT(rbgRatio, 1.3, "URLLC RBGs rise with its quota");
        NS_TEST_ASSERT_MSG_EQ_TOL(goodRatio, rbgRatio, 0.10 * rbgRatio, "URLLC goodput follows its RBGs within 10 %");
        NS_TEST_ASSERT_MSG_LT(static_cast<double>(b.w2.appBytes[0]), 0.75 * a.w2.appBytes[0],
                              "eMBB goodput falls when its quota halves");
    }
};

/**
 * Failure path: invalid quota vectors are refused and leave the quotas in force untouched. ValidateQuotas implements
 * the TS 28.541 RRMPolicyRatio constraint (shares in [0, 1], dedicated ratios summing to at most 100 %). A NaN share
 * used to pass (NaN < 0 and NaN > 1 are both false) and reach floor(NaN x RBGs) cast to an unsigned integer. On the
 * live cell, three invalid vectors are offered at 2.0 s; the RBG shares of the second window must equal those of the
 * first within 0.03, and the cell keeps delivering.
 */
class InvalidQuotasRefusedTest : public TestCase
{
  public:
    InvalidQuotasRefusedTest()
        : TestCase("Invalid quotas (NaN, inf, negative, > 1, sum > 1) refused mid-run, quotas in force kept")
    {
    }

  private:
    void DoRun() override
    {
        const double nan = std::numeric_limits<double>::quiet_NaN();
        const double inf = std::numeric_limits<double>::infinity();
        std::string why;
        NS_TEST_ASSERT_MSG_EQ(NtnSliceQuotaScheduler::ValidateQuotas({0.5, nan, 0.1}, &why), false, "NaN refused");
        NS_TEST_ASSERT_MSG_EQ(why.empty(), false, "a reason is given");
        NS_TEST_ASSERT_MSG_EQ(NtnSliceQuotaScheduler::ValidateQuotas({inf, 0.0}), false, "inf refused");
        NS_TEST_ASSERT_MSG_EQ(NtnSliceQuotaScheduler::ValidateQuotas({-0.1, 0.5}), false, "negative refused");
        NS_TEST_ASSERT_MSG_EQ(NtnSliceQuotaScheduler::ValidateQuotas({1.2, -0.3}), false, "a share > 1 refused");
        NS_TEST_ASSERT_MSG_EQ(NtnSliceQuotaScheduler::ValidateQuotas({0.6, 0.5}), false, "sum > 1 refused");
        NS_TEST_ASSERT_MSG_EQ(NtnSliceQuotaScheduler::ValidateQuotas({0.5, 0.3, 0.2}), true, "sum = 1 accepted");
        NS_TEST_ASSERT_MSG_EQ(NtnSliceQuotaScheduler::ValidateQuotas({}), true, "empty (no slicing) accepted");

        auto refused = std::make_shared<std::vector<bool>>();
        RigOptions o;
        o.quotas = {0.5, 0.4, 0.1};
        o.afterBuild = [refused, nan](NodeContainer&, NtnRealStackHelper&, Ptr<NtnSliceQuotaScheduler> s) {
            Simulator::Schedule(Seconds(2.0) + MicroSeconds(1), [s, refused, nan]() {
                refused->push_back(!s->TrySetQuotas({0.1, nan, 0.1}));
                refused->push_back(!s->TrySetQuotas({0.9, 0.9, 0.0}));
                refused->push_back(!s->TrySetQuotas({1.0, -0.2, 0.2}));
            });
        };
        const auto r = RunRig(o);
        NS_TEST_ASSERT_MSG_EQ(refused->size(), 3u, "three attempts made");
        for (bool x : *refused)
        {
            NS_TEST_ASSERT_MSG_EQ(x, true, "each invalid vector refused");
        }
        NS_TEST_ASSERT_MSG_EQ(r.quotasAtEnd.size(), 3u, "quotas in force kept");
        NS_TEST_ASSERT_MSG_EQ_TOL(r.quotasAtEnd[1], 0.4, 1e-12, "URLLC quota unchanged");
        for (size_t s = 0; s < 3; ++s)
        {
            NS_TEST_ASSERT_MSG_EQ_TOL(Share(r.w2.rbg, s), Share(r.w1.rbg, s), 0.03, "shares unchanged");
        }
        NS_TEST_ASSERT_MSG_EQ(r.overflowSlots, 0u, "slot invariant");
        NS_TEST_ASSERT_MSG_GT(r.w2.appBytes[1], 0u, "URLLC still delivered");
    }
};

/**
 * Unknown slice index: a UE tagged with a slice that has no quota (slice 7 of three) is scheduled as slice 0, as the
 * header documents, and is not dropped. Slice 1 then has no UE and gets no RBG, slice 0 takes eMBB and URLLC, and the
 * URLLC UEs still receive their packets.
 */
class UnknownSliceFallsBackToSliceZeroTest : public TestCase
{
  public:
    UnknownSliceFallsBackToSliceZeroTest()
        : TestCase("A UE in a slice without a quota is scheduled as slice 0 and still served")
    {
    }

  private:
    void DoRun() override
    {
        RigOptions o;
        o.urllcSlice = 7;
        o.urllcMbps = 6.0;
        const auto r = RunRig(o);
        NS_TEST_ASSERT_MSG_EQ(r.urllcSliceOfRnti, 7u, "the tag is kept as given");
        NS_TEST_ASSERT_MSG_EQ(r.w2.rbg[1], 0u, "slice 1 has no UE and gets no RBG");
        NS_TEST_ASSERT_MSG_GT(Share(r.w2.rbg, 0), 0.85, "slice 0 schedules eMBB and the untagged URLLC UEs");
        NS_TEST_ASSERT_MSG_GT(r.w2.appBytes[1], 0u, "URLLC UEs still receive");
        NS_TEST_ASSERT_MSG_GT(static_cast<double>(r.rxPkts[1]), 0.9 * r.txPkts[1],
                              "and lose under 10 % of their packets at 6 Mbit/s each");
    }
};

/**
 * Without quotas the scheduler is plain proportional fair: with an empty quota vector every RBG is counted as slice
 * 0, and the total UDP goodput of the cell matches the parent NrMacSchedulerOfdmaPF on the same cell, seed and load
 * within 10 %. This guards the override of AssignDLRBG against losing capacity.
 */
class EmptyQuotasMatchParentPfTest : public TestCase
{
  public:
    EmptyQuotasMatchParentPfTest()
        : TestCase("Empty quotas: all RBGs in slice 0, cell goodput equal to the parent OFDMA PF scheduler")
    {
    }

  private:
    void DoRun() override
    {
        RigOptions o;
        o.quotas = {};
        o.assignSlices = false;
        const auto q = RunRig(o);
        o.scheduler = "ns3::NrMacSchedulerOfdmaPF";
        const auto pf = RunRig(o);
        NS_TEST_ASSERT_MSG_EQ(q.w2.rbg.empty(), false, "counts kept");
        NS_TEST_ASSERT_MSG_EQ_TOL(Share(q.w2.rbg, 0), 1.0, 1e-12, "all RBGs counted as slice 0");
        NS_TEST_ASSERT_MSG_GT(pf.totalAppBytes, 1000000u, "the PF cell delivers");
        NS_TEST_ASSERT_MSG_EQ_TOL(static_cast<double>(q.totalAppBytes), static_cast<double>(pf.totalAppBytes),
                                  0.10 * pf.totalAppBytes, "same goodput as the parent PF scheduler");
        NS_TEST_ASSERT_MSG_GT(static_cast<double>(q.rxPkts[2]), 0.9 * q.txPkts[2], "mMTC served without slicing");
    }
};

/**
 * A slice with quota 0 is not starved while the floors leave RBGs over. With quotas 0.6/0.3/0 and eMBB and URLLC
 * saturated, floor(0.6 N) + floor(0.3 N) < N, so the rest of each slot goes to the best PF UE with data, which the
 * light mMTC slice wins. mMTC must deliver at least 90 % of its packets and use no more than the leftover share
 * (plus 0.02 for minimum-TB rounding).
 */
class ZeroQuotaSliceServedFromLeftoversTest : public TestCase
{
  public:
    ZeroQuotaSliceServedFromLeftoversTest()
        : TestCase("A zero-quota slice is served from the rounding leftovers, not starved")
    {
    }

  private:
    void DoRun() override
    {
        RigOptions o;
        o.quotas = {0.6, 0.3, 0.0};
        const auto r = RunRig(o);
        const uint32_t n = r.rbgPerSlot;
        const double leftover =
            (n - std::floor(0.6 * n + 1e-9) - std::floor(0.3 * n + 1e-9)) / static_cast<double>(n);
        NS_TEST_ASSERT_MSG_GT(leftover, 0.0, "the floors leave RBGs over");
        NS_TEST_ASSERT_MSG_GT(r.txPkts[2], 10u, "mMTC offered packets");
        NS_TEST_ASSERT_MSG_GT_OR_EQ(static_cast<double>(r.rxPkts[2]), 0.9 * r.txPkts[2],
                                    "the zero-quota mMTC slice delivers at least 90 %");
        NS_TEST_ASSERT_MSG_LT_OR_EQ(Share(r.w2.rbg, 2), leftover + 0.02, "mMTC uses no more than the leftovers");
        NS_TEST_ASSERT_MSG_GT_OR_EQ(Share(r.w2.rbg, 1), 0.27, "URLLC keeps its quota");
    }
};

/**
 * Determinism across RngRun: the same run gives identical RBG counts and identical application bytes (packet level);
 * runs 2 and 3 move the UEs and fading but keep every saturated slice at or above its quota less one RBG of rounding,
 * and below quota plus 0.15.
 */
class SeedDeterminismAndBoundsTest : public TestCase
{
  public:
    SeedDeterminismAndBoundsTest()
        : TestCase("Same RngRun identical RBGs and goodput; runs 2 and 3 inside the quota bounds")
    {
    }

  private:
    void DoRun() override
    {
        RigOptions o;
        o.run = 2;
        const auto a = RunRig(o);
        const auto b = RunRig(o);
        o.run = 3;
        const auto c = RunRig(o);
        RngSeedManager::SetRun(1);
        NS_TEST_ASSERT_MSG_EQ((a.w2.rbg == b.w2.rbg), true, "identical RBG counts");
        NS_TEST_ASSERT_MSG_EQ((a.w2.appBytes == b.w2.appBytes), true, "identical delivered bytes");
        for (const auto* r : {&a, &c})
        {
            const double tol = 1.0 / r->rbgPerSlot;
            NS_TEST_ASSERT_MSG_GT_OR_EQ(Share(r->w2.rbg, 0), 0.6 - tol, "eMBB at least its quota");
            NS_TEST_ASSERT_MSG_GT_OR_EQ(Share(r->w2.rbg, 1), 0.3 - tol, "URLLC at least its quota");
            NS_TEST_ASSERT_MSG_LT(Share(r->w2.rbg, 1), 0.45, "URLLC bounded");
            NS_TEST_ASSERT_MSG_EQ(r->overflowSlots, 0u, "slot invariant");
        }
    }
};

/**
 * Cross-module (oran-ntn): a RIC control E2SM-RC SLICE_PRB_ALLOCATION travels down the E2 node's return feeder path
 * (OranNtnE2Node::ReceiveRcAction) to OranNtnRealStackBinding, which writes NtnSliceQuotaScheduler quotas. URLLC
 * asks for 0.9 while eMBB holds 0.6: the binding scales the vector to sum 1 (TS 28.541 RRMPolicyRatio, dedicated
 * ratios at most 100 %), so the applied quotas are 0.6/1.7, 0.9/1.7, 0.2/1.7. The new quotas must not be in force
 * before the feeder delay and must be after it; URLLC's measured RBG share and goodput rise. Invalid controls (NaN,
 * negative share, gNB 0, a gNB that does not exist) are refused and not counted.
 */
class OranSliceActuatorDrivesQuotasTest : public TestCase
{
  public:
    OranSliceActuatorDrivesQuotasTest()
        : TestCase("Cross-module: E2SM-RC slice PRB control over the E2 feeder sets the live quotas")
    {
    }

  private:
    void DoRun() override
    {
        struct Probe
        {
            Ptr<OranNtnHelper> helper;
            Ptr<OranNtnRealStackBinding> binding;
            std::vector<Ptr<OranNtnE2Node>> nodes;
            std::vector<bool> invalidAccepted;
            uint64_t updatesAfterInvalid{99};
            std::vector<double> beforeDelay;
            std::vector<double> afterDelay;
            Time feeder;
        };
        auto p = std::make_shared<Probe>();
        RigOptions o;
        o.quotas = {0.6, 0.2, 0.2};
        o.afterBuild = [p](NodeContainer& sat, NtnRealStackHelper& rs, Ptr<NtnSliceQuotaScheduler> s) {
            p->helper = CreateObject<OranNtnHelper>();
            auto ric = p->helper->CreateNearRtRic();
            p->nodes = p->helper->CreateSatelliteE2Nodes(sat, ric);
            p->binding = CreateObject<OranNtnRealStackBinding>();
            p->binding->Bind(p->helper, &rs);
            for (uint32_t u = 0; u < kUes; ++u)
            {
                p->binding->SetUeSlice(u, static_cast<uint8_t>(kSliceOfProfile[u % 3]));
            }
            p->feeder = p->nodes.at(0)->GetFeederLinkDelay();
            auto make = [](uint32_t gnb, uint8_t slice, double share) {
                E2RcAction a;
                a.actionType = E2RcActionType::SLICE_PRB_ALLOCATION;
                a.targetGnbId = gnb;
                a.targetSliceId = slice;
                a.parameter1 = share;
                a.xappName = "test-slice";
                return a;
            };
            Simulator::Schedule(Seconds(1.5), [p, make]() {
                const double nan = std::numeric_limits<double>::quiet_NaN();
                for (const auto& a : {make(1, 1, nan), make(1, 1, -0.2), make(0, 1, 0.5), make(2, 1, 0.5)})
                {
                    p->invalidAccepted.push_back(p->helper->ActuateRcAction(a));
                }
                p->updatesAfterInvalid = p->binding->GetQuotaUpdates();
            });
            Simulator::Schedule(Seconds(2.0), [p, make]() { p->nodes.at(0)->ReceiveRcAction(make(1, 1, 0.9)); });
            Simulator::Schedule(Seconds(2.0) + p->feeder / 2, [p, s]() { p->beforeDelay = s->GetQuotas(); });
            Simulator::Schedule(Seconds(2.0) + p->feeder + MicroSeconds(10),
                                [p, s]() { p->afterDelay = s->GetQuotas(); });
        };
        const auto r = RunRig(o);
        NS_TEST_ASSERT_MSG_EQ(p->binding->GetRegisteredActuators().empty(), false, "actuators registered");
        NS_TEST_ASSERT_MSG_EQ(p->invalidAccepted.size(), 4u, "four invalid controls offered");
        for (bool x : p->invalidAccepted)
        {
            NS_TEST_ASSERT_MSG_EQ(x, false, "invalid control refused");
        }
        NS_TEST_ASSERT_MSG_EQ(p->updatesAfterInvalid, 0u, "refused controls are not counted");
        NS_TEST_ASSERT_MSG_GT(p->feeder.GetSeconds(), 0.0, "the E2 node has a feeder delay");
        NS_TEST_ASSERT_MSG_EQ(p->beforeDelay.size(), 3u, "quotas present");
        NS_TEST_ASSERT_MSG_EQ_TOL(p->beforeDelay[1], 0.2, 1e-12, "not applied before the feeder delay");
        NS_TEST_ASSERT_MSG_EQ(p->afterDelay.size(), 3u, "three quotas applied");
        NS_TEST_ASSERT_MSG_EQ_TOL(p->afterDelay[0], 0.6 / 1.7, 1e-9, "eMBB scaled");
        NS_TEST_ASSERT_MSG_EQ_TOL(p->afterDelay[1], 0.9 / 1.7, 1e-9, "URLLC scaled");
        NS_TEST_ASSERT_MSG_EQ_TOL(p->afterDelay[2], 0.2 / 1.7, 1e-9, "mMTC scaled");
        NS_TEST_ASSERT_MSG_EQ_TOL(std::accumulate(p->afterDelay.begin(), p->afterDelay.end(), 0.0), 1.0, 1e-9,
                                  "TS 28.541: dedicated ratios sum to at most 100 %");
        NS_TEST_ASSERT_MSG_EQ(p->binding->GetQuotaUpdates(), 1u, "one control actuated");
        NS_TEST_ASSERT_MSG_GT(Share(r.w2.rbg, 1), Share(r.w1.rbg, 1) + 0.08, "URLLC's RBG share rises");
        NS_TEST_ASSERT_MSG_GT(r.w2.appBytes[1] / 0.9, 1.2 * r.w1.appBytes[1], "URLLC goodput per second rises");
        NS_TEST_ASSERT_MSG_EQ(r.overflowSlots, 0u, "slot invariant");
        p->binding->Dispose();
    }
};

class NtnSliceQuotaSchedulerTestSuite : public TestSuite
{
  public:
    NtnSliceQuotaSchedulerTestSuite()
        : TestSuite("ntn-slice-quota-scheduler", Type::SYSTEM)
    {
        AddTestCase(new QuotaRespectedUnderSaturationTest, TestCase::Duration::QUICK);
        AddTestCase(new WorkConservationTest, TestCase::Duration::QUICK);
        AddTestCase(new ContractTrimConvergenceTest, TestCase::Duration::QUICK);
        AddTestCase(new SlotGridAndPerSlotGuaranteeTest, TestCase::Duration::QUICK);
        AddTestCase(new GoodputTracksQuotaTest, TestCase::Duration::QUICK);
        AddTestCase(new InvalidQuotasRefusedTest, TestCase::Duration::QUICK);
        AddTestCase(new UnknownSliceFallsBackToSliceZeroTest, TestCase::Duration::QUICK);
        AddTestCase(new EmptyQuotasMatchParentPfTest, TestCase::Duration::QUICK);
        AddTestCase(new ZeroQuotaSliceServedFromLeftoversTest, TestCase::Duration::QUICK);
        AddTestCase(new SeedDeterminismAndBoundsTest, TestCase::Duration::QUICK);
        AddTestCase(new OranSliceActuatorDrivesQuotasTest, TestCase::Duration::QUICK);
    }
};

static NtnSliceQuotaSchedulerTestSuite g_ntnSliceQuotaSchedulerTestSuite;

} // namespace
