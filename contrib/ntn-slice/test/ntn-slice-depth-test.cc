/*
 * SPDX-License-Identifier: GPL-2.0-only
 * Copyright (c) 2026 Muhammad Uzair (ns3-ntn-toolkit)
 *
 * In-depth cases (Nov 2026 test campaign): S-NSSAI and profile conformance,
 * selector precedence, orchestrator priority / work conservation / PRB
 * accounting as properties over a seeded demand sweep, isolation-monitor
 * percentile and loss semantics, and the slice-quota scheduler on a live NR
 * cell (per-slot invariants, non-work-conserving mode, a quota change during
 * the run, determinism, and the orchestrator closing the loop on measured
 * per-slice delivery).
 */
#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/nr-module.h"
#include "ns3/ntn-oran-application.h"
#include "ns3/ntn-real-stack-helper.h"
#include "ns3/ntn-slice-helper.h"
#include "ns3/ntn-slice-quota-scheduler.h"
#include "ns3/ntn-slice-selector.h"
#include "ns3/ntn-slice-types.h"
#include "ns3/ntn-tr38811-mobility-model.h"
#include "ns3/sgp4-mobility-model.h"
#include "ns3/slice-isolation-monitor.h"
#include "ns3/slice-orchestrator-xapp.h"
#include "ns3/test.h"
#include "ns3/walker-constellation.h"

#include <cmath>
#include <numeric>

using namespace ns3;
using namespace ns3::ntnslice;

namespace
{

/// TS 23.501 section 5.15.2.1: S-NSSAI = SST (8 bits) + optional SD (24 bits),
/// standardized SST values in Table 5.15.2.2-1 (1 eMBB, 2 URLLC, 3 MIoT, 4 V2X).
/// The default profiles carry the TS 22.261 reference KPIs the header cites.
class SnssaiAndProfileConformanceTest : public TestCase
{
  public:
    SnssaiAndProfileConformanceTest()
        : TestCase("TS 23.501 5.15.2: SST/SD widths and values; TS 22.261 default profiles")
    {
    }

  private:
    void DoRun() override
    {
        NS_TEST_ASSERT_MSG_EQ(+static_cast<uint8_t>(SliceSst::Embb), 1, "SST 1 eMBB");
        NS_TEST_ASSERT_MSG_EQ(+static_cast<uint8_t>(SliceSst::Urllc), 2, "SST 2 URLLC");
        NS_TEST_ASSERT_MSG_EQ(+static_cast<uint8_t>(SliceSst::Mmtc), 3, "SST 3 MIoT");
        NS_TEST_ASSERT_MSG_EQ(+static_cast<uint8_t>(SliceSst::V2x), 4, "SST 4 V2X");
        const Snssai a{SliceSst::Urllc, 0x00ABCDEF};
        NS_TEST_ASSERT_MSG_EQ(PackSnssai(a), 0x02ABCDEFu, "SST in the top byte, SD below");
        NS_TEST_ASSERT_MSG_EQ((UnpackSnssai(PackSnssai(a)) == a), true, "round trip");
        const Snssai wide{SliceSst::Embb, 0x12345678};
        NS_TEST_ASSERT_MSG_EQ(UnpackSnssai(PackSnssai(wide)).sd, 0x345678u,
                              "SD is 24 bits: the excess byte is dropped, not carried into SST");
        NS_TEST_ASSERT_MSG_EQ((UnpackSnssai(PackSnssai(wide)).sst == SliceSst::Embb), true,
                              "and SST is intact");
        NS_TEST_ASSERT_MSG_EQ(UnpackSnssai(PackSnssai({SliceSst::Mmtc, kSdUnset})).sd, kSdUnset,
                              "0xFFFFFF means no SD");
        NS_TEST_ASSERT_MSG_EQ((Snssai{SliceSst::Embb, 5} < Snssai{SliceSst::Urllc, 0}), true,
                              "ordered by SST first");
        NS_TEST_ASSERT_MSG_EQ(std::string(SstName(SliceSst::Urllc)), "urllc", "name");

        const auto u = DefaultUrllc();
        const auto e = DefaultEmbb();
        const auto m = DefaultMmtc();
        NS_TEST_ASSERT_MSG_EQ_TOL(u.reliability, 0.99999, 1e-12,
                                  "URLLC 99.999 % (TS 22.261 Table 7.2.2-1)");
        NS_TEST_ASSERT_MSG_EQ_TOL(u.latencyBudgetMs, 5.0, 1e-12, "URLLC 5 ms end to end");
        NS_TEST_ASSERT_MSG_EQ(u.allowGeo, false, "URLLC skips GEO (about 240 ms one way)");
        NS_TEST_ASSERT_MSG_GT(u.priority, e.priority, "URLLC outranks eMBB");
        NS_TEST_ASSERT_MSG_GT(e.priority, m.priority, "eMBB outranks mMTC");
        NS_TEST_ASSERT_MSG_GT(m.latencyBudgetMs, e.latencyBudgetMs, "mMTC is delay tolerant");
        NS_TEST_ASSERT_MSG_EQ(ShouldSkipGeo(u), true, "GEO skip for URLLC");
        NS_TEST_ASSERT_MSG_EQ(ShouldSkipGeo(e), false, "not for eMBB");
        NS_TEST_ASSERT_MSG_EQ(IsGeoSatellite(35786.0), true, "35786 km is GEO");
        NS_TEST_ASSERT_MSG_EQ(IsGeoSatellite(8062.0), false, "MEO-8000 is not");
        // The slice cell's one-way floor: 600 km LEO, live feeder: 2 x 2.0014 ms.
        NS_TEST_ASSERT_MSG_EQ_TOL(NtnSliceHelper::OneWayFloorMs(600.0, true, 5.0),
                                  2.0 * 600e3 / 299792458.0 * 1e3, 1e-9, "two 600 km legs");
        NS_TEST_ASSERT_MSG_EQ_TOL(NtnSliceHelper::OneWayFloorMs(600.0, false, 5.0),
                                  600e3 / 299792458.0 * 1e3 + 5.0, 1e-9, "one leg + backhaul");
    }
};

/// Selector precedence and boundaries. First match wins whatever the rule type;
/// port ranges are inclusive; an inverted range never matches; an empty app
/// label matches nothing; unmatched flows take the default. The three-slice
/// default maps DSCP EF (46, RFC 4594 / RFC 3246) to URLLC and CoAP (UDP
/// 5683, DTLS 5684, RFC 7252) to mMTC.
class SelectorPrecedenceTest : public TestCase
{
  public:
    SelectorPrecedenceTest()
        : TestCase("Selector: first match wins, inclusive ranges, inverted range, RFC 4594/7252 maps")
    {
    }

  private:
    void DoRun() override
    {
        auto stack = NtnSliceHelper::ThreeSliceDefault();
        auto& sel = *stack.selector;
        NS_TEST_ASSERT_MSG_EQ((sel.Match({0, 5000, 46, ""}).sst == SliceSst::Urllc), true, "EF");
        NS_TEST_ASSERT_MSG_EQ((sel.Match({0, 5683, 0, ""}).sst == SliceSst::Mmtc), true, "CoAP");
        NS_TEST_ASSERT_MSG_EQ((sel.Match({0, 5684, 0, ""}).sst == SliceSst::Mmtc), true, "CoAPS");
        NS_TEST_ASSERT_MSG_EQ((sel.Match({0, 5685, 0, ""}).sst == SliceSst::Embb), true,
                              "5685 is outside the CoAP range: default");
        NS_TEST_ASSERT_MSG_EQ((sel.Match({0, 5683, 46, ""}).sst == SliceSst::Urllc), true,
                              "DSCP rule was added first, so it wins over the port rule");
        NS_TEST_ASSERT_MSG_EQ((sel.Match({0, 80, 0, "mmtc"}).sst == SliceSst::Mmtc), true, "label");

        NtnSliceSelector s;
        s.SetDefault({SliceSst::Mmtc, 7});
        s.AddPortRangeRule(9000, 8000, {SliceSst::Urllc, 1}); // inverted
        s.AddAppLabelRule("", {SliceSst::Urllc, 2});          // empty label
        NS_TEST_ASSERT_MSG_EQ((s.Match({0, 8500, 0, ""}) == Snssai{SliceSst::Mmtc, 7}), true,
                              "an inverted range and an empty label match nothing");
        s.AddPortRangeRule(0, 65535, {SliceSst::Embb, 3});
        NS_TEST_ASSERT_MSG_EQ((s.Match({0, 0, 0, ""}) == Snssai{SliceSst::Embb, 3}), true, "0");
        NS_TEST_ASSERT_MSG_EQ((s.Match({0, 65535, 0, ""}) == Snssai{SliceSst::Embb, 3}), true,
                              "65535 inclusive");
        NS_TEST_ASSERT_MSG_EQ(s.RuleCount(), 3u, "three rules");
    }
};

/// Priority under shortage: the minimums do not all fit. The priority-9 URLLC
/// slice registered LAST must still get its minimum, ahead of eMBB.
class OrchestratorPriorityReservationTest : public TestCase
{
  public:
    OrchestratorPriorityReservationTest()
        : TestCase("Orchestrator: minimums reserved by priority, not registration order")
    {
    }

  private:
    void DoRun() override
    {
        SliceOrchestratorXapp o;
        auto e = DefaultEmbb();
        e.minThroughputMbps = 30.0;
        auto u = DefaultUrllc();
        u.minThroughputMbps = 20.0;
        o.RegisterSlice(e);
        o.RegisterSlice(u);
        o.SetTotalPrb(40);          // 24 Mbps cell
        o.SetPrbCapacityMbps(0.6);
        o.SetDemand(e.snssai, 100.0);
        o.SetDemand(u.snssai, 20.0);
        const auto t = o.Step();
        NS_TEST_ASSERT_MSG_EQ_TOL(t[1].servedMbps, 20.0, 1e-9, "URLLC keeps its 20 Mbps minimum");
        NS_TEST_ASSERT_MSG_EQ(t[1].minThroughputMet, true, "URLLC guarantee met");
        NS_TEST_ASSERT_MSG_EQ_TOL(t[0].servedMbps, 4.0, 1e-9, "eMBB gets the rest");
        NS_TEST_ASSERT_MSG_EQ(t[0].minThroughputMet, false, "eMBB's 30 Mbps cannot be met");
        NS_TEST_ASSERT_MSG_EQ(t[0].prbAllocated + t[1].prbAllocated, 40u, "all 40 PRBs");
    }
};

/// Properties over 500 seeded random ticks (3 slices, random demands, caps,
/// minimums, capacity, with and without external shares):
///  - served <= demand and <= the slice cap;
///  - total served = min(capacity, sum of usable demand) (work conservation);
///  - sum of PRBs <= cell PRBs;
///  - the guarantee is min(minimum, demand, cap): a slice asking for less than its
///    minimum and fully served counts as met;
///  - when capacity covers all minimums, every minimum (or demand) is met.
class OrchestratorPropertySweepTest : public TestCase
{
  public:
    OrchestratorPropertySweepTest()
        : TestCase("Orchestrator properties over 500 random ticks: caps, work conservation, PRB sum")
    {
    }

  private:
    void DoRun() override
    {
        Ptr<UniformRandomVariable> rv = CreateObject<UniformRandomVariable>();
        rv->SetStream(77);
        uint32_t bad = 0;
        std::string why;
        auto flag = [&](const std::string& w) {
            if (bad++ == 0)
            {
                why = w;
            }
        };
        for (int k = 0; k < 500; ++k)
        {
            SliceOrchestratorXapp o;
            std::vector<SliceProfile> p = {DefaultEmbb(), DefaultUrllc(), DefaultMmtc()};
            for (auto& s : p)
            {
                s.minThroughputMbps = rv->GetValue(0.0, 30.0);
                s.maxThroughputMbps = rv->GetValue() < 0.3 ? 0.0 : rv->GetValue(5.0, 150.0);
                o.RegisterSlice(s);
            }
            const uint32_t prb = static_cast<uint32_t>(rv->GetInteger(10, 273));
            o.SetTotalPrb(prb);
            o.SetPrbCapacityMbps(0.6);
            double usable = 0.0;
            for (auto& s : p)
            {
                const double d = rv->GetValue(0.0, 120.0);
                o.SetDemand(s.snssai, d);
                usable += s.maxThroughputMbps > 0 ? std::min(d, s.maxThroughputMbps) : d;
            }
            std::vector<SliceTick> t;
            if (k % 2 == 0)
            {
                t = o.Step();
            }
            else
            {
                t = o.StepWithShares({{p[0].snssai, rv->GetValue()},
                                      {p[1].snssai, rv->GetValue()},
                                      {p[2].snssai, rv->GetValue()}});
            }
            double served = 0.0;
            uint32_t prbSum = 0;
            double minSum = 0.0;
            for (std::size_t i = 0; i < 3; ++i)
            {
                served += t[i].servedMbps;
                prbSum += t[i].prbAllocated;
                const double usableI = p[i].maxThroughputMbps > 0
                                           ? std::min(t[i].demandMbps, p[i].maxThroughputMbps)
                                           : t[i].demandMbps;
                minSum += std::min(p[i].minThroughputMbps, usableI);
                if (t[i].servedMbps > t[i].demandMbps + 1e-9)
                {
                    flag("served > demand");
                }
                if (p[i].maxThroughputMbps > 0 && t[i].servedMbps > p[i].maxThroughputMbps + 1e-9)
                {
                    flag("served > cap");
                }
            }
            const double cap = prb * 0.6;
            if (prbSum > prb)
            {
                flag("PRB sum exceeds the cell");
            }
            if (k % 2 == 0 && std::abs(served - std::min(cap, usable)) > 1e-6)
            {
                flag("not work conserving at tick " + std::to_string(k));
            }
            if (minSum <= cap)
            {
                for (std::size_t i = 0; i < 3; ++i)
                {
                    if (!t[i].minThroughputMet)
                    {
                        flag("guarantee missed although capacity covers all minimums");
                    }
                }
            }
        }
        NS_TEST_ASSERT_MSG_EQ(bad, 0u, "all properties hold; first violation: " << why);
    }
};

/// Isolation monitor semantics. A lost packet is not a latency sample (the
/// examples record losses with the flow's mean delay, after the deliveries);
/// non-finite latencies are ignored; the default window is cumulative, so the
/// p99 does not depend on recording order; the p99 of 1..1000 ms is 990 ms
/// (index floor(0.99 (n - 1)) of the sorted samples); reliability at exactly
/// the target is not a breach; unregistered slices are ignored; the Breach
/// trace fires only for breaching slices.
class MonitorPercentileAndLossTest : public TestCase
{
  public:
    MonitorPercentileAndLossTest()
        : TestCase("Monitor: losses are not latency samples, order-free p99, reliability edge")
    {
    }

  private:
    uint32_t m_breaches{0};

    void OnBreach(const BreachEvent&) { ++m_breaches; }

    void DoRun() override
    {
        auto mp = CreateObject<SliceIsolationMonitor>();
        SliceIsolationMonitor& m = *mp;
        auto e = DefaultEmbb();
        e.latencyBudgetMs = 995.0;
        e.reliability = 0.8;
        m.RegisterSlice(e);
        auto u = DefaultUrllc();
        m.RegisterSlice(u);
        m.TraceConnectWithoutContext("Breach", MakeCallback(&MonitorPercentileAndLossTest::OnBreach, this));
        // Descending order: a tail-keeping window would see only the fast ones.
        for (int ms = 1000; ms >= 1; --ms)
        {
            m.RecordPacket(e.snssai, ms, true);
        }
        for (int i = 0; i < 250; ++i)
        {
            m.RecordPacket(e.snssai, 1.0, false); // lost, recorded with a fake latency
        }
        m.RecordPacket(e.snssai, std::nan(""), true);
        m.RecordPacket({SliceSst::V2x, 1}, 5000.0, false); // unregistered
        auto ev = m.EvaluateAll();
        const auto& eb = ev[0].snssai == e.snssai ? ev[0] : ev[1];
        NS_TEST_ASSERT_MSG_EQ(eb.windowSamples, 1000u, "1000 finite deliveries; losses excluded");
        NS_TEST_ASSERT_MSG_EQ_TOL(eb.observedLatencyP99Ms, 990.0, 1e-9, "p99 of 1..1000 ms");
        NS_TEST_ASSERT_MSG_EQ(eb.latencyBreach, false, "990 ms < 995 ms budget");
        NS_TEST_ASSERT_MSG_EQ_TOL(eb.observedLossRate, 250.0 / 1251.0, 1e-12, "loss over all");
        NS_TEST_ASSERT_MSG_EQ(eb.reliabilityBreach, false, "0.8001 >= 0.8");
        NS_TEST_ASSERT_MSG_EQ(m_breaches, 0u, "no slice breached: no trace");

        // URLLC: 200 deliveries at 6.8 ms (the LEO-550 + 5 ms floor) breach 5 ms.
        for (int i = 0; i < 200; ++i)
        {
            m.RecordPacket(u.snssai, 6.8, true);
        }
        m.EvaluateAll();
        NS_TEST_ASSERT_MSG_EQ(m.BreachCount(), 1u, "the URLLC slice breaches structurally");
        NS_TEST_ASSERT_MSG_EQ(m_breaches, 1u, "and the trace fired once");

        // A sliding window, fed in time order, follows the recent samples.
        auto wp = CreateObject<SliceIsolationMonitor>();
        SliceIsolationMonitor& w = *wp;
        w.SetWindow(100);
        w.RegisterSlice(u);
        for (int i = 0; i < 300; ++i)
        {
            w.RecordPacket(u.snssai, i < 200 ? 20.0 : 1.0, true);
        }
        NS_TEST_ASSERT_MSG_EQ_TOL(w.EvaluateAll()[0].observedLatencyP99Ms, 1.0, 1e-12,
                                  "a 100-sample window forgets the old 20 ms samples");
    }
};

// ---------------------------------------------------------------------------
// Live NR cell under NtnSliceQuotaScheduler (same rig as the quota suite: SGP4
// satellite at 550 km, six TR 38.811 UEs, MixedBouquet, eMBB saturating).
// ---------------------------------------------------------------------------

constexpr uint32_t kSliceOfProfile[3] = {2, 0, 1};

struct CellRun
{
    std::vector<uint64_t> rbgFirst;  //!< 1.0..2.0 s
    std::vector<uint64_t> rbgSecond; //!< 2.0..2.9 s
    uint64_t slots{0};
    uint64_t slotOverflow{0};  //!< slots where RBGs given > RBGs in the slot
    uint64_t slotsUnderUsed{0};
    uint32_t rbgPerSlot{0};
    uint64_t rxUrllc{0};
    uint64_t rxEmbb{0};
    std::vector<double> quotasApplied;
};

struct CellOptions
{
    std::vector<double> quotas{0.5, 0.3, 0.2};
    bool workConserving{true};
    double urllcMbps{6.0};
    std::vector<double> quotasAt2s{};       //!< quota change at 2.0 s, if any
    bool orchestratorLoop{false};           //!< orchestrator sets quotas from measured demand
    uint32_t run{1};
};

CellRun
RunCell(const CellOptions& opt)
{
    RngSeedManager::SetSeed(1);
    RngSeedManager::SetRun(opt.run);
    RngSeedManager::ResetNextStreamIndex();
    Config::SetDefault("ns3::NrHelper::UseIdealRrc", BooleanValue(true));
    NodeContainer sat;
    sat.Create(1);
    NodeContainer ues;
    ues.Create(6);
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
    rs.SetSchedulerTypeName("ns3::ntnslice::NtnSliceQuotaScheduler");
    rs.Build(sat, ues);
    rs.InstallTraffic(NtnRealStackHelper::TrafficProfile::MixedBouquet, Seconds(0.5), Seconds(2.9));
    Ptr<Node> rh = rs.GetRemoteHost();
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
            app->SetAttribute("DataRate", DataRateValue(DataRate("60Mb/s")));
        }
        else if (k % 3 == 2)
        {
            app->SetAttribute("PacketSize", UintegerValue(1000));
            app->SetAttribute("Period",
                              TimeValue(opt.urllcMbps > 0 ? Seconds(8000.0 / (opt.urllcMbps * 1e6))
                                                          : Seconds(100)));
        }
        ++k;
    }
    auto gnb = DynamicCast<NrGnbNetDevice>(rs.GetEnbDevices().Get(0));
    Ptr<NtnSliceQuotaScheduler> sched = DynamicCast<NtnSliceQuotaScheduler>(gnb->GetScheduler(0));
    NS_ABORT_MSG_IF(!sched, "the gNB does not run NtnSliceQuotaScheduler");
    sched->SetAttribute("WorkConserving", BooleanValue(opt.workConserving));
    sched->SetQuotas(opt.quotas);

    CellRun out;
    // Per-slot invariant: the scheduler's own trace, slice by slice.
    struct SlotAcc
    {
        uint32_t given{0};
        uint32_t total{0};
        uint32_t seen{0};
    };
    auto acc = std::make_shared<SlotAcc>();
    sched->TraceConnectWithoutContext(
        "SliceAllocation",
        MakeBoundCallback(
            +[](SlotAcc* a, CellRun* o, uint32_t slice, uint32_t given, uint32_t total) {
                a->given += given;
                a->total = total;
                o->rbgPerSlot = total;
                if (++a->seen == 3)
                {
                    o->slotOverflow += (a->given > a->total) ? 1 : 0;
                    o->slotsUnderUsed += (a->given < a->total) ? 1 : 0;
                    a->given = 0;
                    a->seen = 0;
                }
            },
            acc.get(),
            &out));

    Simulator::Schedule(Seconds(0.4), [&rs, sched]() {
        for (uint32_t u = 0; u < 6; ++u)
        {
            sched->SetSliceOfRnti(rs.GetUeRnti(u), kSliceOfProfile[u % 3]);
        }
    });
    Simulator::Schedule(Seconds(1.0), [sched]() {
        sched->TakeRbgCounts(nullptr);
        sched->TakeBytesPerRbg();
        sched->TakeBacklog();
    });
    Simulator::Schedule(Seconds(2.0), [sched, &out, &opt]() {
        out.rbgFirst = sched->TakeRbgCounts(&out.slots);
        const auto backlog = sched->TakeBacklog();
        if (!opt.quotasAt2s.empty())
        {
            sched->SetQuotas(opt.quotasAt2s);
        }
        else if (opt.orchestratorLoop)
        {
            // The orchestrator turns measured per-slice backlog into shares.
            SliceOrchestratorXapp o;
            const std::vector<SliceProfile> prof = {DefaultEmbb(), DefaultUrllc(), DefaultMmtc()};
            for (const auto& p : prof)
            {
                o.RegisterSlice(p);
            }
            o.SetTotalPrb(100);
            o.SetPrbCapacityMbps(1.0);
            for (size_t s = 0; s < 3 && s < backlog.size(); ++s)
            {
                o.SetDemand(prof[s].snssai, backlog[s] * 8.0 / 1e6 * 1000.0);
            }
            const auto t = o.Step();
            std::vector<double> q(3, 0.0);
            double sum = 0.0;
            for (size_t s = 0; s < 3; ++s)
            {
                q[s] = t[s].prbAllocated / 100.0;
                sum += q[s];
            }
            if (sum > 1.0)
            {
                for (auto& x : q)
                {
                    x /= sum;
                }
            }
            sched->SetQuotas(q);
        }
        out.quotasApplied = sched->GetQuotas();
    });
    Simulator::Schedule(Seconds(2.9), [sched, &out]() {
        out.rbgSecond = sched->TakeRbgCounts(nullptr);
    });
    Simulator::Stop(Seconds(3));
    Simulator::Run();
    for (uint32_t u = 0; u < 6; ++u)
    {
        if (u % 3 == 2)
        {
            out.rxUrllc += rs.GetUeRxPackets(u);
        }
        else if (u % 3 == 1)
        {
            out.rxEmbb += rs.GetUeRxPackets(u);
        }
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

/// Per-slot invariant on the live cell, plus a quota change mid-run: in no slot
/// does the scheduler hand out more RBGs than the slot has; with eMBB
/// saturating, the work-conserving scheduler leaves no RBG idle in most slots;
/// and moving URLLC's quota from 0.3 to 0.1 at 2 s moves its measured share
/// down while URLLC still has demand above 0.1 of the cell (6 Mbit/s per UE).
class QuotaSlotInvariantAndRetuneTest : public TestCase
{
  public:
    QuotaSlotInvariantAndRetuneTest()
        : TestCase("Live NR cell: per-slot RBG invariant, no idle RBG, quota retune takes effect")
    {
    }

  private:
    void DoRun() override
    {
        CellOptions opt;
        opt.urllcMbps = 15.0;
        opt.quotas = {0.4, 0.4, 0.2};
        opt.quotasAt2s = {0.7, 0.1, 0.2};
        const auto r = RunCell(opt);
        NS_TEST_ASSERT_MSG_GT(r.slots, 1500u, "about 2000 slots in 1 s at mu=1");
        NS_TEST_ASSERT_MSG_EQ(r.slotOverflow, 0u, "never more RBGs than the slot has");
        const double before = Share(r.rbgFirst, 1);
        const double after = Share(r.rbgSecond, 1);
        NS_TEST_ASSERT_MSG_GT(before, 0.3, "URLLC near its 0.4 quota before");
        NS_TEST_ASSERT_MSG_LT(after, before - 0.1, "and well below it after the retune");
        NS_TEST_ASSERT_MSG_GT(Share(r.rbgSecond, 0), Share(r.rbgFirst, 0),
                              "the freed RBGs go to eMBB");
        NS_TEST_ASSERT_MSG_GT(r.rxUrllc, 100u, "URLLC packets delivered");
    }
};

/// Work conservation switch (failure path for isolation): with URLLC idle, a
/// work-conserving scheduler lends its quota to eMBB, while the strict one
/// leaves URLLC's RBGs unused, so eMBB's share is capped near its own quota.
class QuotaWorkConservingSwitchTest : public TestCase
{
  public:
    QuotaWorkConservingSwitchTest()
        : TestCase("Live NR cell: WorkConserving=false caps eMBB at its quota when URLLC is idle")
    {
    }

  private:
    void DoRun() override
    {
        CellOptions opt;
        opt.urllcMbps = 0.0;
        opt.quotas = {0.4, 0.4, 0.2};
        const auto wc = RunCell(opt);
        opt.workConserving = false;
        const auto strict = RunCell(opt);
        const double wcEmbb = std::accumulate(wc.rbgFirst.begin(), wc.rbgFirst.end(), 0.0) > 0
                                  ? static_cast<double>(wc.rbgFirst[0])
                                  : 0.0;
        NS_TEST_ASSERT_MSG_GT(wcEmbb, 0.0, "eMBB scheduled");
        NS_TEST_ASSERT_MSG_GT(static_cast<double>(wc.rbgFirst[0]),
                              1.5 * static_cast<double>(strict.rbgFirst[0]),
                              "work conserving: eMBB takes the idle URLLC quota");
        NS_TEST_ASSERT_MSG_GT(strict.slotsUnderUsed, strict.slots / 2,
                              "strict: RBGs stay idle in most slots");
        NS_TEST_ASSERT_MSG_LT(strict.rbgFirst[0],
                              strict.slots * static_cast<uint64_t>(std::floor(0.4 * strict.rbgPerSlot + 1e-9)) + 1,
                              "strict: eMBB never exceeds floor(0.4 x RBGs) in any slot");
        NS_TEST_ASSERT_MSG_EQ(strict.slotOverflow, 0u, "invariant holds in both modes");
    }
};

/// Determinism of the live cell: the same RngRun gives identical RBG counts;
/// another run stays within 10 % on the saturating slice's share.
class QuotaDeterminismTest : public TestCase
{
  public:
    QuotaDeterminismTest()
        : TestCase("Live NR cell: same RngRun identical RBG counts, another run within bounds")
    {
    }

  private:
    void DoRun() override
    {
        CellOptions opt;
        const auto a = RunCell(opt);
        const auto b = RunCell(opt);
        opt.run = 5;
        const auto c = RunCell(opt);
        RngSeedManager::SetRun(1);
        NS_TEST_ASSERT_MSG_EQ((a.rbgFirst == b.rbgFirst), true, "identical first window");
        NS_TEST_ASSERT_MSG_EQ((a.rbgSecond == b.rbgSecond), true, "identical second window");
        NS_TEST_ASSERT_MSG_EQ(a.rxUrllc, b.rxUrllc, "identical URLLC deliveries");
        NS_TEST_ASSERT_MSG_EQ_TOL(Share(c.rbgFirst, 0), Share(a.rbgFirst, 0), 0.1,
                                  "another seed: eMBB share within 0.1");
    }
};

/// Cross-module closed loop (ntn-slice orchestrator -> NR scheduler, on an SGP4
/// satellite and TR 38.811 UEs from ntn-constellation / ntn-traffic): at 2 s the
/// orchestrator reads the scheduler's MEASURED per-slice backlog, allocates
/// PRBs with its policy, and the shares become the scheduler's quotas. The
/// saturating eMBB slice must get the largest quota, the quotas must be valid
/// (sum <= 1), and URLLC packets keep flowing.
class OrchestratorClosesLoopOnNrCellTest : public TestCase
{
  public:
    OrchestratorClosesLoopOnNrCellTest()
        : TestCase("Cross-module: orchestrator sets live NR quotas from measured backlog")
    {
    }

  private:
    void DoRun() override
    {
        CellOptions opt;
        opt.orchestratorLoop = true;
        const auto r = RunCell(opt);
        NS_TEST_ASSERT_MSG_EQ(r.quotasApplied.size(), 3u, "three quotas applied");
        const double sum = std::accumulate(r.quotasApplied.begin(), r.quotasApplied.end(), 0.0);
        NS_TEST_ASSERT_MSG_LT(sum, 1.0 + 1e-9, "quotas sum to at most 1");
        NS_TEST_ASSERT_MSG_GT(r.quotasApplied[0], r.quotasApplied[2],
                              "the backlogged eMBB slice outranks idle-ish mMTC");
        NS_TEST_ASSERT_MSG_GT(r.quotasApplied[1], 0.0, "URLLC keeps a quota");
        NS_TEST_ASSERT_MSG_EQ(r.slotOverflow, 0u, "slot invariant");
        NS_TEST_ASSERT_MSG_GT(r.rxUrllc, 100u, "URLLC delivered under the orchestrated quotas");
        NS_TEST_ASSERT_MSG_GT(r.rxEmbb, 1000u, "eMBB delivered");
    }
};

} // namespace

class NtnSliceDepthTestSuite : public TestSuite
{
  public:
    NtnSliceDepthTestSuite()
        : TestSuite("ntn-slice-depth", Type::UNIT)
    {
        AddTestCase(new SnssaiAndProfileConformanceTest, TestCase::Duration::QUICK);
        AddTestCase(new SelectorPrecedenceTest, TestCase::Duration::QUICK);
        AddTestCase(new OrchestratorPriorityReservationTest, TestCase::Duration::QUICK);
        AddTestCase(new OrchestratorPropertySweepTest, TestCase::Duration::QUICK);
        AddTestCase(new MonitorPercentileAndLossTest, TestCase::Duration::QUICK);
        AddTestCase(new QuotaSlotInvariantAndRetuneTest, TestCase::Duration::QUICK);
        AddTestCase(new QuotaWorkConservingSwitchTest, TestCase::Duration::QUICK);
        AddTestCase(new QuotaDeterminismTest, TestCase::Duration::QUICK);
        AddTestCase(new OrchestratorClosesLoopOnNrCellTest, TestCase::Duration::QUICK);
    }
};

static NtnSliceDepthTestSuite g_ntnSliceDepthTestSuite;
