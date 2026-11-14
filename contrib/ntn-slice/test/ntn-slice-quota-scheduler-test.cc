/*
 * SPDX-License-Identifier: GPL-2.0-only
 * Copyright (c) 2026 Muhammad Uzair (ns3-ntn-toolkit)
 *
 * End-to-end tests of NtnSliceQuotaScheduler on a real NR cell: one SGP4 satellite at 550 km, TR 38.811 UEs, the
 * 5G-LENA stack through NtnRealStackHelper, UDP flows from the remote host. The scheduler's own RBG counters are
 * checked against the quotas.
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
#include "ns3/sgp4-mobility-model.h"
#include "ns3/test.h"
#include "ns3/walker-constellation.h"

#include <numeric>

using namespace ns3;
using namespace ns3::ntnslice;

namespace
{

// MixedBouquet gives UE u the profile u % 3: 0 mMTC, 1 eMBB, 2 URLLC; scheduler slices are 0 eMBB, 1 URLLC, 2 mMTC.
constexpr uint32_t kSliceOfProfile[3] = {2, 0, 1};

/**
 * Run a 3 s NR cell with six UEs under the slice-quota scheduler and return the share of RBGs each slice got after
 * the first second. \p urllcMbps sets each URLLC UE's load (0 makes the slice idle).
 */
std::vector<double>
RunCell(const std::vector<double>& quotas, double urllcMbps)
{
    RngSeedManager::SetSeed(1);
    RngSeedManager::SetRun(1);
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
            app->SetAttribute("DataRate", DataRateValue(DataRate("60Mb/s"))); // eMBB saturates the cell
        }
        else if (k % 3 == 2)
        {
            app->SetAttribute("PacketSize", UintegerValue(1000));
            app->SetAttribute("Period",
                              TimeValue(urllcMbps > 0 ? Seconds(8000.0 / (urllcMbps * 1e6)) : Seconds(100)));
        }
        ++k;
    }
    auto gnb = DynamicCast<NrGnbNetDevice>(rs.GetEnbDevices().Get(0));
    Ptr<NtnSliceQuotaScheduler> sched = DynamicCast<NtnSliceQuotaScheduler>(gnb->GetScheduler(0));
    NS_ABORT_MSG_IF(!sched, "the gNB does not run NtnSliceQuotaScheduler");
    sched->SetQuotas(quotas);
    std::vector<uint64_t> total(3, 0);
    Simulator::Schedule(Seconds(0.4), [&rs, sched]() {
        for (uint32_t u = 0; u < 6; ++u)
        {
            sched->SetSliceOfRnti(rs.GetUeRnti(u), kSliceOfProfile[u % 3]);
        }
    });
    Simulator::Schedule(Seconds(1.0), [sched]() {
        sched->TakeBytesPerRbg();
        sched->TakeRbgCounts(nullptr);
        sched->TakeBacklog();
    });
    Simulator::Schedule(Seconds(2.9), [sched, &total]() {
        sched->TakeBytesPerRbg();
        const auto c = sched->TakeRbgCounts(nullptr);
        for (size_t s = 0; s < 3 && s < c.size(); ++s)
        {
            total[s] = c[s];
        }
    });
    Simulator::Stop(Seconds(3));
    Simulator::Run();
    Simulator::Destroy();
    const double sum = std::accumulate(total.begin(), total.end(), 0.0);
    std::vector<double> share(3, 0.0);
    for (size_t s = 0; s < 3; ++s)
    {
        share[s] = sum > 0 ? total[s] / sum : 0.0;
    }
    return share;
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
        const auto share = RunCell({0.6, 0.3, 0.1}, 40.0);
        NS_TEST_ASSERT_MSG_GT(share[0] + share[1], 0.9, "the two saturated slices must use nearly every RBG");
        NS_TEST_ASSERT_MSG_GT_OR_EQ(share[1], 0.27, "URLLC is saturated and must get its 0.3 quota");
        NS_TEST_ASSERT_MSG_GT_OR_EQ(share[0], 0.57, "eMBB is saturated and must get its 0.6 quota");
        NS_TEST_ASSERT_MSG_LT(share[1], 0.45, "URLLC must not take eMBB's quota while eMBB has data");
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
        const auto share = RunCell({0.3, 0.6, 0.1}, 0.0);
        NS_TEST_ASSERT_MSG_GT(share[0], 0.85, "eMBB must use the RBGs that the idle URLLC slice leaves");
        NS_TEST_ASSERT_MSG_LT(share[1], 0.05, "an idle URLLC slice must take almost no RBGs");
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

class NtnSliceQuotaSchedulerTestSuite : public TestSuite
{
  public:
    NtnSliceQuotaSchedulerTestSuite()
        : TestSuite("ntn-slice-quota-scheduler", Type::SYSTEM)
    {
        AddTestCase(new QuotaRespectedUnderSaturationTest, TestCase::Duration::EXTENSIVE);
        AddTestCase(new WorkConservationTest, TestCase::Duration::EXTENSIVE);
        AddTestCase(new ContractTrimConvergenceTest, TestCase::Duration::QUICK);
    }
};

static NtnSliceQuotaSchedulerTestSuite g_ntnSliceQuotaSchedulerTestSuite;

} // namespace
