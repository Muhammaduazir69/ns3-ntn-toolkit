/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only
//
// Tests for feeder-aware management: the A1 lease expiry fix (ORAN-31), the
// ephemeris contact plan, and the placement / lease / reconciliation rules.

#include "ns3/oran-ntn-a1-interface.h"
#include "ns3/oran-ntn-authority-lease.h"
#include "ns3/oran-ntn-contact-plan.h"
#include "ns3/oran-ntn-gated-error-model.h"
#include "ns3/packet.h"
#include "ns3/string.h"

#include "ns3/constant-velocity-mobility-model.h"
#include "ns3/error-model.h"
#include "ns3/inet-socket-address.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/ipv4-static-routing-helper.h"
#include "ns3/ntn-oran-ai-flow-monitor.h"
#include "ns3/ntn-oran-application.h"
#include "ns3/ntn-oran-sink.h"
#include "ns3/point-to-point-channel.h"
#include "ns3/point-to-point-helper.h"
#include "ns3/pointer.h"
#include "ns3/uinteger.h"

#include <ns3/simulator.h>
#include <ns3/test.h>

#include <algorithm>
#include <cmath>
#include <vector>

using namespace ns3;

namespace
{

A1NtnPolicy
MakeSlicePolicy(double g, Time validUntil)
{
    A1NtnPolicy p;
    p.timestamp = 0.0;
    p.policyId = 0;
    p.type = A1PolicyType::SLICE_SLA;
    p.scope = "slice:1";
    p.priority = 1;
    p.param1 = g;
    p.param2 = 0.1;
    p.param3 = 0.0;
    p.active = true;
    p.orbitalPlaneId = 0;
    p.satelliteId = 0;
    p.beamGroupId = 0;
    p.validFrom = Seconds(0);
    p.validUntil = validUntil;
    p.orbitAware = false;
    p.federatedAggregationWeight = 0.0;
    p.minParticipants = 0;
    p.flRoundInterval = Seconds(0);
    p.maxHandoverRate = 0.0;
    p.minCoverageOverlap = 0.0;
    return p;
}

} // namespace

// ----------------------------------------------------------------------------
// ORAN-31 regression. Before the fix the adapter kept enforcing a policy past
// its validUntil whenever the manager's deactivation could not reach it (the
// feeder-outage case), and it activated a policy that arrived already expired.
// ----------------------------------------------------------------------------
class OranNtnA1LeaseExpiresAtAdapterTest : public TestCase
{
  public:
    OranNtnA1LeaseExpiresAtAdapterTest()
        : TestCase("ORAN-31: A1 adapter honours validUntil without a deactivation message")
    {
    }

  private:
    size_t m_at1{99};
    size_t m_at3{99};
    size_t m_lateAt4{99};
    double m_thrAt3{-1.0};

    void DoRun() override
    {
        auto mgr = CreateObject<OranNtnA1PolicyManager>();
        auto adapter = CreateObject<OranNtnA1Adapter>();
        auto lateAdapter = CreateObject<OranNtnA1Adapter>();
        // The manager reaches the adapter once (creation). Its later expiry
        // deactivation is cut, as a feeder outage would cut it.
        bool linkUp = true;
        mgr->SetDistributionCallback(Callback<void, A1NtnPolicy>(
            [&linkUp, adapter](A1NtnPolicy p) {
                if (linkUp)
                {
                    adapter->HandleIncomingPolicy(p);
                }
            }));
        Simulator::Schedule(Seconds(0.5), [&]() {
            mgr->CreatePolicy(MakeSlicePolicy(0.4, Seconds(2.0)));
        });
        Simulator::Schedule(Seconds(0.6), [&]() { linkUp = false; });
        Simulator::Schedule(Seconds(1.0), [&]() {
            m_at1 = adapter->GetActivePolicies().size();
        });
        Simulator::Schedule(Seconds(3.0), [&]() {
            m_at3 = adapter->GetPoliciesByType(A1PolicyType::SLICE_SLA).size();
            m_thrAt3 = adapter->GetThreshold(A1PolicyType::SLICE_SLA, "tte_min");
        });
        // A policy whose lease lapsed in transit (arrives at 4 s, valid to 3.5 s).
        Simulator::Schedule(Seconds(4.0), [&]() {
            lateAdapter->HandleIncomingPolicy(MakeSlicePolicy(0.5, Seconds(3.5)));
            m_lateAt4 = lateAdapter->GetActivePolicies().size();
        });
        Simulator::Stop(Seconds(5.0));
        Simulator::Run();

        NS_TEST_ASSERT_MSG_EQ(m_at1, 1u, "policy live before its validUntil");
        NS_TEST_ASSERT_MSG_EQ(m_at3, 0u, "policy must lapse at validUntil with no message");
        NS_TEST_ASSERT_MSG_EQ_TOL(m_thrAt3, 0.0, 1e-12, "a lapsed policy yields no threshold");
        NS_TEST_ASSERT_MSG_EQ(m_lateAt4, 0u, "a policy arriving after validUntil is not activated");

        mgr->Dispose();
        adapter->Dispose();
        lateAdapter->Dispose();
        Simulator::Destroy();
    }
};

// validUntil == 0 keeps the historic "no expiry" meaning.
class OranNtnA1NoExpiryUnchangedTest : public TestCase
{
  public:
    OranNtnA1NoExpiryUnchangedTest()
        : TestCase("ORAN-31: a policy without validUntil never lapses")
    {
    }

  private:
    size_t m_n{0};

    void DoRun() override
    {
        auto adapter = CreateObject<OranNtnA1Adapter>();
        adapter->HandleIncomingPolicy(MakeSlicePolicy(0.3, Seconds(0)));
        Simulator::Schedule(Seconds(100), [&]() { m_n = adapter->GetActivePolicies().size(); });
        Simulator::Run();
        NS_TEST_ASSERT_MSG_EQ(m_n, 1u, "no-expiry policy stays active");
        adapter->Dispose();
        Simulator::Destroy();
    }
};

// ----------------------------------------------------------------------------
// Contact plan: a circular equatorial orbit over an equatorial gateway has a
// closed-form visibility window, so edges can be checked to the resolution.
// ----------------------------------------------------------------------------
class OranNtnContactPlanEdgesTest : public TestCase
{
  public:
    OranNtnContactPlanEdgesTest()
        : TestCase("Contact plan windows match the closed-form pass of a circular orbit")
    {
    }

  private:
    void DoRun() override
    {
        // A non-rotating frame is fine for the check: the gateway is fixed in
        // the same frame the position function uses.
        const double re = 6378137.0;
        const double h = 550e3;
        const double r = re + h;
        const double mu = 3.986004418e14;
        const double w = std::sqrt(mu / (r * r * r));
        const double mask = 20.0;
        auto plan = CreateObject<OranNtnContactPlan>();
        plan->AddGateway(7, 0.0, 0.0, 0.0, mask);
        // Satellite crosses the gateway meridian at t = 600 s.
        plan->AddSatellite(1, [=](Time t) {
            const double a = w * (t.GetSeconds() - 600.0);
            return Vector(r * std::cos(a), r * std::sin(a), 0.0);
        });
        plan->Build(Seconds(0), Seconds(1200), Seconds(5), MilliSeconds(1));

        // Closed form: Earth central angle at elevation e is
        // lambda = acos(re cos e / r) - e.
        const double e = mask * M_PI / 180.0;
        const double lambda = std::acos(re * std::cos(e) / r) - e;
        const double half = lambda / w;
        const auto& win = plan->GetWindows(1);
        NS_TEST_ASSERT_MSG_EQ(win.size(), 1u, "one pass in the horizon");
        NS_TEST_ASSERT_MSG_EQ_TOL(win[0].start.GetSeconds(), 600.0 - half, 2e-3, "rise edge");
        NS_TEST_ASSERT_MSG_EQ_TOL(win[0].end.GetSeconds(), 600.0 + half, 2e-3, "set edge");

        NS_TEST_ASSERT_MSG_EQ(plan->IsVisible(1, Seconds(600)), true, "visible at culmination");
        NS_TEST_ASSERT_MSG_EQ(plan->IsVisible(1, Seconds(10)), false, "not visible far away");
        NS_TEST_ASSERT_MSG_EQ_TOL(plan->NextLoss(1, Seconds(600)).GetSeconds(),
                                  600.0 + half,
                                  2e-3,
                                  "next loss is the set edge");
        NS_TEST_ASSERT_MSG_EQ_TOL(plan->NextGain(1, Seconds(10)).GetSeconds(),
                                  600.0 - half,
                                  2e-3,
                                  "next gain is the rise edge");
        NS_TEST_ASSERT_MSG_EQ((plan->NextGain(1, Seconds(1100)) == Time::Max()),
                              true,
                              "no gain after the pass within the horizon");
        NS_TEST_ASSERT_MSG_EQ_TOL(plan->Availability(1, Seconds(0), Seconds(1200)),
                                  2.0 * half / 1200.0,
                                  1e-5,
                                  "availability is the pass fraction");
        Simulator::Destroy();
    }
};

// Union over two gateways merges overlapping pair windows.
class OranNtnContactPlanUnionTest : public TestCase
{
  public:
    OranNtnContactPlanUnionTest()
        : TestCase("Contact plan merges overlapping gateway windows")
    {
    }

  private:
    void DoRun() override
    {
        const double r = 6378137.0 + 550e3;
        const double w = std::sqrt(3.986004418e14 / (r * r * r));
        auto plan = CreateObject<OranNtnContactPlan>();
        plan->AddGateway(1, 0.0, 0.0, 0.0, 20.0);
        plan->AddGateway(2, 0.0, 8.0, 0.0, 20.0);   // overlaps gateway 1
        plan->AddGateway(3, 0.0, 60.0, 0.0, 20.0);  // separate pass
        plan->AddSatellite(1, [=](Time t) {
            const double a = w * t.GetSeconds();
            return Vector(r * std::cos(a), r * std::sin(a), 0.0);
        });
        plan->Build(Seconds(0), Seconds(1500), Seconds(5), MilliSeconds(1));
        NS_TEST_ASSERT_MSG_EQ(plan->GetPairWindows(1, 1).size(), 1u, "gw1 pass");
        NS_TEST_ASSERT_MSG_EQ(plan->GetPairWindows(1, 2).size(), 1u, "gw2 pass");
        NS_TEST_ASSERT_MSG_EQ(plan->GetWindows(1).size(), 2u, "gw1+gw2 merged, gw3 apart");
        NS_TEST_ASSERT_MSG_EQ((plan->GetWindows(1)[0].end == plan->GetPairWindows(1, 2)[0].end),
                              true,
                              "merged window ends where gw2 sets");
        Simulator::Destroy();
    }
};

// ----------------------------------------------------------------------------
// Placement, lease sizing and merge rules.
// ----------------------------------------------------------------------------
class OranNtnAuthorityLeaseRulesTest : public TestCase
{
  public:
    OranNtnAuthorityLeaseRulesTest()
        : TestCase("Authority lease: placement, expiry and reconciliation rules")
    {
    }

  private:
    void DoRun() override
    {
        using L = OranNtnAuthorityLease;
        // Placement: fits with slack and contact long enough -> ground.
        NS_TEST_ASSERT_MSG_EQ((L::Place(MilliSeconds(60), MilliSeconds(100), 0.2, Seconds(30),
                                       Seconds(5)) == L::Host::Ground),
                              true, "60 ms loop fits an 80 ms budget");
        NS_TEST_ASSERT_MSG_EQ((L::Place(MilliSeconds(85), MilliSeconds(100), 0.2, Seconds(30),
                                       Seconds(5)) == L::Host::Onboard),
                              true, "85 ms loop misses an 80 ms budget");
        NS_TEST_ASSERT_MSG_EQ((L::Place(MilliSeconds(10), MilliSeconds(100), 0.2, Seconds(2),
                                       Seconds(5)) == L::Host::Onboard),
                              true, "contact about to end -> onboard");

        // Lease expiry: restore + margin, capped, never before now.
        NS_TEST_ASSERT_MSG_EQ(L::LeaseExpiry(Seconds(10), Seconds(40), Seconds(2), Seconds(600)),
                              Seconds(42), "restore + margin");
        NS_TEST_ASSERT_MSG_EQ(L::LeaseExpiry(Seconds(10), Time::Max(), Seconds(2), Seconds(60)),
                              Seconds(70), "no predicted restore -> cap");
        NS_TEST_ASSERT_MSG_EQ(L::LeaseExpiry(Seconds(10), Seconds(900), Seconds(2), Seconds(60)),
                              Seconds(70), "far restore -> cap");

        // Supersession horizon: eps = 1 - exp(-1) gives exactly the mean.
        NS_TEST_ASSERT_MSG_EQ_TOL(L::LeaseHorizon(Seconds(60), 1.0 - std::exp(-1.0)).GetSeconds(),
                                  60.0, 1e-6, "tau* = T at eps = 1 - 1/e");
        NS_TEST_ASSERT_MSG_EQ_TOL(L::LeaseHorizon(Seconds(60), 0.2).GetSeconds(),
                                  -60.0 * std::log(0.8), 1e-6, "tau* = -T ln(1 - eps)");
        NS_TEST_ASSERT_MSG_EQ(L::LeaseHorizon(Seconds(60), 0.0), Time(0), "eps 0 -> no lease");
        NS_TEST_ASSERT_MSG_EQ((L::LeaseHorizon(Seconds(60), 1.0) == Time::Max()), true,
                              "eps 1 -> hold forever");

        // Merge. Lease [100, 160). Onboard wrote p1 at 120 (v1) and p2 at 130
        // (v1). Ground wrote p1 at 90 (before lease, v1), p1 at 161 right
        // after reconnection (v2), and p3 at 140 (inside the lease, v2).
        std::vector<L::Write> ground = {
            {1, 0.30, 1, Seconds(90), L::Host::Ground},
            {1, 0.45, 2, Seconds(161), L::Host::Ground},
            {3, 0.70, 2, Seconds(140), L::Host::Ground},
        };
        std::vector<L::Write> onboard = {
            {1, 0.35, 2, Seconds(120), L::Host::Onboard},
            {2, 0.50, 1, Seconds(130), L::Host::Onboard},
        };
        auto gw = L::Merge(L::MergeRule::GroundWins, ground, onboard, Seconds(100), Seconds(160),
                           2, 1e-3);
        NS_TEST_ASSERT_MSG_EQ_TOL(gw.state[1], 0.45, 1e-12, "ground wins p1");
        NS_TEST_ASSERT_MSG_EQ((gw.winner[1] == L::Host::Ground), true, "ground winner p1");
        NS_TEST_ASSERT_MSG_EQ_TOL(gw.state[2], 0.50, 1e-12, "only onboard wrote p2");
        NS_TEST_ASSERT_MSG_EQ(gw.directConflicts, 1u, "p1 written by both sides under v2, values differ");
        NS_TEST_ASSERT_MSG_EQ(gw.policyStaleWrites, 1u, "p2 kept under v1 < v2");
        NS_TEST_ASSERT_MSG_EQ(gw.recompute.size(), 0u, "ground-wins never recomputes");

        auto lv = L::Merge(L::MergeRule::LeaseVersioned, ground, onboard, Seconds(100),
                           Seconds(160), 2, 1e-3);
        // p1: authoritative writes are ground@90, ground@161 (outside) and
        // onboard@120 (inside); latest authoritative is ground@161.
        NS_TEST_ASSERT_MSG_EQ_TOL(lv.state[1], 0.45, 1e-12, "post-lease ground write is authoritative");
        // p3: the only write is a ground write inside the lease, which is not
        // authoritative; it is kept only because nothing else exists.
        NS_TEST_ASSERT_MSG_EQ_TOL(lv.state[3], 0.70, 1e-12, "fallback keeps the only write");
        NS_TEST_ASSERT_MSG_EQ((lv.winner[2] == L::Host::Onboard), true, "onboard owns p2");
        NS_TEST_ASSERT_MSG_EQ(lv.recompute.size(), 1u, "p2 flagged for recompute under v2");
        NS_TEST_ASSERT_MSG_EQ(lv.recompute[0], 2u, "the flagged parameter is p2");

        // Writes under different versions are not direct conflicts.
        std::vector<L::Write> g3 = {{1, 0.45, 3, Seconds(161), L::Host::Ground}};
        auto m3 = L::Merge(L::MergeRule::GroundWins, g3, onboard, Seconds(100), Seconds(160), 3, 1e-3);
        NS_TEST_ASSERT_MSG_EQ(m3.directConflicts, 0u, "different versions: not a direct conflict");
        // Without the post-lease ground write, the lease decides p1.
        ground.erase(ground.begin() + 1);
        auto lv2 = L::Merge(L::MergeRule::LeaseVersioned, ground, onboard, Seconds(100),
                            Seconds(160), 1, 1e-3);
        NS_TEST_ASSERT_MSG_EQ_TOL(lv2.state[1], 0.35, 1e-12, "onboard lease write beats older ground");
        NS_TEST_ASSERT_MSG_EQ(lv2.directConflicts, 0u, "no ground write of p1 since lease start");
        // A kept write under the safe intent (version 0) is a fallback, not a
        // superseded intent (D10), and is still recomputed.
        std::vector<L::Write> ob0 = {{1, 0.60, 0, Seconds(150), L::Host::Onboard}};
        std::vector<L::Write> g0 = {{1, 0.30, 2, Seconds(90), L::Host::Ground}};
        auto m0 = L::Merge(L::MergeRule::LeaseVersioned, g0, ob0, Seconds(100), Seconds(160), 2, 1e-3);
        NS_TEST_ASSERT_MSG_EQ(m0.policyStaleWrites, 0u, "safe-intent write is not superseded");
        NS_TEST_ASSERT_MSG_EQ(m0.fallbackWrites, 1u, "safe-intent write counted as fallback");
        NS_TEST_ASSERT_MSG_EQ(m0.recompute.size(), 1u, "safe-intent write recomputed");
        Simulator::Destroy();
    }
};


/// Wave 7B: the feeder-mgmt user plane. Voice packets cross UE -> satellite
/// -> gateway over point-to-point links whose delays are the slant / c of a
/// moving satellite, refreshed every 100 ms. The feeder's receive model is
/// gated by the control plane's on/off RateErrorModel for the same pair.
/// Normal path: everything sent while the control feeder is up arrives, at
/// slant / c plus serialization. Failure path: a 1 s control outage drops
/// exactly the packets sent into it (counted as gated, none delivered), and a
/// lossy inner model on the service link drops packets counted apart.
class OranNtnGatedUserPlaneTest : public TestCase
{
  public:
    OranNtnGatedUserPlaneTest()
        : TestCase("Wave 7B: user plane follows the control feeder state; delay = slant-c")
    {
    }

  private:
    void DoRun() override
    {
        constexpr double kC = 299792458.0;
        NodeContainer n;
        n.Create(3); // UE, satellite, gateway
        InternetStackHelper inet;
        inet.Install(n);
        auto ueM = CreateObject<ConstantVelocityMobilityModel>();
        ueM->SetPosition(Vector(0, 0, 0));
        auto satM = CreateObject<ConstantVelocityMobilityModel>();
        satM->SetPosition(Vector(-300e3, 0, 600e3));
        satM->SetVelocity(Vector(7500, 0, 0));
        auto gwM = CreateObject<ConstantVelocityMobilityModel>();
        gwM->SetPosition(Vector(500e3, 0, 0));

        PointToPointHelper svc;
        svc.SetDeviceAttribute("DataRate", StringValue("1Mbps"));
        NetDeviceContainer d1 = svc.Install(n.Get(0), n.Get(1));
        PointToPointHelper fdr;
        fdr.SetDeviceAttribute("DataRate", StringValue("50Mbps"));
        NetDeviceContainer d2 = fdr.Install(n.Get(1), n.Get(2));
        Ipv4AddressHelper ah;
        ah.SetBase("172.16.0.0", "255.255.255.252");
        Ipv4InterfaceContainer i1 = ah.Assign(d1);
        ah.SetBase("172.32.0.0", "255.255.255.252");
        Ipv4InterfaceContainer i2 = ah.Assign(d2);
        Ipv4StaticRoutingHelper srh;
        srh.GetStaticRouting(n.Get(0)->GetObject<Ipv4>())->SetDefaultRoute(i1.GetAddress(1), 1);
        srh.GetStaticRouting(n.Get(1)->GetObject<Ipv4>())->SetDefaultRoute(i2.GetAddress(1), 2);
        auto svcCh = DynamicCast<PointToPointChannel>(d1.Get(0)->GetChannel());
        auto fdrCh = DynamicCast<PointToPointChannel>(d2.Get(0)->GetChannel());
        std::function<void()> refresh = [&]() {
            svcCh->SetAttribute("Delay", TimeValue(Seconds(satM->GetDistanceFrom(ueM) / kC)));
            fdrCh->SetAttribute("Delay", TimeValue(Seconds(satM->GetDistanceFrom(gwM) / kC)));
            Simulator::Schedule(MilliSeconds(100), refresh);
        };
        refresh();

        // Control plane's on/off switch for this feeder pair; outage 2-3 s.
        auto ctrl = CreateObject<RateErrorModel>();
        ctrl->SetUnit(RateErrorModel::ERROR_UNIT_PACKET);
        ctrl->SetRate(0.0);
        Simulator::Schedule(Seconds(2.0), [ctrl]() { ctrl->SetRate(1.0); });
        Simulator::Schedule(Seconds(3.0), [ctrl]() { ctrl->SetRate(0.0); });
        auto fdrGate = CreateObject<OranNtnGatedErrorModel>();
        fdrGate->SetGate(OranNtnGatedErrorModel::MirrorOutage(ctrl));
        d2.Get(1)->SetAttribute("ReceiveErrorModel", PointerValue(fdrGate));
        // Service link: open gate, lossless until 4 s, then a lossy inner model.
        auto inner = CreateObject<RateErrorModel>();
        inner->SetUnit(RateErrorModel::ERROR_UNIT_PACKET);
        inner->SetRate(0.0);
        Simulator::Schedule(Seconds(4.0), [inner]() { inner->SetRate(1.0); });
        auto svcGate = CreateObject<OranNtnGatedErrorModel>();
        svcGate->SetGate([]() { return false; });
        svcGate->SetInner(inner);
        d1.Get(1)->SetAttribute("ReceiveErrorModel", PointerValue(svcGate));

        Ptr<NtnOranSink> sink = CreateObject<NtnOranSink>();
        sink->SetAttribute("Local", AddressValue(InetSocketAddress(Ipv4Address::GetAny(), 9100)));
        n.Get(2)->AddApplication(sink);
        Ptr<NtnOranApplication> app = CreateObject<NtnOranApplication>();
        app->SetRemote(InetSocketAddress(i2.GetAddress(1), 9100));
        app->SetProfile(NtnOranApplication::CONVERSATIONAL_VOICE);
        app->SetFlowIdentity(1, 1, 1, 1, 1000);
        app->SetAttribute("PacketSize", UintegerValue(92));
        n.Get(0)->AddApplication(app);
        app->SetStartTime(Seconds(1.0));
        app->SetStopTime(Seconds(5.0));
        // Expected one-way delay at the mid-point of the delivered windows.
        double expectMs = 0;
        Simulator::Schedule(Seconds(2.5), [&]() {
            expectMs = (satM->GetDistanceFrom(ueM) + satM->GetDistanceFrom(gwM)) / kC * 1e3 +
                       (92 + 8 + 20 + 2) * 8.0 / 1e6 * 1e3 + (92 + 8 + 20 + 2) * 8.0 / 50e6 * 1e3;
        });
        uint64_t rxAt2 = 0;
        Simulator::Schedule(Seconds(2.0), [&]() { rxAt2 = sink->GetRxPackets(); });
        Simulator::Stop(Seconds(6.0));
        Simulator::Run();

        const uint64_t tx = app->GetTxPackets();
        const uint64_t rx = sink->GetRxPackets();
        NS_TEST_ASSERT_MSG_GT_OR_EQ(tx, 199u, "4 s of 20 ms voice frames");
        // 2-3 s: 50 frames die at the gated feeder; 4-5 s: 50 at the service link.
        NS_TEST_EXPECT_MSG_EQ(fdrGate->GetGatedDrops(), 50u, "outage drops what was sent into it");
        NS_TEST_EXPECT_MSG_GT_OR_EQ(svcGate->GetInnerDrops(), 49u,
                                    "inner model drops counted apart");
        NS_TEST_EXPECT_MSG_EQ(svcGate->GetGatedDrops(), 0u, "open service gate");
        NS_TEST_EXPECT_MSG_EQ(rx, tx - fdrGate->GetGatedDrops() - svcGate->GetInnerDrops(),
                              "everything else is delivered");
        NS_TEST_EXPECT_MSG_EQ(rx, fdrGate->GetPassed(), "the sink saw what the feeder passed");
        NS_TEST_EXPECT_MSG_GT(rxAt2, 45u, "delivered before the outage");
        NS_TEST_EXPECT_MSG_EQ_TOL(sink->GetMeanDelayMs(), expectMs, 0.15,
                                  "one-way delay = (service + feeder slant) / c + serialization");
        Simulator::Destroy();
    }
};

/**
 * Wave 8: the UPF-side sinks of the feeder-mgmt user plane carry UPLINK flows
 * and have no PHY source, so oran-ntn-feeder-mgmt registers them with
 * Direction::Uplink and the KPMs carry the TS 28.552 UL names. Same user
 * plane as above (UE -> satellite -> gateway/UPF, gated feeder), two
 * monitors on the same flows: one registered the way the example now does
 * (UL names, loss counted while the feeder is down: the failure path), one
 * with the old two-argument call, which named these uplink flows DL.
 */
class OranNtnFeederUplinkKpmNamesTest : public TestCase
{
  public:
    OranNtnFeederUplinkKpmNamesTest()
        : TestCase("Wave 8: feeder-mgmt UPF sinks report TS 28.552 UL KPM names")
    {
    }

  private:
    struct Names
    {
        bool ul{false};
        bool dl{false};
        double maxLossUl{0.0};
        double maxThpUl{0.0};
    };

    static Names Scan(const Ptr<NtnOranAiFlowMonitor>& mon)
    {
        Names r;
        for (const auto& [id, series] : mon->GetKpmSeries())
        {
            for (const auto& smp : series)
            {
                r.ul = r.ul || smp.metrics.count("DRB.UEThpUl") ||
                       smp.metrics.count("DRB.PacketLossRateUl") ||
                       smp.metrics.count("DRB.RlcDelayUl");
                r.dl = r.dl || smp.metrics.count("DRB.UEThpDl") ||
                       smp.metrics.count("DRB.PacketLossRateDl") ||
                       smp.metrics.count("DRB.RlcSduDelayDl");
                if (smp.metrics.count("DRB.PacketLossRateUl"))
                {
                    r.maxLossUl = std::max(r.maxLossUl, smp.metrics.at("DRB.PacketLossRateUl"));
                }
                if (smp.metrics.count("DRB.UEThpUl"))
                {
                    r.maxThpUl = std::max(r.maxThpUl, smp.metrics.at("DRB.UEThpUl"));
                }
            }
        }
        return r;
    }

    void DoRun() override
    {
        NodeContainer n;
        n.Create(3); // UE, satellite, gateway with the UPF
        InternetStackHelper inet;
        inet.Install(n);
        PointToPointHelper svc;
        svc.SetDeviceAttribute("DataRate", StringValue("1Mbps"));
        svc.SetChannelAttribute("Delay", StringValue("2ms"));
        NetDeviceContainer d1 = svc.Install(n.Get(0), n.Get(1));
        PointToPointHelper fdr;
        fdr.SetDeviceAttribute("DataRate", StringValue("50Mbps"));
        fdr.SetChannelAttribute("Delay", StringValue("3ms"));
        NetDeviceContainer d2 = fdr.Install(n.Get(1), n.Get(2));
        Ipv4AddressHelper ah;
        ah.SetBase("172.16.0.0", "255.255.255.252");
        Ipv4InterfaceContainer i1 = ah.Assign(d1);
        ah.SetBase("172.32.0.0", "255.255.255.252");
        Ipv4InterfaceContainer i2 = ah.Assign(d2);
        Ipv4StaticRoutingHelper srh;
        srh.GetStaticRouting(n.Get(0)->GetObject<Ipv4>())->SetDefaultRoute(i1.GetAddress(1), 1);
        srh.GetStaticRouting(n.Get(1)->GetObject<Ipv4>())->SetDefaultRoute(i2.GetAddress(1), 2);

        // The serving feeder is down from 3 to 4 s (control-plane state).
        auto ctrl = CreateObject<RateErrorModel>();
        ctrl->SetUnit(RateErrorModel::ERROR_UNIT_PACKET);
        ctrl->SetRate(0.0);
        Simulator::Schedule(Seconds(3.0), [ctrl]() { ctrl->SetRate(1.0); });
        Simulator::Schedule(Seconds(4.0), [ctrl]() { ctrl->SetRate(0.0); });
        auto fdrGate = CreateObject<OranNtnGatedErrorModel>();
        fdrGate->SetGate(OranNtnGatedErrorModel::MirrorOutage(ctrl));
        d2.Get(1)->SetAttribute("ReceiveErrorModel", PointerValue(fdrGate));

        Ptr<NtnOranAiFlowMonitor> mon = CreateObject<NtnOranAiFlowMonitor>();
        Ptr<NtnOranAiFlowMonitor> legacy = CreateObject<NtnOranAiFlowMonitor>();
        std::vector<Ptr<NtnOranApplication>> apps;
        std::vector<Ptr<NtnOranSink>> sinks;
        for (uint32_t u = 0; u < 2; ++u)
        {
            const uint16_t port = static_cast<uint16_t>(9100 + u);
            Ptr<NtnOranSink> sink = CreateObject<NtnOranSink>();
            sink->SetAttribute("Local", AddressValue(InetSocketAddress(Ipv4Address::GetAny(), port)));
            n.Get(2)->AddApplication(sink);
            sink->SetStartTime(Seconds(0.0));
            Ptr<NtnOranApplication> app = CreateObject<NtnOranApplication>();
            app->SetRemote(InetSocketAddress(i2.GetAddress(1), port));
            const bool voice = (u % 2) == 0;
            app->SetProfile(voice ? NtnOranApplication::CONVERSATIONAL_VOICE
                                  : NtnOranApplication::MMTC_PERIODIC);
            app->SetAttribute("PacketSize", UintegerValue(voice ? 92 : 128));
            app->SetFlowIdentity(voice ? 1 : 9, voice ? 1 : 3, 0x000001,
                                 static_cast<uint16_t>(u + 1), 1000);
            n.Get(0)->AddApplication(app);
            app->SetStartTime(Seconds(1.0 + 0.001 * u));
            app->SetStopTime(Seconds(6.0));
            // As oran-ntn-feeder-mgmt registers its user plane now ...
            mon->AddSource(app);
            mon->AddSink(sink, static_cast<int32_t>(u), NtnOranAiFlowMonitor::Direction::Uplink);
            // ... and as it did before (Auto, no PHY source -> DL names).
            legacy->AddSource(app);
            legacy->AddSink(sink, static_cast<int32_t>(u));
            apps.push_back(app);
            sinks.push_back(sink);
        }
        mon->Start();
        legacy->Start();
        Simulator::Stop(Seconds(7.0));
        Simulator::Run();

        uint64_t tx = 0;
        uint64_t rx = 0;
        for (uint32_t u = 0; u < apps.size(); ++u)
        {
            tx += apps[u]->GetTxPackets();
            rx += sinks[u]->GetRxPackets();
        }
        NS_TEST_ASSERT_MSG_GT(rx, 0u, "uplink packets reached the UPF");
        NS_TEST_ASSERT_MSG_LT(rx, tx, "the feeder outage dropped uplink packets");
        NS_TEST_EXPECT_MSG_GT(fdrGate->GetGatedDrops(), 0u, "drops happened at the gated feeder");

        const Names now = Scan(mon);
        NS_TEST_EXPECT_MSG_EQ(now.ul, true, "UPF-side sinks carry the TS 28.552 UL names");
        NS_TEST_EXPECT_MSG_EQ(now.dl, false, "and no DL names");
        NS_TEST_EXPECT_MSG_GT(now.maxThpUl, 0.0, "DRB.UEThpUl measured");
        // Failure path: the outage second shows up as uplink loss (unit 1E-6).
        NS_TEST_EXPECT_MSG_GT(now.maxLossUl, 1.0e5, "DRB.PacketLossRateUl sees the outage");

        const Names old = Scan(legacy);
        NS_TEST_EXPECT_MSG_EQ(old.dl, true, "the two-argument call named these flows DL");
        NS_TEST_EXPECT_MSG_EQ(old.ul, false, "and gave them no UL name");
        Simulator::Destroy();
    }
};

class OranNtnFeederMgmtTestSuite : public TestSuite
{
  public:
    OranNtnFeederMgmtTestSuite()
        : TestSuite("oran-ntn-feeder-mgmt", Type::UNIT)
    {
        AddTestCase(new OranNtnA1LeaseExpiresAtAdapterTest, TestCase::Duration::QUICK);
        AddTestCase(new OranNtnA1NoExpiryUnchangedTest, TestCase::Duration::QUICK);
        AddTestCase(new OranNtnContactPlanEdgesTest, TestCase::Duration::QUICK);
        AddTestCase(new OranNtnContactPlanUnionTest, TestCase::Duration::QUICK);
        AddTestCase(new OranNtnAuthorityLeaseRulesTest, TestCase::Duration::QUICK);
        AddTestCase(new OranNtnGatedUserPlaneTest, TestCase::Duration::QUICK);
        AddTestCase(new OranNtnFeederUplinkKpmNamesTest, TestCase::Duration::QUICK);
    }
};

static OranNtnFeederMgmtTestSuite g_oranNtnFeederMgmtTestSuite;
