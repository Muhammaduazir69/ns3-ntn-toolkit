/*
 * SPDX-License-Identifier: GPL-2.0-only
 * Copyright (c) 2026 Muhammad Uzair (ns3-ntn-toolkit, W5)
 */
#include "ns3/a2g-channel-tr36777.h"
#include "ns3/aeronautical-scenario.h"
#include "ns3/boolean.h"
#include "ns3/box.h"
#include "ns3/constant-position-mobility-model.h"
#include "ns3/double.h"
#include "ns3/haps-mobility-model.h"
#include "ns3/haps-trajectory-mobility-model.h"
#include "ns3/haps-trajectory-trace.h"
#include "ns3/multi-layer-router.h"
#include "ns3/sagin-custody-queue.h"
#include "ns3/data-rate.h"
#include "ns3/error-model.h"
#include "ns3/inet-socket-address.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/ipv4-global-routing-helper.h"
#include "ns3/ipv4.h"
#include "ns3/node-container.h"
#include "ns3/on-off-helper.h"
#include "ns3/packet-sink-helper.h"
#include "ns3/packet-sink.h"
#include "ns3/point-to-point-helper.h"
#include "ns3/point-to-point-net-device.h"
#include "ns3/pointer.h"
#include "ns3/uinteger.h"
#include "ns3/ais-maritime-trace.h"
#include "ns3/ais-mobility-model.h"
#include "ns3/hst-mobility-model.h"
#include "ns3/hst-trace.h"
#include "ns3/opensky-adsb-trace.h"
#include "ns3/opensky-mobility-model.h"
#include "ns3/sagin-a2g-propagation-loss-model.h"
#include "ns3/sagin-slice-router.h"
#include "ns3/simulator.h"
#include "ns3/test.h"
#include "ns3/uav-mobility-models.h"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <vector>

using namespace ns3;

namespace
{

/// SAGIN-8: the TR 36.777 aerial-vehicle band has an UPPER bound too.
///
/// Table B-1.1/B-1.2 fit these coefficients over 1.5 m to 300 m. PathLossDb
/// guarded only the LOWER threshold, so a HAPS at 20 km, or a
/// SaginA2gPropagationLossModel deriving h_UT as max(pa.z, pb.z) on an
/// unconfigured link, evaluated 300 m coefficients two orders of magnitude
/// outside the data behind them, silently.
/// SAGIN-9: a populated layer must not be mandatory transit.
///
/// Route() walked layers 1..3 in order and `continue`d only when a layer was
/// EMPTY, so if the UAV layer held any node at all the ground source had to
/// transit it, however much better a direct ground-to-LEO link was. Once a hop
/// was committed the choice was never revisited.
class MultiLayerRouterSkipsPopulatedLayerTest : public TestCase
{
  public:
    MultiLayerRouterSkipsPopulatedLayerTest()
        : TestCase("SAGIN-9: a hop may skip a populated layer when a higher one scores better")
    {
    }

  private:
    static Ptr<ConstantPositionMobilityModel> At(double x, double y, double z)
    {
        Ptr<ConstantPositionMobilityModel> m = CreateObject<ConstantPositionMobilityModel>();
        m->SetPosition(Vector(x, y, z));
        return m;
    }

    /// The layer of each hop after the ground source.
    static std::vector<uint8_t> LayerSeq(const std::vector<SaginHop>& path)
    {
        std::vector<uint8_t> out;
        for (size_t i = 1; i < path.size(); ++i)
        {
            out.push_back(static_cast<uint8_t>(path[i].layer));
        }
        return out;
    }

    void DoRun() override
    {
        // A UAV far off to the side (low elevation from the source) and a
        // satellite almost overhead. A router that must transit the UAV layer
        // takes the poor hop; one that may skip goes straight up.
        auto src = At(0.0, 0.0, 0.0);
        auto uav = At(400e3, 0.0, 2e3);     // ~0.3 deg elevation: a bad first hop
        auto sat = At(0.0, 0.0, 600e3);     // straight overhead

        Ptr<MultiLayerRouter> forced = CreateObject<MultiLayerRouter>();
        forced->AddNode(SaginLayer::Uav, uav);
        forced->AddNode(SaginLayer::Leo, sat);
        NS_TEST_ASSERT_MSG_EQ(forced->GetAllowLayerSkip(), false,
                              "layer skipping must default off; enabling it changes the path "
                              "every existing SAGIN scenario takes");
        const auto forcedPath = forced->Route(src);
        const auto forcedSeq = LayerSeq(forcedPath);
        NS_TEST_ASSERT_MSG_GT(forcedSeq.size(), 0u, "the forced route must reach something");
        NS_TEST_ASSERT_MSG_EQ(forcedSeq[0], static_cast<uint8_t>(SaginLayer::Uav),
                              "with skipping off the first hop must be the UAV layer, however "
                              "bad that hop is; that is the behaviour being preserved");

        Ptr<MultiLayerRouter> free = CreateObject<MultiLayerRouter>();
        free->AddNode(SaginLayer::Uav, uav);
        free->AddNode(SaginLayer::Leo, sat);
        free->SetAllowLayerSkip(true);
        const auto freePath = free->Route(src);
        const auto freeSeq = LayerSeq(freePath);
        NS_TEST_ASSERT_MSG_GT(freeSeq.size(), 0u, "the free route must reach something");
        NS_TEST_ASSERT_MSG_EQ(freeSeq[0], static_cast<uint8_t>(SaginLayer::Leo),
                              "with skipping on the first hop must be the overhead satellite, "
                              "not a UAV 400 km off to the side");

        // The two must actually DIFFER here, or the geometry does not separate
        // them and neither assertion means anything.
        NS_TEST_ASSERT_MSG_EQ((forcedSeq == freeSeq), false,
                              "the two policies must produce different paths on this geometry");

        // The path must still CLIMB: a skipped layer must not be revisited, or
        // a route could oscillate between layers.
        for (size_t i = 1; i < freeSeq.size(); ++i)
        {
            NS_TEST_ASSERT_MSG_GT(freeSeq[i], freeSeq[i - 1],
                                  "layers must strictly increase along the path");
        }

        // And where the layers do NOT conflict, both policies must agree, or
        // the flag would change every route rather than the ones it should.
        auto uavGood = At(0.0, 0.0, 20e3); // overhead UAV: a fine first hop
        Ptr<MultiLayerRouter> a = CreateObject<MultiLayerRouter>();
        a->AddNode(SaginLayer::Uav, uavGood);
        Ptr<MultiLayerRouter> b = CreateObject<MultiLayerRouter>();
        b->AddNode(SaginLayer::Uav, uavGood);
        b->SetAllowLayerSkip(true);
        NS_TEST_ASSERT_MSG_EQ((LayerSeq(a->Route(src)) == LayerSeq(b->Route(src))), true,
                              "with only one layer populated the two policies must agree");
    }
};

class A2gValidatedHeightBandTest : public TestCase
{
  public:
    A2gValidatedHeightBandTest()
        : TestCase("SAGIN-8: TR 36.777 declares when a height leaves its validation band")
    {
    }

    void DoRun() override
    {
        // Inside the band: a normal prediction, not flagged.
        const double inside =
            A2gChannelTr36777::PathLossDb(A2gScenario::UMa_AV, A2gLink::LOS, 1000.0, 2.0, 100.0);
        NS_TEST_ASSERT_MSG_EQ(A2gChannelTr36777::WasLastCallAboveValidatedHeight(), false,
                              "100 m is inside the 1.5-300 m band and must not be flagged");
        NS_TEST_ASSERT_MSG_GT(inside, 0.0, "and must return a real path loss");

        // At the boundary: still inside.
        A2gChannelTr36777::PathLossDb(A2gScenario::UMa_AV, A2gLink::LOS, 1000.0, 2.0,
                                      A2gChannelTr36777::kMaxValidatedHeightM);
        NS_TEST_ASSERT_MSG_EQ(A2gChannelTr36777::WasLastCallAboveValidatedHeight(), false,
                              "the boundary itself is within the band");

        // A HAPS at 20 km: outside, and it must SAY so.
        const double haps =
            A2gChannelTr36777::PathLossDb(A2gScenario::UMa_AV, A2gLink::LOS, 20000.0, 2.0, 20000.0);
        NS_TEST_ASSERT_MSG_EQ(A2gChannelTr36777::WasLastCallAboveValidatedHeight(), true,
                              "20 km is far outside the aerial-vehicle band and the caller must "
                              "be able to tell an extrapolation from a prediction");

        // What the extrapolation actually gives, per scenario, is the reason the
        // flag is needed rather than a nicety.
        //
        // UMa_AV's LOS coefficients are {28.0, 22.0} with NO height term at
        // all, so a 20 km link gets literally the 300 m answer: a plausible
        // number, constant in altitude, with nothing on its face to show it
        // came from outside the fitted range. Only the flag distinguishes them.
        const double at300 =
            A2gChannelTr36777::PathLossDb(A2gScenario::UMa_AV, A2gLink::LOS, 20000.0, 2.0, 300.0);
        NS_TEST_ASSERT_MSG_EQ_TOL(haps, at300, 1e-9,
                                  "UMa_AV has height-independent LOS coefficients, so the 20 km "
                                  "answer IS the 300 m answer; that is precisely why the caller "
                                  "needs to be told the height left the band");

        // UMi_AV, by contrast, carries a log10(h) slope term, so extrapolating
        // past the band keeps bending the distance exponent with no data behind
        // it. Both failure modes are silent without the flag.
        const double umiIn =
            A2gChannelTr36777::PathLossDb(A2gScenario::UMi_AV, A2gLink::LOS, 20000.0, 2.0, 300.0);
        NS_TEST_ASSERT_MSG_EQ(A2gChannelTr36777::WasLastCallAboveValidatedHeight(), false,
                              "300 m is in band for UMi_AV too");
        const double umiOut =
            A2gChannelTr36777::PathLossDb(A2gScenario::UMi_AV, A2gLink::LOS, 20000.0, 2.0, 20000.0);
        NS_TEST_ASSERT_MSG_EQ(A2gChannelTr36777::WasLastCallAboveValidatedHeight(), true,
                              "and 20 km is not");
        NS_TEST_ASSERT_MSG_NE(umiIn, umiOut,
                              "UMi_AV's slope depends on log10(h), so past the band it keeps "
                              "extrapolating a fitted term with no data behind it");

        // The flag must be per-call, not sticky, or it is useless after the
        // first out-of-band sample.
        A2gChannelTr36777::PathLossDb(A2gScenario::UMa_AV, A2gLink::LOS, 1000.0, 2.0, 50.0);
        NS_TEST_ASSERT_MSG_EQ(A2gChannelTr36777::WasLastCallAboveValidatedHeight(), false,
                              "the flag must reset on the next in-band call");

        // Below the lower threshold the ground model takes over, and that is not
        // an extrapolation either. Ordered deliberately: set the flag with an
        // out-of-band call FIRST, so the ground path has to clear it. Calling
        // it while the flag is already false asserts nothing.
        A2gChannelTr36777::PathLossDb(A2gScenario::UMa_AV, A2gLink::LOS, 20000.0, 2.0, 20000.0);
        NS_TEST_ASSERT_MSG_EQ(A2gChannelTr36777::WasLastCallAboveValidatedHeight(), true,
                              "precondition: the flag is set going into the ground-model call");
        A2gChannelTr36777::PathLossDb(A2gScenario::UMa_AV, A2gLink::LOS, 1000.0, 2.0, 5.0);
        NS_TEST_ASSERT_MSG_EQ(A2gChannelTr36777::WasLastCallAboveValidatedHeight(), false,
                              "the ground-model delegation below the threshold must clear the "
                              "flag; leaving it stale reports a ground link as an out-of-band "
                              "extrapolation");
    }
};

class HapsAltitudeStableTest : public TestCase
{
  public:
    HapsAltitudeStableTest()
        : TestCase("HAPS holds altitude within ±50 m for 1 hour")
    {
    }

    void DoRun() override
    {
        Ptr<HapsMobilityModel> haps = CreateObject<HapsMobilityModel>();
        haps->SetAttribute("Altitude", DoubleValue(20000.0));
        haps->SetAttribute("MaxVerticalDeviation", DoubleValue(50.0));
        haps->SetCenter(Vector{0, 0, 0});

        double minAlt = 1e12, maxAlt = -1e12;
        for (double t = 0; t <= 3600.0; t += 60.0)
        {
            Simulator::Schedule(Seconds(t), [&minAlt, &maxAlt, haps]() {
                double z = haps->GetPosition().z;
                if (z < minAlt) minAlt = z;
                if (z > maxAlt) maxAlt = z;
            });
        }
        Simulator::Stop(Seconds(3601));
        Simulator::Run();
        Simulator::Destroy();

        NS_TEST_ASSERT_MSG_GT_OR_EQ(minAlt, 19950.0, "HAPS dipped below 19 950 m");
        NS_TEST_ASSERT_MSG_LT_OR_EQ(maxAlt, 20050.0, "HAPS exceeded 20 050 m");
    }
};

class UavPatrolReturnsToStartTest : public TestCase
{
  public:
    UavPatrolReturnsToStartTest()
        : TestCase("Patrol UAV returns to start point each cycle")
    {
    }

    void DoRun() override
    {
        Ptr<UavPatrolMobilityModel> uav = CreateObject<UavPatrolMobilityModel>();
        uav->SetAttribute("Speed", DoubleValue(25.0));
        Vector a{-500, 0, 100};
        Vector b{500, 0, 100};
        uav->SetEndpoints(a, b);

        Vector finalPos;
        // 1000 m at 25 m/s + 1 s pause + 1000 m + 1 s pause = ~82 s per round-trip
        Simulator::Schedule(Seconds(82.5), [&finalPos, uav]() {
            finalPos = uav->GetPosition();
        });
        Simulator::Stop(Seconds(83));
        Simulator::Run();
        Simulator::Destroy();

        // After one full round-trip we should be near @c a again (within speed*tickStep).
        Vector d{finalPos.x - a.x, finalPos.y - a.y, finalPos.z - a.z};
        double dist = std::sqrt(d.x * d.x + d.y * d.y + d.z * d.z);
        NS_TEST_ASSERT_MSG_LT(dist, 50.0,
                              "UAV not within 50 m of start after round-trip "
                              "(d=" << dist << " m)");
    }
};

class A2gPathLossSpotCheckTest : public TestCase
{
  public:
    A2gPathLossSpotCheckTest()
        : TestCase("TR 36.777 PL spot-checks within ±2 dB of spec")
    {
    }

    void DoRun() override
    {
        // All expected values recomputed FROM TR 36.777 Table B-1.2, NOT from
        // the model's own constants. Tolerance is tight (0.05 dB) so a wrong
        // coefficient cannot slip through.

        // RMa-AV LOS, h_UT = 50 m, d3D = 1 km, fc = 2 GHz.
        //   slope = max(23.9 - 1.8*log10(50), 20) = max(20.842, 20) = 20.842
        //   const = 20*log10(40*pi/3) = 32.44   (NOT 28 — the old 4.44 dB bug)
        //   PL_LOS = 32.44 + 20.842*log10(1000) + 20*log10(2)
        //          = 32.44 + 62.526 + 6.021 = 100.99 dB
        double pl = A2gChannelTr36777::PathLossDb(
            A2gScenario::RMa_AV, A2gLink::LOS, 1000.0, 2.0, 50.0);
        NS_TEST_ASSERT_MSG_LT(std::abs(pl - 100.99), 0.05,
                              "PL_LOS RMa-AV out of spec (got " << pl << ")");

        // UMa-AV LOS, h_UT = 100 m, d3D = 500 m, fc = 2 GHz.
        //   PL_LOS = 28 + 22*log10(500) + 20*log10(2)
        //          = 28 + 59.379 + 6.021 = 93.40 dB
        pl = A2gChannelTr36777::PathLossDb(
            A2gScenario::UMa_AV, A2gLink::LOS, 500.0, 2.0, 100.0);
        NS_TEST_ASSERT_MSG_LT(std::abs(pl - 93.40), 0.05,
                              "PL_LOS UMa-AV out of spec (got " << pl << ")");

        // UMi-AV LOS, h_UT = 50 m, d3D = 200 m, fc = 2 GHz.
        //   slope = 22.25 - 0.5*log10(50) = 22.25 - 0.849 = 21.401 (height term!)
        //   PL_LOS = 30.9 + 21.401*log10(200) + 20*log10(2)
        //          = 30.9 + 49.245 + 6.021 = 86.17 dB
        pl = A2gChannelTr36777::PathLossDb(
            A2gScenario::UMi_AV, A2gLink::LOS, 200.0, 2.0, 50.0);
        NS_TEST_ASSERT_MSG_LT(std::abs(pl - 86.17), 0.05,
                              "PL_LOS UMi-AV out of spec (got " << pl << ")");

        // UMa-AV NLOS, h_UT = 50 m, d3D = 200 m, fc = 2 GHz.
        //   const = -17.5 + 32.44 = 14.94; slope = 46 - 7*log10(50) = 34.109
        //   raw = 14.94 + 34.109*log10(200) + 6.021 = 14.94 + 78.49 + 6.02 = 99.45 dB
        //   LOS ref = 28 + 22*log10(200) + 6.021 = 84.65 dB → NLOS = max = 99.45 dB
        pl = A2gChannelTr36777::PathLossDb(
            A2gScenario::UMa_AV, A2gLink::NLOS, 200.0, 2.0, 50.0);
        NS_TEST_ASSERT_MSG_LT(std::abs(pl - 99.45), 0.1,
                              "PL_NLOS UMa-AV out of spec (got " << pl << ")");

        // NLOS PL must be greater than LOS PL at same geometry.
        double plLos = A2gChannelTr36777::PathLossDb(
            A2gScenario::UMa_AV, A2gLink::LOS, 200.0, 2.0, 50.0);
        double plNlos = A2gChannelTr36777::PathLossDb(
            A2gScenario::UMa_AV, A2gLink::NLOS, 200.0, 2.0, 50.0);
        NS_TEST_ASSERT_MSG_GT(plNlos, plLos,
                              "NLOS PL must exceed LOS PL");

        // Below the AV threshold the model must switch to the TR 38.901 GROUND
        // formulas, NOT clamp height into the AV coefficients. At h=10 m
        // (< 22.5 m UMa threshold) the ground LOS/NLOS differ structurally from
        // the AV value; assert the ground NLOS exceeds the AV-at-30m value it
        // used to (wrongly) reuse, i.e. the two branches are genuinely distinct.
        double ground = A2gChannelTr36777::PathLossDb(
            A2gScenario::UMa_AV, A2gLink::NLOS, 500.0, 2.0, 10.0);
        double aerial = A2gChannelTr36777::PathLossDb(
            A2gScenario::UMa_AV, A2gLink::NLOS, 500.0, 2.0, 30.0);
        NS_TEST_ASSERT_MSG_GT(std::abs(ground - aerial), 1.0,
                              "ground (h=10) and AV (h=30) branches must differ");
    }
};

class A2gLosProbabilityMonotonicTest : public TestCase
{
  public:
    A2gLosProbabilityMonotonicTest()
        : TestCase("LOS probability matches TR 36.777 Table B-1.1 spot values")
    {
    }

    void DoRun() override
    {
        using A2 = A2gChannelTr36777;

        // ---- Headline spec check (audit G1): UMa-AV h=50 m, d2D=200 m -------
        // d1 = max(460*log10(50) - 700, 18) = max(81.53, 18) = 81.53
        // p1 = 4300*log10(50) - 3800 = 3505.6
        // P  = 81.53/200 + exp(-200/3505.6)*(1 - 81.53/200) = 0.9672
        double p = A2::LosProbability(A2gScenario::UMa_AV, 200.0, 50.0);
        NS_TEST_ASSERT_MSG_EQ_TOL(p, 0.9672, 0.002,
                                  "UMa-AV h=50,d=200 must be 0.967 (got " << p << ")");

        // ---- UMi-AV must keep the sigmoid to 300 m (audit: NOT saturate to 1)
        // h=300 m, d2D=1 km: d1 = max(294.05*log10(300)-432.94,18)=295.5,
        // p1 = 233.98*log10(300)-0.95 = 578.6, P = 0.4206
        p = A2::LosProbability(A2gScenario::UMi_AV, 1000.0, 300.0);
        NS_TEST_ASSERT_MSG_EQ_TOL(p, 0.4206, 0.003,
                                  "UMi-AV h=300,d=1km must be ~0.42 (got " << p << ")");
        // The OLD code saturated UMi to 1.0 at h>=100 m — assert it does not.
        p = A2::LosProbability(A2gScenario::UMi_AV, 1000.0, 100.0);
        NS_TEST_ASSERT_MSG_LT(p, 0.5,
                              "UMi-AV h=100,d=1km must stay a sigmoid, not 1.0");

        // ---- Band-edge d1 point checks (audit: h = 22.5+, 40, 100, 300 m) --
        // UMa h=100: d1=max(460*2-700,18)=220. So P=1 at d2D<=220, <1 beyond.
        NS_TEST_ASSERT_MSG_EQ_TOL(
            A2::LosProbability(A2gScenario::UMa_AV, 200.0, 100.0), 1.0, 1e-9,
            "UMa d1(100 m)=220 → P=1 at d2D=200");
        NS_TEST_ASSERT_MSG_LT(
            A2::LosProbability(A2gScenario::UMa_AV, 250.0, 100.0), 1.0,
            "UMa d1(100 m)=220 → P<1 at d2D=250");
        // UMa just above 22.5 m: d1 floors at 18.
        NS_TEST_ASSERT_MSG_EQ_TOL(
            A2::LosProbability(A2gScenario::UMa_AV, 18.0, 22.6), 1.0, 1e-9,
            "UMa d1 floors at 18 m near 22.5 m");
        // UMa above 100 m → fully LOS.
        NS_TEST_ASSERT_MSG_EQ_TOL(
            A2::LosProbability(A2gScenario::UMa_AV, 5000.0, 150.0), 1.0, 1e-9,
            "UMa h>100 m → P=1");
        // RMa h=40: d1=max(1350.8*log10(40)-1602,18)=561.8.
        NS_TEST_ASSERT_MSG_EQ_TOL(
            A2::LosProbability(A2gScenario::RMa_AV, 500.0, 40.0), 1.0, 1e-9,
            "RMa d1(40 m)=562 → P=1 at d2D=500");
        NS_TEST_ASSERT_MSG_LT(
            A2::LosProbability(A2gScenario::RMa_AV, 600.0, 40.0), 1.0,
            "RMa d1(40 m)=562 → P<1 at d2D=600");
        // RMa above 40 m → fully LOS.
        NS_TEST_ASSERT_MSG_EQ_TOL(
            A2::LosProbability(A2gScenario::RMa_AV, 5000.0, 60.0), 1.0, 1e-9,
            "RMa h>40 m → P=1");

        // ---- Monotonic in altitude at fixed d2D (sanity) -------------------
        double prev = -1.0;
        for (double h : {1.5, 10.0, 25.0, 50.0, 100.0, 200.0})
        {
            double q = A2::LosProbability(A2gScenario::UMa_AV, 200.0, h);
            NS_TEST_ASSERT_MSG_GT_OR_EQ(q, prev, "LOS prob non-monotonic");
            NS_TEST_ASSERT_MSG_LT_OR_EQ(q, 1.0001, "LOS prob > 1");
            prev = q;
        }
    }
};

class MultiLayerRouterConvergesTest : public TestCase
{
  public:
    MultiLayerRouterConvergesTest()
        : TestCase("Router emits a 4-layer path in <5 s wallclock")
    {
    }

    void DoRun() override
    {
        // 50 nodes per layer — exceeds realistic SAGIN sizing.
        Ptr<MultiLayerRouter> router = CreateObject<MultiLayerRouter>();
        for (int i = 0; i < 50; ++i)
        {
            Ptr<ConstantPositionMobilityModel> uav = CreateObject<ConstantPositionMobilityModel>();
            uav->SetPosition(Vector{100.0 * i, 0, 150.0});
            router->AddNode(SaginLayer::Uav, uav);

            Ptr<ConstantPositionMobilityModel> haps = CreateObject<ConstantPositionMobilityModel>();
            haps->SetPosition(Vector{1000.0 * i, 0, 20000.0});
            router->AddNode(SaginLayer::Haps, haps);

            Ptr<ConstantPositionMobilityModel> leo = CreateObject<ConstantPositionMobilityModel>();
            leo->SetPosition(Vector{1.0e6 * i, 0, 550000.0});
            router->AddNode(SaginLayer::Leo, leo);
        }

        Ptr<ConstantPositionMobilityModel> source = CreateObject<ConstantPositionMobilityModel>();
        source->SetPosition(Vector{0, 0, 0});

        auto t0 = std::chrono::steady_clock::now();
        auto path = router->Route(source);
        auto dt = std::chrono::steady_clock::now() - t0;
        double secs = std::chrono::duration<double>(dt).count();

        NS_TEST_ASSERT_MSG_EQ(path.size(), 4u,
                              "expected 4-layer path, got " << path.size());
        NS_TEST_ASSERT_MSG_LT(secs, 5.0,
                              "route() took " << secs << " s, > 5 s gate");
    }
};

class AeronauticalReachesArrivalTest : public TestCase
{
  public:
    AeronauticalReachesArrivalTest()
        : TestCase("Aircraft reaches arrival point within expected ETA")
    {
    }

    void DoRun() override
    {
        Ptr<AeronauticalMobilityModel> ac = CreateObject<AeronauticalMobilityModel>();
        Vector dep{0, 0, 0};
        Vector arr{250000, 0, 0};   // 250 km
        ac->SetFlightPlan(dep, arr, 11000.0, 250.0);
        // ETA = 250 km / 250 m/s = 1000 s

        Vector finalPos;
        Simulator::Schedule(Seconds(1010), [&finalPos, ac]() {
            finalPos = ac->GetPosition();
        });
        Simulator::Stop(Seconds(1020));
        Simulator::Run();
        Simulator::Destroy();

        Vector d{finalPos.x - arr.x, finalPos.y - arr.y, finalPos.z - 11000.0};
        double dist = std::sqrt(d.x * d.x + d.y * d.y + d.z * d.z);
        NS_TEST_ASSERT_MSG_LT(dist, 100.0,
                              "Aircraft not at arrival after ETA (d=" << dist << " m)");
    }
};

// ============================================================================
// Roadmap §4.4.1: OpenSky ADS-B trace importer + replay mobility model
// ============================================================================

class OpenSkyImporterParseTest : public TestCase
{
  public:
    OpenSkyImporterParseTest()
        : TestCase("OpenSky importer parses CSV header + rows for multiple aircraft")
    {
    }

  private:
    void DoRun() override
    {
        sagin::OpenSkyAdsbImporter imp;
        const std::string candidates[] = {
            "contrib/ntn-sagin/data/opensky-sample-trace.csv",
            "/home/uzair/6g_ntn_ns3/ns-3-dev/contrib/ntn-sagin/data/"
            "opensky-sample-trace.csv",
        };
        std::map<std::string, sagin::OpenSkyAdsbTrace> traces;
        for (const auto& p : candidates)
        {
            traces = imp.LoadCsv(p);
            if (!traces.empty())
                break;
        }
        NS_TEST_ASSERT_MSG_GT(traces.size(), 0u,
                              "bundled OpenSky sample not loaded");
        // Sample dump has 3 icao24 codes.
        NS_TEST_ASSERT_MSG_EQ(traces.size(), 3u, "3 aircraft expected");
        NS_TEST_ASSERT_MSG_EQ(traces.count("4ca7b7"), 1u, "DLH123 present");
        NS_TEST_ASSERT_MSG_EQ(traces.count("abc123"), 1u, "BAW456 present");
        NS_TEST_ASSERT_MSG_EQ(traces.count("ground42"), 1u, "ground aircraft");

        const auto& dlh = traces.at("4ca7b7");
        NS_TEST_ASSERT_MSG_EQ(dlh.samples.size(), 7u, "DLH123 has 7 samples");
        // First sample must match the CSV (lat=52, lon=9, alt=1500).
        NS_TEST_ASSERT_MSG_EQ_TOL(dlh.samples.front().lat_deg, 52.0, 1e-9,
                                  "first lat");
        NS_TEST_ASSERT_MSG_EQ_TOL(dlh.samples.front().lon_deg, 9.0, 1e-9,
                                  "first lon");
        NS_TEST_ASSERT_MSG_EQ_TOL(dlh.samples.front().alt_m, 1500.0, 1e-6,
                                  "first alt (baro)");
        // Last sample: lon should have moved ~ 0.162° east.
        NS_TEST_ASSERT_MSG_GT(dlh.samples.back().lon_deg,
                              dlh.samples.front().lon_deg,
                              "longitude advances east");
        NS_TEST_ASSERT_MSG_GT(dlh.samples.back().alt_m,
                              dlh.samples.front().alt_m,
                              "aircraft climbed during trace");

        // Ground aircraft: onground=true => alt clamped to 0 m.
        const auto& gr = traces.at("ground42");
        for (const auto& s : gr.samples)
        {
            NS_TEST_ASSERT_MSG_EQ(s.alt_m, 0.0,
                                  "onground => alt clamped to 0");
        }
    }
};

class OpenSkyImporterMalformedTest : public TestCase
{
  public:
    OpenSkyImporterMalformedTest()
        : TestCase("OpenSky importer skips malformed rows and counts them")
    {
    }

  private:
    void DoRun() override
    {
        const std::string path = "/tmp/opensky-test-bad.csv";
        std::ofstream f(path);
        f << "time,icao24,lat,lon,velocity,heading,vertrate,callsign,"
             "onground,alert,spi,squawk,baroaltitude,geoaltitude,"
             "lastposupdate,lastcontact\n";
        f << "1572912000,abc,52.0,9.0,200,90,0,X,false,false,false,0,1000,1000,0,0\n";
        f << ",,,bad,row,here\n"; // malformed
        f << "1572912010,abc,52.001,9.0,200,90,0,X,false,false,false,0,1000,1000,0,0\n";
        f << "1572912020,,no_icao,,,,,,,,,,,,\n"; // empty icao
        f << "1572912030,def,not_a_number,9.0,200,90,0,X,false,false,false,0,1000,1000,0,0\n";
        f.close();

        sagin::OpenSkyAdsbImporter imp;
        auto traces = imp.LoadCsv(path);
        NS_TEST_ASSERT_MSG_EQ(traces.size(), 1u,
                              "only the 'abc' aircraft is valid");
        NS_TEST_ASSERT_MSG_EQ(traces.at("abc").samples.size(), 2u,
                              "2 valid samples for abc");
        NS_TEST_ASSERT_MSG_GT(imp.LastRowsSkipped(), 0u, "skip counter > 0");
        std::remove(path.c_str());
    }
};

class OpenSkyTraceInterpolationTest : public TestCase
{
  public:
    OpenSkyTraceInterpolationTest()
        : TestCase("OpenSky trace InterpolateAt linearly interpolates lat lon alt")
    {
    }

  private:
    void DoRun() override
    {
        sagin::OpenSkyAdsbTrace tr;
        tr.icao24 = "tst";
        tr.samples.push_back({0.0, 52.0, 9.0, 1000.0, 100.0, 90.0});
        tr.samples.push_back({10.0, 52.01, 9.01, 1500.0, 200.0, 90.0});

        // Midpoint should give average values.
        auto mid = tr.InterpolateAt(5.0);
        NS_TEST_ASSERT_MSG_EQ_TOL(mid.lat_deg, 52.005, 1e-9, "lat mid");
        NS_TEST_ASSERT_MSG_EQ_TOL(mid.lon_deg, 9.005, 1e-9, "lon mid");
        NS_TEST_ASSERT_MSG_EQ_TOL(mid.alt_m, 1250.0, 1e-9, "alt mid");
        NS_TEST_ASSERT_MSG_EQ_TOL(mid.velocity_mps, 150.0, 1e-9, "vel mid");

        // Below range -> first sample.
        auto before = tr.InterpolateAt(-5.0);
        NS_TEST_ASSERT_MSG_EQ_TOL(before.lat_deg, 52.0, 1e-9, "clamp low");
        // Above range -> last sample.
        auto after = tr.InterpolateAt(20.0);
        NS_TEST_ASSERT_MSG_EQ_TOL(after.lat_deg, 52.01, 1e-9, "clamp high");

        // Heading interpolation wraps around 0/360.
        tr.samples.clear();
        tr.samples.push_back({0.0, 0.0, 0.0, 0.0, 100.0, 350.0});
        tr.samples.push_back({10.0, 0.0, 0.0, 0.0, 100.0, 10.0});
        // Midpoint heading should be 0 (or 360 - either is fine).
        auto h = tr.InterpolateAt(5.0);
        const bool headingNearZero =
            (h.heading_deg < 5.0) || (h.heading_deg > 355.0);
        NS_TEST_ASSERT_MSG_EQ(headingNearZero,
                              true,
                              "heading interpolates around 0/360");
    }
};

namespace
{

struct TraceSample
{
    double t_s;
    double east_m;
    double north_m;
    double up_m;
    double speed_mps;
};

void
SampleOpenSkyModel(Ptr<sagin::OpenSkyMobilityModel> mob,
                    std::vector<TraceSample>* out)
{
    Vector p = mob->GetPosition();
    Vector v = mob->GetVelocity();
    const double sp = std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
    out->push_back(
        {Simulator::Now().GetSeconds(), p.x, p.y, p.z, sp});
}

} // namespace

class OpenSkySimulatorTimeReplayTest : public TestCase
{
  public:
    OpenSkySimulatorTimeReplayTest()
        : TestCase("Simulator: 60 s OpenSky replay matches CSV trajectory in ENU")
    {
    }

  private:
    void DoRun() override
    {
        sagin::OpenSkyAdsbImporter imp;
        const std::string candidates[] = {
            "contrib/ntn-sagin/data/opensky-sample-trace.csv",
            "/home/uzair/6g_ntn_ns3/ns-3-dev/contrib/ntn-sagin/data/"
            "opensky-sample-trace.csv",
        };
        std::map<std::string, sagin::OpenSkyAdsbTrace> traces;
        for (const auto& p : candidates)
        {
            traces = imp.LoadCsv(p);
            if (!traces.empty())
                break;
        }
        NS_TEST_ASSERT_MSG_GT(traces.size(), 0u, "sample trace loaded");
        const auto& dlh = traces.at("4ca7b7");

        Ptr<sagin::OpenSkyMobilityModel> mob =
            CreateObject<sagin::OpenSkyMobilityModel>();
        mob->SetTrace(dlh);
        // Reference at the trace start (lat=52, lon=9, alt=1500).
        mob->SetReference(52.0, 9.0, 1500.0);
        mob->SetTraceTimeOffsetSeconds(dlh.TStart());

        std::vector<TraceSample> samples;
        for (int t = 0; t <= 60; t += 5)
        {
            Simulator::Schedule(Seconds(t), &SampleOpenSkyModel, mob,
                                &samples);
        }
        Simulator::Stop(Seconds(61));
        Simulator::Run();

        NS_TEST_ASSERT_MSG_EQ(samples.size(), 13u, "13 samples over 60 s");

        // At t=0, position must be exactly at the reference origin.
        NS_TEST_ASSERT_MSG_EQ_TOL(samples.front().east_m, 0.0, 1.0,
                                  "t=0 east≈0");
        NS_TEST_ASSERT_MSG_EQ_TOL(samples.front().north_m, 0.0, 1.0,
                                  "t=0 north≈0");
        NS_TEST_ASSERT_MSG_EQ_TOL(samples.front().up_m, 0.0, 1.0,
                                  "t=0 up≈0");

        // Monotonic east (aircraft heading 90°, longitude increases).
        for (size_t i = 1; i < samples.size(); ++i)
        {
            NS_TEST_ASSERT_MSG_GT(samples[i].east_m,
                                  samples[i - 1].east_m,
                                  "east increases as aircraft flies E");
        }

        // Vertical climb during the first 30 s.
        NS_TEST_ASSERT_MSG_GT(samples[6].up_m, samples[0].up_m,
                              "altitude climbed by t=30 s");

        // Speed in the trace is 180..250 m/s. ENU velocity magnitude must
        // land in that range.
        for (const auto& s : samples)
        {
            NS_TEST_ASSERT_MSG_GT(s.speed_mps, 100.0,
                                  "speed > 100 m/s during cruise");
            NS_TEST_ASSERT_MSG_LT(s.speed_mps, 400.0,
                                  "speed < 400 m/s during cruise");
        }

        // Past TEnd, position clamps to the last sample.
        Simulator::Destroy();
    }
};

// ============================================================================
// Roadmap §4.4.2: AIS maritime trace importer + replay mobility model
// ============================================================================

class AisImporterParseTest : public TestCase
{
  public:
    AisImporterParseTest()
        : TestCase("AIS Danish importer parses CSV header + rows for multiple MMSIs")
    {
    }

  private:
    void DoRun() override
    {
        sagin::AisDanishImporter imp;
        const std::string candidates[] = {
            "contrib/ntn-sagin/data/ais-sample-trace.csv",
            "/home/uzair/6g_ntn_ns3/ns-3-dev/contrib/ntn-sagin/data/"
            "ais-sample-trace.csv",
        };
        std::map<uint32_t, sagin::AisMaritimeTrace> traces;
        for (const auto& p : candidates)
        {
            traces = imp.LoadCsv(p);
            if (!traces.empty())
                break;
        }
        NS_TEST_ASSERT_MSG_GT(traces.size(), 0u,
                              "bundled AIS sample not loaded");
        // 4 distinct MMSIs in the bundled sample.
        NS_TEST_ASSERT_MSG_EQ(traces.size(), 4u, "4 vessels expected");
        NS_TEST_ASSERT_MSG_EQ(traces.count(219015785u), 1u, "DLK cargo");
        NS_TEST_ASSERT_MSG_EQ(traces.count(257891234u), 1u, "OSL pleasure");
        NS_TEST_ASSERT_MSG_EQ(traces.count(538001234u), 1u, "PIRAEUS tanker");

        const auto& cargo = traces.at(219015785u);
        NS_TEST_ASSERT_MSG_EQ(cargo.samples.size(), 5u, "5 cargo samples");
        NS_TEST_ASSERT_MSG_EQ(cargo.ship_type, "Cargo", "ship type");
        // First sample at lat=55, lon=12, SOG=12, COG=90.
        const auto& s0 = cargo.samples.front();
        NS_TEST_ASSERT_MSG_EQ_TOL(s0.lat_deg, 55.0, 1e-9, "lat");
        NS_TEST_ASSERT_MSG_EQ_TOL(s0.lon_deg, 12.0, 1e-9, "lon");
        NS_TEST_ASSERT_MSG_EQ_TOL(s0.sog_knots, 12.0, 1e-9, "SOG");
        NS_TEST_ASSERT_MSG_EQ_TOL(s0.cog_deg, 90.0, 1e-9, "COG");
        // Heading 511 (not available) sentinel preserved on the base station.
        const auto& base = traces.at(2190001u);
        NS_TEST_ASSERT_MSG_EQ_TOL(base.samples.front().heading_deg, 511.0, 1e-9,
                                  "heading=511 sentinel preserved");

        // Last sample of the cargo vessel has moved east.
        NS_TEST_ASSERT_MSG_GT(cargo.samples.back().lon_deg,
                              cargo.samples.front().lon_deg,
                              "cargo moves east during trace");
    }
};

class AisImporterMalformedTest : public TestCase
{
  public:
    AisImporterMalformedTest()
        : TestCase("AIS importer skips malformed rows and counts them")
    {
    }

  private:
    void DoRun() override
    {
        const std::string path = "/tmp/ais-test-bad.csv";
        std::ofstream f(path);
        f << "Timestamp,Type of mobile,MMSI,Latitude,Longitude,Navigational "
             "status,ROT,SOG,COG,Heading,IMO,Callsign,Name,Ship type,Cargo "
             "type,Width,Length,Type of position fixing device,Draught,"
             "Destination,ETA,Data source type,A,B,C,D\n";
        f << "07/10/2019 00:00:00,Class A,123456789,55.0,12.0,Under way using "
             "engine,0,10,90,90,,,SHIP,Cargo,,,,GPS,0,,,AIS,,,,\n";
        f << ",,,bad,row,here\n";
        f << "07/10/2019 00:00:30,Class A,123456789,55.001,12.0,Under way using "
             "engine,0,10,90,90,,,SHIP,Cargo,,,,GPS,0,,,AIS,,,,\n";
        f << "07/10/2019 00:00:30,Class A,0,55.001,12.0,Under way using "
             "engine,0,10,90,90,,,SHIP,Cargo,,,,GPS,0,,,AIS,,,,\n"; // mmsi=0
        f << "garbage,Class A,234,not_a_lat,12.0,,,,,,,,,,,,,,,,,,,,,\n";
        f.close();

        sagin::AisDanishImporter imp;
        auto traces = imp.LoadCsv(path);
        NS_TEST_ASSERT_MSG_EQ(traces.size(), 1u,
                              "only MMSI 123456789 is valid");
        NS_TEST_ASSERT_MSG_EQ(traces.at(123456789u).samples.size(), 2u,
                              "2 valid samples");
        NS_TEST_ASSERT_MSG_GT(imp.LastRowsSkipped(), 0u, "skip counter > 0");
        std::remove(path.c_str());
    }
};

class AisTraceInterpolationTest : public TestCase
{
  public:
    AisTraceInterpolationTest()
        : TestCase("AIS trace InterpolateAt linearly interpolates lat lon sog cog")
    {
    }

  private:
    void DoRun() override
    {
        sagin::AisMaritimeTrace tr;
        tr.mmsi = 1234;
        tr.samples.push_back({0.0, 55.0, 12.0, 10.0, 90.0, 90.0,
                              "Cargo", "TST"});
        tr.samples.push_back({10.0, 55.001, 12.001, 12.0, 100.0, 100.0,
                              "Cargo", "TST"});

        auto mid = tr.InterpolateAt(5.0);
        NS_TEST_ASSERT_MSG_EQ_TOL(mid.lat_deg, 55.0005, 1e-9, "lat mid");
        NS_TEST_ASSERT_MSG_EQ_TOL(mid.lon_deg, 12.0005, 1e-9, "lon mid");
        NS_TEST_ASSERT_MSG_EQ_TOL(mid.sog_knots, 11.0, 1e-9, "SOG mid");
        NS_TEST_ASSERT_MSG_EQ_TOL(mid.cog_deg, 95.0, 1e-9, "COG mid");

        // Below range -> first sample.
        auto before = tr.InterpolateAt(-5.0);
        NS_TEST_ASSERT_MSG_EQ_TOL(before.lat_deg, 55.0, 1e-9, "clamp low");
        // Above range -> last sample.
        auto after = tr.InterpolateAt(20.0);
        NS_TEST_ASSERT_MSG_EQ_TOL(after.lat_deg, 55.001, 1e-9, "clamp high");
    }
};

namespace
{

struct AisSimSample
{
    double t_s;
    double east_m;
    double north_m;
    double speed_mps;
};

void
SampleAisModel(Ptr<sagin::AisMobilityModel> mob,
                std::vector<AisSimSample>* out)
{
    Vector p = mob->GetPosition();
    Vector v = mob->GetVelocity();
    const double sp = std::sqrt(v.x * v.x + v.y * v.y);
    out->push_back({Simulator::Now().GetSeconds(), p.x, p.y, sp});
}

} // namespace

class AisSimulatorTimeReplayTest : public TestCase
{
  public:
    AisSimulatorTimeReplayTest()
        : TestCase("Simulator: 120 s AIS replay matches CSV trajectory at hull speed")
    {
    }

  private:
    void DoRun() override
    {
        sagin::AisDanishImporter imp;
        const std::string candidates[] = {
            "contrib/ntn-sagin/data/ais-sample-trace.csv",
            "/home/uzair/6g_ntn_ns3/ns-3-dev/contrib/ntn-sagin/data/"
            "ais-sample-trace.csv",
        };
        std::map<uint32_t, sagin::AisMaritimeTrace> traces;
        for (const auto& p : candidates)
        {
            traces = imp.LoadCsv(p);
            if (!traces.empty())
                break;
        }
        NS_TEST_ASSERT_MSG_GT(traces.size(), 0u, "sample loaded");
        const auto& cargo = traces.at(219015785u);

        Ptr<sagin::AisMobilityModel> mob =
            CreateObject<sagin::AisMobilityModel>();
        mob->SetTrace(cargo);
        mob->SetReference(55.0, 12.0);
        mob->SetTraceTimeOffsetSeconds(cargo.TStart());

        std::vector<AisSimSample> samples;
        for (int t = 0; t <= 120; t += 10)
        {
            Simulator::Schedule(Seconds(t), &SampleAisModel, mob, &samples);
        }
        Simulator::Stop(Seconds(121));
        Simulator::Run();

        NS_TEST_ASSERT_MSG_EQ(samples.size(), 13u, "13 samples over 120 s");

        // t=0 at reference origin.
        NS_TEST_ASSERT_MSG_EQ_TOL(samples.front().east_m, 0.0, 1.0, "t=0 east");
        NS_TEST_ASSERT_MSG_EQ_TOL(samples.front().north_m, 0.0, 1.0, "t=0 north");

        // Hull speed at 12 knots = 6.17 m/s.
        const double expectedMps = 12.0 * 0.5144;
        for (const auto& s : samples)
        {
            NS_TEST_ASSERT_MSG_EQ_TOL(
                s.speed_mps,
                expectedMps,
                0.5,
                "speed should be ~12 knots = 6.17 m/s");
        }

        // Past the last sample (t > 120s of trace) position clamps to
        // the last sample's east. With the trace ending at t=120s and the
        // cargo at lon=12.0068 the east displacement is ~430 m.
        const auto& last = samples.back();
        NS_TEST_ASSERT_MSG_GT(last.east_m, 100.0,
                              "vessel travelled > 100 m east");
        NS_TEST_ASSERT_MSG_LT(std::abs(last.north_m), 5.0,
                              "vessel kept ~constant latitude");

        Simulator::Destroy();
    }
};

// ============================================================================
// Roadmap §4.4.3: HST mobility (TR 38.901 §7.5)
// ============================================================================

class HstPresetGeometryTest : public TestCase
{
  public:
    HstPresetGeometryTest()
        : TestCase("TR 38.901 HST presets carry the right speed and Dmin")
    {
    }

  private:
    void DoRun() override
    {
        auto A = sagin::HstTraceGenerator::PresetTR38901_A(60.0, 61);
        NS_TEST_ASSERT_MSG_EQ_TOL(A.speed_kmh, 500.0, 1e-9, "HST-A speed");
        NS_TEST_ASSERT_MSG_EQ_TOL(A.dmin_m, 150.0, 1e-9, "HST-A Dmin");
        NS_TEST_ASSERT_MSG_EQ_TOL(A.cellSpacing_m, 300.0, 1e-9, "HST-A Ds");
        NS_TEST_ASSERT_MSG_EQ_TOL(A.SpeedMps(), 500.0 / 3.6, 1e-9,
                                  "HST-A speed in m/s");

        auto B = sagin::HstTraceGenerator::PresetTR38901_B(60.0, 61);
        NS_TEST_ASSERT_MSG_EQ_TOL(B.speed_kmh, 300.0, 1e-9, "HST-B speed");
        NS_TEST_ASSERT_MSG_EQ_TOL(B.dmin_m, 10.0, 1e-9, "HST-B Dmin");

        auto C = sagin::HstTraceGenerator::PresetTR38901_C(60.0, 61);
        NS_TEST_ASSERT_MSG_EQ_TOL(C.speed_kmh, 350.0, 1e-9, "HST-C speed");

        // 61 samples evenly spaced 0..60 s -> dt = 1 s.
        NS_TEST_ASSERT_MSG_EQ(A.samples.size(), 61u, "61 samples");
        NS_TEST_ASSERT_MSG_EQ_TOL(A.samples[1].time_s - A.samples[0].time_s,
                                  1.0, 1e-9, "1 s sample interval");
        // After 1 s at 500 km/h, x = 138.89 m.
        NS_TEST_ASSERT_MSG_EQ_TOL(A.samples[1].x_m, 138.889, 0.01,
                                  "x at t=1 s");
    }
};

class HstDopplerShiftTest : public TestCase
{
  public:
    HstDopplerShiftTest()
        : TestCase("TR 38.901 HST Doppler shift matches v_radial over c times f_c")
    {
    }

  private:
    void DoRun() override
    {
        auto tr = sagin::HstTraceGenerator::PresetTR38901_A(60.0, 61);
        // gNB sits ahead of the train at along-track x = 1000 m, Dmin = 150 m
        // perpendicular. At t=0 the train is far back (x=0), v=138.89 m/s
        // along +x.
        //   dx = 1000  dy = 150  r = sqrt(1000^2+150^2) = 1011.19 m
        //   v_radial = v * dx/r = 138.89 * 1000 / 1011.19 = 137.36 m/s
        //   f_d = 137.36 / 3e8 * 30e9 = 13.736 kHz (approaching)
        const double f_d_30G = tr.DopplerHzAt(0.0, 1000.0, 30e9);
        NS_TEST_ASSERT_MSG_GT(f_d_30G, 0.0,
                              "approaching => positive Doppler");
        NS_TEST_ASSERT_MSG_EQ_TOL(f_d_30G, 13736.0, 100.0,
                                  "Doppler @ 30 GHz, t=0, gNB 1km ahead");

        // After the train passes the gNB (t large -> train past gNB),
        // Doppler must flip sign (receding).
        const double t_pass = 1000.0 / tr.SpeedMps(); // ~7.2 s
        const double f_d_after = tr.DopplerHzAt(t_pass + 2.0, 1000.0, 30e9);
        NS_TEST_ASSERT_MSG_LT(f_d_after, 0.0,
                              "receding => negative Doppler");

        // Right at closest approach the radial component is 0 ->
        // Doppler ≈ 0.
        const double f_d_closest = tr.DopplerHzAt(t_pass, 1000.0, 30e9);
        NS_TEST_ASSERT_MSG_LT(std::abs(f_d_closest), 100.0,
                              "Doppler ≈ 0 at closest approach");

        // f_d scales linearly in carrier frequency. At 4 GHz the Doppler
        // is 4/30 of the 30 GHz number.
        const double f_d_4G = tr.DopplerHzAt(0.0, 1000.0, 4e9);
        NS_TEST_ASSERT_MSG_EQ_TOL(f_d_4G * 30.0 / 4.0,
                                  f_d_30G,
                                  10.0,
                                  "Doppler scales linearly with carrier");
    }
};

namespace
{

struct HstSimSample
{
    double t_s;
    double x_m;
    double v_mps;
    double doppler_hz;
};

void
SampleHstModel(Ptr<sagin::HstMobilityModel> mob,
                double gnb_x_m,
                std::vector<HstSimSample>* out)
{
    Vector p = mob->GetPosition();
    Vector v = mob->GetVelocity();
    const double d = mob->GetDopplerHz(gnb_x_m);
    out->push_back({Simulator::Now().GetSeconds(), p.x, v.x, d});
}

} // namespace

class HstSimulatorTimePassByTest : public TestCase
{
  public:
    HstSimulatorTimePassByTest()
        : TestCase("Simulator: 60 s HST-A pass-by at 500 kmh yields Doppler "
                   "sign flip at closest approach")
    {
    }

  private:
    void DoRun() override
    {
        Ptr<sagin::HstMobilityModel> mob =
            CreateObject<sagin::HstMobilityModel>();
        mob->SetTrace(sagin::HstTraceGenerator::PresetTR38901_A(60.0, 61));
        mob->SetCarrierFrequencyHz(30e9);

        const double gnb_x_m = 3000.0; // gNB 3 km along-track from start

        std::vector<HstSimSample> samples;
        for (int t = 0; t <= 60; ++t)
        {
            Simulator::Schedule(Seconds(t),
                                &SampleHstModel,
                                mob,
                                gnb_x_m,
                                &samples);
        }
        Simulator::Stop(Seconds(61));
        Simulator::Run();

        NS_TEST_ASSERT_MSG_EQ(samples.size(), 61u, "61 samples");

        // Constant velocity check.
        const double expectV = 500.0 / 3.6;
        for (const auto& s : samples)
        {
            NS_TEST_ASSERT_MSG_EQ_TOL(s.v_mps, expectV, 1e-6,
                                      "velocity constant at 500 km/h");
        }
        // x monotonic.
        for (size_t i = 1; i < samples.size(); ++i)
        {
            NS_TEST_ASSERT_MSG_GT(samples[i].x_m, samples[i - 1].x_m,
                                  "position monotonic");
        }
        // x at t=60 ≈ 138.89 * 60 = 8333.3 m.
        NS_TEST_ASSERT_MSG_EQ_TOL(samples.back().x_m, 8333.33, 1.0,
                                  "60 s travel = 8333 m");

        // Doppler must start positive (train approaching gNB at x=3000)
        // and end negative (train past gNB).
        NS_TEST_ASSERT_MSG_GT(samples.front().doppler_hz, 0.0,
                              "approach: Doppler > 0");
        NS_TEST_ASSERT_MSG_LT(samples.back().doppler_hz, 0.0,
                              "recede: Doppler < 0");

        // The pass-by happens at t = 3000 / 138.89 = 21.6 s. There must
        // be a sample with |Doppler| smaller than the start, sandwiched
        // between two strictly larger samples (zero-crossing region).
        size_t minIdx = 0;
        double minAbs = std::abs(samples.front().doppler_hz);
        for (size_t i = 0; i < samples.size(); ++i)
        {
            if (std::abs(samples[i].doppler_hz) < minAbs)
            {
                minAbs = std::abs(samples[i].doppler_hz);
                minIdx = i;
            }
        }
        NS_TEST_ASSERT_MSG_GT(minIdx, 0u,
                              "pass-by sample is not the first sample");
        NS_TEST_ASSERT_MSG_LT(minIdx, samples.size() - 1,
                              "pass-by sample is not the last sample");
        // The true pass-by Doppler is 0, but at 1 Hz sampling and 500
        // km/h, the train moves 139 m between ticks, so the nearest
        // sample lands several hundred metres off the perpendicular and
        // the min |Doppler| is in the few-kHz range. Assert it is
        // substantially smaller than the starting Doppler.
        NS_TEST_ASSERT_MSG_LT(minAbs,
                              0.5 * std::abs(samples.front().doppler_hz),
                              "min |Doppler| < 50% of start |Doppler|");
        // Sample at minIdx should be near t=21..22 s.
        NS_TEST_ASSERT_MSG_GT(samples[minIdx].t_s, 19.0, "pass-by t > 19 s");
        NS_TEST_ASSERT_MSG_LT(samples[minIdx].t_s, 24.0, "pass-by t < 24 s");

        Simulator::Destroy();
    }
};

// ---------------------------------------------------------------------------
// Roadmap §4.4.10 — RL load-balancing hooks for ISL routing
// ---------------------------------------------------------------------------

/// Build a router with N candidates per layer at deterministic positions.
/// The "max elevation" LEO candidate is the one closest to directly
/// overhead the previous hop.
static Ptr<MultiLayerRouter>
MakeLayeredRouter()
{
    Ptr<MultiLayerRouter> router = CreateObject<MultiLayerRouter>();
    // 3 LEO candidates: index 0 low-elev (far east), index 1 directly
    // overhead (max elev), index 2 low-elev (far west).
    Ptr<ConstantPositionMobilityModel> leoEast =
        CreateObject<ConstantPositionMobilityModel>();
    leoEast->SetPosition(Vector{2.0e6, 0, 550000.0});
    Ptr<ConstantPositionMobilityModel> leoOverhead =
        CreateObject<ConstantPositionMobilityModel>();
    leoOverhead->SetPosition(Vector{0, 0, 550000.0});
    Ptr<ConstantPositionMobilityModel> leoWest =
        CreateObject<ConstantPositionMobilityModel>();
    leoWest->SetPosition(Vector{-2.0e6, 0, 550000.0});
    router->AddNode(SaginLayer::Leo, leoEast);
    router->AddNode(SaginLayer::Leo, leoOverhead);
    router->AddNode(SaginLayer::Leo, leoWest);
    return router;
}

/// Default scorer (no callback installed) must pick the directly-overhead
/// LEO (max elevation) — preserves legacy behaviour.
class RouterDefaultScorerTest : public TestCase
{
  public:
    RouterDefaultScorerTest()
        : TestCase("§4.4.10: default scorer picks max-elevation LEO")
    {
    }

    void DoRun() override
    {
        Ptr<MultiLayerRouter> router = MakeLayeredRouter();
        Ptr<ConstantPositionMobilityModel> src =
            CreateObject<ConstantPositionMobilityModel>();
        src->SetPosition(Vector{0, 0, 0});

        auto path = router->Route(src);
        NS_TEST_ASSERT_MSG_EQ(path.size(), 2u, "ground + LEO hop");
        // The overhead LEO at (0,0,550e3) gives ~90 deg elevation.
        NS_TEST_ASSERT_MSG_GT(path.back().elevationDeg, 80.0,
                              "default picks the overhead (max-elev) LEO");
        NS_TEST_ASSERT_MSG_EQ(router->GetRoutesEvaluated(), 1u, "1 route");
        NS_TEST_ASSERT_MSG_EQ(router->GetCandidatesScored(), 3u,
                              "3 LEO candidates scored");
    }
};

/// A custom scorer that prefers *minimum* range (here: the farthest LEO,
/// to make the override obvious vs the default) must override the choice.
class RouterCustomScorerOverridesTest : public TestCase
{
  public:
    RouterCustomScorerOverridesTest()
        : TestCase("§4.4.10: custom scorer overrides default hop choice")
    {
    }

    void DoRun() override
    {
        Ptr<MultiLayerRouter> router = MakeLayeredRouter();
        // Scorer rewards LARGER range — the opposite of the elevation
        // heuristic — so the far-east or far-west LEO should win.
        router->SetScorer([](const SaginHopCandidate& c) {
            return c.rangeM;
        });

        Ptr<ConstantPositionMobilityModel> src =
            CreateObject<ConstantPositionMobilityModel>();
        src->SetPosition(Vector{0, 0, 0});

        auto path = router->Route(src);
        NS_TEST_ASSERT_MSG_EQ(path.size(), 2u, "ground + LEO hop");
        // The farthest LEO is at slant range sqrt(2e6^2 + 550e3^2) ≈ 2.07e6.
        // The overhead is only 550e3. So custom scorer must pick a far one.
        NS_TEST_ASSERT_MSG_GT(path.back().rangeM, 1.5e6,
                              "custom scorer picks the farthest LEO");
        NS_TEST_ASSERT_MSG_LT(path.back().elevationDeg, 30.0,
                              "the chosen LEO is low-elevation, not overhead");
    }
};

/// The scorer must receive the caller-supplied observation vector on every
/// candidate. We assert the observation round-trips and that the scorer is
/// invoked exactly once per candidate.
class RouterObservationPassThroughTest : public TestCase
{
  public:
    RouterObservationPassThroughTest()
        : TestCase("§4.4.10: scorer receives the RL observation each call")
    {
    }

    void DoRun() override
    {
        Ptr<MultiLayerRouter> router = MakeLayeredRouter();
        const std::vector<double> obs = {0.1, 0.2, 0.3};
        router->SetObservation(obs);

        uint32_t calls = 0;
        bool obsOk = true;
        router->SetScorer([&](const SaginHopCandidate& c) {
            ++calls;
            if (c.observation.size() != 3 ||
                std::abs(c.observation[0] - 0.1) > 1e-9 ||
                std::abs(c.observation[2] - 0.3) > 1e-9)
            {
                obsOk = false;
            }
            return c.elevationDeg;
        });

        Ptr<ConstantPositionMobilityModel> src =
            CreateObject<ConstantPositionMobilityModel>();
        src->SetPosition(Vector{0, 0, 0});
        router->Route(src);

        NS_TEST_ASSERT_MSG_EQ(calls, 3u, "scorer invoked once per candidate");
        NS_TEST_ASSERT_MSG_EQ(obsOk, true, "observation round-trips intact");
    }
};

/// Per-layer scorers must win over the global scorer for their layer only.
class RouterPerLayerScorerTest : public TestCase
{
  public:
    RouterPerLayerScorerTest()
        : TestCase("§4.4.10: per-layer scorer overrides global for one layer")
    {
    }

    void DoRun() override
    {
        Ptr<MultiLayerRouter> router = CreateObject<MultiLayerRouter>();
        // 2 HAPS, 2 LEO.
        Ptr<ConstantPositionMobilityModel> hapsA =
            CreateObject<ConstantPositionMobilityModel>();
        hapsA->SetPosition(Vector{0, 0, 20000.0}); // overhead, max elev
        Ptr<ConstantPositionMobilityModel> hapsB =
            CreateObject<ConstantPositionMobilityModel>();
        hapsB->SetPosition(Vector{500000.0, 0, 20000.0}); // far
        router->AddNode(SaginLayer::Haps, hapsA);
        router->AddNode(SaginLayer::Haps, hapsB);

        Ptr<ConstantPositionMobilityModel> leoA =
            CreateObject<ConstantPositionMobilityModel>();
        leoA->SetPosition(Vector{0, 0, 550000.0}); // overhead, max elev
        Ptr<ConstantPositionMobilityModel> leoB =
            CreateObject<ConstantPositionMobilityModel>();
        leoB->SetPosition(Vector{3.0e6, 0, 550000.0}); // far
        router->AddNode(SaginLayer::Leo, leoA);
        router->AddNode(SaginLayer::Leo, leoB);

        // Global scorer = max elevation (default behaviour).
        // Per-layer LEO scorer = max range (prefers the far LEO).
        router->SetScorer([](const SaginHopCandidate& c) {
            return c.elevationDeg;
        });
        router->SetLayerScorer(SaginLayer::Leo,
                               [](const SaginHopCandidate& c) {
                                   return c.rangeM;
                               });

        Ptr<ConstantPositionMobilityModel> src =
            CreateObject<ConstantPositionMobilityModel>();
        src->SetPosition(Vector{0, 0, 0});
        auto path = router->Route(src);

        NS_TEST_ASSERT_MSG_EQ(path.size(), 3u, "ground + HAPS + LEO");
        // HAPS hop uses global (max elev) → overhead HAPS (high elev).
        NS_TEST_ASSERT_MSG_GT(path[1].elevationDeg, 80.0,
                              "HAPS hop uses global max-elev scorer");
        // LEO hop uses per-layer (max range) → far LEO (low elev).
        NS_TEST_ASSERT_MSG_GT(path[2].rangeM, 2.0e6,
                              "LEO hop uses per-layer max-range scorer");
    }
};

/// ClearScorer restores the default greedy behaviour after a custom scorer.
class RouterClearScorerTest : public TestCase
{
  public:
    RouterClearScorerTest()
        : TestCase("§4.4.10: ClearScorer restores default max-elev choice")
    {
    }

    void DoRun() override
    {
        Ptr<MultiLayerRouter> router = MakeLayeredRouter();
        router->SetScorer([](const SaginHopCandidate& c) {
            return c.rangeM;
        });
        Ptr<ConstantPositionMobilityModel> src =
            CreateObject<ConstantPositionMobilityModel>();
        src->SetPosition(Vector{0, 0, 0});

        auto far = router->Route(src);
        NS_TEST_ASSERT_MSG_GT(far.back().rangeM, 1.5e6, "custom: far LEO");

        router->ClearScorer();
        auto overhead = router->Route(src);
        NS_TEST_ASSERT_MSG_GT(overhead.back().elevationDeg, 80.0,
                              "after clear: back to overhead LEO");
    }
};

/// Simulator-driven: an "RL agent" rewrites the observation + a scorer that
/// switches its preferred LEO based on a scalar in the observation, every
/// 1 s for 10 s. Verify the chosen LEO tracks the observation.
class RouterSimulatorTimeRlTest : public TestCase
{
  public:
    RouterSimulatorTimeRlTest()
        : TestCase("§4.4.10: Simulator-driven RL observation steers routing")
    {
    }

    struct Pick
    {
        double t;
        double obsSelector;
        double chosenRangeM;
    };

    static void Step(Ptr<MultiLayerRouter> router,
                     Ptr<MobilityModel> src,
                     double selector,
                     std::vector<Pick>* out)
    {
        // selector > 0.5 → prefer far LEO (max range); else prefer overhead.
        router->SetObservation({selector});
        auto path = router->Route(src);
        out->push_back({Simulator::Now().GetSeconds(),
                        selector,
                        path.back().rangeM});
    }

    void DoRun() override
    {
        Ptr<MultiLayerRouter> router = MakeLayeredRouter();
        // Scorer reads observation[0]: >0.5 → max range; else → max elev.
        router->SetScorer([](const SaginHopCandidate& c) {
            const double sel =
                c.observation.empty() ? 0.0 : c.observation[0];
            return (sel > 0.5) ? c.rangeM : c.elevationDeg;
        });

        Ptr<ConstantPositionMobilityModel> src =
            CreateObject<ConstantPositionMobilityModel>();
        src->SetPosition(Vector{0, 0, 0});

        std::vector<Pick> picks;
        const double selectors[] = {0.0, 0.9, 0.0, 0.9, 0.0,
                                     0.9, 0.0, 0.9, 0.0, 0.9};
        for (int i = 0; i < 10; ++i)
        {
            Simulator::Schedule(Seconds(i + 1),
                                &Step,
                                router,
                                Ptr<MobilityModel>(src),
                                selectors[i],
                                &picks);
        }
        Simulator::Stop(Seconds(11));
        Simulator::Run();

        NS_TEST_ASSERT_MSG_EQ(picks.size(), 10u, "10 routing decisions");
        for (const auto& p : picks)
        {
            if (p.obsSelector > 0.5)
            {
                NS_TEST_ASSERT_MSG_GT(p.chosenRangeM, 1.5e6,
                                      "selector>0.5 → far LEO");
            }
            else
            {
                NS_TEST_ASSERT_MSG_LT(p.chosenRangeM, 1.0e6,
                                      "selector<0.5 → overhead LEO");
            }
        }
        Simulator::Destroy();
    }
};

// ---------------------------------------------------------------------------
// Roadmap §4.4.7 — cross-layer slice-aware routing
// ---------------------------------------------------------------------------

/// Build a router with a HAPS (20 km, overhead), an off-axis LEO (550 km
/// altitude, ~61° elevation), and a GEO-altitude node (35 786 km, directly
/// overhead) placed in the LEO layer. The GEO has the higher elevation, so
/// latency-tolerant slices that allow GEO prefer it; latency-bound slices
/// fall back to the off-axis LEO; very-tight slices stop at the HAPS.
static Ptr<MultiLayerRouter>
MakeSliceTestRouter()
{
    Ptr<MultiLayerRouter> router = CreateObject<MultiLayerRouter>();
    Ptr<ConstantPositionMobilityModel> haps =
        CreateObject<ConstantPositionMobilityModel>();
    haps->SetPosition(Vector{0, 0, 20000.0}); // 20 km, overhead
    router->AddNode(SaginLayer::Haps, haps);

    Ptr<ConstantPositionMobilityModel> leo =
        CreateObject<ConstantPositionMobilityModel>();
    leo->SetPosition(Vector{300000.0, 0, 550000.0}); // off-axis, ~61° elev,
                                                      // ~627 km slant range
    router->AddNode(SaginLayer::Leo, leo);

    Ptr<ConstantPositionMobilityModel> geo =
        CreateObject<ConstantPositionMobilityModel>();
    geo->SetPosition(Vector{0, 0, 35786000.0}); // GEO altitude, overhead
    router->AddNode(SaginLayer::Leo, geo);
    return router;
}

/// QFI → S-NSSAI mapping follows the default 5QI bands.
class SliceQfiMappingTest : public TestCase
{
  public:
    SliceQfiMappingTest()
        : TestCase("§4.4.7: QFI bands map to URLLC / eMBB / mMTC")
    {
    }

    void DoRun() override
    {
        Ptr<SaginSliceRouter> sr = CreateObject<SaginSliceRouter>();
        // QFI 1..4 → URLLC.
        NS_TEST_ASSERT_MSG_EQ(
            static_cast<int>(sr->QfiToSnssai(1).sst),
            static_cast<int>(ntnslice::SliceSst::Urllc),
            "QFI 1 → URLLC");
        NS_TEST_ASSERT_MSG_EQ(
            static_cast<int>(sr->QfiToSnssai(4).sst),
            static_cast<int>(ntnslice::SliceSst::Urllc),
            "QFI 4 → URLLC");
        // QFI 5..9 → eMBB.
        NS_TEST_ASSERT_MSG_EQ(
            static_cast<int>(sr->QfiToSnssai(7).sst),
            static_cast<int>(ntnslice::SliceSst::Embb),
            "QFI 7 → eMBB");
        // QFI 10..63 → mMTC.
        NS_TEST_ASSERT_MSG_EQ(
            static_cast<int>(sr->QfiToSnssai(50).sst),
            static_cast<int>(ntnslice::SliceSst::Mmtc),
            "QFI 50 → mMTC");
        // Custom override.
        sr->SetQfiSlice(7, {ntnslice::SliceSst::Urllc, ntnslice::kSdUnset});
        NS_TEST_ASSERT_MSG_EQ(
            static_cast<int>(sr->QfiToSnssai(7).sst),
            static_cast<int>(ntnslice::SliceSst::Urllc),
            "QFI 7 overridden to URLLC");
    }
};

/// mMTC (1000 ms budget, allowGeo=true) reaches the GEO node;
/// eMBB (50 ms budget) refuses GEO (119 ms one-way) and lands on LEO.
class SliceMmtcVsEmbbLayerTest : public TestCase
{
  public:
    SliceMmtcVsEmbbLayerTest()
        : TestCase("§4.4.7: mMTC accepts GEO, eMBB caps at LEO by latency")
    {
    }

    void DoRun() override
    {
        Ptr<SaginSliceRouter> sr = CreateObject<SaginSliceRouter>();
        sr->SetRouter(MakeSliceTestRouter());

        Ptr<ConstantPositionMobilityModel> src =
            CreateObject<ConstantPositionMobilityModel>();
        src->SetPosition(Vector{0, 0, 0});

        // mMTC.
        auto mmtc = sr->RouteForSlice(
            src, {ntnslice::SliceSst::Mmtc, ntnslice::kSdUnset});
        NS_TEST_ASSERT_MSG_GT(mmtc.size(), 1u, "mMTC routes beyond ground");
        NS_TEST_ASSERT_MSG_GT(mmtc.back().node->GetPosition().z,
                              30000000.0,
                              "mMTC reaches the GEO node");

        // eMBB.
        auto embb = sr->RouteForSlice(
            src, {ntnslice::SliceSst::Embb, ntnslice::kSdUnset});
        NS_TEST_ASSERT_MSG_GT(embb.size(), 1u, "eMBB routes beyond ground");
        NS_TEST_ASSERT_MSG_EQ_TOL(embb.back().node->GetPosition().z,
                                   550000.0,
                                   1.0,
                                   "eMBB caps at the LEO node (GEO too slow)");
    }
};

/// A tight custom URLLC slice (1 ms budget, allowGeo=false) refuses BOTH
/// GEO (119 ms) and LEO (1.83 ms), stopping at the HAPS hop (0.067 ms).
class SliceTightUrllcStopsAtHapsTest : public TestCase
{
  public:
    SliceTightUrllcStopsAtHapsTest()
        : TestCase("§4.4.7: tight 1 ms URLLC stops at HAPS, refusing LEO/GEO")
    {
    }

    void DoRun() override
    {
        Ptr<SaginSliceRouter> sr = CreateObject<SaginSliceRouter>();
        sr->SetRouter(MakeSliceTestRouter());

        // Register a tight URLLC variant: 1 ms budget, no GEO.
        ntnslice::SliceProfile tight = ntnslice::DefaultUrllc(7);
        tight.latencyBudgetMs = 1.0;
        sr->AddSlice(tight);

        Ptr<ConstantPositionMobilityModel> src =
            CreateObject<ConstantPositionMobilityModel>();
        src->SetPosition(Vector{0, 0, 0});

        auto path = sr->RouteForSlice(src, tight.snssai);
        NS_TEST_ASSERT_MSG_EQ(path.size(), 2u,
                              "ground + HAPS only (LEO/GEO refused)");
        NS_TEST_ASSERT_MSG_EQ(static_cast<int>(path.back().layer),
                              static_cast<int>(SaginLayer::Haps),
                              "tight URLLC stops at HAPS");
        NS_TEST_ASSERT_MSG_EQ_TOL(path.back().node->GetPosition().z,
                                   20000.0,
                                   1.0,
                                   "HAPS altitude 20 km");
    }
};

/// Default 5 ms URLLC (allowGeo=false) accepts LEO (1.83 ms) but refuses
/// GEO — lands on LEO, demonstrating the GEO mode-skip independent of the
/// latency gate.
class SliceDefaultUrllcSkipsGeoTest : public TestCase
{
  public:
    SliceDefaultUrllcSkipsGeoTest()
        : TestCase("§4.4.7: default 5 ms URLLC uses LEO, mode-skips GEO")
    {
    }

    void DoRun() override
    {
        Ptr<SaginSliceRouter> sr = CreateObject<SaginSliceRouter>();
        sr->SetRouter(MakeSliceTestRouter());

        Ptr<ConstantPositionMobilityModel> src =
            CreateObject<ConstantPositionMobilityModel>();
        src->SetPosition(Vector{0, 0, 0});

        auto path = sr->RouteForSlice(
            src, {ntnslice::SliceSst::Urllc, ntnslice::kSdUnset});
        NS_TEST_ASSERT_MSG_EQ_TOL(path.back().node->GetPosition().z,
                                   550000.0,
                                   1.0,
                                   "default URLLC lands on LEO (GEO skipped)");
    }
};

/// Simulator-driven: route three QFIs (1=URLLC, 7=eMBB, 50=mMTC) once per
/// second across 9 s. Each must consistently pick its slice-appropriate
/// top layer. Verifies the slice router is re-entrant under the simulator
/// and doesn't leak scorer state between calls.
class SliceSimulatorTimeTest : public TestCase
{
  public:
    SliceSimulatorTimeTest()
        : TestCase("§4.4.7: Simulator-driven per-QFI routing is stable + isolated")
    {
    }

    struct Pick
    {
        uint8_t qfi;
        double topAltM;
    };

    static void RouteOnce(Ptr<SaginSliceRouter> sr,
                          Ptr<MobilityModel> src,
                          uint8_t qfi,
                          std::vector<Pick>* out)
    {
        auto path = sr->RouteForQfi(src, qfi);
        out->push_back({qfi, path.back().node->GetPosition().z});
    }

    void DoRun() override
    {
        Ptr<SaginSliceRouter> sr = CreateObject<SaginSliceRouter>();
        sr->SetRouter(MakeSliceTestRouter());

        Ptr<ConstantPositionMobilityModel> src =
            CreateObject<ConstantPositionMobilityModel>();
        src->SetPosition(Vector{0, 0, 0});

        std::vector<Pick> picks;
        const uint8_t qfis[] = {1, 7, 50, 1, 7, 50, 1, 7, 50};
        for (int i = 0; i < 9; ++i)
        {
            Simulator::Schedule(Seconds(i + 1),
                                &RouteOnce,
                                sr,
                                Ptr<MobilityModel>(src),
                                qfis[i],
                                &picks);
        }
        Simulator::Stop(Seconds(10));
        Simulator::Run();

        NS_TEST_ASSERT_MSG_EQ(picks.size(), 9u, "9 routing decisions");
        for (const auto& p : picks)
        {
            if (p.qfi == 50) // mMTC → GEO
            {
                NS_TEST_ASSERT_MSG_GT(p.topAltM, 30000000.0,
                                      "mMTC QFI 50 reaches GEO every time");
            }
            else // URLLC(1) + eMBB(7) → LEO
            {
                NS_TEST_ASSERT_MSG_EQ_TOL(p.topAltM, 550000.0, 1.0,
                                          "URLLC/eMBB land on LEO every time");
            }
        }
        Simulator::Destroy();
    }
};

// ============================================================================
//  Roadmap §4.4.8 — HAPS trajectory CSV ingest
// ============================================================================

namespace
{

std::string
MakeHapsCsv(bool include_optional)
{
    const std::string path = "/tmp/haps-trajectory-test.csv";
    std::ofstream f(path);
    if (include_optional)
    {
        f << "time_s,lat_deg,lon_deg,alt_m,heading_deg,speed_mps,"
             "platform_id\n";
        f << "0,40.0,-3.0,20000,90.0,25.0,kea-mk1\n";
        f << "10,40.001,-2.999,20010,92.0,27.0,kea-mk1\n";
        f << "20,40.002,-2.997,20020,95.0,28.0,kea-mk1\n";
        f << "30,40.003,-2.994,20030,100.0,30.0,kea-mk1\n";
    }
    else
    {
        f << "time_s,lat_deg,lon_deg,alt_m\n";
        f << "0,40.0,-3.0,20000\n";
        f << "10,40.001,-2.999,20010\n";
        f << "20,40.002,-2.997,20020\n";
    }
    return path;
}

} // namespace

class HapsTrajectoryImportParseTest : public TestCase
{
  public:
    HapsTrajectoryImportParseTest()
        : TestCase("§4.4.8: HAPS CSV parses header and rows")
    {
    }

    void DoRun() override
    {
        const auto path = MakeHapsCsv(true);
        sagin::HapsTrajectoryImporter imp;
        const auto traces = imp.LoadCsv(path);
        NS_TEST_ASSERT_MSG_EQ(traces.size(), 1u, "1 platform");
        const auto& tr = traces.at("kea-mk1");
        NS_TEST_EXPECT_MSG_EQ(tr.samples.size(), 4u, "4 samples");
        NS_TEST_EXPECT_MSG_EQ(tr.samples.front().has_heading,
                               true,
                               "heading present");
        NS_TEST_EXPECT_MSG_EQ(tr.samples.front().has_speed,
                               true,
                               "speed present");
        NS_TEST_EXPECT_MSG_EQ_TOL(tr.samples.front().alt_m,
                                    20000.0,
                                    1e-6,
                                    "alt parsed");
        NS_TEST_EXPECT_MSG_EQ_TOL(tr.samples.back().alt_m,
                                    20030.0,
                                    1e-6,
                                    "last alt parsed");
        NS_TEST_EXPECT_MSG_EQ(imp.LastRowsRead(),
                               4u,
                               "rows read");
        NS_TEST_EXPECT_MSG_EQ(imp.LastRowsSkipped(),
                               0u,
                               "no skips");
        std::remove(path.c_str());

        // Minimal header (no heading / speed / platform_id).
        const auto path2 = MakeHapsCsv(false);
        const auto traces2 = imp.LoadCsv(path2);
        NS_TEST_ASSERT_MSG_EQ(traces2.size(), 1u, "default key");
        const auto& tr2 = traces2.at("haps");
        NS_TEST_EXPECT_MSG_EQ(tr2.samples.front().has_heading,
                               false,
                               "no heading");
        NS_TEST_EXPECT_MSG_EQ(tr2.samples.front().has_speed,
                               false,
                               "no speed");
        std::remove(path2.c_str());
    }
};

class HapsTrajectoryMalformedTest : public TestCase
{
  public:
    HapsTrajectoryMalformedTest()
        : TestCase("§4.4.8: HAPS CSV skips malformed rows")
    {
    }

    void DoRun() override
    {
        const std::string path = "/tmp/haps-malformed.csv";
        std::ofstream f(path);
        f << "time_s,lat_deg,lon_deg,alt_m\n";
        f << "0,40.0,-3.0,20000\n";
        f << "5,xx,yy,zz\n";
        f << "10,40.001,-2.999\n";
        f << "15,40.002,-2.997,20020\n";
        f.close();
        sagin::HapsTrajectoryImporter imp;
        const auto traces = imp.LoadCsv(path);
        NS_TEST_ASSERT_MSG_EQ(traces.size(), 1u, "1 platform");
        NS_TEST_EXPECT_MSG_EQ(traces.at("haps").samples.size(),
                               2u,
                               "2 good rows");
        NS_TEST_EXPECT_MSG_EQ(imp.LastRowsRead(), 4u, "rows read");
        NS_TEST_EXPECT_MSG_EQ(imp.LastRowsSkipped(),
                               2u,
                               "2 skipped");
        std::remove(path.c_str());
    }
};

class HapsTrajectoryInterpolationTest : public TestCase
{
  public:
    HapsTrajectoryInterpolationTest()
        : TestCase("§4.4.8: HAPS interpolation between waypoints")
    {
    }

    void DoRun() override
    {
        sagin::HapsTrajectoryTrace tr;
        tr.platform_id = "test";
        sagin::HapsTrajectorySample a;
        a.time_s = 0.0;
        a.lat_deg = 40.0;
        a.lon_deg = -3.0;
        a.alt_m = 20000.0;
        a.has_heading = true;
        a.heading_deg = 350.0;
        a.has_speed = true;
        a.speed_mps = 20.0;
        sagin::HapsTrajectorySample b;
        b.time_s = 10.0;
        b.lat_deg = 41.0;
        b.lon_deg = -2.0;
        b.alt_m = 21000.0;
        b.has_heading = true;
        b.heading_deg = 10.0;
        b.has_speed = true;
        b.speed_mps = 40.0;
        tr.samples.push_back(a);
        tr.samples.push_back(b);

        const auto mid = tr.InterpolateAt(5.0);
        NS_TEST_EXPECT_MSG_EQ_TOL(mid.lat_deg, 40.5, 1e-9, "lat mid");
        NS_TEST_EXPECT_MSG_EQ_TOL(mid.lon_deg, -2.5, 1e-9, "lon mid");
        NS_TEST_EXPECT_MSG_EQ_TOL(mid.alt_m, 20500.0, 1e-9, "alt mid");
        NS_TEST_EXPECT_MSG_EQ_TOL(mid.speed_mps,
                                    30.0,
                                    1e-9,
                                    "speed mid");
        const double wrap_dist =
            std::min(mid.heading_deg, 360.0 - mid.heading_deg);
        NS_TEST_EXPECT_MSG_LT(wrap_dist,
                                1e-6,
                                "heading wraps via 0");

        const auto before = tr.InterpolateAt(-5.0);
        NS_TEST_EXPECT_MSG_EQ_TOL(before.lat_deg,
                                    40.0,
                                    1e-9,
                                    "pre-boundary clamp");
        const auto after = tr.InterpolateAt(50.0);
        NS_TEST_EXPECT_MSG_EQ_TOL(after.lat_deg,
                                    41.0,
                                    1e-9,
                                    "post-boundary clamp");
    }
};

class HapsTrajectoryMobilitySimulatorTest : public TestCase
{
  public:
    HapsTrajectoryMobilitySimulatorTest()
        : TestCase("§4.4.8: HAPS mobility advances under Simulator")
    {
    }

    void DoRun() override
    {
        sagin::HapsTrajectoryTrace tr;
        tr.platform_id = "test";
        const double lat0 = 45.0;
        const double lon0 = 7.0;
        const double cosLat = std::cos(lat0 * M_PI / 180.0);
        const double R = 6371000.0;
        for (int i = 0; i <= 60; ++i)
        {
            sagin::HapsTrajectorySample s;
            s.time_s = static_cast<double>(i);
            const double east_m = (i / 60.0) * 1000.0;
            s.lat_deg = lat0;
            s.lon_deg =
                lon0 + (east_m / (R * cosLat)) * 180.0 / M_PI;
            s.alt_m = 20000.0;
            tr.samples.push_back(s);
        }
        auto m = CreateObject<sagin::HapsTrajectoryMobilityModel>();
        m->SetTrace(tr);
        m->SetReference(lat0, lon0, 20000.0);

        std::vector<Vector> pos_samples;
        for (uint32_t k = 0; k <= 60; ++k)
        {
            Simulator::Schedule(
                Seconds(k),
                [m, &pos_samples] {
                    pos_samples.push_back(m->GetPosition());
                });
        }
        Simulator::Stop(Seconds(60.5));
        Simulator::Run();
        Simulator::Destroy();

        NS_TEST_ASSERT_MSG_EQ(pos_samples.size(),
                               61u,
                               "61 samples");
        NS_TEST_EXPECT_MSG_EQ_TOL(pos_samples.front().x,
                                    0.0,
                                    1.0,
                                    "starts at ref");
        NS_TEST_EXPECT_MSG_EQ_TOL(pos_samples.back().x,
                                    1000.0,
                                    1.0,
                                    "ends 1 km east");
        NS_TEST_EXPECT_MSG_EQ_TOL(pos_samples[30].x,
                                    500.0,
                                    1.0,
                                    "halfway");
        for (const auto& p : pos_samples)
        {
            NS_TEST_EXPECT_MSG_EQ_TOL(p.z, 0.0, 1.0, "z=0");
        }
    }
};

// ---------------------------------------------------------------------------
// Audit G1: SaginA2gPropagationLossModel must draw LOS + shadow fading per
// node-pair (wired through DoAssignStreams) and CACHE the realisation — a link
// must not get fresh white noise on every DoCalcRxPower call.
// ---------------------------------------------------------------------------
class A2gStochasticFadingTest : public TestCase
{
  public:
    A2gStochasticFadingTest()
        : TestCase("A2G prop model: cached per-pair stochastic LOS + shadow fading")
    {
    }

    void DoRun() override
    {
        Ptr<SaginA2gPropagationLossModel> loss =
            CreateObject<SaginA2gPropagationLossModel>();
        loss->SetScenario(A2gScenario::UMa_AV);
        loss->SetFrequencyGHz(2.0);

        // DoAssignStreams must consume exactly 2 streams (LOS + shadowing) —
        // proof the randomness is real and stream-controlled (was 0 before).
        const int64_t used = loss->AssignStreams(100);
        NS_TEST_ASSERT_MSG_EQ(used, 2, "must consume 2 RNG streams");

        Ptr<ConstantPositionMobilityModel> gnb =
            CreateObject<ConstantPositionMobilityModel>();
        gnb->SetPosition(Vector{0, 0, 25.0});
        Ptr<ConstantPositionMobilityModel> uav =
            CreateObject<ConstantPositionMobilityModel>();
        uav->SetPosition(Vector{300.0, 0, 80.0});

        // (1) Caching: repeated calls on the SAME stationary pair must return
        //     an IDENTICAL value — no per-call white noise (audit gap S6).
        const double first = loss->CalcRxPower(30.0, gnb, uav);
        for (int i = 0; i < 20; ++i)
        {
            double rep = loss->CalcRxPower(30.0, gnb, uav);
            NS_TEST_ASSERT_MSG_EQ_TOL(rep, first, 1e-9,
                                      "stationary pair must be cached, not re-drawn");
        }

        // The model caches by node-pair identity, so every pair below must be a
        // distinct, still-alive object — hold them in vectors so addresses are
        // not recycled (which would collide the pointer-keyed cache).
        const int N = 400;
        // A2gChannelTr36777 deterministic LOS/NLOS references at the geometry
        // used below (gNB 25 m, UAV at (2000,0,60), d2D=2 km).
        const double d3d = std::sqrt(2000.0 * 2000.0 + 35.0 * 35.0);
        const double losDet =
            A2gChannelTr36777::PathLossDb(A2gScenario::UMa_AV, A2gLink::LOS, d3d, 2.0, 60.0);
        const double nlosDet =
            A2gChannelTr36777::PathLossDb(A2gScenario::UMa_AV, A2gLink::NLOS, d3d, 2.0, 60.0);
        NS_TEST_ASSERT_MSG_GT(nlosDet - losDet, 10.0, "LOS/NLOS must be separable");
        // Free-space reference exactly as the model computes it, and the two
        // discrete rx values the fading-OFF model must land on.
        const double fspl = 20.0 * std::log10(d3d) + 20.0 * std::log10(2e9) - 147.55;
        // SAGIN-5: the excess is SIGNED. This used to carry a
        // `std::max(0.0, ...)` mirroring the clamp that finding removed from
        // the model. It passed only because the excess happens to be positive
        // at this 2 km geometry, so it was a stale expectation waiting to
        // wrongly certify a re-introduced clamp at any shorter range.
        const double rxLosExpect = 30.0 - (losDet - fspl);
        const double rxNlosExpect = 30.0 - (nlosDet - fspl);

        // (2) Stochastic LOS mix (shadow fading OFF): with fading disabled the
        //     received power collapses onto EXACTLY the two deterministic values
        //     (LOS-excess or NLOS-excess). Both classes must appear — proving a
        //     genuine per-pair Bernoulli draw, not a static per-run attribute.
        Ptr<SaginA2gPropagationLossModel> losMix =
            CreateObject<SaginA2gPropagationLossModel>();
        losMix->SetScenario(A2gScenario::UMa_AV);
        losMix->SetFrequencyGHz(2.0);
        losMix->SetAttribute("ApplyShadowFading", BooleanValue(false));
        losMix->AssignStreams(200);

        std::vector<Ptr<ConstantPositionMobilityModel>> gs, us;
        gs.reserve(N);
        us.reserve(N);
        int nLos = 0;
        int nNlos = 0;
        for (int i = 0; i < N; ++i)
        {
            Ptr<ConstantPositionMobilityModel> g =
                CreateObject<ConstantPositionMobilityModel>();
            g->SetPosition(Vector{0, 0, 25.0});
            Ptr<ConstantPositionMobilityModel> u =
                CreateObject<ConstantPositionMobilityModel>();
            u->SetPosition(Vector{2000.0, 0, 60.0});
            gs.push_back(g);
            us.push_back(u);
            // With fading off, rx collapses onto exactly rxLosExpect or
            // rxNlosExpect. Classify by nearest.
            double rx = losMix->CalcRxPower(30.0, g, u);
            if (std::abs(rx - rxLosExpect) < std::abs(rx - rxNlosExpect))
            {
                NS_TEST_ASSERT_MSG_EQ_TOL(rx, rxLosExpect, 1e-6, "LOS rx exact");
                ++nLos;
            }
            else
            {
                NS_TEST_ASSERT_MSG_EQ_TOL(rx, rxNlosExpect, 1e-6, "NLOS rx exact");
                ++nNlos;
            }
        }
        NS_TEST_ASSERT_MSG_GT(nLos, 0, "some pairs must draw LOS");
        NS_TEST_ASSERT_MSG_GT(nNlos, 0, "some pairs must draw NLOS");

        // (3) Shadow fading spread (link FORCED NLOS, fading ON): rx must vary
        //     across distinct pairs purely from the Table B-1.2 shadowing draw.
        Ptr<SaginA2gPropagationLossModel> fadeOnly =
            CreateObject<SaginA2gPropagationLossModel>();
        fadeOnly->SetScenario(A2gScenario::UMa_AV);
        fadeOnly->SetFrequencyGHz(2.0);
        fadeOnly->SetLink(A2gLink::NLOS); // forces DrawStochasticLos = false
        fadeOnly->AssignStreams(400);

        std::vector<Ptr<ConstantPositionMobilityModel>> gs2, us2;
        double minRx = 1e9;
        double maxRx = -1e9;
        for (int i = 0; i < N; ++i)
        {
            Ptr<ConstantPositionMobilityModel> g =
                CreateObject<ConstantPositionMobilityModel>();
            g->SetPosition(Vector{0, 0, 25.0});
            Ptr<ConstantPositionMobilityModel> u =
                CreateObject<ConstantPositionMobilityModel>();
            u->SetPosition(Vector{2000.0, 0, 60.0});
            gs2.push_back(g);
            us2.push_back(u);
            double rx = fadeOnly->CalcRxPower(30.0, g, u);
            minRx = std::min(minRx, rx);
            maxRx = std::max(maxRx, rx);
        }
        NS_TEST_ASSERT_MSG_GT(maxRx - minRx, 3.0,
                              "shadow fading must spread Rx power across pairs");

        Simulator::Destroy();
    }
};


// ---------------------------------------------------------------------------
// Audit gap G3: the router's chosen path must ACTUATE the data plane. Build a
// real IPv4 forwarding topology with two parallel LEO relays:
//
//     SRC --- LEO-A --- GW --- SRV
//         \-- LEO-B --/
//
// The MultiLayerRouter chooses one LEO (greedy max-elevation); we bring that
// LEO's two interfaces UP and the other LEO's DOWN, then let global routing
// forward a UDP flow SRC->SRV. Per-LEO bytes are counted at the GW (MacRx) so
// we can assert the delivered bytes follow the ROUTER's choice: the LEO the
// router selected carries the flow; the LEO it rejected carries ~0. Flipping
// the geometry flips the choice and reroutes the data plane.
// ---------------------------------------------------------------------------
namespace
{
void
AddRxBytes(uint64_t* counter, Ptr<const Packet> p)
{
    *counter += p->GetSize();
}
} // namespace

class RouterGatesDataPlaneTest : public TestCase
{
  public:
    RouterGatesDataPlaneTest()
        : TestCase("G3: router's chosen path gates the data plane (off-route "
                   "hop delivers ~0; flipping the choice reroutes)")
    {
    }

    struct PhaseResult
    {
        double chosenLeoZ;   // altitude of the LEO the router picked
        bool routerPickedA;  // did the router select LEO-A?
        uint64_t carriedA;   // bytes GW received from LEO-A
        uint64_t carriedB;   // bytes GW received from LEO-B
        uint64_t deliveredSrv; // bytes the server received end-to-end
    };

    // Run one phase. `overheadIsA==true` places LEO-A directly overhead the
    // source (max elevation) and LEO-B far/low; false swaps them. Returns what
    // the router picked and what each path actually carried.
    PhaseResult RunPhase(bool overheadIsA)
    {
        NodeContainer nodes;
        nodes.Create(5); // 0=SRC 1=LEO-A 2=LEO-B 3=GW 4=SRV

        // Mobility: the router scores LEO elevation from the SRC position.
        auto src = CreateObject<ConstantPositionMobilityModel>();
        src->SetPosition(Vector{0, 0, 0});
        auto leoAMob = CreateObject<ConstantPositionMobilityModel>();
        auto leoBMob = CreateObject<ConstantPositionMobilityModel>();
        const Vector overhead{0, 0, 550000.0};      // ~90 deg elevation
        const Vector faraway{2.0e6, 0, 550000.0};   // ~15 deg elevation
        leoAMob->SetPosition(overheadIsA ? overhead : faraway);
        leoBMob->SetPosition(overheadIsA ? faraway : overhead);

        Ptr<MultiLayerRouter> router = CreateObject<MultiLayerRouter>();
        router->AddNode(SaginLayer::Leo, leoAMob);
        router->AddNode(SaginLayer::Leo, leoBMob);

        InternetStackHelper internet;
        internet.Install(nodes);

        PointToPointHelper p2p;
        p2p.SetDeviceAttribute("DataRate", DataRateValue(DataRate(uint64_t(50e6))));
        p2p.SetChannelAttribute("Delay", TimeValue(MilliSeconds(2)));
        Ipv4AddressHelper ipv4;

        // Build one link, return device container + a per-side RateErrorModel
        // on the receiving (index-1) device so we can gate it.
        struct Link
        {
            NetDeviceContainer dev;
            Ptr<RateErrorModel> emRx; // on dev.Get(1)
            Ptr<Ipv4> ipA, ipB;
            uint32_t ifA, ifB;
        };
        auto makeLink = [&](Ptr<Node> a, Ptr<Node> b, const char* subnet) -> Link {
            Link l;
            l.dev = p2p.Install(NodeContainer(a, b));
            l.emRx = CreateObject<RateErrorModel>();
            l.emRx->SetUnit(RateErrorModel::ERROR_UNIT_PACKET);
            l.emRx->SetRate(0.0);
            l.dev.Get(1)->SetAttribute("ReceiveErrorModel", PointerValue(l.emRx));
            ipv4.SetBase(Ipv4Address(subnet), "255.255.255.0");
            auto ifc = ipv4.Assign(l.dev);
            l.ipA = a->GetObject<Ipv4>();
            l.ipB = b->GetObject<Ipv4>();
            l.ifA = ifc.Get(0).second;
            l.ifB = ifc.Get(1).second;
            return l;
        };

        Link srcA = makeLink(nodes.Get(0), nodes.Get(1), "10.40.1.0"); // SRC-LEOA
        Link aGw = makeLink(nodes.Get(1), nodes.Get(3), "10.40.2.0");  // LEOA-GW
        Link srcB = makeLink(nodes.Get(0), nodes.Get(2), "10.40.3.0"); // SRC-LEOB
        Link bGw = makeLink(nodes.Get(2), nodes.Get(3), "10.40.4.0");  // LEOB-GW
        Link gwSrv = makeLink(nodes.Get(3), nodes.Get(4), "10.40.9.0"); // GW-SRV
        Ipv4Address srvAddr =
            nodes.Get(4)->GetObject<Ipv4>()->GetAddress(gwSrv.ifB, 0).GetLocal();

        // Count bytes the GW receives from each LEO (GW is side 1 of *-GW).
        uint64_t carriedA = 0;
        uint64_t carriedB = 0;
        aGw.dev.Get(1)->TraceConnectWithoutContext(
            "MacRx", MakeBoundCallback(&AddRxBytes, &carriedA));
        bGw.dev.Get(1)->TraceConnectWithoutContext(
            "MacRx", MakeBoundCallback(&AddRxBytes, &carriedB));

        // ---- ACTUATE THE ROUTER DECISION ONTO THE DATA PLANE ----
        auto path = router->Route(src);
        Ptr<MobilityModel> chosen = path.back().node;
        const bool aChosen = (chosen == leoAMob);
        // Bring the chosen LEO's two links UP + usable; the other DOWN.
        auto gate = [](Link& l, bool on) {
            l.emRx->SetRate(on ? 0.0 : 1.0);
            if (on)
            {
                l.ipA->SetUp(l.ifA);
                l.ipB->SetUp(l.ifB);
            }
            else
            {
                l.ipA->SetDown(l.ifA);
                l.ipB->SetDown(l.ifB);
            }
        };
        gate(srcA, aChosen);
        gate(aGw, aChosen);
        gate(srcB, !aChosen);
        gate(bGw, !aChosen);

        Ipv4GlobalRoutingHelper::PopulateRoutingTables();

        // ---- traffic: SRC -> SRV (stable address, reached over chosen LEO) ---
        const uint16_t port = 8000;
        OnOffHelper onoff("ns3::UdpSocketFactory",
                          InetSocketAddress(srvAddr, port));
        onoff.SetConstantRate(DataRate(uint64_t(4e6)), 1000);
        onoff.SetAttribute("StartTime", TimeValue(Seconds(1.0)));
        onoff.SetAttribute("StopTime", TimeValue(Seconds(6.0)));
        onoff.Install(nodes.Get(0));

        PacketSinkHelper sink("ns3::UdpSocketFactory",
                              InetSocketAddress(Ipv4Address::GetAny(), port));
        ApplicationContainer sinkApp = sink.Install(nodes.Get(4));
        sinkApp.Start(Seconds(0.0));
        sinkApp.Stop(Seconds(6.5));

        Simulator::Stop(Seconds(6.5));
        Simulator::Run();

        PhaseResult r;
        r.chosenLeoZ = chosen->GetPosition().z;
        r.routerPickedA = aChosen;
        r.carriedA = carriedA;
        r.carriedB = carriedB;
        r.deliveredSrv =
            DynamicCast<PacketSink>(sinkApp.Get(0))->GetTotalRx();
        Simulator::Destroy();
        return r;
    }

    void DoRun() override
    {
        // Phase A: LEO-A overhead -> router picks A -> data plane carries via A.
        PhaseResult a = RunPhase(true);
        NS_TEST_ASSERT_MSG_EQ(a.routerPickedA, true,
                              "phase A: router selects overhead LEO-A");
        NS_TEST_ASSERT_MSG_EQ_TOL(a.chosenLeoZ, 550000.0, 1.0,
                                  "phase A: router picked a LEO");
        NS_TEST_ASSERT_MSG_GT(a.deliveredSrv, 100000u,
                              "phase A: flow delivered end-to-end over LEO-A");
        NS_TEST_ASSERT_MSG_GT(a.carriedA, 100000u,
                              "phase A: chosen LEO-A carries the flow");
        NS_TEST_ASSERT_MSG_EQ(a.carriedB, 0u,
                              "phase A: off-route LEO-B carries ~0 bytes");

        // Phase B: LEO-B overhead -> router picks B -> data plane REROUTES to B.
        PhaseResult b = RunPhase(false);
        NS_TEST_ASSERT_MSG_EQ(b.routerPickedA, false,
                              "phase B: router selects overhead LEO-B");
        NS_TEST_ASSERT_MSG_GT(b.deliveredSrv, 100000u,
                              "phase B: flow delivered end-to-end over LEO-B");
        NS_TEST_ASSERT_MSG_GT(b.carriedB, 100000u,
                              "phase B: chosen LEO-B carries the flow");
        NS_TEST_ASSERT_MSG_EQ(b.carriedA, 0u,
                              "phase B: off-route LEO-A carries ~0 bytes");

        // The carried traffic FOLLOWED the router decision, not a fixed chain:
        // A-bytes dominate in phase A, B-bytes dominate in phase B.
        NS_TEST_ASSERT_MSG_GT(a.carriedA, b.carriedA,
                              "LEO-A only carries when the router selects it");
        NS_TEST_ASSERT_MSG_GT(b.carriedB, a.carriedB,
                              "LEO-B only carries when the router selects it");
    }
};

/// SAGIN-5: the chained A2G loss must equal TR 36.777, including where TR 36.777
/// predicts LESS loss than free space.
///
/// SaginA2gPropagationLossModel charges only the EXCESS over free space, so the
/// base Friis term plus this model composes to the TR 36.777 path loss. That
/// invariant was broken by a std::max(0.0, ...) clamp on the excess: the chained
/// net became max(FSPL, PL_TR36777 + sf).
///
/// Deterministically the clamp bites at short range, where UMa-AV LOS is below
/// free space. Statistically it is worse: shadow fading is added before the
/// clamp, so negative fading draws are clipped and the distribution is
/// RECTIFIED rather than shifted, raising the mean and shrinking the variance.
class SaginA2gSignedExcessTest : public TestCase
{
  public:
    SaginA2gSignedExcessTest()
        : TestCase("SAGIN-5 - chained A2G loss equals TR 36.777 even when it is below free space")
    {
    }

  private:
    void DoRun() override
    {
        // 2 GHz, UMa-AV LOS, 100 m: TR 36.777 gives 28.0 + 22*log10(100) +
        // 20*log10(2) = 78.02 dB, while FSPL is 78.47 dB. The excess is
        // NEGATIVE, which is exactly the case the clamp erased.
        const double fcGHz = 2.0;
        const double d3d = 100.0;
        const double hUt = 50.0; // above the UMa-AV height threshold

        const double tr36777 =
            A2gChannelTr36777::PathLossDb(A2gScenario::UMa_AV, A2gLink::LOS, d3d, fcGHz, hUt);
        const double fspl =
            20.0 * std::log10(d3d) + 20.0 * std::log10(fcGHz * 1e9) - 147.55;

        NS_TEST_ASSERT_MSG_LT(tr36777, fspl,
                              "this test only means something where TR 36.777 sits BELOW free "
                              "space; if that is no longer true at 100 m, pick another geometry "
                              "rather than deleting the check");

        // The model must be willing to report a negative excess. A clamp shows
        // up here as an excess of exactly zero.
        auto model = CreateObject<SaginA2gPropagationLossModel>();
        model->SetScenario(A2gScenario::UMa_AV);
        model->SetFrequencyGHz(fcGHz);
        model->SetLink(A2gLink::LOS);
        model->SetUavAltitudeM(hUt);
        // Isolate the deterministic term; the attribute name is the model's own.
        model->SetAttribute("ApplyShadowFading", BooleanValue(false));

        auto a = CreateObject<ConstantPositionMobilityModel>();
        a->SetPosition(Vector(0.0, 0.0, hUt));
        auto b = CreateObject<ConstantPositionMobilityModel>();
        b->SetPosition(Vector(std::sqrt(d3d * d3d - hUt * hUt), 0.0, 0.0));

        (void)model->CalcRxPower(0.0, a, b);
        const double excess = model->GetLastExcessDb();

        NS_TEST_ASSERT_MSG_LT(excess, 0.0,
                              "the excess must be NEGATIVE here; zero means the signed excess is "
                              "being clamped, which silently replaces TR 36.777 with free space "
                              "and rectifies the shadow-fading distribution");
        Simulator::Destroy();
    }
};


/// SAGIN-2: the router must be able to see a saturated link.
///
/// No capacity, queue occupancy or congestion term existed in any SAGIN routing
/// decision. The default scorer returned c.elevationDeg and nothing else, and
/// SaginHopCandidate carried no load field at all, so a router would steer
/// every slice onto the same satellite however full its ISL was - which is the
/// central question these modules exist to study. The P2P ISLs in
/// sagin-sgp4-routed-traffic carry a default DropTail queue, so traffic could
/// be dropped on the very link the router called shortest.
class SaginCongestionAwareRoutingTest : public TestCase
{
  public:
    SaginCongestionAwareRoutingTest()
        : TestCase("SAGIN-2: routing sees link capacity and queue occupancy")
    {
    }

  private:
    static Ptr<ConstantPositionMobilityModel> At(double x, double y, double z)
    {
        Ptr<ConstantPositionMobilityModel> m = CreateObject<ConstantPositionMobilityModel>();
        m->SetPosition(Vector{x, y, z});
        return m;
    }

    void DoRun() override
    {
        // Two LEO candidates. The HIGHER-elevation one is the congested one, so
        // elevation alone and elevation-plus-load give different answers - which
        // is the only way to tell whether load is being consulted at all.
        Ptr<ConstantPositionMobilityModel> ue = At(0.0, 0.0, 0.0);
        Ptr<ConstantPositionMobilityModel> leoHighBusy = At(0.0, 0.0, 600e3);
        Ptr<ConstantPositionMobilityModel> leoLowIdle = At(300e3, 0.0, 600e3);

        auto build = [&]() {
            Ptr<MultiLayerRouter> r = CreateObject<MultiLayerRouter>();
            r->AddNode(SaginLayer::Leo, leoHighBusy);
            r->AddNode(SaginLayer::Leo, leoLowIdle);
            return r;
        };

        // ---- Baseline: no load source, elevation wins ----
        Ptr<MultiLayerRouter> plain = build();
        auto p0 = plain->Route(ue);
        Ptr<MobilityModel> chosen0;
        for (const auto& h : p0)
        {
            if (h.layer == SaginLayer::Leo)
            {
                chosen0 = h.node;
            }
        }
        NS_TEST_ASSERT_MSG_EQ((chosen0 == leoHighBusy), true,
                              "with no load information the overhead satellite wins on elevation");

        // ---- Congestion-aware, with the overhead satellite nearly full ----
        Ptr<MultiLayerRouter> aware = build();
        aware->SetLinkLoadSource(
            [&](Ptr<MobilityModel>, Ptr<MobilityModel> node, double& cap, double& occ) {
                cap = 50e6;
                occ = (node == leoHighBusy) ? 0.90 : 0.05;
                return true;
            });
        aware->SetCongestionAware(true);
        auto p1 = aware->Route(ue);
        Ptr<MobilityModel> chosen1;
        for (const auto& h : p1)
        {
            if (h.layer == SaginLayer::Leo)
            {
                chosen1 = h.node;
            }
        }
        NS_TEST_ASSERT_MSG_EQ((chosen1 == leoLowIdle), true,
                              "a 90%-full link must lose to an idle one even though its elevation "
                              "is higher; picking the congested satellite here is exactly the "
                              "behaviour a load-blind router produces");

        // ---- Unknown load must NOT be scored as idle ----
        // This is the subtle half: a source that declines to answer has to leave
        // the decision on elevation, not hand the candidate a free pass.
        Ptr<MultiLayerRouter> unknown = build();
        unknown->SetLinkLoadSource(
            [](Ptr<MobilityModel>, Ptr<MobilityModel>, double&, double&) { return false; });
        unknown->SetCongestionAware(true);
        auto p2 = unknown->Route(ue);
        Ptr<MobilityModel> chosen2;
        for (const auto& h : p2)
        {
            if (h.layer == SaginLayer::Leo)
            {
                chosen2 = h.node;
            }
        }
        NS_TEST_ASSERT_MSG_EQ((chosen2 == leoHighBusy), true,
                              "with the load source declining, the decision falls back to "
                              "elevation rather than treating the unknown link as empty");

        // ---- A link at the cutoff is rejected outright ----
        Ptr<MultiLayerRouter> full = build();
        full->SetLinkLoadSource(
            [&](Ptr<MobilityModel>, Ptr<MobilityModel> node, double& cap, double& occ) {
                cap = 50e6;
                occ = (node == leoHighBusy) ? 0.99 : 0.05;
                return true;
            });
        full->SetCongestionAware(true, 1.0, 0.95);
        auto p3 = full->Route(ue);
        Ptr<MobilityModel> chosen3;
        for (const auto& h : p3)
        {
            if (h.layer == SaginLayer::Leo)
            {
                chosen3 = h.node;
            }
        }
        NS_TEST_ASSERT_MSG_EQ((chosen3 == leoLowIdle), true, "the saturated link is not chosen");
        NS_TEST_ASSERT_MSG_GT(full->GetCongestionRejections(), 0u,
                              "and the rejection is COUNTED, so a scenario can tell 'the router "
                              "avoided a full link' from 'the router never saw one'");

        // ---- Congestion-awareness off must restore the old behaviour ----
        Ptr<MultiLayerRouter> off = build();
        off->SetLinkLoadSource(
            [&](Ptr<MobilityModel>, Ptr<MobilityModel> node, double& cap, double& occ) {
                cap = 50e6;
                occ = (node == leoHighBusy) ? 0.99 : 0.05;
                return true;
            });
        off->SetCongestionAware(false);
        auto p4 = off->Route(ue);
        Ptr<MobilityModel> chosen4;
        for (const auto& h : p4)
        {
            if (h.layer == SaginLayer::Leo)
            {
                chosen4 = h.node;
            }
        }
        NS_TEST_ASSERT_MSG_EQ((chosen4 == leoHighBusy), true,
                              "with the feature off the router is max-elevation again, so "
                              "existing scenarios keep their behaviour");
    }
};


/// SAGIN-2 (store-and-forward): custody across a contact gap.
///
/// A grep for queue, congestion, buffer, store-and-forward, DTN or bundle
/// across ntn-sagin and ntn-constellation returned nothing relevant. Contact
/// graph routing exists in the space community precisely BECAUSE contacts are
/// intermittent - a node takes custody while the next hop is out of contact and
/// forwards when it opens. Without that, a disconnected-operation scenario
/// cannot be run at all: a router that finds no path simply drops.
class SaginCustodyQueueTest : public TestCase
{
  public:
    SaginCustodyQueueTest()
        : TestCase("SAGIN-2: custody queue holds across a contact gap and drains in order")
    {
    }

  private:
    void DoRun() override
    {
        using namespace ns3::ntnsagin;

        std::vector<uint32_t> delivered;
        auto sink = [&delivered](Ptr<Packet> p) {
            delivered.push_back(p->GetSize());
            return true;
        };

        Ptr<SaginCustodyQueue> q = CreateObject<SaginCustodyQueue>();
        q->SetForwardCallback(sink);
        q->SetCapacityBytes(10000);
        q->SetLifetime(Seconds(100));

        // ---- Contact DOWN: everything is held, nothing is lost ----
        q->SetContactUp(false);
        for (uint32_t i = 1; i <= 5; ++i)
        {
            NS_TEST_ASSERT_MSG_EQ(q->Offer(Create<Packet>(100 * i)), true,
                                  "an offer during a contact gap must be taken into custody, "
                                  "not dropped; dropping here is what happens today with no "
                                  "custody queue at all");
        }
        NS_TEST_ASSERT_MSG_EQ(q->GetInCustody(), 5u, "all five are held");
        NS_TEST_ASSERT_MSG_EQ(delivered.empty(), true, "and none delivered while down");
        NS_TEST_ASSERT_MSG_EQ(q->GetBytesInCustody(), 1500u, "byte accounting tracks the hold");

        // ---- Contact UP: drains in ARRIVAL order ----
        q->SetContactUp(true);
        NS_TEST_ASSERT_MSG_EQ(delivered.size(), 5u, "opening the contact drains the backlog");
        NS_TEST_ASSERT_MSG_EQ(q->GetInCustody(), 0u, "and empties custody");
        NS_TEST_ASSERT_MSG_EQ(q->GetForwardedFromCustody(), 5u, "counted as custody deliveries");
        // Guarded: ns-3 assertions can be configured to continue past a
        // failure, and indexing an empty vector turns a failure into UB.
        if (delivered.size() == 5)
        {
            for (uint32_t i = 0; i < 5; ++i)
            {
                NS_TEST_ASSERT_MSG_EQ(delivered[i], 100 * (i + 1),
                                      "the drain must preserve arrival order; reordering a "
                                      "stream across a contact gap is a different failure from "
                                      "losing it");
            }
        }

        // ---- While UP, a packet goes straight through ----
        delivered.clear();
        NS_TEST_ASSERT_MSG_EQ(q->Offer(Create<Packet>(77)), true, "accepted");
        NS_TEST_ASSERT_MSG_EQ(delivered.size(), 1u, "forwarded immediately");
        NS_TEST_ASSERT_MSG_EQ(q->GetForwardedImmediately(), 1u,
                              "and counted separately from custody deliveries, so a scenario "
                              "can say what the store-and-forward actually bought");

        // ---- A refusing forwarder must NOT lose custody ----
        Ptr<SaginCustodyQueue> q2 = CreateObject<SaginCustodyQueue>();
        q2->SetForwardCallback([](Ptr<Packet>) { return false; });
        q2->SetContactUp(true);
        q2->Offer(Create<Packet>(200));
        NS_TEST_ASSERT_MSG_EQ(q2->GetInCustody(), 1u,
                              "a hop that could not take the packet has not taken it; the packet "
                              "stays in custody rather than being counted as delivered");
        NS_TEST_ASSERT_MSG_EQ(q2->GetForwardedImmediately(), 0u, "and nothing was forwarded");

        // ---- Capacity: the OLDEST is dropped, and it is counted ----
        // Distinct sizes, so WHICH item was evicted is observable. Equal
        // sizes would leave the counts identical whether the oldest or the
        // newest was dropped, and the test would prove nothing about the
        // policy.
        delivered.clear();
        Ptr<SaginCustodyQueue> q3 = CreateObject<SaginCustodyQueue>();
        q3->SetForwardCallback(sink);
        q3->SetCapacityBytes(180);
        q3->SetContactUp(false);
        q3->Offer(Create<Packet>(50)); // A
        q3->Offer(Create<Packet>(60)); // B
        q3->Offer(Create<Packet>(70)); // C -> 180, full
        q3->Offer(Create<Packet>(80)); // D -> evicts A then B, leaving C,D
        NS_TEST_ASSERT_MSG_GT(q3->GetDroppedForSpace(), 0u,
                              "overflow must be COUNTED, not silent - a custody queue that "
                              "quietly loses traffic is worse than none");
        q3->SetContactUp(true);
        NS_TEST_ASSERT_MSG_EQ(delivered.size(), 2u, "two survive the eviction");
        if (delivered.size() == 2)
        {
            NS_TEST_ASSERT_MSG_EQ(delivered[0], 70u,
                                  "the OLDEST items are evicted, so C survives and is delivered "
                                  "first. Dropping the newest instead would leave A here, and "
                                  "with equal packet sizes the two policies are "
                                  "indistinguishable - which is why the sizes differ");
            NS_TEST_ASSERT_MSG_EQ(delivered[1], 80u, "followed by D");
        }

        Simulator::Destroy();
    }
};

/// SAGIN-2: custody has a lifetime, and expiry is reported.
class SaginCustodyExpiryTest : public TestCase
{
  public:
    SaginCustodyExpiryTest()
        : TestCase("SAGIN-2: custody items expire on their lifetime and are counted")
    {
    }

  private:
    Ptr<ns3::ntnsagin::SaginCustodyQueue> m_q;
    uint32_t m_delivered{0};

    void OfferOne()
    {
        m_q->Offer(Create<Packet>(100));
    }

    void OpenContact()
    {
        m_q->SetContactUp(true);
    }

    void DoRun() override
    {
        using namespace ns3::ntnsagin;
        m_q = CreateObject<SaginCustodyQueue>();
        m_q->SetForwardCallback([this](Ptr<Packet>) {
            ++m_delivered;
            return true;
        });
        m_q->SetLifetime(Seconds(10));
        m_q->SetContactUp(false);

        // One packet early, one late. The contact opens after the first has
        // outlived its custody lifetime, so exactly one should survive.
        Simulator::Schedule(Seconds(1.0), &SaginCustodyExpiryTest::OfferOne, this);
        Simulator::Schedule(Seconds(25.0), &SaginCustodyExpiryTest::OfferOne, this);
        Simulator::Schedule(Seconds(30.0), &SaginCustodyExpiryTest::OpenContact, this);
        Simulator::Stop(Seconds(40.0));
        Simulator::Run();

        NS_TEST_ASSERT_MSG_EQ(m_q->GetExpired(), 1u,
                              "the packet held for 29 s against a 10 s lifetime must expire, and "
                              "the expiry must be counted rather than the packet silently "
                              "vanishing from the accounting");
        NS_TEST_ASSERT_MSG_EQ(m_delivered, 1u,
                              "the packet offered 5 s before the contact opened is still within "
                              "its lifetime and must be delivered");
        NS_TEST_ASSERT_MSG_GT(m_q->GetMaxCustodyDelay().GetSeconds(), 4.0,
                              "and the delivered item's custody delay is reported, which is the "
                              "cost store-and-forward trades against the loss it avoids");
        Simulator::Destroy();
    }
};

class NtnSaginTestSuite : public TestSuite
{
  public:
    NtnSaginTestSuite()
        : TestSuite("ntn-sagin", Type::UNIT)
    {
        AddTestCase(new SaginCongestionAwareRoutingTest, TestCase::Duration::QUICK);
        AddTestCase(new SaginCustodyQueueTest, TestCase::Duration::QUICK);
        AddTestCase(new SaginCustodyExpiryTest, TestCase::Duration::QUICK);
        AddTestCase(new HapsAltitudeStableTest, TestCase::Duration::QUICK);
        AddTestCase(new UavPatrolReturnsToStartTest, TestCase::Duration::QUICK);
        AddTestCase(new A2gPathLossSpotCheckTest, TestCase::Duration::QUICK);
        AddTestCase(new A2gLosProbabilityMonotonicTest, TestCase::Duration::QUICK);
        AddTestCase(new A2gStochasticFadingTest, TestCase::Duration::QUICK);
        AddTestCase(new MultiLayerRouterConvergesTest, TestCase::Duration::QUICK);
        AddTestCase(new RouterGatesDataPlaneTest, TestCase::Duration::QUICK);
        AddTestCase(new AeronauticalReachesArrivalTest, TestCase::Duration::QUICK);
        // Roadmap §4.4.1 — OpenSky ADS-B trace importer + replay mobility.
        AddTestCase(new OpenSkyImporterParseTest, TestCase::Duration::QUICK);
        AddTestCase(new OpenSkyImporterMalformedTest, TestCase::Duration::QUICK);
        AddTestCase(new OpenSkyTraceInterpolationTest, TestCase::Duration::QUICK);
        AddTestCase(new OpenSkySimulatorTimeReplayTest, TestCase::Duration::QUICK);
        // Roadmap §4.4.2 — AIS maritime trace importer + replay mobility.
        AddTestCase(new AisImporterParseTest, TestCase::Duration::QUICK);
        AddTestCase(new AisImporterMalformedTest, TestCase::Duration::QUICK);
        AddTestCase(new AisTraceInterpolationTest, TestCase::Duration::QUICK);
        AddTestCase(new AisSimulatorTimeReplayTest, TestCase::Duration::QUICK);
        // Roadmap §4.4.3 — HST high-speed train (TR 38.901 §7.5).
        AddTestCase(new HstPresetGeometryTest, TestCase::Duration::QUICK);
        AddTestCase(new HstDopplerShiftTest, TestCase::Duration::QUICK);
        AddTestCase(new HstSimulatorTimePassByTest, TestCase::Duration::QUICK);
        // Roadmap §4.4.10 — RL load-balancing hooks for ISL routing.
        AddTestCase(new RouterDefaultScorerTest, TestCase::Duration::QUICK);
        AddTestCase(new RouterCustomScorerOverridesTest, TestCase::Duration::QUICK);
        AddTestCase(new RouterObservationPassThroughTest, TestCase::Duration::QUICK);
        AddTestCase(new RouterPerLayerScorerTest, TestCase::Duration::QUICK);
        AddTestCase(new RouterClearScorerTest, TestCase::Duration::QUICK);
        AddTestCase(new RouterSimulatorTimeRlTest, TestCase::Duration::QUICK);
        // Roadmap §4.4.7 — cross-layer slice-aware routing.
        AddTestCase(new SliceQfiMappingTest, TestCase::Duration::QUICK);
        AddTestCase(new SliceMmtcVsEmbbLayerTest, TestCase::Duration::QUICK);
        AddTestCase(new SliceTightUrllcStopsAtHapsTest, TestCase::Duration::QUICK);
        AddTestCase(new SliceDefaultUrllcSkipsGeoTest, TestCase::Duration::QUICK);
        AddTestCase(new SliceSimulatorTimeTest, TestCase::Duration::QUICK);

        // Roadmap §4.4.8 — HAPS trajectory CSV ingest.
        AddTestCase(new HapsTrajectoryImportParseTest, TestCase::Duration::QUICK);
        AddTestCase(new HapsTrajectoryMalformedTest, TestCase::Duration::QUICK);
        AddTestCase(new HapsTrajectoryInterpolationTest, TestCase::Duration::QUICK);
        AddTestCase(new HapsTrajectoryMobilitySimulatorTest, TestCase::Duration::QUICK);
        AddTestCase(new SaginA2gSignedExcessTest, TestCase::Duration::QUICK);
        AddTestCase(new MultiLayerRouterSkipsPopulatedLayerTest,
                    TestCase::Duration::QUICK);
        AddTestCase(new A2gValidatedHeightBandTest, TestCase::Duration::QUICK);
    }
};

static NtnSaginTestSuite g_ntnSaginTestSuite;

} // namespace
