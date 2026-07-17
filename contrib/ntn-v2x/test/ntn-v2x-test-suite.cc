/*
 * SPDX-License-Identifier: GPL-2.0-only
 * Copyright (c) 2026 Muhammad Uzair (ns3-ntn-toolkit, W7)
 */
#include "ns3/box.h"
#include "ns3/constant-position-mobility-model.h"
#include "ns3/constant-velocity-mobility-model.h"
#include "ns3/double.h"
#include "ns3/maritime-scenario.h"
#include "ns3/ntn-v2x-bsm-header.h"
#include "ns3/ntn-v2x-helper.h"
#include "ns3/packet.h"
#include "ns3/simulator.h"
#include "ns3/sumo-traci-bridge.h"
#include "ns3/test.h"
#include "ns3/v2x-leo-direct.h"
#include "ns3/v2x-leo-relay.h"

#include <cmath>
#include <cstdio>

using namespace ns3;
using namespace ns3::ntnv2x;

namespace
{

class FcdTraceReplayJitterTest : public TestCase
{
  public:
    FcdTraceReplayJitterTest()
        : TestCase("Trace replay MEASURED (not injected) sync offset stays under 100 ms")
    {
    }

    void DoRun() override
    {
        const std::string trace = "/tmp/ntn-v2x-test-fcd.csv";
        const std::size_t nVeh = 10;
        const double simSec = 30.0;
        const double dt = 1.0;
        NS_TEST_ASSERT_MSG_EQ(NtnV2xHelper::WriteDeterministicTestFcdCsv(trace, nVeh,
                                                                         1000.0, simSec, dt),
                              true, "deterministic FCD fixture generation failed");

        Ptr<SumoTraciBridge> br = CreateObject<SumoTraciBridge>();
        NS_TEST_ASSERT_MSG_EQ(br->LoadFcdTrace(trace), true, "trace load failed");

        std::vector<Ptr<ConstantPositionMobilityModel>> mobs(nVeh);
        for (std::size_t i = 0; i < nVeh; ++i)
        {
            mobs[i] = CreateObject<ConstantPositionMobilityModel>();
            mobs[i]->SetPosition(Vector{0, 0, 0});
            br->RegisterVehicle("veh" + std::to_string(i), mobs[i]);
        }

        int nSteps = static_cast<int>(simSec / dt);
        for (int s = 0; s <= nSteps; ++s)
        {
            Simulator::Schedule(Seconds(s * dt), [br]() { br->Step(); });
        }
        Simulator::Stop(Seconds(simSec + 1));
        Simulator::Run();
        Simulator::Destroy();

        // The reported offset is now MEASURED from the real ns-3 scheduler vs
        // the trace timestamp (no injected term): in locked replay it is ~0,
        // comfortably under the 100 ms W7 gate.
        double maxJitterMs = br->GetMaxJitterSec() * 1000.0;
        NS_TEST_ASSERT_MSG_LT(maxJitterMs, 100.0,
                              "TraCI replay measured offset " << maxJitterMs
                              << " ms exceeded 100 ms gate");
        std::remove(trace.c_str());
    }
};

class V2xLeoDirectFreeSpaceTest : public TestCase
{
  public:
    V2xLeoDirectFreeSpaceTest()
        : TestCase("V2X-LEO direct free-space PL matches closed form within 0.1 dB")
    {
    }

    void DoRun() override
    {
        // 1000 km slant, 2 GHz: PL = 20*log10(1e6) + 20*log10(2) + 32.45
        //                          = 120 + 6.02 + 32.45 = 158.47 dB
        Vector v{0, 0, 0};
        Vector s{0, 0, 1.0e6};
        auto lb = V2xLeoDirect::ComputeStatic(v, s, /*fcGHz=*/2.0,
                                              /*eirpDbm=*/50.0,
                                              /*noiseDbm=*/-110.0);
        NS_TEST_ASSERT_MSG_LT(std::abs(lb.freeSpacePlDb - 158.47), 0.1,
                              "free-space PL out of spec: " << lb.freeSpacePlDb);
        NS_TEST_ASSERT_MSG_EQ(lb.aboveHorizon, true, "satellite must be above horizon");
        NS_TEST_ASSERT_MSG_LT(std::abs(lb.elevationDeg - 90.0), 0.01, "zenith elevation");
    }
};

class V2xLeoRelayDirectVsRelayTest : public TestCase
{
  public:
    V2xLeoRelayDirectVsRelayTest()
        : TestCase("Relay falls back to peer when direct SNR below threshold")
    {
    }

    void DoRun() override
    {
        // Use a low-altitude pseudo-satellite so vehicle separation is measurable.
        // Real LEO is too far (550 km) for ground-level meters to make a SNR diff —
        // the relay logic itself is identical regardless of magnitudes.
        Ptr<ConstantVelocityMobilityModel> sat = CreateObject<ConstantVelocityMobilityModel>();
        sat->SetPosition(Vector{0, 0, 100.0});  // 100 m AGL test fixture
        Ptr<V2xLeoRelay> relay = CreateObject<V2xLeoRelay>();
        relay->SetSatellite(sat);

        Ptr<ConstantPositionMobilityModel> a = CreateObject<ConstantPositionMobilityModel>();
        a->SetPosition(Vector{0, 0, 0});  // slant = 100 m
        Ptr<ConstantPositionMobilityModel> b = CreateObject<ConstantPositionMobilityModel>();
        b->SetPosition(Vector{300, 0, 0});  // slant ≈ 316 m, V2V range 300 m

        relay->RegisterVehicle("A", a);
        relay->RegisterVehicle("B", b);

        // Get the actual SNRs so threshold can be set between them deterministically.
        auto lbA = V2xLeoDirect::ComputeStatic(a->GetPosition(), sat->GetPosition(),
                                               2.0, 50.0, -110.0);
        auto lbB = V2xLeoDirect::ComputeStatic(b->GetPosition(), sat->GetPosition(),
                                               2.0, 50.0, -110.0);
        // Threshold strictly between the two
        relay->SetMinDirectSnrDb((lbA.snrDb + lbB.snrDb) / 2.0);
        auto out = relay->EvaluateAll();
        bool sawDirect = false, sawRelay = false;
        for (auto& d : out)
        {
            if (d.vehId == "A")
                sawDirect = d.directToLeo;
            if (d.vehId == "B")
                sawRelay = !d.directToLeo && d.relayPeerId == "A";
        }
        NS_TEST_ASSERT_MSG_EQ(sawDirect, true, "vehicle A should be direct");
        NS_TEST_ASSERT_MSG_EQ(sawRelay, true, "vehicle B should relay via A");
    }
};

class MaritimeBouncesInBoxTest : public TestCase
{
  public:
    MaritimeBouncesInBoxTest()
        : TestCase("Maritime mobility stays inside its sea-area bounding box")
    {
    }

    void DoRun() override
    {
        Ptr<MaritimeMobilityModel> m = CreateObject<MaritimeMobilityModel>();
        Box area(-10000, 10000, -10000, 10000, 0, 0);
        m->SetSeaArea(area);

        double minX = 1e9, maxX = -1e9, minY = 1e9, maxY = -1e9;
        for (double t = 0; t <= 1800.0; t += 5.0)
        {
            Simulator::Schedule(Seconds(t), [&minX, &maxX, &minY, &maxY, m]() {
                Vector p = m->GetPosition();
                minX = std::min(minX, p.x);
                maxX = std::max(maxX, p.x);
                minY = std::min(minY, p.y);
                maxY = std::max(maxY, p.y);
            });
        }
        Simulator::Stop(Seconds(1801));
        Simulator::Run();
        Simulator::Destroy();

        NS_TEST_ASSERT_MSG_GT_OR_EQ(minX, area.xMin - 100.0, "drifted west");
        NS_TEST_ASSERT_MSG_LT_OR_EQ(maxX, area.xMax + 100.0, "drifted east");
        NS_TEST_ASSERT_MSG_GT_OR_EQ(minY, area.yMin - 100.0, "drifted south");
        NS_TEST_ASSERT_MSG_LT_OR_EQ(maxY, area.yMax + 100.0, "drifted north");
    }
};

class HundredVehicleSmokeTest : public TestCase
{
  public:
    HundredVehicleSmokeTest()
        : TestCase("100 vehicle 5-min replay completes inside test budget")
    {
    }

    void DoRun() override
    {
        const std::string trace = "/tmp/ntn-v2x-test-100veh.csv";
        const std::size_t n = 100;
        NS_TEST_ASSERT_MSG_EQ(
            NtnV2xHelper::WriteDeterministicTestFcdCsv(trace, n, 30000.0, 300.0, 1.0),
            true, "deterministic fixture generation failed");

        Ptr<SumoTraciBridge> br = CreateObject<SumoTraciBridge>();
        NS_TEST_ASSERT_MSG_EQ(br->LoadFcdTrace(trace), true, "trace load failed");

        std::vector<Ptr<ConstantPositionMobilityModel>> mobs(n);
        for (std::size_t i = 0; i < n; ++i)
        {
            mobs[i] = CreateObject<ConstantPositionMobilityModel>();
            mobs[i]->SetPosition(Vector{-1.0e9, 0, 0});
            br->RegisterVehicle("veh" + std::to_string(i), mobs[i]);
        }
        for (int s = 0; s <= 300; ++s)
        {
            Simulator::Schedule(Seconds(s), [br]() { br->Step(); });
        }
        Simulator::Stop(Seconds(301));
        Simulator::Run();
        Simulator::Destroy();
        // 100 veh × 301 ticks = 30 100 samples expected.
        NS_TEST_ASSERT_MSG_EQ(br->LoadedSampleCount(), 100u * 301u,
                              "trace sample count mismatch");
        std::remove(trace.c_str());
    }
};

/**
 * \brief The J2735 BSM header serialises its kinematic state and reads it back
 *        within each field's encoding resolution — proving the relay packets
 *        carry a real BSM, not opaque padding.
 */
class J2735BsmHeaderRoundTripTest : public TestCase
{
  public:
    J2735BsmHeaderRoundTripTest()
        : TestCase("SAE J2735 BSM header round-trips through a packet")
    {
    }

  private:
    void DoRun() override
    {
        ntnv2x::NtnV2xBsmHeader tx;
        tx.SetFromState(/*msgCnt=*/42, /*id=*/0x0A0B0C0Du, /*secMark=*/12345,
                        /*lat=*/48.137154, /*lon=*/11.576124, /*elev=*/542.3,
                        /*speed=*/27.4, /*heading=*/93.75);

        Ptr<Packet> p = Create<Packet>(0);
        p->AddHeader(tx);
        ntnv2x::NtnV2xBsmHeader rx;
        p->RemoveHeader(rx);

        NS_TEST_ASSERT_MSG_EQ(rx.GetMsgCnt(), 42, "msgCnt");
        NS_TEST_ASSERT_MSG_EQ(rx.GetId(), 0x0A0B0C0Du, "station id");
        NS_TEST_ASSERT_MSG_EQ(rx.GetSecMark(), 12345, "secMark");
        // 1/10 micro-degree resolution -> ~1e-7 deg.
        NS_TEST_ASSERT_MSG_EQ_TOL(rx.GetLatDeg(), 48.137154, 1e-6, "latitude");
        NS_TEST_ASSERT_MSG_EQ_TOL(rx.GetLonDeg(), 11.576124, 1e-6, "longitude");
        NS_TEST_ASSERT_MSG_EQ_TOL(rx.GetElevM(), 542.3, 0.05, "elevation (1 dm)");
        NS_TEST_ASSERT_MSG_EQ_TOL(rx.GetSpeedMps(), 27.4, 0.02, "speed (0.02 m/s)");
        NS_TEST_ASSERT_MSG_EQ_TOL(rx.GetHeadingDeg(), 93.75, 0.0125, "heading (0.0125 deg)");
    }
};

class NtnV2xTestSuite : public TestSuite
{
  public:
    NtnV2xTestSuite()
        : TestSuite("ntn-v2x", Type::UNIT)
    {
        AddTestCase(new FcdTraceReplayJitterTest, TestCase::Duration::QUICK);
        AddTestCase(new V2xLeoDirectFreeSpaceTest, TestCase::Duration::QUICK);
        AddTestCase(new V2xLeoRelayDirectVsRelayTest, TestCase::Duration::QUICK);
        AddTestCase(new MaritimeBouncesInBoxTest, TestCase::Duration::QUICK);
        AddTestCase(new J2735BsmHeaderRoundTripTest, TestCase::Duration::QUICK);
        AddTestCase(new HundredVehicleSmokeTest, TestCase::Duration::EXTENSIVE);
    }
};

static NtnV2xTestSuite g_ntnV2xTestSuite;

} // namespace
