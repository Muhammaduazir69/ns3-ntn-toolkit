/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#include "ns3/constant-position-mobility-model.h"
#include "ns3/constant-velocity-mobility-model.h"
#include "ns3/log.h"
#include "ns3/ntn-rrc-helper.h"
#include "ns3/ntn-sib19.h"
#include "ns3/ntn-timing-advance.h"
#include "ns3/simulator.h"
#include "ns3/test.h"

#include <cmath>

using namespace ns3;
using namespace ns3::ntnrrc;

namespace
{

constexpr double kC = NtnTimingAdvance::kSpeedOfLight;
constexpr double kEarthRadiusMetres = 6378137.0;

Ptr<MobilityModel>
MakeStaticMob(const Vector& pos)
{
    Ptr<ConstantPositionMobilityModel> m = CreateObject<ConstantPositionMobilityModel>();
    m->SetPosition(pos);
    return m;
}

Ptr<MobilityModel>
MakeMovingMob(const Vector& pos, const Vector& vel)
{
    Ptr<ConstantVelocityMobilityModel> m = CreateObject<ConstantVelocityMobilityModel>();
    m->SetPosition(pos);
    m->SetVelocity(vel);
    return m;
}

} // namespace

/// Total TA = 2 * d / c for transparent payload, when UE and satellite are on
/// the y-axis with a 550-km LEO altitude geometry (closed-form known answer).
class NtnTimingAdvanceClosedFormTest : public TestCase
{
  public:
    NtnTimingAdvanceClosedFormTest()
        : TestCase("Total TA matches 2 d over c for transparent payload")
    {
    }

  private:
    void DoRun() override
    {
        NtnRrcHelper helper;
        helper.SetPayloadMode(PayloadMode::Transparent);

        const Vector ue{0.0, 0.0, 0.0};
        const Vector sat{0.0, 0.0, 550e3};
        Ptr<NtnTimingAdvance> ta = helper.InstallTimingAdvance(MakeStaticMob(ue), MakeStaticMob(sat));

        const double expectedSeconds = 2.0 * 550e3 / kC;
        const Time computed = ta->ComputeTotalTa();
        // Sub-microsecond tolerance (10 ns) — closed-form, no floating noise.
        NS_TEST_ASSERT_MSG_EQ_TOL(computed.GetSeconds(), expectedSeconds, 1e-8,
                                  "TA total off from 2 * d / c");
    }
};

/// Regenerative payload halves the TA (single-leg).
class NtnTimingAdvanceRegenerativeTest : public TestCase
{
  public:
    NtnTimingAdvanceRegenerativeTest()
        : TestCase("Regenerative payload halves the TA vs transparent")
    {
    }

  private:
    void DoRun() override
    {
        NtnRrcHelper helper;
        helper.SetPayloadMode(PayloadMode::RegenerativeFull);
        Ptr<NtnTimingAdvance> ta =
            helper.InstallTimingAdvance(MakeStaticMob(Vector{0, 0, 0}),
                                        MakeStaticMob(Vector{0, 0, 550e3}));
        const double expectedSeconds = 550e3 / kC; // single leg
        NS_TEST_ASSERT_MSG_EQ_TOL(ta->ComputeTotalTa().GetSeconds(), expectedSeconds, 1e-8,
                                  "Regenerative TA != d / c");
    }
};

/// 3GPP TR 38.821 cited values: at 600 km LEO altitude, one-way delay ≈ 2 ms,
/// round-trip ≈ 4 ms (Table 6.1.1.1-1 region).
class NtnTimingAdvance38821ReferenceTest : public TestCase
{
  public:
    NtnTimingAdvance38821ReferenceTest()
        : TestCase("TA at 600 km nadir matches TR 38.821 reference within 5%")
    {
    }

  private:
    void DoRun() override
    {
        NtnRrcHelper helper;
        helper.SetPayloadMode(PayloadMode::Transparent);
        Ptr<NtnTimingAdvance> ta =
            helper.InstallTimingAdvance(MakeStaticMob(Vector{0, 0, 0}),
                                        MakeStaticMob(Vector{0, 0, 600e3}));
        const double expectedRoundTripSec = 2.0 * 600e3 / kC; // ~4.0 ms
        const double computed = ta->ComputeTotalTa().GetSeconds();
        NS_TEST_ASSERT_MSG_EQ_TOL(computed, expectedRoundTripSec, 0.05 * expectedRoundTripSec,
                                  "TR 38.821 reference TA out of band");
    }
};

/// SIB19 broadcast value (TA_common) is the 2*d/c to the *reference* point,
/// so a UE displaced from the reference shows a non-zero UE-specific residual.
class NtnTimingAdvanceCommonAndUeSpecificTest : public TestCase
{
  public:
    NtnTimingAdvanceCommonAndUeSpecificTest()
        : TestCase("Common + UE-specific TA decomposition")
    {
    }

  private:
    void DoRun() override
    {
        NtnRrcHelper helper;
        helper.SetPayloadMode(PayloadMode::Transparent);
        const Vector beamCentre{0, 0, 0};
        const Vector ue{0, 50e3, 0}; // 50 km from beam centre
        const Vector sat{0, 0, 550e3};
        helper.SetReferencePosition(beamCentre);
        Ptr<NtnTimingAdvance> ta = helper.InstallTimingAdvance(MakeStaticMob(ue), MakeStaticMob(sat));

        const Time total = ta->ComputeTotalTa();
        const Time common = ta->ComputeCommonTa();
        const Time residual = ta->ComputeUeSpecificTa();

        // Decomposition consistency: total == common + ue-specific by construction.
        NS_TEST_ASSERT_MSG_EQ_TOL(residual.GetSeconds(),
                                  (total - common).GetSeconds(),
                                  1e-12,
                                  "Residual TA does not match total - common");
        NS_TEST_EXPECT_MSG_GT(residual.GetSeconds(), 0.0,
                              "UE 50 km off-centre should have non-zero residual TA");
    }
};

/// Drift rate is bounded for LEO satellites: at 7.5 km/s ground-track speed
/// the radial velocity component cannot exceed c, so drift << 1.
class NtnTimingAdvanceDriftRateTest : public TestCase
{
  public:
    NtnTimingAdvanceDriftRateTest()
        : TestCase("LEO TA drift rate is bounded under 50 us per s")
    {
    }

  private:
    void DoRun() override
    {
        NtnRrcHelper helper;
        helper.SetPayloadMode(PayloadMode::Transparent);
        // 550 km circular orbit, 7.59 km/s tangential (along x).
        const Vector sat{0, 0, kEarthRadiusMetres + 550e3};
        const Vector satV{7590.0, 0.0, 0.0};
        Ptr<NtnTimingAdvance> ta =
            helper.InstallTimingAdvance(MakeStaticMob(Vector{0, 0, 0}),
                                        MakeMovingMob(sat, satV));
        const double drift = std::abs(ta->ComputeTaDriftRate(MilliSeconds(10)));
        // TR 38.821 §6.3.3: TA drift rate at LEO nadir is essentially 0 (purely
        // tangential motion); off-nadir bounds at <50 µs/s.
        NS_TEST_EXPECT_MSG_LT(drift, 50e-6, "Drift rate too large for LEO");
    }
};

/// SIB19 codec round-trip: serialise → parse → all fields equal.
class Sib19CodecRoundTripTest : public TestCase
{
  public:
    Sib19CodecRoundTripTest()
        : TestCase("SIB19 serialise then parse round-trips all fields")
    {
    }

  private:
    void DoRun() override
    {
        Sib19Content sib;
        sib.cellId = 0xBEEF;
        sib.payloadMode = PayloadMode::RegenerativeFull;
        sib.taFrame = TaReferenceFrame::SatelliteSubpoint;
        sib.cellSpecificKoffset = 17;
        sib.kMac = 32;
        sib.taCommon = MicroSeconds(13837);
        sib.taCommonDriftRate = -4.88e-5;
        sib.taCommonDriftVariation = 1.2e-9;
        sib.ulSyncValidity = MilliSeconds(900);
        sib.ephemeris.epoch = MilliSeconds(1715000000123);
        sib.ephemeris.positionEcefM = Vector{1.234e6, -2.345e6, 6.789e6};
        sib.ephemeris.velocityEcefMps = Vector{7590.0, -125.5, 50.25};
        sib.referencePosEcefM = Vector{1234.0, 5678.0, 0.0};

        std::vector<uint8_t> buf(Sib19Codec::kSerialisedBytes);
        const auto written = Sib19Codec::Serialise(sib, buf.data(), buf.size());
        NS_TEST_ASSERT_MSG_EQ(written,
                              Sib19Codec::kSerialisedBytes,
                              "Wrote unexpected number of bytes");

        Sib19Content out;
        NS_TEST_ASSERT_MSG_EQ(Sib19Codec::Parse(buf.data(), buf.size(), out),
                              true,
                              "Parse rejected a valid buffer");

        NS_TEST_EXPECT_MSG_EQ(out.cellId, sib.cellId, "cellId");
        NS_TEST_EXPECT_MSG_EQ(static_cast<int>(out.payloadMode),
                              static_cast<int>(sib.payloadMode),
                              "payloadMode");
        NS_TEST_EXPECT_MSG_EQ(static_cast<int>(out.taFrame),
                              static_cast<int>(sib.taFrame),
                              "taFrame");
        NS_TEST_EXPECT_MSG_EQ(out.cellSpecificKoffset, sib.cellSpecificKoffset, "Koffset");
        NS_TEST_EXPECT_MSG_EQ(out.kMac, sib.kMac, "kMac");
        NS_TEST_EXPECT_MSG_EQ(out.taCommon.GetNanoSeconds(),
                              sib.taCommon.GetNanoSeconds(),
                              "taCommon");
        NS_TEST_EXPECT_MSG_EQ_TOL(out.taCommonDriftRate,
                                  sib.taCommonDriftRate,
                                  1e-15,
                                  "drift rate");
        NS_TEST_EXPECT_MSG_EQ_TOL(out.taCommonDriftVariation,
                                  sib.taCommonDriftVariation,
                                  1e-18,
                                  "drift variation");
        NS_TEST_EXPECT_MSG_EQ(out.ulSyncValidity.GetNanoSeconds(),
                              sib.ulSyncValidity.GetNanoSeconds(),
                              "ulSyncValidity");
        NS_TEST_EXPECT_MSG_EQ(out.ephemeris.epoch.GetNanoSeconds(),
                              sib.ephemeris.epoch.GetNanoSeconds(),
                              "ephem.epoch");
        NS_TEST_EXPECT_MSG_EQ_TOL(out.ephemeris.positionEcefM.x,
                                  sib.ephemeris.positionEcefM.x,
                                  1e-9,
                                  "ephem.pos.x");
        NS_TEST_EXPECT_MSG_EQ_TOL(out.ephemeris.velocityEcefMps.z,
                                  sib.ephemeris.velocityEcefMps.z,
                                  1e-9,
                                  "ephem.vel.z");
        NS_TEST_EXPECT_MSG_EQ_TOL(out.referencePosEcefM.y,
                                  sib.referencePosEcefM.y,
                                  1e-9,
                                  "ref.y");
    }
};

class Sib19CodecRejectsTruncatedTest : public TestCase
{
  public:
    Sib19CodecRejectsTruncatedTest()
        : TestCase("SIB19 codec rejects undersized buffers")
    {
    }

  private:
    void DoRun() override
    {
        Sib19Content sib;
        std::vector<uint8_t> shortBuf(Sib19Codec::kSerialisedBytes - 1);
        std::fill(shortBuf.begin(), shortBuf.end(), 0xff);
        NS_TEST_ASSERT_MSG_EQ(Sib19Codec::Parse(shortBuf.data(), shortBuf.size(), sib),
                              false,
                              "Parse must reject a truncated buffer");
    }
};

/// Broadcaster ticks every period, snapshotting the satellite ephemeris each
/// time. After 5 ticks the latest content reflects the satellite's most
/// recent position.
class Sib19BroadcasterTickTest : public TestCase
{
  public:
    Sib19BroadcasterTickTest()
        : TestCase("SIB19 broadcaster snapshots ephemeris on every tick")
    {
    }

  private:
    void DoRun() override
    {
        Ptr<ConstantVelocityMobilityModel> sat = CreateObject<ConstantVelocityMobilityModel>();
        sat->SetPosition(Vector{0.0, 0.0, 550e3});
        sat->SetVelocity(Vector{7590.0, 0.0, 0.0});

        NtnRrcHelper helper;
        helper.SetPayloadMode(PayloadMode::Transparent);
        helper.SetReferencePosition(Vector{0, 0, 0});
        Ptr<NtnTimingAdvance> ta = helper.InstallTimingAdvance(MakeStaticMob(Vector{0, 0, 0}), sat);

        Ptr<NtnSib19Broadcaster> bc = CreateObject<NtnSib19Broadcaster>();
        bc->SetSatelliteMobility(sat);
        bc->SetTimingAdvance(ta);
        bc->SetReferencePosition(Vector{0, 0, 0});
        bc->SetCellId(0x1234);
        bc->SetPayloadMode(PayloadMode::Transparent);
        bc->SetPeriod(MilliSeconds(160));

        bc->Start();
        Simulator::Stop(MilliSeconds(160 * 5 + 10));
        Simulator::Run();

        const auto& latest = bc->GetLatest();
        NS_TEST_EXPECT_MSG_EQ(latest.cellId, 0x1234, "cellId not propagated");
        // After ~800 ms at 7590 m/s, satellite has moved > 6 km along x.
        NS_TEST_EXPECT_MSG_GT(latest.ephemeris.positionEcefM.x, 6000.0, "ephem not refreshed");
        NS_TEST_EXPECT_MSG_EQ(bc->GetLatestSerialised().size(),
                              Sib19Codec::kSerialisedBytes,
                              "Serialised size mismatch");
        Simulator::Destroy();
    }
};

class NtnRrcTestSuite : public TestSuite
{
  public:
    NtnRrcTestSuite()
        : TestSuite("ntn-rrc", Type::UNIT)
    {
        AddTestCase(new NtnTimingAdvanceClosedFormTest, TestCase::Duration::QUICK);
        AddTestCase(new NtnTimingAdvanceRegenerativeTest, TestCase::Duration::QUICK);
        AddTestCase(new NtnTimingAdvance38821ReferenceTest, TestCase::Duration::QUICK);
        AddTestCase(new NtnTimingAdvanceCommonAndUeSpecificTest, TestCase::Duration::QUICK);
        AddTestCase(new NtnTimingAdvanceDriftRateTest, TestCase::Duration::QUICK);
        AddTestCase(new Sib19CodecRoundTripTest, TestCase::Duration::QUICK);
        AddTestCase(new Sib19CodecRejectsTruncatedTest, TestCase::Duration::QUICK);
        AddTestCase(new Sib19BroadcasterTickTest, TestCase::Duration::QUICK);
    }
};

static NtnRrcTestSuite g_ntnRrcTestSuite; //!< static instance registers the suite
