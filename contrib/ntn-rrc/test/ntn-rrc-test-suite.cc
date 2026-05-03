/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#include "ns3/constant-position-mobility-model.h"
#include "ns3/constant-velocity-mobility-model.h"
#include "ns3/log.h"
#include "ns3/ntn-rrc-helper.h"
#include "ns3/ntn-timing-advance.h"
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
    }
};

static NtnRrcTestSuite g_ntnRrcTestSuite; //!< static instance registers the suite
