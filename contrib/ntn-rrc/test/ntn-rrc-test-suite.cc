/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#include "ns3/constant-position-mobility-model.h"
#include "ns3/constant-velocity-mobility-model.h"
#include "ns3/log.h"
#include "ns3/ntn-drx.h"
#include "ns3/ntn-meas-report.h"
#include "ns3/ntn-rach-window.h"
#include "ns3/ntn-rrc-helper.h"
#include "ns3/ntn-sib19.h"
#include "ns3/ntn-timing-advance.h"
#include "ns3/ntn-ue-location-report.h"
#include "ns3/simulator.h"
#include "ns3/test.h"

#include <cstring>
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

/// Regenerative TA is the round-trip service-link delay (2 d / c). Timing
/// Advance always compensates the round trip; the gNB simply sits on the
/// satellite, so the UE<->gNB round trip is the service link 2 d / c (the
/// earlier "single-leg" value under-compensated the uplink by 2x).
class NtnTimingAdvanceRegenerativeTest : public TestCase
{
  public:
    NtnTimingAdvanceRegenerativeTest()
        : TestCase("Regenerative TA is the round-trip service-link 2 d / c")
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
        const double expectedSeconds = 2.0 * 550e3 / kC; // round trip
        NS_TEST_ASSERT_MSG_EQ_TOL(ta->ComputeTotalTa().GetSeconds(), expectedSeconds, 1e-8,
                                  "Regenerative TA != 2 d / c");
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

/// Regression for the regenerative-mode drift bug (audit CRITICAL #7): drift was
/// (d/c - 2d/c)/dt ~= -0.18 s/s at LEO for RegenerativeFull regardless of
/// geometry, because t0 used 2d/c but t1 used 1d/c. With the fix (t1 = 2d/c) the
/// regenerative drift must equal the transparent drift and stay < 25 µs/s.
class NtnTimingAdvanceRegenerativeDriftRateTest : public TestCase
{
  public:
    NtnTimingAdvanceRegenerativeDriftRateTest()
        : TestCase("Regenerative TA drift rate is bounded under 25 us per s")
    {
    }

  private:
    void DoRun() override
    {
        // Same geometry as the transparent test but RegenerativeFull payload.
        const Vector sat{0, 0, kEarthRadiusMetres + 550e3};
        const Vector satV{7590.0, 0.0, 0.0};

        NtnRrcHelper regen;
        regen.SetPayloadMode(PayloadMode::RegenerativeFull);
        Ptr<NtnTimingAdvance> taRegen =
            regen.InstallTimingAdvance(MakeStaticMob(Vector{0, 0, 0}),
                                       MakeMovingMob(sat, satV));
        const double driftRegen = std::abs(taRegen->ComputeTaDriftRate(MilliSeconds(10)));

        // Must be physically bounded — NOT the ~0.18 s/s the old branch produced.
        NS_TEST_ASSERT_MSG_LT(driftRegen, 25e-6,
                              "Regenerative drift too large (got " << driftRegen << ")");

        // Payload mode must not change the drift: TA_total is 2d/c either way.
        NtnRrcHelper trans;
        trans.SetPayloadMode(PayloadMode::Transparent);
        Ptr<NtnTimingAdvance> taTrans =
            trans.InstallTimingAdvance(MakeStaticMob(Vector{0, 0, 0}),
                                       MakeMovingMob(sat, satV));
        const double driftTrans = std::abs(taTrans->ComputeTaDriftRate(MilliSeconds(10)));
        NS_TEST_ASSERT_MSG_EQ_TOL(driftRegen, driftTrans, 1e-12,
                                  "Regenerative and transparent drift must match");
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

/// GAP R3 (CI gate 14, population half): SIB19 must broadcast a NON-ZERO
/// cellSpecificKoffset / kMac derived from the common TA, covering the cell
/// round trip (TS 38.213 §4.2). Before the fix these were hard 0 on the wire.
/// RRC-2: the timing advance must depend on the payload mode.
///
/// m_payloadMode was stored and never read: ComputeTotalTa/ComputeCommonTa
/// returned the service-link round trip unconditionally, because the class had
/// no gateway geometry. A transparent (bent-pipe) LEO cell was therefore
/// compensated for the service leg alone, when the uplink actually traverses
/// the feeder link to the ground gNB as well.
class TimingAdvancePayloadModeTest : public TestCase
{
  public:
    TimingAdvancePayloadModeTest()
        : TestCase("RRC-2 - transparent TA includes the feeder leg, regenerative does not")
    {
    }

  private:
    void DoRun() override
    {
        constexpr double c = 299792458.0;
        const double serviceM = 600e3;  // UE straight below the satellite
        const double feederM = 1200e3;  // gateway well off to the side

        Ptr<ConstantVelocityMobilityModel> sat = CreateObject<ConstantVelocityMobilityModel>();
        sat->SetPosition(Vector{0.0, 0.0, serviceM});
        sat->SetVelocity(Vector{0.0, 0.0, 0.0});
        auto ue = MakeStaticMob(Vector{0, 0, 0});
        // Gateway placed so the satellite-gateway range is exactly feederM.
        const double gx = std::sqrt(feederM * feederM - serviceM * serviceM);
        auto gw = MakeStaticMob(Vector{gx, 0, 0});

        // Transparent WITHOUT a gateway: unchanged behaviour, and the class says
        // so rather than silently returning a short value.
        {
            NtnRrcHelper helper;
            helper.SetPayloadMode(PayloadMode::Transparent);
            helper.SetReferencePosition(Vector{0, 0, 0});
            Ptr<NtnTimingAdvance> ta = helper.InstallTimingAdvance(ue, sat);
            NS_TEST_ASSERT_MSG_EQ(ta->FeederGeometryMissing(), true,
                                  "a transparent payload with no gateway must report that its "
                                  "feeder geometry is missing");
            NS_TEST_ASSERT_MSG_LT(std::abs(ta->ComputeTotalTa().GetSeconds() -
                                           2.0 * serviceM / c),
                                  1e-9,
                                  "without gateway geometry the TA falls back to service-only");
        }

        // Transparent WITH a gateway: both legs.
        {
            NtnRrcHelper helper;
            helper.SetPayloadMode(PayloadMode::Transparent);
            helper.SetReferencePosition(Vector{0, 0, 0});
            Ptr<NtnTimingAdvance> ta = helper.InstallTimingAdvance(ue, sat);
            ta->SetGatewayMobility(gw);
            NS_TEST_ASSERT_MSG_EQ(ta->FeederGeometryMissing(), false,
                                  "gateway supplied, so nothing is missing");
            const double expected = 2.0 * (serviceM + feederM) / c;
            NS_TEST_ASSERT_MSG_LT(std::abs(ta->ComputeTotalTa().GetSeconds() - expected), 1e-9,
                                  "transparent TA must cover BOTH the service and feeder legs; "
                                  "service-only here is the RRC-2 defect");
        }

        // Regenerative: the gNB is on board, so the feeder leg is not in the
        // timing loop even when a gateway is known.
        {
            NtnRrcHelper helper;
            helper.SetPayloadMode(PayloadMode::RegenerativeFull);
            helper.SetReferencePosition(Vector{0, 0, 0});
            Ptr<NtnTimingAdvance> ta = helper.InstallTimingAdvance(ue, sat);
            ta->SetGatewayMobility(gw);
            const double expected = 2.0 * serviceM / c;
            NS_TEST_ASSERT_MSG_LT(std::abs(ta->ComputeTotalTa().GetSeconds() - expected), 1e-9,
                                  "a full on-board gNB terminates the uplink at the satellite, "
                                  "so the feeder leg must NOT be added");
        }
        Simulator::Destroy();
    }
};

/// RRC-1: the broadcast K_offset must reach a consumer.
///
/// cellSpecificKoffset was written to the wire and read by nothing; the NR
/// scheduler re-derived its own value from its own geometry and numerology, so
/// the network could schedule against a number it had never advertised.
class Sib19KOffsetSinkTest : public TestCase
{
  public:
    Sib19KOffsetSinkTest()
        : TestCase("RRC-1 - broadcast K_offset is delivered to a scheduler sink")
    {
    }

  private:
    uint32_t m_delivered{0};
    uint32_t m_calls{0};

    void OnKOffset(uint32_t slots)
    {
        m_delivered = slots;
        m_calls++;
    }

    void DoRun() override
    {
        Ptr<ConstantVelocityMobilityModel> sat = CreateObject<ConstantVelocityMobilityModel>();
        sat->SetPosition(Vector{0.0, 0.0, 600e3});
        sat->SetVelocity(Vector{0.0, 0.0, 0.0});

        NtnRrcHelper helper;
        helper.SetPayloadMode(PayloadMode::Transparent);
        helper.SetReferencePosition(Vector{0, 0, 0});
        Ptr<NtnTimingAdvance> ta = helper.InstallTimingAdvance(MakeStaticMob(Vector{0, 0, 0}), sat);

        Ptr<NtnSib19Broadcaster> bc = CreateObject<NtnSib19Broadcaster>();
        bc->SetSatelliteMobility(sat);
        bc->SetTimingAdvance(ta);
        bc->SetReferencePosition(Vector{0, 0, 0});
        bc->SetCellId(7);
        bc->SetNumerology(1);
        bc->SetPeriod(MilliSeconds(160));
        bc->SetKOffsetSink(MakeCallback(&Sib19KOffsetSinkTest::OnKOffset, this));
        bc->Start();
        Simulator::Stop(MilliSeconds(400));
        Simulator::Run();

        NS_TEST_ASSERT_MSG_GT(m_calls, 0u,
                              "the sink must fire on every SIB19 refresh; zero calls means the "
                              "broadcast K_offset still reaches nobody (the RRC-1 defect)");
        NS_TEST_ASSERT_MSG_EQ(m_delivered, bc->GetLatest().cellSpecificKoffset,
                              "the delivered value must be exactly what was broadcast, so the "
                              "scheduler and SIB19 cannot disagree");
        Simulator::Destroy();
    }
};

/// RRC-6: SIB19 must fill the fields it declares, and must stay inside the
/// TS 38.331 ranges for the ones it fills.
///
/// `taCommonDriftVariation` and `ulSyncValidity` were declared and never
/// written, so every broadcast carried 0.0 and the 900 s struct default, which
/// is the MAXIMUM validity the standard allows: a LEO cell was telling every UE
/// its timing block stayed good for fifteen minutes. And `kMac` was assigned the
/// same number as `cellSpecificKoffset` with no clamp, although TS 38.331 gives
/// them different ranges (1..1023 and 1..512), so a GEO geometry produced values
/// that cannot be encoded at all.
class Sib19FieldsPopulatedAndInRangeTest : public TestCase
{
  public:
    Sib19FieldsPopulatedAndInRangeTest()
        : TestCase("RRC-6: SIB19 fills driftVariation/ulSyncValidity and clamps K_offset/kMac "
                   "to their TS 38.331 ranges")
    {
    }

  private:
    /// \param crossRangeM downrange offset of the satellite from the UE. At
    ///        zenith the velocity is perpendicular to the line of sight and the
    ///        range rate is genuinely zero, so a drift test must be taken
    ///        OFF-zenith or it measures nothing.
    static Sib19Content Broadcast(double altM, double satVxMps, uint8_t numerology,
                                  double crossRangeM = 0.0)
    {
        Ptr<ConstantVelocityMobilityModel> sat = CreateObject<ConstantVelocityMobilityModel>();
        sat->SetPosition(Vector{crossRangeM, 0.0, altM});
        sat->SetVelocity(Vector{satVxMps, 0.0, 0.0});

        NtnRrcHelper helper;
        helper.SetPayloadMode(PayloadMode::Transparent);
        helper.SetReferencePosition(Vector{0, 0, 0});
        Ptr<NtnTimingAdvance> ta =
            helper.InstallTimingAdvance(MakeStaticMob(Vector{0, 0, 0}), sat);

        Ptr<NtnSib19Broadcaster> bc = CreateObject<NtnSib19Broadcaster>();
        bc->SetSatelliteMobility(sat);
        bc->SetTimingAdvance(ta);
        bc->SetReferencePosition(Vector{0, 0, 0});
        bc->SetCellId(7);
        bc->SetNumerology(numerology);
        bc->SetPeriod(MilliSeconds(160));
        bc->Start();
        Simulator::Stop(MilliSeconds(10));
        Simulator::Run();
        const Sib19Content out = bc->GetLatest();
        Simulator::Destroy();
        return out;
    }

    void DoRun() override
    {
        // ---- LEO, moving: both derived fields must be real numbers ---------
        // 500 km downrange and receding at 7.56 km/s: a real range rate, unlike
        // the zenith pass where the velocity is perpendicular to the line of
        // sight and the drift is correctly zero.
        const Sib19Content leo = Broadcast(600e3, 7560.0, 1, 500e3);

        NS_TEST_ASSERT_MSG_NE(leo.taCommonDriftVariation, 0.0,
                              "ta-CommonDriftVariant must be populated; a satellite moving at "
                              "7.56 km/s has a non-zero second derivative of TA");
        NS_TEST_ASSERT_MSG_EQ(std::isfinite(leo.taCommonDriftVariation), true,
                              "and it must be finite");

        // ulSyncValidity must be shorter than the 900 s maximum for a LEO pass,
        // and must be a member of the TS 38.331 enumerated set.
        NS_TEST_ASSERT_MSG_LT(leo.ulSyncValidity.GetSeconds(), 900.0,
                              "a LEO cell must not broadcast the 900 s maximum validity; that "
                              "was the struct default, not a derived value");
        NS_TEST_ASSERT_MSG_GT(leo.ulSyncValidity.GetSeconds(), 0.0, "and must be positive");
        {
            static const double kEnum[] = {5, 10, 15, 20, 25, 30, 35, 40,
                                           45, 50, 55, 60, 120, 180, 240, 900};
            bool inSet = false;
            for (double v : kEnum)
            {
                if (std::fabs(leo.ulSyncValidity.GetSeconds() - v) < 1e-9)
                {
                    inSet = true;
                }
            }
            NS_TEST_ASSERT_MSG_EQ(inSet, true,
                                  "ul-SyncValidityDuration is ENUMERATED in TS 38.331; "
                                  "an arbitrary real is not encodable (got "
                                      << leo.ulSyncValidity.GetSeconds() << " s)");
        }

        // A STATIONARY satellite has no drift, so the validity must fall back to
        // the maximum rather than dividing by zero.
        const Sib19Content still = Broadcast(600e3, 0.0, 1, 500e3);
        NS_TEST_ASSERT_MSG_EQ_TOL(still.ulSyncValidity.GetSeconds(), 900.0, 1e-9,
                                  "with no drift the block stays valid for the maximum");
        // And a satellite at zenith has no RANGE rate however fast it moves,
        // which is physics rather than a defect: the velocity is perpendicular
        // to the line of sight.
        const Sib19Content zenith = Broadcast(600e3, 7560.0, 1, 0.0);
        NS_TEST_ASSERT_MSG_EQ_TOL(zenith.ulSyncValidity.GetSeconds(), 900.0, 1e-9,
                                  "at zenith the range rate is zero and the validity is the "
                                  "maximum, correctly");

        // ---- GEO: the ranges must be respected ----------------------------
        // 35786 km one way is a 238.8 ms round trip; at numerology 1 that is
        // 478 slots, which fits K_offset but is close to kMac's ceiling. Push
        // to numerology 3 (0.125 ms slots) to exceed both.
        const Sib19Content geo = Broadcast(35786e3, 0.0, 3, 0.0);
        NS_TEST_ASSERT_MSG_LT(geo.cellSpecificKoffset, 1024u,
                              "cellSpecificKoffset-r17 is INTEGER (1..1023); "
                              << geo.cellSpecificKoffset << " cannot be encoded");
        NS_TEST_ASSERT_MSG_GT(geo.cellSpecificKoffset, 0u, "and must be at least 1");
        NS_TEST_ASSERT_MSG_LT(geo.kMac, 513u,
                              "kmac-r17 is INTEGER (1..512); " << geo.kMac
                                  << " cannot be encoded. kMac has the SMALLER range and must "
                                     "not simply inherit K_offset's value");
        NS_TEST_ASSERT_MSG_GT(geo.kMac, 0u, "and must be at least 1");
        // At this geometry the raw requirement exceeds both ceilings, so the two
        // fields must actually DIFFER. If they match, kMac is still being
        // assigned K_offset's number.
        NS_TEST_ASSERT_MSG_NE(geo.kMac, geo.cellSpecificKoffset,
                              "at a geometry past both ceilings the clamps differ, so the two "
                              "fields must differ too");

        // ---- LEO still fits, so the clamp must not be firing everywhere ----
        NS_TEST_ASSERT_MSG_EQ(leo.kMac, leo.cellSpecificKoffset,
                              "at LEO both fit their ranges and should agree; if they differ "
                              "here the clamp is engaging when it should not");
    }
};

class Sib19KOffsetPopulatedTest : public TestCase
{
  public:
    Sib19KOffsetPopulatedTest()
        : TestCase("SIB19 cellSpecificKoffset is derived from the common TA (R3)")
    {
    }

  private:
    void DoRun() override
    {
        // Satellite at 600 km straight overhead -> common TA (RTT) = 2*600km/c
        // = 4.0028 ms. At numerology 1 (0.5 ms slot) K_offset = ceil(4.0028/0.5)
        // + 1 = 9 slots.
        Ptr<ConstantVelocityMobilityModel> sat = CreateObject<ConstantVelocityMobilityModel>();
        sat->SetPosition(Vector{0.0, 0.0, 600e3});
        sat->SetVelocity(Vector{0.0, 0.0, 0.0});

        NtnRrcHelper helper;
        helper.SetPayloadMode(PayloadMode::Transparent);
        helper.SetReferencePosition(Vector{0, 0, 0});
        Ptr<NtnTimingAdvance> ta = helper.InstallTimingAdvance(MakeStaticMob(Vector{0, 0, 0}), sat);

        Ptr<NtnSib19Broadcaster> bc = CreateObject<NtnSib19Broadcaster>();
        bc->SetSatelliteMobility(sat);
        bc->SetTimingAdvance(ta);
        bc->SetReferencePosition(Vector{0, 0, 0});
        bc->SetCellId(7);
        bc->SetNumerology(1); // 30 kHz SCS -> 0.5 ms slot
        bc->SetPeriod(MilliSeconds(160));
        bc->Start();
        Simulator::Stop(MilliSeconds(10));
        Simulator::Run();

        const auto& sib = bc->GetLatest();
        const double c = 299792458.0;
        const double rttS = 2.0 * 600e3 / c;
        const double slotS = 0.5e-3;
        const uint32_t expected = static_cast<uint32_t>(std::ceil(rttS / slotS)) + 1;

        NS_TEST_ASSERT_MSG_NE(sib.cellSpecificKoffset, 0u,
                              "K_offset must not be 0 on the wire (R3)");
        NS_TEST_ASSERT_MSG_EQ(sib.cellSpecificKoffset, expected,
                              "K_offset must cover the common-TA round trip in slots");
        NS_TEST_ASSERT_MSG_EQ(sib.kMac, expected, "kMac must match K_offset coverage");
        Simulator::Destroy();
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

// ----------------------------------------------------------------------------
// UE location report tests
// ----------------------------------------------------------------------------

/// ECEF→geodetic→ECEF round-trip is sub-millimetre accurate.
class GeodeticConversionRoundTripTest : public TestCase
{
  public:
    GeodeticConversionRoundTripTest()
        : TestCase("ECEF then geodetic then ECEF round-trips to sub-mm")
    {
    }

  private:
    void DoRun() override
    {
        struct Sample
        {
            double lat, lon, alt;
        };
        const Sample samples[] = {
            {0.0, 0.0, 0.0},                  // gulf of guinea, sea level
            {33.6844, 73.0479, 540.0},        // Islamabad
            {89.5, 0.0, 50.0},                // near north pole
            {-89.5, 180.0, 2800.0},           // antarctic plateau
            {45.0, -90.0, 10000.0},           // mid-latitude, 10 km alt
        };
        for (const auto& s : samples)
        {
            const Vector ecef = GeodeticWgs84ToEcef(s.lat, s.lon, s.alt);
            double lat2, lon2, alt2;
            EcefToGeodeticWgs84(ecef, lat2, lon2, alt2);
            NS_TEST_EXPECT_MSG_EQ_TOL(lat2, s.lat, 1e-7, "lat round-trip");
            NS_TEST_EXPECT_MSG_EQ_TOL(lon2, s.lon, 1e-7, "lon round-trip");
            NS_TEST_EXPECT_MSG_EQ_TOL(alt2, s.alt, 1e-3, "alt round-trip mm");
        }
    }
};

/// Periodic reporter emits a report on every period tick.
class PeriodicLocationReporterTest : public TestCase
{
  public:
    PeriodicLocationReporterTest()
        : TestCase("Periodic location reporter emits one report per period")
    {
    }

  private:
    void DoRun() override
    {
        const Vector ecef = GeodeticWgs84ToEcef(33.6844, 73.0479, 540.0);
        Ptr<MobilityModel> ue = MakeStaticMob(ecef);

        NtnRrcHelper helper;
        Ptr<NtnUeLocationReporter> rep =
            helper.InstallUeLocationReporter(ue,
                                             LocationReportMode::Periodic,
                                             MilliSeconds(100),
                                             0.0);
        rep->Start();
        Simulator::Stop(MilliSeconds(550));
        Simulator::Run();

        NS_TEST_ASSERT_MSG_EQ(rep->HasReport(), true, "No report emitted");
        const auto& latest = rep->GetLatestReport();
        NS_TEST_EXPECT_MSG_GT(latest.reportSequence, 4u,
                              "Expected at least 5 reports in 550 ms");
        NS_TEST_EXPECT_MSG_EQ_TOL(latest.latDeg, 33.6844, 1e-6, "lat");
        NS_TEST_EXPECT_MSG_EQ_TOL(latest.lonDeg, 73.0479, 1e-6, "lon");
        NS_TEST_EXPECT_MSG_EQ_TOL(latest.altMetres, 540.0, 1e-3, "alt");
        rep->Stop();
        Simulator::Destroy();
    }
};

/// Event-triggered reporter only emits when UE moves more than threshold.
class EventTriggeredReporterTest : public TestCase
{
  public:
    EventTriggeredReporterTest()
        : TestCase("Event-triggered reporter respects move-distance threshold")
    {
    }

  private:
    void DoRun() override
    {
        // UE moves 50 m/s east, threshold = 100 m → report every ~2 s.
        // Polling cadence 50 ms; 15 s sim gives ~7 reports.
        Ptr<ConstantVelocityMobilityModel> ue = CreateObject<ConstantVelocityMobilityModel>();
        ue->SetPosition(GeodeticWgs84ToEcef(0.0, 0.0, 0.0));
        ue->SetVelocity(Vector{50.0, 0.0, 0.0});

        NtnRrcHelper helper;
        Ptr<NtnUeLocationReporter> rep =
            helper.InstallUeLocationReporter(ue,
                                             LocationReportMode::EventTriggered,
                                             MilliSeconds(50),
                                             0.0);
        rep->SetEventTriggerDistanceMetres(100.0);
        rep->Start();
        Simulator::Stop(Seconds(15.0));
        Simulator::Run();

        const auto& latest = rep->GetLatestReport();
        // ~7-8 reports expected: t=0 (first sample), then every ~2 s.
        NS_TEST_EXPECT_MSG_GT(latest.reportSequence, 4u,
                              "Expected several event-triggered reports");
        NS_TEST_EXPECT_MSG_LT(latest.reportSequence, 12u,
                              "Too many reports — threshold not enforced");
        rep->Stop();
        Simulator::Destroy();
    }
};

/// On-demand reporter does not auto-tick; ReportNow() drives every emission.
class OnDemandReporterTest : public TestCase
{
  public:
    OnDemandReporterTest()
        : TestCase("On-demand reporter emits only when ReportNow is called")
    {
    }

  private:
    void DoRun() override
    {
        Ptr<MobilityModel> ue = MakeStaticMob(GeodeticWgs84ToEcef(45.0, 90.0, 100.0));
        NtnRrcHelper helper;
        Ptr<NtnUeLocationReporter> rep =
            helper.InstallUeLocationReporter(ue,
                                             LocationReportMode::OnDemand,
                                             MilliSeconds(50),
                                             0.0);
        rep->Start();
        Simulator::Schedule(MilliSeconds(10), [rep]() { rep->ReportNow(); });
        Simulator::Stop(MilliSeconds(500));
        Simulator::Run();

        NS_TEST_ASSERT_MSG_EQ(rep->HasReport(), true, "No on-demand report");
        NS_TEST_EXPECT_MSG_EQ(rep->GetLatestReport().reportSequence, 1u,
                              "On-demand should fire exactly once");
        rep->Stop();
        Simulator::Destroy();
    }
};

// ----------------------------------------------------------------------------
// NTN-DRX tests
// ----------------------------------------------------------------------------

/// SM ticks through Active → ShortSleep → OnDuration cycle.
class DrxStandardCycleTest : public TestCase
{
  public:
    DrxStandardCycleTest()
        : TestCase("DRX standard cycle visits Active, ShortSleep, and OnDuration")
    {
    }

  private:
    void DoRun() override
    {
        NtnDrxConfig cfg;
        cfg.longCycle = MilliSeconds(320);
        cfg.shortCycle = MilliSeconds(20);
        cfg.onDuration = MilliSeconds(5);
        cfg.inactivityTimer = MilliSeconds(10);
        cfg.shortCycleCount = 2;

        NtnRrcHelper helper;
        Ptr<NtnDrxStateMachine> drx = helper.InstallDrx(cfg);
        std::set<int> seenStates;
        drx->TraceConnectWithoutContext(
            "StateChange",
            MakeCallback(+[](std::set<int>* st, DrxState prev, DrxState next, Time) {
                st->insert(static_cast<int>(next));
            }).Bind(&seenStates));
        drx->Start();
        Simulator::Stop(Seconds(1.0));
        Simulator::Run();

        NS_TEST_EXPECT_MSG_EQ(seenStates.count(static_cast<int>(DrxState::ShortSleep)), 1u,
                              "Never entered ShortSleep");
        NS_TEST_EXPECT_MSG_EQ(seenStates.count(static_cast<int>(DrxState::OnDuration)), 1u,
                              "Never entered OnDuration");
        // Sanity: time spent in sleep states should dominate over onDuration.
        const Time tOff = drx->GetTimeInState(DrxState::ShortSleep) +
                          drx->GetTimeInState(DrxState::LongSleep);
        const Time tAwake = drx->GetTimeInState(DrxState::Active) +
                            drx->GetTimeInState(DrxState::OnDuration);
        NS_TEST_EXPECT_MSG_GT(tOff.GetSeconds(), tAwake.GetSeconds(),
                              "Sleep should dominate active in 1 s window");

        drx->Stop();
        Simulator::Destroy();
    }
};

/// NotifyDataActivity forces the SM back into Active.
class DrxDataActivityTest : public TestCase
{
  public:
    DrxDataActivityTest()
        : TestCase("DRX data activity forces transition to Active")
    {
    }

  private:
    void DoRun() override
    {
        NtnDrxConfig cfg;
        cfg.longCycle = MilliSeconds(200);
        cfg.shortCycle = MilliSeconds(20);
        cfg.onDuration = MilliSeconds(5);
        cfg.inactivityTimer = MilliSeconds(10);

        NtnRrcHelper helper;
        Ptr<NtnDrxStateMachine> drx = helper.InstallDrx(cfg);
        drx->Start();

        // After 100 ms we expect SM to be sleeping; then poke activity at 105 ms.
        Simulator::Schedule(MilliSeconds(100), [drx]() {
            // Should be in some sleep state.
        });
        Simulator::Schedule(MilliSeconds(105),
                            [drx]() { drx->NotifyDataActivity(); });
        Simulator::Schedule(MilliSeconds(106),
                            [drx]() { /* state checked via final */ });

        Simulator::Stop(MilliSeconds(108));
        Simulator::Run();
        NS_TEST_EXPECT_MSG_EQ(static_cast<int>(drx->GetState()),
                              static_cast<int>(DrxState::Active),
                              "Activity did not force Active");
        drx->Stop();
        Simulator::Destroy();
    }
};

/// Pass-aware mode enters AwaitingPass when the next pass is far away.
class DrxPassAwareTest : public TestCase
{
  public:
    DrxPassAwareTest()
        : TestCase("DRX pass-aware mode sleeps deep until predicted pass")
    {
    }

  private:
    void DoRun() override
    {
        NtnDrxConfig cfg;
        cfg.longCycle = MilliSeconds(500);
        cfg.shortCycle = MilliSeconds(20);
        cfg.onDuration = MilliSeconds(5);
        cfg.inactivityTimer = MilliSeconds(10);
        cfg.passAware = true;
        cfg.passDuration = Seconds(600);

        NtnRrcHelper helper;
        Ptr<NtnDrxStateMachine> drx = helper.InstallDrx(cfg);
        drx->NotifyNextPass(Seconds(60), Seconds(600)); // pass starts in 60s
        drx->Start();

        Simulator::Stop(Seconds(20.0)); // well before the pass
        Simulator::Run();
        NS_TEST_EXPECT_MSG_EQ(static_cast<int>(drx->GetState()),
                              static_cast<int>(DrxState::AwaitingPass),
                              "Did not enter AwaitingPass");
        drx->Stop();
        Simulator::Destroy();
    }
};

/// Invalid configs are rejected.
class DrxInvalidConfigTest : public TestCase
{
  public:
    DrxInvalidConfigTest()
        : TestCase("DRX rejects malformed configs")
    {
    }

  private:
    void DoRun() override
    {
        NtnDrxConfig cfg;
        cfg.onDuration = Seconds(0); // invalid
        NS_TEST_EXPECT_MSG_EQ(cfg.IsValid(), false, "zero onDuration must be invalid");

        cfg.onDuration = MilliSeconds(5);
        cfg.shortCycle = MilliSeconds(2); // shortCycle < onDuration
        NS_TEST_EXPECT_MSG_EQ(cfg.IsValid(),
                              false,
                              "shortCycle smaller than onDuration must be invalid");
    }
};


/// RRC-4: the RAR window against real NTN geometries.
///
/// NtnTimingAdvance's header has always said that without pre-compensation
/// "RACH preambles arrive far outside their reception window", and nothing in
/// the toolkit checked it. This does the arithmetic the claim rests on: the nr
/// UE MAC arms its timeout at slotPeriod * (6 + N) from the preamble (TS 38.321
/// section 5.1.4), so whether access can complete at all is decided by whether
/// that window covers the round trip.
class NtnRachWindowGeometryTest : public TestCase
{
  public:
    NtnRachWindowGeometryTest()
        : TestCase("RRC-4: RAR window sizing against LEO and GEO round trips")
    {
    }

  private:
    void DoRun() override
    {
        // TS 38.211 Table 4.2-1.
        NS_TEST_ASSERT_MSG_EQ(NtnRachWindow::SlotPeriodForNumerology(0).GetNanoSeconds(), 1000000,
                              "mu=0 (15 kHz) is a 1 ms slot");
        NS_TEST_ASSERT_MSG_EQ(NtnRachWindow::SlotPeriodForNumerology(1).GetNanoSeconds(), 500000,
                              "mu=1 (30 kHz) is a 500 us slot");
        NS_TEST_ASSERT_MSG_EQ(NtnRachWindow::SlotPeriodForNumerology(2).GetNanoSeconds(), 250000,
                              "mu=2 (60 kHz) is a 250 us slot");

        const Time slot1 = NtnRachWindow::SlotPeriodForNumerology(1);
        const Time slot2 = NtnRachWindow::SlotPeriodForNumerology(2);

        // ---- LEO-600, 30 kHz. The interesting case: it only just fails. ----
        const Time rttLeo = NtnRachWindow::RoundTripForSlantRange(600e3);
        NS_TEST_ASSERT_MSG_EQ_TOL(rttLeo.GetSeconds(), 2.0 * 600e3 / 299792458.0, 1e-9,
                                  "round trip is twice the slant over c");

        NtnRachWindowVerdict leo = NtnRachWindow::Evaluate(rttLeo, slot1);
        // 4.0036 ms round trip + one slot of gNB turnaround = 4.5036 ms.
        NS_TEST_ASSERT_MSG_EQ_TOL(leo.requiredWindow.GetSeconds(), rttLeo.GetSeconds() + 500e-6,
                                  1e-9, "the window must cover the flight plus the turnaround");
        NS_TEST_ASSERT_MSG_EQ(leo.requiredWindowSize, 4u,
                              "LEO-600 at 30 kHz needs N=4; nr's default of 3 buys 4.500 ms "
                              "against a 4.5036 ms requirement, so it misses by 3 microseconds. "
                              "That margin is the whole point: the failure is not dramatic, it "
                              "is arithmetic, and it would look like an unexplained attach "
                              "failure in a run");
        NS_TEST_ASSERT_MSG_EQ(leo.fits, true, "N=4 is inside nr's [2,10] range");
        NS_TEST_ASSERT_MSG_EQ(leo.appliedWindowSize, 4, "so it is applied unchanged");
        NS_TEST_ASSERT_MSG_EQ(leo.shortfall.IsZero(), true, "and leaves no shortfall");
        NS_TEST_ASSERT_MSG_GT(leo.configuredWindow.GetSeconds(), leo.requiredWindow.GetSeconds(),
                              "the applied window actually covers the requirement");

        // The default really is short. This is the assertion that makes the
        // fix a fix rather than a preference.
        const Time defaultWindow = slot1 * (NtnRachWindow::kNrWindowBaseSlots + 3);
        NS_TEST_ASSERT_MSG_LT(defaultWindow.GetSeconds(), leo.requiredWindow.GetSeconds(),
                              "nr's default RaResponseWindowSize of 3 does NOT cover LEO-600");

        // ---- LEO-600 at 60 kHz: halving the slot halves the window. ----
        NtnRachWindowVerdict leo60 = NtnRachWindow::Evaluate(rttLeo, slot2);
        NS_TEST_ASSERT_MSG_EQ(leo60.fits, false,
                              "the same orbit at 60 kHz needs N=12, beyond nr's cap of 10: the "
                              "window is counted in slots, so a shorter slot buys less time for "
                              "the same numeric setting");
        NS_TEST_ASSERT_MSG_EQ(leo60.appliedWindowSize, NtnRachWindow::kNrWindowSizeMax,
                              "the applied value clamps at the maximum");
        NS_TEST_ASSERT_MSG_GT(leo60.shortfall.GetSeconds(), 0.0,
                              "and the shortfall is reported rather than hidden by the clamp");

        // ---- GEO: not close, and the number should say so. ----
        const Time rttGeo = NtnRachWindow::RoundTripForSlantRange(35786e3);
        NtnRachWindowVerdict geo = NtnRachWindow::Evaluate(rttGeo, slot1);
        NS_TEST_ASSERT_MSG_EQ(geo.fits, false, "GEO cannot fit nr's RAR window");
        NS_TEST_ASSERT_MSG_GT(geo.requiredWindowSize, 400u,
                              "GEO needs N in the hundreds against a cap of 10, so this is a "
                              "structural limit of the stack and not a tuning question. TR 38.821 "
                              "section 7.3 solves it by offsetting the window START with "
                              "ta-Common, which nr v3.3 does not implement");
        NS_TEST_ASSERT_MSG_GT(geo.shortfall.GetMilliSeconds(), 200,
                              "the shortfall is the better part of the round trip");

        // A longer round trip can never need a smaller window.
        NS_TEST_ASSERT_MSG_GT(geo.requiredWindowSize, leo.requiredWindowSize,
                              "the requirement is monotone in the round trip");

        // Degenerate inputs must not produce a confident answer.
        NtnRachWindowVerdict none = NtnRachWindow::Evaluate(rttLeo, Time());
        NS_TEST_ASSERT_MSG_EQ(none.requiredWindowSize, 0u,
                              "with no numerology there is no window to report");
    }
};


/// RRC-5: TS 38.133 reporting levels. A MeasurementReport carries level
/// indices, not the floating-point dB a simulator happens to hold, and a
/// consumer that reads back an unquantized value is reading something the air
/// interface cannot express.
class NtnMeasQuantizationTest : public TestCase
{
  public:
    NtnMeasQuantizationTest()
        : TestCase("RRC-5: measurement quantities quantize per TS 38.133")
    {
    }

  private:
    void DoRun() override
    {
        // ---- RSRP: TS 38.133 Table 10.1.6.1-1, 1 dB steps from -156 dBm ----
        NS_TEST_ASSERT_MSG_EQ(NtnMeasQuantity::RsrpToLevel(-156.0), 0,
                              "-156 dBm is the bottom of the RSRP reporting range");
        NS_TEST_ASSERT_MSG_EQ(NtnMeasQuantity::RsrpToLevel(-100.0), 56,
                              "RSRP_LEV = dBm + 156, so -100 dBm is level 56");
        NS_TEST_ASSERT_MSG_EQ_TOL(NtnMeasQuantity::LevelToRsrpDbm(56), -100.0, 1e-9,
                                  "and the inverse returns the dBm");
        // Saturation, not wraparound. A UE deep in a fade must not report the
        // best possible signal.
        NS_TEST_ASSERT_MSG_EQ(NtnMeasQuantity::RsrpToLevel(-200.0), 0,
                              "below the range saturates at 0, it does not wrap to 127");
        NS_TEST_ASSERT_MSG_EQ(NtnMeasQuantity::RsrpToLevel(+50.0), NtnMeasQuantity::kLevelMax,
                              "above the range saturates at 127");
        NS_TEST_ASSERT_MSG_EQ(NtnMeasQuantity::RsrpToLevel(std::nan("")), 0,
                              "an unmeasured quantity must not become a confident level");

        // ---- RSRQ: Table 10.1.11.1-1, 0.5 dB steps from -43 dB ----
        NS_TEST_ASSERT_MSG_EQ(NtnMeasQuantity::RsrqToLevel(-43.0), 0, "RSRQ floor");
        NS_TEST_ASSERT_MSG_EQ(NtnMeasQuantity::RsrqToLevel(-20.0), 46,
                              "RSRQ_LEV = (dB + 43) * 2");
        NS_TEST_ASSERT_MSG_EQ_TOL(NtnMeasQuantity::LevelToRsrqDb(46), -20.0, 1e-9, "RSRQ inverse");

        // ---- SINR: Table 10.1.16.1-1, 0.5 dB steps from -23 dB ----
        NS_TEST_ASSERT_MSG_EQ(NtnMeasQuantity::SinrToLevel(-23.0), 0, "SINR floor");
        NS_TEST_ASSERT_MSG_EQ(NtnMeasQuantity::SinrToLevel(14.0), 74,
                              "SINR_LEV = (dB + 23) * 2, so 14 dB is level 74");
        NS_TEST_ASSERT_MSG_EQ_TOL(NtnMeasQuantity::LevelToSinrDb(74), 14.0, 1e-9, "SINR inverse");

        // The step size must actually be a step: two values inside one 0.5 dB
        // bin land on the same level, and the next bin does not.
        NS_TEST_ASSERT_MSG_EQ(NtnMeasQuantity::SinrToLevel(14.1),
                              NtnMeasQuantity::SinrToLevel(14.0),
                              "0.4 dB apart is inside one SINR bin");
        NS_TEST_ASSERT_MSG_NE(NtnMeasQuantity::SinrToLevel(14.6),
                              NtnMeasQuantity::SinrToLevel(14.0),
                              "0.6 dB apart crosses a bin boundary; equal levels here would mean "
                              "the quantization step is wrong");
    }
};

/// RRC-5: the MeasurementReport survives a wire round trip, and a malformed
/// buffer is refused rather than half-parsed.
class NtnMeasReportCodecTest : public TestCase
{
  public:
    NtnMeasReportCodecTest()
        : TestCase("RRC-5: MeasurementReport round-trips and rejects malformed input")
    {
    }

  private:
    void DoRun() override
    {
        NtnMeasurementReport tx;
        tx.measId = 7;
        tx.servingCell.physCellId = 41;
        tx.servingCell.rsrpLevel = NtnMeasQuantity::RsrpToLevel(-98.5);
        tx.servingCell.haveRsrp = true;
        tx.servingCell.sinrLevel = NtnMeasQuantity::SinrToLevel(11.5);
        tx.servingCell.haveSinr = true;
        // RSRQ deliberately absent: an optional field left out must come back
        // out, not come back as level 0 "measured".
        tx.servingTimingAdvance = MicroSeconds(4271);
        tx.servingSlantRangeM = 640123.5;
        for (uint16_t i = 0; i < 3; ++i)
        {
            NtnMeasResultNr n;
            n.physCellId = static_cast<uint16_t>(100 + i);
            n.rsrpLevel = NtnMeasQuantity::RsrpToLevel(-105.0 - i);
            n.haveRsrp = true;
            tx.neighbours.push_back(n);
        }

        uint8_t buf[256];
        const std::size_t n = NtnMeasReportCodec::Serialise(tx, buf, sizeof(buf));
        NS_TEST_ASSERT_MSG_EQ(n, NtnMeasReportCodec::SerialisedBytes(3),
                              "the encoded size must match the advertised size");

        NtnMeasurementReport rx;
        NS_TEST_ASSERT_MSG_EQ(NtnMeasReportCodec::Parse(buf, n, rx), true, "parse succeeds");
        NS_TEST_ASSERT_MSG_EQ(rx.measId, tx.measId, "measId survives");
        NS_TEST_ASSERT_MSG_EQ(rx.servingCell.physCellId, 41, "serving PCI survives");
        NS_TEST_ASSERT_MSG_EQ(rx.servingCell.rsrpLevel, tx.servingCell.rsrpLevel,
                              "the RSRP LEVEL survives, not a re-derived dB");
        NS_TEST_ASSERT_MSG_EQ(rx.servingCell.haveRsrp, true, "present stays present");
        NS_TEST_ASSERT_MSG_EQ(rx.servingCell.haveRsrq, false,
                              "an omitted optional quantity must stay omitted; coming back as "
                              "level 0 would report -43 dB as a measurement");
        NS_TEST_ASSERT_MSG_EQ(rx.servingTimingAdvance, tx.servingTimingAdvance, "TA survives");
        NS_TEST_ASSERT_MSG_EQ_TOL(rx.servingSlantRangeM, tx.servingSlantRangeM, 1e-6,
                                  "slant range survives");
        NS_TEST_ASSERT_MSG_EQ(rx.neighbours.size(), 3u, "all neighbours survive");
        for (std::size_t i = 0; i < 3; ++i)
        {
            NS_TEST_ASSERT_MSG_EQ(rx.neighbours[i].physCellId, tx.neighbours[i].physCellId,
                                  "neighbour PCI survives in order");
            NS_TEST_ASSERT_MSG_EQ(rx.neighbours[i].rsrpLevel, tx.neighbours[i].rsrpLevel,
                                  "neighbour RSRP level survives");
        }

        // Truncation must be refused, not half-parsed into a plausible report.
        NtnMeasurementReport junk;
        NS_TEST_ASSERT_MSG_EQ(NtnMeasReportCodec::Parse(buf, n - 1, junk), false,
                              "a buffer one byte short is refused");
        NS_TEST_ASSERT_MSG_EQ(NtnMeasReportCodec::Parse(buf, 3, junk), false,
                              "a buffer shorter than the fixed part is refused");
        NS_TEST_ASSERT_MSG_EQ(NtnMeasReportCodec::Parse(nullptr, n, junk), false,
                              "a null buffer is refused");

        // TS 38.331 caps measResultNeighCells at 8.
        NtnMeasurementReport tooMany = tx;
        tooMany.neighbours.resize(NtnMeasurementReport::kMaxNeighbours + 1);
        NS_TEST_ASSERT_MSG_EQ(NtnMeasReportCodec::Serialise(tooMany, buf, sizeof(buf)), 0u,
                              "more than 8 neighbours is not encodable");
        // A count byte beyond the cap must be rejected on parse too, not
        // trusted into an over-long resize.
        uint8_t evil[256];
        std::memcpy(evil, buf, n);
        evil[1] = 200;
        NS_TEST_ASSERT_MSG_EQ(NtnMeasReportCodec::Parse(evil, n, junk), false,
                              "a neighbour count beyond the cap is rejected");

        // A short output buffer must write nothing rather than a partial report.
        uint8_t tiny[8];
        NS_TEST_ASSERT_MSG_EQ(NtnMeasReportCodec::Serialise(tx, tiny, sizeof(tiny)), 0u,
                              "an undersized output buffer yields no bytes");
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
        AddTestCase(new NtnRachWindowGeometryTest, TestCase::Duration::QUICK);
        AddTestCase(new NtnMeasQuantizationTest, TestCase::Duration::QUICK);
        AddTestCase(new NtnMeasReportCodecTest, TestCase::Duration::QUICK);
        AddTestCase(new NtnTimingAdvanceRegenerativeDriftRateTest, TestCase::Duration::QUICK);
        AddTestCase(new Sib19CodecRoundTripTest, TestCase::Duration::QUICK);
        AddTestCase(new Sib19CodecRejectsTruncatedTest, TestCase::Duration::QUICK);
        AddTestCase(new Sib19BroadcasterTickTest, TestCase::Duration::QUICK);
        AddTestCase(new GeodeticConversionRoundTripTest, TestCase::Duration::QUICK);
        AddTestCase(new PeriodicLocationReporterTest, TestCase::Duration::QUICK);
        AddTestCase(new EventTriggeredReporterTest, TestCase::Duration::QUICK);
        AddTestCase(new OnDemandReporterTest, TestCase::Duration::QUICK);
        AddTestCase(new DrxStandardCycleTest, TestCase::Duration::QUICK);
        AddTestCase(new DrxDataActivityTest, TestCase::Duration::QUICK);
        AddTestCase(new DrxPassAwareTest, TestCase::Duration::QUICK);
        AddTestCase(new DrxInvalidConfigTest, TestCase::Duration::QUICK);
        AddTestCase(new Sib19KOffsetPopulatedTest, TestCase::Duration::QUICK);
        AddTestCase(new Sib19FieldsPopulatedAndInRangeTest, TestCase::Duration::QUICK);
        AddTestCase(new TimingAdvancePayloadModeTest, TestCase::Duration::QUICK);
        AddTestCase(new Sib19KOffsetSinkTest, TestCase::Duration::QUICK);
    }
};

static NtnRrcTestSuite g_ntnRrcTestSuite; //!< static instance registers the suite
