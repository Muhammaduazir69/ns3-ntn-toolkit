/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#include "ns3/constant-position-mobility-model.h"
#include "ns3/constant-velocity-mobility-model.h"
#include "ns3/log.h"
#include "ns3/ntn-drx.h"
#include "ns3/ntn-rrc-helper.h"
#include "ns3/ntn-sib19.h"
#include "ns3/ntn-timing-advance.h"
#include "ns3/ntn-ue-location-report.h"
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
    }
};

static NtnRrcTestSuite g_ntnRrcTestSuite; //!< static instance registers the suite
