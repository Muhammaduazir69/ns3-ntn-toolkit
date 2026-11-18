/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only
//
// In-depth cases (Nov 2026 test campaign): TS 38.331 condEventT1 / D1 field
// ranges and entry conditions, the handover KPI recorder over a scripted
// timeline (C-plane, U-plane gap, time of stay, ping-pong per TR 36.839, RLF,
// superseded handovers), the measurement model's geometry, path-loss
// statistics and Doppler against TR 38.811 / TR 38.821, determinism, and the
// KPI recorder on real NR handovers driven through NtnRealStackHelper.

#include "ns3/constant-position-mobility-model.h"
#include "ns3/constant-velocity-mobility-model.h"
#include "ns3/double.h"
#include "ns3/ntn-cho-cond-event-ie.h"
#include "ns3/ntn-ho-kpi-recorder.h"
#include "ns3/ntn-measurement-model.h"
#include "ns3/ntn-orbit-predictor.h"
#include "ns3/nr-epc-helper.h"
#include "ns3/ntn-real-stack-helper.h"
#include "ns3/rng-seed-manager.h"
#include "ns3/simulator.h"
#include "ns3/test.h"

#include <cmath>
#include <fstream>
#include <functional>
#include <numeric>
#include <set>
#include <vector>

using namespace ns3;

namespace
{
constexpr double kC = 299792458.0;
// The equatorial plane of the WGS-84 ellipsoid (GeoCoordinate) is a circle of
// radius a = 6378137 m, so on the equator the geometry is exactly spherical.
constexpr double kRe = 6378137.0;
} // namespace

/// TS 38.331 CondEventT1 (t1-Threshold INTEGER (0..549755813887) in 10 ms
/// since 00:00:00 on 1 January 1900; duration INTEGER (1..6000) in 100 ms) and
/// CondEventD1 (distanceThreshFromReference1/2 in 50 m steps up to 65525,
/// hysteresisLocation in 10 m steps up to 32768); entry conditions per
/// TS 38.331 section 5.5.4.15 (T1: Mt > Thresh1, leaving when
/// Mt > Thresh1 + Duration) and 5.5.4.15a (D1: Ml1 - Hys > Thresh1 and
/// Ml2 + Hys < Thresh2).
class CondEventIeConformanceTest : public TestCase
{
  public:
    CondEventIeConformanceTest()
        : TestCase("TS 38.331 condEventT1/D1: units, ranges, clamping, strict entry conditions")
    {
    }

  private:
    void DoRun() override
    {
        const double t = 1735689600.0; // 2025-01-01T00:00Z
        bool clamped = true;
        const auto ie = CondEventT1Ie::Encode(t, Seconds(12.3), &clamped);
        NS_TEST_ASSERT_MSG_EQ(clamped, false, "in range");
        NS_TEST_ASSERT_MSG_EQ(ie.t1Threshold, static_cast<uint64_t>((t + 2208988800.0) * 100.0),
                              "10 ms units since 1900 (NTP epoch, 2208988800 s before Unix)");
        NS_TEST_ASSERT_MSG_EQ(ie.duration, 123u, "100 ms units");
        NS_TEST_ASSERT_MSG_EQ_TOL(ie.ThresholdUnixSeconds(), t, 1e-6, "inverse");
        NS_TEST_ASSERT_MSG_EQ(ie.IsInside(t), false, "Mt == Thresh1 is not yet inside (strict >)");
        NS_TEST_ASSERT_MSG_EQ(ie.IsInside(t + 0.001), true, "just after");
        NS_TEST_ASSERT_MSG_EQ(ie.IsInside(t + 12.3), true, "the last instant is inside");
        NS_TEST_ASSERT_MSG_EQ(ie.IsInside(t + 12.301), false, "leaving after Thresh1 + Duration");
        const auto lo = CondEventT1Ie::Encode(t, MilliSeconds(10), &clamped);
        NS_TEST_ASSERT_MSG_EQ(lo.duration, 1u, "duration floor 1 (100 ms)");
        NS_TEST_ASSERT_MSG_EQ(clamped, true, "and reported as clamped");
        const auto hi = CondEventT1Ie::Encode(t, Seconds(900), &clamped);
        NS_TEST_ASSERT_MSG_EQ(hi.duration, 6000u, "duration ceiling 6000 (600 s)");
        NS_TEST_ASSERT_MSG_EQ(clamped, true, "clamped");
        const auto before1900 = CondEventT1Ie::Encode(-2208988800.0 - 10.0, Seconds(1), &clamped);
        NS_TEST_ASSERT_MSG_EQ(before1900.t1Threshold, 0u, "before 1900 clamps to 0");
        NS_TEST_ASSERT_MSG_EQ(clamped, true, "clamped");
        // 2^39 - 1 x 10 ms is about 174 years after 1900: year 2074 overflows.
        const auto far = CondEventT1Ie::Encode(3.5e9, Seconds(1), &clamped);
        NS_TEST_ASSERT_MSG_EQ(far.t1Threshold, CondEventT1Ie::kThresholdMax, "39-bit ceiling");
        NS_TEST_ASSERT_MSG_EQ(clamped, true, "clamped");

        NS_TEST_ASSERT_MSG_EQ(CondEventD1Ie::EncodeThreshold(1000.0, &clamped), 20u, "50 m steps");
        NS_TEST_ASSERT_MSG_EQ(CondEventD1Ie::EncodeThreshold(1024.0, &clamped), 20u, "rounded");
        NS_TEST_ASSERT_MSG_EQ(CondEventD1Ie::EncodeThreshold(4e6, &clamped), 65525u, "range max");
        NS_TEST_ASSERT_MSG_EQ(clamped, true, "4000 km is beyond 65525 x 50 m");
        NS_TEST_ASSERT_MSG_EQ(CondEventD1Ie::EncodeThreshold(-5.0, &clamped), 0u, "negative clamps");
        NS_TEST_ASSERT_MSG_EQ(CondEventD1Ie::EncodeHysteresis(330.0, &clamped), 33u, "10 m steps");
        NS_TEST_ASSERT_MSG_EQ(CondEventD1Ie::EncodeHysteresis(1e6, &clamped), 32768u, "max");
        CondEventD1Ie d1;
        d1.distanceThreshFromReference1 = 20; // 1000 m
        d1.distanceThreshFromReference2 = 10; // 500 m
        d1.hysteresisLocation = 5;            // 50 m
        NS_TEST_ASSERT_MSG_EQ(d1.ServingLegMet(1050.0), false, "Ml1 - Hys == Thresh1 is not met");
        NS_TEST_ASSERT_MSG_EQ(d1.ServingLegMet(1050.1), true, "strictly beyond is met");
        NS_TEST_ASSERT_MSG_EQ(d1.CandidateLegMet(450.0), false, "Ml2 + Hys == Thresh2 is not met");
        NS_TEST_ASSERT_MSG_EQ(d1.CandidateLegMet(449.9), true, "strictly inside is met");
    }
};

/// The KPI recorder over a scripted timeline under Simulator::Run.
/// Definitions: C-plane = HandoverEndOk - HandoverStart; U-plane gap = first
/// app packet after the end - last packet before the start; time of stay =
/// time on the source since the previous handover (or attach); ping-pong per
/// TR 36.839 section 5.3 (a return to the previous source within the minimum
/// time of stay, 1 s). Failure paths: a failed handover, a handover end without
/// a start, a new start while one is open (the open one is closed as a failure,
/// it used to stay open forever and was never counted), and RLF counting.
class KpiRecorderScriptedTimelineTest : public TestCase
{
  public:
    KpiRecorderScriptedTimelineTest()
        : TestCase("HO KPI recorder: C-plane, U-plane gap, ToS, ping-pong (TR 36.839), failures")
    {
    }

  private:
    void DoRun() override
    {
        auto r = CreateObject<NtnHoKpiRecorder>();
        r->SetPingPongWindow(Seconds(1));
        auto at = [](double s, std::function<void()> f) { Simulator::Schedule(Seconds(s), f); };
        at(0.10, [r]() { r->NotifyConnected(0, 1); });
        // App traffic every 10 ms, paused around each handover.
        for (int k = 0; k < 600; ++k)
        {
            const double s = 0.2 + 0.01 * k;
            // Gaps: 2.00..2.05 s, 2.50..2.55 s and 4.00..4.19 s carry nothing.
            const bool gap = (k >= 180 && k < 186) || (k >= 230 && k < 236) || (k >= 380 && k < 400);
            if (!gap)
            {
                at(s, [r]() { r->NotifyRx(0); });
            }
        }
        at(2.001, [r]() { r->NotifyHandoverStart(0, 7, 1, 2); });
        at(2.041, [r]() { r->NotifyHandoverEnd(0, 7, 2, true); });   // 40 ms C-plane
        at(2.501, [r]() { r->NotifyHandoverStart(0, 7, 2, 1); });   // back after 0.46 s
        at(2.531, [r]() { r->NotifyHandoverEnd(0, 7, 1, true); });
        at(3.000, [r]() { r->NotifyHandoverEnd(0, 7, 1, true); });   // end without start
        at(4.001, [r]() { r->NotifyHandoverStart(0, 7, 1, 3); });
        at(4.101, [r]() { r->NotifyHandoverEnd(0, 7, 1, false); });  // failure
        at(4.150, [r]() { r->NotifyRadioLinkFailure(0, 1); });
        at(5.001, [r]() { r->NotifyHandoverStart(0, 7, 1, 4); });   // superseded ...
        at(5.051, [r]() { r->NotifyHandoverStart(0, 7, 1, 5); });   // ... by this one
        at(5.081, [r]() { r->NotifyHandoverEnd(0, 7, 5, true); });
        Simulator::Stop(Seconds(6.5));
        Simulator::Run();

        const auto& rec = r->GetRecords();
        NS_TEST_ASSERT_MSG_EQ(rec.size(), 5u, "five handover attempts recorded");
        NS_TEST_ASSERT_MSG_EQ_TOL(rec[0].CplaneMs(), 40.0, 1e-6, "C-plane 40 ms");
        NS_TEST_ASSERT_MSG_EQ_TOL(rec[0].UplaneGapMs(), 70.0, 1e-6,
                                  "U-plane gap: last Rx 1.99 s to first Rx 2.06 s");
        NS_TEST_ASSERT_MSG_EQ_TOL(rec[0].timeOfStayS, 1.901, 1e-9, "ToS since attach at 0.1 s");
        NS_TEST_ASSERT_MSG_EQ(rec[0].pingPong, false, "first handover is no ping-pong");
        NS_TEST_ASSERT_MSG_EQ(rec[1].pingPong, true, "1 -> 2 -> 1 within 0.46 s is a ping-pong");
        NS_TEST_ASSERT_MSG_EQ(rec[1].pingPong1s, true, "within 1 s");
        NS_TEST_ASSERT_MSG_EQ_TOL(rec[1].timeOfStayS, 0.46, 1e-9, "ToS on cell 2");
        NS_TEST_ASSERT_MSG_EQ(rec[2].success, false, "the failed handover");
        NS_TEST_ASSERT_MSG_EQ(rec[2].closed, true, "is closed");
        NS_TEST_ASSERT_MSG_EQ(rec[3].closed, true, "a superseded attempt is closed");
        NS_TEST_ASSERT_MSG_EQ(rec[3].success, false, "as a failure");
        NS_TEST_ASSERT_MSG_EQ(rec[4].success, true, "the superseding one succeeds");
        NS_TEST_ASSERT_MSG_EQ(rec[4].targetCell, 5u, "into cell 5");
        NS_TEST_ASSERT_MSG_EQ(r->GetHandoverCount(true), 3u, "three successes");
        NS_TEST_ASSERT_MSG_EQ(r->GetHandoverFailureCount(), 2u, "two failures");
        NS_TEST_ASSERT_MSG_EQ(r->GetPingPongCount(Seconds(1)), 1u, "one ping-pong");
        NS_TEST_ASSERT_MSG_EQ(r->GetRlfCount(0), 1u, "one RLF");
        NS_TEST_ASSERT_MSG_EQ_TOL(r->GetMedianInterArrivalMs(0), 10.0, 1e-6, "10 ms cadence");
        const std::string csv = CreateTempDirFilename("ho.csv");
        r->WriteCsv(csv);
        std::ifstream f(csv);
        std::string line;
        uint32_t lines = 0;
        while (std::getline(f, line))
        {
            ++lines;
        }
        NS_TEST_ASSERT_MSG_EQ(lines, 6u, "header plus one row per attempt");
        Simulator::Destroy();
    }
};

namespace
{
Ptr<NtnOrbitPredictor>
NadirPredictor(Ptr<ConstantVelocityMobilityModel>& satMob, double altM)
{
    satMob = CreateObject<ConstantVelocityMobilityModel>();
    satMob->SetPosition(Vector(kRe + altM, 0.0, 0.0));
    satMob->SetVelocity(Vector(0.0, 7560.0, 0.0));
    auto pred = CreateObject<NtnOrbitPredictor>();
    pred->SetKinematicsSource(1, satMob);
    pred->SetReferenceEarthRadius(kRe);
    pred->SetGeometricBeam(30.0, 4.4127);
    pred->SetSteeredBeam(true);
    return pred;
}

Ptr<NtnMeasurementModel>
MakeModel(Ptr<NtnOrbitPredictor> pred)
{
    auto m = CreateObject<NtnMeasurementModel>();
    m->SetOrbitPredictor(pred);
    m->SetCarrierFrequency(2.0e9);
    m->SetBandwidth(20.0e6);
    m->SetSatelliteTxPower(47.0);
    m->SetUeNoiseFigure(7.0);
    m->SetNtnScenario(NtnMeasurementModel::NTN_RURAL);
    return m;
}
} // namespace

/// The measurement model at a known geometry (satellite 600 km over the
/// equator, UE at nadir on the WGS-84 equator):
///  - slant 600 km, elevation 90 deg, one-way delay 2.0014 ms;
///  - over 4000 draws, the mean path loss equals the TR 38.811 Eq. (6.6-2)
///    free-space loss 32.45 + 20 log10(f GHz) + 20 log10(d m) plus the gas and
///    rural clutter terms within 0.1 dB, and its standard deviation equals the
///    shadow + scintillation spread within 10 %;
///  - SINR = RSRP - (-174 + 10 log10 B + NF) exactly;
///  - Doppler: 0 at nadir, and toward a UE at 10 deg elevation the closed form
///    f v cos(theta) / c, below TR 38.821 Table 5.3.3.1-1's 24 ppm for LEO-600.
class MeasurementModelReferenceTest : public TestCase
{
  public:
    MeasurementModelReferenceTest()
        : TestCase("Measurement model: TR 38.811 FSPL mean, spread, SINR identity, Doppler <= 24 ppm")
    {
    }

  private:
    void DoRun() override
    {
        Ptr<ConstantVelocityMobilityModel> satMob;
        auto pred = NadirPredictor(satMob, 600e3);
        auto m = MakeModel(pred);
        const GeoCoordinate nadir(0.0, 0.0, 0.0);
        double sum = 0.0;
        double sumSq = 0.0;
        const int n = 4000;
        NtnMeasurementModel::Measurement first;
        for (int k = 0; k < n; ++k)
        {
            const auto meas = m->ComputeMeasurement(nadir, 1, 0);
            if (k == 0)
            {
                first = meas;
            }
            sum += meas.pathLoss_dB;
            sumSq += meas.pathLoss_dB * meas.pathLoss_dB;
            const double noise = -174.0 + 10.0 * std::log10(20e6) + 7.0;
            NS_TEST_ASSERT_MSG_EQ_TOL(meas.sinr_dB, meas.rsrp_dBm - noise, 1e-9, "SINR identity");
        }
        NS_TEST_ASSERT_MSG_EQ_TOL(first.slantRange_km, 600.0, 0.5, "slant at nadir");
        NS_TEST_ASSERT_MSG_EQ_TOL(first.elevationAngle_deg, 90.0, 0.1, "zenith");
        NS_TEST_ASSERT_MSG_EQ_TOL(first.propagationDelay.GetSeconds(), 600e3 / kC, 1e-8,
                                  "2.0014 ms one way");
        const double d = first.slantRange_km * 1e3;
        const double fsplTr38811 = 32.45 + 20.0 * std::log10(2.0) + 20.0 * std::log10(d);
        const double mean = sum / n;
        const double sd = std::sqrt(std::max(0.0, sumSq / n - mean * mean));
        NS_TEST_ASSERT_MSG_EQ_TOL(mean, fsplTr38811 + 0.04 + 0.5, 0.1,
                                  "mean path loss = FSPL + 0.04 dB gas at zenith + 0.5 dB rural");
        // Spread at zenith: shadow sigma 1 dB, scintillation 0.5 sqrt(f/2) = 0.5 dB.
        NS_TEST_ASSERT_MSG_EQ_TOL(sd, std::sqrt(1.0 + 0.25), 0.1 * std::sqrt(1.25),
                                  "path-loss spread = sqrt(sigma_sf^2 + sigma_sc^2)");
        NS_TEST_ASSERT_MSG_EQ_TOL(m->ComputeDopplerShift(nadir, 1), 0.0, 1.0,
                                  "no radial velocity at nadir");

        // UE on the equator in the direction of motion, at 10 deg elevation.
        const double el = 10.0 * M_PI / 180.0;
        const double gamma = std::acos(kRe * std::cos(el) / (kRe + 600e3)) - el;
        const GeoCoordinate ue10(0.0, gamma * 180.0 / M_PI, 0.0);
        const Vector u = ue10.ToVector();
        const Vector s = satMob->GetPosition();
        const Vector los(s.x - u.x, s.y - u.y, s.z - u.z);
        const double cosTheta = (los.y * 7560.0) / (los.GetLength() * 7560.0);
        const double expect = -2.0e9 * 7560.0 * cosTheta / kC;
        const double dop = m->ComputeDopplerShift(ue10, 1);
        NS_TEST_ASSERT_MSG_EQ_TOL(dop, expect, std::abs(expect) * 0.01, "closed-form Doppler");
        NS_TEST_ASSERT_MSG_GT(dop, 0.0, "approaching satellite: positive Doppler");
        NS_TEST_ASSERT_MSG_LT(std::abs(dop), 24e-6 * 2.0e9, "within 24 ppm (TR 38.821)");
        NS_TEST_ASSERT_MSG_GT(std::abs(dop), 20e-6 * 2.0e9, "and above 20 ppm at 10 deg");
    }
};

/// Determinism of the measurement model's random terms: the same RngRun
/// reproduces the exact path-loss sequence; another run differs but keeps the
/// same mean within 0.15 dB over 2000 draws.
class MeasurementModelDeterminismTest : public TestCase
{
  public:
    MeasurementModelDeterminismTest()
        : TestCase("Measurement model: same RngRun same draws, other run same statistics")
    {
    }

  private:
    static std::vector<double> Draws(uint32_t run)
    {
        RngSeedManager::SetRun(run);
        RngSeedManager::ResetNextStreamIndex();
        Ptr<ConstantVelocityMobilityModel> satMob;
        auto m = MakeModel(NadirPredictor(satMob, 600e3));
        std::vector<double> v;
        for (int k = 0; k < 2000; ++k)
        {
            v.push_back(m->ComputeMeasurement(GeoCoordinate(0, 0, 0), 1, 0).pathLoss_dB);
        }
        return v;
    }

    void DoRun() override
    {
        const auto a = Draws(1);
        const auto b = Draws(1);
        const auto c = Draws(2);
        RngSeedManager::SetRun(1);
        NS_TEST_ASSERT_MSG_EQ((a == b), true, "identical sequence");
        NS_TEST_ASSERT_MSG_EQ((a == c), false, "another run differs");
        const double ma = std::accumulate(a.begin(), a.end(), 0.0) / a.size();
        const double mc = std::accumulate(c.begin(), c.end(), 0.0) / c.size();
        NS_TEST_ASSERT_MSG_EQ_TOL(ma, mc, 0.15, "same mean within 0.15 dB");
    }
};

/// Cross-module (ntn-cho KPI recorder on ntn-traffic / nr): two static LEO
/// cells, a UDP flow to the UE, and two commanded X2 handovers 0.5 s apart
/// (A -> B at 3 s, B -> A at 3.5 s). The recorder, attached only to the UE's
/// RRC traces and its application Rx trace, must report two successful
/// handovers matching the helper's count, a C-plane time of at least the RACH
/// slots and below 200 ms, a U-plane gap at least the C-plane time, a
/// first time of stay near the time since attach, and the return as a
/// ping-pong within 1 s.
class KpiRecorderOnRealNrHandoverTest : public TestCase
{
  public:
    KpiRecorderOnRealNrHandoverTest()
        : TestCase("Cross-module: KPI recorder on two real NR X2 handovers (C/U-plane, ping-pong)")
    {
    }

  private:
    void DoRun() override
    {
        NodeContainer sats;
        sats.Create(2);
        NodeContainer ues;
        ues.Create(1);
        for (uint32_t i = 0; i < 2; ++i)
        {
            auto mm = CreateObject<ConstantPositionMobilityModel>();
            mm->SetPosition(NtnRealStackHelper::GeodeticToEcef(40.0 + 2.0 * i, -3.0, 600e3));
            sats.Get(i)->AggregateObject(mm);
        }
        auto um = CreateObject<ConstantPositionMobilityModel>();
        um->SetPosition(NtnRealStackHelper::GeodeticToEcef(41.0, -3.0, 0.0));
        ues.Get(0)->AggregateObject(um);
        NtnRealStackHelper rs;
        rs.SetRadioBackend(NtnRealStackHelper::RadioBackend::Nr);
        rs.SetNumerology(1);
        rs.SetSimTime(Seconds(5.0));
        rs.SetOutputDir(CreateTempDirFilename("rs"));
        rs.SetHandover(true, 3.0, MilliSeconds(256));
        rs.SetAutonomousHandover(false);
        rs.SetSatEirpDensityDbwMhz(NtnRealStackHelper::kTr38821Set1SBandEirpDensityDbwMhz);
        rs.Build(sats, ues);
        rs.InstallTraffic(NtnRealStackHelper::TrafficProfile::UrllcPings, Seconds(1.0), Seconds(4.9));

        auto rec = CreateObject<NtnHoKpiRecorder>();
        rec->SetPingPongWindow(Seconds(1));
        rec->AttachUe(0, ues.Get(0));
        bool rxAttached = false;
        for (uint32_t a = 0; a < ues.Get(0)->GetNApplications() && !rxAttached; ++a)
        {
            rxAttached = rec->AttachRxApplication(0, ues.Get(0)->GetApplication(a));
        }
        uint16_t first = 0;
        Simulator::Schedule(Seconds(3.0), [&]() {
            first = rs.GetUeServingCellId(0);
            const uint16_t other = first == rs.GetGnbCellId(0) ? rs.GetGnbCellId(1) : rs.GetGnbCellId(0);
            rs.TriggerHandover(0, other);
        });
        Simulator::Schedule(Seconds(3.5), [&]() { rs.TriggerHandover(0, first); });
        Simulator::Stop(Seconds(5.0));
        Simulator::Run();

        NS_TEST_ASSERT_MSG_EQ(rxAttached, true, "the UE application exposes an Rx trace");
        NS_TEST_ASSERT_MSG_EQ(rs.GetHandoverCount(), 2u, "two completed X2 handovers");
        NS_TEST_ASSERT_MSG_EQ(rec->GetHandoverCount(true), 2u, "the recorder saw both");
        const auto& r = rec->GetRecords();
        if (r.size() == 2)
        {
            // The nr backend carries the slant delay on the backhaul, not on the
            // air interface (NtnRealStackHelper::SetAirInterfaceDelay is off), so
            // the UE-side procedure is bounded below by the RACH, i.e. a few slots.
            NS_TEST_ASSERT_MSG_GT(r[0].CplaneMs(), 1.0, "C-plane spans at least the RACH slots");
            NS_TEST_ASSERT_MSG_LT(r[0].CplaneMs(), 200.0, "and completes within 200 ms");
            NS_TEST_ASSERT_MSG_GT(r[0].UplaneGapMs() + 1e-9, r[0].CplaneMs() - 10.0,
                                  "the user-plane gap spans the control-plane procedure");
            NS_TEST_ASSERT_MSG_LT(r[0].UplaneGapMs(), 500.0, "and recovers");
            NS_TEST_ASSERT_MSG_GT(r[0].timeOfStayS, 2.0, "ToS since attach");
            NS_TEST_ASSERT_MSG_EQ(r[1].pingPong, true, "the return within 0.5 s is a ping-pong");
            NS_TEST_ASSERT_MSG_EQ(r[1].targetCell, first, "back to the first cell");
        }
        NS_TEST_ASSERT_MSG_EQ(rec->GetPingPongCount(Seconds(1)), 1u, "one ping-pong");
        Simulator::Destroy();
    }
};

/// CondEventD1 and T1 evaluated on a moving UE under Simulator::Run (TS 38.331
/// 5.5.4.15 / 5.5.4.15a). The UE drives at 30 m/s from reference 1 toward
/// reference 2 (5 km apart); with Thresh1 = 1000 m, Thresh2 = 500 m and
/// Hys = 50 m, the serving leg is met from (1050 m)/v = 35 s and the candidate
/// leg from (5000 - 450 m)/v = 151.7 s, so D1 enters at 151.7 s (sampled every
/// 10 ms, within one sample). A T1 window opening at 100 s for 12.3 s, with the
/// simulation clock anchored at a UTC epoch, is inside exactly from 100.01 s to
/// 112.3 s on the same sampling grid.
class CondEventsOnMovingUeTest : public TestCase
{
  public:
    CondEventsOnMovingUeTest()
        : TestCase("TS 38.331 D1/T1 on a moving UE: entry instants from the geometry and the UTC clock")
    {
    }

  private:
    void DoRun() override
    {
        auto ue = CreateObject<ConstantVelocityMobilityModel>();
        ue->SetPosition(Vector(0.0, 0.0, 0.0));
        ue->SetVelocity(Vector(30.0, 0.0, 0.0));
        const Vector ref1(0.0, 0.0, 0.0);
        const Vector ref2(5000.0, 0.0, 0.0);
        CondEventD1Ie d1;
        d1.distanceThreshFromReference1 = CondEventD1Ie::EncodeThreshold(1000.0);
        d1.distanceThreshFromReference2 = CondEventD1Ie::EncodeThreshold(500.0);
        d1.hysteresisLocation = CondEventD1Ie::EncodeHysteresis(50.0);
        const double epoch = 1735689600.0;
        const auto t1 = CondEventT1Ie::Encode(epoch + 100.0, Seconds(12.3));
        double servingFrom = -1.0;
        double enterD1 = -1.0;
        double t1In = -1.0;
        double t1Out = -1.0;
        for (int k = 0; k <= 20000; ++k)
        {
            Simulator::Schedule(MilliSeconds(10 * k), [&]() {
                const double t = Simulator::Now().GetSeconds();
                const Vector p = ue->GetPosition();
                const bool s1 = d1.ServingLegMet(CalculateDistance(p, ref1));
                const bool s2 = d1.CandidateLegMet(CalculateDistance(p, ref2));
                if (s1 && servingFrom < 0)
                {
                    servingFrom = t;
                }
                if (s1 && s2 && enterD1 < 0)
                {
                    enterD1 = t;
                }
                const bool in = t1.IsInside(epoch + t);
                if (in && t1In < 0)
                {
                    t1In = t;
                }
                if (!in && t1In >= 0 && t1Out < 0)
                {
                    t1Out = t;
                }
            });
        }
        Simulator::Stop(Seconds(200.01));
        Simulator::Run();
        Simulator::Destroy();
        NS_TEST_ASSERT_MSG_EQ_TOL(servingFrom, 1050.0 / 30.0, 0.011, "serving leg at 35 s");
        NS_TEST_ASSERT_MSG_EQ_TOL(enterD1, 4550.0 / 30.0, 0.011, "D1 entry at 151.67 s");
        NS_TEST_ASSERT_MSG_EQ_TOL(t1In, 100.01, 1e-6, "T1 inside strictly after Thresh1");
        NS_TEST_ASSERT_MSG_EQ_TOL(t1Out, 112.31, 1e-6, "T1 leaves after Thresh1 + Duration");
    }
};

/// The measurement model's geometry against the spherical reference of
/// TR 38.811 Section 6.6.2 (equatorial plane of WGS-84, radius a): for a UE at
/// central angle gamma from the sub-satellite point of a 600 km satellite,
/// elevation = atan((cos g - R/(R+h)) / sin g), slant = TR 38.811 d(el), and
/// one-way delay = d / c, at 10 points from nadir to 10 deg elevation. Beyond
/// the horizon the elevation is negative.
class MeasurementModelGeometrySweepTest : public TestCase
{
  public:
    MeasurementModelGeometrySweepTest()
        : TestCase("Measurement model geometry: elevation, slant and delay vs TR 38.811 at 10 points")
    {
    }

  private:
    void DoRun() override
    {
        Ptr<ConstantVelocityMobilityModel> satMob;
        auto pred = NadirPredictor(satMob, 600e3);
        auto m = MakeModel(pred);
        const double R = kRe;
        const double h = 600e3;
        uint32_t bad = 0;
        for (int k = 0; k < 10; ++k)
        {
            const double elRef = 90.0 - k * 8.888888;
            const double el = elRef * M_PI / 180.0;
            const double gamma = std::acos(R * std::cos(el) / (R + h)) - el;
            const GeoCoordinate ue(0.0, gamma * 180.0 / M_PI, 0.0);
            const auto meas = m->ComputeMeasurement(ue, 1, 0);
            const double dRef = -R * std::sin(el) + std::sqrt(std::pow(R * std::sin(el), 2) + 2 * R * h + h * h);
            bad += std::abs(meas.elevationAngle_deg - elRef) > 0.05;
            bad += std::abs(meas.slantRange_km * 1e3 - dRef) > 50.0;
            bad += std::abs(meas.propagationDelay.GetSeconds() - dRef / kC) > 2e-7;
        }
        NS_TEST_ASSERT_MSG_EQ(bad, 0u, "elevation within 0.05 deg, slant within 50 m, delay within 0.2 us");
        // A UE 30 deg of longitude away is below the horizon of a 600 km satellite.
        const auto far = m->ComputeMeasurement(GeoCoordinate(0.0, 30.0, 0.0), 1, 0);
        NS_TEST_ASSERT_MSG_LT(far.elevationAngle_deg, 0.0, "beyond the horizon: negative elevation");
    }
};

/// fixes-3: GetVisibleSatellites over satellites registered ONLY as
/// kinematics sources (CHO-2, no Initialize(), no SatMobilityModel). Two
/// satellites start 600 km above the UE's horizontal plane and fly along +y at
/// 7.56 km/s, one over the UE and one 4000 km behind it (8.5 deg elevation,
/// rising above 10 deg at y = -h / tan 10 deg, t = 79 s; the leading one sets
/// at y = +3403 km, t = 450 s). Sampled every 10 s for 900 s inside a running
/// simulation, the visible set must equal the satellites whose independently
/// computed elevation is at least 10 deg (TR 38.821 Section 6.1 minimum
/// elevation), with elevation and slant from the kinematics position, sorted
/// by the steered-beam gain (highest elevation first). The measurement model's
/// ScanVisibleBeams sees the same set. Both satellites are visible at some
/// point, and the coverage passes from the first to the second. Failure paths:
/// a stationary source is refused and never reported; a satellite nothing
/// knows is not reported. Before the fix the loop ran over Initialize()'s
/// satellites only and returned nothing here.
class VisibleSatellitesFromKinematicsTest : public TestCase
{
  public:
    VisibleSatellitesFromKinematicsTest()
        : TestCase("Orbit predictor: kinematics-only satellites are visible above the 10 deg mask")
    {
    }

  private:
    void DoRun() override
    {
        const double h = 600e3;
        std::vector<Ptr<ConstantVelocityMobilityModel>> sats;
        for (double y0 : {0.0, -4000e3})
        {
            auto m = CreateObject<ConstantVelocityMobilityModel>();
            m->SetPosition(Vector(kRe + h, y0, 0.0));
            m->SetVelocity(Vector(0.0, 7560.0, 0.0));
            sats.push_back(m);
        }
        auto pred = CreateObject<NtnOrbitPredictor>();
        pred->SetKinematicsSource(3, sats[0]);
        pred->SetKinematicsSource(5, sats[1]);
        auto still = CreateObject<ConstantPositionMobilityModel>();
        still->SetPosition(Vector(kRe + h, 0.0, 0.0));
        pred->SetKinematicsSource(7, still); // refused: zero velocity
        pred->SetReferenceEarthRadius(kRe);
        pred->SetGeometricBeam(30.0, 4.4127);
        pred->SetSteeredBeam(true);
        // Coverage gated by the elevation mask alone: the steered beam's scan
        // loss at 10 deg elevation is about -4.3 dB (cos^1.2 of a 64 deg scan),
        // below the default -3 dB gain threshold.
        pred->SetAttribute("MinGainThreshold", DoubleValue(-30.0));
        auto model = MakeModel(pred);
        const GeoCoordinate ueGeo(0.0, 0.0, 0.0);
        const Vector ue = ueGeo.ToVector();

        uint32_t mismatches = 0;
        uint32_t geomErrors = 0;
        uint32_t orderErrors = 0;
        uint32_t scanMismatches = 0;
        uint32_t seen3 = 0;
        uint32_t seen5 = 0;
        uint32_t seenBoth = 0;
        uint32_t seen7 = 0;
        double last3 = -1.0;
        double first5 = -1.0;
        for (int k = 0; k <= 90; ++k)
        {
            Simulator::Schedule(Seconds(10.0 * k), [&]() {
                const auto vis = pred->GetVisibleSatellites(ueGeo, 10.0);
                std::set<uint32_t> got;
                for (size_t j = 0; j < vis.size(); ++j)
                {
                    got.insert(vis[j].satId);
                    seen7 += (vis[j].satId == 7);
                    if (j > 0 && vis[j].bestGain_dB > vis[j - 1].bestGain_dB)
                    {
                        ++orderErrors;
                    }
                }
                std::set<uint32_t> expect;
                const uint32_t ids[2] = {3, 5};
                for (uint32_t s = 0; s < 2; ++s)
                {
                    const Vector p = sats[s]->GetPosition();
                    const Vector d(p.x - ue.x, p.y - ue.y, p.z - ue.z);
                    const double el =
                        std::asin((d.x * ue.x + d.y * ue.y + d.z * ue.z) / (d.GetLength() * ue.GetLength())) *
                        180.0 / M_PI;
                    if (el >= 10.0)
                    {
                        expect.insert(ids[s]);
                    }
                    for (const auto& v : vis)
                    {
                        if (v.satId == ids[s])
                        {
                            geomErrors += std::abs(v.elevationAngle_deg - el) > 0.01;
                            geomErrors += std::abs(v.slantRange_km * 1e3 - d.GetLength()) > 1.0;
                        }
                    }
                }
                mismatches += (got != expect);
                scanMismatches += (model->ScanVisibleBeams(ueGeo, 10.0).size() != vis.size());
                const double t = Simulator::Now().GetSeconds();
                if (got.count(3))
                {
                    ++seen3;
                    last3 = t;
                }
                if (got.count(5))
                {
                    ++seen5;
                    if (first5 < 0)
                    {
                        first5 = t;
                    }
                }
                seenBoth += (got.count(3) && got.count(5));
            });
        }
        Simulator::Stop(Seconds(901.0));
        Simulator::Run();
        Simulator::Destroy();
        NS_TEST_ASSERT_MSG_EQ(mismatches, 0u, "visible set = satellites above the 10 deg mask");
        NS_TEST_ASSERT_MSG_EQ(geomErrors, 0u, "elevation within 0.01 deg, slant within 1 m");
        NS_TEST_ASSERT_MSG_EQ(orderErrors, 0u, "sorted by beam gain, descending");
        NS_TEST_ASSERT_MSG_EQ(scanMismatches, 0u, "ScanVisibleBeams measures the same set");
        NS_TEST_ASSERT_MSG_GT(seen3, 10u, "the satellite over the UE is visible at the start");
        NS_TEST_ASSERT_MSG_GT(seen5, 10u, "the trailing satellite rises and is visible");
        NS_TEST_ASSERT_MSG_GT(seenBoth, 0u, "an overlap where both are candidates");
        NS_TEST_ASSERT_MSG_EQ_TOL(first5, 80.0, 0.5, "the trailing satellite rises at 79 s (10 s grid)");
        NS_TEST_ASSERT_MSG_EQ_TOL(last3, 450.0, 10.5, "the leading satellite sets at 450 s");
        NS_TEST_ASSERT_MSG_EQ(seen7, 0u, "a refused stationary source is never visible");
        NS_TEST_ASSERT_MSG_EQ(pred->HasUsableKinematics(7), false, "stationary source refused");
        double el = pred->ComputeElevationAngle(ueGeo, 11);
        NS_TEST_ASSERT_MSG_EQ_TOL(el, -90.0, 1e-9, "an unknown satellite is below the horizon");
    }
};

class NtnChoDepthTestSuite : public TestSuite
{
  public:
    NtnChoDepthTestSuite()
        : TestSuite("ntn-cho-depth", Type::UNIT)
    {
        AddTestCase(new CondEventIeConformanceTest, TestCase::Duration::QUICK);
        AddTestCase(new KpiRecorderScriptedTimelineTest, TestCase::Duration::QUICK);
        AddTestCase(new MeasurementModelReferenceTest, TestCase::Duration::QUICK);
        AddTestCase(new MeasurementModelDeterminismTest, TestCase::Duration::QUICK);
        AddTestCase(new KpiRecorderOnRealNrHandoverTest, TestCase::Duration::QUICK);
        AddTestCase(new CondEventsOnMovingUeTest, TestCase::Duration::QUICK);
        AddTestCase(new MeasurementModelGeometrySweepTest, TestCase::Duration::QUICK);
        AddTestCase(new VisibleSatellitesFromKinematicsTest, TestCase::Duration::QUICK);
    }
};

static NtnChoDepthTestSuite g_ntnChoDepthTestSuite;
