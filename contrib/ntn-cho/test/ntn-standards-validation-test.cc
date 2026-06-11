/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only
//
// WS5 standards-validation tests (AI_NATIVE_ORAN_NTN plan): the toolkit's
// mobility is checked against OFFICIAL orbital theory, not against itself:
//   1. SGP4 vs Kepler: a 550 km / 53 deg Walker element must orbit at
//      v = sqrt(mu/a) (~7.59 km/s) with radius a = Re + h and period
//      T = 2 pi sqrt(a^3/mu) (~95.6 min), within SGP4's perturbation budget;
//   2. Doppler physics: the radial-velocity profile of a zenith pass must be
//      ~0 at culmination and bounded by the orbital speed, with the Doppler
//      shift f*v_r/c at 2 GHz inside the published LEO envelope (< 60 kHz);
//   3. ENU pass geometry: the projected serving element culminates near
//      zenith at t=0 and its elevation decreases monotonically afterwards;
//   4. the spherical-Earth elevation/slant relation used by the new CHO
//      elevation trigger round-trips against direct geometry.

#include "ns3/ntn-tr38811-mobility-model.h"
#include "ns3/sgp4-mobility-model.h"
#include "ns3/simulator.h"
#include "ns3/test.h"
#include "ns3/walker-constellation.h"

#include <algorithm>
#include <cmath>

using namespace ns3;
using namespace ns3::ntncon;

namespace
{
constexpr double kMu = 3.986004418e14; // m^3/s^2
constexpr double kRe = 6371e3;

Ptr<Sgp4MobilityModel>
MakeElement(double altKm, double inclDeg, uint32_t index = 0)
{
    WalkerConfig cfg;
    cfg.num_planes = 1;
    cfg.total_sats = 80;
    cfg.altitude_km = altKm;
    cfg.inclination_deg = inclDeg;
    cfg.epoch_unix_s = 1735689600.0;
    const auto elements = WalkerConstellation::BuildDelta(cfg);
    Ptr<Sgp4MobilityModel> m = CreateObject<Sgp4MobilityModel>();
    m->SetElements(elements[index]);
    return m;
}
} // namespace

class Sgp4KeplerTest : public TestCase
{
  public:
    Sgp4KeplerTest()
        : TestCase("SGP4 propagation matches Keplerian theory (550 km / 53 deg)")
    {
    }

  private:
    void DoRun() override
    {
        Ptr<Sgp4MobilityModel> sat = MakeElement(550.0, 53.0);
        const double a = kRe + 550e3;
        const double vTheory = std::sqrt(kMu / a);            // 7585 m/s
        const double tTheory = 2.0 * M_PI * std::sqrt(a * a * a / kMu); // 5739 s

        // Sample radius + speed over 10 min via the simulator clock.
        double sumR = 0, sumV = 0;
        int n = 0;
        for (double t = 0; t <= 600.0; t += 30.0)
        {
            Simulator::Schedule(Seconds(t), [sat, &sumR, &sumV, &n] {
                const Vector p = sat->GetPosition(); // ECEF
                const Vector v = sat->GetVelocity();
                sumR += std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
                // ECEF speed includes Earth-rotation coupling (<= 0.5 km/s);
                // compare against inertial theory with that budget.
                sumV += std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
                ++n;
            });
        }
        Simulator::Stop(Seconds(601.0));
        Simulator::Run();

        const double meanR = sumR / n;
        const double meanV = sumV / n;
        NS_TEST_ASSERT_MSG_EQ_TOL(meanR, a, 15e3,
                                  "orbital radius = Re + h within SGP4 budget");
        NS_TEST_ASSERT_MSG_EQ_TOL(meanV, vTheory, 500.0,
                                  "orbital speed ~ sqrt(mu/a) within ECEF coupling");

        // Period from true angular rate: track the position angle over a
        // quarter orbit and extrapolate.
        Simulator::Destroy();
        Ptr<Sgp4MobilityModel> sat2 = MakeElement(550.0, 53.0);
        Vector p0;
        Vector p1;
        Simulator::Schedule(Seconds(0.0), [sat2, &p0] { p0 = sat2->GetPosition(); });
        const double dt = tTheory / 4.0;
        Simulator::Schedule(Seconds(dt), [sat2, &p1] { p1 = sat2->GetPosition(); });
        Simulator::Stop(Seconds(dt + 1.0));
        Simulator::Run();
        const double dot = (p0.x * p1.x + p0.y * p1.y + p0.z * p1.z) /
                           (std::hypot(p0.x, p0.y, p0.z) * std::hypot(p1.x, p1.y, p1.z));
        // After a quarter Keplerian period the geocentric angle must be near
        // 90 deg (Earth rotation shifts the ECEF angle by a few degrees).
        const double angleDeg = std::acos(std::clamp(dot, -1.0, 1.0)) * 180.0 / M_PI;
        NS_TEST_ASSERT_MSG_EQ_TOL(angleDeg, 90.0, 8.0,
                                  "quarter-period arc consistent with T = 2pi sqrt(a^3/mu)");
        Simulator::Destroy();
    }
};

class DopplerEnvelopeTest : public TestCase
{
  public:
    DopplerEnvelopeTest()
        : TestCase("zenith-pass Doppler inside the published LEO envelope")
    {
    }

  private:
    void DoRun() override
    {
        Ptr<Sgp4MobilityModel> sat = MakeElement(550.0, 53.0);
        double subLat, subLon, subAlt;
        sat->GetGeodetic(subLat, subLon, subAlt);
        Ptr<NtnEnuProjectionMobilityModel> enu = CreateObject<NtnEnuProjectionMobilityModel>();
        enu->SetSource(sat);
        enu->SetReference(subLat, subLon, 0.0);
        const Vector gnd(0.0, 0.0, 0.0);

        // Radial velocity from 1 s slant-range differencing across the pass.
        double prevSlant = -1.0;
        double maxAbsVr = 0.0;
        double vrAtZenith = 1e9;
        for (double t = 0.0; t <= 240.0; t += 1.0)
        {
            Simulator::Schedule(Seconds(t), [enu, gnd, t, &prevSlant, &maxAbsVr,
                                             &vrAtZenith] {
                const Vector p = enu->GetPosition();
                const double slant =
                    std::hypot(p.x - gnd.x, p.y - gnd.y, p.z - gnd.z);
                if (prevSlant >= 0.0)
                {
                    const double vr = slant - prevSlant; // m/s over 1 s
                    maxAbsVr = std::max(maxAbsVr, std::abs(vr));
                    if (t <= 2.0)
                    {
                        vrAtZenith = std::min(vrAtZenith, std::abs(vr));
                    }
                }
                prevSlant = slant;
            });
        }
        Simulator::Stop(Seconds(241.0));
        Simulator::Run();

        // At culmination the radial velocity ~ 0 (Doppler null).
        NS_TEST_ASSERT_MSG_LT(vrAtZenith, 120.0, "Doppler null at culmination");
        // |v_r| can never exceed the orbital speed.
        NS_TEST_ASSERT_MSG_LT(maxAbsVr, 7600.0, "radial velocity bounded by v_orb");
        // 2 GHz Doppler envelope: f * v_r / c < 60 kHz for LEO-550 (published
        // LEO S-band envelope, e.g. TR 38.811 Doppler analysis ~ +/-48 kHz).
        const double maxDopplerHz = 2.0e9 * maxAbsVr / 299792458.0;
        NS_TEST_ASSERT_MSG_LT(maxDopplerHz, 60e3, "S-band Doppler inside envelope");
        NS_TEST_ASSERT_MSG_GT(maxDopplerHz, 5e3, "pass really moved (Doppler > 5 kHz)");
        Simulator::Destroy();
    }
};

class EnuPassGeometryTest : public TestCase
{
  public:
    EnuPassGeometryTest()
        : TestCase("ENU projection culminates at zenith and decays monotonically")
    {
    }

  private:
    void DoRun() override
    {
        Ptr<Sgp4MobilityModel> sat = MakeElement(550.0, 53.0);
        double subLat, subLon, subAlt;
        sat->GetGeodetic(subLat, subLon, subAlt);
        Ptr<NtnEnuProjectionMobilityModel> enu = CreateObject<NtnEnuProjectionMobilityModel>();
        enu->SetSource(sat);
        enu->SetReference(subLat, subLon, 0.0);

        auto elevDeg = [enu] {
            const Vector p = enu->GetPosition();
            const double horiz = std::max(std::hypot(p.x, p.y), 1e-3);
            return std::atan2(p.z, horiz) * 180.0 / M_PI;
        };

        double e0 = 0;
        std::vector<double> series;
        Simulator::Schedule(Seconds(0.0), [&e0, elevDeg] { e0 = elevDeg(); });
        for (double t = 30.0; t <= 300.0; t += 30.0)
        {
            Simulator::Schedule(Seconds(t),
                                [&series, elevDeg] { series.push_back(elevDeg()); });
        }
        Simulator::Stop(Seconds(301.0));
        Simulator::Run();

        NS_TEST_ASSERT_MSG_GT(e0, 85.0, "serving element starts at zenith");
        double prev = e0;
        for (double e : series)
        {
            NS_TEST_ASSERT_MSG_LT(e, prev + 0.2, "elevation decays monotonically");
            prev = e;
        }
        NS_TEST_ASSERT_MSG_LT(series.back(), 65.0, "5 min into the pass it has set visibly");
        Simulator::Destroy();
    }
};

class ElevationSlantRelationTest : public TestCase
{
  public:
    ElevationSlantRelationTest()
        : TestCase("spherical elevation/slant relation matches direct geometry")
    {
    }

  private:
    void DoRun() override
    {
        // sin(elev) = ((R+h)^2 - R^2 - d^2) / (2 R d) — the relation used by
        // the CHO elevation trigger. Cross-check at known angles for h=550 km.
        const double h = 550e3;
        auto slantOf = [h](double elevDeg) {
            const double e = elevDeg * M_PI / 180.0;
            const double rs = kRe + h;
            // d = -Re sin(e) + sqrt(rs^2 - Re^2 cos^2(e))
            return -kRe * std::sin(e) +
                   std::sqrt(rs * rs - kRe * kRe * std::cos(e) * std::cos(e));
        };
        auto elevOf = [h](double d) {
            const double rs = kRe + h;
            return std::asin((rs * rs - kRe * kRe - d * d) / (2.0 * kRe * d)) * 180.0 /
                   M_PI;
        };
        for (double e : {90.0, 60.0, 30.0, 10.0})
        {
            NS_TEST_ASSERT_MSG_EQ_TOL(elevOf(slantOf(e)), e, 0.01,
                                      "round-trip at " << e << " deg");
        }
        NS_TEST_ASSERT_MSG_EQ_TOL(slantOf(90.0), h, 1.0, "zenith slant = altitude");
        // TR 38.821 LEO-600 check: at 10 deg the slant is ~1932 km (h=600 km).
        const double h600 = 600e3;
        const double rs = kRe + h600;
        const double e10 = 10.0 * M_PI / 180.0;
        const double d10 = -kRe * std::sin(e10) +
                           std::sqrt(rs * rs - kRe * kRe * std::cos(e10) * std::cos(e10));
        NS_TEST_ASSERT_MSG_EQ_TOL(d10 / 1e3, 1932.0, 15.0,
                                  "TR 38.821 LEO-600 10-deg slant ~1932 km");
    }
};

class NtnStandardsValidationTestSuite : public TestSuite
{
  public:
    NtnStandardsValidationTestSuite()
        : TestSuite("ntn-standards-validation", Type::UNIT)
    {
        AddTestCase(new Sgp4KeplerTest, Duration::QUICK);
        AddTestCase(new DopplerEnvelopeTest, Duration::QUICK);
        AddTestCase(new EnuPassGeometryTest, Duration::QUICK);
        AddTestCase(new ElevationSlantRelationTest, Duration::QUICK);
    }
};

static NtnStandardsValidationTestSuite g_ntnStandardsValidationTestSuite;
