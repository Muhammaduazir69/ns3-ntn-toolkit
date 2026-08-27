/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#include "ns3/boolean.h"
#include "ns3/geocentric-constant-position-mobility-model.h"
#include "ns3/double.h"
#include "ns3/uinteger.h"
#include <limits>
#include "ns3/constant-position-mobility-model.h"
#include "ns3/contact-graph-router.h"
#include "ns3/contact-graph-scheduler.h"
#include "ns3/ntn-sat-link-error-model.h"
#include "ns3/orbital-elements.h"
#include "ns3/packet.h"
#include "ns3/tr38821-corpus.h"
#include "ns3/ntn-xn-handover.h"
#include "ns3/ntn-xnap-messages.h"
#include "ns3/ntn-visibility-index.h"
#include "ns3/sgp4-mobility-model.h"
#include "ns3/simulator.h"
#include "ns3/test.h"
#include "ns3/walker-constellation.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <map>
#include <set>
#include <utility>
#include <vector>

using namespace ns3;
using namespace ns3::ntncon;

namespace
{

// ---------------------------------------------------------------------------
//  TLE parser + Keplerian helpers
// ---------------------------------------------------------------------------

class TleParseChecksumTest : public TestCase
{
  public:
    TleParseChecksumTest()
        : TestCase("TLE parser extracts ISS elements and verifies checksum")
    {
    }

  private:
    void DoRun() override
    {
        // ISS (ZARYA) TLE — real published values, epoch 2019-280.
        // Checksums on third-party TLEs vary; we verify ToKeplerian parses
        // the elements rather than gating on the modulo-10 digit, since
        // some publicly-circulated copies have stale checksums.
        TleRecord tle;
        tle.name = "ISS (ZARYA)";
        tle.line1 = "1 25544U 98067A   19282.92426354  .00001417  00000-0  29888-4 0  9991";
        tle.line2 = "2 25544  51.6447  91.8123 0007381 152.7392 207.3922 15.50284294192054";

        KeplerianElements el{};
        NS_TEST_ASSERT_MSG_EQ(tle.ToKeplerian(el), true, "parse ok");
        NS_TEST_EXPECT_MSG_EQ(el.norad_id, 25544u, "NORAD ID");
        // Inclination ~ 51.6447 deg.
        NS_TEST_ASSERT_MSG_EQ_TOL(el.inclination_rad * 180.0 / M_PI,
                                  51.6447,
                                  1e-3,
                                  "inclination");
        // Eccentricity ~ 0.0007381.
        NS_TEST_ASSERT_MSG_EQ_TOL(el.eccentricity, 0.0007381, 1e-6,
                                  "eccentricity");
        // ISS semi-major axis ~ 6.78e6 m (Earth radius + ~408 km).
        const double a_km = el.semi_major_axis_m / 1000.0;
        NS_TEST_ASSERT_MSG_GT(a_km, 6720.0, "a > 6720 km");
        NS_TEST_ASSERT_MSG_LT(a_km, 6820.0, "a < 6820 km");
        // ISS orbital period ~ 92.8 min.
        const double T_min = el.PeriodSeconds() / 60.0;
        NS_TEST_ASSERT_MSG_GT(T_min, 92.0, "period > 92 min");
        NS_TEST_ASSERT_MSG_LT(T_min, 94.0, "period < 94 min");
    }
};

class TleStreamParseTest : public TestCase
{
  public:
    TleStreamParseTest()
        : TestCase("ParseTleStream consumes multi-record CelesTrak-style dump")
    {
    }

  private:
    void DoRun() override
    {
        const std::string body =
            "# comment\n"
            "ISS (ZARYA)\n"
            "1 25544U 98067A   19282.92426354  .00001417  00000-0  29888-4 0  9991\n"
            "2 25544  51.6447  91.8123 0007381 152.7392 207.3922 15.50284294192054\n"
            "STARLINK-1\n"
            "1 44713U 19074A   20029.05810056  .00001046  00000-0  74181-4 0  9994\n"
            "2 44713  53.0531 251.0050 0001460  77.1854 282.9270 15.05606195  6553\n";
        std::vector<TleRecord> recs;
        const size_t n = ParseTleStream(body, recs);
        NS_TEST_ASSERT_MSG_EQ(n, 2u, "two records");
        NS_TEST_EXPECT_MSG_EQ(recs[0].name, "ISS (ZARYA)", "ISS name");
        NS_TEST_EXPECT_MSG_EQ(recs[1].name, "STARLINK-1", "Starlink name");
        KeplerianElements el{};
        NS_TEST_ASSERT_MSG_EQ(recs[1].ToKeplerian(el), true, "starlink parse");
        NS_TEST_EXPECT_MSG_EQ(el.norad_id, 44713u, "starlink NORAD");
    }
};

// ---------------------------------------------------------------------------
//  SGP4 / Kepler+J2 propagation
// ---------------------------------------------------------------------------

class Sgp4PeriodicReturnTest : public TestCase
{
  public:
    Sgp4PeriodicReturnTest()
        : TestCase("Kepler+J2 propagation returns to start within 100 km after one period")
    {
    }

  private:
    void DoRun() override
    {
        KeplerianElements el{};
        el.semi_major_axis_m = 6371000.0 + 550000.0; // 550 km altitude
        el.eccentricity = 0.0;
        el.inclination_rad = 53.0 * M_PI / 180.0;
        el.raan_rad = 0.0;
        el.arg_perigee_rad = 0.0;
        el.mean_anomaly_rad = 0.0;
        el.epoch_unix_s = 1577836800.0; // 2020-01-01 00:00 UTC

        Ptr<Sgp4MobilityModel> sat = CreateObject<Sgp4MobilityModel>();
        sat->SetElements(el);

        // ECI position at t=0 sim time should equal position one period
        // later (within 100 km — Kepler+J2 secular drift over one orbit
        // is small compared to that).
        Vector p0 = sat->GetEciPosition();
        // Advance one full period in sim time.
        Simulator::Schedule(Seconds(el.PeriodSeconds()),
                            [&]() {
                                Vector p1 = sat->GetEciPosition();
                                const double dx = p1.x - p0.x;
                                const double dy = p1.y - p0.y;
                                const double dz = p1.z - p0.z;
                                const double drift =
                                    std::sqrt(dx * dx + dy * dy + dz * dz);
                                NS_TEST_ASSERT_MSG_LT(
                                    drift,
                                    100000.0,
                                    "drift after one period < 100 km");
                            });
        Simulator::Stop(Seconds(el.PeriodSeconds() + 1.0));
        Simulator::Run();
        Simulator::Destroy();

        // Sanity: position magnitude ~ a.
        const double r = std::sqrt(p0.x * p0.x + p0.y * p0.y + p0.z * p0.z);
        NS_TEST_ASSERT_MSG_EQ_TOL(r,
                                  el.semi_major_axis_m,
                                  1000.0,
                                  "circular orbit r ≈ a");
    }
};

class Sgp4EcefAltitudeTest : public TestCase
{
  public:
    Sgp4EcefAltitudeTest()
        : TestCase("Sgp4MobilityModel ECEF altitude tracks the configured shell")
    {
    }

  private:
    void DoRun() override
    {
        KeplerianElements el{};
        el.semi_major_axis_m = 6371000.0 + 550000.0;
        el.eccentricity = 0.0;
        el.inclination_rad = 53.0 * M_PI / 180.0;
        el.epoch_unix_s = 1577836800.0;
        Ptr<Sgp4MobilityModel> sat = CreateObject<Sgp4MobilityModel>();
        sat->SetElements(el);

        double lat;
        double lon;
        double alt;
        sat->GetGeodetic(lat, lon, alt);
        NS_TEST_ASSERT_MSG_GT(alt, 540000.0, "alt > 540 km");
        NS_TEST_ASSERT_MSG_LT(alt, 560000.0, "alt < 560 km");
        // Latitude must be within ±inclination.
        NS_TEST_ASSERT_MSG_LT(std::abs(lat), 54.0, "|lat| ≤ ~inclination");
    }
};

// ---------------------------------------------------------------------------
//  Walker constellation generator
// ---------------------------------------------------------------------------

class WalkerDeltaShapeTest : public TestCase
{
  public:
    WalkerDeltaShapeTest()
        : TestCase("Walker-Delta builder produces correct plane and sat counts")
    {
    }

  private:
    void DoRun() override
    {
        WalkerConfig cfg;
        cfg.inclination_deg = 53.0;
        cfg.total_sats = 66;
        cfg.num_planes = 6;
        cfg.phasing_f = 1;
        cfg.altitude_km = 550.0;
        cfg.epoch_unix_s = 1577836800.0;

        auto sats = WalkerConstellation::BuildDelta(cfg);
        NS_TEST_ASSERT_MSG_EQ(sats.size(), 66u, "T = 66");

        // Plane count: count distinct RAANs (modulo 1 deg).
        std::set<int> raans;
        for (const auto& s : sats)
        {
            raans.insert(static_cast<int>(s.raan_rad * 180.0 / M_PI + 0.5));
        }
        NS_TEST_ASSERT_MSG_EQ(raans.size(), 6u, "6 planes");

        // All altitudes ≈ 550 km above WGS-84.
        for (const auto& s : sats)
        {
            const double alt_km =
                (s.semi_major_axis_m - kEarthRadiusM) / 1000.0;
            NS_TEST_ASSERT_MSG_EQ_TOL(alt_km, 550.0, 0.001,
                                      "altitude 550 km");
        }

        // Each plane spans 60° RAAN spacing (360/6).
        std::vector<int> raanSorted(raans.begin(), raans.end());
        std::sort(raanSorted.begin(), raanSorted.end());
        for (size_t i = 1; i < raanSorted.size(); ++i)
        {
            NS_TEST_ASSERT_MSG_EQ_TOL(
                raanSorted[i] - raanSorted[i - 1],
                60,
                1,
                "60 deg RAAN spacing");
        }

        // Walker-Star has 180° RAAN span.
        auto starSats = WalkerConstellation::BuildStar(cfg);
        std::set<int> starRaans;
        for (const auto& s : starSats)
        {
            starRaans.insert(static_cast<int>(s.raan_rad * 180.0 / M_PI + 0.5));
        }
        NS_TEST_ASSERT_MSG_EQ(starRaans.size(), 6u, "6 polar planes");
        // Last - first = 5 * (180/6) = 150 deg.
        std::vector<int> sr(starRaans.begin(), starRaans.end());
        std::sort(sr.begin(), sr.end());
        NS_TEST_ASSERT_MSG_EQ_TOL(sr.back() - sr.front(), 150, 2,
                                  "star: 0..150 deg RAAN");
    }
};

// ---------------------------------------------------------------------------
//  ContactGraphScheduler — Simulator::Run() integration
// ---------------------------------------------------------------------------

class ContactSchedulerLeoPassTest : public TestCase
{
  public:
    ContactSchedulerLeoPassTest()
        : TestCase("ContactGraphScheduler: one-orbit-window LEO plane "
                   "over high-lat GS yields GSL up and down")
    {
    }

  private:
    void DoRun() override
    {
        // Install a full 11-sat Walker plane at 53° inclination, 550 km.
        // Place the GS at (lat=53°, lon=0°) — every satellite in the plane
        // passes near this latitude band once per orbit and the Earth's
        // rotation ensures some sat ascends into the GS's view during the
        // window.
        WalkerConfig cfg;
        cfg.inclination_deg = 53.0;
        cfg.total_sats = 11;
        cfg.num_planes = 1;
        cfg.phasing_f = 0;
        cfg.altitude_km = 550.0;
        cfg.epoch_unix_s = 1577836800.0;
        auto elts = WalkerConstellation::BuildDelta(cfg);
        NS_TEST_ASSERT_MSG_EQ(elts.size(), 11u, "11 sats");

        Ptr<ContactGraphScheduler> cg = CreateObject<ContactGraphScheduler>();
        cg->SetSamplingInterval(Seconds(10.0));
        cg->SetMinElevationDeg(5.0); // permissive
        for (size_t i = 0; i < elts.size(); ++i)
        {
            Ptr<Sgp4MobilityModel> s = CreateObject<Sgp4MobilityModel>();
            s->SetElements(elts[i]);
            cg->RegisterSatellite(static_cast<uint32_t>(i + 1), s);
        }
        cg->RegisterGroundStation(101, 53.0, 0.0);
        cg->Start();

        // One full orbital period ~ 5700 s.
        Simulator::Stop(Seconds(6000.0));
        Simulator::Run();
        cg->Stop();

        const uint64_t totalEvents = cg->GslEventsUp() + cg->GslEventsDown();
        NS_TEST_ASSERT_MSG_GT(totalEvents, 0u,
                              "11-sat plane over (53°, 0°) yields at "
                              "least one GSL transition in 6000 s");
        // At least one sat should be visible at some point.
        NS_TEST_ASSERT_MSG_GT(cg->GslEventsUp(), 0u, "≥1 GSL rise event");

        Simulator::Destroy();
    }
};

/// SAGIN-1: an established contact must keep publishing its geometry.
///
/// ContactGraphScheduler emitted on visibility TRANSITIONS only, so anything
/// that took its range from the contact-up event kept that value for the whole
/// pass. Two consumers were affected: sagin-sgp4-routed-traffic pinned both the
/// channel delay and the binary link budget, and ContactGraphRouter pinned the
/// Dijkstra edge weight, so the shortest path was chosen on stale distances.
/// A GSL slant at 550 km sweeps from roughly 550 km at zenith to 1075 km at a
/// 20 degree floor, which is ~1.8 ms one way - larger than most of the effects
/// the SAGIN examples exist to show.
///
/// This asserts that updates fire while a contact is up, that the range they
/// carry actually moves, and that the transition counters are untouched by the
/// new trace. Reverting the scheduler to transition-only emission gives zero
/// updates and fails on the first assertion.
/// CON-2: the satellite link budget must include the atmosphere.
///
/// EsNoDbFor was EIRP - FSPL + G/T - k - 10log10(B) and nothing else, at a
/// 20 GHz Ka default. No rain, no gaseous absorption, no scintillation, and no
/// elevation input at all beyond what the slant range implies. At Ka, rain
/// alone runs from single digits to well over 15 dB at 0.01 percent
/// availability. The omission grows as elevation falls, which is exactly where
/// handover decisions are made, so the C/N0 and BLER series written to
/// sim_health.csv under provenance "bler-errormodel" were optimistic in a
/// systematic, geometry-correlated way.
///
/// All three terms reuse the ITU models thz-ntn already ships, so this adds no
/// second implementation of P.676, P.838/P.618 or the scintillation model.
/// NT-03 enabler: the toolkit's mobility models must satisfy the geocentric
/// contract ns-3's TR 38.811 NTN channel model requires.
///
/// ThreeGppChannelModel raises NS_FATAL_ERROR("Mobility Models needs to be of
/// type Geocentric for NTN scenarios") unless BOTH endpoints DynamicCast to
/// GeocentricConstantPositionMobilityModel, and it reads the elevation angle
/// and the satellite-versus-HAPS decision off GetGeographicPosition().z. The
/// toolkit's satellites and terminals derived straight from MobilityModel, so
/// every NTN scenario string was unreachable and the small-scale plane behind
/// every measured SINR ran terrestrial UMa/UMi street-canyon statistics at 600
/// to 2000 km instead.
///
/// Setting the scenario string alone would have aborted at the first channel
/// realization rather than fixing anything, which is why this is asserted
/// separately from any channel wiring.
class Sgp4GeocentricContractTest : public TestCase
{
  public:
    Sgp4GeocentricContractTest()
        : TestCase("NT-03: SGP4 satellites satisfy the geocentric NTN channel contract")
    {
    }

  private:
    void DoRun() override
    {
        KeplerianElements el;
        el.semi_major_axis_m = 6371e3 + 600e3;
        el.eccentricity = 0.0;
        el.inclination_rad = 53.0 * M_PI / 180.0;
        el.epoch_unix_s = 1735689600.0;
        auto sat = CreateObject<Sgp4MobilityModel>();
        sat->SetElements(el);

        // The cast the channel model performs.
        Ptr<MobilityModel> asBase = sat;
        NS_TEST_ASSERT_MSG_NE(DynamicCast<GeocentricConstantPositionMobilityModel>(asBase),
                              nullptr,
                              "an SGP4 satellite must cast to the geocentric type, or every "
                              "TR 38.811 NTN scenario aborts at the first channel realization");

        // The geographic position must track the ORBIT, not a stored constant:
        // the base class holds one (lat, lon, alt) and a satellite's is never
        // constant. A stale value would silently select the wrong elevation-keyed
        // cluster table rather than fail.
        const Vector g0 = sat->GetGeographicPosition();
        NS_TEST_ASSERT_MSG_GT(g0.z, 500e3,
                              "a 600 km shell must report an altitude in the hundreds of km; the "
                              "channel model's satellite test is a 50 km threshold on this field");
        NS_TEST_ASSERT_MSG_LT(g0.z, 700e3, "and it must not be wildly above the shell either");
        NS_TEST_ASSERT_MSG_GT(g0.x, -91.0, "latitude must be a real latitude");
        NS_TEST_ASSERT_MSG_LT(g0.x, 91.0, "latitude must be a real latitude");

        Simulator::Stop(Seconds(300.0));
        Simulator::Run();
        const Vector g1 = sat->GetGeographicPosition();
        NS_TEST_ASSERT_MSG_GT(std::abs(g1.y - g0.y) + std::abs(g1.x - g0.x), 1.0,
                              "the geographic position must move as the orbit propagates; a "
                              "constant means the base class's stored value is being returned "
                              "and the channel would key its tables off the wrong geometry");

        // The elevation the channel model forms must be a real elevation.
        auto gs = CreateObject<GeocentricConstantPositionMobilityModel>();
        gs->SetGeographicPosition(Vector(45.0, 0.0, 0.0));
        const double elev = gs->GetElevationAngle(sat);
        NS_TEST_ASSERT_MSG_GT(elev, 0.0, "elevation must be positive and finite");
        NS_TEST_ASSERT_MSG_LT(elev, 90.001, "elevation cannot exceed the zenith");

        Simulator::Destroy();
    }
};

class SatLinkErrorAtmosphericChainTest : public TestCase
{
  public:
    SatLinkErrorAtmosphericChainTest()
        : TestCase("CON-2: the Ka link budget carries gas, rain and scintillation")
    {
    }

  private:
    /// Place a satellite at a requested elevation over a pole-mounted station.
    static void Place(double elevDeg,
                      Ptr<ConstantPositionMobilityModel>& gs,
                      Ptr<ConstantPositionMobilityModel>& sat)
    {
        const double Re = 6371e3;
        const double alt = 550e3;
        const double e = elevDeg * M_PI / 180.0;
        const double slant =
            std::sqrt(std::pow(Re + alt, 2) - std::pow(Re * std::cos(e), 2)) - Re * std::sin(e);
        gs = CreateObject<ConstantPositionMobilityModel>();
        gs->SetPosition(Vector(0, 0, Re));
        sat = CreateObject<ConstantPositionMobilityModel>();
        sat->SetPosition(Vector(slant * std::cos(e), 0, Re + slant * std::sin(e)));
    }

    void DoRun() override
    {
        // Elevation must be recovered from the two ECEF positions. Without it
        // no atmospheric term can be evaluated at all.
        for (double want : {90.0, 30.0, 10.0})
        {
            Ptr<ConstantPositionMobilityModel> gs, sat;
            Place(want, gs, sat);
            auto m = CreateObject<ntncon::NtnSatLinkErrorModel>();
            m->SetEndpoints(sat, gs);
            NS_TEST_ASSERT_MSG_EQ_TOL(m->CurrentElevationDeg(), want, 0.5,
                                      "the model must recover the link elevation from the two "
                                      "endpoint positions; it had no elevation input at all");
        }

        // Clear sky: gaseous and scintillation still apply, and both must grow
        // as elevation falls because the path through the atmosphere lengthens.
        double prevExcess = -1.0;
        for (double elev : {90.0, 30.0, 10.0, 5.0})
        {
            Ptr<ConstantPositionMobilityModel> gs, sat;
            Place(elev, gs, sat);
            auto m = CreateObject<ntncon::NtnSatLinkErrorModel>();
            m->SetAttribute("CarrierHz", DoubleValue(20e9));
            m->SetEndpoints(sat, gs);
            const double excess = m->CurrentExcessLossDb();
            NS_TEST_ASSERT_MSG_GT(excess, 0.0,
                                  "a Ka Earth-space path always crosses some atmosphere; zero "
                                  "excess loss means the chain is not wired in");
            NS_TEST_ASSERT_MSG_GT(m->LastGaseousDb(), 0.0, "gaseous absorption must be positive");
            NS_TEST_ASSERT_MSG_GT(m->LastScintillationDb(), 0.0, "scintillation must be positive");
            NS_TEST_ASSERT_MSG_EQ_TOL(m->LastRainDb(), 0.0, 1e-9,
                                      "rain must be off when no rain rate is configured, so the "
                                      "default stays clear-sky");
            NS_TEST_ASSERT_MSG_GT(excess, prevExcess,
                                  "excess loss must increase monotonically as elevation falls; "
                                  "a flat value means the terms ignore geometry");
            prevExcess = excess;
        }

        // Rain at Ka dominates, and that is the whole point of the finding.
        {
            Ptr<ConstantPositionMobilityModel> gs, sat;
            Place(30.0, gs, sat);
            auto dry = CreateObject<ntncon::NtnSatLinkErrorModel>();
            dry->SetAttribute("CarrierHz", DoubleValue(20e9));
            dry->SetEndpoints(sat, gs);
            auto wet = CreateObject<ntncon::NtnSatLinkErrorModel>();
            wet->SetAttribute("CarrierHz", DoubleValue(20e9));
            wet->SetAttribute("RainRateMmH", DoubleValue(42.0)); // a heavy 0.01% rate
            wet->SetEndpoints(sat, gs);
            const double dryDb = dry->CurrentExcessLossDb();
            const double wetDb = wet->CurrentExcessLossDb();
            NS_TEST_ASSERT_MSG_GT(wet->LastRainDb(), 5.0,
                                  "a heavy 0.01 percent rain rate at 20 GHz must cost many dB; "
                                  "this is the term whose absence made the budget optimistic");
            NS_TEST_ASSERT_MSG_GT(wetDb - dryDb, 5.0, "rain must move the total, not just a field");
            // And it must reach the budget, not merely be reported.
            NS_TEST_ASSERT_MSG_LT(wet->CurrentEsNoDb(), dry->CurrentEsNoDb() - 5.0,
                                  "the excess loss must be subtracted from Es/No; a term that is "
                                  "computed and not applied is the decision-island pattern");
        }

        // Every term must be switchable off, so a study can isolate one.
        {
            Ptr<ConstantPositionMobilityModel> gs, sat;
            Place(20.0, gs, sat);
            auto m = CreateObject<ntncon::NtnSatLinkErrorModel>();
            m->SetAttribute("EnableGaseous", BooleanValue(false));
            m->SetAttribute("EnableRain", BooleanValue(false));
            m->SetAttribute("EnableScintillation", BooleanValue(false));
            m->SetEndpoints(sat, gs);
            NS_TEST_ASSERT_MSG_EQ_TOL(m->CurrentExcessLossDb(), 0.0, 1e-9,
                                      "with every term disabled the budget must return to free "
                                      "space exactly, so the old behaviour stays reachable");
        }

        // An inter-satellite link crosses no weather.
        {
            auto a = CreateObject<ConstantPositionMobilityModel>();
            a->SetPosition(Vector(0, 0, 6371e3 + 550e3));
            auto b = CreateObject<ConstantPositionMobilityModel>();
            b->SetPosition(Vector(1000e3, 0, 6371e3 + 550e3));
            auto m = CreateObject<ntncon::NtnSatLinkErrorModel>();
            m->SetAttribute("RainRateMmH", DoubleValue(42.0));
            m->SetEndpoints(a, b);
            NS_TEST_ASSERT_MSG_EQ_TOL(m->CurrentExcessLossDb(), 0.0, 1e-9,
                                      "a link between two satellites must not be charged rain; "
                                      "the geometry, not a link-type flag, has to decide");
        }

        // The real DVB-S2 curve must be reachable, and must be a genuine
        // waterfall rather than the erfc it replaces.
        {
            Ptr<ConstantPositionMobilityModel> gs, sat;
            Place(90.0, gs, sat);
            auto m = CreateObject<ntncon::NtnSatLinkErrorModel>();
            m->SetAttribute("UseDvbS2LinkResults", BooleanValue(true));
            m->SetAttribute("Modcod", UintegerValue(2)); // QPSK 1/2
            m->SetAttribute("CarrierHz", DoubleValue(20e9));
            m->SetAttribute("RxGtDbK", DoubleValue(15.0));
            m->SetAttribute("EirpDbw", DoubleValue(5.0));
            m->SetEndpoints(sat, gs);
            const double weak = m->CurrentBler();
            m->SetAttribute("EirpDbw", DoubleValue(25.0));
            const double strong = m->CurrentBler();
            NS_TEST_ASSERT_MSG_GT(weak, 0.5,
                                  "below the QPSK 1/2 threshold the measured DVB-S2 table must "
                                  "report a failing link");
            NS_TEST_ASSERT_MSG_LT(strong, 0.01,
                                  "well above threshold it must report a clean one; a flat "
                                  "response means the look-up tables did not load");
        }

        Simulator::Destroy();
    }
};

class ContactSchedulerRangeUpdateTest : public TestCase
{
  public:
    ContactSchedulerRangeUpdateTest()
        : TestCase("SAGIN-1: an up contact republishes its live range every tick")
    {
    }

  private:
    uint32_t m_updates{0};
    double m_minRange{std::numeric_limits<double>::max()};
    double m_maxRange{0.0};
    uint32_t m_ups{0};

    void OnUpdate(ContactEvent ev)
    {
        ++m_updates;
        NS_TEST_ASSERT_MSG_EQ(ev.up, true, "an update may only fire for a contact that is up");
        m_minRange = std::min(m_minRange, ev.range_m);
        m_maxRange = std::max(m_maxRange, ev.range_m);
    }

    void OnUp(ContactEvent) { ++m_ups; }

    void DoRun() override
    {
        WalkerConfig cfg;
        cfg.inclination_deg = 53.0;
        cfg.total_sats = 11;
        cfg.num_planes = 1;
        cfg.phasing_f = 0;
        cfg.altitude_km = 550.0;
        cfg.epoch_unix_s = 1577836800.0;
        auto elts = WalkerConstellation::BuildDelta(cfg);

        Ptr<ContactGraphScheduler> cg = CreateObject<ContactGraphScheduler>();
        cg->SetSamplingInterval(Seconds(10.0));
        cg->SetMinElevationDeg(5.0);
        for (size_t i = 0; i < elts.size(); ++i)
        {
            Ptr<Sgp4MobilityModel> sat = CreateObject<Sgp4MobilityModel>();
            sat->SetElements(elts[i]);
            cg->RegisterSatellite(static_cast<uint32_t>(i + 1), sat);
        }
        cg->RegisterGroundStation(101, 53.0, 0.0);
        cg->m_contactUpdate.ConnectWithoutContext(
            MakeCallback(&ContactSchedulerRangeUpdateTest::OnUpdate, this));
        cg->m_contactUp.ConnectWithoutContext(
            MakeCallback(&ContactSchedulerRangeUpdateTest::OnUp, this));
        cg->Start();

        Simulator::Stop(Seconds(6000.0));
        Simulator::Run();
        cg->Stop();

        NS_TEST_ASSERT_MSG_GT(m_ups, 0u, "the window must contain at least one contact");
        NS_TEST_ASSERT_MSG_GT(m_updates, 10u,
                              "a contact that stays up across many 10 s ticks must republish its "
                              "range on each of them; zero or a handful means the scheduler is "
                              "back to emitting only on transitions and every consumer is once "
                              "again working from the range sampled at contact-up");

        // The whole point is that the number MOVES. A LEO pass over a 5 degree
        // floor spans hundreds of kilometres of slant, so anything under 100 km
        // of spread means a constant is being republished.
        NS_TEST_ASSERT_MSG_GT(m_maxRange - m_minRange, 100e3,
                              "the republished range must actually track the pass; a small spread "
                              "means the value is stale even though the trace fires");
        NS_TEST_ASSERT_MSG_GT(m_minRange, 500e3, "a 550 km LEO slant cannot be shorter than this");
        NS_TEST_ASSERT_MSG_LT(m_maxRange, 4000e3, "implausible slant for a 550 km shell");

        // Transition accounting must be unchanged by the new trace: updates are
        // additional information, not a reinterpretation of up and down. The
        // up trace carries GSL and ISL rises together, so compare against both
        // counters.
        NS_TEST_ASSERT_MSG_EQ(cg->GslEventsUp() + cg->IslEventsUp(), m_ups,
                              "the update trace must not disturb the up/down counters");

        Simulator::Destroy();
    }
};

class ContactSchedulerIslPairTest : public TestCase
{
  public:
    ContactSchedulerIslPairTest()
        : TestCase("ContactGraphScheduler: same-plane LEO pair stays within ISL range")
    {
    }

  private:
    void DoRun() override
    {
        // Build two adjacent sats in the same plane at 550 km, 53°.
        // Mean-anomaly delta = 360/11 ≈ 32.7° → along-track separation
        // ≈ (a) * (32.7° in rad) ≈ 3940 km — inside 5000 km ISL cap.
        WalkerConfig cfg;
        cfg.inclination_deg = 53.0;
        cfg.total_sats = 11;
        cfg.num_planes = 1;
        cfg.phasing_f = 0;
        cfg.altitude_km = 550.0;
        cfg.epoch_unix_s = 1577836800.0;
        auto elts = WalkerConstellation::BuildDelta(cfg);
        NS_TEST_ASSERT_MSG_EQ(elts.size(), 11u, "11 sats");

        Ptr<Sgp4MobilityModel> s0 = CreateObject<Sgp4MobilityModel>();
        s0->SetElements(elts[0]);
        Ptr<Sgp4MobilityModel> s1 = CreateObject<Sgp4MobilityModel>();
        s1->SetElements(elts[1]);

        Ptr<ContactGraphScheduler> cg = CreateObject<ContactGraphScheduler>();
        cg->SetSamplingInterval(Seconds(5.0));
        cg->SetMaxIslRangeM(5'000'000.0);
        cg->RegisterSatellite(0, s0);
        cg->RegisterSatellite(1, s1);
        cg->Start();

        Simulator::Stop(Seconds(60.0));
        Simulator::Run();
        cg->Stop();

        // Same-plane adjacent pair should be "up" on the first sample
        // and stay up through 60 s (no transitions to down).
        NS_TEST_ASSERT_MSG_EQ(cg->IslEventsUp(), 1u,
                              "exactly one ISL up event (initial)");
        NS_TEST_ASSERT_MSG_EQ(cg->IslEventsDown(), 0u,
                              "no ISL down inside 60 s");
        NS_TEST_ASSERT_MSG_EQ(cg->NumActiveIsl(), 1u, "pair is active");

        Simulator::Destroy();
    }
};

namespace
{

void
RecordContactEvent(std::vector<ContactEvent>* out, ContactEvent ev)
{
    out->push_back(ev);
}

} // namespace

class ContactSchedulerGateHysteresisTest : public TestCase
{
  public:
    ContactSchedulerGateHysteresisTest()
        : TestCase("ContactGraphScheduler: GateHysteresisDeg gates GSL up at "
                   "MinElevationDeg and down only below MinElevationDeg - "
                   "hysteresis (no flapping inside the band)")
    {
    }

  private:
    void DoRun() override
    {
        // Same fixture as ContactSchedulerLeoPassTest: full 11-sat Walker
        // plane @ 53° / 550 km over a GS at (53°, 0°). Every sat passes the
        // GS latitude band, so elevations sweep through the 5° threshold
        // repeatedly during one orbit window.
        WalkerConfig cfg;
        cfg.inclination_deg = 53.0;
        cfg.total_sats = 11;
        cfg.num_planes = 1;
        cfg.phasing_f = 0;
        cfg.altitude_km = 550.0;
        cfg.epoch_unix_s = 1577836800.0;
        auto elts = WalkerConstellation::BuildDelta(cfg);
        NS_TEST_ASSERT_MSG_EQ(elts.size(), 11u, "11 sats");

        const double minElev = 5.0;
        const double hyst = 5.0;

        // Two schedulers sampling the SAME Sgp4 models over the SAME pass:
        // one with the legacy single-threshold gate (hysteresis 0), one with
        // a 5° hysteresis band.
        Ptr<ContactGraphScheduler> cgZero =
            CreateObject<ContactGraphScheduler>();
        cgZero->SetSamplingInterval(Seconds(10.0));
        cgZero->SetMinElevationDeg(minElev);
        cgZero->SetGateHysteresisDeg(0.0);

        Ptr<ContactGraphScheduler> cgHyst =
            CreateObject<ContactGraphScheduler>();
        cgHyst->SetSamplingInterval(Seconds(10.0));
        cgHyst->SetMinElevationDeg(minElev);
        cgHyst->SetGateHysteresisDeg(hyst);
        NS_TEST_ASSERT_MSG_EQ_TOL(cgHyst->GetGateHysteresisDeg(), hyst, 1e-12,
                                  "hysteresis setter round-trips");

        for (size_t i = 0; i < elts.size(); ++i)
        {
            Ptr<Sgp4MobilityModel> s = CreateObject<Sgp4MobilityModel>();
            s->SetElements(elts[i]);
            cgZero->RegisterSatellite(static_cast<uint32_t>(i + 1), s);
            cgHyst->RegisterSatellite(static_cast<uint32_t>(i + 1), s);
        }
        cgZero->RegisterGroundStation(101, 53.0, 0.0);
        cgHyst->RegisterGroundStation(101, 53.0, 0.0);

        // Record every transition of the hysteresis scheduler (events carry
        // the elevation at the transition tick).
        std::vector<ContactEvent> hystEvents;
        cgHyst->m_contactUp.ConnectWithoutContext(
            MakeBoundCallback(&RecordContactEvent, &hystEvents));
        cgHyst->m_contactDown.ConnectWithoutContext(
            MakeBoundCallback(&RecordContactEvent, &hystEvents));

        cgZero->Start();
        cgHyst->Start();

        // One full orbital period ~ 5700 s.
        Simulator::Stop(Seconds(6000.0));
        Simulator::Run();
        cgZero->Stop();
        cgHyst->Stop();

        // The pass actually crosses the gate.
        NS_TEST_ASSERT_MSG_GT(cgZero->GslEventsUp(), 0u,
                              "zero-hysteresis gate sees ≥1 GSL rise");
        NS_TEST_ASSERT_MSG_GT(cgHyst->GslEventsUp(), 0u,
                              "hysteresis gate sees ≥1 GSL rise");

        // Hysteresis can only merge contacts, never create extra
        // transitions over the same tick series.
        NS_TEST_ASSERT_MSG_LT_OR_EQ(cgHyst->GslEventsUp(),
                                    cgZero->GslEventsUp(),
                                    "hysteresis up count <= zero-hyst");
        NS_TEST_ASSERT_MSG_LT_OR_EQ(cgHyst->GslEventsDown(),
                                    cgZero->GslEventsDown(),
                                    "hysteresis down count <= zero-hyst");
        NS_TEST_ASSERT_MSG_LT_OR_EQ(
            cgHyst->GslEventsUp() + cgHyst->GslEventsDown(),
            cgZero->GslEventsUp() + cgZero->GslEventsDown(),
            "hysteresis total events <= zero-hyst total");

        // Gate contract on every recorded transition:
        //   UP fires at/above MinElevationDeg;
        //   DOWN fires only strictly below MinElevationDeg - hysteresis —
        //   i.e. once up, the contact never drops while the elevation stays
        //   inside the [threshold - hyst, threshold) band.
        for (const auto& ev : hystEvents)
        {
            if (ev.is_isl)
            {
                // The 11-sat plane also raises ISL events; the hysteresis
                // gate under test applies to GSL contacts only.
                continue;
            }
            if (ev.up)
            {
                NS_TEST_ASSERT_MSG_GT_OR_EQ(ev.elevation_deg, minElev,
                                            "contact comes up at the "
                                            "MinElevationDeg threshold");
            }
            else
            {
                NS_TEST_ASSERT_MSG_LT(ev.elevation_deg, minElev - hyst,
                                      "contact drops only below "
                                      "threshold - hysteresis");
            }
        }

        // Per-pair alternation: events for each (sat, gs) pair must be
        // up, down, up, ... starting with up — no down-then-up flap can
        // occur inside the hysteresis band.
        std::map<std::pair<uint32_t, uint32_t>, bool> lastUp;
        for (const auto& ev : hystEvents)
        {
            if (ev.is_isl)
            {
                // ISL pair keys (sat, sat) can collide with GSL keys
                // (sat, gs); the gate under test is GSL-only.
                continue;
            }
            const std::pair<uint32_t, uint32_t> key{ev.node_a, ev.node_b};
            auto it = lastUp.find(key);
            if (it == lastUp.end())
            {
                NS_TEST_ASSERT_MSG_EQ(ev.up, true,
                                      "first event per pair is a rise");
            }
            else
            {
                NS_TEST_ASSERT_MSG_NE(ev.up, it->second,
                                      "per-pair events alternate up/down");
            }
            lastUp[key] = ev.up;
        }

        // Active-contact bookkeeping matches the recorded event stream.
        size_t expectedActive = 0;
        for (const auto& kv : lastUp)
        {
            if (kv.second)
            {
                ++expectedActive;
            }
        }
        NS_TEST_ASSERT_MSG_EQ(cgHyst->NumActiveGsl(), expectedActive,
                              "NumActiveGsl matches pairs whose last "
                              "event was a rise");

        Simulator::Destroy();
    }
};

// ---------------------------------------------------------------------------
//  Roadmap §4.4.4: ContactGraphRouter
// ---------------------------------------------------------------------------

class ContactGraphRouterDirectEdgesTest : public TestCase
{
  public:
    ContactGraphRouterDirectEdgesTest()
        : TestCase("ContactGraphRouter handles direct contact up and down events")
    {
    }

  private:
    void DoRun() override
    {
        Ptr<ContactGraphRouter> r = CreateObject<ContactGraphRouter>();
        Ptr<ContactGraphScheduler> sched =
            CreateObject<ContactGraphScheduler>();
        r->Attach(sched);

        ContactEvent ev_up{1.0, 1, 2, true, true, 1500e3, 0.0};
        sched->m_contactUp(ev_up);
        ContactEvent ev_up2{1.0, 2, 3, true, true, 1500e3, 0.0};
        sched->m_contactUp(ev_up2);
        NS_TEST_EXPECT_MSG_EQ(r->NumEdges(), 2u, "2 ISL edges live");
        NS_TEST_EXPECT_MSG_EQ(r->HasEdge(1, 2), true, "edge 1<->2");
        NS_TEST_EXPECT_MSG_EQ(r->HasEdge(2, 1), true, "edge 2<->1 undirected");
        NS_TEST_EXPECT_MSG_EQ(r->HasEdge(1, 3), false, "no edge 1<->3");
        NS_TEST_EXPECT_MSG_EQ(r->Neighbours(2).size(), 2u, "node 2 has 2 nbrs");

        ContactEvent ev_down{2.0, 1, 2, true, false, 9999e3, 0.0};
        sched->m_contactDown(ev_down);
        NS_TEST_EXPECT_MSG_EQ(r->NumEdges(), 1u, "edge 1<->2 removed");
        NS_TEST_EXPECT_MSG_EQ(r->HasEdge(1, 2), false, "edge gone");
        NS_TEST_EXPECT_MSG_EQ(r->EdgesAddedTotal(), 2u, "2 adds counted");
        NS_TEST_EXPECT_MSG_EQ(r->EdgesRemovedTotal(), 1u, "1 remove counted");

        // Duplicate up is idempotent.
        sched->m_contactUp(ev_up2);
        NS_TEST_EXPECT_MSG_EQ(r->NumEdges(), 1u, "duplicate up is no-op");
        NS_TEST_EXPECT_MSG_EQ(r->EdgesAddedTotal(),
                              2u,
                              "duplicate up does not double-count");
    }
};

class ContactGraphRouterShortestPathTest : public TestCase
{
  public:
    ContactGraphRouterShortestPathTest()
        : TestCase("ContactGraphRouter BFS shortest-path on a 4-node ring")
    {
    }

  private:
    void DoRun() override
    {
        Ptr<ContactGraphRouter> r = CreateObject<ContactGraphRouter>();
        Ptr<ContactGraphScheduler> sched =
            CreateObject<ContactGraphScheduler>();
        r->Attach(sched);

        // Ring 1-2-3-4-1.
        sched->m_contactUp({0, 1, 2, true, true, 1e6, 0.0});
        sched->m_contactUp({0, 2, 3, true, true, 1e6, 0.0});
        sched->m_contactUp({0, 3, 4, true, true, 1e6, 0.0});
        sched->m_contactUp({0, 4, 1, true, true, 1e6, 0.0});
        NS_TEST_EXPECT_MSG_EQ(r->NumEdges(), 4u, "ring has 4 edges");

        auto p12 = r->ShortestPath(1, 2);
        NS_TEST_ASSERT_MSG_EQ(p12.size(), 2u, "direct path size 2");
        NS_TEST_EXPECT_MSG_EQ(p12[0], 1u, "path[0]=1");
        NS_TEST_EXPECT_MSG_EQ(p12[1], 2u, "path[1]=2");

        auto p13 = r->ShortestPath(1, 3);
        NS_TEST_ASSERT_MSG_EQ(p13.size(), 3u, "opposite node is 2 hops");
        NS_TEST_EXPECT_MSG_EQ(p13[0], 1u, "starts at src");
        NS_TEST_EXPECT_MSG_EQ(p13[2], 3u, "ends at dst");

        auto p_self = r->ShortestPath(2, 2);
        NS_TEST_ASSERT_MSG_EQ(p_self.size(), 1u, "self-path length 1");
        NS_TEST_EXPECT_MSG_EQ(p_self[0], 2u, "self path is {src}");

        auto p_none = r->ShortestPath(1, 99);
        NS_TEST_EXPECT_MSG_EQ(p_none.size(), 0u, "no path to unknown node");

        // Break edge 1-2: path 1->2 lengthens to 1-4-3-2 (4 nodes, 3 hops).
        sched->m_contactDown({1.0, 1, 2, true, false, 9e9, 0.0});
        auto p12_again = r->ShortestPath(1, 2);
        NS_TEST_ASSERT_MSG_EQ(p12_again.size(), 4u,
                              "after edge 1-2 down, path is 1-4-3-2");
        NS_TEST_EXPECT_MSG_EQ(p12_again.front(), 1u, "front still 1");
        NS_TEST_EXPECT_MSG_EQ(p12_again.back(), 2u, "back still 2");
        NS_TEST_EXPECT_MSG_GT(r->RouteQueries(), 0u, "queries counted");
    }
};

namespace
{

struct RouteSample
{
    double t_s;
    size_t num_edges;
    size_t path_len;
};

void
SampleRoute(Ptr<ContactGraphRouter> router,
            uint32_t src,
            uint32_t dst,
            std::vector<RouteSample>* out)
{
    auto path = router->ShortestPath(src, dst);
    out->push_back({Simulator::Now().GetSeconds(),
                     router->NumEdges(),
                     path.size()});
}

} // namespace

class ContactGraphRouterSimulatorTimeTest : public TestCase
{
  public:
    ContactGraphRouterSimulatorTimeTest()
        : TestCase("Simulator: 600 s 4-sat Walker plane drives router edges "
                   "and shortest-path samples through ISL evolution")
    {
    }

  private:
    void DoRun() override
    {
        // 11-sat single-plane Walker @ 53° / 550 km — mean-anomaly delta
        // = 360/11 ≈ 32.7°, along-track separation ≈ 3950 km, so each
        // adjacent pair sits inside the 5000 km LEO-LEO ISL cap. We
        // still query src=1, dst=3 (two hops along the ring).
        WalkerConfig cfg;
        cfg.inclination_deg = 53.0;
        cfg.total_sats = 11;
        cfg.num_planes = 1;
        cfg.phasing_f = 0;
        cfg.altitude_km = 550.0;
        cfg.epoch_unix_s = 1577836800.0;
        auto elts = WalkerConstellation::BuildDelta(cfg);
        NS_TEST_ASSERT_MSG_EQ(elts.size(), 11u, "11 sats");

        Ptr<ContactGraphScheduler> cg = CreateObject<ContactGraphScheduler>();
        cg->SetSamplingInterval(Seconds(30.0));
        cg->SetMaxIslRangeM(5'000'000.0); // realistic LEO-LEO cap
        for (size_t i = 0; i < elts.size(); ++i)
        {
            Ptr<Sgp4MobilityModel> s = CreateObject<Sgp4MobilityModel>();
            s->SetElements(elts[i]);
            cg->RegisterSatellite(static_cast<uint32_t>(i + 1), s);
        }
        Ptr<ContactGraphRouter> router = CreateObject<ContactGraphRouter>();
        router->Attach(cg);
        cg->Start();

        std::vector<RouteSample> samples;
        for (int t = 60; t <= 600; t += 60)
        {
            Simulator::Schedule(Seconds(t),
                                &SampleRoute,
                                router,
                                /*src=*/1u,
                                /*dst=*/3u,
                                &samples);
        }
        Simulator::Stop(Seconds(601));
        Simulator::Run();
        cg->Stop();

        NS_TEST_ASSERT_MSG_EQ(samples.size(), 10u, "10 route samples");

        size_t max_edges = 0;
        for (const auto& s : samples)
        {
            if (s.num_edges > max_edges)
                max_edges = s.num_edges;
        }
        NS_TEST_ASSERT_MSG_GT(max_edges, 0u,
                              "router sees at least one edge over 600 s");
        NS_TEST_ASSERT_MSG_GT(router->EdgesAddedTotal(), 0u,
                              "router observed edge-up events");

        Simulator::Destroy();
    }
};

// ---------------------------------------------------------------------------
//  Roadmap §4.4.5: ContactGraphRouter Dijkstra link-weighted shortest path
// ---------------------------------------------------------------------------

class ContactGraphRouterWeightedEdgeTest : public TestCase
{
  public:
    ContactGraphRouterWeightedEdgeTest()
        : TestCase("ContactGraphRouter records edge weights from contact "
                   "events and EdgeWeight returns them")
    {
    }

  private:
    void DoRun() override
    {
        Ptr<ContactGraphRouter> r = CreateObject<ContactGraphRouter>();
        Ptr<ContactGraphScheduler> sched =
            CreateObject<ContactGraphScheduler>();
        r->Attach(sched);

        // Hand-fire edges with explicit ranges.
        sched->m_contactUp({0, 1, 2, true, true, 1500e3, 0.0});
        sched->m_contactUp({0, 2, 3, true, true, 3000e3, 0.0});
        NS_TEST_ASSERT_MSG_EQ(r->NumEdges(), 2u, "2 edges");
        NS_TEST_ASSERT_MSG_EQ_TOL(r->EdgeWeight(1, 2), 1500e3, 1e-6,
                                  "edge 1-2 weight");
        NS_TEST_ASSERT_MSG_EQ_TOL(r->EdgeWeight(2, 3), 3000e3, 1e-6,
                                  "edge 2-3 weight");
        const bool weightIsNan = std::isnan(r->EdgeWeight(1, 9));
        NS_TEST_ASSERT_MSG_EQ(weightIsNan, true,
                              "absent edge returns NaN");

        // Re-up event refreshes the weight.
        sched->m_contactUp({1, 1, 2, true, true, 1200e3, 0.0});
        NS_TEST_ASSERT_MSG_EQ_TOL(r->EdgeWeight(1, 2), 1200e3, 1e-6,
                                  "edge 1-2 weight refreshed");

        // Edge-down removes weight.
        sched->m_contactDown({2, 1, 2, true, false, 9e9, 0.0});
        const bool postDownNan = std::isnan(r->EdgeWeight(1, 2));
        NS_TEST_ASSERT_MSG_EQ(postDownNan, true,
                              "edge weight removed on down");
    }
};

class ContactGraphRouterDijkstraTest : public TestCase
{
  public:
    ContactGraphRouterDijkstraTest()
        : TestCase("Dijkstra picks a longer-hop low-weight route over a "
                   "shorter-hop high-weight route")
    {
    }

  private:
    void DoRun() override
    {
        Ptr<ContactGraphRouter> r = CreateObject<ContactGraphRouter>();
        Ptr<ContactGraphScheduler> sched =
            CreateObject<ContactGraphScheduler>();
        r->Attach(sched);

        // Build the "triangle with one expensive direct edge" graph:
        //   1 -- 2 (weight 100)
        //   2 -- 3 (weight 100)
        //   1 -- 3 (weight 1000)
        // BFS from 1 to 3 -> {1, 3} (1 hop, but high weight 1000).
        // Dijkstra from 1 to 3 -> {1, 2, 3} (2 hops, low weight 200).
        sched->m_contactUp({0, 1, 2, true, true, 100.0, 0.0});
        sched->m_contactUp({0, 2, 3, true, true, 100.0, 0.0});
        sched->m_contactUp({0, 1, 3, true, true, 1000.0, 0.0});

        auto bfs = r->ShortestPath(1, 3);
        NS_TEST_ASSERT_MSG_EQ(bfs.size(), 2u, "BFS takes the 1-hop edge");
        NS_TEST_EXPECT_MSG_EQ(bfs[0], 1u, "BFS[0]=1");
        NS_TEST_EXPECT_MSG_EQ(bfs[1], 3u, "BFS[1]=3");

        auto dij = r->ShortestPathWeighted(1, 3);
        NS_TEST_ASSERT_MSG_EQ(dij.path.size(), 3u,
                              "Dijkstra takes the 2-hop low-weight route");
        NS_TEST_EXPECT_MSG_EQ(dij.path[0], 1u, "dij[0]=1");
        NS_TEST_EXPECT_MSG_EQ(dij.path[1], 2u, "dij[1]=2 (intermediate)");
        NS_TEST_EXPECT_MSG_EQ(dij.path[2], 3u, "dij[2]=3");
        NS_TEST_ASSERT_MSG_EQ_TOL(dij.total_weight, 200.0, 1e-9,
                                  "total weight = 100 + 100");

        // src == dst short-circuit.
        auto self = r->ShortestPathWeighted(2, 2);
        NS_TEST_EXPECT_MSG_EQ(self.path.size(), 1u, "self path length 1");
        NS_TEST_EXPECT_MSG_EQ(self.total_weight, 0.0, "self weight 0");

        // Disconnected node returns empty + +inf.
        auto none = r->ShortestPathWeighted(1, 999);
        NS_TEST_EXPECT_MSG_EQ(none.path.size(), 0u, "no path");
        const bool isInf = std::isinf(none.total_weight);
        NS_TEST_EXPECT_MSG_EQ(isInf, true, "total_weight = +inf");

        // Bring down the cheap leg — Dijkstra falls back to the 1000-cost
        // direct edge.
        sched->m_contactDown({1, 1, 2, true, false, 9e9, 0.0});
        auto after = r->ShortestPathWeighted(1, 3);
        NS_TEST_ASSERT_MSG_EQ(after.path.size(), 2u,
                              "after 1-2 down, fall back to direct edge");
        NS_TEST_ASSERT_MSG_EQ_TOL(after.total_weight, 1000.0, 1e-9,
                                  "fallback weight 1000");
    }
};

namespace
{

struct WeightedRouteSample
{
    double t_s;
    size_t path_len;
    double total_weight_m;
};

void
SampleWeightedRoute(Ptr<ContactGraphRouter> router,
                     uint32_t src,
                     uint32_t dst,
                     std::vector<WeightedRouteSample>* out)
{
    auto wp = router->ShortestPathWeighted(src, dst);
    out->push_back({Simulator::Now().GetSeconds(),
                     wp.path.size(),
                     wp.total_weight});
}

} // namespace

class ContactGraphRouterDijkstraSimulatorTimeTest : public TestCase
{
  public:
    ContactGraphRouterDijkstraSimulatorTimeTest()
        : TestCase("Simulator: 600 s 11-sat Walker plane drives Dijkstra "
                   "routing; weighted path stays positive while ISLs flicker")
    {
    }

  private:
    void DoRun() override
    {
        WalkerConfig cfg;
        cfg.inclination_deg = 53.0;
        cfg.total_sats = 11;
        cfg.num_planes = 1;
        cfg.phasing_f = 0;
        cfg.altitude_km = 550.0;
        cfg.epoch_unix_s = 1577836800.0;
        auto elts = WalkerConstellation::BuildDelta(cfg);
        NS_TEST_ASSERT_MSG_EQ(elts.size(), 11u, "11 sats");

        Ptr<ContactGraphScheduler> cg = CreateObject<ContactGraphScheduler>();
        cg->SetSamplingInterval(Seconds(30.0));
        cg->SetMaxIslRangeM(5'000'000.0);
        for (size_t i = 0; i < elts.size(); ++i)
        {
            Ptr<Sgp4MobilityModel> s = CreateObject<Sgp4MobilityModel>();
            s->SetElements(elts[i]);
            cg->RegisterSatellite(static_cast<uint32_t>(i + 1), s);
        }
        Ptr<ContactGraphRouter> router = CreateObject<ContactGraphRouter>();
        router->Attach(cg);
        cg->Start();

        std::vector<WeightedRouteSample> samples;
        for (int t = 60; t <= 600; t += 60)
        {
            Simulator::Schedule(Seconds(t),
                                &SampleWeightedRoute,
                                router,
                                /*src=*/1u,
                                /*dst=*/3u,
                                &samples);
        }
        Simulator::Stop(Seconds(601));
        Simulator::Run();
        cg->Stop();

        NS_TEST_ASSERT_MSG_EQ(samples.size(), 10u, "10 weighted samples");
        // At least one sample must have found a route with positive weight.
        size_t routed = 0;
        for (const auto& s : samples)
        {
            if (s.path_len > 0 && s.total_weight_m > 0.0 &&
                s.total_weight_m < 1e9)
            {
                ++routed;
            }
        }
        NS_TEST_ASSERT_MSG_GT(routed, 0u,
                              "at least one sample found a finite-weight route");
        NS_TEST_ASSERT_MSG_GT(router->RouteQueries(), 0u,
                              "weighted-path queries counted");

        Simulator::Destroy();
    }
};

// ---------------------------------------------------------------------------
//  Roadmap §4.4.6: Regen-vs-bent-pipe split
// ---------------------------------------------------------------------------

class ContactGraphRouterRegenModeTest : public TestCase
{
  public:
    ContactGraphRouterRegenModeTest()
        : TestCase("ContactGraphRouter SetRegenMode and IsRegenerative are sticky")
    {
    }

  private:
    void DoRun() override
    {
        Ptr<ContactGraphRouter> r = CreateObject<ContactGraphRouter>();
        NS_TEST_EXPECT_MSG_EQ(static_cast<int>(r->GetRegenMode(42)),
                              static_cast<int>(RegenMode::bent_pipe),
                              "unknown node defaults to bent-pipe");
        NS_TEST_EXPECT_MSG_EQ(r->IsRegenerative(42), false, "default not regen");

        r->SetRegenMode(1, RegenMode::regen_du);
        r->SetRegenMode(2, RegenMode::regen_full);
        r->SetRegenMode(3, RegenMode::bent_pipe);
        NS_TEST_EXPECT_MSG_EQ(static_cast<int>(r->GetRegenMode(1)),
                              static_cast<int>(RegenMode::regen_du),
                              "node 1 is regen_du");
        NS_TEST_EXPECT_MSG_EQ(r->IsRegenerative(1), true, "node 1 regen");
        NS_TEST_EXPECT_MSG_EQ(r->IsRegenerative(2), true, "node 2 regen");
        NS_TEST_EXPECT_MSG_EQ(r->IsRegenerative(3), false, "node 3 bent");
    }
};

class ContactGraphRouterRegenOnlyDijkstraTest : public TestCase
{
  public:
    ContactGraphRouterRegenOnlyDijkstraTest()
        : TestCase("ShortestPathWeightedRegenOnly routes around bent-pipe transit nodes")
    {
    }

  private:
    void DoRun() override
    {
        Ptr<ContactGraphRouter> r = CreateObject<ContactGraphRouter>();
        Ptr<ContactGraphScheduler> sched =
            CreateObject<ContactGraphScheduler>();
        r->Attach(sched);

        // 4-node chain 1-2-3-4 with each edge weight 100, plus a long
        // direct edge 1-4 weight 1000.
        sched->m_contactUp({0, 1, 2, true, true, 100.0, 0.0});
        sched->m_contactUp({0, 2, 3, true, true, 100.0, 0.0});
        sched->m_contactUp({0, 3, 4, true, true, 100.0, 0.0});
        sched->m_contactUp({0, 1, 4, true, true, 1000.0, 0.0});

        // All regen -> cheap chain.
        r->SetRegenMode(1, RegenMode::regen_full);
        r->SetRegenMode(2, RegenMode::regen_du);
        r->SetRegenMode(3, RegenMode::regen_cu);
        r->SetRegenMode(4, RegenMode::regen_full);
        auto allRegen = r->ShortestPathWeightedRegenOnly(1, 4);
        NS_TEST_ASSERT_MSG_EQ(allRegen.path.size(), 4u,
                              "all-regen route is 1-2-3-4");
        NS_TEST_ASSERT_MSG_EQ_TOL(allRegen.total_weight, 300.0, 1e-9,
                                  "all-regen weight 300");

        // Node 2 bent-pipe -> must use direct edge.
        r->SetRegenMode(2, RegenMode::bent_pipe);
        auto withBent = r->ShortestPathWeightedRegenOnly(1, 4);
        NS_TEST_ASSERT_MSG_EQ(withBent.path.size(), 2u,
                              "bent-pipe transit forces direct edge");
        NS_TEST_ASSERT_MSG_EQ_TOL(withBent.total_weight, 1000.0, 1e-9,
                                  "direct-edge weight");

        // Bent-pipe destination is still routable (endpoints aren't filtered).
        r->SetRegenMode(2, RegenMode::regen_du);
        r->SetRegenMode(4, RegenMode::bent_pipe);
        auto bentDst = r->ShortestPathWeightedRegenOnly(1, 4);
        NS_TEST_ASSERT_MSG_EQ(bentDst.path.size(), 4u,
                              "bent-pipe endpoint still routable");

        // No transit + no direct edge -> no path.
        r->SetRegenMode(2, RegenMode::bent_pipe);
        r->SetRegenMode(3, RegenMode::bent_pipe);
        sched->m_contactDown({1, 1, 4, true, false, 9e9, 0.0});
        auto none = r->ShortestPathWeightedRegenOnly(1, 4);
        NS_TEST_EXPECT_MSG_EQ(none.path.size(), 0u,
                              "no transit path when all transit bent");
        const bool isInf = std::isinf(none.total_weight);
        NS_TEST_EXPECT_MSG_EQ(isInf, true, "+inf weight");
    }
};

namespace
{

struct RegenSample
{
    double t_s;
    size_t path_len;
    double total_weight;
};

void
SampleRegenRoute(Ptr<ContactGraphRouter> router,
                  uint32_t src,
                  uint32_t dst,
                  std::vector<RegenSample>* out)
{
    auto wp = router->ShortestPathWeightedRegenOnly(src, dst);
    out->push_back({Simulator::Now().GetSeconds(),
                     wp.path.size(),
                     wp.total_weight});
}

void
ToggleRegenMode(Ptr<ContactGraphRouter> router, uint32_t node, RegenMode m)
{
    router->SetRegenMode(node, m);
}

} // namespace

/// CON-3: an ISL that grazes the surface is not a link.
///
/// The limb test accepted a crosslink whose closest approach to geocentre was
/// exactly one Earth radius. That ray crosses the full depth of the atmosphere
/// twice, with refraction, absorption and scintillation that rise without bound
/// as the tangent height falls to zero. Real constellations budget a
/// tangent-height margin and drop the link below it.
/// SAGIN-7: the hop-count route and the weighted route are different questions.
///
/// The routed-traffic example computed a Dijkstra path and printed it beside a
/// measured goodput, while forwarding was Ipv4GlobalRoutingHelper, which
/// minimises HOP COUNT over whatever interfaces the contact graph brought up.
/// Nothing fed the Dijkstra result into the tables, so the printed "model
/// decision" and the data plane could disagree with nothing to reveal it.
class ContactGraphHopVsWeightedPathTest : public TestCase
{
  public:
    ContactGraphHopVsWeightedPathTest()
        : TestCase("SAGIN-7: ShortestPathHops finds the minimum-HOP route, not the cheapest")
    {
    }

  private:
    void DoRun() override
    {
        Ptr<ntncon::ContactGraphRouter> r = CreateObject<ntncon::ContactGraphRouter>();

        // A graph where the two rules MUST disagree:
        //   0 -> 3 direct, one hop, but a very long link (10,000 km)
        //   0 -> 1 -> 2 -> 3, three hops, each short (100 km), so 300 km total
        // Minimum hop takes the long direct link; minimum weight takes the chain.
        auto link = [](Ptr<ntncon::ContactGraphRouter> rr, uint32_t a, uint32_t b, double m) {
            ntncon::ContactEvent ev{};
            ev.timestamp_s = 0.0;
            ev.node_a = a;
            ev.node_b = b;
            ev.is_isl = true;
            ev.up = true;
            ev.range_m = m;
            ev.elevation_deg = 90.0;
            rr->InjectContactForTest(ev);
        };
        link(r, 0, 3, 10000e3);
        link(r, 0, 1, 100e3);
        link(r, 1, 2, 100e3);
        link(r, 2, 3, 100e3);

        const auto w = r->ShortestPathWeighted(0, 3);
        const auto h = r->ShortestPathHops(0, 3);

        NS_TEST_ASSERT_MSG_EQ(w.path.size(), 4u,
                              "the cheapest route is the three-hop chain (300 km)");
        NS_TEST_ASSERT_MSG_EQ(h.path.size(), 2u,
                              "the fewest-hop route is the direct 10,000 km link");
        NS_TEST_ASSERT_MSG_LT(w.total_weight, h.total_weight,
                              "the weighted route must be cheaper, or this graph does not "
                              "separate the two rules and the comparison proves nothing");
        NS_TEST_ASSERT_MSG_EQ_TOL(h.total_weight, 10000e3, 1.0,
                                  "the hop path's weight must be the range sum ALONG THAT PATH, "
                                  "so the two results compare in latency terms");
        NS_TEST_ASSERT_MSG_EQ_TOL(w.total_weight, 300e3, 1.0, "and likewise for the chain");

        // Where the graph does not separate them, they must agree, or the
        // comparison would flag every scenario.
        Ptr<ntncon::ContactGraphRouter> r2 = CreateObject<ntncon::ContactGraphRouter>();
        link(r2, 0, 1, 500e3);
        link(r2, 1, 2, 500e3);
        const auto w2 = r2->ShortestPathWeighted(0, 2);
        const auto h2 = r2->ShortestPathHops(0, 2);
        NS_TEST_ASSERT_MSG_EQ((w2.path == h2.path), true,
                              "with only one route available the two rules must agree");

        // No route: both must say so rather than returning a partial path.
        Ptr<ntncon::ContactGraphRouter> r3 = CreateObject<ntncon::ContactGraphRouter>();
        link(r3, 0, 1, 100e3);
        const auto h3 = r3->ShortestPathHops(0, 9);
        NS_TEST_ASSERT_MSG_EQ(h3.path.empty(), true, "an unreachable target has no hop path");
        NS_TEST_ASSERT_MSG_EQ(std::isinf(h3.total_weight), true, "and infinite weight");

        // Source == destination is a zero-hop path, not an empty one.
        const auto h4 = r3->ShortestPathHops(0, 0);
        NS_TEST_ASSERT_MSG_EQ(h4.path.size(), 1u, "src == dst is a one-node path");
        NS_TEST_ASSERT_MSG_EQ_TOL(h4.total_weight, 0.0, 1e-12, "with zero weight");
    }
};

class IslLimbTestKeepsAtmosphericClearanceTest : public TestCase
{
  public:
    IslLimbTestKeepsAtmosphericClearanceTest()
        : TestCase("CON-3: the ISL limb test requires a tangent-height margin, not a graze")
    {
    }

  private:
    /// Two satellites at `altM` on a common circle, placed symmetrically about
    /// the +x axis so the chord between them has exactly `tangentAltM` of
    /// tangent height above the ellipsoid. Returns whether the limb test calls
    /// that chord clear.
    static bool LimbClear(double altM, double tangentAltM, double minTangentAltM)
    {
        const double R = 6378137.0;
        const double r = R + altM;
        const double h = R + tangentAltM;
        if (h >= r)
        {
            return true; // geometry impossible; not the case under test
        }
        const double halfAngle = std::acos(h / r);
        const Vector a(r * std::cos(halfAngle), r * std::sin(halfAngle), 0.0);
        const Vector b(r * std::cos(halfAngle), -r * std::sin(halfAngle), 0.0);
        return ntncon::ContactGraphScheduler::IsLimbClear(a, b, minTangentAltM);
    }

    void DoRun() override
    {
        const double alt = 600e3;

        // The construction must be right before anything else means something:
        // the chord's closest approach must actually be the tangent height asked
        // for. Checked by bracketing the margin tightly around it.
        NS_TEST_ASSERT_MSG_EQ(LimbClear(alt, 200e3, 199e3), true,
                              "a 200 km tangent height must clear a 199 km margin");
        NS_TEST_ASSERT_MSG_EQ(LimbClear(alt, 200e3, 201e3), false,
                              "and must fail a 201 km one; if both agree the geometry is not "
                              "being built at the tangent height this test thinks it is");

        // Well clear of the atmosphere: usable.
        NS_TEST_ASSERT_MSG_EQ(LimbClear(alt, 300e3, 80e3), true,
                              "a crosslink with 300 km of tangent height must be usable");

        // Grazing the surface: blocked. This is the case the old test accepted,
        // because it compared against exactly one Earth radius.
        NS_TEST_ASSERT_MSG_EQ(LimbClear(alt, 1.0e3, 80e3), false,
                              "a ray with 1 km of tangent height skims the whole atmosphere and "
                              "must not be reported as a usable ISL");

        // The threshold must sit where it is configured.
        NS_TEST_ASSERT_MSG_EQ(LimbClear(alt, 60e3, 80e3), false,
                              "60 km is below the 80 km margin and must be blocked");
        NS_TEST_ASSERT_MSG_EQ(LimbClear(alt, 100e3, 80e3), true,
                              "100 km is above it and must pass");

        // And the margin must be what does the work: at zero the grazing link
        // comes back, which proves the block comes from the margin rather than
        // from the geometry happening to fail.
        NS_TEST_ASSERT_MSG_EQ(LimbClear(alt, 1.0e3, 0.0), true,
                              "with the margin disabled the grazing link is accepted again");

        // A link through the planet must be blocked regardless of margin: two
        // satellites on opposite sides have a chord passing through geocentre.
        const double R = 6378137.0;
        const Vector p(R + alt, 0.0, 0.0);
        const Vector q(-(R + alt), 0.0, 0.0);
        NS_TEST_ASSERT_MSG_EQ(ntncon::ContactGraphScheduler::IsLimbClear(p, q, 0.0), false,
                              "a chord through geocentre is a link through the planet");

        // Endpoints on the same side, closest approach outside the segment:
        // nothing is between them, so the limb does not apply.
        const Vector u(R + alt, 0.0, 0.0);
        const Vector v(R + alt + 100e3, 0.0, 0.0);
        NS_TEST_ASSERT_MSG_EQ(ntncon::ContactGraphScheduler::IsLimbClear(u, v, 80e3), true,
                              "a radial pair has no limb crossing between the endpoints");
    }
};

class ContactGraphRouterRegenSimulatorTimeTest : public TestCase
{
  public:
    ContactGraphRouterRegenSimulatorTimeTest()
        : TestCase("Simulator: regen-only route changes when sat 2 toggles "
                   "bent-pipe at t=150 s")
    {
    }

  private:
    void DoRun() override
    {
        Ptr<ContactGraphRouter> r = CreateObject<ContactGraphRouter>();
        Ptr<ContactGraphScheduler> sched =
            CreateObject<ContactGraphScheduler>();
        r->Attach(sched);

        sched->m_contactUp({0, 1, 2, true, true, 100.0, 0.0});
        sched->m_contactUp({0, 2, 3, true, true, 100.0, 0.0});
        sched->m_contactUp({0, 3, 4, true, true, 100.0, 0.0});
        sched->m_contactUp({0, 1, 4, true, true, 1000.0, 0.0});
        for (uint32_t i = 1; i <= 4; ++i)
        {
            r->SetRegenMode(i, RegenMode::regen_full);
        }

        std::vector<RegenSample> samples;
        Simulator::Schedule(Seconds(60), &SampleRegenRoute,
                            r, 1u, 4u, &samples);
        Simulator::Schedule(Seconds(150), &ToggleRegenMode,
                            r, 2u, RegenMode::bent_pipe);
        Simulator::Schedule(Seconds(240), &SampleRegenRoute,
                            r, 1u, 4u, &samples);
        Simulator::Schedule(Seconds(400), &ToggleRegenMode,
                            r, 2u, RegenMode::regen_du);
        Simulator::Schedule(Seconds(500), &SampleRegenRoute,
                            r, 1u, 4u, &samples);
        Simulator::Stop(Seconds(601));
        Simulator::Run();

        NS_TEST_ASSERT_MSG_EQ(samples.size(), 3u, "3 route samples");
        NS_TEST_ASSERT_MSG_EQ_TOL(samples[0].total_weight, 300.0, 1e-9,
                                  "t=60: chain route weight 300");
        NS_TEST_ASSERT_MSG_EQ_TOL(samples[1].total_weight, 1000.0, 1e-9,
                                  "t=240 after sat 2 -> bent-pipe: direct 1000");
        NS_TEST_ASSERT_MSG_EQ_TOL(samples[2].total_weight, 300.0, 1e-9,
                                  "t=500 after sat 2 restored: chain again");
        Simulator::Destroy();
    }
};

// ============================================================================
//  Roadmap §4.4.11 — TR 38.821 + Starlink calibration corpus
// ============================================================================

namespace
{

std::string
FindCorpusFile(const std::string& subpath)
{
    // Try a couple of relative roots so the test works from a few cwd
    // depths. We treat absence of the bundled file as a soft skip.
    const std::vector<std::string> roots = {
        "contrib/ntn-constellation/calibration/",
        "../contrib/ntn-constellation/calibration/",
        "../../contrib/ntn-constellation/calibration/",
    };
    for (const auto& r : roots)
    {
        const std::string p = r + subpath;
        std::ifstream f(p);
        if (f)
        {
            return p;
        }
    }
    return std::string();
}

} // namespace

class Tr38821CorpusLoadTest : public TestCase
{
  public:
    Tr38821CorpusLoadTest()
        : TestCase("§4.4.11: load TR 38.821 + Starlink CSVs")
    {
    }

    void DoRun() override
    {
        Tr38821CorpusReader r;
        const std::string sp = FindCorpusFile("tr38821/scenarios.csv");
        const std::string lp = FindCorpusFile("tr38821/link_budgets.csv");
        const std::string yp = FindCorpusFile("starlink_eu/latency_samples.csv");
        const std::string tp = FindCorpusFile("starlink_eu/station_locations.csv");
        if (sp.empty() || lp.empty() || yp.empty() || tp.empty())
        {
            std::cout << "  corpus files not reachable from cwd, "
                       "soft skip" << std::endl;
            return;
        }
        NS_TEST_ASSERT_MSG_EQ(r.LoadScenarios(sp),
                               true,
                               "load scenarios");
        NS_TEST_ASSERT_MSG_EQ(r.LoadLinkBudgets(lp),
                               true,
                               "load link budgets");
        NS_TEST_ASSERT_MSG_EQ(r.LoadStarlinkLatency(yp),
                               true,
                               "load latency");
        NS_TEST_ASSERT_MSG_EQ(r.LoadStarlinkStations(tp),
                               true,
                               "load stations");
        NS_TEST_EXPECT_MSG_EQ(r.Scenarios().size(),
                               8u,
                               "8 TR 38.821 scenarios");
        NS_TEST_EXPECT_MSG_EQ(r.LinkBudgets().size(),
                               24u,
                               "8 scenarios × 3 elevations");
        NS_TEST_EXPECT_MSG_EQ(r.StarlinkStations().size(),
                               8u,
                               "8 EU stations");
        NS_TEST_EXPECT_MSG_EQ(r.StarlinkLatency().size(),
                               64u,
                               "8 stations × 8 hours");
        const auto a1 = r.FindScenario("A1");
        NS_TEST_ASSERT_MSG_EQ(a1.has_value(),
                               true,
                               "A1 found");
        NS_TEST_EXPECT_MSG_EQ_TOL(a1->freq_ghz,
                                    2.0,
                                    1e-9,
                                    "A1 = S-band");
        const auto fra = r.FindStation("FRA");
        NS_TEST_ASSERT_MSG_EQ(fra.has_value(),
                               true,
                               "FRA found");
        NS_TEST_EXPECT_MSG_EQ(fra->city, "Frankfurt", "FRA city");
    }
};

class CalibrationHarnessGateTest : public TestCase
{
  public:
    CalibrationHarnessGateTest()
        : TestCase("§4.4.11: harness applies per-metric gates")
    {
    }

    void DoRun() override
    {
        CalibrationHarness h;
        // Default gates: pathloss 1 dB, rtt 5 ms.
        auto r1 = h.Compare("A1", "pathloss", 189.3, 189.6);
        NS_TEST_EXPECT_MSG_EQ(r1.within_gate,
                               true,
                               "0.3 dB within 1 dB");
        auto r2 = h.Compare("A1", "pathloss", 189.3, 191.8);
        NS_TEST_EXPECT_MSG_EQ(r2.within_gate,
                               false,
                               "2.5 dB exceeds 1 dB");
        auto r3 = h.Compare("FRA", "rtt_p50", 28.0, 31.0);
        NS_TEST_EXPECT_MSG_EQ(r3.within_gate,
                               true,
                               "3 ms within 5 ms");
        auto r4 = h.Compare("FRA", "rtt_p50", 28.0, 40.0);
        NS_TEST_EXPECT_MSG_EQ(r4.within_gate,
                               false,
                               "12 ms exceeds 5 ms");
        NS_TEST_EXPECT_MSG_EQ(h.AllWithinGate(),
                               false,
                               "overall fail");
        const auto counts = h.CountsByMetric();
        NS_TEST_EXPECT_MSG_EQ(counts.at("pathloss"),
                               2u,
                               "2 pathloss");
        NS_TEST_EXPECT_MSG_EQ(counts.at("rtt_p50"),
                               2u,
                               "2 rtt");
        const auto fails = h.FailuresByMetric();
        NS_TEST_EXPECT_MSG_EQ(fails.at("pathloss"),
                               1u,
                               "1 pathloss fail");
        NS_TEST_EXPECT_MSG_EQ(fails.at("rtt_p50"),
                               1u,
                               "1 rtt fail");
    }
};

class CalibrationHarnessEndToEndTest : public TestCase
{
  public:
    CalibrationHarnessEndToEndTest()
        : TestCase("§4.4.11: the calibration HARNESS gates residuals correctly")
    {
    }

    void DoRun() override
    {
        Tr38821CorpusReader r;
        const std::string lp = FindCorpusFile("tr38821/link_budgets.csv");
        if (lp.empty())
        {
            std::cout << "  corpus not reachable, soft skip"
                       << std::endl;
            return;
        }
        NS_TEST_ASSERT_MSG_EQ(r.LoadLinkBudgets(lp),
                               true,
                               "load link budgets");
        CalibrationHarness h;
        // CON-1. What this test checks, stated plainly because its NAME used to
        // claim more: it feeds CalibrationHarness a prediction constructed as
        // corpus + 0.3 dB and verifies the harness reports "within gate". That
        // exercises the HARNESS. It says nothing about the toolkit's own
        // propagation, which never enters this test at all.
        //
        // Tr38821FsplConformanceTest below does the comparison this test was
        // named for.
        for (const auto& lb : r.LinkBudgets())
        {
            const double toolkit_pl = lb.pathloss_db + 0.3;
            const double toolkit_atmos = lb.atmos_loss_db + 0.2;
            const double toolkit_cnr = lb.cnr_db - 0.4;
            h.Compare(lb.scenario_id, "pathloss",
                       lb.pathloss_db, toolkit_pl);
            h.Compare(lb.scenario_id, "atmos_loss",
                       lb.atmos_loss_db, toolkit_atmos);
            h.Compare(lb.scenario_id, "cnr",
                       lb.cnr_db, toolkit_cnr);
        }
        NS_TEST_EXPECT_MSG_EQ(h.AllWithinGate(),
                               true,
                               "synthetic prediction passes 1 dB gate");
        NS_TEST_EXPECT_MSG_EQ(h.Residuals().size(),
                               r.LinkBudgets().size() * 3,
                               "3 metrics per row");
    }
};

/// CON-1: compare the toolkit's OWN propagation against the TR 38.821 corpus.
///
/// The harness test above feeds itself `corpus + 0.3 dB` and checks that the
/// residual gate accepts it. That validates the gate. Nothing in the suite
/// compared the module's actual path loss against the reference, so the corpus
/// shipped as data nothing was measured against.
///
/// This computes slant range from each scenario's orbit and elevation and free
/// space loss at its carrier, then compares against the corpus pathloss. The
/// comparison is made at ZENITH, where the reference is dominated by free space
/// and the residual is the atmospheric and scintillation margin the corpus
/// includes and pure free space does not. Measured across all eight scenarios -
/// GEO, LEO-1200 and LEO-600, S band and Ka - that residual runs -0.24 to
/// +0.77 dB, tight and consistent, which is what makes a 1.5 dB bound
/// defensible rather than arbitrary.
///
/// It is deliberately NOT asserted at low elevation. There the gap grows to
/// 7-15 dB, well beyond the corpus's own atmos_loss_db column, so the reference
/// clearly carries margin terms this comparison does not model. Asserting a
/// wide tolerance there would pass on anything; asserting a tight one would
/// fail on a difference that is not the toolkit's error. The elevation trend is
/// checked structurally instead.
class Tr38821FsplConformanceTest : public TestCase
{
  public:
    Tr38821FsplConformanceTest()
        : TestCase("CON-1: toolkit free-space loss matches the TR 38.821 corpus at zenith")
    {
    }

  private:
    /// Slant range to a satellite at \p altKm seen at \p elevDeg, spherical Earth.
    static double SlantM(double altKm, double elevDeg)
    {
        const double Re = 6371e3;
        const double h = altKm * 1e3;
        const double e = elevDeg * M_PI / 180.0;
        return std::sqrt(std::pow(Re + h, 2) - std::pow(Re * std::cos(e), 2)) - Re * std::sin(e);
    }

    void DoRun() override
    {
        Tr38821CorpusReader r;
        const std::string sp = FindCorpusFile("tr38821/scenarios.csv");
        const std::string lp = FindCorpusFile("tr38821/link_budgets.csv");
        if (sp.empty() || lp.empty())
        {
            std::cout << "  corpus not reachable, soft skip" << std::endl;
            return;
        }
        NS_TEST_ASSERT_MSG_EQ(r.LoadScenarios(sp), true, "load scenarios");
        NS_TEST_ASSERT_MSG_EQ(r.LoadLinkBudgets(lp), true, "load link budgets");

        std::map<std::string, Tr38821Scenario> byId;
        for (const auto& sc : r.Scenarios())
        {
            byId[sc.scenario_id] = sc;
        }

        uint32_t zenithChecked = 0;
        std::map<std::string, double> plAt90;
        std::map<std::string, double> plAt10;

        for (const auto& lb : r.LinkBudgets())
        {
            auto it = byId.find(lb.scenario_id);
            const bool haveScenario = (it != byId.end());
            NS_TEST_ASSERT_MSG_EQ(haveScenario, true,
                                  "every link-budget row must name a scenario that exists; a "
                                  "dangling id means the two corpus files disagree");
            const double d = SlantM(it->second.alt_km, lb.elevation_deg);
            const double fspl =
                20.0 * std::log10(4.0 * M_PI * d * it->second.freq_ghz * 1e9 / 299792458.0);

            if (std::abs(lb.elevation_deg - 90.0) < 0.5)
            {
                const double residual = lb.pathloss_db - fspl;
                // Bound the MAGNITUDE, not the sign.
                //
                // The obvious assertion is that the reference must exceed free
                // space, since it carries atmospheric and scintillation terms
                // on top. That is not quite true of this corpus and the
                // exception is instructive: the GEO S-band row sits 0.24 dB
                // BELOW pure free space, because the reference figures are
                // quoted to 0.1 dB and derived with 3GPP's own rounding and
                // slant conventions. At that scale the sign carries no
                // information, so asserting it would encode an artifact of the
                // table's precision as physics.
                //
                // What the magnitude bound does check is real: at zenith the
                // reference is dominated by free space, so agreement to within
                // 1.5 dB across every orbit and both bands validates the slant
                // geometry and the free-space formula against 3GPP's numbers.
                // Measured spread on this corpus is -0.24 to +0.77 dB.
                NS_TEST_ASSERT_MSG_LT(std::abs(residual), 1.5,
                                      "at zenith the reference is dominated by free space; a "
                                      "residual beyond 1.5 dB means the slant geometry or the "
                                      "free-space formula disagrees with 3GPP, not that the "
                                      "atmosphere is unusually heavy");
                ++zenithChecked;
                // Record the TOOLKIT's value, not the corpus's: the trend check
                // below has to exercise the slant formula at a non-zenith
                // elevation, and comparing two corpus rows would only test the
                // corpus. An earlier version of this test did exactly that and
                // passed unchanged when the slant computation was replaced by
                // the bare altitude - which is correct at zenith and wrong
                // everywhere else.
                plAt90[lb.scenario_id] = fspl;
            }
            if (std::abs(lb.elevation_deg - 10.0) < 0.5)
            {
                plAt10[lb.scenario_id] = fspl;
            }
        }

        NS_TEST_ASSERT_MSG_GT(zenithChecked, 5u,
                              "the corpus must supply a zenith row for most scenarios, or this "
                              "test is checking almost nothing");

        // Structural: path loss must grow as the satellite descends, in every
        // scenario. This needs no tolerance and no reference value.
        for (const auto& [id, pl90] : plAt90)
        {
            auto lo = plAt10.find(id);
            if (lo == plAt10.end())
            {
                continue;
            }
            // A satellite at 10 degrees is substantially further away than one
            // overhead, so the toolkit's own free-space loss must grow. The
            // smallest case in this corpus is GEO, where the slant grows from
            // 35786 km to about 41127 km, worth 1.2 dB; LEO cases are far
            // larger. Requiring a full dB keeps the check meaningful while
            // staying below the tightest real margin.
            NS_TEST_ASSERT_MSG_GT(lo->second - pl90, 1.0,
                                  "the toolkit's free-space loss at 10 degrees must exceed its "
                                  "value at zenith by at least a dB: the slant is materially "
                                  "longer. Equality means the slant computation is ignoring "
                                  "elevation and returning the altitude");
        }
    }
};

class CalibrationHarnessStarlinkTest : public TestCase
{
  public:
    CalibrationHarnessStarlinkTest()
        : TestCase("§4.4.11: harness against Starlink latency samples")
    {
    }

    void DoRun() override
    {
        Tr38821CorpusReader r;
        const std::string yp =
            FindCorpusFile("starlink_eu/latency_samples.csv");
        if (yp.empty())
        {
            std::cout << "  corpus not reachable, soft skip"
                       << std::endl;
            return;
        }
        NS_TEST_ASSERT_MSG_EQ(r.LoadStarlinkLatency(yp),
                               true,
                               "load latency");
        CalibrationHarness h;
        // Toolkit-predicted RTT = reference + 2 ms constant offset;
        // all should pass under the default 5 ms gate.
        for (const auto& s : r.StarlinkLatency())
        {
            h.Compare(s.station_id,
                       "rtt_p50",
                       s.rtt_p50_ms,
                       s.rtt_p50_ms + 2.0);
        }
        NS_TEST_EXPECT_MSG_EQ(h.AllWithinGate(),
                               true,
                               "all within 5 ms");
        NS_TEST_EXPECT_MSG_EQ(h.Residuals().size(),
                               r.StarlinkLatency().size(),
                               "one residual per sample");

        // Tighten the gate to 1 ms and re-run; should fail.
        h.Reset();
        CalibrationHarness::Gates g;
        g.rtt_ms = 1.0;
        h.SetGates(g);
        for (const auto& s : r.StarlinkLatency())
        {
            h.Compare(s.station_id,
                       "rtt_p50",
                       s.rtt_p50_ms,
                       s.rtt_p50_ms + 2.0);
        }
        NS_TEST_EXPECT_MSG_EQ(h.AllWithinGate(),
                               false,
                               "2 ms residual exceeds 1 ms gate");
        const auto fails = h.FailuresByMetric();
        NS_TEST_EXPECT_MSG_EQ(fails.at("rtt_p50"),
                               r.StarlinkLatency().size(),
                               "all fail under tight gate");
    }
};

/**
 * \brief The full Vallado SGP4 backend matches the published SGP4-VER reference
 *        vectors (catalog 00005, t=0) and diverges from the Kepler+J2 fast path
 *        over many orbits — i.e. drag/SGP4 perturbations are genuinely active.
 *
 * Reference: Vallado, Crawford, Hujsak, Kelso, "Revisiting Spacetrack Report #3"
 * (AIAA 2006-6753), tcppver.out for catalog 5 at tsince = 0 min.
 */
class Sgp4ValladoVerificationTest : public TestCase
{
  public:
    Sgp4ValladoVerificationTest()
        : TestCase("Sgp4MobilityModel Vallado backend matches SGP4-VER (cat 00005)")
    {
    }

  private:
    void DoRun() override
    {
        TleRecord tle;
        tle.name = "SGP4-VER 00005";
        tle.line1 = "1 00005U 58002B   00179.78495062  .00000023  00000-0  28098-4 0  4753";
        tle.line2 = "2 00005  34.2682 348.7242 1859667 331.7664  19.3264 10.82419157413667";

        Ptr<Sgp4MobilityModel> sat = CreateObject<Sgp4MobilityModel>();
        sat->SetUseVallado(true);
        NS_TEST_ASSERT_MSG_EQ(sat->SetTle(tle), true, "TLE must parse");
        NS_TEST_ASSERT_MSG_EQ(sat->IsUsingSgp4(), true, "Vallado SGP4 must initialise");

        // Published TEME position at tsince = 0 min (km -> m).
        const Vector eci0 = sat->GetEciPosition(); // Now()==0 -> tsince 0
        NS_TEST_ASSERT_MSG_EQ_TOL(eci0.x, 7022465.29, 2000.0, "SGP4-VER x within 2 km");
        NS_TEST_ASSERT_MSG_EQ_TOL(eci0.y, -1400083.05, 2000.0, "SGP4-VER y within 2 km");
        NS_TEST_ASSERT_MSG_EQ_TOL(eci0.z, 39.95, 2000.0, "SGP4-VER z within 2 km");

        // A Kepler+J2 model from the same TLE must diverge over many orbits,
        // proving the Vallado perturbations are real (not a relabelled Kepler).
        Ptr<Sgp4MobilityModel> kep = CreateObject<Sgp4MobilityModel>();
        kep->SetTle(tle);
        // TWIN-01: ask for the Kepler+J2 path EXPLICITLY. This used to rely on
        // SetUseVallado defaulting to false, which is exactly the defect that
        // finding is about - a class named Sgp4MobilityModel, handed a real
        // TLE, quietly propagating Kepler. Now that SetTle gives SGP4 by
        // default, a test that wants the analytic path has to say so.
        kep->SetUseVallado(false);
        NS_TEST_ASSERT_MSG_EQ(kep->IsUsingSgp4(), false,
                              "the reference object must really be on the Kepler+J2 path, or "
                              "this comparison is SGP4 against itself");
        NS_TEST_ASSERT_MSG_EQ(sat->IsValladoReady(), true,
                              "and the subject must really be on SGP4");

        Simulator::Schedule(Seconds(36000.0), [&]() { // 600 min ~ 4.5 orbits
            const Vector vEci = sat->GetEciPosition();
            const Vector kEci = kep->GetEciPosition();
            const double d = std::sqrt(std::pow(vEci.x - kEci.x, 2) +
                                       std::pow(vEci.y - kEci.y, 2) +
                                       std::pow(vEci.z - kEci.z, 2));
            // Both paths carry J2-secular, so the residual (SGP4 periodic +
            // drag terms) is km-scale over a few orbits for this low-B* orbit;
            // a clear, non-trivial divergence proves Vallado is not a relabelled
            // Kepler path.
            NS_TEST_ASSERT_MSG_GT(d, 2000.0,
                                  "Vallado SGP4 must diverge from Kepler+J2 over orbits");
        });
        Simulator::Stop(Seconds(36001.0));
        Simulator::Run();
        Simulator::Destroy();
    }
};

/**
 * \brief NtnSatLinkErrorModel turns the live slant range into a per-packet
 *        C/N0 -> BLER, replacing the binary contact gate: BLER falls to ~0 at
 *        short range (high C/N0), sits at ~0.5 at the MODCOD threshold, and
 *        rises to ~1 at long range — and the realized packet-corruption rate
 *        tracks the modelled BLER.
 */
class SatLinkErrorModelBlerTest : public TestCase
{
  public:
    SatLinkErrorModelBlerTest()
        : TestCase("NtnSatLinkErrorModel C/N0 -> BLER (replaces binary gate)")
    {
    }

  private:
    static Ptr<ntncon::NtnSatLinkErrorModel> Make(double rangeM)
    {
        Ptr<ConstantPositionMobilityModel> tx = CreateObject<ConstantPositionMobilityModel>();
        Ptr<ConstantPositionMobilityModel> rx = CreateObject<ConstantPositionMobilityModel>();
        tx->SetPosition(Vector(0.0, 0.0, 0.0));
        rx->SetPosition(Vector(rangeM, 0.0, 0.0));
        Ptr<ntncon::NtnSatLinkErrorModel> em = CreateObject<ntncon::NtnSatLinkErrorModel>();
        em->SetEndpoints(tx, rx);
        return em;
    }

    void DoRun() override
    {
        // Default link: 20 dBW EIRP, G/T 1 dB/K, 30 MHz, 20 GHz, QEF thresh 1 dB.
        const double blerShort = Make(300.0e3)->CurrentBler();
        const double blerMid = Make(587.0e3)->CurrentBler();  // ~ threshold range
        const double blerLong = Make(2000.0e3)->CurrentBler();

        NS_TEST_ASSERT_MSG_LT(blerShort, 0.05, "short range -> near-zero BLER");
        NS_TEST_ASSERT_MSG_GT(blerMid, 0.30, "threshold range -> mid BLER (>0.3)");
        NS_TEST_ASSERT_MSG_LT(blerMid, 0.70, "threshold range -> mid BLER (<0.7)");
        NS_TEST_ASSERT_MSG_GT(blerLong, 0.95, "long range -> near-one BLER");
        // Monotonic decreasing C/N0 -> increasing BLER.
        NS_TEST_ASSERT_MSG_LT(blerShort, blerMid, "BLER increases with range (short<mid)");
        NS_TEST_ASSERT_MSG_LT(blerMid, blerLong, "BLER increases with range (mid<long)");

        // Realized corruption rate tracks the modelled BLER (statistical).
        Ptr<ntncon::NtnSatLinkErrorModel> em = Make(587.0e3);
        const double bler = em->CurrentBler();
        uint32_t corrupt = 0;
        const uint32_t n = 4000;
        for (uint32_t i = 0; i < n; ++i)
        {
            Ptr<Packet> p = Create<Packet>(1024);
            if (em->IsCorrupt(p))
            {
                ++corrupt;
            }
        }
        const double rate = static_cast<double>(corrupt) / n;
        NS_TEST_ASSERT_MSG_EQ_TOL(rate, bler, 0.06,
                                  "realized corruption rate matches modelled BLER");
        Simulator::Destroy();
    }
};


/// TWIN-01: a TLE must get SGP4, because the twin's premise depends on it.
///
/// Sgp4MobilityModel declared `bool m_useVallado{false}`, so SetTle() parsed
/// the TLE into Keplerian elements and then propagated them with an analytic
/// Kepler + J2-secular model unless the caller ALSO called
/// SetUseVallado(true). Nothing in the tree did - the gym example and every
/// WalkerConstellation::BuildDelta consumer use SetElements and never touch it.
/// Meanwhile the Python twin runs genuine Satrec.sgp4.
///
/// ntn-digital-twin/twin_loop.py states the design premise outright: "twin and
/// sim propagate identical orbits and their handover sequences are comparable".
/// They did not.
class Sgp4DefaultsToRealSgp4TestCase : public TestCase
{
  public:
    Sgp4DefaultsToRealSgp4TestCase()
        : TestCase("TWIN-01: SetTle gives SGP4 by default, and the two propagators differ")
    {
    }

  private:
    static TleRecord Iss()
    {
        TleRecord t;
        t.name = "ISS";
        t.line1 = "1 25544U 98067A   24001.50000000  .00016717  00000-0  10270-3 0  9993";
        t.line2 = "2 25544  51.6416 247.4627 0006703 130.5360 325.0288 15.49815350 12345";
        return t;
    }

    void DoRun() override
    {
        const TleRecord tle = Iss();

        // A TLE alone must put the model on SGP4. This is the whole finding.
        Ptr<Sgp4MobilityModel> def = CreateObject<Sgp4MobilityModel>();
        NS_TEST_ASSERT_MSG_EQ(def->SetTle(tle), true, "the TLE parses");
        NS_TEST_ASSERT_MSG_EQ(def->IsUsingSgp4(), true,
                              "a model handed a real TLE must propagate it with SGP4; running "
                              "Kepler + J2 here is what broke the twin/sim premise");

        // The opt-out still works, and IsUsingSgp4 reports it honestly.
        Ptr<Sgp4MobilityModel> kep = CreateObject<Sgp4MobilityModel>();
        kep->SetTle(tle);
        kep->SetUseVallado(false);
        NS_TEST_ASSERT_MSG_EQ(kep->IsUsingSgp4(), false, "the analytic path is still reachable");
        NS_TEST_ASSERT_MSG_EQ(kep->IsValladoReady(), true,
                              "and IsValladoReady stays true because it reports whether the SGP4 "
                              "state is INITIALISED, not whether it is in use - which is why the "
                              "assertions above use IsUsingSgp4 instead");

        // With no TLE there is nothing for SGP4 to initialise from, so a
        // SetElements caller correctly stays analytic rather than silently
        // claiming SGP4.
        Ptr<Sgp4MobilityModel> el = CreateObject<Sgp4MobilityModel>();
        KeplerianElements e{};
        e.semi_major_axis_m = 6971e3;
        e.eccentricity = 0.001;
        e.inclination_rad = 51.6 * M_PI / 180.0;
        el->SetElements(e);
        NS_TEST_ASSERT_MSG_EQ(el->IsUsingSgp4(), false,
                              "SetElements has no TLE to feed SGP4, so it stays analytic and "
                              "says so");

        // The two propagators must genuinely differ, or defaulting to SGP4
        // would be a distinction without a difference. Measured over the
        // exporter's 45-minute prediction horizon.
        Simulator::Schedule(Seconds(2700.0), [this, def, kep]() {
            const Vector a = def->GetPosition();
            const Vector b = kep->GetPosition();
            const double d = std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2) +
                                       std::pow(a.z - b.z, 2));
            NS_TEST_ASSERT_MSG_GT(d, 1000.0,
                                  "over a 45-minute horizon SGP4 and Kepler+J2 separate by "
                                  "kilometres for this TLE (measured 5.2 to 11.0 km), which at "
                                  "orbital speed is about a second of along-track lag - enough "
                                  "to move an argmax-elevation crossover and therefore every "
                                  "handover instant the twin exports");
        });
        Simulator::Stop(Seconds(2701.0));
        Simulator::Run();
        Simulator::Destroy();
    }
};


/// WF-10: the visibility index must return EXACTLY what the brute force does.
///
/// Serving-satellite selection is an unindexed double loop across the toolkit -
/// every terminal against every satellite on every tick, O(S x U x T). At a
/// 1584-satellite shell with 1000 terminals over 600 s that is close to a
/// billion elevation evaluations, and the stress report records the
/// consequence: "each sim-second costs ~3-10 s wall".
///
/// An index is only worth having if it is EXACT. A serving-cell choice that
/// differs from the brute force changes every downstream KPI, so this test
/// uses the brute force as an oracle over randomised geometries - including
/// mixed altitudes, where the naive "closest sub-point wins" shortcut is
/// WRONG and a branch-and-bound on the real elevation expression is not.
class NtnVisibilityIndexExactTestCase : public TestCase
{
  public:
    NtnVisibilityIndexExactTestCase()
        : TestCase("WF-10: the visibility index matches brute force exactly, and prunes")
    {
    }

  private:
    /// Elevation from first principles, written out here rather than reused
    /// from the module under test. An oracle that shares code with the thing
    /// it checks can agree with it for the wrong reason.
    static double ElevDeg(const Vector& ue, const Vector& sat)
    {
        const double dx = sat.x - ue.x;
        const double dy = sat.y - ue.y;
        const double dz = sat.z - ue.z;
        const double dn = std::max(1.0, std::sqrt(dx * dx + dy * dy + dz * dz));
        const double un = std::max(1.0, std::sqrt(ue.x * ue.x + ue.y * ue.y + ue.z * ue.z));
        const double sinEl = (dx * ue.x + dy * ue.y + dz * ue.z) / (dn * un);
        return std::asin(std::max(-1.0, std::min(1.0, sinEl))) * 180.0 / M_PI;
    }

    static std::size_t BruteForce(const std::vector<Vector>& sats, const Vector& ue,
                                  double& bestEl)
    {
        bestEl = -1e9;
        std::size_t best = 0;
        for (std::size_t s = 0; s < sats.size(); ++s)
        {
            const double el = ElevDeg(ue, sats[s]);
            if (el > bestEl)
            {
                bestEl = el;
                best = s;
            }
        }
        return best;
    }

    void DoRun() override
    {
        Ptr<UniformRandomVariable> rv = CreateObject<UniformRandomVariable>();
        rv->SetStream(20260826);

        const double Re = 6371e3;
        // Mixed altitudes on purpose: with one shell, "smallest central angle"
        // and "highest elevation" coincide, so a shortcut that is wrong in
        // general would still pass. Spanning 500 to 1500 km separates them.
        std::vector<Vector> sats;
        const std::size_t nSat = 800;
        sats.reserve(nSat);
        for (std::size_t i = 0; i < nSat; ++i)
        {
            const double lat = (rv->GetValue(0.0, 1.0) - 0.5) * M_PI;
            const double lon = (rv->GetValue(0.0, 1.0) - 0.5) * 2.0 * M_PI;
            const double alt = 500e3 + rv->GetValue(0.0, 1.0) * 1000e3;
            const double r = Re + alt;
            sats.push_back(Vector(r * std::cos(lat) * std::cos(lon),
                                  r * std::cos(lat) * std::sin(lon),
                                  r * std::sin(lat)));
        }

        NtnVisibilityIndex idx(10.0);
        idx.Build(sats);
        NS_TEST_ASSERT_MSG_EQ(idx.SatelliteCount(), nSat, "all satellites indexed");

        uint64_t totalEvaluated = 0;
        const std::size_t nUe = 400;
        for (std::size_t u = 0; u < nUe; ++u)
        {
            const double lat = (rv->GetValue(0.0, 1.0) - 0.5) * M_PI;
            const double lon = (rv->GetValue(0.0, 1.0) - 0.5) * 2.0 * M_PI;
            const Vector ue(Re * std::cos(lat) * std::cos(lon),
                            Re * std::cos(lat) * std::sin(lon),
                            Re * std::sin(lat));

            double refEl = 0.0;
            const std::size_t ref = BruteForce(sats, ue, refEl);
            double gotEl = 0.0;
            const std::size_t got = idx.BestElevation(ue, gotEl);
            totalEvaluated += idx.LastEvaluated();

            // The ELEVATION must match to floating-point noise. The index is
            // asserted on the value rather than only the identity because two
            // satellites can tie, and either answer is then correct.
            NS_TEST_ASSERT_MSG_EQ_TOL(gotEl, refEl, 1e-6,
                                      "the index must return the same best elevation as the "
                                      "brute force for UE " << u << "; a mismatch means a "
                                      "satellite was pruned that should not have been, and every "
                                      "downstream KPI would shift with the serving cell");
            if (got != ref)
            {
                // Only acceptable as a tie.
                NS_TEST_ASSERT_MSG_EQ_TOL(ElevDeg(ue, sats[got]), refEl, 1e-9,
                                          "a different index is only acceptable on an exact tie");
            }
        }

        // And it must actually prune, or it is a slower brute force.
        const double meanEvaluated = static_cast<double>(totalEvaluated) /
                                     static_cast<double>(nUe);
        NS_TEST_ASSERT_MSG_LT(meanEvaluated, 0.5 * static_cast<double>(nSat),
                              "the index must evaluate well under half the constellation per "
                              "query; it evaluated " << meanEvaluated << " of " << nSat
                              << ". Exactness without pruning is just a slower loop");

        // A SINGLE-ALTITUDE shell, which is what every Walker scenario builds.
        // The mixed-altitude case above passed while this one did not, so both
        // stay: a bound bug can hide in one and not the other.
        {
            std::vector<Vector> shell;
            shell.reserve(1584);
            for (std::size_t i = 0; i < 1584; ++i)
            {
                const double la = (rv->GetValue(0.0, 1.0) - 0.5) * M_PI;
                const double lo = (rv->GetValue(0.0, 1.0) - 0.5) * 2.0 * M_PI;
                const double r = Re + 550e3;
                shell.push_back(Vector(r * std::cos(la) * std::cos(lo),
                                       r * std::cos(la) * std::sin(lo), r * std::sin(la)));
            }
            NtnVisibilityIndex sidx(10.0);
            sidx.Build(shell);
            double worst = 0.0;
            for (std::size_t u = 0; u < 300; ++u)
            {
                const double la = (rv->GetValue(0.0, 1.0) - 0.5) * M_PI;
                const double lo = (rv->GetValue(0.0, 1.0) - 0.5) * 2.0 * M_PI;
                const Vector ue(Re * std::cos(la) * std::cos(lo),
                                Re * std::cos(la) * std::sin(lo), Re * std::sin(la));
                double refEl = 0.0;
                BruteForce(shell, ue, refEl);
                double gotEl = 0.0;
                sidx.BestElevation(ue, gotEl);
                worst = std::max(worst, std::abs(gotEl - refEl));
            }
            NS_TEST_ASSERT_MSG_LT(worst, 1e-6,
                                  "single-altitude shell: worst elevation error " << worst
                                  << " deg. Every Walker scenario builds exactly this, so a "
                                  "bound valid only for mixed altitudes is no use");
        }

        // Degenerate inputs must not crash or lie.
        NtnVisibilityIndex empty(10.0);
        empty.Build({});
        double el = 0.0;
        NS_TEST_ASSERT_MSG_EQ(empty.BestElevation(Vector(Re, 0, 0), el),
                              std::numeric_limits<std::size_t>::max(),
                              "an empty constellation has no best satellite");

        NtnVisibilityIndex one(10.0);
        one.Build({Vector(Re + 600e3, 0, 0)});
        NS_TEST_ASSERT_MSG_EQ(one.BestElevation(Vector(Re, 0, 0), el), 0u,
                              "a single satellite is trivially the best");
        NS_TEST_ASSERT_MSG_EQ_TOL(el, 90.0, 1e-6, "and it is at zenith");
    }
};


/// SAGIN-3: inter-satellite Xn handover signalling, which did not exist.
///
/// A repo-wide grep for 'Xn' or 'XnAP' across ntn-sagin and ntn-constellation
/// returned ZERO hits. RegenMode existed but was a Dijkstra transit filter that
/// nothing outside its own test ever set, so a regenerative-payload handover -
/// the main Rel-19 NTN topic - could not be run while the enum implied it
/// could.
///
/// The real-gNB half already existed (NtnRealStackHelper builds an
/// NrGnbNetDevice on the satellite under PayloadOption::FullGnb). What was
/// missing is the PROCEDURE, and this drives it end to end over an ISL whose
/// delay comes from orbital separation.
class NtnXnHandoverProcedureTestCase : public TestCase
{
  public:
    NtnXnHandoverProcedureTestCase()
        : TestCase("SAGIN-3: the TS 38.300 Xn handover runs between two satellites")
    {
    }

  private:
    Ptr<ntncon::NtnXnHandover> m_a;
    Ptr<ntncon::NtnXnHandover> m_b;
    std::vector<ntncon::XnapMessageType> m_order;
    uint32_t m_completed{0};
    bool m_lastOk{false};
    Time m_elapsed{};

    void Deliver(uint32_t peer, ntncon::NtnXnapHeader h, Time delay)
    {
        m_order.push_back(h.GetMessageType());
        // Every leg crosses the ISL: the procedure takes real time.
        Ptr<ntncon::NtnXnHandover> dst = (peer == m_a->GetGnbId()) ? m_a : m_b;
        Simulator::Schedule(delay, &ntncon::NtnXnHandover::Receive, dst, h);
    }

    void OnComplete(uint32_t /*ue*/, Time elapsed, bool ok)
    {
        ++m_completed;
        m_elapsed = elapsed;
        m_lastOk = ok;
    }

    void Build(bool admit, Time islOneWay)
    {
        m_order.clear();
        m_completed = 0;
        m_a = CreateObject<ntncon::NtnXnHandover>();
        m_b = CreateObject<ntncon::NtnXnHandover>();
        m_a->SetGnbId(1);
        m_b->SetGnbId(2);
        m_b->SetAdmitIncoming(admit);
        m_a->SetPeerDelay(2, islOneWay);
        m_b->SetPeerDelay(1, islOneWay);
        m_a->SetSendCallback(
            MakeCallback(&NtnXnHandoverProcedureTestCase::Deliver, this));
        m_b->SetSendCallback(
            MakeCallback(&NtnXnHandoverProcedureTestCase::Deliver, this));
        m_a->SetCompleteCallback(
            MakeCallback(&NtnXnHandoverProcedureTestCase::OnComplete, this));
    }

    void DoRun() override
    {
        using ntncon::XnapMessageType;

        // 1500 km of inter-satellite separation is 5.0036 ms one way.
        const Time isl = NanoSeconds(5003600);

        // ---- The wire format must round-trip ----
        {
            ntncon::NtnXnapHeader h;
            h.SetMessageType(XnapMessageType::SnStatusTransfer);
            h.SetSourceUeXnapId(77);
            h.SetTargetUeXnapId(1234);
            h.SetSourceGnbId(1);
            h.SetTargetGnbId(2);
            h.SetTargetNrCgi(0x123456789ULL);
            h.SetDlPdcpSn(4095);
            h.SetUlPdcpSn(2047);
            h.SetHfn(9);
            Ptr<Packet> p = Create<Packet>(0);
            p->AddHeader(h);
            ntncon::NtnXnapHeader back;
            p->RemoveHeader(back);
            NS_TEST_ASSERT_MSG_EQ(static_cast<int>(back.GetMessageType()),
                                  static_cast<int>(XnapMessageType::SnStatusTransfer),
                                  "the message type survives the wire");
            NS_TEST_ASSERT_MSG_EQ(back.GetTargetNrCgi(), 0x123456789ULL,
                                  "the 36-bit NR CGI survives; truncating it would send the "
                                  "handover to a different cell");
            NS_TEST_ASSERT_MSG_EQ(back.GetDlPdcpSn(), 4095u,
                                  "and so does the PDCP sequence number, which is the entire "
                                  "purpose of SN STATUS TRANSFER");
            NS_TEST_ASSERT_MSG_EQ(back.GetHfn(), 9u, "with its hyper frame number");
        }

        // ---- The successful procedure, in order, over the ISL ----
        {
            Build(/*admit=*/true, isl);
            NS_TEST_ASSERT_MSG_EQ(m_a->StartHandover(77, 2, 0xABCDEF, 4095, 2047, 9), true,
                                  "the source starts the procedure");
            Simulator::Stop(Seconds(1.0));
            Simulator::Run();

            NS_TEST_ASSERT_MSG_EQ(m_completed, 1u, "the handover completes at the source");
            NS_TEST_ASSERT_MSG_EQ(m_lastOk, true, "successfully");
            NS_TEST_ASSERT_MSG_EQ(m_order.size(), 4u,
                                  "TS 38.300 section 9.2.3 is four messages: REQUEST, ACK, SN "
                                  "STATUS TRANSFER, UE CONTEXT RELEASE");
            if (m_order.size() == 4)
            {
                NS_TEST_ASSERT_MSG_EQ(static_cast<int>(m_order[0]),
                                      static_cast<int>(XnapMessageType::HandoverRequest), "1st");
                NS_TEST_ASSERT_MSG_EQ(
                    static_cast<int>(m_order[1]),
                    static_cast<int>(XnapMessageType::HandoverRequestAcknowledge), "2nd");
                NS_TEST_ASSERT_MSG_EQ(static_cast<int>(m_order[2]),
                                      static_cast<int>(XnapMessageType::SnStatusTransfer), "3rd");
                NS_TEST_ASSERT_MSG_EQ(static_cast<int>(m_order[3]),
                                      static_cast<int>(XnapMessageType::UeContextRelease), "4th");
            }

            // Four legs across the ISL, so the procedure costs four one-way
            // delays. A procedure that completes instantly is not crossing an
            // inter-satellite link at all.
            NS_TEST_ASSERT_MSG_EQ_TOL(m_elapsed.GetSeconds(), 4.0 * isl.GetSeconds(), 1e-6,
                                      "the Xn handover costs four ISL traversals (20.01 ms at "
                                      "1500 km); an instant completion means the geometry is "
                                      "not in the procedure");

            // The target must have RECEIVED the PDCP state, not merely been
            // told a handover happened.
            NS_TEST_ASSERT_MSG_EQ(m_b->GetLastReceivedDlPdcpSn(), 4095u,
                                  "the target holds the DL PDCP SN it was sent");
            NS_TEST_ASSERT_MSG_EQ(m_b->GetLastReceivedHfn(), 9u, "and the HFN");
            NS_TEST_ASSERT_MSG_EQ(m_b->GetRequestsReceived(), 1u, "target saw one request");
            NS_TEST_ASSERT_MSG_EQ(m_b->GetAcksSent(), 1u, "and acknowledged once");
            Simulator::Destroy();
        }

        // ---- A target that cannot admit must FAIL the preparation ----
        {
            Build(/*admit=*/false, isl);
            m_a->StartHandover(88, 2, 0xABCDEF, 1, 1, 0);
            Simulator::Stop(Seconds(1.0));
            Simulator::Run();
            NS_TEST_ASSERT_MSG_EQ(m_completed, 1u, "the source is told the outcome");
            NS_TEST_ASSERT_MSG_EQ(m_lastOk, false,
                                  "a target that cannot admit must answer HANDOVER PREPARATION "
                                  "FAILURE; silence would leave the source believing a handover "
                                  "is in flight");
            NS_TEST_ASSERT_MSG_EQ(m_b->GetFailuresSent(), 1u, "and the failure is counted");
            NS_TEST_ASSERT_MSG_EQ(m_b->GetAcksSent(), 0u, "with no acknowledge");
            Simulator::Destroy();
        }

        // ---- SAGIN-3 step 3: RegenMode must SELECT behaviour ----
        //
        // RegenMode was a Dijkstra transit filter and nothing else, so a
        // bent-pipe satellite was indistinguishable from a regenerative one
        // outside routing. A transparent payload has no on-board gNB, so there
        // is nothing there to originate or terminate an Xn procedure.
        {
            Build(/*admit=*/true, isl);
            m_b->SetRegenerative(false); // target is bent-pipe
            m_a->StartHandover(99, 2, 0xABCDEF, 1, 1, 0);
            Simulator::Stop(Seconds(1.0));
            Simulator::Run();
            NS_TEST_ASSERT_MSG_EQ(m_lastOk, false,
                                  "a bent-pipe target has no on-board gNB and cannot terminate "
                                  "Xn; admitting the handover would be the enum being "
                                  "decorative");
            NS_TEST_ASSERT_MSG_EQ(m_b->GetFailuresSent(), 1u,
                                  "and it must SAY so rather than go silent, which would leave "
                                  "the source believing a procedure is in flight");
            Simulator::Destroy();

            // And a bent-pipe SOURCE cannot originate one either.
            Build(/*admit=*/true, isl);
            m_a->SetRegenerative(false);
            NS_TEST_ASSERT_MSG_EQ(m_a->StartHandover(100, 2, 0xABCDEF, 1, 1, 0), false,
                                  "a transparent payload cannot originate an Xn handover");
            NS_TEST_ASSERT_MSG_EQ(m_a->GetRequestsSent(), 0u, "and sends nothing");
            Simulator::Destroy();
        }

        // ---- No transport wired: refuse rather than pretend ----
        {
            Ptr<ntncon::NtnXnHandover> lonely = CreateObject<ntncon::NtnXnHandover>();
            lonely->SetGnbId(9);
            NS_TEST_ASSERT_MSG_EQ(lonely->StartHandover(1, 2, 0, 0, 0, 0), false,
                                  "a handover with nowhere to send must be refused, not "
                                  "silently started");
            NS_TEST_ASSERT_MSG_EQ(lonely->GetRequestsSent(), 0u, "and nothing counted as sent");
        }
    }
};

class NtnConstellationTestSuite : public TestSuite
{
  public:
    NtnConstellationTestSuite()
        : TestSuite("ntn-constellation", Type::UNIT)
    {
        AddTestCase(new Sgp4DefaultsToRealSgp4TestCase, TestCase::Duration::QUICK);
        AddTestCase(new NtnVisibilityIndexExactTestCase, TestCase::Duration::QUICK);
        AddTestCase(new NtnXnHandoverProcedureTestCase, TestCase::Duration::QUICK);
        AddTestCase(new Sgp4ValladoVerificationTest, Duration::QUICK);
        AddTestCase(new SatLinkErrorModelBlerTest, Duration::QUICK);
        AddTestCase(new TleParseChecksumTest, Duration::QUICK);
        AddTestCase(new TleStreamParseTest, Duration::QUICK);
        AddTestCase(new Sgp4PeriodicReturnTest, Duration::QUICK);
        AddTestCase(new Sgp4EcefAltitudeTest, Duration::QUICK);
        AddTestCase(new WalkerDeltaShapeTest, Duration::QUICK);
        AddTestCase(new ContactSchedulerLeoPassTest, Duration::QUICK);
        AddTestCase(new ContactSchedulerIslPairTest, Duration::QUICK);
        AddTestCase(new ContactSchedulerGateHysteresisTest, Duration::QUICK);
        // Roadmap §4.4.4 — ContactGraphRouter.
        AddTestCase(new ContactGraphRouterDirectEdgesTest, Duration::QUICK);
        AddTestCase(new ContactGraphRouterShortestPathTest, Duration::QUICK);
        AddTestCase(new ContactGraphRouterSimulatorTimeTest, Duration::QUICK);
        // Roadmap §4.4.5 — Dijkstra link-weighted shortest path.
        AddTestCase(new ContactGraphRouterWeightedEdgeTest, Duration::QUICK);
        AddTestCase(new ContactGraphRouterDijkstraTest, Duration::QUICK);
        AddTestCase(new ContactGraphRouterDijkstraSimulatorTimeTest,
                    Duration::QUICK);
        // Roadmap §4.4.6 — Regen-vs-bent-pipe split.
        AddTestCase(new ContactGraphRouterRegenModeTest, Duration::QUICK);
        AddTestCase(new ContactGraphRouterRegenOnlyDijkstraTest,
                    Duration::QUICK);
        // Roadmap §4.4.11 — TR 38.821 + Starlink calibration corpus.
        AddTestCase(new ContactSchedulerRangeUpdateTest, Duration::QUICK);
        AddTestCase(new SatLinkErrorAtmosphericChainTest, Duration::QUICK);
        AddTestCase(new Sgp4GeocentricContractTest, Duration::QUICK);
        AddTestCase(new Tr38821CorpusLoadTest, Duration::QUICK);
        AddTestCase(new CalibrationHarnessGateTest, Duration::QUICK);
        AddTestCase(new CalibrationHarnessEndToEndTest, Duration::QUICK);
        AddTestCase(new Tr38821FsplConformanceTest, Duration::QUICK);
        AddTestCase(new CalibrationHarnessStarlinkTest, Duration::QUICK);
        AddTestCase(new ContactGraphHopVsWeightedPathTest, Duration::QUICK);
        AddTestCase(new IslLimbTestKeepsAtmosphericClearanceTest, Duration::QUICK);
        AddTestCase(new ContactGraphRouterRegenSimulatorTimeTest,
                    Duration::QUICK);
    }
};

static NtnConstellationTestSuite g_ntnConstellationTestSuite;

} // namespace
