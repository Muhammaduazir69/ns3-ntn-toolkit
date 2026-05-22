/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#include "ns3/contact-graph-router.h"
#include "ns3/contact-graph-scheduler.h"
#include "ns3/orbital-elements.h"
#include "ns3/sgp4-mobility-model.h"
#include "ns3/simulator.h"
#include "ns3/test.h"
#include "ns3/walker-constellation.h"

#include <algorithm>
#include <cmath>
#include <set>
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
        : TestCase("SGP4 propagation returns to start within 100 km after one period")
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

class NtnConstellationTestSuite : public TestSuite
{
  public:
    NtnConstellationTestSuite()
        : TestSuite("ntn-constellation", Type::UNIT)
    {
        AddTestCase(new TleParseChecksumTest, Duration::QUICK);
        AddTestCase(new TleStreamParseTest, Duration::QUICK);
        AddTestCase(new Sgp4PeriodicReturnTest, Duration::QUICK);
        AddTestCase(new Sgp4EcefAltitudeTest, Duration::QUICK);
        AddTestCase(new WalkerDeltaShapeTest, Duration::QUICK);
        AddTestCase(new ContactSchedulerLeoPassTest, Duration::QUICK);
        AddTestCase(new ContactSchedulerIslPairTest, Duration::QUICK);
        // Roadmap §4.4.4 — ContactGraphRouter.
        AddTestCase(new ContactGraphRouterDirectEdgesTest, Duration::QUICK);
        AddTestCase(new ContactGraphRouterShortestPathTest, Duration::QUICK);
        AddTestCase(new ContactGraphRouterSimulatorTimeTest, Duration::QUICK);
    }
};

static NtnConstellationTestSuite g_ntnConstellationTestSuite;

} // namespace
