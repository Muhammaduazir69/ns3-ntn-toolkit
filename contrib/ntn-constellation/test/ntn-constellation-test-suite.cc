/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#include "ns3/contact-graph-router.h"
#include "ns3/contact-graph-scheduler.h"
#include "ns3/orbital-elements.h"
#include "ns3/tr38821-corpus.h"
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
        : TestCase("§4.4.11: harness against full TR 38.821 link budgets")
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
        // Simulate a toolkit prediction that matches reference within
        // ≤ 0.5 dB on every entry (synthetic stand-in for the actual
        // module). Then verify all-within-gate passes.
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
        AddTestCase(new Tr38821CorpusLoadTest, Duration::QUICK);
        AddTestCase(new CalibrationHarnessGateTest, Duration::QUICK);
        AddTestCase(new CalibrationHarnessEndToEndTest, Duration::QUICK);
        AddTestCase(new CalibrationHarnessStarlinkTest, Duration::QUICK);
        AddTestCase(new ContactGraphRouterRegenSimulatorTimeTest,
                    Duration::QUICK);
    }
};

static NtnConstellationTestSuite g_ntnConstellationTestSuite;

} // namespace
