/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#include "ns3/constant-position-mobility-model.h"
#include "ns3/log.h"
#include "ns3/node-container.h"
#include "ns3/nr-epc-helper.h"
#include "ns3/ntn-real-stack-helper.h"
#include "ns3/ntn-twin-agreement.h"
#include "ns3/ntn-twin-replay.h"
#include "ns3/oran-ntn-twin-prediction-consumer.h"
#include "ns3/ntn-visibility-index.h"
#include "ns3/orbital-elements.h"
#include "ns3/sgp4-mobility-model.h"
#include "ns3/simulator.h"
#include "ns3/test.h"

#include <cmath>
#include <cstdlib>
#include <fstream>
#include <functional>
#include <iostream>
#include <limits>
#include <vector>

using namespace ns3;
using namespace ns3::ntntwin;

namespace
{

/// Where the twin's artifacts live, if a run produced them.
std::string ArtifactDir()
{
    const char* env = std::getenv("NTN_TWIN_ARTIFACT_DIR");
    return env ? std::string(env) : std::string();
}

} // namespace

/// TWIN-02: the agreement metric itself must behave.
///
/// Before this, "twin/sim agreement" was measured by comparing two Python
/// elevation formulas over one Python-propagated constellation - a tautology at
/// the physics level - and recorded as 100%. This tests the metric that
/// replaces it, on sequences whose correct answer is known by construction.
class TwinAgreementMetricTestCase : public TestCase
{
  public:
    TwinAgreementMetricTestCase()
        : TestCase("TWIN-02: the handover-agreement metric distinguishes agreement from noise")
    {
    }

  private:
    void DoRun() override
    {
        const std::vector<TwinHandover> a = {{100.0, 0, 5, 1.0},
                                             {200.0, 0, 9, 1.0},
                                             {300.0, 0, 12, 1.0}};

        // Identical sequences agree completely.
        {
            const auto r = CompareHandovers(a, a, 10.0);
            NS_TEST_ASSERT_MSG_EQ(r.matched, 3u, "all three pair up");
            NS_TEST_ASSERT_MSG_EQ_TOL(r.agreement, 1.0, 1e-9, "identical sequences agree");
            NS_TEST_ASSERT_MSG_EQ_TOL(r.meanAbsOffsetS, 0.0, 1e-9, "with no offset");
        }

        // Same cells, shifted in time but inside the window: still agreement,
        // and the offset is REPORTED rather than hidden.
        {
            std::vector<TwinHandover> b = a;
            for (auto& h : b)
            {
                h.tSec += 4.0;
            }
            const auto r = CompareHandovers(a, b, 10.0);
            NS_TEST_ASSERT_MSG_EQ(r.matched, 3u, "a 4 s shift is inside a 10 s window");
            NS_TEST_ASSERT_MSG_EQ_TOL(r.meanAbsOffsetS, 4.0, 1e-9,
                                      "and the offset is reported; an agreement figure without "
                                      "it cannot distinguish 'the same decision' from 'the same "
                                      "decision half a minute late'");
        }

        // Outside the window: no agreement.
        {
            std::vector<TwinHandover> b = a;
            for (auto& h : b)
            {
                h.tSec += 60.0;
            }
            const auto r = CompareHandovers(a, b, 10.0);
            NS_TEST_ASSERT_MSG_EQ(r.matched, 0u, "a 60 s shift is outside a 10 s window");
            NS_TEST_ASSERT_MSG_EQ_TOL(r.agreement, 0.0, 1e-9, "so nothing agrees");
        }

        // RIGHT TIME, WRONG CELL must NOT count. This is the case that makes
        // the metric worth having: an agreement score that matches on the
        // instant alone would call a handover to the wrong satellite a success.
        {
            std::vector<TwinHandover> b = a;
            for (auto& h : b)
            {
                h.gnbId += 100;
            }
            const auto r = CompareHandovers(a, b, 10.0);
            NS_TEST_ASSERT_MSG_EQ(r.matched, 0u,
                                  "handing over at the right instant to the WRONG cell is not "
                                  "agreement");
        }

        // A twin that predicts a subset cannot score 100%: the denominator is
        // the longer sequence.
        {
            const std::vector<TwinHandover> few = {{100.0, 0, 5, 1.0}};
            const auto r = CompareHandovers(few, a, 10.0);
            NS_TEST_ASSERT_MSG_EQ(r.matched, 1u, "the one prediction pairs up");
            NS_TEST_ASSERT_MSG_LT(r.agreement, 0.4,
                                  "but predicting one of three handovers is not agreement; "
                                  "dividing by the twin's own count would have scored this 100%");
        }
    }
};

namespace
{

/// The committed twin run: Walker 66 satellites in 6 planes of 11 at 53 deg,
/// 600 km, shared epoch 2025-01-01T00:00Z, observer 33.68 N 73.05 E, 2700 s
/// horizon, 10 s step (README, "One deterministic twin run").
std::string FixtureDir()
{
    return std::string(NS_TEST_SOURCEDIR) + "/data/twin-walker";
}

/// Write `body` to `path` (a test temp-dir file) and return the path.
std::string WriteTemp(const std::string& path, const std::string& body)
{
    std::ofstream f(path);
    f << body;
    return path;
}

/// Replay the committed fixture (or `dir`) under `policy` and return the replay.
struct ReplayResult
{
    std::vector<TwinHandover> handovers;
    uint32_t initialCell{0};
    double initialT{-1.0};
    std::size_t samples{0};
    std::size_t noCoverage{0};
    uint64_t fallbacks{0};
    std::size_t sats{0};
    bool scheduled{false};
    std::vector<double> probeElevDeg; //!< elevations probed at requested instants
};

ReplayResult RunReplay(const std::string& dir,
                       TwinReplayPolicy policy,
                       double epochSkewS = 0.0,
                       double observerLonShiftDeg = 0.0,
                       const std::vector<std::pair<double, uint32_t>>& probes = {})
{
    ReplayResult out;
    TwinReplay replay;
    out.sats = replay.SetTles(LoadTleDump(dir + "/twin-orbits.tle"));
    TwinRunMeta meta = LoadTwinRunMeta(dir + "/twin-pred.txt");
    meta.observerLonDeg += observerLonShiftDeg;
    replay.SetRunMeta(meta);
    replay.SetPolicy(policy);
    replay.SetEpochSkewS(epochSkewS);
    out.scheduled = replay.Schedule();
    out.probeElevDeg.assign(probes.size(), -90.0);
    for (std::size_t i = 0; i < probes.size(); ++i)
    {
        // Probe after the sampling event at the same instant.
        Simulator::Schedule(Seconds(probes[i].first) + MicroSeconds(1), [&out, &replay, &probes, i]() {
            out.probeElevDeg[i] = replay.ElevationOfCellDeg(probes[i].second);
        });
    }
    Simulator::Stop(Seconds(meta.horizonS + meta.stepS));
    Simulator::Run();
    out.handovers = replay.GetHandovers();
    out.initialCell = replay.GetInitialCell();
    out.initialT = replay.GetInitialAcquisitionS();
    out.samples = replay.GetSampleCount();
    out.noCoverage = replay.GetSamplesWithoutCoverage();
    out.fallbacks = replay.GetSgp4Fallbacks();
    Simulator::Destroy();
    return out;
}

} // namespace

/// TWIN-02 gate, artifact variant: when a fresh twin run is pointed at by
/// NTN_TWIN_ARTIFACT_DIR, replay it under the twin's own A3 guard and compare.
/// Without the variable the committed fixture is used (see the fixture cases).
class TwinSimHandoverAgreementTestCase : public TestCase
{
  public:
    TwinSimHandoverAgreementTestCase()
        : TestCase("TWIN-02: ns-3 replays the twin's orbits and the handovers are compared")
    {
    }

  private:
    void DoRun() override
    {
        const std::string env = ArtifactDir();
        const std::string dir = env.empty() ? FixtureDir() : env;
        const auto twin = LoadTwinPredictions(dir + "/twin-pred.txt");
        const auto meta = LoadTwinRunMeta(dir + "/twin-pred.txt");
        NS_TEST_ASSERT_MSG_EQ(meta.valid, true,
                              "the prediction file must carry the observer and window it was "
                              "generated with");
        NS_TEST_ASSERT_MSG_GT(twin.size(), 0u, "the twin predicted handovers");
        const auto r = RunReplay(dir, TwinReplayPolicy::A3Guard);
        NS_TEST_ASSERT_MSG_EQ(r.sats, LoadTleDump(dir + "/twin-orbits.tle").size(),
                              "every TLE the twin propagated loads into ns-3 SGP4");
        NS_TEST_ASSERT_MSG_GT(r.handovers.size(), 0u,
                              "ns-3 produces a handover sequence of its own");
        const auto agree = CompareHandovers(twin, r.handovers, 2.0 * meta.stepS);
        std::cout << "  [TWIN-02] twin=" << agree.twinCount << " sim=" << agree.simCount
                  << " matched=" << agree.matched << " agreement=" << agree.agreement
                  << " meanOffset=" << agree.meanAbsOffsetS << " s\n";
        NS_TEST_ASSERT_MSG_LT(agree.agreement, 1.0001, "agreement is a fraction");
        NS_TEST_ASSERT_MSG_GT(agree.agreement, -0.0001, "and non-negative");
    }
};

/// The prediction-file parser: comments, blanks, malformed rows, ordering.
/// Failure paths: a missing file, a non-numeric field and a short row must be
/// skipped, never turned into a fabricated prediction.
class TwinPredictionParserTestCase : public TestCase
{
  public:
    TwinPredictionParserTestCase()
        : TestCase("Twin prediction file: comments, malformed rows, ordering, missing file")
    {
    }

  private:
    void DoRun() override
    {
        const std::string path = WriteTemp(CreateTempDirFilename("pred.txt"),
                                           "# ntn-twin handover predictions  epoch_unix=1.0\n"
                                           "# t_s,ueId,recommendedGnbId,confidence\n"
                                           "\n"
                                           "300.0,0,7,0.5\n"
                                           "abc,0,3,0.9\n"      // non-numeric time
                                           "120.0,1\n"          // too few fields
                                           "100.0,0,4\n"        // no confidence: 0
                                           "200.0,2,5,0.75\n"
                                           "250.0,0,x,0.2\n");  // non-numeric cell
        const auto v = LoadTwinPredictions(path);
        NS_TEST_ASSERT_MSG_EQ(v.size(), 3u, "three well-formed rows survive");
        NS_TEST_ASSERT_MSG_EQ_TOL(v[0].tSec, 100.0, 1e-12, "sorted by time");
        NS_TEST_ASSERT_MSG_EQ(v[0].gnbId, 4u, "cell kept");
        NS_TEST_ASSERT_MSG_EQ_TOL(v[0].confidence, 0.0, 1e-12, "missing confidence reads 0");
        NS_TEST_ASSERT_MSG_EQ(v[1].ueId, 2u, "UE id kept");
        NS_TEST_ASSERT_MSG_EQ_TOL(v[2].tSec, 300.0, 1e-12, "last row last");
        NS_TEST_ASSERT_MSG_EQ(LoadTwinPredictions(path + ".absent").size(), 0u,
                              "a missing file yields no predictions");

        // The committed fixture: five A3 handovers, all for UE 0, cells in range.
        const auto fx = LoadTwinPredictions(FixtureDir() + "/twin-pred.txt");
        NS_TEST_ASSERT_MSG_EQ(fx.size(), 5u, "five twin handovers in the fixture");
        for (const auto& h : fx)
        {
            NS_TEST_ASSERT_MSG_GT(h.gnbId, 0u, "cells are 1-based");
            NS_TEST_ASSERT_MSG_LT(h.gnbId, 67u, "and name one of the 66 satellites");
            NS_TEST_ASSERT_MSG_GT(h.confidence, 0.0, "confidence in (0, 1]");
            NS_TEST_ASSERT_MSG_LT(h.confidence, 1.0001, "confidence in (0, 1]");
        }
    }
};

/// The run-metadata header (TWIN-02): the observer and window travel with the
/// predictions. A file without them must be reported invalid, not defaulted to
/// (0 N, 0 E), which would replay a different scenario without saying so.
class TwinRunMetaParserTestCase : public TestCase
{
  public:
    TwinRunMetaParserTestCase()
        : TestCase("Twin run header: observer and window parsed; incomplete headers rejected")
    {
    }

  private:
    void DoRun() override
    {
        const auto fx = LoadTwinRunMeta(FixtureDir() + "/twin-pred.txt");
        NS_TEST_ASSERT_MSG_EQ(fx.valid, true, "fixture header is complete");
        NS_TEST_ASSERT_MSG_EQ_TOL(fx.epochUnix, 1735689600.0, 1e-3, "2025-01-01T00:00Z");
        NS_TEST_ASSERT_MSG_EQ_TOL(fx.observerLatDeg, 33.68, 1e-9, "observer latitude");
        NS_TEST_ASSERT_MSG_EQ_TOL(fx.observerLonDeg, 73.05, 1e-9, "observer longitude");
        NS_TEST_ASSERT_MSG_EQ_TOL(fx.horizonS, 2700.0, 1e-9, "horizon");
        NS_TEST_ASSERT_MSG_EQ_TOL(fx.stepS, 10.0, 1e-9, "step");

        const auto noObs = LoadTwinRunMeta(
            WriteTemp(CreateTempDirFilename("noobs.txt"), "# epoch_unix=5\n# horizon_s=10 step_s=1\n1,0,1,1\n"));
        NS_TEST_ASSERT_MSG_EQ(noObs.valid, false, "no observer line: invalid");

        const auto zeroStep = LoadTwinRunMeta(WriteTemp(CreateTempDirFilename("zerostep.txt"),
            "# observer_lat_deg=1 observer_lon_deg=2 observer_alt_m=3\n# horizon_s=10 step_s=0\n"));
        NS_TEST_ASSERT_MSG_EQ(zeroStep.valid, false, "zero step: invalid");

        // The header ends at the first data row: keys after it are not header.
        const auto late = LoadTwinRunMeta(
            WriteTemp(CreateTempDirFilename("late.txt"),
                      "# observer_lat_deg=1 observer_lon_deg=2 observer_alt_m=3\n"
                      "5,0,1,1\n# horizon_s=10 step_s=1\n"));
        NS_TEST_ASSERT_MSG_EQ(late.valid, false, "window after data is not header");

        NS_TEST_ASSERT_MSG_EQ(LoadTwinRunMeta("/nonexistent/twin-pred.txt").valid, false,
                              "a missing file is invalid");
    }
};

/// The TLE dump loader keeps only well-formed triples (TLE format, line 1
/// starts '1', line 2 starts '2', 69 columns: CelesTrak / NORAD two-line
/// element format).
class TwinTleLoaderTestCase : public TestCase
{
  public:
    TwinTleLoaderTestCase()
        : TestCase("Twin TLE dump: 66 Walker TLEs load; malformed triples rejected")
    {
    }

  private:
    void DoRun() override
    {
        const auto fx = LoadTleDump(FixtureDir() + "/twin-orbits.tle");
        NS_TEST_ASSERT_MSG_EQ(fx.size(), 66u, "6 planes x 11 satellites");
        NS_TEST_ASSERT_MSG_EQ(fx[0].line1.substr(0, 2), "1 ", "line 1 marker");
        NS_TEST_ASSERT_MSG_EQ(fx[0].line2.substr(0, 2), "2 ", "line 2 marker");
        // 14.89339512 rev/day is a 600 km circular orbit: n = sqrt(mu / a^3),
        // a = 6378.137 + 600 km, mu = 398600.4418 km^3/s^2 (WGS-84).
        const double a = 6378.137 + 600.0;
        const double revPerDay = std::sqrt(398600.4418 / (a * a * a)) * 86400.0 / (2.0 * M_PI);
        NS_TEST_ASSERT_MSG_EQ_TOL(std::stod(fx[0].line2.substr(52, 11)),
                                  revPerDay,
                                  0.02,
                                  "mean motion is that of the 600 km shell");

        const std::string good1 = fx[0].line1;
        const std::string good2 = fx[0].line2;
        std::string bad2 = good2;
        bad2[0] = '3';
        const std::string body = "OK\n" + good1 + "\n" + good2 + "\n" + "WRONG MARKER\n" + good1 +
                                 "\n" + bad2 + "\n" + "SHORT\n" + good1.substr(0, 40) + "\n" +
                                 good2 + "\n" + "DANGLING\n" + good1 + "\n";
        const auto v = LoadTleDump(WriteTemp(CreateTempDirFilename("dump.tle"), body));
        NS_TEST_ASSERT_MSG_EQ(v.size(), 1u, "only the well-formed triple loads");
        NS_TEST_ASSERT_MSG_EQ(v[0].name, "OK", "and keeps its name");
        NS_TEST_ASSERT_MSG_EQ(LoadTleDump("/nonexistent.tle").size(), 0u, "missing file: none");
    }
};

/// CompareHandovers edge cases: one-to-one pairing, inclusive window, empty
/// inputs, nearest-first choice.
class TwinCompareEdgeTestCase : public TestCase
{
  public:
    TwinCompareEdgeTestCase()
        : TestCase("Agreement metric: one-to-one pairing, inclusive window, empty sequences")
    {
    }

  private:
    void DoRun() override
    {
        // Two twin events on the same cell near ONE sim event: only one pairs.
        {
            const std::vector<TwinHandover> twin = {{100.0, 0, 3, 1}, {104.0, 0, 3, 1}};
            const std::vector<TwinHandover> sim = {{102.0, 0, 3, 1}};
            const auto r = CompareHandovers(twin, sim, 10.0);
            NS_TEST_ASSERT_MSG_EQ(r.matched, 1u, "a sim event pairs with at most one twin event");
            NS_TEST_ASSERT_MSG_EQ_TOL(r.agreement, 0.5, 1e-12, "1 / max(2, 1)");
        }
        // The window is inclusive.
        {
            const std::vector<TwinHandover> twin = {{100.0, 0, 3, 1}};
            const auto in = CompareHandovers(twin, {{110.0, 0, 3, 1}}, 10.0);
            const auto out = CompareHandovers(twin, {{110.001, 0, 3, 1}}, 10.0);
            NS_TEST_ASSERT_MSG_EQ(in.matched, 1u, "|dt| == window matches");
            NS_TEST_ASSERT_MSG_EQ(out.matched, 0u, "|dt| > window does not");
        }
        // Nearest sim event is chosen when several qualify.
        {
            const std::vector<TwinHandover> twin = {{100.0, 0, 3, 1}};
            const std::vector<TwinHandover> sim = {{93.0, 0, 3, 1}, {101.0, 0, 3, 1}};
            const auto r = CompareHandovers(twin, sim, 10.0);
            NS_TEST_ASSERT_MSG_EQ_TOL(r.meanAbsOffsetS, 1.0, 1e-12, "nearest event chosen");
            NS_TEST_ASSERT_MSG_EQ_TOL(r.maxAbsOffsetS, 1.0, 1e-12, "max offset reported");
        }
        // Empty sequences: counts reported, no agreement, no division by zero.
        {
            const std::vector<TwinHandover> twin = {{100.0, 0, 3, 1}};
            const auto r1 = CompareHandovers(twin, {}, 10.0);
            const auto r2 = CompareHandovers({}, twin, 10.0);
            const auto r3 = CompareHandovers({}, {}, 10.0);
            NS_TEST_ASSERT_MSG_EQ(r1.twinCount, 1u, "twin count reported");
            NS_TEST_ASSERT_MSG_EQ(r2.simCount, 1u, "sim count reported");
            NS_TEST_ASSERT_MSG_EQ_TOL(r1.agreement + r2.agreement + r3.agreement,
                                      0.0,
                                      1e-12,
                                      "no agreement without both sequences");
            NS_TEST_ASSERT_MSG_EQ(std::isfinite(r3.meanAbsOffsetS), true, "finite offset");
        }
    }
};

/// The twin's link budget against published reference values.
///  - TR 38.811 Section 6.6.2: slant range d equals h at zenith and
///    sqrt(2 R h + h^2) at the horizon.
///  - ITU-R P.525-4 Eq. (6): FSPL = 32.45 + 20 log10(f MHz) + 20 log10(d km),
///    a different constant than the 147.55 dB form the twin uses; both must
///    agree within 0.01 dB.
///  - TR 38.821 Table 6.1.1.1-1 / -2 Set-1 terms: EIRP 34 dBW/MHz over 20 MHz,
///    G/T -31.6 dB/K, k = -228.6 dBW/K/Hz.
class TwinLinkBudgetConformanceTestCase : public TestCase
{
  public:
    TwinLinkBudgetConformanceTestCase()
        : TestCase("Twin link budget: TR 38.811 slant range and ITU-R P.525 FSPL, Set-1 SNR")
    {
    }

  private:
    void DoRun() override
    {
        const double R = 6371e3;
        const double h = 600e3;
        NS_TEST_ASSERT_MSG_EQ_TOL(TwinSlantRangeM(90.0, 600.0), h, 1e-3, "zenith slant = h");
        NS_TEST_ASSERT_MSG_EQ_TOL(TwinSlantRangeM(0.0, 600.0),
                                  std::sqrt(2.0 * R * h + h * h),
                                  1e-3,
                                  "horizon slant = sqrt(2Rh + h^2) = 2829.3 km");
        NS_TEST_ASSERT_MSG_EQ_TOL(TwinSlantRangeM(-5.0, 600.0),
                                  TwinSlantRangeM(0.0, 600.0),
                                  1e-6,
                                  "negative elevation clamps to the horizon");
        NS_TEST_ASSERT_MSG_EQ_TOL(TwinSlantRangeM(10.0, 600.0), 1932e3, 1.0e3,
                                  "TR 38.811 Table 4.2-2: LEO-600 at 10 deg is 1932 km");

        TwinA3Params p;
        const double snrZenith = TwinSnrDb(p, 90.0, 600.0);
        // Independent P.525 path: 77 dBm EIRP -> 47 dBW.
        const double fsplP525 = 32.45 + 20.0 * std::log10(2000.0) + 20.0 * std::log10(600.0);
        const double ref = 47.0 - fsplP525 - 31.6 + 228.6 - 10.0 * std::log10(20e6);
        NS_TEST_ASSERT_MSG_EQ_TOL(snrZenith, ref, 0.01, "Set-1 zenith SNR, P.525 cross-check");
        NS_TEST_ASSERT_MSG_EQ_TOL(snrZenith, 16.96, 0.05, "about 17 dB at zenith");
        NS_TEST_ASSERT_MSG_EQ_TOL(TwinSnrDb(p, 0.0, 600.0), -999.0, 1e-9,
                                  "below the horizon is unusable");
        NS_TEST_ASSERT_MSG_EQ_TOL(TwinSnrDb(p, 90.0, 600.0) - TwinSnrDb(p, 30.0, 600.0),
                                  20.0 * std::log10(TwinSlantRangeM(30.0, 600.0) / h),
                                  1e-9,
                                  "SNR differences are pure FSPL differences");
        // A3 compares differences, so the EIRP and G/T cancel: the admitted
        // handovers cannot depend on them.
        TwinA3Params q = p;
        q.eirpDbm = 62.0;
        q.gtDbPerK = 1.1;
        NS_TEST_ASSERT_MSG_EQ_TOL(TwinSnrDb(q, 20.0, 600.0) - TwinSnrDb(q, 50.0, 600.0),
                                  TwinSnrDb(p, 20.0, 600.0) - TwinSnrDb(p, 50.0, 600.0),
                                  1e-9,
                                  "budget terms cancel in the A3 margin");
        NS_TEST_ASSERT_MSG_EQ_TOL(TwinAltitudeKmFromEcef(Vector(6378135.0 + 600e3, 0, 0)),
                                  600.0,
                                  1e-9,
                                  "altitude from ECEF");
        NS_TEST_ASSERT_MSG_EQ_TOL(TwinAltitudeKmFromEcef(Vector(1.0, 0, 0)), 0.0, 1e-12,
                                  "never negative");
    }
};

/// The A3 evaluator port against the TWIN-04 semantics, tick by tick. Each
/// failure path is one the Python guard documents.
class TwinA3SemanticsTestCase : public TestCase
{
  public:
    TwinA3SemanticsTestCase()
        : TestCase("Twin A3 guard: acquisition, strict margin, TTT restart, min service, outage")
    {
    }

  private:
    /// Elevation at which a 600 km satellite is `db` weaker than one at `el0`.
    static double ElevForDelta(double el0, double db)
    {
        TwinA3Params p;
        const double target = TwinSnrDb(p, el0, 600.0) - db;
        double lo = 0.01;
        double hi = el0;
        for (int i = 0; i < 100; ++i)
        {
            const double mid = 0.5 * (lo + hi);
            (TwinSnrDb(p, mid, 600.0) < target ? lo : hi) = mid;
        }
        return 0.5 * (lo + hi);
    }

    void DoRun() override
    {
        TwinA3Event ev;
        // 1. Initial acquisition is not a handover.
        {
            TwinA3Evaluator e;
            NS_TEST_ASSERT_MSG_EQ(e.Step(0.0, {{0, 30.0, 600.0}, {1, 20.0, 600.0}}, ev), false,
                                  "acquisition is not a handover");
            NS_TEST_ASSERT_MSG_EQ(e.GetServing(), 0u, "best SNR is acquired");
        }
        // 2. The margin must exceed the offset: 2.99 dB does not fire, 3.2 dB does.
        {
            TwinA3Evaluator e;
            e.Step(0.0, {{0, 60.0, 600.0}}, ev);
            const double el3 = ElevForDelta(60.0, 2.99);
            NS_TEST_ASSERT_MSG_EQ(e.Step(10.0, {{0, el3, 600.0}, {1, 60.0, 600.0}}, ev), false,
                                  "a margin below the A3 offset does not fire");
            const double el32 = ElevForDelta(60.0, 3.2);
            NS_TEST_ASSERT_MSG_EQ(e.Step(20.0, {{0, el32, 600.0}, {1, 60.0, 600.0}}, ev), true,
                                  "3.2 dB fires");
            NS_TEST_ASSERT_MSG_EQ(ev.keyIn, 1u, "into the stronger cell");
            NS_TEST_ASSERT_MSG_EQ(ev.keyOut, 0u, "out of the serving one");
            NS_TEST_ASSERT_MSG_EQ_TOL(ev.marginDb, 3.2, 1e-6, "margin reported in dB");
        }
        // 3. TTT restarts when the target changes and when the condition lapses.
        {
            TwinA3Params p;
            p.timeToTriggerS = 20.0;
            TwinA3Evaluator e(p);
            const double weak = ElevForDelta(70.0, 6.0);
            e.Step(0.0, {{0, 70.0, 600.0}}, ev);
            NS_TEST_ASSERT_MSG_EQ(e.Step(10.0, {{0, weak, 600.0}, {1, 70.0, 600.0}}, ev), false,
                                  "TTT starts for cell 1");
            NS_TEST_ASSERT_MSG_EQ(e.Step(20.0, {{0, weak, 600.0}, {2, 70.0, 600.0}}, ev), false,
                                  "best changed to cell 2: window restarts");
            NS_TEST_ASSERT_MSG_EQ(e.Step(30.0, {{0, weak, 600.0}, {2, 70.0, 600.0}}, ev), false,
                                  "10 s held < 20 s TTT");
            NS_TEST_ASSERT_MSG_EQ(e.Step(40.0, {{0, 70.0, 600.0}, {2, 70.0, 600.0}}, ev), false,
                                  "condition lapses");
            NS_TEST_ASSERT_MSG_EQ(e.Step(50.0, {{0, weak, 600.0}, {2, 70.0, 600.0}}, ev), false,
                                  "re-entry starts over");
            NS_TEST_ASSERT_MSG_EQ(e.Step(60.0, {{0, weak, 600.0}, {2, 70.0, 600.0}}, ev), false,
                                  "10 s held");
            NS_TEST_ASSERT_MSG_EQ(e.Step(70.0, {{0, weak, 600.0}, {2, 70.0, 600.0}}, ev), true,
                                  "20 s held fires");
            NS_TEST_ASSERT_MSG_EQ_TOL(ev.tSec, 70.0, 1e-12, "at the tick TTT elapses");
        }
        // 4. Minimum service time holds the next handover back.
        {
            TwinA3Params p;
            p.minServiceS = 30.0;
            TwinA3Evaluator e(p);
            const double weak = ElevForDelta(70.0, 6.0);
            e.Step(0.0, {{0, 70.0, 600.0}}, ev);
            NS_TEST_ASSERT_MSG_EQ(e.Step(10.0, {{0, weak, 600.0}, {1, 70.0, 600.0}}, ev), true,
                                  "first handover (no previous one)");
            NS_TEST_ASSERT_MSG_EQ(e.Step(20.0, {{1, weak, 600.0}, {2, 70.0, 600.0}}, ev), false,
                                  "10 s after a handover < 30 s min service");
            NS_TEST_ASSERT_MSG_EQ(e.Step(40.0, {{1, weak, 600.0}, {2, 70.0, 600.0}}, ev), true,
                                  "30 s after it fires");
        }
        // 5. Outage: nothing visible keeps the serving cell and resets TTT;
        //    a serving cell out of view keeps its last geometry.
        {
            TwinA3Params p;
            p.minElevationDeg = 10.0;
            TwinA3Evaluator e(p);
            e.Step(0.0, {{0, 40.0, 600.0}}, ev);
            NS_TEST_ASSERT_MSG_EQ(e.Step(10.0, {{0, 5.0, 600.0}, {1, 9.0, 600.0}}, ev), false,
                                  "all below the minimum elevation: no handover");
            NS_TEST_ASSERT_MSG_EQ(e.GetServing(), 0u, "serving cell kept through the outage");
            // Serving drops below 10 deg; candidate at 30 deg vs last-known 40 deg.
            NS_TEST_ASSERT_MSG_EQ(e.Step(20.0, {{0, 5.0, 600.0}, {1, 30.0, 600.0}}, ev), false,
                                  "last-known geometry is the worst case for the candidate");
        }
    }
};

/// Fixture replay under the twin's own A3 guard (Simulator::Run): ns-3 SGP4
/// (Vallado, AIAA 2006-6753) plus TEME->ECEF from the WGS-84 geodetic observer
/// must reproduce every twin handover on the same 10 s tick. This is the
/// measurement the TWIN-02 gate exists for, and it scored 0 of 5 while the
/// replay used an elevation argmax from a spherical observer.
class TwinFixtureA3AgreementTestCase : public TestCase
{
  public:
    TwinFixtureA3AgreementTestCase()
        : TestCase("Fixture: ns-3 SGP4 replay under the twin A3 guard reproduces all 5 handovers")
    {
    }

  private:
    void DoRun() override
    {
        const auto twin = LoadTwinPredictions(FixtureDir() + "/twin-pred.txt");
        const auto r = RunReplay(FixtureDir(), TwinReplayPolicy::A3Guard);
        NS_TEST_ASSERT_MSG_EQ(r.scheduled, true, "replay scheduled");
        NS_TEST_ASSERT_MSG_EQ(r.sats, 66u, "66 satellites on SGP4");
        NS_TEST_ASSERT_MSG_EQ(r.fallbacks, 0u, "no SGP4 propagation failure in 45 min");
        NS_TEST_ASSERT_MSG_EQ(r.samples, 271u, "0..2700 s at 10 s = 271 samples");
        NS_TEST_ASSERT_MSG_EQ(r.noCoverage, 0u, "a 66-satellite shell covers 33.68 N throughout");
        NS_TEST_ASSERT_MSG_EQ(r.initialCell, 15u, "initial acquisition: cell 15 (Skyfield)");
        NS_TEST_ASSERT_MSG_EQ_TOL(r.initialT, 0.0, 1e-9, "at t = 0");
        const auto a = CompareHandovers(twin, r.handovers, 0.0);
        std::cout << "  [A3 replay] twin=" << a.twinCount << " sim=" << a.simCount
                  << " matched=" << a.matched << " (same tick)\n";
        NS_TEST_ASSERT_MSG_EQ(r.handovers.size(), twin.size(), "same number of handovers");
        NS_TEST_ASSERT_MSG_EQ(a.matched, twin.size(), "every handover on the same tick and cell");
        NS_TEST_ASSERT_MSG_EQ_TOL(a.agreement, 1.0, 1e-12, "full agreement");
    }
};

/// Fixture replay under the elevation argmax against an independent Skyfield
/// 1.54 / sgp4 2.25 computation of the same argmax from the same observer
/// (offline, embedded below). Also checks per-satellite elevations at the twin's
/// handover instants within 0.02 deg: SGP4 implementations agree to meters
/// (Vallado 2006), and the GMST-only TEME->ECEF rotation leaves under 0.01 deg
/// at these ranges.
class TwinFixtureArgmaxSkyfieldTestCase : public TestCase
{
  public:
    TwinFixtureArgmaxSkyfieldTestCase()
        : TestCase("Fixture: ns-3 argmax replay equals Skyfield's; elevations within 0.02 deg")
    {
    }

  private:
    void DoRun() override
    {
        // Skyfield argmax-elevation serving changes (t s, 1-based cell); initial 15 at t=0.
        const std::vector<TwinHandover> sky = {{100, 0, 24, 1},  {510, 0, 14, 1},
                                               {700, 0, 23, 1},  {1050, 0, 13, 1},
                                               {1310, 0, 33, 1}, {1590, 0, 12, 1},
                                               {1910, 0, 32, 1}, {2130, 0, 22, 1},
                                               {2510, 0, 31, 1}, {2670, 0, 21, 1}};
        // Skyfield elevations (deg) of selected cells at selected instants.
        const std::vector<std::pair<double, uint32_t>> probes = {
            {0, 15}, {280, 24}, {610, 14}, {700, 23}, {1100, 13}, {1650, 12}, {2180, 22}};
        const std::vector<double> skyEl = {7.7891, 16.5621, 10.3135, 11.2124, 9.0831, 9.8323, 9.5536};

        const auto r = RunReplay(FixtureDir(), TwinReplayPolicy::ArgmaxElevation, 0.0, 0.0, probes);
        NS_TEST_ASSERT_MSG_EQ(r.initialCell, 15u, "initial cell");
        const auto a = CompareHandovers(sky, r.handovers, 0.0);
        NS_TEST_ASSERT_MSG_EQ(r.handovers.size(), sky.size(), "ten argmax changes");
        NS_TEST_ASSERT_MSG_EQ(a.matched, sky.size(), "all on the same tick and cell");
        for (std::size_t i = 0; i < probes.size(); ++i)
        {
            NS_TEST_ASSERT_MSG_EQ_TOL(r.probeElevDeg[i],
                                      skyEl[i],
                                      0.02,
                                      "elevation of cell " << probes[i].second << " at t="
                                                           << probes[i].first << " s");
        }
    }
};

/// Determinism, and the regression the replay fixes. The same scenario twice
/// gives identical handover sequences; the argmax policy scores 0 of 5 against
/// the twin's A3 predictions (the old gate's number), so a test that compared
/// those would measure the policy gap, not mobility.
class TwinReplayDeterminismTestCase : public TestCase
{
  public:
    TwinReplayDeterminismTestCase()
        : TestCase("Replay determinism; argmax-vs-A3 policy gap reproduces the old 0-of-5 score")
    {
    }

  private:
    void DoRun() override
    {
        const auto r1 = RunReplay(FixtureDir(), TwinReplayPolicy::A3Guard);
        const auto r2 = RunReplay(FixtureDir(), TwinReplayPolicy::A3Guard);
        NS_TEST_ASSERT_MSG_EQ(r1.handovers.size(), r2.handovers.size(), "same count");
        for (std::size_t i = 0; i < std::min(r1.handovers.size(), r2.handovers.size()); ++i)
        {
            NS_TEST_ASSERT_MSG_EQ(r1.handovers[i].gnbId, r2.handovers[i].gnbId, "same cell");
            NS_TEST_ASSERT_MSG_EQ_TOL(r1.handovers[i].tSec, r2.handovers[i].tSec, 0.0, "same tick");
        }
        const auto twin = LoadTwinPredictions(FixtureDir() + "/twin-pred.txt");
        const auto argmax = RunReplay(FixtureDir(), TwinReplayPolicy::ArgmaxElevation);
        const auto gap = CompareHandovers(twin, argmax.handovers, 20.0);
        NS_TEST_ASSERT_MSG_EQ(gap.matched, 0u,
                              "argmax vs the twin's A3 predictions: 0 matched in a 20 s window");
        NS_TEST_ASSERT_MSG_GT(argmax.handovers.size(), r1.handovers.size(),
                              "argmax hands over more often than the 3 dB guard");
    }
};

/// Failure paths of the replay: a clock that disagrees with the twin's shared
/// epoch, an observer in the wrong place, a twin naming cells that do not
/// exist, and nothing to replay.
class TwinReplayFailurePathsTestCase : public TestCase
{
  public:
    TwinReplayFailurePathsTestCase()
        : TestCase("Replay failure paths: epoch skew, wrong observer, unknown cells, no input")
    {
    }

  private:
    void DoRun() override
    {
        const auto twin = LoadTwinPredictions(FixtureDir() + "/twin-pred.txt");

        // A 60 s clock skew moves every handover by about 6 ticks: outside a
        // 2-step window, so the gate detects an epoch mismatch.
        const auto skew = RunReplay(FixtureDir(), TwinReplayPolicy::A3Guard, 60.0);
        const auto aSkew = CompareHandovers(twin, skew.handovers, 20.0);
        NS_TEST_ASSERT_MSG_LT(aSkew.agreement, 0.5, "a 60 s epoch skew breaks agreement");

        // A skew inside one step shifts by at most one tick and is still matched,
        // with the offset reported.
        const auto small = RunReplay(FixtureDir(), TwinReplayPolicy::A3Guard, 4.0);
        const auto aSmall = CompareHandovers(twin, small.handovers, 10.0);
        NS_TEST_ASSERT_MSG_EQ(aSmall.matched, twin.size(), "4 s skew: all within one step");
        NS_TEST_ASSERT_MSG_LT(aSmall.maxAbsOffsetS, 10.0001, "offset at most one step");

        // Observer 15 deg of longitude away sees a different pass sequence.
        const auto moved = RunReplay(FixtureDir(), TwinReplayPolicy::A3Guard, 0.0, 15.0);
        const auto aMoved = CompareHandovers(twin, moved.handovers, 20.0);
        NS_TEST_ASSERT_MSG_LT(aMoved.agreement, 0.5, "a misplaced observer is detected");

        // Twin predictions naming cells beyond the constellation never match.
        std::vector<TwinHandover> ghost = twin;
        for (auto& h : ghost)
        {
            h.gnbId += 66;
        }
        const auto base = RunReplay(FixtureDir(), TwinReplayPolicy::A3Guard);
        NS_TEST_ASSERT_MSG_EQ(CompareHandovers(ghost, base.handovers, 20.0).matched, 0u,
                              "cells 67..132 do not exist");

        // Nothing to replay.
        TwinReplay empty;
        NS_TEST_ASSERT_MSG_EQ(empty.SetTles({}), 0u, "no TLEs");
        empty.SetRunMeta(LoadTwinRunMeta(FixtureDir() + "/twin-pred.txt"));
        NS_TEST_ASSERT_MSG_EQ(empty.Schedule(), false, "no satellites: nothing scheduled");
        TwinReplay noMeta;
        noMeta.SetTles(LoadTleDump(FixtureDir() + "/twin-orbits.tle"));
        noMeta.SetRunMeta(TwinRunMeta());
        NS_TEST_ASSERT_MSG_EQ(noMeta.Schedule(), false, "invalid header: nothing scheduled");
        NS_TEST_ASSERT_MSG_EQ_TOL(noMeta.ElevationOfCellDeg(0), -90.0, 1e-12, "cell 0 invalid");
        NS_TEST_ASSERT_MSG_EQ_TOL(noMeta.ElevationOfCellDeg(67), -90.0, 1e-12, "cell 67 invalid");
        Simulator::Destroy();
    }
};

/// Cross-module (twin -> oran-ntn -> ntn-traffic -> nr): a twin prediction file
/// is read by OranNtnTwinPredictionConsumer, which actuates a real X2 handover
/// through NtnRealStackHelper on the 5G-LENA stack, while URLLC packets flow.
/// Failure paths in the same run: a low-confidence prediction is filtered, and
/// a prediction naming a cell that does not exist is refused and counted.
/// The completed handover sequence is then scored with CompareHandovers.
class TwinDrivesRealHandoverTestCase : public TestCase
{
  public:
    TwinDrivesRealHandoverTestCase()
        : TestCase("Cross-module: twin predictions actuate a real NR handover via the O-RAN consumer")
    {
    }

  private:
    void DoRun() override
    {
        const std::string pred = WriteTemp(CreateTempDirFilename("pred.txt"),
                                           "# ntn-twin handover predictions  epoch_unix=0\n"
                                           "# t_s,ueId,recommendedGnbId,confidence\n"
                                           "3.0,0,2,0.9\n"
                                           "4.5,0,9,0.95\n"
                                           "5.0,0,1,0.05\n");
        NodeContainer sats;
        sats.Create(2);
        NodeContainer ues;
        ues.Create(1);
        for (uint32_t i = 0; i < 2; ++i)
        {
            auto m = CreateObject<ConstantPositionMobilityModel>();
            m->SetPosition(NtnRealStackHelper::GeodeticToEcef(40.0 + 2.5 * i, -3.0, 600e3));
            sats.Get(i)->AggregateObject(m);
        }
        auto um = CreateObject<ConstantPositionMobilityModel>();
        um->SetPosition(NtnRealStackHelper::GeodeticToEcef(40.0, -3.0, 0.0));
        ues.Get(0)->AggregateObject(um);

        NtnRealStackHelper rs;
        rs.SetRadioBackend(NtnRealStackHelper::RadioBackend::Nr);
        rs.SetNumerology(1);
        rs.SetSimTime(Seconds(7.0));
        rs.SetOutputDir(CreateTempDirFilename("rs"));
        rs.SetHandover(true, 3.0, MilliSeconds(256));
        rs.SetAutonomousHandover(false);
        rs.SetSatEirpDensityDbwMhz(NtnRealStackHelper::kTr38821Set1SBandEirpDensityDbwMhz);
        rs.Build(sats, ues);
        rs.InstallTraffic(NtnRealStackHelper::TrafficProfile::UrllcPings, Seconds(1.0), Seconds(6.8));

        auto consumer = CreateObject<OranNtnTwinPredictionConsumer>();
        NS_TEST_ASSERT_MSG_EQ(consumer->LoadPredictionsFromFile(pred), 3u,
                              "the O-RAN consumer reads the twin's file format");
        NS_TEST_ASSERT_MSG_EQ(LoadTwinPredictions(pred).size(), 3u,
                              "and agrees with the twin-side parser");
        uint32_t accepted = 0;
        uint32_t refused = 0;
        consumer->Start(MakeCallback(
                            +[](NtnRealStackHelper* h, uint32_t* acc, uint32_t* ref, uint32_t ue,
                                uint32_t gnb, double) {
                                const uint16_t cell =
                                    (gnb >= 1 && gnb <= 2) ? h->GetGnbCellId(gnb - 1) : 77;
                                (h->TriggerHandover(ue, cell) ? *acc : *ref) += 1;
                            })
                            .Bind(&rs, &accepted, &refused),
                        0.5);

        // Measure: serving-cell changes every 50 ms, app packets before and after.
        std::vector<TwinHandover> sim;
        uint16_t lastCell = 0;
        uint64_t rxBefore = 0;
        std::function<void()> poll = [&]() {
            const uint16_t c = rs.GetUeServingCellId(0);
            if (lastCell != 0 && c != lastCell)
            {
                uint32_t idx = (c == rs.GetGnbCellId(1)) ? 2 : 1;
                sim.push_back({Simulator::Now().GetSeconds(), 0, idx, 1.0});
            }
            lastCell = c;
            Simulator::Schedule(MilliSeconds(50), poll);
        };
        Simulator::Schedule(Seconds(0.5), poll);
        Simulator::Schedule(Seconds(2.95), [&]() { rxBefore = rs.GetUeRxPackets(0); });
        Simulator::Stop(Seconds(7.0));
        Simulator::Run();
        rs.Collect();
        const uint64_t rxEnd = rs.GetUeRxPackets(0);

        NS_TEST_ASSERT_MSG_EQ(consumer->GetActuatedCount(), 2u,
                              "the 0.05-confidence prediction is filtered (min 0.5)");
        NS_TEST_ASSERT_MSG_EQ(accepted, 1u, "the prediction for gNB 2 is accepted");
        NS_TEST_ASSERT_MSG_EQ(refused, 1u, "gNB 9 does not exist: refused");
        NS_TEST_ASSERT_MSG_GT(rs.GetHandoverRefusals(), 0u, "and the refusal is counted");
        NS_TEST_ASSERT_MSG_EQ(rs.GetHandoverCount(), 1u, "exactly one completed X2 handover");
        NS_TEST_ASSERT_MSG_EQ(rs.GetUeServingCellId(0), rs.GetGnbCellId(1),
                              "the UE ends on the twin's recommended cell");
        NS_TEST_ASSERT_MSG_GT(rxBefore, 100u, "URLLC packets delivered before (10 ms cadence)");
        NS_TEST_ASSERT_MSG_GT(rxEnd, rxBefore + 250u,
                              "and keep flowing through the target cell after the handover");
        // Score the actuated handover against the twin's accepted prediction.
        const auto a = CompareHandovers({{3.0, 0, 2, 0.9}}, sim, 1.0);
        NS_TEST_ASSERT_MSG_EQ(a.matched, 1u, "the completed handover pairs with the prediction");
        NS_TEST_ASSERT_MSG_LT(a.meanAbsOffsetS, 0.5,
                              "and completes within 0.5 s of the predicted instant (RACH over "
                              "a 600 km link)");
        Simulator::Destroy();
    }
};

class NtnDigitalTwinTestSuite : public TestSuite
{
  public:
    NtnDigitalTwinTestSuite()
        : TestSuite("ntn-digital-twin", Type::UNIT)
    {
        AddTestCase(new TwinAgreementMetricTestCase, TestCase::Duration::QUICK);
        AddTestCase(new TwinSimHandoverAgreementTestCase, TestCase::Duration::QUICK);
        AddTestCase(new TwinPredictionParserTestCase, TestCase::Duration::QUICK);
        AddTestCase(new TwinRunMetaParserTestCase, TestCase::Duration::QUICK);
        AddTestCase(new TwinTleLoaderTestCase, TestCase::Duration::QUICK);
        AddTestCase(new TwinCompareEdgeTestCase, TestCase::Duration::QUICK);
        AddTestCase(new TwinLinkBudgetConformanceTestCase, TestCase::Duration::QUICK);
        AddTestCase(new TwinA3SemanticsTestCase, TestCase::Duration::QUICK);
        AddTestCase(new TwinFixtureA3AgreementTestCase, TestCase::Duration::QUICK);
        AddTestCase(new TwinFixtureArgmaxSkyfieldTestCase, TestCase::Duration::QUICK);
        AddTestCase(new TwinReplayDeterminismTestCase, TestCase::Duration::QUICK);
        AddTestCase(new TwinReplayFailurePathsTestCase, TestCase::Duration::QUICK);
        AddTestCase(new TwinDrivesRealHandoverTestCase, TestCase::Duration::QUICK);
    }
};

static NtnDigitalTwinTestSuite g_ntnDigitalTwinTestSuite;
