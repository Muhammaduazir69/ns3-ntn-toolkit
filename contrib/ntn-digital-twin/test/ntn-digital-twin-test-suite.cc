/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#include "ns3/constant-position-mobility-model.h"
#include "ns3/log.h"
#include "ns3/ntn-twin-agreement.h"
#include "ns3/ntn-visibility-index.h"
#include "ns3/orbital-elements.h"
#include "ns3/sgp4-mobility-model.h"
#include "ns3/simulator.h"
#include "ns3/test.h"

#include <cmath>
#include <cstdlib>
#include <fstream>
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

/// TWIN-02: ns-3 propagates the twin's OWN orbits, and the two handover
/// sequences are compared.
///
/// Skipped unless a twin run has produced its artifacts (NTN_TWIN_ARTIFACT_DIR
/// pointing at a directory holding twin-orbits.tle and twin-pred.txt), because
/// generating them needs the Python side. When they are present this is the
/// number the old gate never computed.
class TwinSimHandoverAgreementTestCase : public TestCase
{
  public:
    TwinSimHandoverAgreementTestCase()
        : TestCase("TWIN-02: ns-3 replays the twin's orbits and the handovers are compared")
    {
    }

  private:
    std::vector<Ptr<ntncon::Sgp4MobilityModel>> m_sats;
    std::vector<TwinHandover> m_sim;
    Vector m_ue;
    int m_prev{-1};

    /// One argmax-elevation serving decision at the current simulated time.
    void Sample()
    {
        std::vector<Vector> pos;
        pos.reserve(m_sats.size());
        for (const auto& m : m_sats)
        {
            pos.push_back(m->GetPosition());
        }
        ntncon::NtnVisibilityIndex idx(10.0);
        idx.Build(pos);
        double el = 0.0;
        const std::size_t best = idx.BestElevation(m_ue, el);
        if (best == std::numeric_limits<std::size_t>::max() || el <= 0.0)
        {
            return;
        }
        const int chosen = static_cast<int>(best);
        if (chosen != m_prev)
        {
            // The twin numbers cells 1-based; match that.
            m_sim.push_back({Simulator::Now().GetSeconds(), 0,
                             static_cast<uint32_t>(chosen + 1), 1.0});
            m_prev = chosen;
        }
    }

    void DoRun() override
    {
        const std::string dir = ArtifactDir();
        if (dir.empty())
        {
            return; // no twin run available; the metric test above still runs
        }
        const auto tles = LoadTleDump(dir + "/twin-orbits.tle");
        const auto twin = LoadTwinPredictions(dir + "/twin-pred.txt");
        if (tles.empty() || twin.empty())
        {
            return;
        }

        // Load the twin's OWN elements into ns-3. This is only meaningful
        // because TWIN-01 made SetTle() select SGP4: before it, the same TLE
        // propagated as Kepler + J2 here and as SGP4 in the twin, so any
        // disagreement would have been dominated by geometry rather than by
        // the mobility decision.
        std::vector<Ptr<ntncon::Sgp4MobilityModel>> sats;
        sats.reserve(tles.size());
        for (const auto& t : tles)
        {
            ntncon::TleRecord rec;
            rec.name = t.name;
            rec.line1 = t.line1;
            rec.line2 = t.line2;
            Ptr<ntncon::Sgp4MobilityModel> m = CreateObject<ntncon::Sgp4MobilityModel>();
            if (m->SetTle(rec))
            {
                sats.push_back(m);
            }
        }
        NS_TEST_ASSERT_MSG_GT(sats.size(), 0u, "the twin's TLEs must load into ns-3");
        NS_TEST_ASSERT_MSG_EQ(sats[0]->IsUsingSgp4(), true,
                              "and must propagate with SGP4, not Kepler+J2 - otherwise this "
                              "compares two different orbits and measures nothing about the "
                              "mobility decision");
        NS_TEST_ASSERT_MSG_EQ(sats.size(), tles.size(),
                              "every TLE the twin propagated must load");

        // Replay the twin's OWN geometry, read from the prediction header
        // rather than assumed - if the two sides disagreed about where the
        // terminal is, the comparison would measure that instead.
        const auto meta = LoadTwinRunMeta(dir + "/twin-pred.txt");
        NS_TEST_ASSERT_MSG_EQ(meta.valid, true,
                              "the prediction file must carry the observer and window it was "
                              "generated with");

        const double latR = meta.observerLatDeg * M_PI / 180.0;
        const double lonR = meta.observerLonDeg * M_PI / 180.0;
        const double Re = 6378137.0 + meta.observerAltM;
        const Vector ue(Re * std::cos(latR) * std::cos(lonR),
                        Re * std::cos(latR) * std::sin(lonR),
                        Re * std::sin(latR));

        // Walk the window and record every change of the argmax-elevation
        // serving satellite. This is the ns-3 handover sequence - the quantity
        // the old gate never computed.
        // Sample inside the event loop: Sgp4MobilityModel propagates against
        // Simulator::Now(), so the window has to be walked by the scheduler
        // rather than by a bare loop.
        m_sim.clear();
        m_prev = -1;
        m_sats = sats;
        m_ue = ue;
        for (double t = 0.0; t <= meta.horizonS; t += meta.stepS)
        {
            Simulator::Schedule(Seconds(t), &TwinSimHandoverAgreementTestCase::Sample, this);
        }
        Simulator::Stop(Seconds(meta.horizonS + meta.stepS));
        Simulator::Run();
        const std::vector<TwinHandover> sim = m_sim;
        Simulator::Destroy();

        NS_TEST_ASSERT_MSG_GT(sim.size(), 0u,
                              "ns-3 must produce a handover sequence of its own over the twin's "
                              "window; without one there is nothing to compare and the gate is "
                              "back to comparing the twin with itself");

        const auto agree = CompareHandovers(twin, sim, /*windowS=*/2.0 * meta.stepS);
        std::cout << "  [TWIN-02] twin=" << agree.twinCount << " sim=" << agree.simCount
                  << " matched=" << agree.matched << " agreement="
                  << agree.agreement << " meanOffset=" << agree.meanAbsOffsetS
                  << " s maxOffset=" << agree.maxAbsOffsetS << " s\n";

        // The assertion is deliberately about the metric being COMPUTED and
        // meaningful, not about a target score. The twin applies an A3 guard in
        // dB with hysteresis; this replay is a bare argmax. They are different
        // policies over identical orbits, so a number below 1.0 is the honest
        // expected result - and it is the first time that number exists at all.
        NS_TEST_ASSERT_MSG_GT(agree.simCount, 0u, "a sim sequence exists");
        NS_TEST_ASSERT_MSG_LT(agree.agreement, 1.0001, "agreement is a fraction");
        NS_TEST_ASSERT_MSG_GT(agree.agreement, -0.0001, "and non-negative");
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
    }
};

static NtnDigitalTwinTestSuite g_ntnDigitalTwinTestSuite;
