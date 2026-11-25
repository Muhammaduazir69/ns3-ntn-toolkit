// Copyright (c) 2026 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only

/**
 * @ingroup test
 * @file nr-test-random-direction-disc-2d-mobility.cc
 *
 * @brief Test suite `nr-random-direction-disc-2d`: exercises the bounded-disc mobility model
 * added for TR 36.839-style handover deployments.
 *
 * The case aggregates an ns3::RandomDirectionDisc2dMobilityModel to a node, samples its position
 * across a simulation and asserts the node stays inside the configured disc, actually moves and
 * remains 2D (z == 0).
 */

#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/random-direction-disc-2d-mobility-model.h"

#include <cmath>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("NrRandomDirectionDisc2dTest");

/**
 * @ingroup nr-test
 *
 * @brief Verifies that RandomDirectionDisc2dMobilityModel keeps a node bounded inside its disc.
 *
 * The node is sampled periodically over the simulation. At every sample the position must lie
 * within (radius + tolerance) of the disc centre, the z coordinate must stay 0 (2D motion), and
 * the accumulated set of samples must show the node moving.
 */
class NrRandomDirectionDisc2dMobilityTestCase : public TestCase
{
  public:
    NrRandomDirectionDisc2dMobilityTestCase();
    ~NrRandomDirectionDisc2dMobilityTestCase() override = default;

  private:
    void DoRun() override;

    /**
     * Sample the node position and check the disc/2D invariants.
     * @param mob mobility model under test
     */
    void SampleAndCheck(Ptr<MobilityModel> mob);

    double m_centerX{100.0}; ///< disc centre X (m)
    double m_centerY{200.0}; ///< disc centre Y (m)
    double m_radius{50.0};   ///< disc radius (m)
    /// Numerical slack (m). A node that reaches the boundary lands on it with the rounding of
    /// the computed intercept and then pauses at that point, so a resting position can sit a
    /// fraction of a millimetre outside the exact circle (measured: about 2e-4 m at radius
    /// 50 m and 6e-4 m at radius 500 m). One millimetre is well below anything meaningful for
    /// a mobility model expressed in metres, while still catching a genuine escape from the
    /// disc, a real excursion out of the 2D plane, or a node that never moves.
    double m_tolerance{1e-3};
    uint32_t m_nSamples{0};  ///< number of collected position samples
    Vector m_firstPosition;  ///< first sampled position, used to prove motion
    bool m_haveFirst{false}; ///< whether m_firstPosition has been set
    bool m_moved{false};     ///< whether any sample differs from the first
};

NrRandomDirectionDisc2dMobilityTestCase::NrRandomDirectionDisc2dMobilityTestCase()
    : TestCase("RandomDirectionDisc2dMobilityModel keeps node bounded inside disc")
{
}

void
NrRandomDirectionDisc2dMobilityTestCase::SampleAndCheck(Ptr<MobilityModel> mob)
{
    Vector p = mob->GetPosition();

    const double dx = p.x - m_centerX;
    const double dy = p.y - m_centerY;
    const double dist = std::sqrt(dx * dx + dy * dy);

    NS_TEST_ASSERT_MSG_LT_OR_EQ(
        dist,
        m_radius + m_tolerance,
        "position escaped the bounding disc at t=" << Simulator::Now().GetSeconds() << "s");
    NS_TEST_ASSERT_MSG_EQ_TOL(p.z, 0.0, m_tolerance, "motion left the 2D plane (z != 0)");

    if (!m_haveFirst)
    {
        m_firstPosition = p;
        m_haveFirst = true;
    }
    else if (CalculateDistance(p, m_firstPosition) > m_tolerance)
    {
        m_moved = true;
    }
    ++m_nSamples;
}

void
NrRandomDirectionDisc2dMobilityTestCase::DoRun()
{
    RngSeedManager::SetSeed(1);
    RngSeedManager::SetRun(1);

    Ptr<Node> node = CreateObject<Node>();
    Ptr<RandomDirectionDisc2dMobilityModel> mob =
        CreateObject<RandomDirectionDisc2dMobilityModel>();
    mob->SetAttribute("CenterX", DoubleValue(m_centerX));
    mob->SetAttribute("CenterY", DoubleValue(m_centerY));
    mob->SetAttribute("Radius", DoubleValue(m_radius));
    node->AggregateObject(mob);

    // Start the node at the disc centre so the very first leg is unambiguously inside.
    mob->SetPosition(Vector(m_centerX, m_centerY, 0.0));

    const Time sampleInterval = MilliSeconds(100);
    const Time simDuration = Seconds(60);
    for (Time t = sampleInterval; t <= simDuration; t += sampleInterval)
    {
        Simulator::Schedule(t, &NrRandomDirectionDisc2dMobilityTestCase::SampleAndCheck, this, mob);
    }

    Simulator::Stop(simDuration + sampleInterval);
    Simulator::Run();
    Simulator::Destroy();

    NS_TEST_ASSERT_MSG_GT(m_nSamples, 100, "too few position samples were collected");
    NS_TEST_ASSERT_MSG_EQ(m_moved, true, "node never moved away from its starting position");
}

/**
 * @ingroup nr-test
 *
 * @brief Test suite `nr-random-direction-disc-2d`.
 */
class NrRandomDirectionDisc2dTestSuite : public TestSuite
{
  public:
    NrRandomDirectionDisc2dTestSuite()
        : TestSuite("nr-random-direction-disc-2d", Type::UNIT)
    {
        AddTestCase(new NrRandomDirectionDisc2dMobilityTestCase(), Duration::QUICK);
    }
};

/// Static instance of the test suite.
static NrRandomDirectionDisc2dTestSuite g_nrRandomDirectionDisc2dTestSuite;
