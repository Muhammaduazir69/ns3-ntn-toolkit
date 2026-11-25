// Copyright (c) 2026 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only

#include "ns3/constant-position-mobility-model.h"
#include "ns3/constant-velocity-mobility-model.h"
#include "ns3/fast-fading-constant-position-mobility-model.h"
#include "ns3/hexagonal-grid-scenario-helper.h"
#include "ns3/mobility-model.h"
#include "ns3/resource-assignment-matrix.h"
#include "ns3/simulator.h"
#include "ns3/test.h"
#include "ns3/vector.h"

#include <cmath>

/**
 * @file nr-test-hexagonal-grid-scenario-helper.cc
 * @ingroup test
 *
 * @brief The test checks behaviour of hexagonal grid deployment helper.
 */
namespace ns3
{

/**
 * @brief TestCase for the hexagonal grid deployment
 */
class NrHexagonalGridDeploymentTestCase : public TestCase
{
  public:
    /**
     * @brief Create NrPatternTestCase
     */
    NrHexagonalGridDeploymentTestCase()
        : TestCase("NrHexagonalGridDeploymentTestCase")
    {
    }

  private:
    void DoRun() override;
};

void
NrHexagonalGridDeploymentTestCase::DoRun()
{
    auto getHelper = []() -> HexagonalGridScenarioHelper {
        ScenarioParameters scenarioParameters;
        scenarioParameters.m_isd = 200;
        scenarioParameters.m_bsHeight = 25;
        scenarioParameters.m_utHeight = 0;
        scenarioParameters.m_minBsUtDistance = 20;
        scenarioParameters.m_sectorization = HexagonalGridScenarioHelper::SINGLE;

        HexagonalGridScenarioHelper helper;
        helper.SetScenarioParameters(scenarioParameters);
        helper.SetNumRings(0);
        helper.SetUtNumber(10);
        return helper;
    };
    double indoorUeFraction = 0.5;

    // Set indoor and outdoor nodes with 0 speed should result in them being configured with
    // ConstantPositionMobilityModel
    Vector3D indoorSpeed = {0, 0, 0};
    Vector3D outdoorSpeed = {0, 0, 0};
    HexagonalGridScenarioHelper helper = getHelper();
    helper.CreateScenarioWithMobility(indoorSpeed, outdoorSpeed, indoorUeFraction);
    auto ue0Mm = helper.GetUserTerminals().Get(0)->GetObject<MobilityModel>();
    auto ue5Mm = helper.GetUserTerminals().Get(5)->GetObject<MobilityModel>();
    auto ue0MmTypeId = ue0Mm->GetInstanceTypeId();
    auto ue5MmTypeId = ue5Mm->GetInstanceTypeId();
    NS_TEST_ASSERT_MSG_EQ(ue0MmTypeId,
                          ue5MmTypeId,
                          "User terminals should have the same mobility model type");
    NS_TEST_ASSERT_MSG_EQ(ue0MmTypeId,
                          ConstantPositionMobilityModel::GetTypeId(),
                          "User terminal 0 should have the ConstantPositionMobilityModel type");
    NS_TEST_ASSERT_MSG_EQ(ue5MmTypeId,
                          ConstantPositionMobilityModel::GetTypeId(),
                          "User terminal 5 should have the ConstantPositionMobilityModel type");
    NS_TEST_ASSERT_MSG_EQ(ue0Mm->GetVelocity().GetLength(),
                          0,
                          "User terminal 0 should have zero velocity");
    NS_TEST_ASSERT_MSG_EQ(ue5Mm->GetVelocity().GetLength(),
                          0,
                          "User terminal 5 should have zero velocity");

    // Set indoor and outdoor nodes with 0 and 10m/s speed should result in indoor being configured
    // with ConstantPositionMobilityModel and outdoor as ConstantVelocityPositionModel
    indoorSpeed = {0, 0, 0};
    outdoorSpeed = {10, 0, 0};
    helper = getHelper();
    helper.CreateScenarioWithMobility(indoorSpeed, outdoorSpeed, indoorUeFraction);
    ue0Mm = helper.GetUserTerminals().Get(0)->GetObject<MobilityModel>();
    ue5Mm = helper.GetUserTerminals().Get(5)->GetObject<MobilityModel>();
    ue0MmTypeId = ue0Mm->GetInstanceTypeId();
    ue5MmTypeId = ue5Mm->GetInstanceTypeId();
    NS_TEST_ASSERT_MSG_EQ(ue0MmTypeId,
                          ue5MmTypeId,
                          "User terminals should have the same mobility model type");
    NS_TEST_ASSERT_MSG_EQ(ue0MmTypeId,
                          ConstantVelocityMobilityModel::GetTypeId(),
                          "User terminal 0 should have the ConstantVelocityMobilityModel type");
    NS_TEST_ASSERT_MSG_EQ(ue5MmTypeId,
                          ConstantVelocityMobilityModel::GetTypeId(),
                          "User terminal 5 should have the ConstantVelocityMobilityModel type");
    NS_TEST_ASSERT_MSG_EQ(ue0Mm->GetVelocity().GetLength(),
                          0,
                          "User terminal 0 should have zero velocity");
    NS_TEST_ASSERT_MSG_EQ(ue5Mm->GetVelocity().GetLength(),
                          10,
                          "User terminal 5 should have non-zero velocity");

    // Set indoor and outdoor nodes with 10 and 0m/s speed should result in outdoor being configured
    // with ConstantPositionMobilityModel and indoor as ConstantVelocityPositionModel
    indoorSpeed = {10, 0, 0};
    outdoorSpeed = {0, 0, 0};
    helper = getHelper();
    helper.CreateScenarioWithMobility(indoorSpeed, outdoorSpeed, indoorUeFraction);
    ue0Mm = helper.GetUserTerminals().Get(0)->GetObject<MobilityModel>();
    ue5Mm = helper.GetUserTerminals().Get(5)->GetObject<MobilityModel>();
    ue0MmTypeId = ue0Mm->GetInstanceTypeId();
    ue5MmTypeId = ue5Mm->GetInstanceTypeId();
    NS_TEST_ASSERT_MSG_EQ(ue0MmTypeId,
                          ue5MmTypeId,
                          "User terminals should have the same mobility model type");
    NS_TEST_ASSERT_MSG_EQ(ue0MmTypeId,
                          ConstantVelocityMobilityModel::GetTypeId(),
                          "User terminal 0 should have the ConstantVelocityMobilityModel type");
    NS_TEST_ASSERT_MSG_EQ(ue5MmTypeId,
                          ConstantVelocityMobilityModel::GetTypeId(),
                          "User terminal 5 should have the ConstantVelocityMobilityModel type");
    NS_TEST_ASSERT_MSG_EQ(ue0Mm->GetVelocity().GetLength(),
                          10,
                          "User terminal 0 should have non-zero velocity");
    NS_TEST_ASSERT_MSG_EQ(ue5Mm->GetVelocity().GetLength(),
                          0,
                          "User terminal 5 should have zero velocity");

    // Set indoor and outdoor nodes with 10 and 10m/s speed should result in outdoor and indoor
    // being configured as ConstantVelocityPositionModel
    indoorSpeed = {10, 0, 0};
    outdoorSpeed = {10, 0, 0};
    helper = getHelper();
    helper.CreateScenarioWithMobility(indoorSpeed, outdoorSpeed, indoorUeFraction);
    ue0Mm = helper.GetUserTerminals().Get(0)->GetObject<MobilityModel>();
    ue5Mm = helper.GetUserTerminals().Get(5)->GetObject<MobilityModel>();
    ue0MmTypeId = ue0Mm->GetInstanceTypeId();
    ue5MmTypeId = ue5Mm->GetInstanceTypeId();
    NS_TEST_ASSERT_MSG_EQ(ue0MmTypeId,
                          ue5MmTypeId,
                          "User terminals should have the same mobility model type");
    NS_TEST_ASSERT_MSG_EQ(ue0MmTypeId,
                          ConstantVelocityMobilityModel::GetTypeId(),
                          "User terminal 0 should have the ConstantVelocityMobilityModel type");
    NS_TEST_ASSERT_MSG_EQ(ue5MmTypeId,
                          ConstantVelocityMobilityModel::GetTypeId(),
                          "User terminal 5 should have the ConstantVelocityMobilityModel type");
    NS_TEST_ASSERT_MSG_EQ(ue0Mm->GetVelocity().GetLength(),
                          10,
                          "User terminal 0 should have non-zero velocity");
    NS_TEST_ASSERT_MSG_EQ(ue5Mm->GetVelocity().GetLength(),
                          10,
                          "User terminal 5 should have non-zero velocity");

    // Set indoor and outdoor nodes with 0 and 10m/s speed should result in outdoor and indoor
    // being configured with ns3::FastFadingConstantPositionMobilityModel
    indoorSpeed = {0, 0, 0};
    outdoorSpeed = {10, 0, 0};
    helper = getHelper();
    helper.CreateScenarioWithMobility(indoorSpeed,
                                      outdoorSpeed,
                                      indoorUeFraction,
                                      "ns3::FastFadingConstantPositionMobilityModel");
    ue0Mm = helper.GetUserTerminals().Get(0)->GetObject<MobilityModel>();
    ue5Mm = helper.GetUserTerminals().Get(5)->GetObject<MobilityModel>();
    ue0MmTypeId = ue0Mm->GetInstanceTypeId();
    ue5MmTypeId = ue5Mm->GetInstanceTypeId();
    NS_TEST_ASSERT_MSG_EQ(ue0MmTypeId,
                          ue5MmTypeId,
                          "User terminals should have the same mobility model type");
    NS_TEST_ASSERT_MSG_EQ(
        ue0MmTypeId,
        FastFadingConstantPositionMobilityModel::GetTypeId(),
        "User terminal 0 should have the FastFadingConstantPositionMobilityModel type");
    NS_TEST_ASSERT_MSG_EQ(
        ue5MmTypeId,
        FastFadingConstantPositionMobilityModel::GetTypeId(),
        "User terminal 5 should have the FastFadingConstantPositionMobilityModel type");
    NS_TEST_ASSERT_MSG_EQ(ue0Mm->GetVelocity().GetLength(),
                          0,
                          "User terminal 0 should have non-zero velocity");
    NS_TEST_ASSERT_MSG_EQ(ue5Mm->GetVelocity().GetLength(),
                          10,
                          "User terminal 5 should have non-zero velocity");

    // Set indoor and outdoor nodes with 10 and 10m/s speed should result in outdoor and indoor
    // being configured with ns3::FastFadingConstantPositionMobilityModel
    indoorSpeed = {10, 0, 0};
    outdoorSpeed = {10, 0, 0};
    helper = getHelper();
    helper.CreateScenarioWithMobility(indoorSpeed,
                                      outdoorSpeed,
                                      indoorUeFraction,
                                      "ns3::FastFadingConstantPositionMobilityModel");
    ue0Mm = helper.GetUserTerminals().Get(0)->GetObject<MobilityModel>();
    ue5Mm = helper.GetUserTerminals().Get(5)->GetObject<MobilityModel>();
    ue0MmTypeId = ue0Mm->GetInstanceTypeId();
    ue5MmTypeId = ue5Mm->GetInstanceTypeId();
    NS_TEST_ASSERT_MSG_EQ(ue0MmTypeId,
                          ue5MmTypeId,
                          "User terminals should have the same mobility model type");
    NS_TEST_ASSERT_MSG_EQ(
        ue0MmTypeId,
        FastFadingConstantPositionMobilityModel::GetTypeId(),
        "User terminal 0 should have the FastFadingConstantPositionMobilityModel type");
    NS_TEST_ASSERT_MSG_EQ(
        ue5MmTypeId,
        FastFadingConstantPositionMobilityModel::GetTypeId(),
        "User terminal 5 should have the FastFadingConstantPositionMobilityModel type");
    NS_TEST_ASSERT_MSG_EQ(ue0Mm->GetVelocity().GetLength(),
                          10,
                          "User terminal 0 should have non-zero velocity");
    NS_TEST_ASSERT_MSG_EQ(ue5Mm->GetVelocity().GetLength(),
                          10,
                          "User terminal 5 should have non-zero velocity");
    Simulator::Destroy();
}

/**
 * @brief TestCase for the UE drop of the hexagonal grid with 3 sectors per site: the UEs are
 * dropped uniformly in the hexagon of their cell (TR 38.901 Table 7.2-1), so about 12 % of them are
 * more than 45 degrees off the boresight of their sector, and none is closer than the minimum BS-UT
 * distance to its site. The indoor UEs are on integer floors: h = 3 (n_fl - 1) + 1.5 m, with n_fl
 * uniform in 1..N_fl and N_fl uniform in 4..8 (TR 38.901 Table 7.4.1-1).
 */
class NrHexagonalGridUeDropTestCase : public TestCase
{
  public:
    /**
     * @brief Create NrHexagonalGridUeDropTestCase
     */
    NrHexagonalGridUeDropTestCase()
        : TestCase("NrHexagonalGridUeDropTestCase")
    {
    }

  private:
    void DoRun() override;
};

void
NrHexagonalGridUeDropTestCase::DoRun()
{
    ScenarioParameters scenarioParameters;
    scenarioParameters.m_isd = 200;
    scenarioParameters.m_bsHeight = 25;
    scenarioParameters.m_utHeight = 1.5;
    scenarioParameters.m_minBsUtDistance = 10;
    scenarioParameters.m_antennaOffset = 1;
    scenarioParameters.m_sectorization = HexagonalGridScenarioHelper::TRIPLE;

    const uint32_t numCells = 21; // 1 ring: 7 sites of 3 sectors
    const uint32_t uesPerCell = 100;
    HexagonalGridScenarioHelper helper;
    helper.SetScenarioParameters(scenarioParameters);
    helper.SetNumRings(1);
    helper.SetUtNumber(numCells * uesPerCell);
    helper.AssignStreams(1);
    helper.CreateScenarioWithMobility(Vector(0, 0, 0), Vector(0, 0, 0), 0.8);

    const auto& gnbs = helper.GetBaseStations();
    const auto& ues = helper.GetUserTerminals();
    NS_TEST_ASSERT_MSG_EQ(gnbs.GetN(), numCells, "Unexpected number of cells");
    uint32_t offBoresight = 0;
    for (uint32_t i = 0; i < ues.GetN(); ++i)
    {
        // UE i is dropped in cell i % numCells, whose antenna is 1 m from the site along the
        // boresight of the sector (bearings 30, 150 and 270 degrees)
        const uint32_t cell = i % numCells;
        const double bearing = (30.0 + 120.0 * (cell % 3)) * M_PI / 180;
        const Vector gnb = gnbs.Get(cell)->GetObject<MobilityModel>()->GetPosition();
        const Vector ue = ues.Get(i)->GetObject<MobilityModel>()->GetPosition();
        const double dx = ue.x - (gnb.x - std::cos(bearing));
        const double dy = ue.y - (gnb.y - std::sin(bearing));
        NS_TEST_ASSERT_MSG_GT_OR_EQ(std::hypot(dx, dy),
                                    scenarioParameters.m_minBsUtDistance,
                                    "UE " << i
                                          << " is closer to its site than the minimum distance");
        const double off = std::remainder(std::atan2(dy, dx) - bearing, 2 * M_PI);
        offBoresight += std::abs(off) > M_PI / 4;
        const double floor = (ue.z - 1.5) / 3;
        NS_TEST_ASSERT_MSG_EQ_TOL(floor,
                                  std::round(floor),
                                  1e-9,
                                  "UE " << i << " at height " << ue.z << " is not on a floor");
        NS_TEST_ASSERT_MSG_LT_OR_EQ(floor, 7, "UE " << i << " is above the 8th floor");
    }
    // A uniform drop in the sector hexagon puts 12 % of the UEs more than 45 degrees off the
    // boresight; a drop in the disc inscribed in the hexagon puts none there
    const double fraction = static_cast<double>(offBoresight) / ues.GetN();
    NS_TEST_ASSERT_MSG_EQ_TOL(fraction,
                              0.12,
                              0.03,
                              "Unexpected share of UEs at the sector borders");
    Simulator::Destroy();
}

class NrHexagonalGridDeploymentTestSuite : public TestSuite
{
  public:
    NrHexagonalGridDeploymentTestSuite()
        : TestSuite("nr-hexagonal-deployment", Type::UNIT)
    {
        AddTestCase(new NrHexagonalGridDeploymentTestCase(), Duration::QUICK);
        AddTestCase(new NrHexagonalGridUeDropTestCase(), Duration::QUICK);
    }
};

static NrHexagonalGridDeploymentTestSuite g_nrHexagonalGridDeploymentTestSuite;

//!< Pattern test suite

} // namespace ns3
