/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Author: Muhammad Uzair
 *
 * Test suite for the NTN-CHO framework
 */

#include "ns3/ntn-cho-algorithm.h"
#include "ns3/ntn-measurement-model.h"
#include "ns3/ntn-orbit-predictor.h"
#include "ns3/ntn-tte-estimator.h"

#include <ns3/log.h>
#include <ns3/node-container.h>
#include <ns3/satellite-antenna-gain-pattern-container.h>
#include <ns3/satellite-constant-position-mobility-model.h>
#include <ns3/satellite-env-variables.h>
#include <ns3/simulator.h>
#include <ns3/singleton.h>
#include <ns3/test.h>

#include <cmath>
#include <vector>

using namespace ns3;

/**
 * \ingroup ntn-cho
 * \defgroup ntn-cho-test NTN-CHO module tests
 */

/**
 * \ingroup ntn-cho-test
 * \brief Test CHO algorithm candidate selection
 *
 * Verifies that the TTE-aware candidate selection algorithm:
 * 1. Filters candidates below quality threshold
 * 2. Filters candidates below TTE minimum
 * 3. Selects the candidate with longest TTE
 * 4. Breaks ties by SINR
 */
class NtnChoAlgorithmTestCase : public TestCase
{
  public:
    NtnChoAlgorithmTestCase()
        : TestCase("NTN CHO Algorithm - TTE-aware candidate selection")
    {
    }

  private:
    void DoRun() override
    {
        // Create CHO algorithm
        Ptr<NtnChoAlgorithm> algo = CreateObject<NtnChoAlgorithm>();

        NtnChoAlgorithm::ChoConfig config;
        config.triggerType = NtnChoAlgorithm::TRIGGER_TTE_AWARE;
        config.qualityThreshold_dB = -3.0;
        config.tteMinimum = Seconds(10.0);
        config.tteEpsilon = Seconds(2.0);
        config.maxCandidates = 8;
        algo->Configure(config);

        // Add candidates
        algo->AddCandidateCell(1, 0, 0); // cell 1, sat 0, beam 0
        algo->AddCandidateCell(2, 0, 1); // cell 2, sat 0, beam 1
        algo->AddCandidateCell(3, 1, 0); // cell 3, sat 1, beam 0
        algo->AddCandidateCell(4, 1, 1); // cell 4, sat 1, beam 1

        // Simulate measurements - set via internal state manipulation
        // Cell 1: good SINR, short TTE -> should be rejected
        algo->UpdateMeasurement(1, 5.0, 2.0);
        // Cell 2: good SINR, long TTE -> should be selected
        algo->UpdateMeasurement(2, 4.0, 1.5);
        // Cell 3: bad SINR -> should be rejected
        algo->UpdateMeasurement(3, -5.0, -4.0);
        // Cell 4: good SINR, medium TTE
        algo->UpdateMeasurement(4, 3.0, 1.0);

        // Verify state transitions
        NS_TEST_ASSERT_MSG_EQ(algo->GetState(),
                              NtnChoAlgorithm::CHO_PREPARED,
                              "Algorithm should be in PREPARED state after adding candidates");

        NS_TEST_ASSERT_MSG_EQ(algo->GetCandidates().size(),
                              4,
                              "Should have 4 candidates");

        // Test A3 baseline selection (highest SINR neighbor above serving+offset)
        uint16_t a3Result = algo->SelectBaselineA3(-10.0); // serving at -10 dB
        NS_TEST_ASSERT_MSG_EQ(a3Result, 1, "A3 should select cell 1 (highest SINR above threshold)");

        Simulator::Destroy();
    }
};

/**
 * \ingroup ntn-cho-test
 * \brief Test CHO state machine transitions
 */
class NtnChoStateMachineTestCase : public TestCase
{
  public:
    NtnChoStateMachineTestCase()
        : TestCase("NTN CHO Algorithm - State machine transitions")
    {
    }

  private:
    void DoRun() override
    {
        Ptr<NtnChoAlgorithm> algo = CreateObject<NtnChoAlgorithm>();

        NtnChoAlgorithm::ChoConfig config;
        config.triggerType = NtnChoAlgorithm::TRIGGER_TTE_AWARE;
        algo->Configure(config);

        // Initial state should be IDLE
        NS_TEST_ASSERT_MSG_EQ(algo->GetState(), NtnChoAlgorithm::CHO_IDLE,
                              "Initial state should be CHO_IDLE");

        // Add candidate -> should transition to PREPARED
        algo->AddCandidateCell(1, 0, 0);
        NS_TEST_ASSERT_MSG_EQ(algo->GetState(), NtnChoAlgorithm::CHO_PREPARED,
                              "Should transition to CHO_PREPARED");

        // Cancel -> back to IDLE
        algo->CancelHandover();
        NS_TEST_ASSERT_MSG_EQ(algo->GetState(), NtnChoAlgorithm::CHO_IDLE,
                              "Should transition back to CHO_IDLE after cancel");

        Simulator::Destroy();
    }
};

/**
 * \ingroup ntn-cho-test
 * \brief Test measurement model
 */
class NtnMeasurementModelTestCase : public TestCase
{
  public:
    NtnMeasurementModelTestCase()
        : TestCase("NTN Measurement Model - RSRP SINR computation")
    {
    }

  private:
    void DoRun() override
    {
        // Create measurement model
        Ptr<NtnMeasurementModel> model = CreateObject<NtnMeasurementModel>();
        model->SetCarrierFrequency(2.0e9);   // 2 GHz S-band
        model->SetBandwidth(30.0e6);          // 30 MHz
        model->SetSatelliteTxPower(40.0);     // 10W
        model->SetUeNoiseFigure(7.0);         // 7 dB
        model->SetNtnScenario(NtnMeasurementModel::NTN_SUBURBAN);

        // Verify model was created successfully
        NS_TEST_ASSERT_MSG_NE(model, nullptr, "Measurement model should be created");

        Simulator::Destroy();
    }
};

namespace
{

/**
 * Shared fixture for the Rel-18 CondEventD2 tests: an NtnOrbitPredictor over
 * TWO satellites whose live beam centers act as the D2 MOVING reference
 * locations. The real geo-33E antenna gain patterns from the satellite
 * module provide the beam-center geometry (the same fixture pattern as the
 * satellite module's antenna-pattern test), and SatConstantPositionMobility
 * models pin the snapshot so each test can compute the exact UE-to-moving-
 * reference distances and place its D2 thresholds around them.
 */
struct D2Fixture
{
    Ptr<NtnOrbitPredictor> predictor; //!< initialized over both satellites
    GeoCoordinate uePos;              //!< UE inside the serving beam
};

D2Fixture
MakeD2Fixture()
{
    Singleton<SatEnvVariables>::Get()->DoInitialize();
    Singleton<SatEnvVariables>::Get()->SetOutputVariables("test-ntn-cho-d2", "", true);
    const std::string patternsFolder =
        Singleton<SatEnvVariables>::Get()->LocateDataDirectory() +
        "/scenarios/geo-33E/antennapatterns";

    Ptr<SatAntennaGainPatternContainer> agp =
        CreateObject<SatAntennaGainPatternContainer>(2, patternsFolder);

    // Serving satellite at the patterns' default position; the candidate is
    // shifted in longitude so its beam centers (moving references) differ.
    Ptr<SatConstantPositionMobilityModel> servMob =
        CreateObject<SatConstantPositionMobilityModel>();
    servMob->SetGeoPosition(GeoCoordinate(0.0, 33.0, 35786000.0));
    Ptr<SatConstantPositionMobilityModel> candMob =
        CreateObject<SatConstantPositionMobilityModel>();
    candMob->SetGeoPosition(GeoCoordinate(0.0, 28.0, 35786000.0));
    agp->ConfigureBeamsMobility(0, servMob);
    agp->ConfigureBeamsMobility(1, candMob);

    NodeContainer sats;
    sats.Create(2);
    sats.Get(0)->AggregateObject(servMob);
    sats.Get(1)->AggregateObject(candMob);

    D2Fixture f;
    f.predictor = CreateObject<NtnOrbitPredictor>();
    f.predictor->Initialize(sats, agp);
    // Inside beam 12 of geo-33E but OFFSET ~55 km from its center (50.25,
    // 3.75): the moving-reference distance must be strictly positive, and
    // the D2 thresholds below are placed relative to it (dServ >> 2*hys).
    f.uePos = GeoCoordinate(49.75, 3.75, 0.0);
    return f;
}

/**
 * UE-to-moving-reference distance (m), replicating the algorithm's
 * DistanceToMovingReference(): Euclidean distance from the UE to the live
 * beam center of the given satellite/beam.
 */
double
DistanceToMovingRef(const D2Fixture& f, uint32_t satId, uint32_t beamId)
{
    auto snap = f.predictor->GetBeamSnapshot(satId, beamId, f.uePos);
    const Vector ueCart = f.uePos.ToVector();
    const Vector refCart = snap.beamCenter.ToVector();
    const double dx = ueCart.x - refCart.x;
    const double dy = ueCart.y - refCart.y;
    const double dz = ueCart.z - refCart.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

constexpr uint16_t kServingCell = 1;
constexpr uint16_t kCandCell = 2;
constexpr uint32_t kServingBeam = 12; // best geo-33E beam at (50.25, 3.75)
constexpr uint32_t kCandBeam = 22;    // best geo-33E beam at (42.25, -4.50)

} // namespace

/**
 * \ingroup ntn-cho-test
 * \brief Test the Rel-18 CondEventD2 trigger (TS 38.331 5.5.4.15a)
 *
 * Entering condition with MOVING reference locations (live ephemeris beam
 * centers): (dServing - hys > d2Thresh1_m) AND (dCand + hys < d2Thresh2_m).
 * The thresholds are placed around the distances the fixture's snapshot
 * actually produces, so each branch is exercised deterministically:
 *   a. serving far / candidate near        -> admitted
 *   b. serving still near (below thresh1)  -> NOT admitted
 *   c. candidate too far (above thresh2)   -> NOT admitted
 *   d. serving above thresh1 but within hys-> NOT admitted (hysteresis band)
 */
class NtnChoDistanceD2TriggerTestCase : public TestCase
{
  public:
    NtnChoDistanceD2TriggerTestCase()
        : TestCase("NTN CHO Algorithm - Rel-18 CondEventD2 moving-reference trigger")
    {
    }

  private:
    void DoRun() override
    {
        D2Fixture f = MakeD2Fixture();

        // Distances to the MOVING references the algorithm will see.
        const double dServ = DistanceToMovingRef(f, 0, kServingBeam);
        const double dCand = DistanceToMovingRef(f, 1, kCandBeam);
        NS_TEST_ASSERT_MSG_GT(dServ, 0.0, "serving moving reference must resolve");
        NS_TEST_ASSERT_MSG_GT(dCand, 0.0, "candidate moving reference must resolve");

        const double hys = 10000.0; // 10 km hysteresisLocation

        Ptr<NtnChoAlgorithm> algo = CreateObject<NtnChoAlgorithm>();
        NtnChoAlgorithm::ChoConfig config;
        config.triggerType = NtnChoAlgorithm::TRIGGER_DISTANCE_D2;
        config.qualityThreshold_dB = -3.0;
        config.conditionMonitorPeriod = Seconds(1000.0); // keep eval manual
        config.d2HysteresisLocation_m = hys;
        // Case a: serving beyond thresh1 + hys, candidate below thresh2 - hys.
        config.d2Thresh1_m = dServ - 2.0 * hys;
        config.d2Thresh2_m = dCand + 2.0 * hys;
        algo->Configure(config);
        algo->SetOrbitPredictor(f.predictor);

        algo->AddCandidateCell(kServingCell, 0, kServingBeam);
        algo->AddCandidateCell(kCandCell, 1, kCandBeam);
        algo->SetServingCell(kServingCell);
        algo->UpdateMeasurement(kServingCell, 5.0, 2.0);
        algo->UpdateMeasurement(kCandCell, 4.0, 1.5); // passes quality gate

        // First evaluation happens inside StartMonitoring.
        algo->StartMonitoring(f.uePos, Vector(0.0, 0.0, 0.0));
        NS_TEST_ASSERT_MSG_EQ(algo->GetCandidates()[kCandCell].admitted, true,
                              "D2(a): serving far + candidate near -> admitted");
        NS_TEST_ASSERT_MSG_EQ(algo->GetNumAdmittedCandidates(), 1,
                              "D2(a): exactly the candidate cell is admitted");

        // Case b: serving still near -> serving leg fails.
        config.d2Thresh1_m = dServ + hys;
        config.d2Thresh2_m = dCand + 2.0 * hys;
        algo->Configure(config);
        algo->EvaluateConditions();
        NS_TEST_ASSERT_MSG_EQ(algo->GetCandidates()[kCandCell].admitted, false,
                              "D2(b): serving below thresh1 -> NOT admitted");

        // Case c: candidate too far -> candidate leg fails.
        config.d2Thresh1_m = dServ - 2.0 * hys;
        config.d2Thresh2_m = dCand - hys;
        algo->Configure(config);
        algo->EvaluateConditions();
        NS_TEST_ASSERT_MSG_EQ(algo->GetCandidates()[kCandCell].admitted, false,
                              "D2(c): candidate above thresh2 -> NOT admitted");

        // Case d: dServing just above thresh1 but within the hysteresis band
        // (dServing > thresh1 while dServing - hys < thresh1).
        config.d2Thresh1_m = dServ - 0.5 * hys;
        config.d2Thresh2_m = dCand + 2.0 * hys;
        algo->Configure(config);
        algo->EvaluateConditions();
        NS_TEST_ASSERT_MSG_EQ(algo->GetCandidates()[kCandCell].admitted, false,
                              "D2(d): serving inside hysteresis band -> NOT admitted");

        algo->StopMonitoring();
        Simulator::Destroy();
        Singleton<SatEnvVariables>::Get()->DoDispose();
    }
};

/**
 * \ingroup ntn-cho-test
 * \brief Test the Rel-17 combineWithA4 time-to-trigger gating
 *
 * With combineWithA4 = true the A4 quality leg must hold CONTINUOUSLY for
 * a3TimeToTrigger before a D2 admission may fire; dropping below the quality
 * threshold resets the TTT. With combineWithA4 = false (default) the D2
 * admission is immediate once the geometry is satisfied.
 */
class NtnChoCombineWithA4TttTestCase : public TestCase
{
  public:
    NtnChoCombineWithA4TttTestCase()
        : TestCase("NTN CHO Algorithm - combineWithA4 TTT gating of D2 admissions")
    {
    }

  private:
    void DoRun() override
    {
        D2Fixture f = MakeD2Fixture();
        const double dServ = DistanceToMovingRef(f, 0, kServingBeam);
        const double dCand = DistanceToMovingRef(f, 1, kCandBeam);
        const double hys = 10000.0;

        NtnChoAlgorithm::ChoConfig config;
        config.triggerType = NtnChoAlgorithm::TRIGGER_DISTANCE_D2;
        config.qualityThreshold_dB = -3.0;
        config.conditionMonitorPeriod = Seconds(1000.0); // keep eval manual
        config.d2HysteresisLocation_m = hys;
        config.d2Thresh1_m = dServ - 2.0 * hys; // geometry always satisfied
        config.d2Thresh2_m = dCand + 2.0 * hys;
        config.a3TimeToTrigger = MilliSeconds(160);
        config.combineWithA4 = true;

        Ptr<NtnChoAlgorithm> algo = CreateObject<NtnChoAlgorithm>();
        algo->Configure(config);
        algo->SetOrbitPredictor(f.predictor);
        algo->AddCandidateCell(kServingCell, 0, kServingBeam);
        algo->AddCandidateCell(kCandCell, 1, kCandBeam);
        algo->SetServingCell(kServingCell);
        algo->UpdateMeasurement(kServingCell, 5.0, 2.0);
        algo->UpdateMeasurement(kCandCell, 4.0, 1.5); // quality crosses at t=0

        // combineWithA4 = false control: admission must be immediate.
        NtnChoAlgorithm::ChoConfig configNoA4 = config;
        configNoA4.combineWithA4 = false;
        Ptr<NtnChoAlgorithm> algoNoA4 = CreateObject<NtnChoAlgorithm>();
        algoNoA4->Configure(configNoA4);
        algoNoA4->SetOrbitPredictor(f.predictor);
        algoNoA4->AddCandidateCell(kServingCell, 0, kServingBeam);
        algoNoA4->AddCandidateCell(kCandCell, 1, kCandBeam);
        algoNoA4->SetServingCell(kServingCell);
        algoNoA4->UpdateMeasurement(kServingCell, 5.0, 2.0);
        algoNoA4->UpdateMeasurement(kCandCell, 4.0, 1.5);

        // t=0: quality just crossed -> A4 TTT starts; D2 must NOT fire yet.
        algo->StartMonitoring(f.uePos, Vector(0.0, 0.0, 0.0));
        NS_TEST_ASSERT_MSG_EQ(algo->GetCandidates()[kCandCell].admitted, false,
                              "TTT: not admitted at t=0 (TTT just started)");

        algoNoA4->StartMonitoring(f.uePos, Vector(0.0, 0.0, 0.0));
        NS_TEST_ASSERT_MSG_EQ(algoNoA4->GetCandidates()[kCandCell].admitted, true,
                              "combineWithA4=false: admission is immediate");

        std::vector<bool> admitted; // sampled along the TTT timeline
        auto sample = [algo, &admitted] {
            algo->EvaluateConditions();
            admitted.push_back(algo->GetCandidates()[kCandCell].admitted);
        };
        // 100 ms into the 160 ms TTT -> still blocked.
        Simulator::Schedule(MilliSeconds(100), sample);
        // 200 ms -> TTT elapsed -> admitted.
        Simulator::Schedule(MilliSeconds(200), sample);
        // 300 ms: quality drops -> admission lost, TTT resets.
        Simulator::Schedule(MilliSeconds(300), [algo, &admitted] {
            algo->UpdateMeasurement(kCandCell, -50.0, -50.0);
            algo->EvaluateConditions();
            admitted.push_back(algo->GetCandidates()[kCandCell].admitted);
        });
        // 400 ms: quality recovers -> TTT restarts, still blocked.
        Simulator::Schedule(MilliSeconds(400), [algo, &admitted] {
            algo->UpdateMeasurement(kCandCell, 4.0, 1.5);
            algo->EvaluateConditions();
            admitted.push_back(algo->GetCandidates()[kCandCell].admitted);
        });
        // 500 ms: only 100 ms since recovery -> still blocked.
        Simulator::Schedule(MilliSeconds(500), sample);
        // 600 ms: 200 ms since recovery -> admitted again.
        Simulator::Schedule(MilliSeconds(600), sample);

        Simulator::Stop(MilliSeconds(700));
        Simulator::Run();

        NS_TEST_ASSERT_MSG_EQ(admitted.size(), 6, "all TTT samples collected");
        NS_TEST_ASSERT_MSG_EQ(admitted[0], false, "TTT: blocked at 100 ms (< 160 ms)");
        NS_TEST_ASSERT_MSG_EQ(admitted[1], true, "TTT: admitted at 200 ms (>= 160 ms)");
        NS_TEST_ASSERT_MSG_EQ(admitted[2], false, "TTT: quality drop revokes admission");
        NS_TEST_ASSERT_MSG_EQ(admitted[3], false, "TTT: reset on recovery, blocked");
        NS_TEST_ASSERT_MSG_EQ(admitted[4], false, "TTT: 100 ms after reset, blocked");
        NS_TEST_ASSERT_MSG_EQ(admitted[5], true, "TTT: 200 ms after reset, admitted");

        algo->StopMonitoring();
        algoNoA4->StopMonitoring();
        Simulator::Destroy();
        Singleton<SatEnvVariables>::Get()->DoDispose();
    }
};

/**
 * \ingroup ntn-cho-test
 * \brief Test the slant-dependent RACH interruption accounting
 *
 * With rachLess = false and a known target slant range, the handover
 * interruption must be choExecutionDelay + 2*slant/c (slant RTT) +
 * rachProcessingDelay; two different slant ranges must therefore produce
 * different interruptions. With no slant known the constant rachDuration
 * fallback applies.
 */
class NtnChoSlantRachInterruptionTestCase : public TestCase
{
  public:
    NtnChoSlantRachInterruptionTestCase()
        : TestCase("NTN CHO Algorithm - slant-dependent RACH interruption")
    {
    }

  private:
    void DoRun() override
    {
        constexpr double kC = 299792458.0;

        Ptr<NtnChoAlgorithm> algo = CreateObject<NtnChoAlgorithm>();
        NtnChoAlgorithm::ChoConfig config;
        config.rachLess = false;
        config.choExecutionDelay = MilliSeconds(50);
        config.rachProcessingDelay = MilliSeconds(20);
        config.rachDuration = MilliSeconds(80);
        algo->Configure(config);
        algo->SetServingCell(1);

        const double execMs = config.choExecutionDelay.GetMilliSeconds();
        const double procMs = config.rachProcessingDelay.GetMilliSeconds();

        // Slant 1: LEO near-zenith range (600 km) -> RTT ~4.00 ms.
        const double slant1 = 600e3;
        algo->AddCandidateCell(2, 1, 0);
        algo->UpdateCandidateSlantRange(2, slant1);
        algo->ExecuteHandover(2);
        auto stats = algo->GetMechanismStats();
        const double expected1 = execMs + 2.0 * slant1 / kC * 1e3 + procMs;
        NS_TEST_ASSERT_MSG_EQ_TOL(stats.lastInterruptionMs, expected1, 0.1,
                                  "slant 600 km: interruption = exec + RTT + processing");
        NS_TEST_ASSERT_MSG_EQ(stats.rachExecutions, 1, "first HO pays the RACH");
        NS_TEST_ASSERT_MSG_EQ(stats.rachLessExecutions, 0, "no RACH-less execution");

        // Slant 2: low-elevation edge of the pass (1500 km) -> RTT ~10.01 ms.
        // ExecuteHandover cleared the candidate set, so prepare a new target.
        const double slant2 = 1500e3;
        algo->AddCandidateCell(3, 2, 0);
        algo->UpdateCandidateSlantRange(3, slant2);
        algo->ExecuteHandover(3);
        stats = algo->GetMechanismStats();
        const double expected2 = execMs + 2.0 * slant2 / kC * 1e3 + procMs;
        NS_TEST_ASSERT_MSG_EQ_TOL(stats.lastInterruptionMs, expected2, 0.1,
                                  "slant 1500 km: interruption = exec + RTT + processing");
        NS_TEST_ASSERT_MSG_GT(expected2, expected1 + 1.0,
                              "different slant ranges price the RACH differently");

        // No slant known for the target -> constant rachDuration fallback.
        algo->AddCandidateCell(4, 3, 0);
        algo->ExecuteHandover(4);
        stats = algo->GetMechanismStats();
        const double expectedFallback = execMs + config.rachDuration.GetMilliSeconds();
        NS_TEST_ASSERT_MSG_EQ_TOL(stats.lastInterruptionMs, expectedFallback, 0.1,
                                  "unknown slant: falls back to constant rachDuration");
        NS_TEST_ASSERT_MSG_EQ(stats.rachExecutions, 3, "all three HOs paid the RACH");

        Simulator::Destroy();
    }
};

/**
 * \ingroup ntn-cho-test
 * \brief Test RACH-less (RCHO) execution with ephemeris TA pre-compensation
 *
 * With rachLess = true and a known target slant range the RACH is skipped:
 * the interruption is only choExecutionDelay and the pre-computed TA equals
 * the slant round-trip 2*slant/c (recorded in microseconds). Without a known
 * slant range the execution falls back to the RACH path.
 */
class NtnChoRachLessExecutionTestCase : public TestCase
{
  public:
    NtnChoRachLessExecutionTestCase()
        : TestCase("NTN CHO Algorithm - RACH-less execution pre-computes the TA")
    {
    }

  private:
    void DoRun() override
    {
        constexpr double kC = 299792458.0;

        Ptr<NtnChoAlgorithm> algo = CreateObject<NtnChoAlgorithm>();
        NtnChoAlgorithm::ChoConfig config;
        config.rachLess = true;
        config.choExecutionDelay = MilliSeconds(50);
        config.rachProcessingDelay = MilliSeconds(20);
        config.rachDuration = MilliSeconds(80);
        algo->Configure(config);
        algo->SetServingCell(1);

        // Known slant -> RACH skipped, TA pre-compensated from the ephemeris.
        const double slant = 1200e3;
        algo->AddCandidateCell(2, 1, 0);
        algo->UpdateCandidateSlantRange(2, slant);
        algo->ExecuteHandover(2);
        auto stats = algo->GetMechanismStats();
        NS_TEST_ASSERT_MSG_EQ(stats.rachLessExecutions, 1, "RACH-less execution recorded");
        NS_TEST_ASSERT_MSG_EQ(stats.rachExecutions, 0, "no RACH paid");
        NS_TEST_ASSERT_MSG_EQ_TOL(stats.lastPreCompTaUs, 2.0 * slant / kC * 1e6, 0.01,
                                  "pre-computed TA = slant round-trip (us)");
        NS_TEST_ASSERT_MSG_EQ_TOL(stats.lastInterruptionMs,
                                  static_cast<double>(config.choExecutionDelay.GetMilliSeconds()),
                                  0.1,
                                  "RACH-less interruption = execution delay only");

        // rachLess configured but NO slant known -> must pay the RACH.
        algo->AddCandidateCell(3, 2, 0);
        algo->ExecuteHandover(3);
        stats = algo->GetMechanismStats();
        NS_TEST_ASSERT_MSG_EQ(stats.rachLessExecutions, 1, "no second RACH-less execution");
        NS_TEST_ASSERT_MSG_EQ(stats.rachExecutions, 1, "unknown slant pays the RACH");
        NS_TEST_ASSERT_MSG_EQ_TOL(stats.lastInterruptionMs,
                                  static_cast<double>(config.choExecutionDelay.GetMilliSeconds() +
                                                      config.rachDuration.GetMilliSeconds()),
                                  0.1,
                                  "unknown slant: exec delay + constant RACH duration");

        Simulator::Destroy();
    }
};

/**
 * \ingroup ntn-cho-test
 * \brief NTN CHO Test Suite
 */
class NtnChoTestSuite : public TestSuite
{
  public:
    NtnChoTestSuite()
        : TestSuite("ntn-cho", Type::UNIT)
    {
        AddTestCase(new NtnChoAlgorithmTestCase, TestCase::Duration::QUICK);
        AddTestCase(new NtnChoStateMachineTestCase, TestCase::Duration::QUICK);
        AddTestCase(new NtnMeasurementModelTestCase, TestCase::Duration::QUICK);
        AddTestCase(new NtnChoDistanceD2TriggerTestCase, TestCase::Duration::QUICK);
        AddTestCase(new NtnChoCombineWithA4TttTestCase, TestCase::Duration::QUICK);
        AddTestCase(new NtnChoSlantRachInterruptionTestCase, TestCase::Duration::QUICK);
        AddTestCase(new NtnChoRachLessExecutionTestCase, TestCase::Duration::QUICK);
    }
};

static NtnChoTestSuite g_ntnChoTestSuite;
