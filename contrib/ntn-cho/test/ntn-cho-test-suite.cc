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
#include "ns3/ntn-cho-helper.h"
#include "ns3/ntn-geodetic-fixed-mobility-model.h"
#include "ns3/ntn-measurement-model.h"
#include "ns3/ntn-orbit-predictor.h"
#include "ns3/ntn-rach-window.h"
#include "ns3/ntn-tr38811-mobility-model.h"
#include "ns3/ntn-tte-estimator.h"

#include <filesystem>
#include <fstream>
#include <ns3/log.h>
#include <ns3/constant-velocity-mobility-model.h>
#include <ns3/node-container.h>
#include <ns3/satellite-antenna-gain-pattern-container.h>
#include <ns3/satellite-constant-position-mobility-model.h>
#include <ns3/satellite-sgp4-mobility-model.h>
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
 * \brief TTE-aware candidate selection: the four behaviours this module claims.
 *
 * CHO-9. The test that used to carry this name asserted three things: that the
 * state was CHO_PREPARED, that four candidates had been added, and that an
 * unrelated A3 baseline picked cell 1. It never set a time-to-exit and never
 * called SelectBestCandidate, so all four behaviours listed in its own
 * docblock - quality filtering, TTE-minimum filtering, longest-TTE selection
 * and the SINR tie-break - had no coverage whatsoever, and the tteEpsilon
 * logic had none either. The single test named for the module's contribution
 * certified object construction.
 *
 * The selection rule admits on `admitted && d1Met && sinr >= qualityThreshold
 * && tte >= tteMinimum`, then ranks by TTE descending, then prefers the higher
 * SINR among candidates within tteEpsilon of the best. Each case below is
 * hand-computed against that rule.
 */
class NtnChoAlgorithmTestCase : public TestCase
{
  public:
    NtnChoAlgorithmTestCase()
        : TestCase("NTN CHO Algorithm - TTE-aware candidate selection")
    {
    }

  private:
    /// Build an algorithm with four candidates in a known admitted state.
    Ptr<NtnChoAlgorithm> MakeAlgo(NtnChoAlgorithm::ChoConfig config)
    {
        Ptr<NtnChoAlgorithm> algo = CreateObject<NtnChoAlgorithm>();
        algo->Configure(config);
        algo->SetServingCell(10);
        algo->AddCandidateCell(1, 0, 0);
        algo->AddCandidateCell(2, 0, 1);
        algo->AddCandidateCell(3, 1, 0);
        algo->AddCandidateCell(4, 1, 1);
        return algo;
    }

    void DoRun() override
    {
        NtnChoAlgorithm::ChoConfig config;
        config.triggerType = NtnChoAlgorithm::TRIGGER_TTE_AWARE;
        config.qualityThreshold_dB = -3.0;
        config.tteMinimum = Seconds(10.0);
        config.tteEpsilon = Seconds(2.0);
        config.maxCandidates = 8;

        // ---- 1. Longest TTE wins when nothing else separates the field ----
        {
            Ptr<NtnChoAlgorithm> algo = MakeAlgo(config);
            algo->UpdateMeasurement(1, 5.0, 2.0);
            algo->UpdateMeasurement(2, 5.0, 2.0);
            algo->UpdateMeasurement(3, 5.0, 2.0);
            algo->UpdateMeasurement(4, 5.0, 2.0);
            // TTEs well outside the 2 s epsilon of each other, so the tie-break
            // cannot participate and only the ranking is under test.
            algo->SetCandidateStateForTest(1, Seconds(15.0), true, true);
            algo->SetCandidateStateForTest(2, Seconds(40.0), true, true);
            algo->SetCandidateStateForTest(3, Seconds(25.0), true, true);
            algo->SetCandidateStateForTest(4, Seconds(12.0), true, true);
            NS_TEST_ASSERT_MSG_EQ(algo->SelectBestCandidate(), 2,
                                  "with equal SINR the longest TTE must win: cell 2 at 40 s");
        }

        // ---- 2. Candidates below the quality threshold are filtered ----
        {
            Ptr<NtnChoAlgorithm> algo = MakeAlgo(config);
            // Cell 2 has by far the longest TTE but is below -3 dB, so it must
            // not be selected however attractive its geometry is.
            algo->UpdateMeasurement(1, 5.0, 2.0);
            algo->UpdateMeasurement(2, -5.0, 2.0);
            algo->UpdateMeasurement(3, 4.0, 2.0);
            algo->UpdateMeasurement(4, 4.0, 2.0);
            algo->SetCandidateStateForTest(1, Seconds(15.0), true, true);
            algo->SetCandidateStateForTest(2, Seconds(90.0), true, true);
            algo->SetCandidateStateForTest(3, Seconds(25.0), true, true);
            algo->SetCandidateStateForTest(4, Seconds(12.0), true, true);
            NS_TEST_ASSERT_MSG_EQ(algo->SelectBestCandidate(), 3,
                                  "a candidate below qualityThreshold_dB must be filtered even "
                                  "when it has the longest TTE; cell 3 is the best remaining");
        }

        // ---- 3. Candidates below the TTE minimum are filtered ----
        {
            Ptr<NtnChoAlgorithm> algo = MakeAlgo(config);
            algo->UpdateMeasurement(1, 9.0, 2.0);
            algo->UpdateMeasurement(2, 5.0, 2.0);
            algo->UpdateMeasurement(3, 5.0, 2.0);
            algo->UpdateMeasurement(4, 5.0, 2.0);
            // Cell 1 has the best SINR by 4 dB but exits in 4 s, under the 10 s
            // minimum: handing over to it is the ping-pong this module exists
            // to prevent.
            algo->SetCandidateStateForTest(1, Seconds(4.0), true, true);
            algo->SetCandidateStateForTest(2, Seconds(30.0), true, true);
            algo->SetCandidateStateForTest(3, Seconds(20.0), true, true);
            algo->SetCandidateStateForTest(4, Seconds(9.9), true, true);
            const uint16_t sel = algo->SelectBestCandidate();
            NS_TEST_ASSERT_MSG_NE(sel, 1,
                                  "a candidate below tteMinimum must never be selected, however "
                                  "good its instantaneous SINR");
            NS_TEST_ASSERT_MSG_NE(sel, 4, "9.9 s is below the 10 s minimum and must be filtered");
            NS_TEST_ASSERT_MSG_EQ(sel, 2, "cell 2 has the longest admissible TTE");
        }

        // ---- 4. Within tteEpsilon, the higher SINR wins ----
        {
            Ptr<NtnChoAlgorithm> algo = MakeAlgo(config);
            algo->UpdateMeasurement(1, 2.0, 2.0);
            algo->UpdateMeasurement(2, 8.0, 2.0);
            algo->UpdateMeasurement(3, 5.0, 2.0);
            algo->UpdateMeasurement(4, 5.0, 2.0);
            // Cells 1 and 2 are 1.5 s apart, inside the 2 s epsilon, so the
            // slightly shorter TTE with the much better SINR must win.
            algo->SetCandidateStateForTest(1, Seconds(31.5), true, true);
            algo->SetCandidateStateForTest(2, Seconds(30.0), true, true);
            algo->SetCandidateStateForTest(3, Seconds(20.0), true, true);
            algo->SetCandidateStateForTest(4, Seconds(15.0), true, true);
            NS_TEST_ASSERT_MSG_EQ(algo->SelectBestCandidate(), 2,
                                  "inside tteEpsilon the tie-break is SINR: cell 2 is 1.5 s "
                                  "shorter but 6 dB stronger");

            // Push the same pair outside epsilon and the ranking must revert to
            // pure TTE, which is what proves epsilon is doing the work above
            // rather than SINR quietly dominating everywhere.
            algo->SetCandidateStateForTest(1, Seconds(35.0), true, true);
            NS_TEST_ASSERT_MSG_EQ(algo->SelectBestCandidate(), 1,
                                  "5 s apart is outside the 2 s epsilon, so the longer TTE wins "
                                  "again despite the weaker SINR");
        }

        // ---- 5. Nothing admissible yields no selection, not a default ----
        {
            Ptr<NtnChoAlgorithm> algo = MakeAlgo(config);
            algo->UpdateMeasurement(1, 5.0, 2.0);
            algo->UpdateMeasurement(2, 5.0, 2.0);
            algo->UpdateMeasurement(3, 5.0, 2.0);
            algo->UpdateMeasurement(4, 5.0, 2.0);
            // Admitted and above quality, but every TTE is under the minimum.
            algo->SetCandidateStateForTest(1, Seconds(1.0), true, true);
            algo->SetCandidateStateForTest(2, Seconds(2.0), true, true);
            algo->SetCandidateStateForTest(3, Seconds(3.0), true, true);
            algo->SetCandidateStateForTest(4, Seconds(4.0), true, true);
            NS_TEST_ASSERT_MSG_EQ(algo->SelectBestCandidate(),
                                  NtnChoAlgorithm::INVALID_CELL_ID,
                                  "with no admissible candidate the selector must return "
                                  "NtnChoAlgorithm::INVALID_CELL_ID rather than fall back to a best-of-a-bad-set");
        }

        // ---- 6. The trigger's own admission gate is respected ----
        {
            Ptr<NtnChoAlgorithm> algo = MakeAlgo(config);
            algo->UpdateMeasurement(1, 5.0, 2.0);
            algo->UpdateMeasurement(2, 5.0, 2.0);
            algo->UpdateMeasurement(3, 5.0, 2.0);
            algo->UpdateMeasurement(4, 5.0, 2.0);
            // Cell 2 has the longest TTE but the trigger never admitted it, and
            // cell 3's D1 geometric condition does not hold. Release-17
            // configures the geometric events jointly with the measurement leg
            // precisely so neither can admit on its own.
            algo->SetCandidateStateForTest(1, Seconds(20.0), true, true);
            algo->SetCandidateStateForTest(2, Seconds(60.0), false, true);
            algo->SetCandidateStateForTest(3, Seconds(40.0), true, false);
            algo->SetCandidateStateForTest(4, Seconds(15.0), true, true);
            NS_TEST_ASSERT_MSG_EQ(algo->SelectBestCandidate(), 1,
                                  "selection ranks only what the trigger admitted; an unadmitted "
                                  "candidate or one whose D1 condition is unmet cannot be chosen "
                                  "however long its TTE");
        }

        // The A3 baseline is a separate rule and keeps its own coverage: the
        // strongest neighbour above serving plus the offset.
        {
            Ptr<NtnChoAlgorithm> algo = MakeAlgo(config);
            algo->UpdateMeasurement(1, 5.0, 2.0);
            algo->UpdateMeasurement(2, 4.0, 1.5);
            algo->UpdateMeasurement(3, -5.0, -4.0);
            algo->UpdateMeasurement(4, 3.0, 1.0);
            NS_TEST_ASSERT_MSG_EQ(algo->GetState(), NtnChoAlgorithm::CHO_PREPARED,
                                  "adding candidates moves the machine to PREPARED");
            NS_TEST_ASSERT_MSG_EQ(algo->GetCandidates().size(), 4, "four candidates");
            NS_TEST_ASSERT_MSG_EQ(algo->SelectBaselineA3(-10.0), 1,
                                  "A3 selects the strongest neighbour above serving + offset");
        }

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
        const double rarMs = config.rachResponseWindow.GetMilliSeconds();

        // CHO-10. This test used to assert
        //     lastInterruptionMs == execMs + 2*slant/c + procMs
        // which is the model's own expression copied into the test, so it
        // certified the arithmetic rather than the physics and could not see
        // that the arithmetic was wrong.
        //
        // What TS 38.321 section 5.1 actually specifies for contention-based
        // random access is four messages:
        //     msg1 preamble               UE  -> gNB
        //     msg2 random-access response gNB -> UE
        //     msg3 scheduled transmission UE  -> gNB
        //     msg4 contention resolution  gNB -> UE
        // i.e. FOUR one-way traversals of the service link, not one round
        // trip. The expectations below are written from that message sequence
        // and from the configured RAR window, independently of how the model
        // chooses to sum them.
        const double oneWay1Ms = 600e3 / kC * 1e3;   // ~2.00 ms
        const double oneWay2Ms = 1500e3 / kC * 1e3;  // ~5.00 ms

        // Slant 1: LEO near-zenith range (600 km).
        const double slant1 = 600e3;
        algo->AddCandidateCell(2, 1, 0);
        algo->UpdateCandidateSlantRange(2, slant1);
        algo->ExecuteHandover(2);
        auto stats = algo->GetMechanismStats();
        const double expected1 = execMs + 4.0 * oneWay1Ms + rarMs + procMs;
        NS_TEST_ASSERT_MSG_EQ_TOL(stats.lastInterruptionMs, expected1, 0.1,
                                  "600 km: interruption = RRC execution + four one-way RACH "
                                  "traversals (msg1..msg4) + RAR window + processing");
        NS_TEST_ASSERT_MSG_EQ(stats.lastRachTraversals, 4,
                              "the four-step procedure must record four traversals, so a result "
                              "says which random-access procedure it assumed");
        NS_TEST_ASSERT_MSG_EQ(stats.rachExecutions, 1, "first HO pays the RACH");
        NS_TEST_ASSERT_MSG_EQ(stats.rachLessExecutions, 0, "no RACH-less execution");

        // Slant 2: low-elevation edge of the pass (1500 km).
        const double slant2 = 1500e3;
        algo->AddCandidateCell(3, 2, 0);
        algo->UpdateCandidateSlantRange(3, slant2);
        algo->ExecuteHandover(3);
        stats = algo->GetMechanismStats();
        const double expected2 = execMs + 4.0 * oneWay2Ms + rarMs + procMs;
        NS_TEST_ASSERT_MSG_EQ_TOL(stats.lastInterruptionMs, expected2, 0.1,
                                  "1500 km: same message sequence, longer service link");

        // The structural check that the old test could not make: the
        // interruption must grow by FOUR one-way traversals per unit of slant,
        // not two. Under the old one-round-trip pricing this difference is
        // 6.0 ms; under the four-message sequence it is 12.0 ms. Asserting the
        // SLOPE catches an under-count that any single absolute value, matched
        // to whatever the model computes, would hide.
        const double deltaMs = stats.lastInterruptionMs - expected1;
        NS_TEST_ASSERT_MSG_EQ_TOL(deltaMs, 4.0 * (oneWay2Ms - oneWay1Ms), 0.1,
                                  "interruption must scale with FOUR one-way traversals of the "
                                  "service link; a slope of two means the four-step procedure is "
                                  "being priced as a single round trip, which under-counts every "
                                  "NTN handover by a full slant RTT");

        // The two-step (msgA/msgB) procedure of the same clause costs half the
        // propagation. Configuring it must halve the propagation share and
        // nothing else.
        NtnChoAlgorithm::ChoConfig twoStep = config;
        twoStep.rachOneWayTraversals = 2;
        Ptr<NtnChoAlgorithm> algo2 = CreateObject<NtnChoAlgorithm>();
        algo2->Configure(twoStep);
        algo2->SetServingCell(1);
        algo2->AddCandidateCell(2, 1, 0);
        algo2->UpdateCandidateSlantRange(2, slant2);
        algo2->ExecuteHandover(2);
        auto s2 = algo2->GetMechanismStats();
        NS_TEST_ASSERT_MSG_EQ(s2.lastRachTraversals, 2, "two-step records two traversals");
        NS_TEST_ASSERT_MSG_EQ_TOL(s2.lastRachPropagationMs, 2.0 * oneWay2Ms, 0.01,
                                  "two-step random access crosses the link twice");
        NS_TEST_ASSERT_MSG_EQ_TOL(s2.lastInterruptionMs,
                                  execMs + 2.0 * oneWay2Ms + rarMs + procMs, 0.1,
                                  "only the propagation share changes between the two- and "
                                  "four-step procedures");

        // No slant known for the target -> constant rachDuration fallback.
        algo->AddCandidateCell(4, 3, 0);
        algo->ExecuteHandover(4);
        stats = algo->GetMechanismStats();
        const double expectedFallback = execMs + config.rachDuration.GetMilliSeconds();
        NS_TEST_ASSERT_MSG_EQ_TOL(stats.lastInterruptionMs, expectedFallback, 0.1,
                                  "unknown slant: falls back to constant rachDuration");
        NS_TEST_ASSERT_MSG_EQ(stats.lastRachTraversals, 0,
                              "the constant fallback crosses no modelled link, and must say so "
                              "rather than implying a message count it did not price");
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
 * \brief NtnGeodeticFixedMobilityModel — ground terminal anchored at a real
 *        geodetic location reports the correct ECEF point (round-trips through
 *        ntngeo), is stationary in ECEF, and traces the diurnal circle in ECI.
 */
/// CHO-8: the module's central prediction must actually be tested.
///
/// ComputeTte is the algorithm the whole module is named for and no test called
/// it. A repo-wide grep over contrib/ntn-cho/test returned one docblock mention
/// and nothing else. The regression test written for the orbit-propagation bug
/// pinned its satellites with zero velocity, so r + v*dt could not move them and
/// the test agreed with the code by construction.
///
/// This drives ComputeTte on a geometry whose answer is known without running
/// the estimator: a satellite crossing overhead at a fixed ground speed, with an
/// analytic beam whose half-power contour is a known off-nadir angle. The exit
/// time follows from the crossing geometry, not from the implementation.
class NtnTteEstimatorComputeTteTestCase : public TestCase
{
  public:
    NtnTteEstimatorComputeTteTestCase()
        : TestCase("CHO-8 - ComputeTte predicts the exit of a known crossing geometry")
    {
    }

    void DoRun() override
    {
        // A 600 km satellite moving along +x at 7.56 km/s, starting directly
        // above a terminal at the origin's sub-point. The analytic beam is a
        // 4-degree half-power beamwidth, so the terminal leaves the -3 dB
        // contour when the off-nadir angle reaches 2 degrees.
        //
        // At 600 km altitude, an off-nadir angle of 2 degrees is a ground
        // displacement of 600e3 * tan(2 deg) = 20.95 km. At 7.56 km/s that
        // takes 20.95 / 7.56 = 2.77 s. That number comes from the crossing
        // geometry; nothing in the estimator produced it.
        const double altM = 600e3;
        const double vx = 7560.0;
        const double bw3dB = 4.0;
        const double expectedS = (altM * std::tan(2.0 * M_PI / 180.0)) / vx;

        // ECEF: the terminal sits at (lat 0, lon 0), which is (R, 0, 0), so the
        // satellite directly above it is at (R + alt, 0, 0) - NOT (0, 0, R+alt),
        // which is over the pole and 90 degrees away. Velocity is eastward,
        // along +y, so the sub-satellite point tracks along the equator.
        auto satMob = CreateObject<ConstantVelocityMobilityModel>();
        satMob->SetPosition(Vector(6371e3 + altM, 0.0, 0.0));
        satMob->SetVelocity(Vector(0.0, vx, 0.0));

        auto pred = CreateObject<NtnOrbitPredictor>();
        pred->SetKinematicsSource(1, satMob);
        // Analytic beam, boresight fixed at nadir: SetSteeredBeam(false) keeps
        // the pattern in the body frame so the terminal really does traverse it.
        pred->SetGeometricBeam(30.0, bw3dB);
        pred->SetSteeredBeam(false);

        auto est = CreateObject<NtnTteEstimator>();
        est->SetOrbitPredictor(pred);
        // The coarse step is floored at 100 ms by its own checker; precision on
        // the crossing comes from the binary search that refines between steps,
        // so tighten that instead.
        est->SetAttribute("PredictionStep", TimeValue(MilliSeconds(100)));
        est->SetAttribute("MaxPredictionWindow", TimeValue(Seconds(60.0)));
        est->SetAttribute("BinarySearchTolerance", TimeValue(MilliSeconds(10)));

        // Terminal on the ground at the initial sub-satellite point.
        const GeoCoordinate uePos(0.0, 0.0, 0.0);

        const auto r = est->ComputeTte(uePos, Vector(0, 0, 0), 1, 0, /*gainThreshold*/ -3.0);

        NS_TEST_ASSERT_MSG_EQ(r.isValid, true,
                              "a terminal directly under the beam must yield a valid TTE; an "
                              "invalid result here means the estimator never entered coverage");
        NS_TEST_ASSERT_MSG_GT(r.currentGain_dB, -3.0,
                              "at nadir the terminal sits at the beam peak, comfortably above the "
                              "half-power threshold it is being tested against");

        // The predicted exit must match the crossing geometry. A 25 percent band
        // absorbs the Earth-curvature term the flat-plane estimate above omits
        // and the 10 ms search granularity, while still failing anything that is
        // not tracking the geometry at all.
        NS_TEST_ASSERT_MSG_EQ_TOL(r.tte.GetSeconds(), expectedS, 0.25 * expectedS,
                                  "predicted time-to-exit must follow the crossing geometry: at "
                                  "600 km and 7.56 km/s the terminal reaches the 2 degree "
                                  "off-nadir half-power contour in about 2.8 s");

        // WF-08 gate 7: TTE against geometric truth within 10 PERCENT.
        //
        // The CI tally records gate 7 as "TTE vs geometric truth +/- 10%". The
        // band above is 25%, and it has to be: the flat-plane expectation
        // ignores Earth curvature, which for this geometry is worth 9.4% on its
        // own, so a 10% bound around it would fail on the approximation rather
        // than on the estimator.
        //
        // The fix is a better reference, not a looser bound. Solving the real
        // spherical geometry for the central angle phi at which the off-nadir
        // angle reaches 2 degrees,
        //
        //     tan(theta) = R*sin(phi) / (Rs - R*cos(phi)),   t = phi / (v/Rs)
        //
        // gives 3.0327 s against the flat-plane 2.7715 s. THAT is geometric
        // truth, and the estimator can be held to 10% of it.
        {
            const double Re = 6371e3;
            const double Rs = Re + altM;
            const double omega = vx / Rs; // rad/s about the Earth's centre
            const double theta = 2.0 * M_PI / 180.0;
            double lo = 0.0;
            double hi = 20.0 * M_PI / 180.0;
            for (int it = 0; it < 200; ++it)
            {
                const double mid = 0.5 * (lo + hi);
                const double off = std::atan2(Re * std::sin(mid), Rs - Re * std::cos(mid));
                (off < theta ? lo : hi) = mid;
            }
            const double expectedCurvedS = lo / omega;
            NS_TEST_ASSERT_MSG_EQ_TOL(r.tte.GetSeconds(), expectedCurvedS,
                                      0.10 * expectedCurvedS,
                                      "gate 7: predicted time-to-exit must be within 10% of the "
                                      "spherical-geometry truth (" << expectedCurvedS << " s). "
                                      "The 25% band above is around a FLAT-plane estimate that "
                                      "is itself 9.4% low for this geometry, so it could never "
                                      "have enforced the tolerance the CI tally claims");
        }

        // The exit gain must be at the threshold, not past it: the binary search
        // exists to land on the crossing rather than overshoot to the next step.
        NS_TEST_ASSERT_MSG_LT(std::abs(r.exitGain_dB - (-3.0)), 1.0,
                              "the binary search must land on the threshold crossing; a large "
                              "error means it is reporting the first coarse step that failed "
                              "instead of refining it");

        // Peak gain is recorded over the coverage period and cannot be below
        // the gain the terminal already has.
        NS_TEST_ASSERT_MSG_GT(r.peakGain_dB, r.exitGain_dB,
                              "the peak seen during coverage must exceed the gain at exit");

        // A threshold above the beam peak means the terminal is never covered,
        // and the estimator must say zero rather than return the window.
        const auto never = est->ComputeTte(uePos, Vector(0, 0, 0), 1, 0, /*threshold*/ 40.0);
        NS_TEST_ASSERT_MSG_EQ_TOL(never.tte.GetSeconds(), 0.0, 1e-9,
                                  "with a threshold above the beam peak the terminal is out of "
                                  "coverage now, so TTE is zero - not the full prediction window, "
                                  "which is what the module reported before CHO-1");

        Simulator::Destroy();
    }
};

class NtnGeodeticFixedMobilityTestCase : public TestCase
{
  public:
    NtnGeodeticFixedMobilityTestCase()
        : TestCase("NtnGeodeticFixedMobilityModel - ECEF anchor + ECI rotation")
    {
    }

    void DoRun() override
    {
        // A terminal in Islamabad: 33.6844 N, 73.0479 E, 540 m.
        const double lat = 33.6844;
        const double lon = 73.0479;
        const double alt = 540.0;

        Ptr<NtnGeodeticFixedMobilityModel> m = CreateObject<NtnGeodeticFixedMobilityModel>();
        m->SetGeodetic(lat, lon, alt);

        // ECEF position must match the independent ntngeo conversion to mm.
        const Vector expect = ntngeo::GeodeticToEcef(lat, lon, alt);
        const Vector p0 = m->GetPosition();
        NS_TEST_ASSERT_MSG_EQ_TOL(p0.x, expect.x, 1e-3, "ECEF x mismatch");
        NS_TEST_ASSERT_MSG_EQ_TOL(p0.y, expect.y, 1e-3, "ECEF y mismatch");
        NS_TEST_ASSERT_MSG_EQ_TOL(p0.z, expect.z, 1e-3, "ECEF z mismatch");

        // |ECEF| is sensible for a near-surface point (~6.37e6 m).
        const double r = std::sqrt(p0.x * p0.x + p0.y * p0.y + p0.z * p0.z);
        NS_TEST_ASSERT_MSG_GT(r, 6.36e6, "ECEF radius too small");
        NS_TEST_ASSERT_MSG_LT(r, 6.39e6, "ECEF radius too large");

        // Stationary in the Earth-fixed frame: zero velocity, position constant
        // across simulated time.
        const Vector v = m->GetVelocity();
        NS_TEST_ASSERT_MSG_EQ_TOL(v.x, 0.0, 1e-9, "ECEF vx should be 0");
        NS_TEST_ASSERT_MSG_EQ_TOL(v.y, 0.0, 1e-9, "ECEF vy should be 0");

        Simulator::Schedule(Seconds(100.0), [&]() {
            const Vector pt = m->GetPosition();
            NS_TEST_ASSERT_MSG_EQ_TOL(pt.x, p0.x, 1e-6, "ECEF must not drift in time");
            NS_TEST_ASSERT_MSG_EQ_TOL(pt.y, p0.y, 1e-6, "ECEF must not drift in time");
        });
        Simulator::Stop(Seconds(101.0));
        Simulator::Run();
        Simulator::Destroy();

        // ECI mode: at t=0 ECI==ECEF; the ECI velocity is omega x r (eastward),
        // magnitude ~ omega * sqrt(x^2+y^2).
        m->SetReportFrame(NtnGeodeticFixedMobilityModel::Eci);
        const Vector eci0 = m->GetPosition();
        NS_TEST_ASSERT_MSG_EQ_TOL(eci0.x, p0.x, 1e-3, "ECI==ECEF at t=0 (x)");
        NS_TEST_ASSERT_MSG_EQ_TOL(eci0.y, p0.y, 1e-3, "ECI==ECEF at t=0 (y)");
        const Vector eciVel = m->GetVelocity();
        const double speed = std::sqrt(eciVel.x * eciVel.x + eciVel.y * eciVel.y);
        const double omega = 7.2921159e-5;
        const double expectSpeed = omega * std::sqrt(p0.x * p0.x + p0.y * p0.y);
        NS_TEST_ASSERT_MSG_EQ_TOL(speed, expectSpeed, 1.0, "ECI ground speed ~ omega*rho");
        NS_TEST_ASSERT_MSG_GT(speed, 100.0, "ECI ground speed should be hundreds of m/s");
    }
};

/**
 * \ingroup ntn-cho-test
 * \brief Rel-17 standardized NTN CHO triggers — elevation floor and
 *        timing-advance — fire on the right geometry, independently selectable.
 *
 * These two join the already-tested A3 (NtnChoAlgorithmTestCase) and D2
 * (NtnChoDistanceD2TriggerTestCase) so the full standardized trigger set is
 * covered. Both are pure slant-range geometry, so they are driven directly via
 * UpdateCandidateSlantRange with no orbit predictor.
 */
class NtnChoStandardizedTriggersTestCase : public TestCase
{
  public:
    NtnChoStandardizedTriggersTestCase()
        : TestCase("NTN CHO Algorithm - Rel-17 elevation + timing-advance triggers")
    {
    }

  private:
    void DoRun() override
    {
        // ---- Elevation-floor trigger: serving below floor, candidate above
        //      floor + hysteresis. (550 km shell.) ----
        {
            Ptr<NtnChoAlgorithm> algo = CreateObject<NtnChoAlgorithm>();
            NtnChoAlgorithm::ChoConfig cfg;
            cfg.triggerType = NtnChoAlgorithm::TRIGGER_ELEVATION;
            cfg.qualityThreshold_dB = -20.0; // permissive: geometry decides
            cfg.orbitAltitudeKm = 550.0;
            cfg.elevationMinDeg = 10.0;
            cfg.elevationHystDeg = 2.0;
            cfg.conditionMonitorPeriod = Seconds(1000.0);
            algo->Configure(cfg);
            algo->AddCandidateCell(1, 0, 0); // serving (low elevation)
            algo->AddCandidateCell(2, 1, 0); // high-elevation candidate
            algo->AddCandidateCell(3, 2, 0); // marginal candidate (within hyst)
            algo->SetServingCell(1);
            algo->UpdateMeasurement(1, 0.0, 0.0);
            algo->UpdateMeasurement(2, 0.0, 0.0);
            algo->UpdateMeasurement(3, 0.0, 0.0);
            algo->UpdateCandidateSlantRange(1, 2000.0e3); // elev ~7.5 deg (< 10)
            algo->UpdateCandidateSlantRange(2, 700.0e3);  // elev ~50 deg (>= 12)
            algo->UpdateCandidateSlantRange(3, 1800.0e3); // elev ~10.2 deg (< 12)
            algo->StartMonitoring(GeoCoordinate(0.0, 0.0, 0.0), Vector(0.0, 0.0, 0.0));
            NS_TEST_ASSERT_MSG_EQ(algo->GetCandidates()[2].admitted, true,
                                  "ELEVATION: high-elevation candidate admitted");
            NS_TEST_ASSERT_MSG_EQ(algo->GetCandidates()[3].admitted, false,
                                  "ELEVATION: marginal candidate within hysteresis rejected");
            algo->StopMonitoring();
            Simulator::Destroy();
        }

        // ---- Timing-advance trigger: candidate offering >= taAdvantage less
        //      TA admitted; negligible advantage rejected; serving TA over the
        //      ceiling admits regardless. ----
        {
            Ptr<NtnChoAlgorithm> algo = CreateObject<NtnChoAlgorithm>();
            NtnChoAlgorithm::ChoConfig cfg;
            cfg.triggerType = NtnChoAlgorithm::TRIGGER_TIMING_ADVANCE;
            cfg.qualityThreshold_dB = -20.0;
            cfg.taServingMax = MilliSeconds(8);
            cfg.taAdvantage = MilliSeconds(1);
            cfg.conditionMonitorPeriod = Seconds(1000.0);
            algo->Configure(cfg);
            algo->AddCandidateCell(1, 0, 0); // serving (TA below ceiling)
            algo->AddCandidateCell(2, 1, 0); // candidate with >= 1 ms advantage
            algo->AddCandidateCell(3, 2, 0); // candidate with negligible advantage
            algo->SetServingCell(1);
            algo->UpdateMeasurement(1, 0.0, 0.0);
            algo->UpdateMeasurement(2, 0.0, 0.0);
            algo->UpdateMeasurement(3, 0.0, 0.0);
            algo->UpdateCandidateSlantRange(1, 1000.0e3); // TA = 6.67 ms (< 8)
            algo->UpdateCandidateSlantRange(2, 700.0e3);  // TA = 4.67 ms (2 ms gain)
            algo->UpdateCandidateSlantRange(3, 960.0e3);  // TA = 6.40 ms (0.27 ms gain)
            algo->StartMonitoring(GeoCoordinate(0.0, 0.0, 0.0), Vector(0.0, 0.0, 0.0));
            NS_TEST_ASSERT_MSG_EQ(algo->GetCandidates()[2].admitted, true,
                                  "TA: candidate with >= 1 ms advantage admitted");
            NS_TEST_ASSERT_MSG_EQ(algo->GetCandidates()[3].admitted, false,
                                  "TA: candidate with negligible advantage rejected");
            // Serving TA now exceeds the ceiling -> any candidate admitted.
            algo->UpdateCandidateSlantRange(1, 1500.0e3); // TA = 10 ms (> 8)
            algo->EvaluateConditions();
            NS_TEST_ASSERT_MSG_EQ(algo->GetCandidates()[3].admitted, true,
                                  "TA: serving TA over ceiling admits candidate");
            algo->StopMonitoring();
            Simulator::Destroy();
        }
    }
};

/**
 * \ingroup ntn-cho-test
 * \brief NTN CHO Test Suite
 */
/**
 * \brief GAP C1 REGRESSION: the beam gain must follow the SATELLITE through time.
 *
 * The TTE estimator's forward search asked for the gain at a series of future
 * instants, but the predictor evaluated the antenna pattern against the
 * satellite's position at Simulator::Now() — simulation time does not advance
 * inside a single event, and the "satellite position is automatically updated by
 * the SGP4 model" comment was simply false. For a static UE the gain was
 * therefore IDENTICAL at every step of the search, the beam exit was never
 * found, and ComputeTte() returned the full prediction window (120 s) for every
 * beam. That silently made the TTE-aware admission (tte >= tteMinimum) always
 * true and condEventT1 (serving TTE <= window) unreachable — the module's
 * headline mechanism was a constant.
 *
 * No test caught it because none propagated a satellite through a beam exit.
 * This one pins the invariant directly: moving the satellite along its orbit
 * must change the gain a fixed ground UE sees, and moving it far enough must
 * take the UE out of the beam entirely.
 */
/// CHO-4: the standardized triggers must survive a handover.
///
/// Four of them (T1, elevation, timing-advance, D2) compare a candidate against
/// the SERVING cell, and read the serving cell's satId/beamId/slantRange out of
/// the candidate map. NotifyHandoverComplete correctly erases the cell just
/// moved to, since it is now serving rather than a candidate, so from the first
/// successful handover onward that lookup missed and every one of those triggers
/// returned early. They were dead for the rest of the run while still being
/// reported as the active trigger.
class NtnChoTriggersSurviveHandoverTestCase : public TestCase
{
  public:
    NtnChoTriggersSurviveHandoverTestCase()
        : TestCase("CHO-4 - serving-cell geometry survives a handover, so triggers stay alive")
    {
    }

  private:
    void DoRun() override
    {
        auto cho = CreateObject<NtnChoAlgorithm>();
        NtnChoAlgorithm::ChoConfig cfg = cho->GetConfig();
        cfg.triggerType = NtnChoAlgorithm::TRIGGER_ELEVATION;
        cho->Configure(cfg);

        const uint16_t cellA = 1;
        const uint16_t cellB = 2;
        cho->AddCandidateCell(cellA, 0, 0);
        cho->AddCandidateCell(cellB, 1, 0);
        cho->SetServingCell(cellA);

        // Serving geometry known before any handover.
        cho->UpdateCandidateSlantRange(cellA, 1200e3);
        cho->UpdateMeasurement(cellA, 12.0, 25.0);
        NS_TEST_ASSERT_MSG_EQ(cho->HaveServingState(), true,
                              "serving geometry must be known before the first handover");

        // Execute a handover to B, which is what erases B from the candidate map.
        cho->NotifyHandoverComplete(cellB, true);
        cho->SetServingCell(cellB);

        // Feed the NEW serving cell's geometry. Under the defect this went only
        // into the candidate map, where cellB no longer is, so the serving
        // record stayed empty and every comparison trigger bailed out.
        cho->UpdateCandidateSlantRange(cellB, 1500e3);
        cho->UpdateMeasurement(cellB, 9.0, 22.0);

        NS_TEST_ASSERT_MSG_EQ(cho->HaveServingState(), true,
                              "after a handover the serving cell's geometry must still be "
                              "reachable; losing it is what silently disabled the T1, "
                              "elevation, timing-advance and D2 triggers for the rest of "
                              "the run");
        Simulator::Destroy();
    }
};

/// CHO-1: the orbit predictor must propagate ALONG THE ORBIT, not along the
/// velocity tangent.
///
/// GetBeamSnapshotAtTime used r(t+dt) = r(t) + v(t)*dt under a comment claiming
/// the drift was "< 30 m over 120 s". For a 780 km shell the true straight-line
/// error is ~51 km at 120 s, larger than a service-beam footprint, so the
/// time-to-exit estimate built on it was meaningless at the horizon it was
/// asked about. The comment's own formula, dt^2*v^2/r, evaluates to 112 km.
///
/// Truth here is an analytic circular orbit in ECEF, which is exactly what the
/// two-body propagation should reproduce. The test asserts BOTH that the new
/// form is accurate and that the old form would have failed, so it cannot pass
/// vacuously.
class NtnOrbitPredictorPropagationAccuracyTestCase : public TestCase
{
  public:
    NtnOrbitPredictorPropagationAccuracyTestCase()
        : TestCase("CHO-1 - predicted position tracks the orbit, not the velocity tangent")
    {
    }

  private:
    static constexpr double kMu = 3.986004418e14;
    static constexpr double kWe = 7.2921159e-5;

    /// Analytic circular-orbit ECEF position at time t, inclination inc.
    static Vector TruthEcef(double r, double inc, double t)
    {
        const double vOrb = std::sqrt(kMu / r);
        const double th = vOrb * t / r;
        const double x = r * std::cos(th);
        const double y = r * std::sin(th) * std::cos(inc);
        const double z = r * std::sin(th) * std::sin(inc);
        const double g = kWe * t;
        return Vector(x * std::cos(g) + y * std::sin(g), -x * std::sin(g) + y * std::cos(g), z);
    }

    void DoRun() override
    {
        const double r = 6371e3 + 780e3; // 780 km shell
        const double inc = 53.0 * M_PI / 180.0;

        // ECEF state at t = 0, velocity by central difference on the truth.
        const Vector p0 = TruthEcef(r, inc, 0.0);
        const double h = 1e-3;
        const Vector pP = TruthEcef(r, inc, h);
        const Vector pM = TruthEcef(r, inc, -h);
        const Vector v0((pP.x - pM.x) / (2 * h), (pP.y - pM.y) / (2 * h), (pP.z - pM.z) / (2 * h));

        double maxTwoBody = 0.0;
        double maxLinear = 0.0;
        for (double dt : {10.0, 30.0, 60.0, 120.0})
        {
            const Vector truth = TruthEcef(r, inc, dt);

            const Vector got = NtnOrbitPredictor::PropagateTwoBodyEcef(p0, v0, dt);
            maxTwoBody = std::max(maxTwoBody,
                                  std::sqrt(std::pow(got.x - truth.x, 2) +
                                            std::pow(got.y - truth.y, 2) +
                                            std::pow(got.z - truth.z, 2)));

            // The form this replaces, for comparison.
            const Vector lin(p0.x + v0.x * dt, p0.y + v0.y * dt, p0.z + v0.z * dt);
            maxLinear = std::max(maxLinear,
                                 std::sqrt(std::pow(lin.x - truth.x, 2) +
                                           std::pow(lin.y - truth.y, 2) +
                                           std::pow(lin.z - truth.z, 2)));
        }

        // A 3-degree service beam at 550 km has a footprint radius near 29 km.
        NS_TEST_ASSERT_MSG_LT(maxTwoBody, 1000.0,
                              "two-body propagation must stay within 1 km of a circular orbit "
                              "out to a 120 s horizon");
        NS_TEST_ASSERT_MSG_GT(maxLinear, 20000.0,
                              "the tangent extrapolation this replaced must be shown to exceed "
                              "20 km, otherwise this test proves nothing about the defect");
        NS_TEST_ASSERT_MSG_LT(maxTwoBody * 100.0, maxLinear,
                              "the fix must be at least two orders of magnitude better than the "
                              "form it replaced");
        Simulator::Destroy();
    }
};

class NtnOrbitPredictorTimeOffsetGainTestCase : public TestCase
{
  public:
    NtnOrbitPredictorTimeOffsetGainTestCase()
        : TestCase("Beam gain is evaluated at the propagated satellite position (gap C1)")
    {
    }

  private:
    void DoRun() override
    {
        D2Fixture f = MakeD2Fixture();

        // Gain at the satellite's current position (beam 12 of geo-33E covers
        // the fixture UE).
        const GeoCoordinate satNow(0.0, 33.0, 35786000.0);
        const double gainHere = f.predictor->ComputeBeamGainAt(0, 12, f.uePos, satNow);
        NS_TEST_ASSERT_MSG_GT(gainHere, -100.0, "UE should start inside beam 12");

        // Same UE, same beam, satellite displaced along its track. A body-fixed
        // pattern must now illuminate a different footprint, so the gain the UE
        // sees MUST change. Before the C1 fix the gain could not depend on the
        // satellite position at all.
        const GeoCoordinate satShifted(0.0, 43.0, 35786000.0); // 10 deg along track
        const double gainShifted = f.predictor->ComputeBeamGainAt(0, 12, f.uePos, satShifted);
        NS_TEST_ASSERT_MSG_NE(gainHere,
                              gainShifted,
                              "beam gain must depend on the satellite position — a frozen "
                              "satellite is exactly the C1 defect");

        // Far enough away the UE must fall out of coverage entirely, which is
        // the beam-exit event the TTE search exists to find.
        const GeoCoordinate satFar(0.0, 120.0, 35786000.0);
        const double gainFar = f.predictor->ComputeBeamGainAt(0, 12, f.uePos, satFar);
        NS_TEST_ASSERT_MSG_LT(gainFar,
                              gainHere,
                              "moving the satellite away from the UE must reduce the gain");

        // The snapshot API the TTE search actually calls must route the gain
        // through the PROPAGATED position. This fixture pins its satellites with
        // SatConstantPositionMobilityModel (velocity 0), so r + v*dt cannot move
        // them and the snapshot is correctly time-invariant here — that is the
        // fixture's nature, not a defect. What we CAN pin without a moving
        // mobility is that the snapshot reports the gain of the position it
        // claims to have propagated to, i.e. it agrees with a direct evaluation
        // at snap.satellitePosition rather than silently reporting the
        // "now" gain from some other geometry.
        const auto snap = f.predictor->GetBeamSnapshotAtTime(0, 12, f.uePos, Seconds(120));
        const double direct =
            f.predictor->ComputeBeamGainAt(0, 12, f.uePos, snap.satellitePosition);
        NS_TEST_ASSERT_MSG_EQ_TOL(snap.gainAtUe_dB,
                                  direct,
                                  1e-9,
                                  "the snapshot's gain must be the gain at the satellite position "
                                  "it propagated to");
    }
};


/// CHO-12: RACH-less must change TIMING, not only a counter.
///
/// lastPreCompTaUs was written into a stats struct and read only by example
/// print statements. No code path fed it to a UE MAC or to N_TA, so the
/// difference between RACH-less and classic CHO in this module was which
/// constant was added to an interruption total. The header describing LTM as "a
/// MAC-CE-style fast cell switch" invited a reader to assume otherwise.
///
/// The pre-compensation now reaches a consumer, and this checks the thing that
/// actually matters about it: whether the residual timing advance the gNB would
/// have to absorb fits inside the RAR window.
class NtnChoRachLessDrivesTimingAdvanceTestCase : public TestCase
{
  public:
    NtnChoRachLessDrivesTimingAdvanceTestCase()
        : TestCase("CHO-12: RACH-less pre-compensation reaches a timing-advance consumer")
    {
    }

  private:
    Time m_rtt{};
    Time m_preComp{};
    uint32_t m_calls{0};

    void OnPreComp(Time rtt, Time preComp)
    {
        m_rtt = rtt;
        m_preComp = preComp;
        ++m_calls;
    }

    Ptr<NtnChoAlgorithm> MakeAlgo(bool rachLess)
    {
        Ptr<NtnChoAlgorithm> algo = CreateObject<NtnChoAlgorithm>();
        NtnChoAlgorithm::ChoConfig cfg;
        cfg.triggerType = NtnChoAlgorithm::TRIGGER_TTE_AWARE;
        cfg.rachLess = rachLess;
        algo->Configure(cfg);
        algo->AddCandidateCell(1, 0, 0);
        // 1200 km slant: one way 4.003 ms, round trip 8.006 ms.
        algo->UpdateCandidateSlantRange(1, 1200e3);
        algo->SetPreCompensationSink(
            MakeCallback(&NtnChoRachLessDrivesTimingAdvanceTestCase::OnPreComp, this));
        return algo;
    }

    void DoRun() override
    {
        const double slantM = 1200e3;
        const Time expectRtt = NtnRachWindow::RoundTripForSlantRange(slantM);

        // ---- RACH-less ON: fully pre-compensated, no residual ----
        {
            m_calls = 0;
            Ptr<NtnChoAlgorithm> algo = MakeAlgo(true);
            algo->ExecuteHandover(1);
            NS_TEST_ASSERT_MSG_EQ(m_calls, 1u,
                                  "the execution must report its pre-compensation; zero calls "
                                  "means the value is still stopping in the stats struct");
            NS_TEST_ASSERT_MSG_EQ_TOL(m_rtt.GetSeconds(), expectRtt.GetSeconds(), 1e-6,
                                      "the reported round trip comes from the target's real "
                                      "slant range");
            NS_TEST_ASSERT_MSG_EQ_TOL(m_preComp.GetSeconds(), m_rtt.GetSeconds(), 1e-9,
                                      "RACH-less means the UE applies the WHOLE round trip from "
                                      "ephemeris and GNSS, so the gNB measures no residual - "
                                      "that is what skipping random access rests on");
            NS_TEST_ASSERT_MSG_EQ(algo->GetMechanismStats().rachLessExecutions, 1u,
                                  "and it is counted as a RACH-less execution");
        }

        // ---- RACH-less OFF: nothing pre-compensated ----
        {
            m_calls = 0;
            Ptr<NtnChoAlgorithm> algo = MakeAlgo(false);
            algo->ExecuteHandover(1);
            NS_TEST_ASSERT_MSG_EQ(m_calls, 1u, "the un-compensated case is reported too");
            NS_TEST_ASSERT_MSG_EQ_TOL(m_rtt.GetSeconds(), expectRtt.GetSeconds(), 1e-6,
                                      "same geometry, same round trip");
            NS_TEST_ASSERT_MSG_EQ(m_preComp.IsZero(), true,
                                  "with no pre-compensation the UE applies nothing, so the whole "
                                  "round trip is residual");
        }

        // ---- Why it matters: the residual against the RAR window ----
        // This is the assertion the TA chain was missing at its terminus. A
        // value existing is not a value being sufficient.
        const Time slot = NtnRachWindow::SlotPeriodForNumerology(1);
        const NtnRachWindowVerdict uncompensated = NtnRachWindow::Evaluate(expectRtt, slot);
        NS_TEST_ASSERT_MSG_EQ(uncompensated.fits, false,
                              "un-compensated, a 1200 km link needs a RAR window beyond what nr "
                              "can express, so random access cannot complete - which is exactly "
                              "the failure pre-compensation exists to avoid");
        const NtnRachWindowVerdict compensated = NtnRachWindow::Evaluate(Time(), slot);
        NS_TEST_ASSERT_MSG_EQ(compensated.fits, true,
                              "fully pre-compensated there is no residual flight to cover, so "
                              "the window is not the constraint");
    }
};


/// WF-08 gate 6: every CHO execution must produce exactly one handover.
///
/// The tally records gate 6 as "CHO-driven X2 count EQUALITY". The checker that
/// stands behind it, tools/check_ntn_standards.py, asserts only `handovers >=
/// 1`. That passes if the decision model requests forty handovers and the stack
/// performs one, which is the exact failure mode - a decision island - this
/// campaign has found repeatedly elsewhere.
/// CHO-13: the UE forward projection must read its velocity in the frame the
/// callers actually supply, which is ECEF.
///
/// ProjectUePosition treated the vector's x as a NORTHWARD rate and y as an
/// EASTWARD one. Every caller supplies ECEF, where x and y are axes through the
/// Greenwich meridian and 90 degrees east of it, so outside a very specific spot
/// near (0, 0) the projected UE moved in the wrong direction entirely.
/// CHO-17: one header, two writers, three different row shapes.
///
/// The header declared six columns. RecordHandoverOutcome() wrote four
/// (time, cellId, success, reason) and OnHandoverExecuted() wrote a different
/// four (time, source, target, timeOfStay), interleaved into the same stream.
/// Column 2 meant "the cell an outcome refers to" on some rows and "the source
/// cell of an executed handover" on others, and no row ever had six fields.
/// Anyone parsing this file by column index got silent nonsense.
///
/// No shipped example calls EnableTraces, so the broken file was never actually
/// produced. That is not a defence: the writer would emit it the moment anyone
/// turned traces on, which is exactly what a reader of the header would do.
class ChoHandoverCsvIsWellFormedTestCase : public TestCase
{
  public:
    ChoHandoverCsvIsWellFormedTestCase()
        : TestCase("CHO-17: every handover-trace row fills the declared columns")
    {
    }

  private:
    static std::vector<std::string> Split(const std::string& line)
    {
        std::vector<std::string> out;
        std::string cur;
        bool inQuotes = false;
        for (char c : line)
        {
            if (c == '"')
            {
                inQuotes = !inQuotes;
                cur += c;
            }
            else if (c == ',' && !inQuotes)
            {
                out.push_back(cur);
                cur.clear();
            }
            else
            {
                cur += c;
            }
        }
        out.push_back(cur);
        return out;
    }

    void DoRun() override
    {
        const std::string dir = "test-cho17-csv";
        std::filesystem::create_directories(dir);
        const std::string path = dir + "/ntn-cho-handovers.csv";
        std::remove(path.c_str());

        {
            NtnChoHelper helper;
            helper.EnableTraces(dir);
            // Both writers, interleaved, exactly as a run would produce them.
            helper.OnHandoverExecuted(11, 22, Seconds(3.5));
            helper.RecordHandoverOutcome(22, true, "ok");
            helper.OnHandoverExecuted(22, 33, Seconds(4.25));
            // A reason containing a comma: without quoting this splits the row
            // and every later column shifts, which is the same class of defect
            // one layer down.
            helper.RecordHandoverOutcome(33, false, "radio link failure, no response");
        }

        std::ifstream f(path);
        NS_TEST_ASSERT_MSG_EQ(f.good(), true, "the trace file must be written");
        std::string header;
        std::getline(f, header);
        const auto cols = Split(header);
        NS_TEST_ASSERT_MSG_EQ(cols.size(), 7u,
                              "the header declares its own column count; got " << cols.size());
        NS_TEST_ASSERT_MSG_EQ(cols[1], std::string("Event"),
                              "an Event column must distinguish the two writers, which used to "
                              "share a stream with incompatible row shapes");

        uint32_t rows = 0;
        uint32_t executed = 0;
        uint32_t outcomes = 0;
        std::string line;
        while (std::getline(f, line))
        {
            if (line.empty())
            {
                continue;
            }
            ++rows;
            const auto v = Split(line);
            NS_TEST_ASSERT_MSG_EQ(v.size(), cols.size(),
                                  "row " << rows << " has " << v.size() << " fields under a "
                                         << cols.size() << "-column header: '" << line << "'");
            if (v[1] == "executed")
            {
                ++executed;
                // An execution knows its source and target and its time of stay,
                // and does not yet know the outcome.
                NS_TEST_ASSERT_MSG_NE(v[2], std::string("-"), "executed rows carry a source");
                NS_TEST_ASSERT_MSG_NE(v[3], std::string("-"), "and a target");
                NS_TEST_ASSERT_MSG_EQ(v[5], std::string("-"),
                                      "and must not claim a success value it does not have");
            }
            else if (v[1] == "outcome")
            {
                ++outcomes;
                // An outcome refers to one cell and has no source or dwell.
                NS_TEST_ASSERT_MSG_EQ(v[2], std::string("-"), "outcome rows have no source cell");
                NS_TEST_ASSERT_MSG_EQ(v[4], std::string("-"), "nor a time of stay");
                NS_TEST_ASSERT_MSG_NE(v[5], std::string("-"), "but must carry success");
            }
            else
            {
                NS_TEST_ASSERT_MSG_EQ(true, false, "unknown Event value '" << v[1] << "'");
            }
        }
        NS_TEST_ASSERT_MSG_EQ(rows, 4u, "all four events must be written");
        NS_TEST_ASSERT_MSG_EQ(executed, 2u, "two executions");
        NS_TEST_ASSERT_MSG_EQ(outcomes, 2u, "two outcomes");
    }
};

class ChoUeProjectionUsesEcefFrameTestCase : public TestCase
{
  public:
    ChoUeProjectionUsesEcefFrameTestCase()
        : TestCase("CHO-13: UE projection converts ECEF velocity to local ENU")
    {
    }

  private:
    void DoRun() override
    {
        Ptr<NtnTteEstimator> est = CreateObject<NtnTteEstimator>();

        // A UE at (0 N, 90 E). In ECEF that point lies on the +y axis, so:
        //   local EAST  is -x
        //   local NORTH is +z
        //   local UP    is +y
        const GeoCoordinate ue(0.0, 90.0, 0.0);
        const Time dt = Seconds(100.0);

        // Purely NORTHWARD motion is +z in ECEF here. Latitude must rise and
        // longitude must not move.
        {
            const GeoCoordinate p = est->ProjectUePositionForTest(ue, Vector(0.0, 0.0, 100.0), dt);
            NS_TEST_ASSERT_MSG_GT(p.GetLatitude(), ue.GetLatitude() + 0.05,
                                  "ECEF +z at (0,90) is due north; latitude must increase");
            NS_TEST_ASSERT_MSG_EQ_TOL(p.GetLongitude(), ue.GetLongitude(), 1e-6,
                                      "and longitude must not move");
        }

        // Purely EASTWARD motion is -x in ECEF here. Longitude must rise and
        // latitude must not move. Under the OLD code this vector had x = -100
        // and would have been read as a southward rate, moving the UE in
        // latitude instead.
        {
            const GeoCoordinate p =
                est->ProjectUePositionForTest(ue, Vector(-100.0, 0.0, 0.0), dt);
            NS_TEST_ASSERT_MSG_GT(p.GetLongitude(), ue.GetLongitude() + 0.05,
                                  "ECEF -x at (0,90) is due east; longitude must increase");
            NS_TEST_ASSERT_MSG_EQ_TOL(p.GetLatitude(), ue.GetLatitude(), 1e-6,
                                      "and latitude must not move; the old code read this "
                                      "vector's x as a northward rate and moved it south");
        }

        // Purely UPWARD motion is +y here: altitude only.
        {
            const GeoCoordinate p = est->ProjectUePositionForTest(ue, Vector(0.0, 100.0, 0.0), dt);
            NS_TEST_ASSERT_MSG_EQ_TOL(p.GetLatitude(), ue.GetLatitude(), 1e-6,
                                      "vertical motion must not change latitude");
            NS_TEST_ASSERT_MSG_EQ_TOL(p.GetLongitude(), ue.GetLongitude(), 1e-6,
                                      "nor longitude");
            NS_TEST_ASSERT_MSG_GT(p.GetAltitude(), ue.GetAltitude() + 1.0,
                                  "but must raise the altitude, which the old code discarded");
        }

        // A static UE stays put.
        {
            const GeoCoordinate p = est->ProjectUePositionForTest(ue, Vector(0, 0, 0), dt);
            NS_TEST_ASSERT_MSG_EQ_TOL(p.GetLatitude(), ue.GetLatitude(), 1e-12, "static UE");
            NS_TEST_ASSERT_MSG_EQ_TOL(p.GetLongitude(), ue.GetLongitude(), 1e-12, "static UE");
        }
    }
};

/// CHO-14: a configured trigger that can never fire is worse than an absent one.
///
/// TRIGGER_TIME_BASED matched no dispatch branch, fell into the D1 path, matched
/// none of the four arms there either, and so never set cand.admitted. Since
/// SelectBestCandidate requires admitted, the trigger returned NtnChoAlgorithm::INVALID_CELL_ID
/// forever: the scenario ran, reported no handovers, and looked like a mobility
/// result.
/// CHO-16: condEventT1 is an absolute-epoch window, not a time-to-exit test.
///
/// TS 38.331 condEventT1 enters while the UE's own time lies inside
/// [t1-Threshold, t1-Threshold + duration], with t1-Threshold an ABSOLUTE epoch
/// broadcast in the conditional reconfiguration. TRIGGER_TIME_T1 implemented a
/// serving-cell time-to-exit test under that name: ephemeris-derived and useful,
/// and a different condition wearing a 3GPP label.
class ChoCondEventT1IsAnAbsoluteWindowTestCase : public TestCase
{
  public:
    ChoCondEventT1IsAnAbsoluteWindowTestCase()
        : TestCase("CHO-16: TRIGGER_TIME_T1 evaluates the TS 38.331 absolute-epoch window")
    {
    }

  private:
    void DoRun() override
    {
        Ptr<NtnChoAlgorithm> algo = CreateObject<NtnChoAlgorithm>();
        NtnChoAlgorithm::ChoConfig cfg;
        cfg.triggerType = NtnChoAlgorithm::TRIGGER_TIME_T1;
        cfg.qualityThreshold_dB = 0.0;
        // The window opens at t = 5 s and lasts 3 s, so it is open on [5, 8).
        cfg.t1ThresholdEpoch = Seconds(5.0);
        cfg.t1WindowDuration = Seconds(3.0);
        algo->Configure(cfg);
        algo->SetServingCell(1);
        algo->StartMonitoring(GeoCoordinate(0, 0, 0), Vector(0, 0, 0));
        algo->AddCandidateCell(2, 2, 1);

        auto probe = [algo, this](double t, bool expectOpen) {
            Simulator::Schedule(Seconds(t), [algo, this, t, expectOpen]() {
                algo->UpdateMeasurement(2, 20.0, 30.0);
                algo->EvaluateConditions();
                const bool admitted =
                    (algo->SelectBestCandidate() != NtnChoAlgorithm::INVALID_CELL_ID);
                NS_TEST_ASSERT_MSG_EQ(admitted, expectOpen,
                                      "at t=" << t << " s the condEventT1 window on [5, 8) must be "
                                              << (expectOpen ? "OPEN" : "CLOSED"));
            });
        };

        // Before the epoch: closed. This is the assertion the time-to-exit
        // implementation could not make, because it never looked at the clock.
        probe(1.0, false);
        probe(4.9, false);
        // Inside: open.
        probe(5.1, true);
        probe(7.9, true);
        // Past the duration: closed again. A condition that only tested
        // "after the threshold" would stay open here.
        probe(8.5, false);
        probe(12.0, false);

        Simulator::Stop(Seconds(14.0));
        Simulator::Run();
        Simulator::Destroy();
    }
};

class ChoTimeBasedTriggerFiresTestCase : public TestCase
{
  public:
    ChoTimeBasedTriggerFiresTestCase()
        : TestCase("CHO-14: TRIGGER_TIME_BASED admits a candidate after the beam dwell")
    {
    }

  private:
    static Ptr<NtnChoAlgorithm> MakeAlgo(Time dwell)
    {
        Ptr<NtnChoAlgorithm> algo = CreateObject<NtnChoAlgorithm>();
        NtnChoAlgorithm::ChoConfig cfg;
        cfg.triggerType = NtnChoAlgorithm::TRIGGER_TIME_BASED;
        cfg.beamDwellThreshold = dwell;
        cfg.qualityThreshold_dB = 0.0;
        cfg.beamDwellHysteresis_dB = 1.0;
        algo->Configure(cfg);
        return algo;
    }

    void DoRun() override
    {
        Ptr<NtnChoAlgorithm> algo = MakeAlgo(Seconds(10.0));
        algo->SetServingCell(1);
        algo->StartMonitoring(GeoCoordinate(0, 0, 0), Vector(0, 0, 0));
        // Serving is weak; the candidate is clearly better.
        algo->AddCandidateCell(1, 1, 1);
        algo->UpdateMeasurement(1, 5.0, 30.0);
        algo->AddCandidateCell(2, 2, 1);
        algo->UpdateMeasurement(2, 20.0, 30.0);

        // Before the dwell elapses the trigger must NOT fire. Without this the
        // test would pass on a trigger that admits everything immediately.
        Simulator::Schedule(Seconds(2.0), [algo, this]() {
            algo->EvaluateConditions();
            NS_TEST_ASSERT_MSG_EQ(algo->SelectBestCandidate(), NtnChoAlgorithm::INVALID_CELL_ID,
                                  "2 s into a 10 s dwell the trigger must not admit");
        });
        // After it, the trigger must fire. This is the assertion that was
        // impossible before: it returned NtnChoAlgorithm::INVALID_CELL_ID at every instant.
        Simulator::Schedule(Seconds(15.0), [algo, this]() {
            algo->UpdateMeasurement(2, 20.0, 30.0);
            algo->EvaluateConditions();
            NS_TEST_ASSERT_MSG_EQ(algo->SelectBestCandidate(), 2,
                                  "past the dwell threshold a better candidate must be admitted; "
                                  "this trigger could never fire at all before");
        });
        Simulator::Stop(Seconds(20.0));
        Simulator::Run();
        Simulator::Destroy();

        // And a candidate that is NOT better must still be refused after the
        // dwell, or the trigger is a timer rather than a handover policy.
        Ptr<NtnChoAlgorithm> algo2 = MakeAlgo(Seconds(1.0));
        algo2->SetServingCell(1);
        algo2->StartMonitoring(GeoCoordinate(0, 0, 0), Vector(0, 0, 0));
        algo2->AddCandidateCell(1, 1, 1);
        algo2->UpdateMeasurement(1, 20.0, 30.0);
        algo2->AddCandidateCell(2, 2, 1);
        Simulator::Schedule(Seconds(5.0), [algo2, this]() {
            algo2->UpdateMeasurement(2, 20.2, 30.0); // inside the 1 dB margin
            algo2->EvaluateConditions();
            NS_TEST_ASSERT_MSG_EQ(algo2->SelectBestCandidate(), NtnChoAlgorithm::INVALID_CELL_ID,
                                  "a candidate inside the hysteresis margin must be refused even "
                                  "after the dwell; otherwise the UE churns around a ring of "
                                  "equally poor beams every time the timer expires");
        });
        Simulator::Stop(Seconds(8.0));
        Simulator::Run();
        Simulator::Destroy();
    }
};

class ChoExecutionCountEqualsActuationTestCase : public TestCase
{
  public:
    ChoExecutionCountEqualsActuationTestCase()
        : TestCase("WF-08 gate 6: CHO executions equal actuations, not merely >= 1")
    {
    }

  private:
    uint32_t m_actuations{0};
    uint16_t m_lastTarget{0};

    void OnExecute(uint16_t /*source*/, uint16_t target)
    {
        ++m_actuations;
        m_lastTarget = target;
    }

    void DoRun() override
    {
        Ptr<NtnChoAlgorithm> algo = CreateObject<NtnChoAlgorithm>();
        NtnChoAlgorithm::ChoConfig cfg;
        cfg.triggerType = NtnChoAlgorithm::TRIGGER_TTE_AWARE;
        cfg.rachLess = true;
        algo->Configure(cfg);
        algo->SetHandoverExecutionCallback(
            MakeCallback(&ChoExecutionCountEqualsActuationTestCase::OnExecute, this));

        const uint16_t cells[] = {11, 12, 13, 14};
        for (uint16_t c : cells)
        {
            algo->AddCandidateCell(c, 0, 0);
            algo->UpdateCandidateSlantRange(c, 900e3);
        }

        uint32_t requested = 0;
        for (uint16_t c : cells)
        {
            algo->ExecuteHandover(c);
            ++requested;
            // Close the loop the way a real stack does: the RRC reports
            // completion, which re-arms the remaining candidates.
            algo->NotifyHandoverComplete(c, true);
        }

        const auto st = algo->GetMechanismStats();
        const uint32_t executed = st.rachLessExecutions + st.rachExecutions;

        NS_TEST_ASSERT_MSG_EQ(requested, 4u, "four handovers were requested");
        NS_TEST_ASSERT_MSG_EQ(executed, requested,
                              "the model's own execution counters must account for every request; "
                              "a shortfall means an execution was dropped between the decision "
                              "and the accounting");
        NS_TEST_ASSERT_MSG_EQ(m_actuations, requested,
                              "and the ACTUATION callback must fire exactly once per execution. "
                              "Equality is the assertion: the shipped checker tests only "
                              ">= 1, which passes when a decision model requests four handovers "
                              "and the stack performs one");
        NS_TEST_ASSERT_MSG_EQ(m_lastTarget, cells[3],
                              "and the last actuation must name the last target, so the counts "
                              "cannot agree while the targets diverge");
    }
};


/// CHO-12: LTM must have a measurement period of its own.
///
/// EvaluateLtmConditional filtered an L1 moving average computed in
/// UpdateMeasurement from exactly the per-second SINR samples every other
/// trigger consumes. So "L1-filtered low-latency measurements" and the
/// L3-rate measurements were the same samples under two names, and there was no
/// L1 measurement period at all - which is the part of Rel-19 LTM that makes it
/// low-latency in the first place.
class ChoLtmL1CadenceTestCase : public TestCase
{
  public:
    ChoLtmL1CadenceTestCase()
        : TestCase("CHO-12: the LTM L1 filter has its own measurement period")
    {
    }

  private:
    Ptr<NtnChoAlgorithm> m_algo;

    void Feed()
    {
        m_algo->UpdateMeasurement(7, 12.0, -1.0);
    }

    Ptr<NtnChoAlgorithm> Build(Time l1Period)
    {
        Ptr<NtnChoAlgorithm> a = CreateObject<NtnChoAlgorithm>();
        NtnChoAlgorithm::ChoConfig cfg;
        cfg.triggerType = NtnChoAlgorithm::TRIGGER_LTM_CONDITIONAL;
        cfg.ltmL1MeasurementPeriod = l1Period;
        a->Configure(cfg);
        a->AddCandidateCell(7, 0, 0);
        return a;
    }

    void RunFeeds(Ptr<NtnChoAlgorithm> a, uint32_t n, Time spacing)
    {
        m_algo = a;
        for (uint32_t i = 0; i < n; ++i)
        {
            Simulator::Schedule(spacing * (i + 1), &ChoLtmL1CadenceTestCase::Feed, this);
        }
        Simulator::Stop(spacing * (n + 2));
        Simulator::Run();
        Simulator::Destroy();
    }

    void DoRun() override
    {
        // Default (period 0): the L1 filter advances on every report, which is
        // the old behaviour and must be preserved for existing scenarios.
        {
            Ptr<NtnChoAlgorithm> a = Build(Seconds(0));
            RunFeeds(a, 20, MilliSeconds(10));
            NS_TEST_ASSERT_MSG_EQ(a->GetMeasurementReportCount(), 20u, "twenty reports consumed");
            NS_TEST_ASSERT_MSG_EQ(a->GetL1SampleCount(), 20u,
                                  "with no period configured every report advances the L1 "
                                  "filter, exactly as before");
        }

        // With a 50 ms L1 period and reports every 10 ms, the two cadences must
        // DIVERGE. Equal counts here are what "no distinct L1 measurement
        // period" looks like from outside.
        {
            Ptr<NtnChoAlgorithm> a = Build(MilliSeconds(50));
            RunFeeds(a, 20, MilliSeconds(10));
            NS_TEST_ASSERT_MSG_EQ(a->GetMeasurementReportCount(), 20u, "twenty reports consumed");
            NS_TEST_ASSERT_MSG_LT(a->GetL1SampleCount(), 20u,
                                  "a 50 ms L1 period against 10 ms reports must advance the "
                                  "filter fewer than twenty times; equal counts mean the period "
                                  "is being ignored and L1 is still the report cadence");
            NS_TEST_ASSERT_MSG_GT(a->GetL1SampleCount(), 2u,
                                  "but it must still advance - a period that stops the filter "
                                  "entirely is not a cadence");
            // 200 ms of reports at a 50 ms period is about five samples.
            NS_TEST_ASSERT_MSG_LT(a->GetL1SampleCount(), 8u,
                                  "and roughly at the configured rate, not merely below the "
                                  "report count");
        }
    }
};


/// CHO-12: NTN-NTN candidates reference the satellite, not its ground beam.
///
/// The D2 moving reference resolved only to a candidate's GROUND beam centre,
/// so every CondEventD2 evaluation was a ground-cell handover however the
/// scenario was framed, and the Rel-19 satellite-to-satellite case for
/// Earth-moving cells could not be expressed at all. For a spaceborne terminal
/// the distance to a ground footprint is simply a different quantity from the
/// distance to the satellite, and the D2 thresholds are about the latter.
class ChoNtnNtnCandidateReferenceTestCase : public TestCase
{
  public:
    ChoNtnNtnCandidateReferenceTestCase()
        : TestCase("CHO-12: an NTN-NTN candidate's D2 reference is the satellite's ephemeris")
    {
    }

  private:
    void DoRun() override
    {
        // A satellite 700 km up, and a terminal that is ALSO in orbit 100 km
        // below it - the NTN-NTN geometry. The satellite's ground beam centre
        // is ~700 km away from that terminal in the radial direction; the
        // satellite itself is ~100 km away. The two references differ by almost
        // an order of magnitude, so which one D2 uses is not a detail.
        // Place the satellite off the WGS-84 EQUATORIAL radius, because
        // GeoCoordinate converts the terminal's altitude against that
        // ellipsoid. Using a spherical 6371 km here instead put the two 92.9 km
        // apart rather than 100 km - the model was right and the expectation
        // was wrong, which is worth stating since the discrepancy is exactly
        // the 6378.137 - 6371 difference.
        constexpr double kWgs84EquatorialM = 6378137.0;
        auto satMob = CreateObject<ConstantVelocityMobilityModel>();
        satMob->SetPosition(Vector(kWgs84EquatorialM + 700e3, 0.0, 0.0));
        satMob->SetVelocity(Vector(0.0, 7500.0, 0.0));

        auto pred = CreateObject<NtnOrbitPredictor>();
        pred->SetKinematicsSource(9, satMob);
        pred->SetGeometricBeam(30.0, 4.0);
        pred->SetSteeredBeam(false);

        Ptr<NtnChoAlgorithm> algo = CreateObject<NtnChoAlgorithm>();
        NtnChoAlgorithm::ChoConfig cfg;
        cfg.triggerType = NtnChoAlgorithm::TRIGGER_DISTANCE_D2;
        algo->Configure(cfg);
        algo->SetOrbitPredictor(pred);
        algo->AddCandidateCell(21, 9, 0);

        // The terminal is a spacecraft at 600 km, under the satellite.
        algo->StartMonitoring(GeoCoordinate(0.0, 0.0, 600e3), Vector(0, 0, 0));

        NS_TEST_ASSERT_MSG_EQ(algo->GetCandidateReferenceIsSatellite(21), false,
                              "candidates default to the ground beam centre, so existing "
                              "scenarios are unaffected");
        const double dGround = algo->DistanceToMovingReferenceForTest(21);
        NS_TEST_ASSERT_MSG_GT(dGround, 0.0, "the ground reference resolves");

        NS_TEST_ASSERT_MSG_EQ(algo->SetCandidateReferenceIsSatellite(21, true), true,
                              "the candidate can be marked NTN-NTN");
        NS_TEST_ASSERT_MSG_EQ(algo->GetCandidateReferenceIsSatellite(21), true, "and reports it");
        const double dSat = algo->DistanceToMovingReferenceForTest(21);
        NS_TEST_ASSERT_MSG_GT(dSat, 0.0, "the satellite reference resolves");

        // ~100 km to the satellite against ~700 km to its ground beam centre.
        NS_TEST_ASSERT_MSG_EQ_TOL(dSat, 100e3, 2e3,
                                  "an NTN-NTN candidate's reference is the target SATELLITE, "
                                  "100 km above this terminal");
        NS_TEST_ASSERT_MSG_GT(dGround, 5.0 * dSat,
                              "and it is nowhere near the ground beam centre, which sits ~700 km "
                              "away radially. If the two agreed, the reference kind would be "
                              "decorative");

        // Marking a cell that is not a candidate must fail rather than
        // silently create state.
        NS_TEST_ASSERT_MSG_EQ(algo->SetCandidateReferenceIsSatellite(999, true), false,
                              "an unknown cell cannot be marked");

        // An unknown satellite must be reported, not fall back to the ground
        // beam - falling back would silently measure the wrong thing.
        Ptr<NtnChoAlgorithm> orphan = CreateObject<NtnChoAlgorithm>();
        orphan->Configure(cfg);
        orphan->SetOrbitPredictor(pred);
        orphan->AddCandidateCell(22, 404, 0); // satellite 404 is not registered
        orphan->StartMonitoring(GeoCoordinate(0.0, 0.0, 600e3), Vector(0, 0, 0));
        orphan->SetCandidateReferenceIsSatellite(22, true);
        NS_TEST_ASSERT_MSG_LT(orphan->DistanceToMovingReferenceForTest(22), 0.0,
                              "an unresolvable satellite reference returns -1 rather than "
                              "quietly measuring to a ground beam centre instead");
    }
};

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
        AddTestCase(new NtnGeodeticFixedMobilityTestCase, TestCase::Duration::QUICK);
        AddTestCase(new NtnTteEstimatorComputeTteTestCase, TestCase::Duration::QUICK);
        AddTestCase(new NtnChoStandardizedTriggersTestCase, TestCase::Duration::QUICK);
        AddTestCase(new NtnOrbitPredictorTimeOffsetGainTestCase, TestCase::Duration::QUICK);
        AddTestCase(new NtnOrbitPredictorPropagationAccuracyTestCase, TestCase::Duration::QUICK);
        AddTestCase(new NtnChoTriggersSurviveHandoverTestCase, TestCase::Duration::QUICK);
        AddTestCase(new NtnChoRachLessDrivesTimingAdvanceTestCase, TestCase::Duration::QUICK);
        AddTestCase(new ChoExecutionCountEqualsActuationTestCase, TestCase::Duration::QUICK);
        AddTestCase(new ChoHandoverCsvIsWellFormedTestCase, TestCase::Duration::QUICK);
        AddTestCase(new ChoUeProjectionUsesEcefFrameTestCase, TestCase::Duration::QUICK);
        AddTestCase(new ChoTimeBasedTriggerFiresTestCase, TestCase::Duration::QUICK);
        AddTestCase(new ChoCondEventT1IsAnAbsoluteWindowTestCase,
                    TestCase::Duration::QUICK);
        AddTestCase(new ChoLtmL1CadenceTestCase, TestCase::Duration::QUICK);
        AddTestCase(new ChoNtnNtnCandidateReferenceTestCase, TestCase::Duration::QUICK);
    }
};

static NtnChoTestSuite g_ntnChoTestSuite;
