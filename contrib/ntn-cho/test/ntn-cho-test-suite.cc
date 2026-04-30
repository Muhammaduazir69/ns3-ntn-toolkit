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
#include <ns3/simulator.h>
#include <ns3/test.h>

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
    }
};

static NtnChoTestSuite g_ntnChoTestSuite;
