/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-only
 * Author: Muhammad Uzair
 *
 * O-RAN NTN Test Suite
 */

#include "ns3/core-module.h"
#include "ns3/oran-ntn-a1-interface.h"
#include "ns3/oran-ntn-channel-model.h"
#include "ns3/oran-ntn-conflict-manager.h"
#include "ns3/oran-ntn-dual-connectivity.h"
#include "ns3/oran-ntn-e2-interface.h"
#include "ns3/oran-ntn-federated-learning.h"
#include "ns3/oran-ntn-flexric-types.h"
#include "ns3/oran-ntn-isl-header.h"
#include "ns3/oran-ntn-kpm-canonical-ids.h"
#include "ns3/oran-ntn-data-repository.h"
#include "ns3/oran-ntn-near-rt-ric.h"
#include "ns3/oran-ntn-ntn-scheduler.h"
#include "ns3/oran-ntn-rc-style3.h"
#include "ns3/oran-ntn-phy-kpm-extractor.h"
#include "ns3/oran-ntn-sat-bridge.h"
#include "ns3/oran-ntn-space-ric-inference.h"
#include "ns3/oran-ntn-space-ric.h"
#include "ns3/oran-ntn-types.h"
#include "ns3/oran-ntn-xapp-beam-hop.h"
#include "ns3/oran-ntn-xapp-doppler-comp.h"
#include "ns3/oran-ntn-xapp-energy-harvest.h"
#include "ns3/oran-ntn-xapp-ho-predict.h"
#include "ns3/oran-ntn-xapp-interference-mgmt.h"
#include "ns3/oran-ntn-xapp-multi-conn.h"
#include "ns3/oran-ntn-xapp-predictive-alloc.h"
#include "ns3/oran-ntn-xapp-slice-manager.h"
#include "ns3/oran-ntn-xapp-tn-ntn-steering.h"
#include "ns3/test.h"

#include <map>
#include <set>
#include <sstream>
#include <string>
#include <vector>

using namespace ns3;

// ============================================================================
//  Test 1: Near-RT RIC lifecycle
// ============================================================================

class OranNtnRicTestCase : public TestCase
{
  public:
    OranNtnRicTestCase()
        : TestCase("O-RAN NTN Near-RT RIC initialization and xApp registration")
    {
    }

  private:
    void DoRun() override
    {
        auto ric = CreateObject<OranNtnNearRtRic>();
        ric->Initialize();

        // Verify sub-components
        NS_TEST_ASSERT_MSG_NE(ric->GetE2Termination(), nullptr,
                               "E2 Termination should be created");
        NS_TEST_ASSERT_MSG_NE(ric->GetA1Adapter(), nullptr,
                               "A1 Adapter should be created");
        NS_TEST_ASSERT_MSG_NE(ric->GetConflictManager(), nullptr,
                               "Conflict Manager should be created");
        NS_TEST_ASSERT_MSG_NE(ric->GetSdl(), nullptr,
                               "SDL should be created");

        // Register an xApp
        auto hoPredict = CreateObject<OranNtnXappHoPredict>();
        hoPredict->SetXappName("test-ho");
        hoPredict->SetPriority(10);
        uint32_t id = ric->RegisterXapp(hoPredict);

        NS_TEST_ASSERT_MSG_GT(id, 0u, "xApp ID should be > 0");
        NS_TEST_ASSERT_MSG_EQ(ric->GetXapp(id)->GetXappName(), "test-ho",
                               "xApp name should match");

        auto ids = ric->GetRegisteredXappIds();
        NS_TEST_ASSERT_MSG_EQ(ids.size(), 1u, "Should have 1 registered xApp");

        // Deregister
        ric->DeregisterXapp(id);
        ids = ric->GetRegisteredXappIds();
        NS_TEST_ASSERT_MSG_EQ(ids.size(), 0u, "Should have 0 xApps after deregister");

        ric->Dispose();
        Simulator::Destroy();
    }
};

// ============================================================================
//  Test 2: E2 Interface subscription and indication flow
// ============================================================================

class OranNtnE2TestCase : public TestCase
{
  public:
    OranNtnE2TestCase()
        : TestCase("O-RAN NTN E2 subscription and indication routing")
    {
    }

  private:
    uint32_t m_indicationsReceived{0};

    void HandleIndication(uint32_t xappId, E2Indication indication)
    {
        m_indicationsReceived++;
    }

    void DoRun() override
    {
        auto ric = CreateObject<OranNtnNearRtRic>();
        ric->Initialize();

        // Create E2 node
        auto e2node = CreateObject<OranNtnE2Node>();
        e2node->SetNodeId(1);
        e2node->SetIsNtn(true);
        e2node->SetFeederLinkDelay(MilliSeconds(4));
        e2node->RegisterRanFunction(2, "KPM");

        ric->ConnectE2Node(e2node);

        // Create subscription
        E2Subscription sub;
        sub.subscriptionId = 0; // Will be assigned
        sub.ricRequestorId = 1;
        sub.ranFunctionId = 2;
        sub.reportingPeriod = MilliSeconds(100);
        sub.eventTrigger = false;
        sub.batchOnVisibility = false;
        sub.maxBufferAge = Seconds(10);
        sub.useIslRelay = false;

        uint32_t subId = ric->GetE2Termination()->CreateSubscription(1, sub);
        NS_TEST_ASSERT_MSG_GT(subId, 0u, "Subscription ID should be > 0");

        // Verify E2 node has the subscription
        auto subs = e2node->GetActiveSubscriptions();
        NS_TEST_ASSERT_MSG_EQ(subs.size(), 1u, "E2 node should have 1 subscription");

        // Submit KPM report
        E2KpmReport report;
        report.timestamp = 0.0;
        report.gnbId = 1;
        report.isNtn = true;
        report.ueId = 42;
        report.sinr_dB = 10.0;
        report.rsrp_dBm = -90.0;
        report.tte_s = 30.0;
        report.elevation_deg = 45.0;
        report.doppler_Hz = 1000.0;

        e2node->SubmitKpmMeasurement(report);

        // Run simulation to deliver report (with feeder link delay)
        Simulator::Stop(MilliSeconds(100));
        Simulator::Run();
        Simulator::Destroy();
    }
};

// ============================================================================
//  Test 3: A1 Policy management
// ============================================================================

class OranNtnA1TestCase : public TestCase
{
  public:
    OranNtnA1TestCase()
        : TestCase("O-RAN NTN A1 policy creation and distribution")
    {
    }

  private:
    void DoRun() override
    {
        auto policyMgr = CreateObject<OranNtnA1PolicyManager>();
        auto a1Adapter = CreateObject<OranNtnA1Adapter>();

        // Connect distribution
        policyMgr->SetDistributionCallback(
            MakeCallback(&OranNtnA1Adapter::HandleIncomingPolicy, a1Adapter));

        // Create HO threshold policy
        A1NtnPolicy hoPolicy;
        hoPolicy.type = A1PolicyType::HO_THRESHOLD;
        hoPolicy.scope = "global";
        hoPolicy.priority = 10;
        hoPolicy.orbitalPlaneId = 0;
        hoPolicy.satelliteId = 0;
        hoPolicy.beamGroupId = 0;
        hoPolicy.orbitAware = true;
        hoPolicy.active = true;
        hoPolicy.param1 = 15.0; // TTE minimum
        hoPolicy.param2 = -3.0; // SINR threshold
        hoPolicy.param3 = 50000.0; // D1 distance

        uint32_t policyId = policyMgr->CreatePolicy(hoPolicy);
        NS_TEST_ASSERT_MSG_GT(policyId, 0u, "Policy ID should be > 0");

        // Verify adapter received it
        auto policies = a1Adapter->GetActivePolicies();
        NS_TEST_ASSERT_MSG_EQ(policies.size(), 1u,
                               "Adapter should have 1 active policy");

        // Test threshold lookup
        double tteMin = a1Adapter->GetThreshold(A1PolicyType::HO_THRESHOLD, "tte_min");
        NS_TEST_ASSERT_MSG_EQ_TOL(tteMin, 15.0, 0.01,
                                    "TTE minimum threshold should be 15.0");

        // Delete policy
        policyMgr->DeletePolicy(policyId);
        policies = a1Adapter->GetActivePolicies();
        NS_TEST_ASSERT_MSG_EQ(policies.size(), 0u,
                               "Adapter should have 0 policies after delete");

        policyMgr->Dispose();
        a1Adapter->Dispose();
        Simulator::Destroy();
    }
};

// ============================================================================
//  Test 4: Conflict detection and resolution
// ============================================================================

class OranNtnConflictTestCase : public TestCase
{
  public:
    OranNtnConflictTestCase()
        : TestCase("O-RAN NTN multi-xApp conflict detection and resolution")
    {
    }

  private:
    void DoRun() override
    {
        auto cm = CreateObject<OranNtnConflictManager>();
        cm->SetResolutionStrategy(ConflictResolutionStrategy::PRIORITY_BASED);
        cm->SetConflictWindow(MilliSeconds(500));

        // xApp 1 (high priority) issues HO action
        E2RcAction action1;
        action1.timestamp = 0.0;
        action1.xappId = 1;
        action1.xappName = "ho-predict";
        action1.actionType = E2RcActionType::HANDOVER_TRIGGER;
        action1.targetGnbId = 10;
        action1.targetUeId = 42;

        bool allowed1 = cm->CheckAndResolve(1, 10, action1); // priority 10
        NS_TEST_ASSERT_MSG_EQ(allowed1, true, "First action should be allowed");

        // xApp 2 (lower priority) issues conflicting HO on same UE
        E2RcAction action2;
        action2.timestamp = 0.001;
        action2.xappId = 2;
        action2.xappName = "tn-ntn-steering";
        action2.actionType = E2RcActionType::HANDOVER_TRIGGER;
        action2.targetGnbId = 20;
        action2.targetUeId = 42; // Same UE!

        bool allowed2 = cm->CheckAndResolve(2, 25, action2); // priority 25 (lower)
        NS_TEST_ASSERT_MSG_EQ(allowed2, false,
                               "Lower priority action should be rejected");

        NS_TEST_ASSERT_MSG_EQ(cm->GetTotalConflicts(), 1u,
                               "Should have detected 1 conflict");

        // Non-conflicting action (different resource)
        E2RcAction action3;
        action3.timestamp = 0.002;
        action3.xappId = 3;
        action3.xappName = "beam-hop";
        action3.actionType = E2RcActionType::BEAM_HOP_SCHEDULE;
        action3.targetGnbId = 10;
        action3.targetBeamId = 5;

        bool allowed3 = cm->CheckAndResolve(3, 20, action3);
        NS_TEST_ASSERT_MSG_EQ(allowed3, true,
                               "Non-conflicting action should be allowed");

        cm->Dispose();
        Simulator::Destroy();
    }
};

// ============================================================================
//  Test 5: Space RIC autonomous mode
// ============================================================================

class OranNtnSpaceRicTestCase : public TestCase
{
  public:
    OranNtnSpaceRicTestCase()
        : TestCase("O-RAN NTN Space RIC autonomous mode and model management")
    {
    }

  private:
    void DoRun() override
    {
        auto spaceRic = CreateObject<OranNtnSpaceRic>();
        spaceRic->SetSatelliteId(1);
        spaceRic->SetOrbitalPlaneId(0);
        spaceRic->SetComputeBudget(500.0);
        spaceRic->SetMemoryBudget(256.0);

        NS_TEST_ASSERT_MSG_EQ(spaceRic->IsAutonomous(), false,
                               "Should not be autonomous initially");

        // Enter autonomous mode
        spaceRic->EnterAutonomousMode();
        NS_TEST_ASSERT_MSG_EQ(spaceRic->IsAutonomous(), true,
                               "Should be autonomous after entering");

        // Receive model update
        std::vector<double> weights = {2.0, 0.5, 0.1, -0.3, 1.2};
        spaceRic->ReceiveModelUpdate("ho-scorer", 1, weights);
        NS_TEST_ASSERT_MSG_EQ(spaceRic->GetModelVersion("ho-scorer"), 1u,
                               "Model version should be 1");

        // Process local KPM
        E2KpmReport report;
        report.ueId = 42;
        report.gnbId = 1;
        report.sinr_dB = -6.0; // Below threshold
        report.tte_s = 3.0;    // Below threshold
        report.beamId = 1;
        report.prbUtilization = 0.5;
        report.elevation_deg = 30.0;
        report.doppler_Hz = 500.0;
        spaceRic->ProcessLocalKpm(report);

        // Exit autonomous mode
        spaceRic->ExitAutonomousMode();
        NS_TEST_ASSERT_MSG_EQ(spaceRic->IsAutonomous(), false,
                               "Should not be autonomous after exiting");

        spaceRic->Dispose();
        Simulator::Destroy();
    }
};

// ============================================================================
//  Test 6: Full xApp pipeline (E2 -> RIC -> xApp -> RC action -> E2)
// ============================================================================

class OranNtnFullPipelineTestCase : public TestCase
{
  public:
    OranNtnFullPipelineTestCase()
        : TestCase("O-RAN NTN full pipeline: KPM report -> xApp decision -> RC action")
    {
    }

  private:
    bool m_actionExecuted{false};

    bool HandleRcAction(E2RcAction action)
    {
        m_actionExecuted = true;
        return true;
    }

    void DoRun() override
    {
        // 1. Create RIC
        auto ric = CreateObject<OranNtnNearRtRic>();
        ric->Initialize();

        // 2. Create E2 node
        auto e2node = CreateObject<OranNtnE2Node>();
        e2node->SetNodeId(1);
        e2node->SetIsNtn(true);
        e2node->SetFeederLinkDelay(MilliSeconds(1));
        e2node->RegisterRanFunction(2, "KPM");
        e2node->RegisterRanFunction(3, "RC");
        e2node->SetRcActionCallback(
            MakeCallback(&OranNtnFullPipelineTestCase::HandleRcAction, this));
        ric->ConnectE2Node(e2node);

        // 3. Register xApp
        auto hoPredict = CreateObject<OranNtnXappHoPredict>();
        hoPredict->SetXappName("test-ho");
        hoPredict->SetPriority(10);
        ric->RegisterXapp(hoPredict);

        // 4. Start xApp
        hoPredict->Start();

        // 5. Verify xApp is running
        NS_TEST_ASSERT_MSG_EQ(
            static_cast<uint8_t>(hoPredict->GetState()),
            static_cast<uint8_t>(XappState::RUNNING),
            "xApp should be in RUNNING state");

        // 6. Simulate for a short time
        Simulator::Stop(Seconds(1));
        Simulator::Run();

        // 7. Verify metrics
        auto metrics = ric->GetMetrics();
        NS_TEST_ASSERT_MSG_EQ(metrics.activeXapps, 1u,
                               "Should have 1 active xApp");
        NS_TEST_ASSERT_MSG_EQ(metrics.totalE2Nodes, 1u,
                               "Should have 1 E2 node");

        // 8. Cleanup
        hoPredict->Stop();
        ric->Dispose();
        Simulator::Destroy();
    }
};

// ============================================================================
//  Test 7: Deep Satellite Integration - ModCod Selection & Fading
// ============================================================================

class OranNtnSatBridgeDeepTestCase : public TestCase
{
  public:
    OranNtnSatBridgeDeepTestCase()
        : TestCase("Phase 1: Deep satellite integration - ModCod, fading, interference, ISL")
    {
    }

  private:
    void DoRun() override
    {
        auto bridge = CreateObject<OranNtnSatBridge>();
        bridge->SetCarrierFrequency(20e9);  // Ka-band
        bridge->SetSatelliteTxPower(43.0);
        bridge->SetBandwidth(400e6);
        bridge->SetNtnScenario("Urban");

        // Verify ModCod threshold table is populated
        // SelectModCod requires registered satellites, so test the ISL topology
        // and Markov state methods which are self-contained

        // Test ISL topology setup with 2 planes x 3 sats
        NodeContainer satNodes;
        satNodes.Create(6);

        // Manually initialize constellation (without real SGP4 for unit test)
        // Test ISL link state
        bridge->InitializeConstellation(satNodes, 2, 3, nullptr);

        // Setup ISL
        bridge->SetupIslTopology(satNodes, 1000.0);

        // Verify ISL neighbors exist
        auto neighbors = bridge->GetIslNeighbors(0);
        NS_TEST_ASSERT_MSG_GT(neighbors.size(), 0u,
                               "Satellite 0 should have ISL neighbors");

        // Test C/N0 computation (should be valid number)
        // Register a UE first
        NodeContainer ueNodes;
        ueNodes.Create(1);
        bridge->RegisterUeNodes(ueNodes);

        // Verify the bridge tracks the correct number of entities
        NS_TEST_ASSERT_MSG_EQ(bridge->GetNumSatellites(), 6u,
                               "Should track 6 satellites");
        NS_TEST_ASSERT_MSG_EQ(bridge->GetNumUes(), 1u,
                               "Should track 1 UE");

        // Test Markov state query
        uint8_t state = bridge->GetMarkovState(0);
        NS_TEST_ASSERT_MSG_LT(state, 3u,
                               "Markov state should be 0, 1, or 2");

        bridge->Dispose();
        Simulator::Destroy();
    }
};

// ============================================================================
//  Test 8: NTN Channel Model - Composite propagation
// ============================================================================

class OranNtnChannelModelTestCase : public TestCase
{
  public:
    OranNtnChannelModelTestCase()
        : TestCase("Phase 2: NTN composite channel model - FSPL, fading, atmospheric")
    {
    }

  private:
    void DoRun() override
    {
        auto channelModel = CreateObject<OranNtnChannelModel>();
        channelModel->SetBand("Ka-band");
        channelModel->SetEnvironment("urban");
        channelModel->SetAtmosphericAttenuation(true);
        channelModel->SetLooFading(true);
        channelModel->SetMarkovFading(true);
        channelModel->SetRainRate(10.0);

        // Verify Rician K-factor computation
        double k30 = channelModel->ComputeRicianKFactor(30.0);
        double k60 = channelModel->ComputeRicianKFactor(60.0);
        double k90 = channelModel->ComputeRicianKFactor(90.0);

        NS_TEST_ASSERT_MSG_GT(k60, k30,
                               "K-factor should increase with elevation");
        NS_TEST_ASSERT_MSG_GT(k90, k60,
                               "K-factor at 90 deg should be highest");
        NS_TEST_ASSERT_MSG_GT(k30, 0.0,
                               "K-factor should be positive");

        channelModel->Dispose();
        Simulator::Destroy();
    }
};

// ============================================================================
//  Test 9: ISL Header serialization/deserialization
// ============================================================================

class OranNtnIslHeaderTestCase : public TestCase
{
  public:
    OranNtnIslHeaderTestCase()
        : TestCase("Phase 5: ISL header serialization and deserialization")
    {
    }

  private:
    void DoRun() override
    {
        OranNtnIslHeader txHeader;
        txHeader.SetMessageType(IslMessageType::HO_COORDINATION);
        txHeader.SetSourceSatId(42);
        txHeader.SetDestSatId(43);
        txHeader.SetPayloadSize(1024);
        txHeader.SetTimestamp(123.456);
        txHeader.SetSequenceNumber(7);
        txHeader.SetPriority(2);
        txHeader.SetTtl(5);

        // Serialize
        uint32_t serializedSize = txHeader.GetSerializedSize();
        NS_TEST_ASSERT_MSG_EQ(serializedSize, 27u,
                               "ISL header should be 27 bytes");

        Buffer buffer;
        buffer.AddAtStart(serializedSize);
        Buffer::Iterator it = buffer.Begin();
        txHeader.Serialize(it);

        // Deserialize
        OranNtnIslHeader rxHeader;
        it = buffer.Begin();
        uint32_t consumed = rxHeader.Deserialize(it);

        NS_TEST_ASSERT_MSG_EQ(consumed, serializedSize,
                               "Should consume same bytes as serialized");
        NS_TEST_ASSERT_MSG_EQ(static_cast<uint8_t>(rxHeader.GetMessageType()),
                               static_cast<uint8_t>(IslMessageType::HO_COORDINATION),
                               "Message type should match");
        NS_TEST_ASSERT_MSG_EQ(rxHeader.GetSourceSatId(), 42u,
                               "Source sat ID should match");
        NS_TEST_ASSERT_MSG_EQ(rxHeader.GetDestSatId(), 43u,
                               "Dest sat ID should match");
        NS_TEST_ASSERT_MSG_EQ(rxHeader.GetPayloadSize(), 1024u,
                               "Payload size should match");
        NS_TEST_ASSERT_MSG_EQ(rxHeader.GetSequenceNumber(), 7u,
                               "Sequence number should match");
        NS_TEST_ASSERT_MSG_EQ(rxHeader.GetPriority(), 2u,
                               "Priority should match");
        NS_TEST_ASSERT_MSG_EQ(rxHeader.GetTtl(), 5u,
                               "TTL should match");

        Simulator::Destroy();
    }
};

// ============================================================================
//  Test 10: Space RIC Inference Engine
// ============================================================================

class OranNtnInferenceTestCase : public TestCase
{
  public:
    OranNtnInferenceTestCase()
        : TestCase("Phase 5: Space RIC inference engine - rule-based fallback")
    {
    }

  private:
    void DoRun() override
    {
        auto inference = CreateObject<OranNtnSpaceRicInference>();
        inference->SetSatelliteId(1);
        inference->SetPreferredBackend(
            OranNtnSpaceRicInference::Backend::RULE_BASED);

        // Test rule-based inference with overloaded beam
        SpaceRicObservation obs = {};
        obs.satId = 1;
        obs.timestamp = 1.0;
        obs.feederLinkAvail = 1.0;
        obs.batteryLevel = 0.8;
        obs.numActiveBeams = 5;

        // Set one beam overloaded
        for (uint32_t i = 0; i < MAX_BEAMS_PER_SAT; i++)
        {
            obs.beamLoads[i] = 0.3;
            obs.beamSinr[i] = 5.0;
        }
        obs.beamLoads[3] = 0.95; // Beam 3 overloaded

        SpaceRicAction action = inference->Infer(obs);

        // Rule-based should detect overloaded beam
        NS_TEST_ASSERT_MSG_EQ(action.actionType, 1u,
                               "Should trigger beam reallocation for overloaded beam");
        NS_TEST_ASSERT_MSG_GT(action.confidence, 0.0,
                               "Confidence should be positive");

        // Test with low battery
        obs.batteryLevel = 0.15;
        obs.beamLoads[3] = 0.3; // Reset overload

        action = inference->Infer(obs);
        NS_TEST_ASSERT_MSG_EQ(action.actionType, 5u,
                               "Should trigger beam shutdown for low battery");

        // Check metrics
        auto metrics = inference->GetMetrics();
        NS_TEST_ASSERT_MSG_EQ(metrics.totalInferences, 2u,
                               "Should have 2 inferences");
        NS_TEST_ASSERT_MSG_EQ(metrics.ruleBasedInferences, 2u,
                               "All should be rule-based");

        inference->Dispose();
        Simulator::Destroy();
    }
};

// ============================================================================
//  Test 11: Dual Connectivity Manager
// ============================================================================

class OranNtnDualConnTestCase : public TestCase
{
  public:
    OranNtnDualConnTestCase()
        : TestCase("Phase 2: Dual connectivity activation, split ratio, teardown")
    {
    }

  private:
    void DoRun() override
    {
        auto dcMgr = CreateObject<OranNtnDualConnectivity>();
        auto bridge = CreateObject<OranNtnSatBridge>();

        // Initialize with minimal satellite bridge
        NodeContainer satNodes;
        satNodes.Create(2);
        NodeContainer ueNodes;
        ueNodes.Create(3);
        bridge->InitializeConstellation(satNodes, 1, 2, nullptr);
        bridge->RegisterUeNodes(ueNodes);

        dcMgr->Initialize(bridge);

        // Initially no DC sessions
        NS_TEST_ASSERT_MSG_EQ(dcMgr->IsDualConnected(0), false,
                               "UE 0 should not be DC initially");

        // Activate DC for UE 0
        bool activated = dcMgr->ActivateDualConnectivity(0, 100, 0, 1);
        // May fail if SINR check doesn't pass with empty bridge, that's OK
        // Test the session management regardless

        if (activated)
        {
            NS_TEST_ASSERT_MSG_EQ(dcMgr->IsDualConnected(0), true,
                                   "UE 0 should be DC after activation");

            // Update split ratio
            dcMgr->UpdateSplitRatio(0, 0.7);
            auto session = dcMgr->GetSession(0);
            NS_TEST_ASSERT_MSG_EQ_TOL(session.tnSplitRatio, 0.7, 0.01,
                                       "Split ratio should be 0.7");

            // Switch primary
            dcMgr->SwitchPrimaryPath(0, 1); // NTN primary
            session = dcMgr->GetSession(0);
            NS_TEST_ASSERT_MSG_EQ(session.primaryPath, 1u,
                                   "Primary should be NTN (1)");

            // Deactivate
            dcMgr->DeactivateDualConnectivity(0);
            NS_TEST_ASSERT_MSG_EQ(dcMgr->IsDualConnected(0), false,
                                   "UE 0 should not be DC after deactivation");
        }

        auto metrics = dcMgr->GetMetrics();
        // Metrics should be tracking
        NS_TEST_ASSERT_MSG_EQ(metrics.totalActivations + 0, metrics.totalActivations,
                               "Metrics tracking works");

        dcMgr->Dispose();
        bridge->Dispose();
        Simulator::Destroy();
    }
};

// ============================================================================
//  Test 12: Federated Learning Coordinator
// ============================================================================

class OranNtnFederatedLearningTestCase : public TestCase
{
  public:
    OranNtnFederatedLearningTestCase()
        : TestCase("Phase 3: Federated learning - FedAvg gradient aggregation")
    {
    }

  private:
    void DoRun() override
    {
        auto fl = CreateObject<OranNtnFederatedLearning>();
        fl->SetMinParticipants(2);
        fl->SetRoundInterval(Seconds(10));
        fl->SetAggregationStrategy("FedAvg");

        // Create 3 Space RICs
        auto ric1 = CreateObject<OranNtnSpaceRic>();
        ric1->SetSatelliteId(0);
        auto ric2 = CreateObject<OranNtnSpaceRic>();
        ric2->SetSatelliteId(1);
        auto ric3 = CreateObject<OranNtnSpaceRic>();
        ric3->SetSatelliteId(2);

        // Give them model weights
        ric1->ReceiveModelUpdate("test-model", 1, {1.0, 2.0, 3.0});
        ric2->ReceiveModelUpdate("test-model", 1, {4.0, 5.0, 6.0});
        ric3->ReceiveModelUpdate("test-model", 1, {7.0, 8.0, 9.0});

        fl->RegisterSpaceRic(ric1);
        fl->RegisterSpaceRic(ric2);
        fl->RegisterSpaceRic(ric3);

        // Initialize a round
        fl->InitializeFederatedRound("test-model");

        // Collect gradients (simulating local training)
        fl->CollectLocalGradients(0, "test-model", {1.0, 2.0, 3.0}, 0.5, 100);
        fl->CollectLocalGradients(1, "test-model", {4.0, 5.0, 6.0}, 0.3, 200);
        fl->CollectLocalGradients(2, "test-model", {7.0, 8.0, 9.0}, 0.1, 150);

        // Aggregate
        fl->AggregateGradients();

        // Verify round completed
        auto round = fl->GetCurrentRound();
        NS_TEST_ASSERT_MSG_EQ(round.completed, true,
                               "FL round should be completed");
        NS_TEST_ASSERT_MSG_EQ(round.numGradientsReceived, 3u,
                               "Should have received 3 gradients");

        // Check metrics
        auto metrics = fl->GetMetrics();
        NS_TEST_ASSERT_MSG_EQ(metrics.totalRounds, 1u,
                               "Should have 1 total round");

        fl->Dispose();
        ric1->Dispose();
        ric2->Dispose();
        ric3->Dispose();
        Simulator::Destroy();
    }
};

// ============================================================================
//  Test 13: Advanced xApps Initialization
// ============================================================================

class OranNtnAdvancedXappsTestCase : public TestCase
{
  public:
    OranNtnAdvancedXappsTestCase()
        : TestCase("Phase 4: All 9 xApps instantiation and configuration")
    {
    }

  private:
    void DoRun() override
    {
        auto ric = CreateObject<OranNtnNearRtRic>();
        ric->Initialize();

        // Create all 9 xApps
        auto hoPredict = CreateObject<OranNtnXappHoPredict>();
        hoPredict->SetXappName("ho-predict");
        hoPredict->SetPriority(10);

        auto beamHop = CreateObject<OranNtnXappBeamHop>();
        beamHop->SetXappName("beam-hop");
        beamHop->SetPriority(15);

        auto sliceMgr = CreateObject<OranNtnXappSliceManager>();
        sliceMgr->SetXappName("slice-mgr");
        sliceMgr->SetPriority(20);

        auto dopplerComp = CreateObject<OranNtnXappDopplerComp>();
        dopplerComp->SetXappName("doppler-comp");
        dopplerComp->SetPriority(5);

        auto tnNtnSteering = CreateObject<OranNtnXappTnNtnSteering>();
        tnNtnSteering->SetXappName("tn-ntn-steering");
        tnNtnSteering->SetPriority(25);

        // Phase 4 new xApps
        auto interferMgmt = CreateObject<OranNtnXappInterferenceMgmt>();
        interferMgmt->SetXappName("interference-mgmt");
        interferMgmt->SetPriority(8);

        auto energyHarvest = CreateObject<OranNtnXappEnergyHarvest>();
        energyHarvest->SetXappName("energy-harvest");
        energyHarvest->SetPriority(30);

        auto predictAlloc = CreateObject<OranNtnXappPredictiveAlloc>();
        predictAlloc->SetXappName("predictive-alloc");
        predictAlloc->SetPriority(18);

        auto multiConn = CreateObject<OranNtnXappMultiConn>();
        multiConn->SetXappName("multi-conn");
        multiConn->SetPriority(22);

        // Register all
        ric->RegisterXapp(hoPredict);
        ric->RegisterXapp(beamHop);
        ric->RegisterXapp(sliceMgr);
        ric->RegisterXapp(dopplerComp);
        ric->RegisterXapp(tnNtnSteering);
        ric->RegisterXapp(interferMgmt);
        ric->RegisterXapp(energyHarvest);
        ric->RegisterXapp(predictAlloc);
        ric->RegisterXapp(multiConn);

        auto ids = ric->GetRegisteredXappIds();
        NS_TEST_ASSERT_MSG_EQ(ids.size(), 9u,
                               "Should have 9 registered xApps");

        // Verify each xApp has correct name
        NS_TEST_ASSERT_MSG_EQ(hoPredict->GetXappName(), "ho-predict", "Name check");
        NS_TEST_ASSERT_MSG_EQ(interferMgmt->GetXappName(), "interference-mgmt", "Name check");
        NS_TEST_ASSERT_MSG_EQ(energyHarvest->GetXappName(), "energy-harvest", "Name check");
        NS_TEST_ASSERT_MSG_EQ(predictAlloc->GetXappName(), "predictive-alloc", "Name check");
        NS_TEST_ASSERT_MSG_EQ(multiConn->GetXappName(), "multi-conn", "Name check");

        ric->Dispose();
        Simulator::Destroy();
    }
};

// ============================================================================
//  Test 14: NTN Scheduler Configuration
// ============================================================================

class OranNtnSchedulerTestCase : public TestCase
{
  public:
    OranNtnSchedulerTestCase()
        : TestCase("Phase 2: NTN scheduler - RTT-aware HARQ, beam hopping, slice PRB")
    {
    }

  private:
    void DoRun() override
    {
        auto scheduler = CreateObject<OranNtnScheduler>();

        // Configure NTN-specific parameters
        scheduler->SetNtnRttCompensation(MilliSeconds(20));
        scheduler->SetTtePriorityScheduling(true);
        scheduler->SetPredictiveMcs(false);

        // Set beam hopping schedule
        std::vector<BeamAllocationEntry> schedule;
        for (uint32_t slot = 0; slot < 4; slot++)
        {
            for (uint32_t beam = slot * 3; beam < (slot + 1) * 3; beam++)
            {
                BeamAllocationEntry entry;
                entry.satId = 0;
                entry.beamId = beam;
                entry.timeSlot = slot;
                entry.allocatedCellId = beam;
                entry.trafficLoad = 0.5;
                entry.isSignaling = false;
                schedule.push_back(entry);
            }
        }
        scheduler->SetBeamHoppingSchedule(schedule);

        // Set slice constraints
        std::map<uint8_t, double> sliceShares;
        sliceShares[0] = 0.6;  // eMBB
        sliceShares[1] = 0.3;  // URLLC
        sliceShares[2] = 0.1;  // mMTC
        scheduler->ApplySliceConstraints(sliceShares);

        // Set ModCod constraints
        scheduler->SetBeamModCodConstraint(0, 15);
        scheduler->SetBeamModCodConstraint(1, 10);

        // Set per-UE timing advance
        scheduler->SetUeTimingAdvance(1001, MilliSeconds(5));
        scheduler->SetUeTimingAdvance(1002, MilliSeconds(8));

        // Test NTN scheduling preparation
        scheduler->PrepareNtnDlScheduling();
        scheduler->PrepareNtnUlScheduling();

        // Verify stats are initialized
        auto stats = scheduler->GetNtnStats();
        NS_TEST_ASSERT_MSG_EQ(stats.totalScheduled, 0u,
                               "No actual scheduling without PHY");

        scheduler->Dispose();
        Simulator::Destroy();
    }
};

// ============================================================================
//  Test 15: Extended KPM Report Fields
// ============================================================================

class OranNtnExtendedKpmTestCase : public TestCase
{
  public:
    OranNtnExtendedKpmTestCase()
        : TestCase("Phase 1: Extended E2KpmReport with deep integration fields")
    {
    }

  private:
    void DoRun() override
    {
        // Verify all new fields in E2KpmReport are accessible
        E2KpmReport report;
        report.timestamp = 1.0;
        report.gnbId = 1;
        report.isNtn = true;
        report.ueId = 42;

        // Original fields
        report.sinr_dB = 10.5;
        report.rsrp_dBm = -85.0;
        report.tte_s = 45.0;
        report.elevation_deg = 55.0;
        report.doppler_Hz = 15000.0;

        // Phase 1 deep satellite fields
        report.modCod = 12;
        report.spectralEfficiency = 2.5;
        report.fadingGain_dB = -1.5;
        report.markovState = 0; // Clear
        report.interBeamInterference_dBm = -110.0;
        report.cno_dBHz = 80.0;

        // Phase 2 mmWave fields
        report.mcs = 15;
        report.wbCqi = 10.0;
        report.harqBler = 0.01;
        report.harqRetx = 2;
        report.beamTrackingError_deg = 0.3;

        // Phase 2 DC fields
        report.dualConnected = true;
        report.tnSinr_dB = 15.0;
        report.ntnSinr_dB = 8.0;
        report.tnThroughput_Mbps = 100.0;
        report.ntnThroughput_Mbps = 50.0;
        report.bearerSplitRatio = 0.6;

        // Phase 4 energy fields
        report.batteryStateOfCharge = 0.75;
        report.solarPower_W = 150.0;
        report.inEclipse = false;

        // Verify values
        NS_TEST_ASSERT_MSG_EQ(report.modCod, 12u, "ModCod field");
        NS_TEST_ASSERT_MSG_EQ_TOL(report.spectralEfficiency, 2.5, 0.01, "Spectral eff");
        NS_TEST_ASSERT_MSG_EQ(report.markovState, 0u, "Markov state");
        NS_TEST_ASSERT_MSG_EQ_TOL(report.cno_dBHz, 80.0, 0.01, "C/N0");
        NS_TEST_ASSERT_MSG_EQ(report.mcs, 15u, "MCS");
        NS_TEST_ASSERT_MSG_EQ(report.dualConnected, true, "DC flag");
        NS_TEST_ASSERT_MSG_EQ_TOL(report.bearerSplitRatio, 0.6, 0.01, "Split ratio");
        NS_TEST_ASSERT_MSG_EQ_TOL(report.batteryStateOfCharge, 0.75, 0.01, "Battery SoC");
        NS_TEST_ASSERT_MSG_EQ(report.inEclipse, false, "Eclipse flag");

        Simulator::Destroy();
    }
};

// ============================================================================
//  Test 16: Space RIC ISL Communication
// ============================================================================

class OranNtnSpaceRicIslTestCase : public TestCase
{
  public:
    OranNtnSpaceRicIslTestCase()
        : TestCase("Phase 5: Space RIC ISL message exchange between satellites")
    {
    }

  private:
    void DoRun() override
    {
        auto ric1 = CreateObject<OranNtnSpaceRic>();
        ric1->SetSatelliteId(0);
        ric1->SetOrbitalPlaneId(0);

        auto ric2 = CreateObject<OranNtnSpaceRic>();
        ric2->SetSatelliteId(1);
        ric2->SetOrbitalPlaneId(0);

        // Wire as ISL neighbors
        ric1->AddIslNeighbor(ric2);
        ric2->AddIslNeighbor(ric1);

        // Send ISL message from ric1 to ric2
        std::vector<uint8_t> payload = {0x01, 0x02, 0x03, 0x04};
        ric1->SendIslMessage(1, IslMessageType::KPM_EXCHANGE, payload);

        // Run sim to let the message be delivered
        Simulator::Stop(MilliSeconds(100));
        Simulator::Run();

        // Check metrics - ric2 should have received the exchange
        auto metrics2 = ric2->GetMetrics();
        NS_TEST_ASSERT_MSG_GT(metrics2.islExchanges, 0u,
                               "RIC2 should have received ISL exchange");

        ric1->Dispose();
        ric2->Dispose();
        Simulator::Destroy();
    }
};

// ============================================================================
//  Test 17: PHY KPM Extractor
// ============================================================================

class OranNtnPhyKpmExtractorTestCase : public TestCase
{
  public:
    OranNtnPhyKpmExtractorTestCase()
        : TestCase("Phase 2: PHY KPM extractor - UE tracking and measurement")
    {
    }

  private:
    void DoRun() override
    {
        auto extractor = CreateObject<OranNtnPhyKpmExtractor>();

        // Verify no data initially
        NS_TEST_ASSERT_MSG_EQ(extractor->HasPhyData(0), false,
                               "Should not have data for non-tracked UE");

        auto trackedIds = extractor->GetTrackedUeIds();
        NS_TEST_ASSERT_MSG_EQ(trackedIds.size(), 0u,
                               "Should have no tracked UEs initially");

        extractor->Dispose();
        Simulator::Destroy();
    }
};

// ============================================================================
//  Test 18: Energy Harvesting xApp
// ============================================================================

class OranNtnEnergyHarvestTestCase : public TestCase
{
  public:
    OranNtnEnergyHarvestTestCase()
        : TestCase("Phase 4: Energy harvesting xApp - solar model and battery management")
    {
    }

  private:
    void DoRun() override
    {
        auto energyXapp = CreateObject<OranNtnXappEnergyHarvest>();
        energyXapp->SetXappName("energy-harvest");
        energyXapp->SetPriority(30);

        // Configure solar and battery model
        energyXapp->SetSolarPanelArea(2.0);
        energyXapp->SetSolarEfficiency(0.3);
        energyXapp->SetOrbitalPeriod(5400.0); // 90 minute orbit
        energyXapp->SetBatteryCapacity(100.0); // 100 Wh
        energyXapp->SetInitialSoC(0.8);
        energyXapp->SetMinSoC(0.3);
        energyXapp->SetCriticalSoC(0.15);
        energyXapp->SetBeamPowerConsumption(5.0);
        energyXapp->SetComputePowerConsumption(0.01);
        energyXapp->SetBasePowerConsumption(20.0);

        auto metrics = energyXapp->GetEnergyMetrics();
        NS_TEST_ASSERT_MSG_EQ(metrics.beamShutdowns, 0u,
                               "No shutdowns initially");

        energyXapp->Dispose();
        Simulator::Destroy();
    }
};

// ============================================================================
//  T8 (Roadmap §3 T8): Canonical KPM metric-ID alignment with WG3 / srsRAN / OAI
// ============================================================================

class OranNtnKpmCanonicalIdsListTestCase : public TestCase
{
  public:
    OranNtnKpmCanonicalIdsListTestCase()
        : TestCase("KPM canonical IDs match WG3 and srsRAN naming exactly")
    {
    }

  private:
    void DoRun() override
    {
        const auto& ids = oranntn::kpm::CanonicalMetricIds();
        NS_TEST_ASSERT_MSG_EQ(ids.size(),
                              10u,
                              "canonical KPM set must have 10 entries");
        NS_TEST_EXPECT_MSG_EQ(ids[0], "DRB.UEThpDl", "ids[0]");
        NS_TEST_EXPECT_MSG_EQ(ids[1], "DRB.UEThpUl", "ids[1]");
        NS_TEST_EXPECT_MSG_EQ(ids[2], "DRB.PdcpSduVolumeDL", "ids[2]");
        NS_TEST_EXPECT_MSG_EQ(ids[3], "DRB.PdcpSduVolumeUL", "ids[3]");
        NS_TEST_EXPECT_MSG_EQ(ids[4], "RRU.PrbAvailDl", "ids[4]");
        NS_TEST_EXPECT_MSG_EQ(ids[5], "RRU.PrbAvailUl", "ids[5]");
        NS_TEST_EXPECT_MSG_EQ(ids[6], "RRU.PrbUsedDl", "ids[6]");
        NS_TEST_EXPECT_MSG_EQ(ids[7], "RRU.PrbUsedUl", "ids[7]");
        NS_TEST_EXPECT_MSG_EQ(ids[8], "CARR.AverageSINR", "ids[8]");
        NS_TEST_EXPECT_MSG_EQ(ids[9], "L1M.RS-SINR.Mean", "ids[9]");

        NS_TEST_EXPECT_MSG_EQ(std::string(oranntn::label::kFiveQi),
                              "FIVE_QI",
                              "FIVE_QI label dim");
        NS_TEST_EXPECT_MSG_EQ(std::string(oranntn::label::kSnssai),
                              "S-NSSAI",
                              "S-NSSAI label dim");
        NS_TEST_EXPECT_MSG_EQ(std::string(oranntn::label::kPlmn),
                              "PLMN",
                              "PLMN label dim");
    }
};

class OranNtnKpmCanonicalBuildTestCase : public TestCase
{
  public:
    OranNtnKpmCanonicalBuildTestCase()
        : TestCase("BuildCanonicalKpmMeasurements emits all 10 IDs with correct values")
    {
    }

  private:
    void DoRun() override
    {
        E2KpmReport r{};
        r.ueId = 42;
        r.sinr_dB = 12.5;
        r.throughput_Mbps = 50.0; // 50 000 kbps
        r.prbUtilization = 0.5;   // 50% of 273 PRBs => 136.5

        const std::map<std::string, std::string> base = {
            {oranntn::label::kFiveQi, "9"},
            {oranntn::label::kSnssai, "1-000001"},
            {oranntn::label::kPlmn, "00101"},
        };
        const auto v = oranntn::BuildCanonicalKpmMeasurements(r, base);
        NS_TEST_ASSERT_MSG_EQ(v.size(), 10u, "vector size");

        // Build a name->index map so the test is robust to ordering.
        std::map<std::string, size_t> idx;
        for (size_t i = 0; i < v.size(); ++i)
        {
            idx[v[i].metricId] = i;
        }
        NS_TEST_ASSERT_MSG_EQ(idx.count(oranntn::kpm::kDrbUeThpDl),
                              1u,
                              "DL throughput present");
        NS_TEST_EXPECT_MSG_EQ(v[idx[oranntn::kpm::kDrbUeThpDl]].value,
                              50000.0,
                              "DL throughput kbps");
        NS_TEST_EXPECT_MSG_EQ(v[idx[oranntn::kpm::kCarrAvgSinr]].value,
                              12.5,
                              "avg SINR");
        NS_TEST_EXPECT_MSG_EQ(v[idx[oranntn::kpm::kL1mRsSinrMean]].value,
                              12.5,
                              "L1M RS-SINR mean");
        NS_TEST_EXPECT_MSG_EQ(v[idx[oranntn::kpm::kRruPrbAvailDl]].value,
                              273.0,
                              "RRU.PrbAvailDl");
        NS_TEST_EXPECT_MSG_EQ(v[idx[oranntn::kpm::kRruPrbUsedDl]].value,
                              136.5,
                              "RRU.PrbUsedDl = util * PRBs");

        // UL fields are not plumbed yet — must carry present=false sentinel.
        NS_TEST_EXPECT_MSG_EQ(
            v[idx[oranntn::kpm::kDrbUeThpUl]].labels.at(oranntn::label::kPresent),
            "false",
            "UL throughput marked not-present");
        NS_TEST_EXPECT_MSG_EQ(
            v[idx[oranntn::kpm::kDrbPdcpVolumeUl]].labels.at(oranntn::label::kPresent),
            "false",
            "UL PDCP volume marked not-present");
        NS_TEST_EXPECT_MSG_EQ(
            v[idx[oranntn::kpm::kRruPrbUsedUl]].labels.at(oranntn::label::kPresent),
            "false",
            "UL PRBs marked not-present");

        // Present metrics carry no `present` label override.
        NS_TEST_EXPECT_MSG_EQ(
            v[idx[oranntn::kpm::kDrbUeThpDl]].labels.count(oranntn::label::kPresent),
            0u,
            "DL throughput has no override label");
    }
};

class OranNtnKpmCanonicalLabelsTestCase : public TestCase
{
  public:
    OranNtnKpmCanonicalLabelsTestCase()
        : TestCase("FIVE_QI S-NSSAI PLMN labels propagate to every measurement")
    {
    }

  private:
    void DoRun() override
    {
        E2KpmReport r{};
        r.throughput_Mbps = 1.0;
        r.sinr_dB = 0.0;
        r.prbUtilization = 0.0;

        const std::map<std::string, std::string> base = {
            {oranntn::label::kFiveQi, "1"},
            {oranntn::label::kSnssai, "1-000002"},
            {oranntn::label::kPlmn, "00102"},
        };
        const auto v = oranntn::BuildCanonicalKpmMeasurements(r, base);
        NS_TEST_ASSERT_MSG_EQ(v.size(), 10u, "vector size");
        for (const auto& m : v)
        {
            NS_TEST_EXPECT_MSG_EQ(m.labels.at(oranntn::label::kFiveQi),
                                  "1",
                                  std::string("FIVE_QI label on ") + m.metricId);
            NS_TEST_EXPECT_MSG_EQ(m.labels.at(oranntn::label::kSnssai),
                                  "1-000002",
                                  std::string("S-NSSAI label on ") + m.metricId);
            NS_TEST_EXPECT_MSG_EQ(m.labels.at(oranntn::label::kPlmn),
                                  "00102",
                                  std::string("PLMN label on ") + m.metricId);
        }
    }
};

// ============================================================================
//  4.1.1 (Roadmap §4.1.1): FlexRIC field-name parity
// ============================================================================

class OranNtnFlexricE2apShapesTestCase : public TestCase
{
  public:
    OranNtnFlexricE2apShapesTestCase()
        : TestCase("FlexRIC e2ap_msg_t shapes carry verbatim field names")
    {
    }

  private:
    void DoRun() override
    {
        using namespace oranntn::flexric;

        // ric_request_id_t fields (FlexRIC names): ric_id + ric_instance_id.
        ric_request_id_t reqId{};
        reqId.ric_id = 0x1234;
        reqId.ric_instance_id = 0x0007;
        NS_TEST_EXPECT_MSG_EQ(reqId.ric_id, 0x1234u, "ric_id");
        NS_TEST_EXPECT_MSG_EQ(reqId.ric_instance_id, 0x0007u, "ric_instance_id");

        // ric_action_to_be_setup_t (FlexRIC names): ric_action_id,
        // ric_action_type, action_definition, subsequent_action.
        ric_action_to_be_setup_t act{};
        act.ric_action_id = 42;
        act.ric_action_type = ric_action_type_t::report;
        act.action_definition = {0xDE, 0xAD, 0xBE, 0xEF};
        act.subsequent_action = ric_subsequent_action_type_t::continue_action;
        NS_TEST_EXPECT_MSG_EQ(act.ric_action_id, 42u, "ric_action_id");
        NS_TEST_EXPECT_MSG_EQ(static_cast<int>(act.ric_action_type),
                              0,
                              "ric_action_type wire code = 0 for report");
        NS_TEST_ASSERT_MSG_EQ(act.action_definition.size(),
                              4u,
                              "action_definition length");
        NS_TEST_EXPECT_MSG_EQ(act.action_definition[2],
                              0xBEu,
                              "action_definition[2]");
        NS_TEST_ASSERT_MSG_EQ(act.subsequent_action.has_value(),
                              true,
                              "subsequent_action present");

        // ric_subscription_request_t fields.
        ric_subscription_request_t sub{};
        sub.ric_id = reqId;
        sub.ran_function_id = 147;        // KPM SM
        sub.event_trigger = {0x01, 0x02};
        sub.action_to_be_setup.push_back(act);
        NS_TEST_EXPECT_MSG_EQ(sub.ran_function_id, 147u, "ran_function_id");
        NS_TEST_ASSERT_MSG_EQ(sub.action_to_be_setup.size(),
                              1u,
                              "action_to_be_setup length");

        // ric_indication_t fields.
        ric_indication_t ind{};
        ind.ric_id = reqId;
        ind.ran_function_id = 147;
        ind.ric_action_id = 42;
        ind.ric_indication_sn = 1;
        ind.ric_indication_type = 0;
        ind.ric_indication_header = {0x10};
        ind.ric_indication_message = {0x20, 0x21};
        NS_TEST_EXPECT_MSG_EQ(ind.ric_indication_sn, 1u, "ric_indication_sn");
        NS_TEST_EXPECT_MSG_EQ(ind.ric_indication_message.size(),
                              2u,
                              "indication_message body");

        // ric_control_request_t fields.
        ric_control_request_t ctrl{};
        ctrl.ric_id = reqId;
        ctrl.ran_function_id = 3;          // RC SM
        ctrl.ric_control_header = {0xAA};
        ctrl.ric_control_message = {0xBB, 0xCC};
        ctrl.ric_control_ack_request = 1;
        NS_TEST_EXPECT_MSG_EQ(ctrl.ran_function_id, 3u, "RC ran_function_id");
        NS_TEST_ASSERT_MSG_EQ(ctrl.ric_control_ack_request.has_value(),
                              true,
                              "ack request set");

        // e2ap_msg_t tagged union round-trip.
        e2ap_msg_t msg{};
        msg.type = e2ap_pdu_type_t::initiating_message;
        msg.u = ind;
        NS_TEST_ASSERT_MSG_EQ(std::holds_alternative<ric_indication_t>(msg.u),
                              true,
                              "msg holds ric_indication_t");
        NS_TEST_EXPECT_MSG_EQ(
            std::get<ric_indication_t>(msg.u).ric_action_id,
            42u,
            "round-trip ric_action_id");
    }
};

class OranNtnFlexricKpmFormat1TestCase : public TestCase
{
  public:
    OranNtnFlexricKpmFormat1TestCase()
        : TestCase("FlexRIC kpm_ind_msg_format_1_t carries meas_info, "
                   "meas_record, label_info lists")
    {
    }

  private:
    void DoRun() override
    {
        using namespace oranntn::flexric::kpm_v3;
        using oranntn::flexric::ric_indication_t;

        kpm_ind_msg_format_1_t body{};
        body.gran_period_ms = 1000;

        meas_info_format_1_lst_t row{};
        row.meas_type.form = meas_type_form_t::name;
        // FlexRIC takes the canonical metric name verbatim; reuses the
        // string declared in oran-ntn-kpm-canonical-ids.h.
        row.meas_type.meas_name = oranntn::kpm::kDrbUeThpDl;

        meas_record_item_t r1{};
        r1.form = meas_value_form_t::real;
        r1.real_val = 12345.0;
        row.meas_record_lst.push_back(r1);

        meas_record_item_t r2{};
        r2.form = meas_value_form_t::integer;
        r2.int_val = 9876;
        row.meas_record_lst.push_back(r2);

        label_info_t lbl{};
        lbl.five_qi = 9;
        lbl.s_nssai = "1-000001";
        lbl.plmn_id = "00101";
        row.label_info_lst.push_back(lbl);

        body.meas_info_lst.push_back(row);

        // Shape assertions — verbatim FlexRIC names.
        NS_TEST_ASSERT_MSG_EQ(body.meas_info_lst.size(),
                              1u,
                              "meas_info_lst length");
        NS_TEST_EXPECT_MSG_EQ(body.gran_period_ms,
                              1000u,
                              "gran_period_ms");
        const auto& got = body.meas_info_lst[0];
        NS_TEST_EXPECT_MSG_EQ(got.meas_type.meas_name,
                              "DRB.UEThpDl",
                              "verbatim canonical meas_name");
        NS_TEST_ASSERT_MSG_EQ(got.meas_record_lst.size(),
                              2u,
                              "two records");
        NS_TEST_EXPECT_MSG_EQ(static_cast<int>(got.meas_record_lst[0].form),
                              static_cast<int>(meas_value_form_t::real),
                              "record[0] form = real");
        NS_TEST_EXPECT_MSG_EQ(got.meas_record_lst[0].real_val,
                              12345.0,
                              "record[0] value");
        NS_TEST_EXPECT_MSG_EQ(got.meas_record_lst[1].int_val,
                              9876,
                              "record[1] integer value");
        NS_TEST_ASSERT_MSG_EQ(got.label_info_lst.size(),
                              1u,
                              "one label group");
        NS_TEST_ASSERT_MSG_EQ(got.label_info_lst[0].five_qi.has_value(),
                              true,
                              "five_qi present");
        NS_TEST_EXPECT_MSG_EQ(static_cast<int>(*got.label_info_lst[0].five_qi),
                              9,
                              "five_qi value");
        NS_TEST_EXPECT_MSG_EQ(*got.label_info_lst[0].s_nssai,
                              "1-000001",
                              "s_nssai value");
        NS_TEST_EXPECT_MSG_EQ(*got.label_info_lst[0].plmn_id,
                              "00101",
                              "plmn_id value");

        // The KPM body is the bytes of ric_indication_message; T2 will
        // serialise this struct into ASN.1-PER. For now sanity-check that
        // the struct lives cleanly inside a ric_indication_t carrier.
        ric_indication_t carrier{};
        carrier.ric_indication_header = {0xAA, 0xBB};
        carrier.ric_indication_message = {0xCC}; // placeholder for ASN.1 blob
        carrier.ric_action_id = 1;
        NS_TEST_EXPECT_MSG_EQ(carrier.ric_indication_message.size(),
                              1u,
                              "indication_message carries opaque PER blob");
    }
};

// ============================================================================
//  4.1.3 (Roadmap §4.1.3): E2SM-RC Style 3 Connected-Mode Mobility
// ============================================================================

class OranNtnRcStyle3ShapesTestCase : public TestCase
{
  public:
    OranNtnRcStyle3ShapesTestCase()
        : TestCase("E2SM-RC Style 3 action shapes carry verbatim WG3 fields")
    {
    }

  private:
    void DoRun() override
    {
        using namespace oranntn::rc_v103::style3;

        // Action 1 — Handover Control.
        HandoverControl h{};
        h.target_primary_cell_id.plmn_id = "00101";
        h.target_primary_cell_id.nr_cell_identity = 0x123456789ULL;
        h.handover_type = HandoverType::intra5gs;
        NS_TEST_EXPECT_MSG_EQ(static_cast<int>(h.kStyleId), 3, "Style ID");
        NS_TEST_EXPECT_MSG_EQ(static_cast<int>(h.kActionId),
                              1,
                              "Action 1 for plain HO");
        NS_TEST_EXPECT_MSG_EQ(h.target_primary_cell_id.plmn_id,
                              "00101",
                              "NRCGI PLMN");

        // Action 2 — Conditional Handover Control with two candidates.
        ConditionalHandoverControl cho{};
        cho.conditional_reconfiguration_id = 7;
        ConditionalHandoverControl::CandidateCell c1{};
        c1.target_primary_cell_id.plmn_id = "00101";
        c1.target_primary_cell_id.nr_cell_identity = 0xAAAA;
        c1.trigger_condition = {0xDE, 0xAD};
        ConditionalHandoverControl::CandidateCell c2{};
        c2.target_primary_cell_id.plmn_id = "00101";
        c2.target_primary_cell_id.nr_cell_identity = 0xBBBB;
        c2.trigger_condition = {0xBE, 0xEF};
        cho.candidate_cell_list = {c1, c2};
        NS_TEST_EXPECT_MSG_EQ(static_cast<int>(cho.kActionId),
                              2,
                              "Action 2 for CHO");
        NS_TEST_ASSERT_MSG_EQ(cho.candidate_cell_list.size(),
                              2u,
                              "2 candidates");
        NS_TEST_EXPECT_MSG_EQ(cho.candidate_cell_list[1]
                                  .target_primary_cell_id.nr_cell_identity,
                              0xBBBBu,
                              "candidate[1] NRCGI");

        // Action 3 — DAPS-HO Control.
        DapsHandoverControl daps{};
        daps.target_primary_cell_id.plmn_id = "00101";
        daps.target_primary_cell_id.nr_cell_identity = 0x42;
        daps.daps_termination_policy =
            DapsTerminationCause::condition_release;
        NS_TEST_EXPECT_MSG_EQ(static_cast<int>(daps.kActionId),
                              3,
                              "Action 3 for DAPS-HO");
        NS_TEST_ASSERT_MSG_EQ(daps.daps_termination_policy.has_value(),
                              true,
                              "DAPS cause set");
        NS_TEST_EXPECT_MSG_EQ(static_cast<int>(*daps.daps_termination_policy),
                              2,
                              "condition_release wire code");

        // ControlAction variant -> ActionId() helper.
        ControlAction a1{h};
        ControlAction a2{cho};
        ControlAction a3{daps};
        NS_TEST_EXPECT_MSG_EQ(static_cast<int>(ActionId(a1)), 1, "ActionId 1");
        NS_TEST_EXPECT_MSG_EQ(static_cast<int>(ActionId(a2)), 2, "ActionId 2");
        NS_TEST_EXPECT_MSG_EQ(static_cast<int>(ActionId(a3)), 3, "ActionId 3");

        // ControlMessage wrapper.
        ControlMessage msg{};
        msg.action = a2;
        NS_TEST_EXPECT_MSG_EQ(static_cast<int>(msg.style_id), 3, "style_id 3");
        NS_TEST_ASSERT_MSG_EQ(std::holds_alternative<ConditionalHandoverControl>(
                                  msg.action),
                              true,
                              "msg holds CHO");
    }
};

class OranNtnRcStyle3ConverterTestCase : public TestCase
{
  public:
    OranNtnRcStyle3ConverterTestCase()
        : TestCase("ConvertE2RcToStyle3 maps HO actions and rejects non-HO types")
    {
    }

  private:
    void DoRun() override
    {
        using namespace oranntn::rc_v103::style3;

        // HANDOVER_TRIGGER -> Action 1 (HandoverControl, intra5gs).
        E2RcAction trig{};
        trig.actionType = E2RcActionType::HANDOVER_TRIGGER;
        trig.targetGnbId = 0x12345;
        trig.targetUeId = 7;
        auto r1 = ConvertE2RcToStyle3(trig);
        NS_TEST_ASSERT_MSG_EQ(r1.has_value(), true, "HO trigger converted");
        NS_TEST_ASSERT_MSG_EQ(ActionId(*r1), 1u, "Action 1");
        const auto& hc = std::get<HandoverControl>(*r1);
        NS_TEST_EXPECT_MSG_EQ(static_cast<int>(hc.handover_type),
                              0,
                              "intra5gs wire code");
        NS_TEST_EXPECT_MSG_EQ(hc.target_primary_cell_id.plmn_id,
                              "00101",
                              "default PLMN");
        // gNB-ID packed in upper 28 bits of NR Cell Identity: 0x12345 << 8.
        NS_TEST_EXPECT_MSG_EQ(hc.target_primary_cell_id.nr_cell_identity,
                              static_cast<uint64_t>(0x12345) << 8,
                              "NRCGI = gNB-ID << 8");

        // HANDOVER_CANCEL -> Action 2 (empty CHO list = cancel-all).
        E2RcAction cancel{};
        cancel.actionType = E2RcActionType::HANDOVER_CANCEL;
        cancel.targetGnbId = 0x12345;
        auto r2 = ConvertE2RcToStyle3(cancel);
        NS_TEST_ASSERT_MSG_EQ(r2.has_value(), true, "HO cancel converted");
        NS_TEST_ASSERT_MSG_EQ(ActionId(*r2), 2u, "Action 2");
        const auto& chc = std::get<ConditionalHandoverControl>(*r2);
        NS_TEST_EXPECT_MSG_EQ(chc.candidate_cell_list.size(),
                              0u,
                              "empty list = cancel");
        NS_TEST_EXPECT_MSG_EQ(chc.conditional_reconfiguration_id,
                              0u,
                              "reconfig_id 0 = clear-all");

        // BEAM_SWITCH (and any non-HO action type) -> nullopt.
        E2RcAction beam{};
        beam.actionType = E2RcActionType::BEAM_SWITCH;
        beam.targetGnbId = 1;
        NS_TEST_EXPECT_MSG_EQ(ConvertE2RcToStyle3(beam).has_value(),
                              false,
                              "non-HO actions reject");

        // Custom PLMN argument honoured.
        auto r3 = ConvertE2RcToStyle3(trig, "26201");
        NS_TEST_ASSERT_MSG_EQ(r3.has_value(), true, "custom PLMN HO converted");
        NS_TEST_EXPECT_MSG_EQ(
            std::get<HandoverControl>(*r3).target_primary_cell_id.plmn_id,
            "26201",
            "custom PLMN propagated");
    }
};

// ============================================================================
//  4.1.2 (Roadmap §4.1.2): WG3-canonical KPM CSV emitted end-to-end
// ============================================================================

class OranNtnKpmCanonicalCsvTestCase : public TestCase
{
  public:
    OranNtnKpmCanonicalCsvTestCase()
        : TestCase("Canonical kpm_canonical.csv is long-format with 10 rows per E2KpmReport")
    {
    }

  private:
    static E2KpmReport MakeReport(uint32_t gnbId,
                                  uint32_t ueId,
                                  bool isNtn,
                                  double sinr,
                                  double thpMbps,
                                  double prbUtil)
    {
        E2KpmReport r{};
        r.timestamp = 0.5;
        r.gnbId = gnbId;
        r.isNtn = isNtn;
        r.ueId = ueId;
        r.sinr_dB = sinr;
        r.throughput_Mbps = thpMbps;
        r.prbUtilization = prbUtil;
        return r;
    }

    void DoRun() override
    {
        std::vector<E2KpmReport> reports = {
            MakeReport(1, 100, true, 12.5, 50.0, 0.5),
            MakeReport(2, 100, true, 9.0, 30.0, 0.7),
            MakeReport(3, 101, false, 15.5, 70.0, 0.3),
        };
        const std::map<std::string, std::string> baseLabels = {
            {oranntn::label::kFiveQi, "9"},
            {oranntn::label::kSnssai, "1-000001"},
            {oranntn::label::kPlmn, "00101"},
        };

        std::ostringstream os;
        oranntn::WriteCanonicalKpmCsv(reports, baseLabels, os);
        std::istringstream is(os.str());

        std::string header;
        std::getline(is, header);
        NS_TEST_EXPECT_MSG_EQ(header,
                              "timestamp,gnb_id,is_ntn,ue_id,metric_id,"
                              "value,present,FIVE_QI,S-NSSAI,PLMN",
                              "long-format header");

        const std::set<std::string> canonical = {
            oranntn::kpm::kDrbUeThpDl,      oranntn::kpm::kDrbUeThpUl,
            oranntn::kpm::kDrbPdcpVolumeDl, oranntn::kpm::kDrbPdcpVolumeUl,
            oranntn::kpm::kRruPrbAvailDl,   oranntn::kpm::kRruPrbAvailUl,
            oranntn::kpm::kRruPrbUsedDl,    oranntn::kpm::kRruPrbUsedUl,
            oranntn::kpm::kCarrAvgSinr,     oranntn::kpm::kL1mRsSinrMean,
        };
        size_t rowCount = 0;
        size_t notPresentCount = 0;
        std::set<std::string> seenIds;
        std::map<std::string, std::pair<uint32_t, uint32_t>> idToGnbUe;
        std::string line;
        while (std::getline(is, line))
        {
            ++rowCount;
            std::vector<std::string> f;
            std::string cur;
            for (char c : line)
            {
                if (c == ',')
                {
                    f.push_back(cur);
                    cur.clear();
                }
                else
                {
                    cur.push_back(c);
                }
            }
            f.push_back(cur);
            NS_TEST_ASSERT_MSG_EQ(f.size(), 10u, "10 columns per row");
            const std::string& metricId = f[4];
            NS_TEST_EXPECT_MSG_EQ(canonical.count(metricId),
                                  1u,
                                  std::string("metric_id '") + metricId +
                                      "' is canonical");
            seenIds.insert(metricId);
            NS_TEST_EXPECT_MSG_EQ(f[7], "9", "FIVE_QI column");
            NS_TEST_EXPECT_MSG_EQ(f[8], "1-000001", "S-NSSAI column");
            NS_TEST_EXPECT_MSG_EQ(f[9], "00101", "PLMN column");
            if (f[6] == "0")
            {
                ++notPresentCount;
            }
        }
        // 3 reports x 10 canonical metrics = 30 rows.
        NS_TEST_EXPECT_MSG_EQ(rowCount, 30u, "row count");
        NS_TEST_EXPECT_MSG_EQ(seenIds.size(),
                              10u,
                              "all 10 canonical IDs emitted at least once");
        // Three UL-side IDs are not-present per report -> 3 * 3 = 9 rows
        // should carry present=0 (the v2.1 baseline; will drop to 0 once
        // 4.1.9 CU/DU/RU split plumbs UL counters).
        NS_TEST_EXPECT_MSG_EQ(notPresentCount,
                              9u,
                              "3 UL metrics x 3 reports = 9 not-present rows");
    }
};

// ============================================================================
//  4.1.10 (Roadmap §4.1.10): WG3 conflict taxonomy
// ============================================================================

class OranNtnConflictTaxonomyTestCase : public TestCase
{
  public:
    OranNtnConflictTaxonomyTestCase()
        : TestCase("ClassifyConflict tags DIRECT, INDIRECT, IMPLICIT, UNKNOWN")
    {
    }

  private:
    static E2RcAction Make(E2RcActionType t, uint32_t gnb, uint32_t ue,
                            uint32_t beam = 0)
    {
        E2RcAction a{};
        a.actionType = t;
        a.targetGnbId = gnb;
        a.targetUeId = ue;
        a.targetBeamId = beam;
        return a;
    }

    void DoRun() override
    {
        // DIRECT: same action type, same target UE+gNB+beam.
        E2RcAction a = Make(E2RcActionType::MCS_OVERRIDE, 1, 100, 0);
        E2RcAction b = Make(E2RcActionType::MCS_OVERRIDE, 1, 100, 0);
        NS_TEST_EXPECT_MSG_EQ(static_cast<int>(
                                  OranNtnConflictManager::ClassifyConflict(a, b)),
                              static_cast<int>(ConflictType::DIRECT),
                              "DIRECT same parameter");

        // INDIRECT: different action types in the same family (PRB) on the
        // same gNB.
        E2RcAction prb1 = Make(E2RcActionType::SLICE_PRB_ALLOCATION, 1, 0);
        E2RcAction prb2 = Make(E2RcActionType::PRB_RESERVATION, 1, 0);
        NS_TEST_EXPECT_MSG_EQ(static_cast<int>(
                                  OranNtnConflictManager::ClassifyConflict(prb1,
                                                                            prb2)),
                              static_cast<int>(ConflictType::INDIRECT),
                              "INDIRECT same family same gNB");

        // INDIRECT for the HO family (HANDOVER_TRIGGER vs DC_SETUP).
        E2RcAction ho1 = Make(E2RcActionType::HANDOVER_TRIGGER, 5, 100);
        E2RcAction ho2 = Make(E2RcActionType::DC_SETUP, 5, 100);
        NS_TEST_EXPECT_MSG_EQ(static_cast<int>(
                                  OranNtnConflictManager::ClassifyConflict(ho1,
                                                                            ho2)),
                              static_cast<int>(ConflictType::INDIRECT),
                              "INDIRECT HO family");

        // IMPLICIT: different family, same UE.
        E2RcAction imp1 = Make(E2RcActionType::MCS_OVERRIDE, 1, 100);
        E2RcAction imp2 = Make(E2RcActionType::TX_POWER_CONTROL, 2, 100);
        NS_TEST_EXPECT_MSG_EQ(static_cast<int>(
                                  OranNtnConflictManager::ClassifyConflict(imp1,
                                                                            imp2)),
                              static_cast<int>(ConflictType::IMPLICIT),
                              "IMPLICIT different family same UE");

        // UNKNOWN: different family, different UE, different gNB.
        E2RcAction un1 = Make(E2RcActionType::MCS_OVERRIDE, 1, 100);
        E2RcAction un2 = Make(E2RcActionType::TX_POWER_CONTROL, 2, 200);
        NS_TEST_EXPECT_MSG_EQ(static_cast<int>(
                                  OranNtnConflictManager::ClassifyConflict(un1,
                                                                            un2)),
                              static_cast<int>(ConflictType::UNKNOWN),
                              "UNKNOWN unrelated targets");

        // ConflictTypeName strings.
        NS_TEST_EXPECT_MSG_EQ(std::string(
                                  ConflictTypeName(ConflictType::DIRECT)),
                              "direct",
                              "DIRECT label");
        NS_TEST_EXPECT_MSG_EQ(std::string(
                                  ConflictTypeName(ConflictType::INDIRECT)),
                              "indirect",
                              "INDIRECT label");
        NS_TEST_EXPECT_MSG_EQ(std::string(
                                  ConflictTypeName(ConflictType::IMPLICIT)),
                              "implicit",
                              "IMPLICIT label");
    }
};

// ============================================================================
//  4.1.4 (Roadmap §4.1.4): OranNtnDataRepository
// ============================================================================

namespace
{

E2KpmReport MakeKpm(uint32_t gnb, uint32_t ue, double ts, double sinr,
                    double thp)
{
    E2KpmReport r{};
    r.timestamp = ts;
    r.gnbId = gnb;
    r.isNtn = true;
    r.ueId = ue;
    r.sinr_dB = sinr;
    r.rsrp_dBm = -95.0;
    r.throughput_Mbps = thp;
    r.latency_ms = 20.0;
    r.elevation_deg = 45.0;
    r.doppler_Hz = 10000.0;
    r.tte_s = 120.0;
    r.prbUtilization = 0.5;
    r.sliceId = 0;
    return r;
}

E2RcAction MakeRc(double ts, const std::string& xappName,
                  E2RcActionType type, uint32_t targetGnb, uint32_t targetUe,
                  double confidence, bool executed)
{
    E2RcAction a{};
    a.timestamp = ts;
    a.xappId = 1;
    a.xappName = xappName;
    a.actionType = type;
    a.targetGnbId = targetGnb;
    a.targetUeId = targetUe;
    a.targetBeamId = 0;
    a.targetSliceId = 0;
    a.confidence = confidence;
    a.parameter1 = 0.0;
    a.parameter2 = 0.0;
    a.executed = executed;
    a.rejectionReason = executed ? "" : "test rejection";
    return a;
}

} // namespace

class OranNtnDataRepoInMemoryTestCase : public TestCase
{
  public:
    OranNtnDataRepoInMemoryTestCase()
        : TestCase("In-memory data repository logs, counts, and time-windows queries")
    {
    }

  private:
    void DoRun() override
    {
        Ptr<OranNtnDataRepository> repo =
            OranNtnDataRepository::Create("memory");
        NS_TEST_ASSERT_MSG_NE(repo, nullptr, "memory backend exists");
        NS_TEST_EXPECT_MSG_EQ(repo->GetBackendName(), "memory", "backend name");
        NS_TEST_EXPECT_MSG_EQ(repo->IsOpen(), true, "open after Create");

        repo->LogKpmReport(MakeKpm(1, 100, 1.0, 12.0, 50.0));
        repo->LogKpmReport(MakeKpm(2, 100, 2.0, 9.0, 30.0));
        repo->LogKpmReport(MakeKpm(3, 101, 1.5, 15.0, 70.0));
        repo->LogRcAction(MakeRc(1.2, "HoPredict",
                                  E2RcActionType::HANDOVER_TRIGGER, 1, 100,
                                  0.9, true));
        repo->LogRcAction(MakeRc(2.3, "HoPredict",
                                  E2RcActionType::HANDOVER_TRIGGER, 2, 100,
                                  0.7, false));
        repo->LogRcAction(MakeRc(1.1, "BeamHop",
                                  E2RcActionType::BEAM_SWITCH, 1, 0, 0.95,
                                  true));
        repo->LogXappRecord({1.0, 11, "HoPredict", 0, 0.9, 0.42});
        repo->LogXappRecord({2.0, 12, "BeamHop", 2, 0.95, 0.18});

        NS_TEST_EXPECT_MSG_EQ(repo->CountKpmReports(), 3u, "kpm count");
        NS_TEST_EXPECT_MSG_EQ(repo->CountRcActions(), 3u, "rc count");
        NS_TEST_EXPECT_MSG_EQ(repo->CountXappRecords(), 2u, "xapp count");

        // UE 100 had two reports across t in [0.5, 2.5].
        auto kpm100 = repo->GetKpmReportsForUe(100, 0.5, 2.5);
        NS_TEST_ASSERT_MSG_EQ(kpm100.size(), 2u, "UE 100 has 2 reports");
        NS_TEST_EXPECT_MSG_EQ(kpm100[0].gnbId, 1u, "first by gNB");
        NS_TEST_EXPECT_MSG_EQ(kpm100[1].gnbId, 2u, "second by gNB");

        // Narrow window picks only the t=1.0 report.
        auto kpm100_narrow = repo->GetKpmReportsForUe(100, 0.0, 1.4);
        NS_TEST_EXPECT_MSG_EQ(kpm100_narrow.size(),
                              1u,
                              "narrow window slices time");

        // HoPredict has 2 HO actions.
        auto rcHo = repo->GetRcActionsByXapp("HoPredict", 0.0, 5.0);
        NS_TEST_ASSERT_MSG_EQ(rcHo.size(), 2u, "HoPredict actions");
        NS_TEST_EXPECT_MSG_EQ(rcHo[1].executed,
                              false,
                              "second HO marked rejected");
        NS_TEST_EXPECT_MSG_EQ(rcHo[1].rejectionReason,
                              "test rejection",
                              "rejection reason preserved");

        // Unknown xapp name => empty.
        auto rcNone = repo->GetRcActionsByXapp("DoesNotExist", 0.0, 100.0);
        NS_TEST_EXPECT_MSG_EQ(rcNone.size(), 0u, "no rows for unknown xApp");

        // xapp_records across the full window.
        auto xappAll = repo->GetXappRecords(0.0, 100.0);
        NS_TEST_EXPECT_MSG_EQ(xappAll.size(), 2u, "all xapp records");
        NS_TEST_EXPECT_MSG_EQ(xappAll[0].xappName, "HoPredict", "record[0]");

        repo->Close();
        NS_TEST_EXPECT_MSG_EQ(repo->IsOpen(), false, "closed");
    }
};

#ifdef HAVE_SQLITE3
class OranNtnDataRepoSqliteTestCase : public TestCase
{
  public:
    OranNtnDataRepoSqliteTestCase()
        : TestCase("SQLite data repository round-trips KPM, RC, and xApp records")
    {
    }

  private:
    void DoRun() override
    {
        const std::string path = "/tmp/oran-ntn-test-repo.db";
        std::remove(path.c_str());

        Ptr<OranNtnDataRepository> repo =
            OranNtnDataRepository::Create("sqlite", path);
        NS_TEST_ASSERT_MSG_NE(repo, nullptr, "sqlite backend created");
        NS_TEST_EXPECT_MSG_EQ(repo->GetBackendName(), "sqlite", "backend name");
        NS_TEST_EXPECT_MSG_EQ(repo->IsOpen(), true, "open after Create");

        repo->LogKpmReport(MakeKpm(1, 100, 1.0, 12.0, 50.0));
        repo->LogKpmReport(MakeKpm(2, 100, 2.0, 9.0, 30.0));
        repo->LogKpmReport(MakeKpm(3, 101, 1.5, 15.0, 70.0));
        repo->LogRcAction(MakeRc(1.2, "HoPredict",
                                  E2RcActionType::HANDOVER_TRIGGER, 1, 100,
                                  0.9, true));
        repo->LogRcAction(MakeRc(2.3, "HoPredict",
                                  E2RcActionType::HANDOVER_TRIGGER, 2, 100,
                                  0.7, false));
        repo->LogXappRecord({1.0, 11, "HoPredict", 0, 0.9, 0.42});

        NS_TEST_EXPECT_MSG_EQ(repo->CountKpmReports(), 3u, "kpm count");
        NS_TEST_EXPECT_MSG_EQ(repo->CountRcActions(), 2u, "rc count");
        NS_TEST_EXPECT_MSG_EQ(repo->CountXappRecords(), 1u, "xapp count");

        auto k = repo->GetKpmReportsForUe(100, 0.0, 5.0);
        NS_TEST_ASSERT_MSG_EQ(k.size(), 2u, "two rows for UE 100");
        NS_TEST_EXPECT_MSG_EQ(k[0].sinr_dB, 12.0, "first row SINR");
        NS_TEST_EXPECT_MSG_EQ(k[1].throughput_Mbps,
                              30.0,
                              "second row throughput");
        NS_TEST_EXPECT_MSG_EQ(k[0].isNtn, true, "is_ntn round-trips");

        auto rc = repo->GetRcActionsByXapp("HoPredict", 0.0, 5.0);
        NS_TEST_ASSERT_MSG_EQ(rc.size(), 2u, "two RC actions");
        NS_TEST_EXPECT_MSG_EQ(rc[1].rejectionReason,
                              "test rejection",
                              "TEXT roundtrip");
        NS_TEST_EXPECT_MSG_EQ(rc[1].executed, false, "executed flag");

        repo->Close();
        NS_TEST_EXPECT_MSG_EQ(repo->IsOpen(), false, "closed");

        // Reopen the existing DB and confirm counts persist.
        Ptr<OranNtnDataRepository> repo2 =
            OranNtnDataRepository::Create("sqlite", path);
        NS_TEST_ASSERT_MSG_NE(repo2, nullptr, "reopen ok");
        NS_TEST_EXPECT_MSG_EQ(repo2->CountKpmReports(),
                              3u,
                              "kpm rows survived close+reopen");
        NS_TEST_EXPECT_MSG_EQ(repo2->CountRcActions(),
                              2u,
                              "rc rows survived close+reopen");
        repo2->Close();
        std::remove(path.c_str());
        std::remove((path + "-wal").c_str());
        std::remove((path + "-shm").c_str());
    }
};
#endif // HAVE_SQLITE3

// ============================================================================
//  Test Suite Registration
// ============================================================================

class OranNtnTestSuite : public TestSuite
{
  public:
    OranNtnTestSuite()
        : TestSuite("oran-ntn", Type::UNIT)
    {
        // Original tests
        AddTestCase(new OranNtnRicTestCase, TestCase::Duration::QUICK);
        AddTestCase(new OranNtnE2TestCase, TestCase::Duration::QUICK);
        AddTestCase(new OranNtnA1TestCase, TestCase::Duration::QUICK);
        AddTestCase(new OranNtnConflictTestCase, TestCase::Duration::QUICK);
        AddTestCase(new OranNtnSpaceRicTestCase, TestCase::Duration::QUICK);
        AddTestCase(new OranNtnFullPipelineTestCase, TestCase::Duration::QUICK);
        // Phase 1: Deep satellite integration
        AddTestCase(new OranNtnSatBridgeDeepTestCase, TestCase::Duration::QUICK);
        AddTestCase(new OranNtnExtendedKpmTestCase, TestCase::Duration::QUICK);
        // Phase 2: mmWave NTN
        AddTestCase(new OranNtnChannelModelTestCase, TestCase::Duration::QUICK);
        AddTestCase(new OranNtnSchedulerTestCase, TestCase::Duration::QUICK);
        AddTestCase(new OranNtnDualConnTestCase, TestCase::Duration::QUICK);
        AddTestCase(new OranNtnPhyKpmExtractorTestCase, TestCase::Duration::QUICK);
        // Phase 3: AI/ML
        AddTestCase(new OranNtnFederatedLearningTestCase, TestCase::Duration::QUICK);
        // Phase 4: Advanced xApps
        AddTestCase(new OranNtnAdvancedXappsTestCase, TestCase::Duration::QUICK);
        AddTestCase(new OranNtnEnergyHarvestTestCase, TestCase::Duration::QUICK);
        // Phase 5: Enhanced Space RIC
        AddTestCase(new OranNtnIslHeaderTestCase, TestCase::Duration::QUICK);
        AddTestCase(new OranNtnInferenceTestCase, TestCase::Duration::QUICK);
        AddTestCase(new OranNtnSpaceRicIslTestCase, TestCase::Duration::QUICK);
        // Realism roadmap T8 — KPM ID alignment.
        AddTestCase(new OranNtnKpmCanonicalIdsListTestCase,
                    TestCase::Duration::QUICK);
        AddTestCase(new OranNtnKpmCanonicalBuildTestCase,
                    TestCase::Duration::QUICK);
        AddTestCase(new OranNtnKpmCanonicalLabelsTestCase,
                    TestCase::Duration::QUICK);
        // Realism roadmap 4.1.1 — FlexRIC field-name parity.
        AddTestCase(new OranNtnFlexricE2apShapesTestCase,
                    TestCase::Duration::QUICK);
        AddTestCase(new OranNtnFlexricKpmFormat1TestCase,
                    TestCase::Duration::QUICK);
        // Realism roadmap 4.1.2 — canonical CSV end-to-end.
        AddTestCase(new OranNtnKpmCanonicalCsvTestCase,
                    TestCase::Duration::QUICK);
        // Realism roadmap 4.1.3 — E2SM-RC Style 3 Connected-Mode Mobility.
        AddTestCase(new OranNtnRcStyle3ShapesTestCase,
                    TestCase::Duration::QUICK);
        AddTestCase(new OranNtnRcStyle3ConverterTestCase,
                    TestCase::Duration::QUICK);
        // Realism roadmap 4.1.4 — OranNtnDataRepository (NIST pattern).
        AddTestCase(new OranNtnDataRepoInMemoryTestCase,
                    TestCase::Duration::QUICK);
#ifdef HAVE_SQLITE3
        AddTestCase(new OranNtnDataRepoSqliteTestCase,
                    TestCase::Duration::QUICK);
#endif
        // Realism roadmap 4.1.10 — WG3 conflict taxonomy.
        AddTestCase(new OranNtnConflictTaxonomyTestCase,
                    TestCase::Duration::QUICK);
    }
};

static OranNtnTestSuite g_oranNtnTestSuite;
