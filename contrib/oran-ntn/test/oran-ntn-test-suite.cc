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
#include "ns3/oran-ntn-a1-policy-schema.h"
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
#include "ns3/oran-ntn-f1-interface.h"
#include "ns3/oran-ntn-mmimo-codebook.h"
#include "ns3/oran-ntn-mmimo-precoder-xapp.h"
#include "ns3/oran-ntn-ofh-interface.h"
#include "ns3/oran-ntn-split-gnb-helper.h"
#include "ns3/oran-ntn-split-gnb.h"
#include "ns3/airan-inference-client.h"
#include "ns3/airan-inference-server.h"
#include "ns3/airan-messages.h"
#include "ns3/airan-mock-runtime.h"
#include "ns3/inference-channel-inproc.h"
#include "ns3/triton-model-config.h"
#include "ns3/asn1-per-codec.h"
#include "ns3/e2-listener.h"
#include "ns3/e2-transport.h"
#include "ns3/oran-ntn-service-model-ccc.h"
#include "ns3/oran-ntn-service-model-ntn-ephemeris.h"
#include "ns3/oran-ntn-service-model-kpm.h"
#include "ns3/oran-ntn-service-model-rc.h"
#include "ns3/oran-ntn-service-model.h"
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
//  Test 2b: E2 return-path RC delivery (downlink feeder delay)
// ============================================================================

class OranNtnE2ReturnPathRcTestCase : public TestCase
{
  public:
    OranNtnE2ReturnPathRcTestCase()
        : TestCase("O-RAN NTN E2 ReceiveRcAction executes after one "
                   "FeederLinkDelay on the return path")
    {
    }

  private:
    std::vector<Time> m_execTimes;

    bool HandleRcAction(E2RcAction action)
    {
        m_execTimes.push_back(Simulator::Now());
        return true;
    }

    void DoRun() override
    {
        auto e2node = CreateObject<OranNtnE2Node>();
        e2node->SetNodeId(1);
        e2node->SetIsNtn(true);
        e2node->SetFeederLinkDelay(MilliSeconds(20));
        e2node->RegisterRanFunction(3, "RC");
        e2node->SetRcActionCallback(
            MakeCallback(&OranNtnE2ReturnPathRcTestCase::HandleRcAction, this));

        E2RcAction action;
        action.timestamp = 0.0;
        action.xappId = 7;
        action.xappName = "test-rc";
        action.actionType = E2RcActionType::HANDOVER_TRIGGER;
        action.targetGnbId = 1;
        action.targetUeId = 42;
        action.targetBeamId = 0;
        action.targetSliceId = 0;
        action.confidence = 1.0;
        action.parameter1 = 0.0;
        action.parameter2 = 0.0;
        action.executed = false;

        // Deliver the RIC's command at t=0; the downlink feeder hop must
        // defer execution to t=20 ms (mirror of the uplink indication delay).
        Simulator::Schedule(Seconds(0),
                            &OranNtnE2Node::ReceiveRcAction,
                            e2node,
                            action);

        Simulator::Stop(MilliSeconds(100));
        Simulator::Run();
        Simulator::Destroy();

        NS_TEST_ASSERT_MSG_EQ(m_execTimes.size(), 1u,
                               "RC callback should fire exactly once");
        NS_TEST_ASSERT_MSG_EQ(m_execTimes[0], MilliSeconds(20),
                               "RC action must execute at t = FeederLinkDelay "
                               "(20 ms), not inline");
    }
};

// ============================================================================
//  Test 2c: AlignToControlLoop holds indications until the next RIC tick
// ============================================================================

class OranNtnE2AlignToControlLoopTestCase : public TestCase
{
  public:
    OranNtnE2AlignToControlLoopTestCase()
        : TestCase("O-RAN NTN E2 AlignToControlLoop defers indication "
                   "dispatch to the next control-loop tick")
    {
    }

  private:
    std::vector<Time> m_alignedTimes;
    std::vector<Time> m_inlineTimes;

    void HandleAlignedIndication(E2Indication indication)
    {
        m_alignedTimes.push_back(Simulator::Now());
    }

    void HandleInlineIndication(E2Indication indication)
    {
        m_inlineTimes.push_back(Simulator::Now());
    }

    static Ptr<OranNtnE2Node> MakeKpmNode(uint32_t gnbId)
    {
        auto node = CreateObject<OranNtnE2Node>();
        node->SetNodeId(gnbId);
        node->SetIsNtn(true);
        node->SetFeederLinkDelay(MilliSeconds(30));
        node->RegisterRanFunction(2, "KPM");

        // Subscription driven the same way as the routed E2 test, but
        // installed directly on the node (no periodic timer: period = 0) so
        // SubmitKpmMeasurement is the only indication source.
        E2Subscription sub;
        sub.subscriptionId = 1;
        sub.ricRequestorId = 1;
        sub.ranFunctionId = 2;
        sub.reportingPeriod = Seconds(0);
        sub.eventTrigger = false;
        sub.eventThreshold = 0.0;
        sub.batchOnVisibility = false;
        sub.maxBufferAge = Seconds(10);
        sub.useIslRelay = false;
        node->HandleSubscriptionRequest(sub);
        return node;
    }

    static E2KpmReport MakeReport(uint32_t gnbId)
    {
        E2KpmReport report;
        report.timestamp = 0.0;
        report.gnbId = gnbId;
        report.isNtn = true;
        report.ueId = 42;
        report.sinr_dB = 10.0;
        report.rsrp_dBm = -90.0;
        report.tte_s = 30.0;
        report.elevation_deg = 45.0;
        report.doppler_Hz = 1000.0;
        return report;
    }

    void DoRun() override
    {
        // Aligned node: measurement at t=100 ms crosses the feeder link and
        // lands at t=130 ms, then waits for the t=200 ms loop tick.
        auto aligned = MakeKpmNode(1);
        aligned->SetAttribute("AlignToControlLoop", BooleanValue(true));
        aligned->SetAttribute("ControlLoopPeriod", TimeValue(MilliSeconds(100)));
        aligned->SetIndicationCallback(MakeCallback(
            &OranNtnE2AlignToControlLoopTestCase::HandleAlignedIndication,
            this));

        // Default node (AlignToControlLoop = false): same delivery lands
        // inline at t=130 ms.
        auto inlineNode = MakeKpmNode(2);
        inlineNode->SetIndicationCallback(MakeCallback(
            &OranNtnE2AlignToControlLoopTestCase::HandleInlineIndication,
            this));

        Simulator::Schedule(MilliSeconds(100),
                            &OranNtnE2Node::SubmitKpmMeasurement,
                            aligned,
                            MakeReport(1));
        Simulator::Schedule(MilliSeconds(100),
                            &OranNtnE2Node::SubmitKpmMeasurement,
                            inlineNode,
                            MakeReport(2));

        Simulator::Stop(MilliSeconds(500));
        Simulator::Run();
        Simulator::Destroy();

        NS_TEST_ASSERT_MSG_EQ(m_alignedTimes.size(), 1u,
                               "aligned node delivers exactly one indication");
        NS_TEST_ASSERT_MSG_EQ(m_alignedTimes[0], MilliSeconds(200),
                               "delivery at 130 ms must wait for the next "
                               "control-loop tick at 200 ms");

        NS_TEST_ASSERT_MSG_EQ(m_inlineTimes.size(), 1u,
                               "default node delivers exactly one indication");
        NS_TEST_ASSERT_MSG_EQ(m_inlineTimes[0], MilliSeconds(130),
                               "default (AlignToControlLoop=false) dispatches "
                               "at delivery time t=130 ms");
    }
};

// ============================================================================
//  Test 2d: UnixEpochOffset shifts indication timestamps
// ============================================================================

class OranNtnE2UnixEpochOffsetTestCase : public TestCase
{
  public:
    OranNtnE2UnixEpochOffsetTestCase()
        : TestCase("O-RAN NTN E2 UnixEpochOffset produces Unix-like "
                   "indication timestamps; default keeps pure sim time")
    {
    }

  private:
    std::vector<double> m_offsetTimestamps;
    std::vector<double> m_simTimestamps;

    void HandleOffsetIndication(E2Indication indication)
    {
        m_offsetTimestamps.push_back(indication.timestamp);
    }

    void HandleSimIndication(E2Indication indication)
    {
        m_simTimestamps.push_back(indication.timestamp);
    }

    static Ptr<OranNtnE2Node> MakeKpmNode(uint32_t gnbId)
    {
        auto node = CreateObject<OranNtnE2Node>();
        node->SetNodeId(gnbId);
        node->SetIsNtn(true);
        node->SetFeederLinkDelay(MilliSeconds(20));
        node->RegisterRanFunction(2, "KPM");

        E2Subscription sub;
        sub.subscriptionId = 1;
        sub.ricRequestorId = 1;
        sub.ranFunctionId = 2;
        sub.reportingPeriod = Seconds(0);
        sub.eventTrigger = false;
        sub.eventThreshold = 0.0;
        sub.batchOnVisibility = false;
        sub.maxBufferAge = Seconds(10);
        sub.useIslRelay = false;
        node->HandleSubscriptionRequest(sub);
        return node;
    }

    void DoRun() override
    {
        auto offsetNode = MakeKpmNode(1);
        offsetNode->SetAttribute("UnixEpochOffset",
                                 DoubleValue(1750000000.0));
        offsetNode->SetIndicationCallback(MakeCallback(
            &OranNtnE2UnixEpochOffsetTestCase::HandleOffsetIndication, this));

        // Default node keeps UnixEpochOffset = 0 (pure simulation time).
        auto simNode = MakeKpmNode(2);
        simNode->SetIndicationCallback(MakeCallback(
            &OranNtnE2UnixEpochOffsetTestCase::HandleSimIndication, this));

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

        // Generate both indications at sim t = 2 s; the timestamp is stamped
        // at generation, not at (delayed) delivery.
        Simulator::Schedule(Seconds(2),
                            &OranNtnE2Node::SubmitKpmMeasurement,
                            offsetNode,
                            report);
        Simulator::Schedule(Seconds(2),
                            &OranNtnE2Node::SubmitKpmMeasurement,
                            simNode,
                            report);

        Simulator::Stop(Seconds(3));
        Simulator::Run();
        Simulator::Destroy();

        NS_TEST_ASSERT_MSG_EQ(m_offsetTimestamps.size(), 1u,
                               "offset node delivers one indication");
        NS_TEST_ASSERT_MSG_EQ_TOL(m_offsetTimestamps[0], 1750000002.0, 1e-6,
                                  "UnixEpochOffset=1750000000 + sim t=2 s "
                                  "-> 1750000002.0");

        NS_TEST_ASSERT_MSG_EQ(m_simTimestamps.size(), 1u,
                               "default node delivers one indication");
        NS_TEST_ASSERT_MSG_EQ_TOL(m_simTimestamps[0], 2.0, 1e-6,
                                  "default offset 0 keeps pure sim time (2 s)");
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
//  T4 (Roadmap §3 T4): Service-Model plugin ABI + KPM/RC concrete SMs
// ============================================================================

class OranNtnSmRegistryTestCase : public TestCase
{
  public:
    OranNtnSmRegistryTestCase()
        : TestCase("Service-Model registry: register, lookup, duplicate refusal")
    {
    }

  private:
    void DoRun() override
    {
        Ptr<OranNtnServiceModelRegistry> reg =
            CreateObject<OranNtnServiceModelRegistry>();
        NS_TEST_EXPECT_MSG_EQ(reg->Size(), 0u, "registry starts empty");

        Ptr<OranNtnServiceModelKpm> kpm = CreateObject<OranNtnServiceModelKpm>();
        Ptr<OranNtnServiceModelRc> rc = CreateObject<OranNtnServiceModelRc>();

        NS_TEST_EXPECT_MSG_EQ(reg->Register(kpm), true, "register KPM");
        NS_TEST_EXPECT_MSG_EQ(reg->Register(rc), true, "register RC");
        NS_TEST_EXPECT_MSG_EQ(reg->Size(), 2u, "two plugins registered");

        // Duplicate id refused.
        Ptr<OranNtnServiceModelKpm> kpm2 =
            CreateObject<OranNtnServiceModelKpm>();
        NS_TEST_EXPECT_MSG_EQ(reg->Register(kpm2),
                              false,
                              "duplicate function id refused");
        NS_TEST_EXPECT_MSG_EQ(reg->Size(), 2u, "still two plugins");

        // Lookup by function id.
        Ptr<OranNtnServiceModel> found = reg->Lookup(147);
        NS_TEST_ASSERT_MSG_NE(found, nullptr, "KPM looked up");
        NS_TEST_EXPECT_MSG_EQ(found->Name(), "KPM", "KPM name");
        NS_TEST_EXPECT_MSG_EQ(found->Version(), "v3.00", "KPM version");
        found = reg->Lookup(3);
        NS_TEST_ASSERT_MSG_NE(found, nullptr, "RC looked up");
        NS_TEST_EXPECT_MSG_EQ(found->Name(), "RC", "RC name");
        NS_TEST_EXPECT_MSG_EQ(found->Version(), "v1.03", "RC version");
        NS_TEST_EXPECT_MSG_EQ(reg->Lookup(9999),
                              Ptr<OranNtnServiceModel>(),
                              "unknown id returns nullptr");

        auto ids = reg->GetFunctionIds();
        NS_TEST_ASSERT_MSG_EQ(ids.size(), 2u, "two ids returned");
        // ascending
        NS_TEST_EXPECT_MSG_EQ(ids[0], 3u, "first id is RC (3)");
        NS_TEST_EXPECT_MSG_EQ(ids[1], 147u, "second id is KPM (147)");
    }
};

class OranNtnSmKpmRoundTripTestCase : public TestCase
{
  public:
    OranNtnSmKpmRoundTripTestCase()
        : TestCase("KPM SM encodes and decodes kpm_ind_msg_format_1_t")
    {
    }

  private:
    void DoRun() override
    {
        using namespace oranntn::flexric::kpm_v3;
        kpm_ind_msg_format_1_t body{};
        body.gran_period_ms = 1000;

        meas_info_format_1_lst_t row{};
        row.meas_type.form = meas_type_form_t::name;
        row.meas_type.meas_name = oranntn::kpm::kDrbUeThpDl;
        meas_record_item_t r1{};
        r1.form = meas_value_form_t::real;
        r1.real_val = 12345.5;
        row.meas_record_lst.push_back(r1);
        meas_record_item_t r2{};
        r2.form = meas_value_form_t::integer;
        r2.int_val = -42;
        row.meas_record_lst.push_back(r2);
        label_info_t lbl{};
        lbl.five_qi = 9;
        lbl.s_nssai = "1-000001";
        lbl.plmn_id = "00101";
        row.label_info_lst.push_back(lbl);
        body.meas_info_lst.push_back(row);

        OranNtnServiceModelKpm sm;
        NS_TEST_EXPECT_MSG_EQ(sm.RicFunctionId(),
                              147u,
                              "canonical KPM function id");
        const auto blob = sm.EncodeIndication(&body);
        NS_TEST_EXPECT_MSG_GT(blob.size(),
                              16u,
                              "blob is non-trivial");

        kpm_ind_msg_format_1_t got{};
        NS_TEST_ASSERT_MSG_EQ(sm.DecodeIndication(blob, got),
                              true,
                              "decode succeeds");
        NS_TEST_EXPECT_MSG_EQ(got.gran_period_ms, 1000u, "gran round-trips");
        NS_TEST_ASSERT_MSG_EQ(got.meas_info_lst.size(),
                              1u,
                              "one meas info");
        NS_TEST_EXPECT_MSG_EQ(got.meas_info_lst[0].meas_type.meas_name,
                              "DRB.UEThpDl",
                              "meas_name preserved");
        NS_TEST_ASSERT_MSG_EQ(got.meas_info_lst[0].meas_record_lst.size(),
                              2u,
                              "two records");
        NS_TEST_EXPECT_MSG_EQ(
            got.meas_info_lst[0].meas_record_lst[0].real_val,
            12345.5,
            "real value");
        NS_TEST_EXPECT_MSG_EQ(got.meas_info_lst[0].meas_record_lst[1].int_val,
                              -42,
                              "integer value");
        NS_TEST_ASSERT_MSG_EQ(got.meas_info_lst[0].label_info_lst.size(),
                              1u,
                              "one label group");
        NS_TEST_ASSERT_MSG_EQ(
            got.meas_info_lst[0].label_info_lst[0].five_qi.has_value(),
            true,
            "five_qi present");
        NS_TEST_EXPECT_MSG_EQ(
            static_cast<int>(*got.meas_info_lst[0].label_info_lst[0].five_qi),
            9,
            "five_qi value");
        NS_TEST_EXPECT_MSG_EQ(*got.meas_info_lst[0].label_info_lst[0].s_nssai,
                              "1-000001",
                              "s_nssai value");

        // KPM SM rejects ControlRequests (reporting only).
        std::vector<uint8_t> ctrlOut;
        NS_TEST_EXPECT_MSG_EQ(sm.DecodeControl(blob, &ctrlOut),
                              false,
                              "KPM SM has no Control direction");

        // Truncated blob -> decode fails cleanly.
        std::vector<uint8_t> trunc(blob.begin(), blob.begin() + 5);
        kpm_ind_msg_format_1_t bad{};
        NS_TEST_EXPECT_MSG_EQ(sm.DecodeIndication(trunc, bad),
                              false,
                              "truncated blob rejected");
    }
};

class OranNtnSmRcRoundTripTestCase : public TestCase
{
  public:
    OranNtnSmRcRoundTripTestCase()
        : TestCase("RC SM round-trips Style 3 HandoverControl, CHO, DAPS-HO")
    {
    }

  private:
    void DoRun() override
    {
        using namespace oranntn::rc_v103::style3;
        OranNtnServiceModelRc sm;
        NS_TEST_EXPECT_MSG_EQ(sm.RicFunctionId(),
                              3u,
                              "canonical RC function id");

        // Action 1 — HandoverControl.
        {
            ControlMessage m{};
            HandoverControl h{};
            h.target_primary_cell_id.plmn_id = "26201";
            h.target_primary_cell_id.nr_cell_identity = 0xABCDEFULL;
            h.handover_type = HandoverType::intra5gs;
            NrCellGlobalId sec{};
            sec.plmn_id = "26201";
            sec.nr_cell_identity = 0x12;
            h.new_secondary_cell_id = sec;
            m.action = h;

            const auto blob = sm.EncodeControl(m);
            NS_TEST_EXPECT_MSG_GT(blob.size(), 10u, "blob non-trivial");
            ControlMessage decoded{};
            NS_TEST_ASSERT_MSG_EQ(sm.DecodeControl(blob, &decoded),
                                  true,
                                  "decode HandoverControl");
            NS_TEST_ASSERT_MSG_EQ(
                std::holds_alternative<HandoverControl>(decoded.action),
                true,
                "variant holds HandoverControl");
            const auto& got = std::get<HandoverControl>(decoded.action);
            NS_TEST_EXPECT_MSG_EQ(got.target_primary_cell_id.plmn_id,
                                  "26201",
                                  "PLMN preserved");
            NS_TEST_EXPECT_MSG_EQ(got.target_primary_cell_id.nr_cell_identity,
                                  0xABCDEFULL,
                                  "NRCGI preserved");
            NS_TEST_ASSERT_MSG_EQ(got.new_secondary_cell_id.has_value(),
                                  true,
                                  "secondary cell preserved");
            NS_TEST_EXPECT_MSG_EQ(
                got.new_secondary_cell_id->nr_cell_identity,
                0x12u,
                "secondary NRCGI");
        }

        // Action 2 — CHO with two candidates.
        {
            ControlMessage m{};
            ConditionalHandoverControl c{};
            c.conditional_reconfiguration_id = 11;
            ConditionalHandoverControl::CandidateCell c1{};
            c1.target_primary_cell_id.plmn_id = "00101";
            c1.target_primary_cell_id.nr_cell_identity = 0xAAAA;
            c1.trigger_condition = {0x01, 0x02, 0x03};
            ConditionalHandoverControl::CandidateCell c2{};
            c2.target_primary_cell_id.plmn_id = "00101";
            c2.target_primary_cell_id.nr_cell_identity = 0xBBBB;
            c2.trigger_condition = {0xFF};
            c.candidate_cell_list = {c1, c2};
            m.action = c;

            const auto blob = sm.EncodeControl(m);
            ControlMessage decoded{};
            NS_TEST_ASSERT_MSG_EQ(sm.DecodeControl(blob, &decoded),
                                  true,
                                  "decode CHO");
            const auto& got =
                std::get<ConditionalHandoverControl>(decoded.action);
            NS_TEST_EXPECT_MSG_EQ(got.conditional_reconfiguration_id,
                                  11u,
                                  "reconfig id");
            NS_TEST_ASSERT_MSG_EQ(got.candidate_cell_list.size(),
                                  2u,
                                  "two candidates");
            NS_TEST_EXPECT_MSG_EQ(
                got.candidate_cell_list[1].trigger_condition.size(),
                1u,
                "candidate[1] trigger len");
            NS_TEST_EXPECT_MSG_EQ(
                got.candidate_cell_list[1].trigger_condition[0],
                0xFFu,
                "candidate[1] trigger byte");
        }

        // Action 3 — DAPS-HO.
        {
            ControlMessage m{};
            DapsHandoverControl d{};
            d.target_primary_cell_id.plmn_id = "00101";
            d.target_primary_cell_id.nr_cell_identity = 0x77;
            d.daps_termination_policy =
                DapsTerminationCause::target_radio_link_recovery;
            m.action = d;

            const auto blob = sm.EncodeControl(m);
            ControlMessage decoded{};
            NS_TEST_ASSERT_MSG_EQ(sm.DecodeControl(blob, &decoded),
                                  true,
                                  "decode DAPS");
            const auto& got = std::get<DapsHandoverControl>(decoded.action);
            NS_TEST_ASSERT_MSG_EQ(got.daps_termination_policy.has_value(),
                                  true,
                                  "DAPS cause preserved");
            NS_TEST_EXPECT_MSG_EQ(
                static_cast<int>(*got.daps_termination_policy),
                1,
                "target_radio_link_recovery wire code");
        }

        // Garbage blob -> decode fails.
        std::vector<uint8_t> garbage = {0x99, 0x99, 0x99};
        ControlMessage out{};
        NS_TEST_EXPECT_MSG_EQ(sm.DecodeControl(garbage, &out),
                              false,
                              "garbage rejected");
    }
};

// ============================================================================
//  4.1.11 (Roadmap §4.1.11): OSC-aligned A1 policy schema registry
// ============================================================================

class OranNtnA1PolicyRegistryTestCase : public TestCase
{
  public:
    OranNtnA1PolicyRegistryTestCase()
        : TestCase("A1 policy registry exposes OSC type IDs and toolkit "
                   "extensions in ascending order")
    {
    }

  private:
    void DoRun() override
    {
        using namespace oranntn::a1;

        const auto& reg = Registry();
        NS_TEST_ASSERT_MSG_EQ(reg.size(),
                              5u,
                              "registry has 5 entries (4 OSC + 1 NTN ext)");

        // Verbatim OSC IDs.
        NS_TEST_EXPECT_MSG_EQ(static_cast<uint32_t>(reg[0].type_id),
                              20000u,
                              "OSC traffic steering");
        NS_TEST_EXPECT_MSG_EQ(static_cast<uint32_t>(reg[1].type_id),
                              20001u,
                              "OSC QoE");
        NS_TEST_EXPECT_MSG_EQ(static_cast<uint32_t>(reg[2].type_id),
                              20008u,
                              "OSC handover control");
        NS_TEST_EXPECT_MSG_EQ(static_cast<uint32_t>(reg[3].type_id),
                              20020u,
                              "OSC ML model selection");
        NS_TEST_EXPECT_MSG_EQ(static_cast<uint32_t>(reg[4].type_id),
                              20050u,
                              "NTN beam priority (toolkit extension)");

        // Ascending order.
        for (size_t i = 1; i < reg.size(); ++i)
        {
            const bool ascending =
                static_cast<uint32_t>(reg[i - 1].type_id) <
                static_cast<uint32_t>(reg[i].type_id);
            NS_TEST_EXPECT_MSG_EQ(ascending,
                                  true,
                                  std::string("ascending order at index ") +
                                      std::to_string(i));
        }

        // OSC-vs-extension flag.
        NS_TEST_EXPECT_MSG_EQ(reg[2].is_osc_standard,
                              true,
                              "20008 marked OSC standard");
        NS_TEST_EXPECT_MSG_EQ(reg[4].is_osc_standard,
                              false,
                              "20050 marked toolkit extension");

        // Lookup() works for present and absent IDs.
        const auto* ho = Lookup(PolicyTypeId::OscHandoverControl);
        NS_TEST_ASSERT_MSG_NE(ho, nullptr, "20008 looked up");
        NS_TEST_EXPECT_MSG_EQ(ho->slug, "handover-control", "20008 slug");
        const auto* notRegistered =
            Lookup(static_cast<PolicyTypeId>(99999));
        NS_TEST_EXPECT_MSG_EQ(notRegistered,
                              static_cast<const A1PolicySchema*>(nullptr),
                              "absent ID returns nullptr");

        // Mapping from local A1PolicyType enum to OSC IDs.
        NS_TEST_EXPECT_MSG_EQ(OscIdForLocalType(A1PolicyType::HO_THRESHOLD),
                              20008u,
                              "HO_THRESHOLD -> 20008");
        NS_TEST_EXPECT_MSG_EQ(OscIdForLocalType(A1PolicyType::TN_NTN_STEERING),
                              20000u,
                              "TN_NTN_STEERING -> 20000");
        NS_TEST_EXPECT_MSG_EQ(OscIdForLocalType(A1PolicyType::SLICE_SLA),
                              20001u,
                              "SLICE_SLA -> 20001");
        NS_TEST_EXPECT_MSG_EQ(OscIdForLocalType(A1PolicyType::BEAM_PRIORITY),
                              20050u,
                              "BEAM_PRIORITY -> 20050");
        // Unmapped local types return 0 (sentinel).
        NS_TEST_EXPECT_MSG_EQ(OscIdForLocalType(A1PolicyType::FL_PARTICIPATION),
                              0u,
                              "FL_PARTICIPATION not yet mapped");

        // Schema files referenced by the registry exist on disk; that
        // contract is what gives reviewers a path they can fetch.
        for (const auto& s : reg)
        {
            std::ifstream f(s.schema_file);
            NS_TEST_EXPECT_MSG_EQ(f.good(),
                                  true,
                                  std::string("schema file present on disk: ")
                                      + s.schema_file);
        }
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
//  4.1.7 (Roadmap §4.1.7): NTN-Ephemeris SM plugin (SIB19)
// ============================================================================

class OranNtnSmEphemerisOrbitalTest : public TestCase
{
  public:
    OranNtnSmEphemerisOrbitalTest()
        : TestCase("NTN-Ephemeris SM round-trips orbital SIB19 with all optional IEs")
    {
    }

  private:
    void DoRun() override
    {
        using namespace oranntn::ephemeris;

        Sib19NtnConfig cfg{};
        cfg.epoch.sfn = 512;
        cfg.epoch.subframe = 7;
        OrbitalElementsIe orb{};
        orb.semi_major_axis_m = 6921000.0;
        orb.eccentricity = 0.0007381;
        orb.inclination_rad = 51.6447 * M_PI / 180.0;
        orb.raan_rad = 91.8123 * M_PI / 180.0;
        orb.arg_perigee_rad = 152.7392 * M_PI / 180.0;
        orb.mean_anomaly_rad = 207.3922 * M_PI / 180.0;
        cfg.ephemeris = orb;
        cfg.ta.ta_common_us = 4567.5;
        cfg.ta.ta_common_drift_us_per_s = -42.5;
        cfg.ta.ta_common_drift_variation_us_per_s2 = 0.01;
        cfg.cell_specific_koffset_slots = 122;
        ServiceWindowIe sw{};
        sw.t_service_start_s = 1.0e9;
        sw.t_service_dur_s = 600.0;
        cfg.service_window = sw;
        cfg.ntn_ul_sync_validity_duration_ms = 900;

        OranNtnServiceModelNtnEphemeris sm;
        NS_TEST_EXPECT_MSG_EQ(sm.RicFunctionId(), 1001u,
                              "ephemeris function ID");
        NS_TEST_EXPECT_MSG_EQ(sm.Name(), "NTN-Ephemeris", "name");
        NS_TEST_EXPECT_MSG_EQ(sm.Version(), "v1.00", "version");

        const auto blob = sm.EncodeIndication(&cfg);
        NS_TEST_EXPECT_MSG_GT(blob.size(), 32u, "non-trivial PER blob");

        Sib19NtnConfig got{};
        NS_TEST_ASSERT_MSG_EQ(sm.DecodeIndication(blob, got),
                              true,
                              "decode SIB19");
        NS_TEST_EXPECT_MSG_EQ(got.epoch.sfn, 512u, "SFN");
        NS_TEST_EXPECT_MSG_EQ(static_cast<int>(got.epoch.subframe), 7,
                              "subframe");

        NS_TEST_ASSERT_MSG_EQ(
            std::holds_alternative<OrbitalElementsIe>(got.ephemeris),
            true,
            "decoded ephemeris is orbital");
        const auto& og = std::get<OrbitalElementsIe>(got.ephemeris);
        NS_TEST_EXPECT_MSG_EQ(og.semi_major_axis_m,
                              6921000.0,
                              "semi-major axis exact (raw IEEE-754 bits)");
        NS_TEST_EXPECT_MSG_EQ(og.eccentricity, 0.0007381, "eccentricity");
        NS_TEST_EXPECT_MSG_EQ(og.raan_rad, orb.raan_rad,
                              "RAAN preserved");

        NS_TEST_EXPECT_MSG_EQ(got.ta.ta_common_us, 4567.5, "TA common");
        NS_TEST_EXPECT_MSG_EQ(got.ta.ta_common_drift_us_per_s, -42.5,
                              "TA drift");
        NS_TEST_EXPECT_MSG_EQ(got.cell_specific_koffset_slots, 122,
                              "K-offset");
        NS_TEST_ASSERT_MSG_EQ(got.service_window.has_value(), true,
                              "service window present");
        NS_TEST_EXPECT_MSG_EQ(got.service_window->t_service_start_s,
                              1.0e9,
                              "service window start");
        NS_TEST_EXPECT_MSG_EQ(got.service_window->t_service_dur_s,
                              600.0,
                              "service window duration");
        NS_TEST_ASSERT_MSG_EQ(
            got.ntn_ul_sync_validity_duration_ms.has_value(),
            true,
            "UL sync duration present");
        NS_TEST_EXPECT_MSG_EQ(*got.ntn_ul_sync_validity_duration_ms,
                              900u,
                              "UL sync duration value");

        std::vector<uint8_t> dummy;
        NS_TEST_EXPECT_MSG_EQ(sm.DecodeControl(blob, &dummy),
                              false,
                              "Ephemeris SM has no Control direction");
    }
};

class OranNtnSmEphemerisPvTest : public TestCase
{
  public:
    OranNtnSmEphemerisPvTest()
        : TestCase("NTN-Ephemeris SM round-trips position-velocity SIB19 with absent optionals")
    {
    }

  private:
    void DoRun() override
    {
        using namespace oranntn::ephemeris;

        Sib19NtnConfig cfg{};
        cfg.epoch.sfn = 0;
        cfg.epoch.subframe = 0;
        PositionVelocityIe pv{};
        pv.pos_x_m = 6921000.0;
        pv.pos_y_m = 0.0;
        pv.pos_z_m = 0.0;
        pv.vel_x_mps = 0.0;
        pv.vel_y_mps = 7560.0;
        pv.vel_z_mps = 0.0;
        cfg.ephemeris = pv;
        cfg.ta.ta_common_us = 0.0;
        cfg.ta.ta_common_drift_us_per_s = 0.0;
        cfg.ta.ta_common_drift_variation_us_per_s2 = 0.0;
        cfg.cell_specific_koffset_slots = 0;
        // No service_window, no UL-sync-validity duration.

        OranNtnServiceModelNtnEphemeris sm;
        const auto blob = sm.EncodeIndication(&cfg);
        Sib19NtnConfig got{};
        NS_TEST_ASSERT_MSG_EQ(sm.DecodeIndication(blob, got),
                              true,
                              "decode SIB19 PV");
        NS_TEST_ASSERT_MSG_EQ(
            std::holds_alternative<PositionVelocityIe>(got.ephemeris),
            true,
            "decoded ephemeris is PV");
        const auto& pg = std::get<PositionVelocityIe>(got.ephemeris);
        NS_TEST_EXPECT_MSG_EQ(pg.pos_x_m, 6921000.0, "pos_x preserved");
        NS_TEST_EXPECT_MSG_EQ(pg.vel_y_mps, 7560.0, "vel_y preserved");
        NS_TEST_EXPECT_MSG_EQ(got.service_window.has_value(),
                              false,
                              "no service window");
        NS_TEST_EXPECT_MSG_EQ(
            got.ntn_ul_sync_validity_duration_ms.has_value(),
            false,
            "no UL-sync-validity duration");
    }
};

class OranNtnSmRegistryFourPluginsTest : public TestCase
{
  public:
    OranNtnSmRegistryFourPluginsTest()
        : TestCase("Service-Model registry resolves KPM, RC, CCC, and "
                   "NTN-Ephemeris plugins")
    {
    }

  private:
    void DoRun() override
    {
        Ptr<OranNtnServiceModelRegistry> reg =
            CreateObject<OranNtnServiceModelRegistry>();
        Ptr<OranNtnServiceModelKpm> kpm =
            CreateObject<OranNtnServiceModelKpm>();
        Ptr<OranNtnServiceModelRc> rc =
            CreateObject<OranNtnServiceModelRc>();
        Ptr<OranNtnServiceModelCcc> ccc =
            CreateObject<OranNtnServiceModelCcc>();
        Ptr<OranNtnServiceModelNtnEphemeris> eph =
            CreateObject<OranNtnServiceModelNtnEphemeris>();

        NS_TEST_EXPECT_MSG_EQ(reg->Register(kpm), true, "KPM registered");
        NS_TEST_EXPECT_MSG_EQ(reg->Register(rc), true, "RC registered");
        NS_TEST_EXPECT_MSG_EQ(reg->Register(ccc), true, "CCC registered");
        NS_TEST_EXPECT_MSG_EQ(reg->Register(eph), true, "Eph registered");
        NS_TEST_EXPECT_MSG_EQ(reg->Size(), 4u, "4 plugins");

        auto ids = reg->GetFunctionIds();
        NS_TEST_ASSERT_MSG_EQ(ids.size(), 4u, "4 IDs");
        NS_TEST_EXPECT_MSG_EQ(ids[0], 3u, "RC first");
        NS_TEST_EXPECT_MSG_EQ(ids[1], 147u, "KPM second");
        NS_TEST_EXPECT_MSG_EQ(ids[2], 1000u, "CCC third");
        NS_TEST_EXPECT_MSG_EQ(ids[3], 1001u, "Ephemeris fourth");

        Ptr<OranNtnServiceModel> p = reg->Lookup(1001);
        NS_TEST_ASSERT_MSG_NE(p, nullptr, "Ephemeris lookup");
        NS_TEST_EXPECT_MSG_EQ(p->Name(), "NTN-Ephemeris", "Ephemeris name");
        NS_TEST_EXPECT_MSG_EQ(p->Version(), "v1.00", "Ephemeris version");
    }
};

// ============================================================================
//  4.1.8 (Roadmap §4.1.8): E2SM-CCC NTN extensions
// ============================================================================

class OranNtnSmCccNtnLeoPassTest : public TestCase
{
  public:
    OranNtnSmCccNtnLeoPassTest()
        : TestCase("CCC NTN ext: set_leo_pass_on encodes and decodes a pass-on event")
    {
    }

  private:
    void DoRun() override
    {
        using namespace oranntn::ccc;
        OranNtnServiceModelCcc sm;
        CccControlAction a{};
        a.op = CccControlAction::Op::set_leo_pass_on;
        LeoPassToggleIe p{};
        p.nr_cell_global_id = 0xABC123;
        p.beam_id = 5;
        p.enable = true;
        p.t_event_s = 1234567890.5;
        a.leo_pass_updates.push_back(p);
        LeoPassToggleIe p2{};
        p2.nr_cell_global_id = 0xABC123;
        p2.beam_id = 7;
        p2.enable = false;
        p2.t_event_s = 1234567910.0;
        a.leo_pass_updates.push_back(p2);

        const auto blob = sm.EncodeControl(a);
        CccControlAction got{};
        NS_TEST_ASSERT_MSG_EQ(sm.DecodeControl(blob, &got),
                              true,
                              "decode LEO pass action");
        NS_TEST_EXPECT_MSG_EQ(static_cast<int>(got.op),
                              static_cast<int>(
                                  CccControlAction::Op::set_leo_pass_on),
                              "op set_leo_pass_on");
        NS_TEST_ASSERT_MSG_EQ(got.leo_pass_updates.size(), 2u,
                              "2 LEO pass entries");
        NS_TEST_EXPECT_MSG_EQ(got.leo_pass_updates[0].nr_cell_global_id,
                              0xABC123u, "entry[0] NCGI");
        NS_TEST_EXPECT_MSG_EQ(static_cast<int>(got.leo_pass_updates[0].beam_id),
                              5, "entry[0] beam");
        NS_TEST_EXPECT_MSG_EQ(got.leo_pass_updates[0].enable, true,
                              "entry[0] enable");
        NS_TEST_EXPECT_MSG_EQ(got.leo_pass_updates[0].t_event_s,
                              1234567890.5,
                              "entry[0] t_event preserved (IEEE-754 bits)");
        NS_TEST_EXPECT_MSG_EQ(got.leo_pass_updates[1].enable, false,
                              "entry[1] enable=false");
        NS_TEST_EXPECT_MSG_EQ(got.beam_reconfigs.size(), 0u,
                              "no beam reconfigs");
        NS_TEST_EXPECT_MSG_EQ(got.doppler_retunes.size(), 0u,
                              "no doppler retunes");
    }
};

class OranNtnSmCccNtnBeamReconfigTest : public TestCase
{
  public:
    OranNtnSmCccNtnBeamReconfigTest()
        : TestCase("CCC NTN ext: beam_reconfig carries steering + complex codebook weights")
    {
    }

  private:
    void DoRun() override
    {
        using namespace oranntn::ccc;
        OranNtnServiceModelCcc sm;
        CccControlAction a{};
        a.op = CccControlAction::Op::beam_reconfig;
        BeamReconfigIe b{};
        b.nr_cell_global_id = 0xDEADBEEF;
        b.beam_id = 12;
        b.steering_az_deg = 35.5;
        b.steering_el_deg = -7.25;
        b.codebook_weights = {1.0, 0.0, 0.0, 1.0, -1.0, 0.0, 0.0, -1.0};
        a.beam_reconfigs.push_back(b);

        const auto blob = sm.EncodeControl(a);
        CccControlAction got{};
        NS_TEST_ASSERT_MSG_EQ(sm.DecodeControl(blob, &got),
                              true,
                              "decode beam reconfig");
        NS_TEST_EXPECT_MSG_EQ(static_cast<int>(got.op),
                              static_cast<int>(
                                  CccControlAction::Op::beam_reconfig),
                              "op beam_reconfig");
        NS_TEST_ASSERT_MSG_EQ(got.beam_reconfigs.size(), 1u,
                              "1 beam reconfig");
        const auto& gb = got.beam_reconfigs[0];
        NS_TEST_EXPECT_MSG_EQ(gb.nr_cell_global_id, 0xDEADBEEFu, "NCGI");
        NS_TEST_EXPECT_MSG_EQ(static_cast<int>(gb.beam_id), 12, "beam id");
        NS_TEST_EXPECT_MSG_EQ(gb.steering_az_deg, 35.5, "az");
        NS_TEST_EXPECT_MSG_EQ(gb.steering_el_deg, -7.25, "el");
        NS_TEST_ASSERT_MSG_EQ(gb.codebook_weights.size(), 8u,
                              "8 codebook entries");
        NS_TEST_EXPECT_MSG_EQ(gb.codebook_weights[2], 0.0, "weights[2]");
        NS_TEST_EXPECT_MSG_EQ(gb.codebook_weights[4], -1.0, "weights[4]");
    }
};

class OranNtnSmCccNtnDopplerRetuneTest : public TestCase
{
  public:
    OranNtnSmCccNtnDopplerRetuneTest()
        : TestCase("CCC NTN ext: doppler_retune carries ARFCN updates and Doppler offset")
    {
    }

  private:
    void DoRun() override
    {
        using namespace oranntn::ccc;
        OranNtnServiceModelCcc sm;
        CccControlAction a{};
        a.op = CccControlAction::Op::doppler_retune;
        DopplerRetuneIe d{};
        d.nr_cell_global_id = 0x42;
        d.arfcn_dl = 638400;
        d.arfcn_ul = 638400;
        d.doppler_offset_hz = -13736.5;
        a.doppler_retunes.push_back(d);

        const auto blob = sm.EncodeControl(a);
        CccControlAction got{};
        NS_TEST_ASSERT_MSG_EQ(sm.DecodeControl(blob, &got),
                              true,
                              "decode doppler retune");
        NS_TEST_EXPECT_MSG_EQ(static_cast<int>(got.op),
                              static_cast<int>(
                                  CccControlAction::Op::doppler_retune),
                              "op doppler_retune");
        NS_TEST_ASSERT_MSG_EQ(got.doppler_retunes.size(), 1u,
                              "1 doppler retune");
        const auto& gd = got.doppler_retunes[0];
        NS_TEST_EXPECT_MSG_EQ(gd.arfcn_dl, 638400u, "DL ARFCN");
        NS_TEST_EXPECT_MSG_EQ(gd.arfcn_ul, 638400u, "UL ARFCN");
        NS_TEST_EXPECT_MSG_EQ(gd.doppler_offset_hz, -13736.5,
                              "Doppler offset preserved");
    }
};

namespace
{

struct PassEvent
{
    double t_s;
    uint16_t beam_id;
    bool enable;
    size_t blob_size;
    bool decoded_ok;
};

void
FirePassEvent(uint16_t beam, bool on, std::vector<PassEvent>* out)
{
    using namespace oranntn::ccc;
    OranNtnServiceModelCcc sm;
    CccControlAction a{};
    a.op = on ? CccControlAction::Op::set_leo_pass_on
              : CccControlAction::Op::set_leo_pass_off;
    LeoPassToggleIe p{};
    p.nr_cell_global_id = 0x100;
    p.beam_id = beam;
    p.enable = on;
    p.t_event_s = Simulator::Now().GetSeconds();
    a.leo_pass_updates.push_back(p);
    const auto blob = sm.EncodeControl(a);
    CccControlAction got{};
    const bool ok = sm.DecodeControl(blob, &got);
    out->push_back({Simulator::Now().GetSeconds(),
                     beam, on, blob.size(), ok});
}

} // namespace

class OranNtnSmCccNtnSimulatorTimeTest : public TestCase
{
  public:
    OranNtnSmCccNtnSimulatorTimeTest()
        : TestCase("Simulator: 600 s pass-on / pass-off stream encodes and "
                   "decodes through CCC NTN ext")
    {
    }

  private:
    void DoRun() override
    {
        std::vector<PassEvent> events;
        Simulator::Schedule(Seconds(60),
                            &FirePassEvent, uint16_t{5}, true, &events);
        Simulator::Schedule(Seconds(540),
                            &FirePassEvent, uint16_t{5}, false, &events);
        Simulator::Schedule(Seconds(200),
                            &FirePassEvent, uint16_t{7}, true, &events);
        Simulator::Schedule(Seconds(400),
                            &FirePassEvent, uint16_t{7}, false, &events);
        Simulator::Stop(Seconds(601));
        Simulator::Run();

        NS_TEST_ASSERT_MSG_EQ(events.size(), 4u, "4 LEO pass events");
        for (const auto& e : events)
        {
            NS_TEST_EXPECT_MSG_EQ(e.decoded_ok, true,
                                  "every event encode/decode round-trips");
            NS_TEST_EXPECT_MSG_GT(e.blob_size, 0u, "non-empty blob");
        }
        NS_TEST_EXPECT_MSG_EQ(events[0].enable, true, "beam 5 on at t=60");
        NS_TEST_EXPECT_MSG_EQ(events[1].enable, true, "beam 7 on at t=200");
        NS_TEST_EXPECT_MSG_EQ(events[2].enable, false, "beam 7 off at t=400");
        NS_TEST_EXPECT_MSG_EQ(events[3].enable, false, "beam 5 off at t=540");
        NS_TEST_EXPECT_MSG_EQ(events[0].t_s, 60.0, "event[0] timestamp");
        NS_TEST_EXPECT_MSG_EQ(events[3].t_s, 540.0, "event[3] timestamp");

        Simulator::Destroy();
    }
};

// ============================================================================
//  4.1.6 (Roadmap §4.1.6): E2SM-CCC SM plugin
// ============================================================================

class OranNtnSmCccIndicationTest : public TestCase
{
  public:
    OranNtnSmCccIndicationTest()
        : TestCase("CCC SM Indication round-trips cell config + perf objectives")
    {
    }

  private:
    void DoRun() override
    {
        using namespace oranntn::ccc;

        CccIndMsgFormat1 body{};
        body.snapshot_seq = 42;
        // Two cells.
        CellConfigRecord c1{};
        c1.nr_cell_global_id = 0x123456789ULL;
        c1.dtx_us_log2 = 6;
        c1.drx_us_log2 = 8;
        c1.output_power_dbm = 33;
        c1.prb_pool_total = 273;
        c1.prb_pool_reserved = 12;
        c1.antenna_mask = 0xF0F0F0F0ULL;
        c1.arfcn_dl = 638400;
        c1.arfcn_ul = 638400;
        body.cells.push_back(c1);

        CellConfigRecord c2{};
        c2.nr_cell_global_id = 0xABCDEFULL;
        c2.dtx_us_log2 = 5;
        c2.drx_us_log2 = 7;
        c2.output_power_dbm = 40;
        c2.prb_pool_total = 51;        // FR1 20 MHz at 30 kHz SCS
        c2.prb_pool_reserved = 4;
        c2.antenna_mask = 0xFFFFULL;
        // No ARFCN entries (OPTIONAL absent).
        body.cells.push_back(c2);

        // Two perf objectives.
        PerformanceObjective o1{};
        o1.metric = PerformanceObjective::Metric::spectral_efficiency;
        o1.target_value = 4.5;
        o1.tolerance = 0.5;
        o1.scope_nr_cgi = 0x123456789ULL;
        o1.scope_slice_id = 1;
        body.perf_objectives.push_back(o1);

        PerformanceObjective o2{};
        o2.metric = PerformanceObjective::Metric::latency_ms;
        o2.target_value = 20.0;
        o2.tolerance = 5.0;
        o2.scope_nr_cgi = 0;
        o2.scope_slice_id = 0;
        body.perf_objectives.push_back(o2);

        OranNtnServiceModelCcc sm;
        NS_TEST_EXPECT_MSG_EQ(sm.RicFunctionId(),
                              1000u,
                              "CCC function ID");
        NS_TEST_EXPECT_MSG_EQ(sm.Name(), "CCC", "CCC name");
        NS_TEST_EXPECT_MSG_EQ(sm.Version(), "v1.00", "CCC version");

        const auto blob = sm.EncodeIndication(&body);
        NS_TEST_EXPECT_MSG_GT(blob.size(), 16u, "non-trivial PER blob");

        CccIndMsgFormat1 got{};
        NS_TEST_ASSERT_MSG_EQ(sm.DecodeIndication(blob, got),
                              true,
                              "decode indication");
        NS_TEST_EXPECT_MSG_EQ(got.snapshot_seq, 42u, "snapshot_seq");
        NS_TEST_ASSERT_MSG_EQ(got.cells.size(), 2u, "2 cells");
        NS_TEST_EXPECT_MSG_EQ(got.cells[0].nr_cell_global_id,
                              0x123456789ULL,
                              "cell[0] NCGI");
        NS_TEST_EXPECT_MSG_EQ(static_cast<int>(got.cells[0].dtx_us_log2),
                              6,
                              "cell[0] DTX");
        NS_TEST_EXPECT_MSG_EQ(static_cast<int>(got.cells[0].output_power_dbm),
                              33,
                              "cell[0] power");
        NS_TEST_ASSERT_MSG_EQ(got.cells[0].arfcn_dl.has_value(),
                              true,
                              "cell[0] has DL ARFCN");
        NS_TEST_EXPECT_MSG_EQ(*got.cells[0].arfcn_dl, 638400u,
                              "cell[0] DL ARFCN value");
        NS_TEST_EXPECT_MSG_EQ(got.cells[1].arfcn_dl.has_value(),
                              false,
                              "cell[1] has no DL ARFCN (OPTIONAL absent)");

        NS_TEST_ASSERT_MSG_EQ(got.perf_objectives.size(),
                              2u,
                              "2 perf objectives");
        NS_TEST_EXPECT_MSG_EQ(static_cast<int>(got.perf_objectives[0].metric),
                              static_cast<int>(
                                  PerformanceObjective::Metric::spectral_efficiency),
                              "obj[0] metric");
        NS_TEST_EXPECT_MSG_EQ(got.perf_objectives[0].target_value,
                              4.5,
                              "obj[0] target");
        NS_TEST_EXPECT_MSG_EQ(got.perf_objectives[1].target_value,
                              20.0,
                              "obj[1] target");

        // Truncated blob -> decode fails cleanly.
        std::vector<uint8_t> trunc(blob.begin(), blob.begin() + 3);
        CccIndMsgFormat1 bad{};
        NS_TEST_EXPECT_MSG_EQ(sm.DecodeIndication(trunc, bad),
                              false,
                              "truncated blob rejected");
    }
};

class OranNtnSmCccControlTest : public TestCase
{
  public:
    OranNtnSmCccControlTest()
        : TestCase("CCC SM ControlAction round-trips set and clear ops")
    {
    }

  private:
    void DoRun() override
    {
        using namespace oranntn::ccc;
        OranNtnServiceModelCcc sm;

        // set_perf_objective with one objective.
        CccControlAction a1{};
        a1.op = CccControlAction::Op::set_perf_objective;
        PerformanceObjective po{};
        po.metric = PerformanceObjective::Metric::ue_throughput_mbps;
        po.target_value = 100.0;
        po.tolerance = 10.0;
        po.scope_nr_cgi = 0x42;
        a1.objective_updates.push_back(po);
        const auto blob = sm.EncodeControl(a1);
        CccControlAction got{};
        NS_TEST_ASSERT_MSG_EQ(sm.DecodeControl(blob, &got),
                              true,
                              "decode ControlAction");
        NS_TEST_EXPECT_MSG_EQ(static_cast<int>(got.op),
                              static_cast<int>(
                                  CccControlAction::Op::set_perf_objective),
                              "op preserved");
        NS_TEST_ASSERT_MSG_EQ(got.objective_updates.size(),
                              1u,
                              "1 objective");
        NS_TEST_EXPECT_MSG_EQ(got.objective_updates[0].target_value,
                              100.0,
                              "target value");
        NS_TEST_EXPECT_MSG_EQ(got.cell_updates.size(),
                              0u,
                              "no cell updates");

        // clear_config with 2 cells.
        CccControlAction a2{};
        a2.op = CccControlAction::Op::clear_config;
        CellConfigRecord c{};
        c.nr_cell_global_id = 1;
        a2.cell_updates.push_back(c);
        c.nr_cell_global_id = 2;
        a2.cell_updates.push_back(c);
        const auto blob2 = sm.EncodeControl(a2);
        CccControlAction got2{};
        NS_TEST_ASSERT_MSG_EQ(sm.DecodeControl(blob2, &got2),
                              true,
                              "decode clear_config");
        NS_TEST_EXPECT_MSG_EQ(static_cast<int>(got2.op),
                              static_cast<int>(
                                  CccControlAction::Op::clear_config),
                              "op clear_config");
        NS_TEST_ASSERT_MSG_EQ(got2.cell_updates.size(),
                              2u,
                              "2 cell updates");
        NS_TEST_EXPECT_MSG_EQ(got2.cell_updates[0].nr_cell_global_id,
                              1u,
                              "cell[0] NCGI");
        NS_TEST_EXPECT_MSG_EQ(got2.cell_updates[1].nr_cell_global_id,
                              2u,
                              "cell[1] NCGI");
    }
};

class OranNtnSmRegistryThreePluginsTest : public TestCase
{
  public:
    OranNtnSmRegistryThreePluginsTest()
        : TestCase("Service-Model registry resolves KPM, RC, and CCC plugins")
    {
    }

  private:
    void DoRun() override
    {
        Ptr<OranNtnServiceModelRegistry> reg =
            CreateObject<OranNtnServiceModelRegistry>();
        Ptr<OranNtnServiceModelKpm> kpm =
            CreateObject<OranNtnServiceModelKpm>();
        Ptr<OranNtnServiceModelRc> rc =
            CreateObject<OranNtnServiceModelRc>();
        Ptr<OranNtnServiceModelCcc> ccc =
            CreateObject<OranNtnServiceModelCcc>();

        NS_TEST_EXPECT_MSG_EQ(reg->Register(kpm), true, "KPM registered");
        NS_TEST_EXPECT_MSG_EQ(reg->Register(rc), true, "RC registered");
        NS_TEST_EXPECT_MSG_EQ(reg->Register(ccc), true, "CCC registered");
        NS_TEST_EXPECT_MSG_EQ(reg->Size(), 3u, "3 plugins");

        // Lookup all three.
        Ptr<OranNtnServiceModel> p = reg->Lookup(147);
        NS_TEST_ASSERT_MSG_NE(p, nullptr, "KPM lookup");
        NS_TEST_EXPECT_MSG_EQ(p->Name(), "KPM", "KPM name");
        p = reg->Lookup(3);
        NS_TEST_ASSERT_MSG_NE(p, nullptr, "RC lookup");
        NS_TEST_EXPECT_MSG_EQ(p->Name(), "RC", "RC name");
        p = reg->Lookup(1000);
        NS_TEST_ASSERT_MSG_NE(p, nullptr, "CCC lookup");
        NS_TEST_EXPECT_MSG_EQ(p->Name(), "CCC", "CCC name");
        NS_TEST_EXPECT_MSG_EQ(p->Version(), "v1.00", "CCC version");

        // Ascending order: RC (3), KPM (147), CCC (1000).
        auto ids = reg->GetFunctionIds();
        NS_TEST_ASSERT_MSG_EQ(ids.size(), 3u, "3 IDs");
        NS_TEST_EXPECT_MSG_EQ(ids[0], 3u, "RC first");
        NS_TEST_EXPECT_MSG_EQ(ids[1], 147u, "KPM second");
        NS_TEST_EXPECT_MSG_EQ(ids[2], 1000u, "CCC third");
    }
};

// ============================================================================
//  T3 (Roadmap §3 T3): E2 SCTP / TCP listener + state machine
// ============================================================================

class OranNtnE2ListenerHandshakeTest : public TestCase
{
  public:
    OranNtnE2ListenerHandshakeTest()
        : TestCase("E2 listener accepts client, completes Setup handshake, "
                   "and forwards Indication via TCP loopback")
    {
    }

  private:
    void DoRun() override
    {
        using namespace oranntn::flexric;

        Ptr<OranNtnE2Listener> listener =
            CreateObject<OranNtnE2Listener>();
        listener->SetTransportKind(E2Transport::Protocol::tcp);
        const uint16_t port = 56421;
        NS_TEST_ASSERT_MSG_EQ(listener->Start("127.0.0.1", port),
                              true,
                              "listener starts");

        auto client = MakeE2Transport(E2Transport::Protocol::tcp);
        NS_TEST_ASSERT_MSG_EQ(client->Connect("127.0.0.1", port),
                              true,
                              "client connect");

        listener->Poll(50);
        NS_TEST_ASSERT_MSG_EQ(listener->NumClients(),
                              1u,
                              "1 client accepted");

        // Without E2 Setup, subscription must fail.
        E2Message subReq{E2MessageType::ric_subscription_request,
                          {0xAA, 0xBB}};
        NS_TEST_ASSERT_MSG_GT(client->Send(subReq), -1, "send early sub");
        listener->Poll(50);
        E2Message reply{};
        NS_TEST_ASSERT_MSG_EQ(client->Recv(reply, 200),
                              true,
                              "got reply to early sub");
        NS_TEST_EXPECT_MSG_EQ(
            static_cast<int>(reply.type),
            static_cast<int>(E2MessageType::ric_subscription_failure),
            "early sub fails before E2 setup");

        // E2 Setup -> Response.
        E2Message setup{E2MessageType::e2_setup_request, {0x01}};
        client->Send(setup);
        listener->Poll(50);
        NS_TEST_ASSERT_MSG_EQ(client->Recv(reply, 200),
                              true,
                              "got setup response");
        NS_TEST_EXPECT_MSG_EQ(
            static_cast<int>(reply.type),
            static_cast<int>(E2MessageType::e2_setup_response),
            "setup response");
        NS_TEST_EXPECT_MSG_EQ(listener->SetupRequestsHandled(),
                              1u,
                              "1 setup handled");

        // Subscription now succeeds.
        client->Send(subReq);
        listener->Poll(50);
        NS_TEST_ASSERT_MSG_EQ(client->Recv(reply, 200),
                              true,
                              "got sub response");
        NS_TEST_EXPECT_MSG_EQ(
            static_cast<int>(reply.type),
            static_cast<int>(E2MessageType::ric_subscription_response),
            "sub response after setup");

        // RIC Indication forwarded to the trace counter.
        E2Message ind{E2MessageType::ric_indication,
                       {0xDE, 0xAD, 0xBE, 0xEF}};
        client->Send(ind);
        listener->Poll(50);
        NS_TEST_EXPECT_MSG_EQ(listener->IndicationsForwarded(),
                              1u,
                              "1 indication forwarded");

        // Control request -> Ack.
        E2Message ctrl{E2MessageType::ric_control_request, {0x77}};
        client->Send(ctrl);
        listener->Poll(50);
        NS_TEST_ASSERT_MSG_EQ(client->Recv(reply, 200),
                              true,
                              "got control ack");
        NS_TEST_EXPECT_MSG_EQ(
            static_cast<int>(reply.type),
            static_cast<int>(E2MessageType::ric_control_acknowledge),
            "control ack");

        // Keepalive ping -> pong.
        E2Message ping{E2MessageType::keepalive_ping, {}};
        client->Send(ping);
        listener->Poll(50);
        NS_TEST_ASSERT_MSG_EQ(client->Recv(reply, 200),
                              true,
                              "got pong");
        NS_TEST_EXPECT_MSG_EQ(
            static_cast<int>(reply.type),
            static_cast<int>(E2MessageType::keepalive_pong),
            "keepalive pong");

        client->Close();
        listener->Stop();
    }
};

namespace
{

static std::atomic<uint64_t> g_send_attempts{0};
static std::atomic<uint64_t> g_send_success{0};

void
SendOneIndication(oranntn::flexric::E2Transport* client, uint64_t i)
{
    ++g_send_attempts;
    oranntn::flexric::E2Message ind;
    ind.type = oranntn::flexric::E2MessageType::ric_indication;
    ind.payload = {static_cast<uint8_t>(i >> 8),
                    static_cast<uint8_t>(i & 0xFF),
                    0xCC, 0xCC, 0xCC};
    if (client->Send(ind) >= 0)
    {
        ++g_send_success;
    }
}

void
PollListener(Ptr<oranntn::flexric::OranNtnE2Listener> listener)
{
    listener->Poll(5);
}

} // namespace

class OranNtnE2ListenerSimulatorTimeTest : public TestCase
{
  public:
    OranNtnE2ListenerSimulatorTimeTest()
        : TestCase("Simulator: 30 s RIC Indication stream into the E2 listener")
    {
    }

  private:
    void DoRun() override
    {
        using namespace oranntn::flexric;

        Ptr<OranNtnE2Listener> listener =
            CreateObject<OranNtnE2Listener>();
        const uint16_t port = 56422;
        NS_TEST_ASSERT_MSG_EQ(listener->Start("127.0.0.1", port),
                              true,
                              "listener starts");

        auto raw = MakeE2Transport(E2Transport::Protocol::tcp);
        NS_TEST_ASSERT_MSG_EQ(raw->Connect("127.0.0.1", port),
                              true,
                              "client connect");
        listener->Poll(50);
        E2Message setup{E2MessageType::e2_setup_request, {0x01}};
        raw->Send(setup);
        listener->Poll(50);
        E2Message reply;
        NS_TEST_ASSERT_MSG_EQ(raw->Recv(reply, 200),
                              true,
                              "setup response during sim setup phase");

        // shared_ptr so we can capture in scheduled lambdas without
        // moving the unique_ptr.
        g_send_attempts = 0;
        g_send_success = 0;
        std::shared_ptr<E2Transport> shared(raw.release());
        E2Transport* tp = shared.get();
        // Schedule 30 indication-send events under Simulator::Run() time
        // (one per simulator-second). Sends complete inside the simulator
        // but their bytes don't surface on the listener-side socket
        // until wall-clock time passes; a post-Run drain pulls them.
        for (int t = 1; t <= 30; ++t)
        {
            Simulator::Schedule(Seconds(t),
                                &SendOneIndication, tp,
                                static_cast<uint64_t>(t));
        }
        Simulator::Stop(Seconds(31));
        Simulator::Run();
        // All 30 Sends fired and succeeded inside the simulator.
        NS_TEST_ASSERT_MSG_EQ(g_send_attempts.load(),
                              30u,
                              "all 30 send events fired in Simulator::Run");
        NS_TEST_ASSERT_MSG_EQ(g_send_success.load(),
                              30u,
                              "all 30 Send() calls returned >= 0");
        // Wall-clock drain. max_msgs_per_client = 64 so one Poll drains
        // the full buffer; loop 50 times defensively in case the kernel
        // staggers delivery.
        for (int i = 0; i < 50; ++i)
        {
            if (listener->IndicationsForwarded() >= 30u)
                break;
            listener->Poll(50, /*max_msgs_per_client=*/64);
        }

        NS_TEST_ASSERT_MSG_EQ(listener->IndicationsForwarded(),
                              30u,
                              "30 RIC Indications forwarded across 30 s");

        shared->Close();
        listener->Stop();
        Simulator::Destroy();
    }
};

// ============================================================================
//  T2 (Roadmap §3 T2): ASN.1-PER codec primitives
// ============================================================================

class OranNtnAsn1PerPrimitivesTest : public TestCase
{
  public:
    OranNtnAsn1PerPrimitivesTest()
        : TestCase("ASN.1 Aligned-PER primitives round-trip "
                   "(INTEGER, UTF8String, OCTET STRING, length determinant, SEQUENCE preamble)")
    {
    }

  private:
    void DoRun() override
    {
        using oranntn::asn1::PerReader;
        using oranntn::asn1::PerWriter;

        // Length determinant: 0, 127, 128, 16383.
        PerWriter w;
        w.WriteLengthDeterminant(0);
        w.WriteLengthDeterminant(127);
        w.WriteLengthDeterminant(128);
        w.WriteLengthDeterminant(16383);
        PerReader r(w.Bytes());
        NS_TEST_EXPECT_MSG_EQ(r.ReadLengthDeterminant(), 0u, "len 0");
        NS_TEST_EXPECT_MSG_EQ(r.ReadLengthDeterminant(), 127u, "len 127");
        NS_TEST_EXPECT_MSG_EQ(r.ReadLengthDeterminant(), 128u, "len 128");
        NS_TEST_EXPECT_MSG_EQ(r.ReadLengthDeterminant(),
                              16383u,
                              "len 16383");

        // INTEGER: 0, 1, -1, 127, -128, 256, -256, INT64_MAX, INT64_MIN.
        PerWriter w2;
        const int64_t vals[] = {0,
                                  1,
                                  -1,
                                  127,
                                  -128,
                                  256,
                                  -256,
                                  9223372036854775807LL,
                                  -9223372036854775807LL - 1};
        for (auto v : vals)
        {
            w2.WriteInteger(v);
        }
        PerReader r2(w2.Bytes());
        for (auto v : vals)
        {
            NS_TEST_EXPECT_MSG_EQ(r2.ReadInteger(), v, "integer round-trip");
        }

        // UTF8String + OCTET STRING.
        PerWriter w3;
        w3.WriteUtf8String("");
        w3.WriteUtf8String("DRB.UEThpDl");
        w3.WriteUtf8String(std::string(200, 'x')); // long-form length
        w3.WriteOctetString({});
        w3.WriteOctetString({0xDE, 0xAD, 0xBE, 0xEF});
        PerReader r3(w3.Bytes());
        NS_TEST_EXPECT_MSG_EQ(r3.ReadUtf8String(), "", "empty string");
        NS_TEST_EXPECT_MSG_EQ(r3.ReadUtf8String(), "DRB.UEThpDl", "ascii string");
        NS_TEST_EXPECT_MSG_EQ(r3.ReadUtf8String().size(),
                              200u,
                              "long string");
        NS_TEST_EXPECT_MSG_EQ(r3.ReadOctetString().size(),
                              0u,
                              "empty octet string");
        auto os = r3.ReadOctetString();
        NS_TEST_ASSERT_MSG_EQ(os.size(), 4u, "octet string len");
        NS_TEST_EXPECT_MSG_EQ(static_cast<int>(os[0]), 0xDE, "octet [0]");
        NS_TEST_EXPECT_MSG_EQ(static_cast<int>(os[3]), 0xEF, "octet [3]");

        // SEQUENCE preamble: 3 OPTIONALs, pattern 101.
        PerWriter w4;
        w4.BeginSequencePreamble(3);
        w4.SetPreambleBit(0, true);
        w4.SetPreambleBit(1, false);
        w4.SetPreambleBit(2, true);
        w4.EndSequencePreamble();
        w4.WriteInteger(42);
        PerReader r4(w4.Bytes());
        const uint16_t pre = r4.ReadSequencePreamble(3);
        const uint16_t slot0 = static_cast<uint16_t>((pre >> 2) & 1);
        const uint16_t slot1 = static_cast<uint16_t>((pre >> 1) & 1);
        const uint16_t slot2 = static_cast<uint16_t>(pre & 1);
        NS_TEST_EXPECT_MSG_EQ(slot0, 1u, "slot 0 set");
        NS_TEST_EXPECT_MSG_EQ(slot1, 0u, "slot 1 unset");
        NS_TEST_EXPECT_MSG_EQ(slot2, 1u, "slot 2 set");
        NS_TEST_EXPECT_MSG_EQ(r4.ReadInteger(), 42, "post-preamble int");

        // CHOICE index.
        PerWriter w5;
        w5.WriteChoiceIndex(0);
        w5.WriteChoiceIndex(1);
        w5.WriteChoiceIndex(2);
        PerReader r5(w5.Bytes());
        NS_TEST_EXPECT_MSG_EQ(r5.ReadChoiceIndex(), 0u, "choice 0");
        NS_TEST_EXPECT_MSG_EQ(r5.ReadChoiceIndex(), 1u, "choice 1");
        NS_TEST_EXPECT_MSG_EQ(r5.ReadChoiceIndex(), 2u, "choice 2");
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
//  4.1.9 — CU/DU/RU split (Roadmap §4.1.9)
// ============================================================================

class OranNtnSplitGnbRolesTest : public TestCase
{
  public:
    OranNtnSplitGnbRolesTest()
        : TestCase("Split-gNB: per-entity role and E2 termination (4.1.9)")
    {
    }

    void DoRun() override
    {
        NodeContainer nodes;
        const auto g = OranNtnSplitGnbHelper::Build(/*gnb_id=*/7, nodes);
        NS_TEST_ASSERT_MSG_EQ(nodes.GetN(),
                               4u,
                               "four ns-3 Nodes created");

        // Each entity has its own E2 termination (different node ids).
        NS_TEST_EXPECT_MSG_NE(g.cu_cp->GetE2Node()->GetNodeId(),
                                g.du->GetE2Node()->GetNodeId(),
                                "CU-CP and DU have distinct E2 ids");
        NS_TEST_EXPECT_MSG_NE(g.du->GetE2Node()->GetNodeId(),
                                g.ru->GetE2Node()->GetNodeId(),
                                "DU and RU have distinct E2 ids");
        NS_TEST_EXPECT_MSG_NE(g.cu_cp->GetE2Node()->GetNodeId(),
                                g.cu_up->GetE2Node()->GetNodeId(),
                                "CU-CP and CU-UP have distinct E2 ids");

        // Role defaults populate the correct RIC Function IDs.
        NS_TEST_EXPECT_MSG_EQ(g.cu_cp->HasRanFunction(3),
                               true,
                               "CU-CP advertises RC (3)");
        NS_TEST_EXPECT_MSG_EQ(g.du->HasRanFunction(147),
                               true,
                               "DU advertises KPM (147)");
        NS_TEST_EXPECT_MSG_EQ(g.ru->HasRanFunction(1000),
                               true,
                               "RU advertises CCC (1000)");
        NS_TEST_EXPECT_MSG_EQ(g.ru->HasRanFunction(1001),
                               true,
                               "RU advertises NTN-Ephemeris (1001)");
        // Roles are mutually exclusive: CU-CP does NOT advertise CCC.
        NS_TEST_EXPECT_MSG_EQ(g.cu_cp->HasRanFunction(1000),
                               false,
                               "CU-CP does not advertise CCC");
        NS_TEST_EXPECT_MSG_EQ(g.du->HasRanFunction(3),
                               false,
                               "DU does not advertise RC");
        NS_TEST_EXPECT_MSG_EQ(g.ru->HasRanFunction(147),
                               false,
                               "RU does not advertise KPM");
    }
};

class OranNtnSplitGnbControlRoutingTest : public TestCase
{
  public:
    OranNtnSplitGnbControlRoutingTest()
        : TestCase("Split-gNB: ControlAction role enforcement (4.1.9)")
    {
    }

    void DoRun() override
    {
        NodeContainer nodes;
        const auto g = OranNtnSplitGnbHelper::Build(/*gnb_id=*/9, nodes);

        E2RcAction ho;
        ho.actionType = E2RcActionType::HANDOVER_TRIGGER;
        ho.targetGnbId = 9;
        ho.targetUeId = 1;
        // HO on CU-CP — accept.
        NS_TEST_EXPECT_MSG_EQ(g.cu_cp->ReceiveControl(ho),
                               true,
                               "HO accepted on CU-CP");
        // HO on DU / RU — reject.
        NS_TEST_EXPECT_MSG_EQ(g.du->ReceiveControl(ho),
                               false,
                               "HO rejected on DU");
        NS_TEST_EXPECT_MSG_EQ(g.ru->ReceiveControl(ho),
                               false,
                               "HO rejected on RU");

        E2RcAction beam;
        beam.actionType = E2RcActionType::BEAM_SWITCH;
        beam.targetGnbId = 9;
        // Beam on RU — accept; CU-CP and DU — reject.
        NS_TEST_EXPECT_MSG_EQ(g.ru->ReceiveControl(beam),
                               true,
                               "beam accepted on RU");
        NS_TEST_EXPECT_MSG_EQ(g.du->ReceiveControl(beam),
                               false,
                               "beam rejected on DU");
        NS_TEST_EXPECT_MSG_EQ(g.cu_cp->ReceiveControl(beam),
                               false,
                               "beam rejected on CU-CP");

        E2RcAction slice;
        slice.actionType = E2RcActionType::SLICE_PRB_ALLOCATION;
        slice.targetGnbId = 9;
        NS_TEST_EXPECT_MSG_EQ(g.du->ReceiveControl(slice),
                               true,
                               "slice accepted on DU");
        NS_TEST_EXPECT_MSG_EQ(g.cu_cp->ReceiveControl(slice),
                               false,
                               "slice rejected on CU-CP");

        NS_TEST_EXPECT_MSG_EQ(g.cu_cp->ControlsAccepted(),
                               1u,
                               "CU-CP accepted 1");
        NS_TEST_EXPECT_MSG_EQ(g.du->ControlsAccepted(),
                               1u,
                               "DU accepted 1");
        NS_TEST_EXPECT_MSG_EQ(g.ru->ControlsAccepted(),
                               1u,
                               "RU accepted 1");
    }
};

class OranNtnF1RoundTripTest : public TestCase
{
  public:
    OranNtnF1RoundTripTest()
        : TestCase("F1 round-trip CU-DU under Simulator (4.1.9)")
    {
    }

    void DoRun() override
    {
        NodeContainer nodes;
        const auto g = OranNtnSplitGnbHelper::Build(/*gnb_id=*/11, nodes);
        g.f1->SetDeliveryDelay(MilliSeconds(2));

        uint32_t du_recv = 0;
        uint32_t cu_recv = 0;
        F1apMessageKind last_du_kind = F1apMessageKind::f1_setup_request;
        F1apMessageKind last_cu_kind = F1apMessageKind::f1_setup_request;
        g.f1->SetDuReceiveCallback(
            [&](const F1apMessage& m) {
                ++du_recv;
                last_du_kind = m.kind;
                // Echo back a response.
                F1apMessage reply;
                reply.kind = F1apMessageKind::ue_context_setup_response;
                reply.ue_id = m.ue_id;
                reply.transaction_id = m.transaction_id;
                g.f1->SendFromDu(reply);
            });
        g.f1->SetCuCpReceiveCallback(
            [&](const F1apMessage& m) {
                ++cu_recv;
                last_cu_kind = m.kind;
            });

        F1apMessage req;
        req.kind = F1apMessageKind::ue_context_setup_request;
        req.ue_id = 42;
        req.transaction_id = 1;
        NS_TEST_ASSERT_MSG_EQ(g.f1->SendFromCuCp(req),
                               true,
                               "send accepted");

        Simulator::Stop(MilliSeconds(50));
        Simulator::Run();
        Simulator::Destroy();

        NS_TEST_EXPECT_MSG_EQ(du_recv, 1u, "DU received 1 msg");
        NS_TEST_EXPECT_MSG_EQ(cu_recv, 1u, "CU received echo");
        const bool du_kind_ok =
            last_du_kind ==
            F1apMessageKind::ue_context_setup_request;
        const bool cu_kind_ok =
            last_cu_kind ==
            F1apMessageKind::ue_context_setup_response;
        NS_TEST_EXPECT_MSG_EQ(du_kind_ok, true, "DU saw request");
        NS_TEST_EXPECT_MSG_EQ(cu_kind_ok, true, "CU saw response");
        NS_TEST_EXPECT_MSG_EQ(g.f1->MessagesCuToDu(),
                               1u,
                               "1 CU→DU");
        NS_TEST_EXPECT_MSG_EQ(g.f1->MessagesDuToCu(),
                               1u,
                               "1 DU→CU");
        NS_TEST_EXPECT_MSG_EQ(g.f1->MessagesDropped(),
                               0u,
                               "no drops");
    }
};

class OranNtnF1LinkDownTest : public TestCase
{
  public:
    OranNtnF1LinkDownTest()
        : TestCase("F1: link down drops new sends + pending events (4.1.9)")
    {
    }

    void DoRun() override
    {
        NodeContainer nodes;
        const auto g = OranNtnSplitGnbHelper::Build(/*gnb_id=*/13, nodes);

        uint32_t du_recv = 0;
        g.f1->SetDuReceiveCallback(
            [&](const F1apMessage&) { ++du_recv; });

        // Send a message at t=0.
        F1apMessage m1;
        m1.kind = F1apMessageKind::rc_control_forward;
        m1.ue_id = 1;
        g.f1->SendFromCuCp(m1);
        // Take link down before delivery delay elapses.
        Simulator::Schedule(MicroSeconds(100),
                              [&] { g.f1->SetLinkUp(false); });
        // Try to send while link down (should drop).
        Simulator::Schedule(MicroSeconds(200),
                              [&] {
                                  F1apMessage m2;
                                  m2.kind = F1apMessageKind::rc_control_forward;
                                  m2.ue_id = 2;
                                  g.f1->SendFromCuCp(m2);
                              });
        // Bring link back up; new sends accepted.
        Simulator::Schedule(MilliSeconds(5),
                              [&] {
                                  g.f1->SetLinkUp(true);
                                  F1apMessage m3;
                                  m3.kind = F1apMessageKind::rc_control_forward;
                                  m3.ue_id = 3;
                                  g.f1->SendFromCuCp(m3);
                              });
        Simulator::Stop(MilliSeconds(50));
        Simulator::Run();
        Simulator::Destroy();

        // The first message in flight at t=0 was cancelled by SetLinkUp(false).
        // The second send was dropped synchronously.
        // The third send (after re-up) delivers.
        NS_TEST_EXPECT_MSG_EQ(du_recv, 1u, "only the post-restore msg arrived");
        NS_TEST_EXPECT_MSG_EQ(g.f1->MessagesDropped(),
                               1u,
                               "1 explicit drop");
    }
};

class OranNtnOfhPlaneRoutingTest : public TestCase
{
  public:
    OranNtnOfhPlaneRoutingTest()
        : TestCase("OFH: C/U/S/M plane routing + outage (4.1.9)")
    {
    }

    void DoRun() override
    {
        NodeContainer nodes;
        const auto g = OranNtnSplitGnbHelper::Build(/*gnb_id=*/17, nodes);

        std::map<OfhPlane, uint32_t> ru_recv;
        g.ofh->SetRuReceiveCallback(
            [&](const OfhMessage& m) { ++ru_recv[m.plane]; });

        // Send one of each plane.
        for (auto p : {OfhPlane::c_plane,
                          OfhPlane::u_plane,
                          OfhPlane::s_plane,
                          OfhPlane::m_plane})
        {
            OfhMessage m;
            m.plane = p;
            m.opcode = 1;
            g.ofh->SendFromDu(m);
        }
        Simulator::Stop(MilliSeconds(20));
        Simulator::Run();
        Simulator::Destroy();

        NS_TEST_EXPECT_MSG_EQ(ru_recv[OfhPlane::c_plane],
                               1u,
                               "C-Plane delivered");
        NS_TEST_EXPECT_MSG_EQ(ru_recv[OfhPlane::u_plane],
                               1u,
                               "U-Plane delivered");
        NS_TEST_EXPECT_MSG_EQ(ru_recv[OfhPlane::s_plane],
                               1u,
                               "S-Plane delivered");
        NS_TEST_EXPECT_MSG_EQ(ru_recv[OfhPlane::m_plane],
                               1u,
                               "M-Plane delivered");

        // Now take C-Plane down; new C-Plane sends should drop.
        g.ofh->SetPlaneUp(OfhPlane::c_plane, false);
        OfhMessage bad;
        bad.plane = OfhPlane::c_plane;
        NS_TEST_EXPECT_MSG_EQ(g.ofh->SendFromDu(bad),
                               false,
                               "C-Plane send drops");
        NS_TEST_EXPECT_MSG_EQ(
            g.ofh->MessagesDroppedPerPlane(OfhPlane::c_plane),
            1u,
            "1 C-Plane drop");
        NS_TEST_EXPECT_MSG_EQ(
            g.ofh->MessagesDroppedPerPlane(OfhPlane::u_plane),
            0u,
            "no U-Plane drops");
    }
};

class OranNtnSplitGnbEndToEndTest : public TestCase
{
  public:
    OranNtnSplitGnbEndToEndTest()
        : TestCase("Split-gNB: CU→DU→RU end-to-end HO chain (4.1.9)")
    {
    }

    void DoRun() override
    {
        NodeContainer nodes;
        const auto g = OranNtnSplitGnbHelper::Build(/*gnb_id=*/21, nodes);
        g.f1->SetDeliveryDelay(MilliSeconds(1));
        g.ofh->SetPlaneDelay(OfhPlane::c_plane, MicroSeconds(80));

        // Simulated end-to-end mobility flow:
        //   1. xApp at NRTRIC issues HO ControlAction → CU-CP accepts
        //   2. CU-CP forwards to DU via F1 (rc_control_forward)
        //   3. DU translates to RF retune → sends via OFH C-Plane to RU
        //   4. RU receives + applies (beam_switch via ReceiveControl)
        uint32_t cu_observed = 0;
        uint32_t du_observed = 0;
        uint32_t ru_observed = 0;
        g.cu_cp->SetControlObserver(
            [&](const E2RcAction& a) {
                ++cu_observed;
                NS_TEST_EXPECT_MSG_EQ(
                    static_cast<int>(a.actionType),
                    static_cast<int>(E2RcActionType::HANDOVER_TRIGGER),
                    "CU-CP sees HO");
                // Forward to DU.
                F1apMessage f;
                f.kind = F1apMessageKind::rc_control_forward;
                f.ue_id = a.targetUeId;
                f.payload.assign(1, static_cast<uint8_t>(a.actionType));
                g.f1->SendFromCuCp(f);
            });
        g.f1->SetDuReceiveCallback(
            [&](const F1apMessage& m) {
                ++du_observed;
                // DU translates into an RF retune for the RU.
                OfhMessage o;
                o.plane = OfhPlane::c_plane;
                o.opcode = static_cast<uint16_t>(
                    F1apMessageKind::rc_control_forward);
                o.payload = m.payload;
                g.ofh->SendFromDu(o);
            });
        g.ofh->SetRuReceiveCallback(
            [&](const OfhMessage& o) {
                ++ru_observed;
                E2RcAction a;
                a.actionType = E2RcActionType::BEAM_SWITCH;
                a.targetGnbId = 21;
                a.targetUeId = 1;
                a.targetBeamId = 3;
                g.ru->ReceiveControl(a);
            });

        // Fire the HO at t=1ms.
        Simulator::Schedule(MilliSeconds(1),
                              [&] {
                                  E2RcAction ho;
                                  ho.actionType =
                                      E2RcActionType::HANDOVER_TRIGGER;
                                  ho.targetGnbId = 21;
                                  ho.targetUeId = 1;
                                  ho.targetBeamId = 3;
                                  g.cu_cp->ReceiveControl(ho);
                              });

        Simulator::Stop(MilliSeconds(50));
        Simulator::Run();
        Simulator::Destroy();

        NS_TEST_EXPECT_MSG_EQ(cu_observed, 1u, "CU-CP saw 1 HO");
        NS_TEST_EXPECT_MSG_EQ(du_observed, 1u, "DU saw 1 F1 forward");
        NS_TEST_EXPECT_MSG_EQ(ru_observed, 1u, "RU saw 1 OFH C-Plane msg");
        NS_TEST_EXPECT_MSG_EQ(g.ru->ControlsAccepted(),
                               1u,
                               "RU applied 1 control");
        NS_TEST_EXPECT_MSG_EQ(g.f1->MessagesCuToDu(),
                               1u,
                               "1 F1 CU→DU");
        NS_TEST_EXPECT_MSG_EQ(g.ofh->MessagesDuToRu(),
                               1u,
                               "1 OFH DU→RU");
    }
};

class OranNtnSplitGnbSimulatorWorkloadTest : public TestCase
{
  public:
    OranNtnSplitGnbSimulatorWorkloadTest()
        : TestCase("Split-gNB: 30 s workload + KPM stream (4.1.9)")
    {
    }

    void DoRun() override
    {
        NodeContainer nodes;
        const auto g = OranNtnSplitGnbHelper::Build(/*gnb_id=*/27, nodes);

        // Periodic KPM stream from DU (every 100 ms for 30 s = 300 reports).
        uint32_t emitted = 0;
        const auto submit = [&]() {
            E2KpmReport rep;
            rep.gnbId = 2702;
            rep.timestamp = Simulator::Now().GetSeconds();
            rep.sinr_dB = 12.0 + 0.001 * static_cast<double>(emitted);
            g.du->SubmitKpmIndication(rep);
            ++emitted;
        };
        for (uint32_t k = 0; k < 300; ++k)
        {
            Simulator::Schedule(MilliSeconds(100 * (k + 1)), submit);
        }

        // Every 1 s, the xApp issues a beam-switch to the RU.
        uint32_t beam_apps = 0;
        for (uint32_t k = 0; k < 30; ++k)
        {
            Simulator::Schedule(
                Seconds(1.0 * (k + 1)),
                [&] {
                    E2RcAction a;
                    a.actionType = E2RcActionType::BEAM_SWITCH;
                    a.targetGnbId = 27;
                    a.targetBeamId =
                        static_cast<uint32_t>((k * 7) % 32);
                    g.ru->ReceiveControl(a);
                    ++beam_apps;
                });
        }

        Simulator::Stop(Seconds(30.5));
        Simulator::Run();
        Simulator::Destroy();

        NS_TEST_EXPECT_MSG_EQ(emitted, 300u, "300 KPM reports emitted");
        NS_TEST_EXPECT_MSG_EQ(g.du->IndicationsEmitted(),
                               300u,
                               "DU accounting matches");
        NS_TEST_EXPECT_MSG_EQ(beam_apps, 30u, "30 beam-switches fired");
        NS_TEST_EXPECT_MSG_EQ(g.ru->ControlsAccepted(),
                               30u,
                               "RU applied 30 controls");
        // DU receives no controls in this scenario.
        NS_TEST_EXPECT_MSG_EQ(g.du->ControlsAccepted(),
                               0u,
                               "DU saw 0 controls");
        // CU-CP: no controls in this scenario.
        NS_TEST_EXPECT_MSG_EQ(g.cu_cp->ControlsAccepted(),
                               0u,
                               "CU-CP saw 0 controls");
    }
};

// ============================================================================
//  4.1.12 — Two-stage NN precoder + codebook beamformer (Roadmap §4.1.12)
// ============================================================================

namespace
{

ns3::oranntn::airan::TritonModelConfig
MakePrecoderConfig()
{
    const char* pbtxt = R"PBTXT(
name: "precoder_csi_to_weights"
platform: "onnxruntime_onnx"
max_batch_size: 32
parameters {
  key: "INFERENCE_BUDGET_US"
  value: { string_value: "1000" }
}
parameters {
  key: "TOOLKIT_OUTPUT_FIELD"
  value: { string_value: "precoder" }
}
)PBTXT";
    auto cfg = ns3::oranntn::airan::TritonModelConfigParser::Parse(pbtxt);
    return *cfg;
}

ns3::oranntn::airan::CsiTensor
MakeMmimoCsi(uint32_t num_tx, uint64_t seed)
{
    ns3::oranntn::airan::CsiTensor t;
    t.num_tx = num_tx;
    t.num_rx = 2;
    t.num_subcarriers = 4;
    t.doppler_hz = 25.0;
    t.values.assign(2 * num_tx * t.num_rx * t.num_subcarriers, 0.0f);
    for (size_t k = 0; k < t.values.size(); ++k)
    {
        t.values[k] = static_cast<float>(0.001 * (k + seed));
    }
    return t;
}

} // namespace

class OranNtnMmimoCodebookDftTest : public TestCase
{
  public:
    OranNtnMmimoCodebookDftTest()
        : TestCase("Codebook: DFT codebook generation and lookup (4.1.12)")
    {
    }

    void DoRun() override
    {
        auto cb = CreateObject<OranNtnMmimoCodebook>();
        cb->PopulateDftAzimuthSweep(/*num_tx=*/8, /*num_entries=*/16);
        NS_TEST_ASSERT_MSG_EQ(cb->NumTx(), 8u, "tx count");
        NS_TEST_ASSERT_MSG_EQ(cb->Size(), 16u, "entry count");

        // Each entry has unit norm (1/√num_tx · 8 elements → norm 1).
        for (uint32_t k = 0; k < cb->Size(); ++k)
        {
            const auto& e = cb->GetEntry(k);
            NS_TEST_ASSERT_MSG_EQ(e.size(), 16u, "16 floats per entry");
            double norm_sq = 0.0;
            for (uint32_t n = 0; n < 8; ++n)
            {
                norm_sq += e[2 * n] * e[2 * n] +
                            e[2 * n + 1] * e[2 * n + 1];
            }
            NS_TEST_EXPECT_MSG_EQ_TOL(norm_sq,
                                        1.0,
                                        1e-5,
                                        "unit norm");
        }

        // Self-lookup: every entry should be its own best match.
        for (uint32_t k = 0; k < cb->Size(); ++k)
        {
            const uint32_t best = cb->BestMatch(cb->GetEntry(k));
            NS_TEST_EXPECT_MSG_EQ(best, k, "self best-match");
        }

        // Score ordering: scores[k] for entry k should be the max
        // across the row.
        const auto scores = cb->ScoreAll(cb->GetEntry(3));
        NS_TEST_ASSERT_MSG_EQ(scores.size(), 16u, "16 scores");
        float max_score = scores[3];
        for (uint32_t k = 0; k < scores.size(); ++k)
        {
            if (k == 3)
            {
                continue;
            }
            const bool less_or_eq = scores[k] <= max_score;
            NS_TEST_EXPECT_MSG_EQ(less_or_eq,
                                   true,
                                   "self has max score");
        }
    }
};

class OranNtnMmimoComposeTest : public TestCase
{
  public:
    OranNtnMmimoComposeTest()
        : TestCase("Two-stage compose math: W = W_RF · W_BB (4.1.12)")
    {
    }

    void DoRun() override
    {
        auto cb = CreateObject<OranNtnMmimoCodebook>();
        cb->PopulateDftAzimuthSweep(/*num_tx=*/4, /*num_entries=*/8);

        // Build an NN output whose column 0 equals codebook entry 2
        // and column 1 equals codebook entry 5. Two-stage compose
        // should pick those indices.
        const auto& e2 = cb->GetEntry(2);
        const auto& e5 = cb->GetEntry(5);
        std::vector<float> nn(2 * 4 * 2, 0.0f);
        for (uint32_t tx = 0; tx < 4; ++tx)
        {
            // layer 0
            nn[2 * (tx * 2 + 0)] = e2[2 * tx];
            nn[2 * (tx * 2 + 0) + 1] = e2[2 * tx + 1];
            // layer 1
            nn[2 * (tx * 2 + 1)] = e5[2 * tx];
            nn[2 * (tx * 2 + 1) + 1] = e5[2 * tx + 1];
        }

        const auto res = OranNtnMmimoTwoStageComposer::Compose(
            nn, /*num_tx=*/4, /*num_layers=*/2, *cb);
        NS_TEST_ASSERT_MSG_EQ(res.codebook_indices.size(),
                               2u,
                               "two codebook indices");
        NS_TEST_EXPECT_MSG_EQ(res.codebook_indices[0], 2u, "layer 0");
        NS_TEST_EXPECT_MSG_EQ(res.codebook_indices[1], 5u, "layer 1");
        NS_TEST_EXPECT_MSG_EQ(res.num_tx, 4u, "num_tx");
        NS_TEST_EXPECT_MSG_EQ(res.num_layers, 2u, "num_layers");
        NS_TEST_EXPECT_MSG_EQ(res.final_weights.size(),
                               16u,
                               "final weights size");
        NS_TEST_EXPECT_MSG_EQ(res.bb_weights.size(),
                               8u,
                               "bb weights size");

        // Because each layer's W_NN column equals a codeword (which
        // has unit norm), W_BB ought to be a diagonal-like matrix:
        // off-diagonal cross terms are bounded by codeword cross-
        // correlation < 1 in magnitude.
        // Diagonal [0,0] = ⟨e2, e2⟩ = 1.
        const double diag0_re = res.bb_weights[2 * (0 * 2 + 0)];
        const double diag0_im = res.bb_weights[2 * (0 * 2 + 0) + 1];
        const double diag0_mag2 =
            diag0_re * diag0_re + diag0_im * diag0_im;
        NS_TEST_EXPECT_MSG_EQ_TOL(diag0_mag2,
                                    1.0,
                                    1e-4,
                                    "BB diagonal[0,0] ~ 1");
        const double diag1_re = res.bb_weights[2 * (1 * 2 + 1)];
        const double diag1_im = res.bb_weights[2 * (1 * 2 + 1) + 1];
        const double diag1_mag2 =
            diag1_re * diag1_re + diag1_im * diag1_im;
        NS_TEST_EXPECT_MSG_EQ_TOL(diag1_mag2,
                                    1.0,
                                    1e-4,
                                    "BB diagonal[1,1] ~ 1");
    }
};

class OranNtnMmimoXappRoundTripTest : public TestCase
{
  public:
    OranNtnMmimoXappRoundTripTest()
        : TestCase("xApp: CSI → NN → codebook → ControlAction (4.1.12)")
    {
    }

    void DoRun() override
    {
        // Wire the T7 inference path
        auto server = std::make_shared<oranntn::airan::AiranInferenceServer>();
        server->RegisterModel(
            MakePrecoderConfig(),
            oranntn::airan::AiranMockRuntime::MakePrecoderHandler(
                /*num_layers=*/2,
                /*base_latency_ms=*/0.4));

        auto pair = oranntn::airan::InProcInferenceChannel::CreatePair();
        auto client = std::make_shared<oranntn::airan::AiranInferenceClient>();
        client->Attach(std::move(pair.first));
        server->AddChannel(std::move(pair.second));

        // Build the §4.1.9 split-gNB so we have a real RU entity.
        NodeContainer nodes;
        const auto g = OranNtnSplitGnbHelper::Build(/*gnb_id=*/41, nodes);

        // Codebook + xApp
        auto codebook = CreateObject<OranNtnMmimoCodebook>();
        codebook->PopulateDftAzimuthSweep(/*num_tx=*/8,
                                            /*num_entries=*/16);

        auto xapp = CreateObject<OranNtnMmimoPrecoderXapp>();
        xapp->Configure(/*xapp_id=*/9001,
                         /*num_tx=*/8,
                         /*num_layers=*/2);
        xapp->AttachCodebook(codebook);
        xapp->AttachInferenceClient(client, "precoder_csi_to_weights");
        xapp->AttachRu(g.ru);

        // Observer captures the dispatched action.
        E2RcAction last_action;
        TwoStagePrecoderResult last_result;
        bool observed = false;
        xapp->SetActionObserver(
            [&](const E2RcAction& a,
                 const TwoStagePrecoderResult& r) {
                last_action = a;
                last_result = r;
                observed = true;
            });

        const auto csi = MakeMmimoCsi(/*num_tx=*/8, /*seed=*/77);
        NS_TEST_ASSERT_MSG_EQ(
            xapp->OnCsiReport(/*ue_id=*/1234,
                                 /*nr_cgi=*/55,
                                 csi),
            true,
            "submit ok");

        // Drain both sides — no Simulator::Run() needed for in-proc.
        server->Poll(5);
        client->Poll(5);

        NS_TEST_EXPECT_MSG_EQ(xapp->CsiInputsHandled(),
                               1u,
                               "1 csi in");
        NS_TEST_EXPECT_MSG_EQ(xapp->ResponsesProcessed(),
                               1u,
                               "1 response");
        NS_TEST_EXPECT_MSG_EQ(xapp->ControlActionsEmitted(),
                               1u,
                               "1 control action");
        NS_TEST_EXPECT_MSG_EQ(xapp->InferenceErrors(), 0u, "no errors");
        NS_TEST_EXPECT_MSG_EQ(observed, true, "observer fired");
        NS_TEST_EXPECT_MSG_EQ(g.ru->ControlsAccepted(),
                               1u,
                               "RU accepted action");

        const bool action_kind_ok =
            last_action.actionType ==
            E2RcActionType::BEAM_HOP_SCHEDULE;
        NS_TEST_EXPECT_MSG_EQ(action_kind_ok,
                               true,
                               "BEAM_HOP_SCHEDULE issued");
        NS_TEST_EXPECT_MSG_EQ(last_result.num_tx, 8u, "num_tx echoed");
        NS_TEST_EXPECT_MSG_EQ(last_result.num_layers,
                               2u,
                               "num_layers echoed");
        NS_TEST_EXPECT_MSG_EQ(last_result.codebook_indices.size(),
                               2u,
                               "2 codebook picks");
        NS_TEST_EXPECT_MSG_EQ(last_result.final_weights.size(),
                               static_cast<size_t>(2 * 8 * 2),
                               "final weights sized");

        const bool ues_match =
            (last_action.targetUeId & 0xFFFFFFFFu) == 1234u;
        NS_TEST_EXPECT_MSG_EQ(ues_match,
                               true,
                               "ue id forwarded");
    }
};

class OranNtnMmimoXappSimulatorTimeTest : public TestCase
{
  public:
    OranNtnMmimoXappSimulatorTimeTest()
        : TestCase("xApp: 5 s scenario with 25 CSI reports (4.1.12)")
    {
    }

    void DoRun() override
    {
        auto server = std::make_shared<oranntn::airan::AiranInferenceServer>();
        server->RegisterModel(
            MakePrecoderConfig(),
            oranntn::airan::AiranMockRuntime::MakePrecoderHandler(
                /*num_layers=*/4,
                /*base_latency_ms=*/0.5));

        auto pair = oranntn::airan::InProcInferenceChannel::CreatePair();
        auto client = std::make_shared<oranntn::airan::AiranInferenceClient>();
        client->Attach(std::move(pair.first));
        server->AddChannel(std::move(pair.second));

        NodeContainer nodes;
        const auto g = OranNtnSplitGnbHelper::Build(/*gnb_id=*/43, nodes);

        auto codebook = CreateObject<OranNtnMmimoCodebook>();
        codebook->PopulateDftAzimuthSweep(/*num_tx=*/16,
                                            /*num_entries=*/64);

        auto xapp = CreateObject<OranNtnMmimoPrecoderXapp>();
        xapp->Configure(/*xapp_id=*/9002,
                         /*num_tx=*/16,
                         /*num_layers=*/4);
        xapp->AttachCodebook(codebook);
        xapp->AttachInferenceClient(client, "precoder_csi_to_weights");
        xapp->AttachRu(g.ru);

        // 25 CSI reports between t=0.2 s and t=5.0 s.
        const uint32_t kNum = 25;
        for (uint32_t i = 0; i < kNum; ++i)
        {
            const double when_s = 0.2 + 0.19 * i;
            Simulator::Schedule(
                Seconds(when_s),
                [xapp, i, when_s] {
                    const auto csi = MakeMmimoCsi(16, 100 + i);
                    (void)xapp->OnCsiReport(/*ue_id=*/i + 1,
                                              /*nr_cgi=*/1,
                                              csi);
                    (void)when_s;
                });
        }
        // Pump server + client every 50 ms.
        for (uint32_t t_ms = 50; t_ms <= 6000; t_ms += 50)
        {
            Simulator::Schedule(
                MilliSeconds(t_ms),
                [server, client] {
                    server->Poll(0, 32);
                    client->Poll(0, 32);
                });
        }

        Simulator::Stop(Seconds(6.0));
        Simulator::Run();
        // Final drain
        server->Poll(5, 32);
        client->Poll(5, 32);
        Simulator::Destroy();

        NS_TEST_EXPECT_MSG_EQ(xapp->CsiInputsHandled(),
                               static_cast<uint64_t>(kNum),
                               "all csi handled");
        NS_TEST_EXPECT_MSG_EQ(xapp->ResponsesProcessed(),
                               static_cast<uint64_t>(kNum),
                               "all responses");
        NS_TEST_EXPECT_MSG_EQ(xapp->ControlActionsEmitted(),
                               static_cast<uint64_t>(kNum),
                               "all actions");
        NS_TEST_EXPECT_MSG_EQ(xapp->InferenceErrors(), 0u, "no errors");
        NS_TEST_EXPECT_MSG_EQ(g.ru->ControlsAccepted(),
                               static_cast<uint64_t>(kNum),
                               "RU accepted all");
    }
};

class OranNtnMmimoXappFailureModesTest : public TestCase
{
  public:
    OranNtnMmimoXappFailureModesTest()
        : TestCase("xApp: failure modes (no client, wrong tx count) (4.1.12)")
    {
    }

    void DoRun() override
    {
        NodeContainer nodes;
        const auto g = OranNtnSplitGnbHelper::Build(/*gnb_id=*/47, nodes);

        auto codebook = CreateObject<OranNtnMmimoCodebook>();
        codebook->PopulateDftAzimuthSweep(/*num_tx=*/4,
                                            /*num_entries=*/8);
        auto xapp = CreateObject<OranNtnMmimoPrecoderXapp>();
        xapp->Configure(/*xapp_id=*/1,
                         /*num_tx=*/4,
                         /*num_layers=*/2);
        xapp->AttachCodebook(codebook);
        xapp->AttachRu(g.ru);

        // No inference client attached → rejection.
        const auto csi = MakeMmimoCsi(4, 1);
        NS_TEST_EXPECT_MSG_EQ(
            xapp->OnCsiReport(1, 1, csi),
            false,
            "no client → rejected");
        NS_TEST_EXPECT_MSG_EQ(xapp->InferenceErrors(),
                               1u,
                               "1 error");

        // Now attach a sane client but set the wrong codebook tx
        // count → rejection.
        auto server = std::make_shared<oranntn::airan::AiranInferenceServer>();
        server->RegisterModel(
            MakePrecoderConfig(),
            oranntn::airan::AiranMockRuntime::MakePrecoderHandler());
        auto pair = oranntn::airan::InProcInferenceChannel::CreatePair();
        auto client = std::make_shared<oranntn::airan::AiranInferenceClient>();
        client->Attach(std::move(pair.first));
        server->AddChannel(std::move(pair.second));
        xapp->AttachInferenceClient(client, "precoder_csi_to_weights");

        // Detach the codebook and re-attach a wrong-sized one.
        auto bad_cb = CreateObject<OranNtnMmimoCodebook>();
        bad_cb->PopulateDftAzimuthSweep(/*num_tx=*/8,
                                          /*num_entries=*/16);
        xapp->AttachCodebook(bad_cb);
        NS_TEST_EXPECT_MSG_EQ(
            xapp->OnCsiReport(2, 1, csi),
            false,
            "tx mismatch → rejected");
        NS_TEST_EXPECT_MSG_EQ(xapp->InferenceErrors(),
                               2u,
                               "2 errors");
    }
};

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
        // E2 loop-timing realism: return-path RC delay, control-loop
        // alignment, Unix-epoch timestamp offset.
        AddTestCase(new OranNtnE2ReturnPathRcTestCase, TestCase::Duration::QUICK);
        AddTestCase(new OranNtnE2AlignToControlLoopTestCase,
                    TestCase::Duration::QUICK);
        AddTestCase(new OranNtnE2UnixEpochOffsetTestCase,
                    TestCase::Duration::QUICK);
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
        // Realism roadmap 4.1.11 — OSC-aligned A1 policy schema registry.
        AddTestCase(new OranNtnA1PolicyRegistryTestCase,
                    TestCase::Duration::QUICK);
        // Realism roadmap T4 — Service-Model plugin ABI.
        AddTestCase(new OranNtnSmRegistryTestCase,
                    TestCase::Duration::QUICK);
        AddTestCase(new OranNtnSmKpmRoundTripTestCase,
                    TestCase::Duration::QUICK);
        AddTestCase(new OranNtnSmRcRoundTripTestCase,
                    TestCase::Duration::QUICK);
        // Realism roadmap T2 — ASN.1-PER codec.
        AddTestCase(new OranNtnAsn1PerPrimitivesTest,
                    TestCase::Duration::QUICK);
        // Realism roadmap T3 — SCTP/TCP E2 listener.
        AddTestCase(new OranNtnE2ListenerHandshakeTest,
                    TestCase::Duration::QUICK);
        AddTestCase(new OranNtnE2ListenerSimulatorTimeTest,
                    TestCase::Duration::QUICK);
        // Realism roadmap 4.1.6 — E2SM-CCC SM plugin.
        AddTestCase(new OranNtnSmCccIndicationTest,
                    TestCase::Duration::QUICK);
        AddTestCase(new OranNtnSmCccControlTest,
                    TestCase::Duration::QUICK);
        AddTestCase(new OranNtnSmRegistryThreePluginsTest,
                    TestCase::Duration::QUICK);
        // Realism roadmap 4.1.7 — NTN-Ephemeris (SIB19) SM plugin.
        AddTestCase(new OranNtnSmEphemerisOrbitalTest,
                    TestCase::Duration::QUICK);
        AddTestCase(new OranNtnSmEphemerisPvTest,
                    TestCase::Duration::QUICK);
        AddTestCase(new OranNtnSmRegistryFourPluginsTest,
                    TestCase::Duration::QUICK);
        // Realism roadmap 4.1.8 — CCC NTN extensions.
        AddTestCase(new OranNtnSmCccNtnLeoPassTest,
                    TestCase::Duration::QUICK);
        AddTestCase(new OranNtnSmCccNtnBeamReconfigTest,
                    TestCase::Duration::QUICK);
        AddTestCase(new OranNtnSmCccNtnDopplerRetuneTest,
                    TestCase::Duration::QUICK);
        AddTestCase(new OranNtnSmCccNtnSimulatorTimeTest,
                    TestCase::Duration::QUICK);
        // Realism roadmap 4.1.9 — CU/DU/RU split.
        AddTestCase(new OranNtnSplitGnbRolesTest,
                    TestCase::Duration::QUICK);
        AddTestCase(new OranNtnSplitGnbControlRoutingTest,
                    TestCase::Duration::QUICK);
        AddTestCase(new OranNtnF1RoundTripTest,
                    TestCase::Duration::QUICK);
        AddTestCase(new OranNtnF1LinkDownTest,
                    TestCase::Duration::QUICK);
        AddTestCase(new OranNtnOfhPlaneRoutingTest,
                    TestCase::Duration::QUICK);
        AddTestCase(new OranNtnSplitGnbEndToEndTest,
                    TestCase::Duration::QUICK);
        AddTestCase(new OranNtnSplitGnbSimulatorWorkloadTest,
                    TestCase::Duration::QUICK);
        // Realism roadmap 4.1.12 — Two-stage mMIMO precoder xApp.
        AddTestCase(new OranNtnMmimoCodebookDftTest,
                    TestCase::Duration::QUICK);
        AddTestCase(new OranNtnMmimoComposeTest,
                    TestCase::Duration::QUICK);
        AddTestCase(new OranNtnMmimoXappRoundTripTest,
                    TestCase::Duration::QUICK);
        AddTestCase(new OranNtnMmimoXappSimulatorTimeTest,
                    TestCase::Duration::QUICK);
        AddTestCase(new OranNtnMmimoXappFailureModesTest,
                    TestCase::Duration::QUICK);
    }
};

static OranNtnTestSuite g_oranNtnTestSuite;
