/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-only
 * Author: Muhammad Uzair
 *
 * Gymnasium Environment for TN-NTN Traffic Steering xApp
 */

#include "oran-ntn-gym-steering.h"

#include "oran-ntn-sat-bridge.h"
#include "oran-ntn-xapp-tn-ntn-steering.h"

#include <ns3/container.h>
#include <ns3/double.h>
#include <ns3/log.h>
#include <ns3/simulator.h>
#include <ns3/spaces.h>
#include <ns3/uinteger.h>

#include <cmath>
#include <sstream>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("OranNtnGymSteering");
NS_OBJECT_ENSURE_REGISTERED(OranNtnGymSteering);

TypeId
OranNtnGymSteering::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::OranNtnGymSteering")
            .SetParent<OpenGymEnv>()
            .SetGroupName("OranNtn")
            .AddConstructor<OranNtnGymSteering>()
            .AddAttribute("MaxSteps",
                          "Maximum number of steps per episode",
                          UintegerValue(1000),
                          MakeUintegerAccessor(&OranNtnGymSteering::m_maxSteps),
                          MakeUintegerChecker<uint32_t>());
    return tid;
}

OranNtnGymSteering::OranNtnGymSteering()
    : m_xapp(nullptr),
      m_satBridge(nullptr),
      m_currentUeId(0),
      m_postLatency(0.0),
      m_postThroughput(0.0),
      m_switchOccurred(false),
      m_stepCount(0),
      m_maxSteps(1000)
{
    NS_LOG_FUNCTION(this);
}

OranNtnGymSteering::~OranNtnGymSteering()
{
    NS_LOG_FUNCTION(this);
}

void
OranNtnGymSteering::DoDispose()
{
    NS_LOG_FUNCTION(this);
    m_xapp = nullptr;
    m_satBridge = nullptr;
    OpenGymEnv::DoDispose();
}

void
OranNtnGymSteering::SetXapp(Ptr<OranNtnXappTnNtnSteering> xapp)
{
    NS_LOG_FUNCTION(this << xapp);
    m_xapp = xapp;
}

void
OranNtnGymSteering::SetSatBridge(Ptr<OranNtnSatBridge> bridge)
{
    NS_LOG_FUNCTION(this << bridge);
    m_satBridge = bridge;
}

void
OranNtnGymSteering::SetCurrentUe(uint32_t ueId)
{
    NS_LOG_FUNCTION(this << ueId);
    m_currentUeId = ueId;
}

void
OranNtnGymSteering::UpdatePostAction(double latency, double throughput, bool switchOccurred)
{
    NS_LOG_FUNCTION(this << latency << throughput << switchOccurred);
    m_postLatency = latency;
    m_postThroughput = throughput;
    m_switchOccurred = switchOccurred;
}

// ---------------------------------------------------------------------------
// OpenGymEnv interface
// ---------------------------------------------------------------------------

Ptr<OpenGymSpace>
OranNtnGymSteering::GetActionSpace()
{
    NS_LOG_FUNCTION(this);
    // Discrete: 0=TN only, 1=NTN only, 2=Dual Connectivity
    return CreateObject<OpenGymDiscreteSpace>(3);
}

Ptr<OpenGymSpace>
OranNtnGymSteering::GetObservationSpace()
{
    NS_LOG_FUNCTION(this);
    // 8 features: tnSinr, ntnSinr, tnLoad, ntnLoad,
    //             tnThroughput, ntnThroughput, qosReq, currentNetwork
    std::vector<uint32_t> shape = {8};
    return CreateObject<OpenGymBoxSpace>(-200.0f, 200.0f, shape, "float");
}

Ptr<OpenGymDataContainer>
OranNtnGymSteering::GetObservation()
{
    NS_LOG_FUNCTION(this);

    std::vector<uint32_t> shape = {8};
    auto box = CreateObject<OpenGymBoxContainer<float>>(shape);

    float tnSinr = 0.0f;
    float ntnSinr = 0.0f;
    float tnLoad = 0.0f;
    float ntnLoad = 0.0f;
    float tnThroughput = 0.0f;
    float ntnThroughput = 0.0f;
    float qosReq = 0.0f;
    float currentNetwork = 0.0f;
    // AI-03: measured one-way delay of the serving path, for the reward below.
    double lastReportLatencyMs = 0.0;

    if (m_xapp)
    {
        // Use base class GetUeReportsInWindow to get latest UE report
        auto ueReports = m_xapp->GetUeReportsInWindow(m_currentUeId, Seconds(5));
        E2KpmReport report;
        if (!ueReports.empty())
        {
            report = ueReports.back();
        }
        // E2KpmReport has dual connectivity fields
        tnSinr = static_cast<float>(report.tnSinr_dB);
        ntnSinr = static_cast<float>(report.ntnSinr_dB);
        tnLoad = static_cast<float>(report.prbUtilization); // Approx TN load
        ntnLoad = static_cast<float>(report.prbUtilization); // Approx NTN load
        tnThroughput = static_cast<float>(report.tnThroughput_Mbps);
        ntnThroughput = static_cast<float>(report.ntnThroughput_Mbps);
        qosReq = static_cast<float>(report.sliceId); // Slice as QoS proxy
        lastReportLatencyMs = report.propagationDelay_ms;
        // Encode current network from steering xApp: 0=TN, 1=NTN, 2=DC
        NetworkSelection sel = m_xapp->GetUeNetworkSelection(m_currentUeId);
        currentNetwork = static_cast<float>(static_cast<uint8_t>(sel));
    }

    box->AddValue(tnSinr);
    box->AddValue(ntnSinr);
    box->AddValue(tnLoad);
    box->AddValue(ntnLoad);
    box->AddValue(tnThroughput);
    box->AddValue(ntnThroughput);
    box->AddValue(qosReq);
    box->AddValue(currentNetwork);

    // AI-03. Latch the post-action state from the same measured report the
    // observation was built from. GetReward() is built from m_postLatency,
    // m_postThroughput and m_switchOccurred, and the first two were written ONLY
    // by UpdatePostAction(), which has no caller anywhere in the tree. They held
    // their constructed 0.0 forever, which made the reward
    //     -0/100 + min(0/100, 1) - switchingCost
    // i.e. zero unless the agent switched, and strictly negative when it did.
    // The only policy that could maximise it was "never act".
    //
    // The delivered throughput and the one-way delay of whichever network the UE
    // is actually on are the honest post-action quantities: they are what the
    // steering decision changed.
    // On a dual-connected UE both legs carry traffic, so the delivered rate is
    // the sum; otherwise it is the leg the xApp actually selected.
    if (m_xapp)
    {
        const NetworkSelection sel = m_xapp->GetUeNetworkSelection(m_currentUeId);
        switch (sel)
        {
        case NetworkSelection::TERRESTRIAL:
            m_postThroughput = tnThroughput;
            break;
        case NetworkSelection::SATELLITE:
            m_postThroughput = ntnThroughput;
            break;
        case NetworkSelection::DUAL_CONNECTED:
            m_postThroughput = tnThroughput + ntnThroughput;
            break;
        }
    }
    m_postLatency = lastReportLatencyMs;

    NS_LOG_DEBUG("Steering obs: tnSinr=" << tnSinr << " ntnSinr=" << ntnSinr
                                         << " currentNet=" << currentNetwork);
    return box;
}

float
OranNtnGymSteering::GetReward()
{
    NS_LOG_FUNCTION(this);

    // Latency penalty: higher latency = more penalty
    double latencyPenalty = m_postLatency / 100.0; // Normalize by 100ms

    // Throughput bonus: normalized to [0,1] (assume max 100 Mbps)
    double throughputBonus = std::min(m_postThroughput / 100.0, 1.0);

    // Switching cost
    double switchingCost = m_switchOccurred ? 0.3 : 0.0;

    float reward = static_cast<float>(
        -latencyPenalty + throughputBonus - switchingCost);

    m_stepCount++;

    NS_LOG_DEBUG("Reward=" << reward << " (latPen=" << latencyPenalty
                           << " tputBonus=" << throughputBonus
                           << " switchCost=" << switchingCost << ")");
    return reward;
}

bool
OranNtnGymSteering::GetGameOver()
{
    NS_LOG_FUNCTION(this);
    return (m_stepCount >= m_maxSteps);
}

std::string
OranNtnGymSteering::GetExtraInfo()
{
    NS_LOG_FUNCTION(this);
    std::ostringstream oss;
    oss << "step=" << m_stepCount
        << ",ueId=" << m_currentUeId
        << ",latency=" << m_postLatency
        << ",throughput=" << m_postThroughput
        << ",switched=" << m_switchOccurred;
    return oss.str();
}

bool
OranNtnGymSteering::ExecuteActions(Ptr<OpenGymDataContainer> action)
{
    NS_LOG_FUNCTION(this << action);

    auto discrete = DynamicCast<OpenGymDiscreteContainer>(action);
    if (!discrete)
    {
        NS_LOG_WARN("Expected discrete action container for steering");
        return false;
    }

    uint32_t chosenAction = discrete->GetValue();
    NS_LOG_DEBUG("Steering action=" << chosenAction << " for UE " << m_currentUeId);

    if (!m_xapp)
    {
        return false;
    }

    E2RcAction steerAction;
    steerAction.timestamp = Simulator::Now().GetSeconds();
    steerAction.xappId = m_xapp->GetXappId();
    steerAction.xappName = m_xapp->GetXappName();
    steerAction.targetUeId = m_currentUeId;
    steerAction.targetGnbId = 0;
    steerAction.confidence = 1.0;
    steerAction.executed = false;

    switch (chosenAction)
    {
    case 0: // TN only
        steerAction.actionType = E2RcActionType::DC_TEARDOWN;
        steerAction.parameter1 = 1.0; // Full TN
        NS_LOG_INFO("Steering UE " << m_currentUeId << " to TN only");
        break;
    case 1: // NTN only
        steerAction.actionType = E2RcActionType::DC_TEARDOWN;
        steerAction.parameter1 = 0.0; // Full NTN
        NS_LOG_INFO("Steering UE " << m_currentUeId << " to NTN only");
        break;
    case 2: // Dual Connectivity
        steerAction.actionType = E2RcActionType::DC_SETUP;
        steerAction.parameter1 = 0.5; // 50/50 split
        NS_LOG_INFO("Steering UE " << m_currentUeId << " to DC mode");
        break;
    default:
        NS_LOG_WARN("Invalid steering action: " << chosenAction);
        return false;
    }

    m_xapp->SubmitAction(steerAction);
    return true;
}

} // namespace ns3
