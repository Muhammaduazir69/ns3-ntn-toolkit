/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-only
 * Author: Muhammad Uzair
 *
 * Gymnasium Environment for Slice Manager xApp
 */

#include "oran-ntn-gym-slice.h"

#include "oran-ntn-sat-bridge.h"
#include "oran-ntn-xapp-slice-manager.h"

#include <ns3/container.h>
#include <ns3/double.h>
#include <ns3/log.h>
#include <ns3/simulator.h>
#include <ns3/spaces.h>
#include <ns3/uinteger.h>

#include <algorithm>
#include <cmath>
#include <numeric>
#include <sstream>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("OranNtnGymSlice");
NS_OBJECT_ENSURE_REGISTERED(OranNtnGymSlice);

TypeId
OranNtnGymSlice::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::OranNtnGymSlice")
            .SetParent<OpenGymEnv>()
            .SetGroupName("OranNtn")
            .AddConstructor<OranNtnGymSlice>()
            .AddAttribute("NumSlices",
                          "Number of network slices",
                          UintegerValue(3),
                          MakeUintegerAccessor(&OranNtnGymSlice::m_numSlices),
                          MakeUintegerChecker<uint32_t>(1, 32))
            .AddAttribute("MaxSteps",
                          "Maximum number of steps per episode",
                          UintegerValue(1000),
                          MakeUintegerAccessor(&OranNtnGymSlice::m_maxSteps),
                          MakeUintegerChecker<uint32_t>())
            .AddAttribute("SlaWeight",
                          "Reward weight for SLA compliance",
                          DoubleValue(2.0),
                          MakeDoubleAccessor(&OranNtnGymSlice::m_slaWeight),
                          MakeDoubleChecker<double>())
            .AddAttribute("ViolationPenalty",
                          "Penalty per SLA violation",
                          DoubleValue(5.0),
                          MakeDoubleAccessor(&OranNtnGymSlice::m_violationPenalty),
                          MakeDoubleChecker<double>())
            .AddAttribute("EfficiencyWeight",
                          "Reward weight for resource efficiency",
                          DoubleValue(0.5),
                          MakeDoubleAccessor(&OranNtnGymSlice::m_efficiencyWeight),
                          MakeDoubleChecker<double>());
    return tid;
}

OranNtnGymSlice::OranNtnGymSlice()
    : m_xapp(nullptr),
      m_satBridge(nullptr),
      m_numSlices(3),
      m_currentGnbId(0),
      m_postSlaCompliance(0.0),
      m_postViolations(0),
      m_postEfficiency(0.0),
      m_stepCount(0),
      m_maxSteps(1000),
      m_slaWeight(2.0),
      m_violationPenalty(5.0),
      m_efficiencyWeight(0.5)
{
    NS_LOG_FUNCTION(this);
}

OranNtnGymSlice::~OranNtnGymSlice()
{
    NS_LOG_FUNCTION(this);
}

void
OranNtnGymSlice::DoDispose()
{
    NS_LOG_FUNCTION(this);
    m_xapp = nullptr;
    m_satBridge = nullptr;
    OpenGymEnv::DoDispose();
}

void
OranNtnGymSlice::SetXapp(Ptr<OranNtnXappSliceManager> xapp)
{
    NS_LOG_FUNCTION(this << xapp);
    m_xapp = xapp;
}

void
OranNtnGymSlice::SetSatBridge(Ptr<OranNtnSatBridge> bridge)
{
    NS_LOG_FUNCTION(this << bridge);
    m_satBridge = bridge;
}

void
OranNtnGymSlice::SetNumSlices(uint32_t n)
{
    NS_LOG_FUNCTION(this << n);
    m_numSlices = n;
}

void
OranNtnGymSlice::SetCurrentGnb(uint32_t gnbId)
{
    NS_LOG_FUNCTION(this << gnbId);
    m_currentGnbId = gnbId;
}

void
OranNtnGymSlice::UpdatePostAction(double slaCompliance, uint32_t violations, double efficiency)
{
    NS_LOG_FUNCTION(this << slaCompliance << violations << efficiency);
    m_postSlaCompliance = slaCompliance;
    m_postViolations = violations;
    m_postEfficiency = efficiency;
}

// ---------------------------------------------------------------------------
// OpenGymEnv interface
// ---------------------------------------------------------------------------

Ptr<OpenGymSpace>
OranNtnGymSlice::GetActionSpace()
{
    NS_LOG_FUNCTION(this);
    // Continuous PRB share per slice in [0,1], will be softmax-normalized
    std::vector<uint32_t> shape = {m_numSlices};
    return CreateObject<OpenGymBoxSpace>(0.0f, 1.0f, shape, "float");
}

Ptr<OpenGymSpace>
OranNtnGymSlice::GetObservationSpace()
{
    NS_LOG_FUNCTION(this);
    // Per-slice: [prbShare, throughput, latency, reliability, demand] => numSlices * 5
    std::vector<uint32_t> shape = {m_numSlices * 5};
    return CreateObject<OpenGymBoxSpace>(-200.0f, 200.0f, shape, "float");
}

Ptr<OpenGymDataContainer>
OranNtnGymSlice::GetObservation()
{
    NS_LOG_FUNCTION(this);

    std::vector<uint32_t> shape = {m_numSlices * 5};
    auto box = CreateObject<OpenGymBoxContainer<float>>(shape);

    for (uint32_t s = 0; s < m_numSlices; s++)
    {
        float prbShare = 0.0f;
        float throughput = 0.0f;
        float latency = 0.0f;
        float reliability = 0.0f;
        float demand = 0.0f;

        if (m_xapp)
        {
            // Use base class GetLatestReport for the gNB, filter by slice
            auto report = m_xapp->GetLatestReport(m_currentGnbId);
            if (report.sliceId == static_cast<uint8_t>(s))
            {
                throughput = static_cast<float>(report.sliceThroughput_Mbps);
                latency = static_cast<float>(report.sliceLatency_ms);
                reliability = static_cast<float>(report.sliceReliability);
            }
            // Get current PRB allocation from slice manager
            auto allocations = m_xapp->GetSliceAllocations(m_currentGnbId);
            auto allocIt = allocations.find(static_cast<uint8_t>(s));
            if (allocIt != allocations.end())
            {
                prbShare = static_cast<float>(allocIt->second);
            }
            // Demand estimated from PRB utilization
            demand = static_cast<float>(report.prbUtilization);
        }

        box->AddValue(prbShare);
        box->AddValue(throughput);
        box->AddValue(latency);
        box->AddValue(reliability);
        box->AddValue(demand);
    }

    NS_LOG_DEBUG("Slice observation: " << m_numSlices << " slices for gNB " << m_currentGnbId);
    return box;
}

float
OranNtnGymSlice::GetReward()
{
    NS_LOG_FUNCTION(this);

    float reward = static_cast<float>(
        m_slaWeight * m_postSlaCompliance -
        m_violationPenalty * static_cast<double>(m_postViolations) +
        m_efficiencyWeight * m_postEfficiency);

    m_stepCount++;

    NS_LOG_DEBUG("Reward=" << reward << " (sla=" << m_postSlaCompliance
                           << " violations=" << m_postViolations
                           << " efficiency=" << m_postEfficiency << ")");
    return reward;
}

bool
OranNtnGymSlice::GetGameOver()
{
    NS_LOG_FUNCTION(this);
    return (m_stepCount >= m_maxSteps);
}

std::string
OranNtnGymSlice::GetExtraInfo()
{
    NS_LOG_FUNCTION(this);
    std::ostringstream oss;
    oss << "step=" << m_stepCount
        << ",gnbId=" << m_currentGnbId
        << ",numSlices=" << m_numSlices
        << ",sla=" << m_postSlaCompliance
        << ",violations=" << m_postViolations;
    return oss.str();
}

bool
OranNtnGymSlice::ExecuteActions(Ptr<OpenGymDataContainer> action)
{
    NS_LOG_FUNCTION(this << action);

    auto boxAction = DynamicCast<OpenGymBoxContainer<float>>(action);
    if (!boxAction)
    {
        NS_LOG_WARN("Expected box action container for slice allocation");
        return false;
    }

    std::vector<float> rawShares = boxAction->GetData();

    // Softmax normalization: exp(x_i) / sum(exp(x_j)) to ensure shares sum to 1
    float maxVal = *std::max_element(rawShares.begin(),
                                     rawShares.end());
    std::vector<float> expShares(m_numSlices, 0.0f);
    float expSum = 0.0f;

    for (uint32_t s = 0; s < m_numSlices && s < rawShares.size(); s++)
    {
        expShares[s] = std::exp(rawShares[s] - maxVal); // Subtract max for numerical stability
        expSum += expShares[s];
    }

    if (expSum < 1e-6f)
    {
        expSum = 1.0f; // Fallback to uniform
    }

    std::vector<double> normalizedShares(m_numSlices, 0.0);
    for (uint32_t s = 0; s < m_numSlices; s++)
    {
        normalizedShares[s] = static_cast<double>(expShares[s] / expSum);
        NS_LOG_DEBUG("Slice " << s << " PRB share=" << normalizedShares[s]);
    }

    // Apply to xApp via BuildAction + SubmitAction
    if (m_xapp)
    {
        for (uint32_t s = 0; s < m_numSlices; s++)
        {
            E2RcAction sliceAction;
            sliceAction.timestamp = Simulator::Now().GetSeconds();
            sliceAction.xappId = m_xapp->GetXappId();
            sliceAction.xappName = m_xapp->GetXappName();
            sliceAction.actionType = E2RcActionType::SLICE_PRB_ALLOCATION;
            sliceAction.targetGnbId = m_currentGnbId;
            sliceAction.targetUeId = 0; // cell-wide
            sliceAction.targetSliceId = static_cast<uint8_t>(s);
            sliceAction.parameter1 = normalizedShares[s];
            sliceAction.confidence = 1.0;
            sliceAction.executed = false;

            m_xapp->SubmitAction(sliceAction);
        }
    }

    return true;
}

} // namespace ns3
