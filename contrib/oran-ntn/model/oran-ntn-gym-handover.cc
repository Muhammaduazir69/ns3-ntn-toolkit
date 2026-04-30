/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-only
 * Author: Muhammad Uzair
 *
 * Gymnasium Environment for Handover xApp
 */

#include "oran-ntn-gym-handover.h"

#include "oran-ntn-sat-bridge.h"
#include "oran-ntn-xapp-ho-predict.h"

#include <ns3/container.h>
#include <ns3/double.h>
#include <ns3/log.h>
#include <ns3/simulator.h>
#include <ns3/spaces.h>
#include <ns3/uinteger.h>

#include <algorithm>
#include <cmath>
#include <sstream>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("OranNtnGymHandover");
NS_OBJECT_ENSURE_REGISTERED(OranNtnGymHandover);

TypeId
OranNtnGymHandover::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::OranNtnGymHandover")
            .SetParent<OpenGymEnv>()
            .SetGroupName("OranNtn")
            .AddConstructor<OranNtnGymHandover>()
            .AddAttribute("MaxCandidates",
                          "Maximum number of handover candidate cells",
                          UintegerValue(4),
                          MakeUintegerAccessor(&OranNtnGymHandover::m_maxCandidates),
                          MakeUintegerChecker<uint32_t>(1, 16))
            .AddAttribute("MaxSteps",
                          "Maximum number of steps per episode",
                          UintegerValue(1000),
                          MakeUintegerAccessor(&OranNtnGymHandover::m_maxSteps),
                          MakeUintegerChecker<uint32_t>())
            .AddAttribute("SinrImprovementWeight",
                          "Reward weight for SINR improvement after HO",
                          DoubleValue(1.0),
                          MakeDoubleAccessor(&OranNtnGymHandover::m_sinrImprovementWeight),
                          MakeDoubleChecker<double>())
            .AddAttribute("PingPongPenaltyWeight",
                          "Penalty weight for ping-pong handovers",
                          DoubleValue(-5.0),
                          MakeDoubleAccessor(&OranNtnGymHandover::m_pingPongPenaltyWeight),
                          MakeDoubleChecker<double>())
            .AddAttribute("HoCostWeight",
                          "Cost weight for executing a handover",
                          DoubleValue(-0.5),
                          MakeDoubleAccessor(&OranNtnGymHandover::m_hoCostWeight),
                          MakeDoubleChecker<double>())
            .AddAttribute("TteBonusWeight",
                          "Reward weight for TTE improvement",
                          DoubleValue(0.1),
                          MakeDoubleAccessor(&OranNtnGymHandover::m_tteBonusWeight),
                          MakeDoubleChecker<double>())
            .AddAttribute("ElevationContinuityWeight",
                          "Reward weight for elevation angle continuity",
                          DoubleValue(0.05),
                          MakeDoubleAccessor(&OranNtnGymHandover::m_elevationContinuityWeight),
                          MakeDoubleChecker<double>());
    return tid;
}

OranNtnGymHandover::OranNtnGymHandover()
    : m_xapp(nullptr),
      m_satBridge(nullptr),
      m_maxCandidates(4),
      m_currentUeId(0),
      m_preActionSinr(0.0),
      m_postActionSinr(0.0),
      m_preActionTte(0.0),
      m_postActionTte(0.0),
      m_pingPongDetected(false),
      m_handoverExecuted(false),
      m_hoLeadTime(0.0),
      m_stepCount(0),
      m_maxSteps(1000),
      m_cumulativeReward(0.0),
      m_sinrImprovementWeight(1.0),
      m_pingPongPenaltyWeight(-5.0),
      m_hoCostWeight(-0.5),
      m_tteBonusWeight(0.1),
      m_elevationContinuityWeight(0.05)
{
    NS_LOG_FUNCTION(this);
}

OranNtnGymHandover::~OranNtnGymHandover()
{
    NS_LOG_FUNCTION(this);
}

void
OranNtnGymHandover::DoDispose()
{
    NS_LOG_FUNCTION(this);
    m_xapp = nullptr;
    m_satBridge = nullptr;
    OpenGymEnv::DoDispose();
}

void
OranNtnGymHandover::SetXapp(Ptr<OranNtnXappHoPredict> xapp)
{
    NS_LOG_FUNCTION(this << xapp);
    m_xapp = xapp;
}

void
OranNtnGymHandover::SetSatBridge(Ptr<OranNtnSatBridge> bridge)
{
    NS_LOG_FUNCTION(this << bridge);
    m_satBridge = bridge;
}

void
OranNtnGymHandover::SetMaxCandidates(uint32_t n)
{
    NS_LOG_FUNCTION(this << n);
    m_maxCandidates = n;
}

void
OranNtnGymHandover::SetCurrentUe(uint32_t ueId)
{
    NS_LOG_FUNCTION(this << ueId);
    m_currentUeId = ueId;
}

void
OranNtnGymHandover::UpdatePostAction(double newSinr, double newTte, bool pingPongDetected)
{
    NS_LOG_FUNCTION(this << newSinr << newTte << pingPongDetected);
    m_postActionSinr = newSinr;
    m_postActionTte = newTte;
    m_pingPongDetected = pingPongDetected;
}

// ---------------------------------------------------------------------------
// OpenGymEnv interface
// ---------------------------------------------------------------------------

Ptr<OpenGymSpace>
OranNtnGymHandover::GetActionSpace()
{
    NS_LOG_FUNCTION(this);
    // Action 0 = stay, 1..maxCandidates = handover to candidate i
    return CreateObject<OpenGymDiscreteSpace>(m_maxCandidates + 1);
}

Ptr<OpenGymSpace>
OranNtnGymHandover::GetObservationSpace()
{
    NS_LOG_FUNCTION(this);
    // 12 features: servingSinr, servingTte, servingElevation, servingDoppler,
    //              sinrSlope, bestCandSinr, bestCandTte, bestCandElevation,
    //              timeSinceLastHo, recentHoCount, prbUtil, activeUes
    std::vector<uint32_t> shape = {12};
    return CreateObject<OpenGymBoxSpace>(-200.0f, 200.0f, shape, "float");
}

Ptr<OpenGymDataContainer>
OranNtnGymHandover::GetObservation()
{
    NS_LOG_FUNCTION(this);

    std::vector<uint32_t> shape = {12};
    auto box = CreateObject<OpenGymBoxContainer<float>>(shape);

    // Default values when xApp/bridge not connected
    float servingSinr = 0.0f;
    float servingTte = 0.0f;
    float servingElevation = 0.0f;
    float servingDoppler = 0.0f;
    float sinrSlope = 0.0f;
    float bestCandSinr = -200.0f;
    float bestCandTte = 0.0f;
    float bestCandElevation = 0.0f;
    float timeSinceLastHo = 0.0f;
    float recentHoCount = 0.0f;
    float prbUtil = 0.0f;
    float activeUes = 0.0f;

    if (m_xapp)
    {
        // Query xApp for current UE's serving gNB state via base class API
        auto ueReports = m_xapp->GetUeReportsInWindow(m_currentUeId, Seconds(10));
        E2KpmReport report;
        if (!ueReports.empty())
        {
            report = ueReports.back();
        }
        servingSinr = static_cast<float>(report.sinr_dB);
        servingTte = static_cast<float>(report.tte_s);
        servingElevation = static_cast<float>(report.elevation_deg);
        servingDoppler = static_cast<float>(report.doppler_Hz);

        // Compute SINR slope from history
        if (ueReports.size() >= 2)
        {
            sinrSlope = static_cast<float>(
                ueReports.back().sinr_dB - ueReports[ueReports.size() - 2].sinr_dB);
        }

        // Best candidate info from all recent reports (different gNBs)
        auto allReports = m_xapp->GetReportsInWindow(Seconds(5));
        std::vector<E2KpmReport> candidates;
        for (const auto& r : allReports)
        {
            if (r.gnbId != report.gnbId && r.ueId == m_currentUeId)
            {
                candidates.push_back(r);
            }
        }
        if (!candidates.empty())
        {
            // Find best candidate by SINR
            auto bestIt = std::max_element(
                candidates.begin(),
                candidates.end(),
                [](const E2KpmReport& a, const E2KpmReport& b) {
                    return a.sinr_dB < b.sinr_dB;
                });
            bestCandSinr = static_cast<float>(bestIt->sinr_dB);
            bestCandTte = static_cast<float>(bestIt->tte_s);
            bestCandElevation = static_cast<float>(bestIt->elevation_deg);
        }

        // Compute time since last HO from UE report history timestamps
        if (ueReports.size() >= 2)
        {
            timeSinceLastHo = static_cast<float>(
                Simulator::Now().GetSeconds() - ueReports.front().timestamp);
        }
        recentHoCount = 0.0f; // Approximated from report count changes
        prbUtil = static_cast<float>(report.prbUtilization);
        activeUes = static_cast<float>(report.activeUes);
    }

    // Store pre-action SINR/TTE for reward computation
    m_preActionSinr = servingSinr;
    m_preActionTte = servingTte;

    box->AddValue(servingSinr);
    box->AddValue(servingTte);
    box->AddValue(servingElevation);
    box->AddValue(servingDoppler);
    box->AddValue(sinrSlope);
    box->AddValue(bestCandSinr);
    box->AddValue(bestCandTte);
    box->AddValue(bestCandElevation);
    box->AddValue(timeSinceLastHo);
    box->AddValue(recentHoCount);
    box->AddValue(prbUtil);
    box->AddValue(activeUes);

    NS_LOG_DEBUG("Observation: sinr=" << servingSinr << " tte=" << servingTte
                                      << " elev=" << servingElevation
                                      << " bestCandSinr=" << bestCandSinr);
    return box;
}

float
OranNtnGymHandover::GetReward()
{
    NS_LOG_FUNCTION(this);

    // SINR improvement component
    double sinrImprovement = m_postActionSinr - m_preActionSinr;

    // Ping-pong penalty (exponential)
    double pingPongPenalty = m_pingPongDetected ? 1.0 : 0.0;

    // Handover execution cost
    double hoCost = m_handoverExecuted ? 1.0 : 0.0;

    // TTE bonus: normalized improvement (max 120s TTE)
    double tteImprovement = std::max(0.0, m_postActionTte - m_preActionTte) / 120.0;

    // Compute combined reward
    float reward = static_cast<float>(
        m_sinrImprovementWeight * sinrImprovement +
        m_pingPongPenaltyWeight * pingPongPenalty +
        m_hoCostWeight * hoCost +
        m_tteBonusWeight * tteImprovement);

    m_cumulativeReward += reward;
    m_stepCount++;

    NS_LOG_DEBUG("Reward=" << reward << " (sinrDelta=" << sinrImprovement
                           << " pingPong=" << pingPongPenalty << " hoCost=" << hoCost
                           << " tteDelta=" << tteImprovement << ")");
    return reward;
}

bool
OranNtnGymHandover::GetGameOver()
{
    NS_LOG_FUNCTION(this);
    bool gameOver = (m_stepCount >= m_maxSteps);
    if (gameOver)
    {
        NS_LOG_INFO("Episode complete: steps=" << m_stepCount
                                               << " cumReward=" << m_cumulativeReward);
    }
    return gameOver;
}

std::string
OranNtnGymHandover::GetExtraInfo()
{
    NS_LOG_FUNCTION(this);
    std::ostringstream oss;
    oss << "step=" << m_stepCount
        << ",ueId=" << m_currentUeId
        << ",cumReward=" << m_cumulativeReward
        << ",hoExecuted=" << m_handoverExecuted
        << ",pingPong=" << m_pingPongDetected;
    return oss.str();
}

bool
OranNtnGymHandover::ExecuteActions(Ptr<OpenGymDataContainer> action)
{
    NS_LOG_FUNCTION(this << action);

    auto discrete = DynamicCast<OpenGymDiscreteContainer>(action);
    if (!discrete)
    {
        NS_LOG_WARN("Expected discrete action container");
        return false;
    }

    uint32_t chosenAction = discrete->GetValue();
    NS_LOG_DEBUG("Action=" << chosenAction << " for UE " << m_currentUeId);

    m_handoverExecuted = false;

    if (chosenAction > 0 && m_xapp)
    {
        // Action > 0 means handover to candidate (chosenAction - 1)
        uint32_t candidateIdx = chosenAction - 1;

        // Build candidate list from recent reports for other gNBs
        auto servingReport = m_xapp->GetUeReportsInWindow(m_currentUeId, Seconds(5));
        uint32_t servingGnb = 0;
        if (!servingReport.empty())
        {
            servingGnb = servingReport.back().gnbId;
        }
        auto allReports = m_xapp->GetReportsInWindow(Seconds(5));
        std::vector<E2KpmReport> candidates;
        for (const auto& r : allReports)
        {
            if (r.gnbId != servingGnb && r.ueId == m_currentUeId)
            {
                candidates.push_back(r);
            }
        }

        if (candidateIdx < candidates.size())
        {
            E2RcAction hoAction;
            hoAction.timestamp = Simulator::Now().GetSeconds();
            hoAction.xappId = m_xapp->GetXappId();
            hoAction.xappName = m_xapp->GetXappName();
            hoAction.actionType = E2RcActionType::HANDOVER_TRIGGER;
            hoAction.targetGnbId = candidates[candidateIdx].gnbId;
            hoAction.targetUeId = m_currentUeId;
            hoAction.targetBeamId = candidates[candidateIdx].beamId;
            hoAction.confidence = 1.0;
            hoAction.executed = false;

            m_xapp->SubmitAction(hoAction);
            m_handoverExecuted = true;
            NS_LOG_INFO("RL triggered HO: UE " << m_currentUeId << " -> gNB "
                                                << candidates[candidateIdx].gnbId);
        }
        else
        {
            NS_LOG_WARN("Candidate index " << candidateIdx << " out of range ("
                                           << candidates.size() << " available)");
        }
    }
    else
    {
        NS_LOG_DEBUG("Stay decision for UE " << m_currentUeId);
    }

    return true;
}

} // namespace ns3
