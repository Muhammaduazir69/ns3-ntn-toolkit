/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-only
 * Author: Muhammad Uzair
 *
 * Gymnasium Environment for Predictive Resource Allocation xApp
 *
 * Regression-based RL environment for LSTM traffic prediction.
 * Observation = sliding window of beam metrics (load, sinr, ueCount).
 * Action = predicted loads for next horizon steps.
 * Reward = negative MSE between prediction and actual load.
 */

#include "oran-ntn-gym-predictive.h"

#include "oran-ntn-sat-bridge.h"
#include "oran-ntn-xapp-predictive-alloc.h"

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

NS_LOG_COMPONENT_DEFINE("OranNtnGymPredictive");
NS_OBJECT_ENSURE_REGISTERED(OranNtnGymPredictive);

TypeId
OranNtnGymPredictive::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::OranNtnGymPredictive")
            .SetParent<OpenGymEnv>()
            .SetGroupName("OranNtn")
            .AddConstructor<OranNtnGymPredictive>()
            .AddAttribute("WindowSize",
                          "Sliding window size for observation history",
                          UintegerValue(30),
                          MakeUintegerAccessor(&OranNtnGymPredictive::m_windowSize),
                          MakeUintegerChecker<uint32_t>(1, 256))
            .AddAttribute("Horizon",
                          "Prediction horizon (number of future steps)",
                          UintegerValue(5),
                          MakeUintegerAccessor(&OranNtnGymPredictive::m_horizon),
                          MakeUintegerChecker<uint32_t>(1, 64))
            .AddAttribute("MaxSteps",
                          "Maximum number of steps per episode",
                          UintegerValue(1000),
                          MakeUintegerAccessor(&OranNtnGymPredictive::m_maxSteps),
                          MakeUintegerChecker<uint32_t>());
    return tid;
}

OranNtnGymPredictive::OranNtnGymPredictive()
    : m_xapp(nullptr),
      m_satBridge(nullptr),
      m_currentBeamId(0),
      m_windowSize(30),
      m_horizon(5),
      m_lastMse(0.0),
      m_stepCount(0),
      m_maxSteps(1000)
{
    NS_LOG_FUNCTION(this);
}

OranNtnGymPredictive::~OranNtnGymPredictive()
{
    NS_LOG_FUNCTION(this);
}

void
OranNtnGymPredictive::DoDispose()
{
    NS_LOG_FUNCTION(this);
    m_xapp = nullptr;
    m_satBridge = nullptr;
    m_loadHistory.clear();
    m_lastPrediction.clear();
    m_lastActual.clear();
    OpenGymEnv::DoDispose();
}

void
OranNtnGymPredictive::SetXapp(Ptr<OranNtnXappPredictiveAlloc> xapp)
{
    NS_LOG_FUNCTION(this << xapp);
    m_xapp = xapp;
}

void
OranNtnGymPredictive::SetSatBridge(Ptr<OranNtnSatBridge> bridge)
{
    NS_LOG_FUNCTION(this << bridge);
    m_satBridge = bridge;
}

void
OranNtnGymPredictive::SetCurrentBeam(uint32_t beamId)
{
    NS_LOG_FUNCTION(this << beamId);
    m_currentBeamId = beamId;
}

void
OranNtnGymPredictive::SetWindowSize(uint32_t w)
{
    NS_LOG_FUNCTION(this << w);
    m_windowSize = w;
}

void
OranNtnGymPredictive::SetHorizon(uint32_t h)
{
    NS_LOG_FUNCTION(this << h);
    m_horizon = h;
}

void
OranNtnGymPredictive::UpdateActualLoad(const std::vector<double>& actualLoads)
{
    NS_LOG_FUNCTION(this);
    m_lastActual = actualLoads;

    // Compute MSE between last prediction and actual
    if (!m_lastPrediction.empty() && !m_lastActual.empty())
    {
        double mse = 0.0;
        uint32_t n = std::min(static_cast<uint32_t>(m_lastPrediction.size()),
                              static_cast<uint32_t>(m_lastActual.size()));
        for (uint32_t i = 0; i < n; i++)
        {
            double diff = m_lastPrediction[i] - m_lastActual[i];
            mse += diff * diff;
        }
        m_lastMse = (n > 0) ? mse / static_cast<double>(n) : 0.0;
    }
    else
    {
        m_lastMse = 0.0;
    }

    NS_LOG_DEBUG("Updated actual loads, MSE=" << m_lastMse);
}

// ---------------------------------------------------------------------------
// OpenGymEnv interface
// ---------------------------------------------------------------------------

Ptr<OpenGymSpace>
OranNtnGymPredictive::GetActionSpace()
{
    NS_LOG_FUNCTION(this);
    // Predicted loads for next horizon steps, range [-5, 5] (normalized)
    std::vector<uint32_t> shape = {m_horizon};
    return CreateObject<OpenGymBoxSpace>(-5.0f, 5.0f, shape, "float");
}

Ptr<OpenGymSpace>
OranNtnGymPredictive::GetObservationSpace()
{
    NS_LOG_FUNCTION(this);
    // windowSize * 3 features: load history, sinr history, ueCount history
    std::vector<uint32_t> shape = {m_windowSize * 3};
    return CreateObject<OpenGymBoxSpace>(-200.0f, 200.0f, shape, "float");
}

Ptr<OpenGymDataContainer>
OranNtnGymPredictive::GetObservation()
{
    NS_LOG_FUNCTION(this);

    std::vector<uint32_t> shape = {m_windowSize * 3};
    auto box = CreateObject<OpenGymBoxContainer<float>>(shape);

    if (m_xapp)
    {
        // Build histories from KPM reports in the base class time window
        auto reports = m_xapp->GetReportsInWindow(Seconds(m_windowSize));

        // Extract per-step load, sinr, and ueCount from reports
        std::vector<double> loadHist;
        std::vector<double> sinrHist;
        std::vector<double> ueCountHist;
        for (const auto& r : reports)
        {
            if (r.beamId == m_currentBeamId)
            {
                loadHist.push_back(r.prbUtilization);
                sinrHist.push_back(r.sinr_dB);
                ueCountHist.push_back(static_cast<double>(r.activeUes));
            }
        }

        // Pad histories to windowSize if shorter
        for (uint32_t i = 0; i < m_windowSize; i++)
        {
            float load = (i < loadHist.size()) ? static_cast<float>(loadHist[i]) : 0.0f;
            box->AddValue(load);
        }
        for (uint32_t i = 0; i < m_windowSize; i++)
        {
            float sinr = (i < sinrHist.size()) ? static_cast<float>(sinrHist[i]) : 0.0f;
            box->AddValue(sinr);
        }
        for (uint32_t i = 0; i < m_windowSize; i++)
        {
            float ueCount = (i < ueCountHist.size()) ? static_cast<float>(ueCountHist[i]) : 0.0f;
            box->AddValue(ueCount);
        }

        // Update internal load history deque
        if (!loadHist.empty())
        {
            m_loadHistory.push_back(loadHist.back());
            while (m_loadHistory.size() > m_windowSize)
            {
                m_loadHistory.pop_front();
            }
        }
    }
    else
    {
        // Fill with zeros when no xApp connected
        for (uint32_t i = 0; i < m_windowSize * 3; i++)
        {
            box->AddValue(0.0f);
        }
    }

    NS_LOG_DEBUG("Predictive obs: beam=" << m_currentBeamId
                                         << " windowSize=" << m_windowSize);
    return box;
}

float
OranNtnGymPredictive::GetReward()
{
    NS_LOG_FUNCTION(this);

    // Reward = -MSE between last prediction and last actual
    float reward = static_cast<float>(-m_lastMse);

    m_stepCount++;

    NS_LOG_DEBUG("Reward=" << reward << " (MSE=" << m_lastMse << ")");
    return reward;
}

bool
OranNtnGymPredictive::GetGameOver()
{
    NS_LOG_FUNCTION(this);
    return (m_stepCount >= m_maxSteps);
}

std::string
OranNtnGymPredictive::GetExtraInfo()
{
    NS_LOG_FUNCTION(this);
    std::ostringstream oss;
    oss << "step=" << m_stepCount
        << ",beamId=" << m_currentBeamId
        << ",mse=" << m_lastMse
        << ",horizon=" << m_horizon
        << ",windowSize=" << m_windowSize;
    return oss.str();
}

bool
OranNtnGymPredictive::ExecuteActions(Ptr<OpenGymDataContainer> action)
{
    NS_LOG_FUNCTION(this << action);

    auto boxAction = DynamicCast<OpenGymBoxContainer<float>>(action);
    if (!boxAction)
    {
        NS_LOG_WARN("Expected box action container for predictive allocation");
        return false;
    }

    std::vector<float> rawPredictions = boxAction->GetData();

    // Store prediction for MSE computation on next step
    m_lastPrediction.clear();
    m_lastPrediction.reserve(m_horizon);

    for (uint32_t h = 0; h < m_horizon && h < rawPredictions.size(); h++)
    {
        m_lastPrediction.push_back(static_cast<double>(rawPredictions[h]));
    }

    NS_LOG_DEBUG("Stored prediction with " << m_lastPrediction.size() << " steps");

    // Forward predictions to xApp for proactive PRB reservation
    if (m_xapp)
    {
        // Issue proactive PRB reservation actions via BuildAction + SubmitAction
        for (uint32_t h = 0; h < m_horizon && h < m_lastPrediction.size(); h++)
        {
            E2RcAction reserveAction;
            reserveAction.timestamp = Simulator::Now().GetSeconds();
            reserveAction.xappId = m_xapp->GetXappId();
            reserveAction.xappName = m_xapp->GetXappName();
            reserveAction.actionType = E2RcActionType::PRB_RESERVATION;
            reserveAction.targetGnbId = 0; // cell-wide
            reserveAction.targetUeId = 0; // no specific UE
            reserveAction.targetBeamId = m_currentBeamId;
            reserveAction.parameter1 = m_lastPrediction[h];
            reserveAction.parameter2 = static_cast<double>(h); // Step offset
            reserveAction.confidence = 1.0;
            reserveAction.executed = false;

            m_xapp->SubmitAction(reserveAction);
        }
    }

    return true;
}

} // namespace ns3
