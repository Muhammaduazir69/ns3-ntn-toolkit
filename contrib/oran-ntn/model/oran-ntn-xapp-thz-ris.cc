/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-only
 * Author: Muhammad Uzair
 *
 * THz RIS Management xApp - Implementation
 *
 * Monitors RIS-assisted gain and THz SNR to trigger phase profile
 * reconfiguration when gain drops below threshold. Coordinates
 * multiple RIS panels for optimal THz link enhancement.
 */

#include "oran-ntn-xapp-thz-ris.h"

#include <ns3/double.h>
#include <ns3/log.h>
#include <ns3/simulator.h>
#include <ns3/uinteger.h>

#include <algorithm>
#include <cmath>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("OranNtnXappThzRis");
NS_OBJECT_ENSURE_REGISTERED(OranNtnXappThzRis);

TypeId
OranNtnXappThzRis::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::OranNtnXappThzRis")
            .SetParent<OranNtnXappBase>()
            .SetGroupName("OranNtn")
            .AddConstructor<OranNtnXappThzRis>()
            .AddAttribute("RisGainThreshold",
                          "Minimum acceptable RIS gain (dB) before triggering reconfig",
                          DoubleValue(5.0),
                          MakeDoubleAccessor(&OranNtnXappThzRis::m_risGainThreshold_dB),
                          MakeDoubleChecker<double>())
            .AddAttribute("SnrImprovementTarget",
                          "Target SNR improvement from RIS reconfiguration (dB)",
                          DoubleValue(3.0),
                          MakeDoubleAccessor(&OranNtnXappThzRis::m_snrImprovementTarget_dB),
                          MakeDoubleChecker<double>(0.0))
            .AddAttribute("NumRisPanels",
                          "Number of RIS panels per satellite",
                          UintegerValue(4),
                          MakeUintegerAccessor(&OranNtnXappThzRis::m_numRisPanels),
                          MakeUintegerChecker<uint32_t>(1));
    return tid;
}

OranNtnXappThzRis::OranNtnXappThzRis()
    : m_risGainThreshold_dB(5.0),
      m_snrImprovementTarget_dB(3.0),
      m_numRisPanels(4)
{
    NS_LOG_FUNCTION(this);
    SetXappName("ThzRis");

    m_metrics = {};
}

OranNtnXappThzRis::~OranNtnXappThzRis()
{
    NS_LOG_FUNCTION(this);
}

// --------------------------------------------------------------------------
//  E2 subscription
// --------------------------------------------------------------------------

E2Subscription
OranNtnXappThzRis::GetRequiredSubscription() const
{
    E2Subscription sub;
    sub.subscriptionId = 0;
    sub.ricRequestorId = GetXappId();
    sub.ranFunctionId = 2; // KPM service model
    sub.reportingPeriod = MilliSeconds(200);
    sub.eventTrigger = false;
    sub.eventThreshold = 0.0;
    sub.batchOnVisibility = false;
    sub.maxBufferAge = Seconds(5);
    sub.useIslRelay = true;
    return sub;
}

// --------------------------------------------------------------------------
//  ProcessKpmReport
// --------------------------------------------------------------------------

void
OranNtnXappThzRis::ProcessKpmReport(const E2KpmReport& report)
{
    NS_LOG_FUNCTION(this << report.gnbId);

    uint32_t gnbId = report.gnbId;

    ThzRisState& state = m_risStates[gnbId];
    state.previousGain_dB = state.risGain_dB; // track previous for delta
    state.risGain_dB = report.risGain_dB;
    state.thzSnr_dB = report.thzSnr_dB;
    state.elevation_deg = report.elevation_deg;
    state.thzCenterFreq_GHz = report.thzCenterFreq_GHz;

    NS_LOG_DEBUG("ThzRis: gnb=" << gnbId
                 << " risGain=" << state.risGain_dB << "dB"
                 << " thzSnr=" << state.thzSnr_dB << "dB"
                 << " elev=" << state.elevation_deg << "deg");
}

// --------------------------------------------------------------------------
//  DecisionCycle
// --------------------------------------------------------------------------

void
OranNtnXappThzRis::DecisionCycle()
{
    NS_LOG_FUNCTION(this);

    m_pendingActions.clear();

    double sumGain = 0.0;
    double sumSnr = 0.0;
    uint32_t count = 0;

    for (auto& kv : m_risStates)
    {
        uint32_t gnbId = kv.first;
        ThzRisState& state = kv.second;

        sumGain += state.risGain_dB;
        sumSnr += state.thzSnr_dB;
        count++;

        // Check if RIS gain has dropped below threshold
        if (state.risGain_dB < m_risGainThreshold_dB)
        {
            double achievableGain = EstimateAchievableGain(
                state.risGain_dB, state.thzSnr_dB, state.elevation_deg);

            NS_LOG_INFO("ThzRis: gnb" << gnbId
                        << " RIS gain=" << state.risGain_dB
                        << "dB below threshold=" << m_risGainThreshold_dB
                        << "dB, achievable=" << achievableGain << "dB");

            // Only reconfigure if meaningful improvement is possible
            if (achievableGain > state.risGain_dB + 1.0)
            {
                uint32_t optimalProfile = ComputeOptimalPhaseProfile(gnbId);

                E2RcAction action = BuildAction(
                    E2RcActionType::ACTION_THZ_RIS_CONFIG,
                    gnbId, 0, 0.85);
                action.parameter1 = static_cast<double>(optimalProfile);
                action.parameter2 = achievableGain;

                m_pendingActions.push_back(action);
                m_metrics.risReconfigurations++;

                double improvement = achievableGain - state.risGain_dB;
                m_metrics.maxGainImprovement_dB =
                    std::max(m_metrics.maxGainImprovement_dB, improvement);

                state.currentPhaseProfile = optimalProfile;
            }
        }

        // Also reconfigure if gain is dropping rapidly (negative trend)
        double gainDelta = state.risGain_dB - state.previousGain_dB;
        if (gainDelta < -2.0 && state.risGain_dB > m_risGainThreshold_dB)
        {
            NS_LOG_INFO("ThzRis: gnb" << gnbId
                        << " RIS gain dropping rapidly (delta="
                        << gainDelta << "dB), preemptive reconfig");

            uint32_t optimalProfile = ComputeOptimalPhaseProfile(gnbId);

            E2RcAction action = BuildAction(
                E2RcActionType::ACTION_THZ_RIS_CONFIG,
                gnbId, 0, 0.75);
            action.parameter1 = static_cast<double>(optimalProfile);
            action.parameter2 = gainDelta; // negative delta as urgency indicator

            m_pendingActions.push_back(action);
            m_metrics.risReconfigurations++;
        }
    }

    // Update metrics
    if (count > 0)
    {
        m_metrics.avgRisGain_dB = sumGain / count;
        m_metrics.avgThzSnr_dB = sumSnr / count;
    }

    // Submit all pending actions
    for (auto& action : m_pendingActions)
    {
        SubmitAction(action);
    }

    RecordDecision(true, 0.83, 0.0);
}

// --------------------------------------------------------------------------
//  GenerateRcActions
// --------------------------------------------------------------------------

std::vector<E2RcAction>
OranNtnXappThzRis::GenerateRcActions()
{
    return m_pendingActions;
}

// --------------------------------------------------------------------------
//  ComputeOptimalPhaseProfile
// --------------------------------------------------------------------------

uint32_t
OranNtnXappThzRis::ComputeOptimalPhaseProfile(uint32_t gnbId) const
{
    NS_LOG_FUNCTION(this << gnbId);

    auto it = m_risStates.find(gnbId);
    if (it == m_risStates.end())
    {
        return 0;
    }

    const ThzRisState& state = it->second;

    // Phase profile selection based on elevation angle and frequency:
    // - Low elevation: use profiles optimized for oblique incidence
    // - High elevation: use profiles optimized for near-normal incidence
    // - Higher frequencies need finer phase quantization

    // Profile index encoding:
    // Bits [7:4] = elevation sector (0-15)
    // Bits [3:0] = frequency band index (0-15)
    uint32_t elevSector = static_cast<uint32_t>(state.elevation_deg / 6.0);
    elevSector = std::min(elevSector, 15u);

    uint32_t freqIndex = 0;
    if (state.thzCenterFreq_GHz > 0.8)
    {
        freqIndex = 3;
    }
    else if (state.thzCenterFreq_GHz > 0.5)
    {
        freqIndex = 2;
    }
    else if (state.thzCenterFreq_GHz > 0.3)
    {
        freqIndex = 1;
    }

    uint32_t profileIndex = (elevSector << 4) | freqIndex;

    NS_LOG_DEBUG("ThzRis: gnb" << gnbId
                 << " optimal profile=" << profileIndex
                 << " (elevSector=" << elevSector
                 << ", freqIndex=" << freqIndex << ")");

    return profileIndex;
}

// --------------------------------------------------------------------------
//  EstimateAchievableGain
// --------------------------------------------------------------------------

double
OranNtnXappThzRis::EstimateAchievableGain(double currentGain_dB,
                                           double thzSnr_dB,
                                           double elevation_deg) const
{
    NS_LOG_FUNCTION(this << currentGain_dB << thzSnr_dB << elevation_deg);

    // RIS gain is bounded by: G_max = 10*log10(N^2) for N elements
    // with ideal phase alignment. Practical gains are lower due to
    // quantization, mutual coupling, and non-ideal phase control.

    // Assume 1024 RIS elements per panel, 4 panels
    double totalElements = 1024.0 * m_numRisPanels;
    double theoreticalMaxGain_dB = 20.0 * std::log10(totalElements);

    // Practical efficiency factor (0.3-0.6 depending on conditions)
    double efficiency = 0.4;
    if (elevation_deg > 60.0)
    {
        efficiency = 0.55; // better alignment at high elevation
    }
    else if (elevation_deg < 20.0)
    {
        efficiency = 0.3; // poor alignment at low elevation
    }

    double achievableGain_dB = theoreticalMaxGain_dB * efficiency;

    // Cap at reasonable value
    achievableGain_dB = std::min(achievableGain_dB, 40.0);

    return achievableGain_dB;
}

// --------------------------------------------------------------------------
//  GetThzRisMetrics
// --------------------------------------------------------------------------

OranNtnXappThzRis::ThzRisMetrics
OranNtnXappThzRis::GetThzRisMetrics() const
{
    return m_metrics;
}

} // namespace ns3
