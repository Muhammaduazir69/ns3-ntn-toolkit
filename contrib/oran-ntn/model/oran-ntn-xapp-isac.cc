/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-only
 * Author: Muhammad Uzair
 *
 * ISAC (Integrated Sensing and Communication) xApp - Implementation
 *
 * Manages the sensing/communication resource split for THz-NTN links.
 * Processes debris detection alerts and adjusts the ISAC resource
 * partition based on traffic load.
 */

#include "oran-ntn-xapp-isac.h"

#include <ns3/double.h>
#include <ns3/log.h>
#include <ns3/simulator.h>

#include <algorithm>
#include <cmath>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("OranNtnXappIsac");
NS_OBJECT_ENSURE_REGISTERED(OranNtnXappIsac);

TypeId
OranNtnXappIsac::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::OranNtnXappIsac")
            .SetParent<OranNtnXappBase>()
            .SetGroupName("OranNtn")
            .AddConstructor<OranNtnXappIsac>()
            .AddAttribute("HighTrafficThreshold",
                          "PRB utilization above this is considered high traffic (0-1)",
                          DoubleValue(0.7),
                          MakeDoubleAccessor(&OranNtnXappIsac::m_highTrafficThreshold),
                          MakeDoubleChecker<double>(0.0, 1.0))
            .AddAttribute("LowTrafficThreshold",
                          "PRB utilization below this is considered low traffic (0-1)",
                          DoubleValue(0.3),
                          MakeDoubleAccessor(&OranNtnXappIsac::m_lowTrafficThreshold),
                          MakeDoubleChecker<double>(0.0, 1.0))
            .AddAttribute("DebrisAlertRange",
                          "Alert threshold for debris detection range (km)",
                          DoubleValue(50.0),
                          MakeDoubleAccessor(&OranNtnXappIsac::m_debrisAlertRange_km),
                          MakeDoubleChecker<double>(0.0))
            .AddAttribute("MinSensingRatio",
                          "Minimum fraction of resources allocated to sensing (0-1)",
                          DoubleValue(0.1),
                          MakeDoubleAccessor(&OranNtnXappIsac::m_minSensingRatio),
                          MakeDoubleChecker<double>(0.0, 1.0))
            .AddAttribute("MaxSensingRatio",
                          "Maximum fraction of resources allocated to sensing (0-1)",
                          DoubleValue(0.6),
                          MakeDoubleAccessor(&OranNtnXappIsac::m_maxSensingRatio),
                          MakeDoubleChecker<double>(0.0, 1.0));
    return tid;
}

OranNtnXappIsac::OranNtnXappIsac()
    : m_highTrafficThreshold(0.7),
      m_lowTrafficThreshold(0.3),
      m_debrisAlertRange_km(50.0),
      m_minSensingRatio(0.1),
      m_maxSensingRatio(0.6)
{
    NS_LOG_FUNCTION(this);
    SetXappName("Isac");

    m_metrics = {};
    m_metrics.minDebrisRange_km = 1e6; // initialize to large value
}

OranNtnXappIsac::~OranNtnXappIsac()
{
    NS_LOG_FUNCTION(this);
}

// --------------------------------------------------------------------------
//  E2 subscription
// --------------------------------------------------------------------------

E2Subscription
OranNtnXappIsac::GetRequiredSubscription() const
{
    E2Subscription sub;
    sub.subscriptionId = 0;
    sub.ricRequestorId = GetXappId();
    sub.ranFunctionId = 2; // KPM service model
    sub.reportingPeriod = MilliSeconds(200);
    sub.eventTrigger = false;
    sub.eventThreshold = 0.0;
    // ORAN-05: left at the default (true) so a feeder outage buffers on board
    // instead of silently discarding this xApp's telemetry.
    sub.maxBufferAge = Seconds(5);
    sub.useIslRelay = true;
    return sub;
}

// --------------------------------------------------------------------------
//  ProcessKpmReport
// --------------------------------------------------------------------------

void
OranNtnXappIsac::ProcessKpmReport(const E2KpmReport& report)
{
    NS_LOG_FUNCTION(this << report.gnbId);

    uint32_t gnbId = report.gnbId;

    IsacState& state = m_isacStates[gnbId];
    state.sensingActive = report.isacSensingActive;
    state.debrisDetectionRange_km = report.debrisDetectionRange_km;
    state.prbUtilization = report.prbUtilization;
    state.thzSnr_dB = report.thzSnr_dB;
    state.activeWaveform = report.activeWaveform;

    // Process debris alerts immediately
    if (report.debrisDetectionRange_km > 0.0 &&
        report.debrisDetectionRange_km < m_debrisAlertRange_km)
    {
        ProcessDebrisAlert(gnbId, report.debrisDetectionRange_km);
    }

    NS_LOG_DEBUG("Isac: gnb=" << gnbId
                 << " sensing=" << state.sensingActive
                 << " debris=" << state.debrisDetectionRange_km << "km"
                 << " prb=" << state.prbUtilization
                 << " waveform=" << state.activeWaveform);
}

// --------------------------------------------------------------------------
//  DecisionCycle
// --------------------------------------------------------------------------

void
OranNtnXappIsac::DecisionCycle()
{
    NS_LOG_FUNCTION(this);

    m_pendingActions.clear();

    double sumSensing = 0.0;
    double sumComm = 0.0;
    uint32_t count = 0;

    for (auto& kv : m_isacStates)
    {
        uint32_t gnbId = kv.first;
        IsacState& state = kv.second;

        // Compute optimal sensing/communication split
        double optimalSensingRatio = ComputeOptimalSensingRatio(gnbId);

        sumSensing += optimalSensingRatio;
        sumComm += (1.0 - optimalSensingRatio);
        count++;

        // Check if mode change is needed
        bool shouldSense = (optimalSensingRatio > m_minSensingRatio);
        bool currentlySensing = state.sensingActive;

        if (shouldSense != currentlySensing)
        {
            NS_LOG_INFO("Isac: gnb" << gnbId
                        << " switching ISAC mode: sensing="
                        << (shouldSense ? "ON" : "OFF")
                        << " ratio=" << optimalSensingRatio);

            E2RcAction action = BuildAction(
                E2RcActionType::ACTION_THZ_ISAC_MODE,
                gnbId, 0, 0.85);
            action.parameter1 = shouldSense ? 1.0 : 0.0;
            action.parameter2 = optimalSensingRatio;

            m_pendingActions.push_back(action);
            m_metrics.modeChanges++;
        }
        else if (state.sensingActive)
        {
            // Already sensing, but update resource split if significantly different
            double currentRatio = state.currentSensingRatio;
            if (std::abs(optimalSensingRatio - currentRatio) > 0.1)
            {
                NS_LOG_INFO("Isac: gnb" << gnbId
                            << " updating sensing ratio from "
                            << currentRatio << " to " << optimalSensingRatio);

                E2RcAction action = BuildAction(
                    E2RcActionType::ACTION_THZ_ISAC_MODE,
                    gnbId, 0, 0.80);
                action.parameter1 = 1.0; // keep sensing on
                action.parameter2 = optimalSensingRatio;

                m_pendingActions.push_back(action);
            }
        }

        state.currentSensingRatio = optimalSensingRatio;
    }

    // Update metrics
    if (count > 0)
    {
        m_metrics.avgSensingRatio = sumSensing / count;
        m_metrics.avgCommRatio = sumComm / count;
    }

    // Submit all pending actions
    for (auto& action : m_pendingActions)
    {
        SubmitAction(action);
    }

    RecordDecision(true, 0.80, 0.0);
}

// --------------------------------------------------------------------------
//  GenerateRcActions
// --------------------------------------------------------------------------

std::vector<E2RcAction>
OranNtnXappIsac::GenerateRcActions()
{
    return m_pendingActions;
}

// --------------------------------------------------------------------------
//  ComputeOptimalSensingRatio
// --------------------------------------------------------------------------

double
OranNtnXappIsac::ComputeOptimalSensingRatio(uint32_t gnbId) const
{
    NS_LOG_FUNCTION(this << gnbId);

    auto it = m_isacStates.find(gnbId);
    if (it == m_isacStates.end())
    {
        return m_minSensingRatio;
    }

    const IsacState& state = it->second;
    double sensingRatio = m_minSensingRatio;

    // Traffic-aware resource partitioning:
    // - Low traffic: allocate more resources to sensing
    // - High traffic: minimize sensing, maximize communication
    if (state.prbUtilization < m_lowTrafficThreshold)
    {
        // Low traffic: use up to max sensing ratio
        // Scale linearly: at 0% utilization -> maxSensingRatio
        //                 at lowThreshold    -> midpoint
        double scale = 1.0 - (state.prbUtilization / m_lowTrafficThreshold);
        sensingRatio = m_minSensingRatio +
                       scale * (m_maxSensingRatio - m_minSensingRatio);
    }
    else if (state.prbUtilization > m_highTrafficThreshold)
    {
        // High traffic: use minimum sensing
        sensingRatio = m_minSensingRatio;
    }
    else
    {
        // Medium traffic: linear interpolation
        double range = m_highTrafficThreshold - m_lowTrafficThreshold;
        double position = (state.prbUtilization - m_lowTrafficThreshold) / range;
        double midRatio = (m_minSensingRatio + m_maxSensingRatio) / 2.0;
        sensingRatio = midRatio - position * (midRatio - m_minSensingRatio);
    }

    // Boost sensing if debris was recently detected nearby
    if (state.debrisDetectionRange_km > 0.0 &&
        state.debrisDetectionRange_km < m_debrisAlertRange_km)
    {
        // Closer debris -> higher sensing priority
        double urgency = 1.0 - (state.debrisDetectionRange_km / m_debrisAlertRange_km);
        double boost = urgency * 0.2; // up to 20% boost
        sensingRatio = std::min(sensingRatio + boost, m_maxSensingRatio);
    }

    // Clamp to valid range
    sensingRatio = std::max(m_minSensingRatio, std::min(sensingRatio, m_maxSensingRatio));

    NS_LOG_DEBUG("Isac: gnb" << gnbId
                 << " optimalSensingRatio=" << sensingRatio
                 << " (prb=" << state.prbUtilization
                 << ", debris=" << state.debrisDetectionRange_km << "km)");

    return sensingRatio;
}

// --------------------------------------------------------------------------
//  ProcessDebrisAlert
// --------------------------------------------------------------------------

void
OranNtnXappIsac::ProcessDebrisAlert(uint32_t gnbId, double range_km)
{
    NS_LOG_FUNCTION(this << gnbId << range_km);

    m_metrics.debrisAlerts++;
    m_metrics.minDebrisRange_km = std::min(m_metrics.minDebrisRange_km, range_km);

    NS_LOG_WARN("Isac: DEBRIS ALERT gnb" << gnbId
                << " range=" << range_km << "km");

    // Immediately increase sensing if debris is very close
    if (range_km < m_debrisAlertRange_km * 0.5)
    {
        E2RcAction action = BuildAction(
            E2RcActionType::ACTION_THZ_ISAC_MODE,
            gnbId, 0, 0.95);
        action.parameter1 = 1.0; // sensing ON
        action.parameter2 = m_maxSensingRatio; // maximum sensing

        m_pendingActions.push_back(action);
        m_metrics.modeChanges++;

        NS_LOG_WARN("Isac: emergency sensing mode for gnb" << gnbId
                    << " (debris at " << range_km << "km)");
    }
}

// --------------------------------------------------------------------------
//  GetIsacMetrics
// --------------------------------------------------------------------------

OranNtnXappIsac::IsacMetrics
OranNtnXappIsac::GetIsacMetrics() const
{
    return m_metrics;
}

} // namespace ns3
