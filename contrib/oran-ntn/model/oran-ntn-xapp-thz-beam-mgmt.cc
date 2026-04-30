/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-only
 * Author: Muhammad Uzair
 *
 * THz Beam Management xApp - Implementation
 *
 * Monitors THz beam tracking error and SNR, triggers hierarchical beam
 * search when pointing error exceeds threshold, coordinates beam handover
 * during LEO pass using TTE, and uses orbital prediction for proactive
 * beam pre-steering.
 */

#include "oran-ntn-xapp-thz-beam-mgmt.h"

#include <ns3/double.h>
#include <ns3/log.h>
#include <ns3/simulator.h>
#include <ns3/uinteger.h>

#include <algorithm>
#include <cmath>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("OranNtnXappThzBeamMgmt");
NS_OBJECT_ENSURE_REGISTERED(OranNtnXappThzBeamMgmt);

TypeId
OranNtnXappThzBeamMgmt::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::OranNtnXappThzBeamMgmt")
            .SetParent<OranNtnXappBase>()
            .SetGroupName("OranNtn")
            .AddConstructor<OranNtnXappThzBeamMgmt>()
            .AddAttribute("PointingErrorThreshold",
                          "Beam pointing error threshold (degrees) to trigger beam search",
                          DoubleValue(0.5),
                          MakeDoubleAccessor(&OranNtnXappThzBeamMgmt::m_pointingErrorThreshold_deg),
                          MakeDoubleChecker<double>(0.0))
            .AddAttribute("SnrThreshold",
                          "Minimum acceptable THz link SNR (dB)",
                          DoubleValue(5.0),
                          MakeDoubleAccessor(&OranNtnXappThzBeamMgmt::m_snrThreshold_dB),
                          MakeDoubleChecker<double>())
            .AddAttribute("TtePreSteerThreshold",
                          "Pre-steer beam when TTE is below this value (seconds)",
                          DoubleValue(30.0),
                          MakeDoubleAccessor(&OranNtnXappThzBeamMgmt::m_ttePreSteerThreshold_s),
                          MakeDoubleChecker<double>(0.0))
            .AddAttribute("BeamSquintThreshold",
                          "Maximum tolerable beam squint loss (dB)",
                          DoubleValue(3.0),
                          MakeDoubleAccessor(&OranNtnXappThzBeamMgmt::m_beamSquintThreshold_dB),
                          MakeDoubleChecker<double>(0.0));
    return tid;
}

OranNtnXappThzBeamMgmt::OranNtnXappThzBeamMgmt()
    : m_pointingErrorThreshold_deg(0.5),
      m_snrThreshold_dB(5.0),
      m_ttePreSteerThreshold_s(30.0),
      m_beamSquintThreshold_dB(3.0)
{
    NS_LOG_FUNCTION(this);
    SetXappName("ThzBeamMgmt");

    m_metrics = {};
}

OranNtnXappThzBeamMgmt::~OranNtnXappThzBeamMgmt()
{
    NS_LOG_FUNCTION(this);
}

// --------------------------------------------------------------------------
//  E2 subscription
// --------------------------------------------------------------------------

E2Subscription
OranNtnXappThzBeamMgmt::GetRequiredSubscription() const
{
    E2Subscription sub;
    sub.subscriptionId = 0;
    sub.ricRequestorId = GetXappId();
    sub.ranFunctionId = 2; // KPM service model
    sub.reportingPeriod = MilliSeconds(100); // Fast updates for beam tracking
    sub.eventTrigger = false;
    sub.eventThreshold = 0.0;
    sub.batchOnVisibility = false;
    sub.maxBufferAge = Seconds(2);
    sub.useIslRelay = true;
    return sub;
}

// --------------------------------------------------------------------------
//  ProcessKpmReport
// --------------------------------------------------------------------------

void
OranNtnXappThzBeamMgmt::ProcessKpmReport(const E2KpmReport& report)
{
    NS_LOG_FUNCTION(this << report.gnbId << report.beamId);

    uint32_t gnbId = report.gnbId;

    // Update THz beam state from KPM report
    ThzBeamState& state = m_beamStates[gnbId];
    state.pointingError_deg = report.pointingError_deg;
    state.thzSnr_dB = report.thzSnr_dB;
    state.tte_s = report.tte_s;
    state.beamSquintLoss_dB = report.beamSquintLoss_dB;
    state.currentBeamId = report.beamId;
    state.umMimoElements = report.umMimoElements;
    state.centerFreq_GHz = report.thzCenterFreq_GHz;

    NS_LOG_DEBUG("ThzBeamMgmt: gnb=" << gnbId
                 << " pointErr=" << state.pointingError_deg << "deg"
                 << " thzSnr=" << state.thzSnr_dB << "dB"
                 << " tte=" << state.tte_s << "s"
                 << " squint=" << state.beamSquintLoss_dB << "dB");
}

// --------------------------------------------------------------------------
//  DecisionCycle
// --------------------------------------------------------------------------

void
OranNtnXappThzBeamMgmt::DecisionCycle()
{
    NS_LOG_FUNCTION(this);

    m_pendingActions.clear();

    double sumPointErr = 0.0;
    double sumSnr = 0.0;
    uint32_t count = 0;

    for (auto& kv : m_beamStates)
    {
        uint32_t gnbId = kv.first;
        const ThzBeamState& state = kv.second;

        sumPointErr += state.pointingError_deg;
        sumSnr += state.thzSnr_dB;
        count++;

        // Check 1: Pointing error exceeds threshold -- trigger beam search
        if (NeedsBeamSearch(gnbId))
        {
            NS_LOG_INFO("ThzBeamMgmt: gnb" << gnbId
                        << " pointing error " << state.pointingError_deg
                        << "deg exceeds threshold, triggering beam search");

            // Hierarchical beam search: start with wide beam, narrow down
            E2RcAction action = BuildAction(E2RcActionType::ACTION_THZ_BEAM_CODEBOOK,
                                            gnbId,
                                            0, // cell-wide
                                            0.85);
            // parameter1 = target beam index from search
            // parameter2 = search level (0=wide, 1=medium, 2=narrow)
            action.targetBeamId = state.currentBeamId;
            action.parameter1 = static_cast<double>(state.currentBeamId + 1);
            action.parameter2 = 0.0; // start wide search

            m_pendingActions.push_back(action);
            m_metrics.beamSearchesTriggered++;
        }

        // Check 2: TTE approaching zero -- proactive pre-steer
        if (state.tte_s > 0.0 && state.tte_s < m_ttePreSteerThreshold_s)
        {
            uint32_t targetBeam = ComputePreSteerBeamIndex(gnbId, state.tte_s);
            if (targetBeam != state.currentBeamId)
            {
                NS_LOG_INFO("ThzBeamMgmt: gnb" << gnbId
                            << " TTE=" << state.tte_s
                            << "s, pre-steering to beam " << targetBeam);

                E2RcAction action = BuildAction(E2RcActionType::ACTION_THZ_BEAM_CODEBOOK,
                                                gnbId,
                                                0,
                                                0.90);
                action.targetBeamId = targetBeam;
                action.parameter1 = static_cast<double>(targetBeam);
                action.parameter2 = state.tte_s;

                m_pendingActions.push_back(action);
                m_metrics.proactivePreSteers++;
            }
        }

        // Check 3: SNR below threshold with acceptable pointing -- may need handover
        if (state.thzSnr_dB < m_snrThreshold_dB &&
            state.pointingError_deg < m_pointingErrorThreshold_deg)
        {
            NS_LOG_INFO("ThzBeamMgmt: gnb" << gnbId
                        << " THz SNR=" << state.thzSnr_dB
                        << "dB below threshold despite good pointing");

            E2RcAction action = BuildAction(E2RcActionType::ACTION_THZ_BEAM_CODEBOOK,
                                            gnbId,
                                            0,
                                            0.75);
            action.targetBeamId = state.currentBeamId;
            action.parameter1 = state.thzSnr_dB;
            action.parameter2 = static_cast<double>(state.umMimoElements);

            m_pendingActions.push_back(action);
            m_metrics.beamHandovers++;
        }
    }

    // Update metrics
    if (count > 0)
    {
        m_metrics.avgPointingError_deg = sumPointErr / count;
        m_metrics.avgThzSnr_dB = sumSnr / count;
    }

    // Submit all pending actions
    for (auto& action : m_pendingActions)
    {
        SubmitAction(action);
    }

    RecordDecision(true, 0.85, 0.0);
}

// --------------------------------------------------------------------------
//  GenerateRcActions
// --------------------------------------------------------------------------

std::vector<E2RcAction>
OranNtnXappThzBeamMgmt::GenerateRcActions()
{
    return m_pendingActions;
}

// --------------------------------------------------------------------------
//  NeedsBeamSearch
// --------------------------------------------------------------------------

bool
OranNtnXappThzBeamMgmt::NeedsBeamSearch(uint32_t gnbId) const
{
    auto it = m_beamStates.find(gnbId);
    if (it == m_beamStates.end())
    {
        return false;
    }

    const ThzBeamState& state = it->second;

    // Trigger beam search if pointing error exceeds threshold
    if (state.pointingError_deg > m_pointingErrorThreshold_deg)
    {
        return true;
    }

    // Also trigger if beam squint loss is too high
    if (state.beamSquintLoss_dB > m_beamSquintThreshold_dB)
    {
        return true;
    }

    return false;
}

// --------------------------------------------------------------------------
//  ComputePreSteerBeamIndex -- orbital prediction for next beam
// --------------------------------------------------------------------------

uint32_t
OranNtnXappThzBeamMgmt::ComputePreSteerBeamIndex(uint32_t gnbId, double tte_s) const
{
    NS_LOG_FUNCTION(this << gnbId << tte_s);

    auto it = m_beamStates.find(gnbId);
    if (it == m_beamStates.end())
    {
        return 0;
    }

    // Simple orbital prediction: as TTE decreases, the UE moves toward
    // the next beam in the coverage sequence. Estimate the next beam
    // based on current beam ID and remaining time.
    uint32_t currentBeam = it->second.currentBeamId;

    // For LEO satellites with sequential beam layout, next beam is typically
    // current + 1 in the direction of satellite motion
    uint32_t nextBeam = currentBeam + 1;

    NS_LOG_DEBUG("ThzBeamMgmt: pre-steer from beam " << currentBeam
                 << " to beam " << nextBeam
                 << " (TTE=" << tte_s << "s)");

    return nextBeam;
}

// --------------------------------------------------------------------------
//  EstimateBeamSquintOffset
// --------------------------------------------------------------------------

double
OranNtnXappThzBeamMgmt::EstimateBeamSquintOffset(double centerFreq_GHz,
                                                   double bandwidth_GHz) const
{
    // Beam squint is proportional to fractional bandwidth
    // squint_offset ~ arcsin(BW / (2 * fc)) for phased arrays
    if (centerFreq_GHz <= 0.0)
    {
        return 0.0;
    }

    double fractionalBw = bandwidth_GHz / centerFreq_GHz;
    double squintOffset_rad = std::asin(fractionalBw / 2.0);
    double squintOffset_deg = squintOffset_rad * 180.0 / M_PI;

    return squintOffset_deg;
}

// --------------------------------------------------------------------------
//  GetThzBeamMetrics
// --------------------------------------------------------------------------

OranNtnXappThzBeamMgmt::ThzBeamMetrics
OranNtnXappThzBeamMgmt::GetThzBeamMetrics() const
{
    return m_metrics;
}

} // namespace ns3
