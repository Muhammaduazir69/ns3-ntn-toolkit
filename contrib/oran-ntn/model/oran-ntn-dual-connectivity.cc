/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-only
 * Author: Muhammad Uzair
 *
 * NTN Dual Connectivity Manager - Implementation
 *
 * Manages dual connectivity between terrestrial mmWave and NTN satellite
 * using McUeNetDevice for simultaneous TN + NTN bearer support.
 */

#include "oran-ntn-dual-connectivity.h"

#include "oran-ntn-sat-bridge.h"

#include <ns3/double.h>
#include <ns3/log.h>
#include <ns3/simulator.h>
#include <ns3/uinteger.h>

#include <algorithm>
#include <cmath>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("OranNtnDualConnectivity");
NS_OBJECT_ENSURE_REGISTERED(OranNtnDualConnectivity);

TypeId
OranNtnDualConnectivity::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::OranNtnDualConnectivity")
            .SetParent<Object>()
            .SetGroupName("OranNtn")
            .AddConstructor<OranNtnDualConnectivity>()
            .AddAttribute("MinSinrForDc",
                          "Minimum SINR (dB) on both TN and NTN links to activate DC",
                          DoubleValue(-3.0),
                          MakeDoubleAccessor(&OranNtnDualConnectivity::m_minSinrForDc_dB),
                          MakeDoubleChecker<double>(-20.0, 40.0))
            .AddAttribute("SplitHysteresis",
                          "Hysteresis threshold for split ratio updates (0-1)",
                          DoubleValue(0.05),
                          MakeDoubleAccessor(&OranNtnDualConnectivity::m_splitHysteresis),
                          MakeDoubleChecker<double>(0.0, 1.0))
            .AddTraceSource("DcStateChanged",
                            "Fired when dual connectivity is activated or deactivated",
                            MakeTraceSourceAccessor(&OranNtnDualConnectivity::m_dcStateChanged),
                            "ns3::TracedCallback::UintBool")
            .AddTraceSource("SplitRatioChanged",
                            "Fired when the bearer split ratio changes",
                            MakeTraceSourceAccessor(&OranNtnDualConnectivity::m_splitRatioChanged),
                            "ns3::TracedCallback::UintDouble")
            .AddTraceSource("PrimarySwitched",
                            "Fired when the primary path switches between TN and NTN",
                            MakeTraceSourceAccessor(&OranNtnDualConnectivity::m_primarySwitched),
                            "ns3::TracedCallback::UintUint8");
    return tid;
}

OranNtnDualConnectivity::OranNtnDualConnectivity()
    : m_satBridge(nullptr),
      m_minSinrForDc_dB(-3.0),
      m_splitHysteresis(0.05)
{
    NS_LOG_FUNCTION(this);
    m_metrics.totalActivations = 0;
    m_metrics.totalDeactivations = 0;
    m_metrics.primarySwitches = 0;
    m_metrics.avgSessionDuration_s = 0.0;
    m_metrics.avgSplitRatio = 0.5;
    m_metrics.avgThroughputGain_pct = 0.0;
}

OranNtnDualConnectivity::~OranNtnDualConnectivity()
{
    NS_LOG_FUNCTION(this);
}

void
OranNtnDualConnectivity::DoDispose()
{
    NS_LOG_FUNCTION(this);
    m_satBridge = nullptr;
    m_sessions.clear();
    Object::DoDispose();
}

// ---------------------------------------------------------------------------
//  Initialize
// ---------------------------------------------------------------------------

void
OranNtnDualConnectivity::Initialize(Ptr<OranNtnSatBridge> satBridge)
{
    NS_LOG_FUNCTION(this << satBridge);
    NS_ABORT_MSG_IF(!satBridge, "OranNtnDualConnectivity::Initialize: satBridge is null");
    m_satBridge = satBridge;
    NS_LOG_INFO("DualConnectivity manager initialized with satellite bridge");
}

// ---------------------------------------------------------------------------
//  ActivateDualConnectivity
// ---------------------------------------------------------------------------

bool
OranNtnDualConnectivity::ActivateDualConnectivity(uint32_t ueId,
                                                    uint32_t tnGnbId,
                                                    uint32_t ntnSatId,
                                                    uint32_t ntnBeamId)
{
    NS_LOG_FUNCTION(this << ueId << tnGnbId << ntnSatId << ntnBeamId);

    // Check if UE is already in DC mode
    if (m_sessions.find(ueId) != m_sessions.end())
    {
        NS_LOG_WARN("UE " << ueId << " is already in dual connectivity mode");
        return false;
    }

    // Validate that both links have adequate SINR from satellite bridge
    NS_ABORT_MSG_IF(!m_satBridge, "Satellite bridge not initialized");

    const UeBridgeState& ueState = m_satBridge->GetUeState(ueId);
    double tnSinr = ueState.servingSinr_dB;   // TN SINR from serving cell
    double ntnSinr = m_satBridge->ComputeNtnSinr(ueId, ntnSatId, ntnBeamId);

    NS_LOG_INFO("UE " << ueId << " TN SINR=" << tnSinr
                       << " dB, NTN SINR=" << ntnSinr << " dB"
                       << " (threshold=" << m_minSinrForDc_dB << " dB)");

    if (tnSinr < m_minSinrForDc_dB || ntnSinr < m_minSinrForDc_dB)
    {
        NS_LOG_INFO("DC activation rejected for UE " << ueId
                     << ": insufficient SINR on one or both links");
        return false;
    }

    // Create DualConnSession entry
    DualConnSession session;
    session.ueId = ueId;
    session.tnGnbId = tnGnbId;
    session.ntnSatId = ntnSatId;
    session.ntnBeamId = ntnBeamId;
    session.tnSplitRatio = 0.5;         // Default split ratio
    session.activationTime = Simulator::Now().GetSeconds();
    session.splitBearerActive = true;
    session.primaryPath = 0;            // TN is primary by default

    m_sessions[ueId] = session;

    // Fire trace callback
    m_dcStateChanged(ueId, true);

    // Update metrics
    m_metrics.totalActivations++;

    NS_LOG_INFO("Dual connectivity activated for UE " << ueId
                << " (TN gNB=" << tnGnbId << ", NTN sat=" << ntnSatId
                << " beam=" << ntnBeamId << ")");

    return true;
}

// ---------------------------------------------------------------------------
//  DeactivateDualConnectivity
// ---------------------------------------------------------------------------

void
OranNtnDualConnectivity::DeactivateDualConnectivity(uint32_t ueId)
{
    NS_LOG_FUNCTION(this << ueId);

    auto it = m_sessions.find(ueId);
    if (it == m_sessions.end())
    {
        NS_LOG_WARN("UE " << ueId << " is not in dual connectivity mode");
        return;
    }

    // Compute session duration for metrics
    double sessionDuration = Simulator::Now().GetSeconds() - it->second.activationTime;
    uint32_t totalSessions = m_metrics.totalActivations;
    if (totalSessions > 0)
    {
        // Running average of session duration
        m_metrics.avgSessionDuration_s =
            ((m_metrics.avgSessionDuration_s * (totalSessions - 1)) + sessionDuration) /
            totalSessions;
    }

    // Remove session
    m_sessions.erase(it);

    // Fire trace callback
    m_dcStateChanged(ueId, false);

    // Update metrics
    m_metrics.totalDeactivations++;

    NS_LOG_INFO("Dual connectivity deactivated for UE " << ueId
                << " (duration=" << sessionDuration << " s)");
}

// ---------------------------------------------------------------------------
//  UpdateSplitRatio
// ---------------------------------------------------------------------------

void
OranNtnDualConnectivity::UpdateSplitRatio(uint32_t ueId, double tnRatio)
{
    NS_LOG_FUNCTION(this << ueId << tnRatio);

    auto it = m_sessions.find(ueId);
    if (it == m_sessions.end())
    {
        NS_LOG_WARN("UE " << ueId << " is not in dual connectivity mode");
        return;
    }

    // Clamp to valid range
    tnRatio = std::max(0.0, std::min(1.0, tnRatio));

    // Apply hysteresis: only update if the change exceeds the threshold
    double currentRatio = it->second.tnSplitRatio;
    double delta = std::abs(tnRatio - currentRatio);

    if (delta < m_splitHysteresis)
    {
        NS_LOG_DEBUG("Split ratio change for UE " << ueId
                     << " below hysteresis threshold (" << delta
                     << " < " << m_splitHysteresis << "), skipping update");
        return;
    }

    // Update session split ratio
    it->second.tnSplitRatio = tnRatio;

    // Fire trace callback
    m_splitRatioChanged(ueId, tnRatio);

    // Update average split ratio metric across all active sessions
    if (!m_sessions.empty())
    {
        double sum = 0.0;
        for (const auto& s : m_sessions)
        {
            sum += s.second.tnSplitRatio;
        }
        m_metrics.avgSplitRatio = sum / static_cast<double>(m_sessions.size());
    }

    NS_LOG_INFO("Split ratio updated for UE " << ueId
                << ": " << currentRatio << " -> " << tnRatio);
}

// ---------------------------------------------------------------------------
//  SwitchPrimaryPath
// ---------------------------------------------------------------------------

void
OranNtnDualConnectivity::SwitchPrimaryPath(uint32_t ueId, uint8_t primary)
{
    NS_LOG_FUNCTION(this << ueId << static_cast<uint32_t>(primary));

    auto it = m_sessions.find(ueId);
    if (it == m_sessions.end())
    {
        NS_LOG_WARN("UE " << ueId << " is not in dual connectivity mode");
        return;
    }

    if (primary > 1)
    {
        NS_LOG_WARN("Invalid primary path value " << static_cast<uint32_t>(primary)
                     << " for UE " << ueId << " (must be 0=TN or 1=NTN)");
        return;
    }

    if (it->second.primaryPath == primary)
    {
        NS_LOG_DEBUG("UE " << ueId << " already has primary path set to "
                     << static_cast<uint32_t>(primary));
        return;
    }

    // Update session primary path
    it->second.primaryPath = primary;

    // Fire trace callback
    m_primarySwitched(ueId, primary);

    // Increment primary switch counter
    m_metrics.primarySwitches++;

    NS_LOG_INFO("Primary path switched for UE " << ueId
                << " to " << (primary == 0 ? "TN" : "NTN"));
}

// ---------------------------------------------------------------------------
//  IsDualConnected
// ---------------------------------------------------------------------------

bool
OranNtnDualConnectivity::IsDualConnected(uint32_t ueId) const
{
    return m_sessions.find(ueId) != m_sessions.end();
}

// ---------------------------------------------------------------------------
//  GetSession
// ---------------------------------------------------------------------------

DualConnSession
OranNtnDualConnectivity::GetSession(uint32_t ueId) const
{
    NS_LOG_FUNCTION(this << ueId);

    auto it = m_sessions.find(ueId);
    if (it != m_sessions.end())
    {
        return it->second;
    }

    // Return empty session if UE is not in DC mode
    DualConnSession empty;
    empty.ueId = ueId;
    empty.tnGnbId = 0;
    empty.ntnSatId = 0;
    empty.ntnBeamId = 0;
    empty.tnSplitRatio = 0.0;
    empty.activationTime = 0.0;
    empty.splitBearerActive = false;
    empty.primaryPath = 0;
    return empty;
}

// ---------------------------------------------------------------------------
//  EvaluateDcBenefit
// ---------------------------------------------------------------------------

double
OranNtnDualConnectivity::EvaluateDcBenefit(uint32_t ueId) const
{
    NS_LOG_FUNCTION(this << ueId);
    NS_ABORT_MSG_IF(!m_satBridge, "Satellite bridge not initialized");

    double score = 0.0;

    // Get TN and NTN SINR from satellite bridge
    const UeBridgeState& ueState = m_satBridge->GetUeState(ueId);
    double tnSinr = ueState.servingSinr_dB;

    // Find best visible satellite for NTN SINR
    std::vector<uint32_t> visible = m_satBridge->GetVisibleSatellites(ueId);
    double ntnSinr = -100.0; // very low default
    uint32_t bestSat = 0;
    uint32_t bestBeam = 0;

    for (uint32_t satId : visible)
    {
        uint32_t beam = m_satBridge->FindBestBeam(ueId, satId);
        double sinr = m_satBridge->ComputeNtnSinr(ueId, satId, beam);
        if (sinr > ntnSinr)
        {
            ntnSinr = sinr;
            bestSat = satId;
            bestBeam = beam;
        }
    }

    NS_LOG_DEBUG("EvaluateDcBenefit UE " << ueId
                 << ": tnSinr=" << tnSinr << " dB, ntnSinr=" << ntnSinr
                 << " dB (bestSat=" << bestSat << " beam=" << bestBeam << ")");

    // Factor 1: Both links above minimum threshold? (+1.0)
    if (tnSinr >= m_minSinrForDc_dB && ntnSinr >= m_minSinrForDc_dB)
    {
        score += 1.0;
    }
    else
    {
        // If either link is below threshold, DC is not feasible
        NS_LOG_DEBUG("DC not feasible: one or both links below min SINR threshold");
        return score;
    }

    // Factor 2: Throughput gain estimate
    // Shannon-like capacity approximation: C ~ log2(1 + 10^(SINR/10))
    double tnCapacity = std::log2(1.0 + std::pow(10.0, tnSinr / 10.0));
    double ntnCapacity = std::log2(1.0 + std::pow(10.0, ntnSinr / 10.0));
    double combinedCapacity = tnCapacity + ntnCapacity;
    double maxSingle = std::max(tnCapacity, ntnCapacity);

    if (maxSingle > 0.0 && combinedCapacity > maxSingle)
    {
        double gainPct = ((combinedCapacity - maxSingle) / maxSingle) * 100.0;
        score += gainPct / 100.0;  // Normalized gain contribution
        NS_LOG_DEBUG("Throughput gain: " << gainPct << "% -> +" << (gainPct / 100.0));
    }

    // Factor 3: Latency - can use lower-latency TN for URLLC? (+0.5)
    // TN typically has much lower latency than NTN (satellite propagation delay)
    if (ueState.sliceId == 1) // URLLC slice
    {
        // DC allows routing URLLC traffic via TN while keeping NTN for eMBB
        score += 0.5;
        NS_LOG_DEBUG("URLLC UE benefits from TN low-latency path: +0.5");
    }

    // Factor 4: Cost - DC uses more UE battery (-0.3)
    score -= 0.3;
    NS_LOG_DEBUG("Battery cost penalty: -0.3");

    NS_LOG_INFO("DC benefit score for UE " << ueId << ": " << score);
    return score;
}

// ---------------------------------------------------------------------------
//  GetTnLinkQuality / GetNtnLinkQuality
// ---------------------------------------------------------------------------

double
OranNtnDualConnectivity::GetTnLinkQuality(uint32_t ueId) const
{
    NS_LOG_FUNCTION(this << ueId);

    // If UE is in DC mode, return stored session data via sat bridge
    auto it = m_sessions.find(ueId);
    if (it != m_sessions.end() && m_satBridge)
    {
        const UeBridgeState& ueState = m_satBridge->GetUeState(ueId);
        return ueState.servingSinr_dB;
    }

    // Fallback: query sat bridge directly
    if (m_satBridge)
    {
        const UeBridgeState& ueState = m_satBridge->GetUeState(ueId);
        return ueState.servingSinr_dB;
    }

    NS_LOG_WARN("Cannot get TN link quality: satellite bridge not initialized");
    return -100.0;
}

double
OranNtnDualConnectivity::GetNtnLinkQuality(uint32_t ueId) const
{
    NS_LOG_FUNCTION(this << ueId);

    // If UE is in DC mode, compute NTN SINR for the session's satellite/beam
    auto it = m_sessions.find(ueId);
    if (it != m_sessions.end() && m_satBridge)
    {
        return m_satBridge->ComputeNtnSinr(ueId,
                                            it->second.ntnSatId,
                                            it->second.ntnBeamId);
    }

    // Fallback: find best visible satellite
    if (m_satBridge)
    {
        std::vector<uint32_t> visible = m_satBridge->GetVisibleSatellites(ueId);
        double bestSinr = -100.0;
        for (uint32_t satId : visible)
        {
            uint32_t beam = m_satBridge->FindBestBeam(ueId, satId);
            double sinr = m_satBridge->ComputeNtnSinr(ueId, satId, beam);
            if (sinr > bestSinr)
            {
                bestSinr = sinr;
            }
        }
        return bestSinr;
    }

    NS_LOG_WARN("Cannot get NTN link quality: satellite bridge not initialized");
    return -100.0;
}

// ---------------------------------------------------------------------------
//  GetAllSessions
// ---------------------------------------------------------------------------

std::vector<DualConnSession>
OranNtnDualConnectivity::GetAllSessions() const
{
    NS_LOG_FUNCTION(this);

    std::vector<DualConnSession> sessions;
    sessions.reserve(m_sessions.size());

    for (const auto& entry : m_sessions)
    {
        sessions.push_back(entry.second);
    }

    return sessions;
}

// ---------------------------------------------------------------------------
//  GetMetrics
// ---------------------------------------------------------------------------

OranNtnDualConnectivity::DcMetrics
OranNtnDualConnectivity::GetMetrics() const
{
    NS_LOG_FUNCTION(this);
    return m_metrics;
}

} // namespace ns3
