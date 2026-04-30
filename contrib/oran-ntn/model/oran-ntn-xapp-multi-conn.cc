/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-only
 * Author: Muhammad Uzair
 *
 * Multi-Connectivity Orchestrator xApp - Implementation
 *
 * Manages TN-NTN dual connectivity: activation/teardown decisions,
 * optimal bearer split ratio computation, predictive DC setup before
 * satellite handovers, and QoS-aware primary path selection.
 */

#include "oran-ntn-xapp-multi-conn.h"

#include "oran-ntn-dual-connectivity.h"
#include "oran-ntn-sat-bridge.h"

#include <ns3/boolean.h>
#include <ns3/double.h>
#include <ns3/log.h>
#include <ns3/simulator.h>

#include <algorithm>
#include <cmath>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("OranNtnXappMultiConn");
NS_OBJECT_ENSURE_REGISTERED(OranNtnXappMultiConn);

TypeId
OranNtnXappMultiConn::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::OranNtnXappMultiConn")
            .SetParent<OranNtnXappBase>()
            .SetGroupName("OranNtn")
            .AddConstructor<OranNtnXappMultiConn>()
            .AddAttribute("MinSinrForDc",
                          "Minimum SINR in dB for DC activation on each link",
                          DoubleValue(-3.0),
                          MakeDoubleAccessor(&OranNtnXappMultiConn::m_minSinrForDc_dB),
                          MakeDoubleChecker<double>())
            .AddAttribute("DcBenefitThreshold",
                          "Minimum benefit score to justify DC activation",
                          DoubleValue(0.2),
                          MakeDoubleAccessor(&OranNtnXappMultiConn::m_dcBenefitThreshold),
                          MakeDoubleChecker<double>(0.0, 1.0))
            .AddAttribute("BatteryAware",
                          "Consider UE battery in DC decisions",
                          BooleanValue(false),
                          MakeBooleanAccessor(&OranNtnXappMultiConn::m_batteryAware),
                          MakeBooleanChecker())
            .AddAttribute("PredictiveDc",
                          "Enable predictive DC setup before satellite HO",
                          BooleanValue(true),
                          MakeBooleanAccessor(&OranNtnXappMultiConn::m_predictiveDc),
                          MakeBooleanChecker());
    return tid;
}

OranNtnXappMultiConn::OranNtnXappMultiConn()
    : m_minSinrForDc_dB(-3.0),
      m_dcBenefitThreshold(0.2),
      m_batteryAware(false),
      m_predictiveDc(true)
{
    NS_LOG_FUNCTION(this);
    SetXappName("MultiConn");

    m_mcMetrics = {};
}

OranNtnXappMultiConn::~OranNtnXappMultiConn()
{
    NS_LOG_FUNCTION(this);
}

// --------------------------------------------------------------------------
//  Configuration setters
// --------------------------------------------------------------------------

void
OranNtnXappMultiConn::SetSatBridge(Ptr<OranNtnSatBridge> bridge)
{
    NS_LOG_FUNCTION(this << bridge);
    m_satBridge = bridge;
}

void
OranNtnXappMultiConn::SetDualConnManager(Ptr<OranNtnDualConnectivity> dcMgr)
{
    NS_LOG_FUNCTION(this << dcMgr);
    m_dcMgr = dcMgr;
}

void
OranNtnXappMultiConn::SetMinSinrForDc(double minSinr_dB)
{
    m_minSinrForDc_dB = minSinr_dB;
}

void
OranNtnXappMultiConn::SetDcBenefitThreshold(double threshold)
{
    m_dcBenefitThreshold = threshold;
}

void
OranNtnXappMultiConn::SetBatteryAware(bool enable)
{
    m_batteryAware = enable;
}

void
OranNtnXappMultiConn::SetPredictiveDc(bool enable)
{
    m_predictiveDc = enable;
}

// --------------------------------------------------------------------------
//  Public queries
// --------------------------------------------------------------------------

OranNtnXappMultiConn::MultiConnMetrics
OranNtnXappMultiConn::GetMultiConnMetrics() const
{
    return m_mcMetrics;
}

// --------------------------------------------------------------------------
//  E2 subscription
// --------------------------------------------------------------------------

E2Subscription
OranNtnXappMultiConn::GetRequiredSubscription() const
{
    E2Subscription sub;
    sub.subscriptionId = 0; // Assigned by E2 termination
    sub.ricRequestorId = GetXappId();
    sub.ranFunctionId = 2;  // KPM service model
    sub.reportingPeriod = MilliSeconds(100);
    sub.eventTrigger = false; // Periodic reporting
    sub.eventThreshold = 0.0;
    sub.batchOnVisibility = false;
    sub.maxBufferAge = Seconds(1);
    sub.useIslRelay = true;
    return sub;
}

// --------------------------------------------------------------------------
//  ProcessKpmReport -- update per-UE TN/NTN state from KPM
// --------------------------------------------------------------------------

void
OranNtnXappMultiConn::ProcessKpmReport(const E2KpmReport& report)
{
    NS_LOG_FUNCTION(this << report.ueId);

    uint32_t ueId = report.ueId;
    auto& state = m_ueStates[ueId];

    // Update link quality from KPM fields
    state.tnSinr_dB = report.tnSinr_dB;
    state.ntnSinr_dB = report.ntnSinr_dB;
    state.tnThroughput = report.tnThroughput_Mbps;
    state.ntnThroughput = report.ntnThroughput_Mbps;
    state.currentSplitRatio = report.bearerSplitRatio;
    state.dcActive = report.dualConnected;

    // Estimate load from PRB utilization (use serving cell load)
    state.tnLoad = report.prbUtilization;   // approximate
    state.ntnLoad = report.prbUtilization;

    state.lastEvaluationTime = Simulator::Now().GetSeconds();

    NS_LOG_DEBUG("MultiConn: UE" << ueId << " tnSINR=" << state.tnSinr_dB
                                 << " ntnSINR=" << state.ntnSinr_dB
                                 << " DC=" << state.dcActive
                                 << " split=" << state.currentSplitRatio);
}

// --------------------------------------------------------------------------
//  DecisionCycle
// --------------------------------------------------------------------------

void
OranNtnXappMultiConn::DecisionCycle()
{
    NS_LOG_FUNCTION(this);

    if (!m_satBridge)
    {
        NS_LOG_WARN("MultiConn: no SatBridge configured, skipping cycle");
        return;
    }

    for (auto& kv : m_ueStates)
    {
        uint32_t ueId = kv.first;
        auto& state = kv.second;

        // Skip stale entries (no recent KPM update)
        double age = Simulator::Now().GetSeconds() - state.lastEvaluationTime;
        if (age > 10.0)
        {
            continue;
        }

        if (!state.dcActive)
        {
            // Evaluate DC activation
            if (ShouldActivateDc(ueId))
            {
                // Compute optimal split ratio before activation
                double splitRatio = ComputeOptimalSplitRatio(ueId);
                uint8_t primaryPath = SelectPrimaryPath(ueId);

                // Issue DC_SETUP action
                E2RcAction action = BuildAction(E2RcActionType::DC_SETUP,
                                                0, // target gnb
                                                ueId,
                                                0.85);
                action.parameter1 = splitRatio;
                action.parameter2 = static_cast<double>(primaryPath);

                bool accepted = SubmitAction(action);
                if (accepted)
                {
                    state.dcActive = true;
                    state.currentSplitRatio = splitRatio;
                    m_mcMetrics.dcActivations++;

                    NS_LOG_INFO("MultiConn: DC activated for UE" << ueId
                                                                  << " split=" << splitRatio
                                                                  << " primary=" << (int)primaryPath);
                }
            }
        }
        else
        {
            // DC is active -- check for deactivation or split adjustment
            if (ShouldDeactivateDc(ueId))
            {
                E2RcAction action = BuildAction(E2RcActionType::DC_TEARDOWN,
                                                0,
                                                ueId,
                                                0.80);
                action.parameter1 = state.tnSinr_dB;
                action.parameter2 = state.ntnSinr_dB;

                bool accepted = SubmitAction(action);
                if (accepted)
                {
                    state.dcActive = false;
                    m_mcMetrics.dcDeactivations++;

                    NS_LOG_INFO("MultiConn: DC deactivated for UE" << ueId
                                                                    << " tnSINR="
                                                                    << state.tnSinr_dB
                                                                    << " ntnSINR="
                                                                    << state.ntnSinr_dB);
                }
            }
            else
            {
                // Adjust split ratio dynamically
                double newSplit = ComputeOptimalSplitRatio(ueId);
                double splitDelta = std::abs(newSplit - state.currentSplitRatio);

                // Only issue adjustment if change is significant (> 5%)
                if (splitDelta > 0.05)
                {
                    E2RcAction action = BuildAction(E2RcActionType::BEARER_SPLIT,
                                                    0,
                                                    ueId,
                                                    0.75);
                    action.parameter1 = newSplit;
                    action.parameter2 = state.currentSplitRatio; // previous split

                    bool accepted = SubmitAction(action);
                    if (accepted)
                    {
                        state.currentSplitRatio = newSplit;
                        m_mcMetrics.splitRatioAdjustments++;

                        NS_LOG_DEBUG("MultiConn: split adjusted for UE"
                                     << ueId << " " << state.currentSplitRatio
                                     << " -> " << newSplit);
                    }
                }

                // Check primary path selection
                // SelectPrimaryPath is called but result only used for logging
                // Issue primary switch if needed (parameter2 encodes current primary)
                // This is tracked but doesn't need a separate action type
            }
        }

        // Predictive DC: check if upcoming HO warrants proactive DC setup
        if (m_predictiveDc && !state.dcActive)
        {
            if (PredictiveDcNeeded(ueId))
            {
                double splitRatio = ComputeOptimalSplitRatio(ueId);

                E2RcAction action = BuildAction(E2RcActionType::DC_SETUP,
                                                0,
                                                ueId,
                                                0.70); // lower confidence for predictive
                action.parameter1 = splitRatio;
                action.parameter2 = 1.0; // flag: predictive setup

                bool accepted = SubmitAction(action);
                if (accepted)
                {
                    state.dcActive = true;
                    state.currentSplitRatio = splitRatio;
                    m_mcMetrics.predictiveDcSetups++;

                    NS_LOG_INFO("MultiConn: PREDICTIVE DC setup for UE"
                                << ueId << " (satellite HO imminent)");
                }
            }
        }
    }

    RecordDecision(true, 0.85, 0.0);
}

// --------------------------------------------------------------------------
//  ShouldActivateDc -- evaluate DC benefit for a UE
// --------------------------------------------------------------------------

bool
OranNtnXappMultiConn::ShouldActivateDc(uint32_t ueId) const
{
    NS_LOG_FUNCTION(this << ueId);

    auto it = m_ueStates.find(ueId);
    if (it == m_ueStates.end())
    {
        return false;
    }

    const auto& state = it->second;

    // Condition 1: Both links must meet minimum SINR
    if (state.tnSinr_dB < m_minSinrForDc_dB || state.ntnSinr_dB < m_minSinrForDc_dB)
    {
        NS_LOG_DEBUG("MultiConn: UE" << ueId << " DC rejected: SINR too low (TN="
                                     << state.tnSinr_dB << " NTN=" << state.ntnSinr_dB
                                     << " min=" << m_minSinrForDc_dB << ")");
        return false;
    }

    // Condition 2: Not already in DC
    if (state.dcActive)
    {
        return false;
    }

    // Condition 3: DC benefit score must exceed threshold
    // Benefit = relative throughput gain from combining both links
    double tnSinrLinear = std::pow(10.0, state.tnSinr_dB / 10.0);
    double ntnSinrLinear = std::pow(10.0, state.ntnSinr_dB / 10.0);

    // Shannon capacity approximation (normalized)
    double tnCapacity = std::log2(1.0 + tnSinrLinear);
    double ntnCapacity = std::log2(1.0 + ntnSinrLinear);
    double combinedCapacity = tnCapacity + ntnCapacity;

    // Best single link
    double bestSingle = std::max(tnCapacity, ntnCapacity);

    // Benefit score: relative gain from DC
    double benefit = (bestSingle > 0.0)
                         ? (combinedCapacity - bestSingle) / bestSingle
                         : 0.0;

    if (benefit < m_dcBenefitThreshold)
    {
        NS_LOG_DEBUG("MultiConn: UE" << ueId << " DC benefit too low: " << benefit
                                     << " < " << m_dcBenefitThreshold);
        return false;
    }

    NS_LOG_DEBUG("MultiConn: UE" << ueId << " DC approved: benefit=" << benefit
                                 << " tnCap=" << tnCapacity << " ntnCap=" << ntnCapacity);
    return true;
}

// --------------------------------------------------------------------------
//  ShouldDeactivateDc -- check if DC should be torn down
// --------------------------------------------------------------------------

bool
OranNtnXappMultiConn::ShouldDeactivateDc(uint32_t ueId) const
{
    NS_LOG_FUNCTION(this << ueId);

    auto it = m_ueStates.find(ueId);
    if (it == m_ueStates.end())
    {
        return true; // unknown UE, deactivate
    }

    const auto& state = it->second;

    if (!state.dcActive)
    {
        return false; // not active, nothing to deactivate
    }

    // Condition 1: Either link drops below minSinr - 3 dB hysteresis margin
    double deactivationThreshold = m_minSinrForDc_dB - 3.0;

    if (state.tnSinr_dB < deactivationThreshold ||
        state.ntnSinr_dB < deactivationThreshold)
    {
        NS_LOG_DEBUG("MultiConn: UE" << ueId << " DC deactivation: link too weak (TN="
                                     << state.tnSinr_dB << " NTN=" << state.ntnSinr_dB
                                     << " thresh=" << deactivationThreshold << ")");
        return true;
    }

    // Condition 2: DC benefit has dropped below zero (costs more than it helps)
    double tnSinrLinear = std::pow(10.0, state.tnSinr_dB / 10.0);
    double ntnSinrLinear = std::pow(10.0, state.ntnSinr_dB / 10.0);

    double tnCapacity = std::log2(1.0 + tnSinrLinear);
    double ntnCapacity = std::log2(1.0 + ntnSinrLinear);
    double combinedCapacity = tnCapacity + ntnCapacity;
    double bestSingle = std::max(tnCapacity, ntnCapacity);

    double benefit = (bestSingle > 0.0)
                         ? (combinedCapacity - bestSingle) / bestSingle
                         : 0.0;

    if (benefit < 0.0)
    {
        NS_LOG_DEBUG("MultiConn: UE" << ueId << " DC deactivation: negative benefit="
                                     << benefit);
        return true;
    }

    return false;
}

// --------------------------------------------------------------------------
//  ComputeOptimalSplitRatio -- capacity-proportional split
// --------------------------------------------------------------------------

double
OranNtnXappMultiConn::ComputeOptimalSplitRatio(uint32_t ueId) const
{
    NS_LOG_FUNCTION(this << ueId);

    auto it = m_ueStates.find(ueId);
    if (it == m_ueStates.end())
    {
        return 0.5; // default 50/50 split
    }

    const auto& state = it->second;

    // Convert SINR from dB to linear
    double tnSinrLinear = std::pow(10.0, state.tnSinr_dB / 10.0);
    double ntnSinrLinear = std::pow(10.0, state.ntnSinr_dB / 10.0);

    // Shannon capacity: C = BW * log2(1 + SINR)
    // Assume representative bandwidths: TN = 100 MHz, NTN = 400 MHz (Ka-band)
    double tnBw_MHz = 100.0;
    double ntnBw_MHz = 400.0;

    double tnCapacity = tnBw_MHz * std::log2(1.0 + tnSinrLinear);
    double ntnCapacity = ntnBw_MHz * std::log2(1.0 + ntnSinrLinear);

    double totalCapacity = tnCapacity + ntnCapacity;

    if (totalCapacity < 1e-6)
    {
        return 0.5;
    }

    // Split ratio = fraction of traffic via TN
    // ratio = tnCapacity / (tnCapacity + ntnCapacity)
    double ratio = tnCapacity / totalCapacity;

    // Clamp to [0.05, 0.95] to always keep some traffic on each path
    ratio = std::max(0.05, std::min(0.95, ratio));

    NS_LOG_DEBUG("MultiConn: UE" << ueId << " optimal split=" << ratio
                                 << " tnCap=" << tnCapacity << " ntnCap=" << ntnCapacity);

    return ratio;
}

// --------------------------------------------------------------------------
//  PredictiveDcNeeded -- proactive DC before satellite HO
// --------------------------------------------------------------------------

bool
OranNtnXappMultiConn::PredictiveDcNeeded(uint32_t ueId) const
{
    NS_LOG_FUNCTION(this << ueId);

    if (!m_satBridge)
    {
        return false;
    }

    // Get the UE's current serving satellite TTE
    const auto& ueState = m_satBridge->GetUeState(ueId);
    double tte_s = ueState.servingTte_s;

    // If TTE < 30 seconds, satellite handover is imminent
    double predictiveDcThreshold_s = 30.0;
    if (tte_s > predictiveDcThreshold_s || tte_s <= 0.0)
    {
        return false;
    }

    // Check if there's a visible TN gNB to set up DC with
    // Use the sat bridge to check for visible satellites; for TN, we check
    // if the UE has a valid terrestrial SINR in its KPM data
    auto ueIt = m_ueStates.find(ueId);
    if (ueIt == m_ueStates.end())
    {
        return false;
    }

    const auto& state = ueIt->second;

    // TN gNB is available if we have a valid (non-negligible) TN SINR
    bool tnAvailable = (state.tnSinr_dB > m_minSinrForDc_dB);

    if (!tnAvailable)
    {
        NS_LOG_DEBUG("MultiConn: UE" << ueId << " predictive DC not feasible: no TN gNB "
                                     << "(tnSINR=" << state.tnSinr_dB << ")");
        return false;
    }

    NS_LOG_INFO("MultiConn: UE" << ueId << " predictive DC needed: TTE=" << tte_s
                                << "s tnSINR=" << state.tnSinr_dB << "dB");
    return true;
}

// --------------------------------------------------------------------------
//  SelectPrimaryPath -- QoS-aware primary link selection
// --------------------------------------------------------------------------

uint8_t
OranNtnXappMultiConn::SelectPrimaryPath(uint32_t ueId) const
{
    NS_LOG_FUNCTION(this << ueId);

    // Check slice type from recent KPM data
    uint8_t sliceId = 0; // default eMBB

    // Search across all gNBs for this UE's slice info
    for (const auto& gkv : m_kpmDatabase)
    {
        for (auto rit = gkv.second.rbegin(); rit != gkv.second.rend(); ++rit)
        {
            if (rit->ueId == ueId)
            {
                sliceId = rit->sliceId;
                break;
            }
        }
        if (sliceId != 0)
        {
            break;
        }
    }

    auto ueIt = m_ueStates.find(ueId);
    if (ueIt == m_ueStates.end())
    {
        return 0; // default TN
    }

    const auto& state = ueIt->second;

    // URLLC slice (sliceId=1): prefer TN for lower latency
    if (sliceId == 1)
    {
        NS_LOG_DEBUG("MultiConn: UE" << ueId << " URLLC -> primary=TN");
        return 0; // TN primary
    }

    // eMBB slice (sliceId=0): prefer whichever has higher throughput
    if (sliceId == 0)
    {
        if (state.tnThroughput > state.ntnThroughput)
        {
            NS_LOG_DEBUG("MultiConn: UE" << ueId << " eMBB -> primary=TN (higher tput)");
            return 0;
        }
        else
        {
            NS_LOG_DEBUG("MultiConn: UE" << ueId << " eMBB -> primary=NTN (higher tput)");
            return 1;
        }
    }

    // mMTC slice (sliceId=2): prefer NTN for wider coverage
    if (sliceId == 2)
    {
        NS_LOG_DEBUG("MultiConn: UE" << ueId << " mMTC -> primary=NTN");
        return 1;
    }

    // Default: prefer TN
    return 0;
}

} // namespace ns3
