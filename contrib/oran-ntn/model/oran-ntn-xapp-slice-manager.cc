/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-only
 * Author: Muhammad Uzair
 *
 * Slice Manager xApp - Network slice resource allocation for NTN
 */

#include "oran-ntn-xapp-slice-manager.h"

#include "oran-ntn-near-rt-ric.h"

#include <ns3/log.h>
#include <ns3/simulator.h>
#include <ns3/uinteger.h>

#include <algorithm>
#include <cmath>
#include <numeric>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("OranNtnXappSliceManager");
NS_OBJECT_ENSURE_REGISTERED(OranNtnXappSliceManager);

TypeId
OranNtnXappSliceManager::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::OranNtnXappSliceManager")
            .SetParent<OranNtnXappBase>()
            .SetGroupName("OranNtn")
            .AddConstructor<OranNtnXappSliceManager>()
            .AddAttribute("TotalPrbs",
                          "Total number of PRBs available at each gNB",
                          UintegerValue(273),
                          MakeUintegerAccessor(&OranNtnXappSliceManager::m_totalPrbs),
                          MakeUintegerChecker<uint32_t>(1))
            .AddAttribute("ReallocationInterval",
                          "Minimum interval between slice reallocations",
                          TimeValue(Seconds(1)),
                          MakeTimeAccessor(&OranNtnXappSliceManager::m_reallocationInterval),
                          MakeTimeChecker())
            .AddTraceSource("SliceAllocChanged",
                            "PRB allocation changed for a slice at a gNB",
                            MakeTraceSourceAccessor(
                                &OranNtnXappSliceManager::m_sliceAllocChanged),
                            "ns3::OranNtnXappSliceManager::SliceAllocTracedCallback");
    return tid;
}

OranNtnXappSliceManager::OranNtnXappSliceManager()
    : m_totalPrbs(273),
      m_reallocationInterval(Seconds(1))
{
    NS_LOG_FUNCTION(this);

    SetXappName("SliceManager");

    // Default slice configurations: eMBB, URLLC, mMTC
    SliceConfig embb;
    embb.sliceId = 0;
    embb.name = "eMBB";
    embb.minThroughput_Mbps = 50.0;
    embb.maxLatency_ms = 100.0;
    embb.reliabilityTarget = 0.999;
    embb.prbShare = 0.50;
    embb.harqEnabled = true;
    embb.priority = 2;

    SliceConfig urllc;
    urllc.sliceId = 1;
    urllc.name = "URLLC";
    urllc.minThroughput_Mbps = 1.0;
    urllc.maxLatency_ms = 1.0;
    urllc.reliabilityTarget = 0.99999;
    urllc.prbShare = 0.30;
    urllc.harqEnabled = true;
    urllc.priority = 0; // highest

    SliceConfig mmtc;
    mmtc.sliceId = 2;
    mmtc.name = "mMTC";
    mmtc.minThroughput_Mbps = 0.1;
    mmtc.maxLatency_ms = 10000.0;
    mmtc.reliabilityTarget = 0.99;
    mmtc.prbShare = 0.20;
    mmtc.harqEnabled = false;
    mmtc.priority = 3;

    m_sliceConfigs.push_back(embb);
    m_sliceConfigs.push_back(urllc);
    m_sliceConfigs.push_back(mmtc);

    // Initialize metrics
    m_sliceMetrics = {};
    m_sliceMetrics.reallocationCount = 0;
    m_sliceMetrics.overallSlaCompliance = 1.0;
}

OranNtnXappSliceManager::~OranNtnXappSliceManager()
{
    NS_LOG_FUNCTION(this);
}

// ============================================================================
//  Configuration
// ============================================================================

void
OranNtnXappSliceManager::SetSliceConfigs(const std::vector<SliceConfig>& configs)
{
    NS_LOG_FUNCTION(this << configs.size());
    m_sliceConfigs = configs;

    // Validate that PRB shares sum to 1.0
    double total = 0.0;
    for (const auto& cfg : m_sliceConfigs)
    {
        total += cfg.prbShare;
    }
    if (std::abs(total - 1.0) > 0.01)
    {
        NS_LOG_WARN("Slice PRB shares sum to " << total << ", expected 1.0");
    }
}

void
OranNtnXappSliceManager::SetTotalPrbs(uint32_t prbs)
{
    NS_LOG_FUNCTION(this << prbs);
    m_totalPrbs = prbs;
}

void
OranNtnXappSliceManager::SetReallocationInterval(Time interval)
{
    NS_LOG_FUNCTION(this << interval.As(Time::MS));
    m_reallocationInterval = interval;
}

// ============================================================================
//  Query
// ============================================================================

std::map<uint8_t, double>
OranNtnXappSliceManager::GetSliceAllocations(uint32_t gnbId) const
{
    auto it = m_gnbSliceStates.find(gnbId);
    if (it != m_gnbSliceStates.end())
    {
        return it->second.currentPrbShares;
    }

    // Return default allocations if gNB not yet tracked
    std::map<uint8_t, double> defaults;
    for (const auto& cfg : m_sliceConfigs)
    {
        defaults[cfg.sliceId] = cfg.prbShare;
    }
    return defaults;
}

// ============================================================================
//  Metrics
// ============================================================================

OranNtnXappSliceManager::SliceMetrics
OranNtnXappSliceManager::GetSliceMetrics() const
{
    return m_sliceMetrics;
}

// ============================================================================
//  E2 KPM Processing
// ============================================================================

void
OranNtnXappSliceManager::ProcessKpmReport(const E2KpmReport& report)
{
    NS_LOG_FUNCTION(this << report.gnbId << (uint32_t)report.sliceId);

    uint32_t gnbId = report.gnbId;
    uint8_t sliceId = report.sliceId;

    auto& state = m_gnbSliceStates[gnbId];

    // Initialize default PRB shares if this gNB is seen for the first time
    if (state.currentPrbShares.empty())
    {
        for (const auto& cfg : m_sliceConfigs)
        {
            state.currentPrbShares[cfg.sliceId] = cfg.prbShare;
            state.demandEstimate[cfg.sliceId] = 0.0;
            state.latestThroughput[cfg.sliceId] = 0.0;
            state.latestLatency[cfg.sliceId] = 0.0;
            state.latestReliability[cfg.sliceId] = 1.0;
        }
    }

    // Update per-slice metrics from report
    state.latestThroughput[sliceId] = report.sliceThroughput_Mbps;
    state.latestLatency[sliceId] = report.sliceLatency_ms;
    state.latestReliability[sliceId] = report.sliceReliability;

    // Estimate demand based on PRB utilization and throughput
    // Higher utilization with lower throughput indicates congestion (high demand)
    double utilization = report.prbUtilization;
    state.demandEstimate[sliceId] = utilization;

    // Update running average metrics
    double n = static_cast<double>(m_sliceMetrics.reallocationCount + 1);
    m_sliceMetrics.avgThroughput[sliceId] =
        m_sliceMetrics.avgThroughput[sliceId] * ((n - 1.0) / n) +
        report.sliceThroughput_Mbps / n;
    m_sliceMetrics.avgLatency[sliceId] =
        m_sliceMetrics.avgLatency[sliceId] * ((n - 1.0) / n) +
        report.sliceLatency_ms / n;

    NS_LOG_DEBUG("gNB " << gnbId << " slice " << (uint32_t)sliceId
                 << ": tput=" << report.sliceThroughput_Mbps
                 << " Mbps, lat=" << report.sliceLatency_ms
                 << " ms, rel=" << report.sliceReliability
                 << ", util=" << utilization);
}

// ============================================================================
//  Decision Cycle
// ============================================================================

void
OranNtnXappSliceManager::DecisionCycle()
{
    NS_LOG_FUNCTION(this);

    for (auto& [gnbId, state] : m_gnbSliceStates)
    {
        bool anyViolation = false;

        for (const auto& cfg : m_sliceConfigs)
        {
            if (!CheckSlaCompliance(cfg.sliceId, state))
            {
                anyViolation = true;
                NS_LOG_INFO("SLA violation detected at gNB " << gnbId
                            << " for slice " << cfg.name
                            << " (id=" << (uint32_t)cfg.sliceId << ")");
                break;
            }
        }

        if (anyViolation)
        {
            ReallocateSlices(gnbId);
        }
    }

    // Update overall SLA compliance metric
    uint32_t totalSliceChecks = 0;
    uint32_t compliantChecks = 0;
    for (const auto& [gnbId, state] : m_gnbSliceStates)
    {
        for (const auto& cfg : m_sliceConfigs)
        {
            totalSliceChecks++;
            if (CheckSlaCompliance(cfg.sliceId, state))
            {
                compliantChecks++;
            }
        }
    }

    if (totalSliceChecks > 0)
    {
        m_sliceMetrics.overallSlaCompliance =
            static_cast<double>(compliantChecks) /
            static_cast<double>(totalSliceChecks);
    }
}

// ============================================================================
//  SLA Compliance Check
// ============================================================================

bool
OranNtnXappSliceManager::CheckSlaCompliance(uint8_t sliceId,
                                              const PerGnbSliceState& state) const
{
    // Find the slice config
    const SliceConfig* cfg = nullptr;
    for (const auto& c : m_sliceConfigs)
    {
        if (c.sliceId == sliceId)
        {
            cfg = &c;
            break;
        }
    }

    if (!cfg)
    {
        NS_LOG_WARN("No config found for slice " << (uint32_t)sliceId);
        return true; // Unknown slice, assume compliant
    }

    // Check throughput
    auto tputIt = state.latestThroughput.find(sliceId);
    if (tputIt != state.latestThroughput.end())
    {
        if (tputIt->second < cfg->minThroughput_Mbps && tputIt->second > 0.0)
        {
            NS_LOG_DEBUG("Slice " << cfg->name
                         << " throughput " << tputIt->second
                         << " < min " << cfg->minThroughput_Mbps);
            return false;
        }
    }

    // Check latency
    auto latIt = state.latestLatency.find(sliceId);
    if (latIt != state.latestLatency.end())
    {
        if (latIt->second > cfg->maxLatency_ms && latIt->second > 0.0)
        {
            NS_LOG_DEBUG("Slice " << cfg->name
                         << " latency " << latIt->second
                         << " > max " << cfg->maxLatency_ms);
            return false;
        }
    }

    // Check reliability
    auto relIt = state.latestReliability.find(sliceId);
    if (relIt != state.latestReliability.end())
    {
        if (relIt->second < cfg->reliabilityTarget && relIt->second > 0.0)
        {
            NS_LOG_DEBUG("Slice " << cfg->name
                         << " reliability " << relIt->second
                         << " < target " << cfg->reliabilityTarget);
            return false;
        }
    }

    return true;
}

// ============================================================================
//  Slice Reallocation (Proportional Fair)
// ============================================================================

void
OranNtnXappSliceManager::ReallocateSlices(uint32_t gnbId)
{
    NS_LOG_FUNCTION(this << gnbId);

    auto stateIt = m_gnbSliceStates.find(gnbId);
    if (stateIt == m_gnbSliceStates.end())
    {
        return;
    }
    auto& state = stateIt->second;

    // Step 1: Identify violating and over-provisioned slices
    std::vector<uint8_t> violatingSlices;
    std::vector<uint8_t> overProvisionedSlices;

    for (const auto& cfg : m_sliceConfigs)
    {
        if (!CheckSlaCompliance(cfg.sliceId, state))
        {
            violatingSlices.push_back(cfg.sliceId);
        }
        else
        {
            // Check if this slice is over-provisioned (demand < allocation)
            auto demandIt = state.demandEstimate.find(cfg.sliceId);
            auto shareIt = state.currentPrbShares.find(cfg.sliceId);
            if (demandIt != state.demandEstimate.end() &&
                shareIt != state.currentPrbShares.end())
            {
                // If utilization is well below allocated share, it's over-provisioned
                if (demandIt->second < shareIt->second * 0.7)
                {
                    overProvisionedSlices.push_back(cfg.sliceId);
                }
            }
        }
    }

    if (violatingSlices.empty())
    {
        return;
    }

    // Step 2: Calculate how much we can reclaim from over-provisioned slices
    double reclaimable = 0.0;
    std::map<uint8_t, double> reclaimPerSlice;

    for (uint8_t sid : overProvisionedSlices)
    {
        // Find minimum guaranteed share for this slice
        double minShare = 0.0;
        for (const auto& cfg : m_sliceConfigs)
        {
            if (cfg.sliceId == sid)
            {
                minShare = cfg.prbShare;
                break;
            }
        }

        double currentShare = state.currentPrbShares[sid];
        double demand = state.demandEstimate[sid];

        // Can reclaim down to max(minShare, demand) with a safety margin
        double floor = std::max(minShare * 0.8, demand);
        double canReclaim = std::max(0.0, currentShare - floor);

        // Limit reallocation step size to avoid oscillation
        canReclaim = std::min(canReclaim, 0.10); // max 10% PRB shift per cycle

        reclaimPerSlice[sid] = canReclaim;
        reclaimable += canReclaim;
    }

    if (reclaimable < 0.01)
    {
        NS_LOG_DEBUG("Insufficient reclaimable PRBs for gNB " << gnbId);
        return;
    }

    // Step 3: Distribute reclaimed PRBs proportionally to violating slices
    //         weighted by their priority (lower priority number = higher urgency)
    double totalWeight = 0.0;
    std::map<uint8_t, double> weights;
    for (uint8_t sid : violatingSlices)
    {
        for (const auto& cfg : m_sliceConfigs)
        {
            if (cfg.sliceId == sid)
            {
                // Inverse priority: priority 0 (highest) gets weight 4,
                // priority 3 (lowest) gets weight 1
                double w = 4.0 / (1.0 + cfg.priority);
                weights[sid] = w;
                totalWeight += w;
                break;
            }
        }
    }

    // Step 4: Apply reallocation
    // Decrease over-provisioned
    for (uint8_t sid : overProvisionedSlices)
    {
        double oldShare = state.currentPrbShares[sid];
        double newShare = oldShare - reclaimPerSlice[sid];
        state.currentPrbShares[sid] = newShare;

        NS_LOG_INFO("gNB " << gnbId << " slice " << (uint32_t)sid
                    << ": PRB share " << oldShare << " -> " << newShare
                    << " (reclaimed " << reclaimPerSlice[sid] << ")");
    }

    // Increase violating slices
    for (uint8_t sid : violatingSlices)
    {
        double oldShare = state.currentPrbShares[sid];
        double boost = (totalWeight > 0.0) ? reclaimable * (weights[sid] / totalWeight)
                                            : reclaimable / violatingSlices.size();
        double newShare = oldShare + boost;

        // Cap at reasonable maximum (no single slice gets more than 80%)
        newShare = std::min(newShare, 0.80);
        state.currentPrbShares[sid] = newShare;

        NS_LOG_INFO("gNB " << gnbId << " slice " << (uint32_t)sid
                    << ": PRB share " << oldShare << " -> " << newShare
                    << " (boosted " << boost << ")");
    }

    // Step 5: Normalize to ensure shares sum to 1.0
    double totalShares = 0.0;
    for (const auto& [sid, share] : state.currentPrbShares)
    {
        totalShares += share;
    }
    if (totalShares > 0.0 && std::abs(totalShares - 1.0) > 0.001)
    {
        for (auto& [sid, share] : state.currentPrbShares)
        {
            share /= totalShares;
        }
    }

    // Step 6: Enforce minimum guaranteed shares
    for (const auto& cfg : m_sliceConfigs)
    {
        double minGuaranteed = cfg.prbShare * 0.5; // at least 50% of configured share
        auto& share = state.currentPrbShares[cfg.sliceId];
        if (share < minGuaranteed)
        {
            share = minGuaranteed;
        }
    }

    // Re-normalize after enforcement
    totalShares = 0.0;
    for (const auto& [sid, share] : state.currentPrbShares)
    {
        totalShares += share;
    }
    if (totalShares > 0.0 && std::abs(totalShares - 1.0) > 0.001)
    {
        for (auto& [sid, share] : state.currentPrbShares)
        {
            share /= totalShares;
        }
    }

    // Step 7: Submit RC actions for each slice
    for (const auto& [sliceId, prbShare] : state.currentPrbShares)
    {
        E2RcAction action = BuildAction(E2RcActionType::SLICE_PRB_ALLOCATION,
                                         gnbId,
                                         0, // cell-wide
                                         0.9);
        action.targetSliceId = sliceId;
        action.parameter1 = prbShare;
        action.parameter2 = static_cast<double>(m_totalPrbs);

        bool accepted = SubmitAction(action);

        NS_LOG_INFO("gNB " << gnbId << " slice " << (uint32_t)sliceId
                    << ": SLICE_PRB_ALLOCATION action "
                    << (accepted ? "accepted" : "rejected")
                    << ", share=" << prbShare
                    << " (" << static_cast<uint32_t>(prbShare * m_totalPrbs) << " PRBs)");

        m_sliceAllocChanged(gnbId, sliceId, prbShare);
    }

    // Update metrics
    m_sliceMetrics.reallocationCount++;
    for (const auto& [sid, share] : state.currentPrbShares)
    {
        m_sliceMetrics.avgPrbShare[sid] = share;
    }

    // Track SLA violation rates per slice
    for (const auto& cfg : m_sliceConfigs)
    {
        bool compliant = CheckSlaCompliance(cfg.sliceId, state);
        double n = static_cast<double>(m_sliceMetrics.reallocationCount);
        double& rate = m_sliceMetrics.slaViolationRate[cfg.sliceId];
        double violation = compliant ? 0.0 : 1.0;
        rate = rate * ((n - 1.0) / n) + violation / n;
    }

    RecordDecision(true, 0.9, 0.0);
}

// ============================================================================
//  E2 Subscription
// ============================================================================

E2Subscription
OranNtnXappSliceManager::GetRequiredSubscription() const
{
    E2Subscription sub;
    sub.subscriptionId = 0; // Assigned by E2 termination
    sub.ricRequestorId = GetXappId();
    sub.ranFunctionId = 2;  // KPM service model
    sub.reportingPeriod = MilliSeconds(500);
    sub.eventTrigger = false; // Periodic
    sub.eventThreshold = 0.0;
    sub.batchOnVisibility = false;
    sub.maxBufferAge = Seconds(5);
    sub.useIslRelay = true;
    return sub;
}

} // namespace ns3
