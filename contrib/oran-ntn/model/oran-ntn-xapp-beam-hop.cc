/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-only
 * Author: Muhammad Uzair
 *
 * Beam Hopping xApp implementation - Dynamic beam scheduling for NTN satellites
 */

#include "oran-ntn-xapp-beam-hop.h"

#include "oran-ntn-near-rt-ric.h"

#include <ns3/enum.h>
#include <ns3/log.h>
#include <ns3/simulator.h>
#include <ns3/string.h>
#include <ns3/uinteger.h>

#include <algorithm>
#include <cmath>
#include <numeric>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("OranNtnXappBeamHop");
NS_OBJECT_ENSURE_REGISTERED(OranNtnXappBeamHop);

TypeId
OranNtnXappBeamHop::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::OranNtnXappBeamHop")
            .SetParent<OranNtnXappBase>()
            .SetGroupName("OranNtn")
            .AddConstructor<OranNtnXappBeamHop>()
            .AddAttribute("NumBeams",
                          "Number of beams per satellite",
                          UintegerValue(8),
                          MakeUintegerAccessor(&OranNtnXappBeamHop::m_numBeams),
                          MakeUintegerChecker<uint32_t>(1, 256))
            .AddAttribute("NumTimeSlots",
                          "Number of time slots per hopping frame",
                          UintegerValue(16),
                          MakeUintegerAccessor(&OranNtnXappBeamHop::m_numTimeSlots),
                          MakeUintegerChecker<uint32_t>(1, 1024))
            .AddAttribute("HoppingFrameLength",
                          "Duration of one beam hopping frame",
                          TimeValue(MilliSeconds(10)),
                          MakeTimeAccessor(&OranNtnXappBeamHop::m_hoppingFrameLength),
                          MakeTimeChecker())
            .AddAttribute("MaxSimultaneousBeams",
                          "Maximum number of beams active simultaneously",
                          UintegerValue(4),
                          MakeUintegerAccessor(&OranNtnXappBeamHop::m_maxSimultaneousBeams),
                          MakeUintegerChecker<uint32_t>(1, 256))
            .AddAttribute("SchedulingAlgorithm",
                          "Beam scheduling algorithm: water-fill, proportional, or drl",
                          StringValue("water-fill"),
                          MakeStringAccessor(&OranNtnXappBeamHop::m_schedulingAlgorithm),
                          MakeStringChecker())
            .AddTraceSource("ScheduleUpdated",
                            "A new beam allocation schedule was computed",
                            MakeTraceSourceAccessor(&OranNtnXappBeamHop::m_scheduleUpdated),
                            "ns3::OranNtnXappBeamHop::ScheduleUpdatedTracedCallback");
    return tid;
}

OranNtnXappBeamHop::OranNtnXappBeamHop()
    : m_numBeams(8),
      m_numTimeSlots(16),
      m_hoppingFrameLength(MilliSeconds(10)),
      m_maxSimultaneousBeams(4),
      m_schedulingAlgorithm("water-fill")
{
    NS_LOG_FUNCTION(this);

    m_bhMetrics = {};

    // Initialize DRL weights with small random-like values (deterministic seed)
    // 4 features per beam: trafficDemand, currentLoad, activeUes(normalized), avgSinr(normalized)
    m_drlWeights = {0.4, 0.3, 0.15, 0.15};
}

OranNtnXappBeamHop::~OranNtnXappBeamHop()
{
    NS_LOG_FUNCTION(this);
}

// ============================================================================
//  Configuration
// ============================================================================

void
OranNtnXappBeamHop::SetNumBeams(uint32_t beams)
{
    m_numBeams = beams;
}

void
OranNtnXappBeamHop::SetNumTimeSlots(uint32_t slots)
{
    m_numTimeSlots = slots;
}

void
OranNtnXappBeamHop::SetHoppingFrameLength(Time frameLen)
{
    m_hoppingFrameLength = frameLen;
}

void
OranNtnXappBeamHop::SetMaxSimultaneousBeams(uint32_t maxBeams)
{
    m_maxSimultaneousBeams = maxBeams;
}

void
OranNtnXappBeamHop::SetSchedulingAlgorithm(const std::string& algo)
{
    NS_ASSERT_MSG(algo == "water-fill" || algo == "proportional" || algo == "drl",
                  "Unknown scheduling algorithm: " << algo);
    m_schedulingAlgorithm = algo;
}

// ============================================================================
//  Output & Metrics
// ============================================================================

std::vector<BeamAllocationEntry>
OranNtnXappBeamHop::GetCurrentSchedule() const
{
    return m_currentSchedule;
}

OranNtnXappBeamHop::BeamHopMetrics
OranNtnXappBeamHop::GetBeamHopMetrics() const
{
    return m_bhMetrics;
}

// ============================================================================
//  E2 KPM Processing
// ============================================================================

void
OranNtnXappBeamHop::ProcessKpmReport(const E2KpmReport& report)
{
    NS_LOG_FUNCTION(this << report.gnbId << report.beamId);

    uint32_t beamId = report.beamId;

    auto it = m_beamStates.find(beamId);
    if (it == m_beamStates.end())
    {
        // First report for this beam - initialize state
        BeamState state;
        state.beamId = beamId;
        state.trafficDemand = report.prbUtilization;
        state.currentLoad = report.prbUtilization;
        state.activeUes = report.activeUes;
        state.avgSinr = report.sinr_dB;
        state.interferenceLevel = 0.0;
        state.allocatedSlots = 0;
        m_beamStates[beamId] = state;
    }
    else
    {
        // Update with exponential moving average for smooth tracking
        constexpr double alpha = 0.3;
        BeamState& state = it->second;

        state.trafficDemand = alpha * report.prbUtilization +
                              (1.0 - alpha) * state.trafficDemand;
        state.currentLoad = report.prbUtilization;
        state.activeUes = report.activeUes;
        state.avgSinr = alpha * report.sinr_dB +
                        (1.0 - alpha) * state.avgSinr;
    }

    NS_LOG_DEBUG("Beam " << beamId
                 << " demand=" << m_beamStates[beamId].trafficDemand
                 << " load=" << m_beamStates[beamId].currentLoad
                 << " UEs=" << m_beamStates[beamId].activeUes
                 << " SINR=" << m_beamStates[beamId].avgSinr << " dB");
}

// ============================================================================
//  Decision Cycle
// ============================================================================

void
OranNtnXappBeamHop::DecisionCycle()
{
    NS_LOG_FUNCTION(this);

    if (m_beamStates.empty())
    {
        NS_LOG_DEBUG("BeamHop xApp: No beam state data yet, skipping cycle");
        return;
    }

    double startTime = Simulator::Now().GetSeconds();

    // Run the selected scheduling algorithm
    std::vector<BeamAllocationEntry> schedule;
    if (m_schedulingAlgorithm == "water-fill")
    {
        schedule = WaterFillingSchedule();
    }
    else if (m_schedulingAlgorithm == "proportional")
    {
        schedule = ProportionalSchedule();
    }
    else if (m_schedulingAlgorithm == "drl")
    {
        schedule = DrlSchedule();
    }
    else
    {
        NS_LOG_WARN("Unknown scheduling algorithm: " << m_schedulingAlgorithm
                     << ", falling back to water-fill");
        schedule = WaterFillingSchedule();
    }

    // Count beam switches from previous schedule
    if (!m_currentSchedule.empty())
    {
        uint32_t switches = 0;
        // Build a set of active beams per slot for old and new schedules
        for (const auto& entry : schedule)
        {
            bool wasActive = false;
            for (const auto& old : m_currentSchedule)
            {
                if (old.beamId == entry.beamId && old.timeSlot == entry.timeSlot)
                {
                    wasActive = true;
                    break;
                }
            }
            if (!wasActive)
            {
                switches++;
            }
        }
        m_bhMetrics.beamSwitches += switches;
    }

    // Update current schedule
    m_currentSchedule = schedule;
    m_bhMetrics.schedulingRounds++;

    // Compute utilization and demand satisfaction metrics
    if (!m_beamStates.empty())
    {
        // Beam utilization: fraction of total capacity allocated
        double totalAllocated = static_cast<double>(schedule.size());
        double totalCapacity = static_cast<double>(m_numBeams * m_numTimeSlots);
        m_bhMetrics.avgBeamUtilization = (totalCapacity > 0)
                                             ? totalAllocated / totalCapacity
                                             : 0.0;

        // Demand satisfaction: how well allocations match demands
        double totalDemand = 0.0;
        double satisfiedDemand = 0.0;
        for (const auto& [beamId, state] : m_beamStates)
        {
            totalDemand += state.trafficDemand;
            double beamSlots = 0;
            for (const auto& entry : schedule)
            {
                if (entry.beamId == beamId)
                {
                    beamSlots += 1.0;
                }
            }
            double served = std::min(state.trafficDemand,
                                     beamSlots / static_cast<double>(m_numTimeSlots));
            satisfiedDemand += served;
        }
        m_bhMetrics.avgDemandSatisfaction = (totalDemand > 0)
                                                ? satisfiedDemand / totalDemand
                                                : 1.0;

        // Fairness: compute Jain's index over per-beam allocation ratios
        std::vector<double> allocationRatios;
        for (const auto& [beamId, state] : m_beamStates)
        {
            double beamSlots = 0;
            for (const auto& entry : schedule)
            {
                if (entry.beamId == beamId)
                {
                    beamSlots += 1.0;
                }
            }
            allocationRatios.push_back(beamSlots);
        }
        m_bhMetrics.fairnessIndex = ComputeJainsFairness(allocationRatios);

        // Energy efficiency: approximate as ratio of utilized beams
        // (fewer simultaneous beams = more energy-efficient)
        if (totalCapacity > 0)
        {
            m_bhMetrics.avgEnergyEfficiency =
                1.0 - (totalAllocated / totalCapacity);
        }
    }

    // Submit beam hop schedule as an RC action to the RIC
    E2RcAction action = BuildAction(E2RcActionType::BEAM_HOP_SCHEDULE,
                                    0, // cell-wide (satellite level)
                                    0, // no specific UE
                                    1.0); // full confidence in schedule
    action.parameter1 = static_cast<double>(schedule.size());
    action.parameter2 = m_bhMetrics.fairnessIndex;

    bool accepted = SubmitAction(action);

    double latencyMs = (Simulator::Now().GetSeconds() - startTime) * 1000.0;
    RecordDecision(accepted, 1.0, latencyMs);

    // Fire trace
    m_scheduleUpdated(m_bhMetrics.schedulingRounds, m_currentSchedule);

    NS_LOG_INFO("BeamHop xApp: Round " << m_bhMetrics.schedulingRounds
                << " algo=" << m_schedulingAlgorithm
                << " entries=" << schedule.size()
                << " fairness=" << m_bhMetrics.fairnessIndex
                << " accepted=" << accepted);
}

// ============================================================================
//  Water-Filling Schedule
// ============================================================================

std::vector<BeamAllocationEntry>
OranNtnXappBeamHop::WaterFillingSchedule() const
{
    NS_LOG_FUNCTION(this);

    std::vector<BeamAllocationEntry> schedule;

    // Collect beams sorted by demand (descending)
    std::vector<std::pair<uint32_t, double>> beamDemands;
    for (const auto& [beamId, state] : m_beamStates)
    {
        beamDemands.emplace_back(beamId, state.trafficDemand);
    }

    std::sort(beamDemands.begin(), beamDemands.end(),
              [](const auto& a, const auto& b) { return a.second > b.second; });

    // Total available capacity: numTimeSlots * maxSimultaneousBeams
    uint32_t totalSlotCapacity = m_numTimeSlots * m_maxSimultaneousBeams;
    uint32_t slotsRemaining = totalSlotCapacity;

    // Track per-slot usage to enforce maxSimultaneousBeams constraint
    std::vector<uint32_t> beamsPerSlot(m_numTimeSlots, 0);

    // Water-filling: allocate slots to highest-demand beams first
    // Each beam gets slots proportional to its demand, filled greedily
    double totalDemand = 0.0;
    for (const auto& [beamId, demand] : beamDemands)
    {
        totalDemand += demand;
    }

    if (totalDemand <= 0.0)
    {
        // No demand; assign one slot per beam round-robin for fairness
        uint32_t slotIdx = 0;
        for (const auto& [beamId, demand] : beamDemands)
        {
            if (slotIdx >= m_numTimeSlots)
            {
                break;
            }
            if (beamsPerSlot[slotIdx] < m_maxSimultaneousBeams)
            {
                BeamAllocationEntry entry;
                entry.satId = 0;
                entry.beamId = beamId;
                entry.timeSlot = slotIdx;
                entry.allocatedCellId = beamId;
                entry.trafficLoad = 0.0;
                entry.isSignaling = false;
                schedule.push_back(entry);
                beamsPerSlot[slotIdx]++;
                slotIdx++;
            }
        }
        return schedule;
    }

    for (const auto& [beamId, demand] : beamDemands)
    {
        if (slotsRemaining == 0)
        {
            break;
        }

        // Slots proportional to demand, but at least 1 if beam has any demand
        uint32_t desiredSlots = static_cast<uint32_t>(
            std::ceil((demand / totalDemand) * static_cast<double>(totalSlotCapacity)));
        desiredSlots = std::max(desiredSlots, uint32_t(1));
        desiredSlots = std::min(desiredSlots, slotsRemaining);
        // Cap per beam at numTimeSlots
        desiredSlots = std::min(desiredSlots, m_numTimeSlots);

        uint32_t allocated = 0;
        for (uint32_t s = 0; s < m_numTimeSlots && allocated < desiredSlots; ++s)
        {
            if (beamsPerSlot[s] < m_maxSimultaneousBeams)
            {
                BeamAllocationEntry entry;
                entry.satId = 0;
                entry.beamId = beamId;
                entry.timeSlot = s;
                entry.allocatedCellId = beamId;
                entry.trafficLoad = demand;
                entry.isSignaling = false;
                schedule.push_back(entry);
                beamsPerSlot[s]++;
                allocated++;
                slotsRemaining--;
            }
        }
    }

    return schedule;
}

// ============================================================================
//  Proportional Schedule
// ============================================================================

std::vector<BeamAllocationEntry>
OranNtnXappBeamHop::ProportionalSchedule() const
{
    NS_LOG_FUNCTION(this);

    std::vector<BeamAllocationEntry> schedule;

    // Compute total demand across all beams
    double totalDemand = 0.0;
    for (const auto& [beamId, state] : m_beamStates)
    {
        totalDemand += state.trafficDemand;
    }

    uint32_t totalSlotCapacity = m_numTimeSlots * m_maxSimultaneousBeams;

    // Track per-slot usage
    std::vector<uint32_t> beamsPerSlot(m_numTimeSlots, 0);

    for (const auto& [beamId, state] : m_beamStates)
    {
        // Simple proportional: slots = demand/totalDemand * totalCapacity
        uint32_t desiredSlots;
        if (totalDemand > 0.0)
        {
            desiredSlots = static_cast<uint32_t>(
                std::round((state.trafficDemand / totalDemand) *
                           static_cast<double>(totalSlotCapacity)));
        }
        else
        {
            // Equal share when no demand info
            desiredSlots = totalSlotCapacity /
                           static_cast<uint32_t>(m_beamStates.size());
        }

        // Ensure at least 1 slot per beam with any demand
        if (state.trafficDemand > 0.0 && desiredSlots == 0)
        {
            desiredSlots = 1;
        }

        desiredSlots = std::min(desiredSlots, m_numTimeSlots);

        uint32_t allocated = 0;
        for (uint32_t s = 0; s < m_numTimeSlots && allocated < desiredSlots; ++s)
        {
            if (beamsPerSlot[s] < m_maxSimultaneousBeams)
            {
                BeamAllocationEntry entry;
                entry.satId = 0;
                entry.beamId = beamId;
                entry.timeSlot = s;
                entry.allocatedCellId = beamId;
                entry.trafficLoad = state.trafficDemand;
                entry.isSignaling = false;
                schedule.push_back(entry);
                beamsPerSlot[s]++;
                allocated++;
            }
        }
    }

    return schedule;
}

// ============================================================================
//  DRL-based Schedule (Simplified)
// ============================================================================

std::vector<BeamAllocationEntry>
OranNtnXappBeamHop::DrlSchedule() const
{
    NS_LOG_FUNCTION(this);

    std::vector<BeamAllocationEntry> schedule;

    // Build feature vector for each beam and compute action scores
    // Features: [trafficDemand, currentLoad, normalizedUes, normalizedSinr]
    struct BeamScore
    {
        uint32_t beamId;
        double score;
        double demand;
    };

    // Find max values for normalization
    double maxUes = 1.0;
    double maxSinr = 1.0;
    for (const auto& [beamId, state] : m_beamStates)
    {
        maxUes = std::max(maxUes, static_cast<double>(state.activeUes));
        maxSinr = std::max(maxSinr, std::abs(state.avgSinr));
    }

    std::vector<BeamScore> scores;
    for (const auto& [beamId, state] : m_beamStates)
    {
        // Feature vector
        double features[4] = {
            state.trafficDemand,
            state.currentLoad,
            static_cast<double>(state.activeUes) / maxUes,
            (state.avgSinr > 0) ? state.avgSinr / maxSinr : 0.0,
        };

        // Dot product with DRL weights
        double score = 0.0;
        for (uint32_t i = 0; i < 4 && i < m_drlWeights.size(); ++i)
        {
            score += features[i] * m_drlWeights[i];
        }

        scores.push_back({beamId, score, state.trafficDemand});
    }

    // Sort beams by score (descending) - higher score = more need
    std::sort(scores.begin(), scores.end(),
              [](const BeamScore& a, const BeamScore& b) {
                  return a.score > b.score;
              });

    // Allocate slots based on scores
    uint32_t totalSlotCapacity = m_numTimeSlots * m_maxSimultaneousBeams;
    std::vector<uint32_t> beamsPerSlot(m_numTimeSlots, 0);

    // Compute total score for proportional allocation
    double totalScore = 0.0;
    for (const auto& bs : scores)
    {
        totalScore += std::max(bs.score, 0.0);
    }

    for (const auto& bs : scores)
    {
        uint32_t desiredSlots;
        if (totalScore > 0.0)
        {
            double normalizedScore = std::max(bs.score, 0.0) / totalScore;
            desiredSlots = static_cast<uint32_t>(
                std::ceil(normalizedScore * static_cast<double>(totalSlotCapacity)));
        }
        else
        {
            desiredSlots = 1;
        }

        desiredSlots = std::max(desiredSlots, uint32_t(1));
        desiredSlots = std::min(desiredSlots, m_numTimeSlots);

        uint32_t allocated = 0;
        for (uint32_t s = 0; s < m_numTimeSlots && allocated < desiredSlots; ++s)
        {
            if (beamsPerSlot[s] < m_maxSimultaneousBeams)
            {
                BeamAllocationEntry entry;
                entry.satId = 0;
                entry.beamId = bs.beamId;
                entry.timeSlot = s;
                entry.allocatedCellId = bs.beamId;
                entry.trafficLoad = bs.demand;
                entry.isSignaling = false;
                schedule.push_back(entry);
                beamsPerSlot[s]++;
                allocated++;
            }
        }
    }

    return schedule;
}

// ============================================================================
//  Jain's Fairness Index
// ============================================================================

double
OranNtnXappBeamHop::ComputeJainsFairness(const std::vector<double>& allocations) const
{
    if (allocations.empty())
    {
        return 1.0;
    }

    double n = static_cast<double>(allocations.size());
    double sumX = 0.0;
    double sumX2 = 0.0;

    for (double x : allocations)
    {
        sumX += x;
        sumX2 += x * x;
    }

    if (sumX2 <= 0.0)
    {
        return 1.0; // All zeros is perfectly fair
    }

    // Jain's fairness index: (sum(x))^2 / (n * sum(x^2))
    return (sumX * sumX) / (n * sumX2);
}

// ============================================================================
//  E2 Subscription
// ============================================================================

E2Subscription
OranNtnXappBeamHop::GetRequiredSubscription() const
{
    E2Subscription sub;
    sub.subscriptionId = 0; // Assigned by E2 termination
    sub.ricRequestorId = GetXappId();
    sub.ranFunctionId = 2; // KPM service model
    sub.reportingPeriod = MilliSeconds(200);
    sub.eventTrigger = false; // Periodic reporting
    sub.eventThreshold = 0.0;
    sub.batchOnVisibility = false;
    sub.maxBufferAge = Seconds(5);
    sub.useIslRelay = false;
    return sub;
}

} // namespace ns3
