/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-only
 * Author: Muhammad Uzair
 */

#include "oran-ntn-xapp-tn-ntn-steering.h"

#include "oran-ntn-e2-interface.h"

#include <ns3/double.h>
#include <ns3/log.h>
#include <ns3/simulator.h>
#include <ns3/string.h>
#include <ns3/uinteger.h>

#include <algorithm>
#include <cmath>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("OranNtnXappTnNtnSteering");
NS_OBJECT_ENSURE_REGISTERED(OranNtnXappTnNtnSteering);

TypeId
OranNtnXappTnNtnSteering::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::OranNtnXappTnNtnSteering")
            .SetParent<OranNtnXappBase>()
            .SetGroupName("OranNtn")
            .AddConstructor<OranNtnXappTnNtnSteering>()
            .AddAttribute("SteeringMode",
                          "Traffic steering strategy: latency-optimal, "
                          "throughput-optimal, reliability-optimal, cost-optimal",
                          StringValue("latency-optimal"),
                          MakeStringAccessor(&OranNtnXappTnNtnSteering::m_steeringMode),
                          MakeStringChecker())
            .AddAttribute("TnPreferenceFactor",
                          "Preference factor for terrestrial network (0-1, higher = prefer TN)",
                          DoubleValue(0.6),
                          MakeDoubleAccessor(&OranNtnXappTnNtnSteering::m_tnPreferenceFactor),
                          MakeDoubleChecker<double>(0.0, 1.0))
            .AddAttribute("NtnLatencyPenalty",
                          "Additional latency penalty for NTN path in ms",
                          DoubleValue(20.0),
                          MakeDoubleAccessor(&OranNtnXappTnNtnSteering::m_ntnLatencyPenalty),
                          MakeDoubleChecker<double>(0.0))
            .AddAttribute("MinSinrTn",
                          "Minimum SINR threshold for terrestrial network in dB",
                          DoubleValue(-5.0),
                          MakeDoubleAccessor(&OranNtnXappTnNtnSteering::m_minSinrTn),
                          MakeDoubleChecker<double>())
            .AddAttribute("MinSinrNtn",
                          "Minimum SINR threshold for NTN network in dB",
                          DoubleValue(-3.0),
                          MakeDoubleAccessor(&OranNtnXappTnNtnSteering::m_minSinrNtn),
                          MakeDoubleChecker<double>())
            .AddTraceSource("SteeringDecision",
                            "A steering decision was made for a UE",
                            MakeTraceSourceAccessor(
                                &OranNtnXappTnNtnSteering::m_steeringDecision),
                            "ns3::OranNtnXappTnNtnSteering::SteeringTracedCallback");
    return tid;
}

OranNtnXappTnNtnSteering::OranNtnXappTnNtnSteering()
    : m_steeringMode("latency-optimal"),
      m_tnPreferenceFactor(0.6),
      m_ntnLatencyPenalty(20.0),
      m_minSinrTn(-5.0),
      m_minSinrNtn(-3.0)
{
    NS_LOG_FUNCTION(this);
    m_steeringMetrics = {};
}

OranNtnXappTnNtnSteering::~OranNtnXappTnNtnSteering()
{
    NS_LOG_FUNCTION(this);
}

// ---- Configuration ----

void
OranNtnXappTnNtnSteering::SetSteeringMode(const std::string& mode)
{
    NS_ASSERT_MSG(mode == "latency-optimal" || mode == "throughput-optimal" ||
                  mode == "reliability-optimal" || mode == "cost-optimal",
                  "Invalid steering mode: " << mode);
    m_steeringMode = mode;
}

void
OranNtnXappTnNtnSteering::SetTnPreferenceFactor(double factor)
{
    NS_ASSERT_MSG(factor >= 0.0 && factor <= 1.0,
                  "TN preference factor must be in [0, 1]");
    m_tnPreferenceFactor = factor;
}

void
OranNtnXappTnNtnSteering::SetNtnLatencyPenalty(double ms)
{
    m_ntnLatencyPenalty = ms;
}

void
OranNtnXappTnNtnSteering::SetMinSinrForTn(double dB)
{
    m_minSinrTn = dB;
}

void
OranNtnXappTnNtnSteering::SetMinSinrForNtn(double dB)
{
    m_minSinrNtn = dB;
}

// ---- Query ----

NetworkSelection
OranNtnXappTnNtnSteering::GetUeNetworkSelection(uint32_t ueId) const
{
    auto it = m_ueSteeringStates.find(ueId);
    if (it != m_ueSteeringStates.end())
    {
        return it->second.currentNetwork;
    }
    return NetworkSelection::TERRESTRIAL; // Default
}

double
OranNtnXappTnNtnSteering::GetTnTrafficFraction() const
{
    if (m_ueSteeringStates.empty())
    {
        return 1.0;
    }

    uint32_t tnCount = 0;
    for (const auto& [ueId, state] : m_ueSteeringStates)
    {
        if (state.currentNetwork == NetworkSelection::TERRESTRIAL ||
            state.currentNetwork == NetworkSelection::DUAL_CONNECTED)
        {
            tnCount++;
        }
    }
    return static_cast<double>(tnCount) /
           static_cast<double>(m_ueSteeringStates.size());
}

double
OranNtnXappTnNtnSteering::GetNtnTrafficFraction() const
{
    if (m_ueSteeringStates.empty())
    {
        return 0.0;
    }

    uint32_t ntnCount = 0;
    for (const auto& [ueId, state] : m_ueSteeringStates)
    {
        if (state.currentNetwork == NetworkSelection::SATELLITE ||
            state.currentNetwork == NetworkSelection::DUAL_CONNECTED)
        {
            ntnCount++;
        }
    }
    return static_cast<double>(ntnCount) /
           static_cast<double>(m_ueSteeringStates.size());
}

OranNtnXappTnNtnSteering::SteeringMetrics
OranNtnXappTnNtnSteering::GetSteeringMetrics() const
{
    return m_steeringMetrics;
}

// ---- E2 data processing ----

void
OranNtnXappTnNtnSteering::ProcessKpmReport(const E2KpmReport& report)
{
    NS_LOG_FUNCTION(this << report.gnbId << report.ueId << report.isNtn);

    auto& state = m_ueSteeringStates[report.ueId];

    if (report.isNtn)
    {
        // NTN (satellite) measurements
        state.ntnSinr = report.sinr_dB;
        state.ntnThroughput = report.throughput_Mbps;
        state.ntnLoad = report.prbUtilization;

        NS_LOG_DEBUG("UE " << report.ueId << " NTN report:"
                     << " SINR=" << report.sinr_dB << " dB"
                     << ", throughput=" << report.throughput_Mbps << " Mbps"
                     << ", load=" << report.prbUtilization);
    }
    else
    {
        // TN (terrestrial) measurements
        state.tnSinr = report.sinr_dB;
        state.tnThroughput = report.throughput_Mbps;
        state.tnLoad = report.prbUtilization;

        NS_LOG_DEBUG("UE " << report.ueId << " TN report:"
                     << " SINR=" << report.sinr_dB << " dB"
                     << ", throughput=" << report.throughput_Mbps << " Mbps"
                     << ", load=" << report.prbUtilization);
    }

    state.sliceId = report.sliceId;
}

void
OranNtnXappTnNtnSteering::DecisionCycle()
{
    NS_LOG_FUNCTION(this);

    Time now = Simulator::Now();

    for (auto& [ueId, state] : m_ueSteeringStates)
    {
        // Compute optimal network for this UE
        NetworkSelection optimal = ComputeOptimalNetwork(ueId, state);

        if (optimal == state.currentNetwork)
        {
            continue; // No change needed
        }

        NS_LOG_INFO("UE " << ueId
                    << ": steering from "
                    << static_cast<uint8_t>(state.currentNetwork)
                    << " to " << static_cast<uint8_t>(optimal)
                    << " (mode=" << m_steeringMode << ")");

        NetworkSelection oldNetwork = state.currentNetwork;

        // Handle dual-connectivity setup
        if (optimal == NetworkSelection::DUAL_CONNECTED)
        {
            // Dual connectivity requires both TN and NTN links to be viable
            bool tnViable = (state.tnSinr >= m_minSinrTn);
            bool ntnViable = (state.ntnSinr >= m_minSinrNtn);

            if (!tnViable || !ntnViable)
            {
                NS_LOG_DEBUG("UE " << ueId
                             << ": dual-connectivity not viable"
                             << " (tnSinr=" << state.tnSinr
                             << ", ntnSinr=" << state.ntnSinr << ")");
                // Fall back to the viable single network
                optimal = tnViable ? NetworkSelection::TERRESTRIAL
                                   : NetworkSelection::SATELLITE;
            }
            else
            {
                m_steeringMetrics.dualConnSetups++;
            }
        }

        // Check coverage gap: neither network meets minimum SINR
        bool tnAvailable = (state.tnSinr >= m_minSinrTn);
        bool ntnAvailable = (state.ntnSinr >= m_minSinrNtn);

        if (!tnAvailable && !ntnAvailable)
        {
            m_steeringMetrics.coverageGapEvents++;
            NS_LOG_WARN("UE " << ueId << ": coverage gap detected"
                        << " (tnSinr=" << state.tnSinr
                        << ", ntnSinr=" << state.ntnSinr << ")");
            // Keep current network as best-effort
            continue;
        }

        // If target network is not available, keep current
        if (optimal == NetworkSelection::TERRESTRIAL && !tnAvailable)
        {
            NS_LOG_DEBUG("UE " << ueId << ": TN not available, staying on NTN");
            continue;
        }
        if (optimal == NetworkSelection::SATELLITE && !ntnAvailable)
        {
            NS_LOG_DEBUG("UE " << ueId << ": NTN not available, staying on TN");
            continue;
        }

        // Build and submit handover action to switch network
        double tnScore = ComputeSteeringScore(state, NetworkSelection::TERRESTRIAL);
        double ntnScore = ComputeSteeringScore(state, NetworkSelection::SATELLITE);
        double scoreDiff = std::abs(tnScore - ntnScore);
        double confidence = std::min(1.0, scoreDiff / (tnScore + ntnScore + 1e-9));

        E2RcAction action = BuildAction(E2RcActionType::HANDOVER_TRIGGER,
                                        0, // target gNB determined by RIC
                                        ueId,
                                        confidence);
        // parameter1: target network type (0=TN, 1=NTN, 2=dual)
        action.parameter1 = static_cast<double>(optimal);
        // parameter2: steering score of chosen network
        action.parameter2 = (optimal == NetworkSelection::TERRESTRIAL) ? tnScore
                          : (optimal == NetworkSelection::SATELLITE)   ? ntnScore
                          : std::max(tnScore, ntnScore);

        bool accepted = SubmitAction(action);
        RecordDecision(accepted, confidence, 0.0);

        if (accepted)
        {
            state.currentNetwork = optimal;
            state.lastSwitchTime = now;

            // Update metrics
            if (oldNetwork == NetworkSelection::TERRESTRIAL &&
                optimal == NetworkSelection::SATELLITE)
            {
                m_steeringMetrics.tnToNtnSwitches++;
            }
            else if (oldNetwork == NetworkSelection::SATELLITE &&
                     optimal == NetworkSelection::TERRESTRIAL)
            {
                m_steeringMetrics.ntnToTnSwitches++;
            }

            // Fire trace
            m_steeringDecision(ueId,
                               static_cast<uint8_t>(oldNetwork),
                               static_cast<uint8_t>(optimal));

            NS_LOG_INFO("Steering accepted for UE " << ueId
                        << ": " << static_cast<uint8_t>(oldNetwork)
                        << " -> " << static_cast<uint8_t>(optimal));
        }
    }

    // Update utilization metrics
    m_steeringMetrics.avgTnUtilization = GetTnTrafficFraction();
    m_steeringMetrics.avgNtnUtilization = GetNtnTrafficFraction();

    // Compute average latency and throughput across steered UEs
    if (!m_ueSteeringStates.empty())
    {
        double totalThroughput = 0.0;
        uint32_t count = 0;
        for (const auto& [ueId, state] : m_ueSteeringStates)
        {
            if (state.currentNetwork == NetworkSelection::TERRESTRIAL)
            {
                totalThroughput += state.tnThroughput;
            }
            else if (state.currentNetwork == NetworkSelection::SATELLITE)
            {
                totalThroughput += state.ntnThroughput;
            }
            else
            {
                // Dual-connected: aggregate throughput
                totalThroughput += state.tnThroughput + state.ntnThroughput;
            }
            count++;
        }
        if (count > 0)
        {
            m_steeringMetrics.avgSteeredThroughput_Mbps =
                totalThroughput / static_cast<double>(count);
        }
    }
}

NetworkSelection
OranNtnXappTnNtnSteering::ComputeOptimalNetwork(uint32_t ueId,
                                                  const UeSteeringState& state) const
{
    NS_LOG_FUNCTION(this << ueId << m_steeringMode);

    bool tnAvailable = (state.tnSinr >= m_minSinrTn);
    bool ntnAvailable = (state.ntnSinr >= m_minSinrNtn);

    // If only one network is available, return that
    if (tnAvailable && !ntnAvailable)
    {
        return NetworkSelection::TERRESTRIAL;
    }
    if (!tnAvailable && ntnAvailable)
    {
        return NetworkSelection::SATELLITE;
    }
    if (!tnAvailable && !ntnAvailable)
    {
        return state.currentNetwork; // Keep current as fallback
    }

    // Both networks available - apply steering mode logic
    if (m_steeringMode == "latency-optimal")
    {
        // Prefer lowest delay: TN typically has lower latency
        // NTN has propagation delay penalty
        // Use TN unless its load is very high (>90%) and NTN is lightly loaded
        double tnEffectiveLatency = state.tnLoad * 10.0; // Load-based queuing estimate
        double ntnEffectiveLatency = m_ntnLatencyPenalty + state.ntnLoad * 10.0;

        if (ntnEffectiveLatency < tnEffectiveLatency)
        {
            return NetworkSelection::SATELLITE;
        }
        return NetworkSelection::TERRESTRIAL;
    }
    else if (m_steeringMode == "throughput-optimal")
    {
        // Prefer highest throughput, accounting for load
        double tnEffectiveTput = state.tnThroughput * (1.0 - state.tnLoad * 0.5);
        double ntnEffectiveTput = state.ntnThroughput * (1.0 - state.ntnLoad * 0.5);

        // If both are good, consider dual-connectivity
        if (tnEffectiveTput > 0.0 && ntnEffectiveTput > 0.0 &&
            std::min(tnEffectiveTput, ntnEffectiveTput) >
                0.3 * std::max(tnEffectiveTput, ntnEffectiveTput))
        {
            return NetworkSelection::DUAL_CONNECTED;
        }

        return (ntnEffectiveTput > tnEffectiveTput) ? NetworkSelection::SATELLITE
                                                    : NetworkSelection::TERRESTRIAL;
    }
    else if (m_steeringMode == "reliability-optimal")
    {
        // Prefer better SINR for higher reliability
        // Apply minimum margin above threshold
        double tnMargin = state.tnSinr - m_minSinrTn;
        double ntnMargin = state.ntnSinr - m_minSinrNtn;

        // If both have good margin, consider dual-connectivity for diversity
        if (tnMargin > 5.0 && ntnMargin > 5.0)
        {
            return NetworkSelection::DUAL_CONNECTED;
        }

        return (ntnMargin > tnMargin) ? NetworkSelection::SATELLITE
                                      : NetworkSelection::TERRESTRIAL;
    }
    else if (m_steeringMode == "cost-optimal")
    {
        // Prefer TN when available (lower cost, uses existing infrastructure)
        // Only use NTN when TN is congested or unavailable
        if (tnAvailable && state.tnLoad < 0.85)
        {
            return NetworkSelection::TERRESTRIAL;
        }
        if (ntnAvailable)
        {
            return NetworkSelection::SATELLITE;
        }
        return NetworkSelection::TERRESTRIAL;
    }

    // Fallback: use scoring-based approach
    double tnScore = ComputeSteeringScore(state, NetworkSelection::TERRESTRIAL);
    double ntnScore = ComputeSteeringScore(state, NetworkSelection::SATELLITE);

    return (ntnScore > tnScore) ? NetworkSelection::SATELLITE
                                : NetworkSelection::TERRESTRIAL;
}

double
OranNtnXappTnNtnSteering::ComputeSteeringScore(const UeSteeringState& state,
                                                 NetworkSelection candidate) const
{
    NS_LOG_FUNCTION(this << static_cast<uint8_t>(candidate));

    // Weighted sum of: SINR, throughput, load, latency factors
    // Weights depend on steering mode
    double wSinr = 0.25;
    double wThroughput = 0.25;
    double wLoad = 0.25;
    double wLatency = 0.25;

    if (m_steeringMode == "latency-optimal")
    {
        wSinr = 0.15;
        wThroughput = 0.15;
        wLoad = 0.20;
        wLatency = 0.50;
    }
    else if (m_steeringMode == "throughput-optimal")
    {
        wSinr = 0.20;
        wThroughput = 0.50;
        wLoad = 0.20;
        wLatency = 0.10;
    }
    else if (m_steeringMode == "reliability-optimal")
    {
        wSinr = 0.50;
        wThroughput = 0.15;
        wLoad = 0.15;
        wLatency = 0.20;
    }
    else if (m_steeringMode == "cost-optimal")
    {
        wSinr = 0.20;
        wThroughput = 0.20;
        wLoad = 0.30;
        wLatency = 0.30;
    }

    double sinr = 0.0;
    double throughput = 0.0;
    double load = 0.0;
    double latencyFactor = 0.0;

    if (candidate == NetworkSelection::TERRESTRIAL)
    {
        // Normalize SINR: map [-20, 30] dB range to [0, 1]
        sinr = std::max(0.0, std::min(1.0, (state.tnSinr + 20.0) / 50.0));
        // Normalize throughput: assume max 1000 Mbps
        throughput = std::min(1.0, state.tnThroughput / 1000.0);
        // Load factor: lower load is better
        load = 1.0 - state.tnLoad;
        // Latency factor: TN has inherently low latency
        latencyFactor = 1.0;
        // Apply TN preference
        latencyFactor *= (1.0 + m_tnPreferenceFactor * 0.2);
    }
    else if (candidate == NetworkSelection::SATELLITE)
    {
        sinr = std::max(0.0, std::min(1.0, (state.ntnSinr + 20.0) / 50.0));
        throughput = std::min(1.0, state.ntnThroughput / 1000.0);
        load = 1.0 - state.ntnLoad;
        // NTN latency penalty: higher penalty -> lower score
        // Normalize: 0 ms penalty -> 1.0, 50 ms penalty -> 0.0
        latencyFactor = std::max(0.0, 1.0 - m_ntnLatencyPenalty / 50.0);
        // Apply NTN preference (inverse of TN preference)
        latencyFactor *= (1.0 + (1.0 - m_tnPreferenceFactor) * 0.2);
    }

    double score = wSinr * sinr +
                   wThroughput * throughput +
                   wLoad * load +
                   wLatency * latencyFactor;

    NS_LOG_DEBUG("Score for "
                 << ((candidate == NetworkSelection::TERRESTRIAL) ? "TN" : "NTN")
                 << ": sinr=" << sinr
                 << ", throughput=" << throughput
                 << ", load=" << load
                 << ", latency=" << latencyFactor
                 << " -> total=" << score);

    return score;
}

E2Subscription
OranNtnXappTnNtnSteering::GetRequiredSubscription() const
{
    E2Subscription sub;
    sub.subscriptionId = 0; // Assigned by E2 termination
    sub.ricRequestorId = GetXappId();
    sub.ranFunctionId = 2; // KPM service model
    sub.reportingPeriod = MilliSeconds(500);
    sub.eventTrigger = false; // Periodic reporting
    sub.eventThreshold = 0.0;
    sub.batchOnVisibility = false;
    sub.maxBufferAge = Seconds(10);
    sub.useIslRelay = false;
    return sub;
}

} // namespace ns3
