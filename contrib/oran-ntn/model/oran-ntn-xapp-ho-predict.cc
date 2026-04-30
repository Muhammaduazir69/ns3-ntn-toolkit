/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-only
 * Author: Muhammad Uzair
 *
 * HO Prediction xApp - Proactive handover using SINR trend analysis
 * and simplified DQN/LSTM on-board inference for NTN scenarios.
 */

#include "oran-ntn-xapp-ho-predict.h"

#include <ns3/boolean.h>
#include <ns3/double.h>
#include <ns3/log.h>
#include <ns3/simulator.h>
#include <ns3/uinteger.h>

#include <algorithm>
#include <cmath>
#include <numeric>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("OranNtnXappHoPredict");
NS_OBJECT_ENSURE_REGISTERED(OranNtnXappHoPredict);

TypeId
OranNtnXappHoPredict::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::OranNtnXappHoPredict")
            .SetParent<OranNtnXappBase>()
            .SetGroupName("OranNtn")
            .AddConstructor<OranNtnXappHoPredict>()
            .AddAttribute("SinrDropThreshold",
                          "SINR threshold below which handover is needed (dB)",
                          DoubleValue(3.0),
                          MakeDoubleAccessor(&OranNtnXappHoPredict::m_sinrDropThreshold),
                          MakeDoubleChecker<double>(-20.0, 40.0))
            .AddAttribute("TteMinimum",
                          "Minimum TTE required for a candidate satellite",
                          TimeValue(Seconds(30.0)),
                          MakeTimeAccessor(&OranNtnXappHoPredict::m_tteMinimum),
                          MakeTimeChecker())
            .AddAttribute("ProactiveLeadTime",
                          "How far ahead to predict SINR for proactive HO",
                          TimeValue(Seconds(5.0)),
                          MakeTimeAccessor(&OranNtnXappHoPredict::m_proactiveLeadTime),
                          MakeTimeChecker())
            .AddAttribute("PingPongGuardTime",
                          "Minimum time between consecutive handovers per UE",
                          TimeValue(Seconds(2.0)),
                          MakeTimeAccessor(&OranNtnXappHoPredict::m_pingPongGuardTime),
                          MakeTimeChecker())
            .AddAttribute("TrendWindowSize",
                          "Number of SINR samples for trend analysis",
                          UintegerValue(20),
                          MakeUintegerAccessor(&OranNtnXappHoPredict::m_trendWindowSize),
                          MakeUintegerChecker<uint32_t>(3, 200))
            .AddAttribute("AiEnabled",
                          "Enable AI-assisted mode (DQN/LSTM) vs. rule-based",
                          BooleanValue(false),
                          MakeBooleanAccessor(&OranNtnXappHoPredict::m_aiEnabled),
                          MakeBooleanChecker())
            .AddTraceSource("HoTriggered",
                            "A proactive handover was triggered",
                            MakeTraceSourceAccessor(&OranNtnXappHoPredict::m_hoTriggered),
                            "ns3::OranNtnXappHoPredict::HoTriggeredTracedCallback");
    return tid;
}

OranNtnXappHoPredict::OranNtnXappHoPredict()
    : m_sinrDropThreshold(3.0),
      m_tteMinimum(Seconds(30.0)),
      m_proactiveLeadTime(Seconds(5.0)),
      m_pingPongGuardTime(Seconds(2.0)),
      m_trendWindowSize(20),
      m_aiEnabled(false)
{
    NS_LOG_FUNCTION(this);
    m_hoPredMetrics = {};
}

OranNtnXappHoPredict::~OranNtnXappHoPredict()
{
    NS_LOG_FUNCTION(this);
}

// ============================================================================
//  Configuration setters
// ============================================================================

void
OranNtnXappHoPredict::SetSinrDropThreshold(double dB)
{
    m_sinrDropThreshold = dB;
}

void
OranNtnXappHoPredict::SetTteMinimum(Time tte)
{
    m_tteMinimum = tte;
}

void
OranNtnXappHoPredict::SetProactiveLeadTime(Time lead)
{
    m_proactiveLeadTime = lead;
}

void
OranNtnXappHoPredict::SetPingPongGuardTime(Time guard)
{
    m_pingPongGuardTime = guard;
}

void
OranNtnXappHoPredict::SetTrendWindowSize(uint32_t samples)
{
    m_trendWindowSize = samples;
}

// ============================================================================
//  AI model interface
// ============================================================================

void
OranNtnXappHoPredict::SetDqnWeights(const std::vector<double>& weights)
{
    m_dqnWeights = weights;
}

void
OranNtnXappHoPredict::SetLstmWeights(const std::vector<double>& weights)
{
    m_lstmWeights = weights;
}

void
OranNtnXappHoPredict::SetAiEnabled(bool enabled)
{
    m_aiEnabled = enabled;
}

// ============================================================================
//  Metrics
// ============================================================================

OranNtnXappHoPredict::HoPredictMetrics
OranNtnXappHoPredict::GetHoPredictMetrics() const
{
    return m_hoPredMetrics;
}

// ============================================================================
//  E2 KPM report processing
// ============================================================================

void
OranNtnXappHoPredict::ProcessKpmReport(const E2KpmReport& report)
{
    NS_LOG_FUNCTION(this << report.ueId << report.gnbId << report.sinr_dB);

    auto& state = m_ueStates[report.ueId];

    // Update serving gNB tracking
    state.servingGnbId = report.gnbId;

    // Append SINR measurement to history
    state.sinrHistory.push_back(std::make_pair(report.timestamp, report.sinr_dB));

    // Trim history to window size
    while (state.sinrHistory.size() > m_trendWindowSize)
    {
        state.sinrHistory.pop_front();
    }

    NS_LOG_DEBUG("UE " << report.ueId << ": SINR=" << report.sinr_dB
                 << " dB from gNB " << report.gnbId
                 << " (history size=" << state.sinrHistory.size() << ")");
}

// ============================================================================
//  Decision cycle
// ============================================================================

void
OranNtnXappHoPredict::DecisionCycle()
{
    NS_LOG_FUNCTION(this);

    double now = Simulator::Now().GetSeconds();

    for (auto& [ueId, state] : m_ueStates)
    {
        // Need at least 3 samples for meaningful trend
        if (state.sinrHistory.size() < 3)
        {
            continue;
        }

        // Skip if handover is already pending
        if (state.handoverPending)
        {
            continue;
        }

        // Ping-pong guard: skip if recent handover
        if (state.lastHandoverTime > Time(0) &&
            (Simulator::Now() - state.lastHandoverTime) < m_pingPongGuardTime)
        {
            NS_LOG_DEBUG("UE " << ueId << ": Skipping - within ping-pong guard time");
            m_hoPredMetrics.pingPongsAvoided++;
            continue;
        }

        // Compute SINR trend
        SinrTrend trend = ComputeSinrTrend(ueId);

        NS_LOG_DEBUG("UE " << ueId << ": slope=" << trend.slope
                     << " dB/s, predicted=" << trend.predictedSinr
                     << " dB, R²=" << trend.r_squared);

        // Check if SINR is predicted to drop below threshold within lead time
        bool needsHandover = false;

        if (m_aiEnabled)
        {
            // Use LSTM prediction for more accurate SINR forecast
            double predictedSinr = LstmPredictSinr(ueId, m_proactiveLeadTime);
            needsHandover = (predictedSinr < m_sinrDropThreshold);
        }
        else
        {
            // Rule-based: check linear regression prediction
            needsHandover = (trend.predictedSinr < m_sinrDropThreshold) &&
                            (trend.slope < 0.0) &&
                            (trend.r_squared > 0.3); // Require reasonable fit
        }

        if (!needsHandover)
        {
            continue;
        }

        NS_LOG_INFO("UE " << ueId << ": Proactive HO needed, predicted SINR="
                    << trend.predictedSinr << " dB < threshold="
                    << m_sinrDropThreshold << " dB");

        // Search for best candidate gNB using TTE-aware ranking
        uint32_t bestCandidate = 0;
        double bestScore = -1.0;
        double bestCandSinr = 0.0;
        double bestCandTte = 0.0;
        double bestCandElevation = 0.0;

        // Scan all gNBs in the KPM database for candidate cells
        for (const auto& [gnbId, reports] : m_kpmDatabase)
        {
            if (gnbId == state.servingGnbId || reports.empty())
            {
                continue;
            }

            // Find the latest report for this UE from this gNB
            const E2KpmReport* latestUeReport = nullptr;
            for (auto it = reports.rbegin(); it != reports.rend(); ++it)
            {
                if (it->ueId == ueId)
                {
                    latestUeReport = &(*it);
                    break;
                }
            }

            if (!latestUeReport)
            {
                continue;
            }

            // Filter: candidate must meet minimum TTE
            if (latestUeReport->tte_s < m_tteMinimum.GetSeconds())
            {
                NS_LOG_DEBUG("  Candidate gNB " << gnbId << ": TTE="
                             << latestUeReport->tte_s << " s < min="
                             << m_tteMinimum.GetSeconds() << " s, skipping");
                continue;
            }

            // Filter: candidate SINR must be above threshold
            if (latestUeReport->sinr_dB < m_sinrDropThreshold)
            {
                continue;
            }

            // TTE-aware ranking score:
            //   score = w1 * normalized_sinr + w2 * normalized_tte + w3 * normalized_elevation
            double normSinr = std::min(latestUeReport->sinr_dB / 30.0, 1.0);
            double normTte = std::min(latestUeReport->tte_s / 300.0, 1.0);
            double normElev = std::min(latestUeReport->elevation_deg / 90.0, 1.0);

            double score = 0.4 * normSinr + 0.4 * normTte + 0.2 * normElev;

            NS_LOG_DEBUG("  Candidate gNB " << gnbId << ": SINR="
                         << latestUeReport->sinr_dB << " dB, TTE="
                         << latestUeReport->tte_s << " s, score=" << score);

            if (score > bestScore)
            {
                bestScore = score;
                bestCandidate = gnbId;
                bestCandSinr = latestUeReport->sinr_dB;
                bestCandTte = latestUeReport->tte_s;
                bestCandElevation = latestUeReport->elevation_deg;
            }
        }

        if (bestCandidate == 0)
        {
            NS_LOG_DEBUG("UE " << ueId << ": No suitable candidate found");
            continue;
        }

        // AI-assisted action selection (if enabled)
        if (m_aiEnabled && !m_dqnWeights.empty())
        {
            // Build feature vector for DQN
            HoFeatureVector features;
            features.servingSinr = state.sinrHistory.back().second;
            features.servingTte = 0.0;
            features.servingElevation = 0.0;
            features.servingDoppler = 0.0;
            features.sinrSlope = trend.slope;
            features.bestCandSinr = bestCandSinr;
            features.bestCandTte = bestCandTte;
            features.bestCandElevation = bestCandElevation;
            features.timeSinceLastHo = (state.lastHandoverTime > Time(0))
                                           ? (now - state.lastHandoverTime.GetSeconds())
                                           : 999.0;
            features.recentHoCount = state.recentHandovers;

            // Fetch serving cell info from latest report
            auto servingReport = GetLatestReport(state.servingGnbId);
            features.servingTte = servingReport.tte_s;
            features.servingElevation = servingReport.elevation_deg;
            features.servingDoppler = servingReport.doppler_Hz;

            uint32_t action = DqnSelectAction(features);
            if (action == 0)
            {
                NS_LOG_DEBUG("UE " << ueId << ": DQN says STAY");
                continue; // DQN decided not to handover
            }
        }

        // Submit handover action
        double confidence = std::min(trend.r_squared * bestScore, 1.0);
        E2RcAction hoAction = BuildAction(E2RcActionType::HANDOVER_TRIGGER,
                                           bestCandidate,
                                           ueId,
                                           confidence);
        hoAction.parameter1 = bestCandSinr; // Store candidate SINR
        hoAction.parameter2 = bestCandTte;  // Store candidate TTE

        bool accepted = SubmitAction(hoAction);

        if (accepted)
        {
            state.handoverPending = true;
            state.lastHandoverTime = Simulator::Now();
            state.recentHandovers++;

            m_hoPredMetrics.proactiveHandovers++;

            // Compute lead time (how early we triggered)
            double leadTime = 0.0;
            if (trend.timeToThreshold > Time(0))
            {
                leadTime = trend.timeToThreshold.GetSeconds();
            }
            // Update running average of lead time
            double n = static_cast<double>(m_hoPredMetrics.proactiveHandovers);
            m_hoPredMetrics.avgLeadTime_s =
                m_hoPredMetrics.avgLeadTime_s * ((n - 1.0) / n) + leadTime / n;

            RecordDecision(true, confidence, 0.0);

            m_hoTriggered(ueId, bestCandidate, bestCandSinr, bestCandTte);

            NS_LOG_INFO("UE " << ueId << ": Proactive HO to gNB " << bestCandidate
                        << " (SINR=" << bestCandSinr << " dB, TTE=" << bestCandTte
                        << " s, confidence=" << confidence << ")");
        }
        else
        {
            RecordDecision(false, confidence, 0.0);
            NS_LOG_WARN("UE " << ueId << ": HO action rejected by RIC");
        }
    }
}

// ============================================================================
//  SINR trend analysis via linear regression
// ============================================================================

OranNtnXappHoPredict::SinrTrend
OranNtnXappHoPredict::ComputeSinrTrend(uint32_t ueId) const
{
    SinrTrend result;
    result.slope = 0.0;
    result.intercept = 0.0;
    result.r_squared = 0.0;
    result.predictedSinr = 0.0;
    result.timeToThreshold = Seconds(0);

    auto it = m_ueStates.find(ueId);
    if (it == m_ueStates.end() || it->second.sinrHistory.size() < 3)
    {
        return result;
    }

    const auto& history = it->second.sinrHistory;
    uint32_t n = history.size();

    // Linear regression: SINR = slope * time + intercept
    double sumT = 0.0;
    double sumS = 0.0;
    double sumTS = 0.0;
    double sumTT = 0.0;
    double sumSS = 0.0;

    for (const auto& [t, s] : history)
    {
        sumT += t;
        sumS += s;
        sumTS += t * s;
        sumTT += t * t;
        sumSS += s * s;
    }

    double nd = static_cast<double>(n);
    double denom = nd * sumTT - sumT * sumT;

    if (std::abs(denom) < 1e-12)
    {
        // Degenerate case: all timestamps identical
        result.intercept = sumS / nd;
        result.predictedSinr = result.intercept;
        return result;
    }

    result.slope = (nd * sumTS - sumT * sumS) / denom;
    result.intercept = (sumS - result.slope * sumT) / nd;

    // R-squared (coefficient of determination)
    double meanS = sumS / nd;
    double ssTot = sumSS - nd * meanS * meanS;
    double ssRes = 0.0;
    for (const auto& [t, s] : history)
    {
        double predicted = result.slope * t + result.intercept;
        double residual = s - predicted;
        ssRes += residual * residual;
    }

    result.r_squared = (ssTot > 1e-12) ? (1.0 - ssRes / ssTot) : 0.0;
    result.r_squared = std::max(0.0, result.r_squared);

    // Predict SINR at current time + lead time
    double now = Simulator::Now().GetSeconds();
    double futureTime = now + m_proactiveLeadTime.GetSeconds();
    result.predictedSinr = result.slope * futureTime + result.intercept;

    // Time to threshold: solve slope * t + intercept = threshold
    if (result.slope < -1e-9)
    {
        double tThreshold = (m_sinrDropThreshold - result.intercept) / result.slope;
        double timeRemaining = tThreshold - now;
        if (timeRemaining > 0.0)
        {
            result.timeToThreshold = Seconds(timeRemaining);
        }
    }

    return result;
}

// ============================================================================
//  DQN action selection (simplified on-board inference)
// ============================================================================

uint32_t
OranNtnXappHoPredict::DqnSelectAction(const HoFeatureVector& features) const
{
    NS_LOG_FUNCTION(this);

    if (m_dqnWeights.empty())
    {
        return 1; // Default: handover to best candidate
    }

    // Build feature array
    std::vector<double> featureVec = {
        features.servingSinr / 30.0,          // Normalize to ~[0,1]
        features.servingTte / 300.0,
        features.servingElevation / 90.0,
        features.servingDoppler / 50000.0,
        features.sinrSlope / 5.0,             // dB/s, normalized
        features.bestCandSinr / 30.0,
        features.bestCandTte / 300.0,
        features.bestCandElevation / 90.0,
        std::min(features.timeSinceLastHo / 60.0, 1.0),
        std::min(static_cast<double>(features.recentHoCount) / 5.0, 1.0),
    };

    // Simple dot product with weights for each action
    // Weights layout: [action0_w0..w9, action1_w0..w9, ...]
    uint32_t numFeatures = featureVec.size();
    uint32_t numActions = m_dqnWeights.size() / numFeatures;

    if (numActions < 2)
    {
        return 1; // Not enough weights, default to handover
    }

    uint32_t bestAction = 0;
    double bestQValue = -1e30;

    for (uint32_t a = 0; a < numActions; ++a)
    {
        double qValue = 0.0;
        for (uint32_t f = 0; f < numFeatures; ++f)
        {
            uint32_t wIdx = a * numFeatures + f;
            if (wIdx < m_dqnWeights.size())
            {
                qValue += m_dqnWeights[wIdx] * featureVec[f];
            }
        }

        if (qValue > bestQValue)
        {
            bestQValue = qValue;
            bestAction = a;
        }
    }

    NS_LOG_DEBUG("DQN selected action " << bestAction
                 << " with Q-value " << bestQValue);

    return bestAction;
}

// ============================================================================
//  LSTM SINR prediction (simplified weighted moving average)
// ============================================================================

double
OranNtnXappHoPredict::LstmPredictSinr(uint32_t ueId, Time horizon) const
{
    NS_LOG_FUNCTION(this << ueId << horizon.GetSeconds());

    auto it = m_ueStates.find(ueId);
    if (it == m_ueStates.end() || it->second.sinrHistory.empty())
    {
        return 0.0;
    }

    const auto& history = it->second.sinrHistory;
    uint32_t n = history.size();

    if (n < 2)
    {
        return history.back().second;
    }

    // Weighted moving average: more recent samples get higher weight
    // If LSTM weights are provided, use them; otherwise use exponential decay
    double weightedSum = 0.0;
    double weightSum = 0.0;

    for (uint32_t i = 0; i < n; ++i)
    {
        double weight;
        if (!m_lstmWeights.empty() && i < m_lstmWeights.size())
        {
            weight = m_lstmWeights[i];
        }
        else
        {
            // Exponential decay: more recent = higher weight
            double alpha = 0.9;
            weight = std::pow(alpha, static_cast<double>(n - 1 - i));
        }

        weightedSum += weight * history[i].second;
        weightSum += weight;
    }

    double smoothedSinr = (weightSum > 0.0) ? (weightedSum / weightSum) : 0.0;

    // Estimate velocity from the last two weighted averages and extrapolate
    // Use the linear trend slope for extrapolation
    SinrTrend trend = ComputeSinrTrend(ueId);
    double predictedSinr = smoothedSinr + trend.slope * horizon.GetSeconds();

    NS_LOG_DEBUG("LSTM predict UE " << ueId << ": smoothed=" << smoothedSinr
                 << " dB, slope=" << trend.slope
                 << " dB/s, predicted=" << predictedSinr << " dB");

    return predictedSinr;
}

// ============================================================================
//  E2 subscription
// ============================================================================

E2Subscription
OranNtnXappHoPredict::GetRequiredSubscription() const
{
    E2Subscription sub;
    sub.subscriptionId = 0; // Assigned by E2 termination
    sub.ricRequestorId = GetXappId();
    sub.ranFunctionId = 2;  // KPM service model
    sub.reportingPeriod = MilliSeconds(100);
    sub.eventTrigger = false; // Periodic reporting
    sub.eventThreshold = 0.0;

    // NTN extensions
    sub.batchOnVisibility = false; // Need real-time SINR updates
    sub.maxBufferAge = Seconds(1);
    sub.useIslRelay = true; // Use ISL if feeder link unavailable

    return sub;
}

} // namespace ns3
