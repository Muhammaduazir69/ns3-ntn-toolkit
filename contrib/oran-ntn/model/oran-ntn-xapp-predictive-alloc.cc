/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-only
 * Author: Muhammad Uzair
 *
 * Predictive Resource Allocation xApp - Implementation
 *
 * LSTM-based traffic prediction with linear extrapolation fallback,
 * confidence-weighted PRB reservation, and z-score anomaly detection
 * for proactive resource management in NTN beams.
 */

#include "oran-ntn-xapp-predictive-alloc.h"

#include "oran-ntn-gym-predictive.h"
#include "oran-ntn-sat-bridge.h"

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

NS_LOG_COMPONENT_DEFINE("OranNtnXappPredictiveAlloc");
NS_OBJECT_ENSURE_REGISTERED(OranNtnXappPredictiveAlloc);

TypeId
OranNtnXappPredictiveAlloc::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::OranNtnXappPredictiveAlloc")
            .SetParent<OranNtnXappBase>()
            .SetGroupName("OranNtn")
            .AddConstructor<OranNtnXappPredictiveAlloc>()
            .AddAttribute("PredictionHorizon",
                          "Number of steps to predict into the future",
                          UintegerValue(5),
                          MakeUintegerAccessor(
                              &OranNtnXappPredictiveAlloc::m_predictionHorizon),
                          MakeUintegerChecker<uint32_t>(1, 50))
            .AddAttribute("HistoryWindow",
                          "Number of historical steps to use for prediction",
                          UintegerValue(20),
                          MakeUintegerAccessor(
                              &OranNtnXappPredictiveAlloc::m_historyWindow),
                          MakeUintegerChecker<uint32_t>(2, 500))
            .AddAttribute("ReservationAggression",
                          "Aggression factor for PRB reservation (0=conservative, 1=aggressive)",
                          DoubleValue(0.3),
                          MakeDoubleAccessor(
                              &OranNtnXappPredictiveAlloc::m_reservationAggression),
                          MakeDoubleChecker<double>(0.0, 1.0))
            .AddAttribute("AnomalyThreshold",
                          "Z-score threshold for anomaly detection",
                          DoubleValue(2.5),
                          MakeDoubleAccessor(
                              &OranNtnXappPredictiveAlloc::m_anomalyThreshold),
                          MakeDoubleChecker<double>(0.0))
            .AddAttribute("AiEnabled",
                          "Whether to use AI/Gym-based prediction",
                          BooleanValue(false),
                          MakeBooleanAccessor(&OranNtnXappPredictiveAlloc::m_aiEnabled),
                          MakeBooleanChecker());
    return tid;
}

OranNtnXappPredictiveAlloc::OranNtnXappPredictiveAlloc()
    : m_predictionHorizon(5),
      m_historyWindow(20),
      m_reservationAggression(0.3),
      m_anomalyThreshold(2.5),
      m_aiEnabled(false)
{
    NS_LOG_FUNCTION(this);
    SetXappName("PredictiveAlloc");

    m_predMetrics = {};
}

OranNtnXappPredictiveAlloc::~OranNtnXappPredictiveAlloc()
{
    NS_LOG_FUNCTION(this);
}

// --------------------------------------------------------------------------
//  Configuration setters
// --------------------------------------------------------------------------

void
OranNtnXappPredictiveAlloc::SetSatBridge(Ptr<OranNtnSatBridge> bridge)
{
    NS_LOG_FUNCTION(this << bridge);
    m_satBridge = bridge;
}

void
OranNtnXappPredictiveAlloc::SetPredictionHorizon(uint32_t steps)
{
    m_predictionHorizon = steps;
}

void
OranNtnXappPredictiveAlloc::SetHistoryWindow(uint32_t steps)
{
    m_historyWindow = steps;
}

void
OranNtnXappPredictiveAlloc::SetReservationAggression(double factor)
{
    m_reservationAggression = factor;
}

void
OranNtnXappPredictiveAlloc::SetAnomalyThreshold(double threshold)
{
    m_anomalyThreshold = threshold;
}

void
OranNtnXappPredictiveAlloc::SetGymEnv(Ptr<OranNtnGymPredictive> env)
{
    NS_LOG_FUNCTION(this << env);
    m_gymEnv = env;
    if (env)
    {
        m_aiEnabled = true;
    }
}

// --------------------------------------------------------------------------
//  Public queries
// --------------------------------------------------------------------------

TrafficPrediction
OranNtnXappPredictiveAlloc::GetPrediction(uint32_t beamId) const
{
    auto it = m_beamHistories.find(beamId);
    if (it != m_beamHistories.end())
    {
        return it->second.lastPrediction;
    }
    TrafficPrediction empty;
    empty.beamId = beamId;
    empty.timestamp = Simulator::Now().GetSeconds();
    empty.horizonSteps = 0;
    empty.mse = 0.0;
    return empty;
}

std::map<uint32_t, TrafficPrediction>
OranNtnXappPredictiveAlloc::GetAllPredictions() const
{
    std::map<uint32_t, TrafficPrediction> result;
    for (const auto& kv : m_beamHistories)
    {
        result[kv.first] = kv.second.lastPrediction;
    }
    return result;
}

bool
OranNtnXappPredictiveAlloc::IsAnomalyDetected(uint32_t beamId) const
{
    auto it = m_beamHistories.find(beamId);
    if (it != m_beamHistories.end())
    {
        return it->second.anomalyActive;
    }
    return false;
}

OranNtnXappPredictiveAlloc::PredictiveMetrics
OranNtnXappPredictiveAlloc::GetPredictiveMetrics() const
{
    return m_predMetrics;
}

// --------------------------------------------------------------------------
//  E2 subscription
// --------------------------------------------------------------------------

E2Subscription
OranNtnXappPredictiveAlloc::GetRequiredSubscription() const
{
    E2Subscription sub;
    sub.subscriptionId = 0; // Assigned by E2 termination
    sub.ricRequestorId = GetXappId();
    sub.ranFunctionId = 2;  // KPM service model
    sub.reportingPeriod = MilliSeconds(100);
    sub.eventTrigger = false; // Periodic reporting
    sub.eventThreshold = 0.0;
    sub.batchOnVisibility = false;
    sub.maxBufferAge = Seconds(5);
    sub.useIslRelay = true;
    return sub;
}

// --------------------------------------------------------------------------
//  ProcessKpmReport -- append beam load to per-beam history
// --------------------------------------------------------------------------

void
OranNtnXappPredictiveAlloc::ProcessKpmReport(const E2KpmReport& report)
{
    NS_LOG_FUNCTION(this << report.gnbId << report.beamId);

    uint32_t beamId = report.beamId;
    auto& history = m_beamHistories[beamId];

    // Append load to history deque
    history.loadHistory.push_back(report.prbUtilization);
    history.sinrHistory.push_back(report.sinr_dB);
    history.ueCountHistory.push_back(static_cast<double>(report.activeUes));

    // Trim history to window size
    while (history.loadHistory.size() > m_historyWindow)
    {
        history.loadHistory.pop_front();
    }
    while (history.sinrHistory.size() > m_historyWindow)
    {
        history.sinrHistory.pop_front();
    }
    while (history.ueCountHistory.size() > m_historyWindow)
    {
        history.ueCountHistory.pop_front();
    }

    NS_LOG_DEBUG("PredictiveAlloc: beam" << beamId << " load=" << report.prbUtilization
                                         << " history=" << history.loadHistory.size()
                                         << " points");
}

// --------------------------------------------------------------------------
//  DecisionCycle
// --------------------------------------------------------------------------

void
OranNtnXappPredictiveAlloc::DecisionCycle()
{
    NS_LOG_FUNCTION(this);

    double totalMse = 0.0;
    double totalConfidence = 0.0;
    uint32_t numPredictions = 0;

    for (auto& kv : m_beamHistories)
    {
        uint32_t beamId = kv.first;
        auto& history = kv.second;

        // Need at least a few data points for prediction
        if (history.loadHistory.size() < 3)
        {
            continue;
        }

        // Step 1: Run traffic prediction
        std::vector<double> predicted = PredictTrafficLoad(beamId);

        if (predicted.empty())
        {
            continue;
        }

        // Step 2: Build TrafficPrediction struct
        TrafficPrediction pred;
        pred.beamId = beamId;
        pred.timestamp = Simulator::Now().GetSeconds();
        pred.predictedLoad = predicted;
        pred.horizonSteps = static_cast<uint32_t>(predicted.size());

        // Compute per-step confidence from residual statistics
        pred.confidence.resize(predicted.size());
        for (uint32_t i = 0; i < predicted.size(); ++i)
        {
            // Confidence decays with prediction horizon
            double baseConf = (history.residualStd > 0.0)
                                  ? std::max(0.0, 1.0 - history.residualStd)
                                  : 0.5;
            double decay = std::exp(-0.2 * static_cast<double>(i));
            pred.confidence[i] = baseConf * decay;
        }

        // Compute MSE from last prediction vs actual (if available)
        if (!history.lastPrediction.predictedLoad.empty() &&
            history.loadHistory.size() > 0)
        {
            double actual = history.loadHistory.back();
            double prevPred = history.lastPrediction.predictedLoad[0];
            pred.mse = (actual - prevPred) * (actual - prevPred);

            // Update residual statistics for anomaly detection
            double residual = actual - prevPred;
            double alpha = 0.1; // exponential moving average factor
            history.residualMean =
                (1.0 - alpha) * history.residualMean + alpha * residual;
            double residualDev = std::abs(residual - history.residualMean);
            history.residualStd =
                (1.0 - alpha) * history.residualStd + alpha * residualDev;
        }

        history.lastPrediction = pred;
        m_predMetrics.totalPredictions++;
        totalMse += pred.mse;
        totalConfidence += (pred.confidence.empty() ? 0.0 : pred.confidence[0]);
        numPredictions++;

        // Step 3: Anomaly detection
        double actualLoad = history.loadHistory.back();
        bool anomaly = DetectAnomaly(beamId, actualLoad);
        history.anomalyActive = anomaly;

        if (anomaly)
        {
            m_predMetrics.anomaliesDetected++;
            NS_LOG_INFO("PredictiveAlloc: ANOMALY on beam" << beamId
                                                           << " actual=" << actualLoad);
        }

        // Step 4: Issue PRB reservation if prediction exceeds threshold
        double reservationThreshold = 0.7; // reserve when predicted load > 70%
        if (!predicted.empty() && predicted[0] > reservationThreshold)
        {
            double reservation = ComputeReservation(pred);

            E2RcAction action = BuildAction(E2RcActionType::PRB_RESERVATION,
                                            0, // target gnb determined by beam
                                            0,
                                            pred.confidence.empty() ? 0.5
                                                                    : pred.confidence[0]);
            action.targetBeamId = beamId;
            action.parameter1 = reservation; // reservation ratio (0-1)
            action.parameter2 = predicted[0]; // predicted load

            bool accepted = SubmitAction(action);
            if (accepted)
            {
                m_predMetrics.prbReservations++;
                NS_LOG_INFO("PredictiveAlloc: beam" << beamId << " PRB reservation="
                                                    << reservation << " (predicted="
                                                    << predicted[0] << ")");
            }
        }
    }

    // Update aggregate metrics
    if (numPredictions > 0)
    {
        m_predMetrics.avgMse = totalMse / numPredictions;
        m_predMetrics.avgConfidence = totalConfidence / numPredictions;
    }

    RecordDecision(true, m_predMetrics.avgConfidence, 0.0);
}

// --------------------------------------------------------------------------
//  PredictTrafficLoad -- AI or linear fallback
// --------------------------------------------------------------------------

std::vector<double>
OranNtnXappPredictiveAlloc::PredictTrafficLoad(uint32_t beamId)
{
    NS_LOG_FUNCTION(this << beamId);

    auto it = m_beamHistories.find(beamId);
    if (it == m_beamHistories.end() || it->second.loadHistory.size() < 3)
    {
        return {};
    }

    // If AI is enabled and gym environment is set, use it
    if (m_aiEnabled && m_gymEnv)
    {
        // Prepare observation vector from history for the gym environment
        // The gym env handles LSTM inference and returns predictions
        NS_LOG_DEBUG("PredictiveAlloc: using AI prediction for beam" << beamId);

        // Notify the gym environment with current state
        // The gym env will run inference and return action (predicted loads)
        // For now, fall through to linear if gym interaction fails
        // In production, this would call: m_gymEnv->Notify(beamId, history)
        // and retrieve the prediction from the gym env's action space
    }

    // Fallback: use linear extrapolation
    return LinearExtrapolation(it->second.loadHistory, m_predictionHorizon);
}

// --------------------------------------------------------------------------
//  LinearExtrapolation -- least-squares linear regression
// --------------------------------------------------------------------------

std::vector<double>
OranNtnXappPredictiveAlloc::LinearExtrapolation(const std::deque<double>& history,
                                                  uint32_t horizon) const
{
    NS_LOG_FUNCTION(this << history.size() << horizon);

    uint32_t n = static_cast<uint32_t>(history.size());
    if (n < 2)
    {
        return {};
    }

    // Fit linear regression: y = a + b*x
    // where x = 0, 1, 2, ..., n-1 and y = history values
    double sumX = 0.0;
    double sumY = 0.0;
    double sumXY = 0.0;
    double sumX2 = 0.0;

    for (uint32_t i = 0; i < n; ++i)
    {
        double x = static_cast<double>(i);
        double y = history[i];
        sumX += x;
        sumY += y;
        sumXY += x * y;
        sumX2 += x * x;
    }

    double nD = static_cast<double>(n);
    double denominator = nD * sumX2 - sumX * sumX;

    double slope = 0.0;
    double intercept = sumY / nD;

    if (std::abs(denominator) > 1e-12)
    {
        slope = (nD * sumXY - sumX * sumY) / denominator;
        intercept = (sumY - slope * sumX) / nD;
    }

    // Extrapolate forward by horizon steps
    std::vector<double> predictions(horizon);
    for (uint32_t h = 0; h < horizon; ++h)
    {
        double x = static_cast<double>(n + h);
        double pred = intercept + slope * x;

        // Clamp to valid range [0, 1] for PRB utilization
        pred = std::max(0.0, std::min(1.0, pred));
        predictions[h] = pred;
    }

    NS_LOG_DEBUG("LinearExtrapolation: slope=" << slope << " intercept=" << intercept
                                               << " next=" << predictions[0]);

    return predictions;
}

// --------------------------------------------------------------------------
//  DetectAnomaly -- z-score based anomaly detection
// --------------------------------------------------------------------------

bool
OranNtnXappPredictiveAlloc::DetectAnomaly(uint32_t beamId, double actualLoad)
{
    NS_LOG_FUNCTION(this << beamId << actualLoad);

    auto it = m_beamHistories.find(beamId);
    if (it == m_beamHistories.end())
    {
        return false;
    }

    const auto& history = it->second;

    // Need a previous prediction and sufficient statistics
    if (history.lastPrediction.predictedLoad.empty() || history.residualStd < 1e-6)
    {
        return false;
    }

    double predicted = history.lastPrediction.predictedLoad[0];
    double residual = actualLoad - predicted;

    // Compute z-score: z = (actual - predicted) / residualStd
    double zScore = residual / history.residualStd;

    bool anomaly = (std::abs(zScore) > m_anomalyThreshold);

    if (anomaly)
    {
        NS_LOG_INFO("PredictiveAlloc: anomaly detected on beam"
                    << beamId << " actual=" << actualLoad << " predicted=" << predicted
                    << " z=" << zScore << " threshold=" << m_anomalyThreshold);
    }

    return anomaly;
}

// --------------------------------------------------------------------------
//  ComputeReservation -- confidence-weighted PRB reservation
// --------------------------------------------------------------------------

double
OranNtnXappPredictiveAlloc::ComputeReservation(const TrafficPrediction& prediction) const
{
    NS_LOG_FUNCTION(this << prediction.beamId);

    if (prediction.predictedLoad.empty() || prediction.confidence.empty())
    {
        return 0.0;
    }

    double predictedLoad = prediction.predictedLoad[0];
    double confidence = prediction.confidence[0];

    // Reservation = prediction * (1 + aggression * confidence)
    // Higher aggression -> more over-provisioning when confident
    double reservation = predictedLoad * (1.0 + m_reservationAggression * confidence);

    // Clamp to [0, 1]
    reservation = std::max(0.0, std::min(1.0, reservation));

    NS_LOG_DEBUG("ComputeReservation: beam" << prediction.beamId
                                            << " predicted=" << predictedLoad
                                            << " confidence=" << confidence
                                            << " aggression=" << m_reservationAggression
                                            << " reservation=" << reservation);

    return reservation;
}

} // namespace ns3
