/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-only
 * Author: Muhammad Uzair
 *
 * Predictive Resource Allocation xApp
 *
 * LSTM-based traffic prediction for proactive PRB reservation.
 * Uses historical KPM time series per beam to forecast demand
 * and pre-allocate resources before congestion occurs.
 *
 * Novel features:
 *   - Multi-beam joint traffic prediction capturing spatial correlations
 *   - Orbit-aware periodicity detection (traffic patterns repeat per orbit)
 *   - Confidence-weighted PRB reservation (higher confidence = more aggressive)
 *   - Cross-satellite traffic migration prediction (UEs moving between sats)
 *   - Slice-aware prediction with separate models per slice type
 *   - Anomaly detection for traffic spikes (disaster, events)
 */

#ifndef ORAN_NTN_XAPP_PREDICTIVE_ALLOC_H
#define ORAN_NTN_XAPP_PREDICTIVE_ALLOC_H

#include "oran-ntn-xapp-base.h"

#include <deque>
#include <map>
#include <vector>

namespace ns3
{

class OranNtnSatBridge;
class OranNtnGymPredictive;

/**
 * \ingroup oran-ntn
 * \brief Predictive Resource Allocation xApp
 */
class OranNtnXappPredictiveAlloc : public OranNtnXappBase
{
  public:
    static TypeId GetTypeId();
    OranNtnXappPredictiveAlloc();
    ~OranNtnXappPredictiveAlloc() override;

    void SetSatBridge(Ptr<OranNtnSatBridge> bridge);
    void SetPredictionHorizon(uint32_t steps);
    void SetHistoryWindow(uint32_t steps);
    void SetReservationAggression(double factor);
    void SetAnomalyThreshold(double threshold);
    void SetGymEnv(Ptr<OranNtnGymPredictive> env);

    /**
     * \brief Get predicted traffic load for a beam
     */
    TrafficPrediction GetPrediction(uint32_t beamId) const;

    /**
     * \brief Get all current predictions
     */
    std::map<uint32_t, TrafficPrediction> GetAllPredictions() const;

    /**
     * \brief Check if anomaly is detected for a beam
     */
    bool IsAnomalyDetected(uint32_t beamId) const;

    // ---- Metrics ----
    struct PredictiveMetrics
    {
        uint32_t totalPredictions;
        double avgMse;
        double avgConfidence;
        uint32_t prbReservations;
        uint32_t anomaliesDetected;
        double avgReservationUtilization;
        double wastedReservation_pct;
    };
    PredictiveMetrics GetPredictiveMetrics() const;

  protected:
    void ProcessKpmReport(const E2KpmReport& report) override;
    void DecisionCycle() override;
    E2Subscription GetRequiredSubscription() const override;

  private:
    /**
     * \brief Run LSTM prediction (via Gym env or internal model)
     */
    std::vector<double> PredictTrafficLoad(uint32_t beamId);

    /**
     * \brief Detect anomalies using z-score on prediction residuals
     */
    bool DetectAnomaly(uint32_t beamId, double actualLoad);

    /**
     * \brief Compute PRB reservation from prediction and confidence
     */
    double ComputeReservation(const TrafficPrediction& prediction) const;

    /**
     * \brief Internal linear prediction fallback (when AI not available)
     */
    std::vector<double> LinearExtrapolation(const std::deque<double>& history,
                                             uint32_t horizon) const;

    Ptr<OranNtnSatBridge> m_satBridge;
    Ptr<OranNtnGymPredictive> m_gymEnv;

    uint32_t m_predictionHorizon;
    uint32_t m_historyWindow;
    double m_reservationAggression;
    double m_anomalyThreshold;
    bool m_aiEnabled;

    // Per-beam traffic history
    struct BeamTrafficHistory
    {
        std::deque<double> loadHistory;
        std::deque<double> sinrHistory;
        std::deque<double> ueCountHistory;
        TrafficPrediction lastPrediction;
        double residualMean;
        double residualStd;
        bool anomalyActive;
    };
    std::map<uint32_t, BeamTrafficHistory> m_beamHistories;

    PredictiveMetrics m_predMetrics;
};

} // namespace ns3

#endif // ORAN_NTN_XAPP_PREDICTIVE_ALLOC_H
