/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-only
 * Author: Muhammad Uzair
 *
 * Predictive Resource Allocation xApp
 *
 * Traffic prediction for proactive PRB reservation. Uses historical KPM
 * time series per beam to forecast demand and pre-allocate resources before
 * congestion occurs.
 *
 * NOTE on the "LSTM" label: no recurrent neural network is implemented. The
 * predictor is a linear extrapolation over the history window (optionally
 * routed to a Gym env when `UseAiPrediction` is set); the "LSTM" name marks
 * the intended model slot, not the implemented method. No trained model ships.
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
 *
 * \note Currently exercised by unit tests only. The SetGymEnv() RL hook
 *       (OranNtnGymPredictive) has no in-tree caller yet.
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

    /// AI-10: number of predictions where the AI branch was taken and produced
    /// nothing, so linear extrapolation was used. See the comment in
    /// PredictTrafficLoad(); the gym environment is asynchronous and has no
    /// synchronous inference call for that method to make.
    uint64_t GetAiFallbackCount() const { return m_aiFallbackCount; }

    /**
     * \brief Predicted load for a beam over the configured horizon.
     *
     * AI-10: public because it is the computation whose AI branch produces
     * nothing, and a test that cannot call it can only observe the defect
     * indirectly through a timer-driven decision cycle. It is a query over
     * recorded history with no side effect beyond the fallback counter.
     */
    std::vector<double> PredictTrafficLoad(uint32_t beamId);

    /// AI-10: feed one load sample for a beam, so the prediction path can be
    /// exercised without standing up a RIC and a subscription.
    void RecordBeamLoadForTest(uint32_t beamId, double load);

  protected:
    void ProcessKpmReport(const E2KpmReport& report) override;
    void DecisionCycle() override;
    E2Subscription GetRequiredSubscription() const override;

  private:
    /**
     * \brief Run LSTM prediction (via Gym env or internal model)
     */


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
    /// AI-10: how many times the AI branch was entered and produced nothing.
    ///
    /// Non-zero means AiEnabled was set, a gym environment was attached, and
    /// every prediction still came from linear extrapolation. Nothing outside
    /// could previously tell those two configurations apart.
    uint64_t m_aiFallbackCount{0};
    bool m_aiFallbackWarned{false};

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
