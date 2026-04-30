/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-only
 * Author: Muhammad Uzair
 *
 * HO Prediction xApp - Proactive handover using LSTM/DQN inference
 *
 * Monitors RSRP/SINR trends and TTE predictions from all NTN gNBs,
 * detects imminent handover needs, and triggers proactive handovers
 * before quality degradation. Uses AI model (LSTM for trend prediction,
 * DQN for action selection) via ns3-ai shared memory interface.
 *
 * Key features:
 *   - SINR trend analysis (linear regression over sliding window)
 *   - TTE-aware candidate ranking (reuses NTN-CHO algorithm)
 *   - Proactive HO triggering (acts before quality drops below threshold)
 *   - AI-assisted decision (DQN/LSTM via Python agent)
 *   - Ping-pong avoidance (hysteresis + recent-HO tracking)
 */

#ifndef ORAN_NTN_XAPP_HO_PREDICT_H
#define ORAN_NTN_XAPP_HO_PREDICT_H

#include "oran-ntn-xapp-base.h"

#include <ns3/nstime.h>

#include <deque>
#include <map>
#include <vector>

namespace ns3
{

/**
 * \ingroup oran-ntn
 * \brief HO Prediction xApp for proactive NTN handover
 */
class OranNtnXappHoPredict : public OranNtnXappBase
{
  public:
    static TypeId GetTypeId();
    OranNtnXappHoPredict();
    ~OranNtnXappHoPredict() override;

    // ---- Configuration ----
    void SetSinrDropThreshold(double dB);
    void SetTteMinimum(Time tte);
    void SetProactiveLeadTime(Time lead);
    void SetPingPongGuardTime(Time guard);
    void SetTrendWindowSize(uint32_t samples);

    // ---- AI model interface ----
    /**
     * \brief Set model weights for DQN action selection
     */
    void SetDqnWeights(const std::vector<double>& weights);

    /**
     * \brief Set model weights for LSTM trend prediction
     */
    void SetLstmWeights(const std::vector<double>& weights);

    /**
     * \brief Enable/disable AI-assisted mode (vs. rule-based)
     */
    void SetAiEnabled(bool enabled);

    // ---- Metrics ----
    struct HoPredictMetrics
    {
        uint32_t proactiveHandovers;
        uint32_t reactiveHandovers;
        uint32_t successfulPredictions;
        uint32_t falsePredictions;
        uint32_t pingPongsAvoided;
        double avgLeadTime_s;
        double avgPredictionAccuracy;
    };

    HoPredictMetrics GetHoPredictMetrics() const;

    TracedCallback<uint32_t, uint32_t, double, double> m_hoTriggered;
    //!< ueId, targetGnb, SINR, TTE

  protected:
    void ProcessKpmReport(const E2KpmReport& report) override;
    void DecisionCycle() override;
    E2Subscription GetRequiredSubscription() const override;

  private:
    // SINR trend analysis
    struct SinrTrend
    {
        double slope;              //!< dB/s
        double intercept;
        double r_squared;          //!< Goodness of fit
        double predictedSinr;      //!< Predicted SINR at lead time
        Time timeToThreshold;      //!< When SINR will reach threshold
    };

    SinrTrend ComputeSinrTrend(uint32_t ueId) const;

    // DQN inference (simplified on-board)
    struct HoFeatureVector
    {
        double servingSinr;
        double servingTte;
        double servingElevation;
        double servingDoppler;
        double sinrSlope;
        double bestCandSinr;
        double bestCandTte;
        double bestCandElevation;
        double timeSinceLastHo;
        uint32_t recentHoCount;
    };

    uint32_t DqnSelectAction(const HoFeatureVector& features) const;
    double LstmPredictSinr(uint32_t ueId, Time horizon) const;

    // Per-UE state
    struct UeHoState
    {
        std::deque<std::pair<double, double>> sinrHistory; //!< (time, sinr)
        Time lastHandoverTime;
        uint32_t recentHandovers;
        uint32_t servingGnbId;
        bool handoverPending;
    };

    std::map<uint32_t, UeHoState> m_ueStates;

    // Configuration
    double m_sinrDropThreshold;     //!< SINR threshold for HO (dB)
    Time m_tteMinimum;              //!< Min TTE for candidate
    Time m_proactiveLeadTime;       //!< How far ahead to predict
    Time m_pingPongGuardTime;       //!< Min time between HOs
    uint32_t m_trendWindowSize;     //!< Samples for trend analysis

    // AI model
    bool m_aiEnabled;
    std::vector<double> m_dqnWeights;
    std::vector<double> m_lstmWeights;

    // Metrics
    HoPredictMetrics m_hoPredMetrics;
};

} // namespace ns3

#endif // ORAN_NTN_XAPP_HO_PREDICT_H
