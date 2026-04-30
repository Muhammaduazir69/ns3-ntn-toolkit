/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-only
 * Author: Muhammad Uzair
 *
 * Multi-Connectivity Orchestrator xApp
 *
 * Manages dual/multi-connectivity across TN-NTN using mmWave McUeNetDevice.
 * Decides when to activate/teardown DC, controls bearer split ratios,
 * and optimizes primary path selection.
 *
 * Novel features:
 *   - Predictive DC setup triggered by upcoming satellite handover
 *   - QoS-differentiated splitting (URLLC via TN, eMBB split)
 *   - Battery-aware DC management (DC costs extra UE power)
 *   - Joint optimization of DC with beam hopping schedule
 *   - Seamless fallback: auto-switch to single path on link failure
 *   - Load-aware split: shift traffic to less loaded path dynamically
 */

#ifndef ORAN_NTN_XAPP_MULTI_CONN_H
#define ORAN_NTN_XAPP_MULTI_CONN_H

#include "oran-ntn-xapp-base.h"

#include <map>

namespace ns3
{

class OranNtnSatBridge;
class OranNtnDualConnectivity;

/**
 * \ingroup oran-ntn
 * \brief Multi-Connectivity Orchestrator xApp
 */
class OranNtnXappMultiConn : public OranNtnXappBase
{
  public:
    static TypeId GetTypeId();
    OranNtnXappMultiConn();
    ~OranNtnXappMultiConn() override;

    void SetSatBridge(Ptr<OranNtnSatBridge> bridge);
    void SetDualConnManager(Ptr<OranNtnDualConnectivity> dcMgr);

    void SetMinSinrForDc(double minSinr_dB);
    void SetDcBenefitThreshold(double threshold);
    void SetBatteryAware(bool enable);
    void SetPredictiveDc(bool enable);

    // ---- Metrics ----
    struct MultiConnMetrics
    {
        uint32_t dcActivations;
        uint32_t dcDeactivations;
        uint32_t splitRatioAdjustments;
        uint32_t primarySwitches;
        uint32_t predictiveDcSetups;
        uint32_t fallbackEvents;
        double avgDcDuration_s;
        double avgThroughputGain_pct;
        double avgLatencyReduction_pct;
    };
    MultiConnMetrics GetMultiConnMetrics() const;

  protected:
    void ProcessKpmReport(const E2KpmReport& report) override;
    void DecisionCycle() override;
    E2Subscription GetRequiredSubscription() const override;

  private:
    /**
     * \brief Evaluate whether DC should be activated for a UE
     */
    bool ShouldActivateDc(uint32_t ueId) const;

    /**
     * \brief Evaluate whether DC should be deactivated
     */
    bool ShouldDeactivateDc(uint32_t ueId) const;

    /**
     * \brief Compute optimal split ratio based on link qualities
     */
    double ComputeOptimalSplitRatio(uint32_t ueId) const;

    /**
     * \brief Check if upcoming HO warrants predictive DC setup
     */
    bool PredictiveDcNeeded(uint32_t ueId) const;

    /**
     * \brief Select primary path based on QoS requirements
     */
    uint8_t SelectPrimaryPath(uint32_t ueId) const;

    Ptr<OranNtnSatBridge> m_satBridge;
    Ptr<OranNtnDualConnectivity> m_dcMgr;

    double m_minSinrForDc_dB;
    double m_dcBenefitThreshold;
    bool m_batteryAware;
    bool m_predictiveDc;

    // Per-UE tracking
    struct UeDcState
    {
        double tnSinr_dB;
        double ntnSinr_dB;
        double tnLoad;
        double ntnLoad;
        double tnThroughput;
        double ntnThroughput;
        double currentSplitRatio;
        bool dcActive;
        double lastEvaluationTime;
    };
    std::map<uint32_t, UeDcState> m_ueStates;

    MultiConnMetrics m_mcMetrics;
};

} // namespace ns3

#endif // ORAN_NTN_XAPP_MULTI_CONN_H
