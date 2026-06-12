/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-only
 * Author: Muhammad Uzair
 *
 * ISAC (Integrated Sensing and Communication) xApp
 *
 * Manages the sensing/communication resource split for THz-NTN links.
 * Processes debris detection alerts and adjusts the ISAC resource
 * partition based on traffic load: more sensing when traffic is low,
 * more communication when traffic is high.
 *
 * Novel features:
 *   - Adaptive sensing/communication resource partitioning
 *   - Space debris detection and alert processing
 *   - Traffic-aware ISAC mode switching
 *   - Joint radar-communication waveform selection
 */

#ifndef ORAN_NTN_XAPP_ISAC_H
#define ORAN_NTN_XAPP_ISAC_H

#include "oran-ntn-xapp-base.h"

#include <map>
#include <vector>

namespace ns3
{

/**
 * \ingroup oran-ntn
 * \brief ISAC xApp for integrated sensing and communication management
 *
 * \note Currently exercised by unit tests only.
 */
class OranNtnXappIsac : public OranNtnXappBase
{
  public:
    static TypeId GetTypeId();
    OranNtnXappIsac();
    ~OranNtnXappIsac() override;

    /**
     * \brief Generate RC actions based on current ISAC state
     * \return vector of E2RcAction to submit
     */
    std::vector<E2RcAction> GenerateRcActions();

    // ---- Metrics ----
    struct IsacMetrics
    {
        uint32_t modeChanges;
        uint32_t debrisAlerts;
        double avgSensingRatio;
        double avgCommRatio;
        double minDebrisRange_km;
    };
    IsacMetrics GetIsacMetrics() const;

  protected:
    void ProcessKpmReport(const E2KpmReport& report) override;
    void DecisionCycle() override;
    E2Subscription GetRequiredSubscription() const override;

  private:
    /**
     * \brief Compute optimal sensing/communication resource split
     * \return sensing ratio (0-1), remainder goes to communication
     */
    double ComputeOptimalSensingRatio(uint32_t gnbId) const;

    /**
     * \brief Process debris detection event
     */
    void ProcessDebrisAlert(uint32_t gnbId, double range_km);

    // Thresholds
    double m_highTrafficThreshold;     //!< PRB utilization above this = high traffic
    double m_lowTrafficThreshold;      //!< PRB utilization below this = low traffic
    double m_debrisAlertRange_km;      //!< Alert when debris closer than this
    double m_minSensingRatio;          //!< Minimum sensing resource fraction
    double m_maxSensingRatio;          //!< Maximum sensing resource fraction

    // Per-gNB ISAC state
    struct IsacState
    {
        bool sensingActive;
        double debrisDetectionRange_km;
        double prbUtilization;
        double currentSensingRatio;
        double thzSnr_dB;
        std::string activeWaveform;
    };
    std::map<uint32_t, IsacState> m_isacStates;

    // Pending actions
    std::vector<E2RcAction> m_pendingActions;

    IsacMetrics m_metrics;
};

} // namespace ns3

#endif // ORAN_NTN_XAPP_ISAC_H
