/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-only
 * Author: Muhammad Uzair
 *
 * THz RIS Management xApp
 *
 * Monitors RIS-assisted gain and THz SNR to trigger phase profile
 * reconfiguration when gain drops below threshold. Coordinates
 * multiple RIS panels for optimal THz link enhancement.
 *
 * Novel features:
 *   - Adaptive RIS phase profile reconfiguration for THz bands
 *   - Multi-panel RIS coordination for coverage extension
 *   - SNR-driven RIS gain optimization
 *   - Integration with THz beam management for joint optimization
 */

#ifndef ORAN_NTN_XAPP_THZ_RIS_H
#define ORAN_NTN_XAPP_THZ_RIS_H

#include "oran-ntn-xapp-base.h"

#include <map>
#include <vector>

namespace ns3
{

/**
 * \ingroup oran-ntn
 * \brief THz RIS Management xApp for reconfigurable surface control
 */
class OranNtnXappThzRis : public OranNtnXappBase
{
  public:
    static TypeId GetTypeId();
    OranNtnXappThzRis();
    ~OranNtnXappThzRis() override;

    /**
     * \brief Generate RC actions based on current RIS state
     * \return vector of E2RcAction to submit
     */
    std::vector<E2RcAction> GenerateRcActions();

    // ---- Metrics ----
    struct ThzRisMetrics
    {
        uint32_t risReconfigurations;
        double avgRisGain_dB;
        double avgThzSnr_dB;
        double maxGainImprovement_dB;
    };
    ThzRisMetrics GetThzRisMetrics() const;

  protected:
    void ProcessKpmReport(const E2KpmReport& report) override;
    void DecisionCycle() override;
    E2Subscription GetRequiredSubscription() const override;

  private:
    /**
     * \brief Compute optimal RIS phase profile index for current conditions
     */
    uint32_t ComputeOptimalPhaseProfile(uint32_t gnbId) const;

    /**
     * \brief Estimate achievable gain from RIS reconfiguration
     */
    double EstimateAchievableGain(double currentGain_dB,
                                  double thzSnr_dB,
                                  double elevation_deg) const;

    // Thresholds
    double m_risGainThreshold_dB;      //!< Trigger reconfig when gain below this
    double m_snrImprovementTarget_dB;  //!< Target SNR improvement from RIS
    uint32_t m_numRisPanels;           //!< Number of RIS panels per satellite

    // Per-gNB RIS state
    struct ThzRisState
    {
        double risGain_dB;
        double thzSnr_dB;
        double elevation_deg;
        double thzCenterFreq_GHz;
        uint32_t currentPhaseProfile;
        double previousGain_dB;
    };
    std::map<uint32_t, ThzRisState> m_risStates;

    // Pending actions
    std::vector<E2RcAction> m_pendingActions;

    ThzRisMetrics m_metrics;
};

} // namespace ns3

#endif // ORAN_NTN_XAPP_THZ_RIS_H
