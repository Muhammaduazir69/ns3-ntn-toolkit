/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-only
 * Author: Muhammad Uzair
 *
 * THz Beam Management xApp
 *
 * Monitors THz beam tracking error and SNR, triggers hierarchical beam
 * search when pointing error exceeds threshold, coordinates beam handover
 * during LEO pass using TTE, and uses orbital prediction for proactive
 * beam pre-steering.
 *
 * Novel features:
 *   - Hierarchical beam search with narrow THz beams
 *   - TTE-aware proactive beam pre-steering using orbital prediction
 *   - Beam squint compensation for wideband THz signals
 *   - UM-MIMO element coordination for pencil beam tracking
 */

#ifndef ORAN_NTN_XAPP_THZ_BEAM_MGMT_H
#define ORAN_NTN_XAPP_THZ_BEAM_MGMT_H

#include "oran-ntn-xapp-base.h"

#include <map>
#include <vector>

namespace ns3
{

/**
 * \ingroup oran-ntn
 * \brief THz Beam Management xApp for NTN pencil-beam tracking
 *
 * \warning Experimental: not yet exercised by any example or test; API may
 *          change.
 */
class OranNtnXappThzBeamMgmt : public OranNtnXappBase
{
  public:
    static TypeId GetTypeId();
    OranNtnXappThzBeamMgmt();
    ~OranNtnXappThzBeamMgmt() override;

    /**
     * \brief Generate RC actions based on current beam state
     * \return vector of E2RcAction to submit
     */
    std::vector<E2RcAction> GenerateRcActions();

    // ---- Metrics ----
    struct ThzBeamMetrics
    {
        uint32_t beamSearchesTriggered;
        uint32_t beamHandovers;
        uint32_t proactivePreSteers;
        double avgPointingError_deg;
        double avgThzSnr_dB;
    };
    ThzBeamMetrics GetThzBeamMetrics() const;

  protected:
    void ProcessKpmReport(const E2KpmReport& report) override;
    void DecisionCycle() override;
    E2Subscription GetRequiredSubscription() const override;

  private:
    /**
     * \brief Evaluate if beam search is needed based on pointing error
     */
    bool NeedsBeamSearch(uint32_t gnbId) const;

    /**
     * \brief Compute proactive pre-steer beam index from orbital prediction
     */
    uint32_t ComputePreSteerBeamIndex(uint32_t gnbId, double tte_s) const;

    /**
     * \brief Estimate beam squint compensation offset
     */
    double EstimateBeamSquintOffset(double centerFreq_GHz, double bandwidth_GHz) const;

    // Thresholds
    double m_pointingErrorThreshold_deg;  //!< Trigger beam search above this
    double m_snrThreshold_dB;             //!< Minimum acceptable THz SNR
    double m_ttePreSteerThreshold_s;      //!< Pre-steer when TTE below this
    double m_beamSquintThreshold_dB;      //!< Max tolerable beam squint loss

    // Per-gNB latest THz beam state
    struct ThzBeamState
    {
        double pointingError_deg;
        double thzSnr_dB;
        double tte_s;
        double beamSquintLoss_dB;
        uint32_t currentBeamId;
        uint16_t umMimoElements;
        double centerFreq_GHz;
    };
    std::map<uint32_t, ThzBeamState> m_beamStates;

    // Pending actions from last decision cycle
    std::vector<E2RcAction> m_pendingActions;

    ThzBeamMetrics m_metrics;
};

} // namespace ns3

#endif // ORAN_NTN_XAPP_THZ_BEAM_MGMT_H
