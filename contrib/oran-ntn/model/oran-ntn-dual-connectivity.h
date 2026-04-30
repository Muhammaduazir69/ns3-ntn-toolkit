/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-only
 * Author: Muhammad Uzair
 *
 * NTN Dual Connectivity Manager
 *
 * Manages dual connectivity between terrestrial mmWave and NTN satellite
 * using McUeNetDevice for simultaneous TN + NTN bearer support.
 *
 * Novel features:
 *   - Dynamic split bearer ratio adjustment based on link quality
 *   - Seamless primary path switching between TN and NTN
 *   - O-RAN xApp-driven DC activation/teardown decisions
 *   - QoS-aware bearer splitting per 3GPP TS 37.340
 *   - Predictive DC setup before handover events
 *   - Battery-aware DC management for UE power saving
 */

#ifndef ORAN_NTN_DUAL_CONNECTIVITY_H
#define ORAN_NTN_DUAL_CONNECTIVITY_H

#include "oran-ntn-types.h"

#include <ns3/object.h>
#include <ns3/traced-callback.h>

#include <map>

namespace ns3
{

namespace mmwave
{
class MmWaveHelper;
class McUeNetDevice;
} // namespace mmwave

class OranNtnSatBridge;

/**
 * \ingroup oran-ntn
 * \brief Dual Connectivity manager for TN-NTN networks
 */
class OranNtnDualConnectivity : public Object
{
  public:
    static TypeId GetTypeId();
    OranNtnDualConnectivity();
    ~OranNtnDualConnectivity() override;

    /**
     * \brief Initialize dual connectivity infrastructure
     */
    void Initialize(Ptr<OranNtnSatBridge> satBridge);

    /**
     * \brief Activate dual connectivity for a UE
     * \return true if DC was successfully activated
     */
    bool ActivateDualConnectivity(uint32_t ueId, uint32_t tnGnbId,
                                   uint32_t ntnSatId, uint32_t ntnBeamId);

    /**
     * \brief Deactivate dual connectivity for a UE
     */
    void DeactivateDualConnectivity(uint32_t ueId);

    /**
     * \brief Update bearer split ratio
     * \param tnRatio Fraction routed via terrestrial (0-1)
     */
    void UpdateSplitRatio(uint32_t ueId, double tnRatio);

    /**
     * \brief Switch primary path between TN and NTN
     * \param primary 0=TN, 1=NTN
     */
    void SwitchPrimaryPath(uint32_t ueId, uint8_t primary);

    /**
     * \brief Check if UE is in dual connectivity mode
     */
    bool IsDualConnected(uint32_t ueId) const;

    /**
     * \brief Get DC session info for a UE
     */
    DualConnSession GetSession(uint32_t ueId) const;

    /**
     * \brief Get terrestrial link quality for a UE
     */
    double GetTnLinkQuality(uint32_t ueId) const;

    /**
     * \brief Get NTN link quality for a UE
     */
    double GetNtnLinkQuality(uint32_t ueId) const;

    /**
     * \brief Evaluate whether DC would benefit a UE
     *
     * Returns benefit score (>0 means DC is beneficial):
     * considers both link SINRs, latency requirements, UE battery.
     */
    double EvaluateDcBenefit(uint32_t ueId) const;

    /**
     * \brief Get all active DC sessions
     */
    std::vector<DualConnSession> GetAllSessions() const;

    // ---- Metrics ----
    struct DcMetrics
    {
        uint32_t totalActivations;
        uint32_t totalDeactivations;
        uint32_t primarySwitches;
        double avgSessionDuration_s;
        double avgSplitRatio;
        double avgThroughputGain_pct;
    };
    DcMetrics GetMetrics() const;

    // ---- Trace sources ----
    TracedCallback<uint32_t, bool> m_dcStateChanged;      //!< ueId, activated
    TracedCallback<uint32_t, double> m_splitRatioChanged;  //!< ueId, newRatio
    TracedCallback<uint32_t, uint8_t> m_primarySwitched;   //!< ueId, newPrimary

  protected:
    void DoDispose() override;

  private:
    Ptr<OranNtnSatBridge> m_satBridge;
    std::map<uint32_t, DualConnSession> m_sessions;
    DcMetrics m_metrics;

    double m_minSinrForDc_dB;     //!< Minimum SINR on both links for DC
    double m_splitHysteresis;      //!< Hysteresis for split ratio changes
};

} // namespace ns3

#endif // ORAN_NTN_DUAL_CONNECTIVITY_H
