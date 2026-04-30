/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-only
 * Author: Muhammad Uzair
 *
 * Doppler Compensation xApp - Per-beam frequency offset correction for NTN
 *
 * Computes and applies Doppler pre-compensation at satellite gNBs.
 * Handles common Doppler (beam-center reference) and residual Doppler
 * (per-UE correction) per 3GPP TS 38.821 Section 6.1.4.
 *
 * Supports both satellite-side and UE-side compensation strategies.
 */

#ifndef ORAN_NTN_XAPP_DOPPLER_COMP_H
#define ORAN_NTN_XAPP_DOPPLER_COMP_H

#include "oran-ntn-xapp-base.h"

#include <map>
#include <vector>

namespace ns3
{

/**
 * \ingroup oran-ntn
 * \brief Doppler Compensation xApp for NTN frequency management
 */
class OranNtnXappDopplerComp : public OranNtnXappBase
{
  public:
    static TypeId GetTypeId();
    OranNtnXappDopplerComp();
    ~OranNtnXappDopplerComp() override;

    // ---- Configuration ----
    void SetCarrierFrequency(double freqHz);
    void SetCompensationMode(const std::string& mode); // "common", "per-ue", "hybrid"
    void SetUpdateInterval(Time interval);
    void SetMaxResidualDoppler(double Hz);

    // ---- Query ----
    double GetCommonDopplerShift(uint32_t gnbId) const;
    double GetResidualDoppler(uint32_t gnbId, uint32_t ueId) const;

    // ---- Metrics ----
    struct DopplerMetrics
    {
        uint32_t compensationUpdates;
        double avgCommonDoppler_Hz;
        double maxCommonDoppler_Hz;
        double avgResidualDoppler_Hz;
        double maxResidualDoppler_Hz;
        double avgDopplerRate_Hz_per_s;
    };

    DopplerMetrics GetDopplerMetrics() const;

    TracedCallback<uint32_t, double, double> m_dopplerUpdated;
    //!< gnbId, commonDoppler, maxResidual

  protected:
    void ProcessKpmReport(const E2KpmReport& report) override;
    void DecisionCycle() override;
    E2Subscription GetRequiredSubscription() const override;

  private:
    struct GnbDopplerState
    {
        double commonDopplerShift;         //!< Hz
        double dopplerRate;                //!< Hz/s
        std::map<uint32_t, double> perUeResidual;  //!< ueId -> residual Hz
        double lastUpdateTime;
    };

    void ComputeCommonDoppler(uint32_t gnbId);
    void ComputePerUeResidual(uint32_t gnbId);

    std::map<uint32_t, GnbDopplerState> m_gnbDopplerStates;

    double m_carrierFrequency;
    std::string m_compensationMode;
    Time m_updateInterval;
    double m_maxResidualDoppler;

    DopplerMetrics m_dopplerMetrics;
};

} // namespace ns3

#endif // ORAN_NTN_XAPP_DOPPLER_COMP_H
