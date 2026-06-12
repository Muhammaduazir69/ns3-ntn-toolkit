/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-only
 * Author: Muhammad Uzair
 *
 * THz Spectrum Management xApp
 *
 * Monitors atmospheric conditions and molecular absorption to dynamically
 * select optimal THz atmospheric windows. Implements distance-aware
 * multi-carrier allocation and coordinates spectrum across the
 * constellation via A1 policies.
 *
 * Novel features:
 *   - Real-time atmospheric window selection based on absorption monitoring
 *   - Distance-aware multi-carrier allocation for THz links
 *   - Constellation-wide spectrum coordination via A1 interface
 *   - Weather-adaptive frequency hopping between THz windows
 */

#ifndef ORAN_NTN_XAPP_THZ_SPECTRUM_H
#define ORAN_NTN_XAPP_THZ_SPECTRUM_H

#include "oran-ntn-xapp-base.h"

#include <map>
#include <vector>

namespace ns3
{

/**
 * \ingroup oran-ntn
 * \brief THz Spectrum Management xApp for atmospheric window selection
 *
 * \warning Experimental: not yet exercised by any example or test; API may
 *          change.
 */
class OranNtnXappThzSpectrum : public OranNtnXappBase
{
  public:
    static TypeId GetTypeId();
    OranNtnXappThzSpectrum();
    ~OranNtnXappThzSpectrum() override;

    /**
     * \brief Generate RC actions based on current spectrum state
     * \return vector of E2RcAction to submit
     */
    std::vector<E2RcAction> GenerateRcActions();

    // ---- Metrics ----
    struct ThzSpectrumMetrics
    {
        uint32_t windowHops;
        uint32_t freqSelections;
        double avgAbsorption_dB;
        double avgAtmosphericWindow_GHz;
    };
    ThzSpectrumMetrics GetThzSpectrumMetrics() const;

  protected:
    void ProcessKpmReport(const E2KpmReport& report) override;
    void DecisionCycle() override;
    E2Subscription GetRequiredSubscription() const override;

  private:
    /**
     * \brief Select the best atmospheric window given current conditions
     * \return center frequency of the best window (GHz)
     */
    double SelectBestAtmosphericWindow(double currentAbsorption_dB,
                                       double elevation_deg) const;

    /**
     * \brief Compute distance-aware carrier allocation
     */
    double ComputeOptimalCarrierFreq(double distance_km,
                                     double elevation_deg) const;

    // Thresholds
    double m_absorptionThreshold_dB;   //!< Trigger window hop above this
    double m_weatherLossThreshold_dB;  //!< Weather loss threshold
    double m_minElevation_deg;         //!< Minimum elevation for THz operation

    // Known atmospheric windows (center frequencies in GHz)
    std::vector<double> m_atmosphericWindows_GHz;

    // Per-gNB spectrum state
    struct ThzSpectrumState
    {
        double molecularAbsorption_dB;
        double atmosphericWindow_GHz;
        double thzCenterFreq_GHz;
        double elevation_deg;
        double propagationDelay_ms;
    };
    std::map<uint32_t, ThzSpectrumState> m_spectrumStates;

    // Pending actions
    std::vector<E2RcAction> m_pendingActions;

    ThzSpectrumMetrics m_metrics;
};

} // namespace ns3

#endif // ORAN_NTN_XAPP_THZ_SPECTRUM_H
