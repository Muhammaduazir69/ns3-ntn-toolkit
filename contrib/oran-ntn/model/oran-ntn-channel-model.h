/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-only
 * Author: Muhammad Uzair
 *
 * NTN Composite Channel Model
 *
 * Combines satellite module fading models with mmWave propagation for
 * a unified NTN channel that accounts for:
 *   - Free-space path loss (SatFreeSpaceLoss)
 *   - Loo/Markov fading (SatLooModel/SatMarkovModel)
 *   - Atmospheric attenuation for Ka-band (rain, gas, scintillation)
 *   - Satellite antenna gain patterns (SatAntennaGainPattern)
 *   - 3GPP TR 38.811 NTN propagation model compliance
 *   - Clutter loss for different environments
 *
 * Novel features:
 *   - Elevation-dependent Rician K-factor adaptation
 *   - Dynamic LOS/NLOS probability from Markov state
 *   - Frequency-dependent atmospheric model (ITU-R P.676/P.618)
 *   - Correlated shadow fading across satellite beams
 *   - Time-varying channel from orbital dynamics
 */

#ifndef ORAN_NTN_CHANNEL_MODEL_H
#define ORAN_NTN_CHANNEL_MODEL_H

#include <ns3/propagation-loss-model.h>
#include <ns3/traced-callback.h>

#include <map>
#include <random>

namespace ns3
{

class OranNtnSatBridge;

/**
 * \ingroup oran-ntn
 * \brief Composite NTN channel model combining satellite and mmWave propagation
 */
class OranNtnChannelModel : public PropagationLossModel
{
  public:
    static TypeId GetTypeId();
    OranNtnChannelModel();
    ~OranNtnChannelModel() override;

    void SetSatBridge(Ptr<OranNtnSatBridge> bridge);

    /**
     * \brief Set frequency band
     * \param band "S-band" (2 GHz), "Ka-band" (20/30 GHz), "L-band" (1.5 GHz)
     */
    void SetBand(const std::string& band);

    /**
     * \brief Set propagation environment
     * \param env "urban", "suburban", "rural", "dense-urban", "maritime", "aeronautical"
     */
    void SetEnvironment(const std::string& env);

    /**
     * \brief Enable/disable atmospheric attenuation
     */
    void SetAtmosphericAttenuation(bool enable);

    /**
     * \brief Enable/disable Loo fading model
     */
    void SetLooFading(bool enable);

    /**
     * \brief Enable/disable Markov state transitions (clear/shadow/blocked)
     */
    void SetMarkovFading(bool enable);

    /**
     * \brief Enable correlated shadow fading across beams
     */
    void SetCorrelatedShadowFading(bool enable);

    /**
     * \brief Set rain attenuation parameters for Ka-band
     * \param rainRate_mm_h Rain rate in mm/h (ITU-R P.837)
     */
    void SetRainRate(double rainRate_mm_h);

    /**
     * \brief Get the current Markov state for a UE-satellite pair
     * \return 0=clear, 1=shadow, 2=blocked
     */
    uint8_t GetMarkovState(uint32_t ueId, uint32_t satId) const;

    /**
     * \brief Get the current fading gain for a link
     */
    double GetFadingGain_dB(uint32_t ueId, uint32_t satId) const;

    /**
     * \brief Compute Rician K-factor based on elevation angle
     *
     * Higher elevation → higher K (stronger LOS component).
     * Per 3GPP TR 38.811 Table 6.7.2-1.
     */
    double ComputeRicianKFactor(double elevationDeg) const;

    // ---- Trace sources ----
    TracedCallback<uint32_t, uint32_t, double> m_pathLossComputed;    //!< ueId, satId, PL
    TracedCallback<uint32_t, uint32_t, uint8_t> m_markovTransition;   //!< ueId, satId, newState
    TracedCallback<uint32_t, uint32_t, double> m_fadingComputed;      //!< ueId, satId, gain_dB

  protected:
    void DoDispose() override;

    /**
     * \brief Core channel computation combining all components
     */
    double DoCalcRxPower(double txPowerDbm,
                         Ptr<MobilityModel> a,
                         Ptr<MobilityModel> b) const override;

    int64_t DoAssignStreams(int64_t stream) override;

  private:
    /**
     * \brief Compute free-space path loss
     */
    double ComputeFreeSpaceLoss(double distanceM, double freqHz) const;

    /**
     * \brief Compute atmospheric attenuation (gas + rain + scintillation)
     *
     * Based on ITU-R P.676 (gaseous), P.618 (rain), P.531 (scintillation)
     */
    double ComputeAtmosphericLoss(double elevationDeg, double freqHz) const;

    /**
     * \brief Compute gaseous absorption loss (ITU-R P.676)
     */
    double ComputeGaseousLoss(double elevationDeg, double freqHz) const;

    /**
     * \brief Compute rain attenuation (ITU-R P.618)
     */
    double ComputeRainLoss(double elevationDeg, double freqHz) const;

    /**
     * \brief Compute scintillation loss (ITU-R P.531)
     */
    double ComputeScintillationLoss(double elevationDeg, double freqHz) const;

    /**
     * \brief Compute Loo fading gain
     *
     * Three-state model: direct, scattered, blocked components
     * Parameters depend on elevation and environment.
     */
    double ComputeLooFading(double elevationDeg, const std::string& env) const;

    /**
     * \brief Update Markov state for a UE-satellite link
     */
    void UpdateMarkovState(uint32_t ueId, uint32_t satId, double elevationDeg) const;

    /**
     * \brief Compute clutter loss for different environments
     */
    double ComputeClutterLoss(double elevationDeg, const std::string& env) const;

    /**
     * \brief Compute shadow fading with inter-beam correlation
     */
    double ComputeCorrelatedShadowFading(uint32_t ueId, uint32_t satId,
                                          uint32_t beamId) const;

    // Configuration
    Ptr<OranNtnSatBridge> m_satBridge;
    std::string m_band;
    std::string m_environment;
    double m_centerFreqHz;
    bool m_atmosphericAttenuation;
    bool m_looFading;
    bool m_markovFading;
    bool m_correlatedShadowFading;
    double m_rainRate_mm_h;

    // Markov state per (ueId, satId) pair
    struct MarkovLinkState
    {
        uint8_t currentState;         //!< 0=clear, 1=shadow, 2=blocked
        double lastTransitionTime;
        double fadingGain_dB;
        double shadowFading_dB;
    };
    mutable std::map<uint64_t, MarkovLinkState> m_markovStates;

    // Random number generation
    mutable std::mt19937 m_rng;
    mutable std::normal_distribution<double> m_normalDist;
    mutable std::uniform_real_distribution<double> m_uniformDist;
};

} // namespace ns3

#endif // ORAN_NTN_CHANNEL_MODEL_H
