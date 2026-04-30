/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-only
 * Author: Muhammad Uzair
 *
 * NTN-Aware mmWave Beamforming Model
 *
 * Extends MmWaveBeamformingModel for satellite NTN scenarios with:
 *   - Wide beam patterns (3-5° vs 10-15° terrestrial)
 *   - Elevation-dependent beam steering using satellite geometry
 *   - Doppler-aware beam squint correction per 3GPP TR 38.821
 *   - Periodic beam tracking to compensate LEO satellite motion
 *   - Multi-beam satellite antenna array modeling
 *   - Adaptive beam width based on orbital altitude and elevation
 *
 * Novel features:
 *   - Predictive beam steering using SGP4 orbit propagation
 *   - Cross-beam interference-aware null steering
 *   - Elevation-adaptive codebook selection
 *   - Doppler-compensated beam squint pre-correction
 */

#ifndef ORAN_NTN_MMWAVE_BEAMFORMING_H
#define ORAN_NTN_MMWAVE_BEAMFORMING_H

#include <ns3/mmwave-beamforming-model.h>
#include <ns3/nstime.h>
#include <ns3/object.h>
#include <ns3/traced-callback.h>

#include <complex>
#include <map>
#include <vector>

namespace ns3
{

class OranNtnSatBridge;

/**
 * \ingroup oran-ntn
 * \brief NTN-aware beamforming model for satellite mmWave
 *
 * Accounts for satellite-specific geometry: wide beams, elevation-dependent
 * steering, Doppler-induced beam squint, and LEO orbital motion.
 */
class OranNtnMmWaveBeamforming : public mmwave::MmWaveBeamformingModel
{
  public:
    static TypeId GetTypeId();
    OranNtnMmWaveBeamforming();
    ~OranNtnMmWaveBeamforming() override;

    /**
     * \brief Override: compute beamforming vector for NTN link
     *
     * Uses satellite elevation/azimuth from SatBridge, applies
     * Doppler squint correction, and accounts for wide NTN beams.
     */
    void SetBeamformingVectorForDevice(
        Ptr<NetDevice> otherDevice,
        Ptr<PhasedArrayModel> otherAntenna) override;

    // ---- NTN configuration ----

    void SetSatBridge(Ptr<OranNtnSatBridge> bridge);
    void SetSatelliteId(uint32_t satId);

    /**
     * \brief Set NTN beam width (wider than terrestrial)
     * \param elevDeg Elevation half-power beamwidth in degrees
     * \param azimDeg Azimuth half-power beamwidth in degrees
     */
    void SetBeamWidth(double elevDeg, double azimDeg);

    /**
     * \brief Set beam tracking update interval
     */
    void SetTrackingInterval(Time interval);

    /**
     * \brief Enable Doppler-aware beam squint correction
     */
    void SetDopplerSquintCorrection(bool enable);

    /**
     * \brief Enable predictive beam steering using SGP4 extrapolation
     */
    void SetPredictiveSteering(bool enable);

    /**
     * \brief Set prediction horizon for predictive steering
     */
    void SetPredictionHorizon(Time horizon);

    /**
     * \brief Enable cross-beam interference null steering
     */
    void SetInterferenceNullSteering(bool enable);

    /**
     * \brief Set carrier frequency for wavelength computation
     */
    void SetCarrierFrequency(double freqHz);

    /**
     * \brief Manually trigger beam tracking update
     */
    void UpdateBeamTracking();

    /**
     * \brief Get current beam tracking error for a UE
     */
    double GetBeamTrackingError(uint32_t ueId) const;

    /**
     * \brief Get the number of active beams
     */
    uint32_t GetActiveBeamCount() const;

    // ---- Trace sources ----
    TracedCallback<uint32_t, double, double> m_beamUpdated;  //!< satId, elevation, azimuth
    TracedCallback<uint32_t, double> m_squintCorrected;      //!< ueId, correctionDeg

  protected:
    void DoDispose() override;

  private:
    /**
     * \brief Core NTN beamforming vector computation
     *
     * Computes antenna weight vector for satellite-to-ground link
     * accounting for elevation, azimuth, Doppler squint, and beam width.
     */
    PhasedArrayModel::ComplexVector ComputeNtnBeamformingVector(
        double elevationDeg,
        double azimuthDeg,
        double dopplerHz,
        uint32_t numElements) const;

    /**
     * \brief Compute Doppler-induced beam squint angle
     *
     * Per 3GPP TR 38.821 Sec 6.1.4: beam squint occurs when
     * Doppler shift causes phase progression across the array.
     */
    double ComputeBeamSquint(double dopplerHz, double elevationDeg) const;

    /**
     * \brief Compute elevation-adaptive beam width
     *
     * Lower elevation → wider beam to maintain coverage footprint.
     */
    double ComputeAdaptiveBeamWidth(double elevationDeg) const;

    /**
     * \brief Predict future satellite position for proactive steering
     */
    void PredictiveSteeringUpdate();

    /**
     * \brief Compute null direction for interference avoidance
     */
    double ComputeNullDirection(uint32_t targetBeamId,
                                const std::vector<uint32_t>& interferingBeams) const;

    void PeriodicTrackingCallback();

    // Configuration
    Ptr<OranNtnSatBridge> m_satBridge;
    uint32_t m_satId;
    double m_beamWidthElev_deg;
    double m_beamWidthAzim_deg;
    Time m_trackingInterval;
    bool m_dopplerSquintCorrection;
    bool m_predictiveSteering;
    Time m_predictionHorizon;
    bool m_interferenceNullSteering;
    double m_carrierFreqHz;

    // State
    struct BeamState
    {
        double currentElevation_deg;
        double currentAzimuth_deg;
        double currentDoppler_Hz;
        double trackingError_deg;
        double squintCorrection_deg;
        PhasedArrayModel::ComplexVector weights;
    };
    std::map<uint32_t, BeamState> m_beamStates;  //!< ueId -> beam state

    EventId m_trackingEvent;
};

} // namespace ns3

#endif // ORAN_NTN_MMWAVE_BEAMFORMING_H
