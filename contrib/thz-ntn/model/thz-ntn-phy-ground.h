/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-only
 * Author: Muhammad Uzair
 *
 * Ground Terminal THz-NTN PHY Layer Model
 *
 * Ground terminal specialisation of the THz-NTN physical layer.  Bridges
 * to the satellite module's SatUtPhy for reception and to the mmWave
 * module's CQI feedback pipeline for scheduling compatibility.
 *
 * Integration points:
 *   - SatPhy (SatUtPhy): for satellite link reception
 *   - SatSignalParameters: for Doppler estimation from received signal
 *   - mmWave CQI feedback: generates DlCqiInfo compatible with mmWave scheduler
 *   - SpectrumValue: AGC from actual received power variations
 *   - MobilityModel: ground terminal position for Doppler tracking
 *
 * References:
 *   [1] T. Nagatsuma et al., "Advances in terahertz communications
 *       accelerated by photonics," Nature Photonics, vol. 10, 2016.
 *   [2] I. F. Akyildiz et al., "Terahertz band: Next frontier for
 *       wireless communications," Phys. Commun., vol. 12, 2014.
 *   [3] 3GPP TR 38.811, "Study on New Radio to support non-terrestrial
 *       networks," v15.4.0, 2020.
 */

#ifndef THZ_NTN_PHY_GROUND_H
#define THZ_NTN_PHY_GROUND_H

#include "thz-ntn-phy.h"

#include <ns3/mobility-model.h>
#include <ns3/object.h>
#include <ns3/ptr.h>
#include <ns3/spectrum-value.h>
#include <ns3/traced-callback.h>

#include <string>

namespace ns3
{

// Forward declarations from satellite module
class SatPhy;
class SatSignalParameters;

// Forward declarations from mmwave module
namespace mmwave
{
struct DlCqiInfo;
} // namespace mmwave

/**
 * \ingroup thz-ntn
 * \brief THz receiver architecture type
 */
enum ReceiverType
{
    RX_HETERODYNE = 0,       ///< Coherent heterodyne (IF downconversion)
    RX_DIRECT_DETECTION,     ///< Envelope / direct detection
    RX_HOMODYNE              ///< Coherent homodyne (zero-IF)
};

/**
 * \ingroup thz-ntn
 * \brief Antenna polarization configuration
 */
enum Polarization
{
    POL_SINGLE_H = 0,    ///< Single horizontal linear
    POL_SINGLE_V,        ///< Single vertical linear
    POL_DUAL_LINEAR,     ///< Dual linear (H + V)
    POL_DUAL_CIRCULAR    ///< Dual circular (LHCP + RHCP)
};

/**
 * \ingroup thz-ntn
 * \brief Ground terminal THz-NTN physical layer model with SatUtPhy/mmWave integration
 *
 * Extends the base ThzNtnPhy with ground terminal functionality that
 * interfaces with:
 *   - SatPhy (SatUtPhy) for satellite link reception
 *   - SatSignalParameters for Doppler estimation from received signals
 *   - mmWave CQI pipeline for scheduler-compatible feedback
 *   - SpectrumValue for actual AGC from power variations
 */
class ThzNtnPhyGround : public ThzNtnPhy
{
  public:
    /**
     * \brief Get the ns-3 TypeId for this class
     * \return the TypeId
     */
    static TypeId GetTypeId();

    /** Default constructor */
    ThzNtnPhyGround();

    /** Destructor */
    ~ThzNtnPhyGround() override;

    /** Dispose */
    void DoDispose() override;

    // ---------------------------------------------------------------
    // SatUtPhy integration
    // ---------------------------------------------------------------

    /**
     * \brief Set the UT PHY instance for satellite link reception
     * \param utPhy pointer to the satellite module's SatPhy (SatUtPhy)
     */
    void SetUtPhy(Ptr<SatPhy> utPhy);

    /**
     * \brief Get the UT PHY instance
     * \return pointer to SatPhy
     */
    Ptr<SatPhy> GetUtPhy() const;

    // ---------------------------------------------------------------
    // Doppler tracking from received signal
    // ---------------------------------------------------------------

    /**
     * \brief Estimate Doppler shift from actual received SatSignalParameters
     *
     * Uses the carrier frequency in the received signal parameters and
     * compares with the nominal center frequency to estimate Doppler.
     * If the satellite module has computed SINR, uses that for PLL
     * tracking quality estimation.
     *
     * \param rxParams received signal parameters from satellite channel
     * \return estimated Doppler shift in Hz
     */
    double EstimateDopplerFromSignal(Ptr<SatSignalParameters> rxParams) const;

    /**
     * \brief Compute residual Doppler tracking error
     *
     * PLL tracking error based on actual loop SNR derived from the
     * last measured SINR and configured PLL bandwidth.
     *
     * \return residual Doppler tracking error in Hz (RMS)
     */
    double ComputeDopplerTrackingError_Hz() const;

    // ---------------------------------------------------------------
    // CQI generation compatible with mmWave pipeline
    // ---------------------------------------------------------------

    /**
     * \brief Generate CQI feedback from per-sub-band SINR
     *
     * Takes a SpectrumValue of per-sub-band SINR values and produces
     * a wideband CQI and per-RB CQI vector compatible with the mmWave
     * scheduler's DlCqiInfo structure.
     *
     * \param sinr per-sub-band SINR as a SpectrumValue
     * \param[out] wbCqi wideband CQI value
     * \param[out] wbMcs wideband MCS value
     * \return per-RB CQI vector
     */
    std::vector<uint8_t> GenerateCqiFeedback(Ptr<const SpectrumValue> sinr,
                                              uint8_t& wbCqi,
                                              uint8_t& wbMcs) const;

    /**
     * \brief Generate CQI from a scalar SINR measurement
     *
     * Convenience method that creates a uniform SpectrumValue from the
     * scalar SINR and produces CQI feedback.
     *
     * \param sinrDb SINR in dB
     * \param[out] wbCqi wideband CQI value
     * \param[out] wbMcs wideband MCS value
     */
    void GenerateCqiFromSinr(double sinrDb, uint8_t& wbCqi, uint8_t& wbMcs) const;

    // ---------------------------------------------------------------
    // AGC from actual received power variations
    // ---------------------------------------------------------------

    /**
     * \brief Compute AGC gain from actual received power
     *
     * Uses actual instantaneous received power (in Watts) and a
     * configurable target power to compute the gain adjustment.
     * The gain is clamped to the AGC dynamic range.
     *
     * \param instantRxPower_W instantaneous received power in Watts
     * \param targetRxPower_W target received power in Watts
     * \return AGC gain as a linear multiplier
     */
    double ComputeAgcGain(double instantRxPower_W, double targetRxPower_W) const;

    /**
     * \brief Compute AGC gain from SatSignalParameters
     *
     * Extracts the received power from the signal parameters and
     * computes the AGC gain to bring it to the target level.
     *
     * \param rxParams received signal parameters
     * \return AGC gain as a linear multiplier
     */
    double ComputeAgcGainFromSignal(Ptr<SatSignalParameters> rxParams) const;

    // ---------------------------------------------------------------
    // Receiver configuration
    // ---------------------------------------------------------------

    void SetReceiverType(ReceiverType type);
    ReceiverType GetReceiverType() const;

    void SetPolarization(Polarization pol);
    Polarization GetPolarization() const;

    /**
     * \brief Compute cross-polarization discrimination loss
     * \return polarization discrimination loss in dB
     */
    double ComputePolarizationLoss_dB() const;

    /**
     * \brief Compute receiver sensitivity (minimum detectable signal)
     *
     * Computed from actual noise PSD and receiver-type-dependent margin.
     *
     * \return minimum detectable signal level in dBm
     */
    double ComputeReceiverSensitivity_dBm() const;

    /**
     * \brief Get the current adapted MCS index (from last CQI computation)
     * \return MCS index
     */
    uint8_t GetCurrentMcs() const;

  private:
    Ptr<SatPhy> m_utPhy;              ///< Reference to satellite module's SatUtPhy
    ReceiverType m_receiverType;      ///< THz receiver architecture
    Polarization m_polarization;      ///< Antenna polarization config
    double m_pllBandwidthHz;          ///< PLL loop bandwidth in Hz
    double m_agcTargetPower_W;        ///< AGC target power in Watts
    mutable uint8_t m_currentMcs;     ///< Last computed MCS index

    TracedCallback<uint8_t, uint8_t> m_cqiTrace;    ///< wbCqi, wbMcs
    TracedCallback<double> m_agcTrace;               ///< agcGain_linear
    TracedCallback<double> m_dopplerEstTrace;        ///< estimatedDoppler_Hz

    static constexpr double AGC_MAX_GAIN_LINEAR = 1.0e4;   ///< Maximum AGC gain (40 dB)
    static constexpr double AGC_MIN_GAIN_LINEAR = 0.1;     ///< Minimum AGC gain (-10 dB)
};

} // namespace ns3

#endif // THZ_NTN_PHY_GROUND_H
