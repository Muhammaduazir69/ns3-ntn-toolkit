/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-only
 * Author: Muhammad Uzair
 *
 * THz-NTN Waveform Model
 *
 * Provides waveform-specific parameters and processing for THz
 * non-terrestrial network links.  Models five candidate waveforms
 * for 6G THz-NTN systems, capturing their distinct characteristics
 * in terms of PAPR, Doppler robustness, spectral efficiency, and
 * computational complexity.
 *
 * Supported waveforms:
 *
 *   1. OFDM — Standard multicarrier.  Susceptible to inter-carrier
 *      interference (ICI) from high Doppler at THz/LEO.  Requires
 *      large subcarrier spacing.  PAPR ~11 dB.
 *
 *   2. DFT-s-OFDM — DFT-precoded OFDM.  Lower PAPR (~7 dB) makes it
 *      attractive for uplink where UE power amplifier efficiency
 *      matters.  3GPP NR extension to THz bands.
 *
 *   3. OTFS — Orthogonal Time Frequency Space.  Operates in the
 *      delay-Doppler domain, spreading each symbol across the entire
 *      time-frequency grid.  Extremely robust to high Doppler,
 *      ideal for LEO-ground THz links.  Higher complexity.
 *
 *   4. AFDM — Affine Frequency Division Multiplexing.  Chirp-based
 *      waveform where each subcarrier has a linear frequency variation.
 *      Excellent for joint sensing and communication (ISAC).
 *
 *   5. SC-FDE — Single-carrier with frequency-domain equalization.
 *      Lowest PAPR (~2 dB).  Simple implementation.  Well-suited for
 *      inter-satellite links where power efficiency dominates.
 *
 * References:
 *   [1] R. Hadani et al., "Orthogonal time frequency space modulation,"
 *       IEEE WCNC, 2017.
 *   [2] A. Bemani et al., "Affine frequency division multiplexing,"
 *       IEEE Trans. Wireless Commun., 2023.
 *   [3] T. S. Rappaport et al., "Wireless communications and applications
 *       above 100 GHz," IEEE Access, vol. 7, 2019.
 *   [4] 3GPP TR 38.817-01, "General aspects for UE RF for NR," 2022.
 */

#ifndef THZ_NTN_WAVEFORM_H
#define THZ_NTN_WAVEFORM_H

#include "thz-ntn-phy.h"

#include <ns3/object.h>

#include <string>

namespace ns3
{

/**
 * \ingroup thz-ntn
 * \brief Parameter set describing a THz-NTN waveform
 *
 * Captures the key characteristics of a candidate waveform for
 * THz-NTN links, enabling objective comparison and automatic selection.
 */
struct WaveformParams
{
    std::string name;              ///< Human-readable waveform name
    double paprDb;                 ///< Peak-to-average power ratio in dB
    double dopplerRobustness;      ///< Doppler robustness factor [0, 1]
    double spectralEfficiencyFactor; ///< SE relative to Shannon (0, 1]
    double complexityFactor;       ///< Computational complexity relative to OFDM
    bool isacCapable;              ///< Supports joint sensing and communication
    double maxBandwidthGHz;        ///< Maximum supported bandwidth in GHz
};

/**
 * \ingroup thz-ntn
 * \brief THz-NTN waveform model and selector
 *
 * Encapsulates the parameters and processing characteristics of five
 * candidate waveforms for THz-NTN links.  Provides automatic waveform
 * selection based on channel conditions (Doppler, ISAC requirement,
 * link direction) and computation of waveform-specific metrics such as
 * PAPR reduction, Doppler tolerance, OTFS grid sizing, and AFDM chirp
 * rate configuration.
 */
class ThzNtnWaveform : public Object
{
  public:
    /**
     * \brief Get the ns-3 TypeId for this class
     * \return the TypeId
     */
    static TypeId GetTypeId();

    /** Default constructor */
    ThzNtnWaveform();

    /** Destructor */
    ~ThzNtnWaveform() override;

    // ---------------------------------------------------------------
    // Waveform parameter queries
    // ---------------------------------------------------------------

    /**
     * \brief Get the parameter set for a given waveform type
     * \param type  waveform type
     * \return parameter structure
     */
    WaveformParams GetWaveformParameters(WaveformType type) const;

    /**
     * \brief Automatically select the best waveform for given conditions
     *
     * Selection logic:
     *   1. If ISAC is required, select AFDM (only ISAC-capable waveform)
     *   2. If maximum Doppler exceeds OFDM tolerance, prefer OTFS
     *   3. If uplink, prefer DFT-s-OFDM for lower PAPR
     *   4. Otherwise, default to OFDM for maximum spectral efficiency
     *
     * \param maxDopplerHz   maximum expected Doppler spread in Hz
     * \param isacRequired   whether ISAC capability is needed
     * \param isUplink       whether this is an uplink transmission
     * \return selected waveform type
     */
    WaveformType SelectBestWaveform(double maxDopplerHz,
                                    bool isacRequired,
                                    bool isUplink) const;

    // ---------------------------------------------------------------
    // Waveform-specific computations
    // ---------------------------------------------------------------

    /**
     * \brief Compute PAPR reduction relative to standard OFDM
     *
     * Returns the PAPR advantage (positive value = lower PAPR) of the
     * specified waveform compared to baseline OFDM.
     *
     * \param type  waveform type
     * \return PAPR reduction in dB (positive = better)
     */
    double ComputePaprReduction_dB(WaveformType type) const;

    /**
     * \brief Compute maximum tolerable Doppler for a waveform
     *
     * For OFDM-family waveforms, the Doppler tolerance is bounded by
     * SCS / (2 * dopplerRobustness_correction).  For OTFS and AFDM,
     * tolerance scales with the full bandwidth or chirp rate.
     *
     * \param type   waveform type
     * \param scsHz  subcarrier spacing in Hz (relevant for OFDM family)
     * \return maximum tolerable Doppler shift in Hz
     */
    double ComputeDopplerTolerance_Hz(WaveformType type, double scsHz) const;

    /**
     * \brief Compute OTFS delay-Doppler grid dimensions
     *
     * The OTFS grid M x N is sized to resolve the channel delay spread
     * and Doppler spread:
     *   M >= T_frame * delta_f  (number of subcarriers / delay bins)
     *   N >= T_frame / T_symbol (number of time slots / Doppler bins)
     *
     * Returns M * N as the total grid size.  Individual M and N can be
     * derived from the delay and Doppler requirements.
     *
     * \param maxDelaySpread_us  maximum channel delay spread in microseconds
     * \param maxDopplerHz       maximum Doppler spread in Hz
     * \return total OTFS grid size (M * N)
     */
    double ComputeOtfsGridSize(double maxDelaySpread_us,
                               double maxDopplerHz) const;

    /**
     * \brief Compute AFDM chirp rate for given channel conditions
     *
     * The AFDM chirp rate determines the frequency sweep of each
     * subcarrier.  For optimal Doppler resilience, the chirp rate
     * should satisfy:
     *   chirp_rate >= 2 * f_D_max / T_symbol
     * where T_symbol is the AFDM symbol duration.
     *
     * \param maxDopplerHz  maximum Doppler spread in Hz
     * \param bandwidthHz   channel bandwidth in Hz
     * \return chirp rate in Hz/s
     */
    double ComputeAfdmChirpRate(double maxDopplerHz, double bandwidthHz) const;

    /**
     * \brief Compute relative processing complexity
     *
     * Estimates the computational cost in relative FLOPS for a given
     * waveform and number of subcarriers.  OFDM is the baseline at
     * O(N log N) for the FFT.
     *
     * \param type            waveform type
     * \param numSubcarriers  number of subcarriers / grid points
     * \return relative computational complexity (FLOPS)
     */
    uint32_t ComputeProcessingComplexity(WaveformType type,
                                         uint32_t numSubcarriers) const;

  private:
    WaveformType m_defaultWaveform;  ///< Default waveform type

    /**
     * \brief Static preset waveform parameters
     */
    static const WaveformParams s_presets[WAVEFORM_COUNT];
};

} // namespace ns3

#endif // THZ_NTN_WAVEFORM_H
