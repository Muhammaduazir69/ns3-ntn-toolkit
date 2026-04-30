/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-only
 * Author: Muhammad Uzair
 *
 * THz Hardware Impairments Model for THz-NTN Links
 *
 * Comprehensive hardware non-ideality model that captures the practical
 * capacity ceiling of terahertz non-terrestrial network links.  At THz
 * frequencies, hardware limitations severely bound achievable spectral
 * efficiency even at high SNR.
 *
 * Modeled impairment sources:
 *   1. Power amplifier nonlinearity (Rapp / Saleh models)
 *   2. Phase noise from THz oscillators (Lorentzian PSD)
 *   3. ADC quantization noise (limited to 4--8 bits at THz rates)
 *   4. I/Q imbalance (amplitude and phase mismatch)
 *   5. Digital pre-distortion (DPD) for PA linearisation
 *
 * The aggregate effect is captured by an equivalent EVM that establishes
 * a hardware-limited capacity ceiling:
 *   C = log2(1 + SNR / (1 + SNR * kappa^2))
 * where kappa^2 is the aggregate squared EVM.
 *
 * References:
 *   [1] E. Bjornson et al., "Hardware impairments in large-scale MISO
 *       systems," IEEE Trans. Commun., 2014.
 *   [2] C. Rapp, "Effects of HPA-nonlinearity on a 4-DPSK/OFDM signal,"
 *       ESA SP-332, 1991.
 *   [3] A. A. M. Saleh, "Frequency-independent and frequency-dependent
 *       nonlinear models of TWT amplifiers," IEEE Trans. Commun., 1981.
 *   [4] T. Schenk, "RF Imperfections in High-Rate Wireless Systems:
 *       Impact and Digital Compensation," Springer, 2008.
 */

#ifndef THZ_NTN_HARDWARE_IMPAIRMENTS_H
#define THZ_NTN_HARDWARE_IMPAIRMENTS_H

#include <ns3/object.h>

#include <string>

namespace ns3
{

/**
 * \ingroup thz-ntn
 * \brief THz hardware impairments model for NTN links
 *
 * Models the aggregate effect of power amplifier nonlinearity, oscillator
 * phase noise, ADC quantization noise, and I/Q imbalance on THz-NTN link
 * performance.  The combined impairments establish a hardware-limited
 * spectral efficiency ceiling via an equivalent EVM metric.
 *
 * Two PA models are supported: the Rapp model (solid-state PA, typical
 * for ground terminals) and the Saleh model (traveling-wave tube,
 * typical for satellite transponders).
 */
class ThzNtnHardwareImpairments : public Object
{
  public:
    /**
     * \brief Get the type ID
     * \return the object TypeId
     */
    static TypeId GetTypeId();

    ThzNtnHardwareImpairments();
    ~ThzNtnHardwareImpairments() override;

    /**
     * \brief Compute aggregate EVM from all impairment sources
     *
     * EVM_total = sqrt(EVM_PA^2 + EVM_PN^2 + EVM_ADC^2 + EVM_IQ^2)
     *
     * \return aggregate EVM (dimensionless, 0 to 1)
     */
    double ComputeEvmTotal() const;

    /**
     * \brief Compute EVM contribution from PA nonlinearity
     *
     * Uses either the Rapp or Saleh model depending on configuration.
     * The EVM is estimated from the ratio of distortion power to signal
     * power at the configured input back-off.
     *
     * \param iboDb input back-off in dB
     * \return PA nonlinearity EVM (dimensionless)
     */
    double ComputePaNonlinearityEvm(double iboDb) const;

    /**
     * \brief Compute EVM contribution from oscillator phase noise
     *
     * Models the Lorentzian phase noise PSD and computes its effect on
     * OFDM subcarriers as common phase error (CPE) and inter-carrier
     * interference (ICI).
     *
     * \param freqHz carrier frequency in Hz
     * \param subcarrierSpacingHz OFDM subcarrier spacing in Hz
     * \return phase noise EVM (dimensionless)
     */
    double ComputePhaseNoiseEvm(double freqHz,
                                double subcarrierSpacingHz) const;

    /**
     * \brief Compute ADC quantization noise power
     *
     * SQNR = 6.02 * b + 1.76 dB, where b is the number of ADC bits.
     * Returns the noise floor in dBm relative to the input signal.
     *
     * \param signalPower_dBm input signal power in dBm
     * \return quantization noise power in dBm
     */
    double ComputeAdcQuantizationNoise_dBm(double signalPower_dBm) const;

    /**
     * \brief Compute EVM contribution from I/Q imbalance
     *
     * Models both amplitude imbalance (epsilon) and phase imbalance
     * (delta) between the in-phase and quadrature branches.
     *
     * \return I/Q imbalance EVM (dimensionless)
     */
    double ComputeIqImbalanceEvm() const;

    /**
     * \brief Compute hardware-limited spectral efficiency
     *
     * C = log2(1 + SNR / (1 + SNR * kappa^2))
     * where kappa^2 = EVM_total^2
     *
     * \param snr_linear SNR in linear scale
     * \return spectral efficiency in bits/s/Hz
     */
    double ComputeHardwareLimitedCapacity(double snr_linear) const;

    /**
     * \brief Compute effective SNR after hardware impairments
     *
     * SNR_eff = SNR / (1 + SNR * kappa^2)
     *
     * \param idealSnr_dB ideal SNR in dB (before impairments)
     * \return effective SNR in dB (after impairments)
     */
    double ComputeEffectiveSnr_dB(double idealSnr_dB) const;

    /**
     * \brief Set the PA model type
     * \param model PA model name: "rapp" or "saleh"
     */
    void SetPaModel(const std::string& model);

    /**
     * \brief Enable or disable digital pre-distortion
     * \param enable true to enable DPD
     */
    void EnableDpd(bool enable);

  private:
    /**
     * \brief Compute Rapp model AM/AM characteristic
     *
     * g(r) = r / (1 + (r / A_sat)^(2p))^(1/(2p))
     *
     * \param inputAmplitude normalised input amplitude
     * \return output amplitude
     */
    double RappAmAm(double inputAmplitude) const;

    /**
     * \brief Compute Saleh model AM/AM characteristic
     *
     * g(r) = alpha_a * r / (1 + beta_a * r^2)
     *
     * \param inputAmplitude normalised input amplitude
     * \return output amplitude
     */
    double SalehAmAm(double inputAmplitude) const;

    /**
     * \brief Compute Saleh model AM/PM characteristic
     *
     * phi(r) = alpha_p * r^2 / (1 + beta_p * r^2)
     *
     * \param inputAmplitude normalised input amplitude
     * \return phase distortion in radians
     */
    double SalehAmPm(double inputAmplitude) const;

    /**
     * \brief Compute EVM from Rapp model at given IBO
     * \param iboDb input back-off in dB
     * \return EVM (dimensionless)
     */
    double ComputeRappEvm(double iboDb) const;

    /**
     * \brief Compute EVM from Saleh model at given IBO
     * \param iboDb input back-off in dB
     * \return EVM (dimensionless)
     */
    double ComputeSalehEvm(double iboDb) const;

    // --- Configurable attributes ---

    std::string m_paModel;            //!< PA model: "rapp" or "saleh"
    double m_paSaturation_dBm;        //!< PA saturation power (dBm)
    double m_paSmoothnessFactor;      //!< Rapp model smoothness factor p
    double m_phaseNoiseLinewidth_Hz;  //!< Oscillator 3-dB linewidth (Hz)
    uint32_t m_adcBits;               //!< ADC resolution (bits)
    double m_iqAmplitudeImbalance_dB; //!< I/Q amplitude imbalance (dB)
    double m_iqPhaseImbalance_deg;    //!< I/Q phase imbalance (degrees)
    bool m_enableDpd;                 //!< Digital pre-distortion enable
    double m_inputBackoff_dB;         //!< Default input back-off (dB)

    // --- Saleh model coefficients ---

    static constexpr double SALEH_ALPHA_A = 2.1587; //!< AM/AM alpha
    static constexpr double SALEH_BETA_A = 1.1517;  //!< AM/AM beta
    static constexpr double SALEH_ALPHA_P = 4.0033; //!< AM/PM alpha (rad)
    static constexpr double SALEH_BETA_P = 9.1040;  //!< AM/PM beta

    // --- DPD improvement ---

    static constexpr double DPD_IMPROVEMENT_DB = 15.0; //!< DPD PA EVM reduction

    // --- Conversion constants ---

    static constexpr double DEG_TO_RAD = 3.14159265358979323846 / 180.0;
};

} // namespace ns3

#endif // THZ_NTN_HARDWARE_IMPAIRMENTS_H
