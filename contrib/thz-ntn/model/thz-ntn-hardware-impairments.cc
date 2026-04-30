/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-only
 * Author: Muhammad Uzair
 *
 * THz Hardware Impairments Model for THz-NTN Links -- Implementation
 */

#include "thz-ntn-hardware-impairments.h"

#include <ns3/boolean.h>
#include <ns3/double.h>
#include <ns3/enum.h>
#include <ns3/log.h>
#include <ns3/string.h>
#include <ns3/uinteger.h>

#include <cmath>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("ThzNtnHardwareImpairments");

NS_OBJECT_ENSURE_REGISTERED(ThzNtnHardwareImpairments);

TypeId
ThzNtnHardwareImpairments::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::ThzNtnHardwareImpairments")
            .SetParent<Object>()
            .SetGroupName("ThzNtn")
            .AddConstructor<ThzNtnHardwareImpairments>()
            .AddAttribute(
                "PaModel",
                "Power amplifier model: 'rapp' for solid-state PA "
                "(ground terminal) or 'saleh' for traveling-wave tube "
                "(satellite transponder).",
                StringValue("rapp"),
                MakeStringAccessor(
                    &ThzNtnHardwareImpairments::m_paModel),
                MakeStringChecker())
            .AddAttribute(
                "PaSaturation_dBm",
                "PA output saturation power in dBm.",
                DoubleValue(30.0),
                MakeDoubleAccessor(
                    &ThzNtnHardwareImpairments::m_paSaturation_dBm),
                MakeDoubleChecker<double>(-10.0, 60.0))
            .AddAttribute(
                "PaSmoothnessFactor",
                "Rapp model smoothness factor p. Higher values yield "
                "sharper saturation (p=1: soft limiter, p->inf: hard "
                "clipper). Typical solid-state PA: p=2--3.",
                DoubleValue(3.0),
                MakeDoubleAccessor(
                    &ThzNtnHardwareImpairments::m_paSmoothnessFactor),
                MakeDoubleChecker<double>(0.5, 20.0))
            .AddAttribute(
                "PhaseNoiseLinewidth_Hz",
                "Oscillator 3-dB linewidth (beta) in Hz. Determines "
                "the Lorentzian phase noise PSD. Typical THz sources: "
                "100 Hz (high-quality) to 10 kHz (free-running).",
                DoubleValue(1000.0),
                MakeDoubleAccessor(
                    &ThzNtnHardwareImpairments::m_phaseNoiseLinewidth_Hz),
                MakeDoubleChecker<double>(1.0, 1e8))
            .AddAttribute(
                "AdcBits",
                "ADC resolution in bits. At THz sampling rates "
                "(>100 GS/s), practical ADCs are limited to 4--8 bits.",
                UintegerValue(6),
                MakeUintegerAccessor(&ThzNtnHardwareImpairments::m_adcBits),
                MakeUintegerChecker<uint32_t>(1, 16))
            .AddAttribute(
                "IqAmplitudeImbalance_dB",
                "I/Q mixer amplitude imbalance in dB. Typical range "
                "0.5--2 dB for THz front-ends.",
                DoubleValue(1.0),
                MakeDoubleAccessor(
                    &ThzNtnHardwareImpairments::m_iqAmplitudeImbalance_dB),
                MakeDoubleChecker<double>(0.0, 10.0))
            .AddAttribute(
                "IqPhaseImbalance_deg",
                "I/Q mixer phase imbalance in degrees. Typical range "
                "1--5 degrees for THz front-ends.",
                DoubleValue(3.0),
                MakeDoubleAccessor(
                    &ThzNtnHardwareImpairments::m_iqPhaseImbalance_deg),
                MakeDoubleChecker<double>(0.0, 30.0))
            .AddAttribute(
                "EnableDpd",
                "Enable digital pre-distortion for PA linearisation. "
                "When enabled, PA EVM is reduced by ~15 dB at the cost "
                "of additional processing latency.",
                BooleanValue(false),
                MakeBooleanAccessor(
                    &ThzNtnHardwareImpairments::m_enableDpd),
                MakeBooleanChecker())
            .AddAttribute(
                "InputBackoff_dB",
                "Default input back-off in dB. Operating the PA below "
                "saturation reduces distortion but also output power.",
                DoubleValue(6.0),
                MakeDoubleAccessor(
                    &ThzNtnHardwareImpairments::m_inputBackoff_dB),
                MakeDoubleChecker<double>(0.0, 30.0));
    return tid;
}

ThzNtnHardwareImpairments::ThzNtnHardwareImpairments()
    : m_paModel("rapp"),
      m_paSaturation_dBm(30.0),
      m_paSmoothnessFactor(3.0),
      m_phaseNoiseLinewidth_Hz(1000.0),
      m_adcBits(6),
      m_iqAmplitudeImbalance_dB(1.0),
      m_iqPhaseImbalance_deg(3.0),
      m_enableDpd(false),
      m_inputBackoff_dB(6.0)
{
    NS_LOG_FUNCTION(this);
}

ThzNtnHardwareImpairments::~ThzNtnHardwareImpairments()
{
    NS_LOG_FUNCTION(this);
}

// ---------------------------------------------------------------------------
// Power Amplifier Models
// ---------------------------------------------------------------------------

double
ThzNtnHardwareImpairments::RappAmAm(double inputAmplitude) const
{
    // Rapp model AM/AM characteristic for solid-state PAs:
    //   g(r) = r / (1 + (r / A_sat)^(2p))^(1/(2p))
    //
    // where:
    //   r     = input amplitude (normalised)
    //   A_sat = saturation amplitude (normalised to 1.0)
    //   p     = smoothness factor

    double p = m_paSmoothnessFactor;
    double r = inputAmplitude;
    double aSat = 1.0; // normalised saturation amplitude

    double ratio = r / aSat;
    double ratio2p = std::pow(ratio, 2.0 * p);
    double denominator = std::pow(1.0 + ratio2p, 1.0 / (2.0 * p));

    return r / denominator;
}

double
ThzNtnHardwareImpairments::SalehAmAm(double inputAmplitude) const
{
    // Saleh model AM/AM for TWT amplifiers:
    //   g(r) = alpha_a * r / (1 + beta_a * r^2)

    double r = inputAmplitude;
    return SALEH_ALPHA_A * r / (1.0 + SALEH_BETA_A * r * r);
}

double
ThzNtnHardwareImpairments::SalehAmPm(double inputAmplitude) const
{
    // Saleh model AM/PM for TWT amplifiers:
    //   phi(r) = alpha_p * r^2 / (1 + beta_p * r^2)
    //
    // Returns phase distortion in radians.

    double r = inputAmplitude;
    return SALEH_ALPHA_P * r * r / (1.0 + SALEH_BETA_P * r * r);
}

double
ThzNtnHardwareImpairments::ComputeRappEvm(double iboDb) const
{
    // Estimate EVM from Rapp model by computing the normalised mean-square
    // error between the ideal linear output and the Rapp nonlinear output
    // for a range of input amplitudes.
    //
    // The input amplitude distribution of an OFDM signal is approximately
    // Rayleigh.  We approximate the EVM by numerical integration over
    // the amplitude range [0, 3*sigma], where sigma = A_rms.
    //
    // A_rms is set by the input back-off:
    //   IBO = 10*log10(P_sat / P_in) = 10*log10(A_sat^2 / A_rms^2)
    //   A_rms = A_sat * 10^(-IBO/20)

    double aSat = 1.0;
    double aRms = aSat * std::pow(10.0, -iboDb / 20.0);

    // Numerical integration: Rayleigh-weighted MSE
    constexpr int numSamples = 200;
    double maxAmplitude = 4.0 * aRms; // covers 99.97% of Rayleigh CDF
    double dr = maxAmplitude / numSamples;

    double mseNumerator = 0.0;
    double powerDenominator = 0.0;

    for (int i = 1; i <= numSamples; ++i)
    {
        double r = i * dr;

        // Rayleigh PDF: f(r) = (r / sigma^2) * exp(-r^2 / (2*sigma^2))
        double sigma = aRms;
        double rayleighPdf =
            (r / (sigma * sigma)) *
            std::exp(-r * r / (2.0 * sigma * sigma));

        // Ideal linear output (with gain = 1)
        double idealOutput = r;

        // Rapp nonlinear output
        double nonlinearOutput = RappAmAm(r);

        // Distortion = difference between nonlinear and ideal scaled output
        // We scale ideal to match gain at small signal
        double error = nonlinearOutput - idealOutput;

        mseNumerator += error * error * rayleighPdf * dr;
        powerDenominator += idealOutput * idealOutput * rayleighPdf * dr;
    }

    double evm = 0.0;
    if (powerDenominator > 0.0)
    {
        evm = std::sqrt(mseNumerator / powerDenominator);
    }

    NS_LOG_DEBUG("Rapp EVM at IBO=" << iboDb << " dB: " << evm
                                    << " (A_rms=" << aRms << ")");

    return evm;
}

double
ThzNtnHardwareImpairments::ComputeSalehEvm(double iboDb) const
{
    // Estimate EVM from Saleh model by numerical Rayleigh-weighted
    // integration, similar to the Rapp model but including both AM/AM
    // and AM/PM distortion.

    double aSat = 1.0;
    double aRms = aSat * std::pow(10.0, -iboDb / 20.0);

    constexpr int numSamples = 200;
    double maxAmplitude = 4.0 * aRms;
    double dr = maxAmplitude / numSamples;

    double mseNumerator = 0.0;
    double powerDenominator = 0.0;

    // Small-signal gain of Saleh model: g'(0) = alpha_a
    double smallSignalGain = SALEH_ALPHA_A;

    for (int i = 1; i <= numSamples; ++i)
    {
        double r = i * dr;

        double sigma = aRms;
        double rayleighPdf =
            (r / (sigma * sigma)) *
            std::exp(-r * r / (2.0 * sigma * sigma));

        // Ideal output (linear with small-signal gain)
        double idealOutput = smallSignalGain * r;

        // Saleh nonlinear output (amplitude)
        double nlAmplitude = SalehAmAm(r);

        // Saleh AM/PM phase rotation
        double phaseError = SalehAmPm(r);

        // Complex output: nlAmplitude * exp(j*phaseError)
        // Complex ideal: idealOutput * exp(j*0)
        // Error magnitude^2 = |nl*exp(j*phi) - ideal|^2
        //   = nl^2 + ideal^2 - 2*nl*ideal*cos(phi)
        double errorSq = nlAmplitude * nlAmplitude +
                          idealOutput * idealOutput -
                          2.0 * nlAmplitude * idealOutput *
                              std::cos(phaseError);

        mseNumerator += errorSq * rayleighPdf * dr;
        powerDenominator +=
            idealOutput * idealOutput * rayleighPdf * dr;
    }

    double evm = 0.0;
    if (powerDenominator > 0.0)
    {
        evm = std::sqrt(mseNumerator / powerDenominator);
    }

    NS_LOG_DEBUG("Saleh EVM at IBO=" << iboDb << " dB: " << evm
                                     << " (A_rms=" << aRms << ")");

    return evm;
}

double
ThzNtnHardwareImpairments::ComputePaNonlinearityEvm(double iboDb) const
{
    NS_LOG_FUNCTION(this << iboDb);

    double evm;

    if (m_paModel == "saleh")
    {
        evm = ComputeSalehEvm(iboDb);
    }
    else
    {
        // Default to Rapp model
        evm = ComputeRappEvm(iboDb);
    }

    // Apply DPD improvement if enabled
    if (m_enableDpd)
    {
        // DPD reduces distortion by DPD_IMPROVEMENT_DB in power,
        // which is half that in amplitude (EVM)
        double reductionLinear =
            std::pow(10.0, -DPD_IMPROVEMENT_DB / 20.0);
        evm *= reductionLinear;

        NS_LOG_DEBUG("DPD applied: EVM reduced by "
                     << DPD_IMPROVEMENT_DB << " dB to " << evm);
    }

    return evm;
}

double
ThzNtnHardwareImpairments::ComputePhaseNoiseEvm(
    double freqHz,
    double subcarrierSpacingHz) const
{
    NS_LOG_FUNCTION(this << freqHz << subcarrierSpacingHz);

    // Lorentzian phase noise PSD:
    //   S_phi(f) = beta / (pi * (f^2 + beta^2))
    //
    // where beta is the 3-dB linewidth (half of full linewidth).
    //
    // The phase noise at THz frequencies scales linearly with carrier
    // frequency relative to a reference oscillator.  The linewidth
    // attribute already accounts for this scaling.
    //
    // For OFDM, phase noise causes:
    //   1. Common Phase Error (CPE): phase rotation common to all
    //      subcarriers within one OFDM symbol.
    //   2. Inter-Carrier Interference (ICI): phase noise energy
    //      leaking between subcarriers.
    //
    // Total phase noise power within one subcarrier bandwidth:
    //   sigma_phi^2 = integral of S_phi(f) from -SCS/2 to SCS/2
    //              = (2/pi) * arctan(SCS / (2*beta))
    //
    // For narrowband (SCS >> beta): sigma_phi^2 -> 1 (all noise within SCS)
    // For wideband (SCS << beta):   sigma_phi^2 -> SCS / (pi*beta)
    //
    // ICI power (out-of-subcarrier leakage):
    //   P_ICI ~ 1 - sigma_phi^2 (complement of in-band power)
    //
    // But the more practical metric is the EVM from total integrated
    // phase noise:
    //   EVM_PN ~ sqrt(2 * pi * beta / SCS)  for SCS >> beta

    double beta = m_phaseNoiseLinewidth_Hz;

    // Integrated phase noise variance over the subcarrier
    // Using the exact Lorentzian integral:
    //   sigma_phi^2 = (2/pi) * arctan(SCS / (2*beta))
    double inBandPower =
        (2.0 / M_PI) * std::atan(subcarrierSpacingHz / (2.0 * beta));

    // ICI power (noise leaking outside the subcarrier)
    double iciPower = 1.0 - inBandPower;

    // The EVM is dominated by the ICI component for OFDM:
    // For small beta/SCS ratio, approximate:
    //   EVM_PN ~ sqrt(2 * pi * beta / SCS)
    // For general case, use the ICI power directly:
    double evmPN = std::sqrt(iciPower);

    // Scale by carrier frequency effect: higher frequencies have
    // proportionally worse phase noise.  Normalise to 100 GHz reference.
    constexpr double freqRef_Hz = 100.0e9;
    double freqScaling = std::sqrt(freqHz / freqRef_Hz);
    evmPN *= freqScaling;

    // Clamp to physical limits
    evmPN = std::min(evmPN, 1.0);

    NS_LOG_DEBUG("Phase noise EVM: beta=" << beta
                                          << " Hz, SCS="
                                          << subcarrierSpacingHz
                                          << " Hz, ICI="
                                          << iciPower
                                          << ", EVM=" << evmPN);

    return evmPN;
}

double
ThzNtnHardwareImpairments::ComputeAdcQuantizationNoise_dBm(
    double signalPower_dBm) const
{
    NS_LOG_FUNCTION(this << signalPower_dBm);

    // SQNR for a uniform quantizer with sinusoidal input:
    //   SQNR = 6.02 * b + 1.76  [dB]
    //
    // For complex (I/Q) signals, SQNR applies to each branch.
    // Quantization noise power:
    //   N_q = P_signal - SQNR  [dBm]

    double sqnr_dB = 6.02 * m_adcBits + 1.76;
    double noiseFloor_dBm = signalPower_dBm - sqnr_dB;

    NS_LOG_DEBUG("ADC quantization: " << m_adcBits
                                      << " bits, SQNR="
                                      << sqnr_dB
                                      << " dB, noise="
                                      << noiseFloor_dBm << " dBm");

    return noiseFloor_dBm;
}

double
ThzNtnHardwareImpairments::ComputeIqImbalanceEvm() const
{
    NS_LOG_FUNCTION(this);

    // I/Q imbalance model:
    //   Received signal: y = alpha * x + beta * x*
    //   where x* is complex conjugate (image signal)
    //
    // alpha = cos(delta/2) + j * epsilon * sin(delta/2)
    // beta  = epsilon * cos(delta/2) - j * sin(delta/2)
    //
    // where:
    //   epsilon = 10^(amplitude_imbalance_dB / 20) - 1  (gain error)
    //   delta   = phase_imbalance_rad
    //
    // Image Rejection Ratio (IRR):
    //   IRR = |alpha|^2 / |beta|^2
    //       = (1 + g)^2 + 2*(1+g)*cos(delta) + 1
    //         / ((1-g)^2 - 2*(1-g)*cos(delta) + 1)
    //
    // Simplified IRR for small imbalances:
    //   IRR ~ 1 / (epsilon^2/4 + delta^2/4)
    //
    // EVM from I/Q imbalance:
    //   EVM_IQ = 1 / sqrt(IRR) ~ sqrt(epsilon^2 + delta^2) / 2

    // Convert amplitude imbalance from dB to linear gain error
    double g = std::pow(10.0, m_iqAmplitudeImbalance_dB / 20.0);
    double deltaRad = m_iqPhaseImbalance_deg * DEG_TO_RAD;

    // Compute alpha and beta magnitudes
    // alpha = (1 + g*exp(j*delta)) / 2
    // beta  = (1 - g*exp(j*delta)) / 2
    //
    // |alpha|^2 = (1 + g^2 + 2*g*cos(delta)) / 4
    // |beta|^2  = (1 + g^2 - 2*g*cos(delta)) / 4

    double alphaSq =
        (1.0 + g * g + 2.0 * g * std::cos(deltaRad)) / 4.0;
    double betaSq =
        (1.0 + g * g - 2.0 * g * std::cos(deltaRad)) / 4.0;

    // EVM = |beta| / |alpha| (ratio of image to signal)
    double evmIQ = 0.0;
    if (alphaSq > 0.0)
    {
        evmIQ = std::sqrt(betaSq / alphaSq);
    }

    NS_LOG_DEBUG("I/Q imbalance: amp=" << m_iqAmplitudeImbalance_dB
                                       << " dB, phase="
                                       << m_iqPhaseImbalance_deg
                                       << " deg, IRR="
                                       << 10.0 * std::log10(alphaSq / betaSq)
                                       << " dB, EVM=" << evmIQ);

    return evmIQ;
}

double
ThzNtnHardwareImpairments::ComputeEvmTotal() const
{
    NS_LOG_FUNCTION(this);

    // Compute individual EVM contributions
    double evmPA = ComputePaNonlinearityEvm(m_inputBackoff_dB);

    // Default: 300 GHz carrier, 120 kHz SCS (NR numerology 3)
    double evmPN = ComputePhaseNoiseEvm(300.0e9, 120.0e3);

    // ADC EVM: SQNR maps to EVM as EVM_ADC = 1/sqrt(SQNR_linear)
    double sqnr_dB = 6.02 * m_adcBits + 1.76;
    double sqnr_linear = std::pow(10.0, sqnr_dB / 10.0);
    double evmADC = 1.0 / std::sqrt(sqnr_linear);

    double evmIQ = ComputeIqImbalanceEvm();

    // RSS aggregation (independent error sources):
    //   EVM_total^2 = EVM_PA^2 + EVM_PN^2 + EVM_ADC^2 + EVM_IQ^2
    double evmTotalSq = evmPA * evmPA +
                        evmPN * evmPN +
                        evmADC * evmADC +
                        evmIQ * evmIQ;

    double evmTotal = std::sqrt(evmTotalSq);

    NS_LOG_INFO("EVM components: PA=" << evmPA
                                      << ", PN=" << evmPN
                                      << ", ADC=" << evmADC
                                      << ", IQ=" << evmIQ
                                      << " -> total=" << evmTotal);

    return evmTotal;
}

double
ThzNtnHardwareImpairments::ComputeHardwareLimitedCapacity(
    double snr_linear) const
{
    NS_LOG_FUNCTION(this << snr_linear);

    // Hardware-limited spectral efficiency:
    //   C = log2(1 + SNR / (1 + SNR * kappa^2))
    //
    // where kappa^2 = EVM_total^2 is the aggregate distortion factor.
    //
    // At high SNR, this saturates to:
    //   C_max ~ log2(1 + 1/kappa^2)
    //
    // Typical ceilings:
    //   kappa = 0.08 (with DPD)  -> C_max ~ 7.4 bits/s/Hz
    //   kappa = 0.20 (no DPD)    -> C_max ~ 2.8 bits/s/Hz

    double kappa = ComputeEvmTotal();
    double kappaSq = kappa * kappa;

    double effectiveSnr = snr_linear / (1.0 + snr_linear * kappaSq);
    double capacity = std::log2(1.0 + effectiveSnr);

    NS_LOG_INFO("Hardware-limited capacity: kappa=" << kappa
                                                    << ", SNR="
                                                    << 10.0 * std::log10(snr_linear)
                                                    << " dB, C="
                                                    << capacity
                                                    << " bits/s/Hz");

    return capacity;
}

double
ThzNtnHardwareImpairments::ComputeEffectiveSnr_dB(double idealSnr_dB) const
{
    NS_LOG_FUNCTION(this << idealSnr_dB);

    double kappa = ComputeEvmTotal();
    double kappaSq = kappa * kappa;

    // Convert ideal SNR to linear
    double snrLinear = std::pow(10.0, idealSnr_dB / 10.0);

    // Effective SNR after impairments:
    //   SNR_eff = SNR / (1 + SNR * kappa^2)
    double effectiveSnrLinear = snrLinear / (1.0 + snrLinear * kappaSq);

    // Convert back to dB
    double effectiveSnr_dB = 10.0 * std::log10(effectiveSnrLinear);

    NS_LOG_DEBUG("Effective SNR: ideal=" << idealSnr_dB
                                         << " dB, kappa=" << kappa
                                         << ", effective="
                                         << effectiveSnr_dB << " dB");

    return effectiveSnr_dB;
}

void
ThzNtnHardwareImpairments::SetPaModel(const std::string& model)
{
    NS_LOG_FUNCTION(this << model);

    if (model != "rapp" && model != "saleh")
    {
        NS_LOG_WARN("Unknown PA model '" << model
                                         << "', defaulting to 'rapp'");
        m_paModel = "rapp";
    }
    else
    {
        m_paModel = model;
    }
}

void
ThzNtnHardwareImpairments::EnableDpd(bool enable)
{
    NS_LOG_FUNCTION(this << enable);
    m_enableDpd = enable;
}

} // namespace ns3
