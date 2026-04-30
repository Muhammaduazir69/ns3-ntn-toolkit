/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-only
 * Author: Muhammad Uzair
 *
 * THz-NTN Waveform Model — Implementation
 *
 * Implements waveform parameter lookup, automatic waveform selection,
 * PAPR comparison, Doppler tolerance analysis, OTFS grid sizing, AFDM
 * chirp rate computation, and processing complexity estimation for five
 * candidate THz-NTN waveforms.
 */

#include "thz-ntn-waveform.h"

#include <ns3/double.h>
#include <ns3/enum.h>
#include <ns3/log.h>

#include <cmath>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("ThzNtnWaveform");

NS_OBJECT_ENSURE_REGISTERED(ThzNtnWaveform);

// ---------------------------------------------------------------------------
// Static waveform presets
// ---------------------------------------------------------------------------

const WaveformParams ThzNtnWaveform::s_presets[WAVEFORM_COUNT] = {
    // name,        PAPR, doppRob, SE_fac, complex, ISAC,  maxBW
    {"OFDM",        11.0, 0.30,    1.00,   1.0,     false, 100.0},
    {"DFT-s-OFDM",   7.0, 0.40,    0.95,   1.2,     false, 100.0},
    {"OTFS",          9.0, 0.95,    0.98,   3.0,     false,  50.0},
    {"AFDM",          8.0, 0.85,    0.92,   2.5,     true,   50.0},
    {"SC-FDE",        2.0, 0.70,    0.90,   0.8,     false, 100.0},
};

// ---------------------------------------------------------------------------
// TypeId
// ---------------------------------------------------------------------------

TypeId
ThzNtnWaveform::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::ThzNtnWaveform")
            .SetParent<Object>()
            .SetGroupName("ThzNtn")
            .AddConstructor<ThzNtnWaveform>()
            .AddAttribute("DefaultWaveform",
                          "Default waveform type for THz-NTN links.",
                          EnumValue(WAVEFORM_OFDM),
                          MakeEnumAccessor<WaveformType>(&ThzNtnWaveform::m_defaultWaveform),
                          MakeEnumChecker(WAVEFORM_OFDM, "OFDM",
                                          WAVEFORM_DFT_S_OFDM, "DFT_S_OFDM",
                                          WAVEFORM_OTFS, "OTFS",
                                          WAVEFORM_AFDM, "AFDM",
                                          WAVEFORM_SC_FDE, "SC_FDE"));
    return tid;
}

ThzNtnWaveform::ThzNtnWaveform()
    : m_defaultWaveform(WAVEFORM_OFDM)
{
    NS_LOG_FUNCTION(this);
}

ThzNtnWaveform::~ThzNtnWaveform()
{
    NS_LOG_FUNCTION(this);
}

// ---------------------------------------------------------------------------
// Waveform parameter queries
// ---------------------------------------------------------------------------

WaveformParams
ThzNtnWaveform::GetWaveformParameters(WaveformType type) const
{
    NS_LOG_FUNCTION(this << type);

    if (type >= WAVEFORM_COUNT)
    {
        NS_LOG_WARN("Invalid waveform type " << type << ", defaulting to OFDM");
        return s_presets[WAVEFORM_OFDM];
    }

    return s_presets[type];
}

WaveformType
ThzNtnWaveform::SelectBestWaveform(double maxDopplerHz,
                                   bool isacRequired,
                                   bool isUplink) const
{
    NS_LOG_FUNCTION(this << maxDopplerHz << isacRequired << isUplink);

    // Priority 1: ISAC requirement — only AFDM supports it
    if (isacRequired)
    {
        NS_LOG_INFO("ISAC required: selecting AFDM");
        return WAVEFORM_AFDM;
    }

    // Priority 2: Very high Doppler — OTFS is the most robust
    // Threshold: if Doppler exceeds 500 kHz (typical for >200 GHz LEO),
    // OFDM-family waveforms struggle even with large SCS
    if (maxDopplerHz > 500e3)
    {
        NS_LOG_INFO("High Doppler (" << maxDopplerHz / 1e3
                    << " kHz > 500 kHz): selecting OTFS");
        return WAVEFORM_OTFS;
    }

    // Priority 3: Moderate Doppler with uplink — DFT-s-OFDM for low PAPR
    if (isUplink && maxDopplerHz > 100e3)
    {
        NS_LOG_INFO("Uplink with moderate Doppler: selecting DFT-s-OFDM");
        return WAVEFORM_DFT_S_OFDM;
    }

    // Priority 4: Uplink with low Doppler — still prefer DFT-s-OFDM
    // for PA efficiency at the ground terminal
    if (isUplink)
    {
        NS_LOG_INFO("Uplink: selecting DFT-s-OFDM for low PAPR");
        return WAVEFORM_DFT_S_OFDM;
    }

    // Priority 5: Moderate Doppler downlink — OFDM with sufficient SCS
    if (maxDopplerHz <= 500e3)
    {
        NS_LOG_INFO("Downlink with manageable Doppler: selecting OFDM");
        return WAVEFORM_OFDM;
    }

    // Fallback
    NS_LOG_INFO("Default selection: OFDM");
    return WAVEFORM_OFDM;
}

// ---------------------------------------------------------------------------
// Waveform-specific computations
// ---------------------------------------------------------------------------

double
ThzNtnWaveform::ComputePaprReduction_dB(WaveformType type) const
{
    NS_LOG_FUNCTION(this << type);

    if (type >= WAVEFORM_COUNT)
    {
        NS_LOG_WARN("Invalid waveform type, returning 0 dB reduction");
        return 0.0;
    }

    // PAPR reduction relative to OFDM baseline (11 dB)
    double reduction = s_presets[WAVEFORM_OFDM].paprDb - s_presets[type].paprDb;

    NS_LOG_DEBUG("PAPR reduction for " << s_presets[type].name << ": "
                 << reduction << " dB (OFDM baseline: "
                 << s_presets[WAVEFORM_OFDM].paprDb << " dB, waveform: "
                 << s_presets[type].paprDb << " dB)");

    return reduction;
}

double
ThzNtnWaveform::ComputeDopplerTolerance_Hz(WaveformType type, double scsHz) const
{
    NS_LOG_FUNCTION(this << type << scsHz);

    if (type >= WAVEFORM_COUNT)
    {
        NS_LOG_WARN("Invalid waveform type, returning 0 Hz tolerance");
        return 0.0;
    }

    double tolerance = 0.0;

    switch (type)
    {
    case WAVEFORM_OFDM:
    case WAVEFORM_DFT_S_OFDM:
        // For OFDM-family: maximum tolerable Doppler is a fraction of SCS.
        // The ICI power relative to signal power grows as (f_D / SCS)^2.
        // A common rule-of-thumb is f_D < SCS / 10 for <1 dB degradation.
        // With the Doppler robustness factor (0.3 for OFDM, 0.4 for DFT-s),
        // we scale: tolerance = SCS * robustness_factor
        tolerance = scsHz * s_presets[type].dopplerRobustness;
        break;

    case WAVEFORM_OTFS:
        // OTFS operates in the delay-Doppler domain and can resolve
        // Doppler shifts up to half the subcarrier spacing times the
        // number of Doppler bins.  Effectively, the tolerance is much
        // higher — bounded by the entire bandwidth divided by number
        // of delay bins.  For simplicity, model as:
        //   tolerance = scsHz * N_doppler_bins * robustness_factor
        // With typical N = 128 Doppler bins:
        tolerance = scsHz * 128.0 * s_presets[type].dopplerRobustness;
        break;

    case WAVEFORM_AFDM:
        // AFDM chirp structure provides inherent Doppler resilience.
        // The chirp rate can be tuned to match the Doppler profile.
        // Tolerance scales with SCS and number of chirp cycles:
        tolerance = scsHz * 64.0 * s_presets[type].dopplerRobustness;
        break;

    case WAVEFORM_SC_FDE:
        // SC-FDE has moderate Doppler tolerance.  The frequency-domain
        // equaliser can track Doppler within its convergence range:
        tolerance = scsHz * s_presets[type].dopplerRobustness * 2.0;
        break;

    default:
        tolerance = scsHz * 0.1;
        break;
    }

    NS_LOG_DEBUG("DopplerTolerance for " << s_presets[type].name << " at SCS="
                 << scsHz / 1e3 << " kHz: " << tolerance / 1e3 << " kHz");

    return tolerance;
}

double
ThzNtnWaveform::ComputeOtfsGridSize(double maxDelaySpread_us,
                                     double maxDopplerHz) const
{
    NS_LOG_FUNCTION(this << maxDelaySpread_us << maxDopplerHz);

    // OTFS grid dimensions M x N:
    //
    // M (delay dimension / subcarriers):
    //   Must resolve the maximum delay spread.  With bandwidth B and
    //   delay resolution 1/B:
    //     M >= tau_max * B
    //   We use B = 10 GHz (typical THz bandwidth) and add margin:
    //     M = ceil(tau_max_us * 1e-6 * B * 1.2)
    //
    // N (Doppler dimension / time slots):
    //   Must resolve the maximum Doppler spread.  With frame duration T
    //   and Doppler resolution 1/T:
    //     N >= f_D_max * T
    //   For a practical frame of 1 ms:
    //     N = ceil(f_D_max * T * 1.2)

    double B_Hz = 10e9;            // Assume 10 GHz bandwidth
    double T_frame_s = 1e-3;      // 1 ms frame duration

    double M = std::ceil(maxDelaySpread_us * 1e-6 * B_Hz * 1.2);
    double N = std::ceil(maxDopplerHz * T_frame_s * 1.2);

    // Enforce minimum grid dimensions
    if (M < 16.0)
    {
        M = 16.0;
    }
    if (N < 16.0)
    {
        N = 16.0;
    }

    // Round up to next power of 2 for efficient FFT processing
    M = std::pow(2.0, std::ceil(std::log2(M)));
    N = std::pow(2.0, std::ceil(std::log2(N)));

    double gridSize = M * N;

    NS_LOG_DEBUG("OTFS grid: tau_max=" << maxDelaySpread_us << " us, f_D_max="
                 << maxDopplerHz / 1e3 << " kHz -> M=" << M << ", N=" << N
                 << ", total=" << gridSize);

    return gridSize;
}

double
ThzNtnWaveform::ComputeAfdmChirpRate(double maxDopplerHz, double bandwidthHz) const
{
    NS_LOG_FUNCTION(this << maxDopplerHz << bandwidthHz);

    // AFDM symbol duration: T_sym = N / B where N is the number of
    // subcarriers.  For a typical THz system with 10 GHz bandwidth
    // and 4096 subcarriers: T_sym ~ 0.4 us.
    //
    // The chirp rate must satisfy:
    //   chirp_rate >= 2 * f_D_max / T_sym
    //             = 2 * f_D_max * B / N
    //
    // We use N = 4096 as a reference and add 20% margin.

    double N_subcarriers = 4096.0;
    double T_sym = N_subcarriers / bandwidthHz;

    // Chirp rate with 20% margin
    double chirpRate = 1.2 * 2.0 * maxDopplerHz / T_sym;

    NS_LOG_DEBUG("AFDM chirp rate: f_D_max=" << maxDopplerHz / 1e3
                 << " kHz, BW=" << bandwidthHz / 1e9 << " GHz, T_sym="
                 << T_sym * 1e6 << " us, chirp_rate=" << chirpRate / 1e9
                 << " GHz/s");

    return chirpRate;
}

uint32_t
ThzNtnWaveform::ComputeProcessingComplexity(WaveformType type,
                                             uint32_t numSubcarriers) const
{
    NS_LOG_FUNCTION(this << type << numSubcarriers);

    if (type >= WAVEFORM_COUNT)
    {
        NS_LOG_WARN("Invalid waveform type, returning 0 complexity");
        return 0;
    }

    // Baseline OFDM complexity: N * log2(N) for FFT
    double N = static_cast<double>(numSubcarriers);
    double baseComplexity = N * std::log2(N);

    // Scale by waveform complexity factor
    double scaledComplexity = baseComplexity * s_presets[type].complexityFactor;

    // Additional waveform-specific overhead:
    switch (type)
    {
    case WAVEFORM_OFDM:
        // Baseline — no additional overhead
        break;

    case WAVEFORM_DFT_S_OFDM:
        // Extra DFT precoding stage: adds another N*log2(N)
        scaledComplexity += baseComplexity * 0.2;
        break;

    case WAVEFORM_OTFS:
        // 2D SFFT (symplectic finite Fourier transform) over M x N grid:
        // O(MN * (log M + log N))
        // Approximated as 2x the 1D FFT complexity at same total size
        scaledComplexity += baseComplexity * 2.0;
        break;

    case WAVEFORM_AFDM:
        // Chirp generation and matched filtering:
        // Additional N*log2(N) for chirp demodulation
        scaledComplexity += baseComplexity * 1.5;
        break;

    case WAVEFORM_SC_FDE:
        // FDE requires FFT + channel inversion + IFFT:
        // But single-carrier, so overall slightly less than OFDM
        break;

    default:
        break;
    }

    uint32_t result = static_cast<uint32_t>(std::round(scaledComplexity));

    NS_LOG_DEBUG("ProcessingComplexity for " << s_presets[type].name
                 << " with N=" << numSubcarriers << ": " << result
                 << " relative FLOPS (baseline OFDM FFT: "
                 << static_cast<uint32_t>(baseComplexity) << ")");

    return result;
}

} // namespace ns3
