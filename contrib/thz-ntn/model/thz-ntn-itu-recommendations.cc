/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#include "thz-ntn-itu-recommendations.h"

#include "thz-ntn-molecular-absorption.h"

#include "ns3/double.h"
#include "ns3/enum.h"
#include "ns3/log.h"

#include <algorithm>
#include <array>
#include <cmath>

namespace ns3
{
namespace itu
{

NS_LOG_COMPONENT_DEFINE("ThzNtnItuRecommendations");

// ---------------------------------------------------------------------------
//  P.838-3 rain k / alpha coefficients
// ---------------------------------------------------------------------------

namespace
{

/// ITU-R P.838-3 Annex 1 Tables 1 & 2 at canonical frequencies (GHz).
/// Each row: (f_ghz, k_h, alpha_h, k_v, alpha_v). Values at frequencies
/// between rows are log-log interpolated.
struct PRow
{
    double f_ghz;
    double k_h;
    double alpha_h;
    double k_v;
    double alpha_v;
};

constexpr std::array<PRow, 21> kP838Table = {{
    // Below 10 GHz the table is sparse; we use the published 1-10 GHz entries
    // verbatim. Toolkit consumers operate mostly above 10 GHz.
    {1.0,    0.0000259, 0.9691, 0.0000308, 0.8592},
    {2.0,    0.0000847, 1.0664, 0.0000998, 0.9490},  // k_h/k_v: P.838-3 (were 10x too high)
    {4.0,    0.0001071, 1.6009, 0.0002461, 1.2476},
    {6.0,    0.00175,   1.3088, 0.00149,   1.1825},
    {7.0,    0.00301,   1.3320, 0.00228,   1.1825},
    {8.0,    0.00454,   1.3270, 0.00395,   1.1664},
    {10.0,   0.0101,    1.2760, 0.00887,   1.1640},
    {12.0,   0.0188,    1.2170, 0.0168,    1.1500},
    {15.0,   0.0367,    1.1540, 0.0335,    1.1280},
    {20.0,   0.0751,    1.0990, 0.0691,    1.0650},
    {25.0,   0.1240,    1.0610, 0.1130,    1.0300},
    {30.0,   0.1870,    1.0210, 0.1670,    1.0000},
    {35.0,   0.2630,    0.9790, 0.2330,    0.9630},
    {40.0,   0.3500,    0.9390, 0.3100,    0.9290},
    {45.0,   0.4420,    0.9030, 0.3930,    0.8970},
    {50.0,   0.5360,    0.8730, 0.4790,    0.8680},
    {60.0,   0.7070,    0.8260, 0.6420,    0.8240},
    {70.0,   0.8510,    0.7930, 0.7840,    0.7930},
    {80.0,   0.9750,    0.7690, 0.9060,    0.7690},
    {90.0,   1.0600,    0.7530, 0.9990,    0.7540},
    {100.0,  1.1200,    0.7430, 1.0600,    0.7440},
}};

double
LogLogInterp(double x, double x0, double x1, double y0, double y1)
{
    if (x <= x0)
        return y0;
    if (x >= x1)
        return y1;
    const double lx0 = std::log(x0);
    const double lx1 = std::log(x1);
    const double ly0 = std::log(y0);
    const double ly1 = std::log(y1);
    const double t = (std::log(x) - lx0) / (lx1 - lx0);
    return std::exp(ly0 + t * (ly1 - ly0));
}

} // namespace

std::pair<double, double>
Itu838RainModel::GetKAlpha(double freqHz, Polarization pol)
{
    const double fGhz = std::max(1.0, std::min(freqHz / 1e9, 100.0));
    auto it = std::upper_bound(kP838Table.begin(), kP838Table.end(), fGhz,
                                [](double v, const PRow& r) {
                                    return v < r.f_ghz;
                                });
    if (it == kP838Table.begin())
    {
        ++it;
    }
    if (it == kP838Table.end())
    {
        --it;
    }
    const PRow& hi = *it;
    const PRow& lo = *(it - 1);
    const double k_h = LogLogInterp(fGhz, lo.f_ghz, hi.f_ghz, lo.k_h, hi.k_h);
    const double a_h =
        LogLogInterp(fGhz, lo.f_ghz, hi.f_ghz, lo.alpha_h, hi.alpha_h);
    const double k_v = LogLogInterp(fGhz, lo.f_ghz, hi.f_ghz, lo.k_v, hi.k_v);
    const double a_v =
        LogLogInterp(fGhz, lo.f_ghz, hi.f_ghz, lo.alpha_v, hi.alpha_v);

    switch (pol)
    {
    case Polarization::horizontal:
        return {k_h, a_h};
    case Polarization::vertical:
        return {k_v, a_v};
    case Polarization::circular:
    default:
        return {0.5 * (k_h + k_v), 0.5 * (a_h + a_v)};
    }
}

double
Itu838RainModel::SpecificAttenuationDbKm(double rainRate_mm_h,
                                          double freqHz,
                                          Polarization pol)
{
    if (rainRate_mm_h <= 0.0)
    {
        return 0.0;
    }
    const auto [k, alpha] = GetKAlpha(freqHz, pol);
    return k * std::pow(rainRate_mm_h, alpha);
}

// ---------------------------------------------------------------------------
//  P.618-13 slant-path rain attenuation
// ---------------------------------------------------------------------------

TypeId
Itu618LossModel::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::itu::Itu618LossModel")
            .SetParent<Object>()
            .SetGroupName("ThzNtn")
            .AddConstructor<Itu618LossModel>();
    return tid;
}

double
Itu618LossModel::GetRainHeightKm() const
{
    switch (m_region)
    {
    case ClimateRegion::tropical:
        return 5.0;
    case ClimateRegion::midlat_summer:
        return 3.5;
    case ClimateRegion::midlat_winter:
        return 2.0;
    case ClimateRegion::subarctic:
        return 1.5;
    }
    return 3.5;
}

double
Itu618LossModel::EffectivePathLengthFactor(double pathLengthKm,
                                             double rainRate_mm_h,
                                             double freqHz)
{
    // P.618-13 §2.2.1.1 path reduction factor. The closed form blends
    // rain-rate, frequency, and physical slant length so that high rates
    // over long paths get a sub-unity factor (multipath averaging).
    if (rainRate_mm_h <= 0.0 || pathLengthKm <= 0.0)
    {
        return 1.0;
    }
    const double fGhz = freqHz / 1e9;
    const double r =
        1.0 / (1.0 + 0.78 * std::sqrt(pathLengthKm * rainRate_mm_h / fGhz) -
               0.38 * (1.0 - std::exp(-2.0 * pathLengthKm)));
    return std::max(0.05, std::min(1.0, r));
}

double
Itu618LossModel::SlantPathRainAttenuationDb(double freqHz,
                                              double elevationDeg,
                                              double rainRate_mm_h,
                                              double groundAlt_km,
                                              Polarization pol) const
{
    if (rainRate_mm_h <= 0.0)
    {
        return 0.0;
    }

    const double hR = GetRainHeightKm();
    if (hR <= groundAlt_km)
    {
        return 0.0;
    }
    const double elevRad = elevationDeg * M_PI / 180.0;
    const double sinElev = std::sin(std::max(elevRad, 5.0 * M_PI / 180.0));
    const double Ls = (hR - groundAlt_km) / sinElev; // slant length in km

    // P.530 horizontal projection — used in the reduction factor calc.
    const double Lg = Ls * std::cos(elevRad);
    const double r =
        EffectivePathLengthFactor(Lg, rainRate_mm_h, freqHz);

    const double gamma =
        Itu838RainModel::SpecificAttenuationDbKm(rainRate_mm_h, freqHz, pol);
    return gamma * Ls * r;
}

// ---------------------------------------------------------------------------
//  P.676-13 gaseous attenuation (wraps the in-process molecular absorption)
// ---------------------------------------------------------------------------

TypeId
Itu676AbsorptionModel::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::itu::Itu676AbsorptionModel")
            .SetParent<Object>()
            .SetGroupName("ThzNtn")
            .AddConstructor<Itu676AbsorptionModel>();
    return tid;
}

Itu676AbsorptionModel::Itu676AbsorptionModel() = default;

double
Itu676AbsorptionModel::SpecificAttenuationDbKm(double freqHz,
                                                 double altKm) const
{
    Ptr<ThzNtnMolecularAbsorption> abs =
        CreateObject<ThzNtnMolecularAbsorption>();
    const double trans = abs->GetTransmittance(freqHz, 1000.0, altKm);
    return -10.0 * std::log10(std::max(trans, 1e-30));
}

double
Itu676AbsorptionModel::SlantPathAttenuationDb(double freqHz,
                                                double elevationDeg) const
{
    Ptr<ThzNtnMolecularAbsorption> abs =
        CreateObject<ThzNtnMolecularAbsorption>();
    return abs->ComputeSlantPathAbsorption(freqHz, elevationDeg, 0.0, 100.0);
}

// ---------------------------------------------------------------------------
//  P.681-11 Land Mobile Satellite Lutz 2-state model
// ---------------------------------------------------------------------------

TypeId
Itu681LmsModel::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::itu::Itu681LmsModel")
            .SetParent<Object>()
            .SetGroupName("ThzNtn")
            .AddConstructor<Itu681LmsModel>();
    return tid;
}

Itu681LmsModel::Itu681LmsModel()
    : m_uniform(CreateObject<UniformRandomVariable>()),
      m_normal(CreateObject<NormalRandomVariable>()),
      m_exp(CreateObject<ExponentialRandomVariable>())
{
    ApplyEnvironment();
}

void
Itu681LmsModel::SetEnvironment(Environment env)
{
    m_env = env;
    ApplyEnvironment();
}

void
Itu681LmsModel::ApplyEnvironment()
{
    // Per-environment parameters approximating P.681-11 Table 1 trends.
    // Urban -> high P_bad, large multipath fade; open -> P_bad ~ 0, only
    // residual Rice. Mid-band values pick reasonable defaults; site-
    // specific calibration is the user's job at the example layer.
    switch (m_env)
    {
    case Environment::urban:
        m_pAtoB = 0.30;
        m_pBtoA = 0.20;
        m_riceK_dB = 5.0;
        m_looMu_dB = -12.0;
        m_looSigma_dB = 4.0;
        m_looMultipathDb = 8.0;
        break;
    case Environment::suburban:
        m_pAtoB = 0.10;
        m_pBtoA = 0.30;
        m_riceK_dB = 10.0;
        m_looMu_dB = -8.0;
        m_looSigma_dB = 3.0;
        m_looMultipathDb = 5.0;
        break;
    case Environment::rural:
        m_pAtoB = 0.04;
        m_pBtoA = 0.40;
        m_riceK_dB = 14.0;
        m_looMu_dB = -5.0;
        m_looSigma_dB = 2.5;
        m_looMultipathDb = 3.0;
        break;
    case Environment::open:
        m_pAtoB = 0.005;
        m_pBtoA = 0.50;
        m_riceK_dB = 18.0;
        m_looMu_dB = -3.0;
        m_looSigma_dB = 1.0;
        m_looMultipathDb = 1.5;
        break;
    }
    // Steady-state P(B) = p_AB / (p_AB + p_BA)
    const double denom = m_pAtoB + m_pBtoA;
    m_pBadSteady = (denom > 0.0) ? (m_pAtoB / denom) : 0.0;
}

double
Itu681LmsModel::StepDb()
{
    // Markov step.
    const double u = m_uniform->GetValue();
    if (!m_inBad && u < m_pAtoB)
    {
        m_inBad = true;
    }
    else if (m_inBad && u < m_pBtoA)
    {
        m_inBad = false;
    }

    // Sample fade depth in dB given the new state.
    if (!m_inBad)
    {
        // State A: Rician — most realisations are near 0 dB attenuation.
        // Approximate with K-factor: fade variance ~ 1/(K+1).
        const double sigma = 1.0 / std::sqrt(1.0 + std::pow(10.0,
                                                              m_riceK_dB / 10.0));
        const double sample = std::abs(m_normal->GetValue(0.0, sigma));
        // Convert linear fade magnitude to dB (small positive). Clamp.
        return std::min(15.0, sample * 5.0);
    }
    // State B: Loo distribution combining log-normal direct (mu, sigma)
    // with Rayleigh multipath. Returns positive fade depth in dB.
    const double a_dB = m_normal->GetValue(m_looMu_dB, m_looSigma_dB);
    const double r_dB = m_looMultipathDb * m_exp->GetValue() / m_exp->GetMean();
    return std::min(60.0, std::max(0.0, -a_dB + r_dB));
}

int64_t
Itu681LmsModel::AssignStreams(int64_t stream)
{
    m_uniform->SetStream(stream);
    m_normal->SetStream(stream + 1);
    m_exp->SetStream(stream + 2);
    return 3;
}

} // namespace itu
} // namespace ns3
