/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#include "thz-ntn-itu-recommendations.h"

#include "thz-ntn-molecular-absorption.h"

#include "ns3/double.h"
#include "ns3/enum.h"
#include "ns3/log.h"
#include "ns3/simulator.h"

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

// ITU-R P.838-3 Annex 1 log-Gaussian coefficient model. P.838-3 does NOT
// tabulate k and alpha; it defines them as closed-form functions of
// log10(frequency):
//
//   log10 k     = sum_j a_j exp(-((log10 f - b_j)/c_j)^2) + m_k log10 f + c_k
//   alpha       = sum_j a_j exp(-((log10 f - b_j)/c_j)^2) + m_a log10 f + c_a
//
// with a 4-term Gaussian sum for k and a 5-term sum for alpha, for each of
// horizontal and vertical polarization. Coefficients are from ITU-R P.838-3
// Tables 1-4 (verbatim). Valid 1-1000 GHz. These replace the module's former
// P.838-1 (1999) lookup values, which under-predicted Ku/Ka rain attenuation
// by 20-25% (e.g. k_h(20) was 0.0751 vs the P.838-3 value 0.09164).

struct GaussTerm
{
    double a;
    double b;
    double c;
};

// k coefficients: 4 Gaussian terms + linear (m, c).
constexpr std::array<GaussTerm, 4> kKh = {{{-5.33980, -0.10008, 1.13098},
                                           {-0.35351, 1.26970, 0.45400},
                                           {-0.23789, 0.86036, 0.15354},
                                           {-0.94158, 0.64552, 0.16817}}};
constexpr double kKhM = -0.18961;
constexpr double kKhC = 0.71147;

constexpr std::array<GaussTerm, 4> kKv = {{{-3.80595, 0.56934, 0.81061},
                                           {-3.44965, -0.22911, 0.51059},
                                           {-0.39902, 0.73042, 0.11899},
                                           {0.50167, 1.07319, 0.27195}}};
constexpr double kKvM = -0.16398;
constexpr double kKvC = 0.63297;

// alpha coefficients: 5 Gaussian terms + linear (m, c).
constexpr std::array<GaussTerm, 5> kAh = {{{-0.14318, 1.82442, -0.55187},
                                           {0.29591, 0.77564, 0.19822},
                                           {0.32177, 0.63773, 0.13164},
                                           {-5.37610, -0.96230, 1.47828},
                                           {16.1721, -3.29980, 3.43990}}};
constexpr double kAhM = 0.67849;
constexpr double kAhC = -1.95537;

constexpr std::array<GaussTerm, 5> kAv = {{{-0.07771, 2.33840, -0.76284},
                                           {0.56727, 0.95545, 0.54039},
                                           {-0.20238, 1.14520, 0.26809},
                                           {-48.2991, 0.791669, 0.116226},
                                           {48.5833, 0.791459, 0.116479}}};
constexpr double kAvM = -0.053739;
constexpr double kAvC = 0.83433;

template <std::size_t N>
double
LogGaussianSum(double logf, const std::array<GaussTerm, N>& terms, double m,
               double c)
{
    double s = 0.0;
    for (const auto& t : terms)
    {
        const double z = (logf - t.b) / t.c;
        s += t.a * std::exp(-z * z);
    }
    return s + m * logf + c;
}

} // namespace

std::pair<double, double>
Itu838RainModel::GetKAlpha(double freqHz, Polarization pol)
{
    // P.838-3 is specified for 1-1000 GHz.
    const double fGhz = std::max(1.0, std::min(freqHz / 1e9, 1000.0));
    const double lf = std::log10(fGhz);

    const double k_h = std::pow(10.0, LogGaussianSum(lf, kKh, kKhM, kKhC));
    const double a_h = LogGaussianSum(lf, kAh, kAhM, kAhC);
    const double k_v = std::pow(10.0, LogGaussianSum(lf, kKv, kKvM, kKvC));
    const double a_v = LogGaussianSum(lf, kAv, kAvM, kAvC);

    switch (pol)
    {
    case Polarization::horizontal:
        return {k_h, a_h};
    case Polarization::vertical:
        return {k_v, a_v};
    case Polarization::circular:
    default:
        // P.838-3 eq. for tau=45 deg (circular): the cos(2 tau) term
        // vanishes, leaving the mean of the H and V coefficients.
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
    // Representative rain height h_R = h0 + 0.36 km (ITU-R P.618-13 §2.2.1.1
    // step 1, with h0 the 0 deg-C isotherm height from ITU-R P.839). The
    // four climate regions carry representative P.839 h0 values:
    //   tropical      h0 ~ 4.6 km -> h_R ~ 5.0 km
    //   midlat-summer h0 ~ 3.1 km -> h_R ~ 3.5 km
    //   midlat-winter h0 ~ 1.6 km -> h_R ~ 2.0 km
    //   subarctic     h0 ~ 1.1 km -> h_R ~ 1.5 km
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
Itu618LossModel::EffectivePathLengthFactor(double horizProjLenKm,
                                             double gammaR_dBkm,
                                             double freqHz)
{
    // ITU-R P.618-13 §2.2.1.1 step 6: horizontal path-reduction factor
    //   r_0.01 = 1 / (1 + 0.78 sqrt(L_G gamma_R / f) - 0.38 (1 - e^{-2 L_G}))
    // The argument of the square root is the horizontal projection L_G (km)
    // times the SPECIFIC ATTENUATION gamma_R (dB/km), divided by frequency
    // (GHz) — not the rain rate.
    if (gammaR_dBkm <= 0.0 || horizProjLenKm <= 0.0)
    {
        return 1.0;
    }
    const double fGhz = freqHz / 1e9;
    const double r =
        1.0 / (1.0 + 0.78 * std::sqrt(horizProjLenKm * gammaR_dBkm / fGhz) -
               0.38 * (1.0 - std::exp(-2.0 * horizProjLenKm)));
    return std::max(0.05, std::min(1.0, r));
}

double
Itu618LossModel::SlantPathRainAttenuationDb(double freqHz,
                                              double elevationDeg,
                                              double rainRate_mm_h,
                                              double groundAlt_km,
                                              Polarization pol) const
{
    // Full ITU-R P.618-13 §2.2.1.1 slant-path rain attenuation (A_0.01).
    if (rainRate_mm_h <= 0.0)
    {
        return 0.0;
    }

    const double hR = GetRainHeightKm();               // step 1: rain height
    if (hR <= groundAlt_km)
    {
        return 0.0;
    }

    const double elevDeg = std::max(elevationDeg, 5.0);
    const double elevRad = elevDeg * M_PI / 180.0;
    const double sinElev = std::sin(elevRad);
    const double cosElev = std::cos(elevRad);

    // step 2: slant path length below the rain height.
    const double Ls = (hR - groundAlt_km) / sinElev;   // km
    // step 3: horizontal projection.
    const double Lg = Ls * cosElev;                    // km

    // step 5: specific attenuation gamma_R = k R^alpha (P.838-3).
    const double gammaR =
        Itu838RainModel::SpecificAttenuationDbKm(rainRate_mm_h, freqHz, pol);

    // step 6: horizontal reduction factor (function of gamma_R, not R).
    const double r = EffectivePathLengthFactor(Lg, gammaR, freqHz);

    // step 7: vertical adjustment factor v_0.01.
    const double fGhz = freqHz / 1e9;
    const double zetaDeg =
        std::atan2(hR - groundAlt_km, Lg * r) * 180.0 / M_PI;
    double Lr;
    if (zetaDeg > elevDeg)
    {
        Lr = Lg * r / cosElev;                         // km
    }
    else
    {
        Lr = (hR - groundAlt_km) / sinElev;            // km
    }
    const double absLat = std::abs(m_latDeg);
    const double chi = (absLat < 36.0) ? (36.0 - absLat) : 0.0;   // deg
    const double vDenom =
        1.0 + std::sqrt(sinElev) *
                  (31.0 * (1.0 - std::exp(-elevDeg / (1.0 + chi))) *
                       std::sqrt(Lr * gammaR) / (fGhz * fGhz) -
                   0.45);
    // Guard against a pathological non-positive denominator.
    const double v = (vDenom > 1e-3) ? (1.0 / vDenom) : 1.0;

    // step 8: effective path length, and step 9: attenuation.
    const double Le = Lr * v;                          // km
    return gammaR * Le;                                // dB
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
    // residual Rice. The Lutz state durations are characteristic DISTANCES
    // (metres): the good/bad run-lengths set the steady-state occupancy
    // P(bad) = D_bad / (D_good + D_bad), and the dwell TIME follows from the
    // terminal speed. Mid-band values pick reasonable defaults; site-
    // specific calibration is the user's job at the example layer.
    switch (m_env)
    {
    case Environment::urban:
        m_dGood_m = 16.0;   // P(bad) = 24/(16+24) = 0.60
        m_dBad_m = 24.0;
        m_riceK_dB = 5.0;
        m_looMu_dB = -12.0;
        m_looSigma_dB = 4.0;
        m_looMultipathDb = 8.0;
        break;
    case Environment::suburban:
        m_dGood_m = 30.0;   // P(bad) = 10/(30+10) = 0.25
        m_dBad_m = 10.0;
        m_riceK_dB = 10.0;
        m_looMu_dB = -8.0;
        m_looSigma_dB = 3.0;
        m_looMultipathDb = 5.0;
        break;
    case Environment::rural:
        m_dGood_m = 40.0;   // P(bad) = 4/(40+4) = 0.0909
        m_dBad_m = 4.0;
        m_riceK_dB = 14.0;
        m_looMu_dB = -5.0;
        m_looSigma_dB = 2.5;
        m_looMultipathDb = 3.0;
        break;
    case Environment::open:
        m_dGood_m = 40.0;   // P(bad) = 0.4/(40+0.4) = 0.0099
        m_dBad_m = 0.4;
        m_riceK_dB = 18.0;
        m_looMu_dB = -3.0;
        m_looSigma_dB = 1.0;
        m_looMultipathDb = 1.5;
        break;
    }
    // Steady-state P(B) = D_bad / (D_good + D_bad)
    const double denom = m_dGood_m + m_dBad_m;
    m_pBadSteady = (denom > 0.0) ? (m_dBad_m / denom) : 0.0;
}

double
Itu681LmsModel::StepDb()
{
    // Convert elapsed sim-time since the previous poll into travelled
    // distance, so the Lutz transitions are per-distance (independent of the
    // packet/polling rate).
    const Time now = Simulator::Now();
    double dt = (now - m_lastPollTime).GetSeconds();
    if (dt < 0.0)
    {
        dt = 0.0;
    }
    m_lastPollTime = now;
    return StepByDistance(m_speedMps * dt);
}

double
Itu681LmsModel::StepByDistance(double distanceM)
{
    // Distance-parametrised Lutz transition. Over a travelled distance dx,
    // the probability of leaving the current state is 1 - exp(-dx / D_state),
    // with D_state the mean run length (metres). At dx = 0 no transition
    // occurs — dwell is governed by distance, not by call count.
    const double dx = std::max(0.0, distanceM);
    if (!m_inBad)
    {
        const double pLeave =
            (m_dGood_m > 0.0) ? (1.0 - std::exp(-dx / m_dGood_m)) : 0.0;
        if (m_uniform->GetValue() < pLeave)
        {
            m_inBad = true;
        }
    }
    else
    {
        const double pLeave =
            (m_dBad_m > 0.0) ? (1.0 - std::exp(-dx / m_dBad_m)) : 0.0;
        if (m_uniform->GetValue() < pLeave)
        {
            m_inBad = false;
        }
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
    // Anchor the mobility clock so the first poll measures elapsed time from
    // here rather than from t=0.
    m_lastPollTime = Simulator::Now();
    return 3;
}

} // namespace itu
} // namespace ns3
