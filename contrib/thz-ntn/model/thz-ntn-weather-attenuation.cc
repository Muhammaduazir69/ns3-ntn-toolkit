/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2024 Muhammad Uzair
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Author: Muhammad Uzair <uk5595985@gmail.com>
 */

#include "thz-ntn-itu-recommendations.h"

#include "thz-ntn-weather-attenuation.h"

#include <sstream>

#include <ns3/boolean.h>
#include <ns3/double.h>
#include <ns3/enum.h>
#include <ns3/log.h>

#include <algorithm>
#include <cmath>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("ThzNtnWeatherAttenuation");

NS_OBJECT_ENSURE_REGISTERED(ThzNtnWeatherAttenuation);

TypeId
ThzNtnWeatherAttenuation::GetTypeId(void)
{
    static TypeId tid =
        TypeId("ns3::ThzNtnWeatherAttenuation")
            .SetParent<Object>()
            .SetGroupName("ThzNtn")
            .AddConstructor<ThzNtnWeatherAttenuation>()
            .AddAttribute("RainRate",
                          "Rain rate in mm/h (0 = no rain). "
                          "Typical values: light 2, moderate 10, heavy 50, extreme 100.",
                          DoubleValue(0.0),
                          MakeDoubleAccessor(&ThzNtnWeatherAttenuation::m_rainRate),
                          MakeDoubleChecker<double>(0.0))
            .AddAttribute("LiquidWaterContent",
                          "Liquid water content in g/m^3 for fog/cloud attenuation. "
                          "Typical: 0.05 (light fog), 0.25 (moderate), 0.5 (dense).",
                          DoubleValue(0.0),
                          MakeDoubleAccessor(&ThzNtnWeatherAttenuation::m_liquidWaterContent),
                          MakeDoubleChecker<double>(0.0))
            .AddAttribute("SnowRate",
                          "Snow rate in mm/h (liquid-water equivalent).",
                          DoubleValue(0.0),
                          MakeDoubleAccessor(&ThzNtnWeatherAttenuation::m_snowRate),
                          MakeDoubleChecker<double>(0.0))
            .AddAttribute("DustVisibility",
                          "Visibility in km during sand/dust storm (100 = clear).",
                          DoubleValue(100.0),
                          MakeDoubleAccessor(&ThzNtnWeatherAttenuation::m_dustVisibility),
                          MakeDoubleChecker<double>(0.001))
            .AddAttribute("EnableRain",
                          "Enable rain attenuation computation.",
                          BooleanValue(false),
                          MakeBooleanAccessor(&ThzNtnWeatherAttenuation::m_enableRain),
                          MakeBooleanChecker())
            .AddAttribute("EnableFog",
                          "Enable fog/cloud attenuation computation.",
                          BooleanValue(false),
                          MakeBooleanAccessor(&ThzNtnWeatherAttenuation::m_enableFog),
                          MakeBooleanChecker())
            .AddAttribute("EnableSnow",
                          "Enable snow/ice attenuation computation.",
                          BooleanValue(false),
                          MakeBooleanAccessor(&ThzNtnWeatherAttenuation::m_enableSnow),
                          MakeBooleanChecker())
            .AddAttribute("EnableDust",
                          "Enable sand/dust storm attenuation computation.",
                          BooleanValue(false),
                          MakeBooleanAccessor(&ThzNtnWeatherAttenuation::m_enableDust),
                          MakeBooleanChecker())
            .AddAttribute("WetSnow",
                          "True for wet snow, false for dry snow.",
                          BooleanValue(false),
                          MakeBooleanAccessor(&ThzNtnWeatherAttenuation::m_wetSnow),
                          MakeBooleanChecker())
            .AddAttribute("ClimateRegion",
                          "Climate region for rain height estimation (ITU-R P.839).",
                          EnumValue(ThzNtnWeatherAttenuation::MIDLAT_SUMMER),
                          MakeEnumAccessor<ClimateRegion>(
                              &ThzNtnWeatherAttenuation::m_climateRegion),
                          MakeEnumChecker(ThzNtnWeatherAttenuation::TROPICAL,
                                          "Tropical",
                                          ThzNtnWeatherAttenuation::MIDLAT_SUMMER,
                                          "MidLatSummer",
                                          ThzNtnWeatherAttenuation::MIDLAT_WINTER,
                                          "MidLatWinter"))
            .AddAttribute("StationHeight",
                          "Ground station height above mean sea level in km.",
                          DoubleValue(0.0),
                          MakeDoubleAccessor(&ThzNtnWeatherAttenuation::m_stationHeight),
                          MakeDoubleChecker<double>(0.0))
            .AddAttribute("Temperature",
                          "Ambient temperature in Kelvin for Debye model.",
                          DoubleValue(293.0),
                          MakeDoubleAccessor(&ThzNtnWeatherAttenuation::m_temperature),
                          MakeDoubleChecker<double>(200.0, 330.0))
            .AddAttribute("FogLayerHeight",
                          "Fog/cloud layer thickness in km.",
                          DoubleValue(0.5),
                          MakeDoubleAccessor(&ThzNtnWeatherAttenuation::m_fogLayerHeight),
                          MakeDoubleChecker<double>(0.0))
            .AddAttribute("SnowLayerHeight",
                          "Snow layer thickness in km.",
                          DoubleValue(1.0),
                          MakeDoubleAccessor(&ThzNtnWeatherAttenuation::m_snowLayerHeight),
                          MakeDoubleChecker<double>(0.0))
            .AddAttribute("DustLayerHeight",
                          "Dust storm layer thickness in km.",
                          DoubleValue(1.5),
                          MakeDoubleAccessor(&ThzNtnWeatherAttenuation::m_dustLayerHeight),
                          MakeDoubleChecker<double>(0.0));
    return tid;
}

ThzNtnWeatherAttenuation::ThzNtnWeatherAttenuation()
    : m_rainRate(0.0),
      m_liquidWaterContent(0.0),
      m_snowRate(0.0),
      m_wetSnow(false),
      m_dustVisibility(100.0),
      m_enableRain(false),
      m_enableFog(false),
      m_enableSnow(false),
      m_enableDust(false),
      m_climateRegion(MIDLAT_SUMMER),
      m_stationHeight(0.0),
      m_temperature(293.0),
      m_fogLayerHeight(0.5),
      m_snowLayerHeight(1.0),
      m_dustLayerHeight(1.5)
{
    NS_LOG_FUNCTION(this);
    InitRainCoefficients();
}

ThzNtnWeatherAttenuation::~ThzNtnWeatherAttenuation()
{
    NS_LOG_FUNCTION(this);
}

void
ThzNtnWeatherAttenuation::InitRainCoefficients()
{
    NS_LOG_FUNCTION(this);

    // Rain specific-attenuation coefficients {freqGHz, kH, kV, alphaH, alphaV}.
    // 1-100 GHz: computed from the ITU-R P.838-3 Annex 1 log-Gaussian
    //   coefficient formulas (the recommendation defines k and alpha as
    //   closed-form functions of log10 f — it has NO lookup table). These
    //   replace the earlier P.838-1 (1999) values, which under-predicted
    //   Ku/Ka rain attenuation by 20-25% (e.g. k_h(20) was 0.0751 vs the
    //   P.838-3 value 0.09164).
    // >100 GHz: extended using Mie scattering for a Marshall-Palmer raindrop
    //   size distribution; at THz the drop diameter (~1 mm) is comparable to
    //   the wavelength (0.3-3 mm), so Mie scattering dominates.

    m_rainCoeffTable = {
        {1.0,    0.0000259, 0.0000308, 0.9691, 0.8592},
        {2.0,    0.0000847, 0.0000998, 1.0664, 0.9490},
        {4.0,    0.0001071, 0.0002461, 1.6009, 1.2475},
        {6.0,    0.0007056, 0.0004878, 1.5900, 1.5728},
        {8.0,    0.0041154, 0.0034498, 1.3905, 1.3797},
        {10.0,   0.0121670, 0.0112919, 1.2571, 1.2156},
        {15.0,   0.0448146, 0.0500825, 1.1233, 1.0440},
        {20.0,   0.0916427, 0.0961112, 1.0568, 0.9847},
        {25.0,   0.1570902, 0.1532685, 0.9991, 0.9491},
        {30.0,   0.2403082, 0.2290903, 0.9485, 0.9129},
        {35.0,   0.3373870, 0.3223760, 0.9047, 0.8761},
        {40.0,   0.4430572, 0.4273753, 0.8673, 0.8421},
        {50.0,   0.6599578, 0.6472147, 0.8084, 0.7871},
        {60.0,   0.8606130, 0.8515201, 0.7656, 0.7486},
        {70.0,   1.0314779, 1.0253337, 0.7345, 0.7215},
        {80.0,   1.1704450, 1.1668310, 0.7115, 0.7021},
        {90.0,   1.2807147, 1.2794572, 0.6944, 0.6876},
        {100.0,  1.3671083, 1.3680473, 0.6815, 0.6765},  // P.838-3 (valid to 1000 GHz)
        {120.0,  1.58,      1.50,      0.760,  0.760},    // >100 GHz: Mie extension
        {150.0,  1.95,      1.86,      0.730,  0.730},
        {200.0,  2.35,      2.25,      0.700,  0.700},
        {250.0,  2.56,      2.46,      0.685,  0.685},
        {300.0,  2.73,      2.63,      0.680,  0.680},  // ~300 GHz: k≈2.73, alpha≈0.68
        {350.0,  2.88,      2.78,      0.670,  0.670},
        {400.0,  3.00,      2.90,      0.660,  0.660},
        {500.0,  3.20,      3.10,      0.640,  0.640},  // ~500 GHz: k≈3.20, alpha≈0.64
        {600.0,  3.35,      3.25,      0.625,  0.625},
        {700.0,  3.48,      3.38,      0.610,  0.610},
        {800.0,  3.58,      3.48,      0.600,  0.600},
        {900.0,  3.66,      3.56,      0.592,  0.592},
        {1000.0, 3.73,      3.63,      0.585,  0.585},
    };
}

void
ThzNtnWeatherAttenuation::InterpolateRainCoefficients(double freqGHz,
                                                      double& k,
                                                      double& alpha) const
{
    NS_LOG_FUNCTION(this << freqGHz);

    // THZ-03 FIX (2026-08-25): delegate to ITU-R P.838-3.
    //
    // This used to interpolate a hand-tabulated k/alpha table whose rows above
    // 100 GHz were labelled "Mie extension" and carried no source. They were
    // not merely unsourced, they were wrong: at 300 GHz the table gave
    // k = 2.73 where P.838-3 gives 1.6286, and at 500 GHz 3.20 against 1.5418,
    // so rain attenuation on the packet path was overstated by 1.7 to 2 times
    // across the entire band the module exists to study. The 100 GHz row
    // matched the spec exactly, which is what made the invented rows above it
    // look like a continuation of real data.
    //
    // The closed form was already implemented in this same module, in
    // Itu838RainModel, and is valid to 1000 GHz - the discarded table's own
    // comment said so on the last real row. Averaging H and V approximates the
    // circular polarization typical of an NTN service link.
    const double freqHz = freqGHz * 1e9;
    const auto ka = itu::Itu838RainModel::GetKAlpha(freqHz, itu::Polarization::circular);
    k = ka.first;
    alpha = ka.second;
}

double
ThzNtnWeatherAttenuation::GetRainHeight() const
{
    // ITU-R P.839: rain height = 0-degree isotherm height + 360 m
    // for non-tropical regions.
    switch (m_climateRegion)
    {
    case TROPICAL:
        return 5.0; // km
    case MIDLAT_SUMMER:
        return 3.5; // 3.14 km isotherm + 0.36 km ≈ 3.5 km
    case MIDLAT_WINTER:
        return 2.0; // 1.64 km isotherm + 0.36 km ≈ 2.0 km
    default:
        return 3.5;
    }
}

double
ThzNtnWeatherAttenuation::ComputeRainSlantPath(double elevationDeg,
                                                double rainHeight_km) const
{
    NS_LOG_FUNCTION(this << elevationDeg << rainHeight_km);

    double deltaH = rainHeight_km - m_stationHeight;
    if (deltaH <= 0.0)
    {
        return 0.0; // Station is above rain height
    }

    double elevRad = elevationDeg * M_PI / 180.0;
    double sinEl = std::sin(elevRad);

    if (elevationDeg >= 5.0)
    {
        // Simple slant path for elevation >= 5 degrees
        double Ls = deltaH / sinEl;

        // Horizontal reduction factor (ITU-R P.530-17, simplified)
        // r = 1 / (1 + L_s / L_0) where L_0 depends on rain rate
        double L0 = 35.0 * std::exp(-0.015 * m_rainRate); // km, effective rain cell size
        if (L0 < 1.0)
        {
            L0 = 1.0;
        }
        double Lh = Ls * std::cos(elevRad); // horizontal projection
        double r = 1.0 / (1.0 + Lh / L0);

        return Ls * r;
    }
    else
    {
        // For low elevation angles (< 5 deg), account for Earth curvature
        // ITU-R P.618 Eq. for low elevations
        double Re = 8500.0; // effective Earth radius in km (4/3 model)
        double Ls = std::sqrt(deltaH * deltaH + 2.0 * Re * deltaH +
                              Re * Re * sinEl * sinEl) -
                    Re * sinEl;

        double Lh = Ls * std::cos(elevRad);
        double L0 = 35.0 * std::exp(-0.015 * m_rainRate);
        if (L0 < 1.0)
        {
            L0 = 1.0;
        }
        double r = 1.0 / (1.0 + Lh / L0);
        return Ls * r;
    }
}

double
ThzNtnWeatherAttenuation::ComputeRainAttenuation_dB(double freqHz,
                                                     double elevationDeg,
                                                     double rainRate_mm_h) const
{
    NS_LOG_FUNCTION(this << freqHz << elevationDeg << rainRate_mm_h);

    if (rainRate_mm_h <= 0.0 || elevationDeg <= 0.0)
    {
        return 0.0;
    }

    double freqGHz = freqHz / 1.0e9;

    // Clamp frequency to valid range
    if (freqGHz < 1.0)
    {
        NS_LOG_WARN("Frequency " << freqGHz << " GHz below rain model range (1 GHz)");
        return 0.0;
    }

    // Get rain coefficients k and alpha for this frequency
    double k = 0.0;
    double alpha = 0.0;
    InterpolateRainCoefficients(freqGHz, k, alpha);

    // Specific attenuation (dB/km): gamma_R = k * R^alpha
    double gammaR = k * std::pow(rainRate_mm_h, alpha);

    // Effective slant path through rain layer
    double rainHeight = GetRainHeight();
    double Leff = ComputeRainSlantPath(elevationDeg, rainHeight);

    double attenuation = gammaR * Leff;

    NS_LOG_DEBUG("Rain: f=" << freqGHz << " GHz, R=" << rainRate_mm_h
                            << " mm/h, k=" << k << ", alpha=" << alpha
                            << ", gamma=" << gammaR << " dB/km"
                            << ", Leff=" << Leff << " km"
                            << ", A=" << attenuation << " dB");

    return attenuation;
}

double
ThzNtnWeatherAttenuation::ComputeKl(double freqGHz, double temperature_K) const
{
    NS_LOG_FUNCTION(this << freqGHz << temperature_K);

    // Compute K_l using the Debye relaxation model for liquid water.
    // Based on ITU-R P.840-8, extended to THz frequencies.
    //
    // The complex permittivity of liquid water (Debye single-relaxation model):
    //   epsilon(f) = epsilon_inf + (epsilon_s - epsilon_inf) / (1 + j*f/f_p)
    //
    // where:
    //   epsilon_s = static permittivity
    //   epsilon_inf = high-frequency permittivity
    //   f_p = principal relaxation frequency

    double theta = 300.0 / temperature_K - 1.0;

    // THZ-04 FIX (2026-08-25): ITU-R P.840 double-Debye, with no frequency
    // branch.
    //
    // This used to switch the intermediate permittivity at exactly 100 GHz:
    //
    //     if (freqGHz > 100.0) epsilon_1 = (epsilon_s - epsilon_inf) * 0.1 + epsilon_inf;
    //     else                 epsilon_1 = epsilon_inf;
    //
    // The 0.1 had no source, and the branch made the specific attenuation
    // coefficient jump 50.8 percent across 2 kHz of frequency at the boundary -
    // a step change in a physical quantity that is continuous in reality, sitting
    // right in the middle of the band this module exists to study. The
    // high-frequency permittivity was also computed as 0.0671 * epsilon_s rather
    // than taken from the recommendation.
    //
    // P.840 specifies the two-relaxation form with FIXED intermediate and
    // high-frequency permittivities, which is continuous by construction: there
    // is nothing to blend and no boundary to cross.
    constexpr double kEpsilon1 = 5.48; //!< P.840 intermediate permittivity
    constexpr double kEpsilon2 = 3.51; //!< P.840 high-frequency permittivity

    // Static permittivity of water (ITU-R P.840)
    double epsilon_s = 77.66 + 103.3 * theta;
    double epsilon_inf = kEpsilon2;
    double epsilon_1 = kEpsilon1;

    // Principal relaxation frequency in GHz (ITU-R P.840)
    double fp = 20.20 - 146.4 * theta + 316.0 * theta * theta;

    // Secondary relaxation frequency in GHz (ITU-R P.840)
    double fs = 39.8 * fp;

    // Complex permittivity components (double-Debye)
    double f = freqGHz;
    double term1_denom = 1.0 + (f / fp) * (f / fp);
    double term2_denom = 1.0 + (f / fs) * (f / fs);

    double epsilon_real = kEpsilon2 +
                          (epsilon_s - epsilon_1) / term1_denom +
                          (epsilon_1 - kEpsilon2) / term2_denom;

    double epsilon_imag = (f / fp) * (epsilon_s - epsilon_1) / term1_denom +
                          (f / fs) * (epsilon_1 - epsilon_inf) / term2_denom;

    // Specific attenuation coefficient K_l in (dB/km)/(g/m^3)
    // K_l = (0.819 * f) / (epsilon_imag * (1 + eta^2))
    // where eta = (2 + epsilon_real) / epsilon_imag
    double eta = (2.0 + epsilon_real) / epsilon_imag;
    double Kl = (0.819 * f) / (epsilon_imag * (1.0 + eta * eta));

    NS_LOG_DEBUG("Kl: f=" << freqGHz << " GHz, T=" << temperature_K
                          << " K, eps_r=" << epsilon_real
                          << ", eps_i=" << epsilon_imag
                          << ", Kl=" << Kl << " (dB/km)/(g/m^3)");

    return Kl;
}

double
ThzNtnWeatherAttenuation::ComputeFogSlantPath(double elevationDeg) const
{
    double elevRad = elevationDeg * M_PI / 180.0;
    double sinEl = std::sin(elevRad);

    if (sinEl < 0.01)
    {
        sinEl = 0.01; // Avoid division by zero for very low elevations
    }

    return m_fogLayerHeight / sinEl;
}

double
ThzNtnWeatherAttenuation::ComputeFogAttenuation_dB(double freqHz,
                                                    double elevationDeg,
                                                    double lwc_g_m3) const
{
    NS_LOG_FUNCTION(this << freqHz << elevationDeg << lwc_g_m3);

    if (lwc_g_m3 <= 0.0 || elevationDeg <= 0.0)
    {
        return 0.0;
    }

    double freqGHz = freqHz / 1.0e9;

    if (freqGHz < 1.0)
    {
        return 0.0;
    }

    // Compute specific attenuation coefficient K_l from Debye model
    double Kl = ComputeKl(freqGHz, m_temperature);

    // Specific attenuation: gamma_c = K_l * M (dB/km)
    double gammaC = Kl * lwc_g_m3;

    // Slant path through fog/cloud layer
    double Leff = ComputeFogSlantPath(elevationDeg);

    double attenuation = gammaC * Leff;

    NS_LOG_DEBUG("Fog: f=" << freqGHz << " GHz, M=" << lwc_g_m3
                           << " g/m^3, Kl=" << Kl
                           << ", gamma=" << gammaC << " dB/km"
                           << ", Leff=" << Leff << " km"
                           << ", A=" << attenuation << " dB");

    return attenuation;
}

double
ThzNtnWeatherAttenuation::ComputeSnowSlantPath(double elevationDeg) const
{
    double elevRad = elevationDeg * M_PI / 180.0;
    double sinEl = std::sin(elevRad);

    if (sinEl < 0.01)
    {
        sinEl = 0.01;
    }

    return m_snowLayerHeight / sinEl;
}

double
ThzNtnWeatherAttenuation::ComputeSnowAttenuation_dB(double freqHz,
                                                     double elevationDeg,
                                                     double snowRate_mm_h,
                                                     bool isWet) const
{
    NS_LOG_FUNCTION(this << freqHz << elevationDeg << snowRate_mm_h << isWet);

    if (snowRate_mm_h <= 0.0 || elevationDeg <= 0.0)
    {
        return 0.0;
    }

    double freqGHz = freqHz / 1.0e9;

    if (freqGHz < 1.0)
    {
        return 0.0;
    }

    double gammaSnow = 0.0;

    // Reference frequency for normalization
    const double fRef = 100.0; // GHz

    // THZ-10: these coefficients have NO standard behind them.
    //
    // Neither the f^2 * S wet-snow form and its 0.4 dB/km anchor, nor the
    // f^1.6 * S^0.72 dry-snow form and its 0.1 dB/km anchor, is taken from an
    // ITU-R Recommendation or any cited measurement campaign. They were written
    // as an "empirical coefficient" with no source, and they enter the packet
    // path through ThzNtnPropagationLossModel. ITU-R publishes rain (P.838) and
    // gaseous (P.676) specific attenuation; it does not publish an equivalent
    // for snow or dust at these frequencies, and inventing one silently is the
    // fabrication this campaign exists to find.
    //
    // They are kept because a scenario that wants a snow term needs something,
    // and deleting the feature would be its own kind of dishonesty. What
    // changes is that the model now DECLARES them as unsourced estimates rather
    // than presenting them alongside the P.838 rain term as though they carried
    // the same authority. See IsUnsourcedEstimate() and the provenance string.
    m_usedUnsourcedTerm = true;
    if (isWet)
    {
        // Wet snow: gamma ~ f^2 * S. Unsourced; see the note above.
        double coeff = 0.4 / (fRef * fRef);
        gammaSnow = coeff * freqGHz * freqGHz * snowRate_mm_h;
    }
    else
    {
        // Dry snow: gamma ~ f^1.6 * S^0.72. Unsourced; see the note above.
        double coeff = 0.1 / std::pow(fRef, 1.6);
        gammaSnow = coeff * std::pow(freqGHz, 1.6) * std::pow(snowRate_mm_h, 0.72);
    }

    // Slant path through snow layer
    double Leff = ComputeSnowSlantPath(elevationDeg);

    double attenuation = gammaSnow * Leff;

    NS_LOG_DEBUG("Snow (" << (isWet ? "wet" : "dry") << "): f=" << freqGHz
                          << " GHz, S=" << snowRate_mm_h
                          << " mm/h, gamma=" << gammaSnow << " dB/km"
                          << ", Leff=" << Leff << " km"
                          << ", A=" << attenuation << " dB");

    return attenuation;
}

double
ThzNtnWeatherAttenuation::ComputeDustSlantPath(double elevationDeg) const
{
    double elevRad = elevationDeg * M_PI / 180.0;
    double sinEl = std::sin(elevRad);

    if (sinEl < 0.01)
    {
        sinEl = 0.01;
    }

    return m_dustLayerHeight / sinEl;
}

double
ThzNtnWeatherAttenuation::ComputeDustAttenuation_dB(double freqHz,
                                                     double elevationDeg,
                                                     double visibility_km) const
{
    NS_LOG_FUNCTION(this << freqHz << elevationDeg << visibility_km);

    if (visibility_km >= 100.0 || elevationDeg <= 0.0)
    {
        return 0.0; // No significant dust attenuation for good visibility
    }

    if (visibility_km < 0.001)
    {
        visibility_km = 0.001; // Avoid division by zero
    }

    double freqGHz = freqHz / 1.0e9;

    // Sand/dust storm visibility-based model.
    // Reference frequency: 10 GHz.
    // Attenuation exponent beta depends on dust particle size distribution:
    //   beta ~ 0.5--2.0; typical value 1.2 for desert sand storms.
    //
    // gamma_dust = C * (f/f_ref)^beta / V_km   (dB/km)
    //
    // THZ-10: unsourced, exactly as the snow terms above.
    //
    // The 0.5 dB/km anchor at 10 GHz and V = 1 km, and the beta = 1.2
    // frequency exponent described as "typical value for desert sand storms",
    // carry no citation anywhere in this file or in doc/VALIDATION.md. This
    // term reaches the packet path, so it is declared rather than presented
    // beside the P.838 rain term as if it had the same standing.
    m_usedUnsourcedTerm = true;
    // At 10 GHz, V=1 km: gamma ~ 0.5 dB/km (UNSOURCED empirical baseline)
    const double fRef = 10.0;   // GHz reference frequency
    const double beta = 1.2;    // frequency scaling exponent
    const double C = 0.5;       // empirical coefficient (dB/km) at f_ref, V=1km

    double gammaD = C * std::pow(freqGHz / fRef, beta) / visibility_km;

    // Slant path through dust layer
    double Leff = ComputeDustSlantPath(elevationDeg);

    double attenuation = gammaD * Leff;

    NS_LOG_DEBUG("Dust: f=" << freqGHz << " GHz, V=" << visibility_km
                            << " km, gamma=" << gammaD << " dB/km"
                            << ", Leff=" << Leff << " km"
                            << ", A=" << attenuation << " dB");

    return attenuation;
}

std::string
ThzNtnWeatherAttenuation::ProvenanceNote() const
{
    // THZ-10: say which terms carry a standard and which do not.
    std::ostringstream os;
    os << "[thz-weather/provenance] rain=ITU-R P.838 gas=ITU-R P.676";
    if (m_enableSnow || m_enableDust)
    {
        os << " snow/dust=UNSOURCED (empirical coefficients with no ITU-R "
              "Recommendation or cited campaign behind them)";
    }
    else
    {
        os << " snow/dust=disabled";
    }
    if (m_usedUnsourcedTerm)
    {
        os << "  -> an unsourced term CONTRIBUTED to the loss returned by this model";
    }
    return os.str();
}

double
ThzNtnWeatherAttenuation::ComputeTotalWeatherLoss_dB(double freqHz,
                                                      double elevationDeg) const
{
    NS_LOG_FUNCTION(this << freqHz << elevationDeg);

    double totalLoss = 0.0;

    if (m_enableRain && m_rainRate > 0.0)
    {
        totalLoss += ComputeRainAttenuation_dB(freqHz, elevationDeg, m_rainRate);
    }

    if (m_enableFog && m_liquidWaterContent > 0.0)
    {
        totalLoss += ComputeFogAttenuation_dB(freqHz, elevationDeg, m_liquidWaterContent);
    }

    if (m_enableSnow && m_snowRate > 0.0)
    {
        totalLoss += ComputeSnowAttenuation_dB(freqHz, elevationDeg, m_snowRate, m_wetSnow);
    }

    if (m_enableDust && m_dustVisibility < 100.0)
    {
        totalLoss += ComputeDustAttenuation_dB(freqHz, elevationDeg, m_dustVisibility);
    }

    NS_LOG_INFO("Total weather loss at " << freqHz / 1e9 << " GHz, elev="
                                         << elevationDeg << " deg: "
                                         << totalLoss << " dB");

    return totalLoss;
}

// --- Setters ---

void
ThzNtnWeatherAttenuation::SetRainRate(double rainRate_mm_h)
{
    NS_LOG_FUNCTION(this << rainRate_mm_h);
    m_rainRate = rainRate_mm_h;
}

void
ThzNtnWeatherAttenuation::SetLiquidWaterContent(double lwc_g_m3)
{
    NS_LOG_FUNCTION(this << lwc_g_m3);
    m_liquidWaterContent = lwc_g_m3;
}

void
ThzNtnWeatherAttenuation::SetSnowRate(double snowRate_mm_h)
{
    NS_LOG_FUNCTION(this << snowRate_mm_h);
    m_snowRate = snowRate_mm_h;
}

void
ThzNtnWeatherAttenuation::SetWetSnow(bool isWet)
{
    NS_LOG_FUNCTION(this << isWet);
    m_wetSnow = isWet;
}

void
ThzNtnWeatherAttenuation::SetDustVisibility(double visibility_km)
{
    NS_LOG_FUNCTION(this << visibility_km);
    m_dustVisibility = visibility_km;
}

void
ThzNtnWeatherAttenuation::SetEnableRain(bool enable)
{
    NS_LOG_FUNCTION(this << enable);
    m_enableRain = enable;
}

void
ThzNtnWeatherAttenuation::SetEnableFog(bool enable)
{
    NS_LOG_FUNCTION(this << enable);
    m_enableFog = enable;
}

void
ThzNtnWeatherAttenuation::SetEnableSnow(bool enable)
{
    NS_LOG_FUNCTION(this << enable);
    m_enableSnow = enable;
}

void
ThzNtnWeatherAttenuation::SetEnableDust(bool enable)
{
    NS_LOG_FUNCTION(this << enable);
    m_enableDust = enable;
}

void
ThzNtnWeatherAttenuation::SetClimateRegion(ClimateRegion region)
{
    NS_LOG_FUNCTION(this << region);
    m_climateRegion = region;
}

void
ThzNtnWeatherAttenuation::SetStationHeight(double height_km)
{
    NS_LOG_FUNCTION(this << height_km);
    m_stationHeight = height_km;
}

// --- Getters ---

double
ThzNtnWeatherAttenuation::GetRainRate() const
{
    return m_rainRate;
}

double
ThzNtnWeatherAttenuation::GetLiquidWaterContent() const
{
    return m_liquidWaterContent;
}

double
ThzNtnWeatherAttenuation::GetSnowRate() const
{
    return m_snowRate;
}

double
ThzNtnWeatherAttenuation::GetDustVisibility() const
{
    return m_dustVisibility;
}

} // namespace ns3
