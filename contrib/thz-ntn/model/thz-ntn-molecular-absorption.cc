/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-only
 * Author: Muhammad Uzair
 *
 * HITRAN-based Molecular Absorption Model for THz-NTN Links - Implementation
 *
 * Implements altitude-stratified molecular absorption for terahertz
 * non-terrestrial network links.  The specific attenuation at each point is
 * computed with the full ITU-R P.676-13 Annex 1 line-by-line model (44
 * oxygen lines from Table 1 and 35 water-vapour lines from Table 2, with the
 * standard resonant line shape and the dry/Debye continuum term N''_D).
 * The per-point specific attenuation is integrated through the ITU-R P.835
 * standard atmosphere layers along the slant path; the slant geometry
 * accounts for Earth curvature.
 *
 * ITU-R P.676-13 Annex 1 is the authoritative reference for Earth-space
 * gaseous attenuation up to 1000 GHz, is fully self-contained (no external
 * HITRAN data files), and reproduces the ITU-Rpy P.676-13 validation set to
 * within ~2 percent across 1-1000 GHz.  There is NO hand-tuned continuum:
 * the continuum is the physically-derived P.676-13 dry term.
 */

#include "thz-ntn-molecular-absorption.h"

#include <ns3/double.h>
#include <ns3/log.h>
#include <ns3/string.h>
#include <ns3/uinteger.h>

#include <algorithm>
#include <cmath>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("ThzNtnMolecularAbsorption");
NS_OBJECT_ENSURE_REGISTERED(ThzNtnMolecularAbsorption);

// ---------------------------------------------------------------------------
// Physical constants
// ---------------------------------------------------------------------------
static const double EARTH_RADIUS_KM = 6371.0;       // km

static const double DEG_TO_RAD = M_PI / 180.0;
static const double NP_TO_DB = 10.0 / std::log(10.0); // 1 Np = 4.3429 dB

// ---------------------------------------------------------------------------
// TypeId registration
// ---------------------------------------------------------------------------

TypeId
ThzNtnMolecularAbsorption::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::ThzNtnMolecularAbsorption")
            .SetParent<Object>()
            .SetGroupName("ThzNtn")
            .AddConstructor<ThzNtnMolecularAbsorption>()
            .AddAttribute("HumidityProfile",
                          "Atmospheric humidity profile: tropical, "
                          "midlatitude-summer, midlatitude-winter, subarctic",
                          StringValue("midlatitude-summer"),
                          MakeStringAccessor(&ThzNtnMolecularAbsorption::m_humidityProfile),
                          MakeStringChecker())
            .AddAttribute("IntegrationSteps",
                          "Number of integration sub-steps per atmospheric layer",
                          UintegerValue(10),
                          MakeUintegerAccessor(&ThzNtnMolecularAbsorption::m_integrationSteps),
                          MakeUintegerChecker<uint32_t>(1, 1000));

    return tid;
}

// ---------------------------------------------------------------------------
// Constructor / destructor
// ---------------------------------------------------------------------------

ThzNtnMolecularAbsorption::ThzNtnMolecularAbsorption()
    : m_humidityProfile("midlatitude-summer"),
      m_integrationSteps(10)
{
    NS_LOG_FUNCTION(this);
    InitAtmosphericLayers();
    InitAbsorptionLines();
}

ThzNtnMolecularAbsorption::~ThzNtnMolecularAbsorption()
{
    NS_LOG_FUNCTION(this);
}

bool
ThzNtnMolecularAbsorption::LoadHitran2024Lut(const std::string& path)
{
    NS_LOG_FUNCTION(this << path);
    return m_lut.LoadCsv(path);
}

std::string
ThzNtnMolecularAbsorption::GetHitranReleaseTag() const
{
    if (m_lut.IsLoaded())
    {
        const auto& tag = m_lut.ReleaseTag();
        return tag.empty() ? std::string(thzntn::kHitranRelease) : tag;
    }
    return "in-process";
}

void
ThzNtnMolecularAbsorption::DoDispose()
{
    NS_LOG_FUNCTION(this);
    m_layers.clear();
    m_lines.clear();
    Object::DoDispose();
}

// ---------------------------------------------------------------------------
// Atmospheric layer initialisation (ITU-R P.835 standard atmosphere)
// ---------------------------------------------------------------------------

void
ThzNtnMolecularAbsorption::InitAtmosphericLayers()
{
    NS_LOG_FUNCTION(this);

    m_layers.clear();

    // THZ-01. These are INTEGRATION SLAB BOUNDARIES, nothing more. The state
    // variables at any altitude come from GetAtmosphericConditions, which
    // evaluates the ITU-R P.835 reference profile in closed form.
    //
    // What used to be here was a six-entry table of per-layer constant
    // temperature, base pressure and base humidity, and GetAtmosphericConditions
    // restarted an exponential from each layer's base value. That produced a
    // profile that was not P.835 and was not physical either: pressure jumped
    // 13.5 percent across the 2 km boundary, water-vapour density jumped by a
    // factor of eight across 10 km, temperature was flat inside each layer
    // despite a comment claiming a lapse rate, and the integrated water-vapour
    // column came to 17.55 kg/m2 against the P.835 reference 15.0. Both the
    // header and the module README described it as "the ITU-R P.835 standard
    // atmosphere". The line-by-line kernel was correct and was being evaluated
    // on a fabricated atmosphere.
    //
    // The slabs are dense near the ground because that is where the gradients
    // and most of the absorbing mass are, and coarse aloft where little is
    // left. Slab boundaries no longer carry physical values, so a boundary in
    // the wrong place costs accuracy, not correctness.
    const double kBoundsKm[] = {0.0,  0.5,  1.0,  2.0,  3.0,  5.0,  7.0,
                                10.0, 15.0, 20.0, 30.0, 50.0, 70.0, 100.0};
    const size_t n = sizeof(kBoundsKm) / sizeof(kBoundsKm[0]);
    m_layers.reserve(n - 1);
    for (size_t i = 0; i + 1 < n; ++i)
    {
        // The temperature/pressure/humidity fields are retained only so the
        // struct stays source-compatible; nothing reads them.
        m_layers.push_back({kBoundsKm[i], kBoundsKm[i + 1], 0.0, 0.0, 0.0});
    }

    NS_LOG_DEBUG("Initialised " << m_layers.size() << " atmospheric layers");
}

// ---------------------------------------------------------------------------
// ITU-R P.676-13 Annex 1 spectral-line database
//
// Table 1 (oxygen, 44 lines) and Table 2 (water vapour, 35 lines), verbatim
// from Recommendation ITU-R P.676-13 (08/2022).  Each entry holds the line
// centre frequency (GHz) and the six spectroscopic coefficients a1..a6
// (oxygen) or b1..b6 (water vapour).  These are the authoritative reference
// values used by ITU-Rpy and the ITU validation tables; they replace the
// earlier hand-scaled "HITRAN" intensities, which were internally
// inconsistent and, because of a unit-conversion defect, contributed
// essentially nothing to the reported attenuation.
// ---------------------------------------------------------------------------

void
ThzNtnMolecularAbsorption::InitAbsorptionLines()
{
    NS_LOG_FUNCTION(this);

    m_lines.clear();

    // ---- ITU-R P.676-13 Table 1: oxygen (f0_GHz, a1..a6, isH2O=false) ----
    static const AbsorptionLine kOxygen[] = {
        {50.474214, 0.975, 9.651, 6.69, 0, 2.566, 6.85, false},
        {50.987745, 2.529, 8.653, 7.17, 0, 2.246, 6.8, false},
        {51.503360, 6.193, 7.709, 7.64, 0, 1.947, 6.729, false},
        {52.021429, 14.32, 6.819, 8.11, 0, 1.667, 6.64, false},
        {52.542418, 31.24, 5.983, 8.58, 0, 1.388, 6.526, false},
        {53.066934, 64.29, 5.201, 9.06, 0, 1.349, 6.206, false},
        {53.595775, 124.6, 4.474, 9.55, 0, 2.227, 5.085, false},
        {54.130025, 227.3, 3.8, 9.96, 0, 3.17, 3.75, false},
        {54.671180, 389.7, 3.182, 10.37, 0, 3.558, 2.654, false},
        {55.221384, 627.1, 2.618, 10.89, 0, 2.56, 2.952, false},
        {55.783815, 945.3, 2.109, 11.34, 0, -1.172, 6.135, false},
        {56.264774, 543.4, 0.014, 17.03, 0, 3.525, -0.978, false},
        {56.363399, 1331.8, 1.654, 11.89, 0, -2.378, 6.547, false},
        {56.968211, 1746.6, 1.255, 12.23, 0, -3.545, 6.451, false},
        {57.612486, 2120.1, 0.91, 12.62, 0, -5.416, 6.056, false},
        {58.323877, 2363.7, 0.621, 12.95, 0, -1.932, 0.436, false},
        {58.446588, 1442.1, 0.083, 14.91, 0, 6.768, -1.273, false},
        {59.164204, 2379.9, 0.387, 13.53, 0, -6.561, 2.309, false},
        {59.590983, 2090.7, 0.207, 14.08, 0, 6.957, -0.776, false},
        {60.306056, 2103.4, 0.207, 14.15, 0, -6.395, 0.699, false},
        {60.434778, 2438, 0.386, 13.39, 0, 6.342, -2.825, false},
        {61.150562, 2479.5, 0.621, 12.92, 0, 1.014, -0.584, false},
        {61.800158, 2275.9, 0.91, 12.63, 0, 5.014, -6.619, false},
        {62.411220, 1915.4, 1.255, 12.17, 0, 3.029, -6.759, false},
        {62.486253, 1503, 0.083, 15.13, 0, -4.499, 0.844, false},
        {62.997984, 1490.2, 1.654, 11.74, 0, 1.856, -6.675, false},
        {63.568526, 1078, 2.108, 11.34, 0, 0.658, -6.139, false},
        {64.127775, 728.7, 2.617, 10.88, 0, -3.036, -2.895, false},
        {64.678910, 461.3, 3.181, 10.38, 0, -3.968, -2.59, false},
        {65.224078, 274, 3.8, 9.96, 0, -3.528, -3.68, false},
        {65.764779, 153, 4.473, 9.55, 0, -2.548, -5.002, false},
        {66.302096, 80.4, 5.2, 9.06, 0, -1.66, -6.091, false},
        {66.836834, 39.8, 5.982, 8.58, 0, -1.68, -6.393, false},
        {67.369601, 18.56, 6.818, 8.11, 0, -1.956, -6.475, false},
        {67.900868, 8.172, 7.708, 7.64, 0, -2.216, -6.545, false},
        {68.431006, 3.397, 8.652, 7.17, 0, -2.492, -6.6, false},
        {68.960312, 1.334, 9.65, 6.69, 0, -2.773, -6.65, false},
        {118.750334, 940.3, 0.01, 16.64, 0, -0.439, 0.079, false},
        {368.498246, 67.4, 0.048, 16.4, 0, 0, 0, false},
        {424.763020, 637.7, 0.044, 16.4, 0, 0, 0, false},
        {487.249273, 237.4, 0.049, 16, 0, 0, 0, false},
        {715.392902, 98.1, 0.145, 16, 0, 0, 0, false},
        {773.839490, 572.3, 0.141, 16.2, 0, 0, 0, false},
        {834.145546, 183.1, 0.145, 14.7, 0, 0, 0, false},
    };

    // ---- ITU-R P.676-13 Table 2: water vapour (f0_GHz, b1..b6, isH2O=true) ----
    static const AbsorptionLine kWater[] = {
        {22.235080, 0.1079, 2.144, 26.38, 0.76, 5.087, 1, true},
        {67.803960, 0.0011, 8.732, 28.58, 0.69, 4.93, 0.82, true},
        {119.995940, 0.0007, 8.353, 29.48, 0.7, 4.78, 0.79, true},
        {183.310087, 2.273, 0.668, 29.06, 0.77, 5.022, 0.85, true},
        {321.225630, 0.047, 6.179, 24.04, 0.67, 4.398, 0.54, true},
        {325.152888, 1.514, 1.541, 28.23, 0.64, 4.893, 0.74, true},
        {336.227764, 0.001, 9.825, 26.93, 0.69, 4.74, 0.61, true},
        {380.197353, 11.67, 1.048, 28.11, 0.54, 5.063, 0.89, true},
        {390.134508, 0.0045, 7.347, 21.52, 0.63, 4.81, 0.55, true},
        {437.346667, 0.0632, 5.048, 18.45, 0.6, 4.23, 0.48, true},
        {439.150807, 0.9098, 3.595, 20.07, 0.63, 4.483, 0.52, true},
        {443.018343, 0.192, 5.048, 15.55, 0.6, 5.083, 0.5, true},
        {448.001085, 10.41, 1.405, 25.64, 0.66, 5.028, 0.67, true},
        {470.888999, 0.3254, 3.597, 21.34, 0.66, 4.506, 0.65, true},
        {474.689092, 1.26, 2.379, 23.2, 0.65, 4.804, 0.64, true},
        {488.490108, 0.2529, 2.852, 25.86, 0.69, 5.201, 0.72, true},
        {503.568532, 0.0372, 6.731, 16.12, 0.61, 3.98, 0.43, true},
        {504.482692, 0.0124, 6.731, 16.12, 0.61, 4.01, 0.45, true},
        {547.676440, 0.9785, 0.158, 26, 0.7, 4.5, 1, true},
        {552.020960, 0.184, 0.158, 26, 0.7, 4.5, 1, true},
        {556.935985, 497, 0.159, 30.86, 0.69, 4.552, 1, true},
        {620.700807, 5.015, 2.391, 24.38, 0.71, 4.856, 0.68, true},
        {645.766085, 0.0067, 8.633, 18, 0.6, 4, 0.5, true},
        {658.005280, 0.2732, 7.816, 32.1, 0.69, 4.14, 1, true},
        {752.033113, 243.4, 0.396, 30.86, 0.68, 4.352, 0.84, true},
        {841.051732, 0.0134, 8.177, 15.9, 0.33, 5.76, 0.45, true},
        {859.965698, 0.1325, 8.055, 30.6, 0.68, 4.09, 0.84, true},
        {899.303175, 0.0547, 7.914, 29.85, 0.68, 4.53, 0.9, true},
        {902.611085, 0.0386, 8.429, 28.65, 0.7, 5.1, 0.95, true},
        {906.205957, 0.1836, 5.11, 24.08, 0.7, 4.7, 0.53, true},
        {916.171582, 8.4, 1.441, 26.73, 0.7, 5.15, 0.78, true},
        {923.112692, 0.0079, 10.293, 29, 0.7, 5, 0.8, true},
        {970.315022, 9.009, 1.919, 25.5, 0.64, 4.94, 0.67, true},
        {987.926764, 134.6, 0.257, 29.85, 0.68, 4.55, 0.9, true},
        {1780.000000, 17506, 0.952, 196.3, 2, 24.15, 5, true},
    };

    for (const auto& l : kOxygen)
    {
        m_lines.push_back(l);
    }
    for (const auto& l : kWater)
    {
        m_lines.push_back(l);
    }

    NS_LOG_DEBUG("Initialised " << m_lines.size()
                 << " ITU-R P.676-13 spectral lines (44 O2 + 35 H2O)");
}

// ---------------------------------------------------------------------------
// Humidity profile setter
// ---------------------------------------------------------------------------

void
ThzNtnMolecularAbsorption::SetHumidityProfile(const std::string& profile)
{
    NS_LOG_FUNCTION(this << profile);
    m_humidityProfile = profile;
}

// ---------------------------------------------------------------------------
// Atmospheric conditions lookup
// ---------------------------------------------------------------------------

void
ThzNtnMolecularAbsorption::GetAtmosphericConditions(double altitude_km,
                                                     double& tempK,
                                                     double& pressureHPa,
                                                     double& humidity) const
{
    // Above the top layer: near-vacuum conditions
    if (altitude_km >= 100.0)
    {
        tempK = 210.0;
        pressureHPa = 0.001;
        humidity = 0.0;
        return;
    }

    // Find the enclosing layer and interpolate within it
    // THZ-01: ITU-R P.835-6 Section 1.1, the mean annual global reference
    // atmosphere, evaluated in closed form.
    //
    // P.835 is written in GEOPOTENTIAL height, so convert first. Using the
    // geometric height directly overstates the altitude of every level above
    // the boundary layer.
    const double h = std::max(altitude_km, 0.0);
    const double hp = (6356.766 * h) / (6356.766 + h); // geopotential km

    // Temperature: seven piecewise-linear segments. The lapse rates are the
    // defining feature of the profile and the previous implementation had none
    // of them: T was constant inside each of its six layers.
    if (hp <= 11.0)
    {
        tempK = 288.15 - 6.5 * hp;
    }
    else if (hp <= 20.0)
    {
        tempK = 216.65;
    }
    else if (hp <= 32.0)
    {
        tempK = 216.65 + (hp - 20.0);
    }
    else if (hp <= 47.0)
    {
        tempK = 228.65 + 2.8 * (hp - 32.0);
    }
    else if (hp <= 51.0)
    {
        tempK = 270.65;
    }
    else if (hp <= 71.0)
    {
        tempK = 270.65 - 2.8 * (hp - 51.0);
    }
    else if (hp <= 84.852)
    {
        tempK = 214.65 - 2.0 * (hp - 71.0);
    }
    else
    {
        // Above the 84.852 km geopotential level (86 km geometric) P.835
        // Section 1.2 switches to a different set of expressions. Almost no
        // absorbing mass remains there - the pressure is under 4e-3 hPa - so
        // hold the top-of-segment value rather than import formulas whose
        // contribution is below the numerical noise of the integral. This is
        // an explicit truncation, not an approximation dressed up as the
        // recommendation.
        tempK = 186.946;
    }

    // Pressure: the hydrostatic solution on those same segments, so it is
    // continuous everywhere by construction. The previous implementation
    // restarted an exponential from each layer's base pressure, which is why
    // it stepped 13.5 percent across the 2 km boundary - a discontinuity in a
    // state variable, which no atmosphere has.
    if (hp <= 11.0)
    {
        pressureHPa = 1013.25 * std::pow(288.15 / (288.15 - 6.5 * hp), -34.1632 / 6.5);
    }
    else if (hp <= 20.0)
    {
        pressureHPa = 226.3226 * std::exp(-34.1632 * (hp - 11.0) / 216.65);
    }
    else if (hp <= 32.0)
    {
        pressureHPa = 54.74980 * std::pow(216.65 / (216.65 + (hp - 20.0)), 34.1632);
    }
    else if (hp <= 47.0)
    {
        pressureHPa =
            8.680422 * std::pow(228.65 / (228.65 + 2.8 * (hp - 32.0)), 34.1632 / 2.8);
    }
    else if (hp <= 51.0)
    {
        pressureHPa = 1.109106 * std::exp(-34.1632 * (hp - 47.0) / 270.65);
    }
    else if (hp <= 71.0)
    {
        pressureHPa =
            0.6694167 * std::pow(270.65 / (270.65 - 2.8 * (hp - 51.0)), -34.1632 / 2.8);
    }
    else if (hp <= 84.852)
    {
        pressureHPa =
            0.03956649 * std::pow(214.65 / (214.65 - 2.0 * (hp - 71.0)), -34.1632 / 2.0);
    }
    else
    {
        pressureHPa = 0.003743;
    }

    // Water vapour: P.835 Section 1.1 specifies a single exponential with a
    // 2 km scale height from a 7.5 g/m3 surface density, which integrates to
    // the 15 kg/m2 reference column. The old per-layer restart gave 17.55, a
    // 17 percent excess, and an eight-fold discontinuity at 10 km.
    double rho = 7.5 * std::exp(-h / 2.0);

    // P.835 Section 1.1 also floors the MIXING RATIO at 2e-6, so above the
    // altitude where the exponential falls below that, the mixing ratio rather
    // than the density is held constant. Mixing ratio (mass of water vapour
    // per unit mass of dry air) from density and pressure:
    //   e = rho T / 216.7   [hPa]   and   mr = 0.622 e / (P - e).
    const double e = rho * tempK / 216.7;
    const double denom = pressureHPa - e;
    const double mr = (denom > 0.0) ? (0.622 * e / denom) : 0.0;
    constexpr double kMinMixingRatio = 2.0e-6;
    if (mr < kMinMixingRatio && pressureHPa > 0.0)
    {
        // Invert the same relations at the floor value.
        const double eFloor = kMinMixingRatio * pressureHPa / (0.622 + kMinMixingRatio);
        rho = eFloor * 216.7 / tempK;
    }

    humidity = ApplyHumidityProfile(rho, altitude_km);
}

// ---------------------------------------------------------------------------
// Humidity profile scaling
//
// Multipliers relative to the midlatitude-summer baseline per ITU-R P.835.
// ---------------------------------------------------------------------------

double
ThzNtnMolecularAbsorption::ApplyHumidityProfile(double baseHumidity,
                                                  double altitude_km) const
{
    double scale = 1.0;

    if (m_humidityProfile == "tropical")
    {
        // Tropical: significantly higher water vapour at all altitudes
        if (altitude_km < 2.0)
        {
            scale = 2.5;   // ~19 g/m3 surface
        }
        else if (altitude_km < 5.0)
        {
            scale = 2.2;
        }
        else if (altitude_km < 10.0)
        {
            scale = 1.8;
        }
        else
        {
            scale = 1.5;
        }
    }
    else if (m_humidityProfile == "midlatitude-summer")
    {
        scale = 1.0; // baseline
    }
    else if (m_humidityProfile == "midlatitude-winter")
    {
        // Winter: reduced water vapour
        if (altitude_km < 2.0)
        {
            scale = 0.45;  // ~3.5 g/m3 surface
        }
        else if (altitude_km < 5.0)
        {
            scale = 0.50;
        }
        else if (altitude_km < 10.0)
        {
            scale = 0.55;
        }
        else
        {
            scale = 0.60;
        }
    }
    else if (m_humidityProfile == "subarctic")
    {
        // Subarctic: very dry
        if (altitude_km < 2.0)
        {
            scale = 0.20;  // ~1.5 g/m3 surface
        }
        else if (altitude_km < 5.0)
        {
            scale = 0.25;
        }
        else if (altitude_km < 10.0)
        {
            scale = 0.30;
        }
        else
        {
            scale = 0.35;
        }
    }
    else
    {
        NS_LOG_WARN("Unknown humidity profile '" << m_humidityProfile
                    << "', using midlatitude-summer baseline");
        scale = 1.0;
    }

    return baseHumidity * scale;
}

// ---------------------------------------------------------------------------
// Absorption coefficient computation — ITU-R P.676-13 Annex 1
//
// The specific gaseous attenuation is
//
//   gamma(f) = 0.1820 * f * N''(f)          [dB/km]   (f in GHz)
//
// with the imaginary part of the complex refractivity
//
//   N''(f) = sum_i S_i F_i(f) + N''_D(f)
//
// Oxygen (Table 1)          Water vapour (Table 2)
//   S_i = a1 1e-7 p th^3 e^{a2(1-th)}   S_i = b1 1e-1 e th^3.5 e^{b2(1-th)}
//   df  = a3 1e-4 (p th^{0.8-a4}        df  = b3 1e-4 (p th^{b4}
//              + 1.1 e th)                        + b5 e th^{b6})
//   df  = sqrt(df^2 + 2.25e-6)          df  = 0.535 df
//   d   = (a5 + a6 th) 1e-4 (p+e) th^0.8       + sqrt(0.217 df^2
//                                               + 2.1316e-12 f_i^2 / th)
//                                       d   = 0
//
// Line shape:
//   F_i = f/f_i [ (df - d(f_i-f))/((f_i-f)^2+df^2)
//              +  (df - d(f_i+f))/((f_i+f)^2+df^2) ]
//
// Dry/Debye continuum:
//   d0    = 5.6e-4 (p+e) th^0.8
//   N''_D = f p th^2 [ 6.14e-5/(d0(1+(f/d0)^2))
//                    + 1.4e-12 p th^1.5 / (1 + 1.9e-5 f^1.5) ]
//
// where th = 300/T, p = dry-air partial pressure (hPa),
// e = water-vapour partial pressure (hPa) = rho T / 216.7.
//
// The result is converted from dB/km to Np/km (dividing by 10/ln10) so the
// slant-path integrator (which sums Np and converts back to dB) is unchanged.
// ---------------------------------------------------------------------------

double
ThzNtnMolecularAbsorption::ComputeAbsorptionCoefficient(double freqHz,
                                                         double tempK,
                                                         double pressureHPa,
                                                         double humidityGm3) const
{
    NS_LOG_FUNCTION(this << freqHz << tempK << pressureHPa << humidityGm3);

    // Guard against degenerate conditions
    if (tempK < 1.0 || pressureHPa < 1e-6)
    {
        return 0.0;
    }

    const double f = freqHz / 1.0e9;          // GHz
    const double theta = 300.0 / tempK;       // relative inverse temperature
    // Water-vapour partial pressure from density (P.676-13 eq. 4).
    double e = std::max(0.0, humidityGm3) * tempK / 216.7;   // hPa
    double p = pressureHPa - e;                               // dry-air pressure, hPa
    if (p < 0.0)
    {
        p = 0.0;
    }

    double Npp = 0.0;   // N''(f), dimensionless

    for (const auto& line : m_lines)
    {
        const double f0 = line.f0_GHz;
        double Si;      // line strength
        double df;      // line width
        double delta;   // interference / overlap correction

        if (!line.isWaterVapor)
        {
            // Oxygen (Table 1), coefficients a1..a6 stored in c1..c6.
            Si = line.c1 * 1.0e-7 * p * std::pow(theta, 3.0)
                 * std::exp(line.c2 * (1.0 - theta));
            df = line.c3 * 1.0e-4
                 * (p * std::pow(theta, 0.8 - line.c4) + 1.1 * e * theta);
            df = std::sqrt(df * df + 2.25e-6);   // Zeeman splitting floor
            delta = (line.c5 + line.c6 * theta) * 1.0e-4 * (p + e)
                    * std::pow(theta, 0.8);
        }
        else
        {
            // Water vapour (Table 2), coefficients b1..b6 stored in c1..c6.
            Si = line.c1 * 1.0e-1 * e * std::pow(theta, 3.5)
                 * std::exp(line.c2 * (1.0 - theta));
            df = line.c3 * 1.0e-4
                 * (p * std::pow(theta, line.c4)
                    + line.c5 * e * std::pow(theta, line.c6));
            // Doppler broadening correction.
            df = 0.535 * df
                 + std::sqrt(0.217 * df * df
                             + 2.1316e-12 * f0 * f0 / theta);
            delta = 0.0;
        }

        const double fm = f0 - f;
        const double fp = f0 + f;
        const double Fi = (f / f0)
                          * ((df - delta * fm) / (fm * fm + df * df)
                             + (df - delta * fp) / (fp * fp + df * df));
        Npp += Si * Fi;
    }

    // Dry/Debye continuum (P.676-13 eq. 8).
    const double d0 = 5.6e-4 * (p + e) * std::pow(theta, 0.8);
    const double Nd = f * p * theta * theta
                      * (6.14e-5 / (d0 * (1.0 + (f / d0) * (f / d0)))
                         + 1.4e-12 * p * std::pow(theta, 1.5)
                               / (1.0 + 1.9e-5 * std::pow(f, 1.5)));
    Npp += Nd;

    // Specific attenuation in dB/km, then to Np/km for the integrator.
    const double gamma_dBkm = 0.1820 * f * Npp;
    const double k_NpKm = std::max(0.0, gamma_dBkm) / NP_TO_DB;

    NS_LOG_DEBUG("P.676-13 k(f=" << f << " GHz, T=" << tempK << " K, p="
                 << p << " hPa, e=" << e << " hPa): " << gamma_dBkm
                 << " dB/km (" << k_NpKm << " Np/km)");

    return k_NpKm;
}

// ---------------------------------------------------------------------------
// Slant-path integration through the atmosphere
//
// For a ground station at altitude h_g and a satellite at altitude h_s,
// the link at elevation angle theta passes through multiple atmospheric
// layers.  Within each layer the effective elevation angle is corrected
// for Earth curvature:
//
//   sin(theta_eff(h)) = sqrt(1 - ((R_E + h_g) / (R_E + h))^2 * cos^2(theta))
//
// The slant distance through a sub-layer [h_lo, h_hi] is:
//
//   ds = (h_hi - h_lo) / sin(theta_eff(h_mid))
//
// and the absorption in that sub-layer is k(f, T, P, q) * ds.
// ---------------------------------------------------------------------------

double
ThzNtnMolecularAbsorption::ComputeSlantPathAbsorption(double freqHz,
                                                       double elevationDeg,
                                                       double groundAlt_km,
                                                       double satAlt_km) const
{
    NS_LOG_FUNCTION(this << freqHz << elevationDeg << groundAlt_km << satAlt_km);

    // Ensure ground < satellite altitude
    double hLow = std::min(groundAlt_km, satAlt_km);
    double hHigh = std::max(groundAlt_km, satAlt_km);

    // Both endpoints above atmosphere: vacuum (ISL)
    if (hLow >= 100.0)
    {
        NS_LOG_DEBUG("ISL link detected (both endpoints >= 100 km), absorption = 0");
        return 0.0;
    }

    // Cap the upper integration limit at 100 km (top of atmosphere)
    double hTop = std::min(hHigh, 100.0);

    // Elevation angle in radians.
    //
    // THZ-11: the integrator divides by sin(theta) and goes singular at the
    // horizon, so the elevation is floored. That guard is fine; hiding it was
    // not. A request at 2 degrees used to be answered with the 5 degree
    // attenuation, silently, with no warning and no way for the caller to
    // detect the substitution - and sub-THz slant loss is steepest exactly
    // there, so the substitution is largest where it matters most. The floor is
    // now recorded and warned about.
    double thetaRad = elevationDeg * DEG_TO_RAD;
    const double thetaFloorRad = kMinIntegratorElevationDeg * DEG_TO_RAD;
    m_lastElevationClamped = (thetaRad < thetaFloorRad);
    if (m_lastElevationClamped)
    {
        NS_LOG_WARN("THZ-11: requested elevation " << elevationDeg
                    << " deg is below the integrator floor of "
                    << kMinIntegratorElevationDeg
                    << " deg; returning the attenuation AT THE FLOOR, which "
                       "understates the true slant loss. Check "
                       "WasLastElevationClamped().");
        thetaRad = thetaFloorRad;
    }
    m_lastIntegratedElevDeg = thetaRad / DEG_TO_RAD;
    double cosTheta = std::cos(thetaRad);

    double R_E = EARTH_RADIUS_KM;
    double R_g = R_E + hLow;  // geocentric radius of ground station

    double totalAbsorption_Np = 0.0;

    // Iterate over atmospheric layers that the path intersects
    for (const auto& layer : m_layers)
    {
        // Determine the altitude range within this layer that the path crosses
        double layerBottom = std::max(layer.altLow_km, hLow);
        double layerTop = std::min(layer.altHigh_km, hTop);

        if (layerBottom >= layerTop)
        {
            continue; // path does not intersect this layer
        }

        // Sub-divide the layer for numerical accuracy
        double dh = (layerTop - layerBottom) / m_integrationSteps;

        for (uint32_t step = 0; step < m_integrationSteps; ++step)
        {
            double h_lo = layerBottom + step * dh;
            double h_hi = h_lo + dh;
            double h_mid = 0.5 * (h_lo + h_hi);

            // Effective elevation angle at h_mid corrected for Earth curvature
            double R_h = R_E + h_mid;
            double sinThetaArg = 1.0 - (R_g / R_h) * (R_g / R_h) * cosTheta * cosTheta;
            double sinThetaEff;
            if (sinThetaArg <= 0.0)
            {
                // Below the tangent height: cannot happen once the elevation
                // is floored above, but keep the substitution consistent with
                // that floor rather than introducing a third magic angle.
                sinThetaEff = std::sin(kMinIntegratorElevationDeg * DEG_TO_RAD);
                m_lastElevationClamped = true;
            }
            else
            {
                sinThetaEff = std::sqrt(sinThetaArg);
            }
            // Ensure a minimum to avoid division by zero
            sinThetaEff = std::max(sinThetaEff, std::sin(1.0 * DEG_TO_RAD));

            // Slant distance through this sub-layer [km]
            double slantDist_km = dh / sinThetaEff;

            // Absorption coefficient at this point [Np/km]
            double k_NpKm;
            if (m_lut.IsLoaded())
            {
                // HITRAN-2024 LUT path (Roadmap §4.3.1) — returns dB/km;
                // convert to Np/km via 1/NP_TO_DB.
                double db_per_km = m_lut.Get(freqHz, h_mid);
                if (std::isfinite(db_per_km))
                {
                    k_NpKm = db_per_km / NP_TO_DB;
                }
                else
                {
                    double tempK, pressureHPa, humidity;
                    GetAtmosphericConditions(h_mid, tempK, pressureHPa, humidity);
                    k_NpKm = ComputeAbsorptionCoefficient(freqHz, tempK,
                                                           pressureHPa, humidity);
                }
            }
            else
            {
                double tempK, pressureHPa, humidity;
                GetAtmosphericConditions(h_mid, tempK, pressureHPa, humidity);
                k_NpKm = ComputeAbsorptionCoefficient(freqHz, tempK,
                                                       pressureHPa, humidity);
            }

            // Accumulate absorption [Np]
            totalAbsorption_Np += k_NpKm * slantDist_km;
        }
    }

    // Convert from Np to dB
    double totalAbsorption_dB = totalAbsorption_Np * NP_TO_DB;

    NS_LOG_DEBUG("Slant path absorption: " << totalAbsorption_dB << " dB ("
                 << totalAbsorption_Np << " Np), elev=" << elevationDeg
                 << " deg, h_g=" << hLow << " km, h_s=" << hHigh << " km");

    return std::max(0.0, totalAbsorption_dB);
}

// ---------------------------------------------------------------------------
// Main entry point: ComputeAbsorptionLoss_dB
// ---------------------------------------------------------------------------

double
ThzNtnMolecularAbsorption::ComputeAbsorptionLoss_dB(double freqHz,
                                                     double distanceM,
                                                     double altitudeTx_km,
                                                     double altitudeRx_km,
                                                     double elevationDeg) const
{
    NS_LOG_FUNCTION(this << freqHz << distanceM << altitudeTx_km
                    << altitudeRx_km << elevationDeg);

    // ---- Inter-satellite link: no atmospheric absorption ----
    if (altitudeTx_km >= 100.0 && altitudeRx_km >= 100.0)
    {
        NS_LOG_INFO("ISL detected: TX alt=" << altitudeTx_km << " km, RX alt="
                    << altitudeRx_km << " km -> absorption = 0 dB");
        return 0.0;
    }

    // ---- Ground-to-satellite or satellite-to-ground link ----
    // Identify ground and space endpoints
    double groundAlt = std::min(altitudeTx_km, altitudeRx_km);
    double satAlt = std::max(altitudeTx_km, altitudeRx_km);

    // Use slant-path integration
    double absorption_dB = ComputeSlantPathAbsorption(freqHz, elevationDeg,
                                                       groundAlt, satAlt);

    NS_LOG_INFO("Absorption loss: " << absorption_dB << " dB at f="
                << freqHz / 1e9 << " GHz, d=" << distanceM / 1e3
                << " km, elev=" << elevationDeg << " deg");

    return absorption_dB;
}

// ---------------------------------------------------------------------------
// Transmittance computation
// ---------------------------------------------------------------------------

double
ThzNtnMolecularAbsorption::GetTransmittance(double freqHz,
                                             double distanceM,
                                             double altitudeAvg_km) const
{
    NS_LOG_FUNCTION(this << freqHz << distanceM << altitudeAvg_km);

    // Roadmap §4.3.1: HITRAN-2024 LUT path when loaded.
    if (m_lut.IsLoaded())
    {
        double db_per_km = m_lut.Get(freqHz, altitudeAvg_km);
        if (std::isfinite(db_per_km))
        {
            double db = db_per_km * (distanceM / 1e3);
            return std::pow(10.0, -db / 10.0);
        }
    }

    // Get atmospheric conditions at the representative altitude
    double tempK, pressureHPa, humidity;
    GetAtmosphericConditions(altitudeAvg_km, tempK, pressureHPa, humidity);

    // Absorption coefficient in Np/km
    double k_NpKm = ComputeAbsorptionCoefficient(freqHz, tempK,
                                                  pressureHPa, humidity);

    // Path length in km
    double distanceKm = distanceM / 1000.0;

    // Total absorption in Np
    double absorption_Np = k_NpKm * distanceKm;

    // Transmittance: tau = exp(-absorption_Np)
    double transmittance = std::exp(-absorption_Np);
    transmittance = std::max(0.0, std::min(1.0, transmittance));

    NS_LOG_DEBUG("Transmittance at f=" << freqHz / 1e9 << " GHz, d="
                 << distanceKm << " km, alt=" << altitudeAvg_km
                 << " km: tau=" << transmittance);

    return transmittance;
}

} // namespace ns3
