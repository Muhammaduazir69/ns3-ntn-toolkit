/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2024 Muhammad Uzair
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Author: Muhammad Uzair <uk5595985@gmail.com>
 */

#include "thz-ntn-scintillation.h"

#include <ns3/boolean.h>
#include <ns3/double.h>
#include <ns3/enum.h>
#include <ns3/log.h>
#include <ns3/simulator.h>

#include <algorithm>
#include <cmath>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("ThzNtnScintillation");

NS_OBJECT_ENSURE_REGISTERED(ThzNtnScintillation);

TypeId
ThzNtnScintillation::GetTypeId(void)
{
    static TypeId tid =
        TypeId("ns3::ThzNtnScintillation")
            .SetParent<Object>()
            .SetGroupName("ThzNtn")
            .AddConstructor<ThzNtnScintillation>()
            .AddAttribute("TurbulenceStrength",
                          "Turbulence strength level (Weak, Moderate, Strong). "
                          "Controls the ground-level Cn2 value.",
                          EnumValue(ThzNtnScintillation::MODERATE),
                          MakeEnumAccessor<TurbulenceLevel>(
                              &ThzNtnScintillation::m_turbulenceLevel),
                          MakeEnumChecker(ThzNtnScintillation::WEAK,
                                          "Weak",
                                          ThzNtnScintillation::MODERATE,
                                          "Moderate",
                                          ThzNtnScintillation::STRONG,
                                          "Strong"))
            .AddAttribute("WindSpeed",
                          "Transverse wind speed in m/s for scintillation temporal correlation.",
                          DoubleValue(10.0),
                          MakeDoubleAccessor(&ThzNtnScintillation::m_windSpeed),
                          MakeDoubleChecker<double>(0.1, 100.0))
            .AddAttribute("GroundTemperature",
                          "Ground-level temperature in Kelvin.",
                          DoubleValue(293.0),
                          MakeDoubleAccessor(&ThzNtnScintillation::m_groundTemperature),
                          MakeDoubleChecker<double>(200.0, 330.0))
            .AddAttribute("AntennaDiameter",
                          "Antenna diameter in meters for aperture averaging.",
                          DoubleValue(0.3),
                          MakeDoubleAccessor(&ThzNtnScintillation::m_antennaDiameter),
                          MakeDoubleChecker<double>(0.001, 100.0))
            .AddAttribute("RelativeHumidity",
                          "Relative humidity at ground level in percent.",
                          DoubleValue(50.0),
                          MakeDoubleAccessor(&ThzNtnScintillation::m_relativeHumidity),
                          MakeDoubleChecker<double>(0.0, 100.0))
            .AddAttribute("SamplingPeriod",
                          "Sampling period for the time-series generator.",
                          TimeValue(MilliSeconds(100)),
                          MakeTimeAccessor(&ThzNtnScintillation::m_samplingPeriod),
                          MakeTimeChecker(MicroSeconds(1)))
            .AddAttribute("OuterScale",
                          "Outer scale of turbulence L_0 in meters.",
                          DoubleValue(100.0),
                          MakeDoubleAccessor(&ThzNtnScintillation::m_outerScale),
                          MakeDoubleChecker<double>(1.0, 10000.0))
            .AddAttribute("TurbulentLayerHeight",
                          "Effective turbulent layer height in meters.",
                          DoubleValue(1000.0),
                          MakeDoubleAccessor(&ThzNtnScintillation::m_turbulentLayerHeight),
                          MakeDoubleChecker<double>(100.0, 20000.0));
    return tid;
}

ThzNtnScintillation::ThzNtnScintillation()
    : m_turbulenceLevel(MODERATE),
      m_windSpeed(10.0),
      m_groundTemperature(293.0),
      m_antennaDiameter(0.3),
      m_relativeHumidity(50.0),
      m_samplingPeriod(MilliSeconds(100)),
      m_outerScale(100.0),
      m_turbulentLayerHeight(1000.0),
      m_prevSample(0.0),
      m_initialized(false)
{
    NS_LOG_FUNCTION(this);

    m_normalRng = CreateObject<NormalRandomVariable>();
    m_normalRng->SetAttribute("Mean", DoubleValue(0.0));
    m_normalRng->SetAttribute("Variance", DoubleValue(1.0));
}

ThzNtnScintillation::~ThzNtnScintillation()
{
    NS_LOG_FUNCTION(this);
}

double
ThzNtnScintillation::ComputeNwet() const
{
    NS_LOG_FUNCTION(this);

    // Wet term of refractivity N_wet (ITU-R P.453).
    //
    // Saturation water vapor pressure (Magnus formula):
    //   e_s = 6.1121 * exp((17.502 * t) / (240.97 + t))    [hPa]
    // where t is temperature in Celsius.
    //
    // Water vapor partial pressure:
    //   e = (H/100) * e_s    [hPa]
    //
    // Wet refractivity:
    //   N_wet = (72 * e / T) + (3.75e5 * e / T^2)

    double T = m_groundTemperature;          // Kelvin
    double t = T - 273.15;                    // Celsius
    double H = m_relativeHumidity;            // %

    double es = 6.1121 * std::exp((17.502 * t) / (240.97 + t)); // hPa
    double e = (H / 100.0) * es;                                // hPa

    double Nwet = (72.0 * e / T) + (3.75e5 * e / (T * T));

    NS_LOG_DEBUG("Nwet: T=" << T << " K, t=" << t << " C, H=" << H
                            << " %, e_s=" << es << " hPa, e=" << e
                            << " hPa, Nwet=" << Nwet << " ppm");

    return Nwet;
}

double
ThzNtnScintillation::ComputeSigmaRef() const
{
    NS_LOG_FUNCTION(this);

    // ITU-R P.618 Section 2.4.1:
    // Reference standard deviation of signal amplitude at 4 GHz:
    //   sigma_ref = 3.6e-3 + 1.0e-4 * N_wet    [dB]

    double Nwet = ComputeNwet();
    double sigmaRef = 3.6e-3 + 1.0e-4 * Nwet;

    NS_LOG_DEBUG("sigma_ref = " << sigmaRef << " dB (Nwet=" << Nwet << ")");

    return sigmaRef;
}

double
ThzNtnScintillation::ComputeApertureAveragingFactor(double freqHz,
                                                     double elevationDeg) const
{
    NS_LOG_FUNCTION(this << freqHz << elevationDeg);

    // Antenna aperture averaging factor g(D) from ITU-R P.618 Section 2.4.1.
    //
    // The effective antenna diameter for scintillation:
    //   D_eff = sqrt(eta) * D
    // where eta is the antenna efficiency (assumed 0.5).
    //
    // The averaging factor:
    //   g(x) = sqrt(3.86 * (x^2 + 1)^(11/12)
    //           * sin((11/6) * arctan(1/x)) - 7.08 * x^(5/6))
    //
    // where x = 1.22 * D_eff^2 * f / (c * L) with L the effective path length.
    //
    // Simplified form from ITU-R P.618:
    //   x = pi * D_eff * f / (c * sqrt(h_L * lambda / sin(theta)))
    //   g(x) = [3.86 * (x^2+1)^(11/12) * sin(11/6 * atan(1/x)) - 7.08*x^(5/6)]^0.5

    if (m_antennaDiameter <= 0.001)
    {
        return 1.0; // Point receiver, no averaging
    }

    double c = 3.0e8;                  // speed of light (m/s)
    double lambda = c / freqHz;        // wavelength (m)
    double Deff = std::sqrt(0.5) * m_antennaDiameter; // effective diameter (eta=0.5)
    double elevRad = elevationDeg * M_PI / 180.0;
    double sinEl = std::sin(elevRad);

    if (sinEl < 0.01)
    {
        sinEl = 0.01;
    }

    // Effective turbulent layer height for Fresnel zone calculation
    double hL = m_turbulentLayerHeight; // m

    // Fresnel zone size
    double fresnelSize = std::sqrt(lambda * hL / sinEl);

    if (fresnelSize < 1.0e-6)
    {
        return 1.0;
    }

    // Argument x = pi * Deff / fresnelSize
    // (ratio of antenna diameter to Fresnel zone)
    double x = M_PI * Deff / fresnelSize;

    if (x < 1.0e-4)
    {
        return 1.0; // Very small antenna compared to Fresnel zone
    }

    // ITU-R P.618 aperture averaging factor
    double x2 = x * x;
    double term1 = 3.86 * std::pow(x2 + 1.0, 11.0 / 12.0) *
                   std::sin((11.0 / 6.0) * std::atan(1.0 / x));
    double term2 = 7.08 * std::pow(x, 5.0 / 6.0);

    double gSquared = term1 - term2;

    double g = 1.0;
    if (gSquared > 0.0)
    {
        g = std::sqrt(gSquared);
    }
    else
    {
        // For very large x (large antenna), g approaches an asymptotic value
        g = std::pow(x, -7.0 / 6.0);
    }

    // Clamp to valid range
    if (g > 1.0)
    {
        g = 1.0;
    }
    if (g < 1.0e-6)
    {
        g = 1.0e-6;
    }

    NS_LOG_DEBUG("Aperture avg: D=" << m_antennaDiameter << " m, Deff=" << Deff
                                    << " m, Fresnel=" << fresnelSize
                                    << " m, x=" << x << ", g=" << g);

    return g;
}

double
ThzNtnScintillation::GetCn2Ground() const
{
    switch (m_turbulenceLevel)
    {
    case WEAK:
        return 1.0e-17;    // m^{-2/3}
    case MODERATE:
        return 1.0e-15;    // m^{-2/3}
    case STRONG:
        return 1.0e-13;    // m^{-2/3}
    default:
        return 1.0e-15;
    }
}

double
ThzNtnScintillation::ComputeIntegratedCn2(double elevationDeg) const
{
    NS_LOG_FUNCTION(this << elevationDeg);

    // Hufnagel-Valley (H-V 5/7) model for Cn2 altitude profile:
    //
    //   Cn2(h) = A * exp(-h/100)
    //            + 5.94e-53 * (v_rms/27)^2 * h^10 * exp(-h/1000)
    //            + 2.7e-16 * exp(-h/1500)
    //
    // where:
    //   A = ground-level Cn2 parameter
    //   v_rms = RMS wind speed in the upper troposphere (typically 21 m/s)
    //   h = altitude in meters
    //
    // The integrated Cn2 along the slant path:
    //   integral = (1/sin(theta)) * integral_0^H Cn2(h) dh

    double A = GetCn2Ground();
    double v_rms = 21.0; // m/s, RMS upper-atmosphere wind speed (H-V standard)
    double elevRad = elevationDeg * M_PI / 180.0;
    double sinEl = std::sin(elevRad);

    if (sinEl < 0.01)
    {
        sinEl = 0.01;
    }

    // Numerical integration using trapezoidal rule up to 20 km
    const int nSteps = 2000;
    const double hMax = 20000.0; // m (20 km)
    const double dh = hMax / nSteps;

    double integral = 0.0;

    for (int i = 0; i < nSteps; ++i)
    {
        double h1 = i * dh;
        double h2 = (i + 1) * dh;
        double hMid1 = h1;
        double hMid2 = h2;

        // Cn2 at h1
        double cn2_1 = A * std::exp(-hMid1 / 100.0) +
                       5.94e-53 * std::pow(v_rms / 27.0, 2.0) *
                           std::pow(hMid1, 10.0) * std::exp(-hMid1 / 1000.0) +
                       2.7e-16 * std::exp(-hMid1 / 1500.0);

        // Cn2 at h2
        double cn2_2 = A * std::exp(-hMid2 / 100.0) +
                       5.94e-53 * std::pow(v_rms / 27.0, 2.0) *
                           std::pow(hMid2, 10.0) * std::exp(-hMid2 / 1000.0) +
                       2.7e-16 * std::exp(-hMid2 / 1500.0);

        integral += 0.5 * (cn2_1 + cn2_2) * dh;
    }

    // Correct for slant path
    double slantIntegral = integral / sinEl;

    NS_LOG_DEBUG("Integrated Cn2: A=" << A << ", integral=" << integral
                                      << " m^(1/3), slant=" << slantIntegral
                                      << " m^(1/3) at elev=" << elevationDeg << " deg");

    return slantIntegral;
}

double
ThzNtnScintillation::ComputeCornerFrequency() const
{
    // Taylor's frozen turbulence hypothesis:
    //   f_c = v_wind / sqrt(2 * pi * L_0)
    // where L_0 is the outer scale of turbulence.

    double fc = m_windSpeed / std::sqrt(2.0 * M_PI * m_outerScale);

    NS_LOG_DEBUG("Corner frequency: v=" << m_windSpeed << " m/s, L0="
                                        << m_outerScale << " m, fc=" << fc << " Hz");

    return fc;
}

double
ThzNtnScintillation::ComputeAmplitudeScintillation_dB(double freqHz,
                                                       double elevationDeg) const
{
    NS_LOG_FUNCTION(this << freqHz << elevationDeg);

    if (elevationDeg <= 0.0)
    {
        return 0.0;
    }

    double freqGHz = freqHz / 1.0e9;

    if (freqGHz < 1.0)
    {
        NS_LOG_WARN("Frequency " << freqGHz << " GHz below scintillation model range");
        return 0.0;
    }

    // Reference scintillation sigma at f_ref = 4 GHz (ITU-R P.618)
    double sigmaRef = ComputeSigmaRef();
    const double fRef = 4.0; // GHz

    // Frequency scaling: sigma ~ f^(7/12) (from f^(7/6) for variance)
    double freqScaling = std::pow(freqGHz / fRef, 7.0 / 12.0);

    // Elevation scaling: sigma ~ (sin(theta))^(-11/12)
    double elevRad = elevationDeg * M_PI / 180.0;
    double sinEl = std::sin(elevRad);
    if (sinEl < 0.01)
    {
        sinEl = 0.01;
    }
    double elevScaling = std::pow(sinEl, -11.0 / 12.0);

    // Aperture averaging factor
    double g = ComputeApertureAveragingFactor(freqHz, elevationDeg);

    // RMS amplitude scintillation
    double sigma = sigmaRef * freqScaling * elevScaling * g;

    NS_LOG_DEBUG("Amplitude scintillation: f=" << freqGHz << " GHz, elev="
                                               << elevationDeg << " deg"
                                               << ", sigma_ref=" << sigmaRef
                                               << ", f_scale=" << freqScaling
                                               << ", el_scale=" << elevScaling
                                               << ", g=" << g
                                               << ", sigma=" << sigma << " dB");

    return sigma;
}

double
ThzNtnScintillation::ComputePhaseScintillation_rad(double freqHz,
                                                    double elevationDeg) const
{
    NS_LOG_FUNCTION(this << freqHz << elevationDeg);

    if (elevationDeg <= 0.0)
    {
        return 0.0;
    }

    // Phase scintillation from the Tatarskii spectrum:
    //
    //   sigma_phi^2 = 2.91 * k^2 * integral(Cn2) * L_0^(5/3) * csc(theta)
    //
    // where k = 2*pi*f/c is the wavenumber.
    //
    // However, the Tatarskii result for phase variance over path of length L is:
    //   sigma_phi^2 = 1.0 * k^2 * integral_Cn2_dz
    //
    // Using the standard weak-turbulence result (Rytov variance analog for phase):
    //   sigma_phi = sqrt(2.91 * k^2 * integrated_Cn2)

    double c = 3.0e8;
    double k = 2.0 * M_PI * freqHz / c; // wavenumber (1/m)

    double intCn2 = ComputeIntegratedCn2(elevationDeg);

    // Phase variance
    double phaseVariance = 2.91 * k * k * intCn2;

    // Apply outer-scale correction (von Karman spectrum reduces variance
    // compared to Kolmogorov for large-scale cutoff):
    // Correction factor ~ (2*pi/L_0)^(-5/3) when L_0 is finite.
    // For typical L_0 = 100 m, this is a modest reduction.
    // We use the von Karman correction: multiply by (1 + (2*pi*l_inner/L_0)^2)^(-1/6)
    // For simplicity, apply a factor based on L_0:
    [[maybe_unused]] double kappa0 = 2.0 * M_PI / m_outerScale; // outer scale wavenumber
    // The correction makes the phase variance finite even for zero inner scale
    // We approximate: sigma_phi^2 = 2.91 * k^2 * intCn2 * L_0^(5/3) correction
    // Using the relationship: for von Karman, the integral converges and the
    // factor is approximately (L_0 / (2*pi))^(5/3) * Gamma(5/6) / Gamma(11/6)
    // For practical purposes, the numerical integration already captures the path;
    // we just use the basic Tatarskii result.

    double sigmaPhase = std::sqrt(std::abs(phaseVariance));

    // Saturate for strong turbulence (sigma_phi > 2*pi is not physically meaningful
    // in the sense that the signal is severely disrupted)
    if (sigmaPhase > 2.0 * M_PI)
    {
        NS_LOG_WARN("Phase scintillation saturated at " << sigmaPhase << " rad");
    }

    NS_LOG_DEBUG("Phase scintillation: f=" << freqHz / 1e9 << " GHz, k=" << k
                                           << " 1/m, intCn2=" << intCn2
                                           << ", sigma_phi=" << sigmaPhase << " rad");

    return sigmaPhase;
}

double
ThzNtnScintillation::GetScintillationSample_dB(double freqHz,
                                                double elevationDeg)
{
    NS_LOG_FUNCTION(this << freqHz << elevationDeg);

    // Generate a time-varying scintillation sample using an AR(1) process.
    //
    // The AR(1) model produces a first-order Markov process:
    //   x[n] = rho * x[n-1] + sqrt(1 - rho^2) * w[n]
    //
    // where:
    //   rho = exp(-T_s / tau_c)  is the AR(1) coefficient
    //   tau_c = 1 / (2*pi*f_c)  is the correlation time
    //   w[n] ~ N(0, sigma^2)    is white Gaussian noise
    //   sigma = amplitude scintillation RMS
    //
    // The output x[n] has variance sigma^2 and power spectral density
    // with -20 dB/decade rolloff above f_c (low-pass filtered Gaussian).

    double sigma = ComputeAmplitudeScintillation_dB(freqHz, elevationDeg);

    if (sigma <= 0.0)
    {
        return 0.0;
    }

    // Correlation time from Taylor's frozen turbulence hypothesis
    double fc = ComputeCornerFrequency();
    double tauC = 1.0 / (2.0 * M_PI * fc);

    // Sampling period
    double Ts = m_samplingPeriod.GetSeconds();

    // AR(1) coefficient
    double rho = std::exp(-Ts / tauC);

    // Generate the AR(1) sample
    double noise = m_normalRng->GetValue(); // N(0,1)

    if (!m_initialized)
    {
        // Initialize the process at steady state
        m_prevSample = sigma * noise;
        m_initialized = true;
    }
    else
    {
        // AR(1) update
        m_prevSample = rho * m_prevSample +
                       std::sqrt(1.0 - rho * rho) * sigma * noise;
    }

    NS_LOG_DEBUG("Scintillation sample: sigma=" << sigma << " dB, fc=" << fc
                                                << " Hz, rho=" << rho
                                                << ", sample=" << m_prevSample << " dB");

    return m_prevSample;
}

double
ThzNtnScintillation::ComputeScintillationFadeDepth_dB(double freqHz,
                                                       double elevationDeg,
                                                       double percentage) const
{
    NS_LOG_FUNCTION(this << freqHz << elevationDeg << percentage);

    if (percentage <= 0.0 || percentage >= 100.0)
    {
        NS_LOG_WARN("Invalid percentage " << percentage << " (must be 0 < p < 100)");
        return 0.0;
    }

    double sigma = ComputeAmplitudeScintillation_dB(freqHz, elevationDeg);

    if (sigma <= 0.0)
    {
        return 0.0;
    }

    // ITU-R P.618 fade depth for a given time percentage.
    //
    // For Gaussian scintillation, the fade depth exceeded for p% of time is:
    //   A(p) = a(p) * sigma
    //
    // where a(p) is derived from the inverse of the complementary cumulative
    // distribution function of the standard normal distribution.
    //
    // ITU-R P.618 provides a more refined formula that accounts for the
    // non-Gaussian tail behavior of scintillation at low percentages:
    //
    // For fade (signal decrease):
    //   A_s(p) = -a(p) * sigma                                 for p >= 50%
    //   A_s(p) = sigma * [-0.061*(log10(p))^3 + 0.072*(log10(p))^2
    //            - 1.71*(log10(p)) + 3.0]                      for 1% <= p < 50%
    //   A_s(p) as above with extrapolation                     for p < 1%

    double fadeDepth = 0.0;

    if (percentage >= 50.0)
    {
        // For p >= 50%, the fade is near zero or enhancement
        // Use standard Gaussian: Q^{-1}(p/100) * sigma
        // At 50%, fade = 0
        double p = percentage / 100.0;
        // Approximation of inverse Q function (erfinv)
        // Q^{-1}(p) for p close to 0.5 is close to 0
        double t = std::sqrt(-2.0 * std::log(1.0 - p));
        double a = t - (2.515517 + 0.802853 * t + 0.010328 * t * t) /
                           (1.0 + 1.432788 * t + 0.189269 * t * t +
                            0.001308 * t * t * t);
        fadeDepth = a * sigma;
    }
    else if (percentage >= 1.0)
    {
        // ITU-R P.618 empirical formula for 1% <= p < 50%
        double logP = std::log10(percentage);
        double a = -0.061 * logP * logP * logP +
                    0.072 * logP * logP -
                    1.71 * logP + 3.0;
        fadeDepth = a * sigma;
    }
    else
    {
        // p < 1%: extrapolation using inverse normal approximation
        double p = percentage / 100.0;
        // Rational approximation of inverse normal CDF (Abramowitz & Stegun 26.2.23)
        double t = std::sqrt(-2.0 * std::log(p));
        double c0 = 2.515517;
        double c1 = 0.802853;
        double c2 = 0.010328;
        double d1 = 1.432788;
        double d2 = 0.189269;
        double d3 = 0.001308;
        double a = t - (c0 + c1 * t + c2 * t * t) /
                           (1.0 + d1 * t + d2 * t * t + d3 * t * t * t);
        fadeDepth = a * sigma;
    }

    NS_LOG_DEBUG("Fade depth: p=" << percentage << "%, sigma=" << sigma
                                  << " dB, fade=" << fadeDepth << " dB");

    return fadeDepth;
}

// --- Configuration setters ---

void
ThzNtnScintillation::SetTurbulenceStrength(TurbulenceLevel level)
{
    NS_LOG_FUNCTION(this << level);
    m_turbulenceLevel = level;
}

void
ThzNtnScintillation::SetWindSpeed(double windSpeed_m_s)
{
    NS_LOG_FUNCTION(this << windSpeed_m_s);
    m_windSpeed = windSpeed_m_s;
}

void
ThzNtnScintillation::SetGroundTemperature(double temperature_K)
{
    NS_LOG_FUNCTION(this << temperature_K);
    m_groundTemperature = temperature_K;
}

void
ThzNtnScintillation::SetAntennaDiameter(double diameter_m)
{
    NS_LOG_FUNCTION(this << diameter_m);
    m_antennaDiameter = diameter_m;
}

void
ThzNtnScintillation::SetRelativeHumidity(double humidity)
{
    NS_LOG_FUNCTION(this << humidity);
    m_relativeHumidity = humidity;
}

int64_t
ThzNtnScintillation::AssignStreams(int64_t stream)
{
    NS_LOG_FUNCTION(this << stream);
    m_normalRng->SetStream(stream);
    return 1;
}

} // namespace ns3
