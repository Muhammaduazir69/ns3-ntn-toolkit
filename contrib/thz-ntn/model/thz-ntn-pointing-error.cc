/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-only
 * Author: Muhammad Uzair
 *
 * Beam Pointing Error Model for THz-NTN Links -- Implementation
 */

#include "thz-ntn-pointing-error.h"

#include <ns3/boolean.h>
#include <ns3/double.h>
#include <ns3/log.h>

#include <cmath>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("ThzNtnPointingError");

NS_OBJECT_ENSURE_REGISTERED(ThzNtnPointingError);

TypeId
ThzNtnPointingError::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::ThzNtnPointingError")
            .SetParent<Object>()
            .SetGroupName("ThzNtn")
            .AddConstructor<ThzNtnPointingError>()
            .AddAttribute(
                "VibrationRms_deg",
                "RMS satellite platform vibration jitter in degrees. "
                "Typical range 0.01--0.1 deg for LEO spacecraft due to "
                "reaction wheels, solar panel deployment, and thermal "
                "deformation.",
                DoubleValue(0.02),
                MakeDoubleAccessor(&ThzNtnPointingError::m_vibrationRms_deg),
                MakeDoubleChecker<double>(0.0, 10.0))
            .AddAttribute(
                "EphemerisUpdateInterval_s",
                "Interval between ephemeris updates in seconds. Longer "
                "intervals allow J2 perturbation position errors to grow. "
                "With 10 s update: ~0.001 rad error; without correction: "
                "~0.23 rad.",
                DoubleValue(10.0),
                MakeDoubleAccessor(
                    &ThzNtnPointingError::m_ephemerisUpdateInterval_s),
                MakeDoubleChecker<double>(0.001, 3600.0))
            .AddAttribute(
                "TrackingUpdateRate_Hz",
                "Beam tracking update rate in Hz. Higher rates reduce "
                "tracking latency error at the cost of computation.",
                DoubleValue(100.0),
                MakeDoubleAccessor(
                    &ThzNtnPointingError::m_trackingUpdateRate_Hz),
                MakeDoubleChecker<double>(0.1, 1e6))
            .AddAttribute(
                "TrackingLatency_ms",
                "Tracking processing latency in milliseconds. This is the "
                "delay between beam position measurement and steering "
                "command execution.",
                DoubleValue(1.0),
                MakeDoubleAccessor(&ThzNtnPointingError::m_trackingLatency_ms),
                MakeDoubleChecker<double>(0.0, 1000.0))
            .AddAttribute(
                "EnableAtmosphericRefraction",
                "Enable atmospheric refraction model (ITU-R P.834). "
                "Refraction bends the beam at low elevation angles.",
                BooleanValue(true),
                MakeBooleanAccessor(&ThzNtnPointingError::m_enableRefraction),
                MakeBooleanChecker());
    return tid;
}

ThzNtnPointingError::ThzNtnPointingError()
    : m_vibrationRms_deg(0.02),
      m_ephemerisUpdateInterval_s(10.0),
      m_trackingUpdateRate_Hz(100.0),
      m_trackingLatency_ms(1.0),
      m_enableRefraction(true)
{
    NS_LOG_FUNCTION(this);
    m_uniformRv = CreateObject<UniformRandomVariable>();
}

ThzNtnPointingError::~ThzNtnPointingError()
{
    NS_LOG_FUNCTION(this);
}

double
ThzNtnPointingError::ComputeVibrationError_deg() const
{
    // Platform vibration is modeled as Gaussian jitter with the configured
    // RMS value.  The RMS of the 2-D pointing error equals the single-axis
    // sigma (the Rayleigh parameter).
    return m_vibrationRms_deg;
}

double
ThzNtnPointingError::ComputeJ2PerturbationError_deg(
    double satAltitude_km) const
{
    // J2 perturbation causes along-track and cross-track position errors
    // that grow between ephemeris updates.
    //
    // Position error growth rate:
    //   delta_pos ~ J2 * (R_E / a)^2 * n * dt
    //
    // where:
    //   J2 = 1.08263e-3 (Earth oblateness)
    //   a  = orbital semi-major axis (m)
    //   n  = mean motion (rad/s) = sqrt(mu / a^3)
    //   dt = ephemeris update interval (s)
    //
    // The angular error as seen from the ground is:
    //   theta_J2 = delta_pos / a  (rad)

    double a_m = (R_EARTH_KM + satAltitude_km) * 1.0e3; // semi-major axis (m)
    double n = std::sqrt(MU_EARTH / (a_m * a_m * a_m));  // mean motion (rad/s)

    double reRatio = (R_EARTH_KM * 1.0e3) / a_m; // R_E / a
    double reRatio2 = reRatio * reRatio;

    // Position error accumulated over the update interval
    double dt = m_ephemerisUpdateInterval_s;
    double deltaPos_m = J2 * reRatio2 * n * a_m * dt;

    // Angular error as seen from ground (approximate)
    double thetaJ2_rad = deltaPos_m / a_m;

    NS_LOG_DEBUG("J2 perturbation: a=" << a_m << " m, n=" << n
                                       << " rad/s, deltaPos="
                                       << deltaPos_m << " m, theta="
                                       << thetaJ2_rad * RAD_TO_DEG
                                       << " deg");

    return thetaJ2_rad * RAD_TO_DEG;
}

double
ThzNtnPointingError::ComputeRefractionError_deg(double elevationDeg) const
{
    if (!m_enableRefraction)
    {
        return 0.0;
    }

    // ITU-R P.834 atmospheric refraction model
    //
    // Refraction angle (arcminutes):
    //   R = (16.27 / T_K) * (P_hPa / 1013.25) * cot(elevation)
    //
    // Using standard atmosphere at sea level: T = 288.15 K, P = 1013.25 hPa.
    // The refraction itself does not cause pointing error if perfectly
    // compensated; the error comes from refraction prediction uncertainty,
    // which is typically ~10% of the total refraction.

    // Clamp elevation to avoid singularity at horizon
    double elev = std::max(elevationDeg, 3.0);
    double elevRad = elev * DEG_TO_RAD;

    // Standard atmosphere parameters
    constexpr double T_K = 288.15;      // temperature (K)
    constexpr double P_hPa = 1013.25;   // pressure (hPa)

    // Total refraction in arcminutes (ITU-R P.834)
    double cotElev = std::cos(elevRad) / std::sin(elevRad);
    double refractionArcmin = (16.27 / T_K) * (P_hPa / 1013.25) * cotElev;

    // Refraction prediction uncertainty (~10% of total refraction)
    constexpr double uncertaintyFraction = 0.10;
    double errorArcmin = refractionArcmin * uncertaintyFraction;
    double errorDeg = errorArcmin / 60.0;

    NS_LOG_DEBUG("Refraction at elev=" << elevationDeg
                                       << " deg: total="
                                       << refractionArcmin
                                       << " arcmin, error="
                                       << errorDeg << " deg");

    return errorDeg;
}

double
ThzNtnPointingError::ComputeTrackingError_deg(double elevationDeg,
                                              double satVelocity_km_s) const
{
    // Tracking latency causes a pointing lag equal to angular velocity
    // times the effective latency.
    //
    // The effective latency includes both the processing latency and the
    // inverse of the update rate (whichever is larger):
    //   t_eff = max(trackingLatency, 1/updateRate)
    //
    // Angular velocity as seen from ground depends on elevation and
    // orbital velocity.  For LEO at 550 km, ~1 deg/s at zenith,
    // increasing to ~3 deg/s at low elevation.
    //
    // omega_apparent ~ v_sat * cos(elev) / (R_E + h)

    double elevRad = std::max(elevationDeg, 5.0) * DEG_TO_RAD;

    // Approximate angular velocity as seen from ground (deg/s)
    // Using satellite velocity and slant geometry
    constexpr double defaultAltitude_km = 550.0;
    double orbitalRadius_km = R_EARTH_KM + defaultAltitude_km;

    // Angular velocity (rad/s) = v_tangential / R
    double vSat_m_s = satVelocity_km_s * 1.0e3;
    double angularVelocity_rad_s = vSat_m_s / (orbitalRadius_km * 1.0e3);

    // Apparent angular velocity from ground increases at lower elevation
    double apparentOmega_deg_s =
        angularVelocity_rad_s * RAD_TO_DEG * std::cos(elevRad);

    // Effective latency (seconds)
    double latencyFromUpdate_s = 1.0 / m_trackingUpdateRate_Hz;
    double latencyFromProcessing_s = m_trackingLatency_ms * 1.0e-3;
    double effectiveLatency_s =
        std::max(latencyFromUpdate_s, latencyFromProcessing_s);

    double trackingError_deg = apparentOmega_deg_s * effectiveLatency_s;

    NS_LOG_DEBUG("Tracking error: omega=" << apparentOmega_deg_s
                                          << " deg/s, latency="
                                          << effectiveLatency_s * 1e3
                                          << " ms, error="
                                          << trackingError_deg << " deg");

    return trackingError_deg;
}

double
ThzNtnPointingError::ComputePointingError_deg(double elevationDeg,
                                              double satVelocity_km_s) const
{
    NS_LOG_FUNCTION(this << elevationDeg << satVelocity_km_s);

    // Estimate satellite altitude from velocity (circular orbit assumption)
    // v = sqrt(mu / r), so r = mu / v^2, h = r - R_E
    double vSat_m_s = satVelocity_km_s * 1.0e3;
    double orbitalRadius_m = MU_EARTH / (vSat_m_s * vSat_m_s);
    double satAltitude_km =
        (orbitalRadius_m / 1.0e3) - R_EARTH_KM;

    // Clamp altitude to reasonable LEO/MEO range
    satAltitude_km = std::max(200.0, std::min(satAltitude_km, 36000.0));

    // Compute individual error components
    double eVibration = ComputeVibrationError_deg();
    double eJ2 = ComputeJ2PerturbationError_deg(satAltitude_km);
    double eRefraction = ComputeRefractionError_deg(elevationDeg);
    double eTracking = ComputeTrackingError_deg(elevationDeg, satVelocity_km_s);

    // RSS combination (independent error sources)
    double totalError = std::sqrt(eVibration * eVibration +
                                  eJ2 * eJ2 +
                                  eRefraction * eRefraction +
                                  eTracking * eTracking);

    NS_LOG_INFO("Pointing error components [deg]: vibration="
                << eVibration << ", J2=" << eJ2
                << ", refraction=" << eRefraction
                << ", tracking=" << eTracking
                << " -> total=" << totalError);

    return totalError;
}

double
ThzNtnPointingError::ComputePointingLoss_dB(double pointingError_deg,
                                            double beamwidth3dB_deg) const
{
    NS_LOG_FUNCTION(this << pointingError_deg << beamwidth3dB_deg);

    if (beamwidth3dB_deg <= 0.0)
    {
        NS_LOG_WARN("Invalid beamwidth: " << beamwidth3dB_deg
                                          << " deg, returning 0 dB loss");
        return 0.0;
    }

    // Approximate parabolic model for small pointing errors:
    //   L_point = 12 * (theta_error / theta_3dB)^2  [dB]
    //
    // Valid when theta_error < theta_3dB / 2

    double ratio = pointingError_deg / beamwidth3dB_deg;
    double loss_dB = 12.0 * ratio * ratio;

    NS_LOG_DEBUG("Pointing loss (parabolic): " << loss_dB
                                               << " dB for ratio="
                                               << ratio);

    return loss_dB;
}

double
ThzNtnPointingError::ComputeGaussianPointingLoss_dB(
    double pointingError_deg,
    double beamwidth3dB_deg) const
{
    NS_LOG_FUNCTION(this << pointingError_deg << beamwidth3dB_deg);

    if (beamwidth3dB_deg <= 0.0)
    {
        NS_LOG_WARN("Invalid beamwidth: " << beamwidth3dB_deg
                                          << " deg, returning 0 dB loss");
        return 0.0;
    }

    // Exact Gaussian beam pointing loss:
    //   L_point = G_max * (2 * theta_error / theta_3dB)^2 * ln(2)
    //
    // In dB (using the power pattern):
    //   L_dB = (2 * theta_error / theta_3dB)^2 * ln(2) * (10 / ln(10))
    //
    // Note: 10*ln(2)/ln(10) = 10 * 0.6931 / 2.3026 = 3.0103
    // which gives the familiar 3 dB at the half-power point.
    // For the antenna gain pattern in dB:
    //   L_dB = 12 * ln(2) / ln(10) * (theta / theta_3dB)^2
    //        ≈ 8.686 * ln(2) * (2*theta/theta_3dB)^2  ... exact form

    double ratio = pointingError_deg / beamwidth3dB_deg;

    // Using the exact Gaussian: exp(-4*ln(2)*(theta/theta_3dB)^2)
    // In dB: -10*log10(exp(-4*ln(2)*ratio^2))
    //       = 4 * ln(2) * ratio^2 * 10/ln(10)
    //       = 4 * ln(2) * ratio^2 * 4.3429
    //       = 12.041 * ratio^2
    double loss_dB = 4.0 * std::log(2.0) * ratio * ratio *
                     (10.0 / std::log(10.0));

    NS_LOG_DEBUG("Pointing loss (Gaussian): " << loss_dB
                                              << " dB for ratio="
                                              << ratio);

    return loss_dB;
}

double
ThzNtnPointingError::ComputeTotalPointingLoss_dB(
    double elevationDeg,
    double satVelocity_km_s,
    double beamwidth3dB_deg) const
{
    NS_LOG_FUNCTION(this << elevationDeg << satVelocity_km_s
                         << beamwidth3dB_deg);

    double error = ComputePointingError_deg(elevationDeg, satVelocity_km_s);
    double loss = ComputePointingLoss_dB(error, beamwidth3dB_deg);

    return loss;
}

double
ThzNtnPointingError::GetPointingErrorSample_deg() const
{
    NS_LOG_FUNCTION(this);

    // The pointing error magnitude is Rayleigh-distributed.
    // If the x and y components are independent Gaussian with std dev sigma,
    // the magnitude r = sqrt(x^2 + y^2) follows Rayleigh(sigma).
    //
    // CDF: F(r) = 1 - exp(-r^2 / (2*sigma^2))
    // Inverse CDF: r = sigma * sqrt(-2 * ln(1 - U)),  U ~ Uniform(0,1)
    //
    // We use the combined RMS vibration error as sigma (this provides a
    // stochastic instantaneous pointing error around the deterministic RMS).

    double sigma = m_vibrationRms_deg;

    // Generate Rayleigh sample via inverse CDF
    double u = m_uniformRv->GetValue(0.0, 1.0);

    // Avoid log(0)
    if (u >= 1.0)
    {
        u = 1.0 - 1.0e-15;
    }

    double sample = sigma * std::sqrt(-2.0 * std::log(1.0 - u));

    NS_LOG_DEBUG("Rayleigh sample: sigma=" << sigma
                                           << ", u=" << u
                                           << ", sample=" << sample
                                           << " deg");

    return sample;
}

void
ThzNtnPointingError::SetVibrationRms_deg(double rms)
{
    NS_LOG_FUNCTION(this << rms);
    m_vibrationRms_deg = rms;
}

void
ThzNtnPointingError::SetTrackingUpdateRate_Hz(double rate)
{
    NS_LOG_FUNCTION(this << rate);
    m_trackingUpdateRate_Hz = rate;
}

void
ThzNtnPointingError::SetEphemerisUpdateInterval_s(double interval)
{
    NS_LOG_FUNCTION(this << interval);
    m_ephemerisUpdateInterval_s = interval;
}

int64_t
ThzNtnPointingError::AssignStreams(int64_t stream)
{
    NS_LOG_FUNCTION(this << stream);
    m_uniformRv->SetStream(stream);
    return 1;
}

} // namespace ns3
