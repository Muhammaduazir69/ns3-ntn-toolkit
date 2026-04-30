/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-only
 * Author: Muhammad Uzair
 *
 * Beam Pointing Error Model for THz-NTN Links
 *
 * Composite beam pointing error model for terahertz non-terrestrial network
 * links where pencil beams (sub-degree beamwidths) make alignment a critical
 * challenge.  The model captures four independent error sources and combines
 * them via root-sum-square (RSS) to produce a total pointing error:
 *
 *   1. Satellite platform vibration -- Gaussian jitter from reaction wheels,
 *      solar panel deployment, and thermal deformation.
 *   2. J2 gravitational perturbation -- along-track and cross-track position
 *      errors from Earth oblateness (J2 = 1.08263e-3).
 *   3. Atmospheric refraction -- beam bending at low elevation angles
 *      following ITU-R P.834.
 *   4. Tracking latency -- angular lag from finite beam update rate.
 *
 * The pointing loss is computed using both the approximate parabolic model
 * (12 * (theta/theta_3dB)^2 dB) and the exact Gaussian beam model.
 *
 * References:
 *   [1] ITU-R P.834-9, "Effects of tropospheric refraction on
 *       radiowave propagation"
 *   [2] D. Giggenbach et al., "Optical satellite downlinks to optical
 *       ground stations and high-altitude platforms," Proc. SPIE, 2008.
 *   [3] O. Montenbruck and E. Gill, "Satellite Orbits: Models, Methods
 *       and Applications," Springer, 2000.
 *   [4] C. A. Balanis, "Antenna Theory: Analysis and Design," Wiley, 2016.
 */

#ifndef THZ_NTN_POINTING_ERROR_H
#define THZ_NTN_POINTING_ERROR_H

#include <ns3/object.h>
#include <ns3/random-variable-stream.h>

namespace ns3
{

/**
 * \ingroup thz-ntn
 * \brief Beam pointing error model for THz-NTN pencil-beam links
 *
 * This class models the composite beam pointing error arising from satellite
 * platform vibration, J2 gravitational perturbation, atmospheric refraction,
 * and tracking latency.  The individual error components are combined via
 * root-sum-square (RSS), and the resulting pointing loss is computed for
 * both the approximate parabolic and exact Gaussian beam models.
 *
 * Pointing error samples are drawn from a Rayleigh distribution (the
 * magnitude of a 2-D Gaussian pointing jitter vector), which is the
 * standard model for random misalignment of narrow beams.
 */
class ThzNtnPointingError : public Object
{
  public:
    /**
     * \brief Get the type ID
     * \return the object TypeId
     */
    static TypeId GetTypeId();

    ThzNtnPointingError();
    ~ThzNtnPointingError() override;

    /**
     * \brief Compute the RMS total pointing error in degrees
     *
     * Combines all four error sources (vibration, J2, refraction, tracking)
     * via root-sum-square.
     *
     * \param elevationDeg elevation angle in degrees (5--90)
     * \param satVelocity_km_s satellite ground-track velocity in km/s
     * \return RMS pointing error in degrees
     */
    double ComputePointingError_deg(double elevationDeg,
                                    double satVelocity_km_s) const;

    /**
     * \brief Compute pointing loss from beam misalignment
     *
     * Uses the approximate parabolic model:
     *   L_point = 12 * (theta_error / theta_3dB)^2  [dB]
     *
     * \param pointingError_deg pointing error in degrees
     * \param beamwidth3dB_deg half-power beamwidth in degrees
     * \return pointing loss in dB (positive value)
     */
    double ComputePointingLoss_dB(double pointingError_deg,
                                  double beamwidth3dB_deg) const;

    /**
     * \brief Compute total pointing loss from elevation and velocity
     *
     * Convenience method combining ComputePointingError_deg and
     * ComputePointingLoss_dB.
     *
     * \param elevationDeg elevation angle in degrees
     * \param satVelocity_km_s satellite velocity in km/s
     * \param beamwidth3dB_deg half-power beamwidth in degrees
     * \return total pointing loss in dB (positive value)
     */
    double ComputeTotalPointingLoss_dB(double elevationDeg,
                                       double satVelocity_km_s,
                                       double beamwidth3dB_deg) const;

    /**
     * \brief Draw a random pointing error sample in degrees
     *
     * The pointing error magnitude is Rayleigh-distributed, with its
     * scale parameter (sigma) equal to the RMS single-axis jitter.
     * This method uses the combined RMS error from all deterministic
     * sources as sigma, producing a stochastic realization.
     *
     * \return random pointing error in degrees (>= 0)
     */
    double GetPointingErrorSample_deg() const;

    /**
     * \brief Set the RMS satellite platform vibration jitter
     * \param rms vibration jitter in degrees
     */
    void SetVibrationRms_deg(double rms);

    /**
     * \brief Set the beam tracking update rate
     * \param rate tracking update rate in Hz
     */
    void SetTrackingUpdateRate_Hz(double rate);

    /**
     * \brief Set the ephemeris update interval
     * \param interval interval in seconds
     */
    void SetEphemerisUpdateInterval_s(double interval);

    /**
     * \brief Compute pointing loss using exact Gaussian beam model
     *
     * L_point = G_max * (2 * theta_error / theta_3dB)^2 * ln(2)  [dB]
     *
     * \param pointingError_deg pointing error in degrees
     * \param beamwidth3dB_deg half-power beamwidth in degrees
     * \return pointing loss in dB (positive value)
     */
    double ComputeGaussianPointingLoss_dB(double pointingError_deg,
                                          double beamwidth3dB_deg) const;

    /**
     * \brief Assign a fixed random-variable stream number
     * \param stream first stream index to use
     * \return the number of stream indices consumed
     */
    int64_t AssignStreams(int64_t stream);

  private:
    /**
     * \brief Compute vibration error component
     * \return vibration pointing error in degrees
     */
    double ComputeVibrationError_deg() const;

    /**
     * \brief Compute J2 perturbation error component
     * \param satAltitude_km satellite altitude in km
     * \return J2 perturbation pointing error in degrees
     */
    double ComputeJ2PerturbationError_deg(double satAltitude_km) const;

    /**
     * \brief Compute atmospheric refraction error component
     * \param elevationDeg elevation angle in degrees
     * \return refraction-induced pointing error in degrees
     */
    double ComputeRefractionError_deg(double elevationDeg) const;

    /**
     * \brief Compute tracking latency error component
     * \param elevationDeg elevation angle in degrees
     * \param satVelocity_km_s satellite velocity in km/s
     * \return tracking latency pointing error in degrees
     */
    double ComputeTrackingError_deg(double elevationDeg,
                                    double satVelocity_km_s) const;

    // --- Configurable attributes ---

    double m_vibrationRms_deg;         //!< RMS vibration jitter (degrees)
    double m_ephemerisUpdateInterval_s; //!< Ephemeris update interval (seconds)
    double m_trackingUpdateRate_Hz;    //!< Beam tracking update rate (Hz)
    double m_trackingLatency_ms;       //!< Tracking latency (milliseconds)
    bool   m_enableRefraction;         //!< Enable atmospheric refraction model

    // --- Internal state ---

    Ptr<UniformRandomVariable> m_uniformRv; //!< Uniform RV for Rayleigh sampling

    // --- Physical constants ---

    static constexpr double J2 = 1.08263e-3;        //!< Earth J2 coefficient
    static constexpr double R_EARTH_KM = 6371.0;    //!< Earth radius (km)
    static constexpr double MU_EARTH = 3.986004e14;  //!< Earth GM (m^3/s^2)
    static constexpr double DEG_TO_RAD = 3.14159265358979323846 / 180.0;
    static constexpr double RAD_TO_DEG = 180.0 / 3.14159265358979323846;
};

} // namespace ns3

#endif // THZ_NTN_POINTING_ERROR_H
