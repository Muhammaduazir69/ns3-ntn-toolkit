/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2024 Muhammad Uzair
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Author: Muhammad Uzair <uk5595985@gmail.com>
 */

#ifndef THZ_NTN_SCINTILLATION_H
#define THZ_NTN_SCINTILLATION_H

#include <ns3/nstime.h>
#include <ns3/object.h>
#include <ns3/random-variable-stream.h>

#include <string>

namespace ns3
{

/**
 * \ingroup thz-ntn
 *
 * \brief Atmospheric scintillation model for THz Non-Terrestrial Network (NTN) links.
 *
 * This class models amplitude and phase scintillation caused by tropospheric
 * turbulence on THz satellite-to-ground links. It extends the ITU-R P.618
 * scintillation prediction method (Section 2.4) to terahertz frequencies.
 *
 * At THz frequencies the scintillation variance scales as \f$f^{7/6}\f$, making
 * it significantly stronger than at Ka-band. The elevation dependence follows
 * \f$(\csc\theta)^{11/6}\f$, causing severe degradation below 10 degrees.
 *
 * \section scint_amplitude Amplitude Scintillation
 *
 * The scintillation intensity (standard deviation of log-amplitude fluctuations)
 * is computed from ITU-R P.618:
 * \f[
 *   \sigma_{ref} = 3.6 \times 10^{-3} + 10^{-4} \cdot N_{wet}
 * \f]
 * scaled to the operating frequency:
 * \f[
 *   \sigma = \sigma_{ref} \cdot \left(\frac{f}{f_{ref}}\right)^{7/12}
 *            \cdot \frac{g(D)}{\left(\sin\theta\right)^{11/12}}
 * \f]
 * where \f$g(D)\f$ is the antenna averaging factor.
 *
 * \section scint_phase Phase Scintillation
 *
 * Phase variance from the Tatarskii spectrum of refractive-index fluctuations:
 * \f[
 *   \sigma_\phi^2 = 2.91 \cdot k^2 \cdot \int_0^L C_n^2(h) \, dh
 *                   \cdot L_0^{5/3} \cdot \csc(\theta)
 * \f]
 *
 * \section scint_timeseries Time-Series Generation
 *
 * Time-varying scintillation samples are generated using a first-order
 * autoregressive AR(1) process with the corner frequency derived from
 * Taylor's frozen turbulence hypothesis:
 * \f[
 *   f_c = v_{wind} / \sqrt{2\pi \cdot L_0}
 * \f]
 * where \f$v_{wind}\f$ is the transverse wind speed and \f$L_0\f$ is the
 * outer scale of turbulence.
 */
class ThzNtnScintillation : public Object
{
  public:
    /**
     * \brief Get the type ID.
     * \return the object TypeId.
     */
    static TypeId GetTypeId(void);

    /**
     * \brief Default constructor.
     */
    ThzNtnScintillation();

    /**
     * \brief Destructor.
     */
    ~ThzNtnScintillation() override;

    /**
     * \brief Turbulence strength enumeration.
     */
    enum TurbulenceLevel
    {
        WEAK = 0,      ///< Weak turbulence (Cn2 ~ 1e-17 m^{-2/3})
        MODERATE = 1,  ///< Moderate turbulence (Cn2 ~ 1e-15 m^{-2/3})
        STRONG = 2     ///< Strong turbulence (Cn2 ~ 1e-13 m^{-2/3})
    };

    // --- Amplitude scintillation ---

    /**
     * \brief Compute the RMS amplitude scintillation (standard deviation).
     *
     * Returns the standard deviation of log-amplitude fluctuations in dB,
     * based on ITU-R P.618 Section 2.4 extended to THz frequencies.
     *
     * \param freqHz       operating frequency in Hz.
     * \param elevationDeg elevation angle in degrees (0--90).
     * \return RMS amplitude scintillation in dB.
     */
    double ComputeAmplitudeScintillation_dB(double freqHz,
                                            double elevationDeg) const;

    // --- Phase scintillation ---

    /**
     * \brief Compute the RMS phase scintillation.
     *
     * Returns the standard deviation of phase fluctuations in radians,
     * from the Tatarskii spectrum.
     *
     * \param freqHz       operating frequency in Hz.
     * \param elevationDeg elevation angle in degrees.
     * \return RMS phase scintillation in radians.
     */
    double ComputePhaseScintillation_rad(double freqHz,
                                         double elevationDeg) const;

    // --- Time-varying samples ---

    /**
     * \brief Get a time-varying scintillation amplitude sample.
     *
     * Uses an AR(1) process to generate correlated Gaussian samples
     * representing instantaneous scintillation fade/enhancement in dB.
     *
     * \param freqHz       operating frequency in Hz.
     * \param elevationDeg elevation angle in degrees.
     * \return instantaneous scintillation amplitude in dB (can be positive or negative).
     */
    double GetScintillationSample_dB(double freqHz,
                                     double elevationDeg);

    // --- Fade depth statistics ---

    /**
     * \brief Compute the scintillation fade depth exceeded for a given
     *        percentage of time.
     *
     * Based on the Gaussian distribution of scintillation amplitudes:
     * \f[
     *   A_p = a(p) \cdot \sigma
     * \f]
     * where \f$a(p)\f$ is derived from the inverse complementary cumulative
     * distribution function.
     *
     * \param freqHz       operating frequency in Hz.
     * \param elevationDeg elevation angle in degrees.
     * \param percentage   percentage of time the fade is exceeded (0--100).
     * \return fade depth in dB.
     */
    double ComputeScintillationFadeDepth_dB(double freqHz,
                                            double elevationDeg,
                                            double percentage) const;

    // --- Configuration ---

    /**
     * \brief Set the turbulence strength level.
     * \param level turbulence level (WEAK, MODERATE, or STRONG).
     */
    void SetTurbulenceStrength(TurbulenceLevel level);

    /**
     * \brief Set the transverse wind speed.
     * \param windSpeed_m_s wind speed in m/s.
     */
    void SetWindSpeed(double windSpeed_m_s);

    /**
     * \brief Set the ground temperature.
     * \param temperature_K temperature in Kelvin.
     */
    void SetGroundTemperature(double temperature_K);

    /**
     * \brief Set the antenna diameter for aperture averaging.
     * \param diameter_m antenna diameter in meters.
     */
    void SetAntennaDiameter(double diameter_m);

    /**
     * \brief Set the relative humidity at ground level.
     * \param humidity relative humidity (0--100 %).
     */
    void SetRelativeHumidity(double humidity);

    /**
     * \brief Assign a fixed random variable stream number.
     *
     * This is required by the ns-3 random variable subsystem for
     * reproducible simulations.
     *
     * \param stream first stream index to use.
     * \return the number of stream indices consumed.
     */
    int64_t AssignStreams(int64_t stream);

  private:
    /**
     * \brief Compute the wet term of refractivity N_wet from temperature
     *        and humidity (ITU-R P.453).
     * \return N_wet in ppm.
     */
    double ComputeNwet() const;

    /**
     * \brief Compute the reference scintillation sigma at the reference
     *        frequency (4 GHz), following ITU-R P.618 Section 2.4.1.
     * \return reference sigma in dB.
     */
    double ComputeSigmaRef() const;

    /**
     * \brief Compute the antenna aperture averaging factor g(D).
     *
     * Larger antennas average out scintillation over the aperture.
     *
     * \param freqHz       operating frequency in Hz.
     * \param elevationDeg elevation angle in degrees.
     * \return aperture averaging factor (0 < g <= 1).
     */
    double ComputeApertureAveragingFactor(double freqHz,
                                          double elevationDeg) const;

    /**
     * \brief Get the ground-level Cn2 value for the configured turbulence level.
     * \return Cn2 in m^{-2/3}.
     */
    double GetCn2Ground() const;

    /**
     * \brief Compute the integrated Cn2 along the slant path.
     *
     * Uses the Hufnagel-Valley model for the Cn2 altitude profile:
     * \f[
     *   C_n^2(h) = A \exp(-h/100) + 5.94 \times 10^{-53}
     *              (v/27)^2 h^{10} \exp(-h/1000)
     *              + 2.7 \times 10^{-16} \exp(-h/1500)
     * \f]
     *
     * \param elevationDeg elevation angle in degrees.
     * \return integrated Cn2 in m^{1/3}.
     */
    double ComputeIntegratedCn2(double elevationDeg) const;

    /**
     * \brief Compute the corner frequency of the scintillation power spectrum.
     *
     * From Taylor's frozen turbulence hypothesis:
     * \f$ f_c = v_{wind} / \sqrt{2\pi L_0} \f$
     *
     * \return corner frequency in Hz.
     */
    double ComputeCornerFrequency() const;

    // --- Member variables ---
    TurbulenceLevel m_turbulenceLevel;   ///< turbulence strength setting
    double m_windSpeed;                  ///< transverse wind speed in m/s
    double m_groundTemperature;          ///< ground temperature in Kelvin
    double m_antennaDiameter;            ///< antenna diameter in meters
    double m_relativeHumidity;           ///< relative humidity in %
    Time m_samplingPeriod;               ///< sampling period for time series
    /// THZ-06: last time the AR(1) process was advanced, so its correlation
    /// follows simulated time rather than the call rate. SamplingPeriod is
    /// now only the seed used for the very first sample.
    Time m_lastSampleTime{Seconds(0)};
    double m_outerScale;                 ///< outer scale of turbulence L_0 in m
    double m_turbulentLayerHeight;       ///< effective turbulent layer height in m

    // AR(1) process state
    double m_prevSample;                 ///< previous AR(1) sample
    bool m_initialized;                  ///< whether AR(1) process is initialized

    // Random number generator
    Ptr<NormalRandomVariable> m_normalRng; ///< Gaussian RNG for scintillation
};

} // namespace ns3

#endif /* THZ_NTN_SCINTILLATION_H */
