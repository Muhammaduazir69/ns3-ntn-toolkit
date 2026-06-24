/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-only
 * Author: Muhammad Uzair
 *
 * HITRAN-based Molecular Absorption Model for THz-NTN Links
 *
 * Altitude-stratified molecular absorption model for terahertz non-terrestrial
 * network links.  Unlike terrestrial THz models (e.g. TeraSim), this model
 * accounts for the vertical structure of the atmosphere by integrating
 * absorption along slant paths through six ITU-R P.835 standard atmosphere
 * layers spanning 0--100 km.
 *
 * Key features:
 *   - HITRAN-derived absorption line database (H2O + O2)
 *   - Van Vleck--Weisskopf line shape for individual resonance lines
 *   - Continuum absorption between resonance lines
 *   - Slant-path integration with Earth-curvature correction
 *   - Automatic ISL detection (both endpoints > 100 km -> vacuum)
 *   - Selectable humidity profiles: tropical, midlatitude-summer,
 *     midlatitude-winter, subarctic (ITU-R P.835)
 *
 * References:
 *   [1] ITU-R P.835-6, "Reference standard atmospheres"
 *   [2] ITU-R P.676-13, "Attenuation by atmospheric gases and related effects"
 *   [3] HITRAN database, https://hitran.org
 *   [4] J. M. Jornet and I. F. Akyildiz, "Channel modeling and capacity
 *       analysis for electromagnetic wireless nanonetworks in the THz band,"
 *       IEEE Trans. Wireless Commun., vol. 10, no. 10, 2011.
 */

#ifndef THZ_NTN_MOLECULAR_ABSORPTION_H
#define THZ_NTN_MOLECULAR_ABSORPTION_H

#include "thz-ntn-hitran-lut.h"

#include <ns3/object.h>

#include <string>
#include <vector>

namespace ns3
{

/**
 * \ingroup thz-ntn
 * \brief HITRAN-based molecular absorption model for THz-NTN links
 *
 * This class computes the molecular absorption loss along arbitrary
 * ground-to-satellite, satellite-to-ground, or inter-satellite links in
 * the terahertz band.  The atmosphere is divided into six altitude layers
 * following the ITU-R P.835 standard atmosphere model, each characterised
 * by representative temperature, pressure, and water-vapour density values.
 *
 * For each layer the absorption coefficient is computed from a simplified
 * HITRAN-derived line database using the Van Vleck--Weisskopf line shape,
 * augmented by a power-law continuum term.  The total absorption is obtained
 * by integrating the per-layer specific attenuation along the slant path
 * from transmitter to receiver, accounting for Earth curvature.
 *
 * Inter-satellite links (both endpoints above 100 km) traverse vacuum and
 * therefore experience zero molecular absorption.
 */
class ThzNtnMolecularAbsorption : public Object
{
  public:
    /**
     * \brief Get the TypeId for this class.
     * \return the ns-3 TypeId
     */
    static TypeId GetTypeId();

    ThzNtnMolecularAbsorption();
    ~ThzNtnMolecularAbsorption() override;

    /**
     * \brief Compute total molecular absorption loss in dB along a link.
     *
     * This is the main entry point.  For ground-to-satellite or
     * satellite-to-ground links the method integrates the absorption
     * coefficient through the relevant atmospheric layers.  For
     * inter-satellite links (both altitudes > 100 km) the loss is zero.
     *
     * \param freqHz        carrier frequency in Hz
     * \param distanceM     3-D Euclidean distance between TX and RX in metres
     * \param altitudeTx_km altitude of the transmitter in km above sea level
     * \param altitudeRx_km altitude of the receiver in km above sea level
     * \param elevationDeg  elevation angle of the link in degrees (from the
     *                      lower endpoint's local horizontal)
     * \return absorption loss in dB (non-negative)
     */
    double ComputeAbsorptionLoss_dB(double freqHz,
                                    double distanceM,
                                    double altitudeTx_km,
                                    double altitudeRx_km,
                                    double elevationDeg) const;

    /**
     * \brief Compute the absorption coefficient at a single point.
     *
     * Returns the specific attenuation in Np/km for the given local
     * atmospheric conditions using the HITRAN line database and continuum
     * model.
     *
     * \param freqHz      carrier frequency in Hz
     * \param tempK       local temperature in kelvin
     * \param pressureHPa local pressure in hectopascals (mbar)
     * \param humidityPct water-vapour density in g/m^3
     * \return absorption coefficient in Np/km
     */
    double ComputeAbsorptionCoefficient(double freqHz,
                                        double tempK,
                                        double pressureHPa,
                                        double humidityPct) const;

    /**
     * \brief Integrate molecular absorption along a slant path through the
     *        atmosphere.
     *
     * The integration proceeds layer by layer from \p groundAlt_km up to
     * \p satAlt_km (or to the top of the atmosphere at 100 km, whichever
     * is lower).  Within each layer the effective elevation angle is
     * corrected for Earth curvature.
     *
     * \param freqHz       carrier frequency in Hz
     * \param elevationDeg elevation angle at the ground station in degrees
     * \param groundAlt_km altitude of the ground endpoint in km
     * \param satAlt_km    altitude of the space endpoint in km
     * \return total absorption loss in dB along the slant path
     */
    double ComputeSlantPathAbsorption(double freqHz,
                                      double elevationDeg,
                                      double groundAlt_km,
                                      double satAlt_km) const;

    /**
     * \brief Select the humidity profile.
     *
     * The profile controls the water-vapour density scaling applied to each
     * atmospheric layer.  Supported values are "tropical",
     * "midlatitude-summer" (default), "midlatitude-winter", and "subarctic".
     *
     * \param profile humidity profile name (case-sensitive)
     */
    void SetHumidityProfile(const std::string& profile);

    /**
     * \brief Compute the atmospheric transmittance (linear, 0--1).
     *
     * Convenience method that converts the absorption coefficient at a
     * single representative altitude into a transmittance value over a
     * given distance.
     *
     * \param freqHz         carrier frequency in Hz
     * \param distanceM      path length in metres
     * \param altitudeAvg_km representative altitude in km
     * \return transmittance in the range [0, 1]
     */
    double GetTransmittance(double freqHz,
                            double distanceM,
                            double altitudeAvg_km) const;

    /**
     * \brief Load a HITRAN-2024 specific-attenuation LUT (Roadmap §4.3.1).
     *
     * On success subsequent ComputeAbsorptionCoefficient() calls bilinear-
     * interpolate the LUT instead of summing Van Vleck–Weisskopf terms.
     * On failure (I/O / parse error) the model continues to use the
     * in-process line database so the simulation always makes progress.
     *
     * \param path  path to the CSV LUT produced by
     *              `contrib/thz-ntn/tools/hitran2024-lut-gen.py`
     * \return true iff the LUT was loaded successfully
     */
    bool LoadHitran2024Lut(const std::string& path);

    /// True iff a HITRAN-2024 LUT is loaded and being consulted by the
    /// absorption coefficient path.
    bool IsHitranLutLoaded() const { return m_lut.IsLoaded(); }

    /// Release tag this build advertises in the reproducibility manifest.
    /// Returns "HITRAN-2024" when a LUT is loaded with that tag, else "in-process".
    std::string GetHitranReleaseTag() const;

  protected:
    void DoDispose() override;

  private:
    /**
     * \brief Descriptor for one atmospheric layer.
     *
     * Each layer is characterised by its altitude bounds and representative
     * thermodynamic state from ITU-R P.835.
     */
    struct AtmosphericLayer
    {
        double altLow_km;      //!< lower boundary altitude in km
        double altHigh_km;     //!< upper boundary altitude in km
        double temperature_K;  //!< representative temperature in K
        double pressure_hPa;   //!< representative pressure in hPa
        double humidity_gm3;   //!< water-vapour density in g/m^3
    };

    /**
     * \brief Descriptor for a single spectral absorption line.
     */
    struct AbsorptionLine
    {
        double centerFreqHz;       //!< line centre frequency in Hz
        double lineIntensity;      //!< integrated line intensity in cm^{-1}/(mol cm^{-2})
        double halfWidthHz;        //!< pressure-broadened half-width at half-maximum in Hz
        bool   isWaterVapor;       //!< true for H2O, false for O2
    };

    /**
     * \brief Initialise the atmospheric layer table (ITU-R P.835).
     */
    void InitAtmosphericLayers();

    /**
     * \brief Initialise the HITRAN-derived absorption line database.
     */
    void InitAbsorptionLines();

    /**
     * \brief Van Vleck--Weisskopf line shape function.
     *
     * \param freqHz      evaluation frequency in Hz
     * \param centerHz    line centre frequency in Hz
     * \param halfWidthHz pressure-broadened half-width in Hz
     * \return line shape value in Hz^{-1}
     */
    double VanVleckWeisskopf(double freqHz,
                             double centerHz,
                             double halfWidthHz) const;

    /**
     * \brief Get the atmospheric conditions at a given altitude by
     *        interpolating or selecting the appropriate layer.
     *
     * \param altitude_km altitude in km
     * \param[out] tempK       temperature in K
     * \param[out] pressureHPa pressure in hPa
     * \param[out] humidity    water-vapour density in g/m^3
     */
    void GetAtmosphericConditions(double altitude_km,
                                  double& tempK,
                                  double& pressureHPa,
                                  double& humidity) const;

    /**
     * \brief Apply humidity profile scaling to the base layer humidity.
     *
     * \param baseHumidity base water-vapour density in g/m^3
     * \param altitude_km  altitude in km
     * \return scaled water-vapour density in g/m^3
     */
    double ApplyHumidityProfile(double baseHumidity, double altitude_km) const;

    // Configuration
    std::string m_humidityProfile;  //!< humidity profile name
    uint32_t m_integrationSteps;    //!< number of sub-steps per atmospheric layer

    // Pre-built tables
    std::vector<AtmosphericLayer> m_layers;  //!< atmospheric layer definitions
    std::vector<AbsorptionLine> m_lines;     //!< absorption line database

    // 4.3.1 — HITRAN-2024 LUT path. When loaded, the SLANT-PATH integration
    // (ComputeSlantPathAbsorption) and GetTransmittance consult `m_lut` for the
    // per-sub-layer specific attenuation (dB/km). The single-point
    // ComputeAbsorptionCoefficient remains the Van Vleck–Weisskopf kernel and is
    // used as the per-point fallback whenever the LUT returns a non-finite value
    // (or no LUT is loaded).
    thzntn::HitranLut m_lut;
};

} // namespace ns3

#endif // THZ_NTN_MOLECULAR_ABSORPTION_H
