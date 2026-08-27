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
 * absorption along slant paths through the ITU-R P.835 reference atmosphere
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
 * the terahertz band.  The path is divided into quadrature slabs, and the
 * temperature, pressure and water-vapour density at every quadrature point
 * come from the ITU-R P.835-6 Section 1.1 mean annual global reference
 * atmosphere, evaluated in closed form (see GetAtmosphericConditions).
 *
 * At each quadrature point the absorption coefficient is computed from the full
 * ITU-R P.676-13 Annex 1 line-by-line model (44 oxygen + 35 water-vapour
 * lines with the standard line shape and the Debye/dry continuum).  The
 * total absorption is obtained
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
    /**
     * \brief State of the ITU-R P.835 reference atmosphere at an altitude.
     *
     * THZ-01. Evaluates Recommendation ITU-R P.835-6 Section 1.1, the mean
     * annual global reference atmosphere, in closed form: the seven
     * piecewise-linear temperature segments, the hydrostatic pressure solution
     * on those same segments, and the 2 km scale-height water-vapour
     * exponential with the 2e-6 mixing-ratio floor. Heights are converted to
     * geopotential first, as the recommendation requires.
     *
     * Public because it IS the standards claim this module makes. It was
     * private, so nothing could check that the profile matched the
     * recommendation the header named, and for a long time it did not: the
     * implementation was six isothermal layers whose pressure and humidity
     * restarted from each layer's base value, giving a 13.5 percent pressure
     * step at 2 km, an eight-fold humidity step at 10 km, no lapse rate
     * anywhere, and a water-vapour column of 17.55 kg/m2 against the reference
     * 15.0.
     *
     * \param altitude_km  geometric altitude above mean sea level, km
     * \param[out] tempK       temperature in K
     * \param[out] pressureHPa total pressure in hPa
     * \param[out] humidity    water-vapour density in g/m^3, after any
     *                          configured humidity-profile scaling
     */
    void GetAtmosphericConditions(double altitude_km,
                                  double& tempK,
                                  double& pressureHPa,
                                  double& humidity) const;

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
     * atmospheric conditions using the ITU-R P.676-13 Annex 1 line-by-line
     * model (oxygen + water-vapour lines plus the dry continuum).
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
    /// THZ-11: the elevation below which the slant integrator substitutes a
    /// different geometry than the one asked for.
    ///
    /// The layer integrator divides by sin(theta) and goes singular at the
    /// horizon, so it floors the elevation. That is a defensible numerical
    /// guard; doing it silently is not. A caller requesting 2 degrees was
    /// answered with the attenuation at 5 degrees and had no way to tell.
    static constexpr double kMinIntegratorElevationDeg = 5.0;

    /// True if the most recent ComputeSlantPathAbsorption() call had its
    /// elevation floored, i.e. the returned attenuation is for
    /// kMinIntegratorElevationDeg and not for the angle requested.
    bool WasLastElevationClamped() const { return m_lastElevationClamped; }
    /// The elevation actually integrated on the most recent call, in degrees.
    double GetLastIntegratedElevationDeg() const { return m_lastIntegratedElevDeg; }

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
     * \brief Load a gridded specific-attenuation LUT (Roadmap §4.3.1).
     *
     * On success the slant-path integration and GetTransmittance() bilinear-
     * interpolate the LUT (per sub-layer) instead of evaluating the P.676-13
     * line sum, falling back to the in-process kernel wherever the LUT
     * returns a non-finite value (out of grid). On failure (I/O / parse
     * error) the model continues to use the in-process P.676-13 model so the
     * simulation always makes progress.
     *
     * The bundled LUT is generated by
     * `contrib/thz-ntn/tools/p676-lut-gen.py`, an exact mirror of the
     * in-process ITU-R P.676-13 model. The method name retains the
     * "Hitran2024" spelling only for API compatibility.
     *
     * \param path  path to the CSV LUT
     * \return true iff the LUT was loaded successfully
     */
    bool LoadHitran2024Lut(const std::string& path);

    /// True iff a gridded LUT is loaded and being consulted by the slant-path
    /// integration.
    bool IsHitranLutLoaded() const { return m_lut.IsLoaded(); }

    /// Release tag this build advertises in the reproducibility manifest.
    /// Returns the loaded LUT's tag ("ITU-R-P.676-13" for the bundled table),
    /// else "in-process".
    std::string GetHitranReleaseTag() const;

    // ---- THZ-12: the "Hitran" names above are LEGACY, and no HITRAN data is
    // used anywhere in this module. -------------------------------------------
    //
    // The bundled table `data/hitran2024-lut-subthz.csv` opens with
    // `# release: ITU-R-P.676-13`, `# generated by tools/p676-lut-gen.py` and
    // `# model: ITU-R P.676-13 Annex 1 line-by-line`. It is generated from the
    // ITU recommendation's own line catalogue, not from a HITRAN .par file, and
    // no external HITRAN download is needed or consulted. The module README
    // says so, and a unit test asserts the tag is `ITU-R-P.676-13` with the
    // message "not HITRAN".
    //
    // The symbol names are kept so existing callers and any external user of
    // this header keep compiling; renaming 112 occurrences across a module to
    // fix a label would be churn with a real breakage risk and no functional
    // gain. What was missing is that nothing said so AT THE API, where a reader
    // forms the belief. These aliases are the names to use in new code, and the
    // loader is honest about what it accepts: any tag the file declares.
    bool LoadAtmosphericLut(const std::string& path) { return LoadHitran2024Lut(path); }
    bool IsAtmosphericLutLoaded() const { return IsHitranLutLoaded(); }
    /// The release tag DECLARED BY THE LOADED FILE. It is data, not a claim
    /// this module makes: a caller who loads a table tagged "HITRAN-2024" gets
    /// that string back, which is exactly what the loader round-trip tests do.
    std::string GetLutReleaseTag() const { return GetHitranReleaseTag(); }

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
     * \brief Descriptor for a single ITU-R P.676-13 Annex 1 spectral line.
     *
     * Holds the six spectroscopic coefficients tabulated in P.676-13
     * Table 1 (oxygen, a1..a6) or Table 2 (water vapour, b1..b6).  These
     * are the authoritative reference values; \p f0_GHz is the line centre.
     */
    struct AbsorptionLine
    {
        double f0_GHz;             //!< line centre frequency in GHz
        double c1;                 //!< P.676 a1 (O2) / b1 (H2O)
        double c2;                 //!< P.676 a2 (O2) / b2 (H2O)
        double c3;                 //!< P.676 a3 (O2) / b3 (H2O)
        double c4;                 //!< P.676 a4 (O2) / b4 (H2O)
        double c5;                 //!< P.676 a5 (O2) / b5 (H2O)
        double c6;                 //!< P.676 a6 (O2) / b6 (H2O)
        bool   isWaterVapor;       //!< true for H2O (Table 2), false for O2 (Table 1)
    };

    /**
     * \brief Initialise the integration slab boundaries.
     *
     * THZ-01: these are quadrature boundaries only. The thermodynamic state at
     * any altitude comes from GetAtmosphericConditions, which evaluates
     * ITU-R P.835-6 Section 1.1 in closed form.
     */
    void InitAtmosphericLayers();

    /**
     * \brief Initialise the ITU-R P.676-13 Annex 1 spectral line database.
     */
    void InitAbsorptionLines();



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
    /// THZ-11 bookkeeping: mutable because the compute path is const.
    mutable bool m_lastElevationClamped{false};
    mutable double m_lastIntegratedElevDeg{0.0};

    // (ComputeSlantPathAbsorption) and GetTransmittance consult `m_lut` for the
    // per-sub-layer specific attenuation (dB/km). The single-point
    // ComputeAbsorptionCoefficient remains the Van Vleck–Weisskopf kernel and is
    // used as the per-point fallback whenever the LUT returns a non-finite value
    // (or no LUT is loaded).
    thzntn::HitranLut m_lut;
};

} // namespace ns3

#endif // THZ_NTN_MOLECULAR_ABSORPTION_H
