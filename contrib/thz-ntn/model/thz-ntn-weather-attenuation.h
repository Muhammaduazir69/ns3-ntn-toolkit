/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2024 Muhammad Uzair
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Author: Muhammad Uzair <uk5595985@gmail.com>
 */

#ifndef THZ_NTN_WEATHER_ATTENUATION_H
#define THZ_NTN_WEATHER_ATTENUATION_H

#include <ns3/object.h>

#include <string>
#include <vector>

namespace ns3
{

/**
 * \ingroup thz-ntn
 *
 * \brief Weather-induced attenuation model for THz Non-Terrestrial Network (NTN) links.
 *
 * This class computes weather-related propagation losses for satellite-to-ground
 * THz links. It implements models for rain, fog/cloud, snow/ice, and sand/dust
 * attenuation, extending ITU-R recommendations (P.838, P.840, P.530, P.839)
 * to terahertz frequencies up to 1 THz.
 *
 * At THz frequencies (0.1--1 THz), the wavelengths (0.3--3 mm) become comparable
 * to raindrop diameters (~1 mm), causing Mie scattering to dominate over the
 * Rayleigh regime assumed in lower-frequency models. The fog/cloud model uses
 * the Debye relaxation model for liquid water permittivity, with K_l scaling
 * approximately as f^2 up to ~300 GHz before plateauing.
 *
 * \section weather_rain Rain Attenuation (ITU-R P.838 Extended)
 *
 * Specific attenuation is computed as:
 * \f[
 *   \gamma_R = k \cdot R^{\alpha} \quad \text{(dB/km)}
 * \f]
 * where \f$k\f$ and \f$\alpha\f$ are frequency-dependent coefficients
 * interpolated from a lookup table spanning 1--1000 GHz.
 *
 * The effective slant path length through the rain cell follows ITU-R P.530:
 * \f[
 *   L_s = \frac{h_R - h_S}{\sin(\theta)}
 * \f]
 * where \f$h_R\f$ is the rain height (ITU-R P.839), \f$h_S\f$ is the station
 * height, and \f$\theta\f$ is the elevation angle.
 *
 * \section weather_fog Fog/Cloud Attenuation (ITU-R P.840 Extended)
 *
 * \f[
 *   \gamma_c = K_l \cdot M \quad \text{(dB/km)}
 * \f]
 * where \f$K_l\f$ is the liquid water specific attenuation coefficient computed
 * from the Debye relaxation model, and \f$M\f$ is the liquid water content
 * (g/m^3).
 *
 * \section weather_snow Snow/Ice Attenuation
 *
 * Dry snow: \f$\gamma_{snow,dry} \propto f^{1.6} \cdot S^{0.72}\f$
 * Wet snow: \f$\gamma_{snow,wet} \propto f^{2} \cdot S\f$
 *
 * \section weather_dust Sand/Dust Storm Attenuation
 *
 * Visibility-based model:
 * \f[
 *   \gamma_{dust} = \frac{(f / f_{ref})^{\beta}}{V_{km}}
 * \f]
 */
class ThzNtnWeatherAttenuation : public Object
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
    ThzNtnWeatherAttenuation();

    /**
     * \brief Destructor.
     */
    ~ThzNtnWeatherAttenuation() override;

    /**
     * \brief Climate region enumeration for rain height estimation (ITU-R P.839).
     */
    enum ClimateRegion
    {
        TROPICAL = 0,       ///< Tropical region (rain height ~5.0 km)
        MIDLAT_SUMMER = 1,  ///< Mid-latitude summer (rain height ~3.5 km)
        MIDLAT_WINTER = 2   ///< Mid-latitude winter (rain height ~2.0 km)
    };

    // --- Rain attenuation ---

    /**
     * \brief Compute rain attenuation along the slant path.
     *
     * Uses ITU-R P.838 extended coefficients for THz and the effective
     * slant path length through the rain cell.
     *
     * \param freqHz      operating frequency in Hz.
     * \param elevationDeg elevation angle in degrees (0--90).
     * \param rainRate_mm_h rain rate in mm/h.
     * \return rain attenuation in dB.
     */
    double ComputeRainAttenuation_dB(double freqHz,
                                     double elevationDeg,
                                     double rainRate_mm_h) const;

    // --- Fog/cloud attenuation ---

    /**
     * \brief Compute fog/cloud attenuation along the slant path.
     *
     * Uses the Debye relaxation model for liquid water permittivity
     * to obtain the specific attenuation coefficient K_l.
     *
     * \param freqHz      operating frequency in Hz.
     * \param elevationDeg elevation angle in degrees.
     * \param lwc_g_m3    liquid water content in g/m^3.
     * \return fog/cloud attenuation in dB.
     */
    double ComputeFogAttenuation_dB(double freqHz,
                                    double elevationDeg,
                                    double lwc_g_m3) const;

    // --- Snow/ice attenuation ---

    /**
     * \brief Compute snow/ice attenuation along the slant path.
     *
     * \param freqHz       operating frequency in Hz.
     * \param elevationDeg elevation angle in degrees.
     * \param snowRate_mm_h snow rate in mm/h (liquid-water equivalent).
     * \param isWet        true for wet snow, false for dry snow.
     * \return snow attenuation in dB.
     */
    double ComputeSnowAttenuation_dB(double freqHz,
                                     double elevationDeg,
                                     double snowRate_mm_h,
                                     bool isWet) const;

    // --- Sand/dust storm attenuation ---

    /**
     * \brief Compute sand/dust storm attenuation along the slant path.
     *
     * Uses a visibility-based model appropriate for desert and maritime
     * environments.
     *
     * \param freqHz       operating frequency in Hz.
     * \param elevationDeg elevation angle in degrees.
     * \param visibility_km visibility in km.
     * \return dust attenuation in dB.
     */
    double ComputeDustAttenuation_dB(double freqHz,
                                     double elevationDeg,
                                     double visibility_km) const;

    // --- Combined weather loss ---

    /**
     * \brief Compute total weather-induced attenuation combining all active
     *        weather effects.
     *
     * Only effects that are enabled (via their respective Enable flags) and
     * have non-zero parameters contribute.
     *
     * \param freqHz       operating frequency in Hz.
     * \param elevationDeg elevation angle in degrees.
     * \return total weather attenuation in dB.
     */
    double ComputeTotalWeatherLoss_dB(double freqHz,
                                      double elevationDeg) const;

    // --- Setters ---

    /**
     * \brief Set the rain rate.
     * \param rainRate_mm_h rain rate in mm/h.
     */
    void SetRainRate(double rainRate_mm_h);

    /**
     * \brief Set the liquid water content (fog/cloud density).
     * \param lwc_g_m3 liquid water content in g/m^3.
     */
    void SetLiquidWaterContent(double lwc_g_m3);

    /**
     * \brief Set the snow rate.
     * \param snowRate_mm_h snow rate in mm/h (liquid-water equivalent).
     */
    void SetSnowRate(double snowRate_mm_h);

    /**
     * \brief Set whether snow is wet or dry.
     * \param isWet true for wet snow.
     */
    void SetWetSnow(bool isWet);

    /**
     * \brief Set the dust storm visibility.
     * \param visibility_km visibility in km.
     */
    void SetDustVisibility(double visibility_km);

    /**
     * \brief Enable or disable rain attenuation.
     * \param enable true to enable.
     */
    void SetEnableRain(bool enable);

    /**
     * \brief Enable or disable fog/cloud attenuation.
     * \param enable true to enable.
     */
    void SetEnableFog(bool enable);

    /**
     * \brief Enable or disable snow attenuation.
     * \param enable true to enable.
     */
    void SetEnableSnow(bool enable);

    /**
     * \brief Enable or disable dust attenuation.
     * \param enable true to enable.
     */
    void SetEnableDust(bool enable);

    /**
     * \brief Set the climate region for rain height estimation.
     * \param region climate region enumeration value.
     */
    void SetClimateRegion(ClimateRegion region);

    /**
     * \brief Set the ground station height above mean sea level.
     * \param height_km station height in km.
     */
    void SetStationHeight(double height_km);

    // --- Getters ---

    /**
     * \brief Get the current rain rate.
     * \return rain rate in mm/h.
     */
    double GetRainRate() const;

    /**
     * \brief Get the current liquid water content.
     * \return LWC in g/m^3.
     */
    double GetLiquidWaterContent() const;

    /**
     * \brief Get the current snow rate.
     * \return snow rate in mm/h.
     */
    double GetSnowRate() const;

    /**
     * \brief Get the current dust storm visibility.
     * \return visibility in km.
     */
    double GetDustVisibility() const;

  private:
    /**
     * \brief Structure holding ITU-R P.838 rain coefficients at a given frequency.
     */
    struct RainCoefficients
    {
        double freqGHz; ///< frequency in GHz
        double kH;      ///< k coefficient for horizontal polarization
        double kV;      ///< k coefficient for vertical polarization
        double alphaH;  ///< alpha exponent for horizontal polarization
        double alphaV;  ///< alpha exponent for vertical polarization
    };

    /**
     * \brief Initialize the rain coefficient lookup table.
     */
    void InitRainCoefficients();

    /**
     * \brief Interpolate rain k and alpha coefficients for a given frequency.
     *
     * \param freqGHz frequency in GHz.
     * \param[out] k   interpolated k coefficient.
     * \param[out] alpha interpolated alpha exponent.
     */
    void InterpolateRainCoefficients(double freqGHz,
                                     double& k,
                                     double& alpha) const;

    /**
     * \brief Compute the rain height based on climate region (ITU-R P.839).
     * \return rain height in km above mean sea level.
     */
    double GetRainHeight() const;

    /**
     * \brief Compute the effective slant path length through the rain layer.
     *
     * \param elevationDeg elevation angle in degrees.
     * \param rainHeight_km rain height in km.
     * \return effective path length in km.
     */
    double ComputeRainSlantPath(double elevationDeg,
                                double rainHeight_km) const;

    /**
     * \brief Compute K_l (specific attenuation coefficient for liquid water)
     *        using the Debye relaxation model.
     *
     * \param freqGHz    frequency in GHz.
     * \param temperature_K temperature in Kelvin.
     * \return K_l in (dB/km)/(g/m^3).
     */
    double ComputeKl(double freqGHz, double temperature_K) const;

    /**
     * \brief Compute the effective fog/cloud slant path length.
     *
     * \param elevationDeg elevation angle in degrees.
     * \return slant path length through fog/cloud layer in km.
     */
    double ComputeFogSlantPath(double elevationDeg) const;

    /**
     * \brief Compute the effective snow layer slant path length.
     *
     * \param elevationDeg elevation angle in degrees.
     * \return slant path length through snow layer in km.
     */
    double ComputeSnowSlantPath(double elevationDeg) const;

    /**
     * \brief Compute the effective dust layer slant path length.
     *
     * \param elevationDeg elevation angle in degrees.
     * \return slant path length through dust layer in km.
     */
    double ComputeDustSlantPath(double elevationDeg) const;

    // --- Member variables ---
    double m_rainRate;             ///< rain rate in mm/h
    double m_liquidWaterContent;   ///< liquid water content in g/m^3
    double m_snowRate;             ///< snow rate in mm/h (liquid-water equivalent)
    bool m_wetSnow;                ///< true if wet snow
    double m_dustVisibility;       ///< dust storm visibility in km
    bool m_enableRain;             ///< flag to enable rain attenuation
    bool m_enableFog;              ///< flag to enable fog/cloud attenuation
    bool m_enableSnow;             ///< flag to enable snow attenuation
    bool m_enableDust;             ///< flag to enable dust attenuation
    ClimateRegion m_climateRegion; ///< climate region for rain height
    double m_stationHeight;        ///< ground station height in km ASL
    double m_temperature;          ///< ambient temperature in Kelvin
    double m_fogLayerHeight;       ///< fog/cloud layer thickness in km
    double m_snowLayerHeight;      ///< snow layer thickness in km
    double m_dustLayerHeight;      ///< dust layer thickness in km

    std::vector<RainCoefficients> m_rainCoeffTable; ///< ITU-R P.838 coefficient table
};

} // namespace ns3

#endif /* THZ_NTN_WEATHER_ATTENUATION_H */
