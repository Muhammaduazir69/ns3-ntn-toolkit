/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-only
 * Author: Muhammad Uzair
 *
 * THz-NTN Spectrum Model -- Atmospheric Window Identification
 *
 * Identifies and characterises usable atmospheric transmission windows for
 * terahertz satellite-ground links in the 100 GHz -- 10 THz range.  The model
 * provides a database of five primary atmospheric windows whose parameters
 * are derived from HITRAN radiative transfer simulations under standard
 * midlatitude summer atmospheric conditions at zenith.
 *
 * Primary atmospheric windows for satellite-ground THz links:
 *   140 GHz (D-band low)  -- 20 GHz BW, 0.85 peak transmittance
 *   220 GHz (TeraLink)    -- 30 GHz BW, 0.75 peak transmittance
 *   340 GHz               -- 25 GHz BW, 0.55 peak transmittance
 *   410 GHz               -- 15 GHz BW, 0.45 peak transmittance
 *   460 GHz               -- 10 GHz BW, 0.35 peak transmittance
 *
 * Band classification follows ITU-R and IEEE conventions:
 *   D_BAND_LOW  (110--140 GHz), D_BAND_HIGH (140--170 GHz),
 *   G_BAND      (170--220 GHz), H_BAND      (220--325 GHz),
 *   SUB_THZ     (100--300 GHz), THZ_LOW     (300--500 GHz),
 *   THZ_MID     (500 GHz--1 THz), THZ_HIGH  (1--10 THz),
 *   ISL_ANY     (any frequency, vacuum inter-satellite link)
 *
 * References:
 *   [1] ITU-R P.676-13, "Attenuation by atmospheric gases"
 *   [2] HITRAN2020 molecular spectroscopic database
 *   [3] J. Federici and L. Moeller, "Review of terahertz and subterahertz
 *       wireless communications," J. Appl. Phys., vol. 107, 2010.
 */

#ifndef THZ_NTN_SPECTRUM_H
#define THZ_NTN_SPECTRUM_H

#include <ns3/object.h>

#include <string>
#include <vector>

namespace ns3
{

/**
 * \ingroup thz-ntn
 * \brief Atmospheric window identification and THz band classification for NTN links
 *
 * This utility class provides a database of atmospheric transmission windows
 * suitable for terahertz satellite-to-ground communications.  Each window is
 * characterised by its centre frequency, usable bandwidth, peak transmittance,
 * and maximum one-way zenith attenuation.
 *
 * The class also provides band classification utilities mapping arbitrary
 * frequencies to standard ITU-R / IEEE band designations.
 */
class ThzNtnSpectrum : public Object
{
  public:
    /**
     * \brief Get the TypeId for this class.
     * \return the ns-3 TypeId
     */
    static TypeId GetTypeId();

    ThzNtnSpectrum();
    ~ThzNtnSpectrum() override;

    /**
     * \brief THz band classification enumeration.
     */
    enum BandClass
    {
        D_BAND_LOW = 0,  ///< 110--140 GHz
        D_BAND_HIGH,     ///< 140--170 GHz
        G_BAND,          ///< 170--220 GHz
        H_BAND,          ///< 220--325 GHz
        SUB_THZ,         ///< 100--300 GHz (general sub-THz)
        THZ_LOW,         ///< 300--500 GHz
        THZ_MID,         ///< 500 GHz -- 1 THz
        THZ_HIGH,        ///< 1--10 THz
        ISL_ANY          ///< Any frequency (vacuum inter-satellite link)
    };

    /**
     * \brief Descriptor for one atmospheric transmission window.
     */
    struct AtmosphericWindow
    {
        double centerFreqGHz;          ///< centre frequency in GHz
        double bandwidthGHz;           ///< usable bandwidth in GHz
        double peakTransmittance;      ///< peak transmittance at zenith (0--1)
        double maxZenithAttenuation_dB; ///< maximum one-way zenith attenuation in dB
        bool suitableForSatGround;     ///< true if window is viable for satellite-ground links
    };

    /**
     * \brief Return the five standard atmospheric windows.
     *
     * This static method returns the primary transmission windows for
     * satellite-ground THz links, derived from HITRAN radiative transfer
     * simulations under midlatitude summer conditions.
     *
     * \return vector of five AtmosphericWindow descriptors
     */
    static std::vector<AtmosphericWindow> GetStandardWindows();

    /**
     * \brief Return windows that meet minimum bandwidth and maximum attenuation criteria.
     *
     * \param minBandwidthGHz  minimum required usable bandwidth in GHz
     * \param maxAttenuation_dB maximum acceptable zenith attenuation in dB
     * \return vector of qualifying AtmosphericWindow descriptors
     */
    std::vector<AtmosphericWindow> GetAvailableWindows(double minBandwidthGHz,
                                                       double maxAttenuation_dB) const;

    /**
     * \brief Return the lowest-attenuation window meeting the bandwidth requirement.
     *
     * If no window meets the requirement, the window with the largest bandwidth
     * is returned as a fallback.
     *
     * \param requiredBandwidthGHz minimum required bandwidth in GHz
     * \return the best AtmosphericWindow descriptor
     */
    AtmosphericWindow GetBestWindow(double requiredBandwidthGHz) const;

    /**
     * \brief Compute the transmittance at a given frequency and elevation angle.
     *
     * The transmittance is derived from the closest standard window's zenith
     * attenuation, scaled by 1/sin(elevation) for off-zenith paths.
     *
     * \param freqGHz     frequency in GHz
     * \param elevationDeg elevation angle in degrees (> 0)
     * \return linear transmittance in the range (0, 1]
     */
    double GetWindowTransmittance(double freqGHz, double elevationDeg) const;

    /**
     * \brief Check whether a frequency falls within a standard atmospheric window.
     *
     * A frequency is considered to be within a window if it lies within
     * +/- bandwidthGHz/2 of any standard window's centre frequency.
     *
     * \param freqGHz frequency in GHz
     * \return true if frequency is inside an atmospheric window
     */
    bool IsInAtmosphericWindow(double freqGHz) const;

    /**
     * \brief Return a human-readable band classification string for a frequency.
     *
     * \param freqGHz frequency in GHz
     * \return band name string (e.g. "D_BAND_LOW", "THZ_LOW")
     */
    std::string GetBandName(double freqGHz) const;

  protected:
    void DoDispose() override;

  private:
    /**
     * \brief Initialise the internal window database.
     */
    void InitWindowDatabase();

    /**
     * \brief Find the closest atmospheric window to a given frequency.
     *
     * \param freqGHz frequency in GHz
     * \return pointer to the closest AtmosphericWindow, or nullptr if database is empty
     */
    const AtmosphericWindow* FindClosestWindow(double freqGHz) const;

    std::vector<AtmosphericWindow> m_windows; ///< atmospheric window database
};

} // namespace ns3

#endif // THZ_NTN_SPECTRUM_H
