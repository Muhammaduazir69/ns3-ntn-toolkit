/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-only
 * Author: Muhammad Uzair
 *
 * THz-NTN Spectrum Model -- Atmospheric Window Identification
 */

#include "thz-ntn-spectrum.h"

#include <ns3/log.h>

#include <algorithm>
#include <cmath>
#include <limits>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("ThzNtnSpectrum");
NS_OBJECT_ENSURE_REGISTERED(ThzNtnSpectrum);

TypeId
ThzNtnSpectrum::GetTypeId()
{
    static TypeId tid = TypeId("ns3::ThzNtnSpectrum")
                            .SetParent<Object>()
                            .SetGroupName("ThzNtn")
                            .AddConstructor<ThzNtnSpectrum>();
    return tid;
}

ThzNtnSpectrum::ThzNtnSpectrum()
{
    NS_LOG_FUNCTION(this);
    InitWindowDatabase();
}

ThzNtnSpectrum::~ThzNtnSpectrum()
{
    NS_LOG_FUNCTION(this);
}

void
ThzNtnSpectrum::DoDispose()
{
    NS_LOG_FUNCTION(this);
    m_windows.clear();
    Object::DoDispose();
}

void
ThzNtnSpectrum::InitWindowDatabase()
{
    NS_LOG_FUNCTION(this);
    m_windows = GetStandardWindows();
}

std::vector<ThzNtnSpectrum::AtmosphericWindow>
ThzNtnSpectrum::GetStandardWindows()
{
    // Five primary atmospheric transmission windows for satellite-ground THz
    // links.  Parameters derived from HITRAN radiative transfer simulations
    // under ITU-R P.835 midlatitude summer standard atmosphere at zenith.
    std::vector<AtmosphericWindow> windows;

    // Window 1: 140 GHz (D-band low)
    // Excellent transmittance, moderate bandwidth.  Mature technology baseline.
    windows.push_back({140.0, 20.0, 0.85, 1.2, true});

    // Window 2: 220 GHz (TeraLink candidate)
    // Good balance of bandwidth and transmittance.  Preferred for LEO feeder.
    windows.push_back({220.0, 30.0, 0.75, 2.5, true});

    // Window 3: 340 GHz
    // Moderate transmittance, useful for shorter LEO passes at high elevation.
    windows.push_back({340.0, 25.0, 0.55, 4.8, true});

    // Window 4: 410 GHz
    // Narrow usable bandwidth, suited for high-gain fixed ground terminals.
    windows.push_back({410.0, 15.0, 0.45, 6.5, true});

    // Window 5: 460 GHz
    // Marginal for satellite-ground; borderline for high-altitude platforms.
    windows.push_back({460.0, 10.0, 0.35, 8.2, true});

    return windows;
}

std::vector<ThzNtnSpectrum::AtmosphericWindow>
ThzNtnSpectrum::GetAvailableWindows(double minBandwidthGHz,
                                    double maxAttenuation_dB) const
{
    NS_LOG_FUNCTION(this << minBandwidthGHz << maxAttenuation_dB);

    std::vector<AtmosphericWindow> result;
    for (const auto& w : m_windows)
    {
        if (w.bandwidthGHz >= minBandwidthGHz &&
            w.maxZenithAttenuation_dB <= maxAttenuation_dB &&
            w.suitableForSatGround)
        {
            result.push_back(w);
        }
    }

    NS_LOG_INFO("Found " << result.size() << " windows with BW >= "
                          << minBandwidthGHz << " GHz and atten <= "
                          << maxAttenuation_dB << " dB");
    return result;
}

ThzNtnSpectrum::AtmosphericWindow
ThzNtnSpectrum::GetBestWindow(double requiredBandwidthGHz) const
{
    NS_LOG_FUNCTION(this << requiredBandwidthGHz);

    const AtmosphericWindow* best = nullptr;
    double bestAtten = std::numeric_limits<double>::max();

    // Find the lowest-attenuation window that meets the bandwidth requirement
    for (const auto& w : m_windows)
    {
        if (w.bandwidthGHz >= requiredBandwidthGHz &&
            w.suitableForSatGround &&
            w.maxZenithAttenuation_dB < bestAtten)
        {
            best = &w;
            bestAtten = w.maxZenithAttenuation_dB;
        }
    }

    // Fallback: return window with largest bandwidth
    if (!best)
    {
        NS_LOG_WARN("No window meets bandwidth requirement of "
                    << requiredBandwidthGHz << " GHz; using largest-bandwidth fallback");
        double maxBw = 0.0;
        for (const auto& w : m_windows)
        {
            if (w.bandwidthGHz > maxBw)
            {
                best = &w;
                maxBw = w.bandwidthGHz;
            }
        }
    }

    NS_ASSERT_MSG(best, "Window database is empty");
    NS_LOG_INFO("Best window: " << best->centerFreqGHz << " GHz, BW="
                                << best->bandwidthGHz << " GHz, atten="
                                << best->maxZenithAttenuation_dB << " dB");
    return *best;
}

double
ThzNtnSpectrum::GetWindowTransmittance(double freqGHz, double elevationDeg) const
{
    NS_LOG_FUNCTION(this << freqGHz << elevationDeg);

    // Clamp elevation to avoid singularity at horizon
    double elev = std::max(elevationDeg, 5.0);

    // Find the closest atmospheric window
    const AtmosphericWindow* win = FindClosestWindow(freqGHz);
    if (!win)
    {
        NS_LOG_WARN("No window data available; returning zero transmittance");
        return 0.0;
    }

    // Scale zenith attenuation by 1/sin(elevation) for slant path
    double elevRad = elev * M_PI / 180.0;
    double slantAtten_dB = win->maxZenithAttenuation_dB / std::sin(elevRad);

    // Compute frequency-dependent transmittance scaling
    // Transmittance rolls off away from window centre
    double freqOffset = std::abs(freqGHz - win->centerFreqGHz);
    double halfBw = win->bandwidthGHz / 2.0;
    double freqFactor = 1.0;
    if (freqOffset > halfBw)
    {
        // Outside the window: rapid transmittance drop-off
        double excess = (freqOffset - halfBw) / halfBw;
        freqFactor = std::exp(-2.0 * excess * excess);
    }

    // Convert slant attenuation to transmittance
    double transmittance = win->peakTransmittance * freqFactor *
                           std::pow(10.0, -slantAtten_dB / 10.0) /
                           std::pow(10.0, -win->maxZenithAttenuation_dB / 10.0);

    // Clamp to [0, 1]
    transmittance = std::max(0.0, std::min(1.0, transmittance));

    NS_LOG_DEBUG("Transmittance at " << freqGHz << " GHz, elev=" << elev
                                     << " deg: " << transmittance);
    return transmittance;
}

bool
ThzNtnSpectrum::IsInAtmosphericWindow(double freqGHz) const
{
    NS_LOG_FUNCTION(this << freqGHz);

    for (const auto& w : m_windows)
    {
        double halfBw = w.bandwidthGHz / 2.0;
        if (freqGHz >= (w.centerFreqGHz - halfBw) &&
            freqGHz <= (w.centerFreqGHz + halfBw))
        {
            return true;
        }
    }
    return false;
}

std::string
ThzNtnSpectrum::GetBandName(double freqGHz) const
{
    NS_LOG_FUNCTION(this << freqGHz);

    if (freqGHz >= 110.0 && freqGHz < 140.0)
    {
        return "D_BAND_LOW";
    }
    else if (freqGHz >= 140.0 && freqGHz < 170.0)
    {
        return "D_BAND_HIGH";
    }
    else if (freqGHz >= 170.0 && freqGHz < 220.0)
    {
        return "G_BAND";
    }
    else if (freqGHz >= 220.0 && freqGHz < 325.0)
    {
        return "H_BAND";
    }
    else if (freqGHz >= 100.0 && freqGHz < 300.0)
    {
        // Overlapping classification: general sub-THz
        return "SUB_THZ";
    }
    else if (freqGHz >= 300.0 && freqGHz < 500.0)
    {
        return "THZ_LOW";
    }
    else if (freqGHz >= 500.0 && freqGHz < 1000.0)
    {
        return "THZ_MID";
    }
    else if (freqGHz >= 1000.0 && freqGHz <= 10000.0)
    {
        return "THZ_HIGH";
    }
    else
    {
        return "ISL_ANY";
    }
}

const ThzNtnSpectrum::AtmosphericWindow*
ThzNtnSpectrum::FindClosestWindow(double freqGHz) const
{
    if (m_windows.empty())
    {
        return nullptr;
    }

    const AtmosphericWindow* closest = &m_windows[0];
    double minDist = std::abs(freqGHz - closest->centerFreqGHz);

    for (size_t i = 1; i < m_windows.size(); ++i)
    {
        double dist = std::abs(freqGHz - m_windows[i].centerFreqGHz);
        if (dist < minDist)
        {
            minDist = dist;
            closest = &m_windows[i];
        }
    }
    return closest;
}

} // namespace ns3
