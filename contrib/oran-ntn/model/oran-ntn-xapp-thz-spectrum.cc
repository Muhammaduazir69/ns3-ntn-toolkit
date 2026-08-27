/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-only
 * Author: Muhammad Uzair
 *
 * THz Spectrum Management xApp - Implementation
 *
 * Monitors atmospheric conditions and molecular absorption to dynamically
 * select optimal THz atmospheric windows. Implements distance-aware
 * multi-carrier allocation and coordinates spectrum across the
 * constellation via A1 policies.
 */

#include "oran-ntn-xapp-thz-spectrum.h"

#include <ns3/double.h>
#include <ns3/log.h>
#include <ns3/simulator.h>

#include <algorithm>
#include <cmath>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("OranNtnXappThzSpectrum");
NS_OBJECT_ENSURE_REGISTERED(OranNtnXappThzSpectrum);

TypeId
OranNtnXappThzSpectrum::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::OranNtnXappThzSpectrum")
            .SetParent<OranNtnXappBase>()
            .SetGroupName("OranNtn")
            .AddConstructor<OranNtnXappThzSpectrum>()
            .AddAttribute("AbsorptionThreshold",
                          "Molecular absorption threshold (dB) to trigger window hop",
                          DoubleValue(20.0),
                          MakeDoubleAccessor(&OranNtnXappThzSpectrum::m_absorptionThreshold_dB),
                          MakeDoubleChecker<double>(0.0))
            .AddAttribute("WeatherLossThreshold",
                          "Weather-induced loss threshold (dB)",
                          DoubleValue(10.0),
                          MakeDoubleAccessor(&OranNtnXappThzSpectrum::m_weatherLossThreshold_dB),
                          MakeDoubleChecker<double>(0.0))
            .AddAttribute("MinElevation",
                          "Minimum satellite elevation angle for THz operation (degrees)",
                          DoubleValue(20.0),
                          MakeDoubleAccessor(&OranNtnXappThzSpectrum::m_minElevation_deg),
                          MakeDoubleChecker<double>(0.0, 90.0));
    return tid;
}

OranNtnXappThzSpectrum::OranNtnXappThzSpectrum()
    : m_absorptionThreshold_dB(20.0),
      m_weatherLossThreshold_dB(10.0),
      m_minElevation_deg(20.0)
{
    NS_LOG_FUNCTION(this);
    SetXappName("ThzSpectrum");

    // Define standard THz atmospheric transmission windows
    // These are frequency bands with relatively low molecular absorption
    m_atmosphericWindows_GHz = {
        0.3,   // 300 GHz window
        0.35,  // 350 GHz window
        0.41,  // 410 GHz window
        0.67,  // 670 GHz window
        0.85,  // 850 GHz window
        1.0,   // 1.0 THz window
    };

    m_metrics = {};
}

OranNtnXappThzSpectrum::~OranNtnXappThzSpectrum()
{
    NS_LOG_FUNCTION(this);
}

// --------------------------------------------------------------------------
//  E2 subscription
// --------------------------------------------------------------------------

E2Subscription
OranNtnXappThzSpectrum::GetRequiredSubscription() const
{
    E2Subscription sub;
    sub.subscriptionId = 0;
    sub.ricRequestorId = GetXappId();
    sub.ranFunctionId = 2; // KPM service model
    sub.reportingPeriod = MilliSeconds(200);
    sub.eventTrigger = false;
    sub.eventThreshold = 0.0;
    // ORAN-05: left at the default (true) so a feeder outage buffers on board
    // instead of silently discarding this xApp's telemetry.
    sub.maxBufferAge = Seconds(5);
    sub.useIslRelay = true;
    return sub;
}

// --------------------------------------------------------------------------
//  ProcessKpmReport
// --------------------------------------------------------------------------

void
OranNtnXappThzSpectrum::ProcessKpmReport(const E2KpmReport& report)
{
    NS_LOG_FUNCTION(this << report.gnbId);

    uint32_t gnbId = report.gnbId;

    ThzSpectrumState& state = m_spectrumStates[gnbId];
    state.molecularAbsorption_dB = report.molecularAbsorption_dB;
    state.atmosphericWindow_GHz = report.atmosphericWindow_GHz;
    state.thzCenterFreq_GHz = report.thzCenterFreq_GHz;
    state.elevation_deg = report.elevation_deg;
    state.propagationDelay_ms = report.propagationDelay_ms;

    NS_LOG_DEBUG("ThzSpectrum: gnb=" << gnbId
                 << " absorption=" << state.molecularAbsorption_dB << "dB"
                 << " window=" << state.atmosphericWindow_GHz << "GHz"
                 << " elev=" << state.elevation_deg << "deg");
}

// --------------------------------------------------------------------------
//  DecisionCycle
// --------------------------------------------------------------------------

void
OranNtnXappThzSpectrum::DecisionCycle()
{
    NS_LOG_FUNCTION(this);

    m_pendingActions.clear();

    double sumAbsorption = 0.0;
    double sumWindow = 0.0;
    uint32_t count = 0;

    for (auto& kv : m_spectrumStates)
    {
        uint32_t gnbId = kv.first;
        const ThzSpectrumState& state = kv.second;

        sumAbsorption += state.molecularAbsorption_dB;
        sumWindow += state.atmosphericWindow_GHz;
        count++;

        // Skip if elevation is too low for THz operation
        if (state.elevation_deg < m_minElevation_deg)
        {
            NS_LOG_DEBUG("ThzSpectrum: gnb" << gnbId
                         << " elevation " << state.elevation_deg
                         << "deg below minimum, skipping");
            continue;
        }

        // Check 1: Molecular absorption exceeds threshold -- hop to better window
        if (state.molecularAbsorption_dB > m_absorptionThreshold_dB)
        {
            double bestWindow = SelectBestAtmosphericWindow(
                state.molecularAbsorption_dB, state.elevation_deg);

            if (std::abs(bestWindow - state.atmosphericWindow_GHz) > 0.01)
            {
                NS_LOG_INFO("ThzSpectrum: gnb" << gnbId
                            << " absorption=" << state.molecularAbsorption_dB
                            << "dB, hopping from " << state.atmosphericWindow_GHz
                            << "GHz to " << bestWindow << "GHz");

                E2RcAction hopAction = BuildAction(
                    E2RcActionType::ACTION_THZ_WINDOW_HOP,
                    gnbId, 0, 0.85);
                hopAction.parameter1 = bestWindow;
                hopAction.parameter2 = state.molecularAbsorption_dB;

                m_pendingActions.push_back(hopAction);
                m_metrics.windowHops++;
            }
        }

        // Check 2: Distance-aware frequency selection
        // Convert propagation delay to approximate slant distance
        double distance_km = state.propagationDelay_ms * 3e5 / 1000.0;
        double optimalFreq = ComputeOptimalCarrierFreq(distance_km,
                                                       state.elevation_deg);

        if (std::abs(optimalFreq - state.thzCenterFreq_GHz) > 0.05)
        {
            NS_LOG_INFO("ThzSpectrum: gnb" << gnbId
                        << " optimal freq=" << optimalFreq
                        << "GHz vs current=" << state.thzCenterFreq_GHz << "GHz");

            E2RcAction freqAction = BuildAction(
                E2RcActionType::ACTION_THZ_FREQ_SELECT,
                gnbId, 0, 0.80);
            freqAction.parameter1 = optimalFreq;
            freqAction.parameter2 = distance_km;

            m_pendingActions.push_back(freqAction);
            m_metrics.freqSelections++;
        }
    }

    // Update metrics
    if (count > 0)
    {
        m_metrics.avgAbsorption_dB = sumAbsorption / count;
        m_metrics.avgAtmosphericWindow_GHz = sumWindow / count;
    }

    // Submit all pending actions
    for (auto& action : m_pendingActions)
    {
        SubmitAction(action);
    }

    RecordDecision(true, 0.82, 0.0);
}

// --------------------------------------------------------------------------
//  GenerateRcActions
// --------------------------------------------------------------------------

std::vector<E2RcAction>
OranNtnXappThzSpectrum::GenerateRcActions()
{
    return m_pendingActions;
}

// --------------------------------------------------------------------------
//  SelectBestAtmosphericWindow
// --------------------------------------------------------------------------

double
OranNtnXappThzSpectrum::SelectBestAtmosphericWindow(double currentAbsorption_dB,
                                                     double elevation_deg) const
{
    NS_LOG_FUNCTION(this << currentAbsorption_dB << elevation_deg);

    // For higher elevation angles, higher-frequency windows become viable
    // due to shorter atmospheric path length (proportional to 1/sin(elevation))
    double atmosphericPathFactor = 1.0;
    if (elevation_deg > 0.0)
    {
        atmosphericPathFactor = 1.0 / std::sin(elevation_deg * M_PI / 180.0);
    }

    // Select the window with the lowest expected absorption
    // Lower frequency windows generally have lower absorption
    // but higher frequency windows offer more bandwidth
    double bestWindow = m_atmosphericWindows_GHz.front();

    if (elevation_deg > 60.0)
    {
        // High elevation: can use higher frequency windows (more bandwidth)
        bestWindow = m_atmosphericWindows_GHz.back();
        for (auto it = m_atmosphericWindows_GHz.rbegin();
             it != m_atmosphericWindows_GHz.rend(); ++it)
        {
            // Rough absorption model: absorption scales with frequency^2 and path
            double estAbsorption = (*it) * (*it) * 0.1 * atmosphericPathFactor;
            if (estAbsorption < m_absorptionThreshold_dB * 0.8)
            {
                bestWindow = *it;
                break;
            }
        }
    }
    else
    {
        // Low elevation: use lower frequency windows for reliability
        for (const auto& window : m_atmosphericWindows_GHz)
        {
            double estAbsorption = window * window * 0.1 * atmosphericPathFactor;
            if (estAbsorption < m_absorptionThreshold_dB * 0.8)
            {
                bestWindow = window;
                break;
            }
        }
    }

    NS_LOG_DEBUG("ThzSpectrum: best window=" << bestWindow
                 << "GHz for absorption=" << currentAbsorption_dB
                 << "dB, elevation=" << elevation_deg << "deg");

    return bestWindow;
}

// --------------------------------------------------------------------------
//  ComputeOptimalCarrierFreq
// --------------------------------------------------------------------------

double
OranNtnXappThzSpectrum::ComputeOptimalCarrierFreq(double distance_km,
                                                   double elevation_deg) const
{
    NS_LOG_FUNCTION(this << distance_km << elevation_deg);

    // Distance-aware carrier selection:
    // - Short distance (< 500 km, high elevation): higher THz frequencies viable
    // - Long distance (> 1000 km, low elevation): use lower THz frequencies
    // This accounts for free-space path loss scaling with frequency^2

    if (distance_km < 500.0 && elevation_deg > 45.0)
    {
        return 0.85; // 850 GHz -- high bandwidth, short path
    }
    else if (distance_km < 800.0 && elevation_deg > 30.0)
    {
        return 0.67; // 670 GHz -- medium
    }
    else if (distance_km < 1200.0)
    {
        return 0.41; // 410 GHz -- balanced
    }
    else
    {
        return 0.3; // 300 GHz -- lowest absorption, longest range
    }
}

// --------------------------------------------------------------------------
//  GetThzSpectrumMetrics
// --------------------------------------------------------------------------

OranNtnXappThzSpectrum::ThzSpectrumMetrics
OranNtnXappThzSpectrum::GetThzSpectrumMetrics() const
{
    return m_metrics;
}

} // namespace ns3
