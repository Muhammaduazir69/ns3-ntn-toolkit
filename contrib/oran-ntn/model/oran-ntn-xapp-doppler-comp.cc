/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-only
 * Author: Muhammad Uzair
 */

#include "oran-ntn-xapp-doppler-comp.h"

#include "oran-ntn-e2-interface.h"

#include <ns3/double.h>
#include <ns3/log.h>
#include <ns3/simulator.h>
#include <ns3/string.h>
#include <ns3/uinteger.h>

#include <algorithm>
#include <cmath>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("OranNtnXappDopplerComp");
NS_OBJECT_ENSURE_REGISTERED(OranNtnXappDopplerComp);

TypeId
OranNtnXappDopplerComp::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::OranNtnXappDopplerComp")
            .SetParent<OranNtnXappBase>()
            .SetGroupName("OranNtn")
            .AddConstructor<OranNtnXappDopplerComp>()
            .AddAttribute("CarrierFrequency",
                          "Carrier frequency in Hz for Doppler computation",
                          DoubleValue(2e9),
                          MakeDoubleAccessor(&OranNtnXappDopplerComp::m_carrierFrequency),
                          MakeDoubleChecker<double>(100e6, 100e9))
            .AddAttribute("CompensationMode",
                          "Doppler compensation strategy: common, per-ue, hybrid",
                          StringValue("hybrid"),
                          MakeStringAccessor(&OranNtnXappDopplerComp::m_compensationMode),
                          MakeStringChecker())
            .AddAttribute("UpdateInterval",
                          "Minimum interval between compensation updates",
                          TimeValue(MilliSeconds(200)),
                          MakeTimeAccessor(&OranNtnXappDopplerComp::m_updateInterval),
                          MakeTimeChecker())
            .AddAttribute("MaxResidualDoppler",
                          "Maximum tolerable residual Doppler shift in Hz",
                          DoubleValue(500.0),
                          MakeDoubleAccessor(&OranNtnXappDopplerComp::m_maxResidualDoppler),
                          MakeDoubleChecker<double>(0.0))
            .AddTraceSource("DopplerUpdated",
                            "Doppler compensation update issued",
                            MakeTraceSourceAccessor(
                                &OranNtnXappDopplerComp::m_dopplerUpdated),
                            "ns3::OranNtnXappDopplerComp::DopplerTracedCallback");
    return tid;
}

OranNtnXappDopplerComp::OranNtnXappDopplerComp()
    : m_carrierFrequency(2e9),
      m_compensationMode("hybrid"),
      m_updateInterval(MilliSeconds(200)),
      m_maxResidualDoppler(500.0)
{
    NS_LOG_FUNCTION(this);
    m_dopplerMetrics = {};
}

OranNtnXappDopplerComp::~OranNtnXappDopplerComp()
{
    NS_LOG_FUNCTION(this);
}

// ---- Configuration ----

void
OranNtnXappDopplerComp::SetCarrierFrequency(double freqHz)
{
    m_carrierFrequency = freqHz;
}

void
OranNtnXappDopplerComp::SetCompensationMode(const std::string& mode)
{
    NS_ASSERT_MSG(mode == "common" || mode == "per-ue" || mode == "hybrid",
                  "Invalid compensation mode: " << mode);
    m_compensationMode = mode;
}

void
OranNtnXappDopplerComp::SetUpdateInterval(Time interval)
{
    m_updateInterval = interval;
}

void
OranNtnXappDopplerComp::SetMaxResidualDoppler(double Hz)
{
    m_maxResidualDoppler = Hz;
}

// ---- Query ----

double
OranNtnXappDopplerComp::GetCommonDopplerShift(uint32_t gnbId) const
{
    auto it = m_gnbDopplerStates.find(gnbId);
    if (it != m_gnbDopplerStates.end())
    {
        return it->second.commonDopplerShift;
    }
    return 0.0;
}

double
OranNtnXappDopplerComp::GetResidualDoppler(uint32_t gnbId, uint32_t ueId) const
{
    auto gnbIt = m_gnbDopplerStates.find(gnbId);
    if (gnbIt != m_gnbDopplerStates.end())
    {
        auto ueIt = gnbIt->second.perUeResidual.find(ueId);
        if (ueIt != gnbIt->second.perUeResidual.end())
        {
            return ueIt->second;
        }
    }
    return 0.0;
}

OranNtnXappDopplerComp::DopplerMetrics
OranNtnXappDopplerComp::GetDopplerMetrics() const
{
    return m_dopplerMetrics;
}

// ---- E2 data processing ----

void
OranNtnXappDopplerComp::ProcessKpmReport(const E2KpmReport& report)
{
    NS_LOG_FUNCTION(this << report.gnbId << report.ueId << report.doppler_Hz);

    // Only process NTN gNB reports (Doppler compensation is NTN-specific)
    if (!report.isNtn)
    {
        return;
    }

    // Ensure state entry exists for this gNB
    auto& state = m_gnbDopplerStates[report.gnbId];

    // Store per-UE Doppler measurement (raw value from KPM)
    // The perUeResidual map is temporarily used to store raw Doppler until
    // ComputeCommonDoppler and ComputePerUeResidual are called in DecisionCycle
    state.perUeResidual[report.ueId] = report.doppler_Hz;

    NS_LOG_DEBUG("Stored Doppler for gNB " << report.gnbId
                 << " UE " << report.ueId
                 << ": " << report.doppler_Hz << " Hz");
}

void
OranNtnXappDopplerComp::DecisionCycle()
{
    NS_LOG_FUNCTION(this);

    double now = Simulator::Now().GetSeconds();

    for (auto& [gnbId, state] : m_gnbDopplerStates)
    {
        // Skip if no UE measurements available
        if (state.perUeResidual.empty())
        {
            continue;
        }

        // Check minimum update interval
        double elapsed = now - state.lastUpdateTime;
        if (elapsed < m_updateInterval.GetSeconds() && state.lastUpdateTime > 0.0)
        {
            continue;
        }

        // Step 1: Compute common Doppler (beam-center reference)
        ComputeCommonDoppler(gnbId);

        // Step 2: Compute per-UE residual Doppler
        ComputePerUeResidual(gnbId);

        // Step 3: Check if any residual exceeds threshold
        double maxResidual = 0.0;
        uint32_t worstUeId = 0;
        for (const auto& [ueId, residual] : state.perUeResidual)
        {
            double absResidual = std::abs(residual);
            if (absResidual > maxResidual)
            {
                maxResidual = absResidual;
                worstUeId = ueId;
            }
        }

        NS_LOG_INFO("gNB " << gnbId
                    << ": commonDoppler=" << state.commonDopplerShift << " Hz"
                    << ", dopplerRate=" << state.dopplerRate << " Hz/s"
                    << ", maxResidual=" << maxResidual << " Hz"
                    << " (UE " << worstUeId << ")"
                    << ", threshold=" << m_maxResidualDoppler << " Hz");

        // Step 4: Submit compensation update if needed
        if (maxResidual > m_maxResidualDoppler)
        {
            // Build DOPPLER_COMP_UPDATE action
            // parameter1 = common Doppler shift to apply at gNB
            // parameter2 = Doppler rate for predictive compensation
            double confidence = 1.0 - (maxResidual / (maxResidual + m_maxResidualDoppler));
            E2RcAction action = BuildAction(E2RcActionType::DOPPLER_COMP_UPDATE,
                                            gnbId,
                                            0, // cell-wide common compensation
                                            confidence);
            action.parameter1 = state.commonDopplerShift;
            action.parameter2 = state.dopplerRate;

            bool accepted = SubmitAction(action);
            RecordDecision(accepted, confidence, 0.0);

            if (accepted)
            {
                state.lastUpdateTime = now;
                m_dopplerMetrics.compensationUpdates++;

                NS_LOG_INFO("Doppler compensation update submitted for gNB " << gnbId
                            << ": commonDoppler=" << state.commonDopplerShift << " Hz"
                            << ", rate=" << state.dopplerRate << " Hz/s");
            }

            // For per-ue or hybrid mode, also submit per-UE corrections
            if (m_compensationMode == "per-ue" || m_compensationMode == "hybrid")
            {
                for (const auto& [ueId, residual] : state.perUeResidual)
                {
                    if (std::abs(residual) > m_maxResidualDoppler)
                    {
                        double ueConfidence =
                            1.0 - (std::abs(residual) /
                                   (std::abs(residual) + m_maxResidualDoppler));
                        E2RcAction ueAction =
                            BuildAction(E2RcActionType::DOPPLER_COMP_UPDATE,
                                        gnbId,
                                        ueId,
                                        ueConfidence);
                        ueAction.parameter1 = residual;
                        ueAction.parameter2 = 0.0; // No per-UE rate tracking

                        SubmitAction(ueAction);

                        NS_LOG_DEBUG("Per-UE Doppler correction for UE " << ueId
                                     << ": residual=" << residual << " Hz");
                    }
                }
            }

            // Fire trace
            m_dopplerUpdated(gnbId, state.commonDopplerShift, maxResidual);
        }

        // Update aggregate metrics
        m_dopplerMetrics.avgCommonDoppler_Hz =
            std::abs(state.commonDopplerShift);
        if (std::abs(state.commonDopplerShift) > m_dopplerMetrics.maxCommonDoppler_Hz)
        {
            m_dopplerMetrics.maxCommonDoppler_Hz =
                std::abs(state.commonDopplerShift);
        }
        m_dopplerMetrics.avgResidualDoppler_Hz = maxResidual;
        if (maxResidual > m_dopplerMetrics.maxResidualDoppler_Hz)
        {
            m_dopplerMetrics.maxResidualDoppler_Hz = maxResidual;
        }
        m_dopplerMetrics.avgDopplerRate_Hz_per_s = std::abs(state.dopplerRate);
    }
}

void
OranNtnXappDopplerComp::ComputeCommonDoppler(uint32_t gnbId)
{
    NS_LOG_FUNCTION(this << gnbId);

    auto& state = m_gnbDopplerStates[gnbId];

    // Average Doppler across all UEs in this beam
    // Per 3GPP TS 38.821 Section 6.1.4, the common Doppler is the
    // beam-center reference Doppler that can be pre-compensated at the gNB
    double sumDoppler = 0.0;
    uint32_t count = 0;

    // Use raw Doppler values from KPM reports stored in m_kpmDatabase
    auto dbIt = m_kpmDatabase.find(gnbId);
    if (dbIt != m_kpmDatabase.end())
    {
        // Collect latest Doppler per UE from recent reports
        std::map<uint32_t, double> latestUeDoppler;
        for (const auto& report : dbIt->second)
        {
            if (report.isNtn)
            {
                latestUeDoppler[report.ueId] = report.doppler_Hz;
            }
        }

        for (const auto& [ueId, doppler] : latestUeDoppler)
        {
            sumDoppler += doppler;
            count++;
        }
    }

    double previousCommon = state.commonDopplerShift;
    double previousTime = state.lastUpdateTime;
    double now = Simulator::Now().GetSeconds();

    if (count > 0)
    {
        state.commonDopplerShift = sumDoppler / static_cast<double>(count);
    }
    else
    {
        state.commonDopplerShift = 0.0;
    }

    // Track Doppler rate (change per second) for predictive compensation
    double dt = now - previousTime;
    if (dt > 0.0 && previousTime > 0.0)
    {
        state.dopplerRate =
            (state.commonDopplerShift - previousCommon) / dt;
    }
    else
    {
        state.dopplerRate = 0.0;
    }

    NS_LOG_DEBUG("gNB " << gnbId
                 << ": commonDoppler=" << state.commonDopplerShift << " Hz"
                 << " (averaged over " << count << " UEs)"
                 << ", rate=" << state.dopplerRate << " Hz/s");
}

void
OranNtnXappDopplerComp::ComputePerUeResidual(uint32_t gnbId)
{
    NS_LOG_FUNCTION(this << gnbId);

    auto& state = m_gnbDopplerStates[gnbId];
    double commonDoppler = state.commonDopplerShift;

    // Residual = UE Doppler - common Doppler
    // After common pre-compensation at the gNB, each UE still sees
    // this residual shift that must be compensated at the UE side

    // Get latest per-UE Doppler from KPM database
    auto dbIt = m_kpmDatabase.find(gnbId);
    if (dbIt != m_kpmDatabase.end())
    {
        std::map<uint32_t, double> latestUeDoppler;
        for (const auto& report : dbIt->second)
        {
            if (report.isNtn)
            {
                latestUeDoppler[report.ueId] = report.doppler_Hz;
            }
        }

        // Overwrite perUeResidual with actual residuals
        state.perUeResidual.clear();
        for (const auto& [ueId, doppler] : latestUeDoppler)
        {
            state.perUeResidual[ueId] = doppler - commonDoppler;

            NS_LOG_DEBUG("UE " << ueId
                         << ": rawDoppler=" << doppler << " Hz"
                         << ", residual=" << (doppler - commonDoppler) << " Hz");
        }
    }
}

E2Subscription
OranNtnXappDopplerComp::GetRequiredSubscription() const
{
    E2Subscription sub;
    sub.subscriptionId = 0; // Assigned by E2 termination
    sub.ricRequestorId = GetXappId();
    sub.ranFunctionId = 2; // KPM service model
    sub.reportingPeriod = MilliSeconds(200);
    sub.eventTrigger = false; // Periodic reporting
    sub.eventThreshold = 0.0;
    sub.batchOnVisibility = false;
    sub.maxBufferAge = Seconds(5);
    sub.useIslRelay = false;
    return sub;
}

} // namespace ns3
