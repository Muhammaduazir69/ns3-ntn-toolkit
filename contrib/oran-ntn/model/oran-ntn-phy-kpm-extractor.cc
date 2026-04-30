/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-only
 * Author: Muhammad Uzair
 *
 * PHY KPM Extractor - Implementation
 *
 * Extracts real KPM metrics from mmWave PHY layer traces and combines
 * them with NTN satellite metrics from the OranNtnSatBridge. Provides
 * complete E2KpmReport population for the Near-RT RIC.
 */

#include "oran-ntn-phy-kpm-extractor.h"

#include "oran-ntn-sat-bridge.h"

#include <ns3/double.h>
#include <ns3/log.h>
#include <ns3/mmwave-enb-net-device.h>
#include <ns3/mmwave-ue-net-device.h>
#include <ns3/simulator.h>
#include <ns3/uinteger.h>

#include <algorithm>
#include <cmath>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("OranNtnPhyKpmExtractor");
NS_OBJECT_ENSURE_REGISTERED(OranNtnPhyKpmExtractor);

TypeId
OranNtnPhyKpmExtractor::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::OranNtnPhyKpmExtractor")
            .SetParent<Object>()
            .SetGroupName("OranNtn")
            .AddConstructor<OranNtnPhyKpmExtractor>()
            .AddAttribute("MaxSinrHistory",
                          "Maximum number of SINR samples to retain per UE",
                          UintegerValue(500),
                          MakeUintegerAccessor(&OranNtnPhyKpmExtractor::m_maxSinrHistory),
                          MakeUintegerChecker<uint32_t>(1, 10000))
            .AddAttribute("MaxBytesHistory",
                          "Maximum number of throughput samples to retain per UE",
                          UintegerValue(1000),
                          MakeUintegerAccessor(&OranNtnPhyKpmExtractor::m_maxBytesHistory),
                          MakeUintegerChecker<uint32_t>(1, 50000))
            .AddTraceSource("SinrMeasured",
                            "Fired when a new SINR measurement is recorded",
                            MakeTraceSourceAccessor(&OranNtnPhyKpmExtractor::m_sinrMeasured),
                            "ns3::OranNtnPhyKpmExtractor::SinrTracedCallback")
            .AddTraceSource("CqiMeasured",
                            "Fired when a new CQI measurement is recorded",
                            MakeTraceSourceAccessor(&OranNtnPhyKpmExtractor::m_cqiMeasured),
                            "ns3::OranNtnPhyKpmExtractor::CqiTracedCallback")
            .AddTraceSource("ThroughputMeasured",
                            "Fired when throughput is updated",
                            MakeTraceSourceAccessor(&OranNtnPhyKpmExtractor::m_throughputMeasured),
                            "ns3::OranNtnPhyKpmExtractor::ThroughputTracedCallback");
    return tid;
}

OranNtnPhyKpmExtractor::OranNtnPhyKpmExtractor()
    : m_satBridge(nullptr),
      m_maxSinrHistory(500),
      m_maxBytesHistory(1000)
{
    NS_LOG_FUNCTION(this);
}

OranNtnPhyKpmExtractor::~OranNtnPhyKpmExtractor()
{
    NS_LOG_FUNCTION(this);
}

void
OranNtnPhyKpmExtractor::DoDispose()
{
    NS_LOG_FUNCTION(this);
    m_ueStates.clear();
    m_rntiToUeId.clear();
    m_satBridge = nullptr;
    Object::DoDispose();
}

// ============================================================================
//  Attachment
// ============================================================================

void
OranNtnPhyKpmExtractor::AttachToEnbPhy(Ptr<mmwave::MmWaveEnbNetDevice> enbDev, uint32_t satId)
{
    NS_LOG_FUNCTION(this << enbDev << satId);

    if (!enbDev)
    {
        NS_LOG_WARN("AttachToEnbPhy: null eNB device, skipping");
        return;
    }

    NS_LOG_INFO("PHY KPM Extractor: attached to eNB device on satId=" << satId
                << " (node " << enbDev->GetNode()->GetId() << ")");

    /*
     * In the simplified integration mode we store the satId so that
     * UePhyState entries created via AttachToUePhy can be associated
     * with the correct satellite.  Direct PHY trace connection would
     * require access to MmWaveEnbPhy internals which differ across
     * mmWave module versions; the callbacks are instead driven by
     * the E2 interface layer or external wiring in the scenario script.
     */
}

void
OranNtnPhyKpmExtractor::AttachToUePhy(Ptr<mmwave::MmWaveUeNetDevice> ueDev, uint32_t ueId)
{
    NS_LOG_FUNCTION(this << ueDev << ueId);

    if (!ueDev)
    {
        NS_LOG_WARN("AttachToUePhy: null UE device, skipping");
        return;
    }

    // Create or reset the per-UE state
    UePhyState state{};
    state.ueId = ueId;
    state.rnti = 0; // Will be populated when first PHY callback fires
    state.servingSatId = 0;
    state.latestSinr_dB = -20.0; // pessimistic initial
    state.latestCqi = 0;
    state.latestMcs = 0;
    state.avgThroughput_Mbps = 0.0;
    state.harqAcks = 0;
    state.harqNacks = 0;
    state.harqRetransmissions = 0;
    state.lastUpdateTime = Simulator::Now().GetSeconds();

    m_ueStates[ueId] = state;

    NS_LOG_INFO("PHY KPM Extractor: tracking UE " << ueId
                << " (node " << ueDev->GetNode()->GetId() << ")");
}

void
OranNtnPhyKpmExtractor::SetSatBridge(Ptr<OranNtnSatBridge> bridge)
{
    NS_LOG_FUNCTION(this << bridge);
    m_satBridge = bridge;
}

// ============================================================================
//  KPM report generation
// ============================================================================

E2KpmReport
OranNtnPhyKpmExtractor::GetRealKpmReport(uint32_t ueId) const
{
    NS_LOG_FUNCTION(this << ueId);

    E2KpmReport report{};
    report.timestamp = Simulator::Now().GetSeconds();
    report.ueId = ueId;

    // --- Fill from PHY state ---
    auto it = m_ueStates.find(ueId);
    if (it != m_ueStates.end())
    {
        const UePhyState& s = it->second;

        report.gnbId = s.servingSatId;
        report.sinr_dB = s.latestSinr_dB;
        report.cqi = s.latestCqi;
        report.mcs = s.latestMcs;
        report.wbCqi = static_cast<double>(s.latestCqi);

        // Throughput from bytes history
        report.throughput_Mbps = GetAvgThroughput(ueId, 1.0);

        // HARQ BLER
        report.harqBler = GetHarqBler(ueId);
        report.harqRetx = s.harqRetransmissions;

        // Approximate RSRP from SINR (assumes noise-limited)
        // RSRP ~ SINR + thermal noise floor
        // Noise floor for 100 MHz BW: -174 + 10*log10(1e8) + NF ~ -174 + 80 + 7 = -87 dBm
        double noisePower_dBm = -87.0;
        report.rsrp_dBm = s.latestSinr_dB + noisePower_dBm;
        report.rsrq_dB = s.latestSinr_dB - 3.0; // Simplified RSRQ estimate

        // Spectral efficiency from MCS (approximate Shannon bound)
        // SE ~ log2(1 + 10^(SINR/10))
        double sinrLinear = std::pow(10.0, s.latestSinr_dB / 10.0);
        report.spectralEfficiency = std::log2(1.0 + sinrLinear);

        // ModCod index: map spectral efficiency to DVB-S2X index (0-27 typical range)
        report.modCod = static_cast<uint8_t>(
            std::min(27.0, std::max(0.0, report.spectralEfficiency * 5.4)));

        // C/N0 from SINR and bandwidth
        // C/N0 = SINR + 10*log10(BW)
        double bwHz = 1.0e8; // 100 MHz default
        report.cno_dBHz = s.latestSinr_dB + 10.0 * std::log10(bwHz);

        NS_LOG_DEBUG("PHY KPM for UE " << ueId << ": SINR=" << s.latestSinr_dB
                     << " dB, CQI=" << (uint32_t)s.latestCqi
                     << ", MCS=" << (uint32_t)s.latestMcs
                     << ", Tput=" << report.throughput_Mbps << " Mbps"
                     << ", BLER=" << report.harqBler);
    }
    else
    {
        NS_LOG_WARN("GetRealKpmReport: no PHY data for UE " << ueId);
    }

    // --- Augment with NTN metrics from satellite bridge ---
    if (m_satBridge)
    {
        uint32_t satId = report.gnbId;
        report.isNtn = true;

        try
        {
            report.elevation_deg = m_satBridge->ComputeElevationAngle(ueId, satId);
            report.doppler_Hz = m_satBridge->ComputeDopplerShift(ueId, satId, 2.0e9);
            report.propagationDelay_ms =
                m_satBridge->ComputePropagationDelay(ueId, satId).GetMilliSeconds();

            // TTE from bridge
            const UeBridgeState& ueState = m_satBridge->GetUeState(ueId);
            report.tte_s = ueState.servingTte_s;
            report.beamId = ueState.servingBeamId;
            report.latency_ms = report.propagationDelay_ms * 2.0; // RTT approximation

            // Beam gain at UE position
            report.beamGain_dB =
                m_satBridge->GetBeamGainAtUe(ueId, satId, ueState.servingBeamId);

            // Fading gain: difference between expected free-space and measured
            // (positive means constructive, negative means fading)
            report.fadingGain_dB = 0.0; // Placeholder; real value from channel model

            // Markov state: 0=clear sky for high elevations, 1=shadow for medium
            if (report.elevation_deg > 40.0)
            {
                report.markovState = 0; // Clear
            }
            else if (report.elevation_deg > 20.0)
            {
                report.markovState = 1; // Shadow
            }
            else
            {
                report.markovState = 2; // Blocked
            }

            // Inter-beam interference placeholder
            report.interBeamInterference_dBm = -120.0; // Low default

            // Beam tracking error: proportional to Doppler
            report.beamTrackingError_deg =
                std::abs(report.doppler_Hz) / 1.0e6; // Rough heuristic

            // Cell-level metrics from satellite state
            const SatelliteBridgeState& satState = m_satBridge->GetSatState(satId);
            uint32_t totalActiveUes = 0;
            double totalPrbUtil = 0.0;
            uint32_t beamCount = 0;
            for (const auto& bl : satState.beamLoads)
            {
                totalPrbUtil += bl.second;
                beamCount++;
            }
            for (const auto& bu : satState.beamActiveUes)
            {
                totalActiveUes += bu.second;
            }
            report.activeUes = totalActiveUes;
            report.prbUtilization = (beamCount > 0) ? (totalPrbUtil / beamCount) : 0.0;

            NS_LOG_DEBUG("NTN augmented: elev=" << report.elevation_deg
                         << " deg, Doppler=" << report.doppler_Hz
                         << " Hz, TTE=" << report.tte_s << " s"
                         << ", delay=" << report.propagationDelay_ms << " ms");
        }
        catch (const std::exception& e)
        {
            NS_LOG_WARN("NTN metric extraction failed for UE "
                        << ueId << " sat " << satId << ": " << e.what());
        }
    }
    else
    {
        // No satellite bridge — terrestrial mode
        report.isNtn = false;
        report.elevation_deg = 90.0;
        report.doppler_Hz = 0.0;
        report.propagationDelay_ms = 0.0;
        report.tte_s = 1e9;
        report.latency_ms = 1.0; // 1 ms typical for terrestrial
    }

    // --- Default zero-fill for fields not yet populated ---
    // Dual connectivity defaults
    report.dualConnected = false;
    report.tnSinr_dB = 0.0;
    report.ntnSinr_dB = report.sinr_dB;
    report.tnThroughput_Mbps = 0.0;
    report.ntnThroughput_Mbps = report.throughput_Mbps;
    report.bearerSplitRatio = 0.0;

    // Energy defaults (populated externally if available)
    report.batteryStateOfCharge = 1.0;
    report.solarPower_W = 0.0;
    report.inEclipse = false;

    return report;
}

// ============================================================================
//  PHY trace callbacks
// ============================================================================

void
OranNtnPhyKpmExtractor::DlPhyReceptionCallback(uint16_t rnti,
                                                 uint8_t ccId,
                                                 double sinr,
                                                 uint32_t tbSize,
                                                 uint8_t mcs)
{
    NS_LOG_FUNCTION(this << rnti << (uint32_t)ccId << sinr << tbSize << (uint32_t)mcs);

    auto rntiIt = m_rntiToUeId.find(rnti);
    if (rntiIt == m_rntiToUeId.end())
    {
        NS_LOG_DEBUG("DlPhyReception: unknown RNTI " << rnti << ", ignoring");
        return;
    }

    uint32_t ueId = rntiIt->second;
    auto& state = m_ueStates[ueId];
    double now = Simulator::Now().GetSeconds();

    // Update SINR
    double sinr_dB = 10.0 * std::log10(std::max(sinr, 1e-20));
    state.sinrHistory.push_back({now, sinr_dB});
    while (state.sinrHistory.size() > m_maxSinrHistory)
    {
        state.sinrHistory.pop_front();
    }
    state.latestSinr_dB = sinr_dB;

    // Update MCS
    state.latestMcs = mcs;

    // Add to bytes history for throughput calculation
    state.bytesHistory.push_back({now, tbSize});
    while (state.bytesHistory.size() > m_maxBytesHistory)
    {
        state.bytesHistory.pop_front();
    }

    state.lastUpdateTime = now;

    // Fire trace
    m_sinrMeasured(ueId, sinr_dB);

    NS_LOG_DEBUG("DL PHY RX: UE " << ueId << " SINR=" << sinr_dB
                 << " dB, MCS=" << (uint32_t)mcs << ", TB=" << tbSize << " B");
}

void
OranNtnPhyKpmExtractor::UlPhyReceptionCallback(uint16_t rnti,
                                                 uint8_t ccId,
                                                 double sinr,
                                                 uint32_t tbSize,
                                                 uint8_t mcs)
{
    NS_LOG_FUNCTION(this << rnti << (uint32_t)ccId << sinr << tbSize << (uint32_t)mcs);

    auto rntiIt = m_rntiToUeId.find(rnti);
    if (rntiIt == m_rntiToUeId.end())
    {
        NS_LOG_DEBUG("UlPhyReception: unknown RNTI " << rnti << ", ignoring");
        return;
    }

    uint32_t ueId = rntiIt->second;
    auto& state = m_ueStates[ueId];
    double now = Simulator::Now().GetSeconds();

    // Update SINR (UL measurement at eNB side)
    double sinr_dB = 10.0 * std::log10(std::max(sinr, 1e-20));
    state.sinrHistory.push_back({now, sinr_dB});
    while (state.sinrHistory.size() > m_maxSinrHistory)
    {
        state.sinrHistory.pop_front();
    }
    state.latestSinr_dB = sinr_dB;

    // Update MCS
    state.latestMcs = mcs;

    // Add to bytes history
    state.bytesHistory.push_back({now, tbSize});
    while (state.bytesHistory.size() > m_maxBytesHistory)
    {
        state.bytesHistory.pop_front();
    }

    state.lastUpdateTime = now;

    m_sinrMeasured(ueId, sinr_dB);

    NS_LOG_DEBUG("UL PHY RX: UE " << ueId << " SINR=" << sinr_dB
                 << " dB, MCS=" << (uint32_t)mcs << ", TB=" << tbSize << " B");
}

void
OranNtnPhyKpmExtractor::SinrReportCallback(uint16_t rnti, uint8_t ccId, double sinr)
{
    NS_LOG_FUNCTION(this << rnti << (uint32_t)ccId << sinr);

    auto rntiIt = m_rntiToUeId.find(rnti);
    if (rntiIt == m_rntiToUeId.end())
    {
        NS_LOG_DEBUG("SinrReport: unknown RNTI " << rnti << ", ignoring");
        return;
    }

    uint32_t ueId = rntiIt->second;
    auto& state = m_ueStates[ueId];
    double now = Simulator::Now().GetSeconds();

    double sinr_dB = 10.0 * std::log10(std::max(sinr, 1e-20));

    state.latestSinr_dB = sinr_dB;
    state.sinrHistory.push_back({now, sinr_dB});
    while (state.sinrHistory.size() > m_maxSinrHistory)
    {
        state.sinrHistory.pop_front();
    }

    state.lastUpdateTime = now;

    m_sinrMeasured(ueId, sinr_dB);

    NS_LOG_DEBUG("SINR report: UE " << ueId << " SINR=" << sinr_dB << " dB");
}

void
OranNtnPhyKpmExtractor::CqiReportCallback(uint16_t rnti, uint8_t cqi)
{
    NS_LOG_FUNCTION(this << rnti << (uint32_t)cqi);

    auto rntiIt = m_rntiToUeId.find(rnti);
    if (rntiIt == m_rntiToUeId.end())
    {
        NS_LOG_DEBUG("CqiReport: unknown RNTI " << rnti << ", ignoring");
        return;
    }

    uint32_t ueId = rntiIt->second;
    auto& state = m_ueStates[ueId];

    state.latestCqi = cqi;
    state.lastUpdateTime = Simulator::Now().GetSeconds();

    m_cqiMeasured(ueId, cqi);

    NS_LOG_DEBUG("CQI report: UE " << ueId << " CQI=" << (uint32_t)cqi);
}

void
OranNtnPhyKpmExtractor::HarqFeedbackCallback(uint16_t rnti, uint8_t harqId, bool ack)
{
    NS_LOG_FUNCTION(this << rnti << (uint32_t)harqId << ack);

    auto rntiIt = m_rntiToUeId.find(rnti);
    if (rntiIt == m_rntiToUeId.end())
    {
        NS_LOG_DEBUG("HarqFeedback: unknown RNTI " << rnti << ", ignoring");
        return;
    }

    uint32_t ueId = rntiIt->second;
    auto& state = m_ueStates[ueId];

    if (ack)
    {
        state.harqAcks++;
    }
    else
    {
        state.harqNacks++;
        state.harqRetransmissions++;
    }

    state.lastUpdateTime = Simulator::Now().GetSeconds();

    NS_LOG_DEBUG("HARQ feedback: UE " << ueId << " harqId=" << (uint32_t)harqId
                 << " ack=" << ack
                 << " (total ACK=" << state.harqAcks
                 << ", NACK=" << state.harqNacks << ")");
}

void
OranNtnPhyKpmExtractor::MacThroughputCallback(uint16_t rnti, uint32_t bytes, double time)
{
    NS_LOG_FUNCTION(this << rnti << bytes << time);

    auto rntiIt = m_rntiToUeId.find(rnti);
    if (rntiIt == m_rntiToUeId.end())
    {
        NS_LOG_DEBUG("MacThroughput: unknown RNTI " << rnti << ", ignoring");
        return;
    }

    uint32_t ueId = rntiIt->second;
    auto& state = m_ueStates[ueId];

    state.bytesHistory.push_back({time, bytes});
    while (state.bytesHistory.size() > m_maxBytesHistory)
    {
        state.bytesHistory.pop_front();
    }

    // Update cached average throughput
    state.avgThroughput_Mbps = GetAvgThroughput(ueId, 1.0);
    state.lastUpdateTime = time;

    m_throughputMeasured(ueId, state.avgThroughput_Mbps);

    NS_LOG_DEBUG("MAC throughput: UE " << ueId << " bytes=" << bytes
                 << " avgTput=" << state.avgThroughput_Mbps << " Mbps");
}

// ============================================================================
//  Metric computation helpers
// ============================================================================

double
OranNtnPhyKpmExtractor::GetAvgThroughput(uint32_t ueId, double windowSec) const
{
    NS_LOG_FUNCTION(this << ueId << windowSec);

    auto it = m_ueStates.find(ueId);
    if (it == m_ueStates.end())
    {
        return 0.0;
    }

    const auto& history = it->second.bytesHistory;
    if (history.empty())
    {
        return 0.0;
    }

    double now = Simulator::Now().GetSeconds();
    double windowStart = now - windowSec;

    uint64_t totalBytes = 0;
    double earliestTime = now;
    double latestTime = 0.0;

    for (const auto& entry : history)
    {
        if (entry.first >= windowStart)
        {
            totalBytes += entry.second;
            earliestTime = std::min(earliestTime, entry.first);
            latestTime = std::max(latestTime, entry.first);
        }
    }

    if (totalBytes == 0 || earliestTime >= latestTime)
    {
        // If only one sample or no samples in window, use the window duration
        // to avoid division by zero
        double duration = std::max(windowSec, 1e-6);
        double throughput_Mbps = (totalBytes * 8.0) / (duration * 1e6);
        return throughput_Mbps;
    }

    // Compute throughput over the actual span of samples in the window
    double duration = latestTime - earliestTime;
    if (duration < 1e-9)
    {
        duration = windowSec; // fallback
    }

    double throughput_Mbps = (totalBytes * 8.0) / (duration * 1e6);
    return throughput_Mbps;
}

double
OranNtnPhyKpmExtractor::GetHarqBler(uint32_t ueId) const
{
    NS_LOG_FUNCTION(this << ueId);

    auto it = m_ueStates.find(ueId);
    if (it == m_ueStates.end())
    {
        return 0.0;
    }

    const auto& state = it->second;
    uint32_t total = state.harqAcks + state.harqNacks;
    if (total == 0)
    {
        return 0.0;
    }

    return static_cast<double>(state.harqNacks) / static_cast<double>(total);
}

std::vector<std::pair<double, double>>
OranNtnPhyKpmExtractor::GetSinrHistory(uint32_t ueId, uint32_t maxSamples) const
{
    NS_LOG_FUNCTION(this << ueId << maxSamples);

    auto it = m_ueStates.find(ueId);
    if (it == m_ueStates.end())
    {
        return {};
    }

    const auto& history = it->second.sinrHistory;
    uint32_t count = std::min(static_cast<uint32_t>(history.size()), maxSamples);

    std::vector<std::pair<double, double>> result;
    result.reserve(count);

    // Return the last 'count' entries (most recent)
    auto startIt = history.end();
    std::advance(startIt, -static_cast<int>(count));
    for (auto iter = startIt; iter != history.end(); ++iter)
    {
        result.push_back(*iter);
    }

    return result;
}

// ============================================================================
//  Query helpers
// ============================================================================

bool
OranNtnPhyKpmExtractor::HasPhyData(uint32_t ueId) const
{
    auto it = m_ueStates.find(ueId);
    if (it == m_ueStates.end())
    {
        return false;
    }
    // Consider data available if we have at least one SINR sample
    return !it->second.sinrHistory.empty();
}

std::vector<uint32_t>
OranNtnPhyKpmExtractor::GetTrackedUeIds() const
{
    std::vector<uint32_t> ids;
    ids.reserve(m_ueStates.size());
    for (const auto& kv : m_ueStates)
    {
        ids.push_back(kv.first);
    }
    return ids;
}

} // namespace ns3
