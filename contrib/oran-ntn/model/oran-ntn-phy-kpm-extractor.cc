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

    NS_LOG_INFO("PHY KPM Extractor: associated with eNB device on satId=" << satId
                << " (node " << enbDev->GetNode()->GetId() << ")");

    /*
     * This does NOT auto-connect the mmwave RxPacketTraceUe source: that
     * source emits RxPacketTraceParams, which does not match the extractor's
     * per-RNTI DlPhyReceptionCallback signature, and MmWaveEnbPhy internals
     * differ across mmWave module versions. In the shipped toolkit the
     * extractor's per-UE SINR/throughput/HARQ state is fed by
     * NtnRealStackHelper (which owns the RxPacketTraceUe connection) and the
     * scenario script, or via AttachToUePhy. The auto-attach is intentionally
     * documented as scenario/NtnRealStackHelper-fed (audit item 8) rather than
     * silently no-op; emit a WARN so a caller relying on auto-wiring notices.
     */
    NS_LOG_WARN("AttachToEnbPhy: no auto PHY-trace wiring (RxPacketTraceParams "
                "signature mismatch). Feed the extractor via NtnRealStackHelper "
                "accessors / AttachToUePhy / scenario wiring instead.");
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
    state.rnti = 0; // set by RegisterRnti() when the RRC RNTI is known
    state.servingSatId = 0;
    state.latestSinr_dB = -20.0; // pessimistic initial
    state.latestCqi = 0;
    state.latestMcs = 0;
    state.avgThroughput_Mbps = 0.0;
    state.lastCumulativeRxBytes = 0;
    state.haveCumulativeBaseline = false;
    state.latestTbler = std::nan("");
    state.haveTbler = false;
    state.lastUpdateTime = Simulator::Now().GetSeconds();

    m_ueStates[ueId] = state;

    NS_LOG_INFO("PHY KPM Extractor: tracking UE " << ueId
                << " (node " << ueDev->GetNode()->GetId() << ")");
}

void
OranNtnPhyKpmExtractor::RegisterRnti(uint16_t rnti, uint32_t ueId, uint32_t servingCellId)
{
    NS_LOG_FUNCTION(this << rnti << ueId << servingCellId);

    m_rntiToUeId[rnti] = ueId;

    // Ensure a UE state exists so measured samples land somewhere.
    auto it = m_ueStates.find(ueId);
    if (it == m_ueStates.end())
    {
        UePhyState state{};
        state.ueId = ueId;
        state.rnti = rnti;
        state.servingSatId = servingCellId;
        state.latestSinr_dB = -20.0;
        state.latestCqi = 0;
        state.latestMcs = 0;
        state.avgThroughput_Mbps = 0.0;
        state.lastCumulativeRxBytes = 0;
        state.haveCumulativeBaseline = false;
        state.latestTbler = std::nan("");
        state.haveTbler = false;
        state.lastUpdateTime = Simulator::Now().GetSeconds();
        m_ueStates[ueId] = state;
    }
    else
    {
        it->second.rnti = rnti;
        if (servingCellId != 0)
        {
            it->second.servingSatId = servingCellId;
        }
    }

    NS_LOG_INFO("PHY KPM Extractor: registered RNTI " << rnti << " -> UE " << ueId
                << " (cell " << servingCellId << ")");
}

void
OranNtnPhyKpmExtractor::IngestMeasuredSample(uint16_t rnti, double sinrDb,
                                             uint64_t cumulativeRxBytes, double tbler,
                                             uint8_t cqi)
{
    NS_LOG_FUNCTION(this << rnti << sinrDb << cumulativeRxBytes << tbler
                    << (uint32_t)cqi);

    auto rntiIt = m_rntiToUeId.find(rnti);
    if (rntiIt == m_rntiToUeId.end())
    {
        NS_LOG_WARN("IngestMeasuredSample: unknown RNTI " << rnti
                    << " (call RegisterRnti first); sample dropped");
        return;
    }

    const uint32_t ueId = rntiIt->second;
    auto& state = m_ueStates[ueId];
    const double now = Simulator::Now().GetSeconds();

    // Measured SINR (dB) straight from the real PHY -- no closed-form estimate.
    state.latestSinr_dB = sinrDb;
    state.sinrHistory.push_back({now, sinrDb});
    while (state.sinrHistory.size() > m_maxSinrHistory)
    {
        state.sinrHistory.pop_front();
    }

    // Convert cumulative RX bytes to a per-sample delta for the throughput
    // window. The first sample only establishes the baseline.
    if (state.haveCumulativeBaseline && cumulativeRxBytes >= state.lastCumulativeRxBytes)
    {
        const uint32_t delta =
            static_cast<uint32_t>(cumulativeRxBytes - state.lastCumulativeRxBytes);
        state.bytesHistory.push_back({now, delta});
        while (state.bytesHistory.size() > m_maxBytesHistory)
        {
            state.bytesHistory.pop_front();
        }
    }
    state.lastCumulativeRxBytes = cumulativeRxBytes;
    state.haveCumulativeBaseline = true;

    // Measured DL TBLER (block error rate).
    if (!std::isnan(tbler))
    {
        state.latestTbler = tbler;
        state.haveTbler = true;
    }

    if (cqi > 0)
    {
        state.latestCqi = cqi;
    }

    state.avgThroughput_Mbps = GetAvgThroughput(ueId, 1.0);
    state.lastUpdateTime = now;

    m_sinrMeasured(ueId, sinrDb);
    m_throughputMeasured(ueId, state.avgThroughput_Mbps);
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

        // Throughput from measured RX-byte history (fed via IngestMeasuredSample).
        report.throughput_Mbps = GetAvgThroughput(ueId, 1.0);
        report.throughputMeasured = s.haveCumulativeBaseline;

        // HARQ/TB BLER (measured DL TBLER; NaN if none fed).
        report.harqBler = GetHarqBler(ueId);
        report.blerMeasured = s.haveTbler;
        report.harqRetx = 0;

        // Approximate RSRP from SINR (assumes noise-limited)
        // RSRP ~ SINR + thermal noise floor
        // Noise floor for 100 MHz BW: -174 + 10*log10(1e8) + NF ~ -174 + 80 + 7 = -87 dBm
        double noisePower_dBm = -87.0;
        report.rsrp_dBm = s.latestSinr_dB + noisePower_dBm;
        // Bounded RSRQ from SINR: S/(S+I+N) per RE, clamped to 3GPP [-19.5,-3].
        {
            double sinrLin = std::pow(10.0, s.latestSinr_dB / 10.0);
            report.rsrq_dB = std::max(-19.5, std::min(-3.0,
                                10.0 * std::log10(sinrLin / (1.0 + sinrLin))));
        }

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
    // Measured DL TBLER fed by IngestMeasuredSample (NtnRealStackHelper
    // GetUeRecentTbler). NaN when no error-rate sample has been fed.
    if (state.haveTbler)
    {
        return state.latestTbler;
    }
    return std::nan("");
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

std::vector<oranntn::KpmMeasurement>
OranNtnPhyKpmExtractor::EmitCanonicalKpm(
    uint32_t ueId,
    const std::map<std::string, std::string>& labels) const
{
    const E2KpmReport report = GetRealKpmReport(ueId);
    return oranntn::BuildCanonicalKpmMeasurements(report, labels);
}

} // namespace ns3
