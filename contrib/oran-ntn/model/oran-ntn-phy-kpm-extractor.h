/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-only
 * Author: Muhammad Uzair
 *
 * PHY KPM Extractor - Real KPM extraction from mmWave PHY layer
 *
 * Connects to MmWaveSpectrumPhy trace sources for real SINR/CQI/throughput
 * extraction instead of synthetic generation. Bridges the gap between
 * O-RAN KPM reports and actual ns-3 PHY measurements.
 *
 * Novel features:
 *   - Real-time SINR extraction from MmWaveSpectrumPhy interference calc
 *   - CQI mapping from actual AMC feedback
 *   - HARQ BLER tracking for reliability metrics
 *   - Per-bearer throughput measurement from RLC/PDCP stats
 *   - Latency measurement from MAC scheduling to PHY delivery
 *   - Automatic E2KpmReport population from PHY state
 */

#ifndef ORAN_NTN_PHY_KPM_EXTRACTOR_H
#define ORAN_NTN_PHY_KPM_EXTRACTOR_H

#include "oran-ntn-types.h"

#include <ns3/object.h>
#include <ns3/traced-callback.h>

#include <deque>
#include <map>

namespace ns3
{

namespace mmwave
{
class MmWaveEnbNetDevice;
class MmWaveUeNetDevice;
} // namespace mmwave

class OranNtnSatBridge;

/**
 * \ingroup oran-ntn
 * \brief Extracts real KPM metrics from mmWave PHY layer
 */
class OranNtnPhyKpmExtractor : public Object
{
  public:
    static TypeId GetTypeId();
    OranNtnPhyKpmExtractor();
    ~OranNtnPhyKpmExtractor() override;

    /**
     * \brief Connect to eNB PHY trace sources
     */
    void AttachToEnbPhy(Ptr<mmwave::MmWaveEnbNetDevice> enbDev, uint32_t satId);

    /**
     * \brief Connect to UE PHY trace sources
     */
    void AttachToUePhy(Ptr<mmwave::MmWaveUeNetDevice> ueDev, uint32_t ueId);

    /**
     * \brief Set satellite bridge for NTN-specific metrics
     */
    void SetSatBridge(Ptr<OranNtnSatBridge> bridge);

    /**
     * \brief Build a complete E2KpmReport from actual PHY state
     *
     * Combines mmWave PHY measurements (SINR, CQI, HARQ) with
     * satellite bridge NTN metrics (elevation, Doppler, TTE).
     */
    E2KpmReport GetRealKpmReport(uint32_t ueId) const;

    /**
     * \brief Get historical SINR values for a UE
     */
    std::vector<std::pair<double, double>> GetSinrHistory(uint32_t ueId,
                                                           uint32_t maxSamples = 100) const;

    /**
     * \brief Get current average throughput for a UE
     */
    double GetAvgThroughput(uint32_t ueId, double windowSec = 1.0) const;

    /**
     * \brief Get HARQ BLER for a UE
     */
    double GetHarqBler(uint32_t ueId) const;

    /**
     * \brief Check if PHY data is available for a UE
     */
    bool HasPhyData(uint32_t ueId) const;

    /**
     * \brief Get set of all tracked UE IDs
     */
    std::vector<uint32_t> GetTrackedUeIds() const;

    // ---- Trace sources ----
    TracedCallback<uint32_t, double> m_sinrMeasured;      //!< ueId, sinr_dB
    TracedCallback<uint32_t, uint8_t> m_cqiMeasured;      //!< ueId, cqi
    TracedCallback<uint32_t, double> m_throughputMeasured; //!< ueId, Mbps

  protected:
    void DoDispose() override;

  private:
    // PHY trace callbacks
    void DlPhyReceptionCallback(uint16_t rnti, uint8_t ccId, double sinr,
                                 uint32_t tbSize, uint8_t mcs);
    void UlPhyReceptionCallback(uint16_t rnti, uint8_t ccId, double sinr,
                                 uint32_t tbSize, uint8_t mcs);
    void SinrReportCallback(uint16_t rnti, uint8_t ccId, double sinr);
    void CqiReportCallback(uint16_t rnti, uint8_t cqi);
    void HarqFeedbackCallback(uint16_t rnti, uint8_t harqId, bool ack);
    void MacThroughputCallback(uint16_t rnti, uint32_t bytes, double time);

    // Per-UE measurement state
    struct UePhyState
    {
        uint32_t ueId;
        uint16_t rnti;
        uint32_t servingSatId;

        // SINR tracking
        std::deque<std::pair<double, double>> sinrHistory;  //!< (timestamp, sinr_dB)
        double latestSinr_dB;

        // CQI tracking
        uint8_t latestCqi;
        uint8_t latestMcs;

        // Throughput tracking
        std::deque<std::pair<double, uint32_t>> bytesHistory; //!< (timestamp, bytes)
        double avgThroughput_Mbps;

        // HARQ tracking
        uint32_t harqAcks;
        uint32_t harqNacks;
        uint32_t harqRetransmissions;

        // Timing
        double lastUpdateTime;
    };

    std::map<uint32_t, UePhyState> m_ueStates;   //!< ueId -> state
    std::map<uint16_t, uint32_t> m_rntiToUeId;    //!< RNTI -> ueId mapping

    Ptr<OranNtnSatBridge> m_satBridge;

    uint32_t m_maxSinrHistory;
    uint32_t m_maxBytesHistory;
};

} // namespace ns3

#endif // ORAN_NTN_PHY_KPM_EXTRACTOR_H
