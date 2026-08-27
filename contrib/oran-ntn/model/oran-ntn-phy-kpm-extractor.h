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

#include "oran-ntn-kpm-canonical-ids.h"
#include "oran-ntn-types.h"

#include <ns3/object.h>
#include <ns3/traced-callback.h>

#include <deque>
#include <map>
#include <string>
#include <vector>

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
 *
 * \note Currently exercised by unit tests only; the real-stack examples read
 *       measured PHY KPIs through NtnRealStackHelper accessors instead.
 */
class OranNtnPhyKpmExtractor : public Object
{
  public:
    static TypeId GetTypeId();
    OranNtnPhyKpmExtractor();
    ~OranNtnPhyKpmExtractor() override;

    /**
     * \brief Associate this extractor with an eNB device's satellite id.
     *
     * \note This does NOT auto-connect the mmwave PHY traces: the
     *       RxPacketTraceUe source emits RxPacketTraceParams, which does not
     *       match this extractor's per-RNTI update signature across mmwave
     *       versions. In the shipped toolkit the extractor's per-UE
     *       SINR/throughput/HARQ state is fed by NtnRealStackHelper (which
     *       owns the RxPacketTraceUe connection) and the scenario script, or
     *       via AttachToUePhy. (Audit item 8: the auto-attach is documented as
     *       scenario/NtnRealStackHelper-fed rather than silently a no-op.)
     */
    void AttachToEnbPhy(Ptr<mmwave::MmWaveEnbNetDevice> enbDev, uint32_t satId);

    /**
     * \brief Connect to UE PHY trace sources
     */
    void AttachToUePhy(Ptr<mmwave::MmWaveUeNetDevice> ueDev, uint32_t ueId);

    /**
     * \brief Map a PHY RNTI to a logical UE id and start tracking that UE.
     *
     * NOTHING in the mmwave/nr trace path auto-populates this map (the
     * RxPacketTraceUe source emits RxPacketTraceParams, which does not match a
     * per-RNTI update signature and differs across module versions), so without
     * this call every measured sample fed to IngestMeasuredSample() would be
     * dropped on an unknown-RNTI miss. A scenario built on NtnRealStackHelper
     * registers each UE once via GetUeRnti(i)/GetUeServingCellId(i) (or on an
     * RRC connection-established trace) and then feeds samples per KPM tick.
     */
    void RegisterRnti(uint16_t rnti, uint32_t ueId, uint32_t servingCellId = 0);

    /**
     * \brief ORAN-06: the radio geometry the derived KPIs are computed against.
     *
     * RSRP and C/N0 were each derived from SINR using their OWN hardcoded
     * 100 MHz bandwidth and a 7 dB noise figure baked into a -87 dBm literal,
     * while the shipped caller runs an FR1 numerology-1 carrier that is nowhere
     * near 100 MHz. Two independent constants for one radio also meant the two
     * fields could disagree about the same link. Set these to the carrier the
     * scenario actually runs.
     */
    void SetRadioGeometry(double bandwidthHz, double noiseFigureDb);
    double GetBandwidthHz() const { return m_bandwidthHz; }
    double GetNoiseFigureDb() const { return m_noiseFigureDb; }
    /// Thermal noise power in the configured bandwidth, dBm. -174 dBm/Hz + 10log10(BW) + NF.
    double GetNoiseFloorDbm() const;

    /**
     * \brief ACM margin (dB) held back when selecting a DVB-S2 MODCOD.
     *
     * A real ACM loop does not run at the quasi-error-free threshold with zero
     * headroom. Default 1 dB.
     */
    void SetAcmMarginDb(double m) { m_acmMarginDb = m; }

    /**
     * \brief Feed one MEASURED PHY sample for a UE keyed by its RNTI.
     *
     * This is the real feed path used in the shipped toolkit: the scenario
     * reads NtnRealStackHelper's measured accessors
     * (GetUeRecentSinrDb / GetUeRxBytes / GetUeRecentTbler) each KPM period and
     * calls this. The RNTI is resolved through the RegisterRnti() map; an
     * unregistered RNTI is dropped with a warning (no silent fabrication).
     *
     * \param rnti              PHY RNTI (from NtnRealStackHelper::GetUeRnti)
     * \param sinrDb            measured DL RS-SINR (dB)
     * \param cumulativeRxBytes measured cumulative DL RX bytes for the UE
     * \param tbler             measured DL TBLER (0..1); NaN = not available
     * \param cqi               measured/derived CQI (0..15); pass 0 if unknown
     */
    void IngestMeasuredSample(uint16_t rnti, double sinrDb,
                              uint64_t cumulativeRxBytes, double tbler,
                              uint8_t cqi = 0);

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
     * \brief Emit the canonical 10-metric KPM measurement vector for a UE
     *        with WG3-aligned metric IDs and label set (Roadmap T8).
     *
     * Wraps GetRealKpmReport() in BuildCanonicalKpmMeasurements(); the
     * resulting vector has the exact metric IDs FlexRIC's KPM SM emits,
     * so an xApp written against FlexRIC can consume our KPM CSV with
     * no remapping.
     */
    std::vector<oranntn::KpmMeasurement>
    EmitCanonicalKpm(uint32_t ueId,
                     const std::map<std::string, std::string>& labels) const;

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
    double m_bandwidthHz{20.0e6};   //!< ORAN-06: was two separate 100 MHz literals
    double m_noiseFigureDb{7.0};    //!< ORAN-06: was baked into a -87 dBm constant
    double m_acmMarginDb{1.0};      //!< headroom held back by the ACM selection

    TracedCallback<uint32_t, double> m_sinrMeasured;      //!< ueId, sinr_dB
    TracedCallback<uint32_t, uint8_t> m_cqiMeasured;      //!< ueId, cqi
    TracedCallback<uint32_t, double> m_throughputMeasured; //!< ueId, Mbps

  protected:
    void DoDispose() override;

  private:
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

        // Throughput tracking (bytesHistory holds per-sample byte DELTAS)
        std::deque<std::pair<double, uint32_t>> bytesHistory; //!< (timestamp, bytes)
        double avgThroughput_Mbps;
        uint64_t lastCumulativeRxBytes; //!< to convert cumulative -> per-sample delta
        bool haveCumulativeBaseline;

        // Error rate (measured DL TBLER)
        double latestTbler;             //!< NaN until a measured value is fed
        bool haveTbler;
        /// ORAN-03: a real SINR sample has been ingested for this UE, so the
        /// KPM writer may label it provenance=measured.
        bool haveSinr = false;

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
