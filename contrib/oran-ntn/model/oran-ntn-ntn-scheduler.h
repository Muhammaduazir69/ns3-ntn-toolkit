/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-only
 * Author: Muhammad Uzair
 *
 * NTN-Aware MAC Scheduler
 *
 * Extends mmWave FlexTTI MAC scheduler for satellite-specific constraints:
 *   - Long RTT HARQ timing (5-40ms NTN vs 1ms terrestrial)
 *   - Time-varying link capacity from orbital dynamics
 *   - Beam hopping integration (only schedule in active beams)
 *   - Network slice-aware PRB allocation from xApp decisions
 *   - Doppler-compensated timing advance
 *   - ModCod-aware transport block sizing (DVB-S2X integration)
 *
 * Novel features:
 *   - Orbit-predictive scheduling using TTE-weighted priority
 *   - Adaptive HARQ process pool sizing based on RTT
 *   - Beam-hopping-aware slot allocation with guard periods
 *   - Cross-slice proportional fairness under NTN capacity constraints
 *   - Predictive MCS selection using channel prediction from AI xApp
 */

#ifndef ORAN_NTN_NTN_SCHEDULER_H
#define ORAN_NTN_NTN_SCHEDULER_H

#include "oran-ntn-types.h"

#include <ns3/mmwave-flex-tti-mac-scheduler.h>
#include <ns3/nstime.h>

#include <map>
#include <set>
#include <vector>

namespace ns3
{

class OranNtnSatBridge;

/**
 * \ingroup oran-ntn
 * \brief NTN-aware MAC scheduler extending mmWave FlexTTI
 */
class OranNtnScheduler : public mmwave::MmWaveFlexTtiMacScheduler
{
  public:
    static TypeId GetTypeId();
    OranNtnScheduler();
    ~OranNtnScheduler() override;

    // ---- NTN configuration ----

    void SetSatBridge(Ptr<OranNtnSatBridge> bridge);
    void SetSatelliteId(uint32_t satId);

    /**
     * \brief Set NTN RTT for HARQ timing compensation
     */
    void SetNtnRttCompensation(Time rtt);

    /**
     * \brief Update available link capacity from satellite bridge
     */
    void SetAvailableCapacity(double capacityMbps);

    /**
     * \brief Set beam hopping schedule from xApp
     *
     * Only schedules UEs in currently-active beams per the hopping pattern.
     */
    void SetBeamHoppingSchedule(const std::vector<BeamAllocationEntry>& schedule);

    /**
     * \brief Apply per-slice PRB constraints from Slice Manager xApp
     */
    void ApplySliceConstraints(const std::map<uint8_t, double>& slicePrbShares);

    /**
     * \brief Set per-UE timing advance compensation
     */
    void SetUeTimingAdvance(uint16_t rnti, Time ta);

    /**
     * \brief Enable orbit-predictive priority scheduling
     *
     * UEs with lower TTE get higher priority to complete transfers
     * before handover.
     */
    void SetTtePriorityScheduling(bool enable);

    /**
     * \brief Set DVB-S2X ModCod constraint for a beam
     *
     * Limits MCS selection to match the satellite link's ModCod capability.
     */
    void SetBeamModCodConstraint(uint32_t beamId, uint8_t maxMcs);

    /**
     * \brief Enable predictive MCS from AI channel prediction
     */
    void SetPredictiveMcs(bool enable);

    /**
     * \brief Get current scheduling statistics
     */
    struct NtnSchedulerStats
    {
        uint32_t totalScheduled;
        uint32_t droppedDueToRtt;
        uint32_t droppedDueToBeamHop;
        uint32_t harqTimeouts;
        double avgHarqDelay_ms;
        double avgSliceUtilization;
        uint32_t ttePriorityBoosts;
    };
    NtnSchedulerStats GetNtnStats() const;

    /**
     * \brief Prepare NTN-aware DL scheduling
     */
    void PrepareNtnDlScheduling();

    /**
     * \brief Prepare NTN-aware UL scheduling
     */
    void PrepareNtnUlScheduling();

  protected:
    void DoDispose() override;

  private:
    /**
     * \brief Check if a beam is active in current hopping slot
     */
    bool IsBeamActive(uint32_t beamId) const;

    /**
     * \brief Compute extended HARQ timeout accounting for NTN RTT
     */
    uint32_t ComputeNtnHarqTimeout() const;

    /**
     * \brief Apply TTE-weighted priority adjustment
     */
    void ApplyTtePriority(std::map<uint16_t, double>& priorities) const;

    /**
     * \brief Clamp MCS to beam ModCod constraint
     */
    uint8_t ClampMcsToModCod(uint32_t beamId, uint8_t requestedMcs) const;

    /**
     * \brief Compute per-slice PRB boundaries
     */
    struct SlicePrbRange
    {
        uint32_t startPrb;
        uint32_t endPrb;
        uint8_t sliceId;
    };
    std::vector<SlicePrbRange> ComputeSlicePrbRanges() const;

    // Configuration
    Ptr<OranNtnSatBridge> m_satBridge;
    uint32_t m_satId;
    Time m_ntnRtt;
    double m_availableCapacityMbps;
    bool m_ttePriorityScheduling;
    bool m_predictiveMcs;

    // Beam hopping
    std::vector<BeamAllocationEntry> m_beamHopSchedule;
    std::set<uint32_t> m_activeBeams;
    uint32_t m_currentHopSlot;

    // Slice constraints
    std::map<uint8_t, double> m_slicePrbShares;

    // Per-UE NTN state
    std::map<uint16_t, Time> m_ueTimingAdvance;
    std::map<uint16_t, double> m_ueTte;          //!< UE TTE for priority

    // Beam ModCod constraints
    std::map<uint32_t, uint8_t> m_beamMaxMcs;

    // Statistics
    NtnSchedulerStats m_ntnStats;
};

} // namespace ns3

#endif // ORAN_NTN_NTN_SCHEDULER_H
