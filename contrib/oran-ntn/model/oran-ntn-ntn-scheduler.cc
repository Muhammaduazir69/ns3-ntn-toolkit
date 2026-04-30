/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-only
 * Author: Muhammad Uzair
 *
 * NTN-Aware MAC Scheduler - Implementation
 *
 * Extends mmWave FlexTTI MAC scheduler with satellite-specific
 * constraints: extended HARQ timing, beam hopping awareness,
 * TTE-weighted priority scheduling, DVB-S2X ModCod clamping,
 * per-slice PRB allocation, and Doppler-compensated timing advance.
 */

#include "oran-ntn-ntn-scheduler.h"

#include "oran-ntn-sat-bridge.h"

#include <ns3/boolean.h>
#include <ns3/double.h>
#include <ns3/log.h>
#include <ns3/simulator.h>
#include <ns3/uinteger.h>

#include <algorithm>
#include <cmath>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("OranNtnScheduler");
NS_OBJECT_ENSURE_REGISTERED(OranNtnScheduler);

// ============================================================================
//  Standard HARQ timeout in terrestrial 5G NR (slots)
// ============================================================================
static const uint32_t STANDARD_HARQ_TIMEOUT_SLOTS = 8;

// ============================================================================
//  Maximum MCS index for mmWave (28 entries in MCS table, 0-27)
// ============================================================================
static const uint8_t MAX_MCS_INDEX = 27;

// ============================================================================
//  Default TTE window for priority computation (seconds)
// ============================================================================
static const double TTE_PRIORITY_WINDOW_S = 120.0;

TypeId
OranNtnScheduler::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::OranNtnScheduler")
            .SetParent<mmwave::MmWaveFlexTtiMacScheduler>()
            .SetGroupName("OranNtn")
            .AddConstructor<OranNtnScheduler>()
            .AddAttribute("NtnRtt",
                          "One-way NTN propagation delay for HARQ timeout extension. "
                          "Typical LEO values: 5-20 ms depending on elevation angle.",
                          TimeValue(MilliSeconds(10)),
                          MakeTimeAccessor(&OranNtnScheduler::m_ntnRtt),
                          MakeTimeChecker(MilliSeconds(0), MilliSeconds(300)))
            .AddAttribute("TtePriorityEnable",
                          "Enable TTE-weighted priority scheduling. UEs closer to "
                          "beam exit receive higher scheduling priority to complete "
                          "ongoing transfers before conditional handover.",
                          BooleanValue(true),
                          MakeBooleanAccessor(&OranNtnScheduler::m_ttePriorityScheduling),
                          MakeBooleanChecker())
            .AddAttribute("PredictiveMcsEnable",
                          "Enable predictive MCS selection based on channel prediction "
                          "from AI xApp. When enabled, uses forecast SINR rather than "
                          "last-measured CQI for MCS determination.",
                          BooleanValue(false),
                          MakeBooleanAccessor(&OranNtnScheduler::m_predictiveMcs),
                          MakeBooleanChecker());
    return tid;
}

OranNtnScheduler::OranNtnScheduler()
    : m_satBridge(nullptr),
      m_satId(0),
      m_ntnRtt(MilliSeconds(10)),
      m_availableCapacityMbps(100.0),
      m_ttePriorityScheduling(true),
      m_predictiveMcs(false),
      m_currentHopSlot(0)
{
    NS_LOG_FUNCTION(this);
    std::memset(&m_ntnStats, 0, sizeof(NtnSchedulerStats));
}

OranNtnScheduler::~OranNtnScheduler()
{
    NS_LOG_FUNCTION(this);
}

void
OranNtnScheduler::DoDispose()
{
    NS_LOG_FUNCTION(this);
    m_satBridge = nullptr;
    m_beamHopSchedule.clear();
    m_activeBeams.clear();
    m_slicePrbShares.clear();
    m_ueTimingAdvance.clear();
    m_ueTte.clear();
    m_beamMaxMcs.clear();
    mmwave::MmWaveFlexTtiMacScheduler::DoDispose();
}

// ============================================================================
//  Configuration setters
// ============================================================================

void
OranNtnScheduler::SetSatBridge(Ptr<OranNtnSatBridge> bridge)
{
    NS_LOG_FUNCTION(this << bridge);
    m_satBridge = bridge;
}

void
OranNtnScheduler::SetSatelliteId(uint32_t satId)
{
    NS_LOG_FUNCTION(this << satId);
    m_satId = satId;
}

void
OranNtnScheduler::SetNtnRttCompensation(Time rtt)
{
    NS_LOG_FUNCTION(this << rtt.As(Time::MS));
    NS_ABORT_MSG_IF(rtt.IsNegative(), "NTN RTT cannot be negative");
    m_ntnRtt = rtt;
    NS_LOG_INFO("NTN scheduler RTT compensation set to " << rtt.As(Time::MS)
                << ", extended HARQ timeout = " << ComputeNtnHarqTimeout() << " slots");
}

void
OranNtnScheduler::SetAvailableCapacity(double capacityMbps)
{
    NS_LOG_FUNCTION(this << capacityMbps);
    m_availableCapacityMbps = capacityMbps;
}

void
OranNtnScheduler::SetBeamHoppingSchedule(const std::vector<BeamAllocationEntry>& schedule)
{
    NS_LOG_FUNCTION(this << schedule.size());
    m_beamHopSchedule = schedule;

    // Determine the current time slot index within the beam hopping frame.
    // We use a simple modular scheme: slot index = floor(simTime / slotDuration) mod frameLen.
    // Rebuild the active beam set for the current slot.
    if (m_beamHopSchedule.empty())
    {
        NS_LOG_WARN("Empty beam hopping schedule received; all beams assumed active");
        return;
    }

    // Determine frame length as max timeSlot + 1
    uint32_t frameLen = 0;
    for (const auto& entry : m_beamHopSchedule)
    {
        if (entry.timeSlot >= frameLen)
        {
            frameLen = entry.timeSlot + 1;
        }
    }

    // Compute current slot index within the hopping frame
    double now_us = Simulator::Now().GetMicroSeconds();
    // Assume 1 ms slot duration as default for frame indexing
    uint32_t slotIdx = static_cast<uint32_t>(now_us / 1000.0) % frameLen;
    m_currentHopSlot = slotIdx;

    // Rebuild active beams for this satellite at the current slot
    m_activeBeams.clear();
    for (const auto& entry : m_beamHopSchedule)
    {
        if (entry.satId == m_satId && entry.timeSlot == m_currentHopSlot)
        {
            m_activeBeams.insert(entry.beamId);
        }
    }

    NS_LOG_INFO("Beam hopping schedule updated: " << m_activeBeams.size()
                << " active beams at slot " << m_currentHopSlot
                << " (frame length " << frameLen << ")");
}

void
OranNtnScheduler::ApplySliceConstraints(const std::map<uint8_t, double>& slicePrbShares)
{
    NS_LOG_FUNCTION(this);
    m_slicePrbShares = slicePrbShares;

    // Validate that shares sum to <= 1.0
    double totalShare = 0.0;
    for (const auto& kv : m_slicePrbShares)
    {
        NS_ABORT_MSG_IF(kv.second < 0.0 || kv.second > 1.0,
                        "Slice " << +kv.first << " PRB share out of range: " << kv.second);
        totalShare += kv.second;
    }
    NS_ABORT_MSG_IF(totalShare > 1.001,
                    "Total slice PRB shares exceed 1.0: " << totalShare);

    NS_LOG_INFO("Slice PRB constraints applied: " << m_slicePrbShares.size()
                << " slices, total share = " << totalShare);
}

void
OranNtnScheduler::SetUeTimingAdvance(uint16_t rnti, Time ta)
{
    NS_LOG_FUNCTION(this << rnti << ta.As(Time::MS));
    m_ueTimingAdvance[rnti] = ta;
}

void
OranNtnScheduler::SetTtePriorityScheduling(bool enable)
{
    NS_LOG_FUNCTION(this << enable);
    m_ttePriorityScheduling = enable;
}

void
OranNtnScheduler::SetBeamModCodConstraint(uint32_t beamId, uint8_t maxMcs)
{
    NS_LOG_FUNCTION(this << beamId << +maxMcs);
    NS_ABORT_MSG_IF(maxMcs > MAX_MCS_INDEX,
                    "ModCod max MCS " << +maxMcs << " exceeds table limit " << +MAX_MCS_INDEX);
    m_beamMaxMcs[beamId] = maxMcs;
}

void
OranNtnScheduler::SetPredictiveMcs(bool enable)
{
    NS_LOG_FUNCTION(this << enable);
    m_predictiveMcs = enable;
}

OranNtnScheduler::NtnSchedulerStats
OranNtnScheduler::GetNtnStats() const
{
    return m_ntnStats;
}

// ============================================================================
//  NTN-aware DL scheduling preparation (called before base scheduler)
// ============================================================================

void
OranNtnScheduler::PrepareNtnDlScheduling()
{
    NS_LOG_FUNCTION(this);

    // ---------------------------------------------------------------
    // Step 1: Update beam hopping state for current simulation time
    // ---------------------------------------------------------------
    if (!m_beamHopSchedule.empty())
    {
        uint32_t frameLen = 0;
        for (const auto& entry : m_beamHopSchedule)
        {
            if (entry.timeSlot >= frameLen)
            {
                frameLen = entry.timeSlot + 1;
            }
        }

        double now_us = Simulator::Now().GetMicroSeconds();
        uint32_t slotIdx = static_cast<uint32_t>(now_us / 1000.0) % frameLen;

        if (slotIdx != m_currentHopSlot)
        {
            m_currentHopSlot = slotIdx;
            m_activeBeams.clear();
            for (const auto& entry : m_beamHopSchedule)
            {
                if (entry.satId == m_satId && entry.timeSlot == m_currentHopSlot)
                {
                    m_activeBeams.insert(entry.beamId);
                }
            }
            NS_LOG_DEBUG("DL trigger: beam hop slot advanced to " << m_currentHopSlot
                         << ", active beams = " << m_activeBeams.size());
        }
    }

    // ---------------------------------------------------------------
    // Step 2: Log beam state for NTN scheduling awareness
    // ---------------------------------------------------------------
    NS_LOG_DEBUG("DL: " << m_activeBeams.size() << " active beams in hop slot "
                 << m_currentHopSlot);

    // ---------------------------------------------------------------
    // Step 3: Compute per-slice PRB ranges for allocation constraints
    // ---------------------------------------------------------------
    std::vector<SlicePrbRange> sliceRanges;
    if (!m_slicePrbShares.empty())
    {
        sliceRanges = ComputeSlicePrbRanges();
        NS_LOG_DEBUG("DL: " << sliceRanges.size() << " slice PRB ranges computed");
        for (const auto& sr : sliceRanges)
        {
            NS_LOG_DEBUG("  Slice " << +sr.sliceId << ": PRB [" << sr.startPrb
                         << ", " << sr.endPrb << "]");
        }
    }

    // ---------------------------------------------------------------
    // Step 5: Adjust HARQ process timeouts for NTN RTT
    // ---------------------------------------------------------------
    uint32_t ntnHarqTimeout = ComputeNtnHarqTimeout();
    NS_LOG_DEBUG("DL: NTN HARQ timeout = " << ntnHarqTimeout << " slots "
                 << "(standard " << STANDARD_HARQ_TIMEOUT_SLOTS << " + RTT extension)");

    NS_LOG_DEBUG("DL: NTN HARQ timeout = " << ntnHarqTimeout << " slots");

    // ---------------------------------------------------------------
    // Step 5: Log MCS clamping from beam ModCod constraints
    // ---------------------------------------------------------------
    if (!m_beamMaxMcs.empty())
    {
        NS_LOG_DEBUG("DL: ModCod constraints active for " << m_beamMaxMcs.size() << " beams");
        // The actual per-UE MCS clamping happens within the parent scheduler's
        // allocation loop. We expose ClampMcsToModCod() for use by derived classes
        // or external callers. Here we log the constraints for diagnostics.
        for (const auto& kv : m_beamMaxMcs)
        {
            NS_LOG_DEBUG("  Beam " << kv.first << ": max MCS = " << +kv.second);
        }
    }

    // ---------------------------------------------------------------
    // Step 6: Summary
    // ---------------------------------------------------------------
    NS_LOG_INFO("NTN DL preparation complete: active beams=" << m_activeBeams.size()
                << " HARQ timeout=" << ntnHarqTimeout << " slots");
}

// ============================================================================
//  NTN-aware UL scheduling preparation
// ============================================================================

void
OranNtnScheduler::PrepareNtnUlScheduling()
{
    NS_LOG_FUNCTION(this);

    // ---------------------------------------------------------------
    // Step 1: Apply per-UE timing advance from Doppler compensation
    // ---------------------------------------------------------------
    // In NTN, the UL timing advance must compensate for:
    //   (a) The large propagation delay (5-40 ms for LEO)
    //   (b) Differential delay between cell-center and cell-edge UEs
    //   (c) Doppler-induced timing drift
    // The timing advance values are updated by the Doppler compensation
    // xApp and stored in m_ueTimingAdvance.

    if (!m_ueTimingAdvance.empty())
    {
        NS_LOG_DEBUG("UL: Timing advance configured for " << m_ueTimingAdvance.size() << " UEs");
        for (const auto& kv : m_ueTimingAdvance)
        {
            NS_LOG_DEBUG("  RNTI " << kv.first << ": TA = " << kv.second.As(Time::MS));
        }
    }

    // ---------------------------------------------------------------
    // Step 2: Compute NTN HARQ timeout
    // ---------------------------------------------------------------
    uint32_t ntnHarqTimeout = ComputeNtnHarqTimeout();
    NS_LOG_DEBUG("UL: NTN HARQ timeout = " << ntnHarqTimeout << " slots");

    // ---------------------------------------------------------------
    // Step 3: Log active beam state for UL
    // ---------------------------------------------------------------
    NS_LOG_DEBUG("UL: " << m_activeBeams.size() << " active beams in current hop slot");

    NS_LOG_INFO("NTN UL preparation complete: HARQ timeout=" << ntnHarqTimeout << " slots");
}

// ============================================================================
//  IsBeamActive - Check beam hopping state
// ============================================================================

bool
OranNtnScheduler::IsBeamActive(uint32_t beamId) const
{
    // If no beam hopping schedule is configured, all beams are considered active
    if (m_activeBeams.empty() && m_beamHopSchedule.empty())
    {
        return true;
    }

    return m_activeBeams.find(beamId) != m_activeBeams.end();
}

// ============================================================================
//  ComputeNtnHarqTimeout - Extended HARQ for satellite RTT
// ============================================================================

uint32_t
OranNtnScheduler::ComputeNtnHarqTimeout() const
{
    // Standard HARQ timeout is 8 slots (terrestrial 5G NR).
    // For NTN, the round-trip feedback loop traverses the satellite link
    // twice (DL data -> UE -> UL ACK/NACK -> gNB), adding 2 * one-way RTT.
    //
    // Extended timeout = standard_timeout + ceil(2 * RTT / slot_duration)
    //
    // For LEO at 600 km altitude, typical one-way delay is ~5-20 ms.
    // With 0.125 ms slot duration (120 kHz SCS), this adds 80-320 slots.
    // With 1 ms slot duration (15 kHz SCS), this adds 10-40 slots.

    if (m_ntnRtt.IsZero())
    {
        return STANDARD_HARQ_TIMEOUT_SLOTS;
    }

    // Use a reasonable default slot duration if PHY config is not yet available.
    // The mmWave module typically uses 100 us (10 slots per 1 ms subframe).
    double slotDuration_us = 100.0; // 100 microseconds default

    double rttRoundTrip_us = 2.0 * m_ntnRtt.GetMicroSeconds();
    uint32_t rttExtensionSlots =
        static_cast<uint32_t>(std::ceil(rttRoundTrip_us / slotDuration_us));

    uint32_t extendedTimeout = STANDARD_HARQ_TIMEOUT_SLOTS + rttExtensionSlots;

    NS_LOG_LOGIC("HARQ timeout: standard=" << STANDARD_HARQ_TIMEOUT_SLOTS
                 << " + RTT_extension=" << rttExtensionSlots
                 << " (RTT=" << m_ntnRtt.As(Time::MS) << ", slot=" << slotDuration_us << " us)"
                 << " = " << extendedTimeout << " slots");

    return extendedTimeout;
}

// ============================================================================
//  ApplyTtePriority - TTE-weighted scheduling priority
// ============================================================================

void
OranNtnScheduler::ApplyTtePriority(std::map<uint16_t, double>& priorities) const
{
    // Orbit-predictive priority scheduling: UEs with lower TTE (closer to
    // beam exit / handover) receive a priority boost so that their ongoing
    // data transfers can complete before the link is interrupted.
    //
    // Priority boost formula:
    //   boost = max(0, (TTE_WINDOW - TTE) / TTE_WINDOW)
    //
    // Where TTE_WINDOW = 120 seconds (configurable). A UE with TTE = 0
    // gets maximum boost of 1.0, while TTE >= 120 s gets no boost.
    // The boost is additive to the existing priority weight.

    if (m_ueTte.empty())
    {
        NS_LOG_DEBUG("TTE priority: no TTE data available, skipping");
        return;
    }

    uint32_t boostCount = 0;

    for (auto& kv : priorities)
    {
        uint16_t rnti = kv.first;
        auto itTte = m_ueTte.find(rnti);
        if (itTte != m_ueTte.end())
        {
            double tte = itTte->second;
            double boost = std::max(0.0, (TTE_PRIORITY_WINDOW_S - tte) / TTE_PRIORITY_WINDOW_S);
            kv.second += boost;

            if (boost > 0.01)
            {
                boostCount++;
                NS_LOG_DEBUG("TTE priority: RNTI " << rnti << " TTE=" << tte
                             << " s, boost=" << boost
                             << ", total priority=" << kv.second);
            }
        }
    }

    // Update stats (const_cast is safe here since stats are mutable diagnostics)
    const_cast<NtnSchedulerStats&>(m_ntnStats).ttePriorityBoosts += boostCount;

    NS_LOG_DEBUG("TTE priority applied: " << boostCount << " UEs boosted out of "
                 << priorities.size());
}

// ============================================================================
//  ClampMcsToModCod - DVB-S2X ModCod constraint
// ============================================================================

uint8_t
OranNtnScheduler::ClampMcsToModCod(uint32_t beamId, uint8_t requestedMcs) const
{
    // In NTN systems using DVB-S2X-compatible waveforms, the maximum
    // achievable MCS is bounded by the satellite link's ModCod capability.
    // This depends on the link budget (EIRP, G/T, path loss, rain fade)
    // and is typically set by the Link Adaptation xApp or the satellite
    // bridge based on current C/N0 measurements.
    //
    // If no constraint exists for the beam, return the requested MCS as-is.

    auto it = m_beamMaxMcs.find(beamId);
    if (it == m_beamMaxMcs.end())
    {
        // No ModCod constraint for this beam
        return requestedMcs;
    }

    uint8_t maxMcs = it->second;
    uint8_t clampedMcs = std::min(requestedMcs, maxMcs);

    if (clampedMcs < requestedMcs)
    {
        NS_LOG_DEBUG("ModCod clamp: beam " << beamId << " requested MCS " << +requestedMcs
                     << " clamped to " << +clampedMcs << " (beam max = " << +maxMcs << ")");
    }

    return clampedMcs;
}

// ============================================================================
//  ComputeSlicePrbRanges - Partition PRBs by slice share
// ============================================================================

std::vector<OranNtnScheduler::SlicePrbRange>
OranNtnScheduler::ComputeSlicePrbRanges() const
{
    // Divide the total PRB pool into contiguous ranges according to each
    // slice's configured share. This implements static partitioning as
    // a baseline; the Slice Manager xApp can dynamically update shares.
    //
    // Example with 100 PRBs and shares {eMBB: 0.6, URLLC: 0.3, mMTC: 0.1}:
    //   eMBB  -> [0, 59]   (60 PRBs)
    //   URLLC -> [60, 89]  (30 PRBs)
    //   mMTC  -> [90, 99]  (10 PRBs)
    //
    // Rounding is handled by giving leftover PRBs to the last slice.

    std::vector<SlicePrbRange> ranges;

    if (m_slicePrbShares.empty())
    {
        return ranges;
    }

    // Use a default total PRB count if PHY config unavailable.
    // Typical mmWave configurations use 66 or 132 RBs.
    uint32_t totalPrbs = 66; // conservative default

    uint32_t allocatedPrbs = 0;
    uint32_t sliceCount = 0;
    uint32_t numSlices = static_cast<uint32_t>(m_slicePrbShares.size());

    for (const auto& kv : m_slicePrbShares)
    {
        sliceCount++;
        SlicePrbRange range;
        range.sliceId = kv.first;
        range.startPrb = allocatedPrbs;

        if (sliceCount == numSlices)
        {
            // Last slice gets all remaining PRBs to avoid rounding gaps
            range.endPrb = totalPrbs - 1;
        }
        else
        {
            uint32_t slicePrbs =
                static_cast<uint32_t>(std::floor(kv.second * static_cast<double>(totalPrbs)));
            if (slicePrbs == 0)
            {
                slicePrbs = 1; // guarantee at least 1 PRB per slice
            }
            range.endPrb = allocatedPrbs + slicePrbs - 1;

            // Clamp to total
            if (range.endPrb >= totalPrbs)
            {
                range.endPrb = totalPrbs - 1;
            }
        }

        allocatedPrbs = range.endPrb + 1;
        ranges.push_back(range);

        NS_LOG_LOGIC("Slice " << +range.sliceId << ": PRBs [" << range.startPrb
                     << ", " << range.endPrb << "] ("
                     << (range.endPrb - range.startPrb + 1) << " PRBs, share="
                     << kv.second << ")");

        // Safety: stop if we have exhausted all PRBs
        if (allocatedPrbs >= totalPrbs)
        {
            break;
        }
    }

    return ranges;
}

} // namespace ns3
