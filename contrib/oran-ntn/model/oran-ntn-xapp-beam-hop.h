/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-only
 * Author: Muhammad Uzair
 *
 * Beam Hopping xApp - Dynamic beam scheduling for NTN satellites
 *
 * Optimizes beam-time-slot allocation across satellite beams based on
 * traffic demand, interference, and energy constraints. Implements
 * both heuristic (water-filling) and AI-based (DRL) scheduling.
 *
 * Per O-RAN.WG4.IOT.0 and 3GPP TS 38.821 beam management procedures.
 */

#ifndef ORAN_NTN_XAPP_BEAM_HOP_H
#define ORAN_NTN_XAPP_BEAM_HOP_H

#include "oran-ntn-xapp-base.h"

#include <map>
#include <vector>

namespace ns3
{

/**
 * \ingroup oran-ntn
 * \brief Beam Hopping xApp for dynamic NTN beam scheduling
 */
class OranNtnXappBeamHop : public OranNtnXappBase
{
  public:
    static TypeId GetTypeId();
    OranNtnXappBeamHop();
    ~OranNtnXappBeamHop() override;

    // ---- Configuration ----
    void SetNumBeams(uint32_t beams);
    void SetNumTimeSlots(uint32_t slots);
    void SetHoppingFrameLength(Time frameLen);
    void SetMaxSimultaneousBeams(uint32_t maxBeams);
    void SetSchedulingAlgorithm(const std::string& algo); // "water-fill", "proportional", "drl"

    // ---- Output ----
    /**
     * \brief Get the current beam allocation matrix
     */
    std::vector<BeamAllocationEntry> GetCurrentSchedule() const;

    // ---- Metrics ----
    struct BeamHopMetrics
    {
        uint32_t schedulingRounds;
        double avgBeamUtilization;
        double avgDemandSatisfaction;
        double avgEnergyEfficiency;
        uint32_t beamSwitches;
        double fairnessIndex;          //!< Jain's fairness index
    };

    BeamHopMetrics GetBeamHopMetrics() const;

    TracedCallback<uint32_t, std::vector<BeamAllocationEntry>> m_scheduleUpdated;

  protected:
    void ProcessKpmReport(const E2KpmReport& report) override;
    void DecisionCycle() override;
    E2Subscription GetRequiredSubscription() const override;

  private:
    // Scheduling algorithms
    std::vector<BeamAllocationEntry> WaterFillingSchedule() const;
    std::vector<BeamAllocationEntry> ProportionalSchedule() const;
    std::vector<BeamAllocationEntry> DrlSchedule() const;

    double ComputeJainsFairness(const std::vector<double>& allocations) const;

    // Per-beam state
    struct BeamState
    {
        uint32_t beamId;
        double trafficDemand;         //!< Normalized demand (0-1)
        double currentLoad;           //!< PRB utilization
        uint32_t activeUes;
        double avgSinr;
        double interferenceLevel;
        uint32_t allocatedSlots;
    };

    std::map<uint32_t, BeamState> m_beamStates; //!< Per satellite
    std::vector<BeamAllocationEntry> m_currentSchedule;

    // Configuration
    uint32_t m_numBeams;
    uint32_t m_numTimeSlots;
    Time m_hoppingFrameLength;
    uint32_t m_maxSimultaneousBeams;
    std::string m_schedulingAlgorithm;

    // DRL weights
    std::vector<double> m_drlWeights;

    // Metrics
    BeamHopMetrics m_bhMetrics;
};

} // namespace ns3

#endif // ORAN_NTN_XAPP_BEAM_HOP_H
