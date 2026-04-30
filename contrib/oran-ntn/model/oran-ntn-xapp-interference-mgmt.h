/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-only
 * Author: Muhammad Uzair
 *
 * Interference Management xApp
 *
 * Coordinated inter-beam and TN-NTN interference mitigation using
 * satellite antenna gain patterns, per-packet interference calculations,
 * and mmWave interference models.
 *
 * Novel features:
 *   - Interference graph construction from multi-satellite beam overlaps
 *   - Graph-coloring-based beam frequency assignment
 *   - Adaptive power control with interference temperature constraint
 *   - Cross-layer null steering coordination via O-RAN RC actions
 *   - Predictive interference avoidance using orbit-based beam overlap forecast
 *   - Joint TN-NTN interference management for co-channel scenarios
 */

#ifndef ORAN_NTN_XAPP_INTERFERENCE_MGMT_H
#define ORAN_NTN_XAPP_INTERFERENCE_MGMT_H

#include "oran-ntn-xapp-base.h"

#include <map>
#include <set>
#include <vector>

namespace ns3
{

class OranNtnSatBridge;

/**
 * \ingroup oran-ntn
 * \brief Interference Management xApp for NTN
 */
class OranNtnXappInterferenceMgmt : public OranNtnXappBase
{
  public:
    static TypeId GetTypeId();
    OranNtnXappInterferenceMgmt();
    ~OranNtnXappInterferenceMgmt() override;

    void SetSatBridge(Ptr<OranNtnSatBridge> bridge);
    void SetInterferenceThreshold(double threshold_dBm);
    void SetPowerControlEnabled(bool enable);
    void SetNullSteeringEnabled(bool enable);
    void SetGraphColoringEnabled(bool enable);
    void SetPredictiveEnabled(bool enable);

    // ---- Interference graph access ----
    struct InterferenceEdge
    {
        uint32_t sat1;
        uint32_t beam1;
        uint32_t sat2;
        uint32_t beam2;
        double interference_dBm;
        double couplingLoss_dB;
    };

    std::vector<InterferenceEdge> GetInterferenceGraph() const;
    std::vector<InterferenceMapEntry> GetInterferenceMap() const;

    // ---- Metrics ----
    struct InterferenceMgmtMetrics
    {
        uint32_t powerControlActions;
        uint32_t nullSteeringActions;
        uint32_t frequencyReassignments;
        double avgInterference_dBm;
        double maxInterference_dBm;
        double interferenceReduction_dB;
        uint32_t interferenceEventsDetected;
        uint32_t interferenceEventsMitigated;
    };
    InterferenceMgmtMetrics GetInterferenceMgmtMetrics() const;

  protected:
    void ProcessKpmReport(const E2KpmReport& report) override;
    void DecisionCycle() override;
    E2Subscription GetRequiredSubscription() const override;

  private:
    void BuildInterferenceGraph();
    void ApplyPowerControl();
    void ApplyNullSteering();
    void ApplyGraphColoring();
    void PredictFutureInterference();

    /**
     * \brief Compute inter-beam coupling loss from antenna patterns
     */
    double ComputeCouplingLoss(uint32_t sat1, uint32_t beam1,
                                uint32_t sat2, uint32_t beam2) const;

    /**
     * \brief Graph coloring for frequency assignment
     */
    std::map<uint32_t, uint32_t> GraphColorBeams(
        const std::vector<InterferenceEdge>& graph,
        uint32_t numColors) const;

    Ptr<OranNtnSatBridge> m_satBridge;
    double m_interferenceThreshold_dBm;
    bool m_powerControlEnabled;
    bool m_nullSteeringEnabled;
    bool m_graphColoringEnabled;
    bool m_predictiveEnabled;

    std::vector<InterferenceEdge> m_interferenceGraph;
    std::map<uint64_t, InterferenceMapEntry> m_interferenceMap;

    InterferenceMgmtMetrics m_ifMetrics;
};

} // namespace ns3

#endif // ORAN_NTN_XAPP_INTERFERENCE_MGMT_H
