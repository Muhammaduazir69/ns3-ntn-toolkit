/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-only
 * Author: Muhammad Uzair
 *
 * Slice Manager xApp - Network slice resource allocation for NTN
 *
 * Manages PRB allocation across eMBB/URLLC/mMTC slices on NTN gNBs.
 * Enforces SLA from A1 policies, handles NTN-specific challenges
 * (variable capacity, high latency), and supports cross-satellite
 * slice continuity during handovers.
 */

#ifndef ORAN_NTN_XAPP_SLICE_MANAGER_H
#define ORAN_NTN_XAPP_SLICE_MANAGER_H

#include "oran-ntn-xapp-base.h"

#include <map>
#include <vector>

namespace ns3
{

/**
 * \ingroup oran-ntn
 * \brief Slice Manager xApp for NTN resource allocation
 */
class OranNtnXappSliceManager : public OranNtnXappBase
{
  public:
    static TypeId GetTypeId();
    OranNtnXappSliceManager();
    ~OranNtnXappSliceManager() override;

    // ---- Configuration ----
    void SetSliceConfigs(const std::vector<SliceConfig>& configs);
    void SetTotalPrbs(uint32_t prbs);
    void SetReallocationInterval(Time interval);

    // ---- Query ----
    /**
     * \brief Get current PRB allocation for each slice at a given gNB
     */
    std::map<uint8_t, double> GetSliceAllocations(uint32_t gnbId) const;

    // ---- Metrics ----
    struct SliceMetrics
    {
        uint32_t reallocationCount;
        std::map<uint8_t, double> avgPrbShare;        //!< Per-slice average
        std::map<uint8_t, double> slaViolationRate;   //!< Per-slice
        std::map<uint8_t, double> avgThroughput;      //!< Per-slice
        std::map<uint8_t, double> avgLatency;         //!< Per-slice
        double overallSlaCompliance;
    };

    SliceMetrics GetSliceMetrics() const;

    TracedCallback<uint32_t, uint8_t, double> m_sliceAllocChanged;
    //!< gnbId, sliceId, newPrbShare

  protected:
    void ProcessKpmReport(const E2KpmReport& report) override;
    void DecisionCycle() override;
    E2Subscription GetRequiredSubscription() const override;

  private:
    struct PerGnbSliceState
    {
        std::map<uint8_t, double> currentPrbShares;     //!< sliceId -> share
        std::map<uint8_t, double> demandEstimate;       //!< sliceId -> demand
        std::map<uint8_t, double> latestThroughput;
        std::map<uint8_t, double> latestLatency;
        std::map<uint8_t, double> latestReliability;
    };

    void ReallocateSlices(uint32_t gnbId);
    bool CheckSlaCompliance(uint8_t sliceId, const PerGnbSliceState& state) const;

    std::map<uint32_t, PerGnbSliceState> m_gnbSliceStates;
    std::vector<SliceConfig> m_sliceConfigs;

    uint32_t m_totalPrbs;
    Time m_reallocationInterval;

    // Metrics
    SliceMetrics m_sliceMetrics;
};

} // namespace ns3

#endif // ORAN_NTN_XAPP_SLICE_MANAGER_H
