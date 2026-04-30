/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-only
 * Author: Muhammad Uzair
 *
 * TN-NTN Traffic Steering xApp - Terrestrial/Satellite traffic routing
 *
 * Steers UE traffic between terrestrial (TN) and non-terrestrial (NTN)
 * networks based on coverage, capacity, latency, and QoS requirements.
 * Implements multi-criteria decision making with A1 policy guidance
 * for optimal network selection.
 *
 * Scenarios:
 *   - Urban: TN primary, NTN for coverage gaps / overload
 *   - Rural: NTN primary, TN when available
 *   - Maritime/Aviation: NTN only
 *   - Emergency: NTN backup when TN infrastructure fails
 */

#ifndef ORAN_NTN_XAPP_TN_NTN_STEERING_H
#define ORAN_NTN_XAPP_TN_NTN_STEERING_H

#include "oran-ntn-xapp-base.h"

#include <map>
#include <vector>

namespace ns3
{

/**
 * \brief UE network selection state
 */
enum class NetworkSelection : uint8_t
{
    TERRESTRIAL = 0,
    SATELLITE = 1,
    DUAL_CONNECTED = 2,    //!< Simultaneous TN+NTN (split bearer)
};

/**
 * \ingroup oran-ntn
 * \brief TN-NTN Traffic Steering xApp
 */
class OranNtnXappTnNtnSteering : public OranNtnXappBase
{
  public:
    static TypeId GetTypeId();
    OranNtnXappTnNtnSteering();
    ~OranNtnXappTnNtnSteering() override;

    // ---- Configuration ----
    void SetSteeringMode(const std::string& mode);
        // "latency-optimal", "throughput-optimal", "reliability-optimal", "cost-optimal"
    void SetTnPreferenceFactor(double factor);  //!< 0-1, higher = prefer TN
    void SetNtnLatencyPenalty(double ms);        //!< Extra latency from NTN
    void SetMinSinrForTn(double dB);
    void SetMinSinrForNtn(double dB);

    // ---- Query ----
    NetworkSelection GetUeNetworkSelection(uint32_t ueId) const;
    double GetTnTrafficFraction() const;
    double GetNtnTrafficFraction() const;

    // ---- Metrics ----
    struct SteeringMetrics
    {
        uint32_t tnToNtnSwitches;
        uint32_t ntnToTnSwitches;
        uint32_t dualConnSetups;
        double avgTnUtilization;
        double avgNtnUtilization;
        double avgSteeredLatency_ms;
        double avgSteeredThroughput_Mbps;
        uint32_t coverageGapEvents;
    };

    SteeringMetrics GetSteeringMetrics() const;

    TracedCallback<uint32_t, uint8_t, uint8_t> m_steeringDecision;
    //!< ueId, oldNetwork, newNetwork

  protected:
    void ProcessKpmReport(const E2KpmReport& report) override;
    void DecisionCycle() override;
    E2Subscription GetRequiredSubscription() const override;

  private:
    struct UeSteeringState
    {
        NetworkSelection currentNetwork = NetworkSelection::SATELLITE; // NTN default
        double tnSinr = -999.0;     // Mark as no-data initially
        double ntnSinr = -999.0;
        double tnLoad = 0.0;
        double ntnLoad = 0.0;
        double tnThroughput = 0.0;
        double ntnThroughput = 0.0;
        uint8_t sliceId = 0;
        Time lastSwitchTime = Time(0);
    };

    NetworkSelection ComputeOptimalNetwork(uint32_t ueId,
                                            const UeSteeringState& state) const;
    double ComputeSteeringScore(const UeSteeringState& state,
                                 NetworkSelection candidate) const;

    std::map<uint32_t, UeSteeringState> m_ueSteeringStates;

    // Configuration
    std::string m_steeringMode;
    double m_tnPreferenceFactor;
    double m_ntnLatencyPenalty;
    double m_minSinrTn;
    double m_minSinrNtn;

    SteeringMetrics m_steeringMetrics;
};

} // namespace ns3

#endif // ORAN_NTN_XAPP_TN_NTN_STEERING_H
