/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-only
 * Author: Muhammad Uzair
 *
 * Federated Learning Coordinator for O-RAN NTN
 *
 * Coordinates federated learning between Space RICs and ground RIC.
 * Supports multiple aggregation strategies optimized for NTN constraints:
 *
 * Novel features:
 *   - Orbit-aware participant selection (select satellites with good feeder links)
 *   - ISL-relay gradient forwarding for satellites without direct ground contact
 *   - Asynchronous FL with staleness-weighted aggregation
 *   - Communication-efficient gradient compression (top-k sparsification)
 *   - Heterogeneous model support (different model sizes per satellite OBP)
 *   - Eclipse-aware round scheduling (avoid FL during battery-critical periods)
 */

#ifndef ORAN_NTN_FEDERATED_LEARNING_H
#define ORAN_NTN_FEDERATED_LEARNING_H

#include "oran-ntn-types.h"

#include <ns3/event-id.h>
#include <ns3/nstime.h>
#include <ns3/object.h>
#include <ns3/traced-callback.h>

#include <map>
#include <vector>

namespace ns3
{

class OranNtnSpaceRic;
class OranNtnSatBridge;

/**
 * \ingroup oran-ntn
 * \brief Federated learning coordinator for Space-O-RAN
 */
class OranNtnFederatedLearning : public Object
{
  public:
    static TypeId GetTypeId();
    OranNtnFederatedLearning();
    ~OranNtnFederatedLearning() override;

    // ---- Configuration ----
    void SetSatBridge(Ptr<OranNtnSatBridge> bridge);
    void SetMinParticipants(uint32_t n);
    void SetRoundInterval(Time interval);
    void SetAggregationStrategy(const std::string& strategy);
    void SetGradientCompression(bool enable, double topKRatio = 0.1);
    void SetAsynchronous(bool enable, double stalenessDecay = 0.9);
    void SetEclipseAware(bool enable);

    // ---- Participant management ----
    void RegisterSpaceRic(Ptr<OranNtnSpaceRic> spaceRic);
    void UnregisterSpaceRic(uint32_t satId);

    // ---- FL round lifecycle ----

    /**
     * \brief Initialize a new federated learning round
     */
    void InitializeFederatedRound(const std::string& modelName);

    /**
     * \brief Collect local gradients from a Space RIC
     */
    void CollectLocalGradients(uint32_t satId, const std::string& modelName,
                                const std::vector<double>& gradients,
                                double localLoss, uint32_t localSamples);

    /**
     * \brief Aggregate collected gradients
     *
     * Supports: FedAvg, FedProx, FedNova, SCAFFOLD
     */
    void AggregateGradients();

    /**
     * \brief Distribute global model to all participants
     */
    void DistributeGlobalModel();

    /**
     * \brief Start periodic FL rounds
     */
    void StartPeriodicRounds(const std::string& modelName);

    /**
     * \brief Stop periodic rounds
     */
    void StopPeriodicRounds();

    // ---- State queries ----
    FederatedLearningRound GetCurrentRound() const;
    std::vector<FederatedLearningRound> GetRoundHistory() const;
    uint32_t GetGlobalModelVersion(const std::string& modelName) const;

    // ---- Metrics ----
    struct FLMetrics
    {
        uint32_t totalRounds;
        uint32_t completedRounds;
        uint32_t failedRounds;
        double avgParticipationRate;
        double avgRoundDuration_s;
        double avgConvergenceRate;
        double globalLoss;
        uint32_t totalGradientsExchanged;
        double totalCommsBytes;
    };
    FLMetrics GetMetrics() const;

    // ---- Trace sources ----
    TracedCallback<uint32_t, std::string> m_roundStarted;   //!< roundId, modelName
    TracedCallback<uint32_t, double> m_roundCompleted;       //!< roundId, globalLoss
    TracedCallback<uint32_t, uint32_t> m_gradientReceived;   //!< roundId, satId

  protected:
    void DoDispose() override;

  private:
    void PeriodicRoundCallback();

    /**
     * \brief FedAvg: weighted average by number of local samples
     */
    std::vector<double> FedAvgAggregate(
        const std::vector<std::pair<std::vector<double>, uint32_t>>& gradients) const;

    /**
     * \brief FedProx: FedAvg + proximal term for heterogeneity
     */
    std::vector<double> FedProxAggregate(
        const std::vector<std::pair<std::vector<double>, uint32_t>>& gradients,
        double mu) const;

    /**
     * \brief FedNova: normalized averaging for varying local epochs
     */
    std::vector<double> FedNovaAggregate(
        const std::vector<std::pair<std::vector<double>, uint32_t>>& gradients) const;

    /**
     * \brief Apply top-k gradient sparsification for compression
     */
    std::vector<double> CompressGradients(const std::vector<double>& gradients) const;

    /**
     * \brief Select participants (orbit-aware, eclipse-aware)
     */
    std::vector<uint32_t> SelectParticipants() const;

    // Configuration
    Ptr<OranNtnSatBridge> m_satBridge;
    uint32_t m_minParticipants;
    Time m_roundInterval;
    std::string m_aggregationStrategy;
    bool m_gradientCompression;
    double m_topKRatio;
    bool m_asynchronous;
    double m_stalenessDecay;
    bool m_eclipseAware;

    // Participants
    std::map<uint32_t, Ptr<OranNtnSpaceRic>> m_spaceRics;

    // Current round state
    FederatedLearningRound m_currentRound;
    uint32_t m_nextRoundId;
    std::string m_currentModelName;

    // Collected gradients for current round
    struct GradientEntry
    {
        uint32_t satId;
        std::vector<double> gradients;
        double localLoss;
        uint32_t localSamples;
        double timestamp;
    };
    std::vector<GradientEntry> m_collectedGradients;

    // Global model
    std::map<std::string, std::vector<double>> m_globalModels;
    std::map<std::string, uint32_t> m_globalModelVersions;

    // Round history
    std::vector<FederatedLearningRound> m_roundHistory;

    // Periodic rounds
    EventId m_periodicRoundEvent;
    bool m_periodicActive;

    // Metrics
    FLMetrics m_flMetrics;
};

} // namespace ns3

#endif // ORAN_NTN_FEDERATED_LEARNING_H
