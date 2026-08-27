/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-only
 * Author: Muhammad Uzair
 *
 * Space RIC - On-orbit lightweight RIC for autonomous NTN decisions
 *
 * Based on Space-O-RAN architecture (IEEE CommMag 2026) and
 * O-RAN NTN White Paper (April 2025). Implements:
 *   - On-board dApp (distributed App) execution
 *   - Autonomous handover/beam decisions during feeder link outage
 *   - ISL-based coordination with neighboring Space RICs
 *   - Model synchronization with ground Near-RT RIC
 *   - Federated learning aggregation across orbital plane
 *
 * Architecture (intent vs. implementation):
 *   Space RIC runs on satellite on-board processor (OBP) with limited
 *   compute/memory — the design slot for lightweight on-board ML
 *   (e.g. quantized DQN / decision trees) vs. full models on ground.
 *   As implemented, `OranNtnSpaceRicInference` ships no such trained model:
 *   its backends are an optional LibTorch engine (no model shipped) and a
 *   default rule-based path. The ML descriptors here are the roadmap target,
 *   not the running inference.
 */

#ifndef ORAN_NTN_SPACE_RIC_H
#define ORAN_NTN_SPACE_RIC_H

#include "oran-ntn-isl-header.h"
#include "oran-ntn-msg-interface.h"
#include "oran-ntn-types.h"

#include <ns3/event-id.h>
#include <ns3/nstime.h>
#include <ns3/object.h>
#include <ns3/packet.h>
#include <ns3/traced-callback.h>

#include <deque>
#include <map>
#include <string>
#include <vector>

namespace ns3
{

class OranNtnNearRtRic;
class OranNtnSpaceRicInference;
class OranNtnSatBridge;
class OranNtnE2Node;

// ============================================================================
//  dApp (Distributed App) - lightweight on-satellite xApp
// ============================================================================

/**
 * \brief dApp decision record
 */
struct DappDecision
{
    double timestamp;
    uint32_t dappId;
    std::string dappName;
    E2RcActionType actionType;
    uint32_t targetUeId;
    uint32_t targetBeamId;
    double confidence;
    bool autonomous;               //!< true = no ground RIC involved
    std::string rationale;
};

/**
 * \brief Model synchronization record
 */
struct ModelSyncRecord
{
    double timestamp;
    uint32_t satId;
    std::string modelName;
    uint32_t modelVersion;
    double modelSizeKB;
    Time syncLatency;
    bool viaIsl;                   //!< Synced via ISL or feeder link
};

// ============================================================================
//  Space RIC
// ============================================================================

/**
 * \ingroup oran-ntn
 * \brief On-orbit Space RIC for autonomous NTN decisions
 *
 * Runs on satellite OBP. Provides autonomous control when feeder link
 * is unavailable, and optimized local decisions even when ground
 * RIC is reachable (reducing control loop latency).
 */
class OranNtnSpaceRic : public Object
{
  public:
    static TypeId GetTypeId();
    OranNtnSpaceRic();
    ~OranNtnSpaceRic() override;

    // ---- Configuration ----
    void SetSatelliteId(uint32_t satId);
    void SetOrbitalPlaneId(uint32_t planeId);
    void SetGroundRic(Ptr<OranNtnNearRtRic> ric);

    /**
     * \brief Set the co-located (on-board) E2 node used to ACTUATE autonomous
     *        decisions locally.
     *
     * An on-board Space RIC's whole purpose is to close the loop without a
     * feeder-link round trip: its autonomous HO / beam decisions are actuated
     * through this co-located E2 node (which models the on-board actuation path,
     * i.e. near-zero feeder delay) instead of being buffered until the ground
     * link returns. Without it, AutonomousHandoverDecision / AutonomousBeam
     * Reallocation only buffer and NOTHING executes.
     */
    void SetLocalE2Node(Ptr<OranNtnE2Node> e2Node);

    /// ORAN-02: the co-located E2 node, or nullptr when the space RIC was never
    /// paired with one (in which case every autonomous decision is buffered
    /// rather than actuated). Exposed so a test can assert the pairing.
    Ptr<OranNtnE2Node> GetLocalE2Node() const { return m_localE2Node; }

    void SetComputeBudget(double mflops);       //!< OBP compute limit
    void SetMemoryBudget(double megabytes);     //!< OBP memory limit

    uint32_t GetSatelliteId() const;
    uint32_t GetOrbitalPlaneId() const;

    // ---- Autonomous mode ----
    /**
     * \brief Enter autonomous mode (feeder link lost)
     *
     * Space RIC takes over all decisions locally using on-board models.
     */
    void EnterAutonomousMode();

    /**
     * \brief Exit autonomous mode (feeder link restored)
     *
     * Syncs buffered decisions and metrics back to ground RIC,
     * resumes subordinate operation.
     */
    void ExitAutonomousMode();

    bool IsAutonomous() const;

    // ---- Local decision making ----
    /**
     * \brief Process local KPM data and make beam/HO decisions
     *
     * Uses lightweight models for:
     *   - Intra-satellite beam handover
     *   - Beam power adjustment
     *   - Doppler compensation update
     */
    void ProcessLocalKpm(const E2KpmReport& report);

    /**
     * \brief Autonomous handover decision
     *
     * Triggered when serving beam quality drops and ground RIC
     * is unreachable. Uses on-board TTE model.
     */
    DappDecision AutonomousHandoverDecision(uint32_t ueId,
                                              const E2KpmReport& serving,
                                              const std::vector<E2KpmReport>& candidates);

    /**
     * \brief Autonomous beam reallocation
     *
     * Adjusts beam allocation based on local traffic observations
     * when ground beam hopping schedule cannot be updated.
     */
    DappDecision AutonomousBeamReallocation(
        const std::vector<BeamAllocationEntry>& currentSchedule,
        const std::map<uint32_t, double>& beamLoads);

    // ---- ISL coordination (enhanced with real packet exchange) ----

    /**
     * \brief Set satellite bridge for orbital queries
     */
    void SetSatBridge(Ptr<OranNtnSatBridge> bridge);

    /**
     * \brief Set inference engine for AI-assisted decisions
     */
    void SetInferenceEngine(Ptr<OranNtnSpaceRicInference> engine);

    /**
     * \brief Add ISL neighbor with direct pointer (legacy mode)
     */
    void AddIslNeighbor(Ptr<OranNtnSpaceRic> neighbor);
    /// ORAN-16: how many ISL neighbours this RIC actually has.
    ///
    /// The helper computed neighbour indices and stored none of them, so every
    /// shipped scenario ran a shell of isolated on-board RICs. Nothing outside
    /// could see that, because the list was private and no accessor existed.
    uint32_t GetIslNeighborCount() const { return static_cast<uint32_t>(m_islNeighbors.size()); }

    /**
     * \brief Receive ISL packet (callback for real ISL devices)
     */
    void ReceiveIslPacket(Ptr<Packet> packet);

    /**
     * \brief Send data to neighbor via ISL with packet serialization
     */
    void SendIslMessage(uint32_t neighborSatId, IslMessageType msgType,
                         const std::vector<uint8_t>& payload);

    /**
     * \brief Exchange state with neighboring Space RIC via ISL
     *
     * Shares: beam loads, UE handover candidates, model gradients
     */
    void IslExchangeState(Ptr<OranNtnSpaceRic> neighbor);

    /**
     * \brief Coordinate inter-satellite handover via ISL
     */
    bool CoordinateInterSatHandover(uint32_t ueId,
                                      Ptr<OranNtnSpaceRic> targetSpaceRic);

    // ---- Model management ----
    /**
     * \brief Receive model update from ground RIC
     */
    void ReceiveModelUpdate(const std::string& modelName,
                             uint32_t version,
                             const std::vector<double>& weights);

    /**
     * \brief Send local model gradients for federated aggregation
     */
    std::vector<double> GetLocalGradients(const std::string& modelName) const;

    /**
     * \brief Get current model version
     */
    uint32_t GetModelVersion(const std::string& modelName) const;

    // ---- Metrics ----
    struct SpaceRicMetrics
    {
        uint32_t totalAutonomousDecisions;
        uint32_t totalGroundAssistedDecisions;
        Time totalAutonomousTime;
        uint32_t islExchanges;
        uint32_t modelSyncs;
        double avgDecisionConfidence;
        uint32_t handoversInitiated;
        uint32_t beamReallocations;
    };

    SpaceRicMetrics GetMetrics() const;

    // ---- Decision log ----
    std::vector<DappDecision> GetDecisionLog() const;
    std::vector<ModelSyncRecord> GetSyncLog() const;

    // ---- Trace sources ----
    TracedCallback<uint32_t, DappDecision> m_autonomousDecision;
    TracedCallback<uint32_t, bool> m_autonomousModeChanged;
    TracedCallback<uint32_t, uint32_t> m_islExchange;  //!< thisSat, neighborSat
    TracedCallback<ModelSyncRecord> m_modelSynced;

  protected:
    void DoDispose() override;

  private:
    void AutonomousControlLoop();
    void SyncWithGroundRic();

    /**
     * \brief Actuate a buffered autonomous decision on the co-located E2 node.
     * \return true if a local E2 node was available and the action was delivered.
     */
    bool ExecuteDecisionLocally(const DappDecision& decision);

    uint32_t m_satId;
    uint32_t m_planeId;
    Ptr<OranNtnNearRtRic> m_groundRic;
    Ptr<OranNtnE2Node> m_localE2Node; //!< co-located on-board E2 actuation path
    bool m_autonomous;

    // Compute constraints
    double m_computeBudgetMflops;
    double m_memoryBudgetMB;

    // On-board models (simplified weight vectors)
    struct OnBoardModel
    {
        std::string name;
        uint32_t version;
        std::vector<double> weights;
        double sizeKB;
    };
    std::map<std::string, OnBoardModel> m_models;

    // Local state
    std::map<uint32_t, E2KpmReport> m_localKpm;        //!< Latest per-UE KPM
    std::map<uint32_t, double> m_beamLoads;             //!< per-beam traffic load

    // Autonomous decision buffer (to sync when ground link returns)
    std::deque<DappDecision> m_decisionBuffer;
    std::deque<ModelSyncRecord> m_syncLog;

    // Control loop
    Time m_autonomousLoopInterval;
    EventId m_autonomousLoopEvent;

    // ISL neighbors
    std::vector<Ptr<OranNtnSpaceRic>> m_islNeighbors;

    // Deep integration (Phase 1, 3, 5)
    Ptr<OranNtnSatBridge> m_satBridge;
    Ptr<OranNtnSpaceRicInference> m_inferenceEngine;
    uint32_t m_islSequenceNumber;

    // Energy state awareness
    double m_batteryLevel;           //!< Current battery SoC (0-1)
    double m_solarPower_W;           //!< Current solar panel output

    // Metrics
    SpaceRicMetrics m_metrics;
};

} // namespace ns3

#endif // ORAN_NTN_SPACE_RIC_H
