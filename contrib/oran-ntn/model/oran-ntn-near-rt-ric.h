/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-only
 * Author: Muhammad Uzair
 *
 * Near-RT RIC for NTN - Core controller of the O-RAN NTN architecture
 *
 * Implements Near-RT RIC with NTN extensions per O-RAN architecture:
 *   - xApp lifecycle management (register, start, stop, suspend)
 *   - E2 termination for satellite and terrestrial nodes
 *   - A1 adapter for Non-RT RIC policy ingestion
 *   - Conflict mitigation for multi-xApp scenarios
 *   - Space RIC coordination for on-orbit intelligence
 *   - Database (SDL) for shared xApp data
 *
 * Loop timing:
 *   - Near-RT RIC control loop: 10ms - 1s
 *   - NTN-extended loop: up to 5s (accounting for feeder link delay)
 */

#ifndef ORAN_NTN_NEAR_RT_RIC_H
#define ORAN_NTN_NEAR_RT_RIC_H

#include "oran-ntn-a1-interface.h"
#include "oran-ntn-conflict-manager.h"
#include "oran-ntn-e2-interface.h"
#include "oran-ntn-types.h"
#include "oran-ntn-xapp-base.h"

#include <ns3/callback.h>
#include <ns3/event-id.h>
#include <ns3/nstime.h>
#include <ns3/object.h>
#include <ns3/traced-callback.h>

#include <deque>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace ns3
{

// ============================================================================
//  Shared Data Layer (SDL) - Simplified in-memory key-value store
// ============================================================================

/**
 * \brief Shared Data Layer for xApp data exchange
 *
 * Lightweight in-memory store allowing xApps to share state.
 * Keys are namespaced by xApp ID to prevent collisions.
 */
class OranNtnSdl : public Object
{
  public:
    static TypeId GetTypeId();
    OranNtnSdl();
    ~OranNtnSdl() override;

    void Set(const std::string& ns, const std::string& key, double value);
    double Get(const std::string& ns, const std::string& key, double defaultVal = 0.0) const;
    bool Has(const std::string& ns, const std::string& key) const;
    void Delete(const std::string& ns, const std::string& key);

    // Vector operations for ML feature sharing
    void SetVector(const std::string& ns, const std::string& key,
                    const std::vector<double>& vec);
    std::vector<double> GetVector(const std::string& ns, const std::string& key) const;

    // Report storage for xApp cross-referencing
    void StoreLatestKpm(uint32_t gnbId, const E2KpmReport& report);
    E2KpmReport GetLatestKpm(uint32_t gnbId) const;
    std::map<uint32_t, E2KpmReport> GetAllLatestKpm() const;

  protected:
    void DoDispose() override;

  private:
    std::map<std::string, std::map<std::string, double>> m_scalarStore;
    std::map<std::string, std::map<std::string, std::vector<double>>> m_vectorStore;
    std::map<uint32_t, E2KpmReport> m_latestKpm;
};

// ============================================================================
//  Near-RT RIC
// ============================================================================

/**
 * \ingroup oran-ntn
 * \brief Near-RT RIC - Central controller for O-RAN NTN xApps
 *
 * Orchestrates xApp lifecycle, E2 subscription routing, A1 policy
 * distribution, and conflict resolution for NTN scenarios.
 */
class OranNtnNearRtRic : public Object
{
  public:
    static TypeId GetTypeId();
    OranNtnNearRtRic();
    ~OranNtnNearRtRic() override;

    // ---- Initialization ----
    /**
     * \brief Initialize the RIC and all sub-components
     */
    void Initialize();

    // ---- xApp management ----
    /**
     * \brief Register and start an xApp
     * \return xApp ID assigned by RIC
     */
    uint32_t RegisterXapp(Ptr<OranNtnXappBase> xapp);

    /**
     * \brief Deregister an xApp
     */
    void DeregisterXapp(uint32_t xappId);

    /**
     * \brief Get a registered xApp by ID
     */
    Ptr<OranNtnXappBase> GetXapp(uint32_t xappId) const;

    /**
     * \brief Get all registered xApp IDs
     */
    std::vector<uint32_t> GetRegisteredXappIds() const;

    /**
     * \brief Get xApp by name
     */
    Ptr<OranNtnXappBase> GetXappByName(const std::string& name) const;

    // ---- E2 interface ----
    /**
     * \brief Get the E2 termination point
     */
    Ptr<OranNtnE2Termination> GetE2Termination() const;

    /**
     * \brief Register an E2 node (NTN gNB / satellite)
     */
    void ConnectE2Node(Ptr<OranNtnE2Node> node);

    // ---- A1 interface ----
    /**
     * \brief Get the A1 adapter
     */
    Ptr<OranNtnA1Adapter> GetA1Adapter() const;

    /**
     * \brief Receive A1 policy from Non-RT RIC
     */
    void HandleA1Policy(const A1NtnPolicy& policy);

    // ---- RC action processing ----
    /**
     * \brief Process an RC action from an xApp
     *
     * Runs conflict checking, policy compliance, then routes to E2 node.
     * \return true if action was executed
     */
    bool ProcessXappAction(uint32_t xappId, const E2RcAction& action);

    // ---- Conflict manager ----
    Ptr<OranNtnConflictManager> GetConflictManager() const;

    // ---- SDL ----
    Ptr<OranNtnSdl> GetSdl() const;

    // ---- Metrics ----
    struct RicMetrics
    {
        uint32_t totalXapps;
        uint32_t activeXapps;
        uint32_t totalE2Nodes;
        uint32_t totalSubscriptions;
        uint32_t totalActionsProcessed;
        uint32_t totalConflicts;
        uint32_t totalPolicyViolations;
        Time avgActionLatency;
    };

    RicMetrics GetMetrics() const;

    /**
     * \brief Write comprehensive metrics to file
     */
    void WriteMetrics(const std::string& filename) const;

    // ---- Trace sources ----
    TracedCallback<uint32_t, std::string> m_xappRegistered;   //!< xappId, name
    TracedCallback<uint32_t> m_xappDeregistered;
    TracedCallback<E2RcAction, bool> m_actionProcessed;
    TracedCallback<XappConflict> m_conflictResolved;
    TracedCallback<uint32_t, E2KpmReport> m_kpmReceived;

  protected:
    void DoDispose() override;

  private:
    void RouteIndicationToXapps(uint32_t subscriptionId,
                                 const E2KpmReport& report);
    void ControlLoop();

    // Sub-components
    Ptr<OranNtnE2Termination> m_e2Term;
    Ptr<OranNtnA1Adapter> m_a1Adapter;
    Ptr<OranNtnConflictManager> m_conflictMgr;
    Ptr<OranNtnSdl> m_sdl;

    // xApp registry
    std::map<uint32_t, Ptr<OranNtnXappBase>> m_xapps;
    uint32_t m_nextXappId;

    // Control loop
    Time m_controlLoopInterval;
    EventId m_controlLoopEvent;

    // Metrics
    uint32_t m_totalActionsProcessed;
    uint32_t m_totalConflicts;
    uint32_t m_totalPolicyViolations;
    Time m_cumulativeActionLatency;
};

} // namespace ns3

#endif // ORAN_NTN_NEAR_RT_RIC_H
