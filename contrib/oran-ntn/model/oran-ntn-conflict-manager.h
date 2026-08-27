/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-only
 * Author: Muhammad Uzair
 *
 * O-RAN Conflict Manager for NTN
 *
 * Implements multi-xApp conflict detection and resolution per
 * O-RAN.WG3.RICARCH-v04.00 Conflict Mitigation framework.
 *
 * Resolution strategies:
 *   - Priority-based: higher-priority xApp wins
 *   - Temporal: most recent action wins
 *   - Merge: combine non-conflicting sub-actions
 *   - A1-guided: A1 policy determines winner
 *   - ML-based: learned conflict resolution model
 *
 * NTN-specific conflict scenarios:
 *   - Beam-handover conflict (beam xApp vs HO xApp on same UE)
 *   - Slice-power conflict (slice xApp vs power xApp on same beam)
 *   - Steering-HO conflict (TN-NTN steering vs satellite HO)
 *   - Doppler-beam conflict (frequency compensation vs beam switch)
 */

#ifndef ORAN_NTN_CONFLICT_MANAGER_H
#define ORAN_NTN_CONFLICT_MANAGER_H

#include "oran-ntn-types.h"

#include <ns3/nstime.h>
#include <ns3/object.h>
#include <ns3/traced-callback.h>

#include <deque>
#include <map>
#include <set>
#include <string>
#include <vector>

namespace ns3
{

// ============================================================================
//  Conflict Detection & Resolution
// ============================================================================

/**
 * \brief Resolution strategy for xApp conflicts
 */
enum class ConflictResolutionStrategy : uint8_t
{
    PRIORITY_BASED = 0,    //!< Higher priority xApp wins
    TEMPORAL = 1,          //!< Most recent action wins
    MERGE = 2,             //!< Combine compatible actions
    A1_GUIDED = 3,         //!< A1 policy determines resolution
    ML_BASED = 4,          //!< Learned resolution model
};

/**
 * \brief Pending action in the conflict checking queue
 */
struct PendingAction
{
    uint32_t xappId;
    uint8_t xappPriority;
    E2RcAction action;
    Time submissionTime;
    bool processed;
};

/**
 * \ingroup oran-ntn
 * \brief Multi-xApp conflict detection and resolution
 *
 * Maintains a sliding window of recent actions and detects conflicts
 * when multiple xApps attempt to control the same resource (beam, PRB,
 * UE, power) within the same time window.
 */
class OranNtnConflictManager : public Object
{
  public:
    static TypeId GetTypeId();
    OranNtnConflictManager();
    ~OranNtnConflictManager() override;

    // ---- Configuration ----
    void SetResolutionStrategy(ConflictResolutionStrategy strategy);
    ConflictResolutionStrategy GetResolutionStrategy() const;

    /**
     * \brief Set the conflict detection window
     *
     * Actions within this time window targeting the same resource
     * are checked for conflicts.
     */
    void SetConflictWindow(Time window);

    // ---- Conflict checking ----
    /**
     * \brief Check if a proposed action conflicts with recent actions
     *
     * \param xappId the xApp submitting the action
     * \param xappPriority the xApp's priority level
     * \param action the proposed RC action
     * \return true if the action is allowed (no conflict or won resolution)
     */
    bool CheckAndResolve(uint32_t xappId, uint8_t xappPriority,
                          const E2RcAction& action);

    // ---- Conflict history ----
    std::vector<XappConflict> GetRecentConflicts(Time window) const;
    uint32_t GetTotalConflicts() const;

    /**
     * \brief Get conflict statistics per xApp pair
     */
    std::map<std::pair<uint32_t, uint32_t>, uint32_t> GetConflictMatrix() const;

    // ---- Resource lock management ----
    /**
     * \brief Explicitly lock a resource for an xApp (prevents others)
     */
    /// ORAN-08: supply the active A1 policies so A1_GUIDED can actually
    /// consult them.
    ///
    /// Without this the A1_GUIDED arm was byte-identical to PRIORITY_BASED -
    /// five advertised strategies were three behaviors. A callback rather than a
    /// direct dependency keeps the conflict manager independent of the A1
    /// interface object; the RIC wires them together.
    void SetA1PolicySource(Callback<std::vector<A1Policy>> src) { m_a1Source = src; }

    void LockResource(uint32_t xappId, const std::string& resourceType,
                       uint32_t resourceId, Time duration);

    /**
     * \brief Release a locked resource
     */
    void UnlockResource(const std::string& resourceType, uint32_t resourceId);

    /**
     * \brief Check if a resource is locked by another xApp
     */
    bool IsResourceLocked(const std::string& resourceType, uint32_t resourceId,
                           uint32_t requestingXappId) const;

    // ---- Metrics ----
    void WriteConflictLog(const std::string& filename) const;

    /**
     * \brief Classify a pair of RC actions into the WG3 conflict taxonomy
     *        (Roadmap §4.1.10). Returns UNKNOWN if the pair isn't a conflict
     *        per any of the three categories.
     */
    static ConflictType ClassifyConflict(const E2RcAction& a,
                                          const E2RcAction& b);

    TracedCallback<XappConflict> m_conflictDetected;
    TracedCallback<XappConflict> m_conflictResolved;

  protected:
    void DoDispose() override;

  private:
    bool DetectConflict(const PendingAction& newAction,
                         const PendingAction& existing) const;
    XappConflict ResolveConflict(const PendingAction& a1,
                                  const PendingAction& a2) const;
    std::string GetResourceKey(const E2RcAction& action) const;
    void PruneOldActions();

    // ns-3 attribute accessor for "ResolutionStrategy": maps the uint attribute
    // directly onto m_strategy (the value the resolver actually switches on).
    void SetStrategyValue(uint8_t v);
    uint8_t GetStrategyValue() const;

    ConflictResolutionStrategy m_strategy;
    Time m_conflictWindow;

    // Recent actions indexed by resource key
    /// ORAN-08: secondary index keys. Detection used to compare only actions
    /// whose resource key matched byte for byte, which made the IMPLICIT and
    /// PRB INDIRECT branches of the WG3 taxonomy unreachable through the real
    /// pipeline. These let cross-parameter and cross-family pairs meet.
    const A1Policy* MatchPolicy(const E2RcAction& action) const;
    static std::string FamilyIndexKey(const E2RcAction& action);
    static std::string UeIndexKey(const E2RcAction& action);

    Callback<std::vector<A1Policy>> m_a1Source;
    std::map<std::string, std::deque<PendingAction>> m_recentActions;

    // Resource locks: resourceKey -> (xappId, expiryTime)
    struct ResourceLock
    {
        uint32_t xappId;
        Time expiryTime;
    };
    std::map<std::string, ResourceLock> m_resourceLocks;

    // Conflict log
    std::deque<XappConflict> m_conflictLog;
    uint32_t m_totalConflicts;
    uint32_t m_maxLogSize;
};

} // namespace ns3

#endif // ORAN_NTN_CONFLICT_MANAGER_H
