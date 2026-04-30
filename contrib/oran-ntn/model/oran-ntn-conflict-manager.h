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

    ConflictResolutionStrategy m_strategy;
    uint8_t m_strategyVal;  //!< For ns3 attribute binding
    Time m_conflictWindow;

    // Recent actions indexed by resource key
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
