/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-only
 * Author: Muhammad Uzair
 */

#include "oran-ntn-conflict-manager.h"

#include <ns3/log.h>
#include <ns3/simulator.h>
#include <ns3/uinteger.h>

#include <fstream>
#include <sstream>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("OranNtnConflictManager");

// ORAN-08: hoisted above its first use so the secondary index keys can
// classify by resource family.
enum class ResourceFamily : uint8_t
{
    PRB,
    BEAM,
    POWER,
    HANDOVER,
    TIMING,
    MCS,
    THZ,
    SLICE,
    AI_ML,
    OTHER,
};

ResourceFamily
FamilyOf(E2RcActionType t)
{
    switch (t)
    {
    case E2RcActionType::SLICE_PRB_ALLOCATION:
    case E2RcActionType::PRB_RESERVATION:
        return ResourceFamily::PRB;
    case E2RcActionType::BEAM_SWITCH:
    case E2RcActionType::BEAM_HOP_SCHEDULE:
    case E2RcActionType::BEAM_SHUTDOWN:
    case E2RcActionType::INTERFERENCE_NULLING:
        return ResourceFamily::BEAM;
    case E2RcActionType::TX_POWER_CONTROL:
    case E2RcActionType::ENERGY_PROFILE_UPDATE:
    case E2RcActionType::COMPUTE_THROTTLE:
    case E2RcActionType::ACTION_THZ_POWER_BACKOFF:
        return ResourceFamily::POWER;
    case E2RcActionType::HANDOVER_TRIGGER:
    case E2RcActionType::HANDOVER_CANCEL:
    case E2RcActionType::DC_SETUP:
    case E2RcActionType::DC_TEARDOWN:
    case E2RcActionType::BEARER_SPLIT:
        return ResourceFamily::HANDOVER;
    case E2RcActionType::TIMING_ADVANCE_UPDATE:
    case E2RcActionType::DOPPLER_COMP_UPDATE:
        return ResourceFamily::TIMING;
    case E2RcActionType::MCS_OVERRIDE:
    case E2RcActionType::MODCOD_OVERRIDE:
    case E2RcActionType::CCA_THRESHOLD_ADJUST:
        return ResourceFamily::MCS;
    case E2RcActionType::ACTION_THZ_FREQ_SELECT:
    case E2RcActionType::ACTION_THZ_BEAM_CODEBOOK:
    case E2RcActionType::ACTION_THZ_RIS_CONFIG:
    case E2RcActionType::ACTION_THZ_WAVEFORM_SELECT:
    case E2RcActionType::ACTION_THZ_ISAC_MODE:
    case E2RcActionType::ACTION_THZ_WINDOW_HOP:
        return ResourceFamily::THZ;
    case E2RcActionType::ISL_ROUTE_UPDATE:
    case E2RcActionType::REGEN_MODE_SWITCH:
    case E2RcActionType::FL_MODEL_PUSH:
        return ResourceFamily::AI_ML;
    default:
        return ResourceFamily::OTHER;
    }
}

NS_OBJECT_ENSURE_REGISTERED(OranNtnConflictManager);

TypeId
OranNtnConflictManager::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::OranNtnConflictManager")
            .SetParent<Object>()
            .SetGroupName("OranNtn")
            .AddConstructor<OranNtnConflictManager>()
            .AddAttribute("ResolutionStrategy",
                          "Conflict resolution strategy (0=priority, 1=temporal, "
                          "2=merge, 3=A1-guided, 4=confidence). 4 was advertised as "
                          "'ML' and is a confidence comparison; there is no model.",
                          UintegerValue(0),
                          // Setter/getter accessor so the attribute drives the
                          // SAME m_strategy the resolver switches on. (The old
                          // MakeUintegerAccessor(&m_strategyVal) wrote a member
                          // nothing read, so any attribute-set silently ran
                          // PRIORITY_BASED.)
                          MakeUintegerAccessor(&OranNtnConflictManager::SetStrategyValue,
                                               &OranNtnConflictManager::GetStrategyValue),
                          MakeUintegerChecker<uint8_t>(0, 4))
            .AddAttribute("ConflictWindow",
                          "Time window for conflict detection",
                          TimeValue(MilliSeconds(500)),
                          MakeTimeAccessor(&OranNtnConflictManager::m_conflictWindow),
                          MakeTimeChecker())
            .AddAttribute("MaxLogSize",
                          "Maximum conflict log entries",
                          UintegerValue(10000),
                          MakeUintegerAccessor(&OranNtnConflictManager::m_maxLogSize),
                          MakeUintegerChecker<uint32_t>())
            .AddTraceSource("ConflictDetected",
                            "A conflict between xApps was detected",
                            MakeTraceSourceAccessor(
                                &OranNtnConflictManager::m_conflictDetected),
                            "ns3::OranNtnConflictManager::ConflictTracedCallback")
            .AddTraceSource("ConflictResolved",
                            "A conflict was resolved",
                            MakeTraceSourceAccessor(
                                &OranNtnConflictManager::m_conflictResolved),
                            "ns3::OranNtnConflictManager::ConflictTracedCallback");
    return tid;
}

OranNtnConflictManager::OranNtnConflictManager()
    : m_strategy(ConflictResolutionStrategy::PRIORITY_BASED),
      m_conflictWindow(MilliSeconds(500)),
      m_totalConflicts(0),
      m_maxLogSize(10000)
{
    NS_LOG_FUNCTION(this);
}

OranNtnConflictManager::~OranNtnConflictManager()
{
    NS_LOG_FUNCTION(this);
}

void
OranNtnConflictManager::DoDispose()
{
    m_recentActions.clear();
    m_resourceLocks.clear();
    m_conflictLog.clear();
    Object::DoDispose();
}

void
OranNtnConflictManager::SetResolutionStrategy(ConflictResolutionStrategy strategy)
{
    m_strategy = strategy;
}

ConflictResolutionStrategy
OranNtnConflictManager::GetResolutionStrategy() const
{
    return m_strategy;
}

void
OranNtnConflictManager::SetStrategyValue(uint8_t v)
{
    m_strategy = static_cast<ConflictResolutionStrategy>(v);
}

uint8_t
OranNtnConflictManager::GetStrategyValue() const
{
    return static_cast<uint8_t>(m_strategy);
}

void
OranNtnConflictManager::SetConflictWindow(Time window)
{
    m_conflictWindow = window;
}

std::string
OranNtnConflictManager::GetResourceKey(const E2RcAction& action) const
{
    std::ostringstream oss;

    switch (action.actionType)
    {
    case E2RcActionType::HANDOVER_TRIGGER:
    case E2RcActionType::HANDOVER_CANCEL:
        oss << "handover:ue" << action.targetUeId;
        break;
    case E2RcActionType::BEAM_SWITCH:
    case E2RcActionType::BEAM_HOP_SCHEDULE:
        oss << "beam:gnb" << action.targetGnbId << ":beam" << action.targetBeamId;
        break;
    case E2RcActionType::SLICE_PRB_ALLOCATION:
        oss << "prb:gnb" << action.targetGnbId << ":slice"
            << static_cast<uint32_t>(action.targetSliceId);
        break;
    case E2RcActionType::TX_POWER_CONTROL:
        oss << "power:gnb" << action.targetGnbId << ":beam" << action.targetBeamId;
        break;
    case E2RcActionType::DOPPLER_COMP_UPDATE:
        oss << "doppler:gnb" << action.targetGnbId;
        break;
    case E2RcActionType::TIMING_ADVANCE_UPDATE:
        oss << "ta:gnb" << action.targetGnbId << ":ue" << action.targetUeId;
        break;
    case E2RcActionType::CCA_THRESHOLD_ADJUST:
        oss << "cca:gnb" << action.targetGnbId;
        break;
    case E2RcActionType::MCS_OVERRIDE:
        oss << "mcs:gnb" << action.targetGnbId << ":ue" << action.targetUeId;
        break;
    case E2RcActionType::MODCOD_OVERRIDE:
        oss << "modcod:gnb" << action.targetGnbId << ":beam" << action.targetBeamId;
        break;
    case E2RcActionType::ISL_ROUTE_UPDATE:
        oss << "isl:gnb" << action.targetGnbId;
        break;
    case E2RcActionType::REGEN_MODE_SWITCH:
        oss << "regen:gnb" << action.targetGnbId;
        break;
    case E2RcActionType::BEAM_SHUTDOWN:
        oss << "beam-shutdown:gnb" << action.targetGnbId << ":beam" << action.targetBeamId;
        break;
    case E2RcActionType::COMPUTE_THROTTLE:
        oss << "compute:gnb" << action.targetGnbId;
        break;
    case E2RcActionType::DC_SETUP:
    case E2RcActionType::DC_TEARDOWN:
    case E2RcActionType::BEARER_SPLIT:
        oss << "dc:ue" << action.targetUeId;
        break;
    case E2RcActionType::INTERFERENCE_NULLING:
        oss << "interference:gnb" << action.targetGnbId << ":beam" << action.targetBeamId;
        break;
    case E2RcActionType::PRB_RESERVATION:
        oss << "prb-reserve:gnb" << action.targetGnbId << ":beam" << action.targetBeamId;
        break;
    case E2RcActionType::FL_MODEL_PUSH:
        oss << "fl:gnb" << action.targetGnbId;
        break;
    case E2RcActionType::ENERGY_PROFILE_UPDATE:
        oss << "energy:gnb" << action.targetGnbId;
        break;
    case E2RcActionType::ACTION_THZ_FREQ_SELECT:
    case E2RcActionType::ACTION_THZ_WINDOW_HOP:
        oss << "thz-freq:gnb" << action.targetGnbId;
        break;
    case E2RcActionType::ACTION_THZ_BEAM_CODEBOOK:
        oss << "thz-codebook:gnb" << action.targetGnbId << ":beam" << action.targetBeamId;
        break;
    case E2RcActionType::ACTION_THZ_RIS_CONFIG:
        oss << "thz-ris:gnb" << action.targetGnbId;
        break;
    case E2RcActionType::ACTION_THZ_WAVEFORM_SELECT:
    case E2RcActionType::ACTION_THZ_ISAC_MODE:
        oss << "thz-isac:gnb" << action.targetGnbId;
        break;
    case E2RcActionType::ACTION_THZ_POWER_BACKOFF:
        oss << "thz-power:gnb" << action.targetGnbId << ":beam" << action.targetBeamId;
        break;
    }

    return oss.str();
}

bool
OranNtnConflictManager::DetectConflict(const PendingAction& newAction,
                                        const PendingAction& existing) const
{
    // Same xApp cannot conflict with itself (update of previous action)
    if (newAction.xappId == existing.xappId)
    {
        return false;
    }

    // Check time proximity
    Time timeDiff = newAction.submissionTime - existing.submissionTime;
    if (timeDiff > m_conflictWindow)
    {
        return false;
    }

    // Same resource key = conflict
    return true;
}

const A1Policy*
OranNtnConflictManager::MatchPolicy(const E2RcAction& action) const
{
    // Find an ACTIVE policy whose scope covers this action. Scopes are the
    // strings A1Policy documents: "global", "satellite:<id>", "slice:<id>".
    // A global policy covers everything but loses to a more specific one, so
    // the most specific match is returned.
    if (m_a1Source.IsNull())
    {
        return nullptr;
    }
    static thread_local std::vector<A1Policy> policies;
    policies = m_a1Source();

    const A1Policy* best = nullptr;
    int bestSpecificity = -1;
    for (const auto& p : policies)
    {
        if (!p.active)
        {
            continue;
        }
        int spec = -1;
        if (p.scope == "global")
        {
            spec = 0;
        }
        else if (p.scope == "satellite:" + std::to_string(action.targetGnbId))
        {
            spec = 1;
        }
        else if (p.scope ==
                 "slice:" + std::to_string(static_cast<uint32_t>(action.targetSliceId)))
        {
            spec = 2;
        }
        if (spec > bestSpecificity)
        {
            bestSpecificity = spec;
            best = &p;
        }
    }
    return best;
}

XappConflict
OranNtnConflictManager::ResolveConflict(const PendingAction& a1,
                                          const PendingAction& a2) const
{
    XappConflict conflict;
    conflict.timestamp = Simulator::Now().GetSeconds();
    conflict.xapp1Id = a1.xappId;
    conflict.xapp1Name = a1.action.xappName;
    conflict.xapp2Id = a2.xappId;
    conflict.xapp2Name = a2.action.xappName;
    // Roadmap §4.1.10 — WG3 conflict taxonomy.
    conflict.conflictType = ClassifyConflict(a1.action, a2.action);

    std::string resKey = GetResourceKey(a1.action);
    // Parse resource type from key
    auto colonPos = resKey.find(':');
    conflict.resourceType = resKey.substr(0, colonPos);
    conflict.resourceId = a1.action.targetGnbId;

    switch (m_strategy)
    {
    case ConflictResolutionStrategy::PRIORITY_BASED:
        conflict.resolution = "priority";
        // Lower priority number = higher priority
        conflict.winnerId = (a1.xappPriority <= a2.xappPriority) ? a1.xappId : a2.xappId;
        break;

    case ConflictResolutionStrategy::TEMPORAL:
        conflict.resolution = "temporal";
        // Most recent action wins
        conflict.winnerId =
            (a1.submissionTime >= a2.submissionTime) ? a1.xappId : a2.xappId;
        break;

    case ConflictResolutionStrategy::MERGE:
        conflict.resolution = "merge";
        // Higher confidence action wins for merge
        conflict.winnerId =
            (a1.action.confidence >= a2.action.confidence) ? a1.xappId : a2.xappId;
        break;

    case ConflictResolutionStrategy::A1_GUIDED: {
        // ORAN-08. This arm used to be byte-identical to PRIORITY_BASED, so
        // "A1-guided" meant nothing: the Non-RT RIC's policy had no bearing on
        // which xApp won. It now consults the active policies, and the scoped
        // policy's own priority decides - which can hand the conflict to the
        // xApp with the WEAKER xApp priority, the outcome PRIORITY_BASED can
        // never produce. That difference is the whole point of the strategy.
        const A1Policy* p1 = MatchPolicy(a1.action);
        const A1Policy* p2 = MatchPolicy(a2.action);
        if (p1 && p2)
        {
            conflict.resolution = "a1_guided:policy";
            // Lower policy priority value wins (0 = highest), per A1Policy.
            conflict.winnerId = (p1->priority <= p2->priority) ? a1.xappId : a2.xappId;
        }
        else if (p1 || p2)
        {
            // Exactly one action is covered by an active policy: the covered
            // one wins, because the Non-RT RIC has explicitly scoped it.
            conflict.resolution = "a1_guided:scoped";
            conflict.winnerId = p1 ? a1.xappId : a2.xappId;
        }
        else
        {
            // No policy covers this resource. Fall back to xApp priority, and
            // SAY SO in the log rather than presenting it as A1 guidance.
            conflict.resolution = "a1_guided:no-policy-fallback-priority";
            conflict.winnerId = (a1.xappPriority <= a2.xappPriority) ? a1.xappId : a2.xappId;
        }
        break;
    }

    case ConflictResolutionStrategy::ML_BASED:
        // ORAN-08. There is no model here, and there never was: this arm was
        // byte-identical to CONFIDENCE_BASED. Rather than keep advertising an
        // ML strategy that is a confidence comparison, it names itself for what
        // it does. The attribute description no longer offers it as a distinct
        // choice.
        conflict.resolution = "confidence (ML_BASED is not implemented)";
        conflict.winnerId =
            (a1.action.confidence >= a2.action.confidence) ? a1.xappId : a2.xappId;
        break;
    }

    return conflict;
}

bool
OranNtnConflictManager::CheckAndResolve(uint32_t xappId, uint8_t xappPriority,
                                          const E2RcAction& action)
{
    NS_LOG_FUNCTION(this << xappId << static_cast<uint32_t>(action.actionType));

    PruneOldActions();

    std::string resourceKey = GetResourceKey(action);

    // Check resource lock
    if (IsResourceLocked(resourceKey.substr(0, resourceKey.find(':')),
                          action.targetGnbId, xappId))
    {
        NS_LOG_INFO("ConflictMgr: Resource " << resourceKey
                     << " locked by another xApp, rejecting action from xApp " << xappId);
        return false;
    }

    PendingAction newAction;
    newAction.xappId = xappId;
    newAction.xappPriority = xappPriority;
    newAction.action = action;
    newAction.submissionTime = Simulator::Now();
    newAction.processed = false;

    // ORAN-08: look wider than the exact resource key.
    //
    // Detection used to compare only against m_recentActions[resourceKey], i.e.
    // actions whose key was byte-identical. That made two of the four WG3
    // conflict types structurally unreachable, so conflict_log.csv could never
    // carry them however many xApps were running:
    //
    //   IMPLICIT (different family, same UE) - unreachable, because different
    //   families always produce different key prefixes ("mcs:..." vs
    //   "power:..."), so the two actions were never compared at all.
    //
    //   INDIRECT on PRB - unreachable, because SLICE_PRB_ALLOCATION keys on
    //   "prb:gnbX:sliceY" and PRB_RESERVATION on "prb-reserve:gnbX:beamZ".
    //
    // The taxonomy test passed regardless: it called the static ClassifyConflict
    // directly with exactly those pairs, so it certified the enum mapping and
    // not the mechanism. It would still have passed if CheckAndResolve had never
    // called the classifier.
    //
    // Candidates are now gathered from three indices: the exact key (DIRECT and
    // same-key INDIRECT, as before), the family+gNB index (cross-parameter
    // INDIRECT), and the UE index (cross-family IMPLICIT). Duplicates are
    // suppressed so a pair reachable through two indices is judged once.
    std::vector<const PendingAction*> candidates;
    std::set<const PendingAction*> seen;
    auto gather = [&](const std::string& key) {
        auto it = m_recentActions.find(key);
        if (it == m_recentActions.end())
        {
            return;
        }
        for (const auto& e : it->second)
        {
            if (seen.insert(&e).second)
            {
                candidates.push_back(&e);
            }
        }
    };
    gather(resourceKey);
    gather(FamilyIndexKey(action));
    gather(UeIndexKey(action));

    for (const auto* existingPtr : candidates)
    {
        const PendingAction& existing = *existingPtr;
        if (DetectConflict(newAction, existing))
        {
            XappConflict conflict = ResolveConflict(newAction, existing);
            m_totalConflicts++;

            m_conflictDetected(conflict);

            // Log
            if (m_conflictLog.size() >= m_maxLogSize)
            {
                m_conflictLog.pop_front();
            }
            m_conflictLog.push_back(conflict);

            m_conflictResolved(conflict);

            bool allowed = (conflict.winnerId == xappId);
            NS_LOG_INFO("ConflictMgr: Conflict on " << resourceKey
                         << " between xApp " << xappId << " and xApp "
                         << existing.xappId << " -> winner: xApp "
                         << conflict.winnerId);
            if (!allowed)
            {
                return false;
            }
        }
    }

    // Record this action
    // Record under all three indices so a later action can find this one by
    // whichever relationship applies. The deques are small and pruned on the
    // same window, so the duplication costs little and keeps the lookup O(1).
    newAction.processed = true;
    m_recentActions[resourceKey].push_back(newAction);
    const std::string famKey = FamilyIndexKey(action);
    if (famKey != resourceKey)
    {
        m_recentActions[famKey].push_back(newAction);
    }
    const std::string ueKey = UeIndexKey(action);
    if (!ueKey.empty() && ueKey != resourceKey && ueKey != famKey)
    {
        m_recentActions[ueKey].push_back(newAction);
    }

    return true;
}

std::string
OranNtnConflictManager::FamilyIndexKey(const E2RcAction& action)
{
    // Same resource family on the same gNB, regardless of which parameter of
    // that family the action touches. This is the INDIRECT relationship.
    std::ostringstream oss;
    oss << "#fam" << static_cast<uint32_t>(FamilyOf(action.actionType)) << ":gnb"
        << action.targetGnbId;
    return oss.str();
}

std::string
OranNtnConflictManager::UeIndexKey(const E2RcAction& action)
{
    // Same UE, regardless of family: two xApps tuning unrelated parameters on
    // one UE still couple through that UE's KPIs, which is what IMPLICIT means.
    // UE 0 is the cell-wide wildcard and would otherwise collect every
    // cell-scoped action into one bucket, so it is excluded.
    if (action.targetUeId == 0)
    {
        return {};
    }
    std::ostringstream oss;
    oss << "#ue" << action.targetUeId;
    return oss.str();
}

void
OranNtnConflictManager::PruneOldActions()
{
    Time now = Simulator::Now();
    for (auto& [key, actions] : m_recentActions)
    {
        while (!actions.empty() &&
               (now - actions.front().submissionTime) > m_conflictWindow * 2)
        {
            actions.pop_front();
        }
    }
}

std::vector<XappConflict>
OranNtnConflictManager::GetRecentConflicts(Time window) const
{
    std::vector<XappConflict> result;
    double now = Simulator::Now().GetSeconds();
    double windowSec = window.GetSeconds();

    for (const auto& c : m_conflictLog)
    {
        if (now - c.timestamp <= windowSec)
        {
            result.push_back(c);
        }
    }
    return result;
}

uint32_t
OranNtnConflictManager::GetTotalConflicts() const
{
    return m_totalConflicts;
}

std::map<std::pair<uint32_t, uint32_t>, uint32_t>
OranNtnConflictManager::GetConflictMatrix() const
{
    std::map<std::pair<uint32_t, uint32_t>, uint32_t> matrix;
    for (const auto& c : m_conflictLog)
    {
        auto key = std::make_pair(std::min(c.xapp1Id, c.xapp2Id),
                                   std::max(c.xapp1Id, c.xapp2Id));
        matrix[key]++;
    }
    return matrix;
}

void
OranNtnConflictManager::LockResource(uint32_t xappId, const std::string& resourceType,
                                       uint32_t resourceId, Time duration)
{
    std::string key = resourceType + ":" + std::to_string(resourceId);
    ResourceLock lock;
    lock.xappId = xappId;
    lock.expiryTime = Simulator::Now() + duration;
    m_resourceLocks[key] = lock;

    NS_LOG_INFO("ConflictMgr: xApp " << xappId << " locked resource " << key
                 << " for " << duration.As(Time::MS));
}

void
OranNtnConflictManager::UnlockResource(const std::string& resourceType,
                                         uint32_t resourceId)
{
    std::string key = resourceType + ":" + std::to_string(resourceId);
    m_resourceLocks.erase(key);
}

bool
OranNtnConflictManager::IsResourceLocked(const std::string& resourceType,
                                           uint32_t resourceId,
                                           uint32_t requestingXappId) const
{
    std::string key = resourceType + ":" + std::to_string(resourceId);
    auto it = m_resourceLocks.find(key);
    if (it == m_resourceLocks.end())
    {
        return false;
    }

    if (it->second.xappId == requestingXappId)
    {
        return false; // Own lock
    }

    if (Simulator::Now() >= it->second.expiryTime)
    {
        return false; // Lock expired
    }

    return true;
}

void
OranNtnConflictManager::WriteConflictLog(const std::string& filename) const
{
    std::ofstream ofs(filename);
    ofs << "timestamp,xapp1_id,xapp1_name,xapp2_id,xapp2_name,"
           "resource_type,resource_id,resolution,winner_id,conflict_type\n";

    for (const auto& c : m_conflictLog)
    {
        ofs << c.timestamp << "," << c.xapp1Id << "," << c.xapp1Name << ","
            << c.xapp2Id << "," << c.xapp2Name << "," << c.resourceType << ","
            << c.resourceId << "," << c.resolution << "," << c.winnerId << ","
            << ConflictTypeName(c.conflictType) << "\n";
    }
}

ConflictType
OranNtnConflictManager::ClassifyConflict(const E2RcAction& a,
                                           const E2RcAction& b)
{
    // DIRECT: same parameter on the same UE+gNB+beam.
    if (a.actionType == b.actionType && a.targetGnbId == b.targetGnbId &&
        a.targetUeId == b.targetUeId && a.targetBeamId == b.targetBeamId)
    {
        return ConflictType::DIRECT;
    }
    // INDIRECT: same resource family on the same gNB (different parameter).
    if (FamilyOf(a.actionType) == FamilyOf(b.actionType) &&
        FamilyOf(a.actionType) != ResourceFamily::OTHER &&
        a.targetGnbId == b.targetGnbId)
    {
        return ConflictType::INDIRECT;
    }
    // IMPLICIT: different parameter + different family, but both target the
    // same UE (KPI coupling via that UE). UE 0 is wildcard, exclude it.
    if (a.targetUeId != 0 && a.targetUeId == b.targetUeId)
    {
        return ConflictType::IMPLICIT;
    }
    return ConflictType::UNKNOWN;
}

} // namespace ns3
