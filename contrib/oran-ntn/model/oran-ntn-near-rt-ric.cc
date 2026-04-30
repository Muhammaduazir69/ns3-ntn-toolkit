/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-only
 * Author: Muhammad Uzair
 */

#include "oran-ntn-near-rt-ric.h"

#include <ns3/log.h>
#include <ns3/simulator.h>

#include <fstream>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("OranNtnNearRtRic");

// ============================================================================
//  OranNtnSdl
// ============================================================================

NS_OBJECT_ENSURE_REGISTERED(OranNtnSdl);

TypeId
OranNtnSdl::GetTypeId()
{
    static TypeId tid = TypeId("ns3::OranNtnSdl")
                            .SetParent<Object>()
                            .SetGroupName("OranNtn")
                            .AddConstructor<OranNtnSdl>();
    return tid;
}

OranNtnSdl::OranNtnSdl()
{
}

OranNtnSdl::~OranNtnSdl()
{
}

void
OranNtnSdl::DoDispose()
{
    m_scalarStore.clear();
    m_vectorStore.clear();
    m_latestKpm.clear();
    Object::DoDispose();
}

void
OranNtnSdl::Set(const std::string& ns, const std::string& key, double value)
{
    m_scalarStore[ns][key] = value;
}

double
OranNtnSdl::Get(const std::string& ns, const std::string& key, double defaultVal) const
{
    auto nsIt = m_scalarStore.find(ns);
    if (nsIt != m_scalarStore.end())
    {
        auto keyIt = nsIt->second.find(key);
        if (keyIt != nsIt->second.end())
        {
            return keyIt->second;
        }
    }
    return defaultVal;
}

bool
OranNtnSdl::Has(const std::string& ns, const std::string& key) const
{
    auto nsIt = m_scalarStore.find(ns);
    if (nsIt != m_scalarStore.end())
    {
        return nsIt->second.find(key) != nsIt->second.end();
    }
    return false;
}

void
OranNtnSdl::Delete(const std::string& ns, const std::string& key)
{
    auto nsIt = m_scalarStore.find(ns);
    if (nsIt != m_scalarStore.end())
    {
        nsIt->second.erase(key);
    }
}

void
OranNtnSdl::SetVector(const std::string& ns, const std::string& key,
                        const std::vector<double>& vec)
{
    m_vectorStore[ns][key] = vec;
}

std::vector<double>
OranNtnSdl::GetVector(const std::string& ns, const std::string& key) const
{
    auto nsIt = m_vectorStore.find(ns);
    if (nsIt != m_vectorStore.end())
    {
        auto keyIt = nsIt->second.find(key);
        if (keyIt != nsIt->second.end())
        {
            return keyIt->second;
        }
    }
    return {};
}

void
OranNtnSdl::StoreLatestKpm(uint32_t gnbId, const E2KpmReport& report)
{
    m_latestKpm[gnbId] = report;
}

E2KpmReport
OranNtnSdl::GetLatestKpm(uint32_t gnbId) const
{
    auto it = m_latestKpm.find(gnbId);
    if (it != m_latestKpm.end())
    {
        return it->second;
    }
    return E2KpmReport{};
}

std::map<uint32_t, E2KpmReport>
OranNtnSdl::GetAllLatestKpm() const
{
    return m_latestKpm;
}

// ============================================================================
//  OranNtnNearRtRic
// ============================================================================

NS_OBJECT_ENSURE_REGISTERED(OranNtnNearRtRic);

TypeId
OranNtnNearRtRic::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::OranNtnNearRtRic")
            .SetParent<Object>()
            .SetGroupName("OranNtn")
            .AddConstructor<OranNtnNearRtRic>()
            .AddAttribute("ControlLoopInterval",
                          "Near-RT RIC control loop interval",
                          TimeValue(MilliSeconds(100)),
                          MakeTimeAccessor(&OranNtnNearRtRic::m_controlLoopInterval),
                          MakeTimeChecker())
            .AddTraceSource("XappRegistered",
                            "An xApp was registered",
                            MakeTraceSourceAccessor(&OranNtnNearRtRic::m_xappRegistered),
                            "ns3::OranNtnNearRtRic::XappTracedCallback")
            .AddTraceSource("XappDeregistered",
                            "An xApp was deregistered",
                            MakeTraceSourceAccessor(&OranNtnNearRtRic::m_xappDeregistered),
                            "ns3::OranNtnNearRtRic::XappIdTracedCallback")
            .AddTraceSource("ActionProcessed",
                            "An RC action was processed by the RIC",
                            MakeTraceSourceAccessor(&OranNtnNearRtRic::m_actionProcessed),
                            "ns3::OranNtnNearRtRic::ActionTracedCallback")
            .AddTraceSource("ConflictResolved",
                            "A conflict between xApps was resolved",
                            MakeTraceSourceAccessor(&OranNtnNearRtRic::m_conflictResolved),
                            "ns3::OranNtnNearRtRic::ConflictTracedCallback")
            .AddTraceSource("KpmReceived",
                            "A KPM report was received and processed",
                            MakeTraceSourceAccessor(&OranNtnNearRtRic::m_kpmReceived),
                            "ns3::OranNtnNearRtRic::KpmTracedCallback");
    return tid;
}

OranNtnNearRtRic::OranNtnNearRtRic()
    : m_nextXappId(1),
      m_controlLoopInterval(MilliSeconds(100)),
      m_totalActionsProcessed(0),
      m_totalConflicts(0),
      m_totalPolicyViolations(0),
      m_cumulativeActionLatency(Seconds(0))
{
    NS_LOG_FUNCTION(this);
}

OranNtnNearRtRic::~OranNtnNearRtRic()
{
    NS_LOG_FUNCTION(this);
}

void
OranNtnNearRtRic::DoDispose()
{
    NS_LOG_FUNCTION(this);
    Simulator::Cancel(m_controlLoopEvent);

    for (auto& [id, xapp] : m_xapps)
    {
        xapp->Stop();
    }
    m_xapps.clear();

    if (m_e2Term)
    {
        m_e2Term->Dispose();
        m_e2Term = nullptr;
    }
    if (m_a1Adapter)
    {
        m_a1Adapter->Dispose();
        m_a1Adapter = nullptr;
    }
    if (m_conflictMgr)
    {
        m_conflictMgr->Dispose();
        m_conflictMgr = nullptr;
    }
    if (m_sdl)
    {
        m_sdl->Dispose();
        m_sdl = nullptr;
    }

    Object::DoDispose();
}

void
OranNtnNearRtRic::Initialize()
{
    NS_LOG_FUNCTION(this);

    // Create sub-components
    m_e2Term = CreateObject<OranNtnE2Termination>();
    m_a1Adapter = CreateObject<OranNtnA1Adapter>();
    m_conflictMgr = CreateObject<OranNtnConflictManager>();
    m_sdl = CreateObject<OranNtnSdl>();

    // A1 feedback wiring is done externally via ConnectA1Interface in the helper

    // Start the RIC control loop
    m_controlLoopEvent = Simulator::Schedule(m_controlLoopInterval,
                                              &OranNtnNearRtRic::ControlLoop,
                                              this);

    NS_LOG_INFO("Near-RT RIC initialized with E2 termination, A1 adapter, "
                "conflict manager, and SDL (control loop interval="
                << m_controlLoopInterval.As(Time::MS) << ")");
}

// ---- xApp management ----

uint32_t
OranNtnNearRtRic::RegisterXapp(Ptr<OranNtnXappBase> xapp)
{
    NS_LOG_FUNCTION(this << xapp->GetXappName());

    uint32_t id = m_nextXappId++;
    xapp->SetXappId(id);
    xapp->SetRic(this);

    m_xapps[id] = xapp;
    m_xappRegistered(id, xapp->GetXappName());

    NS_LOG_INFO("Near-RT RIC: Registered xApp '" << xapp->GetXappName()
                << "' with id=" << id << " priority=" << (int)xapp->GetPriority());
    return id;
}

void
OranNtnNearRtRic::DeregisterXapp(uint32_t xappId)
{
    NS_LOG_FUNCTION(this << xappId);
    auto it = m_xapps.find(xappId);
    if (it != m_xapps.end())
    {
        it->second->Stop();
        m_xapps.erase(it);
        m_xappDeregistered(xappId);
    }
}

Ptr<OranNtnXappBase>
OranNtnNearRtRic::GetXapp(uint32_t xappId) const
{
    auto it = m_xapps.find(xappId);
    if (it != m_xapps.end())
    {
        return it->second;
    }
    return nullptr;
}

std::vector<uint32_t>
OranNtnNearRtRic::GetRegisteredXappIds() const
{
    std::vector<uint32_t> ids;
    ids.reserve(m_xapps.size());
    for (const auto& [id, xapp] : m_xapps)
    {
        ids.push_back(id);
    }
    return ids;
}

Ptr<OranNtnXappBase>
OranNtnNearRtRic::GetXappByName(const std::string& name) const
{
    for (const auto& [id, xapp] : m_xapps)
    {
        if (xapp->GetXappName() == name)
        {
            return xapp;
        }
    }
    return nullptr;
}

// ---- E2 interface ----

Ptr<OranNtnE2Termination>
OranNtnNearRtRic::GetE2Termination() const
{
    return m_e2Term;
}

void
OranNtnNearRtRic::ConnectE2Node(Ptr<OranNtnE2Node> node)
{
    NS_LOG_FUNCTION(this << node->GetNodeId());
    m_e2Term->RegisterE2Node(node);
}

// ---- A1 interface ----

Ptr<OranNtnA1Adapter>
OranNtnNearRtRic::GetA1Adapter() const
{
    return m_a1Adapter;
}

void
OranNtnNearRtRic::HandleA1Policy(const A1NtnPolicy& policy)
{
    NS_LOG_FUNCTION(this << policy.policyId);
    m_a1Adapter->HandleIncomingPolicy(policy);
}

// ---- RC action processing ----

bool
OranNtnNearRtRic::ProcessXappAction(uint32_t xappId, const E2RcAction& action)
{
    NS_LOG_FUNCTION(this << xappId << static_cast<uint8_t>(action.actionType));

    auto xappIt = m_xapps.find(xappId);
    if (xappIt == m_xapps.end())
    {
        NS_LOG_WARN("Near-RT RIC: Unknown xApp " << xappId);
        return false;
    }

    uint8_t xappPriority = xappIt->second->GetPriority();

    // Step 1: A1 policy compliance check
    if (!xappIt->second->CheckPolicyCompliance(action))
    {
        m_totalPolicyViolations++;
        NS_LOG_INFO("Near-RT RIC: Action from xApp " << xappId
                     << " violates A1 policy, rejected");
        m_actionProcessed(action, false);
        return false;
    }

    // Step 2: Conflict checking
    bool allowed = m_conflictMgr->CheckAndResolve(xappId, xappPriority, action);
    if (!allowed)
    {
        m_totalConflicts++;

        // Notify the xApp about the conflict
        auto conflicts = m_conflictMgr->GetRecentConflicts(MilliSeconds(100));
        if (!conflicts.empty())
        {
            xappIt->second->HandleConflict(conflicts.back());
            m_conflictResolved(conflicts.back());
        }

        m_actionProcessed(action, false);
        return false;
    }

    // Step 3: Route to E2 node
    bool success = m_e2Term->RouteRcAction(action);

    m_totalActionsProcessed++;
    m_actionProcessed(action, success);

    // Store in SDL for cross-xApp visibility
    m_sdl->Set("ric", "last_action_time", Simulator::Now().GetSeconds());
    m_sdl->Set("ric", "last_action_type",
               static_cast<double>(static_cast<uint8_t>(action.actionType)));
    m_sdl->Set("ric", "last_action_xapp", static_cast<double>(xappId));

    return success;
}

// ---- Conflict manager ----

Ptr<OranNtnConflictManager>
OranNtnNearRtRic::GetConflictManager() const
{
    return m_conflictMgr;
}

// ---- SDL ----

Ptr<OranNtnSdl>
OranNtnNearRtRic::GetSdl() const
{
    return m_sdl;
}

// ---- Metrics ----

OranNtnNearRtRic::RicMetrics
OranNtnNearRtRic::GetMetrics() const
{
    RicMetrics m;
    m.totalXapps = static_cast<uint32_t>(m_xapps.size());
    m.activeXapps = 0;
    for (const auto& [id, xapp] : m_xapps)
    {
        if (xapp->GetState() == XappState::RUNNING)
        {
            m.activeXapps++;
        }
    }
    m.totalE2Nodes = static_cast<uint32_t>(m_e2Term->GetConnectedNodeIds().size());
    m.totalSubscriptions = 0; // Would need E2Term to track this
    m.totalActionsProcessed = m_totalActionsProcessed;
    m.totalConflicts = m_totalConflicts;
    m.totalPolicyViolations = m_totalPolicyViolations;

    if (m_totalActionsProcessed > 0)
    {
        m.avgActionLatency = m_cumulativeActionLatency / m_totalActionsProcessed;
    }

    return m;
}

void
OranNtnNearRtRic::WriteMetrics(const std::string& filename) const
{
    std::ofstream ofs(filename);
    auto m = GetMetrics();

    ofs << "=== Near-RT RIC Metrics ===\n"
        << "Total xApps: " << m.totalXapps << "\n"
        << "Active xApps: " << m.activeXapps << "\n"
        << "Total E2 Nodes: " << m.totalE2Nodes << "\n"
        << "Total Actions Processed: " << m.totalActionsProcessed << "\n"
        << "Total Conflicts: " << m.totalConflicts << "\n"
        << "Total Policy Violations: " << m.totalPolicyViolations << "\n\n";

    ofs << "=== Per-xApp Metrics ===\n";
    for (const auto& [id, xapp] : m_xapps)
    {
        auto xm = xapp->GetMetrics();
        ofs << "xApp " << xapp->GetXappName() << " (id=" << id
            << ", priority=" << (int)xapp->GetPriority() << "):\n"
            << "  Decisions: " << xm.totalDecisions << "\n"
            << "  Successful Actions: " << xm.successfulActions << "\n"
            << "  Failed Actions: " << xm.failedActions << "\n"
            << "  Conflicts: " << xm.conflictsEncountered
            << " (won " << xm.conflictsWon << ")\n"
            << "  Avg Confidence: " << xm.avgConfidence << "\n"
            << "  Avg Decision Latency: " << xm.avgDecisionLatency_ms << " ms\n\n";
    }

    // Write conflict matrix
    ofs << "=== Conflict Matrix ===\n";
    auto conflictMatrix = m_conflictMgr->GetConflictMatrix();
    for (const auto& [pair, count] : conflictMatrix)
    {
        auto xapp1 = GetXapp(pair.first);
        auto xapp2 = GetXapp(pair.second);
        std::string name1 = xapp1 ? xapp1->GetXappName() : std::to_string(pair.first);
        std::string name2 = xapp2 ? xapp2->GetXappName() : std::to_string(pair.second);
        ofs << name1 << " <-> " << name2 << ": " << count << " conflicts\n";
    }
}

void
OranNtnNearRtRic::ControlLoop()
{
    // Periodic RIC housekeeping - prune stale data, check xApp health
    for (auto& [id, xapp] : m_xapps)
    {
        if (xapp->GetState() == XappState::FAILED)
        {
            NS_LOG_WARN("Near-RT RIC: xApp " << xapp->GetXappName()
                         << " in FAILED state, attempting restart");
            xapp->Stop();
            xapp->Start();
        }
    }

    m_controlLoopEvent = Simulator::Schedule(m_controlLoopInterval,
                                              &OranNtnNearRtRic::ControlLoop,
                                              this);
}

void
OranNtnNearRtRic::RouteIndicationToXapps(uint32_t subscriptionId,
                                           const E2KpmReport& report)
{
    // Store in SDL
    m_sdl->StoreLatestKpm(report.gnbId, report);
    m_kpmReceived(report.gnbId, report);

    // Route to subscribed xApps
    for (auto& [id, xapp] : m_xapps)
    {
        if (xapp->GetState() == XappState::RUNNING)
        {
            xapp->HandleKpmIndication(subscriptionId, report);
        }
    }
}

} // namespace ns3
