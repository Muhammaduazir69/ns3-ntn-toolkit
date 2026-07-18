/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-only
 * Author: Muhammad Uzair
 */

#include "oran-ntn-xapp-base.h"

#include "oran-ntn-near-rt-ric.h"

#include <ns3/log.h>
#include <ns3/simulator.h>
#include <ns3/uinteger.h>

#include <limits>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("OranNtnXappBase");
NS_OBJECT_ENSURE_REGISTERED(OranNtnXappBase);

TypeId
OranNtnXappBase::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::OranNtnXappBase")
            .SetParent<Object>()
            .SetGroupName("OranNtn")
            .AddAttribute("DecisionInterval",
                          "Interval between decision cycles",
                          TimeValue(MilliSeconds(100)),
                          MakeTimeAccessor(&OranNtnXappBase::m_decisionInterval),
                          MakeTimeChecker())
            .AddAttribute("MaxReportsPerGnb",
                          "Maximum KPM reports stored per gNB",
                          UintegerValue(100),
                          MakeUintegerAccessor(&OranNtnXappBase::m_maxReportsPerGnb),
                          MakeUintegerChecker<uint32_t>())
            .AddAttribute("DataRetentionWindow",
                          "How long to keep KPM data",
                          TimeValue(Seconds(60)),
                          MakeTimeAccessor(&OranNtnXappBase::m_dataRetentionWindow),
                          MakeTimeChecker())
            .AddTraceSource("ActionSubmitted",
                            "An RC action was submitted to the RIC",
                            MakeTraceSourceAccessor(&OranNtnXappBase::m_actionSubmitted),
                            "ns3::OranNtnXappBase::ActionTracedCallback")
            .AddTraceSource("ActionResult",
                            "Result of an RC action submission",
                            MakeTraceSourceAccessor(&OranNtnXappBase::m_actionResult),
                            "ns3::OranNtnXappBase::ActionResultTracedCallback")
            .AddTraceSource("ConflictEncountered",
                            "This xApp's action conflicted with another",
                            MakeTraceSourceAccessor(&OranNtnXappBase::m_conflictEncountered),
                            "ns3::OranNtnXappBase::ConflictTracedCallback")
            .AddTraceSource("DecisionConfidence",
                            "Confidence of each decision made",
                            MakeTraceSourceAccessor(&OranNtnXappBase::m_decisionConfidence),
                            "ns3::OranNtnXappBase::ConfidenceTracedCallback");
    return tid;
}

OranNtnXappBase::OranNtnXappBase()
    : m_maxReportsPerGnb(100),
      m_dataRetentionWindow(Seconds(60)),
      m_xappId(0),
      m_xappName("unnamed"),
      m_priority(128),
      m_state(XappState::REGISTERED),
      m_decisionInterval(MilliSeconds(100))
{
    NS_LOG_FUNCTION(this);
    m_metrics = {};
}

OranNtnXappBase::~OranNtnXappBase()
{
    NS_LOG_FUNCTION(this);
}

void
OranNtnXappBase::DoDispose()
{
    NS_LOG_FUNCTION(this);
    Simulator::Cancel(m_decisionEvent);
    m_kpmDatabase.clear();
    m_ric = nullptr;
    Object::DoDispose();
}

// ---- Identity ----

void
OranNtnXappBase::SetXappId(uint32_t id)
{
    m_xappId = id;
}

void
OranNtnXappBase::SetXappName(const std::string& name)
{
    m_xappName = name;
}

void
OranNtnXappBase::SetPriority(uint8_t priority)
{
    m_priority = priority;
}

uint32_t
OranNtnXappBase::GetXappId() const
{
    return m_xappId;
}

std::string
OranNtnXappBase::GetXappName() const
{
    return m_xappName;
}

uint8_t
OranNtnXappBase::GetPriority() const
{
    return m_priority;
}

XappState
OranNtnXappBase::GetState() const
{
    return m_state;
}

XappMetrics
OranNtnXappBase::GetMetrics() const
{
    return m_metrics;
}

// ---- Lifecycle ----

void
OranNtnXappBase::Start()
{
    NS_LOG_FUNCTION(this << m_xappName);
    m_state = XappState::RUNNING;

    SetupSubscriptions();

    // Start decision cycle
    m_decisionEvent = Simulator::Schedule(m_decisionInterval,
                                           &OranNtnXappBase::DecisionCycleWrapper,
                                           this);

    NS_LOG_INFO("xApp " << m_xappName << " (id=" << m_xappId << ") started"
                << " with decision interval " << m_decisionInterval.As(Time::MS));
}

void
OranNtnXappBase::Stop()
{
    NS_LOG_FUNCTION(this << m_xappName);
    m_state = XappState::DEREGISTERED;
    Simulator::Cancel(m_decisionEvent);

    // Delete subscriptions
    if (m_ric)
    {
        auto e2term = m_ric->GetE2Termination();
        for (uint32_t subId : m_subscriptionIds)
        {
            e2term->DeleteSubscription(subId);
        }
    }
    m_subscriptionIds.clear();

    NS_LOG_INFO("xApp " << m_xappName << " stopped. Metrics: "
                << m_metrics.totalDecisions << " decisions, "
                << m_metrics.successfulActions << " successful actions");
}

void
OranNtnXappBase::Suspend()
{
    NS_LOG_FUNCTION(this);
    m_state = XappState::SUSPENDED;
    Simulator::Cancel(m_decisionEvent);
}

void
OranNtnXappBase::Resume()
{
    NS_LOG_FUNCTION(this);
    m_state = XappState::RUNNING;
    m_decisionEvent = Simulator::Schedule(m_decisionInterval,
                                           &OranNtnXappBase::DecisionCycleWrapper,
                                           this);
}

void
OranNtnXappBase::SetRic(Ptr<OranNtnNearRtRic> ric)
{
    m_ric = ric;
}

// ---- E2 data ingestion ----

void
OranNtnXappBase::HandleKpmIndication(uint32_t subscriptionId,
                                       const E2KpmReport& report)
{
    NS_LOG_FUNCTION(this << subscriptionId << report.gnbId);

    auto& gnbReports = m_kpmDatabase[report.gnbId];
    gnbReports.push_back(report);

    // Trim to max size
    while (gnbReports.size() > m_maxReportsPerGnb)
    {
        gnbReports.pop_front();
    }

    // Call subclass processing
    ProcessKpmReport(report);
}

E2KpmReport
OranNtnXappBase::GetLatestReport(uint32_t gnbId) const
{
    auto it = m_kpmDatabase.find(gnbId);
    if (it != m_kpmDatabase.end() && !it->second.empty())
    {
        return it->second.back();
    }
    return E2KpmReport{};
}

std::vector<E2KpmReport>
OranNtnXappBase::GetReportsInWindow(Time window) const
{
    std::vector<E2KpmReport> result;
    double now = Simulator::Now().GetSeconds();
    double windowSec = window.GetSeconds();

    for (const auto& [gnbId, reports] : m_kpmDatabase)
    {
        for (const auto& r : reports)
        {
            if (now - r.timestamp <= windowSec)
            {
                result.push_back(r);
            }
        }
    }
    return result;
}

std::vector<E2KpmReport>
OranNtnXappBase::GetUeReportsInWindow(uint32_t ueId, Time window) const
{
    std::vector<E2KpmReport> result;
    double now = Simulator::Now().GetSeconds();
    double windowSec = window.GetSeconds();

    for (const auto& [gnbId, reports] : m_kpmDatabase)
    {
        for (const auto& r : reports)
        {
            if (r.ueId == ueId && (now - r.timestamp) <= windowSec)
            {
                result.push_back(r);
            }
        }
    }
    return result;
}

void
OranNtnXappBase::SetUeServingCell(uint32_t ueId, uint32_t gnbId)
{
    m_ueServingCell[ueId] = gnbId;
}

bool
OranNtnXappBase::GetUeServingCell(uint32_t ueId, uint32_t& gnbId) const
{
    auto it = m_ueServingCell.find(ueId);
    if (it == m_ueServingCell.end())
    {
        return false;
    }
    gnbId = it->second;
    return true;
}

E2KpmReport
OranNtnXappBase::GetUeServingReport(uint32_t ueId, Time window, bool& found) const
{
    found = false;
    E2KpmReport serving{};
    uint32_t servingGnb = 0;
    bool haveServing = GetUeServingCell(ueId, servingGnb);

    // Pass 1: newest report from the KNOWN serving cell.
    // Pass 2 (fallback): newest report from any cell — by timestamp, never by
    // gnbId map order.
    double bestServingTs = -std::numeric_limits<double>::infinity();
    double bestAnyTs = -std::numeric_limits<double>::infinity();
    E2KpmReport newestAny{};
    bool haveAny = false;
    double now = Simulator::Now().GetSeconds();
    double windowSec = window.GetSeconds();

    for (const auto& [gnbId, reports] : m_kpmDatabase)
    {
        for (const auto& r : reports)
        {
            if (r.ueId != ueId || (now - r.timestamp) > windowSec)
            {
                continue;
            }
            if (r.timestamp > bestAnyTs)
            {
                bestAnyTs = r.timestamp;
                newestAny = r;
                haveAny = true;
            }
            if (haveServing && gnbId == servingGnb && r.timestamp > bestServingTs)
            {
                bestServingTs = r.timestamp;
                serving = r;
                found = true;
            }
        }
    }

    if (found)
    {
        return serving;
    }
    // Serving cell unknown or has no report in window: newest report overall.
    found = haveAny;
    return newestAny;
}

// ---- A1 policy ----

std::vector<A1Policy>
OranNtnXappBase::GetApplicablePolicies() const
{
    std::vector<A1Policy> result;
    if (m_ric)
    {
        auto a1Adapter = m_ric->GetA1Adapter();
        auto policies = a1Adapter->GetActivePolicies();
        for (const auto& p : policies)
        {
            result.push_back(p);
        }
    }
    return result;
}

bool
OranNtnXappBase::CheckPolicyCompliance(const E2RcAction& action) const
{
    if (!m_ric)
    {
        return true;
    }

    auto a1Adapter = m_ric->GetA1Adapter();
    auto policies = a1Adapter->GetActivePolicies();

    // Only HANDOVER_TRIGGER actions are governed by HO_THRESHOLD policies here.
    if (action.actionType != E2RcActionType::HANDOVER_TRIGGER)
    {
        return true;
    }

    const Time now = Simulator::Now();

    for (const auto& policy : policies)
    {
        if (policy.type != A1PolicyType::HO_THRESHOLD || !policy.active)
        {
            continue;
        }

        // Action semantics (see OranNtnXappHoPredict::BuildAction):
        //   parameter1 = candidate-cell SINR (dB)
        //   parameter2 = candidate-cell TTE (s)
        // Policy semantics (see OranNtnA1PolicyManager::GenerateOrbitAwarePolicies):
        //   param1 = minimum acceptable TTE (s)
        //   param2 = minimum acceptable SINR threshold (dB)
        const double candSinrDb = action.parameter1;
        const double candTteS = action.parameter2;

        // (1) SINR-threshold check: reject a HO onto a target below the A1 SINR
        //     floor (would trade a working link for a weaker one).
        if (candSinrDb < policy.param2)
        {
            NS_LOG_INFO("xApp " << m_xappName << ": HANDOVER_TRIGGER rejected -- "
                        "candidate SINR " << candSinrDb << " dB < A1 threshold "
                        << policy.param2 << " dB (policy " << policy.policyId << ")");
            return false;
        }

        // (2) TTE-threshold check: reject a HO onto a target that will not stay
        //     in coverage long enough (candidate TTE below the A1 minimum).
        if (candTteS < policy.param1)
        {
            NS_LOG_INFO("xApp " << m_xappName << ": HANDOVER_TRIGGER rejected -- "
                        "candidate TTE " << candTteS << " s < A1 minimum "
                        << policy.param1 << " s (policy " << policy.policyId << ")");
            return false;
        }

        // (3) HO-rate check: enforce A1NtnPolicy::maxHandoverRate (HO per minute)
        //     over a sliding 60 s window of previously-accepted HOs.
        if (policy.maxHandoverRate > 0.0)
        {
            const Time window = Seconds(60.0);
            while (!m_recentHoTimes.empty() && (now - m_recentHoTimes.front()) > window)
            {
                m_recentHoTimes.pop_front();
            }
            if (static_cast<double>(m_recentHoTimes.size()) >= policy.maxHandoverRate)
            {
                NS_LOG_INFO("xApp " << m_xappName << ": HANDOVER_TRIGGER rejected -- "
                            "HO rate " << m_recentHoTimes.size() << "/min reached A1 cap "
                            << policy.maxHandoverRate << "/min (policy "
                            << policy.policyId << ")");
                return false;
            }
        }
    }

    // Compliant with every HO_THRESHOLD policy: record this HO so it counts
    // toward the rate limit for subsequent decisions.
    m_recentHoTimes.push_back(now);
    return true;
}

// ---- RC action submission ----

bool
OranNtnXappBase::SubmitAction(const E2RcAction& action)
{
    NS_LOG_FUNCTION(this << m_xappName << static_cast<uint8_t>(action.actionType));

    m_actionSubmitted(m_xappId, action);

    if (!m_ric)
    {
        NS_LOG_WARN("xApp " << m_xappName << ": No RIC connected, action dropped");
        m_metrics.failedActions++;
        m_actionResult(m_xappId, action, false);
        return false;
    }

    bool success = m_ric->ProcessXappAction(m_xappId, action);

    if (success)
    {
        m_metrics.successfulActions++;
    }
    else
    {
        m_metrics.failedActions++;
    }

    m_actionResult(m_xappId, action, success);
    return success;
}

// ---- Conflict handling ----

void
OranNtnXappBase::HandleConflict(const XappConflict& conflict)
{
    NS_LOG_FUNCTION(this << m_xappName);
    m_metrics.conflictsEncountered++;

    if (conflict.winnerId == m_xappId)
    {
        m_metrics.conflictsWon++;
    }

    m_conflictEncountered(m_xappId, conflict);
}

// ---- Decision cycle ----

void
OranNtnXappBase::SetDecisionInterval(Time interval)
{
    m_decisionInterval = interval;
}

Time
OranNtnXappBase::GetDecisionInterval() const
{
    return m_decisionInterval;
}

// ---- Helpers ----

E2RcAction
OranNtnXappBase::BuildAction(E2RcActionType type, uint32_t targetGnb,
                               uint32_t targetUe, double confidence) const
{
    E2RcAction action;
    action.timestamp = Simulator::Now().GetSeconds();
    action.xappId = m_xappId;
    action.xappName = m_xappName;
    action.actionType = type;
    action.targetGnbId = targetGnb;
    action.targetUeId = targetUe;
    action.targetBeamId = 0;
    action.targetSliceId = 0;
    action.confidence = confidence;
    action.parameter1 = 0.0;
    action.parameter2 = 0.0;
    action.executed = false;
    return action;
}

void
OranNtnXappBase::RecordDecision(bool success, double confidence, double latency_ms)
{
    m_metrics.totalDecisions++;

    // Running average of confidence
    double n = static_cast<double>(m_metrics.totalDecisions);
    m_metrics.avgConfidence =
        m_metrics.avgConfidence * ((n - 1) / n) + confidence / n;

    // Running average of latency
    m_metrics.avgDecisionLatency_ms =
        m_metrics.avgDecisionLatency_ms * ((n - 1) / n) + latency_ms / n;

    // Store the raw sample so the helper can derive P50/P95/P99 at write time.
    // Negative or zero samples (e.g. callers that did not instrument chrono)
    // are skipped to avoid polluting the percentile distribution.
    if (latency_ms > 0.0)
    {
        m_metrics.decisionLatencies_ms.push_back(latency_ms);
    }

    m_decisionConfidence(m_xappId, confidence);
}

void
OranNtnXappBase::DecisionCycleWrapper()
{
    if (m_state != XappState::RUNNING)
    {
        return;
    }

    DecisionCycle();

    m_decisionEvent = Simulator::Schedule(m_decisionInterval,
                                           &OranNtnXappBase::DecisionCycleWrapper,
                                           this);
}

void
OranNtnXappBase::SetupSubscriptions()
{
    if (!m_ric)
    {
        return;
    }

    auto e2term = m_ric->GetE2Termination();
    E2Subscription sub = GetRequiredSubscription();

    // Subscribe to all connected E2 nodes
    auto nodeIds = e2term->GetConnectedNodeIds();
    for (uint32_t gnbId : nodeIds)
    {
        uint32_t subId = e2term->CreateSubscription(gnbId, sub);
        if (subId > 0)
        {
            m_subscriptionIds.push_back(subId);

            // Register callback so E2 indications route to this xApp
            e2term->RegisterXappCallback(
                subId,
                m_xappId,
                MakeCallback(&OranNtnXappBase::HandleIndicationFromE2, this));
        }
    }

    NS_LOG_INFO("xApp " << m_xappName << ": Created " << m_subscriptionIds.size()
                << " E2 subscriptions across " << nodeIds.size() << " nodes");
}

void
OranNtnXappBase::HandleIndicationFromE2(uint32_t xappId, E2Indication indication)
{
    if (indication.ranFunctionId == 2) // KPM
    {
        HandleKpmIndication(indication.subscriptionId, indication.kpmReport);
    }
}

} // namespace ns3
