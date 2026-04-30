/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-only
 * Author: Muhammad Uzair
 */

#include "oran-ntn-e2-interface.h"

#include <ns3/log.h>
#include <ns3/simulator.h>
#include <ns3/uinteger.h>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("OranNtnE2Interface");

// ============================================================================
//  OranNtnE2Node
// ============================================================================

NS_OBJECT_ENSURE_REGISTERED(OranNtnE2Node);

TypeId
OranNtnE2Node::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::OranNtnE2Node")
            .SetParent<Object>()
            .SetGroupName("OranNtn")
            .AddConstructor<OranNtnE2Node>()
            .AddAttribute("FeederLinkDelay",
                          "One-way feeder link delay",
                          TimeValue(MilliSeconds(20)),
                          MakeTimeAccessor(&OranNtnE2Node::m_feederLinkDelay),
                          MakeTimeChecker())
            .AddAttribute("MaxBufferSize",
                          "Maximum number of buffered reports",
                          UintegerValue(1000),
                          MakeUintegerAccessor(&OranNtnE2Node::m_maxBufferSize),
                          MakeUintegerChecker<uint32_t>())
            .AddTraceSource("KpmReportSent",
                            "A KPM report was sent to the RIC",
                            MakeTraceSourceAccessor(&OranNtnE2Node::m_kpmReportSent),
                            "ns3::OranNtnE2Node::KpmReportTracedCallback")
            .AddTraceSource("RcActionExecuted",
                            "An RC action was executed",
                            MakeTraceSourceAccessor(&OranNtnE2Node::m_rcActionExecuted),
                            "ns3::OranNtnE2Node::RcActionTracedCallback")
            .AddTraceSource("ReportBuffered",
                            "A report was buffered due to feeder link unavailability",
                            MakeTraceSourceAccessor(&OranNtnE2Node::m_reportBuffered),
                            "ns3::OranNtnE2Node::BufferTracedCallback")
            .AddTraceSource("ReportDropped",
                            "A report was dropped due to buffer overflow or age",
                            MakeTraceSourceAccessor(&OranNtnE2Node::m_reportDropped),
                            "ns3::OranNtnE2Node::BufferTracedCallback");
    return tid;
}

OranNtnE2Node::OranNtnE2Node()
    : m_gnbId(0),
      m_isNtn(false),
      m_feederLinkDelay(MilliSeconds(20)),
      m_feederLinkAvailable(true),
      m_maxBufferSize(1000),
      m_totalReportsSent(0),
      m_totalReportsDropped(0),
      m_totalActionsExecuted(0)
{
    NS_LOG_FUNCTION(this);
}

OranNtnE2Node::~OranNtnE2Node()
{
    NS_LOG_FUNCTION(this);
}

void
OranNtnE2Node::DoDispose()
{
    NS_LOG_FUNCTION(this);
    for (auto& [id, event] : m_reportTimers)
    {
        Simulator::Cancel(event);
    }
    m_reportTimers.clear();
    Simulator::Cancel(m_bufferCheckEvent);
    m_subscriptions.clear();
    m_reportBuffer.clear();
    m_ranFunctions.clear();
    m_rcActionCb = MakeNullCallback<bool, E2RcAction>();
    m_indicationCb = MakeNullCallback<void, E2Indication>();
    Object::DoDispose();
}

void
OranNtnE2Node::SetNodeId(uint32_t gnbId)
{
    m_gnbId = gnbId;
}

void
OranNtnE2Node::SetIsNtn(bool isNtn)
{
    m_isNtn = isNtn;
}

void
OranNtnE2Node::SetFeederLinkDelay(Time delay)
{
    m_feederLinkDelay = delay;
}

void
OranNtnE2Node::SetOnBoardBufferSize(uint32_t maxReports)
{
    m_maxBufferSize = maxReports;
}

uint32_t
OranNtnE2Node::GetNodeId() const
{
    return m_gnbId;
}

bool
OranNtnE2Node::IsNtn() const
{
    return m_isNtn;
}

void
OranNtnE2Node::RegisterRanFunction(uint32_t functionId, const std::string& description)
{
    NS_LOG_FUNCTION(this << functionId << description);
    m_ranFunctions[functionId] = description;
}

bool
OranNtnE2Node::HandleSubscriptionRequest(const E2Subscription& sub)
{
    NS_LOG_FUNCTION(this << sub.subscriptionId);

    if (m_ranFunctions.find(sub.ranFunctionId) == m_ranFunctions.end())
    {
        NS_LOG_WARN("E2Node " << m_gnbId << ": RAN function " << sub.ranFunctionId
                               << " not supported");
        return false;
    }

    m_subscriptions[sub.subscriptionId] = sub;

    // Start periodic reporting if configured
    if (!sub.eventTrigger && sub.reportingPeriod > Seconds(0))
    {
        EventId timer = Simulator::Schedule(sub.reportingPeriod,
                                             &OranNtnE2Node::PeriodicReportTimer,
                                             this,
                                             sub.subscriptionId);
        m_reportTimers[sub.subscriptionId] = timer;
    }

    NS_LOG_INFO("E2Node " << m_gnbId << ": Subscription " << sub.subscriptionId
                           << " accepted (function=" << sub.ranFunctionId
                           << ", period=" << sub.reportingPeriod.As(Time::MS) << ")");
    return true;
}

void
OranNtnE2Node::HandleSubscriptionDelete(uint32_t subscriptionId)
{
    NS_LOG_FUNCTION(this << subscriptionId);
    auto it = m_reportTimers.find(subscriptionId);
    if (it != m_reportTimers.end())
    {
        Simulator::Cancel(it->second);
        m_reportTimers.erase(it);
    }
    m_subscriptions.erase(subscriptionId);
}

std::vector<E2Subscription>
OranNtnE2Node::GetActiveSubscriptions() const
{
    std::vector<E2Subscription> result;
    result.reserve(m_subscriptions.size());
    for (const auto& [id, sub] : m_subscriptions)
    {
        result.push_back(sub);
    }
    return result;
}

void
OranNtnE2Node::SubmitKpmMeasurement(const E2KpmReport& report)
{
    NS_LOG_FUNCTION(this << report.gnbId);

    // Create indication for each active KPM subscription
    for (const auto& [subId, sub] : m_subscriptions)
    {
        if (sub.ranFunctionId != 2)
        {
            continue; // Not KPM
        }

        // Check event trigger if applicable
        if (sub.eventTrigger)
        {
            bool triggered = false;
            if (report.sinr_dB < sub.eventThreshold)
            {
                triggered = true;
            }
            if (!triggered)
            {
                continue;
            }
        }

        E2Indication indication;
        indication.subscriptionId = subId;
        indication.ranFunctionId = 2;
        indication.timestamp = Simulator::Now().GetSeconds();
        indication.kpmReport = report;
        indication.originalTimestamp = Simulator::Now();

        if (m_feederLinkAvailable)
        {
            indication.isBuffered = false;
            indication.deliveryDelay = m_feederLinkDelay;
            // Schedule delivery with feeder link delay
            Simulator::Schedule(m_feederLinkDelay,
                                &OranNtnE2Node::DeliverReport,
                                this,
                                indication);
        }
        else if (sub.batchOnVisibility)
        {
            // Buffer for later delivery
            indication.isBuffered = true;
            if (m_reportBuffer.size() < m_maxBufferSize)
            {
                m_reportBuffer.push_back(indication);
                m_reportBuffered(m_gnbId, static_cast<uint32_t>(m_reportBuffer.size()));
            }
            else
            {
                // Drop oldest
                m_reportBuffer.pop_front();
                m_reportBuffer.push_back(indication);
                m_totalReportsDropped++;
                m_reportDropped(m_gnbId, m_totalReportsDropped);
            }
        }
    }
}

void
OranNtnE2Node::FlushBufferedReports()
{
    NS_LOG_FUNCTION(this);
    Time now = Simulator::Now();

    while (!m_reportBuffer.empty())
    {
        E2Indication ind = m_reportBuffer.front();
        m_reportBuffer.pop_front();
        ind.deliveryDelay = now - ind.originalTimestamp + m_feederLinkDelay;
        Simulator::Schedule(m_feederLinkDelay,
                            &OranNtnE2Node::DeliverReport,
                            this,
                            ind);
    }
}

void
OranNtnE2Node::DeliverReport(const E2Indication& indication)
{
    NS_LOG_FUNCTION(this << indication.subscriptionId);
    m_totalReportsSent++;
    m_kpmReportSent(m_gnbId, indication.kpmReport);

    if (!m_indicationCb.IsNull())
    {
        m_indicationCb(indication);
    }
}

void
OranNtnE2Node::SetRcActionCallback(RcActionCallback cb)
{
    m_rcActionCb = cb;
}

bool
OranNtnE2Node::ExecuteRcAction(const E2RcAction& action)
{
    NS_LOG_FUNCTION(this << static_cast<uint8_t>(action.actionType));

    bool success = false;
    if (!m_rcActionCb.IsNull())
    {
        success = m_rcActionCb(action);
    }

    if (success)
    {
        m_totalActionsExecuted++;
    }

    m_rcActionExecuted(m_gnbId, action, success);
    NS_LOG_INFO("E2Node " << m_gnbId << ": RC action "
                           << static_cast<uint8_t>(action.actionType)
                           << (success ? " executed" : " FAILED"));
    return success;
}

void
OranNtnE2Node::SetFeederLinkAvailable(bool available)
{
    NS_LOG_FUNCTION(this << available);
    bool wasUnavailable = !m_feederLinkAvailable;
    m_feederLinkAvailable = available;

    if (available && wasUnavailable && !m_reportBuffer.empty())
    {
        NS_LOG_INFO("E2Node " << m_gnbId << ": Feeder link restored, flushing "
                               << m_reportBuffer.size() << " buffered reports");
        FlushBufferedReports();
    }
}

bool
OranNtnE2Node::IsFeederLinkAvailable() const
{
    return m_feederLinkAvailable;
}

double
OranNtnE2Node::GetBufferOccupancy() const
{
    if (m_maxBufferSize == 0)
    {
        return 0.0;
    }
    return static_cast<double>(m_reportBuffer.size()) / m_maxBufferSize;
}

void
OranNtnE2Node::SetIndicationCallback(IndicationCallback cb)
{
    m_indicationCb = cb;
}

void
OranNtnE2Node::PeriodicReportTimer(uint32_t subscriptionId)
{
    NS_LOG_FUNCTION(this << subscriptionId);
    auto it = m_subscriptions.find(subscriptionId);
    if (it == m_subscriptions.end())
    {
        return;
    }

    // Re-schedule next report
    m_reportTimers[subscriptionId] =
        Simulator::Schedule(it->second.reportingPeriod,
                            &OranNtnE2Node::PeriodicReportTimer,
                            this,
                            subscriptionId);
}

void
OranNtnE2Node::CheckBufferAge()
{
    Time now = Simulator::Now();
    auto it = m_reportBuffer.begin();
    while (it != m_reportBuffer.end())
    {
        auto sub = m_subscriptions.find(it->subscriptionId);
        if (sub != m_subscriptions.end())
        {
            Time age = now - it->originalTimestamp;
            if (age > sub->second.maxBufferAge)
            {
                it = m_reportBuffer.erase(it);
                m_totalReportsDropped++;
                m_reportDropped(m_gnbId, m_totalReportsDropped);
                continue;
            }
        }
        ++it;
    }
}

// ============================================================================
//  OranNtnE2Termination
// ============================================================================

NS_OBJECT_ENSURE_REGISTERED(OranNtnE2Termination);

TypeId
OranNtnE2Termination::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::OranNtnE2Termination")
            .SetParent<Object>()
            .SetGroupName("OranNtn")
            .AddConstructor<OranNtnE2Termination>()
            .AddTraceSource("IndicationReceived",
                            "An E2 indication was received from an E2 node",
                            MakeTraceSourceAccessor(
                                &OranNtnE2Termination::m_indicationReceived),
                            "ns3::OranNtnE2Termination::IndicationTracedCallback")
            .AddTraceSource("ActionRouted",
                            "An RC action was routed to an E2 node",
                            MakeTraceSourceAccessor(
                                &OranNtnE2Termination::m_actionRouted),
                            "ns3::OranNtnE2Termination::ActionTracedCallback");
    return tid;
}

OranNtnE2Termination::OranNtnE2Termination()
    : m_nextSubscriptionId(1),
      m_totalIndications(0),
      m_totalActions(0)
{
    NS_LOG_FUNCTION(this);
}

OranNtnE2Termination::~OranNtnE2Termination()
{
    NS_LOG_FUNCTION(this);
}

void
OranNtnE2Termination::DoDispose()
{
    NS_LOG_FUNCTION(this);
    m_e2Nodes.clear();
    m_subscriptionRoutes.clear();
    Object::DoDispose();
}

void
OranNtnE2Termination::RegisterE2Node(Ptr<OranNtnE2Node> node)
{
    NS_LOG_FUNCTION(this << node->GetNodeId());
    uint32_t gnbId = node->GetNodeId();
    m_e2Nodes[gnbId] = node;

    // Set indication callback to route through this termination
    node->SetIndicationCallback(
        MakeCallback(&OranNtnE2Termination::HandleIndication, this));

    NS_LOG_INFO("E2 Termination: Registered E2 node " << gnbId
                << (node->IsNtn() ? " (NTN)" : " (TN)"));
}

void
OranNtnE2Termination::DeregisterE2Node(uint32_t gnbId)
{
    NS_LOG_FUNCTION(this << gnbId);
    m_e2Nodes.erase(gnbId);
}

Ptr<OranNtnE2Node>
OranNtnE2Termination::GetE2Node(uint32_t gnbId) const
{
    auto it = m_e2Nodes.find(gnbId);
    if (it != m_e2Nodes.end())
    {
        return it->second;
    }
    return nullptr;
}

std::vector<uint32_t>
OranNtnE2Termination::GetConnectedNodeIds() const
{
    std::vector<uint32_t> ids;
    ids.reserve(m_e2Nodes.size());
    for (const auto& [id, node] : m_e2Nodes)
    {
        ids.push_back(id);
    }
    return ids;
}

uint32_t
OranNtnE2Termination::CreateSubscription(uint32_t gnbId, const E2Subscription& sub)
{
    NS_LOG_FUNCTION(this << gnbId << sub.ranFunctionId);

    auto nodeIt = m_e2Nodes.find(gnbId);
    if (nodeIt == m_e2Nodes.end())
    {
        NS_LOG_WARN("E2 Termination: E2 node " << gnbId << " not found");
        return 0;
    }

    E2Subscription subCopy = sub;
    subCopy.subscriptionId = m_nextSubscriptionId++;

    if (!nodeIt->second->HandleSubscriptionRequest(subCopy))
    {
        return 0;
    }

    SubscriptionRoute route;
    route.gnbId = gnbId;
    route.subscription = subCopy;
    m_subscriptionRoutes[subCopy.subscriptionId] = route;

    NS_LOG_INFO("E2 Termination: Created subscription " << subCopy.subscriptionId
                << " for gnb " << gnbId);
    return subCopy.subscriptionId;
}

void
OranNtnE2Termination::DeleteSubscription(uint32_t subscriptionId)
{
    NS_LOG_FUNCTION(this << subscriptionId);
    auto it = m_subscriptionRoutes.find(subscriptionId);
    if (it != m_subscriptionRoutes.end())
    {
        auto nodeIt = m_e2Nodes.find(it->second.gnbId);
        if (nodeIt != m_e2Nodes.end())
        {
            nodeIt->second->HandleSubscriptionDelete(subscriptionId);
        }
        m_subscriptionRoutes.erase(it);
    }
}

std::vector<uint32_t>
OranNtnE2Termination::CreateGlobalSubscription(const E2Subscription& sub)
{
    NS_LOG_FUNCTION(this);
    std::vector<uint32_t> ids;
    for (const auto& [gnbId, node] : m_e2Nodes)
    {
        uint32_t subId = CreateSubscription(gnbId, sub);
        if (subId > 0)
        {
            ids.push_back(subId);
        }
    }
    return ids;
}

void
OranNtnE2Termination::RegisterXappCallback(uint32_t subscriptionId, uint32_t xappId,
                                             XappIndicationCallback cb)
{
    NS_LOG_FUNCTION(this << subscriptionId << xappId);
    auto it = m_subscriptionRoutes.find(subscriptionId);
    if (it != m_subscriptionRoutes.end())
    {
        it->second.xappCallbacks[xappId] = cb;
    }
}

bool
OranNtnE2Termination::RouteRcAction(const E2RcAction& action)
{
    NS_LOG_FUNCTION(this << action.targetGnbId);

    // Cell-wide actions (targetGnbId=0) are broadcast-style; accept directly
    if (action.targetGnbId == 0)
    {
        m_totalActions++;
        m_actionRouted(action, true);
        NS_LOG_DEBUG("E2 Termination: Cell-wide RC action accepted (type="
                     << static_cast<uint8_t>(action.actionType) << ")");
        return true;
    }

    auto it = m_e2Nodes.find(action.targetGnbId);
    if (it == m_e2Nodes.end())
    {
        NS_LOG_WARN("E2 Termination: Target E2 node " << action.targetGnbId
                     << " not found for RC action");
        m_actionRouted(action, false);
        return false;
    }

    bool success = it->second->ExecuteRcAction(action);
    m_totalActions++;
    m_actionRouted(action, success);
    return success;
}

void
OranNtnE2Termination::HandleIndication(E2Indication indication)
{
    NS_LOG_FUNCTION(this << indication.subscriptionId);
    m_totalIndications++;
    m_indicationReceived(indication.subscriptionId, indication);

    auto it = m_subscriptionRoutes.find(indication.subscriptionId);
    if (it != m_subscriptionRoutes.end())
    {
        for (const auto& [xappId, cb] : it->second.xappCallbacks)
        {
            cb(xappId, indication);
        }
    }
}

uint32_t
OranNtnE2Termination::GetTotalIndicationsReceived() const
{
    return m_totalIndications;
}

uint32_t
OranNtnE2Termination::GetTotalActionsRouted() const
{
    return m_totalActions;
}

} // namespace ns3
