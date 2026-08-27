/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-only
 * Author: Muhammad Uzair
 */

#include "oran-ntn-e2-interface.h"
#include "oran-ntn-rc-style3.h"
#include "oran-ntn-service-model-rc.h"

#include <ns3/boolean.h>
#include <ns3/double.h>
#include <ns3/log.h>
#include <ns3/simulator.h>
#include <ns3/uinteger.h>

#include <optional>

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
            .AddAttribute("AlignToControlLoop",
                          "When true, indications that crossed the feeder link are "
                          "queued and dispatched to the RIC/xApp callback only on the "
                          "next control-loop tick (measure -> feeder -> loop tick -> "
                          "feeder -> apply) instead of inline execution.",
                          BooleanValue(false),
                          MakeBooleanAccessor(&OranNtnE2Node::m_alignToControlLoop),
                          MakeBooleanChecker())
            .AddAttribute("ControlLoopPeriod",
                          "Near-RT RIC control-loop tick period used when "
                          "AlignToControlLoop is enabled.",
                          TimeValue(MilliSeconds(100)),
                          MakeTimeAccessor(&OranNtnE2Node::m_controlLoopPeriod),
                          MakeTimeChecker())
            .AddAttribute("UnixEpochOffset",
                          "Offset (seconds) added to indication timestamps. 0 keeps "
                          "pure simulation time; set a Unix epoch to produce wall-"
                          "clock-like stamps for external consumers (FlexRIC bridge).",
                          DoubleValue(0.0),
                          MakeDoubleAccessor(&OranNtnE2Node::m_unixEpochOffset),
                          MakeDoubleChecker<double>(0.0))
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
      m_alignToControlLoop(false),
      m_controlLoopPeriod(MilliSeconds(100)),
      m_unixEpochOffset(0.0),
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
    Simulator::Cancel(m_alignedDispatchEvent);
    m_alignedQueue.clear();
    m_subscriptions.clear();
    m_reportBuffer.clear();
    m_ranFunctions.clear();
    m_rcActionCb = MakeNullCallback<bool, E2RcAction>();
    m_indicationCb = MakeNullCallback<void, E2Indication>();
    m_loopProbe = nullptr;
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

    // R2.7 stage `e2_indication_construct`: the CPU cost of turning a KPM
    // measurement into E2 indications and enqueueing their delayed delivery.
    // This is the ON-PATH stand-in for encode + transport hand-off; it is NOT
    // an E2AP wire serializer (E2AP-over-SCTP is not simulated -- see the
    // class doc), which is why the stage is named for message construction.
    std::optional<OranNtnLoopLatencyProbe::ScopedCpuTimer> constructTimer;
    if (m_loopProbe)
    {
        constructTimer.emplace(m_loopProbe, oranntn::loopstage::kE2IndicationConstruct);
    }

    // ORAN-14: remember the most recent measurement so the periodic timer has
    // something to publish at the subscription cadence. Keyed per UE, because a
    // node serves several and the last one to arrive is not the whole picture.
    m_latestReport[report.ueId] = report;
    m_haveLatestReport = true;

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

        // ORAN-14: one implementation, shared with the periodic timer, so the
        // two delivery paths cannot drift apart.
        EmitIndication(sub, report);
    }
}

void
OranNtnE2Node::EmitIndication(const E2Subscription& sub, const E2KpmReport& report)
{
    E2Indication indication;
    indication.subscriptionId = sub.subscriptionId;
    indication.ranFunctionId = 2;
    // UnixEpochOffset = 0 keeps pure sim time; nonzero yields Unix-like
    // stamps so an external RIC (FlexRIC bridge) accepts the indication.
    indication.timestamp = m_unixEpochOffset + Simulator::Now().GetSeconds();
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
    else
    {
        // ORAN-05: the feeder is down and this subscription does not batch,
        // so the indication cannot be delivered. It used to fall through
        // both branches and vanish with no counter and no trace, so a
        // scenario could lose every KPM report of an outage and report
        // nothing unusual. Count it: telemetry lost to an outage is a
        // result, not an absence.
        m_totalReportsDropped++;
        m_reportDropped(m_gnbId, m_totalReportsDropped);
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

    if (m_loopProbe)
    {
        // R2.7 stage `e2_indication_feeder_uplink`: simulated time from the
        // measurement instant to arrival on the RIC side of the feeder link.
        m_loopProbe->RecordSimulatedStage(oranntn::loopstage::kIndicationFeederUplink,
                                          Simulator::Now() - indication.originalTimestamp);
    }

    if (m_alignToControlLoop)
    {
        // Hold the indication until the next Near-RT RIC control-loop tick so
        // xApps cannot react inline (the optimistic-loop artifact found in the
        // 2026-06-12 audit). Tick boundaries are multiples of
        // m_controlLoopPeriod on the simulation clock.
        m_alignedQueue.push_back(indication);
        if (!m_alignedDispatchEvent.IsPending())
        {
            const Time now = Simulator::Now();
            const int64_t period = m_controlLoopPeriod.GetNanoSeconds();
            const int64_t nextTick = ((now.GetNanoSeconds() / period) + 1) * period;
            m_alignedDispatchEvent = Simulator::Schedule(NanoSeconds(nextTick) - now,
                                                         &OranNtnE2Node::DispatchAlignedIndications,
                                                         this);
        }
        return;
    }
    DispatchIndication(indication);
}

void
OranNtnE2Node::DispatchIndication(const E2Indication& indication)
{
    if (m_loopProbe)
    {
        // R2.7 stage `e2_indication_transport`: measurement instant -> the
        // instant the xApp callback is entered. Measured directly, so it
        // includes both the feeder leg and any RIC tick alignment.
        const Time transport = Simulator::Now() - indication.originalTimestamp;
        m_loopProbe->RecordSimulatedStage(oranntn::loopstage::kIndicationTransport,
                                          transport);
        // `ric_tick_align_wait`: whatever the transport spent waiting for the
        // next Near-RT RIC tick beyond the delivery delay already charged to
        // the feeder (indication.deliveryDelay). Exactly zero when
        // AlignToControlLoop is off.
        const Time tickWait = transport - indication.deliveryDelay;
        m_loopProbe->RecordSimulatedStage(
            oranntn::loopstage::kRicTickAlignWait,
            tickWait.IsStrictlyPositive() ? tickWait : Time());
    }
    if (!m_indicationCb.IsNull())
    {
        m_indicationCb(indication);
    }
}

void
OranNtnE2Node::DispatchAlignedIndications()
{
    NS_LOG_FUNCTION(this << m_alignedQueue.size());
    std::deque<E2Indication> batch;
    batch.swap(m_alignedQueue);
    for (const auto& ind : batch)
    {
        DispatchIndication(ind);
    }
}

void
OranNtnE2Node::SetRcActionCallback(RcActionCallback cb)
{
    m_rcActionCb = cb;
}

Time
OranNtnE2Node::GetFeederLinkDelay() const
{
    return m_feederLinkDelay;
}

void
OranNtnE2Node::ReceiveRcAction(const E2RcAction& action)
{
    NS_LOG_FUNCTION(this << static_cast<uint8_t>(action.actionType));
    // Return feeder path: the RIC's command crosses the feeder link too, so a
    // full control loop costs one feeder delay in EACH direction (E2AP/SCTP
    // itself is not simulated; see the class doc).
    // The routing instant travels with the event so the downlink leg is
    // MEASURED at actuation time rather than assumed to equal FeederLinkDelay.
    Simulator::Schedule(m_feederLinkDelay,
                        &OranNtnE2Node::ExecuteRcActionEvent,
                        this,
                        action,
                        Simulator::Now());
}

void
OranNtnE2Node::ExecuteRcActionEvent(E2RcAction action, Time routedAt)
{
    if (m_loopProbe)
    {
        // R2.7 stage `rc_action_transport`: the downlink feeder leg.
        m_loopProbe->RecordSimulatedStage(oranntn::loopstage::kRcActionTransport,
                                          Simulator::Now() - routedAt);
    }
    ExecuteRcAction(action);
}

bool
OranNtnE2Node::ExecuteRcAction(const E2RcAction& action)
{
    NS_LOG_FUNCTION(this << static_cast<uint8_t>(action.actionType));

    // ORAN-10: decode before actuating. If the termination encoded a Style 3
    // ControlMessage, this node must be able to read it back and must find the
    // same target cell in it. A control whose bytes do not decode, or decode to
    // a different cell, is not executed - which is what makes the codec part of
    // the simulation rather than a parallel exercise.
    if (!action.smControlMessage.empty())
    {
        using namespace oranntn::rc_v103::style3;
        OranNtnServiceModelRc rc;
        ControlMessage decoded{};
        if (!rc.DecodeControl(action.smControlMessage, &decoded))
        {
            NS_LOG_WARN("E2Node " << m_gnbId << ": RC control message failed to decode; "
                                  << "not actuating");
            m_rcActionExecuted(m_gnbId, action, false);
            return false;
        }
        if (const auto* hc = std::get_if<HandoverControl>(&decoded.action))
        {
            const uint64_t expect = NrCellIdentityFrom(action.targetGnbId);
            if (hc->target_primary_cell_id.nr_cell_identity != expect)
            {
                NS_LOG_WARN("E2Node " << m_gnbId << ": decoded RC control names cell "
                                      << hc->target_primary_cell_id.nr_cell_identity
                                      << " but the action targets " << expect
                                      << "; not actuating");
                m_rcActionExecuted(m_gnbId, action, false);
                return false;
            }
        }
        ++m_rcControlsDecoded;
    }

    bool success = false;
    if (!m_rcActionCb.IsNull())
    {
        // R2.7 stage `actuation`: CPU cost of applying the action to the radio
        // (the E2 node's registered RC action callback).
        std::optional<OranNtnLoopLatencyProbe::ScopedCpuTimer> actTimer;
        if (m_loopProbe)
        {
            actTimer.emplace(m_loopProbe, oranntn::loopstage::kActuation);
        }
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
OranNtnE2Node::SetLoopLatencyProbe(Ptr<OranNtnLoopLatencyProbe> probe)
{
    m_loopProbe = probe;
}

Ptr<OranNtnLoopLatencyProbe>
OranNtnE2Node::GetLoopLatencyProbe() const
{
    return m_loopProbe;
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

    // ORAN-14: actually report.
    //
    // This used to look the subscription up and do nothing but re-arm, so every
    // xApp's reportingPeriod - 100 to 500 ms across the shipped set - had no
    // effect whatsoever, and indications appeared only when a scenario happened
    // to call SubmitKpmMeasurement on its own schedule. E2SM-KPM periodic
    // report style means the RAN publishes its current measurements at the
    // subscription's cadence, which is what this now does.
    //
    // Gated on SetPeriodicReporting because every shipped xApp requests a
    // period: emitting by default would change the indication count of every
    // existing scenario at once, and the counts in the committed results were
    // measured without it.
    if (m_periodicReporting && !it->second.eventTrigger && it->second.ranFunctionId == 2 &&
        m_haveLatestReport)
    {
        for (const auto& [ueId, rep] : m_latestReport)
        {
            (void)ueId;
            EmitIndication(it->second, rep);
            ++m_periodicIndications;
        }
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
                            "An RC action was accepted for delivery to an E2 node "
                            "(bool = accepted/scheduled, NOT executed: execution "
                            "happens one FeederLinkDelay later and is reported by "
                            "OranNtnE2Node's RcActionExecuted trace)",
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
OranNtnE2Termination::RouteRcAction(const E2RcAction& inAction)
{
    NS_LOG_FUNCTION(this << inAction.targetGnbId);

    // ORAN-10: put the service model on the control path.
    //
    // This used to pass the raw C++ struct straight through to the E2 node, so
    // ConvertE2RcToStyle3 and the RC codec had callers only in the test suite -
    // an encode-only universe running alongside the simulation rather than
    // inside it. Encoding here and decoding at the node makes the codec
    // load-bearing: a broken Style-3 mapping now breaks the run.
    //
    // Only HANDOVER_TRIGGER and HANDOVER_CANCEL have a Style-3 mapping. Other
    // action types are delivered as the struct, and the empty smControlMessage
    // says so rather than implying an encode that did not happen.
    E2RcAction action = inAction;
    if (auto sm = oranntn::rc_v103::style3::ConvertE2RcToStyle3(action))
    {
        oranntn::rc_v103::style3::ControlMessage msg{};
        msg.action = *sm;
        OranNtnServiceModelRc rc;
        action.smControlMessage = rc.EncodeControl(msg);
        if (action.smControlMessage.empty())
        {
            NS_LOG_WARN("E2 Termination: RC action has a Style 3 mapping but failed to "
                        "encode; refusing to route an action we could not put on the wire");
            m_actionRouted(action, false);
            return false;
        }
    }

    // Cell-wide actions (targetGnbId=0) fan out to every registered E2 node.
    // E2AP (O-RAN.WG3.E2AP §8.4) has no "accept and discard" outcome: an RC
    // Control Request either reaches E2 nodes or it fails.
    if (action.targetGnbId == 0)
    {
        if (m_e2Nodes.empty())
        {
            NS_LOG_WARN("E2 Termination: Cell-wide RC action cannot be routed, "
                        "no E2 nodes registered");
            m_actionRouted(action, false);
            return false;
        }

        for (const auto& [gnbId, node] : m_e2Nodes)
        {
            // Delivery crosses the downlink feeder link; execution is deferred
            // by FeederLinkDelay inside ReceiveRcAction().
            node->ReceiveRcAction(action);
        }

        m_totalActions++;
        m_actionRouted(action, true);
        NS_LOG_DEBUG("E2 Termination: Cell-wide RC action fanned out to "
                     << m_e2Nodes.size() << " E2 node(s) (type="
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

    // Route over the RETURN feeder path rather than calling ExecuteRcAction()
    // inline: the RIC's command physically crosses the feeder link (2-15 ms for
    // LEO), so it must not actuate at the instant it is decided.
    it->second->ReceiveRcAction(action);
    m_totalActions++;

    // The outcome is unknowable here -- execution happens one FeederLinkDelay
    // from now. `true` means ACCEPTED/SCHEDULED for delivery, not "executed".
    // The executed/failed outcome is reported by the E2 node's
    // OranNtnE2Node::m_rcActionExecuted trace once the delay elapses.
    m_actionRouted(action, true);
    return true;
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
