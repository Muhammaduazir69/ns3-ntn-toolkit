/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-only
 * Author: Muhammad Uzair
 *
 * O-RAN E2 Interface for NTN
 *
 * Implements E2AP protocol between E2 Nodes (NTN gNBs / satellites)
 * and the Near-RT RIC. Supports E2SM-KPM (monitoring) and E2SM-RC
 * (control) service models per O-RAN.WG3.E2AP-v03.01.
 *
 * Adaptations for NTN:
 *   - Feeder-link-aware E2 message scheduling (batch during visibility)
 *   - Propagation-delay-tolerant subscription management
 *   - On-board buffering for store-and-forward E2 reports
 *   - ISL relay for multi-hop E2 connectivity
 */

#ifndef ORAN_NTN_E2_INTERFACE_H
#define ORAN_NTN_E2_INTERFACE_H

#include "oran-ntn-types.h"

#include <ns3/callback.h>
#include <ns3/event-id.h>
#include <ns3/nstime.h>
#include <ns3/object.h>
#include <ns3/traced-callback.h>

#include <cstdint>
#include <deque>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace ns3
{

// ============================================================================
//  E2 Subscription Management
// ============================================================================

/**
 * \brief Subscription request from Near-RT RIC to E2 Node
 */
struct E2Subscription
{
    uint32_t subscriptionId;
    uint32_t ricRequestorId;
    uint32_t ranFunctionId;        //!< 2 = KPM, 3 = RC
    Time reportingPeriod;          //!< KPM reporting interval
    bool eventTrigger;             //!< true = event-driven, false = periodic
    double eventThreshold;         //!< Threshold for event trigger

    // NTN extensions
    bool batchOnVisibility;        //!< Batch reports during feeder link windows
    Time maxBufferAge;             //!< Max age before discarding buffered reports
    bool useIslRelay;              //!< Route E2 via ISL if feeder link unavailable
};

/**
 * \brief E2 indication (report or insert) sent from E2 Node to Near-RT RIC
 */
struct E2Indication
{
    uint32_t subscriptionId;
    uint32_t ranFunctionId;
    double timestamp;
    E2KpmReport kpmReport;          //!< Populated if ranFunctionId == 2
    bool isBuffered;                //!< Was this report buffered (NTN delay)
    Time originalTimestamp;         //!< When measurement was actually taken
    Time deliveryDelay;             //!< Feeder link + processing delay
};

// ============================================================================
//  E2 Node (runs on each NTN gNB / satellite)
// ============================================================================

/**
 * \ingroup oran-ntn
 * \brief E2 Node agent running on NTN gNB or on-board satellite processor
 *
 * Manages E2 subscriptions, collects KPM metrics, executes RC actions,
 * and handles NTN-specific challenges (buffering, ISL relay, timing).
 */
class OranNtnE2Node : public Object
{
  public:
    static TypeId GetTypeId();
    OranNtnE2Node();
    ~OranNtnE2Node() override;

    // ---- Configuration ----
    void SetNodeId(uint32_t gnbId);
    void SetIsNtn(bool isNtn);
    void SetFeederLinkDelay(Time delay);
    void SetOnBoardBufferSize(uint32_t maxReports);

    uint32_t GetNodeId() const;
    bool IsNtn() const;

    // ---- E2 Setup ----
    /**
     * \brief Register supported RAN functions (KPM, RC)
     */
    void RegisterRanFunction(uint32_t functionId, const std::string& description);

    // ---- Subscription handling ----
    /**
     * \brief Process incoming E2 subscription request from RIC
     */
    bool HandleSubscriptionRequest(const E2Subscription& sub);
    void HandleSubscriptionDelete(uint32_t subscriptionId);
    std::vector<E2Subscription> GetActiveSubscriptions() const;

    // ---- KPM Reporting ----
    /**
     * \brief Submit a KPM measurement from the local gNB/satellite stack
     *
     * The E2 Node will buffer and deliver according to subscription config.
     */
    void SubmitKpmMeasurement(const E2KpmReport& report);

    /**
     * \brief Force flush all buffered reports (e.g., feeder link just became available)
     */
    void FlushBufferedReports();

    // ---- RC Action execution ----
    typedef Callback<bool, E2RcAction> RcActionCallback;

    /**
     * \brief Register callback for executing RC actions from xApps
     */
    void SetRcActionCallback(RcActionCallback cb);

    /**
     * \brief Execute an RC action received from the RIC
     * \return true if action was executed successfully
     */
    bool ExecuteRcAction(const E2RcAction& action);

    // ---- NTN-specific ----
    /**
     * \brief Notify E2 node that feeder link is available/unavailable
     */
    void SetFeederLinkAvailable(bool available);
    bool IsFeederLinkAvailable() const;

    /**
     * \brief Get buffer occupancy (fraction 0-1)
     */
    double GetBufferOccupancy() const;

    // ---- Callbacks to RIC ----
    typedef Callback<void, E2Indication> IndicationCallback;
    void SetIndicationCallback(IndicationCallback cb);

    // ---- Trace sources ----
    TracedCallback<uint32_t, E2KpmReport> m_kpmReportSent;
    TracedCallback<uint32_t, E2RcAction, bool> m_rcActionExecuted;
    TracedCallback<uint32_t, uint32_t> m_reportBuffered;   //!< gnbId, bufferSize
    TracedCallback<uint32_t, uint32_t> m_reportDropped;    //!< gnbId, droppedCount

  protected:
    void DoDispose() override;

  private:
    void PeriodicReportTimer(uint32_t subscriptionId);
    void DeliverReport(const E2Indication& indication);
    void CheckBufferAge();

    uint32_t m_gnbId;
    bool m_isNtn;
    Time m_feederLinkDelay;
    bool m_feederLinkAvailable;
    uint32_t m_maxBufferSize;

    std::map<uint32_t, E2Subscription> m_subscriptions;
    std::map<uint32_t, EventId> m_reportTimers;
    std::deque<E2Indication> m_reportBuffer;
    std::map<uint32_t, std::string> m_ranFunctions;

    RcActionCallback m_rcActionCb;
    IndicationCallback m_indicationCb;

    EventId m_bufferCheckEvent;
    uint32_t m_totalReportsSent;
    uint32_t m_totalReportsDropped;
    uint32_t m_totalActionsExecuted;
};

// ============================================================================
//  E2 Termination (runs on Near-RT RIC side)
// ============================================================================

/**
 * \ingroup oran-ntn
 * \brief E2 Termination point at the Near-RT RIC
 *
 * Receives E2 indications from all E2 Nodes, forwards to subscribed xApps,
 * and routes RC actions from xApps back to appropriate E2 Nodes.
 */
class OranNtnE2Termination : public Object
{
  public:
    static TypeId GetTypeId();
    OranNtnE2Termination();
    ~OranNtnE2Termination() override;

    // ---- E2 Node management ----
    void RegisterE2Node(Ptr<OranNtnE2Node> node);
    void DeregisterE2Node(uint32_t gnbId);
    Ptr<OranNtnE2Node> GetE2Node(uint32_t gnbId) const;
    std::vector<uint32_t> GetConnectedNodeIds() const;

    // ---- Subscription management ----
    /**
     * \brief Create E2 subscription (called by xApps via RIC)
     * \return subscription ID, 0 on failure
     */
    uint32_t CreateSubscription(uint32_t gnbId, const E2Subscription& sub);
    void DeleteSubscription(uint32_t subscriptionId);

    /**
     * \brief Create subscription to ALL connected E2 nodes
     */
    std::vector<uint32_t> CreateGlobalSubscription(const E2Subscription& sub);

    // ---- Indication routing ----
    typedef Callback<void, uint32_t, E2Indication> XappIndicationCallback;

    /**
     * \brief Register xApp to receive indications for a subscription
     */
    void RegisterXappCallback(uint32_t subscriptionId, uint32_t xappId,
                               XappIndicationCallback cb);

    // ---- RC action routing ----
    /**
     * \brief Route an RC action from an xApp to the target E2 Node
     */
    bool RouteRcAction(const E2RcAction& action);

    // ---- Metrics ----
    uint32_t GetTotalIndicationsReceived() const;
    uint32_t GetTotalActionsRouted() const;

    TracedCallback<uint32_t, E2Indication> m_indicationReceived;
    TracedCallback<E2RcAction, bool> m_actionRouted;

  protected:
    void DoDispose() override;

  private:
    void HandleIndication(E2Indication indication);

    std::map<uint32_t, Ptr<OranNtnE2Node>> m_e2Nodes;

    struct SubscriptionRoute
    {
        uint32_t gnbId;
        E2Subscription subscription;
        std::map<uint32_t, XappIndicationCallback> xappCallbacks;
    };
    std::map<uint32_t, SubscriptionRoute> m_subscriptionRoutes;
    uint32_t m_nextSubscriptionId;

    uint32_t m_totalIndications;
    uint32_t m_totalActions;
};

} // namespace ns3

#endif // ORAN_NTN_E2_INTERFACE_H
