/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-only
 * Author: Muhammad Uzair
 *
 * O-RAN xApp Base Class for NTN
 *
 * Abstract base class for Near-RT RIC xApps operating in NTN scenarios.
 * Provides E2 subscription management, A1 policy awareness, conflict
 * resolution interface, and NTN-specific timing abstractions.
 *
 * Concrete xApps:
 *   - HO Prediction xApp (proactive handover; "LSTM"=EWMA+linear slope,
 *     "DQN"=linear dot-product — see its header, not a deep net)
 *   - Beam Hopping xApp (dynamic beam scheduling)
 *   - Slice Manager xApp (eMBB/URLLC/mMTC resource allocation)
 *   - Doppler Compensation xApp (per-beam frequency correction)
 *   - TN-NTN Traffic Steering xApp (terrestrial/satellite selection)
 */

#ifndef ORAN_NTN_XAPP_BASE_H
#define ORAN_NTN_XAPP_BASE_H

#include "oran-ntn-e2-interface.h"
#include "oran-ntn-types.h"
#include "oran-ntn-a1-interface.h"

#include <ns3/callback.h>
#include <ns3/event-id.h>
#include <ns3/nstime.h>
#include <ns3/object.h>
#include <ns3/traced-callback.h>

#include <deque>
#include <map>
#include <string>
#include <vector>
#include <vector>

namespace ns3
{

class OranNtnNearRtRic; // forward declaration

// ============================================================================
//  xApp State & Metrics
// ============================================================================

/**
 * \brief xApp runtime state
 */
enum class XappState : uint8_t
{
    REGISTERED = 0,
    RUNNING = 1,
    SUSPENDED = 2,
    FAILED = 3,
    DEREGISTERED = 4,
};

/**
 * \brief xApp performance metrics
 *
 * `decisionLatencies_ms` stores the wall-clock decision-cycle latency
 * (std::chrono::high_resolution_clock) for every decision the xApp makes.
 * The helper sorts this vector at write time to derive P50/P95/P99
 * percentiles for the per-xApp metrics CSV.  The vector is sampled in
 * the xApp's DecisionCycle() body and is independent of ns-3 simulation
 * time, so its values reflect implementation complexity rather than
 * scheduled simulation events.
 */
struct XappMetrics
{
    uint32_t totalDecisions;
    /// ORAN-13: actions the RIC ACCEPTED AND ROUTED to an E2 node.
    ///
    /// This is not the same thing as an action that changed the simulated
    /// network, and the difference used to be invisible.
    /// OranNtnNearRtRic::ProcessXappAction returns
    /// `m_e2Term->RouteRcAction(action)`, i.e. whether the action passed policy
    /// and conflict checks and reached a node - while the helper's
    /// action_log.csv records `success = actuated`, i.e. whether an actuator
    /// existed and fired. Both were called "success", so xapp_metrics.csv and
    /// action_log.csv could report different numbers for what read as the same
    /// quantity, and a scenario with no actuator wired reported successes for
    /// actions that moved nothing.
    ///
    /// Kept under its original name so existing readers are not silently
    /// re-pointed at a different number; what it counts is now stated.
    uint32_t successfulActions;
    /// ORAN-13: actions that an actuator actually carried out.
    ///
    /// Fed by the action-result path rather than by routing. The gap between
    /// this and successfulActions is exactly the set of actions that were
    /// accepted and reached a node where nothing was wired to act on them,
    /// which is the decision-island condition this campaign keeps finding.
    uint32_t actuatedActions;
    uint32_t failedActions;
    uint32_t conflictsEncountered;
    uint32_t conflictsWon;
    double avgDecisionLatency_ms;
    double avgConfidence;
    Time uptime;
    std::vector<double> decisionLatencies_ms;
};

// ============================================================================
//  xApp Base Class
// ============================================================================

/**
 * \ingroup oran-ntn
 * \brief Abstract base class for O-RAN xApps in NTN scenarios
 *
 * Provides common infrastructure for all xApps: E2 data ingestion,
 * A1 policy compliance checking, RC action submission, conflict
 * handling, and performance tracking.
 *
 * Subclasses implement ProcessKpmReport() and DecisionCycle() to
 * define application-specific intelligence.
 */
class OranNtnXappBase : public Object
{
  public:
    static TypeId GetTypeId();
    OranNtnXappBase();
    ~OranNtnXappBase() override;

    // ---- Identity ----
    void SetXappId(uint32_t id);
    void SetXappName(const std::string& name);
    void SetPriority(uint8_t priority);

    uint32_t GetXappId() const;
    std::string GetXappName() const;
    uint8_t GetPriority() const;
    XappState GetState() const;
    XappMetrics GetMetrics() const;

    // ---- Lifecycle ----
    /**
     * \brief Start the xApp (begin subscribing and processing)
     */
    virtual void Start();

    /**
     * \brief Stop the xApp gracefully
     */
    virtual void Stop();

    /**
     * \brief Suspend (pause decision cycles but keep subscriptions)
     */
    virtual void Suspend();

    /**
     * \brief Resume from suspended state
     */
    virtual void Resume();

    // ---- RIC connection ----
    void SetRic(Ptr<OranNtnNearRtRic> ric);

    // ---- E2 data ingestion ----
    /**
     * \brief Receive E2 KPM indication from RIC
     *
     * Called by RIC when a subscribed E2 indication arrives.
     * Stores in internal database and triggers processing.
     */
    void HandleKpmIndication(uint32_t subscriptionId, const E2KpmReport& report);

    /**
     * \brief Get latest KPM report for a given gNB
     */
    E2KpmReport GetLatestReport(uint32_t gnbId) const;

    /**
     * \brief Get all reports within a time window
     */
    std::vector<E2KpmReport> GetReportsInWindow(Time window) const;

    /**
     * \brief Get reports for a specific UE within a time window
     */
    std::vector<E2KpmReport> GetUeReportsInWindow(uint32_t ueId, Time window) const;

    /**
     * \brief Record the UE's TRUE serving cell (the cell it is RRC-connected to).
     *
     * The KPM database is keyed by gnbId, so "the last report in the window" is a
     * std::map ordering artifact (highest gnbId), NOT the serving cell. Callers
     * that know the real serving cell — the RAN via NtnRealStackHelper
     * ::GetUeServingCellId, or this xApp itself when it actuates a HANDOVER_TRIGGER —
     * publish it here so serving-cell KPIs and handover decisions read the correct
     * baseline. O-RAN WG3 E2SM-KPM identifies the serving cell by CGI/PCI, not by
     * report arrival order.
     */
    void SetUeServingCell(uint32_t ueId, uint32_t gnbId);

    /**
     * \brief The UE's tracked serving cell, or false if none has been published.
     */
    bool GetUeServingCell(uint32_t ueId, uint32_t& gnbId) const;

    /**
     * \brief The UE's most recent report FROM ITS SERVING CELL.
     *
     * Resolves serving via SetUeServingCell(); if the serving cell is unknown it
     * falls back to the most RECENT report by timestamp (defensible), never the
     * highest-gnbId artifact. \p found is false when the UE has no report in window.
     */
    E2KpmReport GetUeServingReport(uint32_t ueId, Time window, bool& found) const;

    // ---- A1 policy access ----
    /**
     * \brief Get applicable A1 policies for this xApp's domain
     */
    std::vector<A1Policy> GetApplicablePolicies() const;

    /**
     * \brief Check if a proposed action complies with A1 policies
     */
    /// ORAN-13: record that an action was actually carried out by an actuator.
    ///
    /// Called by whoever owns the actuator (OranNtnHelper does), because the
    /// xApp cannot see past the RIC's routing decision. Without this the only
    /// number an xApp could report was routing acceptance, under the name
    /// "successful".
    void RecordActuation(bool actuated)
    {
        if (actuated)
        {
            m_metrics.actuatedActions++;
        }
    }

    bool CheckPolicyCompliance(const E2RcAction& action) const;
    /// ORAN-07: enforce a SLICE_SLA policy against a PRB-allocation action.
    ///
    /// Before this, CheckPolicyCompliance returned true for every action that
    /// was not a HANDOVER_TRIGGER, so ten of the eleven A1 policy types
    /// governed nothing and their violation counters could never leave zero.
    bool CheckSlicePolicyCompliance(const E2RcAction& action,
                                    const std::vector<A1NtnPolicy>& policies) const;

    // ---- RC action submission ----
    /**
     * \brief Submit an RC action to the RIC for conflict checking and execution
     * \return true if action was accepted by RIC
     */
    bool SubmitAction(const E2RcAction& action);

    // ---- Conflict handling ----
    /**
     * \brief Notify xApp that its action conflicted with another xApp
     */
    virtual void HandleConflict(const XappConflict& conflict);

    // ---- Decision cycle ----
    /**
     * \brief Set the decision cycle interval
     */
    void SetDecisionInterval(Time interval);
    Time GetDecisionInterval() const;

    // ---- Trace sources ----
    TracedCallback<uint32_t, E2RcAction> m_actionSubmitted;
    TracedCallback<uint32_t, E2RcAction, bool> m_actionResult;
    TracedCallback<uint32_t, XappConflict> m_conflictEncountered;
    TracedCallback<uint32_t, double> m_decisionConfidence;

  protected:
    void DoDispose() override;

    // ---- Abstract methods (subclasses MUST implement) ----

    /**
     * \brief Process a new KPM report (application-specific logic)
     *
     * Called when new E2 data arrives. Update internal state, models,
     * feature vectors, etc.
     */
    virtual void ProcessKpmReport(const E2KpmReport& report) = 0;

    /**
     * \brief Execute one decision cycle
     *
     * Called periodically at m_decisionInterval. Analyze accumulated
     * data, run inference, generate RC actions.
     */
    virtual void DecisionCycle() = 0;

    /**
     * \brief Get the E2 subscription configuration this xApp needs
     */
    virtual E2Subscription GetRequiredSubscription() const = 0;

    // ---- Helpers for subclasses ----

    /**
     * \brief Build an E2RcAction with this xApp's identity pre-filled
     */
    E2RcAction BuildAction(E2RcActionType type, uint32_t targetGnb,
                            uint32_t targetUe, double confidence) const;

    /**
     * \brief Record a decision for metrics tracking
     */
    void RecordDecision(bool success, double confidence, double latency_ms);

    // Access to internal data
    Ptr<OranNtnNearRtRic> m_ric;

    // KPM database: gnbId -> deque of recent reports
    std::map<uint32_t, std::deque<E2KpmReport>> m_kpmDatabase;
    uint32_t m_maxReportsPerGnb;
    Time m_dataRetentionWindow;

    // UE -> serving cell (gnbId), published by the RAN or by this xApp on handover.
    std::map<uint32_t, uint32_t> m_ueServingCell;

  private:
    void DecisionCycleWrapper();
    void SetupSubscriptions();
    void HandleIndicationFromE2(uint32_t xappId, E2Indication indication);

    uint32_t m_xappId;
    std::string m_xappName;
    uint8_t m_priority;
    XappState m_state;
    XappMetrics m_metrics;

    Time m_decisionInterval;
    EventId m_decisionEvent;
    std::vector<uint32_t> m_subscriptionIds;

    // Timestamps of recent HANDOVER_TRIGGER actions this xApp passed the A1
    // HO-rate policy check for. Used by CheckPolicyCompliance() to enforce
    // A1NtnPolicy::maxHandoverRate (HO/min). Mutable: the compliance check is
    // logically const but must record the accepted HO to rate-limit the next.
    mutable std::deque<Time> m_recentHoTimes;
};

} // namespace ns3

#endif // ORAN_NTN_XAPP_BASE_H
