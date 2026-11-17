/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-only
 * Author: Muhammad Uzair
 *
 * O-RAN A1 Interface for NTN
 *
 * Implements A1 policy management between Non-RT RIC (SMO) and
 * Near-RT RIC per O-RAN.WG2.A1-v03.01.
 *
 * NTN Extensions:
 *   - Orbit-aware policy lifecycle (policy validity tied to pass windows)
 *   - Multi-plane policy coordination (inter-plane, intra-plane)
 *   - Federated policy aggregation across ground stations
 *   - Delay-tolerant policy distribution via ISL or store-and-forward
 */

#ifndef ORAN_NTN_A1_INTERFACE_H
#define ORAN_NTN_A1_INTERFACE_H

#include "oran-ntn-types.h"

#include <ns3/callback.h>
#include <ns3/event-id.h>
#include <ns3/nstime.h>
#include <ns3/object.h>
#include <ns3/traced-callback.h>

#include <functional>
#include <map>
#include <ostream>
#include <set>
#include <string>
#include <vector>

namespace ns3
{

// ============================================================================
//  A1 Policy Extensions for NTN
// ============================================================================

/**
 * \brief Extended A1 policy with NTN-specific fields
 */
struct A1NtnPolicy : public A1Policy
{
    // Spatial scope
    uint32_t orbitalPlaneId;       //!< 0 = all planes
    uint32_t satelliteId;          //!< 0 = all satellites in plane
    uint32_t beamGroupId;          //!< 0 = all beams

    // Temporal scope
    Time validFrom;                //!< Policy activation time
    Time validUntil;               //!< Policy expiry time
    bool orbitAware;               //!< Tie validity to satellite pass

    // Federated learning policy
    double federatedAggregationWeight;  //!< Weight for model aggregation
    uint32_t minParticipants;           //!< Minimum satellites for FL round
    Time flRoundInterval;               //!< Federated learning round period

    // QoS enforcement
    double maxHandoverRate;        //!< Max HO/min for this scope
    double minCoverageOverlap;     //!< Min beam overlap fraction for HO
};

// ============================================================================
//  A1 policy status (Wave 6B)
// ============================================================================

/// O-RAN A1TD v09.00 (ETSI TS 103 988) 6.2.2.2: enforceStatus of a policy.
enum class A1EnforceStatus : uint8_t
{
    ENFORCED = 0,
    NOT_ENFORCED = 1,
};
/// A1TD 6.2.2.3: enforceReason, present when NOT_ENFORCED (NONE otherwise).
enum class A1EnforceReason : uint8_t
{
    NONE = 0,
    SCOPE_NOT_APPLICABLE = 1,     //!< the scope names nothing this Near-RT RIC manages
    STATEMENT_NOT_APPLICABLE = 2, //!< no function here can act on the statement
    OTHER_REASON = 3,             //!< e.g. arrived after validUntil, or lapsed
};
const char* A1EnforceStatusName(A1EnforceStatus s);
const char* A1EnforceReasonName(A1EnforceReason r);

/// One status report as received by the Non-RT RIC.
struct A1PolicyStatusRecord
{
    uint32_t policyId{0};
    A1EnforceStatus status{A1EnforceStatus::NOT_ENFORCED};
    A1EnforceReason reason{A1EnforceReason::OTHER_REASON};
    Time reportedAt; //!< when the Near-RT RIC issued it
    Time receivedAt; //!< when the Non-RT RIC got it (A1 delay later)
};

// ============================================================================
//  A1 Policy Manager (Non-RT RIC side)
// ============================================================================

/**
 * \ingroup oran-ntn
 * \brief A1 Policy Manager at the Non-RT RIC / SMO
 *
 * Creates, distributes, and monitors A1 policies. Implements
 * policy lifecycle management with NTN-aware extensions including
 * orbit-scoped validity and delay-tolerant distribution.
 */
class OranNtnA1PolicyManager : public Object
{
  public:
    static TypeId GetTypeId();
    OranNtnA1PolicyManager();
    ~OranNtnA1PolicyManager() override;

    // ---- Policy CRUD ----
    /**
     * \brief Create and distribute a new A1 policy
     * \return policy ID
     */
    uint32_t CreatePolicy(const A1NtnPolicy& policy);

    /**
     * \brief Update an existing policy (triggers re-distribution)
     */
    bool UpdatePolicy(uint32_t policyId, const A1NtnPolicy& policy);

    /**
     * \brief Delete/deactivate a policy
     */
    bool DeletePolicy(uint32_t policyId);

    /**
     * \brief Get current policy by ID
     */
    A1NtnPolicy GetPolicy(uint32_t policyId) const;

    /**
     * \brief Get all active policies of a given type
     */
    std::vector<A1NtnPolicy> GetPoliciesByType(A1PolicyType type) const;

    /**
     * \brief Get all policies applicable to a specific satellite
     */
    std::vector<A1NtnPolicy> GetPoliciesForSatellite(uint32_t satId) const;

    // ---- Policy enforcement feedback ----
    typedef Callback<void, uint32_t, bool, std::string> PolicyFeedbackCallback;

    /**
     * \brief Receive enforcement status feedback from Near-RT RIC
     */
    void HandlePolicyFeedback(uint32_t policyId, bool enforced,
                               std::string status);

    void SetPolicyFeedbackCallback(PolicyFeedbackCallback cb);

    /**
     * \brief Wave 6B: receive an A1TD policy status (enforceStatus and
     *        enforceReason) from the Near-RT RIC.
     *
     * Stores the latest status per policy, the full history, and the time
     * each policy spent ENFORCED, and forwards to the legacy feedback path
     * (m_policyEnforced trace, violation count, PolicyFeedbackCallback with
     * the reason name as the status string).
     */
    void HandlePolicyStatus(A1PolicyStatusRecord rec);
    /// Latest status known for \p policyId (NOT_ENFORCED/OTHER_REASON if none).
    A1PolicyStatusRecord GetPolicyStatus(uint32_t policyId) const;
    const std::vector<A1PolicyStatusRecord>& GetStatusHistory() const { return m_statusHistory; }
    /// Share of time since its first status report that \p policyId was ENFORCED.
    double GetEnforcedTimeShare(uint32_t policyId) const;
    /// CSV: policy_id,reported_s,received_s,enforce_status,enforce_reason
    void WritePolicyStatusCsv(std::ostream& os) const;

    // ---- NTN-specific ----
    /**
     * \brief Generate orbit-aware handover threshold policies
     *
     * Analyzes constellation geometry and creates per-plane policies
     * with appropriate SINR/TTE thresholds based on orbital parameters.
     */
    void GenerateOrbitAwarePolicies(uint32_t numPlanes, uint32_t satsPerPlane,
                                     double inclinationDeg, double altitudeKm);

    /**
     * \brief Generate slice SLA policies from service requirements
     */
    void GenerateSlicePolicies(const std::vector<SliceConfig>& slices);

    // ---- Distribution ----
    typedef Callback<void, A1NtnPolicy> PolicyDistributionCallback;
    void SetDistributionCallback(PolicyDistributionCallback cb);

    /**
     * \brief ORAN-15: one-way A1 delivery delay from the Non-RT RIC to the
     *        Near-RT RIC.
     *
     * DistributePolicy invoked the callback INLINE, with no
     * Simulator::Schedule, so a ground SMO policy reached the Near-RT RIC at
     * the same simulation instant even when that RIC is modelled as
     * satellite-hosted (OranNtnRicPlacement::Site::OnBoardSatellite). A1 is the
     * slowest of the O-RAN interfaces and the one most affected by an NTN
     * feeder leg, so instantaneous delivery is the least defensible place to
     * omit a delay.
     *
     * Defaults to zero, which is the previous behaviour: a scenario that has
     * not thought about A1 latency should not silently acquire one. Set it from
     * the placement the scenario actually models.
     */
    void SetDeliveryDelay(Time d) { m_deliveryDelay = d; }
    Time GetDeliveryDelay() const { return m_deliveryDelay; }
    /// Policies dispatched but not yet delivered (ORAN-15).
    uint32_t GetPoliciesInFlight() const { return m_inFlight; }
    /// ORAN-15 test seam: dispatch a policy down the distribution path.
    /// DistributePolicy is private and is normally reached through the policy
    /// lifecycle, which is why the inline delivery went unnoticed.
    void DistributePolicyForTest(const A1NtnPolicy& p) { DistributePolicy(p); }

    // ---- Metrics ----
    uint32_t GetTotalPolicies() const;
    uint32_t GetActivePolicies() const;
    uint32_t GetViolationCount() const;

    TracedCallback<uint32_t, A1NtnPolicy> m_policyCreated;
    TracedCallback<uint32_t, bool> m_policyEnforced;
    TracedCallback<uint32_t, std::string> m_policyViolation;

  protected:
    void DoDispose() override;

  private:
    void CheckPolicyExpiry();
    void DistributePolicy(const A1NtnPolicy& policy);

    std::map<uint32_t, A1NtnPolicy> m_policies;
    uint32_t m_nextPolicyId;
    EventId m_expiryCheckEvent;
    Time m_expiryCheckInterval;

    PolicyDistributionCallback m_distributionCb;
    Time m_deliveryDelay{Seconds(0)}; //!< ORAN-15: one-way A1 delivery delay
    uint32_t m_inFlight{0};           //!< ORAN-15: dispatched, not yet delivered
    PolicyFeedbackCallback m_feedbackCb;

    uint32_t m_violationCount;

    struct StatusTrack
    {
        A1PolicyStatusRecord last;
        Time first;
        Time enforcedAccum;
    };
    std::map<uint32_t, StatusTrack> m_status;
    std::vector<A1PolicyStatusRecord> m_statusHistory;
};

// ============================================================================
//  A1 Adapter (Near-RT RIC side - receives policies)
// ============================================================================

/**
 * \ingroup oran-ntn
 * \brief A1 Adapter at the Near-RT RIC
 *
 * Receives A1 policies from Non-RT RIC, stores them, and makes them
 * available to xApps for guiding their decisions.
 */
class OranNtnA1Adapter : public Object
{
  public:
    static TypeId GetTypeId();
    OranNtnA1Adapter();
    ~OranNtnA1Adapter() override;

    /**
     * \brief Handle incoming A1 policy from Non-RT RIC
     */
    void HandleIncomingPolicy(A1NtnPolicy policy);

    /**
     * \brief Get all active policies (for xApps to query)
     */
    std::vector<A1NtnPolicy> GetActivePolicies() const;

    /**
     * \brief Get policies by type
     */
    std::vector<A1NtnPolicy> GetPoliciesByType(A1PolicyType type) const;

    /**
     * \brief Get the most restrictive threshold for a given policy type
     */
    double GetThreshold(A1PolicyType type, const std::string& paramName) const;

    /**
     * \brief Report policy enforcement status back to Non-RT RIC
     */
    void ReportEnforcement(uint32_t policyId, bool enforced,
                            std::string status);

    typedef Callback<void, uint32_t, bool, std::string> FeedbackCallback;
    void SetFeedbackCallback(FeedbackCallback cb);

    // ---- Wave 6B: A1TD policy status ----
    /// Where status reports go (the Non-RT RIC), and the A1 delay back to it.
    typedef Callback<void, A1PolicyStatusRecord> StatusCallback;
    void SetStatusCallback(StatusCallback cb) { m_statusCb = cb; }
    void SetFeedbackDelay(Time d) { m_feedbackDelay = d; }
    Time GetFeedbackDelay() const { return m_feedbackDelay; }
    /// Scopes this Near-RT RIC manages ("satellite:<id>", "slice:<id>", ...).
    /// "global" always applies. With none registered every scope applies
    /// (the pre-Wave-6B behavior).
    void AddManagedScope(const std::string& scope) { m_scopes.insert(scope); }
    /// Policy types some function here can enforce. With none registered
    /// every statement applies (the pre-Wave-6B behavior).
    void AddSupportedPolicyType(A1PolicyType t) { m_types.insert(static_cast<uint8_t>(t)); }
    /// Evaluate \p p as it arrives: ENFORCED, or NOT_ENFORCED with a reason.
    std::pair<A1EnforceStatus, A1EnforceReason> Evaluate(const A1NtnPolicy& p) const;
    /// Issue a status report for \p policyId (delivered after FeedbackDelay).
    void ReportPolicyStatus(uint32_t policyId, A1EnforceStatus s, A1EnforceReason r);
    uint64_t GetStatusReportsSent() const { return m_statusSent; }

    TracedCallback<A1NtnPolicy> m_policyReceived;
    TracedCallback<uint32_t, bool> m_enforcementReported;

  protected:
    void DoDispose() override;

  private:
    /// ORAN-31: true while \p p is within its validUntil (or has none).
    static bool IsLive(const A1NtnPolicy& p);

    std::map<uint32_t, A1NtnPolicy> m_activePolicies;
    FeedbackCallback m_feedbackCb;
    /// Lapse at validUntil: the policy stops being enforced and says so.
    void OnPolicyLapse(uint32_t policyId);

    StatusCallback m_statusCb;
    Time m_feedbackDelay{Seconds(0)};
    std::set<std::string> m_scopes;
    std::set<uint8_t> m_types;
    std::map<uint32_t, EventId> m_lapseEvents;
    uint64_t m_statusSent{0};
};

} // namespace ns3

#endif // ORAN_NTN_A1_INTERFACE_H
