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
    PolicyFeedbackCallback m_feedbackCb;

    uint32_t m_violationCount;
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

    TracedCallback<A1NtnPolicy> m_policyReceived;
    TracedCallback<uint32_t, bool> m_enforcementReported;

  protected:
    void DoDispose() override;

  private:
    std::map<uint32_t, A1NtnPolicy> m_activePolicies;
    FeedbackCallback m_feedbackCb;
};

} // namespace ns3

#endif // ORAN_NTN_A1_INTERFACE_H
