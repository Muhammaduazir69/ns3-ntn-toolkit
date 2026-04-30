/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-only
 * Author: Muhammad Uzair
 */

#include "oran-ntn-a1-interface.h"

#include <ns3/log.h>
#include <ns3/simulator.h>

#include <algorithm>
#include <cmath>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("OranNtnA1Interface");

// ============================================================================
//  OranNtnA1PolicyManager (Non-RT RIC side)
// ============================================================================

NS_OBJECT_ENSURE_REGISTERED(OranNtnA1PolicyManager);

TypeId
OranNtnA1PolicyManager::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::OranNtnA1PolicyManager")
            .SetParent<Object>()
            .SetGroupName("OranNtn")
            .AddConstructor<OranNtnA1PolicyManager>()
            .AddAttribute("ExpiryCheckInterval",
                          "How often to check for expired policies",
                          TimeValue(Seconds(10)),
                          MakeTimeAccessor(&OranNtnA1PolicyManager::m_expiryCheckInterval),
                          MakeTimeChecker())
            .AddTraceSource("PolicyCreated",
                            "A new A1 policy was created",
                            MakeTraceSourceAccessor(&OranNtnA1PolicyManager::m_policyCreated),
                            "ns3::OranNtnA1PolicyManager::PolicyTracedCallback")
            .AddTraceSource("PolicyEnforced",
                            "Policy enforcement feedback received",
                            MakeTraceSourceAccessor(&OranNtnA1PolicyManager::m_policyEnforced),
                            "ns3::OranNtnA1PolicyManager::EnforcementTracedCallback")
            .AddTraceSource("PolicyViolation",
                            "A policy violation was detected",
                            MakeTraceSourceAccessor(&OranNtnA1PolicyManager::m_policyViolation),
                            "ns3::OranNtnA1PolicyManager::ViolationTracedCallback");
    return tid;
}

OranNtnA1PolicyManager::OranNtnA1PolicyManager()
    : m_nextPolicyId(1),
      m_expiryCheckInterval(Seconds(10)),
      m_violationCount(0)
{
    NS_LOG_FUNCTION(this);
}

OranNtnA1PolicyManager::~OranNtnA1PolicyManager()
{
    NS_LOG_FUNCTION(this);
}

void
OranNtnA1PolicyManager::DoDispose()
{
    NS_LOG_FUNCTION(this);
    Simulator::Cancel(m_expiryCheckEvent);
    m_policies.clear();
    m_distributionCb = MakeNullCallback<void, A1NtnPolicy>();
    m_feedbackCb = MakeNullCallback<void, uint32_t, bool, std::string>();
    Object::DoDispose();
}

uint32_t
OranNtnA1PolicyManager::CreatePolicy(const A1NtnPolicy& policy)
{
    NS_LOG_FUNCTION(this);
    A1NtnPolicy p = policy;
    p.policyId = m_nextPolicyId++;
    p.active = true;
    p.timestamp = Simulator::Now().GetSeconds();

    m_policies[p.policyId] = p;
    m_policyCreated(p.policyId, p);

    DistributePolicy(p);

    // Start expiry checking if not running
    if (!m_expiryCheckEvent.IsPending())
    {
        m_expiryCheckEvent = Simulator::Schedule(m_expiryCheckInterval,
                                                  &OranNtnA1PolicyManager::CheckPolicyExpiry,
                                                  this);
    }

    NS_LOG_INFO("A1 PolicyManager: Created policy " << p.policyId
                << " type=" << static_cast<uint8_t>(p.type)
                << " scope=" << p.scope);
    return p.policyId;
}

bool
OranNtnA1PolicyManager::UpdatePolicy(uint32_t policyId, const A1NtnPolicy& policy)
{
    NS_LOG_FUNCTION(this << policyId);
    auto it = m_policies.find(policyId);
    if (it == m_policies.end())
    {
        return false;
    }

    A1NtnPolicy p = policy;
    p.policyId = policyId;
    p.timestamp = Simulator::Now().GetSeconds();
    it->second = p;

    DistributePolicy(p);
    return true;
}

bool
OranNtnA1PolicyManager::DeletePolicy(uint32_t policyId)
{
    NS_LOG_FUNCTION(this << policyId);
    auto it = m_policies.find(policyId);
    if (it == m_policies.end())
    {
        return false;
    }
    it->second.active = false;
    DistributePolicy(it->second);
    m_policies.erase(it);
    return true;
}

A1NtnPolicy
OranNtnA1PolicyManager::GetPolicy(uint32_t policyId) const
{
    auto it = m_policies.find(policyId);
    if (it != m_policies.end())
    {
        return it->second;
    }
    return A1NtnPolicy{};
}

std::vector<A1NtnPolicy>
OranNtnA1PolicyManager::GetPoliciesByType(A1PolicyType type) const
{
    std::vector<A1NtnPolicy> result;
    for (const auto& [id, p] : m_policies)
    {
        if (p.type == type && p.active)
        {
            result.push_back(p);
        }
    }
    return result;
}

std::vector<A1NtnPolicy>
OranNtnA1PolicyManager::GetPoliciesForSatellite(uint32_t satId) const
{
    std::vector<A1NtnPolicy> result;
    for (const auto& [id, p] : m_policies)
    {
        if (!p.active)
        {
            continue;
        }
        // Global scope or specific satellite match
        if (p.satelliteId == 0 || p.satelliteId == satId)
        {
            result.push_back(p);
        }
    }
    return result;
}

void
OranNtnA1PolicyManager::HandlePolicyFeedback(uint32_t policyId, bool enforced,
                                               std::string status)
{
    NS_LOG_FUNCTION(this << policyId << enforced << status);
    m_policyEnforced(policyId, enforced);

    if (!enforced)
    {
        m_violationCount++;
        m_policyViolation(policyId, status);
    }

    if (!m_feedbackCb.IsNull())
    {
        m_feedbackCb(policyId, enforced, status);
    }
}

void
OranNtnA1PolicyManager::SetPolicyFeedbackCallback(PolicyFeedbackCallback cb)
{
    m_feedbackCb = cb;
}

void
OranNtnA1PolicyManager::GenerateOrbitAwarePolicies(uint32_t numPlanes,
                                                     uint32_t satsPerPlane,
                                                     double inclinationDeg,
                                                     double altitudeKm)
{
    NS_LOG_FUNCTION(this << numPlanes << satsPerPlane << inclinationDeg << altitudeKm);

    // Compute orbital period: T = 2*pi * sqrt(a^3 / mu)
    double earthRadiusKm = 6371.0;
    double muEarth = 398600.4418; // km^3/s^2
    double semiMajorAxis = earthRadiusKm + altitudeKm;
    double orbitalPeriod = 2.0 * M_PI * std::sqrt(std::pow(semiMajorAxis, 3) / muEarth);

    // Beam dwell time estimate (assuming ground track speed ~7 km/s for LEO)
    double groundSpeed = 2.0 * M_PI * earthRadiusKm * std::cos(inclinationDeg * M_PI / 180.0)
                         / orbitalPeriod;
    double beamRadius = altitudeKm * std::tan(3.0 * M_PI / 180.0); // ~3 deg beamwidth
    double beamDwellTime = 2.0 * beamRadius / groundSpeed;

    // Create per-plane HO threshold policies
    for (uint32_t plane = 0; plane < numPlanes; plane++)
    {
        A1NtnPolicy hoPolicy;
        hoPolicy.type = A1PolicyType::HO_THRESHOLD;
        hoPolicy.scope = "plane:" + std::to_string(plane);
        hoPolicy.priority = 10;
        hoPolicy.orbitalPlaneId = plane + 1;
        hoPolicy.satelliteId = 0; // all sats in plane
        hoPolicy.beamGroupId = 0;
        hoPolicy.orbitAware = true;
        hoPolicy.active = true;

        // TTE threshold = 20% of beam dwell time
        hoPolicy.param1 = beamDwellTime * 0.2;  // TTE minimum (seconds)
        hoPolicy.param2 = -3.0;                  // SINR threshold (dB)
        hoPolicy.param3 = 50000.0;               // D1 distance threshold (m)
        hoPolicy.maxHandoverRate = 60.0 / beamDwellTime; // max 1 HO per dwell

        CreatePolicy(hoPolicy);
    }

    NS_LOG_INFO("A1 PolicyManager: Generated " << numPlanes
                << " orbit-aware HO policies. Orbital period="
                << orbitalPeriod << "s, beam dwell=" << beamDwellTime << "s");
}

void
OranNtnA1PolicyManager::GenerateSlicePolicies(const std::vector<SliceConfig>& slices)
{
    NS_LOG_FUNCTION(this << slices.size());

    for (const auto& slice : slices)
    {
        A1NtnPolicy slicePolicy;
        slicePolicy.type = A1PolicyType::SLICE_SLA;
        slicePolicy.scope = "slice:" + std::to_string(slice.sliceId);
        slicePolicy.priority = 5; // high priority for SLA
        slicePolicy.orbitalPlaneId = 0;
        slicePolicy.satelliteId = 0;
        slicePolicy.beamGroupId = 0;
        slicePolicy.orbitAware = false;
        slicePolicy.active = true;

        slicePolicy.param1 = slice.minThroughput_Mbps;
        slicePolicy.param2 = slice.maxLatency_ms;
        slicePolicy.param3 = slice.reliabilityTarget;

        CreatePolicy(slicePolicy);
    }
}

void
OranNtnA1PolicyManager::SetDistributionCallback(PolicyDistributionCallback cb)
{
    m_distributionCb = cb;
}

uint32_t
OranNtnA1PolicyManager::GetTotalPolicies() const
{
    return static_cast<uint32_t>(m_policies.size());
}

uint32_t
OranNtnA1PolicyManager::GetActivePolicies() const
{
    uint32_t count = 0;
    for (const auto& [id, p] : m_policies)
    {
        if (p.active)
        {
            count++;
        }
    }
    return count;
}

uint32_t
OranNtnA1PolicyManager::GetViolationCount() const
{
    return m_violationCount;
}

void
OranNtnA1PolicyManager::DistributePolicy(const A1NtnPolicy& policy)
{
    if (!m_distributionCb.IsNull())
    {
        m_distributionCb(policy);
    }
}

void
OranNtnA1PolicyManager::CheckPolicyExpiry()
{
    Time now = Simulator::Now();
    for (auto it = m_policies.begin(); it != m_policies.end();)
    {
        if (it->second.validUntil > Seconds(0) && now >= it->second.validUntil)
        {
            NS_LOG_INFO("A1 PolicyManager: Policy " << it->first << " expired");
            it->second.active = false;
            DistributePolicy(it->second);
            it = m_policies.erase(it);
        }
        else
        {
            ++it;
        }
    }

    if (!m_policies.empty())
    {
        m_expiryCheckEvent = Simulator::Schedule(m_expiryCheckInterval,
                                                  &OranNtnA1PolicyManager::CheckPolicyExpiry,
                                                  this);
    }
}

// ============================================================================
//  OranNtnA1Adapter (Near-RT RIC side)
// ============================================================================

NS_OBJECT_ENSURE_REGISTERED(OranNtnA1Adapter);

TypeId
OranNtnA1Adapter::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::OranNtnA1Adapter")
            .SetParent<Object>()
            .SetGroupName("OranNtn")
            .AddConstructor<OranNtnA1Adapter>()
            .AddTraceSource("PolicyReceived",
                            "A1 policy received from Non-RT RIC",
                            MakeTraceSourceAccessor(&OranNtnA1Adapter::m_policyReceived),
                            "ns3::OranNtnA1Adapter::PolicyTracedCallback")
            .AddTraceSource("EnforcementReported",
                            "Policy enforcement status reported",
                            MakeTraceSourceAccessor(&OranNtnA1Adapter::m_enforcementReported),
                            "ns3::OranNtnA1Adapter::EnforcementTracedCallback");
    return tid;
}

OranNtnA1Adapter::OranNtnA1Adapter()
{
    NS_LOG_FUNCTION(this);
}

OranNtnA1Adapter::~OranNtnA1Adapter()
{
    NS_LOG_FUNCTION(this);
}

void
OranNtnA1Adapter::DoDispose()
{
    m_activePolicies.clear();
    m_feedbackCb = MakeNullCallback<void, uint32_t, bool, std::string>();
    Object::DoDispose();
}

void
OranNtnA1Adapter::HandleIncomingPolicy(A1NtnPolicy policy)
{
    NS_LOG_FUNCTION(this << policy.policyId);

    if (policy.active)
    {
        m_activePolicies[policy.policyId] = policy;
        NS_LOG_INFO("A1 Adapter: Activated policy " << policy.policyId);
    }
    else
    {
        m_activePolicies.erase(policy.policyId);
        NS_LOG_INFO("A1 Adapter: Deactivated policy " << policy.policyId);
    }

    m_policyReceived(policy);
}

std::vector<A1NtnPolicy>
OranNtnA1Adapter::GetActivePolicies() const
{
    std::vector<A1NtnPolicy> result;
    result.reserve(m_activePolicies.size());
    for (const auto& [id, p] : m_activePolicies)
    {
        result.push_back(p);
    }
    return result;
}

std::vector<A1NtnPolicy>
OranNtnA1Adapter::GetPoliciesByType(A1PolicyType type) const
{
    std::vector<A1NtnPolicy> result;
    for (const auto& [id, p] : m_activePolicies)
    {
        if (p.type == type)
        {
            result.push_back(p);
        }
    }
    return result;
}

double
OranNtnA1Adapter::GetThreshold(A1PolicyType type, const std::string& paramName) const
{
    double threshold = 0.0;
    bool found = false;

    for (const auto& [id, p] : m_activePolicies)
    {
        if (p.type != type)
        {
            continue;
        }

        double val = 0.0;
        if (paramName == "tte_min")
        {
            val = p.param1;
        }
        else if (paramName == "sinr_threshold")
        {
            val = p.param2;
        }
        else if (paramName == "d1_threshold")
        {
            val = p.param3;
        }

        if (!found || val > threshold)
        {
            threshold = val;
            found = true;
        }
    }
    return threshold;
}

void
OranNtnA1Adapter::ReportEnforcement(uint32_t policyId, bool enforced,
                                      std::string status)
{
    NS_LOG_FUNCTION(this << policyId << enforced);
    m_enforcementReported(policyId, enforced);

    if (!m_feedbackCb.IsNull())
    {
        m_feedbackCb(policyId, enforced, status);
    }
}

void
OranNtnA1Adapter::SetFeedbackCallback(FeedbackCallback cb)
{
    m_feedbackCb = cb;
}

} // namespace ns3
