/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-only
 * Author: Muhammad Uzair
 *
 * Gymnasium Environment for Beam Hopping xApp
 */

#include "oran-ntn-gym-beam-hop.h"

#include "oran-ntn-sat-bridge.h"
#include "oran-ntn-xapp-beam-hop.h"

#include <ns3/container.h>
#include <ns3/double.h>
#include <ns3/log.h>
#include <ns3/simulator.h>
#include <ns3/spaces.h>
#include <ns3/uinteger.h>

#include <algorithm>
#include <cmath>
#include <numeric>
#include <sstream>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("OranNtnGymBeamHop");
NS_OBJECT_ENSURE_REGISTERED(OranNtnGymBeamHop);

TypeId
OranNtnGymBeamHop::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::OranNtnGymBeamHop")
            .SetParent<OpenGymEnv>()
            .SetGroupName("OranNtn")
            .AddConstructor<OranNtnGymBeamHop>()
            .AddAttribute("NumBeams",
                          "Number of beams in the satellite footprint",
                          UintegerValue(7),
                          MakeUintegerAccessor(&OranNtnGymBeamHop::m_numBeams),
                          MakeUintegerChecker<uint32_t>(1, 256))
            .AddAttribute("MaxSteps",
                          "Maximum number of steps per episode",
                          UintegerValue(1000),
                          MakeUintegerAccessor(&OranNtnGymBeamHop::m_maxSteps),
                          MakeUintegerChecker<uint32_t>())
            .AddAttribute("FairnessWeight",
                          "Reward weight for Jain fairness index",
                          DoubleValue(1.0),
                          MakeDoubleAccessor(&OranNtnGymBeamHop::m_fairnessWeight),
                          MakeDoubleChecker<double>())
            .AddAttribute("ThroughputWeight",
                          "Reward weight for normalized throughput",
                          DoubleValue(1.0),
                          MakeDoubleAccessor(&OranNtnGymBeamHop::m_throughputWeight),
                          MakeDoubleChecker<double>())
            .AddAttribute("EnergyWeight",
                          "Reward weight for energy efficiency",
                          DoubleValue(0.5),
                          MakeDoubleAccessor(&OranNtnGymBeamHop::m_energyWeight),
                          MakeDoubleChecker<double>());
    return tid;
}

OranNtnGymBeamHop::OranNtnGymBeamHop()
    : m_xapp(nullptr),
      m_satBridge(nullptr),
      m_numBeams(7),
      m_currentSatId(0),
      m_postFairness(0.0),
      m_postThroughput(0.0),
      m_postEnergyEff(0.0),
      m_stepCount(0),
      m_maxSteps(1000),
      m_fairnessWeight(1.0),
      m_throughputWeight(1.0),
      m_energyWeight(0.5)
{
    NS_LOG_FUNCTION(this);
}

OranNtnGymBeamHop::~OranNtnGymBeamHop()
{
    NS_LOG_FUNCTION(this);
}

void
OranNtnGymBeamHop::DoDispose()
{
    NS_LOG_FUNCTION(this);
    m_xapp = nullptr;
    m_satBridge = nullptr;
    OpenGymEnv::DoDispose();
}

void
OranNtnGymBeamHop::SetXapp(Ptr<OranNtnXappBeamHop> xapp)
{
    NS_LOG_FUNCTION(this << xapp);
    m_xapp = xapp;
}

void
OranNtnGymBeamHop::SetSatBridge(Ptr<OranNtnSatBridge> bridge)
{
    NS_LOG_FUNCTION(this << bridge);
    m_satBridge = bridge;
}

void
OranNtnGymBeamHop::SetNumBeams(uint32_t n)
{
    NS_LOG_FUNCTION(this << n);
    m_numBeams = n;
}

void
OranNtnGymBeamHop::SetCurrentSatellite(uint32_t satId)
{
    NS_LOG_FUNCTION(this << satId);
    m_currentSatId = satId;
}

void
OranNtnGymBeamHop::UpdatePostAction(double fairness, double throughput, double energyEff)
{
    NS_LOG_FUNCTION(this << fairness << throughput << energyEff);
    m_postFairness = fairness;
    m_postThroughput = throughput;
    m_postEnergyEff = energyEff;
}

// ---------------------------------------------------------------------------
// OpenGymEnv interface
// ---------------------------------------------------------------------------

Ptr<OpenGymSpace>
OranNtnGymBeamHop::GetActionSpace()
{
    NS_LOG_FUNCTION(this);
    // Continuous allocation per beam in [0,1]
    std::vector<uint32_t> shape = {m_numBeams};
    return CreateObject<OpenGymBoxSpace>(0.0f, 1.0f, shape, "float");
}

Ptr<OpenGymSpace>
OranNtnGymBeamHop::GetObservationSpace()
{
    NS_LOG_FUNCTION(this);
    // Per-beam: [demand, load, avgSinr, interference] => numBeams * 4
    std::vector<uint32_t> shape = {m_numBeams * 4};
    return CreateObject<OpenGymBoxSpace>(-200.0f, 200.0f, shape, "float");
}

Ptr<OpenGymDataContainer>
OranNtnGymBeamHop::GetObservation()
{
    NS_LOG_FUNCTION(this);

    std::vector<uint32_t> shape = {m_numBeams * 4};
    auto box = CreateObject<OpenGymBoxContainer<float>>(shape);

    for (uint32_t b = 0; b < m_numBeams; b++)
    {
        float demand = 0.0f;
        float load = 0.0f;
        float avgSinr = 0.0f;
        float interference = 0.0f;

        if (m_satBridge)
        {
            // Use sat bridge to get per-beam state
            const auto& satState = m_satBridge->GetSatState(m_currentSatId);
            auto loadIt = satState.beamLoads.find(b);
            if (loadIt != satState.beamLoads.end())
            {
                load = static_cast<float>(loadIt->second);
                demand = load; // Demand approximated from current load
            }
            // Get SINR and interference from bridge link budget
            avgSinr = static_cast<float>(
                m_satBridge->ComputeNtnSinr(0 /*representative UE*/, m_currentSatId, b));
            interference = static_cast<float>(
                m_satBridge->ComputeInterBeamInterference(0, m_currentSatId, b));
        }
        else if (m_xapp)
        {
            // Fallback: use KPM reports from base class
            auto report = m_xapp->GetLatestReport(m_currentSatId);
            load = static_cast<float>(report.prbUtilization);
            demand = load;
            avgSinr = static_cast<float>(report.sinr_dB);
            interference = static_cast<float>(report.interBeamInterference_dBm);
        }

        box->AddValue(demand);
        box->AddValue(load);
        box->AddValue(avgSinr);
        box->AddValue(interference);
    }

    NS_LOG_DEBUG("BeamHop observation: " << m_numBeams << " beams for sat " << m_currentSatId);
    return box;
}

float
OranNtnGymBeamHop::GetReward()
{
    NS_LOG_FUNCTION(this);

    // Normalize throughput to [0,1] range (assume max 1 Gbps per sat)
    double throughputNorm = std::min(m_postThroughput / 1000.0, 1.0);

    float reward = static_cast<float>(
        m_fairnessWeight * m_postFairness +
        m_throughputWeight * throughputNorm +
        m_energyWeight * m_postEnergyEff);

    m_stepCount++;

    NS_LOG_DEBUG("Reward=" << reward << " (fairness=" << m_postFairness
                           << " tput=" << throughputNorm
                           << " energy=" << m_postEnergyEff << ")");
    return reward;
}

bool
OranNtnGymBeamHop::GetGameOver()
{
    NS_LOG_FUNCTION(this);
    return (m_stepCount >= m_maxSteps);
}

std::string
OranNtnGymBeamHop::GetExtraInfo()
{
    NS_LOG_FUNCTION(this);
    std::ostringstream oss;
    oss << "step=" << m_stepCount
        << ",satId=" << m_currentSatId
        << ",numBeams=" << m_numBeams
        << ",fairness=" << m_postFairness
        << ",throughput=" << m_postThroughput;
    return oss.str();
}

bool
OranNtnGymBeamHop::ExecuteActions(Ptr<OpenGymDataContainer> action)
{
    NS_LOG_FUNCTION(this << action);

    auto boxAction = DynamicCast<OpenGymBoxContainer<float>>(action);
    if (!boxAction)
    {
        NS_LOG_WARN("Expected box action container for beam hopping");
        return false;
    }

    // Convert continuous allocation to BeamAllocationEntry vector
    std::vector<BeamAllocationEntry> allocations;
    std::vector<float> rawAlloc = boxAction->GetData();

    // Normalize allocations to sum to 1 (softmax-like clamping)
    float allocSum = 0.0f;
    for (uint32_t b = 0; b < m_numBeams && b < rawAlloc.size(); b++)
    {
        rawAlloc[b] = std::max(0.0f, std::min(1.0f, rawAlloc[b]));
        allocSum += rawAlloc[b];
    }

    if (allocSum < 1e-6f)
    {
        allocSum = 1.0f; // Avoid division by zero; uniform fallback
    }

    for (uint32_t b = 0; b < m_numBeams && b < rawAlloc.size(); b++)
    {
        float normalizedAlloc = rawAlloc[b] / allocSum;

        BeamAllocationEntry entry;
        entry.satId = m_currentSatId;
        entry.beamId = b;
        entry.timeSlot = m_stepCount;
        entry.allocatedCellId = b;
        entry.trafficLoad = static_cast<double>(normalizedAlloc);
        entry.isSignaling = false;
        allocations.push_back(entry);

        NS_LOG_DEBUG("Beam " << b << " allocation=" << normalizedAlloc);
    }

    if (m_xapp)
    {
        // Submit beam allocation via RC actions
        for (const auto& entry : allocations)
        {
            E2RcAction beamAction;
            beamAction.timestamp = Simulator::Now().GetSeconds();
            beamAction.xappId = m_xapp->GetXappId();
            beamAction.xappName = m_xapp->GetXappName();
            beamAction.actionType = E2RcActionType::BEAM_HOP_SCHEDULE;
            beamAction.targetGnbId = m_currentSatId;
            beamAction.targetUeId = 0; // cell-wide
            beamAction.targetBeamId = entry.beamId;
            beamAction.parameter1 = entry.trafficLoad;
            beamAction.parameter2 = static_cast<double>(entry.timeSlot);
            beamAction.confidence = 1.0;
            beamAction.executed = false;

            m_xapp->SubmitAction(beamAction);
        }
    }

    return true;
}

} // namespace ns3
