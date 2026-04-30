/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-only
 * Author: Muhammad Uzair
 *
 * Interference Management xApp - Implementation
 *
 * Coordinated inter-beam and TN-NTN interference mitigation using
 * interference graph construction, greedy graph coloring, adaptive
 * power control, and null steering via O-RAN RC actions.
 */

#include "oran-ntn-xapp-interference-mgmt.h"

#include "oran-ntn-sat-bridge.h"

#include <ns3/boolean.h>
#include <ns3/double.h>
#include <ns3/log.h>
#include <ns3/simulator.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <set>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("OranNtnXappInterferenceMgmt");
NS_OBJECT_ENSURE_REGISTERED(OranNtnXappInterferenceMgmt);

TypeId
OranNtnXappInterferenceMgmt::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::OranNtnXappInterferenceMgmt")
            .SetParent<OranNtnXappBase>()
            .SetGroupName("OranNtn")
            .AddConstructor<OranNtnXappInterferenceMgmt>()
            .AddAttribute("InterferenceThreshold",
                          "Interference threshold in dBm for edge creation",
                          DoubleValue(-90.0),
                          MakeDoubleAccessor(
                              &OranNtnXappInterferenceMgmt::m_interferenceThreshold_dBm),
                          MakeDoubleChecker<double>())
            .AddAttribute("PowerControlEnabled",
                          "Enable adaptive power control mitigation",
                          BooleanValue(true),
                          MakeBooleanAccessor(
                              &OranNtnXappInterferenceMgmt::m_powerControlEnabled),
                          MakeBooleanChecker())
            .AddAttribute("NullSteeringEnabled",
                          "Enable null steering mitigation",
                          BooleanValue(true),
                          MakeBooleanAccessor(
                              &OranNtnXappInterferenceMgmt::m_nullSteeringEnabled),
                          MakeBooleanChecker())
            .AddAttribute("GraphColoringEnabled",
                          "Enable graph-coloring frequency assignment",
                          BooleanValue(true),
                          MakeBooleanAccessor(
                              &OranNtnXappInterferenceMgmt::m_graphColoringEnabled),
                          MakeBooleanChecker())
            .AddAttribute("PredictiveEnabled",
                          "Enable predictive interference avoidance",
                          BooleanValue(false),
                          MakeBooleanAccessor(
                              &OranNtnXappInterferenceMgmt::m_predictiveEnabled),
                          MakeBooleanChecker());
    return tid;
}

OranNtnXappInterferenceMgmt::OranNtnXappInterferenceMgmt()
    : m_interferenceThreshold_dBm(-90.0),
      m_powerControlEnabled(true),
      m_nullSteeringEnabled(true),
      m_graphColoringEnabled(true),
      m_predictiveEnabled(false)
{
    NS_LOG_FUNCTION(this);
    SetXappName("InterferenceMgmt");

    m_ifMetrics = {};
}

OranNtnXappInterferenceMgmt::~OranNtnXappInterferenceMgmt()
{
    NS_LOG_FUNCTION(this);
}

// --------------------------------------------------------------------------
//  Configuration setters
// --------------------------------------------------------------------------

void
OranNtnXappInterferenceMgmt::SetSatBridge(Ptr<OranNtnSatBridge> bridge)
{
    NS_LOG_FUNCTION(this << bridge);
    m_satBridge = bridge;
}

void
OranNtnXappInterferenceMgmt::SetInterferenceThreshold(double threshold_dBm)
{
    NS_LOG_FUNCTION(this << threshold_dBm);
    m_interferenceThreshold_dBm = threshold_dBm;
}

void
OranNtnXappInterferenceMgmt::SetPowerControlEnabled(bool enable)
{
    m_powerControlEnabled = enable;
}

void
OranNtnXappInterferenceMgmt::SetNullSteeringEnabled(bool enable)
{
    m_nullSteeringEnabled = enable;
}

void
OranNtnXappInterferenceMgmt::SetGraphColoringEnabled(bool enable)
{
    m_graphColoringEnabled = enable;
}

void
OranNtnXappInterferenceMgmt::SetPredictiveEnabled(bool enable)
{
    m_predictiveEnabled = enable;
}

// --------------------------------------------------------------------------
//  Public queries
// --------------------------------------------------------------------------

std::vector<OranNtnXappInterferenceMgmt::InterferenceEdge>
OranNtnXappInterferenceMgmt::GetInterferenceGraph() const
{
    return m_interferenceGraph;
}

std::vector<InterferenceMapEntry>
OranNtnXappInterferenceMgmt::GetInterferenceMap() const
{
    std::vector<InterferenceMapEntry> result;
    result.reserve(m_interferenceMap.size());
    for (const auto& kv : m_interferenceMap)
    {
        result.push_back(kv.second);
    }
    return result;
}

OranNtnXappInterferenceMgmt::InterferenceMgmtMetrics
OranNtnXappInterferenceMgmt::GetInterferenceMgmtMetrics() const
{
    return m_ifMetrics;
}

// --------------------------------------------------------------------------
//  E2 subscription
// --------------------------------------------------------------------------

E2Subscription
OranNtnXappInterferenceMgmt::GetRequiredSubscription() const
{
    E2Subscription sub;
    sub.subscriptionId = 0; // Assigned by E2 termination
    sub.ricRequestorId = GetXappId();
    sub.ranFunctionId = 2;  // KPM service model
    sub.reportingPeriod = MilliSeconds(100);
    sub.eventTrigger = false; // Periodic reporting
    sub.eventThreshold = 0.0;
    sub.batchOnVisibility = false;
    sub.maxBufferAge = Seconds(5);
    sub.useIslRelay = true;
    return sub;
}

// --------------------------------------------------------------------------
//  ProcessKpmReport  -- store interference per (sat, beam) pair
// --------------------------------------------------------------------------

void
OranNtnXappInterferenceMgmt::ProcessKpmReport(const E2KpmReport& report)
{
    NS_LOG_FUNCTION(this << report.gnbId << report.beamId);

    // Pack (satId, beamId) into a 64-bit key for fast lookup
    uint64_t key =
        (static_cast<uint64_t>(report.gnbId) << 32) | static_cast<uint64_t>(report.beamId);

    InterferenceMapEntry entry;
    entry.victimSatId = report.gnbId;
    entry.victimBeamId = report.beamId;
    entry.aggressorSatId = 0;        // filled during graph build
    entry.aggressorBeamId = 0;
    entry.interference_dBm = report.interBeamInterference_dBm;
    entry.couplingLoss_dB = 0.0;
    entry.angularSeparation_deg = 0.0;

    m_interferenceMap[key] = entry;

    NS_LOG_DEBUG("InterferenceMgmt: sat=" << report.gnbId << " beam=" << report.beamId
                                          << " interference="
                                          << report.interBeamInterference_dBm << " dBm");
}

// --------------------------------------------------------------------------
//  DecisionCycle  -- main control loop
// --------------------------------------------------------------------------

void
OranNtnXappInterferenceMgmt::DecisionCycle()
{
    NS_LOG_FUNCTION(this);

    if (!m_satBridge)
    {
        NS_LOG_WARN("InterferenceMgmt: no SatBridge configured, skipping cycle");
        return;
    }

    // Step 1: Build the interference graph from current satellite geometry
    BuildInterferenceGraph();

    // Step 2: Detect threshold violations
    bool violationDetected = false;
    for (const auto& edge : m_interferenceGraph)
    {
        if (edge.interference_dBm > m_interferenceThreshold_dBm)
        {
            violationDetected = true;
            m_ifMetrics.interferenceEventsDetected++;
            NS_LOG_INFO("InterferenceMgmt: threshold violation sat["
                        << edge.sat1 << "]beam[" << edge.beam1 << "] <-> sat["
                        << edge.sat2 << "]beam[" << edge.beam2 << "] = "
                        << edge.interference_dBm << " dBm");
        }
    }

    if (!violationDetected)
    {
        NS_LOG_DEBUG("InterferenceMgmt: no threshold violations this cycle");
        RecordDecision(true, 1.0, 0.0);
        return;
    }

    // Step 3: Apply mitigation strategies in order of aggressiveness
    if (m_powerControlEnabled)
    {
        ApplyPowerControl();
    }

    if (m_nullSteeringEnabled)
    {
        ApplyNullSteering();
    }

    if (m_graphColoringEnabled)
    {
        ApplyGraphColoring();
    }

    if (m_predictiveEnabled)
    {
        PredictFutureInterference();
    }

    // Update aggregate metrics
    if (!m_interferenceGraph.empty())
    {
        double sumIf = 0.0;
        double maxIf = -std::numeric_limits<double>::infinity();
        for (const auto& edge : m_interferenceGraph)
        {
            sumIf += edge.interference_dBm;
            if (edge.interference_dBm > maxIf)
            {
                maxIf = edge.interference_dBm;
            }
        }
        m_ifMetrics.avgInterference_dBm = sumIf / m_interferenceGraph.size();
        m_ifMetrics.maxInterference_dBm = maxIf;
    }

    RecordDecision(true, 0.8, 0.0);
}

// --------------------------------------------------------------------------
//  BuildInterferenceGraph
// --------------------------------------------------------------------------

void
OranNtnXappInterferenceMgmt::BuildInterferenceGraph()
{
    NS_LOG_FUNCTION(this);
    m_interferenceGraph.clear();

    uint32_t numSats = m_satBridge->GetNumSatellites();
    if (numSats < 2)
    {
        return;
    }

    // For each pair of satellites, compute inter-beam coupling loss and create
    // an interference edge if the coupling exceeds the threshold
    for (uint32_t s1 = 0; s1 < numSats; ++s1)
    {
        const auto& state1 = m_satBridge->GetSatState(s1);

        for (uint32_t s2 = s1 + 1; s2 < numSats; ++s2)
        {
            const auto& state2 = m_satBridge->GetSatState(s2);

            // Check all beam pairs between these two satellites
            for (const auto& b1 : state1.beamLoads)
            {
                for (const auto& b2 : state2.beamLoads)
                {
                    double couplingLoss_dB = ComputeCouplingLoss(s1, b1.first, s2, b2.first);

                    // Interference = tx power - coupling loss (simplified model)
                    // Use a reference TX power of 30 dBm per beam
                    double txPower_dBm = 30.0;
                    double interference_dBm = txPower_dBm - couplingLoss_dB;

                    // Also check if we have measured interference from KPM reports
                    uint64_t key1 =
                        (static_cast<uint64_t>(s1) << 32) | static_cast<uint64_t>(b1.first);
                    auto it = m_interferenceMap.find(key1);
                    if (it != m_interferenceMap.end())
                    {
                        // Use the worse of computed and reported interference
                        interference_dBm =
                            std::max(interference_dBm, it->second.interference_dBm);
                    }

                    if (interference_dBm > m_interferenceThreshold_dBm)
                    {
                        InterferenceEdge edge;
                        edge.sat1 = s1;
                        edge.beam1 = b1.first;
                        edge.sat2 = s2;
                        edge.beam2 = b2.first;
                        edge.interference_dBm = interference_dBm;
                        edge.couplingLoss_dB = couplingLoss_dB;
                        m_interferenceGraph.push_back(edge);

                        NS_LOG_DEBUG("InterferenceEdge: sat"
                                     << s1 << ".beam" << b1.first << " <-> sat" << s2
                                     << ".beam" << b2.first << " coupling="
                                     << couplingLoss_dB << "dB interference="
                                     << interference_dBm << "dBm");
                    }
                }
            }
        }
    }

    NS_LOG_INFO("InterferenceMgmt: built graph with " << m_interferenceGraph.size()
                                                       << " edges from " << numSats
                                                       << " satellites");
}

// --------------------------------------------------------------------------
//  ComputeCouplingLoss -- antenna pattern overlap model
// --------------------------------------------------------------------------

double
OranNtnXappInterferenceMgmt::ComputeCouplingLoss(uint32_t sat1,
                                                   uint32_t beam1,
                                                   uint32_t sat2,
                                                   uint32_t beam2) const
{
    NS_LOG_FUNCTION(this << sat1 << beam1 << sat2 << beam2);

    // Get satellite positions to compute angular separation between beam centers
    GeoCoordinate pos1 = m_satBridge->GetSatellitePosition(sat1);
    GeoCoordinate pos2 = m_satBridge->GetSatellitePosition(sat2);

    // Compute angular separation between the two satellite sub-beam points
    // as seen from the ground. Use great-circle angular distance.
    double lat1Rad = pos1.GetLatitude() * M_PI / 180.0;
    double lon1Rad = pos1.GetLongitude() * M_PI / 180.0;
    double lat2Rad = pos2.GetLatitude() * M_PI / 180.0;
    double lon2Rad = pos2.GetLongitude() * M_PI / 180.0;

    // Haversine formula for angular separation
    double dLat = lat2Rad - lat1Rad;
    double dLon = lon2Rad - lon1Rad;
    double a = std::sin(dLat / 2.0) * std::sin(dLat / 2.0) +
               std::cos(lat1Rad) * std::cos(lat2Rad) * std::sin(dLon / 2.0) *
                   std::sin(dLon / 2.0);
    double angularSep_rad = 2.0 * std::atan2(std::sqrt(a), std::sqrt(1.0 - a));
    double angularSep_deg = angularSep_rad * 180.0 / M_PI;

    // Account for beam offset within a satellite (beam index offset)
    // Each beam has a 3dB beamwidth offset from satellite boresight
    double beamOffset_deg = std::abs(static_cast<double>(beam1) - static_cast<double>(beam2)) * 0.5;
    double effectiveAngle_deg = std::max(0.0, angularSep_deg - beamOffset_deg);

    // 3GPP-style antenna sidelobe pattern: coupling = 12 * (theta / theta_3dB)^2 dB
    // theta_3dB is the 3 dB beamwidth, typically ~3.5 degrees for LEO Ka-band spot beams
    double theta3dB_deg = 3.5;
    double normalizedAngle = effectiveAngle_deg / theta3dB_deg;
    double patternLoss_dB = 12.0 * normalizedAngle * normalizedAngle;

    // Cap at maximum sidelobe attenuation (front-to-back ratio ~30 dB)
    patternLoss_dB = std::min(patternLoss_dB, 30.0);

    // Add free-space path loss component between satellite positions
    // Distance between satellites in km
    double earthRadius_km = 6371.0;
    double dist_km = angularSep_rad * earthRadius_km;
    double fspl_dB = 0.0;
    if (dist_km > 0.0)
    {
        // Simplified FSPL at Ka-band (20 GHz)
        double freqGHz = 20.0;
        fspl_dB = 20.0 * std::log10(dist_km) + 20.0 * std::log10(freqGHz) + 92.45;
    }

    // Total coupling loss is the sum of pattern isolation and path loss
    double totalCoupling_dB = patternLoss_dB + fspl_dB;

    NS_LOG_DEBUG("CouplingLoss: sat" << sat1 << ".beam" << beam1 << " <-> sat" << sat2
                                     << ".beam" << beam2 << " angSep="
                                     << angularSep_deg << "deg pattern="
                                     << patternLoss_dB << "dB fspl=" << fspl_dB
                                     << "dB total=" << totalCoupling_dB << "dB");

    return totalCoupling_dB;
}

// --------------------------------------------------------------------------
//  ApplyPowerControl -- reduce aggressor beam TX power
// --------------------------------------------------------------------------

void
OranNtnXappInterferenceMgmt::ApplyPowerControl()
{
    NS_LOG_FUNCTION(this);

    for (const auto& edge : m_interferenceGraph)
    {
        if (edge.interference_dBm <= m_interferenceThreshold_dBm)
        {
            continue; // no violation
        }

        // Compute required power reduction: bring interference down to threshold
        double reduction_dB = edge.interference_dBm - m_interferenceThreshold_dBm;

        // Cap reduction to avoid completely shutting off beams
        double maxReduction_dB = 10.0;
        reduction_dB = std::min(reduction_dB, maxReduction_dB);

        // Issue TX_POWER_CONTROL action for the aggressor beam (sat2.beam2)
        // Convention: the higher-indexed satellite is treated as aggressor
        E2RcAction action = BuildAction(E2RcActionType::TX_POWER_CONTROL,
                                        edge.sat2,
                                        0, // cell-wide action
                                        0.85);
        action.targetBeamId = edge.beam2;
        action.parameter1 = -reduction_dB; // negative = power reduction
        action.parameter2 = edge.interference_dBm;

        bool accepted = SubmitAction(action);
        if (accepted)
        {
            m_ifMetrics.powerControlActions++;
            m_ifMetrics.interferenceEventsMitigated++;
            m_ifMetrics.interferenceReduction_dB += reduction_dB;

            NS_LOG_INFO("InterferenceMgmt: power control on sat"
                        << edge.sat2 << ".beam" << edge.beam2 << " reduced by "
                        << reduction_dB << " dB");
        }
    }
}

// --------------------------------------------------------------------------
//  ApplyNullSteering -- beam nulling toward interference direction
// --------------------------------------------------------------------------

void
OranNtnXappInterferenceMgmt::ApplyNullSteering()
{
    NS_LOG_FUNCTION(this);

    // Track which beams already had null steering applied this cycle
    std::set<uint64_t> steeredBeams;

    for (const auto& edge : m_interferenceGraph)
    {
        if (edge.interference_dBm <= m_interferenceThreshold_dBm)
        {
            continue;
        }

        // Apply null steering on the victim beam toward the aggressor direction
        uint64_t victimKey =
            (static_cast<uint64_t>(edge.sat1) << 32) | static_cast<uint64_t>(edge.beam1);

        if (steeredBeams.count(victimKey) > 0)
        {
            continue; // already steered this beam
        }

        // Compute the direction toward the aggressor satellite for null placement
        GeoCoordinate victimPos = m_satBridge->GetSatellitePosition(edge.sat1);
        GeoCoordinate aggressorPos = m_satBridge->GetSatellitePosition(edge.sat2);

        double dLat = aggressorPos.GetLatitude() - victimPos.GetLatitude();
        double dLon = aggressorPos.GetLongitude() - victimPos.GetLongitude();
        double nullDirection_deg = std::atan2(dLon, dLat) * 180.0 / M_PI;

        // Issue BEAM_SWITCH action with null steering direction encoded
        // Using INTERFERENCE_NULLING action type for explicit nulling
        E2RcAction action = BuildAction(E2RcActionType::INTERFERENCE_NULLING,
                                        edge.sat1,
                                        0,
                                        0.75);
        action.targetBeamId = edge.beam1;
        action.parameter1 = nullDirection_deg; // null steering azimuth
        action.parameter2 = edge.interference_dBm;

        bool accepted = SubmitAction(action);
        if (accepted)
        {
            m_ifMetrics.nullSteeringActions++;
            m_ifMetrics.interferenceEventsMitigated++;
            steeredBeams.insert(victimKey);

            NS_LOG_INFO("InterferenceMgmt: null steering on sat"
                        << edge.sat1 << ".beam" << edge.beam1 << " toward "
                        << nullDirection_deg << " deg");
        }
    }
}

// --------------------------------------------------------------------------
//  GraphColorBeams -- greedy graph coloring for frequency assignment
// --------------------------------------------------------------------------

std::map<uint32_t, uint32_t>
OranNtnXappInterferenceMgmt::GraphColorBeams(
    const std::vector<InterferenceEdge>& graph,
    uint32_t numColors) const
{
    NS_LOG_FUNCTION(this << graph.size() << numColors);

    // Build adjacency list and compute node degrees
    // Node ID = packed (satId << 16 | beamId) for uniqueness
    std::map<uint32_t, std::set<uint32_t>> adjacency;
    std::map<uint32_t, uint32_t> degree;

    for (const auto& edge : graph)
    {
        uint32_t node1 = (edge.sat1 << 16) | edge.beam1;
        uint32_t node2 = (edge.sat2 << 16) | edge.beam2;

        adjacency[node1].insert(node2);
        adjacency[node2].insert(node1);
        degree[node1] = static_cast<uint32_t>(adjacency[node1].size());
        degree[node2] = static_cast<uint32_t>(adjacency[node2].size());
    }

    // Sort nodes by degree (descending) -- greedy heuristic
    std::vector<uint32_t> nodes;
    nodes.reserve(degree.size());
    for (const auto& kv : degree)
    {
        nodes.push_back(kv.first);
    }
    std::sort(nodes.begin(), nodes.end(), [&degree](uint32_t a, uint32_t b) {
        return degree[a] > degree[b];
    });

    // Greedy coloring: assign minimum available color to each node
    std::map<uint32_t, uint32_t> coloring;

    for (uint32_t node : nodes)
    {
        // Find colors used by neighbors
        std::set<uint32_t> usedColors;
        for (uint32_t neighbor : adjacency[node])
        {
            auto cIt = coloring.find(neighbor);
            if (cIt != coloring.end())
            {
                usedColors.insert(cIt->second);
            }
        }

        // Assign minimum available color
        uint32_t assignedColor = 0;
        while (usedColors.count(assignedColor) > 0)
        {
            assignedColor++;
            if (assignedColor >= numColors)
            {
                // Wrap around if we exceed available colors -- best effort
                assignedColor = 0;
                break;
            }
        }
        coloring[node] = assignedColor;
    }

    NS_LOG_INFO("GraphColor: colored " << coloring.size() << " nodes with up to "
                                        << numColors << " colors");
    return coloring;
}

// --------------------------------------------------------------------------
//  ApplyGraphColoring -- frequency reassignment based on coloring
// --------------------------------------------------------------------------

void
OranNtnXappInterferenceMgmt::ApplyGraphColoring()
{
    NS_LOG_FUNCTION(this);

    if (m_interferenceGraph.empty())
    {
        return;
    }

    // Use 4 frequency colors (sub-bands) for LEO Ka-band
    uint32_t numColors = 4;
    auto coloring = GraphColorBeams(m_interferenceGraph, numColors);

    // Issue BEAM_SWITCH actions to reassign frequency sub-bands
    for (const auto& kv : coloring)
    {
        uint32_t satId = (kv.first >> 16) & 0xFFFF;
        uint32_t beamId = kv.first & 0xFFFF;
        uint32_t colorIdx = kv.second;

        E2RcAction action = BuildAction(E2RcActionType::BEAM_SWITCH,
                                        satId,
                                        0,
                                        0.9);
        action.targetBeamId = beamId;
        action.parameter1 = static_cast<double>(colorIdx); // frequency sub-band index
        action.parameter2 = 0.0;

        bool accepted = SubmitAction(action);
        if (accepted)
        {
            m_ifMetrics.frequencyReassignments++;
            NS_LOG_DEBUG("InterferenceMgmt: sat" << satId << ".beam" << beamId
                                                  << " -> color " << colorIdx);
        }
    }

    NS_LOG_INFO("InterferenceMgmt: graph coloring reassigned "
                << m_ifMetrics.frequencyReassignments << " beams");
}

// --------------------------------------------------------------------------
//  PredictFutureInterference -- orbit-based overlap forecast
// --------------------------------------------------------------------------

void
OranNtnXappInterferenceMgmt::PredictFutureInterference()
{
    NS_LOG_FUNCTION(this);

    // Look ahead 60 seconds using orbital mechanics to predict upcoming
    // beam overlap situations and pre-apply mitigation
    double lookahead_s = 60.0;

    uint32_t numSats = m_satBridge->GetNumSatellites();
    if (numSats < 2)
    {
        return;
    }

    // For each satellite pair, check if beams will converge in the future
    // by examining the rate of change of angular separation
    for (uint32_t s1 = 0; s1 < numSats; ++s1)
    {
        for (uint32_t s2 = s1 + 1; s2 < numSats; ++s2)
        {
            GeoCoordinate pos1 = m_satBridge->GetSatellitePosition(s1);
            GeoCoordinate pos2 = m_satBridge->GetSatellitePosition(s2);
            Vector vel1 = m_satBridge->GetSatelliteVelocity(s1);
            Vector vel2 = m_satBridge->GetSatelliteVelocity(s2);

            // Current angular separation (simplified as lat/lon distance)
            double currentSep = std::sqrt(
                std::pow(pos2.GetLatitude() - pos1.GetLatitude(), 2) +
                std::pow(pos2.GetLongitude() - pos1.GetLongitude(), 2));

            // Rate of change of separation from velocity difference
            // Project velocity into lat/lon space (rough approximation)
            double dVx = vel2.x - vel1.x;
            double dVy = vel2.y - vel1.y;
            double closingRate = -std::sqrt(dVx * dVx + dVy * dVy) / 111.0; // deg/s approx

            // Predict separation at lookahead time
            double futureSep = currentSep + closingRate * lookahead_s;

            if (futureSep < 5.0 && futureSep < currentSep)
            {
                // Satellites are converging -- preemptive graph coloring warning
                NS_LOG_INFO("InterferenceMgmt: PREDICTIVE - sat"
                            << s1 << " and sat" << s2 << " converging from "
                            << currentSep << " to " << futureSep
                            << " deg in " << lookahead_s << "s");
            }
        }
    }
}

} // namespace ns3
