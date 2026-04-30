/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-only
 * Author: Muhammad Uzair
 */

#include "oran-ntn-space-ric.h"

#include "oran-ntn-near-rt-ric.h"
#include "oran-ntn-sat-bridge.h"
#include "oran-ntn-space-ric-inference.h"

#include <ns3/double.h>
#include <ns3/log.h>
#include <ns3/simulator.h>

#include <algorithm>
#include <cmath>
#include <numeric>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("OranNtnSpaceRic");
NS_OBJECT_ENSURE_REGISTERED(OranNtnSpaceRic);

TypeId
OranNtnSpaceRic::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::OranNtnSpaceRic")
            .SetParent<Object>()
            .SetGroupName("OranNtn")
            .AddConstructor<OranNtnSpaceRic>()
            .AddAttribute("AutonomousLoopInterval",
                          "Control loop interval during autonomous mode",
                          TimeValue(MilliSeconds(200)),
                          MakeTimeAccessor(&OranNtnSpaceRic::m_autonomousLoopInterval),
                          MakeTimeChecker())
            .AddAttribute("ComputeBudget",
                          "OBP compute budget in MFLOPS",
                          DoubleValue(500.0),
                          MakeDoubleAccessor(&OranNtnSpaceRic::m_computeBudgetMflops),
                          MakeDoubleChecker<double>(0.0))
            .AddAttribute("MemoryBudget",
                          "OBP memory budget in MB",
                          DoubleValue(256.0),
                          MakeDoubleAccessor(&OranNtnSpaceRic::m_memoryBudgetMB),
                          MakeDoubleChecker<double>(0.0))
            .AddTraceSource("AutonomousDecision",
                            "An autonomous decision was made on-orbit",
                            MakeTraceSourceAccessor(&OranNtnSpaceRic::m_autonomousDecision),
                            "ns3::OranNtnSpaceRic::DecisionTracedCallback")
            .AddTraceSource("AutonomousModeChanged",
                            "Autonomous mode was entered or exited",
                            MakeTraceSourceAccessor(
                                &OranNtnSpaceRic::m_autonomousModeChanged),
                            "ns3::OranNtnSpaceRic::ModeTracedCallback")
            .AddTraceSource("IslExchange",
                            "State exchanged with neighbor via ISL",
                            MakeTraceSourceAccessor(&OranNtnSpaceRic::m_islExchange),
                            "ns3::OranNtnSpaceRic::IslTracedCallback")
            .AddTraceSource("ModelSynced",
                            "A model was synchronized",
                            MakeTraceSourceAccessor(&OranNtnSpaceRic::m_modelSynced),
                            "ns3::OranNtnSpaceRic::SyncTracedCallback");
    return tid;
}

OranNtnSpaceRic::OranNtnSpaceRic()
    : m_satId(0),
      m_planeId(0),
      m_groundRic(nullptr),
      m_autonomous(false),
      m_computeBudgetMflops(500.0),
      m_memoryBudgetMB(256.0),
      m_autonomousLoopInterval(MilliSeconds(200)),
      m_satBridge(nullptr),
      m_inferenceEngine(nullptr),
      m_islSequenceNumber(0),
      m_batteryLevel(1.0),
      m_solarPower_W(0.0)
{
    NS_LOG_FUNCTION(this);
    m_metrics = {};
}

OranNtnSpaceRic::~OranNtnSpaceRic()
{
    NS_LOG_FUNCTION(this);
}

void
OranNtnSpaceRic::DoDispose()
{
    NS_LOG_FUNCTION(this);
    Simulator::Cancel(m_autonomousLoopEvent);
    m_groundRic = nullptr;
    m_models.clear();
    m_localKpm.clear();
    m_beamLoads.clear();
    m_decisionBuffer.clear();
    m_syncLog.clear();
    m_islNeighbors.clear();
    Object::DoDispose();
}

// ---- Configuration ----

void
OranNtnSpaceRic::SetSatelliteId(uint32_t satId)
{
    m_satId = satId;
}

void
OranNtnSpaceRic::SetOrbitalPlaneId(uint32_t planeId)
{
    m_planeId = planeId;
}

void
OranNtnSpaceRic::SetGroundRic(Ptr<OranNtnNearRtRic> ric)
{
    m_groundRic = ric;
}

void
OranNtnSpaceRic::SetComputeBudget(double mflops)
{
    m_computeBudgetMflops = mflops;
}

void
OranNtnSpaceRic::SetMemoryBudget(double megabytes)
{
    m_memoryBudgetMB = megabytes;
}

uint32_t
OranNtnSpaceRic::GetSatelliteId() const
{
    return m_satId;
}

uint32_t
OranNtnSpaceRic::GetOrbitalPlaneId() const
{
    return m_planeId;
}

// ---- Autonomous mode ----

void
OranNtnSpaceRic::EnterAutonomousMode()
{
    NS_LOG_FUNCTION(this);
    if (m_autonomous)
    {
        return;
    }

    m_autonomous = true;
    m_autonomousModeChanged(m_satId, true);

    // Start autonomous control loop
    m_autonomousLoopEvent = Simulator::Schedule(m_autonomousLoopInterval,
                                                 &OranNtnSpaceRic::AutonomousControlLoop,
                                                 this);

    NS_LOG_INFO("Space RIC (sat " << m_satId << "): Entered AUTONOMOUS mode"
                << " - feeder link lost, on-board control active");
}

void
OranNtnSpaceRic::ExitAutonomousMode()
{
    NS_LOG_FUNCTION(this);
    if (!m_autonomous)
    {
        return;
    }

    m_autonomous = false;
    Simulator::Cancel(m_autonomousLoopEvent);
    m_autonomousModeChanged(m_satId, false);

    // Sync buffered decisions back to ground RIC
    SyncWithGroundRic();

    NS_LOG_INFO("Space RIC (sat " << m_satId << "): Exited autonomous mode"
                << " - synced " << m_decisionBuffer.size() << " buffered decisions");
}

bool
OranNtnSpaceRic::IsAutonomous() const
{
    return m_autonomous;
}

// ---- Local decision making ----

void
OranNtnSpaceRic::ProcessLocalKpm(const E2KpmReport& report)
{
    NS_LOG_FUNCTION(this << report.ueId);

    // Update local state
    m_localKpm[report.ueId] = report;

    // Update beam load tracking
    m_beamLoads[report.beamId] = report.prbUtilization;
}

DappDecision
OranNtnSpaceRic::AutonomousHandoverDecision(uint32_t ueId,
                                               const E2KpmReport& serving,
                                               const std::vector<E2KpmReport>& candidates)
{
    NS_LOG_FUNCTION(this << ueId);

    DappDecision decision;
    decision.timestamp = Simulator::Now().GetSeconds();
    decision.dappId = m_satId * 100; // dApp ID derived from satellite
    decision.dappName = "auto-ho-dapp";
    decision.actionType = E2RcActionType::HANDOVER_TRIGGER;
    decision.targetUeId = ueId;
    decision.autonomous = true;

    // Simple TTE-aware selection using on-board lightweight model
    double bestScore = -1e9;
    uint32_t bestBeam = serving.beamId;
    double bestConfidence = 0.0;

    for (const auto& cand : candidates)
    {
        if (cand.sinr_dB < -3.0)
        {
            continue; // Quality filter
        }
        if (cand.tte_s < 10.0)
        {
            continue; // Stability filter
        }

        // Score = weighted combination of TTE and SINR
        // On-board model uses simple linear combination
        // (vs. DQN/LSTM on ground RIC)
        double score = 0.0;

        // Check if on-board model weights exist
        auto modelIt = m_models.find("ho-scorer");
        if (modelIt != m_models.end() && modelIt->second.weights.size() >= 3)
        {
            const auto& w = modelIt->second.weights;
            score = w[0] * cand.tte_s + w[1] * cand.sinr_dB + w[2] * cand.elevation_deg;
        }
        else
        {
            // Fallback: simple TTE-primary scoring
            score = cand.tte_s * 2.0 + cand.sinr_dB * 0.5 + cand.elevation_deg * 0.1;
        }

        if (score > bestScore)
        {
            bestScore = score;
            bestBeam = cand.beamId;
            bestConfidence = std::min(0.95, 0.5 + 0.1 * cand.sinr_dB);
        }
    }

    decision.targetBeamId = bestBeam;
    decision.confidence = bestConfidence;
    decision.rationale = "on-board TTE+SINR scoring, score=" + std::to_string(bestScore);

    // Buffer decision
    m_decisionBuffer.push_back(decision);
    m_metrics.totalAutonomousDecisions++;
    m_metrics.handoversInitiated++;

    m_autonomousDecision(m_satId, decision);

    NS_LOG_INFO("Space RIC (sat " << m_satId << "): Autonomous HO decision for UE "
                << ueId << " -> beam " << bestBeam
                << " (confidence=" << bestConfidence << ")");

    return decision;
}

DappDecision
OranNtnSpaceRic::AutonomousBeamReallocation(
    const std::vector<BeamAllocationEntry>& currentSchedule,
    const std::map<uint32_t, double>& beamLoads)
{
    NS_LOG_FUNCTION(this);

    DappDecision decision;
    decision.timestamp = Simulator::Now().GetSeconds();
    decision.dappId = m_satId * 100 + 1;
    decision.dappName = "auto-beam-dapp";
    decision.actionType = E2RcActionType::BEAM_HOP_SCHEDULE;
    decision.targetUeId = 0; // Cell-wide
    decision.autonomous = true;

    // Find overloaded and underloaded beams
    double avgLoad = 0.0;
    if (!beamLoads.empty())
    {
        for (const auto& [beamId, load] : beamLoads)
        {
            avgLoad += load;
        }
        avgLoad /= beamLoads.size();
    }

    uint32_t overloaded = 0;
    uint32_t underloaded = 0;
    for (const auto& [beamId, load] : beamLoads)
    {
        if (load > avgLoad * 1.5)
        {
            overloaded++;
        }
        if (load < avgLoad * 0.3)
        {
            underloaded++;
        }
    }

    decision.confidence = std::min(0.9, 0.3 + 0.1 * overloaded);
    decision.rationale = "beam-realloc: " + std::to_string(overloaded) +
                          " overloaded, " + std::to_string(underloaded) +
                          " underloaded, avgLoad=" + std::to_string(avgLoad);
    decision.targetBeamId = 0; // All beams

    m_decisionBuffer.push_back(decision);
    m_metrics.totalAutonomousDecisions++;
    m_metrics.beamReallocations++;

    m_autonomousDecision(m_satId, decision);
    return decision;
}

// ---- ISL coordination ----

void
OranNtnSpaceRic::IslExchangeState(Ptr<OranNtnSpaceRic> neighbor)
{
    NS_LOG_FUNCTION(this << neighbor->GetSatelliteId());

    // Exchange beam loads
    for (const auto& [beamId, load] : m_beamLoads)
    {
        // Neighbor stores our beam loads for coordination
        neighbor->m_beamLoads[m_satId * 1000 + beamId] = load;
    }

    // Exchange model gradients for federated learning
    for (const auto& [name, model] : m_models)
    {
        auto neighborModel = neighbor->m_models.find(name);
        if (neighborModel != neighbor->m_models.end())
        {
            // Simple federated averaging: average weights
            auto& myWeights = m_models[name].weights;
            auto& theirWeights = neighborModel->second.weights;
            size_t minSize = std::min(myWeights.size(), theirWeights.size());

            for (size_t i = 0; i < minSize; i++)
            {
                double avg = (myWeights[i] + theirWeights[i]) / 2.0;
                myWeights[i] = avg;
                theirWeights[i] = avg;
            }
        }
    }

    m_metrics.islExchanges++;
    m_islExchange(m_satId, neighbor->GetSatelliteId());

    NS_LOG_INFO("Space RIC (sat " << m_satId << "): ISL state exchange with sat "
                << neighbor->GetSatelliteId());
}

bool
OranNtnSpaceRic::CoordinateInterSatHandover(uint32_t ueId,
                                               Ptr<OranNtnSpaceRic> targetSpaceRic)
{
    NS_LOG_FUNCTION(this << ueId << targetSpaceRic->GetSatelliteId());

    // Check if target satellite has capacity
    uint32_t targetActiveUes = static_cast<uint32_t>(targetSpaceRic->m_localKpm.size());

    // Simple capacity check
    if (targetActiveUes > 50) // Max 50 UEs per satellite
    {
        NS_LOG_INFO("Space RIC (sat " << m_satId << "): Inter-sat HO for UE "
                     << ueId << " to sat " << targetSpaceRic->GetSatelliteId()
                     << " REJECTED - target at capacity");
        return false;
    }

    // Coordinate the handover
    NS_LOG_INFO("Space RIC (sat " << m_satId << "): Inter-sat HO for UE "
                 << ueId << " to sat " << targetSpaceRic->GetSatelliteId()
                 << " ACCEPTED");
    return true;
}

// ---- Model management ----

void
OranNtnSpaceRic::ReceiveModelUpdate(const std::string& modelName,
                                      uint32_t version,
                                      const std::vector<double>& weights)
{
    NS_LOG_FUNCTION(this << modelName << version);

    OnBoardModel model;
    model.name = modelName;
    model.version = version;
    model.weights = weights;
    model.sizeKB = static_cast<double>(weights.size() * 8) / 1024.0; // 8 bytes per double

    // Check memory budget
    double totalModelSizeKB = model.sizeKB;
    for (const auto& [name, m] : m_models)
    {
        if (name != modelName)
        {
            totalModelSizeKB += m.sizeKB;
        }
    }

    if (totalModelSizeKB > m_memoryBudgetMB * 1024.0)
    {
        NS_LOG_WARN("Space RIC (sat " << m_satId << "): Model " << modelName
                     << " exceeds memory budget, skipping");
        return;
    }

    m_models[modelName] = model;

    ModelSyncRecord record;
    record.timestamp = Simulator::Now().GetSeconds();
    record.satId = m_satId;
    record.modelName = modelName;
    record.modelVersion = version;
    record.modelSizeKB = model.sizeKB;
    record.syncLatency = Seconds(0); // Will be set by caller
    record.viaIsl = false;
    m_syncLog.push_back(record);

    m_metrics.modelSyncs++;
    m_modelSynced(record);

    NS_LOG_INFO("Space RIC (sat " << m_satId << "): Received model '" << modelName
                << "' v" << version << " (" << model.sizeKB << " KB, "
                << weights.size() << " weights)");
}

std::vector<double>
OranNtnSpaceRic::GetLocalGradients(const std::string& modelName) const
{
    auto it = m_models.find(modelName);
    if (it != m_models.end())
    {
        // Return weights as proxy for gradients (simplified)
        return it->second.weights;
    }
    return {};
}

uint32_t
OranNtnSpaceRic::GetModelVersion(const std::string& modelName) const
{
    auto it = m_models.find(modelName);
    if (it != m_models.end())
    {
        return it->second.version;
    }
    return 0;
}

// ---- Metrics ----

OranNtnSpaceRic::SpaceRicMetrics
OranNtnSpaceRic::GetMetrics() const
{
    return m_metrics;
}

std::vector<DappDecision>
OranNtnSpaceRic::GetDecisionLog() const
{
    return std::vector<DappDecision>(m_decisionBuffer.begin(), m_decisionBuffer.end());
}

std::vector<ModelSyncRecord>
OranNtnSpaceRic::GetSyncLog() const
{
    return std::vector<ModelSyncRecord>(m_syncLog.begin(), m_syncLog.end());
}

// ---- Private ----

void
OranNtnSpaceRic::AutonomousControlLoop()
{
    NS_LOG_FUNCTION(this);

    if (!m_autonomous)
    {
        return;
    }

    // Check all tracked UEs for handover needs
    for (const auto& [ueId, report] : m_localKpm)
    {
        // Trigger autonomous HO if SINR < -5 dB or TTE < 5s
        if (report.sinr_dB < -5.0 || report.tte_s < 5.0)
        {
            // Collect candidates from local beam measurements
            std::vector<E2KpmReport> candidates;
            for (const auto& [otherUeId, otherReport] : m_localKpm)
            {
                if (otherReport.beamId != report.beamId)
                {
                    candidates.push_back(otherReport);
                }
            }

            if (!candidates.empty())
            {
                AutonomousHandoverDecision(ueId, report, candidates);
            }
        }
    }

    // Check beam loads for reallocation
    bool hasOverloaded = false;
    for (const auto& [beamId, load] : m_beamLoads)
    {
        if (load > 0.85)
        {
            hasOverloaded = true;
            break;
        }
    }
    if (hasOverloaded)
    {
        AutonomousBeamReallocation({}, m_beamLoads);
    }

    // Track autonomous time
    m_metrics.totalAutonomousTime += m_autonomousLoopInterval;

    // Re-schedule
    m_autonomousLoopEvent = Simulator::Schedule(m_autonomousLoopInterval,
                                                 &OranNtnSpaceRic::AutonomousControlLoop,
                                                 this);
}

void
OranNtnSpaceRic::SyncWithGroundRic()
{
    NS_LOG_FUNCTION(this);

    if (!m_groundRic)
    {
        return;
    }

    NS_LOG_INFO("Space RIC (sat " << m_satId << "): Syncing "
                 << m_decisionBuffer.size() << " autonomous decisions to ground RIC");

    // In a real implementation, this would send decisions via feeder link
    // For simulation, we log the sync event
    m_metrics.totalGroundAssistedDecisions += m_decisionBuffer.size();
}

// ============================================================================
//  Phase 1/3/5: Deep Integration Methods
// ============================================================================

void
OranNtnSpaceRic::SetSatBridge(Ptr<OranNtnSatBridge> bridge)
{
    m_satBridge = bridge;
}

void
OranNtnSpaceRic::SetInferenceEngine(Ptr<OranNtnSpaceRicInference> engine)
{
    m_inferenceEngine = engine;
}

void
OranNtnSpaceRic::AddIslNeighbor(Ptr<OranNtnSpaceRic> neighbor)
{
    NS_LOG_FUNCTION(this << neighbor->GetSatelliteId());

    // Check if already added
    for (const auto& existing : m_islNeighbors)
    {
        if (existing->GetSatelliteId() == neighbor->GetSatelliteId())
        {
            return;
        }
    }
    m_islNeighbors.push_back(neighbor);
}

void
OranNtnSpaceRic::ReceiveIslPacket(Ptr<Packet> packet)
{
    NS_LOG_FUNCTION(this << packet->GetSize());

    OranNtnIslHeader header;
    packet->RemoveHeader(header);

    NS_LOG_INFO("Space RIC (sat " << m_satId << "): Received ISL msg type="
                << (int)header.GetMessageType() << " from sat "
                << header.GetSourceSatId() << " payload=" << header.GetPayloadSize());

    switch (header.GetMessageType())
    {
    case IslMessageType::KPM_EXCHANGE:
        // Process KPM data from neighbor
        NS_LOG_INFO("  -> KPM exchange from neighbor satellite");
        break;

    case IslMessageType::MODEL_GRADIENTS:
        // Process federated learning gradients
        NS_LOG_INFO("  -> FL gradients received via ISL");
        break;

    case IslMessageType::HO_COORDINATION:
        // Process inter-satellite handover request
        NS_LOG_INFO("  -> Inter-sat HO coordination request");
        break;

    case IslMessageType::ENERGY_STATE:
        // Process energy state from neighbor
        NS_LOG_INFO("  -> Energy state update from neighbor");
        break;

    case IslMessageType::BEAM_COORDINATION:
        // Process beam coordination data
        NS_LOG_INFO("  -> Beam coordination data");
        break;

    case IslMessageType::HEARTBEAT:
        NS_LOG_DEBUG("  -> Heartbeat from neighbor");
        break;

    default:
        NS_LOG_WARN("  -> Unknown ISL message type");
        break;
    }

    m_metrics.islExchanges++;
}

void
OranNtnSpaceRic::SendIslMessage(uint32_t neighborSatId, IslMessageType msgType,
                                  const std::vector<uint8_t>& payload)
{
    NS_LOG_FUNCTION(this << neighborSatId << (int)msgType << payload.size());

    // Create ISL packet with header
    OranNtnIslHeader header;
    header.SetMessageType(msgType);
    header.SetSourceSatId(m_satId);
    header.SetDestSatId(neighborSatId);
    header.SetPayloadSize(static_cast<uint32_t>(payload.size()));
    header.SetTimestamp(Simulator::Now().GetSeconds());
    header.SetSequenceNumber(m_islSequenceNumber++);
    header.SetPriority(0);
    header.SetTtl(3);

    Ptr<Packet> packet = Create<Packet>(payload.data(), payload.size());
    packet->AddHeader(header);

    // Find neighbor Space RIC and deliver directly (simulation shortcut)
    for (auto& neighbor : m_islNeighbors)
    {
        if (neighbor->GetSatelliteId() == neighborSatId)
        {
            // Schedule delivery with ISL propagation delay
            Time islDelay = MilliSeconds(5); // Default ISL delay
            if (m_satBridge)
            {
                IslLinkState linkState = m_satBridge->GetIslLinkState(m_satId, neighborSatId);
                if (linkState.active)
                {
                    islDelay = MilliSeconds(linkState.delay_ms);
                }
            }

            Simulator::Schedule(islDelay,
                                &OranNtnSpaceRic::ReceiveIslPacket,
                                neighbor,
                                packet);

            NS_LOG_INFO("Space RIC (sat " << m_satId << "): Sent ISL msg type="
                        << (int)msgType << " to sat " << neighborSatId
                        << " delay=" << islDelay.GetMilliSeconds() << "ms");
            return;
        }
    }

    NS_LOG_WARN("Space RIC (sat " << m_satId << "): No ISL neighbor found for sat "
                << neighborSatId);
}

} // namespace ns3
