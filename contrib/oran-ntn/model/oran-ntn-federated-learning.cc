/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-only
 * Author: Muhammad Uzair
 */

#include "oran-ntn-federated-learning.h"

#include "oran-ntn-sat-bridge.h"
#include "oran-ntn-space-ric.h"

#include <ns3/boolean.h>
#include <ns3/double.h>
#include <ns3/log.h>
#include <ns3/simulator.h>
#include <ns3/string.h>
#include <ns3/uinteger.h>

#include <algorithm>
#include <cmath>
#include <numeric>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("OranNtnFederatedLearning");
NS_OBJECT_ENSURE_REGISTERED(OranNtnFederatedLearning);

TypeId
OranNtnFederatedLearning::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::OranNtnFederatedLearning")
            .SetParent<Object>()
            .SetGroupName("OranNtn")
            .AddConstructor<OranNtnFederatedLearning>()
            .AddAttribute("MinParticipants",
                          "Minimum number of Space RICs required per FL round",
                          UintegerValue(3),
                          MakeUintegerAccessor(&OranNtnFederatedLearning::m_minParticipants),
                          MakeUintegerChecker<uint32_t>(1))
            .AddAttribute("RoundInterval",
                          "Interval between periodic FL rounds",
                          TimeValue(Seconds(60.0)),
                          MakeTimeAccessor(&OranNtnFederatedLearning::m_roundInterval),
                          MakeTimeChecker())
            .AddAttribute("AggregationStrategy",
                          "Gradient aggregation strategy (FedAvg, FedProx, FedNova)",
                          StringValue("FedAvg"),
                          MakeStringAccessor(&OranNtnFederatedLearning::m_aggregationStrategy),
                          MakeStringChecker())
            .AddAttribute("GradientCompression",
                          "Enable top-k gradient sparsification for communication efficiency",
                          BooleanValue(false),
                          MakeBooleanAccessor(&OranNtnFederatedLearning::m_gradientCompression),
                          MakeBooleanChecker())
            .AddAttribute("Asynchronous",
                          "Enable asynchronous FL with staleness-weighted aggregation",
                          BooleanValue(false),
                          MakeBooleanAccessor(&OranNtnFederatedLearning::m_asynchronous),
                          MakeBooleanChecker())
            .AddAttribute("EclipseAware",
                          "Exclude satellites in eclipse from FL rounds (battery preservation)",
                          BooleanValue(true),
                          MakeBooleanAccessor(&OranNtnFederatedLearning::m_eclipseAware),
                          MakeBooleanChecker())
            .AddTraceSource("RoundStarted",
                            "A new federated learning round has started",
                            MakeTraceSourceAccessor(
                                &OranNtnFederatedLearning::m_roundStarted),
                            "ns3::OranNtnFederatedLearning::RoundStartedTracedCallback")
            .AddTraceSource("RoundCompleted",
                            "A federated learning round has completed aggregation",
                            MakeTraceSourceAccessor(
                                &OranNtnFederatedLearning::m_roundCompleted),
                            "ns3::OranNtnFederatedLearning::RoundCompletedTracedCallback")
            .AddTraceSource("GradientReceived",
                            "Local gradients received from a Space RIC",
                            MakeTraceSourceAccessor(
                                &OranNtnFederatedLearning::m_gradientReceived),
                            "ns3::OranNtnFederatedLearning::GradientReceivedTracedCallback");
    return tid;
}

OranNtnFederatedLearning::OranNtnFederatedLearning()
    : m_satBridge(nullptr),
      m_minParticipants(3),
      m_roundInterval(Seconds(60.0)),
      m_aggregationStrategy("FedAvg"),
      m_gradientCompression(false),
      m_topKRatio(0.1),
      m_asynchronous(false),
      m_stalenessDecay(0.9),
      m_eclipseAware(true),
      m_nextRoundId(0),
      m_periodicActive(false)
{
    NS_LOG_FUNCTION(this);
    m_currentRound = {};
    m_flMetrics = {};
}

OranNtnFederatedLearning::~OranNtnFederatedLearning()
{
    NS_LOG_FUNCTION(this);
}

void
OranNtnFederatedLearning::DoDispose()
{
    NS_LOG_FUNCTION(this);
    StopPeriodicRounds();
    m_satBridge = nullptr;
    m_spaceRics.clear();
    m_collectedGradients.clear();
    m_globalModels.clear();
    m_globalModelVersions.clear();
    m_roundHistory.clear();
    Object::DoDispose();
}

// ============================================================================
//  Configuration
// ============================================================================

void
OranNtnFederatedLearning::SetSatBridge(Ptr<OranNtnSatBridge> bridge)
{
    NS_LOG_FUNCTION(this << bridge);
    m_satBridge = bridge;
}

void
OranNtnFederatedLearning::SetMinParticipants(uint32_t n)
{
    NS_LOG_FUNCTION(this << n);
    m_minParticipants = n;
}

void
OranNtnFederatedLearning::SetRoundInterval(Time interval)
{
    NS_LOG_FUNCTION(this << interval);
    m_roundInterval = interval;
}

void
OranNtnFederatedLearning::SetAggregationStrategy(const std::string& strategy)
{
    NS_LOG_FUNCTION(this << strategy);
    m_aggregationStrategy = strategy;
}

void
OranNtnFederatedLearning::SetGradientCompression(bool enable, double topKRatio)
{
    NS_LOG_FUNCTION(this << enable << topKRatio);
    m_gradientCompression = enable;
    m_topKRatio = topKRatio;
}

void
OranNtnFederatedLearning::SetAsynchronous(bool enable, double stalenessDecay)
{
    NS_LOG_FUNCTION(this << enable << stalenessDecay);
    m_asynchronous = enable;
    m_stalenessDecay = stalenessDecay;
}

void
OranNtnFederatedLearning::SetEclipseAware(bool enable)
{
    NS_LOG_FUNCTION(this << enable);
    m_eclipseAware = enable;
}

// ============================================================================
//  Participant Management
// ============================================================================

void
OranNtnFederatedLearning::RegisterSpaceRic(Ptr<OranNtnSpaceRic> spaceRic)
{
    NS_LOG_FUNCTION(this << spaceRic);
    uint32_t satId = spaceRic->GetSatelliteId();
    m_spaceRics[satId] = spaceRic;
    NS_LOG_INFO("FL: Registered Space RIC for satellite " << satId
                << ", total participants=" << m_spaceRics.size());
}

void
OranNtnFederatedLearning::UnregisterSpaceRic(uint32_t satId)
{
    NS_LOG_FUNCTION(this << satId);
    auto it = m_spaceRics.find(satId);
    if (it != m_spaceRics.end())
    {
        m_spaceRics.erase(it);
        NS_LOG_INFO("FL: Unregistered Space RIC for satellite " << satId);
    }
}

// ============================================================================
//  Participant Selection
// ============================================================================

std::vector<uint32_t>
OranNtnFederatedLearning::SelectParticipants() const
{
    NS_LOG_FUNCTION(this);

    std::vector<uint32_t> candidates;
    std::vector<uint32_t> preferred;

    for (const auto& entry : m_spaceRics)
    {
        uint32_t satId = entry.first;
        bool eligible = true;

        // Eclipse-aware filtering: exclude satellites in eclipse to preserve battery
        if (m_eclipseAware && m_satBridge)
        {
            const SatelliteBridgeState& satState = m_satBridge->GetSatState(satId);
            // Check feeder link as proxy for eclipse state; satellites without
            // feeder link may be in eclipse or behind Earth
            if (!satState.feederLinkAvailable)
            {
                NS_LOG_DEBUG("FL: Satellite " << satId
                             << " excluded (no feeder link / potential eclipse)");
                eligible = false;
            }
        }

        if (eligible)
        {
            candidates.push_back(satId);

            // Orbit-aware: prefer satellites with active feeder links
            if (m_satBridge)
            {
                const SatelliteBridgeState& satState = m_satBridge->GetSatState(satId);
                if (satState.feederLinkAvailable)
                {
                    preferred.push_back(satId);
                }
            }
        }
    }

    // Build final participant list: prefer satellites with feeder links
    std::vector<uint32_t> selected;
    if (!preferred.empty())
    {
        selected = preferred;
    }
    else
    {
        selected = candidates;
    }

    // Ensure at least minParticipants if possible
    if (selected.size() < m_minParticipants && candidates.size() >= m_minParticipants)
    {
        // Add remaining candidates to reach minimum
        for (uint32_t satId : candidates)
        {
            if (std::find(selected.begin(), selected.end(), satId) == selected.end())
            {
                selected.push_back(satId);
                if (selected.size() >= m_minParticipants)
                {
                    break;
                }
            }
        }
    }

    // If still not enough candidates, fall back to all registered Space RICs
    if (selected.size() < m_minParticipants)
    {
        selected.clear();
        for (const auto& entry : m_spaceRics)
        {
            selected.push_back(entry.first);
        }
    }

    NS_LOG_INFO("FL: Selected " << selected.size() << " participants"
                << " (min=" << m_minParticipants << ")");
    return selected;
}

// ============================================================================
//  FL Round Lifecycle
// ============================================================================

void
OranNtnFederatedLearning::InitializeFederatedRound(const std::string& modelName)
{
    NS_LOG_FUNCTION(this << modelName);

    // Increment round ID
    uint32_t roundId = m_nextRoundId++;
    m_currentModelName = modelName;

    // Create new round state
    FederatedLearningRound round;
    round.roundId = roundId;
    round.modelName = modelName;
    round.timestamp = Simulator::Now().GetSeconds();
    round.aggregationStrategy = m_aggregationStrategy;
    round.aggregationLoss = 0.0;
    round.completed = false;

    // Select participants
    std::vector<uint32_t> participants = SelectParticipants();
    round.numParticipants = static_cast<uint32_t>(participants.size());
    round.numGradientsReceived = 0;

    m_currentRound = round;

    // Clear collected gradients from previous round
    m_collectedGradients.clear();

    // Fire trace
    m_roundStarted(roundId, modelName);

    // Update metrics
    m_flMetrics.totalRounds++;

    NS_LOG_INFO("FL: Initialized round " << roundId
                << " for model '" << modelName << "'"
                << " with " << participants.size() << " participants"
                << " strategy=" << m_aggregationStrategy);
}

void
OranNtnFederatedLearning::CollectLocalGradients(uint32_t satId,
                                                 const std::string& modelName,
                                                 const std::vector<double>& gradients,
                                                 double localLoss,
                                                 uint32_t localSamples)
{
    NS_LOG_FUNCTION(this << satId << modelName << localLoss << localSamples);

    GradientEntry entry;
    entry.satId = satId;
    entry.localLoss = localLoss;
    entry.localSamples = localSamples;
    entry.timestamp = Simulator::Now().GetSeconds();

    // Apply gradient compression if enabled (top-k sparsification)
    if (m_gradientCompression && !gradients.empty())
    {
        entry.gradients = CompressGradients(gradients);
        NS_LOG_DEBUG("FL: Compressed gradients from satellite " << satId
                     << " (topK=" << m_topKRatio << ")");
    }
    else
    {
        entry.gradients = gradients;
    }

    m_collectedGradients.push_back(entry);
    m_currentRound.numGradientsReceived++;

    // Fire trace
    m_gradientReceived(m_currentRound.roundId, satId);

    // Update metrics
    m_flMetrics.totalGradientsExchanged++;
    m_flMetrics.totalCommsBytes += static_cast<double>(entry.gradients.size()) * sizeof(double);

    NS_LOG_INFO("FL: Received gradients from satellite " << satId
                << " loss=" << localLoss
                << " samples=" << localSamples
                << " (" << m_currentRound.numGradientsReceived
                << "/" << m_currentRound.numParticipants << ")");

    // Check if enough gradients collected to trigger aggregation
    if (m_currentRound.numGradientsReceived >= m_currentRound.numParticipants)
    {
        NS_LOG_INFO("FL: All gradients collected, triggering aggregation");
        AggregateGradients();
    }
    else if (m_asynchronous &&
             m_currentRound.numGradientsReceived >= m_minParticipants)
    {
        // Asynchronous mode: aggregate when minimum threshold is met
        NS_LOG_INFO("FL: Async mode - minimum gradients collected, triggering aggregation");
        AggregateGradients();
    }
}

void
OranNtnFederatedLearning::AggregateGradients()
{
    NS_LOG_FUNCTION(this);

    if (m_collectedGradients.empty())
    {
        NS_LOG_WARN("FL: No gradients to aggregate");
        return;
    }

    // Prepare gradient-samples pairs for aggregation
    std::vector<std::pair<std::vector<double>, uint32_t>> gradientPairs;
    gradientPairs.reserve(m_collectedGradients.size());

    for (const auto& entry : m_collectedGradients)
    {
        gradientPairs.emplace_back(entry.gradients, entry.localSamples);
    }

    // Perform aggregation based on strategy
    std::vector<double> globalWeights;

    if (m_aggregationStrategy == "FedAvg")
    {
        globalWeights = FedAvgAggregate(gradientPairs);
    }
    else if (m_aggregationStrategy == "FedProx")
    {
        double mu = 0.01; // proximal term coefficient
        globalWeights = FedProxAggregate(gradientPairs, mu);
    }
    else if (m_aggregationStrategy == "FedNova")
    {
        globalWeights = FedNovaAggregate(gradientPairs);
    }
    else
    {
        NS_LOG_WARN("FL: Unknown aggregation strategy '"
                     << m_aggregationStrategy << "', falling back to FedAvg");
        globalWeights = FedAvgAggregate(gradientPairs);
    }

    // Store global model
    m_globalModels[m_currentModelName] = globalWeights;

    // Increment global model version
    m_globalModelVersions[m_currentModelName]++;

    // Compute aggregated loss (weighted average of local losses)
    double totalSamples = 0.0;
    double weightedLoss = 0.0;
    for (const auto& entry : m_collectedGradients)
    {
        weightedLoss += entry.localLoss * static_cast<double>(entry.localSamples);
        totalSamples += static_cast<double>(entry.localSamples);
    }
    double globalLoss = (totalSamples > 0.0) ? (weightedLoss / totalSamples) : 0.0;

    // Update current round state
    m_currentRound.aggregationLoss = globalLoss;
    m_currentRound.completed = true;

    // Archive round to history
    m_roundHistory.push_back(m_currentRound);

    // Update metrics
    m_flMetrics.completedRounds++;
    m_flMetrics.globalLoss = globalLoss;
    double totalParticipation = 0.0;
    for (const auto& r : m_roundHistory)
    {
        totalParticipation += static_cast<double>(r.numGradientsReceived) /
                              std::max(static_cast<double>(r.numParticipants), 1.0);
    }
    m_flMetrics.avgParticipationRate =
        totalParticipation / static_cast<double>(m_roundHistory.size());

    // Fire trace
    m_roundCompleted(m_currentRound.roundId, globalLoss);

    NS_LOG_INFO("FL: Round " << m_currentRound.roundId << " completed"
                << " strategy=" << m_aggregationStrategy
                << " globalLoss=" << globalLoss
                << " modelVersion=" << m_globalModelVersions[m_currentModelName]
                << " participants=" << m_collectedGradients.size());
}

// ============================================================================
//  Aggregation Strategies
// ============================================================================

std::vector<double>
OranNtnFederatedLearning::FedAvgAggregate(
    const std::vector<std::pair<std::vector<double>, uint32_t>>& gradients) const
{
    NS_LOG_FUNCTION(this);

    if (gradients.empty())
    {
        return {};
    }

    // Determine weight vector size from first entry
    size_t weightSize = gradients.front().first.size();

    // Compute total number of samples across all participants
    double totalSamples = 0.0;
    for (const auto& g : gradients)
    {
        totalSamples += static_cast<double>(g.second);
    }

    if (totalSamples <= 0.0)
    {
        NS_LOG_WARN("FL FedAvg: Total samples is zero, returning unweighted average");
        totalSamples = static_cast<double>(gradients.size());
    }

    // Weighted average: globalW[i] = sum(localW[i] * numSamples) / totalSamples
    std::vector<double> globalWeights(weightSize, 0.0);

    for (const auto& g : gradients)
    {
        double weight = (totalSamples > 0.0)
                            ? static_cast<double>(g.second) / totalSamples
                            : 1.0 / static_cast<double>(gradients.size());

        const std::vector<double>& localW = g.first;
        size_t len = std::min(localW.size(), weightSize);
        for (size_t i = 0; i < len; ++i)
        {
            globalWeights[i] += localW[i] * weight;
        }
    }

    NS_LOG_DEBUG("FL FedAvg: Aggregated " << gradients.size()
                 << " gradient sets, totalSamples=" << totalSamples
                 << " weightDim=" << weightSize);

    return globalWeights;
}

std::vector<double>
OranNtnFederatedLearning::FedProxAggregate(
    const std::vector<std::pair<std::vector<double>, uint32_t>>& gradients,
    double mu) const
{
    NS_LOG_FUNCTION(this << mu);

    if (gradients.empty())
    {
        return {};
    }

    size_t weightSize = gradients.front().first.size();

    // Start with FedAvg aggregation
    std::vector<double> globalWeights = FedAvgAggregate(gradients);

    // Apply proximal term: globalW[i] += mu * (globalW_prev[i] - localW_avg[i])
    // This regularizes towards the previous global model
    auto prevModelIt = m_globalModels.find(m_currentModelName);
    if (prevModelIt != m_globalModels.end() && !prevModelIt->second.empty())
    {
        const std::vector<double>& prevGlobal = prevModelIt->second;

        // Compute average of local weights (unweighted for proximal term)
        std::vector<double> localAvg(weightSize, 0.0);
        for (const auto& g : gradients)
        {
            size_t len = std::min(g.first.size(), weightSize);
            for (size_t i = 0; i < len; ++i)
            {
                localAvg[i] += g.first[i];
            }
        }
        double n = static_cast<double>(gradients.size());
        for (size_t i = 0; i < weightSize; ++i)
        {
            localAvg[i] /= n;
        }

        // Add proximal regularization
        size_t len = std::min(prevGlobal.size(), weightSize);
        for (size_t i = 0; i < len; ++i)
        {
            globalWeights[i] += mu * (prevGlobal[i] - localAvg[i]);
        }

        NS_LOG_DEBUG("FL FedProx: Applied proximal term with mu=" << mu);
    }
    else
    {
        NS_LOG_DEBUG("FL FedProx: No previous global model, proximal term skipped");
    }

    return globalWeights;
}

std::vector<double>
OranNtnFederatedLearning::FedNovaAggregate(
    const std::vector<std::pair<std::vector<double>, uint32_t>>& gradients) const
{
    NS_LOG_FUNCTION(this);

    if (gradients.empty())
    {
        return {};
    }

    size_t weightSize = gradients.front().first.size();

    // FedNova: normalize by effective number of local steps
    // tau_i = numSamples (proxy for local computation steps)
    // globalW[i] = sum(tau_i * localW[i]) / sum(tau_i)
    double totalTau = 0.0;
    for (const auto& g : gradients)
    {
        totalTau += static_cast<double>(g.second);
    }

    if (totalTau <= 0.0)
    {
        NS_LOG_WARN("FL FedNova: Total tau is zero, returning unweighted average");
        return FedAvgAggregate(gradients);
    }

    std::vector<double> globalWeights(weightSize, 0.0);

    for (const auto& g : gradients)
    {
        double tau_i = static_cast<double>(g.second);
        double normalizedWeight = tau_i / totalTau;

        const std::vector<double>& localW = g.first;
        size_t len = std::min(localW.size(), weightSize);
        for (size_t i = 0; i < len; ++i)
        {
            globalWeights[i] += localW[i] * normalizedWeight;
        }
    }

    NS_LOG_DEBUG("FL FedNova: Aggregated " << gradients.size()
                 << " gradient sets with normalized averaging"
                 << " totalTau=" << totalTau);

    return globalWeights;
}

// ============================================================================
//  Gradient Compression
// ============================================================================

std::vector<double>
OranNtnFederatedLearning::CompressGradients(const std::vector<double>& gradients) const
{
    NS_LOG_FUNCTION(this);

    if (gradients.empty())
    {
        return gradients;
    }

    // Top-k sparsification: keep only top (topKRatio * size) elements by magnitude
    size_t totalSize = gradients.size();
    size_t k = std::max(static_cast<size_t>(1),
                        static_cast<size_t>(std::ceil(m_topKRatio * totalSize)));
    k = std::min(k, totalSize);

    // Create index-magnitude pairs for sorting
    std::vector<std::pair<double, size_t>> magnitudes;
    magnitudes.reserve(totalSize);
    for (size_t i = 0; i < totalSize; ++i)
    {
        magnitudes.emplace_back(std::abs(gradients[i]), i);
    }

    // Partial sort to find top-k by magnitude
    std::partial_sort(magnitudes.begin(),
                      magnitudes.begin() + static_cast<long>(k),
                      magnitudes.end(),
                      [](const std::pair<double, size_t>& a,
                         const std::pair<double, size_t>& b) {
                          return a.first > b.first;
                      });

    // Build compressed vector: keep top-k, zero out the rest
    std::vector<double> compressed(totalSize, 0.0);
    for (size_t j = 0; j < k; ++j)
    {
        size_t idx = magnitudes[j].second;
        compressed[idx] = gradients[idx];
    }

    NS_LOG_DEBUG("FL: Compressed gradients - kept " << k << "/" << totalSize
                 << " elements (ratio=" << m_topKRatio << ")");

    return compressed;
}

// ============================================================================
//  Global Model Distribution
// ============================================================================

void
OranNtnFederatedLearning::DistributeGlobalModel()
{
    NS_LOG_FUNCTION(this);

    auto modelIt = m_globalModels.find(m_currentModelName);
    if (modelIt == m_globalModels.end() || modelIt->second.empty())
    {
        NS_LOG_WARN("FL: No global model to distribute for '" << m_currentModelName << "'");
        return;
    }

    const std::vector<double>& globalWeights = modelIt->second;
    uint32_t version = m_globalModelVersions[m_currentModelName];
    uint32_t distributed = 0;

    for (const auto& entry : m_spaceRics)
    {
        uint32_t satId = entry.first;
        Ptr<OranNtnSpaceRic> spaceRic = entry.second;

        if (spaceRic)
        {
            spaceRic->ReceiveModelUpdate(m_currentModelName, version, globalWeights);
            distributed++;

            NS_LOG_DEBUG("FL: Distributed global model v" << version
                         << " to satellite " << satId);
        }
    }

    NS_LOG_INFO("FL: Distributed global model '" << m_currentModelName
                << "' v" << version
                << " to " << distributed << " Space RICs");
}

// ============================================================================
//  Periodic Round Management
// ============================================================================

void
OranNtnFederatedLearning::StartPeriodicRounds(const std::string& modelName)
{
    NS_LOG_FUNCTION(this << modelName);

    if (m_periodicActive)
    {
        NS_LOG_WARN("FL: Periodic rounds already active, stopping previous schedule");
        StopPeriodicRounds();
    }

    m_currentModelName = modelName;
    m_periodicActive = true;

    // Schedule first round immediately
    m_periodicRoundEvent = Simulator::Schedule(
        Seconds(0.0),
        &OranNtnFederatedLearning::PeriodicRoundCallback,
        this);

    NS_LOG_INFO("FL: Started periodic rounds for model '" << modelName
                << "' interval=" << m_roundInterval.As(Time::S));
}

void
OranNtnFederatedLearning::StopPeriodicRounds()
{
    NS_LOG_FUNCTION(this);

    if (m_periodicActive)
    {
        m_periodicActive = false;
        if (m_periodicRoundEvent.IsPending())
        {
            Simulator::Cancel(m_periodicRoundEvent);
        }
        NS_LOG_INFO("FL: Stopped periodic rounds");
    }
}

void
OranNtnFederatedLearning::PeriodicRoundCallback()
{
    NS_LOG_FUNCTION(this);

    if (!m_periodicActive)
    {
        return;
    }

    // Initialize a new round
    InitializeFederatedRound(m_currentModelName);

    // Collect gradients from all registered participants
    for (const auto& entry : m_spaceRics)
    {
        uint32_t satId = entry.first;
        Ptr<OranNtnSpaceRic> spaceRic = entry.second;

        if (spaceRic)
        {
            std::vector<double> localGradients = spaceRic->GetLocalGradients(m_currentModelName);
            if (!localGradients.empty())
            {
                // Use default values for loss and samples when pulling from Space RICs
                double localLoss = 0.0;
                uint32_t localSamples = 1;

                CollectLocalGradients(satId, m_currentModelName,
                                      localGradients, localLoss, localSamples);
            }
            else
            {
                NS_LOG_DEBUG("FL: Satellite " << satId
                             << " returned empty gradients, skipping");
            }
        }
    }

    // If aggregation was not automatically triggered (insufficient gradients),
    // attempt aggregation with what we have
    if (!m_currentRound.completed && !m_collectedGradients.empty())
    {
        NS_LOG_INFO("FL: Periodic round - forcing aggregation with "
                     << m_collectedGradients.size() << " gradients");
        AggregateGradients();
    }

    // Distribute the updated global model to participants
    if (m_currentRound.completed)
    {
        DistributeGlobalModel();
    }
    else
    {
        m_flMetrics.failedRounds++;
        NS_LOG_WARN("FL: Round " << m_currentRound.roundId
                     << " failed - no gradients collected");
    }

    // Schedule next round
    m_periodicRoundEvent = Simulator::Schedule(
        m_roundInterval,
        &OranNtnFederatedLearning::PeriodicRoundCallback,
        this);
}

// ============================================================================
//  State Queries
// ============================================================================

FederatedLearningRound
OranNtnFederatedLearning::GetCurrentRound() const
{
    return m_currentRound;
}

std::vector<FederatedLearningRound>
OranNtnFederatedLearning::GetRoundHistory() const
{
    return m_roundHistory;
}

uint32_t
OranNtnFederatedLearning::GetGlobalModelVersion(const std::string& modelName) const
{
    auto it = m_globalModelVersions.find(modelName);
    if (it != m_globalModelVersions.end())
    {
        return it->second;
    }
    return 0;
}

// ============================================================================
//  Metrics
// ============================================================================

OranNtnFederatedLearning::FLMetrics
OranNtnFederatedLearning::GetMetrics() const
{
    return m_flMetrics;
}

} // namespace ns3
