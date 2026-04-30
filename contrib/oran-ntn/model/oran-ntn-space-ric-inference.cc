/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-only
 * Author: Muhammad Uzair
 *
 * Space RIC On-Board Inference Engine Implementation
 *
 * Supports three inference backends with automatic fallback:
 *   LibTorch -> MsgInterface -> RuleBased
 */

#include "oran-ntn-space-ric-inference.h"

#include <ns3/enum.h>
#include <ns3/log.h>
#include <ns3/simulator.h>
#include <ns3/string.h>

#include <algorithm>
#include <chrono>
#include <cstring>
#include <numeric>

#ifdef ORAN_NTN_LIBTORCH_AVAILABLE
#include <torch/script.h>
#include <torch/torch.h>
#endif

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("OranNtnSpaceRicInference");
NS_OBJECT_ENSURE_REGISTERED(OranNtnSpaceRicInference);

TypeId
OranNtnSpaceRicInference::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::OranNtnSpaceRicInference")
            .SetParent<Object>()
            .SetGroupName("OranNtn")
            .AddConstructor<OranNtnSpaceRicInference>()
            .AddAttribute("PreferredBackend",
                          "Preferred inference backend (0=LibTorch, 1=MsgInterface, 2=RuleBased)",
                          EnumValue<Backend>(Backend::LIBTORCH),
                          MakeEnumAccessor<Backend>(&OranNtnSpaceRicInference::m_preferredBackend),
                          MakeEnumChecker(Backend::LIBTORCH,
                                          "LibTorch",
                                          Backend::MSG_INTERFACE,
                                          "MsgInterface",
                                          Backend::RULE_BASED,
                                          "RuleBased"))
            .AddTraceSource("InferenceCompleted",
                            "Fired after each inference with satId, backend, latency_ms",
                            MakeTraceSourceAccessor(&OranNtnSpaceRicInference::m_inferenceCompleted),
                            "ns3::TracedCallback::UintUintDouble");
    return tid;
}

OranNtnSpaceRicInference::OranNtnSpaceRicInference()
    : m_satId(0),
      m_preferredBackend(Backend::LIBTORCH),
      m_activeBackend(Backend::RULE_BASED),
      m_torchModelLoaded(false),
      m_torchModelPath("")
{
    NS_LOG_FUNCTION(this);
    std::memset(&m_metrics, 0, sizeof(InferenceMetrics));
}

OranNtnSpaceRicInference::~OranNtnSpaceRicInference()
{
    NS_LOG_FUNCTION(this);
}

void
OranNtnSpaceRicInference::DoDispose()
{
    NS_LOG_FUNCTION(this);
    m_modelWeights.clear();
    m_latencyHistory.clear();
    Object::DoDispose();
}

// ---- Configuration ----

void
OranNtnSpaceRicInference::SetPreferredBackend(Backend backend)
{
    NS_LOG_FUNCTION(this << static_cast<uint8_t>(backend));
    m_preferredBackend = backend;
}

void
OranNtnSpaceRicInference::SetSatelliteId(uint32_t satId)
{
    NS_LOG_FUNCTION(this << satId);
    m_satId = satId;
}

bool
OranNtnSpaceRicInference::IsLibTorchAvailable()
{
#ifdef ORAN_NTN_LIBTORCH_AVAILABLE
    return true;
#else
    return false;
#endif
}

// ---- Inference entry point ----

SpaceRicAction
OranNtnSpaceRicInference::Infer(const SpaceRicObservation& obs)
{
    NS_LOG_FUNCTION(this << obs.satId);

    auto startTime = std::chrono::steady_clock::now();
    SpaceRicAction action;
    std::memset(&action, 0, sizeof(SpaceRicAction));

    bool success = false;

    // Try preferred backend first
    Backend backend = m_preferredBackend;

    // Cascade: preferred -> LibTorch -> MsgInterface -> RuleBased
    if (!success && backend == Backend::LIBTORCH)
    {
        action = InferLibTorch(obs);
        if (action.confidence > 0.0f)
        {
            m_activeBackend = Backend::LIBTORCH;
            m_metrics.libtorchInferences++;
            success = true;
        }
    }

    if (!success && (backend == Backend::MSG_INTERFACE || backend == Backend::LIBTORCH))
    {
        action = InferMsgInterface(obs);
        if (action.confidence > 0.0f)
        {
            m_activeBackend = Backend::MSG_INTERFACE;
            m_metrics.msgInterfaceInferences++;
            if (backend != Backend::MSG_INTERFACE)
            {
                m_metrics.fallbackCount++;
            }
            success = true;
        }
    }

    if (!success)
    {
        action = InferRuleBased(obs);
        m_activeBackend = Backend::RULE_BASED;
        m_metrics.ruleBasedInferences++;
        if (backend != Backend::RULE_BASED)
        {
            m_metrics.fallbackCount++;
        }
        success = true;
    }

    // Track latency
    auto endTime = std::chrono::steady_clock::now();
    double latency_ms =
        std::chrono::duration<double, std::milli>(endTime - startTime).count();

    m_latencyHistory.push_back(latency_ms);
    if (m_latencyHistory.size() > 1000)
    {
        m_latencyHistory.pop_front();
    }

    // Update metrics
    m_metrics.totalInferences++;
    if (latency_ms > m_metrics.maxLatency_ms)
    {
        m_metrics.maxLatency_ms = latency_ms;
    }

    double sumLatency =
        std::accumulate(m_latencyHistory.begin(), m_latencyHistory.end(), 0.0);
    m_metrics.avgLatency_ms = sumLatency / static_cast<double>(m_latencyHistory.size());

    // Running average of confidence
    double n = static_cast<double>(m_metrics.totalInferences);
    m_metrics.avgConfidence =
        m_metrics.avgConfidence * ((n - 1.0) / n) + action.confidence / n;

    // Fire trace source
    m_inferenceCompleted(m_satId,
                         static_cast<uint8_t>(m_activeBackend),
                         latency_ms);

    NS_LOG_INFO("Sat " << m_satId << " inference: backend="
                        << static_cast<uint32_t>(m_activeBackend)
                        << " action=" << action.actionType
                        << " confidence=" << action.confidence
                        << " latency=" << latency_ms << " ms");

    return action;
}

// ---- LibTorch backend ----

SpaceRicAction
OranNtnSpaceRicInference::InferLibTorch(const SpaceRicObservation& obs)
{
    NS_LOG_FUNCTION(this);
    SpaceRicAction action;
    std::memset(&action, 0, sizeof(SpaceRicAction));

#ifdef ORAN_NTN_LIBTORCH_AVAILABLE
    if (!m_torchModelLoaded)
    {
        NS_LOG_WARN("LibTorch backend selected but no model loaded");
        return action; // confidence=0, triggers fallback
    }

    try
    {
        // Load the TorchScript model
        torch::jit::script::Module module = torch::jit::load(m_torchModelPath);
        module.eval();

        // Build input tensor from observation
        // Feature vector: per-beam loads + per-beam SINR + orbital + system state
        const uint32_t numFeatures = MAX_BEAMS_PER_SAT * 2 + 6 + 4;
        std::vector<float> features(numFeatures, 0.0f);

        // Per-beam loads
        for (uint32_t i = 0; i < MAX_BEAMS_PER_SAT; i++)
        {
            features[i] = obs.beamLoads[i];
        }
        // Per-beam SINR
        for (uint32_t i = 0; i < MAX_BEAMS_PER_SAT; i++)
        {
            features[MAX_BEAMS_PER_SAT + i] = obs.beamSinr[i];
        }
        // Orbital state
        uint32_t offset = MAX_BEAMS_PER_SAT * 2;
        features[offset + 0] = obs.latitude;
        features[offset + 1] = obs.longitude;
        features[offset + 2] = obs.altitude_km;
        features[offset + 3] = obs.velocity_x;
        features[offset + 4] = obs.velocity_y;
        features[offset + 5] = obs.velocity_z;
        // System state
        features[offset + 6] = obs.feederLinkAvail;
        features[offset + 7] = obs.batteryLevel;
        features[offset + 8] = obs.solarPower_W;
        features[offset + 9] = obs.computeUtilization;

        auto inputTensor =
            torch::from_blob(features.data(), {1, static_cast<long>(numFeatures)});

        std::vector<torch::jit::IValue> inputs;
        inputs.push_back(inputTensor);

        auto output = module.forward(inputs).toTensor();

        // Parse output: [actionType, targetBeamId, parameter1, confidence]
        action.actionType = static_cast<uint32_t>(output[0][0].item<float>());
        action.targetBeamId = static_cast<uint32_t>(output[0][1].item<float>());
        action.parameter1 = output[0][2].item<float>();
        action.confidence = std::clamp(output[0][3].item<float>(), 0.0f, 1.0f);
        action.numActions = 1;

        NS_LOG_DEBUG("LibTorch inference: action=" << action.actionType
                                                   << " confidence=" << action.confidence);
    }
    catch (const c10::Error& e)
    {
        NS_LOG_WARN("LibTorch inference failed: " << e.what());
        action.confidence = 0.0f; // triggers fallback
    }
#else
    NS_LOG_DEBUG("LibTorch not available at compile time, falling back");
    // action.confidence remains 0, triggering fallback
#endif

    return action;
}

// ---- MsgInterface backend ----

SpaceRicAction
OranNtnSpaceRicInference::InferMsgInterface(const SpaceRicObservation& obs)
{
    NS_LOG_FUNCTION(this);
    SpaceRicAction action;
    std::memset(&action, 0, sizeof(SpaceRicAction));

    // Fill observation for shared memory IPC
    SpaceRicObservation ipcObs;
    std::memcpy(&ipcObs, &obs, sizeof(SpaceRicObservation));

    NS_LOG_INFO("MsgInterface: prepared observation for sat " << obs.satId
                << " with " << obs.numActiveBeams << " active beams, "
                << obs.totalActiveUes << " UEs, avgSINR=" << obs.avgSinr_dB
                << " dB, battery=" << obs.batteryLevel);

    // Log the IPC operation -- actual shared memory transport requires
    // ns3-ai msg-interface runtime setup (Python agent must be running).
    // In production, this would call:
    //   Ns3AiMsgInterface::GetInstance()->Send(&ipcObs, sizeof(SpaceRicObservation));
    //   Ns3AiMsgInterface::GetInstance()->Recv(&action, sizeof(SpaceRicAction));
    NS_LOG_INFO("MsgInterface: IPC send/recv logged for sat " << obs.satId
                << " (actual IPC requires ns3-ai runtime setup)");

    // Return empty action (confidence=0) since actual IPC is not connected.
    // When ns3-ai Python agent is running, this would return a valid action.
    return action;
}

// ---- Rule-based backend ----

SpaceRicAction
OranNtnSpaceRicInference::InferRuleBased(const SpaceRicObservation& obs)
{
    NS_LOG_FUNCTION(this);
    SpaceRicAction action;
    std::memset(&action, 0, sizeof(SpaceRicAction));
    action.numActions = 1;

    // Rule 1: If any beam load > 0.9, reallocate that beam
    for (uint32_t i = 0; i < MAX_BEAMS_PER_SAT; i++)
    {
        if (obs.beamLoads[i] > 0.9f)
        {
            action.actionType = 1; // beam_realloc
            action.targetBeamId = i;
            action.parameter1 = obs.beamLoads[i];
            action.confidence = 0.85f; // high confidence -- clear overload
            NS_LOG_DEBUG("RuleBased: beam " << i << " overloaded (load="
                                            << obs.beamLoads[i] << "), beam_realloc");
            return action;
        }
    }

    // Rule 2: If any beam SINR < -5 dB, boost power
    for (uint32_t i = 0; i < MAX_BEAMS_PER_SAT; i++)
    {
        if (obs.beamSinr[i] < -5.0f && obs.beamLoads[i] > 0.0f)
        {
            action.actionType = 4; // power_ctrl
            action.targetBeamId = i;
            action.parameter1 = 3.0f; // boost by 3 dB
            action.confidence = 0.75f; // moderately confident
            NS_LOG_DEBUG("RuleBased: beam " << i << " low SINR ("
                                            << obs.beamSinr[i] << " dB), power_ctrl +3dB");
            return action;
        }
    }

    // Rule 3: If feeder link unavailable, be conservative (noop)
    if (obs.feederLinkAvail < 0.5f)
    {
        action.actionType = 0; // noop
        action.confidence = 0.9f; // very confident to do nothing
        NS_LOG_DEBUG("RuleBased: feeder link unavailable, conservative noop");
        return action;
    }

    // Rule 4: If battery low, shut down lowest-load beam
    if (obs.batteryLevel < 0.2f)
    {
        uint32_t lowestBeam = 0;
        float lowestLoad = 1.0f;
        for (uint32_t i = 0; i < MAX_BEAMS_PER_SAT; i++)
        {
            if (obs.beamLoads[i] < lowestLoad && obs.beamLoads[i] >= 0.0f)
            {
                lowestLoad = obs.beamLoads[i];
                lowestBeam = i;
            }
        }
        action.actionType = 5; // beam_shutdown
        action.targetBeamId = lowestBeam;
        action.parameter1 = obs.batteryLevel;
        action.confidence = 0.7f; // fairly confident -- battery critical
        NS_LOG_DEBUG("RuleBased: battery low (" << obs.batteryLevel
                                                << "), shutting down beam " << lowestBeam
                                                << " (load=" << lowestLoad << ")");
        return action;
    }

    // Default: noop -- nothing urgent detected
    action.actionType = 0; // noop
    action.confidence = 0.5f; // moderate confidence -- no clear signal
    NS_LOG_DEBUG("RuleBased: no urgent condition detected, noop");
    return action;
}

// ---- Backend selection ----

OranNtnSpaceRicInference::Backend
OranNtnSpaceRicInference::SelectBackend() const
{
    if (m_preferredBackend == Backend::LIBTORCH && IsLibTorchAvailable() && m_torchModelLoaded)
    {
        return Backend::LIBTORCH;
    }
    if (m_preferredBackend == Backend::MSG_INTERFACE ||
        m_preferredBackend == Backend::LIBTORCH)
    {
        return Backend::MSG_INTERFACE;
    }
    return Backend::RULE_BASED;
}

// ---- Model management ----

void
OranNtnSpaceRicInference::UpdateModelWeights(const std::string& modelName,
                                             const std::vector<double>& weights)
{
    NS_LOG_FUNCTION(this << modelName << weights.size());
    m_modelWeights[modelName] = weights;
    NS_LOG_INFO("Updated model weights for '" << modelName << "' with "
                                              << weights.size() << " parameters");
}

bool
OranNtnSpaceRicInference::LoadTorchModel(const std::string& modelPath)
{
    NS_LOG_FUNCTION(this << modelPath);
    m_torchModelPath = modelPath;

#ifdef ORAN_NTN_LIBTORCH_AVAILABLE
    try
    {
        // Validate that the model file can be loaded
        torch::jit::script::Module testModule = torch::jit::load(modelPath);
        m_torchModelLoaded = true;
        NS_LOG_INFO("TorchScript model loaded from " << modelPath);
        return true;
    }
    catch (const c10::Error& e)
    {
        NS_LOG_ERROR("Failed to load TorchScript model: " << e.what());
        m_torchModelLoaded = false;
        return false;
    }
#else
    NS_LOG_WARN("LibTorch not available at compile time, model path stored but not loaded: "
                << modelPath);
    m_torchModelLoaded = false;
    return false;
#endif
}

// ---- Metrics ----

OranNtnSpaceRicInference::Backend
OranNtnSpaceRicInference::GetActiveBackend() const
{
    return m_activeBackend;
}

double
OranNtnSpaceRicInference::GetAvgInferenceLatency_ms() const
{
    if (m_latencyHistory.empty())
    {
        return 0.0;
    }
    double sum = std::accumulate(m_latencyHistory.begin(), m_latencyHistory.end(), 0.0);
    return sum / static_cast<double>(m_latencyHistory.size());
}

OranNtnSpaceRicInference::InferenceMetrics
OranNtnSpaceRicInference::GetMetrics() const
{
    return m_metrics;
}

} // namespace ns3
