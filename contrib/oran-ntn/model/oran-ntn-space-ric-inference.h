/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-only
 * Author: Muhammad Uzair
 *
 * Space RIC On-Board Inference Engine
 *
 * Encapsulates on-board AI inference with multiple backends:
 *   1. LibTorch (fastest, pure C++, requires libtorch installation)
 *   2. ns3-ai msg-interface (Python agent via shared memory IPC)
 *   3. Rule-based fallback (no ML dependencies)
 *
 * Novel features:
 *   - Automatic backend selection based on available libraries
 *   - Model hot-swapping from federated learning updates
 *   - Quantized model support for OBP memory constraints
 *   - Inference latency tracking for real-time decision budgeting
 *   - Multi-model ensemble for improved confidence
 *   - Graceful degradation chain: LibTorch → IPC → rule-based
 */

#ifndef ORAN_NTN_SPACE_RIC_INFERENCE_H
#define ORAN_NTN_SPACE_RIC_INFERENCE_H

#include "oran-ntn-msg-interface.h"

#include <ns3/object.h>
#include <ns3/traced-callback.h>

#include <deque>
#include <map>
#include <string>
#include <vector>

namespace ns3
{

/**
 * \ingroup oran-ntn
 * \brief On-board inference engine for Space RIC
 */
class OranNtnSpaceRicInference : public Object
{
  public:
    static TypeId GetTypeId();
    OranNtnSpaceRicInference();
    ~OranNtnSpaceRicInference() override;

    /**
     * \brief Available inference backends
     */
    enum class Backend : uint8_t
    {
        LIBTORCH = 0,      //!< Pure C++ inference via TorchScript
        MSG_INTERFACE = 1, //!< Python agent via shared memory IPC
        RULE_BASED = 2,    //!< Hand-coded rules (no ML)
    };

    // ---- Configuration ----
    void SetPreferredBackend(Backend backend);
    void SetSatelliteId(uint32_t satId);

    /**
     * \brief Load a TorchScript model for LibTorch backend
     * \param modelPath Path to .pt TorchScript file
     * \return true if model loaded successfully
     */
    bool LoadTorchModel(const std::string& modelPath);

    /**
     * \brief Update model weights (from federated learning)
     */
    void UpdateModelWeights(const std::string& modelName,
                            const std::vector<double>& weights);

    /**
     * \brief Check if LibTorch is available at compile time
     */
    static bool IsLibTorchAvailable();

    // ---- Inference ----

    /**
     * \brief Run inference on Space RIC observation
     * \return Action to execute
     */
    SpaceRicAction Infer(const SpaceRicObservation& obs);

    /**
     * \brief Get the backend currently in use
     */
    Backend GetActiveBackend() const;

    /**
     * \brief Get average inference latency
     */
    double GetAvgInferenceLatency_ms() const;

    // ---- Metrics ----
    struct InferenceMetrics
    {
        uint32_t totalInferences;
        uint32_t libtorchInferences;
        uint32_t msgInterfaceInferences;
        uint32_t ruleBasedInferences;
        double avgLatency_ms;
        double maxLatency_ms;
        uint32_t fallbackCount;
        double avgConfidence;
    };
    InferenceMetrics GetMetrics() const;

    // ---- Trace sources ----
    TracedCallback<uint32_t, uint8_t, double> m_inferenceCompleted;  //!< satId, backend, latency

  protected:
    void DoDispose() override;

  private:
    /**
     * \brief LibTorch inference (fastest)
     */
    SpaceRicAction InferLibTorch(const SpaceRicObservation& obs);

    /**
     * \brief IPC inference via ns3-ai msg-interface
     */
    SpaceRicAction InferMsgInterface(const SpaceRicObservation& obs);

    /**
     * \brief Rule-based inference (fallback)
     */
    SpaceRicAction InferRuleBased(const SpaceRicObservation& obs);

    /**
     * \brief Select best available backend
     */
    Backend SelectBackend() const;

    uint32_t m_satId;
    Backend m_preferredBackend;
    Backend m_activeBackend;

    // Model state
    bool m_torchModelLoaded;
    std::string m_torchModelPath;
    std::map<std::string, std::vector<double>> m_modelWeights;

    // Metrics
    InferenceMetrics m_metrics;
    std::deque<double> m_latencyHistory;
};

} // namespace ns3

#endif // ORAN_NTN_SPACE_RIC_INFERENCE_H
