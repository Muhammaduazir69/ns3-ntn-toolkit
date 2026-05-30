/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#ifndef NS3_AI_NTN_AIRAN_MOCK_RUNTIME_H
#define NS3_AI_NTN_AIRAN_MOCK_RUNTIME_H

// Deterministic mock inference runtimes used by tests and by the
// reference example scripts.
//
// MakePrecoderHandler() returns a handler that:
//   * normalises the input CSI complex values
//   * produces unit-norm precoder weights of shape (num_tx, num_layers)
//     for the requested layer count (default 4)
//   * reports an inference_latency_ms drawn from a sin-of-request-id
//     ramp so the test can assert deterministic latency mixing
//
// MakeBeamHandler() returns a handler that:
//   * argmax's the RSRP vector to pick best_beam_id
//   * returns the top-K scores in descending order
//   * reports a constant inference_latency_ms

#include "airan-inference-server.h"
#include "airan-messages.h"

#include <cstdint>

namespace ns3
{
namespace oranntn
{
namespace airan
{

class AiranMockRuntime
{
  public:
    /// Build a precoder handler that returns `num_layers` layers.
    static AiranInferenceServer::ModelHandler
        MakePrecoderHandler(uint32_t num_layers = 4,
                            double base_latency_ms = 0.4);

    /// Build a beam handler that returns top-K scores.
    static AiranInferenceServer::ModelHandler
        MakeBeamHandler(uint32_t top_k = 4,
                        double latency_ms = 0.25);
};

} // namespace airan
} // namespace oranntn
} // namespace ns3

#endif // NS3_AI_NTN_AIRAN_MOCK_RUNTIME_H
