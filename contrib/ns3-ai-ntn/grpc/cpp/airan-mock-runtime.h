/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#ifndef NS3_AI_NTN_AIRAN_MOCK_RUNTIME_H
#define NS3_AI_NTN_AIRAN_MOCK_RUNTIME_H

// Deterministic mock inference runtimes used by tests and by the
// reference example scripts.
//
// MakePrecoderHandler() returns a handler that:
//   * computes maximum-ratio-transmission weights FROM the CSI, with
//     Gram-Schmidt orthogonalisation across layers (AI-06 - it previously
//     derived its output from req.request_id and read only csi.num_tx, so no
//     channel influenced any beam anywhere in the toolkit)
//   * produces unit-norm precoder weights of shape (num_tx, num_layers)
//     for the requested layer count (default 4)
//   * REFUSES a CSI tensor whose size disagrees with its declared dimensions,
//     or whose num_tx disagrees with the handler's declared input shape
//   * reports an inference_latency_ms drawn from a sin-of-request-id
//     ramp so the test can assert deterministic latency mixing
//
// This is a closed-form beamformer, not a learned one, and the name says mock
// for that reason. What it is not any more is a formula pretending to consume
// data it never read.
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
    ///
    /// \param expected_num_tx the transmit-antenna count this "model" is built
    ///        for. A real model has a fixed input shape - that is exactly what a
    ///        Triton config.pbtxt declares - and declaring it here is what lets
    ///        a TRANSPOSED tensor be caught: swapping num_tx with
    ///        num_subcarriers leaves the total element count unchanged, so a
    ///        size check alone cannot see it. 0 disables the check and accepts
    ///        whatever shape arrives.
    static AiranInferenceServer::ModelHandler
        MakePrecoderHandler(uint32_t num_layers = 4,
                            double base_latency_ms = 0.4,
                            uint32_t expected_num_tx = 0);

    /// Build a beam handler that returns top-K scores.
    static AiranInferenceServer::ModelHandler
        MakeBeamHandler(uint32_t top_k = 4,
                        double latency_ms = 0.25);
};

} // namespace airan
} // namespace oranntn
} // namespace ns3

#endif // NS3_AI_NTN_AIRAN_MOCK_RUNTIME_H
