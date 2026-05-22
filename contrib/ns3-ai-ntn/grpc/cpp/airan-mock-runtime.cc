/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#include "airan-mock-runtime.h"

#include <algorithm>
#include <cmath>

namespace ns3
{
namespace oranntn
{
namespace airan
{

AiranInferenceServer::ModelHandler
AiranMockRuntime::MakePrecoderHandler(uint32_t num_layers,
                                       double base_latency_ms)
{
    return [num_layers, base_latency_ms](const InferenceRequest& req,
                                          InferenceResponse& out) {
        const CsiTensor& csi = req.csi;
        const uint32_t num_tx = std::max(csi.num_tx, 1u);
        out.output_kind = InferenceResponse::OutputKind::precoder;
        out.precoder.num_tx = num_tx;
        out.precoder.num_layers = num_layers;
        out.precoder.values.assign(2 * num_tx * num_layers, 0.0f);

        // Project each tx antenna onto each layer using a simple
        // deterministic kernel that depends on the request_id so the
        // tests can spot-check different responses are produced.
        const double seed = static_cast<double>(req.request_id);
        for (uint32_t tx = 0; tx < num_tx; ++tx)
        {
            for (uint32_t layer = 0; layer < num_layers; ++layer)
            {
                const double phase =
                    seed + 0.1 * tx + 0.37 * layer;
                const float re =
                    static_cast<float>(std::cos(phase) /
                                        std::sqrt(num_tx));
                const float im =
                    static_cast<float>(std::sin(phase) /
                                        std::sqrt(num_tx));
                const uint32_t off = 2 * (tx * num_layers + layer);
                out.precoder.values[off] = re;
                out.precoder.values[off + 1] = im;
            }
        }
        out.status = 0;
        out.inference_latency_ms =
            base_latency_ms +
            0.05 * std::abs(std::sin(seed * 0.01));
    };
}

AiranInferenceServer::ModelHandler
AiranMockRuntime::MakeBeamHandler(uint32_t top_k, double latency_ms)
{
    return [top_k, latency_ms](const InferenceRequest& req,
                                InferenceResponse& out) {
        const RsrpVector& v = req.rsrp;
        out.output_kind = InferenceResponse::OutputKind::beam;
        if (v.rsrp_dbm.empty())
        {
            out.beam.best_beam_id = 0;
            out.beam.score = 0.0f;
            out.status = 0;
            out.inference_latency_ms = latency_ms;
            return;
        }
        const auto max_it =
            std::max_element(v.rsrp_dbm.begin(), v.rsrp_dbm.end());
        out.beam.best_beam_id = static_cast<uint32_t>(
            std::distance(v.rsrp_dbm.begin(), max_it));
        out.beam.score = *max_it;
        // Build sorted top-K
        std::vector<float> sorted = v.rsrp_dbm;
        std::sort(sorted.begin(),
                   sorted.end(),
                   [](float a, float b) { return a > b; });
        const uint32_t k = std::min<uint32_t>(
            top_k, static_cast<uint32_t>(sorted.size()));
        out.beam.scores_top_k.assign(sorted.begin(),
                                      sorted.begin() + k);
        out.status = 0;
        out.inference_latency_ms = latency_ms;
    };
}

} // namespace airan
} // namespace oranntn
} // namespace ns3
