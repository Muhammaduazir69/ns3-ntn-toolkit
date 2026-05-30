/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#ifndef ORAN_NTN_MMIMO_PRECODER_XAPP_H
#define ORAN_NTN_MMIMO_PRECODER_XAPP_H

// mMIMO AI-RAN two-stage precoder xApp (Roadmap §4.1.12).
//
// Architecture mirrors the NU × NVIDIA × AmpliTech open-source
// prototype announced 20 May 2026:
//
//   * Stage 1 — AI Aerial-style 4-layer NN precoder. CSI feedback in,
//                W_NN ∈ ℂ^(N_tx × N_layers) out. Lives at the
//                inference server (Triton) — reached via the T7
//                AiranInferenceClient.
//   * Stage 2 — 64-element codebook beamformer in the O-RU. For each
//                of the N_RF RF chains the xApp picks the codeword
//                that best matches the corresponding column of W_NN,
//                forming W_RF ∈ ℂ^(N_tx × N_RF).
//
// Final hybrid weights handed to the RU:
//
//     W = W_RF · W_BB ∈ ℂ^(N_tx × N_layers)
//
// where W_BB is the residual `(W_RF^H · W_NN)` digital-precoder
// correction. The xApp emits the result as an `E2RcAction` of type
// `BEAM_HOP_SCHEDULE` (cell-wide beam codebook update) addressed at
// the split-gNB's O-RU entity from §4.1.9.

#include "oran-ntn-mmimo-codebook.h"
#include "oran-ntn-split-gnb.h"
#include "oran-ntn-types.h"

#include "ns3/airan-inference-client.h"
#include "ns3/airan-messages.h"

#include <ns3/event-id.h>
#include <ns3/nstime.h>
#include <ns3/object.h>
#include <ns3/ptr.h>
#include <ns3/traced-callback.h>

#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace ns3
{

/// Result of the two-stage compose. Tests inspect every field.
struct TwoStagePrecoderResult
{
    /// One codebook index per RF chain.
    std::vector<uint32_t> codebook_indices;
    /// Final hybrid beamformer weights (re/im interleaved,
    /// N_tx · N_layers · 2 floats).
    std::vector<float> final_weights;
    /// W_BB residual correction (N_RF · N_layers · 2 floats).
    std::vector<float> bb_weights;
    uint32_t num_tx{0};
    uint32_t num_layers{0};
};

/// Standalone two-stage composer — kept outside the xApp so unit tests
/// can call it directly on synthetic W_NN inputs without an inference
/// server in the loop.
class OranNtnMmimoTwoStageComposer
{
  public:
    /// Compose hybrid beamformer.
    /// `nn_re_im` shape: (N_tx × N_layers × 2) flattened row-major
    /// over (tx, layer, re_im). Returns final hybrid weights in the
    /// same layout.
    static TwoStagePrecoderResult
        Compose(const std::vector<float>& nn_re_im,
                 uint32_t num_tx,
                 uint32_t num_layers,
                 const OranNtnMmimoCodebook& codebook);
};

/**
 * \ingroup oran-ntn
 * \brief Two-stage NN+codebook mMIMO precoder xApp (Roadmap §4.1.12).
 */
class OranNtnMmimoPrecoderXapp : public Object
{
  public:
    static TypeId GetTypeId();
    OranNtnMmimoPrecoderXapp();
    ~OranNtnMmimoPrecoderXapp() override;

    /// Configure the xApp identity and physical sizes.
    void Configure(uint32_t xapp_id,
                    uint32_t num_tx,
                    uint32_t num_layers);

    /// Provide the AI-Aerial-style NN precoder client (T7) +
    /// the model name string registered server-side (e.g.
    /// "precoder_csi_to_weights").
    void AttachInferenceClient(
        std::shared_ptr<oranntn::airan::AiranInferenceClient> client,
        const std::string& model_name);

    /// Provide the O-RU codebook (Stage 2).
    void AttachCodebook(Ptr<OranNtnMmimoCodebook> codebook);

    /// Provide the RU entity (from §4.1.9) that receives the final
    /// beamformer ControlAction.
    void AttachRu(Ptr<OranNtnSplitGnbEntity> ru) { m_ru = ru; }

    /// Entry-point from the DU/PHY: a CSI snapshot arrives for one UE
    /// at simulation time `t`. The xApp submits an inference request
    /// asynchronously; when the response lands the codebook stage and
    /// ControlAction follow.
    bool OnCsiReport(uint64_t ue_id,
                       uint64_t nr_cgi,
                       const oranntn::airan::CsiTensor& csi);

    // Counters
    uint64_t CsiInputsHandled() const { return m_csiInputs; }
    uint64_t ResponsesProcessed() const { return m_responses; }
    uint64_t ControlActionsEmitted() const { return m_actions; }
    uint64_t InferenceErrors() const { return m_errors; }

    /// Read the most recent compose result (test introspection).
    const TwoStagePrecoderResult& LastResult() const { return m_lastResult; }

    /// Optional observer fired whenever a ControlAction is dispatched
    /// to the RU. Tests use this to verify final weight values.
    using ActionObserver =
        std::function<void(const E2RcAction&,
                            const TwoStagePrecoderResult&)>;
    void SetActionObserver(ActionObserver cb) { m_observer = std::move(cb); }

  protected:
    void DoDispose() override;

  private:
    void HandleInferenceResponse(
        const oranntn::airan::InferenceResponse& resp,
        uint64_t ue_id,
        uint64_t nr_cgi);

    void EmitControlAction(uint64_t ue_id,
                              uint64_t nr_cgi,
                              const TwoStagePrecoderResult& result);

    uint32_t m_xappId{0};
    uint32_t m_numTx{0};
    uint32_t m_numLayers{0};
    std::string m_modelName;
    std::shared_ptr<oranntn::airan::AiranInferenceClient> m_client;
    Ptr<OranNtnMmimoCodebook> m_codebook;
    Ptr<OranNtnSplitGnbEntity> m_ru;
    ActionObserver m_observer;

    uint64_t m_csiInputs{0};
    uint64_t m_responses{0};
    uint64_t m_actions{0};
    uint64_t m_errors{0};
    TwoStagePrecoderResult m_lastResult;
};

} // namespace ns3

#endif // ORAN_NTN_MMIMO_PRECODER_XAPP_H
