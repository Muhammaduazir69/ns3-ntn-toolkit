/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#include "oran-ntn-mmimo-precoder-xapp.h"

#include "ns3/log.h"
#include "ns3/simulator.h"

#include <cmath>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("OranNtnMmimoPrecoderXapp");

namespace
{

/// Helpers operating on row-major flattened (tx, layer, re_im) buffers.

void
GetColumn(const std::vector<float>& re_im,
           uint32_t num_tx,
           uint32_t num_layers,
           uint32_t layer,
           std::vector<float>& out)
{
    out.assign(2 * num_tx, 0.0f);
    if (layer >= num_layers)
    {
        return;
    }
    for (uint32_t tx = 0; tx < num_tx; ++tx)
    {
        const uint32_t src = 2 * (tx * num_layers + layer);
        out[2 * tx] = re_im[src];
        out[2 * tx + 1] = re_im[src + 1];
    }
}

/// Complex inner product <c, t> = Σ_n c_n* · t_n (conjugate of `c`).
void
InnerProduct(const std::vector<float>& c_re_im,
              const std::vector<float>& t_re_im,
              uint32_t num_tx,
              float& re_out,
              float& im_out)
{
    double re_sum = 0.0;
    double im_sum = 0.0;
    for (uint32_t n = 0; n < num_tx; ++n)
    {
        const double re_c = c_re_im[2 * n];
        const double im_c = c_re_im[2 * n + 1];
        const double re_t = t_re_im[2 * n];
        const double im_t = t_re_im[2 * n + 1];
        re_sum += re_c * re_t + im_c * im_t;
        im_sum += re_c * im_t - im_c * re_t;
    }
    re_out = static_cast<float>(re_sum);
    im_out = static_cast<float>(im_sum);
}

} // namespace

// ----------------------------------------------------------------------------
// OranNtnMmimoTwoStageComposer
// ----------------------------------------------------------------------------

TwoStagePrecoderResult
OranNtnMmimoTwoStageComposer::Compose(
    const std::vector<float>& nn_re_im,
    uint32_t num_tx,
    uint32_t num_layers,
    const OranNtnMmimoCodebook& codebook)
{
    TwoStagePrecoderResult out;
    out.num_tx = num_tx;
    out.num_layers = num_layers;
    if (num_tx == 0 || num_layers == 0 ||
        nn_re_im.size() != static_cast<size_t>(2 * num_tx * num_layers) ||
        codebook.NumTx() != num_tx || codebook.Size() == 0)
    {
        return out;
    }

    // For each layer, pick the codeword that best matches that column
    // of W_NN. The codebook size is `M`, RF chains = num_layers.
    out.codebook_indices.assign(num_layers, 0);
    std::vector<float> col(2 * num_tx, 0.0f);
    for (uint32_t layer = 0; layer < num_layers; ++layer)
    {
        GetColumn(nn_re_im, num_tx, num_layers, layer, col);
        out.codebook_indices[layer] = codebook.BestMatch(col);
    }

    // Assemble W_RF (num_tx × num_layers, complex). Column ℓ = codebook
    // entry at index codebook_indices[ℓ].
    // Then compute W_BB = W_RF^H · W_NN (num_layers × num_layers).
    // Final = W_RF · W_BB (num_tx × num_layers).
    out.bb_weights.assign(2 * num_layers * num_layers, 0.0f);
    for (uint32_t ell = 0; ell < num_layers; ++ell)
    {
        const auto& cb_ell = codebook.GetEntry(out.codebook_indices[ell]);
        for (uint32_t lambda = 0; lambda < num_layers; ++lambda)
        {
            GetColumn(nn_re_im,
                       num_tx,
                       num_layers,
                       lambda,
                       col);
            float re = 0.0f;
            float im = 0.0f;
            InnerProduct(cb_ell, col, num_tx, re, im);
            const uint32_t bb_off = 2 * (ell * num_layers + lambda);
            out.bb_weights[bb_off] = re;
            out.bb_weights[bb_off + 1] = im;
        }
    }

    out.final_weights.assign(2 * num_tx * num_layers, 0.0f);
    // (W_RF · W_BB)[tx, lambda]
    //   = Σ_ell  W_RF[tx, ell]  ·  W_BB[ell, lambda]
    for (uint32_t tx = 0; tx < num_tx; ++tx)
    {
        for (uint32_t lambda = 0; lambda < num_layers; ++lambda)
        {
            double acc_re = 0.0;
            double acc_im = 0.0;
            for (uint32_t ell = 0; ell < num_layers; ++ell)
            {
                const auto& cb_ell =
                    codebook.GetEntry(out.codebook_indices[ell]);
                const double rf_re = cb_ell[2 * tx];
                const double rf_im = cb_ell[2 * tx + 1];
                const uint32_t bb_off = 2 * (ell * num_layers + lambda);
                const double bb_re = out.bb_weights[bb_off];
                const double bb_im = out.bb_weights[bb_off + 1];
                acc_re += rf_re * bb_re - rf_im * bb_im;
                acc_im += rf_re * bb_im + rf_im * bb_re;
            }
            const uint32_t off = 2 * (tx * num_layers + lambda);
            out.final_weights[off] = static_cast<float>(acc_re);
            out.final_weights[off + 1] = static_cast<float>(acc_im);
        }
    }
    return out;
}

// ----------------------------------------------------------------------------
// OranNtnMmimoPrecoderXapp
// ----------------------------------------------------------------------------

TypeId
OranNtnMmimoPrecoderXapp::GetTypeId()
{
    static TypeId tid = TypeId("ns3::OranNtnMmimoPrecoderXapp")
                            .SetParent<Object>()
                            .SetGroupName("OranNtn")
                            .AddConstructor<OranNtnMmimoPrecoderXapp>();
    return tid;
}

OranNtnMmimoPrecoderXapp::OranNtnMmimoPrecoderXapp() = default;
OranNtnMmimoPrecoderXapp::~OranNtnMmimoPrecoderXapp() = default;

void
OranNtnMmimoPrecoderXapp::DoDispose()
{
    m_client.reset();
    m_codebook = nullptr;
    m_ru = nullptr;
    m_observer = nullptr;
    Object::DoDispose();
}

void
OranNtnMmimoPrecoderXapp::Configure(uint32_t xapp_id,
                                      uint32_t num_tx,
                                      uint32_t num_layers)
{
    m_xappId = xapp_id;
    m_numTx = num_tx;
    m_numLayers = num_layers;
}

void
OranNtnMmimoPrecoderXapp::AttachInferenceClient(
    std::shared_ptr<oranntn::airan::AiranInferenceClient> client,
    const std::string& model_name)
{
    m_client = std::move(client);
    m_modelName = model_name;
}

void
OranNtnMmimoPrecoderXapp::AttachCodebook(
    Ptr<OranNtnMmimoCodebook> codebook)
{
    m_codebook = codebook;
}

bool
OranNtnMmimoPrecoderXapp::OnCsiReport(uint64_t ue_id,
                                        uint64_t nr_cgi,
                                        const oranntn::airan::CsiTensor& csi)
{
    if (!m_client || !m_codebook || !m_ru ||
        m_numTx == 0 || m_numLayers == 0)
    {
        ++m_errors;
        return false;
    }
    if (m_codebook->NumTx() != m_numTx)
    {
        ++m_errors;
        return false;
    }
    ++m_csiInputs;
    const double now_s = Simulator::Now().GetSeconds();
    const uint64_t rid = m_client->SubmitPrecoderRequest(
        m_modelName,
        ue_id,
        nr_cgi,
        now_s,
        csi,
        [this, ue_id, nr_cgi](
            const oranntn::airan::InferenceResponse& resp) {
            HandleInferenceResponse(resp, ue_id, nr_cgi);
        });
    if (rid == 0)
    {
        ++m_errors;
        return false;
    }
    return true;
}

void
OranNtnMmimoPrecoderXapp::HandleInferenceResponse(
    const oranntn::airan::InferenceResponse& resp,
    uint64_t ue_id,
    uint64_t nr_cgi)
{
    ++m_responses;
    if (resp.status != 0 ||
        resp.output_kind !=
            oranntn::airan::InferenceResponse::OutputKind::precoder)
    {
        ++m_errors;
        return;
    }
    const auto& p = resp.precoder;
    if (p.num_tx != m_numTx || p.num_layers != m_numLayers ||
        p.values.size() !=
            static_cast<size_t>(2 * m_numTx * m_numLayers))
    {
        ++m_errors;
        return;
    }
    // The codec emits values in row-major (tx, layer, re_im).
    m_lastResult = OranNtnMmimoTwoStageComposer::Compose(
        p.values, m_numTx, m_numLayers, *m_codebook);
    EmitControlAction(ue_id, nr_cgi, m_lastResult);
}

void
OranNtnMmimoPrecoderXapp::EmitControlAction(
    uint64_t ue_id,
    uint64_t nr_cgi,
    const TwoStagePrecoderResult& result)
{
    if (!m_ru || result.final_weights.empty())
    {
        return;
    }
    E2RcAction act;
    act.timestamp = Simulator::Now().GetSeconds();
    act.xappId = m_xappId;
    act.xappName = "mmimo-precoder-xapp";
    act.actionType = E2RcActionType::BEAM_HOP_SCHEDULE;
    act.targetGnbId = static_cast<uint32_t>(m_ru->GetGnbId());
    act.targetUeId = static_cast<uint32_t>(ue_id & 0xFFFFFFFF);
    act.targetBeamId =
        result.codebook_indices.empty()
            ? 0
            : result.codebook_indices.front();
    // AI-06: carry the weights, not just the codebook index. Without this the
    // composed hybrid precoder was discarded one call before it could matter.
    act.beamformingWeights = result.final_weights;
    if (m_ru->ReceiveControl(act))
    {
        ++m_actions;
        if (m_observer)
        {
            m_observer(act, result);
        }
    }
}

} // namespace ns3
