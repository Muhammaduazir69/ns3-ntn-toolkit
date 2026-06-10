/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only
//
// OranNtnOnnxXapp — AI-native inference xApp (Deng 2026 Sec. V; adoption plan
// WS3). Closes the train -> deploy -> infer lifecycle:
//   1. offline training on toolkit gym environments (ns3-ai-ntn),
//   2. export to .onnx,
//   3. THIS xApp loads the model with ONNX Runtime (MIT-licensed; web-checked
//      2026-06-10) and infers on the LIVE measured feature vectors from
//      NtnOranAiFlowMonitor::GetFeatures().
//
// ONNX Runtime is an OPTIONAL dependency, auto-detected at configure time
// (mirrors ns3-ai-ntn's libtensorflow pattern). Without it the xApp falls
// back to a caller-registered heuristic policy, so every example still runs
// — the inference path upgrades transparently when onnxruntime-dev is
// installed. IsOnnxAvailable()/IsModelLoaded() report which path is live.

#ifndef ORAN_NTN_ONNX_XAPP_H
#define ORAN_NTN_ONNX_XAPP_H

#include <ns3/object.h>

#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace ns3
{

class OranNtnOnnxXapp : public Object
{
  public:
    /// Fallback policy: features in, action vector out.
    using HeuristicPolicy =
        std::function<std::vector<double>(const std::vector<double>&)>;

    static TypeId GetTypeId();
    OranNtnOnnxXapp();
    ~OranNtnOnnxXapp() override;

    /// True when the toolkit was built against ONNX Runtime.
    static bool IsOnnxAvailable();
    /**
     * \brief Load a .onnx model (single float input tensor [1,N], single
     *        float output tensor). Returns false (and keeps the heuristic)
     *        when ONNX Runtime is absent or the file fails to load.
     */
    bool LoadModel(const std::string& path);
    bool IsModelLoaded() const { return m_modelLoaded; }
    void RegisterHeuristic(HeuristicPolicy policy) { m_heuristic = std::move(policy); }

    /// Run inference: ONNX session when a model is loaded, else heuristic.
    std::vector<double> Infer(const std::vector<double>& features);

    uint64_t GetInferenceCount() const { return m_inferences; }

  private:
    struct OrtState; // hides onnxruntime types from this header
    std::unique_ptr<OrtState> m_ort;
    bool m_modelLoaded{false};
    HeuristicPolicy m_heuristic;
    uint64_t m_inferences{0};
};

} // namespace ns3

#endif // ORAN_NTN_ONNX_XAPP_H
