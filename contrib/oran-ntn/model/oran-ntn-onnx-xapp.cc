/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only
#include "oran-ntn-onnx-xapp.h"

#include <ns3/log.h>

#ifdef HAVE_ONNXRUNTIME
#include <onnxruntime_cxx_api.h>
#endif

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("OranNtnOnnxXapp");
NS_OBJECT_ENSURE_REGISTERED(OranNtnOnnxXapp);

struct OranNtnOnnxXapp::OrtState
{
#ifdef HAVE_ONNXRUNTIME
    Ort::Env env{ORT_LOGGING_LEVEL_WARNING, "oran-ntn-onnx-xapp"};
    std::unique_ptr<Ort::Session> session;
    std::string inputName;
    std::string outputName;
    int64_t inputDim{0};
#endif
};

TypeId
OranNtnOnnxXapp::GetTypeId()
{
    static TypeId tid = TypeId("ns3::OranNtnOnnxXapp")
                            .SetParent<Object>()
                            .SetGroupName("OranNtn")
                            .AddConstructor<OranNtnOnnxXapp>();
    return tid;
}

OranNtnOnnxXapp::OranNtnOnnxXapp()
    : m_ort(std::make_unique<OrtState>())
{
}

OranNtnOnnxXapp::~OranNtnOnnxXapp() = default;

bool
OranNtnOnnxXapp::IsOnnxAvailable()
{
#ifdef HAVE_ONNXRUNTIME
    return true;
#else
    return false;
#endif
}

bool
OranNtnOnnxXapp::LoadModel(const std::string& path)
{
#ifdef HAVE_ONNXRUNTIME
    try
    {
        Ort::SessionOptions opts;
        opts.SetIntraOpNumThreads(1);
        m_ort->session = std::make_unique<Ort::Session>(m_ort->env, path.c_str(), opts);
        Ort::AllocatorWithDefaultOptions alloc;
        m_ort->inputName = m_ort->session->GetInputNameAllocated(0, alloc).get();
        m_ort->outputName = m_ort->session->GetOutputNameAllocated(0, alloc).get();
        auto shape =
            m_ort->session->GetInputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
        m_ort->inputDim = shape.empty() ? 0 : shape.back();
        m_modelLoaded = true;
        NS_LOG_INFO("ONNX model loaded: " << path << " (input dim " << m_ort->inputDim
                                          << ")");
        return true;
    }
    catch (const std::exception& e)
    {
        NS_LOG_WARN("ONNX model load failed (" << e.what() << "); using heuristic");
        m_modelLoaded = false;
        return false;
    }
#else
    NS_LOG_INFO("ONNX Runtime not built in; " << path
                                              << " ignored, using registered heuristic");
    return false;
#endif
}

std::vector<double>
OranNtnOnnxXapp::Infer(const std::vector<double>& features)
{
    ++m_inferences;
#ifdef HAVE_ONNXRUNTIME
    if (m_modelLoaded)
    {
        std::vector<float> in(features.begin(), features.end());
        const std::vector<int64_t> shape{1, static_cast<int64_t>(in.size())};
        Ort::MemoryInfo mem = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
        Ort::Value input = Ort::Value::CreateTensor<float>(mem, in.data(), in.size(),
                                                           shape.data(), shape.size());
        const char* inNames[] = {m_ort->inputName.c_str()};
        const char* outNames[] = {m_ort->outputName.c_str()};
        auto out = m_ort->session->Run(Ort::RunOptions{nullptr}, inNames, &input, 1,
                                       outNames, 1);
        const float* data = out[0].GetTensorData<float>();
        const size_t n = out[0].GetTensorTypeAndShapeInfo().GetElementCount();
        return std::vector<double>(data, data + n);
    }
#endif
    if (m_heuristic)
    {
        return m_heuristic(features);
    }
    return {};
}

} // namespace ns3
