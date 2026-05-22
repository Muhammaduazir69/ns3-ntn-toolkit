/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#ifndef NS3_AI_NTN_TRITON_MODEL_CONFIG_H
#define NS3_AI_NTN_TRITON_MODEL_CONFIG_H

// Lightweight `config.pbtxt` reader for Triton model configurations.
//
// The toolkit does not embed the full Triton schema. It parses only
// the fields it actually needs to dispatch inference requests in the
// AiranInferenceServer:
//
//   name : string
//   platform : string
//   max_batch_size : integer
//   default_model_filename : string
//   dynamic_batching.max_queue_delay_microseconds : integer
//   parameters { key: "INFERENCE_BUDGET_US" value { string_value: ... } }
//   parameters { key: "TOOLKIT_OUTPUT_FIELD" value { string_value: ... } }
//
// Anything else is preserved as an opaque k/v pair in `params` and is
// exposed via Parameter("MY_KEY") for forward-compatibility.
//
// This is deliberately *not* a full protobuf-text parser — Triton
// users feed real config.pbtxt files into the real Triton server. The
// toolkit's parser exists so AiranInferenceServer can be configured
// from the same on-disk config the production Triton instance uses.

#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <vector>

namespace ns3
{
namespace oranntn
{
namespace airan
{

enum class OutputField
{
    unknown,
    precoder,
    beam,
};

struct TritonModelConfig
{
    std::string name;
    std::string platform;
    uint32_t max_batch_size{0};
    std::string default_model_filename;
    std::optional<uint64_t> max_queue_delay_us;
    OutputField output_field{OutputField::unknown};
    std::optional<uint64_t> inference_budget_us;
    /// Any extra TOOLKIT_* / user-defined parameter strings.
    std::map<std::string, std::string> params;

    /// Convenience: look up a parameter by key, returning empty if
    /// absent.
    std::string Parameter(const std::string& key) const
    {
        const auto it = params.find(key);
        return (it == params.end()) ? std::string() : it->second;
    }
};

class TritonModelConfigParser
{
  public:
    /// Parse the pbtxt content (UTF-8 string). Returns std::nullopt on
    /// any structural error.
    static std::optional<TritonModelConfig>
        Parse(const std::string& pbtxt);

    /// Load and parse a file at `path`.
    static std::optional<TritonModelConfig>
        LoadFile(const std::string& path);
};

} // namespace airan
} // namespace oranntn
} // namespace ns3

#endif // NS3_AI_NTN_TRITON_MODEL_CONFIG_H
