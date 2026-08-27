/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#ifndef NS3_AI_NTN_AIRAN_MESSAGES_H
#define NS3_AI_NTN_AIRAN_MESSAGES_H

// Pure-C++ mirror of the `airan_inference.proto` message set.
//
// The native C++ struct layout matches the proto field names 1:1, but
// the on-the-wire encoding used by the toolkit's TcpInferenceChannel
// and InProcInferenceChannel is a simple tagged-binary format rather
// than full Protobuf varint encoding. This keeps the build dependency
// surface minimal — ns-3 already pulls in `protobuf::libprotobuf` for
// ns3-ai's gym interface, but using *generated* C++ stubs would
// require running `protoc` against multiple .proto files in cmake.
//
// AI-06: an earlier note here promised that ENABLE_GRPC_INFERENCE would wire
// this up to protobuf. That flag does not exist in any CMakeLists or source,
// and grpc/proto/airan_inference.proto is not compiled by the build. This is
// the toolkit's own encoder over raw TCP framing, and nothing more.
// gRPC build will switch the codec to the real protoc-generated
// classes; the high-level AiranInferenceClient / Server APIs that
// callers see don't change.
//
// Wire format (big-endian, sequential — no varints needed because the
// channel layer already provides framing):
//
//   u8  msg_kind            (0 = request, 1 = response)
//   u16 field_count
//   <field_count> times:
//      u16 field_id          (proto field tag)
//      u8  wire_type         (1 = u32, 2 = u64, 3 = f32, 4 = f64,
//                             5 = string, 6 = bytes/packed-f32)
//      u32 payload_len
//      <payload_len> bytes

#include <cstdint>
#include <string>
#include <vector>

namespace ns3
{
namespace oranntn
{
namespace airan
{

// Field IDs — must match the `= N;` numbers in airan_inference.proto.
namespace fid
{

// InferenceRequest
constexpr uint16_t kReqModelName = 1;
constexpr uint16_t kReqRequestId = 2;
constexpr uint16_t kReqUeId = 3;
constexpr uint16_t kReqNrCgi = 4;
constexpr uint16_t kReqSimTimeS = 5;
constexpr uint16_t kReqCsi = 6;
constexpr uint16_t kReqRsrp = 7;

// CsiTensor sub-fields (packed as one bytes blob with sub-tags inside)
constexpr uint16_t kCsiValues = 11;
constexpr uint16_t kCsiNumTx = 12;
constexpr uint16_t kCsiNumRx = 13;
constexpr uint16_t kCsiNumSc = 14;
constexpr uint16_t kCsiDoppler = 15;

// RsrpVector sub-fields
constexpr uint16_t kRsrpRsrp = 21;
constexpr uint16_t kRsrpAz = 22;
constexpr uint16_t kRsrpEl = 23;

// InferenceResponse
constexpr uint16_t kRespRequestId = 1;
constexpr uint16_t kRespLatency = 2;
constexpr uint16_t kRespStatus = 3;
constexpr uint16_t kRespStatusMsg = 4;
constexpr uint16_t kRespPrecoder = 5;
constexpr uint16_t kRespBeam = 6;

// PrecoderWeights sub-fields
constexpr uint16_t kPrecoderValues = 31;
constexpr uint16_t kPrecoderNumTx = 32;
constexpr uint16_t kPrecoderNumLayers = 33;

// BeamPrediction sub-fields
constexpr uint16_t kBeamBestId = 41;
constexpr uint16_t kBeamScore = 42;
constexpr uint16_t kBeamTopK = 43;

} // namespace fid

/// Channel state, as handed to an inference model.
///
/// AI-06: the element ORDER of `values` was never written down anywhere, which
/// is a real gap and not a documentation nicety - a model cannot be fed a
/// tensor whose layout is undefined, and the mock runtime sidestepped the
/// question entirely by ignoring the data and deriving its output from the
/// request counter. The layout is now part of the contract:
///
///   values is interleaved real/imaginary, indexed
///     offset = 2 * ((sc * num_rx + rx) * num_tx + tx)
///   i.e. subcarrier-major, then receive antenna, then transmit antenna, with
///   H[sc][rx][tx] = values[offset] + j * values[offset + 1].
///
/// Size must therefore be 2 * num_subcarriers * num_rx * num_tx. A consumer
/// that disagrees about this ordering produces a wrong beam, which is exactly
/// the failure a layout contract exists to make visible.
struct CsiTensor
{
    std::vector<float> values;
    uint32_t num_tx{0};
    uint32_t num_rx{0};
    uint32_t num_subcarriers{0};
    double doppler_hz{0.0};

    /// Elements `values` must hold for the declared dimensions.
    std::size_t ExpectedSize() const
    {
        return static_cast<std::size_t>(2) * num_subcarriers * num_rx * num_tx;
    }
    /// Whether `values` matches the declared dimensions.
    bool LayoutValid() const
    {
        return num_tx > 0 && num_rx > 0 && num_subcarriers > 0 &&
               values.size() == ExpectedSize();
    }
    /// Offset of H[sc][rx][tx] within `values`. No bounds check: callers hold
    /// the loop bounds.
    std::size_t Index(uint32_t sc, uint32_t rx, uint32_t tx) const
    {
        return static_cast<std::size_t>(2) *
               ((static_cast<std::size_t>(sc) * num_rx + rx) * num_tx + tx);
    }
};

struct RsrpVector
{
    std::vector<float> rsrp_dbm;
    std::vector<float> beam_az_deg;
    std::vector<float> beam_el_deg;
};

struct InferenceRequest
{
    std::string model_name;
    uint64_t request_id{0};
    uint64_t ue_id{0};
    uint64_t nr_cgi{0};
    double sim_time_s{0.0};

    enum class InputKind : uint8_t
    {
        none = 0,
        csi = 1,
        rsrp = 2,
    };
    InputKind input_kind{InputKind::none};
    CsiTensor csi;
    RsrpVector rsrp;
};

struct PrecoderWeights
{
    std::vector<float> values; // interleaved (re, im)
    uint32_t num_tx{0};
    uint32_t num_layers{0};
};

struct BeamPrediction
{
    uint32_t best_beam_id{0};
    float score{0.0f};
    std::vector<float> scores_top_k;
};

struct InferenceResponse
{
    uint64_t request_id{0};
    double inference_latency_ms{0.0};
    uint32_t status{0};
    std::string status_message;

    enum class OutputKind : uint8_t
    {
        none = 0,
        precoder = 1,
        beam = 2,
    };
    OutputKind output_kind{OutputKind::none};
    PrecoderWeights precoder;
    BeamPrediction beam;
};

class AiranMessageCodec
{
  public:
    static std::vector<uint8_t> EncodeRequest(const InferenceRequest& r);
    static std::vector<uint8_t>
        EncodeResponse(const InferenceResponse& r);

    static bool DecodeRequest(const std::vector<uint8_t>& bytes,
                              InferenceRequest& out);
    static bool DecodeResponse(const std::vector<uint8_t>& bytes,
                                InferenceResponse& out);

    /// Peek at the leading u8 kind byte. Returns 0xFF if the frame is
    /// empty.
    static uint8_t PeekKind(const std::vector<uint8_t>& bytes);
};

} // namespace airan
} // namespace oranntn
} // namespace ns3

#endif // NS3_AI_NTN_AIRAN_MESSAGES_H
