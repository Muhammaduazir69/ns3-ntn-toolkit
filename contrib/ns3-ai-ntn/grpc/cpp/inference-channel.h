/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#ifndef NS3_AI_NTN_INFERENCE_CHANNEL_H
#define NS3_AI_NTN_INFERENCE_CHANNEL_H

// Bidirectional byte-pipe abstraction for the AI-RAN inference path
// (Roadmap §3 T7).
//
// The high-level AiranInferenceClient / AiranInferenceServer pair
// speaks length-prefixed serialised protobuf. They do not care which
// concrete transport carries the bytes — that's an InferenceChannel.
// Concrete transports:
//
//   * InProcInferenceChannel  — lock-protected FIFO pair, used for the
//                               unit tests and any in-simulator setup
//                               where the inference engine runs in the
//                               same process.
//   * TcpInferenceChannel     — length-prefixed framing over a raw
//                               TCP socket, using this repo's own
//                               encoder (airan-messages.*). It is NOT
//                               gRPC and not protobuf on the wire.
//
// AI-06: this header used to advertise a third transport,
// `GrpcInferenceChannel`, described as a "real grpc++ stub, built only when
// ENABLE_GRPC_INFERENCE is set at CMake configure time". No such class exists.
// The name appeared in exactly two comments, this one and one in
// inference-channel-tcp.h; there is no declaration, no .cc, and
// ENABLE_GRPC_INFERENCE appears nowhere in any CMakeLists or source. The
// bundled grpc/proto/airan_inference.proto is not compiled either - the only
// protobuf_generate blocks in CMakeLists.txt target
// model/gym-interface/messages.proto.
//
// So: there is no gRPC in this module. The claim is removed rather than left
// standing as a build flag a reader could go looking for. If a gRPC transport
// is added later, it belongs here alongside a compiled .proto and a flag that
// actually exists.

#include <atomic>
#include <cstdint>
#include <string>
#include <vector>

namespace ns3
{
namespace oranntn
{
namespace airan
{

/// Abstract bidirectional byte pipe.
///
/// Implementations must be safe for one writer + one reader in the
/// same thread — none of the toolkit transports use a background
/// thread, so the user pumps the channel from ns-3 simulator
/// callbacks.
class InferenceChannel
{
  public:
    virtual ~InferenceChannel() = default;

    /// Send a complete logical frame. `bytes` is owned by the caller
    /// and may be reused after Send() returns.
    virtual bool Send(const std::vector<uint8_t>& bytes) = 0;

    /// Try to pop the next inbound frame, waiting up to `timeout_ms`
    /// for data to arrive. Returns false on timeout, eof, or error.
    virtual bool TryRecv(std::vector<uint8_t>& bytes,
                         uint32_t timeout_ms) = 0;

    /// `true` while the channel is usable in either direction.
    virtual bool IsOpen() const = 0;

    /// Close the channel; idempotent.
    virtual void Close() = 0;

    /// Human-readable transport kind ("inproc", "tcp", "grpc"); used
    /// in logging.
    virtual std::string Kind() const = 0;

    /// Bytes sent / received so far (best-effort counters for tests).
    uint64_t BytesSent() const { return m_bytesSent.load(); }
    uint64_t BytesRecv() const { return m_bytesRecv.load(); }
    uint64_t FramesSent() const { return m_framesSent.load(); }
    uint64_t FramesRecv() const { return m_framesRecv.load(); }

  protected:
    std::atomic<uint64_t> m_bytesSent{0};
    std::atomic<uint64_t> m_bytesRecv{0};
    std::atomic<uint64_t> m_framesSent{0};
    std::atomic<uint64_t> m_framesRecv{0};
};

} // namespace airan
} // namespace oranntn
} // namespace ns3

#endif // NS3_AI_NTN_INFERENCE_CHANNEL_H
