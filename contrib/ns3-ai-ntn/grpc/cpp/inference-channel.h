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
//                               TCP socket. Mirrors the wire format a
//                               gRPC server would expose if you point
//                               `grpc++` at the same .proto.
//   * GrpcInferenceChannel    — real grpc++ stub, built only when
//                               ENABLE_GRPC_INFERENCE is set at CMake
//                               configure time. The header always
//                               exists so callers don't break; the
//                               .cc implementation is gated.

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
