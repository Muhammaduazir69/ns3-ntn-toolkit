/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#ifndef NS3_AI_NTN_INFERENCE_CHANNEL_TCP_H
#define NS3_AI_NTN_INFERENCE_CHANNEL_TCP_H

// TCP-backed inference channel — length-prefixed binary frames.
//
// Frame format (big-endian for portability):
//
//   [ uint32 len_be ][ len_be bytes payload ]
//
// Payload is the serialised AiranInference protobuf. This matches what
// `grpc++` puts on the wire for unary RPCs once the gRPC framing layer
// is stripped, so flipping `Channel = GrpcInferenceChannel` later is a
// drop-in once the server speaks gRPC.

#include "inference-channel.h"

#include <atomic>
#include <memory>
#include <vector>

namespace ns3
{
namespace oranntn
{
namespace airan
{

class TcpInferenceChannel : public InferenceChannel
{
  public:
    TcpInferenceChannel();
    explicit TcpInferenceChannel(int fd); // adopt an open fd
    ~TcpInferenceChannel() override;

    /// Open as a client: connect to host:port.
    bool ConnectTo(const std::string& host, uint16_t port);

    bool Send(const std::vector<uint8_t>& bytes) override;
    bool TryRecv(std::vector<uint8_t>& bytes,
                 uint32_t timeout_ms) override;
    bool IsOpen() const override;
    void Close() override;
    std::string Kind() const override { return "tcp"; }

    int Fd() const { return m_fd; }

  private:
    bool RecvExact(uint8_t* out, size_t n, uint32_t timeout_ms);

    int m_fd{-1};
    std::vector<uint8_t> m_rxStash; // partial RX state between calls
};

/// Accept-only listener — yields TcpInferenceChannel instances per
/// inbound connection. The owning AiranInferenceServer holds the
/// resulting channels and pumps them.
class TcpInferenceListener
{
  public:
    TcpInferenceListener();
    ~TcpInferenceListener();

    /// Bind + listen on host:port (host "" or "0.0.0.0" = wildcard).
    bool Listen(const std::string& host,
                uint16_t port,
                int backlog = 8);

    /// Try to accept one connection. Returns nullptr on timeout.
    std::unique_ptr<TcpInferenceChannel> AcceptOne(uint32_t timeout_ms);

    /// Resolved listening port (useful for `port=0` ephemeral binds).
    uint16_t LocalPort() const { return m_port; }

    bool IsOpen() const { return m_fd >= 0; }
    void Close();
    int Fd() const { return m_fd; }

  private:
    int m_fd{-1};
    uint16_t m_port{0};
};

} // namespace airan
} // namespace oranntn
} // namespace ns3

#endif // NS3_AI_NTN_INFERENCE_CHANNEL_TCP_H
