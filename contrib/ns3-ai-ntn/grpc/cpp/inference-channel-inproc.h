/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#ifndef NS3_AI_NTN_INFERENCE_CHANNEL_INPROC_H
#define NS3_AI_NTN_INFERENCE_CHANNEL_INPROC_H

// In-process bidirectional channel — two endpoints share a pair of
// FIFOs. Created together with CreatePair(); each end behaves like a
// peer over a socket.

#include "inference-channel.h"

#include <condition_variable>
#include <deque>
#include <memory>
#include <mutex>

namespace ns3
{
namespace oranntn
{
namespace airan
{

class InProcInferenceChannel : public InferenceChannel
{
  public:
    /// Shared queue + lock owned by both endpoints. One endpoint reads
    /// from `a_to_b`, the other from `b_to_a`. They swap on creation.
    struct Pipe
    {
        std::mutex mu;
        std::condition_variable cv;
        std::deque<std::vector<uint8_t>> queue;
        bool closed{false};
    };

    /// Build a connected pair: returns (clientEnd, serverEnd).
    static std::pair<std::unique_ptr<InProcInferenceChannel>,
                     std::unique_ptr<InProcInferenceChannel>>
        CreatePair();

    bool Send(const std::vector<uint8_t>& bytes) override;
    bool TryRecv(std::vector<uint8_t>& bytes,
                 uint32_t timeout_ms) override;
    bool IsOpen() const override;
    void Close() override;
    std::string Kind() const override { return "inproc"; }

  private:
    InProcInferenceChannel(std::shared_ptr<Pipe> outbound,
                           std::shared_ptr<Pipe> inbound);

    std::shared_ptr<Pipe> m_out;
    std::shared_ptr<Pipe> m_in;
};

} // namespace airan
} // namespace oranntn
} // namespace ns3

#endif // NS3_AI_NTN_INFERENCE_CHANNEL_INPROC_H
