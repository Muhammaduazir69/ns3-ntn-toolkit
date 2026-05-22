/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#include "inference-channel-inproc.h"

#include <chrono>

namespace ns3
{
namespace oranntn
{
namespace airan
{

InProcInferenceChannel::InProcInferenceChannel(
    std::shared_ptr<Pipe> outbound,
    std::shared_ptr<Pipe> inbound)
    : m_out(std::move(outbound)),
      m_in(std::move(inbound))
{
}

std::pair<std::unique_ptr<InProcInferenceChannel>,
          std::unique_ptr<InProcInferenceChannel>>
InProcInferenceChannel::CreatePair()
{
    auto a_to_b = std::make_shared<Pipe>();
    auto b_to_a = std::make_shared<Pipe>();
    auto clientEnd = std::unique_ptr<InProcInferenceChannel>(
        new InProcInferenceChannel(a_to_b, b_to_a));
    auto serverEnd = std::unique_ptr<InProcInferenceChannel>(
        new InProcInferenceChannel(b_to_a, a_to_b));
    return {std::move(clientEnd), std::move(serverEnd)};
}

bool
InProcInferenceChannel::Send(const std::vector<uint8_t>& bytes)
{
    if (!m_out)
    {
        return false;
    }
    std::lock_guard<std::mutex> lock(m_out->mu);
    if (m_out->closed)
    {
        return false;
    }
    m_out->queue.push_back(bytes);
    m_bytesSent.fetch_add(bytes.size());
    m_framesSent.fetch_add(1);
    m_out->cv.notify_one();
    return true;
}

bool
InProcInferenceChannel::TryRecv(std::vector<uint8_t>& bytes,
                                 uint32_t timeout_ms)
{
    if (!m_in)
    {
        return false;
    }
    std::unique_lock<std::mutex> lock(m_in->mu);
    if (m_in->queue.empty() && !m_in->closed && timeout_ms > 0)
    {
        m_in->cv.wait_for(lock,
                          std::chrono::milliseconds(timeout_ms),
                          [&] {
                              return !m_in->queue.empty() ||
                                     m_in->closed;
                          });
    }
    if (m_in->queue.empty())
    {
        return false;
    }
    bytes = std::move(m_in->queue.front());
    m_in->queue.pop_front();
    m_bytesRecv.fetch_add(bytes.size());
    m_framesRecv.fetch_add(1);
    return true;
}

bool
InProcInferenceChannel::IsOpen() const
{
    if (!m_in || !m_out)
    {
        return false;
    }
    std::lock_guard<std::mutex> a(m_in->mu);
    return !m_in->closed && !m_out->closed;
}

void
InProcInferenceChannel::Close()
{
    if (m_out)
    {
        std::lock_guard<std::mutex> lock(m_out->mu);
        m_out->closed = true;
        m_out->cv.notify_all();
    }
    if (m_in)
    {
        std::lock_guard<std::mutex> lock(m_in->mu);
        m_in->closed = true;
        m_in->cv.notify_all();
    }
}

} // namespace airan
} // namespace oranntn
} // namespace ns3
