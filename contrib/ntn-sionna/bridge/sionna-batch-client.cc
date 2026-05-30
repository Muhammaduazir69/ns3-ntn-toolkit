/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#include "sionna-batch-client.h"

#include "ns3/log.h"
#include "ns3/simulator.h"

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("SionnaBatchClient");

TypeId
SionnaBatchClient::GetTypeId()
{
    static TypeId tid = TypeId("ns3::SionnaBatchClient")
                            .SetParent<Object>()
                            .SetGroupName("NtnSionna")
                            .AddConstructor<SionnaBatchClient>();
    return tid;
}

SionnaBatchClient::SionnaBatchClient() = default;

SionnaBatchClient::~SionnaBatchClient() = default;

void
SionnaBatchClient::DoDispose()
{
    for (auto& kv : m_batches)
    {
        for (auto& ev : kv.second.events)
        {
            Simulator::Cancel(ev);
        }
    }
    m_batches.clear();
    m_transport = nullptr;
    Object::DoDispose();
}

void
SionnaBatchClient::Attach(Ptr<SionnaTransport> transport)
{
    m_transport = transport;
}

uint64_t
SionnaBatchClient::RequestBatch(const std::vector<Request>& reqs,
                                  BatchCallback cb)
{
    if (!m_transport || reqs.empty())
    {
        return 0;
    }
    const uint64_t handle = m_nextHandle++;
    BatchState& st = m_batches[handle];
    st.responses.assign(reqs.size(), Response{});
    st.filled.assign(reqs.size(), false);
    st.remaining = reqs.size();
    st.cb = std::move(cb);
    ++m_submitted;
    for (size_t i = 0; i < reqs.size(); ++i)
    {
        const Time when = m_stagger * static_cast<int64_t>(i);
        auto ev = Simulator::Schedule(when,
                                        &SionnaBatchClient::DispatchOne,
                                        this,
                                        handle,
                                        i,
                                        reqs[i]);
        st.events.push_back(ev);
    }
    return handle;
}

std::vector<SionnaBatchClient::Response>
SionnaBatchClient::RequestBatchSync(const std::vector<Request>& reqs)
{
    std::vector<Response> out;
    if (!m_transport)
    {
        return out;
    }
    out.reserve(reqs.size());
    ++m_submitted;
    for (const auto& r : reqs)
    {
        ++m_totalQueries;
        out.push_back(m_transport->Query(r));
    }
    ++m_completed;
    return out;
}

bool
SionnaBatchClient::CancelBatch(uint64_t handle)
{
    auto it = m_batches.find(handle);
    if (it == m_batches.end() || it->second.cancelled)
    {
        return false;
    }
    for (auto& ev : it->second.events)
    {
        Simulator::Cancel(ev);
    }
    it->second.cancelled = true;
    ++m_cancelled;
    m_batches.erase(it);
    return true;
}

void
SionnaBatchClient::DispatchOne(uint64_t handle,
                                size_t idx,
                                Request req)
{
    auto it = m_batches.find(handle);
    if (it == m_batches.end() || it->second.cancelled)
    {
        return;
    }
    BatchState& st = it->second;
    if (idx >= st.responses.size())
    {
        return;
    }
    ++m_totalQueries;
    st.responses[idx] = m_transport->Query(req);
    st.filled[idx] = true;
    --st.remaining;
    if (st.remaining == 0)
    {
        BatchCallback cb = std::move(st.cb);
        std::vector<Response> responses = std::move(st.responses);
        ++m_completed;
        m_batches.erase(it);
        if (cb)
        {
            cb(responses);
        }
    }
}

} // namespace ns3
