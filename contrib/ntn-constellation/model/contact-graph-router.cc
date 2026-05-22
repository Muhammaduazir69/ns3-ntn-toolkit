/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#include "contact-graph-router.h"

#include "ns3/log.h"

#include <algorithm>
#include <queue>

namespace ns3
{
namespace ntncon
{

NS_LOG_COMPONENT_DEFINE("ContactGraphRouter");
NS_OBJECT_ENSURE_REGISTERED(ContactGraphRouter);

TypeId
ContactGraphRouter::GetTypeId()
{
    static TypeId tid = TypeId("ns3::ntncon::ContactGraphRouter")
                            .SetParent<Object>()
                            .SetGroupName("NtnConstellation")
                            .AddConstructor<ContactGraphRouter>();
    return tid;
}

ContactGraphRouter::ContactGraphRouter() = default;

void
ContactGraphRouter::Attach(Ptr<ContactGraphScheduler> scheduler)
{
    if (scheduler == nullptr)
    {
        return;
    }
    scheduler->m_contactUp.ConnectWithoutContext(
        MakeCallback(&ContactGraphRouter::OnContactUp, this));
    scheduler->m_contactDown.ConnectWithoutContext(
        MakeCallback(&ContactGraphRouter::OnContactDown, this));
}

void
ContactGraphRouter::HandleContactEvent(const ContactEvent& ev)
{
    const auto key = CanonicalEdge(ev.node_a, ev.node_b);
    if (ev.up)
    {
        const auto inserted = m_edges.insert(key).second;
        if (inserted)
        {
            m_adj[ev.node_a].insert(ev.node_b);
            m_adj[ev.node_b].insert(ev.node_a);
            ++m_added;
        }
    }
    else
    {
        const auto removed = m_edges.erase(key);
        if (removed > 0)
        {
            m_adj[ev.node_a].erase(ev.node_b);
            m_adj[ev.node_b].erase(ev.node_a);
            ++m_removed;
        }
    }
}

bool
ContactGraphRouter::HasEdge(uint32_t a, uint32_t b) const
{
    return m_edges.count(CanonicalEdge(a, b)) != 0;
}

size_t
ContactGraphRouter::NumEdges() const
{
    return m_edges.size();
}

std::set<uint32_t>
ContactGraphRouter::Neighbours(uint32_t node) const
{
    auto it = m_adj.find(node);
    return (it == m_adj.end()) ? std::set<uint32_t>{} : it->second;
}

std::vector<uint32_t>
ContactGraphRouter::ShortestPath(uint32_t src, uint32_t dst) const
{
    ++m_queries;
    if (src == dst)
    {
        return {src};
    }
    // BFS over the adjacency map. Predecessors track path reconstruction.
    std::map<uint32_t, uint32_t> pred;
    std::set<uint32_t> visited;
    std::queue<uint32_t> q;
    visited.insert(src);
    q.push(src);
    while (!q.empty())
    {
        const uint32_t cur = q.front();
        q.pop();
        auto it = m_adj.find(cur);
        if (it == m_adj.end())
        {
            continue;
        }
        for (uint32_t nb : it->second)
        {
            if (visited.insert(nb).second)
            {
                pred[nb] = cur;
                if (nb == dst)
                {
                    std::vector<uint32_t> path{dst};
                    uint32_t c = dst;
                    while (c != src)
                    {
                        c = pred.at(c);
                        path.push_back(c);
                    }
                    std::reverse(path.begin(), path.end());
                    return path;
                }
                q.push(nb);
            }
        }
    }
    return {};
}

} // namespace ntncon
} // namespace ns3
