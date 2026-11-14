/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#include "contact-graph-router.h"

#include <cmath>
#include <limits>

#include <set>

#include <deque>

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
    // SAGIN-1: track the geometry of an established contact, not just its
    // existence. Without this the Dijkstra weight was the range sampled at
    // contact-up and stayed there for the whole pass, so the shortest path was
    // chosen on stale distances and any latency derived from total_weight was a
    // frozen number that happened to look stable.
    scheduler->m_contactUpdate.ConnectWithoutContext(
        MakeCallback(&ContactGraphRouter::OnContactUpdate, this));
}

void
ContactGraphRouter::FlatAdd(uint32_t a, uint32_t b, double w)
{
    if (m_flat.size() <= a)
    {
        m_flat.resize(a + 1);
    }
    auto& v = m_flat[a];
    auto it = std::lower_bound(v.begin(), v.end(), std::make_pair(b, -1e300));
    if (it != v.end() && it->first == b)
    {
        it->second = w;
    }
    else
    {
        v.insert(it, {b, w});
    }
}

void
ContactGraphRouter::FlatErase(uint32_t a, uint32_t b)
{
    if (a >= m_flat.size())
    {
        return;
    }
    auto& v = m_flat[a];
    v.erase(std::remove_if(v.begin(), v.end(), [b](const auto& x) { return x.first == b; }), v.end());
}

void
ContactGraphRouter::FlatSet(uint32_t a, uint32_t b, double w)
{
    if (a >= m_flat.size())
    {
        return;
    }
    for (auto& x : m_flat[a])
    {
        if (x.first == b)
        {
            x.second = w;
            return;
        }
    }
}

void
ContactGraphRouter::UpdateEdgeWeight(const ContactEvent& ev)
{
    // Refresh the weight of an edge that is already in the graph. Deliberately
    // does NOT insert: an update for an edge we never saw come up would mean
    // the transition traces and this one disagree, and silently inventing the
    // edge would hide that.
    FlatSet(ev.node_a, ev.node_b, ev.range_m);
    FlatSet(ev.node_b, ev.node_a, ev.range_m);
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
        // Seed the weight from the contact-up range; OnContactUpdate keeps
        // it current for the rest of the pass.
        m_edgeWeights[key] = ev.range_m;
        FlatAdd(ev.node_a, ev.node_b, ev.range_m);
        FlatAdd(ev.node_b, ev.node_a, ev.range_m);
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
        m_edgeWeights.erase(key);
        FlatErase(ev.node_a, ev.node_b);
        FlatErase(ev.node_b, ev.node_a);
    }
}

bool
ContactGraphRouter::HasEdge(uint32_t a, uint32_t b) const
{
    return !std::isnan(EdgeWeight(a, b));
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

double
ContactGraphRouter::EdgeWeight(uint32_t a, uint32_t b) const
{
    if (a < m_flat.size())
    {
        for (const auto& x : m_flat[a])
        {
            if (x.first == b)
            {
                return x.second;
            }
        }
    }
    return std::numeric_limits<double>::quiet_NaN();
}

void
ContactGraphRouter::SetRegenMode(uint32_t node, RegenMode mode)
{
    m_regenMode[node] = mode;
}

RegenMode
ContactGraphRouter::GetRegenMode(uint32_t node) const
{
    auto it = m_regenMode.find(node);
    return (it == m_regenMode.end()) ? RegenMode::bent_pipe : it->second;
}

bool
ContactGraphRouter::IsRegenerative(uint32_t node) const
{
    return GetRegenMode(node) != RegenMode::bent_pipe;
}

ContactGraphRouter::WeightedPath
ContactGraphRouter::ShortestPathWeightedRegenOnly(uint32_t src,
                                                    uint32_t dst) const
{
    ++m_queries;
    if (src == dst)
    {
        return {{src}, 0.0};
    }
    // Same Dijkstra as ShortestPathWeighted but skips relaxation through
    // bent-pipe transit nodes. Endpoints are always allowed; only
    // intermediate hops require regen capability.
    std::map<uint32_t, double> dist;
    std::map<uint32_t, uint32_t> pred;
    using QEntry = std::pair<double, uint32_t>;
    std::priority_queue<QEntry, std::vector<QEntry>, std::greater<QEntry>>
        pq;
    dist[src] = 0.0;
    pq.push({0.0, src});
    while (!pq.empty())
    {
        const auto [w, u] = pq.top();
        pq.pop();
        if (u == dst)
        {
            std::vector<uint32_t> path{dst};
            uint32_t c = dst;
            while (c != src)
            {
                c = pred.at(c);
                path.push_back(c);
            }
            std::reverse(path.begin(), path.end());
            return {std::move(path), w};
        }
        if (u != src && u != dst && !IsRegenerative(u))
        {
            continue;
        }
        auto dit = dist.find(u);
        if (dit == dist.end() || w > dit->second)
        {
            continue;
        }
        auto nbIt = m_adj.find(u);
        if (nbIt == m_adj.end())
        {
            continue;
        }
        for (uint32_t v : nbIt->second)
        {
            // Weight from the flat adjacency, which UpdateEdgeWeight keeps current.
            const double we = EdgeWeight(u, v);
            if (std::isnan(we))
            {
                continue;
            }
            const double newW = w + we;
            auto dvIt = dist.find(v);
            if (dvIt == dist.end() || newW < dvIt->second)
            {
                dist[v] = newW;
                pred[v] = u;
                pq.push({newW, v});
            }
        }
    }
    return {{}, std::numeric_limits<double>::infinity()};
}

ContactGraphRouter::WeightedPath
ContactGraphRouter::ShortestPathHops(uint32_t src, uint32_t dst) const
{
    // SAGIN-7: BFS, because ns-3's Ipv4GlobalRouting minimises hop count. This
    // is the route packets take; ShortestPathWeighted is the route the model
    // recommends, and a scenario printing one beside measured goodput needs to
    // know whether they are the same route.
    WeightedPath out;
    out.total_weight = std::numeric_limits<double>::infinity();
    if (src == dst)
    {
        out.path = {src};
        out.total_weight = 0.0;
        return out;
    }

    std::map<uint32_t, uint32_t> prev;
    std::set<uint32_t> seen{src};
    std::deque<uint32_t> q{src};
    bool found = false;
    while (!q.empty() && !found)
    {
        const uint32_t u = q.front();
        q.pop_front();
        for (uint32_t v : Neighbours(u))
        {
            if (seen.count(v))
            {
                continue;
            }
            seen.insert(v);
            prev[v] = u;
            if (v == dst)
            {
                found = true;
                break;
            }
            q.push_back(v);
        }
    }
    if (!found)
    {
        return out;
    }

    std::vector<uint32_t> rev{dst};
    while (rev.back() != src)
    {
        rev.push_back(prev[rev.back()]);
    }
    out.path.assign(rev.rbegin(), rev.rend());

    // Range sum along THIS path, so the two results compare in latency terms.
    double w = 0.0;
    for (size_t i = 1; i < out.path.size(); ++i)
    {
        const double e = EdgeWeight(out.path[i - 1], out.path[i]);
        if (!std::isfinite(e))
        {
            return WeightedPath{{}, std::numeric_limits<double>::infinity()};
        }
        w += e;
    }
    out.total_weight = w;
    return out;
}

ContactGraphRouter::WeightedPath
ContactGraphRouter::ShortestPathWeighted(uint32_t src, uint32_t dst) const
{
    return ShortestPathWeightedFiltered(src, dst, nullptr);
}

ContactGraphRouter::LifetimePath
ContactGraphRouter::LifetimeAwarePath(uint32_t src,
                                      uint32_t dst,
                                      double eps,
                                      const std::function<double(uint32_t, uint32_t)>& residualS,
                                      double minLifetimeS) const
{
    auto lifetimeOf = [&](const std::vector<uint32_t>& p) {
        double l = std::numeric_limits<double>::infinity();
        for (size_t k = 0; k + 1 < p.size(); ++k)
        {
            l = std::min(l, residualS(p[k], p[k + 1]));
        }
        return l;
    };
    WeightedPath best = ShortestPathWeighted(src, dst);
    LifetimePath out{best.path, best.total_weight, 0.0, 1};
    if (best.path.empty())
    {
        return out;
    }
    if (minLifetimeS > 0.0 && lifetimeOf(best.path) <= minLifetimeS)
    {
        WeightedPath alive = ShortestPathWeightedFiltered(src, dst, [&](uint32_t a, uint32_t b) {
            return residualS(a, b) > minLifetimeS;
        });
        ++out.dijkstra_runs;
        if (alive.path.empty())
        {
            out.lifetime_s = lifetimeOf(best.path);
            return out;
        }
        best = alive;
        out.path = best.path;
        out.total_weight = best.total_weight;
    }
    const double bound = (1.0 + eps) * best.total_weight;
    out.lifetime_s = lifetimeOf(best.path);
    while (true)
    {
        const double cur = out.lifetime_s;
        WeightedPath p = ShortestPathWeightedFiltered(src, dst, [&](uint32_t a, uint32_t b) {
            return residualS(a, b) > cur;
        });
        ++out.dijkstra_runs;
        if (p.path.empty() || p.total_weight > bound)
        {
            return out;
        }
        out.path = p.path;
        out.total_weight = p.total_weight;
        out.lifetime_s = lifetimeOf(p.path);
    }
}

ContactGraphRouter::WeightedPath
ContactGraphRouter::ShortestPathWeightedFiltered(uint32_t src,
                                                 uint32_t dst,
                                                 const EdgeFilter& keep) const
{
    ++m_queries;
    if (src == dst)
    {
        return {{src}, 0.0};
    }
    // Dijkstra over the flat adjacency; same relaxation order and tie rule as the map version.
    const size_t n = m_flat.size();
    if (src >= n || dst >= n)
    {
        return {{}, std::numeric_limits<double>::infinity()};
    }
    if (m_dist.size() < n)
    {
        m_dist.resize(n);
        m_pred.resize(n);
        m_stamp.assign(n, 0);
        m_stampNow = 0;
    }
    ++m_stampNow;
    const uint32_t st = m_stampNow;
    using QEntry = std::pair<double, uint32_t>; // (weight, node)
    std::priority_queue<QEntry, std::vector<QEntry>, std::greater<QEntry>> pq;
    m_dist[src] = 0.0;
    m_stamp[src] = st;
    pq.push({0.0, src});
    while (!pq.empty())
    {
        const auto [w, u] = pq.top();
        pq.pop();
        if (u == dst)
        {
            std::vector<uint32_t> path{dst};
            uint32_t c = dst;
            while (c != src)
            {
                c = m_pred[c];
                path.push_back(c);
            }
            std::reverse(path.begin(), path.end());
            return {std::move(path), w};
        }
        if (w > m_dist[u])
        {
            continue; // stale
        }
        for (const auto& [v, we] : m_flat[u])
        {
            if (keep && !keep(u, v))
            {
                continue;
            }
            const double newW = w + we;
            if (m_stamp[v] != st || newW < m_dist[v])
            {
                m_stamp[v] = st;
                m_dist[v] = newW;
                m_pred[v] = u;
                pq.push({newW, v});
            }
        }
    }
    return {{}, std::numeric_limits<double>::infinity()};
}

ContactGraphRouter::WeightedPath
ContactGraphRouter::ShortestPathCost(uint32_t src, uint32_t dst, const EdgeCost& cost) const
{
    ++m_queries;
    const size_t n = m_flat.size();
    if (src >= n || dst >= n)
    {
        return {{}, std::numeric_limits<double>::infinity()};
    }
    if (m_dist.size() < n)
    {
        m_dist.resize(n);
        m_pred.resize(n);
        m_stamp.assign(n, 0);
        m_stampNow = 0;
    }
    ++m_stampNow;
    const uint32_t st = m_stampNow;
    using QEntry = std::pair<double, uint32_t>;
    std::priority_queue<QEntry, std::vector<QEntry>, std::greater<QEntry>> pq;
    m_dist[src] = 0.0;
    m_stamp[src] = st;
    pq.push({0.0, src});
    while (!pq.empty())
    {
        const auto [w, u] = pq.top();
        pq.pop();
        if (u == dst)
        {
            std::vector<uint32_t> path{dst};
            uint32_t c = dst;
            double total = 0.0;
            while (c != src)
            {
                total += EdgeWeight(c, m_pred[c]);
                c = m_pred[c];
                path.push_back(c);
            }
            std::reverse(path.begin(), path.end());
            return {std::move(path), total};
        }
        if (w > m_dist[u])
        {
            continue;
        }
        for (const auto& [v, we] : m_flat[u])
        {
            const double ce = cost(u, v, we);
            if (!std::isfinite(ce))
            {
                continue;
            }
            const double newW = w + ce;
            if (m_stamp[v] != st || newW < m_dist[v])
            {
                m_stamp[v] = st;
                m_dist[v] = newW;
                m_pred[v] = u;
                pq.push({newW, v});
            }
        }
    }
    return {{}, std::numeric_limits<double>::infinity()};
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
