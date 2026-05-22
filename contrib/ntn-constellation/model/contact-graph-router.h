/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#ifndef NTN_CONSTELLATION_CONTACT_GRAPH_ROUTER_H
#define NTN_CONSTELLATION_CONTACT_GRAPH_ROUTER_H

// Contact-graph router (Roadmap §4.4.4).
//
// Subscribes to a `ContactGraphScheduler`'s `m_contactUp / m_contactDown`
// trace sources and maintains an undirected adjacency graph keyed by the
// caller-supplied node IDs (satellites + ground stations). Every link
// state change updates the graph; consumers query `ShortestPath(src, dst)`
// to get a hop-count-minimised route through the live constellation.
//
// The router is link-state at the graph level (BFS over current edges)
// but does NOT recompute on every query — it only mutates the graph on
// scheduler events. That keeps the BFS amortised O(V + E) per query and
// O(1) per event.
//
// Used by SAGIN integration to react to GSL/ISL up/down events from the
// contact graph and pick a fresh route through the constellation. The
// route is a list of node IDs starting with `src` and ending with `dst`,
// or empty if no route exists in the current graph.

#include "contact-graph-scheduler.h"

#include <ns3/object.h>

#include <atomic>
#include <cstdint>
#include <map>
#include <set>
#include <vector>

namespace ns3
{
namespace ntncon
{

class ContactGraphRouter : public Object
{
  public:
    static TypeId GetTypeId();
    ContactGraphRouter();
    ~ContactGraphRouter() override = default;

    /// Subscribe to a scheduler's contact-up / contact-down trace
    /// sources. Multiple schedulers may be attached; their events all
    /// converge into the same edge graph.
    void Attach(Ptr<ContactGraphScheduler> scheduler);

    /// Returns true if there is currently an edge between `a` and `b`.
    bool HasEdge(uint32_t a, uint32_t b) const;

    /// Returns the number of currently-active edges.
    size_t NumEdges() const;

    /// All neighbours of `node` in the current graph.
    std::set<uint32_t> Neighbours(uint32_t node) const;

    /// Hop-count shortest path from `src` to `dst`. Returns an empty
    /// vector when no path exists; otherwise the path includes both
    /// endpoints, so a length-1 path is `{src}` (when src == dst) and
    /// a length-2 path is `{src, dst}` (direct edge).
    std::vector<uint32_t> ShortestPath(uint32_t src, uint32_t dst) const;

    /// Counters that the trace handlers tick (test asserts).
    uint64_t EdgesAddedTotal() const { return m_added.load(); }
    uint64_t EdgesRemovedTotal() const { return m_removed.load(); }
    uint64_t RouteQueries() const { return m_queries.load(); }

  private:
    void HandleContactEvent(const ContactEvent& ev);
    void OnContactUp(ContactEvent ev) { HandleContactEvent(ev); }
    void OnContactDown(ContactEvent ev) { HandleContactEvent(ev); }

    static std::pair<uint32_t, uint32_t> CanonicalEdge(uint32_t a, uint32_t b)
    {
        return (a <= b) ? std::pair<uint32_t, uint32_t>{a, b}
                         : std::pair<uint32_t, uint32_t>{b, a};
    }

    /// Adjacency map: node -> set of neighbours.
    std::map<uint32_t, std::set<uint32_t>> m_adj;
    /// Set of canonical (low, high) edge IDs currently up.
    std::set<std::pair<uint32_t, uint32_t>> m_edges;

    mutable std::atomic<uint64_t> m_added{0};
    mutable std::atomic<uint64_t> m_removed{0};
    mutable std::atomic<uint64_t> m_queries{0};
};

} // namespace ntncon
} // namespace ns3

#endif // NTN_CONSTELLATION_CONTACT_GRAPH_ROUTER_H
