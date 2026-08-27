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

/// Per-satellite payload mode per 3GPP TR 38.821 (Roadmap §4.4.6).
///   bent_pipe   — transparent relay; no protocol termination on the sat.
///   regen_du    — gNB-DU on sat: PHY + MAC terminated.
///   regen_cu    — gNB-CU on sat: PDCP + RRC terminated.
///   regen_full  — full gNB on sat: DU + CU.
enum class RegenMode : uint8_t
{
    bent_pipe = 0,
    regen_du = 1,
    regen_cu = 2,
    regen_full = 3,
};

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

    /// Hop-count shortest path from `src` to `dst` (BFS). Returns an
    /// empty vector when no path exists; otherwise the path includes
    /// both endpoints, so a length-1 path is `{src}` (when src == dst)
    /// and a length-2 path is `{src, dst}` (direct edge).
    std::vector<uint32_t> ShortestPath(uint32_t src, uint32_t dst) const;

    /// Link-weighted shortest path (Dijkstra) — Roadmap §4.4.5.
    /// Edge weight is the contact event's `range_m`. Consumers can
    /// interpret `total_weight` as latency by dividing by c. Returns an
    /// empty `path` when no route exists; total_weight is +inf in that
    /// case.
    struct WeightedPath
    {
        std::vector<uint32_t> path;
        double total_weight;
    };
    WeightedPath ShortestPathWeighted(uint32_t src, uint32_t dst) const;

    /**
     * \brief SAGIN-7: minimum-HOP path over the same contact graph.
     *
     * ns-3's Ipv4GlobalRouting forwards on hop count, so this is the route
     * packets actually take, while ShortestPathWeighted is the route the
     * contact-graph model recommends. A scenario that prints the weighted path
     * beside measured goodput is comparing a model decision with a data plane
     * that may have chosen differently, and before this there was no way to
     * tell whether the two agreed.
     *
     * `total_weight` carries the range sum ALONG THE HOP-COUNT PATH, so the two
     * results are directly comparable in latency terms.
     */
    WeightedPath ShortestPathHops(uint32_t src, uint32_t dst) const;

    /// Current weight of the edge (a, b); returns NaN if no edge.
    double EdgeWeight(uint32_t a, uint32_t b) const;

    /// Roadmap §4.4.6 — per-node regenerative payload mode. Defaults to
    /// `bent_pipe` for any node that hasn't been explicitly set.
    void SetRegenMode(uint32_t node, RegenMode mode);
    RegenMode GetRegenMode(uint32_t node) const;

    /// True iff `node`'s mode is regen_du / regen_cu / regen_full.
    bool IsRegenerative(uint32_t node) const;

    /// Dijkstra with the constraint that all INTERMEDIATE hops must be
    /// regen-capable. Endpoints (src and dst) may be in any mode — a
    /// bent-pipe sat / GS is fine as a source or destination, but only
    /// regen sats can transit traffic through ISLs in their payload.
    /// Roadmap §4.4.6.
    WeightedPath ShortestPathWeightedRegenOnly(uint32_t src,
                                                  uint32_t dst) const;

    /// Counters that the trace handlers tick (test asserts).
    uint64_t EdgesAddedTotal() const { return m_added.load(); }
    uint64_t EdgesRemovedTotal() const { return m_removed.load(); }
    uint64_t RouteQueries() const { return m_queries.load(); }

  private:
    void HandleContactEvent(const ContactEvent& ev);
    /// SAGIN-1: refresh an established edge's weight from the live range.
    void UpdateEdgeWeight(const ContactEvent& ev);
  public:
    /// SAGIN-7 test seam: feed a contact directly. The router is normally
    /// driven by an attached scheduler, so a unit test could not build a
    /// specific topology, which is why the hop-vs-weight distinction had no
    /// coverage.
    void InjectContactForTest(const ContactEvent& ev)
    {
        HandleContactEvent(ev);
        UpdateEdgeWeight(ev);
    }

  private:
    void OnContactUp(ContactEvent ev) { HandleContactEvent(ev); }
    void OnContactDown(ContactEvent ev) { HandleContactEvent(ev); }
    void OnContactUpdate(ContactEvent ev) { UpdateEdgeWeight(ev); }

    static std::pair<uint32_t, uint32_t> CanonicalEdge(uint32_t a, uint32_t b)
    {
        return (a <= b) ? std::pair<uint32_t, uint32_t>{a, b}
                         : std::pair<uint32_t, uint32_t>{b, a};
    }

    /// Adjacency map: node -> set of neighbours.
    std::map<uint32_t, std::set<uint32_t>> m_adj;
    /// Set of canonical (low, high) edge IDs currently up.
    std::set<std::pair<uint32_t, uint32_t>> m_edges;
    /// Per-edge weight (link range in metres at the latest event for
    /// this edge). Used by ShortestPathWeighted.
    std::map<std::pair<uint32_t, uint32_t>, double> m_edgeWeights;
    /// Per-node payload mode (Roadmap §4.4.6). Missing entry defaults
    /// to bent_pipe.
    std::map<uint32_t, RegenMode> m_regenMode;

    mutable std::atomic<uint64_t> m_added{0};
    mutable std::atomic<uint64_t> m_removed{0};
    mutable std::atomic<uint64_t> m_queries{0};
};

} // namespace ntncon
} // namespace ns3

#endif // NTN_CONSTELLATION_CONTACT_GRAPH_ROUTER_H
