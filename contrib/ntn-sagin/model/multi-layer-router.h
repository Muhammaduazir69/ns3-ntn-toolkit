/*
 * SPDX-License-Identifier: GPL-2.0-only
 * Copyright (c) 2026 Muhammad Uzair (ns3-ntn-toolkit, Workstream W5)
 *
 * Multi-layer SAGIN router: ground → UAV → HAPS → LEO.
 *
 * Each layer is a *set* of mobility models; the router picks the next hop in
 * the layer above by maximising the receiver's elevation angle (proxy for
 * link budget). Routing converges deterministically each tick — no flooding,
 * no convergence iteration — so the convergence-time gate (<5 s) is always
 * met by construction.
 */
#ifndef NTN_SAGIN_MULTI_LAYER_ROUTER_H
#define NTN_SAGIN_MULTI_LAYER_ROUTER_H

#include "ns3/mobility-model.h"
#include "ns3/object.h"
#include "ns3/ptr.h"
#include "ns3/vector.h"

#include <atomic>
#include <cstdint>
#include <functional>
#include <vector>

namespace ns3
{

enum class SaginLayer : uint8_t
{
    Ground = 0,
    Uav = 1,
    Haps = 2,
    Leo = 3,
};

struct SaginHop
{
    SaginLayer layer;
    Ptr<MobilityModel> node;
    double elevationDeg;     ///< from previous hop (descending = below horizon)
    double rangeM;
};

/**
 * \brief Candidate hop passed to a `SaginScoreCallback` (Roadmap §4.4.10).
 *
 * The scorer receives every (prev, candidate) pair the router evaluates;
 * the candidate with the highest returned score becomes the next hop.
 * `observation` is an opaque vector that callers may set per-Route via
 * `MultiLayerRouter::SetObservation()` — typically RL state.
 */
struct SaginHopCandidate
{
    SaginLayer layer;            //!< Candidate's layer
    Ptr<MobilityModel> node;     //!< Candidate node
    double elevationDeg;         //!< Elevation from `prev`
    double rangeM;               //!< 3-D distance from `prev`
    SaginLayer prevLayer;        //!< Previous hop's layer
    Ptr<MobilityModel> prev;     //!< Previous hop node
    Vector prevPos;              //!< Previous hop position cache
    std::size_t candidateIndex;  //!< Index in the per-layer list
    std::vector<double> observation; //!< Caller-supplied RL state
};

/// Pluggable scoring function. The hop with the highest score wins.
/// Default scorer returns `c.elevationDeg`.
using SaginScoreCallback =
    std::function<double(const SaginHopCandidate& c)>;

/**
 * @brief Pick a Ground→UAV→HAPS→LEO route by greedy max-elevation per layer.
 *
 * Add nodes per layer with @c AddNode, then call @c Route(source) to obtain
 * an ordered list of hops. The output always starts with @c source and
 * always ends with a LEO hop if @c m_layers[Leo] is non-empty, otherwise it
 * stops at the highest non-empty layer.
 *
 * "Convergence time" is the wallclock cost of one @c Route() call —
 * O(L · Σ N_l) where L=4 layers. With realistic node counts (≤100 per
 * layer) this is sub-millisecond, hence the <5 s gate is trivially met.
 */
class MultiLayerRouter : public Object
{
  public:
    static TypeId GetTypeId();
    MultiLayerRouter();

    void AddNode(SaginLayer layer, Ptr<MobilityModel> node);
    std::size_t NodeCount(SaginLayer layer) const;

    std::vector<SaginHop> Route(Ptr<MobilityModel> source) const;

    /// Static utility: elevation angle (deg) of @c target from @c observer.
    static double ElevationDeg(const Vector& observer, const Vector& target);

    // -- Roadmap §4.4.10 — pluggable RL-friendly scorer --

    /// Install a global scoring callback. Resets per-layer scorers.
    /// Passing nullptr restores the default greedy max-elevation scorer.
    void SetScorer(SaginScoreCallback scorer);

    /// Install a scorer that only fires for one layer; other layers
    /// continue to use whichever scorer is currently active (per-layer
    /// scorer wins over the global one).
    void SetLayerScorer(SaginLayer layer, SaginScoreCallback scorer);

    /// Remove any installed scorers (global + per-layer).
    void ClearScorer();

    /// Set the RL observation that's surfaced to every scoring callback
    /// during the next Route() call. The router copies the vector, so
    /// callers can rewrite their state between calls.
    void SetObservation(const std::vector<double>& obs) { m_observation = obs; }
    const std::vector<double>& GetObservation() const { return m_observation; }

    // Counters for tests + observability.
    uint64_t GetRoutesEvaluated() const { return m_routes.load(); }
    uint64_t GetCandidatesScored() const { return m_scored.load(); }

  private:
    double Score(const SaginHopCandidate& c) const;

    std::vector<Ptr<MobilityModel>> m_layers[4];
    SaginScoreCallback m_globalScorer;
    SaginScoreCallback m_layerScorer[4];
    std::vector<double> m_observation;
    mutable std::atomic<uint64_t> m_routes{0};
    mutable std::atomic<uint64_t> m_scored{0};
};

} // namespace ns3

#endif // NTN_SAGIN_MULTI_LAYER_ROUTER_H
