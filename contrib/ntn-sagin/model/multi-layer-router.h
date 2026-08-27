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

    // ---- SAGIN-2: load state, so a router can see a saturated link ----
    //
    // No capacity, queue occupancy or congestion term existed in any SAGIN
    // routing decision: the default scorer returned c.elevationDeg and nothing
    // else, and this struct carried no load field at all. A router that cannot
    // see a saturated ISL steers every slice onto the same satellite - which is
    // the central question these modules exist to study. Worse, the P2P ISLs in
    // sagin-sgp4-routed-traffic have a default DropTail queue, so traffic could
    // be dropped on a link the router still called shortest.
    //
    // These are populated by SetLinkLoadSource(); they stay at their defaults
    // when no source is registered, and the composite scorer degrades to the
    // max-elevation behaviour in that case rather than inventing a load.

    double capacityBps{0.0};      //!< link capacity toward this candidate, 0 = unknown
    double queueOccupancy{0.0};   //!< TxQueue fill fraction 0..1, 0 = unknown/empty
    bool haveLoad{false};         //!< whether the two fields above are real
};

/// Pluggable scoring function. The hop with the highest score wins.
/// Default scorer returns `c.elevationDeg`.
using SaginScoreCallback =
    std::function<double(const SaginHopCandidate& c)>;

/// SAGIN-2: supplies live load for the link from `prev` to `node`.
///
/// Returns false when nothing is known about that link, which is what keeps
/// the composite scorer honest: an unknown load must not be scored as an empty
/// one. A caller wires this to the P2P device's DataRate and TxQueue.
using SaginLinkLoadCallback =
    std::function<bool(Ptr<MobilityModel> prev, Ptr<MobilityModel> node,
                       double& capacityBps, double& queueOccupancy)>;

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

    /**
     * \brief SAGIN-9: let a hop skip a populated layer when a higher one scores
     *        better.
     *
     * Route() walked layers 1..3 in order and `continue`d only when a layer was
     * EMPTY, so if the UAV layer held any node at all the ground source had to
     * transit it, however much better a direct ground-to-LEO link was. With this
     * enabled each hop considers every remaining layer above the current one and
     * takes the best-scoring candidate, so the layer sequence follows the score
     * rather than the loop index. The path still climbs: the next hop resumes
     * above whatever layer was chosen, so a skipped layer is never revisited.
     *
     * Defaults FALSE, because enabling it changes which path every existing
     * SAGIN scenario takes and the committed results were measured without it.
     */
    void SetAllowLayerSkip(bool enable) { m_allowLayerSkip = enable; }
    bool GetAllowLayerSkip() const { return m_allowLayerSkip; }

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

    /// Read back the currently-installed global scorer (may be nullptr).
    /// Together with @c GetLayerScorer this lets a wrapper (e.g.
    /// SaginSliceRouter) SAVE the router's scorer state, install its own for
    /// one Route(), then RESTORE the caller's scorer exactly.
    SaginScoreCallback GetScorer() const { return m_globalScorer; }

    /// Read back the per-layer scorer for @c layer (may be nullptr).
    SaginScoreCallback GetLayerScorer(SaginLayer layer) const
    {
        return m_layerScorer[static_cast<uint8_t>(layer)];
    }

    /// Minimum elevation (deg) a candidate must present to the previous hop
    /// to be feasible. The DEFAULT (greedy max-elevation) scorer returns
    /// -inf below this floor, so a below-horizon hop is never selected and
    /// Route() truncates the path at the last feasible layer. Custom scorers
    /// (installed via SetScorer / SetLayerScorer) are responsible for their
    /// own feasibility and are not affected by this gate.
    void SetMinElevationDeg(double deg) { m_minElevationDeg = deg; }
    double GetMinElevationDeg() const { return m_minElevationDeg; }

    /// Set the RL observation that's surfaced to every scoring callback
    /// during the next Route() call. The router copies the vector, so
    /// callers can rewrite their state between calls.
    void SetObservation(const std::vector<double>& obs) { m_observation = obs; }

    // ---- SAGIN-2 ----

    /// Register a source of live per-link capacity and queue occupancy.
    /// Without one, every candidate carries haveLoad == false and the
    /// composite scorer falls back to elevation alone.
    void SetLinkLoadSource(SaginLinkLoadCallback cb) { m_loadSource = cb; }

    /// Score by elevation AND load: a link whose queue is filling is penalised,
    /// and one at or above the congestion cutoff is rejected outright the same
    /// way a below-horizon hop is.
    ///
    /// score = elevationDeg - congestionWeight * 90 * occupancy
    ///
    /// The 90 scales the penalty into the same units as elevation, so a fully
    /// congested link loses the whole elevation range and a half-full one loses
    /// congestionWeight * 45 degrees of apparent elevation. Candidates whose
    /// load is unknown are scored on elevation alone rather than being assumed
    /// idle - assuming idle is how a saturated link keeps winning.
    void SetCongestionAware(bool on, double congestionWeight = 1.0,
                            double rejectAboveOccupancy = 0.95);
    bool GetCongestionAware() const { return m_congestionAware; }

    /// Candidates rejected by the congestion cutoff since construction.
    uint64_t GetCongestionRejections() const { return m_congestionRejections; }
    const std::vector<double>& GetObservation() const { return m_observation; }

    // Counters for tests + observability.
    uint64_t GetRoutesEvaluated() const { return m_routes.load(); }
    uint64_t GetCandidatesScored() const { return m_scored.load(); }

  private:
    double Score(const SaginHopCandidate& c) const;

    std::vector<Ptr<MobilityModel>> m_layers[4];
    SaginScoreCallback m_globalScorer;
    SaginScoreCallback m_layerScorer[4];
    double m_minElevationDeg{0.0};
    std::vector<double> m_observation;
    SaginLinkLoadCallback m_loadSource;
    bool m_congestionAware{false};
    double m_congestionWeight{1.0};
    double m_rejectAboveOccupancy{0.95};
    mutable uint64_t m_congestionRejections{0};
    bool m_allowLayerSkip{false}; //!< SAGIN-9: consider higher layers per hop
    mutable std::atomic<uint64_t> m_routes{0};
    mutable std::atomic<uint64_t> m_scored{0};
};

} // namespace ns3

#endif // NTN_SAGIN_MULTI_LAYER_ROUTER_H
