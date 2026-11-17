/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only
//
// sagin-sabr-router - Schedule-Aware Bundle Routing, CCSDS 734.3-B-1 (Blue
// Book, July 2019), Section 3 (contact graph routing).
//
// Why this exists (Wave 6B). The toolkit's "contact-graph" router in
// ntn-constellation (ContactGraphRouter) is a snapshot router: a BFS or
// Dijkstra over the links that are up NOW. It never consults future contacts,
// so it is not contact graph routing in the CCSDS sense, and the ntn-sagin
// store-and-forward example cited SABR only as "not implemented". This file
// implements the SABR procedures over a scheduled contact plan:
//
//   3.2.1     contact graph: vertices are contacts, a root contact X->X and a
//             terminal contact D->D; an edge is retention at a node between
//             a contact TO it and a later contact FROM it.
//   3.2.3     route pruning: contacts that ended are deleted, and route lists
//             are discarded whenever the contact plan changes.
//   3.2.4     earliest transmission time and earliest arrival time
//             (start or arrival of the previous contact, plus range in light
//             seconds plus the OWLT margin); best-case delivery time.
//   3.2.5.2   excluded nodes (the node the bundle came from, unless the
//             caller authorizes backward propagation).
//   3.2.6.2   earliest transmission opportunity: adjusted start, applicable
//             backlog (Wave 7B: bits queued for the entry node at the
//             bundle's priority or higher, from a per-neighbor backlog
//             function), backlog relief from applicable prior contacts,
//             residual backlog and backlog lien.
//   3.2.6.3-7 first/last byte transmission and arrival times, projected
//             bundle arrival time (PBAT).
//   3.2.6.8   MTV per priority level (Wave 7B): bulk 0, normal 1,
//             expedited 2; enqueueing a bundle of priority p consumes the
//             EVC from the MTV of p and of every lower level (a bulk bundle
//             can be pre-empted by a normal or expedited one, so it does not
//             reduce their volume; an expedited bundle delays everything
//             below it). Effective start/stop/duration, EVL from the MTV of
//             the bundle's priority, RVL, route depletion.
//   3.2.6.9   candidate filters a) to h).
//   3.2.6.9.1 the next best route is computed only while the list holds no
//             candidate; the cap on additional routes (MaxRoutes) is the
//             implementation matter the clause leaves open.
//   3.2.6.10  next best route: shortest path from root to terminal vertex
//             excluding all previously identified paths, edge cost = earliest
//             arrival time of the terminating contact, by Yen's K shortest
//             paths (the method the NOTE names).
//   3.2.7     no candidate: CGR is not used (the caller holds the bundle).
//   3.2.8.1   best candidate: earliest PBAT, then fewest contacts, then
//             latest termination time, then smallest entry node number; on
//             enqueue the MTV of every contact of the route drops by the EVC.
//   3.2.6.9.2 / 3.2.8.1.3  critical bundles (Wave 7B, RouteCritical): every
//             route up to MaxRoutes is computed, not only until a candidate
//             appears, and one copy goes to the entry node of every
//             candidate route (the best candidate route per entry node).
//
// Not implemented: anticipatory fragmentation (3.2.8.2), the ordinal of
// expedited bundles, probe bundles and excluded-neighbor bookkeeping beyond
// the caller's list (3.2.5.2 b).
// Contact plans in this toolkit come from SGP4 visibility (the ephemeris),
// not from random contacts. The estimated volume consumption (EVC) is the
// bundle's serialized size: convergence-layer overhead is not added.

#ifndef SAGIN_SABR_ROUTER_H
#define SAGIN_SABR_ROUTER_H

#include "ns3/object.h"

#include <array>
#include <cstdint>
#include <functional>
#include <map>
#include <set>
#include <vector>

namespace ns3
{
namespace ntnsagin
{

/// CCSDS 734.3-B-1 3.2.6.8 priority levels.
enum SabrPriority : uint8_t
{
    SABR_BULK = 0,
    SABR_NORMAL = 1,
    SABR_EXPEDITED = 2,
};

/// One scheduled contact: an interval of opportunity from one node to another.
struct SabrContact
{
    uint32_t from{0};
    uint32_t to{0};
    double startS{0.0};    //!< contact start (s, simulation time)
    double endS{0.0};      //!< contact end (s)
    double rateBps{0.0};   //!< data rate (bit/s)
    double owltS{0.0};     //!< range in light seconds during the contact
};

/// A route through the contact graph (contact indices, root excluded).
struct SabrRoute
{
    std::vector<uint32_t> contacts;
    double bestCaseDeliveryS{0.0}; //!< 3.2.4.2
    double terminationS{0.0};      //!< earliest end among the route's contacts
    uint32_t entryNode{0};         //!< receiving node of the first contact
};

/// The outcome of one forwarding decision (3.2.6.9 / 3.2.8.1).
struct SabrDecision
{
    bool found{false};
    uint32_t entryNode{0};
    SabrRoute route;
    double pbatS{0.0};     //!< projected bundle arrival time, 3.2.6.7
    double etoS{0.0};      //!< earliest transmission opportunity, 3.2.6.2
    double rvlBits{0.0};   //!< route volume limit, 3.2.6.8.10
    uint32_t routesComputed{0};
    uint32_t candidates{0};
};

/**
 * \ingroup ntn-sagin
 * \brief CCSDS 734.3-B-1 SABR contact graph routing at one local node.
 */
class SaginSabrRouter : public Object
{
  public:
    static TypeId GetTypeId();
    SaginSabrRouter() = default;

    /// The local node X.
    void SetLocalNode(uint32_t x) { m_local = x; }
    uint32_t GetLocalNode() const { return m_local; }

    /// Replace the contact plan. Discards all route lists (3.2.3.1) and resets
    /// every contact's MTV to its volume (3.2.6.8.8).
    void SetContactPlan(const std::vector<SabrContact>& plan);
    /// Add one contact (a plan change: route lists are discarded, 3.2.3.1).
    void AddContact(const SabrContact& c);
    const std::vector<SabrContact>& GetContactPlan() const { return m_plan; }

    /// OWLT margin added to every range (3.2.4.1.2, 3.2.6.5), seconds.
    void SetOwltMarginS(double s) { m_owltMarginS = s; }
    /// Cap on routes computed per destination while no candidate exists
    /// (3.2.6.9.1 leaves the stopping condition to the implementation).
    void SetMaxRoutes(uint32_t n) { m_maxRoutes = n; }

    /// Bits queued at the local node for transmission to a neighbor, for the
    /// applicable backlog of 3.2.6.2 b). Default: no backlog.
    using BacklogFunction = std::function<double(uint32_t neighbor)>;
    void SetBacklogFunction(BacklogFunction f) { m_backlog = std::move(f); }
    /// Wave 7B, 3.2.6.2 b): bits queued at the local node for transmission to
    /// \p neighbor whose priority is \p priority or higher. Takes precedence
    /// over SetBacklogFunction.
    using PriorityBacklogFunction = std::function<double(uint32_t neighbor, uint8_t priority)>;
    void SetPriorityBacklogFunction(PriorityBacklogFunction f) { m_prioBacklog = std::move(f); }

    /**
     * \brief Select the forwarding route for one bundle (3.2.3 to 3.2.8.1).
     *
     * \param dest          destination node D
     * \param nowS          current time (s)
     * \param evcBits       estimated volume consumption of the bundle (bits)
     * \param expiryS       the bundle's expiration time (creation + lifetime, s)
     * \param excluded      excluded nodes list (3.2.5.2)
     * \param noFragment    the "must not be fragmented" flag (3.2.6.9 g)
     * \param reserve       on a decision, reduce the MTV of the route's contacts
     *                      by the EVC (3.2.8.1.2)
     * \param priority      the bundle's priority (3.2.6.8); with a single
     *                      priority in use every level behaves as one MTV
     */
    SabrDecision Route(uint32_t dest,
                       double nowS,
                       double evcBits,
                       double expiryS,
                       const std::set<uint32_t>& excluded = {},
                       bool noFragment = true,
                       bool reserve = true,
                       uint8_t priority = SABR_NORMAL);

    /// Wave 7B, critical bundle (3.2.6.9.2, 3.2.8.1.3): all routes are
    /// computed; returns the best candidate route of every distinct entry
    /// node, best first. Empty when no route is a candidate.
    std::vector<SabrDecision> RouteCritical(uint32_t dest,
                                            double nowS,
                                            double evcBits,
                                            double expiryS,
                                            const std::set<uint32_t>& excluded = {},
                                            bool noFragment = true,
                                            bool reserve = true,
                                            uint8_t priority = SABR_NORMAL);

    /// 3.2.8.1.2: reduce the MTV of every contact of \p r by \p evcBits (use
    /// with Route(..., reserve = false) once the bundle is actually enqueued),
    /// at \p priority and every lower level (3.2.6.8).
    void Reserve(const SabrRoute& r, double evcBits, uint8_t priority = SABR_NORMAL);
    /// Route list currently held for D (for inspection and tests).
    const std::vector<SabrRoute>& GetRouteList(uint32_t dest) const;
    /// Residual MTV (bits) of a contact at a priority level.
    double GetMtvBits(uint32_t contactIdx, uint8_t priority = SABR_NORMAL) const
    {
        return m_mtv.at(contactIdx).at(priority > 2 ? 2 : priority);
    }

    uint64_t GetRouteComputations() const { return m_routeComputations; }
    uint64_t GetNoRouteDecisions() const { return m_noRoute; }

  private:
    struct Cand
    {
        size_t idx;
        double pbat;
        double eto;
        double rvl;
    };
    /// 3.2.6.9: the candidate routes among the routes held for \p dest.
    std::vector<Cand> Evaluate(uint32_t dest,
                               double nowS,
                               double evcBits,
                               double expiryS,
                               const std::set<uint32_t>& excluded,
                               bool noFragment,
                               uint8_t priority);
    /// 3.2.8.1.4 a) ordering.
    bool Better(uint32_t dest, const Cand& a, const Cand& b) const;
    SabrDecision MakeDecision(uint32_t dest, const Cand& c) const;
    /// Dijkstra over the contact graph from node \p fromNode available at
    /// \p fromTime, to D, avoiding banned contacts and nodes (3.2.4, 3.2.6.10).
    bool ShortestPath(uint32_t fromNode,
                      double fromTime,
                      uint32_t dest,
                      const std::set<uint32_t>& bannedContacts,
                      const std::set<uint32_t>& bannedNodes,
                      std::vector<uint32_t>& out) const;
    /// Earliest arrival time along a contact sequence starting at \p t0 (3.2.4.1).
    bool EarliestArrival(const std::vector<uint32_t>& contacts, double t0, double& arrival) const;
    /// Yen: the next best route not yet in the list; false when none exists.
    bool NextRoute(uint32_t dest, double nowS);
    void Prune(double nowS);
    void FillRouteMetrics(SabrRoute& r, double nowS) const;

    uint32_t m_local{0};
    double m_owltMarginS{0.0};
    uint32_t m_maxRoutes{8};
    std::vector<SabrContact> m_plan;
    std::vector<std::array<double, 3>> m_mtv; //!< per-contact MTV per priority (bits)
    std::vector<bool> m_deleted;             //!< pruned contacts (3.2.3.2)
    std::map<uint32_t, std::vector<uint32_t>> m_byFrom; //!< node -> contacts from it
    std::map<uint32_t, std::vector<SabrRoute>> m_routes; //!< D -> route list
    std::map<uint32_t, std::vector<SabrRoute>> m_yenB;   //!< D -> Yen candidate heap
    BacklogFunction m_backlog;
    PriorityBacklogFunction m_prioBacklog;
    uint64_t m_routeComputations{0};
    uint64_t m_noRoute{0};
};

} // namespace ntnsagin
} // namespace ns3

#endif // SAGIN_SABR_ROUTER_H
