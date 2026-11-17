/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#include "sagin-sabr-router.h"

#include "ns3/log.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>

namespace ns3
{
namespace ntnsagin
{

NS_LOG_COMPONENT_DEFINE("SaginSabrRouter");
NS_OBJECT_ENSURE_REGISTERED(SaginSabrRouter);

TypeId
SaginSabrRouter::GetTypeId()
{
    static TypeId tid = TypeId("ns3::ntnsagin::SaginSabrRouter")
                            .SetParent<Object>()
                            .SetGroupName("NtnSagin")
                            .AddConstructor<SaginSabrRouter>();
    return tid;
}

void
SaginSabrRouter::SetContactPlan(const std::vector<SabrContact>& plan)
{
    m_plan = plan;
    m_mtv.assign(m_plan.size(), {0.0, 0.0, 0.0});
    m_deleted.assign(m_plan.size(), false);
    m_byFrom.clear();
    for (uint32_t i = 0; i < m_plan.size(); ++i)
    {
        // 3.2.6.8.8: the initial MTV of a contact is its volume.
        const double vol = m_plan[i].rateBps * std::max(0.0, m_plan[i].endS - m_plan[i].startS);
        m_mtv[i] = {vol, vol, vol};
        m_byFrom[m_plan[i].from].push_back(i);
    }
    m_routes.clear(); // 3.2.3.1
    m_yenB.clear();
}

void
SaginSabrRouter::AddContact(const SabrContact& c)
{
    m_plan.push_back(c);
    const double vol = c.rateBps * std::max(0.0, c.endS - c.startS);
    m_mtv.push_back({vol, vol, vol});
    m_deleted.push_back(false);
    m_byFrom[c.from].push_back(static_cast<uint32_t>(m_plan.size() - 1));
    m_routes.clear(); // 3.2.3.1: a plan change invalidates every route list
    m_yenB.clear();
}

const std::vector<SabrRoute>&
SaginSabrRouter::GetRouteList(uint32_t dest) const
{
    static const std::vector<SabrRoute> kEmpty;
    auto it = m_routes.find(dest);
    return it == m_routes.end() ? kEmpty : it->second;
}

void
SaginSabrRouter::Prune(double nowS)
{
    // 3.2.3.2: every contact whose end time is in the past is deleted; that is
    // a plan modification, so the route lists go too (3.2.3.1).
    bool changed = false;
    for (uint32_t i = 0; i < m_plan.size(); ++i)
    {
        if (!m_deleted[i] && m_plan[i].endS < nowS)
        {
            m_deleted[i] = true;
            changed = true;
        }
    }
    if (changed)
    {
        m_routes.clear();
        m_yenB.clear();
    }
}

bool
SaginSabrRouter::EarliestArrival(const std::vector<uint32_t>& contacts,
                                 double t0,
                                 double& arrival) const
{
    // 3.2.4.1.1 / 3.2.4.1.2: earliest transmission time is the later of the
    // contact start and the arrival over the previous contact (the current
    // time for the first); arrival adds range and OWLT margin. A contact that
    // ends before its earliest transmission time cannot be in the route.
    double t = t0;
    for (uint32_t c : contacts)
    {
        const SabrContact& k = m_plan[c];
        const double ett = std::max(k.startS, t);
        if (ett >= k.endS)
        {
            return false;
        }
        t = ett + k.owltS + m_owltMarginS;
    }
    arrival = t;
    return true;
}

bool
SaginSabrRouter::ShortestPath(uint32_t fromNode,
                              double fromTime,
                              uint32_t dest,
                              const std::set<uint32_t>& bannedContacts,
                              const std::set<uint32_t>& bannedNodes,
                              std::vector<uint32_t>& out) const
{
    // Dijkstra on the contact graph (3.2.1): vertices are contacts, the cost
    // of reaching a contact is its earliest arrival time (3.2.6.10).
    const double inf = std::numeric_limits<double>::infinity();
    std::vector<double> dist(m_plan.size(), inf);
    std::vector<int64_t> pred(m_plan.size(), -1);
    std::vector<bool> done(m_plan.size(), false);
    using QE = std::pair<double, uint32_t>;
    std::priority_queue<QE, std::vector<QE>, std::greater<QE>> pq;

    auto relax = [&](uint32_t c, double t, int64_t p) {
        if (m_deleted[c] || bannedContacts.count(c) || bannedNodes.count(m_plan[c].to))
        {
            return;
        }
        const SabrContact& k = m_plan[c];
        const double ett = std::max(k.startS, t);
        if (ett >= k.endS)
        {
            return;
        }
        const double eat = ett + k.owltS + m_owltMarginS;
        if (eat < dist[c])
        {
            dist[c] = eat;
            pred[c] = p;
            pq.emplace(eat, c);
        }
    };
    auto it0 = m_byFrom.find(fromNode);
    if (it0 == m_byFrom.end())
    {
        return false;
    }
    for (uint32_t c : it0->second)
    {
        relax(c, fromTime, -1);
    }
    while (!pq.empty())
    {
        const auto [d, c] = pq.top();
        pq.pop();
        if (done[c] || d > dist[c])
        {
            continue;
        }
        done[c] = true;
        if (m_plan[c].to == dest)
        {
            // Reached the terminal vertex D->D through this contact.
            out.clear();
            for (int64_t x = c; x >= 0; x = pred[x])
            {
                out.push_back(static_cast<uint32_t>(x));
            }
            std::reverse(out.begin(), out.end());
            return true;
        }
        // Retention at node to(c), then any later contact from it. A node
        // already on this path is not revisited (no loops).
        std::set<uint32_t> onPath;
        for (int64_t x = c; x >= 0; x = pred[x])
        {
            onPath.insert(m_plan[x].from);
        }
        auto it = m_byFrom.find(m_plan[c].to);
        if (it == m_byFrom.end())
        {
            continue;
        }
        for (uint32_t n : it->second)
        {
            if (!onPath.count(m_plan[n].to))
            {
                relax(n, d, c);
            }
        }
    }
    return false;
}

void
SaginSabrRouter::FillRouteMetrics(SabrRoute& r, double nowS) const
{
    double a = std::numeric_limits<double>::infinity();
    if (!EarliestArrival(r.contacts, nowS, a))
    {
        a = std::numeric_limits<double>::infinity();
    }
    r.bestCaseDeliveryS = a; // 3.2.4.2
    r.terminationS = std::numeric_limits<double>::infinity();
    for (uint32_t c : r.contacts)
    {
        r.terminationS = std::min(r.terminationS, m_plan[c].endS);
    }
    r.entryNode = r.contacts.empty() ? 0 : m_plan[r.contacts.front()].to;
}

bool
SaginSabrRouter::NextRoute(uint32_t dest, double nowS)
{
    ++m_routeComputations;
    auto& A = m_routes[dest];
    auto& B = m_yenB[dest];
    std::set<uint32_t> bannedNodes;
    if (dest != m_local)
    {
        bannedNodes.insert(m_local); // 3.2.6.9 c): no contact back to X
    }
    if (A.empty())
    {
        std::vector<uint32_t> path;
        if (!ShortestPath(m_local, nowS, dest, {}, bannedNodes, path))
        {
            return false;
        }
        SabrRoute r;
        r.contacts = path;
        FillRouteMetrics(r, nowS);
        A.push_back(r);
        return true;
    }
    // 3.2.6.10, NOTE: Yen's K shortest paths. Spur from every vertex of the
    // last route found, banning the continuations already used by routes
    // that share the same root path.
    const std::vector<uint32_t> last = A.back().contacts;
    for (size_t i = 0; i < last.size(); ++i)
    {
        std::vector<uint32_t> root(last.begin(), last.begin() + i);
        double spurTime = nowS;
        if (!root.empty() && !EarliestArrival(root, nowS, spurTime))
        {
            continue;
        }
        const uint32_t spurNode = root.empty() ? m_local : m_plan[root.back()].to;
        std::set<uint32_t> bannedC;
        for (const auto& r : A)
        {
            if (r.contacts.size() > i && std::equal(root.begin(), root.end(), r.contacts.begin()))
            {
                bannedC.insert(r.contacts[i]);
            }
        }
        std::set<uint32_t> bannedN = bannedNodes;
        for (uint32_t c : root)
        {
            bannedN.insert(m_plan[c].from);
        }
        bannedN.erase(spurNode);
        std::vector<uint32_t> spur;
        if (!ShortestPath(spurNode, spurTime, dest, bannedC, bannedN, spur))
        {
            continue;
        }
        SabrRoute cand;
        cand.contacts = root;
        cand.contacts.insert(cand.contacts.end(), spur.begin(), spur.end());
        bool known = false;
        for (const auto& r : A)
        {
            known = known || r.contacts == cand.contacts;
        }
        for (const auto& r : B)
        {
            known = known || r.contacts == cand.contacts;
        }
        if (!known)
        {
            FillRouteMetrics(cand, nowS);
            B.push_back(cand);
        }
    }
    if (B.empty())
    {
        return false;
    }
    auto best = std::min_element(B.begin(), B.end(), [](const SabrRoute& a, const SabrRoute& b) {
        return a.bestCaseDeliveryS < b.bestCaseDeliveryS;
    });
    A.push_back(*best);
    B.erase(best);
    return true;
}

std::vector<SaginSabrRouter::Cand>
SaginSabrRouter::Evaluate(uint32_t dest,
                          double nowS,
                          double evcBits,
                          double expiryS,
                          const std::set<uint32_t>& excluded,
                          bool noFragment,
                          uint8_t priority)
{
    std::vector<Cand> cands;
    auto& list = m_routes[dest];
    for (size_t ri = 0; ri < list.size(); ++ri)
    {
        SabrRoute& r = list[ri];
        FillRouteMetrics(r, nowS); // times move with the current time
        // a) best-case delivery after expiration
        if (r.bestCaseDeliveryS > expiryS)
        {
            continue;
        }
        // b) entry node excluded
        if (excluded.count(r.entryNode))
        {
            continue;
        }
        // c) a contact back to X
        bool toX = false;
        for (uint32_t c : r.contacts)
        {
            toX = toX || (m_plan[c].to == m_local && dest != m_local);
        }
        if (toX)
        {
            continue;
        }
        // 3.2.6.2 earliest transmission opportunity. The applicable backlog
        // is what is queued for the entry node at this priority or higher.
        const SabrContact& c0 = m_plan[r.contacts.front()];
        const double adjStart = std::max(c0.startS, nowS);
        const double backlog = m_prioBacklog ? m_prioBacklog(r.entryNode, priority)
                                             : (m_backlog ? m_backlog(r.entryNode) : 0.0);
        double relief = 0.0;
        for (uint32_t k = 0; k < m_plan.size(); ++k)
        {
            const SabrContact& p = m_plan[k];
            if (!m_deleted[k] && p.from == c0.from && p.to == c0.to && p.startS < c0.startS &&
                p.endS > nowS)
            {
                relief += p.rateBps * (p.endS - std::max(p.startS, nowS));
            }
        }
        const double residual = std::max(0.0, backlog - relief);
        const double eto = adjStart + (c0.rateBps > 0 ? residual / c0.rateBps : 0.0);
        // d) ETO after the end of the initial contact
        if (eto > c0.endS)
        {
            continue;
        }
        // 3.2.6.3 to 3.2.6.7: byte times along the route.
        const size_t n = r.contacts.size();
        std::vector<double> fbtt(n);
        double lbat = 0.0;
        for (size_t i = 0; i < n; ++i)
        {
            const SabrContact& k = m_plan[r.contacts[i]];
            fbtt[i] = (i == 0) ? eto : std::max(k.startS, lbat);
            const double lbtt = fbtt[i] + (k.rateBps > 0 ? evcBits / k.rateBps : 0.0);
            lbat = lbtt + k.owltS + m_owltMarginS;
        }
        const double pbat = lbat;
        // e) projected arrival after expiration
        if (pbat > expiryS)
        {
            continue;
        }
        // 3.2.6.8.5 to 3.2.6.8.10: effective times, EVL at this priority, RVL.
        double rvl = std::numeric_limits<double>::infinity();
        for (size_t i = 0; i < n; ++i)
        {
            const uint32_t ci = r.contacts[i];
            double stop = m_plan[ci].endS;
            for (size_t j = i + 1; j < n; ++j)
            {
                stop = std::min(stop, m_plan[r.contacts[j]].endS);
            }
            const double dur = stop - fbtt[i];
            const double evl = std::min(m_mtv[ci][priority], m_plan[ci].rateBps * dur);
            rvl = std::min(rvl, evl);
        }
        // f) depleted route; g) no fragmentation and RVL below the EVC
        if (rvl <= 0.0 || (noFragment && rvl < evcBits))
        {
            continue;
        }
        cands.push_back({ri, pbat, eto, rvl}); // h)
    }
    return cands;
}

bool
SaginSabrRouter::Better(uint32_t dest, const Cand& a, const Cand& b) const
{
    // 3.2.8.1.4 a): earliest PBAT, fewest contacts, latest termination,
    // smallest entry node number.
    const auto& list = m_routes.at(dest);
    const SabrRoute& ra = list[a.idx];
    const SabrRoute& rb = list[b.idx];
    if (a.pbat != b.pbat)
    {
        return a.pbat < b.pbat;
    }
    if (ra.contacts.size() != rb.contacts.size())
    {
        return ra.contacts.size() < rb.contacts.size();
    }
    if (ra.terminationS != rb.terminationS)
    {
        return ra.terminationS > rb.terminationS;
    }
    return ra.entryNode < rb.entryNode;
}

SabrDecision
SaginSabrRouter::MakeDecision(uint32_t dest, const Cand& c) const
{
    SabrDecision out;
    out.found = true;
    out.route = m_routes.at(dest)[c.idx];
    out.entryNode = out.route.entryNode;
    out.pbatS = c.pbat;
    out.etoS = c.eto;
    out.rvlBits = c.rvl;
    return out;
}

SabrDecision
SaginSabrRouter::Route(uint32_t dest,
                       double nowS,
                       double evcBits,
                       double expiryS,
                       const std::set<uint32_t>& excluded,
                       bool noFragment,
                       bool reserve,
                       uint8_t priority)
{
    SabrDecision out;
    priority = std::min<uint8_t>(priority, SABR_EXPEDITED);
    Prune(nowS);

    // 3.2.2: CGR applies only if some contact transmits to D.
    bool anyToD = false;
    for (uint32_t i = 0; i < m_plan.size() && !anyToD; ++i)
    {
        anyToD = !m_deleted[i] && m_plan[i].to == dest;
    }
    if (!anyToD)
    {
        ++m_noRoute;
        return out;
    }

    auto cands = Evaluate(dest, nowS, evcBits, expiryS, excluded, noFragment, priority);
    // 3.2.6.9.1: compute the next best route only while there is no candidate.
    while (cands.empty() && m_routes[dest].size() < m_maxRoutes && NextRoute(dest, nowS))
    {
        cands = Evaluate(dest, nowS, evcBits, expiryS, excluded, noFragment, priority);
    }
    if (cands.empty())
    {
        out.routesComputed = static_cast<uint32_t>(m_routes[dest].size());
        ++m_noRoute; // 3.2.7: CGR is not used for this bundle
        return out;
    }
    const Cand best = *std::min_element(cands.begin(), cands.end(), [&](const Cand& a, const Cand& b) {
        return Better(dest, a, b);
    });
    out = MakeDecision(dest, best);
    out.routesComputed = static_cast<uint32_t>(m_routes[dest].size());
    out.candidates = static_cast<uint32_t>(cands.size());
    if (reserve)
    {
        Reserve(out.route, evcBits, priority);
    }
    return out;
}

std::vector<SabrDecision>
SaginSabrRouter::RouteCritical(uint32_t dest,
                               double nowS,
                               double evcBits,
                               double expiryS,
                               const std::set<uint32_t>& excluded,
                               bool noFragment,
                               bool reserve,
                               uint8_t priority)
{
    std::vector<SabrDecision> out;
    priority = std::min<uint8_t>(priority, SABR_EXPEDITED);
    Prune(nowS);
    // 3.2.6.9.2: for a critical bundle the route list is completed (up to the
    // implementation cap), not grown only until a candidate exists.
    while (m_routes[dest].size() < m_maxRoutes && NextRoute(dest, nowS))
    {
    }
    auto cands = Evaluate(dest, nowS, evcBits, expiryS, excluded, noFragment, priority);
    if (cands.empty())
    {
        ++m_noRoute;
        return out;
    }
    std::sort(cands.begin(), cands.end(), [&](const Cand& a, const Cand& b) { return Better(dest, a, b); });
    // 3.2.8.1.3: one copy to the entry node of every candidate route; the
    // best candidate of each entry node carries it.
    std::set<uint32_t> entries;
    for (const auto& c : cands)
    {
        const SabrRoute& r = m_routes[dest][c.idx];
        if (!entries.insert(r.entryNode).second)
        {
            continue;
        }
        SabrDecision d = MakeDecision(dest, c);
        d.routesComputed = static_cast<uint32_t>(m_routes[dest].size());
        d.candidates = static_cast<uint32_t>(cands.size());
        out.push_back(d);
    }
    if (reserve)
    {
        for (const auto& d : out)
        {
            Reserve(d.route, evcBits, priority);
        }
    }
    return out;
}

void
SaginSabrRouter::Reserve(const SabrRoute& r, double evcBits, uint8_t priority)
{
    // 3.2.8.1.2 with 3.2.6.8: enqueueing consumes the EVC on every contact of
    // the route, at the bundle's priority and every lower level.
    priority = std::min<uint8_t>(priority, SABR_EXPEDITED);
    for (uint32_t c : r.contacts)
    {
        if (c < m_mtv.size())
        {
            for (uint8_t q = 0; q <= priority; ++q)
            {
                m_mtv[c][q] = std::max(0.0, m_mtv[c][q] - evcBits);
            }
        }
    }
}

} // namespace ntnsagin
} // namespace ns3
