/*
 * SPDX-License-Identifier: GPL-2.0-only
 * Copyright (c) 2026 Muhammad Uzair (ns3-ntn-toolkit, Workstream W5)
 */
#include "multi-layer-router.h"

#include "ns3/double.h"
#include "ns3/log.h"

#include <cmath>
#include <limits>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("MultiLayerRouter");
NS_OBJECT_ENSURE_REGISTERED(MultiLayerRouter);

TypeId
MultiLayerRouter::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::MultiLayerRouter")
            .SetParent<Object>()
            .SetGroupName("NtnSagin")
            .AddConstructor<MultiLayerRouter>()
            .AddAttribute("MinElevationDeg",
                          "Elevation floor (deg) below which the default "
                          "max-elevation scorer treats a candidate as "
                          "infeasible (returns -inf), so Route() truncates "
                          "the path rather than selecting a below-horizon hop.",
                          DoubleValue(0.0),
                          MakeDoubleAccessor(&MultiLayerRouter::m_minElevationDeg),
                          MakeDoubleChecker<double>());
    return tid;
}

MultiLayerRouter::MultiLayerRouter() = default;

void
MultiLayerRouter::AddNode(SaginLayer layer, Ptr<MobilityModel> node)
{
    m_layers[static_cast<uint8_t>(layer)].push_back(node);
}

void
MultiLayerRouter::SetScorer(SaginScoreCallback scorer)
{
    m_globalScorer = std::move(scorer);
    for (auto& s : m_layerScorer)
    {
        s = nullptr;
    }
}

void
MultiLayerRouter::SetLayerScorer(SaginLayer layer, SaginScoreCallback scorer)
{
    m_layerScorer[static_cast<uint8_t>(layer)] = std::move(scorer);
}

void
MultiLayerRouter::ClearScorer()
{
    m_globalScorer = nullptr;
    for (auto& s : m_layerScorer)
    {
        s = nullptr;
    }
}

double
MultiLayerRouter::Score(const SaginHopCandidate& c) const
{
    // Per-layer scorer wins.
    const auto& layerCb = m_layerScorer[static_cast<uint8_t>(c.layer)];
    if (layerCb)
    {
        return layerCb(c);
    }
    if (m_globalScorer)
    {
        return m_globalScorer(c);
    }
    // Default: greedy max-elevation with an honest feasibility gate. A hop
    // whose elevation from the previous node is below the horizon floor is
    // physically unusable, so it is handed -inf and never selected; when a
    // whole layer is below the floor Route() truncates there.
    if (c.elevationDeg < m_minElevationDeg)
    {
        return -std::numeric_limits<double>::infinity();
    }
    if (!m_congestionAware || !c.haveLoad)
    {
        // SAGIN-2: with no load source, score on elevation alone.
        //
        // Note honestly that the !haveLoad half of this test is CLARITY, not
        // distinct behaviour: queueOccupancy defaults to 0, so removing it
        // computes elevationDeg - 0 and lands in the same place. It is kept
        // because it states the intent - an unknown load is not an idle one -
        // and because it stops being a no-op the moment the penalty gains a
        // constant term or the default changes. A revert of this line does not
        // fail any test, and that is recorded rather than papered over.
        return c.elevationDeg;
    }
    if (c.queueOccupancy >= m_rejectAboveOccupancy)
    {
        // A link this close to full is unusable for the same practical reason a
        // below-horizon hop is: the packets the router sends there will be
        // dropped by the TxQueue, not carried.
        ++m_congestionRejections;
        return -std::numeric_limits<double>::infinity();
    }
    return c.elevationDeg - m_congestionWeight * 90.0 * c.queueOccupancy;
}

void
MultiLayerRouter::SetCongestionAware(bool on, double congestionWeight,
                                     double rejectAboveOccupancy)
{
    m_congestionAware = on;
    m_congestionWeight = congestionWeight;
    m_rejectAboveOccupancy = rejectAboveOccupancy;
}

std::size_t
MultiLayerRouter::NodeCount(SaginLayer layer) const
{
    return m_layers[static_cast<uint8_t>(layer)].size();
}

double
MultiLayerRouter::ElevationDeg(const Vector& observer, const Vector& target)
{
    Vector d(target.x - observer.x,
             target.y - observer.y,
             target.z - observer.z);
    double horiz = std::sqrt(d.x * d.x + d.y * d.y);
    if (horiz < 1e-3 && std::abs(d.z) < 1e-3)
    {
        return 90.0;  // colocated
    }
    return std::atan2(d.z, horiz) * 180.0 / M_PI;
}

std::vector<SaginHop>
MultiLayerRouter::Route(Ptr<MobilityModel> source) const
{
    ++m_routes;
    std::vector<SaginHop> path;
    path.push_back({SaginLayer::Ground, source, 90.0, 0.0});

    Ptr<MobilityModel> prev = source;
    SaginLayer prevLayer = SaginLayer::Ground;
    // Try each higher layer in order; stop if a layer is empty.
    // SAGIN-9: which layers this hop may consider.
    //
    // This loop used to advance one layer per iteration and `continue` only when
    // a layer was EMPTY, so a populated UAV layer was mandatory transit for
    // every ground source, however much better a direct ground-to-LEO link was.
    // With layer skipping enabled a hop weighs every remaining layer at once and
    // the sequence follows the score; nextLayer then resumes ABOVE whatever it
    // chose, so a skipped layer is never revisited and the path always climbs.
    uint8_t nextLayer = 1;
    while (nextLayer <= 3)
    {
        const uint8_t lastLayer = m_allowLayerSkip ? 3 : nextLayer;
        std::vector<std::pair<uint8_t, Ptr<MobilityModel>>> pool;
        for (uint8_t ll = nextLayer; ll <= lastLayer; ++ll)
        {
            for (const auto& c : m_layers[ll])
            {
                pool.emplace_back(ll, c);
            }
        }
        if (pool.empty())
        {
            if (!m_allowLayerSkip)
            {
                ++nextLayer; // this layer is empty; try the next
                continue;
            }
            break; // nothing left anywhere above
        }
        Ptr<MobilityModel> best;
        uint8_t bestLayer = nextLayer;
        double bestScore = -std::numeric_limits<double>::infinity();
        double bestEl = -90.0;
        double bestRange = 0.0;
        Vector observerPos = prev->GetPosition();
        for (std::size_t i = 0; i < pool.size(); ++i)
        {
            const uint8_t candLayer = pool[i].first;
            const auto& cand = pool[i].second;
            const Vector candPos = cand->GetPosition();
            const double el = ElevationDeg(observerPos, candPos);
            Vector d(candPos.x - observerPos.x,
                      candPos.y - observerPos.y,
                      candPos.z - observerPos.z);
            const double range =
                std::sqrt(d.x * d.x + d.y * d.y + d.z * d.z);

            SaginHopCandidate hc{
                static_cast<SaginLayer>(candLayer),
                cand,
                el,
                range,
                prevLayer,
                prev,
                observerPos,
                i,
                m_observation};

            // SAGIN-2: ask for the live load of the link this hop would use.
            // Absent a source, haveLoad stays false and Score() falls back to
            // elevation rather than assuming the link is idle.
            if (m_loadSource)
            {
                double capBps = 0.0;
                double occ = 0.0;
                if (m_loadSource(prev, cand, capBps, occ))
                {
                    hc.capacityBps = capBps;
                    hc.queueOccupancy = std::max(0.0, std::min(1.0, occ));
                    hc.haveLoad = true;
                }
            }

            const double score = Score(hc);
            ++m_scored;
            if (score > bestScore)
            {
                bestScore = score;
                bestEl = el;
                bestRange = range;
                best = cand;
                bestLayer = candLayer;
            }
        }
        if (!best)
        {
            break;
        }
        path.push_back(
            {static_cast<SaginLayer>(bestLayer), best, bestEl, bestRange});
        prev = best;
        prevLayer = static_cast<SaginLayer>(bestLayer);
        nextLayer = static_cast<uint8_t>(bestLayer + 1);
    }
    return path;
}

} // namespace ns3
