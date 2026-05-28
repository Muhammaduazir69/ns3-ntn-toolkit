/*
 * SPDX-License-Identifier: GPL-2.0-only
 * Copyright (c) 2026 Muhammad Uzair (ns3-ntn-toolkit, Workstream W5)
 */
#include "multi-layer-router.h"

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
    static TypeId tid = TypeId("ns3::MultiLayerRouter")
                           .SetParent<Object>()
                           .SetGroupName("NtnSagin")
                           .AddConstructor<MultiLayerRouter>();
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
    // Default: greedy max-elevation.
    return c.elevationDeg;
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
    for (uint8_t l = 1; l <= 3; ++l)
    {
        const auto& candidates = m_layers[l];
        if (candidates.empty())
        {
            // Allow skipping a layer if higher layer has nodes.
            continue;
        }
        Ptr<MobilityModel> best;
        double bestScore = -std::numeric_limits<double>::infinity();
        double bestEl = -90.0;
        double bestRange = 0.0;
        Vector observerPos = prev->GetPosition();
        for (std::size_t i = 0; i < candidates.size(); ++i)
        {
            const auto& cand = candidates[i];
            const Vector candPos = cand->GetPosition();
            const double el = ElevationDeg(observerPos, candPos);
            Vector d(candPos.x - observerPos.x,
                      candPos.y - observerPos.y,
                      candPos.z - observerPos.z);
            const double range =
                std::sqrt(d.x * d.x + d.y * d.y + d.z * d.z);

            SaginHopCandidate hc{
                static_cast<SaginLayer>(l),
                cand,
                el,
                range,
                prevLayer,
                prev,
                observerPos,
                i,
                m_observation};
            const double score = Score(hc);
            ++m_scored;
            if (score > bestScore)
            {
                bestScore = score;
                bestEl = el;
                bestRange = range;
                best = cand;
            }
        }
        if (!best)
        {
            break;
        }
        path.push_back(
            {static_cast<SaginLayer>(l), best, bestEl, bestRange});
        prev = best;
        prevLayer = static_cast<SaginLayer>(l);
    }
    return path;
}

} // namespace ns3
