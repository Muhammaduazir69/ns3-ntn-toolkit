/*
 * SPDX-License-Identifier: GPL-2.0-only
 * Copyright (c) 2026 Muhammad Uzair (ns3-ntn-toolkit, Workstream W5)
 */
#include "multi-layer-router.h"

#include "ns3/log.h"

#include <cmath>

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
    std::vector<SaginHop> path;
    path.push_back({SaginLayer::Ground, source, 90.0, 0.0});

    Ptr<MobilityModel> prev = source;
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
        double bestEl = -90.0;
        double bestRange = 0.0;
        Vector observerPos = prev->GetPosition();
        for (auto& cand : candidates)
        {
            double el = ElevationDeg(observerPos, cand->GetPosition());
            if (el > bestEl)
            {
                bestEl = el;
                best = cand;
                Vector d = cand->GetPosition();
                d.x -= observerPos.x; d.y -= observerPos.y; d.z -= observerPos.z;
                bestRange = std::sqrt(d.x * d.x + d.y * d.y + d.z * d.z);
            }
        }
        if (!best)
        {
            break;
        }
        path.push_back({static_cast<SaginLayer>(l), best, bestEl, bestRange});
        prev = best;
    }
    return path;
}

} // namespace ns3
