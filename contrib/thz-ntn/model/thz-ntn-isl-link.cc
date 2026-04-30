/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-only
 * Author: Muhammad Uzair
 *
 * THz-NTN ISL Link Management Implementation
 */

#include "thz-ntn-isl-link.h"

#include <ns3/satellite-point-to-point-isl-net-device.h>

#include <ns3/double.h>
#include <ns3/log.h>
#include <ns3/string.h>
#include <ns3/uinteger.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <set>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("ThzNtnIslLink");
NS_OBJECT_ENSURE_REGISTERED(ThzNtnIslLink);

/// Maximum data rate for ISL link (Gbps).
static constexpr double MAX_ISL_DATA_RATE_GBPS = 17.3;

TypeId
ThzNtnIslLink::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::ThzNtnIslLink")
            .SetParent<Object>()
            .SetGroupName("ThzNtn")
            .AddConstructor<ThzNtnIslLink>()
            .AddAttribute("MaxLinkDistance",
                          "Maximum ISL distance in km",
                          DoubleValue(5000.0),
                          MakeDoubleAccessor(&ThzNtnIslLink::m_maxLinkDistance_km),
                          MakeDoubleChecker<double>(100.0, 50000.0))
            .AddAttribute("MinSnr",
                          "Minimum SNR for link viability in dB",
                          DoubleValue(5.0),
                          MakeDoubleAccessor(&ThzNtnIslLink::m_minSnr_dB),
                          MakeDoubleChecker<double>(-10.0, 50.0))
            .AddAttribute("LinkSetupTime",
                          "Link setup latency in ms",
                          DoubleValue(50.0),
                          MakeDoubleAccessor(&ThzNtnIslLink::m_linkSetupTime_ms),
                          MakeDoubleChecker<double>(0.0, 10000.0))
            .AddAttribute("MaxDataRate",
                          "Maximum ISL data rate in Gbps",
                          DoubleValue(MAX_ISL_DATA_RATE_GBPS),
                          MakeDoubleAccessor(&ThzNtnIslLink::m_maxDataRate_Gbps),
                          MakeDoubleChecker<double>(0.1, 100.0))
            .AddAttribute("RoutingAlgorithm",
                          "Routing algorithm: SHORTEST_PATH, MINIMUM_DELAY, LOAD_BALANCED",
                          StringValue("SHORTEST_PATH"),
                          MakeStringAccessor(&ThzNtnIslLink::m_routingAlgoStr),
                          MakeStringChecker());
    return tid;
}

ThzNtnIslLink::ThzNtnIslLink()
    : m_maxLinkDistance_km(5000.0),
      m_minSnr_dB(5.0),
      m_linkSetupTime_ms(50.0),
      m_maxDataRate_Gbps(MAX_ISL_DATA_RATE_GBPS),
      m_routingAlgoStr("SHORTEST_PATH"),
      m_routingAlgo(ThzNtnRoutingAlgorithm::SHORTEST_PATH),
      m_islChannel(nullptr),
      m_nextLinkId(1)
{
    NS_LOG_FUNCTION(this);
}

ThzNtnIslLink::~ThzNtnIslLink()
{
    NS_LOG_FUNCTION(this);
}

void
ThzNtnIslLink::DoDispose()
{
    NS_LOG_FUNCTION(this);
    m_links.clear();
    m_satMobilities.clear();
    m_islDevices.clear();
    m_islChannel = nullptr;
    Object::DoDispose();
}

// ---------------------------------------------------------------------------
// Sub-model setters
// ---------------------------------------------------------------------------

void
ThzNtnIslLink::SetIslChannel(Ptr<ThzNtnIslChannel> channel)
{
    NS_LOG_FUNCTION(this << channel);
    m_islChannel = channel;
}

void
ThzNtnIslLink::SetIslNetDevice(uint32_t linkId, Ptr<PointToPointIslNetDevice> device)
{
    NS_LOG_FUNCTION(this << linkId << device);
    m_islDevices[linkId] = device;
}

// ---------------------------------------------------------------------------
// Topology management from actual MobilityModels
// ---------------------------------------------------------------------------

void
ThzNtnIslLink::UpdateTopology(const std::map<uint32_t, Ptr<MobilityModel>>& satMobilities)
{
    NS_LOG_FUNCTION(this);

    m_satMobilities = satMobilities;

    // Parse routing algorithm from string attribute
    m_routingAlgo = ParseRoutingAlgorithm(m_routingAlgoStr);

    // Update distances and tear down links that exceed maximum distance
    std::vector<uint32_t> toRemove;
    for (auto& pair : m_links)
    {
        auto& link = pair.second;
        auto srcIt = m_satMobilities.find(link.srcSatId);
        auto dstIt = m_satMobilities.find(link.dstSatId);

        if (srcIt == m_satMobilities.end() || dstIt == m_satMobilities.end())
        {
            toRemove.push_back(pair.first);
            continue;
        }

        // Actual distance from MobilityModel positions
        double distance_m = srcIt->second->GetDistanceFrom(dstIt->second);
        link.distance_km = distance_m / 1.0e3;

        if (link.distance_km > m_maxLinkDistance_km)
        {
            NS_LOG_INFO("Link " << link.linkId << " distance " << link.distance_km
                                << " km exceeds max; tearing down");
            toRemove.push_back(pair.first);
            continue;
        }

        // Update SNR and data rate from actual channel model
        if (m_islChannel)
        {
            link.snr_dB = m_islChannel->ComputeIslSnr_dB(srcIt->second, dstIt->second);
            link.dataRate_Gbps = SnrToDataRate_Gbps(link.snr_dB);

            if (link.snr_dB < m_minSnr_dB)
            {
                link.status = ThzNtnIslLinkStatus::DEGRADED;
                link.dataRate_Gbps = MIN_ISL_DATA_RATE_GBPS;
            }
            else if (link.status == ThzNtnIslLinkStatus::DEGRADED)
            {
                link.status = ThzNtnIslLinkStatus::UP;
            }
        }
    }

    for (auto id : toRemove)
    {
        m_links.erase(id);
    }

    NS_LOG_INFO("Topology updated: " << m_satMobilities.size() << " satellites, "
                                     << m_links.size() << " active links");
}

// ---------------------------------------------------------------------------
// Link lifecycle
// ---------------------------------------------------------------------------

bool
ThzNtnIslLink::EstablishLink(uint32_t srcSatId, uint32_t dstSatId)
{
    NS_LOG_FUNCTION(this << srcSatId << dstSatId);

    auto srcIt = m_satMobilities.find(srcSatId);
    auto dstIt = m_satMobilities.find(dstSatId);

    if (srcIt == m_satMobilities.end() || dstIt == m_satMobilities.end())
    {
        NS_LOG_WARN("Satellite MobilityModel not found for sat " << srcSatId
                    << " or sat " << dstSatId);
        return false;
    }

    // Check distance from actual MobilityModel positions
    double distance_m = srcIt->second->GetDistanceFrom(dstIt->second);
    double distance_km = distance_m / 1.0e3;

    if (distance_km > m_maxLinkDistance_km)
    {
        NS_LOG_WARN("ISL distance " << distance_km << " km exceeds maximum "
                                     << m_maxLinkDistance_km << " km");
        return false;
    }

    // Check for duplicate link
    for (const auto& pair : m_links)
    {
        const auto& link = pair.second;
        if ((link.srcSatId == srcSatId && link.dstSatId == dstSatId) ||
            (link.srcSatId == dstSatId && link.dstSatId == srcSatId))
        {
            if (link.status == ThzNtnIslLinkStatus::UP ||
                link.status == ThzNtnIslLinkStatus::ESTABLISHING)
            {
                NS_LOG_WARN("Link already exists between sat "
                            << srcSatId << " and sat " << dstSatId);
                return false;
            }
        }
    }

    ManagedLink ml;
    ml.linkId = m_nextLinkId++;
    ml.srcSatId = srcSatId;
    ml.dstSatId = dstSatId;
    ml.status = ThzNtnIslLinkStatus::ESTABLISHING;
    ml.distance_km = distance_km;

    // Compute SNR from actual channel model if available
    if (m_islChannel)
    {
        ml.snr_dB = m_islChannel->ComputeIslSnr_dB(srcIt->second, dstIt->second);
        ml.dataRate_Gbps = SnrToDataRate_Gbps(ml.snr_dB);
    }
    else
    {
        ml.snr_dB = 0.0;
        double distRatio = distance_km / m_maxLinkDistance_km;
        ml.dataRate_Gbps = MIN_ISL_DATA_RATE_GBPS +
                           (m_maxDataRate_Gbps - MIN_ISL_DATA_RATE_GBPS) * (1.0 - distRatio);
    }

    ml.dataRate_Gbps = std::min(ml.dataRate_Gbps, m_maxDataRate_Gbps);
    ml.dataRate_Gbps = std::max(ml.dataRate_Gbps, MIN_ISL_DATA_RATE_GBPS);
    ml.status = ThzNtnIslLinkStatus::UP;

    m_links[ml.linkId] = ml;
    NS_LOG_INFO("Established ISL link " << ml.linkId << " between sat " << srcSatId
                                        << " and sat " << dstSatId << " at "
                                        << ml.dataRate_Gbps << " Gbps");
    return true;
}

void
ThzNtnIslLink::TeardownLink(uint32_t linkId)
{
    NS_LOG_FUNCTION(this << linkId);

    auto it = m_links.find(linkId);
    if (it == m_links.end())
    {
        NS_LOG_WARN("Link " << linkId << " not found");
        return;
    }

    it->second.status = ThzNtnIslLinkStatus::TEARING_DOWN;
    NS_LOG_INFO("Tearing down ISL link " << linkId);
    it->second.status = ThzNtnIslLinkStatus::DOWN;
    m_links.erase(it);
}

ThzNtnIslLinkStatus
ThzNtnIslLink::GetLinkStatus(uint32_t linkId) const
{
    NS_LOG_FUNCTION(this << linkId);

    auto it = m_links.find(linkId);
    if (it == m_links.end())
    {
        return ThzNtnIslLinkStatus::DOWN;
    }
    return it->second.status;
}

double
ThzNtnIslLink::GetCurrentDataRate_Gbps(uint32_t linkId) const
{
    NS_LOG_FUNCTION(this << linkId);

    auto it = m_links.find(linkId);
    if (it == m_links.end())
    {
        NS_LOG_WARN("Link " << linkId << " not found, returning 0 Gbps");
        return 0.0;
    }
    return it->second.dataRate_Gbps;
}

void
ThzNtnIslLink::AdaptDataRate(uint32_t linkId)
{
    NS_LOG_FUNCTION(this << linkId);

    auto it = m_links.find(linkId);
    if (it == m_links.end())
    {
        NS_LOG_WARN("Link " << linkId << " not found");
        return;
    }

    // Compute actual SNR from MobilityModels via channel model
    auto srcIt = m_satMobilities.find(it->second.srcSatId);
    auto dstIt = m_satMobilities.find(it->second.dstSatId);

    if (srcIt == m_satMobilities.end() || dstIt == m_satMobilities.end() || !m_islChannel)
    {
        NS_LOG_WARN("Cannot adapt data rate: missing MobilityModel or channel model");
        return;
    }

    double currentSnrDb = m_islChannel->ComputeIslSnr_dB(srcIt->second, dstIt->second);
    it->second.snr_dB = currentSnrDb;

    if (currentSnrDb < m_minSnr_dB)
    {
        it->second.status = ThzNtnIslLinkStatus::DEGRADED;
        it->second.dataRate_Gbps = MIN_ISL_DATA_RATE_GBPS;
        NS_LOG_WARN("Link " << linkId << " SNR " << currentSnrDb
                             << " dB below minimum; link degraded");
        return;
    }

    double newRate = SnrToDataRate_Gbps(currentSnrDb);
    it->second.dataRate_Gbps = newRate;

    if (it->second.status == ThzNtnIslLinkStatus::DEGRADED)
    {
        it->second.status = ThzNtnIslLinkStatus::UP;
    }

    NS_LOG_INFO("Link " << linkId << " rate adapted to " << newRate
                         << " Gbps at SNR " << currentSnrDb << " dB");
}

// ---------------------------------------------------------------------------
// Routing
// ---------------------------------------------------------------------------

ThzNtnIslRoute
ThzNtnIslLink::ComputeRoute(uint32_t srcSatId,
                             uint32_t dstSatId,
                             ThzNtnRoutingAlgorithm algo) const
{
    NS_LOG_FUNCTION(this << srcSatId << dstSatId << static_cast<int>(algo));

    ThzNtnIslRoute route;
    route.totalDelay_ms = 0.0;
    route.totalCapacity_Gbps = 0.0;
    route.numHops = 0;

    if (srcSatId == dstSatId)
    {
        route.hops.push_back(srcSatId);
        route.numHops = 0;
        return route;
    }

    // Build adjacency from active links
    std::map<uint32_t, std::vector<std::tuple<uint32_t, double, double>>> adj;
    std::set<uint32_t> nodes;

    for (const auto& pair : m_links)
    {
        const auto& link = pair.second;
        if (link.status != ThzNtnIslLinkStatus::UP &&
            link.status != ThzNtnIslLinkStatus::DEGRADED)
        {
            continue;
        }
        adj[link.srcSatId].emplace_back(link.dstSatId, link.distance_km, link.dataRate_Gbps);
        adj[link.dstSatId].emplace_back(link.srcSatId, link.distance_km, link.dataRate_Gbps);
        nodes.insert(link.srcSatId);
        nodes.insert(link.dstSatId);
    }

    if (nodes.find(srcSatId) == nodes.end() || nodes.find(dstSatId) == nodes.end())
    {
        NS_LOG_WARN("Source or destination satellite not in topology");
        return route;
    }

    // Dijkstra with configurable edge weight
    std::map<uint32_t, double> dist;
    std::map<uint32_t, uint32_t> prev;
    std::map<uint32_t, double> bottleneck;

    for (auto n : nodes)
    {
        dist[n] = std::numeric_limits<double>::infinity();
        bottleneck[n] = std::numeric_limits<double>::infinity();
    }
    dist[srcSatId] = 0.0;

    using PqEntry = std::pair<double, uint32_t>;
    std::priority_queue<PqEntry, std::vector<PqEntry>, std::greater<PqEntry>> pq;
    pq.push({0.0, srcSatId});

    while (!pq.empty())
    {
        auto [curDist, curNode] = pq.top();
        pq.pop();

        if (curDist > dist[curNode])
        {
            continue;
        }

        if (curNode == dstSatId)
        {
            break;
        }

        auto adjIt = adj.find(curNode);
        if (adjIt == adj.end())
        {
            continue;
        }

        for (const auto& [neighbor, linkDist, linkRate] : adjIt->second)
        {
            double edgeWeight = 0.0;
            switch (algo)
            {
            case ThzNtnRoutingAlgorithm::SHORTEST_PATH:
                edgeWeight = 1.0;
                break;
            case ThzNtnRoutingAlgorithm::MINIMUM_DELAY:
                edgeWeight = (linkDist * 1.0e3) / SPEED_OF_LIGHT_M_S * 1.0e3;
                break;
            case ThzNtnRoutingAlgorithm::LOAD_BALANCED:
                edgeWeight = (linkRate > 0.0) ? (1.0 / linkRate) : 1.0e9;
                break;
            }

            double newDist = dist[curNode] + edgeWeight;
            if (newDist < dist[neighbor])
            {
                dist[neighbor] = newDist;
                prev[neighbor] = curNode;
                bottleneck[neighbor] = std::min(bottleneck[curNode], linkRate);
                pq.push({newDist, neighbor});
            }
        }
    }

    if (dist[dstSatId] == std::numeric_limits<double>::infinity())
    {
        NS_LOG_WARN("No route found from sat " << srcSatId << " to sat " << dstSatId);
        return route;
    }

    std::vector<uint32_t> path;
    for (uint32_t at = dstSatId; at != srcSatId; at = prev[at])
    {
        path.push_back(at);
    }
    path.push_back(srcSatId);
    std::reverse(path.begin(), path.end());

    route.hops = path;
    route.numHops = static_cast<uint32_t>(path.size() - 1);
    route.totalCapacity_Gbps = bottleneck[dstSatId];
    route.totalDelay_ms = ComputeEndToEndDelay_ms(route);

    NS_LOG_INFO("Route computed: " << route.numHops << " hops, "
                                   << route.totalDelay_ms << " ms delay, "
                                   << route.totalCapacity_Gbps << " Gbps bottleneck");
    return route;
}

double
ThzNtnIslLink::ComputeEndToEndDelay_ms(const ThzNtnIslRoute& route) const
{
    NS_LOG_FUNCTION(this);

    double totalDelay = 0.0;
    for (size_t i = 0; i + 1 < route.hops.size(); ++i)
    {
        auto itA = m_satMobilities.find(route.hops[i]);
        auto itB = m_satMobilities.find(route.hops[i + 1]);

        if (itA == m_satMobilities.end() || itB == m_satMobilities.end())
        {
            continue;
        }

        // Distance from actual MobilityModel positions
        double distance_m = itA->second->GetDistanceFrom(itB->second);
        // Propagation delay: distance / c, converted to ms
        totalDelay += (distance_m / SPEED_OF_LIGHT_M_S) * 1.0e3;
    }
    return totalDelay;
}

void
ThzNtnIslLink::SetRoutingAlgorithm(ThzNtnRoutingAlgorithm algo)
{
    NS_LOG_FUNCTION(this << static_cast<int>(algo));
    m_routingAlgo = algo;
}

std::vector<uint32_t>
ThzNtnIslLink::GetActiveLinkIds() const
{
    NS_LOG_FUNCTION(this);

    std::vector<uint32_t> active;
    for (const auto& pair : m_links)
    {
        if (pair.second.status == ThzNtnIslLinkStatus::UP ||
            pair.second.status == ThzNtnIslLinkStatus::DEGRADED)
        {
            active.push_back(pair.first);
        }
    }
    return active;
}

// ---------------------------------------------------------------------------
// Private helpers
// ---------------------------------------------------------------------------

double
ThzNtnIslLink::SnrToDataRate_Gbps(double snrDb) const
{
    struct SnrRateEntry
    {
        double snrThreshold_dB;
        double rate_Gbps;
    };

    static const SnrRateEntry table[] = {
        {5.0, 1.5},    // QPSK 1/4
        {8.0, 3.0},    // QPSK 1/2
        {11.0, 5.0},   // QPSK 3/4
        {14.0, 7.5},   // 8PSK 2/3
        {17.0, 10.0},  // 16QAM 1/2
        {20.0, 12.5},  // 16QAM 3/4
        {23.0, 15.0},  // 64QAM 2/3
        {26.0, 17.3},  // 64QAM 5/6
    };

    static const size_t tableSize = sizeof(table) / sizeof(table[0]);

    if (snrDb < table[0].snrThreshold_dB)
    {
        return MIN_ISL_DATA_RATE_GBPS;
    }

    for (size_t i = tableSize - 1; i > 0; --i)
    {
        if (snrDb >= table[i].snrThreshold_dB)
        {
            return std::min(table[i].rate_Gbps, m_maxDataRate_Gbps);
        }
    }

    return std::min(table[0].rate_Gbps, m_maxDataRate_Gbps);
}

ThzNtnRoutingAlgorithm
ThzNtnIslLink::ParseRoutingAlgorithm(const std::string& s)
{
    if (s == "MINIMUM_DELAY")
    {
        return ThzNtnRoutingAlgorithm::MINIMUM_DELAY;
    }
    if (s == "LOAD_BALANCED")
    {
        return ThzNtnRoutingAlgorithm::LOAD_BALANCED;
    }
    return ThzNtnRoutingAlgorithm::SHORTEST_PATH;
}

} // namespace ns3
