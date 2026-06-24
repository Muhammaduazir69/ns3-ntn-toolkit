/*
 * SPDX-License-Identifier: GPL-2.0-only
 * Copyright (c) 2026 Muhammad Uzair (ns3-ntn-toolkit, Workstream W7)
 */
#include "v2x-leo-relay.h"

#include "ns3/double.h"
#include "ns3/log.h"

#include <cmath>

namespace ns3
{
namespace ntnv2x
{

NS_LOG_COMPONENT_DEFINE("V2xLeoRelay");
NS_OBJECT_ENSURE_REGISTERED(V2xLeoRelay);

TypeId
V2xLeoRelay::GetTypeId()
{
    static TypeId tid = TypeId("ns3::ntnv2x::V2xLeoRelay")
                           .SetParent<Object>()
                           .SetGroupName("NtnV2x")
                           .AddConstructor<V2xLeoRelay>();
    return tid;
}

V2xLeoRelay::V2xLeoRelay()
    : m_direct(CreateObject<V2xLeoDirect>())
{
}

void
V2xLeoRelay::SetMaxV2vRangeM(double m)
{
    m_maxV2vRangeM = m;
}

void
V2xLeoRelay::SetMinDirectSnrDb(double snrDb)
{
    m_minDirectSnrDb = snrDb;
}

void
V2xLeoRelay::SetSatellite(Ptr<MobilityModel> sat)
{
    m_sat = sat;
}

Ptr<MobilityModel>
V2xLeoRelay::GetSatellite() const
{
    return m_sat;
}

void
V2xLeoRelay::RegisterVehicle(const std::string& id, Ptr<MobilityModel> mob)
{
    m_vehicles[id] = mob;
}

void
V2xLeoRelay::SetVehicleBlockageDb(const std::string& id, double blockageDb)
{
    m_blockageDb[id] = blockageDb;
}

std::size_t
V2xLeoRelay::VehicleCount() const
{
    return m_vehicles.size();
}

std::vector<RelayDecision>
V2xLeoRelay::EvaluateAll() const
{
    std::vector<RelayDecision> out;
    if (!m_sat || m_vehicles.empty())
    {
        return out;
    }
    out.reserve(m_vehicles.size());

    // Pre-compute every vehicle's direct SNR, minus its NLOS blockage (a
    // shadowed vehicle has a worse direct link and is the one that must relay).
    std::map<std::string, double> directSnr;
    for (const auto& [id, mob] : m_vehicles)
    {
        auto lb = m_direct->Compute(mob, m_sat);
        double blockage = 0.0;
        auto bit = m_blockageDb.find(id);
        if (bit != m_blockageDb.end())
        {
            blockage = bit->second;
        }
        directSnr[id] = lb.snrDb - blockage;
    }

    for (const auto& [id, mob] : m_vehicles)
    {
        RelayDecision d;
        d.vehId = id;
        d.directSnrDb = directSnr[id];
        if (d.directSnrDb >= m_minDirectSnrDb)
        {
            d.directToLeo = true;
            out.push_back(d);
            continue;
        }
        // Find best peer within range whose direct link is good enough.
        Vector myPos = mob->GetPosition();
        std::string bestId;
        double bestSnr = -1e9;
        double bestRange = 0.0;
        for (const auto& [peerId, peerMob] : m_vehicles)
        {
            if (peerId == id)
                continue;
            Vector pp = peerMob->GetPosition();
            double dx = pp.x - myPos.x, dy = pp.y - myPos.y, dz = pp.z - myPos.z;
            double range = std::sqrt(dx * dx + dy * dy + dz * dz);
            if (range > m_maxV2vRangeM)
                continue;
            double snr = directSnr[peerId];
            if (snr < m_minDirectSnrDb)
                continue;
            if (snr > bestSnr)
            {
                bestSnr = snr;
                bestId = peerId;
                bestRange = range;
            }
        }
        d.directToLeo = false;
        d.relayPeerId = bestId;
        d.viaRelaySnrDb = bestSnr;
        d.v2vRangeM = bestRange;
        out.push_back(d);
    }
    return out;
}

} // namespace ntnv2x
} // namespace ns3
