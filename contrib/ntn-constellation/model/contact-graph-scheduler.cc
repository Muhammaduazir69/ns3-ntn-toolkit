/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#include "contact-graph-scheduler.h"

#include "ns3/double.h"
#include "ns3/log.h"
#include "ns3/simulator.h"

#include <cmath>

namespace ns3
{
namespace ntncon
{

NS_LOG_COMPONENT_DEFINE("ContactGraphScheduler");
NS_OBJECT_ENSURE_REGISTERED(ContactGraphScheduler);

TypeId
ContactGraphScheduler::GetTypeId()
{
    static TypeId tid = TypeId("ns3::ntncon::ContactGraphScheduler")
                            .SetParent<Object>()
                            .SetGroupName("NtnConstellation")
                            .AddConstructor<ContactGraphScheduler>()
                            .AddAttribute("GateHysteresisDeg",
                                          "GSL gate hysteresis (deg): a contact comes UP at "
                                          "MinElevationDeg and goes DOWN only below "
                                          "MinElevationDeg - hysteresis, so the link does not "
                                          "flap while the elevation hovers at the threshold. "
                                          "0 restores the legacy single-threshold gate.",
                                          DoubleValue(2.0),
                                          MakeDoubleAccessor(&ContactGraphScheduler::m_gateHysteresisDeg),
                                          MakeDoubleChecker<double>(0.0));
    return tid;
}

ContactGraphScheduler::ContactGraphScheduler() = default;

void
ContactGraphScheduler::RegisterSatellite(uint32_t id,
                                           Ptr<Sgp4MobilityModel> sat)
{
    m_sats[id] = sat;
}

void
ContactGraphScheduler::RegisterGroundStation(uint32_t id,
                                               double lat_deg,
                                               double lon_deg)
{
    m_gss[id] = {lat_deg, lon_deg};
}

void
ContactGraphScheduler::Start()
{
    m_running = true;
    m_tickEvent = Simulator::Schedule(MilliSeconds(0),
                                       &ContactGraphScheduler::Tick,
                                       this);
}

void
ContactGraphScheduler::Stop()
{
    m_running = false;
    if (m_tickEvent.IsRunning())
    {
        Simulator::Cancel(m_tickEvent);
    }
}

size_t
ContactGraphScheduler::NumActiveGsl() const
{
    size_t n = 0;
    for (const auto& kv : m_gslState)
    {
        if (kv.second)
            ++n;
    }
    return n;
}

size_t
ContactGraphScheduler::NumActiveIsl() const
{
    size_t n = 0;
    for (const auto& kv : m_islState)
    {
        if (kv.second)
            ++n;
    }
    return n;
}

void
ContactGraphScheduler::Tick()
{
    if (!m_running)
    {
        return;
    }
    const double t = Simulator::Now().GetSeconds();

    // GSL visibility.
    for (auto& gsIt : m_gss)
    {
        for (auto& satIt : m_sats)
        {
            const double elev =
                satIt.second->GetElevationDeg(gsIt.second.lat_deg,
                                              gsIt.second.lon_deg);
            const std::pair<uint32_t, uint32_t> key{satIt.first, gsIt.first};
            auto stateIt = m_gslState.find(key);
            const bool prev =
                (stateIt == m_gslState.end()) ? false : stateIt->second;
            // Hysteresis gate: UP at MinElevationDeg, DOWN only below
            // MinElevationDeg - GateHysteresisDeg (anti-flapping).
            const bool visible =
                prev ? (elev >= m_minElevDeg - m_gateHysteresisDeg)
                     : (elev >= m_minElevDeg);
            if (visible != prev)
            {
                m_gslState[key] = visible;
                Vector p = satIt.second->GetEcefPosition();
                // Range to GS via ECEF.
                const double lat = gsIt.second.lat_deg * M_PI / 180.0;
                const double lon = gsIt.second.lon_deg * M_PI / 180.0;
                const double cosLat = std::cos(lat);
                const Vector gs(kEarthRadiusM * cosLat * std::cos(lon),
                                 kEarthRadiusM * cosLat * std::sin(lon),
                                 kEarthRadiusM * std::sin(lat));
                const double dx = p.x - gs.x;
                const double dy = p.y - gs.y;
                const double dz = p.z - gs.z;
                const double range = std::sqrt(dx * dx + dy * dy + dz * dz);
                ContactEvent ev{
                    t, satIt.first, gsIt.first, false, visible, range, elev};
                if (visible)
                {
                    ++m_gslUp;
                    m_contactUp(ev);
                }
                else
                {
                    ++m_gslDown;
                    m_contactDown(ev);
                }
            }
        }
    }

    // ISL visibility (sat pair Euclidean range).
    if (m_sats.size() >= 2)
    {
        for (auto a = m_sats.begin(); a != m_sats.end(); ++a)
        {
            auto b = a;
            ++b;
            for (; b != m_sats.end(); ++b)
            {
                Vector pa = a->second->GetEcefPosition();
                Vector pb = b->second->GetEcefPosition();
                const double dx = pa.x - pb.x;
                const double dy = pa.y - pb.y;
                const double dz = pa.z - pb.z;
                const double range = std::sqrt(dx * dx + dy * dy + dz * dz);
                const bool inRange = range <= m_maxIslRangeM;
                const std::pair<uint32_t, uint32_t> key{a->first, b->first};
                auto stateIt = m_islState.find(key);
                const bool prev =
                    (stateIt == m_islState.end()) ? false : stateIt->second;
                if (inRange != prev)
                {
                    m_islState[key] = inRange;
                    ContactEvent ev{t,           a->first, b->first,
                                     true,       inRange,  range,
                                     0.0};
                    if (inRange)
                    {
                        ++m_islUp;
                        m_contactUp(ev);
                    }
                    else
                    {
                        ++m_islDown;
                        m_contactDown(ev);
                    }
                }
            }
        }
    }

    m_tickEvent =
        Simulator::Schedule(m_dt, &ContactGraphScheduler::Tick, this);
}

} // namespace ntncon
} // namespace ns3
