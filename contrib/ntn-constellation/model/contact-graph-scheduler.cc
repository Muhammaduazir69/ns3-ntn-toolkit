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
                                          MakeDoubleChecker<double>(0.0))
                            .AddAttribute("IslMinTangentAltM",
                                          "CON-3: minimum tangent height (m) above the ellipsoid "
                                          "an inter-satellite link must clear. The limb test used "
                                          "to accept a ray grazing the surface at exactly one "
                                          "Earth radius, which crosses the full depth of the "
                                          "atmosphere twice and is not a usable crosslink. "
                                          "80 km clears the mesosphere. 0 restores the grazing "
                                          "behaviour.",
                                          DoubleValue(80.0e3),
                                          MakeDoubleAccessor(&ContactGraphScheduler::m_islMinTangentAltM),
                                          MakeDoubleChecker<double>(0.0));
    return tid;
}

bool
ContactGraphScheduler::IsLimbClear(const Vector& a, const Vector& b, double minTangentAltM)
{
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    const double dz = b.z - a.z;
    const double seg2 = dx * dx + dy * dy + dz * dz;
    if (seg2 <= 0.0)
    {
        return true; // coincident endpoints: nothing between them
    }
    // Parameter of closest approach to the origin along a->b.
    const double tt = -(a.x * dx + a.y * dy + a.z * dz) / seg2;
    if (tt <= 0.0 || tt >= 1.0)
    {
        // The closest point is outside the segment, so the chord does not pass
        // the limb between the endpoints.
        return true;
    }
    const double cx = a.x + tt * dx;
    const double cy = a.y + tt * dy;
    const double cz = a.z + tt * dz;
    const double cr = std::sqrt(cx * cx + cy * cy + cz * cz);
    // CON-3: a MARGIN above the ellipsoid, not a graze. `cr >= kEarthRadiusM`
    // accepted a ray skimming the surface, which crosses the full depth of the
    // atmosphere twice and is not a usable crosslink at any wavelength this
    // toolkit models.
    return cr >= kEarthRadiusM + minTangentAltM;
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
            // SAGIN-1: the range is needed on every tick a link is up, not
            // only on the transition, so hoist it out of the transition test.
            Vector p = satIt.second->GetEcefPosition();
            // Range to GS via ECEF.
            const double lat = gsIt.second.lat_deg * M_PI / 180.0;
            const double lon = gsIt.second.lon_deg * M_PI / 180.0;
            const double cosLat = std::cos(lat);
            const double sinLat = std::sin(lat);
            // WGS-84 ellipsoid surface (gap B2: was a sphere at the
            // equatorial radius). N = a / sqrt(1 - e^2 sin^2 lat).
            const double Ngs =
                kEarthRadiusM / std::sqrt(1.0 - kWgs84E2 * sinLat * sinLat);
            const Vector gs(Ngs * cosLat * std::cos(lon),
                             Ngs * cosLat * std::sin(lon),
                             Ngs * (1.0 - kWgs84E2) * sinLat);
            const double dx = p.x - gs.x;
            const double dy = p.y - gs.y;
            const double dz = p.z - gs.z;
            const double range = std::sqrt(dx * dx + dy * dy + dz * dz);
            ContactEvent ev{
                t, satIt.first, gsIt.first, false, visible, range, elev};
            if (visible != prev)
            {
                m_gslState[key] = visible;
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
            else if (visible)
            {
                // Same contact, new geometry.
                m_contactUpdate(ev);
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
                // Earth-occultation (limb) test (gap B1): an ISL is blocked if
                // the segment between the two satellites passes within one
                // Earth radius of geocentre — previously a pure range gate
                // declared a link THROUGH the planet. Closest-approach of the
                // segment to the origin; endpoints are satellites (above the
                // surface), so a blocked link only occurs when the closest
                // point lies strictly between them.
                const bool losClear = IsLimbClear(pa, pb, m_islMinTangentAltM);
                const bool inRange = (range <= m_maxIslRangeM) && losClear;
                const std::pair<uint32_t, uint32_t> key{a->first, b->first};
                auto stateIt = m_islState.find(key);
                const bool prev =
                    (stateIt == m_islState.end()) ? false : stateIt->second;
                ContactEvent ev{t,           a->first, b->first,
                                 true,       inRange,  range,
                                 0.0};
                if (inRange != prev)
                {
                    m_islState[key] = inRange;
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
                else if (inRange)
                {
                    // SAGIN-1: ISL range changes continuously as the two
                    // satellites move relative to each other, so an established
                    // link still needs its geometry republished.
                    m_contactUpdate(ev);
                }
            }
        }
    }

    m_tickEvent =
        Simulator::Schedule(m_dt, &ContactGraphScheduler::Tick, this);
}

} // namespace ntncon
} // namespace ns3
