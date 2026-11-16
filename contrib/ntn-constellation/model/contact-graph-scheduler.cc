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
                                               double lon_deg,
                                               double alt_m)
{
    m_gss[id] = {lat_deg, lon_deg, alt_m};
}

void
ContactGraphScheduler::SetGslCandidates(const std::vector<std::pair<uint32_t, uint32_t>>& satGs)
{
    m_gslCand.clear();
    for (const auto& [s, gsId] : satGs)
    {
        const auto& g = m_gss.at(gsId);
        const double lat = g.lat_deg * M_PI / 180.0;
        const double lon = g.lon_deg * M_PI / 180.0;
        const double Ngs = kEarthRadiusM / std::sqrt(1.0 - kWgs84E2 * std::sin(lat) * std::sin(lat));
        GslCand gc;
        gc.sat = s;
        gc.gs = gsId;
        gc.mob = m_sats.at(s);
        gc.latDeg = g.lat_deg;
        gc.lonDeg = g.lon_deg;
        gc.altM = g.alt_m;
        gc.gsEcef = Vector((Ngs + g.alt_m) * std::cos(lat) * std::cos(lon),
                           (Ngs + g.alt_m) * std::cos(lat) * std::sin(lon),
                           (Ngs * (1.0 - kWgs84E2) + g.alt_m) * std::sin(lat));
        m_gslCand.push_back(gc);
    }
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

double
ContactGraphScheduler::ElevationDeg(const Vector& p, double latDeg, double lonDeg, double altM)
{
    const double lat = latDeg * M_PI / 180.0;
    const double lon = lonDeg * M_PI / 180.0;
    const double sinLat = std::sin(lat);
    const double Ngs = kEarthRadiusM / std::sqrt(1.0 - kWgs84E2 * sinLat * sinLat);
    const Vector gs((Ngs + altM) * std::cos(lat) * std::cos(lon),
                    (Ngs + altM) * std::cos(lat) * std::sin(lon),
                    (Ngs * (1.0 - kWgs84E2) + altM) * sinLat);
    // Local up is the ellipsoid normal (geodetic), not the geocentric radius.
    const Vector up(std::cos(lat) * std::cos(lon), std::cos(lat) * std::sin(lon), sinLat);
    const double dx = p.x - gs.x;
    const double dy = p.y - gs.y;
    const double dz = p.z - gs.z;
    const double rng = std::sqrt(dx * dx + dy * dy + dz * dz);
    return std::asin((dx * up.x + dy * up.y + dz * up.z) / rng) * 180.0 / M_PI;
}

bool
ContactGraphScheduler::ContactUpAt(uint32_t a, uint32_t b, bool isl, double offsetS) const
{
    Vector pa;
    Vector va;
    m_sats.at(a)->GetEcefStateAt(Seconds(offsetS), pa, va);
    if (!isl)
    {
        const auto& g = m_gss.at(b);
        return ElevationDeg(pa, g.lat_deg, g.lon_deg, g.alt_m) >=
               m_minElevDeg - m_gateHysteresisDeg;
    }
    Vector pb;
    Vector vb;
    m_sats.at(b)->GetEcefStateAt(Seconds(offsetS), pb, vb);
    const double dx = pa.x - pb.x;
    const double dy = pa.y - pb.y;
    const double dz = pa.z - pb.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz) <= m_maxIslRangeM &&
           IsLimbClear(pa, pb, m_islMinTangentAltM) && !CrossPlaneBlocked(a, pa, b, pb);
}

double
ContactGraphScheduler::PredictContactEndS(uint32_t a,
                                          uint32_t b,
                                          bool isl,
                                          double horizonS,
                                          double stepS) const
{
    if (!ContactUpAt(a, b, isl, 0.0))
    {
        return 0.0;
    }
    double lo = 0.0;
    for (double t = stepS; t <= horizonS; t += stepS)
    {
        if (!ContactUpAt(a, b, isl, t))
        {
            double hi = t;
            while (hi - lo > 0.01)
            {
                const double mid = 0.5 * (lo + hi);
                (ContactUpAt(a, b, isl, mid) ? lo : hi) = mid;
            }
            return lo;
        }
        lo = t;
    }
    return horizonS;
}

void
ContactGraphScheduler::RegisterIslCandidate(uint32_t a, uint32_t b, bool crossPlane)
{
    m_islCandidates.emplace_back(std::min(a, b), std::max(a, b));
    if (crossPlane)
    {
        m_crossPlane.insert({std::min(a, b), std::max(a, b)});
    }
}

bool
ContactGraphScheduler::CrossPlaneBlocked(uint32_t a, const Vector& pa, uint32_t b, const Vector& pb) const
{
    if (m_crossLatCutoffDeg <= 0.0 || !m_crossPlane.count({std::min(a, b), std::max(a, b)}))
    {
        return false;
    }
    const double lim = std::sin(m_crossLatCutoffDeg * M_PI / 180.0);
    auto sinLat = [](const Vector& p) { return p.z / std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z); };
    return std::abs(sinLat(pa)) > lim || std::abs(sinLat(pb)) > lim;
}

void
ContactGraphScheduler::CheckIslPair(double t,
                                    uint32_t a,
                                    Ptr<Sgp4MobilityModel> ma,
                                    uint32_t b,
                                    Ptr<Sgp4MobilityModel> mb)
{
    Vector pa = ma->GetEcefPosition();
    Vector pb = mb->GetEcefPosition();
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
    const bool inRange = (range <= m_maxIslRangeM) && losClear && !CrossPlaneBlocked(a, pa, b, pb);
    const std::pair<uint32_t, uint32_t> key{a, b};
    auto stateIt = m_islState.find(key);
    const bool prev =
        (stateIt == m_islState.end()) ? false : stateIt->second;
    ContactEvent ev{t,           a, b,
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

void
ContactGraphScheduler::Tick()
{
    if (!m_running)
    {
        return;
    }
    const double t = Simulator::Now().GetSeconds();

    // P7: sparse GSL sampling. Elevation changes no faster than kMaxElevRate deg/s at any LEO altitude
    // used here (0.9 deg/s overhead at 460 km), so a satellite e degrees below the mask cannot cross it
    // for (e / rate) seconds. Decisions stay on the sampling grid and match the full scan.
    constexpr double kMaxElevRate = 2.0;
    for (auto& c : m_gslCand)
    {
        if (!c.up && t < c.nextCheck)
        {
            continue;
        }
        // Sea-level stations keep the original call (bit-identical results).
        const double elev = (c.altM == 0.0)
                                ? c.mob->GetElevationDeg(c.latDeg, c.lonDeg)
                                : ElevationDeg(c.mob->GetEcefPosition(), c.latDeg, c.lonDeg, c.altM);
        const bool visible = c.up ? (elev >= m_minElevDeg - m_gateHysteresisDeg) : (elev >= m_minElevDeg);
        if (!visible)
        {
            c.nextCheck = t + std::max(0.0, (m_minElevDeg - elev) / kMaxElevRate - 0.2);
        }
        if (!visible && !c.up)
        {
            continue;
        }
        const Vector p = c.mob->GetEcefPosition();
        const double dx = p.x - c.gsEcef.x;
        const double dy = p.y - c.gsEcef.y;
        const double dz = p.z - c.gsEcef.z;
        const double range = std::sqrt(dx * dx + dy * dy + dz * dz);
        ContactEvent ev{t, c.sat, c.gs, false, visible, range, elev};
        if (visible != c.up)
        {
            c.up = visible;
            m_gslState[{c.sat, c.gs}] = visible;
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
            m_contactUpdate(ev);
        }
    }

    // GSL visibility (full scan when no candidate list was set).
    if (m_gslCand.empty())
    for (auto& gsIt : m_gss)
    {
        for (auto& satIt : m_sats)
        {
            const double gsAlt = gsIt.second.alt_m;
            const double elev =
                (gsAlt == 0.0)
                    ? satIt.second->GetElevationDeg(gsIt.second.lat_deg, gsIt.second.lon_deg)
                    : ElevationDeg(satIt.second->GetEcefPosition(),
                                   gsIt.second.lat_deg,
                                   gsIt.second.lon_deg,
                                   gsAlt);
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
            const Vector gs((Ngs + gsAlt) * cosLat * std::cos(lon),
                             (Ngs + gsAlt) * cosLat * std::sin(lon),
                             (Ngs * (1.0 - kWgs84E2) + gsAlt) * sinLat);
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

    // ISL visibility (sat pair Euclidean range): candidate pairs when the
    // topology is terminal-limited, otherwise every pair.
    if (!m_islCandidates.empty() && (m_tickN++ % m_islEvery != 0))
    {
        // P7: not this tick
    }
    else if (!m_islCandidates.empty())
    {
        for (const auto& [a, b] : m_islCandidates)
        {
            CheckIslPair(t, a, m_sats.at(a), b, m_sats.at(b));
        }
    }
    else if (m_sats.size() >= 2)
    {
        for (auto a = m_sats.begin(); a != m_sats.end(); ++a)
        {
            auto b = a;
            ++b;
            for (; b != m_sats.end(); ++b)
            {
                CheckIslPair(t, a->first, a->second, b->first, b->second);
            }
        }
    }

    m_tickEvent =
        Simulator::Schedule(m_dt, &ContactGraphScheduler::Tick, this);
}

} // namespace ntncon
} // namespace ns3
