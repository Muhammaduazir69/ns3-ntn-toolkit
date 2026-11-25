/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#include "contact-graph-scheduler.h"

#include "ns3/boolean.h"
#include "ns3/double.h"
#include "ns3/log.h"
#include "ns3/simulator.h"
#include "ns3/nstime.h"

#include <cmath>
#include <unordered_map>

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
                                          MakeDoubleChecker<double>(0.0))
                            .AddAttribute("TransitionResolution",
                                          "Wave 6B: each tick looks one sampling interval ahead "
                                          "and schedules contact up/down events at the crossing "
                                          "instant, bisected to this resolution, instead of on "
                                          "the sampling grid. 0 restores grid-quantized "
                                          "transitions (pre-Wave-6B behavior).",
                                          TimeValue(MilliSeconds(1)),
                                          MakeTimeAccessor(&ContactGraphScheduler::m_resolution),
                                          MakeTimeChecker(Seconds(0)))
                            .AddAttribute("IslMarginSkip",
                                          "Wave 7B: skip the ISL look-ahead of a pair until its "
                                          "state could first change, bounded by the range margin "
                                          "over the closing-speed bound and the limb margin over "
                                          "the speed bound (vis-viva perigee speed plus Earth "
                                          "rotation at apogee). Exact: no transition is missed. "
                                          "false re-evaluates every pair on every refresh tick.",
                                          BooleanValue(true),
                                          MakeBooleanAccessor(&ContactGraphScheduler::m_islMarginSkip),
                                          MakeBooleanChecker());
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
    if (m_tickEvent.IsPending())
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
ContactGraphScheduler::SpeedBoundMps(uint32_t sat) const
{
    auto it = m_speedBound.find(sat);
    if (it != m_speedBound.end())
    {
        return it->second;
    }
    // Vis-viva at perigee plus the Earth-rotation transport at apogee bounds
    // the ECEF speed on the whole orbit; 5 % covers SGP4 mean-element
    // perturbations. Without usable elements, 14.3 km/s bounds every bound
    // orbit up to GEO radius (11.2 km/s + omega_E x 42164 km).
    double vmax = 14.3e3;
    const auto& el = m_sats.at(sat)->GetElements();
    if (el.semi_major_axis_m > kEarthRadiusM && el.eccentricity >= 0.0 && el.eccentricity < 1.0)
    {
        constexpr double kMu = 3.986004418e14;
        const double a = el.semi_major_axis_m;
        const double e = el.eccentricity;
        vmax = 1.05 * (std::sqrt(kMu * (1.0 + e) / (a * (1.0 - e))) +
                       kEarthRotationRadS * a * (1.0 + e));
    }
    m_speedBound[sat] = vmax;
    return vmax;
}

double
ContactGraphScheduler::IslQuietTimeS(uint32_t a,
                                     const Vector& pa,
                                     uint32_t b,
                                     const Vector& pb,
                                     double range) const
{
    if (m_crossLatCutoffDeg > 0.0 && m_crossPlane.count({std::min(a, b), std::max(a, b)}))
    {
        return 0.0; // latitude cutoff: no bound kept, check every refresh
    }
    const double va = SpeedBoundMps(a);
    const double vb = SpeedBoundMps(b);
    // |d range / dt| <= |v_a - v_b| <= va + vb.
    const double tRange = std::abs(m_maxIslRangeM - range) / (va + vb);
    // Distance from the geocentre to the segment is 1-Lipschitz in each
    // endpoint (Hausdorff), so it moves no faster than max(va, vb).
    const double dx = pb.x - pa.x;
    const double dy = pb.y - pa.y;
    const double dz = pb.z - pa.z;
    const double seg2 = dx * dx + dy * dy + dz * dz;
    double tt = seg2 > 0.0 ? -(pa.x * dx + pa.y * dy + pa.z * dz) / seg2 : 0.0;
    tt = std::min(1.0, std::max(0.0, tt));
    const Vector c(pa.x + tt * dx, pa.y + tt * dy, pa.z + tt * dz);
    const double dist = c.GetLength();
    const double tLimb = std::abs(dist - (kEarthRadiusM + m_islMinTangentAltM)) / std::max(va, vb);
    return std::min(tRange, tLimb);
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
            const double res = m_resolution.IsZero() ? 0.01 : m_resolution.GetSeconds();
            while (hi - lo > res)
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
    if (m_resolution.IsZero())
    {
        // Legacy: transitions land on the sampling grid.
        SamplePass(t);
    }
    else if (!m_lookaheadInit)
    {
        // The first tick establishes the contacts that are up at t = 0 on the
        // grid (there is no earlier sample to bisect against), then looks ahead.
        SamplePass(t);
        m_lookaheadInit = true;
        for (auto& c : m_gslCand)
        {
            c.projUp = c.up;
        }
        m_gslProj = m_gslState;
        m_islProj = m_islState;
        LookaheadPass(t, false);
    }
    else
    {
        LookaheadPass(t, true);
    }
    m_tickEvent = Simulator::Schedule(m_dt, &ContactGraphScheduler::Tick, this);
}

void
ContactGraphScheduler::SamplePass(double t)
{

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
}


// ---------------------------------------------------------------------------
// Wave 6B: event-driven contact transitions.
//
// The sampler used to declare a contact up or down on the tick that first saw
// it, so every transition was quantized to the sampling interval (0.5 s in the
// route-lifetime study, 1 s by default) and routing reacted up to one interval
// late. Hypatia (IMC 2020) shows a 1 s forwarding-state step misses path
// changes for 6 percent of pairs against 0.4 percent at 100 ms. Here each tick
// LOOKS AHEAD one horizon on the same ephemeris, and when a link's state at
// the horizon differs from its projected state the crossing is bisected to
// TransitionResolution and the up/down event is SCHEDULED at that instant.
// Geometry updates of established links still fire on the tick, with the
// geometry of the tick.
// ---------------------------------------------------------------------------

double
ContactGraphScheduler::GslElevationAt(Ptr<Sgp4MobilityModel> mob,
                                      double latDeg,
                                      double lonDeg,
                                      double altM,
                                      double offsetS) const
{
    Vector p;
    Vector v;
    mob->GetEcefStateAt(Seconds(offsetS), p, v);
    return ElevationDeg(p, latDeg, lonDeg, altM);
}

double
ContactGraphScheduler::BisectCrossing(const std::function<bool(double)>& isNew, double horizonS) const
{
    // isNew(0) is false (old state), isNew(horizonS) is true (new state).
    double lo = 0.0;
    double hi = horizonS;
    const double res = std::max(1e-6, m_resolution.GetSeconds());
    while (hi - lo > res)
    {
        const double mid = 0.5 * (lo + hi);
        (isNew(mid) ? hi : lo) = mid;
    }
    return hi;
}

void
ContactGraphScheduler::EmitGsl(uint32_t sat, uint32_t gs, bool up, int candIdx)
{
    if (!m_running)
    {
        return;
    }
    const std::pair<uint32_t, uint32_t> key{sat, gs};
    auto it = m_gslState.find(key);
    const bool prev = (it == m_gslState.end()) ? false : it->second;
    if (prev == up)
    {
        return;
    }
    m_gslState[key] = up;
    if (candIdx >= 0)
    {
        m_gslCand[candIdx].up = up;
    }
    const auto& g = m_gss.at(gs);
    Ptr<Sgp4MobilityModel> mob = m_sats.at(sat);
    // Exact state at the crossing instant: GetEcefPosition() is cached on a
    // 100 ms grain and would report the geometry of the grain start.
    Vector p;
    Vector vel;
    mob->GetEcefStateAt(Seconds(0), p, vel);
    const double elev = ElevationDeg(p, g.lat_deg, g.lon_deg, g.alt_m);
    const double lat = g.lat_deg * M_PI / 180.0;
    const double lon = g.lon_deg * M_PI / 180.0;
    const double Ngs = kEarthRadiusM / std::sqrt(1.0 - kWgs84E2 * std::sin(lat) * std::sin(lat));
    const Vector gsE((Ngs + g.alt_m) * std::cos(lat) * std::cos(lon),
                     (Ngs + g.alt_m) * std::cos(lat) * std::sin(lon),
                     (Ngs * (1.0 - kWgs84E2) + g.alt_m) * std::sin(lat));
    const double range = CalculateDistance(p, gsE);
    ContactEvent ev{Simulator::Now().GetSeconds(), sat, gs, false, up, range, elev};
    ++m_eventDrivenTransitions;
    if (up)
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

void
ContactGraphScheduler::EmitIsl(uint32_t a, uint32_t b, bool up)
{
    if (!m_running)
    {
        return;
    }
    const std::pair<uint32_t, uint32_t> key{a, b};
    auto it = m_islState.find(key);
    const bool prev = (it == m_islState.end()) ? false : it->second;
    if (prev == up)
    {
        return;
    }
    m_islState[key] = up;
    Vector pa;
    Vector pb;
    Vector v;
    m_sats.at(a)->GetEcefStateAt(Seconds(0), pa, v);
    m_sats.at(b)->GetEcefStateAt(Seconds(0), pb, v);
    const double range = CalculateDistance(pa, pb);
    ContactEvent ev{Simulator::Now().GetSeconds(), a, b, true, up, range, 0.0};
    ++m_eventDrivenTransitions;
    if (up)
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

void
ContactGraphScheduler::LookaheadPass(double t, bool publishUpdates)
{
    const double H = m_dt.GetSeconds();
    constexpr double kMaxElevRate = 2.0; // deg/s, as in SamplePass (P7)

    auto gslUpdate = [this](uint32_t sat, uint32_t gs, Ptr<Sgp4MobilityModel> mob, double lat,
                            double lon, double alt, const Vector& gsE) {
        const Vector p = mob->GetEcefPosition();
        const double elev = ElevationDeg(p, lat, lon, alt);
        ContactEvent ev{Simulator::Now().GetSeconds(), sat, gs, false, true,
                        CalculateDistance(p, gsE), elev};
        m_contactUpdate(ev);
    };

    for (size_t idx = 0; idx < m_gslCand.size(); ++idx)
    {
        auto& c = m_gslCand[idx];
        if (publishUpdates && c.up)
        {
            gslUpdate(c.sat, c.gs, c.mob, c.latDeg, c.lonDeg, c.altM, c.gsEcef);
        }
        if (!c.projUp && t + H < c.nextCheck)
        {
            continue;
        }
        const double elevH = GslElevationAt(c.mob, c.latDeg, c.lonDeg, c.altM, H);
        const double thr = c.projUp ? m_minElevDeg - m_gateHysteresisDeg : m_minElevDeg;
        const bool visH = elevH >= thr;
        if (!visH)
        {
            c.nextCheck = t + H + std::max(0.0, (m_minElevDeg - elevH) / kMaxElevRate - 0.2);
        }
        if (visH != c.projUp)
        {
            const double tStar = BisectCrossing(
                [&](double off) {
                    return (GslElevationAt(c.mob, c.latDeg, c.lonDeg, c.altM, off) >= thr) == visH;
                },
                H);
            c.projUp = visH;
            Simulator::Schedule(Seconds(tStar), &ContactGraphScheduler::EmitGsl, this, c.sat, c.gs,
                                visH, static_cast<int>(idx));
        }
    }

    if (m_gslCand.empty())
    {
        for (auto& gsIt : m_gss)
        {
            const auto& g = gsIt.second;
            const double lat = g.lat_deg * M_PI / 180.0;
            const double lon = g.lon_deg * M_PI / 180.0;
            const double Ngs =
                kEarthRadiusM / std::sqrt(1.0 - kWgs84E2 * std::sin(lat) * std::sin(lat));
            const Vector gsE((Ngs + g.alt_m) * std::cos(lat) * std::cos(lon),
                             (Ngs + g.alt_m) * std::cos(lat) * std::sin(lon),
                             (Ngs * (1.0 - kWgs84E2) + g.alt_m) * std::sin(lat));
            for (auto& satIt : m_sats)
            {
                const std::pair<uint32_t, uint32_t> key{satIt.first, gsIt.first};
                auto st = m_gslState.find(key);
                if (publishUpdates && st != m_gslState.end() && st->second)
                {
                    gslUpdate(satIt.first, gsIt.first, satIt.second, g.lat_deg, g.lon_deg, g.alt_m,
                              gsE);
                }
                bool& proj = m_gslProj[key];
                const double thr = proj ? m_minElevDeg - m_gateHysteresisDeg : m_minElevDeg;
                const bool visH =
                    GslElevationAt(satIt.second, g.lat_deg, g.lon_deg, g.alt_m, H) >= thr;
                if (visH != proj)
                {
                    Ptr<Sgp4MobilityModel> mob = satIt.second;
                    const double tStar = BisectCrossing(
                        [&](double off) {
                            return (GslElevationAt(mob, g.lat_deg, g.lon_deg, g.alt_m, off) >=
                                    thr) == visH;
                        },
                        H);
                    proj = visH;
                    Simulator::Schedule(Seconds(tStar), &ContactGraphScheduler::EmitGsl, this,
                                        satIt.first, gsIt.first, visH, -1);
                }
            }
        }
    }

    // ISLs: on refresh ticks only (SetIslRefreshEvery), looking ahead to the
    // next refresh tick, so a transition anywhere in between is still caught
    // and placed at its own instant.
    const bool islTick = (m_tickN++ % m_islEvery == 0);
    if (!islTick)
    {
        return;
    }
    const double Hi = H * m_islEvery;
    // Wave 7B: one propagation per satellite per refresh tick at the horizon,
    // instead of two per pair (each +Grid satellite sits in four pairs).
    std::unordered_map<uint32_t, Vector> posH;
    auto stateH = [&](uint32_t s) -> Vector {
        auto it = posH.find(s);
        if (it != posH.end())
        {
            return it->second;
        }
        Vector p;
        Vector v;
        m_sats.at(s)->GetEcefStateAt(Seconds(Hi), p, v);
        posH.emplace(s, p);
        return p;
    };
    // Wave 7B: the published geometry of an established ISL is the state at
    // this tick, read once per satellite (each +Grid satellite is in four
    // pairs) rather than once per pair end.
    std::unordered_map<uint32_t, Vector> posNow;
    auto stateNow = [&](uint32_t s) -> Vector {
        auto it = posNow.find(s);
        if (it != posNow.end())
        {
            return it->second;
        }
        const Vector p = m_sats.at(s)->GetEcefPosition();
        posNow.emplace(s, p);
        return p;
    };
    auto checkPair = [&](uint32_t a, uint32_t b) {
        const std::pair<uint32_t, uint32_t> key{a, b};
        auto st = m_islState.find(key);
        if (publishUpdates && st != m_islState.end() && st->second)
        {
            const double range = CalculateDistance(stateNow(a), stateNow(b));
            ContactEvent ev{Simulator::Now().GetSeconds(), a, b, true, true, range, 0.0};
            m_contactUpdate(ev);
        }
        double& next = m_islNextCheck[key];
        if (m_islMarginSkip && t + Hi < next)
        {
            ++m_islChecksSkipped;
            return;
        }
        ++m_islChecks;
        bool& proj = m_islProj[key];
        const Vector pa = stateH(a);
        const Vector pb = stateH(b);
        const double range = CalculateDistance(pa, pb);
        const bool upH = range <= m_maxIslRangeM && IsLimbClear(pa, pb, m_islMinTangentAltM) &&
                         !CrossPlaneBlocked(a, pa, b, pb);
        if (upH != proj)
        {
            const double tStar =
                BisectCrossing([&](double off) { return ContactUpAt(a, b, true, off) == upH; }, Hi);
            proj = upH;
            Simulator::Schedule(Seconds(tStar), &ContactGraphScheduler::EmitIsl, this, a, b, upH);
            next = 0.0;
            return;
        }
        next = t + Hi + IslQuietTimeS(a, pa, b, pb, range);
    };
    if (!m_islCandidates.empty())
    {
        for (const auto& [a, b] : m_islCandidates)
        {
            checkPair(a, b);
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
                checkPair(a->first, b->first);
            }
        }
    }
}

} // namespace ntncon
} // namespace ns3
