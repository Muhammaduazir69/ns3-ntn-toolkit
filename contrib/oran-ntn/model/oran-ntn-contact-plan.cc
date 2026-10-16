/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only
#include "oran-ntn-contact-plan.h"

#include <ns3/abort.h>
#include <ns3/geographic-positions.h>
#include <ns3/log.h>

#include <algorithm>
#include <cmath>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("OranNtnContactPlan");
NS_OBJECT_ENSURE_REGISTERED(OranNtnContactPlan);

TypeId
OranNtnContactPlan::GetTypeId()
{
    static TypeId tid = TypeId("ns3::OranNtnContactPlan")
                            .SetParent<Object>()
                            .SetGroupName("OranNtn")
                            .AddConstructor<OranNtnContactPlan>();
    return tid;
}

Vector
OranNtnContactPlan::GeodeticToEcef(double latDeg, double lonDeg, double altM)
{
    return GeographicPositions::GeographicToCartesianCoordinates(latDeg,
                                                                 lonDeg,
                                                                 altM,
                                                                 GeographicPositions::WGS84);
}

double
OranNtnContactPlan::ElevationDeg(const Vector& sat, const Vector& gs)
{
    // Elevation above the local horizontal of the ground point. The local
    // "up" is the ellipsoid normal; using the geocentric radius instead would
    // bias the angle by up to ~0.19 deg at mid latitudes, which is not
    // negligible against a 10-25 deg feeder mask edge, so recover geodetic
    // latitude/longitude first.
    const Vector geo = GeographicPositions::CartesianToGeographicCoordinates(
        gs, GeographicPositions::WGS84);
    const double lat = geo.x * M_PI / 180.0;
    const double lon = geo.y * M_PI / 180.0;
    const double ux = std::cos(lat) * std::cos(lon);
    const double uy = std::cos(lat) * std::sin(lon);
    const double uz = std::sin(lat);
    const double dx = sat.x - gs.x;
    const double dy = sat.y - gs.y;
    const double dz = sat.z - gs.z;
    const double r = std::sqrt(dx * dx + dy * dy + dz * dz);
    if (r <= 0.0)
    {
        return 90.0;
    }
    const double s = (dx * ux + dy * uy + dz * uz) / r;
    return std::asin(std::max(-1.0, std::min(1.0, s))) * 180.0 / M_PI;
}

void
OranNtnContactPlan::AddGateway(uint32_t gwId,
                               double latDeg,
                               double lonDeg,
                               double altM,
                               double minElevDeg)
{
    m_gw.push_back({gwId, GeodeticToEcef(latDeg, lonDeg, altM), minElevDeg});
}

void
OranNtnContactPlan::AddSatellite(uint32_t satId, EcefFn ecef)
{
    NS_ABORT_MSG_IF(!ecef, "contact plan: null position function for sat " << satId);
    m_sat[satId] = std::move(ecef);
}

Vector
OranNtnContactPlan::GetGatewayEcef(uint32_t gwId) const
{
    for (const auto& g : m_gw)
    {
        if (g.id == gwId)
        {
            return g.ecef;
        }
    }
    NS_ABORT_MSG("contact plan: unknown gateway " << gwId);
    return Vector();
}

double
OranNtnContactPlan::GetGatewayMaskDeg(uint32_t gwId) const
{
    for (const auto& g : m_gw)
    {
        if (g.id == gwId)
        {
            return g.maskDeg;
        }
    }
    NS_ABORT_MSG("contact plan: unknown gateway " << gwId);
    return 0.0;
}

bool
OranNtnContactPlan::SeesGateway(const EcefFn& f, const Gateway& g, Time t) const
{
    return ElevationDeg(f(t), g.ecef) >= g.maskDeg;
}

std::vector<OranNtnContactPlan::Window>
OranNtnContactPlan::ScanPair(const EcefFn& f,
                             const Gateway& g,
                             Time from,
                             Time to,
                             Time step,
                             Time res) const
{
    // Bisection between a sample known to be in state `a` and one known to be
    // in the other state; returns the first instant in the new state.
    auto refine = [&](Time lo, Time hi, bool loState) {
        while (hi - lo > res)
        {
            const Time mid = lo + (hi - lo) / 2;
            if (SeesGateway(f, g, mid) == loState)
            {
                lo = mid;
            }
            else
            {
                hi = mid;
            }
        }
        return hi;
    };

    std::vector<Window> out;
    bool prev = SeesGateway(f, g, from);
    Time open = prev ? from : Time::Max();
    Time tPrev = from;
    for (Time t = from + step; t <= to; t += step)
    {
        const bool cur = SeesGateway(f, g, t);
        if (cur != prev)
        {
            const Time edge = refine(tPrev, t, prev);
            if (cur)
            {
                open = edge;
            }
            else
            {
                out.push_back({open, edge});
                open = Time::Max();
            }
            prev = cur;
        }
        tPrev = t;
    }
    if (prev)
    {
        out.push_back({open, to});
    }
    return out;
}

std::vector<OranNtnContactPlan::Window>
OranNtnContactPlan::Union(std::vector<Window> w)
{
    std::sort(w.begin(), w.end(), [](const Window& a, const Window& b) {
        return a.start < b.start;
    });
    std::vector<Window> out;
    for (const auto& x : w)
    {
        if (!out.empty() && x.start <= out.back().end)
        {
            out.back().end = std::max(out.back().end, x.end);
        }
        else
        {
            out.push_back(x);
        }
    }
    return out;
}

void
OranNtnContactPlan::Build(Time from, Time to, Time step, Time resolution)
{
    NS_ABORT_MSG_IF(step <= Time(0) || resolution <= Time(0), "contact plan: bad step");
    m_merged.clear();
    m_pair.clear();
    for (const auto& [satId, f] : m_sat)
    {
        std::vector<Window> all;
        for (const auto& g : m_gw)
        {
            auto w = ScanPair(f, g, from, to, step, resolution);
            all.insert(all.end(), w.begin(), w.end());
            m_pair[{satId, g.id}] = std::move(w);
        }
        m_merged[satId] = Union(std::move(all));
    }
}

const std::vector<OranNtnContactPlan::Window>&
OranNtnContactPlan::GetWindows(uint32_t satId) const
{
    auto it = m_merged.find(satId);
    return it == m_merged.end() ? m_empty : it->second;
}

const std::vector<OranNtnContactPlan::Window>&
OranNtnContactPlan::GetPairWindows(uint32_t satId, uint32_t gwId) const
{
    auto it = m_pair.find({satId, gwId});
    return it == m_pair.end() ? m_empty : it->second;
}

bool
OranNtnContactPlan::IsVisible(uint32_t satId, Time t) const
{
    for (const auto& w : GetWindows(satId))
    {
        if (t >= w.start && t < w.end)
        {
            return true;
        }
        if (w.start > t)
        {
            break;
        }
    }
    return false;
}

Time
OranNtnContactPlan::NextLoss(uint32_t satId, Time t) const
{
    for (const auto& w : GetWindows(satId))
    {
        if (t >= w.start && t < w.end)
        {
            return w.end;
        }
        if (w.start > t)
        {
            break;
        }
    }
    return IsVisible(satId, t) ? Time::Max() : t;
}

Time
OranNtnContactPlan::NextGain(uint32_t satId, Time t) const
{
    for (const auto& w : GetWindows(satId))
    {
        if (t < w.end)
        {
            return std::max(t, w.start);
        }
    }
    return Time::Max();
}

double
OranNtnContactPlan::Availability(uint32_t satId, Time from, Time to) const
{
    if (to <= from)
    {
        return 0.0;
    }
    Time covered(0);
    for (const auto& w : GetWindows(satId))
    {
        const Time a = std::max(w.start, from);
        const Time b = std::min(w.end, to);
        if (b > a)
        {
            covered += b - a;
        }
    }
    return covered.GetSeconds() / (to - from).GetSeconds();
}

} // namespace ns3
