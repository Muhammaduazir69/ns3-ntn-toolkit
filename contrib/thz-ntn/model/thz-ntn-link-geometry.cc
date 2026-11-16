/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#include "thz-ntn-link-geometry.h"

#include "ns3/geocentric-constant-position-mobility-model.h"
#include "ns3/mobility-model.h"

#include <algorithm>
#include <cmath>

namespace ns3
{

namespace
{
constexpr double kA = 6378137.0;               // WGS-84 semi-major axis (m)
constexpr double kF = 1.0 / 298.257223563;     // WGS-84 flattening
constexpr double kE2 = kF * (2.0 - kF);        // first eccentricity squared
constexpr double kB = kA * (1.0 - kF);         // semi-minor axis
constexpr double kEp2 = (kA * kA - kB * kB) / (kB * kB);
constexpr double kHalfEarth = 0.5 * 6371000.0; // ECEF detection threshold

double
Norm(const Vector& v)
{
    return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}
} // namespace

Vector
ThzNtnLinkGeometry::EcefToGeodetic(const Vector& e)
{
    // Bowring (1976) closed form, accurate to well below 1 mm for terrestrial
    // and LEO altitudes.
    const double p = std::sqrt(e.x * e.x + e.y * e.y);
    const double theta = std::atan2(e.z * kA, p * kB);
    const double st = std::sin(theta);
    const double ct = std::cos(theta);
    const double lat =
        std::atan2(e.z + kEp2 * kB * st * st * st, p - kE2 * kA * ct * ct * ct);
    const double lon = std::atan2(e.y, e.x);
    const double sl = std::sin(lat);
    const double n = kA / std::sqrt(1.0 - kE2 * sl * sl);
    double alt;
    if (std::abs(std::cos(lat)) > 1e-9)
    {
        alt = p / std::cos(lat) - n;
    }
    else
    {
        alt = std::abs(e.z) - kB;
    }
    return Vector(lat * 180.0 / M_PI, lon * 180.0 / M_PI, alt);
}

ThzNtnLinkGeometry
ThzNtnLinkGeometry::FromPositions(const Vector& pa, const Vector& pb)
{
    ThzNtnLinkGeometry g;
    const double dx0 = pb.x - pa.x;
    const double dy0 = pb.y - pa.y;
    const double dz0 = pb.z - pa.z;
    g.slantM = std::sqrt(dx0 * dx0 + dy0 * dy0 + dz0 * dz0);
    g.ecef = (Norm(pa) > kHalfEarth && Norm(pb) > kHalfEarth);
    if (!g.ecef)
    {
        const bool aLow = pa.z <= pb.z;
        const Vector& lo = aLow ? pa : pb;
        const Vector& hi = aLow ? pb : pa;
        const double dx = hi.x - lo.x;
        const double dy = hi.y - lo.y;
        const double dz = hi.z - lo.z;
        const double horiz = std::sqrt(dx * dx + dy * dy);
        g.elevationDeg = std::atan2(dz, std::max(horiz, 1e-9)) * 180.0 / M_PI;
        g.lowerAltKm = lo.z / 1000.0;
        g.upperAltKm = hi.z / 1000.0;
        g.lowerEnu = lo;
        const double n = std::max(g.slantM, 1e-9);
        g.dirEnu = Vector(dx / n, dy / n, dz / n);
        return g;
    }
    const Vector ga = EcefToGeodetic(pa);
    const Vector gb = EcefToGeodetic(pb);
    const bool aLow = ga.z <= gb.z;
    const Vector& lo = aLow ? pa : pb;
    const Vector& hi = aLow ? pb : pa;
    const Vector& glo = aLow ? ga : gb;
    const Vector& ghi = aLow ? gb : ga;
    g.lowerAltKm = glo.z / 1000.0;
    g.upperAltKm = ghi.z / 1000.0;
    g.lowerLatDeg = glo.x;
    g.lowerLonDeg = glo.y;
    const double lat = glo.x * M_PI / 180.0;
    const double lon = glo.y * M_PI / 180.0;
    const double dx = hi.x - lo.x;
    const double dy = hi.y - lo.y;
    const double dz = hi.z - lo.z;
    // ECEF -> local ENU at the lower end (geodetic normal).
    const double e = -std::sin(lon) * dx + std::cos(lon) * dy;
    const double nn = -std::sin(lat) * std::cos(lon) * dx - std::sin(lat) * std::sin(lon) * dy +
                      std::cos(lat) * dz;
    const double u = std::cos(lat) * std::cos(lon) * dx + std::cos(lat) * std::sin(lon) * dy +
                     std::sin(lat) * dz;
    const double s = std::max(g.slantM, 1e-9);
    g.dirEnu = Vector(e / s, nn / s, u / s);
    g.elevationDeg = std::asin(std::clamp(u / s, -1.0, 1.0)) * 180.0 / M_PI;
    return g;
}

Vector
ThzNtnLinkGeometry::PositionOf(const MobilityModel* m)
{
    // Geocentric models return topocentric coordinates from GetPosition();
    // the ECEF vector is GetGeocentricPosition() (same rule as
    // NtnEarthBlockageLossModel::EcefOf).
    auto g = dynamic_cast<const GeocentricConstantPositionMobilityModel*>(m);
    return g ? g->GetGeocentricPosition() : m->GetPosition();
}

ThzNtnLinkGeometry
ThzNtnLinkGeometry::Compute(const MobilityModel* a, const MobilityModel* b)
{
    return FromPositions(PositionOf(a), PositionOf(b));
}

} // namespace ns3
