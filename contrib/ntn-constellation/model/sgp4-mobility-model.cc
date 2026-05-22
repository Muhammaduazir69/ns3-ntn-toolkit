/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#include "sgp4-mobility-model.h"

#include "ns3/double.h"
#include "ns3/log.h"
#include "ns3/simulator.h"

#include <cmath>

namespace ns3
{
namespace ntncon
{

NS_LOG_COMPONENT_DEFINE("Sgp4MobilityModel");
NS_OBJECT_ENSURE_REGISTERED(Sgp4MobilityModel);

namespace
{

constexpr double kPi = M_PI;
constexpr double kTwoPi = 2.0 * M_PI;
constexpr double kDegToRad = kPi / 180.0;
constexpr double kRadToDeg = 180.0 / kPi;

/// Solve Kepler's equation M = E - e*sin(E) for E by Newton iteration.
double
SolveKepler(double M, double e)
{
    // Wrap M into (-pi, pi].
    while (M > kPi)
        M -= kTwoPi;
    while (M <= -kPi)
        M += kTwoPi;
    double E = (e < 0.8) ? M : kPi;
    for (int it = 0; it < 20; ++it)
    {
        const double f = E - e * std::sin(E) - M;
        const double fp = 1.0 - e * std::cos(E);
        const double dE = f / fp;
        E -= dE;
        if (std::abs(dE) < 1e-12)
            break;
    }
    return E;
}

/// Julian Date from Unix seconds.
double
UnixToJulian(double unix_s)
{
    return 2440587.5 + unix_s / 86400.0;
}

/// Greenwich Mean Sidereal Time (radians) from Julian Date. Accurate to
/// ~arcsec for the v2.1 baseline; we don't need ITRF precision yet.
double
GmstRad(double unix_s)
{
    const double jd = UnixToJulian(unix_s);
    const double T = (jd - 2451545.0) / 36525.0;
    // IAU 1982 formula (degrees) -> rad.
    double gmst_deg = 280.46061837 + 360.98564736629 * (jd - 2451545.0) +
                       T * T * (0.000387933 - T / 38710000.0);
    double gmst_rad = std::fmod(gmst_deg * kDegToRad, kTwoPi);
    if (gmst_rad < 0.0)
        gmst_rad += kTwoPi;
    return gmst_rad;
}

} // namespace

TypeId
Sgp4MobilityModel::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::ntncon::Sgp4MobilityModel")
            .SetParent<MobilityModel>()
            .SetGroupName("NtnConstellation")
            .AddConstructor<Sgp4MobilityModel>();
    return tid;
}

Sgp4MobilityModel::Sgp4MobilityModel() = default;

void
Sgp4MobilityModel::SetElements(const KeplerianElements& elements)
{
    m_elements = elements;
    m_cacheValid = false;
}

bool
Sgp4MobilityModel::SetTle(const TleRecord& tle)
{
    KeplerianElements el{};
    if (!tle.ToKeplerian(el))
    {
        NS_LOG_WARN("Sgp4MobilityModel::SetTle: parse failed");
        return false;
    }
    SetElements(el);
    return true;
}

void
Sgp4MobilityModel::Propagate(double unix_s, Vector& pos_eci, Vector& vel_eci)
    const
{
    const double dt = unix_s - m_elements.epoch_unix_s;
    const double a = m_elements.semi_major_axis_m;
    const double e = m_elements.eccentricity;
    const double i = m_elements.inclination_rad;
    const double n0 = m_elements.MeanMotionRadS();

    // J2 secular rates per Vallado §9 (small-eccentricity approximation).
    const double p = a * (1.0 - e * e);
    const double aRe2 = (kEarthRadiusM / p) * (kEarthRadiusM / p);
    const double cosI = std::cos(i);
    const double sinI = std::sin(i);
    const double raanDot = -1.5 * kEarthJ2 * aRe2 * n0 * cosI;
    const double argpDot =
        0.75 * kEarthJ2 * aRe2 * n0 * (4.0 - 5.0 * sinI * sinI);

    const double raan = m_elements.raan_rad + raanDot * dt;
    const double argp = m_elements.arg_perigee_rad + argpDot * dt;
    const double M = m_elements.mean_anomaly_rad + n0 * dt;
    const double E = SolveKepler(M, e);

    // Position in perifocal frame.
    const double cosE = std::cos(E);
    const double sinE = std::sin(E);
    const double xp = a * (cosE - e);
    const double yp = a * std::sqrt(1.0 - e * e) * sinE;
    // True anomaly (for velocity).
    const double nu = std::atan2(std::sqrt(1.0 - e * e) * sinE, cosE - e);
    const double r = a * (1.0 - e * cosE);
    const double h = std::sqrt(kEarthMuM3S2 * p);
    const double vxp = -h / p * std::sin(nu);
    const double vyp = h / p * (e + std::cos(nu));

    // Rotate perifocal -> ECI: R_z(-raan) * R_x(-i) * R_z(-argp).
    const double cw = std::cos(argp);
    const double sw = std::sin(argp);
    const double co = std::cos(raan);
    const double so = std::sin(raan);
    const double ci = cosI;
    const double si = sinI;

    auto rotate = [&](double xp_, double yp_) -> Vector {
        const double x1 = cw * xp_ - sw * yp_;
        const double y1 = sw * xp_ + cw * yp_;
        // Now rotate by inclination about x.
        const double x2 = x1;
        const double y2 = ci * y1; // z' part
        const double z2 = si * y1;
        // Rotate by RAAN about z.
        return Vector(co * x2 - so * y2, so * x2 + co * y2, z2);
    };

    pos_eci = rotate(xp, yp);
    vel_eci = rotate(vxp, vyp);
    (void)r;
}

Vector
Sgp4MobilityModel::EciToEcef(const Vector& eci, double unix_s)
{
    const double theta = GmstRad(unix_s);
    const double c = std::cos(theta);
    const double s = std::sin(theta);
    return Vector(c * eci.x + s * eci.y, -s * eci.x + c * eci.y, eci.z);
}

void
Sgp4MobilityModel::EnsureCache() const
{
    const Time now = Simulator::Now();
    if (m_cacheValid && (now - m_cacheTime).GetMilliSeconds() <
                            m_cacheGrain.GetMilliSeconds())
    {
        return;
    }
    const double unix_s = m_elements.epoch_unix_s + now.GetSeconds();
    Vector pos_eci;
    Vector vel_eci;
    Propagate(unix_s, pos_eci, vel_eci);
    m_cachedEci = pos_eci;
    m_cachedEciVel = vel_eci;
    m_cacheTime = now;
    m_cacheValid = true;
}

Vector
Sgp4MobilityModel::GetEciPosition() const
{
    EnsureCache();
    return m_cachedEci;
}

Vector
Sgp4MobilityModel::GetEcefPosition() const
{
    EnsureCache();
    const double unix_s = m_elements.epoch_unix_s +
                            Simulator::Now().GetSeconds();
    return EciToEcef(m_cachedEci, unix_s);
}

Vector
Sgp4MobilityModel::GetEcefVelocity() const
{
    EnsureCache();
    const double unix_s = m_elements.epoch_unix_s +
                            Simulator::Now().GetSeconds();
    // ECEF velocity = R(ECI->ECEF) * vel_eci - omega x r_ecef
    Vector v_eci_in_ecef = EciToEcef(m_cachedEciVel, unix_s);
    Vector r_ecef = EciToEcef(m_cachedEci, unix_s);
    const double w = kEarthRotationRadS;
    Vector omega_cross_r(-w * r_ecef.y, w * r_ecef.x, 0.0);
    return Vector(v_eci_in_ecef.x - omega_cross_r.x,
                  v_eci_in_ecef.y - omega_cross_r.y,
                  v_eci_in_ecef.z);
}

void
Sgp4MobilityModel::GetGeodetic(double& lat_deg, double& lon_deg,
                                 double& alt_m) const
{
    Vector r = GetEcefPosition();
    const double x = r.x;
    const double y = r.y;
    const double z = r.z;
    const double rho = std::sqrt(x * x + y * y);
    // Spherical approximation — adequate for the v2.1 baseline; ellipsoidal
    // refinement is a 4.4.X follow-on.
    const double r_norm = std::sqrt(rho * rho + z * z);
    lat_deg = std::atan2(z, rho) * kRadToDeg;
    lon_deg = std::atan2(y, x) * kRadToDeg;
    alt_m = r_norm - kEarthRadiusM;
}

double
Sgp4MobilityModel::GetElevationDeg(double gs_lat_deg, double gs_lon_deg) const
{
    Vector r_sat = GetEcefPosition();
    // GS position in ECEF (sea level).
    const double lat = gs_lat_deg * kDegToRad;
    const double lon = gs_lon_deg * kDegToRad;
    const double cosLat = std::cos(lat);
    Vector r_gs(kEarthRadiusM * cosLat * std::cos(lon),
                 kEarthRadiusM * cosLat * std::sin(lon),
                 kEarthRadiusM * std::sin(lat));
    // Vector GS -> Sat.
    Vector d(r_sat.x - r_gs.x, r_sat.y - r_gs.y, r_sat.z - r_gs.z);
    // Local up direction at GS.
    Vector up(cosLat * std::cos(lon), cosLat * std::sin(lon), std::sin(lat));
    const double dotUp = d.x * up.x + d.y * up.y + d.z * up.z;
    const double dNorm = std::sqrt(d.x * d.x + d.y * d.y + d.z * d.z);
    if (dNorm <= 0.0)
        return 0.0;
    const double sinEl = dotUp / dNorm;
    return std::asin(std::max(-1.0, std::min(1.0, sinEl))) * kRadToDeg;
}

Vector
Sgp4MobilityModel::DoGetPosition() const
{
    return GetEcefPosition();
}

void
Sgp4MobilityModel::DoSetPosition(const Vector& /*position*/)
{
    NS_LOG_WARN("Sgp4MobilityModel::DoSetPosition ignored — orbital "
                "propagation drives position; install new elements via "
                "SetElements / SetTle instead");
}

Vector
Sgp4MobilityModel::DoGetVelocity() const
{
    return GetEcefVelocity();
}

} // namespace ntncon
} // namespace ns3
