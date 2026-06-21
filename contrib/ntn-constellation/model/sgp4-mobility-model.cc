/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#include "sgp4-mobility-model.h"

#include "ns3/double.h"
#include "ns3/log.h"
#include "ns3/satellite-sgp4io.h"   // twoline2rv
#include "ns3/satellite-sgp4unit.h" // elsetrec, sgp4, gravconsttype
#include "ns3/simulator.h"

#include <cmath>
#include <cstring>

namespace ns3
{
namespace ntncon
{

/// Full Vallado SGP4 record, reusing the satellite module's reference port.
struct ValladoState
{
    std::string line1;
    std::string line2;
    elsetrec rec{};
    bool ready{false};
};

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

/// Unix seconds from Julian Date.
double
JulianToUnix(double jd)
{
    return (jd - 2440587.5) * 86400.0;
}

/// WGS-72 is the gravity model SGP4 was fit against (Vallado recommendation,
/// matching the satellite module's SatSGP4MobilityModel).
constexpr gravconsttype kWGeoSys = wgs72;

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
    // Retain the raw lines so the Vallado backend can (re)initialise; both lines
    // must be present and TLE-sized.
    if (!m_vallado)
    {
        m_vallado = std::make_shared<ValladoState>();
    }
    m_vallado->line1 = tle.line1;
    m_vallado->line2 = tle.line2;
    m_vallado->ready = false;
    if (m_useVallado)
    {
        InitVallado();
    }
    return true;
}

void
Sgp4MobilityModel::SetUseVallado(bool on)
{
    m_useVallado = on;
    m_cacheValid = false;
    if (on && m_vallado && !m_vallado->line1.empty() && !m_vallado->ready)
    {
        InitVallado();
    }
}

bool
Sgp4MobilityModel::IsValladoReady() const
{
    return m_vallado && m_vallado->ready;
}

void
Sgp4MobilityModel::InitVallado()
{
    if (!m_vallado || m_vallado->line1.size() < 60 || m_vallado->line2.size() < 60)
    {
        return;
    }
    char l1[130];
    char l2[130];
    std::memset(l1, 0, sizeof(l1));
    std::memset(l2, 0, sizeof(l2));
    std::memcpy(l1, m_vallado->line1.c_str(),
                std::min(m_vallado->line1.size(), sizeof(l1) - 1));
    std::memcpy(l2, m_vallado->line2.c_str(),
                std::min(m_vallado->line2.size(), sizeof(l2) - 1));
    double startmfe = 0.0;
    double stopmfe = 0.0;
    double deltamin = 0.0;
    // 'c' catalog run, 'e' epoch-relative time, 'i' improved operation — the
    // same invocation the satellite module uses.
    twoline2rv(l1, l2, 'c', 'e', 'i', kWGeoSys, startmfe, stopmfe, deltamin, m_vallado->rec);
    double r[3];
    double v[3];
    const bool ok = sgp4(kWGeoSys, m_vallado->rec, 0.0, r, v);
    m_vallado->ready = ok && (m_vallado->rec.error == 0);
    if (m_vallado->ready)
    {
        // Anchor the model's epoch to the TLE epoch so tsince == Now exactly.
        m_elements.epoch_unix_s = JulianToUnix(m_vallado->rec.jdsatepoch);
        m_cacheValid = false;
    }
    else
    {
        NS_LOG_WARN("Sgp4MobilityModel::InitVallado: sgp4 init failed (error "
                    << m_vallado->rec.error << ")");
    }
}

void
Sgp4MobilityModel::Propagate(double unix_s, Vector& pos_eci, Vector& vel_eci)
    const
{
    // ---- Full Vallado SGP4 path (drag/B* included) ----
    if (m_useVallado && m_vallado && m_vallado->ready)
    {
        const double tsince_min = (UnixToJulian(unix_s) - m_vallado->rec.jdsatepoch) * 1440.0;
        double r[3];
        double v[3];
        if (sgp4(kWGeoSys, m_vallado->rec, tsince_min, r, v) && m_vallado->rec.error == 0)
        {
            // sgp4 returns TEME km / km-s; the GMST-rotation EciToEcef below
            // (the same the satellite module uses) takes TEME -> ECEF.
            pos_eci = Vector(r[0] * 1000.0, r[1] * 1000.0, r[2] * 1000.0);
            vel_eci = Vector(v[0] * 1000.0, v[1] * 1000.0, v[2] * 1000.0);
            return;
        }
        NS_LOG_WARN("sgp4 propagation failed (error " << m_vallado->rec.error
                                                      << "); falling back to Kepler+J2");
    }

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
