/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026 Muhammad Uzair
 * SPDX-License-Identifier: GPL-2.0-only
 */
#include "ntn-tr38811-mobility-model.h"

#include "ns3/log.h"
#include "ns3/simulator.h"

#include <cmath>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("NtnTr38811MobilityModel");
NS_OBJECT_ENSURE_REGISTERED(NtnTr38811MobilityModel);

namespace ntngeo
{
namespace
{
constexpr double kA = 6378137.0;             // WGS84 semi-major axis
constexpr double kF = 1.0 / 298.257223563;   // WGS84 flattening
constexpr double kE2 = kF * (2.0 - kF);      // first eccentricity^2
constexpr double kC = 299792458.0;           // speed of light
} // namespace

Vector
GeodeticToEcef(double latDeg, double lonDeg, double altM)
{
    const double lat = latDeg * M_PI / 180.0;
    const double lon = lonDeg * M_PI / 180.0;
    const double s = std::sin(lat), c = std::cos(lat);
    const double n = kA / std::sqrt(1.0 - kE2 * s * s);
    return Vector((n + altM) * c * std::cos(lon),
                  (n + altM) * c * std::sin(lon),
                  (n * (1.0 - kE2) + altM) * s);
}

void
EcefToGeodetic(const Vector& ecef, double& latDeg, double& lonDeg, double& altM)
{
    // Bowring's iterative method (3 iterations are sub-mm for Earth surface).
    const double p = std::sqrt(ecef.x * ecef.x + ecef.y * ecef.y);
    lonDeg = std::atan2(ecef.y, ecef.x) * 180.0 / M_PI;
    double lat = std::atan2(ecef.z, p * (1.0 - kE2));
    double n = kA;
    double alt = 0.0;
    for (int i = 0; i < 3; ++i)
    {
        const double s = std::sin(lat);
        n = kA / std::sqrt(1.0 - kE2 * s * s);
        alt = p / std::cos(lat) - n;
        lat = std::atan2(ecef.z, p * (1.0 - kE2 * n / (n + alt)));
    }
    latDeg = lat * 180.0 / M_PI;
    altM = alt;
}

Vector
EnuVelocityToEcef(double latDeg, double lonDeg, double vEast, double vNorth, double vUp)
{
    const double lat = latDeg * M_PI / 180.0;
    const double lon = lonDeg * M_PI / 180.0;
    const double sLat = std::sin(lat), cLat = std::cos(lat);
    const double sLon = std::sin(lon), cLon = std::cos(lon);
    // ENU -> ECEF rotation.
    return Vector(-sLon * vEast - sLat * cLon * vNorth + cLat * cLon * vUp,
                  cLon * vEast - sLat * sLon * vNorth + cLat * sLon * vUp,
                  cLat * vNorth + sLat * vUp);
}

double
ElevationDeg(const Vector& ueEcef, const Vector& satEcef)
{
    const Vector d(satEcef.x - ueEcef.x, satEcef.y - ueEcef.y, satEcef.z - ueEcef.z);
    const double dNorm = std::max(1.0, d.GetLength());
    const double ueNorm = std::max(1.0, ueEcef.GetLength());
    // Local up at the UE is the (normalized) ECEF position vector.
    const double cosZenith =
        (d.x * ueEcef.x + d.y * ueEcef.y + d.z * ueEcef.z) / (dNorm * ueNorm);
    return std::asin(std::max(-1.0, std::min(1.0, cosZenith))) * 180.0 / M_PI;
}

double
SlantRangeM(const Vector& a, const Vector& b)
{
    const double dx = a.x - b.x, dy = a.y - b.y, dz = a.z - b.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double
DopplerHz(const Vector& ueEcef,
          const Vector& ueVelEcef,
          const Vector& satEcef,
          const Vector& satVelEcef,
          double freqHz)
{
    Vector los(satEcef.x - ueEcef.x, satEcef.y - ueEcef.y, satEcef.z - ueEcef.z);
    const double n = std::max(1.0, los.GetLength());
    los.x /= n;
    los.y /= n;
    los.z /= n;
    const Vector rel(satVelEcef.x - ueVelEcef.x,
                     satVelEcef.y - ueVelEcef.y,
                     satVelEcef.z - ueVelEcef.z);
    const double rangeRate = rel.x * los.x + rel.y * los.y + rel.z * los.z;
    // Receding (positive range rate) -> negative Doppler.
    return -(rangeRate / kC) * freqHz;
}

Vector
EcefToEnu(const Vector& ecef, double lat0Deg, double lon0Deg, double alt0M)
{
    const Vector origin = GeodeticToEcef(lat0Deg, lon0Deg, alt0M);
    const Vector d(ecef.x - origin.x, ecef.y - origin.y, ecef.z - origin.z);
    const double lat = lat0Deg * M_PI / 180.0;
    const double lon = lon0Deg * M_PI / 180.0;
    const double sLat = std::sin(lat), cLat = std::cos(lat);
    const double sLon = std::sin(lon), cLon = std::cos(lon);
    return Vector(-sLon * d.x + cLon * d.y,
                  -sLat * cLon * d.x - sLat * sLon * d.y + cLat * d.z,
                  cLat * cLon * d.x + cLat * sLon * d.y + sLat * d.z);
}

Vector
EnuToEcef(const Vector& enu, double lat0Deg, double lon0Deg, double alt0M)
{
    const Vector origin = GeodeticToEcef(lat0Deg, lon0Deg, alt0M);
    const Vector r = EnuVelocityToEcef(lat0Deg, lon0Deg, enu.x, enu.y, enu.z);
    return Vector(origin.x + r.x, origin.y + r.y, origin.z + r.z);
}
} // namespace ntngeo

NS_OBJECT_ENSURE_REGISTERED(NtnEnuProjectionMobilityModel);

TypeId
NtnEnuProjectionMobilityModel::GetTypeId()
{
    static TypeId tid = TypeId("ns3::NtnEnuProjectionMobilityModel")
                            .SetParent<MobilityModel>()
                            .SetGroupName("NtnCho")
                            .AddConstructor<NtnEnuProjectionMobilityModel>();
    return tid;
}

NtnEnuProjectionMobilityModel::NtnEnuProjectionMobilityModel() = default;
NtnEnuProjectionMobilityModel::~NtnEnuProjectionMobilityModel() = default;

void
NtnEnuProjectionMobilityModel::SetReference(double lat0Deg, double lon0Deg, double alt0M)
{
    m_lat0 = lat0Deg;
    m_lon0 = lon0Deg;
    m_alt0 = alt0M;
}

Vector
NtnEnuProjectionMobilityModel::DoGetPosition() const
{
    if (!m_source)
    {
        return Vector(0, 0, 0);
    }
    return ntngeo::EcefToEnu(m_source->GetPosition(), m_lat0, m_lon0, m_alt0);
}

void
NtnEnuProjectionMobilityModel::DoSetPosition(const Vector& /*position*/)
{
    // The projection is read-only; the source model owns the trajectory.
}

Vector
NtnEnuProjectionMobilityModel::DoGetVelocity() const
{
    if (!m_source)
    {
        return Vector(0, 0, 0);
    }
    // Rotate the ECEF velocity into the ENU frame (same rotation as positions,
    // without the origin translation).
    const Vector v = m_source->GetVelocity();
    const double lat = m_lat0 * M_PI / 180.0;
    const double lon = m_lon0 * M_PI / 180.0;
    const double sLat = std::sin(lat), cLat = std::cos(lat);
    const double sLon = std::sin(lon), cLon = std::cos(lon);
    return Vector(-sLon * v.x + cLon * v.y,
                  -sLat * cLon * v.x - sLat * sLon * v.y + cLat * v.z,
                  cLat * cLon * v.x + cLat * sLon * v.y + sLat * v.z);
}

TypeId
NtnTr38811MobilityModel::GetTypeId()
{
    static TypeId tid = TypeId("ns3::NtnTr38811MobilityModel")
                            .SetParent<MobilityModel>()
                            .SetGroupName("NtnCho")
                            .AddConstructor<NtnTr38811MobilityModel>();
    return tid;
}

NtnTr38811MobilityModel::NtnTr38811MobilityModel() = default;
NtnTr38811MobilityModel::~NtnTr38811MobilityModel() = default;

void
NtnTr38811MobilityModel::SetUeState(const RealisticUe& ue,
                                    std::shared_ptr<NtnRealisticMobilityHelper> engine)
{
    m_ue = ue;
    m_engine = std::move(engine);
    m_lastUpdate = Simulator::Now();
}

void
NtnTr38811MobilityModel::Advance() const
{
    if (!m_engine)
    {
        return;
    }
    const Time now = Simulator::Now();
    while (m_lastUpdate < now)
    {
        const Time dt = std::min(m_step, now - m_lastUpdate);
        m_engine->AdvanceUe(m_ue, dt.GetSeconds());
        m_lastUpdate += dt;
    }
}

Vector
NtnTr38811MobilityModel::DoGetPosition() const
{
    Advance();
    return ntngeo::GeodeticToEcef(m_ue.lat, m_ue.lon, m_ue.altitude_m);
}

void
NtnTr38811MobilityModel::DoSetPosition(const Vector& position)
{
    ntngeo::EcefToGeodetic(position, m_ue.lat, m_ue.lon, m_ue.altitude_m);
    m_lastUpdate = Simulator::Now();
    NotifyCourseChange();
}

Vector
NtnTr38811MobilityModel::DoGetVelocity() const
{
    Advance();
    return ntngeo::EnuVelocityToEcef(m_ue.lat, m_ue.lon, m_ue.vEast, m_ue.vNorth, m_ue.vUp);
}

void
NtnTr38811MobilityModel::GetGeodetic(double& latDeg, double& lonDeg, double& altM) const
{
    Advance();
    latDeg = m_ue.lat;
    lonDeg = m_ue.lon;
    altM = m_ue.altitude_m;
}

NtnTr38811MobilityHelper::NtnTr38811MobilityHelper(uint64_t seed)
    : m_engine(std::make_shared<NtnRealisticMobilityHelper>(seed))
{
}

std::vector<Ptr<NtnTr38811MobilityModel>>
NtnTr38811MobilityHelper::Install(NodeContainer nodes,
                                  const MobilityProfile& profile,
                                  double minLat,
                                  double maxLat,
                                  double minLon,
                                  double maxLon)
{
    std::vector<Ptr<NtnTr38811MobilityModel>> models;
    auto ues = m_engine->GenerateUes(nodes.GetN(), profile, minLat, maxLat, minLon, maxLon);
    models.reserve(ues.size());
    for (uint32_t i = 0; i < nodes.GetN(); ++i)
    {
        Ptr<NtnTr38811MobilityModel> m = CreateObject<NtnTr38811MobilityModel>();
        m->SetUeState(ues[i], m_engine);
        nodes.Get(i)->AggregateObject(m);
        models.push_back(m);
    }
    return models;
}

} // namespace ns3
