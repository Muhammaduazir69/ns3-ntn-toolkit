/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026 Muhammad Uzair
 * SPDX-License-Identifier: GPL-2.0-only
 */

#include "ntn-geodetic-fixed-mobility-model.h"

#include "ntn-tr38811-mobility-model.h" // ns3::ntngeo geodetic helpers

#include "ns3/double.h"
#include "ns3/enum.h"
#include "ns3/log.h"
#include "ns3/simulator.h"

#include <cmath>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("NtnGeodeticFixedMobilityModel");
NS_OBJECT_ENSURE_REGISTERED(NtnGeodeticFixedMobilityModel);

namespace
{
constexpr double kEarthRotRate = 7.2921159e-5; //!< rad/s, IERS
} // namespace

TypeId
NtnGeodeticFixedMobilityModel::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::NtnGeodeticFixedMobilityModel")
            .SetParent<MobilityModel>()
            .SetGroupName("Mobility")
            .AddConstructor<NtnGeodeticFixedMobilityModel>()
            .AddAttribute("Latitude",
                          "WGS84 geodetic latitude (deg).",
                          DoubleValue(0.0),
                          MakeDoubleAccessor(&NtnGeodeticFixedMobilityModel::m_latDeg),
                          MakeDoubleChecker<double>(-90.0, 90.0))
            .AddAttribute("Longitude",
                          "WGS84 geodetic longitude (deg).",
                          DoubleValue(0.0),
                          MakeDoubleAccessor(&NtnGeodeticFixedMobilityModel::m_lonDeg),
                          MakeDoubleChecker<double>(-180.0, 360.0))
            .AddAttribute("Altitude",
                          "Altitude above the WGS84 ellipsoid (m).",
                          DoubleValue(0.0),
                          MakeDoubleAccessor(&NtnGeodeticFixedMobilityModel::m_altM),
                          MakeDoubleChecker<double>())
            .AddAttribute("ReportFrame",
                          "Frame for GetPosition/GetVelocity: Ecef (default) or Eci.",
                          EnumValue(NtnGeodeticFixedMobilityModel::Ecef),
                          MakeEnumAccessor<ReportFrame>(&NtnGeodeticFixedMobilityModel::m_frame),
                          MakeEnumChecker(NtnGeodeticFixedMobilityModel::Ecef,
                                          "Ecef",
                                          NtnGeodeticFixedMobilityModel::Eci,
                                          "Eci"));
    return tid;
}

NtnGeodeticFixedMobilityModel::NtnGeodeticFixedMobilityModel() = default;
NtnGeodeticFixedMobilityModel::~NtnGeodeticFixedMobilityModel() = default;

void
NtnGeodeticFixedMobilityModel::SetGeodetic(double latDeg, double lonDeg, double altM)
{
    m_latDeg = latDeg;
    m_lonDeg = lonDeg;
    m_altM = altM;
    m_ecef = ntngeo::GeodeticToEcef(latDeg, lonDeg, altM);
    NotifyCourseChange();
}

void
NtnGeodeticFixedMobilityModel::GetGeodetic(double& latDeg, double& lonDeg, double& altM) const
{
    latDeg = m_latDeg;
    lonDeg = m_lonDeg;
    altM = m_altM;
}

Vector
NtnGeodeticFixedMobilityModel::DoGetPosition() const
{
    // The attribute path can set lat/lon/alt without going through
    // SetGeodetic(); make sure the cached ECEF anchor is current.
    Vector ecef = ntngeo::GeodeticToEcef(m_latDeg, m_lonDeg, m_altM);

    if (m_frame == Ecef)
    {
        return ecef;
    }

    // ECI mode: rotate the Earth-fixed anchor by +omega*t about the z-axis so a
    // ground point traces its diurnal circle in the inertial frame.
    const double t = (m_epochOffset + Simulator::Now()).GetSeconds();
    const double a = kEarthRotRate * t;
    const double ca = std::cos(a);
    const double sa = std::sin(a);
    return Vector(ca * ecef.x - sa * ecef.y, sa * ecef.x + ca * ecef.y, ecef.z);
}

void
NtnGeodeticFixedMobilityModel::DoSetPosition(const Vector& position)
{
    // Treat an externally-set position as an ECEF anchor and back-solve the
    // geodetic so GetGeodetic stays consistent. (Rarely used; the geodetic
    // setter/attributes are the normal path.)
    m_ecef = position;
    ntngeo::EcefToGeodetic(position, m_latDeg, m_lonDeg, m_altM);
    NotifyCourseChange();
}

Vector
NtnGeodeticFixedMobilityModel::DoGetVelocity() const
{
    if (m_frame == Ecef)
    {
        // Stationary in the Earth-fixed frame.
        return Vector(0.0, 0.0, 0.0);
    }

    // ECI velocity is omega x r_eci (rigid rotation about z).
    const Vector r = DoGetPosition();
    return Vector(-kEarthRotRate * r.y, kEarthRotRate * r.x, 0.0);
}

} // namespace ns3
