// SPDX-License-Identifier: GPL-2.0-only
// Copyright (c) 2026 Muhammad Uzair
#include "ntn-geocentric-mobility-adapter.h"

#include "ns3/abort.h"
#include "ns3/constant-position-mobility-model.h"
#include "ns3/log.h"

#include <cmath>

namespace ns3
{
namespace ntncon
{

NS_LOG_COMPONENT_DEFINE("NtnGeocentricMobilityAdapter");
NS_OBJECT_ENSURE_REGISTERED(NtnGeocentricMobilityAdapter);

namespace
{
constexpr double kA = 6378137.0;               // WGS-84 semi-major axis (m)
constexpr double kF = 1.0 / 298.257223563;     // WGS-84 flattening
constexpr double kE2 = kF * (2.0 - kF);        // first eccentricity squared
constexpr double kB = kA * (1.0 - kF);         // semi-minor axis (m)
constexpr double kEp2 = (kA * kA - kB * kB) / (kB * kB); // second eccentricity squared
} // namespace

TypeId
NtnGeocentricMobilityAdapter::GetTypeId()
{
    static TypeId tid = TypeId("ns3::ntncon::NtnGeocentricMobilityAdapter")
                            .SetParent<GeocentricConstantPositionMobilityModel>()
                            .SetGroupName("NtnConstellation")
                            .AddConstructor<NtnGeocentricMobilityAdapter>();
    return tid;
}

NtnGeocentricMobilityAdapter::NtnGeocentricMobilityAdapter()
{
    // A fixed ECEF origin until a source is attached.
    m_source = CreateObject<ConstantPositionMobilityModel>();
}

NtnGeocentricMobilityAdapter::~NtnGeocentricMobilityAdapter() = default;

void
NtnGeocentricMobilityAdapter::SetSource(Ptr<MobilityModel> ecefSource)
{
    NS_ABORT_MSG_IF(!ecefSource, "NtnGeocentricMobilityAdapter: null source");
    m_source = ecefSource;
    m_source->TraceConnectWithoutContext(
        "CourseChange",
        MakeCallback(&NtnGeocentricMobilityAdapter::SourceCourseChange, this));
}

void
NtnGeocentricMobilityAdapter::SourceCourseChange(Ptr<const MobilityModel> /*m*/)
{
    NotifyCourseChange();
}

Vector
NtnGeocentricMobilityAdapter::EcefToGeodetic(const Vector& p)
{
    // Bowring (1976) with one refinement; sub-millimetre for terrestrial and
    // orbital altitudes.
    const double lon = std::atan2(p.y, p.x);
    const double rho = std::sqrt(p.x * p.x + p.y * p.y);
    double theta = std::atan2(p.z * kA, rho * kB);
    double lat = std::atan2(p.z + kEp2 * kB * std::pow(std::sin(theta), 3),
                            rho - kE2 * kA * std::pow(std::cos(theta), 3));
    theta = std::atan2((1.0 - kF) * std::sin(lat), std::cos(lat));
    lat = std::atan2(p.z + kEp2 * kB * std::pow(std::sin(theta), 3),
                     rho - kE2 * kA * std::pow(std::cos(theta), 3));
    const double n = kA / std::sqrt(1.0 - kE2 * std::sin(lat) * std::sin(lat));
    double alt;
    if (std::abs(std::cos(lat)) > 1e-9)
    {
        alt = rho / std::cos(lat) - n;
    }
    else
    {
        alt = std::abs(p.z) - kB;
    }
    return Vector(lat * 180.0 / M_PI, lon * 180.0 / M_PI, alt);
}

Vector
NtnGeocentricMobilityAdapter::GeodeticToEcef(const Vector& lla)
{
    const double lat = lla.x * M_PI / 180.0;
    const double lon = lla.y * M_PI / 180.0;
    const double n = kA / std::sqrt(1.0 - kE2 * std::sin(lat) * std::sin(lat));
    return Vector((n + lla.z) * std::cos(lat) * std::cos(lon),
                  (n + lla.z) * std::cos(lat) * std::sin(lon),
                  (n * (1.0 - kE2) + lla.z) * std::sin(lat));
}

Vector
NtnGeocentricMobilityAdapter::DoGetPosition() const
{
    return m_source->GetPosition();
}

void
NtnGeocentricMobilityAdapter::DoSetPosition(const Vector& position)
{
    m_source->SetPosition(position);
}

Vector
NtnGeocentricMobilityAdapter::DoGetVelocity() const
{
    return m_source->GetVelocity();
}

Vector
NtnGeocentricMobilityAdapter::DoGetGeographicPosition() const
{
    return EcefToGeodetic(m_source->GetPosition());
}

void
NtnGeocentricMobilityAdapter::DoSetGeographicPosition(const Vector& latLonAlt)
{
    m_source->SetPosition(GeodeticToEcef(latLonAlt));
}

Vector
NtnGeocentricMobilityAdapter::DoGetGeocentricPosition() const
{
    return m_source->GetPosition();
}

void
NtnGeocentricMobilityAdapter::DoSetGeocentricPosition(const Vector& position)
{
    m_source->SetPosition(position);
}

} // namespace ntncon
} // namespace ns3
