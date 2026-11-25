/*
 * Copyright (c) 2006,2007 INRIA
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Author: Mattia Sandri <mattia.sandri@unipd.it>
 */
#include "geocentric-constant-position-mobility-model.h"

#include "ns3/angles.h"
#include "ns3/global-value.h"
#include "ns3/string.h"

#include <cmath>

/**
 *  @file
 *  @ingroup mobility
 *  Class GeocentricConstantPositionMobilityModel implementation.
 */

namespace ns3
{

NS_OBJECT_ENSURE_REGISTERED(GeocentricConstantPositionMobilityModel);

// ns3-ntn-toolkit src patch 0001 (contrib/ntn-traffic/patches/ns3): opt-in
// elevation convention. "Legacy" keeps the ns-3.43 behaviour bit for bit.
// "Wgs84LowerEnd" picks the lower end by geographic altitude (not by the ECEF
// z coordinate, which swaps the ends whenever the satellite is equatorward of
// the terminal) and returns the signed elevation about the local normal of the
// lower end's own geographic latitude/longitude (TR 38.811 clause 6.6.1 uses
// the elevation at the terminal), floored at 0.01 deg so the 1/sin(elevation)
// gas term of the TR 38.811 path loss stays finite; below 0 deg the link is
// blocked by the Earth anyway (NtnEarthBlockageLossModel).
static GlobalValue g_geocentricElevationModel(
    "GeocentricElevationModel",
    "Elevation convention of GeocentricConstantPositionMobilityModel::GetElevationAngle: "
    "Legacy (ns-3.43) or Wgs84LowerEnd (lower end by altitude, signed, local normal)",
    StringValue("Legacy"),
    MakeStringChecker());

TypeId
GeocentricConstantPositionMobilityModel::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::GeocentricConstantPositionMobilityModel")
            .SetParent<MobilityModel>()
            .SetGroupName("Mobility")
            .AddConstructor<GeocentricConstantPositionMobilityModel>()
            .AddAttribute(
                "PositionLatLongAlt",
                "The geographic position, in degrees (lat/lon) and meter (alt), in the order: "
                "latitude, longitude and "
                "altitude",
                Vector3DValue({0, 0, 0}),
                MakeVector3DAccessor(&GeocentricConstantPositionMobilityModel::m_position),
                MakeVector3DChecker())
            .AddAttribute("GeographicReferencePoint",
                          "The point, in meters, taken as reference when converting from "
                          "geographic to topographic.",
                          Vector3DValue({0, 0, 0}),
                          MakeVector3DAccessor(
                              &GeocentricConstantPositionMobilityModel::m_geographicReferencePoint),
                          MakeVector3DChecker());
    return tid;
}

Vector
GeocentricConstantPositionMobilityModel::GetGeographicPosition() const
{
    return DoGetGeographicPosition();
}

void
GeocentricConstantPositionMobilityModel::SetGeographicPosition(const Vector& latLonAlt)
{
    DoSetGeographicPosition(latLonAlt);
}

Vector
GeocentricConstantPositionMobilityModel::GetGeocentricPosition() const
{
    return DoGetGeocentricPosition();
}

void
GeocentricConstantPositionMobilityModel::SetGeocentricPosition(const Vector& position)
{
    DoSetGeocentricPosition(position);
}

double
GeocentricConstantPositionMobilityModel::GetElevationAngle(
    Ptr<const GeocentricConstantPositionMobilityModel> other)
{
    return DoGetElevationAngle(other);
}

void
GeocentricConstantPositionMobilityModel::SetCoordinateTranslationReferencePoint(
    const Vector& position)
{
    DoSetCoordinateTranslationReferencePoint(position);
}

Vector
GeocentricConstantPositionMobilityModel::GetCoordinateTranslationReferencePoint() const
{
    return DoGetCoordinateTranslationReferencePoint();
}

Vector
GeocentricConstantPositionMobilityModel::GetPosition() const
{
    return DoGetPosition();
}

void
GeocentricConstantPositionMobilityModel::SetPosition(const Vector& position)
{
    return DoSetPosition(position);
}

double
GeocentricConstantPositionMobilityModel::GetDistanceFrom(
    Ptr<const GeocentricConstantPositionMobilityModel> other) const
{
    return DoGetDistanceFrom(other);
}

Vector
GeocentricConstantPositionMobilityModel::DoGetPosition() const
{
    Vector topographicCoordinates =
        GeographicPositions::GeographicToTopocentricCoordinates(m_position,
                                                                m_geographicReferencePoint,
                                                                GeographicPositions::SPHERE);
    return topographicCoordinates;
}

void
GeocentricConstantPositionMobilityModel::DoSetPosition(const Vector& position)
{
    Vector geographicCoordinates =
        GeographicPositions::TopocentricToGeographicCoordinates(position,
                                                                m_geographicReferencePoint,
                                                                GeographicPositions::SPHERE);
    m_position = geographicCoordinates;
    NotifyCourseChange();
}

double
GeocentricConstantPositionMobilityModel::DoGetDistanceFrom(
    Ptr<const GeocentricConstantPositionMobilityModel> other) const
{
    Vector cartesianCoordA =
        GeographicPositions::GeographicToCartesianCoordinates(m_position.x,
                                                              m_position.y,
                                                              m_position.z,
                                                              GeographicPositions::SPHERE);
    Vector cartesianCoordB = other->DoGetGeocentricPosition();

    double distance = (cartesianCoordA - cartesianCoordB).GetLength();

    return distance;
}

Vector
GeocentricConstantPositionMobilityModel::DoGetGeographicPosition() const
{
    return m_position;
}

void
GeocentricConstantPositionMobilityModel::DoSetGeographicPosition(const Vector& latLonAlt)
{
    NS_ASSERT_MSG((latLonAlt.x >= -90) && (latLonAlt.x <= 90),
                  "Latitude must be between -90 deg and +90 deg");
    NS_ASSERT_MSG(latLonAlt.z >= 0, "Altitude must be higher or equal than 0 meters");

    m_position = latLonAlt;
    // Normalize longitude to [-180, 180]
    m_position.y = WrapTo180(m_position.y);
    NotifyCourseChange();
}

Vector
GeocentricConstantPositionMobilityModel::DoGetGeocentricPosition() const
{
    Vector geocentricCoordinates =
        GeographicPositions::GeographicToCartesianCoordinates(m_position.x,
                                                              m_position.y,
                                                              m_position.z,
                                                              GeographicPositions::SPHERE);
    return geocentricCoordinates;
}

void
GeocentricConstantPositionMobilityModel::DoSetGeocentricPosition(const Vector& position)
{
    Vector geographicCoordinates =
        GeographicPositions::CartesianToGeographicCoordinates(position,
                                                              GeographicPositions::SPHERE);
    m_position = geographicCoordinates;
    NotifyCourseChange();
}

double
GeocentricConstantPositionMobilityModel::DoGetElevationAngle(
    Ptr<const GeocentricConstantPositionMobilityModel> other)
{
    Vector me = this->DoGetGeocentricPosition();
    Vector them = other->DoGetGeocentricPosition();

    StringValue mode;
    g_geocentricElevationModel.GetValue(mode);
    if (mode.Get() == "Wgs84LowerEnd")
    {
        const Vector meGeo = this->DoGetGeographicPosition();
        const Vector themGeo = other->DoGetGeographicPosition();
        const bool meLower = meGeo.z <= themGeo.z;
        const Vector& lowGeo = meLower ? meGeo : themGeo;
        const Vector& low = meLower ? me : them;
        const Vector& high = meLower ? them : me;
        const double lat = lowGeo.x * M_PI / 180.0;
        const double lon = lowGeo.y * M_PI / 180.0;
        const Vector n(std::cos(lat) * std::cos(lon), std::cos(lat) * std::sin(lon), std::sin(lat));
        Vector d = high - low;
        const double dl = d.GetLength();
        if (dl <= 0.0)
        {
            return 90.0;
        }
        double sx = (n.x * d.x + n.y * d.y + n.z * d.z) / dl;
        sx = std::min(1.0, std::max(-1.0, sx));
        return std::max(0.01, (180.0 * M_1_PI) * std::asin(sx));
    }

    // a is the terminal with the lowest altitude, used as the vertex whose local
    // vertical (the radial direction a/|a|) defines the horizon. The altitude is
    // approximated by the distance from the Earth's centre, not by the ECEF
    // z-coordinate, which is the distance from the equatorial plane
    // (@issueid{1308}).
    Vector& a = (me.GetLength() < them.GetLength() ? me : them);
    Vector& b = (me.GetLength() < them.GetLength() ? them : me);

    Vector bMinusA = b - a;
    // sin(elevation) = (a . (b - a)) / (|a| |b - a|). The dot product is signed:
    // it is negative when the other terminal is below the local horizon, so the
    // sign must be preserved.
    double numerator = a * bMinusA;
    double denominator = a.GetLength() * bMinusA.GetLength();
    double x = numerator / denominator;

    // Enforce the appropriate domain range ([-1, 1]) for the argument of std::asin.
    x = std::min(x, 1.0);
    x = std::max(x, -1.0);

    // asin returns radians, we convert to degrees. A negative result means the
    // other terminal is below the local horizon.
    double elevAngle = (180.0 * M_1_PI) * asin(x);

    return elevAngle;
}

void
GeocentricConstantPositionMobilityModel::DoSetCoordinateTranslationReferencePoint(
    const Vector& refPoint)
{
    m_geographicReferencePoint = refPoint;
}

Vector
GeocentricConstantPositionMobilityModel::DoGetCoordinateTranslationReferencePoint() const
{
    return m_geographicReferencePoint;
}

Vector
GeocentricConstantPositionMobilityModel::DoGetVelocity() const
{
    return Vector(0.0, 0.0, 0.0);
}

} // namespace ns3
