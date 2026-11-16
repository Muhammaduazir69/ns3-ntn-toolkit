/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#ifndef THZ_NTN_LINK_GEOMETRY_H
#define THZ_NTN_LINK_GEOMETRY_H

#include "ns3/vector.h"

namespace ns3
{

class MobilityModel;

/**
 * \ingroup thz-ntn
 * \brief Earth-space link geometry shared by the spectrum-level loss models.
 *
 * Two frames are recognised, the same convention the ntn-traffic Earth
 * blockage model uses:
 *  - ECEF (both endpoints farther than half an Earth radius from the origin,
 *    e.g. Sgp4MobilityModel or GeocentricConstantPositionMobilityModel):
 *    the lower end is the one with the lower WGS-84 geodetic altitude, the
 *    elevation is taken about the WGS-84 ellipsoid normal at that end, and the
 *    path direction is expressed in the local ENU frame of the lower end.
 *  - flat local ENU (z up, metres; the frame NtnEnuProjectionMobilityModel
 *    produces): the lower end is the one with the smaller z, the elevation is
 *    atan2(dz, horizontal distance), and altitudes are the z values.
 */
struct ThzNtnLinkGeometry
{
    bool ecef{false};          //!< true when both ends are ECEF
    double elevationDeg{90.0}; //!< elevation of the upper end seen from the lower end
    double lowerAltKm{0.0};    //!< altitude of the lower end (km)
    double upperAltKm{0.0};    //!< altitude of the upper end (km)
    double slantM{0.0};        //!< straight-line distance (m)
    double lowerLatDeg{0.0};   //!< geodetic latitude of the lower end (ECEF only)
    double lowerLonDeg{0.0};   //!< geodetic longitude of the lower end (ECEF only)
    Vector lowerEnu;           //!< lower end in the caller's flat frame (flat only)
    Vector dirEnu;             //!< unit vector lower -> upper in local ENU

    /// Compute the geometry of the (a, b) link at the current simulation time.
    static ThzNtnLinkGeometry Compute(const MobilityModel* a, const MobilityModel* b);

    /// Same, from two positions (flat or ECEF, decided from their norms).
    static ThzNtnLinkGeometry FromPositions(const Vector& pa, const Vector& pb);

    /// ECEF for geocentric models (GetGeocentricPosition), GetPosition() otherwise.
    static Vector PositionOf(const MobilityModel* m);

    /// WGS-84 ECEF (m) to geodetic (lat deg, lon deg, alt m), Bowring.
    static Vector EcefToGeodetic(const Vector& ecef);
};

} // namespace ns3

#endif // THZ_NTN_LINK_GEOMETRY_H
