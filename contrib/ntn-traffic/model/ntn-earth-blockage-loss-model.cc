// SPDX-License-Identifier: GPL-2.0-only
// Copyright (c) 2026 Muhammad Uzair
#include "ntn-earth-blockage-loss-model.h"

#include "ns3/double.h"
#include "ns3/geocentric-constant-position-mobility-model.h"
#include "ns3/log.h"
#include "ns3/mobility-model.h"

#include <algorithm>
#include <cmath>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("NtnEarthBlockageLossModel");
NS_OBJECT_ENSURE_REGISTERED(NtnEarthBlockageLossModel);

namespace
{
// WGS-84 (NIMA TR8350.2): semi-major axis and semi-minor axis.
constexpr double kWgsA = 6378137.0;
constexpr double kWgsB = 6356752.314245;

double
Dot(const Vector& u, const Vector& v)
{
    return u.x * v.x + u.y * v.y + u.z * v.z;
}

// Map the ellipsoid onto a sphere of radius a (z stretched by a/b), where the
// surface test is a radius test.
Vector
ToSphere(const Vector& p)
{
    return Vector(p.x, p.y, p.z * (kWgsA / kWgsB));
}
} // namespace

TypeId
NtnEarthBlockageLossModel::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::NtnEarthBlockageLossModel")
            .SetParent<PropagationLossModel>()
            .SetGroupName("NtnTraffic")
            .AddConstructor<NtnEarthBlockageLossModel>()
            .AddAttribute("MinElevationDeg",
                          "Elevation mask at a ground end (deg). 0 = geometric horizon; "
                          "TR 38.821 clause 6.1 evaluates links above a minimum elevation.",
                          DoubleValue(0.0),
                          MakeDoubleAccessor(&NtnEarthBlockageLossModel::m_minElevationDeg),
                          MakeDoubleChecker<double>(-90.0, 90.0))
            .AddAttribute("GroundAltitudeThreshold",
                          "Ends below this geodetic altitude (m) are ground terminals and "
                          "use the elevation mask; higher ends use the limb test.",
                          DoubleValue(5000.0),
                          MakeDoubleAccessor(&NtnEarthBlockageLossModel::m_groundAltitudeM),
                          MakeDoubleChecker<double>(0.0))
            .AddAttribute("MinGrazingAltitude",
                          "Minimum clearance (m) of an air or space link above the "
                          "ellipsoid (limb test). 0 = geometric occultation only.",
                          DoubleValue(0.0),
                          MakeDoubleAccessor(&NtnEarthBlockageLossModel::m_minGrazingAltM),
                          MakeDoubleChecker<double>(0.0))
            .AddAttribute("BlockedLoss",
                          "Loss (dB) applied to a blocked link, large enough that no "
                          "receiver can decode or measure it.",
                          DoubleValue(300.0),
                          MakeDoubleAccessor(&NtnEarthBlockageLossModel::m_blockedLossDb),
                          MakeDoubleChecker<double>(0.0));
    return tid;
}

NtnEarthBlockageLossModel::NtnEarthBlockageLossModel() = default;
NtnEarthBlockageLossModel::~NtnEarthBlockageLossModel() = default;

Vector
NtnEarthBlockageLossModel::EcefOf(Ptr<MobilityModel> m)
{
    if (!m)
    {
        return Vector(0, 0, 0);
    }
    Ptr<GeocentricConstantPositionMobilityModel> g =
        DynamicCast<GeocentricConstantPositionMobilityModel>(m);
    return g ? g->GetGeocentricPosition() : m->GetPosition();
}

double
NtnEarthBlockageLossModel::GeodeticElevationDeg(const Vector& groundEcef, const Vector& otherEcef)
{
    // Outward ellipsoid normal: gradient of x^2/a^2 + y^2/a^2 + z^2/b^2.
    Vector n(groundEcef.x / (kWgsA * kWgsA),
             groundEcef.y / (kWgsA * kWgsA),
             groundEcef.z / (kWgsB * kWgsB));
    const double nl = n.GetLength();
    Vector d = otherEcef - groundEcef;
    const double dl = d.GetLength();
    if (nl <= 0.0 || dl <= 0.0)
    {
        return 90.0;
    }
    const double s = std::clamp(Dot(n, d) / (nl * dl), -1.0, 1.0);
    return std::asin(s) * 180.0 / M_PI;
}

bool
NtnEarthBlockageLossModel::IsBlocked(const Vector& aEcef, const Vector& bEcef, bool& frameOk) const
{
    const Vector as = ToSphere(aEcef);
    const Vector bs = ToSphere(bEcef);
    const double ra = as.GetLength();
    const double rb = bs.GetLength();
    // Not a geocentric frame (an end near the origin): do not judge.
    if (ra < 0.5 * kWgsA || rb < 0.5 * kWgsA)
    {
        frameOk = false;
        return false;
    }
    frameOk = true;
    const bool aLower = ra <= rb;
    const Vector& lowE = aLower ? aEcef : bEcef;
    const Vector& highE = aLower ? bEcef : aEcef;
    const Vector& lowS = aLower ? as : bs;
    const Vector& highS = aLower ? bs : as;
    // Scaled radius minus a is the altitude on the stretched sphere (exact on
    // the surface, a close proxy above it).
    const double lowAlt = lowS.GetLength() - kWgsA;
    const double highAlt = highS.GetLength() - kWgsA;
    if (highAlt < m_groundAltitudeM)
    {
        // Both ends are ground terminals (e.g. UE-to-UE interference): a
        // terrestrial path, not an NTN link. Earth curvature over such paths
        // is the terrestrial channel model's business (two antennas at 0 m
        // altitude always see each other a hair below the horizon), so it is
        // not judged here.
        return false;
    }
    if (lowAlt < m_groundAltitudeM)
    {
        return GeodeticElevationDeg(lowE, highE) < m_minElevationDeg;
    }
    // Air or space link: closest approach of the open segment to the centre.
    const Vector d = highS - lowS;
    const double dd = Dot(d, d);
    if (dd <= 0.0)
    {
        return false;
    }
    const double t = -Dot(lowS, d) / dd;
    if (t <= 0.0 || t >= 1.0)
    {
        return false; // closest point is an end, which is above the surface
    }
    const Vector c(lowS.x + t * d.x, lowS.y + t * d.y, lowS.z + t * d.z);
    return c.GetLength() < kWgsA + m_minGrazingAltM;
}

double
NtnEarthBlockageLossModel::DoCalcRxPower(double txPowerDbm,
                                         Ptr<MobilityModel> a,
                                         Ptr<MobilityModel> b) const
{
    bool frameOk = true;
    const bool blocked = IsBlocked(EcefOf(a), EcefOf(b), frameOk);
    if (!frameOk)
    {
        ++m_nonGeocentric;
        return txPowerDbm;
    }
    if (blocked)
    {
        ++m_blocked;
        return txPowerDbm - m_blockedLossDb;
    }
    ++m_clear;
    return txPowerDbm;
}

int64_t
NtnEarthBlockageLossModel::DoAssignStreams(int64_t /*stream*/)
{
    return 0;
}

} // namespace ns3
