/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Author: Muhammad Uzair
 */

#include "ntn-orbit-predictor.h"

#include "ns3/satellite-constant-position-mobility-model.h"

#include <ns3/double.h>
#include <ns3/log.h>
#include <ns3/satellite-sgp4-mobility-model.h>
#include <ns3/simulator.h>

#include <algorithm>
#include <cmath>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("NtnOrbitPredictor");

Vector
NtnOrbitPredictor::PropagateTwoBodyEcef(const Vector& rEcef, const Vector& vEcef, double dtS)
{
    constexpr double kEarthRotRadPerS = 7.2921159e-5;

    if (dtS == 0.0)
    {
        return rEcef;
    }

    // 1. ECEF velocity -> inertial velocity: v_eci = v_ecef + omega x r.
    const Vector vEci(vEcef.x - kEarthRotRadPerS * rEcef.y,
                      vEcef.y + kEarthRotRadPerS * rEcef.x,
                      vEcef.z);

    const double rMag = std::sqrt(rEcef.x * rEcef.x + rEcef.y * rEcef.y + rEcef.z * rEcef.z);
    const double vMag = std::sqrt(vEci.x * vEci.x + vEci.y * vEci.y + vEci.z * vEci.z);
    if (rMag <= 0.0 || vMag <= 0.0)
    {
        // Degenerate state (a stationary or unpositioned satellite): fall back
        // to the linear form rather than dividing by zero.
        return Vector(rEcef.x + vEcef.x * dtS, rEcef.y + vEcef.y * dtS, rEcef.z + vEcef.z * dtS);
    }

    // 2. Rotate r about the orbit normal n = (r x v)/|r x v| by theta = w*dt.
    Vector n(rEcef.y * vEci.z - rEcef.z * vEci.y,
             rEcef.z * vEci.x - rEcef.x * vEci.z,
             rEcef.x * vEci.y - rEcef.y * vEci.x);
    const double nMag = std::sqrt(n.x * n.x + n.y * n.y + n.z * n.z);
    if (nMag <= 0.0)
    {
        // Radial motion only: no orbit plane to rotate in.
        return Vector(rEcef.x + vEcef.x * dtS, rEcef.y + vEcef.y * dtS, rEcef.z + vEcef.z * dtS);
    }
    n = Vector(n.x / nMag, n.y / nMag, n.z / nMag);

    const double theta = (vMag / rMag) * dtS;
    const double c = std::cos(theta);
    const double sN = std::sin(theta);
    const double nDotR = n.x * rEcef.x + n.y * rEcef.y + n.z * rEcef.z;
    const Vector nCrossR(n.y * rEcef.z - n.z * rEcef.y,
                         n.z * rEcef.x - n.x * rEcef.z,
                         n.x * rEcef.y - n.y * rEcef.x);
    const Vector rotated(rEcef.x * c + nCrossR.x * sN + n.x * nDotR * (1.0 - c),
                         rEcef.y * c + nCrossR.y * sN + n.y * nDotR * (1.0 - c),
                         rEcef.z * c + nCrossR.z * sN + n.z * nDotR * (1.0 - c));

    // 3. Rotate back through the Earth's own turn over dt.
    const double g = kEarthRotRadPerS * dtS;
    return Vector(rotated.x * std::cos(g) + rotated.y * std::sin(g),
                  -rotated.x * std::sin(g) + rotated.y * std::cos(g),
                  rotated.z);
}

void
NtnOrbitPredictor::SetGeometricBeam(double peakGainDbi, double beamwidth3dbDeg)
{
    NS_ABORT_MSG_IF(beamwidth3dbDeg <= 0.0, "3 dB beamwidth must be positive");
    m_geometricBeam = true;
    m_beamPeakGainDbi = peakGainDbi;
    m_beam3dbDeg = beamwidth3dbDeg;
}

double
NtnOrbitPredictor::GeometricBeamGainDb(const GeoCoordinate& uePosition,
                                       const GeoCoordinate& satPosition) const
{
    // TR 38.811 Section 6.4.1 aperture pattern, normalized so G(0) = 0 dB and
    // G(theta_3dB / 2) = -3.01 dB, referenced to the configured peak gain.
    const Vector sat = satPosition.ToVector();
    const Vector ue = uePosition.ToVector();
    const double satMag = std::sqrt(sat.x * sat.x + sat.y * sat.y + sat.z * sat.z);
    if (satMag <= 0.0)
    {
        return -100.0;
    }
    // Boresight is nadir: from the satellite toward the geocentre.
    const Vector nadir(-sat.x / satMag, -sat.y / satMag, -sat.z / satMag);
    const Vector toUe(ue.x - sat.x, ue.y - sat.y, ue.z - sat.z);
    const double toUeMag = std::sqrt(toUe.x * toUe.x + toUe.y * toUe.y + toUe.z * toUe.z);
    if (toUeMag <= 0.0)
    {
        return 0.0; // at boresight, relative gain is 0 dB
    }
    double cosTheta = (nadir.x * toUe.x + nadir.y * toUe.y + nadir.z * toUe.z) / toUeMag;
    cosTheta = std::max(-1.0, std::min(1.0, cosTheta));
    const double theta = std::acos(cosTheta);

    const double half3db = 0.5 * m_beam3dbDeg * M_PI / 180.0;
    const double denom = std::sin(half3db);
    if (denom <= 0.0)
    {
        return 0.0; // relative convention: boresight is 0 dB
    }
    double rolloffDb = 0.0;
    if (m_steeredBeam)
    {
        // Boresight tracks the terminal, so the aperture pattern contributes
        // 0 dB and what remains is phased-array scan loss as the beam is
        // steered `theta` off the array normal (nadir): G ~ cos^n(theta).
        // Beam exit then happens when the satellite nears the horizon, which is
        // the event a LEO time-to-exit is actually about.
        const double cosScan = std::max(std::cos(theta), 1e-6);
        rolloffDb = 10.0 * m_scanLossExp * std::log10(cosScan);
    }
    else
    {
        const double u = 1.6163 * std::sin(theta) / denom;
        if (u > 1e-9)
        {
            const double j1 = std::cyl_bessel_j(1.0, u);
            const double g = 4.0 * (j1 / u) * (j1 / u);
            rolloffDb = 10.0 * std::log10(std::max(g, 1e-12));
        }
    }
    // Floor the far-out tail the way the sat beam model does, so a terminal far
    // outside the beam reports a finite, very low value rather than -inf.
    rolloffDb = std::max(rolloffDb, -40.0);

    // RELATIVE to boresight, matching the SNS3 pattern convention this
    // substitutes for. Those grids are normalized so the peak is ~0 dB, which
    // is why ChoConfig::gainThreshold_dB defaults to -3.0: the half-power
    // contour. Returning an ABSOLUTE gain here instead would put the value on a
    // different scale from every threshold that reads it, and a -3 dB threshold
    // against a 30 dBi peak can never be crossed. The configured peak gain is
    // kept for callers that want the absolute figure via GetBeamPeakGainDbi().
    return rolloffDb;
}

double
NtnOrbitPredictor::GainThresholdForMinElevationDb(double minElevDeg, double satAltM) const
{
    // Geometry: for a satellite at altitude h seen at elevation e, the off-nadir
    // (scan) angle theta satisfies sin(theta) = Re/(Re+h) * cos(e).
    constexpr double kRe = 6371000.0;
    const double e = minElevDeg * M_PI / 180.0;
    const double sinTheta = (kRe / (kRe + satAltM)) * std::cos(e);
    const double theta = std::asin(std::max(-1.0, std::min(1.0, sinTheta)));

    if (m_steeredBeam)
    {
        const double cosScan = std::max(std::cos(theta), 1e-6);
        return 10.0 * m_scanLossExp * std::log10(cosScan);
    }
    const double half3db = 0.5 * m_beam3dbDeg * M_PI / 180.0;
    const double denom = std::sin(half3db);
    if (denom <= 0.0)
    {
        return 0.0;
    }
    const double u = 1.6163 * std::sin(theta) / denom;
    if (u <= 1e-9)
    {
        return 0.0;
    }
    const double j1 = std::cyl_bessel_j(1.0, u);
    const double g = 4.0 * (j1 / u) * (j1 / u);
    return std::max(10.0 * std::log10(std::max(g, 1e-12)), -40.0);
}

void
NtnOrbitPredictor::SetSteeredBeam(bool steered, double scanLossExponent)
{
    m_steeredBeam = steered;
    m_scanLossExp = scanLossExponent;
}

void
NtnOrbitPredictor::SetKinematicsSource(uint32_t satId, Ptr<MobilityModel> mob)
{
    if (!mob)
    {
        return;
    }
    const Vector v = mob->GetVelocity();
    if (v.x == 0.0 && v.y == 0.0 && v.z == 0.0)
    {
        NS_LOG_WARN("NtnOrbitPredictor: refusing a kinematics source for satellite "
                    << satId
                    << " that reports zero velocity. A stationary source cannot be "
                       "forward-propagated, so every time-to-exit derived from it would be a "
                       "constant. Register the real orbital model instead (CHO-2).");
        return;
    }
    m_kinematics[satId] = mob;
}

bool
NtnOrbitPredictor::HasUsableKinematics(uint32_t satId) const
{
    auto it = m_kinematics.find(satId);
    if (it != m_kinematics.end())
    {
        return true; // SetKinematicsSource already rejected stationary sources
    }
    // Fall back to the SatMobilityModel: usable only if it actually moves.
    Ptr<SatMobilityModel> sm = GetSatelliteMobility(satId);
    if (!sm)
    {
        return false;
    }
    const Vector v = sm->GetVelocity();
    return !(v.x == 0.0 && v.y == 0.0 && v.z == 0.0);
}

uint32_t
NtnOrbitPredictor::CountFrozenSatellites() const
{
    uint32_t frozen = 0;
    for (uint32_t i = 0; i < m_satellites.GetN(); ++i)
    {
        if (!HasUsableKinematics(i))
        {
            ++frozen;
        }
    }
    return frozen;
}

NS_OBJECT_ENSURE_REGISTERED(NtnOrbitPredictor);

TypeId
NtnOrbitPredictor::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::NtnOrbitPredictor")
            .SetParent<Object>()
            .SetGroupName("NtnCho")
            .AddConstructor<NtnOrbitPredictor>()
            .AddAttribute("MinGainThreshold",
                          "Minimum antenna gain in dB for valid beam coverage",
                          DoubleValue(-3.0),
                          MakeDoubleAccessor(&NtnOrbitPredictor::m_minGainThreshold_dB),
                          MakeDoubleChecker<double>(-30.0, 30.0));
    return tid;
}

NtnOrbitPredictor::NtnOrbitPredictor()
    : m_minGainThreshold_dB(-3.0),
      m_initialized(false)
{
    NS_LOG_FUNCTION(this);
}

NtnOrbitPredictor::~NtnOrbitPredictor()
{
    NS_LOG_FUNCTION(this);
}

void
NtnOrbitPredictor::DoDispose()
{
    NS_LOG_FUNCTION(this);
    m_agpContainer = nullptr;
    Object::DoDispose();
}

void
NtnOrbitPredictor::Initialize(NodeContainer satellites,
                              Ptr<SatAntennaGainPatternContainer> agpContainer)
{
    NS_LOG_FUNCTION(this << satellites.GetN());
    m_satellites = satellites;
    m_agpContainer = agpContainer;
    m_initialized = true;
    NS_LOG_INFO("NtnOrbitPredictor initialized with " << satellites.GetN() << " satellites");
}

NtnOrbitPredictor::BeamSnapshot
NtnOrbitPredictor::GetBeamSnapshot(uint32_t satId,
                                   uint32_t beamId,
                                   GeoCoordinate uePosition) const
{
    NS_LOG_FUNCTION(this << satId << beamId);
    NS_ASSERT_MSG(m_initialized, "NtnOrbitPredictor not initialized. Call Initialize() first.");

    BeamSnapshot snap;
    snap.satId = satId;
    snap.beamId = beamId;

    // Get satellite mobility model.
    // CHO-2: prefer the registered kinematics source, exactly as
    // GetBeamSnapshotAtTime does. Reading the SatMobilityModel here instead
    // would return the aux stand-in's position, which scenarios do not
    // meaningfully place (it exists only to satisfy the pattern container), and
    // an unset stand-in sits at the origin whose geodetic latitude is NaN.
    Ptr<SatMobilityModel> satMob = GetSatelliteMobility(satId);
    GeoCoordinate satPos;
    auto kinNow = m_kinematics.find(satId);
    if (kinNow != m_kinematics.end())
    {
        satPos = GeoCoordinate(kinNow->second->GetPosition());
    }
    else if (satMob)
    {
        satPos = satMob->GetGeoPosition();
    }
    else
    {
        // Neither source knows this satellite. Return a snapshot that cannot
        // pass any threshold rather than dereferencing null: -100 dB of gain
        // and a below-horizon elevation are the same "unusable" convention
        // ComputeBeamGain already uses, so every caller's existing comparison
        // rejects it without needing a new flag on the struct.
        snap.gainAtUe_dB = -100.0;
        snap.elevationAngle_deg = -90.0;
        snap.slantRange_km = 0.0;
        snap.propagationDelay = Seconds(0);
        return snap;
    }
    snap.satellitePosition = satPos;

    // Compute beam center using antenna gain pattern
    // CHO-2: with the analytic steered beam the boresight is the terminal and
    // the footprint centre is the sub-satellite point, so the GEO-authored
    // pattern grid is neither needed nor valid here. Touching it would abort
    // on beam ids the grid does not define.
    Ptr<SatAntennaGainPattern> agp =
        m_geometricBeam ? nullptr : m_agpContainer->GetAntennaGainPattern(beamId);
    if (agp)
    {
        snap.beamCenter = GeoCoordinate(agp->GetCenterLatitude(satMob),
                                        agp->GetCenterLongitude(satMob),
                                        0.0);
    }
    else
    {
        snap.beamCenter = GeoCoordinate(satPos.GetLatitude(), satPos.GetLongitude(), 0.0);
    }

    // Compute gain at UE position
    snap.gainAtUe_dB = ComputeBeamGain(satId, beamId, uePosition);

    // Compute geometric parameters
    snap.elevationAngle_deg = ComputeElevationAngle(uePosition, satId);
    snap.slantRange_km = Compute3dDistance(uePosition, satPos) / 1000.0;
    snap.propagationDelay = ComputePropagationDelay(uePosition, satId);

    return snap;
}

NtnOrbitPredictor::BeamSnapshot
NtnOrbitPredictor::GetBeamSnapshotAtTime(uint32_t satId,
                                          uint32_t beamId,
                                          GeoCoordinate uePosition,
                                          Time timeOffset) const
{
    NS_LOG_FUNCTION(this << satId << beamId << timeOffset.GetSeconds());
    NS_ASSERT_MSG(m_initialized, "NtnOrbitPredictor not initialized.");

    BeamSnapshot snap;
    snap.satId = satId;
    snap.beamId = beamId;

    // CHO-1 FIX (2026-08-24). This used to forward-propagate with first-order
    // kinematics, r(t+dt) = r(t) + v(t)*dt, under a comment claiming the drift
    // was "< 30 m over 120 s". That was wrong by roughly three orders of
    // magnitude, and the comment's own formula (dt^2 * v^2 / r) evaluates to
    // 112 km, not the 60 m it was labelled. For a 780 km shell the true
    // straight-line error is:
    //
    //     dt =  10 s ->   0.4 km        dt =  60 s ->  12.9 km
    //     dt =  30 s ->   3.2 km        dt = 120 s ->  51.7 km
    //
    // A LEO service beam has a footprint radius of tens of kilometres, so at
    // the 120 s horizon the predicted position was leaving the beam it was
    // supposed to be predicting exit from: the TTE estimate that depends on it
    // was not merely imprecise, it was meaningless.
    //
    // We cannot fix this by calling SGP4 at the offset epoch: that machinery
    // lives in the vendored `satellite` module, which is not tracked in this
    // repository, so a change there would not reach anyone. Instead propagate
    // the two-body way, entirely inside this module:
    //   1. lift the ECEF velocity into an inertial frame, v_eci = v + w x r;
    //   2. rotate r about the orbit normal by theta = |v_eci| / |r| * dt
    //      (Rodrigues), which is exact for a circular orbit and second-order
    //      for a near-circular one;
    //   3. rotate back through the Earth's own turn over dt.
    // Validated against an analytic circular truth model: sub-metre out to
    // 120 s, against 51.7 km for the code this replaces. Residual error against
    // real SGP4 is the eccentricity and J2 term, far below a beam radius.
    Ptr<SatMobilityModel> satMob = GetSatelliteMobility(satId);
    Vector satCartNow;
    Vector satVel;
    auto kin = m_kinematics.find(satId);
    if (kin != m_kinematics.end())
    {
        // CHO-2: real orbital kinematics, registered separately because the
        // toolkit's SGP4 model is not a SatMobilityModel.
        satCartNow = kin->second->GetPosition();
        satVel = kin->second->GetVelocity();
    }
    else if (satMob)
    {
        satCartNow = satMob->GetGeoPosition().ToVector();
        satVel = satMob->GetVelocity();
    }
    else
    {
        // As in GetBeamSnapshot: neither source knows this satellite, so return
        // a snapshot no threshold can accept.
        snap.gainAtUe_dB = -100.0;
        snap.elevationAngle_deg = -90.0;
        snap.slantRange_km = 0.0;
        snap.propagationDelay = Seconds(0);
        return snap;
    }
    const double dt = timeOffset.GetSeconds();
    Vector satCartFuture = PropagateTwoBodyEcef(satCartNow, satVel, dt);
    GeoCoordinate satPosFuture(satCartFuture);
    snap.satellitePosition = satPosFuture;

    // Beam center scrolls with the satellite footprint at the same dt.
    // CHO-2: with the analytic steered beam the boresight is the terminal and
    // the footprint centre is the sub-satellite point, so the GEO-authored
    // pattern grid is neither needed nor valid here. Touching it would abort
    // on beam ids the grid does not define.
    Ptr<SatAntennaGainPattern> agp =
        m_geometricBeam ? nullptr : m_agpContainer->GetAntennaGainPattern(beamId);
    if (agp)
    {
        // CHO-11. Both branches used to assign the identical value, so the
        // antenna pattern was ignored and the FUTURE beam centre was always the
        // sub-satellite point - while the present-time sibling GetBeamSnapshot
        // takes it from agp->GetCenterLatitude/Longitude. A distance-based
        // trigger therefore compared a pattern-derived centre now against a
        // nadir centre later, and the discontinuity between them was read as
        // beam motion.
        //
        // The comment above already stated the right physics and the code did
        // not implement it: the pattern is body-fixed, so its centre keeps a
        // constant offset from the sub-satellite point. Take that offset at the
        // present instant and carry it to the projected sub-satellite point.
        const GeoCoordinate satPosNow(satCartNow);
        const double dLat = agp->GetCenterLatitude(satMob) - satPosNow.GetLatitude();
        double dLon = agp->GetCenterLongitude(satMob) - satPosNow.GetLongitude();
        // Longitude difference on the shorter arc, so a beam sitting across the
        // antimeridian does not acquire a 360-degree offset.
        while (dLon > 180.0)
        {
            dLon -= 360.0;
        }
        while (dLon < -180.0)
        {
            dLon += 360.0;
        }
        double lon = satPosFuture.GetLongitude() + dLon;
        while (lon > 180.0)
        {
            lon -= 360.0;
        }
        while (lon < -180.0)
        {
            lon += 360.0;
        }
        const double lat =
            std::max(-90.0, std::min(90.0, satPosFuture.GetLatitude() + dLat));
        snap.beamCenter = GeoCoordinate(lat, lon, 0.0);
    }
    else
    {
        // Steered/geometric beam: the boresight is the terminal and the
        // footprint centre IS the sub-satellite point.
        snap.beamCenter = GeoCoordinate(satPosFuture.GetLatitude(),
                                        satPosFuture.GetLongitude(), 0.0);
    }

    // Geometric quantities from the propagated satellite position.
    Vector ueCart = uePosition.ToVector();
    Vector toSat(satCartFuture.x - ueCart.x,
                 satCartFuture.y - ueCart.y,
                 satCartFuture.z - ueCart.z);
    double dist = std::sqrt(toSat.x * toSat.x + toSat.y * toSat.y +
                            toSat.z * toSat.z);
    snap.slantRange_km = dist / 1000.0;
    double ueNorm = std::sqrt(ueCart.x * ueCart.x + ueCart.y * ueCart.y +
                              ueCart.z * ueCart.z);
    if (ueNorm > 0.0 && dist > 0.0)
    {
        double cosZenith = (toSat.x * ueCart.x + toSat.y * ueCart.y +
                            toSat.z * ueCart.z) /
                           (dist * ueNorm);
        snap.elevationAngle_deg =
            std::asin(std::max(-1.0, std::min(1.0, cosZenith))) * 180.0 / M_PI;
    }
    else
    {
        snap.elevationAngle_deg = 0.0;
    }
    snap.propagationDelay = Seconds(dist / SPEED_OF_LIGHT);

    // ---- GAP C1 FIX: evaluate the beam gain at the PROPAGATED geometry ----
    //
    // This used to be ComputeBeamGain(satId, beamId, uePosition), i.e. the gain
    // at the satellite's position RIGHT NOW, with a comment claiming the UE had
    // been "shifted back along the ground track" — no such shift was ever
    // performed. Everything else in this snapshot (slant range, elevation,
    // delay) correctly used satCartFuture, so only the gain was frozen. Since
    // the TTE estimator's whole forward search keys off this gain, TTE came back
    // as the full prediction window for every beam, which in turn made the
    // condEventT1 trigger unreachable and the TTE-aware admission a no-op.
    //
    // The SNS3 pattern is body-fixed and takes the satellite mobility to resolve
    // the geometry, so evaluating it against a temporary mobility placed at the
    // extrapolated position gives the gain the UE will see at t+dt.
    snap.gainAtUe_dB = ComputeBeamGainAt(satId, beamId, uePosition, satPosFuture);
    return snap;
}

double
NtnOrbitPredictor::ComputeBeamGainAt(uint32_t satId,
                                     uint32_t beamId,
                                     GeoCoordinate uePosition,
                                     GeoCoordinate satPosition) const
{
    // C1: gain of `beamId` at `uePosition` with the satellite placed at
    // `satPosition` (rather than wherever it happens to be at Simulator::Now()).
    //
    // CHO-2: when the analytic beam is configured, use it. The shipped SNS3
    // pattern grids are GEO-referenced and return NaN for a LEO geometry, which
    // floors the gain and collapses every TTE to zero.
    if (m_geometricBeam)
    {
        return GeometricBeamGainDb(uePosition, satPosition);
    }
    if (!m_agpContainer)
    {
        return -100.0;
    }
    // CHO-2: with the analytic steered beam the boresight is the terminal and
    // the footprint centre is the sub-satellite point, so the GEO-authored
    // pattern grid is neither needed nor valid here. Touching it would abort
    // on beam ids the grid does not define.
    Ptr<SatAntennaGainPattern> agp =
        m_geometricBeam ? nullptr : m_agpContainer->GetAntennaGainPattern(beamId);
    if (!agp)
    {
        return -100.0;
    }
    Ptr<SatConstantPositionMobilityModel> tempMob =
        CreateObject<SatConstantPositionMobilityModel>();
    tempMob->SetGeoPosition(satPosition);
    const double gainLin = agp->GetAntennaGain_lin(uePosition, tempMob);
    // NaN guard, not just <= 0: the SNS3 pattern returns NaN when the UE falls
    // outside the sampled grid, which is EXACTLY the beam-exit case this
    // function exists to detect. `NaN <= 0.0` is false, so without this the NaN
    // would sail through std::log10 and poison the TTE search with a NaN gain
    // (comparisons against a threshold then silently fail and no exit is ever
    // found — the same symptom as the C1 bug this fix is for).
    if (!std::isfinite(gainLin) || gainLin <= 0.0)
    {
        return -100.0; // outside coverage
    }
    return 10.0 * std::log10(gainLin);
}

std::vector<NtnOrbitPredictor::VisibleSatellite>
NtnOrbitPredictor::GetVisibleSatellites(GeoCoordinate uePosition,
                                        double minElevation_deg) const
{
    NS_LOG_FUNCTION(this << minElevation_deg);
    NS_ASSERT_MSG(m_initialized, "NtnOrbitPredictor not initialized.");

    std::vector<VisibleSatellite> visible;

    for (uint32_t satId = 0; satId < m_satellites.GetN(); satId++)
    {
        double elev = ComputeElevationAngle(uePosition, satId);
        if (elev < minElevation_deg)
        {
            continue;
        }

        // Find best beam for this satellite
        uint32_t bestBeamId = m_agpContainer->GetBestBeamId(satId, uePosition, true);
        double bestGain = m_agpContainer->GetBeamGain(satId, bestBeamId, uePosition);
        double bestGain_dB = (bestGain > 0) ? 10.0 * std::log10(bestGain) : -100.0;

        if (bestGain_dB >= m_minGainThreshold_dB)
        {
            VisibleSatellite vs;
            vs.satId = satId;
            vs.bestBeamId = bestBeamId;
            vs.bestGain_dB = bestGain_dB;
            vs.elevationAngle_deg = elev;
            vs.slantRange_km =
                Compute3dDistance(uePosition, GetSatelliteMobility(satId)->GetGeoPosition()) /
                1000.0;
            visible.push_back(vs);
        }
    }

    // Sort by gain descending
    std::sort(visible.begin(), visible.end(), [](const VisibleSatellite& a, const VisibleSatellite& b) {
        return a.bestGain_dB > b.bestGain_dB;
    });

    NS_LOG_INFO("Found " << visible.size() << " visible satellites from position ("
                          << uePosition.GetLatitude() << ", " << uePosition.GetLongitude() << ")");
    return visible;
}

std::vector<std::pair<Time, GeoCoordinate>>
NtnOrbitPredictor::PredictBeamTrack(uint32_t satId,
                                    uint32_t beamId,
                                    Time startOffset,
                                    Time endOffset,
                                    Time step) const
{
    NS_LOG_FUNCTION(this << satId << beamId);
    NS_ASSERT_MSG(m_initialized, "NtnOrbitPredictor not initialized.");

    std::vector<std::pair<Time, GeoCoordinate>> track;
    Ptr<SatMobilityModel> satMob = GetSatelliteMobility(satId);
    // CHO-2: with the analytic steered beam the boresight is the terminal and
    // the footprint centre is the sub-satellite point, so the GEO-authored
    // pattern grid is neither needed nor valid here. Touching it would abort
    // on beam ids the grid does not define.
    Ptr<SatAntennaGainPattern> agp =
        m_geometricBeam ? nullptr : m_agpContainer->GetAntennaGainPattern(beamId);

    for (Time t = startOffset; t <= endOffset; t += step)
    {
        GeoCoordinate beamCenter;
        if (agp)
        {
            beamCenter = GeoCoordinate(agp->GetCenterLatitude(satMob),
                                       agp->GetCenterLongitude(satMob),
                                       0.0);
        }
        else
        {
            // CHO-2 follow-up: honour the kinematics source, and do not
            // dereference a mobility model that a kinematics-only scenario
            // never registered.
            GeoCoordinate satPos;
            if (!ResolveSatPosition(satId, satPos))
            {
                continue;
            }
            beamCenter = GeoCoordinate(satPos.GetLatitude(), satPos.GetLongitude(), 0.0);
        }
        track.push_back(std::make_pair(t, beamCenter));
    }

    return track;
}

double
NtnOrbitPredictor::ComputeBeamGain(uint32_t satId,
                                   uint32_t beamId,
                                   GeoCoordinate uePosition) const
{
    NS_LOG_FUNCTION(this << satId << beamId);

    // CHO-2: the analytic beam must apply here too. This is the gain the TTE
    // estimator reads in its step-0 coverage check, while the forward search
    // reads ComputeBeamGainAt(). Leaving the two on different beam models made
    // the estimator compare a GEO pattern-grid value against an analytic one,
    // so a terminal that step 0 judged well inside the beam was judged outside
    // it one search step later and the time-to-exit collapsed to zero.
    if (m_geometricBeam)
    {
        Vector satEcef;
        auto kin = m_kinematics.find(satId);
        if (kin != m_kinematics.end())
        {
            satEcef = kin->second->GetPosition();
        }
        else
        {
            Ptr<SatMobilityModel> sm = GetSatelliteMobility(satId);
            if (!sm)
            {
                return -100.0;
            }
            satEcef = sm->GetGeoPosition().ToVector();
        }
        return GeometricBeamGainDb(uePosition, GeoCoordinate(satEcef));
    }

    double gain_lin = m_agpContainer->GetBeamGain(satId, beamId, uePosition);
    // Same NaN guard as ComputeBeamGainAt: the SNS3 pattern returns NaN outside
    // its sampled grid, and `NaN <= 0.0` is false, so a bare <= 0 check lets NaN
    // through std::log10 and out into the caller's threshold comparisons.
    if (!std::isfinite(gain_lin) || gain_lin <= 0.0)
    {
        return -100.0; // Very low gain indicates outside coverage
    }
    return 10.0 * std::log10(gain_lin);
}

// CHO-2 follow-up: resolve a satellite's position from whichever source the
// scenario registered.
//
// Two registration paths exist and only one was honoured here.
// SetKinematicsSource() was added by CHO-2 precisely because the toolkit's SGP4
// model is not a SatMobilityModel, and GetBeamSnapshot / PredictBeamSnapshot
// both consult m_kinematics first. But ComputeElevationAngle and
// ComputePropagationDelay went straight to GetSatelliteMobility() and
// dereferenced the result with no fallback and no null check, so a predictor
// configured the modern way - kinematics only, no SatMobilityModel - segfaulted
// the moment anything asked for an elevation. That is the configuration
// CHO-2 introduced and the one a geometric-beam scenario uses.
//
// Returns false when neither source knows the satellite, so callers can decline
// rather than crash.
bool
NtnOrbitPredictor::ResolveSatPosition(uint32_t satId, GeoCoordinate& out) const
{
    auto kin = m_kinematics.find(satId);
    if (kin != m_kinematics.end() && kin->second)
    {
        out = GeoCoordinate(kin->second->GetPosition());
        return true;
    }
    Ptr<SatMobilityModel> satMob = GetSatelliteMobility(satId);
    if (!satMob)
    {
        return false;
    }
    out = satMob->GetGeoPosition();
    return true;
}

double
NtnOrbitPredictor::ComputeElevationAngle(GeoCoordinate uePosition, uint32_t satId) const
{
    NS_LOG_FUNCTION(this << satId);

    GeoCoordinate satPos;
    if (!ResolveSatPosition(satId, satPos))
    {
        // Below the horizon is the honest answer for a satellite nothing knows
        // about, and it keeps every caller's threshold comparison meaningful.
        return -90.0;
    }

    // Convert to Cartesian (ECEF)
    Vector ueCart = uePosition.ToVector();
    Vector satCart = satPos.ToVector();

    // Vector from UE to satellite
    Vector toSat(satCart.x - ueCart.x, satCart.y - ueCart.y, satCart.z - ueCart.z);

    // UE normal vector (radial direction from Earth center)
    double ueNorm = std::sqrt(ueCart.x * ueCart.x + ueCart.y * ueCart.y + ueCart.z * ueCart.z);
    Vector ueNormal(ueCart.x / ueNorm, ueCart.y / ueNorm, ueCart.z / ueNorm);

    // Distance to satellite
    double dist = std::sqrt(toSat.x * toSat.x + toSat.y * toSat.y + toSat.z * toSat.z);
    if (dist < 1.0)
    {
        return 90.0;
    }

    // Dot product gives cos(zenith angle)
    double cosZenith =
        (toSat.x * ueNormal.x + toSat.y * ueNormal.y + toSat.z * ueNormal.z) / dist;

    // Elevation = 90 - zenith angle
    double elevRad = std::asin(std::max(-1.0, std::min(1.0, cosZenith)));
    return elevRad * 180.0 / M_PI;
}

Time
NtnOrbitPredictor::ComputePropagationDelay(GeoCoordinate uePosition, uint32_t satId) const
{
    NS_LOG_FUNCTION(this << satId);

    GeoCoordinate satPos;
    if (!ResolveSatPosition(satId, satPos))
    {
        return Seconds(0);
    }

    double distance_m = Compute3dDistance(uePosition, satPos);
    double delay_s = distance_m / SPEED_OF_LIGHT;

    return Seconds(delay_s);
}

uint32_t
NtnOrbitPredictor::GetNumBeamsPerSat() const
{
    return m_agpContainer ? m_agpContainer->GetNAntennaGainPatterns() : 0;
}

uint32_t
NtnOrbitPredictor::GetNumSatellites() const
{
    return m_satellites.GetN();
}

Ptr<Node>
NtnOrbitPredictor::GetSatelliteNode(uint32_t satId) const
{
    // CHO-2 follow-up: return null rather than indexing past the end.
    //
    // The bounds check was an NS_ASSERT, which is compiled OUT of the optimized
    // build the toolkit ships and tests with, so on a predictor configured
    // through SetKinematicsSource alone - no registered satellite nodes at all -
    // this indexed an empty NodeContainer and segfaulted. The kinematics path is
    // the one CHO-2 introduced precisely because the SGP4 model is not a
    // SatMobilityModel, so "no nodes registered" is a supported configuration,
    // not a caller error.
    if (satId >= m_satellites.GetN())
    {
        return nullptr;
    }
    return m_satellites.Get(satId);
}

Ptr<SatMobilityModel>
NtnOrbitPredictor::GetSatelliteMobility(uint32_t satId) const
{
    Ptr<Node> satNode = GetSatelliteNode(satId);
    if (!satNode)
    {
        return nullptr;
    }
    // Also an NS_ASSERT before, so in an optimized build a node without a
    // SatMobilityModel returned null and every caller dereferenced it.
    return satNode->GetObject<SatMobilityModel>();
}

double
NtnOrbitPredictor::ComputeGroundDistance(GeoCoordinate a, GeoCoordinate b) const
{
    // Haversine formula for great-circle distance
    double lat1 = a.GetLatitude() * M_PI / 180.0;
    double lat2 = b.GetLatitude() * M_PI / 180.0;
    double dlat = (b.GetLatitude() - a.GetLatitude()) * M_PI / 180.0;
    double dlon = (b.GetLongitude() - a.GetLongitude()) * M_PI / 180.0;

    double hav = std::sin(dlat / 2.0) * std::sin(dlat / 2.0) +
                 std::cos(lat1) * std::cos(lat2) * std::sin(dlon / 2.0) * std::sin(dlon / 2.0);
    double c = 2.0 * std::atan2(std::sqrt(hav), std::sqrt(1.0 - hav));

    return GeoCoordinate::equatorRadius * c; // distance in meters
}

double
NtnOrbitPredictor::Compute3dDistance(GeoCoordinate a, GeoCoordinate b) const
{
    Vector va = a.ToVector();
    Vector vb = b.ToVector();
    double dx = va.x - vb.x;
    double dy = va.y - vb.y;
    double dz = va.z - vb.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

} // namespace ns3
