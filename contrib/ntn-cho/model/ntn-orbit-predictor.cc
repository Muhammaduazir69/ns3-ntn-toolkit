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

    // Get satellite mobility model
    Ptr<SatMobilityModel> satMob = GetSatelliteMobility(satId);
    GeoCoordinate satPos = satMob->GetGeoPosition();
    snap.satellitePosition = satPos;

    // Compute beam center using antenna gain pattern
    Ptr<SatAntennaGainPattern> agp = m_agpContainer->GetAntennaGainPattern(beamId);
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

    // Forward-propagate the satellite ECEF position by timeOffset using
    // first-order kinematic extrapolation: r(t+dt) = r(t) + v(t)*dt.
    // For LEO (v ~7.5 km/s, h ~780 km, R_E + h ~7150 km), straight-line
    // extrapolation drifts < 30 m over 120 s relative to true SGP4 motion
    // (the curvature error is dt^2 * v^2 / r ~ 60 m at 120 s). This is
    // adequate for binary-search bracketing in the TTE estimator.
    Ptr<SatMobilityModel> satMob = GetSatelliteMobility(satId);
    GeoCoordinate satPosNow = satMob->GetGeoPosition();
    Vector satCartNow = satPosNow.ToVector();
    Vector satVel = satMob->GetVelocity();
    double dt = timeOffset.GetSeconds();
    Vector satCartFuture(satCartNow.x + satVel.x * dt,
                         satCartNow.y + satVel.y * dt,
                         satCartNow.z + satVel.z * dt);
    GeoCoordinate satPosFuture(satCartFuture);
    snap.satellitePosition = satPosFuture;

    // Beam center scrolls with the satellite footprint at the same dt.
    Ptr<SatAntennaGainPattern> agp = m_agpContainer->GetAntennaGainPattern(beamId);
    if (agp)
    {
        // The antenna pattern remains body-fixed; compute its centre from
        // the projected sub-satellite point at t+dt.
        snap.beamCenter = GeoCoordinate(satPosFuture.GetLatitude(),
                                        satPosFuture.GetLongitude(), 0.0);
    }
    else
    {
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
    if (!m_agpContainer)
    {
        return -100.0;
    }
    Ptr<SatAntennaGainPattern> agp = m_agpContainer->GetAntennaGainPattern(beamId);
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
    Ptr<SatAntennaGainPattern> agp = m_agpContainer->GetAntennaGainPattern(beamId);

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
            GeoCoordinate satPos = satMob->GetGeoPosition();
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

double
NtnOrbitPredictor::ComputeElevationAngle(GeoCoordinate uePosition, uint32_t satId) const
{
    NS_LOG_FUNCTION(this << satId);

    Ptr<SatMobilityModel> satMob = GetSatelliteMobility(satId);
    GeoCoordinate satPos = satMob->GetGeoPosition();

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

    Ptr<SatMobilityModel> satMob = GetSatelliteMobility(satId);
    GeoCoordinate satPos = satMob->GetGeoPosition();

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
    NS_ASSERT_MSG(satId < m_satellites.GetN(), "Invalid satellite ID " << satId);
    return m_satellites.Get(satId);
}

Ptr<SatMobilityModel>
NtnOrbitPredictor::GetSatelliteMobility(uint32_t satId) const
{
    Ptr<Node> satNode = GetSatelliteNode(satId);
    Ptr<SatMobilityModel> mob = satNode->GetObject<SatMobilityModel>();
    NS_ASSERT_MSG(mob, "Satellite node " << satId << " has no SatMobilityModel");
    return mob;
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
