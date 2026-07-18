/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Author: Muhammad Uzair
 */

#include "ntn-tte-estimator.h"

#include <ns3/double.h>
#include <ns3/log.h>
#include <ns3/simulator.h>
#include <ns3/uinteger.h>

#include <algorithm>
#include <cmath>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("NtnTteEstimator");
NS_OBJECT_ENSURE_REGISTERED(NtnTteEstimator);

TypeId
NtnTteEstimator::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::NtnTteEstimator")
            .SetParent<Object>()
            .SetGroupName("NtnCho")
            .AddConstructor<NtnTteEstimator>()
            .AddAttribute("PredictionStep",
                          "Time step for coarse forward search",
                          TimeValue(Seconds(1.0)),
                          MakeTimeAccessor(&NtnTteEstimator::m_predictionStep),
                          MakeTimeChecker(MilliSeconds(100), Seconds(10)))
            .AddAttribute("MaxPredictionWindow",
                          "Maximum look-ahead time for TTE prediction",
                          TimeValue(Seconds(120.0)),
                          MakeTimeAccessor(&NtnTteEstimator::m_maxPredictionWindow),
                          MakeTimeChecker(Seconds(10), Seconds(600)))
            .AddAttribute("BinarySearchTolerance",
                          "Precision of binary search for exit time",
                          TimeValue(MilliSeconds(100)),
                          MakeTimeAccessor(&NtnTteEstimator::m_binarySearchTolerance),
                          MakeTimeChecker(MilliSeconds(10), Seconds(1)))
            .AddAttribute("MaxBinarySearchIterations",
                          "Maximum iterations for binary search convergence",
                          UintegerValue(20),
                          MakeUintegerAccessor(&NtnTteEstimator::m_maxBinarySearchIter),
                          MakeUintegerChecker<uint32_t>(5, 50))
            .AddTraceSource("TteComputed",
                            "Fired when a TTE is computed for a candidate beam",
                            MakeTraceSourceAccessor(&NtnTteEstimator::m_tteComputedTrace),
                            "ns3::NtnTteEstimator::TteComputedTracedCallback");
    return tid;
}

NtnTteEstimator::NtnTteEstimator()
    : m_predictionStep(Seconds(1.0)),
      m_maxPredictionWindow(Seconds(120.0)),
      m_binarySearchTolerance(MilliSeconds(100)),
      m_maxBinarySearchIter(20)
{
    NS_LOG_FUNCTION(this);
}

NtnTteEstimator::~NtnTteEstimator()
{
    NS_LOG_FUNCTION(this);
}

void
NtnTteEstimator::DoDispose()
{
    NS_LOG_FUNCTION(this);
    m_orbitPredictor = nullptr;
    Object::DoDispose();
}

void
NtnTteEstimator::SetOrbitPredictor(Ptr<NtnOrbitPredictor> predictor)
{
    NS_LOG_FUNCTION(this << predictor);
    m_orbitPredictor = predictor;
}

NtnTteEstimator::TteResult
NtnTteEstimator::ComputeTte(GeoCoordinate uePosition,
                             Vector ueVelocity,
                             uint32_t satId,
                             uint32_t beamId,
                             double gainThreshold_dB) const
{
    NS_LOG_FUNCTION(this << satId << beamId << gainThreshold_dB);
    NS_ASSERT_MSG(m_orbitPredictor, "OrbitPredictor not set");

    TteResult result;
    result.satId = satId;
    result.beamId = beamId;
    result.cellId = 0;
    result.isValid = false;

    // Step 0: Check current gain
    double currentGain = m_orbitPredictor->ComputeBeamGain(satId, beamId, uePosition);
    result.currentGain_dB = currentGain;
    result.peakGain_dB = currentGain;

    if (currentGain < gainThreshold_dB)
    {
        // UE is not currently covered by this beam
        NS_LOG_DEBUG("Beam " << beamId << " of sat " << satId
                              << " does not cover UE (gain=" << currentGain
                              << " < threshold=" << gainThreshold_dB << ")");
        result.tte = Seconds(0);
        result.exitGain_dB = currentGain;
        return result;
    }

    // Step 1: Coarse forward search - find first time gain drops below threshold
    Time tGood = Seconds(0);
    Time tBad = m_maxPredictionWindow;
    bool exitFound = false;

    for (Time t = m_predictionStep; t <= m_maxPredictionWindow; t += m_predictionStep)
    {
        // Project UE position (for mobile UEs)
        GeoCoordinate projectedUe = ProjectUePosition(uePosition, ueVelocity, t);

        // ---- GAP C1 FIX: advance the SATELLITE too ------------------------
        // This used to call ComputeBeamGain(satId, beamId, projectedUe), whose
        // comment claimed "the satellite position is automatically updated by
        // the SGP4 model". It is not: simulation time does not advance inside a
        // single event, so every iteration of this loop evaluated the gain with
        // the satellite frozen at Simulator::Now(). For a static UE the gain was
        // therefore IDENTICAL at every step, the exit was never found, and this
        // function returned TTE == m_maxPredictionWindow (120 s) for every beam
        // — making the TTE-aware admission (tte >= tteMinimum) always true and
        // condEventT1 (serving TTE <= window) unreachable. The headline
        // mechanism was a constant.
        //
        // GetBeamSnapshotAtTime propagates the satellite to t+dt and (since the
        // C1 fix in NtnOrbitPredictor) evaluates the body-fixed beam pattern at
        // that propagated geometry.
        const NtnOrbitPredictor::BeamSnapshot snap =
            m_orbitPredictor->GetBeamSnapshotAtTime(satId, beamId, projectedUe, t);
        double gain = snap.gainAtUe_dB;

        if (gain > result.peakGain_dB)
        {
            result.peakGain_dB = gain;
        }

        if (gain < gainThreshold_dB)
        {
            // Found exit: beam quality dropped below threshold
            tBad = t;
            tGood = t - m_predictionStep;
            exitFound = true;
            NS_LOG_DEBUG("Coarse exit found at t=" << t.GetSeconds()
                                                    << "s (gain=" << gain << " dB)");
            break;
        }
    }

    if (!exitFound)
    {
        // Beam covers UE for the entire prediction window
        result.tte = m_maxPredictionWindow;
        result.exitGain_dB = m_orbitPredictor->ComputeBeamGain(
            satId, beamId,
            ProjectUePosition(uePosition, ueVelocity, m_maxPredictionWindow));
        result.isValid = true;
        NS_LOG_DEBUG("No exit within prediction window. TTE >= "
                     << m_maxPredictionWindow.GetSeconds() << "s");
        m_tteComputedTrace(satId, beamId, result.tte, result.currentGain_dB);
        return result;
    }

    // Step 2: Binary search for precise exit time between tGood and tBad
    Time preciseExit = FindBeamExitTime(uePosition, satId, beamId, gainThreshold_dB, tGood, tBad);

    result.tte = preciseExit;
    result.exitGain_dB = gainThreshold_dB; // By definition at the exit point
    result.isValid = true;

    NS_LOG_INFO("TTE for sat " << satId << " beam " << beamId << " = " << result.tte.GetSeconds()
                                << "s (current gain=" << result.currentGain_dB
                                << " dB, peak=" << result.peakGain_dB << " dB)");

    m_tteComputedTrace(satId, beamId, result.tte, result.currentGain_dB);
    return result;
}

std::vector<NtnTteEstimator::TteResult>
NtnTteEstimator::ComputeBatchTte(GeoCoordinate uePosition,
                                  Vector ueVelocity,
                                  std::vector<CandidateBeamInfo> candidates,
                                  double gainThreshold_dB) const
{
    NS_LOG_FUNCTION(this << candidates.size() << gainThreshold_dB);

    std::vector<TteResult> results;
    results.reserve(candidates.size());

    for (const auto& cand : candidates)
    {
        TteResult res = ComputeTte(uePosition, ueVelocity, cand.satId, cand.beamId, gainThreshold_dB);
        res.cellId = cand.cellId;
        res.currentSinr_dB = cand.sinr_dB;
        results.push_back(res);
    }

    // Sort by TTE descending (longest coverage first)
    std::sort(results.begin(), results.end(), [](const TteResult& a, const TteResult& b) {
        return a.tte > b.tte;
    });

    return results;
}

NtnTteEstimator::TteResult
NtnTteEstimator::ComputeTteDistance(GeoCoordinate uePosition,
                                    Vector ueVelocity,
                                    uint32_t satId,
                                    uint32_t beamId,
                                    double d1Threshold_m) const
{
    NS_LOG_FUNCTION(this << satId << beamId << d1Threshold_m);
    NS_ASSERT_MSG(m_orbitPredictor, "OrbitPredictor not set");

    TteResult result;
    result.satId = satId;
    result.beamId = beamId;
    result.cellId = 0;
    result.isValid = false;
    result.currentGain_dB = m_orbitPredictor->ComputeBeamGain(satId, beamId, uePosition);
    result.peakGain_dB = result.currentGain_dB;

    // Get current beam center
    auto snap = m_orbitPredictor->GetBeamSnapshot(satId, beamId, uePosition);
    GeoCoordinate beamCenter = snap.beamCenter;

    // Check current distance
    Vector ueCart = uePosition.ToVector();
    Vector bcCart = beamCenter.ToVector();
    double dx = ueCart.x - bcCart.x;
    double dy = ueCart.y - bcCart.y;
    double dz = ueCart.z - bcCart.z;
    double currentDist = std::sqrt(dx * dx + dy * dy + dz * dz);

    if (currentDist > d1Threshold_m)
    {
        result.tte = Seconds(0);
        result.exitGain_dB = result.currentGain_dB;
        return result;
    }

    // Coarse search
    Time tGood = Seconds(0);
    Time tBad = m_maxPredictionWindow;
    bool exitFound = false;

    for (Time t = m_predictionStep; t <= m_maxPredictionWindow; t += m_predictionStep)
    {
        GeoCoordinate projUe = ProjectUePosition(uePosition, ueVelocity, t);
        auto futureSnap = m_orbitPredictor->GetBeamSnapshotAtTime(satId, beamId, projUe, t);

        Vector pCart = projUe.ToVector();
        Vector fbcCart = futureSnap.beamCenter.ToVector();
        double ddx = pCart.x - fbcCart.x;
        double ddy = pCart.y - fbcCart.y;
        double ddz = pCart.z - fbcCart.z;
        double dist = std::sqrt(ddx * ddx + ddy * ddy + ddz * ddz);

        if (dist > d1Threshold_m)
        {
            tBad = t;
            tGood = t - m_predictionStep;
            exitFound = true;
            break;
        }
    }

    if (!exitFound)
    {
        result.tte = m_maxPredictionWindow;
        result.exitGain_dB = result.currentGain_dB;
        result.isValid = true;
        m_tteComputedTrace(satId, beamId, result.tte, result.currentGain_dB);
        return result;
    }

    result.tte = FindDistanceExitTime(uePosition, satId, beamId, d1Threshold_m, tGood, tBad);
    result.exitGain_dB = m_orbitPredictor->ComputeBeamGain(
        satId, beamId, ProjectUePosition(uePosition, ueVelocity, result.tte));
    result.isValid = true;

    m_tteComputedTrace(satId, beamId, result.tte, result.currentGain_dB);
    return result;
}

Time
NtnTteEstimator::FindBeamExitTime(GeoCoordinate uePos,
                                   uint32_t satId,
                                   uint32_t beamId,
                                   double threshold_dB,
                                   Time tGood,
                                   Time tBad) const
{
    NS_LOG_FUNCTION(this << satId << beamId << tGood.GetSeconds() << tBad.GetSeconds());

    for (uint32_t iter = 0; iter < m_maxBinarySearchIter; iter++)
    {
        if ((tBad - tGood) <= m_binarySearchTolerance)
        {
            break;
        }

        Time tMid = Seconds((tGood.GetSeconds() + tBad.GetSeconds()) / 2.0);
        // GAP C1 (refinement half): propagate the SATELLITE to tMid via a beam
        // snapshot — exactly as the coarse search (line ~148) and the sibling
        // FindDistanceExitTime (line ~356) do. The old code called
        // ComputeBeamGain(satId, beamId, uePos) with no time, so the gain was
        // frozen at Simulator::Now() across the whole binary search: it never
        // crossed the threshold and the refinement collapsed to the upper
        // bracket (tBad), silently returning the coarse-grid granularity instead
        // of a refined exit time. Evaluating at tMid makes the search real.
        double gain = m_orbitPredictor->GetBeamSnapshotAtTime(satId, beamId, uePos, tMid).gainAtUe_dB;

        if (gain >= threshold_dB)
        {
            tGood = tMid;
        }
        else
        {
            tBad = tMid;
        }
    }

    return tGood;
}

Time
NtnTteEstimator::FindDistanceExitTime(GeoCoordinate uePos,
                                       uint32_t satId,
                                       uint32_t beamId,
                                       double distThreshold_m,
                                       Time tGood,
                                       Time tBad) const
{
    NS_LOG_FUNCTION(this << satId << beamId << tGood.GetSeconds() << tBad.GetSeconds());

    for (uint32_t iter = 0; iter < m_maxBinarySearchIter; iter++)
    {
        if ((tBad - tGood) <= m_binarySearchTolerance)
        {
            break;
        }

        Time tMid = Seconds((tGood.GetSeconds() + tBad.GetSeconds()) / 2.0);
        auto snap = m_orbitPredictor->GetBeamSnapshotAtTime(satId, beamId, uePos, tMid);

        Vector uCart = uePos.ToVector();
        Vector bCart = snap.beamCenter.ToVector();
        double dx = uCart.x - bCart.x;
        double dy = uCart.y - bCart.y;
        double dz = uCart.z - bCart.z;
        double dist = std::sqrt(dx * dx + dy * dy + dz * dz);

        if (dist <= distThreshold_m)
        {
            tGood = tMid;
        }
        else
        {
            tBad = tMid;
        }
    }

    return tGood;
}

GeoCoordinate
NtnTteEstimator::ProjectUePosition(GeoCoordinate uePos, Vector velocity, Time dt) const
{
    if (velocity.x == 0 && velocity.y == 0 && velocity.z == 0)
    {
        return uePos; // Static UE
    }

    double dtSec = dt.GetSeconds();

    // Approximate: convert velocity (m/s) to lat/lon displacement
    // At the equator, 1 degree latitude ~ 111,320 m
    // 1 degree longitude ~ 111,320 * cos(latitude) m
    double latRad = uePos.GetLatitude() * M_PI / 180.0;
    double metersPerDegLat = 111320.0;
    double metersPerDegLon = 111320.0 * std::cos(latRad);

    double dLat = (velocity.x * dtSec) / metersPerDegLat;
    double dLon = (velocity.y * dtSec) / metersPerDegLon;

    return GeoCoordinate(uePos.GetLatitude() + dLat,
                         uePos.GetLongitude() + dLon,
                         uePos.GetAltitude());
}

double
NtnTteEstimator::ComputeThzBeamTte(double satAlt_km,
                                    double satVelocity_km_s,
                                    double beamwidth_deg,
                                    double pointingError_deg,
                                    double elevationDeg) const
{
    NS_LOG_FUNCTION(this << satAlt_km << satVelocity_km_s << beamwidth_deg
                         << pointingError_deg << elevationDeg);

    // Effective coverage diameter accounting for pointing error
    double effectiveDiameter_km = ComputeThzEffectiveCoverage_km(
        satAlt_km, beamwidth_deg, pointingError_deg);

    if (effectiveDiameter_km <= 0.0)
    {
        NS_LOG_DEBUG("THz effective coverage is zero, TTE=0");
        return 0.0;
    }

    // Ground track velocity component
    // elevation_from_nadir = 90 - elevationDeg
    double nadirAngleRad = (90.0 - elevationDeg) * M_PI / 180.0;
    double groundTrackVelocity_km_s = satVelocity_km_s * std::cos(nadirAngleRad);

    if (groundTrackVelocity_km_s <= 0.0)
    {
        NS_LOG_DEBUG("Ground track velocity is zero, TTE=maxWindow");
        return m_maxPredictionWindow.GetSeconds();
    }

    double tte_s = effectiveDiameter_km / groundTrackVelocity_km_s;

    NS_LOG_INFO("THz beam TTE: alt=" << satAlt_km << " km, beamwidth=" << beamwidth_deg
                << " deg, pointErr=" << pointingError_deg << " deg, elev=" << elevationDeg
                << " deg -> coverage=" << effectiveDiameter_km << " km, v_ground="
                << groundTrackVelocity_km_s << " km/s, TTE=" << tte_s << " s");
    return tte_s;
}

double
NtnTteEstimator::ComputeThzEffectiveCoverage_km(double satAlt_km,
                                                  double beamwidth_deg,
                                                  double pointingError_deg) const
{
    NS_LOG_FUNCTION(this << satAlt_km << beamwidth_deg << pointingError_deg);

    static constexpr double DEG_TO_RAD = M_PI / 180.0;

    // Coverage radius = altitude * tan(beamwidth/2)
    double halfBeamRad = (beamwidth_deg / 2.0) * DEG_TO_RAD;
    double radius_km = satAlt_km * std::tan(halfBeamRad);

    // Reduce by pointing error factor
    double errorFactor = 1.0 - pointingError_deg / beamwidth_deg;
    if (errorFactor < 0.0)
    {
        errorFactor = 0.0;
    }

    double effectiveRadius_km = radius_km * errorFactor;
    double effectiveDiameter_km = 2.0 * effectiveRadius_km;

    NS_LOG_DEBUG("THz effective coverage: radius=" << radius_km << " km"
                 << ", errorFactor=" << errorFactor
                 << ", effective diameter=" << effectiveDiameter_km << " km");
    return effectiveDiameter_km;
}

} // namespace ns3
