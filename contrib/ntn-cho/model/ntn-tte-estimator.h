/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Author: Muhammad Uzair
 *
 * NTN TTE (Time-to-Exit) Estimator
 *
 * THE CORE NOVEL CONTRIBUTION: Computes how long each candidate satellite beam's
 * D1 condition will remain valid for a UE, enabling TTE-aware conditional handover
 * that prevents late or unstable handover executions.
 *
 * Algorithm:
 *   For each candidate beam, propagate satellite position forward in time using SGP4,
 *   compute beam gain at UE position at each step, and find the time when beam quality
 *   drops below the D1 threshold. Uses binary search for precision.
 *
 * Reference: Novel contribution - no existing 3GPP specification
 */

#ifndef NTN_TTE_ESTIMATOR_H
#define NTN_TTE_ESTIMATOR_H

#include "ntn-orbit-predictor.h"

#include <ns3/geo-coordinate.h>
#include <ns3/nstime.h>
#include <ns3/object.h>
#include <ns3/traced-callback.h>
#include <ns3/vector.h>

#include <vector>

namespace ns3
{

/**
 * \ingroup ntn-cho
 * \brief Estimates Time-to-Exit (TTE) for satellite beam coverage
 *
 * Given a UE position, a candidate satellite beam, and a D1 distance threshold,
 * this class estimates how long the beam's coverage condition will remain valid.
 * It uses SGP4 orbit propagation + antenna gain patterns + binary search to find
 * the precise exit time with configurable accuracy.
 *
 * This is the core algorithmic novelty of the TTE-aware CHO framework.
 */
class NtnTteEstimator : public Object
{
  public:
    /**
     * \brief Result of TTE computation for a single candidate beam
     */
    struct TteResult
    {
        uint16_t cellId;         //!< Logical cell identifier
        uint32_t satId;          //!< Satellite identifier
        uint32_t beamId;         //!< Beam identifier
        Time tte;                //!< Estimated Time-to-Exit
        double currentGain_dB;   //!< Current antenna gain at UE position
        double peakGain_dB;      //!< Maximum gain during coverage period
        double exitGain_dB;      //!< Gain at the moment of exit
        double currentSinr_dB;   //!< Current SINR estimate
        bool isValid;            //!< Whether TTE computation succeeded
    };

    /**
     * \brief Information about a candidate beam for batch TTE computation
     */
    struct CandidateBeamInfo
    {
        uint16_t cellId;     //!< Logical cell identifier
        uint32_t satId;      //!< Satellite identifier
        uint32_t beamId;     //!< Beam identifier
        double sinr_dB;      //!< Latest SINR measurement
    };

    static TypeId GetTypeId();
    NtnTteEstimator();
    ~NtnTteEstimator() override;

    /**
     * \brief Set the orbit predictor used for satellite position/beam computation
     * \param predictor The NtnOrbitPredictor instance
     */
    void SetOrbitPredictor(Ptr<NtnOrbitPredictor> predictor);

    /**
     * \brief Compute TTE for a single candidate beam
     *
     * Algorithm:
     * 1. At current time, check if beam covers UE (gain > threshold). If not, TTE = 0.
     * 2. Coarse search: step forward in predictionStep increments until gain < threshold
     * 3. Binary search between last good step and first bad step for precise exit time
     * 4. Record peak gain encountered during the coverage period
     *
     * \param uePosition UE ground position (lat/lon/alt)
     * \param ueVelocity UE velocity vector (m/s) for mobile UEs
     * \param satId Satellite identifier
     * \param beamId Beam identifier
     * \param gainThreshold_dB Minimum gain for D1 condition validity (dB)
     * \return TteResult with estimated time-to-exit and quality metrics
     */
    TteResult ComputeTte(GeoCoordinate uePosition,
                         Vector ueVelocity,
                         uint32_t satId,
                         uint32_t beamId,
                         double gainThreshold_dB) const;

    /**
     * \brief Batch TTE computation for multiple candidate beams
     *
     * Computes TTE for each candidate and returns results sorted by TTE (descending).
     * This is the primary interface used by the CHO algorithm for candidate ranking.
     *
     * \param uePosition UE ground position
     * \param ueVelocity UE velocity vector
     * \param candidates Vector of candidate beam info
     * \param gainThreshold_dB Minimum gain for D1 condition validity
     * \return Vector of TteResults sorted by TTE descending (longest first)
     */
    std::vector<TteResult> ComputeBatchTte(
        GeoCoordinate uePosition,
        Vector ueVelocity,
        std::vector<CandidateBeamInfo> candidates,
        double gainThreshold_dB) const;

    /**
     * \brief Compute TTE using distance-based D1 condition
     *
     * Instead of antenna gain threshold, uses distance from beam center as the
     * D1 condition. This implements the 3GPP condEventD1 more directly.
     *
     * \param uePosition UE position
     * \param ueVelocity UE velocity
     * \param satId Satellite identifier
     * \param beamId Beam identifier
     * \param d1Threshold_m Maximum distance from beam center (meters)
     * \return TteResult
     */
    TteResult ComputeTteDistance(GeoCoordinate uePosition,
                                Vector ueVelocity,
                                uint32_t satId,
                                uint32_t beamId,
                                double d1Threshold_m) const;

    // Trace source for TTE computation events
    TracedCallback<uint32_t, uint32_t, Time, double> m_tteComputedTrace;
    //                satId,  beamId,  tte,   gain

  protected:
    void DoDispose() override;

  private:
    /**
     * \brief Binary search to find precise beam exit time
     *
     * Given that the beam covers the UE at tGood but not at tBad,
     * binary search for the exact exit time within tolerance.
     *
     * \param uePos UE position
     * \param satId Satellite ID
     * \param beamId Beam ID
     * \param threshold_dB Gain threshold
     * \param tGood Last time when beam was above threshold
     * \param tBad First time when beam dropped below threshold
     * \return Precise exit time offset from now
     */
    Time FindBeamExitTime(GeoCoordinate uePos,
                          uint32_t satId,
                          uint32_t beamId,
                          double threshold_dB,
                          Time tGood,
                          Time tBad) const;

    /**
     * \brief Binary search for distance-based exit time
     */
    Time FindDistanceExitTime(GeoCoordinate uePos,
                              uint32_t satId,
                              uint32_t beamId,
                              double distThreshold_m,
                              Time tGood,
                              Time tBad) const;

    /**
     * \brief Project UE position forward in time given velocity
     * \param uePos Current UE position
     * \param velocity UE velocity vector (m/s)
     * \param dt Time delta
     * \return Projected position (approximate, linearized)
     */
    GeoCoordinate ProjectUePosition(GeoCoordinate uePos,
                                    Vector velocity,
                                    Time dt) const;

    /**
     * \brief Compute TTE for a THz narrow beam based on geometric coverage
     *
     * \param satAlt_km Satellite altitude in km
     * \param satVelocity_km_s Satellite orbital velocity in km/s
     * \param beamwidth_deg THz beam 3dB beamwidth in degrees
     * \param pointingError_deg Current pointing error in degrees
     * \param elevationDeg Elevation angle from UE to satellite in degrees
     * \return TTE in seconds
     */
    double ComputeThzBeamTte(double satAlt_km,
                              double satVelocity_km_s,
                              double beamwidth_deg,
                              double pointingError_deg,
                              double elevationDeg) const;

    /**
     * \brief Compute effective THz beam coverage diameter on ground
     *
     * \param satAlt_km Satellite altitude in km
     * \param beamwidth_deg THz beam 3dB beamwidth in degrees
     * \param pointingError_deg Current pointing error in degrees
     * \return Effective coverage diameter in km
     */
    double ComputeThzEffectiveCoverage_km(double satAlt_km,
                                           double beamwidth_deg,
                                           double pointingError_deg) const;

    Ptr<NtnOrbitPredictor> m_orbitPredictor;   //!< Orbit prediction engine
    Time m_predictionStep;                      //!< Coarse search step (default 1s)
    Time m_maxPredictionWindow;                 //!< Max look-ahead window (default 120s)
    Time m_binarySearchTolerance;               //!< Binary search precision (default 100ms)
    uint32_t m_maxBinarySearchIter;             //!< Max iterations for binary search
};

} // namespace ns3

#endif // NTN_TTE_ESTIMATOR_H
