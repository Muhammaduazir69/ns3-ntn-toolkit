/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Author: Muhammad Uzair
 *
 * NTN Orbit Predictor - Ephemeris-based beam coverage prediction
 * Wraps SGP4 satellite mobility model and antenna gain patterns to predict
 * beam center positions and coverage boundaries over time.
 */

#ifndef NTN_ORBIT_PREDICTOR_H
#define NTN_ORBIT_PREDICTOR_H

#include <ns3/geo-coordinate.h>
#include <ns3/node-container.h>
#include <ns3/nstime.h>
#include <ns3/object.h>
#include <ns3/satellite-antenna-gain-pattern-container.h>
#include <ns3/satellite-mobility-model.h>
#include <ns3/vector.h>

#include <map>
#include <vector>

namespace ns3
{

/**
 * \ingroup ntn-cho
 * \brief Predicts satellite beam positions and coverage over time using SGP4 ephemeris
 *
 * This class bridges the satellite module's SGP4 orbit propagation and antenna gain
 * pattern models to provide beam coverage timeline predictions. It answers questions
 * such as "how long will beam B of satellite S cover position P?" which is essential
 * for the TTE (Time-to-Exit) estimation.
 *
 * Reference: 3GPP TR 38.821 Section 6.1 (NTN mobility procedures)
 */
class NtnOrbitPredictor : public Object
{
  public:
    /**
     * \brief Information about a satellite beam at a point in time
     */
    struct BeamSnapshot
    {
        uint32_t satId;              //!< Satellite identifier
        uint32_t beamId;             //!< Beam identifier within satellite
        GeoCoordinate beamCenter;    //!< Beam center ground position
        GeoCoordinate satellitePosition; //!< Satellite ECEF/geodetic position
        double gainAtUe_dB;          //!< Antenna gain at UE position (dB)
        double elevationAngle_deg;   //!< Satellite elevation angle from UE (deg)
        double slantRange_km;        //!< Distance from UE to satellite (km)
        Time propagationDelay;       //!< One-way propagation delay
    };

    /**
     * \brief A visible satellite and its best beam for a UE position
     */
    struct VisibleSatellite
    {
        uint32_t satId;
        uint32_t bestBeamId;
        double bestGain_dB;
        double elevationAngle_deg;
        double slantRange_km;
    };

    static TypeId GetTypeId();
    NtnOrbitPredictor();
    ~NtnOrbitPredictor() override;

    /**
     * \brief Initialize with satellite constellation nodes and antenna patterns
     * \param satellites NodeContainer holding all satellite nodes (with SatSGP4MobilityModel)
     * \param agpContainer Antenna gain pattern container from satellite module
     */
    void Initialize(NodeContainer satellites,
                    Ptr<SatAntennaGainPatternContainer> agpContainer);

    /**
     * \brief Get beam snapshot for a specific satellite beam at current time
     * \param satId Satellite identifier
     * \param beamId Beam identifier
     * \param uePosition UE ground position
     * \return BeamSnapshot with current metrics
     */
    BeamSnapshot GetBeamSnapshot(uint32_t satId,
                                 uint32_t beamId,
                                 GeoCoordinate uePosition) const;

    /**
     * \brief Get beam snapshot at a future time offset from now
     * \param satId Satellite identifier
     * \param beamId Beam identifier
     * \param uePosition UE ground position
     * \param timeOffset Time offset from current simulation time
     * \return BeamSnapshot at the predicted future time
     */
    BeamSnapshot GetBeamSnapshotAtTime(uint32_t satId,
                                       uint32_t beamId,
                                       GeoCoordinate uePosition,
                                       Time timeOffset) const;

    /**
     * \brief Find all visible satellites from a ground position
     * \param uePosition UE ground position
     * \param minElevation_deg Minimum elevation angle (default 10 deg)
     * \return Vector of visible satellites sorted by gain (descending)
     */
    std::vector<VisibleSatellite> GetVisibleSatellites(
        GeoCoordinate uePosition,
        double minElevation_deg = 10.0) const;

    /**
     * \brief Predict the beam center ground track over a time window
     * \param satId Satellite identifier
     * \param beamId Beam identifier
     * \param startOffset Start time offset from now
     * \param endOffset End time offset from now
     * \param step Time step for sampling
     * \return Vector of (time offset, beam center position) pairs
     */
    std::vector<std::pair<Time, GeoCoordinate>> PredictBeamTrack(
        uint32_t satId,
        uint32_t beamId,
        Time startOffset,
        Time endOffset,
        Time step) const;

    /**
     * \brief Compute the antenna gain of a specific beam at a position
     * \param satId Satellite identifier
     * \param beamId Beam identifier
     * \param uePosition Ground position to evaluate
     * \return Gain in dB (negative if outside coverage)
     */
    /**
     * \brief C1: beam gain at \p uePosition with the satellite placed at
     *        \p satPosition, instead of wherever it is at Simulator::Now().
     *
     * Needed by any forward-looking search (e.g. the TTE estimator): sim time
     * does not advance inside one event, so ComputeBeamGain() below always
     * reports the gain for the CURRENT satellite position no matter what
     * future UE position it is handed.
     */
    double ComputeBeamGainAt(uint32_t satId,
                             uint32_t beamId,
                             GeoCoordinate uePosition,
                             GeoCoordinate satPosition) const;

    double ComputeBeamGain(uint32_t satId,
                           uint32_t beamId,
                           GeoCoordinate uePosition) const;

    /**
     * \brief Compute elevation angle from UE to satellite
     * \param uePosition UE ground position
     * \param satId Satellite identifier
     * \return Elevation angle in degrees
     */
    double ComputeElevationAngle(GeoCoordinate uePosition, uint32_t satId) const;

    /**
     * \brief Compute one-way propagation delay from UE to satellite
     * \param uePosition UE ground position
     * \param satId Satellite identifier
     * \return Propagation delay
     */
    Time ComputePropagationDelay(GeoCoordinate uePosition, uint32_t satId) const;

    /**
     * \brief Get the number of beams per satellite
     */
    uint32_t GetNumBeamsPerSat() const;

    /**
     * \brief Get the number of satellites
     */
    uint32_t GetNumSatellites() const;

    /**
     * \brief Get satellite node
     */
    Ptr<Node> GetSatelliteNode(uint32_t satId) const;

    /**
     * \brief Get satellite mobility model
     */
    Ptr<SatMobilityModel> GetSatelliteMobility(uint32_t satId) const;

  protected:
    void DoDispose() override;

  private:
    /**
     * \brief Compute great-circle distance between two ground positions
     * \param a First position
     * \param b Second position
     * \return Distance in meters
     */
    double ComputeGroundDistance(GeoCoordinate a, GeoCoordinate b) const;

    /**
     * \brief Compute 3D Euclidean distance
     */
    double Compute3dDistance(GeoCoordinate a, GeoCoordinate b) const;

    NodeContainer m_satellites;                            //!< Satellite nodes
    Ptr<SatAntennaGainPatternContainer> m_agpContainer;    //!< Antenna gain patterns
    double m_minGainThreshold_dB;                          //!< Minimum gain for valid coverage
    bool m_initialized;                                    //!< Whether Initialize() was called

    static constexpr double SPEED_OF_LIGHT = 299792458.0;  //!< Speed of light (m/s)
};

} // namespace ns3

#endif // NTN_ORBIT_PREDICTOR_H
