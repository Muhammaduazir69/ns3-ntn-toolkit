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
    /// CHO-1: two-body forward propagation of an ECEF state over \p dtS
    /// seconds, accounting for the Earth's rotation.
    ///
    /// Replaces the first-order r + v*dt extrapolation this class used, whose
    /// error reaches ~51 km at a 120 s horizon on a LEO shell, larger than a
    /// service-beam footprint. Lifts the ECEF velocity into an inertial frame,
    /// rotates the position about the orbit normal, and rotates back through
    /// the Earth's turn. Exact for a circular orbit; the residual against real
    /// SGP4 is the eccentricity and J2 term. Public and static so it can be
    /// validated directly against an analytic orbit.
    static Vector PropagateTwoBodyEcef(const Vector& rEcef, const Vector& vEcef, double dtS);

    /// CHO-2: supply the real kinematics for a satellite.
    ///
    /// Initialize() takes SNS3 SatMobilityModel objects because the antenna
    /// gain-pattern container needs them, but the toolkit's own SGP4 model
    /// (ntncon::Sgp4MobilityModel) derives from plain ns3::MobilityModel and
    /// cannot be passed there. Scenarios worked around that by handing the
    /// predictor stationary GEO stand-ins, which report zero velocity: every
    /// forward propagation then returned the present position, so the
    /// time-to-exit was a constant for every candidate at every tick and the
    /// "prediction" predicted nothing.
    ///
    /// Register the real moving model here and the predictor uses it for
    /// position and velocity, keeping the SatMobilityModel only for beam
    /// geometry. Velocity is what makes a prediction possible, so a source with
    /// none is refused rather than silently accepted.
    void SetKinematicsSource(uint32_t satId, Ptr<MobilityModel> mob);

    /// CHO-2: use an analytic TR 38.811 Section 6.4.1 aperture beam instead of
    /// the SNS3 antenna-gain-pattern grid.
    ///
    /// The shipped pattern sets are authored for a GEO sub-point. Evaluating
    /// them at a LEO satellite position puts the terminal outside the sampled
    /// grid, so the pattern returns NaN, the gain floors at -100 dB and every
    /// predicted time-to-exit collapses to zero. That is not a beam model of a
    /// LEO cell, it is a lookup miss. This computes the gain from the actual
    /// off-nadir angle instead, which is coherent at any altitude.
    ///
    /// \param peakGainDbi boresight gain
    /// \param beamwidth3dbDeg full 3 dB beamwidth
    void SetGeometricBeam(double peakGainDbi, double beamwidth3dbDeg);

    /// CHO-2: model the service beam as STEERED at the terminal rather than
    /// fixed at nadir.
    ///
    /// A LEO service beam tracks its terminal, which is exactly why the
    /// toolkit's own array-gain calibration measures an off-boresight angle of
    /// zero for the steered case. Under a nadir-fixed beam the terminal sits
    /// outside a few-degree footprint almost immediately and the time-to-exit
    /// is zero everywhere, which describes the model rather than the network.
    /// Steered, the gain falls through phased-array SCAN LOSS as the satellite
    /// works its way toward the horizon, so beam exit becomes the
    /// elevation-driven event it physically is.
    ///
    /// \param scanLossExponent cos^n scan-loss law; 1.2 is the usual planar-array value.
    void SetSteeredBeam(bool steered, double scanLossExponent = 1.2);

    /// CHO-2b: the beam gain, in dB, that corresponds to a terminal at
    /// \p minElevDeg elevation seeing a satellite at \p satAltM.
    ///
    /// A time-to-exit needs a threshold on the quantity it searches, which is
    /// antenna gain. Scenarios were passing their SINR quality threshold
    /// instead, a different quantity on a different scale: against a 26 to
    /// 29 dB steered-beam gain a -3 dB "threshold" can never be crossed, so
    /// every time-to-exit saturated at the prediction horizon. Deriving the
    /// threshold from the minimum usable elevation keeps it on the right scale
    /// and ties it to a documented figure (TR 38.821 uses a 10 degree cell
    /// edge) rather than to a number tuned until the output looked reasonable.
    double GainThresholdForMinElevationDb(double minElevDeg, double satAltM) const;

    /// True when the analytic beam is in use rather than the pattern grid.
    bool UsingGeometricBeam() const { return m_geometricBeam; }

    /// Configured boresight gain, in dBi. The analytic beam reports gain
    /// RELATIVE to boresight, matching the SNS3 pattern convention, so this is
    /// the figure to add when an absolute value is wanted.
    double GetBeamPeakGainDbi() const { return m_beamPeakGainDbi; }

    /// True when \p satId has a kinematics source reporting non-zero velocity,
    /// i.e. forward propagation can actually move it.
    bool HasUsableKinematics(uint32_t satId) const;

    /// Number of registered satellites that cannot be propagated because they
    /// report zero velocity. Non-zero means any TTE derived from this predictor
    /// is a constant.
    uint32_t CountFrozenSatellites() const;

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
    /// CHO-2: analytic TR 38.811 6.4.1 beam gain, used when SetGeometricBeam() was called.
    double GeometricBeamGainDb(const GeoCoordinate& uePosition, const GeoCoordinate& satPosition) const;
    /// CHO-2 follow-up: resolve a satellite position from the kinematics source
    /// if one is registered, else from a SatMobilityModel. False when neither
    /// knows the satellite, so callers decline instead of dereferencing null.
    bool ResolveSatPosition(uint32_t satId, GeoCoordinate& out) const;
    /// CHO-2: real kinematics per satellite, when supplied.
    std::map<uint32_t, Ptr<MobilityModel>> m_kinematics;
    bool m_geometricBeam{false};      ///< CHO-2: analytic beam instead of the pattern grid
    double m_beamPeakGainDbi{30.0};   ///< CHO-2: boresight gain when analytic
    double m_beam3dbDeg{4.4127};      ///< CHO-2: full 3 dB beamwidth when analytic
    bool m_steeredBeam{false};        ///< CHO-2: boresight tracks the terminal
    double m_scanLossExp{1.2};        ///< CHO-2: cos^n scan-loss exponent

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
