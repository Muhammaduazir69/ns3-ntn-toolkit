/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-only
 * Author: Muhammad Uzair
 *
 * ML-Enhanced Beam Tracking for THz LEO-NTN Links
 *
 * Integrated with actual satellite MobilityModel (SatSgp4MobilityModel)
 * for EKF state prediction and with PHY SINR TracedCallback for
 * measurement-driven updates.
 *
 * Three tracking modes are supported:
 *   EKF            — Extended Kalman Filter on angular state, updated from
 *                    actual SatSgp4MobilityModel position/velocity.
 *   POSITION_BASED — Direct beam pointing from satellite MobilityModel.
 *   ML_ASSISTED    — External ML/RL-based prediction via user callback.
 *
 * References:
 *   [1] M. Giordani et al., "Non-terrestrial networks in the 6G era:
 *       Challenges and opportunities," IEEE Access, 2021.
 *   [2] V. Va et al., "Beam tracking for mobile millimeter wave
 *       communication systems," IEEE GlobalSIP, 2016.
 *   [3] 3GPP TR 38.821, "Solutions for NR to support NTN," v16.1.0, 2021.
 */

#ifndef THZ_NTN_BEAM_TRACKING_H
#define THZ_NTN_BEAM_TRACKING_H

#include <ns3/callback.h>
#include <ns3/mobility-model.h>
#include <ns3/nstime.h>
#include <ns3/object.h>
#include <ns3/ptr.h>

#include <array>
#include <cstdint>
#include <string>
#include <utility>

namespace ns3
{

/**
 * \ingroup thz-ntn
 * \brief Beam tracking algorithm selection.
 */
enum class ThzTrackingMode : uint8_t
{
    EKF,            ///< Extended Kalman Filter
    POSITION_BASED, ///< MobilityModel-driven pointing
    ML_ASSISTED     ///< External ML/RL prediction callback
};

/**
 * \ingroup thz-ntn
 * \brief Beam tracking finite-state machine states.
 */
enum class ThzBeamTrackingState : uint8_t
{
    TRACKING,     ///< Normal operation — beam is aligned
    SEARCHING,    ///< Initial or re-acquisition search
    BEAM_FAILURE, ///< SINR below threshold for too long
    RECOVERY      ///< Wider-beam fallback + re-search
};

/**
 * \ingroup thz-ntn
 * \brief Performance metrics for beam tracking evaluation.
 */
struct ThzTrackingMetrics
{
    double   trackingError_deg{0.0};     ///< instantaneous pointing error
    double   avgTrackingError_deg{0.0};  ///< exponentially weighted average
    uint32_t beamFailureCount{0};        ///< total beam failures observed
    double   beamRecoveryTime_ms{0.0};   ///< last recovery duration
    double   predictionHorizon_ms{0.0};  ///< how far ahead the predictor looks
    double   predictionAccuracy_deg{0.0};///< RMS prediction error
};

/**
 * \ingroup thz-ntn
 * \brief Callback type for ML-assisted beam prediction.
 */
typedef Callback<std::pair<double, double>, double, double, double>
    ThzBeamPredictionCallback;

/**
 * \ingroup thz-ntn
 * \brief Predictive beam tracking engine for THz LEO-NTN links.
 *
 * EKF state is updated from actual satellite MobilityModel
 * position/velocity. Measurements are driven by connection to PHY
 * SINR TracedCallback. Position-based mode uses actual MobilityModel
 * for direct beam pointing computation.
 */
class ThzNtnBeamTracking : public Object
{
  public:
    static TypeId GetTypeId();

    ThzNtnBeamTracking();
    ~ThzNtnBeamTracking() override;

    // ---- MobilityModel integration ----

    /**
     * \brief Set the satellite MobilityModel for EKF state updates.
     *
     * The EKF predict step uses the satellite's actual position and
     * velocity to update the angular state prediction.
     * \param satMobility satellite MobilityModel (e.g. SatSgp4MobilityModel)
     */
    void SetSatelliteMobility(Ptr<MobilityModel> satMobility);

    /**
     * \brief Set the UE MobilityModel for reference position.
     * \param ueMobility UE MobilityModel
     */
    void SetUeMobility(Ptr<MobilityModel> ueMobility);

    // ---- SINR trace connection ----

    /**
     * \brief Callback for PHY SINR TracedCallback connection.
     *
     * Signature: (sinr_dB)
     * When connected, each received SINR sample triggers a tracking update.
     * \param sinr_dB measured SINR in dB
     */
    void OnSinrMeasurement(double sinr_dB);

    // ---- Initialisation ----

    /**
     * \brief Initialise the EKF state.
     * \param initTheta     initial elevation angle (deg)
     * \param initPhi       initial azimuth angle (deg)
     * \param initRate_theta elevation angular rate (deg/s)
     * \param initRate_phi   azimuth angular rate (deg/s)
     */
    void Initialize(double initTheta,
                    double initPhi,
                    double initRate_theta,
                    double initRate_phi);

    // ---- Measurement update ----

    /**
     * \brief Feed a new beam-domain measurement to the tracker.
     * \param measuredTheta measured elevation angle (deg)
     * \param measuredPhi   measured azimuth angle (deg)
     * \param sinr_dB       measured SINR in dB
     * \param timestamp_s   simulation time of measurement (s)
     */
    void UpdateMeasurement(double measuredTheta,
                           double measuredPhi,
                           double sinr_dB,
                           double timestamp_s);

    // ---- Prediction from actual MobilityModel ----

    /**
     * \brief Predict beam direction at a future time.
     *
     * Uses actual satellite MobilityModel when available for
     * position-based prediction, otherwise EKF extrapolation.
     * \param futureTime future simulation time
     * \return (theta, phi) prediction in degrees
     */
    std::pair<double, double> PredictBeamDirection(Time futureTime) const;

    /**
     * \brief Predict beam direction at a future time (seconds).
     */
    std::pair<double, double> PredictBeamDirection(double futureTime_s) const;

    /**
     * \brief Compute beam pointing from satellite and UE MobilityModels.
     * \return (elevation, azimuth) angles in degrees
     */
    std::pair<double, double> PredictFromMobility() const;

    /**
     * \brief Compute beam pointing from satellite ephemeris data.
     */
    std::pair<double, double> PredictFromEphemeris(double satLat,
                                                   double satLon,
                                                   double satAlt,
                                                   double ueLat,
                                                   double ueLon) const;

    // ---- State queries ----

    ThzBeamTrackingState GetTrackingState() const;
    bool DetectBeamFailure() const;
    void TriggerBeamRecovery();
    ThzTrackingMetrics GetTrackingMetrics() const;
    double ComputeTrackingOverhead(double updateRate_Hz,
                                   double searchTime_us) const;

    // ---- ML callback ----

    void SetBeamPredictionCallback(ThzBeamPredictionCallback cb);

  private:
    void EkfPredict(double dt);
    void EkfUpdate(double measTheta, double measPhi);
    void ParseModeString();

    /**
     * \brief Compute elevation and azimuth from satellite and UE positions.
     */
    void ComputeAnglesFromMobility(double& elevDeg, double& azDeg) const;

    // State vector: x = [theta, phi, dTheta/dt, dPhi/dt]
    std::array<double, 4> m_state;
    // 4x4 covariance matrix stored row-major
    std::array<double, 16> m_P;

    // MobilityModel references
    Ptr<MobilityModel> m_satMobility;  ///< Satellite MobilityModel
    Ptr<MobilityModel> m_ueMobility;   ///< UE MobilityModel

    // Configuration
    ThzTrackingMode      m_trackingMode;
    ThzBeamTrackingState m_trackingState;

    std::string m_modeStr;
    double      m_beamFailureThreshold_dB;
    uint32_t    m_consecutiveFailuresThreshold;
    double      m_processNoise;
    double      m_measurementNoise;
    double      m_updateRate_Hz;

    // Runtime state
    double   m_lastTimestamp_s;
    uint32_t m_consecutiveLowSinr;
    bool     m_initialized;

    ThzTrackingMetrics m_metrics;
    uint32_t           m_measurementCount;

    ThzBeamPredictionCallback m_mlCallback;
    bool                      m_hasMLCallback;
};

} // namespace ns3

#endif // THZ_NTN_BEAM_TRACKING_H
