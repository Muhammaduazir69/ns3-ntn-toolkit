/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-only
 * Author: Muhammad Uzair
 *
 * ML-Enhanced Beam Tracking for THz LEO-NTN — implementation
 */

#include "thz-ntn-beam-tracking.h"

#include <ns3/boolean.h>
#include <ns3/double.h>
#include <ns3/log.h>
#include <ns3/mobility-model.h>
#include <ns3/simulator.h>
#include <ns3/string.h>
#include <ns3/uinteger.h>

#include <algorithm>
#include <cmath>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("ThzNtnBeamTracking");
NS_OBJECT_ENSURE_REGISTERED(ThzNtnBeamTracking);

static constexpr double PI      = 3.14159265358979323846;
static constexpr double DEG2RAD = PI / 180.0;
static constexpr double RAD2DEG = 180.0 / PI;
static constexpr double EARTH_RADIUS_M = 6371000.0;

// ---- TypeId ----

TypeId
ThzNtnBeamTracking::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::ThzNtnBeamTracking")
            .SetParent<Object>()
            .SetGroupName("ThzNtn")
            .AddConstructor<ThzNtnBeamTracking>()
            .AddAttribute("TrackingMode",
                          "Beam tracking algorithm: EKF, POSITION_BASED, or ML_ASSISTED.",
                          StringValue("EKF"),
                          MakeStringAccessor(&ThzNtnBeamTracking::m_modeStr),
                          MakeStringChecker())
            .AddAttribute("BeamFailureThreshold_dB",
                          "SINR threshold below which a measurement is considered failed (dB).",
                          DoubleValue(-10.0),
                          MakeDoubleAccessor(&ThzNtnBeamTracking::m_beamFailureThreshold_dB),
                          MakeDoubleChecker<double>(-50.0, 30.0))
            .AddAttribute("ConsecutiveFailures",
                          "Number of consecutive low-SINR measurements before beam failure.",
                          UintegerValue(5),
                          MakeUintegerAccessor(
                              &ThzNtnBeamTracking::m_consecutiveFailuresThreshold),
                          MakeUintegerChecker<uint32_t>(1, 100))
            .AddAttribute("ProcessNoise",
                          "EKF process noise standard deviation for angular rate (deg/s).",
                          DoubleValue(0.01),
                          MakeDoubleAccessor(&ThzNtnBeamTracking::m_processNoise),
                          MakeDoubleChecker<double>(1e-6, 10.0))
            .AddAttribute("MeasurementNoise",
                          "EKF measurement noise standard deviation (deg).",
                          DoubleValue(0.1),
                          MakeDoubleAccessor(&ThzNtnBeamTracking::m_measurementNoise),
                          MakeDoubleChecker<double>(1e-6, 30.0))
            .AddAttribute("UpdateRate_Hz",
                          "Beam tracking measurement update rate (Hz).",
                          DoubleValue(100.0),
                          MakeDoubleAccessor(&ThzNtnBeamTracking::m_updateRate_Hz),
                          MakeDoubleChecker<double>(1.0, 1e6));
    return tid;
}

// ---- Constructor / destructor ----

ThzNtnBeamTracking::ThzNtnBeamTracking()
    : m_satMobility(nullptr),
      m_ueMobility(nullptr),
      m_trackingMode(ThzTrackingMode::EKF),
      m_trackingState(ThzBeamTrackingState::SEARCHING),
      m_modeStr("EKF"),
      m_beamFailureThreshold_dB(-10.0),
      m_consecutiveFailuresThreshold(5),
      m_processNoise(0.01),
      m_measurementNoise(0.1),
      m_updateRate_Hz(100.0),
      m_lastTimestamp_s(0.0),
      m_consecutiveLowSinr(0),
      m_initialized(false),
      m_measurementCount(0),
      m_hasMLCallback(false)
{
    NS_LOG_FUNCTION(this);
    m_state.fill(0.0);

    m_P.fill(0.0);
    m_P[0]  = 10.0;
    m_P[5]  = 10.0;
    m_P[10] = 1.0;
    m_P[15] = 1.0;
}

ThzNtnBeamTracking::~ThzNtnBeamTracking()
{
    NS_LOG_FUNCTION(this);
}

void
ThzNtnBeamTracking::ParseModeString()
{
    if (m_modeStr == "POSITION_BASED")
    {
        m_trackingMode = ThzTrackingMode::POSITION_BASED;
    }
    else if (m_modeStr == "ML_ASSISTED")
    {
        m_trackingMode = ThzTrackingMode::ML_ASSISTED;
    }
    else
    {
        m_trackingMode = ThzTrackingMode::EKF;
    }
}

// ---- MobilityModel integration ----

void
ThzNtnBeamTracking::SetSatelliteMobility(Ptr<MobilityModel> satMobility)
{
    NS_LOG_FUNCTION(this);
    m_satMobility = satMobility;
}

void
ThzNtnBeamTracking::SetUeMobility(Ptr<MobilityModel> ueMobility)
{
    NS_LOG_FUNCTION(this);
    m_ueMobility = ueMobility;
}

// ---- Compute angles from actual MobilityModel ----

void
ThzNtnBeamTracking::ComputeAnglesFromMobility(double& elevDeg, double& azDeg) const
{
    if (!m_satMobility || !m_ueMobility)
    {
        elevDeg = m_state[0];
        azDeg = m_state[1];
        return;
    }

    Vector satPos = m_satMobility->GetPosition();
    Vector uePos = m_ueMobility->GetPosition();

    // Direction vector from UE to satellite
    double dx = satPos.x - uePos.x;
    double dy = satPos.y - uePos.y;
    double dz = satPos.z - uePos.z;
    double dist = std::sqrt(dx * dx + dy * dy + dz * dz);

    if (dist < 1.0)
    {
        elevDeg = 90.0;
        azDeg = 0.0;
        return;
    }

    // UE's local "up" direction (radial from Earth centre)
    double ueR = std::sqrt(uePos.x * uePos.x + uePos.y * uePos.y + uePos.z * uePos.z);
    if (ueR < 1.0)
    {
        ueR = EARTH_RADIUS_M;
    }

    double upX = uePos.x / ueR;
    double upY = uePos.y / ueR;
    double upZ = uePos.z / ueR;

    // Unit direction to satellite
    double ux = dx / dist;
    double uy = dy / dist;
    double uz = dz / dist;

    // Elevation = angle above local horizontal
    double sinElev = ux * upX + uy * upY + uz * upZ;
    sinElev = std::max(-1.0, std::min(1.0, sinElev));
    elevDeg = std::asin(sinElev) * RAD2DEG;

    // Azimuth: project onto local horizontal plane
    // East direction
    double xyNorm = std::sqrt(uePos.x * uePos.x + uePos.y * uePos.y);
    double eastX = 0.0, eastY = 1.0, eastZ = 0.0;
    if (xyNorm > 1.0)
    {
        eastX = -uePos.y / xyNorm;
        eastY = uePos.x / xyNorm;
        eastZ = 0.0;
    }

    // North = up x east
    double northX = upY * eastZ - upZ * eastY;
    double northY = upZ * eastX - upX * eastZ;
    double northZ = upX * eastY - upY * eastX;

    double projEast = ux * eastX + uy * eastY + uz * eastZ;
    double projNorth = ux * northX + uy * northY + uz * northZ;
    azDeg = std::atan2(projEast, projNorth) * RAD2DEG;
    if (azDeg < 0.0)
    {
        azDeg += 360.0;
    }

    NS_LOG_DEBUG("AnglesFromMobility: elev=" << elevDeg << " az=" << azDeg);
}

// ---- SINR trace connection ----

void
ThzNtnBeamTracking::OnSinrMeasurement(double sinr_dB)
{
    NS_LOG_FUNCTION(this << sinr_dB);

    double timestamp_s = Simulator::Now().GetSeconds();

    // Compute current direction from MobilityModel
    double measTheta = m_state[0];
    double measPhi = m_state[1];

    if (m_satMobility && m_ueMobility)
    {
        ComputeAnglesFromMobility(measTheta, measPhi);
    }

    UpdateMeasurement(measTheta, measPhi, sinr_dB, timestamp_s);
}

// ---- Initialisation ----

void
ThzNtnBeamTracking::Initialize(double initTheta,
                               double initPhi,
                               double initRate_theta,
                               double initRate_phi)
{
    NS_LOG_FUNCTION(this << initTheta << initPhi << initRate_theta << initRate_phi);

    ParseModeString();

    m_state[0] = initTheta;
    m_state[1] = initPhi;
    m_state[2] = initRate_theta;
    m_state[3] = initRate_phi;

    m_P.fill(0.0);
    m_P[0]  = 1.0;
    m_P[5]  = 1.0;
    m_P[10] = 0.1;
    m_P[15] = 0.1;

    m_trackingState    = ThzBeamTrackingState::TRACKING;
    m_consecutiveLowSinr = 0;
    m_initialized      = true;
    m_lastTimestamp_s   = 0.0;
    m_measurementCount  = 0;
    m_metrics = ThzTrackingMetrics();

    NS_LOG_INFO("BeamTracking initialized: theta=" << initTheta << " phi=" << initPhi
                << " rate=(" << initRate_theta << "," << initRate_phi << ") deg/s");
}

// ---- EKF predict step ----

void
ThzNtnBeamTracking::EkfPredict(double dt)
{
    NS_LOG_FUNCTION(this << dt);

    // If satellite MobilityModel is available, update angular rates
    // from actual satellite velocity
    if (m_satMobility && m_ueMobility)
    {
        double elevNow = 0.0, azNow = 0.0;
        ComputeAnglesFromMobility(elevNow, azNow);

        // Estimate angular rates from position change
        if (m_measurementCount > 0 && dt > 0.0)
        {
            m_state[2] = (elevNow - m_state[0]) / dt;
            m_state[3] = (azNow - m_state[1]) / dt;
        }
    }

    // State prediction (constant-velocity model)
    m_state[0] += m_state[2] * dt;
    m_state[1] += m_state[3] * dt;

    // Covariance prediction: P = F * P * F^T + Q
    auto P = [this](int i, int j) -> double& { return m_P[i * 4 + j]; };

    std::array<double, 16> Pold = m_P;
    auto Po = [&Pold](int i, int j) -> double { return Pold[i * 4 + j]; };

    for (int i = 0; i < 4; ++i)
    {
        for (int j = 0; j < 4; ++j)
        {
            double val = Po(i, j);
            if (i < 2)
            {
                val += dt * Po(i + 2, j);
            }
            if (j < 2)
            {
                val += dt * Po(i, j + 2);
            }
            if (i < 2 && j < 2)
            {
                val += dt * dt * Po(i + 2, j + 2);
            }
            P(i, j) = val;
        }
    }

    double q_pos = m_processNoise * m_processNoise * dt * dt;
    double q_vel = m_processNoise * m_processNoise * dt * 10.0;

    P(0, 0) += q_pos;
    P(1, 1) += q_pos;
    P(2, 2) += q_vel;
    P(3, 3) += q_vel;

    NS_LOG_DEBUG("EKF predict: dt=" << dt << " state=[" << m_state[0] << ","
                 << m_state[1] << "," << m_state[2] << "," << m_state[3] << "]");
}

// ---- EKF update step ----

void
ThzNtnBeamTracking::EkfUpdate(double measTheta, double measPhi)
{
    NS_LOG_FUNCTION(this << measTheta << measPhi);

    auto P = [this](int i, int j) -> double& { return m_P[i * 4 + j]; };

    double r = m_measurementNoise * m_measurementNoise;

    double y0 = measTheta - m_state[0];
    double y1 = measPhi - m_state[1];

    double S00 = P(0, 0) + r;
    double S01 = P(0, 1);
    double S10 = P(1, 0);
    double S11 = P(1, 1) + r;

    double detS = S00 * S11 - S01 * S10;
    if (std::abs(detS) < 1e-30)
    {
        NS_LOG_WARN("EKF update: S matrix is singular, skipping update");
        return;
    }
    double invDetS = 1.0 / detS;
    double Si00 =  S11 * invDetS;
    double Si01 = -S01 * invDetS;
    double Si10 = -S10 * invDetS;
    double Si11 =  S00 * invDetS;

    double K[4][2];
    for (int i = 0; i < 4; ++i)
    {
        double ph0 = P(i, 0);
        double ph1 = P(i, 1);
        K[i][0] = ph0 * Si00 + ph1 * Si10;
        K[i][1] = ph0 * Si01 + ph1 * Si11;
    }

    for (int i = 0; i < 4; ++i)
    {
        m_state[i] += K[i][0] * y0 + K[i][1] * y1;
    }

    std::array<double, 16> Pold = m_P;
    auto Po = [&Pold](int i, int j) -> double { return Pold[i * 4 + j]; };

    for (int i = 0; i < 4; ++i)
    {
        for (int j = 0; j < 4; ++j)
        {
            double sum = 0.0;
            for (int k = 0; k < 4; ++k)
            {
                double ikh = (i == k ? 1.0 : 0.0);
                if (k == 0)
                {
                    ikh -= K[i][0];
                }
                if (k == 1)
                {
                    ikh -= K[i][1];
                }
                sum += ikh * Po(k, j);
            }
            P(i, j) = sum;
        }
    }

    double err = std::sqrt(y0 * y0 + y1 * y1);
    m_metrics.trackingError_deg = err;

    double alpha = 0.1;
    m_metrics.avgTrackingError_deg =
        alpha * err + (1.0 - alpha) * m_metrics.avgTrackingError_deg;

    NS_LOG_DEBUG("EKF update: meas=(" << measTheta << "," << measPhi
                 << ") state=(" << m_state[0] << "," << m_state[1]
                 << ") err=" << err << " deg");
}

// ---- Measurement interface ----

void
ThzNtnBeamTracking::UpdateMeasurement(double measuredTheta,
                                      double measuredPhi,
                                      double sinr_dB,
                                      double timestamp_s)
{
    NS_LOG_FUNCTION(this << measuredTheta << measuredPhi << sinr_dB << timestamp_s);

    if (!m_initialized)
    {
        Initialize(measuredTheta, measuredPhi, 0.0, 0.0);
        m_lastTimestamp_s = timestamp_s;
        return;
    }

    double dt = timestamp_s - m_lastTimestamp_s;
    if (dt < 0.0)
    {
        NS_LOG_WARN("Negative dt=" << dt << ", ignoring measurement");
        return;
    }
    m_lastTimestamp_s = timestamp_s;
    ++m_measurementCount;

    // Beam failure detection
    if (sinr_dB < m_beamFailureThreshold_dB)
    {
        ++m_consecutiveLowSinr;
        if (m_consecutiveLowSinr >= m_consecutiveFailuresThreshold &&
            m_trackingState != ThzBeamTrackingState::BEAM_FAILURE &&
            m_trackingState != ThzBeamTrackingState::RECOVERY)
        {
            m_trackingState = ThzBeamTrackingState::BEAM_FAILURE;
            ++m_metrics.beamFailureCount;
            NS_LOG_INFO("BEAM FAILURE detected after " << m_consecutiveLowSinr
                        << " consecutive low-SINR measurements");
            return;
        }
    }
    else
    {
        m_consecutiveLowSinr = 0;
        if (m_trackingState == ThzBeamTrackingState::RECOVERY)
        {
            m_trackingState = ThzBeamTrackingState::TRACKING;
            NS_LOG_INFO("Beam recovery succeeded, resuming TRACKING");
        }
        else if (m_trackingState == ThzBeamTrackingState::SEARCHING)
        {
            m_trackingState = ThzBeamTrackingState::TRACKING;
        }
    }

    if (m_trackingState == ThzBeamTrackingState::BEAM_FAILURE)
    {
        return;
    }

    ParseModeString();

    switch (m_trackingMode)
    {
    case ThzTrackingMode::EKF:
        if (dt > 0.0)
        {
            EkfPredict(dt);
        }
        EkfUpdate(measuredTheta, measuredPhi);
        break;

    case ThzTrackingMode::POSITION_BASED:
    {
        // Use actual MobilityModel positions for direct pointing
        if (m_satMobility && m_ueMobility)
        {
            double elev = 0.0, az = 0.0;
            ComputeAnglesFromMobility(elev, az);
            m_state[0] = elev;
            m_state[1] = az;
            m_metrics.trackingError_deg = std::sqrt(
                std::pow(elev - measuredTheta, 2) +
                std::pow(az - measuredPhi, 2));
        }
        else
        {
            m_state[0] = measuredTheta;
            m_state[1] = measuredPhi;
            m_metrics.trackingError_deg = 0.0;
        }
        break;
    }

    case ThzTrackingMode::ML_ASSISTED:
        if (m_hasMLCallback)
        {
            auto pred = m_mlCallback(measuredTheta, measuredPhi, timestamp_s);
            m_state[0] = 0.7 * pred.first + 0.3 * measuredTheta;
            m_state[1] = 0.7 * pred.second + 0.3 * measuredPhi;
            double err = std::sqrt(
                std::pow(m_state[0] - measuredTheta, 2) +
                std::pow(m_state[1] - measuredPhi, 2));
            m_metrics.trackingError_deg = err;
            m_metrics.predictionAccuracy_deg = err;
        }
        else
        {
            NS_LOG_WARN("ML_ASSISTED mode but no callback set, falling back to EKF");
            if (dt > 0.0)
            {
                EkfPredict(dt);
            }
            EkfUpdate(measuredTheta, measuredPhi);
        }
        break;
    }
}

// ---- Prediction ----

std::pair<double, double>
ThzNtnBeamTracking::PredictBeamDirection(Time futureTime) const
{
    return PredictBeamDirection(futureTime.GetSeconds());
}

std::pair<double, double>
ThzNtnBeamTracking::PredictBeamDirection(double futureTime_s) const
{
    NS_LOG_FUNCTION(this << futureTime_s);

    // If satellite MobilityModel is available and we are in POSITION_BASED mode,
    // compute directly from positions
    if (m_satMobility && m_ueMobility &&
        m_trackingMode == ThzTrackingMode::POSITION_BASED)
    {
        // For future prediction, we cannot move the MobilityModel forward,
        // so we extrapolate from current velocity
        [[maybe_unused]] Vector satPos = m_satMobility->GetPosition();
        [[maybe_unused]] Vector satVel = m_satMobility->GetVelocity();
        double dt = futureTime_s - m_lastTimestamp_s;

        // Simple linear extrapolation of satellite position
        // (The actual MobilityModel may use SGP4 which is more accurate,
        // but we cannot query it at arbitrary future times from here)
        double predTheta = m_state[0] + m_state[2] * std::max(dt, 0.0);
        double predPhi   = m_state[1] + m_state[3] * std::max(dt, 0.0);
        return {predTheta, predPhi};
    }

    // EKF constant-velocity extrapolation
    double dt = futureTime_s - m_lastTimestamp_s;
    if (dt < 0.0)
    {
        dt = 0.0;
    }

    double predTheta = m_state[0] + m_state[2] * dt;
    double predPhi   = m_state[1] + m_state[3] * dt;

    return {predTheta, predPhi};
}

std::pair<double, double>
ThzNtnBeamTracking::PredictFromMobility() const
{
    NS_LOG_FUNCTION(this);

    double elev = m_state[0];
    double az = m_state[1];

    if (m_satMobility && m_ueMobility)
    {
        const_cast<ThzNtnBeamTracking*>(this)->ComputeAnglesFromMobility(elev, az);
    }

    return {elev, az};
}

std::pair<double, double>
ThzNtnBeamTracking::PredictFromEphemeris(double satLat,
                                         double satLon,
                                         double satAlt,
                                         double ueLat,
                                         double ueLon) const
{
    NS_LOG_FUNCTION(this << satLat << satLon << satAlt << ueLat << ueLon);

    double Re = EARTH_RADIUS_M / 1000.0; // km
    double h  = satAlt;

    double uLatR  = ueLat * DEG2RAD;
    double sLatR  = satLat * DEG2RAD;
    double dLonR  = (satLon - ueLon) * DEG2RAD;

    double cosGamma = std::sin(uLatR) * std::sin(sLatR) +
                      std::cos(uLatR) * std::cos(sLatR) * std::cos(dLonR);
    cosGamma = std::max(-1.0, std::min(1.0, cosGamma));
    double sinGamma = std::sqrt(1.0 - cosGamma * cosGamma);

    double elevation_rad = std::atan2(cosGamma - Re / (Re + h), sinGamma);
    double elevation_deg = elevation_rad * RAD2DEG;

    double azimuth_rad = std::atan2(
        std::sin(dLonR) * std::cos(sLatR),
        std::cos(uLatR) * std::sin(sLatR) -
            std::sin(uLatR) * std::cos(sLatR) * std::cos(dLonR));
    double azimuth_deg = azimuth_rad * RAD2DEG;
    if (azimuth_deg < 0.0)
    {
        azimuth_deg += 360.0;
    }

    return {elevation_deg, azimuth_deg};
}

// ---- State queries ----

ThzBeamTrackingState
ThzNtnBeamTracking::GetTrackingState() const
{
    return m_trackingState;
}

bool
ThzNtnBeamTracking::DetectBeamFailure() const
{
    return m_consecutiveLowSinr >= m_consecutiveFailuresThreshold;
}

void
ThzNtnBeamTracking::TriggerBeamRecovery()
{
    NS_LOG_FUNCTION(this);

    m_trackingState = ThzBeamTrackingState::RECOVERY;
    m_consecutiveLowSinr = 0;

    m_P.fill(0.0);
    m_P[0]  = 100.0;
    m_P[5]  = 100.0;
    m_P[10] = 10.0;
    m_P[15] = 10.0;

    m_metrics.beamRecoveryTime_ms = 1000.0 / m_updateRate_Hz *
                                    static_cast<double>(m_consecutiveFailuresThreshold);

    NS_LOG_INFO("Beam recovery triggered: state reset with high uncertainty");
}

ThzTrackingMetrics
ThzNtnBeamTracking::GetTrackingMetrics() const
{
    return m_metrics;
}

double
ThzNtnBeamTracking::ComputeTrackingOverhead(double updateRate_Hz,
                                             double searchTime_us) const
{
    double overhead = updateRate_Hz * searchTime_us / 1e6;
    return std::min(overhead, 1.0);
}

// ---- ML callback ----

void
ThzNtnBeamTracking::SetBeamPredictionCallback(ThzBeamPredictionCallback cb)
{
    NS_LOG_FUNCTION(this);
    m_mlCallback    = cb;
    m_hasMLCallback = true;
    NS_LOG_INFO("ML beam prediction callback registered");
}

} // namespace ns3
