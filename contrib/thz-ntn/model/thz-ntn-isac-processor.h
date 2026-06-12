/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-only
 * Author: Muhammad Uzair
 *
 * ISAC Signal Processing for THz Non-Terrestrial Networks
 *
 * Radar signal processing for the THz-NTN ISAC system including
 * range-Doppler map generation, CFAR target detection, joint
 * beamforming design, and multi-target tracking.  Integrates with
 * ThzNtnPhy for actual signal parameters and ThzNtnAntennaArray
 * for actual beam pattern computation.
 *
 * References:
 *   [1] M. A. Richards, "Fundamentals of Radar Signal Processing,"
 *       2nd ed., McGraw-Hill, 2014.
 *   [2] H. Rohling, "Radar CFAR thresholding in clutter and multiple
 *       target situations," IEEE Trans. Aerosp. Electron. Syst., 1983.
 *   [3] F. Liu et al., "Joint transmit beamforming for multiuser MIMO
 *       communications and MIMO radar," IEEE TSP, vol. 68, 2020.
 */

#ifndef THZ_NTN_ISAC_PROCESSOR_H
#define THZ_NTN_ISAC_PROCESSOR_H

#include <ns3/object.h>
#include <ns3/ptr.h>

#include <cstdint>
#include <map>
#include <utility>
#include <vector>

namespace ns3
{

// Forward declarations
class ThzNtnPhy;
class ThzNtnAntennaArray;

/**
 * \ingroup thz-ntn
 * \brief A single cell in the range-Doppler map.
 */
struct RangeDopplerBin
{
    uint32_t rangeIndex{0};
    uint32_t dopplerIndex{0};
    double   range_m{0.0};
    double   velocity_m_s{0.0};
    double   power_dB{0.0};
};

/**
 * \ingroup thz-ntn
 * \brief CFAR detection result.
 */
struct CfarResult
{
    std::vector<RangeDopplerBin> detectedTargets;
    double noiseFloor_dB{0.0};
    double threshold_dB{0.0};
    uint32_t numFalseAlarms{0};
};

/**
 * \ingroup thz-ntn
 * \brief ISAC signal processing for THz non-terrestrial networks.
 *
 * Provides range-Doppler map generation, CFAR detection, joint
 * beamforming gain computation, and multi-target tracking.  When
 * ThzNtnPhy and ThzNtnAntennaArray are set, signal parameters and
 * beam patterns come from actual configurations rather than passed
 * values.
 *
 * \warning Experimental: not yet exercised by any example or test.
 */
class ThzNtnIsacProcessor : public Object
{
  public:
    static TypeId GetTypeId();

    ThzNtnIsacProcessor();
    ~ThzNtnIsacProcessor() override;

    // ---- Sub-model setters ----

    /**
     * \brief Set the PHY model for actual bandwidth and integration time.
     * \param phy pointer to ThzNtnPhy
     */
    void SetPhy(Ptr<ThzNtnPhy> phy);

    /**
     * \brief Set the antenna array for actual beam pattern computation.
     * \param array pointer to ThzNtnAntennaArray
     */
    void SetAntennaArray(Ptr<ThzNtnAntennaArray> array);

    // ---- Range-Doppler processing ----

    /**
     * \brief Compute a range-Doppler map using actual PHY parameters.
     *
     * When PHY is set, max range = c / (2 * SCS) and max velocity = lambda * SCS / 2
     * are derived from actual bandwidth and center frequency.
     *
     * \param targetRange_m true target range in metres
     * \param targetVelocity_m_s true target velocity in m/s
     * \param snrLinear target signal-to-noise ratio (linear)
     * \return 2D power map in dB
     */
    std::vector<std::vector<double>> ComputeRangeDopplerMap(
        double targetRange_m,
        double targetVelocity_m_s,
        double snrLinear) const;

    /**
     * \brief Compute a range-Doppler map with explicit parameters (fallback).
     */
    std::vector<std::vector<double>> ComputeRangeDopplerMap(
        uint32_t numRangeBins,
        uint32_t numDopplerBins,
        double maxRange_m,
        double maxVelocity_m_s,
        double targetRange_m,
        double targetVelocity_m_s,
        double snrLinear) const;

    // ---- CFAR detection ----

    /**
     * \brief Perform CFAR detection using actual noise statistics.
     *
     * When PHY is set, noise floor is derived from actual noise temperature
     * and bandwidth.
     *
     * \param rdMap the range-Doppler map (2D power in dB)
     * \return CfarResult with detected targets
     */
    CfarResult PerformCfar(const std::vector<std::vector<double>>& rdMap) const;

    /**
     * \brief Perform CFAR detection with explicit parameters (fallback).
     */
    CfarResult PerformCfar(const std::vector<std::vector<double>>& rdMap,
                           double pfa,
                           uint32_t guardCells,
                           uint32_t refCells) const;

    // ---- Joint beamforming ----

    /**
     * \brief Compute joint beamforming gain using actual antenna array.
     *
     * When ThzNtnAntennaArray is set, the actual array factor is used
     * instead of analytical ULA approximation.
     *
     * \param commAngle_deg communication target angle in degrees
     * \param sensingAngle_deg sensing target angle in degrees
     * \return pair of [commGain_dB, sensingGain_dB]
     */
    std::pair<double, double> ComputeJointBeamformingGain(
        double commAngle_deg,
        double sensingAngle_deg) const;

    /**
     * \brief Compute beam pattern mismatch using actual antenna array.
     * \param commAngle_deg communication direction in degrees
     * \param sensingAngle_deg sensing direction in degrees
     * \return mismatch loss in dB
     */
    double ComputeBeamPatternMismatch_dB(double commAngle_deg,
                                          double sensingAngle_deg) const;

    // ---- Target tracking ----

    void UpdateTargetTrack(uint32_t targetId,
                           double range_m,
                           double velocity_m_s,
                           double timestamp_s);

    std::pair<double, double> PredictTargetPosition(uint32_t targetId,
                                                     double futureTime_s) const;

  private:
    uint32_t m_cfarGuardCells;
    uint32_t m_cfarRefCells;
    double   m_cfarPfa;
    uint32_t m_maxTargets;
    uint32_t m_numRangeBins;
    uint32_t m_numDopplerBins;

    Ptr<ThzNtnPhy> m_phy;
    Ptr<ThzNtnAntennaArray> m_array;

    struct TrackState
    {
        double lastRange_m{0.0};
        double lastVelocity_m_s{0.0};
        double lastTimestamp_s{0.0};
        uint32_t updateCount{0};
    };

    std::map<uint32_t, TrackState> m_tracks;

    static constexpr double PI = 3.14159265358979323846;
    static constexpr double SPEED_OF_LIGHT = 299792458.0;
};

} // namespace ns3

#endif // THZ_NTN_ISAC_PROCESSOR_H
