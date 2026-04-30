/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-only
 * Author: Muhammad Uzair
 *
 * Joint Radar-Communication (ISAC) Model for THz Non-Terrestrial Networks
 *
 * Implements Integrated Sensing and Communication (ISAC) at terahertz
 * frequencies for LEO satellite networks.  Integrates with ThzNtnPhy for
 * TX power / bandwidth / frequency, ThzNtnAntennaArray for actual antenna
 * gain, and MobilityModel for actual range to targets.
 *
 * References:
 *   [1] F. Liu et al., "Integrated sensing and communications: Toward
 *       dual-functional wireless networks for 6G and beyond," IEEE JSAC,
 *       vol. 40, no. 6, 2022.
 *   [2] ESA Space Debris Office, "ESA's annual space environment report,"
 *       2024.
 *   [3] C. Han et al., "Terahertz communications (TeraCom): Challenges
 *       and impact on 6G key performance indicators," IEEE COMST, 2024.
 *   [4] M. A. Richards, "Fundamentals of Radar Signal Processing,"
 *       2nd ed., McGraw-Hill, 2014.
 */

#ifndef THZ_NTN_ISAC_H
#define THZ_NTN_ISAC_H

#include <ns3/mobility-model.h>
#include <ns3/object.h>
#include <ns3/ptr.h>

#include <cstdint>
#include <string>
#include <vector>

namespace ns3
{

// Forward declarations
class ThzNtnPhy;
class ThzNtnAntennaArray;

/**
 * \ingroup thz-ntn
 * \brief ISAC operating mode.
 */
enum IsacMode : uint8_t
{
    COMMUNICATION_ONLY = 0,
    SENSING_ONLY,
    JOINT_ISAC,
    COMMUNICATION_CENTRIC,
    SENSING_CENTRIC
};

/**
 * \ingroup thz-ntn
 * \brief Type of sensing target.
 */
enum SensingTarget : uint8_t
{
    SPACE_DEBRIS = 0,
    SATELLITE,
    ATMOSPHERIC,
    GROUND_OBJECT
};

/**
 * \ingroup thz-ntn
 * \brief Result of a single sensing measurement.
 */
struct SensingResult
{
    bool   targetDetected{false};
    double estimatedRange_m{0.0};
    double estimatedVelocity_m_s{0.0};
    double estimatedAzimuth_deg{0.0};
    double estimatedElevation_deg{0.0};
    double radarCrossSection_m2{0.0};
    double snr_dB{0.0};
    double rangeResolution_m{0.0};
    double velocityResolution_m_s{0.0};
    double angularResolution_deg{0.0};
    double detectionProbability{0.0};
    double falseAlarmProbability{0.0};
};

/**
 * \ingroup thz-ntn
 * \brief Joint ISAC performance summary.
 */
struct IsacPerformance
{
    double commRate_Gbps{0.0};
    double commSinr_dB{0.0};
    double sensingRange_km{0.0};
    double rangeAccuracy_m{0.0};
    double velocityAccuracy_m_s{0.0};
    double resourceSplitRatio{0.0};
    double totalEfficiency{0.0};
};

/**
 * \ingroup thz-ntn
 * \brief Space debris radar cross-section model for THz frequencies.
 */
struct DebrisModel
{
    std::string sizeCategory;
    double rcs_dBsm{-40.0};
    double typicalAltitude_km{0.0};
    double typicalVelocity_km_s{0.0};
};

/**
 * \ingroup thz-ntn
 * \brief Joint radar-communication (ISAC) model for THz non-terrestrial networks.
 *
 * Integrates with ThzNtnPhy for actual TX power, bandwidth, and frequency;
 * ThzNtnAntennaArray for actual antenna gain; and MobilityModel for actual
 * range to sensing targets.
 */
class ThzNtnIsac : public Object
{
  public:
    static TypeId GetTypeId();

    ThzNtnIsac();
    ~ThzNtnIsac() override;

    // ---- Sub-model setters ----

    /**
     * \brief Set the PHY model for actual TX power, bandwidth, frequency.
     * \param phy pointer to ThzNtnPhy
     */
    void SetPhy(Ptr<ThzNtnPhy> phy);

    /**
     * \brief Set the antenna array for actual gain computation.
     * \param array pointer to ThzNtnAntennaArray
     */
    void SetAntennaArray(Ptr<ThzNtnAntennaArray> array);

    // ---- ISAC mode control ----

    void SetIsacMode(IsacMode mode);
    IsacMode GetIsacMode() const;
    void SetResourceSplit(double sensingFraction);
    double GetResourceSplit() const;

    // ---- MobilityModel-based sensing ----

    /**
     * \brief Perform sensing using actual satellite position and parameters.
     *
     * Range from MobilityModel distance, gain from ThzNtnAntennaArray,
     * TX power and bandwidth from ThzNtnPhy.
     *
     * \param satellite satellite MobilityModel
     * \param targetRange_m range to target in metres
     * \param targetRcs_m2 target radar cross section in m^2
     * \return SensingResult with detection outcome and estimates
     */
    SensingResult PerformSensing(Ptr<MobilityModel> satellite,
                                 double targetRange_m,
                                 double targetRcs_m2) const;

    // ---- Standalone computations (when sub-models not available) ----

    /**
     * \brief Compute radar SNR using the radar equation.
     * \param freqHz carrier frequency in Hz
     * \param txPowerDbm transmit power in dBm
     * \param totalGainDbi total antenna gain (Gt + Gr) in dBi
     * \param range_m range to target in metres
     * \param rcs_m2 radar cross section in m^2
     * \param bandwidthHz receiver bandwidth in Hz
     * \return radar SNR in dB
     */
    double ComputeRadarSnr_dB(double freqHz,
                              double txPowerDbm,
                              double totalGainDbi,
                              double range_m,
                              double rcs_m2,
                              double bandwidthHz) const;

    double ComputeRangeResolution_m(double bandwidthHz) const;
    double ComputeVelocityResolution_m_s(double freqHz, double integrationTime_s) const;
    double ComputeAngularResolution_deg(double freqHz, double aperture_m) const;
    double ComputeCramerRaoBound_range(double snrLinear, double bandwidthHz) const;
    double ComputeCramerRaoBound_velocity(double snrLinear, double freqHz,
                                          double integrationTime_s) const;
    double ComputeDetectionProbability(double snrDb, double pfa) const;

    /**
     * \brief Compute maximum detection range using actual parameters from PHY/antenna.
     * \param targetRcs_m2 target radar cross section in m^2
     * \param minSnrDb minimum required SNR in dB
     * \return maximum detection range in km
     */
    double ComputeMaxDetectionRange_km(double targetRcs_m2, double minSnrDb) const;

    /**
     * \brief Evaluate joint ISAC performance with actual parameters.
     * \param range_m link range in metres
     * \return IsacPerformance summary structure
     */
    IsacPerformance EvaluatePerformance(double range_m) const;

    DebrisModel GetDebrisModel(const std::string& sizeCategory) const;

  private:
    IsacMode m_isacMode;
    double   m_resourceSplit;
    double   m_integrationTime_s;
    double   m_targetPfa;
    double   m_noiseTemp_K;           ///< Receiver noise temperature (K)

    Ptr<ThzNtnPhy> m_phy;             ///< PHY model for TX power, BW, freq
    Ptr<ThzNtnAntennaArray> m_array;  ///< Antenna array for gain

    static constexpr double SPEED_OF_LIGHT = 299792458.0;
    static constexpr double BOLTZMANN_K = 1.380649e-23;
    static constexpr double PI = 3.14159265358979323846;
};

} // namespace ns3

#endif // THZ_NTN_ISAC_H
