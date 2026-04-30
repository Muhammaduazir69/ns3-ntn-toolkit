/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-only
 * Author: Muhammad Uzair
 *
 * Joint Radar-Communication (ISAC) Model for THz Non-Terrestrial Networks
 */

#include "thz-ntn-isac.h"

#include "thz-ntn-antenna-array.h"
#include "thz-ntn-phy.h"

#include <ns3/double.h>
#include <ns3/enum.h>
#include <ns3/log.h>
#include <ns3/uinteger.h>

#include <cmath>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("ThzNtnIsac");
NS_OBJECT_ENSURE_REGISTERED(ThzNtnIsac);

TypeId
ThzNtnIsac::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::ThzNtnIsac")
            .SetParent<Object>()
            .SetGroupName("ThzNtn")
            .AddConstructor<ThzNtnIsac>()
            .AddAttribute("IsacMode",
                          "ISAC operating mode",
                          EnumValue(JOINT_ISAC),
                          MakeEnumAccessor<IsacMode>(&ThzNtnIsac::m_isacMode),
                          MakeEnumChecker(COMMUNICATION_ONLY, "COMMUNICATION_ONLY",
                                         SENSING_ONLY, "SENSING_ONLY",
                                         JOINT_ISAC, "JOINT_ISAC",
                                         COMMUNICATION_CENTRIC, "COMMUNICATION_CENTRIC",
                                         SENSING_CENTRIC, "SENSING_CENTRIC"))
            .AddAttribute("ResourceSplit",
                          "Fraction of resources allocated to sensing (0 = all comms, "
                          "1 = all sensing)",
                          DoubleValue(0.3),
                          MakeDoubleAccessor(&ThzNtnIsac::m_resourceSplit),
                          MakeDoubleChecker<double>(0.0, 1.0))
            .AddAttribute("IntegrationTime_s",
                          "Coherent integration time in seconds",
                          DoubleValue(0.001),
                          MakeDoubleAccessor(&ThzNtnIsac::m_integrationTime_s),
                          MakeDoubleChecker<double>(1e-6, 1.0))
            .AddAttribute("TargetPfa",
                          "Target probability of false alarm",
                          DoubleValue(1e-6),
                          MakeDoubleAccessor(&ThzNtnIsac::m_targetPfa),
                          MakeDoubleChecker<double>(1e-12, 0.1))
            .AddAttribute("NoiseTemperature_K",
                          "Receiver noise temperature in Kelvin",
                          DoubleValue(290.0),
                          MakeDoubleAccessor(&ThzNtnIsac::m_noiseTemp_K),
                          MakeDoubleChecker<double>(1.0, 10000.0));
    return tid;
}

ThzNtnIsac::ThzNtnIsac()
    : m_isacMode(JOINT_ISAC),
      m_resourceSplit(0.3),
      m_integrationTime_s(0.001),
      m_targetPfa(1e-6),
      m_noiseTemp_K(290.0),
      m_phy(nullptr),
      m_array(nullptr)
{
    NS_LOG_FUNCTION(this);
}

ThzNtnIsac::~ThzNtnIsac()
{
    NS_LOG_FUNCTION(this);
}

// ---------------------------------------------------------------------------
// Sub-model setters
// ---------------------------------------------------------------------------

void
ThzNtnIsac::SetPhy(Ptr<ThzNtnPhy> phy)
{
    NS_LOG_FUNCTION(this << phy);
    m_phy = phy;
}

void
ThzNtnIsac::SetAntennaArray(Ptr<ThzNtnAntennaArray> array)
{
    NS_LOG_FUNCTION(this << array);
    m_array = array;
}

// ---------------------------------------------------------------------------
// ISAC mode control
// ---------------------------------------------------------------------------

void
ThzNtnIsac::SetIsacMode(IsacMode mode)
{
    NS_LOG_FUNCTION(this << static_cast<uint32_t>(mode));
    m_isacMode = mode;
}

IsacMode
ThzNtnIsac::GetIsacMode() const
{
    return m_isacMode;
}

void
ThzNtnIsac::SetResourceSplit(double sensingFraction)
{
    NS_LOG_FUNCTION(this << sensingFraction);
    m_resourceSplit = std::max(0.0, std::min(1.0, sensingFraction));
}

double
ThzNtnIsac::GetResourceSplit() const
{
    return m_resourceSplit;
}

// ---------------------------------------------------------------------------
// MobilityModel-based sensing
// ---------------------------------------------------------------------------

SensingResult
ThzNtnIsac::PerformSensing(Ptr<MobilityModel> satellite,
                            double targetRange_m,
                            double targetRcs_m2) const
{
    NS_LOG_FUNCTION(this << targetRange_m << targetRcs_m2);

    SensingResult result;

    // Get actual parameters from PHY if available, otherwise use defaults
    double freqHz = 300.0e9;
    double txPowerDbm = 30.0;
    double bandwidthHz = 20.0e9;

    if (m_phy)
    {
        freqHz = m_phy->GetCenterFrequency();
        txPowerDbm = m_phy->GetTxPower();
        bandwidthHz = m_phy->GetBandwidth();
    }

    // Get actual antenna gain from array if available
    double totalGainDbi = 50.0; // default
    if (m_array)
    {
        // For monostatic radar: Gt = Gr = array max gain
        double singleGainDbi = m_array->ComputeMaxGain_dBi();
        totalGainDbi = 2.0 * singleGainDbi;
    }

    // Compute radar SNR with actual parameters
    double snrDb = ComputeRadarSnr_dB(freqHz, txPowerDbm, totalGainDbi,
                                       targetRange_m, targetRcs_m2, bandwidthHz);

    // Apply sensing resource fraction
    double sensingFraction = m_resourceSplit;
    if (m_isacMode == COMMUNICATION_ONLY)
    {
        sensingFraction = 0.0;
    }
    else if (m_isacMode == SENSING_ONLY)
    {
        sensingFraction = 1.0;
    }
    snrDb += 10.0 * std::log10(std::max(sensingFraction, 1e-10));

    // Coherent integration gain: 10*log10(T_int * BW)
    double integrationGain_dB = 10.0 * std::log10(m_integrationTime_s * bandwidthHz);
    snrDb += integrationGain_dB;

    result.snr_dB = snrDb;

    // Compute resolutions from actual bandwidth and frequency
    result.rangeResolution_m = ComputeRangeResolution_m(bandwidthHz);
    result.velocityResolution_m_s = ComputeVelocityResolution_m_s(freqHz, m_integrationTime_s);

    // Angular resolution from actual aperture if array is available
    double aperture_m = 0.05; // default
    if (m_array)
    {
        aperture_m = m_array->ComputePhysicalSize_m();
    }
    result.angularResolution_deg = ComputeAngularResolution_deg(freqHz, aperture_m);

    // Detection probability
    result.falseAlarmProbability = m_targetPfa;
    result.detectionProbability = ComputeDetectionProbability(snrDb, m_targetPfa);
    result.targetDetected = (result.detectionProbability > 0.5);

    if (result.targetDetected)
    {
        result.estimatedRange_m = targetRange_m;
        result.estimatedVelocity_m_s = 0.0;
        result.estimatedAzimuth_deg = 0.0;
        result.estimatedElevation_deg = 0.0;
        result.radarCrossSection_m2 = targetRcs_m2;
    }

    return result;
}

// ---------------------------------------------------------------------------
// Radar equation
// ---------------------------------------------------------------------------

double
ThzNtnIsac::ComputeRadarSnr_dB(double freqHz,
                                double txPowerDbm,
                                double totalGainDbi,
                                double range_m,
                                double rcs_m2,
                                double bandwidthHz) const
{
    NS_LOG_FUNCTION(this << freqHz << txPowerDbm << totalGainDbi
                         << range_m << rcs_m2 << bandwidthHz);

    double lambda = SPEED_OF_LIGHT / freqHz;
    double ptDbw = txPowerDbm - 30.0;
    double lambdaSq_dB = 10.0 * std::log10(lambda * lambda);
    double rcs_dB = 10.0 * std::log10(rcs_m2);
    double fourPiCubed_dB = 10.0 * std::log10(std::pow(4.0 * PI, 3.0));
    double rangeFourth_dB = 40.0 * std::log10(range_m);

    // Noise power using actual noise temperature
    double noisePower_dBW = 10.0 * std::log10(BOLTZMANN_K * m_noiseTemp_K * bandwidthHz);

    double snrDb = ptDbw + totalGainDbi + lambdaSq_dB + rcs_dB
                   - fourPiCubed_dB - rangeFourth_dB - noisePower_dBW;

    NS_LOG_DEBUG("Radar SNR: " << snrDb << " dB at range " << range_m << " m");
    return snrDb;
}

// ---------------------------------------------------------------------------
// Resolution computations
// ---------------------------------------------------------------------------

double
ThzNtnIsac::ComputeRangeResolution_m(double bandwidthHz) const
{
    NS_LOG_FUNCTION(this << bandwidthHz);
    return SPEED_OF_LIGHT / (2.0 * bandwidthHz);
}

double
ThzNtnIsac::ComputeVelocityResolution_m_s(double freqHz, double integrationTime_s) const
{
    NS_LOG_FUNCTION(this << freqHz << integrationTime_s);
    double lambda = SPEED_OF_LIGHT / freqHz;
    return lambda / (2.0 * integrationTime_s);
}

double
ThzNtnIsac::ComputeAngularResolution_deg(double freqHz, double aperture_m) const
{
    NS_LOG_FUNCTION(this << freqHz << aperture_m);
    double lambda = SPEED_OF_LIGHT / freqHz;
    double resRad = lambda / aperture_m;
    return resRad * (180.0 / PI);
}

// ---------------------------------------------------------------------------
// CRB computations
// ---------------------------------------------------------------------------

double
ThzNtnIsac::ComputeCramerRaoBound_range(double snrLinear, double bandwidthHz) const
{
    NS_LOG_FUNCTION(this << snrLinear << bandwidthHz);
    if (snrLinear <= 0.0)
    {
        return 1e6;
    }
    return SPEED_OF_LIGHT / (2.0 * bandwidthHz * std::sqrt(2.0 * snrLinear));
}

double
ThzNtnIsac::ComputeCramerRaoBound_velocity(double snrLinear,
                                            double freqHz,
                                            double integrationTime_s) const
{
    NS_LOG_FUNCTION(this << snrLinear << freqHz << integrationTime_s);
    if (snrLinear <= 0.0)
    {
        return 1e6;
    }
    double lambda = SPEED_OF_LIGHT / freqHz;
    return lambda / (2.0 * integrationTime_s * std::sqrt(2.0 * snrLinear));
}

// ---------------------------------------------------------------------------
// Detection probability (Swerling-I)
// ---------------------------------------------------------------------------

double
ThzNtnIsac::ComputeDetectionProbability(double snrDb, double pfa) const
{
    NS_LOG_FUNCTION(this << snrDb << pfa);

    double snrLinear = std::pow(10.0, snrDb / 10.0);
    if (snrLinear <= 0.0)
    {
        return 0.0;
    }

    double pd = std::pow(pfa, 1.0 / (1.0 + snrLinear));

    NS_LOG_DEBUG("Pd = " << pd << " at SNR = " << snrDb << " dB, Pfa = " << pfa);
    return pd;
}

// ---------------------------------------------------------------------------
// Max detection range using actual parameters from PHY/antenna
// ---------------------------------------------------------------------------

double
ThzNtnIsac::ComputeMaxDetectionRange_km(double targetRcs_m2, double minSnrDb) const
{
    NS_LOG_FUNCTION(this << targetRcs_m2 << minSnrDb);

    double freqHz = 300.0e9;
    double txPowerDbm = 30.0;
    double bandwidthHz = 20.0e9;

    if (m_phy)
    {
        freqHz = m_phy->GetCenterFrequency();
        txPowerDbm = m_phy->GetTxPower();
        bandwidthHz = m_phy->GetBandwidth();
    }

    double totalGainDbi = 50.0;
    if (m_array)
    {
        totalGainDbi = 2.0 * m_array->ComputeMaxGain_dBi();
    }

    double ptW = std::pow(10.0, (txPowerDbm - 30.0) / 10.0);
    double gainLinear = std::pow(10.0, totalGainDbi / 10.0);
    double lambda = SPEED_OF_LIGHT / freqHz;
    double snrMinLinear = std::pow(10.0, minSnrDb / 10.0);
    double noisePowerW = BOLTZMANN_K * m_noiseTemp_K * bandwidthHz;

    double numerator = ptW * gainLinear * lambda * lambda * targetRcs_m2;
    double denominator = std::pow(4.0 * PI, 3.0) * noisePowerW * snrMinLinear;

    if (denominator <= 0.0)
    {
        return 0.0;
    }

    double rFourth = numerator / denominator;
    double rangeM = std::pow(rFourth, 0.25);

    return rangeM / 1000.0;
}

// ---------------------------------------------------------------------------
// Joint performance evaluation with actual parameters
// ---------------------------------------------------------------------------

IsacPerformance
ThzNtnIsac::EvaluatePerformance(double range_m) const
{
    NS_LOG_FUNCTION(this << range_m);

    IsacPerformance perf;
    perf.resourceSplitRatio = m_resourceSplit;

    // Get actual parameters from PHY
    double freqHz = 300.0e9;
    double txPowerDbm = 30.0;
    double bandwidthHz = 20.0e9;

    if (m_phy)
    {
        freqHz = m_phy->GetCenterFrequency();
        txPowerDbm = m_phy->GetTxPower();
        bandwidthHz = m_phy->GetBandwidth();
    }

    // Get actual antenna gain
    double commGainDbi = 40.0;
    double sensingGainDbi = 50.0;
    if (m_array)
    {
        double maxGain = m_array->ComputeMaxGain_dBi();
        commGainDbi = maxGain;
        sensingGainDbi = 2.0 * maxGain; // monostatic
    }

    // Communication performance
    double commFraction = 1.0 - m_resourceSplit;
    if (m_isacMode == SENSING_ONLY)
    {
        commFraction = 0.0;
    }
    else if (m_isacMode == COMMUNICATION_ONLY)
    {
        commFraction = 1.0;
    }

    double lambda = SPEED_OF_LIGHT / freqHz;
    double fspl_dB = 20.0 * std::log10(4.0 * PI * range_m / lambda);
    double ptDbw = txPowerDbm - 30.0;
    double commBw = bandwidthHz * commFraction;

    if (commBw > 0.0)
    {
        double noisePowerDbw = 10.0 * std::log10(BOLTZMANN_K * m_noiseTemp_K * commBw);
        perf.commSinr_dB = ptDbw + commGainDbi - fspl_dB - noisePowerDbw
                            + 10.0 * std::log10(std::max(commFraction, 1e-10));

        double commSinrLinear = std::pow(10.0, perf.commSinr_dB / 10.0);
        double capacityBps = commBw * std::log2(1.0 + commSinrLinear);
        perf.commRate_Gbps = capacityBps / 1e9;
    }

    // Sensing performance
    double sensingBw = bandwidthHz * m_resourceSplit;
    if (sensingBw > 0.0)
    {
        double rcs_m2 = std::pow(10.0, -20.0 / 10.0);
        double snrDb = ComputeRadarSnr_dB(freqHz, txPowerDbm, sensingGainDbi,
                                           range_m, rcs_m2, sensingBw);
        double snrLinear = std::pow(10.0, snrDb / 10.0);

        perf.sensingRange_km = ComputeMaxDetectionRange_km(rcs_m2, 10.0);
        perf.rangeAccuracy_m = ComputeCramerRaoBound_range(
            std::max(snrLinear, 1.0), sensingBw);
        perf.velocityAccuracy_m_s = ComputeCramerRaoBound_velocity(
            std::max(snrLinear, 1.0), freqHz, m_integrationTime_s);
    }

    // Total efficiency
    double commWeight = 1.0 - m_resourceSplit;
    double sensingWeight = m_resourceSplit;
    double normComm = std::min(perf.commRate_Gbps / 100.0, 1.0);
    double normSensing = std::min(perf.sensingRange_km / 100.0, 1.0);
    perf.totalEfficiency = commWeight * normComm + sensingWeight * normSensing;

    return perf;
}

// ---------------------------------------------------------------------------
// Debris model
// ---------------------------------------------------------------------------

DebrisModel
ThzNtnIsac::GetDebrisModel(const std::string& sizeCategory) const
{
    NS_LOG_FUNCTION(this << sizeCategory);

    DebrisModel model;
    model.sizeCategory = sizeCategory;

    if (sizeCategory == "small_1cm")
    {
        model.rcs_dBsm = -40.0;
        model.typicalAltitude_km = 800.0;
        model.typicalVelocity_km_s = 7.5;
    }
    else if (sizeCategory == "medium_10cm")
    {
        model.rcs_dBsm = -20.0;
        model.typicalAltitude_km = 700.0;
        model.typicalVelocity_km_s = 7.8;
    }
    else if (sizeCategory == "large_1m")
    {
        model.rcs_dBsm = 0.0;
        model.typicalAltitude_km = 600.0;
        model.typicalVelocity_km_s = 7.6;
    }
    else
    {
        NS_LOG_WARN("Unknown debris size category: " << sizeCategory
                    << ", defaulting to medium_10cm");
        model.sizeCategory = "medium_10cm";
        model.rcs_dBsm = -20.0;
        model.typicalAltitude_km = 700.0;
        model.typicalVelocity_km_s = 7.8;
    }

    return model;
}

} // namespace ns3
