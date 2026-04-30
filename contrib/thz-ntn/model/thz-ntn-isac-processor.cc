/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-only
 * Author: Muhammad Uzair
 *
 * ISAC Signal Processing for THz Non-Terrestrial Networks
 */

#include "thz-ntn-isac-processor.h"

#include "thz-ntn-antenna-array.h"
#include "thz-ntn-phy.h"

#include <ns3/double.h>
#include <ns3/log.h>
#include <ns3/uinteger.h>

#include <algorithm>
#include <cmath>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("ThzNtnIsacProcessor");
NS_OBJECT_ENSURE_REGISTERED(ThzNtnIsacProcessor);

TypeId
ThzNtnIsacProcessor::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::ThzNtnIsacProcessor")
            .SetParent<Object>()
            .SetGroupName("ThzNtn")
            .AddConstructor<ThzNtnIsacProcessor>()
            .AddAttribute("CfarGuardCells",
                          "Number of CFAR guard cells on each side of the cell under test",
                          UintegerValue(2),
                          MakeUintegerAccessor(&ThzNtnIsacProcessor::m_cfarGuardCells),
                          MakeUintegerChecker<uint32_t>(0, 16))
            .AddAttribute("CfarRefCells",
                          "Number of CFAR reference cells on each side of the cell under test",
                          UintegerValue(8),
                          MakeUintegerAccessor(&ThzNtnIsacProcessor::m_cfarRefCells),
                          MakeUintegerChecker<uint32_t>(1, 64))
            .AddAttribute("CfarPfa",
                          "CFAR target probability of false alarm",
                          DoubleValue(1e-6),
                          MakeDoubleAccessor(&ThzNtnIsacProcessor::m_cfarPfa),
                          MakeDoubleChecker<double>(1e-12, 0.1))
            .AddAttribute("MaxTargets",
                          "Maximum number of simultaneously tracked targets",
                          UintegerValue(10),
                          MakeUintegerAccessor(&ThzNtnIsacProcessor::m_maxTargets),
                          MakeUintegerChecker<uint32_t>(1, 100))
            .AddAttribute("NumRangeBins",
                          "Number of range bins in the RD map",
                          UintegerValue(256),
                          MakeUintegerAccessor(&ThzNtnIsacProcessor::m_numRangeBins),
                          MakeUintegerChecker<uint32_t>(16, 4096))
            .AddAttribute("NumDopplerBins",
                          "Number of Doppler bins in the RD map",
                          UintegerValue(128),
                          MakeUintegerAccessor(&ThzNtnIsacProcessor::m_numDopplerBins),
                          MakeUintegerChecker<uint32_t>(16, 4096));
    return tid;
}

ThzNtnIsacProcessor::ThzNtnIsacProcessor()
    : m_cfarGuardCells(2),
      m_cfarRefCells(8),
      m_cfarPfa(1e-6),
      m_maxTargets(10),
      m_numRangeBins(256),
      m_numDopplerBins(128),
      m_phy(nullptr),
      m_array(nullptr)
{
    NS_LOG_FUNCTION(this);
}

ThzNtnIsacProcessor::~ThzNtnIsacProcessor()
{
    NS_LOG_FUNCTION(this);
}

// ---------------------------------------------------------------------------
// Sub-model setters
// ---------------------------------------------------------------------------

void
ThzNtnIsacProcessor::SetPhy(Ptr<ThzNtnPhy> phy)
{
    NS_LOG_FUNCTION(this << phy);
    m_phy = phy;
}

void
ThzNtnIsacProcessor::SetAntennaArray(Ptr<ThzNtnAntennaArray> array)
{
    NS_LOG_FUNCTION(this << array);
    m_array = array;
}

// ---------------------------------------------------------------------------
// Range-Doppler map using actual PHY parameters
// ---------------------------------------------------------------------------

std::vector<std::vector<double>>
ThzNtnIsacProcessor::ComputeRangeDopplerMap(double targetRange_m,
                                             double targetVelocity_m_s,
                                             double snrLinear) const
{
    NS_LOG_FUNCTION(this << targetRange_m << targetVelocity_m_s << snrLinear);

    // Derive max range and max velocity from actual PHY parameters
    double bandwidthHz = 20.0e9;
    double freqHz = 300.0e9;

    if (m_phy)
    {
        bandwidthHz = m_phy->GetBandwidth();
        freqHz = m_phy->GetCenterFrequency();
    }

    // Max unambiguous range: R_max = c / (2 * delta_f)
    // delta_f = bandwidth / numRangeBins
    double deltaF = bandwidthHz / static_cast<double>(m_numRangeBins);
    double maxRange_m = SPEED_OF_LIGHT / (2.0 * deltaF);

    // Max unambiguous velocity: v_max = lambda / (4 * T_int)
    // where T_int = 1 / deltaF_Doppler, deltaF_Doppler = 1 / (numDopplerBins * PRI)
    double lambda = SPEED_OF_LIGHT / freqHz;
    double maxVelocity_m_s = lambda * bandwidthHz / (4.0 * static_cast<double>(m_numRangeBins));

    return ComputeRangeDopplerMap(m_numRangeBins, m_numDopplerBins,
                                  maxRange_m, maxVelocity_m_s,
                                  targetRange_m, targetVelocity_m_s,
                                  snrLinear);
}

std::vector<std::vector<double>>
ThzNtnIsacProcessor::ComputeRangeDopplerMap(uint32_t numRangeBins,
                                             uint32_t numDopplerBins,
                                             double maxRange_m,
                                             double maxVelocity_m_s,
                                             double targetRange_m,
                                             double targetVelocity_m_s,
                                             double snrLinear) const
{
    NS_LOG_FUNCTION(this << numRangeBins << numDopplerBins << maxRange_m
                         << maxVelocity_m_s << targetRange_m
                         << targetVelocity_m_s << snrLinear);

    std::vector<std::vector<double>> rdMap(numRangeBins,
                                            std::vector<double>(numDopplerBins, 0.0));

    double rangeBinSize = maxRange_m / static_cast<double>(numRangeBins);
    double dopplerBinSize = (2.0 * maxVelocity_m_s) / static_cast<double>(numDopplerBins);

    uint32_t targetRangeBin = static_cast<uint32_t>(targetRange_m / rangeBinSize);
    uint32_t targetDopplerBin = static_cast<uint32_t>(
        (targetVelocity_m_s + maxVelocity_m_s) / dopplerBinSize);

    targetRangeBin = std::min(targetRangeBin, numRangeBins - 1);
    targetDopplerBin = std::min(targetDopplerBin, numDopplerBins - 1);

    double targetPower_dB = 10.0 * std::log10(snrLinear + 1.0);
    rdMap[targetRangeBin][targetDopplerBin] = targetPower_dB;

    // Sinc-like sidelobes around the target
    for (uint32_t ri = 0; ri < numRangeBins; ++ri)
    {
        for (uint32_t di = 0; di < numDopplerBins; ++di)
        {
            if (ri == targetRangeBin && di == targetDopplerBin)
            {
                continue;
            }

            double rangeDist = std::abs(static_cast<double>(ri) -
                                        static_cast<double>(targetRangeBin));
            double dopplerDist = std::abs(static_cast<double>(di) -
                                          static_cast<double>(targetDopplerBin));
            double dist = std::sqrt(rangeDist * rangeDist + dopplerDist * dopplerDist);

            if (dist > 0.0 && dist < 5.0)
            {
                double sidelobePower = targetPower_dB - 13.0 -
                                       10.0 * std::log10(dist * dist);
                rdMap[ri][di] = std::max(rdMap[ri][di], sidelobePower);
            }
        }
    }

    NS_LOG_DEBUG("RD map generated: target at bin [" << targetRangeBin << ","
                 << targetDopplerBin << "] with " << targetPower_dB << " dB");
    return rdMap;
}

// ---------------------------------------------------------------------------
// CFAR detection
// ---------------------------------------------------------------------------

CfarResult
ThzNtnIsacProcessor::PerformCfar(const std::vector<std::vector<double>>& rdMap) const
{
    NS_LOG_FUNCTION(this);
    return PerformCfar(rdMap, m_cfarPfa, m_cfarGuardCells, m_cfarRefCells);
}

CfarResult
ThzNtnIsacProcessor::PerformCfar(const std::vector<std::vector<double>>& rdMap,
                                  double pfa,
                                  uint32_t guardCells,
                                  uint32_t refCells) const
{
    NS_LOG_FUNCTION(this << pfa << guardCells << refCells);

    CfarResult result;

    if (rdMap.empty() || rdMap[0].empty())
    {
        return result;
    }

    uint32_t numRange = static_cast<uint32_t>(rdMap.size());
    uint32_t numDoppler = static_cast<uint32_t>(rdMap[0].size());
    uint32_t margin = guardCells + refCells;

    // CA-CFAR threshold factor
    uint32_t totalRefCells = 4 * refCells;
    if (totalRefCells == 0)
    {
        totalRefCells = 1;
    }
    double alpha = static_cast<double>(totalRefCells) *
                   (std::pow(pfa, -1.0 / static_cast<double>(totalRefCells)) - 1.0);
    double alpha_dB = 10.0 * std::log10(1.0 + alpha);

    double sumNoise = 0.0;
    uint32_t noiseCount = 0;

    for (uint32_t ri = margin; ri < numRange - margin; ++ri)
    {
        for (uint32_t di = margin; di < numDoppler - margin; ++di)
        {
            double refSum = 0.0;
            uint32_t refCount = 0;

            for (uint32_t k = guardCells + 1; k <= margin; ++k)
            {
                if (ri >= k)
                {
                    refSum += std::pow(10.0, rdMap[ri - k][di] / 10.0);
                    refCount++;
                }
                if (ri + k < numRange)
                {
                    refSum += std::pow(10.0, rdMap[ri + k][di] / 10.0);
                    refCount++;
                }
                if (di >= k)
                {
                    refSum += std::pow(10.0, rdMap[ri][di - k] / 10.0);
                    refCount++;
                }
                if (di + k < numDoppler)
                {
                    refSum += std::pow(10.0, rdMap[ri][di + k] / 10.0);
                    refCount++;
                }
            }

            if (refCount == 0)
            {
                continue;
            }

            double avgNoiseLin = refSum / static_cast<double>(refCount);
            double thresholdLin = avgNoiseLin * (1.0 + alpha);

            double cellPowerLin = std::pow(10.0, rdMap[ri][di] / 10.0);

            sumNoise += avgNoiseLin;
            noiseCount++;

            if (cellPowerLin > thresholdLin)
            {
                RangeDopplerBin bin;
                bin.rangeIndex = ri;
                bin.dopplerIndex = di;
                bin.power_dB = rdMap[ri][di];
                bin.range_m = 0.0;
                bin.velocity_m_s = 0.0;
                result.detectedTargets.push_back(bin);
            }
        }
    }

    if (noiseCount > 0)
    {
        result.noiseFloor_dB = 10.0 * std::log10(sumNoise / static_cast<double>(noiseCount));
    }
    result.threshold_dB = result.noiseFloor_dB + alpha_dB;

    NS_LOG_DEBUG("CFAR: " << result.detectedTargets.size() << " targets detected, "
                 << "noise floor = " << result.noiseFloor_dB << " dB, "
                 << "threshold = " << result.threshold_dB << " dB");

    return result;
}

// ---------------------------------------------------------------------------
// Joint beamforming using actual antenna array
// ---------------------------------------------------------------------------

std::pair<double, double>
ThzNtnIsacProcessor::ComputeJointBeamformingGain(double commAngle_deg,
                                                  double sensingAngle_deg) const
{
    NS_LOG_FUNCTION(this << commAngle_deg << sensingAngle_deg);

    if (m_array)
    {
        // Use actual antenna array for beam pattern computation
        // Steer to the midpoint direction
        double midAngle = (commAngle_deg + sensingAngle_deg) / 2.0;

        // Compute gains from actual array factor
        double commGain_dBi = m_array->ComputeArrayGain_dBi(commAngle_deg, 0.0, midAngle, 0.0);
        double sensingGain_dBi = m_array->ComputeArrayGain_dBi(sensingAngle_deg, 0.0,
                                                                midAngle, 0.0);

        NS_LOG_DEBUG("Joint BF gain (array): comm = " << commGain_dBi
                     << " dBi, sensing = " << sensingGain_dBi << " dBi");
        return std::make_pair(commGain_dBi, sensingGain_dBi);
    }

    // Fallback: analytical ULA model
    uint32_t numAntennas = 1024; // default
    double commAngleRad = commAngle_deg * PI / 180.0;
    double sensingAngleRad = sensingAngle_deg * PI / 180.0;
    double midAngleRad = (commAngleRad + sensingAngleRad) / 2.0;

    double N = static_cast<double>(numAntennas);

    // Array factor gain at communication angle
    double psiComm = PI * (std::sin(commAngleRad) - std::sin(midAngleRad));
    double commGainLin;
    if (std::abs(psiComm) < 1e-10)
    {
        commGainLin = N;
    }
    else
    {
        double num = std::sin(N * psiComm / 2.0);
        double den = N * std::sin(psiComm / 2.0);
        commGainLin = N * (num * num) / (den * den + 1e-30);
    }

    // Array factor gain at sensing angle
    double psiSens = PI * (std::sin(sensingAngleRad) - std::sin(midAngleRad));
    double sensingGainLin;
    if (std::abs(psiSens) < 1e-10)
    {
        sensingGainLin = N;
    }
    else
    {
        double num = std::sin(N * psiSens / 2.0);
        double den = N * std::sin(psiSens / 2.0);
        sensingGainLin = N * (num * num) / (den * den + 1e-30);
    }

    double commGain_dB = 10.0 * std::log10(std::max(commGainLin, 1e-10));
    double sensingGain_dB = 10.0 * std::log10(std::max(sensingGainLin, 1e-10));

    NS_LOG_DEBUG("Joint BF gain (fallback): comm = " << commGain_dB << " dB, sensing = "
                 << sensingGain_dB << " dB");
    return std::make_pair(commGain_dB, sensingGain_dB);
}

double
ThzNtnIsacProcessor::ComputeBeamPatternMismatch_dB(double commAngle_deg,
                                                    double sensingAngle_deg) const
{
    NS_LOG_FUNCTION(this << commAngle_deg << sensingAngle_deg);

    if (m_array)
    {
        // Compute mismatch from actual beam pattern
        double maxGain = m_array->ComputeMaxGain_dBi();
        double beamwidth = m_array->ComputeBeamwidth3dB_deg();

        double angleDiff = std::abs(commAngle_deg - sensingAngle_deg);
        double normalizedDiff = angleDiff / std::max(beamwidth, 0.01);
        double mismatch_dB = 3.0 * normalizedDiff * normalizedDiff;
        return std::min(mismatch_dB, maxGain);
    }

    // Fallback
    double angleDiff = std::abs(commAngle_deg - sensingAngle_deg);
    double hpbw_deg = 1.0;
    double normalizedDiff = angleDiff / hpbw_deg;
    double mismatch_dB = 3.0 * normalizedDiff * normalizedDiff;
    return std::min(mismatch_dB, 30.0);
}

// ---------------------------------------------------------------------------
// Target tracking
// ---------------------------------------------------------------------------

void
ThzNtnIsacProcessor::UpdateTargetTrack(uint32_t targetId,
                                        double range_m,
                                        double velocity_m_s,
                                        double timestamp_s)
{
    NS_LOG_FUNCTION(this << targetId << range_m << velocity_m_s << timestamp_s);

    if (m_tracks.size() >= m_maxTargets && m_tracks.find(targetId) == m_tracks.end())
    {
        NS_LOG_WARN("Maximum number of tracked targets reached (" << m_maxTargets
                    << "), ignoring target " << targetId);
        return;
    }

    TrackState& track = m_tracks[targetId];

    if (track.updateCount > 0)
    {
        double alpha = 0.3;
        track.lastRange_m = alpha * range_m + (1.0 - alpha) * track.lastRange_m;
        track.lastVelocity_m_s = alpha * velocity_m_s +
                                  (1.0 - alpha) * track.lastVelocity_m_s;
    }
    else
    {
        track.lastRange_m = range_m;
        track.lastVelocity_m_s = velocity_m_s;
    }

    track.lastTimestamp_s = timestamp_s;
    track.updateCount++;

    NS_LOG_DEBUG("Track " << targetId << " updated: R = " << track.lastRange_m
                 << " m, V = " << track.lastVelocity_m_s << " m/s");
}

std::pair<double, double>
ThzNtnIsacProcessor::PredictTargetPosition(uint32_t targetId,
                                            double futureTime_s) const
{
    NS_LOG_FUNCTION(this << targetId << futureTime_s);

    auto it = m_tracks.find(targetId);
    if (it == m_tracks.end())
    {
        NS_LOG_WARN("No track found for target " << targetId);
        return std::make_pair(0.0, 0.0);
    }

    const TrackState& track = it->second;

    double dt = futureTime_s - track.lastTimestamp_s;
    double predictedRange = track.lastRange_m + track.lastVelocity_m_s * dt;
    double predictedVelocity = track.lastVelocity_m_s;

    NS_LOG_DEBUG("Predicted target " << targetId << " at t=" << futureTime_s
                 << ": R = " << predictedRange << " m, V = " << predictedVelocity
                 << " m/s");

    return std::make_pair(predictedRange, predictedVelocity);
}

} // namespace ns3
