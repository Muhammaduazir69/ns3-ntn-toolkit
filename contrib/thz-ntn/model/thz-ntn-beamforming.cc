/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-only
 * Author: Muhammad Uzair
 *
 * THz Beam Management for LEO NTN — implementation
 */

#include "thz-ntn-beamforming.h"

#include "thz-ntn-antenna-array.h"

// mmWave module header for beamforming model bridge
#include <ns3/mmwave-beamforming-model.h>

#include <ns3/boolean.h>
#include <ns3/double.h>
#include <ns3/log.h>
#include <ns3/mobility-model.h>
#include <ns3/string.h>
#include <ns3/uinteger.h>

#include <algorithm>
#include <cmath>
#include <complex>
#include <limits>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("ThzNtnBeamforming");
NS_OBJECT_ENSURE_REGISTERED(ThzNtnBeamforming);

static constexpr double PI      = 3.14159265358979323846;
static constexpr double DEG2RAD = PI / 180.0;
static constexpr double RAD2DEG = 180.0 / PI;
static constexpr double SPEED_OF_LIGHT = 299792458.0;

// ---- TypeId ----

TypeId
ThzNtnBeamforming::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::ThzNtnBeamforming")
            .SetParent<Object>()
            .SetGroupName("ThzNtn")
            .AddConstructor<ThzNtnBeamforming>()
            .AddAttribute("Mode",
                          "Beamforming architecture: ANALOG_ONLY, HYBRID, or DIGITAL_ONLY.",
                          StringValue("HYBRID"),
                          MakeStringAccessor(&ThzNtnBeamforming::m_modeStr),
                          MakeStringChecker())
            .AddAttribute("NumAnalogBeams",
                          "Number of analog beams.",
                          UintegerValue(64),
                          MakeUintegerAccessor(&ThzNtnBeamforming::m_numAnalogBeams),
                          MakeUintegerChecker<uint32_t>(1, 4096))
            .AddAttribute("NumDigitalStreams",
                          "Number of digital baseband streams.",
                          UintegerValue(4),
                          MakeUintegerAccessor(&ThzNtnBeamforming::m_numDigitalStreams),
                          MakeUintegerChecker<uint32_t>(1, 256))
            .AddAttribute("PhaseShifterBits",
                          "Phase quantisation resolution in bits.",
                          UintegerValue(4),
                          MakeUintegerAccessor(&ThzNtnBeamforming::m_phaseShifterBits),
                          MakeUintegerChecker<uint32_t>(1, 12))
            .AddAttribute("BeamSwitchDelay_us",
                          "Beam switching latency in microseconds.",
                          DoubleValue(50.0),
                          MakeDoubleAccessor(&ThzNtnBeamforming::m_beamSwitchDelay_us),
                          MakeDoubleChecker<double>(0.0, 1e6))
            .AddAttribute("EnableBeamSquintComp",
                          "Enable wideband beam squint compensation.",
                          BooleanValue(true),
                          MakeBooleanAccessor(&ThzNtnBeamforming::m_enableSquintComp),
                          MakeBooleanChecker())
            .AddAttribute("Frequency",
                          "Centre frequency in Hz for codebook design.",
                          DoubleValue(300e9),
                          MakeDoubleAccessor(&ThzNtnBeamforming::m_frequency),
                          MakeDoubleChecker<double>(1e9, 10e12))
            .AddAttribute("Bandwidth",
                          "Signal bandwidth in Hz for beam squint computation.",
                          DoubleValue(10e9),
                          MakeDoubleAccessor(&ThzNtnBeamforming::m_bandwidthHz),
                          MakeDoubleChecker<double>(1e6, 100e9))
            .AddAttribute("NumElements",
                          "Total number of antenna elements.",
                          UintegerValue(1024),
                          MakeUintegerAccessor(&ThzNtnBeamforming::m_numElements),
                          MakeUintegerChecker<uint32_t>(1, 65536));
    return tid;
}

// ---- Constructor / destructor ----

ThzNtnBeamforming::ThzNtnBeamforming()
    : m_mode(ThzBeamformingMode::HYBRID),
      m_mmWaveBfModel(nullptr),
      m_antennaArray(nullptr),
      m_modeStr("HYBRID"),
      m_numAnalogBeams(64),
      m_numDigitalStreams(4),
      m_phaseShifterBits(4),
      m_beamSwitchDelay_us(50.0),
      m_enableSquintComp(true),
      m_frequency(300e9),
      m_bandwidthHz(10e9),
      m_numElements(1024)
{
    NS_LOG_FUNCTION(this);

    // Pre-generate default codebooks for all levels
    GenerateCodebook(DefaultNumBeams(ThzCodebookLevel::WIDE), ThzCodebookLevel::WIDE);
    GenerateCodebook(DefaultNumBeams(ThzCodebookLevel::MEDIUM), ThzCodebookLevel::MEDIUM);
    GenerateCodebook(DefaultNumBeams(ThzCodebookLevel::NARROW), ThzCodebookLevel::NARROW);
    GenerateCodebook(DefaultNumBeams(ThzCodebookLevel::ULTRA_NARROW),
                     ThzCodebookLevel::ULTRA_NARROW);
}

ThzNtnBeamforming::~ThzNtnBeamforming()
{
    NS_LOG_FUNCTION(this);
}

void
ThzNtnBeamforming::ParseModeString()
{
    if (m_modeStr == "ANALOG_ONLY")
    {
        m_mode = ThzBeamformingMode::ANALOG_ONLY;
    }
    else if (m_modeStr == "DIGITAL_ONLY")
    {
        m_mode = ThzBeamformingMode::DIGITAL_ONLY;
    }
    else
    {
        m_mode = ThzBeamformingMode::HYBRID;
    }
}

uint32_t
ThzNtnBeamforming::DefaultNumBeams(ThzCodebookLevel level)
{
    switch (level)
    {
    case ThzCodebookLevel::WIDE: return 4;
    case ThzCodebookLevel::MEDIUM: return 16;
    case ThzCodebookLevel::NARROW: return 64;
    case ThzCodebookLevel::ULTRA_NARROW: return 256;
    }
    return 64;
}

// ---- MmWaveBeamformingModel bridge ----

void
ThzNtnBeamforming::SetMmWaveBeamformingModel(Ptr<mmwave::MmWaveBeamformingModel> bfModel)
{
    NS_LOG_FUNCTION(this);
    m_mmWaveBfModel = bfModel;
}

void
ThzNtnBeamforming::SetAntennaArray(Ptr<ThzNtnAntennaArray> array)
{
    NS_LOG_FUNCTION(this);
    m_antennaArray = array;
}

// ---- Codebook generation ----

void
ThzNtnBeamforming::GenerateCodebook(uint32_t numBeams, ThzCodebookLevel level)
{
    NS_LOG_FUNCTION(this << numBeams << static_cast<int>(level));

    uint32_t idx = static_cast<uint32_t>(level);
    m_codebook[idx].clear();
    m_codebook[idx].reserve(numBeams);

    // Wavelength from actual frequency
    double lambda = SPEED_OF_LIGHT / m_frequency;
    double d = lambda / 2.0; // element spacing from wavelength

    for (uint32_t k = 0; k < numBeams; ++k)
    {
        ThzCodebookEntry entry;
        entry.index = k;

        double sinTheta;
        if (numBeams > 1)
        {
            sinTheta = -1.0 + 2.0 * static_cast<double>(k) /
                                     static_cast<double>(numBeams - 1);
        }
        else
        {
            sinTheta = 0.0;
        }
        sinTheta = std::max(-1.0, std::min(1.0, sinTheta));

        entry.thetaCenter_deg = std::asin(sinTheta) * RAD2DEG;
        entry.phiCenter_deg   = 0.0;

        // Beamwidth from actual array geometry
        double Neff = static_cast<double>(m_numElements) /
                      static_cast<double>(numBeams);
        Neff = std::max(Neff, 1.0);
        double bw_rad = 0.886 * lambda / (std::sqrt(Neff) * d);
        entry.beamwidth_deg = bw_rad * RAD2DEG;

        // Peak gain from array: 10*log10(N) + element_gain - quantisation loss
        double arrayGain = 10.0 * std::log10(static_cast<double>(m_numElements));
        double quantLoss = 0.0;
        if (m_phaseShifterBits < 8)
        {
            double denom = std::pow(2.0, static_cast<double>(m_phaseShifterBits));
            quantLoss = std::pow(PI / (denom * std::sqrt(3.0)), 2.0);
            quantLoss = -10.0 * std::log10(1.0 - quantLoss);
        }
        entry.gain_dBi = arrayGain + 5.0 - quantLoss;

        m_codebook[idx].push_back(entry);
    }

    NS_LOG_INFO("Generated codebook level " << idx << " with " << numBeams
                << " beams, beamwidth ~" << m_codebook[idx][0].beamwidth_deg << " deg");
}

std::vector<ThzCodebookEntry>
ThzNtnBeamforming::GetCodebook(ThzCodebookLevel level) const
{
    return m_codebook[static_cast<uint32_t>(level)];
}

// ---- Beam selection from MobilityModel positions ----

uint32_t
ThzNtnBeamforming::SelectBeamForTarget(Ptr<MobilityModel> target,
                                       Ptr<MobilityModel> self,
                                       ThzCodebookLevel level) const
{
    NS_LOG_FUNCTION(this);

    if (!target || !self)
    {
        NS_LOG_WARN("Null MobilityModel, returning beam 0");
        return 0;
    }

    uint32_t idx = static_cast<uint32_t>(level);
    const auto& codebook = m_codebook[idx];

    if (codebook.empty())
    {
        return 0;
    }

    if (m_antennaArray)
    {
        // Evaluate actual array gain for each codebook beam toward the target
        // and select the one with highest gain.
        Vector selfPos = self->GetPosition();
        Vector targetPos = target->GetPosition();

        // Compute target direction in antenna frame
        double dx = targetPos.x - selfPos.x;
        double dy = targetPos.y - selfPos.y;
        double dz = targetPos.z - selfPos.z;
        double dist = std::sqrt(dx * dx + dy * dy + dz * dz);

        if (dist < 1.0)
        {
            return 0;
        }

        // Compute target theta/phi in antenna frame
        double selfR = std::sqrt(selfPos.x * selfPos.x +
                                 selfPos.y * selfPos.y +
                                 selfPos.z * selfPos.z);
        double bx = (selfR > 1.0) ? selfPos.x / selfR : 0.0;
        double by = (selfR > 1.0) ? selfPos.y / selfR : 0.0;
        double bz = (selfR > 1.0) ? selfPos.z / selfR : 1.0;

        double ux = dx / dist;
        double uy = dy / dist;
        double uz = dz / dist;

        double cosTheta = bx * ux + by * uy + bz * uz;
        cosTheta = std::max(-1.0, std::min(1.0, cosTheta));
        double targetTheta = std::acos(cosTheta) * RAD2DEG;
        double targetPhi = 0.0; // simplified for elevation scan

        // Evaluate each codebook beam
        uint32_t bestBeam = 0;
        double bestGain = -1000.0;

        for (const auto& entry : codebook)
        {
            double gain = m_antennaArray->ComputeArrayGain_dBi(
                targetTheta, targetPhi,
                entry.thetaCenter_deg, entry.phiCenter_deg);
            if (gain > bestGain)
            {
                bestGain = gain;
                bestBeam = entry.index;
            }
        }

        NS_LOG_DEBUG("SelectBeamForTarget: theta=" << targetTheta
                     << " bestBeam=" << bestBeam << " gain=" << bestGain << " dBi");
        return bestBeam;
    }

    // Fallback: compute angles and use nearest-beam selection
    Vector selfPos = self->GetPosition();
    Vector targetPos = target->GetPosition();
    double dx = targetPos.x - selfPos.x;
    double dy = targetPos.y - selfPos.y;
    double dz = targetPos.z - selfPos.z;
    double dist = std::sqrt(dx * dx + dy * dy + dz * dz);

    double selfR = std::sqrt(selfPos.x * selfPos.x +
                             selfPos.y * selfPos.y +
                             selfPos.z * selfPos.z);
    double cosTheta = 0.0;
    if (dist > 1.0 && selfR > 1.0)
    {
        double ux = dx / dist;
        double uy = dy / dist;
        double uz = dz / dist;
        cosTheta = (selfPos.x * ux + selfPos.y * uy + selfPos.z * uz) / selfR;
    }
    cosTheta = std::max(-1.0, std::min(1.0, cosTheta));
    double targetTheta = std::acos(cosTheta) * RAD2DEG;

    return SelectBeam(targetTheta, 0.0, level);
}

// ---- Hierarchical search with actual gain evaluation ----

ThzBeamState
ThzNtnBeamforming::PerformHierarchicalSearch(Ptr<MobilityModel> target,
                                             Ptr<MobilityModel> self) const
{
    NS_LOG_FUNCTION(this);

    ThzBeamState result;

    static const ThzCodebookLevel levels[] = {
        ThzCodebookLevel::WIDE,
        ThzCodebookLevel::MEDIUM,
        ThzCodebookLevel::NARROW,
        ThzCodebookLevel::ULTRA_NARROW};

    for (auto lvl : levels)
    {
        uint32_t idx = static_cast<uint32_t>(lvl);
        if (m_codebook[idx].empty())
        {
            continue;
        }

        uint32_t beamIdx = SelectBeamForTarget(target, self, lvl);
        const auto& entry = m_codebook[idx][beamIdx];

        result.beamIndex     = beamIdx;
        result.codebookLevel = lvl;
        result.steerTheta_deg = entry.thetaCenter_deg;
        result.steerPhi_deg   = entry.phiCenter_deg;
        result.gain_dBi       = entry.gain_dBi;
    }

    result.isTracking = true;
    NS_LOG_INFO("HierarchicalSearch (MobilityModel): beam " << result.beamIndex
                << " theta=" << result.steerTheta_deg
                << " gain=" << result.gain_dBi << " dBi");
    return result;
}

// ---- Legacy angle-based selection ----

uint32_t
ThzNtnBeamforming::SelectBeam(double targetTheta_deg,
                              double targetPhi_deg,
                              ThzCodebookLevel level) const
{
    NS_LOG_FUNCTION(this << targetTheta_deg << targetPhi_deg << static_cast<int>(level));

    uint32_t idx = static_cast<uint32_t>(level);
    const auto& codebook = m_codebook[idx];

    if (codebook.empty())
    {
        return 0;
    }

    // If antenna array is set, evaluate actual gain per beam
    if (m_antennaArray)
    {
        uint32_t bestBeam = 0;
        double bestGain = -1000.0;

        for (const auto& entry : codebook)
        {
            double gain = m_antennaArray->ComputeArrayGain_dBi(
                targetTheta_deg, targetPhi_deg,
                entry.thetaCenter_deg, entry.phiCenter_deg);
            if (gain > bestGain)
            {
                bestGain = gain;
                bestBeam = entry.index;
            }
        }
        return bestBeam;
    }

    // Fallback: find beam with smallest angular distance
    uint32_t bestBeam = 0;
    double bestDist   = std::numeric_limits<double>::max();

    for (const auto& entry : codebook)
    {
        double dTheta = targetTheta_deg - entry.thetaCenter_deg;
        double dPhi   = targetPhi_deg - entry.phiCenter_deg;
        double dist   = dTheta * dTheta + dPhi * dPhi;
        if (dist < bestDist)
        {
            bestDist = dist;
            bestBeam = entry.index;
        }
    }

    return bestBeam;
}

ThzBeamState
ThzNtnBeamforming::PerformHierarchicalSearch(double targetTheta_deg,
                                             double targetPhi_deg) const
{
    NS_LOG_FUNCTION(this << targetTheta_deg << targetPhi_deg);

    ThzBeamState result;
    double theta = targetTheta_deg;
    double phi   = targetPhi_deg;

    static const ThzCodebookLevel levels[] = {
        ThzCodebookLevel::WIDE,
        ThzCodebookLevel::MEDIUM,
        ThzCodebookLevel::NARROW,
        ThzCodebookLevel::ULTRA_NARROW};

    for (auto lvl : levels)
    {
        uint32_t idx = static_cast<uint32_t>(lvl);
        if (m_codebook[idx].empty())
        {
            continue;
        }

        uint32_t beamIdx = SelectBeam(theta, phi, lvl);
        const auto& entry = m_codebook[idx][beamIdx];

        theta = entry.thetaCenter_deg;
        phi   = entry.phiCenter_deg;

        result.beamIndex     = beamIdx;
        result.codebookLevel = lvl;
        result.steerTheta_deg = entry.thetaCenter_deg;
        result.steerPhi_deg   = entry.phiCenter_deg;
        result.gain_dBi       = entry.gain_dBi;
    }

    result.isTracking = true;
    return result;
}

// ---- Beam squint ----

double
ThzNtnBeamforming::ComputeBeamSquintAngle_deg(double freqHz,
                                               double centerFreqHz,
                                               double steerAngle_deg) const
{
    double sinSteer  = std::sin(steerAngle_deg * DEG2RAD);
    double sinActual = (centerFreqHz / freqHz) * sinSteer;
    sinActual = std::max(-1.0, std::min(1.0, sinActual));
    return std::asin(sinActual) * RAD2DEG;
}

double
ThzNtnBeamforming::ComputeBeamSquintLoss_dB() const
{
    NS_LOG_FUNCTION(this);

    if (m_enableSquintComp)
    {
        return 0.0;
    }

    // Use internal m_bandwidthHz and m_frequency
    double steerAngle_deg = m_activeBeam.steerTheta_deg;

    double fLow  = m_frequency - m_bandwidthHz / 2.0;
    double fHigh = m_frequency + m_bandwidthHz / 2.0;

    double thetaLow  = ComputeBeamSquintAngle_deg(fLow, m_frequency, steerAngle_deg);
    double thetaHigh = ComputeBeamSquintAngle_deg(fHigh, m_frequency, steerAngle_deg);

    double maxDeviation = std::max(std::abs(thetaLow - steerAngle_deg),
                                   std::abs(thetaHigh - steerAngle_deg));

    // 3 dB beamwidth from actual array geometry
    double lambda = SPEED_OF_LIGHT / m_frequency;
    double d = lambda / 2.0;
    double Nsqrt = std::sqrt(static_cast<double>(m_numElements));
    double bw3dB_deg = 0.886 * lambda / (Nsqrt * d) * RAD2DEG;

    double ratio = maxDeviation / std::max(bw3dB_deg, 0.001);
    double loss  = 12.0 * ratio * ratio;

    NS_LOG_DEBUG("BeamSquintLoss: BW=" << m_bandwidthHz / 1e9 << " GHz, steer="
                 << steerAngle_deg << " deg, loss=" << loss << " dB");
    return std::min(loss, 30.0);
}

// ---- Mode & switching ----

void
ThzNtnBeamforming::SetBeamformingMode(ThzBeamformingMode mode)
{
    m_mode = mode;
    switch (mode)
    {
    case ThzBeamformingMode::ANALOG_ONLY: m_modeStr = "ANALOG_ONLY"; break;
    case ThzBeamformingMode::HYBRID: m_modeStr = "HYBRID"; break;
    case ThzBeamformingMode::DIGITAL_ONLY: m_modeStr = "DIGITAL_ONLY"; break;
    }
}

double
ThzNtnBeamforming::ComputeBeamSwitchingDelay_us() const
{
    switch (m_mode)
    {
    case ThzBeamformingMode::DIGITAL_ONLY:
        return 1.0;
    case ThzBeamformingMode::HYBRID:
    case ThzBeamformingMode::ANALOG_ONLY:
    default:
        return m_beamSwitchDelay_us;
    }
}

// ---- Tracking interface ----

void
ThzNtnBeamforming::UpdateBeamTracking(double newTheta_deg, double newPhi_deg)
{
    NS_LOG_FUNCTION(this << newTheta_deg << newPhi_deg);

    ThzCodebookLevel finest = ThzCodebookLevel::ULTRA_NARROW;
    uint32_t fIdx = static_cast<uint32_t>(finest);

    while (m_codebook[fIdx].empty() && fIdx > 0)
    {
        --fIdx;
    }

    auto lvl = static_cast<ThzCodebookLevel>(fIdx);
    uint32_t beamIdx = SelectBeam(newTheta_deg, newPhi_deg, lvl);
    const auto& entry = m_codebook[fIdx][beamIdx];

    m_activeBeam.beamIndex      = beamIdx;
    m_activeBeam.codebookLevel  = lvl;
    m_activeBeam.steerTheta_deg = entry.thetaCenter_deg;
    m_activeBeam.steerPhi_deg   = entry.phiCenter_deg;
    m_activeBeam.gain_dBi       = entry.gain_dBi;
    m_activeBeam.isTracking     = true;
}

double
ThzNtnBeamforming::ComputeAnalogBeamGain_dBi(double theta_deg,
                                              double phi_deg) const
{
    NS_LOG_FUNCTION(this << theta_deg << phi_deg);

    // If antenna array is available, use it for actual gain computation
    if (m_antennaArray)
    {
        return m_antennaArray->ComputeArrayGain_dBi(
            theta_deg, phi_deg,
            m_activeBeam.steerTheta_deg, m_activeBeam.steerPhi_deg);
    }

    // Fallback: compute with quantised phase shifters
    uint32_t N = m_numElements;
    double sinObs   = std::sin(theta_deg * DEG2RAD);
    double sinSteer = std::sin(m_activeBeam.steerTheta_deg * DEG2RAD);

    std::complex<double> af(0.0, 0.0);
    const std::complex<double> j(0.0, 1.0);
    double quantStep = 2.0 * PI / std::pow(2.0, static_cast<double>(m_phaseShifterBits));

    for (uint32_t n = 0; n < N; ++n)
    {
        double idealPhase = 2.0 * PI * static_cast<double>(n) * sinSteer /
                            static_cast<double>(N);
        double quantPhase = std::round(idealPhase / quantStep) * quantStep;
        double obsPhase = 2.0 * PI * static_cast<double>(n) * sinObs /
                          static_cast<double>(N);
        af += std::exp(j * (obsPhase - quantPhase));
    }

    double afNorm = std::abs(af) / static_cast<double>(N);
    double gain   = 10.0 * std::log10(static_cast<double>(N)) +
                    20.0 * std::log10(std::max(afNorm, 1e-12)) +
                    5.0; // element gain

    return gain;
}

uint32_t
ThzNtnBeamforming::GetActiveBeamIndex() const
{
    return m_activeBeam.beamIndex;
}

const ThzBeamState&
ThzNtnBeamforming::GetActiveBeamState() const
{
    return m_activeBeam;
}

} // namespace ns3
