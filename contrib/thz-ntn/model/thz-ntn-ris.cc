/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-only
 * Author: Muhammad Uzair
 *
 * THz-NTN Reconfigurable Intelligent Surface (RIS) Model
 */

#include "thz-ntn-ris.h"

#include <ns3/satellite-free-space-loss.h>

#include <ns3/double.h>
#include <ns3/log.h>
#include <ns3/string.h>
#include <ns3/uinteger.h>

#include <algorithm>
#include <cmath>
#include <numeric>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("ThzNtnRis");
NS_OBJECT_ENSURE_REGISTERED(ThzNtnRis);

TypeId
ThzNtnRis::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::ThzNtnRis")
            .SetParent<Object>()
            .SetGroupName("ThzNtn")
            .AddConstructor<ThzNtnRis>()
            .AddAttribute("NumElementsX",
                          "Number of RIS elements along X axis",
                          UintegerValue(64),
                          MakeUintegerAccessor(&ThzNtnRis::m_numX),
                          MakeUintegerChecker<uint32_t>(1, 256))
            .AddAttribute("NumElementsY",
                          "Number of RIS elements along Y axis",
                          UintegerValue(64),
                          MakeUintegerAccessor(&ThzNtnRis::m_numY),
                          MakeUintegerChecker<uint32_t>(1, 256))
            .AddAttribute("Frequency",
                          "Operating frequency in Hz",
                          DoubleValue(300.0e9),
                          MakeDoubleAccessor(&ThzNtnRis::m_frequency),
                          MakeDoubleChecker<double>(100.0e9, 10.0e12))
            .AddAttribute("Deployment",
                          "Deployment type: SPACE_BORNE, AERIAL, GROUND",
                          StringValue("GROUND"),
                          MakeStringAccessor(&ThzNtnRis::m_deploymentStr),
                          MakeStringChecker())
            .AddAttribute("PhaseModel",
                          "Phase model: CONTINUOUS, DISCRETE_1BIT, DISCRETE_2BIT, "
                          "DISCRETE_3BIT, DISCRETE_4BIT",
                          StringValue("DISCRETE_2BIT"),
                          MakeStringAccessor(&ThzNtnRis::m_phaseModelStr),
                          MakeStringChecker())
            .AddAttribute("ReflectionEfficiency",
                          "Reflection efficiency of each element (0-1)",
                          DoubleValue(0.8),
                          MakeDoubleAccessor(&ThzNtnRis::m_reflectionEfficiency),
                          MakeDoubleChecker<double>(0.0, 1.0));
    return tid;
}

ThzNtnRis::ThzNtnRis()
    : m_numX(64),
      m_numY(64),
      m_frequency(300.0e9),
      m_deployment(RisDeployment::GROUND),
      m_phaseModel(PhaseModel::DISCRETE_2BIT),
      m_reflectionEfficiency(0.8),
      m_deploymentStr("GROUND"),
      m_phaseModelStr("DISCRETE_2BIT"),
      m_elementGain_dBi(5.0),
      m_fsl(nullptr)
{
    NS_LOG_FUNCTION(this);
    m_phases_rad.resize(m_numX * m_numY, 0.0);
}

ThzNtnRis::~ThzNtnRis()
{
    NS_LOG_FUNCTION(this);
}

void
ThzNtnRis::DoDispose()
{
    NS_LOG_FUNCTION(this);
    m_phases_rad.clear();
    m_fsl = nullptr;
    Object::DoDispose();
}

// ---------------------------------------------------------------------------
// Sub-model setters
// ---------------------------------------------------------------------------

void
ThzNtnRis::SetFreeSpaceLossModel(Ptr<SatFreeSpaceLoss> fsl)
{
    NS_LOG_FUNCTION(this << fsl);
    m_fsl = fsl;
}

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

void
ThzNtnRis::Configure(uint32_t numX, uint32_t numY, double freqHz, RisDeployment deploy)
{
    NS_LOG_FUNCTION(this << numX << numY << freqHz << static_cast<int>(deploy));

    m_numX = numX;
    m_numY = numY;
    m_frequency = freqHz;
    m_deployment = deploy;

    // Parse string attributes
    m_deployment = ParseDeployment(m_deploymentStr);
    if (deploy != m_deployment)
    {
        m_deployment = deploy;
    }
    m_phaseModel = ParsePhaseModel(m_phaseModelStr);

    // Re-initialize phase profile
    m_phases_rad.resize(m_numX * m_numY, 0.0);
    std::fill(m_phases_rad.begin(), m_phases_rad.end(), 0.0);

    // Element gain: directivity of lambda/2 patch (approx 5 dBi)
    m_elementGain_dBi = 5.0;

    NS_LOG_INFO("RIS configured: " << m_numX << "x" << m_numY << " = "
                                   << GetTotalElements() << " elements at "
                                   << m_frequency / 1.0e9 << " GHz, lambda="
                                   << GetWavelength() * 1.0e3 << " mm");
}

// ---------------------------------------------------------------------------
// Angle computation from 3D positions
// ---------------------------------------------------------------------------

void
ThzNtnRis::ComputeAnglesFromPositions(const Vector& risPos,
                                      const Vector& targetPos,
                                      double& theta_deg,
                                      double& phi_deg) const
{
    double dx = targetPos.x - risPos.x;
    double dy = targetPos.y - risPos.y;
    double dz = targetPos.z - risPos.z;
    double dist = std::sqrt(dx * dx + dy * dy + dz * dz);

    if (dist < 1.0)
    {
        theta_deg = 0.0;
        phi_deg = 0.0;
        return;
    }

    // Theta: angle from boresight (z-axis) = acos(dz/dist)
    theta_deg = std::acos(dz / dist) / DEG2RAD;
    // Phi: azimuth in x-y plane = atan2(dy, dx)
    phi_deg = std::atan2(dy, dx) / DEG2RAD;
}

// ---------------------------------------------------------------------------
// MobilityModel-based RIS gain
// ---------------------------------------------------------------------------

double
ThzNtnRis::ComputeRisGainForLink(Ptr<MobilityModel> tx,
                                 Ptr<MobilityModel> ris,
                                 Ptr<MobilityModel> rx) const
{
    NS_LOG_FUNCTION(this);
    NS_ASSERT_MSG(tx && ris && rx, "MobilityModel pointers must not be null");

    Vector risPos = ris->GetPosition();
    Vector txPos = tx->GetPosition();
    Vector rxPos = rx->GetPosition();

    double thetaIn_deg, phiIn_deg, thetaOut_deg, phiOut_deg;
    ComputeAnglesFromPositions(risPos, txPos, thetaIn_deg, phiIn_deg);
    ComputeAnglesFromPositions(risPos, rxPos, thetaOut_deg, phiOut_deg);

    return ComputeRisGain_dB(thetaIn_deg, phiIn_deg, thetaOut_deg, phiOut_deg);
}

// ---------------------------------------------------------------------------
// Cascaded path loss using SatFreeSpaceLoss
// ---------------------------------------------------------------------------

double
ThzNtnRis::ComputeCascadedPathLoss_dB(Ptr<MobilityModel> tx,
                                      Ptr<MobilityModel> ris,
                                      Ptr<MobilityModel> rx) const
{
    NS_LOG_FUNCTION(this);
    NS_ASSERT_MSG(tx && ris && rx, "MobilityModel pointers must not be null");

    double fspl1_dB;
    double fspl2_dB;

    if (m_fsl)
    {
        // Use actual SatFreeSpaceLoss for each segment
        fspl1_dB = m_fsl->GetFsldB(tx, ris, m_frequency);
        fspl2_dB = m_fsl->GetFsldB(ris, rx, m_frequency);
    }
    else
    {
        // Fallback: compute FSPL from positions directly
        double d1_m = tx->GetDistanceFrom(ris);
        double d2_m = ris->GetDistanceFrom(rx);
        double lambda = GetWavelength();

        if (d1_m <= 0.0 || d2_m <= 0.0)
        {
            NS_LOG_WARN("Invalid distances for cascaded path loss");
            return 0.0;
        }

        fspl1_dB = 20.0 * std::log10(4.0 * PI_VAL * d1_m / lambda);
        fspl2_dB = 20.0 * std::log10(4.0 * PI_VAL * d2_m / lambda);
    }

    // Total cascaded path loss (without RIS gain compensation)
    double cascadedPL = fspl1_dB + fspl2_dB;

    NS_LOG_DEBUG("Cascaded PL: fspl1=" << fspl1_dB << " dB, fspl2="
                                       << fspl2_dB << " dB, total="
                                       << cascadedPL << " dB");
    return cascadedPL;
}

// ---------------------------------------------------------------------------
// Optimal phases from actual 3D geometry
// ---------------------------------------------------------------------------

void
ThzNtnRis::ComputeOptimalPhases(Ptr<MobilityModel> tx,
                                Ptr<MobilityModel> ris,
                                Ptr<MobilityModel> rx)
{
    NS_LOG_FUNCTION(this);
    NS_ASSERT_MSG(tx && ris && rx, "MobilityModel pointers must not be null");

    Vector risPos = ris->GetPosition();
    Vector txPos = tx->GetPosition();
    Vector rxPos = rx->GetPosition();

    double thetaIn_deg, phiIn_deg, thetaOut_deg, phiOut_deg;
    ComputeAnglesFromPositions(risPos, txPos, thetaIn_deg, phiIn_deg);
    ComputeAnglesFromPositions(risPos, rxPos, thetaOut_deg, phiOut_deg);

    // Delegate to angle-based method
    ComputeOptimalPhases(thetaIn_deg, phiIn_deg, thetaOut_deg, phiOut_deg);
}

// ---------------------------------------------------------------------------
// Angle-based computations
// ---------------------------------------------------------------------------

double
ThzNtnRis::ComputeRisGain_dB(double thetaIn_deg,
                              double phiIn_deg,
                              double thetaOut_deg,
                              double phiOut_deg) const
{
    NS_LOG_FUNCTION(this << thetaIn_deg << phiIn_deg << thetaOut_deg << phiOut_deg);

    const uint32_t N = GetTotalElements();
    if (N == 0)
    {
        return 0.0;
    }

    const double lambda = GetWavelength();
    const double d = GetElementSpacing_m();
    const double k = 2.0 * PI_VAL / lambda;

    // Convert angles to radians
    double thetaIn = thetaIn_deg * DEG2RAD;
    double phiIn = phiIn_deg * DEG2RAD;
    double thetaOut = thetaOut_deg * DEG2RAD;
    double phiOut = phiOut_deg * DEG2RAD;

    // Direction cosines for incoming wave
    double uxIn = std::sin(thetaIn) * std::cos(phiIn);
    double uyIn = std::sin(thetaIn) * std::sin(phiIn);

    // Direction cosines for outgoing wave
    double uxOut = std::sin(thetaOut) * std::cos(phiOut);
    double uyOut = std::sin(thetaOut) * std::sin(phiOut);

    // Compute array factor with current phase profile
    double realSum = 0.0;
    double imagSum = 0.0;

    for (uint32_t ix = 0; ix < m_numX; ++ix)
    {
        for (uint32_t iy = 0; iy < m_numY; ++iy)
        {
            uint32_t idx = ix * m_numY + iy;
            double posX = ix * d;
            double posY = iy * d;

            // Phase from incoming wave to element
            double phaseIn = k * (uxIn * posX + uyIn * posY);
            // Phase from element to outgoing direction
            double phaseOut = k * (uxOut * posX + uyOut * posY);
            // Element applied phase shift
            double phaseShift = (idx < m_phases_rad.size()) ? m_phases_rad[idx] : 0.0;

            // Quantize phase if discrete model
            double effectivePhase = phaseShift;
            if (m_phaseModel != PhaseModel::CONTINUOUS)
            {
                effectivePhase = QuantizePhase(phaseShift);
            }

            // Total phase: incoming + element shift - outgoing
            double totalPhase = phaseIn + effectivePhase - phaseOut;

            realSum += std::cos(totalPhase);
            imagSum += std::sin(totalPhase);
        }
    }

    // Array factor magnitude squared
    double afMagSq = realSum * realSum + imagSum * imagSum;

    // Include reflection efficiency
    double efficiencyLinear = m_reflectionEfficiency * m_reflectionEfficiency;

    // Gain in dB: AF gain + element gain
    double afGain_dB = 10.0 * std::log10(afMagSq * efficiencyLinear / static_cast<double>(N));
    double totalGain = afGain_dB + m_elementGain_dBi;

    NS_LOG_DEBUG("RIS gain: AF=" << afGain_dB << " dB, element=" << m_elementGain_dBi
                                 << " dBi, total=" << totalGain << " dB");
    return totalGain;
}

double
ThzNtnRis::ComputeMaxGain_dB() const
{
    NS_LOG_FUNCTION(this);

    const uint32_t N = GetTotalElements();
    if (N == 0)
    {
        return 0.0;
    }

    double maxGain = 20.0 * std::log10(static_cast<double>(N)) + m_elementGain_dBi;
    maxGain += 10.0 * std::log10(m_reflectionEfficiency);

    return maxGain;
}

void
ThzNtnRis::SetPhaseProfile(const std::vector<double>& phases_rad)
{
    NS_LOG_FUNCTION(this << phases_rad.size());

    const uint32_t N = GetTotalElements();
    if (phases_rad.size() != N)
    {
        NS_LOG_WARN("Phase profile size " << phases_rad.size()
                                          << " does not match element count " << N);
        return;
    }

    m_phases_rad = phases_rad;
}

void
ThzNtnRis::ComputeOptimalPhases(double thetaIn_deg,
                                double phiIn_deg,
                                double thetaOut_deg,
                                double phiOut_deg)
{
    NS_LOG_FUNCTION(this << thetaIn_deg << phiIn_deg << thetaOut_deg << phiOut_deg);

    // All derived from actual wavelength
    const double lambda = GetWavelength();
    const double d = GetElementSpacing_m();
    const double k = 2.0 * PI_VAL / lambda;

    double thetaIn = thetaIn_deg * DEG2RAD;
    double phiIn = phiIn_deg * DEG2RAD;
    double thetaOut = thetaOut_deg * DEG2RAD;
    double phiOut = phiOut_deg * DEG2RAD;

    double uxIn = std::sin(thetaIn) * std::cos(phiIn);
    double uyIn = std::sin(thetaIn) * std::sin(phiIn);
    double uxOut = std::sin(thetaOut) * std::cos(phiOut);
    double uyOut = std::sin(thetaOut) * std::sin(phiOut);

    m_phases_rad.resize(m_numX * m_numY);

    for (uint32_t ix = 0; ix < m_numX; ++ix)
    {
        for (uint32_t iy = 0; iy < m_numY; ++iy)
        {
            uint32_t idx = ix * m_numY + iy;
            double posX = ix * d;
            double posY = iy * d;

            double optPhase = k * ((uxOut - uxIn) * posX + (uyOut - uyIn) * posY);

            // Wrap to [-pi, pi]
            optPhase = std::fmod(optPhase, 2.0 * PI_VAL);
            if (optPhase > PI_VAL)
            {
                optPhase -= 2.0 * PI_VAL;
            }
            if (optPhase < -PI_VAL)
            {
                optPhase += 2.0 * PI_VAL;
            }

            // Quantize if discrete
            if (m_phaseModel != PhaseModel::CONTINUOUS)
            {
                optPhase = QuantizePhase(optPhase);
            }

            m_phases_rad[idx] = optPhase;
        }
    }

    NS_LOG_INFO("Optimal phases computed for in=(" << thetaIn_deg << "," << phiIn_deg
                                                   << ") out=(" << thetaOut_deg << ","
                                                   << phiOut_deg << "), lambda="
                                                   << lambda * 1.0e3 << " mm");
}

double
ThzNtnRis::ComputeQuantizationLoss_dB() const
{
    NS_LOG_FUNCTION(this);

    if (m_phaseModel == PhaseModel::CONTINUOUS)
    {
        return 0.0;
    }

    uint32_t levels = GetQuantizationLevels();
    if (levels == 0)
    {
        return 0.0;
    }

    double piOverL = PI_VAL / static_cast<double>(levels);
    double sincVal = std::sin(piOverL) / piOverL;
    double loss_dB = -10.0 * std::log10(sincVal * sincVal);

    NS_LOG_DEBUG("Quantization loss: " << levels << " levels -> " << loss_dB << " dB");
    return loss_dB;
}

double
ThzNtnRis::ComputeSnrGain_dB(bool perfectCsi) const
{
    NS_LOG_FUNCTION(this << perfectCsi);

    const uint32_t N = GetTotalElements();
    if (N == 0)
    {
        return 0.0;
    }

    double nDouble = static_cast<double>(N);
    double gain_dB;

    if (perfectCsi)
    {
        gain_dB = 20.0 * std::log10(nDouble);
    }
    else
    {
        gain_dB = 10.0 * std::log10(nDouble);
    }

    gain_dB -= ComputeQuantizationLoss_dB();
    gain_dB += 10.0 * std::log10(m_reflectionEfficiency);

    NS_LOG_INFO("SNR gain: " << gain_dB << " dB (N=" << N
                              << ", perfectCSI=" << perfectCsi << ")");
    return gain_dB;
}

bool
ThzNtnRis::IsInNearField(double distance_m) const
{
    NS_LOG_FUNCTION(this << distance_m);

    double D = GetPanelSize_m();
    double lambda = GetWavelength();
    double fraunhoferDist = 2.0 * D * D / lambda;

    bool nearField = (distance_m < fraunhoferDist);

    NS_LOG_DEBUG("Near-field check: D=" << D << " m, lambda=" << lambda
                                        << " m, Fraunhofer=" << fraunhoferDist
                                        << " m, distance=" << distance_m
                                        << " -> " << (nearField ? "NEAR" : "FAR"));
    return nearField;
}

double
ThzNtnRis::ComputeAerialRisCoverage_deg() const
{
    NS_LOG_FUNCTION(this);

    switch (m_deployment)
    {
    case RisDeployment::AERIAL:
        return 360.0;
    case RisDeployment::SPACE_BORNE:
        return 180.0;
    case RisDeployment::GROUND:
    default:
        return 180.0;
    }
}

uint32_t
ThzNtnRis::GetTotalElements() const
{
    return m_numX * m_numY;
}

double
ThzNtnRis::GetWavelength() const
{
    return C_LIGHT / m_frequency;
}

void
ThzNtnRis::GetPanelDimensions(uint32_t& numX, uint32_t& numY) const
{
    numX = m_numX;
    numY = m_numY;
}

// ---------------------------------------------------------------------------
// Private helpers
// ---------------------------------------------------------------------------

double
ThzNtnRis::GetElementSpacing_m() const
{
    // Half-wavelength spacing computed from actual frequency
    return GetWavelength() / 2.0;
}

double
ThzNtnRis::GetPanelSize_m() const
{
    double d = GetElementSpacing_m();
    double sizeX = m_numX * d;
    double sizeY = m_numY * d;
    return std::max(sizeX, sizeY);
}

uint32_t
ThzNtnRis::GetQuantizationLevels() const
{
    switch (m_phaseModel)
    {
    case PhaseModel::CONTINUOUS:
        return 0;
    case PhaseModel::DISCRETE_1BIT:
        return 2;
    case PhaseModel::DISCRETE_2BIT:
        return 4;
    case PhaseModel::DISCRETE_3BIT:
        return 8;
    case PhaseModel::DISCRETE_4BIT:
        return 16;
    default:
        return 0;
    }
}

double
ThzNtnRis::QuantizePhase(double phase_rad) const
{
    uint32_t levels = GetQuantizationLevels();
    if (levels == 0)
    {
        return phase_rad;
    }

    double step = 2.0 * PI_VAL / static_cast<double>(levels);

    double p = std::fmod(phase_rad, 2.0 * PI_VAL);
    if (p < 0.0)
    {
        p += 2.0 * PI_VAL;
    }

    double idx = std::round(p / step);
    double quantized = idx * step;

    if (quantized > PI_VAL)
    {
        quantized -= 2.0 * PI_VAL;
    }
    return quantized;
}

RisDeployment
ThzNtnRis::ParseDeployment(const std::string& s)
{
    if (s == "SPACE_BORNE")
    {
        return RisDeployment::SPACE_BORNE;
    }
    if (s == "AERIAL")
    {
        return RisDeployment::AERIAL;
    }
    return RisDeployment::GROUND;
}

PhaseModel
ThzNtnRis::ParsePhaseModel(const std::string& s)
{
    if (s == "CONTINUOUS")
    {
        return PhaseModel::CONTINUOUS;
    }
    if (s == "DISCRETE_1BIT")
    {
        return PhaseModel::DISCRETE_1BIT;
    }
    if (s == "DISCRETE_3BIT")
    {
        return PhaseModel::DISCRETE_3BIT;
    }
    if (s == "DISCRETE_4BIT")
    {
        return PhaseModel::DISCRETE_4BIT;
    }
    return PhaseModel::DISCRETE_2BIT;
}

} // namespace ns3
