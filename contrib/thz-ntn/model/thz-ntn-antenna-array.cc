/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-only
 * Author: Muhammad Uzair
 *
 * Ultra-Massive MIMO Antenna Array — implementation
 */

#include "thz-ntn-antenna-array.h"

// Satellite module header for gain pattern delegation
#include <ns3/satellite-antenna-gain-pattern.h>
#include <ns3/satellite-mobility-model.h>

#include <ns3/double.h>
#include <ns3/log.h>
#include <ns3/string.h>
#include <ns3/uinteger.h>

#include <algorithm>
#include <cmath>
#include <complex>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("ThzNtnAntennaArray");
NS_OBJECT_ENSURE_REGISTERED(ThzNtnAntennaArray);

// Physical constants
static constexpr double SPEED_OF_LIGHT = 299792458.0;
static constexpr double PI             = 3.14159265358979323846;
static constexpr double DEG2RAD        = PI / 180.0;
static constexpr double RAD2DEG        = 180.0 / PI;
static constexpr double EARTH_RADIUS_M = 6371000.0;

// ---- TypeId ----

TypeId
ThzNtnAntennaArray::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::ThzNtnAntennaArray")
            .SetParent<Object>()
            .SetGroupName("ThzNtn")
            .AddConstructor<ThzNtnAntennaArray>()
            .AddAttribute("NumElementsX",
                          "Number of array elements along the x-axis (columns).",
                          UintegerValue(32),
                          MakeUintegerAccessor(&ThzNtnAntennaArray::SetNumElementsX,
                                              &ThzNtnAntennaArray::GetNumElementsX),
                          MakeUintegerChecker<uint32_t>(1, 4096))
            .AddAttribute("NumElementsY",
                          "Number of array elements along the y-axis (rows).",
                          UintegerValue(32),
                          MakeUintegerAccessor(&ThzNtnAntennaArray::SetNumElementsY,
                                              &ThzNtnAntennaArray::GetNumElementsY),
                          MakeUintegerChecker<uint32_t>(1, 4096))
            .AddAttribute("Frequency",
                          "Operating frequency in Hz.",
                          DoubleValue(300e9),
                          MakeDoubleAccessor(&ThzNtnAntennaArray::SetFreqHz,
                                            &ThzNtnAntennaArray::GetFreqHz),
                          MakeDoubleChecker<double>(1e9, 10e12))
            .AddAttribute("ArrayType",
                          "Array geometry: UPA, UCA, or CASSEGRAIN.",
                          StringValue("UPA"),
                          MakeStringAccessor(&ThzNtnAntennaArray::m_arrayTypeStr),
                          MakeStringChecker())
            .AddAttribute("ElementPattern",
                          "Per-element pattern: ISOTROPIC, PATCH, or COSINE.",
                          StringValue("COSINE"),
                          MakeStringAccessor(&ThzNtnAntennaArray::m_elementPatternStr),
                          MakeStringChecker())
            .AddAttribute("ElementGain",
                          "Peak per-element gain in dBi.",
                          DoubleValue(5.0),
                          MakeDoubleAccessor(&ThzNtnAntennaArray::SetMaxElementGain,
                                            &ThzNtnAntennaArray::GetMaxElementGain),
                          MakeDoubleChecker<double>(-30.0, 30.0))
            .AddAttribute("ApertureDiameter",
                          "Cassegrain reflector aperture diameter in metres.",
                          DoubleValue(0.45),
                          MakeDoubleAccessor(&ThzNtnAntennaArray::m_apertureDiam),
                          MakeDoubleChecker<double>(0.01, 10.0));
    return tid;
}

// ---- Constructor / destructor ----

ThzNtnAntennaArray::ThzNtnAntennaArray()
    : m_lambda(0.0),
      m_k(0.0),
      m_cosExponent(1.5),
      m_apertureDiam(0.45),
      m_satGainPattern(nullptr),
      m_arrayTypeStr("UPA"),
      m_elementPatternStr("COSINE")
{
    NS_LOG_FUNCTION(this);
    RecalculateDerived();
}

ThzNtnAntennaArray::~ThzNtnAntennaArray()
{
    NS_LOG_FUNCTION(this);
}

// ---- Configuration ----

void
ThzNtnAntennaArray::Configure(uint32_t numX,
                              uint32_t numY,
                              double freqHz,
                              ThzArrayType type)
{
    NS_LOG_FUNCTION(this << numX << numY << freqHz << static_cast<int>(type));
    m_cfg.numElementsX = numX;
    m_cfg.numElementsY = numY;
    m_cfg.freqHz       = freqHz;
    m_cfg.arrayType    = type;

    switch (type)
    {
    case ThzArrayType::UPA: m_arrayTypeStr = "UPA"; break;
    case ThzArrayType::UCA: m_arrayTypeStr = "UCA"; break;
    case ThzArrayType::CASSEGRAIN: m_arrayTypeStr = "CASSEGRAIN"; break;
    }

    RecalculateDerived();
}

void
ThzNtnAntennaArray::RecalculateDerived()
{
    // Parse string enums
    if (m_arrayTypeStr == "UCA")
    {
        m_cfg.arrayType = ThzArrayType::UCA;
    }
    else if (m_arrayTypeStr == "CASSEGRAIN")
    {
        m_cfg.arrayType = ThzArrayType::CASSEGRAIN;
    }
    else
    {
        m_cfg.arrayType = ThzArrayType::UPA;
    }

    if (m_elementPatternStr == "ISOTROPIC")
    {
        m_cfg.elementPattern = ThzElementPattern::ISOTROPIC;
    }
    else if (m_elementPatternStr == "PATCH")
    {
        m_cfg.elementPattern = ThzElementPattern::PATCH;
    }
    else
    {
        m_cfg.elementPattern = ThzElementPattern::COSINE;
    }

    // Wavelength and wavenumber from actual frequency
    m_lambda = SPEED_OF_LIGHT / m_cfg.freqHz;
    m_k      = 2.0 * PI / m_lambda;

    // Element spacing = lambda / 2 (from actual wavelength)
    m_cfg.elementSpacing_m = m_lambda / 2.0;
    m_cfg.totalElements    = m_cfg.numElementsX * m_cfg.numElementsY;

    // Cosine exponent q: chosen so that the element 3 dB beamwidth is ~65 deg
    // cos^q(32.5 deg) = 0.5  =>  q = -ln(2)/ln(cos(32.5 deg))
    m_cosExponent = -std::log(2.0) / std::log(std::cos(32.5 * DEG2RAD));

    NS_LOG_INFO("ThzNtnAntennaArray configured: "
                << m_cfg.numElementsX << "x" << m_cfg.numElementsY << " @ "
                << m_cfg.freqHz / 1e9 << " GHz, lambda=" << m_lambda * 1e3
                << " mm, spacing=" << m_cfg.elementSpacing_m * 1e3 << " mm");
}

double
ThzNtnAntennaArray::GetWavelength() const
{
    return SPEED_OF_LIGHT / m_cfg.freqHz;
}

// ---- SatAntennaGainPattern integration ----

void
ThzNtnAntennaArray::SetSatAntennaGainPattern(Ptr<SatAntennaGainPattern> gainPattern)
{
    NS_LOG_FUNCTION(this);
    m_satGainPattern = gainPattern;
}

double
ThzNtnAntennaArray::GetGainFromPosition(Ptr<MobilityModel> satellite,
                                        Ptr<MobilityModel> target) const
{
    NS_LOG_FUNCTION(this);

    if (m_satGainPattern)
    {
        // Delegate to SatAntennaGainPattern using the satellite's GeoCoordinate
        // We need a SatMobilityModel to get GeoPosition
        Ptr<SatMobilityModel> satMob = DynamicCast<SatMobilityModel>(satellite);
        if (satMob)
        {
            Vector tgtPos = target->GetPosition();
            // Convert target position to GeoCoordinate
            double tgtR = std::sqrt(tgtPos.x * tgtPos.x +
                                    tgtPos.y * tgtPos.y +
                                    tgtPos.z * tgtPos.z);
            // Approximate lat/lon from Cartesian (ECEF)
            double lat = std::asin(tgtPos.z / std::max(tgtR, 1.0)) * RAD2DEG;
            double lon = std::atan2(tgtPos.y, tgtPos.x) * RAD2DEG;

            GeoCoordinate targetGeo(lat, lon, std::max(0.0, tgtR - EARTH_RADIUS_M));
            double gainLin = m_satGainPattern->GetAntennaGain_lin(targetGeo, satMob);

            if (!std::isnan(gainLin) && gainLin > 0.0)
            {
                double gain_dBi = 10.0 * std::log10(gainLin);
                NS_LOG_DEBUG("SatAntennaGainPattern gain: " << gain_dBi << " dBi");
                return gain_dBi;
            }
        }
        NS_LOG_DEBUG("SatAntennaGainPattern delegation failed, using own array computation");
    }

    // Fall back to own array factor computation
    return ComputeArrayGainToward(target, satellite);
}

// ---- MobilityModel-based array gain ----

void
ThzNtnAntennaArray::ComputeAnglesFromPositions(const Vector& selfPos,
                                                const Vector& targetPos,
                                                double& thetaDeg,
                                                double& phiDeg) const
{
    // Compute direction vector from self to target
    double dx = targetPos.x - selfPos.x;
    double dy = targetPos.y - selfPos.y;
    double dz = targetPos.z - selfPos.z;
    double dist = std::sqrt(dx * dx + dy * dy + dz * dz);

    if (dist < 1.0)
    {
        thetaDeg = 0.0;
        phiDeg = 0.0;
        return;
    }

    // In the antenna's local frame, assume boresight is along the
    // radial direction (from Earth centre through the antenna position).
    // Theta is the angle from boresight (nadir for satellite).
    double selfR = std::sqrt(selfPos.x * selfPos.x +
                             selfPos.y * selfPos.y +
                             selfPos.z * selfPos.z);

    if (selfR < 1.0)
    {
        // Degenerate case: at origin
        thetaDeg = std::acos(dz / dist) * RAD2DEG;
        phiDeg = std::atan2(dy, dx) * RAD2DEG;
        return;
    }

    // Boresight unit vector (pointing from Earth centre through self = nadir direction)
    double bx = selfPos.x / selfR;
    double by = selfPos.y / selfR;
    double bz = selfPos.z / selfR;

    // Direction to target unit vector
    double ux = dx / dist;
    double uy = dy / dist;
    double uz = dz / dist;

    // Theta from boresight
    double cosTheta = bx * ux + by * uy + bz * uz;
    cosTheta = std::max(-1.0, std::min(1.0, cosTheta));
    thetaDeg = std::acos(cosTheta) * RAD2DEG;

    // Phi: project target direction onto plane perpendicular to boresight
    // Use an arbitrary reference direction in the perpendicular plane
    // Choose east direction as reference
    double eastX = -selfPos.y / std::max(std::sqrt(selfPos.x * selfPos.x +
                                                    selfPos.y * selfPos.y), 1.0);
    double eastY = selfPos.x / std::max(std::sqrt(selfPos.x * selfPos.x +
                                                    selfPos.y * selfPos.y), 1.0);
    double eastZ = 0.0;

    // North = boresight x east
    double northX = by * eastZ - bz * eastY;
    double northY = bz * eastX - bx * eastZ;
    double northZ = bx * eastY - by * eastX;

    // Project direction onto east/north
    double projEast = ux * eastX + uy * eastY + uz * eastZ;
    double projNorth = ux * northX + uy * northY + uz * northZ;
    phiDeg = std::atan2(projEast, projNorth) * RAD2DEG;
}

double
ThzNtnAntennaArray::ComputeArrayGainToward(Ptr<MobilityModel> target,
                                           Ptr<MobilityModel> self) const
{
    NS_LOG_FUNCTION(this);

    if (!target || !self)
    {
        NS_LOG_WARN("Null MobilityModel, returning max gain");
        return ComputeMaxGain_dBi();
    }

    Vector selfPos = self->GetPosition();
    Vector targetPos = target->GetPosition();

    double thetaDeg = 0.0;
    double phiDeg = 0.0;
    ComputeAnglesFromPositions(selfPos, targetPos, thetaDeg, phiDeg);

    // Steer toward the target (steering = observation for max gain)
    double gain = ComputeArrayGain_dBi(thetaDeg, phiDeg, thetaDeg, phiDeg);

    // If there is pointing error, the gain would be computed with
    // different steer angles. For perfect tracking, steer = obs.
    NS_LOG_DEBUG("ArrayGainToward: theta=" << thetaDeg << " phi=" << phiDeg
                 << " gain=" << gain << " dBi");
    return gain;
}

// ---- Element pattern ----

double
ThzNtnAntennaArray::ComputeElementPattern_dBi(double thetaDeg) const
{
    double thetaRad = std::abs(thetaDeg) * DEG2RAD;

    switch (m_cfg.elementPattern)
    {
    case ThzElementPattern::ISOTROPIC:
        return 0.0;

    case ThzElementPattern::PATCH: {
        if (std::abs(thetaDeg) >= 90.0)
        {
            return -100.0;
        }
        double gain_lin = std::cos(thetaRad);
        return m_cfg.maxElementGain_dBi + 10.0 * std::log10(std::max(gain_lin, 1e-12));
    }

    case ThzElementPattern::COSINE:
    default: {
        if (std::abs(thetaDeg) >= 90.0)
        {
            return -100.0;
        }
        double gain_lin = std::pow(std::cos(thetaRad), m_cosExponent);
        return m_cfg.maxElementGain_dBi + 10.0 * std::log10(std::max(gain_lin, 1e-12));
    }
    }
}

// ---- Array factor — UPA ----

double
ThzNtnAntennaArray::ComputeUpaArrayFactor(double thetaRad,
                                          double phiRad,
                                          double steerThetaRad,
                                          double steerPhiRad) const
{
    const double dx = m_cfg.elementSpacing_m;
    const double dy = m_cfg.elementSpacing_m;
    const uint32_t Mx = m_cfg.numElementsX;
    const uint32_t My = m_cfg.numElementsY;

    const double psiX = m_k * dx *
                        (std::sin(thetaRad) * std::cos(phiRad) -
                         std::sin(steerThetaRad) * std::cos(steerPhiRad));
    const double psiY = m_k * dy *
                        (std::sin(thetaRad) * std::sin(phiRad) -
                         std::sin(steerThetaRad) * std::sin(steerPhiRad));

    auto sinc_ratio = [](double psi, uint32_t N) -> double {
        double halfPsi = psi / 2.0;
        double denom   = std::sin(halfPsi);
        if (std::abs(denom) < 1e-15)
        {
            return 1.0;
        }
        return std::abs(std::sin(static_cast<double>(N) * halfPsi) /
                        (static_cast<double>(N) * denom));
    };

    return sinc_ratio(psiX, Mx) * sinc_ratio(psiY, My);
}

// ---- Array factor — UCA ----

double
ThzNtnAntennaArray::ComputeUcaArrayFactor(double thetaRad,
                                          double phiRad,
                                          double steerThetaRad,
                                          double steerPhiRad) const
{
    const uint32_t N = m_cfg.totalElements;
    const double a   = static_cast<double>(N) * m_cfg.elementSpacing_m / (2.0 * PI);

    std::complex<double> af(0.0, 0.0);
    const std::complex<double> j(0.0, 1.0);

    for (uint32_t n = 0; n < N; ++n)
    {
        double phi_n = 2.0 * PI * static_cast<double>(n) / static_cast<double>(N);
        double phase = m_k * a *
                       (std::cos(phiRad - phi_n) * std::sin(thetaRad) -
                        std::cos(steerPhiRad - phi_n) * std::sin(steerThetaRad));
        af += std::exp(j * phase);
    }

    return std::abs(af) / static_cast<double>(N);
}

// ---- Public: array factor ----

double
ThzNtnAntennaArray::ComputeArrayFactor(double thetaDeg,
                                       double phiDeg,
                                       double steerThetaDeg,
                                       double steerPhiDeg) const
{
    NS_LOG_FUNCTION(this << thetaDeg << phiDeg << steerThetaDeg << steerPhiDeg);

    const double thetaRad      = thetaDeg * DEG2RAD;
    const double phiRad        = phiDeg * DEG2RAD;
    const double steerThetaRad = steerThetaDeg * DEG2RAD;
    const double steerPhiRad   = steerPhiDeg * DEG2RAD;

    switch (m_cfg.arrayType)
    {
    case ThzArrayType::UCA:
        return ComputeUcaArrayFactor(thetaRad, phiRad, steerThetaRad, steerPhiRad);
    case ThzArrayType::CASSEGRAIN:
        return 1.0;
    case ThzArrayType::UPA:
    default:
        return ComputeUpaArrayFactor(thetaRad, phiRad, steerThetaRad, steerPhiRad);
    }
}

// ---- Public: total array gain ----

double
ThzNtnAntennaArray::ComputeArrayGain_dBi(double thetaDeg,
                                         double phiDeg,
                                         double steerThetaDeg,
                                         double steerPhiDeg) const
{
    NS_LOG_FUNCTION(this << thetaDeg << phiDeg << steerThetaDeg << steerPhiDeg);

    if (m_cfg.arrayType == ThzArrayType::CASSEGRAIN)
    {
        return ComputeCassegrainGain_dBi(thetaDeg, m_apertureDiam, m_cfg.freqHz);
    }

    double af         = ComputeArrayFactor(thetaDeg, phiDeg, steerThetaDeg, steerPhiDeg);
    double af_dB      = 20.0 * std::log10(std::max(af, 1e-12));
    double arrayDir   = 10.0 * std::log10(static_cast<double>(m_cfg.totalElements));
    double elemGain   = ComputeElementPattern_dBi(thetaDeg);
    double coupling   = ComputeMutualCouplingLoss_dB();

    double gain = elemGain + arrayDir + af_dB - coupling;

    NS_LOG_DEBUG("ArrayGain: elem=" << elemGain << " dir=" << arrayDir
                 << " AF=" << af_dB << " coupling=-" << coupling
                 << " total=" << gain << " dBi");
    return gain;
}

// ---- Public: max gain ----

double
ThzNtnAntennaArray::ComputeMaxGain_dBi() const
{
    if (m_cfg.arrayType == ThzArrayType::CASSEGRAIN)
    {
        return ComputeCassegrainGain_dBi(0.0, m_apertureDiam, m_cfg.freqHz);
    }

    double arrayDir = 10.0 * std::log10(static_cast<double>(m_cfg.totalElements));
    return arrayDir + m_cfg.maxElementGain_dBi;
}

// ---- Public: 3 dB beamwidth from actual geometry ----

double
ThzNtnAntennaArray::ComputeBeamwidth3dB_deg() const
{
    if (m_cfg.arrayType == ThzArrayType::CASSEGRAIN)
    {
        // theta_3dB = 70 * lambda / D (degrees)
        double lambda = GetWavelength();
        return 70.0 * lambda / m_apertureDiam;
    }

    // UPA: theta_3dB = 0.886 * lambda / (N * d) in radians
    double lambda = GetWavelength();
    uint32_t Nmax = std::max(m_cfg.numElementsX, m_cfg.numElementsY);
    double bw_rad = 0.886 * lambda /
                    (static_cast<double>(Nmax) * m_cfg.elementSpacing_m);
    return bw_rad * RAD2DEG;
}

// ---- Cassegrain gain ----

double
ThzNtnAntennaArray::ComputeCassegrainGain_dBi(double thetaDeg,
                                              double apertureDiameter_m,
                                              double freqHz) const
{
    NS_LOG_FUNCTION(this << thetaDeg << apertureDiameter_m << freqHz);

    double lambda = SPEED_OF_LIGHT / freqHz;
    double eta    = 0.55; // illumination efficiency
    double arg    = PI * apertureDiameter_m / lambda;
    double G0_dBi = 10.0 * std::log10(eta * arg * arg);

    double theta3dB = 70.0 * lambda / apertureDiameter_m;
    double ratio    = thetaDeg / theta3dB;

    double rolloff = 0.0;
    if (std::abs(ratio) < 1.0)
    {
        rolloff = 12.0 * ratio * ratio;
    }
    else
    {
        rolloff = 12.0 + 10.0 * std::log10(std::max(ratio * ratio, 1.0));
    }

    return G0_dBi - rolloff;
}

// ---- Mutual coupling loss ----

double
ThzNtnAntennaArray::ComputeMutualCouplingLoss_dB() const
{
    double d_over_lambda = m_cfg.elementSpacing_m / m_lambda;
    double alpha = 0.5;
    double loss  = alpha / (d_over_lambda * d_over_lambda);
    return std::min(loss, 6.0);
}

// ---- Near-field distance: d_F = 2*D^2/lambda ----

double
ThzNtnAntennaArray::ComputeNearFieldDistance_m() const
{
    double D = ComputePhysicalSize_m();
    return 2.0 * D * D / m_lambda;
}

bool
ThzNtnAntennaArray::IsInNearField(double distance_m) const
{
    return distance_m < ComputeNearFieldDistance_m();
}

// ---- Physical size ----

double
ThzNtnAntennaArray::ComputePhysicalSize_m() const
{
    if (m_cfg.arrayType == ThzArrayType::CASSEGRAIN)
    {
        return m_apertureDiam;
    }

    double Lx = static_cast<double>(m_cfg.numElementsX) * m_cfg.elementSpacing_m;
    double Ly = static_cast<double>(m_cfg.numElementsY) * m_cfg.elementSpacing_m;
    return std::sqrt(Lx * Lx + Ly * Ly);
}

const ThzArrayConfig&
ThzNtnAntennaArray::GetConfig() const
{
    return m_cfg;
}

} // namespace ns3
