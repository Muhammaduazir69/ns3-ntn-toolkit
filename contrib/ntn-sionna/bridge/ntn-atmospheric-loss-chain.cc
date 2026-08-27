/*
 * SPDX-License-Identifier: GPL-2.0-only
 * Copyright (c) 2026 Muhammad Uzair (ns3-ntn-toolkit, Roadmap §4.2.7)
 */
#include "ntn-atmospheric-loss-chain.h"

#include "ns3/boolean.h"
#include "ns3/double.h"
#include "ns3/integer.h"
#include "ns3/log.h"

#include "ns3/thz-ntn-itu-recommendations.h"

#include <algorithm>
#include <cmath>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("NtnAtmosphericLossChain");
NS_OBJECT_ENSURE_REGISTERED(NtnAtmosphericLossChain);

namespace
{
constexpr double kEarthRadiusM = 6371000.0;
constexpr double kEcefThresholdM = 0.9 * kEarthRadiusM;

itu::Polarization
ToPol(int p)
{
    switch (p)
    {
    case 0:
        return itu::Polarization::horizontal;
    case 2:
        return itu::Polarization::circular;
    case 1:
    default:
        return itu::Polarization::vertical;
    }
}

itu::Itu618LossModel::ClimateRegion
ToClimate(int r)
{
    using C = itu::Itu618LossModel::ClimateRegion;
    switch (r)
    {
    case 0:
        return C::tropical;
    case 2:
        return C::midlat_winter;
    case 3:
        return C::subarctic;
    case 1:
    default:
        return C::midlat_summer;
    }
}

itu::Itu681LmsModel::Environment
ToLmsEnv(int e)
{
    using E = itu::Itu681LmsModel::Environment;
    switch (e)
    {
    case 0:
        return E::urban;
    case 2:
        return E::rural;
    case 3:
        return E::open;
    case 1:
    default:
        return E::suburban;
    }
}
} // namespace

TypeId
NtnAtmosphericLossChain::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::NtnAtmosphericLossChain")
            .SetParent<Object>()
            .SetGroupName("NtnSionna")
            .AddConstructor<NtnAtmosphericLossChain>()
            .AddAttribute("FrequencyHz",
                          "Carrier frequency used by P.676 + P.838.",
                          DoubleValue(12.0e9),
                          MakeDoubleAccessor(&NtnAtmosphericLossChain::SetFrequencyHz,
                                             &NtnAtmosphericLossChain::GetFrequencyHz),
                          MakeDoubleChecker<double>(1e6, 1e12))
            .AddAttribute("RainRate",
                          "Rain rate (mm/h) at 0.01% exceedance. 0 disables rain.",
                          DoubleValue(0.0),
                          MakeDoubleAccessor(&NtnAtmosphericLossChain::SetRainRateMmH,
                                             &NtnAtmosphericLossChain::GetRainRateMmH),
                          MakeDoubleChecker<double>(0.0, 500.0))
            .AddAttribute("GroundAltKm",
                          "Ground-station altitude (km MSL).",
                          DoubleValue(0.05),
                          MakeDoubleAccessor(&NtnAtmosphericLossChain::SetGroundAltKm,
                                             &NtnAtmosphericLossChain::GetGroundAltKm),
                          MakeDoubleChecker<double>(0.0, 12.0))
            .AddAttribute("EnableGaseous",
                          "Apply P.676 gaseous absorption.",
                          BooleanValue(true),
                          MakeBooleanAccessor(&NtnAtmosphericLossChain::m_enableGaseous),
                          MakeBooleanChecker())
            .AddAttribute("EnableRain",
                          "Apply P.618/P.838 rain attenuation.",
                          BooleanValue(true),
                          MakeBooleanAccessor(&NtnAtmosphericLossChain::m_enableRain),
                          MakeBooleanChecker())
            .AddAttribute("EnableLms",
                          "Apply P.681 LMS Markov shadowing.",
                          BooleanValue(false),
                          MakeBooleanAccessor(&NtnAtmosphericLossChain::m_enableLms),
                          MakeBooleanChecker())
            .AddAttribute("ClimateRegion",
                          "0 tropical, 1 midlat_summer, 2 midlat_winter, 3 subarctic.",
                          IntegerValue(1),
                          MakeIntegerAccessor(&NtnAtmosphericLossChain::SetClimateRegionInt,
                                              &NtnAtmosphericLossChain::GetClimateRegionInt),
                          MakeIntegerChecker<int>(0, 3))
            .AddAttribute("Polarization",
                          "0 horizontal, 1 vertical, 2 circular.",
                          IntegerValue(1),
                          MakeIntegerAccessor(&NtnAtmosphericLossChain::SetPolarizationInt,
                                              &NtnAtmosphericLossChain::GetPolarizationInt),
                          MakeIntegerChecker<int>(0, 2))
            .AddAttribute("LmsEnvironment",
                          "0 urban, 1 suburban, 2 rural, 3 open.",
                          IntegerValue(1),
                          MakeIntegerAccessor(&NtnAtmosphericLossChain::SetLmsEnvironmentInt,
                                              &NtnAtmosphericLossChain::GetLmsEnvironmentInt),
                          MakeIntegerChecker<int>(0, 3))
            .AddAttribute("LmsTerminalSpeedMps",
                          "THZ-08: terminal speed for the ITU-R P.681-11 LMS fade-duration "
                          "statistics, m/s. Was pinned at the 13.9 m/s default in every "
                          "shipped run because this chain never passed one through.",
                          DoubleValue(13.9),
                          MakeDoubleAccessor(&NtnAtmosphericLossChain::SetTerminalSpeedMps,
                                             &NtnAtmosphericLossChain::GetTerminalSpeedMps),
                          MakeDoubleChecker<double>(0.0));
    return tid;
}

NtnAtmosphericLossChain::NtnAtmosphericLossChain() = default;

void
NtnAtmosphericLossChain::SetTerminalSpeedMps(double v)
{
    m_lmsSpeedMps = v;
    if (m_lms)
    {
        m_lms->SetSpeedMps(v);
    }
}

NtnAtmosphericLossChain::~NtnAtmosphericLossChain() = default;

void
NtnAtmosphericLossChain::SetFrequencyHz(double freqHz)
{
    m_freqHz = freqHz;
}

void
NtnAtmosphericLossChain::SetRainRateMmH(double rateMmH)
{
    m_rainRate = std::max(0.0, rateMmH);
}

void
NtnAtmosphericLossChain::SetGroundAltKm(double altKm)
{
    m_groundAltKm = std::max(0.0, altKm);
}

void
NtnAtmosphericLossChain::SetClimateRegionInt(int region)
{
    m_climateRegionInt = std::clamp(region, 0, 3);
    if (m_modelsBuilt)
    {
        ApplyConfig();
    }
}

void
NtnAtmosphericLossChain::SetPolarizationInt(int pol)
{
    m_polInt = std::clamp(pol, 0, 2);
}

void
NtnAtmosphericLossChain::SetLmsEnvironmentInt(int env)
{
    m_lmsEnvInt = std::clamp(env, 0, 3);
    if (m_modelsBuilt)
    {
        ApplyConfig();
    }
}

void
NtnAtmosphericLossChain::EnsureModels() const
{
    if (m_modelsBuilt)
    {
        return;
    }
    m_rain = CreateObject<itu::Itu618LossModel>();
    m_gas = CreateObject<itu::Itu676AbsorptionModel>();
    m_lms = CreateObject<itu::Itu681LmsModel>();
    m_modelsBuilt = true;
    ApplyConfig();
}

void
NtnAtmosphericLossChain::ApplyConfig() const
{
    if (m_rain)
    {
        m_rain->SetClimateRegion(ToClimate(m_climateRegionInt));
    }
    if (m_lms)
    {
        m_lms->SetEnvironment(ToLmsEnv(m_lmsEnvInt));
        // THZ-08: pass the speed through. Setting only the environment left the
        // P.681 fade-duration statistics on their 13.9 m/s default for every
        // terminal class the toolkit models.
        m_lms->SetSpeedMps(m_lmsSpeedMps);
    }
}

double
NtnAtmosphericLossChain::SlantPathLengthKm(const Vector& a, const Vector& b)
{
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    const double dz = b.z - a.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz) / 1000.0;
}

double
NtnAtmosphericLossChain::ComputeElevationDeg(const Vector& groundPos,
                                              const Vector& satPos)
{
    const double rGround = std::sqrt(groundPos.x * groundPos.x +
                                      groundPos.y * groundPos.y +
                                      groundPos.z * groundPos.z);
    double ux, uy, uz;
    if (rGround > kEcefThresholdM)
    {
        // ECEF: up = radial unit from Earth center to ground.
        ux = groundPos.x / rGround;
        uy = groundPos.y / rGround;
        uz = groundPos.z / rGround;
    }
    else
    {
        // Local ENU: up is +z.
        ux = 0.0;
        uy = 0.0;
        uz = 1.0;
    }
    const double lx = satPos.x - groundPos.x;
    const double ly = satPos.y - groundPos.y;
    const double lz = satPos.z - groundPos.z;
    const double lNorm = std::sqrt(lx * lx + ly * ly + lz * lz);
    if (lNorm <= 0.0)
    {
        return 90.0;
    }
    double cosZenith = (lx * ux + ly * uy + lz * uz) / lNorm;
    cosZenith = std::clamp(cosZenith, -1.0, 1.0);
    const double zenithDeg = std::acos(cosZenith) * 180.0 / M_PI;
    return std::clamp(90.0 - zenithDeg, -90.0, 90.0);
}

double
NtnAtmosphericLossChain::ComputeAttenuationDb(const Vector& groundPos,
                                                const Vector& satPos) const
{
    EnsureModels();
    const double elevDeg = ComputeElevationDeg(groundPos, satPos);
    m_last.elevationDeg = elevDeg;
    m_last.gaseousDb = 0.0;
    m_last.rainDb = 0.0;

    if (elevDeg <= 0.0)
    {
        // Below horizon — by convention the link is unusable; report a
        // large additional attenuation so receivers downstream fall back
        // to noise-only. 100 dB is more than enough for any LEO budget.
        const double belowHorizonDb = 100.0;
        m_last.gaseousDb = m_enableGaseous ? belowHorizonDb : 0.0;
        return m_last.gaseousDb;
    }

    double total = 0.0;
    if (m_enableGaseous)
    {
        // SIONNA-05: pass the configured station altitude. It used to reach
        // the rain term only, so a mountain-top station paid the full sea-level
        // gaseous column while its rain path was correctly shortened - the two
        // halves of the same cascade disagreed about where the station was.
        m_last.gaseousDb =
            m_gas->SlantPathAttenuationDb(m_freqHz, elevDeg, m_groundAltKm);
        total += m_last.gaseousDb;
    }
    if (m_enableRain && m_rainRate > 0.0)
    {
        m_last.rainDb = m_rain->SlantPathRainAttenuationDb(
            m_freqHz, elevDeg, m_rainRate, m_groundAltKm, ToPol(m_polInt));
        total += m_last.rainDb;
    }
    return total;
}

double
NtnAtmosphericLossChain::StepLmsDb() const
{
    if (!m_enableLms)
    {
        m_last.lmsDb = 0.0;
        return 0.0;
    }
    EnsureModels();
    m_last.lmsDb = m_lms->StepDb();
    return m_last.lmsDb;
}

int64_t
NtnAtmosphericLossChain::AssignStreams(int64_t stream)
{
    EnsureModels();
    return m_lms->AssignStreams(stream);
}

} // namespace ns3
