/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-only
 * Author: Muhammad Uzair
 *
 * THz-NTN Free-Space Loss Extension for Satellite Module Integration
 */

#include "thz-ntn-free-space-loss.h"

#include "thz-ntn-molecular-absorption.h"
#include "thz-ntn-pointing-error.h"
#include "thz-ntn-scintillation.h"
#include "thz-ntn-weather-attenuation.h"

#include <ns3/boolean.h>
#include <ns3/double.h>
#include <ns3/log.h>

#include <ns3/satellite-const-variables.h>
#include <ns3/satellite-mobility-model.h>
#include <ns3/satellite-utils.h>
#include <ns3/geo-coordinate.h>

#include <cmath>

NS_LOG_COMPONENT_DEFINE("ThzNtnFreeSpaceLoss");

namespace ns3
{

NS_OBJECT_ENSURE_REGISTERED(ThzNtnFreeSpaceLoss);

TypeId
ThzNtnFreeSpaceLoss::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::ThzNtnFreeSpaceLoss")
            .SetParent<SatFreeSpaceLoss>()
            .SetGroupName("ThzNtn")
            .AddConstructor<ThzNtnFreeSpaceLoss>()
            .AddAttribute("EnableMolecularAbsorption",
                          "Enable molecular absorption modelling",
                          BooleanValue(true),
                          MakeBooleanAccessor(&ThzNtnFreeSpaceLoss::m_enableAbsorption),
                          MakeBooleanChecker())
            .AddAttribute("EnableWeatherEffects",
                          "Enable weather attenuation modelling",
                          BooleanValue(false),
                          MakeBooleanAccessor(&ThzNtnFreeSpaceLoss::m_enableWeather),
                          MakeBooleanChecker())
            .AddAttribute("EnableScintillation",
                          "Enable tropospheric scintillation modelling",
                          BooleanValue(false),
                          MakeBooleanAccessor(&ThzNtnFreeSpaceLoss::m_enableScintillation),
                          MakeBooleanChecker())
            .AddAttribute("EnablePointingError",
                          "Enable beam pointing error modelling",
                          BooleanValue(false),
                          MakeBooleanAccessor(&ThzNtnFreeSpaceLoss::m_enablePointing),
                          MakeBooleanChecker())
            .AddAttribute("Beamwidth3dB",
                          "Half-power beamwidth in degrees for pointing loss computation",
                          DoubleValue(0.1),
                          MakeDoubleAccessor(&ThzNtnFreeSpaceLoss::m_beamwidth3dB_deg),
                          MakeDoubleChecker<double>(0.001, 10.0))
            .AddAttribute("SatVelocity",
                          "Satellite ground-track velocity in km/s",
                          DoubleValue(7.5),
                          MakeDoubleAccessor(&ThzNtnFreeSpaceLoss::m_satVelocity_km_s),
                          MakeDoubleChecker<double>(0.0, 30.0))
            .AddTraceSource("ThzLossBreakdown",
                            "Per-component loss breakdown: baseFspl, absorption, "
                            "weather, scintillation, pointing (all in dB)",
                            MakeTraceSourceAccessor(
                                &ThzNtnFreeSpaceLoss::m_thzLossBreakdown),
                            "ns3::ThzNtnFreeSpaceLoss::LossBreakdownTracedCallback");
    return tid;
}

ThzNtnFreeSpaceLoss::ThzNtnFreeSpaceLoss()
    : m_absorptionModel(nullptr),
      m_weatherModel(nullptr),
      m_scintillationModel(nullptr),
      m_pointingErrorModel(nullptr),
      m_enableAbsorption(true),
      m_enableWeather(false),
      m_enableScintillation(false),
      m_enablePointing(false),
      m_beamwidth3dB_deg(0.1),
      m_satVelocity_km_s(7.5),
      m_lastBaseFspl_dB(0.0),
      m_lastAbsorption_dB(0.0),
      m_lastWeather_dB(0.0),
      m_lastScintillation_dB(0.0),
      m_lastPointing_dB(0.0)
{
    NS_LOG_FUNCTION(this);
}

ThzNtnFreeSpaceLoss::~ThzNtnFreeSpaceLoss()
{
    NS_LOG_FUNCTION(this);
}

// ---------------------------------------------------------------------------
// Overridden SatFreeSpaceLoss methods
// ---------------------------------------------------------------------------

double
ThzNtnFreeSpaceLoss::GetFsldB(Ptr<MobilityModel> a,
                               Ptr<MobilityModel> b,
                               double frequencyHz) const
{
    NS_LOG_FUNCTION(this << frequencyHz);

    // --- 1. Base FSPL computed directly from 3D positions (works with any MobilityModel) ---
    static constexpr double SPEED_OF_LIGHT = 299792458.0;
    double distance = a->GetDistanceFrom(b);
    double baseFspl_dB = 0.0;
    if (distance > 0.0 && frequencyHz > 0.0)
    {
        baseFspl_dB = 20.0 * std::log10(4.0 * M_PI * distance * frequencyHz / SPEED_OF_LIGHT);
    }
    m_lastBaseFspl_dB = baseFspl_dB;

    // --- 2. Determine node roles (ground vs space) ---
    double altA_km = ComputeAltitude(a);
    double altB_km = ComputeAltitude(b);

    bool aIsSpace = (altA_km > ISL_ALTITUDE_THRESHOLD_KM);
    bool bIsSpace = (altB_km > ISL_ALTITUDE_THRESHOLD_KM);
    bool isIsl = (aIsSpace && bIsSpace);

    // --- 3. ISL: no atmospheric effects, only pointing loss ---
    if (isIsl)
    {
        double pointing_dB = 0.0;
        if (m_enablePointing && m_pointingErrorModel)
        {
            // For ISL, use a nominal elevation of 90 deg (no atmospheric refraction)
            pointing_dB = m_pointingErrorModel->ComputeTotalPointingLoss_dB(
                90.0, m_satVelocity_km_s, m_beamwidth3dB_deg);
        }

        m_lastAbsorption_dB = 0.0;
        m_lastWeather_dB = 0.0;
        m_lastScintillation_dB = 0.0;
        m_lastPointing_dB = pointing_dB;

        double totalLoss_dB = baseFspl_dB + pointing_dB;

        m_thzLossBreakdown(baseFspl_dB, 0.0, 0.0, 0.0, pointing_dB);

        NS_LOG_INFO("THz-NTN ISL: baseFSPL=" << baseFspl_dB
                    << " dB, pointing=" << pointing_dB
                    << " dB, total=" << totalLoss_dB << " dB");

        return totalLoss_dB;
    }

    // --- 4. Atmospheric link: identify ground and space nodes ---
    Ptr<MobilityModel> groundMob = (altA_km <= altB_km) ? a : b;
    Ptr<MobilityModel> spaceMob = (altA_km <= altB_km) ? b : a;
    double groundAlt_km = std::min(altA_km, altB_km);
    double spaceAlt_km = std::max(altA_km, altB_km);

    // --- 5. Compute elevation angle ---
    double elevationDeg = ComputeElevationAngle(groundMob, spaceMob);
    double elevForAtmospheric = std::max(elevationDeg, MIN_ELEVATION_DEG);

    // Compute 3-D distance for absorption model
    Ptr<SatMobilityModel> satMobA = DynamicCast<SatMobilityModel>(a);
    Ptr<SatMobilityModel> satMobB = DynamicCast<SatMobilityModel>(b);
    double distanceM;
    if (satMobA && satMobB)
    {
        distanceM = satMobA->GetDistanceFrom(satMobB);
    }
    else
    {
        Vector posA = a->GetPosition();
        Vector posB = b->GetPosition();
        double dx = posA.x - posB.x;
        double dy = posA.y - posB.y;
        double dz = posA.z - posB.z;
        distanceM = std::sqrt(dx * dx + dy * dy + dz * dz);
    }

    NS_LOG_DEBUG("Atmospheric link: groundAlt=" << groundAlt_km
                 << " km, spaceAlt=" << spaceAlt_km
                 << " km, elev=" << elevationDeg
                 << " deg, dist=" << distanceM << " m");

    // --- 6. Molecular absorption ---
    double absorption_dB = 0.0;
    if (m_enableAbsorption && m_absorptionModel)
    {
        absorption_dB = m_absorptionModel->ComputeAbsorptionLoss_dB(
            frequencyHz, distanceM, groundAlt_km, spaceAlt_km, elevForAtmospheric);
    }

    // --- 7. Weather attenuation ---
    double weather_dB = 0.0;
    if (m_enableWeather && m_weatherModel)
    {
        weather_dB = m_weatherModel->ComputeTotalWeatherLoss_dB(
            frequencyHz, elevForAtmospheric);
    }

    // --- 8. Scintillation ---
    double scintillation_dB = 0.0;
    if (m_enableScintillation && m_scintillationModel)
    {
        // GetScintillationSample_dB is non-const (AR(1) state update),
        // so we const_cast to call it from the const GetFsldB method.
        scintillation_dB = const_cast<ThzNtnScintillation*>(
            PeekPointer(m_scintillationModel))->GetScintillationSample_dB(
                frequencyHz, elevForAtmospheric);
        // Scintillation can be negative (enhancement) or positive (fade);
        // we take the absolute value as a loss contribution
        scintillation_dB = std::abs(scintillation_dB);
    }

    // --- 9. Pointing error ---
    double pointing_dB = 0.0;
    if (m_enablePointing && m_pointingErrorModel)
    {
        pointing_dB = m_pointingErrorModel->ComputeTotalPointingLoss_dB(
            elevForAtmospheric, m_satVelocity_km_s, m_beamwidth3dB_deg);
    }

    // --- 10. Total loss ---
    double totalLoss_dB = baseFspl_dB + absorption_dB + weather_dB +
                          scintillation_dB + pointing_dB;

    // --- 11. Cache individual losses for KPM reporting ---
    m_lastAbsorption_dB = absorption_dB;
    m_lastWeather_dB = weather_dB;
    m_lastScintillation_dB = scintillation_dB;
    m_lastPointing_dB = pointing_dB;

    // --- 12. Fire traced callback with breakdown ---
    m_thzLossBreakdown(baseFspl_dB, absorption_dB, weather_dB,
                       scintillation_dB, pointing_dB);

    NS_LOG_INFO("THz-NTN Channel: baseFSPL=" << baseFspl_dB
                << " dB, absorption=" << absorption_dB
                << " dB, weather=" << weather_dB
                << " dB, scintillation=" << scintillation_dB
                << " dB, pointing=" << pointing_dB
                << " dB, total=" << totalLoss_dB << " dB");

    return totalLoss_dB;
}

double
ThzNtnFreeSpaceLoss::GetFsl(Ptr<MobilityModel> a,
                             Ptr<MobilityModel> b,
                             double frequencyHz) const
{
    NS_LOG_FUNCTION(this << frequencyHz);

    // Compute total loss in dB, then convert to linear ratio
    double totalLoss_dB = GetFsldB(a, b, frequencyHz);
    return SatUtils::DbToLinear(totalLoss_dB);
}

// ---------------------------------------------------------------------------
// Sub-model setters
// ---------------------------------------------------------------------------

void
ThzNtnFreeSpaceLoss::SetMolecularAbsorptionModel(Ptr<ThzNtnMolecularAbsorption> model)
{
    NS_LOG_FUNCTION(this << model);
    m_absorptionModel = model;
}

void
ThzNtnFreeSpaceLoss::SetWeatherModel(Ptr<ThzNtnWeatherAttenuation> model)
{
    NS_LOG_FUNCTION(this << model);
    m_weatherModel = model;
}

void
ThzNtnFreeSpaceLoss::SetScintillationModel(Ptr<ThzNtnScintillation> model)
{
    NS_LOG_FUNCTION(this << model);
    m_scintillationModel = model;
}

void
ThzNtnFreeSpaceLoss::SetPointingErrorModel(Ptr<ThzNtnPointingError> model)
{
    NS_LOG_FUNCTION(this << model);
    m_pointingErrorModel = model;
}

// ---------------------------------------------------------------------------
// Enable/disable flags
// ---------------------------------------------------------------------------

void
ThzNtnFreeSpaceLoss::EnableMolecularAbsorption(bool enable)
{
    NS_LOG_FUNCTION(this << enable);
    m_enableAbsorption = enable;
}

void
ThzNtnFreeSpaceLoss::EnableWeatherEffects(bool enable)
{
    NS_LOG_FUNCTION(this << enable);
    m_enableWeather = enable;
}

void
ThzNtnFreeSpaceLoss::EnableScintillation(bool enable)
{
    NS_LOG_FUNCTION(this << enable);
    m_enableScintillation = enable;
}

void
ThzNtnFreeSpaceLoss::EnablePointingError(bool enable)
{
    NS_LOG_FUNCTION(this << enable);
    m_enablePointing = enable;
}

// ---------------------------------------------------------------------------
// Cached result accessors
// ---------------------------------------------------------------------------

double
ThzNtnFreeSpaceLoss::GetLastBaseFspl_dB() const
{
    return m_lastBaseFspl_dB;
}

double
ThzNtnFreeSpaceLoss::GetLastMolecularAbsorption_dB() const
{
    return m_lastAbsorption_dB;
}

double
ThzNtnFreeSpaceLoss::GetLastWeatherLoss_dB() const
{
    return m_lastWeather_dB;
}

double
ThzNtnFreeSpaceLoss::GetLastScintillationLoss_dB() const
{
    return m_lastScintillation_dB;
}

double
ThzNtnFreeSpaceLoss::GetLastPointingLoss_dB() const
{
    return m_lastPointing_dB;
}

// ---------------------------------------------------------------------------
// Private helpers
// ---------------------------------------------------------------------------

double
ThzNtnFreeSpaceLoss::ComputeElevationAngle(Ptr<MobilityModel> ground,
                                            Ptr<MobilityModel> sat) const
{
    NS_LOG_FUNCTION(this);

    // Try to use SatMobilityModel for geodetic coordinates (more accurate)
    Ptr<SatMobilityModel> satMobGround = DynamicCast<SatMobilityModel>(ground);
    Ptr<SatMobilityModel> satMobSat = DynamicCast<SatMobilityModel>(sat);

    if (satMobGround && satMobSat)
    {
        // Use geodetic coordinates with Earth-curvature correction
        GeoCoordinate geoGround = satMobGround->GetGeoPosition();
        GeoCoordinate geoSat = satMobSat->GetGeoPosition();

        double groundAlt_m = geoGround.GetAltitude();
        double satAlt_m = geoSat.GetAltitude();
        double distanceM = satMobGround->GetDistanceFrom(satMobSat);

        // Earth-curvature-corrected elevation angle
        // Using the law of cosines on the triangle:
        //   Earth center -- ground station -- satellite
        double rGround = EARTH_RADIUS_KM * 1000.0 + groundAlt_m;
        double rSat = EARTH_RADIUS_KM * 1000.0 + satAlt_m;

        if (distanceM < 1.0)
        {
            return 90.0;
        }

        // Central angle from the law of cosines:
        // d^2 = rG^2 + rS^2 - 2*rG*rS*cos(centralAngle)
        double cosGamma = (rGround * rGround + rSat * rSat -
                           distanceM * distanceM) /
                          (2.0 * rGround * rSat);
        cosGamma = std::max(-1.0, std::min(1.0, cosGamma));

        // Elevation at the ground station:
        // sin(elev) = (rSat * cos(gamma) - rGround) / d
        double gamma = std::acos(cosGamma);
        double sinElev = (rSat * std::cos(gamma) - rGround) / distanceM;
        sinElev = std::max(-1.0, std::min(1.0, sinElev));

        double elevDeg = std::asin(sinElev) * 180.0 / M_PI;
        return std::max(0.0, std::min(90.0, elevDeg));
    }

    // Fallback: detect geocentric vs topocentric coordinates
    Vector posG = ground->GetPosition();
    Vector posS = sat->GetPosition();

    double rG = std::sqrt(posG.x * posG.x + posG.y * posG.y + posG.z * posG.z);
    double rS = std::sqrt(posS.x * posS.x + posS.y * posS.y + posS.z * posS.z);

    // Geocentric coordinates: both r values > R_E/2
    if (rG > EARTH_RADIUS_KM * 500.0 && rS > EARTH_RADIUS_KM * 500.0)
    {
        double distanceM = ground->GetDistanceFrom(sat);
        if (distanceM < 1.0)
        {
            return 90.0;
        }
        // Elevation via law of cosines
        double cosGamma = (rG * rG + rS * rS - distanceM * distanceM) / (2.0 * rG * rS);
        cosGamma = std::max(-1.0, std::min(1.0, cosGamma));
        double gamma = std::acos(cosGamma);
        double sinElev = (rS * std::cos(gamma) - rG) / distanceM;
        sinElev = std::max(-1.0, std::min(1.0, sinElev));
        double elevDeg = std::asin(sinElev) * 180.0 / M_PI;
        return std::max(0.0, std::min(90.0, elevDeg));
    }

    // Topocentric: Z = altitude
    double dh = std::sqrt(std::pow(posS.x - posG.x, 2) +
                          std::pow(posS.y - posG.y, 2));
    double dv = posS.z - posG.z;

    if (dh < 1.0 && dv < 1.0)
    {
        return 90.0;
    }

    double elevRad = std::atan2(dv, dh);
    double elevDeg = elevRad * 180.0 / M_PI;
    return std::max(0.0, std::min(90.0, elevDeg));
}

double
ThzNtnFreeSpaceLoss::ComputeAltitude(Ptr<MobilityModel> node) const
{
    static constexpr double EARTH_RADIUS_M = 6371000.0;

    // Try SatMobilityModel for geodetic altitude
    Ptr<SatMobilityModel> satMob = DynamicCast<SatMobilityModel>(node);
    if (satMob)
    {
        GeoCoordinate geo = satMob->GetGeoPosition();
        return geo.GetAltitude() / 1000.0; // convert m to km
    }

    // Fallback: detect geocentric coordinates (|r| > R_E/2) vs local (z as altitude)
    Vector pos = node->GetPosition();
    double r = std::sqrt(pos.x * pos.x + pos.y * pos.y + pos.z * pos.z);
    if (r > EARTH_RADIUS_M * 0.5)
    {
        // Geocentric: altitude = distance from Earth center minus Earth radius
        return (r - EARTH_RADIUS_M) / 1000.0;
    }
    // Local coordinate system: Z = altitude in metres
    return pos.z / 1000.0;
}

bool
ThzNtnFreeSpaceLoss::IsSpaceNode(Ptr<MobilityModel> node) const
{
    return (ComputeAltitude(node) > ISL_ALTITUDE_THRESHOLD_KM);
}

} // namespace ns3
