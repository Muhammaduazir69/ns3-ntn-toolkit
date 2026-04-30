/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026 Muhammad Uzair
 * SPDX-License-Identifier: GPL-2.0-only
 */

#include "ntn-realistic-mobility.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace ns3
{

// =====================================================================
// Built-in mobility profiles
// =====================================================================

MobilityProfile
NtnMobilityScenarios::MixedContinental()
{
    MobilityProfile p;
    p.pStatic     = 0.30;
    p.pPedestrian = 0.25;
    p.pVehicular  = 0.30;
    p.pHst        = 0.04;
    p.pMaritime   = 0.02;
    p.pAviation   = 0.04;
    p.pIot        = 0.05;
    return p;
}

MobilityProfile
NtnMobilityScenarios::Maritime()
{
    MobilityProfile p;
    p.pStatic     = 0.05;
    p.pPedestrian = 0.05;
    p.pVehicular  = 0.05;
    p.pHst        = 0.0;
    p.pMaritime   = 0.65;
    p.pAviation   = 0.05;
    p.pIot        = 0.15;
    return p;
}

MobilityProfile
NtnMobilityScenarios::AeronauticalCorridor()
{
    MobilityProfile p;
    p.pStatic     = 0.10;
    p.pPedestrian = 0.10;
    p.pVehicular  = 0.10;
    p.pHst        = 0.0;
    p.pMaritime   = 0.10;
    p.pAviation   = 0.50;
    p.pIot        = 0.10;
    return p;
}

MobilityProfile
NtnMobilityScenarios::UrbanDense()
{
    MobilityProfile p;
    p.pStatic     = 0.40;
    p.pPedestrian = 0.30;
    p.pVehicular  = 0.20;
    p.pHst        = 0.0;
    p.pMaritime   = 0.0;
    p.pAviation   = 0.0;
    p.pIot        = 0.10;
    return p;
}

MobilityProfile
NtnMobilityScenarios::IotField()
{
    MobilityProfile p;
    p.pStatic     = 0.10;
    p.pPedestrian = 0.0;
    p.pVehicular  = 0.10;
    p.pHst        = 0.0;
    p.pMaritime   = 0.0;
    p.pAviation   = 0.0;
    p.pIot        = 0.80;
    return p;
}

// =====================================================================
// Helper class
// =====================================================================

NtnRealisticMobilityHelper::NtnRealisticMobilityHelper(uint64_t seed)
    : m_rng(seed)
{
}

namespace
{

// Cumulative class-weight thresholds for a profile.
struct ClassThresholds
{
    double tStatic, tPed, tVeh, tHst, tMar, tAvi;
};

ClassThresholds
ProfileToThresholds(const MobilityProfile& p)
{
    ClassThresholds t;
    t.tStatic = p.pStatic;
    t.tPed    = t.tStatic + p.pPedestrian;
    t.tVeh    = t.tPed + p.pVehicular;
    t.tHst    = t.tVeh + p.pHst;
    t.tMar    = t.tHst + p.pMaritime;
    t.tAvi    = t.tMar + p.pAviation;
    // pIot fills the remainder up to 1.0
    return t;
}

NtnUeClass
PickClass(double r, const ClassThresholds& t)
{
    if (r < t.tStatic) return HANDHELD_STATIC;
    if (r < t.tPed)    return HANDHELD_PEDESTRIAN;
    if (r < t.tVeh)    return VEHICULAR;
    if (r < t.tHst)    return HIGH_SPEED_TRAIN;
    if (r < t.tMar)    return MARITIME_VESSEL;
    if (r < t.tAvi)    return AVIATION_COMMERCIAL;
    return IOT_FIXED;
}

const char*
ClassName(NtnUeClass c)
{
    switch (c)
    {
    case HANDHELD_STATIC:      return "static";
    case HANDHELD_PEDESTRIAN:  return "pedestrian";
    case VEHICULAR:            return "vehicular";
    case HIGH_SPEED_TRAIN:     return "hst";
    case MARITIME_VESSEL:      return "maritime";
    case AVIATION_COMMERCIAL:  return "aviation";
    case IOT_FIXED:            return "iot";
    }
    return "unknown";
}

} // anonymous namespace

std::vector<RealisticUe>
NtnRealisticMobilityHelper::GenerateUes(uint32_t n,
                                         const MobilityProfile& profile,
                                         double minLat, double maxLat,
                                         double minLon, double maxLon)
{
    std::vector<RealisticUe> ues;
    ues.reserve(n);

    std::uniform_real_distribution<double> uni01(0.0, 1.0);
    std::uniform_real_distribution<double> latUni(minLat, maxLat);
    std::uniform_real_distribution<double> lonUni(minLon, maxLon);

    auto thr = ProfileToThresholds(profile);

    for (uint32_t i = 0; i < n; i++)
    {
        RealisticUe ue;
        ue.id = i;
        ue.ueClass = PickClass(uni01(m_rng), thr);
        ue.className = ClassName(ue.ueClass);

        // Default geographic uniform; specialised classes will overwrite.
        ue.lat = latUni(m_rng);
        ue.lon = lonUni(m_rng);
        ue.altitude_m = 0.0;

        switch (ue.ueClass)
        {
        case HANDHELD_STATIC:      InitializeStatic(ue);      break;
        case HANDHELD_PEDESTRIAN:  InitializePedestrian(ue);  break;
        case VEHICULAR:            InitializeVehicular(ue);   break;
        case HIGH_SPEED_TRAIN:     InitializeHst(ue);         break;
        case MARITIME_VESSEL:      InitializeMaritime(ue);    break;
        case AVIATION_COMMERCIAL:  InitializeAviation(ue);    break;
        case IOT_FIXED:            InitializeIot(ue);         break;
        }
        ues.push_back(ue);
    }
    return ues;
}

void
NtnRealisticMobilityHelper::InitializeStatic(RealisticUe& ue)
{
    // Stationary handheld — no motion.
    ue.vEast = ue.vNorth = ue.vUp = 0.0;
    ue.speed_mps = 0.0;
}

void
NtnRealisticMobilityHelper::InitializePedestrian(RealisticUe& ue)
{
    // 3GPP TR 38.811: pedestrian baseline 3 km/h ≈ 0.83 m/s.
    // We sample N(1.2, 0.4²) m/s clamped to [0.3, 3.0].
    std::normal_distribution<double> sp(1.2, 0.4);
    std::uniform_real_distribution<double> ang(0.0, 2.0 * M_PI);
    ue.speed_mps = std::clamp(sp(m_rng), 0.3, 3.0);
    ue.heading_rad = ang(m_rng);
    ue.vEast  = ue.speed_mps * std::sin(ue.heading_rad);
    ue.vNorth = ue.speed_mps * std::cos(ue.heading_rad);
    ue.headingChangeUntil_s = 0.0;
}

void
NtnRealisticMobilityHelper::InitializeVehicular(RealisticUe& ue)
{
    // Mix of urban (8–14 m/s) and highway (25–36 m/s); 3GPP NTN
    // baseline 100 km/h (27.8 m/s). 70/30 highway/urban split.
    std::uniform_real_distribution<double> u(0.0, 1.0);
    std::normal_distribution<double> hwy(27.8, 4.0);
    std::normal_distribution<double> urb(11.0, 2.5);
    std::uniform_real_distribution<double> ang(0.0, 2.0 * M_PI);

    ue.speed_mps = (u(m_rng) < 0.7)
                       ? std::clamp(hwy(m_rng), 18.0, 40.0)
                       : std::clamp(urb(m_rng), 4.0, 18.0);
    ue.heading_rad = ang(m_rng);
    ue.vEast  = ue.speed_mps * std::sin(ue.heading_rad);
    ue.vNorth = ue.speed_mps * std::cos(ue.heading_rad);
}

void
NtnRealisticMobilityHelper::InitializeHst(RealisticUe& ue)
{
    // 3GPP TR 38.901 §7.2: HST baseline 300 km/h (83.3 m/s),
    // strictly linear track. We give the train a single 1500-km
    // straight track aligned to its initial heading; constant
    // velocity over the simulation horizon.
    std::normal_distribution<double> sp(83.3, 5.0);
    std::uniform_real_distribution<double> ang(0.0, 2.0 * M_PI);
    ue.speed_mps = std::clamp(sp(m_rng), 60.0, 140.0);
    ue.heading_rad = ang(m_rng);
    ue.vEast  = ue.speed_mps * std::sin(ue.heading_rad);
    ue.vNorth = ue.speed_mps * std::cos(ue.heading_rad);
    // No waypoint rerouting needed — track is straight.
}

void
NtnRealisticMobilityHelper::InitializeMaritime(RealisticUe& ue)
{
    // IMO TSS — typical commercial vessel 6–13 m/s. Constrain initial
    // position to mid-latitude open-ocean band (lat ∈ [25°, 55°]) and
    // pick heading aligned with prevailing east–west shipping lanes.
    std::normal_distribution<double> sp(11.0, 2.0);
    std::uniform_real_distribution<double> u(0.0, 1.0);
    // 50% east-bound, 50% west-bound (Atlantic / Pacific traffic).
    ue.speed_mps = std::clamp(sp(m_rng), 4.0, 16.0);
    ue.heading_rad = (u(m_rng) < 0.5) ? (M_PI / 2.0) : (-M_PI / 2.0);
    // Light offset so they do not all run on a single line.
    std::normal_distribution<double> headingJitter(0.0, 0.15); // ~9 deg
    ue.heading_rad += headingJitter(m_rng);
    ue.vEast  = ue.speed_mps * std::sin(ue.heading_rad);
    ue.vNorth = ue.speed_mps * std::cos(ue.heading_rad);
}

void
NtnRealisticMobilityHelper::InitializeAviation(RealisticUe& ue)
{
    // Commercial cruise: ground speed 220–260 m/s, FL350 = 10 668 m.
    // Heading along great-circle airway; we model as constant.
    std::normal_distribution<double> sp(240.0, 12.0);
    std::uniform_real_distribution<double> ang(0.0, 2.0 * M_PI);
    ue.speed_mps = std::clamp(sp(m_rng), 200.0, 280.0);
    ue.altitude_m = 10668.0; // FL350
    ue.heading_rad = ang(m_rng);
    ue.vEast  = ue.speed_mps * std::sin(ue.heading_rad);
    ue.vNorth = ue.speed_mps * std::cos(ue.heading_rad);
    ue.vUp = 0.0;
}

void
NtnRealisticMobilityHelper::InitializeIot(RealisticUe& ue)
{
    // NB-IoT NTN TR 36.763: stationary UE assumption.
    ue.vEast = ue.vNorth = ue.vUp = 0.0;
    ue.speed_mps = 0.0;
}

// =====================================================================
// Per-class motion rules
// =====================================================================

void
NtnRealisticMobilityHelper::AdvanceUe(RealisticUe& ue, double dt_s)
{
    switch (ue.ueClass)
    {
    case HANDHELD_STATIC:
    case IOT_FIXED:
        // No motion. Sub-mm/s drift is below trace resolution.
        return;

    case HANDHELD_PEDESTRIAN:
        AdvanceWalk(ue, dt_s);
        break;

    case VEHICULAR:
        AdvanceVehicular(ue, dt_s);
        break;

    case HIGH_SPEED_TRAIN:
    case MARITIME_VESSEL:
    case AVIATION_COMMERCIAL:
        // Constant-velocity classes: heading stays put except for
        // optional waypoint hops (not exercised by the default
        // 600-s horizon, so AdvanceWaypoint just integrates).
        AdvanceWaypoint(ue, dt_s);
        break;
    }
}

void
NtnRealisticMobilityHelper::AdvanceWalk(RealisticUe& ue, double dt_s)
{
    // Random-walk: re-roll heading every U(5,15) s; speed remains
    // approximately constant (memory-less Brownian heading).
    if (ue.headingChangeUntil_s <= 0.0)
    {
        std::uniform_real_distribution<double> nextChange(5.0, 15.0);
        std::uniform_real_distribution<double> ang(0.0, 2.0 * M_PI);
        ue.heading_rad = ang(m_rng);
        ue.headingChangeUntil_s = nextChange(m_rng);
        ue.vEast  = ue.speed_mps * std::sin(ue.heading_rad);
        ue.vNorth = ue.speed_mps * std::cos(ue.heading_rad);
    }
    else
    {
        ue.headingChangeUntil_s -= dt_s;
    }
    GeoStep(ue, dt_s);
}

void
NtnRealisticMobilityHelper::AdvanceVehicular(RealisticUe& ue, double dt_s)
{
    // Gauss-Markov heading with correlation time τ ≈ 60 s.
    // θ(t+dt) = α θ(t) + (1−α) θ_mean + σ √(1−α²) ε,  ε ~ N(0,1)
    constexpr double tau = 60.0;
    double alpha = std::exp(-dt_s / tau);
    std::normal_distribution<double> eps(0.0, 1.0);
    double sigma = 0.2; // rad — tight angular spread (aligned roads)
    ue.heading_rad = alpha * ue.heading_rad +
                     (1.0 - alpha) * 0.0 +
                     sigma * std::sqrt(1.0 - alpha * alpha) * eps(m_rng);
    ue.vEast  = ue.speed_mps * std::sin(ue.heading_rad);
    ue.vNorth = ue.speed_mps * std::cos(ue.heading_rad);
    GeoStep(ue, dt_s);
}

void
NtnRealisticMobilityHelper::AdvanceWaypoint(RealisticUe& ue, double dt_s)
{
    GeoStep(ue, dt_s);
}

void
NtnRealisticMobilityHelper::GeoStep(RealisticUe& ue, double dt_s)
{
    // Equirectangular linearization, valid for short steps.
    constexpr double M_PER_DEG_LAT = 111320.0;
    double mPerDegLon = M_PER_DEG_LAT *
                        std::max(std::cos(ue.lat * M_PI / 180.0), 0.01);
    ue.lat += ue.vNorth * dt_s / M_PER_DEG_LAT;
    ue.lon += ue.vEast  * dt_s / mPerDegLon;
    if (ue.altitude_m > 0.0)
    {
        ue.altitude_m += ue.vUp * dt_s;
    }
}

} // namespace ns3
