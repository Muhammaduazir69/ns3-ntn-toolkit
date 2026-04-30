/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-only
 * Author: Muhammad Uzair
 *
 * Energy Harvesting xApp - Implementation
 *
 * Solar panel state + battery management for power-aware satellite
 * resource allocation. Integrates orbital mechanics for eclipse
 * prediction, proactive beam shutdown, and energy-proportional compute.
 */

#include "oran-ntn-xapp-energy-harvest.h"

#include "oran-ntn-sat-bridge.h"

#include <ns3/double.h>
#include <ns3/log.h>
#include <ns3/simulator.h>

#include <algorithm>
#include <cmath>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("OranNtnXappEnergyHarvest");
NS_OBJECT_ENSURE_REGISTERED(OranNtnXappEnergyHarvest);

TypeId
OranNtnXappEnergyHarvest::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::OranNtnXappEnergyHarvest")
            .SetParent<OranNtnXappBase>()
            .SetGroupName("OranNtn")
            .AddConstructor<OranNtnXappEnergyHarvest>()
            .AddAttribute("SolarPanelArea",
                          "Solar panel area in m^2",
                          DoubleValue(10.0),
                          MakeDoubleAccessor(&OranNtnXappEnergyHarvest::m_solarPanelArea_m2),
                          MakeDoubleChecker<double>(0.0))
            .AddAttribute("SolarEfficiency",
                          "Solar cell conversion efficiency (0-1)",
                          DoubleValue(0.30),
                          MakeDoubleAccessor(&OranNtnXappEnergyHarvest::m_solarEfficiency),
                          MakeDoubleChecker<double>(0.0, 1.0))
            .AddAttribute("OrbitalPeriod",
                          "Orbital period in seconds",
                          DoubleValue(5400.0),
                          MakeDoubleAccessor(&OranNtnXappEnergyHarvest::m_orbitalPeriod_s),
                          MakeDoubleChecker<double>(0.0))
            .AddAttribute("BatteryCapacity",
                          "Battery capacity in Wh",
                          DoubleValue(500.0),
                          MakeDoubleAccessor(&OranNtnXappEnergyHarvest::m_batteryCapacity_Wh),
                          MakeDoubleChecker<double>(0.0))
            .AddAttribute("InitialSoC",
                          "Initial state of charge (0-1)",
                          DoubleValue(0.9),
                          MakeDoubleAccessor(&OranNtnXappEnergyHarvest::m_initialSoC),
                          MakeDoubleChecker<double>(0.0, 1.0))
            .AddAttribute("MinSoC",
                          "Minimum allowed SoC before energy saving kicks in",
                          DoubleValue(0.3),
                          MakeDoubleAccessor(&OranNtnXappEnergyHarvest::m_minSoC),
                          MakeDoubleChecker<double>(0.0, 1.0))
            .AddAttribute("CriticalSoC",
                          "Critical SoC threshold for aggressive saving",
                          DoubleValue(0.15),
                          MakeDoubleAccessor(&OranNtnXappEnergyHarvest::m_criticalSoC),
                          MakeDoubleChecker<double>(0.0, 1.0))
            .AddAttribute("BeamPowerConsumption",
                          "Power consumption per active beam in watts",
                          DoubleValue(15.0),
                          MakeDoubleAccessor(&OranNtnXappEnergyHarvest::m_beamPower_W),
                          MakeDoubleChecker<double>(0.0))
            .AddAttribute("ComputePowerConsumption",
                          "Power consumption per MFLOPS in watts",
                          DoubleValue(0.1),
                          MakeDoubleAccessor(&OranNtnXappEnergyHarvest::m_computePower_W),
                          MakeDoubleChecker<double>(0.0))
            .AddAttribute("BasePowerConsumption",
                          "Baseline satellite power consumption in watts",
                          DoubleValue(50.0),
                          MakeDoubleAccessor(&OranNtnXappEnergyHarvest::m_basePower_W),
                          MakeDoubleChecker<double>(0.0));
    return tid;
}

OranNtnXappEnergyHarvest::OranNtnXappEnergyHarvest()
    : m_solarPanelArea_m2(10.0),
      m_solarEfficiency(0.30),
      m_orbitalPeriod_s(5400.0),
      m_solarConstant_W_m2(1361.0),
      m_batteryCapacity_Wh(500.0),
      m_initialSoC(0.9),
      m_minSoC(0.3),
      m_criticalSoC(0.15),
      m_beamPower_W(15.0),
      m_computePower_W(0.1),
      m_basePower_W(50.0)
{
    NS_LOG_FUNCTION(this);
    SetXappName("EnergyHarvest");

    m_energyMetrics = {};
}

OranNtnXappEnergyHarvest::~OranNtnXappEnergyHarvest()
{
    NS_LOG_FUNCTION(this);
}

// --------------------------------------------------------------------------
//  Configuration setters
// --------------------------------------------------------------------------

void
OranNtnXappEnergyHarvest::SetSatBridge(Ptr<OranNtnSatBridge> bridge)
{
    NS_LOG_FUNCTION(this << bridge);
    m_satBridge = bridge;
}

void
OranNtnXappEnergyHarvest::SetSolarPanelArea(double area_m2)
{
    m_solarPanelArea_m2 = area_m2;
}

void
OranNtnXappEnergyHarvest::SetSolarEfficiency(double efficiency)
{
    m_solarEfficiency = efficiency;
}

void
OranNtnXappEnergyHarvest::SetOrbitalPeriod(double period_s)
{
    m_orbitalPeriod_s = period_s;
}

void
OranNtnXappEnergyHarvest::SetBatteryCapacity(double capacity_Wh)
{
    m_batteryCapacity_Wh = capacity_Wh;
}

void
OranNtnXappEnergyHarvest::SetInitialSoC(double soc)
{
    m_initialSoC = soc;
}

void
OranNtnXappEnergyHarvest::SetMinSoC(double minSoc)
{
    m_minSoC = minSoc;
}

void
OranNtnXappEnergyHarvest::SetCriticalSoC(double critSoc)
{
    m_criticalSoC = critSoc;
}

void
OranNtnXappEnergyHarvest::SetBeamPowerConsumption(double perBeam_W)
{
    m_beamPower_W = perBeam_W;
}

void
OranNtnXappEnergyHarvest::SetComputePowerConsumption(double perMflops_W)
{
    m_computePower_W = perMflops_W;
}

void
OranNtnXappEnergyHarvest::SetBasePowerConsumption(double base_W)
{
    m_basePower_W = base_W;
}

// --------------------------------------------------------------------------
//  Public energy queries
// --------------------------------------------------------------------------

SatelliteEnergyState
OranNtnXappEnergyHarvest::GetEnergyState(uint32_t satId) const
{
    auto it = m_energyStates.find(satId);
    if (it != m_energyStates.end())
    {
        return it->second;
    }
    SatelliteEnergyState empty = {};
    empty.satId = satId;
    return empty;
}

double
OranNtnXappEnergyHarvest::GetCurrentSolarPower(uint32_t satId) const
{
    auto it = m_energyStates.find(satId);
    if (it != m_energyStates.end())
    {
        return it->second.currentSolarPower_W;
    }
    return 0.0;
}

double
OranNtnXappEnergyHarvest::GetBatteryStateOfCharge(uint32_t satId) const
{
    auto it = m_energyStates.find(satId);
    if (it != m_energyStates.end())
    {
        return it->second.batteryLevel_Wh / it->second.batteryCapacity_Wh;
    }
    return m_initialSoC;
}

bool
OranNtnXappEnergyHarvest::IsInEclipse(uint32_t satId) const
{
    return DetectEclipse(satId);
}

double
OranNtnXappEnergyHarvest::GetTimeToEclipse(uint32_t satId) const
{
    return PredictEclipseTime(satId);
}

double
OranNtnXappEnergyHarvest::GetEclipseDuration(uint32_t satId) const
{
    // LEO eclipse duration is approximately 35% of orbital period for typical inclinations
    auto it = m_energyStates.find(satId);
    if (it != m_energyStates.end())
    {
        return it->second.eclipseDuration_s;
    }
    return m_orbitalPeriod_s * 0.35;
}

OranNtnXappEnergyHarvest::EnergyMetrics
OranNtnXappEnergyHarvest::GetEnergyMetrics() const
{
    return m_energyMetrics;
}

// --------------------------------------------------------------------------
//  E2 subscription
// --------------------------------------------------------------------------

E2Subscription
OranNtnXappEnergyHarvest::GetRequiredSubscription() const
{
    E2Subscription sub;
    sub.subscriptionId = 0; // Assigned by E2 termination
    sub.ricRequestorId = GetXappId();
    sub.ranFunctionId = 2;  // KPM service model
    sub.reportingPeriod = MilliSeconds(200);
    sub.eventTrigger = false; // Periodic reporting
    sub.eventThreshold = 0.0;
    sub.batchOnVisibility = false;
    sub.maxBufferAge = Seconds(5);
    sub.useIslRelay = true;
    return sub;
}

// --------------------------------------------------------------------------
//  ProcessKpmReport -- update beam loads per satellite from KPM
// --------------------------------------------------------------------------

void
OranNtnXappEnergyHarvest::ProcessKpmReport(const E2KpmReport& report)
{
    NS_LOG_FUNCTION(this << report.gnbId << report.beamId);

    uint32_t satId = report.gnbId;

    // Initialize energy state for this satellite if not yet tracked
    if (m_energyStates.find(satId) == m_energyStates.end())
    {
        SatelliteEnergyState state;
        state.satId = satId;
        state.timestamp = report.timestamp;
        state.batteryCapacity_Wh = m_batteryCapacity_Wh;
        state.batteryLevel_Wh = m_initialSoC * m_batteryCapacity_Wh;
        state.solarPanelArea_m2 = m_solarPanelArea_m2;
        state.solarEfficiency = m_solarEfficiency;
        state.currentSolarPower_W = 0.0;
        state.currentConsumption_W = m_basePower_W;
        state.inEclipse = report.inEclipse;
        state.timeToEclipse_s = 0.0;
        state.eclipseDuration_s = m_orbitalPeriod_s * 0.35;
        state.activeBeams = 0;
        state.computeLoad = 0.0;
        m_energyStates[satId] = state;
    }

    // Update energy state from KPM fields
    auto& state = m_energyStates[satId];
    state.timestamp = report.timestamp;
    state.inEclipse = report.inEclipse;

    if (report.solarPower_W > 0.0)
    {
        state.currentSolarPower_W = report.solarPower_W;
    }

    if (report.batteryStateOfCharge > 0.0)
    {
        state.batteryLevel_Wh = report.batteryStateOfCharge * m_batteryCapacity_Wh;
    }

    NS_LOG_DEBUG("EnergyHarvest: sat=" << satId << " SoC="
                                        << (state.batteryLevel_Wh / state.batteryCapacity_Wh)
                                        << " eclipse=" << state.inEclipse
                                        << " solar=" << state.currentSolarPower_W << "W");
}

// --------------------------------------------------------------------------
//  DecisionCycle
// --------------------------------------------------------------------------

void
OranNtnXappEnergyHarvest::DecisionCycle()
{
    NS_LOG_FUNCTION(this);

    if (!m_satBridge)
    {
        NS_LOG_WARN("EnergyHarvest: no SatBridge configured, skipping cycle");
        return;
    }

    // Step 1: Update energy states for all tracked satellites
    UpdateEnergyStates();

    // Step 2: For each satellite, check battery levels and take action
    double sumSoC = 0.0;
    double minSoC = 1.0;
    uint32_t numSats = 0;

    for (auto& kv : m_energyStates)
    {
        uint32_t satId = kv.first;
        auto& state = kv.second;

        double soc = state.batteryLevel_Wh / state.batteryCapacity_Wh;
        sumSoC += soc;
        minSoC = std::min(minSoC, soc);
        numSats++;

        // Check if energy saving is needed
        if (soc < m_minSoC)
        {
            NS_LOG_INFO("EnergyHarvest: sat" << satId << " SoC=" << soc
                                              << " below minSoC=" << m_minSoC);
            ApplyEnergySavingActions(satId);
        }

        if (soc < m_criticalSoC)
        {
            m_energyMetrics.criticalEnergyEvents++;
            NS_LOG_WARN("EnergyHarvest: sat" << satId << " CRITICAL SoC=" << soc);
        }

        // Step 3: Proactive eclipse preparation
        ProactiveEclipsePreparation(satId);
    }

    // Update aggregate metrics
    if (numSats > 0)
    {
        m_energyMetrics.avgBatteryLevel = sumSoC / numSats;
        m_energyMetrics.minBatteryLevel = minSoC;
    }

    RecordDecision(true, 0.9, 0.0);
}

// --------------------------------------------------------------------------
//  UpdateEnergyStates -- solar power computation and battery model
// --------------------------------------------------------------------------

void
OranNtnXappEnergyHarvest::UpdateEnergyStates()
{
    NS_LOG_FUNCTION(this);

    double dt_s = GetDecisionInterval().GetSeconds();
    double dt_h = dt_s / 3600.0; // convert to hours for Wh computation

    for (auto& kv : m_energyStates)
    {
        uint32_t satId = kv.first;
        auto& state = kv.second;

        // Compute solar power for this satellite
        ComputeSolarPower(satId);

        // Compute total power consumption
        // Count active beams from the sat bridge
        uint32_t activeBeams = 0;
        if (m_satBridge)
        {
            const auto& satState = m_satBridge->GetSatState(satId);
            activeBeams = static_cast<uint32_t>(satState.beamLoads.size());
        }
        state.activeBeams = activeBeams;

        double totalConsumption_W =
            m_basePower_W + (activeBeams * m_beamPower_W) +
            (state.computeLoad * m_computePower_W * 1000.0); // computeLoad * MFLOPS

        state.currentConsumption_W = totalConsumption_W;

        // Battery model: SoC += (solarPower - consumption) * dt / capacity
        double netPower_W = state.currentSolarPower_W - totalConsumption_W;
        double energyDelta_Wh = netPower_W * dt_h;

        state.batteryLevel_Wh += energyDelta_Wh;

        // Clamp battery level to [0, capacity]
        state.batteryLevel_Wh =
            std::max(0.0, std::min(state.batteryLevel_Wh, state.batteryCapacity_Wh));

        // Track energy harvested and consumed
        if (state.currentSolarPower_W > 0.0)
        {
            m_energyMetrics.totalEnergyHarvested_Wh += state.currentSolarPower_W * dt_h;
        }
        m_energyMetrics.totalEnergyConsumed_Wh += totalConsumption_W * dt_h;

        state.timestamp = Simulator::Now().GetSeconds();

        NS_LOG_DEBUG("EnergyState: sat" << satId << " solar=" << state.currentSolarPower_W
                                        << "W consumption=" << totalConsumption_W
                                        << "W battery=" << state.batteryLevel_Wh << "/"
                                        << state.batteryCapacity_Wh << "Wh");
    }
}

// --------------------------------------------------------------------------
//  ComputeSolarPower -- orbit-based solar irradiance model
// --------------------------------------------------------------------------

void
OranNtnXappEnergyHarvest::ComputeSolarPower(uint32_t satId)
{
    NS_LOG_FUNCTION(this << satId);

    auto it = m_energyStates.find(satId);
    if (it == m_energyStates.end())
    {
        return;
    }
    auto& state = it->second;

    // Check eclipse condition first -- zero solar power in eclipse
    if (DetectEclipse(satId))
    {
        state.currentSolarPower_W = 0.0;
        if (!state.inEclipse)
        {
            state.inEclipse = true;
            m_energyMetrics.eclipseTransitions++;
            NS_LOG_INFO("EnergyHarvest: sat" << satId << " entered eclipse");
        }
        return;
    }

    if (state.inEclipse)
    {
        state.inEclipse = false;
        NS_LOG_INFO("EnergyHarvest: sat" << satId << " exited eclipse");
    }

    // Solar power = solarConstant * area * efficiency * cos(incidenceAngle)
    // Incidence angle depends on satellite orientation relative to the Sun
    // Model: use orbital position to compute sun angle
    double simTime = Simulator::Now().GetSeconds();
    double orbitalPhase = std::fmod(simTime, m_orbitalPeriod_s) / m_orbitalPeriod_s;

    // Sun incidence angle varies sinusoidally with orbital phase
    // At phase=0.0, sun is directly overhead; at phase=0.5, maximum incidence
    double incidenceAngle_rad = std::acos(std::cos(2.0 * M_PI * orbitalPhase) * 0.8);

    // Clamp incidence angle to [0, pi/2] for cosine computation
    if (incidenceAngle_rad > M_PI / 2.0)
    {
        incidenceAngle_rad = M_PI / 2.0;
    }

    double cosIncidence = std::cos(incidenceAngle_rad);

    // Power = solarConstant * area * efficiency * cos(incidence)
    state.currentSolarPower_W =
        m_solarConstant_W_m2 * m_solarPanelArea_m2 * m_solarEfficiency * cosIncidence;

    // Ensure non-negative
    state.currentSolarPower_W = std::max(0.0, state.currentSolarPower_W);

    NS_LOG_DEBUG("SolarPower: sat" << satId << " phase=" << orbitalPhase
                                   << " incidence=" << (incidenceAngle_rad * 180.0 / M_PI)
                                   << "deg power=" << state.currentSolarPower_W << "W");
}

// --------------------------------------------------------------------------
//  DetectEclipse -- Earth shadow cone detection
// --------------------------------------------------------------------------

bool
OranNtnXappEnergyHarvest::DetectEclipse(uint32_t satId) const
{
    NS_LOG_FUNCTION(this << satId);

    if (!m_satBridge)
    {
        return false;
    }

    GeoCoordinate satPos = m_satBridge->GetSatellitePosition(satId);

    // Earth radius in km
    double earthRadius_km = 6371.0;

    // Satellite altitude from position (bridge provides GeoCoordinate with altitude)
    double satAltitude_km = satPos.GetAltitude() / 1000.0; // convert m to km
    if (satAltitude_km < 100.0)
    {
        satAltitude_km = 600.0; // fallback LEO altitude
    }

    double satDistance_km = earthRadius_km + satAltitude_km;

    // Eclipse detection using Earth shadow cone geometry:
    // The satellite is in eclipse if its orbital position places it within
    // the cylindrical shadow approximation of the Earth.
    //
    // Simplified model: compute the angle between the satellite-to-Sun vector
    // and the satellite-to-Earth-center vector. If this angle is less than
    // arcsin(R_earth / sat_distance), the satellite is in Earth's shadow.

    // Sun direction: approximate the Sun as being along +X in ECI frame.
    // Map simulation time to sun angle (24h = full rotation).
    double simTime = Simulator::Now().GetSeconds();
    double sunAngle_rad = 2.0 * M_PI * simTime / 86400.0; // once per day

    // Satellite position angle from lat/lon (approximate ECI angle)
    double satAngle_rad =
        satPos.GetLongitude() * M_PI / 180.0 +
        2.0 * M_PI * std::fmod(simTime, m_orbitalPeriod_s) / m_orbitalPeriod_s;

    // Angle between satellite position and sun direction
    double angleSatSun = std::fmod(std::abs(satAngle_rad - sunAngle_rad), 2.0 * M_PI);
    if (angleSatSun > M_PI)
    {
        angleSatSun = 2.0 * M_PI - angleSatSun;
    }

    // Shadow cone half-angle
    double shadowHalfAngle_rad = std::asin(earthRadius_km / satDistance_km);

    // The satellite is in eclipse if it's on the anti-sun side and within
    // the shadow cone. This occurs when the satellite is roughly opposite
    // the Sun (angle ~ pi) and within the shadow cone width.
    bool inEclipse = (angleSatSun > (M_PI - shadowHalfAngle_rad));

    NS_LOG_DEBUG("Eclipse: sat" << satId << " angleSatSun=" << (angleSatSun * 180.0 / M_PI)
                                << "deg shadowHalf=" << (shadowHalfAngle_rad * 180.0 / M_PI)
                                << "deg inEclipse=" << inEclipse);

    return inEclipse;
}

// --------------------------------------------------------------------------
//  PredictEclipseTime -- time until next eclipse entry
// --------------------------------------------------------------------------

double
OranNtnXappEnergyHarvest::PredictEclipseTime(uint32_t satId) const
{
    NS_LOG_FUNCTION(this << satId);

    if (DetectEclipse(satId))
    {
        return 0.0; // already in eclipse
    }

    // Estimate time to next eclipse from orbital mechanics
    // Eclipse occurs once per orbit when satellite passes through Earth's shadow
    double simTime = Simulator::Now().GetSeconds();
    double orbitalPhase = std::fmod(simTime, m_orbitalPeriod_s) / m_orbitalPeriod_s;

    // Eclipse typically starts around phase 0.65-0.70 for sun-synchronous orbits
    double eclipseStartPhase = 0.67;
    double phaseToEclipse = eclipseStartPhase - orbitalPhase;
    if (phaseToEclipse < 0.0)
    {
        phaseToEclipse += 1.0; // next orbit
    }

    double timeToEclipse_s = phaseToEclipse * m_orbitalPeriod_s;
    return timeToEclipse_s;
}

// --------------------------------------------------------------------------
//  RankBeamsForShutdown -- sort beams by shutdown priority
// --------------------------------------------------------------------------

std::vector<uint32_t>
OranNtnXappEnergyHarvest::RankBeamsForShutdown(uint32_t satId) const
{
    NS_LOG_FUNCTION(this << satId);

    if (!m_satBridge)
    {
        return {};
    }

    const auto& satState = m_satBridge->GetSatState(satId);

    // Collect beam info: (beamId, traffic_load, slice_priority)
    struct BeamInfo
    {
        uint32_t beamId;
        double trafficLoad;
        uint8_t slicePriority; // lower = higher priority (URLLC = 0)
    };

    std::vector<BeamInfo> beams;
    beams.reserve(satState.beamLoads.size());

    for (const auto& kv : satState.beamLoads)
    {
        BeamInfo info;
        info.beamId = kv.first;
        info.trafficLoad = kv.second; // PRB utilization 0-1

        // Determine slice priority from recent KPM data
        // Default to eMBB (medium priority) if unknown
        info.slicePriority = 128;

        // Check KPM database for slice info on this beam
        auto dbIt = m_kpmDatabase.find(satId);
        if (dbIt != m_kpmDatabase.end() && !dbIt->second.empty())
        {
            for (auto rit = dbIt->second.rbegin(); rit != dbIt->second.rend(); ++rit)
            {
                if (rit->beamId == kv.first)
                {
                    // sliceId: 0=eMBB, 1=URLLC, 2=mMTC
                    // URLLC gets highest protection (priority=0), mMTC lowest
                    if (rit->sliceId == 1)
                    {
                        info.slicePriority = 0; // URLLC - never shutdown first
                    }
                    else if (rit->sliceId == 0)
                    {
                        info.slicePriority = 128; // eMBB - medium priority
                    }
                    else
                    {
                        info.slicePriority = 255; // mMTC - lowest priority
                    }
                    break;
                }
            }
        }

        beams.push_back(info);
    }

    // Sort by composite score: low traffic + low priority beams first
    // Score = trafficLoad * (256 - slicePriority) -- higher score = more important
    std::sort(beams.begin(), beams.end(), [](const BeamInfo& a, const BeamInfo& b) {
        double scoreA = a.trafficLoad * (256.0 - static_cast<double>(a.slicePriority));
        double scoreB = b.trafficLoad * (256.0 - static_cast<double>(b.slicePriority));
        return scoreA < scoreB; // ascending: least important first
    });

    std::vector<uint32_t> ranked;
    ranked.reserve(beams.size());
    for (const auto& b : beams)
    {
        ranked.push_back(b.beamId);
    }

    return ranked;
}

// --------------------------------------------------------------------------
//  ApplyEnergySavingActions -- beam shutdown and compute throttle
// --------------------------------------------------------------------------

void
OranNtnXappEnergyHarvest::ApplyEnergySavingActions(uint32_t satId)
{
    NS_LOG_FUNCTION(this << satId);

    auto it = m_energyStates.find(satId);
    if (it == m_energyStates.end())
    {
        return;
    }

    double soc = it->second.batteryLevel_Wh / it->second.batteryCapacity_Wh;

    // Get beams ranked by shutdown priority (least important first)
    auto rankedBeams = RankBeamsForShutdown(satId);

    if (soc < m_minSoC)
    {
        // Shutdown lowest-priority beams to save power
        // Shutdown up to 30% of beams when below minSoC
        uint32_t beamsToShutdown =
            std::max(1u, static_cast<uint32_t>(rankedBeams.size() * 0.3));

        for (uint32_t i = 0; i < beamsToShutdown && i < rankedBeams.size(); ++i)
        {
            E2RcAction action = BuildAction(E2RcActionType::BEAM_SHUTDOWN,
                                            satId,
                                            0, // cell-wide
                                            0.85);
            action.targetBeamId = rankedBeams[i];
            action.parameter1 = soc;       // current SoC for logging
            action.parameter2 = m_minSoC;  // threshold that triggered this

            bool accepted = SubmitAction(action);
            if (accepted)
            {
                m_energyMetrics.beamShutdowns++;
                NS_LOG_INFO("EnergyHarvest: shutdown beam " << rankedBeams[i]
                                                             << " on sat" << satId
                                                             << " (SoC=" << soc << ")");
            }
        }
    }

    if (soc < m_criticalSoC)
    {
        // Critical: also throttle compute (Space RIC processing)
        E2RcAction action = BuildAction(E2RcActionType::COMPUTE_THROTTLE,
                                        satId,
                                        0,
                                        0.95);
        action.parameter1 = 0.5; // throttle to 50% compute capacity
        action.parameter2 = soc;

        bool accepted = SubmitAction(action);
        if (accepted)
        {
            m_energyMetrics.computeThrottles++;
            NS_LOG_INFO("EnergyHarvest: throttle compute on sat" << satId
                                                                  << " (critical SoC=" << soc
                                                                  << ")");
        }

        // Shutdown more beams aggressively -- up to 60%
        uint32_t extraShutdowns =
            std::max(1u, static_cast<uint32_t>(rankedBeams.size() * 0.6));
        uint32_t alreadyShut =
            std::max(1u, static_cast<uint32_t>(rankedBeams.size() * 0.3));

        for (uint32_t i = alreadyShut; i < extraShutdowns && i < rankedBeams.size(); ++i)
        {
            E2RcAction shutAction = BuildAction(E2RcActionType::BEAM_SHUTDOWN,
                                                satId,
                                                0,
                                                0.95);
            shutAction.targetBeamId = rankedBeams[i];
            shutAction.parameter1 = soc;
            shutAction.parameter2 = m_criticalSoC;

            bool accepted2 = SubmitAction(shutAction);
            if (accepted2)
            {
                m_energyMetrics.beamShutdowns++;
            }
        }
    }
}

// --------------------------------------------------------------------------
//  ProactiveEclipsePreparation -- preemptive shutdowns before eclipse
// --------------------------------------------------------------------------

void
OranNtnXappEnergyHarvest::ProactiveEclipsePreparation(uint32_t satId)
{
    NS_LOG_FUNCTION(this << satId);

    auto it = m_energyStates.find(satId);
    if (it == m_energyStates.end())
    {
        return;
    }

    auto& state = it->second;

    // Predict time to next eclipse
    double timeToEclipse_s = PredictEclipseTime(satId);
    state.timeToEclipse_s = timeToEclipse_s;

    // If already in eclipse, nothing to prepare
    if (state.inEclipse)
    {
        return;
    }

    // Preparation threshold: 5 minutes (300 seconds) before eclipse
    double preparationThreshold_s = 300.0;

    if (timeToEclipse_s > preparationThreshold_s)
    {
        return; // too far from eclipse, no preparation needed
    }

    // Estimate if battery will last through the eclipse
    double eclipseDuration_s = GetEclipseDuration(satId);
    double eclipseDuration_h = eclipseDuration_s / 3600.0;
    double energyNeeded_Wh = state.currentConsumption_W * eclipseDuration_h;
    double energyAvailable_Wh = state.batteryLevel_Wh;

    if (energyAvailable_Wh >= energyNeeded_Wh * 1.1) // 10% margin
    {
        NS_LOG_DEBUG("EnergyHarvest: sat" << satId << " has sufficient energy for eclipse ("
                                          << energyAvailable_Wh << " >= "
                                          << energyNeeded_Wh << " Wh)");
        return;
    }

    // Not enough energy -- start preemptive beam shutdowns
    NS_LOG_INFO("EnergyHarvest: sat" << satId << " proactive eclipse prep: "
                                     << timeToEclipse_s << "s to eclipse, need "
                                     << energyNeeded_Wh << "Wh, have "
                                     << energyAvailable_Wh << "Wh");

    // Calculate how many beams to shutdown to survive eclipse
    double deficit_Wh = energyNeeded_Wh * 1.1 - energyAvailable_Wh;
    double savingsPerBeam_Wh = m_beamPower_W * eclipseDuration_h;
    uint32_t beamsToShutdown = 0;
    if (savingsPerBeam_Wh > 0.0)
    {
        beamsToShutdown =
            static_cast<uint32_t>(std::ceil(deficit_Wh / savingsPerBeam_Wh));
    }

    auto rankedBeams = RankBeamsForShutdown(satId);
    beamsToShutdown = std::min(beamsToShutdown, static_cast<uint32_t>(rankedBeams.size()));

    for (uint32_t i = 0; i < beamsToShutdown; ++i)
    {
        E2RcAction action = BuildAction(E2RcActionType::BEAM_SHUTDOWN,
                                        satId,
                                        0,
                                        0.80);
        action.targetBeamId = rankedBeams[i];
        action.parameter1 = timeToEclipse_s; // time to eclipse
        action.parameter2 = deficit_Wh;      // energy deficit

        bool accepted = SubmitAction(action);
        if (accepted)
        {
            m_energyMetrics.beamShutdowns++;
            NS_LOG_INFO("EnergyHarvest: preemptive shutdown beam "
                        << rankedBeams[i] << " on sat" << satId
                        << " (eclipse in " << timeToEclipse_s << "s)");
        }
    }
}

} // namespace ns3
