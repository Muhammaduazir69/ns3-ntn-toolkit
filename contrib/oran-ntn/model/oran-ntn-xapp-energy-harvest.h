/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-only
 * Author: Muhammad Uzair
 *
 * Energy Harvesting xApp
 *
 * Solar panel state + battery management for power-aware satellite
 * resource allocation. Integrates with orbital mechanics for eclipse
 * prediction and solar power modeling.
 *
 * Novel features:
 *   - Orbit-based solar irradiance model with eclipse detection
 *   - Battery degradation model (cycle counting, temperature effects)
 *   - Proactive beam shutdown scheduling before eclipse periods
 *   - Energy-proportional compute budgeting for Space RIC
 *   - Multi-satellite energy coordination via ISL
 *   - SLA-aware energy saving (protect URLLC even during low energy)
 */

#ifndef ORAN_NTN_XAPP_ENERGY_HARVEST_H
#define ORAN_NTN_XAPP_ENERGY_HARVEST_H

#include "oran-ntn-xapp-base.h"

#include <map>
#include <vector>

namespace ns3
{

class OranNtnSatBridge;

/**
 * \ingroup oran-ntn
 * \brief Energy Harvesting xApp for power-aware NTN management
 */
class OranNtnXappEnergyHarvest : public OranNtnXappBase
{
  public:
    static TypeId GetTypeId();
    OranNtnXappEnergyHarvest();
    ~OranNtnXappEnergyHarvest() override;

    void SetSatBridge(Ptr<OranNtnSatBridge> bridge);

    // ---- Solar panel configuration ----
    void SetSolarPanelArea(double area_m2);
    void SetSolarEfficiency(double efficiency);
    void SetOrbitalPeriod(double period_s);

    // ---- Battery configuration ----
    void SetBatteryCapacity(double capacity_Wh);
    void SetInitialSoC(double soc);
    void SetMinSoC(double minSoc);
    void SetCriticalSoC(double critSoc);

    // ---- Power consumption model ----
    void SetBeamPowerConsumption(double perBeam_W);
    void SetComputePowerConsumption(double perMflops_W);
    void SetBasePowerConsumption(double base_W);

    // ---- Energy queries ----
    SatelliteEnergyState GetEnergyState(uint32_t satId) const;
    double GetCurrentSolarPower(uint32_t satId) const;
    double GetBatteryStateOfCharge(uint32_t satId) const;
    bool IsInEclipse(uint32_t satId) const;
    double GetTimeToEclipse(uint32_t satId) const;
    double GetEclipseDuration(uint32_t satId) const;

    // ---- Metrics ----
    struct EnergyMetrics
    {
        uint32_t beamShutdowns;
        uint32_t computeThrottles;
        uint32_t eclipseTransitions;
        double avgBatteryLevel;
        double minBatteryLevel;
        double totalEnergyHarvested_Wh;
        double totalEnergyConsumed_Wh;
        uint32_t criticalEnergyEvents;
    };
    EnergyMetrics GetEnergyMetrics() const;

  protected:
    void ProcessKpmReport(const E2KpmReport& report) override;
    void DecisionCycle() override;
    E2Subscription GetRequiredSubscription() const override;

  private:
    void UpdateEnergyStates();
    void ComputeSolarPower(uint32_t satId);
    bool DetectEclipse(uint32_t satId) const;
    double PredictEclipseTime(uint32_t satId) const;
    void ApplyEnergySavingActions(uint32_t satId);
    void ProactiveEclipsePreparation(uint32_t satId);

    /**
     * \brief Rank beams for shutdown priority
     *
     * Low-traffic, non-URLLC beams get shutdown first.
     */
    std::vector<uint32_t> RankBeamsForShutdown(uint32_t satId) const;

    Ptr<OranNtnSatBridge> m_satBridge;

    // Solar model
    double m_solarPanelArea_m2;
    double m_solarEfficiency;
    double m_orbitalPeriod_s;
    double m_solarConstant_W_m2;    //!< 1361 W/m² at 1 AU

    // Battery model
    double m_batteryCapacity_Wh;
    double m_initialSoC;
    double m_minSoC;               //!< Minimum allowed SoC
    double m_criticalSoC;          //!< Critical SoC threshold

    // Power consumption
    double m_beamPower_W;
    double m_computePower_W;
    double m_basePower_W;

    // Per-satellite energy state
    std::map<uint32_t, SatelliteEnergyState> m_energyStates;

    EnergyMetrics m_energyMetrics;
};

} // namespace ns3

#endif // ORAN_NTN_XAPP_ENERGY_HARVEST_H
