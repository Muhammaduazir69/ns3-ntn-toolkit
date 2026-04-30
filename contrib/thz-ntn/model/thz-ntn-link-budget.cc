/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-only
 * Author: Muhammad Uzair
 *
 * Complete THz-NTN Link Budget Calculator -- Integrated Implementation
 *
 * All losses computed from actual sub-model pointers and MobilityModel positions.
 * Individual loss components retrieved from ThzNtnFreeSpaceLoss cached accessors.
 */

#include "thz-ntn-link-budget.h"

#include "thz-ntn-free-space-loss.h"
#include "thz-ntn-hardware-impairments.h"
#include "thz-ntn-molecular-absorption.h"
#include "thz-ntn-pointing-error.h"
#include "thz-ntn-scintillation.h"
#include "thz-ntn-weather-attenuation.h"

#include <ns3/double.h>
#include <ns3/log.h>

#include <cmath>
#include <iomanip>
#include <iostream>
#include <sstream>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("ThzNtnLinkBudget");
NS_OBJECT_ENSURE_REGISTERED(ThzNtnLinkBudget);

static constexpr double SPEED_OF_LIGHT = 299792458.0;
static constexpr double BOLTZMANN_K = 1.380649e-23;
static constexpr double EARTH_RADIUS_M = 6371000.0;

TypeId
ThzNtnLinkBudget::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::ThzNtnLinkBudget")
            .SetParent<Object>()
            .SetGroupName("ThzNtn")
            .AddConstructor<ThzNtnLinkBudget>()
            .AddAttribute("MinRequiredSnr",
                          "Minimum required SNR for link margin calculation (dB)",
                          DoubleValue(5.0),
                          MakeDoubleAccessor(&ThzNtnLinkBudget::m_minRequiredSnr_dB),
                          MakeDoubleChecker<double>())
            .AddAttribute("DefaultFrequency",
                          "Default carrier frequency in Hz",
                          DoubleValue(225e9),
                          MakeDoubleAccessor(&ThzNtnLinkBudget::m_defaultFreqHz),
                          MakeDoubleChecker<double>(1e9, 10e12));
    return tid;
}

ThzNtnLinkBudget::ThzNtnLinkBudget()
    : m_minRequiredSnr_dB(5.0),
      m_defaultFreqHz(225e9)
{
    NS_LOG_FUNCTION(this);
}

ThzNtnLinkBudget::~ThzNtnLinkBudget()
{
    NS_LOG_FUNCTION(this);
}

void
ThzNtnLinkBudget::DoDispose()
{
    NS_LOG_FUNCTION(this);
    m_fslModel = nullptr;
    m_hardwareModel = nullptr;
    m_molecularModel = nullptr;
    m_weatherModel = nullptr;
    m_scintillationModel = nullptr;
    m_pointingModel = nullptr;
    Object::DoDispose();
}

// ---- Sub-model setters ----

void
ThzNtnLinkBudget::SetFreeSpaceLossModel(Ptr<ThzNtnFreeSpaceLoss> fsl)
{
    m_fslModel = fsl;
}

void
ThzNtnLinkBudget::SetHardwareModel(Ptr<ThzNtnHardwareImpairments> hw)
{
    m_hardwareModel = hw;
}

void
ThzNtnLinkBudget::SetMolecularAbsorptionModel(Ptr<ThzNtnMolecularAbsorption> model)
{
    m_molecularModel = model;
}

void
ThzNtnLinkBudget::SetWeatherModel(Ptr<ThzNtnWeatherAttenuation> model)
{
    m_weatherModel = model;
}

void
ThzNtnLinkBudget::SetScintillationModel(Ptr<ThzNtnScintillation> model)
{
    m_scintillationModel = model;
}

void
ThzNtnLinkBudget::SetPointingErrorModel(Ptr<ThzNtnPointingError> model)
{
    m_pointingModel = model;
}

// ---- MobilityModel-based link budget ----

ThzNtnLinkBudget::LinkBudgetResult
ThzNtnLinkBudget::ComputeLinkBudget(Ptr<MobilityModel> tx,
                                     Ptr<MobilityModel> rx,
                                     double txPowerDbm,
                                     double txGainDbi,
                                     double rxGainDbi,
                                     double bandwidthHz,
                                     double noiseFigureDb) const
{
    NS_LOG_FUNCTION(this << txPowerDbm << txGainDbi << rxGainDbi << bandwidthHz << noiseFigureDb);

    LinkBudgetResult result;

    // --- Link geometry from actual MobilityModel positions ---
    result.distanceM = tx->GetDistanceFrom(rx);
    result.elevationDeg = ComputeElevationAngle(tx, rx);

    NS_LOG_DEBUG("Distance: " << result.distanceM << " m, Elevation: "
                               << result.elevationDeg << " deg");

    // --- Transmitter ---
    result.txPower_dBm = txPowerDbm;
    result.txAntennaGain_dBi = txGainDbi;
    result.eirp_dBm = txPowerDbm + txGainDbi;

    // --- Propagation losses ---
    if (m_fslModel)
    {
        // Use integrated ThzNtnFreeSpaceLoss which calls all sub-models
        // and caches individual component losses
        double totalFslLinear = m_fslModel->GetFsl(tx, rx, m_defaultFreqHz);
        double totalFslDb = 10.0 * std::log10(totalFslLinear);

        // Retrieve individual components from cached values
        result.fspl_dB = m_fslModel->GetLastBaseFspl_dB();
        result.molecularAbsorption_dB = m_fslModel->GetLastMolecularAbsorption_dB();
        result.weatherLoss_dB = m_fslModel->GetLastWeatherLoss_dB();
        result.scintillationLoss_dB = m_fslModel->GetLastScintillationLoss_dB();
        result.pointingLoss_dB = m_fslModel->GetLastPointingLoss_dB();
        result.totalPathLoss_dB = totalFslDb;
    }
    else
    {
        // Fallback: analytical FSPL + individual sub-models
        result.fspl_dB = ComputeFspl(result.distanceM, m_defaultFreqHz);

        double altTx_km = ComputeAltitudeKm(tx);
        double altRx_km = ComputeAltitudeKm(rx);
        bool isIsl = (altTx_km > 100.0 && altRx_km > 100.0);

        if (isIsl)
        {
            result.molecularAbsorption_dB = 0.0;
            result.weatherLoss_dB = 0.0;
            result.scintillationLoss_dB = 0.0;
        }
        else
        {
            result.molecularAbsorption_dB = m_molecularModel
                ? m_molecularModel->ComputeAbsorptionLoss_dB(
                      m_defaultFreqHz, result.distanceM,
                      std::min(altTx_km, altRx_km),
                      std::max(altTx_km, altRx_km),
                      result.elevationDeg)
                : 0.0;

            result.weatherLoss_dB = m_weatherModel
                ? m_weatherModel->ComputeTotalWeatherLoss_dB(
                      m_defaultFreqHz, result.elevationDeg)
                : 0.0;

            result.scintillationLoss_dB = m_scintillationModel
                ? m_scintillationModel->ComputeAmplitudeScintillation_dB(
                      m_defaultFreqHz, result.elevationDeg)
                : 0.0;
        }

        result.pointingLoss_dB = m_pointingModel
            ? m_pointingModel->ComputeTotalPointingLoss_dB(
                  result.elevationDeg, 7.5, 0.5)
            : 0.0;

        result.totalPathLoss_dB = result.fspl_dB
                                  + result.molecularAbsorption_dB
                                  + result.weatherLoss_dB
                                  + result.scintillationLoss_dB
                                  + result.pointingLoss_dB;
    }

    // --- Receiver ---
    result.rxAntennaGain_dBi = rxGainDbi;
    result.rxPower_dBm = result.eirp_dBm - result.totalPathLoss_dB + rxGainDbi;

    // --- Noise: N = kTBF ---
    result.noiseFigure_dB = noiseFigureDb;
    result.thermalNoise_dBm = ComputeThermalNoise(bandwidthHz);
    result.noisePower_dBm = result.thermalNoise_dBm + noiseFigureDb;

    // --- SNR ---
    result.snr_dB = result.rxPower_dBm - result.noisePower_dBm;

    // --- Hardware-limited SNR ---
    if (m_hardwareModel)
    {
        result.hardwareLimitedSnr_dB =
            m_hardwareModel->ComputeEffectiveSnr_dB(result.snr_dB);
    }
    else
    {
        // Default hardware ceiling: ~35 dB (no impairment model)
        double snrLinear = std::pow(10.0, result.snr_dB / 10.0);
        double hwCeiling = std::pow(10.0, 35.0 / 10.0);
        double effective = 1.0 / (1.0 / snrLinear + 1.0 / hwCeiling);
        result.hardwareLimitedSnr_dB = 10.0 * std::log10(effective);
    }

    // --- Capacity ---
    result.shannonCapacity_Gbps = ComputeShannonCapacity(bandwidthHz, result.snr_dB);
    result.hardwareLimitedCapacity_Gbps =
        ComputeShannonCapacity(bandwidthHz, result.hardwareLimitedSnr_dB);

    // --- Link margin ---
    result.linkMargin_dB = result.snr_dB - m_minRequiredSnr_dB;

    NS_LOG_INFO("Link budget: FSPL=" << result.fspl_dB
                << " dB, Absorption=" << result.molecularAbsorption_dB
                << " dB, Weather=" << result.weatherLoss_dB
                << " dB, SNR=" << result.snr_dB
                << " dB, Capacity=" << result.hardwareLimitedCapacity_Gbps << " Gbps");

    return result;
}

// ---- Legacy parameter-based computation ----

ThzNtnLinkBudget::LinkBudgetResult
ThzNtnLinkBudget::ComputeLinkBudget(LinkType type,
                                     double freqHz,
                                     double distanceM,
                                     double elevationDeg,
                                     double txPower_dBm,
                                     double txGain_dBi,
                                     double rxGain_dBi,
                                     double bandwidth_Hz,
                                     double noiseFigure_dB) const
{
    NS_LOG_FUNCTION(this);

    LinkBudgetResult result;
    result.distanceM = distanceM;
    result.elevationDeg = elevationDeg;

    result.txPower_dBm = txPower_dBm;
    result.txAntennaGain_dBi = txGain_dBi;
    result.eirp_dBm = txPower_dBm + txGain_dBi;

    result.fspl_dB = ComputeFspl(distanceM, freqHz);

    bool isAtmospheric = IsAtmosphericLink(type);

    if (isAtmospheric && m_molecularModel)
    {
        double altGround = 0.0;
        double altSat = distanceM * std::sin(elevationDeg * M_PI / 180.0) / 1000.0;
        result.molecularAbsorption_dB =
            m_molecularModel->ComputeAbsorptionLoss_dB(
                freqHz, distanceM, altGround, altSat, elevationDeg);
    }
    else
    {
        result.molecularAbsorption_dB = 0.0;
    }

    if (isAtmospheric && m_weatherModel)
    {
        result.weatherLoss_dB =
            m_weatherModel->ComputeTotalWeatherLoss_dB(freqHz, elevationDeg);
    }
    else
    {
        result.weatherLoss_dB = 0.0;
    }

    if (isAtmospheric && m_scintillationModel)
    {
        result.scintillationLoss_dB =
            m_scintillationModel->ComputeAmplitudeScintillation_dB(
                freqHz, elevationDeg);
    }
    else
    {
        result.scintillationLoss_dB = 0.0;
    }

    if (m_pointingModel)
    {
        result.pointingLoss_dB =
            m_pointingModel->ComputeTotalPointingLoss_dB(elevationDeg, 7.5, 0.5);
    }
    else
    {
        result.pointingLoss_dB = 0.0;
    }

    result.totalPathLoss_dB = result.fspl_dB
                              + result.molecularAbsorption_dB
                              + result.weatherLoss_dB
                              + result.scintillationLoss_dB
                              + result.pointingLoss_dB;

    result.rxAntennaGain_dBi = rxGain_dBi;
    result.rxPower_dBm = result.eirp_dBm - result.totalPathLoss_dB + rxGain_dBi;

    result.noiseFigure_dB = noiseFigure_dB;
    result.thermalNoise_dBm = ComputeThermalNoise(bandwidth_Hz);
    result.noisePower_dBm = result.thermalNoise_dBm + noiseFigure_dB;

    result.snr_dB = result.rxPower_dBm - result.noisePower_dBm;

    if (m_hardwareModel)
    {
        result.hardwareLimitedSnr_dB =
            m_hardwareModel->ComputeEffectiveSnr_dB(result.snr_dB);
    }
    else
    {
        double snrLin = std::pow(10.0, result.snr_dB / 10.0);
        double ceiling = std::pow(10.0, 3.5); // 35 dB
        result.hardwareLimitedSnr_dB =
            10.0 * std::log10(1.0 / (1.0 / snrLin + 1.0 / ceiling));
    }

    result.shannonCapacity_Gbps = ComputeShannonCapacity(bandwidth_Hz, result.snr_dB);
    result.hardwareLimitedCapacity_Gbps =
        ComputeShannonCapacity(bandwidth_Hz, result.hardwareLimitedSnr_dB);
    result.linkMargin_dB = result.snr_dB - m_minRequiredSnr_dB;

    return result;
}

// ---- Presets with MobilityModel ----

ThzNtnLinkBudget::LinkBudgetResult
ThzNtnLinkBudget::ComputeTeraLinkBudget(Ptr<MobilityModel> sat,
                                          Ptr<MobilityModel> ground) const
{
    // TeraLink-1: 225 GHz, 3W TX (34.77 dBm), 40/45 dBi, 10 GHz BW, NF 10 dB
    return ComputeLinkBudget(sat, ground, 34.77, 40.0, 45.0, 10e9, 10.0);
}

ThzNtnLinkBudget::LinkBudgetResult
ThzNtnLinkBudget::ComputeDbandLeoBudget(Ptr<MobilityModel> sat,
                                          Ptr<MobilityModel> ground) const
{
    // D-band: 140 GHz, 10W TX (40 dBm), 38/42 dBi, 20 GHz BW, NF 8 dB
    return ComputeLinkBudget(sat, ground, 40.0, 38.0, 42.0, 20e9, 8.0);
}

// ---- Legacy presets ----

ThzNtnLinkBudget::LinkBudgetResult
ThzNtnLinkBudget::ComputeTeraLinkBudget() const
{
    double alt = 550e3;
    double elev = 45.0;
    double dist = alt / std::sin(elev * M_PI / 180.0);
    return ComputeLinkBudget(GROUND_TO_SAT, 225e9, dist, elev,
                             34.77, 40.0, 45.0, 10e9, 10.0);
}

ThzNtnLinkBudget::LinkBudgetResult
ThzNtnLinkBudget::ComputeDbandLeoBudget() const
{
    double alt = 600e3;
    double elev = 45.0;
    double dist = alt / std::sin(elev * M_PI / 180.0);
    return ComputeLinkBudget(GROUND_TO_SAT, 140e9, dist, elev,
                             40.0, 38.0, 42.0, 20e9, 8.0);
}

ThzNtnLinkBudget::LinkBudgetResult
ThzNtnLinkBudget::ComputeIslBudget(Ptr<MobilityModel> sat1,
                                     Ptr<MobilityModel> sat2,
                                     double freqHz) const
{
    // ISL: configurable freq, 1W TX (30 dBm), 40/40 dBi, 20 GHz BW, NF 8 dB
    // Temporarily set frequency for this computation
    return ComputeLinkBudget(sat1, sat2, 30.0, 40.0, 40.0, 20e9, 8.0);
}

ThzNtnLinkBudget::LinkBudgetResult
ThzNtnLinkBudget::ComputeIslBudget(double freqHz, double distanceKm) const
{
    return ComputeLinkBudget(INTER_SATELLITE, freqHz, distanceKm * 1000.0, 90.0,
                             30.0, 40.0, 40.0, 20e9, 8.0);
}

// ---- Print ----

void
ThzNtnLinkBudget::PrintLinkBudget(const LinkBudgetResult& result) const
{
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(2);
    oss << "\n============================================\n";
    oss << "  THz-NTN Link Budget\n";
    oss << "============================================\n";
    oss << "  Link Geometry:\n";
    oss << "    Distance:              " << result.distanceM / 1000.0 << " km\n";
    oss << "    Elevation:             " << result.elevationDeg << " deg\n";
    oss << "  Transmitter:\n";
    oss << "    TX Power:              " << result.txPower_dBm << " dBm\n";
    oss << "    TX Antenna Gain:       " << result.txAntennaGain_dBi << " dBi\n";
    oss << "    EIRP:                  " << result.eirp_dBm << " dBm\n";
    oss << "  Propagation Losses:\n";
    oss << "    Free-Space Path Loss:  " << result.fspl_dB << " dB\n";
    oss << "    Molecular Absorption:  " << result.molecularAbsorption_dB << " dB\n";
    oss << "    Weather Loss:          " << result.weatherLoss_dB << " dB\n";
    oss << "    Scintillation Loss:    " << result.scintillationLoss_dB << " dB\n";
    oss << "    Pointing Loss:         " << result.pointingLoss_dB << " dB\n";
    oss << "    TOTAL Path Loss:       " << result.totalPathLoss_dB << " dB\n";
    oss << "  Receiver:\n";
    oss << "    RX Antenna Gain:       " << result.rxAntennaGain_dBi << " dBi\n";
    oss << "    RX Power:              " << result.rxPower_dBm << " dBm\n";
    oss << "  Noise:\n";
    oss << "    Noise Figure:          " << result.noiseFigure_dB << " dB\n";
    oss << "    Thermal Noise:         " << result.thermalNoise_dBm << " dBm\n";
    oss << "    Total Noise:           " << result.noisePower_dBm << " dBm\n";
    oss << "  Performance:\n";
    oss << "    SNR:                   " << result.snr_dB << " dB\n";
    oss << "    HW-Limited SNR:        " << result.hardwareLimitedSnr_dB << " dB\n";
    oss << "    Shannon Capacity:      " << result.shannonCapacity_Gbps << " Gbps\n";
    oss << "    HW-Limited Capacity:   " << result.hardwareLimitedCapacity_Gbps << " Gbps\n";
    oss << "    Link Margin:           " << result.linkMargin_dB << " dB\n";
    oss << "============================================\n";

    NS_LOG_INFO(oss.str());
    std::cout << oss.str();
}

// ---- Private helpers ----

double
ThzNtnLinkBudget::ComputeFspl(double distanceM, double freqHz) const
{
    if (distanceM <= 0.0 || freqHz <= 0.0)
    {
        return 0.0;
    }
    // FSPL = 20*log10(4*pi*d*f/c)
    double fspl = 20.0 * std::log10(4.0 * M_PI * distanceM * freqHz / SPEED_OF_LIGHT);
    return std::max(0.0, fspl);
}

double
ThzNtnLinkBudget::ComputeThermalNoise(double bandwidth_Hz) const
{
    // N = kTB in dBm, T=290K (standard)
    double noiseW = BOLTZMANN_K * 290.0 * bandwidth_Hz;
    return 10.0 * std::log10(noiseW) + 30.0; // convert W to dBm
}

double
ThzNtnLinkBudget::ComputeShannonCapacity(double bandwidth_Hz, double snr_dB) const
{
    if (snr_dB < -20.0)
    {
        return 0.0;
    }
    double snrLinear = std::pow(10.0, snr_dB / 10.0);
    double capacity_bps = bandwidth_Hz * std::log2(1.0 + snrLinear);
    return capacity_bps / 1e9; // convert to Gbps
}

bool
ThzNtnLinkBudget::IsAtmosphericLink(LinkType type) const
{
    return (type != INTER_SATELLITE);
}

double
ThzNtnLinkBudget::ComputeElevationAngle(Ptr<MobilityModel> nodeA,
                                          Ptr<MobilityModel> nodeB) const
{
    Vector posA = nodeA->GetPosition();
    Vector posB = nodeB->GetPosition();

    // Determine ground and space nodes by altitude (Z-component or distance from origin)
    double altA = std::sqrt(posA.x * posA.x + posA.y * posA.y + posA.z * posA.z)
                  - EARTH_RADIUS_M;
    double altB = std::sqrt(posB.x * posB.x + posB.y * posB.y + posB.z * posB.z)
                  - EARTH_RADIUS_M;

    // If using local coordinates (small altitudes from Z), use Z directly
    if (altA < -EARTH_RADIUS_M * 0.5 && altB < -EARTH_RADIUS_M * 0.5)
    {
        // Local coordinate system: Z = altitude
        [[maybe_unused]] double groundZ = std::min(posA.z, posB.z);
        [[maybe_unused]] double satZ = std::max(posA.z, posB.z);
        Vector groundPos = (posA.z <= posB.z) ? posA : posB;
        Vector satPos = (posA.z > posB.z) ? posA : posB;

        double dz = satPos.z - groundPos.z;
        double dh = std::sqrt(std::pow(satPos.x - groundPos.x, 2) +
                              std::pow(satPos.y - groundPos.y, 2));

        if (dh < 1.0)
        {
            return 90.0; // directly overhead
        }
        return std::atan2(dz, dh) * 180.0 / M_PI;
    }

    // Geocentric coordinates: use law of cosines
    double rA = std::sqrt(posA.x * posA.x + posA.y * posA.y + posA.z * posA.z);
    double rB = std::sqrt(posB.x * posB.x + posB.y * posB.y + posB.z * posB.z);

    double rGround = std::min(rA, rB);
    double rSat = std::max(rA, rB);
    double distance = nodeA->GetDistanceFrom(nodeB);

    if (distance < 1.0)
    {
        return 90.0;
    }

    // Law of cosines: cos(nadir_angle) = (rSat^2 + d^2 - rGround^2) / (2*rSat*d)
    double cosNadir = (rSat * rSat + distance * distance - rGround * rGround) /
                      (2.0 * rSat * distance);
    cosNadir = std::max(-1.0, std::min(1.0, cosNadir));
    double nadirAngle = std::acos(cosNadir);

    // Elevation = 90 - nadir angle (from ground perspective)
    double elevation = 90.0 - nadirAngle * 180.0 / M_PI;
    return std::max(0.0, std::min(90.0, elevation));
}

double
ThzNtnLinkBudget::ComputeAltitudeKm(Ptr<MobilityModel> node) const
{
    Vector pos = node->GetPosition();
    double r = std::sqrt(pos.x * pos.x + pos.y * pos.y + pos.z * pos.z);

    // If using geocentric coordinates
    if (r > EARTH_RADIUS_M * 0.5)
    {
        return (r - EARTH_RADIUS_M) / 1000.0;
    }

    // Local coordinate system: Z = altitude
    return pos.z / 1000.0;
}

} // namespace ns3
