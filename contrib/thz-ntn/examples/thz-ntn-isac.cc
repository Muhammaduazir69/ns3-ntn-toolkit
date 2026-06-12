/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-only
 * Author: Muhammad Uzair
 *
 * Example: ISAC Space Debris Detection at THz Frequencies
 *
 * Demonstrates joint Integrated Sensing and Communication (ISAC) for
 * space debris detection using a LEO satellite equipped with a THz
 * UM-MIMO array at 300 GHz.  Evaluates radar detection performance
 * for debris of various sizes and the communication-sensing tradeoff.
 *
 * Analysis-only example: parametric geometry (radar-equation sweeps over
 * debris range), no packet transmission. NOTE: this source is currently
 * excluded from the build (legacy ThzNtnIsac API; see
 * examples/CMakeLists.txt) pending the Q4 2026 ISAC scheduler redesign.
 * For a measured-radio ISAC scenario see thz-ntn-isac-coexist-traffic.
 */

#include <ns3/command-line.h>
#include <ns3/core-module.h>
#include <ns3/mobility-module.h>
#include <ns3/node-container.h>

#include <cmath>
#include <iomanip>
#include <iostream>
#include <vector>

// Forward declarations
namespace ns3
{
class ThzNtnHelper;
class ThzNtnIsac;
class ThzNtnAntennaArray;
} // namespace ns3

#include "ns3/thz-ntn-helper.h"
#include "ns3/thz-ntn-isac.h"
#include "ns3/thz-ntn-antenna-array.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("ThzNtnIsac");

int
main(int argc, char* argv[])
{
    // ---- Default parameters ----
    double freq = 300e9;          // 300 GHz
    double txPower = 30.0;        // dBm (1 W)
    double bandwidth = 10e9;      // 10 GHz
    uint32_t arraySize = 32;      // 32x32 UM-MIMO
    double integrationTime = 1e-3; // 1 ms coherent integration

    // ---- Parse command-line arguments ----
    CommandLine cmd(__FILE__);
    cmd.AddValue("freq", "Carrier frequency in Hz [default: 300e9]", freq);
    cmd.AddValue("txPower", "Transmit power in dBm [default: 30]", txPower);
    cmd.AddValue("bandwidth", "Signal bandwidth in Hz [default: 10e9]", bandwidth);
    cmd.AddValue("arraySize", "Array elements per side [default: 32]", arraySize);
    cmd.Parse(argc, argv);

    // Compute antenna gains
    double elementGain = 5.0; // dBi per element
    uint32_t totalElements = arraySize * arraySize;
    double arrayGain = elementGain + 10.0 * std::log10(static_cast<double>(totalElements));
    double totalGain = 2.0 * arrayGain; // Tx + Rx (monostatic radar)

    std::cout << "=============================================================\n";
    std::cout << "  THz ISAC Space Debris Detection Analysis\n";
    std::cout << "=============================================================\n";
    std::cout << "  Frequency:       " << freq / 1e9 << " GHz\n";
    std::cout << "  Wavelength:      " << std::setprecision(2)
              << (3e8 / freq) * 1000.0 << " mm\n";
    std::cout << "  Tx Power:        " << txPower << " dBm\n";
    std::cout << "  Bandwidth:       " << bandwidth / 1e9 << " GHz\n";
    std::cout << "  Array:           " << arraySize << "x" << arraySize
              << " UM-MIMO (" << totalElements << " elements)\n";
    std::cout << "  Array Gain:      " << std::fixed << std::setprecision(1)
              << arrayGain << " dBi\n";
    std::cout << "  Total Gain:      " << totalGain << " dBi (monostatic)\n";
    std::cout << "  Integration:     " << integrationTime * 1e3 << " ms\n";
    std::cout << "-------------------------------------------------------------\n\n";

    // ---- Create nodes ----
    NodeContainer satNodes;
    satNodes.Create(1);

    // Local ENU frame (z = altitude): the satellite sits 550 km straight up;
    // the sensing targets are parametric ranges from the spacecraft, so a
    // fixed platform is the intended geometry for this radar-equation sweep.
    Ptr<ConstantPositionMobilityModel> satMob = CreateObject<ConstantPositionMobilityModel>();
    satMob->SetPosition(Vector(0.0, 0.0, 550e3));
    satNodes.Get(0)->AggregateObject(satMob);

    // ---- Create THz-NTN components ----
    Ptr<ThzNtnHelper> helper = CreateObject<ThzNtnHelper>();
    Ptr<ThzNtnIsac> isac = helper->CreateIsac("JOINT_ISAC");
    isac->SetIsacMode(JOINT_ISAC);

    // Configure antenna array
    Ptr<ThzNtnAntennaArray> array = helper->CreateAntennaArray("ISL-300GHz");
    array->Configure(arraySize, arraySize, freq, ThzArrayType::UPA);

    // ---- Resolution capabilities ----
    double rangeRes = isac->ComputeRangeResolution_m(bandwidth);
    double velRes = isac->ComputeVelocityResolution_m_s(freq, integrationTime);
    double angRes = isac->ComputeAngularResolution_deg(freq, array->ComputePhysicalSize_m());

    std::cout << "  1. Sensing Resolution Capabilities:\n";
    std::cout << "     Range resolution:   " << std::fixed << std::setprecision(4)
              << rangeRes << " m (" << rangeRes * 100.0 << " cm)\n";
    std::cout << "     Velocity resolution: " << std::setprecision(2)
              << velRes << " m/s\n";
    std::cout << "     Angular resolution:  " << std::setprecision(4)
              << angRes << " deg\n\n";

    // ---- Debris detection analysis ----
    std::vector<std::string> debrisCategories = {"small_1cm", "medium_10cm", "large_1m"};
    std::vector<std::string> debrisLabels = {"1 cm", "10 cm", "1 m"};
    std::vector<double> targetRanges = {1e3, 5e3, 10e3, 50e3, 100e3}; // metres

    std::cout << "  2. Debris Detection vs Range:\n";
    for (size_t d = 0; d < debrisCategories.size(); d++)
    {
        DebrisModel debris = isac->GetDebrisModel(debrisCategories[d]);

        std::cout << "\n  --- Debris size: " << debrisLabels[d]
                  << " (RCS: " << debris.rcs_dBsm << " dBsm) ---\n";
        std::cout << "  " << std::string(66, '-') << "\n";
        std::cout << std::setw(12) << "Range"
                  << std::setw(12) << "Radar SNR"
                  << std::setw(12) << "Pd"
                  << std::setw(12) << "CRB Range"
                  << std::setw(12) << "CRB Vel"
                  << "\n";
        std::cout << std::setw(12) << "(km)"
                  << std::setw(12) << "(dB)"
                  << std::setw(12) << ""
                  << std::setw(12) << "(m)"
                  << std::setw(12) << "(m/s)"
                  << "\n";
        std::cout << "  " << std::string(66, '-') << "\n";

        double rcs_m2 = std::pow(10.0, debris.rcs_dBsm / 10.0);

        for (double range : targetRanges)
        {
            // Perform sensing
            SensingResult result = isac->PerformSensing(
                freq, txPower, arrayGain, arrayGain,
                bandwidth, range, rcs_m2, integrationTime);

            // CRB computation
            double snrLinear = std::pow(10.0, result.snr_dB / 10.0);
            double crbRange = (snrLinear > 0.01)
                ? isac->ComputeCramerRaoBound_range(snrLinear, bandwidth) : -1.0;
            double crbVel = (snrLinear > 0.01)
                ? isac->ComputeCramerRaoBound_velocity(snrLinear, freq, integrationTime) : -1.0;

            std::cout << std::fixed << std::setprecision(1)
                      << std::setw(12) << range / 1000.0
                      << std::setw(12) << result.snr_dB
                      << std::setprecision(4)
                      << std::setw(12) << result.detectionProbability;
            if (crbRange > 0.0)
            {
                std::cout << std::setprecision(3)
                          << std::setw(12) << crbRange
                          << std::setw(12) << crbVel;
            }
            else
            {
                std::cout << std::setw(12) << "N/A"
                          << std::setw(12) << "N/A";
            }
            std::cout << "\n";
        }

        // Maximum detection range (Pd > 0.9, Pfa = 1e-6)
        double maxRange = isac->ComputeMaxDetectionRange_km(
            freq, txPower, totalGain, rcs_m2, 13.0, bandwidth);
        std::cout << "  Max detection range (Pd>0.9): " << std::setprecision(1)
                  << maxRange << " km\n";
    }

    // ---- ISAC resource split tradeoff ----
    std::cout << "\n  3. ISAC Performance Tradeoff (10 cm debris at 10 km):\n";
    std::cout << "  " << std::string(72, '-') << "\n";
    std::cout << std::setw(14) << "Sensing Frac"
              << std::setw(14) << "Comm Rate"
              << std::setw(14) << "Radar SNR"
              << std::setw(14) << "Pd"
              << std::setw(14) << "Efficiency"
              << "\n";
    std::cout << std::setw(14) << ""
              << std::setw(14) << "(Gbps)"
              << std::setw(14) << "(dB)"
              << std::setw(14) << ""
              << std::setw(14) << ""
              << "\n";
    std::cout << "  " << std::string(72, '-') << "\n";

    std::vector<double> splits = {0.1, 0.3, 0.5, 0.7, 0.9};
    DebrisModel medDebris = isac->GetDebrisModel("medium_10cm");
    double debrisRange = 10e3; // 10 km
    double debrisRcs = std::pow(10.0, medDebris.rcs_dBsm / 10.0);

    for (double split : splits)
    {
        isac->SetResourceSplit(split);

        // Evaluate joint performance
        IsacPerformance perf = isac->EvaluatePerformance(freq, bandwidth, txPower, debrisRange);

        // Radar SNR with split resources
        double radarSnr = isac->ComputeRadarSnr_dB(
            freq, txPower + 10.0 * std::log10(split),
            totalGain, debrisRange, debrisRcs, bandwidth * split);
        double pd = isac->ComputeDetectionProbability(radarSnr, 1e-6);

        std::cout << std::fixed << std::setprecision(2)
                  << std::setw(14) << split
                  << std::setw(14) << perf.commRate_Gbps
                  << std::setprecision(1)
                  << std::setw(14) << radarSnr
                  << std::setprecision(4)
                  << std::setw(14) << pd
                  << std::setprecision(3)
                  << std::setw(14) << perf.totalEfficiency
                  << "\n";
    }

    // ---- Run simulation ----
    Simulator::Stop(Seconds(1.0));
    Simulator::Run();
    Simulator::Destroy();

    std::cout << "\n  Simulation complete.\n";
    std::cout << "=============================================================\n";

    return 0;
}
