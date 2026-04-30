/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-only
 * Author: Muhammad Uzair
 *
 * Example: RIS-Assisted THz Satellite Link
 *
 * Demonstrates the SNR improvement from deploying a ground-based
 * Reconfigurable Intelligent Surface (RIS) to assist a THz satellite
 * downlink.  Compares direct vs RIS-assisted link performance and
 * verifies the N^2 scaling law.
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
class ThzNtnRis;
class ThzNtnLinkBudget;
class ThzNtnChannelModel;
} // namespace ns3

#include "ns3/thz-ntn-helper.h"
#include "ns3/thz-ntn-ris.h"
#include "ns3/thz-ntn-link-budget.h"
#include "ns3/thz-ntn-channel-model.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("ThzNtnRisAssisted");

/**
 * \brief Compute slant range from elevation angle and altitude.
 */
static double
ComputeSlantRange(double elevDeg, double altKm)
{
    const double Re = 6371.0;
    double elevRad = elevDeg * M_PI / 180.0;
    double ratio = (Re + altKm) / Re;
    double d_km = Re * (std::sqrt(ratio * ratio - std::cos(elevRad) * std::cos(elevRad))
                        - std::sin(elevRad));
    return d_km * 1000.0;
}

int
main(int argc, char* argv[])
{
    // ---- Default parameters ----
    double freq = 300e9;         // 300 GHz
    uint32_t risSize = 64;       // 64x64 default
    double altitude = 550.0;     // km
    double elevation = 30.0;     // degrees
    double txPower = 34.77;      // dBm (3W)
    double bandwidth = 10e9;     // 10 GHz
    double txGain = 40.0;        // dBi
    double rxGain = 42.0;        // dBi
    double noiseFigure = 10.0;   // dB

    // ---- Parse command-line arguments ----
    CommandLine cmd(__FILE__);
    cmd.AddValue("freq", "Carrier frequency in Hz [default: 300e9]", freq);
    cmd.AddValue("risSize", "RIS elements per side [default: 64]", risSize);
    cmd.AddValue("elevation", "Elevation angle in degrees [default: 30]", elevation);
    cmd.AddValue("txPower", "Satellite Tx power in dBm [default: 34.77]", txPower);
    cmd.Parse(argc, argv);

    std::cout << "=============================================================\n";
    std::cout << "  RIS-Assisted THz Satellite Link Analysis\n";
    std::cout << "=============================================================\n";
    std::cout << "  Frequency:    " << freq / 1e9 << " GHz\n";
    std::cout << "  Altitude:     " << altitude << " km\n";
    std::cout << "  Elevation:    " << elevation << " deg\n";
    std::cout << "  Tx Power:     " << txPower << " dBm\n";
    std::cout << "  Bandwidth:    " << bandwidth / 1e9 << " GHz\n";
    std::cout << "  RIS deploy:   Ground-based\n";
    std::cout << "-------------------------------------------------------------\n\n";

    // ---- Create nodes ----
    NodeContainer satNodes;
    satNodes.Create(1);

    NodeContainer gtNodes;
    gtNodes.Create(1);

    NodeContainer risNodes;
    risNodes.Create(1);

    // Set up mobility
    double slantRange = ComputeSlantRange(elevation, altitude);
    Ptr<ConstantPositionMobilityModel> satMob = CreateObject<ConstantPositionMobilityModel>();
    satMob->SetPosition(Vector(0.0, 0.0, altitude * 1000.0));
    satNodes.Get(0)->AggregateObject(satMob);

    Ptr<ConstantPositionMobilityModel> gtMob = CreateObject<ConstantPositionMobilityModel>();
    gtMob->SetPosition(Vector(0.0, 0.0, 0.0));
    gtNodes.Get(0)->AggregateObject(gtMob);

    // RIS placed 200m from ground terminal
    double risDist = 200.0; // metres
    Ptr<ConstantPositionMobilityModel> risMob = CreateObject<ConstantPositionMobilityModel>();
    risMob->SetPosition(Vector(risDist, 0.0, 10.0)); // 10m height
    risNodes.Get(0)->AggregateObject(risMob);

    // ---- Create THz-NTN components ----
    Ptr<ThzNtnHelper> helper = CreateObject<ThzNtnHelper>();
    Ptr<ThzNtnLinkBudget> linkBudget = helper->CreateLinkBudget();

    // ---- Compute direct link (no RIS) baseline ----
    ThzNtnLinkBudget::LinkBudgetResult directResult = linkBudget->ComputeLinkBudget(
        ThzNtnLinkBudget::SAT_TO_GROUND,
        freq,
        slantRange,
        elevation,
        txPower,
        txGain,
        rxGain,
        bandwidth,
        noiseFigure);

    std::cout << "  1. Direct Link (No RIS) Baseline:\n";
    std::cout << "     Slant range:    " << std::fixed << std::setprecision(1)
              << slantRange / 1000.0 << " km\n";
    std::cout << "     Total path loss: " << directResult.totalPathLoss_dB << " dB\n";
    std::cout << "     SNR:             " << directResult.snr_dB << " dB\n";
    std::cout << "     Capacity:        " << std::setprecision(2)
              << directResult.shannonCapacity_Gbps << " Gbps\n\n";

    // ---- Vary RIS size and compute RIS-assisted performance ----
    std::vector<uint32_t> risSizes = {16, 32, 64, 128};

    std::cout << "  2. RIS-Assisted Performance vs Panel Size:\n";
    std::cout << "  " << std::string(82, '-') << "\n";
    std::cout << std::setw(10) << "RIS Size"
              << std::setw(12) << "Elements"
              << std::setw(12) << "RIS Gain"
              << std::setw(14) << "Cascaded PL"
              << std::setw(12) << "Total SNR"
              << std::setw(12) << "SNR Improv"
              << std::setw(12) << "Capacity"
              << "\n";
    std::cout << std::setw(10) << "(NxN)"
              << std::setw(12) << "(N^2)"
              << std::setw(12) << "(dB)"
              << std::setw(14) << "(dB)"
              << std::setw(12) << "(dB)"
              << std::setw(12) << "(dB)"
              << std::setw(12) << "(Gbps)"
              << "\n";
    std::cout << "  " << std::string(82, '-') << "\n";

    for (uint32_t N : risSizes)
    {
        // Create RIS with this size
        Ptr<ThzNtnRis> ris = helper->CreateRis(N, N, "ground");
        ris->Configure(N, N, freq, RisDeployment::GROUND);

        // Set optimal phase profile for satellite->RIS->GT
        ris->ComputeOptimalPhases(elevation, 0.0, 0.0, 0.0);

        // RIS gain with perfect CSI (N^2 scaling)
        double risGain = ris->ComputeSnrGain_dB(true);
        double maxRisGain = ris->ComputeMaxGain_dB();

        // Cascaded path loss: Sat->RIS + RIS->GT
        double d1 = slantRange;     // Sat to RIS ~ same as slant range
        double d2 = risDist;        // RIS to GT
        double cascadedPL = ris->ComputeCascadedPathLoss_dB(d1, d2, freq);

        // Total SNR with RIS = direct SNR + RIS gain
        double totalSnr = directResult.snr_dB + risGain;
        double snrImprovement = risGain;

        // Capacity with RIS
        double snrLinear = std::pow(10.0, totalSnr / 10.0);
        double capacity = bandwidth * std::log2(1.0 + snrLinear) / 1e9;

        uint32_t totalElements = N * N;

        std::cout << std::fixed << std::setprecision(1)
                  << std::setw(7) << N << "x" << N
                  << std::setw(12 - 2) << totalElements
                  << std::setw(12) << risGain
                  << std::setw(14) << cascadedPL
                  << std::setw(12) << totalSnr
                  << std::setw(12) << snrImprovement
                  << std::setprecision(2)
                  << std::setw(12) << capacity
                  << "\n";
    }

    // ---- N^2 scaling law verification ----
    std::cout << "\n  3. N^2 Scaling Law Verification (Perfect CSI):\n";
    std::cout << "  " << std::string(58, '-') << "\n";
    std::cout << std::setw(10) << "N"
              << std::setw(12) << "N^2"
              << std::setw(16) << "10*log10(N^2)"
              << std::setw(16) << "Measured Gain"
              << "\n";
    std::cout << std::setw(10) << ""
              << std::setw(12) << ""
              << std::setw(16) << "(dB)"
              << std::setw(16) << "(dB)"
              << "\n";
    std::cout << "  " << std::string(58, '-') << "\n";

    for (uint32_t N : risSizes)
    {
        Ptr<ThzNtnRis> ris = helper->CreateRis(N, N, "ground");
        ris->Configure(N, N, freq, RisDeployment::GROUND);

        double measuredGain = ris->ComputeSnrGain_dB(true);
        uint32_t N2 = N * N;
        double theoreticalGain = 10.0 * std::log10(static_cast<double>(N2));

        std::cout << std::fixed << std::setprecision(1)
                  << std::setw(10) << N
                  << std::setw(12) << N2
                  << std::setw(16) << theoreticalGain
                  << std::setw(16) << measuredGain
                  << "\n";
    }

    // ---- Quantization loss analysis ----
    std::cout << "\n  4. Phase Quantization Impact (64x64 RIS):\n";
    std::cout << "  " << std::string(40, '-') << "\n";
    std::cout << "  Continuous phase:  0.0 dB loss\n";

    Ptr<ThzNtnRis> ris64 = helper->CreateRis(64, 64, "ground");
    ris64->Configure(64, 64, freq, RisDeployment::GROUND);
    double qLoss = ris64->ComputeQuantizationLoss_dB();
    std::cout << "  Current config:    " << std::fixed << std::setprecision(2)
              << qLoss << " dB loss\n";

    // Near-field check
    bool nearField = ris64->IsInNearField(risDist);
    std::cout << "\n  RIS at " << risDist << " m from GT: "
              << (nearField ? "Near-field" : "Far-field") << " region\n";
    std::cout << "  Total RIS elements: " << ris64->GetTotalElements() << "\n";

    // ---- Run simulation ----
    Simulator::Stop(Seconds(1.0));
    Simulator::Run();
    Simulator::Destroy();

    std::cout << "\n  Simulation complete.\n";
    std::cout << "=============================================================\n";

    return 0;
}
