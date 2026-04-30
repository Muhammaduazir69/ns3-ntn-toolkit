/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-only
 * Author: Muhammad Uzair
 *
 * Example: UM-MIMO Array Analysis at THz Frequencies
 *
 * Characterises Ultra-Massive MIMO antenna arrays at 300 GHz.
 * Computes gain, beamwidth, near-field distance, physical size,
 * and beam patterns for various array sizes.  Also compares with
 * a Cassegrain reflector antenna for ground terminal use.
 */

#include <ns3/command-line.h>
#include <ns3/core-module.h>
#include <ns3/node-container.h>

#include <cmath>
#include <iomanip>
#include <iostream>
#include <vector>

// Forward declarations
namespace ns3
{
class ThzNtnHelper;
class ThzNtnAntennaArray;
class ThzNtnBeamforming;
} // namespace ns3

#include "ns3/thz-ntn-helper.h"
#include "ns3/thz-ntn-antenna-array.h"
#include "ns3/thz-ntn-beamforming.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("ThzNtnUmMimo");

int
main(int argc, char* argv[])
{
    // ---- Default parameters ----
    double freq = 300e9;        // 300 GHz
    uint32_t numElements = 32;  // default array side
    double cassegrainDiam = 0.45; // metres

    // ---- Parse command-line arguments ----
    CommandLine cmd(__FILE__);
    cmd.AddValue("freq", "Operating frequency in Hz [default: 300e9]", freq);
    cmd.AddValue("numElements", "Array elements per side [default: 32]", numElements);
    cmd.AddValue("cassegrainDiam", "Cassegrain aperture diameter in m [default: 0.45]",
                 cassegrainDiam);
    cmd.Parse(argc, argv);

    double lambda = 3e8 / freq;
    double lambdaMm = lambda * 1000.0;

    std::cout << "=============================================================\n";
    std::cout << "  UM-MIMO Antenna Array Analysis at THz Frequencies\n";
    std::cout << "=============================================================\n";
    std::cout << "  Frequency:     " << freq / 1e9 << " GHz\n";
    std::cout << "  Wavelength:    " << std::fixed << std::setprecision(3)
              << lambdaMm << " mm\n";
    std::cout << "  Element spacing: " << lambdaMm / 2.0 << " mm (lambda/2)\n";
    std::cout << "-------------------------------------------------------------\n\n";

    // ---- Create helper ----
    Ptr<ThzNtnHelper> helper = CreateObject<ThzNtnHelper>();

    // ---- Array comparison table ----
    std::vector<uint32_t> arraySizes = {8, 16, 32, 64};

    std::cout << "  1. Array Configuration Comparison:\n";
    std::cout << "  " << std::string(82, '-') << "\n";
    std::cout << std::setw(10) << "Array"
              << std::setw(10) << "Elements"
              << std::setw(12) << "Max Gain"
              << std::setw(12) << "Beamwidth"
              << std::setw(14) << "Near-Field"
              << std::setw(12) << "Phys Size"
              << std::setw(12) << "Coupling"
              << "\n";
    std::cout << std::setw(10) << "(NxN)"
              << std::setw(10) << "(N^2)"
              << std::setw(12) << "(dBi)"
              << std::setw(12) << "(deg)"
              << std::setw(14) << "(m)"
              << std::setw(12) << "(mm)"
              << std::setw(12) << "(dB)"
              << "\n";
    std::cout << "  " << std::string(82, '-') << "\n";

    for (uint32_t N : arraySizes)
    {
        Ptr<ThzNtnAntennaArray> array = CreateObject<ThzNtnAntennaArray>();
        array->Configure(N, N, freq, ThzArrayType::UPA);

        double maxGain = array->ComputeMaxGain_dBi();
        double beamwidth = array->ComputeBeamwidth3dB_deg();
        double nearField = array->ComputeNearFieldDistance_m();
        double physSize = array->ComputePhysicalSize_m();
        double coupling = array->ComputeMutualCouplingLoss_dB();

        std::cout << std::fixed
                  << std::setw(7) << N << "x" << N
                  << std::setw(10 - 2) << (N * N)
                  << std::setprecision(1)
                  << std::setw(12) << maxGain
                  << std::setprecision(3)
                  << std::setw(12) << beamwidth
                  << std::setprecision(2)
                  << std::setw(14) << nearField
                  << std::setprecision(1)
                  << std::setw(12) << physSize * 1000.0
                  << std::setw(12) << coupling
                  << "\n";
    }

    // ---- Cassegrain antenna for comparison ----
    std::cout << "\n  Cassegrain reflector (ground terminal):\n";

    Ptr<ThzNtnAntennaArray> cassArray = CreateObject<ThzNtnAntennaArray>();
    cassArray->Configure(1, 1, freq, ThzArrayType::CASSEGRAIN);

    double cassGain = cassArray->ComputeCassegrainGain_dBi(0.0, cassegrainDiam, freq);
    double cassNearField = 2.0 * cassegrainDiam * cassegrainDiam / lambda;

    std::cout << "    Diameter:       " << cassegrainDiam * 100.0 << " cm\n";
    std::cout << "    Boresight gain: " << std::fixed << std::setprecision(1)
              << cassGain << " dBi\n";
    std::cout << "    Near-field:     " << std::setprecision(1)
              << cassNearField << " m\n";

    // ---- Beam pattern (gain vs angle) for selected array ----
    std::cout << "\n  2. Beam Pattern (" << numElements << "x" << numElements
              << " array, boresight steering):\n";
    std::cout << "  " << std::string(36, '-') << "\n";
    std::cout << std::setw(12) << "Angle"
              << std::setw(12) << "Gain"
              << std::setw(12) << "Norm Gain"
              << "\n";
    std::cout << std::setw(12) << "(deg)"
              << std::setw(12) << "(dBi)"
              << std::setw(12) << "(dB)"
              << "\n";
    std::cout << "  " << std::string(36, '-') << "\n";

    Ptr<ThzNtnAntennaArray> patternArray = CreateObject<ThzNtnAntennaArray>();
    patternArray->Configure(numElements, numElements, freq, ThzArrayType::UPA);
    double peakGain = patternArray->ComputeMaxGain_dBi();

    // Print at selected angles for readability
    std::vector<int> angles;
    for (int a = -90; a <= 90; a += 5)
    {
        angles.push_back(a);
    }

    for (int angleDeg : angles)
    {
        double gain = patternArray->ComputeArrayGain_dBi(
            static_cast<double>(angleDeg), 0.0, 0.0, 0.0);
        double normGain = gain - peakGain;

        std::cout << std::fixed << std::setprecision(1)
                  << std::setw(12) << angleDeg
                  << std::setw(12) << gain
                  << std::setw(12) << normGain
                  << "\n";
    }

    // ---- Beam squint analysis ----
    std::cout << "\n  3. Beam Squint at Band Edges:\n";
    std::cout << "  " << std::string(58, '-') << "\n";

    Ptr<ThzNtnBeamforming> beamforming = helper->CreateBeamforming(64);

    double bw = 20e9; // 20 GHz bandwidth
    std::vector<double> steerAngles = {0.0, 10.0, 20.0, 30.0, 45.0, 60.0};

    std::cout << std::setw(14) << "Steer Angle"
              << std::setw(14) << "Lower Edge"
              << std::setw(14) << "Upper Edge"
              << std::setw(14) << "Squint Loss"
              << "\n";
    std::cout << std::setw(14) << "(deg)"
              << std::setw(14) << "(deg)"
              << std::setw(14) << "(deg)"
              << std::setw(14) << "(dB)"
              << "\n";
    std::cout << "  " << std::string(58, '-') << "\n";

    for (double steer : steerAngles)
    {
        double lowerEdge = beamforming->ComputeBeamSquintAngle_deg(
            freq - bw / 2.0, freq, steer);
        double upperEdge = beamforming->ComputeBeamSquintAngle_deg(
            freq + bw / 2.0, freq, steer);
        double squintLoss = beamforming->ComputeBeamSquintLoss_dB(bw, freq, steer);

        std::cout << std::fixed << std::setprecision(2)
                  << std::setw(14) << steer
                  << std::setprecision(3)
                  << std::setw(14) << lowerEdge
                  << std::setw(14) << upperEdge
                  << std::setprecision(2)
                  << std::setw(14) << squintLoss
                  << "\n";
    }

    // ---- Near-field distance analysis ----
    std::cout << "\n  4. Near-Field Boundary (Fraunhofer Distance):\n";
    std::cout << "  " << std::string(40, '-') << "\n";

    for (uint32_t N : arraySizes)
    {
        Ptr<ThzNtnAntennaArray> arr = CreateObject<ThzNtnAntennaArray>();
        arr->Configure(N, N, freq, ThzArrayType::UPA);
        double nfDist = arr->ComputeNearFieldDistance_m();
        double physSz = arr->ComputePhysicalSize_m();

        std::cout << "    " << N << "x" << N << " array ("
                  << std::setprecision(1) << physSz * 1000.0 << " mm): "
                  << std::setprecision(2) << nfDist << " m";

        // Check if typical satellite distance is in near-field
        bool satNearField = arr->IsInNearField(550e3);
        std::cout << (satNearField ? " [SAT: near-field!]" : " [SAT: far-field]")
                  << "\n";
    }

    // ---- Element pattern ----
    std::cout << "\n  5. Element Radiation Pattern (Cosine model):\n";
    std::cout << "  " << std::string(30, '-') << "\n";
    std::cout << std::setw(12) << "Angle"
              << std::setw(14) << "Element Gain"
              << "\n";
    std::cout << std::setw(12) << "(deg)"
              << std::setw(14) << "(dBi)"
              << "\n";
    std::cout << "  " << std::string(30, '-') << "\n";

    for (int ang = 0; ang <= 90; ang += 10)
    {
        double elemGain = patternArray->ComputeElementPattern_dBi(static_cast<double>(ang));
        std::cout << std::fixed << std::setprecision(1)
                  << std::setw(12) << ang
                  << std::setw(14) << elemGain
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
