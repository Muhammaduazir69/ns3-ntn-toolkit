/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-only
 * Author: Muhammad Uzair
 *
 * Example: THz Inter-Satellite Link (ISL) Analysis
 *
 * Demonstrates THz ISL between two LEO satellites at 300 GHz.
 * Evaluates link performance at various inter-satellite distances
 * and compares Shannon vs hardware-limited capacity.
 */

#include <ns3/command-line.h>
#include <ns3/core-module.h>
#include <ns3/mobility-module.h>
#include <ns3/node-container.h>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>

#include "ns3/ntn-realistic-traffic-helper.h"

// Forward declarations of THz-NTN classes
namespace ns3
{
class ThzNtnHelper;
class ThzNtnIslChannel;
class ThzNtnLinkBudget;
class ThzNtnAntennaArray;
} // namespace ns3

#include "ns3/thz-ntn-helper.h"
#include "ns3/thz-ntn-isl-channel.h"
#include "ns3/thz-ntn-link-budget.h"
#include "ns3/thz-ntn-antenna-array.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("ThzNtnIsl");

int
main(int argc, char* argv[])
{
    // ---- Default parameters ----
    double freq = 300e9;
    double txPower = 30.0;
    double bandwidth = 20e9;
    double txGain = 40.0;
    double rxGain = 40.0;
    double altitude = 550.0;
    double simTime = 60.0;
    std::string outputDir = "thz-isl-out";

    CommandLine cmd(__FILE__);
    cmd.AddValue("freq", "Carrier frequency in Hz [default: 300e9]", freq);
    cmd.AddValue("txPower", "Transmit power in dBm [default: 30]", txPower);
    cmd.AddValue("bandwidth", "Channel bandwidth in Hz [default: 20e9]", bandwidth);
    cmd.AddValue("txGain", "Tx antenna gain in dBi [default: 40]", txGain);
    cmd.AddValue("rxGain", "Rx antenna gain in dBi [default: 40]", rxGain);
    cmd.AddValue("simTime", "Simulation time (s)", simTime);
    cmd.AddValue("outputDir", "Output directory", outputDir);
    cmd.Parse(argc, argv);

    std::cout << "=============================================================\n";
    std::cout << "  THz Inter-Satellite Link (ISL) Analysis\n";
    std::cout << "=============================================================\n";
    std::cout << "  Frequency:  " << freq / 1e9 << " GHz\n";
    std::cout << "  Tx Power:   " << txPower << " dBm\n";
    std::cout << "  Bandwidth:  " << bandwidth / 1e9 << " GHz\n";
    std::cout << "  Tx Gain:    " << txGain << " dBi\n";
    std::cout << "  Rx Gain:    " << rxGain << " dBi\n";
    std::cout << "  Altitude:   " << altitude << " km (both satellites)\n";
    std::cout << "  Channel:    Vacuum (no molecular absorption)\n";
    std::cout << "-------------------------------------------------------------\n\n";

    // ---- Create nodes: two satellites at 550 km ----
    NodeContainer satNodes;
    satNodes.Create(2);

    Ptr<ConstantPositionMobilityModel> sat1Mob = CreateObject<ConstantPositionMobilityModel>();
    sat1Mob->SetPosition(Vector(0.0, 0.0, altitude * 1000.0));
    satNodes.Get(0)->AggregateObject(sat1Mob);

    Ptr<ConstantPositionMobilityModel> sat2Mob = CreateObject<ConstantPositionMobilityModel>();
    sat2Mob->SetPosition(Vector(500e3, 0.0, altitude * 1000.0)); // 500 km apart initially
    satNodes.Get(1)->AggregateObject(sat2Mob);

    // ---- Create THz-NTN components ----
    Ptr<ThzNtnHelper> helper = CreateObject<ThzNtnHelper>();
    Ptr<ThzNtnIslChannel> islChannel = helper->CreateIslChannel("ISL-300GHz");
    Ptr<ThzNtnLinkBudget> linkBudget = helper->CreateLinkBudget();

    // ---- Evaluate at various ISL distances ----
    std::vector<double> distances_km = {100.0, 500.0, 1000.0, 2000.0, 5000.0};

    std::cout << "  1. ISL Performance vs Distance\n";
    std::cout << "  " << std::string(78, '-') << "\n";
    std::cout << std::setw(10) << "Distance"
              << std::setw(10) << "FSPL"
              << std::setw(10) << "SNR"
              << std::setw(14) << "Shannon Cap"
              << std::setw(14) << "HW-Ltd Cap"
              << std::setw(10) << "Doppler"
              << std::setw(10) << "Feasible"
              << "\n";
    std::cout << std::setw(10) << "(km)"
              << std::setw(10) << "(dB)"
              << std::setw(10) << "(dB)"
              << std::setw(14) << "(Gbps)"
              << std::setw(14) << "(Gbps)"
              << std::setw(10) << "(MHz)"
              << std::setw(10) << ""
              << "\n";
    std::cout << "  " << std::string(78, '-') << "\n";

    for (double dist_km : distances_km)
    {
        double dist_m = dist_km * 1000.0;

        double fspl = 20.0 * std::log10(dist_m)
                    + 20.0 * std::log10(freq)
                    + 20.0 * std::log10(4.0 * M_PI / 299792458.0);
        ThzNtnLinkBudget::LinkBudgetResult result = linkBudget->ComputeIslBudget(freq, dist_km);
        double snr = result.snr_dB;
        double shannonCap = islChannel->ComputeIslCapacity_Gbps(snr);
        double hwCap = result.hardwareLimitedCapacity_Gbps;

        // Inline Doppler from relative radial velocity.
        double relVel = (dist_km > 2000.0) ? 1000.0 : 100.0;
        double doppler = relVel / 299792458.0 * freq;

        bool feasible = (snr >= 3.0);

        std::cout << std::fixed << std::setprecision(1)
                  << std::setw(10) << dist_km
                  << std::setw(10) << fspl
                  << std::setw(10) << snr
                  << std::setprecision(2)
                  << std::setw(14) << shannonCap
                  << std::setw(14) << hwCap
                  << std::setprecision(1)
                  << std::setw(10) << doppler / 1e6
                  << std::setw(10) << (feasible ? "Yes" : "No")
                  << "\n";
    }

    // ---- Shannon vs Hardware-limited capacity comparison ----
    std::cout << "\n  2. Shannon vs Hardware-Limited Capacity Comparison\n";
    std::cout << "  " << std::string(60, '-') << "\n";
    std::cout << std::setw(10) << "SNR"
              << std::setw(16) << "Shannon"
              << std::setw(16) << "HW-Limited"
              << std::setw(16) << "Efficiency"
              << "\n";
    std::cout << std::setw(10) << "(dB)"
              << std::setw(16) << "(Gbps)"
              << std::setw(16) << "(Gbps)"
              << std::setw(16) << "(%)"
              << "\n";
    std::cout << "  " << std::string(60, '-') << "\n";

    for (double snr_dB = 0.0; snr_dB <= 40.0; snr_dB += 5.0)
    {
        double shannonCap = islChannel->ComputeIslCapacity_Gbps(snr_dB);

        // Hardware-limited: capacity saturates due to EVM floor
        [[maybe_unused]] double snrLinear = std::pow(10.0, snr_dB / 10.0);
        double hwSnrCeiling = 25.0; // dB, typical EVM-limited ceiling
        double effectiveSnr = std::min(snr_dB, hwSnrCeiling);
        double effectiveSnrLin = std::pow(10.0, effectiveSnr / 10.0);
        double hwCap = bandwidth * std::log2(1.0 + effectiveSnrLin) / 1e9;

        double efficiency = (shannonCap > 0.0) ? (hwCap / shannonCap * 100.0) : 0.0;

        std::cout << std::fixed << std::setprecision(1)
                  << std::setw(10) << snr_dB
                  << std::setprecision(2)
                  << std::setw(16) << shannonCap
                  << std::setw(16) << hwCap
                  << std::setprecision(1)
                  << std::setw(16) << efficiency
                  << "\n";
    }

    // ---- ISL type comparison ----
    std::cout << "\n  3. ISL Type Characteristics\n";
    std::cout << "  " << std::string(60, '-') << "\n";
    std::cout << "  Intra-plane ISL:  Same orbital plane, ~2600 km spacing\n";
    std::cout << "  Inter-plane ISL:  Adjacent planes, ~1000-3000 km\n";
    std::cout << "  Cross-link ISL:   Cross-seam, up to ~5000 km\n\n";

    // Compute noise temperature in space
    double noiseTemp = islChannel->ComputeSpaceNoiseTemperature_K();
    std::cout << "  Space noise temperature: " << noiseTemp << " K\n";
    std::cout << "  (CMB 2.7 K + receiver contribution)\n";

    // Maximum feasible distance
    double maxDist = islChannel->ComputeMaxLinkDistance_km(3.0);
    std::cout << "\n  Maximum ISL distance (SNR > 3 dB): " << std::fixed << std::setprecision(1)
              << maxDist << " km\n";

    // ---- Real packet plane + dynamic ISL-distance sampler (v2) ----
    NtnRealisticTrafficHelper traffic;
    traffic.SetSimTime(Seconds(simTime));
    traffic.SetOutputDir(outputDir);
    traffic.SetRunTag("thz-ntn-isl");
    traffic.SetProfile(NtnRealisticTrafficHelper::TrafficProfile::EmbbStreaming);
    traffic.InstallUes(6);

    std::filesystem::create_directories(outputDir);
    std::ofstream isltimes(outputDir + "/isl_timeseries.csv");
    isltimes << "time_s,distance_km,fspl_dB,snr_dB,shannon_Gbps,doppler_MHz\n";

    traffic.RegisterPeriodicCallback(Seconds(1.0), [&](Time nowT) {
        double t = nowT.GetSeconds();
        // Sinusoidal in-plane separation between 100 km and 5000 km
        double d_km = 100.0 + (4900.0) * 0.5 * (1 - std::cos(2 * M_PI * t / simTime));
        auto r = linkBudget->ComputeLinkBudget(ThzNtnLinkBudget::INTER_SATELLITE,
            freq, d_km * 1000.0, 90.0, txPower, txGain, rxGain, bandwidth, 10.0);
        double relV = 100.0 * std::sin(2 * M_PI * t / simTime);
        double doppler_MHz = relV / 299792458.0 * freq / 1e6;
        isltimes << std::fixed << std::setprecision(2)
                 << t << "," << d_km << "," << r.fspl_dB << "," << r.snr_dB
                 << "," << r.shannonCapacity_Gbps << "," << doppler_MHz << "\n";
    });
    traffic.Wire();

    Simulator::Stop(Seconds(simTime + 0.5));
    Simulator::Run();
    traffic.WriteHealthReport();
    isltimes.close();
    Simulator::Destroy();

    std::cout << "\n  Simulation complete.\n";
    std::cout << "=============================================================\n";

    return 0;
}
