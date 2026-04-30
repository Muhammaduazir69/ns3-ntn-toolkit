/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-only
 * Author: Muhammad Uzair
 *
 * Example: THz-NTN LEO-to-Ground TeraLink Scenario
 *
 * Demonstrates a single LEO satellite (550 km) to ground terminal link
 * at 225 GHz using the TeraLink preset.  Computes link budget at various
 * elevation angles and prints a detailed results table.
 */

#include <ns3/command-line.h>
#include <ns3/core-module.h>
#include <ns3/mobility-module.h>
#include <ns3/node-container.h>

#include <cmath>
#include <iomanip>
#include <iostream>

// Forward declarations of THz-NTN classes
namespace ns3
{
class ThzNtnHelper;
class ThzNtnChannelModel;
class ThzNtnPhySat;
class ThzNtnPhyGround;
class ThzNtnLinkBudget;
class ThzNtnAntennaArray;
} // namespace ns3

#include "ns3/thz-ntn-helper.h"
#include "ns3/thz-ntn-link-budget.h"
#include "ns3/thz-ntn-phy-sat.h"
#include "ns3/thz-ntn-phy-ground.h"
#include "ns3/thz-ntn-antenna-array.h"
#include "ns3/thz-ntn-channel-model.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("ThzNtnLeoGround");

/**
 * \brief Compute slant range from elevation angle and altitude.
 *
 * Uses the geometric relation for Earth-satellite slant distance:
 *   d = Re * (sqrt((h/Re + 1)^2 - cos^2(el)) - sin(el))
 *
 * \param elevDeg elevation angle in degrees
 * \param altKm satellite altitude in km
 * \return slant range in metres
 */
static double
ComputeSlantRange(double elevDeg, double altKm)
{
    const double Re = 6371.0; // Earth radius in km
    double elevRad = elevDeg * M_PI / 180.0;
    double ratio = (Re + altKm) / Re;
    double d_km = Re * (std::sqrt(ratio * ratio - std::cos(elevRad) * std::cos(elevRad))
                        - std::sin(elevRad));
    return d_km * 1000.0; // return in metres
}

int
main(int argc, char* argv[])
{
    // ---- Default parameters ----
    double freq = 225e9;       // 225 GHz
    double altitude = 550.0;   // km
    double txPower = 34.77;    // dBm (3 W)
    double bandwidth = 10e9;   // 10 GHz
    double txGain = 40.0;      // dBi (satellite UM-MIMO)
    double rxGain = 45.0;      // dBi (ground Cassegrain)
    double noiseFigure = 10.0; // dB

    // ---- Parse command-line arguments ----
    CommandLine cmd(__FILE__);
    cmd.AddValue("freq", "Carrier frequency in Hz [default: 225e9]", freq);
    cmd.AddValue("altitude", "Satellite altitude in km [default: 550]", altitude);
    cmd.AddValue("txPower", "Transmit power in dBm [default: 34.77]", txPower);
    cmd.AddValue("bandwidth", "Channel bandwidth in Hz [default: 10e9]", bandwidth);
    cmd.AddValue("txGain", "Tx antenna gain in dBi [default: 40]", txGain);
    cmd.AddValue("rxGain", "Rx antenna gain in dBi [default: 45]", rxGain);
    cmd.Parse(argc, argv);

    std::cout << "=============================================================\n";
    std::cout << "  THz-NTN LEO-to-Ground Link Analysis (TeraLink-225GHz)\n";
    std::cout << "=============================================================\n";
    std::cout << "  Frequency:    " << freq / 1e9 << " GHz\n";
    std::cout << "  Altitude:     " << altitude << " km\n";
    std::cout << "  Tx Power:     " << txPower << " dBm ("
              << std::pow(10.0, (txPower - 30.0) / 10.0) << " W)\n";
    std::cout << "  Bandwidth:    " << bandwidth / 1e9 << " GHz\n";
    std::cout << "  Tx Gain:      " << txGain << " dBi (UM-MIMO 32x32)\n";
    std::cout << "  Rx Gain:      " << rxGain << " dBi (Cassegrain 0.45m)\n";
    std::cout << "  Noise Figure: " << noiseFigure << " dB\n";
    std::cout << "-------------------------------------------------------------\n\n";

    // ---- Create nodes ----
    NodeContainer satNodes;
    satNodes.Create(1);

    NodeContainer gtNodes;
    gtNodes.Create(1);

    // ---- Set up mobility: satellite at altitude, ground at origin ----
    Ptr<ConstantPositionMobilityModel> satMob = CreateObject<ConstantPositionMobilityModel>();
    satMob->SetPosition(Vector(0.0, 0.0, altitude * 1000.0));
    satNodes.Get(0)->AggregateObject(satMob);

    Ptr<ConstantPositionMobilityModel> gtMob = CreateObject<ConstantPositionMobilityModel>();
    gtMob->SetPosition(Vector(0.0, 0.0, 0.0));
    gtNodes.Get(0)->AggregateObject(gtMob);

    // ---- Create THz-NTN helper and components ----
    Ptr<ThzNtnHelper> helper = CreateObject<ThzNtnHelper>();

    // Create satellite PHY (CubeSat, regenerative, 3W)
    Ptr<ThzNtnPhySat> satPhy = helper->CreateSatellitePhy("TeraLink-225GHz");
    satPhy->SetSatelliteClass(SAT_CUBESAT);
    satPhy->SetPayloadMode(PAYLOAD_REGENERATIVE);

    // Create ground PHY (heterodyne, dual-circular polarization)
    Ptr<ThzNtnPhyGround> gtPhy = helper->CreateGroundPhy("TeraLink-225GHz");
    gtPhy->SetReceiverType(RX_HETERODYNE);
    gtPhy->SetPolarization(POL_DUAL_CIRCULAR);

    // Create channel model
    Ptr<ThzNtnChannelModel> channel = helper->CreateChannelModel("TeraLink-225GHz");
    channel->SetLinkType(ThzNtnChannelModel::SAT_TO_GROUND);
    channel->EnableMolecularAbsorption(true);
    channel->EnableWeatherEffects(true);
    channel->EnableScintillation(true);

    // Create link budget calculator
    Ptr<ThzNtnLinkBudget> linkBudget = helper->CreateLinkBudget();

    // ---- Evaluate at various elevation angles ----
    std::vector<double> elevations = {5.0, 10.0, 20.0, 30.0, 45.0, 60.0, 90.0};

    // Print header
    std::cout << std::setw(8) << "Elev"
              << std::setw(12) << "Range"
              << std::setw(10) << "FSPL"
              << std::setw(10) << "MolAbs"
              << std::setw(10) << "Weather"
              << std::setw(12) << "TotalPL"
              << std::setw(10) << "SNR"
              << std::setw(12) << "Capacity"
              << "\n";
    std::cout << std::setw(8) << "(deg)"
              << std::setw(12) << "(km)"
              << std::setw(10) << "(dB)"
              << std::setw(10) << "(dB)"
              << std::setw(10) << "(dB)"
              << std::setw(12) << "(dB)"
              << std::setw(10) << "(dB)"
              << std::setw(12) << "(Gbps)"
              << "\n";
    std::cout << std::string(84, '-') << "\n";

    for (double elev : elevations)
    {
        double range_m = ComputeSlantRange(elev, altitude);
        double range_km = range_m / 1000.0;

        // Compute full link budget
        ThzNtnLinkBudget::LinkBudgetResult result = linkBudget->ComputeLinkBudget(
            ThzNtnLinkBudget::SAT_TO_GROUND,
            freq,
            range_m,
            elev,
            txPower,
            txGain,
            rxGain,
            bandwidth,
            noiseFigure);

        std::cout << std::fixed << std::setprecision(1)
                  << std::setw(8) << elev
                  << std::setw(12) << range_km
                  << std::setw(10) << result.fspl_dB
                  << std::setw(10) << result.molecularAbsorption_dB
                  << std::setw(10) << result.weatherLoss_dB
                  << std::setw(12) << result.totalPathLoss_dB
                  << std::setw(10) << result.snr_dB
                  << std::setprecision(2)
                  << std::setw(12) << result.shannonCapacity_Gbps
                  << "\n";
    }

    std::cout << "\n-------------------------------------------------------------\n";
    std::cout << "  Shannon capacity: C = B * log2(1 + SNR)\n";
    std::cout << "  Path loss includes: FSPL + molecular absorption + weather\n";
    std::cout << "                      + scintillation + pointing error\n";
    std::cout << "-------------------------------------------------------------\n";

    // ---- Show detailed budget for 20-degree elevation ----
    double refElev = 20.0;
    double refRange = ComputeSlantRange(refElev, altitude);
    ThzNtnLinkBudget::LinkBudgetResult detailResult = linkBudget->ComputeLinkBudget(
        ThzNtnLinkBudget::SAT_TO_GROUND,
        freq,
        refRange,
        refElev,
        txPower,
        txGain,
        rxGain,
        bandwidth,
        noiseFigure);

    std::cout << "\n  Detailed Link Budget at " << refElev << " deg elevation:\n";
    std::cout << "    Tx Power:                " << detailResult.txPower_dBm << " dBm\n";
    std::cout << "    Tx Antenna Gain:         " << detailResult.txAntennaGain_dBi << " dBi\n";
    std::cout << "    EIRP:                    " << detailResult.eirp_dBm << " dBm\n";
    std::cout << "    FSPL:                    " << detailResult.fspl_dB << " dB\n";
    std::cout << "    Molecular Absorption:    " << detailResult.molecularAbsorption_dB << " dB\n";
    std::cout << "    Weather Loss:            " << detailResult.weatherLoss_dB << " dB\n";
    std::cout << "    Scintillation Loss:      " << detailResult.scintillationLoss_dB << " dB\n";
    std::cout << "    Pointing Loss:           " << detailResult.pointingLoss_dB << " dB\n";
    std::cout << "    Total Path Loss:         " << detailResult.totalPathLoss_dB << " dB\n";
    std::cout << "    Rx Antenna Gain:         " << detailResult.rxAntennaGain_dBi << " dBi\n";
    std::cout << "    Rx Power:                " << detailResult.rxPower_dBm << " dBm\n";
    std::cout << "    Noise Power:             " << detailResult.noisePower_dBm << " dBm\n";
    std::cout << "    SNR:                     " << detailResult.snr_dB << " dB\n";
    std::cout << "    HW-limited SNR:          " << detailResult.hardwareLimitedSnr_dB << " dB\n";
    std::cout << "    Shannon Capacity:        " << detailResult.shannonCapacity_Gbps << " Gbps\n";
    std::cout << "    HW-limited Capacity:     " << detailResult.hardwareLimitedCapacity_Gbps
              << " Gbps\n";
    std::cout << "    Link Margin:             " << detailResult.linkMargin_dB << " dB\n";

    // ---- Doppler analysis ----
    double satVelocity = 7.6; // km/s for 550 km LEO
    std::cout << "\n  Doppler Pre-Compensation Analysis:\n";
    for (double elev : elevations)
    {
        double dopplerHz = satPhy->ComputeDopplerPreCompensation_Hz(satVelocity, elev, 0.0);
        std::cout << "    Elev " << std::setw(4) << elev << " deg: Doppler offset = "
                  << std::setprecision(0) << dopplerHz / 1e3 << " kHz\n";
    }

    // ---- Run simulation (minimal, just validates setup) ----
    Simulator::Stop(Seconds(1.0));
    Simulator::Run();
    Simulator::Destroy();

    std::cout << "\n  Simulation complete.\n";
    std::cout << "=============================================================\n";

    return 0;
}
