/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-only
 * Author: Muhammad Uzair
 *
 * Example: D-band (140 GHz) LEO Constellation Access
 *
 * Demonstrates multiple LEO satellites in a Walker constellation providing
 * D-band (140 GHz) access to ground terminals.  Each terminal selects
 * the satellite with the highest elevation angle and computes link quality.
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
class ThzNtnChannelModel;
class ThzNtnLinkBudget;
} // namespace ns3

#include "ns3/thz-ntn-helper.h"
#include "ns3/thz-ntn-channel-model.h"
#include "ns3/thz-ntn-link-budget.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("ThzNtnDbandConstellation");

/**
 * \brief Satellite position in a Walker constellation.
 */
struct SatPosition
{
    uint32_t id;
    double latDeg;
    double lonDeg;
    double altKm;
    double x; // Cartesian x (metres)
    double y; // Cartesian y (metres)
    double z; // Cartesian z (metres)
};

/**
 * \brief Ground terminal position.
 */
struct TerminalPosition
{
    uint32_t id;
    double latDeg;
    double lonDeg;
    double x;
    double y;
    double z;
};

/**
 * \brief Compute elevation angle and slant range between ground and satellite.
 */
static void
ComputeGeometry(const TerminalPosition& gt,
                const SatPosition& sat,
                double& elevDeg,
                double& slantRange_m)
{
    double dx = sat.x - gt.x;
    double dy = sat.y - gt.y;
    double dz = sat.z - gt.z;
    double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
    slantRange_m = dist;

    // Elevation angle: angle above the local horizontal
    double groundDist = std::sqrt(dx * dx + dy * dy);
    if (groundDist < 1.0)
    {
        elevDeg = 90.0;
    }
    else
    {
        elevDeg = std::atan2(dz, groundDist) * 180.0 / M_PI;
    }
}

int
main(int argc, char* argv[])
{
    // ---- Default parameters ----
    uint32_t numSats = 4;
    uint32_t numUts = 10;
    double freq = 140e9;       // 140 GHz D-band
    double altitude = 550.0;   // km
    double txPower = 30.0;     // dBm (1 W, SmallSat)
    double bandwidth = 5e9;    // 5 GHz
    double txGain = 38.0;      // dBi
    double rxGain = 42.0;      // dBi
    double noiseFigure = 8.0;  // dB

    // ---- Parse command-line arguments ----
    CommandLine cmd(__FILE__);
    cmd.AddValue("numSats", "Number of satellites [default: 4]", numSats);
    cmd.AddValue("numUts", "Number of ground terminals [default: 10]", numUts);
    cmd.AddValue("freq", "Carrier frequency in Hz [default: 140e9]", freq);
    cmd.AddValue("altitude", "Orbital altitude in km [default: 550]", altitude);
    cmd.AddValue("txPower", "Satellite Tx power in dBm [default: 30]", txPower);
    cmd.Parse(argc, argv);

    std::cout << "=============================================================\n";
    std::cout << "  D-band (140 GHz) LEO Constellation Access Analysis\n";
    std::cout << "=============================================================\n";
    std::cout << "  Frequency:    " << freq / 1e9 << " GHz\n";
    std::cout << "  Satellites:   " << numSats << " in Walker constellation\n";
    std::cout << "  Terminals:    " << numUts << " ground terminals\n";
    std::cout << "  Altitude:     " << altitude << " km\n";
    std::cout << "  Tx Power:     " << txPower << " dBm\n";
    std::cout << "  Bandwidth:    " << bandwidth / 1e9 << " GHz\n";
    std::cout << "-------------------------------------------------------------\n\n";

    // ---- Create satellite constellation (Walker star) ----
    // Distribute satellites evenly across orbital positions
    std::vector<SatPosition> sats(numSats);
    double orbitRadius = (6371.0 + altitude) * 1000.0; // metres

    for (uint32_t i = 0; i < numSats; i++)
    {
        double angleDeg = 360.0 * i / numSats;
        double angleRad = angleDeg * M_PI / 180.0;
        sats[i].id = i;
        sats[i].latDeg = 0.0; // Equatorial orbit for simplicity
        sats[i].lonDeg = angleDeg;
        sats[i].altKm = altitude;
        sats[i].x = orbitRadius * std::cos(angleRad);
        sats[i].y = orbitRadius * std::sin(angleRad);
        sats[i].z = altitude * 1000.0; // Simplified: height above ground plane
    }

    // ---- Create ground terminals distributed over a region ----
    std::vector<TerminalPosition> terminals(numUts);
    double regionSize = 500e3; // 500 km x 500 km region

    Ptr<UniformRandomVariable> rng = CreateObject<UniformRandomVariable>();
    rng->SetAttribute("Min", DoubleValue(-regionSize / 2.0));
    rng->SetAttribute("Max", DoubleValue(regionSize / 2.0));

    for (uint32_t i = 0; i < numUts; i++)
    {
        terminals[i].id = i;
        terminals[i].x = rng->GetValue();
        terminals[i].y = rng->GetValue();
        terminals[i].z = 0.0;
        terminals[i].latDeg = terminals[i].y / 111e3; // approximate
        terminals[i].lonDeg = terminals[i].x / 111e3;
    }

    // ---- Create ns-3 nodes ----
    NodeContainer satNodes;
    satNodes.Create(numSats);

    NodeContainer gtNodes;
    gtNodes.Create(numUts);

    // Set up mobility
    for (uint32_t i = 0; i < numSats; i++)
    {
        Ptr<ConstantPositionMobilityModel> mob = CreateObject<ConstantPositionMobilityModel>();
        mob->SetPosition(Vector(sats[i].x, sats[i].y, sats[i].z));
        satNodes.Get(i)->AggregateObject(mob);
    }

    for (uint32_t i = 0; i < numUts; i++)
    {
        Ptr<ConstantPositionMobilityModel> mob = CreateObject<ConstantPositionMobilityModel>();
        mob->SetPosition(Vector(terminals[i].x, terminals[i].y, 0.0));
        gtNodes.Get(i)->AggregateObject(mob);
    }

    // ---- Create THz-NTN components ----
    Ptr<ThzNtnHelper> helper = CreateObject<ThzNtnHelper>();

    Ptr<ThzNtnChannelModel> channel = helper->CreateChannelModel("D-band-140GHz");
    channel->EnableMolecularAbsorption(true);
    channel->EnableWeatherEffects(true);

    Ptr<ThzNtnLinkBudget> linkBudget = helper->CreateLinkBudget();

    // ---- For each terminal, find best satellite and compute link quality ----
    std::cout << "  Per-Terminal Link Quality:\n";
    std::cout << "  " << std::string(74, '-') << "\n";
    std::cout << std::setw(6) << "UT"
              << std::setw(10) << "Best Sat"
              << std::setw(10) << "Elev"
              << std::setw(12) << "Range"
              << std::setw(10) << "FSPL"
              << std::setw(10) << "SNR"
              << std::setw(12) << "Capacity"
              << "\n";
    std::cout << std::setw(6) << "ID"
              << std::setw(10) << "ID"
              << std::setw(10) << "(deg)"
              << std::setw(12) << "(km)"
              << std::setw(10) << "(dB)"
              << std::setw(10) << "(dB)"
              << std::setw(12) << "(Gbps)"
              << "\n";
    std::cout << "  " << std::string(74, '-') << "\n";

    double totalCapacity = 0.0;
    uint32_t servedTerminals = 0;

    for (uint32_t t = 0; t < numUts; t++)
    {
        double bestElev = -90.0;
        uint32_t bestSatId = 0;
        double bestRange = 0.0;

        // Find satellite with highest elevation
        for (uint32_t s = 0; s < numSats; s++)
        {
            double elev;
            double range;
            ComputeGeometry(terminals[t], sats[s], elev, range);
            if (elev > bestElev)
            {
                bestElev = elev;
                bestSatId = s;
                bestRange = range;
            }
        }

        double bestRangeKm = bestRange / 1000.0;

        // Compute link budget only if elevation is above minimum (5 deg)
        if (bestElev >= 5.0)
        {
            ThzNtnLinkBudget::LinkBudgetResult result = linkBudget->ComputeLinkBudget(
                ThzNtnLinkBudget::SAT_TO_GROUND,
                freq,
                bestRange,
                bestElev,
                txPower,
                txGain,
                rxGain,
                bandwidth,
                noiseFigure);

            std::cout << std::fixed << std::setprecision(1)
                      << std::setw(6) << t
                      << std::setw(10) << bestSatId
                      << std::setw(10) << bestElev
                      << std::setw(12) << bestRangeKm
                      << std::setw(10) << result.fspl_dB
                      << std::setw(10) << result.snr_dB
                      << std::setprecision(2)
                      << std::setw(12) << result.shannonCapacity_Gbps
                      << "\n";

            totalCapacity += result.shannonCapacity_Gbps;
            servedTerminals++;
        }
        else
        {
            std::cout << std::fixed << std::setprecision(1)
                      << std::setw(6) << t
                      << std::setw(10) << bestSatId
                      << std::setw(10) << bestElev
                      << std::setw(12) << bestRangeKm
                      << std::setw(10) << "N/A"
                      << std::setw(10) << "N/A"
                      << std::setw(12) << "No cover"
                      << "\n";
        }
    }

    // ---- Aggregate statistics ----
    std::cout << "\n  " << std::string(50, '-') << "\n";
    std::cout << "  Constellation Aggregate Results:\n";
    std::cout << "    Terminals served:     " << servedTerminals << " / " << numUts << "\n";
    std::cout << "    Total capacity:       " << std::fixed << std::setprecision(2)
              << totalCapacity << " Gbps\n";
    std::cout << "    Avg capacity/UT:      "
              << (servedTerminals > 0 ? totalCapacity / servedTerminals : 0.0) << " Gbps\n";
    std::cout << "    Coverage ratio:       "
              << std::setprecision(1)
              << (100.0 * servedTerminals / numUts) << " %\n";

    // ---- Satellite load distribution ----
    std::cout << "\n  Satellite Load Distribution:\n";
    std::vector<uint32_t> satLoad(numSats, 0);
    for (uint32_t t = 0; t < numUts; t++)
    {
        double bestElev = -90.0;
        uint32_t bestSatId = 0;
        for (uint32_t s = 0; s < numSats; s++)
        {
            double elev;
            double range;
            ComputeGeometry(terminals[t], sats[s], elev, range);
            if (elev > bestElev)
            {
                bestElev = elev;
                bestSatId = s;
            }
        }
        if (bestElev >= 5.0)
        {
            satLoad[bestSatId]++;
        }
    }
    for (uint32_t s = 0; s < numSats; s++)
    {
        std::cout << "    Satellite " << s << ": " << satLoad[s] << " terminals\n";
    }

    // ---- Run simulation ----
    Simulator::Stop(Seconds(1.0));
    Simulator::Run();
    Simulator::Destroy();

    std::cout << "\n  Simulation complete.\n";
    std::cout << "=============================================================\n";

    return 0;
}
