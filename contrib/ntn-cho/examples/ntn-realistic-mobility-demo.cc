/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026 Muhammad Uzair
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Demonstrates the per-class realistic mobility generator.
 * Spawns a fixed UE per 3GPP TR 38.811 §6.1.1.1 class and writes its
 * trajectory over 600 s to CSV for inspection / plotting.
 *
 *   ./ns3 run "ntn-realistic-mobility-demo --outputDir=/tmp/mob_demo"
 */
#include "ns3/core-module.h"
#include "ns3/ntn-realistic-mobility.h"

#include <fstream>
#include <iomanip>
#include <iostream>

using namespace ns3;

int
main(int argc, char** argv)
{
    std::string outputDir = "./mob_demo";
    double simTime = 600.0;
    double dt = 1.0;
    uint32_t rngRun = 1;
    CommandLine cmd;
    cmd.AddValue("outputDir", "Output directory", outputDir);
    cmd.AddValue("simTime",   "Simulation duration (s)", simTime);
    cmd.AddValue("dt",        "Time step (s)", dt);
    cmd.AddValue("rngRun",    "RNG seed", rngRun);
    cmd.Parse(argc, argv);

    NtnRealisticMobilityHelper helper(rngRun);

    // One UE per class — ride them through 600 s of motion.
    MobilityProfile fairProfile;
    fairProfile.pStatic     = 1.0 / 7;
    fairProfile.pPedestrian = 1.0 / 7;
    fairProfile.pVehicular  = 1.0 / 7;
    fairProfile.pHst        = 1.0 / 7;
    fairProfile.pMaritime   = 1.0 / 7;
    fairProfile.pAviation   = 1.0 / 7;
    fairProfile.pIot        = 1.0 / 7;

    // Generate 14 UEs so that each class is statistically present
    // (a single uniform draw from 7 classes is noisy with only 7 UEs).
    auto ues = helper.GenerateUes(/*n=*/14, fairProfile,
                                  /*minLat=*/30.0, /*maxLat=*/55.0,
                                  /*minLon=*/-10.0, /*maxLon=*/30.0);

    std::ofstream csv(outputDir + "/mobility_trace.csv");
    csv << "time_s,ue_id,class,lat,lon,alt_m,vEast_mps,vNorth_mps,speed_mps\n";

    std::cout << "Spawned " << ues.size() << " UEs:\n";
    for (const auto& ue : ues)
    {
        std::cout << "  UE " << std::setw(2) << ue.id << "  class=" << ue.className
                  << "  init pos=(" << ue.lat << ", " << ue.lon
                  << ") alt=" << ue.altitude_m << " m"
                  << "  speed=" << ue.speed_mps << " m/s\n";
    }
    std::cout << "\nWriting trajectory to " << outputDir << "/mobility_trace.csv\n";

    for (double t = 0.0; t <= simTime; t += dt)
    {
        for (auto& ue : ues)
        {
            csv << t << "," << ue.id << "," << ue.className << ","
                << ue.lat << "," << ue.lon << "," << ue.altitude_m << ","
                << ue.vEast << "," << ue.vNorth << "," << ue.speed_mps << "\n";
            helper.AdvanceUe(ue, dt);
        }
    }
    csv.close();

    std::cout << "\nFinal positions:\n";
    for (const auto& ue : ues)
    {
        std::cout << "  UE " << std::setw(2) << ue.id << "  " << ue.className
                  << "  pos=(" << ue.lat << ", " << ue.lon << ") alt=" << ue.altitude_m
                  << " m\n";
    }
    return 0;
}
