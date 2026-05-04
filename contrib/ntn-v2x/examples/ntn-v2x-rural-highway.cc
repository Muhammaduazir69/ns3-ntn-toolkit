/*
 * SPDX-License-Identifier: GPL-2.0-only
 * Copyright (c) 2026 Muhammad Uzair (ns3-ntn-toolkit, W7)
 *
 * Demo: 100 vehicles on a rural highway pulling connectivity from a
 * single LEO satellite. Drives the SUMO TraCI bridge in trace-replay
 * mode (so CI does not need a live SUMO), then runs the V2X-LEO relay
 * to decide which vehicles use direct uplink vs. peer relay.
 *
 * Validation gates exercised here:
 *   - TraCI bridge sync jitter (< 100 ms) — measured live
 *   - 100-vehicle 5-min run completes
 *   - Per-second link-budget snapshot logged
 */
#include "ns3/command-line.h"
#include "ns3/constant-position-mobility-model.h"
#include "ns3/constant-velocity-mobility-model.h"
#include "ns3/ntn-v2x-helper.h"
#include "ns3/simulator.h"
#include "ns3/sumo-traci-bridge.h"
#include "ns3/v2x-leo-direct.h"
#include "ns3/v2x-leo-relay.h"

#include <fstream>
#include <iomanip>

using namespace ns3;
using namespace ns3::ntnv2x;

int
main(int argc, char* argv[])
{
    std::size_t nVehicles = 100;
    double simTimeSec = 300.0;  // 5-min validation gate
    double dtSec = 1.0;
    std::string tracePath = "/tmp/ntn-v2x-fcd.csv";
    std::string csvPath = "ntn-v2x-rural-highway.csv";
    double minDirectSnrDb = 4.0;   // realistic Starlink-class threshold for demo
    double maxV2vRangeM = 1500.0;

    CommandLine cmd(__FILE__);
    cmd.AddValue("vehicles", "Number of vehicles", nVehicles);
    cmd.AddValue("simTime", "Simulation duration (s)", simTimeSec);
    cmd.AddValue("dt", "TraCI tick (s)", dtSec);
    cmd.AddValue("trace", "FCD CSV trace path (generated if missing)", tracePath);
    cmd.AddValue("minDirectSnr", "Minimum dB for direct uplink", minDirectSnrDb);
    cmd.AddValue("maxV2vRange", "Maximum V2V range (m) for relay", maxV2vRangeM);
    cmd.AddValue("csv", "Output CSV", csvPath);
    cmd.Parse(argc, argv);

    NtnV2xHelper::WriteSyntheticFcdCsv(tracePath, nVehicles,
                                       /*roadLengthM=*/30000.0,
                                       simTimeSec, dtSec);

    Ptr<SumoTraciBridge> bridge = CreateObject<SumoTraciBridge>();
    if (!bridge->LoadFcdTrace(tracePath))
    {
        std::cerr << "failed to load FCD trace\n";
        return 1;
    }

    // LEO satellite — ConstantVelocity east-bound at 7590 m/s, 550 km up.
    Ptr<ConstantVelocityMobilityModel> sat = CreateObject<ConstantVelocityMobilityModel>();
    sat->SetPosition(Vector{-2.0e6, 0.0, 550e3});
    sat->SetVelocity(Vector{7590.0, 0.0, 0.0});

    Ptr<V2xLeoRelay> relay = CreateObject<V2xLeoRelay>();
    relay->SetSatellite(sat);
    relay->SetMaxV2vRangeM(maxV2vRangeM);
    relay->SetMinDirectSnrDb(minDirectSnrDb);

    std::vector<Ptr<MobilityModel>> vMobs;
    vMobs.reserve(nVehicles);
    for (std::size_t i = 0; i < nVehicles; ++i)
    {
        Ptr<ConstantPositionMobilityModel> mob = CreateObject<ConstantPositionMobilityModel>();
        mob->SetPosition(Vector{-1.0e9, 0, 0}); // sentinel until first sample
        std::string id = "veh" + std::to_string(i);
        bridge->RegisterVehicle(id, mob);
        relay->RegisterVehicle(id, mob);
        vMobs.push_back(mob);
    }

    std::ofstream out(csvPath);
    out << "time_s,n_direct,n_relay,n_orphan,direct_pct,jitter_ms,best_snr_db\n";

    int nSteps = static_cast<int>(simTimeSec / dtSec);
    for (int step = 0; step <= nSteps; ++step)
    {
        double t = step * dtSec;
        Simulator::Schedule(Seconds(t), [t, &out, bridge, relay]() {
            bridge->Step();
            auto decisions = relay->EvaluateAll();
            int nDirect = 0, nRelay = 0, nOrphan = 0;
            double bestSnr = -1e9;
            for (auto& d : decisions)
            {
                if (d.directToLeo)
                    nDirect += 1;
                else if (!d.relayPeerId.empty())
                    nRelay += 1;
                else
                    nOrphan += 1;
                if (d.directSnrDb > bestSnr)
                    bestSnr = d.directSnrDb;
            }
            double directPct = decisions.empty()
                                   ? 0.0
                                   : 100.0 * nDirect / decisions.size();
            out << std::fixed << std::setprecision(3) << t << ","
                << nDirect << "," << nRelay << "," << nOrphan << ","
                << directPct << ","
                << bridge->GetLastJitterSec() * 1000.0 << "," << bestSnr << "\n";
        });
    }

    Simulator::Stop(Seconds(simTimeSec + 1));
    Simulator::Run();
    Simulator::Destroy();

    std::cout << "ntn-v2x-rural-highway done.\n"
              << "  vehicles      : " << nVehicles << "\n"
              << "  simTime       : " << simTimeSec << " s\n"
              << "  trace samples : " << bridge->LoadedSampleCount() << "\n"
              << "  max jitter    : " << bridge->GetMaxJitterSec() * 1000.0 << " ms\n"
              << "  csv           : " << csvPath << "\n";
    return 0;
}
