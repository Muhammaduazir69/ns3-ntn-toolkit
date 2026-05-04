/*
 * SPDX-License-Identifier: GPL-2.0-only
 * Copyright (c) 2026 Muhammad Uzair (ns3-ntn-toolkit, W5)
 *
 * Demo: 8-UAV swarm — 2 random-waypoint, 4 patrol, 2 search-pattern — with
 * a 3GPP TR 36.777 RMa-AV path-loss spot-check at every tick. Verifies that
 * mobility + channel hooks compose cleanly inside ns-3's event loop.
 */
#include "ns3/a2g-channel-tr36777.h"
#include "ns3/box.h"
#include "ns3/command-line.h"
#include "ns3/constant-position-mobility-model.h"
#include "ns3/sagin-helper.h"
#include "ns3/simulator.h"
#include "ns3/uav-mobility-models.h"

#include <fstream>
#include <iomanip>
#include <vector>

using namespace ns3;

int
main(int argc, char* argv[])
{
    double simTimeSec = 600.0;
    double fcGHz = 2.0;
    std::string csvPath = "sagin-uav-swarm.csv";

    CommandLine cmd(__FILE__);
    cmd.AddValue("simTime", "Simulation duration (s)", simTimeSec);
    cmd.AddValue("fcGHz", "Carrier frequency (GHz)", fcGHz);
    cmd.AddValue("csv", "Output CSV path", csvPath);
    cmd.Parse(argc, argv);

    SaginHelper helper;
    std::vector<Ptr<MobilityModel>> uavs;

    // ground gNB at origin, 35 m mast (RMa-AV scenario)
    Ptr<ConstantPositionMobilityModel> gnb = CreateObject<ConstantPositionMobilityModel>();
    gnb->SetPosition(Vector{0.0, 0.0, 35.0});

    // 2 random-waypoint
    for (int i = 0; i < 2; ++i)
    {
        Box box(-1500, 1500, -1500, 1500, 50, 250);
        uavs.push_back(helper.CreateUavWaypoint(box, /*speed=*/22.0));
    }
    // 4 patrol
    for (int i = 0; i < 4; ++i)
    {
        Vector a{-1500.0 + 750.0 * i, 0.0, 100.0};
        Vector b{1500.0 - 750.0 * i, 0.0, 100.0};
        uavs.push_back(helper.CreateUavPatrol(a, b, /*speed=*/18.0));
    }
    // 2 search
    uavs.push_back(helper.CreateUavSearch(Vector{-1000, -500, 0},
                                          /*leg=*/2000.0, /*strip=*/100.0,
                                          /*nStrips=*/8, /*alt=*/120.0));
    uavs.push_back(helper.CreateUavSearch(Vector{500, -500, 0},
                                          1500.0, 80.0, 6, 80.0));

    std::ofstream out(csvPath);
    out << "time_s,uav_idx,x_m,y_m,z_m,d3d_m,pl_los_db,pl_nlos_db\n";

    for (double t = 0; t < simTimeSec; t += 5.0)
    {
        Simulator::Schedule(Seconds(t), [&out, &uavs, gnb, fcGHz, t]() {
            for (std::size_t i = 0; i < uavs.size(); ++i)
            {
                Vector p = uavs[i]->GetPosition();
                Vector g = gnb->GetPosition();
                Vector d{p.x - g.x, p.y - g.y, p.z - g.z};
                double d3d = std::sqrt(d.x * d.x + d.y * d.y + d.z * d.z);
                double plLos = A2gChannelTr36777::PathLossDb(
                    A2gScenario::RMa_AV, A2gLink::LOS, d3d, fcGHz, p.z);
                double plNlos = A2gChannelTr36777::PathLossDb(
                    A2gScenario::RMa_AV, A2gLink::NLOS, d3d, fcGHz, p.z);
                out << std::fixed << std::setprecision(2)
                    << t << "," << i << "," << p.x << "," << p.y << "," << p.z
                    << "," << d3d << "," << plLos << "," << plNlos << "\n";
            }
        });
    }

    Simulator::Stop(Seconds(simTimeSec));
    Simulator::Run();
    Simulator::Destroy();
    std::cout << "Wrote " << csvPath << " (" << uavs.size() << " UAVs over " << simTimeSec << " s)\n";
    return 0;
}
