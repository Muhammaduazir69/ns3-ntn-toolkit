/*
 * SPDX-License-Identifier: GPL-2.0-only
 * Copyright (c) 2026 Muhammad Uzair (ns3-ntn-toolkit, W5)
 *
 * Demo: ground-UE → UAV-relay → HAPS → LEO multi-layer route.
 *
 * Drives the W5 multi-layer router every second across a 1 h scenario and
 * prints the chosen path, range per hop, and elevation per hop. Shows how
 * to compose `SaginHelper` + `MultiLayerRouter` for a SAGIN scenario.
 */
#include "ns3/aeronautical-scenario.h"
#include "ns3/box.h"
#include "ns3/command-line.h"
#include "ns3/constant-position-mobility-model.h"
#include "ns3/constant-velocity-mobility-model.h"
#include "ns3/haps-mobility-model.h"
#include "ns3/multi-layer-router.h"
#include "ns3/sagin-helper.h"
#include "ns3/simulator.h"
#include "ns3/uav-mobility-models.h"

#include <fstream>
#include <iomanip>

using namespace ns3;

int
main(int argc, char* argv[])
{
    double simTimeSec = 3600.0;
    std::string csvPath = "sagin-haps-leo-relay.csv";

    CommandLine cmd(__FILE__);
    cmd.AddValue("simTime", "Simulation duration (s)", simTimeSec);
    cmd.AddValue("csv", "Output CSV path", csvPath);
    cmd.Parse(argc, argv);

    SaginHelper helper;

    Ptr<ConstantPositionMobilityModel> ueMob = CreateObject<ConstantPositionMobilityModel>();
    ueMob->SetPosition(Vector{0.0, 0.0, 0.0});

    auto uav = helper.CreateUavPatrol(Vector{-2000, 0, 100},
                                      Vector{2000, 0, 100}, 25.0);
    auto haps = helper.CreateHaps(Vector{0, 0, 0}, /*alt=*/20000.0, /*radius=*/3000.0);
    Ptr<ConstantVelocityMobilityModel> leo = CreateObject<ConstantVelocityMobilityModel>();
    leo->SetPosition(Vector{-2.0e6, 0.0, 550e3});
    leo->SetVelocity(Vector{7590.0, 0.0, 0.0});

    Ptr<MultiLayerRouter> router = helper.CreateRouter();
    router->AddNode(SaginLayer::Uav, uav);
    router->AddNode(SaginLayer::Haps, haps);
    router->AddNode(SaginLayer::Leo, leo);

    std::ofstream out(csvPath);
    out << "time_s,n_hops,uav_el_deg,uav_range_km,haps_el_deg,haps_range_km,leo_el_deg,leo_range_km\n";

    for (double t = 0; t < simTimeSec; t += 1.0)
    {
        Simulator::Schedule(Seconds(t), [&out, ueMob, router, t]() {
            auto path = router->Route(ueMob);
            int nHops = static_cast<int>(path.size()) - 1;
            double uavEl = -90, uavRng = 0, hapsEl = -90, hapsRng = 0, leoEl = -90, leoRng = 0;
            for (auto& h : path)
            {
                if (h.layer == SaginLayer::Uav) { uavEl = h.elevationDeg; uavRng = h.rangeM / 1000.0; }
                else if (h.layer == SaginLayer::Haps) { hapsEl = h.elevationDeg; hapsRng = h.rangeM / 1000.0; }
                else if (h.layer == SaginLayer::Leo) { leoEl = h.elevationDeg; leoRng = h.rangeM / 1000.0; }
            }
            out << std::fixed << std::setprecision(3)
                << t << "," << nHops << "," << uavEl << "," << uavRng << ","
                << hapsEl << "," << hapsRng << "," << leoEl << "," << leoRng << "\n";
        });
    }

    Simulator::Stop(Seconds(simTimeSec));
    Simulator::Run();
    Simulator::Destroy();

    std::cout << "Wrote " << csvPath << " (" << simTimeSec << " s SAGIN scenario)\n";
    return 0;
}
