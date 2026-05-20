/*
 * SPDX-License-Identifier: GPL-2.0-only
 * Copyright (c) 2026 Muhammad Uzair (ns3-ntn-toolkit, W5)
 *
 * Demo: passenger broadband on a commercial flight (ISL → LEO → aircraft).
 * A 4 000 km flight at 11 km / 250 m/s; LEO satellite passes overhead; the
 * router selects LEO directly (no UAV/HAPS in this scenario).
 */
#include "ns3/aeronautical-scenario.h"
#include "ns3/command-line.h"
#include "ns3/constant-velocity-mobility-model.h"
#include "ns3/multi-layer-router.h"
#include "ns3/sagin-helper.h"
#include "ns3/simulator.h"
#include "ns3/ntn-realistic-traffic-helper.h"

#include <fstream>
#include <iomanip>

using namespace ns3;

int
main(int argc, char* argv[])
{
    double simTimeSec = 3600.0;
    std::string outputDir = ".";
    double cruiseAltM = 11000.0;
    double speedMps = 250.0;
    std::string csvPath = "sagin-aeronautical.csv";

    CommandLine cmd(__FILE__);
    cmd.AddValue("simTime", "Simulation duration (s)", simTimeSec);
    cmd.AddValue("cruise", "Cruise altitude (m)", cruiseAltM);
    cmd.AddValue("speed", "Cruise speed (m/s)", speedMps);
    cmd.AddValue("csv", "Output CSV path", csvPath);
    cmd.AddValue("outputDir", "Output directory for sim_health.csv", outputDir);
    cmd.Parse(argc, argv);

    SaginHelper helper;

    auto ac = helper.CreateFlight(Vector{0, 0, 0},
                                  Vector{2.0e6, 0, 0}, // 2000 km along +x
                                  cruiseAltM, speedMps);

    Ptr<ConstantVelocityMobilityModel> leo = CreateObject<ConstantVelocityMobilityModel>();
    leo->SetPosition(Vector{-2.0e6, 0.0, 550e3});
    leo->SetVelocity(Vector{7590.0, 0.0, 0.0});

    Ptr<MultiLayerRouter> router = helper.CreateRouter();
    router->AddNode(SaginLayer::Leo, leo);

    std::ofstream out(csvPath);
    out << "time_s,ac_x_m,ac_y_m,ac_alt_m,leo_x_m,leo_y_m,leo_z_m,leo_el_deg,slant_km\n";

    for (double t = 0; t < simTimeSec; t += 5.0)
    {
        Simulator::Schedule(Seconds(t), [&out, ac, leo, router, t]() {
            auto path = router->Route(ac);
            double el = -90, rng = 0;
            for (auto& h : path)
            {
                if (h.layer == SaginLayer::Leo)
                {
                    el = h.elevationDeg;
                    rng = h.rangeM / 1000.0;
                }
            }
            Vector ap = ac->GetPosition();
            Vector lp = leo->GetPosition();
            out << std::fixed << std::setprecision(2) << t << ","
                << ap.x << "," << ap.y << "," << ap.z << ","
                << lp.x << "," << lp.y << "," << lp.z << ","
                << el << "," << rng << "\n";
        });
    }
    // ==== v2 realistic traffic plane (auto-injected) =====================
    NtnRealisticTrafficHelper _ntn_traffic;
    _ntn_traffic.SetSimTime(Seconds(simTimeSec));
    _ntn_traffic.SetOutputDir(outputDir);
    _ntn_traffic.SetRunTag("sagin-aeronautical");
    _ntn_traffic.SetProfile(NtnRealisticTrafficHelper::TrafficProfile::MixedBouquet);
    _ntn_traffic.InstallUes(8);
    _ntn_traffic.Wire();

    

    Simulator::Stop(Seconds(simTimeSec));
    Simulator::Run();
    _ntn_traffic.WriteHealthReport();
    Simulator::Destroy();
    std::cout << "Wrote " << csvPath << " (" << simTimeSec << " s commercial flight)\n";
    return 0;
}
