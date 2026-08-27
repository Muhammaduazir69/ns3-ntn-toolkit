/*
 * SPDX-License-Identifier: GPL-2.0-only
 * Copyright (c) 2026 Muhammad Uzair (ns3-ntn-toolkit, W5)
 *
 * Demo: passenger broadband on a commercial flight (LEO → aircraft). A great-
 * circle cruise leg at 11 km / 250 m/s; a LEO satellite passes overhead and
 * serves the aircraft terminal over a REAL mmwave NR NTN cell (NtnRealStack
 * Helper). The MultiLayerRouter still selects the access layer and logs the
 * per-hop elevation/range geometry, but the radio KPIs (SINR/TBLER/throughput)
 * are now MEASURED off the mmwave PHY trace — no closed-form SINR, no P2P star.
 *
 * WHAT THE ROUTER DOES HERE, AND WHAT IT DOES NOT (audit SAGIN-6).
 *
 * MultiLayerRouter::Route() is called once a second and its result is written
 * to the CSV. That is ALL it does in this example: the route is not actuated
 * onto any data plane. The real stack built below is a single UAV-to-ground
 * access cell, and its behaviour does not depend on which relay layer the
 * router picked - so the CSV's route columns cannot be cross-checked against
 * delivered traffic, and no measured KPI here validates the routing decision.
 *
 * Read the route columns as a GEOMETRY TRACE: which layer was visible, at what
 * elevation and range, second by second. That is genuinely useful and it is
 * what this example is for. It is not a demonstration that traffic followed
 * the chosen path.
 *
 * The actuation pattern exists in this same module: sagin-multihop-traffic.cc
 * gates a P2P leg per candidate LEO (ApplyRoute -> SetGatedLink ->
 * RecomputeRoutingTables) so its route column CAN be validated against per-leg
 * MacRx bytes. Use that example when the question is whether the router steers
 * traffic; use this one when the question is what the sky looked like.
 */
#include "ns3/command-line.h"
#include "ns3/ntn-tr38811-mobility-model.h"
#include "ns3/sgp4-mobility-model.h"
#include "ns3/walker-constellation.h"
#include "ns3/constant-velocity-mobility-model.h"
#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include "ns3/multi-layer-router.h"
#include "ns3/network-module.h"
#include "ns3/ntn-real-stack-helper.h"
#include "ns3/sagin-helper.h"

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>

using namespace ns3;

int
main(int argc, char* argv[])
{
    double simTimeSec = 30.0;
    uint32_t numUes = 1;
    double cruiseAltM = 11000.0;
    double speedMps = 250.0;
    double leoAltKm = 550.0;
    double satEirpDbm = -1.0; // sentinel: backend-appropriate default chosen below
    std::string outputDir = "sagin-aeronautical-output";
    std::string csvPath = "sagin-aeronautical.csv";
    std::string radio = "nr"; // radio backend: "nr" (5G-LENA FR1) | "mmwave" (FR2)

    CommandLine cmd(__FILE__);
    cmd.AddValue("simTime", "Simulation duration (s)", simTimeSec);
    cmd.AddValue("numUes", "Number of aircraft terminals (UEs)", numUes);
    cmd.AddValue("cruise", "Cruise altitude (m)", cruiseAltM);
    cmd.AddValue("speed", "Cruise speed (m/s)", speedMps);
    cmd.AddValue("leoAltKm", "LEO altitude (km)", leoAltKm);
    cmd.AddValue("satEirpDbm", "Satellite EIRP / gNB Tx power (dBm); -1 = backend default",
                 satEirpDbm);
    cmd.AddValue("radio", "Radio backend: nr (FR1) or mmwave", radio);
    cmd.AddValue("csv", "Output CSV path (router geometry)", csvPath);
    cmd.AddValue("outputDir", "Output directory", outputDir);
    cmd.Parse(argc, argv);

    // Backend-appropriate EIRP default (honoured only if the user did not set it):
    // nr's Friis LEO link needs ~70 dBm for a healthy SINR; mmwave keeps 55 dBm.
    if (satEirpDbm < 0.0)
    {
        satEirpDbm = (radio == "mmwave") ? 55.0 : 70.0;
    }

    // Aircraft terminal = UE (great-circle cruise); LEO satellite = mmwave gNB.
    NodeContainer ueNodes;
    ueNodes.Create(numUes);
    Ptr<ConstantVelocityMobilityModel> acMob = CreateObject<ConstantVelocityMobilityModel>();
    acMob->SetPosition(Vector(0.0, 0.0, cruiseAltM));
    acMob->SetVelocity(Vector(speedMps, 0.0, 0.0));
    ueNodes.Get(0)->AggregateObject(acMob);
    MobilityHelper mh;
    mh.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    for (uint32_t i = 1; i < numUes; ++i)
    {
        Ptr<ConstantVelocityMobilityModel> m = CreateObject<ConstantVelocityMobilityModel>();
        m->SetPosition(Vector(1500.0 * i, 0.0, cruiseAltM));
        m->SetVelocity(Vector(speedMps, 0.0, 0.0));
        ueNodes.Get(i)->AggregateObject(m);
    }

    NodeContainer satNodes;
    satNodes.Create(1);
    // Real SGP4 orbit projected into the scenario's local ENU frame: the
    // satellite passes overhead near t=0 and recedes with genuine orbital
    // dynamics (no straight-line placeholder).
    ns3::ntncon::WalkerConfig wcfgSat;
    wcfgSat.num_planes = 1;
    wcfgSat.total_sats = 80;
    wcfgSat.altitude_km = leoAltKm;
    wcfgSat.inclination_deg = 53.0;
    wcfgSat.epoch_unix_s = 1735689600.0;
    const auto satElements = ns3::ntncon::WalkerConstellation::BuildDelta(wcfgSat);
    Ptr<ns3::ntncon::Sgp4MobilityModel> satSgp4 =
        CreateObject<ns3::ntncon::Sgp4MobilityModel>();
    satSgp4->SetElements(satElements[0]);
    double satSubLat, satSubLon, satSubAlt;
    satSgp4->GetGeodetic(satSubLat, satSubLon, satSubAlt);
    Ptr<NtnEnuProjectionMobilityModel> leo = CreateObject<NtnEnuProjectionMobilityModel>();
    leo->SetSource(satSgp4);
    leo->SetReference(satSubLat, satSubLon, 0.0);
    satNodes.Get(0)->AggregateObject(leo);

    // Multi-layer router: selects the LEO access layer, logs per-hop geometry.
    SaginHelper helper;
    Ptr<MultiLayerRouter> router = helper.CreateRouter();
    router->AddNode(SaginLayer::Leo, leo);

    std::filesystem::create_directories(outputDir);
    std::ofstream out(outputDir + "/" + csvPath);
    out << "time_s,ac_x_m,ac_alt_m,leo_x_m,leo_z_m,leo_el_deg,slant_km\n";
    for (double t = 0; t < simTimeSec; t += 2.0)
    {
        Simulator::Schedule(Seconds(t), [&out, acMob, leo, router, t]() {
            auto path = router->Route(acMob);
            double el = -90, rng = 0;
            for (auto& h : path)
            {
                if (h.layer == SaginLayer::Leo)
                {
                    el = h.elevationDeg;
                    rng = h.rangeM / 1000.0;
                }
            }
            Vector ap = acMob->GetPosition();
            Vector lp = leo->GetPosition();
            out << std::fixed << std::setprecision(2) << t << "," << ap.x << "," << ap.z << ","
                << lp.x << "," << lp.z << "," << el << "," << rng << "\n";
        });
    }

    // Real mmwave NR cell over the flight + measured KPIs.
    NtnRealStackHelper rs;
    rs.SetSimTime(Seconds(simTimeSec));
    rs.SetOutputDir(outputDir);
    rs.SetRunTag("sagin-aeronautical");
    // NT-02: declared as CONDUCTED power at the array input. This carrier has
    // no TR 38.821 Set-1 reference in the toolkit, so the EIRP health gate
    // reports "not asserted" rather than certifying an uncalibrated budget.
    rs.SetSatConductedPowerDbm(satEirpDbm);
    rs.SetRadioBackend(radio == "mmwave" ? NtnRealStackHelper::RadioBackend::Mmwave
                                         : NtnRealStackHelper::RadioBackend::Nr);
    if (radio != "mmwave")
    {
        rs.SetNumerology(1); // FR1 30 kHz SCS
    }
    rs.Build(satNodes, ueNodes);
    rs.InstallTraffic(NtnRealStackHelper::TrafficProfile::EmbbStreaming,
                      Seconds(1.0), Seconds(simTimeSec - 0.5));
    rs.EnableAiFlowMonitor("sagin-aeronautical"); // WS2 KPM series (TS 28.552 names)

    Simulator::Stop(Seconds(simTimeSec));
    Simulator::Run();
    rs.Collect();
    rs.WriteHealthReport();
    out.close();

    std::cout << "sagin-aeronautical complete (commercial flight on a real mmwave NR cell).\n"
              << "  measured mean SINR : " << rs.GetMeanDlSinrDb() << " dB\n"
              << "  measured throughput: " << rs.GetRxThroughputMbps() << " Mbps\n"
              << "  router geometry csv: " << outputDir << "/" << csvPath << "\n";
    Simulator::Destroy();
    return 0;
}
