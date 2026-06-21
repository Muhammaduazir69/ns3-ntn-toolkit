/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026  Muhammad Uzair
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * ntn-netsimulyzer-official-demo — Path B of the 3D-visualization plan.
 *
 * The toolkit's primary 3D path is the home-grown NtnSceneRecorder (emits a
 * NetSimulyzer-1.0 JSON + Cesium CZML from the same tap). This example shows the
 * SECOND, schema-exact path: driving the OFFICIAL usnistgov NetSimulyzer ns-3
 * module (vendored at contrib/netsimulyzer, v1.0.13) from the toolkit's REAL
 * mobility models — SGP4 satellites + TR 38.811 UEs. The official Orchestrator
 * auto-polls each node's MobilityModel, so once NodeConfiguration is installed
 * the trace updates from genuine orbital dynamics.
 *
 * Coordinate note: the toolkit's satellites report ECEF metres. NetSimulyzer is
 * a local-Cartesian player with no globe, so positions render at planetary scale
 * about an Earth sphere placed at the origin — the established community pattern
 * (satellites are given a large display Scale so they remain visible). Use the
 * NtnSceneRecorder + Cesium path for a true geographic globe.
 *
 * Usage:
 *   ./ns3 run "ntn-netsimulyzer-official-demo --sats=6 --ues=4 --duration=30 \
 *              --out=ntn-official.json"
 */

#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include "ns3/netsimulyzer-module.h"
#include "ns3/ntn-tr38811-mobility-model.h"
#include "ns3/sgp4-mobility-model.h"
#include "ns3/walker-constellation.h"

#include <iostream>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("NtnNetSimulyzerOfficialDemo");

int
main(int argc, char* argv[])
{
    uint32_t sats = 6;
    uint32_t ues = 4;
    double duration = 30.0;
    double altitudeKm = 550.0;
    std::string out = "ntn-official.json";

    CommandLine cmd(__FILE__);
    cmd.AddValue("sats", "Satellites to render", sats);
    cmd.AddValue("ues", "UEs to render", ues);
    cmd.AddValue("duration", "Simulation duration (s)", duration);
    cmd.AddValue("altitude", "Constellation altitude (km)", altitudeKm);
    cmd.AddValue("out", "Official NetSimulyzer JSON output path", out);
    cmd.Parse(argc, argv);

    // ---- REAL SGP4 satellites (ECEF) ----
    ns3::ntncon::WalkerConfig wcfg;
    wcfg.num_planes = 1;
    wcfg.total_sats = std::max<uint32_t>(sats, 1);
    wcfg.altitude_km = altitudeKm;
    wcfg.inclination_deg = 53.0;
    wcfg.epoch_unix_s = 1735689600.0;
    const auto elements = ns3::ntncon::WalkerConstellation::BuildDelta(wcfg);

    NodeContainer satNodes;
    satNodes.Create(sats);
    for (uint32_t s = 0; s < sats; ++s)
    {
        Ptr<ns3::ntncon::Sgp4MobilityModel> m = CreateObject<ns3::ntncon::Sgp4MobilityModel>();
        m->SetElements(elements[s]);
        satNodes.Get(s)->AggregateObject(m);
    }

    // ---- REAL TR 38.811 UEs (ECEF) under the first satellite's sub-point ----
    double subLat, subLon, subAlt;
    satNodes.Get(0)->GetObject<ns3::ntncon::Sgp4MobilityModel>()->GetGeodetic(subLat, subLon, subAlt);
    NodeContainer ueNodes;
    ueNodes.Create(ues);
    NtnTr38811MobilityHelper ueMob(/*seed=*/1);
    auto profile = NtnMobilityScenarios::MixedContinental();
    ueMob.Install(ueNodes, profile, subLat - 0.05, subLat + 0.05, subLon - 0.05, subLon + 0.05);

    // ---- Earth reference sphere at the ECEF origin (NetSimulyzer has no globe) ----
    NodeContainer earth;
    earth.Create(1);
    MobilityHelper em;
    em.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    em.Install(earth);
    earth.Get(0)->GetObject<MobilityModel>()->SetPosition(Vector(0, 0, 0));

    // ---- Official NetSimulyzer orchestrator + node configurations ----
    auto orchestrator = CreateObject<netsimulyzer::Orchestrator>(out);

    netsimulyzer::NodeConfigurationHelper nc{orchestrator};

    // Earth sphere sized to the real radius so the ECEF nodes sit on its surface.
    nc.Set("Model", netsimulyzer::models::SPHERE_VALUE);
    nc.Set("Scale", DoubleValue(6371000.0));
    nc.Install(earth.Get(0));

    // Satellites: diamonds, enlarged so they read at planetary scale.
    nc.Set("Model", netsimulyzer::models::DIAMOND_VALUE);
    nc.Set("Scale", DoubleValue(150000.0));
    nc.Install(satNodes);

    // UEs: smartphones (also enlarged to be visible from orbit-scale views).
    nc.Set("Model", netsimulyzer::models::SMARTPHONE_VALUE);
    nc.Set("Scale", DoubleValue(80000.0));
    nc.Install(ueNodes);

    std::cout << "\n=== ntn-netsimulyzer-official-demo (Path B: official module) ===\n"
              << "  " << sats << " SGP4 sats + " << ues << " TR 38.811 UEs -> " << out << "\n"
              << "  open with the NetSimulyzer desktop app (Qt6).\n\n";

    Simulator::Stop(Seconds(duration));
    Simulator::Run();
    Simulator::Destroy();
    std::cout << "  wrote official NetSimulyzer trace: " << out << "\n";
    return 0;
}
