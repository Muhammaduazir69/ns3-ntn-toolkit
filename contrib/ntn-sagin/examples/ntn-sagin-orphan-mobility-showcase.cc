/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026  Muhammad Uzair
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * ntn-sagin-orphan-mobility-showcase — gives two finished-but-unused toolkit
 * mobility models a real home and a 3D scene:
 *
 *   - sagin::HapsTrajectoryMobilityModel — a HAPS flying a supplied trajectory
 *     (here a programmatic loiter racetrack), interpolated to Simulator::Now().
 *   - ntnv2x::MaritimeMobilityModel — a vessel under way in a sea area.
 *
 * Both were complete classes with no example (flagged in the architecture
 * audit). They run here under a real SGP4 satellite, all in the scenario's
 * local-ENU frame, and the whole scene is streamed to NetSimulyzer + Cesium via
 * the NtnSceneHelper.
 *
 * Usage:
 *   ./ns3 run "ntn-sagin-orphan-mobility-showcase --duration=120 \
 *              --czml=orphans.czml --netSim=orphans.json"
 */

#include "ns3/box.h"
#include "ns3/core-module.h"
#include "ns3/haps-trajectory-mobility-model.h"
#include "ns3/haps-trajectory-trace.h"
#include "ns3/maritime-scenario.h"
#include "ns3/mobility-module.h"
#include "ns3/ntn-scene-helper.h"
#include "ns3/ntn-tr38811-mobility-model.h" // NtnEnuProjectionMobilityModel
#include "ns3/sgp4-mobility-model.h"
#include "ns3/walker-constellation.h"

#include <cmath>
#include <iostream>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("NtnSaginOrphanMobilityShowcase");

int
main(int argc, char* argv[])
{
    double duration = 120.0;
    std::string netSimOut;
    std::string czmlOut;

    CommandLine cmd(__FILE__);
    cmd.AddValue("duration", "Simulation duration (s)", duration);
    cmd.AddValue("netSim", "NetSimulyzer 3D JSON output (empty=off)", netSimOut);
    cmd.AddValue("czml", "Cesium CZML 3D output (empty=off)", czmlOut);
    cmd.Parse(argc, argv);

    // Scenario origin: a coastal site (the local-ENU reference for everything).
    const double refLat = 36.0;
    const double refLon = 15.0;

    // ---- Satellite: real SGP4 projected into the local ENU frame ----
    ns3::ntncon::WalkerConfig wcfg;
    wcfg.num_planes = 1;
    wcfg.total_sats = 40;
    wcfg.altitude_km = 550.0;
    wcfg.inclination_deg = 53.0;
    wcfg.epoch_unix_s = 1735689600.0;
    const auto elements = ns3::ntncon::WalkerConstellation::BuildDelta(wcfg);
    Ptr<ns3::ntncon::Sgp4MobilityModel> satSgp4 = CreateObject<ns3::ntncon::Sgp4MobilityModel>();
    satSgp4->SetElements(elements[0]);
    Ptr<NtnEnuProjectionMobilityModel> satEnu = CreateObject<NtnEnuProjectionMobilityModel>();
    satEnu->SetSource(satSgp4);
    satEnu->SetReference(refLat, refLon, 0.0);
    NodeContainer satNodes;
    satNodes.Create(1);
    satNodes.Get(0)->AggregateObject(satEnu);

    // ---- HAPS: a programmatic loiter racetrack at ~20 km ----
    sagin::HapsTrajectoryTrace trace;
    trace.platform_id = "haps-1";
    for (int k = 0; k <= 36; ++k)
    {
        const double frac = static_cast<double>(k) / 36.0;
        const double ang = 2.0 * M_PI * frac; // one loiter loop over the run
        sagin::HapsTrajectorySample s;
        s.time_s = frac * duration;
        s.lat_deg = refLat + 0.05 * std::sin(ang);
        s.lon_deg = refLon + 0.08 * std::sin(2.0 * ang); // figure-eight ground track
        s.alt_m = 20000.0;
        trace.samples.push_back(s);
    }
    Ptr<sagin::HapsTrajectoryMobilityModel> hapsMob =
        CreateObject<sagin::HapsTrajectoryMobilityModel>();
    hapsMob->SetReference(refLat, refLon, 0.0);
    hapsMob->SetTrace(trace);
    NodeContainer hapsNodes;
    hapsNodes.Create(1);
    hapsNodes.Get(0)->AggregateObject(hapsMob);

    // ---- Vessel: MaritimeMobilityModel under way in a coastal sea box ----
    Ptr<ntnv2x::MaritimeMobilityModel> shipMob = CreateObject<ntnv2x::MaritimeMobilityModel>();
    shipMob->SetAttribute("Speed", DoubleValue(8.0)); // ~15.5 kn
    shipMob->SetSeaArea(Box(-5000.0, 5000.0, -5000.0, 5000.0, 0.0, 0.0));
    shipMob->SetPosition(Vector(0.0, 0.0, 0.0));
    NodeContainer shipNodes;
    shipNodes.Create(1);
    shipNodes.Get(0)->AggregateObject(shipMob);

    std::cout << "\n=== orphan-mobility showcase (HAPS-trajectory + maritime + SGP4) ===\n"
              << "  HAPS loiter @20km, vessel @8 m/s, real LEO pass, local-ENU about ("
              << refLat << "," << refLon << ")\n\n";

    // ---- One scene tap for all of them (local-ENU frame) ----
    ntnobs::NtnSceneHelper scene;
    if (!netSimOut.empty())
    {
        scene.SetNetSimulyzer(netSimOut);
    }
    if (!czmlOut.empty())
    {
        scene.SetCzml(czmlOut);
    }
    scene.SetEnuFrame(refLat, refLon, 0.0);
    // Track sats via the sats slot; HAPS + ship via the "ues"/"gateways" slots
    // (the helper labels them; kind is cosmetic for the 3D view).
    Ptr<ntnobs::NtnSceneRecorder> rec = scene.Build(satNodes, hapsNodes, shipNodes);

    Simulator::Stop(Seconds(duration));
    Simulator::Run();
    if (rec)
    {
        rec->Stop();
        std::cout << "  scene events: " << rec->GetEventCount() << "\n";
    }
    Simulator::Destroy();
    return 0;
}
