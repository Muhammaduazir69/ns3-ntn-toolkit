/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only
//
// ntn-constellation-walker-traffic — a Walker-Delta constellation drives a
// real UDP data plane between two geographically-separated ground stations,
// while the ContactGraphScheduler tracks per-link visibility changes. The
// data plane (NtnRealisticTrafficHelper from ntn-traffic) keeps the
// simulator event queue busy and writes sim_health.csv with packets_tx,
// packets_rx, etc.
//
// Cross-module composition:
//   * `ntn-constellation`            : WalkerConstellation::BuildDelta +
//                                       Sgp4MobilityModel + ContactGraphScheduler
//   * `ntn-traffic` (helper)         : NtnRealisticTrafficHelper
//   * `src/{internet, point-to-point, mobility, applications, flow-monitor}`
//
// Quick test:  --simSeconds=120 --numPlanes=4 --satsPerPlane=11

#include "ns3/applications-module.h"
#include "ns3/command-line.h"
#include "ns3/core-module.h"
#include "ns3/flow-monitor-helper.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/point-to-point-helper.h"

#include "ns3/contact-graph-scheduler.h"
#include "ns3/sgp4-mobility-model.h"
#include "ns3/walker-constellation.h"

#include "ns3/ntn-realistic-traffic-helper.h"

#include <cmath>
#include <cstdio>
#include <vector>

using namespace ns3;
using ns3::ntncon::ContactEvent;
using ns3::ntncon::ContactGraphScheduler;
using ns3::ntncon::Sgp4MobilityModel;
using ns3::ntncon::WalkerConfig;
using ns3::ntncon::WalkerConstellation;

int
main(int argc, char* argv[])
{
    double simSeconds = 120.0;
    uint32_t numPlanes = 4;
    uint32_t satsPerPlane = 11;
    double altKm = 550.0;
    double inclinationDeg = 53.0;
    double islRangeCapKm = 5000.0;
    std::string outputDir = ".";
    std::string runTag = "ntn-constellation-walker-traffic";

    CommandLine cmd(__FILE__);
    cmd.AddValue("simSeconds", "Simulation duration (s)", simSeconds);
    cmd.AddValue("numPlanes", "Walker orbital planes", numPlanes);
    cmd.AddValue("satsPerPlane", "Satellites per plane", satsPerPlane);
    cmd.AddValue("altKm", "Satellite altitude (km)", altKm);
    cmd.AddValue("inclinationDeg", "Orbital inclination (deg)",
                   inclinationDeg);
    cmd.AddValue("islRangeCapKm",
                   "Max ISL range for contact graph (km)",
                   islRangeCapKm);
    cmd.AddValue("outputDir", "Output dir for sim_health.csv", outputDir);
    cmd.Parse(argc, argv);

    // ---- Build constellation ---------------------------------------------
    WalkerConfig wcfg;
    wcfg.num_planes = numPlanes;
    wcfg.total_sats = numPlanes * satsPerPlane;
    wcfg.altitude_km = altKm;
    wcfg.inclination_deg = inclinationDeg;
    wcfg.epoch_unix_s = 1735689600.0; // 2025-01-01
    const auto elements = WalkerConstellation::BuildDelta(wcfg);

    std::vector<Ptr<Sgp4MobilityModel>> sats;
    sats.reserve(elements.size());
    for (const auto& el : elements)
    {
        auto m = CreateObject<Sgp4MobilityModel>();
        m->SetElements(el);
        sats.push_back(m);
    }

    // Two ground stations on the same latitude band, separated in lon —
    // each at lat ~ 35°, separated by 90° longitude.
    auto scheduler = CreateObject<ContactGraphScheduler>();
    scheduler->SetMinElevationDeg(10.0);
    scheduler->SetMaxIslRangeM(islRangeCapKm * 1e3);
    scheduler->RegisterGroundStation(0, /*lat=*/35.0, /*lon=*/-75.0);
    scheduler->RegisterGroundStation(1, /*lat=*/35.0, /*lon=*/ 15.0);
    for (size_t i = 0; i < sats.size(); ++i)
    {
        scheduler->RegisterSatellite(static_cast<uint32_t>(i + 100),
                                       sats[i]);
    }

    // ---- Drop in realistic UDP data plane --------------------------------
    NtnRealisticTrafficHelper traffic;
    traffic.SetSimTime(Seconds(simSeconds));
    traffic.SetOutputDir(outputDir);
    traffic.SetRunTag(runTag);
    traffic.SetProfile(
        NtnRealisticTrafficHelper::TrafficProfile::MixedBouquet);
    traffic.InstallUes(8);
    traffic.Wire();

    scheduler->Start();
    Simulator::Stop(Seconds(simSeconds));
    Simulator::Run();
    scheduler->Stop();
    traffic.WriteHealthReport();

    const uint64_t gslUp = scheduler->GslEventsUp();
    const uint64_t gslDown = scheduler->GslEventsDown();
    const uint64_t islUp = scheduler->IslEventsUp();
    const uint64_t islDown = scheduler->IslEventsDown();
    Simulator::Destroy();

    std::printf(
        "# === ntn-constellation-walker-traffic summary ===\n"
        "#   Walker: planes=%u  sats/plane=%u  total=%u  altKm=%.0f  inc=%.1f deg\n"
        "#   GS-A=(lat=35,lon=-75)  GS-B=(lat=35,lon=15)  ISL cap=%.0f km\n"
        "#   GSL up=%lu down=%lu  ISL up=%lu down=%lu\n",
        numPlanes, satsPerPlane,
        static_cast<unsigned>(numPlanes * satsPerPlane),
        altKm, inclinationDeg, islRangeCapKm,
        static_cast<unsigned long>(gslUp),
        static_cast<unsigned long>(gslDown),
        static_cast<unsigned long>(islUp),
        static_cast<unsigned long>(islDown));
    return 0;
}
