/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only
//
// ntn-constellation-sgp4-mobility-traffic — a single LEO satellite from a
// TLE drives ns-3 traffic over a P2P link while ContactGraphScheduler
// tracks GSL up / down events. Uses Sgp4MobilityModel from
// ntn-constellation and the realistic-traffic helper from ntn-traffic.
//
// Cross-module composition:
//   * `ntn-constellation`  : Sgp4MobilityModel + ContactGraphScheduler
//   * `ntn-traffic`        : NtnRealisticTrafficHelper installs traffic
//   * `src/{...}`          : internet / mobility / point-to-point / apps
//
// The ground station defaults to the satellite's t=0 sub-point so a single
// LEO always produces a demonstrative GSL up/down pass; pin --gsLat/--gsLon
// for a fixed site.
//
// Quick test:  --simSeconds=600 --tle=contrib/ntn-rrc/data/iss-zarya.tle

#include "ns3/applications-module.h"
#include "ns3/command-line.h"
#include "ns3/core-module.h"
#include "ns3/flow-monitor-helper.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/point-to-point-helper.h"

#include "ns3/contact-graph-scheduler.h"
#include "ns3/orbital-elements.h"
#include "ns3/sgp4-mobility-model.h"

#include "ns3/ntn-realistic-traffic-helper.h"

#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>

using namespace ns3;
using ns3::ntncon::ContactGraphScheduler;
using ns3::ntncon::Sgp4MobilityModel;
using ns3::ntncon::TleRecord;

namespace
{

bool
ReadThreeLineTle(const std::string& path, TleRecord& tle)
{
    std::ifstream f(path);
    if (!f)
    {
        return false;
    }
    if (!std::getline(f, tle.name) || !std::getline(f, tle.line1) ||
        !std::getline(f, tle.line2))
    {
        return false;
    }
    return tle.line1.size() >= 60 && tle.line2.size() >= 60;
}

} // namespace

int
main(int argc, char* argv[])
{
    double simSeconds = 120.0;
    std::string tlePath;
    // A NaN sentinel means "auto-place the GS under the satellite's t=0
    // sub-point" so a single LEO always produces a demonstrative GSL pass
    // regardless of TLE epoch. Override --gsLat / --gsLon for a fixed site.
    double gsLatDeg = std::nan("");
    double gsLonDeg = std::nan("");
    double minElevDeg = 10.0;
    std::string outputDir = ".";

    CommandLine cmd(__FILE__);
    cmd.AddValue("simSeconds", "Simulation duration (s)", simSeconds);
    cmd.AddValue("tle",
                   "Path to 3-line TLE (default: contrib/ntn-rrc/data/iss-zarya.tle)",
                   tlePath);
    cmd.AddValue("gsLat",
                   "Ground-station latitude (deg; default = sat sub-point)",
                   gsLatDeg);
    cmd.AddValue("gsLon",
                   "Ground-station longitude (deg; default = sat sub-point)",
                   gsLonDeg);
    cmd.AddValue("minElev", "Min elevation for in-contact (deg)",
                   minElevDeg);
    cmd.AddValue("outputDir", "Output dir for sim_health.csv", outputDir);
    cmd.Parse(argc, argv);

    if (tlePath.empty())
    {
        for (const std::string& c : {
                 std::string("contrib/ntn-rrc/data/iss-zarya.tle"),
                 std::string("../contrib/ntn-rrc/data/iss-zarya.tle"),
                 std::string("../../contrib/ntn-rrc/data/iss-zarya.tle"),
             })
        {
            std::ifstream p(c);
            if (p)
            {
                tlePath = c;
                break;
            }
        }
    }
    TleRecord tle;
    if (!ReadThreeLineTle(tlePath, tle))
    {
        std::fprintf(stderr,
                       "error: could not read 3-line TLE from %s\n",
                       tlePath.c_str());
        return 2;
    }

    auto sat = CreateObject<Sgp4MobilityModel>();
    if (!sat->SetTle(tle))
    {
        std::fprintf(stderr, "error: TLE parse failed\n");
        return 2;
    }

    // Auto-place the ground station beneath the satellite's t=0 sub-point
    // (GetGeodetic uses Simulator::Now(), which is 0 before Run) unless the
    // user pinned a site. A GS directly below the orbit guarantees the
    // satellite climbs above the horizon then sets — a real GSL up+down.
    if (std::isnan(gsLatDeg) || std::isnan(gsLonDeg))
    {
        double subLat;
        double subLon;
        double subAlt;
        sat->GetGeodetic(subLat, subLon, subAlt);
        gsLatDeg = subLat;
        gsLonDeg = subLon;
    }

    auto scheduler = CreateObject<ContactGraphScheduler>();
    scheduler->SetMinElevationDeg(minElevDeg);
    scheduler->RegisterGroundStation(0, gsLatDeg, gsLonDeg);
    scheduler->RegisterSatellite(100, sat);
    scheduler->Start();

    NtnRealisticTrafficHelper traffic;
    traffic.SetSimTime(Seconds(simSeconds));
    traffic.SetOutputDir(outputDir);
    traffic.SetRunTag("ntn-constellation-sgp4-mobility-traffic");
    traffic.SetProfile(
        NtnRealisticTrafficHelper::TrafficProfile::MixedBouquet);
    traffic.InstallUes(6);
    traffic.Wire();

    Simulator::Stop(Seconds(simSeconds));
    Simulator::Run();
    scheduler->Stop();
    traffic.WriteHealthReport();
    const uint64_t gslUp = scheduler->GslEventsUp();
    const uint64_t gslDown = scheduler->GslEventsDown();
    Simulator::Destroy();

    std::printf(
        "# === ntn-constellation-sgp4-mobility-traffic summary ===\n"
        "#   TLE         : %s\n"
        "#   GS (lat,lon): (%.4f, %.4f)\n"
        "#   minElev     : %.1f deg\n"
        "#   GSL up=%lu  down=%lu\n",
        tlePath.c_str(), gsLatDeg, gsLonDeg, minElevDeg,
        static_cast<unsigned long>(gslUp),
        static_cast<unsigned long>(gslDown));
    return 0;
}
