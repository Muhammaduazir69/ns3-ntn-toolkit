/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only
//
// ntn-constellation-sgp4-mobility-traffic — a single LEO satellite from a TLE
// drives a REAL mmwave NR NTN cell (NtnRealStackHelper: SpectrumPhy + MAC +
// HARQ + RLC/PDCP + RRC + EPC) toward a ground terminal, while the
// ContactGraphScheduler tracks GSL up/down events from the live SGP4 geometry.
// The ground station is auto-placed at the satellite's t=0 sub-point so a real
// rise->zenith->set pass occurs, and the radio KPIs (SINR/TBLER/throughput) are
// MEASURED off the mmwave PHY trace — no closed-form SINR, no P2P star.
//
// Quick test:  --simSeconds=20 --tle=contrib/ntn-rrc/data/iss-zarya.tle

#include "ns3/command-line.h"
#include "ns3/core-module.h"
#include "ns3/mmwave-enb-net-device.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/ntn-real-stack-helper.h"
#include "ns3/ntn-tr38811-mobility-model.h"

#include "ns3/contact-graph-scheduler.h"
#include "ns3/sgp4-mobility-model.h"

#include <cmath>
#include <cstdio>
#include <fstream>

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

Vector
GeodeticToEcef(double latDeg, double lonDeg, double altM)
{
    constexpr double kA = 6378137.0;
    constexpr double kF = 1.0 / 298.257223563;
    constexpr double kE2 = kF * (2.0 - kF);
    const double latR = latDeg * M_PI / 180.0;
    const double lonR = lonDeg * M_PI / 180.0;
    const double s = std::sin(latR), c = std::cos(latR);
    const double N = kA / std::sqrt(1.0 - kE2 * s * s);
    return Vector((N + altM) * c * std::cos(lonR),
                  (N + altM) * c * std::sin(lonR),
                  (N * (1.0 - kE2) + altM) * s);
}
} // namespace

int
main(int argc, char* argv[])
{
    double simSeconds = 20.0;
    uint32_t numUes = 4;
    double satEirpDbm = 58.0;
    std::string tlePath;
    double gsLatDeg = std::nan("");
    double gsLonDeg = std::nan("");
    double minElevDeg = 10.0;
    std::string outputDir = "ntn-constellation-sgp4-output";

    CommandLine cmd(__FILE__);
    cmd.AddValue("simSeconds", "Simulation duration (s)", simSeconds);
    cmd.AddValue("numUes", "Number of ground UEs", numUes);
    cmd.AddValue("satEirpDbm", "Satellite EIRP / gNB Tx power (dBm)", satEirpDbm);
    cmd.AddValue("tle", "Path to 3-line TLE (default: contrib/ntn-rrc/data/iss-zarya.tle)", tlePath);
    cmd.AddValue("gsLat", "Ground-station latitude (deg; default = sat sub-point)", gsLatDeg);
    cmd.AddValue("gsLon", "Ground-station longitude (deg; default = sat sub-point)", gsLonDeg);
    cmd.AddValue("minElev", "Min elevation for in-contact (deg)", minElevDeg);
    cmd.AddValue("outputDir", "Output directory", outputDir);
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
        std::fprintf(stderr, "error: could not read 3-line TLE from %s\n", tlePath.c_str());
        return 2;
    }

    auto sat = CreateObject<Sgp4MobilityModel>();
    if (!sat->SetTle(tle))
    {
        std::fprintf(stderr, "error: TLE parse failed\n");
        return 2;
    }
    double subLat, subLon, subAlt;
    sat->GetGeodetic(subLat, subLon, subAlt);
    if (std::isnan(gsLatDeg) || std::isnan(gsLonDeg))
    {
        gsLatDeg = subLat;
        gsLonDeg = subLon;
    }
    const Vector gsEcef = GeodeticToEcef(gsLatDeg, gsLonDeg, 540.0);

    // ---- nodes: the SGP4 satellite (real mmwave gNB) + ground UEs ----
    NodeContainer satNodes;
    satNodes.Create(1);
    satNodes.Get(0)->AggregateObject(sat);
    NodeContainer ueNodes;
    ueNodes.Create(numUes);
    // TR 38.811 class UEs (real MobilityModel) around the ground station.
    NtnTr38811MobilityHelper ueMobility(1);
    auto mobProfile = NtnMobilityScenarios::MixedContinental();
    ueMobility.Install(ueNodes, mobProfile, gsLatDeg - 0.03, gsLatDeg + 0.03,
                       gsLonDeg - 0.03, gsLonDeg + 0.03);

    // ---- GSL contact tracking from the live geometry ----
    auto scheduler = CreateObject<ContactGraphScheduler>();
    scheduler->SetMinElevationDeg(minElevDeg);
    scheduler->RegisterGroundStation(0, gsLatDeg, gsLonDeg);
    scheduler->RegisterSatellite(100, sat);
    scheduler->Start();

    // ---- real mmwave NR cell + measured traffic ----
    NtnRealStackHelper rs;
    rs.SetSimTime(Seconds(simSeconds));
    rs.SetOutputDir(outputDir);
    rs.SetRunTag("ntn-constellation-sgp4-mobility-traffic");
    rs.SetSatEirpDbm(satEirpDbm);
    rs.Build(satNodes, ueNodes);
    rs.InstallTraffic(NtnRealStackHelper::TrafficProfile::MixedBouquet,
                      Seconds(1.0), Seconds(simSeconds - 0.5));
    rs.EnableAiFlowMonitor("ntn-constellation-sgp4-mobility-traffic"); // WS2 KPM series (TS 28.552 names)

    Simulator::Stop(Seconds(simSeconds));
    Simulator::Run();
    scheduler->Stop();
    rs.Collect();
    rs.WriteHealthReport();
    const uint64_t gslUp = scheduler->GslEventsUp();
    const uint64_t gslDown = scheduler->GslEventsDown();

    std::printf("# === ntn-constellation-sgp4-mobility-traffic summary ===\n"
                "#   TLE: %s   GS(lat,lon)=(%.4f, %.4f)  minElev=%.1f deg\n"
                "#   measured SINR=%.2f dB  measured throughput=%.3f Mbps  GSL up=%lu down=%lu\n",
                tlePath.c_str(), gsLatDeg, gsLonDeg, minElevDeg, rs.GetMeanDlSinrDb(),
                rs.GetRxThroughputMbps(), static_cast<unsigned long>(gslUp),
                static_cast<unsigned long>(gslDown));

    Simulator::Destroy();
    return 0;
}
