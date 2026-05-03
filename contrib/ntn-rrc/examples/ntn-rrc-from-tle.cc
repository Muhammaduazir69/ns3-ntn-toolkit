/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only
//
// W1 + W2 integration: read a real TLE (the kind ntn-constellation pulls
// from CelesTrak), drive a SNS3 SatelliteSGP4MobilityModel from it, and
// log NtnTimingAdvance values across a pass.
//
// Output CSV columns:
//   time_s, sat_x_m, sat_y_m, sat_z_m, slant_km, ta_total_us, ta_drift_rate

#include "ns3/constant-position-mobility-model.h"
#include "ns3/core-module.h"
#include "ns3/ntn-rrc-helper.h"
#include "ns3/ntn-timing-advance.h"
#include "ns3/satellite-sgp4-mobility-model.h"

#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

using namespace ns3;
using namespace ns3::ntnrrc;

namespace
{

struct TleFile
{
    std::string name;
    std::string line1;
    std::string line2;
};

bool
ReadTle(const std::string& path, TleFile& out)
{
    std::ifstream f(path);
    if (!f)
    {
        return false;
    }
    std::vector<std::string> lines;
    std::string line;
    while (std::getline(f, line))
    {
        if (!line.empty())
        {
            lines.push_back(line);
        }
    }
    if (lines.size() < 3)
    {
        return false;
    }
    out.name = lines[0];
    out.line1 = lines[1];
    out.line2 = lines[2];
    return true;
}

void
SampleStep(Ptr<NtnTimingAdvance> ta,
           Ptr<SatSGP4MobilityModel> sat,
           std::ostream* csv,
           Time step,
           Time stopAt)
{
    if (Simulator::Now() > stopAt)
    {
        return;
    }
    const Vector p = sat->GetPosition();
    const double slant = ta->GetSlantRangeMetres() / 1000.0;
    const double taTotalUs = ta->ComputeTotalTa().GetMicroSeconds();
    const double drift = ta->ComputeTaDriftRate(MilliSeconds(10));
    *csv << std::fixed << std::setprecision(3) << Simulator::Now().GetSeconds() << ","
         << std::setprecision(2) << p.x << "," << p.y << "," << p.z << "," << std::setprecision(3)
         << slant << "," << static_cast<long long>(taTotalUs) << "," << std::scientific
         << std::setprecision(3) << drift << "\n";
    Simulator::Schedule(step, &SampleStep, ta, sat, csv, step, stopAt);
}

} // namespace

int
main(int argc, char* argv[])
{
    std::string tlePath;
    std::string startUtc;     // "YYYY-MM-DD HH:MM:SS"
    double ueLatDeg = 33.6844;
    double ueLonDeg = 73.0479;
    double ueAltM = 540.0;
    double simTimeSec = 600.0;
    double stepSec = 1.0;
    bool transparent = true;
    std::string csvPath = "ntn-rrc-from-tle.csv";

    CommandLine cmd(__FILE__);
    cmd.AddValue("tle", "Path to a 3-line TLE file (name, line1, line2)", tlePath);
    cmd.AddValue("start", "Scenario start UTC, ISO format YYYY-MM-DD HH:MM:SS", startUtc);
    cmd.AddValue("ueLat", "UE latitude (deg)", ueLatDeg);
    cmd.AddValue("ueLon", "UE longitude (deg)", ueLonDeg);
    cmd.AddValue("ueAlt", "UE altitude (m)", ueAltM);
    cmd.AddValue("simTime", "Simulation duration (s)", simTimeSec);
    cmd.AddValue("step", "Sample period (s)", stepSec);
    cmd.AddValue("transparent", "Transparent (true) vs regenerative (false)", transparent);
    cmd.AddValue("csv", "Output CSV path", csvPath);
    cmd.Parse(argc, argv);

    TleFile tle;
    if (tlePath.empty() || !ReadTle(tlePath, tle))
    {
        std::cerr << "error: --tle is required and must be a 3-line file\n";
        return 2;
    }
    if (startUtc.empty())
    {
        std::cerr << "error: --start is required (UTC timestamp)\n";
        return 2;
    }

    // ---- UE in ECEF ----
    constexpr double kA = 6378137.0;
    constexpr double kF = 1.0 / 298.257223563;
    constexpr double kE2 = kF * (2.0 - kF);
    const double latR = ueLatDeg * M_PI / 180.0;
    const double lonR = ueLonDeg * M_PI / 180.0;
    const double sLat = std::sin(latR);
    const double cLat = std::cos(latR);
    const double N = kA / std::sqrt(1.0 - kE2 * sLat * sLat);
    const Vector ueEcef{(N + ueAltM) * cLat * std::cos(lonR),
                        (N + ueAltM) * cLat * std::sin(lonR),
                        (N * (1.0 - kE2) + ueAltM) * sLat};

    Ptr<ConstantPositionMobilityModel> ueMob = CreateObject<ConstantPositionMobilityModel>();
    ueMob->SetPosition(ueEcef);

    // CommandLine truncates arg values at whitespace, so `--start` accepts
    // either "YYYY-MM-DDTHH:MM:SS" (ISO with T separator) or just the date.
    // We translate the T separator back to a space for SNS3's parser.
    std::string startSpace = startUtc;
    for (auto& ch : startSpace)
    {
        if (ch == 'T')
            ch = ' ';
    }

    // ---- Satellite via SGP4 (SNS3) ----
    Ptr<SatSGP4MobilityModel> satMob = CreateObject<SatSGP4MobilityModel>();
    satMob->SetStartDate(startSpace);
    satMob->SetTleInfo(tle.line1 + "\n" + tle.line2);

    // ---- TA pre-comp ----
    NtnRrcHelper helper;
    helper.SetPayloadMode(transparent ? PayloadMode::Transparent : PayloadMode::RegenerativeFull);
    helper.SetReferencePosition(ueEcef);
    Ptr<NtnTimingAdvance> ta = helper.InstallTimingAdvance(ueMob, satMob);

    std::ofstream csv(csvPath);
    csv << "time_s,sat_x_m,sat_y_m,sat_z_m,slant_km,ta_total_us,ta_drift_rate\n";
    Simulator::ScheduleNow(&SampleStep, ta, satMob, &csv, Seconds(stepSec),
                           Seconds(simTimeSec));

    Simulator::Stop(Seconds(simTimeSec) + MilliSeconds(1));
    Simulator::Run();
    Simulator::Destroy();

    std::cout << "wrote " << csvPath << " (" << simTimeSec << " s pass of " << tle.name
              << ", step " << stepSec << " s)\n";
    return 0;
}
