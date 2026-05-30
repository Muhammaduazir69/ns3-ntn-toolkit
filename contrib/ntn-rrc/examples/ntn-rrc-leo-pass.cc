/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only
//
// Demonstrates ephemeris-driven Timing Advance pre-compensation across a
// LEO pass. Logs total/common/UE-specific TA each second so the resulting
// CSV can be plotted to show the classic "smile" curve.

#include "ns3/constant-position-mobility-model.h"
#include "ns3/constant-velocity-mobility-model.h"
#include "ns3/core-module.h"
#include "ns3/ntn-rrc-helper.h"
#include "ns3/ntn-timing-advance.h"
#include "ns3/ntn-realistic-traffic-helper.h"

#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>

using namespace ns3;
using namespace ns3::ntnrrc;

namespace
{

void
SampleTa(Ptr<NtnTimingAdvance> ta, std::ostream* out)
{
    const Time total = ta->ComputeTotalTa();
    const Time common = ta->ComputeCommonTa();
    const Time residual = ta->ComputeUeSpecificTa();
    const double drift = ta->ComputeTaDriftRate(MilliSeconds(10));
    if (out)
    {
        *out << std::fixed << std::setprecision(6) << Simulator::Now().GetSeconds() << ","
             << total.GetMicroSeconds() << "," << common.GetMicroSeconds() << ","
             << residual.GetMicroSeconds() << "," << std::scientific << std::setprecision(3)
             << (drift * 1e6) << "\n"; // s/s -> us/s for the CSV column
    }
    Simulator::Schedule(Seconds(1.0), &SampleTa, ta, out);
}

} // namespace

int
main(int argc, char* argv[])
{
    double simTimeSec = 600.0;
    std::string outputDir = ".";
    bool transparent = true;
    std::string csvPath = "ntn-rrc-leo-pass.csv";

    CommandLine cmd(__FILE__);
    cmd.AddValue("simTime", "Simulation duration (s)", simTimeSec);
    cmd.AddValue("transparent", "Transparent (true) vs regenerative (false)", transparent);
    cmd.AddValue("csv", "Output CSV path", csvPath);
    cmd.AddValue("outputDir", "Output directory for sim_health.csv", outputDir);
    cmd.Parse(argc, argv);

    // 550-km LEO circular orbit moving along +x at 7.59 km/s, fly-over geometry.
    Ptr<ConstantPositionMobilityModel> ueMob = CreateObject<ConstantPositionMobilityModel>();
    ueMob->SetPosition(Vector{0.0, 0.0, 0.0});

    Ptr<ConstantVelocityMobilityModel> satMob = CreateObject<ConstantVelocityMobilityModel>();
    satMob->SetPosition(Vector{-2.0e6, 0.0, 550e3}); // start 2000 km west
    satMob->SetVelocity(Vector{7590.0, 0.0, 0.0});

    NtnRrcHelper helper;
    helper.SetPayloadMode(transparent ? PayloadMode::Transparent : PayloadMode::RegenerativeFull);
    helper.SetReferencePosition(Vector{0.0, 0.0, 0.0}); // beam centre = sub-UE point

    Ptr<NtnTimingAdvance> ta = helper.InstallTimingAdvance(ueMob, satMob);

    std::ofstream out(csvPath);
    out << "time_s,ta_total_us,ta_common_us,ta_ue_us,ta_drift_rate_us_per_s\n";
    Simulator::ScheduleNow(&SampleTa, ta, &out);
    // ==== v2 realistic traffic plane (auto-injected) =====================
    NtnRealisticTrafficHelper _ntn_traffic;
    _ntn_traffic.SetSimTime(Seconds(simTimeSec));
    _ntn_traffic.SetOutputDir(outputDir);
    _ntn_traffic.SetRunTag("ntn-rrc-leo-pass");
    _ntn_traffic.SetProfile(NtnRealisticTrafficHelper::TrafficProfile::MixedBouquet);
    _ntn_traffic.InstallUes(8);
    _ntn_traffic.Wire();

    

    Simulator::Stop(Seconds(simTimeSec));
    Simulator::Run();
    _ntn_traffic.WriteHealthReport();
    Simulator::Destroy();

    std::cout << "Wrote " << csvPath << " (" << simTimeSec << " s pass, "
              << (transparent ? "transparent" : "regenerative") << " payload)\n";
    return 0;
}
