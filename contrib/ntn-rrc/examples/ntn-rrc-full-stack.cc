/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only
//
// End-to-end W2 example: a 600-second LEO pass exercising every
// `ntn-rrc` component at once — Timing Advance pre-comp, SIB19 broadcast,
// UE GNSS location reporting, and NTN-DRX state machine.
//
// Three CSVs are written:
//   <prefix>-ta.csv      — TA total / common / residual / drift
//   <prefix>-sib19.csv   — broadcast count + sat ECEF position per tick
//   <prefix>-ue.csv      — UE GNSS report (lat, lon, alt, sequence)
//   <prefix>-drx.csv     — DRX state at each second + cumulative awake time

#include "ns3/constant-position-mobility-model.h"
#include "ns3/core-module.h"
#include "ns3/satellite-sgp4-mobility-model.h"
#include "ns3/ntn-drx.h"
#include "ns3/ntn-rrc-helper.h"
#include "ns3/ntn-sib19.h"
#include "ns3/ntn-timing-advance.h"
#include "ns3/ntn-ue-location-report.h"
#include "ns3/ntn-realistic-traffic-helper.h"

#include <cstdio>
#include <fstream>
#include <iomanip>
#include <iostream>

using namespace ns3;
using namespace ns3::ntnrrc;

namespace
{

struct Sinks
{
    std::ofstream taOut;
    std::ofstream sibOut;
    std::ofstream ueOut;
    std::ofstream drxOut;
    uint32_t sib19Count{0};
    uint32_t reportCount{0};
};

void
SampleTa(Ptr<NtnTimingAdvance> ta, Sinks* s)
{
    if (!s->taOut.is_open())
    {
        return;
    }
    s->taOut << std::fixed << std::setprecision(3) << Simulator::Now().GetSeconds() << ","
             << ta->ComputeTotalTa().GetMicroSeconds() << ","
             << ta->ComputeCommonTa().GetMicroSeconds() << ","
             << ta->ComputeUeSpecificTa().GetMicroSeconds() << "," << std::scientific
             << std::setprecision(3) << (ta->ComputeTaDriftRate(MilliSeconds(10)) * 1e6) << "\n"; // s/s -> us/s
    Simulator::Schedule(Seconds(1.0), &SampleTa, ta, s);
}

void
SampleDrx(Ptr<NtnDrxStateMachine> drx, Sinks* s)
{
    if (!s->drxOut.is_open())
    {
        return;
    }
    s->drxOut << Simulator::Now().GetSeconds() << "," << static_cast<int>(drx->GetState()) << ","
              << drx->GetTimeInState(DrxState::Active).GetMilliSeconds() << ","
              << drx->GetTimeInState(DrxState::OnDuration).GetMilliSeconds() << ","
              << drx->GetTimeInState(DrxState::ShortSleep).GetMilliSeconds() << ","
              << drx->GetTimeInState(DrxState::LongSleep).GetMilliSeconds() << ","
              << drx->GetTimeInState(DrxState::AwaitingPass).GetMilliSeconds() << "\n";
    Simulator::Schedule(Seconds(1.0), &SampleDrx, drx, s);
}

void
OnSib19(Sinks* s, const Sib19Content& sib)
{
    s->sib19Count += 1;
    s->sibOut << Simulator::Now().GetSeconds() << "," << s->sib19Count << "," << sib.cellId << ","
              << sib.ephemeris.positionEcefM.x << "," << sib.ephemeris.positionEcefM.y << ","
              << sib.ephemeris.positionEcefM.z << "," << sib.taCommon.GetMicroSeconds() << ","
              << (sib.taCommonDriftRate * 1e6) << "\n"; // s/s -> us/s
}

void
OnUeReport(Sinks* s, const UeLocationReport& r)
{
    s->reportCount += 1;
    s->ueOut << r.timestamp.GetSeconds() << "," << r.reportSequence << "," << std::fixed
             << std::setprecision(7) << r.latDeg << "," << r.lonDeg << "," << std::setprecision(2)
             << r.altMetres << "\n";
}

} // namespace

int
main(int argc, char* argv[])
{
    double simTimeSec = 600.0;
    std::string outputDir = ".";
    bool transparent = true;
    bool passAwareDrx = true;
    std::string prefix = "ntn-rrc-full";

    CommandLine cmd(__FILE__);
    cmd.AddValue("simTime", "Pass duration (s)", simTimeSec);
    cmd.AddValue("transparent", "Transparent payload (true) vs regenerative (false)", transparent);
    cmd.AddValue("passAwareDrx", "Enable NTN pass-aware DRX deep sleep", passAwareDrx);
    cmd.AddValue("prefix", "CSV file prefix", prefix);
    cmd.AddValue("outputDir", "Output directory for sim_health.csv", outputDir);
    cmd.Parse(argc, argv);

    // ----- mobility -----
    Ptr<ConstantPositionMobilityModel> ueMob = CreateObject<ConstantPositionMobilityModel>();
    // UE in Islamabad (lat 33.6844, lon 73.0479, ~540 m AMSL) in ECEF — a real
    // ground terminal (the old coordinate sat 319 km up, not on the surface).
    ueMob->SetPosition(Vector{1545854.5, 5071422.4, 3533770.1});

    // Satellite on a real LEO orbit via SGP4 (bundled ISS TLE) rather than a
    // straight-line constant velocity, which would climb out of the orbital
    // shell over the pass and freeze the y/z ephemeris components.
    Ptr<SatSGP4MobilityModel> satMob = CreateObject<SatSGP4MobilityModel>();
    satMob->SetStartDate("2024-01-01 12:00:00");
    satMob->SetTleInfo(
        std::string("1 25544U 98067A   24001.50000000  .00006000  00000-0  11000-3 0  9991") +
        "\n" + "2 25544  51.6400  60.0000 0006000  90.0000 270.0000 15.49000000123456");

    // ----- helper + 4 components -----
    NtnRrcHelper helper;
    helper.SetPayloadMode(transparent ? PayloadMode::Transparent : PayloadMode::RegenerativeFull);
    // Reference (beam centre) offset ~50 km north of the UE so the UE-specific
    // TA residual (ta_ue = total - common) is non-zero, exercising the
    // common/UE-specific TA split instead of collapsing it to 0.
    helper.SetReferencePosition(Vector{1537714.6, 5044718.0, 3575300.8});

    Ptr<NtnTimingAdvance> ta = helper.InstallTimingAdvance(ueMob, satMob);
    Ptr<NtnSib19Broadcaster> sib19 =
        helper.InstallSib19Broadcaster(satMob, /*cellId=*/0xC0DE, ta, MilliSeconds(160));

    Ptr<NtnUeLocationReporter> rep =
        helper.InstallUeLocationReporter(ueMob,
                                         LocationReportMode::Periodic,
                                         Seconds(5.0),
                                         5.0);

    NtnDrxConfig drxCfg;
    drxCfg.longCycle = MilliSeconds(320);
    drxCfg.shortCycle = MilliSeconds(20);
    drxCfg.onDuration = MilliSeconds(5);
    drxCfg.inactivityTimer = MilliSeconds(50);
    drxCfg.passAware = passAwareDrx;
    drxCfg.passDuration = Seconds(simTimeSec);
    Ptr<NtnDrxStateMachine> drx = helper.InstallDrx(drxCfg);
    drx->NotifyNextPass(Seconds(0.0), Seconds(simTimeSec));

    // ----- sinks -----
    Sinks sinks;
    sinks.taOut.open(prefix + "-ta.csv");
    sinks.taOut << "time_s,ta_total_us,ta_common_us,ta_ue_us,ta_drift_rate_us_per_s\n";
    sinks.sibOut.open(prefix + "-sib19.csv");
    sinks.sibOut << "time_s,broadcast_seq,cell_id,sat_x,sat_y,sat_z,ta_common_us,drift_rate_us_per_s\n";
    sinks.ueOut.open(prefix + "-ue.csv");
    sinks.ueOut << "time_s,sequence,lat_deg,lon_deg,alt_m\n";
    sinks.drxOut.open(prefix + "-drx.csv");
    sinks.drxOut << "time_s,state,active_ms,onDuration_ms,shortSleep_ms,longSleep_ms,awaitingPass_ms\n";

    sib19->TraceConnectWithoutContext(
        "Broadcast", MakeCallback(&OnSib19).Bind(&sinks));
    rep->TraceConnectWithoutContext(
        "Report", MakeCallback(&OnUeReport).Bind(&sinks));

    sib19->Start();
    rep->Start();
    drx->Start();
    Simulator::ScheduleNow(&SampleTa, ta, &sinks);
    Simulator::ScheduleNow(&SampleDrx, drx, &sinks);
    // ==== v2 realistic traffic plane (auto-injected) =====================
    NtnRealisticTrafficHelper _ntn_traffic;
    _ntn_traffic.SetSimTime(Seconds(simTimeSec));
    _ntn_traffic.SetOutputDir(outputDir);
    _ntn_traffic.SetRunTag("ntn-rrc-full-stack");
    _ntn_traffic.SetProfile(NtnRealisticTrafficHelper::TrafficProfile::MixedBouquet);
    _ntn_traffic.InstallUes(8);
    _ntn_traffic.Wire();

    

    Simulator::Stop(Seconds(simTimeSec));
    Simulator::Run();
    _ntn_traffic.WriteHealthReport();

    sib19->Stop();
    rep->Stop();
    drx->Stop();

    Simulator::Destroy();

    std::cout << "ntn-rrc full-stack run complete.\n"
              << "  pass: " << simTimeSec << " s, "
              << (transparent ? "transparent" : "regenerative") << " payload\n"
              << "  sib19 broadcasts: " << sinks.sib19Count << " (period 160 ms)\n"
              << "  ue reports      : " << sinks.reportCount << " (period 5 s)\n"
              << "  drx total active : "
              << drx->GetTimeInState(DrxState::Active).GetMilliSeconds() << " ms\n"
              << "  drx total awaiting-pass: "
              << drx->GetTimeInState(DrxState::AwaitingPass).GetMilliSeconds() << " ms\n"
              << "  csvs: " << prefix << "-{ta,sib19,ue,drx}.csv\n";
    return 0;
}
