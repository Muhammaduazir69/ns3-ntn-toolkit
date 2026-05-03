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
#include "ns3/constant-velocity-mobility-model.h"
#include "ns3/core-module.h"
#include "ns3/ntn-drx.h"
#include "ns3/ntn-rrc-helper.h"
#include "ns3/ntn-sib19.h"
#include "ns3/ntn-timing-advance.h"
#include "ns3/ntn-ue-location-report.h"

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
             << std::setprecision(3) << ta->ComputeTaDriftRate(MilliSeconds(10)) << "\n";
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
              << sib.taCommonDriftRate << "\n";
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
    bool transparent = true;
    bool passAwareDrx = true;
    std::string prefix = "ntn-rrc-full";

    CommandLine cmd(__FILE__);
    cmd.AddValue("simTime", "Pass duration (s)", simTimeSec);
    cmd.AddValue("transparent", "Transparent payload (true) vs regenerative (false)", transparent);
    cmd.AddValue("passAwareDrx", "Enable NTN pass-aware DRX deep sleep", passAwareDrx);
    cmd.AddValue("prefix", "CSV file prefix", prefix);
    cmd.Parse(argc, argv);

    // ----- mobility -----
    Ptr<ConstantPositionMobilityModel> ueMob = CreateObject<ConstantPositionMobilityModel>();
    // UE in Islamabad expressed in ECEF.
    ueMob->SetPosition(Vector{1146054.7, 5567530.7, 3525200.6});

    Ptr<ConstantVelocityMobilityModel> satMob = CreateObject<ConstantVelocityMobilityModel>();
    satMob->SetPosition(Vector{-1.5e6, 5.5e6, 4.0e6});      // initial sat ECEF
    satMob->SetVelocity(Vector{7590.0, 0.0, 0.0});

    // ----- helper + 4 components -----
    NtnRrcHelper helper;
    helper.SetPayloadMode(transparent ? PayloadMode::Transparent : PayloadMode::RegenerativeFull);
    helper.SetReferencePosition(Vector{1146054.7, 5567530.7, 3525200.6});

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
    sinks.taOut << "time_s,ta_total_us,ta_common_us,ta_ue_us,ta_drift_rate\n";
    sinks.sibOut.open(prefix + "-sib19.csv");
    sinks.sibOut << "time_s,broadcast_seq,cell_id,sat_x,sat_y,sat_z,ta_common_us,drift_rate\n";
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

    Simulator::Stop(Seconds(simTimeSec));
    Simulator::Run();

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
