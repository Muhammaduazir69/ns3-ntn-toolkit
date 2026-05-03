/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only
//
// W2 + W3 integration: runs the full ntn-rrc stack while pushing every KPI
// into an InfluxDB sink (file or UDP) and a NetSimulyzer JSON trace.
//
// Usage:
//   ns3.43-ntn-observability-demo-default --simTime=120 --influxFile=/tmp/run.lp --netSim=/tmp/run.json
//   ns3.43-ntn-observability-demo-default --simTime=120 --udpHost=127.0.0.1

#include "ns3/constant-position-mobility-model.h"
#include "ns3/constant-velocity-mobility-model.h"
#include "ns3/core-module.h"
#include "ns3/ntn-drx.h"
#include "ns3/ntn-influx-sink.h"
#include "ns3/ntn-metric-schema.h"
#include "ns3/ntn-netsimulyzer-exporter.h"
#include "ns3/ntn-observability-helper.h"
#include "ns3/ntn-rrc-helper.h"
#include "ns3/ntn-sib19.h"
#include "ns3/ntn-timing-advance.h"
#include "ns3/ntn-ue-location-report.h"

#include <cstdio>
#include <iostream>

using namespace ns3;
using namespace ns3::ntnobs;
using namespace ns3::ntnrrc;

namespace
{

struct Wiring
{
    Ptr<NtnInfluxSink> sink;
    Ptr<NtnNetSimulyzerExporter> netSim;
    Ptr<NtnTimingAdvance> ta;
    Ptr<NtnSib19Broadcaster> sib19;
    Ptr<NtnUeLocationReporter> ueRep;
    Ptr<NtnDrxStateMachine> drx;
    Ptr<MobilityModel> ueMob;
    Ptr<MobilityModel> satMob;
    std::string runId;
    uint32_t satNodeId{1};
    uint32_t ueNodeId{2};
    uint32_t taSeriesIdx{0};
    uint32_t rsrpSeriesIdx{0};
};

void
SampleEverySecond(Wiring* w)
{
    const double now = Simulator::Now().GetSeconds();

    {
        Point p;
        p.measurement = measurement::kTa;
        p.tags[tag::kRunId] = w->runId;
        p.tags[tag::kPayloadMode] = "transparent";
        p.fieldsFloat[field::kTaTotalUs] = w->ta->ComputeTotalTa().GetMicroSeconds();
        p.fieldsFloat[field::kTaCommonUs] = w->ta->ComputeCommonTa().GetMicroSeconds();
        p.fieldsFloat[field::kTaDriftUsPerS] =
            w->ta->ComputeTaDriftRate(MilliSeconds(10)) * 1.0e6;
        w->sink->Push(p);
    }
    {
        const Vector pos = w->satMob->GetPosition();
        const Vector vel = w->satMob->GetVelocity();
        Point p;
        p.measurement = measurement::kSatPosition;
        p.tags[tag::kRunId] = w->runId;
        p.fieldsFloat[field::kSatXM] = pos.x;
        p.fieldsFloat[field::kSatYM] = pos.y;
        p.fieldsFloat[field::kSatZM] = pos.z;
        p.fieldsFloat[field::kSatVxMps] = vel.x;
        p.fieldsFloat[field::kSatVyMps] = vel.y;
        p.fieldsFloat[field::kSatVzMps] = vel.z;
        w->sink->Push(p);
        // Also push to NetSimulyzer
        w->netSim->NodeMove(w->satNodeId, now, pos);
    }
    {
        Point p;
        p.measurement = measurement::kDrx;
        p.tags[tag::kRunId] = w->runId;
        p.fieldsInt[field::kDrxState] = static_cast<long long>(w->drx->GetState());
        p.fieldsFloat[field::kDrxAwakeMs] =
            (w->drx->GetTimeInState(DrxState::Active) +
             w->drx->GetTimeInState(DrxState::OnDuration)).GetMilliSeconds();
        w->sink->Push(p);
    }
    // Synthetic RSRP signal that breathes with slant range so the dashboard
    // demo has something interesting to show. Not based on a real link budget.
    const double slantKm = w->ta->GetSlantRangeMetres() / 1000.0;
    const double syntheticRsrp = -90.0 - 30.0 * (slantKm / 2000.0);
    {
        Point p;
        p.measurement = measurement::kRadio;
        p.tags[tag::kRunId] = w->runId;
        p.tags[tag::kCellId] = "C-1";
        p.tags[tag::kUeImsi] = "100001";
        p.fieldsFloat[field::kRsrpDbm] = syntheticRsrp;
        p.fieldsFloat[field::kSinrDb] = 20.0 - 0.005 * slantKm;
        w->sink->Push(p);
    }
    w->netSim->SampleSeries(w->taSeriesIdx, now,
                            w->ta->ComputeTotalTa().GetMicroSeconds());
    w->netSim->SampleSeries(w->rsrpSeriesIdx, now, syntheticRsrp);

    Simulator::Schedule(Seconds(1.0), &SampleEverySecond, w);
}

void
OnSib19(Wiring* w, const Sib19Content& sib)
{
    Point p;
    p.measurement = measurement::kSib;
    p.tags[tag::kRunId] = w->runId;
    p.tags[tag::kCellId] = std::to_string(sib.cellId);
    p.fieldsInt[field::kBroadcastSeq] = static_cast<long long>(sib.cellId);
    p.fieldsFloat[field::kTaCommonUs] = sib.taCommon.GetMicroSeconds();
    w->sink->Push(p);
}

void
OnUeReport(Wiring* w, const UeLocationReport& r)
{
    Point p;
    p.measurement = measurement::kUeReport;
    p.tags[tag::kRunId] = w->runId;
    p.tags[tag::kUeImsi] = "100001";
    p.fieldsFloat[field::kLatDeg] = r.latDeg;
    p.fieldsFloat[field::kLonDeg] = r.lonDeg;
    p.fieldsFloat[field::kAltM] = r.altMetres;
    p.fieldsInt[field::kReportSeq] = r.reportSequence;
    w->sink->Push(p);

    char msg[128];
    std::snprintf(msg,
                  sizeof(msg),
                  "GNSS report #%u  lat=%.4f lon=%.4f",
                  r.reportSequence,
                  r.latDeg,
                  r.lonDeg);
    w->netSim->LogMessage(w->ueNodeId, r.timestamp.GetSeconds(), msg);
}

} // namespace

int
main(int argc, char* argv[])
{
    double simTimeSec = 600.0;
    std::string runId = "demo-1";
    std::string influxFile = "/tmp/ntn-observability-demo.lp";
    std::string udpHost;
    uint16_t udpPort = 8089;
    std::string netSimPath = "/tmp/ntn-observability-demo.json";

    CommandLine cmd(__FILE__);
    cmd.AddValue("simTime", "Simulation duration (s)", simTimeSec);
    cmd.AddValue("runId", "Tag value attached to every point", runId);
    cmd.AddValue("influxFile", "Influx line-protocol output file", influxFile);
    cmd.AddValue("udpHost",
                 "If set, push to InfluxDB UDP at host:port instead of file",
                 udpHost);
    cmd.AddValue("udpPort", "InfluxDB UDP listener port", udpPort);
    cmd.AddValue("netSim", "NetSimulyzer JSON output", netSimPath);
    cmd.Parse(argc, argv);

    Wiring w;
    w.runId = runId;

    // ---- mobility ----
    Ptr<ConstantPositionMobilityModel> ueMob = CreateObject<ConstantPositionMobilityModel>();
    ueMob->SetPosition(Vector{1146054.7, 5567530.7, 3525200.6});
    Ptr<ConstantVelocityMobilityModel> satMob = CreateObject<ConstantVelocityMobilityModel>();
    satMob->SetPosition(Vector{-1.5e6, 5.5e6, 4.0e6});
    satMob->SetVelocity(Vector{7590.0, 0.0, 0.0});
    w.ueMob = ueMob;
    w.satMob = satMob;

    // ---- W2 components ----
    NtnRrcHelper rrcHelper;
    rrcHelper.SetPayloadMode(PayloadMode::Transparent);
    rrcHelper.SetReferencePosition(Vector{1146054.7, 5567530.7, 3525200.6});

    w.ta = rrcHelper.InstallTimingAdvance(ueMob, satMob);
    w.sib19 = rrcHelper.InstallSib19Broadcaster(satMob, /*cellId=*/100, w.ta, MilliSeconds(160));
    w.ueRep = rrcHelper.InstallUeLocationReporter(ueMob,
                                                  LocationReportMode::Periodic,
                                                  Seconds(5.0),
                                                  5.0);
    NtnDrxConfig drxCfg;
    drxCfg.longCycle = MilliSeconds(320);
    drxCfg.shortCycle = MilliSeconds(20);
    drxCfg.onDuration = MilliSeconds(5);
    drxCfg.inactivityTimer = MilliSeconds(50);
    w.drx = rrcHelper.InstallDrx(drxCfg);

    // ---- W3 sinks ----
    NtnObservabilityHelper obsHelper;
    obsHelper.SetRunId(runId);
    if (!udpHost.empty())
    {
        obsHelper.SetInfluxUdp(udpHost, udpPort);
    }
    else
    {
        obsHelper.SetInfluxFile(influxFile);
    }
    obsHelper.SetNetSimulyzerOutput(netSimPath);
    obsHelper.SetFlushPeriod(Seconds(1.0));

    w.sink = obsHelper.InstallInfluxSink();
    w.netSim = obsHelper.InstallNetSimulyzerExporter();

    w.netSim->AddNode(w.satNodeId, "satellite", satMob->GetPosition(), 1.0);
    w.netSim->AddNode(w.ueNodeId, "ue", ueMob->GetPosition(), 0.5);
    w.taSeriesIdx = w.netSim->AddSeries("TA total", "time (s)", "TA total", "us");
    w.rsrpSeriesIdx = w.netSim->AddSeries("RSRP", "time (s)", "RSRP", "dBm");

    // ---- wire traces ----
    w.sib19->TraceConnectWithoutContext("Broadcast",
                                        MakeCallback(&OnSib19).Bind(&w));
    w.ueRep->TraceConnectWithoutContext("Report",
                                        MakeCallback(&OnUeReport).Bind(&w));

    // ---- start everything ----
    w.sib19->Start();
    w.ueRep->Start();
    w.drx->Start();
    w.sink->Start();
    w.netSim->Start();

    Simulator::ScheduleNow(&SampleEverySecond, &w);
    Simulator::Stop(Seconds(simTimeSec));
    Simulator::Run();

    w.sib19->Stop();
    w.ueRep->Stop();
    w.drx->Stop();
    w.sink->Stop();
    w.netSim->Stop();

    Simulator::Destroy();

    std::cout << "ntn-observability demo complete.\n"
              << "  influx points emitted: " << w.sink->GetEmittedCount() << "\n"
              << "  netsimulyzer events  : " << w.netSim->GetEventCount() << "\n";
    if (udpHost.empty())
    {
        std::cout << "  influx file          : " << influxFile << "\n";
    }
    else
    {
        std::cout << "  influx udp           : " << udpHost << ":" << udpPort << "\n";
    }
    std::cout << "  netsimulyzer file    : " << netSimPath << "\n";
    return 0;
}
