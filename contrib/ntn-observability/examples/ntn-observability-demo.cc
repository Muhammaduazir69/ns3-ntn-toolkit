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
#include "ns3/ntn-real-stack-helper.h"
#include "ns3/ntn-tr38811-mobility-model.h"
#include "ns3/sgp4-mobility-model.h"
#include "ns3/walker-constellation.h"
#include "ns3/network-module.h"
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

#include <cmath>
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
    NtnRealStackHelper* rs{nullptr};
    std::string runId;
    uint32_t satNodeId{1};
    uint32_t ueNodeId{2};
    uint32_t taSeriesIdx{0};
    uint32_t rsrpSeriesIdx{0};
    uint64_t sib19BroadcastCount{0}; //!< actual SIB19 broadcasts counted in OnSib19
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
    // MEASURED radio KPIs from the real NR cell (phy-trace provenance):
    // the dashboard now shows the genuine link, not a synthetic curve. The
    // radio point is only exported when a measured SINR sample exists — no
    // heuristic fallback values masquerade as measurements. RSRP is derived
    // from the cell's actual configuration: this is a single-cell,
    // noise-limited link, so SINR == SNR and the received signal power is
    //   RSRP [dBm] = SINR [dB] + noise floor [dBm]
    //   noise floor = -174 dBm/Hz + NF + 10 log10(BW)
    // with BW read from the helper's configured carrier bandwidth and NF the
    // UE PHY default (5 dB for both backends; the helper leaves it untouched).
    const double measSinr = w->rs ? w->rs->GetUeRecentSinrDb(0) : std::nan("");
    if (!std::isnan(measSinr))
    {
        constexpr double kUeNoiseFigureDb = 5.0; // UE PHY NoiseFigure default
        const double bwHz = w->rs->GetBandwidthHz();
        const double noiseFloorDbm = -174.0 + kUeNoiseFigureDb + 10.0 * std::log10(bwHz);
        const double measRsrp = measSinr + noiseFloorDbm;
        Point p;
        p.measurement = measurement::kRadio;
        p.tags[tag::kRunId] = w->runId;
        p.tags[tag::kCellId] = "C-1";
        p.tags[tag::kUeImsi] = "100001";
        // sinr_db is MEASURED (PHY trace); rsrp_dbm is DERIVED closed-form from
        // that SINR + noise floor (see above). Tag the derived provenance of the
        // RSRP field so a dashboard/query never mistakes it for a measured RSRP.
        p.tags["rsrp_provenance"] = "derived";
        p.fieldsFloat[field::kRsrpDbm] = measRsrp;
        p.fieldsFloat[field::kSinrDb] = measSinr;
        w->sink->Push(p);
        w->netSim->SampleSeries(w->rsrpSeriesIdx, now, measRsrp);
    }
    w->netSim->SampleSeries(w->taSeriesIdx, now,
                            w->ta->ComputeTotalTa().GetMicroSeconds());

    Simulator::Schedule(Seconds(1.0), &SampleEverySecond, w);
}

void
OnSib19(Wiring* w, const Sib19Content& sib)
{
    // Count ACTUAL SIB19 broadcasts so the ntn_sib19 series is a rising counter,
    // not a constant (it was previously filled with sib.cellId).
    ++w->sib19BroadcastCount;
    Point p;
    p.measurement = measurement::kSib;
    p.tags[tag::kRunId] = w->runId;
    p.tags[tag::kCellId] = std::to_string(sib.cellId);
    p.fieldsInt[field::kBroadcastSeq] = static_cast<long long>(w->sib19BroadcastCount);
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
    double simTimeSec = 20.0;
    std::string outputDir = ".";
    std::string runId = "demo-1";
    std::string radio = "nr"; // radio backend: "nr" (5G-LENA FR1) | "mmwave" (FR2)
    std::string influxFile = "/tmp/ntn-observability-demo.lp";
    std::string udpHost;
    uint16_t udpPort = 8089;
    std::string netSimPath = "/tmp/ntn-observability-demo.json";

    CommandLine cmd(__FILE__);
    cmd.AddValue("simTime", "Simulation duration (s)", simTimeSec);
    cmd.AddValue("runId", "Tag value attached to every point", runId);
    cmd.AddValue("radio", "Radio backend: nr (FR1) or mmwave", radio);
    cmd.AddValue("influxFile", "Influx line-protocol output file", influxFile);
    cmd.AddValue("udpHost",
                 "If set, push to InfluxDB UDP at host:port instead of file",
                 udpHost);
    cmd.AddValue("udpPort", "InfluxDB UDP listener port", udpPort);
    cmd.AddValue("netSim", "NetSimulyzer JSON output", netSimPath);
    cmd.AddValue("outputDir", "Output directory for sim_health.csv", outputDir);
    cmd.Parse(argc, argv);

    Wiring w;
    w.runId = runId;

    // ---- mobility: real SGP4 Walker sat + TR 38.811 UE at its sub-point ----
    ns3::ntncon::WalkerConfig wcfg;
    wcfg.num_planes = 1;
    wcfg.total_sats = 80;
    wcfg.altitude_km = 550.0;
    wcfg.inclination_deg = 53.0;
    wcfg.epoch_unix_s = 1735689600.0;
    const auto wElements = ns3::ntncon::WalkerConstellation::BuildDelta(wcfg);
    Ptr<ns3::ntncon::Sgp4MobilityModel> satMob =
        CreateObject<ns3::ntncon::Sgp4MobilityModel>();
    satMob->SetElements(wElements[0]);
    NodeContainer satNodes;
    satNodes.Create(1);
    satNodes.Get(0)->AggregateObject(satMob);
    NodeContainer ueNodes;
    ueNodes.Create(1);
    double subLat, subLon, subAlt;
    satMob->GetGeodetic(subLat, subLon, subAlt);
    NtnTr38811MobilityHelper ueMobility(1);
    auto mobProfile = NtnMobilityScenarios::MixedContinental();
    auto ueModels = ueMobility.Install(ueNodes, mobProfile, subLat - 0.02, subLat + 0.02,
                                       subLon - 0.02, subLon + 0.02);
    Ptr<MobilityModel> ueMob = ueModels[0];
    w.ueMob = ueMob;
    w.satMob = satMob;

    // ---- real NR cell: the MEASURED radio the dashboard observes ----
    NtnRealStackHelper rs;
    rs.SetRadioBackend(radio == "mmwave" ? NtnRealStackHelper::RadioBackend::Mmwave
                                         : NtnRealStackHelper::RadioBackend::Nr);
    if (radio != "mmwave")
    {
        rs.SetNumerology(1); // FR1 30 kHz SCS
    }
    rs.SetSimTime(Seconds(simTimeSec));
    rs.SetOutputDir(outputDir);
    rs.SetRunTag("ntn-observability-demo");
    // nr's Friis LEO link needs ~70 dBm for a healthy SINR; mmwave keeps 55 dBm.
    rs.SetSatEirpDbm(radio == "mmwave" ? 55.0 : 70.0);
    rs.Build(satNodes, ueNodes);
    // One UE -> a saturating eMBB stream so the dashboard observes a live
    // data plane (MixedBouquet would give the single UE the 1 kbps NB-IoT mix).
    rs.InstallTraffic(NtnRealStackHelper::TrafficProfile::EmbbStreaming,
                      Seconds(1.0), Seconds(simTimeSec - 0.5));
    w.rs = &rs;

    // ---- W2 components ----
    NtnRrcHelper rrcHelper;
    rrcHelper.SetPayloadMode(PayloadMode::Transparent);
    rrcHelper.SetReferencePosition(ntngeo::GeodeticToEcef(subLat, subLon, 0.0));

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
    rs.Collect();
    rs.WriteHealthReport();

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
