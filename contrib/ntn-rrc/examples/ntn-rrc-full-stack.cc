/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only
//
// ntn-rrc-full-stack — end-to-end W2 example exercising every `ntn-rrc`
// component at once over a REAL mmwave NR NTN cell (NtnRealStackHelper:
// SpectrumPhy + MAC + HARQ + RLC/PDCP + RRC + EPC): Timing Advance pre-comp,
// SIB19 broadcast, UE GNSS location reporting, and NTN-DRX — all bound to the
// live LEO-pass geometry while real UDP traffic flows over the radio. The radio
// KPIs (SINR/TBLER/throughput) are MEASURED off the mmwave PHY trace, not
// closed-form. Writes the four RRC CSVs plus an honest sim_health.csv.
//
//   <prefix>-ta.csv      — TA total / common / residual / drift
//   <prefix>-sib19.csv   — broadcast count + sat ECEF position per tick
//   <prefix>-ue.csv      — UE GNSS report (lat, lon, alt, sequence)
//   <prefix>-drx.csv     — DRX state at each second + cumulative awake time

#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/ntn-drx.h"
#include "ns3/ntn-real-stack-helper.h"
#include "ns3/ntn-tr38811-mobility-model.h"
#include "ns3/sgp4-mobility-model.h"
#include "ns3/walker-constellation.h"

#include "ns3/ntn-rrc-helper.h"
#include "ns3/ntn-sib19.h"
#include "ns3/ntn-timing-advance.h"
#include "ns3/ntn-ue-location-report.h"

#include <cstdio>
#include <filesystem>
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

NtnRealStackHelper* g_rs = nullptr;

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
             << ta->ComputeUeSpecificTa().GetMicroSeconds() << ","
             << (g_rs ? g_rs->GetUeRecentSinrDb(0) : 0.0) << "\n";
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
              << (sib.taCommonDriftRate * 1e6) << "\n";
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
    double simTimeSec = 20.0;
    uint32_t numUes = 4;
    double altitudeKm = 550.0;
    double satEirpDbm = -1.0; // sentinel: backend-appropriate default chosen below
    std::string radio = "nr"; // radio spine: "nr" (5G-LENA FR1) | "mmwave" (FR2)
    bool transparent = true;
    bool passAwareDrx = true;
    std::string prefix = "ntn-rrc-full";
    std::string outputDir = "ntn-rrc-full-stack-output";

    CommandLine cmd(__FILE__);
    cmd.AddValue("simTime", "Pass duration (s)", simTimeSec);
    cmd.AddValue("numUes", "Number of UEs on the serving cell", numUes);
    cmd.AddValue("altitude", "Satellite altitude (km)", altitudeKm);
    cmd.AddValue("satEirpDbm", "Satellite EIRP / gNB Tx power (dBm); -1 = backend default", satEirpDbm);
    cmd.AddValue("radio", "Radio backend: nr (FR1) or mmwave", radio);
    cmd.AddValue("transparent", "Transparent payload (true) vs regenerative (false)", transparent);
    cmd.AddValue("passAwareDrx", "Enable NTN pass-aware DRX deep sleep", passAwareDrx);
    cmd.AddValue("prefix", "CSV file prefix", prefix);
    cmd.AddValue("outputDir", "Output directory", outputDir);
    cmd.Parse(argc, argv);

    // Backend-appropriate EIRP default: nr's Friis LEO link needs ~+15 dB vs
    // mmwave, so honour the historical 55 dBm for mmwave but give nr 70 dBm.
    const bool useNr = (radio == "nr");
    if (satEirpDbm < 0.0)
    {
        satEirpDbm = useNr ? 70.0 : 55.0;
    }

    // ----- nodes + LEO-pass mobility (real geometry, guaranteed in view) -----
    NodeContainer satNodes;
    satNodes.Create(1);
    NodeContainer ueNodes;
    ueNodes.Create(numUes);

    // Real SGP4 Walker orbit for the serving satellite (genuine LEO pass).
    ns3::ntncon::WalkerConfig wcfg;
    wcfg.num_planes = 1;
    wcfg.total_sats = 80;
    wcfg.altitude_km = altitudeKm;
    wcfg.inclination_deg = 53.0;
    wcfg.epoch_unix_s = 1735689600.0;
    const auto wElements = ns3::ntncon::WalkerConstellation::BuildDelta(wcfg);
    Ptr<ns3::ntncon::Sgp4MobilityModel> satSgp4 =
        CreateObject<ns3::ntncon::Sgp4MobilityModel>();
    satSgp4->SetElements(wElements[0]);
    satNodes.Get(0)->AggregateObject(satSgp4);
    Ptr<MobilityModel> satMob = satSgp4;
    double subLat, subLon, subAlt;
    satSgp4->GetGeodetic(subLat, subLon, subAlt);

    // TR 38.811 class UEs (real MobilityModel) under the t=0 sub-point.
    NtnTr38811MobilityHelper ueMobility(1);
    auto mobProfile = NtnMobilityScenarios::MixedContinental();
    auto ueModels = ueMobility.Install(ueNodes, mobProfile, subLat - 0.03, subLat + 0.03,
                                       subLon - 0.03, subLon + 0.03);
    Ptr<MobilityModel> ueMob = ueModels[0];

    // ----- real mmwave NR cell + traffic -----
    NtnRealStackHelper rs;
    rs.SetRadioBackend(radio == "mmwave" ? NtnRealStackHelper::RadioBackend::Mmwave
                                         : NtnRealStackHelper::RadioBackend::Nr);
    if (radio != "mmwave")
    {
        rs.SetNumerology(1); // FR1 30 kHz SCS (nr backend only)
    }
    rs.SetSimTime(Seconds(simTimeSec));
    rs.SetOutputDir(outputDir);
    rs.SetRunTag("ntn-rrc-full-stack");
    rs.SetSatEirpDbm(satEirpDbm);
    rs.Build(satNodes, ueNodes);
    rs.InstallTraffic(NtnRealStackHelper::TrafficProfile::MixedBouquet,
                      Seconds(1.0), Seconds(simTimeSec - 0.5));
    rs.EnableAiFlowMonitor("ntn-rrc-full-stack"); // WS2 KPM series (TS 28.552 names)
    g_rs = &rs;
    const uint16_t cellId = rs.GetServingCellId(); // radio-agnostic (mmwave or nr gNB)

    // ----- the four RRC NTN components, bound to the same geometry -----
    NtnRrcHelper helper;
    helper.SetPayloadMode(transparent ? PayloadMode::Transparent : PayloadMode::RegenerativeFull);
    // Beam centre ~50 km north of the sub-point (ECEF) -> non-zero residual TA.
    helper.SetReferencePosition(ntngeo::GeodeticToEcef(subLat + 0.45, subLon, 0.0));
    Ptr<NtnTimingAdvance> ta = helper.InstallTimingAdvance(ueMob, satMob);
    Ptr<NtnSib19Broadcaster> sib19 =
        helper.InstallSib19Broadcaster(satMob, cellId, ta, MilliSeconds(160));
    Ptr<NtnUeLocationReporter> rep =
        helper.InstallUeLocationReporter(ueMob, LocationReportMode::Periodic, Seconds(5.0), 5.0);

    NtnDrxConfig drxCfg;
    drxCfg.longCycle = MilliSeconds(320);
    drxCfg.shortCycle = MilliSeconds(20);
    drxCfg.onDuration = MilliSeconds(5);
    drxCfg.inactivityTimer = MilliSeconds(50);
    drxCfg.passAware = passAwareDrx;
    drxCfg.passDuration = Seconds(simTimeSec);
    Ptr<NtnDrxStateMachine> drx = helper.InstallDrx(drxCfg);
    drx->NotifyNextPass(Seconds(0.0), Seconds(simTimeSec));

    // ----- CSV sinks -----
    std::filesystem::create_directories(outputDir);
    Sinks sinks;
    sinks.taOut.open(outputDir + "/" + prefix + "-ta.csv");
    sinks.taOut << "time_s,ta_total_us,ta_common_us,ta_ue_us,measured_sinr_db\n";
    sinks.sibOut.open(outputDir + "/" + prefix + "-sib19.csv");
    sinks.sibOut << "time_s,broadcast_seq,cell_id,sat_x,sat_y,sat_z,ta_common_us,drift_rate_us_per_s\n";
    sinks.ueOut.open(outputDir + "/" + prefix + "-ue.csv");
    sinks.ueOut << "time_s,sequence,lat_deg,lon_deg,alt_m\n";
    sinks.drxOut.open(outputDir + "/" + prefix + "-drx.csv");
    sinks.drxOut << "time_s,state,active_ms,onDuration_ms,shortSleep_ms,longSleep_ms,awaitingPass_ms\n";

    sib19->TraceConnectWithoutContext("Broadcast", MakeCallback(&OnSib19).Bind(&sinks));
    rep->TraceConnectWithoutContext("Report", MakeCallback(&OnUeReport).Bind(&sinks));

    sib19->Start();
    rep->Start();
    drx->Start();
    Simulator::ScheduleNow(&SampleTa, ta, &sinks);
    Simulator::ScheduleNow(&SampleDrx, drx, &sinks);

    Simulator::Stop(Seconds(simTimeSec));
    Simulator::Run();
    rs.Collect();
    rs.WriteHealthReport();

    sib19->Stop();
    rep->Stop();
    drx->Stop();

    std::cout << "ntn-rrc full-stack run complete (real " << (useNr ? "5G-LENA nr" : "mmwave")
              << " NR cell).\n"
              << "  pass: " << simTimeSec << " s, "
              << (transparent ? "transparent" : "regenerative") << " payload\n"
              << "  measured mean SINR : " << rs.GetMeanDlSinrDb() << " dB\n"
              << "  measured throughput: " << rs.GetRxThroughputMbps() << " Mbps\n"
              << "  sib19 broadcasts   : " << sinks.sib19Count << " (period 160 ms)\n"
              << "  ue reports         : " << sinks.reportCount << " (period 5 s)\n"
              << "  drx total active   : " << drx->GetTimeInState(DrxState::Active).GetMilliSeconds()
              << " ms\n"
              << "  csvs: " << outputDir << "/" << prefix << "-{ta,sib19,ue,drx}.csv\n";

    Simulator::Destroy();
    return 0;
}
