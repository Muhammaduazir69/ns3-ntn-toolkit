/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only
//
// ntn-rrc-full-stack — end-to-end W2 example exercising every `ntn-rrc`
// component at once over a REAL mmwave NR NTN cell (NtnRealStackHelper:
// SpectrumPhy + MAC + HARQ + RLC/PDCP + RRC + EPC): Timing Advance pre-comp,
// SIB19 broadcast, UE GNSS location reporting, and NTN-DRX — all bound to the
// live LEO-pass geometry while real UDP traffic flows over the radio (DRX
// activity is fed from UE 0's delivered packets, TS 38.321 section 5.7). The radio
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
uint64_t g_drxLastRxPkts = 0;
double g_drxStopS = 0.0;

// RRC-3c: feed the DRX state machine from the real downlink.
//
// The state machine was started and never told about a single packet, so it
// left Active after its 50 ms inactivity timer and slept for the rest of a run
// in which UDP traffic flowed to UE 0 the whole time: "drx total active 50 ms"
// described no terminal. TS 38.321 section 5.7 restarts drx-InactivityTimer on
// every PDCCH that schedules new data, so the activity is taken from the UE's
// own delivered-packet counter. The poll runs every 1 ms (one 15 kHz slot);
// against the 50 ms inactivity timer the sampling error is at most 2 percent,
// unlike the one-poll-per-long-cycle scheme SCOPE A18 records for
// ntn-rrc-drx-data-traffic.
void
DrxActivityPoll(Ptr<NtnDrxStateMachine> drx)
{
    if (Simulator::Now().GetSeconds() >= g_drxStopS || !g_rs)
    {
        return;
    }
    const uint64_t pkts = g_rs->GetUeRxPackets(0);
    if (pkts > g_drxLastRxPkts)
    {
        drx->NotifyDataActivity();
    }
    g_drxLastRxPkts = pkts;
    Simulator::Schedule(MilliSeconds(1), &DrxActivityPoll, drx);
}

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
    cmd.AddValue("satEirpDbm", "-1 = TR 38.821 Set-1 S-band EIRP density (34 dBW/MHz); otherwise conducted power at the array input (dBm)", satEirpDbm);
    cmd.AddValue("radio", "Radio backend: nr (FR1) or mmwave", radio);
    cmd.AddValue("transparent", "Transparent payload (true) vs regenerative (false)", transparent);
    cmd.AddValue("passAwareDrx", "Enable NTN pass-aware DRX deep sleep", passAwareDrx);
    cmd.AddValue("prefix", "CSV file prefix", prefix);
    cmd.AddValue("outputDir", "Output directory", outputDir);
    cmd.Parse(argc, argv);

    const bool useNr = (radio == "nr");
    // EIRP default (NT-02 calibration). The nr default used to be 70 dBm of
    // CONDUCTED power, about 88 dBm EIRP once the ~18 dB array gain is added,
    // ~11 dB above the TR 38.821 Table 6.1.1.1-1 Set-1 S-band payload
    // (34 dBW/MHz, 77 dBm over 20 MHz). On an S-band FR1 carrier the nr
    // default is now that Set-1 density, as ntn-v2x-real-stack does; an
    // explicit --satEirpDbm keeps meaning conducted power, and mmwave keeps
    // its historical 55.0 dBm.
    // The probe mirrors the cell's carrier and bandwidth and returns the
    // equivalent CONDUCTED power from the helper's own formula
    // (density + 10log10(BW_MHz) + 30 - ArrayGainDb()), so every other
    // consumer of satEirpDbm stays consistent with the radio.
    NtnRealStackHelper eirpProbe;
    const bool eirpFromTr38821 = satEirpDbm < 0.0 && useNr &&
                                 eirpProbe.GetCarrierFrequencyHz() > 1.0e9 &&
                                 eirpProbe.GetCarrierFrequencyHz() <= 7.125e9;
    if (eirpFromTr38821)
    {
        eirpProbe.SetSatEirpDensityDbwMhz(NtnRealStackHelper::kTr38821Set1SBandEirpDensityDbwMhz);
        satEirpDbm = eirpProbe.GetSatEirpDbm();
    }
    else if (satEirpDbm < 0.0)
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
    if (eirpFromTr38821)
    {
        // NT-02: TR 38.821 Table 6.1.1.1-1 Set-1 S-band EIRP density. The
        // helper back-computes the conducted power from the bandwidth and
        // the array gain, and again in Build().
        rs.SetSatEirpDensityDbwMhz(NtnRealStackHelper::kTr38821Set1SBandEirpDensityDbwMhz);
    }
    else
    {
        // NT-02: an explicit value (or a default on a carrier outside the
        // Set-1 S-band reference) is CONDUCTED power at the array input.
        rs.SetSatConductedPowerDbm(satEirpDbm);
    }
    rs.Build(satNodes, ueNodes);
    // satEirpDbm was resolved before Build() and handed to the rest of the
    // example; make sure the radio landed on the same conducted power.
    NS_ABORT_MSG_IF(eirpFromTr38821 && std::abs(rs.GetSatEirpDbm() - satEirpDbm) > 1e-6,
                    "Set-1 conducted power mismatch: radio "
                        << rs.GetSatEirpDbm() << " dBm vs " << satEirpDbm << " dBm");
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
    // RRC-2c: give the transparent payload its feeder leg, as ntn-rrc-real-stack
    // does. Without a gateway NtnTimingAdvance returns a zero feeder range, so
    // --transparent=1 and --transparent=0 printed the same TA and the payload
    // flag was inert. TS 38.213 section 4.2: the common TA the network signals
    // covers the feeder (gateway to satellite) round trip; TR 38.821 Table
    // 4.2-2 lists the transparent RTT as service plus feeder. The gateway is a
    // fixed ground station 300 km from the t=0 sub-point, in view for the pass.
    if (transparent)
    {
        Ptr<ConstantPositionMobilityModel> gwMob = CreateObject<ConstantPositionMobilityModel>();
        gwMob->SetPosition(ntngeo::GeodeticToEcef(subLat + 2.7, subLon, 0.0));
        helper.SetGatewayMobility(gwMob);
    }
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
    g_drxStopS = simTimeSec;
    Simulator::ScheduleNow(&DrxActivityPoll, drx);

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
