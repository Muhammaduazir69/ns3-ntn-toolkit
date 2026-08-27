/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only
//
// thz-ntn-ric-controlled-traffic — Roadmap §4.3.7 (thz-ntn × oran-ntn closed-
// loop full-stack example) on a REAL mmwave NR NTN cell.
//
//   telemetry: every second the KPM tick reads the MEASURED DL SINR off the
//              mmwave PHY trace (RxPacketTraceUe) and submits it as an E2-KPM
//              report via OranNtnE2Node::SubmitKpmMeasurement(). The reported
//              value is the INTRINSIC link quality — measured SINR minus the
//              compensation the RIC itself applied — so the xApp's decision
//              tracks the channel, not its own actuation (no bang-bang).
//   control:   an xApp callback (SetIndicationCallback) receives every
//              indication; when intrinsic SINR falls below --sinrThreshDb it
//              ENGAGES the ground RIS, whose ThzNtnRis::ComputeSnrGain_dB()
//              array gain is applied as a LIVE channel reconfiguration
//              (NtnStaticExtraLossModel); when the channel recovers it
//              releases the RIS.
//   data:      the saturating downlink flow rides the same real radio, so
//              the goodput tracks the closed loop — it collapses when an
//              urban-canyon blockage hits mid-run, recovers when the xApp
//              engages the RIS, and stays up when the blockage clears.
//
// Audit fix (2026-06 protocol-fidelity audit): previously the KPM
// SINR was a closed-form FSPL+absorption formula and the data plane a P2P
// RateErrorModel behind a sigmoid SnrToPer() — now both halves of the loop
// ride the measured radio. THz molecular absorption stays in the packet path
// via ThzNtnPropagationLossModel. Mobility is real (SGP4 satellite).
//
// Quick test:  --simSeconds=40 --xapp=1   (re-run with --xapp=0 to see the
//              goodput stay down for the whole blockage window).
#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/ntn-real-stack-helper.h"
#include "ns3/ntn-static-extra-loss-model.h"
#include "ns3/ntn-tr38811-mobility-model.h"
#include "ns3/oran-ntn-e2-interface.h"
#include "ns3/oran-ntn-types.h"
#include "ns3/sgp4-mobility-model.h"
#include "ns3/thz-ntn-propagation-loss-model.h"
#include "ns3/thz-ntn-ris.h"
#include "ns3/walker-constellation.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("ThzNtnRicControlledTraffic");

namespace
{

NtnRealStackHelper* g_rs = nullptr;
Ptr<NtnStaticExtraLossModel> g_nlos; // blockage minus applied RIS compensation
Ptr<OranNtnE2Node> g_e2;

double g_sinrThreshDb = 15.0;
bool g_xappEnabled = true;
double g_blockageDb = 0.0;   // current canyon blockage (0 = LOS)
double g_risCompDb = 0.0;    // RIS compensation available when engaged
bool g_risEngaged = false;
uint32_t g_risEngagements = 0;
uint32_t g_kpmsSent = 0;
uint32_t g_xappActions = 0;

void
ApplyChannelState()
{
    const double comp = g_risEngaged ? std::min(g_blockageDb, g_risCompDb) : 0.0;
    g_nlos->SetLossDb(g_blockageDb - comp);
}

// The xApp: invoked on every E2-KPM indication. Engages/releases the RIS as a
// LIVE channel reconfiguration based on the reported intrinsic SINR.
void
RisXapp(E2Indication ind)
{
    ++g_xappActions;
    if (!g_xappEnabled)
    {
        return;
    }
    const double sinrDb = ind.kpmReport.sinr_dB; // intrinsic (see Tick)
    const bool wantEngage = (sinrDb < g_sinrThreshDb);
    if (wantEngage && !g_risEngaged)
    {
        g_risEngaged = true;
        ++g_risEngagements;
        ApplyChannelState();
    }
    else if (!wantEngage && g_risEngaged)
    {
        g_risEngaged = false;
        ApplyChannelState();
    }
}

} // namespace

int
main(int argc, char* argv[])
{
    double simSeconds = 40.0;
    double freqGHz = 100.0;    // sub-THz (3GPP spectrum model upper bound)
    double satEirpDbm = 115.0;
    double blockageDb = 35.0;
    double sinrThreshDb = 15.0;
    bool xappEnabled = true;
    uint32_t risN = 64; // 64x64 RIS panel
    std::string humidityProfile = "mid_latitude_summer";
    std::string outputDir = "thz-ntn-ric-controlled-output";
    std::string radio = "nr"; // radio backend: nr (FR1) or mmwave

    CommandLine cmd(__FILE__);
    cmd.AddValue("simSeconds", "Simulation duration (s)", simSeconds);
    cmd.AddValue("freqGHz", "THz carrier frequency (GHz), capped at 100", freqGHz);
    cmd.AddValue("satEirpDbm", "Satellite EIRP / gNB Tx power (dBm)", satEirpDbm);
    cmd.AddValue("radio", "Radio backend: nr (FR1) or mmwave", radio);
    cmd.AddValue("blockageDb", "Urban-canyon blockage applied mid-run (dB)", blockageDb);
    cmd.AddValue("sinrThreshDb",
                 "Intrinsic SINR threshold below which the xApp engages the RIS",
                 sinrThreshDb);
    cmd.AddValue("xapp", "Enable the RIS control xApp (0/1)", xappEnabled);
    cmd.AddValue("risN", "RIS panel side N (NxN elements)", risN);
    cmd.AddValue("humidityProfile", "Humidity profile label (reporting only)",
                 humidityProfile);
    cmd.AddValue("outputDir", "Output directory", outputDir);
    cmd.Parse(argc, argv);

    if (freqGHz > 100.0)
    {
        std::printf("# NOTE: 3GPP spectrum model caps the carrier at 100 GHz; "
                    "clamping %.0f -> 100 GHz\n",
                    freqGHz);
        freqGHz = 100.0;
    }
    g_sinrThreshDb = sinrThreshDb;
    g_xappEnabled = xappEnabled;

    // --- thz-ntn RIS physics (sets the recovery magnitude) ---
    Ptr<ThzNtnRis> ris = CreateObject<ThzNtnRis>();
    ris->Configure(risN, risN, freqGHz * 1e9, RisDeployment::GROUND);
    ris->ComputeOptimalPhases(45.0, 0.0, 0.0, 0.0);
    g_risCompDb = ris->ComputeSnrGain_dB(true);

    // --- nodes + REAL mobility (SGP4 satellite, fixed ground UE) ---
    NodeContainer satNodes;
    satNodes.Create(1);
    NodeContainer ueNodes;
    ueNodes.Create(1);

    ns3::ntncon::WalkerConfig wcfg;
    wcfg.num_planes = 1;
    wcfg.total_sats = 80;
    wcfg.altitude_km = 550.0;
    wcfg.inclination_deg = 53.0;
    wcfg.epoch_unix_s = 1735689600.0;
    const auto elements = ns3::ntncon::WalkerConstellation::BuildDelta(wcfg);
    Ptr<ns3::ntncon::Sgp4MobilityModel> satSgp4 =
        CreateObject<ns3::ntncon::Sgp4MobilityModel>();
    satSgp4->SetElements(elements[0]);
    double subLat, subLon, subAlt;
    satSgp4->GetGeodetic(subLat, subLon, subAlt);
    Ptr<NtnEnuProjectionMobilityModel> satEnu = CreateObject<NtnEnuProjectionMobilityModel>();
    satEnu->SetSource(satSgp4);
    satEnu->SetReference(subLat, subLon, 0.0);
    satNodes.Get(0)->AggregateObject(satEnu);

    MobilityHelper mob;
    mob.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    Ptr<ListPositionAllocator> uePos = CreateObject<ListPositionAllocator>();
    uePos->Add(Vector(0.0, 0.0, 1.5));
    mob.SetPositionAllocator(uePos);
    mob.Install(ueNodes);

    // --- REAL radio + THz physics in the packet path ---
    NtnRealStackHelper rs;
    rs.SetRadioBackend(radio == "mmwave" ? NtnRealStackHelper::RadioBackend::Mmwave
                                         : NtnRealStackHelper::RadioBackend::Nr);
    if (radio != "mmwave")
    {
        rs.SetNumerology(1); // FR1 30 kHz SCS
    }
    rs.SetSimTime(Seconds(simSeconds));
    rs.SetOutputDir(outputDir);
    rs.SetRunTag("thz-ntn-ric-controlled-traffic");
    rs.SetCarrierFrequencyHz(freqGHz * 1e9);
    // NT-02: declared as CONDUCTED power at the array input. This carrier has
    // no TR 38.821 Set-1 reference in the toolkit, so the EIRP health gate
    // reports "not asserted" rather than certifying an uncalibrated budget.
    rs.SetSatConductedPowerDbm(satEirpDbm);
    rs.Build(satNodes, ueNodes);
    g_rs = &rs;

    Ptr<ThzNtnPropagationLossModel> thz = CreateObject<ThzNtnPropagationLossModel>();
    thz->SetFrequency(freqGHz * 1e9);
    rs.AddExtraPropagationLoss(thz);

    g_nlos = CreateObject<NtnStaticExtraLossModel>();
    g_nlos->SetLossDb(0.0);
    rs.AddExtraPropagationLoss(g_nlos);

    rs.InstallTraffic(NtnRealStackHelper::TrafficProfile::EmbbStreaming,
                      Seconds(1.0), Seconds(simSeconds - 0.5));
    rs.EnableAiFlowMonitor("thz-ntn-ric-controlled-traffic");

    // --- oran-ntn E2 node + xApp closed-loop wiring ---
    g_e2 = CreateObject<OranNtnE2Node>();
    g_e2->SetNodeId(10);
    g_e2->SetIsNtn(true);
    g_e2->SetFeederLinkDelay(MilliSeconds(2));
    g_e2->RegisterRanFunction(2, "E2SM-KPM v03.00 (THz downlink measured SINR)");
    E2Subscription sub{};
    sub.subscriptionId = 1;
    sub.ranFunctionId = 2;
    sub.reportingPeriod = Seconds(1.0);
    sub.eventTrigger = false;
    g_e2->HandleSubscriptionRequest(sub);
    g_e2->SetIndicationCallback(MakeCallback(&RisXapp));

    // --- urban-canyon blockage event in the REAL channel ---
    const double tBlock = 0.30 * simSeconds;
    const double tClear = 0.75 * simSeconds;
    Simulator::Schedule(Seconds(tBlock), [blockageDb] {
        g_blockageDb = blockageDb;
        ApplyChannelState();
    });
    Simulator::Schedule(Seconds(tClear), [] {
        g_blockageDb = 0.0;
        ApplyChannelState();
    });

    std::printf("# thz-ntn-ric-controlled-traffic (Roadmap §4.3.7, measured radio)\n");
    std::printf("#   sim=%.0fs freq=%.0fGHz EIRP=%.1fdBm RIS=%ux%u (gain %.1f dB) "
                "xApp=%s thresh=%.1fdB\n",
                simSeconds, freqGHz, satEirpDbm, risN, risN, g_risCompDb,
                xappEnabled ? "on" : "off", sinrThreshDb);
    std::printf("#   blockage: %.0f dB @%.0fs, clears @%.0fs (live channel events)\n",
                blockageDb, tBlock, tClear);
    std::printf("# %5s  %9s  %9s  %5s  %8s  %8s  %9s\n",
                "t_s", "measured", "intrinsic", "ris", "extra_dB", "tbler", "goodput");

    // KPM tick: telemetry half of the loop, on the MEASURED radio.
    Ptr<MobilityModel> ueMob = ueNodes.Get(0)->GetObject<MobilityModel>();
    uint64_t lastRx = 0;
    rs.RegisterPeriodicCallback(
        Seconds(1.0),
        [ueMob, satEnu, &lastRx](Time now) {
            const double measured = g_rs->GetUeRecentSinrDb(0);
            const double comp =
                g_risEngaged ? std::min(g_blockageDb, g_risCompDb) : 0.0;
            const double intrinsic = measured - comp;
            const double tbler = g_rs->GetUeRecentTbler(0);
            const uint64_t rx = g_rs->GetUeRxBytes(0);
            const double mbps = (rx - lastRx) * 8.0 / 1e6;
            lastRx = rx;

            const Vector u = ueMob->GetPosition();
            const Vector s = satEnu->GetPosition();
            const double dx = s.x - u.x, dy = s.y - u.y, dz = s.z - u.z;
            const double elev =
                std::atan2(dz, std::max(std::sqrt(dx * dx + dy * dy), 1e-3)) * 180.0 / M_PI;

            if (!std::isnan(measured))
            {
                E2KpmReport report{};
                report.timestamp = now.GetSeconds();
                report.gnbId = 10;
                report.isNtn = true;
                report.ueId = 1;
                report.sinr_dB = intrinsic; // measured minus own actuation
                report.elevation_deg = elev;
                g_e2->SubmitKpmMeasurement(report);
                ++g_kpmsSent;
            }
            std::printf("  %5.1f  %9.2f  %9.2f  %5s  %8.2f  %8.3f  %9.3f\n",
                        now.GetSeconds(), measured, intrinsic,
                        g_risEngaged ? "ON" : "off", g_nlos->GetLossDb(), tbler, mbps);
        });

    Simulator::Stop(Seconds(simSeconds));
    Simulator::Run();
    rs.Collect();
    rs.WriteHealthReport();

    std::printf("# === summary ===  KPMs sent=%u  xApp actions=%u  RIS engagements=%u\n"
                "#                  measured cell SINR=%.2f dB  throughput=%.3f Mbps "
                "(closed loop on the measured radio)\n",
                g_kpmsSent, g_xappActions, g_risEngagements, rs.GetMeanDlSinrDb(),
                rs.GetRxThroughputMbps());

    Simulator::Destroy();
    return 0;
}
