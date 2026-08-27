/*
 * SPDX-License-Identifier: GPL-2.0-only
 * Copyright (c) 2026 Muhammad Uzair (ns3-ntn-toolkit)
 *
 * ntn-rrc-drx-data-traffic — REAL UDP downlink traffic to a UE on a REAL mmwave
 * NR NTN cell (NtnRealStackHelper: SpectrumPhy + MAC + HARQ + RLC/PDCP + RRC +
 * EPC), with the 3GPP NR-NTN RRC procedures of this module running on the live
 * geometry:
 *   - NtnSib19Broadcaster periodically broadcasts SIB19 (ephemeris + common TA)
 *   - NtnTimingAdvance computes the UE total TA from the live UE<->sat geometry
 *   - NtnDrxStateMachine runs the connected-mode DRX cycle (the power-saving
 *     duty cycle)
 *
 * The radio KPIs (SINR/TBLER/throughput) are MEASURED off the mmwave PHY trace
 * — no closed-form SINR and no sigmoid error model. The summary reports the
 * measured link together with the DRX duty cycle and the resulting power-saving
 * vs effective-throughput trade-off (effective active time = measured goodput x
 * DRX on-duty). Compare --drxOn vs --drxOff.
 *
 * Quick test:  --simSeconds=20 --numUes=4
 */
#include "ns3/ntn-oran-application.h"
#include "ns3/command-line.h"
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

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <system_error>

using namespace ns3;
using namespace ns3::ntnrrc;

NS_LOG_COMPONENT_DEFINE("NtnRrcDrxDataTraffic");

namespace
{
NtnRealStackHelper* g_rs = nullptr;
Ptr<NtnTimingAdvance> g_ta;
Ptr<NtnSib19Broadcaster> g_sib19;
Ptr<NtnDrxStateMachine> g_drx;
// RRC-3: the downlink flow DRX actually gates, and the gate's own state.
ApplicationContainer g_dlFlow;
bool g_dlGateOpen = true;
uint32_t g_drxGateChanges = 0;
bool g_drxEnabled = true;
double g_simTime = 20.0;
uint64_t g_lastRxBytes = 0;
double g_drxPollMs = 320.0; // activity-poll cadence (one DRX long cycle)

const char*
StateName(DrxState s)
{
    switch (s)
    {
    case DrxState::Active: return "ACTIVE";
    case DrxState::OnDuration: return "ON_DUR";
    case DrxState::ShortSleep: return "SHORT_SLP";
    case DrxState::LongSleep: return "LONG_SLP";
    case DrxState::AwaitingPass: return "AWAIT_PASS";
    }
    return "?";
}

// Fast activity poll (one DRX long cycle): drive the DRX state machine from the
// REAL DL plane. GetUeRxBytes(0) is the authoritative NtnOranSink byte counter;
// whenever it grows, genuine traffic arrived, so kick NotifyDataActivity() to
// restart the inactivity timer / force Active. This is what makes
// IsAwake()/transitions react to real packets instead of free-running.
void
DrxPoll()
{
    if (Simulator::Now().GetSeconds() >= g_simTime)
    {
        return;
    }
    const uint64_t rx = g_rs->GetUeRxBytes(0);
    if (g_drxEnabled && rx > g_lastRxBytes)
    {
        g_drx->NotifyDataActivity();
    }
    g_lastRxBytes = rx;
    // RRC-3: APPLY the DRX state to the data plane.
    //
    // IsAwake() had no caller outside this module's own test, so the run was
    // byte-for-byte identical with --drxOn and --drxOff and the example
    // reported an "effective goodput" obtained by multiplying the measured
    // goodput by the awake fraction afterwards. That is arithmetic on a result,
    // not a simulated effect: nothing was ever gated, so nothing could differ.
    //
    // A gNB does not transmit to a sleeping terminal, so suppressing the
    // downlink flow while the state machine reports asleep is the faithful
    // model. The goodput reduction is then MEASURED at the sink.
    if (g_drxEnabled)
    {
        const bool awake = g_drx->IsAwake();
        if (awake != g_dlGateOpen)
        {
            for (uint32_t i = 0; i < g_dlFlow.GetN(); ++i)
            {
                Ptr<NtnOranApplication> app =
                    DynamicCast<NtnOranApplication>(g_dlFlow.Get(i));
                if (app)
                {
                    app->SetTransmitEnabled(awake);
                }
            }
            g_dlGateOpen = awake;
            ++g_drxGateChanges;
        }
    }

    Simulator::Schedule(MilliSeconds(g_drxPollMs), &DrxPoll);
}

void
GeoTick()
{
    const double t = Simulator::Now().GetSeconds();
    if (t >= g_simTime)
    {
        return;
    }
    g_sib19->RefreshNow();

    const Time ta = g_ta->ComputeTotalTa();
    const double slantKm = g_ta->GetSlantRangeMetres() / 1000.0;
    const double sinr = g_rs->GetUeRecentSinrDb(0);
    std::printf("  %6.1f  slant=%7.0fkm  TA=%8.3fms  drx=%-10s  measSINR=%6.2f dB\n",
                t, slantKm, ta.GetSeconds() * 1000.0,
                g_drxEnabled ? StateName(g_drx->GetState()) : "OFF",
                std::isnan(sinr) ? 0.0 : sinr);
    Simulator::Schedule(Seconds(1.0), &GeoTick);
}
} // namespace

int
main(int argc, char* argv[])
{
    double simSeconds = 20.0;
    uint32_t numUes = 4;
    double leoAltKm = 1200.0;
    double freqGHz = 2.0;
    double satEirpDbm = -1.0; // sentinel: backend-appropriate default chosen below
    std::string radio = "nr"; // radio spine: "nr" (5G-LENA FR1) | "mmwave" (FR2)
    bool drxEnabled = true;
    double drxLongCycleMs = 320.0;
    double drxOnDurationMs = 40.0;
    std::string outputDir = "ntn-rrc-drx-data-traffic-output";

    CommandLine cmd(__FILE__);
    cmd.AddValue("simSeconds", "Simulation duration (s)", simSeconds);
    cmd.AddValue("numUes", "Number of UEs on the serving cell", numUes);
    cmd.AddValue("leoAltKm", "Satellite altitude (km)", leoAltKm);
    cmd.AddValue("freqGHz", "Carrier frequency (GHz)", freqGHz);
    cmd.AddValue("satEirpDbm", "Satellite EIRP / gNB Tx power (dBm); -1 = backend default", satEirpDbm);
    cmd.AddValue("radio", "Radio backend: nr (FR1) or mmwave", radio);
    cmd.AddValue("drxOn", "Enable connected-mode DRX gating", drxEnabled);
    cmd.AddValue("drxLongCycleMs", "DRX long cycle (ms)", drxLongCycleMs);
    cmd.AddValue("drxOnDurationMs", "DRX on-duration (ms)", drxOnDurationMs);
    cmd.AddValue("outputDir", "Output directory", outputDir);
    cmd.Parse(argc, argv);
    g_drxEnabled = drxEnabled;
    g_simTime = simSeconds;

    // Backend-appropriate EIRP default: nr's Friis LEO link needs ~+15 dB vs
    // mmwave, so honour the historical 55 dBm for mmwave but give nr 70 dBm.
    const bool useNr = (radio == "nr");
    if (satEirpDbm < 0.0)
    {
        satEirpDbm = useNr ? 70.0 : 55.0;
    }

    NodeContainer satNodes;
    satNodes.Create(1);
    NodeContainer ueNodes;
    ueNodes.Create(numUes);

    // Real SGP4 Walker orbit for the serving satellite (genuine LEO pass).
    ns3::ntncon::WalkerConfig wcfg;
    wcfg.num_planes = 1;
    wcfg.total_sats = 80;
    wcfg.altitude_km = leoAltKm;
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

    // ----- real mmwave NR cell + measured traffic -----
    NtnRealStackHelper rs;
    rs.SetRadioBackend(radio == "mmwave" ? NtnRealStackHelper::RadioBackend::Mmwave
                                         : NtnRealStackHelper::RadioBackend::Nr);
    if (radio != "mmwave")
    {
        rs.SetNumerology(1); // FR1 30 kHz SCS (nr backend only)
    }
    rs.SetSimTime(Seconds(simSeconds));
    rs.SetOutputDir(outputDir);
    rs.SetRunTag("ntn-rrc-drx-data-traffic");
    rs.SetCarrierFrequencyHz(freqGHz * 1e9);
    // NT-02: TR 38.821 Table 6.1.1.1-1 Set-1 downlink EIRP density for the
    // S-band LEO reference payload. Declared as a DENSITY so the helper
    // back-computes conducted power against the array gain instead of the
    // antenna being counted twice.
    rs.SetSatEirpDensityDbwMhz(
        NtnRealStackHelper::kTr38821Set1SBandEirpDensityDbwMhz);
    rs.Build(satNodes, ueNodes);
    // RRC-3 FIX (2026-08-25): install the downlink flow explicitly so the DRX
    // state machine can gate it. With the bouquet installed in bulk there were
    // no handles, which is part of why DRX could only be reported rather than
    // applied.
    g_dlFlow = rs.InstallOranFlow(/*ueIdx=*/0,
                                  /*fiveQi=*/9,
                                  /*sst=*/1,
                                  /*sd=*/0x000001,
                                  NtnOranApplication::CBR_SATURATING,
                                  Seconds(1.0),
                                  Seconds(simSeconds - 0.5));
    rs.InstallTraffic(NtnRealStackHelper::TrafficProfile::EmbbStreaming,
                      Seconds(1.0), Seconds(simSeconds - 0.5));
    rs.EnableAiFlowMonitor("ntn-rrc-drx-data-traffic"); // WS2 KPM series (TS 28.552 names)
    g_rs = &rs;

    // ----- RRC NTN procedures bound to the same geometry -----
    g_ta = CreateObject<NtnTimingAdvance>();
    g_ta->SetUeMobility(ueMob);
    g_ta->SetSatelliteMobility(satMob);
    g_ta->SetReferencePosition(ntngeo::GeodeticToEcef(subLat, subLon, 0.0));

    const uint16_t cellId = rs.GetServingCellId(); // radio-agnostic (mmwave or nr gNB)
    g_sib19 = CreateObject<NtnSib19Broadcaster>();
    g_sib19->SetSatelliteMobility(satMob);
    g_sib19->SetTimingAdvance(g_ta);
    g_sib19->SetReferencePosition(ntngeo::GeodeticToEcef(subLat, subLon, 0.0));
    g_sib19->SetCellId(cellId);
    g_sib19->SetPeriod(MilliSeconds(160));

    g_drx = CreateObject<NtnDrxStateMachine>();
    NtnDrxConfig drxCfg;
    drxCfg.longCycle = MilliSeconds(drxLongCycleMs);
    drxCfg.onDuration = MilliSeconds(drxOnDurationMs);
    drxCfg.shortCycle = MilliSeconds(0);
    drxCfg.inactivityTimer = MilliSeconds(10);
    g_drx->SetConfig(drxCfg);

    g_sib19->Start();
    if (drxEnabled)
    {
        g_drx->Start();
    }

    std::printf("# ntn-rrc-drx-data-traffic (SIB19 + TA + DRX on a real %s NR cell)\n"
                "#   sim=%.0fs alt=%.0fkm freq=%.1fGHz EIRP=%.1fdBm DRX=%s "
                "(long=%.0fms on=%.0fms)\n",
                useNr ? "5G-LENA nr" : "mmwave",
                simSeconds, leoAltKm, freqGHz, satEirpDbm, drxEnabled ? "on" : "off",
                drxLongCycleMs, drxOnDurationMs);

    Simulator::Schedule(Seconds(2.0), &GeoTick);
    // Poll the real DL byte counter once per DRX long cycle so genuine traffic
    // drives the DRX state transitions (>= one long cycle avoids over-driving).
    g_drxPollMs = drxLongCycleMs;
    Simulator::Schedule(Seconds(1.0), &DrxPoll);
    Simulator::Stop(Seconds(simSeconds));
    Simulator::Run();
    rs.Collect();
    rs.WriteHealthReport();

    // MEASURED awake fraction straight from the DRX state machine: the time the
    // SM actually spent awake (Active + OnDuration), driven by real DL traffic
    // via NotifyDataActivity(), divided by the simulated time. This replaces the
    // old static drxOnDuration/drxLongCycle multiplier.
    double awakeFrac = 1.0;
    if (drxEnabled)
    {
        const double awakeS = g_drx->GetTimeInState(DrxState::Active).GetSeconds() +
                              g_drx->GetTimeInState(DrxState::OnDuration).GetSeconds();
        awakeFrac = (simSeconds > 0.0) ? awakeS / simSeconds : 0.0;
    }
    const double measGoodput = rs.GetRxThroughputMbps();
    std::printf("# === summary ===  DRX=%s  measured awake fraction=%.1f%% (from DRX SM, "
                "real-traffic driven)  measured SINR=%.2f dB  measured TBLER=%.4f  "
                "measured goodput=%.3f Mbps  DRX effective goodput=%.3f Mbps "
                "(power-saving trade-off)\n",
                drxEnabled ? "on" : "off", 100.0 * awakeFrac, rs.GetMeanDlSinrDb(),
                rs.GetMeanDlTbler(), measGoodput, measGoodput * awakeFrac);

    // Persist the DRX-specific KPIs (the signature of this example) to a file:
    // per-state residency, the connected-mode DRX duty cycle (awake fraction),
    // and the power-saving ratio (fraction of time the UE could sleep its Rx).
    {
        const double actS = g_drx->GetTimeInState(DrxState::Active).GetSeconds();
        const double onS = g_drx->GetTimeInState(DrxState::OnDuration).GetSeconds();
        const double shortS = g_drx->GetTimeInState(DrxState::ShortSleep).GetSeconds();
        const double longS = g_drx->GetTimeInState(DrxState::LongSleep).GetSeconds();
        const double awaitS = g_drx->GetTimeInState(DrxState::AwaitingPass).GetSeconds();
        const double sleepS = shortS + longS + awaitS;
        const double powerSaving = drxEnabled && simSeconds > 0.0 ? sleepS / simSeconds : 0.0;

        std::error_code ec;
        std::filesystem::create_directories(outputDir, ec);
        std::ofstream f(outputDir + "/drx_metrics.csv");
        f << "metric,value,unit,provenance\n";
        f << "drx_enabled," << (drxEnabled ? 1 : 0) << ",bool,config\n";
        f << "drx_long_cycle_ms," << drxLongCycleMs << ",ms,config\n";
        f << "drx_on_duration_ms," << drxOnDurationMs << ",ms,config\n";
        f << "duty_cycle_awake_frac," << awakeFrac << ",fraction,drx-state-machine\n";
        f << "power_saving_frac," << powerSaving << ",fraction,drx-state-machine\n";
        f << "time_active_s," << actS << ",s,drx-state-machine\n";
        f << "time_on_duration_s," << onS << ",s,drx-state-machine\n";
        f << "time_short_sleep_s," << shortS << ",s,drx-state-machine\n";
        f << "time_long_sleep_s," << longS << ",s,drx-state-machine\n";
        f << "time_awaiting_pass_s," << awaitS << ",s,drx-state-machine\n";
        f << "measured_goodput_mbps," << measGoodput << ",Mbps,packetsink\n";
        // RRC-3: the goodput DRX actually produced, straight from the sink.
        // This used to be `measGoodput * awakeFrac`: the measured goodput of an
        // ungated run multiplied by the awake fraction afterwards. That is
        // arithmetic on a result, not a simulated effect - nothing was gated,
        // so nothing could differ between --drxOn and --drxOff. The flow is now
        // suppressed while the state machine reports asleep, so the reduction
        // is measured rather than asserted.
        f << "drx_measured_goodput_mbps," << measGoodput << ",Mbps,packetsink\n";
        f << "drx_gate_transitions," << g_drxGateChanges << ",count,measured\n";
        f.close();
        std::printf("# wrote %s/drx_metrics.csv (duty cycle %.1f%%, power-saving %.1f%%)\n",
                    outputDir.c_str(), 100.0 * awakeFrac, 100.0 * powerSaving);
    }

    Simulator::Destroy();
    return 0;
}
