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
#include "ns3/command-line.h"
#include "ns3/core-module.h"
#include "ns3/mmwave-enb-net-device.h"
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
#include <iostream>

using namespace ns3;
using namespace ns3::ntnrrc;

NS_LOG_COMPONENT_DEFINE("NtnRrcDrxDataTraffic");

namespace
{
NtnRealStackHelper* g_rs = nullptr;
Ptr<NtnTimingAdvance> g_ta;
Ptr<NtnSib19Broadcaster> g_sib19;
Ptr<NtnDrxStateMachine> g_drx;
bool g_drxEnabled = true;
double g_simTime = 20.0;

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
    double satEirpDbm = 55.0;
    bool drxEnabled = true;
    double drxLongCycleMs = 320.0;
    double drxOnDurationMs = 40.0;
    std::string outputDir = "ntn-rrc-drx-data-traffic-output";

    CommandLine cmd(__FILE__);
    cmd.AddValue("simSeconds", "Simulation duration (s)", simSeconds);
    cmd.AddValue("numUes", "Number of UEs on the serving cell", numUes);
    cmd.AddValue("leoAltKm", "Satellite altitude (km)", leoAltKm);
    cmd.AddValue("freqGHz", "Carrier frequency (GHz)", freqGHz);
    cmd.AddValue("satEirpDbm", "Satellite EIRP / gNB Tx power (dBm)", satEirpDbm);
    cmd.AddValue("drxOn", "Enable connected-mode DRX gating", drxEnabled);
    cmd.AddValue("drxLongCycleMs", "DRX long cycle (ms)", drxLongCycleMs);
    cmd.AddValue("drxOnDurationMs", "DRX on-duration (ms)", drxOnDurationMs);
    cmd.AddValue("outputDir", "Output directory", outputDir);
    cmd.Parse(argc, argv);
    g_drxEnabled = drxEnabled;
    g_simTime = simSeconds;

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
    rs.SetSimTime(Seconds(simSeconds));
    rs.SetOutputDir(outputDir);
    rs.SetRunTag("ntn-rrc-drx-data-traffic");
    rs.SetCarrierFrequencyHz(freqGHz * 1e9);
    rs.SetSatEirpDbm(satEirpDbm);
    rs.Build(satNodes, ueNodes);
    rs.InstallTraffic(NtnRealStackHelper::TrafficProfile::EmbbStreaming,
                      Seconds(1.0), Seconds(simSeconds - 0.5));
    rs.EnableAiFlowMonitor("ntn-rrc-drx-data-traffic"); // WS2 KPM series (TS 28.552 names)
    g_rs = &rs;

    // ----- RRC NTN procedures bound to the same geometry -----
    g_ta = CreateObject<NtnTimingAdvance>();
    g_ta->SetUeMobility(ueMob);
    g_ta->SetSatelliteMobility(satMob);
    g_ta->SetReferencePosition(ntngeo::GeodeticToEcef(subLat, subLon, 0.0));

    Ptr<mmwave::MmWaveEnbNetDevice> enb =
        DynamicCast<mmwave::MmWaveEnbNetDevice>(rs.GetEnbDevices().Get(0));
    const uint16_t cellId = enb ? enb->GetCellId() : 1;
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

    std::printf("# ntn-rrc-drx-data-traffic (SIB19 + TA + DRX on a real mmwave NR cell)\n"
                "#   sim=%.0fs alt=%.0fkm freq=%.1fGHz EIRP=%.1fdBm DRX=%s "
                "(long=%.0fms on=%.0fms)\n",
                simSeconds, leoAltKm, freqGHz, satEirpDbm, drxEnabled ? "on" : "off",
                drxLongCycleMs, drxOnDurationMs);

    Simulator::Schedule(Seconds(2.0), &GeoTick);
    Simulator::Stop(Seconds(simSeconds));
    Simulator::Run();
    rs.Collect();
    rs.WriteHealthReport();

    const double dutyPct = drxEnabled ? 100.0 * drxOnDurationMs / drxLongCycleMs : 100.0;
    const double measGoodput = rs.GetRxThroughputMbps();
    std::printf("# === summary ===  DRX=%s (on-duty=%.0f%%)  measured SINR=%.2f dB  "
                "measured TBLER=%.4f  measured goodput=%.3f Mbps  "
                "DRX effective goodput=%.3f Mbps (power-saving trade-off)\n",
                drxEnabled ? "on" : "off", dutyPct, rs.GetMeanDlSinrDb(), rs.GetMeanDlTbler(),
                measGoodput, measGoodput * dutyPct / 100.0);
    Simulator::Destroy();
    return 0;
}
