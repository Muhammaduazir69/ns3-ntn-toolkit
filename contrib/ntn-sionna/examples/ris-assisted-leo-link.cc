/*
 * SPDX-License-Identifier: GPL-2.0-only
 * Copyright (c) 2026 Muhammad Uzair (ns3-ntn-toolkit, Roadmap §4.2.12)
 *
 * ris-assisted-leo-link — a RIS-assisted LEO ground link whose headline KPIs
 * (DL SINR / TBLER / goodput) are MEASURED on a real mmwave NR packet plane
 * (NtnRealStackHelper) with the ITU-R atmospheric excess in the packet path.
 *
 * History (2026-06-24 fidelity fix): the previous version was a probe-only
 * pair of NtnSionnaCascadeChannel CalcRxPower() queries (no-RIS vs RIS) under
 * a Simulator::Schedule loop — the RIS gain was a printed Rx-power scalar that
 * never reached a measured KPI. It now drives a REAL packet plane:
 *
 *   - real SGP4 satellite (ENU-projected) + fixed ground UE;
 *   - a real mmwave NR cell carries EmbbStreaming traffic; DL SINR / TBLER /
 *     goodput are MEASURED (GetMeanDlSinrDb / GetRxThroughputMbps, etc.);
 *   - the ITU-R atmospheric cascade rides the packet path as the established
 *     no-double-count excess adapter (NtnAtmosphericPropagationLossModel
 *     wrapping NtnAtmosphericLossChain — the exact adapter used in
 *     ntn-sionna-ris-relay-traffic.cc).
 *
 * The RIS reflection gain remains an analytic probe column. AddExtraPropagation-
 * Loss chains AFTER the built-in Friis FSPL via SetNext and can only SUBTRACT
 * loss; a RIS reflection is a POSITIVE power delta, and there is no existing
 * "negative-loss" / gain adapter that injects power onto the real plane without
 * net-new functionality. Rather than fabricate one, the RIS gain is computed
 * from the SAME two-channel Sionna cascade probe (rx_RIS - rx_noRIS) and printed
 * as a per-tick gain column beside the MEASURED SINR/goodput. The measured-radio
 * RIS counterpart (blockage recovery as a LIVE channel reconfiguration) is
 * ntn-sionna-ris-relay-traffic.cc.
 *
 * Run:
 *   ./ns3 run "ris-assisted-leo-link --freqHz=28e9 --rows=32 --cols=32"
 */
#include "ns3/command-line.h"
#include "ns3/constant-position-mobility-model.h"
#include "ns3/core-module.h"

#include <iostream>
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/ntn-real-stack-helper.h"
#include "ns3/ntn-tr38811-mobility-model.h"
#include "ns3/sgp4-mobility-model.h"
#include "ns3/simulator.h"
#include "ns3/walker-constellation.h"

#include "ns3/ns3-sionna-channel.h"
#include "ns3/ntn-atmospheric-loss-chain.h"
#include "ns3/ntn-atmospheric-propagation-loss-model.h"
#include "ns3/ntn-sionna-cascade-channel.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <numeric>
#include <vector>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("RisAssistedLeoLink");

namespace
{
NtnRealStackHelper* g_rs = nullptr;
Ptr<NtnSionnaCascadeChannel> g_chNoRis;
Ptr<NtnSionnaCascadeChannel> g_chRis;
Ptr<NtnEnuProjectionMobilityModel> g_satEnu;
Ptr<MobilityModel> g_ueMob;
double g_simTime = 30.0;
uint64_t g_lastRx = 0;
std::vector<double> g_gains;

void
Tick(Time now)
{
    const double t = now.GetSeconds();
    if (t >= g_simTime)
    {
        return;
    }
    // Analytic RIS-reflection probe (POSITIVE power delta — no real-plane
    // gain-adapter exists, so it stays an analytic column).
    const double txDbm = 30.0;
    const double rxNo = g_chNoRis->CalcRxPower(txDbm, g_satEnu, g_ueMob);
    const double rxYes = g_chRis->CalcRxPower(txDbm, g_satEnu, g_ueMob);
    const double risGainDb = rxYes - rxNo;
    g_gains.push_back(risGainDb);
    const double elevDeg = g_chRis->GetLastComponents().elevationDeg;

    // MEASURED radio KPIs from the real packet plane.
    const double sinr = g_rs->GetUeRecentSinrDb(0);
    const double tbler = g_rs->GetUeRecentTbler(0);
    const uint64_t rx = g_rs->GetUeRxBytes(0);
    const double mbps = (rx - g_lastRx) * 8.0 / 1e6;
    g_lastRx = rx;

    std::printf("  %6.2f  %7.2f  %9.3f  %9.2f  %8.3f  %9.3f\n",
                t, elevDeg, risGainDb, sinr, tbler, mbps);
}
} // namespace

int
main(int argc, char* argv[])
{
    double duration = 30.0;
    std::string host = "127.0.0.1";
    uint16_t port = 8765;
    double freqHz = 28.0e9;
    double altKm = 550.0;
    double rainMmH = 0.0;
    double satEirpDbm = 75.0;
    uint32_t risRows = 32;
    uint32_t risCols = 32;
    double risPosX = 706.5; // halfway between UE and sub-sat ground point
    double risPosZ = 50.0;
    std::string phaseProfile = "focus";
    uint32_t timeoutMs = 200;
    std::string radio = "nr"; // radio spine: "nr" (5G-LENA FR1) | "mmwave" (FR2)
    std::string outputDir = "ris-assisted-leo-link-output";

    CommandLine cmd(__FILE__);
    cmd.AddValue("duration", "Simulation duration (s)", duration);
    cmd.AddValue("host", "Sionna server host", host);
    cmd.AddValue("port", "Sionna server UDP port", port);
    cmd.AddValue("freqHz", "Carrier frequency (Hz)", freqHz);
    cmd.AddValue("altKm", "Satellite altitude (km)", altKm);
    cmd.AddValue("rainMmH", "Rain rate (mm/h)", rainMmH);
    cmd.AddValue("satEirpDbm", "Satellite EIRP / gNB Tx power (dBm)", satEirpDbm);
    cmd.AddValue("radio", "Radio backend: nr (FR1) or mmwave", radio);
    cmd.AddValue("rows", "RIS rows", risRows);
    cmd.AddValue("cols", "RIS cols", risCols);
    cmd.AddValue("risPosX", "RIS x (m)", risPosX);
    cmd.AddValue("risPosZ", "RIS altitude (m)", risPosZ);
    cmd.AddValue("phaseProfile", "RIS phase profile (focus/flat/random)", phaseProfile);
    cmd.AddValue("timeoutMs", "Per-query timeout (ms)", timeoutMs);
    cmd.AddValue("outputDir", "Output directory", outputDir);
    cmd.Parse(argc, argv);
    g_simTime = duration;

    std::printf("\n=== ris-assisted-leo-link (MEASURED real radio + RIS analytic probe) ===\n"
                "  serving cell: real NR (%s) link, 1 UE, real SGP4 pass\n"
                "  ITU-R atmospheric excess chained on the packet path (no double-count)\n"
                "  RIS reflection gain remains an analytic probe (no real-plane gain-adapter)\n"
                "  freq=%.3f GHz alt=%.0f km RIS=%ux%u phase=%s\n\n",
                radio.c_str(), freqHz / 1e9, altKm, risRows, risCols, phaseProfile.c_str());

    NodeContainer satNodes;
    satNodes.Create(1);
    NodeContainer ueNodes;
    ueNodes.Create(1);

    // Real SGP4 orbit projected into the scenario's local ENU frame.
    ns3::ntncon::WalkerConfig wcfgSat;
    wcfgSat.num_planes = 1;
    wcfgSat.total_sats = 80;
    wcfgSat.altitude_km = altKm;
    wcfgSat.inclination_deg = 53.0;
    wcfgSat.epoch_unix_s = 1735689600.0;
    const auto satElements = ns3::ntncon::WalkerConstellation::BuildDelta(wcfgSat);
    Ptr<ns3::ntncon::Sgp4MobilityModel> satSgp4 =
        CreateObject<ns3::ntncon::Sgp4MobilityModel>();
    satSgp4->SetElements(satElements[0]);
    double satSubLat, satSubLon, satSubAlt;
    satSgp4->GetGeodetic(satSubLat, satSubLon, satSubAlt);
    Ptr<NtnEnuProjectionMobilityModel> satEnu = CreateObject<NtnEnuProjectionMobilityModel>();
    satEnu->SetSource(satSgp4);
    satEnu->SetReference(satSubLat, satSubLon, 0.0);
    satNodes.Get(0)->AggregateObject(satEnu);
    g_satEnu = satEnu;

    MobilityHelper mob;
    mob.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    Ptr<ListPositionAllocator> uePos = CreateObject<ListPositionAllocator>();
    uePos->Add(Vector(0.0, 0.0, 1.5));
    mob.SetPositionAllocator(uePos);
    mob.Install(ueNodes);
    g_ueMob = ueNodes.Get(0)->GetObject<MobilityModel>();

    NtnRealStackHelper rs;
    rs.SetRadioBackend(radio == "mmwave" ? NtnRealStackHelper::RadioBackend::Mmwave
                                         : NtnRealStackHelper::RadioBackend::Nr);
    if (radio != "mmwave")
    {
        rs.SetNumerology(1); // FR1 30 kHz SCS
    }
    rs.SetSimTime(Seconds(duration));
    rs.SetOutputDir(outputDir);
    rs.SetRunTag("ris-assisted-leo-link");
    rs.SetCarrierFrequencyHz(freqHz);
    // NT-02: declared as CONDUCTED power at the array input. This carrier has
    // no TR 38.821 Set-1 reference in the toolkit, so the EIRP health gate
    // reports "not asserted" rather than certifying an uncalibrated budget.
    rs.SetSatConductedPowerDbm(satEirpDbm);
    rs.Build(satNodes, ueNodes);

    // ITU-R atmospheric excess stays in the packet path the whole run (the
    // established no-double-count adapter used in ntn-sionna-ris-relay-traffic).
    Ptr<NtnAtmosphericLossChain> chainPkt = CreateObject<NtnAtmosphericLossChain>();
    chainPkt->SetFrequencyHz(freqHz);
    chainPkt->SetRainRateMmH(rainMmH);
    Ptr<NtnAtmosphericPropagationLossModel> atmo =
        CreateObject<NtnAtmosphericPropagationLossModel>();
    atmo->SetChain(chainPkt);
    rs.AddExtraPropagationLoss(atmo);

    rs.InstallTraffic(NtnRealStackHelper::TrafficProfile::EmbbStreaming,
                      Seconds(1.0), Seconds(duration - 0.5));
    rs.EnableAiFlowMonitor("ris-assisted-leo-link"); // WS2 KPM series (TS 28.552 names)
    g_rs = &rs;

    // --- Analytic RIS probe: two Sionna cascade channels (no-RIS vs RIS) ---
    Ptr<NtnSionnaChannel> baseNoRis = CreateObject<NtnSionnaChannel>();
    baseNoRis->SetServer(host, port);
    baseNoRis->SetFrequencyHz(freqHz);
    baseNoRis->SetTimeoutMs(timeoutMs);
    Ptr<NtnAtmosphericLossChain> chainNoRis = CreateObject<NtnAtmosphericLossChain>();
    chainNoRis->SetFrequencyHz(freqHz);
    chainNoRis->SetRainRateMmH(rainMmH);
    g_chNoRis = CreateObject<NtnSionnaCascadeChannel>();
    g_chNoRis->SetSionnaChannel(baseNoRis);
    g_chNoRis->SetAtmosphericChain(chainNoRis);

    Ptr<NtnSionnaChannel> baseRis = CreateObject<NtnSionnaChannel>();
    baseRis->SetServer(host, port);
    baseRis->SetFrequencyHz(freqHz);
    baseRis->SetTimeoutMs(timeoutMs);
    RisConfig ris;
    ris.pos_x = risPosX;
    ris.pos_y = 0.0;
    ris.pos_z = risPosZ;
    ris.normal_x = 0.0;
    ris.normal_y = 0.0;
    ris.normal_z = 1.0;
    ris.rows = static_cast<uint16_t>(risRows);
    ris.cols = static_cast<uint16_t>(risCols);
    ris.spacing_lambda = 0.5;
    ris.phase_profile = phaseProfile;
    ris.focal_x = 0.0;
    ris.focal_y = 0.0;
    ris.focal_z = 0.0;
    baseRis->SetRis(ris);
    Ptr<NtnAtmosphericLossChain> chainRis = CreateObject<NtnAtmosphericLossChain>();
    chainRis->SetFrequencyHz(freqHz);
    chainRis->SetRainRateMmH(rainMmH);
    g_chRis = CreateObject<NtnSionnaCascadeChannel>();
    g_chRis->SetSionnaChannel(baseRis);
    g_chRis->SetAtmosphericChain(chainRis);

    std::printf("# %6s  %7s  %9s  %9s  %8s  %9s\n",
                "t_s", "elev", "ris_gain", "sinr_dB", "tbler", "goodput");
    rs.RegisterPeriodicCallback(Seconds(1.0), &Tick);

    Simulator::Stop(Seconds(duration));
    Simulator::Run();
    rs.Collect();
    rs.WriteHealthReport();

    double gMin = 0.0, gMax = 0.0, gMean = 0.0;
    if (!g_gains.empty())
    {
        gMin = *std::min_element(g_gains.begin(), g_gains.end());
        gMax = *std::max_element(g_gains.begin(), g_gains.end());
        gMean = std::accumulate(g_gains.begin(), g_gains.end(), 0.0) / g_gains.size();
    }

    std::printf("\n--- ris-assisted-leo-link Summary (MEASURED real radio) ---\n"
                "  MEASURED DL SINR mean:        %.2f dB\n"
                "  measured DL TBLER (mean):     %.4f\n"
                "  measured DL throughput:       %.3f Mbps\n"
                "  analytic RIS gain (n=%zu):     min=%.3f max=%.3f mean=%.3f dB\n"
                "  -> headline SINR/TBLER/goodput are MEASURED; RIS reflection delta is an\n"
                "     analytic probe (no real-plane gain-adapter exists). Measured-radio RIS\n"
                "     counterpart: ntn-sionna-ris-relay-traffic.cc.\n",
                rs.GetMeanDlSinrDb(), rs.GetMeanDlTbler(), rs.GetRxThroughputMbps(),
                g_gains.size(), gMin, gMax, gMean);

    Simulator::Destroy();

    // WF-12: say what produced these numbers. GetFallbacks() existed and no
    // example read it, so on a host without Sionna every run here printed
    // Sionna framing over pure free-space results with nothing to show it.
    std::cout << baseNoRis->ProvenanceLine() << std::endl;
    return 0;
}
