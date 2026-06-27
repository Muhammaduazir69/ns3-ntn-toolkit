/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only
//
// ntn-sionna-composed-channel-traffic — Roadmap §4.2.8 (wire the ntn-sionna
// channel into the ns-3 propagation pipeline so modules compose) on a REAL
// mmwave NR NTN cell.
//
//   1. CORRECT COMPOSITION — the ITU-R atmospheric cascade (P.676 gaseous +
//      P.618/P.838 rain), re-homed as NtnAtmosphericPropagationLossModel
//      (pure EXCESS), and the built-in ns-3 NakagamiPropagationLossModel
//      (small-scale fading delta from src/propagation) are BOTH chained onto
//      the stack's Friis loss via the standard SetNext() pipeline:
//
//          Friis --[ ITU-R excess ]--[ Nakagami fading ]--> spectrum PHY
//
//      One full path-loss model + delta models — the ns-3 idiom; nothing is
//      double-counted, and the composition governs every transmitted packet.
//
//   2. INTERCHANGEABILITY with oran-ntn — OranNtnChannelModel (oran-ntn's TR
//      38.811 model) is evaluated each second on the SAME live geometry and
//      printed beside the measured SINR, showing the two channels as drop-in
//      alternatives for the same PropagationLossModel slot.
//
//   3. REAL DATA PLANE — the audit fix: previously the composed model was
//      polled in a probe loop driving a sigmoid SnrToPer() RateErrorModel.
//      Now the composed chain attenuates real packets on the real radio, so
//      the Nakagami tick-to-tick variation and the rain excess show up in
//      the MEASURED SINR / TBLER / goodput.
//
// Mobility is real: SGP4 satellite (ENU-projected), fixed ground terminal.
//
// Quick test:  --simSeconds=40 --rainRateMmH=15
#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/ntn-atmospheric-loss-chain.h"
#include "ns3/ntn-atmospheric-propagation-loss-model.h"
#include "ns3/ntn-real-stack-helper.h"
#include "ns3/ntn-tr38811-mobility-model.h"
#include "ns3/oran-ntn-channel-model.h"
#include "ns3/propagation-module.h" // NakagamiPropagationLossModel
#include "ns3/sgp4-mobility-model.h"
#include "ns3/walker-constellation.h"

#include <cmath>
#include <cstdio>
#include <string>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("NtnSionnaComposedChannelTraffic");

int
main(int argc, char* argv[])
{
    double simSeconds = 40.0;
    double freqGHz = 2.0; // S-band (so the oran-ntn TR 38.811 band matches)
    double satEirpDbm = 70.0; // healthy nr (FR1 Friis) LEO downlink
    double rainRateMmH = 0.0;
    std::string radio = "nr"; // radio spine: "nr" (5G-LENA FR1) | "mmwave" (FR2)
    std::string outputDir = "ntn-sionna-composed-channel-output";

    CommandLine cmd(__FILE__);
    cmd.AddValue("simSeconds", "Simulation duration (s)", simSeconds);
    cmd.AddValue("freqGHz", "Carrier frequency (GHz)", freqGHz);
    cmd.AddValue("satEirpDbm", "Satellite EIRP / gNB Tx power (dBm)", satEirpDbm);
    cmd.AddValue("radio", "Radio backend: nr (FR1) or mmwave", radio);
    cmd.AddValue("rainRateMmH", "Rain rate on the ITU-R chain (mm/h)", rainRateMmH);
    cmd.AddValue("outputDir", "Output directory", outputDir);
    cmd.Parse(argc, argv);

    std::printf("# ntn-sionna-composed-channel-traffic (Roadmap §4.2.8, REAL radio)\n");
    std::printf("#   sim=%.0fs freq=%.1fGHz EIRP=%.1fdBm rain=%.1fmm/h\n",
                simSeconds, freqGHz, satEirpDbm, rainRateMmH);
    std::printf("#   chain: Friis -> ITU-R excess -> Nakagami fading -> spectrum PHY\n");

    NodeContainer satNodes;
    satNodes.Create(1);
    NodeContainer gndNodes;
    gndNodes.Create(1);

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
    Ptr<ListPositionAllocator> gndPos = CreateObject<ListPositionAllocator>();
    gndPos->Add(Vector(0.0, 0.0, 1.5));
    mob.SetPositionAllocator(gndPos);
    mob.Install(gndNodes);

    NtnRealStackHelper rs;
    rs.SetRadioBackend(radio == "mmwave" ? NtnRealStackHelper::RadioBackend::Mmwave
                                         : NtnRealStackHelper::RadioBackend::Nr);
    if (radio != "mmwave")
    {
        rs.SetNumerology(1); // FR1 30 kHz SCS
    }
    rs.SetSimTime(Seconds(simSeconds));
    rs.SetOutputDir(outputDir);
    rs.SetRunTag("ntn-sionna-composed-channel-traffic");
    rs.SetCarrierFrequencyHz(freqGHz * 1e9);
    rs.SetSatEirpDbm(satEirpDbm);
    rs.Build(satNodes, gndNodes);

    // (1) the composed chain in the REAL packet path.
    Ptr<NtnAtmosphericLossChain> chain = CreateObject<NtnAtmosphericLossChain>();
    chain->SetFrequencyHz(freqGHz * 1e9);
    chain->SetRainRateMmH(rainRateMmH);
    Ptr<NtnAtmosphericPropagationLossModel> atmo =
        CreateObject<NtnAtmosphericPropagationLossModel>();
    atmo->SetChain(chain);
    rs.AddExtraPropagationLoss(atmo);

    // LEO LOS is Rician-like: Nakagami m>1 in the near field, heavier tails
    // further out. The fading delta varies per transmission — real packets
    // see a different draw each TB.
    Ptr<NakagamiPropagationLossModel> nakagami =
        CreateObject<NakagamiPropagationLossModel>();
    nakagami->SetAttribute("m0", DoubleValue(3.0));
    nakagami->SetAttribute("m1", DoubleValue(3.0));
    nakagami->SetAttribute("m2", DoubleValue(3.0));
    rs.AddExtraPropagationLoss(nakagami);

    // (2) oran-ntn's TR 38.811 channel for the same slot, evaluated on the
    // SAME live geometry as a drop-in alternative (printed, not in-path).
    Ptr<OranNtnChannelModel> oran = CreateObject<OranNtnChannelModel>();
    oran->SetBand("S-band");
    oran->SetEnvironment("rural");
    oran->SetAtmosphericAttenuation(true);

    rs.InstallTraffic(NtnRealStackHelper::TrafficProfile::EmbbStreaming,
                      Seconds(1.0), Seconds(simSeconds - 0.5));
    rs.EnableAiFlowMonitor("ntn-sionna-composed-channel-traffic"); // WS2 KPM series (TS 28.552 names)

    std::printf("# %5s  %8s  %9s  %8s  %8s  %9s\n",
                "t_s", "ituDb", "oranRxDbm", "sinr_dB", "tbler", "goodput");

    Ptr<MobilityModel> gndMob = gndNodes.Get(0)->GetObject<MobilityModel>();
    uint64_t lastRx = 0;
    rs.RegisterPeriodicCallback(
        Seconds(1.0),
        [&rs, atmo, oran, gndMob, satEnu, satEirpDbm, &lastRx](Time now) {
            // Drop-in alternative evaluated on the live geometry.
            const double oranRxDbm = oran->CalcRxPower(satEirpDbm, satEnu, gndMob);
            const double sinr = rs.GetUeRecentSinrDb(0);
            const double tbler = rs.GetUeRecentTbler(0);
            const uint64_t rx = rs.GetUeRxBytes(0);
            const double mbps = (rx - lastRx) * 8.0 / 1e6;
            lastRx = rx;
            std::printf("  %5.1f  %8.2f  %9.2f  %8.2f  %8.3f  %9.3f\n",
                        now.GetSeconds(), atmo->GetLastLossDb(), oranRxDbm, sinr,
                        tbler, mbps);
        });

    Simulator::Stop(Seconds(simSeconds));
    Simulator::Run();
    rs.Collect();
    rs.WriteHealthReport();

    std::printf("# === summary ===  measured cell SINR=%.2f dB TBLER=%.4f "
                "throughput=%.3f Mbps (ITU-R excess + Nakagami fading composed in "
                "the real packet path; oran-ntn TR 38.811 shown as drop-in)\n",
                rs.GetMeanDlSinrDb(), rs.GetMeanDlTbler(), rs.GetRxThroughputMbps());

    Simulator::Destroy();
    return 0;
}
