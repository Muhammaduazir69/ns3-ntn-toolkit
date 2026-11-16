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
    double freqGHz = 2.185; // TS 38.101-5 n256 DOWNLINK block (2170-2200 MHz). // S-band (so the oran-ntn TR 38.811 band matches)
    double satEirpDbm = -1.0; // sentinel: TR 38.821 Set-1 S-band EIRP density
    double rainRateMmH = 0.0;
    std::string radio = "nr"; // radio spine: "nr" (5G-LENA FR1) | "mmwave" (FR2)
    std::string outputDir = "ntn-sionna-composed-channel-output";
    std::string extraFading = "none"; // Wave 2 M3 / NT-07: one small-scale process by default

    CommandLine cmd(__FILE__);
    cmd.AddValue("simSeconds", "Simulation duration (s)", simSeconds);
    cmd.AddValue("freqGHz", "Carrier frequency (GHz)", freqGHz);
    cmd.AddValue("satEirpDbm",
                 "-1 = TR 38.821 Set-1 S-band EIRP density (34 dBW/MHz); otherwise conducted power at the array input (dBm)",
                 satEirpDbm);
    cmd.AddValue("radio", "Radio backend: nr (FR1) or mmwave", radio);
    cmd.AddValue("rainRateMmH", "Rain rate on the ITU-R chain (mm/h)", rainRateMmH);
    cmd.AddValue("outputDir", "Output directory", outputDir);
    cmd.AddValue("extraFading",
                 "none (default: the 3GPP spatial channel is the only small-scale process) or "
                 "nakagami (old demo: a second, independent m=3 draw per TB on top)",
                 extraFading);
    cmd.Parse(argc, argv);

    std::printf("# ntn-sionna-composed-channel-traffic (Roadmap §4.2.8, REAL radio)\n");
    std::printf("#   sim=%.0fs freq=%.1fGHz rain=%.1fmm/h\n",
                simSeconds, freqGHz, rainRateMmH);
    std::printf("#   chain: Friis -> ITU-R excess%s -> spectrum PHY (3GPP spatial fading)\n",
                extraFading == "nakagami" ? " -> Nakagami m=3" : "");

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
    satSgp4->SetElementsSgp4(elements[0]);
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
    // NT-02: by default (and on an S-band FR1 carrier) the cell takes the
    // TR 38.821 Table 6.1.1.1-1 Set-1 downlink EIRP density for the S-band LEO
    // reference payload, declared as a DENSITY so the helper back-computes
    // conducted power against the array gain instead of the antenna being
    // counted twice. An explicit --satEirpDbm used to reach only the printed
    // drop-in column; it is now the cell's CONDUCTED power at the array input.
    const bool eirpFromTr38821 = satEirpDbm < 0.0 && freqGHz > 1.0 && freqGHz <= 7.125;
    if (eirpFromTr38821)
    {
        rs.SetSatEirpDensityDbwMhz(NtnRealStackHelper::kTr38821Set1SBandEirpDensityDbwMhz);
    }
    else
    {
        if (satEirpDbm < 0.0)
        {
            satEirpDbm = 70.0; // legacy default for a carrier outside Set-1
        }
        rs.SetSatConductedPowerDbm(satEirpDbm);
    }
    rs.Build(satNodes, gndNodes);
    satEirpDbm = rs.GetSatEirpDbm(); // conducted power the radio actually uses
    // The oran-ntn drop-in adds no antenna gain (its input is radiated EIRP),
    // so it is fed the cell's effective EIRP, conducted power plus array gain,
    // rather than the conducted figure. It used to get a flat 70 dBm.
    const double oranEirpDbm = rs.GetEffectiveEirpDbm();
    std::printf("#   gNB conducted=%.1fdBm effective EIRP=%.1fdBm (%s)\n", satEirpDbm,
                oranEirpDbm,
                eirpFromTr38821 ? "TR 38.821 Set-1 S-band density" : "explicit conducted");

    // (1) the composed chain in the REAL packet path.
    Ptr<NtnAtmosphericLossChain> chain = CreateObject<NtnAtmosphericLossChain>();
    chain->SetFrequencyHz(freqGHz * 1e9);
    chain->SetRainRateMmH(rainRateMmH);
    Ptr<NtnAtmosphericPropagationLossModel> atmo =
        CreateObject<NtnAtmosphericPropagationLossModel>();
    atmo->SetChain(chain);
    rs.AddExtraPropagationLoss(atmo);

    // LEO LOS is Rician-like: Nakagami m=3 (about K = 7 dB) at every range.
    // The fading delta varies per transmission (i.i.d. per TB, no Doppler
    // correlation). SIONNA-07 note: the nr backend's 3GPP spectrum model already
    // applies small-scale fading, so this is a second, independent fading
    // realization on the same link (see SCOPE_AND_LIMITATIONS NT-07); it is a
    // composition demo, not a calibrated NTN fading model.
    // Wave 2 M3: off by default (TR 38.811 6.7 fading should be the only
    // small-scale term); --extraFading=nakagami keeps the old composition demo.
    if (extraFading == "nakagami")
    {
        Ptr<NakagamiPropagationLossModel> nakagami =
            CreateObject<NakagamiPropagationLossModel>();
        nakagami->SetAttribute("m0", DoubleValue(3.0));
        nakagami->SetAttribute("m1", DoubleValue(3.0));
        nakagami->SetAttribute("m2", DoubleValue(3.0));
        rs.AddExtraPropagationLoss(nakagami);
    }

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
        [&rs, atmo, oran, gndMob, satEnu, oranEirpDbm, &lastRx](Time now) {
            // Drop-in alternative evaluated on the live geometry.
            const double oranRxDbm = oran->CalcRxPower(oranEirpDbm, satEnu, gndMob);
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
                "throughput=%.3f Mbps (ITU-R excess%s composed in "
                "the real packet path; oran-ntn TR 38.811 shown as drop-in)\n",
                rs.GetMeanDlSinrDb(), rs.GetMeanDlTbler(), rs.GetRxThroughputMbps(),
                extraFading == "nakagami" ? " + Nakagami fading" : "");

    Simulator::Destroy();
    return 0;
}
