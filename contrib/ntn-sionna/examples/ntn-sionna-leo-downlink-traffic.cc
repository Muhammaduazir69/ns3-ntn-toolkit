/*
 * SPDX-License-Identifier: GPL-2.0-only
 * Copyright (c) 2026 Muhammad Uzair (ns3-ntn-toolkit)
 *
 * ntn-sionna-leo-downlink-traffic — END-TO-END packet transmission over a
 * LEO downlink on a REAL mmwave NR NTN cell (NtnRealStackHelper: SpectrumPhy +
 * MAC + HARQ + RLC/PDCP + RRC + EPC), with the ntn-sionna ITU-R atmospheric
 * cascade (P.676 gaseous + P.618/P.838 rain + optional P.681 LMS shadowing)
 * as a LIVE channel plug-in.
 *
 * Audit fix (2026-06 protocol-fidelity audit, channel-plugin recipe):
 * the old version asked NtnSionnaCascadeChannel for an Rx power in a probe
 * loop and drove a P2P RateErrorModel through a sigmoid SnrToPer() — packets
 * never crossed a radio. Here the atmospheric physics is re-homed as
 * NtnAtmosphericPropagationLossModel (pure EXCESS — FSPL comes from the
 * stack's own Friis model over the live SGP4 geometry) and chained via
 * AddExtraPropagationLoss(), so it attenuates the transmitted packets and
 * shows up in the MEASURED SINR / TBLER / goodput. With --lms=1 the P.681
 * Lutz Markov chain adds temporally-correlated land-mobile-satellite fades
 * that real packets feel.
 *
 * Mobility is real: the satellite is an SGP4 Walker element projected into
 * the scenario's local ENU frame (genuine pass dynamics; the slant range and
 * elevation evolve with the orbit), and the gateway is a fixed ground site.
 *
 * Run:
 *   ./ns3 run "ntn-sionna-leo-downlink-traffic --simSeconds=60 --rainMmH=10 --lms=1"
 */
#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/ntn-atmospheric-loss-chain.h"
#include "ns3/ntn-atmospheric-propagation-loss-model.h"
#include "ns3/ntn-real-stack-helper.h"
#include "ns3/ntn-scene-helper.h"
#include "ns3/ntn-tr38811-mobility-model.h"
#include "ns3/sgp4-mobility-model.h"
#include "ns3/walker-constellation.h"

#include <cmath>
#include <cstdio>
#include <string>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("NtnSionnaLeoDownlinkTraffic");

namespace
{

double
ElevDegEnu(const Vector& gnd, const Vector& sat)
{
    const double dx = sat.x - gnd.x;
    const double dy = sat.y - gnd.y;
    const double dz = sat.z - gnd.z;
    const double horiz = std::max(std::sqrt(dx * dx + dy * dy), 1e-3);
    return std::atan2(dz, horiz) * 180.0 / M_PI;
}

double
DistM(const Vector& a, const Vector& b)
{
    const double dx = a.x - b.x, dy = a.y - b.y, dz = a.z - b.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

} // namespace

int
main(int argc, char* argv[])
{
    double simSeconds = 60.0;
    double freqGHz = 12.0;    // Ku-band gateway downlink
    double satEirpDbm = 75.0; // Ku feeder beam (closes ~169 dB FSPL with margin)
    double rainMmH = 0.0;
    bool lms = false;
    std::string outputDir = "ntn-sionna-leo-downlink-output";

    CommandLine cmd(__FILE__);
    cmd.AddValue("simSeconds", "Simulation duration (s)", simSeconds);
    cmd.AddValue("freqGHz", "Carrier frequency (GHz)", freqGHz);
    cmd.AddValue("satEirpDbm", "Satellite EIRP / gNB Tx power (dBm)", satEirpDbm);
    cmd.AddValue("rainMmH", "Static rain rate over the run (mm/h)", rainMmH);
    cmd.AddValue("lms", "Enable P.681 LMS Markov shadowing (0/1)", lms);
    cmd.AddValue("outputDir", "Output directory", outputDir);
    std::string netSimOut;
    std::string czmlOut;
    cmd.AddValue("netSim", "NetSimulyzer 3D JSON output (empty=off)", netSimOut);
    cmd.AddValue("czml", "Cesium CZML 3D output (empty=off)", czmlOut);
    cmd.Parse(argc, argv);

    std::printf("# ntn-sionna-leo-downlink-traffic (REAL radio, ITU-R cascade in the "
                "packet path)\n");
    std::printf("#   sim=%.0fs freq=%.1fGHz EIRP=%.1fdBm rain=%.1fmm/h lms=%s\n",
                simSeconds, freqGHz, satEirpDbm, rainMmH, lms ? "on" : "off");

    NodeContainer satNodes;
    satNodes.Create(1);
    NodeContainer gndNodes;
    gndNodes.Create(1);
    // (3D scene wiring below, after the ENU origin subLat/subLon is known)

    // Real SGP4 orbit projected into the local ENU frame: the serving Walker
    // element is at zenith at t=0 and recedes with genuine orbital dynamics.
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

    // The gateway is a fixed ground site at the sub-point.
    MobilityHelper mob;
    mob.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    Ptr<ListPositionAllocator> gndPos = CreateObject<ListPositionAllocator>();
    gndPos->Add(Vector(0.0, 0.0, 5.0));
    mob.SetPositionAllocator(gndPos);
    mob.Install(gndNodes);

    NtnRealStackHelper rs;
    rs.SetSimTime(Seconds(simSeconds));
    rs.SetOutputDir(outputDir);
    rs.SetRunTag("ntn-sionna-leo-downlink-traffic");
    rs.SetCarrierFrequencyHz(freqGHz * 1e9);
    rs.SetSatEirpDbm(satEirpDbm);
    rs.Build(satNodes, gndNodes);

    // Channel plug-in: ITU-R atmospheric EXCESS chained AFTER the built-in
    // Friis loss (no FSPL double-count). LMS adds a stateful Markov fade.
    Ptr<NtnAtmosphericLossChain> chain = CreateObject<NtnAtmosphericLossChain>();
    chain->SetFrequencyHz(freqGHz * 1e9);
    chain->SetRainRateMmH(rainMmH);
    chain->SetEnableLms(lms);
    Ptr<NtnAtmosphericPropagationLossModel> atmo =
        CreateObject<NtnAtmosphericPropagationLossModel>();
    atmo->SetChain(chain);
    rs.AddExtraPropagationLoss(atmo);

    rs.InstallTraffic(NtnRealStackHelper::TrafficProfile::EmbbStreaming,
                      Seconds(1.0), Seconds(simSeconds - 0.5));
    rs.EnableAiFlowMonitor("ntn-sionna-leo-downlink-traffic"); // WS2 KPM series (TS 28.552 names)

    std::printf("# %5s  %7s  %9s  %8s  %8s  %8s  %9s\n",
                "t_s", "elev", "slant_km", "atten_dB", "sinr_dB", "tbler", "goodput");

    Ptr<MobilityModel> gndMob = gndNodes.Get(0)->GetObject<MobilityModel>();
    uint64_t lastRx = 0;
    rs.RegisterPeriodicCallback(
        Seconds(1.0),
        [&rs, atmo, gndMob, satEnu, &lastRx](Time now) {
            const Vector g = gndMob->GetPosition();
            const Vector s = satEnu->GetPosition();
            const double elev = ElevDegEnu(g, s);
            const double slantKm = DistM(g, s) / 1000.0;
            const double sinr = rs.GetUeRecentSinrDb(0);
            const double tbler = rs.GetUeRecentTbler(0);
            const uint64_t rx = rs.GetUeRxBytes(0);
            const double mbps = (rx - lastRx) * 8.0 / 1e6;
            lastRx = rx;
            std::printf("  %5.1f  %7.2f  %9.1f  %8.2f  %8.2f  %8.3f  %9.3f\n",
                        now.GetSeconds(), elev, slantKm, atmo->GetLastLossDb(), sinr,
                        tbler, mbps);
        });

    // 3D scene trace in the scenario's local-ENU frame (sat projected about
    // its sub-point; ground node in the same ENU metres).
    ns3::ntnobs::NtnSceneHelper ntnScene;
    if (!netSimOut.empty()) ntnScene.SetNetSimulyzer(netSimOut);
    if (!czmlOut.empty()) ntnScene.SetCzml(czmlOut);
    ntnScene.SetEnuFrame(subLat, subLon, 0.0);
    Ptr<ns3::ntnobs::NtnSceneRecorder> ntnSceneRec = ntnScene.Build(satNodes, gndNodes);

    Simulator::Stop(Seconds(simSeconds));
    Simulator::Run();
    if (ntnSceneRec) ntnSceneRec->Stop();
    rs.Collect();
    rs.WriteHealthReport();

    std::printf("# === summary ===  measured cell SINR=%.2f dB TBLER=%.4f "
                "throughput=%.3f Mbps (ITU-R cascade applied to real packets)\n",
                rs.GetMeanDlSinrDb(), rs.GetMeanDlTbler(), rs.GetRxThroughputMbps());

    Simulator::Destroy();
    return 0;
}
