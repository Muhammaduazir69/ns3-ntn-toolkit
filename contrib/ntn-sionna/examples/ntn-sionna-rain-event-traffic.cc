/*
 * SPDX-License-Identifier: GPL-2.0-only
 * Copyright (c) 2026 Muhammad Uzair (ns3-ntn-toolkit)
 *
 * ntn-sionna-rain-event-traffic — a convective rain cell sweeps over the
 * gateway mid-pass, exercising the NtnAtmosphericLossChain dynamically while
 * REAL traffic flows on a REAL mmwave NR NTN cell (NtnRealStackHelper:
 * SpectrumPhy + MAC + HARQ + RLC/PDCP + RRC + EPC).
 *
 * Audit fix (2026-06 protocol-fidelity audit, channel-plugin recipe):
 * the old version queried the cascade in a probe loop and drove a P2P
 * RateErrorModel through a sigmoid SnrToPer() — packets never saw the rain.
 * Here the SAME ITU-R physics (P.676 gaseous + P.618/P.838 rain), re-homed as
 * NtnAtmosphericPropagationLossModel (pure atmospheric EXCESS, no FSPL
 * double-count), is chained onto the real spectrum channel via
 * AddExtraPropagationLoss(). The rain-rate schedule reconfigures the LIVE
 * chain, so Ka-band attenuation rises and falls in the MEASURED SINR / TBLER
 * and the delivered goodput dips during the cell and recovers — driven
 * entirely by the channel, nothing closed-form in the packet path.
 *
 * Mobility is real: the satellite is an SGP4 Walker element projected into
 * the scenario's local ENU frame; the gateway is a fixed Ka-band ground site.
 *
 * Quick test:  --simSeconds=40 --peakRainMmH=50
 */
#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/ntn-atmospheric-loss-chain.h"
#include "ns3/ntn-atmospheric-propagation-loss-model.h"
#include "ns3/ntn-real-stack-helper.h"
#include "ns3/ntn-tr38811-mobility-model.h"
#include "ns3/sgp4-mobility-model.h"
#include "ns3/walker-constellation.h"

#include <cmath>
#include <cstdio>
#include <string>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("NtnSionnaRainEventTraffic");

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

} // namespace

int
main(int argc, char* argv[])
{
    double simSeconds = 40.0;
    double freqGHz = 20.0;     // Ka-band — rain-sensitive
    double satEirpDbm = 95.0;  // Ka feeder beam (closes ~173 dB FSPL with margin)
    double peakRainMmH = 50.0;
    std::string radio = "nr"; // radio spine: "nr" (5G-LENA FR1) | "mmwave" (FR2)
    std::string outputDir = "ntn-sionna-rain-event-output";

    CommandLine cmd(__FILE__);
    cmd.AddValue("simSeconds", "Simulation duration (s)", simSeconds);
    cmd.AddValue("freqGHz", "Carrier frequency (GHz)", freqGHz);
    cmd.AddValue("satEirpDbm", "Satellite EIRP / gNB Tx power (dBm)", satEirpDbm);
    cmd.AddValue("radio", "Radio backend: nr (FR1) or mmwave", radio);
    cmd.AddValue("peakRainMmH", "Peak rain rate of the cell (mm/h)", peakRainMmH);
    cmd.AddValue("outputDir", "Output directory", outputDir);
    cmd.Parse(argc, argv);

    std::printf("# ntn-sionna-rain-event-traffic (REAL radio, ITU-R rain in the packet path)\n");
    std::printf("#   sim=%.0fs freq=%.1fGHz EIRP=%.1fdBm peakRain=%.0fmm/h\n",
                simSeconds, freqGHz, satEirpDbm, peakRainMmH);

    NodeContainer satNodes;
    satNodes.Create(1);
    NodeContainer gndNodes;
    gndNodes.Create(1);

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

    // The gateway is a fixed Ka-band ground site at the sub-point.
    MobilityHelper mob;
    mob.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    Ptr<ListPositionAllocator> gndPos = CreateObject<ListPositionAllocator>();
    gndPos->Add(Vector(0.0, 0.0, 5.0));
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
    rs.SetRunTag("ntn-sionna-rain-event-traffic");
    rs.SetCarrierFrequencyHz(freqGHz * 1e9);
    rs.SetSatEirpDbm(satEirpDbm);
    rs.Build(satNodes, gndNodes);

    // Channel plug-in: ITU-R P.676 gaseous + P.618/P.838 rain as pure
    // atmospheric EXCESS chained AFTER the built-in Friis loss (no FSPL
    // double-count). The rain schedule reconfigures this LIVE chain.
    Ptr<NtnAtmosphericLossChain> chain = CreateObject<NtnAtmosphericLossChain>();
    chain->SetFrequencyHz(freqGHz * 1e9);
    chain->SetRainRateMmH(0.0);
    Ptr<NtnAtmosphericPropagationLossModel> atmo =
        CreateObject<NtnAtmosphericPropagationLossModel>();
    atmo->SetChain(chain);
    rs.AddExtraPropagationLoss(atmo);

    rs.InstallTraffic(NtnRealStackHelper::TrafficProfile::EmbbStreaming,
                      Seconds(1.0), Seconds(simSeconds - 0.5));
    rs.EnableAiFlowMonitor("ntn-sionna-rain-event-traffic"); // WS2 KPM series (TS 28.552 names)

    // Rain cell schedule: clear → building → peak → clearing → clear,
    // scaled to simSeconds. Each step reconfigures the live channel plug-in.
    const double tBuild = 0.30 * simSeconds;
    const double tPeak = 0.40 * simSeconds;
    const double tEase = 0.55 * simSeconds;
    const double tClear = 0.65 * simSeconds;
    Simulator::Schedule(Seconds(tBuild),
                        [chain, peakRainMmH] { chain->SetRainRateMmH(0.25 * peakRainMmH); });
    Simulator::Schedule(Seconds(tPeak),
                        [chain, peakRainMmH] { chain->SetRainRateMmH(peakRainMmH); });
    Simulator::Schedule(Seconds(tEase),
                        [chain, peakRainMmH] { chain->SetRainRateMmH(0.50 * peakRainMmH); });
    Simulator::Schedule(Seconds(tClear), [chain] { chain->SetRainRateMmH(0.0); });
    std::printf("#   rain cell: clear→25%%@%.0fs →peak@%.0fs →50%%@%.0fs →clear@%.0fs\n",
                tBuild, tPeak, tEase, tClear);
    std::printf("# %5s  %7s  %7s  %8s  %8s  %8s  %9s\n",
                "t_s", "elev", "rain", "atten_dB", "sinr_dB", "tbler", "goodput");

    // 1 Hz probe: MEASURED SINR/TBLER off the PHY trace, goodput off the UE
    // PacketSink, atmospheric loss off the live plug-in.
    Ptr<MobilityModel> gndMob = gndNodes.Get(0)->GetObject<MobilityModel>();
    uint64_t lastRx = 0;
    double clearSumSinr = 0.0, peakSumSinr = 0.0;
    uint64_t clearN = 0, peakN = 0;
    rs.RegisterPeriodicCallback(
        Seconds(1.0),
        [&rs, atmo, chain, gndMob, satEnu, &lastRx, &clearSumSinr, &peakSumSinr, &clearN,
         &peakN, peakRainMmH](Time now) {
            const double elev = ElevDegEnu(gndMob->GetPosition(), satEnu->GetPosition());
            const double sinr = rs.GetUeRecentSinrDb(0);
            const double tbler = rs.GetUeRecentTbler(0);
            const uint64_t rx = rs.GetUeRxBytes(0);
            const double mbps = (rx - lastRx) * 8.0 / 1e6;
            lastRx = rx;
            if (!std::isnan(sinr))
            {
                if (chain->GetRainRateMmH() == 0.0)
                {
                    clearSumSinr += sinr;
                    clearN++;
                }
                else if (chain->GetRainRateMmH() == peakRainMmH)
                {
                    peakSumSinr += sinr;
                    peakN++;
                }
            }
            std::printf("  %5.1f  %7.2f  %7.2f  %8.2f  %8.2f  %8.3f  %9.3f\n",
                        now.GetSeconds(), elev, chain->GetRainRateMmH(),
                        atmo->GetLastLossDb(), sinr, tbler, mbps);
        });

    Simulator::Stop(Seconds(simSeconds));
    Simulator::Run();
    rs.Collect();
    rs.WriteHealthReport();

    const double clearSinr = clearN ? clearSumSinr / clearN : std::nan("");
    const double peakSinr = peakN ? peakSumSinr / peakN : std::nan("");
    std::printf("# === summary ===  MEASURED clear-sky SINR=%.2f dB, peak-rain SINR=%.2f dB "
                "(rain fade=%.2f dB), cell SINR=%.2f dB TBLER=%.4f throughput=%.3f Mbps\n",
                clearSinr, peakSinr, clearSinr - peakSinr, rs.GetMeanDlSinrDb(),
                rs.GetMeanDlTbler(), rs.GetRxThroughputMbps());

    Simulator::Destroy();
    return 0;
}
