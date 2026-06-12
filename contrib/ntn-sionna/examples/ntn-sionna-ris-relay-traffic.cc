/*
 * SPDX-License-Identifier: GPL-2.0-only
 * Copyright (c) 2026 Muhammad Uzair (ns3-ntn-toolkit)
 *
 * ntn-sionna-ris-relay-traffic — a LEO downlink whose direct path is blocked
 * (NLOS, e.g. urban canyon / terrain) is recovered by a Reconfigurable
 * Intelligent Surface switched ON mid-simulation, while REAL traffic flows on
 * a REAL mmwave NR NTN cell (NtnRealStackHelper).
 *
 * Audit fix (2026-06 protocol-fidelity audit, channel-plugin recipe):
 * the old version folded the blockage and the RIS array gain into a
 * closed-form SNR and drove a P2P RateErrorModel through a sigmoid
 * SnrToPer() — packets never felt the blockage. Here the blockage onset and
 * the RIS engagement are LIVE channel reconfigurations
 * (NtnStaticExtraLossModel chained onto the real spectrum channel): the
 * MEASURED SINR collapses when the path is blocked and recovers when the RIS
 * engages. The ITU-R atmospheric cascade also stays in the packet path.
 *
 * The RIS gain is the standard perfect-CSI coherent-combining law
 *   G_ris(N) = 20*log10(N_elements)   [dB]
 * from --risRows x --risCols (the same rows/cols descriptor a live Sionna RT
 * server consumes), clamped at the LOS level — a passive reflector cannot
 * beat the unobstructed direct path in this abstraction.
 *
 * Mobility is real: SGP4 satellite (ENU-projected), fixed ground terminal.
 *
 * Quick test:  --simSeconds=40 --risRows=32 --risCols=32
 */
#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/ntn-atmospheric-loss-chain.h"
#include "ns3/ntn-atmospheric-propagation-loss-model.h"
#include "ns3/ntn-real-stack-helper.h"
#include "ns3/ntn-static-extra-loss-model.h"
#include "ns3/ntn-tr38811-mobility-model.h"
#include "ns3/sgp4-mobility-model.h"
#include "ns3/walker-constellation.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("NtnSionnaRisRelayTraffic");

int
main(int argc, char* argv[])
{
    double simSeconds = 40.0;
    double freqGHz = 12.0;
    double satEirpDbm = 75.0;
    double blockageDb = 30.0;
    uint32_t risRows = 32;
    uint32_t risCols = 32;
    double blockFraction = 0.25;
    double risOnFraction = 0.55;
    std::string outputDir = "ntn-sionna-ris-relay-output";

    CommandLine cmd(__FILE__);
    cmd.AddValue("simSeconds", "Simulation duration (s)", simSeconds);
    cmd.AddValue("freqGHz", "Carrier frequency (GHz)", freqGHz);
    cmd.AddValue("satEirpDbm", "Satellite EIRP / gNB Tx power (dBm)", satEirpDbm);
    cmd.AddValue("blockageDb", "NLOS blockage on the direct path (dB)", blockageDb);
    cmd.AddValue("risRows", "RIS element rows", risRows);
    cmd.AddValue("risCols", "RIS element cols", risCols);
    cmd.AddValue("blockFraction", "Fraction of sim at which the blockage starts",
                 blockFraction);
    cmd.AddValue("risOnFraction", "Fraction of sim at which the RIS engages",
                 risOnFraction);
    cmd.AddValue("outputDir", "Output directory", outputDir);
    cmd.Parse(argc, argv);

    const double risGainDb = 20.0 * std::log10(std::max(1u, risRows * risCols));
    const double risCompDb = std::min(blockageDb, risGainDb);

    std::printf("# ntn-sionna-ris-relay-traffic (REAL radio, blockage + RIS in the "
                "packet path)\n");
    std::printf("#   sim=%.0fs freq=%.1fGHz EIRP=%.1fdBm blockage=%.0fdB "
                "RIS=%ux%u->%.1fdB gain (%.1f dB applied)\n",
                simSeconds, freqGHz, satEirpDbm, blockageDb, risRows, risCols,
                risGainDb, risCompDb);

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
    rs.SetSimTime(Seconds(simSeconds));
    rs.SetOutputDir(outputDir);
    rs.SetRunTag("ntn-sionna-ris-relay-traffic");
    rs.SetCarrierFrequencyHz(freqGHz * 1e9);
    rs.SetSatEirpDbm(satEirpDbm);
    rs.Build(satNodes, gndNodes);

    // ITU-R atmospheric excess stays in the packet path the whole run.
    Ptr<NtnAtmosphericLossChain> chain = CreateObject<NtnAtmosphericLossChain>();
    chain->SetFrequencyHz(freqGHz * 1e9);
    Ptr<NtnAtmosphericPropagationLossModel> atmo =
        CreateObject<NtnAtmosphericPropagationLossModel>();
    atmo->SetChain(chain);
    rs.AddExtraPropagationLoss(atmo);

    // Blockage / RIS as a LIVE channel reconfiguration in the real path.
    Ptr<NtnStaticExtraLossModel> nlos = CreateObject<NtnStaticExtraLossModel>();
    nlos->SetLossDb(0.0);
    rs.AddExtraPropagationLoss(nlos);

    rs.InstallTraffic(NtnRealStackHelper::TrafficProfile::EmbbStreaming,
                      Seconds(1.0), Seconds(simSeconds - 0.5));
    rs.EnableAiFlowMonitor("ntn-sionna-ris-relay-traffic"); // WS2 KPM series (TS 28.552 names)

    const double tBlock = blockFraction * simSeconds;
    const double tRis = risOnFraction * simSeconds;
    bool risOn = false;
    Simulator::Schedule(Seconds(tBlock), [nlos, blockageDb] {
        nlos->SetLossDb(blockageDb);
    });
    Simulator::Schedule(Seconds(tRis), [nlos, blockageDb, risCompDb, &risOn] {
        risOn = true;
        nlos->SetLossDb(blockageDb - risCompDb);
    });
    std::printf("#   timeline: LOS -> blocked@%.0fs -> RIS ON@%.0fs\n", tBlock, tRis);
    std::printf("# %5s  %5s  %8s  %8s  %8s  %9s\n",
                "t_s", "ris", "extra_dB", "sinr_dB", "tbler", "goodput");

    uint64_t lastRx = 0;
    rs.RegisterPeriodicCallback(
        Seconds(1.0),
        [&rs, nlos, &risOn, &lastRx](Time now) {
            const double sinr = rs.GetUeRecentSinrDb(0);
            const double tbler = rs.GetUeRecentTbler(0);
            const uint64_t rx = rs.GetUeRxBytes(0);
            const double mbps = (rx - lastRx) * 8.0 / 1e6;
            lastRx = rx;
            std::printf("  %5.1f  %5s  %8.2f  %8.2f  %8.3f  %9.3f\n",
                        now.GetSeconds(), risOn ? "ON" : "off", nlos->GetLossDb(),
                        sinr, tbler, mbps);
        });

    Simulator::Stop(Seconds(simSeconds));
    Simulator::Run();
    rs.Collect();
    rs.WriteHealthReport();

    std::printf("# === summary ===  measured cell SINR=%.2f dB TBLER=%.4f "
                "throughput=%.3f Mbps (blockage + RIS applied to real packets)\n",
                rs.GetMeanDlSinrDb(), rs.GetMeanDlTbler(), rs.GetRxThroughputMbps());

    Simulator::Destroy();
    return 0;
}
