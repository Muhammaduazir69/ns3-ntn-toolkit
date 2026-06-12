/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only
//
// thz-ntn-ris-relay-traffic — a sub-THz LEO downlink whose direct path is
// blocked (THz signals are easily obstructed) is recovered by a ThzNtnRis
// surface switched ON mid-simulation, while REAL traffic flows on a REAL
// mmwave NR NTN cell (NtnRealStackHelper).
//
// Audit fix (2026-06 protocol-fidelity audit, channel-plugin recipe):
// the old version folded blockage and RIS gain into a closed-form SNR and
// drove a P2P RateErrorModel through a sigmoid SnrToPer() — packets never
// felt the blockage. Here the blockage onset and the RIS engagement are LIVE
// channel reconfigurations (NtnStaticExtraLossModel chained onto the real
// spectrum channel): the MEASURED SINR collapses when the canyon blocks the
// path and recovers when the RIS engages. The recovery magnitude is the
// module's own physics — ThzNtnRis::ComputeSnrGain_dB(true) from the
// configured NumElementsX/Y — clamped at the LOS level (a passive reflector
// cannot beat the unobstructed direct path in this abstraction).
//
// Molecular absorption (ThzNtnPropagationLossModel) also stays in the packet
// path the whole time. Mobility is real: SGP4 satellite, fixed ground site.
//
// Quick test:  --simSeconds=40 --risX=40 --risY=40
#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/ntn-real-stack-helper.h"
#include "ns3/ntn-static-extra-loss-model.h"
#include "ns3/ntn-tr38811-mobility-model.h"
#include "ns3/sgp4-mobility-model.h"
#include "ns3/thz-ntn-propagation-loss-model.h"
#include "ns3/thz-ntn-ris.h"
#include "ns3/walker-constellation.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("ThzNtnRisRelayTraffic");

int
main(int argc, char* argv[])
{
    double simSeconds = 40.0;
    double freqGHz = 100.0;    // sub-THz (3GPP spectrum model upper bound)
    double satEirpDbm = 115.0;
    double blockageDb = 40.0;
    uint32_t risX = 40;
    uint32_t risY = 40;
    double reflEff = 0.9;
    double blockFraction = 0.25; // canyon blocks the direct path here
    double risOnFraction = 0.55; // RIS engages here
    std::string outputDir = "thz-ntn-ris-relay-output";

    CommandLine cmd(__FILE__);
    cmd.AddValue("simSeconds", "Simulation duration (s)", simSeconds);
    cmd.AddValue("freqGHz", "Carrier frequency (GHz), capped at 100", freqGHz);
    cmd.AddValue("satEirpDbm", "Satellite EIRP / gNB Tx power (dBm)", satEirpDbm);
    cmd.AddValue("blockageDb", "NLOS blockage on the direct path (dB)", blockageDb);
    cmd.AddValue("risX", "RIS elements along X", risX);
    cmd.AddValue("risY", "RIS elements along Y", risY);
    cmd.AddValue("reflEff", "RIS reflection efficiency (0-1)", reflEff);
    cmd.AddValue("blockFraction", "Fraction of sim at which the blockage starts",
                 blockFraction);
    cmd.AddValue("risOnFraction", "Fraction of sim at which the RIS engages",
                 risOnFraction);
    cmd.AddValue("outputDir", "Output directory", outputDir);
    cmd.Parse(argc, argv);

    if (freqGHz > 100.0)
    {
        std::printf("# NOTE: 3GPP spectrum model caps the carrier at 100 GHz; "
                    "clamping %.0f -> 100 GHz\n",
                    freqGHz);
        freqGHz = 100.0;
    }

    // The RIS recovery magnitude is the module's own array physics.
    Ptr<ThzNtnRis> ris = CreateObject<ThzNtnRis>();
    ris->SetAttribute("NumElementsX", UintegerValue(risX));
    ris->SetAttribute("NumElementsY", UintegerValue(risY));
    ris->SetAttribute("Frequency", DoubleValue(freqGHz * 1e9));
    ris->SetAttribute("ReflectionEfficiency", DoubleValue(reflEff));
    const double risGainDb = ris->ComputeSnrGain_dB(true);
    // A passive reflector restores at most the LOS budget here.
    const double risCompDb = std::min(blockageDb, risGainDb);

    std::printf("# thz-ntn-ris-relay-traffic (REAL radio, blockage + RIS in the packet path)\n");
    std::printf("#   sim=%.0fs freq=%.0fGHz EIRP=%.1fdBm blockage=%.0fdB "
                "RIS=%ux%u(eff %.2f)->%.1fdB gain (%.1f dB applied)\n",
                simSeconds, freqGHz, satEirpDbm, blockageDb, risX, risY, reflEff,
                risGainDb, risCompDb);

    NodeContainer satNodes;
    satNodes.Create(1);
    NodeContainer gndNodes;
    gndNodes.Create(1);

    // Real SGP4 orbit projected into the local ENU frame.
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

    // Fixed ground terminal in the urban canyon.
    MobilityHelper mob;
    mob.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    Ptr<ListPositionAllocator> gndPos = CreateObject<ListPositionAllocator>();
    gndPos->Add(Vector(0.0, 0.0, 1.5));
    mob.SetPositionAllocator(gndPos);
    mob.Install(gndNodes);

    NtnRealStackHelper rs;
    rs.SetSimTime(Seconds(simSeconds));
    rs.SetOutputDir(outputDir);
    rs.SetRunTag("thz-ntn-ris-relay-traffic");
    rs.SetCarrierFrequencyHz(freqGHz * 1e9);
    rs.SetSatEirpDbm(satEirpDbm);
    rs.Build(satNodes, gndNodes);

    // THz molecular absorption stays in the packet path the whole run.
    Ptr<ThzNtnPropagationLossModel> thz = CreateObject<ThzNtnPropagationLossModel>();
    thz->SetFrequency(freqGHz * 1e9);
    rs.AddExtraPropagationLoss(thz);

    // Blockage / RIS as a LIVE channel reconfiguration in the real path.
    Ptr<NtnStaticExtraLossModel> nlos = CreateObject<NtnStaticExtraLossModel>();
    nlos->SetLossDb(0.0); // LOS until the canyon blocks the path
    rs.AddExtraPropagationLoss(nlos);

    rs.InstallTraffic(NtnRealStackHelper::TrafficProfile::EmbbStreaming,
                      Seconds(1.0), Seconds(simSeconds - 0.5));
    rs.EnableAiFlowMonitor("thz-ntn-ris-relay-traffic");

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
                "throughput=%.3f Mbps (blockage + RIS applied to real packets, "
                "RIS gain %.1f dB from ThzNtnRis physics)\n",
                rs.GetMeanDlSinrDb(), rs.GetMeanDlTbler(), rs.GetRxThroughputMbps(),
                risGainDb);

    Simulator::Destroy();
    return 0;
}
