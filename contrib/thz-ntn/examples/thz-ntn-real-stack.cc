/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026  Muhammad Uzair
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * thz-ntn-real-stack — Phase 1 of 2026-06 protocol-fidelity audit
 * (channel-plugin pattern).
 *
 * The audit found every thz-ntn example was a CSV calculator (or an empty Run()):
 * the molecular-absorption / weather physics were never applied to a packet. Here
 * those physics are re-homed as a real ns-3 PropagationLossModel
 * (ThzNtnPropagationLossModel) and chained onto the REAL mmwave NTN spectrum
 * channel via NtnRealStackHelper::AddExtraPropagationLoss(). The THz atmospheric
 * loss now actually attenuates the transmitted packets, so it shows up in the
 * MEASURED SINR — and toggling rain mid-run visibly degrades the measured link.
 *
 * Usage:
 *   ./ns3 run "thz-ntn-real-stack --duration=16 --freqGhz=100 --rainMmH=25"
 */

#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/ntn-real-stack-helper.h"
#include "ns3/ntn-tr38811-mobility-model.h"
#include "ns3/sgp4-mobility-model.h"
#include "ns3/thz-ntn-propagation-loss-model.h"
#include "ns3/walker-constellation.h"

#include <iostream>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("ThzNtnRealStack");

int
main(int argc, char* argv[])
{
    double duration = 16.0;
    uint32_t numUes = 4;
    double altitudeKm = 550.0;
    double freqGhz = 100.0;      // sub-THz / W-band (within the stack's range)
    double satEirpDbm = 92.0;    // high-gain THz beam EIRP (closes the long slant)
    double rainMmH = 4.0;        // light rain: degrades but keeps the link alive
    std::string outputDir = "thz-ntn-real-stack-output";

    CommandLine cmd(__FILE__);
    cmd.AddValue("duration", "Simulation duration (s)", duration);
    cmd.AddValue("numUes", "Number of ground UEs", numUes);
    cmd.AddValue("altitude", "Satellite altitude (km)", altitudeKm);
    cmd.AddValue("freqGhz", "Carrier frequency (GHz)", freqGhz);
    cmd.AddValue("satEirpDbm", "Satellite EIRP / gNB Tx power (dBm)", satEirpDbm);
    cmd.AddValue("rainMmH", "Rain rate applied in the 2nd half (mm/h)", rainMmH);
    cmd.AddValue("outputDir", "Output directory", outputDir);
    cmd.Parse(argc, argv);

    std::cout << "\n=== thz-ntn REAL-STACK (THz channel plug-in on a real radio) ===\n"
              << "  carrier: " << freqGhz << " GHz, " << numUes << " UEs\n"
              << "  THz physics: ThzNtnPropagationLossModel (gaseous absorption + rain)\n"
              << "               chained onto the real mmwave channel -> attenuates packets\n"
              << "  duration: " << duration << " s\n\n";

    NodeContainer satNodes;
    satNodes.Create(1);
    NodeContainer ueNodes;
    ueNodes.Create(numUes);

    // Real SGP4 orbit projected into the scenario's local ENU frame: the
    // serving Walker element is at zenith at t=0 (min atmospheric path) and
    // recedes with genuine orbital dynamics (no fixed-overhead placeholder).
    ns3::ntncon::WalkerConfig wcfgSat;
    wcfgSat.num_planes = 1;
    wcfgSat.total_sats = 80;
    wcfgSat.altitude_km = altitudeKm;
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

    MobilityHelper mob;
    mob.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    Ptr<ListPositionAllocator> uePos = CreateObject<ListPositionAllocator>();
    for (uint32_t i = 0; i < numUes; ++i)
    {
        uePos->Add(Vector(800.0 * i, 0.0, 0.0));
    }
    mob.SetPositionAllocator(uePos);
    mob.Install(ueNodes);

    NtnRealStackHelper rs;
    rs.SetSimTime(Seconds(duration));
    rs.SetOutputDir(outputDir);
    rs.SetRunTag("thz-ntn-real-stack");
    rs.SetCarrierFrequencyHz(freqGhz * 1e9);
    rs.SetSatEirpDbm(satEirpDbm);
    rs.Build(satNodes, ueNodes);

    // Re-home the THz physics as a real channel plug-in on the mmwave channel.
    Ptr<ThzNtnPropagationLossModel> thz = CreateObject<ThzNtnPropagationLossModel>();
    thz->SetFrequency(freqGhz * 1e9);
    thz->SetRainRate(0.0); // clear sky for the first half
    rs.AddExtraPropagationLoss(thz);

    rs.InstallTraffic(NtnRealStackHelper::TrafficProfile::EmbbStreaming,
                      Seconds(1.0), Seconds(duration - 0.5));
    rs.EnableAiFlowMonitor("thz-ntn-real-stack");

    // Rain event over the traffic window -> measured SINR drops (THz physics in
    // the packet path). Run with --rainMmH=0 vs a high value to see the delta.
    Simulator::Schedule(Seconds(1.0), [thz, rainMmH] { thz->SetRainRate(rainMmH); });

    Simulator::Stop(Seconds(duration));
    Simulator::Run();
    rs.Collect();
    rs.WriteHealthReport();

    // Query the plug-in on the actual sat->UE pair (the cached "last loss" can
    // be a ground-to-ground UE pair query on a shared multi-UE channel).
    const double satUeLossDb =
        -thz->CalcRxPower(0.0, satEnu, ueNodes.Get(0)->GetObject<MobilityModel>());
    std::cout << "\n--- THz Summary (physics applied to real packets) ---\n"
              << "  measured DL SINR (mean):  " << rs.GetMeanDlSinrDb() << " dB\n"
              << "  measured TBLER:           " << rs.GetMeanDlTbler() << "\n"
              << "  measured throughput:      " << rs.GetRxThroughputMbps() << " Mbps\n"
              << "  THz atmospheric loss on the sat->UE path: " << satUeLossDb << " dB"
              << " (rain " << rainMmH << " mm/h)\n";

    Simulator::Destroy();
    return 0;
}
