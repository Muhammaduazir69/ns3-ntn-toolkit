/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026  Muhammad Uzair
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * sagin-a2g-real-stack — Phase 2 of 2026-06 protocol-fidelity audit
 * (channel-plugin recipe).
 *
 * Audit finding for ntn-sagin: A2gChannelTr36777::PathLossDb() (3GPP TR 36.777
 * air-to-ground, with the LOS/NLOS branches) was only ever called from
 * user-space loops and written to CSV — no packet was ever attenuated by it, so
 * the air-layer KPIs were closed-form arithmetic.
 *
 * Here the TR 36.777 model is re-homed as a real ns-3 PropagationLossModel
 * (SaginA2gPropagationLossModel) and chained onto a REAL mmwave NR air interface
 * (NtnRealStackHelper) via AddExtraPropagationLoss(). A UAV/HAPS aerial platform
 * is the gNB; ground nodes are UEs. The TR 36.777 path loss now actually
 * attenuates packets, so the air-to-ground SINR is MEASURED off the mmwave PHY
 * trace — and the LOS-vs-NLOS shadowing branch is visible as a real SINR /
 * throughput gap, exactly like a measured A2G link.
 *
 * Usage:
 *   ./ns3 run "sagin-a2g-real-stack --duration=12 --link=NLOS --scenario=UMa_AV"
 */

#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/ntn-real-stack-helper.h"
#include "ns3/sagin-a2g-propagation-loss-model.h"

#include <iostream>
#include <string>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("SaginA2gRealStack");

namespace
{
A2gScenario
ParseScenario(const std::string& s)
{
    if (s == "RMa_AV")
    {
        return A2gScenario::RMa_AV;
    }
    if (s == "UMi_AV")
    {
        return A2gScenario::UMi_AV;
    }
    return A2gScenario::UMa_AV;
}
} // namespace

int
main(int argc, char* argv[])
{
    double duration = 12.0;
    uint32_t numUes = 4;
    double uavAltM = 100.0;
    double gnbTxDbm = -7.0; // aerial-platform gNB Tx power; tuned so the short A2G
                            // range lands in a realistic SINR band (error model active)
    double freqGhz = 2.0;
    std::string linkStr = "NLOS";
    std::string scenarioStr = "UMa_AV";
    std::string outputDir = "sagin-a2g-real-stack-output";
    std::string radio = "nr"; // radio backend: "nr" (5G-LENA FR1) | "mmwave" (FR2)

    CommandLine cmd(__FILE__);
    cmd.AddValue("duration", "Simulation duration (s)", duration);
    cmd.AddValue("numUes", "Number of ground UEs", numUes);
    cmd.AddValue("uavAlt", "UAV/HAPS aerial gNB altitude AGL (m)", uavAltM);
    cmd.AddValue("gnbTxDbm", "Aerial gNB Tx power (dBm)", gnbTxDbm);
    cmd.AddValue("freqGhz", "Carrier frequency (GHz)", freqGhz);
    cmd.AddValue("link", "TR 36.777 link branch: LOS | NLOS", linkStr);
    cmd.AddValue("scenario", "TR 36.777 scenario: UMa_AV | RMa_AV | UMi_AV", scenarioStr);
    cmd.AddValue("outputDir", "Output directory", outputDir);
    cmd.AddValue("radio", "Radio backend: nr (FR1) or mmwave", radio);
    cmd.Parse(argc, argv);

    const A2gLink link = (linkStr == "LOS") ? A2gLink::LOS : A2gLink::NLOS;
    const A2gScenario scenario = ParseScenario(scenarioStr);

    std::cout << "\n=== sagin-a2g REAL-STACK (TR 36.777 A2G as a real channel plug-in) ===\n"
              << "  aerial gNB at " << uavAltM << " m AGL, " << numUes << " ground UEs\n"
              << "  channel: TR 36.777 " << scenarioStr << " / " << linkStr
              << " chained on a real mmwave NR link\n"
              << "  air-to-ground SINR is MEASURED off the PHY (not A2gChannelTr36777 CSV)\n"
              << "  duration: " << duration << " s\n\n";

    // ---- Aerial platform = gNB; ground nodes = UEs ----
    NodeContainer gnbNodes;
    gnbNodes.Create(1);
    NodeContainer ueNodes;
    ueNodes.Create(numUes);

    MobilityHelper mob;
    mob.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    Ptr<ListPositionAllocator> gnbPos = CreateObject<ListPositionAllocator>();
    gnbPos->Add(Vector(0.0, 0.0, uavAltM));
    mob.SetPositionAllocator(gnbPos);
    mob.Install(gnbNodes);
    Ptr<ListPositionAllocator> uePos = CreateObject<ListPositionAllocator>();
    for (uint32_t i = 0; i < numUes; ++i)
    {
        uePos->Add(Vector(150.0 + 120.0 * i, 0.0, 1.5)); // ground UTs, 150-630 m out
    }
    mob.SetPositionAllocator(uePos);
    mob.Install(ueNodes);

    // ---- Real mmwave NR air interface ----
    NtnRealStackHelper rs;
    rs.SetSimTime(Seconds(duration));
    rs.SetOutputDir(outputDir);
    rs.SetRunTag("sagin-a2g-real-stack");
    rs.SetCarrierFrequencyHz(freqGhz * 1e9);
    rs.SetSatEirpDbm(gnbTxDbm); // short-range A2G gNB Tx (not a LEO link) — kept as-is
    rs.SetRadioBackend(radio == "mmwave" ? NtnRealStackHelper::RadioBackend::Mmwave
                                         : NtnRealStackHelper::RadioBackend::Nr);
    if (radio != "mmwave")
    {
        rs.SetNumerology(1); // FR1 30 kHz SCS
    }
    rs.Build(gnbNodes, ueNodes);

    // ---- Channel plug-in: TR 36.777 A2G loss attenuates real packets ----
    Ptr<SaginA2gPropagationLossModel> a2g = CreateObject<SaginA2gPropagationLossModel>();
    a2g->SetFrequencyGHz(freqGhz);
    a2g->SetScenario(scenario);
    a2g->SetLink(link);
    a2g->SetUavAltitudeM(uavAltM);
    rs.AddExtraPropagationLoss(a2g);

    rs.InstallTraffic(NtnRealStackHelper::TrafficProfile::EmbbStreaming,
                      Seconds(1.0), Seconds(duration - 0.5));
    rs.EnableAiFlowMonitor("sagin-a2g-real-stack"); // WS2 KPM series (TS 28.552 names)

    Simulator::Stop(Seconds(duration));
    Simulator::Run();
    rs.Collect();
    rs.WriteHealthReport();

    std::cout << "\n--- SAGIN A2G Summary (TR 36.777 on MEASURED radio) ---\n"
              << "  TR 36.777 branch:             " << scenarioStr << " / " << linkStr << "\n"
              << "  last total A2G path loss:     " << a2g->GetLastLossDb() << " dB\n"
              << "  last excess over free space:  " << a2g->GetLastExcessDb()
              << " dB (charged on the chained channel)\n"
              << "  MEASURED air-to-ground SINR:  " << rs.GetMeanDlSinrDb() << " dB\n"
              << "  measured DL TBLER (mean):     " << rs.GetMeanDlTbler() << "\n"
              << "  measured DL throughput:       " << rs.GetRxThroughputMbps() << " Mbps\n"
              << "  -> TR 36.777 A2G loss attenuates real packets; SINR is measured.\n";

    Simulator::Destroy();
    return 0;
}
