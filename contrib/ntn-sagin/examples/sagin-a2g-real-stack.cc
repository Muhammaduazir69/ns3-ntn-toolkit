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
 * Wave 7B, one flow through the toolkit (space-air-ground):
 *  - the ground UEs are TR 38.811 UE-class terminals (ntn-cho
 *    NtnTr38811MobilityModel, urban-dense class mix: static and pedestrian
 *    handhelds, vehicles, IoT) drawn in a box around the platform's ground
 *    point and projected into the cell's local ENU frame
 *    (NtnEnuProjectionMobilityModel). --ueLayout=line restores the earlier fixed
 *    line of UEs at 150-630 m;
 *  - the aerial gNB reaches its core over a bent-pipe LEO hop of a Walker shell
 *    on Vallado SGP4 (ntn-constellation, sagin-leo-backhaul.h): live delay from
 *    the exact SGP4 state every 100 ms, switches, outages, Ka-band loss from
 *    two NtnSatLinkErrorModel legs. --leoBackhaul=0 restores the old backhaul;
 *  - ntn-traffic eMBB traffic with the WS2 KPM series (TS 28.552 names).
 *
 * Usage:
 *   ./ns3 run "sagin-a2g-real-stack --duration=12 --link=NLOS --scenario=UMa_AV"
 */

#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/ntn-real-stack-helper.h"
#include "ns3/sagin-a2g-propagation-loss-model.h"
#include "ns3/ntn-tr38811-mobility-model.h"
#include "ns3/system-path.h"

#include "sagin-leo-backhaul.h"

#include <cstdio>
#include <iostream>
#include <memory>
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
    double freqGhz = 2.185; // TS 38.101-5 n256 downlink block (2170-2200 MHz). Was 2.0,
                              // which is n256 UPLINK and never conformant as a DL carrier.
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
    std::string ueLayout = "tr38811";
    bool leoBackhaul = true;
    SaginLeoBackhaulConfig bh;
    bh.siteLatDeg = 39.74; // urban platform ground point (UMa_AV default)
    bh.siteLonDeg = -104.99;
    bh.gwLatDeg = 40.58; // gateway about 95 km away
    bh.gwLonDeg = -105.08;
    double bhMbps = 100.0;
    cmd.AddValue("ueLayout", "ground UEs: tr38811 (TR 38.811 class mix, default) | line (fixed line)",
                 ueLayout);
    cmd.AddValue("leoBackhaul", "aerial gNB backhaul over a bent-pipe LEO hop (SGP4 Walker shell)",
                 leoBackhaul);
    cmd.AddValue("siteLat", "platform ground-point latitude (deg)", bh.siteLatDeg);
    cmd.AddValue("siteLon", "platform ground-point longitude (deg)", bh.siteLonDeg);
    cmd.AddValue("gwLat", "LEO gateway latitude (deg)", bh.gwLatDeg);
    cmd.AddValue("gwLon", "LEO gateway longitude (deg)", bh.gwLonDeg);
    cmd.AddValue("bhMaskDeg", "backhaul elevation mask at site and gateway (deg)", bh.siteMaskDeg);
    cmd.AddValue("bhMbps", "backhaul rate (Mb/s), sets the Ka noise bandwidth", bhMbps);
    cmd.Parse(argc, argv);
    NS_ABORT_MSG_UNLESS(ueLayout == "tr38811" || ueLayout == "line", "--ueLayout=tr38811|line");
    bh.gwMaskDeg = bh.siteMaskDeg;
    bh.rateBps = bhMbps * 1e6;
    bh.siteAltM = uavAltM; // the backhaul terminal rides on the platform

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
    if (ueLayout == "line")
    {
        Ptr<ListPositionAllocator> uePos = CreateObject<ListPositionAllocator>();
        for (uint32_t i = 0; i < numUes; ++i)
        {
            uePos->Add(Vector(150.0 + 120.0 * i, 0.0, 1.5)); // ground UTs, 150-630 m out
        }
        mob.SetPositionAllocator(uePos);
        mob.Install(ueNodes);
    }
    else
    {
        // TR 38.811 class terminals within ~450 m of the platform's ground point (the range
        // the gNB power was set for), in the cell's ENU frame (UE height 1.5 m).
        auto engine = std::make_shared<NtnRealisticMobilityHelper>(11);
        const double dLat = 0.004, dLon = 0.004 / std::cos(bh.siteLatDeg * M_PI / 180.0);
        auto ues = engine->GenerateUes(numUes, NtnMobilityScenarios::UrbanDense(),
                                       bh.siteLatDeg - dLat, bh.siteLatDeg + dLat,
                                       bh.siteLonDeg - dLon, bh.siteLonDeg + dLon);
        for (uint32_t j = 0; j < numUes; ++j)
        {
            Ptr<NtnTr38811MobilityModel> src = CreateObject<NtnTr38811MobilityModel>();
            src->SetUeState(ues[j], engine);
            Ptr<NtnEnuProjectionMobilityModel> enu = CreateObject<NtnEnuProjectionMobilityModel>();
            enu->SetSource(src);
            enu->SetReference(bh.siteLatDeg, bh.siteLonDeg, -1.5);
            ueNodes.Get(j)->AggregateObject(enu);
            const Vector p = enu->GetPosition();
            std::printf("  ground UE %u: TR 38.811 class %s, %.0f m from the platform ground point\n",
                        j, ues[j].className.c_str(), std::hypot(p.x, p.y));
        }
    }

    // ---- Real mmwave NR air interface ----
    NtnRealStackHelper rs;
    rs.SetSimTime(Seconds(duration));
    rs.SetOutputDir(outputDir);
    rs.SetRunTag("sagin-a2g-real-stack");
    rs.SetCarrierFrequencyHz(freqGhz * 1e9);
    // NT-02: declared as CONDUCTED power at the array input. This carrier has
    // no TR 38.821 Set-1 reference in the toolkit, so the EIRP health gate
    // reports "not asserted" rather than certifying an uncalibrated budget.
    rs.SetSatConductedPowerDbm(gnbTxDbm); // short-range A2G gNB Tx (not a LEO link) — kept as-is
    rs.SetRadioBackend(radio == "mmwave" ? NtnRealStackHelper::RadioBackend::Mmwave
                                         : NtnRealStackHelper::RadioBackend::Nr);
    if (radio != "mmwave")
    {
        rs.SetNumerology(1); // FR1 30 kHz SCS
    }
    std::unique_ptr<SaginLeoBackhaul> leo;
    if (leoBackhaul)
    {
        leo = std::make_unique<SaginLeoBackhaul>(bh);
        leo->Configure(rs);
    }
    rs.Build(gnbNodes, ueNodes);
    if (leo)
    {
        SystemPath::MakeDirectories(outputDir);
        leo->Attach(rs, outputDir + "/leo_backhaul.csv");
    }

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
    if (leo)
    {
        leo->Stop();
    }
    rs.Collect();
    rs.WriteHealthReport();
    if (leo)
    {
        leo->PrintSummary("sagin-a2g-real-stack");
    }

    std::cout << "\n--- SAGIN A2G Summary (TR 36.777 on MEASURED radio) ---\n"
              << "  TR 36.777 branch:             " << scenarioStr << " / " << linkStr << "\n"
              << "  last total A2G path loss:     " << a2g->GetLastLossDb() << " dB\n"
              << "  last excess over free space:  " << a2g->GetLastExcessDb()
              << " dB (charged on the chained channel)\n"
              << "  MEASURED air-to-ground SINR:  " << rs.GetMeanDlSinrDb() << " dB\n"
              << "  measured DL TBLER (mean):     " << rs.GetMeanDlTbler() << "\n"
              << "  measured DL throughput:       " << rs.GetRxThroughputMbps() << " Mbps\n"
              << "  measured app one-way delay:   " << rs.GetMeanDelayMs() << " ms (jitter "
              << rs.GetMeanJitterMs() << " ms, app loss " << rs.GetAppLossRatio() << ")\n"
              << "  -> TR 36.777 A2G loss attenuates real packets; SINR is measured.\n";

    Simulator::Destroy();
    return 0;
}
