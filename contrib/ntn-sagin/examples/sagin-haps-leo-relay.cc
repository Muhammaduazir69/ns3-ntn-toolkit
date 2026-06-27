/*
 * SPDX-License-Identifier: GPL-2.0-only
 * Copyright (c) 2026 Muhammad Uzair (ns3-ntn-toolkit, W5)
 *
 * Demo: ground-UE → UAV-relay → HAPS → LEO multi-layer route. The MultiLayer
 * Router selects the layered path every second and logs the per-hop elevation
 * /range, while the ACCESS hop (ground UE <-> UAV relay) is a REAL mmwave NR
 * cell (NtnRealStackHelper) carrying the TR 36.777 air-to-ground channel
 * (SaginA2gPropagationLossModel) — so the access SINR/TBLER/throughput are
 * MEASURED off the mmwave PHY trace, not a closed-form formula. The UAV/HAPS/LEO
 * backhaul layers remain ephemeris/geometry context for the router.
 */
#include "ns3/command-line.h"
#include "ns3/ntn-tr38811-mobility-model.h"
#include "ns3/sgp4-mobility-model.h"
#include "ns3/walker-constellation.h"
#include "ns3/constant-position-mobility-model.h"
#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include "ns3/multi-layer-router.h"
#include "ns3/network-module.h"
#include "ns3/ntn-real-stack-helper.h"
#include "ns3/sagin-a2g-propagation-loss-model.h"
#include "ns3/sagin-helper.h"
#include "ns3/uav-mobility-models.h"

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>

using namespace ns3;

int
main(int argc, char* argv[])
{
    double simTimeSec = 30.0;
    uint32_t numUes = 1;
    double uavAltM = 100.0;
    double gnbTxDbm = -7.0; // short A2G range -> low Tx for a realistic SINR band
    std::string outputDir = "sagin-haps-leo-relay-output";
    std::string csvPath = "sagin-haps-leo-relay.csv";
    std::string radio = "nr"; // radio backend: "nr" (5G-LENA FR1) | "mmwave" (FR2)

    CommandLine cmd(__FILE__);
    cmd.AddValue("simTime", "Simulation duration (s)", simTimeSec);
    cmd.AddValue("numUes", "Number of ground UEs", numUes);
    cmd.AddValue("uavAlt", "UAV relay altitude AGL (m)", uavAltM);
    cmd.AddValue("gnbTxDbm", "UAV relay Tx power (dBm)", gnbTxDbm);
    cmd.AddValue("csv", "Output CSV path (router geometry)", csvPath);
    cmd.AddValue("outputDir", "Output directory", outputDir);
    cmd.AddValue("radio", "Radio backend: nr (FR1) or mmwave", radio);
    cmd.Parse(argc, argv);

    // Ground UE(s) = mmwave UE; UAV relay = mmwave gNB at uavAltM AGL.
    NodeContainer ueNodes;
    ueNodes.Create(numUes);
    MobilityHelper mh;
    mh.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    Ptr<ListPositionAllocator> uePos = CreateObject<ListPositionAllocator>();
    for (uint32_t i = 0; i < numUes; ++i)
    {
        uePos->Add(Vector(150.0 + 120.0 * i, 0.0, 1.5));
    }
    mh.SetPositionAllocator(uePos);
    mh.Install(ueNodes);

    NodeContainer satNodes; // the "gNB" here is the UAV relay
    satNodes.Create(1);
    // Deterministic 25 m/s A<->B patrol leg at uavAltM from the module's own
    // UAV mobility catalog (same pattern as sagin-multihop-traffic) — the UAV
    // genuinely turns around instead of flying off on a straight line.
    Ptr<UavPatrolMobilityModel> uav = CreateObject<UavPatrolMobilityModel>();
    uav->SetAttribute("Speed", DoubleValue(25.0));
    uav->SetEndpoints(Vector(0.0, 0.0, uavAltM), Vector(750.0, 0.0, uavAltM));
    satNodes.Get(0)->AggregateObject(uav);

    // HAPS + LEO backhaul layers (geometry context for the multi-layer router).
    Ptr<ConstantPositionMobilityModel> haps = CreateObject<ConstantPositionMobilityModel>();
    haps->SetPosition(Vector(0.0, 0.0, 20000.0));
    // Real SGP4 orbit projected into the scenario's local ENU frame: the
    // satellite passes overhead near t=0 and recedes with genuine orbital
    // dynamics (no straight-line placeholder).
    ns3::ntncon::WalkerConfig wcfgSat;
    wcfgSat.num_planes = 1;
    wcfgSat.total_sats = 80;
    wcfgSat.altitude_km = 550.0;
    wcfgSat.inclination_deg = 53.0;
    wcfgSat.epoch_unix_s = 1735689600.0;
    const auto satElements = ns3::ntncon::WalkerConstellation::BuildDelta(wcfgSat);
    Ptr<ns3::ntncon::Sgp4MobilityModel> satSgp4 =
        CreateObject<ns3::ntncon::Sgp4MobilityModel>();
    satSgp4->SetElements(satElements[0]);
    double satSubLat, satSubLon, satSubAlt;
    satSgp4->GetGeodetic(satSubLat, satSubLon, satSubAlt);
    Ptr<NtnEnuProjectionMobilityModel> leo = CreateObject<NtnEnuProjectionMobilityModel>();
    leo->SetSource(satSgp4);
    leo->SetReference(satSubLat, satSubLon, 0.0);
    SaginHelper helper;
    Ptr<MultiLayerRouter> router = helper.CreateRouter();
    router->AddNode(SaginLayer::Uav, uav);
    router->AddNode(SaginLayer::Haps, haps);
    router->AddNode(SaginLayer::Leo, leo);

    Ptr<MobilityModel> ueMob = ueNodes.Get(0)->GetObject<MobilityModel>();
    std::filesystem::create_directories(outputDir);
    std::ofstream out(outputDir + "/" + csvPath);
    out << "time_s,n_hops,uav_el_deg,uav_range_km,haps_el_deg,haps_range_km,leo_el_deg,leo_range_km\n";
    for (double t = 0; t < simTimeSec; t += 1.0)
    {
        Simulator::Schedule(Seconds(t), [&out, ueMob, router, t]() {
            auto path = router->Route(ueMob);
            int nHops = static_cast<int>(path.size()) - 1;
            double uavEl = -90, uavRng = 0, hapsEl = -90, hapsRng = 0, leoEl = -90, leoRng = 0;
            for (auto& h : path)
            {
                if (h.layer == SaginLayer::Uav) { uavEl = h.elevationDeg; uavRng = h.rangeM / 1000.0; }
                else if (h.layer == SaginLayer::Haps) { hapsEl = h.elevationDeg; hapsRng = h.rangeM / 1000.0; }
                else if (h.layer == SaginLayer::Leo) { leoEl = h.elevationDeg; leoRng = h.rangeM / 1000.0; }
            }
            out << std::fixed << std::setprecision(3) << t << "," << nHops << "," << uavEl << ","
                << uavRng << "," << hapsEl << "," << hapsRng << "," << leoEl << "," << leoRng << "\n";
        });
    }

    // Real mmwave NR access cell UAV<->ground with the TR 36.777 A2G channel.
    NtnRealStackHelper rs;
    rs.SetSimTime(Seconds(simTimeSec));
    rs.SetOutputDir(outputDir);
    rs.SetRunTag("sagin-haps-leo-relay");
    rs.SetCarrierFrequencyHz(2.0e9);
    rs.SetSatEirpDbm(gnbTxDbm); // short-range A2G access gNB Tx (not a LEO link) — kept as-is
    rs.SetRadioBackend(radio == "mmwave" ? NtnRealStackHelper::RadioBackend::Mmwave
                                         : NtnRealStackHelper::RadioBackend::Nr);
    if (radio != "mmwave")
    {
        rs.SetNumerology(1); // FR1 30 kHz SCS
    }
    rs.Build(satNodes, ueNodes);
    Ptr<SaginA2gPropagationLossModel> a2g = CreateObject<SaginA2gPropagationLossModel>();
    a2g->SetFrequencyGHz(2.0);
    a2g->SetScenario(A2gScenario::UMa_AV);
    a2g->SetLink(A2gLink::NLOS);
    a2g->SetUavAltitudeM(uavAltM);
    rs.AddExtraPropagationLoss(a2g);
    rs.InstallTraffic(NtnRealStackHelper::TrafficProfile::EmbbStreaming,
                      Seconds(1.0), Seconds(simTimeSec - 0.5));
    rs.EnableAiFlowMonitor("sagin-haps-leo-relay");

    Simulator::Stop(Seconds(simTimeSec));
    Simulator::Run();
    rs.Collect();
    rs.WriteHealthReport();
    out.close();

    std::cout << "sagin-haps-leo-relay complete (UAV->ground access on a real mmwave NR cell).\n"
              << "  measured access SINR : " << rs.GetMeanDlSinrDb() << " dB (TR 36.777 A2G)\n"
              << "  measured throughput  : " << rs.GetRxThroughputMbps() << " Mbps\n"
              << "  router geometry csv  : " << outputDir << "/" << csvPath << "\n";
    Simulator::Destroy();
    return 0;
}
