/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only
//
// ntn-sagin-remote-coverage — the paper's remote/rural coverage use case with
// INFRASTRUCTURE SHARING between two MNOs (Deng 2026 Sec. V-B; adoption plan
// WS6), on a REAL LEO cell.
//
// A remote village has four households: two subscribe to MNO-A, two to
// MNO-B. Only MNO-A has a satellite overhead. With --sharing=1 the O-RU/O-DU
// is shared: MNO-B's UEs ride MNO-A's REAL cell on their own S-NSSAI
// (SST 1 / SD 0xB), and the per-MNO delivered volume — measured in-band by
// the KPM monitor — yields the cost-sharing split. With --sharing=0 MNO-B's
// households simply have NO service (their flows never start): the measured
// coverage gain of sharing is the whole point.
//
// Run both:  ./ns3 run "ntn-sagin-remote-coverage --sharing=0"
//            ./ns3 run "ntn-sagin-remote-coverage --sharing=1"
#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/ntn-oran-ai-flow-monitor.h"
#include "ns3/ntn-oran-application.h"
#include "ns3/ntn-oran-sink.h"
#include "ns3/ntn-real-stack-helper.h"
#include "ns3/ntn-tr38811-mobility-model.h"
#include "ns3/sgp4-mobility-model.h"
#include "ns3/walker-constellation.h"

#include <cstdio>
#include <string>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("NtnSaginRemoteCoverage");

int
main(int argc, char* argv[])
{
    double simSeconds = 40.0;
    bool sharing = true;
    std::string outputDir = "ntn-sagin-remote-coverage-output";

    CommandLine cmd(__FILE__);
    cmd.AddValue("simSeconds", "Simulation duration (s)", simSeconds);
    cmd.AddValue("sharing", "Multi-MNO infrastructure sharing on/off", sharing);
    cmd.AddValue("outputDir", "Output directory", outputDir);
    cmd.Parse(argc, argv);

    std::printf("# ntn-sagin-remote-coverage (REAL cell, sharing=%d)\n", sharing ? 1 : 0);

    NodeContainer satNodes;
    satNodes.Create(1);
    NodeContainer ueNodes;
    ueNodes.Create(4); // 0,1 = MNO-A; 2,3 = MNO-B

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
    Ptr<ListPositionAllocator> pos = CreateObject<ListPositionAllocator>();
    pos->Add(Vector(0.0, 0.0, 1.5));
    pos->Add(Vector(800.0, 0.0, 1.5));
    pos->Add(Vector(0.0, 800.0, 1.5));
    pos->Add(Vector(-800.0, -800.0, 1.5));
    mob.SetPositionAllocator(pos);
    mob.Install(ueNodes);

    NtnRealStackHelper rs;
    rs.SetSimTime(Seconds(simSeconds));
    rs.SetOutputDir(outputDir);
    rs.SetRunTag(std::string("ntn-sagin-remote-coverage-") +
                 (sharing ? "shared" : "unshared"));
    rs.SetCarrierFrequencyHz(2.0e9);
    rs.SetSatEirpDbm(60.0);
    rs.Build(satNodes, ueNodes);

    const Time start = Seconds(1.0);
    const Time stop = Seconds(simSeconds - 0.5);
    // MNO-A households (SST 1 / SD 0xA): always served — it's their satellite.
    rs.InstallOranFlow(0, 2, 1, 0x00000A, NtnOranApplication::CBR_SATURATING, start, stop);
    rs.InstallOranFlow(1, 9, 1, 0x00000A, NtnOranApplication::MMTC_PERIODIC, start, stop);
    // MNO-B households (SST 1 / SD 0xB): only with infrastructure sharing.
    if (sharing)
    {
        rs.InstallOranFlow(2, 2, 1, 0x00000B, NtnOranApplication::CBR_SATURATING, start,
                           stop);
        rs.InstallOranFlow(3, 9, 1, 0x00000B, NtnOranApplication::MMTC_PERIODIC, start,
                           stop);
    }
    Ptr<NtnOranAiFlowMonitor> kpm = rs.EnableOranFlowMonitor();

    Simulator::Stop(Seconds(simSeconds));
    Simulator::Run();
    rs.Collect();
    rs.WriteHealthReport();
    kpm->WriteCsv(outputDir + "/kpm_series.csv");

    // Per-MNO measured accounting (in-band SD tag = the MNO).
    double mnoABytes = 0, mnoBBytes = 0;
    uint32_t servedA = 0, servedB = 0;
    for (uint32_t u = 0; u < 4; ++u)
    {
        const uint64_t rx = rs.GetUeRxBytes(u);
        if (u < 2)
        {
            mnoABytes += rx;
            servedA += (rx > 0) ? 1 : 0;
        }
        else
        {
            mnoBBytes += rx;
            servedB += (rx > 0) ? 1 : 0;
        }
    }
    const double total = mnoABytes + mnoBBytes;
    std::printf("# === measured coverage / cost share ===\n");
    std::printf("#   MNO-A: %u/2 households served, %.3f Mbps delivered\n", servedA,
                mnoABytes * 8.0 / simSeconds / 1e6);
    std::printf("#   MNO-B: %u/2 households served, %.3f Mbps delivered%s\n", servedB,
                mnoBBytes * 8.0 / simSeconds / 1e6,
                sharing ? "" : "  (NO SERVICE without sharing)");
    if (total > 0)
    {
        std::printf("#   cost-sharing split (by measured volume): A=%.1f%% B=%.1f%%\n",
                    100.0 * mnoABytes / total, 100.0 * mnoBBytes / total);
    }
    std::printf("# === summary ===  village coverage %u/4 households, cell SINR=%.2f dB "
                "thr=%.3f Mbps owd=%.2f ms\n",
                servedA + servedB, rs.GetMeanDlSinrDb(), rs.GetRxThroughputMbps(),
                rs.GetMeanDelayMs());
    Simulator::Destroy();
    return 0;
}
