/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only
//
// ntn-v2x-edge-urllc — the paper's V2X use case with EDGE AI at the NTN
// platform (Deng 2026 Sec. V-C; adoption plan WS6), on a REAL LEO cell.
//
// Two highway vehicles ride a real mmwave NR NTN cell from an SGP4 LEO:
//   * URLLC slice (5QI 82, SST 2): periodic platoon control messages —
//     5GAA-style KPIs (one-way delay, PDR) measured in-band;
//   * eMBB slice (5QI 2, SST 1): sensor/diagnostics upload.
// A hazard-detection xApp (OranNtnOnnxXapp: .onnx model when ONNX Runtime is
// built in, registered heuristic otherwise) consumes the URLLC flow's
// MEASURED KPM feature window each 100 ms and decides whether to issue a
// brake command. The inference can run --edge=sat (on the regenerative
// payload, processing-only E2 latency) or --edge=ground (slant/c + core),
// via OranNtnRicPlacement — the measured decision latency (feature sense ->
// command applied) is the experiment outcome, exactly the paper's edge-AI
// argument.
//
// Run both:  ./ns3 run "ntn-v2x-edge-urllc --edge=sat"
//            ./ns3 run "ntn-v2x-edge-urllc --edge=ground"
#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/ntn-oran-ai-flow-monitor.h"
#include "ns3/ntn-oran-application.h"
#include "ns3/ntn-oran-sink.h"
#include "ns3/ntn-real-stack-helper.h"
#include "ns3/ntn-tr38811-mobility-model.h"
#include "ns3/oran-ntn-onnx-xapp.h"
#include "ns3/oran-ntn-ric-placement.h"
#include "ns3/sgp4-mobility-model.h"
#include "ns3/walker-constellation.h"

#include <cstdio>
#include <string>
#include <vector>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("NtnV2xEdgeUrllc");

int
main(int argc, char* argv[])
{
    double simSeconds = 40.0;
    std::string edge = "sat";
    std::string radio = "nr"; // radio backend: "nr" (5G-LENA FR1) | "mmwave" (FR2)
    std::string outputDir = "ntn-v2x-edge-urllc-output";

    CommandLine cmd(__FILE__);
    cmd.AddValue("simSeconds", "Simulation duration (s)", simSeconds);
    cmd.AddValue("edge", "Inference placement: sat|ground", edge);
    cmd.AddValue("radio", "Radio backend: nr (FR1) or mmwave", radio);
    cmd.AddValue("outputDir", "Output directory", outputDir);
    cmd.Parse(argc, argv);

    std::printf("# ntn-v2x-edge-urllc (REAL %s cell, edge=%s)\n", radio.c_str(), edge.c_str());

    NodeContainer satNodes;
    satNodes.Create(1);
    NodeContainer vehNodes;
    vehNodes.Create(2);

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

    // Highway platoon: two vehicles at 27.8 m/s (100 km/h), 50 m headway —
    // real ConstantVelocity ground UEs (TR 38.811 vehicular class speed).
    for (uint32_t i = 0; i < 2; ++i)
    {
        Ptr<ConstantVelocityMobilityModel> vm = CreateObject<ConstantVelocityMobilityModel>();
        vm->SetPosition(Vector(i * -50.0, 0.0, 1.5));
        vm->SetVelocity(Vector(27.8, 0.0, 0.0));
        vehNodes.Get(i)->AggregateObject(vm);
    }

    NtnRealStackHelper rs;
    rs.SetRadioBackend(radio == "mmwave" ? NtnRealStackHelper::RadioBackend::Mmwave
                                         : NtnRealStackHelper::RadioBackend::Nr);
    if (radio != "mmwave")
    {
        rs.SetNumerology(1); // FR1 30 kHz SCS
    }
    rs.SetSimTime(Seconds(simSeconds));
    rs.SetOutputDir(outputDir);
    rs.SetRunTag("ntn-v2x-edge-urllc-" + edge);
    rs.SetCarrierFrequencyHz(2.0e9);
    // nr's Friis LEO link needs ~70 dBm for a healthy SINR; mmwave keeps 60 dBm.
    rs.SetSatEirpDbm(radio == "mmwave" ? 60.0 : 70.0);
    rs.Build(satNodes, vehNodes);

    const Time start = Seconds(1.0);
    const Time stop = Seconds(simSeconds - 0.5);
    ApplicationContainer ctrl = rs.InstallOranFlow(
        0, 82, 2, 0x000001, NtnOranApplication::URLLC_PERIODIC, start, stop);
    rs.InstallOranFlow(1, 2, 1, 0x000001, NtnOranApplication::CBR_SATURATING, start, stop);
    Ptr<NtnOranAiFlowMonitor> kpm = rs.EnableOranFlowMonitor();

    // Edge inference placement -> E2/decision latency from real geometry.
    Ptr<OranNtnRicPlacement> placement = CreateObject<OranNtnRicPlacement>();
    placement->SetSite(edge == "ground" ? OranNtnRicPlacement::Site::GroundCloud
                                        : OranNtnRicPlacement::Site::OnBoardSatellite);

    Ptr<OranNtnOnnxXapp> xapp = CreateObject<OranNtnOnnxXapp>();
    xapp->RegisterHeuristic([](const std::vector<double>& f) {
        // f = {urllc delayMeanMs, lossMean, sinrMeanDb}: brake on degradation
        // (a degraded control link means the platoon must widen its gap).
        return std::vector<double>{(f[0] > 30.0 || f[1] > 0.05 || f[2] < 5.0) ? 1.0 : 0.0};
    });

    Ptr<MobilityModel> vehMob = vehNodes.Get(0)->GetObject<MobilityModel>();
    std::vector<double> decisionLatenciesMs;
    uint32_t brakeCommands = 0;
    rs.RegisterPeriodicCallback(MilliSeconds(100), [&](Time now) {
        // Feature sense on the URLLC flow.
        FlowId urllcId = 0;
        for (const auto& [id, series] : kpm->GetKpmSeries())
        {
            OranFlowKey key;
            if (kpm->GetClassifier()->FindFlow(id, key) && key.fiveQi == 82)
            {
                urllcId = id;
            }
        }
        if (urllcId == 0)
        {
            return;
        }
        const auto f = kpm->GetFeatures(urllcId);
        // Decision rides the placement latency BOTH ways (KPM up, command
        // down) before it applies at the vehicle.
        const double slantM = vehMob->GetDistanceFrom(satEnu);
        const Time leg = placement->ComputeE2Delay(slantM);
        const Time sensedAt = now;
        Simulator::Schedule(leg, [&, f, sensedAt, leg] {
            const auto act = xapp->Infer({f.delayMeanMs, f.lossMean, f.sinrMeanDb});
            const bool brake = !act.empty() && act[0] > 0.5;
            Simulator::Schedule(leg, [&, sensedAt, brake] {
                decisionLatenciesMs.push_back(
                    (Simulator::Now() - sensedAt).GetSeconds() * 1e3);
                if (brake)
                {
                    ++brakeCommands;
                }
            });
        });
    });

    Simulator::Stop(Seconds(simSeconds));
    Simulator::Run();
    rs.Collect();
    rs.WriteHealthReport();
    kpm->WriteCsv(outputDir + "/kpm_series.csv");

    double meanDecMs = 0;
    for (double d : decisionLatenciesMs)
    {
        meanDecMs += d;
    }
    meanDecMs /= std::max<size_t>(1, decisionLatenciesMs.size());

    // 5GAA-style KPIs on the URLLC control flow, measured in-band.
    Ptr<NtnOranSink> ctrlSink = DynamicCast<NtnOranSink>(ctrl.Get(1));
    std::printf("# === 5GAA-style KPIs (measured) ===\n");
    for (const auto& [k, fs] : ctrlSink->GetFlowStats())
    {
        std::printf("#   control 5QI=%u owd=%.2f ms (target <100 ms %s) "
                    "PDR=%.4f jitter=%.3f ms\n",
                    fs.fiveQi, fs.MeanDelayMs(),
                    fs.MeanDelayMs() < 100.0 ? "MET" : "MISSED",
                    1.0 - fs.LossRatio(), fs.jitterMs);
    }
    std::printf("# === summary ===  edge=%s inference=%s decisions=%zu "
                "meanDecisionLatency=%.2f ms brakeCommands=%u cellSINR=%.2f dB\n",
                edge.c_str(),
                OranNtnOnnxXapp::IsOnnxAvailable() ? "onnx" : "heuristic",
                decisionLatenciesMs.size(), meanDecMs, brakeCommands,
                rs.GetMeanDlSinrDb());
    Simulator::Destroy();
    return 0;
}
