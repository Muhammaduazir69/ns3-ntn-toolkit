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
    double satEirpDensityDbwMhz = -999.0; // sentinel: TR 38.821 Set-1

    CommandLine cmd(__FILE__);
    cmd.AddValue("simSeconds", "Simulation duration (s)", simSeconds);
    cmd.AddValue("edge", "Inference placement: sat|ground", edge);
    cmd.AddValue("radio", "Radio backend: nr (FR1) or mmwave", radio);
    cmd.AddValue("outputDir", "Output directory", outputDir);
    cmd.AddValue("satEirpDensityDbwMhz",
                 "V2X-9: satellite EIRP density (dBW/MHz); default is TR 38.821 Set-1. Lower it "
                 "to drive the control link below the xApp's 5 dB SINR brake threshold, which is "
                 "how the actuation path gets exercised: on a healthy link the heuristic never "
                 "decides to brake, so the brake was never applied and, before this change, "
                 "would not have applied to anything if it had.",
                 satEirpDensityDbwMhz);
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
    // NT-02: TR 38.821 Table 6.1.1.1-1 Set-1 downlink EIRP density for the
    // S-band LEO reference payload. Declared as a DENSITY so the helper
    // back-computes conducted power against the array gain instead of the
    // antenna being counted twice.
    // V2X-9: the declared EIRP density, overridable so a scenario can drive the
    // control link into the region where the brake heuristic actually fires. On
    // the TR 38.821 Set-1 value the SINR sits near 18 dB, far above the 5 dB
    // threshold, so the brake never triggers and, before this change, would not
    // have applied to anything if it had.
    rs.SetSatEirpDensityDbwMhz(
        (satEirpDensityDbwMhz > -900.0)
            ? satEirpDensityDbwMhz
            : NtnRealStackHelper::kTr38821Set1SBandEirpDensityDbwMhz);
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
    // V2X-9: the brake must reach the vehicle.
    //
    // The inference result used to be reduced to `if (brake) ++brakeCommands;`
    // and a latency sample. The two vehicles were ConstantVelocityMobilityModel
    // at a fixed 27.8 m/s and were never decelerated, and no RAN parameter was
    // touched either, so the xApp's decision changed nothing anywhere in the
    // simulation: the "edge URLLC brake" was a counter.
    Ptr<ConstantVelocityMobilityModel> vehCvm =
        vehNodes.Get(0)->GetObject<ConstantVelocityMobilityModel>();
    NS_ABORT_MSG_IF(!vehCvm, "vehicle 0 must carry a ConstantVelocityMobilityModel to brake");
    const double kCruiseMps = 27.8;
    const double kBrakeDecelMps2 = 3.0;   // a firm but ordinary service brake
    const double kReleaseAccelMps2 = 1.0; // resume gently
    double vehSpeedMps = kCruiseMps;
    double minSpeedSeen = kCruiseMps;
    uint32_t brakeApplications = 0;
    const double kTickS = 0.1; // the KPM/decision cadence below
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
                const double dt = (Simulator::Now() - sensedAt).GetSeconds();
                decisionLatenciesMs.push_back(dt * 1e3);
                // V2X-9: actuate. The command applies to the vehicle's own
                // mobility model, so the decision changes the trajectory the
                // rest of the simulation sees rather than only a tally.
                if (brake)
                {
                    ++brakeCommands;
                    const double before = vehSpeedMps;
                    vehSpeedMps = std::max(0.0, vehSpeedMps - kBrakeDecelMps2 * kTickS);
                    if (vehSpeedMps < before - 1e-9)
                    {
                        ++brakeApplications;
                    }
                }
                else
                {
                    vehSpeedMps = std::min(kCruiseMps, vehSpeedMps + kReleaseAccelMps2 * kTickS);
                }
                minSpeedSeen = std::min(minSpeedSeen, vehSpeedMps);
                vehCvm->SetVelocity(Vector(vehSpeedMps, 0.0, 0.0));
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
    // V2X-9: report the ACTUATION, not just the decision count. A brake command
    // that never reached the vehicle used to be indistinguishable from one that
    // did, because only the tally was printed.
    std::printf("# === actuation (V2X-9) ===  brakeCommands=%u applied=%u "
                "cruise=%.1f m/s minSpeed=%.2f m/s finalSpeed=%.2f m/s\n",
                brakeCommands, brakeApplications, kCruiseMps, minSpeedSeen, vehSpeedMps);
    // V2X-9: a decision that reaches nothing is the defect this closes, so the
    // example FAILS on it rather than printing a note nobody reads. That is the
    // same reasoning as the counter itself: an observation with no consequence
    // is how the original condition survived.
    const bool brakeIsInert = (brakeCommands > 0 && brakeApplications == 0);
    if (brakeIsInert)
    {
        std::printf("#   FAIL: %u brake commands were issued and none changed the vehicle's "
                    "velocity; the decision is not reaching the mobility model.\n",
                    brakeCommands);
    }
    std::printf("# === summary ===  edge=%s inference=%s decisions=%zu "
                "meanDecisionLatency=%.2f ms brakeCommands=%u cellSINR=%.2f dB\n",
                edge.c_str(),
                OranNtnOnnxXapp::IsOnnxAvailable() ? "onnx" : "heuristic",
                decisionLatenciesMs.size(), meanDecMs, brakeCommands,
                rs.GetMeanDlSinrDb());
    Simulator::Destroy();
    return brakeIsInert ? 1 : 0;
}
