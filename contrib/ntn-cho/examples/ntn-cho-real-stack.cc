/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026  Muhammad Uzair
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * ntn-cho-real-stack — real-stack flagship for ntn-cho, on the REAL NTN
 * mobility architecture:
 *
 *   - Satellites: SGP4-propagated Walker-Delta orbits (ntn-constellation
 *     Sgp4MobilityModel). The serving satellite starts at zenith over the UE
 *     field and recedes; the in-plane neighbour genuinely approaches. The
 *     handover emerges from REAL orbital dynamics — no teleports.
 *   - UEs: 3GPP TR 38.811 §6.1.1.1 class mobility (NtnTr38811MobilityModel,
 *     ECEF) — pedestrians/vehicles/IoT moving under the pass.
 *   - Radio: real mmwave NR NTN cell (NtnRealStackHelper: SpectrumPhy + MAC +
 *     HARQ + RLC/PDCP + RRC + EPC); serving SINR/TBLER MEASURED off the PHY.
 *
 * Novel 6G handover mechanisms (selectable, implemented in NtnChoAlgorithm):
 *   --trigger=tte-aware  TTE-aware CHO (the toolkit's Rel-17 baseline novelty)
 *   --trigger=ltm        Rel-19 conditional LTM: L1-filtered measurements +
 *                        CHO reliability; MAC-CE fast switch (~25 ms)
 *   --trigger=pcho       trajectory-prediction CHO: forecast serving outage +
 *                        max predicted time-of-stay candidate
 *   --rachLess           RCHO: ephemeris/GNSS TA pre-compensation skips RACH
 *
 * Candidate link prediction is ephemeris-based (3GPP NTN CHO design): the
 * candidate SINR is the MEASURED serving SINR corrected by the real Friis
 * range ratio 20*log10(servSlant/candSlant) from the live SGP4 slant ranges.
 *
 * Usage:
 *   ./ns3 run "ntn-cho-real-stack --duration=60 --trigger=pcho --rachLess=1"
 */

#include "ns3/core-module.h"
#include "ns3/mmwave-enb-net-device.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/ntn-cho-algorithm.h"
#include "ns3/ntn-cho-helper.h"
#include "ns3/ntn-real-stack-helper.h"
#include "ns3/ntn-scene-recorder.h"
#include "ns3/ntn-tr38811-mobility-model.h"

#include "ns3/sgp4-mobility-model.h"
#include "ns3/walker-constellation.h"

#include <cmath>
#include <cstdio>
#include <iostream>
#include <vector>

using namespace ns3;
using ns3::ntncon::Sgp4MobilityModel;
using ns3::ntncon::WalkerConfig;
using ns3::ntncon::WalkerConstellation;

NS_LOG_COMPONENT_DEFINE("NtnChoRealStack");

namespace
{
Ptr<NtnChoAlgorithm> g_cho;
NtnRealStackHelper* g_rs = nullptr;
Ptr<MobilityModel> g_servMob;
Ptr<MobilityModel> g_candMob;
Ptr<NtnTr38811MobilityModel> g_ueMob;
uint16_t g_servingCellId = 0;
uint16_t g_candCellId = 0;
uint16_t g_serving = 0;
uint32_t g_handovers = 0;
uint32_t g_evals = 0;
double g_simTime = 60.0;

// Optional 3D scene trace (NetSimulyzer + Cesium CZML). Off unless --netSim or
// --czml is given; when on, the recorder streams the REAL SGP4 sat + TR 38.811
// UE positions and the MEASURED serving SINR, and logs each handover.
Ptr<ntnobs::NtnSceneRecorder> g_scene;
uint32_t g_sinrSeries = 0;
uint32_t g_servSatNodeId = 0;
uint32_t g_candSatNodeId = 0;
uint32_t g_ueNodeId = 0;

void
ChoTick()
{
    if (Simulator::Now().GetSeconds() >= g_simTime)
    {
        return;
    }
    ++g_evals;
    const Vector u = g_ueMob->GetPosition();        // TR 38.811 UE, ECEF
    const Vector sPos = g_servMob->GetPosition();   // SGP4 serving sat, ECEF
    const Vector cPos = g_candMob->GetPosition();   // SGP4 candidate sat, ECEF
    const double servElev = ntngeo::ElevationDeg(u, sPos);
    const double candElev = ntngeo::ElevationDeg(u, cPos);
    const double servSlant = ntngeo::SlantRangeM(u, sPos);
    const double candSlant = ntngeo::SlantRangeM(u, cPos);

    // Serving SINR is MEASURED from the real mmwave PHY (UE 0).
    const double servSinr = g_rs->GetUeRecentSinrDb(0);
    if (std::isnan(servSinr))
    {
        Simulator::Schedule(Seconds(1.0), &ChoTick);
        return;
    }
    // Feed the measured serving SINR to the 3D scene KPI series (if enabled).
    if (g_scene)
    {
        g_scene->RecordKpi(g_sinrSeries, servSinr);
    }
    // Candidate SINR: ephemeris-predicted from the REAL slant-range ratio
    // (Friis correction off the measured serving baseline — 3GPP NTN CHO).
    const double candSinr = servSinr + 20.0 * std::log10(servSlant / std::max(1.0, candSlant));
    const double servGain = std::max(-20.0, (servElev - 45.0) / 5.0);
    const double candGain = std::max(-20.0, (candElev - 45.0) / 5.0);

    g_cho->UpdateMeasurement(g_servingCellId, servSinr, servGain);
    g_cho->UpdateMeasurement(g_candCellId, candSinr, candGain);
    // Live ephemeris slant ranges -> RACH-less TA pre-compensation (RCHO).
    g_cho->UpdateCandidateSlantRange(g_servingCellId, servSlant);
    g_cho->UpdateCandidateSlantRange(g_candCellId, candSlant);
    g_cho->EvaluateConditions();

    uint16_t chosen = g_cho->SelectBestCandidate();
    if (chosen != g_serving && chosen != 0 && chosen != g_servingCellId)
    {
        ++g_handovers;
        g_cho->ExecuteHandover(chosen);
        if (g_scene)
        {
            g_scene->OnHandover(g_ueNodeId, g_servSatNodeId, g_candSatNodeId);
        }
        const auto st = g_cho->GetMechanismStats();
        std::printf("  %6.1fs  HANDOVER cell %u -> %u  (servSINR meas=%.1f dB, candSINR "
                    "pred=%.1f dB, interruption=%.1f ms%s)\n",
                    Simulator::Now().GetSeconds(), g_serving, chosen, servSinr, candSinr,
                    st.lastInterruptionMs,
                    st.rachLessExecutions > 0 ? ", RACH-less" : "");
        g_serving = chosen;
    }
    if (g_evals % 5 == 0)
    {
        std::printf("  %6.1fs  servElev=%5.1f candElev=%5.1f  servSlant=%6.0fkm "
                    "candSlant=%6.0fkm  measSINR=%6.2f dB\n",
                    Simulator::Now().GetSeconds(), servElev, candElev, servSlant / 1e3,
                    candSlant / 1e3, servSinr);
    }
    Simulator::Schedule(Seconds(1.0), &ChoTick);
}
} // namespace

int
main(int argc, char* argv[])
{
    double duration = 60.0;
    uint32_t numUes = 2;
    double altitudeKm = 550.0;
    double satEirpDbm = 55.0;
    double freqGhz = 2.0;
    double tteMinSec = 3.0;
    std::string trigger = "pcho";
    bool rachLess = true;
    uint32_t satsPerPlane = 80; // 4.5 deg in-plane spacing -> ~500 km along-track
    std::string outputDir = "ntn-cho-real-stack-output";
    std::string netSimOut; // NetSimulyzer JSON trace (empty = off)
    std::string czmlOut;   // Cesium CZML trace (empty = off)

    CommandLine cmd(__FILE__);
    cmd.AddValue("duration", "Simulation duration (s)", duration);
    cmd.AddValue("numUes", "Number of TR 38.811 UEs on the serving cell", numUes);
    cmd.AddValue("altitude", "Constellation altitude (km)", altitudeKm);
    cmd.AddValue("satEirpDbm", "Satellite EIRP / gNB Tx power (dBm)", satEirpDbm);
    cmd.AddValue("freqGhz", "Carrier frequency (GHz)", freqGhz);
    cmd.AddValue("tteMin", "Minimum TTE for CHO admission (s)", tteMinSec);
    cmd.AddValue("trigger",
                 "Handover trigger: a3|d1|t1|ta|elevation|d2|tte-aware|ltm|pcho",
                 trigger);
    cmd.AddValue("rachLess", "RACH-less execution (ephemeris TA pre-comp)", rachLess);
    cmd.AddValue("satsPerPlane", "Walker in-plane satellites (spacing)", satsPerPlane);
    cmd.AddValue("outputDir", "Output directory", outputDir);
    cmd.AddValue("netSim", "NetSimulyzer 3D JSON trace output path (empty = off)", netSimOut);
    cmd.AddValue("czml", "Cesium CZML 3D trace output path (empty = off)", czmlOut);
    bool liveScene = false;
    cmd.AddValue("liveScene", "Stream live scene frames to stdout (mid-run globe)", liveScene);
    cmd.Parse(argc, argv);
    g_simTime = duration;

    std::cout << "\n=== ntn-cho REAL-STACK (real SGP4 orbits + TR 38.811 UEs) ===\n"
              << "  serving + candidate: SGP4 Walker neighbours (real pass dynamics)\n"
              << "  UEs: TR 38.811 class mobility (real ns-3 MobilityModel, ECEF)\n"
              << "  mechanism: " << trigger << (rachLess ? " + RACH-less (RCHO)" : "")
              << ", duration " << duration << " s\n\n";

    // ---- Real Walker-Delta orbits (SGP4) ----
    WalkerConfig wcfg;
    wcfg.num_planes = 1;
    wcfg.total_sats = satsPerPlane;
    wcfg.altitude_km = altitudeKm;
    wcfg.inclination_deg = 53.0;
    wcfg.epoch_unix_s = 1735689600.0; // 2025-01-01
    const auto elements = WalkerConstellation::BuildDelta(wcfg);

    Ptr<Sgp4MobilityModel> serv = CreateObject<Sgp4MobilityModel>();
    serv->SetElements(elements[0]);
    Ptr<Sgp4MobilityModel> nbrA = CreateObject<Sgp4MobilityModel>();
    nbrA->SetElements(elements[1]);
    Ptr<Sgp4MobilityModel> nbrB = CreateObject<Sgp4MobilityModel>();
    nbrB->SetElements(elements[satsPerPlane - 1]);

    // ---- TR 38.811 UEs in a small box at the serving sat's t=0 sub-point ----
    double subLat, subLon, subAlt;
    serv->GetGeodetic(subLat, subLon, subAlt);
    NodeContainer ueNodes;
    ueNodes.Create(numUes);
    NtnTr38811MobilityHelper ueMobility(/*seed=*/1);
    auto profile = NtnMobilityScenarios::MixedContinental();
    auto ueModels = ueMobility.Install(ueNodes, profile, subLat - 0.03, subLat + 0.03,
                                       subLon - 0.03, subLon + 0.03);
    g_ueMob = ueModels[0];

    // ---- Pick the genuinely APPROACHING in-plane neighbour as candidate ----
    const Vector ue0 = g_ueMob->GetPosition();
    auto approaching = [&ue0](Ptr<Sgp4MobilityModel> s) {
        const Vector p = s->GetPosition();
        const Vector v = s->GetVelocity();
        const Vector toUe(ue0.x - p.x, ue0.y - p.y, ue0.z - p.z);
        return (v.x * toUe.x + v.y * toUe.y + v.z * toUe.z) > 0.0;
    };
    Ptr<Sgp4MobilityModel> cand = approaching(nbrA) ? nbrA : nbrB;

    NodeContainer servSat;
    servSat.Create(1);
    servSat.Get(0)->AggregateObject(serv);
    NodeContainer candSat;
    candSat.Create(1);
    candSat.Get(0)->AggregateObject(cand);
    g_servMob = serv;
    g_candMob = cand;

    // ---- Real mmwave NTN serving cell + measured traffic ----
    NtnRealStackHelper rs;
    rs.SetSimTime(Seconds(duration));
    rs.SetOutputDir(outputDir);
    rs.SetRunTag("ntn-cho-real-stack");
    rs.SetCarrierFrequencyHz(freqGhz * 1e9);
    rs.SetSatEirpDbm(satEirpDbm);
    rs.Build(servSat, ueNodes);
    rs.InstallTraffic(NtnRealStackHelper::TrafficProfile::EmbbStreaming,
                      Seconds(1.0), Seconds(duration - 0.5));
    rs.EnableAiFlowMonitor("ntn-cho-real-stack"); // WS2 KPM series (TS 28.552 names)
    g_rs = &rs;

    // ---- The CHO algorithm with the selected novel 6G mechanism ----
    Ptr<NtnChoHelper> choHelper = CreateObject<NtnChoHelper>();
    choHelper->SetCarrierFrequency(freqGhz * 1e9);
    choHelper->SetSatelliteTxPower(satEirpDbm);
    choHelper->SetTteMinimum(Seconds(tteMinSec));
    g_cho = choHelper->CreateChoAlgorithm();

    NtnChoAlgorithm::ChoConfig cfg = g_cho->GetConfig();
    // Full selectable trigger set: Rel-17 standardized (a3 / d1 / t1 / ta /
    // elevation), Rel-18 (d2), and the toolkit's novel mechanisms (tte-aware /
    // ltm / pcho).
    if (trigger == "a3")
    {
        cfg.triggerType = NtnChoAlgorithm::TRIGGER_EVENT_A3;
    }
    else if (trigger == "d1")
    {
        cfg.triggerType = NtnChoAlgorithm::TRIGGER_LOCATION_D1;
    }
    else if (trigger == "t1")
    {
        cfg.triggerType = NtnChoAlgorithm::TRIGGER_TIME_T1;
    }
    else if (trigger == "ta")
    {
        cfg.triggerType = NtnChoAlgorithm::TRIGGER_TIMING_ADVANCE;
    }
    else if (trigger == "elevation")
    {
        cfg.triggerType = NtnChoAlgorithm::TRIGGER_ELEVATION;
    }
    else if (trigger == "d2")
    {
        cfg.triggerType = NtnChoAlgorithm::TRIGGER_DISTANCE_D2;
    }
    else if (trigger == "ltm")
    {
        cfg.triggerType = NtnChoAlgorithm::TRIGGER_LTM_CONDITIONAL;
    }
    else if (trigger == "pcho")
    {
        cfg.triggerType = NtnChoAlgorithm::TRIGGER_TRAJECTORY_PREDICTIVE;
    }
    else
    {
        cfg.triggerType = NtnChoAlgorithm::TRIGGER_TTE_AWARE;
    }
    cfg.rachLess = rachLess;
    cfg.tteMinimum = Seconds(tteMinSec);
    // PCHO outage threshold sits just under the opening measured SINR so the
    // genuinely-declining pass crosses it during the run.
    cfg.qualityThreshold_dB = 8.0;
    cfg.predictionHorizon = Seconds(8.0);
    cfg.minPredictedTos = Seconds(3.0);
    g_cho->Configure(cfg);

    Ptr<mmwave::MmWaveEnbNetDevice> enb =
        DynamicCast<mmwave::MmWaveEnbNetDevice>(rs.GetEnbDevices().Get(0));
    g_servingCellId = enb ? enb->GetCellId() : 1;
    g_candCellId = g_servingCellId + 100;
    g_serving = g_servingCellId;
    g_cho->SetServingCell(g_servingCellId);
    g_cho->AddCandidateCell(g_servingCellId, 0, 0);
    g_cho->AddCandidateCell(g_candCellId, 1, 0);

    // ---- Optional 3D scene trace (NetSimulyzer + Cesium CZML) ----
    // One recorder taps the REAL SGP4 sats + TR 38.811 UEs (all ECEF) and the
    // MEASURED serving SINR; fans out to both viewers. Off unless a path is set.
    if (!netSimOut.empty() || !czmlOut.empty() || liveScene)
    {
        g_servSatNodeId = servSat.Get(0)->GetId();
        g_candSatNodeId = candSat.Get(0)->GetId();
        g_ueNodeId = ueNodes.Get(0)->GetId();

        g_scene = CreateObject<ntnobs::NtnSceneRecorder>();
        g_scene->SetFrame(ntnobs::NtnSceneRecorder::EcefGlobal);
        g_scene->TrackNode(servSat.Get(0), ntnobs::NtnSceneRecorder::Sat, "serving-sat");
        g_scene->TrackNode(candSat.Get(0), ntnobs::NtnSceneRecorder::Sat, "candidate-sat");
        for (uint32_t i = 0; i < ueNodes.GetN(); ++i)
        {
            g_scene->TrackNode(ueNodes.Get(i),
                               ntnobs::NtnSceneRecorder::Ue,
                               "ue-" + std::to_string(i));
        }
        g_sinrSeries = g_scene->TrackKpiSeries(g_ueNodeId,
                                               ntnobs::NtnSceneRecorder::Sinr,
                                               "serving-SINR-dl");
        g_scene->TrackBeam(g_servSatNodeId, g_ueNodeId);
        if (!netSimOut.empty())
        {
            g_scene->EnableNetSimulyzer(netSimOut);
        }
        if (!czmlOut.empty())
        {
            g_scene->EnableCzml(czmlOut);
        }
        if (liveScene)
        {
            g_scene->EnableLiveStdout(true);
        }
        g_scene->Start();
    }

    Simulator::Schedule(Seconds(1.0), &ChoTick);

    Simulator::Stop(Seconds(duration));
    Simulator::Run();
    if (g_scene)
    {
        g_scene->Stop();
        std::cout << "  3D scene trace events:        " << g_scene->GetEventCount() << "\n";
    }
    rs.Collect();
    rs.WriteHealthReport();

    const auto st = g_cho->GetMechanismStats();
    std::cout << "\n--- CHO Summary (novel 6G mechanisms on REAL orbits + MEASURED SINR) ---\n"
              << "  mechanism:                    " << trigger
              << (rachLess ? " + RACH-less" : "") << "\n"
              << "  measured serving SINR (mean): " << rs.GetMeanDlSinrDb() << " dB\n"
              << "  measured DL throughput:       " << rs.GetRxThroughputMbps() << " Mbps\n"
              << "  CHO evaluations:              " << g_evals << "\n"
              << "  handovers executed:           " << g_handovers << "\n"
              << "  LTM fast switches:            " << st.ltmSwitches << "\n"
              << "  PCHO trajectory triggers:     " << st.pchoTriggers << "\n"
              << "  RACH-less executions:         " << st.rachLessExecutions << " (RACH paid: "
              << st.rachExecutions << ")\n"
              << "  last interruption:            " << st.lastInterruptionMs << " ms\n"
              << "  last pre-computed TA:         " << st.lastPreCompTaUs << " us (ephemeris)\n"
              << "  final serving cell:           " << g_serving << "\n";

    Simulator::Destroy();
    return 0;
}
