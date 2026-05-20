/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// SPDX-License-Identifier: GPL-2.0-only
//
// ntn-cho-leo-basic — smoke test for the realistic event-driven scenario
// path.  Spawns a small number of UEs, runs real UDP traffic through the
// ns-3 stack toward a remote host, and exercises the CHO algorithm on a
// 200 ms cadence so that Simulator::Run() actually advances wall-clock
// time in proportion to simTime.  Emits sim_health.csv at end.

#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/ntn-cho-algorithm.h"
#include "ns3/ntn-cho-helper.h"
#include "ns3/ntn-measurement-model.h"
#include "ns3/ntn-realistic-traffic-helper.h"

#include <iomanip>
#include <iostream>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("NtnChoLeoBasic");

int
main(int argc, char* argv[])
{
    double simTime = 60.0;
    std::string scenario = "suburban";
    std::string triggerType = "tte-aware";
    double d1Threshold = 50000;
    double qualityTh = -3.0;
    double tteMinimum = 15.0;
    uint32_t numUes = 6;
    std::string outputDir = "ntn-cho-basic-out";
    std::string trafficProfile = "mixed";
    bool strict = false;

    CommandLine cmd(__FILE__);
    cmd.AddValue("simTime", "Simulation time in seconds", simTime);
    cmd.AddValue("scenario", "NTN scenario: dense-urban|urban|suburban|rural", scenario);
    cmd.AddValue("trigger", "CHO trigger: a3|location|tte-aware", triggerType);
    cmd.AddValue("d1Threshold", "D1 distance threshold in meters", d1Threshold);
    cmd.AddValue("qualityTh", "SINR quality threshold in dB", qualityTh);
    cmd.AddValue("tteMinimum", "Minimum TTE in seconds", tteMinimum);
    cmd.AddValue("numUes", "Number of UEs", numUes);
    cmd.AddValue("outputDir", "Output directory", outputDir);
    cmd.AddValue("trafficProfile", "Profile: nb-iot|embb|urllc|dt|mixed", trafficProfile);
    cmd.AddValue("strict", "NS_FATAL_ERROR on missed health gates", strict);
    cmd.Parse(argc, argv);

    NtnMeasurementModel::NtnScenario ntnScenario = NtnMeasurementModel::NTN_SUBURBAN;
    if (scenario == "dense-urban") ntnScenario = NtnMeasurementModel::NTN_DENSE_URBAN;
    else if (scenario == "urban") ntnScenario = NtnMeasurementModel::NTN_URBAN;
    else if (scenario == "rural") ntnScenario = NtnMeasurementModel::NTN_RURAL;

    NtnChoAlgorithm::TriggerType trigger = NtnChoAlgorithm::TRIGGER_TTE_AWARE;
    if (triggerType == "a3") trigger = NtnChoAlgorithm::TRIGGER_EVENT_A3;
    else if (triggerType == "location") trigger = NtnChoAlgorithm::TRIGGER_LOCATION_D1;

    std::cout << "========================================\n"
              << "NTN-CHO LEO Basic (event-driven v2)\n"
              << "========================================\n"
              << "  simTime: " << simTime << " s\n"
              << "  numUes:  " << numUes << "\n"
              << "  trigger: " << triggerType << "\n"
              << "  output:  " << outputDir << "\n";

    // ---- Configure the CHO helper (analytical layer) ----
    Ptr<NtnChoHelper> ntnHelper = CreateObject<NtnChoHelper>();
    ntnHelper->SetNtnScenario(ntnScenario);
    ntnHelper->SetChoTriggerType(trigger);
    ntnHelper->SetCarrierFrequency(2.0e9);
    ntnHelper->SetBandwidth(30.0e6);
    ntnHelper->SetSatelliteTxPower(40.0);
    ntnHelper->SetD1Threshold(d1Threshold);
    ntnHelper->SetQualityThreshold(qualityTh);
    ntnHelper->SetTteMinimum(Seconds(tteMinimum));

    Ptr<NtnChoAlgorithm> choAlgo = ntnHelper->CreateChoAlgorithm();
    choAlgo->AddCandidateCell(1, 0, 0);
    choAlgo->AddCandidateCell(2, 0, 5);
    choAlgo->AddCandidateCell(3, 1, 0);
    choAlgo->AddCandidateCell(4, 1, 3);
    choAlgo->UpdateMeasurement(1, 8.0, 5.0);
    choAlgo->UpdateMeasurement(2, 3.0, 1.0);
    choAlgo->UpdateMeasurement(3, 6.0, 3.5);
    choAlgo->UpdateMeasurement(4, -5.0, -4.0);

    // ---- Real traffic plane ----
    NtnRealisticTrafficHelper traffic;
    traffic.SetSimTime(Seconds(simTime));
    traffic.SetOutputDir(outputDir);
    traffic.SetRunTag("ntn-cho-leo-basic_" + triggerType);
    traffic.SetStrictGates(strict);
    if (trafficProfile == "nb-iot")
        traffic.SetProfile(NtnRealisticTrafficHelper::TrafficProfile::NbIotPeriodic);
    else if (trafficProfile == "embb")
        traffic.SetProfile(NtnRealisticTrafficHelper::TrafficProfile::EmbbStreaming);
    else if (trafficProfile == "urllc")
        traffic.SetProfile(NtnRealisticTrafficHelper::TrafficProfile::UrllcPings);
    else if (trafficProfile == "dt")
        traffic.SetProfile(NtnRealisticTrafficHelper::TrafficProfile::DigitalTwinTelemetry);
    else
        traffic.SetProfile(NtnRealisticTrafficHelper::TrafficProfile::MixedBouquet);

    NodeContainer ues = traffic.InstallUes(numUes);

    // Register an analytical CHO tick at 200 ms cadence; each tick
    // re-evaluates the candidate set so the algorithm path is exercised.
    traffic.RegisterPeriodicCallback(MilliSeconds(200), [&](Time now) {
        double sinrJitter = std::sin(now.GetSeconds() * 0.31) * 1.5;
        choAlgo->UpdateMeasurement(1, 8.0 + sinrJitter, 5.0);
        choAlgo->UpdateMeasurement(3, 6.0 - sinrJitter, 3.5);
        choAlgo->SelectBaselineLocationOnly();
        choAlgo->SelectBaselineA3(-2.0);
    });

    traffic.Wire();

    Simulator::Stop(Seconds(simTime + 0.5));
    Simulator::Run();
    traffic.WriteHealthReport();
    Simulator::Destroy();
    return 0;
}
