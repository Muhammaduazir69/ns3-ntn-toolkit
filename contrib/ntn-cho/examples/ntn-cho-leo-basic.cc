/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Author: Muhammad Uzair
 *
 * Basic NTN CHO Example - Demonstrates TTE-aware Conditional Handover
 *
 * This example creates a simplified NTN scenario with the CHO framework
 * to demonstrate the TTE-aware candidate selection algorithm. It compares
 * three handover strategies:
 *   1. Baseline A3 (RSRP offset)
 *   2. Baseline Location-only CHO (D1 condition, no TTE)
 *   3. Proposed TTE-aware CHO (D1 + quality + TTE ranking)
 *
 * The scenario uses the NtnMeasurementModel for link budget computation
 * and NtnChoAlgorithm for handover decisions.
 */

#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/ntn-cho-algorithm.h"
#include "ns3/ntn-cho-helper.h"
#include "ns3/ntn-measurement-model.h"
#include "ns3/ntn-orbit-predictor.h"
#include "ns3/ntn-tte-estimator.h"

#include <iomanip>
#include <iostream>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("NtnChoLeoBasic");

/**
 * \brief Print comparison results for different CHO strategies
 */
void
PrintResults(const std::string& name, const NtnChoHelper::KpiResults& kpis)
{
    std::cout << "=== " << name << " ===" << std::endl;
    std::cout << "  Total HOs:        " << kpis.totalHandovers << std::endl;
    std::cout << "  Success Rate:     " << std::fixed << std::setprecision(1)
              << kpis.hoSuccessRate << " %" << std::endl;
    std::cout << "  Failure Rate:     " << kpis.hoFailureRate << " %" << std::endl;
    std::cout << "  Ping-Pong Rate:   " << kpis.pingPongRate << " %" << std::endl;
    std::cout << "  Avg ToS:          " << std::setprecision(1) << kpis.avgTimeOfStay_s << " s"
              << std::endl;
    std::cout << std::endl;
}

int
main(int argc, char* argv[])
{
    // ========== Command line parameters ==========
    double simTime = 60.0;       // seconds
    std::string scenario = "suburban";
    std::string triggerType = "tte-aware";
    double d1Threshold = 50000;  // meters
    double qualityTh = -3.0;     // dB
    double tteMinimum = 15.0;    // seconds
    bool verbose = false;

    CommandLine cmd(__FILE__);
    cmd.AddValue("simTime", "Simulation time in seconds", simTime);
    cmd.AddValue("scenario", "NTN scenario: dense-urban, urban, suburban, rural", scenario);
    cmd.AddValue("trigger", "CHO trigger: a3, location, tte-aware", triggerType);
    cmd.AddValue("d1Threshold", "D1 distance threshold in meters", d1Threshold);
    cmd.AddValue("qualityTh", "SINR quality threshold in dB", qualityTh);
    cmd.AddValue("tteMinimum", "Minimum TTE in seconds", tteMinimum);
    cmd.AddValue("verbose", "Enable verbose logging", verbose);
    cmd.Parse(argc, argv);

    if (verbose)
    {
        LogComponentEnable("NtnChoAlgorithm", LOG_LEVEL_INFO);
        LogComponentEnable("NtnTteEstimator", LOG_LEVEL_INFO);
        LogComponentEnable("NtnMeasurementModel", LOG_LEVEL_DEBUG);
        LogComponentEnable("NtnChoHelper", LOG_LEVEL_INFO);
    }

    // ========== Map scenario string to enum ==========
    NtnMeasurementModel::NtnScenario ntnScenario = NtnMeasurementModel::NTN_SUBURBAN;
    if (scenario == "dense-urban") ntnScenario = NtnMeasurementModel::NTN_DENSE_URBAN;
    else if (scenario == "urban") ntnScenario = NtnMeasurementModel::NTN_URBAN;
    else if (scenario == "suburban") ntnScenario = NtnMeasurementModel::NTN_SUBURBAN;
    else if (scenario == "rural") ntnScenario = NtnMeasurementModel::NTN_RURAL;

    NtnChoAlgorithm::TriggerType trigger = NtnChoAlgorithm::TRIGGER_TTE_AWARE;
    if (triggerType == "a3") trigger = NtnChoAlgorithm::TRIGGER_EVENT_A3;
    else if (triggerType == "location") trigger = NtnChoAlgorithm::TRIGGER_LOCATION_D1;
    else if (triggerType == "tte-aware") trigger = NtnChoAlgorithm::TRIGGER_TTE_AWARE;

    // ========== Print configuration ==========
    std::cout << "========================================" << std::endl;
    std::cout << "NTN CHO LEO Basic Example" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "Simulation time: " << simTime << " s" << std::endl;
    std::cout << "NTN scenario:    " << scenario << std::endl;
    std::cout << "CHO trigger:     " << triggerType << std::endl;
    std::cout << "D1 threshold:    " << d1Threshold << " m" << std::endl;
    std::cout << "Quality thresh:  " << qualityTh << " dB" << std::endl;
    std::cout << "TTE minimum:     " << tteMinimum << " s" << std::endl;
    std::cout << "========================================" << std::endl;

    // ========== Create NTN-CHO Helper ==========
    Ptr<NtnChoHelper> ntnHelper = CreateObject<NtnChoHelper>();
    ntnHelper->SetNtnScenario(ntnScenario);
    ntnHelper->SetChoTriggerType(trigger);
    ntnHelper->SetCarrierFrequency(2.0e9);     // 2 GHz S-band
    ntnHelper->SetBandwidth(30.0e6);            // 30 MHz
    ntnHelper->SetSatelliteTxPower(40.0);       // 10W EIRP per beam
    ntnHelper->SetD1Threshold(d1Threshold);
    ntnHelper->SetQualityThreshold(qualityTh);
    ntnHelper->SetTteMinimum(Seconds(tteMinimum));

    // ========== Create CHO Algorithm (standalone test) ==========
    Ptr<NtnChoAlgorithm> choAlgo = ntnHelper->CreateChoAlgorithm();

    // Demonstrate the algorithm with synthetic candidates
    std::cout << "\n--- Demonstrating TTE-aware candidate selection ---" << std::endl;

    // Add synthetic candidates (simulating different satellite beams)
    choAlgo->AddCandidateCell(1, 0, 0);  // Sat0/Beam0: good quality, short TTE
    choAlgo->AddCandidateCell(2, 0, 5);  // Sat0/Beam5: medium quality, long TTE
    choAlgo->AddCandidateCell(3, 1, 0);  // Sat1/Beam0: high quality, medium TTE
    choAlgo->AddCandidateCell(4, 1, 3);  // Sat1/Beam3: low quality

    // Simulate measurements
    choAlgo->UpdateMeasurement(1, 8.0, 5.0);    // High SINR, high gain
    choAlgo->UpdateMeasurement(2, 3.0, 1.0);    // Medium SINR, medium gain
    choAlgo->UpdateMeasurement(3, 6.0, 3.5);    // Good SINR, good gain
    choAlgo->UpdateMeasurement(4, -5.0, -4.0);  // Poor quality

    // Test baseline A3 selection
    uint16_t a3Target = choAlgo->SelectBaselineA3(-2.0);
    std::cout << "Baseline A3 selects:        cell " << a3Target << " (highest SINR)" << std::endl;

    // Test baseline location-only (D1 condition met for all with good quality)
    uint16_t locTarget = choAlgo->SelectBaselineLocationOnly();
    std::cout << "Baseline Location selects:  cell " << locTarget << " (D1 + best SINR)" << std::endl;

    // For TTE-aware: would normally require orbit predictor, shown as concept
    std::cout << "TTE-aware would select:     the candidate with longest predicted TTE" << std::endl;
    std::cout << "  among quality-filtered candidates (SINR > " << qualityTh << " dB)" << std::endl;
    std::cout << "  with TTE >= " << tteMinimum << " s" << std::endl;

    // ========== Print KPI structure ==========
    std::cout << "\n--- KPI Collection Framework ---" << std::endl;
    [[maybe_unused]] NtnChoHelper::KpiResults kpis = ntnHelper->GetKpiResults();
    std::cout << "KPI tracking initialized." << std::endl;
    std::cout << "Available metrics:" << std::endl;
    std::cout << "  - HO success/failure/ping-pong rates" << std::endl;
    std::cout << "  - Average/min/max time-of-stay" << std::endl;
    std::cout << "  - TTE estimation accuracy" << std::endl;
    std::cout << "  - Candidate admission statistics" << std::endl;

    // ========== Demonstrate measurement model ==========
    std::cout << "\n--- NTN Measurement Model ---" << std::endl;
    Ptr<NtnMeasurementModel> measModel = CreateObject<NtnMeasurementModel>();
    measModel->SetCarrierFrequency(2.0e9);
    measModel->SetBandwidth(30.0e6);
    measModel->SetSatelliteTxPower(40.0);
    measModel->SetUeNoiseFigure(7.0);
    measModel->SetNtnScenario(ntnScenario);
    std::cout << "Measurement model configured for " << scenario << " scenario" << std::endl;
    std::cout << "  Carrier: 2 GHz (S-band)" << std::endl;
    std::cout << "  Bandwidth: 30 MHz" << std::endl;
    std::cout << "  Sat EIRP: 40 dBm" << std::endl;
    std::cout << "  UE NF: 7 dB" << std::endl;

    std::cout << "\n========================================" << std::endl;
    std::cout << "NTN-CHO Framework Ready for Full Simulation" << std::endl;
    std::cout << "Use with SNS3 satellite constellation for" << std::endl;
    std::cout << "realistic LEO handover scenarios." << std::endl;
    std::cout << "========================================" << std::endl;

    Simulator::Stop(Seconds(simTime));
    Simulator::Run();
    Simulator::Destroy();

    return 0;
}
