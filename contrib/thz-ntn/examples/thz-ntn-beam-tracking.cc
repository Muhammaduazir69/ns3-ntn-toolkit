/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-only
 * Author: Muhammad Uzair
 *
 * Example: EKF Beam Tracking for THz LEO Satellite Pass
 *
 * Demonstrates Extended Kalman Filter beam tracking during a 60-second
 * LEO satellite pass.  Compares EKF vs position-based tracking and
 * shows tracking error, gain loss, and beam state over time.
 */

#include <ns3/command-line.h>
#include <ns3/core-module.h>
#include <ns3/mobility-module.h>
#include <ns3/node-container.h>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>

#include "ns3/ntn-realistic-traffic-helper.h"

// Forward declarations
namespace ns3
{
class ThzNtnHelper;
class ThzNtnBeamTracking;
class ThzNtnAntennaArray;
class ThzNtnBeamforming;
} // namespace ns3

#include "ns3/thz-ntn-helper.h"
#include "ns3/thz-ntn-beam-tracking.h"
#include "ns3/thz-ntn-antenna-array.h"
#include "ns3/thz-ntn-beamforming.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("ThzNtnBeamTrackingExample");

/**
 * \brief Simulate satellite angular position during a pass.
 *
 * Models a LEO satellite passing overhead.  The elevation angle
 * rises from low elevation, peaks near zenith, then descends.
 *
 * \param t time since start of pass (seconds)
 * \param passDuration total pass duration (seconds)
 * \param maxElev maximum elevation during pass (degrees)
 * \param elev output elevation angle
 * \param azim output azimuth angle
 */
static void
SimulateSatellitePass(double t,
                      double passDuration,
                      double maxElev,
                      double& elev,
                      double& azim)
{
    // Elevation follows a sinusoidal profile
    double phase = M_PI * t / passDuration;
    elev = maxElev * std::sin(phase);
    if (elev < 0.0)
    {
        elev = 0.0;
    }

    // Azimuth sweeps from 0 to 180 degrees during the pass
    azim = 180.0 * t / passDuration;
}

int
main(int argc, char* argv[])
{
    // ---- Default parameters ----
    std::string trackingMode = "EKF";
    double updateRate = 100.0;
    double passDuration = 60.0;
    double maxElevation = 75.0;
    double freq = 300e9;
    uint32_t arraySize = 32;
    std::string outputDir = "thz-beam-track-out";

    CommandLine cmd(__FILE__);
    cmd.AddValue("trackingMode", "Tracking mode: EKF or POSITION_BASED", trackingMode);
    cmd.AddValue("updateRate", "Tracking update rate in Hz", updateRate);
    cmd.AddValue("passDuration", "Satellite pass duration (s)", passDuration);
    cmd.AddValue("maxElevation", "Maximum elevation angle", maxElevation);
    cmd.AddValue("outputDir", "Output directory", outputDir);
    cmd.Parse(argc, argv);

    std::cout << "=============================================================\n";
    std::cout << "  THz Beam Tracking During LEO Satellite Pass\n";
    std::cout << "=============================================================\n";
    std::cout << "  Tracking Mode:  " << trackingMode << "\n";
    std::cout << "  Update Rate:    " << updateRate << " Hz\n";
    std::cout << "  Pass Duration:  " << passDuration << " s\n";
    std::cout << "  Max Elevation:  " << maxElevation << " deg\n";
    std::cout << "  Frequency:      " << freq / 1e9 << " GHz\n";
    std::cout << "  Array:          " << arraySize << "x" << arraySize << " UM-MIMO\n";
    std::cout << "-------------------------------------------------------------\n\n";

    // ---- Create nodes ----
    NodeContainer satNodes;
    satNodes.Create(1);

    NodeContainer gtNodes;
    gtNodes.Create(1);

    Ptr<ConstantPositionMobilityModel> satMob = CreateObject<ConstantPositionMobilityModel>();
    satMob->SetPosition(Vector(0.0, 0.0, 550e3));
    satNodes.Get(0)->AggregateObject(satMob);

    Ptr<ConstantPositionMobilityModel> gtMob = CreateObject<ConstantPositionMobilityModel>();
    gtMob->SetPosition(Vector(0.0, 0.0, 0.0));
    gtNodes.Get(0)->AggregateObject(gtMob);

    // ---- Create THz-NTN components ----
    Ptr<ThzNtnHelper> helper = CreateObject<ThzNtnHelper>();

    // Create antenna array for gain computation
    Ptr<ThzNtnAntennaArray> array = helper->CreateAntennaArray("ISL-300GHz");
    array->Configure(arraySize, arraySize, freq, ThzArrayType::UPA);

    double beamwidth = array->ComputeBeamwidth3dB_deg();
    double maxGain = array->ComputeMaxGain_dBi();

    std::cout << "  Array 3-dB beamwidth: " << std::fixed << std::setprecision(3)
              << beamwidth << " deg\n";
    std::cout << "  Array max gain:       " << std::setprecision(1)
              << maxGain << " dBi\n\n";

    // ---- EKF Tracking ----
    Ptr<ThzNtnBeamTracking> ekfTracker = helper->CreateBeamTracking();

    // Get initial satellite position
    double initElev;
    double initAzim;
    SimulateSatellitePass(0.0, passDuration, maxElevation, initElev, initAzim);

    // Estimate initial angular rates
    double dt_init = 0.01;
    double elev2;
    double azim2;
    SimulateSatellitePass(dt_init, passDuration, maxElevation, elev2, azim2);
    double elevRate = (elev2 - initElev) / dt_init;
    double azimRate = (azim2 - initAzim) / dt_init;

    ekfTracker->Initialize(initElev, initAzim, elevRate, azimRate);

    // ---- Position-based tracking (for comparison) ----
    Ptr<ThzNtnBeamTracking> posTracker = helper->CreateBeamTracking();
    posTracker->Initialize(initElev, initAzim, elevRate, azimRate);

    // ---- Simulate the pass ----
    double dt = 1.0 / updateRate;
    uint32_t numSteps = static_cast<uint32_t>(passDuration * updateRate);
    uint32_t printInterval = numSteps / 30; // Print ~30 rows
    if (printInterval == 0) printInterval = 1;

    std::cout << "  Beam Tracking Time Series (EKF vs Position-Based):\n";
    std::cout << "  " << std::string(86, '-') << "\n";
    std::cout << std::setw(8) << "Time"
              << std::setw(10) << "TrueElev"
              << std::setw(10) << "TrueAzim"
              << std::setw(10) << "EKF Elev"
              << std::setw(10) << "EKF Azim"
              << std::setw(10) << "EKF Err"
              << std::setw(10) << "Pos Err"
              << std::setw(10) << "GainLoss"
              << std::setw(10) << "State"
              << "\n";
    std::cout << std::setw(8) << "(s)"
              << std::setw(10) << "(deg)"
              << std::setw(10) << "(deg)"
              << std::setw(10) << "(deg)"
              << std::setw(10) << "(deg)"
              << std::setw(10) << "(deg)"
              << std::setw(10) << "(deg)"
              << std::setw(10) << "(dB)"
              << std::setw(10) << ""
              << "\n";
    std::cout << "  " << std::string(86, '-') << "\n";

    double ekfTotalError = 0.0;
    double posTotalError = 0.0;
    uint32_t ekfFailures = 0;

    for (uint32_t step = 0; step < numSteps; step++)
    {
        double t = step * dt;

        // True satellite angular position
        double trueElev;
        double trueAzim;
        SimulateSatellitePass(t, passDuration, maxElevation, trueElev, trueAzim);

        if (trueElev < 5.0)
        {
            continue; // Below minimum elevation
        }

        // Add measurement noise (simulate realistic beam measurement)
        double measNoise = 0.05; // degrees RMS
        double noisyElev = trueElev + measNoise * (2.0 * ((step % 7) / 6.0) - 1.0);
        double noisyAzim = trueAzim + measNoise * (2.0 * ((step % 11) / 10.0) - 1.0);

        // Compute SNR at true pointing (for beam failure detection)
        double sinr = 15.0 + 10.0 * std::sin(M_PI * t / passDuration);

        // EKF update
        ekfTracker->UpdateMeasurement(noisyElev, noisyAzim, sinr, t);
        std::pair<double, double> ekfPred = ekfTracker->PredictBeamDirection(t + dt);

        // Position-based prediction (uses ephemeris)
        std::pair<double, double> posPred = posTracker->PredictFromEphemeris(
            0.0, t * 0.05, 550.0, 0.0, 0.0); // simplified ephemeris

        posTracker->UpdateMeasurement(noisyElev, noisyAzim, sinr, t);

        // Compute tracking errors
        double ekfError = std::sqrt(
            std::pow(ekfPred.first - trueElev, 2.0) +
            std::pow(ekfPred.second - trueAzim, 2.0));
        double posError = std::sqrt(
            std::pow(posPred.first - trueElev, 2.0) +
            std::pow(posPred.second - trueAzim, 2.0));

        ekfTotalError += ekfError;
        posTotalError += posError;

        // Gain loss from pointing error
        double gainLoss = 0.0;
        if (beamwidth > 0.0)
        {
            // Approximate: 3 dB loss at beamwidth/2, quadratic rolloff
            gainLoss = 3.0 * std::pow(2.0 * ekfError / beamwidth, 2.0);
            if (gainLoss > 20.0) gainLoss = 20.0; // cap
        }

        // Beam state
        ThzBeamTrackingState state = ekfTracker->GetTrackingState();
        std::string stateStr;
        switch (state)
        {
            case ThzBeamTrackingState::TRACKING: stateStr = "TRACK"; break;
            case ThzBeamTrackingState::SEARCHING: stateStr = "SEARCH"; break;
            case ThzBeamTrackingState::BEAM_FAILURE: stateStr = "FAIL"; ekfFailures++; break;
            case ThzBeamTrackingState::RECOVERY: stateStr = "RECOV"; break;
        }

        if (step % printInterval == 0)
        {
            std::cout << std::fixed << std::setprecision(2)
                      << std::setw(8) << t
                      << std::setprecision(2)
                      << std::setw(10) << trueElev
                      << std::setw(10) << trueAzim
                      << std::setw(10) << ekfPred.first
                      << std::setw(10) << ekfPred.second
                      << std::setprecision(4)
                      << std::setw(10) << ekfError
                      << std::setw(10) << posError
                      << std::setprecision(2)
                      << std::setw(10) << gainLoss
                      << std::setw(10) << stateStr
                      << "\n";
        }
    }

    // ---- Summary ----
    uint32_t validSteps = 0;
    for (uint32_t step = 0; step < numSteps; step++)
    {
        double t = step * dt;
        double elev;
        double azim;
        SimulateSatellitePass(t, passDuration, maxElevation, elev, azim);
        if (elev >= 5.0) validSteps++;
    }

    ThzTrackingMetrics metrics = ekfTracker->GetTrackingMetrics();
    double overhead = ekfTracker->ComputeTrackingOverhead(updateRate, 50.0);

    std::cout << "\n  " << std::string(50, '-') << "\n";
    std::cout << "  Tracking Performance Summary:\n";
    std::cout << "    EKF avg tracking error:  " << std::fixed << std::setprecision(4)
              << (validSteps > 0 ? ekfTotalError / validSteps : 0.0) << " deg\n";
    std::cout << "    Pos-based avg error:     "
              << (validSteps > 0 ? posTotalError / validSteps : 0.0) << " deg\n";
    std::cout << "    Beam failures:           " << metrics.beamFailureCount << "\n";
    std::cout << "    Tracking overhead:       " << std::setprecision(2)
              << overhead * 100.0 << " %\n";
    std::cout << "    Prediction accuracy:     " << std::setprecision(4)
              << metrics.predictionAccuracy_deg << " deg (RMS)\n";
    std::cout << "    3-dB beamwidth:          " << std::setprecision(3)
              << beamwidth << " deg\n";

    // ---- Real packet plane (v2 event-driven) ----
    NtnRealisticTrafficHelper traffic;
    traffic.SetSimTime(Seconds(passDuration));
    traffic.SetOutputDir(outputDir);
    traffic.SetRunTag("thz-ntn-beam-tracking");
    traffic.SetProfile(NtnRealisticTrafficHelper::TrafficProfile::EmbbStreaming);
    traffic.InstallUes(6);
    traffic.Wire();

    Simulator::Stop(Seconds(passDuration + 0.5));
    Simulator::Run();
    traffic.WriteHealthReport();
    Simulator::Destroy();

    std::cout << "\n  Simulation complete.\n";
    std::cout << "=============================================================\n";

    return 0;
}
