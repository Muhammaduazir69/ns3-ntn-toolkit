/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-only
 * Author: Muhammad Uzair
 *
 * Example: Full THz-NTN Stack Integration Demo
 *
 * Complete scenario combining all THz-NTN module components:
 *   - 2 LEO satellites with THz downlink (225 GHz) and ISL (300 GHz)
 *   - 1 ground terminal with Cassegrain antenna
 *   - Ground-deployed RIS (64x64 elements) assisting downlink
 *   - ISAC debris sensing on Satellite 1
 *   - EKF beam tracking following satellite motion
 *   - MAC layer with DAMC resource allocation
 *
 * Simulates a 30-second scenario and prints aggregate results.
 */

#include <ns3/command-line.h>
#include <ns3/core-module.h>
#include <ns3/mobility-module.h>
#include <ns3/node-container.h>

#include <cmath>
#include <iomanip>
#include <iostream>

// Forward declarations
namespace ns3
{
class ThzNtnHelper;
class ThzNtnChannelModel;
class ThzNtnPhySat;
class ThzNtnPhyGround;
class ThzNtnLinkBudget;
class ThzNtnIslChannel;
class ThzNtnRis;
class ThzNtnIsac;
class ThzNtnBeamTracking;
class ThzNtnBeamforming;
class ThzNtnAntennaArray;
class ThzNtnMac;
} // namespace ns3

#include "ns3/thz-ntn-helper.h"
#include "ns3/thz-ntn-channel-model.h"
#include "ns3/thz-ntn-phy-sat.h"
#include "ns3/thz-ntn-phy-ground.h"
#include "ns3/thz-ntn-link-budget.h"
#include "ns3/thz-ntn-isl-channel.h"
#include "ns3/thz-ntn-ris.h"
#include "ns3/thz-ntn-isac.h"
#include "ns3/thz-ntn-beam-tracking.h"
#include "ns3/thz-ntn-beamforming.h"
#include "ns3/thz-ntn-antenna-array.h"
#include "ns3/thz-ntn-mac.h"
// Needed for the complete type behind CreateMac()'s defaulted
// Ptr<ThzNtnMolecularAbsorption> argument.
#include "ns3/thz-ntn-molecular-absorption.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("ThzNtnFullStack");

/**
 * \brief Compute slant range from elevation angle and altitude.
 */
static double
ComputeSlantRange(double elevDeg, double altKm)
{
    const double Re = 6371.0;
    double elevRad = elevDeg * M_PI / 180.0;
    double ratio = (Re + altKm) / Re;
    double d_km = Re * (std::sqrt(ratio * ratio - std::cos(elevRad) * std::cos(elevRad))
                        - std::sin(elevRad));
    return d_km * 1000.0;
}

/**
 * \brief Simulate satellite elevation profile.
 */
static double
ComputeElevation(double t, double duration, double maxElev)
{
    double phase = M_PI * t / duration;
    double elev = maxElev * std::sin(phase);
    return (elev > 0.0) ? elev : 0.0;
}

int
main(int argc, char* argv[])
{
    // ---- Default parameters ----
    double duration = 30.0;                // seconds
    std::string preset = "TeraLink-225GHz";
    double downlinkFreq = 225e9;           // 225 GHz
    double islFreq = 300e9;                // 300 GHz
    double downlinkBw = 10e9;              // 10 GHz
    double islBw = 20e9;                   // 20 GHz
    double altitude = 550.0;               // km
    double islDistance = 2000.0;            // km between satellites
    double debrisRangeKm = 0.5;            // ISAC debris target range

    // ---- Parse command-line arguments ----
    CommandLine cmd(__FILE__);
    cmd.AddValue("duration", "Simulation duration in seconds [default: 30]", duration);
    cmd.AddValue("preset", "Configuration preset [default: TeraLink-225GHz]", preset);
    cmd.AddValue("debrisRangeKm", "ISAC debris target range in km [default: 0.5]",
                 debrisRangeKm);
    cmd.Parse(argc, argv);

    std::cout << "=============================================================\n";
    std::cout << "  THz-NTN Full Stack Integration Demo\n";
    std::cout << "=============================================================\n";
    std::cout << "  Preset:        " << preset << "\n";
    std::cout << "  Duration:      " << duration << " s\n";
    std::cout << "  Downlink:      " << downlinkFreq / 1e9 << " GHz, "
              << downlinkBw / 1e9 << " GHz BW\n";
    std::cout << "  ISL:           " << islFreq / 1e9 << " GHz, "
              << islBw / 1e9 << " GHz BW\n";
    std::cout << "  Altitude:      " << altitude << " km\n";
    std::cout << "  ISL Distance:  " << islDistance << " km\n";
    std::cout << "-------------------------------------------------------------\n\n";

    // ---- Create nodes ----
    NodeContainer satNodes;
    satNodes.Create(2);

    NodeContainer gtNodes;
    gtNodes.Create(1);

    NodeContainer risNodes;
    risNodes.Create(1);

    // Mobility: Sat1 moving overhead, Sat2 at ISL distance
    Ptr<ConstantPositionMobilityModel> sat1Mob = CreateObject<ConstantPositionMobilityModel>();
    sat1Mob->SetPosition(Vector(0.0, 0.0, altitude * 1000.0));
    satNodes.Get(0)->AggregateObject(sat1Mob);

    Ptr<ConstantPositionMobilityModel> sat2Mob = CreateObject<ConstantPositionMobilityModel>();
    sat2Mob->SetPosition(Vector(islDistance * 1000.0, 0.0, altitude * 1000.0));
    satNodes.Get(1)->AggregateObject(sat2Mob);

    Ptr<ConstantPositionMobilityModel> gtMob = CreateObject<ConstantPositionMobilityModel>();
    gtMob->SetPosition(Vector(0.0, 0.0, 0.0));
    gtNodes.Get(0)->AggregateObject(gtMob);

    Ptr<ConstantPositionMobilityModel> risMob = CreateObject<ConstantPositionMobilityModel>();
    risMob->SetPosition(Vector(150.0, 0.0, 8.0)); // 150m from GT, 8m height
    risNodes.Get(0)->AggregateObject(risMob);

    // ---- Create THz-NTN components using helper ----
    Ptr<ThzNtnHelper> helper = CreateObject<ThzNtnHelper>();

    // Install full stack on satellites
    helper->InstallOnSatellite(satNodes.Get(0), preset);
    helper->InstallOnSatellite(satNodes.Get(1), preset);

    // Install full stack on ground terminal
    helper->InstallOnGroundTerminal(gtNodes.Get(0), preset);

    // Create individual components for analysis
    Ptr<ThzNtnChannelModel> dlChannel = helper->CreateChannelModel(preset);
    dlChannel->EnableMolecularAbsorption(true);
    dlChannel->EnableWeatherEffects(true);
    dlChannel->EnableScintillation(true);
    dlChannel->EnablePointingError(true);

    Ptr<ThzNtnPhySat> satPhy = helper->CreateSatellitePhy(preset);
    satPhy->SetSatelliteClass(SAT_CUBESAT);
    satPhy->SetPayloadMode(PAYLOAD_REGENERATIVE);

    Ptr<ThzNtnPhyGround> gtPhy = helper->CreateGroundPhy(preset);
    gtPhy->SetReceiverType(RX_HETERODYNE);
    gtPhy->SetPolarization(POL_DUAL_CIRCULAR);

    Ptr<ThzNtnLinkBudget> linkBudget = helper->CreateLinkBudget();
    Ptr<ThzNtnIslChannel> islChannel = helper->CreateIslChannel("ISL-300GHz");

    // RIS: 64x64 ground-deployed
    Ptr<ThzNtnRis> ris = CreateObject<ThzNtnRis>();
    ris->Configure(64, 64, downlinkFreq, RisDeployment::GROUND);

    // ISAC on Satellite 1
    Ptr<ThzNtnIsac> isac = CreateObject<ThzNtnIsac>();
    isac->SetIsacMode(JOINT_ISAC);
    isac->SetResourceSplit(0.2); // 20% for sensing

    // Beam tracking
    Ptr<ThzNtnBeamTracking> tracker = helper->CreateBeamTracking();

    // Antenna array
    Ptr<ThzNtnAntennaArray> array = helper->CreateAntennaArray(preset);
    array->Configure(32, 32, downlinkFreq, ThzArrayType::UPA);

    // Wire the satellite's real PHY and UM-MIMO array into the ISAC so the
    // radar SNR uses the actual TX power, bandwidth and array gain rather than
    // the standalone defaults.
    isac->SetPhy(satPhy);
    isac->SetAntennaArray(array);

    // Beamforming
    Ptr<ThzNtnBeamforming> beamforming = helper->CreateBeamforming(64);

    // MAC layer
    Ptr<ThzNtnMac> mac = helper->CreateMac(preset);

    // ---- Print component configuration ----
    std::cout << "  Component Configuration:\n";
    std::cout << "    Sat PHY:     CubeSat, regenerative, 3W\n";
    std::cout << "    Ground PHY:  Heterodyne, dual-circular pol\n";
    std::cout << "    Array:       32x32 UM-MIMO (boresight "
              << std::fixed << std::setprecision(1) << array->ComputeMaxGain_dBi()
              << " dBi)\n";
    std::cout << "    Beamwidth:   " << std::setprecision(3) << array->ComputeBeamwidth3dB_deg()
              << " deg\n";
    std::cout << "    RIS:         64x64 = " << ris->GetTotalElements() << " elements\n";
    std::cout << "    RIS gain:    " << std::setprecision(1) << ris->ComputeSnrGain_dB(true)
              << " dB (perfect CSI)\n";
    std::cout << "    ISAC mode:   JOINT (20% sensing)\n";
    std::cout << "    Beam track:  EKF\n";
    helper->PrintConfiguration();
    std::cout << "\n";

    // ---- Simulate 30-second scenario ----
    double maxElev = 70.0;
    double dt = 1.0; // 1-second time steps
    uint32_t numSteps = static_cast<uint32_t>(duration / dt);

    // Initialize beam tracker
    double initElev = ComputeElevation(0.0, duration, maxElev);
    tracker->Initialize(initElev, 90.0, 2.0, 3.0);

    // ISAC target: a 10 cm debris fragment at the configured range. Detection
    // is decided by the radar equation in PerformSensing (TX power, array gain,
    // RCS, coherent-integration gain) -- a short envelope is expected at THz
    // for a cm-scale target, so the range is a CLI knob (--debrisRangeKm).
    DebrisModel debris = isac->GetDebrisModel("medium_10cm");
    double debrisRcs_m2 = std::pow(10.0, debris.rcs_dBsm / 10.0);
    double debrisRange_m = debrisRangeKm * 1000.0;
    double lastSensingPd = 0.0; // last scan's detection probability (for report)

    // Accumulators
    double totalThroughput = 0.0;
    double totalTrackingError = 0.0;
    uint32_t validSteps = 0;
    uint32_t isacScans = 0;
    uint32_t isacDetections = 0;

    std::cout << "  Time-Series Simulation Results:\n";
    std::cout << "  " << std::string(90, '-') << "\n";
    std::cout << std::setw(6) << "Time"
              << std::setw(8) << "Elev"
              << std::setw(10) << "DL SNR"
              << std::setw(10) << "DL Rate"
              << std::setw(12) << "ISL Cap"
              << std::setw(10) << "RIS Gain"
              << std::setw(10) << "TrkErr"
              << std::setw(10) << "ISAC"
              << std::setw(10) << "SpecEff"
              << "\n";
    std::cout << std::setw(6) << "(s)"
              << std::setw(8) << "(deg)"
              << std::setw(10) << "(dB)"
              << std::setw(10) << "(Gbps)"
              << std::setw(12) << "(Gbps)"
              << std::setw(10) << "(dB)"
              << std::setw(10) << "(deg)"
              << std::setw(10) << ""
              << std::setw(10) << "(b/Hz)"
              << "\n";
    std::cout << "  " << std::string(90, '-') << "\n";

    for (uint32_t step = 0; step < numSteps; step++)
    {
        double t = step * dt;
        double elev = ComputeElevation(t, duration, maxElev);

        if (elev < 5.0)
        {
            continue; // Below minimum elevation
        }
        validSteps++;

        double slantRange = ComputeSlantRange(elev, altitude);
        double azim = 90.0 + 3.0 * t; // slowly changing azimuth

        // ---- 1. Downlink: Sat1 -> Ground Terminal ----
        ThzNtnLinkBudget::LinkBudgetResult dlResult = linkBudget->ComputeLinkBudget(
            ThzNtnLinkBudget::SAT_TO_GROUND,
            downlinkFreq,
            slantRange,
            elev,
            34.77,  // 3W CubeSat
            40.0,   // Tx gain
            45.0,   // Rx gain (Cassegrain)
            downlinkBw,
            10.0);  // NF

        // ---- 2. RIS-assisted SNR improvement ----
        ris->ComputeOptimalPhases(elev, azim, 0.0, 0.0);
        double risGain = ris->ComputeSnrGain_dB(true);
        double risSnr = dlResult.snr_dB + risGain;

        // Effective communication rate (80% of resources for comms)
        double commBw = downlinkBw * 0.8;
        double effectiveSnrLin = std::pow(10.0, risSnr / 10.0);
        double dlRate = commBw * std::log2(1.0 + effectiveSnrLin) / 1e9;
        totalThroughput += dlRate * dt; // Gbit

        // ---- 3. ISL: Sat1 <-> Sat2 ----
        // SNR and capacity come from the two satellites' actual MobilityModels
        // (distance/Doppler from positions, FSPL + configured TX/gains/noise).
        double islSnr = islChannel->ComputeIslSnr_dB(sat1Mob, sat2Mob);
        double islCap = islChannel->ComputeIslCapacity_Gbps(islSnr);

        // ---- 4. Beam tracking ----
        double measNoise = 0.03;
        double noisyElev = elev + measNoise * std::sin(t * 7.3);
        double noisyAzim = azim + measNoise * std::cos(t * 5.1);
        tracker->UpdateMeasurement(noisyElev, noisyAzim, risSnr, t);
        std::pair<double, double> pred = tracker->PredictBeamDirection(t + dt);
        double trackError = std::sqrt(
            std::pow(pred.first - elev, 2.0) + std::pow(pred.second - azim, 2.0));
        totalTrackingError += trackError;

        // ---- 5. ISAC sensing (periodic, every 5 seconds) ----
        std::string isacStatus = "-";
        if (step % 5 == 0)
        {
            // Radar SNR follows the radar equation (range, RCS, ISAC split,
            // coherent-integration gain) using Sat1's MobilityModel.
            ++isacScans;
            SensingResult sensing =
                isac->PerformSensing(sat1Mob, debrisRange_m, debrisRcs_m2);
            lastSensingPd = sensing.detectionProbability;
            if (sensing.targetDetected)
            {
                isacDetections++;
                isacStatus = "DET";
            }
            else
            {
                isacStatus = "scan";
            }
        }

        // ---- 6. Achievable spectral efficiency from the live link ----
        // Adaptive coding tracks the channel: report the Shannon-achievable
        // spectral efficiency for the current (RIS-assisted) SNR. (A discrete
        // DVB ModCod table can be driven via ThzNtnPhySat::SelectWaveformId()
        // when a SatWaveformConf is wired in.)
        double spectralEff = std::log2(1.0 + effectiveSnrLin);

        // Print every other step for readability
        if (step % 2 == 0)
        {
            std::cout << std::fixed << std::setprecision(1)
                      << std::setw(6) << t
                      << std::setw(8) << elev
                      << std::setw(10) << risSnr
                      << std::setprecision(2)
                      << std::setw(10) << dlRate
                      << std::setw(12) << islCap
                      << std::setprecision(1)
                      << std::setw(10) << risGain
                      << std::setprecision(4)
                      << std::setw(10) << trackError
                      << std::setw(10) << isacStatus
                      << std::setprecision(2)
                      << std::setw(10) << spectralEff
                      << "\n";
        }
    }

    // ---- Summary ----
    std::cout << "\n  " << std::string(60, '=') << "\n";
    std::cout << "  SIMULATION SUMMARY (" << duration << " seconds)\n";
    std::cout << "  " << std::string(60, '=') << "\n";

    // Downlink
    double avgRate = (validSteps > 0) ? totalThroughput / (validSteps * dt) : 0.0;
    std::cout << "\n  Downlink (Sat1 -> Ground Terminal):\n";
    std::cout << "    Total data transferred: " << std::fixed << std::setprecision(2)
              << totalThroughput << " Gbit\n";
    std::cout << "    Average throughput:     " << avgRate << " Gbps\n";
    std::cout << "    Active time:            " << validSteps << " s / "
              << numSteps << " s\n";

    // ISL
    double islSnr = islChannel->ComputeIslSnr_dB(sat1Mob, sat2Mob);
    double islCap = islChannel->ComputeIslCapacity_Gbps(islSnr);
    std::cout << "\n  ISL (Sat1 <-> Sat2):\n";
    std::cout << "    Distance:      " << islDistance << " km\n";
    std::cout << "    SNR:           " << std::setprecision(1) << islSnr << " dB\n";
    std::cout << "    Capacity:      " << std::setprecision(2) << islCap << " Gbps\n";

    // Beam tracking
    double avgTrackErr = (validSteps > 0) ? totalTrackingError / validSteps : 0.0;
    ThzTrackingMetrics metrics = tracker->GetTrackingMetrics();
    std::cout << "\n  Beam Tracking:\n";
    std::cout << "    Avg tracking error: " << std::setprecision(4)
              << avgTrackErr << " deg\n";
    std::cout << "    Beam failures:      " << metrics.beamFailureCount << "\n";
    std::cout << "    Beamwidth:          " << std::setprecision(3)
              << array->ComputeBeamwidth3dB_deg() << " deg\n";

    // RIS
    std::cout << "\n  RIS-Assisted Link:\n";
    std::cout << "    Panel size:   64x64 = " << ris->GetTotalElements() << " elements\n";
    std::cout << "    Max SNR gain: " << std::setprecision(1)
              << ris->ComputeSnrGain_dB(true) << " dB\n";
    std::cout << "    Quant loss:   " << std::setprecision(2)
              << ris->ComputeQuantizationLoss_dB() << " dB\n";

    // ISAC
    std::cout << "\n  ISAC (Space Debris Sensing):\n";
    std::cout << "    Target:         10 cm debris (" << std::setprecision(1)
              << debris.rcs_dBsm << " dBsm)\n";
    std::cout << "    Target range:   " << std::setprecision(2)
              << debrisRange_m / 1000.0 << " km\n";
    std::cout << "    Detection prob: " << std::setprecision(3)
              << lastSensingPd << "\n";
    std::cout << "    Sensing scans:  " << isacScans << "\n";
    std::cout << "    Detections:     " << isacDetections << "\n";
    std::cout << "    Resource split: 20% sensing / 80% comms\n";

    // Satellite processing
    double procDelay = satPhy->ComputeOnBoardProcessingDelay().GetMicroSeconds();
    std::cout << "\n  Satellite Processing:\n";
    std::cout << "    Payload mode:     Regenerative\n";
    std::cout << "    Processing delay: " << std::setprecision(0)
              << procDelay << " us\n";

    // Ground terminal
    double rxSensitivity = gtPhy->ComputeReceiverSensitivity_dBm();
    std::cout << "\n  Ground Terminal:\n";
    std::cout << "    Receiver:     Heterodyne\n";
    std::cout << "    Polarization: Dual-circular\n";
    std::cout << "    Sensitivity:  " << std::setprecision(1)
              << rxSensitivity << " dBm\n";

    // ---- Run ns-3 simulation ----
    Simulator::Stop(Seconds(duration));
    Simulator::Run();
    Simulator::Destroy();

    std::cout << "\n  ns-3 simulation complete.\n";
    std::cout << "=============================================================\n";

    return 0;
}
