/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026 Muhammad Uzair
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * thz-ntn-beam-tracking — EKF beam tracking of a REAL SGP4 LEO pass, closed
 * over a REAL mmwave NR NTN cell.
 *
 * Audit fix (2026-06 protocol-fidelity audit): the old version drove
 * the EKF with a sinusoidal fake pass and a synthetic SINR (15+10*sin), and
 * its bolted-on traffic helper never saw the tracking. Now:
 *
 *   - the TRUE beam direction (az/el) comes from the live SGP4 geometry of a
 *     genuine Walker element (ENU-projected), not a sine profile;
 *   - the EKF measurement update is fed the MEASURED DL SINR off the mmwave
 *     PHY trace (beam-failure detection sees the real link state);
 *   - the tracker's prediction error maps through the ThzNtnAntennaArray
 *     3-dB beamwidth to a pointing loss that is applied as a LIVE channel
 *     reconfiguration (NtnStaticExtraLossModel), so mispointing degrades the
 *     MEASURED SINR / TBLER / goodput of the real packets — the tracking
 *     loop and the data plane are finally the same loop.
 *
 * --trackingMode=EKF steers the beam with the filter prediction;
 * --trackingMode=POSITION_BASED steers with the last (noisy) measurement
 * (dead reckoning), so the two modes produce measurably different links.
 *
 * Quick test:  --simSeconds=40 --updateRate=10
 */

#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/ntn-real-stack-helper.h"
#include "ns3/ntn-static-extra-loss-model.h"
#include "ns3/ntn-tr38811-mobility-model.h"
#include "ns3/sgp4-mobility-model.h"
#include "ns3/thz-ntn-antenna-array.h"
#include "ns3/thz-ntn-beam-tracking.h"
#include "ns3/thz-ntn-helper.h"
#include "ns3/walker-constellation.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("ThzNtnBeamTrackingExample");

namespace
{

/// True az/el (deg) of the satellite seen from the ground terminal, from the
/// LIVE ENU geometry (no sine-profile placeholder).
void
TrueBeamDirection(const Vector& gnd, const Vector& sat, double& elevDeg, double& azimDeg)
{
    const double dx = sat.x - gnd.x;
    const double dy = sat.y - gnd.y;
    const double dz = sat.z - gnd.z;
    const double horiz = std::max(std::sqrt(dx * dx + dy * dy), 1e-3);
    elevDeg = std::atan2(dz, horiz) * 180.0 / M_PI;
    azimDeg = std::fmod(std::atan2(dx, dy) * 180.0 / M_PI + 360.0, 360.0);
}

} // namespace

int
main(int argc, char* argv[])
{
    std::string trackingMode = "EKF";
    double updateRate = 10.0; // Hz
    double simSeconds = 40.0;
    double freqGHz = 100.0; // sub-THz (3GPP spectrum model upper bound)
    double satEirpDbm = 115.0;
    uint32_t arraySize = 32;
    double measNoiseDeg = 0.05;
    std::string outputDir = "thz-beam-track-out";
    std::string radio = "nr"; // radio backend: nr (FR1) or mmwave

    CommandLine cmd(__FILE__);
    cmd.AddValue("trackingMode", "Tracking mode: EKF or POSITION_BASED", trackingMode);
    cmd.AddValue("updateRate", "Tracking update rate in Hz", updateRate);
    cmd.AddValue("simSeconds", "Simulation duration (s)", simSeconds);
    cmd.AddValue("freqGHz", "Carrier frequency (GHz), capped at 100", freqGHz);
    cmd.AddValue("satEirpDbm", "Satellite EIRP / gNB Tx power (dBm)", satEirpDbm);
    cmd.AddValue("radio", "Radio backend: nr (FR1) or mmwave", radio);
    cmd.AddValue("arraySize", "UM-MIMO array side (NxN)", arraySize);
    cmd.AddValue("measNoiseDeg", "Beam measurement noise RMS (deg)", measNoiseDeg);
    cmd.AddValue("outputDir", "Output directory", outputDir);
    cmd.Parse(argc, argv);

    if (freqGHz > 100.0)
    {
        freqGHz = 100.0;
    }

    std::printf("# thz-ntn-beam-tracking (EKF on a REAL SGP4 pass, loss in the real "
                "packet path)\n");
    std::printf("#   mode=%s rate=%.0fHz sim=%.0fs freq=%.0fGHz array=%ux%u\n",
                trackingMode.c_str(), updateRate, simSeconds, freqGHz, arraySize,
                arraySize);

    // --- REAL mobility: SGP4 Walker element, ENU-projected ---
    NodeContainer satNodes;
    satNodes.Create(1);
    NodeContainer gtNodes;
    gtNodes.Create(1);

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
    Ptr<ListPositionAllocator> gtPos = CreateObject<ListPositionAllocator>();
    gtPos->Add(Vector(0.0, 0.0, 1.5));
    mob.SetPositionAllocator(gtPos);
    mob.Install(gtNodes);
    Ptr<MobilityModel> gtMob = gtNodes.Get(0)->GetObject<MobilityModel>();

    // --- the module's array physics (sets beamwidth -> pointing-loss map) ---
    Ptr<ThzNtnHelper> helper = CreateObject<ThzNtnHelper>();
    Ptr<ThzNtnAntennaArray> array = helper->CreateAntennaArray("GT-100GHz");
    array->Configure(arraySize, arraySize, freqGHz * 1e9, ThzArrayType::UPA);
    const double beamwidth = array->ComputeBeamwidth3dB_deg();
    const double maxGain = array->ComputeMaxGain_dBi();
    std::printf("#   array 3-dB beamwidth=%.3f deg, max gain=%.1f dBi\n", beamwidth,
                maxGain);

    // --- REAL radio with the pointing loss in the packet path ---
    NtnRealStackHelper rs;
    rs.SetRadioBackend(radio == "mmwave" ? NtnRealStackHelper::RadioBackend::Mmwave
                                         : NtnRealStackHelper::RadioBackend::Nr);
    if (radio != "mmwave")
    {
        rs.SetNumerology(1); // FR1 30 kHz SCS
    }
    rs.SetSimTime(Seconds(simSeconds));
    rs.SetOutputDir(outputDir);
    rs.SetRunTag("thz-ntn-beam-tracking");
    rs.SetCarrierFrequencyHz(freqGHz * 1e9);
    rs.SetSatEirpDbm(satEirpDbm);
    rs.Build(satNodes, gtNodes);

    Ptr<NtnStaticExtraLossModel> pointing = CreateObject<NtnStaticExtraLossModel>();
    pointing->SetLossDb(0.0);
    rs.AddExtraPropagationLoss(pointing);

    rs.InstallTraffic(NtnRealStackHelper::TrafficProfile::EmbbStreaming,
                      Seconds(1.0), Seconds(simSeconds - 0.5));
    rs.EnableAiFlowMonitor("thz-ntn-beam-tracking");

    // --- EKF tracker initialised from the REAL geometry ---
    Ptr<ThzNtnBeamTracking> tracker = helper->CreateBeamTracking();
    {
        double e0, a0, e1, a1;
        TrueBeamDirection(gtMob->GetPosition(), satEnu->GetPosition(), e0, a0);
        // Angular rates estimated from the real ephemeris a moment later.
        const Vector satNow = satEnu->GetPosition();
        const Vector satVel = satEnu->GetVelocity();
        const Vector satSoon(satNow.x + satVel.x, satNow.y + satVel.y,
                             satNow.z + satVel.z); // +1 s along the real velocity
        TrueBeamDirection(gtMob->GetPosition(), satSoon, e1, a1);
        tracker->Initialize(e0, a0, e1 - e0, a1 - a0);
    }

    const bool useEkf = (trackingMode == "EKF");
    const double dt = 1.0 / updateRate;
    struct TrackState
    {
        double sumErr = 0.0;
        uint64_t n = 0;
        double sumErrConverged = 0.0; // after the acquisition window (t > 5 s)
        uint64_t nConverged = 0;
        double lastErr = 0.0;
        uint32_t failures = 0;
        double lastMeasElev = 0.0;
        double lastMeasAzim = 0.0;
    };
    static TrackState st;
    static uint32_t step = 0;

    // Tracking tick: real geometry -> noisy measurement -> filter -> steer ->
    // pointing loss applied to the LIVE channel.
    rs.RegisterPeriodicCallback(
        Seconds(dt),
        [&rs, gtMob, satEnu, tracker, pointing, useEkf, beamwidth, measNoiseDeg,
         dt](Time now) {
            double trueElev, trueAzim;
            TrueBeamDirection(gtMob->GetPosition(), satEnu->GetPosition(), trueElev,
                              trueAzim);
            if (trueElev < 5.0)
            {
                pointing->SetLossDb(200.0); // below the service mask
                return;
            }
            // Deterministic pseudo-noise (reproducible runs).
            const double nE = measNoiseDeg * (2.0 * ((step % 7) / 6.0) - 1.0);
            const double nA = measNoiseDeg * (2.0 * ((step % 11) / 10.0) - 1.0);
            ++step;
            const double measElev = trueElev + nE;
            const double measAzim = trueAzim + nA;

            // Feed the filter the MEASURED link SINR (beam-failure detection
            // sees the real radio, not a formula).
            double sinr = rs.GetUeRecentSinrDb(0);
            if (std::isnan(sinr))
            {
                sinr = 0.0;
            }
            tracker->UpdateMeasurement(measElev, measAzim, sinr, now.GetSeconds());

            double steerElev, steerAzim;
            if (useEkf)
            {
                const auto pred = tracker->PredictBeamDirection(now.GetSeconds() + dt);
                steerElev = pred.first;
                steerAzim = pred.second;
            }
            else
            {
                steerElev = st.lastMeasElev != 0.0 ? st.lastMeasElev : measElev;
                steerAzim = st.lastMeasAzim != 0.0 ? st.lastMeasAzim : measAzim;
            }
            st.lastMeasElev = measElev;
            st.lastMeasAzim = measAzim;

            const double err = std::sqrt(std::pow(steerElev - trueElev, 2.0) +
                                         std::pow(steerAzim - trueAzim, 2.0));
            st.sumErr += err;
            st.n++;
            st.lastErr = err;
            if (now.GetSeconds() > 5.0)
            {
                st.sumErrConverged += err;
                st.nConverged++;
            }

            // Pointing error -> array gain loss (3 dB at half beamwidth,
            // quadratic rolloff, 20 dB cap) applied to REAL packets.
            double lossDb = 0.0;
            if (beamwidth > 0.0)
            {
                lossDb = std::min(20.0, 3.0 * std::pow(2.0 * err / beamwidth, 2.0));
            }
            pointing->SetLossDb(lossDb);

            if (tracker->GetTrackingState() == ThzBeamTrackingState::BEAM_FAILURE)
            {
                st.failures++;
            }
        });

    std::printf("# %5s  %8s  %8s  %8s  %8s  %8s  %9s\n",
                "t_s", "trueElev", "trkErr", "loss_dB", "sinr_dB", "tbler", "goodput");
    uint64_t lastRx = 0;
    rs.RegisterPeriodicCallback(
        Seconds(1.0),
        [&rs, gtMob, satEnu, pointing, &lastRx](Time now) {
            double trueElev, trueAzim;
            TrueBeamDirection(gtMob->GetPosition(), satEnu->GetPosition(), trueElev,
                              trueAzim);
            const uint64_t rx = rs.GetUeRxBytes(0);
            const double mbps = (rx - lastRx) * 8.0 / 1e6;
            lastRx = rx;
            std::printf("  %5.1f  %8.2f  %8.4f  %8.2f  %8.2f  %8.3f  %9.3f\n",
                        now.GetSeconds(), trueElev, st.lastErr, pointing->GetLossDb(),
                        rs.GetUeRecentSinrDb(0), rs.GetUeRecentTbler(0), mbps);
        });

    Simulator::Stop(Seconds(simSeconds));
    Simulator::Run();
    rs.Collect();
    rs.WriteHealthReport();

    const ThzTrackingMetrics metrics = tracker->GetTrackingMetrics();
    std::printf("# === summary ===  mode=%s mean tracking error=%.4f deg "
                "(converged, t>5s: %.4f deg), beam failures=%u, prediction "
                "RMS=%.4f deg\n",
                trackingMode.c_str(), st.n ? st.sumErr / st.n : 0.0,
                st.nConverged ? st.sumErrConverged / st.nConverged : 0.0, st.failures,
                metrics.predictionAccuracy_deg);
    std::printf("#                  measured cell SINR=%.2f dB TBLER=%.4f "
                "throughput=%.3f Mbps (pointing loss applied to real packets)\n",
                rs.GetMeanDlSinrDb(), rs.GetMeanDlTbler(), rs.GetRxThroughputMbps());

    Simulator::Destroy();
    return 0;
}
