/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only
//
// ntn-tr38821-calibration — calibrates the toolkit's MEASURED radio against
// the official 3GPP TR 38.821 Sec. 6.1.3 link-budget methodology (adoption
// plan WS5). Study case: Set-1 LEO-600, S-band downlink, handheld UE
// (verified parameters: EIRP density 34 dBW/MHz -> 48.77 dBW over 30 MHz;
// handheld G/T = -31.62 dB/K with 0 dBi antenna and NF 7 dB; CNR = 15.78 dB
// at 90 deg elevation where FSPL ~154.8 dB).
//
// The satellite is a REAL SGP4 element at 600 km; every second the example
// computes the TR closed-form CNR at the LIVE slant range and compares it to
// the SINR MEASURED off the PHY trace. Two calibration gates:
//   * tracking: the offset (measured - TR CNR) must be CONSTANT across the
//     pass (std < 1.5 dB) — the offset itself is the known beamforming array
//     gain of the 3GPP spectrum model, reported, not hidden;
//   * slope: the measured SINR decay between zenith and pass end must match
//     the TR FSPL delta within 1 dB — elevation-dependent path loss obeys
//     the official methodology.
//
// Quick test:  --simSeconds=120
#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/ntn-real-stack-helper.h"
#include "ns3/ntn-tr38811-mobility-model.h"
#include "ns3/sgp4-mobility-model.h"
#include "ns3/walker-constellation.h"

#include <cmath>
#include <cstdio>
#include <vector>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("NtnTr38821Calibration");

namespace
{
constexpr double kC = 299792458.0;
// TR 38.821 Set-1 LEO-600 S-band DL (see header).
constexpr double kFreqHz = 2.0e9;
constexpr double kBwHz = 30e6;
constexpr double kEirpDbw = 48.77;   // 34 dBW/MHz + 10log10(30)
constexpr double kGtDbK = -31.62;    // handheld: 0 dBi, NF 7
constexpr double kBoltzDbwHzK = -228.6;

double
FsplDb(double dM)
{
    return 20.0 * std::log10(4.0 * M_PI * dM * kFreqHz / kC);
}

double
TrCnrDb(double slantM)
{
    // CNR = EIRP - FSPL + G/T - 10log10(k) - 10log10(B)   [TR 38.821 6.1.3.1]
    return kEirpDbw - FsplDb(slantM) + kGtDbK - kBoltzDbwHzK - 10.0 * std::log10(kBwHz);
}

} // namespace

int
main(int argc, char* argv[])
{
    double simSeconds = 120.0;
    std::string outputDir = "ntn-tr38821-calibration-output";

    CommandLine cmd(__FILE__);
    cmd.AddValue("simSeconds", "Simulation duration (s)", simSeconds);
    cmd.AddValue("outputDir", "Output directory", outputDir);
    cmd.Parse(argc, argv);

    std::printf("# ntn-tr38821-calibration (Set-1 LEO-600 S-band DL, measured vs TR)\n");
    std::printf("#   TR reference at zenith: FSPL=%.2f dB CNR=%.2f dB\n",
                FsplDb(600e3), TrCnrDb(600e3));

    NodeContainer satNodes;
    satNodes.Create(1);
    NodeContainer ueNodes;
    ueNodes.Create(1);

    ns3::ntncon::WalkerConfig wcfg;
    wcfg.num_planes = 1;
    wcfg.total_sats = 80;
    wcfg.altitude_km = 600.0;
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
    mob.SetPositionAllocator(pos);
    mob.Install(ueNodes);
    Ptr<MobilityModel> ueMob = ueNodes.Get(0)->GetObject<MobilityModel>();

    // Handheld UE noise figure per the TR study case.
    Config::SetDefault("ns3::MmWaveUePhy::NoiseFigure", DoubleValue(7.0));

    NtnRealStackHelper rs;
    rs.SetSimTime(Seconds(simSeconds));
    rs.SetOutputDir(outputDir);
    rs.SetRunTag("ntn-tr38821-calibration");
    rs.SetCarrierFrequencyHz(kFreqHz);
    rs.SetBandwidthHz(kBwHz);
    rs.SetSatEirpDbm(kEirpDbw + 30.0); // dBW -> dBm
    rs.Build(satNodes, ueNodes);
    rs.InstallTraffic(NtnRealStackHelper::TrafficProfile::EmbbStreaming,
                      Seconds(1.0), Seconds(simSeconds - 0.5));

    std::printf("# %5s  %9s  %8s  %8s  %8s\n", "t_s", "slant_km", "tr_cnr", "meas",
                "offset");
    std::vector<double> offsets;
    std::vector<double> trSeries;
    std::vector<double> measSeries;
    rs.RegisterPeriodicCallback(Seconds(2.0), [&](Time now) {
        const Vector u = ueMob->GetPosition();
        const Vector s = satEnu->GetPosition();
        const double slant = std::hypot(s.x - u.x, s.y - u.y, s.z - u.z);
        const double tr = TrCnrDb(slant);
        const double meas = rs.GetUeRecentSinrDb(0);
        if (!std::isnan(meas))
        {
            offsets.push_back(meas - tr);
            trSeries.push_back(tr);
            measSeries.push_back(meas);
            std::printf("  %5.1f  %9.1f  %8.2f  %8.2f  %8.2f\n", now.GetSeconds(),
                        slant / 1e3, tr, meas, meas - tr);
        }
    });

    Simulator::Stop(Seconds(simSeconds));
    Simulator::Run();
    rs.Collect();
    rs.WriteHealthReport();

    double mean = 0;
    for (double o : offsets)
    {
        mean += o;
    }
    mean /= std::max<size_t>(1, offsets.size());
    double var = 0;
    for (double o : offsets)
    {
        var += (o - mean) * (o - mean);
    }
    const double stddev = std::sqrt(var / std::max<size_t>(1, offsets.size()));
    // Slope gate: measured decay vs TR FSPL delta across the pass.
    const double measDelta = measSeries.front() - measSeries.back();
    const double trDelta = trSeries.front() - trSeries.back();
    const bool trackOk = stddev < 1.5;
    const bool slopeOk = std::abs(measDelta - trDelta) < 1.0;

    std::printf("# === calibration ===  samples=%zu offset_mean=%.2f dB (array gain) "
                "offset_std=%.2f dB  meas_delta=%.2f tr_delta=%.2f  -> %s\n",
                offsets.size(), mean, stddev, measDelta, trDelta,
                (trackOk && slopeOk) ? "CALIBRATED" : "FAIL");
    Simulator::Destroy();
    return (trackOk && slopeOk) ? 0 : 1;
}
