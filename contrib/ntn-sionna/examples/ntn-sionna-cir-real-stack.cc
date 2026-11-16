/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026  Muhammad Uzair
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * ntn-sionna-cir-real-stack — Phase 2 of 2026-06 protocol-fidelity audit
 * (channel-plugin recipe).
 *
 * Audit finding for ntn-sionna: the Sionna RT result was collapsed to a single
 * scalar path_loss_db (NtnSionnaChannel), discarding the multipath structure and
 * the Doppler-driven small-scale fading — so the "ray-traced" link behaved like
 * a static free-space attenuator and the channel KPIs were effectively a number.
 *
 * Here a multipath CIR is applied to the packets. NOTE: the Sionna RT server
 * returns only a scalar path_loss_db (the wire Response carries no taps), so the
 * snapshot used here is NOT ray-traced — it is a hand-authored synthetic Rician
 * profile (one LOS tap + several reflected taps, each with its own delay and
 * direction) installed on
 * SionnaCirPropagationLossModel, which coherently combines the taps after
 * rotating each by its per-tap Doppler (CirDopplerSynthesizer = the C++
 * paths.apply_doppler()) and charges the resulting time-varying fading onto a
 * REAL mmwave NR link (NtnRealStackHelper::AddExtraPropagationLoss). The MEASURED
 * DL SINR then fluctuates with genuine multipath fading — variance the scalar
 * path_loss_db cannot reproduce. The summary reports the measured SINR spread to
 * make that exact difference visible.
 *
 * Usage:
 *   ./ns3 run "ntn-sionna-cir-real-stack --duration=12 --numUes=4"
 */

#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/ntn-real-stack-helper.h"
#include "ns3/ntn-tr38811-mobility-model.h"
#include "ns3/sgp4-mobility-model.h"
#include "ns3/sionna-cir-propagation-loss-model.h"
#include "ns3/walker-constellation.h"

#include <cmath>
#include <complex>
#include <iostream>
#include <limits>
#include <vector>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("NtnSionnaCirRealStack");

namespace
{
NtnRealStackHelper* g_rs = nullptr;
Ptr<SionnaCirPropagationLossModel> g_cir;
double g_simTime = 12.0;
// Measured-SINR fading statistics (the whole point: variance the scalar drops).
double g_sumSinr = 0.0, g_sumSinr2 = 0.0;
double g_minSinr = std::numeric_limits<double>::infinity();
double g_maxSinr = -std::numeric_limits<double>::infinity();
uint64_t g_nSinr = 0;
double g_minFadeDb = std::numeric_limits<double>::infinity();
double g_maxFadeDb = -std::numeric_limits<double>::infinity();

void
SampleSinr()
{
    const double t = Simulator::Now().GetSeconds();
    if (t >= g_simTime)
    {
        return;
    }
    const double s = g_rs->GetUeRecentSinrDb(0);
    if (!std::isnan(s))
    {
        g_sumSinr += s;
        g_sumSinr2 += s * s;
        g_minSinr = std::min(g_minSinr, s);
        g_maxSinr = std::max(g_maxSinr, s);
        ++g_nSinr;
    }
    const double f = g_cir->GetLastFadingDb();
    g_minFadeDb = std::min(g_minFadeDb, f);
    g_maxFadeDb = std::max(g_maxFadeDb, f);
    Simulator::Schedule(MilliSeconds(20), &SampleSinr);
}

// Build a hand-authored synthetic Rician multipath CIR snapshot (NOT ray-traced;
// the Sionna RT server returns only a scalar path loss): a dominant LOS tap plus
// three single-bounce taps off local scatterers.
//
// SIONNA-07: the tap directions and delays are now DERIVED from a geometry
// (satellite position at t=0, UE position, three scatterer points near the UE)
// instead of being free numbers. The old taps left the satellite in four
// directions up to 90 degrees apart, so the 7.6 km/s orbital velocity gave the
// taps Doppler offsets tens of kHz apart and the "fading" was the satellite's
// own motion beating against itself. Physically every path leaves a 550 km
// satellite in almost the same direction (local scatterers subtend about
// 1e-4 rad), so the satellite Doppler is a common shift (pre-compensated per
// TS 38.300 Section 16.14) and only the small differential term, about 1 Hz
// at S band, plus the UE's own motion against the distinct arrival angles, fade
// the link. Directions follow the CirPathTap convention: tx_dir leaves the
// satellite along the path, rx_dir points from the UE back toward the arrival.
CirSnapshot
MakeSnapshot(double freqHz, const Vector& sat, const Vector& ue)
{
    CirSnapshot snap;
    snap.wavelength_m = 299792458.0 / freqHz;
    snap.t_ref_s = 0.0;
    auto unit = [](const Vector& v) {
        double n = std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
        n = (n > 0) ? n : 1.0;
        return Vector(v.x / n, v.y / n, v.z / n);
    };
    auto sub = [](const Vector& a, const Vector& b) {
        return Vector(a.x - b.x, a.y - b.y, a.z - b.z);
    };
    auto len = [](const Vector& v) { return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z); };
    const double c = 299792458.0;
    const double dLos = len(sub(ue, sat));
    // Dominant LOS tap + weaker reflections -> a Rician profile (K ~ 7 dB),
    // typical for an NTN ground link with a clear-ish sky and local scatter.
    snap.taps.push_back({{1.00, 0.00}, 0.0, unit(sub(ue, sat)), unit(sub(sat, ue))});
    struct Scat
    {
        Vector offset; // scatterer position relative to the UE (m)
        std::complex<double> amp;
    };
    const Scat scat[] = {{Vector(8.0, 3.0, -ue.z), {0.26, 0.12}},  // ground patch
                         {Vector(-15.0, 10.0, 6.0), {0.18, -0.18}}, // facade
                         {Vector(5.0, -25.0, 10.0), {0.20, 0.10}}};  // facade
    for (const auto& sc : scat)
    {
        const Vector p(ue.x + sc.offset.x, ue.y + sc.offset.y, ue.z + sc.offset.z);
        const double excess = len(sub(p, sat)) + len(sub(ue, p)) - dLos;
        snap.taps.push_back({sc.amp, std::max(0.0, excess) / c, unit(sub(p, sat)),
                             unit(sub(p, ue))});
    }
    return snap;
}
} // namespace

int
main(int argc, char* argv[])
{
    double duration = 12.0;
    uint32_t numUes = 4;
    double altitudeKm = 550.0;
    double satEirpDbm = -1.0; // sentinel: TR 38.821 Set-1 S-band density (nr), 70 dBm (mmwave)
    double freqGhz = 2.185; // TS 38.101-5 n256 downlink block (2170-2200 MHz). Was 2.0,
                              // which is n256 UPLINK and never conformant as a DL carrier.
    double platformSpeed = 0.0; // 0 = use the REAL SGP4 ephemeris velocity
    std::string radio = "nr";   // radio spine: "nr" (5G-LENA FR1) | "mmwave" (FR2)
    std::string outputDir = "ntn-sionna-cir-real-stack-output";

    CommandLine cmd(__FILE__);
    cmd.AddValue("duration", "Simulation duration (s)", duration);
    cmd.AddValue("numUes", "Number of ground UEs", numUes);
    cmd.AddValue("altitude", "Satellite altitude (km)", altitudeKm);
    cmd.AddValue("satEirpDbm",
                 "-1 = TR 38.821 Set-1 S-band EIRP density (34 dBW/MHz); otherwise conducted power at the array input (dBm)",
                 satEirpDbm);
    cmd.AddValue("radio", "Radio backend: nr (FR1) or mmwave", radio);
    cmd.AddValue("freqGhz", "Carrier frequency (GHz)", freqGhz);
    cmd.AddValue("platformSpeed",
                 "Fallback Tx speed (m/s) used only when the CIR model is driven without "
                 "mobility; the real stack always uses the live SGP4 velocity (SIONNA-06)",
                 platformSpeed);
    cmd.AddValue("outputDir", "Output directory", outputDir);
    cmd.Parse(argc, argv);
    g_simTime = duration;

    std::cout << "\n=== ntn-sionna CIR REAL-STACK (multipath CIR, not scalar PL) ===\n"
              << "  serving cell: real NR (" << radio << ") link, " << numUes << " UEs\n"
              << "  channel: 4-tap synthetic Rician CIR + Doppler (hand-authored, not "
                 "ray-traced) chained on the real link\n"
              << "  measured SINR fades with multipath (variance the scalar PL discards)\n"
              << "  duration: " << duration << " s\n\n";

    NodeContainer satNodes;
    satNodes.Create(1);
    NodeContainer ueNodes;
    ueNodes.Create(numUes);

    // Real SGP4 orbit projected into the scenario's local ENU frame: the
    // serving Walker element is at zenith at t=0 and recedes with genuine
    // orbital dynamics (no fixed-overhead placeholder).
    ns3::ntncon::WalkerConfig wcfgSat;
    wcfgSat.num_planes = 1;
    wcfgSat.total_sats = 80;
    wcfgSat.altitude_km = altitudeKm;
    wcfgSat.inclination_deg = 53.0;
    wcfgSat.epoch_unix_s = 1735689600.0;
    const auto satElements = ns3::ntncon::WalkerConstellation::BuildDelta(wcfgSat);
    Ptr<ns3::ntncon::Sgp4MobilityModel> satSgp4 =
        CreateObject<ns3::ntncon::Sgp4MobilityModel>();
    satSgp4->SetElements(satElements[0]);
    double satSubLat, satSubLon, satSubAlt;
    satSgp4->GetGeodetic(satSubLat, satSubLon, satSubAlt);
    Ptr<NtnEnuProjectionMobilityModel> satEnu = CreateObject<NtnEnuProjectionMobilityModel>();
    satEnu->SetSource(satSgp4);
    satEnu->SetReference(satSubLat, satSubLon, 0.0);
    satNodes.Get(0)->AggregateObject(satEnu);

    MobilityHelper mob;
    mob.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    Ptr<ListPositionAllocator> uePos = CreateObject<ListPositionAllocator>();
    for (uint32_t i = 0; i < numUes; ++i)
    {
        uePos->Add(Vector(1500.0 * i, 0.0, 0.0));
    }
    mob.SetPositionAllocator(uePos);
    mob.Install(ueNodes);

    NtnRealStackHelper rs;
    rs.SetRadioBackend(radio == "mmwave" ? NtnRealStackHelper::RadioBackend::Mmwave
                                         : NtnRealStackHelper::RadioBackend::Nr);
    if (radio != "mmwave")
    {
        rs.SetNumerology(1); // FR1 30 kHz SCS
    }
    rs.SetSimTime(Seconds(duration));
    rs.SetOutputDir(outputDir);
    rs.SetRunTag("ntn-sionna-cir-real-stack");
    rs.SetCarrierFrequencyHz(freqGhz * 1e9);
    // EIRP default (NT-02 calibration). This used to be a fixed 70 dBm of
    // CONDUCTED power, about 88 dBm EIRP once the ~18 dB array gain is added,
    // ~11 dB above the TR 38.821 Table 6.1.1.1-1 Set-1 S-band payload
    // (34 dBW/MHz, 77 dBm over 20 MHz). For nr on an S-band FR1 carrier the
    // default is now that Set-1 density and the helper back-computes the
    // conducted power from the bandwidth and the array gain (again in
    // Build()). mmwave keeps its historical 70 dBm, and an explicit
    // --satEirpDbm keeps meaning conducted power at the array input.
    const bool eirpFromTr38821 = satEirpDbm < 0.0 && radio != "mmwave" &&
                                 freqGhz > 1.0 && freqGhz <= 7.125;
    if (eirpFromTr38821)
    {
        rs.SetSatEirpDensityDbwMhz(NtnRealStackHelper::kTr38821Set1SBandEirpDensityDbwMhz);
    }
    else
    {
        if (satEirpDbm < 0.0)
        {
            satEirpDbm = 70.0; // legacy default (mmwave, or a carrier outside Set-1)
        }
        rs.SetSatConductedPowerDbm(satEirpDbm);
    }
    rs.Build(satNodes, ueNodes);

    // ---- Channel plug-in: the FULL CIR fades real packets ----
    g_cir = CreateObject<SionnaCirPropagationLossModel>();
    // SIONNA-07: one snapshot from UE 0's geometry at t=0 serves every UE (they
    // sit within a few km of each other under a 550 km satellite). The UEs
    // therefore share one fading process; per-UE independent fading would need
    // one snapshot per link.
    g_cir->SetSnapshot(MakeSnapshot(freqGhz * 1e9, satEnu->GetPosition(),
                                    ueNodes.Get(0)->GetObject<MobilityModel>()->GetPosition()));
    // Fallback velocities only. On this real stack the model reads the live
    // SGP4 and UE velocities from the mobility it is handed (SIONNA-06), so
    // --platformSpeed does not reach the packet path.
    g_cir->SetTxVelocity(platformSpeed > 0.0 ? Vector(platformSpeed, 0.0, 0.0)
                                             : satEnu->GetVelocity());
    g_cir->SetRxVelocity(Vector(0.0, 0.0, 0.0));
    rs.AddExtraPropagationLoss(g_cir);

    rs.InstallTraffic(NtnRealStackHelper::TrafficProfile::EmbbStreaming,
                      Seconds(1.0), Seconds(duration - 0.5));
    rs.EnableAiFlowMonitor("ntn-sionna-cir-real-stack"); // WS2 KPM series (TS 28.552 names)
    g_rs = &rs;

    Simulator::Schedule(Seconds(1.0), &SampleSinr);

    Simulator::Stop(Seconds(duration));
    Simulator::Run();
    rs.Collect();
    rs.WriteHealthReport();

    const double meanSinr = g_nSinr ? g_sumSinr / g_nSinr : std::nan("");
    const double varSinr =
        g_nSinr ? std::max(0.0, g_sumSinr2 / g_nSinr - meanSinr * meanSinr) : 0.0;

    std::cout << "\n--- Sionna CIR Summary (synthetic Rician CIR on MEASURED radio) ---\n"
              << "  CIR taps installed:           " << g_cir->GetTapCount()
              << " (>1 => real multipath, not scalar PL)\n"
              << "  CIR fading deviation range:   [" << g_minFadeDb << ", " << g_maxFadeDb
              << "] dB\n"
              << "  MEASURED SINR mean:           " << rs.GetMeanDlSinrDb() << " dB\n"
              << "  MEASURED SINR sampled spread: [" << g_minSinr << ", " << g_maxSinr
              << "] dB, std=" << std::sqrt(varSinr) << " dB\n"
              << "  measured DL TBLER (mean):     " << rs.GetMeanDlTbler() << "\n"
              << "  measured DL throughput:       " << rs.GetRxThroughputMbps() << " Mbps\n"
              << "  -> the SINR spread is multipath fading the scalar path_loss_db drops.\n";

    Simulator::Destroy();
    return 0;
}
