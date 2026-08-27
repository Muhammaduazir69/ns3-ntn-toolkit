/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-only
 * Author: Muhammad Uzair
 *
 * Example: THz Inter-Satellite Link (ISL) Analysis
 *
 * Demonstrates THz ISL between two LEO satellites at 300 GHz.
 * Evaluates link performance at various inter-satellite distances
 * and compares Shannon vs hardware-limited capacity.
 *
 * Analysis-only example: parametric distance sweep plus an ISL time series,
 * no measured radio. The distance-sweep table is parametric by design; the
 * per-second time series is driven by two REAL SGP4 cross-plane neighbours
 * of a Starlink-class Walker shell (72 x 22, 53 deg, 550 km), whose
 * separation breathes with latitude — not the old sinusoidal drift.
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

#include "ns3/satellite-link-results.h"
#include "ns3/satellite-enums.h"
#include "ns3/ntn-realistic-traffic-helper.h"
#include "ns3/ntn-tr38811-mobility-model.h"
#include "ns3/sgp4-mobility-model.h"
#include "ns3/walker-constellation.h"

// Forward declarations of THz-NTN classes
namespace ns3
{
class ThzNtnHelper;
class ThzNtnIslChannel;
class ThzNtnLinkBudget;
class ThzNtnAntennaArray;
} // namespace ns3

#include "ns3/thz-ntn-helper.h"
#include "ns3/thz-ntn-isl-channel.h"
#include "ns3/thz-ntn-link-budget.h"
#include "ns3/thz-ntn-antenna-array.h"

#include <cstdio>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("ThzNtnIsl");

namespace
{
double g_lastAppliedOwdMs = 0.0;
double g_lastAppliedPer = 0.0;
uint64_t g_couplingUpdates = 0;
constexpr uint32_t kNumIslUes = 6;

/// WF-06: convert the link budget's SNR into a packet error rate, using the
/// vendored SNS3 DVB-S2 forward-link tables rather than a sigmoid.
///
/// These two examples computed a full THz link budget - FSPL, molecular
/// absorption, weather, scintillation, pointing - printed it to CSV, and then
/// carried their packets over a point-to-point star with a hardcoded 15 ms
/// delay and a RateErrorModel pinned at 0.0. There was no SpectrumPhy, no MAC,
/// no SINR and no TBLER anywhere in the data path, so nothing the physics
/// computed could affect a single packet. The helper's coupling hook,
/// UpdateUeLink(ueIndex, oneWayDelay, per), had zero callers across all 94
/// example files.
///
/// The curve is measured per MODCOD and shipped with the satellite module, so
/// this is a published mapping, not a shape with two tunable constants. THz
/// links do not use DVB-S2 MODCODs - this is a stand-in for a THz-native
/// waterfall and is labelled as one - but it is a real curve with real
/// thresholds, which the previous PER of exactly zero was not.
double
PerFromSnrDb(double snrDb)
{
    static Ptr<SatLinkResultsDvbS2> lr;
    if (!lr)
    {
        lr = CreateObject<SatLinkResultsDvbS2>();
        lr->Initialize();
    }
    const double bler = lr->GetBler(SatEnums::SAT_MODCOD_QPSK_1_TO_2,
                                    SatEnums::NORMAL_FRAME, snrDb);
    return std::min(1.0, std::max(0.0, bler));
}
} // namespace

int
main(int argc, char* argv[])
{
    std::printf("[analytic-tool] This example drives the module's physics/calibration APIs\n"
                "directly (link budgets, scaling laws, comparisons); it does NOT simulate a\n"
                "packet data plane. For measured end-to-end KPIs on a real radio, see this\n"
                "module's *-traffic / *-real-stack examples.\n\n");
    // ---- Default parameters ----
    double freq = 300e9;
    double txPower = 30.0;
    double bandwidth = 20e9;
    double txGain = 40.0;
    double rxGain = 40.0;
    double altitude = 550.0;
    double simTime = 60.0;
    std::string outputDir = "thz-isl-out";

    CommandLine cmd(__FILE__);
    cmd.AddValue("freq", "Carrier frequency in Hz [default: 300e9]", freq);
    cmd.AddValue("txPower", "Transmit power in dBm [default: 30]", txPower);
    cmd.AddValue("bandwidth", "Channel bandwidth in Hz [default: 20e9]", bandwidth);
    cmd.AddValue("txGain", "Tx antenna gain in dBi [default: 40]", txGain);
    cmd.AddValue("rxGain", "Rx antenna gain in dBi [default: 40]", rxGain);
    cmd.AddValue("simTime", "Simulation time (s)", simTime);
    cmd.AddValue("outputDir", "Output directory", outputDir);
    cmd.Parse(argc, argv);

    std::cout << "=============================================================\n";
    std::cout << "  THz Inter-Satellite Link (ISL) Analysis\n";
    std::cout << "=============================================================\n";
    std::cout << "  Frequency:  " << freq / 1e9 << " GHz\n";
    std::cout << "  Tx Power:   " << txPower << " dBm\n";
    std::cout << "  Bandwidth:  " << bandwidth / 1e9 << " GHz\n";
    std::cout << "  Tx Gain:    " << txGain << " dBi\n";
    std::cout << "  Rx Gain:    " << rxGain << " dBi\n";
    std::cout << "  Altitude:   " << altitude << " km (both satellites)\n";
    std::cout << "  Channel:    Vacuum (no molecular absorption)\n";
    std::cout << "-------------------------------------------------------------\n\n";

    // ---- Create nodes: two REAL SGP4 satellites at 550 km ----
    // Cross-plane neighbours (plane 0 / slot 0 and plane 1 / slot 0) of a
    // Starlink-class Walker delta shell, projected into one common local ENU
    // frame at sat 1's initial sub-point. Sanity: in ECEF each satellite sits
    // at |position| = Re + altitude (~6921 km for 550 km); their cross-plane
    // separation oscillates with latitude over the orbit.
    NodeContainer satNodes;
    satNodes.Create(2);

    ns3::ntncon::WalkerConfig wcfg;
    wcfg.num_planes = 72;
    wcfg.total_sats = 72 * 22;
    wcfg.altitude_km = altitude;
    wcfg.inclination_deg = 53.0;
    wcfg.epoch_unix_s = 1735689600.0;
    const auto elements = ns3::ntncon::WalkerConstellation::BuildDelta(wcfg);

    Ptr<ns3::ntncon::Sgp4MobilityModel> sat1Sgp4 =
        CreateObject<ns3::ntncon::Sgp4MobilityModel>();
    sat1Sgp4->SetElements(elements[0]); // plane 0, slot 0
    Ptr<ns3::ntncon::Sgp4MobilityModel> sat2Sgp4 =
        CreateObject<ns3::ntncon::Sgp4MobilityModel>();
    sat2Sgp4->SetElements(elements[22]); // plane 1, slot 0

    double refLat;
    double refLon;
    double refAlt;
    sat1Sgp4->GetGeodetic(refLat, refLon, refAlt);

    Ptr<NtnEnuProjectionMobilityModel> sat1Mob = CreateObject<NtnEnuProjectionMobilityModel>();
    sat1Mob->SetSource(sat1Sgp4);
    sat1Mob->SetReference(refLat, refLon, 0.0);
    satNodes.Get(0)->AggregateObject(sat1Mob);

    Ptr<NtnEnuProjectionMobilityModel> sat2Mob = CreateObject<NtnEnuProjectionMobilityModel>();
    sat2Mob->SetSource(sat2Sgp4);
    sat2Mob->SetReference(refLat, refLon, 0.0);
    satNodes.Get(1)->AggregateObject(sat2Mob);

    // ---- Create THz-NTN components ----
    Ptr<ThzNtnHelper> helper = CreateObject<ThzNtnHelper>();
    Ptr<ThzNtnIslChannel> islChannel = helper->CreateIslChannel("ISL-300GHz");
    Ptr<ThzNtnLinkBudget> linkBudget = helper->CreateLinkBudget();

    // ---- Evaluate at various ISL distances ----
    std::vector<double> distances_km = {100.0, 500.0, 1000.0, 2000.0, 5000.0};

    std::cout << "  1. ISL Performance vs Distance\n";
    std::cout << "  " << std::string(78, '-') << "\n";
    std::cout << std::setw(10) << "Distance"
              << std::setw(10) << "FSPL"
              << std::setw(10) << "SNR"
              << std::setw(14) << "Shannon Cap"
              << std::setw(14) << "HW-Ltd Cap"
              << std::setw(10) << "Doppler"
              << std::setw(10) << "Feasible"
              << "\n";
    std::cout << std::setw(10) << "(km)"
              << std::setw(10) << "(dB)"
              << std::setw(10) << "(dB)"
              << std::setw(14) << "(Gbps)"
              << std::setw(14) << "(Gbps)"
              << std::setw(10) << "(MHz)"
              << std::setw(10) << ""
              << "\n";
    std::cout << "  " << std::string(78, '-') << "\n";

    for (double dist_km : distances_km)
    {
        double dist_m = dist_km * 1000.0;

        double fspl = 20.0 * std::log10(dist_m)
                    + 20.0 * std::log10(freq)
                    + 20.0 * std::log10(4.0 * M_PI / 299792458.0);
        ThzNtnLinkBudget::LinkBudgetResult result = linkBudget->ComputeIslBudget(freq, dist_km);
        double snr = result.snr_dB;
        double shannonCap = islChannel->ComputeIslCapacity_Gbps(snr);
        double hwCap = result.hardwareLimitedCapacity_Gbps;

        // Inline Doppler from relative radial velocity.
        double relVel = (dist_km > 2000.0) ? 1000.0 : 100.0;
        double doppler = relVel / 299792458.0 * freq;

        bool feasible = (snr >= 3.0);

        std::cout << std::fixed << std::setprecision(1)
                  << std::setw(10) << dist_km
                  << std::setw(10) << fspl
                  << std::setw(10) << snr
                  << std::setprecision(2)
                  << std::setw(14) << shannonCap
                  << std::setw(14) << hwCap
                  << std::setprecision(1)
                  << std::setw(10) << doppler / 1e6
                  << std::setw(10) << (feasible ? "Yes" : "No")
                  << "\n";
    }

    // ---- Shannon vs Hardware-limited capacity comparison ----
    std::cout << "\n  2. Shannon vs Hardware-Limited Capacity Comparison\n";
    std::cout << "  " << std::string(60, '-') << "\n";
    std::cout << std::setw(10) << "SNR"
              << std::setw(16) << "Shannon"
              << std::setw(16) << "HW-Limited"
              << std::setw(16) << "Efficiency"
              << "\n";
    std::cout << std::setw(10) << "(dB)"
              << std::setw(16) << "(Gbps)"
              << std::setw(16) << "(Gbps)"
              << std::setw(16) << "(%)"
              << "\n";
    std::cout << "  " << std::string(60, '-') << "\n";

    for (double snr_dB = 0.0; snr_dB <= 40.0; snr_dB += 5.0)
    {
        double shannonCap = islChannel->ComputeIslCapacity_Gbps(snr_dB);

        // Hardware-limited: capacity saturates due to EVM floor
        [[maybe_unused]] double snrLinear = std::pow(10.0, snr_dB / 10.0);
        double hwSnrCeiling = 25.0; // dB, typical EVM-limited ceiling
        double effectiveSnr = std::min(snr_dB, hwSnrCeiling);
        double effectiveSnrLin = std::pow(10.0, effectiveSnr / 10.0);
        double hwCap = bandwidth * std::log2(1.0 + effectiveSnrLin) / 1e9;

        double efficiency = (shannonCap > 0.0) ? (hwCap / shannonCap * 100.0) : 0.0;

        std::cout << std::fixed << std::setprecision(1)
                  << std::setw(10) << snr_dB
                  << std::setprecision(2)
                  << std::setw(16) << shannonCap
                  << std::setw(16) << hwCap
                  << std::setprecision(1)
                  << std::setw(16) << efficiency
                  << "\n";
    }

    // ---- ISL type comparison ----
    std::cout << "\n  3. ISL Type Characteristics\n";
    std::cout << "  " << std::string(60, '-') << "\n";
    std::cout << "  Intra-plane ISL:  Same orbital plane, ~2600 km spacing\n";
    std::cout << "  Inter-plane ISL:  Adjacent planes, ~1000-3000 km\n";
    std::cout << "  Cross-link ISL:   Cross-seam, up to ~5000 km\n\n";

    // Compute noise temperature in space
    double noiseTemp = islChannel->ComputeSpaceNoiseTemperature_K();
    std::cout << "  Space noise temperature: " << noiseTemp << " K\n";
    std::cout << "  (CMB 2.7 K + receiver contribution)\n";

    // Maximum feasible distance
    double maxDist = islChannel->ComputeMaxLinkDistance_km(3.0);
    std::cout << "\n  Maximum ISL distance (SNR > 3 dB): " << std::fixed << std::setprecision(1)
              << maxDist << " km\n";

    // ---- Real packet plane + dynamic ISL-distance sampler (v2) ----
    NtnRealisticTrafficHelper traffic;
    traffic.SetSimTime(Seconds(simTime));
    traffic.SetOutputDir(outputDir);
    traffic.SetRunTag("thz-ntn-isl");
    traffic.SetProfile(NtnRealisticTrafficHelper::TrafficProfile::EmbbStreaming);
    traffic.InstallUes(kNumIslUes);

    std::filesystem::create_directories(outputDir);
    std::ofstream isltimes(outputDir + "/isl_timeseries.csv");
    isltimes << "time_s,distance_km,fspl_dB,snr_dB,shannon_Gbps,doppler_MHz\n";

    traffic.RegisterPeriodicCallback(Seconds(1.0), [&](Time nowT) {
        double t = nowT.GetSeconds();
        // REAL cross-plane separation and radial velocity from the two live
        // SGP4 (ENU-projected) mobility models.
        const Vector p1 = sat1Mob->GetPosition();
        const Vector p2 = sat2Mob->GetPosition();
        const Vector v1 = sat1Mob->GetVelocity();
        const Vector v2 = sat2Mob->GetVelocity();
        const Vector dp(p2.x - p1.x, p2.y - p1.y, p2.z - p1.z);
        const double d_m = std::max(
            std::sqrt(dp.x * dp.x + dp.y * dp.y + dp.z * dp.z), 1.0);
        const double d_km = d_m / 1000.0;
        auto r = linkBudget->ComputeLinkBudget(ThzNtnLinkBudget::INTER_SATELLITE,
            freq, d_m, 90.0, txPower, txGain, rxGain, bandwidth, 10.0);
        // Radial (range-rate) component of the relative velocity -> Doppler.
        const double relV = ((v2.x - v1.x) * dp.x + (v2.y - v1.y) * dp.y +
                             (v2.z - v1.z) * dp.z) / d_m;
        double doppler_MHz = relV / 299792458.0 * freq / 1e6;
        isltimes << std::fixed << std::setprecision(2)
                 << t << "," << d_km << "," << r.fspl_dB << "," << r.snr_dB
                 << "," << r.shannonCapacity_Gbps << "," << doppler_MHz << "\n";

        // WF-06: APPLY the ISL physics to the data plane. The packets crossed
        // a point-to-point star with a hardcoded 15 ms delay and a
        // RateErrorModel pinned at 0.0, so none of the budget above could
        // affect a single one. The helper's coupling hook had no callers.
        const Time owd = Seconds(d_km * 1000.0 / 299792458.0);
        const double per = PerFromSnrDb(r.snr_dB);
        for (uint32_t u = 0; u < kNumIslUes; ++u)
        {
            traffic.UpdateUeLink(u, owd, per);
        }
        g_lastAppliedOwdMs = owd.GetSeconds() * 1e3;
        g_lastAppliedPer = per;
        ++g_couplingUpdates;
    });
    traffic.Wire();

    Simulator::Stop(Seconds(simTime + 0.5));
    Simulator::Run();
    traffic.WriteHealthReport();
    std::cout << "  [WF-06] ISL physics -> data plane: " << g_couplingUpdates
              << " link updates applied; last one-way delay " << g_lastAppliedOwdMs
              << " ms (was a hardcoded 15 ms), last PER " << g_lastAppliedPer
              << " (was pinned at 0)\n";
    if (g_lastAppliedPer > 0.5)
    {
        std::cout << "  [WF-06] LINK DOES NOT CLOSE at these defaults: the computed SNR is below\n"
                     "          any usable MODCOD threshold, so the data plane correctly delivers\n"
                     "          (almost) nothing. This was invisible while the error rate was\n"
                     "          pinned at 0 and every packet arrived regardless of the physics.\n"
                     "          The dominant term is the noise bandwidth: --bandwidth=10e9 puts the\n"
                     "          thermal floor at -174 + 100 dB. Narrowing it closes the link.\n";
    }
    isltimes.close();
    Simulator::Destroy();

    std::cout << "\n  Simulation complete.\n";
    std::cout << "=============================================================\n";

    return 0;
}
