/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-only
 * Author: Muhammad Uzair
 *
 * Example: RIS-Assisted THz Satellite Link
 *
 * Demonstrates the SNR improvement from deploying a ground-based
 * Reconfigurable Intelligent Surface (RIS) to assist a THz satellite
 * downlink.  Compares direct vs RIS-assisted link performance and
 * verifies the N^2 scaling law.
 *
 * Analysis-only example: parametric geometry, no packet transmission.
 * The configurable elevation angle IS the experiment, so the satellite is
 * placed parametrically along the actual line of sight (local ENU frame),
 * making the node positions consistent with the parametric slant range used
 * by the link budget. For a measured-radio RIS scenario see
 * thz-ntn-ris-relay-traffic.
 *
 * Wave 7C: after the analysis, a measured packet companion
 * (thz-ntn-pass-kit.h) runs the same carrier and satellite power over an SGP4
 * pass (ntn-constellation) to a TR 38.811 static terminal (ntn-cho) twice:
 * direct only, and with a --risSize panel 200 m from the terminal whose
 * co-phased contribution FSPL_direct - PL_cascaded is recomputed every 100 ms
 * from the live ECEF positions and applied to the ThzNtnPacketLink. The
 * ntn-traffic 5QI 2 flow measures what the RIS changes. The companion uses
 * declared apertures (--companionGainDbi) and channel (--companionBandwidthGhz)
 * so that the direct link closes; --companion=0 skips it.
 */

#include "thz-ntn-pass-kit.h"
#include "ns3/ntn-dataset-writer.h"
#include "ns3/thz-ntn-link-geometry.h"

#include <ns3/command-line.h>
#include <ns3/core-module.h>
#include <ns3/mobility-module.h>
#include <ns3/node-container.h>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <vector>

// Forward declarations
namespace ns3
{
class ThzNtnHelper;
class ThzNtnRis;
class ThzNtnLinkBudget;
class ThzNtnChannelModel;
} // namespace ns3

#include "ns3/thz-ntn-helper.h"
#include "ns3/thz-ntn-ris.h"
#include "ns3/thz-ntn-link-budget.h"
#include "ns3/thz-ntn-channel-model.h"

#include <cstdio>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("ThzNtnRisAssisted");

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

int
main(int argc, char* argv[])
{
    std::printf("[analytic-tool] Sections 1-4 drive the module's physics APIs directly (link\n"
                "budgets, scaling laws). The measured packet companion that follows carries\n"
                "real packets over an SGP4 pass with and without the RIS.\n\n");
    // ---- Default parameters ----
    double freq = 300e9;         // 300 GHz
    uint32_t risSize = 64;       // 64x64 default
    double altitude = 550.0;     // km
    double elevation = 30.0;     // degrees
    double txPower = 34.77;      // dBm (3W)
    double bandwidth = 10e9;     // 10 GHz
    double txGain = 40.0;        // dBi
    double rxGain = 42.0;        // dBi
    double noiseFigure = 10.0;   // dB

    // ---- Parse command-line arguments ----
    CommandLine cmd(__FILE__);
    cmd.AddValue("freq", "Carrier frequency in Hz [default: 300e9]", freq);
    cmd.AddValue("risSize", "RIS elements per side [default: 64]", risSize);
    cmd.AddValue("elevation", "Elevation angle in degrees [default: 30]", elevation);
    cmd.AddValue("txPower", "Satellite Tx power in dBm [default: 34.77]", txPower);
    bool companion = true;
    double companionGainDbi = 55.0;
    double companionBandwidthGhz = 1.0;
    double simSeconds = 20.0;
    double appRateMbps = 500.0;
    std::string outputDir = "thz-ntn-ris-assisted-out";
    cmd.AddValue("companion", "Run the measured packet companion (direct vs RIS)", companion);
    cmd.AddValue("companionGainDbi", "Companion aperture gain at both ends (dBi)",
                 companionGainDbi);
    cmd.AddValue("companionBandwidthGhz", "Companion channel bandwidth (GHz)",
                 companionBandwidthGhz);
    cmd.AddValue("simSeconds", "Companion run length (s)", simSeconds);
    cmd.AddValue("appRateMbps", "Companion offered 5QI 2 load (Mb/s)", appRateMbps);
    cmd.AddValue("outputDir", "Companion output directory", outputDir);
    cmd.Parse(argc, argv);

    std::cout << "=============================================================\n";
    std::cout << "  RIS-Assisted THz Satellite Link Analysis\n";
    std::cout << "=============================================================\n";
    std::cout << "  Frequency:    " << freq / 1e9 << " GHz\n";
    std::cout << "  Altitude:     " << altitude << " km\n";
    std::cout << "  Elevation:    " << elevation << " deg\n";
    std::cout << "  Tx Power:     " << txPower << " dBm\n";
    std::cout << "  Bandwidth:    " << bandwidth / 1e9 << " GHz\n";
    std::cout << "  RIS deploy:   Ground-based\n";
    std::cout << "-------------------------------------------------------------\n\n";

    // ---- Create nodes ----
    NodeContainer satNodes;
    satNodes.Create(1);

    NodeContainer gtNodes;
    gtNodes.Create(1);

    NodeContainer risNodes;
    risNodes.Create(1);

    // Set up mobility (local ENU frame, ground terminal at the origin).
    // Geometry fix (2026-06 audit): the satellite used to sit at zenith
    // (0, 0, h) regardless of the --elevation argument, so the cascaded
    // Sat->RIS->GT path loss disagreed with the parametric slant range. It
    // now sits ON the line of sight at the requested elevation:
    // |position - GT| = slantRange exactly, z ~= altitude (537 km up for a
    // 550 km orbit seen at 30 deg through the curved-Earth slant relation).
    double slantRange = ComputeSlantRange(elevation, altitude);
    double elevRad = elevation * M_PI / 180.0;
    Ptr<ConstantPositionMobilityModel> satMob = CreateObject<ConstantPositionMobilityModel>();
    satMob->SetPosition(Vector(slantRange * std::cos(elevRad), 0.0,
                               slantRange * std::sin(elevRad)));
    satNodes.Get(0)->AggregateObject(satMob);

    Ptr<ConstantPositionMobilityModel> gtMob = CreateObject<ConstantPositionMobilityModel>();
    gtMob->SetPosition(Vector(0.0, 0.0, 0.0));
    gtNodes.Get(0)->AggregateObject(gtMob);

    // RIS placed 200m from ground terminal
    double risDist = 200.0; // metres
    Ptr<ConstantPositionMobilityModel> risMob = CreateObject<ConstantPositionMobilityModel>();
    risMob->SetPosition(Vector(risDist, 0.0, 10.0)); // 10m height
    risNodes.Get(0)->AggregateObject(risMob);

    // ---- Create THz-NTN components ----
    Ptr<ThzNtnHelper> helper = CreateObject<ThzNtnHelper>();
    Ptr<ThzNtnLinkBudget> linkBudget = helper->CreateLinkBudget();

    // ---- Compute direct link (no RIS) baseline ----
    ThzNtnLinkBudget::LinkBudgetResult directResult = linkBudget->ComputeLinkBudget(
        ThzNtnLinkBudget::SAT_TO_GROUND,
        freq,
        slantRange,
        elevation,
        txPower,
        txGain,
        rxGain,
        bandwidth,
        noiseFigure);

    std::cout << "  1. Direct Link (No RIS) Baseline:\n";
    std::cout << "     Slant range:    " << std::fixed << std::setprecision(1)
              << slantRange / 1000.0 << " km\n";
    std::cout << "     Total path loss: " << directResult.totalPathLoss_dB << " dB\n";
    std::cout << "     SNR:             " << directResult.snr_dB << " dB\n";
    std::cout << "     Capacity:        " << std::setprecision(2)
              << directResult.shannonCapacity_Gbps << " Gbps\n\n";

    // ---- Vary RIS size and compute RIS-assisted performance ----
    std::vector<uint32_t> risSizes = {16, 32, 64, 128};

    std::cout << "  2. RIS-Assisted Performance vs Panel Size:\n";
    std::cout << "  " << std::string(82, '-') << "\n";
    std::cout << std::setw(10) << "RIS Size"
              << std::setw(12) << "Elements"
              << std::setw(12) << "N^2 Gain"
              << std::setw(14) << "Cascaded PL"
              << std::setw(12) << "Total SNR"
              << std::setw(12) << "SNR Improv"
              << std::setw(12) << "Capacity"
              << "\n";
    std::cout << std::setw(10) << "(NxN)"
              << std::setw(12) << "(N^2)"
              << std::setw(12) << "(dB)"
              << std::setw(14) << "(dB)"
              << std::setw(12) << "(dB)"
              << std::setw(12) << "(dB)"
              << std::setw(12) << "(Gbps)"
              << "\n";
    std::cout << "  " << std::string(82, '-') << "\n";

    // ntn-dataset@1: sections 2 and 3 as datasets next to the run record in outputDir,
    // so the results follow the parameters.
    std::filesystem::create_directories(outputDir);
    std::ofstream perfCsv(outputDir + "/thz-ntn-ris-assisted-performance.csv");
    perfCsv << "ris_side,elements,array_factor_gain_db,cascaded_path_loss_db,total_snr_db,"
               "snr_improvement_db,capacity_gbps\n";
    std::vector<ns3::ntnobs::DatasetMetric> metrics;
    metrics.push_back({"slant_range_km", std::to_string(slantRange / 1000.0), "analytic"});
    metrics.push_back(
        {"direct_path_loss_db", std::to_string(directResult.totalPathLoss_dB), "analytic"});
    metrics.push_back({"direct_snr_db", std::to_string(directResult.snr_dB), "analytic"});
    metrics.push_back({"direct_capacity_gbps",
                       std::to_string(directResult.shannonCapacity_Gbps),
                       "analytic"});

    for (uint32_t N : risSizes)
    {
        // Create RIS with this size
        Ptr<ThzNtnRis> ris = CreateObject<ThzNtnRis>();
        ris->Configure(N, N, freq, RisDeployment::GROUND);

        // Set optimal phase profile for satellite->RIS->GT
        ris->ComputeOptimalPhases(elevation, 0.0, 0.0, 0.0);

        // Array factor with perfect CSI (N^2 vs ONE element; not a link gain)
        double risGain = ris->ComputeArrayFactorGain_dB(true);

        // Cascaded path loss Sat->RIS->GT from the actual 3D node positions
        // (the model derives both hop distances from the MobilityModels).
        double cascadedPL = ris->ComputeCascadedPathLoss_dB(satMob, risMob, gtMob);

        // The RIS ray arrives (FSPL_direct - PL_cascaded) dB relative to the
        // direct ray and adds to it coherently-in-power (co-phased by the
        // optimal profile; power sum is the conservative form). The old
        // "direct SNR + 20 log10(N)" added the gain over ONE element as if it
        // were a gain over the direct link, ignoring the RIS->GT spreading
        // loss; at 200 m that overstated the improvement by tens of dB.
        const double risRelDb = directResult.fspl_dB - cascadedPL;
        double totalSnr = directResult.snr_dB +
                          10.0 * std::log10(1.0 + std::pow(10.0, risRelDb / 10.0));
        double snrImprovement = totalSnr - directResult.snr_dB;

        // Capacity with RIS
        double snrLinear = std::pow(10.0, totalSnr / 10.0);
        double capacity = bandwidth * std::log2(1.0 + snrLinear) / 1e9;

        uint32_t totalElements = N * N;

        std::cout << std::fixed << std::setprecision(1)
                  << std::setw(7) << N << "x" << N
                  << std::setw(12 - 2) << totalElements
                  << std::setw(12) << risGain
                  << std::setw(14) << cascadedPL
                  << std::setw(12) << totalSnr
                  << std::setw(12) << snrImprovement
                  << std::setprecision(2)
                  << std::setw(12) << capacity
                  << "\n";
        perfCsv << N << "," << totalElements << "," << risGain << "," << cascadedPL << ","
                << totalSnr << "," << snrImprovement << "," << capacity << "\n";
        if (N == 64)
        {
            metrics.push_back({"ris64_total_snr_db", std::to_string(totalSnr), "analytic"});
            metrics.push_back(
                {"ris64_snr_improvement_db", std::to_string(snrImprovement), "analytic"});
            metrics.push_back({"ris64_capacity_gbps", std::to_string(capacity), "analytic"});
        }
    }

    // ---- N^2 scaling law verification ----
    std::cout << "\n  3. N^2 Scaling Law Verification (Perfect CSI):\n";
    std::cout << "  " << std::string(58, '-') << "\n";
    std::cout << std::setw(10) << "N"
              << std::setw(12) << "N^2"
              << std::setw(16) << "10*log10(N^2)"
              << std::setw(16) << "Array factor"
              << "\n";
    std::cout << std::setw(10) << ""
              << std::setw(12) << ""
              << std::setw(16) << "(dB)"
              << std::setw(16) << "(dB)"
              << "\n";
    std::cout << "  " << std::string(58, '-') << "\n";

    std::ofstream scaleCsv(outputDir + "/thz-ntn-ris-assisted-scaling.csv");
    scaleCsv << "n,n_squared,theoretical_gain_db,array_factor_gain_db\n";
    for (uint32_t N : risSizes)
    {
        Ptr<ThzNtnRis> ris = CreateObject<ThzNtnRis>();
        ris->Configure(N, N, freq, RisDeployment::GROUND);

        // Array factor of the panel vs one element (the N^2 law); the link
        // improvement is the "SNR Improv" column of the table above.
        double measuredGain = ris->ComputeArrayFactorGain_dB(true);
        uint32_t N2 = N * N;
        double theoreticalGain = 10.0 * std::log10(static_cast<double>(N2));

        std::cout << std::fixed << std::setprecision(1)
                  << std::setw(10) << N
                  << std::setw(12) << N2
                  << std::setw(16) << theoreticalGain
                  << std::setw(16) << measuredGain
                  << "\n";
        scaleCsv << N << "," << N2 << "," << theoreticalGain << "," << measuredGain << "\n";
    }

    // ---- Quantization loss analysis ----
    std::cout << "\n  4. Phase Quantization Impact (64x64 RIS):\n";
    std::cout << "  " << std::string(40, '-') << "\n";
    std::cout << "  Continuous phase:  0.0 dB loss\n";

    Ptr<ThzNtnRis> ris64 = CreateObject<ThzNtnRis>();
    ris64->Configure(64, 64, freq, RisDeployment::GROUND);
    double qLoss = ris64->ComputeQuantizationLoss_dB();
    std::cout << "  Current config:    " << std::fixed << std::setprecision(2)
              << qLoss << " dB loss\n";

    // Near-field check
    bool nearField = ris64->IsInNearField(risDist);
    std::cout << "\n  RIS at " << risDist << " m from GT: "
              << (nearField ? "Near-field" : "Far-field") << " region\n";
    std::cout << "  Total RIS elements: " << ris64->GetTotalElements() << "\n";

    // Globe scene of the analytic geometry. The local-frame nodes above stay as they are;
    // device-less context nodes place the same geometry on the Earth: an SGP4 satellite of
    // a Walker shell at --altitude (ntn-constellation), the terminal on the parallel west of
    // its t = 0 sub-point where that satellite is seen at --elevation about the WGS-84
    // normal, and the RIS panel risDist east of the terminal, 10 m up.
    {
        ntncon::WalkerConfig w;
        w.num_planes = 72;
        w.total_sats = 72 * 22;
        w.altitude_km = altitude;
        w.inclination_deg = 53.0;
        w.epoch_unix_s = ntnobs::NtnSceneRecorder::kDefaultSceneEpochUnix;
        auto sceneSat = CreateObject<ntncon::Sgp4MobilityModel>();
        sceneSat->SetElementsSgp4(ntncon::WalkerConstellation::BuildDelta(w)[0]);
        const Vector satE = sceneSat->GetPosition();
        double subLat;
        double subLon;
        double subAlt;
        ntngeo::EcefToGeodetic(satE, subLat, subLon, subAlt);
        double lo = 0.0;
        double hi = 40.0;
        for (int it = 0; it < 60; ++it)
        {
            const double mid = 0.5 * (lo + hi);
            const double e =
                ntngeo::GeodeticElevationDeg(ntngeo::GeodeticToEcef(subLat, subLon - mid, 0.0),
                                             satE);
            (e > elevation ? lo : hi) = mid;
        }
        const double gtLon = subLon - 0.5 * (lo + hi);
        auto sceneGt = CreateObject<ConstantPositionMobilityModel>();
        sceneGt->SetPosition(ntngeo::GeodeticToEcef(subLat, gtLon, 0.0));
        const double dLon = risDist / (111195.0 * std::cos(subLat * M_PI / 180.0));
        auto sceneRis = CreateObject<ConstantPositionMobilityModel>();
        sceneRis->SetPosition(ntngeo::GeodeticToEcef(subLat, gtLon + dLon, 10.0));
        ntnobs::AddContextSatelliteNodes(std::vector<Ptr<ntncon::Sgp4MobilityModel>>{sceneSat});
        for (const Ptr<MobilityModel>& m :
             {Ptr<MobilityModel>(sceneGt), Ptr<MobilityModel>(sceneRis)})
        {
            Ptr<Node> n = CreateObject<Node>();
            n->AggregateObject(m);
        }
        ntnobs::StartSceneFromNodeList(outputDir);
    }

    // ---- Run simulation ----
    Simulator::Stop(Seconds(1.0));
    Simulator::Run();
    // ntn-dataset@1: standard manifest + sim_health for this analytic example
    // (plan/MASTER_PLAN_GLOBE_DATASET_E2E_2026-11.md).
    perfCsv.close();
    scaleCsv.close();
    metrics.push_back({"quantization_loss_db", std::to_string(qLoss), "analytic"});
    metrics.push_back({"ris_near_field", nearField ? "1" : "0", "analytic"});
    ns3::ntnobs::WriteDatasetRecord(outputDir,
                                    "thz-ntn-ris-assisted",
                                    "analytic",
                                    argc,
                                    argv,
                                    metrics);
    Simulator::Destroy();

    std::cout << "\n  Simulation complete.\n";
    std::cout << "=============================================================\n";

    if (companion)
    {
        thzkit::PassResult res[2];
        double lastRisDb = 0.0;
        for (int withRis = 0; withRis < 2; ++withRis)
        {
            thzkit::PassConfig pc;
            pc.runTag = withRis ? "ris-assisted-ris" : "ris-assisted-direct";
            pc.outputDir = outputDir;
            pc.simSeconds = simSeconds;
            pc.altitudeKm = altitude;
            pc.appRateMbps = appRateMbps;
            pc.argc = argc;
            pc.argv = argv;
            pc.link.freqHz = freq;
            pc.link.txPowerDbm = txPower;
            pc.link.spaceGainDbi = companionGainDbi;
            pc.link.groundGainDbi = companionGainDbi;
            pc.link.bandwidthHz = companionBandwidthGhz * 1e9;
            pc.link.noiseFigureDb = noiseFigure;
            Ptr<ThzNtnRis> ris = CreateObject<ThzNtnRis>();
            ris->Configure(risSize, risSize, freq, RisDeployment::GROUND);
            res[withRis] = thzkit::RunPass(
                pc,
                [&, withRis, ris](Ptr<thzntn::ThzNtnPacketLink> link,
                                  Ptr<MobilityModel> sat,
                                  Ptr<MobilityModel> gt) {
                    if (!withRis)
                    {
                        return;
                    }
                    // The panel: risDist m east of the terminal, 10 m up (ECEF).
                    const Vector gtE = ThzNtnLinkGeometry::PositionOf(PeekPointer(gt));
                    const Vector geo = ThzNtnLinkGeometry::EcefToGeodetic(gtE);
                    const double dLon = risDist / (111195.0 * std::cos(geo.x * M_PI / 180.0));
                    const Vector risE = ntngeo::GeodeticToEcef(geo.x, geo.y + dLon, geo.z + 10.0);
                    const double lambda = ris->GetWavelength();
                    double* sink = &lastRisDb;
                    auto update = std::make_shared<std::function<void()>>();
                    std::weak_ptr<std::function<void()>> self = update;
                    *update = [link, sat, gt, ris, risE, lambda, sink, self]() {
                        const Vector s = ThzNtnLinkGeometry::PositionOf(PeekPointer(sat));
                        const Vector g = ThzNtnLinkGeometry::PositionOf(PeekPointer(gt));
                        const double d = CalculateDistance(s, g);
                        const double fsplDirect = 20.0 * std::log10(4.0 * M_PI * d / lambda);
                        const double cascaded = ris->ComputeCascadedPathLoss_dB(s, risE, g);
                        const double rel = fsplDirect - cascaded;
                        const double gainDb = 10.0 * std::log10(1.0 + std::pow(10.0, rel / 10.0));
                        *sink = gainDb;
                        link->SetExtraLossDb(-gainDb);
                        if (auto f = self.lock())
                        {
                            Simulator::Schedule(MilliSeconds(100), [f]() { (*f)(); });
                        }
                    };
                    Simulator::ScheduleNow([update]() { (*update)(); });
                });
        }
        std::printf("# measured companion (%.0f GHz, %ux%u RIS at %.0f m): direct PDR %.5f "
                    "goodput %.2f Mb/s ACM %.1f Mb/s | RIS PDR %.5f goodput %.2f Mb/s ACM "
                    "%.1f Mb/s, last RIS contribution %.4f dB\n",
                    freq / 1e9, risSize, risSize, risDist, res[0].pdr, res[0].goodputMbps,
                    res[0].meanLineRateMbps, res[1].pdr, res[1].goodputMbps,
                    res[1].meanLineRateMbps, lastRisDb);
    }
    return 0;
}
