/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-only
 * Author: Muhammad Uzair
 *
 * Example: THz-NTN LEO-to-Ground TeraLink Scenario
 *
 * Demonstrates a single LEO satellite (550 km) to ground terminal link
 * at 225 GHz using the TeraLink preset.  Computes link budget at various
 * elevation angles and prints a detailed results table.
 *
 * Analysis-only example: parametric link-budget sweep plus a pass time
 * series, no measured radio. The elevation-sweep table is parametric by
 * design; the per-second pass time series is driven by a REAL SGP4 Walker
 * element (zenith at t=0, receding with genuine orbital dynamics) projected
 * into the local ENU frame — not the old triangular elevation profile.
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
class ThzNtnChannelModel;
class ThzNtnPhySat;
class ThzNtnPhyGround;
class ThzNtnLinkBudget;
class ThzNtnAntennaArray;
} // namespace ns3

#include "ns3/thz-ntn-helper.h"
#include "ns3/thz-ntn-link-budget.h"
#include "ns3/thz-ntn-phy-sat.h"
#include "ns3/thz-ntn-phy-ground.h"
#include "ns3/thz-ntn-antenna-array.h"
#include "ns3/thz-ntn-channel-model.h"

#include <cstdio>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("ThzNtnLeoGround");

/**
 * \brief Compute slant range from elevation angle and altitude.
 *
 * Uses the geometric relation for Earth-satellite slant distance:
 *   d = Re * (sqrt((h/Re + 1)^2 - cos^2(el)) - sin(el))
 *
 * \param elevDeg elevation angle in degrees
 * \param altKm satellite altitude in km
 * \return slant range in metres
 */
static double
ComputeSlantRange(double elevDeg, double altKm)
{
    const double Re = 6371.0; // Earth radius in km
    double elevRad = elevDeg * M_PI / 180.0;
    double ratio = (Re + altKm) / Re;
    double d_km = Re * (std::sqrt(ratio * ratio - std::cos(elevRad) * std::cos(elevRad))
                        - std::sin(elevRad));
    return d_km * 1000.0; // return in metres
}

namespace
{
double g_lastAppliedOwdMs = 0.0;
double g_lastAppliedPer = 0.0;
uint64_t g_couplingUpdates = 0;
constexpr uint32_t kNumThzUes = 8;
double g_perSum = 0.0;
double g_perMin = 1e9;
double g_perMax = -1e9;

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
    double freq = 225e9;       // 225 GHz
    double altitude = 550.0;   // km
    double txPower = 34.77;    // dBm (3 W)
    double bandwidth = 10e9;   // 10 GHz
    double txGain = 40.0;      // dBi (satellite UM-MIMO)
    double rxGain = 45.0;      // dBi (ground Cassegrain)
    double noiseFigure = 10.0; // dB
    double simTime = 60.0;     // seconds (v2 — event-driven pass)
    std::string outputDir = "thz-leo-ground-out";

    // ---- Parse command-line arguments ----
    CommandLine cmd(__FILE__);
    cmd.AddValue("freq", "Carrier frequency in Hz [default: 225e9]", freq);
    cmd.AddValue("altitude", "Satellite altitude in km [default: 550]", altitude);
    cmd.AddValue("txPower", "Transmit power in dBm [default: 34.77]", txPower);
    cmd.AddValue("bandwidth", "Channel bandwidth in Hz [default: 10e9]", bandwidth);
    cmd.AddValue("txGain", "Tx antenna gain in dBi [default: 40]", txGain);
    cmd.AddValue("rxGain", "Rx antenna gain in dBi [default: 45]", rxGain);
    cmd.AddValue("simTime", "Simulation time in seconds [default: 60]", simTime);
    cmd.AddValue("outputDir", "Output directory", outputDir);
    cmd.Parse(argc, argv);

    std::cout << "=============================================================\n";
    std::cout << "  THz-NTN LEO-to-Ground Link Analysis (TeraLink-225GHz)\n";
    std::cout << "=============================================================\n";
    std::cout << "  Frequency:    " << freq / 1e9 << " GHz\n";
    std::cout << "  Altitude:     " << altitude << " km\n";
    std::cout << "  Tx Power:     " << txPower << " dBm ("
              << std::pow(10.0, (txPower - 30.0) / 10.0) << " W)\n";
    std::cout << "  Bandwidth:    " << bandwidth / 1e9 << " GHz\n";
    std::cout << "  Tx Gain:      " << txGain << " dBi (UM-MIMO 32x32)\n";
    std::cout << "  Rx Gain:      " << rxGain << " dBi (Cassegrain 0.45m)\n";
    std::cout << "  Noise Figure: " << noiseFigure << " dB\n";
    std::cout << "-------------------------------------------------------------\n\n";

    // ---- Create nodes ----
    NodeContainer satNodes;
    satNodes.Create(1);

    NodeContainer gtNodes;
    gtNodes.Create(1);

    // ---- Set up mobility: REAL SGP4 orbit, ground at origin ----
    // The serving Walker element is at zenith at t=0 and recedes with genuine
    // orbital dynamics, projected into the local ENU frame at its sub-point.
    // Sanity: in ECEF |position| = Re + altitude (~6921 km for 550 km), so at
    // t=0 the ENU position is ~altitude*1000 m straight "up".
    ns3::ntncon::WalkerConfig wcfg;
    wcfg.num_planes = 1;
    wcfg.total_sats = 80;
    wcfg.altitude_km = altitude;
    wcfg.inclination_deg = 53.0;
    wcfg.epoch_unix_s = 1735689600.0;
    const auto satElements = ns3::ntncon::WalkerConstellation::BuildDelta(wcfg);
    Ptr<ns3::ntncon::Sgp4MobilityModel> satSgp4 =
        CreateObject<ns3::ntncon::Sgp4MobilityModel>();
    satSgp4->SetElements(satElements[0]);
    double subLat;
    double subLon;
    double subAlt;
    satSgp4->GetGeodetic(subLat, subLon, subAlt);
    Ptr<NtnEnuProjectionMobilityModel> satMob = CreateObject<NtnEnuProjectionMobilityModel>();
    satMob->SetSource(satSgp4);
    satMob->SetReference(subLat, subLon, 0.0);
    satNodes.Get(0)->AggregateObject(satMob);

    Ptr<ConstantPositionMobilityModel> gtMob = CreateObject<ConstantPositionMobilityModel>();
    gtMob->SetPosition(Vector(0.0, 0.0, 0.0));
    gtNodes.Get(0)->AggregateObject(gtMob);

    // ---- Create THz-NTN helper and components ----
    Ptr<ThzNtnHelper> helper = CreateObject<ThzNtnHelper>();

    // Create satellite PHY (CubeSat, regenerative, 3W)
    Ptr<ThzNtnPhySat> satPhy = helper->CreateSatellitePhy("TeraLink-225GHz");
    satPhy->SetSatelliteClass(SAT_CUBESAT);
    satPhy->SetPayloadMode(PAYLOAD_REGENERATIVE);

    // Create ground PHY (heterodyne, dual-circular polarization)
    Ptr<ThzNtnPhyGround> gtPhy = helper->CreateGroundPhy("TeraLink-225GHz");
    gtPhy->SetReceiverType(RX_HETERODYNE);
    gtPhy->SetPolarization(POL_DUAL_CIRCULAR);

    // Create channel model
    Ptr<ThzNtnChannelModel> channel = helper->CreateChannelModel("TeraLink-225GHz");
    channel->SetLinkType(ThzNtnChannelModel::SAT_TO_GROUND);
    channel->EnableMolecularAbsorption(true);
    channel->EnableWeatherEffects(true);
    channel->EnableScintillation(true);

    // Create link budget calculator
    Ptr<ThzNtnLinkBudget> linkBudget = helper->CreateLinkBudget();

    // ---- Evaluate at various elevation angles ----
    std::vector<double> elevations = {5.0, 10.0, 20.0, 30.0, 45.0, 60.0, 90.0};

    // Print header
    std::cout << std::setw(8) << "Elev"
              << std::setw(12) << "Range"
              << std::setw(10) << "FSPL"
              << std::setw(10) << "MolAbs"
              << std::setw(10) << "Weather"
              << std::setw(12) << "TotalPL"
              << std::setw(10) << "SNR"
              << std::setw(12) << "Capacity"
              << "\n";
    std::cout << std::setw(8) << "(deg)"
              << std::setw(12) << "(km)"
              << std::setw(10) << "(dB)"
              << std::setw(10) << "(dB)"
              << std::setw(10) << "(dB)"
              << std::setw(12) << "(dB)"
              << std::setw(10) << "(dB)"
              << std::setw(12) << "(Gbps)"
              << "\n";
    std::cout << std::string(84, '-') << "\n";

    for (double elev : elevations)
    {
        double range_m = ComputeSlantRange(elev, altitude);
        double range_km = range_m / 1000.0;

        // Compute full link budget
        ThzNtnLinkBudget::LinkBudgetResult result = linkBudget->ComputeLinkBudget(
            ThzNtnLinkBudget::SAT_TO_GROUND,
            freq,
            range_m,
            elev,
            txPower,
            txGain,
            rxGain,
            bandwidth,
            noiseFigure);

        std::cout << std::fixed << std::setprecision(1)
                  << std::setw(8) << elev
                  << std::setw(12) << range_km
                  << std::setw(10) << result.fspl_dB
                  << std::setw(10) << result.molecularAbsorption_dB
                  << std::setw(10) << result.weatherLoss_dB
                  << std::setw(12) << result.totalPathLoss_dB
                  << std::setw(10) << result.snr_dB
                  << std::setprecision(2)
                  << std::setw(12) << result.shannonCapacity_Gbps
                  << "\n";
    }

    std::cout << "\n-------------------------------------------------------------\n";
    std::cout << "  Shannon capacity: C = B * log2(1 + SNR)\n";
    std::cout << "  Path loss includes: FSPL + molecular absorption + weather\n";
    std::cout << "                      + scintillation + pointing error\n";
    std::cout << "-------------------------------------------------------------\n";

    // ---- Show detailed budget for 20-degree elevation ----
    double refElev = 20.0;
    double refRange = ComputeSlantRange(refElev, altitude);
    ThzNtnLinkBudget::LinkBudgetResult detailResult = linkBudget->ComputeLinkBudget(
        ThzNtnLinkBudget::SAT_TO_GROUND,
        freq,
        refRange,
        refElev,
        txPower,
        txGain,
        rxGain,
        bandwidth,
        noiseFigure);

    std::cout << "\n  Detailed Link Budget at " << refElev << " deg elevation:\n";
    std::cout << "    Tx Power:                " << detailResult.txPower_dBm << " dBm\n";
    std::cout << "    Tx Antenna Gain:         " << detailResult.txAntennaGain_dBi << " dBi\n";
    std::cout << "    EIRP:                    " << detailResult.eirp_dBm << " dBm\n";
    std::cout << "    FSPL:                    " << detailResult.fspl_dB << " dB\n";
    std::cout << "    Molecular Absorption:    " << detailResult.molecularAbsorption_dB << " dB\n";
    std::cout << "    Weather Loss:            " << detailResult.weatherLoss_dB << " dB\n";
    std::cout << "    Scintillation Loss:      " << detailResult.scintillationLoss_dB << " dB\n";
    std::cout << "    Pointing Loss:           " << detailResult.pointingLoss_dB << " dB\n";
    std::cout << "    Total Path Loss:         " << detailResult.totalPathLoss_dB << " dB\n";
    std::cout << "    Rx Antenna Gain:         " << detailResult.rxAntennaGain_dBi << " dBi\n";
    std::cout << "    Rx Power:                " << detailResult.rxPower_dBm << " dBm\n";
    std::cout << "    Noise Power:             " << detailResult.noisePower_dBm << " dBm\n";
    std::cout << "    SNR:                     " << detailResult.snr_dB << " dB\n";
    std::cout << "    HW-limited SNR:          " << detailResult.hardwareLimitedSnr_dB << " dB\n";
    std::cout << "    Shannon Capacity:        " << detailResult.shannonCapacity_Gbps << " Gbps\n";
    std::cout << "    HW-limited Capacity:     " << detailResult.hardwareLimitedCapacity_Gbps
              << " Gbps\n";
    std::cout << "    Link Margin:             " << detailResult.linkMargin_dB << " dB\n";

    // ---- Doppler analysis (geometric, freq-scaled radial velocity) ----
    double satVelocity = 7600.0; // m/s for 550 km LEO
    std::cout << "\n  Doppler Pre-Compensation Analysis:\n";
    for (double elev : elevations)
    {
        double vRadial = satVelocity * std::cos(elev * M_PI / 180.0);
        double dopplerHz = vRadial / 299792458.0 * freq;
        std::cout << "    Elev " << std::setw(4) << elev << " deg: Doppler offset = "
                  << std::setprecision(0) << dopplerHz / 1e3 << " kHz\n";
    }

    // ---- Real packet plane + per-second pass-geometry sampler (v2) ----
    NtnRealisticTrafficHelper traffic;
    traffic.SetSimTime(Seconds(simTime));
    traffic.SetOutputDir(outputDir);
    traffic.SetRunTag("thz-ntn-leo-ground");
    traffic.SetProfile(NtnRealisticTrafficHelper::TrafficProfile::EmbbStreaming);
    traffic.InstallUes(kNumThzUes);

    std::filesystem::create_directories(outputDir);
    std::ofstream passCsv(outputDir + "/pass_timeseries.csv");
    passCsv << "time_s,elev_deg,range_km,fspl_dB,molabs_dB,weather_dB,scint_dB,"
               "pointing_dB,total_loss_dB,snr_dB,shannon_Gbps\n";

    traffic.RegisterPeriodicCallback(Seconds(1.0), [&](Time nowT) {
        double t = nowT.GetSeconds();
        // REAL LEO pass: elevation and slant range from the live SGP4 (ENU)
        // geometry.  The budget input is floored at 5 deg elevation to keep
        // the atmospheric models defined near the horizon.
        const Vector g = gtMob->GetPosition();
        const Vector s = satMob->GetPosition();
        const double dx = s.x - g.x;
        const double dy = s.y - g.y;
        const double dz = s.z - g.z;
        const double horiz = std::max(std::sqrt(dx * dx + dy * dy), 1e-3);
        double elev = std::atan2(dz, horiz) * 180.0 / M_PI;
        double range_m = std::sqrt(dx * dx + dy * dy + dz * dz);
        elev = std::max(5.0, elev);
        auto r = linkBudget->ComputeLinkBudget(ThzNtnLinkBudget::SAT_TO_GROUND,
            freq, range_m, elev, txPower, txGain, rxGain, bandwidth, noiseFigure);
        passCsv << std::fixed << std::setprecision(2)
                << t << "," << elev << "," << range_m/1000.0 << ","
                << r.fspl_dB << "," << r.molecularAbsorption_dB << ","
                << r.weatherLoss_dB << "," << r.scintillationLoss_dB << ","
                << r.pointingLoss_dB << "," << r.totalPathLoss_dB << ","
                << r.snr_dB << "," << r.shannonCapacity_Gbps << "\n";

        // WF-06: APPLY the physics to the data plane.
        //
        // Everything above was computed and printed. The packets meanwhile
        // crossed a point-to-point star with a hardcoded 15 ms delay and a
        // RateErrorModel pinned at 0.0, so a run in a rainstorm at 5 degrees
        // elevation delivered exactly what a clear zenith pass did. The helper
        // has had the coupling hook all along and nothing called it.
        //
        // The delay is the real slant range over c, replacing the constant. The
        // error rate comes from the SNR this budget just produced.
        const Time owd = Seconds(range_m / 299792458.0);
        const double per = PerFromSnrDb(r.snr_dB);
        // Every UE shares this satellite link, so every UE gets the update.
        // Updating only index 0 left the other seven on the hardcoded 15 ms
        // and PER 0, which is why rx/tx stayed at 1.
        for (uint32_t u = 0; u < kNumThzUes; ++u)
        {
            traffic.UpdateUeLink(u, owd, per);
        }
        g_lastAppliedOwdMs = owd.GetSeconds() * 1e3;
        g_lastAppliedPer = per;
        ++g_couplingUpdates;
        g_perSum += per;
        g_perMin = std::min(g_perMin, per);
        g_perMax = std::max(g_perMax, per);
    });
    traffic.Wire();
    Simulator::Stop(Seconds(simTime + 0.5));
    Simulator::Run();
    traffic.WriteHealthReport();
    std::cout << "  [WF-06] THz physics -> data plane: " << g_couplingUpdates
              << " link updates applied; last one-way delay " << g_lastAppliedOwdMs
              << " ms (was a hardcoded 15 ms), last PER " << g_lastAppliedPer
              << " (was pinned at 0); PER over the pass min=" << g_perMin << " max=" << g_perMax << " mean=" << (g_couplingUpdates ? g_perSum/g_couplingUpdates : 0.0) << "\n";
    if (g_lastAppliedPer > 0.5)
    {
        std::cout << "  [WF-06] LINK DOES NOT CLOSE at these defaults: the computed SNR is below\n"
                     "          any usable MODCOD threshold, so the data plane correctly delivers\n"
                     "          (almost) nothing. This was invisible while the error rate was\n"
                     "          pinned at 0 and every packet arrived regardless of the physics.\n"
                     "          The dominant term is the noise bandwidth: --bandwidth=10e9 puts the\n"
                     "          thermal floor at -174 + 100 dB. Narrowing it closes the link.\n";
    }
    passCsv.close();
    Simulator::Destroy();

    std::cout << "\n  Simulation complete.\n";
    std::cout << "=============================================================\n";

    return 0;
}
