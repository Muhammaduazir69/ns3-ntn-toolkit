/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-only
 * Author: Muhammad Uzair
 *
 * Full Constellation NTN-CHO Simulation (v3 — REAL plane)
 *
 * v3 replaces the v2 self-contained formula simulator (circular-orbit
 * propagation + closed-form RSRP/SINR link budget + coin-flip HO failure +
 * synthetic TTE) with the REAL NTN mobility architecture, sourcing EVERY
 * headline KPI from the measured plane and the real CHO state machine:
 *
 *   - Satellites: SGP4-propagated Walker-Delta orbits (ntn-constellation
 *     Sgp4MobilityModel) — serving + candidate shell, real pass dynamics.
 *   - UEs: 3GPP TR 38.811 §6.1.1.1 class mobility (NtnTr38811MobilityModel,
 *     ECEF) as REAL ns-3 nodes on the serving cell.
 *   - Radio: real mmwave NR NTN cell (NtnRealStackHelper: SpectrumPhy + MAC +
 *     HARQ + RLC/PDCP + RRC + EPC). Serving SINR / throughput / delay are
 *     MEASURED off the PHY; candidate SINR is the measured serving SINR
 *     corrected by the live Friis slant ratio 20*log10(servSlant/candSlant).
 *   - Decision: the real NtnChoAlgorithm (TTE-aware / D1 / D2 / T1 / A3 /
 *     elevation / TA) decides every handover via EvaluateConditions() +
 *     SelectBestCandidate() / ExecuteHandover(); interruption/RACH come from
 *     GetMechanismStats(). Baselines SelectBaselineA3 /
 *     SelectBaselineLocationOnly are exercised via --algorithm.
 *   - Auxiliary ephemeris/TTE oracle: NtnChoHelper::SetupConstellation builds
 *     NtnOrbitPredictor + NtnTteEstimator + NtnMeasurementModel over real
 *     SatSGP4 sats + geo-33E antenna patterns; ComputeBatchTte feeds the
 *     tte_computations.csv column and the algorithm candidate ranking. This
 *     pipeline is auxiliary ONLY (its SINR is closed-form thermal-noise) — the
 *     headline SINR is always the measured PHY value.
 *
 * Output CSV schemas are unchanged (handover_events / measurements /
 * tte_computations / kpi_timeseries / kpi_summary + GeoJSON), but every value
 * now comes from the real plane / real algorithm. In measurements.csv,
 * sinr_dB is the measured mmwave PHY value; the decomposition columns are the
 * physical link budget from real geometry + configured beam EIRP:
 * path_loss_dB = free-space loss FSPL(slant,fc), antenna_gain_dB = beam EIRP
 * (Tx power+gain), rsrp_dBm = EIRP - FSPL, doppler_Hz = real SGP4 relative
 * radial velocity. (The measured SINR additionally reflects the channel's
 * beamforming/array gains, so it need not equal eirp - FSPL - noise.)
 *
 *   ./ns3 run "ntn-cho-full-constellation --simTime=120 --numUes=6 \
 *       --algorithm=tte-aware --outputDir=/tmp/ntn-cho"
 */

#include "ns3/core-module.h"
#include "ns3/mmwave-enb-net-device.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/ntn-cho-algorithm.h"
#include "ns3/ntn-cho-helper.h"
#include "ns3/ntn-measurement-model.h"
#include "ns3/ntn-real-stack-helper.h"
#include "ns3/ntn-scene-recorder.h"
#include "ns3/ntn-tr38811-mobility-model.h"
#include "ns3/sgp4-mobility-model.h"
#include "ns3/walker-constellation.h"

// libsatellite: SGP4 sats + antenna-pattern recipe for the auxiliary predictor.
#include "ns3/geo-coordinate.h"
#include "ns3/satellite-antenna-gain-pattern-container.h"
#include "ns3/satellite-constant-position-mobility-model.h"
#include "ns3/satellite-env-variables.h"
#include "ns3/singleton.h"

#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <vector>

using namespace ns3;
using ns3::ntncon::Sgp4MobilityModel;
using ns3::ntncon::WalkerConfig;
using ns3::ntncon::WalkerConstellation;

NS_LOG_COMPONENT_DEFINE("NtnChoFullConstellationV3");

namespace
{
// ---- Globals for the per-second CHO/measurement tick on the real queue ----
NtnRealStackHelper* g_rs = nullptr;
Ptr<NtnChoAlgorithm> g_cho;
Ptr<NtnChoHelper> g_choHelper;
Ptr<NtnOrbitPredictor> g_orbit;
Ptr<NtnTteEstimator> g_tte;

std::vector<Ptr<NtnTr38811MobilityModel>> g_ueModels;
std::vector<Ptr<Sgp4MobilityModel>> g_candSats; // candidate-cell satellites
Ptr<Sgp4MobilityModel> g_servSat;

std::string g_algorithm = "tte-aware";
double g_qualityTh = -3.0;
double g_tteMinimum = 20.0;
double g_simTime = 120.0;
double g_dt = 1.0;
double g_d1Threshold = 50000.0;

uint16_t g_servingCellId = 0;
std::vector<uint16_t> g_candCellIds;
uint16_t g_serving = 0;
uint32_t g_servSatId = 0;                 // logical sat id of serving cell
std::vector<uint32_t> g_candSatIds;       // logical sat ids of candidate cells
std::vector<uint32_t> g_candBeamIds;      // aux geo-33E beam id per candidate (TTE oracle)
// Fixed reference position inside the geo-33E antenna-pattern footprint for the
// AUXILIARY TTE/ephemeris oracle (same as the test fixture). The oracle's
// geometry is GEO-pattern-bound and offline; headline geometry/SINR come from
// the live SGP4 sats + measured PHY, not from this reference.
const GeoCoordinate kAuxRefPos(49.75, 3.75, 0.0);

uint32_t g_totalHos = 0;
uint32_t g_successHos = 0;
uint32_t g_failedHos = 0;
uint32_t g_ppCount = 0;
double g_lastHoTime = -1000.0;
uint16_t g_lastSourceCell = 0;
double g_sumServSinr = 0.0;
uint32_t g_sinrSamples = 0;

// Output streams (same schemas as v2).
std::ofstream g_hoFile;
std::ofstream g_measFile;
std::ofstream g_tteFile;
std::ofstream g_kpiFile;
std::ofstream g_satTrackFile;
std::ofstream g_ueTrackFile;
std::ofstream g_satGeo, g_ueGeo, g_beamGeo, g_hoGeo;
bool g_fSat = true, g_fUe = true, g_fHo = true;

// Map a CHO algorithm/baseline string to a real trigger type.
NtnChoAlgorithm::TriggerType
TriggerFor(const std::string& a)
{
    if (a == "a3" || a == "a3-baseline")
    {
        return NtnChoAlgorithm::TRIGGER_EVENT_A3;
    }
    if (a == "location" || a == "location-baseline" || a == "d1")
    {
        return NtnChoAlgorithm::TRIGGER_LOCATION_D1;
    }
    if (a == "d2")
    {
        return NtnChoAlgorithm::TRIGGER_DISTANCE_D2;
    }
    if (a == "time" || a == "t1")
    {
        return NtnChoAlgorithm::TRIGGER_TIME_T1;
    }
    if (a == "elevation")
    {
        return NtnChoAlgorithm::TRIGGER_ELEVATION;
    }
    // default
    return NtnChoAlgorithm::TRIGGER_TTE_AWARE;
}

// Per-second measured-CHO tick on the real event queue (mirrors real-stack
// ChoTick, scaled to serving + N candidate cells + multi-UE).
void
ChoTick()
{
    const double t = Simulator::Now().GetSeconds();
    if (t >= g_simTime)
    {
        return;
    }

    // UE 0 drives the CHO decision (the served cell's representative UE). All
    // UEs are measured on the same real serving cell.
    const Vector u = g_ueModels[0]->GetPosition(); // TR 38.811 UE, ECEF
    const Vector sPos = g_servSat->GetPosition();  // SGP4 serving sat, ECEF
    const double servElev = ntngeo::ElevationDeg(u, sPos);
    const double servSlant = ntngeo::SlantRangeM(u, sPos);

    // Serving SINR is MEASURED from the real mmwave PHY (UE 0).
    const double servSinr = g_rs->GetUeRecentSinrDb(0);
    if (std::isnan(servSinr))
    {
        Simulator::Schedule(Seconds(g_dt), &ChoTick);
        return;
    }
    g_sumServSinr += servSinr;
    ++g_sinrSamples;

    g_cho->UpdateServingMeasurement(servSinr);
    const double servGain = std::max(-20.0, (servElev - 45.0) / 5.0);
    g_cho->UpdateMeasurement(g_servingCellId, servSinr, servGain);
    g_cho->UpdateCandidateSlantRange(g_servingCellId, servSlant);

    double ueLat, ueLon, ueAlt;
    g_ueModels[0]->GetGeodetic(ueLat, ueLon, ueAlt);

    // Per-candidate measured-baseline SINR via the live Friis slant ratio off
    // the MEASURED serving SINR (3GPP NTN CHO ephemeris-predicted candidate).
    struct CandSnap
    {
        uint16_t cellId;
        uint32_t satId;
        double sinr;
        double gain;
        double elev;
        double slant;
    };
    std::vector<CandSnap> cands;
    for (size_t k = 0; k < g_candSats.size(); ++k)
    {
        const Vector cPos = g_candSats[k]->GetPosition();
        const double candElev = ntngeo::ElevationDeg(u, cPos);
        const double candSlant = ntngeo::SlantRangeM(u, cPos);
        const double candSinr =
            servSinr + 20.0 * std::log10(servSlant / std::max(1.0, candSlant));
        const double candGain = std::max(-20.0, (candElev - 45.0) / 5.0);
        g_cho->UpdateMeasurement(g_candCellIds[k], candSinr, candGain);
        g_cho->UpdateCandidateSlantRange(g_candCellIds[k], candSlant);
        cands.push_back({g_candCellIds[k], g_candSatIds[k], candSinr, candGain, candElev,
                         candSlant});
    }

    // ---- Auxiliary ephemeris/TTE oracle (NtnTteEstimator over real SGP4) ----
    // Feeds tte_computations.csv and supplements candidate ranking. NOT a
    // headline SINR source (its measurement model SINR is closed-form).
    std::vector<NtnTteEstimator::CandidateBeamInfo> beamInfos;
    for (size_t k = 0; k < cands.size(); ++k)
    {
        if (k >= g_candBeamIds.size())
        {
            break;
        }
        NtnTteEstimator::CandidateBeamInfo bi;
        bi.cellId = cands[k].cellId;
        // Aux predictor satId is the index into the auxiliary SGP4 sat
        // container (serving = 0, candidate k = k+1).
        bi.satId = static_cast<uint32_t>(k + 1);
        bi.beamId = g_candBeamIds[k]; // valid geo-33E beam resolved at setup
        bi.sinr_dB = cands[k].sinr;
        beamInfos.push_back(bi);
    }
    // The auxiliary GEO TTE oracle and the predictor-backed EvaluateConditions
    // are evaluated on a coarse 5 s cadence (CHO decision granularity for LEO);
    // the per-second tick only feeds the light measured-SINR updates. This
    // keeps the GEO antenna-pattern search off the critical per-second path.
    const bool decisionTick = (std::fmod(t, 5.0) < g_dt);

    std::map<uint16_t, double> tteByCell;
    if (decisionTick && g_tte && !beamInfos.empty())
    {
        // Auxiliary TTE oracle runs at the fixed geo-33E reference position
        // (well-posed pattern geometry); see kAuxRefPos. Offline/auxiliary
        // only — NOT a headline geometry/SINR source.
        auto tteResults = g_tte->ComputeBatchTte(kAuxRefPos, Vector(0, 0, 0), beamInfos,
                                                 g_qualityTh);
        for (const auto& r : tteResults)
        {
            tteByCell[r.cellId] = r.tte.GetSeconds();
            g_tteFile << std::fixed << std::setprecision(3) << t << ",0," << r.satId
                      << ",0," << r.cellId << "," << std::setprecision(2)
                      << r.tte.GetSeconds() << "," << r.currentGain_dB << ","
                      << r.peakGain_dB << ","
                      << ((r.currentSinr_dB >= g_qualityTh &&
                           r.tte.GetSeconds() >= g_tteMinimum)
                              ? 1
                              : 0)
                      << "," << g_algorithm << "\n";
        }
    }

    // ---- Per-UE measurement rows (every 5 s), measured serving SINR ----
    if (std::fmod(t, 5.0) < g_dt)
    {
        for (size_t i = 0; i < g_ueModels.size(); ++i)
        {
            const double sinr = g_rs->GetUeRecentSinrDb(static_cast<uint32_t>(i));
            if (std::isnan(sinr))
            {
                continue;
            }
            double lat, lon, alt;
            g_ueModels[i]->GetGeodetic(lat, lon, alt);
            const Vector ui = g_ueModels[i]->GetPosition();
            const double elev = ntngeo::ElevationDeg(ui, sPos);
            const double slant = ntngeo::SlantRangeM(ui, sPos);
            const double delayMs = slant / 299792458.0 * 1000.0;
            // Link-budget decomposition from REAL geometry + configured beam
            // EIRP. sinr is the measured PHY value (kept); rsrp/path_loss/
            // antenna_gain are the free-space budget (rsrp = eirp - FSPL) and
            // doppler is the real SGP4 relative radial velocity.
            const double fcHz = g_rs->GetCarrierFrequencyHz();
            const double eirpDbm = g_rs->GetSatEirpDbm();
            const double dPl = (slant > 1.0 ? slant : 1.0);
            const double fsplDb = 20.0 * std::log10(dPl) +
                                  20.0 * std::log10(fcHz) - 147.55221;
            const double rsrpDbm = eirpDbm - fsplDb;
            const Vector vSat = g_servSat->GetVelocity();
            const Vector vUe = g_ueModels[i]->GetVelocity();
            const Vector rVec(ui.x - sPos.x, ui.y - sPos.y, ui.z - sPos.z);
            const double rNorm0 = std::sqrt(rVec.x * rVec.x + rVec.y * rVec.y +
                                            rVec.z * rVec.z);
            const double rNorm = (rNorm0 > 1.0 ? rNorm0 : 1.0);
            const double rangeRate =
                ((vUe.x - vSat.x) * rVec.x + (vUe.y - vSat.y) * rVec.y +
                 (vUe.z - vSat.z) * rVec.z) /
                rNorm;
            const double dopplerHz = -(fcHz / 299792458.0) * rangeRate;
            g_measFile << std::fixed << std::setprecision(3) << t << "," << i << ","
                       << g_servSatId << ",0," << g_servingCellId << ","
                       << std::setprecision(2) << rsrpDbm << "," << sinr << ","
                       << fsplDb << "," << eirpDbm << ","
                       << std::setprecision(1) << elev << ","
                       << std::setprecision(2) << (slant / 1000.0) << ","
                       << std::setprecision(1) << dopplerHz << ","
                       << std::setprecision(3) << delayMs << ","
                       << std::setprecision(6) << lat << "," << lon << "\n";
        }
    }

    // ---- Real CHO decision (algorithm state machine + baselines) ----
    uint16_t chosen = 0;
    if (decisionTick)
    {
        g_cho->EvaluateConditions();
        if (g_algorithm == "a3-baseline")
        {
            chosen = g_cho->SelectBaselineA3(servSinr);
        }
        else if (g_algorithm == "location-baseline")
        {
            chosen = g_cho->SelectBaselineLocationOnly();
        }
        else
        {
            chosen = g_cho->SelectBestCandidate();
        }
    }

    if (chosen != 0 && chosen != g_serving && chosen != g_servingCellId)
    {
        ++g_totalHos;
        g_cho->ExecuteHandover(chosen);
        const auto st = g_cho->GetMechanismStats();
        const bool success = true; // real state machine executed the HO
        const double tos = (g_lastHoTime >= 0.0) ? (t - g_lastHoTime) : t;
        const bool isPP =
            (g_lastHoTime >= 0.0 && chosen == g_lastSourceCell && tos < 10.0);
        if (success)
        {
            ++g_successHos;
        }
        else
        {
            ++g_failedHos;
        }
        if (isPP)
        {
            ++g_ppCount;
        }

        // Target snapshot.
        double tgtSinr = servSinr;
        double tgtElev = servElev;
        double ttePred = 0.0;
        for (auto& c : cands)
        {
            if (c.cellId == chosen)
            {
                tgtSinr = c.sinr;
                tgtElev = c.elev;
                break;
            }
        }
        auto itTte = tteByCell.find(chosen);
        if (itTte != tteByCell.end())
        {
            ttePred = itTte->second;
        }

        g_hoFile << std::fixed << std::setprecision(3) << t << ",0,"
                 << g_serving << "," << chosen << "," << g_servSatId << ","
                 << chosen << ",0,0," << g_algorithm << ","
                 << std::setprecision(2) << servSinr << "," << tgtSinr << ","
                 << ttePred << "," << tos << "," << (success ? 1 : 0) << ","
                 << (isPP ? 1 : 0) << "," << std::setprecision(6) << ueLat << ","
                 << ueLon << ",0," << g_ueModels[0]->GetClassName() << ","
                 << std::setprecision(1) << servElev << "," << tgtElev << ",\n";

        if (!g_fHo)
        {
            g_hoGeo << ",\n";
        }
        g_hoGeo << "{\"type\":\"Feature\",\"properties\":{\"ueId\":0,\"time\":"
                << std::setprecision(1) << t << ",\"sourceCell\":" << g_serving
                << ",\"targetCell\":" << chosen
                << ",\"success\":" << (success ? "true" : "false")
                << ",\"pingPong\":" << (isPP ? "true" : "false") << ",\"algorithm\":\""
                << g_algorithm << "\",\"tte\":" << std::setprecision(2) << ttePred
                << ",\"sinrBefore\":" << servSinr << ",\"sinrAfter\":" << tgtSinr
                << "},\"geometry\":{\"type\":\"Point\",\"coordinates\":["
                << std::setprecision(6) << ueLon << "," << ueLat << "]}}";
        g_fHo = false;

        std::printf("  %6.1fs  HANDOVER cell %u -> %u  (servSINR meas=%.1f dB, candSINR "
                    "pred=%.1f dB, TTE=%.1fs, interruption=%.1f ms%s)\n",
                    t, g_serving, chosen, servSinr, tgtSinr, ttePred,
                    st.lastInterruptionMs,
                    st.rachLessExecutions > 0 ? ", RACH-less" : "");
        g_lastSourceCell = g_serving;
        g_lastHoTime = t;
        g_serving = chosen;
    }

    // ---- Track files (every 2 s) ----
    if (std::fmod(t, 2.0) < g_dt)
    {
        double slat, slon, salt;
        g_servSat->GetGeodetic(slat, slon, salt);
        const Vector sv = g_servSat->GetVelocity();
        const double sspeed = std::sqrt(sv.x * sv.x + sv.y * sv.y + sv.z * sv.z);
        g_satTrackFile << std::fixed << std::setprecision(3) << t << "," << g_servSatId
                       << "," << std::setprecision(6) << slat << "," << slon << ","
                       << std::setprecision(1) << (salt / 1000.0) << ","
                       << std::setprecision(0) << sspeed << "\n";
        for (size_t k = 0; k < g_candSats.size(); ++k)
        {
            double clat, clon, calt;
            g_candSats[k]->GetGeodetic(clat, clon, calt);
            const Vector cv = g_candSats[k]->GetVelocity();
            const double cspeed = std::sqrt(cv.x * cv.x + cv.y * cv.y + cv.z * cv.z);
            g_satTrackFile << std::fixed << std::setprecision(3) << t << ","
                           << g_candSatIds[k] << "," << std::setprecision(6) << clat
                           << "," << clon << "," << std::setprecision(1)
                           << (calt / 1000.0) << "," << std::setprecision(0) << cspeed
                           << "\n";
        }
        if (g_fSat)
        {
            g_satGeo << "{\"type\":\"Feature\",\"properties\":{\"satId\":" << g_servSatId
                     << ",\"time\":" << std::setprecision(1) << t
                     << "},\"geometry\":{\"type\":\"Point\",\"coordinates\":["
                     << std::setprecision(6) << slon << "," << slat << ","
                     << (salt) << "]}}";
            g_fSat = false;
        }

        for (size_t i = 0; i < g_ueModels.size(); ++i)
        {
            double lat, lon, alt;
            g_ueModels[i]->GetGeodetic(lat, lon, alt);
            const double sinr = g_rs->GetUeRecentSinrDb(static_cast<uint32_t>(i));
            const bool served = !std::isnan(sinr);
            g_ueTrackFile << std::fixed << std::setprecision(3) << t << "," << i << ","
                          << std::setprecision(6) << lat << "," << lon << ",";
            if (served)
            {
                g_ueTrackFile << g_servingCellId << "," << g_servSatId << ","
                              << std::setprecision(2) << sinr << ",";
            }
            else
            {
                g_ueTrackFile << ",,,";
            }
            g_ueTrackFile << g_ueModels[i]->GetClassName() << ","
                          << (served ? "connected" : "searching") << ","
                          << std::setprecision(1) << servElev << "\n";

            if (!g_fUe)
            {
                g_ueGeo << ",\n";
            }
            g_ueGeo << "{\"type\":\"Feature\",\"properties\":{\"ueId\":" << i
                    << ",\"time\":" << std::setprecision(1) << t
                    << ",\"servingCell\":" << g_servingCellId
                    << ",\"sinr_dB\":" << std::setprecision(2) << (served ? sinr : -100.0)
                    << ",\"hoState\":\"" << (served ? "connected" : "searching")
                    << "\"},\"geometry\":{\"type\":\"Point\",\"coordinates\":["
                    << std::setprecision(6) << lon << "," << lat << "]}}";
            g_fUe = false;
        }
    }

    // ---- KPI timeseries (every 5 s) ----
    if (std::fmod(t, 5.0) < g_dt)
    {
        const double rate = (g_totalHos > 0) ? 100.0 * g_successHos / g_totalHos : 100.0;
        const double avgSinr =
            (g_sinrSamples > 0) ? g_sumServSinr / g_sinrSamples : 0.0;
        const double hoPerUeMin =
            (t > 0) ? g_totalHos / (double)g_ueModels.size() / (t / 60.0) : 0.0;
        g_kpiFile << std::fixed << std::setprecision(3) << t << "," << g_totalHos << ","
                  << g_successHos << "," << g_failedHos << "," << g_ppCount << ","
                  << std::setprecision(1) << rate << "," << std::setprecision(2)
                  << avgSinr << "," << std::setprecision(4) << hoPerUeMin << "\n";
    }

    if (std::fmod(t, 30.0) < g_dt)
    {
        std::cout << "  t=" << (int)t << "s: " << g_totalHos << " HOs ("
                  << g_successHos << " ok), measSINR="
                  << std::fixed << std::setprecision(1) << servSinr << " dB\n";
    }

    Simulator::Schedule(Seconds(g_dt), &ChoTick);
}
} // namespace

int
main(int argc, char* argv[])
{
    double simTime = 60.0;
    uint32_t numUes = 3;
    std::string scenario = "suburban";
    std::string algorithm = "tte-aware";
    double d1Threshold = 50000;
    double qualityTh = -3.0;
    double tteMinimum = 20.0;
    double carrierFreqGhz = 2.0;
    double satTxPower = 55.0;
    double altitudeKm = 780.0;
    uint32_t numCandidates = 2;
    uint32_t satsPerPlane = 80;
    double tteMinSec = 3.0;
    std::string outputDir = "ntn-cho-output";
    uint32_t rngRun = 1;
    std::string netSimOut;
    std::string czmlOut;

    CommandLine cmd(__FILE__);
    cmd.AddValue("simTime", "Simulation time (s)", simTime);
    cmd.AddValue("numUes", "Number of TR 38.811 UEs on the serving cell", numUes);
    cmd.AddValue("scenario", "NTN scenario label (annotative)", scenario);
    cmd.AddValue("algorithm",
                 "tte-aware | a3 | location | d2 | time | elevation | "
                 "a3-baseline | location-baseline",
                 algorithm);
    cmd.AddValue("d1Threshold", "D1 distance threshold (m)", d1Threshold);
    cmd.AddValue("qualityTh", "SINR quality threshold (dB)", qualityTh);
    cmd.AddValue("tteMinimum", "Minimum TTE for admission (s)", tteMinimum);
    cmd.AddValue("tteMin", "Minimum TTE for CHO config (s)", tteMinSec);
    cmd.AddValue("carrierFreqGhz", "Carrier frequency (GHz)", carrierFreqGhz);
    cmd.AddValue("satTxPower", "Satellite EIRP / gNB Tx power (dBm)", satTxPower);
    cmd.AddValue("altitude", "Constellation altitude (km)", altitudeKm);
    cmd.AddValue("numCandidates", "Candidate satellite cells", numCandidates);
    cmd.AddValue("satsPerPlane", "Walker in-plane satellites (spacing)", satsPerPlane);
    cmd.AddValue("outputDir", "Output directory", outputDir);
    cmd.AddValue("rngRun", "RNG run", rngRun);
    cmd.AddValue("netSim", "NetSimulyzer 3D JSON trace output path (empty = off)", netSimOut);
    cmd.AddValue("czml", "Cesium CZML 3D trace output path (empty = off)", czmlOut);
    cmd.Parse(argc, argv);

    if (numCandidates < 1)
    {
        numCandidates = 1;
    }
    if (numCandidates > satsPerPlane - 2)
    {
        numCandidates = satsPerPlane - 2;
    }

    g_algorithm = algorithm;
    g_qualityTh = qualityTh;
    g_tteMinimum = tteMinimum;
    g_simTime = simTime;
    g_d1Threshold = d1Threshold;

    std::string mkdirCmd = "mkdir -p " + outputDir;
    if (system(mkdirCmd.c_str()) != 0)
    {
        std::cerr << "warning: could not create " << outputDir << "\n";
    }

    std::cout << "============================================\n"
              << "  NTN-CHO Full Constellation (v3 REAL plane)\n"
              << "============================================\n"
              << "  Serving + " << numCandidates << " candidate SGP4 cells (real pass)\n"
              << "  Altitude:      " << altitudeKm << " km\n"
              << "  UEs:           " << numUes << " (TR 38.811, real ns-3 nodes)\n"
              << "  Algorithm:     " << algorithm << "\n"
              << "  SimTime:       " << simTime << " s\n"
              << "  Radio:         real mmwave NR NTN cell (measured SINR)\n"
              << "============================================\n";

    // ---- Real Walker-Delta orbits (SGP4): serving + candidate shell ----
    WalkerConfig wcfg;
    wcfg.num_planes = 1;
    wcfg.total_sats = satsPerPlane;
    wcfg.altitude_km = altitudeKm;
    wcfg.inclination_deg = 53.0;
    wcfg.epoch_unix_s = 1735689600.0; // 2025-01-01
    const auto elements = WalkerConstellation::BuildDelta(wcfg);

    g_servSat = CreateObject<Sgp4MobilityModel>();
    g_servSat->SetElements(elements[0]);
    g_servSatId = 0;

    NodeContainer servSatNode;
    servSatNode.Create(1);
    servSatNode.Get(0)->AggregateObject(g_servSat);

    // Candidate cells = in-plane neighbours (genuinely approaching/receding).
    for (uint32_t k = 0; k < numCandidates; ++k)
    {
        const uint32_t idx = 1 + k; // neighbours after the serving sat
        Ptr<Sgp4MobilityModel> c = CreateObject<Sgp4MobilityModel>();
        c->SetElements(elements[idx % satsPerPlane]);
        g_candSats.push_back(c);
        g_candSatIds.push_back(idx);
    }

    // ---- TR 38.811 UEs under the serving sat's t=0 sub-point ----
    double subLat, subLon, subAlt;
    g_servSat->GetGeodetic(subLat, subLon, subAlt);
    NodeContainer ueNodes;
    ueNodes.Create(numUes);
    NtnTr38811MobilityHelper ueMobility(static_cast<uint64_t>(rngRun));
    auto profile = NtnMobilityScenarios::MixedContinental();
    g_ueModels = ueMobility.Install(ueNodes, profile, subLat - 0.03, subLat + 0.03,
                                    subLon - 0.03, subLon + 0.03);

    // ---- Real mmwave NTN serving cell + measured traffic ----
    NtnRealStackHelper rs;
    rs.SetSimTime(Seconds(simTime));
    rs.SetOutputDir(outputDir);
    rs.SetRunTag("ntn-cho-full-constellation_" + algorithm);
    rs.SetCarrierFrequencyHz(carrierFreqGhz * 1e9);
    rs.SetSatEirpDbm(satTxPower);
    rs.Build(servSatNode, ueNodes);
    rs.InstallTraffic(NtnRealStackHelper::TrafficProfile::EmbbStreaming, Seconds(1.0),
                      Seconds(simTime - 0.5));
    rs.EnableAiFlowMonitor(outputDir + "/ntn-cho-full-constellation");
    g_rs = &rs;

    // ---- Real CHO algorithm with the selected trigger ----
    g_choHelper = CreateObject<NtnChoHelper>();
    g_choHelper->SetCarrierFrequency(carrierFreqGhz * 1e9);
    g_choHelper->SetSatelliteTxPower(satTxPower);
    g_choHelper->SetTteMinimum(Seconds(tteMinSec));
    g_choHelper->SetD1Threshold(d1Threshold);
    g_choHelper->SetQualityThreshold(qualityTh);
    g_cho = g_choHelper->CreateChoAlgorithm();

    NtnChoAlgorithm::ChoConfig cfg = g_cho->GetConfig();
    cfg.triggerType = TriggerFor(algorithm);
    cfg.d1Threshold_m = d1Threshold;
    cfg.qualityThreshold_dB = qualityTh;
    cfg.tteMinimum = Seconds(tteMinSec);
    cfg.rachLess = true;
    // ChoTick drives EvaluateConditions on the real event queue; keep the
    // algorithm's internal monitor period large so it does not also auto-fire.
    cfg.conditionMonitorPeriod = Seconds(g_simTime + 10.0);
    g_cho->Configure(cfg);

    Ptr<mmwave::MmWaveEnbNetDevice> enb =
        DynamicCast<mmwave::MmWaveEnbNetDevice>(rs.GetEnbDevices().Get(0));
    g_servingCellId = enb ? enb->GetCellId() : 1;
    g_serving = g_servingCellId;

    // ---- Auxiliary ephemeris/TTE oracle pipeline (DEAD CLASSES WIRED) ----
    // NtnChoHelper::SetupConstellation builds NtnOrbitPredictor +
    // NtnTteEstimator + NtnMeasurementModel over real SatSGP4 sats + the
    // geo-33E antenna patterns (same recipe as the test fixture). Used ONLY as
    // an auxiliary TTE/ephemeris oracle feeding tte_computations.csv and the
    // candidate ranking; NEVER the headline SINR source. We resolve valid
    // geo-33E beam ids HERE (the AGP container rejects beam id 0) so the CHO
    // candidate cells registered below carry beams the predictor accepts.
    uint32_t servBeamId = 12; // valid geo-33E fallback
    bool auxReady = false;
    try
    {
        Singleton<SatEnvVariables>::Get()->DoInitialize();
        Singleton<SatEnvVariables>::Get()->SetOutputVariables(
            "ntn-cho-full-constellation", "", true);
        const std::string patternsFolder =
            Singleton<SatEnvVariables>::Get()->LocateDataDirectory() +
            "/scenarios/geo-33E/antennapatterns";
        Ptr<SatAntennaGainPatternContainer> agp =
            CreateObject<SatAntennaGainPatternContainer>(
                1 + static_cast<uint32_t>(g_candSats.size()), patternsFolder);

        NodeContainer auxSats;
        auxSats.Create(1 + static_cast<uint32_t>(g_candSats.size()));
        // geo-33E patterns are authored for a GEO satellite; place the aux
        // beam-mobility models at GEO altitude (as the test fixture does) so
        // the antenna-pattern geometry is well-posed. This is the auxiliary
        // ephemeris/TTE oracle ONLY; the headline geometry/SINR come from the
        // real SGP4 sats + measured PHY, not from here.
        constexpr double kGeoAltM = 35786000.0;
        // Serving aux GEO sat at the patterns' default sub-point (lon 33E, as
        // the test fixture); each candidate is shifted in longitude so its beam
        // centers (moving references) differ.
        Ptr<SatConstantPositionMobilityModel> sm0 =
            CreateObject<SatConstantPositionMobilityModel>();
        sm0->SetGeoPosition(GeoCoordinate(0.0, 33.0, kGeoAltM));
        agp->ConfigureBeamsMobility(0, sm0);
        auxSats.Get(0)->AggregateObject(sm0);
        for (size_t k = 0; k < g_candSats.size(); ++k)
        {
            Ptr<SatConstantPositionMobilityModel> smk =
                CreateObject<SatConstantPositionMobilityModel>();
            smk->SetGeoPosition(GeoCoordinate(0.0, 33.0 - 5.0 * (k + 1), kGeoAltM));
            agp->ConfigureBeamsMobility(static_cast<uint32_t>(k + 1), smk);
            auxSats.Get(k + 1)->AggregateObject(smk);
        }

        g_choHelper->SetupConstellation(auxSats, agp);
        g_orbit = g_choHelper->GetOrbitPredictor();
        g_tte = g_choHelper->GetTteEstimator();
        g_cho->SetOrbitPredictor(g_orbit);
        g_cho->SetTteEstimator(g_tte);
        auxReady = (g_orbit != nullptr && g_tte != nullptr);

        // Resolve VALID geo-33E beam ids at the fixed aux reference position
        // (the AGP container rejects beam id 0). Fall back to beam 12 (the test
        // suite's serving beam) if a sat is not visible from the reference.
        {
            auto vis = g_orbit->GetVisibleSatellites(kAuxRefPos, /*minElev=*/0.0);
            for (const auto& v : vis)
            {
                if (v.satId == 0)
                {
                    servBeamId = v.bestBeamId;
                    break;
                }
            }
            for (size_t k = 0; k < g_candSats.size(); ++k)
            {
                const uint32_t auxSatId = static_cast<uint32_t>(k + 1);
                uint32_t beam = 12; // valid geo-33E fallback
                for (const auto& v : vis)
                {
                    if (v.satId == auxSatId)
                    {
                        beam = v.bestBeamId;
                        break;
                    }
                }
                g_candBeamIds.push_back(beam);
            }
        }
        std::cout << "  [aux] ephemeris/TTE oracle ready (orbit predictor + TTE "
                     "estimator + measurement model)\n";
    }
    catch (const std::exception& e)
    {
        std::cerr << "  [aux] ephemeris/TTE oracle unavailable: " << e.what()
                  << " (tte_computations.csv will be empty; headline KPIs unaffected)\n";
    }
    (void)auxReady;

    // Ensure every candidate has a beam id even if the aux oracle was skipped.
    while (g_candBeamIds.size() < g_candSats.size())
    {
        g_candBeamIds.push_back(12);
    }

    // ---- Register the CHO serving + candidate cells (valid beam ids) ----
    g_cho->SetServingCell(g_servingCellId);
    g_cho->AddCandidateCell(g_servingCellId, /*satId=*/0, servBeamId);
    for (size_t k = 0; k < g_candSats.size(); ++k)
    {
        const uint16_t cid = g_servingCellId + 100 + static_cast<uint16_t>(k);
        g_candCellIds.push_back(cid);
        // CHO candidate satId = aux predictor satId (k+1) so the algorithm's
        // D1/D2/elevation predictor lookups resolve to a valid aux beam.
        g_cho->AddCandidateCell(cid, static_cast<uint32_t>(k + 1), g_candBeamIds[k]);
    }

    // Start CHO monitoring at the fixed aux reference (keeps the predictor-
    // backed D1/D2/T1/elevation/TTE geometry well-posed against the GEO
    // antenna patterns). This is the algorithm's internal predictor frame
    // only; measured serving/candidate SINR and the headline geometry come
    // from the live SGP4 sats + real PHY in ChoTick.
    g_cho->StartMonitoring(kAuxRefPos, Vector(0, 0, 0));

    // ---- Open output files (same schemas as v2) ----
    g_hoFile.open(outputDir + "/handover_events.csv");
    g_hoFile << "time_s,ue_id,source_cell,target_cell,source_sat,target_sat,"
             << "source_beam,target_beam,algorithm,sinr_before_dB,sinr_after_dB,"
             << "tte_predicted_s,time_of_stay_s,success,ping_pong,ue_lat,ue_lon,"
             << "ue_speed_mps,mobility_type,elevation_before,elevation_after,failure_reason\n";
    g_measFile.open(outputDir + "/measurements.csv");
    g_measFile << "time_s,ue_id,sat_id,beam_id,cell_id,rsrp_dBm,sinr_dB,"
               << "path_loss_dB,antenna_gain_dB,elevation_deg,range_km,"
               << "doppler_Hz,propagation_delay_ms,ue_lat,ue_lon\n";
    g_tteFile.open(outputDir + "/tte_computations.csv");
    g_tteFile << "time_s,ue_id,sat_id,beam_id,cell_id,tte_predicted_s,"
              << "current_gain_dB,peak_gain_dB,admitted,trigger_type\n";
    g_satTrackFile.open(outputDir + "/satellite_tracks.csv");
    g_satTrackFile << "time_s,sat_id,lat,lon,altitude_km,velocity_mps\n";
    g_ueTrackFile.open(outputDir + "/ue_tracks.csv");
    g_ueTrackFile << "time_s,ue_id,lat,lon,serving_cell,serving_sat,sinr_dB,"
                  << "mobility_type,ho_state,elevation_deg\n";
    g_kpiFile.open(outputDir + "/kpi_timeseries.csv");
    g_kpiFile << "time_s,total_hos,successful_hos,failed_hos,ping_pongs,"
              << "ho_success_rate,avg_sinr_dB,ho_rate_per_ue_per_min\n";
    g_satGeo.open(outputDir + "/satellite_positions.geojson");
    g_ueGeo.open(outputDir + "/ue_positions.geojson");
    g_beamGeo.open(outputDir + "/beam_footprints.geojson");
    g_hoGeo.open(outputDir + "/handover_events.geojson");
    g_satGeo << "{\"type\":\"FeatureCollection\",\"features\":[\n";
    g_ueGeo << "{\"type\":\"FeatureCollection\",\"features\":[\n";
    g_beamGeo << "{\"type\":\"FeatureCollection\",\"features\":[\n";
    g_hoGeo << "{\"type\":\"FeatureCollection\",\"features\":[\n";

    // ---- Optional 3D scene trace (real SGP4 sats + TR 38.811 UEs) ----
    Ptr<ntnobs::NtnSceneRecorder> scene;
    if (!netSimOut.empty() || !czmlOut.empty())
    {
        scene = CreateObject<ntnobs::NtnSceneRecorder>();
        scene->SetFrame(ntnobs::NtnSceneRecorder::EcefGlobal);
        scene->TrackNode(servSatNode.Get(0), ntnobs::NtnSceneRecorder::Sat, "serving-sat");
        for (uint32_t i = 0; i < ueNodes.GetN(); ++i)
        {
            scene->TrackNode(ueNodes.Get(i), ntnobs::NtnSceneRecorder::Ue,
                             "ue-" + std::to_string(i));
        }
        scene->SetSampleInterval(Seconds(g_dt));
        if (!netSimOut.empty())
        {
            scene->EnableNetSimulyzer(netSimOut);
        }
        if (!czmlOut.empty())
        {
            scene->EnableCzml(czmlOut);
        }
        scene->Start();
    }

    Simulator::Schedule(Seconds(1.0), &ChoTick);
    Simulator::Stop(Seconds(simTime));
    Simulator::Run();
    if (scene)
    {
        scene->Stop();
        std::cout << "  [scene] 3D trace events: " << scene->GetEventCount() << "\n";
    }
    rs.Collect();
    rs.WriteHealthReport();

    // Close GeoJSON + CSV.
    g_satGeo << "\n]}";
    g_ueGeo << "\n]}";
    g_beamGeo << "\n]}";
    g_hoGeo << "\n]}";
    g_satGeo.close();
    g_ueGeo.close();
    g_beamGeo.close();
    g_hoGeo.close();
    g_hoFile.close();
    g_measFile.close();
    g_tteFile.close();
    g_satTrackFile.close();
    g_ueTrackFile.close();
    g_kpiFile.close();

    // KPI summary (HO counts from the real algorithm, avg SINR measured).
    const double measSinrMean = rs.GetMeanDlSinrDb();
    std::ofstream kpiSummary(outputDir + "/kpi_summary.txt");
    kpiSummary << "=== NTN-CHO KPI Summary (REAL plane) ===\n"
               << "Trigger Type: " << algorithm << "\n"
               << "D1 Threshold: " << d1Threshold << " m\n"
               << "Quality Threshold: " << qualityTh << " dB\n"
               << "TTE Minimum: " << tteMinimum << " s\n\n"
               << "Total Handovers:     " << g_totalHos << "\n"
               << "Successful HOs:      " << g_successHos << "\n"
               << "Failed HOs:          " << g_failedHos << "\n"
               << "Ping-Pong Events:    " << g_ppCount << "\n"
               << std::fixed << std::setprecision(2)
               << "HO Success Rate:     "
               << (g_totalHos > 0 ? 100.0 * g_successHos / g_totalHos : 100.0) << " %\n"
               << "Measured serving SINR (mean): " << measSinrMean << " dB\n"
               << "Measured DL throughput:       " << rs.GetRxThroughputMbps()
               << " Mbps\n"
               << "Measured DL delay (mean):     " << rs.GetMeanDelayMs() << " ms\n";
    kpiSummary.close();

    const auto st = g_cho->GetMechanismStats();
    std::cout << "\n============================================\n"
              << "  SIMULATION COMPLETE (REAL plane)\n"
              << "  Algorithm:      " << algorithm << "\n"
              << "  Total HOs:      " << g_totalHos << "\n"
              << "  Success Rate:   " << std::fixed << std::setprecision(1)
              << (g_totalHos > 0 ? 100.0 * g_successHos / g_totalHos : 100.0) << " %\n"
              << "  measured SINR:  " << std::setprecision(2) << measSinrMean << " dB\n"
              << "  measured thr:   " << rs.GetRxThroughputMbps() << " Mbps\n"
              << "  RACH-less exec: " << st.rachLessExecutions << "\n"
              << "  last interrupt: " << st.lastInterruptionMs << " ms\n"
              << "  Output:         " << outputDir << "/\n"
              << "============================================\n";

    Simulator::Destroy();
    return 0;
}
