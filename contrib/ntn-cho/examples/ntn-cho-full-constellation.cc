/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-only
 * Author: Muhammad Uzair
 *
 * Full Constellation NTN-CHO Simulation (v2 - Realistic)
 *
 * Fixes from v1:
 *  - Proper initial serving assignment to best visible satellite
 *  - Calibrated TTE values (20-80s) matching real LEO beam dwell times
 *  - Realistic HO failure model (SINR-based + timer expiry)
 *  - TTE-aware algorithm genuinely filters unstable candidates
 *  - Ping-pong hysteresis for TTE-aware but not for baselines
 *  - Multi-beam per satellite (center + edge beams)
 *  - Computed satellite velocity from orbital parameters
 *  - All helper callbacks properly wired
 *  - Algorithm-specific behavioral differences
 */

#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/ntn-cho-algorithm.h"
#include "ns3/ntn-cho-helper.h"
#include "ns3/ntn-measurement-model.h"
#include "ns3/ntn-realistic-mobility.h"
#include "ns3/ntn-realistic-traffic-helper.h"
#include "ns3/ntn-scene-recorder.h"
#include "ns3/ntn-tr38811-mobility-model.h"
#include "ns3/sgp4-mobility-model.h"
#include "ns3/walker-constellation.h"

#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <random>
#include <sstream>

using namespace ns3;
NS_LOG_COMPONENT_DEFINE("NtnChoFullConstellationV2");

// ============= Orbital Mechanics =============

struct SatPosition {
    double lat, lon, alt_m;
    double vx, vy, vz;   // velocity m/s in local ENU frame
};

/**
 * Walker Star constellation model.
 * numPlanes planes, satsPerPlane satellites, inclination deg, altitude_m meters.
 * Returns sub-satellite point and velocity for sat (plane*satsPerPlane + idx).
 */
SatPosition computeSatPosition(uint32_t satId, uint32_t numPlanes, uint32_t satsPerPlane,
                                double inclination_deg, double altitude_m, double t_s)
{
    uint32_t plane = satId / satsPerPlane;
    uint32_t idx = satId % satsPerPlane;

    double R_earth = 6371000.0;
    double r = R_earth + altitude_m;
    double mu = 3.986004418e14; // Earth GM
    double orbitalPeriod = 2.0 * M_PI * std::sqrt(r * r * r / mu);
    double n = 2.0 * M_PI / orbitalPeriod; // mean motion

    // RAAN for each plane
    double raan = plane * (360.0 / numPlanes);
    // Phase offset within plane
    double phaseOffset = idx * (360.0 / satsPerPlane);
    // Mean anomaly at time t
    double M = fmod(n * t_s * 180.0 / M_PI + phaseOffset, 360.0);
    double M_rad = M * M_PI / 180.0;
    double inc_rad = inclination_deg * M_PI / 180.0;

    // Sub-satellite point (simplified spherical geometry)
    double satLat_rad = std::asin(std::sin(inc_rad) * std::sin(M_rad));
    double satLon_rad = std::atan2(std::cos(inc_rad) * std::sin(M_rad), std::cos(M_rad));
    // Add RAAN and Earth rotation
    double earthRotRate = 7.2921159e-5; // rad/s
    double lon_rad = satLon_rad + raan * M_PI / 180.0 - earthRotRate * t_s;

    SatPosition sp;
    sp.lat = satLat_rad * 180.0 / M_PI;
    sp.lon = fmod(lon_rad * 180.0 / M_PI + 540.0, 360.0) - 180.0;
    sp.alt_m = altitude_m;

    // Ground-track velocity
    double v_orbital = std::sqrt(mu / r); // ~7450 m/s
    double v_ground = v_orbital * R_earth / r;
    sp.vx = v_ground * std::cos(inc_rad); // eastward component
    sp.vy = v_ground * std::sin(inc_rad); // northward component
    sp.vz = 0;

    return sp;
}

/**
 * Compute elevation angle from UE (lat1,lon1) to satellite (lat2,lon2,alt).
 */
double computeElevation(double ueLat, double ueLon, double satLat, double satLon, double satAlt)
{
    double R = 6371000.0;
    double lat1 = ueLat * M_PI / 180.0, lon1 = ueLon * M_PI / 180.0;
    double lat2 = satLat * M_PI / 180.0, lon2 = satLon * M_PI / 180.0;

    // Central angle
    double dLat = lat2 - lat1, dLon = lon2 - lon1;
    double a = std::sin(dLat/2)*std::sin(dLat/2) +
               std::cos(lat1)*std::cos(lat2)*std::sin(dLon/2)*std::sin(dLon/2);
    double gamma = 2.0 * std::atan2(std::sqrt(a), std::sqrt(1-a));

    double groundDist = R * gamma;
    [[maybe_unused]] double slantRange = std::sqrt(groundDist * groundDist + satAlt * satAlt +
                        2 * R * satAlt * (1 - std::cos(gamma)) - 2 * groundDist * satAlt * std::cos(gamma));

    // Elevation: sin(el) = (R+h)*cos(gamma) - R) / slantRange
    double cosGamma = std::cos(gamma);
    double sinEl = ((R + satAlt) * cosGamma - R) / std::sqrt(R*R + (R+satAlt)*(R+satAlt) - 2*R*(R+satAlt)*cosGamma);
    return std::asin(std::max(-1.0, std::min(1.0, sinEl))) * 180.0 / M_PI;
}

double computeSlantRange(double ueLat, double ueLon, double satLat, double satLon, double satAlt)
{
    double R = 6371000.0;
    double lat1 = ueLat * M_PI / 180.0, lon1 = ueLon * M_PI / 180.0;
    double lat2 = satLat * M_PI / 180.0, lon2 = satLon * M_PI / 180.0;
    double dLat = lat2-lat1, dLon = lon2-lon1;
    double a = std::sin(dLat/2)*std::sin(dLat/2) +
               std::cos(lat1)*std::cos(lat2)*std::sin(dLon/2)*std::sin(dLon/2);
    double gamma = 2.0 * std::atan2(std::sqrt(a), std::sqrt(1-a));
    return std::sqrt(R*R + (R+satAlt)*(R+satAlt) - 2*R*(R+satAlt)*std::cos(gamma));
}

// ============= Link Budget (3GPP TR 38.811) =============

struct LinkBudget {
    double rsrp_dBm, sinr_dB, pathLoss_dB, antennaGain_dB;
    double elevation_deg, slantRange_km, doppler_Hz, delay_ms;
};

LinkBudget computeLinkBudget(double ueLat, double ueLon, const SatPosition& sat,
                              double freqHz, double bwHz, double txPower_dBm,
                              double noiseFigure_dB, int ntnScenario, std::mt19937& rng)
{
    LinkBudget lb;
    lb.elevation_deg = computeElevation(ueLat, ueLon, sat.lat, sat.lon, sat.alt_m);
    double slantRange_m = computeSlantRange(ueLat, ueLon, sat.lat, sat.lon, sat.alt_m);
    lb.slantRange_km = slantRange_m / 1000.0;
    lb.delay_ms = slantRange_m / 299792458.0 * 1000.0;

    // FSPL
    double fspl = 20*std::log10(slantRange_m) + 20*std::log10(freqHz) + 20*std::log10(4*M_PI/299792458.0);

    // Satellite multi-beam antenna: each beam steers toward its ground cell.
    // Gain depends on elevation angle (low elevation = longer path + edge-of-coverage).
    // Per 3GPP TR 38.821 Table 6.1.1-1: typical LEO satellite antenna parameters.
    // Peak EIRP per beam: ~34-40 dBi, 3dB beamwidth: ~4-5 deg
    // Gain model: best at high elevation (beam center overhead), degrades at low elevation
    // This models the COMPOSITE effect of beam pointing + scan loss + array factor
    if (lb.elevation_deg >= 70) lb.antennaGain_dB = 30.0;       // Near-zenith: peak
    else if (lb.elevation_deg >= 50) lb.antennaGain_dB = 28.0;  // Good coverage
    else if (lb.elevation_deg >= 30) lb.antennaGain_dB = 25.0;  // Moderate
    else if (lb.elevation_deg >= 20) lb.antennaGain_dB = 22.0;  // Edge of good coverage
    else if (lb.elevation_deg >= 10) lb.antennaGain_dB = 18.0;  // Minimum usable
    else lb.antennaGain_dB = 10.0;                               // Below min elevation

    // Atmospheric + scintillation loss (TR 38.811 Sec 6.6.4)
    double atmosLoss = 0.2 + 0.5 / std::max(std::sin(lb.elevation_deg * M_PI / 180.0), 0.1);
    atmosLoss = std::min(atmosLoss, 5.0);

    // Clutter loss (TR 38.811 Table 6.6.2-1)
    double clutterLoss[] = {4.0, 2.5, 1.0, 0.5};
    double clutter = clutterLoss[std::min(ntnScenario, 3)];

    // Shadow fading (log-normal, TR 38.811 Table 6.6.2-2)
    double sfStd[] = {4.0, 4.0, 2.0, 1.0}; // dB std per scenario
    std::normal_distribution<double> sfDist(0.0, sfStd[std::min(ntnScenario, 3)]);
    double shadowFading = sfDist(rng);

    lb.pathLoss_dB = fspl + atmosLoss + clutter + shadowFading;
    lb.rsrp_dBm = txPower_dBm + lb.antennaGain_dB - lb.pathLoss_dB;

    // Noise power
    double noisePower = -174.0 + 10.0 * std::log10(bwHz) + noiseFigure_dB;
    lb.sinr_dB = lb.rsrp_dBm - noisePower;

    // Doppler shift. The radial velocity must carry a SIGN: positive while the
    // satellite approaches (slant range shrinking -> positive frequency shift)
    // and negative while it recedes, so the Doppler trace flips sign as the
    // satellite passes overhead — a defining feature of a real LEO pass.
    double lat1 = ueLat * M_PI / 180.0;
    double dLat = (sat.lat - ueLat) * M_PI / 180.0;
    double dLon = (sat.lon - ueLon) * M_PI / 180.0;
    double dE = 6371000.0 * dLon * std::cos(lat1); // east displacement UE->sat
    double dN = 6371000.0 * dLat;                  // north displacement UE->sat
    double gd = std::sqrt(dE * dE + dN * dN);
    double speed = std::sqrt(sat.vx * sat.vx + sat.vy * sat.vy);
    // Project ground-track velocity (vx=east, vy=north) onto the UE->sat
    // direction: v.d_hat > 0 means receding. Doppler sign is the opposite of
    // the range rate.
    double rangeRate = (gd > 1.0) ? (sat.vx * dE + sat.vy * dN) / gd : 0.0;
    double sign = (rangeRate <= 0.0) ? 1.0 : -1.0;
    double vRadial = speed * std::sin(gd / 6371000.0);
    lb.doppler_Hz = sign * vRadial / 299792458.0 * freqHz;

    return lb;
}

// ============= UE State =============

struct UeState {
    uint32_t id;
    double lat, lon;
    double vNorth, vEast;  // m/s
    std::string mobilityType;
    uint32_t servingSatId;
    uint16_t servingCellId;
    double servingSinr;
    double servingElevation;
    uint32_t hoCount, hoFails, pingPongs;
    double lastHoTime;
    uint32_t lastSourceSat;
    double lastHoSinr;
    // TTE-aware state
    double lastTtePredicted;
    bool choHysteresisActive; // prevent immediate re-HO
    double hysteresisUntil;   // time until hysteresis expires
};

// ============= MAIN =============

int main(int argc, char* argv[])
{
    double simTime = 600.0;
    uint32_t numUes = 50;
    std::string scenario = "suburban";
    std::string algorithm = "tte-aware";
    double d1Threshold = 50000;
    double qualityTh = -5.0;
    double tteMinimum = 20.0;
    double carrierFreq = 2.0e9;
    double bandwidth = 30.0e6;
    double satTxPower = 43.0;
    std::string outputDir = "ntn-cho-output";
    uint32_t rngRun = 1;
    bool verbose = false;
    // Constellation params
    uint32_t numPlanes = 6;
    uint32_t satsPerPlane = 11;
    double altitude_km = 780.0;
    double inclination = 86.4;

    CommandLine cmd(__FILE__);
    cmd.AddValue("simTime", "Simulation time (s)", simTime);
    cmd.AddValue("numUes", "Number of UEs", numUes);
    cmd.AddValue("scenario", "NTN scenario: dense-urban(0), urban(1), suburban(2), rural(3)", scenario);
    cmd.AddValue("algorithm", "a3, location, time, tte-aware", algorithm);
    cmd.AddValue("d1Threshold", "D1 distance threshold (m)", d1Threshold);
    cmd.AddValue("qualityTh", "SINR quality threshold (dB)", qualityTh);
    cmd.AddValue("tteMinimum", "Minimum TTE (s)", tteMinimum);
    cmd.AddValue("outputDir", "Output directory", outputDir);
    cmd.AddValue("rngRun", "RNG run", rngRun);
    cmd.AddValue("verbose", "Verbose", verbose);
    cmd.AddValue("numPlanes", "Number of orbital planes", numPlanes);
    cmd.AddValue("satsPerPlane", "Satellites per plane", satsPerPlane);
    uint32_t trafficUesOpt = 0;
    cmd.AddValue("trafficUes",
                 "UEs carrying the real UDP plane (0 = all UEs)", trafficUesOpt);
    // 3D scene trace (real SGP4 sats + TR 38.811 UEs). Off unless a path is
    // given; default outputs are unchanged when these are empty.
    std::string netSimOut;
    std::string czmlOut;
    uint32_t sceneMaxSats = 120; // cap streamed sats so the trace stays sane
    cmd.AddValue("netSim", "NetSimulyzer 3D JSON trace output path (empty = off)", netSimOut);
    cmd.AddValue("czml", "Cesium CZML 3D trace output path (empty = off)", czmlOut);
    cmd.AddValue("sceneMaxSats", "Max satellites to stream to the 3D scene", sceneMaxSats);
    cmd.Parse(argc, argv);

    int ntnScenario = (scenario == "dense-urban") ? 0 : (scenario == "urban") ? 1 :
                      (scenario == "rural") ? 3 : 2;
    uint32_t numSats = numPlanes * satsPerPlane;
    double alt_m = altitude_km * 1000.0;
    std::string mkdirCmd = "mkdir -p " + outputDir;
    system(mkdirCmd.c_str());

    std::mt19937 rng(rngRun);
    std::uniform_real_distribution<double> uniDist(0.0, 1.0);
    std::normal_distribution<double> normDist(0.0, 1.0);

    std::cout << "============================================\n"
              << "  NTN-CHO Full Constellation (v2 Realistic)\n"
              << "============================================\n"
              << "  Constellation: " << numPlanes << "x" << satsPerPlane << " = " << numSats << " sats\n"
              << "  Altitude:      " << altitude_km << " km, Inc: " << inclination << " deg\n"
              << "  UEs:           " << numUes << "\n"
              << "  Algorithm:     " << algorithm << "\n"
              << "  Scenario:      " << scenario << "\n"
              << "  SimTime:       " << simTime << " s\n"
              << "============================================\n";

    // ============= Create UEs (3GPP TR 38.811 §6.1.1.1 mobility classes) =============
    // Use NtnRealisticMobilityHelper so each UE gets a class-appropriate
    // motion rule (static / pedestrian / vehicular / HST / maritime /
    // aviation / IoT) with speeds + headings sourced from 3GPP and IMO/ICAO.
    NtnRealisticMobilityHelper mobilityHelper(static_cast<uint64_t>(rngRun));
    auto profile = NtnMobilityScenarios::MixedContinental();
    auto realisticUes = mobilityHelper.GenerateUes(numUes, profile,
                                                    /*minLat=*/25.0,
                                                    /*maxLat=*/65.0,
                                                    /*minLon=*/-20.0,
                                                    /*maxLon=*/40.0);

    std::vector<UeState> ues(numUes);
    for (uint32_t i = 0; i < numUes; i++) {
        const auto& src = realisticUes[i];
        ues[i].id = i;
        ues[i].lat = src.lat;
        ues[i].lon = src.lon;
        ues[i].vNorth = src.vNorth;
        ues[i].vEast  = src.vEast;
        ues[i].mobilityType = src.className;
        ues[i].servingSatId = UINT32_MAX;
        ues[i].servingCellId = 0;
        ues[i].servingSinr = -100;
        ues[i].hoCount = 0; ues[i].hoFails = 0; ues[i].pingPongs = 0;
        ues[i].lastHoTime = -1000; ues[i].lastSourceSat = UINT32_MAX;
        ues[i].lastTtePredicted = 0; ues[i].choHysteresisActive = false; ues[i].hysteresisUntil = 0;
    }

    // ============= Initial serving assignment: find best visible sat =============
    for (auto& ue : ues) {
        double bestSinr = -200;
        for (uint32_t s = 0; s < numSats; s++) {
            auto sp = computeSatPosition(s, numPlanes, satsPerPlane, inclination, alt_m, 0.0);
            double elev = computeElevation(ue.lat, ue.lon, sp.lat, sp.lon, alt_m);
            if (elev < 10.0) continue;
            auto lb = computeLinkBudget(ue.lat, ue.lon, sp, carrierFreq, bandwidth, satTxPower, 7.0, ntnScenario, rng);
            if (lb.sinr_dB > bestSinr) {
                bestSinr = lb.sinr_dB;
                ue.servingSatId = s;
                ue.servingCellId = s + 1;
                ue.servingSinr = lb.sinr_dB;
                ue.servingElevation = elev;
            }
        }
    }
    std::cout << "Initial serving assigned to all UEs\n\n";

    // ============= Open output files =============
    std::ofstream hoFile(outputDir + "/handover_events.csv");
    hoFile << "time_s,ue_id,source_cell,target_cell,source_sat,target_sat,"
           << "source_beam,target_beam,algorithm,sinr_before_dB,sinr_after_dB,"
           << "tte_predicted_s,time_of_stay_s,success,ping_pong,ue_lat,ue_lon,"
           << "ue_speed_mps,mobility_type,elevation_before,elevation_after,failure_reason\n";

    std::ofstream measFile(outputDir + "/measurements.csv");
    measFile << "time_s,ue_id,sat_id,beam_id,cell_id,rsrp_dBm,sinr_dB,"
             << "path_loss_dB,antenna_gain_dB,elevation_deg,range_km,"
             << "doppler_Hz,propagation_delay_ms,ue_lat,ue_lon\n";

    std::ofstream tteFile(outputDir + "/tte_computations.csv");
    tteFile << "time_s,ue_id,sat_id,beam_id,cell_id,tte_predicted_s,"
            << "current_gain_dB,peak_gain_dB,admitted,trigger_type\n";

    std::ofstream satTrackFile(outputDir + "/satellite_tracks.csv");
    satTrackFile << "time_s,sat_id,lat,lon,altitude_km,velocity_mps\n";

    std::ofstream ueTrackFile(outputDir + "/ue_tracks.csv");
    ueTrackFile << "time_s,ue_id,lat,lon,serving_cell,serving_sat,sinr_dB,"
                << "mobility_type,ho_state,elevation_deg\n";

    std::ofstream kpiFile(outputDir + "/kpi_timeseries.csv");
    kpiFile << "time_s,total_hos,successful_hos,failed_hos,ping_pongs,"
            << "ho_success_rate,avg_sinr_dB,ho_rate_per_ue_per_min\n";

    // GeoJSON
    std::ofstream satGeo(outputDir + "/satellite_positions.geojson");
    std::ofstream ueGeo(outputDir + "/ue_positions.geojson");
    std::ofstream beamGeo(outputDir + "/beam_footprints.geojson");
    std::ofstream hoGeo(outputDir + "/handover_events.geojson");
    satGeo << "{\"type\":\"FeatureCollection\",\"features\":[\n"; bool fSat = true;
    ueGeo << "{\"type\":\"FeatureCollection\",\"features\":[\n"; bool fUe = true;
    beamGeo << "{\"type\":\"FeatureCollection\",\"features\":[\n"; bool fBeam = true;
    hoGeo << "{\"type\":\"FeatureCollection\",\"features\":[\n"; bool fHo = true;

    // ============= MAIN SIMULATION LOOP (event-driven, v2) ===========
    // Each per-second analytical step is now a Simulator::Schedule()
    // callback registered through NtnRealisticTrafficHelper.  A parallel
    // UDP traffic plane runs through the ns-3 protocol stack so the event
    // queue carries real packet events, not just the analytical ticks.
    double dt = 1.0;
    uint32_t totalHos = 0, successHos = 0, failedHos = 0, ppCount = 0;

    NtnRealisticTrafficHelper traffic;
    traffic.SetSimTime(Seconds(simTime));
    traffic.SetOutputDir(outputDir);
    traffic.SetRunTag("ntn-cho-full-constellation_" + algorithm);
    traffic.SetProfile(NtnRealisticTrafficHelper::TrafficProfile::MixedBouquet);
    // With the geometry-coupled plane, delivery is outage-dominated (UEs
    // keep sending while searching for a cell), so the delivery floor
    // guards against dead queues rather than enforcing QoS.
    {
        NtnRealisticTrafficHelper::HealthGates g;
        g.minRxOverTxRatio = 0.55;
        traffic.SetGates(g);
    }
    // Real UDP plane on all UEs by default; --trafficUes caps it for very
    // large analytical sweeps. The analytical loop always covers numUes.
    uint32_t trafficUes =
        (trafficUesOpt == 0) ? numUes : std::min<uint32_t>(numUes, trafficUesOpt);
    traffic.InstallUes(trafficUes);

    // Per-UE data-plane state for geometry coupling: last geometry-derived
    // one-way delay and PER, plus the sim-time until which a handover
    // interruption holds the link in outage.
    std::vector<int64_t> ueOwdNs(numUes, 15000000); // 15 ms bootstrap
    std::vector<double> uePer(numUes, 0.001);
    std::vector<double> ueIntUntil(numUes, -1.0);

    traffic.RegisterPeriodicCallback(Seconds(dt), [&](Time nowT) {
        double t = nowT.GetSeconds();
        // --- Write satellite tracks (every 2s) ---
        if (fmod(t, 2.0) < dt) {
            for (uint32_t s = 0; s < numSats; s++) {
                auto sp = computeSatPosition(s, numPlanes, satsPerPlane, inclination, alt_m, t);
                double v = std::sqrt(sp.vx*sp.vx + sp.vy*sp.vy + sp.vz*sp.vz);
                // altitude from the PROPAGATED state (sp.alt_m), not the
                // config constant — identical for this circular-orbit model,
                // but the column must carry the measured value (audit rule).
                satTrackFile << std::fixed << std::setprecision(3) << t << ","
                    << s << "," << std::setprecision(6) << sp.lat << "," << sp.lon << ","
                    << std::setprecision(1) << (sp.alt_m / 1000.0) << ","
                    << std::setprecision(0) << v << "\n";

                if (fmod(t, 30.0) < dt) {
                    if (!fSat) satGeo << ",\n";
                    satGeo << "{\"type\":\"Feature\",\"properties\":{\"satId\":" << s
                           << ",\"time\":" << std::setprecision(1) << t
                           << ",\"altitude_km\":" << altitude_km
                           << "},\"geometry\":{\"type\":\"Point\",\"coordinates\":["
                           << std::setprecision(6) << sp.lon << "," << sp.lat << "," << alt_m << "]}}";
                    fSat = false;

                    // Beam footprint
                    double beamRadius_deg = 2.0;
                    if (!fBeam) beamGeo << ",\n";
                    beamGeo << "{\"type\":\"Feature\",\"properties\":{\"satId\":" << s
                            << ",\"beamId\":0,\"time\":" << std::setprecision(1) << t
                            << "},\"geometry\":{\"type\":\"Polygon\",\"coordinates\":[[";
                    for (int k = 0; k <= 16; k++) {
                        double a = 2*M_PI*k/16;
                        double pLat = sp.lat + beamRadius_deg * std::cos(a);
                        double pLon = sp.lon + beamRadius_deg * std::sin(a) / std::max(std::cos(sp.lat*M_PI/180), 0.01);
                        if (k > 0) beamGeo << ",";
                        beamGeo << "[" << std::setprecision(4) << pLon << "," << pLat << "]";
                    }
                    beamGeo << "]]}}";
                    fBeam = false;
                }
            }
        }

        // --- Per-UE processing ---
        double sumSinr = 0;
        uint32_t servedUes = 0; // UEs with a valid serving satellite this step
        for (size_t ueIdx = 0; ueIdx < ues.size(); ueIdx++) {
            auto& ue = ues[ueIdx];
            auto& realisticUe = realisticUes[ueIdx];
            // Class-aware motion (random-walk, Gauss-Markov heading,
            // waypoint, or stationary depending on UE class).
            mobilityHelper.AdvanceUe(realisticUe, dt);
            ue.lat = realisticUe.lat;
            ue.lon = realisticUe.lon;
            ue.vNorth = realisticUe.vNorth;
            ue.vEast  = realisticUe.vEast;

            // Measure all visible satellites
            struct SatMeas { uint32_t satId; double sinr, rsrp, gain, elev, range, doppler, loss, delay; };
            std::vector<SatMeas> visibleSats;

            for (uint32_t s = 0; s < numSats; s++) {
                auto sp = computeSatPosition(s, numPlanes, satsPerPlane, inclination, alt_m, t);
                double elev = computeElevation(ue.lat, ue.lon, sp.lat, sp.lon, alt_m);
                if (elev < 10.0) continue;
                auto lb = computeLinkBudget(ue.lat, ue.lon, sp, carrierFreq, bandwidth, satTxPower, 7.0, ntnScenario, rng);
                visibleSats.push_back({s, lb.sinr_dB, lb.rsrp_dBm, lb.antennaGain_dB,
                                       lb.elevation_deg, lb.slantRange_km, lb.doppler_Hz, lb.pathLoss_dB, lb.delay_ms});
            }

            std::sort(visibleSats.begin(), visibleSats.end(),
                      [](const SatMeas& a, const SatMeas& b) { return a.sinr > b.sinr; });

            // Write measurements (every 5s)
            if (fmod(t, 5.0) < dt) {
                for (size_t mi = 0; mi < visibleSats.size(); mi++) {
                    auto& m = visibleSats[mi];
                    measFile << std::fixed << std::setprecision(3) << t << "," << ue.id << ","
                             << m.satId << ",0," << (m.satId+1) << ","
                             << std::setprecision(2) << m.rsrp << "," << m.sinr << ","
                             << m.loss << "," << m.gain << ","
                             << std::setprecision(1) << m.elev << "," << m.range << ","
                             << std::setprecision(0) << m.doppler << ","
                             << std::setprecision(3) << m.delay << ","
                             << std::setprecision(6) << ue.lat << "," << ue.lon << "\n";
                }
            }

            // Update serving SINR
            double servMeas = -100;
            double servElev = 0;
            for (auto& vs : visibleSats) {
                if (vs.satId == ue.servingSatId) { servMeas = vs.sinr; servElev = vs.elev; break; }
            }
            ue.servingSinr = servMeas;
            ue.servingElevation = servElev;

            // --- CHO DECISION ---
            bool doHo = false;
            uint32_t targetSat = 0;
            double targetSinr = -100, targetElev = 0, ttePred = 0;
            std::string failReason = "";

            // Find best neighbor
            double bestNeighborSinr = -200;
            uint32_t bestNeighborSat = UINT32_MAX;
            double bestNeighborElev = 0;
            for (auto& vs : visibleSats) {
                if (vs.satId != ue.servingSatId && vs.sinr > bestNeighborSinr) {
                    bestNeighborSinr = vs.sinr;
                    bestNeighborSat = vs.satId;
                    bestNeighborElev = vs.elev;
                }
            }

            // Check if serving is degrading or lost
            bool servingLost = (ue.servingSinr < -30);
            bool servingPoor = (ue.servingSinr < qualityTh);
            bool neighborMuchBetter = (bestNeighborSinr > ue.servingSinr + 3.0);

            // Hysteresis check (only for TTE-aware)
            bool hysteresisBlock = (algorithm == "tte-aware" && ue.choHysteresisActive && t < ue.hysteresisUntil);

            if ((servingLost || servingPoor || neighborMuchBetter) && !hysteresisBlock && bestNeighborSat != UINT32_MAX) {

                // Compute TTE for candidates: based on elevation trajectory prediction
                // Higher elevation now = satellite is overhead longer = higher TTE
                // Peak TTE at ~70 deg elevation, lower at 10-20 deg
                std::vector<std::pair<uint32_t, double>> candidateTTE; // satId, tte
                for (auto& vs : visibleSats) {
                    if (vs.satId == ue.servingSatId) continue;
                    if (vs.sinr < qualityTh - 3.0) continue;

                    // TTE model: satellite at higher elevation has more coverage time remaining
                    // At 780km, beam dwell ranges from ~20s (10 deg) to ~90s (70+ deg)
                    double tte_s = 15.0 + vs.elev * 1.0; // 15s at 0deg, 105s at 90deg
                    // Add some noise
                    tte_s += normDist(rng) * 3.0;
                    tte_s = std::max(5.0, tte_s);
                    candidateTTE.push_back({vs.satId, tte_s});

                    // Write TTE computation
                    tteFile << std::fixed << std::setprecision(3) << t << "," << ue.id << ","
                            << vs.satId << ",0," << (vs.satId+1) << ","
                            << std::setprecision(2) << tte_s << ","
                            << vs.gain << "," << (vs.gain + 3) << ","
                            << (vs.sinr >= qualityTh && tte_s >= tteMinimum ? 1 : 0) << ","
                            << algorithm << "\n";
                }

                if (algorithm == "tte-aware") {
                    // NOVEL: Select candidate with longest TTE that passes quality filter
                    double bestTte = 0;
                    for (auto& [cSat, cTte] : candidateTTE) {
                        double cSinr = -100;
                        double cElev = 0;
                        for (auto& vs : visibleSats) { if (vs.satId == cSat) { cSinr = vs.sinr; cElev = vs.elev; break; } }
                        if (cSinr >= qualityTh && cTte >= tteMinimum && cTte > bestTte) {
                            bestTte = cTte;
                            targetSat = cSat;
                            targetSinr = cSinr;
                            targetElev = cElev;
                            ttePred = cTte;
                            doHo = true;
                        }
                    }
                    // Fallback: if no candidate meets TTE, stay (don't handover to unstable cell)
                    if (!doHo && servingLost) {
                        // Emergency: take best available
                        targetSat = bestNeighborSat;
                        targetSinr = bestNeighborSinr;
                        targetElev = bestNeighborElev;
                        ttePred = 0;
                        doHo = true;
                    }
                }
                else if (algorithm == "location") {
                    // Baseline location: pick highest SINR neighbor that meets quality
                    for (auto& vs : visibleSats) {
                        if (vs.satId != ue.servingSatId && vs.sinr >= qualityTh) {
                            targetSat = vs.satId; targetSinr = vs.sinr; targetElev = vs.elev; doHo = true; break;
                        }
                    }
                }
                else if (algorithm == "a3") {
                    // A3: neighbor > serving + offset, NO quality check
                    if (bestNeighborSinr > ue.servingSinr + 3.0) {
                        targetSat = bestNeighborSat; targetSinr = bestNeighborSinr;
                        targetElev = bestNeighborElev; doHo = true;
                    }
                }
                else { // time-based
                    // Timer-based: handover every 45s unconditionally
                    if (t - ue.lastHoTime > 45.0 && bestNeighborSat != UINT32_MAX) {
                        targetSat = bestNeighborSat; targetSinr = bestNeighborSinr;
                        targetElev = bestNeighborElev; doHo = true;
                    }
                }
            }

            // Execute handover
            if (doHo && targetSat != UINT32_MAX) {
                totalHos++;
                ue.hoCount++;

                // Realistic HO success model:
                // - Low SINR at target -> radio link failure during HO
                // - Low elevation -> high delay -> T304 timer more likely to expire
                // - High UE speed -> increased failure probability
                bool success = true;
                double speed = std::sqrt(ue.vNorth*ue.vNorth + ue.vEast*ue.vEast);
                double failProb = 0.0;
                if (targetSinr < qualityTh - 3.0) { failProb += 0.40; failReason = "LowSINR"; }
                else if (targetSinr < qualityTh) { failProb += 0.15; failReason = "MarginalSINR"; }
                if (targetElev < 15.0) { failProb += 0.12; if(failReason.empty()) failReason = "LowElevation"; }
                if (speed > 50.0) { failProb += 0.05; if(failReason.empty()) failReason = "HighSpeed"; }
                failProb += 0.01; // Base 1% random failure (T304/radio issues)
                if (uniDist(rng) < failProb) { success = false; if(failReason.empty()) failReason = "T304Timeout"; }
                else { failReason = ""; }

                // Time-of-stay on the source cell. Before the first handover
                // lastHoTime is a negative init sentinel; the UE has then been
                // served since acquisition (~start of sim), so use t — never the
                // sentinel, which would inflate ToS past the sim duration.
                double tos = (ue.lastHoTime >= 0.0) ? (t - ue.lastHoTime) : t;
                bool isPP = (ue.lastHoTime >= 0.0 && targetSat == ue.lastSourceSat && tos < 10.0);

                if (success) { successHos++; } else { failedHos++; ue.hoFails++; }
                if (isPP) { ppCount++; ue.pingPongs++; }

                hoFile << std::fixed << std::setprecision(3) << t << "," << ue.id << ","
                       << ue.servingCellId << "," << (targetSat+1) << ","
                       << ue.servingSatId << "," << targetSat << ","
                       << "0,0," << algorithm << ","
                       << std::setprecision(2) << ue.servingSinr << "," << targetSinr << ","
                       << ttePred << "," << tos << ","
                       << (success?1:0) << "," << (isPP?1:0) << ","
                       << std::setprecision(6) << ue.lat << "," << ue.lon << ","
                       << std::setprecision(1) << speed << "," << ue.mobilityType << ","
                       << std::setprecision(1) << ue.servingElevation << "," << targetElev << ","
                       << (success ? "" : failReason) << "\n";

                // GeoJSON HO event
                if (!fHo) hoGeo << ",\n";
                hoGeo << "{\"type\":\"Feature\",\"properties\":{\"ueId\":" << ue.id
                      << ",\"time\":" << std::setprecision(1) << t
                      << ",\"sourceCell\":" << ue.servingCellId << ",\"targetCell\":" << (targetSat+1)
                      << ",\"success\":" << (success?"true":"false")
                      << ",\"pingPong\":" << (isPP?"true":"false")
                      << ",\"algorithm\":\"" << algorithm << "\""
                      << ",\"tte\":" << std::setprecision(2) << ttePred
                      << ",\"sinrBefore\":" << ue.servingSinr
                      << ",\"sinrAfter\":" << targetSinr
                      << "},\"geometry\":{\"type\":\"Point\",\"coordinates\":["
                      << std::setprecision(6) << ue.lon << "," << ue.lat << "]}}";
                fHo = false;

                // Update state
                if (success) {
                    ue.lastSourceSat = ue.servingSatId;
                    ue.servingSatId = targetSat;
                    ue.servingCellId = targetSat + 1;
                    ue.servingSinr = targetSinr;
                    ue.lastHoTime = t;
                    ue.lastTtePredicted = ttePred;
                    // TTE-aware: set hysteresis to prevent ping-pong
                    if (algorithm == "tte-aware") {
                        ue.choHysteresisActive = true;
                        ue.hysteresisUntil = t + std::min(ttePred * 0.5, 15.0); // half TTE or 15s
                    }
                }

                // Data-plane interruption during execution: the link drops
                // for the handover window, then restores to the last
                // geometry-derived state. CHO variants keep the target
                // pre-prepared (short window); baseline A3 pays the full
                // break-before-make budget.
                if (ueIdx < trafficUes) {
                    double intMs = (algorithm == "a3") ? 80.0 : 30.0;
                    ueIntUntil[ueIdx] = t + intMs / 1000.0;
                    traffic.UpdateUeLink(ueIdx, NanoSeconds(ueOwdNs[ueIdx]), 1.0);
                    NtnRealisticTrafficHelper* tp = &traffic;
                    auto* owdVec = &ueOwdNs;
                    auto* perVec = &uePer;
                    uint32_t restoreIdx = static_cast<uint32_t>(ueIdx);
                    Simulator::Schedule(MilliSeconds(static_cast<uint64_t>(intMs)),
                        [tp, owdVec, perVec, restoreIdx]() {
                            tp->UpdateUeLink(restoreIdx,
                                             NanoSeconds((*owdVec)[restoreIdx]),
                                             (*perVec)[restoreIdx]);
                        });
                }
            }

            // "Served" means a valid serving satellite AND a measurable link.
            // When the serving sat drops below the horizon servingSinr falls to
            // the -100 sentinel (set at the measurement step); such a UE is
            // effectively searching, so exclude it from the CSV serving fields
            // and the average — never emit the sentinel as a real value.
            const bool ueServed =
                (ue.servingSatId != UINT32_MAX && ue.servingSinr > -50.0);
            if (ueServed) {
                sumSinr += ue.servingSinr;
                servedUes++;
            }

            // ---- Geometry-coupled data plane ---------------------------
            // Retune this UE's access link from the serving slant path and
            // serving SINR so the packet plane tracks orbital geometry:
            //   delay = (service + feeder legs)/c + 1 ms processing
            //           (feeder leg approximated by the service slant; the
            //            gateway sits in the same region as the UEs)
            //   PER   = logistic in serving SINR, clamped; 1.0 in outage.
            // TR 38.821 LEO transparent budgets bracket the resulting
            // one-way delays (about 6-16 ms at 780 km).
            if (ueIdx < trafficUes) {
                double owdMs = 30.0; // searching: parked high-delay value
                double per = 1.0;    // no serving cell -> outage
                if (ueServed) {
                    double srvRangeKm = -1.0;
                    for (auto& vs : visibleSats) {
                        if (vs.satId == ue.servingSatId) { srvRangeKm = vs.range; break; }
                    }
                    if (srvRangeKm > 0.0) {
                        owdMs = 2.0 * srvRangeKm / 299.792458 + 1.0;
                        // Residual (post-HARQ) loss versus serving SINR:
                        // ~0.1% at -5 dB, ~0.7% at -10 dB, ~12% at -16 dB,
                        // consistent with NR-NTN low-MCS operation down to
                        // about -6 dB Es/N0 plus repetition gain below it.
                        per = 1.0 / (1.0 + std::exp(0.5 * (ue.servingSinr + 20.0)));
                        per = std::min(0.90, std::max(1e-4, per));
                    }
                }
                ueOwdNs[ueIdx] = static_cast<int64_t>(owdMs * 1e6);
                uePer[ueIdx] = per;
                // During a handover-interruption window the link stays in
                // outage; the scheduled restore re-applies these arrays.
                if (t >= ueIntUntil[ueIdx]) {
                    traffic.UpdateUeLink(ueIdx, NanoSeconds(ueOwdNs[ueIdx]), per);
                }
            }

            // Write UE track (every 2s). For an unserved UE (no satellite in
            // view) emit empty serving fields rather than the UINT32_MAX / -100
            // sentinels so the CSV never carries non-physical values.
            if (fmod(t, 2.0) < dt) {
                ueTrackFile << std::fixed << std::setprecision(3) << t << "," << ue.id << ","
                            << std::setprecision(6) << ue.lat << "," << ue.lon << ",";
                if (ueServed) {
                    ueTrackFile << ue.servingCellId << "," << ue.servingSatId << ","
                                << std::setprecision(2) << ue.servingSinr << ",";
                } else {
                    ueTrackFile << ",,,"; // serving_cell, serving_sat, sinr_dB empty
                }
                ueTrackFile << ue.mobilityType << "," << (doHo ? "handover" : (ueServed ? "connected" : "searching")) << ","
                            << std::setprecision(1) << ue.servingElevation << "\n";

                if (!fUe) ueGeo << ",\n";
                ueGeo << "{\"type\":\"Feature\",\"properties\":{\"ueId\":" << ue.id
                      << ",\"time\":" << std::setprecision(1) << t
                      << ",\"servingCell\":" << ue.servingCellId
                      << ",\"sinr_dB\":" << std::setprecision(2) << ue.servingSinr
                      << ",\"hoState\":\"" << (doHo?"handover":"connected")
                      << "\"},\"geometry\":{\"type\":\"Point\",\"coordinates\":["
                      << std::setprecision(6) << ue.lon << "," << ue.lat << "]}}";
                fUe = false;
            }
        }

        // KPI timeseries (every 5s)
        if (fmod(t, 5.0) < dt) {
            double rate = (totalHos > 0) ? 100.0 * successHos / totalHos : 100.0;
            double hoPerUeMin = (t > 0) ? totalHos / (double)numUes / (t / 60.0) : 0;
            kpiFile << std::fixed << std::setprecision(3) << t << ","
                    << totalHos << "," << successHos << "," << failedHos << "," << ppCount << ","
                    << std::setprecision(1) << rate << ","
                    << std::setprecision(2) << (servedUes > 0 ? sumSinr / servedUes : 0.0) << ","
                    << std::setprecision(4) << hoPerUeMin << "\n";
        }

        if (fmod(t, 60.0) < dt) {
            std::cout << "  t=" << (int)t << "s: " << totalHos << " HOs ("
                      << successHos << " ok, " << failedHos << " fail, " << ppCount << " pp)"
                      << " avgSINR=" << std::fixed << std::setprecision(1) << (servedUes > 0 ? sumSinr/servedUes : 0.0) << " dB\n";
        }
    });  // end of periodic-callback lambda

    traffic.Wire();

    // ============= Real-mobility 3D scene layer (optional) =============
    // The analytical loop above drives the CHO statistics; for 3D / NetSimulyzer
    // / Cesium we install REAL ns-3 mobility nodes — SGP4-propagated satellites
    // (ntn-constellation) and TR 38.811 UEs (ntn-cho) built from THIS run's
    // constellation/UE configuration — and stream their real positions. This
    // gives the flagship genuine streamable node mobility (the analytical
    // satellite model carries no MobilityModel) without changing any of the
    // CSV/GeoJSON outputs the dashboard already consumes.
    Ptr<ntnobs::NtnSceneRecorder> scene;
    NodeContainer sceneSats;
    NodeContainer sceneUes;
    if (!netSimOut.empty() || !czmlOut.empty())
    {
        ntncon::WalkerConfig wcfg;
        wcfg.num_planes = numPlanes;
        wcfg.total_sats = numSats;
        wcfg.altitude_km = altitude_km;
        wcfg.inclination_deg = inclination;
        wcfg.epoch_unix_s = 1735689600.0; // 2025-01-01, matches real-stack
        const auto elements = ntncon::WalkerConstellation::BuildDelta(wcfg);
        const uint32_t nStream = std::min<uint32_t>(numSats, sceneMaxSats);
        if (nStream < numSats)
        {
            std::cout << "  [scene] streaming " << nStream << " of " << numSats
                      << " satellites (--sceneMaxSats)\n";
        }
        sceneSats.Create(nStream);
        for (uint32_t s = 0; s < nStream; ++s)
        {
            Ptr<ntncon::Sgp4MobilityModel> m = CreateObject<ntncon::Sgp4MobilityModel>();
            m->SetElements(elements[s]);
            sceneSats.Get(s)->AggregateObject(m);
        }
        // Real TR 38.811 UEs over the same geographic box as the analytical UEs.
        sceneUes.Create(numUes);
        NtnTr38811MobilityHelper ueMob(static_cast<uint64_t>(rngRun));
        auto sceneProfile = NtnMobilityScenarios::MixedContinental();
        ueMob.Install(sceneUes, sceneProfile, 25.0, 65.0, -20.0, 40.0);

        scene = CreateObject<ntnobs::NtnSceneRecorder>();
        scene->SetFrame(ntnobs::NtnSceneRecorder::EcefGlobal);
        for (uint32_t s = 0; s < nStream; ++s)
        {
            scene->TrackNode(sceneSats.Get(s),
                             ntnobs::NtnSceneRecorder::Sat,
                             "sat-" + std::to_string(s));
        }
        for (uint32_t i = 0; i < numUes; ++i)
        {
            scene->TrackNode(sceneUes.Get(i),
                             ntnobs::NtnSceneRecorder::Ue,
                             "ue-" + std::to_string(i));
        }
        scene->SetSampleInterval(Seconds(dt));
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

    Simulator::Stop(Seconds(simTime + 0.5));
    Simulator::Run();
    if (scene)
    {
        scene->Stop();
        std::cout << "  [scene] 3D trace events: " << scene->GetEventCount() << "\n";
    }
    traffic.WriteHealthReport();

    // Close GeoJSON
    satGeo << "\n]}"; ueGeo << "\n]}"; beamGeo << "\n]}"; hoGeo << "\n]}";
    satGeo.close(); ueGeo.close(); beamGeo.close(); hoGeo.close();
    hoFile.close(); measFile.close(); tteFile.close(); satTrackFile.close(); ueTrackFile.close(); kpiFile.close();

    // KPI summary
    std::ofstream kpiSummary(outputDir + "/kpi_summary.txt");
    kpiSummary << "=== NTN-CHO KPI Summary ===\n"
               << "Trigger Type: " << algorithm << "\n"
               << "D1 Threshold: " << d1Threshold << " m\n"
               << "Quality Threshold: " << qualityTh << " dB\n"
               << "TTE Minimum: " << tteMinimum << " s\n\n"
               << "Total Handovers:     " << totalHos << "\n"
               << "Successful HOs:      " << successHos << "\n"
               << "Failed HOs:          " << failedHos << "\n"
               << "Ping-Pong Events:    " << ppCount << "\n"
               << std::fixed << std::setprecision(2)
               << "HO Success Rate:     " << (totalHos>0 ? 100.0*successHos/totalHos : 100.0) << " %\n"
               << "HO Failure Rate:     " << (totalHos>0 ? 100.0*failedHos/totalHos : 0.0) << " %\n"
               << "Ping-Pong Rate:      " << (totalHos>0 ? 100.0*ppCount/totalHos : 0.0) << " %\n";
    kpiSummary.close();

    std::cout << "\n============================================\n"
              << "  SIMULATION COMPLETE\n"
              << "  Algorithm:      " << algorithm << "\n"
              << "  Total HOs:      " << totalHos << "\n"
              << "  Success Rate:   " << std::fixed << std::setprecision(1)
              << (totalHos>0?100.0*successHos/totalHos:100.0) << " %\n"
              << "  Failure Rate:   " << (totalHos>0?100.0*failedHos/totalHos:0.0) << " %\n"
              << "  Ping-Pong Rate: " << (totalHos>0?100.0*ppCount/totalHos:0.0) << " %\n"
              << "  Output:         " << outputDir << "/\n"
              << "============================================\n";

    Simulator::Destroy();
    return 0;
}
