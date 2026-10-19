/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-only
 * Author: Muhammad Uzair
 *
 * Constellation-scale conditional-handover campaign with ephemeris error.
 *
 * WHAT THIS SCENARIO IS FOR
 *
 * ntn-cho-full-constellation runs the CHO state machine over a measured 5G-LENA
 * cell, which is the right tool for asking whether a decision survives the RRC,
 * and the wrong one for asking how a policy behaves across a mega-constellation:
 * the vendored NR stack carries a handful of cells and terminals, and one UE
 * decides for all of them. This scenario answers the second question. Every
 * terminal runs its OWN NtnChoAlgorithm against every satellite of a Walker
 * shell, and the comparison between policies is made on the same orbits, the
 * same terminals and the same random draws.
 *
 * It is a system-level simulation in the sense of 3GPP TR 38.821 Section 6.1:
 * the radio is a link budget evaluated per terminal and per satellite on the
 * ns-3 event queue, not a packet-level PHY. What is NOT closed-form is
 * everything a handover policy reacts to: the orbits (Kepler + J2 secular,
 * ntn-constellation Sgp4MobilityModel), the terminal motion (TR 38.811 Section
 * 6.1.1.1 classes), the path loss (ns-3's own ThreeGppNTN propagation model,
 * TR 38.811 Section 6.6), the decision logic (the module's NtnChoAlgorithm,
 * unchanged) and the prediction (NtnOrbitPredictor + NtnTteEstimator, fed an
 * ephemeris that is deliberately wrong by a configurable amount).
 *
 * MODEL, stated so a reader can check every term
 *
 *  - Constellation: Walker-Delta T/P/F (default 1584/72/17 at 550 km, 53 deg,
 *    a Starlink-shell-1-like geometry). The epoch is offset per seed by a
 *    uniform draw over one day, so each seed sees a different geometry.
 *  - Cells: one cell per satellite, served by a beam STEERED at the terminal.
 *    Gain falls with off-nadir scan angle as cos^n (n = 1.2), and a satellite
 *    cannot serve below the minimum TOPOCENTRIC elevation (default 25 deg,
 *    WGS-84 normal), the array's scan limit; the predictor's steered beam is
 *    indexed by the same elevation, so truth and prediction cut at one contour. Peak EIRP density 34 dBW/MHz (TR 38.821 Table
 *    6.1.1.1-1, LEO-600 S-band, Set-1) over the configured bandwidth.
 *  - Carrier 2.185 GHz with a 20 MHz channel (inside the n256 downlink block of
 *    TS 38.101-5). Handheld terminal: 0 dBi, 3 dB polarization loss, 7 dB noise
 *    figure (TR 38.821 Table 6.1.1.1-2).
 *  - Environments (--channel): los and suburban use the suburban model and
 *    tables; urban and denseurban use ns-3's ThreeGppNTNUrban/DenseUrban
 *    models with their TR 38.811 Table 6.6.1-1 LOS probabilities and
 *    Table 6.6.2-2/2-3 S-band shadowing sigmas.
 *  - Path loss: ns3::ThreeGppNTNSuburbanPropagationLossModel (TR 38.811
 *    Section 6.6) under a line-of-sight condition. At S-band its atmospheric
 *    absorption is applied only below 10 deg (zero here) and its scintillation
 *    is a constant per-elevation term, not a random process. Its built-in shadowing is disabled because it
 *    decorrelates on the satellite's 3D displacement, which over a 100 ms tick
 *    is about 750 m and makes every sample independent; shadowing here is
 *    drawn with the TR 38.811 Table 6.6.2-1 S-band LOS sigma at the link's
 *    quantized elevation and correlated over TERMINAL displacement with the 37 m
 *    decorrelation distance of Table 6.7.2-5a, which is how 3GPP specifies it;
 *    a change of the link's 10-degree elevation bin (the satellite's own
 *    motion) starts a new realization at that bin's sigma.
 *  - Interference: none. Satellites covering the same area are assumed to use
 *    orthogonal resources, so SINR is SNR. A co-channel study would change
 *    absolute SINR, not the geometry every exit time is measured on.
 *  - Measurement: L1 sample every 100 ms with 1 dB measurement error, layer-3
 *    filter of TS 38.331 Section 5.5.3.2 with filterCoefficient k = 4, its
 *    coefficient adapted to the 100 ms input period. Decisions run at every
 *    L3 result (100 ms), so time-to-trigger timers have 100 ms granularity.
 *    Every handover interruption is booked as outage at its exact duration.
 *  - Radio link failure: Qout = -8 dB, Qin = -6 dB, T310 = 1 s, N310 = N311 = 1.
 *    Leaving the coverage cone counts as losing the link. Re-establishment
 *    costs a fixed 200 ms plus four one-way slant traversals for the RACH.
 *  - Handover execution: the interruption the algorithm itself prices (RRC
 *    execution plus the four-message RACH at the target's slant). The handover
 *    succeeds when, at completion, the target still covers the terminal and its
 *    L1 SINR is at least Qout; otherwise it is a handover failure followed by
 *    re-establishment.
 *  - Ephemeris error: each satellite's predicted state carries a fixed
 *    radial / along-track / cross-track offset drawn from N(0, sigma^2) per
 *    axis, or (--ephErrorFile) one measured TLE-vs-precise RTN error sample
 *    drawn per satellite from an element-set age window. The orbit the satellite flies is unaffected; only what the network
 *    believes is wrong. The network's assumed sigmas (what it budgets its margin
 *    for) can differ from the true ones.
 *
 * POLICIES (--policy)
 *
 *   a3         CondEventA3, offset/TTT configurable (Rel-16 CHO)
 *   d2         CondEventD2, moving references at the sub-satellite points (Rel-18)
 *   t1         CondEventT1, absolute epoch set by the network from its predicted
 *              exit of the serving cell, strongest qualifying candidate (Rel-17)
 *   elev       elevation trigger (TR 38.821 Section 6 study item)
 *   tte-eager  switch whenever a candidate is predicted to outlast the serving cell
 *   mst        exit-triggered, longest predicted time-to-exit, margin covers the
 *              execution latency only (classic maximum-service-time)
 *   tte-cho    exit-triggered, longest predicted time-to-exit, margin covers the
 *              execution latency plus z sigma of the serving cell's exit time
 *
 *   Options on top of the policies: --rankByUsable (rank targets by the
 *   expected usable time to exit, the LOS-probability integral of the
 *   predicted pass), --prepareByTte (prepare the K candidates by that score
 *   instead of by L3), --escapeHoldoffS (hold off the cell left through the
 *   safety net), --measFallback (TTE-CHO's measurement fallback for the
 *   standardized conditions), --tteRank (rank a standardized condition's
 *   admitted candidates by predicted exit).
 *
 * OUTPUT (--outputDir)
 *
 *   summary.csv      one row per run
 *   ue_summary.csv   one row per terminal
 *   events.csv       one row per handover or re-establishment
 *   tte_accuracy.csv (--mode=tte-accuracy) predicted vs realized exit times
 *
 * Example:
 *   ./ns3 run "ntn-cho-tte-campaign --policy=tte-cho --numUes=100 --simTime=600 \
 *       --ephSigmaAlongM=1000 --rngRun=1 --outputDir=/tmp/tte"
 */

#include "ns3/channel-condition-model.h"
#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/ntn-cho-algorithm.h"
#include "ns3/ntn-measurement-model.h"
#include "ns3/ntn-orbit-predictor.h"
#include "ns3/ntn-realistic-mobility.h"
#include "ns3/ntn-tr38811-mobility-model.h"
#include "ns3/ntn-tte-estimator.h"
#include "ns3/orbital-elements.h"
#include "ns3/sgp4-mobility-model.h"
#include "ns3/three-gpp-propagation-loss-model.h"
#include "ns3/walker-constellation.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <set>
#include <sstream>
#include <unordered_map>
#include <vector>

using namespace ns3;
using ns3::ntncon::KeplerianElements;
using ns3::ntncon::Sgp4MobilityModel;
using ns3::ntncon::TleRecord;
using ns3::ntncon::WalkerConfig;
using ns3::ntncon::WalkerConstellation;

NS_LOG_COMPONENT_DEFINE("NtnChoTteCampaign");

namespace
{

constexpr double kC = 299792458.0;
// R2: one Earth radius everywhere (WGS-84 equatorial, as the truth orbits and
// the text); the predictor's elevation/scan mapping is set to the same value.
constexpr double kRe = 6378137.0;
constexpr double kNegInf = -1.0e9;
// What a terminal measures on a cell that no longer illuminates it.
constexpr double kNoSignalDb = -30.0;

// TR 38.811 Table 6.6.2-1, S-band, line-of-sight shadow-fading sigma (dB) for the
// suburban and rural scenarios, by elevation in 10 degree steps. These are the
// values ns-3's ThreeGppNTNSuburbanPropagationLossModel carries
// (src/propagation/model/three-gpp-propagation-loss-model.cc, SFCL_SuburbanRural).
// They are repeated here only because that model keeps its accessor private.
const std::map<int, double> kSfLosSigmaS{{10, 1.79},
                                         {20, 1.14},
                                         {30, 1.14},
                                         {40, 0.92},
                                         {50, 1.42},
                                         {60, 1.56},
                                         {70, 0.85},
                                         {80, 0.72},
                                         {90, 0.72}};
// TR 38.811 Table 6.6.2-1, S-band, NON-line-of-sight shadow-fading sigma (dB),
// suburban and rural (ns-3 SFCL_SuburbanRural, S_NLOS_sigF column).
const std::map<int, double> kSfNlosSigmaS{{10, 8.93},
                                          {20, 9.08},
                                          {30, 8.78},
                                          {40, 10.25},
                                          {50, 10.56},
                                          {60, 10.74},
                                          {70, 10.17},
                                          {80, 11.52},
                                          {90, 11.52}};
// TR 38.811 Table 6.6.1-1, suburban and rural line-of-sight probability (%), the
// values ns-3's ThreeGppNTNSuburbanChannelConditionModel carries
// (src/propagation/model/channel-condition-model.cc, SuburbanRuralLOSProb).
const std::map<int, double> kPlosSuburban{{10, 78.2},
                                          {20, 86.9},
                                          {30, 91.9},
                                          {40, 92.9},
                                          {50, 93.5},
                                          {60, 94.0},
                                          {70, 94.9},
                                          {80, 95.2},
                                          {90, 99.8}};
// Urban and dense-urban environments (TR 38.811 Tables 6.6.1-1 and 6.6.2-1/2/3,
// S band), as carried by ns-3: LOS probabilities from
// src/propagation/model/channel-condition-model.cc (UrbanLOSProb,
// DenseUrbanLOSProb) and shadow-fading sigmas from
// src/propagation/model/three-gpp-propagation-loss-model.cc (SFCL_Urban,
// SFCL_DenseUrban, columns S_LOS_sigF and S_NLOS_sigF).
const std::map<int, double> kPlosUrban{{10, 24.6}, {20, 38.6}, {30, 49.3}, {40, 61.3}, {50, 72.6},
                                       {60, 80.5}, {70, 91.9}, {80, 96.8}, {90, 99.2}};
const std::map<int, double> kPlosDenseUrban{{10, 28.2}, {20, 33.1}, {30, 39.8}, {40, 46.8}, {50, 53.7},
                                            {60, 61.2}, {70, 73.8}, {80, 82.0}, {90, 98.1}};
const std::map<int, double> kSfLosSigmaUrban{{10, 4.0}, {20, 4.0}, {30, 4.0}, {40, 4.0}, {50, 4.0},
                                             {60, 4.0}, {70, 4.0}, {80, 4.0}, {90, 4.0}};
const std::map<int, double> kSfNlosSigmaUrban{{10, 6.0}, {20, 6.0}, {30, 6.0}, {40, 6.0}, {50, 6.0},
                                              {60, 6.0}, {70, 6.0}, {80, 6.0}, {90, 6.0}};
const std::map<int, double> kSfLosSigmaDenseUrban{{10, 3.5}, {20, 3.4}, {30, 2.9}, {40, 3.0}, {50, 3.1},
                                                  {60, 2.7}, {70, 2.5}, {80, 2.3}, {90, 1.2}};
const std::map<int, double> kSfNlosSigmaDenseUrban{{10, 15.5}, {20, 13.9}, {30, 12.4}, {40, 11.7},
                                                   {50, 10.6}, {60, 10.5}, {70, 10.1}, {80, 9.2},
                                                   {90, 9.2}};
// TR 38.811 Table 6.7.2-5a/6a: shadowing decorrelation distance, suburban.
// ns-3 applies the same 37 m (LOS) and 50 m (NLOS) to its urban and
// dense-urban NTN models (TR 38.811 Tables 6.7.2-1a/b to 6.7.2-3a/b), so the
// two values serve every environment here.
constexpr double kSfDecorrM = 37.0;
constexpr double kSfDecorrNlosM = 50.0;
// The line-of-sight state is redrawn when the terminal has moved --losCorrM
// (default 50 m; TR 38.901 Section 7.6.3.3 spatial consistency uses the same
// scale) or when the link's elevation crosses into another 10-degree bin of
// Table 6.6.1-1 (--losModel=redraw), or decorrelates as a Markov process
// (--losModel=markov).

int
ElevBin(double elevDeg)
{
    int q = static_cast<int>(std::lround(elevDeg / 10.0)) * 10;
    return std::max(10, std::min(90, q));
}

double
SfSigmaAt(const std::map<int, double>& table, double elevDeg)
{
    return table.at(ElevBin(elevDeg));
}

// R2 (M8): environment of one terminal. A terminal class that does not move
// through street clutter (aircraft, ships) sees the sky in line of sight, and
// a high-speed train runs through rural terrain, whose TR 38.811 Table 6.6.1-1
// row is the suburban/rural one; every other class sees the environment of
// the run.
enum EnvIdx
{
    kEnvLos = 0,
    kEnvSub = 1,
    kEnvUrb = 2,
    kEnvDense = 3
};
struct EnvTables
{
    const std::map<int, double>* plos;
    const std::map<int, double>* sfLos;
    const std::map<int, double>* sfNlos;
};
EnvTables g_env[4] = {{&kPlosSuburban, &kSfLosSigmaS, &kSfNlosSigmaS},
                      {&kPlosSuburban, &kSfLosSigmaS, &kSfNlosSigmaS},
                      {&kPlosUrban, &kSfLosSigmaUrban, &kSfNlosSigmaUrban},
                      {&kPlosDenseUrban, &kSfLosSigmaDenseUrban, &kSfNlosSigmaDenseUrban}};

// Chord distance between the terminal and the sub-satellite point of a satellite
// seen at elevation e from altitude h (the quantity CondEventD2 compares).
double
ChordToNadirAtElevation(double elevDeg, double altM)
{
    const double e = elevDeg * M_PI / 180.0;
    const double nadirAngle = std::asin(kRe / (kRe + altM) * std::cos(e));
    const double central = M_PI / 2.0 - e - nadirAngle;
    return 2.0 * kRe * std::sin(central / 2.0);
}

struct Params
{
    std::string policy = "tte-cho";
    std::string mode = "campaign";
    uint32_t numUes = 100;
    double simTime = 600.0;
    uint32_t rngRun = 1;
    std::string outputDir = "ntn-cho-tte-campaign";
    // constellation
    uint32_t totalSats = 1584;
    uint32_t numPlanes = 72;
    uint32_t phasing = 17;
    double altitudeKm = 550.0;
    double inclinationDeg = 53.0;
    // service area
    double centerLat = 30.0;
    double centerLon = 70.0;
    double halfSpanDeg = 1.0;
    std::string ueProfile = "mixed"; // mixed | static
    std::string channel = "los";     // los | suburban | urban | denseurban (TR 38.811 LOS/NLOS process)
    // radio
    double fcGhz = 2.185;
    double bwMhz = 20.0;
    double eirpDensityDbwPerMhz = 34.0;
    double scanLossExp = 1.2;
    double minElevDeg = 25.0;
    double polLossDb = 3.0;
    double ueNfDb = 7.0;
    double measErrDb = 1.0;
    uint32_t l3FilterK = 4;
    double qOutDb = -8.0;
    double qInDb = -6.0;
    double t310S = 1.0;
    double reestBaseMs = 200.0;
    // R2 additions
    bool classEnv = true;             // M8: per-class environment (air/sea LOS, train rural)
    std::string losModel = "redraw";  // M16: redraw (50 m or new bin) | markov
    double losCorrM = 50.0;           // LOS-state correlation distance (m)
    double losCorrDeg = 10.0;         // markov: angular decorrelation scale (deg of elevation)
    std::string ephModel = "propagated"; // M10: propagated (own orbit) | offset (re-anchored)
    bool escapeByQuality = false;     // M2: escape to the strongest candidate better than serving
    bool prepAboveContour = false;    // M7: prepare only candidates predicted inside e_Q for tau_min
    std::string tleTruthFile;         // M10: real constellation, truth element sets (SupGP)
    std::string tleNetFile;           //      network element sets (public GP)
    double tleInclMinDeg = 52.5;
    double tleInclMaxDeg = 53.5;
    double tleAltMinKm = 455.0;
    double tleAltMaxKm = 485.0;
    double tleStartSpreadH = 1.0;     // start at the median truth epoch +- this
    // CHO common
    double qualityThDb = -3.0;
    uint32_t maxCandidates = 8;
    // F3: decisions at every L3 result, so time-to-trigger timers are
    // evaluated at the measurement period rather than at a slower epoch
    double decisionPeriodS = 0.1;
    double tteCacheS = 1.0;          // F3b: candidate prediction cache refresh
    double candRefreshS = 1.0;
    // A3
    double a3OffsetDb = 3.0;
    double a3TttMs = 160.0;
    // D2: thresholds expressed as the elevations they correspond to
    double d2ServElevDeg = 35.0;
    double d2CandElevDeg = 45.0;
    double d2HysM = 10000.0;
    // T1
    double t1DurationS = 10.0;
    // elevation trigger
    double elevFloorDeg = 30.0;
    double elevHystDeg = 2.0;
    // time-to-exit policies
    double tteMinS = 10.0;
    double tteExitElevDeg = -1.0; // quality-feasible exit; -1 = the coverage limit
    double tteFloorDb = -6.0;     // serving-loss safety net (blockage)
    double tteFloorTttMs = 160.0; // time below the floor before it fires (A3's default TTT)
    // Single-factor controls for the standardized conditions:
    //  measFallback gives A3/D2/T1/elev the same serving-floor safety net as
    //  TTE-CHO (a second prepared candidate on a measurement condition), executed
    //  to the strongest prepared candidate;
    //  tteRank keeps the configured execution condition but takes, among the
    //  candidates it admitted, the one with the longest predicted time to exit
    //  (the rule of Noreen and Schotten, CCNC 2026, mapped onto moving cells).
    bool measFallback = false;
    bool tteRank = false;
    // N1-N3 (new policies)
    bool rankByUsable = false;       // rank by expected usable time to exit
    bool prepareByTte = false;       // prepared set by predicted score, not by L3
    double escapeHoldoffS = 0.0;     // hold-off of the cell left via the safety net
    double escapeNearBestDb = -1.0;  // escape target within this margin of the strongest (-1 = off)
    uint32_t prepStrongest = 0;      // prepared slots reserved for the strongest satellites (with prepareByTte)
    int32_t dumpLinksUe = -1;        // F8 check: per-tick link realization dump
    bool dumpLosDraws = false;       // write every LOS-state draw (elevation bin, state)
    double execMarginS = 0.2;
    double marginZ = 2.0;
    double predStepS = 2.0;
    double horizonS = 900.0;
    // ephemeris error: what is true, and what the network assumes
    double ephSigmaRadialM = 0.0;
    double ephSigmaAlongM = 0.0;
    double ephSigmaCrossM = 0.0;
    double assumedSigmaRadialM = -1.0; // -1: same as true
    double assumedSigmaAlongM = -1.0;
    double assumedSigmaCrossM = -1.0;
    // N4: empirical ephemeris error, one fixed RTN offset per satellite drawn
    // from measured TLE-vs-precise-ephemeris samples (dt_min == 0 rows)
    std::string ephErrorFile;
    double ephAgeMinH = 0.0;
    double ephAgeMaxH = 1e9;
    // accuracy mode
    uint32_t accuracySamples = 2000;
    int32_t traceUe = -1; // write trace.csv for this terminal (1 s cadence)
};

Params g_p;

struct LinkState
{
    bool los = true;
    bool losInit = false;
    int losBin = 0;
    Vector losUePos;
    double losElev = 0.0;     // markov: elevation at the last update
    double sf = 0.0;          // current shadowing (dB)
    Vector sfUePos;           // terminal position at the last shadowing draw
    bool sfInit = false;
    int sfBin = 0;            // F12: elevation bin of the current shadowing draw
    double l3 = kNegInf;      // layer-3 filtered SINR (dB)
    double l1 = kNegInf;      // last L1 SINR sample (dB)
    double elevDeg = -90.0;
    double slantM = 0.0;
    double scanLossDb = 0.0;
    bool covered = false;
};

struct UeCtx
{
    Ptr<Node> node;
    Ptr<NtnTr38811MobilityModel> mob;
    Ptr<NtnChoAlgorithm> cho;
    std::string ueClass;
    int env = kEnvLos;                // R2 (M8): environment of this terminal
    Ptr<NtnTteEstimator> tte;         // estimator carrying this environment's LOS table
    int32_t serving = -1;        // satellite index, -1 = not connected
    double connectedSince = 0.0; // time the current serving association began
    bool executing = false;      // handover in flight
    bool reestablishing = false;
    double outageUntil = 0.0;    // no data before this time
    double t310Start = -1.0;     // running T310 since (s), -1 = stopped
    double floorSince = -1.0;    // serving L3 below the fallback floor since (s)
    int32_t escCell = -1;        // cell left through the fallback (standardized conditions)
    double escUntil = -1.0;      // its hold-off end (s)
    double intUntil = -1.0;      // F1: interruption already booked as outage until (s)
    Vector tickPos;              // F8: position and velocity read at the last tick
    Vector tickVel;
    int32_t prevServing = -1;
    double lastHoTime = -1e9;
    std::unordered_map<uint32_t, LinkState> links;
    std::set<uint32_t> candidates;
    // counters
    uint32_t hoAttempts = 0;
    uint32_t hoSuccess = 0;
    uint32_t hof = 0;
    uint32_t rlf = 0;
    uint32_t pp1 = 0;
    uint32_t pp5 = 0;
    double outageS = 0.0;
    double sinrSum = 0.0;
    double seSum = 0.0;
    uint64_t samples = 0;
    std::vector<double> tos;
    uint32_t lbHandovers = 0;
    uint64_t decisions = 0;      // F3b cost: decision epochs evaluated
    uint32_t t1Rearms = 0;       // F5: CondEventT1 windows re-armed
    // R2 (M6): conditional-reconfiguration messages the configuration implies
    bool rrcDirty = false;
    bool rrcCandDirty = false;
    bool rrcWinDirty = false;
    uint32_t rrc = 0;            // epochs at which a new conditional reconfiguration is needed
    uint32_t rrcCand = 0;        // of which the prepared set changed
    uint32_t rrcWin = 0;         // of which the execution window / target moved
    double sigEnd = -1.0;        // TTE family: window end last signaled
    uint16_t sigTarget = 0;      //             and its candidate
    uint32_t hoPlanned = 0;      // handovers by trigger kind
    uint32_t hoEscape = 0;
};

std::vector<Ptr<Sgp4MobilityModel>> g_sats;
std::vector<Vector> g_satPos; // ECEF at the current tick
std::vector<UeCtx> g_ues;
Ptr<NtnOrbitPredictor> g_pred;
Ptr<NtnTteEstimator> g_tte;
Ptr<ThreeGppPropagationLossModel> g_plEnv[4];     // line of sight, per environment
Ptr<ThreeGppPropagationLossModel> g_plNlosEnv[4]; // non line of sight (TR 38.811 clutter loss)
Ptr<NtnTteEstimator> g_tteEnv[4];
std::vector<Ptr<Sgp4MobilityModel>> g_net; // R2 (M10): the network's own copy of each orbit
bool g_ephActive = false;                  // any ephemeris error injected
std::vector<Vector> g_ephDrawn;            // R2: drawn RTN error per satellite at t = 0
Ptr<UniformRandomVariable> g_uni;
Ptr<NormalRandomVariable> g_norm;
double g_noiseDbm = 0.0;
double g_eirpDbm = 0.0;
double g_tteGainTh = 0.0;   // exit threshold the prediction-based policies use
double g_coverGainTh = 0.0; // scan limit: where a satellite stops covering at all
std::ofstream g_events;
std::ofstream g_acc;
std::ofstream g_trace;
std::ofstream g_dump;
std::ofstream g_losDump; // --dumpLosDraws: one row per LOS-state draw
Ptr<NormalRandomVariable> g_normOut; // F8: draws whose occurrence depends on the policy
const double kTick = 0.1;

uint16_t
CellOf(uint32_t sat)
{
    return static_cast<uint16_t>(sat + 1);
}

uint32_t
SatOf(uint16_t cell)
{
    return static_cast<uint32_t>(cell - 1);
}

// The predictor works in SNS3 GeoCoordinate, which defaults to a SPHERICAL
// Earth. Constructing it from WGS84 latitude and height would put the terminal
// about 19 km from where it is at 30 degrees latitude; constructing it from the
// ECEF vector round-trips exactly.
GeoCoordinate
GeoOf(const Vector& ecef)
{
    return GeoCoordinate(ecef);
}

// Off-nadir scan angle of a steered beam pointing from sat to ue, and its loss.
double
ScanLossDb(const Vector& ue, const Vector& sat)
{
    const double sn = std::sqrt(sat.x * sat.x + sat.y * sat.y + sat.z * sat.z);
    const Vector d(ue.x - sat.x, ue.y - sat.y, ue.z - sat.z);
    const double dn = std::sqrt(d.x * d.x + d.y * d.y + d.z * d.z);
    const double c = -(sat.x * d.x + sat.y * d.y + sat.z * d.z) / (sn * dn);
    return 10.0 * g_p.scanLossExp * std::log10(std::max(c, 1e-6));
}

// Coverage rule: the array can steer no further than the scan angle that
// corresponds to the nominal minimum elevation. It is evaluated on the scan
// angle itself, the quantity NtnOrbitPredictor's steered beam evaluates, so
// that truth and prediction disagree only where the orbit or the ephemeris
// does. Testing elevation here instead, on a spherical Earth, against a scan
// threshold derived on another Earth model, put a sub-degree bias between the
// two that a grazing pass turns into minutes of apparent prediction error.
bool
Covers(const Vector& ue, const Vector& sat)
{
    // F13: coverage ends at the minimum TOPOCENTRIC elevation (WGS-84 normal).
    // The predictor's steered beam is indexed by the same elevation
    // (SetSteeredBeamElevationIndexed), so truth and prediction cut at one
    // contour; the scan-angle rule this replaces put the edge near 24.9 deg.
    return ntngeo::GeodeticElevationDeg(ue, sat) >= g_p.minElevDeg;
}

// Realized exit: first time the satellite's TRUE orbit drops below the minimum
// elevation as seen from `ue` (held fixed over the horizon). Coarse step then
// bisection to 10 ms. Returns 0 when not covering now, horizon when no exit.
double
TrueExitS(uint32_t sat, const Vector& ue, double horizon)
{
    auto coveredAt = [&](double dt) {
        Vector p;
        Vector v;
        g_sats[sat]->GetEcefStateAt(Seconds(dt), p, v);
        return Covers(ue, p);
    };
    if (!coveredAt(0.0))
    {
        return 0.0;
    }
    double lo = 0.0;
    double hi = -1.0;
    for (double t = 1.0; t <= horizon; t += 1.0)
    {
        if (!coveredAt(t))
        {
            hi = t;
            break;
        }
        lo = t;
    }
    if (hi < 0.0)
    {
        return horizon;
    }
    while (hi - lo > 0.01)
    {
        const double m = 0.5 * (lo + hi);
        (coveredAt(m) ? lo : hi) = m;
    }
    return lo;
}

void
UpdateSatPositions()
{
    for (size_t s = 0; s < g_sats.size(); ++s)
    {
        g_satPos[s] = g_sats[s]->GetPosition();
    }
}

// One L1 measurement of every satellite the terminal can see.
void
MeasureUe(UeCtx& u, const Vector& uePos)
{
    // F14: TS 38.331 Section 5.5.3.2 defines a = 1/2^(k/4) for a 200 ms input
    // period and asks for the filter to be adapted so its time characteristics
    // are kept at other sampling rates: a_eff = 1 - (1 - a)^(T_s / 200 ms).
    const double a = NtnL3FilterCoefficient(g_p.l3FilterK, kTick);
    std::vector<uint32_t> gone;
    for (auto& [sat, ls] : u.links)
    {
        ls.covered = false;
    }
    for (size_t s = 0; s < g_satPos.size(); ++s)
    {
        const Vector& sp = g_satPos[s];
        // cheap reject: the satellite must be above the local horizon
        const double dot = (sp.x - uePos.x) * uePos.x + (sp.y - uePos.y) * uePos.y +
                           (sp.z - uePos.z) * uePos.z;
        if (dot <= 0.0)
        {
            continue;
        }
        if (!Covers(uePos, sp))
        {
            continue;
        }
        const double elev = ntngeo::GeodeticElevationDeg(uePos, sp);
        LinkState& ls = u.links[static_cast<uint32_t>(s)];
        ls.covered = true;
        ls.elevDeg = elev;
        ls.slantM = ntngeo::SlantRangeM(uePos, sp);
        ls.scanLossDb = ScanLossDb(uePos, sp);
        // line-of-sight state (blockage channels only)
        bool losChanged = false;
        const EnvTables& et = g_env[u.env];
        if (u.env != kEnvLos && g_p.losModel == "markov")
        {
            // M16: first-order Markov LOS state. The state decorrelates with
            // terminal displacement over losCorrM and with the satellite's
            // elevation change over losCorrDeg; when it decorrelates it is
            // redrawn from the marginal of the current bin, so the marginal
            // tracks TR 38.811 Table 6.6.1-1 and the correlation is
            // exp(-dx/d - de/theta). One uniform is drawn per link and tick
            // whatever the outcome, so the stream is policy-independent.
            const int bin = ElevBin(elev);
            double pre = 1.0;
            if (ls.losInit)
            {
                const double mx = uePos.x - ls.losUePos.x;
                const double my = uePos.y - ls.losUePos.y;
                const double mz = uePos.z - ls.losUePos.z;
                const double moved = std::sqrt(mx * mx + my * my + mz * mz);
                pre = 1.0 - std::exp(-moved / g_p.losCorrM - std::abs(elev - ls.losElev) / g_p.losCorrDeg);
            }
            const double x = g_uni->GetValue(0.0, 1.0);
            if (!ls.losInit || x < pre)
            {
                const bool was = ls.los;
                ls.los = g_uni->GetValue(0.0, 100.0) < et.plos->at(bin);
                if (g_p.dumpLosDraws)
                {
                    g_losDump << bin << "," << (ls.los ? 1 : 0) << "\n";
                }
                losChanged = ls.losInit && (was != ls.los);
                ls.losInit = true;
            }
            ls.losBin = bin;
            ls.losUePos = uePos;
            ls.losElev = elev;
        }
        else if (u.env != kEnvLos)
        {
            const int bin = ElevBin(elev);
            double moved = 0.0;
            if (ls.losInit)
            {
                const double mx = uePos.x - ls.losUePos.x;
                const double my = uePos.y - ls.losUePos.y;
                const double mz = uePos.z - ls.losUePos.z;
                moved = std::sqrt(mx * mx + my * my + mz * mz);
            }
            if (!ls.losInit || bin != ls.losBin || moved > g_p.losCorrM)
            {
                const bool was = ls.los;
                ls.los = g_uni->GetValue(0.0, 100.0) < et.plos->at(bin);
                if (g_p.dumpLosDraws)
                {
                    g_losDump << bin << "," << (ls.los ? 1 : 0) << "\n";
                }
                losChanged = ls.losInit && (was != ls.los);
                ls.losInit = true;
                ls.losBin = bin;
                ls.losUePos = uePos;
            }
        }
        // shadowing, correlated over terminal displacement; a change of
        // line-of-sight state starts an independent realization, as ns-3's
        // ThreeGppPropagationLossModel does
        const double sig = ls.los ? SfSigmaAt(*et.sfLos, elev) : et.sfNlos->at(ElevBin(elev));
        const double decorr = ls.los ? kSfDecorrM : kSfDecorrNlosM;
        // F12: the satellite's own motion decorrelates the shadowing too. A
        // change of the link's 10-degree elevation bin (the granularity of the
        // TR 38.811 tables and of the LOS process) starts a new realization at
        // that bin's sigma; for a static terminal it was otherwise frozen for a
        // whole pass.
        const int sfBinNow = ElevBin(elev);
        if (!ls.sfInit || losChanged || sfBinNow != ls.sfBin)
        {
            ls.sf = sig * g_norm->GetValue();
            ls.sfUePos = uePos;
            ls.sfInit = true;
            ls.sfBin = sfBinNow;
        }
        else
        {
            const double dx = uePos.x - ls.sfUePos.x;
            const double dy = uePos.y - ls.sfUePos.y;
            const double dz = uePos.z - ls.sfUePos.z;
            const double disp = std::sqrt(dx * dx + dy * dy + dz * dz);
            if (disp > 0.0)
            {
                const double r = std::exp(-disp / decorr);
                ls.sf = r * ls.sf + std::sqrt(1.0 - r * r) * sig * g_norm->GetValue();
                ls.sfUePos = uePos;
            }
        }
        const double rx = (ls.los ? g_plEnv[u.env] : g_plNlosEnv[u.env])->CalcRxPower(g_eirpDbm, g_sats[s], u.mob) +
                          ls.scanLossDb -
                          ls.sf - g_p.polLossDb;
        const double sinr = rx - g_noiseDbm + g_p.measErrDb * g_norm->GetValue();
        ls.l1 = sinr;
        ls.l3 = (ls.l3 <= kNegInf + 1.0) ? sinr : (1.0 - a) * ls.l3 + a * sinr;
    }
    for (auto& [sat, ls] : u.links)
    {
        if (!ls.covered)
        {
            gone.push_back(sat);
        }
    }
    for (uint32_t sat : gone)
    {
        // A satellite that left coverage keeps its entry only while it serves,
        // so the serving link can be seen to fail.
        if (static_cast<int32_t>(sat) != u.serving)
        {
            u.links.erase(sat);
        }
        else
        {
            // The serving satellite has left the coverage cone. The terminal
            // still measures, and what it measures is noise: feed that into the
            // L1 sample and the layer-3 filter so every trigger sees the link
            // collapse, rather than acting on the last value it had.
            LinkState& ls = u.links[sat];
            ls.l1 = kNoSignalDb + g_p.measErrDb * g_normOut->GetValue();
            ls.l3 = (1.0 - a) * ls.l3 + a * ls.l1;
        }
    }
}

int32_t
StrongestCovered(const UeCtx& u)
{
    int32_t best = -1;
    double bestL3 = kNegInf;
    for (const auto& [sat, ls] : u.links)
    {
        if (ls.covered && ls.l3 > bestL3)
        {
            bestL3 = ls.l3;
            best = static_cast<int32_t>(sat);
        }
    }
    return best;
}

// R2 (M9): the elevation a terminal evaluates for the elevation trigger is
// computed from its GNSS position and the broadcast ephemeris, so under
// ephemeris error it is the network copy's elevation, not the true one.
double
NetElevDeg(const UeCtx& u, uint32_t sat, const LinkState& ls)
{
    if (!g_ephActive || g_p.policy != "elev")
    {
        return ls.elevDeg;
    }
    Vector pp;
    Vector pv;
    if (!g_pred->GetPredictedSatEcef(sat, pp, pv))
    {
        return ls.elevDeg;
    }
    return ntngeo::GeodeticElevationDeg(u.tickPos, pp);
}

double
ExecMarginFor(const UeCtx& u)
{
    // The margin the network budgets for the serving cell's exit: execution
    // latency plus z sigma of the predicted exit, as NtnChoAlgorithm computes it.
    double m = g_p.execMarginS;
    const auto cfg = u.cho->GetConfig();
    if (cfg.ephSigmaRadialM > 0.0 || cfg.ephSigmaAlongM > 0.0 || cfg.ephSigmaCrossM > 0.0)
    {
        m += g_p.marginZ * u.tte->ComputeTteSigma(GeoOf(u.tickPos), u.tickVel,
                                                  static_cast<uint32_t>(u.serving), 0,
                                                  g_tteGainTh, cfg.ephSigmaRadialM,
                                                  cfg.ephSigmaAlongM, cfg.ephSigmaCrossM);
    }
    return m;
}

// CondEventT1 is keyed on an absolute epoch the network broadcasts. Set it from
// the network's own prediction of when the serving cell will be lost.
void
ArmT1(UeCtx& u)
{
    if (g_p.policy != "t1" || u.serving < 0)
    {
        return;
    }
    // F8: the position of the last tick; reading the mobility model here, at an
    // event time off the 100 ms grid, advanced it and shifted every later draw
    // of the shared engine for this policy only.
    const auto r = u.tte->ComputeTte(GeoOf(u.tickPos), u.tickVel,
                                     static_cast<uint32_t>(u.serving), 0, g_tteGainTh);
    const double now = Simulator::Now().GetSeconds();
    const double end = now + r.tte.GetSeconds() - ExecMarginFor(u);
    auto cfg = u.cho->GetConfig();
    const Time newEpoch = Seconds(std::max(now + 1e-3, end - g_p.t1DurationS));
    if (std::abs((newEpoch - cfg.t1ThresholdEpoch).GetSeconds()) > g_p.decisionPeriodS)
    {
        u.rrcDirty = true; // R2: a moved t1-Threshold is a new conditional reconfiguration
        u.rrcWinDirty = true;
    }
    cfg.t1ThresholdEpoch = newEpoch;
    cfg.t1WindowDuration = Seconds(g_p.t1DurationS);
    u.cho->Configure(cfg);
}

// F5: the network re-arms CondEventT1 from its current prediction while the
// window has not opened, and again after a window closed without execution.
void
RefreshT1(UeCtx& u)
{
    if (g_p.policy != "t1" || u.serving < 0 || u.executing)
    {
        return;
    }
    const double now = Simulator::Now().GetSeconds();
    const auto cfg = u.cho->GetConfig();
    const double open = cfg.t1ThresholdEpoch.GetSeconds();
    const double close = open + cfg.t1WindowDuration.GetSeconds();
    if (now < open || now >= close)
    {
        ArmT1(u);
        ++u.t1Rearms;
    }
}

void
Attach(UeCtx& u, int32_t sat)
{
    u.serving = sat;
    u.connectedSince = Simulator::Now().GetSeconds();
    u.t310Start = -1.0;
    u.floorSince = -1.0;
    // The serving cell has to be known to the algorithm with its geometry, so
    // the triggers that compare against it work from the first evaluation.
    u.cho->ClearCandidates();
    u.candidates.clear();
    u.cho->AddCandidateCell(CellOf(sat), sat, 0);
    const auto& ls = u.links[sat];
    u.cho->UpdateCandidateSlantRange(CellOf(sat), ls.slantM);
    u.cho->UpdateCandidateElevation(CellOf(sat), NetElevDeg(u, sat, ls));
    u.cho->UpdateMeasurement(CellOf(sat), ls.l3, -ls.scanLossDb);
    u.cho->SetServingCell(CellOf(sat));
    ArmT1(u);
}

void
StartReestablishment(UeCtx& u, const std::string& cause)
{
    const double now = Simulator::Now().GetSeconds();
    const int32_t from = u.serving;
    u.serving = -1;
    u.executing = false;
    u.reestablishing = true;
    u.t310Start = -1.0;
    const int32_t to = StrongestCovered(u);
    double delay = g_p.reestBaseMs / 1000.0;
    if (to >= 0)
    {
        delay += 4.0 * u.links[to].slantM / kC;
    }
    u.outageUntil = std::max(u.outageUntil, now + delay);
    g_events << std::fixed << std::setprecision(3) << now << "," << (&u - &g_ues[0]) << ","
             << from << "," << to << ",reest-" << cause << ",0,0,0,,,,,,,,,\n";
    const size_t idx = static_cast<size_t>(&u - &g_ues[0]);
    Simulator::Schedule(Seconds(delay), [idx]() {
        UeCtx& v = g_ues[idx];
        v.reestablishing = false;
        const int32_t best = StrongestCovered(v);
        if (best >= 0)
        {
            Attach(v, best);
        }
    });
}

// Trigger kind of a handover (R2, m5): planned exit of the exit-triggered
// family, its safety-net escape, a standardized condition, or the
// standardized conditions' measurement fallback.
const char* const kTrigName[] = {"planned", "escape", "cond", "netesc"};

void
CompleteHandover(size_t idx, uint32_t target, double tStart, double predServTte,
                 double margin, double trueServExit, double tgtTte, double tgtL3, int tgtRank,
                 int trig)
{
    UeCtx& u = g_ues[idx];
    if (!u.executing)
    {
        return; // superseded by a radio link failure during execution
    }
    u.executing = false;
    const double now = Simulator::Now().GetSeconds();
    auto it = u.links.find(target);
    const bool ok = (it != u.links.end() && it->second.covered && it->second.l1 >= g_p.qOutDb);
    const int32_t from = u.serving;
    const bool pp1 = (static_cast<int32_t>(target) == u.prevServing && tStart - u.lastHoTime <= 1.0);
    const bool pp5 = (static_cast<int32_t>(target) == u.prevServing && tStart - u.lastHoTime <= 5.0);
    g_events << std::fixed << std::setprecision(3) << tStart << "," << idx << "," << from << ","
             << target << ",ho," << (ok ? 1 : 0) << "," << (pp1 ? 1 : 0) << "," << (pp5 ? 1 : 0)
             << "," << std::setprecision(2) << predServTte << "," << margin << ","
             << trueServExit << "," << tgtTte << ","
             << (it != u.links.end() ? it->second.elevDeg : -90.0) << ","
             << (now - tStart) * 1000.0 << "," << tgtL3 << "," << tgtRank << ","
             << kTrigName[trig] << "\n";
    u.cho->NotifyHandoverComplete(CellOf(target), ok);
    if (!ok)
    {
        ++u.hof;
        StartReestablishment(u, "hof");
        return;
    }
    ++u.hoSuccess;
    if (pp1)
    {
        ++u.pp1;
    }
    if (pp5)
    {
        ++u.pp5;
    }
    u.tos.push_back(tStart - u.connectedSince);
    u.prevServing = from;
    u.lastHoTime = tStart;
    u.serving = static_cast<int32_t>(target);
    u.connectedSince = now;
    u.t310Start = -1.0;
    ArmT1(u);
}

void
SyncCandidates(UeCtx& u)
{
    // The prepared set: the strongest covered satellites other than the
    // serving one, as a terminal's measurement report would rank them.
    std::vector<std::pair<double, uint32_t>> ranked;
    std::vector<std::pair<double, uint32_t>> strongest; // by L3, for reserved slots
    std::map<uint32_t, std::pair<double, double>> pred; // N2: (tte, usable) per satellite
    for (const auto& [sat, ls] : u.links)
    {
        if (!ls.covered || static_cast<int32_t>(sat) == u.serving)
        {
            continue;
        }
        strongest.emplace_back(ls.l3, sat);
        if (g_p.prepareByTte)
        {
            // N2: the network ranks the covering satellites that meet the
            // quality floor by the same predicted score the rule ranks
            // targets with, so the satellite that will last longest is
            // prepared even when it is the weakest.
            if (ls.l3 < g_p.qualityThDb)
            {
                continue;
            }
            const auto r = u.tte->ComputeTte(GeoOf(u.tickPos), u.tickVel, sat, 0, g_tteGainTh);
            pred[sat] = {r.tte.GetSeconds(), r.usable.GetSeconds()};
            ranked.emplace_back(g_p.rankByUsable ? r.usable.GetSeconds() : r.tte.GetSeconds(), sat);
        }
        else
        {
            if (g_p.prepAboveContour)
            {
                // M7: an operator preparing a condition that executes at the
                // e_Q contour prepares only satellites predicted to stay
                // inside that contour for at least tau_min.
                const auto r = u.tte->ComputeTte(GeoOf(u.tickPos), u.tickVel, sat, 0, g_tteGainTh);
                if (r.tte.GetSeconds() < g_p.tteMinS)
                {
                    continue;
                }
            }
            ranked.emplace_back(ls.l3, sat);
        }
    }
    std::sort(ranked.rbegin(), ranked.rend());
    std::set<uint32_t> want;
    // The prepared set serves two purposes: the planned target, which the
    // predicted score picks, and the escape target the safety net needs when a
    // link is blocked, which is a strong satellite now. Preparing by the
    // score alone left no strong satellite for the escape, and the terminal
    // fell through to radio link failure. prepStrongest reserves that many
    // slots for the strongest covering satellites.
    const uint32_t reserve = (g_p.prepareByTte || g_p.prepAboveContour)
                                 ? std::min<uint32_t>(g_p.prepStrongest, g_p.maxCandidates)
                                 : 0;
    for (size_t k = 0; k < ranked.size() && want.size() < g_p.maxCandidates - reserve; ++k)
    {
        want.insert(ranked[k].second);
    }
    if (reserve > 0)
    {
        std::sort(strongest.rbegin(), strongest.rend());
        for (size_t k = 0; k < strongest.size() && want.size() < g_p.maxCandidates; ++k)
        {
            want.insert(strongest[k].second);
        }
    }
    for (auto it = u.candidates.begin(); it != u.candidates.end();)
    {
        if (want.count(*it) == 0)
        {
            u.cho->RemoveCandidateCell(CellOf(*it));
            it = u.candidates.erase(it);
            u.rrcDirty = true;
            u.rrcCandDirty = true;
        }
        else
        {
            ++it;
        }
    }
    // The algorithm's own map may hold the cell it just left (it re-arms it).
    const auto held = u.cho->GetCandidates();
    for (const auto& [cell, info] : held)
    {
        const uint32_t sat = SatOf(cell);
        if (static_cast<int32_t>(sat) != u.serving && want.count(sat) == 0)
        {
            u.cho->RemoveCandidateCell(cell);
        }
    }
    for (uint32_t sat : want)
    {
        if (u.candidates.count(sat) == 0)
        {
            u.cho->AddCandidateCell(CellOf(sat), sat, 0);
            u.candidates.insert(sat);
            u.rrcDirty = true;
            u.rrcCandDirty = true;
        }
        const auto pr = pred.find(sat);
        if (pr != pred.end())
        {
            u.cho->SeedCandidatePrediction(CellOf(sat), Seconds(pr->second.first),
                                           Seconds(pr->second.second));
        }
    }
}

void
Decide(size_t idx)
{
    UeCtx& u = g_ues[idx];
    if (u.serving < 0 || u.executing || u.reestablishing)
    {
        return;
    }
    const Vector pos = u.tickPos;
    const GeoCoordinate geo = GeoOf(pos);
    u.cho->UpdateUeKinematics(geo, u.tickVel);
    const auto sv = u.links.find(static_cast<uint32_t>(u.serving));
    if (sv == u.links.end())
    {
        return;
    }
    u.cho->UpdateCandidateSlantRange(CellOf(u.serving), sv->second.slantM);
    u.cho->UpdateCandidateElevation(CellOf(u.serving),
                                    NetElevDeg(u, static_cast<uint32_t>(u.serving), sv->second));
    u.cho->UpdateMeasurement(CellOf(u.serving), sv->second.l3, -sv->second.scanLossDb);
    for (auto it = u.candidates.begin(); it != u.candidates.end();)
    {
        auto ls = u.links.find(*it);
        if (ls == u.links.end() || !ls->second.covered)
        {
            // Left coverage since the last refresh: a prepared candidate the
            // terminal can no longer measure is released, not kept at its
            // last value.
            u.cho->RemoveCandidateCell(CellOf(*it));
            it = u.candidates.erase(it);
            u.rrcDirty = true;
            u.rrcCandDirty = true;
            continue;
        }
        u.cho->UpdateCandidateSlantRange(CellOf(*it), ls->second.slantM);
        u.cho->UpdateCandidateElevation(CellOf(*it), NetElevDeg(u, *it, ls->second));
        u.cho->UpdateMeasurement(CellOf(*it), ls->second.l3, -ls->second.scanLossDb);
        ++it;
    }
    ++u.decisions;
    u.cho->EvaluateConditions();
    uint16_t target = u.cho->SelectBestCandidate();
    const bool isTteFamily = (g_p.policy == "tte-cho" || g_p.policy == "mst" || g_p.policy == "tte-eager");
    bool viaNet = isTteFamily && u.cho->LastSelectionViaSafetyNet();
    if (isTteFamily && g_p.policy != "tte-eager")
    {
        // R2 (M6): signaled as a CondEventT1 window on one candidate, the rule
        // needs a new conditional reconfiguration whenever its planned target
        // changes or the window end moves by more than a decision period.
        const double nowS = Simulator::Now().GetSeconds();
        const uint16_t pt = u.cho->GetPlannedTarget();
        const double end = nowS + u.cho->GetLastServingTte().GetSeconds() -
                           u.cho->GetLastExitMargin().GetSeconds();
        if (pt != NtnChoAlgorithm::INVALID_CELL_ID &&
            (pt != u.sigTarget || std::abs(end - u.sigEnd) > g_p.decisionPeriodS + 1e-9))
        {
            u.rrcDirty = true;
            u.rrcWinDirty = true;
            u.sigTarget = pt;
            u.sigEnd = end;
        }
    }
    if (g_p.tteRank && !isTteFamily && target != NtnChoAlgorithm::INVALID_CELL_ID)
    {
        // Same execution instant, different target: longest predicted exit
        // among the admitted candidates that pass the quality floor.
        double bestTte = -1.0;
        for (const auto& [cellId, info] : u.cho->GetCandidates())
        {
            if (!info.admitted || info.sinr_dB < g_p.qualityThDb ||
                SatOf(cellId) == static_cast<uint32_t>(u.serving))
            {
                continue;
            }
            const double tt =
                u.tte->ComputeTte(geo, u.tickVel, SatOf(cellId), 0, g_tteGainTh).tte.GetSeconds();
            if (tt > bestTte)
            {
                bestTte = tt;
                target = cellId;
            }
        }
    }
    if (g_p.measFallback && !isTteFamily)
    {
        const double now = Simulator::Now().GetSeconds();
        if (sv->second.l3 < g_p.tteFloorDb)
        {
            if (u.floorSince < 0.0)
            {
                u.floorSince = now;
            }
        }
        else
        {
            u.floorSince = -1.0;
        }
        const bool lost = u.floorSince >= 0.0 && (now - u.floorSince) * 1e3 >= g_p.tteFloorTttMs - 1e-6;
        // Escape hold-off for the standardized conditions (the same rule the
        // exit-triggered policy applies): the cell left through the fallback is
        // not a target for escapeHoldoffS unless it is the only candidate.
        const bool holdOn = g_p.escapeHoldoffS > 0.0 && u.escCell >= 0 && now < u.escUntil;
        if (holdOn && !lost && target != NtnChoAlgorithm::INVALID_CELL_ID &&
            static_cast<int32_t>(SatOf(target)) == u.escCell)
        {
            target = NtnChoAlgorithm::INVALID_CELL_ID; // the condition's own choice, held off
        }
        if (lost && (target == NtnChoAlgorithm::INVALID_CELL_ID ||
                     SatOf(target) == static_cast<uint32_t>(u.serving)))
        {
            double best = -1e9;
            int32_t heldBest = -1;
            for (uint32_t s : u.candidates)
            {
                const auto ls = u.links.find(s);
                // F4: the same quality floor Q as TTE-CHO's own net
                if (s != static_cast<uint32_t>(u.serving) && ls != u.links.end() && ls->second.covered &&
                    ls->second.l3 >= g_p.qualityThDb && ls->second.l3 > sv->second.l3 &&
                    ls->second.l3 > best)
                {
                    if (holdOn && static_cast<int32_t>(s) == u.escCell)
                    {
                        heldBest = static_cast<int32_t>(s);
                        continue;
                    }
                    best = ls->second.l3;
                    target = CellOf(s);
                }
            }
            if (best < -1e8 && heldBest >= 0)
            {
                target = CellOf(static_cast<uint32_t>(heldBest)); // the only candidate
            }
            if (target != NtnChoAlgorithm::INVALID_CELL_ID)
            {
                viaNet = true;
            }
            if (target != NtnChoAlgorithm::INVALID_CELL_ID && g_p.escapeHoldoffS > 0.0)
            {
                u.escCell = u.serving;
                u.escUntil = now + g_p.escapeHoldoffS;
            }
        }
    }
    if (target == NtnChoAlgorithm::INVALID_CELL_ID || SatOf(target) == static_cast<uint32_t>(u.serving))
    {
        return;
    }
    u.floorSince = -1.0;
    // Record what the network predicted for the cell it is leaving, and what
    // will actually happen to it, before executing.
    const double predServ =
        u.tte->ComputeTte(geo, u.tickVel, static_cast<uint32_t>(u.serving), 0, g_tteGainTh)
            .tte.GetSeconds();
    const double margin = (g_p.policy == "tte-cho" || g_p.policy == "mst" || g_p.policy == "t1")
                              ? ExecMarginFor(u)
                              : 0.0;
    const double trueServ = TrueExitS(static_cast<uint32_t>(u.serving), pos, g_p.horizonS);
    const double tgtTte = u.cho->GetCandidateTte(target).GetSeconds();
    const auto tl = u.links.find(SatOf(target));
    const double tgtL3 = (tl != u.links.end()) ? tl->second.l3 : kNegInf;
    // Diagnostic: rank of the target's predicted exit among ALL covering
    // satellites (1 = the greedy choice over the whole sky, not only the
    // prepared set).
    int tgtRank = 1;
    {
        const double mine =
            u.tte->ComputeTte(geo, u.tickVel, SatOf(target), 0, g_tteGainTh).tte.GetSeconds();
        for (const auto& [sat, ls] : u.links)
        {
            if (!ls.covered || static_cast<int32_t>(sat) == u.serving || sat == SatOf(target))
            {
                continue;
            }
            if (u.tte->ComputeTte(geo, u.tickVel, sat, 0, g_tteGainTh).tte.GetSeconds() > mine + 1e-6)
            {
                ++tgtRank;
            }
        }
    }
    ++u.hoAttempts;
    const int trig = isTteFamily ? (viaNet ? 1 : 0) : (viaNet ? 3 : 2);
    if (viaNet)
    {
        ++u.hoEscape;
    }
    else
    {
        ++u.hoPlanned;
    }
    u.executing = true;
    u.cho->ExecuteHandover(target);
    // Scheduled and booked at the same nanosecond resolution: MilliSeconds()
    // on the double truncated the completion to whole milliseconds while the
    // booked outage kept the fraction.
    const Time intT = NanoSeconds(std::llround(u.cho->GetMechanismStats().lastInterruptionMs * 1e6));
    const double intMs = intT.GetSeconds() * 1000.0;
    const double now = Simulator::Now().GetSeconds();
    u.outageUntil = std::max(u.outageUntil, now + intMs / 1000.0);
    // F1: the interruption starts after this tick's outage check and ends
    // before the next tick, so tick sampling never saw it. Book its exact
    // duration here and keep the ticks it spans out of both the outage count
    // and the SINR samples.
    u.outageS += intMs / 1000.0;
    u.intUntil = now + intMs / 1000.0;
    Simulator::Schedule(intT, &CompleteHandover, idx, SatOf(target), now, predServ,
                        margin, trueServ, tgtTte, tgtL3, tgtRank, trig);
}

void
Tick()
{
    const double now = Simulator::Now().GetSeconds();
    if (now > g_p.simTime)
    {
        return;
    }
    UpdateSatPositions();
    const bool decision = std::fmod(now + 1e-9, g_p.decisionPeriodS) < kTick - 1e-9;
    const bool refresh = std::fmod(now + 1e-9, g_p.candRefreshS) < kTick - 1e-9;
    for (size_t i = 0; i < g_ues.size(); ++i)
    {
        UeCtx& u = g_ues[i];
        const Vector pos = u.mob->GetPosition();
        u.tickPos = pos;
        u.tickVel = u.mob->GetVelocity();
        MeasureUe(u, pos);
        if (static_cast<int32_t>(i) == g_p.dumpLinksUe)
        {
            for (const auto& [sat, ls] : u.links)
            {
                if (ls.covered)
                {
                    g_dump << std::fixed << std::setprecision(1) << now << "," << sat << ","
                           << (ls.los ? 1 : 0) << "," << std::setprecision(6) << ls.sf << ","
                           << ls.l1 << "\n";
                }
            }
        }
        if (u.serving < 0)
        {
            if (!u.reestablishing)
            {
                const int32_t best = StrongestCovered(u);
                if (best >= 0)
                {
                    Attach(u, best);
                }
                else
                {
                    u.outageS += kTick; // nothing in coverage at all
                    continue;
                }
            }
            else
            {
                u.outageS += kTick;
                continue;
            }
        }
        LinkState& sl = u.links[static_cast<uint32_t>(u.serving)];
        const double l1 = sl.l1;
        // T310: started below Qout, stopped at or above Qin.
        if (l1 < g_p.qOutDb && u.t310Start < 0.0)
        {
            u.t310Start = now;
        }
        else if (l1 >= g_p.qInDb)
        {
            u.t310Start = -1.0;
        }
        if (u.t310Start >= 0.0 && now - u.t310Start >= g_p.t310S - 1e-9)
        {
            ++u.rlf;
            if (u.executing)
            {
                u.executing = false;
                u.cho->NotifyHandoverComplete(CellOf(u.serving), false);
            }
            StartReestablishment(u, "rlf");
            u.outageS += kTick;
            continue;
        }
        const bool inOutage = (now < u.outageUntil) || l1 < g_p.qOutDb;
        if (now < u.intUntil - 1e-9)
        {
            // F1: inside an interruption already booked at execution
        }
        else if (inOutage)
        {
            u.outageS += kTick;
        }
        else
        {
            u.sinrSum += l1;
            u.seSum += std::min(std::log2(1.0 + std::pow(10.0, l1 / 10.0)), 5.55);
            ++u.samples;
        }
        if (static_cast<int32_t>(i) == g_p.traceUe && refresh)
        {
            // one row per covering satellite: what the terminal sees, and which
            // cell it is on, once a second
            for (const auto& [sat, ls] : u.links)
            {
                if (ls.covered)
                {
                    g_trace << std::fixed << std::setprecision(1) << now << "," << sat << ","
                            << std::setprecision(2) << ls.elevDeg << "," << ls.l3 << ","
                            << (static_cast<int32_t>(sat) == u.serving ? 1 : 0) << ","
                            << (ls.los ? 1 : 0) << "\n";
                }
            }
        }
        if (refresh && !u.executing)
        {
            SyncCandidates(u);
            RefreshT1(u);
        }
        if (decision)
        {
            Decide(i);
        }
        if (u.rrcDirty)
        {
            ++u.rrc;
            u.rrcCand += u.rrcCandDirty ? 1 : 0;
            u.rrcWin += u.rrcWinDirty ? 1 : 0;
            u.rrcDirty = u.rrcCandDirty = u.rrcWinDirty = false;
        }
    }
    Simulator::Schedule(Seconds(kTick), &Tick);
}

// Offline lower bound on handovers for a static terminal: greedy longest
// remaining coverage on the TRUE orbits (interval covering), from t = 0.
uint32_t
LowerBoundHandovers(const Vector& ue)
{
    uint32_t hos = 0;
    double t = 0.0;
    bool first = true;
    while (t < g_p.simTime)
    {
        double bestExit = -1.0;
        for (size_t s = 0; s < g_sats.size(); ++s)
        {
            Vector p;
            Vector v;
            g_sats[s]->GetEcefStateAt(Seconds(t), p, v);
            if (!Covers(ue, p))
            {
                continue;
            }
            // remaining coverage from t: scan the true orbit forward
            double lo = t;
            double hi = -1.0;
            for (double x = t + 2.0; x <= t + g_p.horizonS; x += 2.0)
            {
                g_sats[s]->GetEcefStateAt(Seconds(x), p, v);
                if (!Covers(ue, p))
                {
                    hi = x;
                    break;
                }
                lo = x;
            }
            const double exitT = (hi < 0.0) ? t + g_p.horizonS : lo;
            bestExit = std::max(bestExit, exitT);
        }
        if (bestExit <= t)
        {
            t += 1.0; // coverage hole: wait
            continue;
        }
        if (!first)
        {
            ++hos;
        }
        first = false;
        t = bestExit;
    }
    return hos;
}

// Accuracy mode: predicted vs realized time-to-exit for random visible pairs.
void
RunAccuracy()
{
    UpdateSatPositions();
    // F16: draw (terminal, satellite) pairs uniformly at random among the
    // covering ones. Enumerating them in index order filled the sample budget
    // from the first terminals only.
    std::vector<std::pair<size_t, size_t>> pairs;
    for (size_t i = 0; i < g_ues.size(); ++i)
    {
        const Vector pos = g_ues[i].mob->GetPosition();
        for (size_t s = 0; s < g_satPos.size(); ++s)
        {
            if (Covers(pos, g_satPos[s]))
            {
                pairs.emplace_back(i, s);
            }
        }
    }
    Ptr<UniformRandomVariable> pick = CreateObject<UniformRandomVariable>();
    pick->SetStream(4000);
    for (size_t k = pairs.size(); k > 1; --k)
    {
        const size_t j = static_cast<size_t>(pick->GetInteger(0, static_cast<uint32_t>(k - 1)));
        std::swap(pairs[k - 1], pairs[j]);
    }
    const size_t n = std::min<size_t>(pairs.size(), g_p.accuracySamples);
    for (size_t q = 0; q < n; ++q)
    {
        const size_t i = pairs[q].first;
        const size_t s = pairs[q].second;
        UeCtx& u = g_ues[i];
        const Vector pos = u.mob->GetPosition();
        const GeoCoordinate geo = GeoOf(pos);
        const double elev = ntngeo::GeodeticElevationDeg(pos, g_satPos[s]);
        const auto c0 = std::chrono::steady_clock::now();
        const auto r = g_tte->ComputeTte(geo, Vector(0, 0, 0), static_cast<uint32_t>(s), 0,
                                         g_tteGainTh);
        const double us =
            std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - c0)
                .count();
        const double truth = TrueExitS(static_cast<uint32_t>(s), pos, g_p.horizonS);
        const double sigma = g_tte->ComputeTteSigma(
            geo, Vector(0, 0, 0), static_cast<uint32_t>(s), 0, g_tteGainTh,
            g_p.ephSigmaRadialM, g_p.ephSigmaAlongM, g_p.ephSigmaCrossM);
        // geometry descriptor: the pass's peak elevation, from the true orbit,
        // scanned over the whole pass
        double peak = elev;
        for (double x = -g_p.horizonS; x <= truth; x += 2.0)
        {
            Vector p;
            Vector v;
            g_sats[s]->GetEcefStateAt(Seconds(x), p, v);
            peak = std::max(peak, ntngeo::GeodeticElevationDeg(pos, p));
        }
        const Vector e = g_ephDrawn[s];
        g_acc << i << "," << s << "," << std::fixed << std::setprecision(3) << elev << ","
              << peak << "," << r.tte.GetSeconds() << "," << truth << "," << sigma << ","
              << e.x << "," << e.y << "," << e.z << "," << us << "\n";
    }
}

// R2 (M10): the network's copy of a satellite is an orbit of its own. Its
// elements are those of the true (circular) orbit perturbed so that at t = 0
// the copy sits at the drawn radial / along-track / cross-track offset, and
// from then on it is propagated by the same Kepler + J2 model, never
// re-anchored to the truth. The along-track offset is a shift of the argument
// of latitude; the cross-track one a tilt of the orbit plane about the
// satellite's position (inclination and node changed together); the radial
// one a change of the semi-major axis, so that, as for a real element set, a
// radial (energy) error turns into an along-track error growing at
// -1.5 n da per second. When a measured along-track growth rate is supplied,
// the semi-major-axis error is set from it instead (da = -(2/3) rate / n) and
// the rest of the radial offset is carried by a small eccentricity with the
// apse at the satellite.
KeplerianElements
PerturbedCopy(const KeplerianElements& t, double er, double ea, double ec, double rateA, bool useRate)
{
    KeplerianElements k = t;
    const double a = t.semi_major_axis_m;
    const double n = std::sqrt(ntncon::kEarthMuM3S2 / (a * a * a));
    const double u0 = t.arg_perigee_rad + t.mean_anomaly_rad; // circular truth
    const double da = useRate ? -(2.0 / 3.0) * rateA / n : er;
    const double si = std::sin(t.inclination_rad);
    const double dI = (ec / a) * std::sin(u0);
    const double dO = (si > 1e-6) ? -(ec / a) * std::cos(u0) / si : 0.0;
    const double du = ea / a - std::cos(t.inclination_rad) * dO;
    k.semi_major_axis_m = a + da;
    k.inclination_rad = t.inclination_rad + dI;
    k.raan_rad = t.raan_rad + dO;
    const double rem = er - da;
    if (std::abs(rem) > 1e-3)
    {
        k.eccentricity = std::abs(rem) / (a + da);
        // perigee (rem < 0) or apogee (rem > 0) at the satellite's position
        k.arg_perigee_rad = u0 + du - (rem > 0.0 ? M_PI : 0.0);
        k.mean_anomaly_rad = (rem > 0.0 ? M_PI : 0.0);
    }
    else
    {
        k.mean_anomaly_rad = t.mean_anomaly_rad + du;
    }
    return k;
}

// Copy minus truth, resolved in the true orbit's RTN frame (inertial velocity).
Vector
RtnDiff(const Vector& tp, const Vector& ve, const Vector& pp)
{
    constexpr double kOmegaE = 7.2921159e-5;
    const Vector tv(ve.x - kOmegaE * tp.y, ve.y + kOmegaE * tp.x, ve.z);
    const double rn = tp.GetLength();
    const Vector R(tp.x / rn, tp.y / rn, tp.z / rn);
    Vector N(R.y * tv.z - R.z * tv.y, R.z * tv.x - R.x * tv.z, R.x * tv.y - R.y * tv.x);
    const double nn = N.GetLength();
    N = Vector(N.x / nn, N.y / nn, N.z / nn);
    const Vector T(N.y * R.z - N.z * R.y, N.z * R.x - N.x * R.z, N.x * R.y - N.y * R.x);
    const Vector d(pp.x - tp.x, pp.y - tp.y, pp.z - tp.z);
    return Vector(d.x * R.x + d.y * R.y + d.z * R.z, d.x * T.x + d.y * T.y + d.z * T.z,
                  d.x * N.x + d.y * N.y + d.z * N.z);
}

// Element sets of a TLE file, keyed by catalog number.
std::map<std::string, TleRecord>
ReadTleFile(const std::string& path)
{
    std::ifstream f(path);
    NS_ABORT_MSG_IF(!f.good(), "cannot read " << path);
    std::stringstream ss;
    ss << f.rdbuf();
    std::vector<TleRecord> recs;
    ntncon::ParseTleStream(ss.str(), recs);
    std::map<std::string, TleRecord> out;
    for (const auto& r : recs)
    {
        if (r.line1.size() >= 7)
        {
            std::string cat = r.line1.substr(2, 5);
            cat.erase(0, cat.find_first_not_of(' '));
            out[cat] = r;
        }
    }
    return out;
}

} // namespace

int
main(int argc, char* argv[])
{
    Params& p = g_p;
    CommandLine cmd(__FILE__);
    cmd.AddValue("policy", "a3 | d2 | t1 | elev | tte-eager | mst | tte-cho", p.policy);
    cmd.AddValue("mode", "campaign | tte-accuracy", p.mode);
    cmd.AddValue("numUes", "terminals", p.numUes);
    cmd.AddValue("simTime", "simulated time (s)", p.simTime);
    cmd.AddValue("rngRun", "RNG run (seed index)", p.rngRun);
    cmd.AddValue("outputDir", "output directory", p.outputDir);
    cmd.AddValue("totalSats", "Walker T", p.totalSats);
    cmd.AddValue("numPlanes", "Walker P", p.numPlanes);
    cmd.AddValue("phasing", "Walker F", p.phasing);
    cmd.AddValue("altitudeKm", "shell altitude (km)", p.altitudeKm);
    cmd.AddValue("inclinationDeg", "shell inclination (deg)", p.inclinationDeg);
    cmd.AddValue("centerLat", "service-area centre latitude", p.centerLat);
    cmd.AddValue("centerLon", "service-area centre longitude", p.centerLon);
    cmd.AddValue("halfSpanDeg", "service-area half span (deg)", p.halfSpanDeg);
    cmd.AddValue("ueProfile", "mixed | static", p.ueProfile);
    cmd.AddValue("channel", "los | suburban | urban | denseurban (TR 38.811 Table 6.6.1-1 LOS/NLOS process)", p.channel);
    cmd.AddValue("minElevDeg", "minimum service elevation (deg)", p.minElevDeg);
    cmd.AddValue("measErrDb", "L1 measurement error sigma (dB)", p.measErrDb);
    cmd.AddValue("qualityThDb", "candidate quality floor (dB)", p.qualityThDb);
    cmd.AddValue("maxCandidates", "prepared candidates", p.maxCandidates);
    cmd.AddValue("a3OffsetDb", "A3 offset (dB)", p.a3OffsetDb);
    cmd.AddValue("a3TttMs", "A3 time to trigger (ms)", p.a3TttMs);
    cmd.AddValue("d2ServElevDeg", "D2 Thresh1 as the serving elevation it corresponds to", p.d2ServElevDeg);
    cmd.AddValue("d2CandElevDeg", "D2 Thresh2 as the candidate elevation it corresponds to", p.d2CandElevDeg);
    cmd.AddValue("t1DurationS", "T1 window duration (s)", p.t1DurationS);
    cmd.AddValue("elevFloorDeg", "elevation-trigger floor (deg)", p.elevFloorDeg);
    cmd.AddValue("tteMinS", "minimum candidate time-to-exit (s)", p.tteMinS);
    cmd.AddValue("tteFloorDb", "serving-loss safety-net threshold (dB)", p.tteFloorDb);
    cmd.AddValue("tteFloorTttMs", "time-to-trigger of the safety net (ms)", p.tteFloorTttMs);
    cmd.AddValue("tteExitElevDeg",
                 "elevation at which a cell stops meeting the quality the prediction-based "
                 "policies plan for (-1 = the coverage limit)",
                 p.tteExitElevDeg);
    cmd.AddValue("execMarginS", "exit margin for execution latency (s)", p.execMarginS);
    cmd.AddValue("marginZ", "sigma multiplier on the exit-time margin", p.marginZ);
    cmd.AddValue("predStepS", "TTE coarse search step (s)", p.predStepS);
    cmd.AddValue("horizonS", "TTE prediction horizon (s)", p.horizonS);
    cmd.AddValue("ephSigmaRadialM", "true ephemeris error sigma, radial (m)", p.ephSigmaRadialM);
    cmd.AddValue("ephSigmaAlongM", "true ephemeris error sigma, along-track (m)", p.ephSigmaAlongM);
    cmd.AddValue("ephSigmaCrossM", "true ephemeris error sigma, cross-track (m)", p.ephSigmaCrossM);
    cmd.AddValue("assumedSigmaRadialM", "network-assumed sigma, radial (m); -1 = true", p.assumedSigmaRadialM);
    cmd.AddValue("assumedSigmaAlongM", "network-assumed sigma, along (m); -1 = true", p.assumedSigmaAlongM);
    cmd.AddValue("assumedSigmaCrossM", "network-assumed sigma, cross (m); -1 = true", p.assumedSigmaCrossM);
    cmd.AddValue("accuracySamples", "samples in tte-accuracy mode", p.accuracySamples);
    cmd.AddValue("traceUe", "terminal index to trace at 1 s (-1 = none)", p.traceUe);
    cmd.AddValue("measFallback", "serving-floor fallback for the standardized conditions", p.measFallback);
    cmd.AddValue("tteRank", "rank the admitted candidates by predicted time to exit", p.tteRank);
    cmd.AddValue("decisionPeriodS", "decision period (s); default = the 100 ms measurement period",
                 p.decisionPeriodS);
    cmd.AddValue("tteCacheS", "candidate prediction cache refresh (s); 0 = recompute always",
                 p.tteCacheS);
    cmd.AddValue("rankByUsable", "N1: rank targets by expected usable time to exit", p.rankByUsable);
    cmd.AddValue("prepStrongest", "prepared slots reserved for the strongest satellites (prepareByTte)", p.prepStrongest);
    cmd.AddValue("escapeNearBestDb", "escape target within this margin of the strongest candidate (dB, -1 off)", p.escapeNearBestDb);
    cmd.AddValue("prepareByTte", "N2: prepare candidates by predicted score instead of by L3",
                 p.prepareByTte);
    cmd.AddValue("escapeHoldoffS", "N3: hold-off (s) of the cell left via the safety net",
                 p.escapeHoldoffS);
    cmd.AddValue("dumpLinksUe", "write per-tick link realizations of this terminal", p.dumpLinksUe);
    cmd.AddValue("dumpLosDraws", "write every LOS-state draw to los_draws.csv", p.dumpLosDraws);
    cmd.AddValue("ephErrorFile", "N4: CSV of measured RTN errors (catnr,name,alt_km,age_h,dt_min,"
                 "err_r_m,err_a_m,err_c_m); replaces the Gaussian draw", p.ephErrorFile);
    cmd.AddValue("ephAgeMinH", "N4: minimum element-set age (h) of the drawn samples", p.ephAgeMinH);
    cmd.AddValue("ephAgeMaxH", "N4: maximum element-set age (h), exclusive", p.ephAgeMaxH);
    cmd.AddValue("reestBaseMs", "re-establishment base time (ms), before the RACH over the slant", p.reestBaseMs);
    cmd.AddValue("classEnv", "R2: per-class environment (aircraft/ship LOS, train rural)", p.classEnv);
    cmd.AddValue("losModel", "R2: LOS-state process, redraw | markov", p.losModel);
    cmd.AddValue("losCorrM", "R2: LOS-state correlation distance (m)", p.losCorrM);
    cmd.AddValue("losCorrDeg", "R2: markov LOS angular decorrelation (deg)", p.losCorrDeg);
    cmd.AddValue("ephModel", "R2: network ephemeris copy, propagated | offset", p.ephModel);
    cmd.AddValue("escapeByQuality", "R2: safety-net escape to the strongest candidate better than serving",
                 p.escapeByQuality);
    cmd.AddValue("prepAboveContour", "R2: prepare only candidates predicted inside e_Q for tau_min",
                 p.prepAboveContour);
    cmd.AddValue("prepStrongestStd", "alias kept for scripts: same as prepStrongest", p.prepStrongest);
    cmd.AddValue("tleTruthFile", "R2: truth element sets (real constellation)", p.tleTruthFile);
    cmd.AddValue("tleNetFile", "R2: network element sets", p.tleNetFile);
    cmd.AddValue("tleInclMinDeg", "R2: TLE shell filter, inclination min", p.tleInclMinDeg);
    cmd.AddValue("tleInclMaxDeg", "R2: TLE shell filter, inclination max", p.tleInclMaxDeg);
    cmd.AddValue("tleAltMinKm", "R2: TLE shell filter, mean altitude min", p.tleAltMinKm);
    cmd.AddValue("tleAltMaxKm", "R2: TLE shell filter, mean altitude max", p.tleAltMaxKm);
    cmd.AddValue("tleStartSpreadH", "R2: start within +- this of the median truth epoch (h)", p.tleStartSpreadH);
    cmd.Parse(argc, argv);
    NS_ABORT_MSG_IF(p.losModel != "redraw" && p.losModel != "markov", "unknown --losModel");
    NS_ABORT_MSG_IF(p.ephModel != "propagated" && p.ephModel != "offset", "unknown --ephModel");

    if (!p.ephErrorFile.empty())
    {
        // The Gaussian sigmas are not used in file mode; what the network
        // budgets for has to be given explicitly (zero when it is not).
        p.assumedSigmaRadialM = std::max(0.0, p.assumedSigmaRadialM);
        p.assumedSigmaAlongM = std::max(0.0, p.assumedSigmaAlongM);
        p.assumedSigmaCrossM = std::max(0.0, p.assumedSigmaCrossM);
    }
    if (p.assumedSigmaRadialM < 0.0)
    {
        p.assumedSigmaRadialM = p.ephSigmaRadialM;
    }
    if (p.assumedSigmaAlongM < 0.0)
    {
        p.assumedSigmaAlongM = p.ephSigmaAlongM;
    }
    if (p.assumedSigmaCrossM < 0.0)
    {
        p.assumedSigmaCrossM = p.ephSigmaCrossM;
    }
    const std::set<std::string> policies{"a3", "d2", "t1", "elev", "tte-eager", "mst", "tte-cho"};
    NS_ABORT_MSG_IF(policies.count(p.policy) == 0, "unknown --policy " << p.policy);

    RngSeedManager::SetSeed(1);
    RngSeedManager::SetRun(p.rngRun);
    const auto wall0 = std::chrono::steady_clock::now();

    std::string mk = "mkdir -p " + p.outputDir;
    if (system(mk.c_str()) != 0)
    {
        std::cerr << "cannot create " << p.outputDir << "\n";
    }

    // ---- constellation (true orbits) --------------------------------------
    Ptr<UniformRandomVariable> epochDraw = CreateObject<UniformRandomVariable>();
    epochDraw->SetStream(1000);
    WalkerConfig wc;
    wc.total_sats = p.totalSats;
    wc.num_planes = p.numPlanes;
    wc.phasing_f = p.phasing;
    wc.altitude_km = p.altitudeKm;
    wc.inclination_deg = p.inclinationDeg;
    // A different slice of the same shell per seed.
    wc.epoch_unix_s = 1735689600.0 - epochDraw->GetValue(0.0, 86400.0);
    const bool tleMode = !p.tleTruthFile.empty();
    std::vector<KeplerianElements> elements;
    std::vector<std::pair<TleRecord, TleRecord>> tlePairs; // (truth, network)
    double tleStartUnix = 0.0;
    if (tleMode)
    {
        // R2 (M10): a real constellation. Truth: the supplemental element
        // sets CelesTrak fits to the operator's ephemerides; network copy: the
        // public GP sets. Both are propagated with SGP4 from one common start.
        NS_ABORT_MSG_IF(p.tleNetFile.empty(), "--tleTruthFile needs --tleNetFile");
        const auto tru = ReadTleFile(p.tleTruthFile);
        const auto net = ReadTleFile(p.tleNetFile);
        std::vector<double> epochs;
        for (const auto& [cat, tr] : tru)
        {
            auto it = net.find(cat);
            KeplerianElements ke;
            KeplerianElements kn;
            if (it == net.end() || !tr.ToKeplerian(ke) || !it->second.ToKeplerian(kn))
            {
                continue;
            }
            const double altKm = (ke.semi_major_axis_m - ntncon::kEarthRadiusM) / 1e3;
            const double incl = ke.inclination_rad * 180.0 / M_PI;
            if (altKm < p.tleAltMinKm || altKm > p.tleAltMaxKm || incl < p.tleInclMinDeg ||
                incl > p.tleInclMaxDeg)
            {
                continue;
            }
            tlePairs.emplace_back(tr, it->second);
            epochs.push_back(ke.epoch_unix_s);
        }
        NS_ABORT_MSG_IF(tlePairs.empty(), "no element sets in the TLE shell filter");
        std::sort(epochs.begin(), epochs.end());
        tleStartUnix = epochs[epochs.size() / 2] +
                       3600.0 * epochDraw->GetValue(-p.tleStartSpreadH, p.tleStartSpreadH);
    }
    else
    {
        elements = WalkerConstellation::BuildDelta(wc);
        NS_ABORT_MSG_IF(elements.empty(), "Walker shell not expressible");
    }
    std::vector<Ptr<Sgp4MobilityModel>> tleNet;
    if (tleMode)
    {
        // drop satellites whose two element sets straddle a maneuver (copy
        // more than 100 km from the truth at the start), as the measurement does
        for (const auto& [tr, nr] : tlePairs)
        {
            Ptr<Sgp4MobilityModel> mt = CreateObject<Sgp4MobilityModel>();
            Ptr<Sgp4MobilityModel> mn = CreateObject<Sgp4MobilityModel>();
            mt->SetTle(tr);
            mn->SetTle(nr);
            mt->SetSimulationEpochUnix(tleStartUnix);
            mn->SetSimulationEpochUnix(tleStartUnix);
            mt->SetCacheGranularity(MilliSeconds(1));
            mn->SetCacheGranularity(MilliSeconds(1));
            const Vector d = mt->GetPosition() - mn->GetPosition();
            if (d.GetLength() > 1e5)
            {
                continue;
            }
            g_sats.push_back(mt);
            tleNet.push_back(mn);
        }
    }
    else
    {
        g_sats.resize(elements.size());
        for (size_t s = 0; s < elements.size(); ++s)
        {
            g_sats[s] = CreateObject<Sgp4MobilityModel>();
            g_sats[s]->SetElements(elements[s]);
            g_sats[s]->SetCacheGranularity(MilliSeconds(1));
        }
    }
    NodeContainer satNodes;
    satNodes.Create(g_sats.size());
    g_satPos.resize(g_sats.size());
    for (size_t s = 0; s < g_sats.size(); ++s)
    {
        satNodes.Get(s)->AggregateObject(g_sats[s]);
    }

    // ---- terminals ----------------------------------------------------------
    NodeContainer ueNodes;
    ueNodes.Create(p.numUes);
    NtnTr38811MobilityHelper ueHelper(static_cast<uint64_t>(p.rngRun));
    MobilityProfile prof = NtnMobilityScenarios::MixedContinental();
    if (p.ueProfile == "static")
    {
        prof = MobilityProfile{1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    }
    const auto ueMobs = ueHelper.Install(ueNodes, prof, p.centerLat - p.halfSpanDeg,
                                         p.centerLat + p.halfSpanDeg, p.centerLon - p.halfSpanDeg,
                                         p.centerLon + p.halfSpanDeg);

    // ---- radio ---------------------------------------------------------------
    g_noiseDbm = -174.0 + 10.0 * std::log10(p.bwMhz * 1e6) + p.ueNfDb;
    g_eirpDbm = p.eirpDensityDbwPerMhz + 30.0 + 10.0 * std::log10(p.bwMhz);
    // One LOS and one NLOS path-loss model per environment (R2, M8): line of
    // sight and suburban/rural use the suburban model, urban and dense urban
    // their own.
    auto makePl = [](int env) -> Ptr<ThreeGppPropagationLossModel> {
        if (env == kEnvUrb)
        {
            return CreateObject<ThreeGppNTNUrbanPropagationLossModel>();
        }
        if (env == kEnvDense)
        {
            return CreateObject<ThreeGppNTNDenseUrbanPropagationLossModel>();
        }
        return CreateObject<ThreeGppNTNSuburbanPropagationLossModel>();
    };
    for (int env = 0; env < 4; ++env)
    {
        g_plEnv[env] = makePl(env);
        g_plEnv[env]->SetAttribute("Frequency", DoubleValue(p.fcGhz * 1e9));
        g_plEnv[env]->SetAttribute("ShadowingEnabled", BooleanValue(false));
        g_plEnv[env]->SetChannelConditionModel(CreateObject<AlwaysLosChannelConditionModel>());
        g_plNlosEnv[env] = makePl(env);
        g_plNlosEnv[env]->SetAttribute("Frequency", DoubleValue(p.fcGhz * 1e9));
        g_plNlosEnv[env]->SetAttribute("ShadowingEnabled", BooleanValue(false));
        g_plNlosEnv[env]->SetChannelConditionModel(CreateObject<NeverLosChannelConditionModel>());
    }
    const int baseEnv = (p.channel == "urban")        ? kEnvUrb
                        : (p.channel == "denseurban") ? kEnvDense
                        : (p.channel == "suburban")   ? kEnvSub
                                                      : kEnvLos;
    g_norm = CreateObject<NormalRandomVariable>();
    g_norm->SetStream(2000);
    g_uni = CreateObject<UniformRandomVariable>();
    g_uni->SetStream(2500);
    g_normOut = CreateObject<NormalRandomVariable>();
    g_normOut->SetStream(2600);
    NS_ABORT_MSG_IF(p.channel != "los" && p.channel != "suburban" && p.channel != "urban" &&
                        p.channel != "denseurban",
                    "unknown --channel");

    // ---- the network's predictor: same satellites, wrong ephemeris ----------
    g_pred = CreateObject<NtnOrbitPredictor>();
    g_pred->Initialize(NodeContainer(), nullptr);
    g_pred->SetGeometricBeam(30.0, 4.4127);
    g_pred->SetSteeredBeam(true, p.scanLossExp);
    g_pred->SetSteeredBeamElevationIndexed(true, p.altitudeKm * 1e3);
    g_pred->SetReferenceEarthRadius(kRe); // R2 (m3): the radius of the orbits and the text
    Ptr<NormalRandomVariable> ephDraw = CreateObject<NormalRandomVariable>();
    ephDraw->SetStream(3000);
    // N4: measured error samples, drawn on their own stream so the Gaussian
    // stream (and every other stream) is unchanged by the choice of model
    std::vector<Vector> ephRows;
    std::vector<double> ephRates; // R2: measured along-track growth (m/s) of each row
    if (!p.ephErrorFile.empty())
    {
        std::ifstream ef(p.ephErrorFile);
        NS_ABORT_MSG_IF(!ef.good(), "cannot read --ephErrorFile " << p.ephErrorFile);
        std::string line;
        std::getline(ef, line); // header
        // dt = 30 min rows of the same element-set pair give the along-track
        // growth rate of the copy (R2, M10)
        std::map<std::string, double> along30;
        std::vector<std::pair<std::string, Vector>> rows0;
        while (std::getline(ef, line))
        {
            std::stringstream ss(line);
            std::string f[8];
            for (int k = 0; k < 8 && std::getline(ss, f[k], ','); ++k)
            {
            }
            if (f[7].empty())
            {
                continue;
            }
            const double age = std::stod(f[3]);
            const double dt = std::stod(f[4]);
            const double er = std::stod(f[5]);
            const double ea = std::stod(f[6]);
            const double ec = std::stod(f[7]);
            if (dt == 30.0 && std::abs(ea) <= 1e5)
            {
                along30[f[0]] = ea;
            }
            if (dt == 0.0 && age >= p.ephAgeMinH && age < p.ephAgeMaxH && std::abs(er) <= 1e5 &&
                std::abs(ea) <= 1e5 && std::abs(ec) <= 1e5)
            {
                ephRows.emplace_back(er, ea, ec);
                rows0.emplace_back(f[0], Vector(er, ea, ec));
            }
        }
        for (const auto& [cat, v] : rows0)
        {
            auto it = along30.find(cat);
            ephRates.push_back(it == along30.end() ? 0.0 : (it->second - v.y) / 1800.0);
        }
        NS_ABORT_MSG_IF(ephRows.empty(), "no eligible rows in --ephErrorFile for the age window");
    }
    Ptr<UniformRandomVariable> ephPick = CreateObject<UniformRandomVariable>();
    ephPick->SetStream(3100);
    std::ofstream ephLog;
    const bool anyGauss = p.ephSigmaRadialM > 0.0 || p.ephSigmaAlongM > 0.0 || p.ephSigmaCrossM > 0.0;
    g_ephActive = tleMode || anyGauss || !ephRows.empty();
    if (g_ephActive)
    {
        if (!ephLog.is_open())
        {
            ephLog.open(p.outputDir + "/eph_offsets.csv");
        }
        ephLog << "sat,err_r_m,err_a_m,err_c_m,rate_a_mps,d0_r_m,d0_a_m,d0_c_m,dT_r_m,dT_a_m,dT_c_m\n";
    }
    double ephCheckMax = 0.0;
    g_ephDrawn.assign(g_sats.size(), Vector(0, 0, 0));
    g_net.assign(g_sats.size(), nullptr);
    for (size_t s = 0; s < g_sats.size(); ++s)
    {
        double er = p.ephSigmaRadialM * ephDraw->GetValue();
        double ea = p.ephSigmaAlongM * ephDraw->GetValue();
        double ec = p.ephSigmaCrossM * ephDraw->GetValue();
        double rate = 0.0;
        if (!ephRows.empty())
        {
            const uint32_t k = ephPick->GetInteger(0, static_cast<uint32_t>(ephRows.size() - 1));
            er = ephRows[k].x;
            ea = ephRows[k].y;
            ec = ephRows[k].z;
            rate = ephRates[k];
        }
        Ptr<Sgp4MobilityModel> netCopy = g_sats[s];
        if (tleMode)
        {
            netCopy = tleNet[s];
        }
        else if (er != 0.0 || ea != 0.0 || ec != 0.0)
        {
            if (p.ephModel == "offset")
            {
                g_pred->SetEphemerisErrorRtn(static_cast<uint32_t>(s), er, ea, ec);
            }
            else
            {
                netCopy = CreateObject<Sgp4MobilityModel>();
                netCopy->SetElements(PerturbedCopy(elements[s], er, ea, ec, rate, !ephRows.empty()));
                netCopy->SetCacheGranularity(MilliSeconds(1));
            }
        }
        g_net[s] = netCopy;
        g_pred->SetKinematicsSource(static_cast<uint32_t>(s), netCopy);
        g_ephDrawn[s] = Vector(er, ea, ec);
        if (g_ephActive)
        {
            // check: copy minus truth in the true RTN frame, now and at the end
            Vector pp;
            Vector pv;
            g_pred->GetPredictedSatEcef(static_cast<uint32_t>(s), pp, pv);
            const Vector d0 = RtnDiff(g_sats[s]->GetPosition(), g_sats[s]->GetVelocity(), pp);
            Vector tpT;
            Vector tvT;
            Vector npT;
            Vector nvT;
            g_sats[s]->GetEcefStateAt(Seconds(p.simTime), tpT, tvT);
            netCopy->GetEcefStateAt(Seconds(p.simTime), npT, nvT);
            Vector dT = RtnDiff(tpT, tvT, npT);
            if (p.ephModel == "offset" && !tleMode)
            {
                dT = d0; // re-anchored: the offset does not change
            }
            if (tleMode)
            {
                g_ephDrawn[s] = d0;
            }
            ephCheckMax = std::max(ephCheckMax, (d0 - g_ephDrawn[s]).GetLength());
            ephLog << s << "," << std::setprecision(10) << g_ephDrawn[s].x << "," << g_ephDrawn[s].y
                   << "," << g_ephDrawn[s].z << "," << rate << "," << d0.x << "," << d0.y << ","
                   << d0.z << "," << dT.x << "," << dT.y << "," << dT.z << "\n";
        }
    }
    if (ephLog.is_open())
    {
        ephLog.close();
        std::cout << "eph-check: model=" << (tleMode ? "tle" : p.ephModel) << " " << ephRows.size()
                  << " file rows, max |copy - true - drawn RTN| at t=0 = " << ephCheckMax << " m\n";
    }
    for (int env = 0; env < 4; ++env)
    {
        g_tteEnv[env] = CreateObject<NtnTteEstimator>();
        g_tteEnv[env]->SetOrbitPredictor(g_pred);
        g_tteEnv[env]->SetAttribute("PredictionStep", TimeValue(Seconds(p.predStepS)));
        g_tteEnv[env]->SetAttribute("MaxPredictionWindow", TimeValue(Seconds(p.horizonS)));
        g_tteEnv[env]->SetAttribute("BinarySearchTolerance", TimeValue(MilliSeconds(50)));
        if (env != kEnvLos)
        {
            // N1: the network knows the terminal's environment class, not the
            // drawn state, so the usable time weighs the predicted pass by the
            // same TR 38.811 Table 6.6.1-1 probabilities the channel is drawn from.
            g_tteEnv[env]->SetLosProbabilityTable(*g_env[env].plos);
        }
    }
    g_tte = g_tteEnv[baseEnv];
    g_coverGainTh = g_pred->GainThresholdForMinElevationDb(p.minElevDeg, p.altitudeKm * 1e3);
    // A quality floor is planned for as an EXIT: with a steered beam both the
    // scan loss and the slant loss are monotone in elevation, so "this cell no
    // longer meets the floor" is "this cell has descended below e_Q", and the
    // time-to-exit is measured to that contour rather than to the scan limit.
    const double exitElev = std::max(p.minElevDeg, p.tteExitElevDeg);
    g_tteGainTh = g_pred->GainThresholdForMinElevationDb(exitElev, p.altitudeKm * 1e3);

    // ---- one CHO algorithm per terminal --------------------------------------
    g_ues.resize(p.numUes);
    for (uint32_t i = 0; i < p.numUes; ++i)
    {
        UeCtx& u = g_ues[i];
        u.node = ueNodes.Get(i);
        u.mob = ueMobs[i];
        u.ueClass = u.mob->GetClassName();
        u.env = baseEnv;
        if (p.classEnv && baseEnv != kEnvLos)
        {
            if (u.ueClass == "aviation" || u.ueClass == "maritime")
            {
                u.env = kEnvLos; // above or away from street clutter
            }
            else if (u.ueClass == "hst")
            {
                u.env = kEnvSub; // rural row of TR 38.811 Table 6.6.1-1
            }
        }
        u.tte = g_tteEnv[u.env];
        u.cho = CreateObject<NtnChoAlgorithm>();
        u.cho->SetOrbitPredictor(g_pred);
        u.cho->SetTteEstimator(u.tte);
        NtnChoAlgorithm::ChoConfig c = u.cho->GetConfig();
        c.maxCandidates = static_cast<uint8_t>(p.maxCandidates + 2);
        c.qualityThreshold_dB = p.qualityThDb;
        c.gainThreshold_dB = g_tteGainTh;
        c.tteMinimum = Seconds(p.tteMinS);
        c.orbitAltitudeKm = p.altitudeKm;
        c.conditionMonitorPeriod = Seconds(p.simTime + 10.0); // driven from Tick
        c.t304Timer = Seconds(p.simTime + 10.0); // outcome is decided by CompleteHandover
        c.tteCacheRefresh = Seconds(p.tteCacheS);
        c.rankByUsableTime = p.rankByUsable;
        c.escapeNearBestDb = p.escapeNearBestDb;
        c.escapeHoldoff = Seconds(p.escapeHoldoffS);
        c.escapeByQuality = p.escapeByQuality;
        if (p.policy == "a3")
        {
            c.triggerType = NtnChoAlgorithm::TRIGGER_EVENT_A3;
            c.a3Offset_dB = p.a3OffsetDb;
            c.a3TimeToTrigger = MilliSeconds(p.a3TttMs);
        }
        else if (p.policy == "d2")
        {
            c.triggerType = NtnChoAlgorithm::TRIGGER_DISTANCE_D2;
            c.d2Thresh1_m = ChordToNadirAtElevation(p.d2ServElevDeg, p.altitudeKm * 1e3);
            c.d2Thresh2_m = ChordToNadirAtElevation(p.d2CandElevDeg, p.altitudeKm * 1e3);
            c.d2HysteresisLocation_m = p.d2HysM;
        }
        else if (p.policy == "t1")
        {
            c.triggerType = NtnChoAlgorithm::TRIGGER_TIME_T1;
            c.t1WindowDuration = Seconds(p.t1DurationS);
            c.t1ThresholdEpoch = Seconds(p.simTime + 1.0); // armed on attach
            c.ephSigmaRadialM = p.assumedSigmaRadialM;
            c.ephSigmaAlongM = p.assumedSigmaAlongM;
            c.ephSigmaCrossM = p.assumedSigmaCrossM;
        }
        else if (p.policy == "elev")
        {
            c.triggerType = NtnChoAlgorithm::TRIGGER_ELEVATION;
            c.elevationMinDeg = p.elevFloorDeg;
            c.elevationHystDeg = p.elevHystDeg;
        }
        else
        {
            c.triggerType = NtnChoAlgorithm::TRIGGER_TTE_AWARE;
            c.tteRequireD1 = false;
            c.requireBetterThanServing = true;
            c.tteExitTriggered = (p.policy != "tte-eager");
            c.tteExitMargin = Seconds(p.execMarginS);
            c.tteMarginZ = p.marginZ;
            c.tteServingFloor_dB = p.tteFloorDb;
            c.tteServingFloorTtt = MilliSeconds(p.tteFloorTttMs);
            if (p.policy == "tte-cho")
            {
                c.ephSigmaRadialM = p.assumedSigmaRadialM;
                c.ephSigmaAlongM = p.assumedSigmaAlongM;
                c.ephSigmaCrossM = p.assumedSigmaCrossM;
            }
        }
        u.cho->Configure(c);
        // Decisions are actuated by this scenario's own execution model.
        u.cho->SetHandoverExecutionCallback(MakeCallback(+[](uint16_t, uint16_t) {}));
    }

    std::cout << "ntn-cho-tte-campaign: policy=" << p.policy << " mode=" << p.mode
              << " sats=" << g_sats.size() << " ues=" << p.numUes << " T=" << p.simTime
              << " s run=" << p.rngRun << " EIRP=" << g_eirpDbm << " dBm N=" << g_noiseDbm
              << " dBm coverGainTh=" << g_coverGainTh << " dB tteGainTh=" << g_tteGainTh << " dB eph(R,A,C)=(" << p.ephSigmaRadialM
              << "," << p.ephSigmaAlongM << "," << p.ephSigmaCrossM << ") m\n";

    if (p.mode == "tte-accuracy")
    {
        g_acc.open(p.outputDir + "/tte_accuracy.csv");
        g_acc << "ue,sat,elev_deg,peak_elev_deg,pred_tte_s,true_tte_s,sigma_pred_s,err_r_m,"
                 "err_a_m,err_c_m,compute_us\n";
        Simulator::Schedule(Seconds(0.0), &RunAccuracy);
        Simulator::Stop(Seconds(0.01));
        Simulator::Run();
        Simulator::Destroy();
        g_acc.close();
        return 0;
    }

    if (p.traceUe >= 0)
    {
        g_trace.open(p.outputDir + "/trace.csv");
        g_trace << "t_s,sat,elev_deg,l3_db,serving,los\n";
    }
    if (p.dumpLosDraws)
    {
        g_losDump.open(p.outputDir + "/los_draws.csv");
        g_losDump << "elev_bin,los\n";
    }
    if (p.dumpLinksUe >= 0)
    {
        g_dump.open(p.outputDir + "/link_dump.csv");
        g_dump << "t_s,sat,los,sf_db,l1_db\n";
    }
    g_events.open(p.outputDir + "/events.csv");
    g_events << "t_s,ue,from_sat,to_sat,kind,success,pp1,pp5,pred_serv_tte_s,margin_s,"
                "true_serv_exit_s,target_tte_s,target_elev_deg,interruption_ms,target_l3_db,target_rank_all,"
                "trigger\n";
    Simulator::Schedule(Seconds(0.0), &Tick);
    Simulator::Stop(Seconds(p.simTime + 5.0));
    Simulator::Run();

    // ---- results --------------------------------------------------------------
    std::ofstream us(p.outputDir + "/ue_summary.csv");
    us << "ue,class,ho_attempts,ho_success,hof,rlf,pp1,pp5,outage_s,mean_sinr_db,mean_se,"
          "mean_tos_s,lb_handovers,env,rrc,rrc_cand,rrc_win,ho_planned,ho_escape\n";
    double sumAtt = 0;
    double sumOk = 0;
    double sumHof = 0;
    double sumRlf = 0;
    double sumPp1 = 0;
    double sumPp5 = 0;
    double sumOut = 0;
    double sumSinr = 0;
    double sumSe = 0;
    double nSamp = 0;
    double sumLb = 0;
    uint32_t nLb = 0;
    double sumEvals = 0;
    double sumT1Rearms = 0;
    double sumDecisions = 0;
    double sumRrc = 0;
    double sumRrcCand = 0;
    double sumRrcWin = 0;
    double sumPlanned = 0;
    double sumEscape = 0;
    std::vector<double> allTos;
    for (size_t i = 0; i < g_ues.size(); ++i)
    {
        UeCtx& u = g_ues[i];
        if (p.ueProfile == "static")
        {
            u.lbHandovers = LowerBoundHandovers(u.mob->GetPosition());
            sumLb += u.lbHandovers;
            ++nLb;
        }
        double mt = 0.0;
        for (double x : u.tos)
        {
            mt += x;
            allTos.push_back(x);
        }
        mt = u.tos.empty() ? 0.0 : mt / u.tos.size();
        us << i << "," << u.ueClass << "," << u.hoAttempts << "," << u.hoSuccess << "," << u.hof
           << "," << u.rlf << "," << u.pp1 << "," << u.pp5 << "," << std::fixed
           << std::setprecision(3) << u.outageS << ","
           << (u.samples ? u.sinrSum / u.samples : 0.0) << ","
           << (u.samples ? u.seSum / u.samples : 0.0) << "," << mt << ","
           << (p.ueProfile == "static" ? std::to_string(u.lbHandovers) : std::string("")) << ","
           << u.env << "," << u.rrc << "," << u.rrcCand << "," << u.rrcWin << "," << u.hoPlanned << ","
           << u.hoEscape << "\n";
        sumEvals += static_cast<double>(u.cho->GetTteEvaluationCount());
        sumDecisions += static_cast<double>(u.decisions);
        sumT1Rearms += u.t1Rearms;
        sumRrc += u.rrc;
        sumRrcCand += u.rrcCand;
        sumRrcWin += u.rrcWin;
        sumPlanned += u.hoPlanned;
        sumEscape += u.hoEscape;
        sumAtt += u.hoAttempts;
        sumOk += u.hoSuccess;
        sumHof += u.hof;
        sumRlf += u.rlf;
        sumPp1 += u.pp1;
        sumPp5 += u.pp5;
        sumOut += u.outageS;
        sumSinr += u.sinrSum;
        sumSe += u.seSum;
        nSamp += u.samples;
    }
    us.close();
    std::sort(allTos.begin(), allTos.end());
    const double medTos = allTos.empty() ? 0.0 : allTos[allTos.size() / 2];
    double meanTos = 0.0;
    for (double x : allTos)
    {
        meanTos += x;
    }
    meanTos = allTos.empty() ? 0.0 : meanTos / allTos.size();
    const double ueMin = p.numUes * p.simTime / 60.0;
    const double wall =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - wall0).count();
    std::ofstream sm(p.outputDir + "/summary.csv");
    sm << "policy,run,num_ues,sim_time_s,ho_attempts,ho_success,hof,rlf,pp1,pp5,"
          "ho_per_ue_min,pp5_rate,rlf_per_ue_min,outage_frac,mean_sinr_db,mean_se,mean_tos_s,"
          "median_tos_s,lb_handovers_per_ue,eph_r_m,eph_a_m,eph_c_m,assumed_a_m,a3_offset_db,"
          "a3_ttt_ms,quality_db,min_elev_deg,channel,tte_floor_db,tte_floor_ttt_ms,tte_exit_elev_deg,d2_serv_elev_deg,d2_cand_elev_deg,elev_floor_deg,"
          "rank_by_usable,prepare_by_tte,escape_holdoff_s,meas_fallback,tte_rank,decision_period_s,"
          "tte_evals_per_decision,t1_rearms,wall_s,rrc_per_ue_min,rrc_cand_per_ue_min,"
          "rrc_win_per_ue_min,escape_share,class_env,los_model,los_corr_m,los_corr_deg,eph_model,"
          "escape_by_quality,prep_above_contour,reest_base_ms,n_sats\n";
    sm << p.policy << "," << p.rngRun << "," << p.numUes << "," << p.simTime << "," << sumAtt
       << "," << sumOk << "," << sumHof << "," << sumRlf << "," << sumPp1 << "," << sumPp5 << ","
       << std::fixed << std::setprecision(4) << sumOk / ueMin << ","
       << (sumOk > 0 ? sumPp5 / sumOk : 0.0) << "," << sumRlf / ueMin << ","
       << sumOut / (p.numUes * p.simTime) << "," << (nSamp ? sumSinr / nSamp : 0.0) << ","
       << (nSamp ? sumSe / nSamp : 0.0) << "," << meanTos << "," << medTos << ","
       << (nLb ? sumLb / nLb : -1.0) << "," << p.ephSigmaRadialM << "," << p.ephSigmaAlongM
       << "," << p.ephSigmaCrossM << "," << p.assumedSigmaAlongM << "," << p.a3OffsetDb << ","
       << p.a3TttMs << "," << p.qualityThDb << "," << p.minElevDeg << "," << p.channel << ","
       << p.tteFloorDb << "," << p.tteFloorTttMs << "," << p.tteExitElevDeg
       << "," << p.d2ServElevDeg << "," << p.d2CandElevDeg << "," << p.elevFloorDeg << ","
       << (p.rankByUsable ? 1 : 0) << "," << (p.prepareByTte ? 1 : 0) << "," << p.escapeHoldoffS
       << "," << (p.measFallback ? 1 : 0) << "," << (p.tteRank ? 1 : 0) << "," << p.decisionPeriodS
       << "," << (sumDecisions > 0 ? sumEvals / sumDecisions : 0.0) << "," << sumT1Rearms << ","
       << wall << "," << sumRrc / ueMin << "," << sumRrcCand / ueMin << "," << sumRrcWin / ueMin
       << "," << (sumPlanned + sumEscape > 0 ? sumEscape / (sumPlanned + sumEscape) : 0.0) << ","
       << (p.classEnv ? 1 : 0) << "," << p.losModel << "," << p.losCorrM << "," << p.losCorrDeg
       << "," << (tleMode ? std::string("tle") : p.ephModel) << "," << (p.escapeByQuality ? 1 : 0)
       << "," << (p.prepAboveContour ? 1 : 0) << "," << p.reestBaseMs << "," << g_sats.size()
       << "\n";
    sm.close();
    g_events.close();
    std::cout << "  HO/UE/min=" << sumOk / ueMin << " success=" << sumOk << "/" << sumAtt
              << " HOF=" << sumHof << " RLF=" << sumRlf << " PP5=" << sumPp5
              << " outage=" << 100.0 * sumOut / (p.numUes * p.simTime) << "% meanSINR="
              << (nSamp ? sumSinr / nSamp : 0.0) << " dB meanToS=" << meanTos
              << " s wall=" << wall << " s\n";
    Simulator::Destroy();
    return 0;
}
