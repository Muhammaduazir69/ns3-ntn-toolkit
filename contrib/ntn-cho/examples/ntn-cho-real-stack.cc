/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026  Muhammad Uzair
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * ntn-cho-real-stack — real-stack flagship for ntn-cho, on the REAL NTN
 * mobility architecture:
 *
 *   - Satellites: SGP4 (Vallado) Walker-Delta orbits from synthesized TLEs
 *     (ntn-constellation Sgp4MobilityModel; full Vallado SGP4 available via
 *     SetUseVallado). The serving satellite starts at zenith over the UE
 *     field and recedes; the in-plane neighbour genuinely approaches. The
 *     handover emerges from REAL orbital dynamics — no teleports.
 *   - UEs: 3GPP TR 38.811 §6.1.1.1 class mobility (NtnTr38811MobilityModel,
 *     ECEF) — pedestrians/vehicles/IoT moving under the pass.
 *   - Radio: real mmwave NR NTN cell (NtnRealStackHelper: SpectrumPhy + MAC +
 *     HARQ + RLC/PDCP + RRC + EPC); serving SINR/TBLER MEASURED off the PHY.
 *
 * Novel 6G handover mechanisms (selectable, implemented in NtnChoAlgorithm):
 *   --trigger=tte-aware  TTE-aware CHO (the toolkit's Rel-17 baseline novelty)
 *   --trigger=ltm        Rel-19 conditional LTM: L1-filtered measurements +
 *                        CHO reliability; MAC-CE fast switch (~25 ms)
 *   --trigger=pcho       trajectory-prediction CHO: forecast serving outage +
 *                        max predicted time-of-stay candidate
 *   --rachLess           RCHO: ephemeris/GNSS TA pre-compensation skips RACH
 *
 * Decision inputs (Wave 2 M1, P0-2): every quantity the CHO decides on is
 * MEASURED. The serving SINR comes from the UE PHY; the candidate is the UE's
 * own TS 38.331 5.5.5 measurement report (periodic, 480 ms, L3-filtered at the
 * UE per 5.5.3.2), on a cell-level SS-RSRP (SSB on an Earth-fixed cell beam,
 * TS 38.215 5.1.1). The candidate quality handed to the algorithm is the
 * serving SINR plus the reported RSRP difference (candidate minus serving):
 * same receiver, same noise and interference, so the difference carries the
 * measured link imbalance. No Friis extrapolation and no elevation-to-gain map
 * enters the decision. `--candSource=friis` restores the earlier ephemeris
 * extrapolation for comparison only.
 *
 * Handover KPIs (P0-3) are MEASURED by NtnHoKpiRecorder from the UE RRC traces
 * (HandoverStart, HandoverEndOk/Error, RadioLinkFailure) and the UE's packet
 * arrivals: C-plane and U-plane interruption, ping-pong, time of stay, RLF.
 * The algorithm's priced interruption is written next to them as "modeled".
 *
 * Usage:
 *   ./ns3 run "ntn-cho-real-stack --duration=60 --trigger=pcho --rachLess=1"
 */

#include "ns3/core-module.h"
#include "ns3/mmwave-enb-net-device.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/ntn-cho-algorithm.h"
#include "ns3/ntn-cho-helper.h"
#include "ns3/ntn-orbit-predictor.h"
#include "ns3/ntn-ho-kpi-recorder.h"
#include "ntn-cho-measured-inputs.h"

#include "ns3/ntn-real-stack-helper.h"
#include "ns3/ntn-scene-recorder.h"
#include "ns3/ntn-tr38811-mobility-model.h"

#include "ns3/sgp4-mobility-model.h"
#include "ns3/walker-constellation.h"

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <limits>
#include <iostream>
#include <vector>

using namespace ns3;
using ns3::ntncon::Sgp4MobilityModel;
using ns3::ntncon::WalkerConfig;
using ns3::ntncon::WalkerConstellation;

NS_LOG_COMPONENT_DEFINE("NtnChoRealStack");

namespace
{
Ptr<NtnChoAlgorithm> g_cho;
NtnRealStackHelper* g_rs = nullptr;
Ptr<MobilityModel> g_servMob;
Ptr<MobilityModel> g_candMob;
Ptr<NtnTr38811MobilityModel> g_ueMob;
uint16_t g_servingCellId = 0;
uint16_t g_candCellId = 0;
uint16_t g_serving = 0;
uint32_t g_handovers = 0;
uint32_t g_evals = 0;
double g_simTime = 60.0;
std::string g_candSource = "rrc"; // rrc (measured reports) | friis (legacy prediction)
std::string g_trigger;
Ptr<NtnHoKpiRecorder> g_hoRec;
FILE* g_trace = nullptr;
uint32_t g_ticksWithReport = 0;
uint32_t g_ticksAfter2s = 0;

// Optional 3D scene trace (NetSimulyzer + Cesium CZML). Off unless --netSim or
// --czml is given; when on, the recorder streams the SGP4 sat + TR 38.811
// UE positions and the MEASURED serving SINR, and logs each handover.
Ptr<ntnobs::NtnSceneRecorder> g_scene;
uint32_t g_sinrSeries = 0;
uint32_t g_servSatNodeId = 0;
uint32_t g_candSatNodeId = 0;
uint32_t g_ueNodeId = 0;

void
ChoTick()
{
    if (Simulator::Now().GetSeconds() >= g_simTime)
    {
        return;
    }
    ++g_evals;
    const Vector u = g_ueMob->GetPosition();        // TR 38.811 UE, ECEF
    const Vector sPos = g_servMob->GetPosition();   // SGP4 serving sat, ECEF
    const Vector cPos = g_candMob->GetPosition();   // SGP4 candidate sat, ECEF
    const double servElev = ntngeo::ElevationDeg(u, sPos);
    const double candElev = ntngeo::ElevationDeg(u, cPos);
    const double servSlant = ntngeo::SlantRangeM(u, sPos);
    const double candSlant = ntngeo::SlantRangeM(u, cPos);

    // Serving SINR is MEASURED at UE 0's PHY: linear mean of its decoded TBs
    // over the last tick (nr), or the helper's last TB SINR (mmwave).
    const double servSinr = g_rs->GetRadioBackend() == NtnRealStackHelper::RadioBackend::Nr
                                ? g_rs->TakeUeSinrWindowMeanDb(0)
                                : g_rs->GetUeRecentSinrDb(0);
    if (std::isnan(servSinr))
    {
        Simulator::Schedule(Seconds(1.0), &ChoTick);
        return;
    }
    // Feed the measured serving SINR to the 3D scene KPI series (if enabled).
    if (g_scene)
    {
        g_scene->RecordKpi(g_sinrSeries, servSinr);
    }
    // Candidate SINR. The candidate satellite is now a REAL gNB (see Build), so
    // prefer its MEASURED cell SINR from the PHY trace. Fall back to the
    // ephemeris-predicted Friis correction off the measured serving baseline
    // only while the neighbour has no samples yet (it carries no traffic until a
    // UE is handed to it) — that fallback is what CHO candidate prediction
    // legitimately is (3GPP NTN CHO predicts the target from ephemeris), but it
    // must not masquerade as a measurement when a real one exists (gap C6).
    //
    // CHO-25: the measured SINR belongs to the cell the radio ACTUALLY serves
    // UE 0 from (TS 38.331 section 5.5.3: Mp is the current PCell). It used to
    // be filed under the original serving cell for the whole run, so after the
    // handover the new cell's measurement was reported as the old cell's. The
    // other cell's run-mean is only used while UE 0 is still on the original
    // cell: after a handover the old cell's mean is dominated by samples taken
    // before the UE left it, which is history, not a measurement of now.
    const uint16_t radioCell = g_rs->GetUeServingCellId(0);
    const bool onCand = (radioCell == g_candCellId);
    const uint16_t attCell = onCand ? g_candCellId : g_servingCellId;
    const uint16_t othCell = onCand ? g_servingCellId : g_candCellId;
    const double attSlant = onCand ? candSlant : servSlant;
    const double othSlant = onCand ? servSlant : candSlant;
    const double attElev = onCand ? candElev : servElev;
    const double othElev = onCand ? servElev : candElev;
    if (radioCell != 0 && radioCell != g_cho->GetServingCellId() &&
        g_cho->GetState() != NtnChoAlgorithm::CHO_EXECUTING)
    {
        g_cho->SetServingCell(radioCell); // radio moved without a CHO decision
    }
    // P0-2: the candidate comes from the UE's own measurement reports
    // (TS 38.331 5.5.5, periodic 480 ms; already L3-filtered at the UE per
    // 5.5.3.2, so it is not filtered again here). Mp and Mn are cell-level
    // SS-RSRP of the serving and the neighbour cell (TS 38.215 5.1.1).
    const ntnchoex::ReportedCandidate rep =
        ntnchoex::ReadReportedCandidate(*g_rs, 0, othCell, servSinr);
    const double servRsrp = rep.servRsrpDbm;
    const double candRsrp = rep.candRsrpDbm;
    const Time age = Seconds(rep.ageS);
    // Ephemeris extrapolation, kept only as a printed comparison column (and as
    // the decision input under --candSource=friis).
    const double candFriis = servSinr + 20.0 * std::log10(attSlant / std::max(1.0, othSlant));
    double candSinr = std::numeric_limits<double>::quiet_NaN();
    std::string candSrc = "none";
    if (g_candSource == "friis")
    {
        candSinr = candFriis;
        candSrc = "friis-pred";
    }
    else if (rep.fresh)
    {
        candSinr = rep.candSinrDb;
        candSrc = "rrc-report";
    }
    if (Simulator::Now() >= Seconds(2.0))
    {
        ++g_ticksAfter2s;
        g_ticksWithReport += (candSrc == "rrc-report") ? 1 : 0;
    }
    // No beam-gain input: the decision uses measured quantities only (the old
    // (elev-45)/5 mapping was invented). NaN marks the field as not provided.
    const double noGain = std::numeric_limits<double>::quiet_NaN();
    const bool candIsMeasured = (candSrc == "rrc-report");

    // CHO-21, same defect as ntn-cho-handover-traffic. D1, D2, T1 and the TTE
    // estimator evaluate against m_uePosition, and this scenario never set it,
    // so every geometric trigger it offers was measuring from a
    // default-constructed coordinate while the radio flew the real orbit.
    g_cho->UpdateUeKinematics(GeoCoordinate(u), g_ueMob->GetVelocity());

    g_cho->UpdateMeasurement(attCell, servSinr, noGain);
    if (!std::isnan(candSinr))
    {
        g_cho->UpdateMeasurement(othCell, candSinr, noGain);
    }
    // Live ephemeris slant ranges -> RACH-less TA pre-compensation (RCHO).
    g_cho->UpdateCandidateSlantRange(attCell, attSlant);
    g_cho->UpdateCandidateSlantRange(othCell, othSlant);
    g_cho->EvaluateConditions();

    uint16_t chosen = g_cho->SelectBestCandidate();
    if (g_trace)
    {
        std::fprintf(g_trace,
                     "%.1f,%u,%u,%.3f,%.2f,%.2f,%.3f,%s,%.3f,%.3f,%.2f,%.2f,%.1f,%.1f,%u,%u\n",
                     Simulator::Now().GetSeconds(), attCell, othCell, servSinr, servRsrp,
                     candRsrp, candSinr, candSrc.c_str(), age.GetSeconds(), candFriis, attElev,
                     othElev, attSlant / 1e3, othSlant / 1e3, chosen,
                     static_cast<unsigned>(g_cho->GetState()));
    }
    // CHO-25: no new decision while a reconfiguration is in flight (T304,
    // TS 38.331 section 5.3.5.8.3), and the reference is the radio's cell.
    const uint16_t curCell = (radioCell != 0) ? radioCell : g_cho->GetServingCellId();
    if (chosen != 0 && chosen != curCell && chosen != g_cho->GetServingCellId() &&
        g_cho->GetState() != NtnChoAlgorithm::CHO_EXECUTING)
    {
        ++g_handovers;
        // ExecuteHandover now calls the registered callback, which issues a REAL
        // X2 reconfiguration-with-sync via NtnRealStackHelper::TriggerHandover
        // (see main()). The outcome arrives asynchronously from the RRC; T304
        // runs until then, so a failed handover is now actually representable.
        g_cho->ExecuteHandover(chosen);
        // P0-3: the measured record of this handover carries the algorithm's
        // priced value as a separate, labeled column.
        g_hoRec->AnnotateNext(0, g_cho->GetMechanismStats().lastInterruptionMs, "cho:" + g_trigger);
        if (g_scene)
        {
            g_scene->OnHandover(g_ueNodeId, g_servSatNodeId, g_candSatNodeId);
        }
        const auto st = g_cho->GetMechanismStats();
        std::printf("  %6.1fs  HANDOVER cell %u -> %u  (servSINR meas=%.1f dB, cand=%s "
                    "%.1f dB, modeled interruption=%.2f ms%s)\n",
                    Simulator::Now().GetSeconds(), curCell, chosen, servSinr,
                    candIsMeasured ? "meas-rsrp" : candSrc.c_str(), candSinr,
                    st.lastInterruptionMs,
                    st.rachLessExecutions > 0 ? ", RACH-less" : "");
        g_serving = chosen;
    }
    if (g_evals % 5 == 0)
    {
        std::printf("  %6.1fs  servElev=%5.1f candElev=%5.1f  servSlant=%6.0fkm "
                    "candSlant=%6.0fkm  measSINR=%6.2f dB\n",
                    Simulator::Now().GetSeconds(), servElev, candElev, servSlant / 1e3,
                    candSlant / 1e3, servSinr);
    }
    Simulator::Schedule(Seconds(1.0), &ChoTick);
}
} // namespace

int
main(int argc, char* argv[])
{
    double duration = 60.0;
    uint32_t numUes = 2;
    double altitudeKm = 550.0;
    double satEirpDbm = -1.0; // sentinel: backend-appropriate default chosen below
    double freqGhz = 2.185; // TS 38.101-5 n256 downlink block (2170-2200 MHz). Was 2.0,
                              // which is n256 UPLINK and never conformant as a DL carrier.
    double tteMinSec = 3.0;
    std::string radio = "nr"; // radio spine: "nr" (5G-LENA FR1) | "mmwave" (FR2)
    std::string trigger = "pcho";
    bool rachLess = true;
    uint32_t satsPerPlane = 80; // 4.5 deg in-plane spacing -> ~500 km along-track
    std::string outputDir = "ntn-cho-real-stack-output";
    std::string candSource = "rrc";
    bool cellBeam = true;
    bool set1 = true;
    bool fading = true;
    std::string channel = "legacy";
    uint32_t pchoMinSamples = 16;
    double gwDistanceKm = 500.0;
    double ppWindowS = 1.0;
    std::string netSimOut; // NetSimulyzer JSON trace (empty = off)
    std::string czmlOut;   // Cesium CZML trace (empty = off)

    CommandLine cmd(__FILE__);
    cmd.AddValue("duration", "Simulation duration (s)", duration);
    cmd.AddValue("numUes", "Number of TR 38.811 UEs on the serving cell", numUes);
    cmd.AddValue("altitude", "Constellation altitude (km)", altitudeKm);
    cmd.AddValue("satEirpDbm", "-1 = TR 38.821 Set-1 S-band EIRP density (34 dBW/MHz); otherwise conducted power at the array input (dBm)", satEirpDbm);
    cmd.AddValue("freqGhz", "Carrier frequency (GHz)", freqGhz);
    cmd.AddValue("radio", "Radio spine: nr (5G-LENA FR1, 30 kHz SCS) | mmwave (FR2)", radio);
    cmd.AddValue("tteMin", "Minimum TTE for CHO admission (s)", tteMinSec);
    cmd.AddValue("trigger",
                 "Handover trigger: a3|d1|t1|ta|elevation|d2|tte-aware|ltm|pcho",
                 trigger);
    cmd.AddValue("rachLess", "RACH-less execution (ephemeris TA pre-comp)", rachLess);
    cmd.AddValue("satsPerPlane", "Walker in-plane satellites (spacing)", satsPerPlane);
    cmd.AddValue("outputDir", "Output directory", outputDir);
    cmd.AddValue("candSource",
                 "CHO candidate input: rrc (UE measurement reports, measured RSRP) | friis "
                 "(legacy ephemeris extrapolation, comparison only)",
                 candSource);
    cmd.AddValue("cellBeam", "SSB on an Earth-fixed cell beam (cell-level SS-RSRP, nr only)", cellBeam);
    cmd.AddValue("channel",
                 "legacy (toolkit excess-loss chain) | ns3ntn (TR 38.811 path loss, condition and "
                 "NTN fast fading, C3)",
                 channel);
    cmd.AddValue("pchoMinSamples",
                 "PCHO: minimum 1 s samples before the linear forecast is trusted (span >= 2x "
                 "the 8 s horizon; reports are quantized to 1 dB, TS 38.133)",
                 pchoMinSamples);
    cmd.AddValue("fading", "small-scale fading on (false = deterministic LOS plane wave, C3)", fading);
    cmd.AddValue("set1", "TR 38.821 Set-1 terminal and payload (nr S band)", set1);
    cmd.AddValue("gwDistanceKm", "feeder gateway distance north of the UEs (km); <= 0 = 5 ms constant",
                 gwDistanceKm);
    cmd.AddValue("ppWindowS", "ping-pong window for the measured KPI (s, TS 38.300 15.5.2.2)", ppWindowS);
    cmd.AddValue("netSim", "NetSimulyzer 3D JSON trace output path (empty = off)", netSimOut);
    cmd.AddValue("czml", "Cesium CZML 3D trace output path (empty = off)", czmlOut);
    bool liveScene = false;
    cmd.AddValue("liveScene", "Stream live scene frames to stdout (mid-run globe)", liveScene);
    cmd.Parse(argc, argv);
    g_simTime = duration;
    g_trigger = trigger;
    NS_ABORT_MSG_IF(candSource != "rrc" && candSource != "friis", "--candSource=rrc|friis");
    g_candSource = candSource;

    const bool useNr = (radio == "nr");
    // EIRP default (NT-02 calibration). The nr default used to be 70 dBm of
    // CONDUCTED power, about 88 dBm EIRP once the ~18 dB array gain is added,
    // ~11 dB above the TR 38.821 Table 6.1.1.1-1 Set-1 S-band payload
    // (34 dBW/MHz, 77 dBm over 20 MHz). On an S-band FR1 carrier the nr
    // default is now that Set-1 density, as ntn-v2x-real-stack does; an
    // explicit --satEirpDbm keeps meaning conducted power, and mmwave keeps
    // its historical 55.0 dBm.
    // The probe mirrors the cell's carrier and bandwidth and returns the
    // equivalent CONDUCTED power from the helper's own formula
    // (density + 10log10(BW_MHz) + 30 - ArrayGainDb()), so every other
    // consumer of satEirpDbm stays consistent with the radio.
    NtnRealStackHelper eirpProbe;
    eirpProbe.SetCarrierFrequencyHz(freqGhz * 1e9);
    const bool eirpFromTr38821 = satEirpDbm < 0.0 && useNr &&
                                 eirpProbe.GetCarrierFrequencyHz() > 1.0e9 &&
                                 eirpProbe.GetCarrierFrequencyHz() <= 7.125e9;
    if (eirpFromTr38821)
    {
        eirpProbe.SetSatEirpDensityDbwMhz(NtnRealStackHelper::kTr38821Set1SBandEirpDensityDbwMhz);
        satEirpDbm = eirpProbe.GetSatEirpDbm();
    }
    else if (satEirpDbm < 0.0)
    {
        satEirpDbm = useNr ? 70.0 : 55.0;
    }

    std::cout << "\n=== ntn-cho REAL-STACK (SGP4 (Vallado) orbits + TR 38.811 UEs) ===\n"
              << "  radio spine: " << (useNr ? "5G-LENA nr FR1 (30 kHz SCS) [A5(i)]" : "mmwave FR2")
              << " at " << freqGhz << " GHz, EIRP " << satEirpDbm << " dBm\n"
              << "  serving + candidate: SGP4 (Vallado) Walker neighbours (real pass dynamics)\n"
              << "  CHO candidate input: "
              << (candSource == "rrc" ? "UE measurement reports (measured SS-RSRP, TS 38.331 5.5.5)"
                                      : "ephemeris Friis extrapolation (comparison mode)")
              << "\n"
              << "  UEs: TR 38.811 class mobility (real ns-3 MobilityModel, ECEF)\n"
              << "  mechanism: " << trigger << (rachLess ? " + RACH-less (RCHO)" : "")
              << ", duration " << duration << " s\n\n";

    // ---- Real Walker-Delta orbits (SGP4 (Vallado) via SetElementsSgp4) ----
    WalkerConfig wcfg;
    wcfg.num_planes = 1;
    wcfg.total_sats = satsPerPlane;
    wcfg.altitude_km = altitudeKm;
    wcfg.inclination_deg = 53.0;
    wcfg.epoch_unix_s = 1735689600.0; // 2025-01-01
    const auto elements = WalkerConstellation::BuildDelta(wcfg);

    Ptr<Sgp4MobilityModel> serv = CreateObject<Sgp4MobilityModel>();
    serv->SetElementsSgp4(elements[0]);
    Ptr<Sgp4MobilityModel> nbrA = CreateObject<Sgp4MobilityModel>();
    nbrA->SetElementsSgp4(elements[1]);
    Ptr<Sgp4MobilityModel> nbrB = CreateObject<Sgp4MobilityModel>();
    nbrB->SetElementsSgp4(elements[satsPerPlane - 1]);

    // ---- TR 38.811 UEs in a small box at the serving sat's t=0 sub-point ----
    double subLat, subLon, subAlt;
    serv->GetGeodetic(subLat, subLon, subAlt);
    NodeContainer ueNodes;
    ueNodes.Create(numUes);
    NtnTr38811MobilityHelper ueMobility(/*seed=*/1);
    auto profile = NtnMobilityScenarios::MixedContinental();
    auto ueModels = ueMobility.Install(ueNodes, profile, subLat - 0.03, subLat + 0.03,
                                       subLon - 0.03, subLon + 0.03);
    g_ueMob = ueModels[0];

    // ---- Pick the genuinely APPROACHING in-plane neighbour as candidate ----
    const Vector ue0 = g_ueMob->GetPosition();
    auto approaching = [&ue0](Ptr<Sgp4MobilityModel> s) {
        const Vector p = s->GetPosition();
        const Vector v = s->GetVelocity();
        const Vector toUe(ue0.x - p.x, ue0.y - p.y, ue0.z - p.z);
        return (v.x * toUe.x + v.y * toUe.y + v.z * toUe.z) > 0.0;
    };
    Ptr<Sgp4MobilityModel> cand = approaching(nbrA) ? nbrA : nbrB;

    NodeContainer servSat;
    servSat.Create(1);
    servSat.Get(0)->AggregateObject(serv);
    NodeContainer candSat;
    candSat.Create(1);
    candSat.Get(0)->AggregateObject(cand);
    g_servMob = serv;
    g_candMob = cand;

    // ---- Real NR NTN cells + measured traffic (mmwave FR2 or nr FR1) ------
    //
    // GAP C6/H2 FIX: BOTH satellites are gNBs now.
    //
    // Previously only `servSat` was handed to Build(), so the "candidate cell"
    // (servingCellId + 100) had no NetDevice, no PHY and no radio at all: its
    // SINR was extrapolated closed-form from the serving cell's, and the CHO
    // could never hand a UE to it because it did not exist as a cell. With both
    // satellites built as real gNBs, the candidate is a genuine neighbour cell
    // and a CHO decision can drive an actual X2 reconfiguration-with-sync.
    NodeContainer gnbSats;
    gnbSats.Add(servSat.Get(0));
    gnbSats.Add(candSat.Get(0));

    NtnRealStackHelper rs;
    rs.SetRadioBackend(useNr ? NtnRealStackHelper::RadioBackend::Nr
                             : NtnRealStackHelper::RadioBackend::Mmwave);
    if (useNr)
    {
        rs.SetNumerology(1); // FR1 30 kHz SCS
    }
    rs.SetSimTime(Seconds(duration));
    rs.SetOutputDir(outputDir);
    rs.SetRunTag("ntn-cho-real-stack");
    rs.SetCarrierFrequencyHz(freqGhz * 1e9);
    if (eirpFromTr38821 && set1)
    {
        // C4: TR 38.821 Set-1 handheld (1T2R, NF 7 dB, 23 dBm) and the Set-1
        // S-band payload EIRP density (34 dBW/MHz).
        rs.ApplyTr38821Set1Defaults();
    }
    else if (eirpFromTr38821)
    {
        // NT-02: TR 38.821 Table 6.1.1.1-1 Set-1 S-band EIRP density. The
        // helper back-computes the conducted power from the bandwidth and
        // the array gain, and again in Build().
        rs.SetSatEirpDensityDbwMhz(NtnRealStackHelper::kTr38821Set1SBandEirpDensityDbwMhz);
    }
    else
    {
        // NT-02: an explicit value (or a default on a carrier outside the
        // Set-1 S-band reference) is CONDUCTED power at the array input.
        rs.SetSatConductedPowerDbm(satEirpDbm);
    }
    // Stand up the X2 between the two satellites so a handover can be executed.
    // The A3 algorithm this also installs is the vendored *baseline*; the CHO
    // decision below drives handovers explicitly via TriggerHandover, and
    // GetHandoverRequestedCount() vs GetHandoverCount() proves the loop closed.
    // A3 baseline config (the CHO drives handovers explicitly; these values
    // damp the vendored algorithm so the two do not fight over the same UE).
    rs.SetHandover(true, /*hysteresisDb=*/6.0, MilliSeconds(1024));
    // The CHO below is the ONLY decision maker: keep the X2 for its
    // TriggerHandover but do not install the vendored A3 as well. With both
    // armed, A3 moved the UE back within about a second of every CHO
    // execution under NTN fading (8 handovers for one CHO decision).
    rs.SetAutonomousHandover(false);
    // With A3 off the helper installs the TS 38.331 5.5.4.1 periodical
    // RSRP/RSRQ report (480 ms) on every gNB (Wave 1 C2), so the UE keeps
    // reporting serving and neighbour cells to the CHO.
    if (useNr && cellBeam)
    {
        // C2: SSB on an Earth-fixed cell beam centred on the UE site, so the
        // reported SS-RSRP is cell-level (TS 38.215 5.1.1), not whichever UE
        // each gNB happens to be serving.
        Ptr<ConstantPositionMobilityModel> cellCenter = CreateObject<ConstantPositionMobilityModel>();
        cellCenter->SetPosition(NtnRealStackHelper::GeodeticToEcef(subLat, subLon, 0.0));
        rs.SetSsbCellBeam(true, cellCenter);
    }
    NS_ABORT_MSG_IF(channel != "legacy" && channel != "ns3ntn", "--channel=legacy|ns3ntn");
    if (useNr)
    {
        rs.SetChannelModel(channel == "ns3ntn" ? NtnRealStackHelper::ChannelModel::Ns3Tr38811
                                               : NtnRealStackHelper::ChannelModel::Legacy);
    }
    rs.SetSmallScaleFading(fading);
    rs.Build(gnbSats, ueNodes);
    std::filesystem::create_directories(outputDir);
    rs.EnableMeasurementTrace(outputDir + "/meas_reports.csv");
    if (gwDistanceKm > 0.0)
    {
        // C5: feeder from a real gateway, tracking the serving satellite.
        rs.SetFeederGatewayGeodetic(subLat + gwDistanceKm / 111.195, subLon, 0.0);
    }
    // satEirpDbm was resolved before Build() and handed to the rest of the
    // example; make sure the radio landed on the same conducted power.
    NS_ABORT_MSG_IF(eirpFromTr38821 && std::abs(rs.GetSatEirpDbm() - satEirpDbm) > 1e-6,
                    "Set-1 conducted power mismatch: radio "
                        << rs.GetSatEirpDbm() << " dBm vs " << satEirpDbm << " dBm");
    rs.InstallTraffic(NtnRealStackHelper::TrafficProfile::EmbbStreaming,
                      Seconds(1.0), Seconds(duration - 0.5));
    rs.EnableAiFlowMonitor("ntn-cho-real-stack"); // WS2 KPM series (TS 28.552 names)
    g_rs = &rs;
    // P0-3: measured handover KPIs from every UE's RRC traces and packet arrivals.
    g_hoRec = CreateObject<NtnHoKpiRecorder>();
    g_hoRec->SetPingPongWindow(Seconds(ppWindowS));
    for (uint32_t i = 0; i < ueNodes.GetN(); ++i)
    {
        g_hoRec->AttachUe(i, ueNodes.Get(i));
    }
    g_trace = std::fopen((outputDir + "/cho_decision_trace.csv").c_str(), "w");
    if (g_trace)
    {
        std::fprintf(g_trace,
                     "t_s,serving_cell,candidate_cell,serv_sinr_meas_db,serv_rsrp_dbm,"
                     "cand_rsrp_dbm,cand_sinr_db,cand_source,report_age_s,cand_sinr_friis_pred_db,"
                     "serv_elev_deg,cand_elev_deg,serv_slant_km,cand_slant_km,cho_selected,"
                     "cho_state\n");
    }

    // ---- The CHO algorithm with the selected novel 6G mechanism ----
    Ptr<NtnChoHelper> choHelper = CreateObject<NtnChoHelper>();
    choHelper->SetCarrierFrequency(freqGhz * 1e9);
    choHelper->SetSatelliteTxPower(satEirpDbm);
    choHelper->SetTteMinimum(Seconds(tteMinSec));

    // CHO-5 FIX (2026-08-24): stand up the orbit predictor and TTE estimator.
    //
    // This example never called SetupConstellation(), so the helper handed the
    // algorithm a null predictor and a null estimator. Every predictor-dependent
    // trigger (d1, t1, tte-aware, pcho, d2) then returned early from its
    // condition check and could not fire for the whole run, while the summary
    // still named it as the mechanism under test. The DEFAULT trigger here is
    // pcho, so the example's out-of-the-box configuration was measuring nothing.
    {
        // P1-H: the predictor takes the real SGP4 satellites and the analytic
        // steered beam (TR 38.811 6.4.1, Set-1 4.4127 deg) directly; no SNS3 GEO
        // pattern container and no auxiliary SatMobilityModel stand-ins.
        choHelper->SetupConstellation(std::vector<Ptr<MobilityModel>>{serv, cand},
                                      /*peakGainDbi=*/30.0,
                                      /*beamwidth3dbDeg=*/4.4127);
        Ptr<NtnOrbitPredictor> orbit = choHelper->GetOrbitPredictor();
        NS_ABORT_MSG_IF(!orbit || orbit->CountFrozenSatellites() > 0,
                        "CHO predictor still has stationary satellites");
    }

    g_cho = choHelper->CreateChoAlgorithm();

    NtnChoAlgorithm::ChoConfig cfg = g_cho->GetConfig();
    // Full selectable trigger set: Rel-17 standardized (a3 / d1 / t1 / ta /
    // elevation), Rel-18 (d2), and the toolkit's novel mechanisms (tte-aware /
    // ltm / pcho).
    if (trigger == "a3")
    {
        cfg.triggerType = NtnChoAlgorithm::TRIGGER_EVENT_A3;
    }
    else if (trigger == "d1")
    {
        cfg.triggerType = NtnChoAlgorithm::TRIGGER_LOCATION_D1;
    }
    else if (trigger == "t1")
    {
        cfg.triggerType = NtnChoAlgorithm::TRIGGER_TIME_T1;
    }
    else if (trigger == "ta")
    {
        cfg.triggerType = NtnChoAlgorithm::TRIGGER_TIMING_ADVANCE;
    }
    else if (trigger == "elevation")
    {
        cfg.triggerType = NtnChoAlgorithm::TRIGGER_ELEVATION;
    }
    else if (trigger == "d2")
    {
        cfg.triggerType = NtnChoAlgorithm::TRIGGER_DISTANCE_D2;
    }
    else if (trigger == "ltm")
    {
        cfg.triggerType = NtnChoAlgorithm::TRIGGER_LTM_CONDITIONAL;
    }
    else if (trigger == "pcho")
    {
        cfg.triggerType = NtnChoAlgorithm::TRIGGER_TRAJECTORY_PREDICTIVE;
    }
    else
    {
        cfg.triggerType = NtnChoAlgorithm::TRIGGER_TTE_AWARE;
    }
    cfg.rachLess = rachLess;
    cfg.tteMinimum = Seconds(tteMinSec);
    // PCHO outage threshold sits just under the opening measured SINR so the
    // genuinely-declining pass crosses it during the run.
    cfg.qualityThreshold_dB = 8.0;
    cfg.predictionHorizon = Seconds(8.0);
    cfg.minPredictedTos = Seconds(3.0);
    // The PCHO forecast is a least-squares line through the 1 s samples,
    // extrapolated 8 s ahead. On MEASURED inputs the candidate carries the
    // 1 dB report quantization of TS 38.133 plus fading, and a line fitted
    // through 4 samples (the class default) turns a 1 dB step into a
    // 2 dB/8 s trend and fires on it (measured: handovers at 5 s and 8 s with
    // the candidate 1 to 3 dB below the serving cell). Requiring the fit to
    // span at least twice the horizon keeps the slope error near 0.02 dB/s.
    cfg.predictionMinSamples = static_cast<uint8_t>(std::min<uint32_t>(pchoMinSamples, 32));
    g_cho->Configure(cfg);

    // GAP C6 FIX: use the REAL cell ids of the two gNBs.
    //
    // This used to be `g_candCellId = g_servingCellId + 100` — an id that named
    // no gNB anywhere in the scenario. The CHO therefore "handed over" to a cell
    // that did not exist, which is why nothing could ever actuate.
    g_servingCellId = rs.GetGnbCellId(0);
    g_candCellId = rs.GetGnbCellId(1);
    NS_ABORT_MSG_IF(g_candCellId == 0 || g_candCellId == g_servingCellId,
                    "expected two distinct gNB cells (serving + candidate)");
    g_serving = g_servingCellId;
    g_cho->SetServingCell(g_servingCellId);
    g_cho->AddCandidateCell(g_servingCellId, 0, 0);
    g_cho->AddCandidateCell(g_candCellId, 1, 0);

    // ---- GAP H2 FIX: close the loop onto the real radio -------------------
    // The CHO decision now issues a genuine X2 reconfiguration-with-sync, and
    // the RRC's completion reports back into the algorithm so T304 can stop (or
    // expire into a real failure). Before this, ExecuteHandover only incremented
    // counters while the radio was moved — if at all — by the vendored A3
    // algorithm, which never saw the CHO decision.
    g_cho->SetHandoverExecutionCallback(
        MakeCallback(+[](uint16_t /*from*/, uint16_t to) {
            if (!g_rs->TriggerHandover(0, to))
            {
                // TriggerHandover already WARNed the reason; tell the algorithm
                // the handover did not happen rather than let T304 mask it.
                g_cho->NotifyHandoverComplete(to, false);
            }
        }));
    // The RRC reports completion; feed it back so success is the RADIO's word,
    // not the model's assumption.
    // CHO-26: only UE 0's completion closes UE 0's CHO; other UEs' handovers
    // (vendored A3) fire the same trace. At HandoverEndOk UE 0 already holds its
    // target-cell RNTI, so (cellId, rnti) identifies it.
    Config::Connect("/NodeList/*/DeviceList/*/NrGnbRrc/HandoverEndOk",
                    MakeCallback(+[](std::string /*ctx*/, uint64_t /*imsi*/, uint16_t cellId,
                                     uint16_t rnti) {
                        if (g_cho && g_rs && cellId == g_rs->GetUeServingCellId(0) &&
                            rnti == g_rs->GetUeRnti(0))
                        {
                            g_cho->NotifyHandoverComplete(cellId, true);
                        }
                    }));

    // ---- Optional 3D scene trace (NetSimulyzer + Cesium CZML) ----
    // One recorder taps the SGP4 sats + TR 38.811 UEs (all ECEF) and the
    // MEASURED serving SINR; fans out to both viewers. Off unless a path is set.
    if (!netSimOut.empty() || !czmlOut.empty() || liveScene)
    {
        g_servSatNodeId = servSat.Get(0)->GetId();
        g_candSatNodeId = candSat.Get(0)->GetId();
        g_ueNodeId = ueNodes.Get(0)->GetId();

        g_scene = CreateObject<ntnobs::NtnSceneRecorder>();
        g_scene->SetFrame(ntnobs::NtnSceneRecorder::EcefGlobal);
        g_scene->TrackNode(servSat.Get(0), ntnobs::NtnSceneRecorder::Sat, "serving-sat");
        g_scene->TrackNode(candSat.Get(0), ntnobs::NtnSceneRecorder::Sat, "candidate-sat");
        for (uint32_t i = 0; i < ueNodes.GetN(); ++i)
        {
            g_scene->TrackNode(ueNodes.Get(i),
                               ntnobs::NtnSceneRecorder::Ue,
                               "ue-" + std::to_string(i));
        }
        g_sinrSeries = g_scene->TrackKpiSeries(g_ueNodeId,
                                               ntnobs::NtnSceneRecorder::Sinr,
                                               "serving-SINR-dl");
        g_scene->TrackBeam(g_servSatNodeId, g_ueNodeId);
        if (!netSimOut.empty())
        {
            g_scene->EnableNetSimulyzer(netSimOut);
        }
        if (!czmlOut.empty())
        {
            g_scene->EnableCzml(czmlOut);
        }
        if (liveScene)
        {
            g_scene->EnableLiveStdout(true);
        }
        g_scene->Start();
    }

    Simulator::Schedule(Seconds(1.0), &ChoTick);

    Simulator::Stop(Seconds(duration));
    Simulator::Run();
    if (g_scene)
    {
        g_scene->Stop();
        std::cout << "  3D scene trace events:        " << g_scene->GetEventCount() << "\n";
    }
    rs.Collect();
    rs.WriteHealthReport();
    if (g_trace)
    {
        std::fclose(g_trace);
        g_trace = nullptr;
    }
    g_hoRec->WriteCsv(outputDir + "/handover_events.csv");
    g_hoRec->WriteKpm(outputDir + "/ho_kpm.csv");

    const auto st = g_cho->GetMechanismStats();
    std::cout << "\n--- CHO Summary (novel 6G mechanisms on REAL orbits + MEASURED SINR) ---\n"
              << "  mechanism:                    " << trigger
              << (rachLess ? " + RACH-less" : "") << "\n"
              << "  measured serving SINR (mean): " << rs.GetMeanDlSinrDb() << " dB\n"
              << "  measured DL throughput:       " << rs.GetRxThroughputMbps() << " Mbps\n"
              << "  CHO evaluations:              " << g_evals << "\n"
              << "  handovers executed:           " << g_handovers << "\n"
              << "  LTM fast switches:            " << st.ltmSwitches << "\n"
              << "  PCHO trajectory triggers:     " << st.pchoTriggers << "\n"
              << "  RACH-less executions:         " << st.rachLessExecutions << " (RACH paid: "
              << st.rachExecutions << ")\n"
              << "  last interruption (modeled):  " << st.lastInterruptionMs << " ms\n"
              << "  last pre-computed TA:         " << st.lastPreCompTaUs << " us (ephemeris)\n"
              << "  final serving cell:           " << rs.GetUeServingCellId(0)
              << " (radio; last CHO target " << g_serving << ")\n";
    const double ueReportShare =
        g_ticksAfter2s ? 100.0 * g_ticksWithReport / g_ticksAfter2s : 0.0;
    std::cout << "\n--- Measured inputs and handover KPIs (UE RRC traces + packet arrivals) ---\n"
              << "  UE 0 measurement reports:     " << rs.GetUeMeasurementReportCount(0)
              << " (TS 38.331 5.5.4.1 periodical, 480 ms)\n"
              << "  ticks on rrc-report (t>=2s):  " << g_ticksWithReport << "/" << g_ticksAfter2s
              << " (" << ueReportShare << " %)\n"
              << "  handovers (RRC, all UEs):     " << g_hoRec->GetHandoverCount(true)
              << " ok, " << g_hoRec->GetHandoverFailureCount() << " failed\n"
              << "  C-plane interruption (mean):  " << g_hoRec->GetMeanCplaneMs()
              << " ms (HandoverEndOk - HandoverStart)\n"
              << "  U-plane gap (mean):           " << g_hoRec->GetMeanUplaneGapMs()
              << " ms (UE 0 median packet spacing " << g_hoRec->GetMedianInterArrivalMs(0)
              << " ms)\n"
              << "  ping-pong (" << ppWindowS << " s / 5 s):        "
              << g_hoRec->GetPingPongCount(Seconds(ppWindowS)) << " / "
              << g_hoRec->GetPingPongCount(Seconds(5)) << "\n"
              << "  RLF (T310):                   " << g_hoRec->GetTotalRlfCount() << "\n"
              << "  time of stay (mean):          " << g_hoRec->GetMeanTimeOfStayS() << " s\n"
              << "  files: " << outputDir << "/handover_events.csv, ho_kpm.csv, "
              << "cho_decision_trace.csv, meas_reports.csv\n"
              << "  note: the modeled interruption is the algorithm's priced value; the measured\n"
              << "        one runs over ideal RRC with no air delay (A6), so it is a lower bound.\n";

    Simulator::Destroy();
    return 0;
}
