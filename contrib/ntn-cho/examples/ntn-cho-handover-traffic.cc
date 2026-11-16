/*
 * SPDX-License-Identifier: GPL-2.0-only
 * Copyright (c) 2026 Muhammad Uzair (ns3-ntn-toolkit)
 *
 * ntn-cho-handover-traffic — REAL UDP downlink traffic to TR 38.811 UEs served
 * by a real NR NTN cell (NtnRealStackHelper: SpectrumPhy + MAC + HARQ +
 * RLC/PDCP + RRC + EPC), with the NtnChoAlgorithm deciding the handover on
 * MEASURED inputs (serving SINR at the UE PHY, candidate from the UE's TS 38.331
 * 5.5.5 measurement reports on a cell-level SS-RSRP) while the constellation
 * flies SGP4 (Vallado) Walker orbits:
 * the serving satellite passes zenith and recedes, the in-plane neighbour
 * approaches, and the handover falls out of the genuine orbital crossover.
 * UEs move under 3GPP TR 38.811 §6.1.1.1 class mobility (real MobilityModel).
 *
 * Mechanism selectable: --trigger=tte-aware|ltm|pcho, --rachLess=1 (RCHO).
 * Only the CHO trigger decides (P0-4). The earlier example-level rule
 * "candidate predicted 1 dB better" is opt-in (--fallback=1) and its handovers
 * are reported as "baseline rule", never as CHO handovers.
 * Handover KPIs are measured by NtnHoKpiRecorder (P0-3): handover_events.csv,
 * ho_kpm.csv.
 * The data-continuity result is the measured serving-cell goodput across the
 * handover, reported in the summary.
 *
 * Quick test:  --simSeconds=60 --numUes=2 --trigger=tte-aware
 */
#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/ntn-cho-algorithm.h"
#include "ns3/ntn-cho-helper.h"
#include "ns3/ntn-orbit-predictor.h"
#include "ns3/ntn-ho-kpi-recorder.h"
#include "ntn-cho-measured-inputs.h"

#include "ns3/ntn-real-stack-helper.h"
#include "ns3/ntn-tr38811-mobility-model.h"

#include "ns3/sgp4-mobility-model.h"
#include "ns3/walker-constellation.h"

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <limits>

using namespace ns3;
using ns3::ntncon::Sgp4MobilityModel;
using ns3::ntncon::WalkerConfig;
using ns3::ntncon::WalkerConstellation;

NS_LOG_COMPONENT_DEFINE("NtnChoHandoverTraffic");

namespace
{
Ptr<NtnChoAlgorithm> g_cho;
NtnRealStackHelper* g_rs = nullptr;
std::FILE* g_choTrace = nullptr; // per-tick decision trace (cho_decision_trace.csv)
Ptr<MobilityModel> g_servMob;
Ptr<MobilityModel> g_candMob;
Ptr<NtnTr38811MobilityModel> g_ueMob;
uint16_t g_servingCellId = 0;
uint16_t g_candCellId = 0;
uint16_t g_serving = 0;
uint32_t g_handovers = 0;
uint32_t g_triggerFires = 0;    // ticks where the CHO trigger selected a cell
uint32_t g_triggerHandovers = 0; // handovers the trigger decided
uint32_t g_fallbackHandovers = 0; // handovers the fallback rule decided
uint32_t g_evals = 0;
double g_simTime = 60.0;
double g_sinrAtHo = 0.0;
bool g_fallback = false;          // P0-4: example baseline rule, opt-in
std::string g_candSource = "rrc"; // rrc | friis (comparison only)
std::string g_trigger;
Ptr<NtnHoKpiRecorder> g_hoRec;
uint32_t g_ticksWithReport = 0;
uint32_t g_ticksAfter2s = 0;

void
ChoTick()
{
    if (Simulator::Now().GetSeconds() >= g_simTime)
    {
        return;
    }
    ++g_evals;
    const Vector u = g_ueMob->GetPosition();
    const Vector sPos = g_servMob->GetPosition();
    const Vector cPos = g_candMob->GetPosition();
    const double servElev = ntngeo::ElevationDeg(u, sPos);
    const double candElev = ntngeo::ElevationDeg(u, cPos);
    const double servSlant = ntngeo::SlantRangeM(u, sPos);
    const double candSlant = ntngeo::SlantRangeM(u, cPos);

    // Measured at UE 0's PHY (linear mean of its TBs over the last tick).
    const double measSinr = g_rs->GetRadioBackend() == NtnRealStackHelper::RadioBackend::Nr
                                ? g_rs->TakeUeSinrWindowMeanDb(0)
                                : g_rs->GetUeRecentSinrDb(0);
    if (std::isnan(measSinr))
    {
        Simulator::Schedule(Seconds(1.0), &ChoTick);
        return;
    }
    // CHO-25: attribute the measurement to the cell the radio ACTUALLY serves.
    //
    // GetUeRecentSinrDb(0) is the SINR of whichever gNB UE 0 is attached to.
    // It was always filed under the original serving cell, so once the vendored
    // A3 (or the CHO itself) moved the UE to the neighbour, the neighbour's
    // measured SINR was reported as the old cell's and the old cell's predicted
    // value as the neighbour's: every decision after the first cell change ran
    // on swapped inputs. TS 38.331 section 5.5.3 measures the serving cell
    // (Mp) and neighbours (Mn) relative to the CURRENT PCell.
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
        // The radio changed cell without a CHO decision (vendored A3): follow it.
        g_cho->SetServingCell(radioCell);
    }
    // P0-2: candidate from the UE's measurement reports (TS 38.331 5.5.5,
    // L3-filtered at the UE per 5.5.3.2): serving SINR plus the reported
    // RSRP difference. The Friis extrapolation is only a comparison column
    // (and the input under --candSource=friis).
    const double servSinr = measSinr;
    const double candFriis = measSinr + 20.0 * std::log10(attSlant / std::max(1.0, othSlant));
    const ntnchoex::ReportedCandidate rep =
        ntnchoex::ReadReportedCandidate(*g_rs, 0, othCell, servSinr);
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
    // No invented elevation-to-gain map: NaN marks the gain as not provided.
    const double noGain = std::numeric_limits<double>::quiet_NaN();

    // CHO-21: hand the algorithm the UE's real position every tick.
    //
    // D1, D2, T1 and the TTE estimator all evaluate against m_uePosition, which
    // only StartMonitoring() used to set, and this scenario never called it
    // because it drives EvaluateConditions() on its own 1 s cadence. So those
    // triggers were measuring distances from a default-constructed coordinate
    // while the radio flew the real orbit: the geometric trigger classes could
    // not fire at any geometry, and the handovers the run reported for them came
    // from the fallback rule below instead.
    g_cho->UpdateUeKinematics(GeoCoordinate(u), g_ueMob->GetVelocity());

    g_cho->UpdateMeasurement(attCell, servSinr, noGain);
    if (!std::isnan(candSinr))
    {
        g_cho->UpdateMeasurement(othCell, candSinr, noGain);
    }
    g_cho->UpdateCandidateSlantRange(attCell, attSlant);
    g_cho->UpdateCandidateSlantRange(othCell, othSlant);
    g_cho->EvaluateConditions();

    uint16_t chosen = g_cho->SelectBestCandidate();

    // Per-tick decision trace. Without this the example could report
    // "CHO decisions=0" with no way to tell whether the trigger never fired,
    // the candidate never became attractive, or the geometry never allowed it.
    // That opacity is how the six-trigger conformance gate stayed green while
    // measuring nothing (see rd-audit-2026-08-24, CVC-03).
    // P0-4: who decided this tick's selection, if anyone.
    const bool fromTrigger = (chosen != 0);
    bool fromFallback = false;
    if (chosen == 0 && g_fallback && !std::isnan(candSinr))
    {
        // Opt-in example baseline rule (not CHO): candidate 1 dB better and
        // above 10 degrees. Its handovers are counted and printed apart.
        chosen = (candSinr > servSinr + 1.0 && othElev >= 10.0) ? othCell : attCell;
        fromFallback = (chosen == othCell);
    }
    if (g_choTrace)
    {
        std::fprintf(g_choTrace,
                     "%.1f,%u,%u,%.3f,%.3f,%.2f,%.2f,%.1f,%.1f,%u,%s,%.2f,%.2f,%.3f,%.3f,%s\n",
                     Simulator::Now().GetSeconds(), attCell, othCell, servSinr,
                     candSinr, attElev, othElev, attSlant / 1e3, othSlant / 1e3,
                     fromTrigger ? chosen : 0u, candSrc.c_str(), rep.servRsrpDbm,
                     rep.candRsrpDbm, rep.ageS, candFriis,
                     fromTrigger ? "cho" : (fromFallback ? "baseline-rule" : "none"));
    }
    // CHO-22: remember WHO decided, so the fallback is never reported as the
    // named trigger. Every committed "CHO decisions=N (a3 ...)" line was
    // produced by the rule below rather than by A3, and because the rule is the
    // same for all six classes, all six runs printed identical numbers.
    if (fromTrigger)
    {
        ++g_triggerFires;
    }
    // CHO-25: a decision is a cell change relative to the radio's serving cell,
    // and none is issued while a reconfiguration is still in flight (T304
    // running, TS 38.331 section 5.3.5.8.3). The old guard compared against the
    // ORIGINAL serving id, so a return handover could never be executed and a
    // UE already moved by A3 could be "handed over" to the cell it was on.
    const uint16_t curCell = (radioCell != 0) ? radioCell : g_cho->GetServingCellId();
    if (chosen != 0 && chosen != curCell &&
        g_cho->GetState() != NtnChoAlgorithm::CHO_EXECUTING)
    {
        ++g_handovers;
        if (fromTrigger)
        {
            ++g_triggerHandovers;
        }
        else
        {
            ++g_fallbackHandovers;
        }
        g_sinrAtHo = servSinr;
        g_cho->ExecuteHandover(chosen);
        const auto st = g_cho->GetMechanismStats();
        g_hoRec->AnnotateNext(0, st.lastInterruptionMs,
                              fromTrigger ? "cho:" + g_trigger : std::string("baseline-rule"));
        std::printf("  %6.1fs  HANDOVER cell %u -> %u by %s  (servSINR meas=%.1f dB, cand=%s "
                    "%.1f dB, modeled interruption=%.2f ms)\n",
                    Simulator::Now().GetSeconds(), curCell, chosen,
                    fromTrigger ? "CHO" : "baseline rule", servSinr,
                    candSrc == "rrc-report" ? "meas-rsrp" : candSrc.c_str(), candSinr,
                    st.lastInterruptionMs);
        g_serving = chosen;
    }
    Simulator::Schedule(Seconds(1.0), &ChoTick);
}
} // namespace

int
main(int argc, char* argv[])
{
    double simSeconds = 60.0;
    uint32_t numUes = 2;
    double leoAltKm = 550.0;
    double freqGHz = 2.185; // TS 38.101-5 n256 DOWNLINK block (2170-2200 MHz).
    double satEirpDbm = -1.0; // sentinel: backend-appropriate default chosen below
    double tteMinSec = 3.0;
    double hoHystDb = 2.0; // A3 hysteresis for the actuated NR X2 handover
    std::string trigger = "tte-aware";
    std::string radio = "nr"; // radio backend: "nr" (5G-LENA FR1, 30 kHz SCS) | "mmwave" (FR2)
    bool rachLess = false;
    uint32_t satsPerPlane = 80;
    std::string outputDir = "ntn-cho-handover-traffic-output";
    bool fallback = false;
    std::string candSource = "rrc";
    bool cellBeam = true;
    bool set1 = true;
    double gwDistanceKm = 500.0;
    double ppWindowS = 1.0;
    uint32_t pchoMinSamples = 16;

    CommandLine cmd(__FILE__);
    cmd.AddValue("simSeconds", "Simulation duration (s)", simSeconds);
    cmd.AddValue("numUes", "Number of TR 38.811 UEs on the serving cell", numUes);
    cmd.AddValue("leoAltKm", "Constellation altitude (km)", leoAltKm);
    cmd.AddValue("freqGHz", "Carrier frequency (GHz)", freqGHz);
    cmd.AddValue("satEirpDbm", "-1 = TR 38.821 Set-1 S-band EIRP density (34 dBW/MHz); otherwise conducted power at the array input (dBm)", satEirpDbm);
    cmd.AddValue("radio", "Radio backend: nr (5G-LENA FR1, 30 kHz SCS) | mmwave (FR2)", radio);
    cmd.AddValue("tteMinSec", "Minimum TTE for CHO admission (s)", tteMinSec);
    cmd.AddValue("hoHystDb", "A3 hysteresis (dB) for the actuated NR X2 handover", hoHystDb);
    cmd.AddValue("trigger",
                 "Handover trigger: tte-aware|ltm|pcho|a3|d1|t1|d2|elevation|ta "
                 "(a3/d1/t1 = Rel-17 CondEvents, d2 = Rel-18 CondEventD2, "
                 "elevation/ta = TR 38.821-studied mechanisms)",
                 trigger);
    cmd.AddValue("rachLess", "RACH-less execution (ephemeris TA pre-comp)", rachLess);
    cmd.AddValue("satsPerPlane", "Walker in-plane satellites (spacing)", satsPerPlane);
    cmd.AddValue("outputDir", "Output directory", outputDir);
    cmd.AddValue("fallback",
                 "enable the example's baseline rule (candidate 1 dB better) when the CHO trigger "
                 "selects nothing; its handovers are reported as baseline, not CHO",
                 fallback);
    cmd.AddValue("candSource", "CHO candidate input: rrc (measured reports) | friis (comparison)",
                 candSource);
    cmd.AddValue("cellBeam", "SSB on an Earth-fixed cell beam (cell-level SS-RSRP, nr only)", cellBeam);
    cmd.AddValue("set1", "TR 38.821 Set-1 terminal and payload (nr S band)", set1);
    cmd.AddValue("gwDistanceKm", "feeder gateway distance north of the UEs (km); <= 0 = 5 ms constant",
                 gwDistanceKm);
    cmd.AddValue("ppWindowS", "ping-pong window for the measured KPI (s)", ppWindowS);
    cmd.AddValue("pchoMinSamples", "PCHO minimum 1 s samples before forecasting", pchoMinSamples);
    cmd.Parse(argc, argv);
    g_simTime = simSeconds;
    g_fallback = fallback;
    g_trigger = trigger;
    NS_ABORT_MSG_IF(candSource != "rrc" && candSource != "friis", "--candSource=rrc|friis");
    g_candSource = candSource;

    const bool useNr = (radio != "mmwave");
    // EIRP default (NT-02 calibration). The nr default used to be 70 dBm of
    // CONDUCTED power, about 88 dBm EIRP once the ~18 dB array gain is added,
    // ~11 dB above the TR 38.821 Table 6.1.1.1-1 Set-1 S-band payload
    // (34 dBW/MHz, 77 dBm over 20 MHz). On an S-band FR1 carrier the
    // default is now that Set-1 density, as ntn-v2x-real-stack does; an
    // explicit --satEirpDbm keeps meaning conducted power. The radio here
    // already took the Set-1 density on BOTH backends, so the default is
    // backend-agnostic; what changes is that the printed budget and the
    // decision model now receive the same conducted power as the radio.
    // The probe mirrors the cell's carrier and bandwidth and returns the
    // equivalent CONDUCTED power from the helper's own formula
    // (density + 10log10(BW_MHz) + 30 - ArrayGainDb()), so every other
    // consumer of satEirpDbm stays consistent with the radio.
    NtnRealStackHelper eirpProbe;
    eirpProbe.SetCarrierFrequencyHz(freqGHz * 1e9);
    const bool eirpFromTr38821 = satEirpDbm < 0.0 &&
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

    std::printf("# ntn-cho-handover-traffic (CHO on MEASURED inputs over SGP4 (Vallado) orbits)\n"
                "#   radio=%s mechanism=%s%s  sim=%.0fs alt=%.0fkm freq=%.1fGHz EIRP=%.1fdBm\n",
                useNr ? "nr-FR1" : "mmwave-FR2", trigger.c_str(), rachLess ? "+rachLess" : "",
                simSeconds, leoAltKm, freqGHz, satEirpDbm);

    // ---- Real Walker-Delta orbits (SGP4 (Vallado) via SetElementsSgp4;
    //      SetUseVallado): serving + approaching neighbour ----
    WalkerConfig wcfg;
    wcfg.num_planes = 1;
    wcfg.total_sats = satsPerPlane;
    wcfg.altitude_km = leoAltKm;
    wcfg.inclination_deg = 53.0;
    wcfg.epoch_unix_s = 1735689600.0;
    const auto elements = WalkerConstellation::BuildDelta(wcfg);

    Ptr<Sgp4MobilityModel> serv = CreateObject<Sgp4MobilityModel>();
    serv->SetElementsSgp4(elements[0]);
    Ptr<Sgp4MobilityModel> nbrA = CreateObject<Sgp4MobilityModel>();
    nbrA->SetElementsSgp4(elements[1]);
    Ptr<Sgp4MobilityModel> nbrB = CreateObject<Sgp4MobilityModel>();
    nbrB->SetElementsSgp4(elements[satsPerPlane - 1]);

    // ---- TR 38.811 UEs under the serving sat's t=0 sub-point ----
    double subLat, subLon, subAlt;
    serv->GetGeodetic(subLat, subLon, subAlt);
    NodeContainer ueNodes;
    ueNodes.Create(numUes);
    NtnTr38811MobilityHelper ueMobility(1);
    auto profile = NtnMobilityScenarios::MixedContinental();
    auto ueModels = ueMobility.Install(ueNodes, profile, subLat - 0.03, subLat + 0.03,
                                       subLon - 0.03, subLon + 0.03);
    g_ueMob = ueModels[0];

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

    // ---- Real NR NTN serving link + measured traffic (mmwave FR2 or nr FR1) ----
    NtnRealStackHelper rs;
    rs.SetRadioBackend(useNr ? NtnRealStackHelper::RadioBackend::Nr
                             : NtnRealStackHelper::RadioBackend::Mmwave);
    if (useNr)
    {
        rs.SetNumerology(1); // FR1 30 kHz SCS
    }
    rs.SetSimTime(Seconds(simSeconds));
    rs.SetOutputDir(outputDir);
    rs.SetRunTag("ntn-cho-handover-traffic");
    rs.SetCarrierFrequencyHz(freqGHz * 1e9);
    if (eirpFromTr38821 && set1 && useNr)
    {
        // C4: TR 38.821 Set-1 handheld (1T2R, NF 7 dB) and payload density.
        rs.ApplyTr38821Set1Defaults();
    }
    else if (eirpFromTr38821)
    {
        // NT-02: TR 38.821 Table 6.1.1.1-1 Set-1 downlink EIRP density for
        // the S-band LEO reference payload. Declared as a DENSITY so the
        // helper back-computes conducted power against the array gain
        // instead of the antenna being counted twice.
        rs.SetSatEirpDensityDbwMhz(NtnRealStackHelper::kTr38821Set1SBandEirpDensityDbwMhz);
    }
    else
    {
        // NT-02: an explicit --satEirpDbm (or a default on a carrier outside
        // the Set-1 S-band reference) is CONDUCTED power at the array input.
        // It used to be ignored here: the radio always took the density
        // while the printed budget and the decision model took this value.
        rs.SetSatConductedPowerDbm(satEirpDbm);
    }
    // ACTUATED handover: hand BOTH satellites to the radio helper as gNBs and
    // arm the real NR A3-RSRP + X2 handover, so a UE physically moves to the
    // neighbour cell on measured RSRP (not just a decision-model counter). The
    // candidate sat was previously created but never given to the radio.
    NodeContainer gnbSats;
    gnbSats.Add(servSat.Get(0));
    gnbSats.Add(candSat.Get(0));
    rs.SetHandover(true, hoHystDb, MilliSeconds(256));
    // The CHO below is the ONLY decision maker: keep the X2 for its
    // TriggerHandover but do not install the vendored A3 as well. With both
    // armed, A3 moved the UE back within about a second of every CHO
    // execution under NTN fading (8 handovers for one CHO decision).
    rs.SetAutonomousHandover(false);
    // A3 off: the helper installs the periodical 480 ms RSRP/RSRQ report
    // (TS 38.331 5.5.4.1) on every gNB (Wave 1 C2).
    if (useNr && cellBeam)
    {
        // Cell-level SS-RSRP (TS 38.215 5.1.1): SSB on an Earth-fixed cell beam.
        Ptr<ConstantPositionMobilityModel> cellCenter = CreateObject<ConstantPositionMobilityModel>();
        cellCenter->SetPosition(NtnRealStackHelper::GeodeticToEcef(subLat, subLon, 0.0));
        rs.SetSsbCellBeam(true, cellCenter);
    }
    rs.Build(gnbSats, ueNodes);
    std::filesystem::create_directories(outputDir);
    rs.EnableMeasurementTrace(outputDir + "/meas_reports.csv");
    if (gwDistanceKm > 0.0)
    {
        rs.SetFeederGatewayGeodetic(subLat + gwDistanceKm / 111.195, subLon, 0.0); // C5
    }
    // satEirpDbm was resolved before Build() and handed to the rest of the
    // example; make sure the radio landed on the same conducted power.
    NS_ABORT_MSG_IF(eirpFromTr38821 && std::abs(rs.GetSatEirpDbm() - satEirpDbm) > 1e-6,
                    "Set-1 conducted power mismatch: radio "
                        << rs.GetSatEirpDbm() << " dBm vs " << satEirpDbm << " dBm");
    rs.InstallTraffic(NtnRealStackHelper::TrafficProfile::EmbbStreaming,
                      Seconds(1.0), Seconds(simSeconds - 0.5));
    rs.EnableAiFlowMonitor("ntn-cho-handover-traffic"); // WS2 KPM series (TS 28.552 names)
    g_rs = &rs;
    g_hoRec = CreateObject<NtnHoKpiRecorder>(); // P0-3
    g_hoRec->SetPingPongWindow(Seconds(ppWindowS));
    for (uint32_t i = 0; i < ueNodes.GetN(); ++i)
    {
        g_hoRec->AttachUe(i, ueNodes.Get(i));
    }

    // ---- CHO algorithm + mechanism ----
    Ptr<NtnChoHelper> choHelper = CreateObject<NtnChoHelper>();
    choHelper->SetCarrierFrequency(freqGHz * 1e9);
    choHelper->SetSatelliteTxPower(satEirpDbm);
    choHelper->SetTteMinimum(Seconds(tteMinSec));

    // CHO-5 FIX (2026-08-24): stand up the orbit predictor and TTE estimator.
    // Without them every predictor-dependent trigger (d1, t1, tte-aware, pcho,
    // d2) returned early from its condition check and could not fire, so the
    // handovers this scenario recorded for those modes came from the fallback
    // rule below rather than from the trigger it named.
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
    if (trigger == "ltm")
    {
        cfg.triggerType = NtnChoAlgorithm::TRIGGER_LTM_CONDITIONAL;
    }
    else if (trigger == "pcho")
    {
        cfg.triggerType = NtnChoAlgorithm::TRIGGER_TRAJECTORY_PREDICTIVE;
    }
    else if (trigger == "a3") // 3GPP class 1: measurement-based
    {
        cfg.triggerType = NtnChoAlgorithm::TRIGGER_EVENT_A3;
    }
    else if (trigger == "d1") // class 2: location-based (CondEventD1)
    {
        cfg.triggerType = NtnChoAlgorithm::TRIGGER_LOCATION_D1;
    }
    else if (trigger == "t1") // class 3: time-based (CondEventT1, ephemeris)
    {
        cfg.triggerType = NtnChoAlgorithm::TRIGGER_TIME_T1;
    }
    else if (trigger == "elevation") // class 4: elevation-based
    {
        cfg.triggerType = NtnChoAlgorithm::TRIGGER_ELEVATION;
    }
    else if (trigger == "ta") // class 5: timing-advance-based
    {
        cfg.triggerType = NtnChoAlgorithm::TRIGGER_TIMING_ADVANCE;
    }
    else if (trigger == "d2") // Rel-18 CondEventD2: moving reference locations
    {
        cfg.triggerType = NtnChoAlgorithm::TRIGGER_DISTANCE_D2;
        // Scenario-tuned thresholds (set per cell pair by the network in a
        // real deployment): the serving moving reference must have receded
        // 250 km from the UE (the serving sub-point starts at zenith and
        // recedes at the ~7.6 km/s ground-track speed), while the candidate
        // moving reference must be within 2000 km.
        cfg.d2Thresh1_m = 250e3;
        cfg.d2Thresh2_m = 2000e3;
    }
    else
    {
        cfg.triggerType = NtnChoAlgorithm::TRIGGER_TTE_AWARE;
    }
    cfg.orbitAltitudeKm = leoAltKm;
    cfg.rachLess = rachLess;
    cfg.tteMinimum = Seconds(tteMinSec);
    cfg.qualityThreshold_dB = 8.0;
    cfg.minPredictedTos = Seconds(3.0);
    // PCHO line fit must span >= 2x its 8 s horizon on 1 dB-quantized reports
    // (see ntn-cho-real-stack).
    cfg.predictionMinSamples = static_cast<uint8_t>(std::min<uint32_t>(pchoMinSamples, 32));
    g_cho->Configure(cfg);

    // Radio-agnostic serving cell id (mmwave or nr gNB under the hood).
    //
    // CVC-03 FIX (2026-08-24). The candidate id used to be
    // `g_servingCellId + 100`, a synthetic number naming no gNB in the
    // simulation, and no execution callback was registered. So every trigger
    // mode "executed" handovers that moved nothing: the only cell changes the
    // radio performed came from the vendored A3-RSRP algorithm, identically for
    // all six trigger settings, which made the six-trigger conformance gate
    // unable to tell them apart. Both satellites are already real gNBs here
    // (servSat + candSat, built through rs.Build with SetHandover enabled), so
    // resolving the second cell id and wiring the callback is all that was
    // missing. This mirrors ntn-cho-real-stack.cc.
    g_servingCellId = rs.GetGnbCellId(0);
    g_candCellId = rs.GetGnbCellId(1);
    NS_ABORT_MSG_IF(g_candCellId == 0 || g_candCellId == g_servingCellId,
                    "CHO needs two distinct real cells; got serving="
                        << g_servingCellId << " candidate=" << g_candCellId);
    g_serving = g_servingCellId;
    g_cho->SetServingCell(g_servingCellId);
    g_cho->AddCandidateCell(g_servingCellId, 0, 0);
    g_cho->AddCandidateCell(g_candCellId, 1, 0);

    // Turn the CHO decision into a real TS 38.331 reconfiguration-with-sync
    // over X2, and let the RRC (not the model) decide whether it succeeded.
    g_cho->SetHandoverExecutionCallback(MakeCallback(+[](uint16_t /*from*/, uint16_t to) {
        if (!g_rs->TriggerHandover(0, to))
        {
            g_cho->NotifyHandoverComplete(to, false);
        }
    }));
    // CHO-26: only UE 0's completions close UE 0's CHO. The trace fires for
    // every UE; with --numUes > 1 another terminal's A3 handover stopped this
    // UE's T304 and moved the algorithm's serving cell. The gNB fires
    // HandoverEndOk on RRCReconfigurationComplete, by which time UE 0 holds its
    // target-cell RNTI, so (cellId, rnti) identifies it radio-agnostically.
    Config::Connect("/NodeList/*/DeviceList/*/NrGnbRrc/HandoverEndOk",
                    MakeCallback(+[](std::string /*ctx*/, uint64_t /*imsi*/, uint16_t cellId,
                                     uint16_t rnti) {
                        if (g_cho && g_rs && cellId == g_rs->GetUeServingCellId(0) &&
                            rnti == g_rs->GetUeRnti(0))
                        {
                            g_cho->NotifyHandoverComplete(cellId, true);
                        }
                    }));

    {
        const std::string tracePath = outputDir + "/cho_decision_trace.csv";
        g_choTrace = std::fopen(tracePath.c_str(), "w");
        if (g_choTrace)
        {
            std::fprintf(g_choTrace,
                         "t_s,serving_cell,candidate_cell,serving_sinr_db,candidate_sinr_db,"
                         "serving_elev_deg,candidate_elev_deg,serving_slant_km,candidate_slant_km,"
                         "cho_selected_cell,cand_source,serv_rsrp_dbm,cand_rsrp_dbm,report_age_s,"
                         "cand_sinr_friis_pred_db,decided_by\n");
        }
    }

    Simulator::Schedule(Seconds(1.0), &ChoTick);

    Simulator::Stop(Seconds(simSeconds));
    Simulator::Run();
    rs.Collect();
    rs.WriteHealthReport();
    if (g_choTrace)
    {
        std::fclose(g_choTrace);
        g_choTrace = nullptr;
    }

    g_hoRec->WriteCsv(outputDir + "/handover_events.csv");
    g_hoRec->WriteKpm(outputDir + "/ho_kpm.csv");
    const auto st = g_cho->GetMechanismStats();
    const uint32_t actuatedHo = rs.GetHandoverCount();
    std::printf("# CHO trigger accounting: fires=%u  trigger-decided handovers=%u  "
                "fallback-decided handovers=%u (baseline rule %s)  (mechanism=%s)\n",
                g_triggerFires, g_triggerHandovers, g_fallbackHandovers,
                g_fallback ? "enabled" : "disabled", trigger.c_str());
    if (g_triggerHandovers == 0)
    {
        std::printf("# no CHO handover: the %s trigger admitted no candidate in %.0f s "
                    "(fires=%u; see cho_decision_trace.csv for the per-tick inputs)\n",
                    trigger.c_str(), simSeconds, g_triggerFires);
    }
    std::printf("# measured: UE0 reports=%u  ticks on rrc-report (t>=2s)=%u/%u  RRC handovers=%u ok "
                "%u failed  C-plane mean=%.3f ms  U-plane gap mean=%.3f ms (median spacing %.3f ms)  "
                "ping-pong(%.0fs)=%u ping-pong(5s)=%u  RLF=%u  ToS mean=%.2f s\n",
                rs.GetUeMeasurementReportCount(0), g_ticksWithReport, g_ticksAfter2s,
                g_hoRec->GetHandoverCount(true), g_hoRec->GetHandoverFailureCount(),
                g_hoRec->GetMeanCplaneMs(), g_hoRec->GetMeanUplaneGapMs(),
                g_hoRec->GetMedianInterArrivalMs(0), ppWindowS,
                g_hoRec->GetPingPongCount(Seconds(ppWindowS)), g_hoRec->GetPingPongCount(Seconds(5)),
                g_hoRec->GetTotalRlfCount(), g_hoRec->GetMeanTimeOfStayS());
    std::printf("# === summary ===  CHO decisions=%u (%s on measured SINR, real orbits)  "
                "ACTUATED X2 handovers=%u  serving-cell measured goodput=%.3f Mbps  "
                "mean SINR=%.2f dB  SINR@handover=%.2f dB  modeled interruption(last)=%.2f ms  "
                "rachless=%u  final cell=%u\n",
                g_handovers, trigger.c_str(), actuatedHo, rs.GetRxThroughputMbps(),
                rs.GetMeanDlSinrDb(), g_sinrAtHo, st.lastInterruptionMs, st.rachLessExecutions,
                rs.GetUeServingCellId(0)); // CHO-25: the radio's serving cell, not the model's
    // NOTE: "CHO decisions" is the ntn-cho algorithm's trigger count; "ACTUATED
    // X2 handovers" is how many times a UE was physically moved to the neighbour
    // gNB by the real NR A3-RSRP + X2 machinery. On a real LEO pass the RSRP
    // crossover between two co-altitude sats is slow, so the actuated count can
    // be small over a short window — exactly why ntn-cho adds elevation/TTE/D2
    // triggers. Lengthen --simSeconds or lower --hoHystDb to force a crossover.

    Simulator::Destroy();
    return 0;
}
