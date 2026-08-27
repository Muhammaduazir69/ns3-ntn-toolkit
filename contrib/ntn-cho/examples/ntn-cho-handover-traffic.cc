/*
 * SPDX-License-Identifier: GPL-2.0-only
 * Copyright (c) 2026 Muhammad Uzair (ns3-ntn-toolkit)
 *
 * ntn-cho-handover-traffic — REAL UDP downlink traffic to TR 38.811 UEs served
 * by a real mmwave NR NTN cell (NtnRealStackHelper: SpectrumPhy + MAC + HARQ +
 * RLC/PDCP + RRC + EPC), with the NtnChoAlgorithm deciding the handover on the
 * MEASURED serving SINR while the constellation flies Kepler+J2-secular Walker
 * orbits (Vallado SGP4 available via SetUseVallado):
 * the serving satellite passes zenith and recedes, the in-plane neighbour
 * approaches, and the handover falls out of the genuine orbital crossover.
 * UEs move under 3GPP TR 38.811 §6.1.1.1 class mobility (real MobilityModel).
 *
 * Mechanism selectable: --trigger=tte-aware|ltm|pcho, --rachLess=1 (RCHO).
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

#include <ns3/satellite-antenna-gain-pattern-container.h>
#include <ns3/satellite-constant-position-mobility-model.h>
#include <ns3/satellite-env-variables.h>
#include <ns3/singleton.h>
#include "ns3/ntn-real-stack-helper.h"
#include "ns3/ntn-tr38811-mobility-model.h"

#include "ns3/sgp4-mobility-model.h"
#include "ns3/walker-constellation.h"

#include <cmath>
#include <cstdio>

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
uint32_t g_evals = 0;
double g_simTime = 60.0;
double g_sinrAtHo = 0.0;

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

    const double servSinr = g_rs->GetUeRecentSinrDb(0);
    if (std::isnan(servSinr))
    {
        Simulator::Schedule(Seconds(1.0), &ChoTick);
        return;
    }
    // Ephemeris-predicted candidate SINR: measured baseline + real Friis ratio.
    const double candSinr = servSinr + 20.0 * std::log10(servSlant / std::max(1.0, candSlant));
    const double servGain = std::max(-20.0, (servElev - 45.0) / 5.0);
    const double candGain = std::max(-20.0, (candElev - 45.0) / 5.0);

    g_cho->UpdateMeasurement(g_servingCellId, servSinr, servGain);
    g_cho->UpdateMeasurement(g_candCellId, candSinr, candGain);
    g_cho->UpdateCandidateSlantRange(g_servingCellId, servSlant);
    g_cho->UpdateCandidateSlantRange(g_candCellId, candSlant);
    g_cho->EvaluateConditions();

    uint16_t chosen = g_cho->SelectBestCandidate();

    // Per-tick decision trace. Without this the example could report
    // "CHO decisions=0" with no way to tell whether the trigger never fired,
    // the candidate never became attractive, or the geometry never allowed it.
    // That opacity is how the six-trigger conformance gate stayed green while
    // measuring nothing (see rd-audit-2026-08-24, CVC-03).
    if (g_choTrace)
    {
        std::fprintf(g_choTrace,
                     "%.1f,%u,%u,%.3f,%.3f,%.2f,%.2f,%.1f,%.1f,%u\n",
                     Simulator::Now().GetSeconds(), g_servingCellId, g_candCellId, servSinr,
                     candSinr, servElev, candElev, servSlant / 1e3, candSlant / 1e3, chosen);
    }
    if (chosen == 0)
    {
        // TTE-aware fallback before the estimator admits: better-predicted
        // visible cell with a 1 dB execution offset (3GPP NTN CHO
        // execution-condition analogue, matched to the LTM hysteresis).
        chosen = (candSinr > servSinr + 1.0 && candElev >= 10.0) ? g_candCellId
                                                                 : g_servingCellId;
    }
    if (chosen != g_serving && chosen != 0 && chosen != g_servingCellId)
    {
        ++g_handovers;
        g_sinrAtHo = servSinr;
        g_cho->ExecuteHandover(chosen);
        const auto st = g_cho->GetMechanismStats();
        std::printf("  %6.1fs  HANDOVER cell %u -> %u  (servSINR meas=%.1f dB, candSINR "
                    "pred=%.1f dB, interruption=%.1f ms)\n",
                    Simulator::Now().GetSeconds(), g_serving, chosen, servSinr, candSinr,
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
    double freqGHz = 2.0;
    double satEirpDbm = -1.0; // sentinel: backend-appropriate default chosen below
    double tteMinSec = 3.0;
    double hoHystDb = 2.0; // A3 hysteresis for the actuated NR X2 handover
    std::string trigger = "tte-aware";
    std::string radio = "nr"; // radio backend: "nr" (5G-LENA FR1, 30 kHz SCS) | "mmwave" (FR2)
    bool rachLess = false;
    uint32_t satsPerPlane = 80;
    std::string outputDir = "ntn-cho-handover-traffic-output";

    CommandLine cmd(__FILE__);
    cmd.AddValue("simSeconds", "Simulation duration (s)", simSeconds);
    cmd.AddValue("numUes", "Number of TR 38.811 UEs on the serving cell", numUes);
    cmd.AddValue("leoAltKm", "Constellation altitude (km)", leoAltKm);
    cmd.AddValue("freqGHz", "Carrier frequency (GHz)", freqGHz);
    cmd.AddValue("satEirpDbm", "Satellite EIRP / gNB Tx power (dBm); -1 = backend default", satEirpDbm);
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
    cmd.Parse(argc, argv);
    g_simTime = simSeconds;

    const bool useNr = (radio != "mmwave");
    // Backend-appropriate EIRP default: nr's Friis LEO link needs ~70 dBm for a
    // healthy SINR; mmwave keeps its historical 55 dBm (zero regression).
    if (satEirpDbm < 0.0)
    {
        satEirpDbm = useNr ? 70.0 : 55.0;
    }

    std::printf("# ntn-cho-handover-traffic (CHO on MEASURED SINR over Kepler+J2 orbits)\n"
                "#   radio=%s mechanism=%s%s  sim=%.0fs alt=%.0fkm freq=%.1fGHz EIRP=%.1fdBm\n",
                useNr ? "nr-FR1" : "mmwave-FR2", trigger.c_str(), rachLess ? "+rachLess" : "",
                simSeconds, leoAltKm, freqGHz, satEirpDbm);

    // ---- Real Walker-Delta orbits (Kepler+J2-secular; Vallado SGP4 via
    //      SetUseVallado): serving + approaching neighbour ----
    WalkerConfig wcfg;
    wcfg.num_planes = 1;
    wcfg.total_sats = satsPerPlane;
    wcfg.altitude_km = leoAltKm;
    wcfg.inclination_deg = 53.0;
    wcfg.epoch_unix_s = 1735689600.0;
    const auto elements = WalkerConstellation::BuildDelta(wcfg);

    Ptr<Sgp4MobilityModel> serv = CreateObject<Sgp4MobilityModel>();
    serv->SetElements(elements[0]);
    Ptr<Sgp4MobilityModel> nbrA = CreateObject<Sgp4MobilityModel>();
    nbrA->SetElements(elements[1]);
    Ptr<Sgp4MobilityModel> nbrB = CreateObject<Sgp4MobilityModel>();
    nbrB->SetElements(elements[satsPerPlane - 1]);

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
    // NT-02: TR 38.821 Table 6.1.1.1-1 Set-1 downlink EIRP density for the
    // S-band LEO reference payload. Declared as a DENSITY so the helper
    // back-computes conducted power against the array gain instead of the
    // antenna being counted twice.
    rs.SetSatEirpDensityDbwMhz(
        NtnRealStackHelper::kTr38821Set1SBandEirpDensityDbwMhz);
    // ACTUATED handover: hand BOTH satellites to the radio helper as gNBs and
    // arm the real NR A3-RSRP + X2 handover, so a UE physically moves to the
    // neighbour cell on measured RSRP (not just a decision-model counter). The
    // candidate sat was previously created but never given to the radio.
    NodeContainer gnbSats;
    gnbSats.Add(servSat.Get(0));
    gnbSats.Add(candSat.Get(0));
    rs.SetHandover(true, hoHystDb, MilliSeconds(256));
    rs.Build(gnbSats, ueNodes);
    rs.InstallTraffic(NtnRealStackHelper::TrafficProfile::EmbbStreaming,
                      Seconds(1.0), Seconds(simSeconds - 0.5));
    rs.EnableAiFlowMonitor("ntn-cho-handover-traffic"); // WS2 KPM series (TS 28.552 names)
    g_rs = &rs;

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
        NodeContainer auxSats;
        auxSats.Create(2);
        Ptr<SatConstantPositionMobilityModel> auxServ =
            CreateObject<SatConstantPositionMobilityModel>();
        Ptr<SatConstantPositionMobilityModel> auxCand =
            CreateObject<SatConstantPositionMobilityModel>();
        auxSats.Get(0)->AggregateObject(auxServ);
        auxSats.Get(1)->AggregateObject(auxCand);

        Singleton<SatEnvVariables>::Get()->DoInitialize();
        Singleton<SatEnvVariables>::Get()->SetOutputVariables("ntn-cho-handover-traffic", "", true);
        Ptr<SatAntennaGainPatternContainer> agp = CreateObject<SatAntennaGainPatternContainer>(
            2,
            Singleton<SatEnvVariables>::Get()->LocateDataDirectory() +
                "/scenarios/geo-33E/antennapatterns");
        agp->ConfigureBeamsMobility(0, auxServ);
        agp->ConfigureBeamsMobility(1, auxCand);

        choHelper->SetupConstellation(auxSats, agp);
        Ptr<NtnOrbitPredictor> orbit = choHelper->GetOrbitPredictor();
        if (orbit)
        {
            orbit->SetKinematicsSource(0, serv);
            orbit->SetKinematicsSource(1, cand);
            orbit->SetGeometricBeam(/*peakGainDbi=*/30.0, /*beamwidth3dbDeg=*/4.4127);
            orbit->SetSteeredBeam(true);
            NS_ABORT_MSG_IF(orbit->CountFrozenSatellites() > 0,
                            "CHO predictor still has stationary satellites");
        }
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
    Config::Connect("/NodeList/*/DeviceList/*/NrGnbRrc/HandoverEndOk",
                    MakeCallback(+[](std::string /*ctx*/, uint64_t /*imsi*/, uint16_t cellId,
                                     uint16_t /*rnti*/) {
                        if (g_cho)
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
                         "cho_selected_cell\n");
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

    const auto st = g_cho->GetMechanismStats();
    const uint32_t actuatedHo = rs.GetHandoverCount();
    std::printf("# === summary ===  CHO decisions=%u (%s on measured SINR, real orbits)  "
                "ACTUATED X2 handovers=%u  serving-cell measured goodput=%.3f Mbps  "
                "mean SINR=%.2f dB  SINR@handover=%.2f dB  interruption(last)=%.1f ms  "
                "rachless=%u  final cell=%u\n",
                g_handovers, trigger.c_str(), actuatedHo, rs.GetRxThroughputMbps(),
                rs.GetMeanDlSinrDb(), g_sinrAtHo, st.lastInterruptionMs, st.rachLessExecutions,
                g_serving);
    // NOTE: "CHO decisions" is the ntn-cho algorithm's trigger count; "ACTUATED
    // X2 handovers" is how many times a UE was physically moved to the neighbour
    // gNB by the real NR A3-RSRP + X2 machinery. On a real LEO pass the RSRP
    // crossover between two co-altitude sats is slow, so the actuated count can
    // be small over a short window — exactly why ntn-cho adds elevation/TTE/D2
    // triggers. Lengthen --simSeconds or lower --hoHystDb to force a crossover.

    Simulator::Destroy();
    return 0;
}
