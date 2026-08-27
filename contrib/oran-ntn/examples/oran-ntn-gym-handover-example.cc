/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026 Muhammad Uzair
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * oran-ntn-gym-handover-example — boots the OranNtnGymHandover RL environment
 * on the MEASURED radio (gap A1).
 *
 * The five OranNtnGym* environments (handover/beam-hop/slice/steering/
 * predictive) are the toolkit's measured RL bridge: GetObservation() reads a
 * real E2KpmReport (measured SINR/TBLER + live-ephemeris elevation/Doppler/TTE)
 * and ExecuteActions() emits a real handover decision. But NOTHING instantiated
 * them — no example, no test — so `ns3_ai_ntn.envs` could truthfully say "there
 * is no ns3-ai C++ env binary" and the Python envs fell back to synthetic
 * closed-form dynamics.
 *
 * This example wires the handover env to a genuine measured scenario:
 *   - a real Walker/SGP4 constellation (serving + candidate satellites),
 *   - a real NR-NTN access link (NtnRealStackHelper: SpectrumPhy + MAC + RRC),
 *   - the ho-predict xApp fed MEASURED PHY SINR every 100 ms,
 *   - OranNtnGymHandover reading that xApp.
 *
 * Two run modes:
 *   default (--gym=0): the env is STEPPED IN-PROCESS every control cycle — the
 *     observation vector is pulled from the measured plane and printed, and the
 *     xApp's own decision drives ExecuteActions(). This proves the env sees
 *     real state and is fully runnable with no Python peer (sweep-safe).
 *   --gym=1: registers OpenGymInterface::Get() so a Python RL agent
 *     (ns3_ai_ntn.envs Ns3Env) can connect over shared memory and train. Blocks
 *     waiting for the peer, so it is opt-in.
 */

#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/ntn-real-stack-helper.h"
#include "ns3/ntn-tr38811-mobility-model.h"
#include "ns3/oran-ntn-gym-handover.h"
#include "ns3/oran-ntn-helper.h"
#include "ns3/oran-ntn-near-rt-ric.h"
#include "ns3/oran-ntn-xapp-ho-predict.h"
#include "ns3/oran-ntn-phy-kpm-extractor.h"
#include "ns3/sgp4-mobility-model.h"
#include "ns3/walker-constellation.h"

#include "ns3/ns3-ai-gym-interface.h"

#include <cstdio>
#include <map>

using namespace ns3;
using ns3::ntncon::Sgp4MobilityModel;
using ns3::ntncon::WalkerConfig;
using ns3::ntncon::WalkerConstellation;

NS_LOG_COMPONENT_DEFINE("OranNtnGymHandoverExample");

namespace
{
// File scope so the RC actuator can be a plain function pointer (ns-3 callbacks
// take no captures). Set immediately after rs.Build().
NtnRealStackHelper* g_rs = nullptr;
} // namespace

int
main(int argc, char* argv[])
{
    double duration = 30.0;
    uint32_t numUes = 2;
    uint32_t numSats = 3; // serving + 2 candidates
    double altitudeKm = 600.0;
    double freqGhz = 2.0;
    double minElevDeg = 10.0;
    bool gym = false; // --gym=1 opens the Python RL peer; default steps in-process
    std::string outputDir = "oran-ntn-gym-handover-output";

    CommandLine cmd(__FILE__);
    cmd.AddValue("duration", "Simulation duration (s)", duration);
    cmd.AddValue("numUes", "Ground UEs", numUes);
    cmd.AddValue("numSats", "Satellites (serving + candidates)", numSats);
    cmd.AddValue("gym", "Open the ns3-ai Python RL peer (blocks for a connection)", gym);
    cmd.AddValue("outputDir", "Output directory", outputDir);
    cmd.Parse(argc, argv);
    numSats = std::max<uint32_t>(numSats, 2);

    // ---- Real Walker-Delta constellation (SGP4) ----
    WalkerConfig wcfg;
    wcfg.num_planes = 1;
    wcfg.total_sats = 24;
    wcfg.altitude_km = altitudeKm;
    wcfg.inclination_deg = 53.0;
    wcfg.epoch_unix_s = 1735689600.0;
    const auto elements = WalkerConstellation::BuildDelta(wcfg);

    NodeContainer satNodes;
    satNodes.Create(numSats);
    std::vector<Ptr<Sgp4MobilityModel>> satMobs(numSats);
    for (uint32_t s = 0; s < numSats; ++s)
    {
        satMobs[s] = CreateObject<Sgp4MobilityModel>();
        satMobs[s]->SetElements(elements[s % wcfg.total_sats]);
        satNodes.Get(s)->AggregateObject(satMobs[s]);
    }

    // ---- TR 38.811 UEs under the serving sat sub-point ----
    double subLat, subLon, subAlt;
    satMobs[0]->GetGeodetic(subLat, subLon, subAlt);
    NodeContainer ueNodes;
    ueNodes.Create(numUes);
    NtnTr38811MobilityHelper ueMobility(1);
    auto profile = NtnMobilityScenarios::MixedContinental();
    auto ueModels = ueMobility.Install(ueNodes, profile, subLat - 0.03, subLat + 0.03,
                                       subLon - 0.03, subLon + 0.03);

    // ---- Real NR-NTN access link ----
    // AI-01 FIX (2026-08-24): every satellite is a real gNB, not just the
    // serving one. Previously rs.Build() received a single node, so the RL
    // action could travel xApp -> RIC -> E2 -> RC and still have nowhere to
    // actuate: there was no second cell to hand over to. The candidates were
    // E2 nodes carrying injected KPM but had no radio.
    NtnRealStackHelper rs;
    rs.SetRadioBackend(NtnRealStackHelper::RadioBackend::Nr);
    rs.SetNumerology(1);
    rs.SetSimTime(Seconds(duration));
    rs.SetOutputDir(outputDir);
    rs.SetRunTag("oran-ntn-gym-handover");
    rs.SetCarrierFrequencyHz(freqGhz * 1e9);
    rs.SetHandover(true, /*hysteresisDb=*/3.0, MilliSeconds(256));
    rs.Build(satNodes, ueNodes);
    g_rs = &rs;
    rs.InstallTraffic(NtnRealStackHelper::TrafficProfile::EmbbStreaming, Seconds(1.0),
                      Seconds(duration - 0.5));

    // ---- O-RAN near-RT RIC + ho-predict xApp ----
    auto helper = CreateObject<OranNtnHelper>();
    helper->SetOutputDirectory(outputDir);
    auto nearRtRic = helper->CreateNearRtRic();
    // Satellite E2 nodes (1-indexed): serving = id 1, candidates = 2..numSats.
    // InjectKpmReport drops a report whose gNB id has no E2 node, so these must
    // exist for the measured KPM to reach the xApp.
    auto satE2Nodes = helper->CreateSatelliteE2Nodes(satNodes, nearRtRic);
    auto xapps = helper->CreateAllXapps(nearRtRic);
    helper->StartAllXapps(nearRtRic);
    Ptr<OranNtnXappHoPredict> hoPredict =
        DynamicCast<OranNtnXappHoPredict>(nearRtRic->GetXappByName("ho-predict"));
    NS_ABORT_MSG_IF(!hoPredict, "ho-predict xApp not found");

    // ORAN-01 / AI-01 FIX: give the RC path a real actuator. Without this the
    // helper logs "action LOGGED ONLY -- no handover actuator wired" and reports
    // actuated=false, so the whole RL loop terminated in a log line.
    // SetHandoverActuator had exactly one caller in the tree, inside a unit
    // test, and none in any shipped scenario.
    helper->SetHandoverActuator(MakeCallback(
        +[](E2RcAction action) -> bool { return g_rs->TriggerHandover(action.targetUeId,
                                                                            action.targetGnbId); }));

    // ---- The gym env, wired to the measured xApp (A1 FIX) ----
    Ptr<OranNtnGymHandover> env = CreateObject<OranNtnGymHandover>();
    env->SetXapp(hoPredict);
    env->SetMaxCandidates(numSats - 1);

    // ---- D3: a REAL production feed for the PHY-KPM extractor. The extractor's
    //      RegisterRnti/IngestMeasuredSample were previously only exercised by
    //      unit tests; here the scenario feeds it from NtnRealStackHelper's
    //      measured accessors each KPM period (exactly the path its docstring
    //      describes), so GetRealKpmReport() is a genuine measured producer. ----
    Ptr<OranNtnPhyKpmExtractor> kpmExtractor = CreateObject<OranNtnPhyKpmExtractor>();
    // ORAN-06: tell the extractor which carrier it is reporting on. Its RSRP and
    // C/N0 were each derived against their OWN hardcoded 100 MHz literal while
    // this scenario runs an FR1 carrier, so both were wrong and could disagree.
    // Take the value from the helper rather than restating it, or the two drift.
    kpmExtractor->SetRadioGeometry(rs.GetBandwidthHz(), 7.0);
    std::set<uint32_t> registeredRnti;

    // ---- KPM feed: MEASURED PHY SINR + live-ephemeris enrichment ----
    std::map<uint32_t, double> lastElev;
    std::map<uint32_t, uint64_t> lastRx;
    const double kpmIntervalS = 0.1;
    uint32_t g_steps = 0;
    double g_obsSum = 0.0;
    rs.RegisterPeriodicCallback(Seconds(kpmIntervalS), [&](Time) {
        for (uint32_t ue = 0; ue < numUes; ++ue)
        {
            const double sinr = rs.GetUeRecentSinrDb(ue);
            if (std::isnan(sinr))
            {
                continue;
            }
            const Vector uePos = ueModels[ue]->GetPosition();
            const Vector ueVel = ueModels[ue]->GetVelocity();
            const Vector sp = satMobs[0]->GetPosition();
            const Vector sv = satMobs[0]->GetVelocity();
            const double elev = ntngeo::ElevationDeg(uePos, sp);
            const double doppler = ntngeo::DopplerHz(uePos, ueVel, sp, sv, freqGhz * 1e9);
            double elevRate = 0.0;
            if (auto it = lastElev.find(0); it != lastElev.end())
            {
                elevRate = (elev - it->second) / kpmIntervalS;
            }
            lastElev[0] = elev;
            const double tte =
                (elevRate < -1e-3) ? std::max(1.0, (elev - minElevDeg) / -elevRate) : 600.0;
            const uint64_t rxBytes = rs.GetUeRxBytes(ue);
            double tbler = rs.GetUeRecentTbler(ue);
            if (std::isnan(tbler))
            {
                tbler = -1.0;
            }
            double thp = -1.0;
            if (auto b = lastRx.find(ue); b != lastRx.end() && rxBytes >= b->second)
            {
                thp = (rxBytes - b->second) * 8.0 / (kpmIntervalS * 1e6);
            }
            lastRx[ue] = rxBytes;
            // AI-07: the SERVING SINR is measured, so say so.
            //
            // This call passed ten arguments, leaving measPrbUtil and
            // sinrMeasured at their defaults, so the serving report was
            // labelled DERIVED even though its SINR came straight off
            // rs.GetUeRecentSinrDb(). The candidate reports below are correctly
            // left unflagged because theirs is a slant-range extrapolation; the
            // one report that had a right to the measured label was the one not
            // claiming it.
            helper->InjectKpmReport(1, ue, sinr, sinr - 95.0, tte, elev, doppler, thp, rxBytes,
                                    tbler, /*measPrbUtil=*/-1.0, /*sinrMeasured=*/true);
            // Publish the UE's TRUE serving cell (E2 node 1 in this scenario) so
            // the gym reads the serving baseline by identity, not by gnbId map
            // order. In a full RAN this is rs.GetUeServingCellId(ue).
            hoPredict->SetUeServingCell(ue, 1);

            // D3: feed the PHY-KPM extractor from the MEASURED radio. Register the
            // UE's real RNTI once, then ingest each measured sample.
            const uint16_t rnti = rs.GetUeRnti(ue);
            if (rnti != 0 && registeredRnti.find(ue) == registeredRnti.end())
            {
                kpmExtractor->RegisterRnti(rnti, ue, rs.GetUeServingCellId(ue));
                registeredRnti.insert(ue);
            }
            if (rnti != 0)
            {
                kpmExtractor->IngestMeasuredSample(rnti, sinr, rxBytes, tbler);
            }

            // Candidate satellites: real ephemeris elevation + Friis-ratio SINR
            // prediction off the measured serving baseline.
            //
            // AI-07: this is a free-space slant-range extrapolation. It carries
            // no antenna pattern, no per-satellite beam gain, no atmospheric
            // term and no interference, so it cannot reproduce TR 38.821
            // candidate-cell behaviour, and it is the feature the example's
            // action rule keys on. It stays unflagged (sinrMeasured defaults
            // false) so downstream consumers see it as derived, which is the
            // honest label for what it is.
            for (uint32_t c = 1; c < numSats; ++c)
            {
                const Vector cp = satMobs[c]->GetPosition();
                const Vector cv = satMobs[c]->GetVelocity();
                const double cElev = ntngeo::ElevationDeg(uePos, cp);
                const double cDop = ntngeo::DopplerHz(uePos, ueVel, cp, cv, freqGhz * 1e9);
                const double servSlant = ntngeo::SlantRangeM(uePos, sp);
                const double candSlant = ntngeo::SlantRangeM(uePos, cp);
                const double cSinr = sinr + 20.0 * std::log10(servSlant / std::max(1.0, candSlant));
                helper->InjectKpmReport(c + 1, ue, cSinr, cSinr - 95.0, 600.0, cElev, cDop);
            }
        }
    });

    // ---- In-process gym stepping (A1): pull the observation off the measured
    //      plane each control cycle; drive ExecuteActions with the xApp policy.
    if (!gym)
    {
        rs.RegisterPeriodicCallback(Seconds(1.0), [&](Time) {
            env->SetCurrentUe(0);
            Ptr<OpenGymDataContainer> obs = env->GetObservation();
            const float reward = env->GetReward();
            auto box = DynamicCast<OpenGymBoxContainer<float>>(obs);
            double servingSinr = 0.0;
            double bestCandSinr = -200.0;
            if (box && box->GetData().size() > 5)
            {
                servingSinr = box->GetData()[0];  // feature 0 = serving SINR
                bestCandSinr = box->GetData()[5]; // feature 5 = best candidate SINR
                g_obsSum += std::abs(servingSinr);
                ++g_steps;
            }

            // A1: CLOSE THE ACTION LEG. A greedy A3-style policy (best candidate
            // must beat serving by an offset) chooses the discrete action, and we
            // actually call ExecuteActions — the RL decision now leaves the env
            // and travels the xApp -> RIC -> E2 -> RC path (see the gate-10 test
            // for the end-to-end actuation proof). Action 0 = stay; action 1 =
            // hand over to the best candidate.
            const double a3OffsetDb = 3.0;
            uint32_t chosen = (bestCandSinr > servingSinr + a3OffsetDb) ? 1u : 0u;
            auto act = CreateObject<OpenGymDiscreteContainer>(numSats); // 0=stay, 1..=candidate
            act->SetValue(chosen);
            env->ExecuteActions(act);

            // D3 consumer: the extractor's measured report for UE 0 (fed from the
            // real radio above), proving the producer/consumer path is live.
            const E2KpmReport phyRep = kpmExtractor->GetRealKpmReport(0);

            std::printf("  %5.1fs  gym UE0 servingSINR=%6.2f dB  bestCand=%6.2f dB  "
                        "action=%u(%s)  reward=%6.3f  kpmExtractorSINR=%6.2f dB\n",
                        Simulator::Now().GetSeconds(), servingSinr, bestCandSinr, chosen,
                        chosen ? "HANDOVER" : "STAY", reward, phyRep.sinr_dB);
        });
    }
    else
    {
        // Python RL path: hand the env to the ns3-ai interface.
        //
        // AI-02 FIX (2026-08-24): registering the interface is not enough. The
        // agent is only stepped when someone calls Notify(), and there was no
        // Notify() call site anywhere in the tree, so --gym=1 opened a peer that
        // then sat idle for the whole run: the observation was never sent, no
        // action was ever received, and the example still exited reporting
        // success. Step it on the same control period the in-process branch
        // uses, and close the episode at the end so the peer does not hang.
        Ptr<OpenGymInterface> gymIface = OpenGymInterface::Get();
        env->SetOpenGymInterface(gymIface);
        rs.RegisterPeriodicCallback(Seconds(1.0), [&](Time) {
            env->SetCurrentUe(0);
            env->Notify(); // observation -> Python, action -> ExecuteActions
        });
        Simulator::Schedule(Seconds(duration - 0.01), [&]() { env->NotifySimulationEnd(); });
        std::printf("# --gym=1: stepping a Python ns3-ai RL agent every 1.0 s\n");
    }

    std::printf("# oran-ntn-gym-handover-example — OranNtnGymHandover on the MEASURED radio\n"
                "#   %u sats (1 serving + %u candidates), %u UEs, %.0fs, mode=%s\n",
                numSats, numSats - 1, numUes, duration, gym ? "python-rl" : "in-process");

    Simulator::Stop(Seconds(duration));
    Simulator::Run();
    rs.Collect();
    rs.WriteHealthReport();
    if (gym)
    {
        OpenGymInterface::Get()->NotifySimulationEnd();
    }

    // AI-07: name WHICH feature is measured.
    //
    // This line read "non-zero proves the env read the MEASURED plane", which is
    // true of feature 0 and of nothing else. Feature 5, bestCandSinr, is a
    // free-space slant-range extrapolation, and it is the feature the action
    // rule at `chosen = (bestCandSinr > servingSinr + 3.0)` actually keys on.
    // A statement true of one feature, printed as a claim about the whole
    // observation, is how a derived number acquires a measured reputation.
    std::printf("# === summary === gym steps=%u  mean|servingSINR| from env obs=%.2f dB "
                "(feature 0 is MEASURED, off RxPacketTraceUe; feature 5 "
                "bestCandSinr is a free-space slant-range extrapolation and is "
                "what the action rule keys on)\n",
                g_steps, g_steps ? g_obsSum / g_steps : 0.0);
    Simulator::Destroy();
    return 0;
}
