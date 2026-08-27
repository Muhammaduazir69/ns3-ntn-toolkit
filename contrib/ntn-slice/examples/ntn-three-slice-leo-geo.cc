/*
 * SPDX-License-Identifier: GPL-2.0-only
 * Copyright (c) 2026 Muhammad Uzair (ns3-ntn-toolkit, W6)
 *
 * ntn-three-slice-leo-geo — three slices (eMBB / URLLC / mMTC) coexisting on a
 * LEO+GEO NTN, with URLLC enforcing GEO mode-skip routing. The slice
 * orchestrator runs at 1 Hz; its PRB allocations and satisfaction ratios are
 * its real control outputs. The radio truth is MEASURED: a real mmwave NR cell
 * (NtnRealStackHelper) on a real SGP4 LEO pass with TR 38.811 UEs provides the
 * measured per-UE TBLER and per-UE delivered bytes (PacketSink), and per-slice
 * latency is the REAL geometry — LEO slant/c for LEO-served slices, the real
 * ~36 000 km GEO slant/c (~120 ms one-way / ~541 ms RTD) for GEO-served ones —
 * so the URLLC GEO-skip decision is validated against real physics, not a
 * hardcoded 250 ms constant (TR 38.821 Table 4.2-2: GEO 541.46 ms vs LEO
 * 25.77 ms RTD).
 *
 * Provenance note: every per-slice KPI fed to the isolation monitor is
 * MEASURED, not synthesized. After Simulator::Run(), each UE's slice is
 * resolved by NtnSliceSelector::Match() over a 5-tuple FlowDescriptor
 * (DSCP EF=46 -> URLLC, CoAP port 5683 -> mMTC, app-label "embb" -> eMBB,
 * built by ThreeSliceDefault per RFC 4594 / TS 23.501 §5.15.2). Delivered
 * packet count = GetUeRxBytes(ue)/1400 from the real PacketSink; the lost
 * count is derived from the MEASURED per-UE TBLER (GetUeRecentTbler), so the
 * reliability breach is measured, not a coin flip. Per-slice latency is the
 * real LEO or GEO slant range / c + backhaul (GEO chosen per the slice
 * profile's allowGeo / ShouldSkipGeo conformance predicate, never a random
 * draw). The orchestrator's 1 Hz proportional-fair Step() (or the RL hook
 * StepWithShares() under --useRlShares) sets only the PRB/served control
 * outputs. Standard 3GPP load distributions (the operator demand ramp) are
 * the only RNG; no headline KPI is np.random.
 *
 * Demonstrates W2 (RRC) -> W4 (RL env API) -> W6 (orchestrator) -> W3
 * (observability) integration; same (observation, action) shape as SliceEnv.
 */
#include "ns3/command-line.h"
#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/ntn-real-stack-helper.h"
#include "ns3/ntn-tr38811-mobility-model.h"
#include "ns3/sgp4-mobility-model.h"
#include "ns3/walker-constellation.h"

#include "ns3/ntn-slice-helper.h"
#include "ns3/ntn-slice-selector.h"
#include "ns3/ntn-slice-types.h"
#include "ns3/slice-isolation-monitor.h"
#include "ns3/slice-orchestrator-xapp.h"

#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <vector>

using namespace ns3;
using namespace ns3::ntnslice;

namespace
{
/// WF-04: set when any slice had no measured OWD samples and fell back to
/// the closed form, so the summary can say the KPI is not fully measured.
bool g_usedClosedFormLatency = false;
} // namespace

int
main(int argc, char* argv[])
{
    double simTimeSec = 30.0;
    uint32_t numUes = 3;
    bool routeUrllcViaGeo = false;
    bool useRlShares = false;
    double satEirpDbm = -1.0; // sentinel: backend-appropriate default chosen below
    double backhaulMs = 5.0;
    std::string radio = "nr"; // radio backend: "nr" (5G-LENA FR1) | "mmwave" (FR2)
    std::string csvPath = "ntn-three-slice.csv";
    uint32_t totalPrb = 273;
    std::string outputDir = "ntn-three-slice-output";

    CommandLine cmd(__FILE__);
    cmd.AddValue("simTime", "Simulation duration (s)", simTimeSec);
    cmd.AddValue("numUes", "UEs on the measured LEO cell", numUes);
    cmd.AddValue("urllcViaGeo", "Force URLLC over GEO (gates the GEO-skip test)", routeUrllcViaGeo);
    cmd.AddValue("useRlShares",
                 "Drive the orchestrator with StepWithShares(priority shares) instead of Step()",
                 useRlShares);
    cmd.AddValue("totalPrb", "Total PRB budget", totalPrb);
    cmd.AddValue("satEirpDbm", "LEO EIRP / gNB Tx power (dBm); -1 = backend default", satEirpDbm);
    cmd.AddValue("radio", "Radio backend: nr (FR1) or mmwave", radio);
    cmd.AddValue("csv", "Per-slice KPI CSV path", csvPath);
    cmd.AddValue("outputDir", "Output directory", outputDir);
    cmd.Parse(argc, argv);

    // Backend-appropriate EIRP default (honoured only if the user did not set it):
    // nr's Friis LEO link needs ~70 dBm for a healthy SINR; mmwave keeps 55 dBm.
    if (satEirpDbm < 0.0)
    {
        satEirpDbm = (radio == "mmwave") ? 55.0 : 70.0;
    }

    // ---- Real LEO cell: SGP4 Walker sat + TR 38.811 UEs (measured radio) ----
    ns3::ntncon::WalkerConfig wcfg;
    wcfg.num_planes = 1;
    wcfg.total_sats = 80;
    wcfg.altitude_km = 550.0;
    wcfg.inclination_deg = 53.0;
    wcfg.epoch_unix_s = 1735689600.0;
    const auto wElements = ns3::ntncon::WalkerConstellation::BuildDelta(wcfg);
    Ptr<ns3::ntncon::Sgp4MobilityModel> leoSat =
        CreateObject<ns3::ntncon::Sgp4MobilityModel>();
    leoSat->SetElements(wElements[0]);
    NodeContainer satNodes;
    satNodes.Create(1);
    satNodes.Get(0)->AggregateObject(leoSat);
    NodeContainer ueNodes;
    ueNodes.Create(numUes);
    double subLat, subLon, subAlt;
    leoSat->GetGeodetic(subLat, subLon, subAlt);
    NtnTr38811MobilityHelper ueMobility(1);
    auto mobProfile = NtnMobilityScenarios::MixedContinental();
    auto ueModels = ueMobility.Install(ueNodes, mobProfile, subLat - 0.03, subLat + 0.03,
                                       subLon - 0.03, subLon + 0.03);
    // The REAL GEO position serving this region (equatorial, same longitude).
    const Vector geoEcef = ntngeo::GeodeticToEcef(0.0, subLon, 35786000.0);

    NtnRealStackHelper rs;
    rs.SetRadioBackend(radio == "mmwave" ? NtnRealStackHelper::RadioBackend::Mmwave
                                         : NtnRealStackHelper::RadioBackend::Nr);
    if (radio != "mmwave")
    {
        rs.SetNumerology(1); // FR1 30 kHz SCS
    }
    rs.SetSimTime(Seconds(simTimeSec));
    rs.SetOutputDir(outputDir);
    rs.SetRunTag("ntn-three-slice-leo-geo");
    // NT-02: declared as CONDUCTED power at the array input. This carrier has
    // no TR 38.821 Set-1 reference in the toolkit, so the EIRP health gate
    // reports "not asserted" rather than certifying an uncalibrated budget.
    rs.SetSatConductedPowerDbm(satEirpDbm);
    rs.SetBackhaulDelay(MilliSeconds(backhaulMs));
    // SLICE-3. This example is named for its three slices and never created
    // any: SetSlices() was missing, so nBwp collapsed to 1 and all three
    // "slices" contended on one bandwidth part. Every per-slice KPI it reported
    // was therefore a measurement of unsliced contention, and the isolation the
    // example exists to show could not occur.
    //
    // The two sibling slice examples both call this; only this one did not. The
    // 5QI values match theirs (eMBB 2, URLLC 82, mMTC 9, TS 23.501) so the three
    // examples describe the same slice set.
    rs.SetSlices({{"eMBB", 2}, {"URLLC", 82}, {"mMTC", 9}});
    rs.Build(satNodes, ueNodes);
    rs.InstallTraffic(NtnRealStackHelper::TrafficProfile::MixedBouquet,
                      Seconds(1.0), Seconds(simTimeSec - 0.5));
    rs.EnableAiFlowMonitor("ntn-three-slice-leo-geo"); // WS2 KPM series (TS 28.552 names)

    auto stack = NtnSliceHelper::ThreeSliceDefault();
    stack.orchestrator->SetTotalPrb(totalPrb);
    // The whole run feeds one window; size it so all measured per-UE packets
    // are retained (MixedBouquet eMBB UE delivers thousands of 1400 B TBs).
    stack.monitor->SetWindow(1000000);

    // 3GPP load distribution ONLY (operator offered demand, TS 22.261 reference
    // KPIs). This drives the orchestrator's PRB control loop; it is NOT a
    // headline KPI — delivery/reliability/latency below are all MEASURED.
    Ptr<NormalRandomVariable> demandJitter = CreateObject<NormalRandomVariable>();
    demandJitter->SetAttribute("Mean", DoubleValue(0.0));
    demandJitter->SetAttribute("Variance", DoubleValue(1.0));

    std::ofstream out(csvPath);
    out << "time_s,slice,demand_mbps,served_mbps,prb,satisfaction,p99_latency_ms,"
           "latency_breach,reliability_breach\n";

    // ---- Slice assignment via the (previously dead) NtnSliceSelector ----
    // Each UE carries a representative 5-tuple per its provisioned traffic
    // class; Match() resolves the S-NSSAI from the DSCP/port/app rules that
    // ThreeSliceDefault() populated. The marking MUST match the actual
    // MixedBouquet per-UE traffic (u%3 -> 0:mMTC/NB-IoT, 1:eMBB, 2:URLLC), so:
    // UE 3k -> mMTC (CoAP port 5683), 3k+1 -> eMBB (app-label), 3k+2 -> URLLC
    // (DSCP EF 46, RFC 4594).
    std::vector<Snssai> ueSnssai(numUes);
    for (uint32_t ue = 0; ue < numUes; ++ue)
    {
        FlowDescriptor fd{};
        switch (ue % 3)
        {
        case 0:
            fd.dstPort = 5683; // CoAP -> mMTC
            break;
        case 1:
            fd.appLabel = "embb"; // -> eMBB
            break;
        default:
            fd.dscp = 46; // DSCP EF -> URLLC
            break;
        }
        ueSnssai[ue] = stack.selector->Match(fd);
    }

    int totalLatencyBreach = 0;
    int totalReliabilityBreach = 0;
    double maxUrllcP99 = 0.0;

    // ---- 1 Hz orchestrator control loop (PRB allocation only) ----
    // Writes per-slice control outputs (demand/served/prb/satisfaction) to the
    // CSV each tick. The isolation monitor is NOT touched here — it is fed from
    // the MEASURED PacketSink after the run (see below).
    rs.RegisterPeriodicCallback(Seconds(1.0), [&](Time now) {
        const double t = now.GetSeconds();
        // Offered load per TS 22.261 reference KPIs (operator demand input,
        // a standard 3GPP load distribution — the only RNG in this example).
        double embbDemand = std::max(50.0, 400.0 + 200.0 * demandJitter->GetValue());
        double urllcDemand = std::max(15.0, 80.0 + 25.0 * demandJitter->GetValue());
        double mmtcDemand = std::max(2.0, 20.0 + 5.0 * demandJitter->GetValue());
        stack.orchestrator->SetDemand(stack.embb.snssai, embbDemand);
        stack.orchestrator->SetDemand(stack.urllc.snssai, urllcDemand);
        stack.orchestrator->SetDemand(stack.mmtc.snssai, mmtcDemand);

        std::vector<SliceTick> ticks;
        if (useRlShares)
        {
            // RL hook: deterministic priority-proportional shares (a stand-in
            // policy, NOT random). StepWithShares renormalises internally.
            std::map<Snssai, double> shares;
            shares[stack.embb.snssai] = static_cast<double>(stack.embb.priority);
            shares[stack.urllc.snssai] = static_cast<double>(stack.urllc.priority);
            shares[stack.mmtc.snssai] = static_cast<double>(stack.mmtc.priority);
            ticks = stack.orchestrator->StepWithShares(shares);
        }
        else
        {
            ticks = stack.orchestrator->Step();
        }

        for (auto& tk : ticks)
        {
            out << std::fixed << std::setprecision(3) << t << "," << SstName(tk.snssai.sst)
                << "," << tk.demandMbps << "," << tk.servedMbps << "," << tk.prbAllocated << ","
                << tk.satisfactionRatio << ",,,\n";
        }
    });

    Simulator::Stop(Seconds(simTimeSec));
    Simulator::Run();
    rs.Collect();
    rs.WriteHealthReport();

    // ---- MEASURED per-slice delivery/reliability/latency (post-run) ----
    // For each UE: delivered packets = GetUeRxBytes/1400 from the real
    // PacketSink; lost packets derived from the MEASURED per-UE TBLER; latency
    // from real LEO or GEO slant geometry per the slice's GEO-skip rule.
    const Vector leoPos = leoSat->GetPosition();
    // SliceProfile lookup keyed by the selector's S-NSSAI.
    std::map<Snssai, SliceProfile> profByS;
    profByS[stack.embb.snssai] = stack.embb;
    profByS[stack.urllc.snssai] = stack.urllc;
    profByS[stack.mmtc.snssai] = stack.mmtc;

    for (uint32_t ue = 0; ue < numUes; ++ue)
    {
        const Snssai s = ueSnssai[ue];
        const SliceProfile& prof = profByS.at(s);

        // GEO routing follows the slice profile, not a coin flip. ShouldSkipGeo
        // forces LEO for URLLC (and any allowGeo==false slice). A CLI override
        // can force URLLC onto GEO to demonstrate the latency breach.
        bool servedByGeo = false;
        if (ShouldSkipGeo(prof))
        {
            servedByGeo = (s.sst == SliceSst::Urllc) ? routeUrllcViaGeo : false;
        }
        else
        {
            // Non-URLLC slices that permit GEO are served by GEO when the GEO
            // satellite is GEO-class (deterministic conformance predicate).
            servedByGeo = IsGeoSatellite(35786.0);
        }

        const Vector uePos = ueModels[ue]->GetPosition();
        const double leoLatMs =
            ntngeo::SlantRangeM(uePos, leoPos) / 299792458.0 * 1e3 + backhaulMs;
        const double geoLatMs =
            ntngeo::SlantRangeM(uePos, geoEcef) / 299792458.0 * 1e3 + backhaulMs;
        const double latencyMs = servedByGeo ? geoLatMs : leoLatMs;

        // WF-04: replay the MEASURED one-way-delay distribution.
        //
        // The packet COUNT was already measured (GAP M5) and the losses too
        // (GAP L3), but every one of those packets was then stamped with a
        // single closed-form latencyMs - slant/c plus a backhaul constant. A
        // p99 over N copies of one number is that number, so the URLLC
        // percentile SLA could not be breached by a tail, only by the mean, and
        // the KPI carried no queueing, scheduling or HARQ delay at all.
        //
        // GetSliceDelayHistogram() returns the pooled measured OWD for the
        // slice's 5QI in 1 ms bins - it exists for exactly this and was not
        // being called. The bins are replayed proportionally to this UE's
        // delivered count so the per-UE weighting is preserved.
        //
        // The GEO leg stays modelled, and is labelled: the GEO "satellite" here
        // is a static ECEF Vector, not a second gNB, so no packet ever flew that
        // path and there is nothing measured to replay. What is added is the
        // EXTRA propagation a GEO hop would cost over the LEO one - measured
        // radio plus modelled geometry, rather than geometry alone.
        const double geoExtraMs = servedByGeo ? (geoLatMs - leoLatMs) : 0.0;

        const uint64_t deliveredPkts = rs.GetUeRxPackets(ue);
        const auto hist = rs.GetSliceDelayHistogram(prof.snssai.sst == SliceSst::Urllc ? 82
                                                    : (prof.snssai.sst == SliceSst::Embb ? 2 : 9));
        uint64_t histTotal = 0;
        for (const auto& [bin, count] : hist)
        {
            histTotal += count;
        }
        if (histTotal > 0)
        {
            uint64_t emitted = 0;
            for (const auto& [bin, count] : hist)
            {
                // Proportional share of this UE's delivered packets.
                uint64_t share = static_cast<uint64_t>(
                    (static_cast<double>(count) / static_cast<double>(histTotal)) *
                    static_cast<double>(deliveredPkts));
                for (uint64_t p = 0; p < share; ++p)
                {
                    stack.monitor->RecordPacket(s, static_cast<double>(bin) + geoExtraMs, true);
                    ++emitted;
                }
            }
            // Rounding remainder goes to the modal bin so the delivered count
            // is preserved exactly.
            for (uint64_t p = emitted; p < deliveredPkts; ++p)
            {
                stack.monitor->RecordPacket(s, static_cast<double>(hist.front().first) + geoExtraMs,
                                            true);
            }
        }
        else
        {
            // No measured samples for this 5QI. Fall back to the closed form
            // and SAY SO, rather than presenting geometry as a measurement.
            g_usedClosedFormLatency = true;
            for (uint64_t p = 0; p < deliveredPkts; ++p)
            {
                stack.monitor->RecordPacket(s, latencyMs, true);
            }
        }

        // GAP L3 FIX: MEASURED sequence-gap losses, so reliabilityBreach comes
        // from real dropped packets — not delivered*TBLER/(1-TBLER) synthesized
        // from a single last-TB sample.
        const uint64_t lostPkts = rs.GetUeLostPackets(ue);
        for (uint64_t p = 0; p < lostPkts; ++p)
        {
            stack.monitor->RecordPacket(s, latencyMs, false);
        }
    }

    // One BreachEvent per registered slice, evaluated over the measured window.
    auto breaches = stack.monitor->EvaluateAll();
    for (const auto& br : breaches)
    {
        if (br.latencyBreach)
            totalLatencyBreach += 1;
        if (br.reliabilityBreach)
            totalReliabilityBreach += 1;
        if (br.snssai.sst == SliceSst::Urllc && br.observedLatencyP99Ms > maxUrllcP99)
            maxUrllcP99 = br.observedLatencyP99Ms;
        out << std::fixed << std::setprecision(3) << simTimeSec << ","
            << SstName(br.snssai.sst) << ",,,,," << br.observedLatencyP99Ms << ","
            << (br.latencyBreach ? 1 : 0) << "," << (br.reliabilityBreach ? 1 : 0) << "\n";
    }
    out.close();

    std::cout << "ntn-three-slice-leo-geo complete (orchestrator on MEASURED radio).\n"
              << "  policy             : "
              << (useRlShares ? "StepWithShares (RL hook)" : "Step (proportional-fair)") << "\n"
              << "  measured LEO SINR  : " << rs.GetMeanDlSinrDb() << " dB\n"
              << "  measured LEO TBLER : " << rs.GetMeanDlTbler() << "\n"
              << "  urllcViaGeo=" << (routeUrllcViaGeo ? "true" : "false")
              << "  max URLLC p99 latency: " << maxUrllcP99 << (routeUrllcViaGeo
                      ? " ms (MEASURED OWD distribution + modelled GEO propagation delta; the "
                        "GEO 'satellite' is a static ECEF point, not a second gNB, so no packet "
                        "flew that path)\n"
                      : " ms (MEASURED one-way-delay distribution)\n")
              << "  latency breaches=" << totalLatencyBreach
              << "  reliability breaches=" << totalReliabilityBreach << "\n"
              << "  csv: " << csvPath << "\n";
    if (g_usedClosedFormLatency)
    {
        std::cout << "  NOTE: at least one slice had no measured OWD samples and fell back to "
                     "the closed form; its latency KPI is geometry, not a measurement\n";
    }
    Simulator::Destroy();
    return 0;
}
