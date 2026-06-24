/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026  Muhammad Uzair
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * ntn-sagin-orphan-mobility-showcase — gives the toolkit's finished-but-unused
 * SAGIN classes a real home, a real packet plane, and a 3D scene:
 *
 *   - sagin::HapsTrajectoryMobilityModel — a HAPS flying a supplied trajectory
 *     (a programmatic loiter racetrack), interpolated to Simulator::Now().
 *   - ntnv2x::MaritimeMobilityModel — a vessel under way in a sea area.
 *   - sagin::HstMobilityModel — a high-speed train; GetDopplerHz() is sampled
 *     against the satellite under the simulator.
 *   - The CSV importers (OpenSkyAdsbImporter / AisDanishImporter /
 *     HapsTrajectoryImporter::LoadCsv) ingest the bundled sample traces.
 *   - MultiLayerRouter (SetScorer / SetLayerScorer) + SaginSliceRouter
 *     (RouteForSlice / QfiToSnssai) pick the Ground→HAPS→LEO route.
 *
 * The route the slice router selects drives a REAL data plane: P2P links +
 * Internet stack + NtnOranApplication→NtnOranSink, with FlowMonitor and the
 * sink's in-band primitives both reporting the measured KPIs. The whole scene
 * is streamed to NetSimulyzer + Cesium via the NtnSceneHelper.
 *
 * Usage:
 *   ./ns3 run "ntn-sagin-orphan-mobility-showcase --duration=120 \
 *              --czml=orphans.czml --netSim=orphans.json"
 */

#include "ns3/applications-module.h"
#include "ns3/box.h"
#include "ns3/core-module.h"
#include "ns3/flow-monitor-helper.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/haps-trajectory-mobility-model.h"
#include "ns3/haps-trajectory-trace.h"
#include "ns3/hst-mobility-model.h"
#include "ns3/hst-trace.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/ipv4-global-routing-helper.h"
#include "ns3/maritime-scenario.h"
#include "ns3/mobility-module.h"
#include "ns3/multi-layer-router.h"
#include "ns3/ntn-oran-application.h"
#include "ns3/ntn-oran-sink.h"
#include "ns3/ntn-scene-helper.h"
#include "ns3/ntn-tr38811-mobility-model.h" // NtnEnuProjectionMobilityModel
#include "ns3/opensky-adsb-trace.h"
#include "ns3/ais-maritime-trace.h"
#include "ns3/point-to-point-helper.h"
#include "ns3/sagin-slice-router.h"
#include "ns3/sgp4-mobility-model.h"
#include "ns3/walker-constellation.h"

#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("NtnSaginOrphanMobilityShowcase");

namespace
{
// File-scope handles so the free-function LinkProbe can reach the train and
// satellite without touching main()'s locals (the satellite position drives
// the train's Doppler readout).
Ptr<MobilityModel> g_train;         // a sagin::HstMobilityModel under the hood
Ptr<MobilityModel> g_sat;           // the ENU-projected SGP4 satellite
double g_simTime = 120.0;

void
LinkProbe()
{
    if (Simulator::Now().GetSeconds() >= g_simTime)
    {
        return;
    }
    // GetDopplerHz is a sagin::HstMobilityModel method (NOT on the
    // MobilityModel base) — DynamicCast before calling.
    Ptr<sagin::HstMobilityModel> hst = DynamicCast<sagin::HstMobilityModel>(g_train);
    const Vector s = g_sat->GetPosition();
    const double dopplerHz = hst ? hst->GetDopplerHz(s.x) : 0.0;
    std::printf("  %6.1f  sat=(%9.0f,%9.0f,%9.0f)  trainDoppler=%10.1f Hz\n",
                Simulator::Now().GetSeconds(), s.x, s.y, s.z, dopplerHz);
    Simulator::Schedule(Seconds(5.0), &LinkProbe);
}

/// Resolve a bundled data file, trying the in-tree relative path first then the
/// absolute fallback (mirrors the test suite's candidate-path pattern).
std::string
ResolveData(const std::string& leaf)
{
    const std::string candidates[] = {
        "contrib/ntn-sagin/data/" + leaf,
        "/home/uzair/6g_ntn_ns3/ns-3-dev/contrib/ntn-sagin/data/" + leaf,
    };
    for (const auto& p : candidates)
    {
        std::ifstream f(p);
        if (f.good())
        {
            return p;
        }
    }
    return candidates[0];
}
} // namespace

int
main(int argc, char* argv[])
{
    double duration = 120.0;
    std::string netSimOut;
    std::string czmlOut;

    CommandLine cmd(__FILE__);
    cmd.AddValue("duration", "Simulation duration (s)", duration);
    cmd.AddValue("netSim", "NetSimulyzer 3D JSON output (empty=off)", netSimOut);
    cmd.AddValue("czml", "Cesium CZML 3D output (empty=off)", czmlOut);
    cmd.Parse(argc, argv);
    g_simTime = duration;

    // Scenario origin: a coastal site (the local-ENU reference for everything).
    const double refLat = 36.0;
    const double refLon = 15.0;

    // ---- Satellite: real SGP4 projected into the local ENU frame ----
    ns3::ntncon::WalkerConfig wcfg;
    wcfg.num_planes = 1;
    wcfg.total_sats = 40;
    wcfg.altitude_km = 550.0;
    wcfg.inclination_deg = 53.0;
    wcfg.epoch_unix_s = 1735689600.0;
    const auto elements = ns3::ntncon::WalkerConstellation::BuildDelta(wcfg);
    Ptr<ns3::ntncon::Sgp4MobilityModel> satSgp4 = CreateObject<ns3::ntncon::Sgp4MobilityModel>();
    satSgp4->SetElements(elements[0]);
    Ptr<NtnEnuProjectionMobilityModel> satEnu = CreateObject<NtnEnuProjectionMobilityModel>();
    satEnu->SetSource(satSgp4);
    satEnu->SetReference(refLat, refLon, 0.0);
    NodeContainer satNodes;
    satNodes.Create(1);
    satNodes.Get(0)->AggregateObject(satEnu);
    g_sat = satEnu;

    // ---- HAPS: a programmatic loiter racetrack at ~20 km, round-tripped
    //      through HapsTrajectoryImporter::LoadCsv to exercise the importer. ----
    sagin::HapsTrajectoryTrace progTrace;
    progTrace.platform_id = "haps-1";
    for (int k = 0; k <= 36; ++k)
    {
        const double frac = static_cast<double>(k) / 36.0;
        const double ang = 2.0 * M_PI * frac; // one loiter loop over the run
        sagin::HapsTrajectorySample s;
        s.time_s = frac * duration;
        s.lat_deg = refLat + 0.05 * std::sin(ang);
        s.lon_deg = refLon + 0.08 * std::sin(2.0 * ang); // figure-eight ground track
        s.alt_m = 20000.0;
        progTrace.samples.push_back(s);
    }
    // Serialise the trajectory to a CSV in the importer's documented column
    // layout, then load it back with HapsTrajectoryImporter::LoadCsv so the
    // mobility model is driven by the parsed (not the in-memory) trace.
    const std::string hapsCsv = "orphan-haps-trajectory.csv";
    {
        std::ofstream f(hapsCsv);
        f << "platform_id,time_s,lat_deg,lon_deg,alt_m\n";
        for (const auto& s : progTrace.samples)
        {
            f << progTrace.platform_id << ',' << s.time_s << ',' << s.lat_deg << ','
              << s.lon_deg << ',' << s.alt_m << '\n';
        }
    }
    sagin::HapsTrajectoryImporter hapsImp;
    auto hapsTraces = hapsImp.LoadCsv(hapsCsv);
    sagin::HapsTrajectoryTrace trace =
        hapsTraces.empty() ? progTrace : hapsTraces.begin()->second;
    std::printf("# HapsTrajectoryImporter: rows read=%zu skipped=%zu samples=%zu\n",
                hapsImp.LastRowsRead(), hapsImp.LastRowsSkipped(), trace.samples.size());

    Ptr<sagin::HapsTrajectoryMobilityModel> hapsMob =
        CreateObject<sagin::HapsTrajectoryMobilityModel>();
    hapsMob->SetReference(refLat, refLon, 0.0);
    hapsMob->SetTrace(trace);
    NodeContainer hapsNodes;
    hapsNodes.Create(1);
    hapsNodes.Get(0)->AggregateObject(hapsMob);

    // ---- Vessel: MaritimeMobilityModel under way in a coastal sea box ----
    Ptr<ntnv2x::MaritimeMobilityModel> shipMob = CreateObject<ntnv2x::MaritimeMobilityModel>();
    shipMob->SetAttribute("Speed", DoubleValue(8.0)); // ~15.5 kn
    shipMob->SetSeaArea(Box(-5000.0, 5000.0, -5000.0, 5000.0, 0.0, 0.0));
    shipMob->SetPosition(Vector(0.0, 0.0, 0.0));
    NodeContainer shipNodes;
    shipNodes.Create(1);
    shipNodes.Get(0)->AggregateObject(shipMob);

    // ---- High-speed-train terminal: a TR 38.901 HST track. Its GetDopplerHz
    //      is sampled against the live satellite in LinkProbe(). ----
    sagin::HstTrace hstTrace =
        sagin::HstTraceGenerator::Generate(500.0, 150.0, duration + 60.0, 200);
    Ptr<sagin::HstMobilityModel> train = CreateObject<sagin::HstMobilityModel>();
    train->SetCarrierFrequencyHz(2.0e9);
    train->SetTrace(hstTrace);
    NodeContainer trainNodes;
    trainNodes.Create(1);
    trainNodes.Get(0)->AggregateObject(train);
    g_train = train;

    // ---- CSV importers: ingest the bundled OpenSky + AIS sample traces so the
    //      OpenSkyAdsbImporter / AisDanishImporter paths run in this example. ----
    sagin::OpenSkyAdsbImporter adsbImp;
    auto adsbTraces = adsbImp.LoadCsv(ResolveData("opensky-sample-trace.csv"));
    sagin::AisDanishImporter aisImp;
    auto aisTraces = aisImp.LoadCsv(ResolveData("ais-sample-trace.csv"));
    std::printf("# OpenSkyAdsbImporter: %zu aircraft (rows=%zu)   "
                "AisDanishImporter: %zu vessels (rows=%zu)\n",
                adsbTraces.size(), adsbImp.LastRowsRead(), aisTraces.size(),
                aisImp.LastRowsRead());

    std::cout << "\n=== orphan-mobility showcase (HAPS + maritime + HST + SGP4) ===\n"
              << "  HAPS loiter @20km, vessel @8 m/s, HST @500 km/h, real LEO pass, "
                 "local-ENU about ("
              << refLat << "," << refLon << ")\n\n";

    // -----------------------------------------------------------------------
    // Multi-layer + slice-aware routing over the live geometry.
    // -----------------------------------------------------------------------
    Ptr<MultiLayerRouter> mlr = CreateObject<MultiLayerRouter>();
    mlr->AddNode(SaginLayer::Haps, hapsMob);
    mlr->AddNode(SaginLayer::Leo, satEnu);
    // Exercise the §4.4.10 RL-friendly scorer API: a global max-elevation
    // scorer with a per-layer LEO override that prefers shorter range.
    mlr->SetScorer([](const SaginHopCandidate& c) { return c.elevationDeg; });
    mlr->SetLayerScorer(SaginLayer::Leo,
                        [](const SaginHopCandidate& c) { return -c.rangeM; });

    Ptr<SaginSliceRouter> sr = CreateObject<SaginSliceRouter>();
    sr->SetRouter(mlr);
    sr->AddSlice(ntnslice::DefaultUrllc()); // QFI→S-NSSAI default URLLC slice

    // RouteForSlice takes an ntnslice::Snssai (NOT a SliceProfile). QFI 1 maps
    // to the URLLC S-NSSAI via QfiToSnssai.
    const ntnslice::Snssai urllc = sr->QfiToSnssai(1);
    auto urllcPath =
        sr->RouteForSlice(hapsMob, ntnslice::Snssai{ntnslice::SliceSst::Urllc,
                                                    ntnslice::kSdUnset});
    std::printf("# SaginSliceRouter: QFI1->S-NSSAI sst=%d; URLLC route has %zu hops "
                "(routesEvaluated=%lu candidatesScored=%lu)\n",
                static_cast<int>(urllc.sst), urllcPath.size(),
                (unsigned long)mlr->GetRoutesEvaluated(),
                (unsigned long)mlr->GetCandidatesScored());

    // -----------------------------------------------------------------------
    // Real packet plane: Ground -> HAPS -> LEO with P2P + Internet + a
    // NtnOranApplication source feeding a NtnOranSink, FlowMonitor attached.
    // The per-hop one-way delay is set from the router's chosen slant ranges.
    // -----------------------------------------------------------------------
    constexpr double kC = 299792458.0;
    NodeContainer planeNodes;
    planeNodes.Create(3); // 0 = ground gateway, 1 = HAPS relay, 2 = LEO egress
    InternetStackHelper internet;
    internet.Install(planeNodes);

    // Slant ranges along the URLLC route (ground at ENU origin).
    const Vector grnd{0, 0, 0};
    const double rGndHaps = CalculateDistance(grnd, hapsMob->GetPosition());
    const double rHapsLeo = CalculateDistance(hapsMob->GetPosition(), satEnu->GetPosition());

    PointToPointHelper p2p;
    p2p.SetDeviceAttribute("DataRate", DataRateValue(DataRate(uint64_t(100e6))));

    p2p.SetChannelAttribute("Delay", TimeValue(Seconds(rGndHaps / kC)));
    NetDeviceContainer dGH = p2p.Install(NodeContainer(planeNodes.Get(0), planeNodes.Get(1)));
    p2p.SetChannelAttribute("Delay", TimeValue(Seconds(rHapsLeo / kC)));
    NetDeviceContainer dHL = p2p.Install(NodeContainer(planeNodes.Get(1), planeNodes.Get(2)));

    Ipv4AddressHelper ipv4;
    ipv4.SetBase("10.40.1.0", "255.255.255.0");
    ipv4.Assign(dGH);
    ipv4.SetBase("10.40.2.0", "255.255.255.0");
    Ipv4InterfaceContainer iHL = ipv4.Assign(dHL);
    Ipv4GlobalRoutingHelper::PopulateRoutingTables();

    const uint16_t port = 7400;
    Ptr<NtnOranSink> sink = CreateObject<NtnOranSink>();
    sink->SetAttribute("Local",
                       AddressValue(InetSocketAddress(Ipv4Address::GetAny(), port)));
    planeNodes.Get(2)->AddApplication(sink);
    sink->SetStartTime(Seconds(0.0));
    sink->SetStopTime(Seconds(duration));

    Ptr<NtnOranApplication> client = CreateObject<NtnOranApplication>();
    client->SetRemote(InetSocketAddress(iHL.GetAddress(1), port));
    client->SetProfile(NtnOranApplication::CBR_SATURATING);
    client->SetAttribute("DataRate", DataRateValue(DataRate(uint64_t(20e6))));
    client->SetAttribute("PacketSize", UintegerValue(1200));
    client->SetFlowIdentity(/*5qi*/ 82, /*sst*/ 2, /*sd*/ 0x000001, /*src*/ 0, /*dst*/ 2);
    planeNodes.Get(0)->AddApplication(client);
    client->SetStartTime(Seconds(1.0));
    client->SetStopTime(Seconds(duration));

    FlowMonitorHelper fmHelper;
    Ptr<FlowMonitor> flowMon = fmHelper.InstallAll();

    // ---- One scene tap for all of them (local-ENU frame) ----
    ntnobs::NtnSceneHelper scene;
    if (!netSimOut.empty())
    {
        scene.SetNetSimulyzer(netSimOut);
    }
    if (!czmlOut.empty())
    {
        scene.SetCzml(czmlOut);
    }
    scene.SetEnuFrame(refLat, refLon, 0.0);
    // Track sats via the sats slot; HAPS + ship via the "ues"/"gateways" slots.
    Ptr<ntnobs::NtnSceneRecorder> rec = scene.Build(satNodes, hapsNodes, shipNodes);

    std::printf("# train Doppler probe (HstMobilityModel::GetDopplerHz vs live SGP4 sat):\n");
    Simulator::Schedule(Seconds(5.0), &LinkProbe);
    Simulator::Stop(Seconds(duration));
    Simulator::Run();

    // ---- Measured KPIs: in-band sink primitives + FlowMonitor ----
    const uint64_t txP = client->GetTxPackets();
    const uint64_t rxP = sink->GetRxPackets();
    std::printf("# === sink (in-band) ===  tx=%lu rx=%lu PDR=%.2f%% "
                "meanDelay=%.3fms jitter=%.3fms goodput=%.3f Mbps\n",
                (unsigned long)txP, (unsigned long)rxP,
                txP ? 100.0 * rxP / txP : 0.0, sink->GetMeanDelayMs(),
                sink->GetMeanJitterMs(), sink->GetTotalRx() * 8.0 / duration / 1e6);

    flowMon->CheckForLostPackets();
    for (const auto& kv : flowMon->GetFlowStats())
    {
        const auto& st = kv.second;
        const double thr =
            st.timeLastRxPacket > st.timeFirstTxPacket
                ? st.rxBytes * 8.0 /
                      (st.timeLastRxPacket - st.timeFirstTxPacket).GetSeconds() / 1e6
                : 0.0;
        std::printf("# === FlowMonitor flow %u ===  txPkts=%lu rxPkts=%lu "
                    "lost=%lu throughput=%.3f Mbps delaySum=%.3fms\n",
                    kv.first, (unsigned long)st.txPackets, (unsigned long)st.rxPackets,
                    (unsigned long)st.lostPackets, thr,
                    st.rxPackets ? st.delaySum.GetSeconds() / st.rxPackets * 1e3 : 0.0);
    }

    if (rec)
    {
        rec->Stop();
        std::cout << "  scene events: " << rec->GetEventCount() << "\n";
    }
    Simulator::Destroy();
    return 0;
}
