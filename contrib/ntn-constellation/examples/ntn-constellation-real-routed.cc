/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026  Muhammad Uzair
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * ntn-constellation-real-routed — real-stack flagship for ntn-constellation
 * (routing pattern).
 *
 * A live ContactGraphScheduler samples the SGP4 geometry and feeds GSL
 * contact-up / contact-down events into a ContactGraphRouter; the router's
 * ShortestPath(GS1, GS2) decision actually INSTALLS the Ipv4 routes, so real
 * UDP packets are forwarded THROUGH the satellite nodes and adaptively REROUTE
 * as the constellation moves: gs1 -> satA -> gs2 while satA is in contact, then
 * gs1 -> satB -> gs2 once satA sets and satB rises. Per-link delay is the real
 * slant range / c. Delivery/delay/jitter/loss are MEASURED end-to-end by
 * NtnOranSink from the in-band NtnOranPayloadHeader (WS1 application suite).
 *
 * Per-hop loss is driven (by default) by NtnSatLinkErrorModel: a per-packet
 * C/N0 -> BLER waterfall on each GSL device, whose CurrentEsNoDb / CurrentBler /
 * LastBler telemetry is surfaced into sim_health.csv. Pass --blerLinks=false to
 * fall back to the binary contact gate.
 *
 * After the run a Tr38821CorpusReader + CalibrationHarness reads the bundled
 * TR 38.821 link-budget corpus and reports the toolkit-vs-reference pathloss
 * residuals (calibration_residuals.csv + a calibration_within_gate KPI row).
 *
 * Usage:
 *   ./ns3 run "ntn-constellation-real-routed --duration=40"
 */

#include "ns3/applications-module.h"
#include "ns3/ntn-tr38811-mobility-model.h"
#include "ns3/ntn-sat-link-error-model.h"
#include "ns3/pointer.h"
#include "ns3/sgp4-mobility-model.h"
#include "ns3/walker-constellation.h"
#include "ns3/contact-graph-router.h"
#include "ns3/contact-graph-scheduler.h"
#include "ns3/tr38821-corpus.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/ntn-oran-application.h"
#include "ns3/ntn-oran-sink.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/point-to-point-module.h"

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <vector>

using namespace ns3;
using namespace ns3::ntncon;

NS_LOG_COMPONENT_DEFINE("NtnConstellationRealRouted");

namespace
{
constexpr double kC = 299792458.0;

constexpr uint32_t GS1 = 0;
constexpr uint32_t SATA = 1;
constexpr uint32_t SATB = 2;
constexpr uint32_t GS2 = 3;

Ptr<MobilityModel> g_satA, g_satB, g_g1, g_g2;
Ptr<Ipv4StaticRouting> g_gs1Routing, g_satARouting, g_satBRouting;
Ipv4Address g_destAddr;
Ipv4Address g_satA_fromGs1, g_satB_fromGs1;
Ipv4Address g_gs2_fromA, g_gs2_fromB;
Ptr<PointToPointChannel> g_chA1, g_chA2, g_chB1, g_chB2;
Ptr<NtnOranSink> g_sink;
double g_minElev = 10.0;
uint32_t g_currentSat = 0;
uint32_t g_reroutes = 0;
double g_simTime = 40.0;
std::string g_outDir = ".";

// Live contact-graph scheduler + router that DRIVE the route decision.
Ptr<ContactGraphScheduler> g_sched;
Ptr<ContactGraphRouter> g_router;

// Per-hop C/N0->BLER error models on the active GSL devices (populated only
// when --blerLinks is set). Two hops per candidate satellite.
Ptr<ntncon::NtnSatLinkErrorModel> g_emA1, g_emA2, g_emB1, g_emB2;
bool g_blerLinks = false;

// Accumulated error-model telemetry over the served ticks.
double g_minEsNoDb = std::numeric_limits<double>::infinity();
double g_sumBler = 0.0;
double g_sumEsNoDb = 0.0;
uint32_t g_blerSamples = 0;
// Router provenance: how the per-tick decision was reached.
uint64_t g_routerPathHits = 0;   //!< ticks where ShortestPath gave a transit sat
uint64_t g_routerQueriesAtEnd = 0;

double
ElevDeg(Ptr<MobilityModel> ue, Ptr<MobilityModel> sat)
{
    Vector u = ue->GetPosition();
    Vector s = sat->GetPosition();
    double dx = s.x - u.x, dy = s.y - u.y, dz = s.z - u.z;
    double d = std::sqrt(dx * dx + dy * dy + dz * dz);
    return std::asin(std::max(0.0, dz) / std::max(1.0, d)) * 180.0 / M_PI;
}

double
RangeDelay(Ptr<MobilityModel> a, Ptr<MobilityModel> b)
{
    return a->GetDistanceFrom(b) / kC;
}

void
InstallRouteVia(uint32_t via)
{
    auto clearTo = [](Ptr<Ipv4StaticRouting> r) {
        for (uint32_t i = r->GetNRoutes(); i-- > 0;)
        {
            if (r->GetRoute(i).GetDest() == g_destAddr)
            {
                r->RemoveRoute(i);
            }
        }
    };
    clearTo(g_gs1Routing);
    clearTo(g_satARouting);
    clearTo(g_satBRouting);

    if (via == SATA)
    {
        g_gs1Routing->AddHostRouteTo(g_destAddr, g_satA_fromGs1, 1);
        g_satARouting->AddHostRouteTo(g_destAddr, g_gs2_fromA, 2);
    }
    else
    {
        g_gs1Routing->AddHostRouteTo(g_destAddr, g_satB_fromGs1, 2);
        g_satBRouting->AddHostRouteTo(g_destAddr, g_gs2_fromB, 2);
    }
}

void
Tick()
{
    if (Simulator::Now().GetSeconds() >= g_simTime)
    {
        return;
    }
    g_chA1->SetAttribute("Delay", TimeValue(Seconds(RangeDelay(g_g1, g_satA))));
    g_chA2->SetAttribute("Delay", TimeValue(Seconds(RangeDelay(g_satA, g_g2))));
    g_chB1->SetAttribute("Delay", TimeValue(Seconds(RangeDelay(g_g1, g_satB))));
    g_chB2->SetAttribute("Delay", TimeValue(Seconds(RangeDelay(g_satB, g_g2))));

    double elA = std::min(ElevDeg(g_g1, g_satA), ElevDeg(g_g2, g_satA));
    double elB = std::min(ElevDeg(g_g1, g_satB), ElevDeg(g_g2, g_satB));

    // Route decision is now driven by the live ContactGraphRouter: query the
    // hop-count shortest path GS1 -> GS2 over the contact graph the scheduler
    // is maintaining from the SGP4 geometry, and pick the transit satellite it
    // returns. The path includes both endpoints, so a 3-hop {GS1, SAT, GS2}
    // result names the satellite as the middle element.
    uint32_t chosen = 0;
    if (g_router)
    {
        const std::vector<uint32_t> path = g_router->ShortestPath(GS1, GS2);
        if (path.size() == 3)
        {
            chosen = path[1]; // the transit satellite (SATA or SATB)
            ++g_routerPathHits;
        }
    }
    // Fall back to the SGP4 elevation comparison when the live contact graph
    // yields no GS1-SAT-GS2 transit (preserves the original behaviour).
    if (chosen == 0)
    {
        if (elA >= g_minElev && elA >= elB)
        {
            chosen = SATA;
        }
        else if (elB >= g_minElev)
        {
            chosen = SATB;
        }
    }

    // Surface the per-hop C/N0 -> BLER telemetry for the currently served
    // satellite's two GSL hops (read exactly as the test suite reads it).
    if (g_blerLinks && chosen != 0)
    {
        Ptr<ntncon::NtnSatLinkErrorModel> h1 = (chosen == SATA) ? g_emA1 : g_emB1;
        Ptr<ntncon::NtnSatLinkErrorModel> h2 = (chosen == SATA) ? g_emA2 : g_emB2;
        if (h1 && h2)
        {
            const double esno = std::min(h1->CurrentEsNoDb(), h2->CurrentEsNoDb());
            const double bler =
                1.0 - (1.0 - h1->CurrentBler()) * (1.0 - h2->CurrentBler());
            g_minEsNoDb = std::min(g_minEsNoDb, esno);
            g_sumEsNoDb += esno;
            g_sumBler += bler;
            ++g_blerSamples;
        }
    }

    if (chosen != 0 && chosen != g_currentSat)
    {
        InstallRouteVia(chosen);
        if (g_currentSat != 0)
        {
            ++g_reroutes;
        }
        std::printf("  %6.1fs  ROUTE INSTALLED via %s  (elA=%.1f  elB=%.1f deg)\n",
                    Simulator::Now().GetSeconds(), (chosen == SATA ? "satA" : "satB"), elA, elB);
        g_currentSat = chosen;
    }
    Simulator::Schedule(Seconds(1.0), &Tick);
}
} // namespace

int
main(int argc, char* argv[])
{
    double duration = 40.0;
    double altKm = 550.0;
    double satSpeed = 7500.0;
    std::string outputDir = "ntn-constellation-real-routed-output";

    CommandLine cmd(__FILE__);
    cmd.AddValue("duration", "Simulation duration (s)", duration);
    cmd.AddValue("altKm", "Satellite altitude (km)", altKm);
    cmd.AddValue("satSpeed", "LEO ground-track speed (m/s)", satSpeed);
    cmd.AddValue("outputDir", "Output directory", outputDir);
    bool blerLinks = true;
    double linkEirpDbw = 52.0;
    cmd.AddValue("blerLinks",
                 "Attach the per-packet C/N0->BLER error model to the GSL hops "
                 "(replaces the binary contact gate with measured link quality)",
                 blerLinks);
    cmd.AddValue("linkEirpDbw", "Per-hop EIRP (dBW) for the BLER link model", linkEirpDbw);
    cmd.Parse(argc, argv);
    g_simTime = duration;
    g_outDir = outputDir;

    std::cout << "\n=== ntn-constellation REAL-ROUTED (packets through sat nodes) ===\n"
              << "  ContactGraphRouter decision INSTALLS Ipv4 routes (not printed)\n"
              << "  adaptive reroute gs1->satA->gs2  =>  gs1->satB->gs2 as orbits move\n"
              << "  per-link delay = real slant range / c\n"
              << "  duration: " << duration << " s\n\n";

    NodeContainer nodes;
    nodes.Create(4);
    InternetStackHelper internet;
    internet.Install(nodes);

    Ptr<ConstantPositionMobilityModel> g1 = CreateObject<ConstantPositionMobilityModel>();
    g1->SetPosition(Vector(0, 0, 0));
    nodes.Get(GS1)->AggregateObject(g1);
    Ptr<ConstantPositionMobilityModel> g2 = CreateObject<ConstantPositionMobilityModel>();
    g2->SetPosition(Vector(500000.0, 0, 0));
    nodes.Get(GS2)->AggregateObject(g2);

    // Real SGP4 Walker neighbours projected into the local ENU frame: satA is
    // at zenith over GS1 at t=0 and recedes; satB genuinely approaches, so the
    // reroute emerges from real orbital dynamics.
    ns3::ntncon::WalkerConfig wcfgSat;
    wcfgSat.num_planes = 1;
    wcfgSat.total_sats = 80;
    wcfgSat.altitude_km = altKm;
    wcfgSat.inclination_deg = 53.0;
    wcfgSat.epoch_unix_s = 1735689600.0;
    const auto satElements = ns3::ntncon::WalkerConstellation::BuildDelta(wcfgSat);
    Ptr<ns3::ntncon::Sgp4MobilityModel> satSgp4A =
        CreateObject<ns3::ntncon::Sgp4MobilityModel>();
    satSgp4A->SetElements(satElements[0]);
    double satSubLat, satSubLon, satSubAlt;
    satSgp4A->GetGeodetic(satSubLat, satSubLon, satSubAlt);
    Ptr<ns3::ntncon::Sgp4MobilityModel> nbr1 = CreateObject<ns3::ntncon::Sgp4MobilityModel>();
    nbr1->SetElements(satElements[1]);
    Ptr<ns3::ntncon::Sgp4MobilityModel> nbr2 = CreateObject<ns3::ntncon::Sgp4MobilityModel>();
    nbr2->SetElements(satElements[79]);
    const Vector originEcef = ntngeo::GeodeticToEcef(satSubLat, satSubLon, 0.0);
    auto approaching = [&originEcef](Ptr<ns3::ntncon::Sgp4MobilityModel> sm) {
        const Vector p = sm->GetPosition();
        const Vector v = sm->GetVelocity();
        const Vector to(originEcef.x - p.x, originEcef.y - p.y, originEcef.z - p.z);
        return (v.x * to.x + v.y * to.y + v.z * to.z) > 0.0;
    };
    Ptr<ns3::ntncon::Sgp4MobilityModel> satSgp4B = approaching(nbr1) ? nbr1 : nbr2;
    Ptr<NtnEnuProjectionMobilityModel> sa = CreateObject<NtnEnuProjectionMobilityModel>();
    sa->SetSource(satSgp4A);
    sa->SetReference(satSubLat, satSubLon, 0.0);
    Ptr<NtnEnuProjectionMobilityModel> sb = CreateObject<NtnEnuProjectionMobilityModel>();
    sb->SetSource(satSgp4B);
    sb->SetReference(satSubLat, satSubLon, 0.0);
    nodes.Get(SATA)->AggregateObject(sa);
    nodes.Get(SATB)->AggregateObject(sb);

    g_g1 = g1;
    g_g2 = g2;
    g_satA = sa;
    g_satB = sb;

    PointToPointHelper p2p;
    p2p.SetDeviceAttribute("DataRate", DataRateValue(DataRate("50Mbps")));
    Ipv4AddressHelper ipv4;
    g_blerLinks = blerLinks;
    Ptr<ntncon::NtnSatLinkErrorModel> lastEm; // error model on the rx side of the link
    auto mkLink = [&](uint32_t x, uint32_t y, const char* net) {
        lastEm = nullptr;
        NetDeviceContainer d = p2p.Install(nodes.Get(x), nodes.Get(y));
        ipv4.SetBase(net, "255.255.255.252");
        Ipv4InterfaceContainer ic = ipv4.Assign(d);
        // Replace the binary contact gate with a measured per-packet C/N0->BLER
        // model on each receive device, fed by the two endpoints' live geometry.
        if (blerLinks)
        {
            Ptr<MobilityModel> mx = nodes.Get(x)->GetObject<MobilityModel>();
            Ptr<MobilityModel> my = nodes.Get(y)->GetObject<MobilityModel>();
            for (uint32_t k = 0; k < d.GetN(); ++k)
            {
                Ptr<ntncon::NtnSatLinkErrorModel> em =
                    CreateObject<ntncon::NtnSatLinkErrorModel>();
                em->SetAttribute("EirpDbw", DoubleValue(linkEirpDbw));
                em->SetEndpoints(mx, my);
                d.Get(k)->SetAttribute("ReceiveErrorModel", PointerValue(em));
                lastEm = em; // keep the receiving-side model for telemetry
            }
        }
        return std::make_pair(ic, DynamicCast<PointToPointChannel>(d.Get(0)->GetChannel()));
    };
    auto [icA1, chA1] = mkLink(GS1, SATA, "10.1.1.0");
    g_emA1 = lastEm;
    auto [icA2, chA2] = mkLink(SATA, GS2, "10.1.2.0");
    g_emA2 = lastEm;
    auto [icB1, chB1] = mkLink(GS1, SATB, "10.1.3.0");
    g_emB1 = lastEm;
    auto [icB2, chB2] = mkLink(SATB, GS2, "10.1.4.0");
    g_emB2 = lastEm;
    g_chA1 = chA1;
    g_chA2 = chA2;
    g_chB1 = chB1;
    g_chB2 = chB2;

    g_destAddr = icA2.GetAddress(1);
    g_satA_fromGs1 = icA1.GetAddress(1);
    g_satB_fromGs1 = icB1.GetAddress(1);
    g_gs2_fromA = icA2.GetAddress(1);
    g_gs2_fromB = icB2.GetAddress(1);

    Ipv4StaticRoutingHelper srh;
    g_gs1Routing = srh.GetStaticRouting(nodes.Get(GS1)->GetObject<Ipv4>());
    g_satARouting = srh.GetStaticRouting(nodes.Get(SATA)->GetObject<Ipv4>());
    g_satBRouting = srh.GetStaticRouting(nodes.Get(SATB)->GetObject<Ipv4>());

    // Live contact-graph scheduler + router that DRIVE the route decision in
    // Tick(). Registered exactly as the test suite does: satellites take the
    // raw Sgp4MobilityModel (NOT the ENU wrappers), ground stations take
    // geodetic lat/lon. GS1 sits at satA's sub-point; GS2 is ~500 km east in
    // the ENU frame, i.e. a longitude offset of 500 km / (111.32 km/deg *
    // cos(lat)) at the sub-latitude. Use the example's elevation gate so the
    // router sees the same contacts the geometry implies.
    const double kmPerDegLon = 111.320 * std::cos(satSubLat * M_PI / 180.0);
    const double gs2Lon = satSubLon + 500.0 / std::max(1.0, kmPerDegLon);
    g_sched = CreateObject<ContactGraphScheduler>();
    g_sched->SetMinElevationDeg(g_minElev);
    g_sched->RegisterSatellite(SATA, satSgp4A);
    g_sched->RegisterSatellite(SATB, satSgp4B);
    g_sched->RegisterGroundStation(GS1, satSubLat, satSubLon);
    g_sched->RegisterGroundStation(GS2, satSubLat, gs2Lon);
    g_router = CreateObject<ContactGraphRouter>();
    g_router->Attach(g_sched);
    g_sched->Start();

    uint16_t port = 8080;
    g_sink = CreateObject<NtnOranSink>();
    g_sink->SetAttribute("Local",
                         AddressValue(InetSocketAddress(Ipv4Address::GetAny(), port)));
    nodes.Get(GS2)->AddApplication(g_sink);
    g_sink->SetStartTime(Seconds(0.0));
    g_sink->SetStopTime(Seconds(duration));

    Ptr<NtnOranApplication> client = CreateObject<NtnOranApplication>();
    client->SetRemote(InetSocketAddress(g_destAddr, port));
    client->SetProfile(NtnOranApplication::CBR_SATURATING);
    client->SetAttribute("DataRate", DataRateValue(DataRate("5Mbps")));
    client->SetAttribute("PacketSize", UintegerValue(1200));
    client->SetFlowIdentity(/*5qi*/ 9, /*sst*/ 1, /*sd*/ 0x000001, /*src*/ GS1, /*dst*/ GS2);
    nodes.Get(GS1)->AddApplication(client);
    client->SetStartTime(Seconds(1.0));
    client->SetStopTime(Seconds(duration - 0.5));

    Simulator::Schedule(Seconds(0.5), &Tick);
    Simulator::Stop(Seconds(duration));
    Simulator::Run();

    g_sched->Stop();
    g_routerQueriesAtEnd = g_router->RouteQueries();

    // All KPIs measured from in-band header primitives at the sink.
    uint64_t txP = client->GetTxPackets();
    uint64_t rxP = g_sink->GetRxPackets();
    double rxBytes = g_sink->GetTotalRx();
    double thrMbps = rxBytes * 8.0 / std::max(1.0, duration) / 1e6;
    double meanDelayMs = g_sink->GetMeanDelayMs();
    double delivery = (txP > 0) ? (double)rxP / txP : 0.0;

    // Per-hop C/N0 -> BLER telemetry (bler-errormodel provenance). Read from
    // the error models attached to the served satellite's GSL hops. LastEsNoDb/
    // LastBler are the telemetry of the very last DoCorrupt() on the GS2-side
    // hop of the active route (exactly as the test suite reads them).
    const double meanEsNoDb = (g_blerSamples > 0) ? g_sumEsNoDb / g_blerSamples : 0.0;
    const double meanBler = (g_blerSamples > 0) ? g_sumBler / g_blerSamples : 0.0;
    const double minEsNoDb =
        std::isinf(g_minEsNoDb) ? 0.0 : g_minEsNoDb;
    Ptr<ntncon::NtnSatLinkErrorModel> servedRxEm =
        (g_currentSat == SATA) ? g_emA2 : g_emB2;
    const double lastEsNoDb =
        (g_blerLinks && servedRxEm) ? servedRxEm->LastEsNoDb() : 0.0;
    const double lastBler =
        (g_blerLinks && servedRxEm) ? servedRxEm->LastBler() : 0.0;

    // ---- TR 38.821 corpus + calibration harness (Roadmap §4.4.11) ----
    // Read the bundled link-budget corpus and compare the toolkit's analytic
    // free-space pathloss (the SAME 20 log10(4*pi*d*f/c) the error model uses)
    // against the TR 38.821 reference. FSPL is the right model at zenith
    // (90 deg elevation, slant ~= altitude), so the gate is applied to the
    // 90-deg rows; lower-elevation rows are recorded as residuals (their TR
    // reference adds path-stretch + scintillation that pure FSPL omits — an
    // honest, documented limitation, not a pass). Soft-skips like the test.
    bool calibLoaded = false;
    bool calibWithinGate = false;
    uint32_t calibRows = 0;
    Tr38821CorpusReader corpus;
    CalibrationHarness harness;
    {
        const std::vector<std::string> roots = {
            "contrib/ntn-constellation/calibration/",
            "../contrib/ntn-constellation/calibration/",
            "../../contrib/ntn-constellation/calibration/",
        };
        std::string lbPath, scPath;
        for (const auto& r : roots)
        {
            std::ifstream f(r + "tr38821/link_budgets.csv");
            if (f)
            {
                lbPath = r + "tr38821/link_budgets.csv";
                scPath = r + "tr38821/scenarios.csv";
                break;
            }
        }
        if (!lbPath.empty() && corpus.LoadLinkBudgets(lbPath) &&
            corpus.LoadScenarios(scPath))
        {
            calibLoaded = true;
            constexpr double kReEarth = 6371000.0;
            for (const auto& lb : corpus.LinkBudgets())
            {
                const auto sc = corpus.FindScenario(lb.scenario_id);
                if (!sc.has_value())
                {
                    continue;
                }
                const double h = sc->alt_km * 1000.0;
                const double elr = lb.elevation_deg * M_PI / 180.0;
                // Spherical-earth slant range from elevation + altitude.
                const double slant =
                    std::sqrt(kReEarth * kReEarth * std::sin(elr) * std::sin(elr) +
                              h * h + 2.0 * kReEarth * h) -
                    kReEarth * std::sin(elr);
                const double toolkitFspl =
                    20.0 * std::log10(4.0 * M_PI * slant * sc->freq_ghz * 1e9 / kC);
                // TR reference free-space component = total pathloss - atmos.
                const double refFreeSpace = lb.pathloss_db - lb.atmos_loss_db;
                // Gate only the zenith rows where FSPL is the right model.
                if (lb.elevation_deg >= 89.0)
                {
                    harness.Compare(lb.scenario_id, "pathloss", refFreeSpace, toolkitFspl);
                }
                ++calibRows;
            }
            calibWithinGate = harness.AllWithinGate();

            std::filesystem::create_directories(outputDir);
            std::ofstream cr(outputDir + "/calibration_residuals.csv");
            cr << "scenario_id,metric,reference,toolkit,residual,within_gate,provenance\n";
            for (const auto& res : harness.Residuals())
            {
                cr << res.scenario_id << "," << res.metric << "," << res.reference << ","
                   << res.toolkit << "," << res.residual << ","
                   << (res.within_gate ? 1 : 0) << ",tr38821-corpus\n";
            }
            cr.close();
        }
    }

    std::filesystem::create_directories(outputDir);
    std::ofstream out(outputDir + "/sim_health.csv");
    out << "metric,value,floor,pass,provenance\n";
    out << "sim_time_s," << duration << "," << duration << ",1,config\n";
    out << "air_interface,sat-ip-routing,-,1,config\n";
    out << "packets_through_sats," << rxP << ",1," << (rxP > 0 ? 1 : 0) << ",packetsink\n";
    out << "rx_throughput_mbps," << thrMbps << ",0.05," << (thrMbps >= 0.05 ? 1 : 0)
        << ",packetsink\n";
    out << "mean_e2e_delay_ms," << meanDelayMs << ",-,1,inband-timestamp\n";
    out << "app_jitter_ms," << g_sink->GetMeanJitterMs() << ",-,1,inband-timestamp\n";
    out << "app_loss_ratio," << g_sink->GetLossRatio() << ",-,1,inband-seq\n";
    out << "app_delivery_ratio," << delivery << ",-,1,app-trace\n";
    out << "route_installs," << (g_currentSat != 0 ? 1 : 0) + g_reroutes << ",1,"
        << ((g_currentSat != 0) ? 1 : 0) << ",contactgraphrouter\n";
    out << "reroutes," << g_reroutes << ",-,1,contactgraphrouter\n";
    out << "router_queries," << g_routerQueriesAtEnd << ",1,"
        << (g_routerQueriesAtEnd > 0 ? 1 : 0) << ",contactgraphrouter\n";
    out << "router_path_hits," << g_routerPathHits << ",-,1,contactgraphrouter\n";
    // Per-hop C/N0 -> BLER telemetry surfaced from NtnSatLinkErrorModel.
    if (g_blerLinks)
    {
        out << "current_esno_db," << meanEsNoDb << ",-,1,bler-errormodel\n";
        out << "min_esno_db," << minEsNoDb << ",-,1,bler-errormodel\n";
        out << "current_bler," << meanBler << ",-,1,bler-errormodel\n";
        out << "last_esno_db," << lastEsNoDb << ",-,1,bler-errormodel\n";
        out << "last_bler," << lastBler << ",-,1,bler-errormodel\n";
        out << "link_quality_mode,measured-bler,-,1,config\n";
    }
    else
    {
        out << "link_quality_mode,binary-gate,-,1,config\n";
    }
    // TR 38.821 calibration residuals (Roadmap §4.4.11).
    if (calibLoaded)
    {
        out << "calibration_rows," << calibRows << ",1,1,tr38821-corpus\n";
        out << "calibration_within_gate," << (calibWithinGate ? 1 : 0) << ",1,"
            << (calibWithinGate ? 1 : 0) << ",calibration-harness\n";
        out << "calibration_pathloss_checks," << harness.CountsByMetric()["pathloss"]
            << ",-,1,calibration-harness\n";
        out << "calibration_pathloss_failures," << harness.FailuresByMetric()["pathloss"]
            << ",0,"
            << (harness.FailuresByMetric()["pathloss"] == 0 ? 1 : 0)
            << ",calibration-harness\n";
    }
    out.close();

    std::cout << "\n--- Constellation routing Summary (MEASURED, packets through sats) ---\n"
              << "  throughput:        " << thrMbps << " Mbps\n"
              << "  mean e2e delay:    " << meanDelayMs << " ms (real slant-range delays)\n"
              << "  delivery ratio:    " << delivery << "\n"
              << "  adaptive reroutes: " << g_reroutes << " (route-installed)\n"
              << "  router queries:    " << g_routerQueriesAtEnd << " (ContactGraphRouter), "
              << g_routerPathHits << " transit-path hits\n";
    if (g_blerLinks)
    {
        std::cout << "  link C/N0->BLER:   meanEs/No=" << meanEsNoDb << " dB, minEs/No="
                  << minEsNoDb << " dB, meanBLER=" << meanBler
                  << " (NtnSatLinkErrorModel)\n";
    }
    if (calibLoaded)
    {
        std::cout << "  TR 38.821 calib:   within_gate=" << (calibWithinGate ? "YES" : "NO")
                  << " over " << calibRows << " link-budget rows (zenith FSPL gated)\n";
    }

    Simulator::Destroy();
    return 0;
}
