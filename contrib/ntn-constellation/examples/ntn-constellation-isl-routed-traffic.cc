/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026  Muhammad Uzair
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * ntn-constellation-isl-routed-traffic — real ISL-routed packet forwarding
 * GS1 -> satA -> [ISL] -> satB -> GS2. Real UDP packets are forwarded THROUGH
 * the two satellite nodes (real Ipv4 forwarding), each hop's propagation delay
 * is the real slant range / c (including the inter-satellite link range), and a
 * hop carries traffic only while it is in contact — a GSL hop above the minimum
 * elevation and the ISL hop within the range cap. The contact gate is driven by
 * the live orbital geometry, NOT a closed-form SINR/sigmoid: when a hop falls
 * out of contact its link drops, and delivery recovers when contact resumes.
 * Delivery / delay / jitter / loss are MEASURED end-to-end by NtnOranSink
 * from the in-band NtnOranPayloadHeader (WS1 application suite).
 *
 * Usage:
 *   ./ns3 run "ntn-constellation-isl-routed-traffic --duration=40"
 */

#include "ns3/applications-module.h"
#include "ns3/ntn-tr38811-mobility-model.h"
#include "ns3/sgp4-mobility-model.h"
#include "ns3/walker-constellation.h"
#include "ns3/core-module.h"
#include "ns3/error-model.h"
#include "ns3/ntn-oran-application.h"
#include "ns3/ntn-oran-sink.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/point-to-point-module.h"

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iostream>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("NtnConstellationIslRoutedTraffic");

namespace
{
constexpr double kC = 299792458.0;
constexpr uint32_t GS1 = 0, SATA = 1, SATB = 2, GS2 = 3;

Ptr<MobilityModel> g_g1, g_satA, g_satB, g_g2;
Ptr<PointToPointChannel> g_chGsl1, g_chIsl, g_chGsl2;
Ptr<RateErrorModel> g_emGsl1, g_emIsl, g_emGsl2; // binary geometry contact gates
double g_minElev = 10.0;
double g_islRangeCapKm = 5000.0;
double g_simTime = 40.0;
uint64_t g_contactDrops = 0;

double
ElevDeg(Ptr<MobilityModel> gs, Ptr<MobilityModel> sat)
{
    // All nodes are ECEF: elevation about the WGS-84 normal at the station.
    // The former flat-frame asin(dz/d) ignored ~9 deg of Earth curvature at
    // GS2, 1000 km away, and overstated its elevation by several degrees.
    return ntngeo::GeodeticElevationDeg(gs->GetPosition(), sat->GetPosition());
}

double
RangeDelay(Ptr<MobilityModel> a, Ptr<MobilityModel> b)
{
    return a->GetDistanceFrom(b) / kC;
}

void
Tick()
{
    if (Simulator::Now().GetSeconds() >= g_simTime)
    {
        return;
    }
    g_chGsl1->SetAttribute("Delay", TimeValue(Seconds(RangeDelay(g_g1, g_satA))));
    g_chIsl->SetAttribute("Delay", TimeValue(Seconds(RangeDelay(g_satA, g_satB))));
    g_chGsl2->SetAttribute("Delay", TimeValue(Seconds(RangeDelay(g_satB, g_g2))));

    // Geometry contact gates (NOT a fabricated SINR): a hop forwards only while
    // in contact. error rate 0.0 = link up, 1.0 = out of contact (dropped).
    const bool gsl1 = ElevDeg(g_g1, g_satA) >= g_minElev;
    const bool isl = (g_satA->GetDistanceFrom(g_satB) / 1e3) <= g_islRangeCapKm;
    const bool gsl2 = ElevDeg(g_g2, g_satB) >= g_minElev;
    g_emGsl1->SetRate(gsl1 ? 0.0 : 1.0);
    g_emIsl->SetRate(isl ? 0.0 : 1.0);
    g_emGsl2->SetRate(gsl2 ? 0.0 : 1.0);
    if (!(gsl1 && isl && gsl2))
    {
        ++g_contactDrops;
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
    std::string outputDir = "ntn-constellation-isl-routed-output";

    CommandLine cmd(__FILE__);
    cmd.AddValue("duration", "Simulation duration (s)", duration);
    cmd.AddValue("altKm", "Satellite altitude (km)", altKm);
    cmd.AddValue("satSpeed",
                 "Ignored, kept for CLI compatibility: the orbital speed follows from altKm "
                 "(the measured value is printed and written to sim_health.csv)",
                 satSpeed);
    cmd.AddValue("minElev", "Min GSL elevation for contact (deg)", g_minElev);
    cmd.AddValue("islRangeCapKm", "Max ISL range for contact (km)", g_islRangeCapKm);
    cmd.AddValue("outputDir", "Output directory", outputDir);
    cmd.Parse(argc, argv);
    g_simTime = duration;

    std::cout << "\n=== ntn-constellation ISL-ROUTED (real packets through sat nodes) ===\n"
              << "  chain GS1 -> satA -> [ISL] -> satB -> GS2, real Ipv4 forwarding\n"
              << "  per-hop delay = real slant/ISL range / c; contact = live geometry\n"
              << "  duration: " << duration << " s\n\n";

    NodeContainer nodes;
    nodes.Create(4);
    InternetStackHelper internet;
    internet.Install(nodes);

    // satA and satB are in-plane neighbours of one shell, so the ISL range
    // stays within cap while each GSL's elevation evolves with the real orbit.
    // Walker neighbours propagated by Vallado SGP4 (SetElementsSgp4: synthesized TLE)
    // in ECEF: satA is at zenith over GS1 at t=0 and recedes; satB is the
    // neighbour approaching GS1's sub-point.
    ns3::ntncon::WalkerConfig wcfgSat;
    wcfgSat.num_planes = 1;
    wcfgSat.total_sats = 80;
    wcfgSat.altitude_km = altKm;
    wcfgSat.inclination_deg = 53.0;
    wcfgSat.epoch_unix_s = 1735689600.0;
    const auto satElements = ns3::ntncon::WalkerConstellation::BuildDelta(wcfgSat);
    Ptr<ns3::ntncon::Sgp4MobilityModel> satSgp4A =
        CreateObject<ns3::ntncon::Sgp4MobilityModel>();
    satSgp4A->SetElementsSgp4(satElements[0]);
    double satSubLat, satSubLon, satSubAlt;
    satSgp4A->GetGeodetic(satSubLat, satSubLon, satSubAlt);
    Ptr<ns3::ntncon::Sgp4MobilityModel> nbr1 = CreateObject<ns3::ntncon::Sgp4MobilityModel>();
    nbr1->SetElementsSgp4(satElements[1]);
    Ptr<ns3::ntncon::Sgp4MobilityModel> nbr2 = CreateObject<ns3::ntncon::Sgp4MobilityModel>();
    nbr2->SetElementsSgp4(satElements[79]);
    const Vector originEcef = ntngeo::GeodeticToEcef(satSubLat, satSubLon, 0.0);
    auto approaching = [&originEcef](Ptr<ns3::ntncon::Sgp4MobilityModel> sm) {
        const Vector p = sm->GetPosition();
        const Vector v = sm->GetVelocity();
        const Vector to(originEcef.x - p.x, originEcef.y - p.y, originEcef.z - p.z);
        return (v.x * to.x + v.y * to.y + v.z * to.z) > 0.0;
    };
    Ptr<ns3::ntncon::Sgp4MobilityModel> satSgp4B = approaching(nbr1) ? nbr1 : nbr2;
    // Ground stations on the WGS-84 ellipsoid in the same ECEF frame as the
    // satellites. They used to sit at (0,0,0) and (1000 km,0,0) of a flat
    // tangent plane, which put GS2 ~78 km above the real ground and gave both
    // stations the same "up". GS1 is satA's t=0 sub-point; GS2 is 1000 km east.
    const double kmPerDegLon = 111.320 * std::cos(satSubLat * M_PI / 180.0);
    const double gs2Lon = satSubLon + 1000.0 / std::max(1.0, kmPerDegLon);
    Ptr<ConstantPositionMobilityModel> g1 = CreateObject<ConstantPositionMobilityModel>();
    g1->SetPosition(ntngeo::GeodeticToEcef(satSubLat, satSubLon, 0.0));
    nodes.Get(GS1)->AggregateObject(g1);
    Ptr<ConstantPositionMobilityModel> g2 = CreateObject<ConstantPositionMobilityModel>();
    g2->SetPosition(ntngeo::GeodeticToEcef(satSubLat, gs2Lon, 0.0));
    nodes.Get(GS2)->AggregateObject(g2);
    Ptr<MobilityModel> sa = satSgp4A;
    Ptr<MobilityModel> sb = satSgp4B;
    nodes.Get(SATA)->AggregateObject(satSgp4A);
    nodes.Get(SATB)->AggregateObject(satSgp4B);
    // The orbit fixes the speed; satSpeed is not an input.
    const double satSpeedMps = satSgp4A->GetVelocity().GetLength();
    std::cout << "  satellite speed (ECEF, SGP4 model at altKm=" << altKm
              << "): " << satSpeedMps << " m/s";
    if (std::abs(satSpeed - 7500.0) > 1e-9)
    {
        std::cout << "  [--satSpeed=" << satSpeed << " ignored]";
    }
    std::cout << "\n\n";

    g_g1 = g1;
    g_g2 = g2;
    g_satA = sa;
    g_satB = sb;

    // Real link chain: GS1-satA (GSL), satA-satB (ISL), satB-GS2 (GSL).
    PointToPointHelper p2p;
    p2p.SetDeviceAttribute("DataRate", DataRateValue(DataRate("50Mbps")));
    Ipv4AddressHelper ipv4;
    auto mkLink = [&](uint32_t x, uint32_t y, const char* net, Ptr<RateErrorModel>& emOut,
                      Ptr<PointToPointChannel>& chOut) {
        NetDeviceContainer d = p2p.Install(nodes.Get(x), nodes.Get(y));
        emOut = CreateObject<RateErrorModel>();
        emOut->SetUnit(RateErrorModel::ERROR_UNIT_PACKET);
        emOut->SetRate(0.0);
        d.Get(1)->SetAttribute("ReceiveErrorModel", PointerValue(emOut));
        chOut = DynamicCast<PointToPointChannel>(d.Get(0)->GetChannel());
        ipv4.SetBase(net, "255.255.255.252");
        return ipv4.Assign(d);
    };
    Ipv4InterfaceContainer icGsl1 = mkLink(GS1, SATA, "10.1.1.0", g_emGsl1, g_chGsl1);
    (void)icGsl1;
    Ipv4InterfaceContainer icIsl = mkLink(SATA, SATB, "10.1.2.0", g_emIsl, g_chIsl);
    (void)icIsl;
    Ipv4InterfaceContainer icGsl2 = mkLink(SATB, GS2, "10.1.3.0", g_emGsl2, g_chGsl2);

    // Real end-to-end routing: packets are forwarded through satA and satB.
    Ipv4GlobalRoutingHelper::PopulateRoutingTables();

    uint16_t port = 8080;
    Ptr<NtnOranSink> ps = CreateObject<NtnOranSink>();
    ps->SetAttribute("Local", AddressValue(InetSocketAddress(Ipv4Address::GetAny(), port)));
    nodes.Get(GS2)->AddApplication(ps);
    ps->SetStartTime(Seconds(0.0));
    ps->SetStopTime(Seconds(duration));

    Ptr<NtnOranApplication> client = CreateObject<NtnOranApplication>();
    client->SetRemote(InetSocketAddress(icGsl2.GetAddress(1), port));
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

    // All KPIs measured from in-band header primitives at the sink.
    uint64_t txP = client->GetTxPackets();
    uint64_t rxP = ps->GetRxPackets();
    double rxBytes = ps->GetTotalRx();
    double thrMbps = rxBytes * 8.0 / std::max(1.0, duration) / 1e6;
    double meanDelayMs = ps->GetMeanDelayMs();
    double delivery = (txP > 0) ? (double)rxP / txP : 0.0;

    std::filesystem::create_directories(outputDir);
    std::ofstream out(outputDir + "/sim_health.csv");
    out << "metric,value,floor,pass,provenance\n";
    out << "sim_time_s," << duration << "," << duration << ",1,config\n";
    out << "air_interface,sat-ip-isl-routing,-,1,config\n";
    out << "sat_speed_ecef_mps," << satSpeedMps << ",-,-,sgp4-propagated\n";
    out << "packets_through_sats," << rxP << ",1," << (rxP > 0 ? 1 : 0) << ",packetsink\n";
    out << "rx_throughput_mbps," << thrMbps << ",0.05," << (thrMbps >= 0.05 ? 1 : 0)
        << ",packetsink\n";
    out << "mean_e2e_delay_ms," << meanDelayMs << ",-,1,inband-timestamp\n";
    out << "app_jitter_ms," << ps->GetMeanJitterMs() << ",-,1,inband-timestamp\n";
    out << "app_loss_ratio," << ps->GetLossRatio() << ",-,1,inband-seq\n";
    out << "app_delivery_ratio," << delivery << ",-,1,app-trace\n";
    out << "contact_drop_ticks," << g_contactDrops << ",-,1,router\n";
    out.close();

    std::cout << "\n--- ISL routing Summary (MEASURED, packets through sats) ---\n"
              << "  throughput:        " << thrMbps << " Mbps\n"
              << "  mean e2e delay:    " << meanDelayMs << " ms (real slant + ISL delays)\n"
              << "  delivery ratio:    " << delivery << "\n"
              << "  out-of-contact ticks: " << g_contactDrops << "\n";

    Simulator::Destroy();
    return 0;
}
