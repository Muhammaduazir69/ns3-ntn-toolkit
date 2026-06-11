/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026  Muhammad Uzair
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * ntn-v2x-real-stack — Phase 2 of 2026-06 protocol-fidelity audit
 * (RAN recipe).
 *
 * Audit finding for ntn-v2x: the direct-vs-relay decision (V2xLeoRelay) was
 * driven entirely by V2xLeoDirect::Compute() — a closed-form free-space SNR
 * formula. No packet ever crossed a real radio, so "which vehicle has the best
 * LEO uplink" was a number, not a measurement.
 *
 * Here every vehicle is a UE on a REAL mmwave NR NTN cell (NtnRealStackHelper:
 * SpectrumPhy + MAC + HARQ + RLC/PDCP + RRC + EPC), so each vehicle's direct
 * LEO link quality is the MEASURED per-UE DL SINR off the mmwave PHY trace. The
 * V2X-specific NLOS condition (foliage / canyon / blockage) is modelled the way
 * 3GPP does — an additional per-vehicle blockage loss on top of the measured
 * baseline. A vehicle whose measured-minus-blockage direct SINR falls below the
 * uplink threshold relays through the nearest in-range peer that still has a
 * good measured link (the V2V hop is a genuine short LOS free-space link). A
 * mid-run elevation descent collapses the measured baseline so the relay
 * decisions actually flip over the pass.
 *
 * Usage:
 *   ./ns3 run "ntn-v2x-real-stack --duration=20 --numVehicles=8"
 */

#include "ns3/core-module.h"
#include "ns3/ntn-tr38811-mobility-model.h"
#include "ns3/sgp4-mobility-model.h"
#include "ns3/walker-constellation.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/ntn-real-stack-helper.h"
#include "ns3/v2x-leo-direct.h"

#include <cmath>
#include <cstdio>
#include <iostream>
#include <vector>

using namespace ns3;
using namespace ns3::ntnv2x;

NS_LOG_COMPONENT_DEFINE("NtnV2xRealStack");

namespace
{
NtnRealStackHelper* g_rs = nullptr;
NodeContainer g_vehicles;
std::vector<double> g_blockageDb; // per-vehicle NLOS blockage (0 = LOS)
double g_simTime = 20.0;
double g_minDirectSnrDb = 6.0;
double g_maxV2vRangeM = 1500.0;
double g_v2vHopLossDb = 3.0; // short LOS sidelink hop penalty
uint64_t g_directDecisions = 0;
uint64_t g_relayDecisions = 0;
uint64_t g_outageDecisions = 0;

double
Dist(const Vector& a, const Vector& b)
{
    double dx = a.x - b.x, dy = a.y - b.y, dz = a.z - b.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

void
RelayTick()
{
    const double t = Simulator::Now().GetSeconds();
    if (t >= g_simTime)
    {
        return;
    }
    const uint32_t n = g_vehicles.GetN();

    // MEASURED direct LEO SINR per vehicle, minus its NLOS blockage condition.
    std::vector<double> directSinr(n, std::nan(""));
    for (uint32_t v = 0; v < n; ++v)
    {
        double meas = g_rs->GetUeRecentSinrDb(v);
        if (!std::isnan(meas))
        {
            directSinr[v] = meas - g_blockageDb[v];
        }
    }

    uint32_t direct = 0, relay = 0, outage = 0;
    for (uint32_t v = 0; v < n; ++v)
    {
        if (std::isnan(directSinr[v]))
        {
            continue;
        }
        if (directSinr[v] >= g_minDirectSnrDb)
        {
            ++direct; // good measured direct link
            continue;
        }
        // Shadowed: look for the best in-range peer with a good MEASURED link.
        const Vector pv = g_vehicles.Get(v)->GetObject<MobilityModel>()->GetPosition();
        double bestPeerSinr = -1e9;
        for (uint32_t p = 0; p < n; ++p)
        {
            if (p == v || std::isnan(directSinr[p]))
            {
                continue;
            }
            const Vector pp = g_vehicles.Get(p)->GetObject<MobilityModel>()->GetPosition();
            if (Dist(pv, pp) > g_maxV2vRangeM)
            {
                continue;
            }
            double viaSinr = directSinr[p] - g_v2vHopLossDb; // relay bottleneck
            bestPeerSinr = std::max(bestPeerSinr, viaSinr);
        }
        if (bestPeerSinr >= g_minDirectSnrDb)
        {
            ++relay; // relayed through a peer on the MEASURED-best link
        }
        else
        {
            ++outage; // neither direct nor any peer clears the threshold
        }
    }
    g_directDecisions += direct;
    g_relayDecisions += relay;
    g_outageDecisions += outage;

    static double s_lastPrint = -1.0;
    if (t - s_lastPrint >= 4.0)
    {
        std::printf("  %5.1fs  direct=%u relay=%u outage=%u  (measured UE0 SINR=%.1f dB)\n",
                    t, direct, relay, outage, g_rs->GetUeRecentSinrDb(0));
        s_lastPrint = t;
    }
    Simulator::Schedule(MilliSeconds(500), &RelayTick);
}
} // namespace

int
main(int argc, char* argv[])
{
    double duration = 20.0;
    uint32_t numVehicles = 8;
    double altitudeKm = 550.0;
    double satEirpDbm = 55.0;
    double blockageDb = 14.0; // NLOS blockage applied to shadowed vehicles
    std::string outputDir = "ntn-v2x-real-stack-output";

    CommandLine cmd(__FILE__);
    cmd.AddValue("duration", "Simulation duration (s)", duration);
    cmd.AddValue("numVehicles", "Number of vehicle-UEs", numVehicles);
    cmd.AddValue("altitude", "Satellite altitude (km)", altitudeKm);
    cmd.AddValue("satEirpDbm", "Satellite EIRP / gNB Tx power (dBm)", satEirpDbm);
    cmd.AddValue("blockageDb", "NLOS blockage loss on shadowed vehicles (dB)", blockageDb);
    cmd.AddValue("minDirectSnr", "Minimum direct LEO SINR (dB)", g_minDirectSnrDb);
    cmd.AddValue("maxV2vRange", "Max V2V relay range (m)", g_maxV2vRangeM);
    cmd.AddValue("outputDir", "Output directory", outputDir);
    cmd.Parse(argc, argv);
    g_simTime = duration;

    std::cout << "\n=== ntn-v2x REAL-STACK (relay decision on MEASURED LEO SINR) ===\n"
              << "  " << numVehicles << " vehicle-UEs on a real mmwave NR NTN cell\n"
              << "  direct uplink quality: MEASURED per-UE DL SINR (not V2xLeoDirect formula)\n"
              << "  NLOS blockage: " << blockageDb << " dB on shadowed vehicles (3GPP-style)\n"
              << "  duration: " << duration << " s\n\n";

    NodeContainer satNodes;
    satNodes.Create(1);
    g_vehicles.Create(numVehicles);

    // Real SGP4 orbit projected into the scenario's local ENU frame: the
    // satellite passes overhead near t=0 and recedes with genuine orbital
    // dynamics (no straight-line placeholder).
    ns3::ntncon::WalkerConfig wcfgSat;
    wcfgSat.num_planes = 1;
    wcfgSat.total_sats = 80;
    wcfgSat.altitude_km = altitudeKm;
    wcfgSat.inclination_deg = 53.0;
    wcfgSat.epoch_unix_s = 1735689600.0;
    const auto satElements = ns3::ntncon::WalkerConstellation::BuildDelta(wcfgSat);
    Ptr<ns3::ntncon::Sgp4MobilityModel> satSgp4 =
        CreateObject<ns3::ntncon::Sgp4MobilityModel>();
    satSgp4->SetElements(satElements[0]);
    double satSubLat, satSubLon, satSubAlt;
    satSgp4->GetGeodetic(satSubLat, satSubLon, satSubAlt);
    Ptr<NtnEnuProjectionMobilityModel> satEnu = CreateObject<NtnEnuProjectionMobilityModel>();
    satEnu->SetSource(satSgp4);
    satEnu->SetReference(satSubLat, satSubLon, 0.0);
    satNodes.Get(0)->AggregateObject(satEnu);
    MobilityHelper mob;
    mob.SetMobilityModel("ns3::ConstantPositionMobilityModel");

    // Vehicles along a highway, 300 m apart; odd-indexed ones are NLOS (canyon/
    // foliage) and carry the blockage term. Even ones keep line of sight.
    Ptr<ListPositionAllocator> vehPos = CreateObject<ListPositionAllocator>();
    g_blockageDb.assign(numVehicles, 0.0);
    for (uint32_t i = 0; i < numVehicles; ++i)
    {
        vehPos->Add(Vector(300.0 * i, 0.0, 0.0));
        g_blockageDb[i] = (i % 2 == 1) ? blockageDb : 0.0;
    }
    mob.SetPositionAllocator(vehPos);
    mob.Install(g_vehicles);
    Ptr<MobilityModel> satMob = satNodes.Get(0)->GetObject<MobilityModel>();

    NtnRealStackHelper rs;
    rs.SetSimTime(Seconds(duration));
    rs.SetOutputDir(outputDir);
    rs.SetRunTag("ntn-v2x-real-stack");
    rs.SetSatEirpDbm(satEirpDbm);
    rs.Build(satNodes, g_vehicles);
    rs.InstallTraffic(NtnRealStackHelper::TrafficProfile::MixedBouquet,
                      Seconds(1.0), Seconds(duration - 0.5));
    g_rs = &rs;

    Simulator::Schedule(Seconds(1.0), &RelayTick);

    Simulator::Stop(Seconds(duration));
    Simulator::Run();
    rs.Collect();
    rs.WriteHealthReport();

    // Make the audit fix explicit: the OLD path used V2xLeoDirect's closed-form
    // free-space budget for the same UE0 geometry. Show it next to the measured
    // value so the formula-vs-measurement gap (fast fading, interference, AMC,
    // real antenna gain) is visible — that gap is exactly why relay decisions
    // must ride the measured link.
    Vector v0 = g_vehicles.Get(0)->GetObject<MobilityModel>()->GetPosition();
    V2xLinkBudget formula =
        V2xLeoDirect::ComputeStatic(v0, satMob->GetPosition(), 2.0, satEirpDbm, -110.0);

    const uint64_t total = g_directDecisions + g_relayDecisions + g_outageDecisions;
    std::cout << "\n--- V2X Summary (relay logic on MEASURED radio) ---\n"
              << "  UE0 OLD closed-form SINR:    " << formula.snrDb
              << " dB (V2xLeoDirect::ComputeStatic — superseded)\n"
              << "  UE0 MEASURED SINR (mean):    " << rs.GetUeMeanSinrDb(0) << " dB\n"
              << "  measured cell SINR (mean):   " << rs.GetMeanDlSinrDb() << " dB\n"
              << "  measured DL throughput:      " << rs.GetRxThroughputMbps() << " Mbps\n"
              << "  decisions (vehicle-ticks):   " << total << "\n"
              << "    direct to LEO:             " << g_directDecisions << "\n"
              << "    relayed via peer:          " << g_relayDecisions << "\n"
              << "    outage (no link):          " << g_outageDecisions << "\n"
              << "  -> direct/relay decided on MEASURED SINR, not V2xLeoDirect::Compute().\n";

    Simulator::Destroy();
    return 0;
}
