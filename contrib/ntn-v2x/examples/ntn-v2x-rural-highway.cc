/*
 * SPDX-License-Identifier: GPL-2.0-only
 * Copyright (c) 2026 Muhammad Uzair (ns3-ntn-toolkit, W7)
 *
 * ntn-v2x-rural-highway — a fleet of vehicles on a rural highway pulling
 * connectivity from a single LEO satellite. The vehicle mobility is replayed
 * from a SUMO FCD trace (SumoTraciBridge, trace-replay mode, so CI needs no live
 * SUMO). The LEO link quality is MEASURED off a real mmwave NR cell
 * (NtnRealStackHelper) over representative highway terminals; each vehicle's
 * direct uplink SINR is that measured baseline minus its 3GPP-style NLOS
 * blockage, and a vehicle that cannot clear the threshold relays through the
 * nearest in-range peer that can. The direct/relay/orphan split is therefore
 * driven by the MEASURED radio, not V2xLeoDirect's closed-form budget.
 *
 * Quick test:  --vehicles=40 --simTime=30
 */
#include "ns3/command-line.h"
#include "ns3/ntn-tr38811-mobility-model.h"
#include "ns3/sgp4-mobility-model.h"
#include "ns3/walker-constellation.h"
#include "ns3/constant-position-mobility-model.h"
#include "ns3/constant-velocity-mobility-model.h"
#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/ntn-real-stack-helper.h"
#include "ns3/ntn-v2x-helper.h"
#include "ns3/sumo-traci-bridge.h"
#include "ns3/v2x-leo-direct.h"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <vector>

using namespace ns3;
using namespace ns3::ntnv2x;

namespace
{
NtnRealStackHelper* g_rs = nullptr;
Ptr<SumoTraciBridge> g_bridge;
std::vector<Ptr<MobilityModel>> g_vMobs;
std::vector<double> g_blockageDb;
std::ofstream g_out;
double g_minDirectSnrDb = 4.0;
double g_maxV2vRangeM = 1500.0;
double g_simTime = 30.0;
double g_dt = 1.0;

double
Dist(const Vector& a, const Vector& b)
{
    const double dx = a.x - b.x, dy = a.y - b.y, dz = a.z - b.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

void
Step()
{
    const double t = Simulator::Now().GetSeconds();
    if (t >= g_simTime)
    {
        return;
    }
    g_bridge->Step();
    // MEASURED LEO baseline SINR (mean over the real-cell terminals).
    const double baseline = g_rs->GetMeanDlSinrDb();
    const uint32_t n = static_cast<uint32_t>(g_vMobs.size());

    std::vector<double> directSinr(n);
    for (uint32_t v = 0; v < n; ++v)
    {
        directSinr[v] = baseline - g_blockageDb[v];
    }
    uint32_t nDirect = 0, nRelay = 0, nOrphan = 0;
    for (uint32_t v = 0; v < n; ++v)
    {
        if (directSinr[v] >= g_minDirectSnrDb)
        {
            ++nDirect;
            continue;
        }
        const Vector pv = g_vMobs[v]->GetPosition();
        double bestPeer = -1e9;
        for (uint32_t p = 0; p < n; ++p)
        {
            if (p == v || Dist(pv, g_vMobs[p]->GetPosition()) > g_maxV2vRangeM)
            {
                continue;
            }
            bestPeer = std::max(bestPeer, directSinr[p] - 3.0); // V2V hop penalty
        }
        if (bestPeer >= g_minDirectSnrDb)
        {
            ++nRelay;
        }
        else
        {
            ++nOrphan;
        }
    }
    const double directPct = n ? 100.0 * nDirect / n : 0.0;
    if (g_out.is_open())
    {
        g_out << std::fixed << std::setprecision(3) << t << "," << nDirect << "," << nRelay << ","
              << nOrphan << "," << directPct << "," << g_bridge->GetLastJitterSec() * 1000.0 << ","
              << baseline << "\n";
    }
    Simulator::Schedule(Seconds(g_dt), &Step);
}
} // namespace

int
main(int argc, char* argv[])
{
    std::size_t nVehicles = 40;
    double simTimeSec = 30.0;
    uint32_t numCellUes = 4; // representative real-cell terminals for the baseline
    double satEirpDbm = 55.0;
    double blockageDb = 14.0;
    double dtSec = 1.0;
    std::string tracePath = "/tmp/ntn-v2x-fcd.csv";
    std::string outputDir = "ntn-v2x-rural-highway-output";

    CommandLine cmd(__FILE__);
    cmd.AddValue("vehicles", "Number of vehicles", nVehicles);
    cmd.AddValue("simTime", "Simulation duration (s)", simTimeSec);
    cmd.AddValue("numCellUes", "Representative real-cell terminals", numCellUes);
    cmd.AddValue("satEirpDbm", "Satellite EIRP / gNB Tx power (dBm)", satEirpDbm);
    cmd.AddValue("blockageDb", "NLOS blockage on shadowed vehicles (dB)", blockageDb);
    cmd.AddValue("dt", "TraCI tick (s)", dtSec);
    cmd.AddValue("trace", "FCD CSV trace path (generated if missing)", tracePath);
    cmd.AddValue("minDirectSnr", "Minimum dB for direct uplink", g_minDirectSnrDb);
    cmd.AddValue("maxV2vRange", "Maximum V2V range (m) for relay", g_maxV2vRangeM);
    cmd.AddValue("outputDir", "Output directory", outputDir);
    cmd.Parse(argc, argv);
    g_simTime = simTimeSec;
    g_dt = dtSec;

    // SUMO FCD vehicle mobility (real replay).
    NtnV2xHelper::WriteSyntheticFcdCsv(tracePath, nVehicles, 30000.0, simTimeSec, dtSec);
    g_bridge = CreateObject<SumoTraciBridge>();
    if (!g_bridge->LoadFcdTrace(tracePath))
    {
        std::cerr << "failed to load FCD trace\n";
        return 1;
    }
    g_blockageDb.assign(nVehicles, 0.0);
    for (std::size_t i = 0; i < nVehicles; ++i)
    {
        Ptr<ConstantPositionMobilityModel> mob = CreateObject<ConstantPositionMobilityModel>();
        mob->SetPosition(Vector(300.0 * i, 0, 1.5));
        g_bridge->RegisterVehicle("veh" + std::to_string(i), mob);
        g_vMobs.push_back(mob);
        g_blockageDb[i] = (i % 2 == 1) ? blockageDb : 0.0; // odd-indexed = NLOS
    }

    // Real mmwave NR cell over representative highway terminals -> MEASURED baseline.
    NodeContainer satNodes;
    satNodes.Create(1);
    NodeContainer ueNodes;
    ueNodes.Create(numCellUes);
    MobilityHelper mh;
    mh.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    // Real SGP4 orbit projected into the scenario's local ENU frame: the
    // satellite passes overhead near t=0 and recedes with genuine orbital
    // dynamics (no straight-line placeholder).
    ns3::ntncon::WalkerConfig wcfgSat;
    wcfgSat.num_planes = 1;
    wcfgSat.total_sats = 80;
    wcfgSat.altitude_km = 550.0;
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
    Ptr<ListPositionAllocator> uePos = CreateObject<ListPositionAllocator>();
    for (uint32_t i = 0; i < numCellUes; ++i)
    {
        uePos->Add(Vector(2000.0 * i, 0.0, 1.5));
    }
    mh.SetPositionAllocator(uePos);
    mh.Install(ueNodes);

    NtnRealStackHelper rs;
    rs.SetSimTime(Seconds(simTimeSec));
    rs.SetOutputDir(outputDir);
    rs.SetRunTag("ntn-v2x-rural-highway");
    rs.SetSatEirpDbm(satEirpDbm);
    rs.Build(satNodes, ueNodes);
    rs.InstallTraffic(NtnRealStackHelper::TrafficProfile::MixedBouquet,
                      Seconds(1.0), Seconds(simTimeSec - 0.5));
    g_rs = &rs;

    std::filesystem::create_directories(outputDir);
    g_out.open(outputDir + "/ntn-v2x-rural-highway.csv");
    g_out << "time_s,n_direct,n_relay,n_orphan,direct_pct,jitter_ms,measured_baseline_db\n";

    Simulator::Schedule(Seconds(1.0), &Step);
    Simulator::Stop(Seconds(simTimeSec));
    Simulator::Run();
    rs.Collect();
    rs.WriteHealthReport();
    g_out.close();

    // Contrast: the OLD closed-form direct budget for a reference terminal.
    V2xLinkBudget formula = V2xLeoDirect::ComputeStatic(Vector(0, 0, 1.5),
                                                        Vector(0, 0, 550000.0), 2.0, satEirpDbm,
                                                        -110.0);
    std::cout << "ntn-v2x-rural-highway done.\n"
              << "  vehicles            : " << nVehicles << "\n"
              << "  measured cell SINR  : " << rs.GetMeanDlSinrDb() << " dB (baseline)\n"
              << "  OLD closed-form SINR: " << formula.snrDb << " dB (V2xLeoDirect — superseded)\n"
              << "  measured throughput : " << rs.GetRxThroughputMbps() << " Mbps\n"
              << "  csv                 : " << outputDir << "/ntn-v2x-rural-highway.csv\n";
    Simulator::Destroy();
    return 0;
}
