/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026 Muhammad Uzair
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Demonstrates the per-class realistic mobility generator on a REAL ns-3
 * plane. Each 3GPP TR 38.811 §6.1.1.1 UE class is installed as a genuine
 * ns-3 node carrying a NtnTr38811MobilityModel (ECEF), under a real
 * Kepler+J2-secular propagated LEO serving satellite (Vallado SGP4 available
 * via SetUseVallado) with a real mmwave NR NTN cell
 * (NtnRealStackHelper: SpectrumPhy + MAC + HARQ + RLC/PDCP + RRC + EPC) and
 * a measured UDP downlink. The UE trajectory written to mobility_trace.csv
 * is sampled from the REAL MobilityModel::GetPosition() on a Simulator tick
 * (no bare AdvanceUe loop), and the run drives Simulator::Run() so the radio
 * stack and the mobility actually advance together.
 *
 *   ./ns3 run "ntn-realistic-mobility-demo --outputDir=/tmp/mob_demo"
 */
#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/ntn-real-stack-helper.h"
#include "ns3/ntn-realistic-mobility.h"
#include "ns3/ntn-tr38811-mobility-model.h"
#include "ns3/sgp4-mobility-model.h"
#include "ns3/walker-constellation.h"

#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <vector>

using namespace ns3;
using ns3::ntncon::Sgp4MobilityModel;
using ns3::ntncon::WalkerConfig;
using ns3::ntncon::WalkerConstellation;

namespace
{
std::ofstream g_csv;
std::vector<Ptr<NtnTr38811MobilityModel>> g_ueModels;
NodeContainer g_ueNodes;
double g_simTime = 120.0;
double g_dt = 1.0;

// Sample every UE's REAL ns-3 MobilityModel position on the live event queue.
void
MobilityTick()
{
    const double t = Simulator::Now().GetSeconds();
    for (size_t i = 0; i < g_ueModels.size(); ++i)
    {
        Ptr<NtnTr38811MobilityModel> m = g_ueModels[i];
        double lat, lon, alt;
        m->GetGeodetic(lat, lon, alt);            // REAL ECEF model -> geodetic
        const Vector v = m->GetVelocity();        // REAL ECEF velocity (m/s)
        const double speed = std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
        g_csv << std::fixed << std::setprecision(3) << t << "," << i << ","
              << m->GetClassName() << "," << std::setprecision(6) << lat << ","
              << lon << "," << std::setprecision(1) << alt << ","
              << std::setprecision(2) << speed << "\n";
    }
    if (t + g_dt <= g_simTime + 1e-6)
    {
        Simulator::Schedule(Seconds(g_dt), &MobilityTick);
    }
}
} // namespace

int
main(int argc, char** argv)
{
    std::string outputDir = "./mob_demo";
    double simTime = 20.0;
    double dt = 1.0;
    uint32_t numUes = 4; // mixed TR 38.811 classes on the real radio PHY
    uint32_t rngRun = 1;
    double altitudeKm = 550.0;
    double freqGhz = 2.0;
    double satEirpDbm = -1.0; // sentinel: backend-appropriate default chosen below
    std::string radio = "nr"; // radio backend: "nr" (5G-LENA FR1, 30 kHz SCS) | "mmwave" (FR2)
    CommandLine cmd;
    cmd.AddValue("outputDir", "Output directory", outputDir);
    cmd.AddValue("simTime", "Simulation duration (s)", simTime);
    cmd.AddValue("dt", "Mobility sampling step (s)", dt);
    cmd.AddValue("rngRun", "RNG seed", rngRun);
    cmd.AddValue("altitude", "Serving constellation altitude (km)", altitudeKm);
    cmd.AddValue("freqGhz", "Carrier frequency (GHz)", freqGhz);
    cmd.AddValue("satEirpDbm", "Satellite EIRP / gNB Tx power (dBm); -1 = backend default", satEirpDbm);
    cmd.AddValue("radio", "Radio backend: nr (5G-LENA FR1, 30 kHz SCS) | mmwave (FR2)", radio);
    cmd.AddValue("numUes", "Number of real ns-3 UEs (one per TR 38.811 class)", numUes);
    cmd.Parse(argc, argv);
    g_simTime = simTime;
    g_dt = dt;

    const bool useNr = (radio != "mmwave");
    // Backend-appropriate EIRP default: nr's Friis LEO link needs ~70 dBm for a
    // healthy SINR; mmwave keeps its historical 55 dBm (zero regression).
    if (satEirpDbm < 0.0)
    {
        satEirpDbm = useNr ? 70.0 : 55.0;
    }

    std::string mkdir = "mkdir -p " + outputDir;
    if (system(mkdir.c_str()) != 0)
    {
        std::cerr << "warning: could not create " << outputDir << "\n";
    }

    // ---- Real Kepler+J2-secular serving satellite (one Walker-Delta plane;
    //      Vallado SGP4 via SetUseVallado) ----
    WalkerConfig wcfg;
    wcfg.num_planes = 1;
    wcfg.total_sats = 80;
    wcfg.altitude_km = altitudeKm;
    wcfg.inclination_deg = 53.0;
    wcfg.epoch_unix_s = 1735689600.0; // 2025-01-01
    const auto elements = WalkerConstellation::BuildDelta(wcfg);
    Ptr<Sgp4MobilityModel> serv = CreateObject<Sgp4MobilityModel>();
    serv->SetElements(elements[0]);
    NodeContainer servSat;
    servSat.Create(1);
    servSat.Get(0)->AggregateObject(serv);

    // UEs spawned in a small box under the serving satellite's t=0 sub-point so
    // they sit inside the real cell's coverage.
    double subLat, subLon, subAlt;
    serv->GetGeodetic(subLat, subLon, subAlt);

    // ---- One UE per 3GPP TR 38.811 class, installed as REAL ns-3 nodes ----
    MobilityProfile fairProfile;
    fairProfile.pStatic = 1.0 / 7;
    fairProfile.pPedestrian = 1.0 / 7;
    fairProfile.pVehicular = 1.0 / 7;
    fairProfile.pHst = 1.0 / 7;
    fairProfile.pMaritime = 1.0 / 7;
    fairProfile.pAviation = 1.0 / 7;
    fairProfile.pIot = 1.0 / 7;

    g_ueNodes.Create(numUes);
    NtnTr38811MobilityHelper ueMobility(static_cast<uint64_t>(rngRun));
    g_ueModels = ueMobility.Install(g_ueNodes, fairProfile, subLat - 0.05, subLat + 0.05,
                                    subLon - 0.05, subLon + 0.05);

    std::cout << "Spawned " << g_ueModels.size()
              << " REAL ns-3 UEs (TR 38.811 classes) under the serving pass:\n";
    for (size_t i = 0; i < g_ueModels.size(); ++i)
    {
        double lat, lon, alt;
        g_ueModels[i]->GetGeodetic(lat, lon, alt);
        std::cout << "  UE " << std::setw(2) << i << "  class=" << g_ueModels[i]->GetClassName()
                  << "  init pos=(" << std::fixed << std::setprecision(4) << lat << ", " << lon
                  << ") alt=" << std::setprecision(1) << alt << " m\n";
    }

    // ---- Real NR NTN serving cell + measured UDP plane (mmwave FR2 or nr FR1) ----
    NtnRealStackHelper rs;
    rs.SetRadioBackend(useNr ? NtnRealStackHelper::RadioBackend::Nr
                             : NtnRealStackHelper::RadioBackend::Mmwave);
    if (useNr)
    {
        rs.SetNumerology(1); // FR1 30 kHz SCS
    }
    rs.SetSimTime(Seconds(simTime));
    rs.SetOutputDir(outputDir);
    rs.SetRunTag("ntn-realistic-mobility-demo");
    rs.SetCarrierFrequencyHz(freqGhz * 1e9);
    rs.SetSatEirpDbm(satEirpDbm);
    rs.Build(servSat, g_ueNodes);
    rs.InstallTraffic(NtnRealStackHelper::TrafficProfile::EmbbStreaming, Seconds(1.0),
                      Seconds(simTime - 0.5));

    g_csv.open(outputDir + "/mobility_trace.csv");
    g_csv << "time_s,ue_id,class,lat,lon,alt_m,speed_mps\n";
    std::cout << "\nSampling REAL MobilityModel positions to " << outputDir
              << "/mobility_trace.csv (Simulator-driven)\n";

    Simulator::Schedule(Seconds(0.0), &MobilityTick);
    Simulator::Stop(Seconds(simTime));
    Simulator::Run();
    g_csv.close();

    rs.Collect();
    rs.WriteHealthReport();

    std::cout << "\nFinal positions (from REAL MobilityModel):\n";
    for (size_t i = 0; i < g_ueModels.size(); ++i)
    {
        double lat, lon, alt;
        g_ueModels[i]->GetGeodetic(lat, lon, alt);
        std::cout << "  UE " << std::setw(2) << i << "  " << g_ueModels[i]->GetClassName()
                  << "  pos=(" << std::fixed << std::setprecision(4) << lat << ", " << lon
                  << ") alt=" << std::setprecision(1) << alt << " m\n";
    }
    std::cout << "\n  measured serving SINR (mean): " << rs.GetMeanDlSinrDb() << " dB\n"
              << "  measured DL throughput:       " << rs.GetRxThroughputMbps() << " Mbps\n";

    Simulator::Destroy();
    return 0;
}
