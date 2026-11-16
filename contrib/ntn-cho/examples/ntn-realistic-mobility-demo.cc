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
    uint32_t numUes = 7; // one per TR 38.811 class (CHO-28), cycled if larger
    uint32_t rngRun = 1;
    double altitudeKm = 550.0;
    double freqGhz = 2.185; // TS 38.101-5 n256 downlink block (2170-2200 MHz). Was 2.0,
                              // which is n256 UPLINK and never conformant as a DL carrier.
    double satEirpDbm = -1.0; // sentinel: backend-appropriate default chosen below
    std::string radio = "nr"; // radio backend: "nr" (5G-LENA FR1, 30 kHz SCS) | "mmwave" (FR2)
    CommandLine cmd;
    cmd.AddValue("outputDir", "Output directory", outputDir);
    cmd.AddValue("simTime", "Simulation duration (s)", simTime);
    cmd.AddValue("dt", "Mobility sampling step (s)", dt);
    cmd.AddValue("rngRun", "RNG seed", rngRun);
    cmd.AddValue("altitude", "Serving constellation altitude (km)", altitudeKm);
    cmd.AddValue("freqGhz", "Carrier frequency (GHz)", freqGhz);
    cmd.AddValue("satEirpDbm", "-1 = TR 38.821 Set-1 S-band EIRP density (34 dBW/MHz); otherwise conducted power at the array input (dBm)", satEirpDbm);
    cmd.AddValue("radio", "Radio backend: nr (5G-LENA FR1, 30 kHz SCS) | mmwave (FR2)", radio);
    cmd.AddValue("numUes", "Number of real ns-3 UEs; classes assigned round robin over the 7 "
                 "TR 38.811 classes (7 = one per class)", numUes);
    cmd.Parse(argc, argv);
    g_simTime = simTime;
    g_dt = dt;

    const bool useNr = (radio != "mmwave");
    // EIRP default (NT-02 calibration). The nr default used to be 70 dBm of
    // CONDUCTED power, about 88 dBm EIRP once the ~18 dB array gain is added,
    // ~11 dB above the TR 38.821 Table 6.1.1.1-1 Set-1 S-band payload
    // (34 dBW/MHz, 77 dBm over 20 MHz). On an S-band FR1 carrier the nr
    // default is now that Set-1 density, as ntn-v2x-real-stack does; an
    // explicit --satEirpDbm keeps meaning conducted power, and mmwave keeps
    // its historical 55.0 dBm.
    // The probe mirrors the cell's carrier and bandwidth and returns the
    // equivalent CONDUCTED power from the helper's own formula
    // (density + 10log10(BW_MHz) + 30 - ArrayGainDb()), so every other
    // consumer of satEirpDbm stays consistent with the radio.
    NtnRealStackHelper eirpProbe;
    eirpProbe.SetCarrierFrequencyHz(freqGhz * 1e9);
    const bool eirpFromTr38821 = satEirpDbm < 0.0 && useNr &&
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
    // CHO-28: the README and registry promise one UE per TR 38.811 section
    // 6.1.1.1 class, but the classes were DRAWN from an equal-probability
    // profile: the 4-UE default could not cover 7 classes, and the stored 6-UE
    // run had two vehicular terminals and no static or IoT one. Assign the
    // classes round robin with a one-hot profile per node instead; positions
    // and class-specific motion still come from the same generator and seed.
    (void)fairProfile;
    for (uint32_t i = 0; i < numUes; ++i)
    {
        MobilityProfile oneHot{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        switch (i % 7)
        {
        case 0: oneHot.pStatic = 1.0; break;
        case 1: oneHot.pPedestrian = 1.0; break;
        case 2: oneHot.pVehicular = 1.0; break;
        case 3: oneHot.pHst = 1.0; break;
        case 4: oneHot.pMaritime = 1.0; break;
        case 5: oneHot.pAviation = 1.0; break;
        default: oneHot.pIot = 1.0; break;
        }
        NodeContainer one(g_ueNodes.Get(i));
        auto m = ueMobility.Install(one, oneHot, subLat - 0.05, subLat + 0.05,
                                    subLon - 0.05, subLon + 0.05);
        g_ueModels.push_back(m[0]);
    }

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
    if (eirpFromTr38821)
    {
        // NT-02: TR 38.821 Table 6.1.1.1-1 Set-1 S-band EIRP density. The
        // helper back-computes the conducted power from the bandwidth and
        // the array gain, and again in Build().
        rs.SetSatEirpDensityDbwMhz(NtnRealStackHelper::kTr38821Set1SBandEirpDensityDbwMhz);
    }
    else
    {
        // NT-02: an explicit value (or a default on a carrier outside the
        // Set-1 S-band reference) is CONDUCTED power at the array input.
        rs.SetSatConductedPowerDbm(satEirpDbm);
    }
    rs.Build(servSat, g_ueNodes);
    // satEirpDbm was resolved before Build() and handed to the rest of the
    // example; make sure the radio landed on the same conducted power.
    NS_ABORT_MSG_IF(eirpFromTr38821 && std::abs(rs.GetSatEirpDbm() - satEirpDbm) > 1e-6,
                    "Set-1 conducted power mismatch: radio "
                        << rs.GetSatEirpDbm() << " dBm vs " << satEirpDbm << " dBm");
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
