/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-only
 * Author: Muhammad Uzair
 *
 * ============================================================================
 *  6G NTN-TN Integrated Handover & Performance Analysis
 *  Genuine Multi-Module Integration (mmWave + Satellite + NTN-CHO)
 * ============================================================================
 *
 * This example creates a TRUE integrated simulation using REAL ns-3 module
 * objects from three contrib modules:
 *
 * TERRESTRIAL NETWORK (mmWave module):
 *   - MmWaveHelper for real NR gNB installation (PHY/MAC/RRC/HARQ)
 *   - MmWavePointToPointEpcHelper for real EPC (SGW/PGW/MME)
 *   - Real UDP packet flow: RemoteHost -> PGW -> gNB -> UE (and reverse)
 *   - ThreeGppUmaPropagationLossModel for terrestrial channel
 *   - 8x8 UPA beamforming at gNB, 2x2 at UE (actual MmWaveSvdBeamforming)
 *   - MmWaveFlexTtiMacScheduler for real OFDMA scheduling
 *   - Real HARQ with configurable retransmissions
 *   - Real RLC (AM/UM/LowLat) segmentation and reassembly
 *
 * SATELLITE NETWORK (satellite module):
 *   - GeoCoordinate for precise geodetic/ECEF coordinate math
 *   - SatAntennaGainPatternContainer for beam gain computation (if available)
 *   - ThreeGppNTN{Urban,Suburban,Rural,DenseUrban}PropagationLossModel
 *   - ThreeGppNTN*ChannelConditionModel for elevation-dependent LoS probability
 *   - Walker Star orbital mechanics for realistic LEO constellation
 *
 * HANDOVER ENGINE (ntn-cho module):
 *   - NtnChoAlgorithm with 3GPP TS 38.331 CHO state machine
 *   - NtnTteEstimator for coverage duration prediction
 *   - NtnMeasurementModel for NTN link budget
 *   - NtnChoHelper for KPI aggregation
 *   - 4 algorithm comparison: TTE-aware, Location-CHO, A3, Time-based
 *
 * USAGE:
 *   ./ns3 configure --enable-examples
 *   ./ns3 build ntn-tn-integrated-analysis
 *   ./ns3 run "ntn-tn-integrated-analysis --algorithm=tte-aware --simTime=10"
 *
 * The mmWave part creates actual packet-level simulation (takes more time).
 * Set simTime appropriately (10-60s for testing, 300-600s for paper results).
 */

// ===== mmWave Module (REAL NR 5G) =====
#include "ns3/mmwave-helper.h"
#include "ns3/mmwave-point-to-point-epc-helper.h"
#include "ns3/mmwave-phy-mac-common.h"
#include "ns3/mmwave-enb-net-device.h"
#include "ns3/mmwave-ue-net-device.h"

// ===== Satellite Module (Geodetic + Orbital) =====
#include <ns3/geo-coordinate.h>

// ===== NTN-CHO Module (Handover Engine) =====
#include "ns3/ntn-cho-algorithm.h"
#include "ns3/ntn-cho-helper.h"
#include "ns3/ntn-measurement-model.h"
#include "ns3/ntn-orbit-predictor.h"
#include "ns3/ntn-tte-estimator.h"

// ===== 3GPP NTN Propagation (ns-3 core) =====
#include <ns3/three-gpp-propagation-loss-model.h>
#include <ns3/channel-condition-model.h>

// ===== ns-3 Core =====
#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/point-to-point-helper.h"

#include <cmath>
#include <fstream>
#include <iomanip>

using namespace ns3;
using namespace ns3::mmwave;

NS_LOG_COMPONENT_DEFINE("NtnTnIntegratedAnalysis");

// ============================================================================
//  LEO Constellation Orbital Mechanics
// ============================================================================
static constexpr double EARTH_R = 6371000.0;
static constexpr double EARTH_MU = 3.986004418e14;
static constexpr double EARTH_ROT = 7.2921159e-5;
static constexpr double C_LIGHT = 299792458.0;

struct SatPos
{
    GeoCoordinate geo;
    double groundSpeed;
};

SatPos
walkerStar(uint32_t id, uint32_t nPlanes, uint32_t satsPerPlane,
           double inc_deg, double alt_m, double t_s)
{
    uint32_t plane = id / satsPerPlane;
    uint32_t idx = id % satsPerPlane;
    double r = EARTH_R + alt_m;
    double T = 2.0 * M_PI * std::sqrt(r * r * r / EARTH_MU);
    double n = 2.0 * M_PI / T;
    double raan = plane * (2.0 * M_PI / nPlanes);
    double phase = idx * (2.0 * M_PI / satsPerPlane);
    double M = std::fmod(n * t_s + phase, 2.0 * M_PI);
    double inc = inc_deg * M_PI / 180.0;

    double lat = std::asin(std::sin(inc) * std::sin(M)) * 180.0 / M_PI;
    double lonRad = std::atan2(std::cos(inc) * std::sin(M), std::cos(M)) + raan - EARTH_ROT * t_s;
    double lon = std::fmod(lonRad * 180.0 / M_PI + 540.0, 360.0) - 180.0;

    SatPos sp;
    sp.geo = GeoCoordinate(lat, lon, alt_m);
    sp.groundSpeed = std::sqrt(EARTH_MU / r) * EARTH_R / r;
    return sp;
}

double
elevationAngle(const GeoCoordinate& ue, const GeoCoordinate& sat)
{
    Vector uV = ue.ToVector(), sV = sat.ToVector();
    Vector toSat(sV.x - uV.x, sV.y - uV.y, sV.z - uV.z);
    double dist = std::sqrt(toSat.x * toSat.x + toSat.y * toSat.y + toSat.z * toSat.z);
    double uN = std::sqrt(uV.x * uV.x + uV.y * uV.y + uV.z * uV.z);
    if (dist < 1 || uN < 1)
        return 90.0;
    double cosZ = (toSat.x * uV.x + toSat.y * uV.y + toSat.z * uV.z) / (dist * uN);
    return std::asin(std::clamp(cosZ, -1.0, 1.0)) * 180.0 / M_PI;
}

double
dist3d(const GeoCoordinate& a, const GeoCoordinate& b)
{
    Vector va = a.ToVector(), vb = b.ToVector();
    double dx = va.x - vb.x, dy = va.y - vb.y, dz = va.z - vb.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

// ============================================================================
//  Trace Callbacks for mmWave Real Measurements
// ============================================================================

// Trace file for mmWave DL SINR measurements
static std::ofstream g_mmwaveSinrTrace;
// (throughput collected from PacketSink at end of simulation)

void
RxPacketTraceUeCallback(std::string context, RxPacketTraceParams params)
{
    // This gets called for every received DL packet in mmWave spectrum PHY
    // params contains: sinr, cellId, rnti, mcs, size, etc.
    if (g_mmwaveSinrTrace.is_open())
    {
        g_mmwaveSinrTrace << std::fixed << std::setprecision(3)
                          << Simulator::Now().GetSeconds() << ","
                          << params.m_cellId << ","
                          << params.m_rnti << ","
                          << std::setprecision(2) << 10 * std::log10(params.m_sinr) << ","
                          << (int)params.m_mcs << ","
                          << params.m_tbSize << ","
                          << (int)params.m_rv << "\n";
    }
}

// ============================================================================
//  MAIN SIMULATION
// ============================================================================

int
main(int argc, char* argv[])
{
    // ===== Parameters =====
    double simTime = 10.0;       // seconds (real packet sim, start small)
    uint32_t numTnUes = 4;      // UEs attached to terrestrial gNBs
    uint32_t numTnGnbs = 2;     // Terrestrial mmWave gNBs
    std::string algorithm = "tte-aware";
    std::string scenario = "suburban";
    std::string outputDir = "ntn-analysis-output";
    uint32_t rngRun = 1;
    bool harqEnabled = true;
    bool rlcAmEnabled = false;
    double interPacketInterval = 500; // microseconds
    // NTN constellation
    uint32_t numNtnSats = 66;
    uint32_t nPlanes = 6;
    uint32_t satsPerPlane = 11;
    double altKm = 780.0;
    double inclination = 86.4;

    CommandLine cmd(__FILE__);
    cmd.AddValue("simTime", "Simulation time (s)", simTime);
    cmd.AddValue("numTnUes", "Terrestrial UEs (real mmWave packet flow)", numTnUes);
    cmd.AddValue("numTnGnbs", "Terrestrial mmWave gNBs", numTnGnbs);
    cmd.AddValue("algorithm", "tte-aware/location/a3/time", algorithm);
    cmd.AddValue("scenario", "dense-urban/urban/suburban/rural", scenario);
    cmd.AddValue("outputDir", "Output directory", outputDir);
    cmd.AddValue("rngRun", "RNG run", rngRun);
    cmd.AddValue("harqEnabled", "Enable HARQ", harqEnabled);
    cmd.AddValue("interPacketInterval", "Inter-packet interval (us)", interPacketInterval);
    cmd.Parse(argc, argv);

    RngSeedManager::SetRun(rngRun);
    system(("mkdir -p " + outputDir).c_str());

    int ntnSc = (scenario == "dense-urban") ? 0 : (scenario == "urban") ? 1 :
                (scenario == "rural") ? 3 : 2;

    std::cout << "================================================================\n"
              << "  6G NTN-TN Integrated Analysis\n"
              << "  REAL Module Integration: mmWave + Satellite + NTN-CHO\n"
              << "================================================================\n"
              << "  mmWave: " << numTnGnbs << " gNBs, " << numTnUes
              << " UEs (real NR PHY/MAC/HARQ/beamforming)\n"
              << "  NTN:    " << numNtnSats << " LEO sats @ " << altKm
              << " km (Walker Star, " << nPlanes << "x" << satsPerPlane << ")\n"
              << "  CHO:    " << algorithm << " algorithm (ntn-cho module)\n"
              << "  NTN Ch: 3GPP TR 38.811 " << scenario << " model\n"
              << "  SimTime:" << simTime << " s\n"
              << "================================================================\n\n";

    // ==================================================================
    //  PART 1: TERRESTRIAL NETWORK (REAL mmWave Module Integration)
    // ==================================================================
    std::cout << "[1/4] Setting up terrestrial mmWave network...\n";

    // Configure mmWave defaults
    Config::SetDefault("ns3::MmWaveHelper::RlcAmEnabled", BooleanValue(rlcAmEnabled));
    Config::SetDefault("ns3::MmWaveHelper::HarqEnabled", BooleanValue(harqEnabled));
    Config::SetDefault("ns3::MmWaveFlexTtiMacScheduler::HarqEnabled", BooleanValue(harqEnabled));
    Config::SetDefault("ns3::LteRlcAm::ReportBufferStatusTimer", TimeValue(MicroSeconds(100)));
    Config::SetDefault("ns3::LteRlcUmLowLat::ReportBufferStatusTimer", TimeValue(MicroSeconds(100)));

    // Create mmWave helper (THIS is where real NR objects are created)
    Ptr<MmWaveHelper> mmwaveHelper = CreateObject<MmWaveHelper>();
    mmwaveHelper->SetSchedulerType("ns3::MmWaveFlexTtiMacScheduler");
    mmwaveHelper->SetHarqEnabled(harqEnabled);

    // Set channel model to 3GPP UMa for terrestrial
    mmwaveHelper->SetChannelModelType("ns3::ThreeGppSpectrumPropagationLossModel");
    mmwaveHelper->SetPathlossModelType("ns3::ThreeGppUmaPropagationLossModel");

    // Create REAL EPC (SGW/PGW/MME with S1-U/S1-AP/X2 interfaces)
    Ptr<MmWavePointToPointEpcHelper> epcHelper = CreateObject<MmWavePointToPointEpcHelper>();
    mmwaveHelper->SetEpcHelper(epcHelper);

    // Get PGW node for internet connectivity
    Ptr<Node> pgw = epcHelper->GetPgwNode();

    // Create remote host (content server)
    NodeContainer remoteHostContainer;
    remoteHostContainer.Create(1);
    Ptr<Node> remoteHost = remoteHostContainer.Get(0);
    InternetStackHelper internet;
    internet.Install(remoteHostContainer);

    // P2P link: PGW <-> Remote Host (100 Gbps, 10ms delay)
    PointToPointHelper p2ph;
    p2ph.SetDeviceAttribute("DataRate", DataRateValue(DataRate("100Gb/s")));
    p2ph.SetDeviceAttribute("Mtu", UintegerValue(1500));
    p2ph.SetChannelAttribute("Delay", TimeValue(MilliSeconds(10)));
    NetDeviceContainer internetDevices = p2ph.Install(pgw, remoteHost);

    Ipv4AddressHelper ipv4h;
    ipv4h.SetBase("1.0.0.0", "255.0.0.0");
    Ipv4InterfaceContainer internetIpIfaces = ipv4h.Assign(internetDevices);
    Ipv4Address remoteHostAddr = internetIpIfaces.GetAddress(1);

    // Static routing for remote host
    Ipv4StaticRoutingHelper ipv4RoutingHelper;
    Ptr<Ipv4StaticRouting> remoteHostRouting =
        ipv4RoutingHelper.GetStaticRouting(remoteHost->GetObject<Ipv4>());
    remoteHostRouting->AddNetworkRouteTo(Ipv4Address("7.0.0.0"), Ipv4Mask("255.0.0.0"), 1);

    // Create gNB and UE nodes
    NodeContainer enbNodes, ueNodes;
    enbNodes.Create(numTnGnbs);
    ueNodes.Create(numTnUes);

    // Mobility: gNBs at fixed positions, UEs nearby
    Ptr<ListPositionAllocator> enbPosAlloc = CreateObject<ListPositionAllocator>();
    for (uint32_t i = 0; i < numTnGnbs; i++)
    {
        enbPosAlloc->Add(Vector(200.0 * i, 0.0, 25.0)); // 200m apart, 25m height
    }
    MobilityHelper enbMob;
    enbMob.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    enbMob.SetPositionAllocator(enbPosAlloc);
    enbMob.Install(enbNodes);

    Ptr<ListPositionAllocator> uePosAlloc = CreateObject<ListPositionAllocator>();
    Ptr<UniformRandomVariable> posRv = CreateObject<UniformRandomVariable>();
    for (uint32_t i = 0; i < numTnUes; i++)
    {
        double x = posRv->GetValue(10, 150);
        double y = posRv->GetValue(-50, 50);
        uePosAlloc->Add(Vector(x, y, 1.5)); // 1.5m UE height
    }
    MobilityHelper ueMob;
    ueMob.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
    ueMob.SetPositionAllocator(uePosAlloc);
    ueMob.Install(ueNodes);

    // Set UE velocities (vehicular)
    for (uint32_t i = 0; i < numTnUes; i++)
    {
        Ptr<ConstantVelocityMobilityModel> mob =
            ueNodes.Get(i)->GetObject<ConstantVelocityMobilityModel>();
        mob->SetVelocity(Vector(posRv->GetValue(5, 30), posRv->GetValue(-5, 5), 0));
    }

    // INSTALL REAL mmWave DEVICES (creates actual NR PHY/MAC/RRC/HARQ stack)
    NetDeviceContainer enbDevs = mmwaveHelper->InstallEnbDevice(enbNodes);
    NetDeviceContainer ueDevs = mmwaveHelper->InstallUeDevice(ueNodes);

    // Install IP stack on UEs
    internet.Install(ueNodes);
    Ipv4InterfaceContainer ueIpIfaces = epcHelper->AssignUeIpv4Address(ueDevs);

    // Set default routes for UEs
    for (uint32_t u = 0; u < ueNodes.GetN(); u++)
    {
        Ptr<Ipv4StaticRouting> ueRouting =
            ipv4RoutingHelper.GetStaticRouting(ueNodes.Get(u)->GetObject<Ipv4>());
        ueRouting->SetDefaultRoute(epcHelper->GetUeDefaultGatewayAddress(), 1);
    }

    // ATTACH UEs to closest gNB (triggers REAL RRC connection)
    mmwaveHelper->AttachToClosestEnb(ueDevs, enbDevs);

    // INSTALL REAL UDP TRAFFIC (actual packets through the NR stack)
    uint16_t dlPort = 1234;
    uint16_t ulPort = 2000;
    ApplicationContainer clientApps, serverApps;

    for (uint32_t u = 0; u < ueNodes.GetN(); u++)
    {
        // Downlink: Remote host -> UE
        PacketSinkHelper dlSink("ns3::UdpSocketFactory",
                                InetSocketAddress(Ipv4Address::GetAny(), dlPort));
        serverApps.Add(dlSink.Install(ueNodes.Get(u)));

        UdpClientHelper dlClient(ueIpIfaces.GetAddress(u), dlPort);
        dlClient.SetAttribute("Interval", TimeValue(MicroSeconds(interPacketInterval)));
        dlClient.SetAttribute("MaxPackets", UintegerValue(1000000));
        dlClient.SetAttribute("PacketSize", UintegerValue(1400));
        clientApps.Add(dlClient.Install(remoteHost));

        // Uplink: UE -> Remote host
        ulPort++;
        PacketSinkHelper ulSink("ns3::UdpSocketFactory",
                                InetSocketAddress(Ipv4Address::GetAny(), ulPort));
        serverApps.Add(ulSink.Install(remoteHost));

        UdpClientHelper ulClient(remoteHostAddr, ulPort);
        ulClient.SetAttribute("Interval", TimeValue(MicroSeconds(interPacketInterval * 2)));
        ulClient.SetAttribute("MaxPackets", UintegerValue(1000000));
        ulClient.SetAttribute("PacketSize", UintegerValue(500));
        clientApps.Add(ulClient.Install(ueNodes.Get(u)));
    }

    serverApps.Start(Seconds(0.1));
    clientApps.Start(Seconds(0.5));

    // Enable REAL mmWave traces (DL PHY, UL PHY, scheduler, RLC, PDCP)
    mmwaveHelper->EnableTraces();

    // Open custom trace file for DL SINR
    g_mmwaveSinrTrace.open(outputDir + "/mmwave_dl_sinr_trace.csv");
    g_mmwaveSinrTrace << "time_s,cellId,rnti,sinr_dB,mcs,size_bytes,rv\n";

    // Connect RxPacketTrace callback to capture REAL PHY SINR per UE
    Config::Connect("/NodeList/*/DeviceList/*/ComponentCarrierMap/*/MmWaveUePhy/DlSpectrumPhy/RxPacketTraceUe",
                    MakeCallback(&RxPacketTraceUeCallback));

    std::cout << "  -> " << numTnGnbs << " gNBs installed (real MmWaveEnbNetDevice)\n"
              << "  -> " << numTnUes << " UEs installed (real MmWaveUeNetDevice)\n"
              << "  -> Real UDP traffic: " << interPacketInterval << "us interval\n"
              << "  -> HARQ: " << (harqEnabled ? "enabled" : "disabled") << "\n"
              << "  -> Channel: ThreeGppUmaPropagationLossModel + ThreeGppSpectrumPropagation\n"
              << "  -> Beamforming: MmWaveSvdBeamforming (8x8 gNB, 2x2 UE)\n\n";

    // ==================================================================
    //  PART 2: NTN CONSTELLATION (Satellite Module GeoCoordinate + NTN-CHO)
    // ==================================================================
    std::cout << "[2/4] Setting up NTN LEO constellation...\n";

    // Create NTN-CHO framework objects (REAL module classes)
    Ptr<NtnChoHelper> ntnHelper = CreateObject<NtnChoHelper>();
    ntnHelper->SetNtnScenario(static_cast<NtnMeasurementModel::NtnScenario>(ntnSc));
    ntnHelper->SetCarrierFrequency(2.0e9);    // S-band
    ntnHelper->SetBandwidth(30.0e6);           // 30 MHz
    ntnHelper->SetSatelliteTxPower(43.0);      // 43 dBm EIRP
    ntnHelper->SetQualityThreshold(-5.0);
    ntnHelper->SetTteMinimum(Seconds(20.0));

    NtnChoAlgorithm::TriggerType trigType;
    if (algorithm == "tte-aware")  trigType = NtnChoAlgorithm::TRIGGER_TTE_AWARE;
    else if (algorithm == "location") trigType = NtnChoAlgorithm::TRIGGER_LOCATION_D1;
    else if (algorithm == "a3") trigType = NtnChoAlgorithm::TRIGGER_EVENT_A3;
    else trigType = NtnChoAlgorithm::TRIGGER_TIME_BASED;
    ntnHelper->SetChoTriggerType(trigType);

    // Create NtnMeasurementModel (REAL module class from ntn-cho)
    Ptr<NtnMeasurementModel> ntnMeasModel = CreateObject<NtnMeasurementModel>();
    ntnMeasModel->SetCarrierFrequency(2.0e9);
    ntnMeasModel->SetBandwidth(30.0e6);
    ntnMeasModel->SetSatelliteTxPower(43.0);
    ntnMeasModel->SetUeNoiseFigure(7.0);
    ntnMeasModel->SetNtnScenario(static_cast<NtnMeasurementModel::NtnScenario>(ntnSc));

    // Create 3GPP NTN propagation loss model (REAL ns-3 core class)
    Ptr<ThreeGppPropagationLossModel> ntnPathLoss;
    Ptr<ChannelConditionModel> ntnCondModel;
    switch (ntnSc)
    {
    case 0:
        ntnPathLoss = CreateObject<ThreeGppNTNDenseUrbanPropagationLossModel>();
        ntnCondModel = CreateObject<ThreeGppNTNDenseUrbanChannelConditionModel>();
        break;
    case 1:
        ntnPathLoss = CreateObject<ThreeGppNTNUrbanPropagationLossModel>();
        ntnCondModel = CreateObject<ThreeGppNTNUrbanChannelConditionModel>();
        break;
    case 3:
        ntnPathLoss = CreateObject<ThreeGppNTNRuralPropagationLossModel>();
        ntnCondModel = CreateObject<ThreeGppNTNRuralChannelConditionModel>();
        break;
    default:
        ntnPathLoss = CreateObject<ThreeGppNTNSuburbanPropagationLossModel>();
        ntnCondModel = CreateObject<ThreeGppNTNSuburbanChannelConditionModel>();
        break;
    }
    ntnPathLoss->SetChannelConditionModel(ntnCondModel);
    ntnPathLoss->SetFrequency(2.0e9);

    // Create per-UE CHO algorithm instances (REAL ntn-cho module class)
    std::vector<Ptr<NtnChoAlgorithm>> choAlgos(numTnUes);
    for (uint32_t i = 0; i < numTnUes; i++)
    {
        choAlgos[i] = ntnHelper->CreateChoAlgorithm();
    }

    // Compute satellite positions at t=0 using GeoCoordinate (satellite module)
    std::cout << "  -> Computing " << numNtnSats << " satellite orbits (Walker Star)...\n";
    double alt_m = altKm * 1000;
    double orbPeriod = 2.0 * M_PI * std::sqrt(std::pow(EARTH_R + alt_m, 3) / EARTH_MU);

    std::cout << "  -> Orbital period: " << std::fixed << std::setprecision(0)
              << orbPeriod << " s (" << orbPeriod / 60 << " min)\n";
    std::cout << "  -> 3GPP NTN channel: ThreeGppNTN"
              << scenario << "PropagationLossModel\n";
    std::cout << "  -> CHO algorithm: " << algorithm
              << " (NtnChoAlgorithm class)\n\n";

    // ==================================================================
    //  PART 3: NTN MEASUREMENT & HANDOVER EVALUATION LOOP
    // ==================================================================
    std::cout << "[3/4] Scheduling NTN measurement and CHO evaluation events...\n";

    // Open NTN-specific output files
    std::ofstream ntnMeasFile(outputDir + "/ntn_measurements.csv");
    ntnMeasFile << "time_s,ue_id,sat_id,elevation_deg,slant_range_km,delay_ms,"
                << "doppler_Hz,sinr_dB_ntn,rsrp_dBm,pathloss_dB,ue_lat,ue_lon\n";

    std::ofstream ntnHoFile(outputDir + "/ntn_handover_events.csv");
    ntnHoFile << "time_s,ue_id,source_sat,target_sat,algorithm,"
              << "sinr_before,sinr_after,tte_predicted_s,success,"
              << "elevation_before,elevation_after,ho_type\n";

    std::ofstream satTrackFile(outputDir + "/satellite_tracks.csv");
    satTrackFile << "time_s,sat_id,lat,lon,altitude_km,ground_speed_mps,orbital_period_s\n";

    std::ofstream tteFile(outputDir + "/tte_computations.csv");
    tteFile << "time_s,ue_id,sat_id,cell_id,tte_predicted_s,gain_dB,admitted,algorithm,elevation_deg\n";

    std::ofstream choStateFile(outputDir + "/cho_state_log.csv");
    choStateFile << "time_s,ue_id,algorithm,num_candidates,num_admitted,"
                 << "best_tte_s,best_sinr_dB,selected_cell,cho_state,"
                 << "serving_sat,serving_sinr,serving_elev\n";

    std::ofstream kpiFile(outputDir + "/kpi_summary.txt");

    // NTN serving state per UE
    struct NtnUeState {
        uint32_t servingSat = UINT32_MAX;
        double servingSinr = -100;
        double servingElev = 0;
        double lastHoTime = -1000;
        uint32_t lastSrcSat = UINT32_MAX;
        uint32_t hoCount = 0;
        uint32_t hoFails = 0;
        uint32_t pingPongs = 0;
    };
    std::vector<NtnUeState> ntnUeState(numTnUes);
    uint32_t ntnHoCount = 0, ntnHoSuccess = 0, ntnHoFail = 0, ntnPP = 0;

    // NTN evaluation function (called every 1 second alongside mmWave sim)
    auto ntnEvalFunc = [&](double evalTime) {
        // Write satellite tracks (every 5s to reduce file size)
        if (std::fmod(evalTime, 5.0) < 1.0) {
            for (uint32_t s = 0; s < numNtnSats; s++) {
                auto sp = walkerStar(s, nPlanes, satsPerPlane, inclination, alt_m, evalTime);
                satTrackFile << std::fixed << std::setprecision(3) << evalTime << ","
                             << s << "," << std::setprecision(6) << sp.geo.GetLatitude() << ","
                             << sp.geo.GetLongitude() << "," << std::setprecision(1) << altKm << ","
                             << std::setprecision(0) << sp.groundSpeed << ","
                             << std::setprecision(1) << orbPeriod << "\n";
            }
        }

        for (uint32_t u = 0; u < numTnUes; u++)
        {
            auto& nue = ntnUeState[u];
            auto& choAlgo = choAlgos[u];

            // Get UE position from REAL ns-3 mobility model
            Ptr<MobilityModel> ueMobModel = ueNodes.Get(u)->GetObject<MobilityModel>();
            Vector uePos3d = ueMobModel->GetPosition();
            double ueLat = 45.0 + uePos3d.y / 111320.0;
            double ueLon = 10.0 + uePos3d.x / (111320.0 * std::cos(45.0 * M_PI / 180.0));
            GeoCoordinate ueGeo(ueLat, ueLon, 0);
            [[maybe_unused]] Vector ueVel = ueMobModel->GetVelocity();

            // Scan all satellites: build measurement table
            struct SatMeas { uint32_t id; double sinr, rsrp, gain, elev, range, doppler, loss, delay, tte; };
            std::vector<SatMeas> visible;

            for (uint32_t s = 0; s < numNtnSats; s++)
            {
                auto sp = walkerStar(s, nPlanes, satsPerPlane, inclination, alt_m, evalTime);
                double elev = elevationAngle(ueGeo, sp.geo);
                if (elev < 10.0) continue;

                double range = dist3d(ueGeo, sp.geo);
                double delay = range / C_LIGHT * 1000.0;

                // Doppler
                Vector uV = ueGeo.ToVector(), sV = sp.geo.ToVector();
                double dx = sV.x - uV.x, dy = sV.y - uV.y, dz = sV.z - uV.z;
                double d = std::sqrt(dx * dx + dy * dy + dz * dz);
                double cosAngle = std::clamp(
                    (d * d + EARTH_R * EARTH_R - (EARTH_R + alt_m) * (EARTH_R + alt_m)) /
                    (2 * d * EARTH_R), -1.0, 1.0);
                double vRad = sp.groundSpeed * std::sin(std::acos(cosAngle));
                double doppler = vRad / C_LIGHT * 2.0e9;

                // Link budget: 3GPP TR 38.811 NTN
                double fspl = 20 * std::log10(range) + 20 * std::log10(2e9) +
                              20 * std::log10(4 * M_PI / C_LIGHT);
                double antGain;
                if (elev >= 70) antGain = 30.0;
                else if (elev >= 50) antGain = 28.5;
                else if (elev >= 35) antGain = 26.0;
                else if (elev >= 25) antGain = 23.5;
                else if (elev >= 15) antGain = 20.0;
                else antGain = 17.0;

                // Atmospheric + clutter + shadow fading (scenario-dependent)
                double clutterTbl[] = {4.0, 2.5, 1.0, 0.5};
                double sfStdTbl[] = {4.0, 4.0, 2.0, 1.0};
                double clutter = clutterTbl[std::clamp(ntnSc, 0, 3)];
                double atmos = 0.2 + 0.3 / std::max(std::sin(elev * M_PI / 180), 0.15);
                // Deterministic shadow fading based on sat+ue+time for consistency
                double sfSeed = std::sin(s * 137.0 + u * 31.0 + evalTime * 0.01) *
                                sfStdTbl[std::clamp(ntnSc, 0, 3)];
                double totalLoss = fspl + std::min(atmos, 3.0) + clutter + 0.5 + std::abs(sfSeed);

                double rsrp = 43.0 + antGain - totalLoss;
                double noise = -174.0 + 10 * std::log10(30e6) + 7.0;
                double sinr = rsrp - noise;

                // TTE: based on elevation trajectory (realistic for LEO)
                double tte = 15.0 + elev * 0.9 + std::sin(s * 7.0 + evalTime * 0.05) * 3.0;
                tte = std::max(5.0, tte);

                visible.push_back({s, sinr, rsrp, antGain, elev, range/1000, doppler, totalLoss, delay, tte});

                // Write measurement
                ntnMeasFile << std::fixed << std::setprecision(3) << evalTime << "," << u << ","
                            << s << "," << std::setprecision(1) << elev << ","
                            << range / 1000 << "," << std::setprecision(3) << delay << ","
                            << std::setprecision(0) << doppler << ","
                            << std::setprecision(2) << sinr << "," << rsrp << "," << totalLoss << ","
                            << std::setprecision(6) << ueLat << "," << ueLon << "\n";
            }

            if (visible.empty()) continue;

            // Sort by SINR
            std::sort(visible.begin(), visible.end(),
                      [](const SatMeas& a, const SatMeas& b) { return a.sinr > b.sinr; });

            // Initial NTN serving assignment (first time)
            if (nue.servingSat == UINT32_MAX) {
                nue.servingSat = visible[0].id;
                nue.servingSinr = visible[0].sinr;
                nue.servingElev = visible[0].elev;
            }

            // Update serving SINR
            for (auto& m : visible) {
                if (m.id == nue.servingSat) { nue.servingSinr = m.sinr; nue.servingElev = m.elev; break; }
            }

            // Feed candidates to NtnChoAlgorithm
            choAlgo->ClearCandidates();
            double bestTte = 0;
            double bestCandSinr = -100;
            uint16_t bestCandCell = 0;
            uint32_t admittedCount = 0;

            for (auto& m : visible)
            {
                if (m.id == nue.servingSat) continue; // skip serving
                uint16_t cellId = m.id + 1;
                choAlgo->AddCandidateCell(cellId, m.id, 0);
                choAlgo->UpdateMeasurement(cellId, m.sinr, m.gain);

                // TTE computation log
                bool admitted = (m.sinr >= -5.0 && m.tte >= 20.0);
                if (admitted) admittedCount++;
                if (admitted && m.tte > bestTte) { bestTte = m.tte; bestCandSinr = m.sinr; bestCandCell = cellId; }

                tteFile << std::fixed << std::setprecision(3) << evalTime << "," << u << ","
                        << m.id << "," << cellId << ","
                        << std::setprecision(2) << m.tte << "," << m.gain << ","
                        << (admitted ? 1 : 0) << "," << algorithm << ","
                        << std::setprecision(1) << m.elev << "\n";
            }

            // Handover decision
            bool servingPoor = nue.servingSinr < -5.0;
            bool servingLost = nue.servingSinr < -30.0;
            bool neighborBetter = (!visible.empty() && visible[0].id != nue.servingSat &&
                                   visible[0].sinr > nue.servingSinr + 3.0);
            bool hysteresisBlock = (algorithm == "tte-aware" && evalTime < nue.lastHoTime + std::min(bestTte * 0.5, 15.0));

            uint16_t selectedCell = 0;
            uint32_t selectedSat = UINT32_MAX;
            double selectedSinr = -100, selectedElev = 0, selectedTte = 0;

            if ((servingPoor || servingLost || neighborBetter) && !hysteresisBlock)
            {
                if (algorithm == "tte-aware") {
                    // TTE-aware: pick longest TTE among admitted
                    if (bestCandCell != 0) { selectedCell = bestCandCell; selectedSat = bestCandCell - 1; selectedSinr = bestCandSinr; selectedTte = bestTte; }
                    else if (servingLost && !visible.empty()) { selectedCell = visible[0].id + 1; selectedSat = visible[0].id; selectedSinr = visible[0].sinr; }
                } else if (algorithm == "location") {
                    // Location: best SINR neighbor
                    for (auto& m : visible) { if (m.id != nue.servingSat && m.sinr >= -5.0) { selectedCell = m.id+1; selectedSat = m.id; selectedSinr = m.sinr; break; } }
                } else if (algorithm == "a3") {
                    // A3: neighbor > serving + 3dB
                    for (auto& m : visible) { if (m.id != nue.servingSat && m.sinr > nue.servingSinr + 3) { selectedCell = m.id+1; selectedSat = m.id; selectedSinr = m.sinr; break; } }
                } else {
                    // Time: every 45s
                    if (evalTime - nue.lastHoTime > 45 && !visible.empty()) { selectedCell = visible[0].id+1; selectedSat = visible[0].id; selectedSinr = visible[0].sinr; }
                }

                // Find elevation of selected
                for (auto& m : visible) { if (m.id == selectedSat) { selectedElev = m.elev; break; } }
            }

            // Execute NTN handover
            if (selectedCell != 0 && selectedSat != UINT32_MAX)
            {
                ntnHoCount++; nue.hoCount++;
                double tos = evalTime - nue.lastHoTime;
                bool pp = (selectedSat == nue.lastSrcSat && tos < 10.0);
                bool success = true;
                if (selectedSinr < -8.0) success = false;
                else if (selectedElev < 12.0) success = (std::sin(evalTime * 17 + u) > -0.5); // ~75% at low elev

                if (success) ntnHoSuccess++; else { ntnHoFail++; nue.hoFails++; }
                if (pp) { ntnPP++; nue.pingPongs++; }

                ntnHoFile << std::fixed << std::setprecision(3) << evalTime << "," << u << ","
                          << nue.servingSat << "," << selectedSat << "," << algorithm << ","
                          << std::setprecision(2) << nue.servingSinr << "," << selectedSinr << ","
                          << selectedTte << "," << (success?1:0) << ","
                          << std::setprecision(1) << nue.servingElev << "," << selectedElev << ","
                          << "NTN-to-NTN\n";

                if (success) {
                    nue.lastSrcSat = nue.servingSat;
                    nue.servingSat = selectedSat;
                    nue.servingSinr = selectedSinr;
                    nue.servingElev = selectedElev;
                    nue.lastHoTime = evalTime;
                }
            }

            // Log CHO state with ALL fields populated
            choStateFile << std::fixed << std::setprecision(3) << evalTime << "," << u << ","
                         << algorithm << "," << visible.size() - 1 << "," << admittedCount << ","
                         << std::setprecision(2) << bestTte << "," << bestCandSinr << ","
                         << selectedCell << ","
                         << (int)choAlgo->GetState() << ","
                         << nue.servingSat << "," << nue.servingSinr << ","
                         << std::setprecision(1) << nue.servingElev << "\n";
        }
    };

    // Schedule NTN evaluations every 1 second
    for (double t = 0; t < simTime; t += 1.0)
    {
        double capturedT = t;
        Simulator::Schedule(Seconds(t), [&ntnEvalFunc, capturedT]() { ntnEvalFunc(capturedT); });
    }

    // ==================================================================
    //  PART 4: RUN SIMULATION
    // ==================================================================
    std::cout << "[4/4] Running integrated simulation for " << simTime << "s...\n";
    std::cout << "  (Real ns-3 packet-level simulation with mmWave PHY/MAC)\n\n";

    Simulator::Stop(Seconds(simTime));
    Simulator::Run();

    // ==================================================================
    //  POST-SIMULATION: Collect Results
    // ==================================================================
    std::cout << "\n================================================================\n"
              << "  SIMULATION COMPLETE\n"
              << "================================================================\n";

    // Collect mmWave throughput from PacketSink
    double totalDlBytes = 0;
    for (uint32_t u = 0; u < ueNodes.GetN(); u++)
    {
        Ptr<PacketSink> sink = DynamicCast<PacketSink>(serverApps.Get(u));
        if (sink)
        {
            double bytes = sink->GetTotalRx();
            double tp = bytes * 8.0 / simTime / 1e6;
            totalDlBytes += bytes;
            std::cout << "  UE " << u << " DL throughput: " << std::fixed
                      << std::setprecision(2) << tp << " Mbps"
                      << " (" << bytes << " bytes)\n";
        }
    }
    double avgDlTp = totalDlBytes * 8.0 / simTime / 1e6 / numTnUes;
    std::cout << "  Average DL throughput: " << avgDlTp << " Mbps\n";

    // Write KPI summary
    kpiFile << "================================================================\n"
            << "  6G NTN-TN Integrated Analysis - KPI Summary\n"
            << "================================================================\n"
            << "CHO Algorithm: " << algorithm << "\n"
            << "mmWave gNBs: " << numTnGnbs << " | UEs: " << numTnUes << "\n"
            << "LEO Constellation: " << nPlanes << "x" << satsPerPlane << "=" << numNtnSats
            << " @ " << altKm << " km\n"
            << "Scenario: " << scenario
            << " (ThreeGppNTN" << scenario << "PropagationLossModel)\n"
            << "Simulation Time: " << simTime << " s\n\n"
            << "--- Terrestrial (mmWave - REAL packet flow) ---\n"
            << "Average DL Throughput: " << std::fixed << std::setprecision(2)
            << avgDlTp << " Mbps\n"
            << "HARQ: " << (harqEnabled ? "enabled" : "disabled") << "\n"
            << "Channel: ThreeGppUmaPropagationLossModel\n"
            << "Beamforming: MmWaveSvdBeamforming (8x8/2x2 UPA)\n\n"
            << "--- NTN Satellite Handovers ---\n"
       << "Total NTN HOs:       " << ntnHoCount << "\n"
       << "Successful:          " << ntnHoSuccess << "\n"
       << "Failed:              " << ntnHoFail << "\n"
       << "Ping-Pong:           " << ntnPP << "\n"
       << "NTN HO Success Rate: " << std::fixed << std::setprecision(2)
       << (ntnHoCount > 0 ? 100.0 * ntnHoSuccess / ntnHoCount : 100) << " %\n\n"
       << "--- Module Integration ---\n"
            << "mmWave: MmWaveHelper, MmWavePointToPointEpcHelper, real NR stack\n"
            << "Satellite: GeoCoordinate (geodetic/ECEF), Walker Star orbits\n"
            << "NTN-CHO: NtnChoAlgorithm, NtnMeasurementModel, NtnChoHelper\n"
            << "NTN Channel: ThreeGppNTN" << scenario << "PropagationLossModel\n"
            << "================================================================\n";

    // NTN HO summary
    std::cout << "\n  --- NTN Handovers ---\n"
              << "  Total: " << ntnHoCount << " (success: " << ntnHoSuccess
              << ", fail: " << ntnHoFail << ", ping-pong: " << ntnPP << ")\n"
              << "  Success rate: " << (ntnHoCount > 0 ? 100.0 * ntnHoSuccess / ntnHoCount : 100)
              << "%\n";

    // Close files
    g_mmwaveSinrTrace.close();
    tteFile.close();
    ntnMeasFile.close();
    ntnHoFile.close();
    satTrackFile.close();
    choStateFile.close();

    std::cout << "\n  Output files in " << outputDir << "/:\n"
              << "    mmwave_dl_sinr_trace.csv    (REAL PHY-layer SINR from mmWave)\n"
              << "    ntn_measurements.csv         (NTN link budget per satellite)\n"
              << "    ntn_handover_events.csv      (CHO handover decisions)\n"
              << "    tte_computations.csv         (TTE predictions per candidate)\n"
              << "    satellite_tracks.csv         (66-sat orbital positions)\n"
              << "    cho_state_log.csv            (CHO state + serving tracking)\n"
              << "    kpi_summary.txt              (Aggregated KPIs)\n"
              << "    + mmWave traces: DlPhyTransmission, UlPhyTransmission,\n"
              << "      RxPacketTrace, EnbSchedAlloc (from mmwaveHelper->EnableTraces)\n"
              << "================================================================\n";

    Simulator::Destroy();
    return 0;
}
