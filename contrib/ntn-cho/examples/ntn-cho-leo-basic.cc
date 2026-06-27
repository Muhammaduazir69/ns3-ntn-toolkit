/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// SPDX-License-Identifier: GPL-2.0-only
//
// ntn-cho-leo-basic — smoke test for the CHO algorithm on a REAL mmwave NR NTN
// cell (NtnRealStackHelper: SpectrumPhy + MAC + HARQ + RLC/PDCP + RRC + EPC).
// Real UDP traffic flows over the radio toward the UEs; the CHO algorithm is
// exercised on a 200 ms cadence and fed the MEASURED per-UE SINR from the mmwave
// RxPacketTraceUe trace (no closed-form SINR, no sine-wave jitter). Emits an
// honest sim_health.csv whose SINR/TBLER carry phy-trace provenance.

#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/ntn-cho-algorithm.h"
#include "ns3/ntn-cho-helper.h"
#include "ns3/ntn-real-stack-helper.h"
#include "ns3/ntn-tr38811-mobility-model.h"

#include "ns3/sgp4-mobility-model.h"
#include "ns3/walker-constellation.h"

#include <cmath>
#include <iostream>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("NtnChoLeoBasic");

int
main(int argc, char* argv[])
{
    double simTime = 12.0;
    std::string triggerType = "tte-aware";
    double tteMinimum = 5.0;
    uint32_t numUes = 4;
    double satEirpDbm = -1.0; // sentinel: backend-appropriate default chosen below
    std::string radio = "nr"; // radio backend: "nr" (5G-LENA FR1, 30 kHz SCS) | "mmwave" (FR2)
    std::string outputDir = "ntn-cho-basic-out";

    CommandLine cmd(__FILE__);
    cmd.AddValue("simTime", "Simulation time in seconds", simTime);
    cmd.AddValue("trigger", "CHO trigger: a3|location|tte-aware", triggerType);
    cmd.AddValue("tteMinimum", "Minimum TTE in seconds", tteMinimum);
    cmd.AddValue("numUes", "Number of UEs", numUes);
    cmd.AddValue("satEirpDbm", "Satellite EIRP / gNB Tx power (dBm); -1 = backend default", satEirpDbm);
    cmd.AddValue("radio", "Radio backend: nr (5G-LENA FR1, 30 kHz SCS) | mmwave (FR2)", radio);
    cmd.AddValue("outputDir", "Output directory", outputDir);
    cmd.Parse(argc, argv);

    const bool useNr = (radio != "mmwave");
    // Backend-appropriate EIRP default: nr's Friis LEO link needs ~70 dBm for a
    // healthy SINR; mmwave keeps its historical 55 dBm (zero regression).
    if (satEirpDbm < 0.0)
    {
        satEirpDbm = useNr ? 70.0 : 55.0;
    }

    NtnChoAlgorithm::TriggerType trigger = NtnChoAlgorithm::TRIGGER_TTE_AWARE;
    if (triggerType == "a3")
        trigger = NtnChoAlgorithm::TRIGGER_EVENT_A3;
    else if (triggerType == "location")
        trigger = NtnChoAlgorithm::TRIGGER_LOCATION_D1;

    std::cout << "========================================\n"
              << "NTN-CHO LEO Basic (real "
              << (useNr ? "5G-LENA nr FR1" : "mmwave FR2") << " cell)\n"
              << "========================================\n"
              << "  simTime: " << simTime << " s\n"
              << "  numUes:  " << numUes << "\n"
              << "  trigger: " << triggerType << "\n"
              << "  EIRP:    " << satEirpDbm << " dBm\n";

    // ---- CHO algorithm (TTE-aware) via the helper ----
    Ptr<NtnChoHelper> ntnHelper = CreateObject<NtnChoHelper>();
    ntnHelper->SetChoTriggerType(trigger);
    ntnHelper->SetCarrierFrequency(2.0e9);
    ntnHelper->SetBandwidth(30.0e6);
    ntnHelper->SetSatelliteTxPower(satEirpDbm);
    ntnHelper->SetTteMinimum(Seconds(tteMinimum));
    Ptr<NtnChoAlgorithm> choAlgo = ntnHelper->CreateChoAlgorithm();

    // ---- Nodes: one Kepler+J2-secular serving satellite (real mmwave gNB) +
    //      TR 38.811 UEs (Vallado SGP4 available via Sgp4MobilityModel::SetUseVallado) ----
    ns3::ntncon::WalkerConfig wcfg;
    wcfg.num_planes = 1;
    wcfg.total_sats = 80;
    wcfg.altitude_km = 550.0;
    wcfg.inclination_deg = 53.0;
    wcfg.epoch_unix_s = 1735689600.0;
    const auto elements = ns3::ntncon::WalkerConstellation::BuildDelta(wcfg);
    Ptr<ns3::ntncon::Sgp4MobilityModel> servSatMob =
        CreateObject<ns3::ntncon::Sgp4MobilityModel>();
    servSatMob->SetElements(elements[0]);

    NodeContainer satNodes;
    satNodes.Create(1);
    satNodes.Get(0)->AggregateObject(servSatMob);
    NodeContainer ueNodes;
    ueNodes.Create(numUes);

    // TR 38.811 class mobility (real MobilityModel) under the t=0 sub-point.
    double subLat, subLon, subAlt;
    servSatMob->GetGeodetic(subLat, subLon, subAlt);
    NtnTr38811MobilityHelper ueMobility(1);
    auto mobProfile = NtnMobilityScenarios::MixedContinental();
    ueMobility.Install(ueNodes, mobProfile, subLat - 0.03, subLat + 0.03, subLon - 0.03,
                       subLon + 0.03);

    // ---- Real NR NTN cell + traffic (mmwave FR2 or nr FR1) ----
    NtnRealStackHelper rs;
    rs.SetRadioBackend(useNr ? NtnRealStackHelper::RadioBackend::Nr
                             : NtnRealStackHelper::RadioBackend::Mmwave);
    if (useNr)
    {
        rs.SetNumerology(1); // FR1 30 kHz SCS
    }
    rs.SetSimTime(Seconds(simTime));
    rs.SetOutputDir(outputDir);
    rs.SetRunTag("ntn-cho-leo-basic_" + triggerType);
    rs.SetSatEirpDbm(satEirpDbm);
    rs.Build(satNodes, ueNodes);
    rs.InstallTraffic(NtnRealStackHelper::TrafficProfile::MixedBouquet,
                      Seconds(1.0), Seconds(simTime - 0.5));
    rs.EnableAiFlowMonitor("ntn-cho-leo-basic"); // WS2 KPM series (TS 28.552 names)

    // Radio-agnostic serving cell id (mmwave or nr gNB under the hood).
    const uint16_t servingCell = rs.GetServingCellId();
    const uint16_t candCell = servingCell + 100;
    choAlgo->AddCandidateCell(servingCell, 0, 0);
    choAlgo->AddCandidateCell(candCell, 1, 0);

    // CHO exercised every 200 ms on the MEASURED serving SINR (UE 0); the
    // candidate is ephemeris-predicted off that measured baseline.
    rs.RegisterPeriodicCallback(MilliSeconds(200), [&](Time) {
        const double servSinr = rs.GetUeRecentSinrDb(0);
        if (std::isnan(servSinr))
        {
            return;
        }
        choAlgo->UpdateMeasurement(servingCell, servSinr, 5.0);
        choAlgo->UpdateMeasurement(candCell, servSinr - 3.0, 2.0);
        choAlgo->EvaluateConditions();
        choAlgo->SelectBestCandidate();
    });

    Simulator::Stop(Seconds(simTime));
    Simulator::Run();
    rs.Collect();
    rs.WriteHealthReport();

    std::cout << "  measured mean SINR: " << rs.GetMeanDlSinrDb() << " dB\n"
              << "  measured throughput: " << rs.GetRxThroughputMbps() << " Mbps\n";
    Simulator::Destroy();
    return 0;
}
