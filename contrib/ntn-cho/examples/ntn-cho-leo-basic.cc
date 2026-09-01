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
#include "ns3/ntn-orbit-predictor.h"
#include "ns3/ntn-real-stack-helper.h"
#include "ns3/ntn-tr38811-mobility-model.h"

#include "ns3/geo-coordinate.h"
#include "ns3/satellite-antenna-gain-pattern-container.h"
#include "ns3/satellite-constant-position-mobility-model.h"
#include "ns3/satellite-env-variables.h"
#include "ns3/sgp4-mobility-model.h"
#include "ns3/singleton.h"
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
    // CHO-5: the algorithm used to be created here, before the satellite nodes
    // below existed, so SetupConstellation() could not have run first and the
    // helper handed it a null time-to-exit estimator. The example's own DEFAULT
    // trigger is tte-aware, which needs that estimator, so out of the box this
    // aborted on the guard in EvaluateConditions(). Created below instead.

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

    // CHO-23: give the scenario a real second cell.
    //
    // This example used `candCell = servingCell + 100`, an id naming no gNB in
    // the simulation, and fed it a candidate SINR of `servSinr - 3.0`, a fixed
    // offset the comment described as ephemeris-predicted. With one satellite
    // there was no candidate to hand over to, the CHO evaluation could not
    // change anything, and SelectBestCandidate()'s return value was discarded
    // without being read. The Walker build already returns 80 elements, so the
    // neighbour in the same plane is available at no modelling cost.
    Ptr<ns3::ntncon::Sgp4MobilityModel> candSatMob =
        CreateObject<ns3::ntncon::Sgp4MobilityModel>();
    candSatMob->SetElements(elements[1]);

    NodeContainer satNodes;
    satNodes.Create(2);
    satNodes.Get(0)->AggregateObject(servSatMob);
    satNodes.Get(1)->AggregateObject(candSatMob);
    NodeContainer ueNodes;
    ueNodes.Create(numUes);

    // Give the helper a constellation before asking it for an algorithm, then
    // switch the predictor to the analytic beam, which is what the other three
    // CHO examples do and what this one was missing.
    Singleton<SatEnvVariables>::Get()->DoInitialize();
    Singleton<SatEnvVariables>::Get()->SetOutputVariables("ntn-cho-leo-basic", "", true);
    Ptr<SatAntennaGainPatternContainer> agp = CreateObject<SatAntennaGainPatternContainer>(
        2,
        Singleton<SatEnvVariables>::Get()->LocateDataDirectory() +
            "/scenarios/geo-33E/antennapatterns");
    NodeContainer auxSats;
    auxSats.Create(2);
    Ptr<SatConstantPositionMobilityModel> auxServ =
        CreateObject<SatConstantPositionMobilityModel>();
    Ptr<SatConstantPositionMobilityModel> auxCand =
        CreateObject<SatConstantPositionMobilityModel>();
    auxServ->SetGeoPosition(GeoCoordinate(servSatMob->GetPosition()));
    auxCand->SetGeoPosition(GeoCoordinate(servSatMob->GetPosition()));
    auxSats.Get(0)->AggregateObject(auxServ);
    auxSats.Get(1)->AggregateObject(auxCand);
    agp->ConfigureBeamsMobility(0, auxServ);
    agp->ConfigureBeamsMobility(1, auxCand);
    ntnHelper->SetupConstellation(auxSats, agp);

    // CHO-2: the shipped SNS3 pattern grids are GEO-referenced and are not
    // defined for beam id 0, so touching them aborts. The analytic TR 38.811
    // 6.4.1 beam is what the other CHO examples use; same Set-1 parameters.
    if (Ptr<NtnOrbitPredictor> orbit = ntnHelper->GetOrbitPredictor())
    {
        orbit->SetGeometricBeam(/*peakGainDbi=*/30.0, /*beamwidth3dbDeg=*/4.4127);
        // CHO-23: the aux models above are stationary and exist only to carry
        // the antenna-pattern plumbing. Bind the predictor's kinematics to the
        // real propagated orbits, as the other CHO examples do.
        orbit->SetKinematicsSource(0, servSatMob);
        orbit->SetKinematicsSource(1, candSatMob);
        NS_ABORT_MSG_IF(orbit->CountFrozenSatellites() > 0,
                        "CHO predictor still has stationary satellites");
    }

    Ptr<NtnChoAlgorithm> choAlgo = ntnHelper->CreateChoAlgorithm();

    // TR 38.811 class mobility (real MobilityModel) under the t=0 sub-point.
    double subLat, subLon, subAlt;
    servSatMob->GetGeodetic(subLat, subLon, subAlt);
    NtnTr38811MobilityHelper ueMobility(1);
    auto mobProfile = NtnMobilityScenarios::MixedContinental();
    auto ueModels = ueMobility.Install(ueNodes, mobProfile, subLat - 0.03, subLat + 0.03,
                                      subLon - 0.03, subLon + 0.03);
    Ptr<NtnTr38811MobilityModel> ueMob = ueModels[0];

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
    // NT-02: declared as CONDUCTED power at the array input. This carrier has
    // no TR 38.821 Set-1 reference in the toolkit, so the EIRP health gate
    // reports "not asserted" rather than certifying an uncalibrated budget.
    rs.SetSatConductedPowerDbm(satEirpDbm);
    rs.Build(satNodes, ueNodes);
    rs.InstallTraffic(NtnRealStackHelper::TrafficProfile::MixedBouquet,
                      Seconds(1.0), Seconds(simTime - 0.5));
    rs.EnableAiFlowMonitor("ntn-cho-leo-basic"); // WS2 KPM series (TS 28.552 names)

    // Radio-agnostic serving cell id (mmwave or nr gNB under the hood).
    const uint16_t servingCell = rs.GetGnbCellId(0);
    const uint16_t candCell = rs.GetGnbCellId(1);
    NS_ABORT_MSG_IF(candCell == 0 || candCell == servingCell,
                    "CHO needs two distinct real cells; got serving=" << servingCell
                                                                     << " candidate=" << candCell);
    choAlgo->AddCandidateCell(servingCell, 0, 0);
    choAlgo->AddCandidateCell(candCell, 1, 0);
    uint32_t choDecisions = 0;

    // CHO exercised every 200 ms on the MEASURED serving SINR (UE 0); the
    // candidate is ephemeris-predicted off that measured baseline.
    rs.RegisterPeriodicCallback(MilliSeconds(200), [&](Time) {
        const double servSinr = rs.GetUeRecentSinrDb(0);
        if (std::isnan(servSinr))
        {
            return;
        }
        // CHO-23: candidate quality from the real slant geometry of the second
        // satellite, not a constant offset, and the UE position the geometric
        // triggers need (CHO-21).
        const Vector u = ueMob->GetPosition();
        const Vector sPos = servSatMob->GetPosition();
        const Vector cPos = candSatMob->GetPosition();
        const double servSlant = ntngeo::SlantRangeM(u, sPos);
        const double candSlant = ntngeo::SlantRangeM(u, cPos);
        const double servElev = ntngeo::ElevationDeg(u, sPos);
        const double candElev = ntngeo::ElevationDeg(u, cPos);
        const double candSinr =
            servSinr + 20.0 * std::log10(servSlant / std::max(1.0, candSlant));

        choAlgo->UpdateUeKinematics(GeoCoordinate(u), ueMob->GetVelocity());
        choAlgo->UpdateMeasurement(servingCell, servSinr, std::max(-20.0, (servElev - 45.0) / 5.0));
        choAlgo->UpdateMeasurement(candCell, candSinr, std::max(-20.0, (candElev - 45.0) / 5.0));
        choAlgo->UpdateCandidateSlantRange(servingCell, servSlant);
        choAlgo->UpdateCandidateSlantRange(candCell, candSlant);
        choAlgo->EvaluateConditions();
        if (choAlgo->SelectBestCandidate() != 0)
        {
            ++choDecisions;
        }
    });

    Simulator::Stop(Seconds(simTime));
    Simulator::Run();
    rs.Collect();
    rs.WriteHealthReport();

    std::cout << "  CHO trigger (" << triggerType << ") fired on " << choDecisions
              << " evaluations against real cell " << candCell << "\n"
              << "  measured mean SINR: " << rs.GetMeanDlSinrDb() << " dB\n"
              << "  measured throughput: " << rs.GetRxThroughputMbps() << " Mbps\n";
    Simulator::Destroy();
    return 0;
}
