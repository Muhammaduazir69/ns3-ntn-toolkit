/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-only
 * Author: Muhammad Uzair
 *
 * O-RAN NTN Full Scenario
 *
 * Demonstrates the complete O-RAN NTN framework with:
 *   - 66-satellite LEO Walker Star constellation (6 planes x 11 sats)
 *   - 5 terrestrial gNBs
 *   - 100 UEs (mixed mobility: static, pedestrian, vehicular)
 *   - Non-RT RIC with orbit-aware A1 policies
 *   - Near-RT RIC with 5 xApps running simultaneously
 *   - Space RICs on each satellite (autonomous mode demo)
 *   - Multi-xApp conflict resolution
 *   - Full KPM reporting and RC action pipeline
 *   - Comprehensive output: CSV datasets, metrics, conflict logs
 *
 * Usage:
 *   ./ns3 run "oran-ntn-full-scenario --duration=600 --numUes=100"
 */

#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"

#include "ns3/oran-ntn-a1-interface.h"
#include "ns3/oran-ntn-e2-interface.h"
#include "ns3/oran-ntn-helper.h"
#include "ns3/oran-ntn-near-rt-ric.h"
#include "ns3/oran-ntn-space-ric.h"
#include "ns3/oran-ntn-types.h"
#include "ns3/oran-ntn-xapp-beam-hop.h"
#include "ns3/oran-ntn-xapp-doppler-comp.h"
#include "ns3/oran-ntn-xapp-ho-predict.h"
#include "ns3/oran-ntn-xapp-slice-manager.h"
#include "ns3/oran-ntn-xapp-tn-ntn-steering.h"

#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("OranNtnFullScenario");

// ============================================================================
//  Simulation parameters
// ============================================================================

struct SimParams
{
    double duration = 600.0;       // seconds
    uint32_t numPlanes = 6;
    uint32_t satsPerPlane = 11;
    double altitudeKm = 550.0;
    double inclinationDeg = 53.0;
    uint32_t numTnGnbs = 5;
    uint32_t numUes = 100;
    double kpmInterval = 0.1;      // seconds
    std::string outputDir = "oran-ntn-output";
    std::string conflictStrategy = "priority";
    bool enableSpaceRic = true;
    bool enableFederatedLearning = false;
    uint32_t feederLinkOutageStart = 200;  // seconds
    uint32_t feederLinkOutageDuration = 30; // seconds
};

// ============================================================================
//  KPM Report Generator (synthetic for standalone demo)
// ============================================================================

class KpmGenerator
{
  public:
    KpmGenerator(Ptr<OranNtnHelper> helper, uint32_t numSats, uint32_t numTnGnbs,
                  uint32_t numUes)
        : m_helper(helper),
          m_numSats(numSats),
          m_numTnGnbs(numTnGnbs),
          m_numUes(numUes)
    {
    }

    void GenerateKpmReports()
    {
        double t = Simulator::Now().GetSeconds();

        for (uint32_t ue = 0; ue < m_numUes; ue++)
        {
            double phase = t * 0.01 + ue * 0.1;

            // Serving satellite changes over time (satellite pass simulation)
            uint32_t servingSat = (static_cast<uint32_t>(t / 30.0) + ue) % m_numSats + 1;

            // Serving satellite signal - SINR degrades over time within a pass
            double passProgress = std::fmod(t + ue * 3.0, 90.0) / 90.0; // 0->1 over 90s pass
            double elevation = 10.0 + 70.0 * std::sin(passProgress * M_PI); // peaks at mid-pass
            double sinr = -3.0 + elevation * 0.25 + 1.5 * std::sin(phase * 3.0);

            // SINR degrades significantly at end of pass to trigger handover
            if (passProgress > 0.7)
            {
                sinr -= (passProgress - 0.7) * 30.0; // Sharp drop at pass end
            }

            double rsrp = -125.0 + elevation * 0.6;
            double tte = std::max(2.0, (1.0 - passProgress) * 90.0);

            // Per-UE Doppler varies based on UE position offset
            double ueOffset = (ue % 10) * 0.01; // Different UE positions
            double doppler = 25000.0 * std::cos(phase * 0.1 + ueOffset) +
                             500.0 * std::sin(ue * 0.7 + t * 0.05); // Per-UE residual

            m_helper->InjectKpmReport(servingSat, ue, sinr, rsrp, tte, elevation, doppler);

            // Report from 1-2 neighbor satellites (candidates for handover)
            uint32_t neighbor1 = (servingSat % m_numSats) + 1;
            double nElev1 = std::max(5.0, elevation - 15.0 + 10.0 * std::sin(phase * 0.7));
            double nSinr1 = -2.0 + nElev1 * 0.25;
            double nTte1 = std::max(10.0, 80.0 - std::fmod(t + 10.0, 90.0));
            double nDoppler1 = 24000.0 * std::cos(phase * 0.11 + ueOffset);
            m_helper->InjectKpmReport(neighbor1, ue, nSinr1, -120.0 + nElev1 * 0.5,
                                       nTte1, nElev1, nDoppler1);

            if (ue % 3 == 0)
            {
                uint32_t neighbor2 = ((servingSat + 1) % m_numSats) + 1;
                double nElev2 = std::max(5.0, elevation - 25.0 + 8.0 * std::cos(phase * 0.9));
                double nSinr2 = -4.0 + nElev2 * 0.28;
                double nTte2 = std::max(15.0, 70.0 - std::fmod(t + 25.0, 90.0));
                m_helper->InjectKpmReport(neighbor2, ue, nSinr2, -122.0 + nElev2 * 0.5,
                                           nTte2, nElev2, 23000.0 * std::cos(phase * 0.12));
            }

            // Half the UEs also see a terrestrial gNB (for TN-NTN steering)
            if (ue % 2 == 0 && m_numTnGnbs > 0)
            {
                uint32_t tnGnb = 10001 + (ue % m_numTnGnbs);
                // TN signal varies - sometimes better, sometimes worse than NTN
                double tnSinr = 8.0 + 12.0 * std::sin(phase * 0.3 + ue * 0.05);
                double tnRsrp = -75.0 + 10.0 * std::sin(phase * 0.2);
                m_helper->InjectKpmReport(tnGnb, ue, tnSinr, tnRsrp, 999.0, 90.0, 0.0);
            }
        }

        // Re-schedule
        Simulator::Schedule(MilliSeconds(100), &KpmGenerator::GenerateKpmReports, this);
    }

  private:
    Ptr<OranNtnHelper> m_helper;
    uint32_t m_numSats;
    uint32_t m_numTnGnbs;
    uint32_t m_numUes;
};

// ============================================================================
//  Feeder Link Outage Simulator
// ============================================================================

void
SimulateFeederLinkOutage(std::vector<Ptr<OranNtnE2Node>>& satE2Nodes,
                           std::vector<Ptr<OranNtnSpaceRic>>& spaceRics,
                           uint32_t startSat, uint32_t endSat)
{
    NS_LOG_INFO("=== FEEDER LINK OUTAGE: Satellites " << startSat << "-" << endSat
                << " losing feeder link ===");

    for (uint32_t i = startSat; i <= endSat && i < satE2Nodes.size(); i++)
    {
        satE2Nodes[i]->SetFeederLinkAvailable(false);
        if (i < spaceRics.size())
        {
            spaceRics[i]->EnterAutonomousMode();
        }
    }
}

void
RestoreFeederLink(std::vector<Ptr<OranNtnE2Node>>& satE2Nodes,
                   std::vector<Ptr<OranNtnSpaceRic>>& spaceRics,
                   uint32_t startSat, uint32_t endSat)
{
    NS_LOG_INFO("=== FEEDER LINK RESTORED: Satellites " << startSat << "-" << endSat
                << " ===");

    for (uint32_t i = startSat; i <= endSat && i < satE2Nodes.size(); i++)
    {
        satE2Nodes[i]->SetFeederLinkAvailable(true);
        if (i < spaceRics.size())
        {
            spaceRics[i]->ExitAutonomousMode();
        }
    }
}

// ============================================================================
//  Main
// ============================================================================

int
main(int argc, char* argv[])
{
    SimParams params;

    CommandLine cmd(__FILE__);
    cmd.AddValue("duration", "Simulation duration (s)", params.duration);
    cmd.AddValue("numPlanes", "Number of orbital planes", params.numPlanes);
    cmd.AddValue("satsPerPlane", "Satellites per plane", params.satsPerPlane);
    cmd.AddValue("altitude", "Satellite altitude (km)", params.altitudeKm);
    cmd.AddValue("inclination", "Orbital inclination (deg)", params.inclinationDeg);
    cmd.AddValue("numTnGnbs", "Number of terrestrial gNBs", params.numTnGnbs);
    cmd.AddValue("numUes", "Number of UEs", params.numUes);
    cmd.AddValue("kpmInterval", "KPM reporting interval (s)", params.kpmInterval);
    cmd.AddValue("outputDir", "Output directory", params.outputDir);
    cmd.AddValue("conflictStrategy", "Conflict resolution: priority, temporal, merge",
                 params.conflictStrategy);
    cmd.AddValue("enableSpaceRic", "Enable Space RICs", params.enableSpaceRic);
    cmd.AddValue("enableFL", "Enable federated learning", params.enableFederatedLearning);
    cmd.Parse(argc, argv);

    uint32_t totalSats = params.numPlanes * params.satsPerPlane;

    std::cout << "\n"
              << "============================================================\n"
              << "  O-RAN NTN Full Scenario Simulation\n"
              << "============================================================\n"
              << "  Constellation: " << params.numPlanes << " planes x "
              << params.satsPerPlane << " sats = " << totalSats << " satellites\n"
              << "  Altitude: " << params.altitudeKm << " km, Inclination: "
              << params.inclinationDeg << " deg\n"
              << "  Terrestrial gNBs: " << params.numTnGnbs << "\n"
              << "  UEs: " << params.numUes << "\n"
              << "  Duration: " << params.duration << " s\n"
              << "  Space RIC: " << (params.enableSpaceRic ? "ON" : "OFF") << "\n"
              << "  Federated Learning: "
              << (params.enableFederatedLearning ? "ON" : "OFF") << "\n"
              << "  Conflict Strategy: " << params.conflictStrategy << "\n"
              << "  Output: " << params.outputDir << "\n"
              << "============================================================\n\n";

    // ---- Create helper ----
    auto helper = CreateObject<OranNtnHelper>();
    helper->SetOutputDirectory(params.outputDir);

    // ---- Create nodes (simplified - no actual ns3 satellite module needed) ----
    NodeContainer satNodes;
    satNodes.Create(totalSats);

    NodeContainer gnbNodes;
    gnbNodes.Create(params.numTnGnbs);

    NodeContainer ueNodes;
    ueNodes.Create(params.numUes);

    // ---- Create O-RAN architecture ----
    std::cout << "[1/7] Creating Non-RT RIC..." << std::endl;
    auto nonRtRic = helper->CreateNonRtRic();

    std::cout << "[2/7] Creating Near-RT RIC..." << std::endl;
    auto nearRtRic = helper->CreateNearRtRic();

    // Connect A1 interface
    helper->ConnectA1Interface(nearRtRic, nonRtRic);

    // Set conflict resolution strategy
    auto cm = nearRtRic->GetConflictManager();
    if (params.conflictStrategy == "temporal")
    {
        cm->SetResolutionStrategy(ConflictResolutionStrategy::TEMPORAL);
    }
    else if (params.conflictStrategy == "merge")
    {
        cm->SetResolutionStrategy(ConflictResolutionStrategy::MERGE);
    }
    else
    {
        cm->SetResolutionStrategy(ConflictResolutionStrategy::PRIORITY_BASED);
    }

    // ---- Create E2 nodes ----
    std::cout << "[3/7] Creating " << totalSats << " satellite E2 nodes + "
              << params.numTnGnbs << " terrestrial E2 nodes..." << std::endl;
    auto satE2Nodes = helper->CreateSatelliteE2Nodes(satNodes, nearRtRic);
    auto tnE2Nodes = helper->CreateTerrestrialE2Nodes(gnbNodes, nearRtRic);

    // ---- Create Space RICs ----
    std::vector<Ptr<OranNtnSpaceRic>> spaceRics;
    if (params.enableSpaceRic)
    {
        std::cout << "[4/7] Creating " << totalSats << " Space RICs..." << std::endl;
        spaceRics = helper->CreateSpaceRics(satNodes, params.numPlanes,
                                              params.satsPerPlane, nearRtRic);
    }
    else
    {
        std::cout << "[4/7] Space RICs disabled." << std::endl;
    }

    // ---- Generate A1 policies ----
    std::cout << "[5/7] Generating constellation A1 policies..." << std::endl;
    helper->GenerateConstellationPolicies(nonRtRic, params.numPlanes,
                                           params.satsPerPlane,
                                           params.inclinationDeg,
                                           params.altitudeKm);

    // ---- Create and start xApps ----
    std::cout << "[6/7] Creating and starting 5 xApps..." << std::endl;
    auto xapps = helper->CreateAllXapps(nearRtRic);
    helper->StartAllXapps(nearRtRic);

    // Print xApp info
    for (const auto& [name, xapp] : xapps)
    {
        std::cout << "  xApp: " << name << " (id=" << xapp->GetXappId()
                  << ", priority=" << (int)xapp->GetPriority()
                  << ", interval=" << xapp->GetDecisionInterval().As(Time::MS)
                  << ")" << std::endl;
    }

    // ---- Start KPM feed ----
    std::cout << "[7/7] Starting KPM feed simulation..." << std::endl;
    KpmGenerator kpmGen(helper, totalSats, params.numTnGnbs, params.numUes);
    Simulator::Schedule(Seconds(1.0), &KpmGenerator::GenerateKpmReports, &kpmGen);

    // ---- Schedule feeder link outage event ----
    if (params.enableSpaceRic)
    {
        // Outage for plane 0 satellites (satellites 0-10) at t=200s for 30s
        Simulator::Schedule(
            Seconds(params.feederLinkOutageStart),
            &SimulateFeederLinkOutage,
            std::ref(satE2Nodes), std::ref(spaceRics),
            (uint32_t)0, params.satsPerPlane - 1);

        Simulator::Schedule(
            Seconds(params.feederLinkOutageStart + params.feederLinkOutageDuration),
            &RestoreFeederLink,
            std::ref(satE2Nodes), std::ref(spaceRics),
            (uint32_t)0, params.satsPerPlane - 1);
    }

    // ---- Run simulation ----
    std::cout << "\nSimulation running..." << std::endl;
    Simulator::Stop(Seconds(params.duration));
    Simulator::Run();

    // ---- Collect and write results ----
    std::cout << "\n============================================================\n"
              << "  SIMULATION COMPLETE - Writing Results\n"
              << "============================================================\n";

    helper->WriteAllMetrics(nearRtRic);

    // Print summary
    auto ricMetrics = nearRtRic->GetMetrics();
    std::cout << "\n--- RIC Summary ---\n"
              << "  Active xApps: " << ricMetrics.activeXapps << "\n"
              << "  E2 Nodes: " << ricMetrics.totalE2Nodes << "\n"
              << "  Actions Processed: " << ricMetrics.totalActionsProcessed << "\n"
              << "  Conflicts: " << ricMetrics.totalConflicts << "\n"
              << "  Policy Violations: " << ricMetrics.totalPolicyViolations << "\n";

    std::cout << "\n--- Per-xApp Summary ---\n";
    for (const auto& [name, xapp] : xapps)
    {
        auto m = xapp->GetMetrics();
        std::cout << "  " << std::left << std::setw(18) << name
                  << " | decisions: " << std::setw(6) << m.totalDecisions
                  << " | actions: " << std::setw(5) << m.successfulActions << "/"
                  << (m.successfulActions + m.failedActions)
                  << " | conflicts: " << m.conflictsEncountered
                  << " | confidence: " << std::fixed << std::setprecision(3)
                  << m.avgConfidence << "\n";
    }

    if (params.enableSpaceRic)
    {
        uint32_t totalAutonomous = 0;
        double totalAutoTime = 0;
        for (const auto& sric : spaceRics)
        {
            auto sm = sric->GetMetrics();
            totalAutonomous += sm.totalAutonomousDecisions;
            totalAutoTime += sm.totalAutonomousTime.GetSeconds();
        }
        std::cout << "\n--- Space RIC Summary ---\n"
                  << "  Total autonomous decisions: " << totalAutonomous << "\n"
                  << "  Total autonomous time: " << totalAutoTime << " s\n";
    }

    std::cout << "\nResults written to: " << params.outputDir << "/\n" << std::endl;

    // Cleanup
    Simulator::Destroy();
    return 0;
}
