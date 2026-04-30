/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-only
 * Author: Muhammad Uzair
 */

#include "oran-ntn-helper.h"

#include "ns3/log.h"
#include "ns3/node.h"
#include "ns3/simulator.h"
#include "ns3/string.h"

#include "ns3/oran-ntn-a1-interface.h"
#include "ns3/oran-ntn-e2-interface.h"
#include "ns3/oran-ntn-near-rt-ric.h"
#include "ns3/oran-ntn-space-ric.h"
#include "ns3/oran-ntn-xapp-beam-hop.h"
#include "ns3/oran-ntn-xapp-doppler-comp.h"
#include "ns3/oran-ntn-xapp-ho-predict.h"
#include "ns3/oran-ntn-xapp-slice-manager.h"
#include "ns3/oran-ntn-xapp-tn-ntn-steering.h"

#include <fstream>
#include <sys/stat.h>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("OranNtnHelper");
NS_OBJECT_ENSURE_REGISTERED(OranNtnHelper);

TypeId
OranNtnHelper::GetTypeId()
{
    static TypeId tid = TypeId("ns3::OranNtnHelper")
                            .SetParent<Object>()
                            .SetGroupName("OranNtn")
                            .AddConstructor<OranNtnHelper>()
                            .AddAttribute("OutputDirectory",
                                          "Directory for output files",
                                          StringValue("oran-ntn-output"),
                                          MakeStringAccessor(&OranNtnHelper::m_outputDir),
                                          MakeStringChecker());
    return tid;
}

OranNtnHelper::OranNtnHelper()
    : m_outputDir("oran-ntn-output")
{
    NS_LOG_FUNCTION(this);
}

OranNtnHelper::~OranNtnHelper()
{
    NS_LOG_FUNCTION(this);
}

void
OranNtnHelper::DoDispose()
{
    Simulator::Cancel(m_kpmFeed.feedEvent);
    m_e2Nodes.clear();
    m_spaceRics.clear();
    Object::DoDispose();
}

// ---- Core setup ----

Ptr<OranNtnNearRtRic>
OranNtnHelper::CreateNearRtRic()
{
    NS_LOG_FUNCTION(this);
    auto ric = CreateObject<OranNtnNearRtRic>();
    ric->Initialize();
    NS_LOG_INFO("OranNtnHelper: Created Near-RT RIC");
    return ric;
}

Ptr<OranNtnA1PolicyManager>
OranNtnHelper::CreateNonRtRic()
{
    NS_LOG_FUNCTION(this);
    auto nonRtRic = CreateObject<OranNtnA1PolicyManager>();
    NS_LOG_INFO("OranNtnHelper: Created Non-RT RIC (A1 Policy Manager)");
    return nonRtRic;
}

void
OranNtnHelper::ConnectA1Interface(Ptr<OranNtnNearRtRic> nearRtRic,
                                    Ptr<OranNtnA1PolicyManager> nonRtRic)
{
    NS_LOG_FUNCTION(this);

    // Non-RT RIC distributes policies to Near-RT RIC's A1 adapter
    auto a1Adapter = nearRtRic->GetA1Adapter();
    nonRtRic->SetDistributionCallback(
        MakeCallback(&OranNtnA1Adapter::HandleIncomingPolicy, a1Adapter));

    // Near-RT RIC sends enforcement feedback back to Non-RT RIC
    a1Adapter->SetFeedbackCallback(
        MakeCallback(&OranNtnA1PolicyManager::HandlePolicyFeedback, nonRtRic));

    NS_LOG_INFO("OranNtnHelper: Connected A1 interface between Non-RT RIC and Near-RT RIC");
}

// ---- E2 node setup ----

std::vector<Ptr<OranNtnE2Node>>
OranNtnHelper::CreateSatelliteE2Nodes(NodeContainer satNodes,
                                        Ptr<OranNtnNearRtRic> ric)
{
    NS_LOG_FUNCTION(this << satNodes.GetN());
    std::vector<Ptr<OranNtnE2Node>> nodes;

    for (uint32_t i = 0; i < satNodes.GetN(); i++)
    {
        auto e2node = CreateObject<OranNtnE2Node>();
        e2node->SetNodeId(i + 1); // 1-indexed
        e2node->SetIsNtn(true);

        // Estimate feeder link delay from LEO altitude (~550 km -> ~3.7 ms round trip)
        e2node->SetFeederLinkDelay(MilliSeconds(4));
        e2node->SetOnBoardBufferSize(500);

        // Register KPM and RC service models
        e2node->RegisterRanFunction(2, "E2SM-KPM-NTN");
        e2node->RegisterRanFunction(3, "E2SM-RC-NTN");

        // Default RC action handler: accept all actions (simulation stub)
        e2node->SetRcActionCallback(
            MakeCallback(&OranNtnHelper::DefaultRcActionHandler, this));

        ric->ConnectE2Node(e2node);
        m_e2Nodes[i + 1] = e2node;
        nodes.push_back(e2node);
    }

    NS_LOG_INFO("OranNtnHelper: Created " << nodes.size() << " satellite E2 nodes");
    return nodes;
}

std::vector<Ptr<OranNtnE2Node>>
OranNtnHelper::CreateTerrestrialE2Nodes(NodeContainer gnbNodes,
                                          Ptr<OranNtnNearRtRic> ric)
{
    NS_LOG_FUNCTION(this << gnbNodes.GetN());
    std::vector<Ptr<OranNtnE2Node>> nodes;

    uint32_t baseId = 10000; // Offset to avoid collision with satellite IDs
    for (uint32_t i = 0; i < gnbNodes.GetN(); i++)
    {
        auto e2node = CreateObject<OranNtnE2Node>();
        e2node->SetNodeId(baseId + i + 1);
        e2node->SetIsNtn(false);
        e2node->SetFeederLinkDelay(MilliSeconds(1)); // Fiber backhaul
        e2node->RegisterRanFunction(2, "E2SM-KPM");
        e2node->RegisterRanFunction(3, "E2SM-RC");

        e2node->SetRcActionCallback(
            MakeCallback(&OranNtnHelper::DefaultRcActionHandler, this));

        ric->ConnectE2Node(e2node);
        m_e2Nodes[baseId + i + 1] = e2node;
        nodes.push_back(e2node);
    }

    NS_LOG_INFO("OranNtnHelper: Created " << nodes.size() << " terrestrial E2 nodes");
    return nodes;
}

// ---- Space RIC setup ----

std::vector<Ptr<OranNtnSpaceRic>>
OranNtnHelper::CreateSpaceRics(NodeContainer satNodes,
                                 uint32_t numPlanes, uint32_t satsPerPlane,
                                 Ptr<OranNtnNearRtRic> groundRic)
{
    NS_LOG_FUNCTION(this << satNodes.GetN() << numPlanes << satsPerPlane);
    std::vector<Ptr<OranNtnSpaceRic>> spaceRics;

    for (uint32_t i = 0; i < satNodes.GetN(); i++)
    {
        auto spaceRic = CreateObject<OranNtnSpaceRic>();
        spaceRic->SetSatelliteId(i + 1);
        spaceRic->SetOrbitalPlaneId(i / satsPerPlane);
        spaceRic->SetGroundRic(groundRic);

        // Initialize with default HO scoring model
        std::vector<double> hoWeights = {2.0, 0.5, 0.1}; // TTE, SINR, elevation
        spaceRic->ReceiveModelUpdate("ho-scorer", 1, hoWeights);

        spaceRics.push_back(spaceRic);
    }

    // Connect ISL neighbors (intra-plane: adjacent sats; inter-plane: same index)
    for (uint32_t i = 0; i < spaceRics.size(); i++)
    {
        uint32_t plane = i / satsPerPlane;
        uint32_t posInPlane = i % satsPerPlane;

        // Intra-plane neighbor (next satellite in same plane)
        uint32_t nextInPlane = plane * satsPerPlane + ((posInPlane + 1) % satsPerPlane);
        if (nextInPlane < spaceRics.size())
        {
            // ISL neighbors stored internally by Space RIC
        }

        // Inter-plane neighbor (same position in adjacent plane)
        if (plane + 1 < numPlanes)
        {
            uint32_t interPlaneIdx = (plane + 1) * satsPerPlane + posInPlane;
            if (interPlaneIdx < spaceRics.size())
            {
                // ISL neighbors stored internally by Space RIC
            }
        }
    }

    m_spaceRics = spaceRics;
    NS_LOG_INFO("OranNtnHelper: Created " << spaceRics.size()
                << " Space RICs across " << numPlanes << " orbital planes");
    return spaceRics;
}

// ---- xApp instantiation ----

std::map<std::string, Ptr<OranNtnXappBase>>
OranNtnHelper::CreateAllXapps(Ptr<OranNtnNearRtRic> ric)
{
    NS_LOG_FUNCTION(this);
    std::map<std::string, Ptr<OranNtnXappBase>> xapps;

    // 1. HO Prediction xApp (highest priority for safety-critical HOs)
    auto hoPredict = CreateObject<OranNtnXappHoPredict>();
    hoPredict->SetXappName("ho-predict");
    hoPredict->SetPriority(10); // High priority
    hoPredict->SetDecisionInterval(MilliSeconds(100));
    ric->RegisterXapp(hoPredict);
    xapps["ho-predict"] = hoPredict;

    // 2. Beam Hopping xApp
    auto beamHop = CreateObject<OranNtnXappBeamHop>();
    beamHop->SetXappName("beam-hop");
    beamHop->SetPriority(20);
    beamHop->SetDecisionInterval(MilliSeconds(200));
    ric->RegisterXapp(beamHop);
    xapps["beam-hop"] = beamHop;

    // 3. Slice Manager xApp
    auto sliceMgr = CreateObject<OranNtnXappSliceManager>();
    sliceMgr->SetXappName("slice-manager");
    sliceMgr->SetPriority(30);
    sliceMgr->SetDecisionInterval(MilliSeconds(500));
    ric->RegisterXapp(sliceMgr);
    xapps["slice-manager"] = sliceMgr;

    // 4. Doppler Compensation xApp
    auto dopplerComp = CreateObject<OranNtnXappDopplerComp>();
    dopplerComp->SetXappName("doppler-comp");
    dopplerComp->SetPriority(15);
    dopplerComp->SetDecisionInterval(MilliSeconds(200));
    ric->RegisterXapp(dopplerComp);
    xapps["doppler-comp"] = dopplerComp;

    // 5. TN-NTN Traffic Steering xApp
    auto tnNtnSteering = CreateObject<OranNtnXappTnNtnSteering>();
    tnNtnSteering->SetXappName("tn-ntn-steering");
    tnNtnSteering->SetPriority(25);
    tnNtnSteering->SetDecisionInterval(MilliSeconds(500));
    ric->RegisterXapp(tnNtnSteering);
    xapps["tn-ntn-steering"] = tnNtnSteering;

    NS_LOG_INFO("OranNtnHelper: Created and registered 5 xApps");
    return xapps;
}

void
OranNtnHelper::StartAllXapps(Ptr<OranNtnNearRtRic> ric)
{
    NS_LOG_FUNCTION(this);
    auto ids = ric->GetRegisteredXappIds();
    for (uint32_t id : ids)
    {
        auto xapp = ric->GetXapp(id);
        if (xapp)
        {
            xapp->Start();
        }
    }
    NS_LOG_INFO("OranNtnHelper: Started " << ids.size() << " xApps");
}

// ---- Policy generation ----

void
OranNtnHelper::GenerateConstellationPolicies(Ptr<OranNtnA1PolicyManager> nonRtRic,
                                               uint32_t numPlanes,
                                               uint32_t satsPerPlane,
                                               double inclinationDeg,
                                               double altitudeKm)
{
    NS_LOG_FUNCTION(this);

    // Generate orbit-aware HO threshold policies
    nonRtRic->GenerateOrbitAwarePolicies(numPlanes, satsPerPlane,
                                          inclinationDeg, altitudeKm);

    // Generate default slice SLA policies
    std::vector<SliceConfig> slices;

    SliceConfig embb;
    embb.sliceId = 0;
    embb.name = "eMBB";
    embb.minThroughput_Mbps = 50.0;
    embb.maxLatency_ms = 100.0;
    embb.reliabilityTarget = 0.999;
    embb.prbShare = 0.5;
    embb.harqEnabled = true;
    embb.priority = 2;
    slices.push_back(embb);

    SliceConfig urllc;
    urllc.sliceId = 1;
    urllc.name = "URLLC";
    urllc.minThroughput_Mbps = 1.0;
    urllc.maxLatency_ms = 1.0;
    urllc.reliabilityTarget = 0.99999;
    urllc.prbShare = 0.3;
    urllc.harqEnabled = true;
    urllc.priority = 1;
    slices.push_back(urllc);

    SliceConfig mmtc;
    mmtc.sliceId = 2;
    mmtc.name = "mMTC";
    mmtc.minThroughput_Mbps = 0.1;
    mmtc.maxLatency_ms = 10000.0;
    mmtc.reliabilityTarget = 0.99;
    mmtc.prbShare = 0.2;
    mmtc.harqEnabled = false;
    mmtc.priority = 3;
    slices.push_back(mmtc);

    nonRtRic->GenerateSlicePolicies(slices);

    NS_LOG_INFO("OranNtnHelper: Generated constellation policies for "
                << numPlanes << "x" << satsPerPlane << " constellation at "
                << altitudeKm << " km");
}

// ---- KPM feed ----

void
OranNtnHelper::StartKpmFeed(Ptr<OranNtnNearRtRic> ric, Time reportInterval)
{
    NS_LOG_FUNCTION(this << reportInterval.As(Time::MS));
    m_kpmFeed.interval = reportInterval;
    m_kpmFeed.feedEvent = Simulator::Schedule(reportInterval,
                                               &OranNtnHelper::KpmFeedCallback,
                                               this);
}

void
OranNtnHelper::KpmFeedCallback()
{
    // This would be connected to actual PHY measurements in a full integration.
    // For standalone oran-ntn testing, it generates synthetic KPM reports.

    m_kpmFeed.feedEvent = Simulator::Schedule(m_kpmFeed.interval,
                                               &OranNtnHelper::KpmFeedCallback,
                                               this);
}

void
OranNtnHelper::InjectKpmReport(uint32_t gnbId, uint32_t ueId,
                                 double sinr, double rsrp, double tte,
                                 double elevation, double doppler)
{
    auto it = m_e2Nodes.find(gnbId);
    if (it == m_e2Nodes.end())
    {
        NS_LOG_WARN("OranNtnHelper: E2 node " << gnbId << " not found for KPM injection");
        return;
    }

    E2KpmReport report;
    report.timestamp = Simulator::Now().GetSeconds();
    report.gnbId = gnbId;
    report.isNtn = it->second->IsNtn();
    report.ueId = ueId;
    report.sinr_dB = sinr;
    report.rsrp_dBm = rsrp;
    report.tte_s = tte;
    report.elevation_deg = elevation;
    report.doppler_Hz = doppler;
    report.rsrq_dB = sinr - 3.0; // Simplified
    report.cqi = static_cast<uint8_t>(std::max(0.0, std::min(15.0, sinr + 6.0)));
    report.throughput_Mbps = std::max(0.0, 10.0 * (1.0 + sinr / 30.0));
    report.latency_ms = it->second->IsNtn() ? 20.0 : 5.0;
    report.propagationDelay_ms = it->second->IsNtn() ? 3.7 : 0.1;
    report.beamId = 1;
    report.beamGain_dB = elevation * 0.3; // Simplified
    report.prbUtilization = 0.5;
    report.activeUes = 10;
    report.cellThroughput_Mbps = 100.0;
    // Assign slice based on UE modular grouping: eMBB=0, URLLC=1, mMTC=2
    report.sliceId = static_cast<uint8_t>(ueId % 3);
    // Slice-aware throughput and latency
    if (report.sliceId == 0) // eMBB
    {
        report.sliceThroughput_Mbps = report.throughput_Mbps;
        report.sliceLatency_ms = report.latency_ms;
        report.sliceReliability = 0.999;
    }
    else if (report.sliceId == 1) // URLLC
    {
        report.sliceThroughput_Mbps = std::max(0.1, report.throughput_Mbps * 0.1);
        report.sliceLatency_ms = report.latency_ms; // NTN latency violates URLLC SLA
        report.sliceReliability = 0.9999;
    }
    else // mMTC
    {
        report.sliceThroughput_Mbps = std::max(0.01, report.throughput_Mbps * 0.02);
        report.sliceLatency_ms = report.latency_ms * 2.0;
        report.sliceReliability = 0.99;
    }

    it->second->SubmitKpmMeasurement(report);
    m_kpmDataset.push_back(report);
}

bool
OranNtnHelper::DefaultRcActionHandler(E2RcAction action)
{
    NS_LOG_INFO("OranNtnHelper: RC action executed - type="
                << static_cast<uint8_t>(action.actionType)
                << " xApp=" << action.xappName
                << " targetGnb=" << action.targetGnbId
                << " targetUe=" << action.targetUeId
                << " confidence=" << action.confidence);

    // Log the action
    ActionLogEntry entry;
    entry.timestamp = action.timestamp;
    entry.xappId = action.xappId;
    entry.xappName = action.xappName;
    entry.actionType = static_cast<uint8_t>(action.actionType);
    entry.targetGnb = action.targetGnbId;
    entry.targetUe = action.targetUeId;
    entry.confidence = action.confidence;
    entry.success = true;
    m_actionLog.push_back(entry);

    return true; // Accept all actions in simulation
}

// ---- Output ----

void
OranNtnHelper::SetOutputDirectory(const std::string& dir)
{
    m_outputDir = dir;
    mkdir(dir.c_str(), 0755);
}

void
OranNtnHelper::WriteAllMetrics(Ptr<OranNtnNearRtRic> ric) const
{
    NS_LOG_FUNCTION(this);
    mkdir(m_outputDir.c_str(), 0755);

    // Write RIC metrics
    ric->WriteMetrics(m_outputDir + "/ric_metrics.txt");

    // Write conflict log
    ric->GetConflictManager()->WriteConflictLog(m_outputDir + "/conflict_log.csv");

    // Write action log
    WriteActionLog(m_outputDir + "/action_log.csv");

    // Write KPM dataset
    WriteKpmDataset(m_outputDir + "/kpm_dataset.csv");

    // Write per-xApp metrics
    std::ofstream xappOfs(m_outputDir + "/xapp_metrics.csv");
    xappOfs << "xapp_id,xapp_name,priority,decisions,successful_actions,"
               "failed_actions,conflicts,conflicts_won,avg_confidence,"
               "avg_latency_ms\n";

    auto xappIds = ric->GetRegisteredXappIds();
    for (uint32_t id : xappIds)
    {
        auto xapp = ric->GetXapp(id);
        auto m = xapp->GetMetrics();
        xappOfs << id << "," << xapp->GetXappName() << ","
                << (int)xapp->GetPriority() << "," << m.totalDecisions << ","
                << m.successfulActions << "," << m.failedActions << ","
                << m.conflictsEncountered << "," << m.conflictsWon << ","
                << m.avgConfidence << "," << m.avgDecisionLatency_ms << "\n";
    }

    // Write Space RIC metrics
    if (!m_spaceRics.empty())
    {
        std::ofstream sricOfs(m_outputDir + "/space_ric_metrics.csv");
        sricOfs << "sat_id,plane_id,autonomous_decisions,ground_assisted,"
                   "autonomous_time_s,isl_exchanges,model_syncs,"
                   "handovers,beam_reallocs,avg_confidence\n";

        for (const auto& sric : m_spaceRics)
        {
            auto m = sric->GetMetrics();
            sricOfs << sric->GetSatelliteId() << ","
                    << sric->GetOrbitalPlaneId() << ","
                    << m.totalAutonomousDecisions << ","
                    << m.totalGroundAssistedDecisions << ","
                    << m.totalAutonomousTime.GetSeconds() << ","
                    << m.islExchanges << "," << m.modelSyncs << ","
                    << m.handoversInitiated << "," << m.beamReallocations << ","
                    << m.avgDecisionConfidence << "\n";
        }
    }

    NS_LOG_INFO("OranNtnHelper: Wrote all metrics to " << m_outputDir);
}

void
OranNtnHelper::WriteActionLog(const std::string& filename) const
{
    std::ofstream ofs(filename);
    ofs << "timestamp,xapp_id,xapp_name,action_type,target_gnb,"
           "target_ue,confidence,success\n";

    for (const auto& entry : m_actionLog)
    {
        ofs << entry.timestamp << "," << entry.xappId << ","
            << entry.xappName << "," << (int)entry.actionType << ","
            << entry.targetGnb << "," << entry.targetUe << ","
            << entry.confidence << "," << (entry.success ? 1 : 0) << "\n";
    }
}

void
OranNtnHelper::WriteConflictLog(Ptr<OranNtnNearRtRic> ric,
                                  const std::string& filename) const
{
    ric->GetConflictManager()->WriteConflictLog(filename);
}

void
OranNtnHelper::WriteKpmDataset(const std::string& filename) const
{
    std::ofstream ofs(filename);
    ofs << "timestamp,gnb_id,is_ntn,ue_id,rsrp_dBm,rsrq_dB,sinr_dB,cqi,"
           "throughput_Mbps,latency_ms,elevation_deg,doppler_Hz,"
           "propagation_delay_ms,tte_s,beam_id,beam_gain_dB,"
           "prb_utilization,active_ues,cell_throughput_Mbps,"
           "slice_id,slice_throughput_Mbps,slice_latency_ms,slice_reliability\n";

    for (const auto& r : m_kpmDataset)
    {
        ofs << r.timestamp << "," << r.gnbId << "," << (r.isNtn ? 1 : 0) << ","
            << r.ueId << "," << r.rsrp_dBm << "," << r.rsrq_dB << ","
            << r.sinr_dB << "," << (int)r.cqi << "," << r.throughput_Mbps << ","
            << r.latency_ms << "," << r.elevation_deg << "," << r.doppler_Hz << ","
            << r.propagationDelay_ms << "," << r.tte_s << "," << r.beamId << ","
            << r.beamGain_dB << "," << r.prbUtilization << "," << r.activeUes << ","
            << r.cellThroughput_Mbps << "," << r.sliceId << ","
            << r.sliceThroughput_Mbps << "," << r.sliceLatency_ms << ","
            << r.sliceReliability << "\n";
    }

    NS_LOG_INFO("OranNtnHelper: Wrote " << m_kpmDataset.size()
                << " KPM records to " << filename);
}

} // namespace ns3
