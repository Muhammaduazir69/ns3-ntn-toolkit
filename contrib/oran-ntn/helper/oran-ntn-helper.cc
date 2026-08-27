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
#include "ns3/oran-ntn-kpm-canonical-ids.h"
#include "ns3/oran-ntn-near-rt-ric.h"
#include "ns3/oran-ntn-space-ric.h"
#include "ns3/oran-ntn-xapp-beam-hop.h"
#include "ns3/oran-ntn-xapp-doppler-comp.h"
#include "ns3/oran-ntn-xapp-energy-harvest.h"
#include "ns3/oran-ntn-xapp-ho-predict.h"
#include "ns3/oran-ntn-xapp-interference-mgmt.h"
#include "ns3/oran-ntn-xapp-isac.h"
#include "ns3/oran-ntn-xapp-multi-conn.h"
#include "ns3/oran-ntn-xapp-predictive-alloc.h"
#include "ns3/oran-ntn-xapp-slice-manager.h"
#include "ns3/oran-ntn-xapp-thz-beam-mgmt.h"
#include "ns3/oran-ntn-xapp-thz-ris.h"
#include "ns3/oran-ntn-xapp-thz-spectrum.h"
#include "ns3/oran-ntn-xapp-tn-ntn-steering.h"

#include "ns3/ntn-static-extra-loss-model.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sys/stat.h>
#include <vector>

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

        // ORAN-02 FIX (2026-08-24): give the space RIC the E2 node it is
        // co-located with. Without it ExecuteDecisionLocally() returns false on
        // every call and logs "autonomous decision NOT actuated", while the
        // autonomy counters incremented regardless, so a satellite-hosted RIC
        // reported autonomous decisions it never carried out. SetLocalE2Node
        // had no caller anywhere in the tree; the helper builds both objects
        // and simply never connected them. Satellite ids are 1-indexed in both
        // CreateSatelliteE2Nodes() and here, so the lookup is exact.
        auto e2it = m_e2Nodes.find(i + 1);
        if (e2it != m_e2Nodes.end())
        {
            spaceRic->SetLocalE2Node(e2it->second);
        }
        else
        {
            NS_LOG_WARN("Space RIC for satellite "
                        << (i + 1)
                        << " has no co-located E2 node: call CreateSatelliteE2Nodes() BEFORE "
                           "CreateSpaceRics(), otherwise on-board decisions cannot actuate "
                           "and will be counted as buffered only.");
        }

        // Initialize with default HO scoring model
        std::vector<double> hoWeights = {2.0, 0.5, 0.1}; // TTE, SINR, elevation
        spaceRic->ReceiveModelUpdate("ho-scorer", 1, hoWeights);

        spaceRics.push_back(spaceRic);
    }

    // Connect ISL neighbours (intra-plane: adjacent sats; inter-plane: same index).
    //
    // ORAN-16: this loop computed nextInPlane and interPlaneIdx and then entered
    // two if-blocks whose entire bodies were the comment "ISL neighbors stored
    // internally by Space RIC". Nothing was stored. AddIslNeighbor() was called
    // only from a unit test, so m_islNeighbors was empty for every
    // helper-built Space RIC and the whole ISL path - IslExchangeState() and
    // SendIslMessage(), which does apply a real per-link delay from
    // OranNtnSatBridge::GetIslLinkState - never fired in any shipped scenario.
    // A Walker shell of on-board RICs was, in every run, a set of isolated
    // nodes.
    //
    // Links are made in BOTH directions: an ISL is bidirectional, and a
    // one-way neighbour list would let a satellite send where it cannot
    // receive.
    uint32_t islLinks = 0;
    for (uint32_t i = 0; i < spaceRics.size(); i++)
    {
        const uint32_t plane = i / satsPerPlane;
        const uint32_t posInPlane = i % satsPerPlane;

        // Intra-plane neighbour: the next satellite in the same plane, wrapping
        // at the end of the ring. Skip the degenerate case of a single-satellite
        // plane, where the wrap makes a satellite its own neighbour.
        const uint32_t nextInPlane = plane * satsPerPlane + ((posInPlane + 1) % satsPerPlane);
        if (satsPerPlane > 1 && nextInPlane < spaceRics.size() && nextInPlane != i)
        {
            spaceRics[i]->AddIslNeighbor(spaceRics[nextInPlane]);
            spaceRics[nextInPlane]->AddIslNeighbor(spaceRics[i]);
            ++islLinks;
        }

        // Inter-plane neighbour: same position in the adjacent plane. Not
        // wrapped, because the seam between the first and last planes of a
        // Walker-Delta shell is where the relative velocity is highest and a
        // permanent ISL is not generally maintained across it.
        if (plane + 1 < numPlanes)
        {
            const uint32_t interPlaneIdx = (plane + 1) * satsPerPlane + posInPlane;
            if (interPlaneIdx < spaceRics.size())
            {
                spaceRics[i]->AddIslNeighbor(spaceRics[interPlaneIdx]);
                spaceRics[interPlaneIdx]->AddIslNeighbor(spaceRics[i]);
                ++islLinks;
            }
        }
    }
    NS_LOG_INFO("OranNtnHelper: wired " << islLinks << " bidirectional ISL neighbour links across "
                << spaceRics.size() << " Space RICs");

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

    // 6..14. Previously-dead xApps: instantiate and register them so they
    // receive ProcessKpmReport from the SAME measured KPM feed (audit
    // section 2 utilisation; Global invariant 4 — real classes exercised
    // end-to-end). Each derives from OranNtnXappBase and is RIC-managed.
    auto regXapp = [&](Ptr<OranNtnXappBase> x, const std::string& name,
                       uint8_t prio, Time interval) {
        x->SetXappName(name);
        x->SetPriority(prio);
        x->SetDecisionInterval(interval);
        ric->RegisterXapp(x);
        xapps[name] = x;
    };
    regXapp(CreateObject<OranNtnXappEnergyHarvest>(), "energy-harvest", 40,
            MilliSeconds(1000));
    regXapp(CreateObject<OranNtnXappInterferenceMgmt>(), "interference-mgmt", 22,
            MilliSeconds(200));
    regXapp(CreateObject<OranNtnXappIsac>(), "isac", 35, MilliSeconds(500));
    regXapp(CreateObject<OranNtnXappMultiConn>(), "multi-conn", 28,
            MilliSeconds(300));
    regXapp(CreateObject<OranNtnXappPredictiveAlloc>(), "predictive-alloc", 18,
            MilliSeconds(200));
    regXapp(CreateObject<OranNtnXappThzBeamMgmt>(), "thz-beam-mgmt", 26,
            MilliSeconds(200));
    regXapp(CreateObject<OranNtnXappThzRis>(), "thz-ris", 27, MilliSeconds(300));
    regXapp(CreateObject<OranNtnXappThzSpectrum>(), "thz-spectrum", 24,
            MilliSeconds(300));

    NS_LOG_INFO("OranNtnHelper: Created and registered " << xapps.size()
                << " xApps");
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

// ---- KPM injection ----

void
OranNtnHelper::InjectKpmReport(uint32_t gnbId, uint32_t ueId,
                                 double sinr, double rsrp, double tte,
                                 double elevation, double doppler,
                                 double measThroughputMbps,
                                 uint64_t measRxBytes,
                                 double measTbler,
                                 double measPrbUtil,
                                 bool sinrMeasured)
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
    report.sinrMeasured = sinrMeasured; // ORAN-03: the caller's word, not an assumption
    m_lastInjectedReport = report; // AI-07: expose what this call actually labelled
    report.rsrp_dBm = rsrp;
    report.tte_s = tte;
    report.elevation_deg = elevation;
    report.doppler_Hz = doppler;
    // RSRQ = N·RSRP/RSSI. For a loaded cell this reduces to S/(S+I+N) per RE,
    // i.e. SINR_lin/(1+SINR_lin), bounded to the 3GPP RSRQ range [-19.5,-3] dB.
    // (The old `sinr - 3` produced physically-impossible positive RSRQ.)
    {
        double sinrLin = std::pow(10.0, sinr / 10.0);
        report.rsrq_dB = std::max(-19.5, std::min(-3.0,
                            10.0 * std::log10(sinrLin / (1.0 + sinrLin))));
    }
    // CQI derived from SINR via the 3GPP TS 38.214 Table 5.2.2.1-2 (CQI table 1)
    // efficiency breakpoints, NOT the old "sinr + 6" clamp. Monotonic SINR->CQI
    // using the ~ -6.7..+14 dB span of the 15 usable indices; provenance =
    // derived (a real AMC would refine this from BLER/HARQ).
    {
        static const double kCqiSinrLb[16] = {
            -1e9,   // 0: out of range
            -6.7, -4.7, -2.3, 0.2, 2.4, 4.3, 5.9,  // 1..7 (QPSK/16QAM)
            8.1, 10.3, 11.7, 14.1, 16.3, 18.7, 21.0, 22.7 // 8..15 (64/256QAM)
        };
        uint8_t cqi = 0;
        for (uint8_t idx = 1; idx <= 15; ++idx)
        {
            if (sinr >= kCqiSinrLb[idx])
            {
                cqi = idx;
            }
        }
        report.cqi = cqi;
    }
    // Throughput (DRB.UEThpDl): use the MEASURED goodput when the caller has a
    // real data plane; only fall back to the SINR-derived link-budget estimate
    // for radio-less (geometry-budget) UEs. Global invariant 2 / rubric B.2.
    report.throughputMeasured = (measThroughputMbps >= 0.0);
    report.throughput_Mbps = report.throughputMeasured
                                 ? measThroughputMbps
                                 : std::max(0.0, 10.0 * (1.0 + sinr / 30.0));
    // DRB.PdcpSduVolumeDL is exported by WriteKpmDatasetCanonical as
    // throughput x interval; when measRxBytes>0 the measured throughput above
    // already reflects the real PDCP delivery, so the canonical volume tracks
    // the measured plane. (No separate struct field — kept measRxBytes in the
    // signature so callers thread real bytes into the throughput conversion.)
    (void)measRxBytes;
    // HARQ BLER (RRU/DRB error): from the measured TBLER when available.
    report.blerMeasured = (measTbler >= 0.0);
    if (report.blerMeasured)
    {
        report.harqBler = measTbler;
    }
    else
    {
        // Not measured: do NOT fabricate an error rate. NaN = "no measurement".
        report.harqBler = std::nan("");
    }
    // Propagation delay from the actual slant range, derived from elevation for
    // a LEO shell (h~600 km), instead of a single is_ntn constant. One-way
    // user-plane latency = propagation + ~1 ms processing/scheduling.
    if (it->second->IsNtn())
    {
        const double Re = 6371.0;
        const double h = 600.0;
        const double e = elevation * M_PI / 180.0;
        const double slantKm =
            Re * (std::sqrt(std::pow((Re + h) / Re, 2.0) - std::cos(e) * std::cos(e)) -
                  std::sin(e));
        report.propagationDelay_ms = slantKm / 299.792458; // km / (km/ms)
        report.latency_ms = report.propagationDelay_ms + 1.0;
    }
    else
    {
        report.propagationDelay_ms = 0.05;
        report.latency_ms = 4.0;
    }
    report.beamId = 1;
    // beamGain_dB is an antenna-pattern quantity that no measured source in this
    // synthetic injector supplies. Do NOT fabricate it from elevation*0.3;
    // emit NaN so consumers/exporters see "not measured" (real beam gain comes
    // from the NtnRealStackHelper array path, not this injector).
    report.beamGain_dB = std::nan("");
    // RRU.PrbUsedDl: use measured PRB utilisation when the caller supplies it
    // (real scheduler load). Otherwise emit NaN instead of a sine-of-sim-time
    // fabrication -- an unsupplied PRB load is UNKNOWN, not a wave. Global
    // invariant 2 / rubric B.2 / TS 28.552 measured-plane mandate.
    report.prbMeasured = (measPrbUtil >= 0.0);
    report.prbUtilization = report.prbMeasured
                                ? std::max(0.0, std::min(1.0, measPrbUtil))
                                : std::nan("");
    // activeUes: uint sentinel 0 = "not measured" (no active-UE count is plumbed
    // into this injector; the old 12+6*sin fabrication is removed).
    report.activeUes = 0;
    // Cell throughput follows PRB utilisation; when PRB is unknown (NaN) this
    // propagates NaN rather than fabricating a load-derived number.
    report.cellThroughput_Mbps = report.prbUtilization * 273.0 * 0.75; // util x PRB x Mbps/PRB
    // Assign slice based on UE modular grouping: eMBB=0, URLLC=1, mMTC=2
    report.sliceId = static_cast<uint8_t>(ueId % 3);
    // Slice-aware throughput/latency are DERIVED from the (measured-or-derived)
    // per-UE throughput/latency, so they are kept. sliceReliability, however,
    // was a hardcoded SLA TARGET (0.999/0.9999/0.99) masquerading as a measured
    // packet-delivery ratio -- no delivery ratio is measured here, so emit NaN.
    if (report.sliceId == 0) // eMBB
    {
        report.sliceThroughput_Mbps = report.throughput_Mbps;
        report.sliceLatency_ms = report.latency_ms;
    }
    else if (report.sliceId == 1) // URLLC
    {
        report.sliceThroughput_Mbps = std::max(0.1, report.throughput_Mbps * 0.1);
        report.sliceLatency_ms = report.latency_ms; // NTN latency violates URLLC SLA
    }
    else // mMTC
    {
        report.sliceThroughput_Mbps = std::max(0.01, report.throughput_Mbps * 0.02);
        report.sliceLatency_ms = report.latency_ms * 2.0;
    }
    report.sliceReliability = std::nan("");

    it->second->SubmitKpmMeasurement(report);
    m_kpmDataset.push_back(report);
}

void
OranNtnHelper::RegisterBeamLossModel(uint32_t gnbId, Ptr<NtnStaticExtraLossModel> model)
{
    NS_LOG_FUNCTION(this << gnbId << model);
    BeamActuatorState st;
    st.model = model;
    m_beamActuators[gnbId] = st;
}

void
OranNtnHelper::SetHandoverActuator(RcActuatorCallback cb)
{
    m_handoverActuator = cb;
}

void
OranNtnHelper::SetSliceActuator(RcActuatorCallback cb)
{
    m_sliceActuator = cb;
}

bool
OranNtnHelper::ApplyBeamAction(const E2RcAction& action)
{
    auto it = m_beamActuators.find(action.targetGnbId);
    if (it == m_beamActuators.end())
    {
        NS_LOG_WARN("OranNtnHelper: BEAM action for gNB " << action.targetGnbId
                    << " NOT actuated -- no beam loss model registered "
                    "(call RegisterBeamLossModel to wire the measured radio)");
        return false;
    }

    BeamActuatorState& st = it->second;
    if (action.actionType == E2RcActionType::BEAM_SHUTDOWN || action.parameter1 <= 0.0)
    {
        // Disengage the beam: no commanded gain on the channel.
        st.on = false;
        st.gainDb = 0.0;
    }
    else
    {
        // parameter1 = commanded array gain (dB); a fresh engage counts once.
        if (!st.on)
        {
            ++st.activations;
        }
        st.on = true;
        st.gainDb = action.parameter1;
    }

    // Apply as a LIVE channel reconfiguration: negative loss = array gain, so
    // the effect shows up in the MEASURED SINR/TBLER (same recipe as the
    // oran-ntn-ric-controlled-traffic reference loop).
    const double gain = st.on ? st.gainDb : 0.0;
    st.model->SetFloorDb(-gain);
    st.model->SetLossDb(-gain);
    return true;
}

bool
OranNtnHelper::DefaultRcActionHandler(E2RcAction action)
{
    NS_LOG_FUNCTION(this << static_cast<uint8_t>(action.actionType)
                    << action.xappName << action.targetGnbId << action.targetUeId
                    << action.confidence);

    // Dispatch the RC action into a real actuator per action type. "RIC control"
    // must not terminate in a log file: an action that cannot be actuated is
    // reported as a failure (never a silent accept), and the logged success is
    // exactly the value returned to the RIC (action_log <-> xapp_metrics agree).
    bool actuated = false;
    switch (action.actionType)
    {
    case E2RcActionType::BEAM_SWITCH:
    case E2RcActionType::BEAM_SHUTDOWN:
        actuated = ApplyBeamAction(action);
        break;

    case E2RcActionType::HANDOVER_TRIGGER:
    case E2RcActionType::HANDOVER_CANCEL:
        if (!m_handoverActuator.IsNull())
        {
            actuated = m_handoverActuator(action);
        }
        else
        {
            NS_LOG_WARN("OranNtnHelper: HANDOVER action from xApp "
                        << action.xappName << " for gNB " << action.targetGnbId
                        << " UE " << action.targetUeId << " NOT actuated -- no "
                        "handover actuator wired (SetHandoverActuator, e.g. "
                        "NtnRealStackHelper::SetHandover); action LOGGED ONLY");
            actuated = false;
        }
        break;

    case E2RcActionType::SLICE_PRB_ALLOCATION:
    case E2RcActionType::PRB_RESERVATION:
        // AI-04: these used to fall into default: and be logged only, so the
        // slice and predictive-allocation gym environments emitted action types
        // that could not reach a scheduler under any configuration.
        if (!m_sliceActuator.IsNull())
        {
            actuated = m_sliceActuator(action);
        }
        else
        {
            NS_LOG_WARN("OranNtnHelper: SLICE/PRB action from xApp "
                        << action.xappName << " for gNB " << action.targetGnbId
                        << " slice " << static_cast<uint32_t>(action.targetSliceId)
                        << " NOT actuated -- no slice actuator wired "
                        "(SetSliceActuator, e.g. SliceOrchestratorXapp::StepWithShares); "
                        "action LOGGED ONLY");
            actuated = false;
        }
        break;

    default:
        // No real-stack actuator is wired in this helper for the remaining RC
        // action types. Be honest: log it, name what is not actuated, fail.
        NS_LOG_WARN("OranNtnHelper: RC action type "
                    << static_cast<uint32_t>(static_cast<uint8_t>(action.actionType))
                    << " from xApp " << action.xappName << " (gNB "
                    << action.targetGnbId << ") has no actuator in OranNtnHelper; "
                    "action LOGGED ONLY, not actuated");
        actuated = false;
        break;
    }

    // Log the action -- entry.success is EXACTLY the returned value (truthful:
    // action_log.csv success can no longer contradict xapp_metrics.csv).
    ActionLogEntry entry;
    entry.timestamp = action.timestamp;
    entry.xappId = action.xappId;
    entry.xappName = action.xappName;
    entry.actionType = static_cast<uint8_t>(action.actionType);
    entry.targetGnb = action.targetGnbId;
    entry.targetUe = action.targetUeId;
    entry.confidence = action.confidence;
    entry.success = actuated;
    m_actionLog.push_back(entry);

    // ORAN-13: tell the issuing xApp what actually happened. Its own
    // successfulActions counter records only that the RIC ROUTED the action;
    // whether an actuator existed and fired is knowable here and nowhere else,
    // so without this the two files disagreed about the word "success".
    for (const auto& kv : m_allXapps)
    {
        if (kv.second && kv.second->GetXappId() == action.xappId)
        {
            kv.second->RecordActuation(actuated);
            break;
        }
    }

    return actuated;
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

    // Write KPM dataset (wide legacy format + WG3-canonical long format).
    WriteKpmDataset(m_outputDir + "/kpm_dataset.csv");
    WriteKpmDatasetCanonical(m_outputDir + "/kpm_canonical.csv");

    // Write per-xApp metrics including wall-clock decision latency percentiles.
    // Latency samples are collected by each xApp's DecisionCycle via
    // std::chrono::high_resolution_clock and pushed into XappMetrics
    // (oran-ntn-xapp-base.cc::RecordDecision).  We sort the per-xApp vector
    // and read off P50/P95/P99 here; -1 indicates no samples were recorded.
    std::ofstream xappOfs(m_outputDir + "/xapp_metrics.csv");
    // CVC-08: actuated_actions sits next to successful_actions on purpose.
    //
    // successful_actions counts ROUTING acceptance. This file was read as an
    // actuation count (an audit summed it to 71,967 and called them actions
    // that "actuate nothing"), and nothing in the file said otherwise. The two
    // columns side by side make the gap arithmetic rather than a matter of
    // knowing which is which: whatever routed but never reached an actuator is
    // successful_actions minus actuated_actions.
    xappOfs << "xapp_id,xapp_name,priority,decisions,successful_actions,"
               "actuated_actions,failed_actions,conflicts,conflicts_won,"
               "avg_confidence,avg_latency_ms,p50_latency_ms,p95_latency_ms,"
               "p99_latency_ms,min_latency_ms,max_latency_ms\n";

    auto pctl = [](const std::vector<double>& v, double q) {
        if (v.empty())
        {
            return -1.0;
        }
        double idx = q * static_cast<double>(v.size() - 1);
        size_t lo = static_cast<size_t>(std::floor(idx));
        size_t hi = static_cast<size_t>(std::ceil(idx));
        double frac = idx - static_cast<double>(lo);
        return v[lo] + (v[hi] - v[lo]) * frac;
    };

    auto xappIds = ric->GetRegisteredXappIds();
    for (uint32_t id : xappIds)
    {
        auto xapp = ric->GetXapp(id);
        auto m = xapp->GetMetrics();
        auto lat = m.decisionLatencies_ms;
        std::sort(lat.begin(), lat.end());
        double p50 = pctl(lat, 0.50);
        double p95 = pctl(lat, 0.95);
        double p99 = pctl(lat, 0.99);
        double lmin = lat.empty() ? -1.0 : lat.front();
        double lmax = lat.empty() ? -1.0 : lat.back();
        xappOfs << id << "," << xapp->GetXappName() << ","
                << (int)xapp->GetPriority() << "," << m.totalDecisions << ","
                << m.successfulActions << "," << m.actuatedActions << ","
                << m.failedActions << ","
                << m.conflictsEncountered << "," << m.conflictsWon << ","
                << m.avgConfidence << "," << m.avgDecisionLatency_ms << ","
                << p50 << "," << p95 << "," << p99 << ","
                << lmin << "," << lmax << "\n";
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
            << r.cellThroughput_Mbps << "," << (int)r.sliceId << ","
            << r.sliceThroughput_Mbps << "," << r.sliceLatency_ms << ","
            << r.sliceReliability << "\n";
    }

    NS_LOG_INFO("OranNtnHelper: Wrote " << m_kpmDataset.size()
                << " KPM records to " << filename);
}

void
OranNtnHelper::WriteKpmDatasetCanonical(const std::string& filename) const
{
    // Stable label set for the v2.1 baseline scenario: 5QI 9 (eMBB
    // default), single-slice S-NSSAI, PLMN 00101. Per-UE label routing is
    // a 4.1.4 / DataRepository concern (Q3 sprint follow-up).
    const std::map<std::string, std::string> baseLabels = {
        {oranntn::label::kFiveQi, "9"},
        {oranntn::label::kSnssai, "1-000001"},
        {oranntn::label::kPlmn, "00101"},
    };

    std::ofstream ofs(filename);
    oranntn::WriteCanonicalKpmCsv(m_kpmDataset, baseLabels, ofs);
    NS_LOG_INFO("OranNtnHelper: Wrote " << (m_kpmDataset.size() * 10)
                << " canonical KPM rows to " << filename);
}

} // namespace ns3
