/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-only
 * Author: Muhammad Uzair
 *
 * O-RAN NTN Helper - Top-level scenario builder
 *
 * Provides convenient API for setting up O-RAN NTN simulations:
 *   - Creates Near-RT RIC, Non-RT RIC, Space RICs
 *   - Instantiates and registers xApps
 *   - Connects E2 nodes from satellite constellation
 *   - Generates A1 policies from constellation parameters
 *   - Configures KPM reporting and RC action routing
 *   - Collects comprehensive metrics and writes CSV output
 */

#ifndef ORAN_NTN_HELPER_H
#define ORAN_NTN_HELPER_H

#include "ns3/node-container.h"
#include "ns3/nstime.h"
#include "ns3/object.h"
#include "ns3/oran-ntn-sat-bridge.h"
#include "ns3/oran-ntn-types.h"
#include "ns3/ptr.h"

#include <map>
#include <string>
#include <vector>

namespace ns3
{

class OranNtnNearRtRic;
class OranNtnA1PolicyManager;
class OranNtnSpaceRic;
class OranNtnE2Node;
class OranNtnXappBase;
class OranNtnSatBridge;

/**
 * \ingroup oran-ntn
 * \brief Helper for creating O-RAN NTN simulation scenarios
 */
class OranNtnHelper : public Object
{
  public:
    static TypeId GetTypeId();
    OranNtnHelper();
    ~OranNtnHelper() override;

    // ---- Core setup ----

    /**
     * \brief Create the Near-RT RIC
     */
    Ptr<OranNtnNearRtRic> CreateNearRtRic();

    /**
     * \brief Create the Non-RT RIC / SMO (A1 policy manager)
     */
    Ptr<OranNtnA1PolicyManager> CreateNonRtRic();

    /**
     * \brief Connect Near-RT RIC to Non-RT RIC via A1
     */
    void ConnectA1Interface(Ptr<OranNtnNearRtRic> nearRtRic,
                             Ptr<OranNtnA1PolicyManager> nonRtRic);

    // ---- E2 node setup ----

    /**
     * \brief Create E2 nodes for satellite constellation
     *
     * One E2 node per satellite node. Automatically configures
     * feeder link delay based on satellite altitude.
     */
    std::vector<Ptr<OranNtnE2Node>> CreateSatelliteE2Nodes(
        NodeContainer satNodes, Ptr<OranNtnNearRtRic> ric);

    /**
     * \brief Create E2 nodes for terrestrial gNBs
     */
    std::vector<Ptr<OranNtnE2Node>> CreateTerrestrialE2Nodes(
        NodeContainer gnbNodes, Ptr<OranNtnNearRtRic> ric);

    // ---- Space RIC setup ----

    /**
     * \brief Create Space RICs for satellite constellation
     *
     * One Space RIC per satellite, grouped by orbital plane.
     * Automatically sets up ISL neighbor connections.
     */
    std::vector<Ptr<OranNtnSpaceRic>> CreateSpaceRics(
        NodeContainer satNodes, uint32_t numPlanes, uint32_t satsPerPlane,
        Ptr<OranNtnNearRtRic> groundRic);

    // ---- xApp instantiation ----

    /**
     * \brief Create and register all 5 standard xApps
     *
     * Creates: HO-Predict, Beam-Hop, Slice-Manager, Doppler-Comp, TN-NTN-Steering
     * \return map of xApp name -> xApp pointer
     */
    std::map<std::string, Ptr<OranNtnXappBase>> CreateAllXapps(
        Ptr<OranNtnNearRtRic> ric);

    /**
     * \brief Start all registered xApps
     */
    void StartAllXapps(Ptr<OranNtnNearRtRic> ric);

    // ---- Policy generation ----

    /**
     * \brief Generate orbit-aware A1 policies for the constellation
     */
    void GenerateConstellationPolicies(Ptr<OranNtnA1PolicyManager> nonRtRic,
                                        uint32_t numPlanes, uint32_t satsPerPlane,
                                        double inclinationDeg, double altitudeKm);

    // ---- KPM feed simulation ----

    /**
     * \brief Start simulated KPM feed from E2 nodes
     *
     * Generates realistic KPM reports at the given interval from
     * all connected E2 nodes. Uses satellite position + antenna
     * patterns for link budget computation.
     */
    void StartKpmFeed(Ptr<OranNtnNearRtRic> ric, Time reportInterval);

    /**
     * \brief Submit a manual KPM report to an E2 node
     */
    void InjectKpmReport(uint32_t gnbId, uint32_t ueId,
                          double sinr, double rsrp, double tte,
                          double elevation, double doppler);

    // ---- Phase 2: mmWave NTN stack setup ----

    /**
     * \brief Setup mmWave NTN PHY stack on satellite gNBs
     *
     * Installs NTN beamforming model, NTN channel model, and NTN scheduler.
     */
    void SetupMmWaveNtnStack(NodeContainer satNodes, NodeContainer ueNodes,
                              double carrierFreqHz = 20e9);

    /**
     * \brief Setup dual connectivity between TN and NTN
     */
    void SetupDualConnectivity(NodeContainer tnGnbs, NodeContainer ueNodes);

    // ---- Phase 3: AI integration ----

    /**
     * \brief Setup ns3-ai Gymnasium environments for all xApps
     */
    void SetupAiIntegration(Ptr<OranNtnNearRtRic> ric, bool useGym = true);

    /**
     * \brief Enable real PHY KPM extraction (vs synthetic)
     */
    void EnablePhyKpmExtraction();

    // ---- Phase 4: Advanced xApps ----

    /**
     * \brief Create and register all 9 xApps (5 original + 4 new)
     */
    std::map<std::string, Ptr<OranNtnXappBase>> CreateAllAdvancedXapps(
        Ptr<OranNtnNearRtRic> ric);

    // ---- Phase 5: ISL and Federated Learning ----

    /**
     * \brief Setup ISL network topology between satellites
     */
    void SetupIslNetwork(NodeContainer satNodes, uint32_t numPlanes,
                          uint32_t satsPerPlane, double islDataRate_Mbps = 1000.0);

    /**
     * \brief Setup federated learning coordination
     */
    void SetupFederatedLearning(const std::string& modelName, Time roundInterval);

    /**
     * \brief Get the satellite bridge (for external access)
     */
    Ptr<OranNtnSatBridge> GetSatBridge() const;

    // ---- Output ----

    /**
     * \brief Set output directory for all CSV/log files
     */
    void SetOutputDirectory(const std::string& dir);

    /**
     * \brief Write all metrics at end of simulation
     */
    void WriteAllMetrics(Ptr<OranNtnNearRtRic> ric) const;

    /**
     * \brief Write xApp action log to CSV
     */
    void WriteActionLog(const std::string& filename) const;

    /**
     * \brief Write conflict log to CSV
     */
    void WriteConflictLog(Ptr<OranNtnNearRtRic> ric,
                           const std::string& filename) const;

    /**
     * \brief Write KPM data to CSV for AI training
     */
    void WriteKpmDataset(const std::string& filename) const;

  protected:
    void DoDispose() override;

  private:
    std::string m_outputDir;
    std::map<uint32_t, Ptr<OranNtnE2Node>> m_e2Nodes;
    std::vector<Ptr<OranNtnSpaceRic>> m_spaceRics;

    // KPM simulation state
    struct KpmFeedState
    {
        EventId feedEvent;
        Time interval;
    };
    KpmFeedState m_kpmFeed;

    void KpmFeedCallback();
    bool DefaultRcActionHandler(E2RcAction action);

    // Action log
    struct ActionLogEntry
    {
        double timestamp;
        uint32_t xappId;
        std::string xappName;
        uint8_t actionType;
        uint32_t targetGnb;
        uint32_t targetUe;
        double confidence;
        bool success;
    };
    mutable std::vector<ActionLogEntry> m_actionLog;

    // KPM dataset accumulation
    mutable std::vector<E2KpmReport> m_kpmDataset;

    // Phase 2-5 components
    Ptr<OranNtnSatBridge> m_satBridge;
    std::map<std::string, Ptr<OranNtnXappBase>> m_allXapps;
    bool m_phyKpmEnabled;
    bool m_aiEnabled;
};

} // namespace ns3

#endif // ORAN_NTN_HELPER_H
