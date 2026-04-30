/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-only
 * Author: Muhammad Uzair
 *
 * O-RAN NTN Satellite Bridge - Deep integration with SNS3 satellite module
 *
 * Bridges the O-RAN NTN framework with the SNS3 satellite module to provide:
 *   - Real SGP4 orbit propagation for satellite positions
 *   - Physical antenna gain patterns for beam coverage
 *   - GeoCoordinate-based distance and elevation computations
 *   - Real-time KPM report generation from satellite module state
 *   - Feeder link availability based on gateway visibility
 *   - ISL topology from constellation geometry
 *
 * Also bridges with mmWave module for:
 *   - NR PHY/MAC stack on satellite gNBs
 *   - 3GPP NTN channel models (TR 38.811)
 *   - Doppler-aware spectrum management
 *   - Beamforming model integration
 */

#ifndef ORAN_NTN_SAT_BRIDGE_H
#define ORAN_NTN_SAT_BRIDGE_H

#include "oran-ntn-e2-interface.h"
#include "oran-ntn-space-ric.h"
#include "oran-ntn-types.h"

#include <ns3/geo-coordinate.h>
#include <ns3/node-container.h>
#include <ns3/nstime.h>
#include <ns3/object.h>
#include <ns3/satellite-antenna-gain-pattern-container.h>
#include <ns3/satellite-antenna-gain-pattern.h>
#include <ns3/satellite-free-space-loss.h>
#include <ns3/satellite-mobility-model.h>
#include <ns3/satellite-sgp4-mobility-model.h>
#include <ns3/three-gpp-propagation-loss-model.h>
#include <ns3/traced-callback.h>

#include <map>
#include <utility>
#include <vector>

namespace ns3
{

// Forward declarations
namespace mmwave
{
class MmWaveHelper;
class MmWaveEnbNetDevice;
class MmWaveUeNetDevice;
class MmWavePhyMacCommon;
} // namespace mmwave

/**
 * \brief Per-satellite state tracked by the bridge
 */
struct SatelliteBridgeState
{
    uint32_t satId;
    uint32_t planeId;
    uint32_t posInPlane;
    Ptr<Node> node;
    Ptr<SatMobilityModel> mobility;
    Ptr<SatSGP4MobilityModel> sgp4Mobility;  //!< SGP4 orbit model (if available)
    Ptr<SatAntennaGainPatternContainer> antennaPatterns;
    Ptr<OranNtnE2Node> e2Node;
    Ptr<OranNtnSpaceRic> spaceRic;

    // Cached orbital state
    GeoCoordinate lastPosition;
    Vector lastVelocity;
    double altitude_km;
    double groundSpeed_kmps;

    // Beam state
    std::map<uint32_t, double> beamLoads;       //!< beamId -> PRB utilization
    std::map<uint32_t, uint32_t> beamActiveUes;  //!< beamId -> count

    // Feeder link
    bool feederLinkAvailable;
    uint32_t servingGatewayId;

    // Deep satellite integration (Phase 1)
    uint8_t markovFadingState;       //!< 0=clear, 1=shadow, 2=blocked
    double lastFadingGain_dB;        //!< Last computed fading channel gain
    uint8_t currentModCod;           //!< Current DVB-S2X ModCod index
    double spectralEfficiency;       //!< Current spectral efficiency (bps/Hz)
    uint8_t regenerationMode;        //!< 0=transparent, 1=regen_phy, 2=regen_link, 3=regen_full
};

/**
 * \brief Per-UE state tracked by the bridge
 */
struct UeBridgeState
{
    uint32_t ueId;
    Ptr<Node> node;
    Ptr<SatMobilityModel> ueMobility;
    GeoCoordinate position;
    Vector velocity;

    // Serving cell
    uint32_t servingSatId;
    uint32_t servingBeamId;
    double servingSinr_dB;
    double servingRsrp_dBm;
    double servingTte_s;
    double servingElevation_deg;
    double servingDoppler_Hz;
    double servingDelay_ms;

    // Slice assignment
    uint8_t sliceId;

    // Mobility classification
    enum MobilityClass
    {
        STATIC,
        PEDESTRIAN,
        VEHICULAR,
        HST,
        AERIAL
    } mobilityClass;
};

// ============================================================================
//  Satellite Bridge
// ============================================================================

/**
 * \ingroup oran-ntn
 * \brief Deep integration bridge between O-RAN NTN and SNS3 satellite + mmWave
 *
 * This class is the core integration point. It:
 *   1. Wraps satellite module's SGP4 mobility for orbit computation
 *   2. Uses real antenna gain patterns for link budget
 *   3. Computes 3GPP NTN pathloss per TR 38.811
 *   4. Generates realistic KPM reports from physical layer state
 *   5. Drives E2 nodes and Space RICs with real satellite data
 *   6. Manages feeder link availability from gateway geometry
 */
class OranNtnSatBridge : public Object
{
  public:
    static TypeId GetTypeId();
    OranNtnSatBridge();
    ~OranNtnSatBridge() override;

    // ---- Constellation setup ----

    /**
     * \brief Initialize from existing satellite constellation nodes
     *
     * Reads SGP4 mobility models and antenna patterns from satellite nodes
     * that were already created by SatHelper.
     */
    void InitializeConstellation(NodeContainer satNodes,
                                  uint32_t numPlanes,
                                  uint32_t satsPerPlane,
                                  Ptr<SatAntennaGainPatternContainer> antennaPatterns);

    /**
     * \brief Register UE nodes for tracking
     */
    void RegisterUeNodes(NodeContainer ueNodes);

    /**
     * \brief Wire E2 nodes to satellite states
     */
    void AttachE2Nodes(const std::vector<Ptr<OranNtnE2Node>>& e2Nodes);

    /**
     * \brief Wire Space RICs to satellite states
     */
    void AttachSpaceRics(const std::vector<Ptr<OranNtnSpaceRic>>& spaceRics);

    // ---- Real-time satellite queries ----

    /**
     * \brief Get current satellite position from SGP4
     */
    GeoCoordinate GetSatellitePosition(uint32_t satId) const;

    /**
     * \brief Get satellite velocity vector
     */
    Vector GetSatelliteVelocity(uint32_t satId) const;

    /**
     * \brief Get UE position
     */
    GeoCoordinate GetUePosition(uint32_t ueId) const;

    /**
     * \brief Compute elevation angle from UE to satellite using real geometry
     */
    double ComputeElevationAngle(uint32_t ueId, uint32_t satId) const;

    /**
     * \brief Compute slant range (3D distance) between UE and satellite
     */
    double ComputeSlantRange(uint32_t ueId, uint32_t satId) const;

    /**
     * \brief Compute propagation delay (slant range / c)
     */
    Time ComputePropagationDelay(uint32_t ueId, uint32_t satId) const;

    /**
     * \brief Compute Doppler shift from relative satellite-UE motion
     */
    double ComputeDopplerShift(uint32_t ueId, uint32_t satId,
                                double carrierFreqHz) const;

    /**
     * \brief Get antenna gain at UE position for a specific beam
     */
    double GetBeamGainAtUe(uint32_t ueId, uint32_t satId, uint32_t beamId) const;

    // ---- Link budget using 3GPP NTN models ----

    /**
     * \brief Compute RSRP using 3GPP TR 38.811 NTN pathloss model
     *
     * Uses ThreeGppNTN{DenseUrban,Urban,Suburban,Rural}PropagationLossModel
     * based on configured scenario.
     */
    double ComputeNtnRsrp(uint32_t ueId, uint32_t satId, uint32_t beamId) const;

    /**
     * \brief Compute SINR from RSRP and interference
     */
    double ComputeNtnSinr(uint32_t ueId, uint32_t satId, uint32_t beamId) const;

    /**
     * \brief Compute full link budget and return KPM report
     */
    E2KpmReport ComputeFullLinkBudget(uint32_t ueId, uint32_t satId,
                                        uint32_t beamId) const;

    // ---- Visibility and coverage ----

    /**
     * \brief Find all satellites visible to a UE (elevation > threshold)
     */
    std::vector<uint32_t> GetVisibleSatellites(uint32_t ueId,
                                                 double minElevationDeg = 10.0) const;

    /**
     * \brief Find the best beam for a UE on a given satellite
     */
    uint32_t FindBestBeam(uint32_t ueId, uint32_t satId) const;

    /**
     * \brief Compute TTE for a UE under a satellite beam using real SGP4
     */
    double ComputeTte(uint32_t ueId, uint32_t satId, uint32_t beamId) const;

    // ---- Feeder link management ----

    /**
     * \brief Update feeder link availability based on gateway positions
     */
    void UpdateFeederLinkStatus(NodeContainer gatewayNodes);

    // ---- Periodic KPM generation ----

    /**
     * \brief Start periodic KPM report generation from real satellite state
     */
    void StartRealisticKpmFeed(Time interval);

    /**
     * \brief Stop KPM feed
     */
    void StopKpmFeed();

    // ---- Deep satellite integration (Phase 1) ----

    /**
     * \brief Compute SINR with satellite fading model (Loo/Markov)
     */
    double ComputeNtnSinrWithFading(uint32_t ueId, uint32_t satId, uint32_t beamId) const;

    /**
     * \brief Select optimal DVB-S2X ModCod for current link conditions
     * \return ModCod index (0-27 for DVB-S2)
     */
    uint8_t SelectModCod(uint32_t ueId, uint32_t satId) const;

    /**
     * \brief Compute inter-beam interference power
     */
    double ComputeInterBeamInterference(uint32_t ueId, uint32_t satId,
                                         uint32_t beamId) const;

    /**
     * \brief Setup ISL network topology between orbital neighbors
     */
    void SetupIslTopology(NodeContainer satNodes, double islDataRate_Mbps);

    /**
     * \brief Get ISL link state between two satellites
     */
    IslLinkState GetIslLinkState(uint32_t satId1, uint32_t satId2) const;

    /**
     * \brief Get all ISL neighbors for a satellite
     */
    std::vector<uint32_t> GetIslNeighbors(uint32_t satId) const;

    /**
     * \brief Compute C/N0 (carrier-to-noise density) for a link
     */
    double ComputeCno(uint32_t ueId, uint32_t satId, uint32_t beamId) const;

    /**
     * \brief Get Markov fading state for a satellite
     */
    uint8_t GetMarkovState(uint32_t satId) const;

    // ---- NTN scenario configuration ----

    /**
     * \brief Set NTN propagation scenario
     */
    void SetNtnScenario(const std::string& scenario);

    /**
     * \brief Set carrier frequency
     */
    void SetCarrierFrequency(double freqHz);

    /**
     * \brief Set satellite TX power per beam
     */
    void SetSatelliteTxPower(double txPower_dBm);

    /**
     * \brief Set system bandwidth
     */
    void SetBandwidth(double bwHz);

    // ---- State access ----

    const SatelliteBridgeState& GetSatState(uint32_t satId) const;
    const UeBridgeState& GetUeState(uint32_t ueId) const;
    uint32_t GetNumSatellites() const;
    uint32_t GetNumUes() const;

    // ---- Trace sources ----
    TracedCallback<uint32_t, E2KpmReport> m_kpmGenerated;
    TracedCallback<uint32_t, bool> m_feederLinkChanged;
    TracedCallback<uint32_t, uint32_t, double> m_elevationComputed;

  protected:
    void DoDispose() override;

  private:
    void PeriodicKpmCallback();
    void UpdateSatelliteStates();
    void UpdateUeStates();

    // Constellation
    std::map<uint32_t, SatelliteBridgeState> m_satStates;
    std::map<uint32_t, UeBridgeState> m_ueStates;
    uint32_t m_numPlanes;
    uint32_t m_satsPerPlane;

    // Shared antenna patterns
    Ptr<SatAntennaGainPatternContainer> m_antennaPatterns;

    // 3GPP NTN propagation model
    Ptr<ThreeGppPropagationLossModel> m_ntnPathloss;
    std::string m_ntnScenario;

    // Physical parameters
    double m_carrierFreqHz;
    double m_txPower_dBm;
    double m_bandwidth_Hz;
    double m_noiseFigure_dB;
    double m_boltzmann;
    double m_temperature_K;
    double m_speedOfLight;

    // KPM feed
    EventId m_kpmFeedEvent;
    Time m_kpmInterval;
    bool m_kpmFeedActive;

    // === Deep satellite integration (Phase 1) ===
    Ptr<SatFreeSpaceLoss> m_freeSpaceLoss;   //!< Satellite module FSPL calculator

    // ISL topology: (satId1, satId2) -> link state
    std::map<std::pair<uint32_t, uint32_t>, IslLinkState> m_islLinks;

    // Markov fading state per satellite
    mutable std::map<uint32_t, uint8_t> m_markovFadingStates;

    // DVB-S2X ModCod SINR thresholds (ascending order)
    static const std::vector<std::pair<double, uint8_t>> s_modcodThresholds;
};

} // namespace ns3

#endif // ORAN_NTN_SAT_BRIDGE_H
