/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-only
 * Author: Muhammad Uzair
 *
 * O-RAN NTN Types - Core data structures for the O-RAN NTN framework
 *
 * Defines E2-KPM reports, E2-RC actions, A1 policies, xApp conflict
 * records, and all shared types used across the Space-O-RAN architecture.
 *
 * Based on:
 *   - O-RAN.WG3.E2SM-KPM R004 v06.00 (ETSI TS 104 040 v4.0.0, 2025) (KPM Service Model)
 *   - O-RAN.WG3.E2SM-RC R004 v07.00 (2025) (RAN Control Service Model)
 *   - O-RAN.WG2.A1-v03.01 (A1 Interface)
 *   - Space-O-RAN (IEEE CommMag 2026)
 *   - O-RAN NTN White Paper (April 2025)
 */

#ifndef ORAN_NTN_TYPES_H
#define ORAN_NTN_TYPES_H

#include <cstdint>
#include <map>
#include <string>
#include <vector>

namespace ns3
{

// ============================================================================
//  E2-KPM: Key Performance Metrics (O-RAN WG3 E2SM-KPM)
// ============================================================================

/**
 * \brief E2 KPM report from an NTN gNB (satellite or terrestrial)
 *
 * Contains radio measurements and performance metrics reported
 * to the Near-RT RIC via the E2 interface every reporting period.
 */
struct E2KpmReport
{
    double timestamp;              //!< Simulation time (s)
    uint32_t gnbId;                //!< gNB/satellite ID
    bool isNtn;                    //!< true = satellite, false = terrestrial

    // Per-UE measurements (latest)
    uint32_t ueId;
    double rsrp_dBm;              //!< Reference Signal Received Power
    double rsrq_dB;               //!< Reference Signal Received Quality
    double sinr_dB;                //!< Signal-to-Interference+Noise Ratio
    uint8_t cqi;                   //!< Channel Quality Indicator (0-15)
    double throughput_Mbps;        //!< Instantaneous DL throughput
    double latency_ms;             //!< E2E latency

    // NTN-specific metrics
    double elevation_deg;          //!< Satellite elevation angle
    double doppler_Hz;             //!< Doppler shift
    double propagationDelay_ms;    //!< One-way delay
    double tte_s;                  //!< Time-to-Exit (beam coverage remaining)
    uint32_t beamId;               //!< Active beam ID
    double beamGain_dB;            //!< Antenna gain at UE position

    // Cell-level metrics
    double prbUtilization;         //!< PRB usage ratio (0-1)
    uint32_t activeUes;            //!< Number of connected UEs
    double cellThroughput_Mbps;    //!< Total cell throughput

    // Slice metrics
    uint8_t sliceId;               //!< 0=eMBB, 1=URLLC, 2=mMTC
    double sliceThroughput_Mbps;
    double sliceLatency_ms;
    double sliceReliability;       //!< Packet delivery ratio

    // === Deep satellite integration fields (Phase 1) ===
    uint8_t modCod;                    //!< DVB-S2X ModCod index
    double spectralEfficiency;         //!< bps/Hz from mutual information table
    double fadingGain_dB;              //!< Current channel fading gain
    uint8_t markovState;               //!< Markov fading state (0=clear,1=shadow,2=blocked)
    double interBeamInterference_dBm;  //!< Adjacent beam interference power
    double cno_dBHz;                   //!< Carrier-to-noise density ratio

    // === mmWave PHY integration fields (Phase 2) ===
    uint8_t mcs;                       //!< MCS index from mmWave AMC
    double wbCqi;                      //!< Wideband CQI from mmWave PHY
    double harqBler;                   //!< HARQ block error rate
    uint32_t harqRetx;                 //!< HARQ retransmission count
    double beamTrackingError_deg;      //!< Beamforming tracking error

    // === Dual connectivity fields (Phase 2) ===
    bool dualConnected;                //!< UE is in DC mode
    double tnSinr_dB;                  //!< Terrestrial link SINR
    double ntnSinr_dB;                 //!< NTN link SINR
    double tnThroughput_Mbps;          //!< Terrestrial throughput
    double ntnThroughput_Mbps;         //!< NTN throughput
    double bearerSplitRatio;           //!< Fraction routed via TN (0-1)

    // === Energy/power fields (Phase 4) ===
    double batteryStateOfCharge;       //!< Satellite battery SoC (0-1)
    double solarPower_W;               //!< Current solar panel output
    bool inEclipse;                    //!< Satellite in Earth shadow

    // === THz-NTN metrics (Phase 5: THz integration) ===
    double thzCenterFreq_GHz;              //!< Operating THz frequency
    double molecularAbsorption_dB;         //!< Current molecular absorption loss
    double pointingError_deg;              //!< Beam pointing error
    double thzSnr_dB;                      //!< THz link SNR
    uint16_t umMimoElements;               //!< Active UM-MIMO elements
    double risGain_dB;                     //!< RIS-assisted gain
    double hardwareImpairmentLoss_dB;      //!< PA + phase noise + ADC losses
    double atmosphericWindow_GHz;          //!< Current atmospheric window center
    bool isacSensingActive;                //!< ISAC mode flag
    double debrisDetectionRange_km;        //!< ISAC debris detection range
    double beamSquintLoss_dB;              //!< Wideband beam squint loss
    std::string activeWaveform;            //!< Current waveform (OFDM/OTFS/AFDM/etc)

    // === Provenance flags (TS 28.552 / measured-plane mandate) ===
    // Default false = the headline KPI was NOT taken from a real data plane, so
    // the canonical exporter labels it "derived" (link-budget estimate) or
    // "synthesized" (fabrication suppressed to NaN) rather than "measured".
    // InjectKpmReport() sets these true only when a measured value was supplied.
    bool throughputMeasured = false;   //!< throughput_Mbps came from FlowMonitor/RX bytes
    bool prbMeasured = false;          //!< prbUtilization came from the real scheduler
    bool blerMeasured = false;         //!< harqBler came from measured HARQ NACKs
};

// ============================================================================
//  E2-RC: RAN Control Actions (O-RAN WG3 E2SM-RC)
// ============================================================================

/**
 * \brief Types of RAN control actions an xApp can issue
 */
enum class E2RcActionType : uint8_t
{
    HANDOVER_TRIGGER = 0,          //!< Trigger handover to target cell
    HANDOVER_CANCEL = 1,           //!< Cancel pending handover
    BEAM_SWITCH = 2,               //!< Switch beam within same satellite
    BEAM_HOP_SCHEDULE = 3,         //!< Set beam allocation matrix
    SLICE_PRB_ALLOCATION = 4,      //!< Adjust PRB allocation per slice
    TIMING_ADVANCE_UPDATE = 5,     //!< Update TA for NTN link
    DOPPLER_COMP_UPDATE = 6,       //!< Update Doppler compensation
    CCA_THRESHOLD_ADJUST = 7,      //!< Adjust CCA threshold
    TX_POWER_CONTROL = 8,          //!< Adjust transmission power
    MCS_OVERRIDE = 9,              //!< Override MCS selection
    MODCOD_OVERRIDE = 10,          //!< Override DVB-S2X ModCod selection
    ISL_ROUTE_UPDATE = 11,         //!< Update ISL routing table
    REGEN_MODE_SWITCH = 12,        //!< Switch satellite regeneration mode
    BEAM_SHUTDOWN = 13,            //!< Shutdown low-priority beam (energy saving)
    COMPUTE_THROTTLE = 14,         //!< Throttle Space RIC compute budget
    DC_SETUP = 15,                 //!< Activate dual connectivity for UE
    DC_TEARDOWN = 16,              //!< Deactivate dual connectivity
    BEARER_SPLIT = 17,             //!< Adjust TN/NTN bearer split ratio
    INTERFERENCE_NULLING = 18,     //!< Apply beam nulling for interference mitigation
    PRB_RESERVATION = 19,          //!< Proactive PRB reservation from traffic prediction
    FL_MODEL_PUSH = 20,            //!< Push federated learning model update
    ENERGY_PROFILE_UPDATE = 21,    //!< Update satellite energy profile/mode
    ACTION_THZ_FREQ_SELECT = 22,        //!< Switch THz operating band
    ACTION_THZ_BEAM_CODEBOOK = 23,      //!< Update beam codebook
    ACTION_THZ_RIS_CONFIG = 24,         //!< Configure RIS phase profile
    ACTION_THZ_WAVEFORM_SELECT = 25,    //!< Switch waveform (OFDM/OTFS/AFDM)
    ACTION_THZ_ISAC_MODE = 26,         //!< Enable/disable ISAC sensing
    ACTION_THZ_POWER_BACKOFF = 27,     //!< PA backoff for linearity
    ACTION_THZ_WINDOW_HOP = 28,        //!< Hop to different atmospheric window
};

/**
 * \brief E2 RC action issued by an xApp to control the RAN
 */
struct E2RcAction
{
    double timestamp;
    uint32_t xappId;               //!< xApp that issued the action
    std::string xappName;          //!< Human-readable xApp name
    E2RcActionType actionType;
    uint32_t targetGnbId;          //!< Target gNB/satellite
    uint32_t targetUeId;           //!< Target UE (0 = cell-wide)
    uint32_t targetBeamId;         //!< Target beam
    uint8_t targetSliceId;         //!< Target slice
    double confidence;             //!< Agent confidence (0-1)
    double parameter1;             //!< Action-specific parameter
    double parameter2;             //!< Action-specific parameter
    bool executed;                 //!< Whether action was executed
    std::string rejectionReason;   //!< Why rejected (if not executed)
};

// ============================================================================
//  A1 Interface: Policies from Non-RT RIC (O-RAN WG2)
// ============================================================================

/**
 * \brief A1 policy types
 */
enum class A1PolicyType : uint8_t
{
    HO_THRESHOLD = 0,              //!< Handover SINR/TTE thresholds
    SLICE_SLA = 1,                 //!< Slice SLA requirements
    BEAM_PRIORITY = 2,             //!< Beam allocation priorities
    TN_NTN_STEERING = 3,          //!< TN vs NTN traffic steering weight
    LOAD_BALANCE = 4,              //!< Load balancing parameters
    ENERGY_SAVING = 5,             //!< Energy saving mode policy
    INTERFERENCE_LIMIT = 6,        //!< Max inter-beam interference threshold
    DC_PREFERENCE = 7,             //!< Dual connectivity activation criteria
    FL_PARTICIPATION = 8,          //!< Federated learning participation policy
    MODCOD_RANGE = 9,              //!< Allowed ModCod range per beam/slice
    PREDICTIVE_ALLOC = 10,         //!< Predictive allocation lookahead policy
};

/**
 * \brief A1 policy from Non-RT RIC to Near-RT RIC
 */
struct A1Policy
{
    double timestamp;
    uint32_t policyId;
    A1PolicyType type;
    std::string scope;             //!< "global", "satellite:<id>", "slice:<id>"
    uint8_t priority;              //!< 0=highest, 255=lowest
    double param1;                 //!< Policy-specific parameter
    double param2;
    double param3;
    bool active;
};

// ============================================================================
//  xApp Conflict Management
// ============================================================================

/**
 * \brief WG3-aligned conflict taxonomy (Roadmap §4.1.10).
 *
 * Per O-RAN.WG3 conflict-mitigation terminology three categories matter:
 *   - DIRECT   two xApps target the same parameter on the same UE/cell
 *   - INDIRECT two xApps target different parameters but in the same
 *              resource family on the same gNB (e.g. PRB reservation vs
 *              slice PRB share)
 *   - IMPLICIT actions are coupled through a shared KPI; here flagged
 *              when both target the same UE but the action types and
 *              resource families differ
 *   - UNKNOWN  classifier could not place the pair (kept so non-WG3
 *              conflicts still log without losing taxonomy)
 */
enum class ConflictType : uint8_t
{
    UNKNOWN = 0,
    DIRECT = 1,
    INDIRECT = 2,
    IMPLICIT = 3,
};

inline const char*
ConflictTypeName(ConflictType c)
{
    switch (c)
    {
    case ConflictType::DIRECT:   return "direct";
    case ConflictType::INDIRECT: return "indirect";
    case ConflictType::IMPLICIT: return "implicit";
    default:                     return "unknown";
    }
}

/**
 * \brief Record of a conflict between two xApps
 */
struct XappConflict
{
    double timestamp;
    uint32_t xapp1Id;
    std::string xapp1Name;
    uint32_t xapp2Id;
    std::string xapp2Name;
    std::string resourceType;      //!< "beam", "prb", "handover", "power"
    uint32_t resourceId;           //!< Which specific resource
    std::string resolution;        //!< "priority", "merge", "reject_lower"
    uint32_t winnerId;             //!< Which xApp's action was executed
    ConflictType conflictType{ConflictType::UNKNOWN}; //!< WG3 taxonomy (Roadmap §4.1.10)
};

// ============================================================================
//  Beam Allocation Matrix (for Beam Hopping)
// ============================================================================

/**
 * \brief Single entry in the beam allocation matrix
 */
struct BeamAllocationEntry
{
    uint32_t satId;
    uint32_t beamId;
    uint32_t timeSlot;             //!< Slot index within hopping frame
    uint32_t allocatedCellId;      //!< Which cell gets this beam-slot
    double trafficLoad;            //!< Predicted traffic load (0-1)
    bool isSignaling;              //!< true = signaling, false = data
};

// ============================================================================
//  Network Slice Configuration
// ============================================================================

/**
 * \brief Network slice definition per 3GPP TS 23.501
 */
struct SliceConfig
{
    uint8_t sliceId;               //!< 0=eMBB, 1=URLLC, 2=mMTC
    std::string name;              //!< "eMBB", "URLLC", "mMTC"
    double minThroughput_Mbps;     //!< Guaranteed minimum
    double maxLatency_ms;          //!< Maximum tolerable latency
    double reliabilityTarget;      //!< Target PDR (e.g., 0.99999)
    double prbShare;               //!< Allocated PRB fraction (0-1)
    bool harqEnabled;              //!< HARQ for this slice
    uint8_t priority;              //!< Scheduling priority
};

// ============================================================================
//  O-RAN Node Types
// ============================================================================

/**
 * \brief Type of O-RAN node
 */
enum class OranNodeType : uint8_t
{
    O_CU_CP = 0,                  //!< Central Unit - Control Plane
    O_CU_UP = 1,                  //!< Central Unit - User Plane
    O_DU = 2,                     //!< Distributed Unit
    O_RU = 3,                     //!< Radio Unit
    NEAR_RT_RIC = 4,              //!< Near-Real-Time RIC
    NON_RT_RIC = 5,               //!< Non-Real-Time RIC / SMO
    SPACE_RIC = 6,                //!< Space RIC (on-orbit, Space-O-RAN)
    D_APP = 7,                    //!< Distributed App (on-satellite)
};

/**
 * \brief Deployment mode for NTN O-RAN
 */
enum class NtnDeploymentMode : uint8_t
{
    TRANSPARENT = 0,               //!< O-RU on satellite, rest on ground
    REGENERATIVE_DU = 1,           //!< O-DU + O-RU on satellite
    REGENERATIVE_FULL = 2,         //!< Full gNB on satellite (Rel-19)
};

// ============================================================================
//  ISL Link State (for Space RIC coordination)
// ============================================================================

struct IslLinkState
{
    uint32_t satId1;
    uint32_t satId2;
    double distance_km;
    double delay_ms;
    double dataRate_Mbps;
    double utilization;              //!< Link utilization (0-1)
    bool active;                     //!< ISL link is operational
};

// ============================================================================
//  Regeneration Mode Mapping
// ============================================================================

struct NtnRegenerationConfig
{
    NtnDeploymentMode oranMode;
    uint8_t satRegenMode;            //!< 0=TRANSPARENT, 1=REGEN_PHY, 2=REGEN_LINK, 3=REGEN_NETWORK
    bool onBoardScheduling;          //!< Satellite performs MAC scheduling
    bool onBoardRouting;             //!< Satellite performs IP routing
};

// ============================================================================
//  Federated Learning Round State
// ============================================================================

struct FederatedLearningRound
{
    uint32_t roundId;
    std::string modelName;
    double timestamp;
    uint32_t numParticipants;
    uint32_t numGradientsReceived;
    std::string aggregationStrategy;  //!< "FedAvg", "FedProx", "FedNova"
    double aggregationLoss;           //!< Global loss after aggregation
    bool completed;
};

// ============================================================================
//  Energy Harvesting State
// ============================================================================

struct SatelliteEnergyState
{
    uint32_t satId;
    double timestamp;
    double batteryCapacity_Wh;
    double batteryLevel_Wh;
    double solarPanelArea_m2;
    double solarEfficiency;
    double currentSolarPower_W;
    double currentConsumption_W;
    bool inEclipse;
    double timeToEclipse_s;          //!< Time until next eclipse entry
    double eclipseDuration_s;        //!< Duration of next eclipse
    uint32_t activeBeams;            //!< Number of active beams
    double computeLoad;              //!< OBP compute utilization (0-1)
};

// ============================================================================
//  Interference Map Entry
// ============================================================================

struct InterferenceMapEntry
{
    uint32_t victimSatId;
    uint32_t victimBeamId;
    uint32_t aggressorSatId;
    uint32_t aggressorBeamId;
    double interference_dBm;
    double couplingLoss_dB;          //!< Antenna pattern overlap
    double angularSeparation_deg;
};

// ============================================================================
//  Traffic Prediction State
// ============================================================================

struct TrafficPrediction
{
    uint32_t beamId;
    double timestamp;
    std::vector<double> predictedLoad;    //!< Predicted load for next N steps
    std::vector<double> confidence;       //!< Per-step confidence
    double mse;                           //!< Prediction error (last round)
    uint32_t horizonSteps;
};

// ============================================================================
//  Dual Connectivity Session
// ============================================================================

struct DualConnSession
{
    uint32_t ueId;
    uint32_t tnGnbId;               //!< Terrestrial gNB ID
    uint32_t ntnSatId;              //!< NTN satellite ID
    uint32_t ntnBeamId;             //!< NTN beam ID
    double tnSplitRatio;            //!< Fraction via TN (0-1)
    double activationTime;
    bool splitBearerActive;
    uint8_t primaryPath;            //!< 0=TN, 1=NTN
};

} // namespace ns3

#endif // ORAN_NTN_TYPES_H
