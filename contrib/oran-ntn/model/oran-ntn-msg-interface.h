/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-only
 * Author: Muhammad Uzair
 *
 * ns3-ai Message Interface Structures for O-RAN NTN
 *
 * Defines shared memory structs for high-performance IPC between
 * ns-3 C++ simulation and Python AI agents via ns3-ai msg-interface.
 *
 * Used by:
 *   - Space RIC for on-board inference (15x faster than socket-based)
 *   - Federated learning weight exchange between satellites
 *   - Ground RIC inference for latency-sensitive xApp decisions
 *
 * Design: Fixed-size structs for zero-copy shared memory transfer.
 * All arrays sized for max constellation (72 beams per satellite).
 */

#ifndef ORAN_NTN_MSG_INTERFACE_H
#define ORAN_NTN_MSG_INTERFACE_H

#include <cstdint>

namespace ns3
{

// ============================================================================
//  Space RIC Observation/Action (msg-interface struct mode)
// ============================================================================

static constexpr uint32_t MAX_BEAMS_PER_SAT = 72;
static constexpr uint32_t MAX_ISL_NEIGHBORS = 4;
static constexpr uint32_t MAX_UES_PER_BEAM = 20;

/**
 * \brief Observation sent from C++ (Space RIC) to Python agent
 *
 * Contains complete satellite state for on-board AI inference.
 */
struct SpaceRicObservation
{
    // Identity
    uint32_t satId;
    uint32_t planeId;
    float timestamp;

    // Per-beam state (72 beams max)
    float beamLoads[MAX_BEAMS_PER_SAT];       //!< PRB utilization per beam
    float beamSinr[MAX_BEAMS_PER_SAT];        //!< Avg SINR per beam (dB)
    float beamUeCount[MAX_BEAMS_PER_SAT];     //!< Active UEs per beam
    float beamInterference[MAX_BEAMS_PER_SAT]; //!< Inter-beam interference (dBm)
    float beamThroughput[MAX_BEAMS_PER_SAT];   //!< Per-beam throughput (Mbps)

    // Orbital state
    float latitude;
    float longitude;
    float altitude_km;
    float velocity_x;
    float velocity_y;
    float velocity_z;

    // ISL neighbor state
    float islDelays[MAX_ISL_NEIGHBORS];        //!< Delay to each ISL neighbor (ms)
    float islUtilization[MAX_ISL_NEIGHBORS];   //!< ISL link utilization
    uint32_t islNeighborIds[MAX_ISL_NEIGHBORS];

    // System state
    float feederLinkAvail;                     //!< 0=unavailable, 1=available
    float batteryLevel;                        //!< State of charge (0-1)
    float solarPower_W;                        //!< Current solar panel output
    float computeUtilization;                  //!< OBP compute usage (0-1)

    // Aggregate metrics
    float totalThroughput_Mbps;
    float avgSinr_dB;
    float avgLatency_ms;
    uint32_t totalActiveUes;
    uint32_t numActiveBeams;
};

/**
 * \brief Action received from Python agent to C++ (Space RIC)
 *
 * Contains the decision for Space RIC to execute.
 */
struct SpaceRicAction
{
    // Primary action
    uint32_t actionType;           //!< 0=noop, 1=beam_realloc, 2=handover,
                                   //!< 3=modcod, 4=power_ctrl, 5=beam_shutdown,
                                   //!< 6=dc_setup, 7=interference_null

    // Action targets
    uint32_t targetBeamId;
    uint32_t targetUeId;
    uint32_t targetSatId;          //!< For inter-sat actions

    // Action parameters
    float parameter1;              //!< Action-specific (e.g., new power level)
    float parameter2;              //!< Action-specific (e.g., split ratio)
    float confidence;              //!< Agent confidence (0-1)

    // Multi-action support (up to 4 simultaneous actions)
    uint32_t numActions;
    uint32_t secondaryActionTypes[3];
    uint32_t secondaryTargets[3];
    float secondaryParams[3];
};

// ============================================================================
//  Handover xApp Observation/Action
// ============================================================================

static constexpr uint32_t MAX_HO_CANDIDATES = 10;

/**
 * \brief Per-UE handover observation for ground RIC xApp
 */
struct HandoverObservation
{
    uint32_t ueId;
    float timestamp;

    // Serving cell state
    float servingSinr_dB;
    float servingTte_s;
    float servingElevation_deg;
    float servingDoppler_Hz;
    float sinrSlope;                //!< SINR trend (dB/s)
    float servingPrbUtil;
    uint32_t servingActiveUes;

    // Candidate cells
    uint32_t numCandidates;
    float candSinr[MAX_HO_CANDIDATES];
    float candTte[MAX_HO_CANDIDATES];
    float candElevation[MAX_HO_CANDIDATES];
    float candLoad[MAX_HO_CANDIDATES];
    uint32_t candSatId[MAX_HO_CANDIDATES];
    uint32_t candBeamId[MAX_HO_CANDIDATES];

    // History
    float timeSinceLastHo;
    uint32_t recentHoCount;
    uint32_t pingPongCount;
};

struct HandoverAction
{
    uint32_t selectedCandidate;     //!< 0=stay, 1..N=handover to candidate
    float confidence;
    uint32_t ueId;
};

// ============================================================================
//  Beam Hopping xApp Observation/Action
// ============================================================================

struct BeamHopObservation
{
    uint32_t satId;
    float timestamp;
    uint32_t numBeams;

    float beamDemand[MAX_BEAMS_PER_SAT];
    float beamLoad[MAX_BEAMS_PER_SAT];
    float beamAvgSinr[MAX_BEAMS_PER_SAT];
    float beamInterference[MAX_BEAMS_PER_SAT];
    float beamUeCount[MAX_BEAMS_PER_SAT];
    float beamFairness;                         //!< Current Jain's fairness index
    float totalCapacity_Mbps;
};

struct BeamHopAction
{
    float beamAllocation[MAX_BEAMS_PER_SAT];    //!< Time slot fraction per beam (0-1)
    float confidence;
};

// ============================================================================
//  Slice Manager xApp Observation/Action
// ============================================================================

static constexpr uint32_t MAX_SLICES = 3;

struct SliceObservation
{
    uint32_t gnbId;
    float timestamp;

    float slicePrbShare[MAX_SLICES];
    float sliceThroughput[MAX_SLICES];
    float sliceLatency[MAX_SLICES];
    float sliceReliability[MAX_SLICES];
    float sliceDemand[MAX_SLICES];
    float sliceSlaViolation[MAX_SLICES];       //!< 1 if SLA violated, 0 otherwise

    float totalPrbUtil;
    float totalThroughput_Mbps;
};

struct SliceAction
{
    float newPrbShare[MAX_SLICES];              //!< New PRB allocation (sum=1)
    float confidence;
};

// ============================================================================
//  TN-NTN Steering xApp Observation/Action
// ============================================================================

struct SteeringObservation
{
    uint32_t ueId;
    float timestamp;

    float tnSinr_dB;
    float ntnSinr_dB;
    float tnLoad;
    float ntnLoad;
    float tnThroughput_Mbps;
    float ntnThroughput_Mbps;
    float tnLatency_ms;
    float ntnLatency_ms;

    uint8_t sliceId;
    float qosRequirement;                      //!< Slice-specific QoS target
    uint8_t currentNetwork;                    //!< 0=TN, 1=NTN, 2=DC
};

struct SteeringAction
{
    uint8_t selectedNetwork;                   //!< 0=TN, 1=NTN, 2=DC
    float splitRatio;                          //!< TN fraction if DC (0-1)
    float confidence;
};

// ============================================================================
//  Predictive Allocation xApp Observation/Action
// ============================================================================

static constexpr uint32_t PREDICTION_WINDOW = 30;
static constexpr uint32_t PREDICTION_HORIZON = 5;

struct PredictiveObservation
{
    uint32_t beamId;
    float timestamp;

    float loadHistory[PREDICTION_WINDOW];      //!< Historical beam loads
    float sinrHistory[PREDICTION_WINDOW];      //!< Historical avg SINR
    float ueCountHistory[PREDICTION_WINDOW];   //!< Historical UE counts

    float currentLoad;
    float currentSinr;
    uint32_t currentUeCount;
};

struct PredictiveAction
{
    float predictedLoad[PREDICTION_HORIZON];   //!< Predicted loads
    float prbReservation[PREDICTION_HORIZON];  //!< Recommended PRB reservation
    float confidence;
};

// ============================================================================
//  Federated Learning Weight Exchange
// ============================================================================

static constexpr uint32_t MAX_MODEL_WEIGHTS = 4096;

struct FederatedLearningGradients
{
    uint32_t satId;
    uint32_t roundId;
    uint32_t modelVersion;
    uint32_t numWeights;
    float weights[MAX_MODEL_WEIGHTS];
    float localLoss;
    uint32_t localSamples;
};

struct FederatedLearningUpdate
{
    uint32_t roundId;
    uint32_t newModelVersion;
    uint32_t numWeights;
    float globalWeights[MAX_MODEL_WEIGHTS];
    float globalLoss;
    bool roundComplete;
};

} // namespace ns3

#endif // ORAN_NTN_MSG_INTERFACE_H
