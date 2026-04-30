/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-only
 * Author: Muhammad Uzair
 *
 * NTN AI Interface - Shared Memory Bridge for AI-Driven Handover
 *
 * Integrates ns3-ai shared memory with NTN-CHO framework to enable
 * real-time RL/FL-based handover decision making.
 *
 * Architecture:
 *   C++ (ns-3 simulation) <--shared memory--> Python (AI agent)
 *
 *   Observation (C++ -> Python):
 *     - Serving cell SINR, elevation, Doppler, TTE
 *     - Top-K candidate cells with measurements
 *     - UE mobility state (speed, direction)
 *     - Historical handover outcomes
 *
 *   Action (Python -> C++):
 *     - Selected target cell index (0 = stay, 1-K = handover to candidate k)
 *     - Predicted optimal handover timing offset
 *
 * Supports:
 *   - DQN (Deep Q-Network) for single-agent handover selection
 *   - PPO (Proximal Policy Optimization) for continuous action spaces
 *   - Multi-Agent DRL for multi-UE cooperative handover
 *   - Federated Learning for distributed model training across UEs
 *
 * References:
 *   [1] ETRI Journal 2026 - Multi-agent DRL for LEO-TN handover
 *   [2] MDPI Electronics 2026 - Rainbow-DQN for LEO handover
 *   [3] Springer Wireless Networks 2026 - Federated MARL for LEO
 */

#ifndef NTN_AI_INTERFACE_H
#define NTN_AI_INTERFACE_H

#include <ns3/object.h>
#include <ns3/nstime.h>

#include <cstdint>
#include <vector>

namespace ns3
{

/**
 * \brief Maximum number of candidate cells in observation space
 */
static constexpr uint32_t NTN_AI_MAX_CANDIDATES = 8;

/**
 * \brief Observation struct sent from C++ (ns-3) to Python (AI agent)
 *
 * This is placed in shared memory via ns3-ai's Ns3AiMsgInterface.
 * All fields are plain-old-data (POD) for shared memory compatibility.
 */
struct NtnAiObservation
{
    // UE identification
    uint32_t ueId;
    double simTime;

    // Serving cell state
    uint32_t servingSatId;
    double servingSinr_dB;
    double servingRsrp_dBm;
    double servingElevation_deg;
    double servingDoppler_Hz;
    double servingTte_s;           // Remaining beam coverage time
    double servingDelay_ms;

    // UE mobility
    double ueSpeed_mps;
    double ueDirection_deg;        // Heading in degrees
    double ueLat;
    double ueLon;

    // Candidate cells (up to NTN_AI_MAX_CANDIDATES)
    uint32_t numCandidates;
    struct CandidateObs
    {
        uint32_t satId;
        double sinr_dB;
        double rsrp_dBm;
        double elevation_deg;
        double doppler_Hz;
        double tte_s;
        double delay_ms;
        double distToServing_km;   // Distance between candidate and serving sat
    } candidates[NTN_AI_MAX_CANDIDATES];

    // Historical context (for LSTM/attention models)
    uint32_t recentHoCount;        // HOs in last 60 seconds
    uint32_t recentHoFailures;     // Failed HOs in last 60 seconds
    uint32_t recentPingPongs;      // Ping-pongs in last 60 seconds
    double avgSinrLast30s;         // Average serving SINR
    double avgTosLast5HOs;         // Average time-of-stay for last 5 HOs

    // Reward from previous action
    double previousReward;
};

/**
 * \brief Action struct sent from Python (AI agent) to C++ (ns-3)
 */
struct NtnAiAction
{
    // Handover decision
    uint32_t selectedAction;       // 0 = stay, 1-K = handover to candidate[k-1]
    double confidence;             // Agent's confidence in action (0-1)

    // Optional: predicted timing (for proactive handover)
    double predictedOptimalHoTime; // Seconds from now to execute HO

    // Federated learning: gradient info (for FL aggregation)
    uint32_t modelVersion;         // Current model version
    double localLoss;              // Training loss for FL monitoring
};

/**
 * \ingroup ntn-cho
 * \brief AI-driven handover interface using ns3-ai shared memory
 *
 * This class wraps the ns3-ai shared memory interface to provide
 * a clean API for AI-driven NTN handover decisions.
 */
class NtnAiInterface : public Object
{
  public:
    static TypeId GetTypeId();
    NtnAiInterface();
    ~NtnAiInterface() override;

    /**
     * \brief Initialize the shared memory interface
     * \param segmentName Name of shared memory segment
     * \param memorySize Size of shared memory in bytes
     */
    void Initialize(const std::string& segmentName = "NtnChoAiSeg",
                    uint32_t memorySize = 65536);

    /**
     * \brief Send observation to Python agent and receive action
     * \param obs The observation to send
     * \return The action received from the agent
     */
    NtnAiAction GetAction(const NtnAiObservation& obs);

    /**
     * \brief Send final reward signal (end of episode)
     * \param reward Final reward value
     */
    void SendReward(double reward);

    /**
     * \brief Signal simulation end to Python side
     */
    void NotifySimulationEnd();

    /**
     * \brief Check if AI interface is active
     */
    bool IsActive() const;

    /**
     * \brief Compute reward for a handover outcome
     *
     * Reward function based on:
     *   + Positive: successful HO to better cell, long time-of-stay
     *   - Negative: HO failure, ping-pong, service interruption
     *
     * \param hoSuccess Whether handover succeeded
     * \param sinrImprovement SINR difference (target - source)
     * \param timeOfStay_s Time spent in previous cell
     * \param isPingPong Whether this is a ping-pong event
     * \param interruptionTime_ms Service interruption duration
     * \return Computed reward value
     */
    static double ComputeReward(bool hoSuccess,
                                double sinrImprovement,
                                double timeOfStay_s,
                                bool isPingPong,
                                double interruptionTime_ms);

  protected:
    void DoDispose() override;

  private:
    bool m_initialized;
    bool m_active;
    std::string m_segmentName;
};

} // namespace ns3

#endif // NTN_AI_INTERFACE_H
