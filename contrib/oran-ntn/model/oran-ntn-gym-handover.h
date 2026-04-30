/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-only
 * Author: Muhammad Uzair
 *
 * Gymnasium Environment for Handover xApp
 *
 * OpenGymEnv subclass providing RL interface for the HO Prediction xApp.
 * Observation space encodes serving cell state, candidate cells, and
 * handover history. Action space selects target cell or stay decision.
 * Reward balances QoS improvement, ping-pong avoidance, and stability.
 *
 * Novel reward shaping:
 *   - TTE-weighted SINR improvement bonus
 *   - Exponential ping-pong penalty with decay
 *   - Proactive handover lead-time bonus
 *   - Elevation-continuity reward for smooth satellite transitions
 */

#ifndef ORAN_NTN_GYM_HANDOVER_H
#define ORAN_NTN_GYM_HANDOVER_H

#include "oran-ntn-types.h"

#include <ns3/ns3-ai-gym-env.h>

#include <deque>
#include <vector>

namespace ns3
{

class OranNtnXappHoPredict;
class OranNtnSatBridge;

/**
 * \ingroup oran-ntn
 * \brief Gymnasium environment for handover RL training
 */
class OranNtnGymHandover : public OpenGymEnv
{
  public:
    static TypeId GetTypeId();
    OranNtnGymHandover();
    ~OranNtnGymHandover() override;

    void SetXapp(Ptr<OranNtnXappHoPredict> xapp);
    void SetSatBridge(Ptr<OranNtnSatBridge> bridge);
    void SetMaxCandidates(uint32_t n);

    // OpenGymEnv interface
    Ptr<OpenGymSpace> GetActionSpace() override;
    Ptr<OpenGymSpace> GetObservationSpace() override;
    Ptr<OpenGymDataContainer> GetObservation() override;
    float GetReward() override;
    bool GetGameOver() override;
    std::string GetExtraInfo() override;
    bool ExecuteActions(Ptr<OpenGymDataContainer> action) override;

    /**
     * \brief Set the UE to observe/control in current step
     */
    void SetCurrentUe(uint32_t ueId);

    /**
     * \brief Update state after xApp decision cycle
     */
    void UpdatePostAction(double newSinr, double newTte, bool pingPongDetected);

  protected:
    void DoDispose() override;

  private:
    Ptr<OranNtnXappHoPredict> m_xapp;
    Ptr<OranNtnSatBridge> m_satBridge;
    uint32_t m_maxCandidates;
    uint32_t m_currentUeId;

    // State for reward computation
    double m_preActionSinr;
    double m_postActionSinr;
    double m_preActionTte;
    double m_postActionTte;
    bool m_pingPongDetected;
    bool m_handoverExecuted;
    double m_hoLeadTime;

    // Episode tracking
    uint32_t m_stepCount;
    uint32_t m_maxSteps;
    double m_cumulativeReward;

    // Reward shaping weights
    double m_sinrImprovementWeight;
    double m_pingPongPenaltyWeight;
    double m_hoCostWeight;
    double m_tteBonusWeight;
    double m_elevationContinuityWeight;
};

} // namespace ns3

#endif // ORAN_NTN_GYM_HANDOVER_H
