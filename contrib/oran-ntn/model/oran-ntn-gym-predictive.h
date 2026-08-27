/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-only
 * Author: Muhammad Uzair
 *
 * Gymnasium Environment for Predictive Resource Allocation xApp
 *
 * Regression-based RL environment for LSTM traffic prediction.
 * Observation = sliding window of beam loads (30 steps).
 * Action = predicted loads for next 5 steps.
 * Reward = negative MSE between prediction and actual load.
 */

#ifndef ORAN_NTN_GYM_PREDICTIVE_H
#define ORAN_NTN_GYM_PREDICTIVE_H

#include "oran-ntn-types.h"

#include <ns3/ns3-ai-gym-env.h>

#include <deque>
#include <vector>

namespace ns3
{

class OranNtnXappPredictiveAlloc;
class OranNtnSatBridge;

/**
 * \ingroup oran-ntn
 * \brief Gymnasium environment for predictive-allocation RL training
 *
 * \warning AI-04 - NOT WIRED END TO END, and not exercised anywhere.
 *          No example or test constructs this class; the only reference to it
 *          outside model/ is a setter. Its emitted action type (PRB_RESERVATION) has no
 *          actuator in OranNtnHelper, so it lands in the dispatcher's default
 *          arm, is logged as "not actuated", and reports failure. Nothing this
 *          environment decides can reach a scheduler, beam manager or DC
 *          controller under any configuration.
 *
 *          Its observation and action shapes have never been checked against a
 *          live observation either, so treat the numeric contract as unverified.
 *          Of the five RL environments the README advertises, one
 *          (OranNtnGymSlice) now has an actuator path; this is not it.
 *
 * \warning Experimental: not yet exercised by any example or test; API may
 *          change. OranNtnXappPredictiveAlloc exposes a SetGymEnv() hook for
 *          this environment, but no in-tree scenario wires it yet.
 */
class OranNtnGymPredictive : public OpenGymEnv
{
  public:
    static TypeId GetTypeId();
    OranNtnGymPredictive();
    ~OranNtnGymPredictive() override;

    void SetXapp(Ptr<OranNtnXappPredictiveAlloc> xapp);
    void SetSatBridge(Ptr<OranNtnSatBridge> bridge);
    void SetCurrentBeam(uint32_t beamId);
    void SetWindowSize(uint32_t w);
    void SetHorizon(uint32_t h);

    Ptr<OpenGymSpace> GetActionSpace() override;
    Ptr<OpenGymSpace> GetObservationSpace() override;
    Ptr<OpenGymDataContainer> GetObservation() override;
    float GetReward() override;
    bool GetGameOver() override;
    std::string GetExtraInfo() override;
    bool ExecuteActions(Ptr<OpenGymDataContainer> action) override;

    void UpdateActualLoad(const std::vector<double>& actualLoads);

  protected:
    void DoDispose() override;

  private:
    Ptr<OranNtnXappPredictiveAlloc> m_xapp;
    Ptr<OranNtnSatBridge> m_satBridge;
    uint32_t m_currentBeamId;
    uint32_t m_windowSize;
    uint32_t m_horizon;

    std::deque<double> m_loadHistory;
    std::vector<double> m_lastPrediction;
    std::vector<double> m_lastActual;
    double m_lastMse;

    uint32_t m_stepCount;
    uint32_t m_maxSteps;
};

} // namespace ns3

#endif // ORAN_NTN_GYM_PREDICTIVE_H
