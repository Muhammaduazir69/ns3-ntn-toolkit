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
