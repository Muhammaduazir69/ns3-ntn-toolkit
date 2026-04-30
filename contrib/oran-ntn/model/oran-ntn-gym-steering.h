/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-only
 * Author: Muhammad Uzair
 *
 * Gymnasium Environment for TN-NTN Traffic Steering xApp
 */

#ifndef ORAN_NTN_GYM_STEERING_H
#define ORAN_NTN_GYM_STEERING_H

#include "oran-ntn-types.h"

#include <ns3/ns3-ai-gym-env.h>

namespace ns3
{

class OranNtnXappTnNtnSteering;
class OranNtnSatBridge;

class OranNtnGymSteering : public OpenGymEnv
{
  public:
    static TypeId GetTypeId();
    OranNtnGymSteering();
    ~OranNtnGymSteering() override;

    void SetXapp(Ptr<OranNtnXappTnNtnSteering> xapp);
    void SetSatBridge(Ptr<OranNtnSatBridge> bridge);
    void SetCurrentUe(uint32_t ueId);

    Ptr<OpenGymSpace> GetActionSpace() override;
    Ptr<OpenGymSpace> GetObservationSpace() override;
    Ptr<OpenGymDataContainer> GetObservation() override;
    float GetReward() override;
    bool GetGameOver() override;
    std::string GetExtraInfo() override;
    bool ExecuteActions(Ptr<OpenGymDataContainer> action) override;

    void UpdatePostAction(double latency, double throughput, bool switchOccurred);

  protected:
    void DoDispose() override;

  private:
    Ptr<OranNtnXappTnNtnSteering> m_xapp;
    Ptr<OranNtnSatBridge> m_satBridge;
    uint32_t m_currentUeId;

    double m_postLatency;
    double m_postThroughput;
    bool m_switchOccurred;

    uint32_t m_stepCount;
    uint32_t m_maxSteps;
};

} // namespace ns3

#endif // ORAN_NTN_GYM_STEERING_H
