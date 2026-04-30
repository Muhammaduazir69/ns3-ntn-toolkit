/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-only
 * Author: Muhammad Uzair
 *
 * Gymnasium Environment for Slice Manager xApp
 */

#ifndef ORAN_NTN_GYM_SLICE_H
#define ORAN_NTN_GYM_SLICE_H

#include "oran-ntn-types.h"

#include <ns3/ns3-ai-gym-env.h>

#include <vector>

namespace ns3
{

class OranNtnXappSliceManager;
class OranNtnSatBridge;

class OranNtnGymSlice : public OpenGymEnv
{
  public:
    static TypeId GetTypeId();
    OranNtnGymSlice();
    ~OranNtnGymSlice() override;

    void SetXapp(Ptr<OranNtnXappSliceManager> xapp);
    void SetSatBridge(Ptr<OranNtnSatBridge> bridge);
    void SetNumSlices(uint32_t n);
    void SetCurrentGnb(uint32_t gnbId);

    Ptr<OpenGymSpace> GetActionSpace() override;
    Ptr<OpenGymSpace> GetObservationSpace() override;
    Ptr<OpenGymDataContainer> GetObservation() override;
    float GetReward() override;
    bool GetGameOver() override;
    std::string GetExtraInfo() override;
    bool ExecuteActions(Ptr<OpenGymDataContainer> action) override;

    void UpdatePostAction(double slaCompliance, uint32_t violations, double efficiency);

  protected:
    void DoDispose() override;

  private:
    Ptr<OranNtnXappSliceManager> m_xapp;
    Ptr<OranNtnSatBridge> m_satBridge;
    uint32_t m_numSlices;
    uint32_t m_currentGnbId;

    double m_postSlaCompliance;
    uint32_t m_postViolations;
    double m_postEfficiency;

    uint32_t m_stepCount;
    uint32_t m_maxSteps;

    double m_slaWeight;
    double m_violationPenalty;
    double m_efficiencyWeight;
};

} // namespace ns3

#endif // ORAN_NTN_GYM_SLICE_H
