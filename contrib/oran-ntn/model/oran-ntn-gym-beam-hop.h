/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-only
 * Author: Muhammad Uzair
 *
 * Gymnasium Environment for Beam Hopping xApp
 */

#ifndef ORAN_NTN_GYM_BEAM_HOP_H
#define ORAN_NTN_GYM_BEAM_HOP_H

#include "oran-ntn-types.h"

#include <ns3/ns3-ai-gym-env.h>

#include <vector>

namespace ns3
{

class OranNtnXappBeamHop;
class OranNtnSatBridge;

class OranNtnGymBeamHop : public OpenGymEnv
{
  public:
    static TypeId GetTypeId();
    OranNtnGymBeamHop();
    ~OranNtnGymBeamHop() override;

    void SetXapp(Ptr<OranNtnXappBeamHop> xapp);
    void SetSatBridge(Ptr<OranNtnSatBridge> bridge);
    void SetNumBeams(uint32_t n);
    void SetCurrentSatellite(uint32_t satId);

    Ptr<OpenGymSpace> GetActionSpace() override;
    Ptr<OpenGymSpace> GetObservationSpace() override;
    Ptr<OpenGymDataContainer> GetObservation() override;
    float GetReward() override;
    bool GetGameOver() override;
    std::string GetExtraInfo() override;
    bool ExecuteActions(Ptr<OpenGymDataContainer> action) override;

    void UpdatePostAction(double fairness, double throughput, double energyEff);

  protected:
    void DoDispose() override;

  private:
    Ptr<OranNtnXappBeamHop> m_xapp;
    Ptr<OranNtnSatBridge> m_satBridge;
    uint32_t m_numBeams;
    uint32_t m_currentSatId;

    double m_postFairness;
    double m_postThroughput;
    double m_postEnergyEff;

    uint32_t m_stepCount;
    uint32_t m_maxSteps;

    double m_fairnessWeight;
    double m_throughputWeight;
    double m_energyWeight;
};

} // namespace ns3

#endif // ORAN_NTN_GYM_BEAM_HOP_H
