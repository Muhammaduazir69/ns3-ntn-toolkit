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

/**
 * \ingroup oran-ntn
 * \brief Gymnasium environment for slice-management RL training
 *
 * \note AI-04: this is the one RL environment with an actuator path. Its
 *       SLICE_PRB_ALLOCATION actions are dispatched by OranNtnHelper to the
 *       callback installed via SetSliceActuator - point that at
 *       SliceOrchestratorXapp::StepWithShares and the per-slice prbAllocated
 *       moves measurably. With no actuator wired the action is reported as
 *       NOT actuated rather than silently accepted. The other three
 *       environments remain unwired; their headers say so.
 *
 * \warning Experimental: not yet exercised by any example or test; API may
 *          change.
 */
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

    /// AI-03: kept so a scenario that measures the post-action state out of
    /// band can still override what GetObservation() latches.
    void UpdatePostAction(double slaCompliance, uint32_t violations, double efficiency);

    /// TS 22.261 latency bound in ms for a slice id (0 eMBB, 1 URLLC, 2 mMTC).
    static double SliceLatencyBoundMs(uint32_t sliceId);

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
