/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-only
 * Author: Muhammad Uzair
 */

#include "ntn-ai-interface.h"

#include <ns3/log.h>
#include <ns3/simulator.h>

// ns3-ai is an optional dependency. When the module is built with it present
// the build system defines NTN_CHO_HAS_NS3AI (see CMakeLists.txt) and the
// shared-memory bridge is compiled in; otherwise the learning-based path is
// stubbed out with a clear fatal error and the C++ CHO baselines are
// unaffected.
#ifdef NTN_CHO_HAS_NS3AI
#include <ns3/ns3-ai-msg-interface.h>
#endif

#include <algorithm> // std::clamp / std::min (do not rely on transitive includes)
#include <cmath>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("NtnAiInterface");
NS_OBJECT_ENSURE_REGISTERED(NtnAiInterface);

TypeId
NtnAiInterface::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::NtnAiInterface")
            .SetParent<Object>()
            .SetGroupName("NtnCho")
            .AddConstructor<NtnAiInterface>();
    return tid;
}

NtnAiInterface::NtnAiInterface()
    : m_initialized(false),
      m_active(false),
      m_segmentName("NtnChoAiSeg")
{
    NS_LOG_FUNCTION(this);
}

NtnAiInterface::~NtnAiInterface()
{
    NS_LOG_FUNCTION(this);
}

void
NtnAiInterface::DoDispose()
{
    if (m_active)
    {
        NotifySimulationEnd();
    }
    Object::DoDispose();
}

void
NtnAiInterface::Initialize(const std::string& segmentName, uint32_t memorySize)
{
    NS_LOG_FUNCTION(this << segmentName << memorySize);
    m_segmentName = segmentName;

#ifdef NTN_CHO_HAS_NS3AI
    // Configure ns3-ai shared memory interface
    auto& msgIf = *Singleton<Ns3AiMsgInterface>::Get();
    msgIf.SetIsMemoryCreator(true);
    msgIf.SetUseVector(false);
    msgIf.SetHandleFinish(true);
    msgIf.SetMemorySize(memorySize);
    msgIf.SetNames(segmentName,
                   "NtnObservation",
                   "NtnAction",
                   "NtnSync");

    m_initialized = true;
    m_active = true;

    NS_LOG_INFO("NTN AI Interface initialized: segment=" << segmentName
                << " size=" << memorySize);
#else
    NS_FATAL_ERROR("NtnAiInterface requires the ns3-ai module, but ntn-cho was "
                   "built without it. Add ns3-ai to contrib/ and rebuild, or "
                   "use the C++ CHO baselines (a3 / location / time / tte-aware).");
#endif
}

NtnAiAction
NtnAiInterface::GetAction(const NtnAiObservation& obs)
{
    NS_LOG_FUNCTION(this << obs.ueId << obs.simTime);
    NS_ASSERT_MSG(m_initialized, "NtnAiInterface not initialized");

#ifdef NTN_CHO_HAS_NS3AI
    auto msgInterface =
        Singleton<Ns3AiMsgInterface>::Get()
            ->GetInterface<NtnAiObservation, NtnAiAction>();

    // Send observation to Python
    msgInterface->CppSendBegin();
    auto* envStruct = msgInterface->GetCpp2PyStruct();
    *envStruct = obs;
    msgInterface->CppSendEnd();

    // Receive action from Python
    msgInterface->CppRecvBegin();
    NtnAiAction action = *msgInterface->GetPy2CppStruct();
    msgInterface->CppRecvEnd();

    NS_LOG_DEBUG("AI action: selectedAction=" << action.selectedAction
                 << " confidence=" << action.confidence);
    return action;
#else
    NS_FATAL_ERROR("NtnAiInterface::GetAction requires the ns3-ai module "
                   "(ntn-cho was built without it).");
    return NtnAiAction{}; // unreachable
#endif
}

void
NtnAiInterface::SendReward(double reward)
{
    NS_LOG_FUNCTION(this << reward);
    // Reward is embedded in next observation's previousReward field
}

void
NtnAiInterface::NotifySimulationEnd()
{
    NS_LOG_FUNCTION(this);
#ifdef NTN_CHO_HAS_NS3AI
    if (m_active)
    {
        auto msgInterface =
            Singleton<Ns3AiMsgInterface>::Get()
                ->GetInterface<NtnAiObservation, NtnAiAction>();
        msgInterface->CppSetFinished();
        m_active = false;
    }
#endif
}

bool
NtnAiInterface::IsActive() const
{
    return m_active;
}

double
NtnAiInterface::ComputeReward(bool hoSuccess,
                               double sinrImprovement,
                               double timeOfStay_s,
                               bool isPingPong,
                               double interruptionTime_ms)
{
    double reward = 0.0;

    if (!hoSuccess)
    {
        // Handover failure: large negative reward
        reward = -10.0;
    }
    else
    {
        // Base reward for successful handover
        reward = 1.0;

        // SINR improvement bonus (scaled)
        reward += std::clamp(sinrImprovement * 0.5, -3.0, 5.0);

        // Time-of-stay bonus: longer is better (diminishing returns)
        reward += std::min(timeOfStay_s / 30.0, 3.0);

        // Ping-pong penalty
        if (isPingPong)
        {
            reward -= 5.0;
        }

        // Interruption time penalty
        reward -= interruptionTime_ms / 100.0;
    }

    return reward;
}

} // namespace ns3
