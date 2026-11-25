// Copyright (c) 2024 Seoul National University (SNU)
// Copyright (c) 2024 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only

#include "nr-mac-scheduler-ai-ns3-gym-env.h"

#ifdef HAVE_OPENGYM

namespace ns3
{
NS_LOG_COMPONENT_DEFINE("NrMacSchedulerAiNs3GymEnv");
NS_OBJECT_ENSURE_REGISTERED(NrMacSchedulerAiNs3GymEnv);

NrMacSchedulerAiNs3GymEnv::NrMacSchedulerAiNs3GymEnv()
{
    NS_LOG_FUNCTION(this);
}

NrMacSchedulerAiNs3GymEnv::NrMacSchedulerAiNs3GymEnv(uint32_t numUes)
{
    NS_LOG_FUNCTION(this);
    m_numUes = numUes;
}

NrMacSchedulerAiNs3GymEnv::~NrMacSchedulerAiNs3GymEnv()
{
    NS_LOG_FUNCTION(this);
}

TypeId
NrMacSchedulerAiNs3GymEnv::GetTypeId()
{
    static TypeId tid = TypeId("NrMacSchedulerAiNs3GymEnv")
                            .SetParent<OpenGymEnv>()
                            .AddConstructor<NrMacSchedulerAiNs3GymEnv>();
    return tid;
}

void
NrMacSchedulerAiNs3GymEnv::DoDispose()
{
    NS_LOG_FUNCTION(this);
}

Ptr<OpenGymSpace>
NrMacSchedulerAiNs3GymEnv::GetActionSpace()
{
    NS_LOG_FUNCTION(this);
    float low = 0.0;
    float high = 1.0;
    std::vector<uint32_t> shape = {m_numUes};
    std::string dtype = TypeNameGet<float>();
    return CreateObject<OpenGymBoxSpace>(low, high, shape, dtype);
}

Ptr<OpenGymSpace>
NrMacSchedulerAiNs3GymEnv::GetObservationSpace()
{
    NS_LOG_FUNCTION(this);
    float low = 0.0;
    float high = 1000.0;
    std::vector<uint32_t> shape = {
        m_numUes,
        4,
    };
    std::string dtype = TypeNameGet<float>();
    return CreateObject<OpenGymBoxSpace>(low, high, shape, dtype);
}

bool
NrMacSchedulerAiNs3GymEnv::GetGameOver()
{
    NS_LOG_FUNCTION(this);
    return m_gameOver;
}

Ptr<OpenGymDataContainer>
NrMacSchedulerAiNs3GymEnv::GetObservation()
{
    NS_LOG_FUNCTION(this);
    std::vector<uint32_t> shape = {
        m_numUes,
        4,
    };
    Ptr<OpenGymBoxContainer<float>> observation = CreateObject<OpenGymBoxContainer<float>>(shape);
    for (auto& obs : m_observation)
    {
        observation->AddValue(obs.rnti);
        observation->AddValue(obs.priority);
        observation->AddValue(obs.holDelay);
        observation->AddValue(obs.avgTput);
    }
    return observation;
}

float
NrMacSchedulerAiNs3GymEnv::GetReward()
{
    NS_LOG_FUNCTION(this);
    return m_reward;
}

std::string
NrMacSchedulerAiNs3GymEnv::GetExtraInfo()
{
    NS_LOG_FUNCTION(this);
    return m_extraInfo;
}

bool
NrMacSchedulerAiNs3GymEnv::ExecuteActions(Ptr<OpenGymDataContainer> action)
{
    NS_LOG_FUNCTION(this);
    Ptr<OpenGymBoxContainer<float>> actionBox = DynamicCast<OpenGymBoxContainer<float>>(action);
    std::vector<float> actionData = actionBox->GetData();
    NrMacSchedulerUeInfoAi::Weights weights;

    for (uint32_t i = 0; i < m_numUes; i++)
    {
        weights[m_observation[i].rnti] = actionData[i];
    }

    m_updateAllUeWeightsFn(weights);
    return true;
}

void
NrMacSchedulerAiNs3GymEnv::NotifyCurrentIteration(
    const std::vector<NrMacSchedulerUeInfoAi::UeObservation>& observations,
    bool isGameOver,
    float reward,
    const std::string& extraInfo,
    const NrMacSchedulerUeInfoAi::UpdateAllUeWeightsFn& updateAllUeWeightsFn)
{
    NS_LOG_FUNCTION(this);
    m_observation = observations;
    m_gameOver = isGameOver;
    m_reward = reward;
    m_extraInfo = extraInfo;
    m_updateAllUeWeightsFn = updateAllUeWeightsFn;
    Notify();
}

} // namespace ns3

#endif // HAVE_OPENGYM
