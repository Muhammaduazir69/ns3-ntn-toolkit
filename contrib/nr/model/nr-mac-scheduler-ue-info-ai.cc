// Copyright (c) 2024 Seoul National University (SNU)
// Copyright (c) 2024 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only

#include "nr-mac-scheduler-ue-info-ai.h"

#include "ns3/log.h"

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("NrMacSchedulerUeInfoAi");

void
NrMacSchedulerUeInfoAi::UpdateDlAiMetric(bool isAssigned,
                                         const NrMacSchedulerNs3::FTResources& totAssigned,
                                         double timeWindow)
{
    NS_LOG_FUNCTION(this);
    NrMacSchedulerUeInfoQos::UpdateDlQosMetric(totAssigned, timeWindow);
    m_selectedDl = isAssigned;
    m_currTputDl = (m_dlTbSize * 8 / m_slotPeriod.GetSeconds()) / 1e6;
    m_avgTputDl =
        ((1.0 - (1.0 / timeWindow)) * m_lastAvgTputDl) + ((1.0 / timeWindow) * m_currTputDl);
    // printf("UE %d is assigned %d\n", m_rnti, isAssigned);
}

void
NrMacSchedulerUeInfoAi::UpdateUlAiMetric(bool isAssigned,
                                         const NrMacSchedulerNs3::FTResources& totAssigned,
                                         double timeWindow)
{
    NS_LOG_FUNCTION(this);
    NrMacSchedulerUeInfoQos::UpdateUlQosMetric(totAssigned, timeWindow);
    m_selectedUl = isAssigned;
}

NrMacSchedulerUeInfoAi::UeObservation
NrMacSchedulerUeInfoAi::GetDlObservation()
{
    NS_LOG_FUNCTION(this);
    bool isGbrMain = false;
    NrMacSchedulerLC* mainLc;
    std::vector<NrMacSchedulerLC*> gbrLcs;
    std::vector<NrMacSchedulerLC*> nonGbrLcs;
    uint8_t numLcs = 0;
    uint64_t sumGuaranteedBytes = 0;

    for (const auto& ueLcg : m_dlLCG)
    {
        std::vector<uint8_t> ueActiveLCs = ueLcg.second->GetActiveLCIds();

        for (const auto lcId : ueActiveLCs)
        {
            std::unique_ptr<NrMacSchedulerLC>& LCPtr = ueLcg.second->GetLC(lcId);
            if (LCPtr->m_resourceType == nr::LogicalChannelConfigListElement_s::QBT_DGBR ||
                LCPtr->m_resourceType == nr::LogicalChannelConfigListElement_s::QBT_GBR)
            {
                gbrLcs.emplace_back(LCPtr.get());
            }
            else
            {
                nonGbrLcs.emplace_back(LCPtr.get());
            }
            numLcs++;
        }
    }

    std::sort(gbrLcs.begin(), gbrLcs.end(), [](NrMacSchedulerLC* a, NrMacSchedulerLC* b) {
        return a->m_priority < b->m_priority;
    });

    for (auto& gbrLc : gbrLcs)
    {
        sumGuaranteedBytes += (gbrLc->m_eRabGuaranteedBitrateDl / 8);
        if (sumGuaranteedBytes > m_dlTbSize)
        {
            mainLc = gbrLc;
            isGbrMain = true;
            break;
        }
    }

    if (!isGbrMain)
    {
        std::sort(nonGbrLcs.begin(), nonGbrLcs.end(), [](NrMacSchedulerLC* a, NrMacSchedulerLC* b) {
            uint8_t aPriority = a->m_priority == 0 ? 100 : a->m_priority;
            uint8_t bPriority = b->m_priority == 0 ? 100 : b->m_priority;
            double aMetric = (1.0 + static_cast<double>(a->m_rlcTransmissionQueueHolDelay)) /
                             static_cast<double>(a->m_delayBudget.GetMilliSeconds()) /
                             static_cast<double>(aPriority);
            double bMetric = (1.0 + static_cast<double>(b->m_rlcTransmissionQueueHolDelay)) /
                             static_cast<double>(b->m_delayBudget.GetMilliSeconds()) /
                             static_cast<double>(bPriority);
            return aMetric > bMetric;
        });
        mainLc = nonGbrLcs.front();
    }

    m_selectedLcResTypeDl = mainLc->m_resourceType;

    return {m_rnti,
            numLcs,
            static_cast<uint8_t>(mainLc->m_id),
            mainLc->m_fiveQi,
            mainLc->m_priority,
            mainLc->m_rlcTransmissionQueueHolDelay,
            m_dlTbSize,
            static_cast<float>(m_avgTputDl)};
}

NrMacSchedulerUeInfoAi::UeObservation
NrMacSchedulerUeInfoAi::GetUlObservation()
{
    NS_LOG_FUNCTION(this);
    std::vector<NrMacSchedulerLC*> activeLcs;
    uint8_t numLcs = 0;

    for (const auto& ueLcg : m_ulLCG)
    {
        std::vector<uint8_t> ueActiveLCs = ueLcg.second->GetActiveLCIds();

        for (const auto lcId : ueActiveLCs)
        {
            std::unique_ptr<NrMacSchedulerLC>& LCPtr = ueLcg.second->GetLC(lcId);
            activeLcs.push_back(LCPtr.get());
            numLcs++;
        }
    }

    std::sort(activeLcs.begin(), activeLcs.end(), [](NrMacSchedulerLC* a, NrMacSchedulerLC* b) {
        double aMetric = (1.0 + static_cast<double>(a->m_rlcTransmissionQueueHolDelay)) /
                         static_cast<double>(a->m_delayBudget.GetMilliSeconds()) /
                         static_cast<double>(std::min(a->m_priority, static_cast<uint8_t>(100)));
        double bMetric = (1.0 + static_cast<double>(b->m_rlcTransmissionQueueHolDelay)) /
                         static_cast<double>(b->m_delayBudget.GetMilliSeconds()) /
                         static_cast<double>(std::min(b->m_priority, static_cast<uint8_t>(100)));
        return aMetric > bMetric;
    });

    m_selectedLcResTypeUl = activeLcs.front()->m_resourceType;

    return {m_rnti,
            numLcs,
            static_cast<uint8_t>(activeLcs.front()->m_id),
            activeLcs.front()->m_fiveQi,
            activeLcs.front()->m_priority,
            activeLcs.front()->m_rlcTransmissionQueueHolDelay,
            m_ulTbSize,
            static_cast<float>(m_avgTputUl)};
}

void
NrMacSchedulerUeInfoAi::UpdateDlWeight(double weight)
{
    m_weightDl = weight;
}

void
NrMacSchedulerUeInfoAi::UpdateUlWeight(double weight)
{
    m_weightUl = weight;
}

float
NrMacSchedulerUeInfoAi::GetDlReward()
{
    float reward = 0.0;
    float numLcs = 0;
    bool isWeighted = (m_selectedLcResTypeDl == nr::LogicalChannelConfigListElement_s::QBT_DGBR ||
                       m_selectedLcResTypeDl == nr::LogicalChannelConfigListElement_s::QBT_GBR);

    for (const auto& ueLcg : m_dlLCG)
    {
        std::vector<uint8_t> ueActiveLCs = ueLcg.second->GetActiveLCIds();

        for (const auto lcId : ueActiveLCs)
        {
            std::unique_ptr<NrMacSchedulerLC>& LCPtr = ueLcg.second->GetLC(lcId);
            float lcReward = (1.0 + LCPtr->m_rlcTransmissionQueueHolDelay) /
                             static_cast<float>(LCPtr->m_delayBudget.GetMilliSeconds()) *
                             (100.0 / LCPtr->m_priority);
            if (LCPtr->m_resourceType == nr::LogicalChannelConfigListElement_s::QBT_DGBR ||
                LCPtr->m_resourceType == nr::LogicalChannelConfigListElement_s::QBT_GBR)
            {
                if (isWeighted)
                {
                    lcReward *= (1.0 / LCPtr->m_priority);
                }
            }
            reward += lcReward;
            numLcs++;
        }
    }
    reward *= 1.0 / (m_avgTputDl / numLcs + 1E-6);
    return m_selectedDl ? reward : -reward;
}

float
NrMacSchedulerUeInfoAi::GetUlReward()
{
    float reward = 0.0;
    float numLcs = 0;
    bool isWeighted = (m_selectedLcResTypeUl == nr::LogicalChannelConfigListElement_s::QBT_DGBR ||
                       m_selectedLcResTypeUl == nr::LogicalChannelConfigListElement_s::QBT_GBR);
    for (const auto& ueLcg : m_ulLCG)
    {
        std::vector<uint8_t> ueActiveLCs = ueLcg.second->GetActiveLCIds();

        for (const auto lcId : ueActiveLCs)
        {
            std::unique_ptr<NrMacSchedulerLC>& LCPtr = ueLcg.second->GetLC(lcId);
            float lcReward = (1.0 + LCPtr->m_rlcTransmissionQueueHolDelay) /
                             static_cast<float>(LCPtr->m_delayBudget.GetMilliSeconds()) *
                             (100.0 / LCPtr->m_priority);
            if (LCPtr->m_resourceType == nr::LogicalChannelConfigListElement_s::QBT_DGBR ||
                LCPtr->m_resourceType == nr::LogicalChannelConfigListElement_s::QBT_GBR)
            {
                if (isWeighted)
                {
                    lcReward *= (1.0 / LCPtr->m_priority);
                }
            }
            reward += lcReward;
            numLcs++;
        }
    }
    reward *= 1.0 / (m_avgTputUl / numLcs + 1E-6);
    return m_selectedUl ? reward : -reward;
}

} // namespace ns3
