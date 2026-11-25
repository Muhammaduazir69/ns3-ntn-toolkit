// Copyright (c) 2019 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only

#include "bwp-manager-ue.h"

#include "bwp-manager-algorithm.h"
#include "nr-control-messages.h"

#include "ns3/log.h"
#include "ns3/pointer.h"

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("BwpManagerUe");
NS_OBJECT_ENSURE_REGISTERED(BwpManagerUe);

BwpManagerUe::BwpManagerUe()
    : NrSimpleUeComponentCarrierManager()
{
    NS_LOG_FUNCTION(this);
}

BwpManagerUe::~BwpManagerUe()
{
    NS_LOG_FUNCTION(this);
}

void
BwpManagerUe::DoDispose()
{
    NS_LOG_FUNCTION(this);
    // This callback captures a Ptr to the UE RRC, which in turn holds callbacks capturing this
    // manager. Release it to break the reference cycle.
    m_getPrimaryUlFn = nullptr;
    m_algorithm = nullptr;
    NrSimpleUeComponentCarrierManager::DoDispose();
}

void
BwpManagerUe::SetBwpManagerAlgorithm(const Ptr<BwpManagerAlgorithm>& algorithm)
{
    NS_LOG_FUNCTION(this);
    m_algorithm = algorithm;
}

TypeId
BwpManagerUe::GetTypeId()
{
    static TypeId tid = TypeId("ns3::BwpManagerUe")
                            .SetParent<NrSimpleUeComponentCarrierManager>()
                            .SetGroupName("nr")
                            .AddConstructor<BwpManagerUe>()
                            .AddAttribute("BwpManagerAlgorithm",
                                          "The algorithm pointer",
                                          PointerValue(),
                                          MakePointerAccessor(&BwpManagerUe::m_algorithm),
                                          MakePointerChecker<BwpManagerAlgorithm>());
    return tid;
}

void
BwpManagerUe::DoTransmitBufferStatusReport(NrMacSapProvider::BufferStatusReportParameters params)
{
    NS_LOG_FUNCTION(this);
    NS_ASSERT(m_algorithm != nullptr);

    uint8_t dataBwpIndex = m_algorithm->GetBwpForQosFlow(m_lcToFlowMap.at(params.lcid));
    uint8_t bwpIndex = m_getPrimaryUlFn();
    // Control (LCID 0 and 1) always go through primary BWP index
    // Data (LCID >1) go through the BWP associated with EPS bearer,
    // if not equals to BwpManagerAlgorithm::NO_BWP_ASSIGNED
    if (params.lcid > 1 && dataBwpIndex != BwpManagerAlgorithm::NO_BWP_ASSIGNED)
    {
        bwpIndex = dataBwpIndex;
    }

    NS_LOG_DEBUG("BSR of size " << params.txQueueSize
                                << " from RLC for LCID = " << static_cast<uint32_t>(params.lcid)
                                << " traffic type " << m_lcToFlowMap.at(params.lcid)
                                << " reported to CcId " << static_cast<uint32_t>(bwpIndex));

    m_componentCarrierLcMap.at(bwpIndex).at(params.lcid)->BufferStatusReport(params);
}

std::vector<NrUeCcmRrcSapProvider::LcsConfig>
BwpManagerUe::DoAddLc(uint8_t lcId,
                      NrUeCmacSapProvider::LogicalChannelConfig lcConfig,
                      NrMacSapUser* msu)
{
    NS_LOG_FUNCTION(this);

    NS_LOG_INFO("For LC ID " << static_cast<uint32_t>(lcId) << " flow QFI "
                             << static_cast<uint32_t>(lcConfig.fiveQi) << " from priority "
                             << static_cast<uint32_t>(lcConfig.priority));

    // see nr-gnb-rrc.cc
    m_lcToFlowMap.insert(std::make_pair(lcId, lcConfig.fiveQi));

    return NrSimpleUeComponentCarrierManager::DoAddLc(lcId, lcConfig, msu);
}

NrMacSapUser*
BwpManagerUe::DoConfigureSignalBearer(uint8_t lcId,
                                      NrUeCmacSapProvider::LogicalChannelConfig lcConfig,
                                      NrMacSapUser* msu)
{
    NS_LOG_FUNCTION(this);

    m_lcToFlowMap.insert(std::make_pair(lcId, lcConfig.fiveQi));

    return NrSimpleUeComponentCarrierManager::DoConfigureSignalBearer(lcId, lcConfig, msu);
}

uint8_t
BwpManagerUe::RouteDlHarqFeedback(const DlHarqInfo& m) const
{
    NS_LOG_FUNCTION(this);

    return m.m_bwpIndex;
}

void
BwpManagerUe::SetOutputLink(uint32_t sourceBwp, uint32_t outputBwp)
{
    NS_LOG_FUNCTION(this);
    m_outputLinks[sourceBwp] = outputBwp;
    m_explicitOutputLinks.insert(sourceBwp);
}

void
BwpManagerUe::SetDefaultOutputLink(uint32_t sourceBwp, uint32_t outputBwp)
{
    NS_LOG_FUNCTION(this);
    if (m_explicitOutputLinks.find(sourceBwp) == m_explicitOutputLinks.end())
    {
        m_outputLinks[sourceBwp] = outputBwp;
    }
}

void
BwpManagerUe::ClearOutputLinks()
{
    NS_LOG_FUNCTION(this);
    m_outputLinks.clear();
    m_explicitOutputLinks.clear();
}

void
BwpManagerUe::SetBwpForQosFlow(uint8_t fiveQi, uint8_t bwpIndex)
{
    NS_LOG_FUNCTION(this);
    auto algo = DynamicCast<BwpManagerAlgorithmStatic>(m_algorithm);
    NS_ABORT_MSG_IF(algo == nullptr,
                    "SetBwpForQosFlow requires a BwpManagerAlgorithmStatic algorithm");
    algo->SetBwpForQosFlow(fiveQi, bwpIndex);
}

uint8_t
BwpManagerUe::RouteOutgoingCtrlMsg(const Ptr<NrControlMessage>& msg, uint8_t sourceBwpId) const
{
    NS_LOG_FUNCTION(this);

    NS_LOG_INFO("Msg type " << msg->GetMessageType() << " that wants to go out from UE");

    if (m_outputLinks.empty())
    {
        NS_LOG_INFO("No linked BWP, routing outgoing msg to the source: " << +sourceBwpId);
        return sourceBwpId;
    }

    auto it = m_outputLinks.find(sourceBwpId);
    if (it == m_outputLinks.end())
    {
        NS_LOG_INFO("Source BWP not in the map, routing outgoing msg to itself: " << +sourceBwpId);
        return sourceBwpId;
    }

    NS_LOG_INFO("routing outgoing msg to bwp: " << +it->second);
    return it->second;
}

uint32_t
BwpManagerUe::RouteIngoingCtrlMsg(const Ptr<NrControlMessage>& msg, uint32_t sourceBwpId) const
{
    NS_LOG_FUNCTION(this);

    NS_LOG_INFO("Msg type " << msg->GetMessageType() << " comes from BWP " << +sourceBwpId
                            << " that wants to go in the UE, goes in BWP "
                            << msg->GetSourceBwpArfcn());
    return msg->GetSourceBwpArfcn();
}

Ptr<const BwpManagerAlgorithm>
BwpManagerUe::GetAlgorithm() const
{
    return m_algorithm;
}

void
BwpManagerUe::SetGetPrimaryUlFn(std::function<uint8_t()> fn)
{
    m_getPrimaryUlFn = fn;
}

} // namespace ns3
