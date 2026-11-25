// Copyright (c) 2019 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only

#include "nr-gnb-net-device.h"

#include "bandwidth-part-gnb.h"
#include "bwp-manager-algorithm.h"
#include "bwp-manager-gnb.h"
#include "nr-epc-gnb-application.h"
#include "nr-gnb-component-carrier-manager.h"
#include "nr-gnb-mac.h"
#include "nr-gnb-phy.h"
#include "nr-gnb-rrc.h"

#include "ns3/abort.h"
#include "ns3/iana-ieee802-numbers.h"
#include "ns3/log.h"
#include "ns3/object-map.h"
#include "ns3/pointer.h"

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("NrGnbNetDevice");

NS_OBJECT_ENSURE_REGISTERED(NrGnbNetDevice);

TypeId
NrGnbNetDevice::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::NrGnbNetDevice")
            .SetParent<NrNetDevice>()
            .AddConstructor<NrGnbNetDevice>()
            .AddAttribute("NrGnbComponentCarrierManager",
                          "The component carrier manager associated to this GnbNetDevice",
                          PointerValue(),
                          MakePointerAccessor(&NrGnbNetDevice::m_componentCarrierManager),
                          MakePointerChecker<NrGnbComponentCarrierManager>())
            .AddAttribute("BandwidthPartMap",
                          "List of Bandwidth Part container.",
                          ObjectMapValue(),
                          MakeObjectMapAccessor(&NrGnbNetDevice::m_ccMap),
                          MakeObjectMapChecker<BandwidthPartGnb>())
            .AddAttribute("NrGnbRrc",
                          "The RRC layer associated with the gNB",
                          PointerValue(),
                          MakePointerAccessor(&NrGnbNetDevice::m_rrc),
                          MakePointerChecker<NrGnbRrc>())
            .AddAttribute("NrHandoverAlgorithm",
                          "The handover algorithm associated to this GnbNetDevice",
                          PointerValue(),
                          MakePointerAccessor(&NrGnbNetDevice::m_handoverAlgorithm),
                          MakePointerChecker<NrHandoverAlgorithm>());
    return tid;
}

NrGnbNetDevice::NrGnbNetDevice()
    : m_cellId(0)
{
    NS_LOG_FUNCTION(this);
}

NrGnbNetDevice::~NrGnbNetDevice()
{
    NS_LOG_FUNCTION(this);
}

Ptr<NrMacScheduler>
NrGnbNetDevice::GetScheduler(uint8_t index) const
{
    NS_LOG_FUNCTION(this);
    return m_ccMap.at(index)->GetScheduler();
}

void
NrGnbNetDevice::SetCcMap(const std::map<uint8_t, Ptr<BandwidthPartGnb>>& ccm)
{
    NS_ABORT_IF(!m_ccMap.empty());
    m_ccMap = ccm;
}

uint32_t
NrGnbNetDevice::GetCcMapSize() const
{
    return static_cast<uint32_t>(m_ccMap.size());
}

void
NrGnbNetDevice::SetNrFhControl(Ptr<NrFhControl> nrFh)
{
    NS_LOG_FUNCTION(this);
    m_nrFhControl = nrFh;
}

Ptr<NrFhControl>
NrGnbNetDevice::GetNrFhControl()
{
    NS_LOG_FUNCTION(this);
    return m_nrFhControl;
}

void
NrGnbNetDevice::RouteIngoingCtrlMsgs(const std::list<Ptr<NrControlMessage>>& msgList,
                                     uint32_t sourceBwpArfcn)
{
    NS_LOG_FUNCTION(this);

    for (const auto& msg : msgList)
    {
        uint32_t bwpArfcn = DynamicCast<BwpManagerGnb>(m_componentCarrierManager)
                                ->RouteIngoingCtrlMsgs(msg, sourceBwpArfcn);
        auto bwpIt = std::find_if(m_ccMap.begin(), m_ccMap.end(), [bwpArfcn](const auto& bwp) {
            return bwp.second->GetArfcn() == bwpArfcn;
        });
        NS_ASSERT_MSG(bwpIt != m_ccMap.end(), "gNB missing BWP with ARFCN: " << bwpArfcn);
        bwpIt->second->GetPhy()->PhyCtrlMessagesReceived(msg);
    }
}

void
NrGnbNetDevice::RouteOutgoingCtrlMsgs(const std::list<Ptr<NrControlMessage>>& msgList,
                                      uint8_t sourceBwpId)
{
    NS_LOG_FUNCTION(this);

    for (const auto& msg : msgList)
    {
        uint8_t bwpId = DynamicCast<BwpManagerGnb>(m_componentCarrierManager)
                            ->RouteOutgoingCtrlMsg(msg, sourceBwpId);
        NS_ASSERT_MSG(m_ccMap.size() > bwpId,
                      "Returned bwp " << +bwpId << " is not present. Check your configuration");
        NS_ASSERT_MSG(
            m_ccMap.at(bwpId)->GetPhy()->HasDlSlot(),
            "Returned bwp "
                << +bwpId
                << " has no DL slot, so the message can't go out. Check your configuration");
        m_ccMap.at(bwpId)->GetPhy()->EncodeCtrlMsg(msg);
    }
}

void
NrGnbNetDevice::DoInitialize()
{
    NS_LOG_FUNCTION(this);
    if (!m_isCellConfigured)
    {
        ConfigureCell();
    }
}

void
NrGnbNetDevice::DoDispose()
{
    NS_LOG_FUNCTION(this);

    m_rrc->Dispose();
    m_rrc = nullptr;
    for (const auto& it : m_ccMap)
    {
        it.second->Dispose();
    }
    m_ccMap.clear();
    m_componentCarrierManager->Dispose();
    m_componentCarrierManager = nullptr;
    NrNetDevice::DoDispose();
}

Ptr<NrGnbMac>
NrGnbNetDevice::GetMac(uint8_t index) const
{
    return m_ccMap.at(index)->GetMac();
}

Ptr<NrGnbPhy>
NrGnbNetDevice::GetPhy(uint8_t index) const
{
    NS_LOG_FUNCTION(this);
    return m_ccMap.at(index)->GetPhy();
}

Ptr<BwpManagerGnb>
NrGnbNetDevice::GetBwpManager() const
{
    return DynamicCast<BwpManagerGnb>(m_componentCarrierManager);
}

uint16_t
NrGnbNetDevice::GetCellId() const
{
    NS_LOG_FUNCTION(this);
    return m_cellId;
}

std::vector<uint16_t>
NrGnbNetDevice::GetBwpIds() const
{
    std::vector<uint16_t> bwpIds;

    bwpIds.reserve(m_ccMap.size());
    for (auto& it : m_ccMap)
    {
        bwpIds.push_back(it.second->GetBwpId());
    }
    return bwpIds;
}

void
NrGnbNetDevice::SetCellId(uint16_t cellId)
{
    NS_LOG_FUNCTION(this);
    m_cellId = cellId;
}

uint16_t
NrGnbNetDevice::GetCellId(uint8_t index) const
{
    NS_LOG_FUNCTION(this);
    return m_ccMap.at(index)->GetCellId();
}

void
NrGnbNetDevice::SetRrc(Ptr<NrGnbRrc> rrc)
{
    m_rrc = rrc;
}

Ptr<NrGnbRrc>
NrGnbNetDevice::GetRrc()
{
    return m_rrc;
}

bool
NrGnbNetDevice::DoSend(Ptr<Packet> packet, const Address& dest, uint16_t protocolNumber)
{
    NS_LOG_FUNCTION(this << packet << dest << protocolNumber);
    NS_ABORT_MSG_IF(protocolNumber != iana::ieee802numbers::IPV4 &&
                        protocolNumber != iana::ieee802numbers::IPV6 &&
                        protocolNumber != NrEpcGnbApplication::UNSTRUCTURED_SOCKET_PROTOCOL,
                    "unsupported protocol " << protocolNumber);

    NS_LOG_INFO("Forward received packet to RRC Layer");
    m_txTrace(packet, dest);

    return m_rrc->SendData(packet);
}

void
NrGnbNetDevice::ForwardUnclassifiedUp(Ptr<Packet> p)
{
    NS_LOG_FUNCTION(this << p);
    // An uplink packet that is not IP belongs to an unstructured PDU session, so it
    // rides the unstructured socket up to the gNB application, which routes
    // it by the QoS flow tag it carries rather than by its contents.
    ForwardUp(p, NrEpcGnbApplication::UNSTRUCTURED_SOCKET_PROTOCOL);
}

void
NrGnbNetDevice::UpdateConfig()
{
    NS_LOG_FUNCTION(this);
    // No longer does anything; replaced by ConfigureCell()
}

void
NrGnbNetDevice::ConfigureCell()
{
    NS_LOG_FUNCTION(this);
    NS_ASSERT_MSG(!m_isCellConfigured, "ConfigureCell() has already been called");
    NS_ASSERT_MSG(!m_ccMap.empty(), "Component carrier map is empty");
    m_isCellConfigured = true;

    // Route the outgoing (DL) messages of any UL-only carrier through the
    // cell's first DL-capable carrier, unless an explicit mapping was
    // configured (mixed TDD/FDD carrier aggregation setups pair them manually)
    auto bwpManager = DynamicCast<BwpManagerGnb>(m_componentCarrierManager);
    auto dlCcIt = std::find_if(m_ccMap.begin(), m_ccMap.end(), [](const auto& cc) {
        return cc.second->GetPhy()->HasDlSlot();
    });
    if (bwpManager && dlCcIt != m_ccMap.end())
    {
        for (const auto& cc : m_ccMap)
        {
            if (!cc.second->GetPhy()->HasDlSlot() && !bwpManager->HasOutputLink(cc.first))
            {
                bwpManager->SetOutputLink(cc.first, dlCcIt->first);
            }
        }
    }

    // Advertise the QoS flow to BWP mapping to UEs in dedicated RRC config, by
    // carrier ARFCN (BWP indices need not coincide between gNB and UE)
    if (bwpManager)
    {
        auto algo = DynamicCast<const BwpManagerAlgorithmStatic>(bwpManager->GetAlgorithm());
        if (algo)
        {
            std::map<uint8_t, uint32_t> qosFlowToBwpArfcn;
            for (const auto& [fiveQi, bwpIndex] : algo->GetBwpMap())
            {
                if (bwpIndex != BwpManagerAlgorithm::NO_BWP_ASSIGNED && bwpIndex < m_ccMap.size())
                {
                    qosFlowToBwpArfcn[fiveQi] = GetBwpArfcn(bwpIndex);
                }
            }
            m_rrc->SetQosFlowToBwpArfcn(qosFlowToBwpArfcn);
        }

        // Pair each DL carrier with the UL carrier that receives for it by
        // inverting the output links (UL-only source -> DL target), so UEs
        // learn the UL pairing of every DL-only carrier over RRC, including
        // manually paired mixed TDD/FDD carrier aggregation setups
        std::map<uint32_t, uint32_t> ulCarrierPairing;
        for (const auto& [sourceBwp, outputBwp] : bwpManager->GetOutputLinks())
        {
            if (m_ccMap.count(outputBwp) && !m_ccMap.at(sourceBwp)->GetPhy()->HasDlSlot())
            {
                ulCarrierPairing[GetBwpArfcn(outputBwp)] = GetBwpArfcn(sourceBwp);
            }
        }
        m_rrc->SetUlCarrierPairing(ulCarrierPairing);
    }

    m_rrc->ConfigureCell(m_ccMap);
    m_handoverAlgorithm->Initialize();
}

bool
NrGnbNetDevice::IsCellConfigured() const
{
    return m_isCellConfigured;
}

uint16_t
NrGnbNetDevice::GetBwpDlBandwidth(uint16_t bwpId) const
{
    if (m_rrc->HasBwpId(bwpId))
    {
        for (const auto& [key, cc] : m_ccMap)
        {
            if (cc->GetBwpId() == bwpId)
            {
                return cc->GetDlBandwidth();
            }
        }
    }
    return 0;
}

uint16_t
NrGnbNetDevice::GetBwpUlBandwidth(uint16_t bwpId) const
{
    NS_ASSERT_MSG(m_rrc->HasBwpId(bwpId), "Unknown bwpId");
    if (m_rrc->HasBwpId(bwpId))
    {
        for (const auto& [key, cc] : m_ccMap)
        {
            if (cc->GetBwpId() == bwpId)
            {
                return cc->GetUlBandwidth();
            }
        }
    }
    return 0;
}

uint32_t
NrGnbNetDevice::GetBwpArfcn(uint16_t bwpId) const
{
    NS_ASSERT_MSG(m_rrc->HasBwpId(bwpId), "Unknown bwpId");
    if (m_rrc->HasBwpId(bwpId))
    {
        for (const auto& [key, cc] : m_ccMap)
        {
            if (cc->GetBwpId() == bwpId)
            {
                return cc->GetArfcn();
            }
        }
    }
    return 0;
}

uint16_t
NrGnbNetDevice::GetArfcnBwpId(uint32_t arfcn) const
{
    for (std::size_t i = 0; i < m_ccMap.size(); i++)
    {
        if (m_ccMap.at(i)->GetArfcn() == arfcn)
        {
            return (uint16_t)i;
        }
    }
    NS_ABORT_MSG("gNB should have the searched arfcn");
}

} // namespace ns3
