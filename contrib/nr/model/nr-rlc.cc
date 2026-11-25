// Copyright (c) 2011 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only
//
// Author: Nicola Baldo <nbaldo@cttc.es>

#include "nr-rlc.h"

#include "nr-rlc-am-header.h"
#include "nr-rlc-header.h"
#include "nr-rlc-sap.h"
#include "nr-rlc-tag.h"
// #include "nr-mac-sap.h"
// #include "nr-ff-mac-sched-sap.h"

#include "ns3/log.h"
#include "ns3/simulator.h"

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("NrRlc");

/// NrRlcSpecificNrMacSapUser class
class NrRlcSpecificNrMacSapUser : public NrMacSapUser
{
  public:
    /**
     * Constructor
     *
     * @param rlc the RLC
     */
    NrRlcSpecificNrMacSapUser(NrRlc* rlc);

    // Interface implemented from NrMacSapUser
    void NotifyTxOpportunity(NrMacSapUser::TxOpportunityParameters params) override;
    void NotifyHarqDeliveryFailure() override;
    void ReceivePdu(NrMacSapUser::ReceivePduParameters params) override;

  private:
    NrRlcSpecificNrMacSapUser();
    NrRlc* m_rlc; ///< the RLC
};

NrRlcSpecificNrMacSapUser::NrRlcSpecificNrMacSapUser(NrRlc* rlc)
    : m_rlc(rlc)
{
}

NrRlcSpecificNrMacSapUser::NrRlcSpecificNrMacSapUser()
{
}

void
NrRlcSpecificNrMacSapUser::NotifyTxOpportunity(TxOpportunityParameters params)
{
    m_rlc->DoNotifyTxOpportunity(params);
}

void
NrRlcSpecificNrMacSapUser::NotifyHarqDeliveryFailure()
{
    m_rlc->DoNotifyHarqDeliveryFailure();
}

void
NrRlcSpecificNrMacSapUser::ReceivePdu(NrMacSapUser::ReceivePduParameters params)
{
    m_rlc->DoReceivePdu(params);
}

///////////////////////////////////////

NS_OBJECT_ENSURE_REGISTERED(NrRlc);

/// Monotonic source of unique RLC entity identifiers (see NrRlc::m_rlcEntityId)
static uint32_t g_rlcEntityIdCounter = 0;

NrRlc::NrRlc()
    : m_rlcSapUser(nullptr),
      m_macSapProvider(nullptr),
      m_rnti(0),
      m_lcid(0),
      m_rlcEntityId(++g_rlcEntityIdCounter)
{
    NS_LOG_FUNCTION(this);
    m_rlcSapProvider = new NrRlcSpecificNrRlcSapProvider<NrRlc>(this);
    m_macSapUser = new NrRlcSpecificNrMacSapUser(this);
}

NrRlc::~NrRlc()
{
    NS_LOG_FUNCTION(this);
}

TypeId
NrRlc::GetTypeId()
{
    static TypeId tid = TypeId("ns3::NrRlc")
                            .SetParent<Object>()
                            .SetGroupName("Nr")
                            .AddTraceSource("TxPDU",
                                            "PDU transmission notified to the MAC.",
                                            MakeTraceSourceAccessor(&NrRlc::m_txPdu),
                                            "ns3::NrRlc::NotifyTxTracedCallback")
                            .AddTraceSource("RxPDU",
                                            "PDU received.",
                                            MakeTraceSourceAccessor(&NrRlc::m_rxPdu),
                                            "ns3::NrRlc::ReceiveTracedCallback")
                            .AddTraceSource("TxDrop",
                                            "Trace source indicating a packet "
                                            "has been dropped before transmission",
                                            MakeTraceSourceAccessor(&NrRlc::m_txDropTrace),
                                            "ns3::Packet::TracedCallback");
    return tid;
}

void
NrRlc::DoDispose()
{
    NS_LOG_FUNCTION(this);
    delete (m_rlcSapProvider);
    delete (m_macSapUser);
}

void
NrRlc::ReestablishRxSide()
{
    NS_LOG_FUNCTION(this);
}

bool
NrRlc::AcceptPduFromPeerEntity(uint32_t txEntityId)
{
    if (txEntityId == 0)
    {
        return true;
    }

    if (txEntityId < m_peerRlcEntityId)
    {
        NS_LOG_WARN("Discarding PDU from old-epoch RLC entity "
                    << txEntityId << " (current peer entity " << m_peerRlcEntityId << ")");
        return false;
    }

    if (txEntityId > m_peerRlcEntityId)
    {
        if (m_peerRlcEntityId != 0)
        {
            NS_LOG_WARN("Peer RLC entity re-established (" << m_peerRlcEntityId << " -> "
                                                           << txEntityId
                                                           << "): re-establishing RX side");
            ReestablishRxSide();
        }
        m_peerRlcEntityId = txEntityId;
    }

    return true;
}

void
NrRlc::SetRnti(uint16_t rnti)
{
    NS_LOG_FUNCTION(this << (uint32_t)rnti);
    m_rnti = rnti;
}

void
NrRlc::SetLcId(uint8_t lcId)
{
    NS_LOG_FUNCTION(this << (uint32_t)lcId);
    m_lcid = lcId;
}

void
NrRlc::SetPacketDelayBudgetMs(uint16_t packetDelayBudget)
{
    NS_LOG_FUNCTION(this << +packetDelayBudget);
    m_packetDelayBudgetMs = packetDelayBudget;
}

void
NrRlc::SetNrRlcSapUser(NrRlcSapUser* s)
{
    NS_LOG_FUNCTION(this << s);
    m_rlcSapUser = s;
}

NrRlcSapProvider*
NrRlc::GetNrRlcSapProvider()
{
    NS_LOG_FUNCTION(this);
    return m_rlcSapProvider;
}

void
NrRlc::SetNrMacSapProvider(NrMacSapProvider* s)
{
    NS_LOG_FUNCTION(this << s);
    m_macSapProvider = s;
}

NrMacSapUser*
NrRlc::GetNrMacSapUser()
{
    NS_LOG_FUNCTION(this);
    return m_macSapUser;
}

void
NrRlc::SetMaxRetxReachedCallback(Callback<void> cb)
{
    NS_LOG_FUNCTION(this);
    m_maxRetxReachedCallback = cb;
}

template <typename RlcHeader>
void
NrRlc::SplitDataFields(RlcHeader& header, Ptr<Packet> packet, std::list<Ptr<Packet>>& dataFields)
{
    NS_LOG_FUNCTION(this << packet);

    while (true)
    {
        const uint8_t extensionBit = header.PopExtensionBit();
        NS_LOG_LOGIC("E = " << (uint16_t)extensionBit);

        if (extensionBit == 0)
        {
            dataFields.push_back(packet);
            return;
        }

        const uint16_t lengthIndicator = header.PopLengthIndicator();
        NS_LOG_LOGIC("LI = " << lengthIndicator);

        // Every LI-delimited data field must leave at least one byte for the last
        // field, so a valid PDU always holds more data than the LI value.
        NS_ABORT_MSG_IF(lengthIndicator >= packet->GetSize(),
                        "Not enough data in the packet (" << packet->GetSize()
                                                          << "). Needed LI=" << lengthIndicator);

        dataFields.push_back(packet->CreateFragment(0, lengthIndicator));
        packet->RemoveAtStart(lengthIndicator);
    }
}

template void NrRlc::SplitDataFields(NrRlcHeader&, Ptr<Packet>, std::list<Ptr<Packet>>&);
template void NrRlc::SplitDataFields(NrRlcAmHeader&, Ptr<Packet>, std::list<Ptr<Packet>>&);

bool
NrRlc::ReassembleSdus(std::list<Ptr<Packet>>& dataFields,
                      Ptr<Packet>& keepS0,
                      bool firstFieldContinuesSdu,
                      bool lastFieldIsComplete,
                      bool heldSegmentIsUsable)
{
    NS_LOG_FUNCTION(this << dataFields.size() << firstFieldContinuesSdu << lastFieldIsComplete
                         << heldSegmentIsUsable);
    NS_ASSERT(!dataFields.empty());

    if (firstFieldContinuesSdu && keepS0 && heldSegmentIsUsable)
    {
        // The first data field continues the held first segment: concatenate them
        // and let the reassembled SDU take its place at the front of the list.
        keepS0->AddAtEnd(dataFields.front());
        dataFields.front() = keepS0;
        keepS0 = nullptr;
    }
    else
    {
        // Either there is no segment to continue, or the held one can no longer be
        // completed (loss, or framing info contradicting the reassembly state).
        // Discard it, and with it a first data field that continues an SDU we are
        // not reassembling, which is now orphaned.
        if (keepS0)
        {
            NS_LOG_LOGIC("Discarding the held partial SDU");
            keepS0 = nullptr;
        }
        if (firstFieldContinuesSdu)
        {
            NS_LOG_LOGIC("Discarding the orphaned SDU segment");
            dataFields.pop_front();
        }
    }

    // A data field that does not end on an SDU boundary is the first segment of the
    // next SDU: hold it back until its continuation arrives.
    if (!lastFieldIsComplete && !dataFields.empty())
    {
        keepS0 = dataFields.back();
        dataFields.pop_back();
    }

    for (const auto& sdu : dataFields)
    {
        m_rlcSapUser->ReceivePdcpPdu(sdu);
    }
    dataFields.clear();

    return keepS0 != nullptr;
}

////////////////////////////////////////

NS_OBJECT_ENSURE_REGISTERED(NrRlcSm);

NrRlcSm::NrRlcSm()
{
    NS_LOG_FUNCTION(this);
}

NrRlcSm::~NrRlcSm()
{
    NS_LOG_FUNCTION(this);
}

TypeId
NrRlcSm::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::NrRlcSm").SetParent<NrRlc>().SetGroupName("Nr").AddConstructor<NrRlcSm>();
    return tid;
}

void
NrRlcSm::DoInitialize()
{
    NS_LOG_FUNCTION(this);
    BufferStatusReport();
}

void
NrRlcSm::DoDispose()
{
    NS_LOG_FUNCTION(this);
    NrRlc::DoDispose();
}

void
NrRlcSm::DoTransmitPdcpPdu(Ptr<Packet> p)
{
    NS_LOG_FUNCTION(this << p);
}

void
NrRlcSm::DoReceivePdu(NrMacSapUser::ReceivePduParameters rxPduParams)
{
    NS_LOG_FUNCTION(this << rxPduParams.p);
    // RLC Performance evaluation
    NrRlcTag rlcTag;
    Time delay;
    bool ret = rxPduParams.p->FindFirstMatchingByteTag(rlcTag);
    NS_ASSERT_MSG(ret, "NrRlcTag is missing");
    delay = Simulator::Now() - rlcTag.GetSenderTimestamp();
    NS_LOG_LOGIC(" RNTI=" << m_rnti << " LCID=" << (uint32_t)m_lcid << " size="
                          << rxPduParams.p->GetSize() << " delay=" << delay.As(Time::NS));
    m_rxPdu(m_rnti, m_lcid, rxPduParams.p->GetSize(), delay.GetNanoSeconds());
}

void
NrRlcSm::DoNotifyTxOpportunity(NrMacSapUser::TxOpportunityParameters txOpParams)
{
    NS_LOG_FUNCTION(this << txOpParams.bytes);
    NrMacSapProvider::TransmitPduParameters params;
    NrRlcTag tag(Simulator::Now());

    params.pdu = Create<Packet>(txOpParams.bytes);
    NS_ABORT_MSG_UNLESS(txOpParams.bytes > 0, "Bytes must be > 0");
    /**
     * For RLC SM, the packets are not passed to the upper layers, therefore,
     * in the absence of an header we can safely byte tag the entire packet.
     */
    params.pdu->AddByteTag(tag, 1, params.pdu->GetSize());

    params.rnti = m_rnti;
    params.lcid = m_lcid;
    params.layer = txOpParams.layer;
    params.harqProcessId = txOpParams.harqId;
    params.componentCarrierId = txOpParams.componentCarrierId;

    // RLC Performance evaluation
    NS_LOG_LOGIC(" RNTI=" << m_rnti << " LCID=" << (uint32_t)m_lcid
                          << " size=" << txOpParams.bytes);
    m_txPdu(m_rnti, m_lcid, txOpParams.bytes);

    m_macSapProvider->TransmitPdu(params);
    BufferStatusReport();
}

void
NrRlcSm::DoNotifyHarqDeliveryFailure()
{
    NS_LOG_FUNCTION(this);
}

void
NrRlcSm::BufferStatusReport()
{
    NS_LOG_FUNCTION(this);
    NrMacSapProvider::BufferStatusReportParameters p;
    p.rnti = m_rnti;
    p.lcid = m_lcid;
    p.txQueueSize = 80000;
    p.txQueueHolDelay = 10;
    p.retxQueueSize = 0;
    p.retxQueueHolDelay = 0;
    p.statusPduSize = 0;
    p.expBsrTimer = false;
    m_macSapProvider->BufferStatusReport(p);
}

//////////////////////////////////////////

// NrRlcTm::~NrRlcTm ()
// {
// }

//////////////////////////////////////////

// NrRlcUm::~NrRlcUm ()
// {
// }

//////////////////////////////////////////

// NrRlcAm::~NrRlcAm ()
// {
// }

} // namespace ns3
