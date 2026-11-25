// Copyright (c) 2019 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only

#define NS_LOG_APPEND_CONTEXT                                                                      \
    do                                                                                             \
    {                                                                                              \
        std::clog << " [ CellId " << GetCellId() << ", bwpId " << GetBwpId() << ", rnti "          \
                  << m_rnti << "] ";                                                               \
    } while (false);

#include "nr-ue-mac.h"

#include "nr-control-messages.h"
#include "nr-mac-header-vs.h"
#include "nr-mac-phr-ce.h"
#include "nr-mac-short-bsr-ce.h"
#include "nr-phy-sap.h"
#include "nr-radio-bearer-tag.h"

#include "ns3/boolean.h"
#include "ns3/double.h"
#include "ns3/log.h"
#include "ns3/nstime.h"
#include "ns3/random-variable-stream.h"
#include "ns3/uinteger.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("NrUeMac");
NS_OBJECT_ENSURE_REGISTERED(NrUeMac);

namespace
{
/// Bytes of the SHORT_BSR MAC CE, including its subheader, appended to every UL TB
constexpr uint32_t SHORT_BSR_BYTES = 5;
/// Minimum TxOpportunity bytes that RLC AM can use
constexpr uint32_t RLC_AM_MIN_TXOP_BYTES = 7;

/**
 * @param pduBytes Size of a MAC SDU.
 * @return Bytes of the MAC subheader that carries it.
 */
uint32_t
MacSubheaderBytes(uint32_t pduBytes)
{
    NrMacHeaderVs header;
    // Any size beyond the 16-bit L field needs the same, largest, subheader.
    header.SetSize(std::min<uint32_t>(pduBytes, std::numeric_limits<uint16_t>::max()));
    return header.GetSerializedSize();
}

/**
 * @param wanted Bytes the RLC entity would like to send.
 * @param room Bytes available for the sub-PDU, subheader included.
 * @return Largest TxOpportunity, at most @p wanted, whose sub-PDU fits in @p room.
 */
uint32_t
TxOppFitting(uint32_t wanted, uint32_t room)
{
    uint32_t bytes = std::min(wanted, room);
    while (bytes > 0 && bytes + MacSubheaderBytes(bytes) > room)
    {
        bytes = room > MacSubheaderBytes(bytes) ? room - MacSubheaderBytes(bytes) : 0;
    }
    return bytes;
}

/**
 * @return Bytes of the smallest data sub-PDU: the minimum RLC AM TxOpportunity and its subheader.
 */
uint32_t
MinDataSubPduBytes()
{
    return RLC_AM_MIN_TXOP_BYTES + MacSubheaderBytes(RLC_AM_MIN_TXOP_BYTES);
}
} // namespace

uint8_t NrUeMac::g_raPreambleId = 0;

///////////////////////////////////////////////////////////
// SAP forwarders
///////////////////////////////////////////////////////////

class UeMemberNrUeCmacSapProvider : public NrUeCmacSapProvider
{
  public:
    UeMemberNrUeCmacSapProvider(NrUeMac* mac);

    // inherited from NrUeCmacSapProvider
    void ConfigureRach(RachConfig rc) override;
    void StartContentionBasedRandomAccessProcedure() override;
    void StartNonContentionBasedRandomAccessProcedure(uint16_t rnti,
                                                      uint8_t preambleId,
                                                      uint8_t prachMask) override;
    void AddLc(uint8_t lcId,
               NrUeCmacSapProvider::LogicalChannelConfig lcConfig,
               NrMacSapUser* msu) override;
    void RemoveLc(uint8_t lcId) override;
    void Reset() override;
    void SetRnti(uint16_t rnti) override;
    void NotifyConnectionSuccessful() override;
    void SetImsi(uint64_t imsi) override;
    void RegisterToGnb(uint16_t cellId) override;

  private:
    NrUeMac* m_mac;
};

UeMemberNrUeCmacSapProvider::UeMemberNrUeCmacSapProvider(NrUeMac* mac)
    : m_mac(mac)
{
}

void
UeMemberNrUeCmacSapProvider::ConfigureRach(RachConfig rc)
{
    m_mac->DoConfigureRach(rc);
}

void
UeMemberNrUeCmacSapProvider::StartContentionBasedRandomAccessProcedure()
{
    m_mac->DoStartContentionBasedRandomAccessProcedure();
}

void
UeMemberNrUeCmacSapProvider::StartNonContentionBasedRandomAccessProcedure(uint16_t rnti,
                                                                          uint8_t preambleId,
                                                                          uint8_t prachMask)
{
    m_mac->DoStartNonContentionBasedRandomAccessProcedure(rnti, preambleId, prachMask);
}

void
UeMemberNrUeCmacSapProvider::AddLc(uint8_t lcId, LogicalChannelConfig lcConfig, NrMacSapUser* msu)
{
    m_mac->AddLc(lcId, lcConfig, msu);
}

void
UeMemberNrUeCmacSapProvider::RemoveLc(uint8_t lcid)
{
    m_mac->DoRemoveLc(lcid);
}

void
UeMemberNrUeCmacSapProvider::Reset()
{
    m_mac->DoReset();
}

void
UeMemberNrUeCmacSapProvider::SetRnti(uint16_t rnti)
{
    m_mac->SetRnti(rnti);
}

void
UeMemberNrUeCmacSapProvider::NotifyConnectionSuccessful()
{
    m_mac->DoNotifyConnectionSuccessful();
}

void
UeMemberNrUeCmacSapProvider::SetImsi(uint64_t imsi)
{
    m_mac->DoSetImsi(imsi);
}

void
UeMemberNrUeCmacSapProvider::RegisterToGnb(uint16_t cellId)
{
    m_mac->m_phySapProvider->RegisterToGnb(cellId);
}

class UeMemberNrMacSapProvider : public NrMacSapProvider
{
  public:
    UeMemberNrMacSapProvider(NrUeMac* mac);

    // inherited from NrMacSapProvider
    void TransmitPdu(TransmitPduParameters params) override;
    void BufferStatusReport(BufferStatusReportParameters params) override;

  private:
    NrUeMac* m_mac;
};

UeMemberNrMacSapProvider::UeMemberNrMacSapProvider(NrUeMac* mac)
    : m_mac(mac)
{
}

void
UeMemberNrMacSapProvider::TransmitPdu(TransmitPduParameters params)
{
    m_mac->DoTransmitPdu(params);
}

void
UeMemberNrMacSapProvider::BufferStatusReport(BufferStatusReportParameters params)
{
    m_mac->DoTransmitBufferStatusReport(params);
}

class NrUePhySapUser;

class MacUeMemberPhySapUser : public NrUePhySapUser
{
  public:
    MacUeMemberPhySapUser(NrUeMac* mac);

    void ReceivePhyPdu(Ptr<Packet> p) override;

    void ReceiveControlMessage(Ptr<NrControlMessage> msg) override;

    void SlotIndication(SfnSf sfn) override;

    // virtual void NotifyHarqDeliveryFailure (uint8_t harqId);

    uint8_t GetNumHarqProcess() const override;

  private:
    NrUeMac* m_mac;
};

MacUeMemberPhySapUser::MacUeMemberPhySapUser(NrUeMac* mac)
    : m_mac(mac)
{
}

void
MacUeMemberPhySapUser::ReceivePhyPdu(Ptr<Packet> p)
{
    m_mac->DoReceivePhyPdu(p);
}

void
MacUeMemberPhySapUser::ReceiveControlMessage(Ptr<NrControlMessage> msg)
{
    m_mac->DoReceiveControlMessage(msg);
}

void
MacUeMemberPhySapUser::SlotIndication(SfnSf sfn)
{
    m_mac->DoSlotIndication(sfn);
}

uint8_t
MacUeMemberPhySapUser::GetNumHarqProcess() const
{
    return m_mac->GetNumHarqProcess();
}

//-----------------------------------------------------------------------

TypeId
NrUeMac::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::NrUeMac")
            .SetParent<Object>()
            .AddConstructor<NrUeMac>()
            .AddAttribute(
                "RaResponseWindowStartOffset",
                "NTN (TS 38.321 5.1.4, Rel-17): extra delay between the end of the RA preamble "
                "and the start of ra-ResponseWindow, i.e. the UE-gNB RTT T_TA + k_mac. The window "
                "end moves by the same amount. A RAR that arrives before the window opens is "
                "ignored, as the UE is not monitoring the RA-RNTI then. Zero (default) is the "
                "terrestrial behavior.",
                TimeValue(Time(0)),
                MakeTimeAccessor(&NrUeMac::m_raWindowStartOffset),
                MakeTimeChecker(Time(0)))
            .AddAttribute(
                "PhrPeriodicTimer",
                "TS 38.321 5.4.6 / TS 38.331 PHR-Config phr-PeriodicTimer. A positive value "
                "configures power headroom reporting: a Single Entry PHR MAC CE (TS 38.321 "
                "6.1.3.8) carrying the type 1 power headroom of the PUSCH it rides on (TS 38.213 "
                "7.7.1, needs NrUePhy::EnableUplinkPowerControl) is sent on a new UL transmission "
                "upon configuration, on expiry of this timer, and when the path loss changed by "
                "more than PhrTxPowerFactorChange after PhrProhibitTimer. Zero (default) leaves "
                "PHR unconfigured, as in nr before ntn-patch 16.",
                TimeValue(Time(0)),
                MakeTimeAccessor(&NrUeMac::m_phrPeriodicTimer),
                MakeTimeChecker(Time(0)))
            .AddAttribute("PhrProhibitTimer",
                          "TS 38.331 PHR-Config phr-ProhibitTimer",
                          TimeValue(MilliSeconds(10)),
                          MakeTimeAccessor(&NrUeMac::m_phrProhibitTimer),
                          MakeTimeChecker(Time(0)))
            .AddAttribute("PhrTxPowerFactorChange",
                          "TS 38.331 PHR-Config phr-Tx-PowerFactorChange (dB): a path-loss "
                          "change above this since the last PHR triggers a new one once "
                          "phr-ProhibitTimer has expired",
                          DoubleValue(1.0),
                          MakeDoubleAccessor(&NrUeMac::m_phrTxPowerFactorChange),
                          MakeDoubleChecker<double>(0.0))
            .AddAttribute(
                "BsrTriggering",
                "TS 38.321 5.4.5 (ntn-patch 17): send the SHORT_BSR MAC CE only when a BSR is "
                "triggered: a Regular BSR when UL data becomes available for a logical channel of "
                "higher priority than any with data, or when no logical channel had data, and on "
                "retxBSR-Timer expiry; a Periodic BSR on periodicBSR-Timer expiry; a Padding BSR "
                "when the padding of a new transmission can hold the CE. A triggered BSR is "
                "cancelled when the grant holds all pending data but not the CE as well. False "
                "(default) keeps nr's behavior, a BSR in every UL transport block.",
                BooleanValue(false),
                MakeBooleanAccessor(&NrUeMac::m_bsrTriggering),
                MakeBooleanChecker())
            .AddAttribute("PeriodicBsrTimer",
                          "TS 38.331 BSR-Config periodicBSR-Timer, used with BsrTriggering. "
                          "Zero (default) is infinity: no Periodic BSR.",
                          TimeValue(Time(0)),
                          MakeTimeAccessor(&NrUeMac::m_periodicBsrTimer),
                          MakeTimeChecker(Time(0)))
            .AddTraceSource("BsrSent",
                            "A SHORT_BSR MAC CE was multiplexed into a PUSCH: IMSI, RNTI, "
                            "BsrType (0 regular, 1 periodic, 2 padding, 3 unconditional), "
                            "reported bytes",
                            MakeTraceSourceAccessor(&NrUeMac::m_bsrTrace),
                            "ns3::NrUeMac::BsrTracedCallback")
            .AddTraceSource("PowerHeadroomReport",
                            "A Single Entry PHR MAC CE was multiplexed into a PUSCH (TS 38.321 "
                            "5.4.6): IMSI, RNTI, PH (dB), PH index, M_RB, path loss (dB)",
                            MakeTraceSourceAccessor(&NrUeMac::m_phrTrace),
                            "ns3::NrUeMac::PhrTracedCallback")
            .AddTraceSource("RarOutsideWindow",
                            "A RAR addressed to this UE arrived while ra-ResponseWindow was not "
                            "running (before the NTN start offset elapsed) and was ignored. "
                            "Exports the IMSI.",
                            MakeTraceSourceAccessor(&NrUeMac::m_rarOutsideWindowTrace),
                            "ns3::NrUeMac::ImsiTracedCallback")
            .AddAttribute(
                "NumHarqProcess",
                "Number of concurrent stop-and-wait Hybrid ARQ processes per user",
                UintegerValue(16),
                MakeUintegerAccessor(&NrUeMac::SetNumHarqProcess, &NrUeMac::GetNumHarqProcess),
                MakeUintegerChecker<uint8_t>())
            .AddAttribute("SrProhibitTimer",
                          "Minimum time between two transmissions of the scheduling request "
                          "(sr-ProhibitTimer of 3GPP TS 38.331). While the timer is running the "
                          "pending scheduling request is not transmitted, but it stays pending "
                          "and is sent on the first occasion after the timer expires. Set to zero "
                          "to transmit the scheduling request on every occasion.",
                          TimeValue(MilliSeconds(10)),
                          MakeTimeAccessor(&NrUeMac::m_srProhibitTimer),
                          MakeTimeChecker())
            .AddAttribute("SrTransMax",
                          "Maximum number of transmissions of a pending scheduling request "
                          "(sr-TransMax of 3GPP TS 38.331). Once that many go unanswered the "
                          "request is cancelled and a random access procedure is started to get "
                          "uplink resources instead.",
                          UintegerValue(64),
                          MakeUintegerAccessor(&NrUeMac::m_srTransMax),
                          MakeUintegerChecker<uint32_t>(1))
            .AddTraceSource("UeMacRxedCtrlMsgsTrace",
                            "Ue MAC Control Messages Traces.",
                            MakeTraceSourceAccessor(&NrUeMac::m_macRxedCtrlMsgsTrace),
                            "ns3::NrMacRxTrace::RxedUeMacCtrlMsgsTracedCallback")
            .AddTraceSource("UeMacTxedCtrlMsgsTrace",
                            "Ue MAC Control Messages Traces.",
                            MakeTraceSourceAccessor(&NrUeMac::m_macTxedCtrlMsgsTrace),
                            "ns3::NrMacRxTrace::TxedUeMacCtrlMsgsTracedCallback")
            .AddTraceSource("RaResponseTimeout",
                            "Trace fired upon RA response timeout",
                            MakeTraceSourceAccessor(&NrUeMac::m_raResponseTimeoutTrace),
                            "ns3::NrUeMac::RaResponseTimeoutTracedCallback")
            .AddTraceSource("UeMacStateMachineTrace",
                            "UE MAC state machine trace",
                            MakeTraceSourceAccessor(&NrUeMac::m_macUeStateMachine),
                            "ns3::NrUeMac::UeMacStateMachineTracedCallback");
    return tid;
}

NrUeMac::NrUeMac()
    : Object()
{
    NS_LOG_FUNCTION(this);
    m_cmacSapProvider = new UeMemberNrUeCmacSapProvider(this);
    m_macSapProvider = new UeMemberNrMacSapProvider(this);
    m_phySapUser = new MacUeMemberPhySapUser(this);
    m_raPreambleUniformVariable = CreateObject<UniformRandomVariable>();
}

NrUeMac::~NrUeMac()
{
}

void
NrUeMac::DoDispose()
{
    m_miUlHarqProcessesPacket.clear();
    m_miUlHarqProcessesPacketTimer.clear();
    m_ulBsrReceived.clear();
    m_lcInfoMap.clear();
    m_raPreambleUniformVariable = nullptr;
    delete m_macSapProvider;
    delete m_cmacSapProvider;
    delete m_phySapUser;
}

void
NrUeMac::SetRnti(uint16_t rnti)
{
    NS_LOG_FUNCTION(this);
    m_rnti = rnti;
}

void
NrUeMac::DoNotifyConnectionSuccessful()
{
    NS_LOG_FUNCTION(this);
    m_phySapProvider->NotifyConnectionSuccessful();
}

void
NrUeMac::DoSetImsi(uint64_t imsi)
{
    NS_LOG_FUNCTION(this);
    m_imsi = imsi;
}

uint16_t
NrUeMac::GetBwpId() const
{
    if (m_phySapProvider)
    {
        return m_phySapProvider->GetBwpId();
    }
    else
    {
        return UINT16_MAX;
    }
}

uint16_t
NrUeMac::GetCellId() const
{
    if (m_phySapProvider)
    {
        return m_phySapProvider->GetCellId();
    }
    else
    {
        return UINT16_MAX;
    }
}

uint16_t
NrUeMac::GetRnti() const
{
    return m_rnti;
}

uint64_t
NrUeMac::GetImsi() const
{
    return m_imsi;
}

void
NrUeMac::SetCurrentSlot(const SfnSf& sfn)
{
    m_currentSlot = sfn;
}

uint32_t
NrUeMac::GetTotalBufSize() const
{
    uint32_t ret = 0;
    for (const auto& it : m_ulBsrReceived)
    {
        ret += (it.second.txQueueSize + it.second.retxQueueSize + it.second.statusPduSize);
    }
    return ret;
}

/**
 * @brief Sets the number of HARQ processes
 * @param numHarqProcesses the maximum number of harq processes
 */
uint32_t
NrUeMac::GetPendingUlBytesWithHeaders() const
{
    uint32_t ret = 0;
    for (const auto& it : m_ulBsrReceived)
    {
        ret += it.second.statusPduSize;
        ret += it.second.txQueueSize > 0 ? it.second.txQueueSize + 3 : 0;
        ret += it.second.retxQueueSize > 0 ? it.second.retxQueueSize + 3 : 0;
    }
    return ret;
}

void
NrUeMac::SetNumHarqProcess(uint8_t numHarqProcess)
{
    m_numHarqProcess = numHarqProcess;

    m_miUlHarqProcessesPacket.resize(GetNumHarqProcess());
    for (auto& i : m_miUlHarqProcessesPacket)
    {
        if (i.m_pktBurst == nullptr)
        {
            Ptr<PacketBurst> pb = CreateObject<PacketBurst>();
            i.m_pktBurst = pb;
        }
    }
    m_miUlHarqProcessesPacketTimer.resize(GetNumHarqProcess(), 0);
}

/**
 * @return number of HARQ processes
 */
uint8_t
NrUeMac::GetNumHarqProcess() const
{
    return m_numHarqProcess;
}

// forwarded from MAC SAP
void
NrUeMac::DoTransmitPdu(NrMacSapProvider::TransmitPduParameters params)
{
    NS_LOG_FUNCTION(this);
    if (m_ulDci == nullptr)
    {
        NS_LOG_WARN("Null DCI received for transmission.");
        return;
    }
    NS_ASSERT(m_ulDci);
    NS_ASSERT(m_ulDci->m_harqProcess == params.harqProcessId);

    m_miUlHarqProcessesPacket.at(params.harqProcessId).m_lcidList.push_back(params.lcid);

    NrMacHeaderVs header;
    header.SetLcId(params.lcid);
    header.SetSize(params.pdu->GetSize());

    params.pdu->AddHeader(header);

    NrRadioBearerTag bearerTag(params.rnti, params.lcid, 0);
    params.pdu->AddPacketTag(bearerTag);

    if (!m_miUlHarqProcessesPacket.at(params.harqProcessId).m_pktBurst)
    {
        m_miUlHarqProcessesPacket.at(params.harqProcessId).m_pktBurst = CreateObject<PacketBurst>();
    }
    m_miUlHarqProcessesPacket.at(params.harqProcessId).m_pktBurst->AddPacket(params.pdu);
    m_miUlHarqProcessesPacketTimer.at(params.harqProcessId) = GetNumHarqProcess();

    m_ulDciTotalUsed += params.pdu->GetSize();

    NS_ASSERT_MSG(m_ulDciTotalUsed <= m_ulDci->m_tbSize,
                  "We used more data than the DCI allowed us.");

    m_phySapProvider->SendMacPdu(params.pdu, m_ulDciSfnsf, m_ulDci->m_symStart, m_ulDci->m_rnti);
}

void
NrUeMac::DoTransmitBufferStatusReport(NrMacSapProvider::BufferStatusReportParameters params)
{
    NS_LOG_FUNCTION(this << static_cast<uint32_t>(params.lcid));

    // The RLC entity of a logical channel that the MAC has dropped (handover, or the
    // DoReset() that precedes a re-establishment) keeps the SAP it was given and can still
    // report a buffer status. Such a report can never be served, because no NrMacSapUser is
    // left to receive the transmission opportunity, and recording it would leave an LCID in
    // m_ulBsrReceived that SendNewStatusData(), SendRetxData() and SendTxData() go on to look
    // up in m_lcInfoMap.
    if (m_lcInfoMap.find(params.lcid) == m_lcInfoMap.end())
    {
        NS_LOG_WARN("Discarding the BSR of LCID " << static_cast<uint32_t>(params.lcid)
                                                  << ", which is not configured in this MAC");
        return;
    }

    auto it = m_ulBsrReceived.find(params.lcid);

    NS_LOG_INFO("Received BSR for LC Id" << static_cast<uint32_t>(params.lcid));

    // New UL data for this channel: RLC's buffer grew since RLC last reported it. Since 5G-LENA
    // v5.2 RLC also refreshes its status from within every transmit opportunity, which is not
    // new data; v5.1 reported only when an SDU arrived.
    auto rlcTotal = [](const NrMacSapProvider::BufferStatusReportParameters& b) {
        return b.txQueueSize + b.retxQueueSize + b.statusPduSize;
    };
    const uint32_t rlcBefore = m_rlcReportedBytes[params.lcid];
    const bool newData = rlcTotal(params) > rlcBefore;

    // ntn-patch 17: TS 38.321 5.4.5 Regular BSR trigger. UL data becomes
    // available for a logical channel in an LCG (LCID 0 is in none) and either
    // no logical channel in an LCG had data, or this one has a higher priority
    // (a lower priority value) than every logical channel with data.
    if (m_bsrTriggering && params.lcid != 0)
    {
        auto total = [](const NrMacSapProvider::BufferStatusReportParameters& b) {
            return b.txQueueSize + b.retxQueueSize + b.statusPduSize;
        };
        // Compare with what RLC itself last reported for each channel, not with the cached
        // m_ulBsrReceived: since 5G-LENA v5.2 the MAC decrements that cache as it hands
        // transmit opportunities to RLC, and RLC then refreshes its buffer status from within
        // the opportunity. Against the decremented cache every refresh looked like new data
        // arriving in an empty buffer (measured: 1076 Regular BSRs for 62 UL packets, so
        // periodicBSR-Timer never expired).
        const uint32_t before = rlcBefore;
        if (newData)
        {
            bool anyData = false;
            uint8_t highest = UINT8_MAX;
            for (const auto& [lcid, bytes] : m_rlcReportedBytes)
            {
                if (lcid == 0 || lcid == params.lcid || bytes == 0)
                {
                    continue;
                }
                anyData = true;
                auto lc = m_lcInfoMap.find(lcid);
                if (lc != m_lcInfoMap.end())
                {
                    highest = std::min(highest, lc->second.lcConfig.priority);
                }
            }
            auto own = m_lcInfoMap.find(params.lcid);
            const uint8_t prio = (own != m_lcInfoMap.end()) ? own->second.lcConfig.priority
                                                            : UINT8_MAX;
            anyData = anyData || before > 0;
            if (!anyData || prio < highest)
            {
                m_regularBsrTriggered = true;
            }
        }
    }

    m_rlcReportedBytes[params.lcid] = rlcTotal(params);

    if (it != m_ulBsrReceived.end())
    {
        // update entry
        (*it).second = params;
    }
    else
    {
        it = m_ulBsrReceived.insert(std::make_pair(params.lcid, params)).first;
    }

    // Re-arm the SR whenever new buffer status arrives while backlogged, as the UE keeps
    // requesting resources while a BSR is pending and no UL-SCH resources are available
    // (3GPP TS 38.321, clauses 5.4.4 and 5.4.5). The previous condition keyed the re-arm to
    // the last received UL DCI (HARQ process 0, or rv 3), which deadlocks the uplink when the
    // bootstrap BSR TBs are lost: the gNB stops granting without a BSR, the stale m_ulDci
    // never satisfies the condition, and no SR is ever sent again.
    if (m_srState == INACTIVE || (m_srState == ACTIVE && GetTotalBufSize() > 0 && newData))
    {
        if (m_srState == INACTIVE)
        {
            // A scheduling request is triggered while no other one is pending, so the count of
            // its transmissions starts over (3GPP TS 38.321, clause 5.4.4).
            m_srCounter = 0;
            NS_LOG_INFO("m_srState = INACTIVE -> TO_SEND, bufSize " << GetTotalBufSize());
            m_macUeStateMachine(m_imsi,
                                m_currentSlot,
                                GetCellId(),
                                m_rnti,
                                GetBwpId(),
                                m_srState,
                                m_ulBsrReceived,
                                1,
                                "DoTransmitBufferStatusReport");
        }
        else
        {
            NS_LOG_INFO("m_srState = ACTIVE (BSR Timer expired) -> TO_SEND, bufSize "
                        << GetTotalBufSize());
            m_macUeStateMachine(m_imsi,
                                m_currentSlot,
                                GetCellId(),
                                m_rnti,
                                GetBwpId(),
                                m_srState,
                                m_ulBsrReceived,
                                0,
                                "DoTransmitBufferStatusReport");
        }
        m_srState = TO_SEND;
        m_regularBsrTriggered = true; // ntn-patch 17: a Regular BSR
    }
}

void
NrUeMac::SendBufferStatusReport(const SfnSf& dataSfn, uint8_t symStart, uint8_t type)
{
    NS_LOG_FUNCTION(this);

    if (m_rnti == 0)
    {
        NS_LOG_INFO("MAC not initialized, BSR deferred");
        return;
    }

    // Every UL TB carries a SHORT_BSR, all zero when nothing is buffered: it is then the
    // padding BSR of 3GPP TS 38.321, clause 5.4.5, and no TB is left empty.
    MacCeElement bsr = MacCeElement();
    bsr.m_rnti = m_rnti;
    bsr.m_macCeType = MacCeElement::BSR;

    // BSR is reported for each LCG
    std::unordered_map<uint8_t, NrMacSapProvider::BufferStatusReportParameters>::iterator it;
    std::vector<uint32_t> queue(4, 0); // one value per each of the 4 LCGs, initialized to 0
    for (it = m_ulBsrReceived.begin(); it != m_ulBsrReceived.end(); it++)
    {
        uint8_t lcid = it->first;
        std::unordered_map<uint8_t, LcInfo>::iterator lcInfoMapIt;
        lcInfoMapIt = m_lcInfoMap.find(lcid);
        NS_ASSERT(lcInfoMapIt != m_lcInfoMap.end());
        NS_ASSERT_MSG((lcid != 0) ||
                          (((*it).second.txQueueSize == 0) && ((*it).second.retxQueueSize == 0) &&
                           ((*it).second.statusPduSize == 0)),
                      "BSR should not be used for LCID 0");
        uint8_t lcg = lcInfoMapIt->second.lcConfig.logicalChannelGroup;
        queue.at(lcg) +=
            ((*it).second.txQueueSize + (*it).second.retxQueueSize + (*it).second.statusPduSize);

        // Each non-empty queue goes out in a sub-PDU whose subheader depends on its size.
        using QueueBytes = std::pair<const char*, uint32_t>;
        for (const auto& [kind, bytes] : {QueueBytes{"TX", (*it).second.txQueueSize},
                                          QueueBytes{"RETX", (*it).second.retxQueueSize},
                                          QueueBytes{"STATUS", (*it).second.statusPduSize}})
        {
            if (bytes == 0)
            {
                continue;
            }
            NS_LOG_DEBUG("Adding " << MacSubheaderBytes(bytes) << " bytes for " << kind
                                   << " subheader for LCID:" << +lcid);
            queue.at(lcg) += MacSubheaderBytes(bytes);
        }
    }

    // The next TB carries one SHORT_BSR, whatever the number of LCs and LCGs with data.
    auto firstActiveLcg =
        std::find_if(queue.begin(), queue.end(), [](uint32_t bytes) { return bytes > 0; });
    if (firstActiveLcg != queue.end())
    {
        NS_LOG_DEBUG("Adding " << SHORT_BSR_BYTES << " bytes for SHORT_BSR to LCG "
                               << std::distance(queue.begin(), firstActiveLcg));
        *firstActiveLcg += SHORT_BSR_BYTES;
    }

    NS_LOG_INFO("Sending BSR with this info for the LCG: "
                << queue.at(0) << " " << queue.at(1) << " " << queue.at(2) << " " << queue.at(3));
    // FF API says that all 4 LCGs are always present
    bsr.m_macCeValue.m_bufferStatus.push_back(NrMacShortBsrCe::FromBytesToLevel(queue.at(0)));
    bsr.m_macCeValue.m_bufferStatus.push_back(NrMacShortBsrCe::FromBytesToLevel(queue.at(1)));
    bsr.m_macCeValue.m_bufferStatus.push_back(NrMacShortBsrCe::FromBytesToLevel(queue.at(2)));
    bsr.m_macCeValue.m_bufferStatus.push_back(NrMacShortBsrCe::FromBytesToLevel(queue.at(3)));

    // create the message. It is used only for tracing, but we don't send it...
    Ptr<NrBsrMessage> msg = Create<NrBsrMessage>();
    msg->SetSourceBwpArfcn(m_phySapProvider->GetArfcn());
    msg->SetBsr(bsr);

    m_macTxedCtrlMsgsTrace(m_imsi, m_currentSlot, GetCellId(), bsr.m_rnti, GetBwpId(), msg);

    // Here we send the real SHORT_BSR, as a subpdu.
    Ptr<Packet> p = Create<Packet>();

    // Please note that the levels are defined from the standard. In this case,
    // we have 5 bit available, so use such standard levels. In the future,
    // when LONG BSR will be implemented, this have to change.
    NrMacShortBsrCe header;
    header.m_bufferSizeLevel_0 = NrMacShortBsrCe::FromBytesToLevel(queue.at(0));
    header.m_bufferSizeLevel_1 = NrMacShortBsrCe::FromBytesToLevel(queue.at(1));
    header.m_bufferSizeLevel_2 = NrMacShortBsrCe::FromBytesToLevel(queue.at(2));
    header.m_bufferSizeLevel_3 = NrMacShortBsrCe::FromBytesToLevel(queue.at(3));

    p->AddHeader(header);

    NrRadioBearerTag bearerTag(m_rnti, NrMacHeaderFsUl::SHORT_BSR, 0);
    p->AddPacketTag(bearerTag);

    m_ulDciTotalUsed += p->GetSize();
    NS_ASSERT_MSG(m_ulDciTotalUsed <= m_ulDci->m_tbSize,
                  "We used more data than the DCI allowed us.");

    if (type != BSR_UNCONDITIONAL)
    {
        // ntn-patch 17: the triggered BSR is part of this MAC PDU, so HARQ
        // retransmits it with the PDU (nr regenerates its unconditional BSR on
        // every retransmission instead).
        auto& harq = m_miUlHarqProcessesPacket.at(m_ulDci->m_harqProcess);
        if (!harq.m_pktBurst)
        {
            harq.m_pktBurst = CreateObject<PacketBurst>();
        }
        harq.m_pktBurst->AddPacket(p->Copy());
        m_miUlHarqProcessesPacketTimer.at(m_ulDci->m_harqProcess) = GetNumHarqProcess();
        // TS 38.321 5.4.5: a MAC PDU with a BSR cancels all triggered BSRs and
        // (re)starts periodicBSR-Timer (the CE carries every LCG, never a
        // truncated BSR); retxBSR-Timer restarts with the UL grant.
        m_regularBsrTriggered = false;
        m_periodicBsrTriggered = false;
        m_periodicBsrExpiry = m_periodicBsrTimer.IsStrictlyPositive()
                                  ? Simulator::Now() + m_periodicBsrTimer
                                  : Time(0);
    }
    if (type < m_bsrSent.size())
    {
        ++m_bsrSent[type];
    }
    m_bsrTrace(m_imsi, m_rnti, type, queue.at(0) + queue.at(1) + queue.at(2) + queue.at(3));


    m_phySapProvider->SendMacPdu(p, dataSfn, symStart, m_ulDci->m_rnti);

    // The transmitted MAC PDU carries the buffer status, so the pending scheduling requests are
    // cancelled and their sr-ProhibitTimer is stopped (3GPP TS 38.321, clause 5.4.4). The gNB now
    // knows what the UE has, so the transmissions spent asking for this grant no longer count
    // against sr-TransMax: only requests that go unanswered do.
    m_srCounter = 0;
    m_lastSrSent = Time::Min();

    m_macUeStateMachine(m_imsi,
                        m_currentSlot,
                        GetCellId(),
                        m_rnti,
                        GetBwpId(),
                        m_srState,
                        m_ulBsrReceived,
                        m_ulDci->m_ndi,
                        "SendBufferStatusReport");
}

uint32_t
NrUeMac::PreparePhr()
{
    NS_LOG_FUNCTION(this);
    m_phrPending = NrPhySapProvider::PuschPowerHeadroom{};
    if (!m_phrPeriodicTimer.IsStrictlyPositive() || m_rnti == 0)
    {
        return 0; // PHR not configured
    }
    const Time now = Simulator::Now();
    const auto ph = m_phySapProvider->GetPuschPowerHeadroom(m_ulDci->m_rbgBitmask);
    if (!ph.valid)
    {
        return 0; // no PUSCH power control, so no power headroom to report
    }
    // TS 38.321 5.4.6 triggers.
    if (m_phrPeriodicExpiry.IsStrictlyPositive() && now >= m_phrPeriodicExpiry)
    {
        m_phrTriggered = true; // phr-PeriodicTimer expired
    }
    if (now >= m_phrProhibitExpiry && std::isfinite(m_phrLastPathloss) &&
        std::abs(ph.pathlossDb - m_phrLastPathloss) > m_phrTxPowerFactorChange)
    {
        m_phrTriggered = true; // phr-ProhibitTimer expired and PL changed
    }
    if (!m_phrTriggered)
    {
        return 0;
    }
    // The CE (2 octets) and its subheader (1 octet) must fit after the
    // status PDUs and the SHORT_BSR (5 bytes), which come first.
    uint32_t statusBytes = 0;
    for (const auto& b : m_ulBsrReceived)
    {
        statusBytes += b.second.statusPduSize;
    }
    NrMacSingleEntryPhrCe ce;
    const uint32_t ceBytes = ce.GetSerializedSize();
    if (m_ulDciTotalUsed + statusBytes + m_bsrReservedBytes + ceBytes > m_ulDci->m_tbSize)
    {
        return 0; // stays triggered for the next grant
    }
    m_phrPending = ph;
    return ceBytes;
}

void
NrUeMac::SendPhr(const SfnSf& dataSfn, uint8_t symStart)
{
    NS_LOG_FUNCTION(this);
    NS_ASSERT(m_phrPending.valid);
    NrMacSingleEntryPhrCe ce;
    ce.m_phIndex = NrMacSingleEntryPhrCe::PhDbToIndex(m_phrPending.phDb);
    ce.m_pcmaxIndex = NrMacSingleEntryPhrCe::PcmaxDbmToIndex(m_phrPending.pcmaxDbm);

    Ptr<Packet> p = Create<Packet>();
    p->AddHeader(ce);
    NrRadioBearerTag bearerTag(m_rnti, NrMacHeaderFsUl::SINGLE_ENTRY_PHR, 0);
    p->AddPacketTag(bearerTag);

    m_ulDciTotalUsed += p->GetSize();
    NS_ASSERT_MSG(m_ulDciTotalUsed <= m_ulDci->m_tbSize,
                  "We used more data than the DCI allowed us.");
    // The CE is part of the MAC PDU, so a HARQ retransmission of this TB
    // carries it again (unlike nr's SHORT_BSR, which is regenerated): at the
    // cell edge the first transmission is often lost and the PHR would only
    // ever arrive on a TB that decodes the first time.
    auto& harq = m_miUlHarqProcessesPacket.at(m_ulDci->m_harqProcess);
    if (!harq.m_pktBurst)
    {
        harq.m_pktBurst = CreateObject<PacketBurst>();
    }
    harq.m_pktBurst->AddPacket(p->Copy());
    m_miUlHarqProcessesPacketTimer.at(m_ulDci->m_harqProcess) = GetNumHarqProcess();
    m_phySapProvider->SendMacPdu(p, dataSfn, symStart, m_ulDci->m_rnti);

    // TS 38.321 5.4.6: start or restart both timers, cancel the trigger.
    const Time now = Simulator::Now();
    m_phrPeriodicExpiry = now + m_phrPeriodicTimer;
    m_phrProhibitExpiry = now + m_phrProhibitTimer;
    m_phrLastPathloss = m_phrPending.pathlossDb;
    m_phrTriggered = false;
    ++m_phrSent;
    m_phrTrace(m_imsi,
               m_rnti,
               m_phrPending.phDb,
               ce.m_phIndex,
               m_phrPending.numRb,
               m_phrPending.pathlossDb);
    NS_LOG_INFO("PHR sent: PH " << m_phrPending.phDb << " dB (index " << +ce.m_phIndex
                                << ") on " << m_phrPending.numRb << " RBs, PL "
                                << m_phrPending.pathlossDb << " dB");
    m_phrPending = NrPhySapProvider::PuschPowerHeadroom{};
}

void
NrUeMac::SetUeCmacSapUser(NrUeCmacSapUser* s)
{
    m_cmacSapUser = s;
}

NrUeCmacSapProvider*
NrUeMac::GetUeCmacSapProvider()
{
    return m_cmacSapProvider;
}

void
NrUeMac::RefreshHarqProcessesPacketBuffer()
{
    NS_LOG_FUNCTION(this);

    for (std::size_t i = 0; i < m_miUlHarqProcessesPacketTimer.size(); i++)
    {
        if (m_miUlHarqProcessesPacketTimer.at(i) == 0 && m_miUlHarqProcessesPacket.at(i).m_pktBurst)
        {
            if (m_miUlHarqProcessesPacket.at(i).m_pktBurst->GetSize() > 0)
            {
                // timer expired: drop packets in buffer for this process
                NS_LOG_INFO("HARQ Proc Id " << i << " packets buffer expired");
                Ptr<PacketBurst> emptyPb = CreateObject<PacketBurst>();
                m_miUlHarqProcessesPacket.at(i).m_pktBurst = emptyPb;
                m_miUlHarqProcessesPacket.at(i).m_lcidList.clear();
            }
        }
        else
        {
            // m_miUlHarqProcessesPacketTimer.at (i)--;  // ignore HARQ timeout
        }
    }
}

void
NrUeMac::DoSlotIndication(const SfnSf& sfn)
{
    NS_LOG_FUNCTION(this);
    m_currentSlot = sfn;
    NS_LOG_INFO("Slot " << m_currentSlot);

    RefreshHarqProcessesPacketBuffer();

    // A pending scheduling request is transmitted on the first occasion on which
    // sr-ProhibitTimer is not running (3GPP TS 38.321, clause 5.4.4). While the timer runs the
    // request stays pending, so a backlogged UE asks again as soon as it is allowed to, without
    // repeating the request on every slot.
    if (m_srState == TO_SEND &&
        (m_lastSrSent == Time::Min() || Simulator::Now() - m_lastSrSent >= m_srProhibitTimer))
    {
        if (m_srCounter < m_srTransMax)
        {
            NS_LOG_INFO("Sending SR to PHY in slot " << sfn << ", transmission " << m_srCounter + 1
                                                     << " of " << m_srTransMax);
            SendSR();
            m_srCounter++;
            m_lastSrSent = Simulator::Now();
            m_srState = ACTIVE;
            NS_LOG_INFO("m_srState = TO_SEND -> ACTIVE");
            m_macUeStateMachine(m_imsi,
                                m_currentSlot,
                                GetCellId(),
                                m_rnti,
                                GetBwpId(),
                                m_srState,
                                m_ulBsrReceived,
                                1,
                                "DoSlotIndication");
        }
        else
        {
            // sr-TransMax transmissions went unanswered, so the scheduling request procedure is
            // abandoned: the pending requests are cancelled and a random access procedure is
            // started to get uplink resources instead (3GPP TS 38.321, clause 5.4.4).
            NS_LOG_WARN("Reached sr-TransMax ("
                        << m_srTransMax
                        << ") scheduling requests without a grant, falling back to random access");
            m_srState = INACTIVE;
            m_srCounter = 0;
            m_lastSrSent = Time::Min();
            m_macUeStateMachine(m_imsi,
                                m_currentSlot,
                                GetCellId(),
                                m_rnti,
                                GetBwpId(),
                                m_srState,
                                m_ulBsrReceived,
                                0,
                                "DoSlotIndication");

            if (m_rachConfigured)
            {
                RandomlySelectAndSendRaPreamble();
            }
            else
            {
                NS_LOG_WARN("RACH is not configured, cannot fall back to random access");
            }
        }
    }

    // Feedback missing
}

void
NrUeMac::SendSR() const
{
    NS_LOG_FUNCTION(this);

    if (m_rnti == 0)
    {
        NS_LOG_INFO("MAC not initialized, SR deferred");
        return;
    }

    // create the SR to send to the gNB
    Ptr<NrSRMessage> msg = Create<NrSRMessage>();
    msg->SetSourceBwpArfcn(m_phySapProvider->GetArfcn());
    msg->SetRNTI(m_rnti);

    m_macTxedCtrlMsgsTrace(m_imsi, m_currentSlot, GetCellId(), m_rnti, GetBwpId(), msg);
    m_phySapProvider->SendControlMessage(msg);
}

void
NrUeMac::DoReceivePhyPdu(Ptr<Packet> p)
{
    NS_LOG_FUNCTION(this);

    NrRadioBearerTag tag;
    p->RemovePacketTag(tag);

    if (tag.GetRnti() != m_rnti) // Packet is for another user
    {
        NS_LOG_WARN("Packet is for another user. RNTI does not correspond.");
        return;
    }

    NrMacHeaderVs header;
    p->RemoveHeader(header);

    NrMacSapUser::ReceivePduParameters rxParams;
    rxParams.p = p;
    rxParams.rnti = m_rnti;
    rxParams.lcid = header.GetLcId();

    auto it = m_lcInfoMap.find(header.GetLcId());
    // Ignore non-existing lcids
    if (it == m_lcInfoMap.end())
    {
        NS_LOG_WARN("LC info not found for this logical channel id:" << +header.GetLcId());
        return;
    }

    // p can be empty. Well, right now no, but when someone will add CE in downlink,
    // then p can be empty.
    if (rxParams.p->GetSize() > 0)
    {
        NS_LOG_INFO("Call MAC SAP user to receive PDU" << rxParams.p->GetSize());
        it->second.macSapUser->ReceivePdu(rxParams);
    }
    else
    {
        NS_LOG_WARN("Empty packet.");
    }
}

void
NrUeMac::RecvRaResponse(NrBuildRarListElement_s raResponse)
{
    NS_LOG_FUNCTION(this);
    m_waitingForRaResponse = false;
    m_noRaResponseReceivedEvent.Cancel();
    NS_LOG_INFO(" IMSI " << m_imsi << " RNTI " << m_rnti << " received RAR for RA preamble ID "
                         << +m_raPreambleId
                         << ", setting T-C-RNTI = " << raResponse.ulMsg3Dci->m_rnti
                         << " at: " << Simulator::Now().As(Time::MS));
    // Save data for MSG3
    m_rnti = raResponse.ulMsg3Dci->m_rnti;
    m_ulDciSfnsf = m_currentSlot;
    m_ulDciSfnsf.Add(raResponse.k2Delay);
    m_ulDciTotalUsed = 0;
    m_ulDci = raResponse.ulMsg3Dci;

    m_cmacSapUser->SetTemporaryCellRnti(m_rnti);
    // in principle we should wait for contention resolution,
    // but in the current NR model when two or more identical
    // preambles are sent no one is received, so there is no need
    // for contention resolution
    m_cmacSapUser->NotifyRandomAccessSuccessful();
    // Trigger Tx opportunity for Message 3 over LC 0
    // this is needed since Message 3's UL GRANT is in the RAR, not in UL-DCIs
    const uint8_t lc0Lcid = 0;
    auto lc0InfoIt = m_lcInfoMap.find(lc0Lcid);
    NS_ASSERT_MSG(lc0InfoIt != m_lcInfoMap.end(),
                  "LC0 not mapped to this UE MAC with bwpId:" << GetBwpId());
    auto lc0BsrIt = m_ulBsrReceived.find(lc0Lcid);
    if ((lc0BsrIt != m_ulBsrReceived.end()) && (lc0BsrIt->second.txQueueSize > 0))
    {
        NS_LOG_INFO(
            "Notify RLC about transmission opportunity for sending RRC CONNECTION REQUEST.");
        NS_ASSERT_MSG(raResponse.ulMsg3Dci->m_tbSize > lc0BsrIt->second.txQueueSize,
                      "segmentation of Message 3 is not allowed");
        NrMacSapUser::TxOpportunityParameters txOpParams;
        txOpParams.lcid = lc0Lcid;
        txOpParams.rnti = m_rnti;
        txOpParams.bytes = raResponse.ulMsg3Dci->m_tbSize;
        txOpParams.layer = 0;
        txOpParams.harqId = 0;
        txOpParams.componentCarrierId = GetBwpId();

        lc0InfoIt->second.macSapUser->NotifyTxOpportunity(txOpParams);
        lc0BsrIt->second.txQueueSize = 0;
        lc0BsrIt->second.retxQueueSize = 0;
        lc0BsrIt->second.statusPduSize = 0;
        NS_LOG_INFO("BSR erase lcid=" << +lc0Lcid << " reason Msg3 sent");
        m_ulBsrReceived.erase(lc0BsrIt);
    }
}

void
NrUeMac::ProcessUlDci(const Ptr<NrUlDciMessage>& dciMsg)
{
    NS_LOG_FUNCTION(this);

    SfnSf dataSfn = m_currentSlot;
    dataSfn.Add(dciMsg->GetKDelay());

    // Saving the data we need in DoTransmitPdu
    m_ulDciSfnsf = dataSfn;
    m_ulDciTotalUsed = 0;
    m_ulDci = dciMsg->GetDciInfoElement();

    m_macRxedCtrlMsgsTrace(m_imsi, m_currentSlot, GetCellId(), m_rnti, GetBwpId(), dciMsg);

    NS_LOG_INFO("UL DCI received, transmit data in slot "
                << dataSfn << " Harq Process " << +m_ulDci->m_harqProcess << " TBS "
                << m_ulDci->m_tbSize << " total queue " << GetTotalBufSize());

    if (m_ulDci->m_ndi == 0)
    {
        // This method will retransmit the data saved in the harq buffer
        TransmitRetx();
        m_macUeStateMachine(m_imsi,
                            m_currentSlot,
                            GetCellId(),
                            m_rnti,
                            GetBwpId(),
                            m_srState,
                            m_ulBsrReceived,
                            m_ulDci->m_ndi,
                            "ProcessUlDci");

        // This method will transmit a new BSR. ntn-patch 17: with BSR
        // triggering, a BSR the PDU carried is in the HARQ buffer already.
        if (!m_bsrTriggering)
        {
            SendBufferStatusReport(dataSfn, m_ulDci->m_symStart);
        }
    }
    else if (m_ulDci->m_ndi == 1)
    {
        ++m_ulNewTxGrants;
        // ntn-patch 17: TS 38.321 5.4.5. Decide whether this new transmission
        // carries a Regular or Periodic BSR before LCP fills it.
        constexpr uint32_t kShortBsrBytes = 5; // nr's SHORT_BSR subPDU
        uint8_t bsrType = BSR_UNCONDITIONAL;
        m_bsrReservedBytes = kShortBsrBytes;
        if (m_bsrTriggering)
        {
            m_bsrReservedBytes = 0;
            if (m_periodicBsrExpiry.IsStrictlyPositive() &&
                Simulator::Now() >= m_periodicBsrExpiry)
            {
                m_periodicBsrTriggered = true; // periodicBSR-Timer expired
                m_periodicBsrExpiry = Time(0);
            }
            if (m_regularBsrTriggered || m_periodicBsrTriggered)
            {
                const uint32_t pending = GetPendingUlBytesWithHeaders();
                if (pending <= m_ulDci->m_tbSize && pending + kShortBsrBytes > m_ulDci->m_tbSize)
                {
                    // The grant holds all pending data but not the BSR as
                    // well: all triggered BSRs may be cancelled.
                    m_regularBsrTriggered = false;
                    m_periodicBsrTriggered = false;
                    bsrType = UINT8_MAX;
                }
                else
                {
                    bsrType = m_regularBsrTriggered ? BSR_REGULAR : BSR_PERIODIC;
                    m_bsrReservedBytes = kShortBsrBytes;
                }
            }
            else
            {
                bsrType = UINT8_MAX; // none triggered: padding BSR at most
            }
        }

        // ntn-patch 16: TS 38.321 5.4.6, a triggered PHR goes into a new
        // transmission when the grant can hold it (LCP: after the BSR, before
        // data, TS 38.321 5.4.3.1.3).
        m_phrReservedBytes = PreparePhr();
        SendNewData();
        if (m_phrReservedBytes > 0)
        {
            SendPhr(dataSfn, m_ulDci->m_symStart);
            m_phrReservedBytes = 0;
        }
        m_macUeStateMachine(m_imsi,
                            m_currentSlot,
                            GetCellId(),
                            m_rnti,
                            GetBwpId(),
                            m_srState,
                            m_ulBsrReceived,
                            m_ulDci->m_ndi,
                            "ProcessUlDci");

        NS_LOG_INFO("After sending NewData, bufSize " << GetTotalBufSize());

        // Send a new BSR. SendNewData() already took into account the size of
        // the BSR. ntn-patch 17: with BsrTriggering only a triggered BSR, or a
        // Padding BSR when the padding holds the CE (TS 38.321 5.4.5).
        if (bsrType != UINT8_MAX)
        {
            SendBufferStatusReport(dataSfn, m_ulDci->m_symStart, bsrType);
        }
        else if (m_ulDci->m_tbSize >= m_ulDciTotalUsed + kShortBsrBytes)
        {
            SendBufferStatusReport(dataSfn, m_ulDci->m_symStart, BSR_PADDING);
        }
        m_bsrReservedBytes = kShortBsrBytes;

        NS_LOG_INFO("UL DCI processing done, sent to PHY a total of "
                    << m_ulDciTotalUsed << " B out of " << m_ulDci->m_tbSize
                    << " allocated bytes ");

        if (GetTotalBufSize() == 0)
        {
            m_srState = INACTIVE;
            // The buffer was drained by a transport block that also carried the buffer status,
            // so the pending scheduling requests are cancelled and their sr-ProhibitTimer is
            // stopped (3GPP TS 38.321, clause 5.4.4): a request triggered by data arriving later
            // must not be held back by the request that this grant already answered, and it gets
            // the full sr-TransMax budget of its own.
            m_lastSrSent = Time::Min();
            m_srCounter = 0;
            NS_LOG_INFO("m_srState = ACTIVE -> INACTIVE, bufSize " << GetTotalBufSize());
        }
    }
}

void
NrUeMac::TransmitRetx()
{
    NS_LOG_FUNCTION(this);

    Ptr<PacketBurst> pb = m_miUlHarqProcessesPacket.at(m_ulDci->m_harqProcess).m_pktBurst;

    if (pb == nullptr)
    {
        NS_LOG_WARN(
            "The previous transmission did not contain any new data; "
            "probably it was BSR only. To not send an old BSR to the scheduler, "
            "we don't send anything back in this allocation. Eventually, "
            "the Harq timer at gnb will expire, and soon this allocation will be forgotten.");
        return;
    }

    NS_LOG_DEBUG("UE MAC RETX HARQ " << +m_ulDci->m_harqProcess);

    NS_ASSERT(pb->GetNPackets() > 0);

    for (auto j = pb->Begin(); j != pb->End(); ++j)
    {
        Ptr<Packet> pkt = (*j)->Copy();
        NrRadioBearerTag bearerTag;
        if (!pkt->PeekPacketTag(bearerTag))
        {
            NS_FATAL_ERROR("No radio bearer tag");
        }
        m_phySapProvider->SendMacPdu(pkt, m_ulDciSfnsf, m_ulDci->m_symStart, m_ulDci->m_rnti);
    }

    m_miUlHarqProcessesPacketTimer.at(m_ulDci->m_harqProcess) = GetNumHarqProcess();
}

void
NrUeMac::SendRetxData(uint32_t tbLimit, uint32_t activeLcsRetx)
{
    NS_LOG_FUNCTION(this);

    if (activeLcsRetx == 0)
    {
        NS_LOG_INFO("Send Retx called, but active LCS retransmitting is 0");
        return;
    }

    // Share what is left of the TB equally among the LCs with a RETX queue.
    uint32_t lcsLeft = activeLcsRetx;
    for (auto& itBsr : m_ulBsrReceived)
    {
        auto& bsr = itBsr.second;
        if (bsr.retxQueueSize == 0)
        {
            continue;
        }
        const uint32_t room = (tbLimit - m_ulDciTotalUsed) / lcsLeft--;
        const uint32_t bytes = TxOppFitting(bsr.retxQueueSize, room);
        if (bytes < RLC_AM_MIN_TXOP_BYTES)
        {
            NS_LOG_DEBUG("Only " << room << " B left for the RETX of LCID " << +bsr.lcid);
            continue;
        }

        NrMacSapUser::TxOpportunityParameters txParams;
        txParams.lcid = bsr.lcid;
        txParams.rnti = m_rnti;
        txParams.bytes = bytes;
        txParams.layer = 0;
        txParams.harqId = m_ulDci->m_harqProcess;
        txParams.componentCarrierId = GetBwpId();

        NS_LOG_INFO("Notifying RLC of LCID " << +bsr.lcid << " of a TxOpp of " << bytes
                                             << " B for a RETX PDU");

        // Update the cached BSR first: RLC may report its new buffer from within the TxOpportunity.
        bsr.retxQueueSize -= std::min(bytes, bsr.retxQueueSize);
        m_lcInfoMap.at(bsr.lcid).macSapUser->NotifyTxOpportunity(txParams);
    }
}

void
NrUeMac::SendTxData(uint32_t tbLimit, uint32_t activeTx)
{
    NS_LOG_FUNCTION(this);

    if (activeTx == 0)
    {
        NS_LOG_DEBUG("No active Tx flows for this UL-DCI");
        return;
    }

    constexpr uint8_t signalingLcid = 1;
    constexpr uint32_t rlcAmWorstCaseHeaderBytes = 4;

    // Apply shortest-job first policy to prioritize lcids with less data
    while (m_ulDciTotalUsed < tbLimit)
    {
        uint32_t availableBytes = tbLimit - m_ulDciTotalUsed;
        if (availableBytes < MinDataSubPduBytes())
        {
            NS_LOG_WARN("Not enough bytes available to send a TxPDU, skipping");
            break;
        }

        uint32_t smallestBufferBytes = std::numeric_limits<uint32_t>::max();
        uint8_t smallestBufferBsrLcid = std::numeric_limits<uint8_t>::max();
        auto signalingIt = m_ulBsrReceived.find(signalingLcid);
        if (signalingIt != m_ulBsrReceived.end() && signalingIt->second.txQueueSize > 0)
        {
            // Prioritize LCID 1 (signaling) when it has pending data.
            // We assume a worst-case RLC AM header of 4 bytes to reduce
            // fragmentation of small signaling SDUs.
            // Example: a 4-byte signaling SDU may need 8 bytes TxOpportunity
            // (4 payload + 4 AM header) to be sent in one shot.
            smallestBufferBsrLcid = signalingLcid;
            smallestBufferBytes = signalingIt->second.txQueueSize + rlcAmWorstCaseHeaderBytes;
            NS_LOG_DEBUG("LCID " << +signalingLcid
                                 << " has signaling priority, selecting it first");
        }
        else
        {
            for (auto& itBsr : m_ulBsrReceived)
            {
                const auto& bsr = itBsr.second;
                // Skip lcid with empty queue
                if (bsr.txQueueSize == 0)
                {
                    NS_LOG_DEBUG("LCID " << +bsr.lcid << "has no data to send, skipping");
                    continue;
                }
                if (bsr.txQueueSize < smallestBufferBytes)
                {
                    NS_LOG_DEBUG("LCID " << +bsr.lcid
                                         << " has less data to send so far, prioritizing it");
                    smallestBufferBsrLcid = bsr.lcid;
                    smallestBufferBytes = bsr.txQueueSize;
                }
            }
        }

        if (smallestBufferBytes == std::numeric_limits<uint32_t>::max())
        {
            NS_LOG_INFO("No LCID left to offer transmission opportunity, even though there are "
                        "bytes available.");
            break;
        }

        // We need to allocate at least RLC_AM_MIN_TXOP_BYTES per LCID due to RLC limitations,
        // but at most what fits in availableBytes with the MAC subheader.
        uint32_t bytesPerLcId =
            TxOppFitting(std::max<uint32_t>(smallestBufferBytes, RLC_AM_MIN_TXOP_BYTES),
                         availableBytes);

        auto& bsr = m_ulBsrReceived.at(smallestBufferBsrLcid);
        NS_LOG_DEBUG("LCID " << +bsr.lcid << " assigned a TxOp of " << bytesPerLcId << "/"
                             << bsr.txQueueSize << ", from the pool of " << availableBytes
                             << "B available");

        NrMacSapUser::TxOpportunityParameters txParams;
        txParams.lcid = bsr.lcid;
        txParams.rnti = m_rnti;
        txParams.bytes = bytesPerLcId;
        txParams.layer = 0;
        txParams.harqId = m_ulDci->m_harqProcess;
        txParams.componentCarrierId = GetBwpId();

        NS_LOG_INFO("Notifying RLC of LCID " << +bsr.lcid
                                             << " of a TxOpp "
                                                "of "
                                             << bytesPerLcId << " B for a TX PDU");
        // Update the cached BSR first: RLC may report its new buffer from within the
        // TxOpportunity. bytesPerLcId can exceed the queue, as it is at least the RLC minimum.
        bsr.txQueueSize -= std::min(bytesPerLcId, bsr.txQueueSize);
        m_lcInfoMap.at(bsr.lcid).macSapUser->NotifyTxOpportunity(txParams);
    }
}

void
NrUeMac::SendNewData()
{
    NS_LOG_FUNCTION(this);
    // New transmission -> empty pkt buffer queue (for deleting eventual pkts not acked )
    Ptr<PacketBurst> pb = CreateObject<PacketBurst>();
    m_miUlHarqProcessesPacket.at(m_ulDci->m_harqProcess).m_pktBurst = pb;
    m_miUlHarqProcessesPacket.at(m_ulDci->m_harqProcess).m_lcidList.clear();
    NS_LOG_INFO("Reset HARQP " << +m_ulDci->m_harqProcess);

    // Send STATUS first, reserving its MAC subheader and the SHORT_BSR.
    SendNewStatusData();

    // Let's count how many LC we have, that are waiting with some data
    uint16_t activeLcsRetx = 0;
    uint16_t activeLcsTx = 0;
    for (const auto& itBsr : m_ulBsrReceived)
    {
        if (itBsr.second.retxQueueSize > 0)
        {
            activeLcsRetx++;
        }
        if (itBsr.second.txQueueSize > 0)
        {
            activeLcsTx++;
        }
    }

    // STATUS admission already reserved the SHORT_BSR (ntn-patch 17: only when a BSR is
    // triggered) and the PHR MAC CE (ntn-patch 16). Use the remaining space for retransmissions
    // and new data.
    const uint32_t ceReserved = m_bsrReservedBytes + m_phrReservedBytes;
    NS_ASSERT_MSG(m_ulDciTotalUsed + ceReserved <= m_ulDci->m_tbSize,
                  "The UL TBS of " << m_ulDci->m_tbSize << " B cannot fit the " << ceReserved
                                   << " B of MAC CEs after " << m_ulDciTotalUsed
                                   << " B of STATUS PDUs");
    const uint32_t tbLimit = m_ulDci->m_tbSize - ceReserved;

    if (activeLcsRetx == 0 && activeLcsTx == 0 && m_ulDciTotalUsed < tbLimit)
    {
        NS_LOG_LOGIC("This UE tx opportunity will be wasted: " << tbLimit - m_ulDciTotalUsed
                                                               << " bytes.");
    }

    if (activeLcsRetx > 0)
    {
        if (m_ulDciTotalUsed + activeLcsRetx * MinDataSubPduBytes() > tbLimit)
        {
            NS_LOG_DEBUG("The overhead for transmitting retx data is greater than the space for "
                         "transmitting it. Ignore the "
                         << tbLimit - m_ulDciTotalUsed << " B left.");
        }
        else
        {
            SendRetxData(tbLimit, activeLcsRetx);
        }
    }

    NS_ASSERT_MSG(m_ulDciTotalUsed <= tbLimit,
                  "The UL TBS of " << m_ulDci->m_tbSize << " B cannot fit the " << ceReserved
                                   << " B of MAC CEs after " << m_ulDciTotalUsed
                                   << " B of STATUS and RETX PDUs");

    if (activeLcsTx > 0)
    {
        if (m_ulDciTotalUsed + activeLcsTx * MinDataSubPduBytes() > tbLimit)
        {
            NS_LOG_DEBUG("The overhead for transmitting new data is greater than the space for "
                         "transmitting it. Ignore the "
                         << tbLimit - m_ulDciTotalUsed << " B left.");
        }
        else
        {
            SendTxData(tbLimit, activeLcsTx);
        }
    }

    // If we did not used the packet burst, explicitly signal it to the HARQ
    // retx, if any.
    if (m_ulDciTotalUsed == 0)
    {
        m_miUlHarqProcessesPacket.at(m_ulDci->m_harqProcess).m_pktBurst = nullptr;
        m_miUlHarqProcessesPacket.at(m_ulDci->m_harqProcess).m_lcidList.clear();
    }
}

void
NrUeMac::SendNewStatusData()
{
    NS_LOG_FUNCTION(this);

    for (auto& bsrIt : m_ulBsrReceived)
    {
        auto& bsr = bsrIt.second;

        if (bsr.statusPduSize > 0)
        {
            if (m_ulDciTotalUsed + bsr.statusPduSize + MacSubheaderBytes(bsr.statusPduSize) +
                    m_bsrReservedBytes + m_phrReservedBytes <=
                m_ulDci->m_tbSize)
            {
                NrMacSapUser::TxOpportunityParameters txParams;
                txParams.lcid = bsr.lcid;
                txParams.rnti = m_rnti;
                txParams.bytes = bsr.statusPduSize;
                txParams.layer = 0;
                txParams.harqId = m_ulDci->m_harqProcess;
                txParams.componentCarrierId = GetBwpId();

                NS_LOG_INFO("Notifying RLC of LCID " << +bsr.lcid
                                                     << " of a TxOpp "
                                                        "of "
                                                     << bsr.statusPduSize << " B for a status PDU");

                // Update the cached BSR first: RLC may report its new buffer from within the
                // TxOpportunity.
                bsr.statusPduSize = 0;
                m_lcInfoMap.at(bsr.lcid).macSapUser->NotifyTxOpportunity(txParams);
            }
            else
            {
                NS_LOG_WARN("Cannot send StatusPdu of " << bsr.statusPduSize
                                                        << " B, we already used all the TBS");
            }
        }
    }
}

void
NrUeMac::DoReceiveControlMessage(Ptr<NrControlMessage> msg)
{
    NS_LOG_FUNCTION(this << msg);

    switch (msg->GetMessageType())
    {
    case (NrControlMessage::UL_DCI): {
        ProcessUlDci(DynamicCast<NrUlDciMessage>(msg));
        break;
    }
    case (NrControlMessage::RAR): {
        NS_LOG_INFO("Received RAR in slot " << m_currentSlot);

        m_macRxedCtrlMsgsTrace(m_imsi, m_currentSlot, GetCellId(), m_rnti, GetBwpId(), msg);

        if (!m_waitingForRaResponse && m_raWindowStartOffset.IsStrictlyPositive() &&
            m_noRaResponseReceivedEvent.IsPending())
        {
            Ptr<NrRarMessage> early = DynamicCast<NrRarMessage>(msg);
            for (auto it = early->RarListBegin(); it != early->RarListEnd(); ++it)
            {
                if (it->rarPayload.raPreambleId == m_raPreambleId)
                {
                    NS_LOG_INFO("RAR for preamble " << +m_raPreambleId
                                                    << " arrived before ra-ResponseWindow opened"
                                                    << " (NTN start offset "
                                                    << m_raWindowStartOffset.As(Time::MS)
                                                    << "), ignored");
                    m_rarOutsideWindowTrace(m_imsi);
                    break;
                }
            }
        }
        if (m_waitingForRaResponse)
        {
            Ptr<NrRarMessage> rarMsg = DynamicCast<NrRarMessage>(msg);
            NS_LOG_LOGIC("got RAR with RA-RNTI " << +rarMsg->GetRaRnti() << ", expecting "
                                                 << +m_raRnti);
            for (auto it = rarMsg->RarListBegin(); it != rarMsg->RarListEnd(); ++it)
            {
                if (it->rarPayload.raPreambleId == m_raPreambleId)
                {
                    // TS 38.321 5.1.4: the first MAC subPDU whose RAPID matches the transmitted
                    // preamble completes RAR reception. A RAR can also carry a stale entry for the
                    // same (wrapped) preamble ID that another UE abandoned; handling a second match
                    // made the RRC see "random access successful" twice and abort in IDLE_CONNECTING.
                    RecvRaResponse(it->rarPayload);
                    break;
                }
            }
        }
        break;
    }

    default:
        NS_LOG_LOGIC("Control message not supported/expected: " << msg->GetMessageType());
    }
}

NrUePhySapUser*
NrUeMac::GetPhySapUser()
{
    return m_phySapUser;
}

void
NrUeMac::SetPhySapProvider(NrPhySapProvider* ptr)
{
    m_phySapProvider = ptr;
}

void
NrUeMac::RaResponseTimeout(bool contention)
{
    NS_LOG_FUNCTION(this << contention);
    m_waitingForRaResponse = false;
    // 3GPP 36.321 5.1.4
    ++m_preambleTransmissionCounter;
    // fire RA response timeout trace
    m_raResponseTimeoutTrace(m_imsi,
                             contention,
                             m_preambleTransmissionCounter,
                             m_rachConfig.preambleTransMax + 1);
    if (m_preambleTransmissionCounter == m_rachConfig.preambleTransMax + 1)
    {
        NS_LOG_INFO("RAR timeout, preambleTransMax reached => giving up");
        m_cmacSapUser->NotifyRandomAccessFailed();
    }
    else
    {
        NS_LOG_INFO("RAR timeout while waiting for raPreambleId " << +m_raPreambleId
                                                                  << " re-send preamble");
        if (contention)
        {
            RandomlySelectAndSendRaPreamble();
        }
        else
        {
            SendRaPreamble(contention);
        }
    }
}

void
NrUeMac::DoConfigureRach(NrUeCmacSapProvider::RachConfig rc)
{
    NS_LOG_FUNCTION(this);
    m_rachConfig = rc;
    m_rachConfigured = true;
}

NrUeCmacSapProvider::RachConfig
NrUeMac::GetRachConfig() const
{
    return m_rachConfig;
}

void
NrUeMac::DoStartContentionBasedRandomAccessProcedure()
{
    NS_LOG_FUNCTION(this);
    RandomlySelectAndSendRaPreamble();
}

void
NrUeMac::RandomlySelectAndSendRaPreamble()
{
    NS_LOG_FUNCTION(this);
    // 3GPP 36.321 5.1.1
    NS_ASSERT_MSG(m_rachConfigured, "RACH not configured " << ", cellId " << GetCellId());
    // assume that there is no Random Access Preambles group B
    m_raPreambleId =
        m_raPreambleUniformVariable->GetInteger(0, m_rachConfig.numberOfRaPreambles - 1);
    NS_LOG_DEBUG(m_currentSlot << " Received System Information, send to PHY the "
                                  "RA preamble: "
                               << +m_raPreambleId << ", cellId " << GetCellId());
    SendRaPreamble(true);
}

void
NrUeMac::SendRaPreamble(bool contention)
{
    NS_LOG_FUNCTION(this << (uint32_t)m_raPreambleId << contention << ", cellId " << GetCellId());

    if (contention)
    {
        // m_raPreambleId = m_raPreambleUniformVariable->GetInteger (0, 64 - 1);
        m_raPreambleId = g_raPreambleId++;
        bool preambleOverflow = m_raPreambleId == 255;
        m_raPreambleId += preambleOverflow;
        g_raPreambleId += preambleOverflow;
    }
    /*raRnti should be subframeNo -1 */
    m_raRnti = 1;

    // 3GPP 36.321 5.1.4
    m_phySapProvider->SendRachPreamble(m_raPreambleId, m_raRnti);
    NS_LOG_INFO(" Sent preamble id " << +m_raPreambleId
                                     << " at: " << Simulator::Now().As(Time::MS));
    // NTN (TS 38.321 5.1.4, Rel-17): the window starts T_TA + k_mac after the
    // preamble, so the UE does not monitor (or time out on) a RAR that cannot
    // have arrived yet. Both window edges move by the offset; with a zero
    // offset this is the terrestrial window, unchanged.
    Time raWindowBegin = m_phySapProvider->GetSlotPeriod() + m_raWindowStartOffset;
    Time raWindowEnd = m_phySapProvider->GetSlotPeriod() * (6 + m_rachConfig.raResponseWindowSize) +
                       m_raWindowStartOffset;
    if (m_raWindowStartOffset.IsStrictlyPositive())
    {
        m_waitingForRaResponse = false;
    }
    Simulator::Schedule(raWindowBegin, &NrUeMac::StartWaitingForRaResponse, this);
    m_noRaResponseReceivedEvent =
        Simulator::Schedule(raWindowEnd, &NrUeMac::RaResponseTimeout, this, contention);

    // Tracing purposes
    Ptr<NrRachPreambleMessage> rachMsg = Create<NrRachPreambleMessage>();
    rachMsg->SetSourceBwpArfcn(m_phySapProvider->GetArfcn());
    m_macTxedCtrlMsgsTrace(m_imsi, m_currentSlot, GetCellId(), m_rnti, GetBwpId(), rachMsg);
}

void
NrUeMac::StartWaitingForRaResponse()
{
    NS_LOG_FUNCTION(this);
    m_waitingForRaResponse = true;
}

void
NrUeMac::DoStartNonContentionBasedRandomAccessProcedure(uint16_t rnti,
                                                        [[maybe_unused]] uint8_t preambleId,
                                                        uint8_t prachMask)
{
    NS_LOG_FUNCTION(this << rnti << (uint16_t)preambleId << (uint16_t)prachMask);
    NS_ASSERT_MSG(prachMask == 0,
                  "requested PRACH MASK = " << (uint32_t)prachMask
                                            << ", but only PRACH MASK = 0 is supported");
    m_rnti = rnti;
    m_raPreambleId = preambleId;
    m_preambleTransmissionCounter = 0;
    bool contention = false;
    SendRaPreamble(contention);
}

void
NrUeMac::AddLc(uint8_t lcId, NrUeCmacSapProvider::LogicalChannelConfig lcConfig, NrMacSapUser* msu)
{
    NS_LOG_FUNCTION(this << " lcId" << (uint32_t)lcId);
    NS_ASSERT_MSG(m_lcInfoMap.find(lcId) == m_lcInfoMap.end(),
                  "cannot add channel because LCID " << (uint16_t)lcId << " is already present");

    LcInfo lcInfo;
    lcInfo.lcConfig = lcConfig;
    lcInfo.macSapUser = msu;
    m_lcInfoMap[lcId] = lcInfo;
}

void
NrUeMac::DoRemoveLc(uint8_t lcId)
{
    NS_LOG_FUNCTION(this << " lcId" << lcId);
    NS_ASSERT_MSG(m_lcInfoMap.find(lcId) != m_lcInfoMap.end(),
                  "could not find LCID " << (uint16_t)lcId);
    m_lcInfoMap.erase(lcId);
    m_ulBsrReceived.erase(lcId); // empty BSR buffer for this lcId
    // for (auto& harqProcess: m_miUlHarqProcessesPacket)
    //{
    //     uint32_t packets = harqProcess.m_pktBurst->GetNPackets();
    //     auto itPackets = harqProcess.m_pktBurst->GetPackets();
    //     for (uint32_t p = 0; p < packets; p++)
    //     {
    //         uint32_t i = packets-p-1;
    //         if (harqProcess.m_lcidList.at(i) == lcId)
    //         {
    //             // how to erase from packetburst?
    //             //auto it = itPackets.begin();
    //             //std::advance(it, i);
    //             //harqProcess.m_pktBurst->GetPackets().erase(it);
    //             auto it2 = harqProcess.m_lcidList.begin();
    //             std::advance(it2, i);
    //             harqProcess.m_lcidList.erase(it2);
    //         }
    //     }
    // }
}

NrMacSapProvider*
NrUeMac::GetUeMacSapProvider()
{
    return m_macSapProvider;
}

void
NrUeMac::DoReset()
{
    NS_LOG_FUNCTION(this);
    auto it = m_lcInfoMap.begin();
    while (it != m_lcInfoMap.end())
    {
        // don't delete CCCH)
        if (it->first == 0)
        {
            ++it;
        }
        else
        {
            // note: use of postfix operator preserves validity of iterator
            m_lcInfoMap.erase(it++);
        }
    }
    // note: rnti will be assigned by the gNB using RA response message
    m_rnti = 0;
    m_noRaResponseReceivedEvent.Cancel();
    m_rachConfigured = false;
    m_ulBsrReceived.clear();
    // ntn-patch 16: a PHR is triggered when the MAC entity is (re)configured
    // for a new cell (TS 38.321 5.4.6); the timers restart from there.
    m_phrTriggered = true;
    m_phrPeriodicExpiry = Time(0);
    m_phrProhibitExpiry = Time(0);
    // ntn-patch 17: no BSR stays triggered across a MAC reset.
    m_regularBsrTriggered = false;
    m_rlcReportedBytes.clear();
    m_periodicBsrTriggered = false;
    m_periodicBsrExpiry = Time(0);
    m_srState = INACTIVE;
    m_lastSrSent = Time::Min();
    m_srCounter = 0;
}

//////////////////////////////////////////////

int64_t
NrUeMac::AssignStreams(int64_t stream)
{
    return DoAssignStreams(stream);
}

int64_t
NrUeMac::DoAssignStreams(int64_t stream)
{
    NS_LOG_FUNCTION(this << stream);
    m_raPreambleUniformVariable->SetStream(stream);
    return 1;
}

} // namespace ns3
