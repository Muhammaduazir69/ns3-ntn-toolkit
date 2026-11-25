// Copyright (c) 2011 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only
//
// Author: Manuel Requena <manuel.requena@cttc.es>
//         Nicola Baldo <nbaldo@cttc.es>

#include "nr-rlc-am.h"

#include "nr-rlc-am-header.h"
#include "nr-rlc-sdu-status-tag.h"
#include "nr-rlc-tag.h"

#include "ns3/abort.h"
#include "ns3/log.h"
#include "ns3/simulator.h"

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("NrRlcAm");

NS_OBJECT_ENSURE_REGISTERED(NrRlcAm);

NrRlcAm::NrRlcAm()
{
    NS_LOG_FUNCTION(this);

    // Buffers
    m_txonBufferSize = 0;
    m_retxBuffer.resize(1024);
    m_retxBufferSize = 0;
    m_txedBuffer.resize(1024);
    m_txedBufferSize = 0;

    m_statusPduRequested = false;
    m_statusPduBufferSize = 0;

    // State variables: transmitting side
    m_windowSize = 512;
    m_vtA = 0;
    m_vtMs = m_vtA + m_windowSize;
    m_vtS = 0;
    m_pollSn = 0;

    // State variables: receiving side
    m_vrR = 0;
    m_vrMr = m_vrR + m_windowSize;
    m_vrX = 0;
    m_vrMs = 0;
    m_vrH = 0;

    // Counters
    m_pduWithoutPoll = 0;
    m_byteWithoutPoll = 0;

    // Configurable parameters
    m_maxRetxThreshold = 5;
    m_pollPdu = 1;
    m_pollByte = 50;

    // SDU reassembling process
    m_reassemblingState = WAITING_S0_FULL;
    m_expectedSeqNumber = 0;

    m_pollRetransmitTimerJustExpired = false;
}

NrRlcAm::~NrRlcAm()
{
    NS_LOG_FUNCTION(this);
}

TypeId
NrRlcAm::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::NrRlcAm")
            .SetParent<NrRlc>()
            .SetGroupName("Nr")
            .AddConstructor<NrRlcAm>()
            .AddAttribute("PollRetransmitTimer",
                          "Value of the t-PollRetransmit timer (See section 7.3 of 3GPP TS 36.322)",
                          TimeValue(MilliSeconds(20)),
                          MakeTimeAccessor(&NrRlcAm::m_pollRetransmitTimerValue),
                          MakeTimeChecker())
            .AddAttribute("ReorderingTimer",
                          "Value of the t-Reordering timer (See section 7.3 of 3GPP TS 36.322)",
                          TimeValue(MilliSeconds(10)),
                          MakeTimeAccessor(&NrRlcAm::m_reorderingTimerValue),
                          MakeTimeChecker())
            .AddAttribute("StatusProhibitTimer",
                          "Value of the t-StatusProhibit timer (See section 7.3 of 3GPP TS 36.322)",
                          TimeValue(MilliSeconds(10)),
                          MakeTimeAccessor(&NrRlcAm::m_statusProhibitTimerValue),
                          MakeTimeChecker())
            .AddAttribute(
                "BufferStatusReportTimer",
                "How much to wait to issue a new Buffer Status Report since the last time "
                "a new SDU was received",
                TimeValue(MilliSeconds(20)),
                MakeTimeAccessor(&NrRlcAm::m_bsrTimerValue),
                MakeTimeChecker())
            .AddAttribute("TxOpportunityForRetxAlwaysBigEnough",
                          "If true, always pretend that the size of a TxOpportunity is big enough "
                          "for retransmission. If false (default and realistic behavior), no retx "
                          "is performed unless the corresponding TxOpportunity is big enough.",
                          BooleanValue(false),
                          MakeBooleanAccessor(&NrRlcAm::m_txOpportunityForRetxAlwaysBigEnough),
                          MakeBooleanChecker())
            .AddAttribute("MaxTxBufferSize",
                          "Maximum Size of the Transmission Buffer (in Bytes).  If zero is "
                          "configured, the buffer is unlimited.",
                          UintegerValue(10 * 1024),
                          MakeUintegerAccessor(&NrRlcAm::m_maxTxBufferSize),
                          MakeUintegerChecker<uint32_t>());
    return tid;
}

void
NrRlcAm::DoDispose()
{
    NS_LOG_FUNCTION(this);
    m_pollRetransmitTimer.Cancel();
    m_reorderingTimer.Cancel();
    m_statusProhibitTimer.Cancel();
    m_bsrTimer.Cancel();

    m_maxTxBufferSize = 0;
    m_txonBuffer.clear();
    m_txonBufferSize = 0;
    m_txedBuffer.clear();
    m_txedBufferSize = 0;
    m_retxBuffer.clear();
    m_retxBufferSize = 0;
    m_rxonBuffer.clear();
    m_sdusBuffer.clear();
    m_keepS0 = nullptr;

    NrRlc::DoDispose();
}

/**
 * RLC SAP
 */

void
NrRlcAm::DoTransmitPdcpPdu(Ptr<Packet> p)
{
    NS_LOG_FUNCTION(this << m_rnti << (uint32_t)m_lcid << p->GetSize());

    if (m_txonBufferSize + p->GetSize() <= m_maxTxBufferSize || (m_maxTxBufferSize == 0))
    {
        /** Store PDCP PDU */
        NrRlcSduStatusTag tag;
        tag.SetStatus(NrRlcSduStatusTag::FULL_SDU);
        p->AddPacketTag(tag);

        NS_LOG_LOGIC("Txon Buffer: New packet added");
        m_txonBuffer.emplace_back(p, Simulator::Now());
        m_txonBufferSize += p->GetSize();
        NS_LOG_LOGIC("NumOfBuffers = " << m_txonBuffer.size());
        NS_LOG_LOGIC("txonBufferSize = " << m_txonBufferSize);
    }
    else
    {
        // Discard full RLC SDU
        NS_LOG_LOGIC("TxonBuffer is full. RLC SDU discarded");
        NS_LOG_LOGIC("MaxTxBufferSize = " << m_maxTxBufferSize);
        NS_LOG_LOGIC("txonBufferSize    = " << m_txonBufferSize);
        NS_LOG_LOGIC("packet size     = " << p->GetSize());
        m_txDropTrace(p);
    }

    /** Transmit Buffer Status Report */
    DoTransmitBufferStatusReport();
    m_bsrTimer.Cancel();
    m_bsrTimer = Simulator::Schedule(m_bsrTimerValue, &NrRlcAm::ExpireBsrTimer, this);
}

/**
 * MAC SAP
 */

void
NrRlcAm::DoNotifyTxOpportunity(NrMacSapUser::TxOpportunityParameters txOpParams)
{
    NS_LOG_FUNCTION(this << m_rnti << (uint32_t)m_lcid << txOpParams.bytes);

    if (txOpParams.bytes < 4)
    {
        // Stingy MAC: In general, we need more bytes.
        // There are a more restrictive test for each particular case
        NS_LOG_ERROR("TxOpportunity (size = "
                     << txOpParams.bytes
                     << ") too small: the MAC scheduler assigned too few resource"
                        " blocks; skipping this transmission opportunity");
        return;
    }

    if (m_statusPduRequested && !m_statusProhibitTimer.IsPending())
    {
        if (txOpParams.bytes < m_statusPduBufferSize)
        {
            // Stingy MAC: We need more bytes for the STATUS PDU
            NS_LOG_LOGIC("TxOpportunity (size = " << txOpParams.bytes
                                                  << ") too small for the STATUS PDU (size = "
                                                  << m_statusPduBufferSize << ")");
            NS_ASSERT_MSG(false,
                          "TxOpportunity (size = "
                              << txOpParams.bytes << ") too small for the STATUS PDU (size = "
                              << m_statusPduBufferSize << ")\n"
                              << "Your MAC scheduler is assigned too few resource blocks.");
            return;
        }

        NS_LOG_LOGIC("Sending STATUS PDU");

        Ptr<Packet> packet = Create<Packet>();
        NrRlcAmHeader rlcAmHeader;
        rlcAmHeader.SetControlPdu(NrRlcAmHeader::STATUS_PDU);

        NS_LOG_LOGIC("Check for SNs to NACK from " << m_vrR.GetValue() << " to "
                                                   << m_vrMs.GetValue());
        nr::SequenceNumber10 sn;
        sn.SetModulusBase(m_vrR);
        for (sn = m_vrR; sn < m_vrMs; sn++)
        {
            NS_LOG_LOGIC("SN = " << sn);
            if (!rlcAmHeader.OneMoreNackWouldFitIn(txOpParams.bytes))
            {
                NS_LOG_LOGIC("Can't fit more NACKs in STATUS PDU");
                break;
            }
            auto pduIt = m_rxonBuffer.find(sn.GetValue());
            if (pduIt == m_rxonBuffer.end() || (!(pduIt->second.m_pduComplete)))
            {
                NS_LOG_LOGIC("adding NACK_SN " << sn.GetValue());
                rlcAmHeader.PushNack(sn.GetValue());
            }
        }
        NS_LOG_LOGIC("SN at end of NACK loop = " << sn);
        // 3GPP TS 36.322 section 6.2.2.1.4 ACK SN
        // find the  SN of the next not received RLC Data PDU
        // which is not reported as missing in the STATUS PDU.
        auto pduIt = m_rxonBuffer.find(sn.GetValue());
        while ((sn < m_vrMs) && (pduIt != m_rxonBuffer.end()) && (pduIt->second.m_pduComplete))
        {
            NS_LOG_LOGIC("SN = " << sn << " < " << m_vrMs << " = " << (sn < m_vrMs));
            sn++;
            NS_LOG_LOGIC("SN = " << sn);
            pduIt = m_rxonBuffer.find(sn.GetValue());
        }

        NS_ASSERT_MSG(sn <= m_vrMs,
                      "first SN not reported as missing = " << sn << ", VR(MS) = " << m_vrMs);
        rlcAmHeader.SetAckSn(sn);

        NS_LOG_LOGIC("RLC header: " << rlcAmHeader);
        packet->AddHeader(rlcAmHeader);

        // Sender timestamp and transmitting entity identity
        NrRlcTag rlcTag(Simulator::Now());
        rlcTag.SetTxEntityId(m_rlcEntityId);
        packet->AddByteTag(rlcTag, 1, rlcAmHeader.GetSerializedSize());
        m_txPdu(m_rnti, m_lcid, packet->GetSize());

        // Send RLC PDU to MAC layer
        NrMacSapProvider::TransmitPduParameters params;
        params.pdu = packet;
        params.rnti = m_rnti;
        params.lcid = m_lcid;
        params.layer = txOpParams.layer;
        params.harqProcessId = txOpParams.harqId;
        params.componentCarrierId = txOpParams.componentCarrierId;

        m_macSapProvider->TransmitPdu(params);

        m_statusPduRequested = false;
        m_statusPduBufferSize = 0;
        m_statusProhibitTimer = Simulator::Schedule(m_statusProhibitTimerValue,
                                                    &NrRlcAm::ExpireStatusProhibitTimer,
                                                    this);
        return;
    }
    else if (m_retxBufferSize > 0)
    {
        NS_LOG_LOGIC("retxBufferSize = " << m_retxBufferSize);
        NS_LOG_LOGIC("Sending data from Retransmission Buffer");
        NS_ASSERT(m_vtA < m_vtS);
        bool anyRetxPduFound = false;
        nr::SequenceNumber10 sn;
        sn.SetModulusBase(m_vtA);
        for (sn = m_vtA; sn < m_vtS; sn++)
        {
            uint16_t seqNumberValue = sn.GetValue();
            NS_LOG_LOGIC("SN = " << seqNumberValue << " m_pdu "
                                 << m_retxBuffer.at(seqNumberValue).m_pdu);

            if (m_retxBuffer.at(seqNumberValue).m_pdu)
            {
                anyRetxPduFound = true;
                Ptr<Packet> packet = m_retxBuffer.at(seqNumberValue).m_pdu->Copy();

                if ((packet->GetSize() <= txOpParams.bytes) ||
                    m_txOpportunityForRetxAlwaysBigEnough)
                {
                    // According to 5.2.1, the data field is left as is, but we rebuild the header
                    NrRlcAmHeader rlcAmHeader;
                    packet->RemoveHeader(rlcAmHeader);
                    NS_LOG_LOGIC("old AM RLC header: " << rlcAmHeader);

                    // Calculate the Polling Bit (5.2.2.1)
                    rlcAmHeader.SetPollingBit(NrRlcAmHeader::STATUS_REPORT_NOT_REQUESTED);

                    NS_LOG_LOGIC("polling conditions: m_txonBuffer.empty="
                                 << m_txonBuffer.empty() << " retxBufferSize=" << m_retxBufferSize
                                 << " packet->GetSize ()=" << packet->GetSize());
                    if (((m_txonBuffer.empty()) &&
                         (m_retxBufferSize ==
                          packet->GetSize() + rlcAmHeader.GetSerializedSize())) ||
                        (m_vtS >= m_vtMs) || m_pollRetransmitTimerJustExpired)
                    {
                        m_pollRetransmitTimerJustExpired = false;
                        rlcAmHeader.SetPollingBit(NrRlcAmHeader::STATUS_REPORT_IS_REQUESTED);
                        m_pduWithoutPoll = 0;
                        m_byteWithoutPoll = 0;

                        m_pollSn = m_vtS - 1;
                        NS_LOG_LOGIC("New POLL_SN = " << m_pollSn);

                        if (!m_pollRetransmitTimer.IsPending())
                        {
                            NS_LOG_LOGIC("Start PollRetransmit timer");

                            m_pollRetransmitTimer =
                                Simulator::Schedule(m_pollRetransmitTimerValue,
                                                    &NrRlcAm::ExpirePollRetransmitTimer,
                                                    this);
                        }
                        else
                        {
                            NS_LOG_LOGIC("Restart PollRetransmit timer");

                            m_pollRetransmitTimer.Cancel();
                            m_pollRetransmitTimer =
                                Simulator::Schedule(m_pollRetransmitTimerValue,
                                                    &NrRlcAm::ExpirePollRetransmitTimer,
                                                    this);
                        }
                    }

                    packet->AddHeader(rlcAmHeader);

                    NrRlcTag rlcTag;
                    rlcTag.SetSenderTimestamp(Simulator::Now());
                    rlcTag.SetTxEntityId(m_rlcEntityId);

                    packet->AddByteTag(rlcTag, 1, rlcAmHeader.GetSerializedSize());

                    NS_LOG_LOGIC("new AM RLC header: " << rlcAmHeader);

                    m_txPdu(m_rnti, m_lcid, packet->GetSize());

                    // Send RLC PDU to MAC layer
                    NrMacSapProvider::TransmitPduParameters params;
                    params.pdu = packet;
                    params.rnti = m_rnti;
                    params.lcid = m_lcid;
                    params.layer = txOpParams.layer;
                    params.harqProcessId = txOpParams.harqId;
                    params.componentCarrierId = txOpParams.componentCarrierId;

                    m_macSapProvider->TransmitPdu(params);

                    m_retxBuffer.at(seqNumberValue).m_retxCount++;
                    m_retxBuffer.at(seqNumberValue).m_waitingSince = Simulator::Now();
                    NS_LOG_INFO("Incr RETX_COUNT for SN = " << seqNumberValue);
                    if (m_retxBuffer.at(seqNumberValue).m_retxCount >= m_maxRetxThreshold)
                    {
                        NS_LOG_INFO("Max RETX_COUNT for SN = " << seqNumberValue);
                        // TS 38.331 5.3.10.3: reaching maxRetxThreshold on an AM RLC
                        // entity is a radio-link-failure trigger. Raise the indication
                        // once (the RRC decides whether to act on it). Without this the
                        // PDU is simply requeued and retransmitted indefinitely, so an
                        // undeliverable uplink (e.g. a lost RrcConnectionReconfiguration
                        // Complete on a degraded link) never fails and the peer can stay
                        // stuck awaiting it.
                        if (!m_maxRetxReachedNotified && !m_maxRetxReachedCallback.IsNull())
                        {
                            m_maxRetxReachedNotified = true;
                            m_maxRetxReachedCallback();
                        }
                    }

                    NS_LOG_INFO("Move SN = " << seqNumberValue << " back to txedBuffer");
                    m_txedBuffer.at(seqNumberValue).m_pdu =
                        m_retxBuffer.at(seqNumberValue).m_pdu->Copy();
                    m_txedBuffer.at(seqNumberValue).m_retxCount =
                        m_retxBuffer.at(seqNumberValue).m_retxCount;
                    m_txedBuffer.at(seqNumberValue).m_waitingSince =
                        m_retxBuffer.at(seqNumberValue).m_waitingSince;
                    m_txedBufferSize += m_txedBuffer.at(seqNumberValue).m_pdu->GetSize();

                    m_retxBufferSize -= m_retxBuffer.at(seqNumberValue).m_pdu->GetSize();
                    m_retxBuffer.at(seqNumberValue).m_pdu = nullptr;
                    m_retxBuffer.at(seqNumberValue).m_retxCount = 0;
                    m_retxBuffer.at(seqNumberValue).m_waitingSince = MilliSeconds(0);

                    NS_LOG_LOGIC("retxBufferSize = " << m_retxBufferSize);

                    return;
                }
                else
                {
                    // This PDU does not fit; a later NACKed SN might (AM PDUs are
                    // not re-segmented, so PDU sizes vary). Do not let one oversized
                    // PDU head-of-line block every other pending retransmission.
                    NS_LOG_LOGIC("TxOpportunity (size = "
                                 << txOpParams.bytes
                                 << ") too small for retransmission of the packet (size = "
                                 << packet->GetSize() << "); trying the next NACKed SN");
                }
            }
        }
        NS_ABORT_MSG_UNLESS(anyRetxPduFound,
                            "m_retxBufferSize > 0, but no PDU considered for retx found");
        NS_LOG_LOGIC("No pending retransmission fits in this TxOpportunity ("
                     << txOpParams.bytes << " bytes); waiting for a bigger one");
        return;
    }
    else if (m_txonBufferSize > 0)
    {
        if (txOpParams.bytes < 7)
        {
            // Stingy MAC: We need more bytes for new DATA PDUs.
            NS_LOG_ERROR("TxOpportunity (size = "
                         << txOpParams.bytes
                         << ") too small for DATA PDU; skipping this transmission opportunity");
            return;
        }

        NS_ASSERT(m_vtS <= m_vtMs);
        if (m_vtS == m_vtMs)
        {
            NS_LOG_INFO("cannot transmit new RLC PDU due to window stalling");
            return;
        }

        NS_LOG_LOGIC("Sending data from Transmission Buffer");
    }
    else
    {
        NS_LOG_LOGIC("No data pending");
        return;
    }

    //
    //
    // Build new PDU
    //
    //

    Ptr<Packet> packet = Create<Packet>();
    NrRlcAmHeader rlcAmHeader;
    rlcAmHeader.SetDataPdu();

    // Build Data field
    uint32_t nextSegmentSize = txOpParams.bytes - 4;
    uint32_t nextSegmentId = 1;
    uint32_t dataFieldAddedSize = 0;
    std::vector<Ptr<Packet>> dataField;

    // Remove the first packet from the transmission buffer.
    // If only a segment of the packet is taken, then the remaining is given back later
    if (m_txonBuffer.empty())
    {
        NS_LOG_LOGIC("No data pending");
        return;
    }

    NS_LOG_LOGIC("SDUs in TxonBuffer  = " << m_txonBuffer.size());
    NS_LOG_LOGIC("First SDU buffer  = " << m_txonBuffer.begin()->m_pdu);
    NS_LOG_LOGIC("First SDU size    = " << m_txonBuffer.begin()->m_pdu->GetSize());
    NS_LOG_LOGIC("Next segment size = " << nextSegmentSize);
    NS_LOG_LOGIC("Remove SDU from TxBuffer");
    Time firstSegmentTime = m_txonBuffer.begin()->m_waitingSince;
    Ptr<Packet> firstSegment = m_txonBuffer.begin()->m_pdu->Copy();
    m_txonBufferSize -= m_txonBuffer.begin()->m_pdu->GetSize();
    NS_LOG_LOGIC("txBufferSize      = " << m_txonBufferSize);
    m_txonBuffer.erase(m_txonBuffer.begin());

    while (firstSegment && (firstSegment->GetSize() > 0) && (nextSegmentSize > 0))
    {
        NS_LOG_LOGIC("WHILE ( firstSegment && firstSegment->GetSize > 0 && nextSegmentSize > 0 )");
        NS_LOG_LOGIC("    firstSegment size = " << firstSegment->GetSize());
        NS_LOG_LOGIC("    nextSegmentSize   = " << nextSegmentSize);
        if ((firstSegment->GetSize() > nextSegmentSize) ||
            // Segment larger than 2047 octets can only be mapped to the end of the Data field
            (firstSegment->GetSize() > 2047))
        {
            // Take the minimum size, due to the 2047-bytes 3GPP exception
            // This exception is due to the length of the LI field (just 11 bits)
            uint32_t currSegmentSize = std::min(firstSegment->GetSize(), nextSegmentSize);

            NS_LOG_LOGIC("    IF ( firstSegment > nextSegmentSize ||");
            NS_LOG_LOGIC("         firstSegment > 2047 )");

            // Segment txBuffer.FirstBuffer and
            // Give back the remaining segment to the transmission buffer
            Ptr<Packet> newSegment = firstSegment->CreateFragment(0, currSegmentSize);
            NS_LOG_LOGIC("    newSegment size   = " << newSegment->GetSize());

            // Status tag of the new and remaining segments
            // Note: This is the only place where a PDU is segmented and
            // therefore its status can change
            NrRlcSduStatusTag oldTag;
            NrRlcSduStatusTag newTag;
            firstSegment->RemovePacketTag(oldTag);
            newSegment->RemovePacketTag(newTag);
            if (oldTag.GetStatus() == NrRlcSduStatusTag::FULL_SDU)
            {
                newTag.SetStatus(NrRlcSduStatusTag::FIRST_SEGMENT);
                oldTag.SetStatus(NrRlcSduStatusTag::LAST_SEGMENT);
            }
            else if (oldTag.GetStatus() == NrRlcSduStatusTag::LAST_SEGMENT)
            {
                newTag.SetStatus(NrRlcSduStatusTag::MIDDLE_SEGMENT);
                // oldTag.SetStatus (NrRlcSduStatusTag::LAST_SEGMENT);
            }

            // Give back the remaining segment to the transmission buffer
            firstSegment->RemoveAtStart(currSegmentSize);
            NS_LOG_LOGIC(
                "    firstSegment size (after RemoveAtStart) = " << firstSegment->GetSize());
            if (firstSegment->GetSize() > 0)
            {
                firstSegment->AddPacketTag(oldTag);

                m_txonBuffer.insert(m_txonBuffer.begin(), TxPdu(firstSegment, firstSegmentTime));
                m_txonBufferSize += m_txonBuffer.begin()->m_pdu->GetSize();

                NS_LOG_LOGIC("    Txon buffer: Give back the remaining segment");
                NS_LOG_LOGIC("    Txon buffers = " << m_txonBuffer.size());
                NS_LOG_LOGIC("    Front buffer size = " << m_txonBuffer.begin()->m_pdu->GetSize());
                NS_LOG_LOGIC("    txonBufferSize = " << m_txonBufferSize);
            }
            else
            {
                // Whole segment was taken, so adjust tag
                if (newTag.GetStatus() == NrRlcSduStatusTag::FIRST_SEGMENT)
                {
                    newTag.SetStatus(NrRlcSduStatusTag::FULL_SDU);
                }
                else if (newTag.GetStatus() == NrRlcSduStatusTag::MIDDLE_SEGMENT)
                {
                    newTag.SetStatus(NrRlcSduStatusTag::LAST_SEGMENT);
                }
            }
            // Segment is completely taken or
            // the remaining segment is given back to the transmission buffer
            firstSegment = nullptr;

            // Put status tag once it has been adjusted
            newSegment->AddPacketTag(newTag);

            // Add Segment to Data field
            dataFieldAddedSize = newSegment->GetSize();
            dataField.push_back(newSegment);
            newSegment = nullptr;

            // ExtensionBit (Next_Segment - 1) = 0
            rlcAmHeader.PushExtensionBit(NrRlcAmHeader::DATA_FIELD_FOLLOWS);

            // no LengthIndicator for the last one

            nextSegmentSize -= dataFieldAddedSize;
            nextSegmentId++;

            // nextSegmentSize MUST be zero (only if segment is smaller or equal to 2047)

            // (NO more segments) ? exit
            // break;
        }
        else if ((nextSegmentSize - firstSegment->GetSize() <= 2) || m_txonBuffer.empty())
        {
            NS_LOG_LOGIC(
                "    IF nextSegmentSize - firstSegment->GetSize () <= 2 || txonBuffer.size == 0");

            // Add txBuffer.FirstBuffer to DataField
            dataFieldAddedSize = firstSegment->GetSize();
            dataField.push_back(firstSegment);
            firstSegment = nullptr;

            // ExtensionBit (Next_Segment - 1) = 0
            rlcAmHeader.PushExtensionBit(NrRlcAmHeader::DATA_FIELD_FOLLOWS);

            // no LengthIndicator for the last one

            nextSegmentSize -= dataFieldAddedSize;
            nextSegmentId++;

            NS_LOG_LOGIC("        SDUs in TxBuffer  = " << m_txonBuffer.size());
            if (!m_txonBuffer.empty())
            {
                NS_LOG_LOGIC("        First SDU buffer  = " << m_txonBuffer.begin()->m_pdu);
                NS_LOG_LOGIC(
                    "        First SDU size    = " << m_txonBuffer.begin()->m_pdu->GetSize());
            }
            NS_LOG_LOGIC("        Next segment size = " << nextSegmentSize);

            // nextSegmentSize <= 2 (only if txBuffer is not empty)

            // (NO more segments) ? exit
            // break;
        }
        else // (firstSegment->GetSize () < m_nextSegmentSize) && (m_txBuffer.size () > 0)
        {
            NS_LOG_LOGIC("    IF firstSegment < NextSegmentSize && txonBuffer.size > 0");
            // Add txBuffer.FirstBuffer to DataField
            dataFieldAddedSize = firstSegment->GetSize();
            dataField.push_back(firstSegment);

            // ExtensionBit (Next_Segment - 1) = 1
            rlcAmHeader.PushExtensionBit(NrRlcAmHeader::E_LI_FIELDS_FOLLOWS);

            // LengthIndicator (Next_Segment)  = txBuffer.FirstBuffer.length()
            rlcAmHeader.PushLengthIndicator(firstSegment->GetSize());

            nextSegmentSize -= ((nextSegmentId % 2) ? (2) : (1)) + dataFieldAddedSize;
            nextSegmentId++;

            NS_LOG_LOGIC("        SDUs in TxBuffer  = " << m_txonBuffer.size());
            if (!m_txonBuffer.empty())
            {
                NS_LOG_LOGIC("        First SDU buffer  = " << m_txonBuffer.begin()->m_pdu);
                NS_LOG_LOGIC(
                    "        First SDU size    = " << m_txonBuffer.begin()->m_pdu->GetSize());
            }
            NS_LOG_LOGIC("        Next segment size = " << nextSegmentSize);
            NS_LOG_LOGIC("        Remove SDU from TxBuffer");

            // (more segments)
            firstSegment = m_txonBuffer.begin()->m_pdu->Copy();
            firstSegmentTime = m_txonBuffer.begin()->m_waitingSince;
            m_txonBufferSize -= m_txonBuffer.begin()->m_pdu->GetSize();
            m_txonBuffer.erase(m_txonBuffer.begin());
            NS_LOG_LOGIC("        txBufferSize = " << m_txonBufferSize);
        }
    }

    //
    // Build RLC header
    //

    rlcAmHeader.SetSequenceNumber(m_vtS++);
    rlcAmHeader.SetResegmentationFlag(NrRlcAmHeader::PDU);
    rlcAmHeader.SetLastSegmentFlag(NrRlcAmHeader::LAST_PDU_SEGMENT);
    rlcAmHeader.SetSegmentOffset(0);

    NS_ASSERT_MSG(rlcAmHeader.GetSequenceNumber() < m_vtMs, "SN above TX window");
    NS_ASSERT_MSG(rlcAmHeader.GetSequenceNumber() >= m_vtA, "SN below TX window");

    // Calculate FramingInfo flag according the status of the SDUs in the DataField
    uint8_t framingInfo = 0;
    auto it = dataField.begin();

    // FIRST SEGMENT
    NrRlcSduStatusTag tag;
    NS_ASSERT_MSG((*it)->PeekPacketTag(tag), "NrRlcSduStatusTag is missing");
    (*it)->PeekPacketTag(tag);
    if ((tag.GetStatus() == NrRlcSduStatusTag::FULL_SDU) ||
        (tag.GetStatus() == NrRlcSduStatusTag::FIRST_SEGMENT))
    {
        framingInfo |= NrRlcAmHeader::FIRST_BYTE;
    }
    else
    {
        framingInfo |= NrRlcAmHeader::NO_FIRST_BYTE;
    }

    // Add all SDUs (in DataField) to the Packet
    while (it < dataField.end())
    {
        NS_LOG_LOGIC("Adding SDU/segment to packet, length = " << (*it)->GetSize());

        NS_ASSERT_MSG((*it)->PeekPacketTag(tag), "NrRlcSduStatusTag is missing");
        (*it)->RemovePacketTag(tag);
        if (packet->GetSize() > 0)
        {
            packet->AddAtEnd(*it);
        }
        else
        {
            packet = (*it);
        }
        it++;
    }

    // LAST SEGMENT (Note: There could be only one and be the first one)
    it--;
    if ((tag.GetStatus() == NrRlcSduStatusTag::FULL_SDU) ||
        (tag.GetStatus() == NrRlcSduStatusTag::LAST_SEGMENT))
    {
        framingInfo |= NrRlcAmHeader::LAST_BYTE;
    }
    else
    {
        framingInfo |= NrRlcAmHeader::NO_LAST_BYTE;
    }

    // Set the FramingInfo flag after the calculation
    rlcAmHeader.SetFramingInfo(framingInfo);

    // Calculate the Polling Bit (5.2.2.1)
    rlcAmHeader.SetPollingBit(NrRlcAmHeader::STATUS_REPORT_NOT_REQUESTED);

    m_pduWithoutPoll++;
    NS_LOG_LOGIC("PDU_WITHOUT_POLL = " << m_pduWithoutPoll);
    m_byteWithoutPoll += packet->GetSize();
    NS_LOG_LOGIC("BYTE_WITHOUT_POLL = " << m_byteWithoutPoll);

    if ((m_pduWithoutPoll >= m_pollPdu) || (m_byteWithoutPoll >= m_pollByte) ||
        ((m_txonBuffer.empty()) && (m_retxBufferSize == 0)) || (m_vtS >= m_vtMs) ||
        m_pollRetransmitTimerJustExpired)
    {
        m_pollRetransmitTimerJustExpired = false;
        rlcAmHeader.SetPollingBit(NrRlcAmHeader::STATUS_REPORT_IS_REQUESTED);
        m_pduWithoutPoll = 0;
        m_byteWithoutPoll = 0;

        m_pollSn = m_vtS - 1;
        NS_LOG_LOGIC("New POLL_SN = " << m_pollSn);

        if (!m_pollRetransmitTimer.IsPending())
        {
            NS_LOG_LOGIC("Start PollRetransmit timer");

            m_pollRetransmitTimer = Simulator::Schedule(m_pollRetransmitTimerValue,
                                                        &NrRlcAm::ExpirePollRetransmitTimer,
                                                        this);
        }
        else
        {
            NS_LOG_LOGIC("Restart PollRetransmit timer");

            m_pollRetransmitTimer.Cancel();
            m_pollRetransmitTimer = Simulator::Schedule(m_pollRetransmitTimerValue,
                                                        &NrRlcAm::ExpirePollRetransmitTimer,
                                                        this);
        }
    }

    // Build RLC PDU with DataField and Header
    NS_LOG_LOGIC("AM RLC header: " << rlcAmHeader);

    NrRlcTag rlcTag;
    rlcTag.SetSenderTimestamp(Simulator::Now());
    rlcTag.SetTxEntityId(m_rlcEntityId);

    packet->AddHeader(rlcAmHeader);
    packet->AddByteTag(rlcTag, 1, rlcAmHeader.GetSerializedSize());

    // Store new PDU into the Transmitted PDU Buffer
    NS_LOG_LOGIC("Put transmitted PDU in the txedBuffer");
    m_txedBufferSize += packet->GetSize();
    m_txedBuffer.at(rlcAmHeader.GetSequenceNumber().GetValue()).m_pdu = packet->Copy();
    m_txedBuffer.at(rlcAmHeader.GetSequenceNumber().GetValue()).m_retxCount = 0;
    m_txedBuffer.at(rlcAmHeader.GetSequenceNumber().GetValue()).m_waitingSince = Simulator::Now();

    m_txPdu(m_rnti, m_lcid, packet->GetSize());

    // Send RLC PDU to MAC layer
    NrMacSapProvider::TransmitPduParameters params;
    params.pdu = packet;
    params.rnti = m_rnti;
    params.lcid = m_lcid;
    params.layer = txOpParams.layer;
    params.harqProcessId = txOpParams.harqId;
    params.componentCarrierId = txOpParams.componentCarrierId;

    m_macSapProvider->TransmitPdu(params);
}

void
NrRlcAm::DoNotifyHarqDeliveryFailure()
{
    NS_LOG_FUNCTION(this);
}

void
NrRlcAm::DoReceivePdu(NrMacSapUser::ReceivePduParameters rxPduParams)
{
    NS_LOG_FUNCTION(this << m_rnti << (uint32_t)m_lcid << rxPduParams.p->GetSize());

    // Get RLC header parameters
    NrRlcAmHeader rlcAmHeader;
    rxPduParams.p->PeekHeader(rlcAmHeader);
    NS_LOG_LOGIC("RLC header: " << rlcAmHeader);

    // Receiver timestamp
    Time delay;
    NrRlcTag rlcTag;

    bool ret = rxPduParams.p->FindFirstMatchingByteTag(rlcTag);
    NS_ASSERT_MSG(ret, "NrRlcTag not found in RLC Header. The packet went into a real network?");

    // Old-epoch PDUs of a peer entity destroyed at handover alias into the new SN
    // space and would corrupt reassembly or the transmit window undetectably:
    // discard them.
    if (!AcceptPduFromPeerEntity(rlcTag.GetTxEntityId()))
    {
        return;
    }

    delay = Simulator::Now() - rlcTag.GetSenderTimestamp();

    m_rxPdu(m_rnti, m_lcid, rxPduParams.p->GetSize(), delay.GetNanoSeconds());

    if (rlcAmHeader.IsDataPdu())
    {
        // 5.1.3.1   Transmit operations

        // 5.1.3.1.1       General
        //
        // The transmitting side of an AM RLC entity shall prioritize transmission of RLC control
        // PDUs over RLC data PDUs. The transmitting side of an AM RLC entity shall prioritize
        // retransmission of RLC data PDUs over transmission of new AMD PDUs.
        //
        // The transmitting side of an AM RLC entity shall maintain a transmitting window according
        // to state variables VT(A) and VT(MS) as follows:
        // - a SN falls within the transmitting window if VT(A) <= SN < VT(MS);
        // - a SN falls outside of the transmitting window otherwise.
        //
        // The transmitting side of an AM RLC entity shall not deliver to lower layer any RLC data
        // PDU whose SN falls outside of the transmitting window.
        //
        // When delivering a new AMD PDU to lower layer, the transmitting side of an AM RLC entity
        // shall:
        // - set the SN of the AMD PDU to VT(S), and then increment VT(S) by one.
        //
        // The transmitting side of an AM RLC entity can receive a positive acknowledgement
        // (confirmation of successful reception by its peer AM RLC entity) for a RLC data PDU by
        // the following:
        // - STATUS PDU from its peer AM RLC entity.
        //
        // When receiving a positive acknowledgement for an AMD PDU with SN = VT(A), the
        // transmitting side of an AM RLC entity shall:
        // - set VT(A) equal to the SN of the AMD PDU with the smallest SN, whose SN falls within
        // the
        //   range VT(A) <= SN <= VT(S) and for which a positive acknowledgment has not been
        //   received yet.
        // - if positive acknowledgements have been received for all AMD PDUs associated with
        //   a transmitted RLC SDU:
        // - send an indication to the upper layers of successful delivery of the RLC SDU.

        // 5.1.3.2 Receive operations
        //
        // 5.1.3.2.1 General
        //
        // The receiving side of an AM RLC entity shall maintain a receiving window according to
        // state variables VR(R) and VR(MR) as follows:
        // - a SN falls within the receiving window if VR(R) <= SN < VR(MR);
        // - a SN falls outside of the receiving window otherwise.
        //
        // When receiving a RLC data PDU from lower layer, the receiving side of an AM RLC entity
        // shall:
        // - either discard the received RLC data PDU or place it in the reception buffer (see sub
        // clause 5.1.3.2.2);
        // - if the received RLC data PDU was placed in the reception buffer:
        // - update state variables, reassemble and deliver RLC SDUs to upper layer and start/stop
        // t-Reordering as needed (see sub clause 5.1.3.2.3).
        //
        // When t-Reordering expires, the receiving side of an AM RLC entity shall:
        // - update state variables and start t-Reordering as needed (see sub clause 5.1.3.2.4).

        nr::SequenceNumber10 seqNumber = rlcAmHeader.GetSequenceNumber();
        seqNumber.SetModulusBase(m_vrR);

        if (rlcAmHeader.GetResegmentationFlag() == NrRlcAmHeader::SEGMENT)
        {
            NS_LOG_LOGIC("PDU segment received ( SN = " << seqNumber << " )");
        }
        else if (rlcAmHeader.GetResegmentationFlag() == NrRlcAmHeader::PDU)
        {
            NS_LOG_LOGIC("PDU received ( SN = " << seqNumber << " )");
        }
        else
        {
            NS_ASSERT_MSG(false, "Neither a PDU segment nor a PDU received");
            return;
        }

        // STATUS PDU is requested
        if (rlcAmHeader.GetPollingBit() == NrRlcAmHeader::STATUS_REPORT_IS_REQUESTED)
        {
            m_statusPduRequested = true;
            m_statusPduBufferSize = 4;

            if (!m_statusProhibitTimer.IsPending())
            {
                DoTransmitBufferStatusReport();
            }
        }

        // 5.1.3.2.2 Actions when a RLC data PDU is received from lower layer
        //
        // When a RLC data PDU is received from lower layer, where the RLC data PDU contains
        // byte segment numbers y to z of an AMD PDU with SN = x, the receiving side of an AM RLC
        // entity shall:
        // - if x falls outside of the receiving window; or
        // - if byte segment numbers y to z of the AMD PDU with SN = x have been received before:
        //     - discard the received RLC data PDU;
        // - else:
        //     - place the received RLC data PDU in the reception buffer;
        //     - if some byte segments of the AMD PDU contained in the RLC data PDU have been
        //     received before:
        //         - discard the duplicate byte segments.

        NS_LOG_LOGIC("VR(R)  = " << m_vrR);
        NS_LOG_LOGIC("VR(MR) = " << m_vrMr);
        NS_LOG_LOGIC("VR(X)  = " << m_vrX);
        NS_LOG_LOGIC("VR(MS) = " << m_vrMs);
        NS_LOG_LOGIC("VR(H)  = " << m_vrH);

        // - if x falls outside of the receiving window; or
        // - if byte segment numbers y to z of the AMD PDU with SN = x have been received before:
        if (!IsInsideReceivingWindow(seqNumber))
        {
            NS_LOG_LOGIC("PDU discarded");
            return;
        }
        else
        {
            // - if some byte segments of the AMD PDU contained in the RLC data PDU have been
            // received before:
            //         - discard the duplicate byte segments.
            // note: re-segmentation of AMD PDU is currently not supported,
            // so we just check that the segment was not received before
            auto it = m_rxonBuffer.find(seqNumber.GetValue());
            if (it != m_rxonBuffer.end())
            {
                NS_ASSERT(!it->second.m_byteSegments.empty());
                NS_ASSERT_MSG(it->second.m_byteSegments.size() == 1,
                              "re-segmentation not supported");
                NS_LOG_LOGIC("PDU segment already received, discarded");
            }
            else
            {
                NS_LOG_LOGIC("Place PDU in the reception buffer ( SN = " << seqNumber << " )");
                m_rxonBuffer[seqNumber.GetValue()].m_byteSegments.push_back(rxPduParams.p);
                m_rxonBuffer[seqNumber.GetValue()].m_pduComplete = true;
            }
        }

        // 5.1.3.2.3 Actions when a RLC data PDU is placed in the reception buffer
        // When a RLC data PDU with SN = x is placed in the reception buffer,
        // the receiving side of an AM RLC entity shall:

        // - if x >= VR(H)
        //     - update VR(H) to x+ 1;

        if (seqNumber >= m_vrH)
        {
            m_vrH = seqNumber + 1;
            NS_LOG_LOGIC("New VR(H)  = " << m_vrH);
        }

        // - if all byte segments of the AMD PDU with SN = VR(MS) are received:
        //     - update VR(MS) to the SN of the first AMD PDU with SN > current VR(MS) for
        //       which not all byte segments have been received;

        auto it = m_rxonBuffer.find(m_vrMs.GetValue());
        if (it != m_rxonBuffer.end() && it->second.m_pduComplete)
        {
            int firstVrMs = m_vrMs.GetValue();
            while (it != m_rxonBuffer.end() && it->second.m_pduComplete)
            {
                m_vrMs++;
                it = m_rxonBuffer.find(m_vrMs.GetValue());
                NS_LOG_LOGIC("Incr VR(MS) = " << m_vrMs);

                NS_ASSERT_MSG(firstVrMs != m_vrMs.GetValue(), "Infinite loop in RxonBuffer");
            }
            NS_LOG_LOGIC("New VR(MS) = " << m_vrMs);
        }

        // - if x = VR(R):
        //     - if all byte segments of the AMD PDU with SN = VR(R) are received:
        //         - update VR(R) to the SN of the first AMD PDU with SN > current VR(R) for which
        //         not all byte segments have been received;
        //         - update VR(MR) to the updated VR(R) + AM_Window_Size;
        //     - reassemble RLC SDUs from any byte segments of AMD PDUs with SN that falls outside
        //     of the receiving window and in-sequence byte segments of the AMD PDU with SN = VR(R),
        //     remove RLC headers when doing so and deliver the reassembled RLC SDUs to upper layer
        //     in sequence if not delivered before;

        if (seqNumber == m_vrR)
        {
            auto it = m_rxonBuffer.find(seqNumber.GetValue());
            if (it != m_rxonBuffer.end() && it->second.m_pduComplete)
            {
                it = m_rxonBuffer.find(m_vrR.GetValue());
                int firstVrR = m_vrR.GetValue();
                while (it != m_rxonBuffer.end() && it->second.m_pduComplete)
                {
                    NS_LOG_LOGIC("Reassemble and Deliver ( SN = " << m_vrR << " )");
                    NS_ASSERT_MSG(it->second.m_byteSegments.size() == 1,
                                  "Too many segments. PDU Reassembly process didn't work");
                    ReassembleAndDeliver(it->second.m_byteSegments.front());
                    m_rxonBuffer.erase(m_vrR.GetValue());

                    m_vrR++;
                    m_vrR.SetModulusBase(m_vrR);
                    m_vrX.SetModulusBase(m_vrR);
                    m_vrMs.SetModulusBase(m_vrR);
                    m_vrH.SetModulusBase(m_vrR);
                    it = m_rxonBuffer.find(m_vrR.GetValue());

                    NS_ASSERT_MSG(firstVrR != m_vrR.GetValue(), "Infinite loop in RxonBuffer");
                }
                NS_LOG_LOGIC("New VR(R)  = " << m_vrR);
                m_vrMr = m_vrR + m_windowSize;

                NS_LOG_LOGIC("New VR(MR) = " << m_vrMr);
            }
        }

        // - if t-Reordering is running:
        //     - if VR(X) = VR(R); or
        //     - if VR(X) falls outside of the receiving window and VR(X) is not equal to VR(MR):
        //         - stop and reset t-Reordering;

        if (m_reorderingTimer.IsPending())
        {
            NS_LOG_LOGIC("Reordering timer is running");
            if ((m_vrX == m_vrR) || ((!IsInsideReceivingWindow(m_vrX)) && (m_vrX != m_vrMr)))
            {
                /// @todo stop and reset the t-Reordering
                NS_LOG_LOGIC("Stop reordering timer");
                m_reorderingTimer.Cancel();
            }
        }

        // - if t-Reordering is not running (includes the case t-Reordering is stopped due to
        // actions above):
        //     - if VR (H) > VR(R):
        //         - start t-Reordering;
        //         - set VR(X) to VR(H).

        if (!m_reorderingTimer.IsPending())
        {
            NS_LOG_LOGIC("Reordering timer is not running");
            if (m_vrH > m_vrR)
            {
                NS_LOG_LOGIC("Start reordering timer");
                m_reorderingTimer = Simulator::Schedule(m_reorderingTimerValue,
                                                        &NrRlcAm::ExpireReorderingTimer,
                                                        this);
                m_vrX = m_vrH;
                NS_LOG_LOGIC("New VR(X) = " << m_vrX);
            }
        }
    }
    else if (rlcAmHeader.IsControlPdu())
    {
        NS_LOG_INFO("Control AM RLC PDU");

        nr::SequenceNumber10 ackSn = rlcAmHeader.GetAckSn();
        nr::SequenceNumber10 sn;

        NS_LOG_INFO("ackSn     = " << ackSn);
        NS_LOG_INFO("VT(A)     = " << m_vtA);
        NS_LOG_INFO("VT(S)     = " << m_vtS);
        NS_LOG_LOGIC("retxBufferSize = " << m_retxBufferSize);
        NS_LOG_LOGIC("txedBufferSize = " << m_txedBufferSize);

        m_vtA.SetModulusBase(m_vtA);
        m_vtS.SetModulusBase(m_vtA);
        m_vtMs.SetModulusBase(m_vtA);
        ackSn.SetModulusBase(m_vtA);
        sn.SetModulusBase(m_vtA);

        // A STATUS PDU is only meaningful if VT(A) <= ACK_SN <= VT(S) (TS 36.322
        // 5.2.1). A stale one -- duplicated or reordered by HARQ, or from the old
        // epoch after the entities were re-created at handover, aliasing in the
        // 10-bit SN space -- can carry an ACK_SN outside the live transmit window;
        // applying it would spuriously acknowledge (and release) PDUs the peer never
        // received. Discard it instead.
        if (!((m_vtA <= ackSn) && (ackSn <= m_vtS)))
        {
            NS_LOG_WARN("Discarding STATUS PDU with ACK_SN="
                        << ackSn << " outside the transmit window [" << m_vtA << ", " << m_vtS
                        << "]: stale, duplicated or reordered STATUS PDU");
            return;
        }

        bool incrementVtA = true;

        for (sn = m_vtA; sn < ackSn && sn < m_vtS; sn++)
        {
            NS_LOG_LOGIC("sn = " << sn);

            uint16_t seqNumberValue = sn.GetValue();

            if (m_pollRetransmitTimer.IsPending() && (seqNumberValue == m_pollSn.GetValue()))
            {
                m_pollRetransmitTimer.Cancel();
            }

            // A NACK for an SN that is held in neither buffer refers to a PDU that an
            // earlier STATUS PDU already positively acknowledged and released. It can
            // only come from a stale or reordered STATUS PDU: there is nothing left to
            // retransmit, so treat the SN as acknowledged (TS 36.322 5.2.1) instead of
            // asserting or freezing VT(A) on it forever.
            const bool isHeld =
                m_txedBuffer.at(seqNumberValue).m_pdu || m_retxBuffer.at(seqNumberValue).m_pdu;

            if (rlcAmHeader.IsNackPresent(sn) && !isHeld)
            {
                NS_LOG_WARN("NACK for SN=" << seqNumberValue
                                           << ", which was already acknowledged and released "
                                              "(stale or reordered STATUS PDU): treating the SN "
                                              "as acknowledged");
            }

            if (rlcAmHeader.IsNackPresent(sn) && isHeld)
            {
                NS_LOG_LOGIC("sn " << sn << " is NACKed");

                incrementVtA = false;

                if (m_txedBuffer.at(seqNumberValue).m_pdu)
                {
                    NS_LOG_INFO("Move SN = " << seqNumberValue << " to retxBuffer");
                    m_retxBuffer.at(seqNumberValue).m_pdu =
                        m_txedBuffer.at(seqNumberValue).m_pdu->Copy();
                    m_retxBuffer.at(seqNumberValue).m_retxCount =
                        m_txedBuffer.at(seqNumberValue).m_retxCount;
                    m_retxBuffer.at(seqNumberValue).m_waitingSince =
                        m_txedBuffer.at(seqNumberValue).m_waitingSince;
                    m_retxBufferSize += m_retxBuffer.at(seqNumberValue).m_pdu->GetSize();

                    m_txedBufferSize -= m_txedBuffer.at(seqNumberValue).m_pdu->GetSize();
                    m_txedBuffer.at(seqNumberValue).m_pdu = nullptr;
                    m_txedBuffer.at(seqNumberValue).m_retxCount = 0;
                    m_txedBuffer.at(seqNumberValue).m_waitingSince = MilliSeconds(0);
                }

                NS_ASSERT(m_retxBuffer.at(seqNumberValue).m_pdu);
            }
            else
            {
                NS_LOG_LOGIC("sn " << sn << " is ACKed");

                if (m_txedBuffer.at(seqNumberValue).m_pdu)
                {
                    NS_LOG_INFO("ACKed SN = " << seqNumberValue << " from txedBuffer");
                    //               NS_LOG_INFO ("m_txedBuffer( " << m_vtA << " )->GetSize = " <<
                    //               m_txedBuffer.at (m_vtA.GetValue ())->GetSize ());
                    m_txedBufferSize -= m_txedBuffer.at(seqNumberValue).m_pdu->GetSize();
                    m_txedBuffer.at(seqNumberValue).m_pdu = nullptr;
                    m_txedBuffer.at(seqNumberValue).m_waitingSince = MilliSeconds(0);
                    NS_ASSERT(!m_retxBuffer.at(seqNumberValue).m_pdu);
                }

                if (m_retxBuffer.at(seqNumberValue).m_pdu)
                {
                    NS_LOG_INFO("ACKed SN = " << seqNumberValue << " from retxBuffer");
                    m_retxBufferSize -= m_retxBuffer.at(seqNumberValue).m_pdu->GetSize();
                    m_retxBuffer.at(seqNumberValue).m_pdu = nullptr;
                    m_retxBuffer.at(seqNumberValue).m_retxCount = 0;
                    m_retxBuffer.at(seqNumberValue).m_waitingSince = MilliSeconds(0);
                }
            }

            NS_LOG_LOGIC("retxBufferSize = " << m_retxBufferSize);
            NS_LOG_LOGIC("txedBufferSize = " << m_txedBufferSize);

            if (incrementVtA)
            {
                m_vtA++;
                m_vtMs = m_vtA + m_windowSize;
                NS_LOG_INFO("New VT(A) = " << m_vtA);
                m_vtA.SetModulusBase(m_vtA);
                m_vtMs.SetModulusBase(m_vtA);
                m_vtS.SetModulusBase(m_vtA);
                ackSn.SetModulusBase(m_vtA);
                sn.SetModulusBase(m_vtA);
            }

        } // loop over SN : VT(A) <= SN < ACK SN

        return;
    }
    else
    {
        NS_LOG_WARN("Wrong AM RLC PDU type");
        return;
    }
}

bool
NrRlcAm::IsInsideReceivingWindow(nr::SequenceNumber10 seqNumber)
{
    NS_LOG_FUNCTION(this << seqNumber);
    NS_LOG_LOGIC("Receiving Window: " << m_vrR << " <= " << seqNumber << " <= " << m_vrMr);

    m_vrR.SetModulusBase(m_vrR);
    m_vrMr.SetModulusBase(m_vrR);
    seqNumber.SetModulusBase(m_vrR);

    if ((m_vrR <= seqNumber) && (seqNumber < m_vrMr))
    {
        NS_LOG_LOGIC(seqNumber << " is INSIDE the receiving window");
        return true;
    }
    else
    {
        NS_LOG_LOGIC(seqNumber << " is OUTSIDE the receiving window");
        return false;
    }
}

void
NrRlcAm::ReestablishRxSide()
{
    NS_LOG_FUNCTION(this << m_rnti << (uint32_t)m_lcid);

    // TS 38.322 5.1.2: on re-establishment the RLC entity shall discard all RLC
    // SDUs, RLC SDU segments and RLC PDUs, stop and reset all timers, and reset all
    // state variables to their initial values (receiving side).
    m_reorderingTimer.Cancel();
    m_statusProhibitTimer.Cancel();
    m_rxonBuffer.clear();
    m_keepS0 = nullptr;
    m_reassemblingState = WAITING_S0_FULL;
    // Assign fresh SequenceNumber10 objects rather than raw values:
    // operator=(uint16_t) keeps the old modulus base, and a stale base makes the
    // window comparisons against the next received SN abort (their operators
    // require equal bases) or misorder.
    m_expectedSeqNumber = nr::SequenceNumber10(0);
    m_vrR = nr::SequenceNumber10(0);
    m_vrMr = m_vrR + m_windowSize;
    m_vrX = nr::SequenceNumber10(0);
    m_vrMs = nr::SequenceNumber10(0);
    m_vrH = nr::SequenceNumber10(0);
    m_statusPduRequested = false;
    m_statusPduBufferSize = 0;
}

void
NrRlcAm::ReassembleAndDeliver(Ptr<Packet> packet)
{
    NrRlcAmHeader rlcAmHeader;
    NrRlcTag rlcTag;
    bool ret = packet->FindFirstMatchingByteTag(rlcTag);
    NS_ASSERT(ret);
    packet->RemoveHeader(rlcAmHeader);
    ret = packet->FindFirstMatchingByteTag(rlcTag);
    NS_ASSERT(!ret);
    uint8_t framingInfo = rlcAmHeader.GetFramingInfo();
    nr::SequenceNumber10 currSeqNumber = rlcAmHeader.GetSequenceNumber();
    bool expectedSnLost;

    if (currSeqNumber != m_expectedSeqNumber)
    {
        expectedSnLost = true;
        NS_LOG_LOGIC("There are losses. Expected SN = " << m_expectedSeqNumber
                                                        << ". Current SN = " << currSeqNumber);
        m_expectedSeqNumber = currSeqNumber + 1;
    }
    else
    {
        expectedSnLost = false;
        NS_LOG_LOGIC("No losses. Expected SN = " << m_expectedSeqNumber
                                                 << ". Current SN = " << currSeqNumber);
        m_expectedSeqNumber = m_expectedSeqNumber + 1;
    }

    // Build list of SDUs
    SplitDataFields(rlcAmHeader, packet, m_sdusBuffer);

    // Current reassembling state
    if (m_reassemblingState == WAITING_S0_FULL)
    {
        NS_LOG_LOGIC("Reassembling State = 'WAITING_S0_FULL'");
    }
    else if (m_reassemblingState == WAITING_SI_SF)
    {
        NS_LOG_LOGIC("Reassembling State = 'WAITING_SI_SF'");
    }
    else
    {
        NS_LOG_LOGIC("Reassembling State = Unknown state");
    }

    // Received framing Info
    NS_LOG_LOGIC("Framing Info = " << (uint16_t)framingInfo);
    NS_LOG_LOGIC("m_sdusBuffer = " << m_sdusBuffer.size());

    // The FI field is the transmitter's ground truth about SDU boundaries, while
    // expectedSnLost is inferred from 10-bit SN continuity, which aliases: after a
    // handover both RLC entities are re-created and an old-epoch PDU still in flight
    // can be accepted by the fresh entity and later delivered in sequence at an
    // aliased position. When the FI contradicts the reassembly state, the awaited
    // segment was lost even though the SN looks consecutive, so the stale partial
    // SDU is discarded rather than concatenated with unrelated data,
    // resynchronising on the next SDU boundary.
    const bool firstFieldContinuesSdu = ((framingInfo & NrRlcAmHeader::NO_FIRST_BYTE) != 0);
    const bool lastFieldIsComplete = ((framingInfo & NrRlcAmHeader::NO_LAST_BYTE) == 0);
    if (!expectedSnLost && firstFieldContinuesSdu != (m_reassemblingState == WAITING_SI_SF))
    {
        NS_LOG_WARN("SN=" << currSeqNumber
                          << " matches the expected SN but FI=" << (uint16_t)framingInfo
                          << " contradicts the reassembly state: resynchronising");
    }

    m_reassemblingState = ReassembleSdus(m_sdusBuffer,
                                         m_keepS0,
                                         firstFieldContinuesSdu,
                                         lastFieldIsComplete,
                                         !expectedSnLost)
                              ? WAITING_SI_SF
                              : WAITING_S0_FULL;
}

void
NrRlcAm::DoTransmitBufferStatusReport()
{
    NS_LOG_FUNCTION(this);

    Time now = Simulator::Now();

    NS_LOG_LOGIC("txonBufferSize = " << m_txonBufferSize);
    NS_LOG_LOGIC("retxBufferSize = " << m_retxBufferSize);
    NS_LOG_LOGIC("txedBufferSize = " << m_txedBufferSize);
    NS_LOG_LOGIC("VT(A) = " << m_vtA);
    NS_LOG_LOGIC("VT(S) = " << m_vtS);

    // Transmission Queue HOL time
    Time txonQueueHolDelay(0);
    if (m_txonBufferSize > 0)
    {
        txonQueueHolDelay = now - m_txonBuffer.front().m_waitingSince;
    }

    // Retransmission Queue HOL time
    Time retxQueueHolDelay;
    if (m_retxBufferSize > 0)
    {
        Time senderTimestamp;
        if (m_retxBuffer.at(m_vtA.GetValue()).m_pdu)
        {
            senderTimestamp = m_retxBuffer.at(m_vtA.GetValue()).m_waitingSince;
        }
        else
        {
            senderTimestamp = m_txedBuffer.at(m_vtA.GetValue()).m_waitingSince;
        }
        retxQueueHolDelay = now - senderTimestamp;
    }
    else
    {
        retxQueueHolDelay = Seconds(0);
    }

    NrMacSapProvider::BufferStatusReportParameters r{};
    r.rnti = m_rnti;
    r.lcid = m_lcid;
    r.txQueueSize = m_txonBufferSize;
    r.txQueueHolDelay = txonQueueHolDelay.GetMilliSeconds();
    r.retxQueueSize = m_retxBufferSize + m_txedBufferSize;
    r.retxQueueHolDelay = retxQueueHolDelay.GetMilliSeconds();

    if (m_statusPduRequested && !m_statusProhibitTimer.IsPending())
    {
        r.statusPduSize = m_statusPduBufferSize;
    }
    else
    {
        r.statusPduSize = 0;
    }

    if (r.txQueueSize != 0 || r.retxQueueSize != 0 || r.statusPduSize != 0)
    {
        NS_LOG_INFO("Send BufferStatusReport: " << r.txQueueSize << ", " << r.txQueueHolDelay
                                                << ", " << r.retxQueueSize << ", "
                                                << r.retxQueueHolDelay << ", " << r.statusPduSize);
        m_macSapProvider->BufferStatusReport(r);
    }
    else
    {
        NS_LOG_INFO("BufferStatusReport don't needed");
    }
}

void
NrRlcAm::ExpireReorderingTimer()
{
    NS_LOG_FUNCTION(this);
    NS_LOG_LOGIC("Reordering Timer has expired");

    // 5.1.3.2.4 Actions when t-Reordering expires
    // When t-Reordering expires, the receiving side of an AM RLC entity shall:
    // - update VR(MS) to the SN of the first AMD PDU with SN >= VR(X) for which not all byte
    // segments
    //   have been received;
    // - if VR(H) > VR(MS):
    //    - start t-Reordering;
    //    - set VR(X) to VR(H).

    m_vrMs = m_vrX;
    int firstVrMs = m_vrMs.GetValue();
    auto it = m_rxonBuffer.find(m_vrMs.GetValue());
    while (it != m_rxonBuffer.end() && it->second.m_pduComplete)
    {
        m_vrMs++;
        it = m_rxonBuffer.find(m_vrMs.GetValue());

        NS_ASSERT_MSG(firstVrMs != m_vrMs.GetValue(), "Infinite loop in ExpireReorderingTimer");
    }
    NS_LOG_LOGIC("New VR(MS) = " << m_vrMs);

    if (m_vrH > m_vrMs)
    {
        NS_LOG_LOGIC("Start reordering timer");
        m_reorderingTimer =
            Simulator::Schedule(m_reorderingTimerValue, &NrRlcAm::ExpireReorderingTimer, this);
        m_vrX = m_vrH;
        NS_LOG_LOGIC("New VR(MS) = " << m_vrMs);
    }

    // Section 5.2.3 Status Reporting:
    //   - The receiving side of an AM RLC entity shall trigger a
    //     STATUS report when T_reordering expires.
    // Advertise the pending STATUS to the MAC as the poll path does; without the
    // buffer-status report a receive-only bearer would wait for an unrelated
    // transmission opportunity to send it.
    m_statusPduRequested = true;
    m_statusPduBufferSize = 4;
    if (!m_statusProhibitTimer.IsPending())
    {
        DoTransmitBufferStatusReport();
    }
}

void
NrRlcAm::ExpirePollRetransmitTimer()
{
    NS_LOG_FUNCTION(this);
    NS_LOG_LOGIC("PollRetransmit Timer has expired");

    NS_LOG_LOGIC("txonBufferSize = " << m_txonBufferSize);
    NS_LOG_LOGIC("retxBufferSize = " << m_retxBufferSize);
    NS_LOG_LOGIC("txedBufferSize = " << m_txedBufferSize);
    NS_LOG_LOGIC("statusPduRequested = " << m_statusPduRequested);

    m_pollRetransmitTimerJustExpired = true;

    // see section 5.2.2.3
    // note the difference between Rel 8 and Rel 11 specs; we follow Rel 11 here
    NS_ASSERT(m_vtS <= m_vtMs);
    if ((m_txonBufferSize == 0 && m_retxBufferSize == 0) || (m_vtS == m_vtMs))
    {
        NS_LOG_INFO("txonBuffer and retxBuffer empty. Move PDUs up to = " << m_vtS.GetValue() - 1
                                                                          << " to retxBuffer");
        for (nr::SequenceNumber10 sn = m_vtA; sn < m_vtS; sn++)
        {
            bool pduAvailable = (bool)m_txedBuffer.at(sn.GetValue()).m_pdu;

            if (pduAvailable)
            {
                uint16_t snValue = sn.GetValue();
                NS_LOG_INFO("Move PDU " << sn << " from txedBuffer to retxBuffer");
                m_retxBuffer.at(snValue).m_pdu = m_txedBuffer.at(snValue).m_pdu->Copy();
                m_retxBuffer.at(snValue).m_retxCount = m_txedBuffer.at(snValue).m_retxCount;
                m_retxBuffer.at(snValue).m_waitingSince = m_txedBuffer.at(snValue).m_waitingSince;
                m_retxBufferSize += m_retxBuffer.at(snValue).m_pdu->GetSize();

                m_txedBufferSize -= m_txedBuffer.at(snValue).m_pdu->GetSize();
                m_txedBuffer.at(snValue).m_pdu = nullptr;
                m_txedBuffer.at(snValue).m_retxCount = 0;
                m_txedBuffer.at(snValue).m_waitingSince = MilliSeconds(0);
            }
        }
    }

    DoTransmitBufferStatusReport();
}

void
NrRlcAm::ExpireStatusProhibitTimer()
{
    NS_LOG_FUNCTION(this);
}

void
NrRlcAm::ExpireBsrTimer()
{
    NS_LOG_LOGIC("BSR Timer expires");

    if (m_txonBufferSize + m_txedBufferSize + m_retxBufferSize > 0)
    {
        DoTransmitBufferStatusReport();
        m_bsrTimer = Simulator::Schedule(m_bsrTimerValue, &NrRlcAm::ExpireBsrTimer, this);
    }
}

} // namespace ns3
