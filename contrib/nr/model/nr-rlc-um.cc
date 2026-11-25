// Copyright (c) 2011 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only
//
// Author: Manuel Requena <manuel.requena@cttc.es>

#include "nr-rlc-um.h"

#include "nr-rlc-header.h"
#include "nr-rlc-sdu-status-tag.h"
#include "nr-rlc-tag.h"

#include "ns3/abort.h"
#include "ns3/log.h"
#include "ns3/simulator.h"

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("NrRlcUm");

NS_OBJECT_ENSURE_REGISTERED(NrRlcUm);

NrRlcUm::NrRlcUm()
    : m_maxTxBufferSize(10 * 1024),
      m_txBufferSize(0),
      m_sequenceNumber(0),
      m_vrUr(0),
      m_vrUx(0),
      m_vrUh(0),
      m_windowSize(512),
      m_expectedSeqNumber(0)
{
    NS_LOG_FUNCTION(this);
    m_reassemblingState = WAITING_S0_FULL;
}

NrRlcUm::~NrRlcUm()
{
    NS_LOG_FUNCTION(this);
}

TypeId
NrRlcUm::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::NrRlcUm")
            .SetParent<NrRlc>()
            .SetGroupName("Nr")
            .AddConstructor<NrRlcUm>()
            .AddAttribute("MaxTxBufferSize",
                          "Maximum Size of the Transmission Buffer (in Bytes). "
                          "If set to 0, the buffer is unlimited.",
                          UintegerValue(10 * 1024),
                          MakeUintegerAccessor(&NrRlcUm::m_maxTxBufferSize),
                          MakeUintegerChecker<uint32_t>())
            .AddAttribute("ReorderingTimer",
                          "Value of the t-Reordering timer (See section 7.3 of 3GPP TS 36.322)",
                          TimeValue(MilliSeconds(100)),
                          MakeTimeAccessor(&NrRlcUm::m_reorderingTimerValue),
                          MakeTimeChecker())
            .AddAttribute(
                "EnablePdcpDiscarding",
                "Whether to use the PDCP discarding, i.e., perform discarding at the moment "
                "of passing the PDCP SDU to RLC)",
                BooleanValue(true),
                MakeBooleanAccessor(&NrRlcUm::m_enablePdcpDiscarding),
                MakeBooleanChecker())
            .AddAttribute("DiscardTimerMs",
                          "Discard timer in milliseconds to be used to discard packets. "
                          "If set to 0 then packet delay budget will be used as the discard "
                          "timer value, otherwise it will be used this value.",
                          UintegerValue(0),
                          MakeUintegerAccessor(&NrRlcUm::m_discardTimerMs),
                          MakeUintegerChecker<uint32_t>())
            .AddAttribute("BufferStatusReportTimer",
                          "How much to wait to issue a new Buffer Status Report while the "
                          "transmission buffer is not empty",
                          TimeValue(MilliSeconds(10)),
                          MakeTimeAccessor(&NrRlcUm::m_bsrTimerValue),
                          MakeTimeChecker())
            .AddAttribute("OutOfOrderDelivery",
                          "Whether to deliver RLC SDUs out of order without waiting for a "
                          "reordering timer to expire",
                          BooleanValue(false),
                          MakeBooleanAccessor(&NrRlcUm::m_outOfOrderDelivery),
                          MakeBooleanChecker());
    return tid;
}

void
NrRlcUm::DoDispose()
{
    NS_LOG_FUNCTION(this);
    m_reorderingTimer.Cancel();
    m_bsrTimer.Cancel();

    NrRlc::DoDispose();
}

/**
 * RLC SAP
 */

void
NrRlcUm::DoTransmitPdcpPdu(Ptr<Packet> p)
{
    NS_LOG_FUNCTION(this << m_rnti << (uint32_t)m_lcid << p->GetSize());
    if ((m_txBufferSize + p->GetSize() <= m_maxTxBufferSize) || (m_maxTxBufferSize == 0))
    {
        if (m_enablePdcpDiscarding)
        {
            // discard the packet
            uint32_t headOfLineDelayInMs = 0;
            uint32_t discardTimerMs =
                (m_discardTimerMs > 0) ? m_discardTimerMs : m_packetDelayBudgetMs;

            if (!m_txBuffer.empty())
            {
                headOfLineDelayInMs =
                    (Simulator::Now() - m_txBuffer.begin()->m_waitingSince).GetMilliSeconds();
            }
            NS_LOG_DEBUG("head of line delay in MS:" << headOfLineDelayInMs);
            if (headOfLineDelayInMs > discardTimerMs)
            {
                NS_LOG_INFO("Tx HOL is higher than this packet can allow. RLC SDU discarded");
                NS_LOG_DEBUG("headOfLineDelayInMs    = " << headOfLineDelayInMs);
                NS_LOG_DEBUG("m_packetDelayBudgetMs    = " << m_packetDelayBudgetMs);
                NS_LOG_DEBUG("packet size     = " << p->GetSize());
                m_txDropTrace(p);

                // The packet has been discarded, so do not store it in the Tx buffer.
                // A Buffer Status Report is still issued below to reflect the current state.
                // A running report timer is left alone: new data arriving is what triggers a
                // buffer status report, never a reason to stop reporting.
                DoTransmitBufferStatusReport();
                return;
            }
        }

        /** Store PDCP PDU */
        NrRlcSduStatusTag tag;
        tag.SetStatus(NrRlcSduStatusTag::FULL_SDU);
        p->AddPacketTag(tag);
        NS_LOG_INFO("Adding RLC SDU to Tx Buffer after adding NrRlcSduStatusTag: FULL_SDU");
        m_txBuffer.emplace_back(p, Simulator::Now());
        m_txBufferSize += p->GetSize();
        NS_LOG_LOGIC("NumOfBuffers = " << m_txBuffer.size());
        NS_LOG_LOGIC("txBufferSize = " << m_txBufferSize);
    }
    else
    {
        // Discard full RLC SDU
        NS_LOG_INFO("Tx Buffer is full. RLC SDU discarded");
        NS_LOG_LOGIC("MaxTxBufferSize = " << m_maxTxBufferSize);
        NS_LOG_LOGIC("txBufferSize    = " << m_txBufferSize);
        NS_LOG_LOGIC("packet size     = " << p->GetSize());
        m_txDropTrace(p);
    }

    /** Transmit Buffer Status Report */
    // A running report timer is left alone: new data arriving is what triggers a buffer status
    // report (3GPP TS 38.321, clause 5.4.5), never a reason to stop reporting. Cancelling it here
    // used to strand the uplink, as the timer is only armed again by a transmission opportunity:
    // once the uplink stalled, the last cancellation was final and the buffer status went stale.
    DoTransmitBufferStatusReport();
}

/**
 * MAC SAP
 */

void
NrRlcUm::DoNotifyTxOpportunity(NrMacSapUser::TxOpportunityParameters txOpParams)
{
    NS_LOG_FUNCTION(this << m_rnti << (uint32_t)m_lcid << txOpParams.bytes);
    NS_LOG_INFO("RLC layer is preparing data for the following Tx opportunity of "
                << txOpParams.bytes << " bytes for RNTI=" << m_rnti << ", LCID=" << (uint32_t)m_lcid
                << ", CCID=" << (uint32_t)txOpParams.componentCarrierId << ", HARQ ID="
                << (uint32_t)txOpParams.harqId << ", MIMO Layer=" << (uint32_t)txOpParams.layer);

    if (txOpParams.bytes <= 2)
    {
        // Stingy MAC: Header fix part is 2 bytes, we need more bytes for the data
        NS_LOG_INFO("TX opportunity too small - Only " << txOpParams.bytes << " bytes");
        return;
    }

    Ptr<Packet> packet = Create<Packet>();
    NrRlcHeader rlcHeader;

    // Build Data field
    uint32_t nextSegmentSize = txOpParams.bytes - 2;
    uint32_t nextSegmentId = 1;
    uint32_t dataFieldAddedSize = 0;
    std::vector<Ptr<Packet>> dataField;

    // Remove the first packet from the transmission buffer.
    // If only a segment of the packet is taken, then the remaining is given back later
    if (m_txBuffer.empty())
    {
        NS_LOG_LOGIC("No data pending");
        return;
    }

    Ptr<Packet> firstSegment = m_txBuffer.begin()->m_pdu->Copy();
    Time firstSegmentTime = m_txBuffer.begin()->m_waitingSince;

    NS_LOG_LOGIC("SDUs in TxBuffer  = " << m_txBuffer.size());
    NS_LOG_LOGIC("First SDU buffer  = " << firstSegment);
    NS_LOG_LOGIC("First SDU size    = " << firstSegment->GetSize());
    NS_LOG_LOGIC("Next segment size = " << nextSegmentSize);
    NS_LOG_LOGIC("Remove SDU from TxBuffer");
    m_txBufferSize -= firstSegment->GetSize();
    NS_LOG_LOGIC("txBufferSize      = " << m_txBufferSize);
    m_txBuffer.erase(m_txBuffer.begin());

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

                m_txBuffer.insert(m_txBuffer.begin(), TxPdu(firstSegment, firstSegmentTime));
                m_txBufferSize += m_txBuffer.begin()->m_pdu->GetSize();

                NS_LOG_LOGIC("    TX buffer: Give back the remaining segment");
                NS_LOG_LOGIC("    TX buffers = " << m_txBuffer.size());
                NS_LOG_LOGIC("    Front buffer size = " << m_txBuffer.begin()->m_pdu->GetSize());
                NS_LOG_LOGIC("    txBufferSize = " << m_txBufferSize);
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
            rlcHeader.PushExtensionBit(NrRlcHeader::DATA_FIELD_FOLLOWS);

            // no LengthIndicator for the last one

            nextSegmentSize -= dataFieldAddedSize;
            nextSegmentId++;

            // nextSegmentSize MUST be zero (only if segment is smaller or equal to 2047)

            // (NO more segments) → exit
            // break;
        }
        else if ((nextSegmentSize - firstSegment->GetSize() <= 2) || m_txBuffer.empty())
        {
            NS_LOG_LOGIC(
                "    IF nextSegmentSize - firstSegment->GetSize () <= 2 || txBuffer.size == 0");
            // Add txBuffer.FirstBuffer to DataField
            dataFieldAddedSize = firstSegment->GetSize();
            dataField.push_back(firstSegment);
            firstSegment = nullptr;

            // ExtensionBit (Next_Segment - 1) = 0
            rlcHeader.PushExtensionBit(NrRlcHeader::DATA_FIELD_FOLLOWS);

            // no LengthIndicator for the last one

            nextSegmentSize -= dataFieldAddedSize;
            nextSegmentId++;

            NS_LOG_LOGIC("        SDUs in TxBuffer  = " << m_txBuffer.size());
            if (!m_txBuffer.empty())
            {
                NS_LOG_LOGIC("        First SDU buffer  = " << m_txBuffer.begin()->m_pdu);
                NS_LOG_LOGIC(
                    "        First SDU size    = " << m_txBuffer.begin()->m_pdu->GetSize());
            }
            NS_LOG_LOGIC("        Next segment size = " << nextSegmentSize);

            // nextSegmentSize <= 2 (only if txBuffer is not empty)

            // (NO more segments) → exit
            // break;
        }
        else // (firstSegment->GetSize () < m_nextSegmentSize) && (m_txBuffer.size () > 0)
        {
            NS_LOG_LOGIC("    IF firstSegment < NextSegmentSize && txBuffer.size > 0");
            // Add txBuffer.FirstBuffer to DataField
            dataFieldAddedSize = firstSegment->GetSize();
            dataField.push_back(firstSegment);

            // ExtensionBit (Next_Segment - 1) = 1
            rlcHeader.PushExtensionBit(NrRlcHeader::E_LI_FIELDS_FOLLOWS);

            // LengthIndicator (Next_Segment)  = txBuffer.FirstBuffer.length()
            rlcHeader.PushLengthIndicator(firstSegment->GetSize());

            nextSegmentSize -= ((nextSegmentId % 2) ? (2) : (1)) + dataFieldAddedSize;
            nextSegmentId++;

            NS_LOG_LOGIC("        SDUs in TxBuffer  = " << m_txBuffer.size());
            if (!m_txBuffer.empty())
            {
                NS_LOG_LOGIC("        First SDU buffer  = " << m_txBuffer.begin()->m_pdu);
                NS_LOG_LOGIC(
                    "        First SDU size    = " << m_txBuffer.begin()->m_pdu->GetSize());
            }
            NS_LOG_LOGIC("        Next segment size = " << nextSegmentSize);
            NS_LOG_LOGIC("        Remove SDU from TxBuffer");

            // (more segments)
            firstSegment = m_txBuffer.begin()->m_pdu->Copy();
            firstSegmentTime = m_txBuffer.begin()->m_waitingSince;
            m_txBufferSize -= firstSegment->GetSize();
            m_txBuffer.pop_front();
            NS_LOG_LOGIC("        txBufferSize = " << m_txBufferSize);
        }
    }

    // Build RLC header
    rlcHeader.SetSequenceNumber(m_sequenceNumber++);

    // Build RLC PDU with DataField and Header
    auto it = dataField.begin();

    uint8_t framingInfo = 0;

    // FIRST SEGMENT
    NrRlcSduStatusTag tag;
    NS_ASSERT_MSG((*it)->PeekPacketTag(tag), "NrRlcSduStatusTag is missing");
    (*it)->PeekPacketTag(tag);
    if ((tag.GetStatus() == NrRlcSduStatusTag::FULL_SDU) ||
        (tag.GetStatus() == NrRlcSduStatusTag::FIRST_SEGMENT))
    {
        framingInfo |= NrRlcHeader::FIRST_BYTE;
    }
    else
    {
        framingInfo |= NrRlcHeader::NO_FIRST_BYTE;
    }

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
        framingInfo |= NrRlcHeader::LAST_BYTE;
    }
    else
    {
        framingInfo |= NrRlcHeader::NO_LAST_BYTE;
    }

    rlcHeader.SetFramingInfo(framingInfo);

    NS_LOG_LOGIC("RLC header: " << rlcHeader);
    packet->AddHeader(rlcHeader);

    // Sender timestamp and transmitting entity identity
    NrRlcTag rlcTag(Simulator::Now());
    rlcTag.SetTxEntityId(m_rlcEntityId);
    packet->AddByteTag(rlcTag, 1, rlcHeader.GetSerializedSize());
    m_txPdu(m_rnti, m_lcid, packet->GetSize());

    // Send RLC PDU to MAC layer
    NrMacSapProvider::TransmitPduParameters params;
    params.pdu = packet;
    params.rnti = m_rnti;
    params.lcid = m_lcid;
    params.layer = txOpParams.layer;
    params.harqProcessId = txOpParams.harqId;
    params.componentCarrierId = txOpParams.componentCarrierId;

    NS_LOG_INFO("Forward RLC PDU to MAC Layer");
    m_macSapProvider->TransmitPdu(params);

    // The reported queue only accounts for one header per SDU, while every segment takes one, so
    // the MAC's count of the queue can reach zero with data left here. Report what is left, so
    // that the MAC keeps serving it instead of waiting for the BSR timer.
    if (!m_txBuffer.empty())
    {
        DoTransmitBufferStatusReport();
    }

    RestartBsrTimer();
}

void
NrRlcUm::DoNotifyHarqDeliveryFailure()
{
    NS_LOG_FUNCTION(this);
}

void
NrRlcUm::DoReceivePdu(NrMacSapUser::ReceivePduParameters rxPduParams)
{
    NS_LOG_FUNCTION(this << m_rnti << (uint32_t)m_lcid << rxPduParams.p->GetSize());

    // Receiver timestamp
    NrRlcTag rlcTag;
    Time delay;

    bool ret = rxPduParams.p->FindFirstMatchingByteTag(rlcTag);
    NS_ASSERT_MSG(ret, "NrRlcTag is missing");

    delay = Simulator::Now() - rlcTag.GetSenderTimestamp();
    m_rxPdu(m_rnti, m_lcid, rxPduParams.p->GetSize(), delay.GetNanoSeconds());

    // Old-epoch PDUs of a peer entity destroyed at handover alias into the new SN
    // space and would corrupt reassembly undetectably: discard them.
    if (!AcceptPduFromPeerEntity(rlcTag.GetTxEntityId()))
    {
        return;
    }

    // 5.1.2.2 Receive operations

    // Get RLC header parameters
    NrRlcHeader rlcHeader;
    rxPduParams.p->PeekHeader(rlcHeader);
    NS_LOG_LOGIC("RLC header: " << rlcHeader);
    nr::SequenceNumber10 seqNumber = rlcHeader.GetSequenceNumber();

    // 5.1.2.2.1 General
    // The receiving UM RLC entity shall maintain a reordering window according to state variable
    // VR(UH) as follows:
    // - a SN falls within the reordering window if (VR(UH) - UM_Window_Size) <= SN < VR(UH);
    // - a SN falls outside of the reordering window otherwise.
    // When receiving an UMD PDU from lower layer, the receiving UM RLC entity shall:
    // - either discard the received UMD PDU or place it in the reception buffer (see sub
    // clause 5.1.2.2.2);
    // - if the received UMD PDU was placed in the reception buffer:
    // - update state variables, reassemble and deliver RLC SDUs to upper layer and start/stop
    // t-Reordering as needed (see sub clause 5.1.2.2.3); When t-Reordering expires, the receiving
    // UM RLC entity shall:
    // - update state variables, reassemble and deliver RLC SDUs to upper layer and start
    // t-Reordering as needed (see sub clause 5.1.2.2.4).

    // A PDU whose SN falls outside the reordering window is normally the newest
    // transmission and slides the window forward (5.1.2.2.3). But a PDU delayed by
    // more than UM_Window_Size SNs (e.g. a last-round HARQ retransmission at high
    // rate) aliases to the same out-of-window position, and sliding the window back
    // onto it would discard live in-window traffic wholesale. The two cases are
    // indistinguishable in the 10-bit SN space, but the newest transmission always
    // carries the newest sender timestamp: discard an out-of-window PDU that was
    // sent before traffic already received.
    if (!IsInsideReorderingWindow(seqNumber) &&
        (rlcTag.GetSenderTimestamp() < m_maxRxSenderTimestamp))
    {
        NS_LOG_WARN("Discarding PDU with SN="
                    << seqNumber << " outside the reordering window but sent ("
                    << rlcTag.GetSenderTimestamp().As(Time::MS) << ") before already-received "
                    << "traffic (" << m_maxRxSenderTimestamp.As(Time::MS)
                    << "): severely delayed duplicate, not new traffic");
        return;
    }
    if (rlcTag.GetSenderTimestamp() > m_maxRxSenderTimestamp)
    {
        m_maxRxSenderTimestamp = rlcTag.GetSenderTimestamp();
    }

    // 5.1.2.2.2 Actions when an UMD PDU is received from lower layer
    // When an UMD PDU with SN = x is received from lower layer, the receiving UM RLC entity shall:
    // - if VR(UR) < x < VR(UH) and the UMD PDU with SN = x has been received before; or
    // - if (VR(UH) - UM_Window_Size) <= x < VR(UR):
    //    - discard the received UMD PDU;
    // - else:
    //    - place the received UMD PDU in the reception buffer.

    NS_LOG_LOGIC("VR(UR) = " << m_vrUr);
    NS_LOG_LOGIC("VR(UX) = " << m_vrUx);
    NS_LOG_LOGIC("VR(UH) = " << m_vrUh);
    NS_LOG_LOGIC("SN = " << seqNumber);

    m_vrUr.SetModulusBase(m_vrUh - m_windowSize);
    m_vrUh.SetModulusBase(m_vrUh - m_windowSize);
    seqNumber.SetModulusBase(m_vrUh - m_windowSize);

    if (((m_vrUr < seqNumber) && (seqNumber < m_vrUh) &&
         (m_rxBuffer.count(seqNumber.GetValue()) > 0)) ||
        (((m_vrUh - m_windowSize) <= seqNumber) && (seqNumber < m_vrUr)))
    {
        NS_LOG_LOGIC("PDU discarded");
        rxPduParams.p = nullptr;
        return;
    }
    else
    {
        NS_ASSERT_MSG(m_rxBuffer.count(seqNumber.GetValue()) == 0,
                      "INTERNAL ERROR: PDU with SN=" << seqNumber.GetValue()
                                                     << " already exists in the RX buffer.");

        NS_LOG_LOGIC("Place PDU in the reception buffer");
        m_rxBuffer[seqNumber.GetValue()] = rxPduParams.p;
    }

    // 5.1.2.2.3 Actions when an UMD PDU is placed in the reception buffer
    // When an UMD PDU with SN = x is placed in the reception buffer, the receiving UM RLC entity
    // shall:
    // - if rlc-OutOfOrderDelivery is configured:
    //   - if all byte segments of the UMD PDU are received:
    //   - reassemble the RLC SDU using the byte segments of the UMD PDU,
    //     remove RLC headers when doing so and deliver the reassembled RLC
    //     SDU to upper layer if not delivered before;

    if (m_outOfOrderDelivery)
    {
        DeliverOutOfOrder(seqNumber);
    }

    // - if x falls outside of the reordering window:
    //    - update VR(UH) to x + 1;
    //    - reassemble RLC SDUs from any UMD PDUs with SN that falls outside of the reordering
    //    window, remove
    //      RLC headers when doing so and deliver the reassembled RLC SDUs to upper layer in
    //      ascending order of the RLC SN if not delivered before;
    //    - if VR(UR) falls outside of the reordering window:
    //        - set VR(UR) to (VR(UH) - UM_Window_Size);

    if (!IsInsideReorderingWindow(seqNumber))
    {
        NS_LOG_LOGIC("SN is outside the reordering window");

        m_vrUh = seqNumber + 1;
        NS_LOG_LOGIC("New VR(UH) = " << m_vrUh);

        ReassembleOutsideWindow();

        if (!IsInsideReorderingWindow(m_vrUr))
        {
            m_vrUr = m_vrUh - m_windowSize;
            NS_LOG_LOGIC("VR(UR) is outside the reordering window");
            NS_LOG_LOGIC("New VR(UR) = " << m_vrUr);
        }
    }

    // - if the reception buffer contains an UMD PDU with SN = VR(UR):
    //    - update VR(UR) to the SN of the first UMD PDU with SN > current VR(UR) that has not been
    //    received;
    //    - reassemble RLC SDUs from any UMD PDUs with SN < updated VR(UR), remove RLC headers when
    //    doing
    //      so and deliver the reassembled RLC SDUs to upper layer in ascending order of the RLC SN
    //      if not delivered before;

    if (m_rxBuffer.count(m_vrUr.GetValue()) > 0)
    {
        NS_LOG_LOGIC("Reception buffer contains SN = " << m_vrUr);

        nr::SequenceNumber10 oldVrUr = m_vrUr;

        // Advance with a SequenceNumber10 so the scan wraps at the 10-bit boundary:
        // a raw counter would stop at 1024 (never a buffer key) and strand buffered
        // or tombstoned entries at SN 0 while VR(UR) wraps onto them.
        nr::SequenceNumber10 newVrUr = m_vrUr + 1;
        while (m_rxBuffer.count(newVrUr.GetValue()) > 0)
        {
            newVrUr++;
        }
        m_vrUr = newVrUr;
        NS_LOG_LOGIC("New VR(UR) = " << m_vrUr);

        ReassembleSnInterval(oldVrUr, m_vrUr);
    }

    // m_vrUh can change previously, set new modulus base
    // for the t-Reordering timer-related comparisons
    m_vrUr.SetModulusBase(m_vrUh - m_windowSize);
    m_vrUx.SetModulusBase(m_vrUh - m_windowSize);
    m_vrUh.SetModulusBase(m_vrUh - m_windowSize);

    // - if t-Reordering is running:
    //    - if VR(UX) <= VR(UR); or
    //    - if VR(UX) falls outside of the reordering window and VR(UX) is not equal to VR(UH)::
    //        - stop and reset t-Reordering;
    if (m_reorderingTimer.IsPending())
    {
        NS_LOG_LOGIC("Reordering timer is running");

        if ((m_vrUx <= m_vrUr) || ((!IsInsideReorderingWindow(m_vrUx)) && (m_vrUx != m_vrUh)))
        {
            NS_LOG_LOGIC("Stop reordering timer");
            m_reorderingTimer.Cancel();
        }
    }

    // - if t-Reordering is not running (includes the case when t-Reordering is stopped due to
    // actions above):
    //    - if VR(UH) > VR(UR):
    //        - start t-Reordering;
    //        - set VR(UX) to VR(UH).
    if (!m_reorderingTimer.IsPending())
    {
        NS_LOG_LOGIC("Reordering timer is not running");

        if (m_vrUh > m_vrUr)
        {
            NS_LOG_LOGIC("VR(UH) > VR(UR)");
            NS_LOG_LOGIC("Start reordering timer");
            m_reorderingTimer =
                Simulator::Schedule(m_reorderingTimerValue, &NrRlcUm::ExpireReorderingTimer, this);
            m_vrUx = m_vrUh;
            NS_LOG_LOGIC("New VR(UX) = " << m_vrUx);
        }
    }

    if (m_outOfOrderDelivery)
    {
        DiscardIncompleteSegments();
    }
}

bool
NrRlcUm::IsInsideReorderingWindow(nr::SequenceNumber10 seqNumber)
{
    NS_LOG_FUNCTION(this << seqNumber);
    NS_LOG_LOGIC("Reordering Window: " << m_vrUh << " - " << m_windowSize << " <= " << seqNumber
                                       << " < " << m_vrUh);

    m_vrUh.SetModulusBase(m_vrUh - m_windowSize);
    seqNumber.SetModulusBase(m_vrUh - m_windowSize);

    if (((m_vrUh - m_windowSize) <= seqNumber) && (seqNumber < m_vrUh))
    {
        NS_LOG_LOGIC(seqNumber << " is INSIDE the reordering window");
        return true;
    }
    else
    {
        NS_LOG_LOGIC(seqNumber << " is OUTSIDE the reordering window");
        return false;
    }
}

void
NrRlcUm::ReassembleAndDeliver(Ptr<Packet> packet)
{
    NrRlcHeader rlcHeader;
    packet->RemoveHeader(rlcHeader);
    uint8_t framingInfo = rlcHeader.GetFramingInfo();
    nr::SequenceNumber10 currSeqNumber = rlcHeader.GetSequenceNumber();
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
        m_expectedSeqNumber++;
    }

    // Build list of SDUs
    SplitDataFields(rlcHeader, packet, m_sdusBuffer);

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

    // The FI field is the transmitter's ground truth about SDU boundaries, while
    // expectedSnLost is inferred from 10-bit SN continuity, which aliases: after a
    // handover both RLC entities are re-created and an old-epoch PDU still in flight
    // can reach the new entity at a position where SN == m_expectedSeqNumber, and
    // >= 1023 consecutive losses alias the same way. When the FI contradicts the
    // reassembly state, the awaited segment was lost even though the SN looks
    // consecutive, so the stale partial SDU is discarded rather than concatenated
    // with unrelated data, resynchronising on the next SDU boundary.
    const bool firstFieldContinuesSdu = ((framingInfo & NrRlcHeader::NO_FIRST_BYTE) != 0);
    const bool lastFieldIsComplete = ((framingInfo & NrRlcHeader::NO_LAST_BYTE) == 0);
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
NrRlcUm::ReestablishRxSide()
{
    NS_LOG_FUNCTION(this << m_rnti << (uint32_t)m_lcid);

    // TS 38.322 5.1.2: on re-establishment the RLC entity shall discard all RLC
    // SDUs, RLC SDU segments and RLC PDUs, stop and reset all timers, and reset all
    // state variables to their initial values.
    m_reorderingTimer.Cancel();
    m_rxBuffer.clear();
    m_outOfOrderSegments.clear();
    m_keepS0 = nullptr;
    m_reassemblingState = WAITING_S0_FULL;
    // Assign fresh SequenceNumber10 objects rather than raw values:
    // operator=(uint16_t) keeps the old modulus base (see the AM twin).
    m_expectedSeqNumber = nr::SequenceNumber10(0);
    m_vrUr = nr::SequenceNumber10(0);
    m_vrUx = nr::SequenceNumber10(0);
    m_vrUh = nr::SequenceNumber10(0);
    m_maxRxSenderTimestamp = Time();
}

void
NrRlcUm::ReassembleOutsideWindow()
{
    NS_LOG_LOGIC("Reassemble Outside Window");

    auto it = m_rxBuffer.lower_bound(m_vrUr.GetValue()); // First SN such that SN >= VR(UR)

    for (size_t remaining = m_rxBuffer.size(); remaining > 0; remaining--)
    {
        if (it == m_rxBuffer.end())
        {
            // wrap-around manual
            it = m_rxBuffer.begin();
        }

        if (IsInsideReorderingWindow(nr::SequenceNumber10(it->first)))
        {
            break;
        }

        NS_LOG_LOGIC("SN = " << it->first);

        // Reassemble RLC SDUs and deliver the PDCP PDU to upper layer. A null entry
        // is a tombstone for a PDU already delivered out of order: consume it
        // without delivering again.
        if (it->second)
        {
            ReassembleAndDeliver(it->second);
        }

        auto it_tmp = it;
        ++it;
        m_rxBuffer.erase(it_tmp);
    }

    if (it != m_rxBuffer.end())
    {
        NS_LOG_LOGIC("(SN = " << it->first << ") is inside the reordering window");
    }
}

void
NrRlcUm::DeliverOutOfOrder(nr::SequenceNumber10 seqNumber)
{
    NS_LOG_FUNCTION(this << seqNumber);

    const uint16_t sn = seqNumber.GetValue();
    Ptr<Packet> packet = m_rxBuffer.at(sn);
    m_rxBuffer[sn] = nullptr;

    NrRlcHeader rlcHeader;
    packet->RemoveHeader(rlcHeader);
    const uint8_t framingInfo = rlcHeader.GetFramingInfo();
    std::list<Ptr<Packet>> dataFields;
    SplitDataFields(rlcHeader, packet, dataFields);

    // Unlike the in-order reassembly, whose state machine needs the PDUs in SN order, the
    // segments at the edges of the PDU are held by SN, so that an SDU is reassembled whatever
    // the order its PDUs arrive in (HARQ retransmissions overtake each other all the time).
    OutOfOrderSegments segments;
    if ((framingInfo & NrRlcHeader::NO_FIRST_BYTE) != 0)
    {
        segments.m_head = dataFields.front();
        dataFields.pop_front();
    }
    if ((framingInfo & NrRlcHeader::NO_LAST_BYTE) != 0)
    {
        if (dataFields.empty())
        {
            segments.m_tail = segments.m_head;
        }
        else
        {
            segments.m_tail = dataFields.back();
            dataFields.pop_back();
        }
    }

    for (const auto& sdu : dataFields)
    {
        m_rlcSapUser->ReceivePdcpPdu(sdu);
    }

    if (!segments.m_head && !segments.m_tail)
    {
        return;
    }
    m_outOfOrderSegments[sn] = segments;
    if (segments.m_head)
    {
        DeliverSegmentedSdu(sn);
    }
    auto it = m_outOfOrderSegments.find(sn);
    if (it != m_outOfOrderSegments.end() && it->second.m_tail)
    {
        DeliverSegmentedSdu((seqNumber + 1).GetValue());
    }
}

void
NrRlcUm::DeliverSegmentedSdu(uint16_t boundary)
{
    NS_LOG_FUNCTION(this << boundary);

    // The SNs carrying the segments of the SDU, from the PDU whose last data field starts it to
    // the PDU whose first data field ends it; the PDUs in between carry a middle segment only.
    std::deque<uint16_t> chain;
    nr::SequenceNumber10 sn = nr::SequenceNumber10(boundary) - 1;
    for (uint16_t n = 0;; n++)
    {
        auto it = m_outOfOrderSegments.find(sn.GetValue());
        if (it == m_outOfOrderSegments.end() || !it->second.m_tail || n >= m_windowSize)
        {
            return;
        }
        chain.push_front(sn.GetValue());
        if (it->second.m_head != it->second.m_tail)
        {
            break;
        }
        sn = sn - 1;
    }
    sn = nr::SequenceNumber10(boundary);
    for (uint16_t n = 0;; n++)
    {
        auto it = m_outOfOrderSegments.find(sn.GetValue());
        if (it == m_outOfOrderSegments.end() || !it->second.m_head || n >= m_windowSize)
        {
            return;
        }
        chain.push_back(sn.GetValue());
        if (it->second.m_head != it->second.m_tail)
        {
            break;
        }
        sn++;
    }

    Ptr<Packet> sdu = m_outOfOrderSegments.at(chain.front()).m_tail->Copy();
    m_outOfOrderSegments.at(chain.front()).m_tail = nullptr;
    for (size_t i = 1; i < chain.size(); i++)
    {
        OutOfOrderSegments& segments = m_outOfOrderSegments.at(chain.at(i));
        sdu->AddAtEnd(segments.m_head);
        if (segments.m_tail == segments.m_head)
        {
            segments.m_tail = nullptr;
        }
        segments.m_head = nullptr;
    }
    for (uint16_t chainSn : chain)
    {
        const OutOfOrderSegments& segments = m_outOfOrderSegments.at(chainSn);
        if (!segments.m_head && !segments.m_tail)
        {
            m_outOfOrderSegments.erase(chainSn);
        }
    }

    NS_LOG_LOGIC("Delivering an SDU reassembled from SNs " << chain.front() << " to "
                                                           << chain.back());
    m_rlcSapUser->ReceivePdcpPdu(sdu);
}

void
NrRlcUm::DiscardIncompleteSegments()
{
    NS_LOG_FUNCTION(this);

    // Whether a PDU that was not received can no longer arrive: it is behind VR(UR) inside the
    // reordering window. Outside of the window, an SN is ahead of VR(UH) and may still arrive.
    auto givenUp = [this](uint16_t value) {
        nr::SequenceNumber10 neighbour(value);
        if (!IsInsideReorderingWindow(neighbour))
        {
            return false;
        }
        neighbour.SetModulusBase(m_vrUh - m_windowSize);
        m_vrUr.SetModulusBase(m_vrUh - m_windowSize);
        return neighbour < m_vrUr;
    };
    // Whether the neighbouring SN can still provide the matching segment: it is held, or it was
    // neither received (its segments would be held) nor given up.
    auto canProvide = [this, &givenUp](uint16_t value, bool head) {
        auto it = m_outOfOrderSegments.find(value);
        if (it != m_outOfOrderSegments.end())
        {
            return (head ? it->second.m_head : it->second.m_tail) != nullptr;
        }
        return m_rxBuffer.count(value) == 0 && !givenUp(value);
    };

    // Discarding a middle segment breaks the SDU on both sides, so repeat until stable.
    bool changed = true;
    while (changed)
    {
        changed = false;
        for (auto it = m_outOfOrderSegments.begin(); it != m_outOfOrderSegments.end();)
        {
            const nr::SequenceNumber10 sn(it->first);
            OutOfOrderSegments& segments = it->second;
            const bool middle = segments.m_head && segments.m_head == segments.m_tail;
            // Every received PDU is behind VR(UH): a held SN outside the window is stale.
            const bool stale = !IsInsideReorderingWindow(sn);
            bool dropHead = segments.m_head && (stale || !canProvide((sn - 1).GetValue(), false));
            bool dropTail = segments.m_tail && (stale || !canProvide((sn + 1).GetValue(), true));
            if (middle && (dropHead || dropTail))
            {
                dropHead = true;
                dropTail = true;
            }
            if (dropHead || dropTail)
            {
                NS_LOG_LOGIC("Discarding the incomplete SDU segments held for SN = " << sn);
                changed = true;
                if (dropHead)
                {
                    segments.m_head = nullptr;
                }
                if (dropTail)
                {
                    segments.m_tail = nullptr;
                }
            }
            if (!segments.m_head && !segments.m_tail)
            {
                it = m_outOfOrderSegments.erase(it);
            }
            else
            {
                ++it;
            }
        }
    }
}

void
NrRlcUm::ReassembleSnInterval(nr::SequenceNumber10 lowSeqNumber, nr::SequenceNumber10 highSeqNumber)
{
    NS_LOG_LOGIC("Reassemble SN between " << lowSeqNumber << " and " << highSeqNumber);

    nr::SequenceNumber10 reassembleSn = lowSeqNumber;
    NS_LOG_LOGIC("reassembleSN = " << reassembleSn);
    NS_LOG_LOGIC("highSeqNumber = " << highSeqNumber);
    while (reassembleSn < highSeqNumber)
    {
        NS_LOG_LOGIC("reassembleSn < highSeqNumber");
        auto it = m_rxBuffer.find(reassembleSn.GetValue());
        if (it != m_rxBuffer.end())
        {
            NS_LOG_LOGIC("it->first  = " << it->first);
            NS_LOG_LOGIC("it->second = " << it->second);
            NS_LOG_LOGIC("SN = " << it->first);

            // Reassemble RLC SDUs and deliver the PDCP PDU to upper layer. A null
            // entry is a tombstone for a PDU already delivered out of order:
            // consume it without delivering again.
            if (it->second)
            {
                ReassembleAndDeliver(it->second);
            }

            m_rxBuffer.erase(it);
        }

        reassembleSn++;
    }
}

void
NrRlcUm::DoTransmitBufferStatusReport()
{
    Time holDelay(0);
    uint32_t queueSize = 0;

    if (!m_txBuffer.empty())
    {
        holDelay = Simulator::Now() - m_txBuffer.front().m_waitingSince;

        queueSize =
            m_txBufferSize + 2 * m_txBuffer.size(); // Data in tx queue + estimated headers size
    }

    NrMacSapProvider::BufferStatusReportParameters r;
    r.rnti = m_rnti;
    r.lcid = m_lcid;
    r.txQueueSize = queueSize;
    r.txQueueHolDelay = holDelay.GetMilliSeconds();
    r.retxQueueSize = 0;
    r.retxQueueHolDelay = 0;
    r.statusPduSize = 0;
    r.expBsrTimer = m_expBsrTimer;

    m_expBsrTimer = false;

    NS_LOG_LOGIC("Send BufferStatusReport = " << r.txQueueSize << ", " << r.txQueueHolDelay);
    m_macSapProvider->BufferStatusReport(r);
}

void
NrRlcUm::ExpireReorderingTimer()
{
    NS_LOG_FUNCTION(this << m_rnti << (uint32_t)m_lcid);
    NS_LOG_LOGIC("Reordering timer has expired");

    // 5.1.2.2.4 Actions when t-Reordering expires
    // When t-Reordering expires, the receiving UM RLC entity shall:
    // - update VR(UR) to the SN of the first UMD PDU with SN >= VR(UX) that has not been received;
    // - reassemble RLC SDUs from any UMD PDUs with SN < updated VR(UR), remove RLC headers when
    // doing so
    //   and deliver the reassembled RLC SDUs to upper layer in ascending order of the RLC SN if not
    //   delivered before;
    // - if VR(UH) > VR(UR):
    //    - start t-Reordering;
    //    - set VR(UX) to VR(UH).

    nr::SequenceNumber10 newVrUr = m_vrUx;

    while (m_rxBuffer.find(newVrUr.GetValue()) != m_rxBuffer.end())
    {
        newVrUr++;
    }
    nr::SequenceNumber10 oldVrUr = m_vrUr;
    m_vrUr = newVrUr;
    NS_LOG_LOGIC("New VR(UR) = " << m_vrUr);

    ReassembleSnInterval(oldVrUr, m_vrUr);

    // t-Reordering expiry means the missing segment(s) at the front of the reordering
    // window are given up on. Per TS 38.322 (5.1.2.2.4 / 5.2.2.2), any RLC SDU that can
    // no longer be fully reassembled must be discarded. If a partially-reassembled SDU is
    // still held (WAITING_SI_SF, i.e. S0 is kept waiting for its continuation) after the
    // above in-window reassembly, its continuation was among the lost SNs and will never
    // arrive: drop it and resynchronise. Leaving it held lets it survive until the 10-bit
    // SN wraps (~1024 PDUs), after which an unrelated later segment aliases as the awaited
    // continuation and is concatenated onto the stale head, delivering a corrupt (non-IP)
    // SDU to upper layers (observed as the "Unknown IP type" abort during fast handovers).
    if (m_reassemblingState == WAITING_SI_SF)
    {
        NS_LOG_LOGIC("Discarding partially reassembled SDU stranded by t-Reordering expiry");
        m_keepS0 = nullptr;
        m_reassemblingState = WAITING_S0_FULL;
    }

    if (m_vrUh > m_vrUr)
    {
        NS_LOG_LOGIC("Start reordering timer");
        m_reorderingTimer =
            Simulator::Schedule(m_reorderingTimerValue, &NrRlcUm::ExpireReorderingTimer, this);
        m_vrUx = m_vrUh;
        NS_LOG_LOGIC("New VR(UX) = " << m_vrUx);
    }

    if (m_outOfOrderDelivery)
    {
        DiscardIncompleteSegments();
    }
}

void
NrRlcUm::ExpireBsrTimer()
{
    NS_LOG_LOGIC("BSR Timer expires");

    if (!m_txBuffer.empty())
    {
        m_expBsrTimer = true;
        DoTransmitBufferStatusReport();
        RestartBsrTimer();
    }
}

void
NrRlcUm::RestartBsrTimer()
{
    NS_LOG_FUNCTION(this);

    m_bsrTimer.Cancel();

    if (!m_txBuffer.empty())
    {
        m_bsrTimer = Simulator::Schedule(m_bsrTimerValue, &NrRlcUm::ExpireBsrTimer, this);
    }
}

} // namespace ns3
