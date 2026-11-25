// Copyright (c) 2011 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only
//
// Author: Manuel Requena <manuel.requena@cttc.es>

#ifndef NR_RLC_UM_H
#define NR_RLC_UM_H

#include "nr-rlc-sequence-number.h"
#include "nr-rlc.h"

#include "ns3/event-id.h"
#include "ns3/nr-export.h"

#include <deque>
#include <map>

namespace ns3
{

/**
 * LTE RLC Unacknowledged Mode (UM), see 3GPP TS 36.322
 */
class NR_EXPORT NrRlcUm : public NrRlc
{
  public:
    NrRlcUm();
    ~NrRlcUm() override;
    /**
     * @brief Get the type ID.
     * @return the object TypeId
     */
    static TypeId GetTypeId();
    void DoDispose() override;

    /**
     * RLC SAP
     *
     * @param p packet
     */
    void DoTransmitPdcpPdu(Ptr<Packet> p) override;

    /**
     * MAC SAP
     *
     * @param txOpParams the NrMacSapUser::TxOpportunityParameters
     */
    void DoNotifyTxOpportunity(NrMacSapUser::TxOpportunityParameters txOpParams) override;
    void DoNotifyHarqDeliveryFailure() override;
    void DoReceivePdu(NrMacSapUser::ReceivePduParameters rxPduParams) override;

  private:
    /// Expire reordering timer
    void ExpireReorderingTimer();
    /// Expire BSR timer
    void ExpireBsrTimer();
    /**
     * @brief Restart the buffer status report timer, or stop it if there is nothing left to send.
     *
     * Called whenever the buffer status is reported to the MAC. While data is waiting the timer
     * keeps running, so its expiry triggers a new report (the equivalent of retxBSR-Timer of 3GPP
     * TS 38.321, clause 5.4.5, whose expiry triggers a regular buffer status report); once the
     * buffer is empty there is nothing to report and the timer is left stopped.
     */
    void RestartBsrTimer();

    /**
     * Is inside reordering window function
     *
     * @param seqNumber the sequence number
     * @returns true if inside the window
     */
    bool IsInsideReorderingWindow(nr::SequenceNumber10 seqNumber);

    void ReestablishRxSide() override;

    /// Reassemble outside window
    void ReassembleOutsideWindow();

    /**
     * Out-of-order delivery (rlc-OutOfOrderDelivery): deliver the complete SDUs of a PDU just
     * placed in the reception buffer, and the segmented SDUs it completes together with the
     * segments held from other PDUs, then leave a null entry (tombstone) in the reception buffer.
     *
     * The tombstone keeps the PDU counted as received for the state variables and the duplicate
     * detection of the reordering window, which then only advances over it.
     *
     * @param seqNumber the SN of the PDU just placed in the reception buffer
     */
    void DeliverOutOfOrder(nr::SequenceNumber10 seqNumber);

    /**
     * Out-of-order delivery: deliver the SDU segmented across the boundary between the PDUs
     * with SN = boundary - 1 and SN = boundary, if all its segments are held.
     *
     * @param boundary the SN of the PDU whose first data field continues the SDU
     */
    void DeliverSegmentedSdu(uint16_t boundary);

    /**
     * Out-of-order delivery: discard the held segments that can no longer be completed, i.e.
     * whose neighbouring PDU was received without the matching segment, or was given up by the
     * reordering window (behind VR(UR) without having been received).
     */
    void DiscardIncompleteSegments();
    /**
     * Reassemble SN interval function
     *
     * @param lowSeqNumber the low sequence number
     * @param highSeqNumber the high sequence number
     */
    void ReassembleSnInterval(nr::SequenceNumber10 lowSeqNumber,
                              nr::SequenceNumber10 highSeqNumber);

    /**
     * Reassemble and deliver function
     *
     * @param packet the packet
     */
    void ReassembleAndDeliver(Ptr<Packet> packet);

    /// Buffer status report
    void DoTransmitBufferStatusReport();

  private:
    friend class NrRlcUmTestCase; ///< Grant NrRlcUmTestCase access to private members for
                                  ///<  RLC UM RX verification
    /// Grant the t-Reordering discard regression test access to private reassembly state
    friend class NrRlcUmReorderingDiscardTestCase;
    /// Grant the PDCP discard regression test access to private transmit buffer state
    friend class NrRlcUmTxPdcpDiscardTestCase;
    /// Grant the ReassembleSnInterval gap regression test access to private reception state
    friend class NrRlcUmReassembleGapTestCase;
    /// Grant the reassembly state machine matrix test access to private reassembly state
    friend class NrRlcUmReassemblyMatrixTestCase;
    /// Grant the SN-alias resynchronisation regression test access to private reassembly state
    friend class NrRlcUmSnAliasResyncTestCase;
    /// Grant the Tx buffer overflow test access to private transmit buffer state
    friend class NrRlcUmTxBufferOverflowTestCase;
    /// Grant the Tx opportunity edge-case test access to private transmit buffer state
    friend class NrRlcUmTxOpportunityEdgeTestCase;
    /// Grant the t-Reordering timer test access to private timer and window state
    friend class NrRlcUmReorderingTimerTestCase;
    /// Grant the out-of-order delivery test access to private reception buffer state
    friend class NrRlcUmOutOfOrderDeliveryTestCase;
    /// Grant the out-of-order data-integrity test access to the held segments
    friend class NrRlcUmOutOfOrderDataIntegrityTestCase;
    /// Grant the severely-delayed-PDU test access to private window state
    friend class NrRlcUmLatePduTestCase;
    uint32_t m_maxTxBufferSize; ///< maximum transmit buffer status
    uint32_t m_txBufferSize;    ///< transmit buffer size

    /**
     * @brief Store an incoming (from layer above us) PDU, waiting to transmit it
     */
    struct NR_EXPORT TxPdu
    {
        /**
         * @brief TxPdu default constructor
         * @param pdu the PDU
         * @param time the arrival time
         */
        TxPdu(const Ptr<Packet>& pdu, const Time& time)
            : m_pdu(pdu),
              m_waitingSince(time)
        {
        }

        TxPdu() = delete;

        Ptr<Packet> m_pdu;   ///< PDU
        Time m_waitingSince; ///< Layer arrival time
    };

    std::deque<TxPdu> m_txBuffer;               ///< Transmission buffer
    std::map<uint16_t, Ptr<Packet>> m_rxBuffer; ///< Reception buffer

    std::list<Ptr<Packet>> m_sdusBuffer; ///< List of SDUs in a packet

    /// Segments of SDUs split across PDUs, held by out-of-order delivery until complete
    struct OutOfOrderSegments
    {
        Ptr<Packet> m_head; ///< First data field, continuing the SDU of the previous SN, or null
        Ptr<Packet> m_tail; ///< Last data field, continued by the next SN, or null (the same
                            ///< packet as m_head for a PDU carrying only a middle segment)
    };

    std::map<uint16_t, OutOfOrderSegments> m_outOfOrderSegments; ///< Held segments, by SN

    /**
     * State variables. See section 7.1 in TS 36.322
     */
    nr::SequenceNumber10 m_sequenceNumber; ///< VT(US)

    nr::SequenceNumber10 m_vrUr; ///< VR(UR)
    nr::SequenceNumber10 m_vrUx; ///< VR(UX)
    nr::SequenceNumber10 m_vrUh; ///< VR(UH)

    /**
     * Constants. See section 7.2 in TS 36.322
     */
    uint16_t m_windowSize; ///< windows size

    /**
     * Timers. See section 7.3 in TS 36.322
     */
    Time m_reorderingTimerValue;        ///< reordering timer value
    EventId m_reorderingTimer;          ///< reordering timer
    EventId m_bsrTimer;                 ///< BSR timer
    Time m_bsrTimerValue;               ///< BSR timer value
    bool m_enablePdcpDiscarding{false}; //!< whether to use the PDCP discarding (perform discarding
                                        //!< at the moment of passing the PDCP SDU to RLC)
    uint32_t m_discardTimerMs{0};       //!< the discard timer value in milliseconds
    bool m_outOfOrderDelivery{false};   //!< whether to deliver RLC SDUs without reordering timer

    /**
     * Reassembling state
     */
    enum ReassemblingState_t
    {
        NONE = 0,
        WAITING_S0_FULL = 1,
        WAITING_SI_SF = 2
    };

    ReassemblingState_t m_reassemblingState; ///< reassembling state
    Ptr<Packet> m_keepS0;                    ///< keep S0

    /**
     * Expected Sequence Number
     */
    nr::SequenceNumber10 m_expectedSeqNumber;

    /**
     * Newest sender timestamp among the received PDUs. Used to tell a genuinely new
     * out-of-window PDU (which slides the reordering window forward) from a PDU
     * delayed by more than UM_Window_Size SNs that aliases to the same position:
     * the newest transmission always carries the newest sender timestamp.
     */
    Time m_maxRxSenderTimestamp;

    bool m_expBsrTimer{false};
};

} // namespace ns3

#endif // NR_RLC_UM_H
