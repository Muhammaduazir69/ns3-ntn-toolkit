// Copyright (c) 2026 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only

/**
 * @ingroup test
 * @file nr-test-rlc-am-status.cc
 *
 * @brief Test suite `nr-test-rlc-am-status`: white-box regression tests of the NrRlcAm
 * robustness against stale, duplicated or reordered inputs. STATUS PDUs have no sequence
 * protection: HARQ processes can deliver them out of order, and after a handover both RLC
 * entities are re-created, so old-epoch PDUs can reach a fresh entity at an aliased 10-bit
 * SN. The cases verify that a NACK for an SN that was already positively acknowledged and
 * released is treated as acknowledged (instead of asserting or freezing VT(A)); that a
 * STATUS PDU whose ACK_SN falls outside the transmit window is discarded without touching
 * the buffers; that a valid STATUS PDU still moves NACKed SNs to the retransmission buffer
 * and advances VT(A); and that a data PDU whose framing info contradicts the reassembly
 * state resynchronises the reassembler (discarding the stale partial SDU) instead of
 * leaving segments queued in m_sdusBuffer where they would corrupt every subsequently
 * delivered SDU. A data-integrity case additionally drives a transmitting entity
 * through segmentation and concatenation and verifies byte-exactly that the receiving
 * entity delivers the transmitted SDUs in order, under lossless, reordered and
 * ARQ-retransmission conditions.
 */

#include "nr-rlc-test-utils.h"
#include "nr-test-entities.h"

#include "ns3/log.h"
#include "ns3/nr-rlc-am-header.h"
#include "ns3/nr-rlc-am.h"
#include "ns3/nr-rlc-sap.h"
#include "ns3/nr-rlc-tag.h"
#include "ns3/packet.h"
#include "ns3/simulator.h"
#include "ns3/uinteger.h"

#include <string>
#include <utility>
#include <vector>

namespace ns3
{

namespace
{

/**
 * Build an AMD PDU carrying the given data-field segments, with an RLC tag covering
 * its header.
 *
 * @param sn the RLC sequence number
 * @param framingInfo the FI field
 * @param segments payload segments (fill byte and size per segment)
 * @param requestPoll whether to set the polling bit
 * @return the AMD PDU
 */
Ptr<Packet>
MakeAmdPdu(uint16_t sn,
           uint8_t framingInfo,
           const std::vector<NrRlcSduSpec>& segments,
           bool requestPoll)
{
    Ptr<Packet> pdu = Create<Packet>();
    for (const auto& [fill, size] : segments)
    {
        Ptr<Packet> segment = MakeUniformSdu(fill, size);
        if (pdu->GetSize() > 0)
        {
            pdu->AddAtEnd(segment);
        }
        else
        {
            pdu = segment;
        }
    }

    NrRlcAmHeader header;
    header.SetDataPdu();
    header.SetSequenceNumber(nr::SequenceNumber10(sn));
    header.SetResegmentationFlag(NrRlcAmHeader::PDU);
    header.SetLastSegmentFlag(NrRlcAmHeader::LAST_PDU_SEGMENT);
    header.SetSegmentOffset(0);
    header.SetFramingInfo(framingInfo);
    header.SetPollingBit(requestPoll ? NrRlcAmHeader::STATUS_REPORT_IS_REQUESTED
                                     : NrRlcAmHeader::STATUS_REPORT_NOT_REQUESTED);
    for (uint32_t i = 0; i + 1 < segments.size(); i++)
    {
        header.PushExtensionBit(NrRlcAmHeader::E_LI_FIELDS_FOLLOWS);
        header.PushLengthIndicator(segments[i].second);
    }
    header.PushExtensionBit(NrRlcAmHeader::DATA_FIELD_FOLLOWS);
    pdu->AddHeader(header);

    NrRlcTag rlcTag(Simulator::Now());
    pdu->AddByteTag(rlcTag, 1, header.GetSerializedSize());

    return pdu;
}

/**
 * Deliver one full-SDU AMD PDU to the RLC AM entity through the MAC SAP.
 *
 * @param rlc the receiving RLC entity
 * @param sn the RLC sequence number
 * @param size the SDU payload size in bytes
 * @param requestPoll whether to set the polling bit
 */
void
DeliverAmDataPdu(Ptr<NrRlcAm> rlc, uint16_t sn, uint32_t size, bool requestPoll)
{
    const uint8_t fi00 = NrRlcAmHeader::FIRST_BYTE | NrRlcAmHeader::LAST_BYTE;
    DeliverPduToRlc(rlc, MakeAmdPdu(sn, fi00, {{0, size}}, requestPoll));
}

/**
 * Deliver a STATUS PDU to the RLC AM entity through the MAC SAP.
 *
 * @param rlc the RLC entity under test
 * @param ackSn the ACK_SN value
 * @param nacks the NACK_SN values
 */
void
DeliverStatusPdu(Ptr<NrRlcAm> rlc, uint16_t ackSn, const std::vector<int>& nacks)
{
    NrRlcAmHeader header;
    header.SetControlPdu(NrRlcAmHeader::STATUS_PDU);
    header.SetAckSn(nr::SequenceNumber10(ackSn));
    for (int nack : nacks)
    {
        header.PushNack(nack);
    }

    Ptr<Packet> status = Create<Packet>();
    status->AddHeader(header);

    NrRlcTag rlcTag(Simulator::Now());
    status->AddByteTag(rlcTag, 1, header.GetSerializedSize());

    DeliverPduToRlc(rlc, status);
}

} // namespace

/**
 * @ingroup tests
 *
 * @brief Regression test for NrRlcAm STATUS PDU robustness.
 *
 * Drives a transmitting AM entity to emit SNs 0..2, then feeds it STATUS PDUs that
 * are reordered or stale and verifies the entity neither aborts nor corrupts its
 * transmit window bookkeeping.
 */
class NrRlcAmStaleStatusTestCase : public NrRlcTestCaseBase
{
  public:
    NrRlcAmStaleStatusTestCase()
        : NrRlcTestCaseBase("Test RLC AM TX: stale and reordered STATUS PDUs are handled safely")
    {
    }

  private:
    void DoRun() override;
};

void
NrRlcAmStaleStatusTestCase::DoRun()
{
    Ptr<NrRlcAm> rlc = CreateObject<NrRlcAm>();

    Ptr<NrTestMac> mac = CreateObject<NrTestMac>();
    mac->SetRlcHeaderType(NrTestMac::AM_RLC_HEADER);
    rlc->SetNrMacSapProvider(mac->GetNrMacSapProvider());
    mac->SetNrMacSapUser(rlc->GetNrMacSapUser());

    // Emit three AMD PDUs (SN=0, 1, 2): interleave SDU arrivals and transmission
    // opportunities so each opportunity carries exactly one SDU.
    TransmitSdusAligned(rlc, {{'A', 20}, {'B', 20}, {'C', 20}}, 100);

    NS_TEST_ASSERT_MSG_EQ(rlc->m_vtA.GetValue(), 0, "precondition: VT(A) = 0");
    NS_TEST_ASSERT_MSG_EQ(rlc->m_vtS.GetValue(), 3, "precondition: VT(S) = 3");
    for (uint16_t sn = 0; sn < 3; sn++)
    {
        NS_TEST_ASSERT_MSG_EQ((rlc->m_txedBuffer.at(sn).m_pdu != nullptr),
                              true,
                              "precondition: SN=" << sn << " must be in the txed buffer");
    }

    // A stale STATUS PDU (old epoch or corrupted) whose ACK_SN falls outside the
    // transmit window [VT(A), VT(S)] must be discarded without touching anything.
    DeliverStatusPdu(rlc, 700, {});
    NS_TEST_ASSERT_MSG_EQ(rlc->m_vtA.GetValue(),
                          0,
                          "an out-of-window ACK_SN must not advance VT(A)");
    for (uint16_t sn = 0; sn < 3; sn++)
    {
        NS_TEST_ASSERT_MSG_EQ((rlc->m_txedBuffer.at(sn).m_pdu != nullptr),
                              true,
                              "an out-of-window ACK_SN must not release SN=" << sn);
    }

    // Reordering scenario: the NEWER STATUS PDU (ACK_SN=3, NACK 0) arrives first.
    // SN=0 moves to the retransmission buffer; SN=1 and SN=2 are positively
    // acknowledged and released; VT(A) stays at the NACKed SN=0.
    DeliverStatusPdu(rlc, 3, {0});
    NS_TEST_ASSERT_MSG_EQ((rlc->m_retxBuffer.at(0).m_pdu != nullptr),
                          true,
                          "the NACKed SN=0 must move to the retx buffer");
    NS_TEST_ASSERT_MSG_EQ((rlc->m_txedBuffer.at(1).m_pdu == nullptr),
                          true,
                          "the ACKed SN=1 must be released from the txed buffer");
    NS_TEST_ASSERT_MSG_EQ((rlc->m_txedBuffer.at(2).m_pdu == nullptr),
                          true,
                          "the ACKed SN=2 must be released from the txed buffer");
    NS_TEST_ASSERT_MSG_EQ(rlc->m_vtA.GetValue(), 0, "VT(A) must hold at the NACKed SN");

    // The OLDER STATUS PDU (ACK_SN=3, NACK 0, NACK 1) arrives late: its NACK for
    // SN=1 refers to a PDU already acknowledged and released. Before the fix this
    // hit NS_ASSERT(m_retxBuffer.at(sn).m_pdu) and aborted the simulation; it must
    // instead be treated as acknowledged.
    DeliverStatusPdu(rlc, 3, {0, 1});
    NS_TEST_ASSERT_MSG_EQ((rlc->m_retxBuffer.at(1).m_pdu == nullptr),
                          true,
                          "a stale NACK for a released SN must not resurrect it");
    NS_TEST_ASSERT_MSG_EQ(rlc->m_vtA.GetValue(),
                          0,
                          "VT(A) must still hold at the genuinely NACKed SN=0");

    // A subsequent valid STATUS PDU acknowledging everything releases the retx
    // PDU and advances VT(A) to VT(S): normal operation is unaffected.
    DeliverStatusPdu(rlc, 3, {});
    NS_TEST_ASSERT_MSG_EQ((rlc->m_retxBuffer.at(0).m_pdu == nullptr),
                          true,
                          "the final ACK must release the retx PDU");
    NS_TEST_ASSERT_MSG_EQ(rlc->m_vtA.GetValue(), 3, "the final ACK must advance VT(A) to VT(S)");
    NS_TEST_ASSERT_MSG_EQ(rlc->m_txedBufferSize, 0, "txed buffer size must drain to zero");
    NS_TEST_ASSERT_MSG_EQ(rlc->m_retxBufferSize, 0, "retx buffer size must drain to zero");

    Simulator::Destroy();
}

/**
 * @ingroup tests
 *
 * @brief Regression test for the NrRlcAm reassembly resynchronisation.
 *
 * A data PDU delivered in sequence (SN == expected SN, e.g. an old-epoch PDU
 * aliasing into a fresh entity after handover) whose framing info contradicts the
 * reassembly state used to fall through the "Transition not possible" defaults
 * WITHOUT consuming m_sdusBuffer: the stale segments stayed queued and corrupted
 * every subsequently delivered SDU (silent PDCP/RRC corruption). The reassembler
 * must instead resynchronise on an SDU boundary, discard un-completable partial
 * SDUs, salvage complete SDUs, and deliver later PDUs byte-exact.
 */
class NrRlcAmReassemblyResyncTestCase : public NrRlcTestCaseBase
{
  public:
    NrRlcAmReassemblyResyncTestCase()
        : NrRlcTestCaseBase("Test RLC AM RX: FI-state contradiction resynchronises the reassembler")
    {
    }

  private:
    void DoRun() override;
};

void
NrRlcAmReassemblyResyncTestCase::DoRun()
{
    const uint8_t fi00 = NrRlcAmHeader::FIRST_BYTE | NrRlcAmHeader::LAST_BYTE;
    const uint8_t fi01 = NrRlcAmHeader::FIRST_BYTE | NrRlcAmHeader::NO_LAST_BYTE;
    const uint8_t fi10 = NrRlcAmHeader::NO_FIRST_BYTE | NrRlcAmHeader::LAST_BYTE;

    // Feed a PDU straight to the reassembler, bypassing the reception window.
    auto reassemble = [](Ptr<NrRlcAm> rlc,
                         uint16_t sn,
                         uint8_t framingInfo,
                         const std::vector<NrRlcSduSpec>& segments) {
        rlc->ReassembleAndDeliver(MakeAmdPdu(sn, framingInfo, segments, false));
    };

    // Case 1: mid-SDU (WAITING_SI_SF, S0 held) and an in-sequence PDU that STARTS a
    // new SDU (old-epoch alias). The stale head must be discarded and the new SDU
    // delivered intact -- never concatenated with the held head.
    {
        Ptr<NrRlcAm> rlc = CreateObject<NrRlcAm>();
        NrRlcTestSduSink sink;
        rlc->SetNrRlcSapUser(sink.GetSapUser());

        reassemble(rlc, 0, fi01, {{'S', 40}}); // head of an SDU, continuation never arrives
        NS_TEST_ASSERT_MSG_EQ((int)rlc->m_reassemblingState,
                              (int)NrRlcAm::WAITING_SI_SF,
                              "precondition: reassembler holds a partial SDU");

        reassemble(rlc, 1, fi00, {{'A', 21}}); // in-sequence, but starts a new SDU

        NS_TEST_ASSERT_MSG_EQ(sink.m_sdus.size(), 1, "exactly one SDU must be delivered");
        NS_TEST_ASSERT_MSG_EQ(IsUniformSdu(sink.m_sdus.at(0), 'A', 21),
                              true,
                              "the new SDU must be delivered intact, not concatenated with the "
                              "stale head");
        NS_TEST_ASSERT_MSG_EQ((int)rlc->m_reassemblingState,
                              (int)NrRlcAm::WAITING_S0_FULL,
                              "the reassembler must resynchronise to an SDU boundary");
        NS_TEST_ASSERT_MSG_EQ((rlc->m_keepS0 == nullptr),
                              true,
                              "the stale partial SDU must be discarded");
        NS_TEST_ASSERT_MSG_EQ(rlc->m_sdusBuffer.size(),
                              0,
                              "no segments may remain queued in m_sdusBuffer");
    }

    // Case 2: at an SDU boundary (WAITING_S0_FULL) and an in-sequence PDU that
    // CONTINUES a previous SDU (orphan continuation, e.g. old-epoch tail). Before
    // the fix this fell through without consuming m_sdusBuffer, so the next
    // delivered SDU was corrupted by the stale queued segments. The orphan leading
    // segment must be discarded, the complete trailing SDU salvaged, and the next
    // PDU delivered byte-exact.
    {
        Ptr<NrRlcAm> rlc = CreateObject<NrRlcAm>();
        NrRlcTestSduSink sink;
        rlc->SetNrRlcSapUser(sink.GetSapUser());

        reassemble(rlc, 0, fi00, {{'A', 21}});            // normal full SDU
        reassemble(rlc, 1, fi10, {{'X', 22}, {'B', 23}}); // orphan tail 'X' + full SDU 'B'
        reassemble(rlc, 2, fi00, {{'C', 24}});            // normal full SDU, must arrive clean

        NS_TEST_ASSERT_MSG_EQ(sink.m_sdus.size(), 3, "exactly three SDUs must be delivered");
        NS_TEST_ASSERT_MSG_EQ(IsUniformSdu(sink.m_sdus.at(0), 'A', 21),
                              true,
                              "the first SDU must be delivered intact");
        NS_TEST_ASSERT_MSG_EQ(IsUniformSdu(sink.m_sdus.at(1), 'B', 23),
                              true,
                              "the complete SDU after the orphan tail must be salvaged intact");
        NS_TEST_ASSERT_MSG_EQ(IsUniformSdu(sink.m_sdus.at(2), 'C', 24),
                              true,
                              "the SDU following the contradiction must not be corrupted by "
                              "stale queued segments");
        NS_TEST_ASSERT_MSG_EQ(rlc->m_sdusBuffer.size(),
                              0,
                              "no segments may remain queued in m_sdusBuffer");
    }

    Simulator::Destroy();
}

/**
 * @ingroup tests
 *
 * @brief End-to-end data-integrity test of the RLC AM transmit and receive entities.
 *
 * A transmitting NrRlcAm segments and concatenates SDUs with distinct, identifiable
 * payloads into AMD PDUs (small transmission opportunities force both). The captured
 * PDUs are delivered to fresh receiving entities under three conditions -- lossless
 * in order, with two PDUs swapped (HARQ reordering), and with one PDU withheld and
 * delivered last (an ARQ retransmission closing the gap) -- verifying byte-exactly
 * that every SDU is delivered equal to what was transmitted, in order, and that
 * nothing is delivered past an open gap (AM delivers strictly in sequence).
 */
class NrRlcAmDataIntegrityTestCase : public NrRlcTestCaseBase
{
  public:
    NrRlcAmDataIntegrityTestCase()
        : NrRlcTestCaseBase("Test RLC AM: transmitted data is received byte-exact under "
                            "segmentation, reordering and retransmission")
    {
    }

  private:
    void DoRun() override;
};

void
NrRlcAmDataIntegrityTestCase::DoRun()
{
    const std::vector<NrRlcSduSpec> txSdus =
        {{'A', 45}, {'B', 52}, {'C', 59}, {'D', 66}, {'E', 73}, {'F', 80}};

    // Transmit side: queue the SDUs and grant small opportunities so the AMD PDUs
    // both segment large SDUs and concatenate small remainders.
    Ptr<NrRlcAm> txRlc = CreateObject<NrRlcAm>();
    NrRlcTestCaptureMac txCapture;
    txRlc->SetNrMacSapProvider(&txCapture);
    QueueSdus(txRlc, txSdus);
    GrantTxOpportunities(txRlc, 44, 20);
    const std::vector<Ptr<Packet>>& pdus = txCapture.m_pdus;

    CheckSegmentationAndConcatenation<NrRlcAmHeader>(pdus);

    // Scenario 1: all PDUs delivered in order -- every SDU arrives byte-exact.
    {
        Ptr<NrRlcAm> rxRlc = CreateObject<NrRlcAm>();
        NrRlcTestCaptureMac rxCapture;
        rxRlc->SetNrMacSapProvider(&rxCapture);
        NrRlcTestSduSink sink;
        rxRlc->SetNrRlcSapUser(sink.GetSapUser());
        for (const auto& pdu : pdus)
        {
            DeliverPduToRlc(rxRlc, pdu);
        }
        VerifyDelivered(sink, txSdus, "lossless in-order");
    }

    // Scenario 2: two adjacent PDUs swapped (HARQ processes deliver out of order) --
    // the receiving window restores SN order and every SDU arrives byte-exact.
    {
        Ptr<NrRlcAm> rxRlc = CreateObject<NrRlcAm>();
        NrRlcTestCaptureMac rxCapture;
        rxRlc->SetNrMacSapProvider(&rxCapture);
        NrRlcTestSduSink sink;
        rxRlc->SetNrRlcSapUser(sink.GetSapUser());
        std::vector<Ptr<Packet>> reordered = pdus;
        std::swap(reordered.at(2), reordered.at(3));
        for (const auto& pdu : reordered)
        {
            DeliverPduToRlc(rxRlc, pdu);
        }
        VerifyDelivered(sink, txSdus, "reordered");
    }

    // Scenario 3: one mid-stream PDU is withheld and delivered last, like an ARQ
    // retransmission closing the gap. AM delivers strictly in sequence: while the
    // gap is open, no SDU at or beyond it may be delivered (and never partially);
    // once the retransmission arrives, everything must be delivered in order and
    // byte-exact.
    {
        const uint32_t retxIndex = pdus.size() / 2;

        Ptr<NrRlcAm> rxRlc = CreateObject<NrRlcAm>();
        NrRlcTestCaptureMac rxCapture;
        rxRlc->SetNrMacSapProvider(&rxCapture);
        NrRlcTestSduSink sink;
        rxRlc->SetNrRlcSapUser(sink.GetSapUser());

        for (uint32_t i = 0; i < pdus.size(); i++)
        {
            if (i == retxIndex)
            {
                continue;
            }
            DeliverPduToRlc(rxRlc, pdus.at(i));
        }

        // With the gap open, every delivered SDU must still be byte-exact (a prefix
        // of the transmitted sequence: nothing at or beyond the gap).
        NS_TEST_ASSERT_MSG_EQ((sink.m_sdus.size() < txSdus.size()),
                              true,
                              "with an open gap, not all SDUs may be delivered");
        for (uint32_t i = 0; i < sink.m_sdus.size(); i++)
        {
            NS_TEST_ASSERT_MSG_EQ(
                IsUniformSdu(sink.m_sdus.at(i), txSdus[i].first, txSdus[i].second),
                true,
                "SDUs delivered before the gap must be byte-exact and in order");
        }

        // The "retransmission" closes the gap: everything is delivered in order.
        DeliverPduToRlc(rxRlc, pdus.at(retxIndex));
        VerifyDelivered(sink, txSdus, "retransmission closes the gap");
    }

    // Scenario 4: SN epoch mixing after a handover, with real transmitters on both
    // sides. An OLD-epoch transmitter (created FIRST, so it carries the older
    // entity identity) emits three SDU-aligned PDUs; its SN=2 PDU is still in
    // flight when the entities are re-created and reaches the fresh receiver
    // before the NEW-epoch transmitter's traffic. The receiver locks onto the old
    // identity, and the first new-epoch PDU (higher identity) re-establishes the
    // receiving side (TS 38.322 5.1.2: discard all, reset state): the whole
    // new-epoch stream is then delivered in order and byte-exact -- nothing is
    // shadowed, nothing stale is delivered, nothing mixes.
    {
        const std::vector<NrRlcSduSpec> oldSdus = {{'X', 30}, {'Y', 34}, {'Z', 38}};
        Ptr<NrRlcAm> oldTxRlc = CreateObject<NrRlcAm>(); // created first: older identity
        NrRlcTestCaptureMac oldCapture;
        oldTxRlc->SetNrMacSapProvider(&oldCapture);
        TransmitSdusAligned(oldTxRlc, oldSdus, 60);
        NS_TEST_ASSERT_MSG_EQ(oldCapture.m_pdus.size(),
                              3,
                              "epoch-mix sanity: one SDU-aligned PDU per old-epoch SDU");
        Ptr<Packet> stalePdu = oldCapture.m_pdus.at(2); // old-epoch SN=2, carries 'Z'

        Ptr<NrRlcAm> newTxRlc = CreateObject<NrRlcAm>(); // created second: newer identity
        NrRlcTestCaptureMac newCapture;
        newTxRlc->SetNrMacSapProvider(&newCapture);
        QueueSdus(newTxRlc, txSdus);
        GrantTxOpportunities(newTxRlc, 44, 20);

        Ptr<NrRlcAm> rxRlc = CreateObject<NrRlcAm>();
        NrRlcTestCaptureMac rxCapture;
        rxRlc->SetNrMacSapProvider(&rxCapture);
        NrRlcTestSduSink sink;
        rxRlc->SetNrRlcSapUser(sink.GetSapUser());

        // Pre-handover, the receiver operated normally in the old epoch: SN=0 is
        // delivered in order (advancing VR(R), so the re-establishment must also
        // reset the sequence-number modulus bases, not just the values), and SN=2
        // is buffered behind the lost SN=1.
        DeliverPduToRlc(rxRlc, oldCapture.m_pdus.at(0));
        DeliverPduToRlc(rxRlc, stalePdu);
        NS_TEST_ASSERT_MSG_EQ(sink.m_sdus.size(), 1, "the in-order old-epoch SDU is delivered");

        for (const auto& pdu : newCapture.m_pdus)
        {
            DeliverPduToRlc(rxRlc, pdu);
        }

        // The re-establishment discards the buffered stale PDU; the whole new-epoch
        // stream is delivered in order, everything byte-exact and nothing mixed.
        std::vector<NrRlcSduSpec> expected = {{'X', 30}};
        expected.insert(expected.end(), txSdus.begin(), txSdus.end());
        VerifyDelivered(sink, expected, "epoch mix with re-establishment");
    }

    Simulator::Destroy();
}

/**
 * @ingroup tests
 *
 * @brief Conformance-style test of the AM receiver status triggers and STATUS
 * generation (TS 38.523-1 clause 7.1.2.3.7, adapted to this 36.322-style entity).
 *
 * Verifies that a poll triggers a STATUS report; that the generated STATUS carries
 * the correct ACK_SN and NACKs exactly the missing SNs below VR(MS) (this is the
 * only direct test of the STATUS generation content); that t-Reordering expiry
 * triggers a STATUS report by itself; and that a report triggered while
 * t-StatusProhibit runs is withheld until the prohibit timer expires.
 */
class NrRlcAmStatusTriggerTestCase : public NrRlcTestCaseBase
{
  public:
    NrRlcAmStatusTriggerTestCase()
        : NrRlcTestCaseBase("Test RLC AM RX: status triggers and generated STATUS content")
    {
    }

  private:
    void DoRun() override;
};

void
NrRlcAmStatusTriggerTestCase::DoRun()
{
    Ptr<NrRlcAm> rlc = CreateObject<NrRlcAm>();
    NrRlcTestCaptureMac capture;
    rlc->SetNrMacSapProvider(&capture);
    NrRlcTestSduSink sink;
    rlc->SetNrRlcSapUser(sink.GetSapUser());

    // In-order SN=0, then SN=2 with a poll (SN=1 missing): the poll must trigger a
    // STATUS report reflecting the receiver state.
    DeliverAmDataPdu(rlc, 0, 20, false);
    DeliverAmDataPdu(rlc, 2, 22, true);
    GrantTxOpportunities(rlc, 20, 1);
    NS_TEST_ASSERT_MSG_EQ(capture.m_pdus.size(), 1, "the poll must trigger a STATUS report");
    {
        NrRlcAmHeader status;
        capture.m_pdus.at(0)->PeekHeader(status);
        NS_TEST_ASSERT_MSG_EQ(status.IsControlPdu(), true, "a STATUS PDU must be emitted");
        // VR(MS) has not advanced past the missing SN=1 yet, so nothing may be
        // NACKed and ACK_SN acknowledges the in-order prefix only.
        NS_TEST_ASSERT_MSG_EQ(status.GetAckSn().GetValue(),
                              1,
                              "ACK_SN must acknowledge the in-order prefix");
        NS_TEST_ASSERT_MSG_EQ(status.IsNackPresent(nr::SequenceNumber10(1)),
                              false,
                              "SN=1 may not be NACKed before t-Reordering expires");
    }

    // t-Reordering expiry (10 ms) must advance VR(MS) past the loss and trigger a
    // STATUS report on its own; the report must NACK exactly the missing SN=1.
    // (Bounded run: RLC BSR timers self-reschedule while data is outstanding.)
    Simulator::Stop(MilliSeconds(15));
    Simulator::Run();
    GrantTxOpportunities(rlc, 20, 1);
    NS_TEST_ASSERT_MSG_EQ(capture.m_pdus.size(),
                          2,
                          "t-Reordering expiry must trigger a STATUS report");
    {
        NrRlcAmHeader status;
        capture.m_pdus.at(1)->PeekHeader(status);
        NS_TEST_ASSERT_MSG_EQ(status.IsNackPresent(nr::SequenceNumber10(1)),
                              true,
                              "the missing SN=1 must be NACKed");
        NS_TEST_ASSERT_MSG_EQ(status.IsNackPresent(nr::SequenceNumber10(2)),
                              false,
                              "the received SN=2 must not be NACKed");
        NS_TEST_ASSERT_MSG_EQ(status.GetAckSn().GetValue(),
                              3,
                              "ACK_SN must point past the highest reported SN");
    }

    // A report triggered while t-StatusProhibit runs (just re-armed by the report
    // above) must be withheld until the prohibit timer expires.
    DeliverAmDataPdu(rlc, 4, 24, true);
    GrantTxOpportunities(rlc, 20, 1);
    NS_TEST_ASSERT_MSG_EQ(capture.m_pdus.size(),
                          2,
                          "the report must be withheld while t-StatusProhibit runs");
    Simulator::Stop(MilliSeconds(15)); // t-StatusProhibit and t-Reordering expire
    Simulator::Run();
    GrantTxOpportunities(rlc, 20, 1);
    NS_TEST_ASSERT_MSG_EQ(capture.m_pdus.size(),
                          3,
                          "the withheld report must be sent after t-StatusProhibit expires");
    {
        NrRlcAmHeader status;
        capture.m_pdus.at(2)->PeekHeader(status);
        NS_TEST_ASSERT_MSG_EQ(status.IsNackPresent(nr::SequenceNumber10(1)),
                              true,
                              "the still-missing SN=1 must be NACKed");
        NS_TEST_ASSERT_MSG_EQ(status.IsNackPresent(nr::SequenceNumber10(3)),
                              true,
                              "the missing SN=3 must be NACKed");
        NS_TEST_ASSERT_MSG_EQ(status.GetAckSn().GetValue(),
                              5,
                              "ACK_SN must point past the highest reported SN");
    }

    Simulator::Destroy();
}

/**
 * @ingroup tests
 *
 * @brief Conformance-style test of AM polling (TS 38.523-1 clause 7.1.2.3.6,
 * adapted to this 36.322-style entity).
 *
 * Verifies that the last data in the buffer is transmitted with the polling bit
 * set, and that t-PollRetransmit expiry without a STATUS answer retransmits the
 * polled PDU with the polling bit set again.
 */
class NrRlcAmPollingTestCase : public NrRlcTestCaseBase
{
  public:
    NrRlcAmPollingTestCase()
        : NrRlcTestCaseBase("Test RLC AM TX: polling bit on last data and t-PollRetransmit expiry")
    {
    }

  private:
    void DoRun() override;
};

void
NrRlcAmPollingTestCase::DoRun()
{
    Ptr<NrRlcAm> rlc = CreateObject<NrRlcAm>();
    NrRlcTestCaptureMac capture;
    rlc->SetNrMacSapProvider(&capture);

    // Transmitting the last (only) data in the buffer must set the polling bit.
    rlc->DoTransmitPdcpPdu(Create<Packet>(30));
    GrantTxOpportunities(rlc, 40, 1);
    NS_TEST_ASSERT_MSG_EQ(capture.m_pdus.size(), 1, "one AMD PDU must be transmitted");
    {
        NrRlcAmHeader header;
        capture.m_pdus.at(0)->PeekHeader(header);
        NS_TEST_ASSERT_MSG_EQ(header.IsControlPdu(), false, "an AMD PDU must be emitted");
        NS_TEST_ASSERT_MSG_EQ((uint16_t)header.GetPollingBit(),
                              (uint16_t)NrRlcAmHeader::STATUS_REPORT_IS_REQUESTED,
                              "the last data in the buffer must carry a poll");
    }

    // No STATUS answers the poll: t-PollRetransmit expiry (20 ms) must move the
    // polled PDU to the retransmission buffer, and the next opportunity must
    // retransmit it with the polling bit set again. (Bounded run: the BSR timer
    // self-reschedules while un-acknowledged data is outstanding.)
    Simulator::Stop(MilliSeconds(25));
    Simulator::Run();
    GrantTxOpportunities(rlc, 40, 1);
    NS_TEST_ASSERT_MSG_EQ(capture.m_pdus.size(),
                          2,
                          "t-PollRetransmit expiry must lead to a retransmission");
    {
        NrRlcAmHeader header;
        capture.m_pdus.at(1)->PeekHeader(header);
        NS_TEST_ASSERT_MSG_EQ(header.IsControlPdu(), false, "the retransmission is an AMD PDU");
        NS_TEST_ASSERT_MSG_EQ(header.GetSequenceNumber().GetValue(),
                              0,
                              "the polled PDU must be the one retransmitted");
        NS_TEST_ASSERT_MSG_EQ((uint16_t)header.GetPollingBit(),
                              (uint16_t)NrRlcAmHeader::STATUS_REPORT_IS_REQUESTED,
                              "the retransmission after poll expiry must poll again");
    }

    Simulator::Destroy();
}

/**
 * @ingroup tests
 *
 * @brief Conformance-style test of the AM transmit and receive window control
 * (TS 38.523-1 clause 7.1.2.3.5, adapted to this 36.322-style entity).
 *
 * Verifies that the transmitter stalls at VT(A) + AM_Window_Size and does not emit
 * AMD PDUs beyond the transmit window; that acknowledging part of the window
 * resumes transmission within the updated range; and that the receiver discards
 * AMD PDUs outside the receive window while accepting in-window ones.
 */
class NrRlcAmWindowTestCase : public NrRlcTestCaseBase
{
  public:
    NrRlcAmWindowTestCase()
        : NrRlcTestCaseBase("Test RLC AM: transmit window stall, resume on ACK, receive window")
    {
    }

  private:
    void DoRun() override;
};

void
NrRlcAmWindowTestCase::DoRun()
{
    const uint16_t windowSize = 512;

    Ptr<NrRlcAm> rlc = CreateObject<NrRlcAm>();
    rlc->SetAttribute("MaxTxBufferSize", UintegerValue(0)); // unlimited
    NrRlcTestCaptureMac capture;
    rlc->SetNrMacSapProvider(&capture);

    // Queue more SDUs than the transmit window holds (at least 100 beyond it, so
    // the resume step below is not limited by the buffer); grants are sized so
    // each PDU carries exactly one SDU. The transmitter must stall at VT(A) + 512.
    for (uint32_t i = 0; i < windowSize + 188u; i++)
    {
        rlc->DoTransmitPdcpPdu(Create<Packet>(36));
    }
    GrantTxOpportunities(rlc, 40, windowSize + 188u);
    NS_TEST_ASSERT_MSG_EQ(capture.m_pdus.size(),
                          windowSize,
                          "the transmitter must not emit AMD PDUs beyond VT(A) + AM_Window_Size");
    {
        NrRlcAmHeader header;
        capture.m_pdus.back()->PeekHeader(header);
        NS_TEST_ASSERT_MSG_EQ(header.GetSequenceNumber().GetValue(),
                              windowSize - 1,
                              "the last emitted SN must be the window edge");
    }

    // Acknowledging the first 100 SNs slides the window: exactly 100 more PDUs may
    // now be transmitted, and no more.
    DeliverStatusPdu(rlc, 100, {});
    GrantTxOpportunities(rlc, 40, 200);
    NS_TEST_ASSERT_MSG_EQ(capture.m_pdus.size(),
                          (uint32_t)windowSize + 100,
                          "acknowledging part of the window must resume transmission within the "
                          "updated range");
    {
        NrRlcAmHeader header;
        capture.m_pdus.back()->PeekHeader(header);
        NS_TEST_ASSERT_MSG_EQ(header.GetSequenceNumber().GetValue(),
                              windowSize + 99,
                              "the last emitted SN must be the updated window edge");
    }

    // Receive window: a PDU with SN outside [VR(R), VR(MR)) must be discarded, and
    // in-window PDUs must still be accepted afterwards.
    Ptr<NrRlcAm> rxRlc = CreateObject<NrRlcAm>();
    NrRlcTestCaptureMac rxCapture;
    rxRlc->SetNrMacSapProvider(&rxCapture);
    NrRlcTestSduSink sink;
    rxRlc->SetNrRlcSapUser(sink.GetSapUser());

    DeliverAmDataPdu(rxRlc, 700, 20, false); // outside [0, 512): must be discarded
    NS_TEST_ASSERT_MSG_EQ(sink.m_sdus.size(),
                          0,
                          "a PDU outside the receive window must be discarded");
    DeliverAmDataPdu(rxRlc, 0, 20, false); // in window: delivered in sequence
    NS_TEST_ASSERT_MSG_EQ(sink.m_sdus.size(),
                          1,
                          "in-window PDUs must still be accepted after the discard");

    Simulator::Destroy();
}

class NrRlcAmStatusTestSuite : public TestSuite
{
  public:
    NrRlcAmStatusTestSuite()
        : TestSuite("nr-test-rlc-am-status", Type::SYSTEM)
    {
        AddTestCase(new NrRlcAmStaleStatusTestCase(), Duration::QUICK);
        AddTestCase(new NrRlcAmReassemblyResyncTestCase(), Duration::QUICK);
        AddTestCase(new NrRlcAmDataIntegrityTestCase(), Duration::QUICK);
        AddTestCase(new NrRlcAmStatusTriggerTestCase(), Duration::QUICK);
        AddTestCase(new NrRlcAmPollingTestCase(), Duration::QUICK);
        AddTestCase(new NrRlcAmWindowTestCase(), Duration::QUICK);
    }
};

static NrRlcAmStatusTestSuite g_nrRlcAmStatusTestSuite;

} // namespace ns3
