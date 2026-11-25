// Copyright (c) 2026 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only

/**
 * @ingroup test
 * @file nr-test-rlc-um-rx.cc
 *
 * @brief Test suite `nr-test-rlc-um-rx`: white-box regression tests of the NrRlcUm reception and
 * transmission buffers, driving an NrRlcUm instance wired to test PDCP/MAC entities with
 * hand-crafted PDUs and pre-set state variables. The cases verify that buffered SNs are
 * reassembled (and removed) before VR(UR) advances so no PDU is left outside the reception
 * window, including at 10-bit SN wrap-around; that a stranded partial SDU is discarded on
 * t-Reordering expiry so no corrupt SDU is delivered to PDCP; that an SDU discarded by the PDCP
 * discard timer fires the TxDrop trace once and is not enqueued in the Tx buffer; that
 * ReassembleSnInterval skips missing SNs safely with logging enabled while reassembling and
 * delivering the SNs that are present; that every transition of the reassembly state machine
 * (state x FI x SN-continuity x segment count) delivers exactly the expected SDU bytes; that a
 * stale PDU aliasing the expected SN after a handover resynchronises the reassembler instead of
 * aborting the simulation; that the Tx side drops SDUs on buffer overflow and on
 * packet-delay-budget expiry, produces no PDU for tiny or empty-buffer Tx opportunities, and
 * never concatenates after a > 2047-byte SDU; that t-Reordering starts, stops, delivers past
 * unfilled holes and restarts correctly; and that OutOfOrderDelivery delivers immediately
 * without waiting for missing SNs, byte-exact whatever the order the PDUs of a segmented SDU
 * arrive in.
 */

#include "nr-rlc-test-utils.h"
#include "nr-test-entities.h"

#include "ns3/abort.h"
#include "ns3/boolean.h"
#include "ns3/log.h"
#include "ns3/nr-rlc-header.h"
#include "ns3/nr-rlc-sap.h"
#include "ns3/nr-rlc-tag.h"
#include "ns3/nr-rlc-um.h"
#include "ns3/packet.h"
#include "ns3/simulator.h"
#include "ns3/uinteger.h"

#include <algorithm>
#include <set>
#include <string>
#include <utility>
#include <vector>

NS_LOG_COMPONENT_DEFINE("NrRlcUmTestCase");

namespace ns3
{

namespace
{

/**
 * Create an UMD PDU carrying the given payload.
 *
 * @param sn RLC Sequence Number (SN) of the PDU
 * @param framingInfo Framing Info (FI) flag, indicates SDU boundaries:
 *                    00 = starts and ends with an SDU
 *                    01 = starts with an SDU but does not end with one
 *                    10 = does not start with an SDU but ends with one
 *                    11 = neither starts nor ends with an SDU
 * @param payload the PDU payload, to which the RLC header is prepended
 * @param liSizes the length indicators delimiting the data field elements (one per
 *                element except the last, which is delimited by E = 0)
 * @return the UMD PDU
 */
Ptr<Packet>
MakeUmPdu(uint16_t sn,
          uint8_t framingInfo,
          Ptr<Packet> payload,
          const std::vector<uint16_t>& liSizes = {})
{
    NrRlcHeader rlcHeader;
    rlcHeader.SetSequenceNumber(nr::SequenceNumber10(sn));
    for (uint16_t li : liSizes)
    {
        rlcHeader.PushExtensionBit(NrRlcHeader::E_LI_FIELDS_FOLLOWS);
        rlcHeader.PushLengthIndicator(li);
    }
    rlcHeader.PushExtensionBit(NrRlcHeader::DATA_FIELD_FOLLOWS);
    rlcHeader.SetFramingInfo(framingInfo);

    payload->AddHeader(rlcHeader);
    return payload;
}

/**
 * Deliver a PDU to the RLC entity through the MAC SAP, tagged with the given sender
 * timestamp.
 *
 * @param rlc the RLC entity under test
 * @param pdu the PDU to deliver
 * @param senderTimestamp the sender timestamp carried by the PDU
 */
void
DeliverUmPdu(Ptr<NrRlcUm> rlc, Ptr<Packet> pdu, Time senderTimestamp)
{
    NrRlcTag rlcTag(senderTimestamp);
    pdu->AddByteTag(rlcTag);

    NrMacSapUser::ReceivePduParameters rxPduParams = {};
    rxPduParams.p = pdu;
    rxPduParams.rnti = 0;
    rxPduParams.lcid = 0;
    rlc->DoReceivePdu(rxPduParams);
}

/**
 * Deliver one single-segment PDU with the given sender timestamp to the RLC entity
 * through the MAC SAP.
 *
 * @param rlc the RLC entity under test
 * @param sn the RLC sequence number
 * @param framingInfo the FI field
 * @param payloadSize the payload size in bytes (used by the tests to identify the SDU)
 * @param senderTimestamp the sender timestamp carried by the PDU
 */
void
DeliverPduAt(Ptr<NrRlcUm> rlc,
             uint16_t sn,
             uint8_t framingInfo,
             uint32_t payloadSize,
             Time senderTimestamp)
{
    DeliverUmPdu(rlc, MakeUmPdu(sn, framingInfo, Create<Packet>(payloadSize)), senderTimestamp);
}

/**
 * Deliver one single-segment PDU to the RLC entity through the MAC SAP.
 *
 * @param rlc the RLC entity under test
 * @param sn the RLC sequence number
 * @param framingInfo the FI field
 * @param payloadSize the payload size in bytes (used by the tests to identify the SDU)
 */
void
DeliverPdu(Ptr<NrRlcUm> rlc, uint16_t sn, uint8_t framingInfo, uint32_t payloadSize)
{
    DeliverPduAt(rlc, sn, framingInfo, payloadSize, Simulator::Now());
}

/// Size in bytes of the payload segment identified by each fill letter ('S', 'A', 'B', 'C')
uint32_t
SegmentSize(char letter)
{
    switch (letter)
    {
    case 'S':
        return 40;
    case 'A':
        return 21;
    case 'B':
        return 22;
    case 'C':
        return 23;
    default:
        NS_ABORT_MSG("Unknown segment letter " << letter);
        return 0;
    }
}

/**
 * Create a payload packet composed of the given segments, each filled with its letter.
 *
 * @param letters the segment letters, e.g. "SA" = 40 bytes of 'S' followed by 21 bytes of 'A'
 * @return the payload packet
 */
Ptr<Packet>
MakePayload(const std::string& letters)
{
    std::vector<uint8_t> buf;
    for (char letter : letters)
    {
        buf.insert(buf.end(), SegmentSize(letter), (uint8_t)letter);
    }
    return Create<Packet>(buf.data(), buf.size());
}

/**
 * Check that a delivered SDU is exactly the concatenation of the given segments.
 *
 * @param sdu the delivered SDU
 * @param letters the expected segment letters
 * @return true if size and every byte match
 */
bool
SduMatches(Ptr<Packet> sdu, const std::string& letters)
{
    uint32_t expectedSize = 0;
    for (char letter : letters)
    {
        expectedSize += SegmentSize(letter);
    }
    if (sdu->GetSize() != expectedSize)
    {
        return false;
    }
    std::vector<uint8_t> buf(sdu->GetSize());
    sdu->CopyData(buf.data(), buf.size());
    uint32_t offset = 0;
    for (char letter : letters)
    {
        for (uint32_t i = 0; i < SegmentSize(letter); i++)
        {
            if (buf[offset + i] != (uint8_t)letter)
            {
                return false;
            }
        }
        offset += SegmentSize(letter);
    }
    return true;
}

} // namespace

class NrRlcUmTestCase : public NrRlcTestCaseBase
{
  public:
    NrRlcUmTestCase()
        : NrRlcTestCaseBase("Test RLC UM RX: Check packets not reassembled when window advances")
    {
    }

  private:
    void DoRun() override;

    Ptr<NrTestPdcp> rxPdcp; ///< The reception PDCP
};

void
NrRlcUmTestCase::DoRun()
{
    auto logLevel = (LogLevel)(LOG_PREFIX_FUNC | LOG_PREFIX_TIME | LOG_LEVEL_ALL);
    LogComponentEnable("NrRlcUmTestCase", logLevel);
    LogComponentEnable("NrRlcUm", logLevel);

    NS_LOG_INFO("-----------------------------------------------------------------------------");
    NS_LOG_INFO("DoRun started");

    // Instantiate RLC
    Ptr<NrRlcUm> rlc = CreateObject<NrRlcUm>();

    // Create transmission PDCP test entity
    rxPdcp = CreateObject<NrTestPdcp>();

    // Connect RLC with Test PDCP
    rlc->SetNrRlcSapUser(rxPdcp->GetNrRlcSapUser());

    // 1) Bug check: In the implementation before MR !354, SNs {1016, 1017} remained in m_rxBuffer
    // un-reassembled. In the updated implementation (MR !354), these PDUs are reassembled before
    // advancing VR(UR) to 1018.

    NS_LOG_INFO("Step 1.1: Initialized m_rxBuffer with SN 0, 502, 1016, 1017, 1018");
    rlc->m_rxBuffer[0] = MakeUmPdu(0, NrRlcHeader::FIRST_BYTE, Create<Packet>(10));
    rlc->m_rxBuffer[1016] = MakeUmPdu(1016, NrRlcHeader::FIRST_BYTE, Create<Packet>(20));
    rlc->m_rxBuffer[1017] =
        MakeUmPdu(1017, NrRlcHeader::FIRST_BYTE | NrRlcHeader::NO_LAST_BYTE, Create<Packet>(15));
    rlc->m_rxBuffer[1018] =
        MakeUmPdu(1018, NrRlcHeader::NO_FIRST_BYTE | NrRlcHeader::LAST_BYTE, Create<Packet>(15));
    rlc->m_rxBuffer[502] = MakeUmPdu(502, NrRlcHeader::FIRST_BYTE, Create<Packet>(30));

    NS_LOG_INFO(
        "Step 1.2: Reception window set (VR(UR)=1015, VR(UH)=503, windowSize=512) -> [1015, 503)");
    rlc->m_vrUr = 1015;
    rlc->m_vrUx = 0;
    rlc->m_vrUh = 503;
    rlc->m_windowSize = 512;

    rlc->m_expectedSeqNumber = 1015;
    rlc->m_reassemblingState = NrRlcUm::WAITING_S0_FULL;

    NS_LOG_INFO("Step 1.3: Creating PDU with SN=505");
    NS_LOG_INFO("Step 1.4: Calling DoReceivePdu with SN=505");
    NS_LOG_INFO("-----------------------------------------------------------------------------");
    DeliverPdu(rlc, 505, NrRlcHeader::FIRST_BYTE, 50);

    NS_LOG_INFO("-----------------------------------------------------------------------------");
    NS_LOG_INFO("Step 1.5: DoReceivePdu finished");

    for (const auto& entry : rlc->m_rxBuffer)
    {
        uint16_t sn = entry.first;
        uint16_t vrUr = rlc->m_vrUr.GetValue();
        uint16_t vrUh = rlc->m_vrUh.GetValue();

        bool insideWindow;

        if (vrUr <= vrUh)
        {
            // No wrap
            insideWindow = (sn >= vrUr && sn < vrUh);
        }
        else
        {
            // Wrap-around
            insideWindow = (sn >= vrUr || sn < vrUh);
        }

        if (!insideWindow)
        {
            NS_LOG_ERROR(
                " ERROR: Packet stored outside reception window, SN=" + std::to_string(sn) +
                ", [VR(UR)=" + std::to_string(vrUr) + ", VR(UH)=" + std::to_string(vrUh) + ")");

            // Trigger the assert
            NS_TEST_ASSERT_MSG_EQ(false, true, "");
        }
    }

    NS_LOG_INFO("m_rxBuffer key (SN) contents after DoReceivePdu:");
    for (const auto& entry : rlc->m_rxBuffer)
    {
        NS_LOG_INFO("SN = " << entry.first);
    }

    NS_LOG_INFO("-----------------------------------------------------------------------------");

    // 2) Bug check: In the implementation before MR !354, SN {1016, 1017} remain in m_rxBuffer
    // without being reassembled. Therefore, when a new PDU with SN=1016 is received by RLC UM RX,
    // the code detects that a PDU with the same SN already exists in the buffer and the ASSERT is
    // triggered. In the updated implementation (MR !354), these PDUs are reassembled before
    // advancing VR(UR) to 1018, so they are removed from m_rxBuffer.

    NS_LOG_INFO(
        "Step 2.1: Reception window set (VR(UR)=504, VR(UH)=1016, windowSize=512) -> [504, 1016)");
    rlc->m_vrUr = 504;
    rlc->m_vrUx = 0;
    rlc->m_vrUh = 1016;
    rlc->m_windowSize = 512;

    rlc->m_expectedSeqNumber = 504;
    rlc->m_reassemblingState = NrRlcUm::WAITING_S0_FULL;

    NS_LOG_INFO("Step 2.2: Store SN=1015 and remove SN=502 from the RX buffer (m_rxBuffer), as it "
                "should have already been reassembled");
    rlc->m_rxBuffer[1015] = MakeUmPdu(1015, NrRlcHeader::FIRST_BYTE, Create<Packet>(20));
    rlc->m_rxBuffer.erase(502);

    NS_LOG_INFO("Step 2.3: Creating PDU with SN=1016");
    NS_LOG_INFO("Step 2.4: Calling DoReceivePdu with SN=1016");
    NS_LOG_INFO("-----------------------------------------------------------------------------");
    DeliverPdu(rlc, 1016, NrRlcHeader::FIRST_BYTE | NrRlcHeader::NO_LAST_BYTE, 50);
    NS_LOG_INFO("-----------------------------------------------------------------------------");
    NS_LOG_INFO("m_rxBuffer key (SN) contents after DoReceivePdu:");
    for (const auto& entry : rlc->m_rxBuffer)
    {
        NS_LOG_INFO("SN = " << entry.first);
    }
    NS_LOG_INFO("-----------------------------------------------------------------------------");

    // 3) Bug check: In the implementation before MR !354, some PDUs were not removed from the RX
    // buffer. This could cause the same SN to be received again in a new cycle while a PDU with the
    // same SN from the previous cycle still exists in m_rxBuffer. For example, SN=1016 could
    // arrive again even though SN=1016 from the previous cycle is still present. If the current
    // PDU with SN=1016 is lost (corrupted or not received), the leftover PDU from the previous
    // cycle could be processed, leading to an incorrect FI transition.
    //
    // In the updated implementation (MR !354), these PDUs are reassembled before advancing VR(UR)
    // to 1018, ensuring they are removed from m_rxBuffer and avoiding such incorrect FI
    // transitions.

    NS_LOG_INFO(
        "Step 3.1: Reception window set (VR(UR)=1014, VR(UH)=502, windowSize=512) -> [1014, 502)");
    rlc->m_vrUr = 1014;
    rlc->m_vrUx = 0;
    rlc->m_vrUh = 502;
    rlc->m_windowSize = 512;

    rlc->m_expectedSeqNumber = 1014;
    rlc->m_reassemblingState = NrRlcUm::WAITING_S0_FULL;

    NS_LOG_INFO("Step 3.2: Creating PDU with SN=502");
    NS_LOG_INFO("Step 3.3: Calling DoReceivePdu with SN=502");
    NS_LOG_INFO("-----------------------------------------------------------------------------");
    DeliverPdu(rlc, 502, NrRlcHeader::FIRST_BYTE | NrRlcHeader::NO_LAST_BYTE, 50);

    NS_LOG_INFO("-----------------------------------------------------------------------------");
    NS_LOG_INFO("m_rxBuffer key (SN) contents after DoReceivePdu:");
    for (const auto& entry : rlc->m_rxBuffer)
    {
        NS_LOG_INFO("SN = " << entry.first);
    }
    NS_LOG_INFO("-----------------------------------------------------------------------------");

    // 4) Check wrap-around: Set the window VR(UR) to 1023 with a packet SN=0 already in the buffer,
    // and verify that the wrap-around behaves correctly.

    NS_LOG_INFO("Step 4.1: Window check, Reception window set (VR(UR)=1023, VR(UH)=511, "
                "windowSize=512) -> [1023, 511)");
    rlc->m_vrUr = 1023;
    rlc->m_vrUx = 0;
    rlc->m_vrUh = 511;
    rlc->m_windowSize = 512;

    rlc->m_expectedSeqNumber = 1023;
    rlc->m_reassemblingState = NrRlcUm::WAITING_S0_FULL;

    NS_LOG_INFO("Step 4.2: Creating PDU with SN=511");
    NS_LOG_INFO("Step 4.3: Calling DoReceivePdu with SN=511");
    NS_LOG_INFO("-----------------------------------------------------------------------------");
    DeliverPdu(rlc, 511, NrRlcHeader::FIRST_BYTE | NrRlcHeader::NO_LAST_BYTE, 50);

    // Flush the event scheduled by the NrTestPdcp constructor, so that it does not
    // outlive this test case and fire on a destroyed object in the next one.
    Simulator::Destroy();
}

/**
 * @ingroup tests
 *
 * @brief Regression test for the RLC UM reassembly t-Reordering discard.
 *
 * Reproduces the condition that produced corrupt (non-IP) SDUs during fast handovers:
 * a first SDU segment (S0) is received and kept (state WAITING_SI_SF) awaiting a
 * continuation that is then lost. Per TS 38.322 (5.1.2.2.4 / 5.2.2.2) the un-completable
 * partial SDU must be discarded when t-Reordering expires. Without that discard the stale
 * S0 survives until the 10-bit SN wraps (~1024 PDUs), after which an unrelated later
 * segment aliases as the awaited continuation and is concatenated onto the stale head,
 * delivering a corrupt SDU upwards. This test asserts the discard happens.
 */
class NrRlcUmReorderingDiscardTestCase : public NrRlcTestCaseBase
{
  public:
    NrRlcUmReorderingDiscardTestCase()
        : NrRlcTestCaseBase("Test RLC UM RX: stranded partial SDU discarded on t-Reordering expiry")
    {
    }

  private:
    void DoRun() override;
};

void
NrRlcUmReorderingDiscardTestCase::DoRun()
{
    Ptr<NrRlcUm> rlc = CreateObject<NrRlcUm>();
    Ptr<NrTestPdcp> rxPdcp = CreateObject<NrTestPdcp>();
    rlc->SetNrRlcSapUser(rxPdcp->GetNrRlcSapUser());

    // Put the reassembler mid-SDU: a first segment (S0) has been received and is held,
    // awaiting its continuation at SN = m_expectedSeqNumber (= VR(UR) = 200). That
    // continuation is lost (not in m_rxBuffer) and a reordering gap is open.
    rlc->m_keepS0 = Create<Packet>(40);
    rlc->m_reassemblingState = NrRlcUm::WAITING_SI_SF;
    rlc->m_windowSize = 512;
    rlc->m_expectedSeqNumber = nr::SequenceNumber10(200);
    rlc->m_vrUr = 200; // awaited (missing) SN
    rlc->m_vrUx = 200;
    rlc->m_vrUh = 200; // no further buffered SNs -> timer is not rescheduled

    NS_TEST_ASSERT_MSG_EQ((int)rlc->m_reassemblingState,
                          (int)NrRlcUm::WAITING_SI_SF,
                          "precondition: reassembler holds a partial SDU");
    NS_TEST_ASSERT_MSG_EQ((rlc->m_keepS0 != nullptr), true, "precondition: S0 is held");

    // t-Reordering expiry: the missing continuation will never arrive.
    rlc->ExpireReorderingTimer();

    NS_TEST_ASSERT_MSG_EQ((int)rlc->m_reassemblingState,
                          (int)NrRlcUm::WAITING_S0_FULL,
                          "stranded partial SDU must reset the reassembly state on t-Reordering "
                          "expiry");
    NS_TEST_ASSERT_MSG_EQ((rlc->m_keepS0 == nullptr),
                          true,
                          "stranded partial SDU (S0) must be discarded on t-Reordering expiry");
    NS_TEST_ASSERT_MSG_EQ(rxPdcp->GetDataReceived().size(),
                          0,
                          "no corrupt SDU may be delivered to PDCP when the partial SDU is "
                          "discarded");

    // Flush the event scheduled by the NrTestPdcp constructor, so that it does not
    // outlive this test case and fire on a destroyed object in the next one.
    Simulator::Destroy();
}

/**
 * @ingroup tests
 *
 * @brief Regression test for the PDCP discard path in NrRlcUm::DoTransmitPdcpPdu.
 *
 * When PDCP discarding is enabled and the head-of-line delay exceeds the discard
 * timer, the incoming RLC SDU must be dropped (TxDrop trace fired) and, crucially,
 * NOT stored in the transmission buffer. Before the fix the code fell through and
 * still enqueued the "discarded" packet, making the discard a no-op and
 * double-counting the drop statistics. This test asserts the packet is not stored.
 */
class NrRlcUmTxPdcpDiscardTestCase : public NrRlcTestCaseBase
{
  public:
    NrRlcUmTxPdcpDiscardTestCase()
        : NrRlcTestCaseBase("Test RLC UM TX: PDCP-discarded SDU is not enqueued in the Tx buffer")
    {
    }

  private:
    void DoRun() override;
};

void
NrRlcUmTxPdcpDiscardTestCase::DoRun()
{
    Ptr<NrRlcUm> rlc = CreateObject<NrRlcUm>();
    rlc->SetAttribute("EnablePdcpDiscarding", BooleanValue(true));
    rlc->SetAttribute("DiscardTimerMs", UintegerValue(10));

    Ptr<NrTestMac> mac = CreateObject<NrTestMac>();
    mac->SetRlcHeaderType(NrTestMac::UM_RLC_HEADER);
    rlc->SetNrMacSapProvider(mac->GetNrMacSapProvider());
    mac->SetNrMacSapUser(rlc->GetNrMacSapUser());
    rlc->TraceConnectWithoutContext("TxDrop", MakeCallback(&NrRlcTestCaseBase::HandleTxDrop, this));

    // t = 0 ms: first SDU is stored (Tx buffer is empty, so head-of-line delay is 0).
    Simulator::Schedule(MilliSeconds(0), [rlc]() { rlc->DoTransmitPdcpPdu(Create<Packet>(100)); });

    // t = 20 ms: head-of-line delay (20 ms) exceeds the 10 ms discard timer, so the
    // second SDU must be discarded and must NOT be added to the Tx buffer.
    Simulator::Schedule(MilliSeconds(20), [rlc]() { rlc->DoTransmitPdcpPdu(Create<Packet>(200)); });

    Simulator::Run();

    NS_TEST_ASSERT_MSG_EQ(rlc->m_txBuffer.size(),
                          1,
                          "the discarded SDU must not be enqueued in the Tx buffer");
    NS_TEST_ASSERT_MSG_EQ(rlc->m_txBufferSize,
                          100,
                          "the Tx buffer size must only account for the stored (first) SDU");
    NS_TEST_ASSERT_MSG_EQ(m_dropCount,
                          1,
                          "the discarded SDU must fire the TxDrop trace exactly once");

    Simulator::Destroy();
}

/**
 * @ingroup tests
 *
 * @brief Regression test for NrRlcUm::ReassembleSnInterval with a gap in the buffer.
 *
 * ReassembleSnInterval iterates over an SN interval that, by design, may contain
 * missing SNs (holes left by losses). Before the fix, the two NS_LOG_LOGIC lines
 * printing it->first / it->second were placed BEFORE the it != end() check,
 * dereferencing m_rxBuffer.end() for every missing SN (undefined behaviour, tripping
 * asserts in debug builds when logging is enabled). This test enables NrRlcUm logging
 * and reassembles an interval with a hole to ensure gaps are skipped safely while the
 * present SNs are reassembled and removed from the reception buffer.
 */
class NrRlcUmReassembleGapTestCase : public NrRlcTestCaseBase
{
  public:
    NrRlcUmReassembleGapTestCase()
        : NrRlcTestCaseBase("Test RLC UM RX: ReassembleSnInterval safely skips missing SNs")
    {
    }

  private:
    void DoRun() override;
};

void
NrRlcUmReassembleGapTestCase::DoRun()
{
    // Force the code path that used to dereference m_rxBuffer.end(): the NS_LOG_LOGIC
    // lines only execute (and only crash) when the log component is enabled.
    LogComponentEnable("NrRlcUm", (LogLevel)(LOG_LEVEL_ALL | LOG_PREFIX_ALL));

    Ptr<NrRlcUm> rlc = CreateObject<NrRlcUm>();
    Ptr<NrTestPdcp> rxPdcp = CreateObject<NrTestPdcp>();
    rlc->SetNrRlcSapUser(rxPdcp->GetNrRlcSapUser());

    rlc->m_windowSize = 512;
    rlc->m_reassemblingState = NrRlcUm::WAITING_S0_FULL;

    // Build two self-contained full-SDU PDUs at SN=5 and SN=7, leaving SN=6 as a gap.
    const uint8_t fi00 = NrRlcHeader::FIRST_BYTE | NrRlcHeader::LAST_BYTE;
    rlc->m_rxBuffer[5] = MakeUmPdu(5, fi00, Create<Packet>(20));
    rlc->m_rxBuffer[7] = MakeUmPdu(7, fi00, Create<Packet>(20));

    // Reassemble the interval [5, 8): SN=6 is missing and must be skipped without any
    // dereference of m_rxBuffer.end().
    rlc->ReassembleSnInterval(nr::SequenceNumber10(5), nr::SequenceNumber10(8));

    NS_TEST_ASSERT_MSG_EQ(rlc->m_rxBuffer.count(5),
                          0,
                          "SN=5 must be reassembled and removed from the reception buffer");
    NS_TEST_ASSERT_MSG_EQ(rlc->m_rxBuffer.count(7),
                          0,
                          "SN=7 must be reassembled and removed from the reception buffer");
    // NrTestPdcp overwrites its buffer on each delivery, so it holds the last SDU only;
    // a non-empty 20-byte payload confirms a full SDU was reassembled and delivered.
    NS_TEST_ASSERT_MSG_EQ(rxPdcp->GetDataReceived().size(),
                          20,
                          "a full SDU must be reassembled and delivered to PDCP");

    // Flush the event scheduled by the NrTestPdcp constructor, so that it does not
    // outlive this test case and fire on a destroyed object in the next one.
    Simulator::Destroy();
}

/**
 * @ingroup tests
 *
 * @brief Exhaustive matrix test of the NrRlcUm reassembly state machine.
 *
 * Drives ReassembleAndDeliver directly through every transition: both reassembling
 * states (WAITING_S0_FULL, WAITING_SI_SF), all four FI values, in-sequence and
 * out-of-sequence SNs, single- and multi-segment PDUs, and the defensive
 * WAITING_SI_SF-without-held-S0 condition. For each transition it verifies the
 * delivered SDUs (count, order, and exact payload bytes), the resulting state, the
 * held S0 composition, the updated expected SN, and the state/S0 consistency
 * invariant.
 *
 * In particular it pins the resynchronisation behaviour when the SN matches the
 * expected SN but the FI contradicts the reassembly state (PDU starts a new SDU
 * while a partial SDU is held). That combination is reachable over the air: 10-bit
 * SNs alias when RLC entities are re-created at handover while old-epoch PDUs are
 * still in flight, or after >= 1023 consecutive losses. It used to abort the
 * simulation with "INTERNAL ERROR: We are in the WAITING_SI_SF state and no packet
 * loss has occurred"; it must instead discard the stale partial SDU and deliver the
 * new SDUs intact.
 */
class NrRlcUmReassemblyMatrixTestCase : public NrRlcTestCaseBase
{
  public:
    NrRlcUmReassemblyMatrixTestCase()
        : NrRlcTestCaseBase("Test RLC UM RX: reassembly state machine transition matrix")
    {
    }

  private:
    /// One reassembly transition and its expected outcome
    struct MatrixCase
    {
        std::string name;                      ///< case description
        int startState;                        ///< initial reassembling state
        bool holdS0;                           ///< whether a partial SDU 'S' is held on entry
        bool snMatches;                        ///< whether the PDU SN equals the expected SN
        uint8_t framingInfo;                   ///< FI field of the PDU
        std::string segments;                  ///< PDU data-field segments, one letter each
        std::vector<std::string> expectedSdus; ///< expected delivered SDUs, in order
        int endState;                          ///< expected reassembling state after the call
        std::string expectedKeepS0;            ///< expected held S0 after the call ("" = none)
    };

    void DoRun() override;

    /**
     * Run one matrix case against a fresh RLC entity.
     *
     * @param c the case to run
     */
    void RunCase(const MatrixCase& c);
};

void
NrRlcUmReassemblyMatrixTestCase::RunCase(const MatrixCase& c)
{
    Ptr<NrRlcUm> rlc = CreateObject<NrRlcUm>();
    NrRlcTestSduSink sink;
    rlc->SetNrRlcSapUser(sink.GetSapUser());

    rlc->m_reassemblingState = (NrRlcUm::ReassemblingState_t)c.startState;
    rlc->m_keepS0 = c.holdS0 ? MakePayload("S") : nullptr;
    rlc->m_expectedSeqNumber = nr::SequenceNumber10(10);
    const uint16_t sn = c.snMatches ? 10 : 13;

    // Build the PDU: concatenated segments plus an UM header carrying SN, FI and
    // one E/LI pair per segment except the last (which is delimited by E=0).
    std::vector<uint16_t> liSizes;
    for (uint32_t i = 0; i + 1 < c.segments.size(); i++)
    {
        liSizes.push_back(SegmentSize(c.segments[i]));
    }
    rlc->ReassembleAndDeliver(MakeUmPdu(sn, c.framingInfo, MakePayload(c.segments), liSizes));

    NS_TEST_ASSERT_MSG_EQ(sink.m_sdus.size(),
                          c.expectedSdus.size(),
                          c.name << ": number of delivered SDUs");
    for (uint32_t i = 0; i < std::min(sink.m_sdus.size(), c.expectedSdus.size()); i++)
    {
        NS_TEST_ASSERT_MSG_EQ(SduMatches(sink.m_sdus[i], c.expectedSdus[i]),
                              true,
                              c.name << ": SDU " << i << " must be exactly '" << c.expectedSdus[i]
                                     << "'");
    }

    NS_TEST_ASSERT_MSG_EQ((int)rlc->m_reassemblingState, c.endState, c.name << ": final state");
    if (c.expectedKeepS0.empty())
    {
        NS_TEST_ASSERT_MSG_EQ((rlc->m_keepS0 == nullptr), true, c.name << ": no S0 must be held");
    }
    else
    {
        NS_TEST_ASSERT_MSG_EQ((rlc->m_keepS0 != nullptr), true, c.name << ": an S0 must be held");
        NS_TEST_ASSERT_MSG_EQ(SduMatches(rlc->m_keepS0, c.expectedKeepS0),
                              true,
                              c.name << ": held S0 must be exactly '" << c.expectedKeepS0 << "'");
    }

    // Cross-cutting invariants: S0 is held if and only if the reassembler is
    // mid-SDU, the per-PDU segment list is fully consumed, and the expected SN
    // always advances to SN + 1.
    NS_TEST_ASSERT_MSG_EQ((rlc->m_keepS0 != nullptr),
                          (rlc->m_reassemblingState == NrRlcUm::WAITING_SI_SF),
                          c.name << ": state/S0 consistency invariant");
    NS_TEST_ASSERT_MSG_EQ(rlc->m_sdusBuffer.size(), 0, c.name << ": segment list fully consumed");
    NS_TEST_ASSERT_MSG_EQ(rlc->m_expectedSeqNumber.GetValue(),
                          (uint16_t)(sn + 1),
                          c.name << ": expected SN advances to SN + 1");
}

void
NrRlcUmReassemblyMatrixTestCase::DoRun()
{
    const int s0Full = NrRlcUm::WAITING_S0_FULL;
    const int siSf = NrRlcUm::WAITING_SI_SF;
    const uint8_t fi00 = NrRlcHeader::FIRST_BYTE | NrRlcHeader::LAST_BYTE;
    const uint8_t fi01 = NrRlcHeader::FIRST_BYTE | NrRlcHeader::NO_LAST_BYTE;
    const uint8_t fi10 = NrRlcHeader::NO_FIRST_BYTE | NrRlcHeader::LAST_BYTE;
    const uint8_t fi11 = NrRlcHeader::NO_FIRST_BYTE | NrRlcHeader::NO_LAST_BYTE;

    std::vector<MatrixCase> cases;

    // WAITING_S0_FULL: nothing is held, so in-sequence and out-of-sequence PDUs
    // must be handled identically. Each shape is run with both SN conditions.
    for (bool snMatches : {true, false})
    {
        const std::string tag = snMatches ? " (SN in sequence)" : " (SN loss)";
        cases.push_back(
            {"S0_FULL FI=00 [A]" + tag, s0Full, false, snMatches, fi00, "A", {"A"}, s0Full, ""});
        cases.push_back({"S0_FULL FI=00 [A,B]" + tag,
                         s0Full,
                         false,
                         snMatches,
                         fi00,
                         "AB",
                         {"A", "B"},
                         s0Full,
                         ""});
        cases.push_back(
            {"S0_FULL FI=01 [A]" + tag, s0Full, false, snMatches, fi01, "A", {}, siSf, "A"});
        cases.push_back(
            {"S0_FULL FI=01 [A,B]" + tag, s0Full, false, snMatches, fi01, "AB", {"A"}, siSf, "B"});
        cases.push_back(
            {"S0_FULL FI=10 [A]" + tag, s0Full, false, snMatches, fi10, "A", {}, s0Full, ""});
        cases.push_back(
            {"S0_FULL FI=10 [A,B]" + tag, s0Full, false, snMatches, fi10, "AB", {"B"}, s0Full, ""});
        cases.push_back(
            {"S0_FULL FI=11 [A]" + tag, s0Full, false, snMatches, fi11, "A", {}, s0Full, ""});
        cases.push_back(
            {"S0_FULL FI=11 [A,B]" + tag, s0Full, false, snMatches, fi11, "AB", {}, siSf, "B"});
        cases.push_back({"S0_FULL FI=11 [A,B,C]" + tag,
                         s0Full,
                         false,
                         snMatches,
                         fi11,
                         "ABC",
                         {"B"},
                         siSf,
                         "C"});
    }

    // WAITING_SI_SF, in-sequence continuation (FI = 10/11): the held S0 is
    // completed with the first segment.
    cases.push_back(
        {"SI_SF FI=10 [A] continuation", siSf, true, true, fi10, "A", {"SA"}, s0Full, ""});
    cases.push_back(
        {"SI_SF FI=10 [A,B] continuation", siSf, true, true, fi10, "AB", {"SA", "B"}, s0Full, ""});
    cases.push_back({"SI_SF FI=11 [A] continuation", siSf, true, true, fi11, "A", {}, siSf, "SA"});
    cases.push_back(
        {"SI_SF FI=11 [A,B] continuation", siSf, true, true, fi11, "AB", {"SA"}, siSf, "B"});
    cases.push_back({"SI_SF FI=11 [A,B,C] continuation",
                     siSf,
                     true,
                     true,
                     fi11,
                     "ABC",
                     {"SA", "B"},
                     siSf,
                     "C"});

    // WAITING_SI_SF, in-sequence SN but the PDU starts a new SDU (FI = 00/01):
    // the SN aliased (handover epoch mix or >= 1023 losses). The stale S0 must be
    // discarded and the new SDUs delivered intact -- this used to abort with
    // "INTERNAL ERROR: We are in the WAITING_SI_SF state and no packet loss ...".
    cases.push_back(
        {"SI_SF FI=00 [A] aliased SN resync", siSf, true, true, fi00, "A", {"A"}, s0Full, ""});
    cases.push_back({"SI_SF FI=00 [A,B] aliased SN resync",
                     siSf,
                     true,
                     true,
                     fi00,
                     "AB",
                     {"A", "B"},
                     s0Full,
                     ""});
    cases.push_back(
        {"SI_SF FI=01 [A] aliased SN resync", siSf, true, true, fi01, "A", {}, siSf, "A"});
    cases.push_back(
        {"SI_SF FI=01 [A,B] aliased SN resync", siSf, true, true, fi01, "AB", {"A"}, siSf, "B"});

    // WAITING_SI_SF, out-of-sequence SN (loss): the held S0 can never be
    // completed and must be discarded; leading continuation segments of the lost
    // SDU are discarded too.
    cases.push_back({"SI_SF FI=00 [A] loss", siSf, true, false, fi00, "A", {"A"}, s0Full, ""});
    cases.push_back(
        {"SI_SF FI=00 [A,B] loss", siSf, true, false, fi00, "AB", {"A", "B"}, s0Full, ""});
    cases.push_back({"SI_SF FI=01 [A] loss", siSf, true, false, fi01, "A", {}, siSf, "A"});
    cases.push_back({"SI_SF FI=01 [A,B] loss", siSf, true, false, fi01, "AB", {"A"}, siSf, "B"});
    cases.push_back({"SI_SF FI=10 [A] loss", siSf, true, false, fi10, "A", {}, s0Full, ""});
    cases.push_back({"SI_SF FI=10 [A,B] loss", siSf, true, false, fi10, "AB", {"B"}, s0Full, ""});
    cases.push_back({"SI_SF FI=11 [A] loss", siSf, true, false, fi11, "A", {}, s0Full, ""});
    cases.push_back({"SI_SF FI=11 [A,B] loss", siSf, true, false, fi11, "AB", {}, siSf, "B"});
    cases.push_back({"SI_SF FI=11 [A,B,C] loss", siSf, true, false, fi11, "ABC", {"B"}, siSf, "C"});

    // WAITING_SI_SF with no held S0 (defensive: inconsistent entry state). The
    // PDU must be resynchronised, salvaging any complete SDUs it carries instead
    // of dropping the whole PDU.
    cases.push_back({"SI_SF no S0 FI=00 [A]", siSf, false, true, fi00, "A", {"A"}, s0Full, ""});
    cases.push_back({"SI_SF no S0 FI=01 [A]", siSf, false, true, fi01, "A", {}, siSf, "A"});
    cases.push_back({"SI_SF no S0 FI=10 [A,B]", siSf, false, true, fi10, "AB", {"B"}, s0Full, ""});
    cases.push_back({"SI_SF no S0 FI=11 [A,B]", siSf, false, true, fi11, "AB", {}, siSf, "B"});

    for (const auto& c : cases)
    {
        RunCase(c);
    }

    Simulator::Destroy();
}

/**
 * @ingroup tests
 *
 * @brief Regression test for the RLC-UM assert after handover (SN epoch mixing).
 *
 * At handover both RLC entities are destroyed and re-created (nr-ue-rrc clears the
 * DRB map; the target gNB builds fresh entities), so the transmitter restarts at
 * SN=0 -- but a PDU of the old SN epoch still in flight can land in the fresh
 * receiving entity. When VR(UR) later sweeps past its SN, the stale PDU is
 * delivered exactly at the aliased position: its SN equals m_expectedSeqNumber, so
 * the reassembler believes no loss occurred while it is mid-SDU, and the stale
 * PDU's FI (a new SDU boundary) used to trip
 * "INTERNAL ERROR: We are in the WAITING_SI_SF state and no packet loss has
 * occurred". This test replays that sequence through DoReceivePdu and verifies the
 * receiver instead resynchronises: the un-completable partial SDU is discarded, all
 * complete SDUs (including the stale one) are delivered intact, the late true
 * continuation is discarded by the reception window, and reception continues
 * normally afterwards.
 */
class NrRlcUmSnAliasResyncTestCase : public NrRlcTestCaseBase
{
  public:
    NrRlcUmSnAliasResyncTestCase()
        : NrRlcTestCaseBase("Test RLC UM RX: stale PDU aliasing the expected SN after handover")
    {
    }

  private:
    void DoRun() override;
};

void
NrRlcUmSnAliasResyncTestCase::DoRun()
{
    const uint8_t fi00 = NrRlcHeader::FIRST_BYTE | NrRlcHeader::LAST_BYTE;
    const uint8_t fi01 = NrRlcHeader::FIRST_BYTE | NrRlcHeader::NO_LAST_BYTE;
    const uint8_t fi10 = NrRlcHeader::NO_FIRST_BYTE | NrRlcHeader::LAST_BYTE;

    Ptr<NrRlcUm> rlc = CreateObject<NrRlcUm>();
    NrRlcTestSduSink sink;
    rlc->SetNrRlcSapUser(sink.GetSapUser());

    auto receive = [&rlc](uint16_t sn, uint8_t framingInfo, const std::string& letters) {
        DeliverUmPdu(rlc, MakeUmPdu(sn, framingInfo, MakePayload(letters)), Simulator::Now());
    };

    // A stale PDU of the old SN epoch (a complete SDU, sent as SN=2 before its
    // transmitter was destroyed) reaches the freshly created receiving entity
    // first: SN=2 falls outside the initial reordering window [512, 0), so it
    // slides VR(UH) to 3 and stays buffered, waiting for SN=0..1.
    receive(2, fi00, "S");
    NS_TEST_ASSERT_MSG_EQ(sink.m_sdus.size(), 0, "the stale PDU alone must not be delivered");

    // New-epoch traffic starts at SN=0: a complete SDU, delivered immediately.
    receive(0, fi00, "A");

    // New-epoch SN=1 opens a new SDU (FI=01); its continuation, the new-epoch
    // SN=2, was lost in the handover interruption. Advancing VR(UR) past the
    // buffered stale SN=2 delivers the stale PDU exactly at the aliased position:
    // SN == m_expectedSeqNumber ("no loss") while mid-SDU, with FI=00. This used
    // to abort the simulation; the receiver must instead discard the partial SDU
    // head and deliver the stale complete SDU intact.
    receive(1, fi01, "B");

    NS_TEST_ASSERT_MSG_EQ(sink.m_sdus.size(), 2, "SDUs 'A' and stale 'S' must have been delivered");
    NS_TEST_ASSERT_MSG_EQ(SduMatches(sink.m_sdus[0], "A"),
                          true,
                          "the first new-epoch SDU must be delivered intact");
    NS_TEST_ASSERT_MSG_EQ(SduMatches(sink.m_sdus[1], "S"),
                          true,
                          "the stale SDU must be delivered intact, not concatenated with the "
                          "discarded partial SDU head");
    NS_TEST_ASSERT_MSG_EQ((int)rlc->m_reassemblingState,
                          (int)NrRlcUm::WAITING_S0_FULL,
                          "the resynchronisation must discard the un-completable partial SDU");
    NS_TEST_ASSERT_MSG_EQ((rlc->m_keepS0 == nullptr),
                          true,
                          "no partial SDU may be held after the resynchronisation");

    // The true new-epoch continuation of SN=1 arrives late: its SN=2 is now below
    // VR(UR), so the reception window discards it silently.
    receive(2, fi10, "C");
    NS_TEST_ASSERT_MSG_EQ(sink.m_sdus.size(), 2, "the late duplicate SN must be discarded");

    // Reception continues normally afterwards.
    receive(3, fi00, "C");
    NS_TEST_ASSERT_MSG_EQ(sink.m_sdus.size(), 3, "reception must continue after resynchronising");
    NS_TEST_ASSERT_MSG_EQ(SduMatches(sink.m_sdus[2], "C"),
                          true,
                          "post-resynchronisation SDUs must be delivered intact");

    Simulator::Destroy();
}

/**
 * @ingroup tests
 *
 * @brief Test of the Tx buffer overflow path in NrRlcUm::DoTransmitPdcpPdu.
 *
 * When storing an incoming RLC SDU would exceed MaxTxBufferSize, the SDU must be
 * dropped (TxDrop trace fired) and the Tx buffer left untouched.
 */
class NrRlcUmTxBufferOverflowTestCase : public NrRlcTestCaseBase
{
  public:
    NrRlcUmTxBufferOverflowTestCase()
        : NrRlcTestCaseBase("Test RLC UM TX: SDU overflowing MaxTxBufferSize is dropped")
    {
    }

  private:
    void DoRun() override;
};

void
NrRlcUmTxBufferOverflowTestCase::DoRun()
{
    Ptr<NrRlcUm> rlc = CreateObject<NrRlcUm>();
    rlc->SetAttribute("MaxTxBufferSize", UintegerValue(250));
    rlc->SetAttribute("EnablePdcpDiscarding", BooleanValue(false));

    Ptr<NrTestMac> mac = CreateObject<NrTestMac>();
    mac->SetRlcHeaderType(NrTestMac::UM_RLC_HEADER);
    rlc->SetNrMacSapProvider(mac->GetNrMacSapProvider());
    mac->SetNrMacSapUser(rlc->GetNrMacSapUser());
    rlc->TraceConnectWithoutContext("TxDrop", MakeCallback(&NrRlcTestCaseBase::HandleTxDrop, this));

    rlc->DoTransmitPdcpPdu(Create<Packet>(100)); // stored (100 <= 250)
    rlc->DoTransmitPdcpPdu(Create<Packet>(100)); // stored (200 <= 250)
    rlc->DoTransmitPdcpPdu(Create<Packet>(100)); // 300 > 250: must be dropped

    NS_TEST_ASSERT_MSG_EQ(rlc->m_txBuffer.size(),
                          2,
                          "the overflowing SDU must not be enqueued in the Tx buffer");
    NS_TEST_ASSERT_MSG_EQ(rlc->m_txBufferSize,
                          200,
                          "the Tx buffer size must only account for the stored SDUs");
    NS_TEST_ASSERT_MSG_EQ(m_dropCount, 1, "the overflowing SDU must fire the TxDrop trace once");

    // MaxTxBufferSize = 0 means an unlimited buffer (consistent with NrRlcAm and
    // NrRlcTm): nothing is dropped.
    Ptr<NrRlcUm> unlimitedRlc = CreateObject<NrRlcUm>();
    unlimitedRlc->SetAttribute("MaxTxBufferSize", UintegerValue(0));
    unlimitedRlc->SetAttribute("EnablePdcpDiscarding", BooleanValue(false));
    unlimitedRlc->SetNrMacSapProvider(mac->GetNrMacSapProvider());
    for (uint32_t i = 0; i < 3; i++)
    {
        unlimitedRlc->DoTransmitPdcpPdu(Create<Packet>(1000));
    }
    NS_TEST_ASSERT_MSG_EQ(unlimitedRlc->m_txBuffer.size(),
                          3,
                          "MaxTxBufferSize = 0 must mean an unlimited Tx buffer");

    Simulator::Destroy();
}

/**
 * @ingroup tests
 *
 * @brief Test of the packet-delay-budget variant of the PDCP discard path.
 *
 * With DiscardTimerMs = 0, NrRlcUm::DoTransmitPdcpPdu must fall back to the
 * packet delay budget as the discard timer, dropping an incoming SDU when the
 * head-of-line delay exceeds it.
 */
class NrRlcUmTxDelayBudgetDiscardTestCase : public NrRlcTestCaseBase
{
  public:
    NrRlcUmTxDelayBudgetDiscardTestCase()
        : NrRlcTestCaseBase("Test RLC UM TX: PDCP discard falls back to the packet delay budget")
    {
    }

  private:
    void DoRun() override;
};

void
NrRlcUmTxDelayBudgetDiscardTestCase::DoRun()
{
    Ptr<NrRlcUm> rlc = CreateObject<NrRlcUm>();
    rlc->SetAttribute("EnablePdcpDiscarding", BooleanValue(true));
    rlc->SetAttribute("DiscardTimerMs", UintegerValue(0)); // fall back to the delay budget
    rlc->SetPacketDelayBudgetMs(10);

    Ptr<NrTestMac> mac = CreateObject<NrTestMac>();
    mac->SetRlcHeaderType(NrTestMac::UM_RLC_HEADER);
    rlc->SetNrMacSapProvider(mac->GetNrMacSapProvider());
    mac->SetNrMacSapUser(rlc->GetNrMacSapUser());
    rlc->TraceConnectWithoutContext("TxDrop", MakeCallback(&NrRlcTestCaseBase::HandleTxDrop, this));

    // t = 0 ms: first SDU is stored (Tx buffer is empty, so head-of-line delay is 0).
    Simulator::Schedule(MilliSeconds(0), [rlc]() { rlc->DoTransmitPdcpPdu(Create<Packet>(100)); });

    // t = 20 ms: head-of-line delay (20 ms) exceeds the 10 ms packet delay budget,
    // so the second SDU must be discarded.
    Simulator::Schedule(MilliSeconds(20), [rlc]() { rlc->DoTransmitPdcpPdu(Create<Packet>(200)); });

    Simulator::Run();

    NS_TEST_ASSERT_MSG_EQ(m_dropCount,
                          1,
                          "the SDU exceeding the packet delay budget must fire the TxDrop trace "
                          "exactly once");

    Simulator::Destroy();
}

/**
 * @ingroup tests
 *
 * @brief Test of the Tx opportunity edge cases in NrRlcUm::DoNotifyTxOpportunity.
 *
 * A transmission opportunity of 2 bytes or fewer cannot fit any data beyond the
 * fixed UM header and must not produce a PDU, and neither must an opportunity
 * granted while the Tx buffer is empty. A subsequent valid opportunity must then
 * produce a PDU normally.
 */
class NrRlcUmTxOpportunityEdgeTestCase : public NrRlcTestCaseBase
{
  public:
    NrRlcUmTxOpportunityEdgeTestCase()
        : NrRlcTestCaseBase("Test RLC UM TX: tiny and empty-buffer Tx opportunities produce no PDU")
    {
    }

  private:
    void DoRun() override;
};

void
NrRlcUmTxOpportunityEdgeTestCase::DoRun()
{
    Ptr<NrRlcUm> rlc = CreateObject<NrRlcUm>();

    Ptr<NrTestMac> mac = CreateObject<NrTestMac>();
    mac->SetRlcHeaderType(NrTestMac::UM_RLC_HEADER);
    rlc->SetNrMacSapProvider(mac->GetNrMacSapProvider());
    mac->SetNrMacSapUser(rlc->GetNrMacSapUser());
    rlc->TraceConnectWithoutContext("TxPDU", MakeCallback(&NrRlcTestCaseBase::HandleTxPdu, this));

    // Opportunity while the Tx buffer is empty: no PDU.
    GrantTxOpportunities(rlc, 10, 1);
    NS_TEST_ASSERT_MSG_EQ(m_pduSizes.size(),
                          0,
                          "an opportunity with an empty Tx buffer must not produce a PDU");

    // Opportunity too small for the 2-byte fixed header plus data: no PDU, and the
    // buffered SDU must remain queued.
    rlc->DoTransmitPdcpPdu(Create<Packet>(50));
    GrantTxOpportunities(rlc, 2, 1);
    NS_TEST_ASSERT_MSG_EQ(m_pduSizes.size(),
                          0,
                          "an opportunity of 2 bytes or fewer must not produce a PDU");
    NS_TEST_ASSERT_MSG_EQ(rlc->m_txBuffer.size(),
                          1,
                          "the SDU must remain queued after a too-small opportunity");

    // Valid opportunity: the whole SDU fits, one PDU of 50 + 2 header bytes.
    GrantTxOpportunities(rlc, 100, 1);
    NS_TEST_ASSERT_MSG_EQ(m_pduSizes.size(), 1, "a valid opportunity must produce one PDU");
    NS_TEST_ASSERT_MSG_EQ(m_pduSizes.at(0), 52, "the PDU must carry the SDU plus the fixed header");
    NS_TEST_ASSERT_MSG_EQ(rlc->m_txBuffer.size(), 0, "the Tx buffer must be drained");

    Simulator::Destroy();
}

/**
 * @ingroup tests
 *
 * @brief Test of the 2047-byte length-indicator exception in DoNotifyTxOpportunity.
 *
 * The UM length indicator is an 11-bit field, so a data field element larger than
 * 2047 bytes can only be mapped to the end of the data field (TS 36.322): no
 * further SDU may be concatenated after it, even if the transmission opportunity
 * has room left.
 */
class NrRlcUmTxLargeSduTestCase : public NrRlcTestCaseBase
{
  public:
    NrRlcUmTxLargeSduTestCase()
        : NrRlcTestCaseBase("Test RLC UM TX: SDU larger than 2047 bytes ends the data field")
    {
    }

  private:
    void DoRun() override;
};

void
NrRlcUmTxLargeSduTestCase::DoRun()
{
    Ptr<NrRlcUm> rlc = CreateObject<NrRlcUm>();

    Ptr<NrTestMac> mac = CreateObject<NrTestMac>();
    mac->SetRlcHeaderType(NrTestMac::UM_RLC_HEADER);
    rlc->SetNrMacSapProvider(mac->GetNrMacSapProvider());
    mac->SetNrMacSapUser(rlc->GetNrMacSapUser());
    rlc->TraceConnectWithoutContext("TxPDU", MakeCallback(&NrRlcTestCaseBase::HandleTxPdu, this));

    rlc->DoTransmitPdcpPdu(Create<Packet>(3000));
    rlc->DoTransmitPdcpPdu(Create<Packet>(100));

    // The opportunity has room for both SDUs, but the > 2047-byte SDU cannot be
    // followed by a length indicator: the first PDU must carry only the large SDU.
    GrantTxOpportunities(rlc, 5000, 1);
    NS_TEST_ASSERT_MSG_EQ(m_pduSizes.size(), 1, "one PDU must be produced");
    NS_TEST_ASSERT_MSG_EQ(m_pduSizes.at(0),
                          3002,
                          "the > 2047-byte SDU must be sent alone (no concatenation after it)");

    // The second SDU goes out in the next opportunity.
    GrantTxOpportunities(rlc, 200, 1);
    NS_TEST_ASSERT_MSG_EQ(m_pduSizes.size(), 2, "the second SDU must go out separately");
    NS_TEST_ASSERT_MSG_EQ(m_pduSizes.at(1), 102, "the second PDU must carry the small SDU");

    Simulator::Destroy();
}

/**
 * @ingroup tests
 *
 * @brief Test of the t-Reordering timer lifecycle in NrRlcUm::DoReceivePdu.
 *
 * Covers the full timer state machine with in-order delivery (OutOfOrderDelivery
 * disabled): a reception gap starts the timer and holds back later SNs; filling
 * the gap delivers the held SDUs in SN order and stops the timer (VR(UX) <=
 * VR(UR)); an unfilled gap makes the timer expire, which advances VR(UR) past the
 * hole, delivers the buffered SDUs beyond it, and restarts the timer while a
 * further gap remains (VR(UH) > VR(UR)).
 */
class NrRlcUmReorderingTimerTestCase : public NrRlcTestCaseBase
{
  public:
    NrRlcUmReorderingTimerTestCase()
        : NrRlcTestCaseBase("Test RLC UM RX: t-Reordering start, stop and expiry delivery")
    {
    }

  private:
    void DoRun() override;
};

void
NrRlcUmReorderingTimerTestCase::DoRun()
{
    const uint8_t fi00 = NrRlcHeader::FIRST_BYTE | NrRlcHeader::LAST_BYTE;

    Ptr<NrRlcUm> rlc = CreateObject<NrRlcUm>();
    NrRlcTestSduSink sink;
    rlc->SetNrRlcSapUser(sink.GetSapUser());

    // Part 1: a gap (missing SN=1) starts t-Reordering and holds back SN=2; the
    // late SN=1 fills the gap, delivers SN=1 and SN=2 in order and stops the timer.
    // The SDU payload size (20 + SN) identifies each SDU in the delivery order.
    Simulator::Schedule(MilliSeconds(0), [rlc]() { DeliverPdu(rlc, 0, fi00, 20); });
    Simulator::Schedule(MilliSeconds(1), [rlc]() { DeliverPdu(rlc, 2, fi00, 22); });
    Simulator::Schedule(MilliSeconds(2), [this, rlc, &sink]() {
        NS_TEST_ASSERT_MSG_EQ(sink.m_sdus.size(),
                              1,
                              "SN=2 must be held back while SN=1 is missing");
        NS_TEST_ASSERT_MSG_EQ(rlc->m_reorderingTimer.IsPending(),
                              true,
                              "a reception gap must start t-Reordering");
    });
    Simulator::Schedule(MilliSeconds(5), [rlc]() { DeliverPdu(rlc, 1, fi00, 21); });
    Simulator::Schedule(MilliSeconds(6), [this, rlc, &sink]() {
        NS_TEST_ASSERT_MSG_EQ(sink.m_sdus.size(), 3, "filling the gap must deliver the held SDUs");
        NS_TEST_ASSERT_MSG_EQ(sink.m_sdus.at(1)->GetSize(), 21, "SN=1 must be delivered second");
        NS_TEST_ASSERT_MSG_EQ(sink.m_sdus.at(2)->GetSize(), 22, "SN=2 must be delivered third");
        NS_TEST_ASSERT_MSG_EQ(rlc->m_reorderingTimer.IsPending(),
                              false,
                              "filling the gap must stop t-Reordering (VR(UX) <= VR(UR))");
    });

    // Part 2: gaps at SN=3 and SN=5 are never filled. The first expiry (at
    // t = 10 ms + 100 ms) must advance VR(UR) past the SN=3 hole, deliver SN=4 and
    // restart the timer for the SN=5 hole; the second expiry delivers SN=6.
    Simulator::Schedule(MilliSeconds(10), [rlc]() { DeliverPdu(rlc, 4, fi00, 24); });
    Simulator::Schedule(MilliSeconds(12), [rlc]() { DeliverPdu(rlc, 6, fi00, 26); });
    Simulator::Schedule(MilliSeconds(150), [this, rlc, &sink]() {
        NS_TEST_ASSERT_MSG_EQ(sink.m_sdus.size(),
                              4,
                              "the first expiry must deliver the SDU beyond the hole");
        NS_TEST_ASSERT_MSG_EQ(sink.m_sdus.at(3)->GetSize(), 24, "SN=4 must be delivered fourth");
        NS_TEST_ASSERT_MSG_EQ(rlc->m_reorderingTimer.IsPending(),
                              true,
                              "t-Reordering must restart while a further gap remains");
    });
    Simulator::Schedule(MilliSeconds(250), [this, rlc, &sink]() {
        NS_TEST_ASSERT_MSG_EQ(sink.m_sdus.size(),
                              5,
                              "the second expiry must deliver the remaining SDU");
        NS_TEST_ASSERT_MSG_EQ(sink.m_sdus.at(4)->GetSize(), 26, "SN=6 must be delivered fifth");
        NS_TEST_ASSERT_MSG_EQ(rlc->m_reorderingTimer.IsPending(),
                              false,
                              "t-Reordering must not restart once VR(UR) reaches VR(UH)");
    });

    Simulator::Run();

    NS_TEST_ASSERT_MSG_EQ(sink.m_sdus.size(), 5, "all receivable SDUs must have been delivered");
    NS_TEST_ASSERT_MSG_EQ(rlc->m_rxBuffer.size(), 0, "the reception buffer must be drained");

    Simulator::Destroy();
}

/**
 * @ingroup tests
 *
 * @brief Test of the OutOfOrderDelivery mode of NrRlcUm.
 *
 * With rlc-OutOfOrderDelivery configured, a PDU that slides the reordering window
 * is reassembled and delivered immediately without waiting for the missing SNs,
 * and a late in-window PDU is still delivered (on t-Reordering expiry) rather
 * than lost or duplicated.
 */
class NrRlcUmOutOfOrderDeliveryTestCase : public NrRlcTestCaseBase
{
  public:
    NrRlcUmOutOfOrderDeliveryTestCase()
        : NrRlcTestCaseBase("Test RLC UM RX: out-of-order delivery bypasses reordering")
    {
    }

  private:
    void DoRun() override;
};

void
NrRlcUmOutOfOrderDeliveryTestCase::DoRun()
{
    const uint8_t fi00 = NrRlcHeader::FIRST_BYTE | NrRlcHeader::LAST_BYTE;

    Ptr<NrRlcUm> rlc = CreateObject<NrRlcUm>();
    rlc->SetAttribute("OutOfOrderDelivery", BooleanValue(true));
    NrRlcTestSduSink sink;
    rlc->SetNrRlcSapUser(sink.GetSapUser());

    // SN=0 and SN=2 are delivered immediately, without waiting for the missing
    // SN=1. The SDU payload size (20 + SN) identifies each SDU.
    Simulator::Schedule(MilliSeconds(0), [rlc]() { DeliverPdu(rlc, 0, fi00, 20); });
    Simulator::Schedule(MilliSeconds(1), [rlc]() { DeliverPdu(rlc, 2, fi00, 22); });
    Simulator::Schedule(MilliSeconds(2), [this, &sink]() {
        NS_TEST_ASSERT_MSG_EQ(sink.m_sdus.size(),
                              2,
                              "out-of-order mode must deliver without waiting for missing SNs");
        NS_TEST_ASSERT_MSG_EQ(sink.m_sdus.at(1)->GetSize(),
                              22,
                              "SN=2 must be delivered before the missing SN=1");
    });

    // A duplicate of the out-of-order-delivered SN=2 (e.g. a HARQ duplicate)
    // arrives after its SN slid inside the reordering window. It used to be
    // re-accepted (the delivered PDU was no longer buffered) and delivered to PDCP
    // a second time; it must be recognised and discarded.
    Simulator::Schedule(MilliSeconds(3), [rlc]() { DeliverPdu(rlc, 2, fi00, 22); });
    Simulator::Schedule(MilliSeconds(4), [this, &sink]() {
        NS_TEST_ASSERT_MSG_EQ(sink.m_sdus.size(),
                              2,
                              "a duplicate of an out-of-order-delivered PDU must be discarded");
    });

    // The late SN=1 falls inside the reordering window; it is buffered and
    // delivered when t-Reordering expires, exactly once.
    Simulator::Schedule(MilliSeconds(5), [rlc]() { DeliverPdu(rlc, 1, fi00, 21); });

    Simulator::Run();

    NS_TEST_ASSERT_MSG_EQ(sink.m_sdus.size(), 3, "the late SDU must be delivered exactly once");
    NS_TEST_ASSERT_MSG_EQ(sink.m_sdus.at(2)->GetSize(), 21, "the late SN=1 must be delivered last");
    NS_TEST_ASSERT_MSG_EQ(rlc->m_rxBuffer.size(), 0, "the reception buffer must be drained");

    // Part 2: tombstone at the 10-bit SN wrap. SN=0 is delivered out of order
    // (leaving a tombstone at SN 0), then SNs 1022 and 1023 complete the sequence
    // up to the wrap: VR(UR) must advance across the boundary and consume the
    // tombstone. A late duplicate of SN=0 must then be discarded -- the advance
    // used to stop at the wrap (raw counter reaching 1024), stranding the
    // tombstone at VR(UR)=0, where the duplicate hit the "already exists in the
    // RX buffer" assert (or was re-delivered with asserts disabled).
    {
        Ptr<NrRlcUm> wrapRlc = CreateObject<NrRlcUm>();
        wrapRlc->SetAttribute("OutOfOrderDelivery", BooleanValue(true));
        NrRlcTestSduSink wrapSink;
        wrapRlc->SetNrRlcSapUser(wrapSink.GetSapUser());
        wrapRlc->m_vrUr = 1022;
        wrapRlc->m_vrUx = 1022;
        wrapRlc->m_vrUh = 1022;
        wrapRlc->m_expectedSeqNumber = 1022;

        DeliverPdu(wrapRlc, 0, fi00, 20); // out of order: delivered and tombstoned
        NS_TEST_ASSERT_MSG_EQ(wrapSink.m_sdus.size(), 1, "SN=0 must be delivered out of order");
        DeliverPdu(wrapRlc, 1022, fi00, 21);
        DeliverPdu(wrapRlc, 1023, fi00, 22); // VR(UR) crosses the wrap here
        NS_TEST_ASSERT_MSG_EQ(wrapSink.m_sdus.size(),
                              3,
                              "SNs 1022 and 1023 must be delivered in order");
        NS_TEST_ASSERT_MSG_EQ(wrapRlc->m_rxBuffer.size(),
                              0,
                              "the tombstone at SN=0 must be consumed across the wrap");
        NS_TEST_ASSERT_MSG_EQ(wrapRlc->m_vrUr.GetValue(),
                              1,
                              "VR(UR) must advance across the wrap boundary");

        DeliverPdu(wrapRlc, 0, fi00, 20); // late duplicate of the tombstoned SN
        NS_TEST_ASSERT_MSG_EQ(wrapSink.m_sdus.size(),
                              3,
                              "the late duplicate of SN=0 must be discarded");
        Simulator::Run();
        NS_TEST_ASSERT_MSG_EQ(wrapSink.m_sdus.size(),
                              3,
                              "no further deliveries after the timers drain");
    }

    Simulator::Destroy();
}

/**
 * @ingroup tests
 *
 * @brief Regression test for the severely-delayed-PDU guard of the reordering window.
 *
 * A PDU delayed by more than UM_Window_Size SNs (e.g. a last-round HARQ
 * retransmission at high rate) aliases in the 10-bit SN space to an out-of-window
 * "new" position: the receiver used to slide the reordering window back onto it
 * (VR(UH) = SN + 1), wholesale-discarding live in-window traffic until the stream
 * caught up again. The newest transmission always carries the newest sender
 * timestamp, so such a PDU must be discarded instead, while genuinely new
 * out-of-window traffic must still slide the window forward.
 */
class NrRlcUmLatePduTestCase : public NrRlcTestCaseBase
{
  public:
    NrRlcUmLatePduTestCase()
        : NrRlcTestCaseBase("Test RLC UM RX: severely delayed PDU must not rewind the window")
    {
    }

  private:
    void DoRun() override;
};

void
NrRlcUmLatePduTestCase::DoRun()
{
    const uint8_t fi00 = NrRlcHeader::FIRST_BYTE | NrRlcHeader::LAST_BYTE;

    Ptr<NrRlcUm> rlc = CreateObject<NrRlcUm>();
    NrRlcTestSduSink sink;
    rlc->SetNrRlcSapUser(sink.GetSapUser());

    // Receiver mid-stream, fully in sequence at SN=600.
    rlc->m_vrUr = 600;
    rlc->m_vrUx = 600;
    rlc->m_vrUh = 600;
    rlc->m_expectedSeqNumber = 600;

    // In-sequence PDU SN=600, sent at t=10ms: delivered, window advances to 601.
    DeliverPduAt(rlc, 600, fi00, 20, MilliSeconds(10));
    NS_TEST_ASSERT_MSG_EQ(sink.m_sdus.size(), 1, "the in-sequence PDU must be delivered");
    NS_TEST_ASSERT_MSG_EQ(rlc->m_vrUh.GetValue(), 601, "VR(UH) must advance to 601");

    // A severely delayed PDU: SN=50 is more than 512 SNs behind, aliasing as
    // out-of-window "new" traffic, but it was sent (t=2ms) before traffic already
    // received. It must be discarded, not slide the window back onto SN 50.
    DeliverPduAt(rlc, 50, fi00, 22, MilliSeconds(2));
    NS_TEST_ASSERT_MSG_EQ(sink.m_sdus.size(), 1, "the severely delayed PDU must not be delivered");
    NS_TEST_ASSERT_MSG_EQ(rlc->m_vrUh.GetValue(),
                          601,
                          "the reordering window must not move backwards");
    NS_TEST_ASSERT_MSG_EQ(rlc->m_rxBuffer.size(),
                          0,
                          "the severely delayed PDU must not be buffered");

    // Genuinely new out-of-window traffic (fresh timestamp) still slides the window.
    DeliverPduAt(rlc, 700, fi00, 24, MilliSeconds(12));
    NS_TEST_ASSERT_MSG_EQ(rlc->m_vrUh.GetValue(),
                          701,
                          "genuinely new traffic must still slide the window forward");

    Simulator::Run(); // t-Reordering flushes the SDU beyond the SN 601-699 gap
    NS_TEST_ASSERT_MSG_EQ(sink.m_sdus.size(), 2, "the new out-of-window SDU must be delivered");
    NS_TEST_ASSERT_MSG_EQ(sink.m_sdus.at(1)->GetSize(), 24, "SN=700 must be delivered second");

    Simulator::Destroy();
}

/**
 * @ingroup tests
 *
 * @brief End-to-end data-integrity test of the RLC UM transmit and receive entities.
 *
 * A transmitting NrRlcUm segments and concatenates a set of SDUs with distinct,
 * identifiable payloads into PDUs (small transmission opportunities force both). The
 * captured PDUs are then delivered to fresh receiving entities under four conditions
 * -- lossless in order, with two PDUs swapped (HARQ reordering), with one PDU lost,
 * and with a duplicated PDU -- verifying byte-exactly that every delivered SDU equals
 * the transmitted one, deliveries are in order without duplicates, and an SDU with a
 * segment in the lost PDU is never delivered partially or corrupted. The RLC e2e
 * suites only count PDUs; this is where received payload content is checked against
 * transmitted content.
 */
class NrRlcUmDataIntegrityTestCase : public NrRlcTestCaseBase
{
  public:
    NrRlcUmDataIntegrityTestCase()
        : NrRlcTestCaseBase("Test RLC UM: transmitted data is received byte-exact under "
                            "segmentation, reordering, loss and duplication")
    {
    }

  private:
    void DoRun() override;
};

void
NrRlcUmDataIntegrityTestCase::DoRun()
{
    const std::vector<NrRlcSduSpec> txSdus =
        {{'A', 45}, {'B', 52}, {'C', 59}, {'D', 66}, {'E', 73}, {'F', 80}};

    // Transmit side: queue the SDUs and grant small opportunities so the PDUs both
    // segment large SDUs and concatenate small remainders.
    Ptr<NrRlcUm> txRlc = CreateObject<NrRlcUm>();
    NrRlcTestCaptureMac capture;
    txRlc->SetNrMacSapProvider(&capture);
    QueueSdus(txRlc, txSdus);
    GrantTxOpportunities(txRlc, 40, 20);
    const std::vector<Ptr<Packet>>& pdus = capture.m_pdus;

    CheckSegmentationAndConcatenation<NrRlcHeader>(pdus);

    // Scenario 1: all PDUs delivered in order -- every SDU arrives byte-exact.
    {
        Ptr<NrRlcUm> rxRlc = CreateObject<NrRlcUm>();
        NrRlcTestSduSink sink;
        rxRlc->SetNrRlcSapUser(sink.GetSapUser());
        for (const auto& pdu : pdus)
        {
            DeliverPduToRlc(rxRlc, pdu);
        }
        Simulator::Run();
        VerifyDelivered(sink, txSdus, "lossless in-order");
    }

    // Scenario 2: two adjacent PDUs swapped (HARQ processes deliver out of order) --
    // the reordering window restores SN order and every SDU arrives byte-exact.
    {
        Ptr<NrRlcUm> rxRlc = CreateObject<NrRlcUm>();
        NrRlcTestSduSink sink;
        rxRlc->SetNrRlcSapUser(sink.GetSapUser());
        std::vector<Ptr<Packet>> reordered = pdus;
        std::swap(reordered.at(2), reordered.at(3));
        for (const auto& pdu : reordered)
        {
            DeliverPduToRlc(rxRlc, pdu);
        }
        Simulator::Run();
        VerifyDelivered(sink, txSdus, "reordered");
    }

    // Scenario 3: one mid-stream PDU is lost. Every SDU with a segment in the lost
    // PDU must be absent; every other SDU must be delivered byte-exact and in order
    // (after t-Reordering expires and flushes the SDUs beyond the gap). No partial
    // or corrupted SDU may ever reach the upper layer.
    {
        const uint32_t lostIndex = pdus.size() / 2;
        std::set<uint8_t> lostFills = FillsInPdu<NrRlcHeader>(pdus.at(lostIndex));
        std::vector<NrRlcSduSpec> expected;
        for (const auto& sdu : txSdus)
        {
            if (lostFills.count(sdu.first) == 0)
            {
                expected.push_back(sdu);
            }
        }
        NS_TEST_ASSERT_MSG_EQ((expected.size() < txSdus.size()),
                              true,
                              "loss scenario sanity: the lost PDU must carry SDU data");

        Ptr<NrRlcUm> rxRlc = CreateObject<NrRlcUm>();
        NrRlcTestSduSink sink;
        rxRlc->SetNrRlcSapUser(sink.GetSapUser());
        for (uint32_t i = 0; i < pdus.size(); i++)
        {
            if (i == lostIndex)
            {
                continue;
            }
            DeliverPduToRlc(rxRlc, pdus.at(i));
        }
        Simulator::Run(); // let t-Reordering expire and flush the SDUs beyond the gap
        VerifyDelivered(sink, expected, "one PDU lost");
    }

    // Scenario 4: one PDU is duplicated -- the duplicate must be discarded and the
    // delivered sequence must be identical to the lossless one.
    {
        Ptr<NrRlcUm> rxRlc = CreateObject<NrRlcUm>();
        NrRlcTestSduSink sink;
        rxRlc->SetNrRlcSapUser(sink.GetSapUser());
        for (const auto& pdu : pdus)
        {
            DeliverPduToRlc(rxRlc, pdu);
        }
        DeliverPduToRlc(rxRlc, pdus.at(3));
        Simulator::Run();
        VerifyDelivered(sink, txSdus, "duplicated PDU");
    }

    // Scenario 5: SN epoch mixing after a handover, with real transmitters on both
    // sides. An OLD-epoch transmitter (the entity destroyed at handover, created
    // FIRST so it carries the older entity identity) emits three self-contained
    // PDUs; its SN=2 PDU is still in flight when the entities are re-created and
    // reaches the fresh receiver before the NEW-epoch transmitter's traffic. The
    // receiver locks onto the old identity, and the first new-epoch PDU (higher
    // identity) re-establishes the receiving side (TS 38.322 5.1.2: discard all,
    // reset state): the whole new-epoch stream is then delivered in order and
    // byte-exact -- nothing is shadowed, nothing stale is delivered, nothing mixes.
    {
        const std::vector<NrRlcSduSpec> oldSdus = {{'X', 30}, {'Y', 34}, {'Z', 38}};
        Ptr<NrRlcUm> oldTxRlc = CreateObject<NrRlcUm>(); // created first: older identity
        NrRlcTestCaptureMac oldCapture;
        oldTxRlc->SetNrMacSapProvider(&oldCapture);
        TransmitSdusAligned(oldTxRlc, oldSdus, 40);
        NS_TEST_ASSERT_MSG_EQ(oldCapture.m_pdus.size(),
                              3,
                              "epoch-mix sanity: one SDU-aligned PDU per old-epoch SDU");
        Ptr<Packet> stalePdu = oldCapture.m_pdus.at(2); // old-epoch SN=2, carries 'Z'

        Ptr<NrRlcUm> newTxRlc = CreateObject<NrRlcUm>(); // created second: newer identity
        NrRlcTestCaptureMac newCapture;
        newTxRlc->SetNrMacSapProvider(&newCapture);
        QueueSdus(newTxRlc, txSdus);
        GrantTxOpportunities(newTxRlc, 40, 20);

        Ptr<NrRlcUm> rxRlc = CreateObject<NrRlcUm>();
        NrRlcTestSduSink sink;
        rxRlc->SetNrRlcSapUser(sink.GetSapUser());

        // Pre-handover, the receiver operated normally in the old epoch: SN=0 is
        // delivered in order (advancing the window state, so the re-establishment
        // must also reset the sequence-number modulus bases, not just the values),
        // and SN=2 is buffered behind the lost SN=1.
        DeliverPduToRlc(rxRlc, oldCapture.m_pdus.at(0));
        DeliverPduToRlc(rxRlc, stalePdu);
        NS_TEST_ASSERT_MSG_EQ(sink.m_sdus.size(), 1, "the in-order old-epoch SDU is delivered");

        for (const auto& pdu : newCapture.m_pdus)
        {
            DeliverPduToRlc(rxRlc, pdu);
        }
        Simulator::Run();

        // The re-establishment discards the buffered stale PDU; the whole new-epoch
        // stream is delivered in order, everything byte-exact and nothing mixed.
        std::vector<NrRlcSduSpec> expected = {{'X', 30}};
        expected.insert(expected.end(), txSdus.begin(), txSdus.end());
        VerifyDelivered(sink, expected, "epoch mix with re-establishment");
    }

    // Scenario 6: the stale in-flight PDU ends mid-SDU. Without the entity
    // identity check this is the undetectable corruption case: the new-epoch
    // continuation matches the expected SN and framing info, and the receiver
    // concatenates old- and new-epoch bytes into one SDU. With the identity check
    // the stale partial is discarded at re-establishment, so no delivered SDU may
    // ever mix epochs, and the whole new-epoch stream arrives byte-exact.
    {
        const std::vector<NrRlcSduSpec> oldSdus = {{'X', 70}, {'Y', 74}};
        Ptr<NrRlcUm> oldTxRlc = CreateObject<NrRlcUm>(); // created first: older identity
        NrRlcTestCaptureMac oldCapture;
        oldTxRlc->SetNrMacSapProvider(&oldCapture);
        QueueSdus(oldTxRlc, oldSdus);
        GrantTxOpportunities(oldTxRlc, 40, 10);
        NS_TEST_ASSERT_MSG_EQ((oldCapture.m_pdus.size() > 1),
                              true,
                              "epoch-mix sanity: the old-epoch SDUs must be segmented");
        // The second PDU starts mid-'X' and ends mid-'Y': a mid-SDU stale PDU.
        Ptr<Packet> stalePdu = oldCapture.m_pdus.at(1);

        Ptr<NrRlcUm> newTxRlc = CreateObject<NrRlcUm>(); // created second: newer identity
        NrRlcTestCaptureMac newCapture;
        newTxRlc->SetNrMacSapProvider(&newCapture);
        QueueSdus(newTxRlc, txSdus);
        GrantTxOpportunities(newTxRlc, 40, 20);

        Ptr<NrRlcUm> rxRlc = CreateObject<NrRlcUm>();
        NrRlcTestSduSink sink;
        rxRlc->SetNrRlcSapUser(sink.GetSapUser());

        DeliverPduToRlc(rxRlc, stalePdu); // in-flight mid-SDU old-epoch PDU first
        for (const auto& pdu : newCapture.m_pdus)
        {
            DeliverPduToRlc(rxRlc, pdu);
        }
        Simulator::Run();

        // The stale PDU carried only partial SDUs: nothing of the old epoch may be
        // delivered, no SDU may mix epochs, and the new stream arrives complete.
        VerifyDelivered(sink, txSdus, "mid-SDU stale PDU with re-establishment");
    }

    Simulator::Destroy();
}

/**
 * @ingroup tests
 *
 * @brief End-to-end data-integrity test of the out-of-order delivery of NrRlcUm.
 *
 * Out-of-order delivery used to feed the PDUs, in their arrival order, to the in-order
 * reassembly state machine, which takes an SN gap as a loss: every PDU overtaken by
 * another (a HARQ retransmission, routinely) discarded the SDUs segmented across its
 * boundaries, and in system simulations most flows lost data. A transmitting NrRlcUm
 * segments and concatenates SDUs, one of them spanning at least three PDUs, and the
 * captured PDUs are delivered to fresh out-of-order receiving entities in reverse order,
 * with two PDUs swapped, with one PDU lost and with a duplicated PDU. Every SDU must be
 * delivered exactly once and byte-exact as soon as all its segments are received, in any
 * order; an SDU with a segment in the lost PDU must never be delivered, and its other
 * segments must be discarded once the reordering window gives the lost PDU up.
 */
class NrRlcUmOutOfOrderDataIntegrityTestCase : public NrRlcTestCaseBase
{
  public:
    NrRlcUmOutOfOrderDataIntegrityTestCase()
        : NrRlcTestCaseBase("Test RLC UM: out-of-order delivery is byte-exact under "
                            "reordering, loss and duplication")
    {
    }

  private:
    void DoRun() override;

    /**
     * Verify the delivered SDUs are exactly the expected ones, byte-exact and each
     * delivered once, in any order.
     *
     * @param sink the sink holding the delivered SDUs
     * @param expected the expected SDUs
     * @param label scenario label for the assertion messages
     */
    void VerifyDeliveredOnce(const NrRlcTestSduSink& sink,
                             const std::vector<NrRlcSduSpec>& expected,
                             const std::string& label);
};

void
NrRlcUmOutOfOrderDataIntegrityTestCase::VerifyDeliveredOnce(
    const NrRlcTestSduSink& sink,
    const std::vector<NrRlcSduSpec>& expected,
    const std::string& label)
{
    NS_TEST_ASSERT_MSG_EQ(sink.m_sdus.size(),
                          expected.size(),
                          label << ": number of delivered SDUs");
    for (const auto& [fill, size] : expected)
    {
        const auto delivered =
            std::count_if(sink.m_sdus.begin(), sink.m_sdus.end(), [fill, size](const auto& sdu) {
                return IsUniformSdu(sdu, fill, size);
            });
        NS_TEST_ASSERT_MSG_EQ(delivered,
                              1,
                              label << ": SDU '" << (char)fill << "' (" << size
                                    << " bytes) must be delivered once, byte-exact");
    }
}

void
NrRlcUmOutOfOrderDataIntegrityTestCase::DoRun()
{
    // 'C' spans at least three 40-byte PDUs, so that a PDU carries a middle segment only.
    const std::vector<NrRlcSduSpec> txSdus =
        {{'A', 45}, {'B', 52}, {'C', 150}, {'D', 66}, {'E', 73}, {'F', 80}};

    Ptr<NrRlcUm> txRlc = CreateObject<NrRlcUm>();
    NrRlcTestCaptureMac capture;
    txRlc->SetNrMacSapProvider(&capture);
    QueueSdus(txRlc, txSdus);
    GrantTxOpportunities(txRlc, 40, 20);
    const std::vector<Ptr<Packet>>& pdus = capture.m_pdus;
    CheckSegmentationAndConcatenation<NrRlcHeader>(pdus);

    auto newReceiver = [](NrRlcTestSduSink& sink) {
        Ptr<NrRlcUm> rxRlc = CreateObject<NrRlcUm>();
        rxRlc->SetAttribute("OutOfOrderDelivery", BooleanValue(true));
        rxRlc->SetNrRlcSapUser(sink.GetSapUser());
        return rxRlc;
    };

    // Scenario 1: every PDU arrives in reverse SN order. Each SDU is delivered as soon as
    // its last segment arrives, without waiting for t-Reordering.
    {
        NrRlcTestSduSink sink;
        Ptr<NrRlcUm> rxRlc = newReceiver(sink);
        for (auto it = pdus.rbegin(); it != pdus.rend(); ++it)
        {
            DeliverPduToRlc(rxRlc, (*it)->Copy());
        }
        VerifyDeliveredOnce(sink, txSdus, "reverse order, before t-Reordering");
        Simulator::Run();
        VerifyDeliveredOnce(sink, txSdus, "reverse order");
        NS_TEST_ASSERT_MSG_EQ(rxRlc->m_outOfOrderSegments.size(),
                              0,
                              "reverse order: no segment may be left held");
    }

    // Scenario 2: two adjacent PDUs swapped, as a HARQ retransmission overtaken by the
    // next transport block.
    {
        NrRlcTestSduSink sink;
        Ptr<NrRlcUm> rxRlc = newReceiver(sink);
        std::vector<Ptr<Packet>> reordered = pdus;
        std::swap(reordered.at(2), reordered.at(3));
        for (const auto& pdu : reordered)
        {
            DeliverPduToRlc(rxRlc, pdu->Copy());
        }
        VerifyDeliveredOnce(sink, txSdus, "swapped, before t-Reordering");
        Simulator::Run();
        VerifyDeliveredOnce(sink, txSdus, "swapped");
    }

    // Scenario 3: the PDU carrying only a middle segment of 'C' is lost. 'C' must never be
    // delivered, every other SDU must be, and the held segments of 'C' must be discarded
    // once t-Reordering gives the lost PDU up.
    {
        uint32_t lostIndex = pdus.size();
        for (uint32_t i = 0; i < pdus.size(); i++)
        {
            std::set<uint8_t> fills = FillsInPdu<NrRlcHeader>(pdus.at(i));
            if (fills.size() == 1 && fills.count('C') == 1 && i > 0 &&
                FillsInPdu<NrRlcHeader>(pdus.at(i - 1)).count('C') == 1 && i + 1 < pdus.size() &&
                FillsInPdu<NrRlcHeader>(pdus.at(i + 1)).count('C') == 1)
            {
                lostIndex = i;
                break;
            }
        }
        NS_TEST_ASSERT_MSG_LT(lostIndex, pdus.size(), "a PDU must carry a middle segment of 'C'");
        std::vector<NrRlcSduSpec> expected;
        for (const auto& sdu : txSdus)
        {
            if (sdu.first != 'C')
            {
                expected.push_back(sdu);
            }
        }

        NrRlcTestSduSink sink;
        Ptr<NrRlcUm> rxRlc = newReceiver(sink);
        for (auto it = pdus.rbegin(); it != pdus.rend(); ++it)
        {
            if (static_cast<uint32_t>(pdus.rend() - it - 1) != lostIndex)
            {
                DeliverPduToRlc(rxRlc, (*it)->Copy());
            }
        }
        Simulator::Run();
        VerifyDeliveredOnce(sink, expected, "middle segment lost");
        NS_TEST_ASSERT_MSG_EQ(rxRlc->m_outOfOrderSegments.size(),
                              0,
                              "the segments of an SDU that cannot complete must be discarded");
    }

    // Scenario 4: a PDU is duplicated after its SDUs were delivered; nothing may be
    // delivered twice.
    {
        NrRlcTestSduSink sink;
        Ptr<NrRlcUm> rxRlc = newReceiver(sink);
        for (const auto& pdu : pdus)
        {
            DeliverPduToRlc(rxRlc, pdu->Copy());
        }
        DeliverPduToRlc(rxRlc, pdus.at(3)->Copy());
        Simulator::Run();
        VerifyDeliveredOnce(sink, txSdus, "duplicated PDU");
    }

    Simulator::Destroy();
}

class NrRlcUmTestSuite : public TestSuite
{
  public:
    NrRlcUmTestSuite()
        : TestSuite("nr-test-rlc-um-rx", Type::SYSTEM)
    {
        AddTestCase(new NrRlcUmTestCase(), Duration::QUICK);
        AddTestCase(new NrRlcUmReorderingDiscardTestCase(), Duration::QUICK);
        AddTestCase(new NrRlcUmTxPdcpDiscardTestCase(), Duration::QUICK);
        AddTestCase(new NrRlcUmReassembleGapTestCase(), Duration::QUICK);
        AddTestCase(new NrRlcUmReassemblyMatrixTestCase(), Duration::QUICK);
        AddTestCase(new NrRlcUmSnAliasResyncTestCase(), Duration::QUICK);
        AddTestCase(new NrRlcUmTxBufferOverflowTestCase(), Duration::QUICK);
        AddTestCase(new NrRlcUmTxDelayBudgetDiscardTestCase(), Duration::QUICK);
        AddTestCase(new NrRlcUmTxOpportunityEdgeTestCase(), Duration::QUICK);
        AddTestCase(new NrRlcUmTxLargeSduTestCase(), Duration::QUICK);
        AddTestCase(new NrRlcUmReorderingTimerTestCase(), Duration::QUICK);
        AddTestCase(new NrRlcUmOutOfOrderDeliveryTestCase(), Duration::QUICK);
        AddTestCase(new NrRlcUmLatePduTestCase(), Duration::QUICK);
        AddTestCase(new NrRlcUmDataIntegrityTestCase(), Duration::QUICK);
        AddTestCase(new NrRlcUmOutOfOrderDataIntegrityTestCase(), Duration::QUICK);
    }
};

static NrRlcUmTestSuite g_nrRlcUmTestSuite;

} // namespace ns3
