// Copyright (c) 2026 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only

/**
 * @ingroup test
 * @file nr-test-rlc-tm.cc
 *
 * @brief Test suite `nr-test-rlc-tm`: white-box tests of the NrRlcTm transparent mode
 * entity, which previously had no test coverage at all despite carrying SRB0/CCCH on both
 * the gNB and the UE. The cases verify that SDUs are queued and transmitted unmodified
 * (no RLC header); that a transmission opportunity smaller than the head-of-line SDU or
 * granted with an empty buffer produces no PDU; that an SDU overflowing MaxTxBufferSize
 * is dropped and fires the TxDrop trace; and that received PDUs are delivered to the
 * upper layer unmodified.
 */

#include "nr-rlc-test-utils.h"
#include "nr-test-entities.h"

#include "ns3/log.h"
#include "ns3/nr-rlc-sap.h"
#include "ns3/nr-rlc-tm.h"
#include "ns3/packet.h"
#include "ns3/simulator.h"
#include "ns3/uinteger.h"

namespace ns3
{

/**
 * @ingroup tests
 *
 * @brief Test of the NrRlcTm transmit and receive paths.
 */
class NrRlcTmTestCase : public NrRlcTestCaseBase
{
  public:
    NrRlcTmTestCase()
        : NrRlcTestCaseBase("Test RLC TM: transparent pass-through, opportunity edge cases and "
                            "overflow drop")
    {
    }

  private:
    void DoRun() override;
};

void
NrRlcTmTestCase::DoRun()
{
    Ptr<NrRlcTm> rlc = CreateObject<NrRlcTm>();
    rlc->SetAttribute("MaxTxBufferSize", UintegerValue(100));

    Ptr<NrTestMac> mac = CreateObject<NrTestMac>();
    rlc->SetNrMacSapProvider(mac->GetNrMacSapProvider());
    mac->SetNrMacSapUser(rlc->GetNrMacSapUser());

    NrRlcTestSduSink sink;
    rlc->SetNrRlcSapUser(sink.GetSapUser());
    rlc->TraceConnectWithoutContext("TxPDU", MakeCallback(&NrRlcTestCaseBase::HandleTxPdu, this));
    rlc->TraceConnectWithoutContext("TxDrop", MakeCallback(&NrRlcTestCaseBase::HandleTxDrop, this));

    // An opportunity with an empty buffer produces no PDU.
    GrantTxOpportunities(rlc, 100, 1);
    NS_TEST_ASSERT_MSG_EQ(m_pduSizes.size(),
                          0,
                          "an opportunity with an empty Tx buffer must not produce a PDU");

    // An SDU is queued; an opportunity smaller than the SDU produces no PDU and the
    // SDU stays queued (TM cannot segment).
    rlc->DoTransmitPdcpPdu(Create<Packet>(60));
    GrantTxOpportunities(rlc, 59, 1);
    NS_TEST_ASSERT_MSG_EQ(m_pduSizes.size(),
                          0,
                          "an opportunity smaller than the SDU must not produce a PDU");
    NS_TEST_ASSERT_MSG_EQ(rlc->m_txBuffer.size(),
                          1,
                          "the SDU must remain queued after a too-small opportunity");

    // A second SDU overflowing MaxTxBufferSize (60 + 50 > 100) is dropped, fires the
    // TxDrop trace, and leaves the buffer untouched.
    rlc->DoTransmitPdcpPdu(Create<Packet>(50));
    NS_TEST_ASSERT_MSG_EQ(m_dropCount, 1, "the overflowing SDU must fire the TxDrop trace once");
    NS_TEST_ASSERT_MSG_EQ(rlc->m_txBuffer.size(),
                          1,
                          "the overflowing SDU must not be enqueued in the Tx buffer");
    NS_TEST_ASSERT_MSG_EQ(rlc->m_txBufferSize,
                          60,
                          "the Tx buffer size must only account for the stored SDU");

    // A large-enough opportunity transmits the SDU unmodified: transparent mode adds
    // no RLC header, so the PDU size equals the SDU size.
    GrantTxOpportunities(rlc, 60, 1);
    NS_TEST_ASSERT_MSG_EQ(m_pduSizes.size(), 1, "a valid opportunity must produce one PDU");
    NS_TEST_ASSERT_MSG_EQ(m_pduSizes.at(0),
                          60,
                          "transparent mode must transmit the SDU without any header");
    NS_TEST_ASSERT_MSG_EQ(rlc->m_txBuffer.size(), 0, "the Tx buffer must be drained");
    NS_TEST_ASSERT_MSG_EQ(rlc->m_txBufferSize, 0, "the Tx buffer size must drain to zero");

    // A received PDU is delivered to the upper layer unmodified.
    DeliverPduToRlc(rlc, Create<Packet>(42));
    NS_TEST_ASSERT_MSG_EQ(sink.m_sdus.size(), 1, "a received PDU must be delivered upward");
    NS_TEST_ASSERT_MSG_EQ(sink.m_sdus.at(0)->GetSize(),
                          42,
                          "transparent mode must deliver the PDU unmodified");

    // MaxTxBufferSize = 0 means an unlimited buffer (consistent with NrRlcAm and
    // NrRlcUm): nothing is dropped.
    Ptr<NrRlcTm> unlimitedRlc = CreateObject<NrRlcTm>();
    unlimitedRlc->SetAttribute("MaxTxBufferSize", UintegerValue(0));
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

class NrRlcTmTestSuite : public TestSuite
{
  public:
    NrRlcTmTestSuite()
        : TestSuite("nr-test-rlc-tm", Type::SYSTEM)
    {
        AddTestCase(new NrRlcTmTestCase(), Duration::QUICK);
    }
};

static NrRlcTmTestSuite g_nrRlcTmTestSuite;

} // namespace ns3
