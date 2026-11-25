// Copyright (c) 2026 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only

#include "ns3/nr-epc-x2-sap.h"
#include "ns3/nr-gnb-rrc.h"
#include "ns3/packet.h"
#include "ns3/test.h"

#include <cstdint>

/**
 * @file nr-x2-forwarding-drop-test.cc
 * @ingroup test
 *
 * @brief Unit test for the NrGnbRrc::X2DataForwardingDrop trace source.
 *
 * During an inter-frequency handover, user-plane packets forwarded over X2-U
 * can arrive at the target gNB outside the data-forwarding window: before the
 * target bearer is set up, or after it has been torn down at handover
 * completion. In that case NrGnbRrc has no X2-U TEID mapping for the packet and
 * drops it (rather than aborting the simulation). This test verifies that the
 * X2DataForwardingDrop trace fires for such an orphaned forwarded packet,
 * carrying the source/target cell IDs and the TEID, so the condition can be
 * detected programmatically instead of by parsing logs.
 */
namespace ns3
{

/**
 * @ingroup tests
 *
 * @brief Feeds an NrGnbRrc a forwarded X2-U packet with no registered TEID
 * mapping and checks that the drop trace fires with the expected values.
 */
class NrX2ForwardingDropTestCase : public TestCase
{
  public:
    NrX2ForwardingDropTestCase();

  private:
    void DoRun() override;

    /**
     * @brief Trace sink for the X2DataForwardingDrop trace source.
     * @param sourceCellId the forwarding (source) cell
     * @param targetCellId the receiving (target) cell
     * @param gtpTeid the TEID of the dropped forwarded packet
     * @param packet the dropped packet
     */
    void DropSink(uint16_t sourceCellId,
                  uint16_t targetCellId,
                  uint32_t gtpTeid,
                  Ptr<const Packet> packet);

    uint32_t m_dropCount{0};  ///< number of times the drop trace fired
    uint16_t m_lastSource{0}; ///< last reported source cell id
    uint16_t m_lastTarget{0}; ///< last reported target cell id
    uint32_t m_lastTeid{0};   ///< last reported TEID
    uint32_t m_lastSize{0};   ///< last reported packet size
};

NrX2ForwardingDropTestCase::NrX2ForwardingDropTestCase()
    : TestCase("X2-U forwarded data with no TEID mapping is dropped and traced")
{
}

void
NrX2ForwardingDropTestCase::DropSink(uint16_t sourceCellId,
                                     uint16_t targetCellId,
                                     uint32_t gtpTeid,
                                     Ptr<const Packet> packet)
{
    m_dropCount++;
    m_lastSource = sourceCellId;
    m_lastTarget = targetCellId;
    m_lastTeid = gtpTeid;
    m_lastSize = packet->GetSize();
}

void
NrX2ForwardingDropTestCase::DoRun()
{
    Ptr<NrGnbRrc> rrc = CreateObject<NrGnbRrc>();

    bool connected =
        rrc->TraceConnectWithoutContext("X2DataForwardingDrop",
                                        MakeCallback(&NrX2ForwardingDropTestCase::DropSink, this));
    NS_TEST_ASSERT_MSG_EQ(connected, true, "could not connect to the X2DataForwardingDrop trace");

    NrEpcX2SapUser* x2SapUser = rrc->GetEpcX2SapUser();
    NS_TEST_ASSERT_MSG_EQ(x2SapUser != nullptr, true, "NrGnbRrc must expose an X2 SAP user");

    // No X2-U TEID mapping has been registered on this fresh RRC, so any
    // forwarded packet is "outside the forwarding window" and must be dropped.
    NrEpcX2SapUser::UeDataParams params;
    params.sourceCellId = 1;
    params.targetCellId = 2;
    params.gtpTeid = 12345;
    params.ueData = Create<Packet>(100);

    x2SapUser->RecvUeData(params);

    NS_TEST_ASSERT_MSG_EQ(m_dropCount, 1, "drop trace should fire once for the unknown TEID");
    NS_TEST_ASSERT_MSG_EQ(m_lastSource, 1, "traced source cell id mismatch");
    NS_TEST_ASSERT_MSG_EQ(m_lastTarget, 2, "traced target cell id mismatch");
    NS_TEST_ASSERT_MSG_EQ(m_lastTeid, 12345u, "traced TEID mismatch");
    NS_TEST_ASSERT_MSG_EQ(m_lastSize, 100u, "traced packet size mismatch");

    rrc->Dispose();
}

/**
 * @ingroup tests
 *
 * @brief Test suite for the NrGnbRrc X2-U forwarded-data drop trace.
 */
class NrX2ForwardingDropTestSuite : public TestSuite
{
  public:
    NrX2ForwardingDropTestSuite();
};

NrX2ForwardingDropTestSuite::NrX2ForwardingDropTestSuite()
    : TestSuite("nr-x2-forwarding-drop", Type::UNIT)
{
    AddTestCase(new NrX2ForwardingDropTestCase(), Duration::QUICK);
}

static NrX2ForwardingDropTestSuite g_nrX2ForwardingDropTestSuite; ///< the test suite instance

} // namespace ns3
