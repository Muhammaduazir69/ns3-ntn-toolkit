// Copyright (c) 2026 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only

#include "ns3/nr-mac-sap.h"
#include "ns3/nr-ue-cmac-sap.h"
#include "ns3/nr-ue-mac.h"
#include "ns3/test.h"

/**
 * @file nr-ue-mac-stale-bsr-test.cc
 * @ingroup tests
 * @brief Buffer status reports of logical channels unknown to the UE MAC
 */

namespace ns3
{

/**
 * @ingroup tests
 * @brief Discards a buffer status report whose logical channel is not configured in the MAC.
 *
 * The RLC entity of a logical channel that the MAC has dropped, on handover or through the
 * DoReset() that precedes a re-establishment, keeps the service access point it was given and
 * can still report a buffer status. Recording such a report used to leave an LCID in the
 * buffer status map with no counterpart in the logical channel map, which
 * NrUeMac::SendNewStatusData(), SendRetxData() and SendTxData() look up unconditionally when
 * an uplink grant arrives, terminating the simulation with std::out_of_range.
 */
class NrUeMacStaleBsrTestCase : public TestCase
{
  public:
    /**
     * Constructor.
     *
     * @param name Name of the test case.
     * @param removeLc Drop the logical channel with RemoveLc() when true, with Reset() otherwise.
     */
    NrUeMacStaleBsrTestCase(const std::string& name, bool removeLc)
        : TestCase(name),
          m_removeLc(removeLc)
    {
    }

  private:
    void DoRun() override;

    /**
     * Build a buffer status report carrying pending data.
     *
     * @param lcid Logical channel the report belongs to.
     * @return The report parameters.
     */
    static NrMacSapProvider::BufferStatusReportParameters MakeBsr(uint8_t lcid);

    bool m_removeLc; ///< Drop the logical channel with RemoveLc() rather than Reset()
};

NrMacSapProvider::BufferStatusReportParameters
NrUeMacStaleBsrTestCase::MakeBsr(uint8_t lcid)
{
    NrMacSapProvider::BufferStatusReportParameters params;
    params.rnti = 1;
    params.lcid = lcid;
    params.txQueueSize = 100;
    params.txQueueHolDelay = 0;
    params.retxQueueSize = 0;
    params.retxQueueHolDelay = 0;
    params.statusPduSize = 0;
    params.expBsrTimer = false;
    return params;
}

void
NrUeMacStaleBsrTestCase::DoRun()
{
    constexpr uint8_t lcid = 3;

    Ptr<NrUeMac> mac = CreateObject<NrUeMac>();
    mac->SetRnti(1);

    NrUeCmacSapProvider::LogicalChannelConfig lcConfig{};
    lcConfig.priority = 1;
    lcConfig.fiveQi = NrQosFlow::NGBR_LOW_LAT_EMBB;
    lcConfig.prioritizedBitRateKbps = 0;
    lcConfig.bucketSizeDurationMs = 0;
    lcConfig.logicalChannelGroup = 1;

    // A configured logical channel reports a buffer status, which the MAC records.
    mac->GetUeCmacSapProvider()->AddLc(lcid, lcConfig, nullptr);
    mac->GetUeMacSapProvider()->BufferStatusReport(MakeBsr(lcid));
    NS_TEST_ASSERT_MSG_GT(mac->GetTotalBufSize(),
                          0,
                          "the buffer status of a configured logical channel was not recorded");

    // The channel goes away, and so does its buffer status.
    if (m_removeLc)
    {
        mac->GetUeCmacSapProvider()->RemoveLc(lcid);
    }
    else
    {
        mac->GetUeCmacSapProvider()->Reset();
    }
    NS_TEST_ASSERT_MSG_EQ(mac->GetTotalBufSize(),
                          0,
                          "dropping the logical channel left its buffer status behind");

    // Its RLC entity reports once more, from the service access point it still holds.
    mac->GetUeMacSapProvider()->BufferStatusReport(MakeBsr(lcid));
    NS_TEST_ASSERT_MSG_EQ(mac->GetTotalBufSize(),
                          0,
                          "the buffer status of an unknown logical channel was recorded, and an "
                          "uplink grant selecting it would throw");

    mac->Dispose();
}

/**
 * @ingroup tests
 * @brief Test suite for the buffer status reports of unknown logical channels.
 */
class NrUeMacStaleBsrTestSuite : public TestSuite
{
  public:
    NrUeMacStaleBsrTestSuite();
};

NrUeMacStaleBsrTestSuite::NrUeMacStaleBsrTestSuite()
    : TestSuite("nr-ue-mac-stale-bsr", Type::UNIT)
{
    AddTestCase(new NrUeMacStaleBsrTestCase("BSR after RemoveLc", true), Duration::QUICK);
    AddTestCase(new NrUeMacStaleBsrTestCase("BSR after Reset", false), Duration::QUICK);
}

static NrUeMacStaleBsrTestSuite g_nrUeMacStaleBsrTestSuite; //!< Stale BSR test suite

} // namespace ns3
