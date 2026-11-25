// Copyright (c) 2026 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only

#include "ns3/nr-eesm-ir-t1.h"
#include "ns3/nr-error-model.h"
#include "ns3/nr-spectrum-phy.h"
#include "ns3/nr-spectrum-value-helper.h"
#include "ns3/packet-burst.h"
#include "ns3/simulator.h"
#include "ns3/test.h"

#include <cmath>

/**
 * @file nr-test-harq-flush.cc
 * @ingroup test
 *
 * @brief Unit test for flushing the HARQ soft-combining history when a new
 * transmission (NDI set) arrives on a HARQ process that still holds history
 * from an earlier corrupted transport block.
 */
namespace ns3
{

/**
 * @brief Checks that NrSpectrumPhy::CheckTransportBlockCorruptionStatus resets
 * the HARQ process history for a new transmission (NDI == 1) and preserves it
 * for a retransmission (NDI == 0).
 */
class NrSpectrumPhyHarqFlushTestCase : public TestCase
{
  public:
    /**
     * @brief Constructor
     * @param ndi new data indicator of the incoming transport block
     * @param isDownlink whether the transport block is downlink (true) or uplink (false)
     * @param name test case name
     */
    NrSpectrumPhyHarqFlushTestCase(uint8_t ndi, bool isDownlink, const std::string& name)
        : TestCase(name),
          m_ndi(ndi),
          m_isDownlink(isDownlink)
    {
    }

  private:
    void DoRun() override;

    uint8_t m_ndi;     //!< new data indicator of the incoming transport block
    bool m_isDownlink; //!< direction of the incoming transport block
};

void
NrSpectrumPhyHarqFlushTestCase::DoRun()
{
    const uint16_t rnti = 10;
    const uint8_t harqProcessId = 3;
    const uint32_t numRbs = 4;

    Ptr<NrSpectrumPhy> phy = CreateObject<NrSpectrumPhy>();
    phy->SetRnti(rnti);
    phy->SetAttribute("ErrorModelType", TypeIdValue(NrEesmIrT1::GetTypeId()));

    // Provide a perceived SINR so the TB corruption check can run
    Ptr<const SpectrumModel> spectrumModel =
        NrSpectrumValueHelper::GetSpectrumModel(numRbs, 3.5e9, 30000);
    SpectrumValue sinr(spectrumModel);
    sinr += 10.0; // linear scale
    phy->m_sinrPerceived = sinr;
    phy->m_rxPacketBurstList.push_back(CreateObject<PacketBurst>());

    std::vector<int> rbBitmap;
    rbBitmap.reserve(numRbs);
    for (uint32_t i = 0; i < numRbs; ++i)
    {
        rbBitmap.push_back(static_cast<int>(i));
    }

    // Leave stale history on the HARQ process, as a corrupted transport block
    // whose retransmission chain ended without a successful reception would
    Ptr<NrEesmIrT1> errorModel = CreateObject<NrEesmIrT1>();
    Ptr<NrErrorModelOutput> staleOutput =
        errorModel->GetTbDecodificationStats(sinr, rbBitmap, 100, 4, {});
    if (m_isDownlink)
    {
        phy->m_harqPhyModule.UpdateDlHarqProcessStatus(rnti, harqProcessId, staleOutput);
    }
    else
    {
        phy->m_harqPhyModule.UpdateUlHarqProcessStatus(rnti, harqProcessId, staleOutput);
    }

    phy->AddExpectedTb(ExpectedTb(m_ndi,
                                  100,
                                  4,
                                  1,
                                  rnti,
                                  rbBitmap,
                                  harqProcessId,
                                  0,
                                  m_isDownlink,
                                  0,
                                  1,
                                  SfnSf()));

    phy->CheckTransportBlockCorruptionStatus();

    const auto& history =
        phy->m_harqPhyModule.GetHarqProcessInfoDlUl(m_isDownlink, rnti, harqProcessId);
    if (m_ndi == 1)
    {
        NS_TEST_ASSERT_MSG_EQ(history.size(),
                              0,
                              "A new transmission (NDI set) must flush the stale HARQ history");
    }
    else
    {
        NS_TEST_ASSERT_MSG_EQ(history.size(),
                              1,
                              "A retransmission (NDI clear) must keep the HARQ history");
    }

    Simulator::Destroy();
}

/**
 * @brief Regression test for the scenario reported in issue #281: with HARQ
 * retransmissions disabled, fixed MCS, and SINR a couple of dB below the
 * decoding cliff, every new transport block must fail (BLER close to 1).
 * Without the history flush, soft combining with the history left by the
 * previous corrupted transport block lifts the effective SINR above the
 * cliff and roughly every other transport block is decoded (BLER ~ 0.5).
 */
class NrHarqFlushBlerTestCase : public TestCase
{
  public:
    /**
     * @brief Constructor
     * @param name test case name
     */
    NrHarqFlushBlerTestCase(const std::string& name)
        : TestCase(name)
    {
    }

  private:
    void DoRun() override;
};

void
NrHarqFlushBlerTestCase::DoRun()
{
    const uint16_t rnti = 11;
    const uint8_t harqProcessId = 0;
    const uint32_t numRbs = 4;
    const uint8_t mcs = 5;
    const uint32_t tbSize = 256;
    const uint32_t numTbs = 40;

    Ptr<NrEesmIrT1> errorModel = CreateObject<NrEesmIrT1>();
    Ptr<const SpectrumModel> spectrumModel =
        NrSpectrumValueHelper::GetSpectrumModel(numRbs, 3.5e9, 30000);
    std::vector<int> rbBitmap;
    rbBitmap.reserve(numRbs);
    for (uint32_t i = 0; i < numRbs; ++i)
    {
        rbBitmap.push_back(static_cast<int>(i));
    }

    auto tblerAt = [&errorModel,
                    &spectrumModel,
                    &rbBitmap](double sinrDb, NrErrorModel::NrErrorModelHistory history) {
        SpectrumValue sinr(spectrumModel);
        sinr += std::pow(10.0, sinrDb / 10.0);
        return errorModel->GetTbDecodificationStats(sinr, rbBitmap, tbSize, mcs, history)->m_tbler;
    };

    // Locate the decoding cliff (TBLER == 0.5) for a fresh transmission
    double lo = -30.0;
    double hi = 40.0;
    for (int i = 0; i < 60; ++i)
    {
        double mid = 0.5 * (lo + hi);
        if (tblerAt(mid, {}) > 0.5)
        {
            lo = mid;
        }
        else
        {
            hi = mid;
        }
    }
    const double sinrDb = 0.5 * (lo + hi) - 2.0;

    // Fresh TBs must almost always fail a couple of dB below the cliff...
    const double freshTbler = tblerAt(sinrDb, {});
    NS_TEST_ASSERT_MSG_GT(freshTbler,
                          0.9,
                          "SINR two dB below the cliff must give TBLER close to 1, got "
                              << freshTbler);

    // ...while combining with one stale reception would lift the effective
    // SINR enough to decode about half of the time
    SpectrumValue sinr(spectrumModel);
    sinr += std::pow(10.0, sinrDb / 10.0);
    NrErrorModel::NrErrorModelHistory staleHistory;
    staleHistory.push_back(errorModel->GetTbDecodificationStats(sinr, rbBitmap, tbSize, mcs, {}));
    const double combinedTbler = tblerAt(sinrDb, staleHistory);
    NS_TEST_ASSERT_MSG_LT(combinedTbler,
                          0.5,
                          "Combining with stale history must drop TBLER below 0.5, got "
                              << combinedTbler);

    Ptr<NrSpectrumPhy> phy = CreateObject<NrSpectrumPhy>();
    phy->SetRnti(rnti);
    phy->SetAttribute("ErrorModelType", TypeIdValue(NrEesmIrT1::GetTypeId()));
    phy->m_sinrPerceived = sinr;
    phy->m_rxPacketBurstList.push_back(CreateObject<PacketBurst>());

    // HARQ retransmissions disabled: every TB is new data (NDI set, rv == 0)
    // on the same process. Mirror SendDlHarqFeedback: a corrupted TB leaves
    // its error model output on the process history.
    uint32_t corrupted = 0;
    for (uint32_t i = 0; i < numTbs; ++i)
    {
        phy->AddExpectedTb(
            ExpectedTb(1, tbSize, mcs, 1, rnti, rbBitmap, harqProcessId, 0, true, 0, 1, SfnSf()));
        phy->CheckTransportBlockCorruptionStatus();
        const TransportBlockInfo& tbInfo = phy->m_transportBlocks.at(rnti);
        if (tbInfo.m_isCorrupted)
        {
            ++corrupted;
            phy->m_harqPhyModule.UpdateDlHarqProcessStatus(rnti,
                                                           harqProcessId,
                                                           tbInfo.m_outputOfEM);
        }
    }

    const double bler = static_cast<double>(corrupted) / numTbs;
    NS_TEST_ASSERT_MSG_GT(bler,
                          0.9,
                          "BLER below the cliff with retransmissions disabled must be close to "
                          "1, got "
                              << bler);

    Simulator::Destroy();
}

class NrHarqFlushTestSuite : public TestSuite
{
  public:
    NrHarqFlushTestSuite()
        : TestSuite("nr-test-harq-flush", Type::UNIT)
    {
        AddTestCase(
            new NrSpectrumPhyHarqFlushTestCase(1, true, "NDI set flushes stale DL HARQ history"),
            Duration::QUICK);
        AddTestCase(new NrSpectrumPhyHarqFlushTestCase(0, true, "NDI clear keeps DL HARQ history"),
                    Duration::QUICK);
        AddTestCase(
            new NrSpectrumPhyHarqFlushTestCase(1, false, "NDI set flushes stale UL HARQ history"),
            Duration::QUICK);
        AddTestCase(new NrSpectrumPhyHarqFlushTestCase(0, false, "NDI clear keeps UL HARQ history"),
                    Duration::QUICK);
        AddTestCase(new NrHarqFlushBlerTestCase("BLER stays close to 1 below the cliff when HARQ "
                                                "retransmissions are disabled"),
                    Duration::QUICK);
    }
};

static NrHarqFlushTestSuite g_harqFlushTestSuite; //!< HARQ flush test suite

} // namespace ns3
