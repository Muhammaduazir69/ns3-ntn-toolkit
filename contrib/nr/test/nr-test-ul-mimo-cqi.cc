// Copyright (c) 2026 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only

#include "ns3/nr-gnb-phy.h"
#include "ns3/nr-mimo-chunk-processor.h"
#include "ns3/nr-phy-sap.h"
#include "ns3/nr-spectrum-phy.h"
#include "ns3/nr-spectrum-value-helper.h"
#include "ns3/simulator.h"
#include "ns3/test.h"

#include <cmath>
#include <optional>

/**
 * @file nr-test-ul-mimo-cqi.cc
 * @ingroup test
 *
 * @brief Check that the PUSCH CQI is generated from the post-MMSE MIMO SINR.
 */
namespace ns3
{

/**
 * @ingroup test
 * @brief gNB PHY SAP user that only records the UL CQI reports.
 */
class UlMimoCqiPhySapUser : public NrGnbPhySapUser
{
  public:
    void ReceivePhyPdu(Ptr<Packet>) override
    {
    }

    void ReceiveControlMessage(Ptr<NrControlMessage>) override
    {
    }

    void SetCurrentSfn(const SfnSf&) override
    {
    }

    void SlotDlIndication(const SfnSf&, LteNrTddSlotType) override
    {
    }

    void SlotUlIndication(const SfnSf&, LteNrTddSlotType) override
    {
    }

    void UlCqiReport(NrMacSchedSapProvider::SchedUlCqiInfoReqParameters ulcqi) override
    {
        m_report = ulcqi;
    }

    void ReceiveRachPreamble(uint32_t) override
    {
    }

    void UlHarqFeedback(UlHarqInfo) override
    {
    }

    void BeamChangeReport(BeamId, uint8_t) override
    {
    }

    uint32_t GetNumRbPerRbg() const override
    {
        return 1;
    }

    std::shared_ptr<DciInfoElementTdma> GetDlCtrlDci() const override
    {
        return nullptr;
    }

    std::shared_ptr<DciInfoElementTdma> GetUlCtrlDci() const override
    {
        return nullptr;
    }

    uint8_t GetDlCtrlSymbols() const override
    {
        return 1;
    }

    /// Last UL CQI report received, if any
    std::optional<NrMacSchedSapProvider::SchedUlCqiInfoReqParameters> m_report;
};

/**
 * @ingroup test
 * @brief Feed MIMO SINR chunks to NrGnbPhy::GenerateMimoDataCqiReport() and check the
 * per-RB SINR reported to the MAC.
 *
 * Two UEs of the cell are received on disjoint RBs, and a UE of another cell overlaps
 * them with a higher SINR. Each RB must carry the SINR of its own UE, averaged over the
 * duration of the chunks, with two-layer SINRs combined by their geometric mean, and the
 * UE of the other cell must be ignored.
 */
class NrUlMimoCqiTestCase : public TestCase
{
  public:
    NrUlMimoCqiTestCase()
        : TestCase("PUSCH CQI from the MIMO SINR")
    {
    }

  private:
    void DoRun() override
    {
        // A tiny 4-RB band is enough to place two UEs side by side
        constexpr std::size_t numRbs = 4;
        constexpr uint16_t cellId = 0; // NrPhy default
        // The report is a SpectrumValue over the RX spectrum model of the gNB.
        // The model is taken from the noise PSD set on the spectrum PHY.
        auto model = NrSpectrumValueHelper::GetSpectrumModel(numRbs, 3.5e9, 15e3);
        auto noisePsd = Create<SpectrumValue>(model);

        // Bare gNB PHY: no channel or MAC, only what the CQI report needs
        auto phy = CreateObject<NrGnbPhy>();
        auto spectrumPhy = CreateObject<NrSpectrumPhy>();
        spectrumPhy->SetNoisePowerSpectralDensity(noisePsd);
        phy->InstallSpectrumPhy(spectrumPhy);
        // The stub SAP user records the report sent to the MAC
        UlMimoCqiPhySapUser sapUser;
        phy->SetPhySapUser(&sapUser);

        // UE 1, rank 1, on RBs 0-1: SINR 10 for 1 us, then 40 for 3 us
        NrSinrMatrix ue1a{1, numRbs};
        NrSinrMatrix ue1b{1, numRbs};
        ue1a(0, 0) = ue1a(0, 1) = 10.0;
        ue1b(0, 0) = ue1b(0, 1) = 40.0;
        // UE 2, rank 2, on RBs 2-3: layer SINRs 4 and 16 for 4 us
        NrSinrMatrix ue2{2, numRbs};
        ue2(0, 2) = ue2(0, 3) = 4.0;
        ue2(1, 2) = ue2(1, 3) = 16.0;
        // UE of another cell, with the same RNTI as UE 1, on all RBs.
        // Its SINR is high, so the result would show it if it was not filtered out.
        NrSinrMatrix other{1, numRbs};
        for (std::size_t rb = 0; rb < numRbs; ++rb)
        {
            other(0, rb) = 1000.0;
        }

        // Feed the chunks as the MIMO chunk processor would at the end of a reception
        phy->GenerateMimoDataCqiReport({
            MimoSinrChunk{ue1a, 1, cellId, MicroSeconds(1)},
            MimoSinrChunk{ue1b, 1, cellId, MicroSeconds(3)},
            MimoSinrChunk{ue2, 2, cellId, MicroSeconds(4)},
            MimoSinrChunk{other, 1, cellId + 1, MicroSeconds(4)},
        });

        // Exactly one PUSCH report with one SINR value per RB must reach the MAC
        NS_TEST_ASSERT_MSG_EQ(sapUser.m_report.has_value(), true, "No UL CQI report");
        const auto& sinr = sapUser.m_report->m_ulCqi.m_sinr;
        NS_TEST_ASSERT_MSG_EQ(sinr.size(), numRbs, "One SINR value per RB expected");
        NS_TEST_EXPECT_MSG_EQ(sapUser.m_report->m_ulCqi.m_type,
                              UlCqiInfo::PUSCH,
                              "Wrong UL CQI type");
        // UE 1: time average weighted by chunk duration
        const double ue1Sinr = (10.0 * 1 + 40.0 * 3) / 4;
        // UE 2: geometric mean of its two layers
        const double ue2Sinr = std::sqrt(4.0 * 16.0);
        // Each RB carries the SINR of the UE allocated there, and nothing from the other cell
        NS_TEST_EXPECT_MSG_EQ_TOL(sinr.at(0), ue1Sinr, 1e-9, "Wrong SINR on RB 0");
        NS_TEST_EXPECT_MSG_EQ_TOL(sinr.at(1), ue1Sinr, 1e-9, "Wrong SINR on RB 1");
        NS_TEST_EXPECT_MSG_EQ_TOL(sinr.at(2), ue2Sinr, 1e-9, "Wrong SINR on RB 2");
        NS_TEST_EXPECT_MSG_EQ_TOL(sinr.at(3), ue2Sinr, 1e-9, "Wrong SINR on RB 3");

        // Break the PHY <-> spectrum PHY reference cycle
        phy->Dispose();
        spectrumPhy->Dispose();
        Simulator::Destroy();
    }
};

/**
 * @ingroup test
 * @brief Test suite for the PUSCH CQI generated from the MIMO SINR.
 */
class NrUlMimoCqiTestSuite : public TestSuite
{
  public:
    NrUlMimoCqiTestSuite()
        : TestSuite("nr-ul-mimo-cqi", Type::UNIT)
    {
        AddTestCase(new NrUlMimoCqiTestCase(), Duration::QUICK);
    }
};

static NrUlMimoCqiTestSuite g_nrUlMimoCqiTestSuite; //!< Static variable for test initialization

} // namespace ns3
