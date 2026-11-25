// Copyright (c) 2020 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only

/**
 * @ingroup test
 * @file nr-spectrum-phy-test.cc
 *
 * @brief Test suite (nr-spectrum-phy-test) checking that
 * NrSpectrumPhy::SetNoisePowerSpectralDensity reconfigures the receiver noise correctly. Each
 * SetNoisePsdTestCase connects a transmitting and a receiving NrSpectrumPhy through a
 * MultiModelSpectrumChannel with a no-loss propagation model, transmits the same data frame
 * twice, and changes the noise figure between the two receptions. The SNR values reported by the
 * NrInterference "SnrPerProcessedChunk" trace must differ between the receptions, and each must
 * match the analytically computed SNR (tx PSD over thermal noise PSD times the noise figure)
 * within 10 percent. The suite sweeps bandwidths from 1.4 MHz to 1 GHz, several transmit powers
 * and noise figure pairs, and numerologies 0 to 4.
 */

#include "nr-spectrum-phy-test.h"

#include "ns3/beam-manager.h"
#include "ns3/constant-position-mobility-model.h"
#include "ns3/multi-model-spectrum-channel.h"
#include "ns3/nr-eesm-ir-t1.h"
#include "ns3/nr-gnb-phy.h"
#include "ns3/nr-interference.h"
#include "ns3/nr-mimo-chunk-processor.h"
#include "ns3/nr-spectrum-phy.h"
#include "ns3/nr-spectrum-signal-parameters.h"
#include "ns3/nr-spectrum-value-helper.h"
#include "ns3/uniform-planar-array.h"

#include <cmath>
#include <complex>
#include <numeric>

namespace ns3
{

NoLossSpectrumPropagationLossModel::NoLossSpectrumPropagationLossModel()
{
}

NoLossSpectrumPropagationLossModel::~NoLossSpectrumPropagationLossModel()
{
}

TypeId
NoLossSpectrumPropagationLossModel::GetTypeId()
{
    static TypeId tid = TypeId("ns3::NoLossSpectrumPropagationLossModel")
                            .SetParent<SpectrumPropagationLossModel>()
                            .SetGroupName("Spectrum");
    return tid;
}

Ptr<SpectrumValue>
NoLossSpectrumPropagationLossModel::DoCalcRxPowerSpectralDensity(
    Ptr<const SpectrumSignalParameters> params,
    Ptr<const MobilityModel> a,
    Ptr<const MobilityModel> b) const
{
    return Copy(params->psd);
}

int64_t
NoLossSpectrumPropagationLossModel::DoAssignStreams(int64_t stream)
{
    return 0;
}

SetNoisePsdTestCase::SetNoisePsdTestCase(double txPower,
                                         double bandwidth,
                                         double noiseFigureFirst,
                                         double noiseFigureSecond,
                                         double expectedSnrFirst,
                                         double expectedSnrSecond,
                                         uint8_t numerology)
    : TestCase("NrSpectrumPhy configuration test case")
{
    m_txPower = txPower;
    m_bandwidth = bandwidth;
    m_noiseFigureFirst = noiseFigureFirst;
    m_noiseFigureSecond = noiseFigureSecond;
    m_expectedSnrFirst = expectedSnrFirst;
    m_expectedSnrSecond = expectedSnrSecond;
    m_numerology = numerology;
}

SetNoisePsdTestCase::~SetNoisePsdTestCase()
{
}

void
SetNoisePsdTestCase::SaveSnr(double snr)
{
    m_snr.push_back(snr);
}

void
TestSaveSnr(SetNoisePsdTestCase* test, double snr)
{
    test->SaveSnr(snr);
}

void
SetNoisePsdTestCase::DoEvaluateTest()
{
    if (m_snr.size() != 2)
    {
        NS_TEST_ASSERT_MSG_EQ(true,
                              false,
                              "Test could not be evaluated, something wrong in test setup. Expects "
                              "to obtain two SNR values during the simulation.");
    }
    else
    {
        NS_TEST_ASSERT_MSG_NE(m_snr.at(0),
                              m_snr.at(1),
                              "SNR should not be equal because noise figure has been changed.");
        NS_TEST_ASSERT_MSG_EQ_TOL(m_snr.at(0),
                                  m_expectedSnrFirst,
                                  m_expectedSnrFirst * 0.1,
                                  "First SNR is not as expected.");
        NS_TEST_ASSERT_MSG_EQ_TOL(m_snr.at(1),
                                  m_expectedSnrSecond,
                                  m_expectedSnrSecond * 0.1,
                                  "Second SNR is not as expected.");
    }
}

void
SetNoisePsdTestCase::DoRun()
{
    double centerFrequency = 28e9;

    Ptr<NrSpectrumPhy> rxPhy = CreateObject<NrSpectrumPhy>();
    rxPhy->SetMobility(CreateObject<ConstantPositionMobilityModel>());
    Ptr<MultiModelSpectrumChannel> spectrumChannel = CreateObject<MultiModelSpectrumChannel>();
    spectrumChannel->AddSpectrumPropagationLossModel(
        CreateObject<NoLossSpectrumPropagationLossModel>());
    rxPhy->SetChannel(spectrumChannel);
    Ptr<NrGnbPhy> phy = CreateObject<NrGnbPhy>();
    Ptr<BeamManager> beamManager = CreateObject<BeamManager>();
    phy->InstallSpectrumPhy(rxPhy);
    rxPhy->InstallPhy(phy);
    Ptr<UniformPlanarArray> antenna = CreateObject<UniformPlanarArray>();
    rxPhy->SetAntenna(antenna);
    beamManager->Configure(antenna);
    phy->DoSetCellId(99);
    rxPhy->InstallPhy(phy);
    double subcarrierSpacing = 15000 * static_cast<uint32_t>(std::pow(2, m_numerology));
    uint32_t rbNum = m_bandwidth / (12 * subcarrierSpacing);
    Ptr<const SpectrumModel> sm =
        NrSpectrumValueHelper::GetSpectrumModel(rbNum, centerFrequency, subcarrierSpacing);

    std::vector<int> activeRbs(sm->GetNumBands());
    std::iota(activeRbs.begin(), activeRbs.end(), 0);

    Ptr<const SpectrumValue> txPsd = NrSpectrumValueHelper::CreateTxPowerSpectralDensity(
        m_txPower,
        activeRbs,
        sm,
        NrSpectrumValueHelper::UNIFORM_POWER_ALLOCATION_BW);
    Ptr<const SpectrumValue> nsv0first =
        NrSpectrumValueHelper::CreateNoisePowerSpectralDensity(m_noiseFigureFirst, sm);
    Ptr<const SpectrumValue> nsv0second =
        NrSpectrumValueHelper::CreateNoisePowerSpectralDensity(m_noiseFigureSecond, sm);

    Ptr<NrSpectrumSignalParametersDataFrame> params1 =
        Create<NrSpectrumSignalParametersDataFrame>();
    params1->duration = Time(MilliSeconds(1));
    params1->psd = Copy(txPsd);
    params1->cellId = 99;
    Ptr<NrSpectrumPhy> txPhy = CreateObject<NrSpectrumPhy>();
    txPhy->SetMobility(CreateObject<ConstantPositionMobilityModel>());
    Ptr<UniformPlanarArray> ueAntenna = CreateObject<UniformPlanarArray>();
    Ptr<NrGnbPhy> gnbPhy = CreateObject<NrGnbPhy>();
    gnbPhy->InstallSpectrumPhy(txPhy);
    txPhy->InstallPhy(gnbPhy);
    txPhy->SetAntenna(ueAntenna);
    Ptr<BeamManager> ueBeamManager = CreateObject<BeamManager>();
    ueBeamManager->Configure(ueAntenna);
    gnbPhy->DoSetCellId(99);

    params1->txPhy = txPhy;

    rxPhy->GetNrInterference()->TraceConnectWithoutContext("SnrPerProcessedChunk",
                                                           MakeBoundCallback(&TestSaveSnr, this));

    Simulator::Schedule(MilliSeconds(0),
                        &NrSpectrumPhy::SetNoisePowerSpectralDensity,
                        rxPhy,
                        nsv0first);
    // spectrum phy can be attached to spectrum channel only once that the spectrum model of the
    // spectrum phy is being set spectrum model is being set when noise power spectral density is
    // set for the first time
    Simulator::Schedule(MilliSeconds(0), &MultiModelSpectrumChannel::AddRx, spectrumChannel, rxPhy);
    Simulator::Schedule(MilliSeconds(1),
                        &MultiModelSpectrumChannel::StartTx,
                        spectrumChannel,
                        params1);
    Simulator::Schedule(MilliSeconds(3), &NrInterference::EndRx, rxPhy->GetNrInterference());

    Simulator::Schedule(MilliSeconds(4),
                        &NrSpectrumPhy::SetNoisePowerSpectralDensity,
                        rxPhy,
                        nsv0second);
    Simulator::Schedule(MilliSeconds(5),
                        &MultiModelSpectrumChannel::StartTx,
                        spectrumChannel,
                        params1);
    Simulator::Schedule(MilliSeconds(7), &NrInterference::EndRx, rxPhy->GetNrInterference());

    Simulator::Schedule(MilliSeconds(9), &SetNoisePsdTestCase::DoEvaluateTest, this);

    Simulator::Run();
    rxPhy->Dispose();  // Explicitly dispose NrSpectrumPhy since it is not aggregated to a Node
    gnbPhy->Dispose(); // Explicitly dispose NrPhy since it is not aggregated to a Node
    Simulator::Destroy();
}

/**
 * @ingroup test
 * @brief Check the channel matrix NrSpectrumPhy builds for a signal without one.
 *
 * The matrix must carry the received power, whatever the transmit power, and the MIMO
 * SINR computed from it must be the received power over the noise.
 */
class NrSpectrumPhyChannelMatrixTestCase : public TestCase
{
  public:
    NrSpectrumPhyChannelMatrixTestCase()
        : TestCase("Channel matrix without a channel model carries the received power")
    {
    }

  private:
    void DoRun() override
    {
        // Received PSD with different values per RB, including an RB without signal
        const uint32_t numRbs = 4;
        const double noisePsdValue = 1e-20;
        const std::vector<double> rxPsdValues{1e-17, 2e-18, 0.0, 5e-19};

        // Single-port PHYs, with a transmit PSD far from the received one
        auto sm = NrSpectrumValueHelper::GetSpectrumModel(numRbs, 3.5e9, 15e3);
        auto txPsd = Create<SpectrumValue>(sm);
        *txPsd = 1e-6;
        auto txPhy = CreateObject<NrSpectrumPhy>();
        txPhy->SetAntenna(CreateObject<UniformPlanarArray>());
        txPhy->SetTxPowerSpectralDensity(txPsd);
        auto rxPhy = CreateObject<NrSpectrumPhy>();
        rxPhy->SetAntenna(CreateObject<UniformPlanarArray>());

        // Signal without a channel matrix, for which NrSpectrumPhy builds one
        auto params = Create<NrSpectrumSignalParametersDataFrame>();
        params->psd = Create<SpectrumValue>(sm);
        for (uint32_t rb = 0; rb < numRbs; ++rb)
        {
            (*params->psd)[rb] = rxPsdValues[rb];
        }
        params->txPhy = txPhy;
        params->cellId = 1;
        params->rnti = 1;
        params->spectrumChannelMatrix = rxPhy->CreateSpectrumChannelMatrix(params);

        // The squared channel gain must not depend on the transmit PSD
        for (uint32_t rb = 0; rb < numRbs; ++rb)
        {
            NS_TEST_EXPECT_MSG_EQ_TOL(std::norm(params->spectrumChannelMatrix->Elem(0, 0, rb)),
                                      rxPsdValues[rb],
                                      1e-9 * rxPsdValues[rb],
                                      "Channel gain must be the received PSD on RB " << rb);
        }

        // Evaluate the MIMO SINR of the signal alone, over the noise
        auto noisePsd = Create<SpectrumValue>(sm);
        *noisePsd = noisePsdValue;
        std::vector<MimoSinrChunk> chunks;
        auto mimoChunk = Create<NrMimoChunkProcessor>();
        mimoChunk->AddCallback(Callback<void, const std::vector<MimoSinrChunk>&>(
            [&chunks](const std::vector<MimoSinrChunk>& c) { chunks = c; }));
        auto interference = CreateObject<NrInterference>();
        interference->AddMimoChunkProcessor(mimoChunk);
        interference->SetNoisePowerSpectralDensity(noisePsd);
        interference->AddSignalMimo(params, MilliSeconds(1));
        interference->StartRxMimo(params);
        Simulator::Schedule(MilliSeconds(1), &NrInterference::EndRx, interference);
        Simulator::Run();

        // The SINR must follow the link budget on every RB
        NS_TEST_ASSERT_MSG_EQ(chunks.empty(), false, "No MIMO SINR chunk was evaluated");
        for (uint32_t rb = 0; rb < numRbs; ++rb)
        {
            const double expected = rxPsdValues[rb] / noisePsdValue;
            NS_TEST_EXPECT_MSG_EQ_TOL(chunks.back().mimoSinr(0, rb),
                                      expected,
                                      1e-9 * expected,
                                      "SINR must be the received PSD over the noise on RB " << rb);
        }

        // Break the reference cycles, as the PHYs are not aggregated to a node
        txPhy->Dispose();
        rxPhy->Dispose();
        Simulator::Destroy();
    }
};

/**
 * @ingroup test
 * @brief Check the SINR of a transport block reported in the RX packet trace.
 *
 * With MIMO, the reported SINR must be the per-layer SINR that the error model decodes
 * with, over the RBs of the TB, and the TBLER must be consistent with it even when the
 * SISO SINR is much higher. Without MIMO, the SISO SINR must be reported.
 */
class NrSpectrumPhyTbSinrTestCase : public TestCase
{
  public:
    /**
     * Constructor.
     *
     * @param mimo Whether the reception has MIMO SINR chunks.
     */
    NrSpectrumPhyTbSinrTestCase(bool mimo)
        : TestCase(std::string("Reported TB SINR, ") + (mimo ? "MIMO" : "SISO")),
          m_mimo(mimo)
    {
    }

  private:
    void DoRun() override
    {
        // A TB of a high MCS on the first two of four RBs
        const uint16_t rnti = 10;
        const uint32_t numRbs = 4;
        const uint8_t rank = m_mimo ? 2 : 1;
        const uint8_t mcs = 20;

        // Bare spectrum PHY, whose NrPhy only provides the cell ID
        auto phy = CreateObject<NrSpectrumPhy>();
        auto gnbPhy = CreateObject<NrGnbPhy>();
        phy->InstallPhy(gnbPhy);
        phy->SetRnti(rnti);
        phy->SetAttribute("ErrorModelType", TypeIdValue(NrEesmIrT1::GetTypeId()));
        phy->m_rxPacketBurstList.push_back(CreateObject<PacketBurst>());

        // High on the RBs of the TB, and higher on the others, which must be ignored
        SpectrumValue siso(NrSpectrumValueHelper::GetSpectrumModel(numRbs, 3.5e9, 30000));
        siso[0] = 1000.0;
        siso[1] = 3000.0;
        siso[2] = siso[3] = 1e6;
        phy->m_sinrPerceived = siso;

        if (m_mimo)
        {
            // Layer SINRs per RB of the first chunk; those of the second one are twice as high
            NrSinrMatrix first{rank, numRbs};
            NrSinrMatrix second{rank, numRbs};
            const std::vector<std::vector<double>> sinr{{0.04, 0.16}, {0.08, 0.32}};
            for (std::size_t rb = 0; rb < numRbs; ++rb)
            {
                for (std::size_t layer = 0; layer < rank; ++layer)
                {
                    first(layer, rb) = rb < sinr.size() ? sinr[rb][layer] : 1e6;
                    second(layer, rb) = 2 * first(layer, rb);
                }
            }

            // Another UE of the cell with a high SINR, which must not be averaged in
            NrSinrMatrix otherUe{rank, numRbs};
            for (std::size_t rb = 0; rb < numRbs; ++rb)
            {
                otherUe(0, rb) = otherUe(1, rb) = 1e6;
            }

            // Feed the chunks as the MIMO chunk processor would at the end of a reception
            phy->m_mimoSinrPerceived = {
                MimoSinrChunk{first, rnti, phy->GetCellId(), MicroSeconds(1)},
                MimoSinrChunk{second, rnti, phy->GetCellId(), MicroSeconds(3)},
                MimoSinrChunk{otherUe, rnti + 1, phy->GetCellId(), MicroSeconds(4)},
            };
        }

        // Decode the TB
        phy->AddExpectedTb(ExpectedTb(1, 1000, mcs, rank, rnti, {0, 1}, 0, 0, true, 0, 1, SfnSf()));
        phy->CheckTransportBlockCorruptionStatus();

        // Check the SINR that RxPacketTraceParams copies from the TB
        const auto& tb = phy->m_transportBlocks.at(rnti);
        // Duration-weighted mean of the two chunks: 1.75 times the first one
        const double expectedAvg = m_mimo ? 1.75 * (0.04 + 0.16 + 0.08 + 0.32) / 4 : 2000.0;
        const double expectedMin = m_mimo ? 1.75 * 0.04 : 1000.0;
        NS_TEST_EXPECT_MSG_EQ_TOL(tb.m_sinrAvg, expectedAvg, 1e-9 * expectedAvg, "Wrong mean SINR");
        NS_TEST_EXPECT_MSG_EQ_TOL(tb.m_sinrMin, expectedMin, 1e-9 * expectedMin, "Wrong min SINR");

        // The TBLER must be consistent with the reported SINR
        if (m_mimo)
        {
            // 64-QAM does not decode at these SINRs, whatever the SISO SINR says
            NS_TEST_EXPECT_MSG_GT(tb.m_outputOfEM->m_tbler,
                                  0.99,
                                  "TBLER must match the low reported SINR");
        }
        else
        {
            NS_TEST_EXPECT_MSG_LT(tb.m_outputOfEM->m_tbler,
                                  0.01,
                                  "TBLER must match the high reported SINR");
        }

        // Break the PHY <-> spectrum PHY reference cycle
        phy->Dispose();
        gnbPhy->Dispose();
        Simulator::Destroy();
    }

    bool m_mimo; ///< Whether the reception has MIMO SINR chunks
};

NrSpectrumPhyTestSuite::NrSpectrumPhyTestSuite()
    : TestSuite("nr-spectrum-phy-test")
{
    struct TestInputValues
    {
        double bandwidth{100e6}; // Hz
        double txPower{10};      // dBm
        double noiseFigure1{5};  // dB
        double noiseFigure2{6};  // dB
        uint8_t numerology{0};   // integer value

        TestInputValues(double b, double t, double n1, double n2, double u)
            : bandwidth(b),
              txPower(t),
              noiseFigure1(n1),
              noiseFigure2(n2),
              numerology(u)
        {
        }
    };

    std::vector<TestInputValues> testInputValuesSet = {
        {100e6, 10.0, 4, 6, 0},
        {200e6, 5.0, 5, 6, 0},
        {300e6, 20.0, 7, 5, 0},
        {1e9, 30.0, 4, 5, 0},
        {20e6, 4.0, 5, 6, 0},
        {10e6, 1.0, 8, 9, 0},
        {5e6, 1.0, 8, 9, 0},
        {1.4e6, 0.5, 8, 9, 0},
        {100e6, 10.0, 4, 6, 1},
        {200e6, 5.0, 5, 6, 1},
        {300e6, 20.0, 7, 5, 1},
        {1e9, 30.0, 4, 5, 1},
        {20e6, 4.0, 5, 6, 1},
        {10e6, 1.0, 8, 9, 1},
        {5e6, 1.0, 8, 9, 1},
        {1.4e6, 0.5, 8, 9, 1},
        {100e6, 10.0, 4, 6, 2},
        {200e6, 5.0, 5, 6, 2},
        {300e6, 20.0, 7, 5, 2},
        {1e9, 30.0, 4, 5, 2},
        {20e6, 4.0, 5, 6, 2},
        {10e6, 1.0, 8, 9, 2},
        {5e6, 1.0, 8, 9, 2},
        {1.4e6, 0.5, 8, 9, 2},
        {100e6, 10.0, 4, 6, 3},
        {200e6, 5.0, 5, 6, 3},
        {300e6, 20.0, 7, 5, 3},
        {1e9, 30.0, 4, 5, 3},
        {20e6, 4.0, 5, 6, 3},
        {10e6, 1.0, 8, 9, 3},
        {5e6, 1.0, 8, 9, 3},
        //{ 1.4e6,   0.5, 8, 9, 3}, This test case will not work since 1.4MHz cannot s not enough
        // width for numerology 3, because numerology 3 RB width is 1.44 MHz
        {100e6, 10.0, 4, 6, 4},
        {200e6, 5.0, 5, 6, 4},
        {300e6, 20.0, 7, 5, 4},
        {1e9, 30.0, 4, 5, 4},
        {20e6, 4.0, 5, 6, 4},
        {10e6, 1.0, 8, 9, 4},
        {5e6, 1.0, 8, 9, 4},
        //{ 1.4e6,   0.5, 8, 9, 4} This test case will not work since 1.4MHz cannot is not enough
        // width for numerology 3, because numerology 3 RB width is 2.88 MHz
    };

    for (auto input : testInputValuesSet)
    {
        uint64_t effectiveBandwidth =
            NrSpectrumValueHelper::GetEffectiveBandwidth(input.bandwidth, input.numerology);
        double txPowerW = std::pow(10., (input.txPower - 30) / 10);     // W
        double txPowerWDensity = txPowerW / (double)effectiveBandwidth; // W/Hz
        double kT_W_Hz = std::pow(10.0, (-174.0 - 30) / 10.0);          // W/Hz

        // calculate what should be SNR for noise figure 1
        double noiseFigureLinear1 = std::pow(10.0, input.noiseFigure1 / 10.0);
        double noisePowerSpectralDensity1 = kT_W_Hz * noiseFigureLinear1;
        double expectedSnr1 = txPowerWDensity / noisePowerSpectralDensity1;

        // calculate what should be SNR for noise figure 2
        double noiseFigureLinear2 = std::pow(10.0, input.noiseFigure2 / 10.0);
        double noisePowerSpectralDensity2 = kT_W_Hz * noiseFigureLinear2;
        double expectedSnr2 = txPowerWDensity / noisePowerSpectralDensity2;

        AddTestCase(new SetNoisePsdTestCase(input.txPower,
                                            input.bandwidth,
                                            input.noiseFigure1,
                                            input.noiseFigure2,
                                            expectedSnr1,
                                            expectedSnr2,
                                            input.numerology),
                    Duration::QUICK);
    }
    AddTestCase(new NrSpectrumPhyChannelMatrixTestCase(), Duration::QUICK);
    AddTestCase(new NrSpectrumPhyTbSinrTestCase(true), Duration::QUICK);
    AddTestCase(new NrSpectrumPhyTbSinrTestCase(false), Duration::QUICK);
}

// Allocate an instance of this TestSuite
static NrSpectrumPhyTestSuite g_nrSpectrumPhyTestSuite;

} // namespace ns3
