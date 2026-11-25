// Copyright (c) 2026 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only

#include "nr-spectrum-phy-test.h" // reuse NoLossSpectrumPropagationLossModel

#include "ns3/beam-manager.h"
#include "ns3/constant-position-mobility-model.h"
#include "ns3/multi-model-spectrum-channel.h"
#include "ns3/nr-chunk-processor.h"
#include "ns3/nr-gnb-phy.h"
#include "ns3/nr-interference.h"
#include "ns3/nr-spectrum-phy.h"
#include "ns3/nr-spectrum-value-helper.h"
#include "ns3/simulator.h"
#include "ns3/spectrum-value.h"
#include "ns3/test.h"
#include "ns3/uniform-planar-array.h"

#include <cmath>
#include <numeric>
#include <vector>

/**
 * @file nr-ue-handover-interference-test.cc
 * @ingroup test
 *
 * @brief Tests that, during handover, a UE that simultaneously receives a
 * stale data signal from its previous (source) gNB and the data signal from
 * its current (target) gNB still receives the current gNB, with the stale
 * signal accounted only as interference.
 *
 * The two signals share the same resource blocks (non-orthogonal). The
 * previous gNB uses a different cellId, so NrSpectrumPhy::StartRx routes it to
 * the interference accumulator (AddSignalMimo) and never offers it to the
 * receive path. The current gNB signal (matching cellId and RNTI) is received,
 * and its SINR is degraded by the stale signal. No overlap assertion fires
 * because only one signal ever enters the receive path.
 */
namespace ns3
{

/**
 * @ingroup tests
 *
 * @brief A UE receives the current-cell data frame plus, optionally, a stale
 * previous-cell data frame on the same RBs, and checks reception and SINR.
 */
class NrUeHandoverInterferenceTestCase : public TestCase
{
  public:
    /**
     * @brief Constructor.
     *
     * @param name human-readable test case name
     * @param withStaleSignal whether the previous (source) gNB also transmits
     */
    NrUeHandoverInterferenceTestCase(std::string name, bool withStaleSignal);

    /**
     * @brief Trace sink storing the SNR (signal/noise) of each processed chunk.
     * @param snr the reported SNR
     */
    void SaveSnr(double snr);

  private:
    void DoRun() override;

    bool m_withStaleSignal;               ///< whether the stale previous-cell signal is present
    std::vector<double> m_snr;            ///< SNR values reported during the reception
    NrSpectrumValueCatcher m_sinrCatcher; ///< catches the per-RB SINR of the reception
};

NrUeHandoverInterferenceTestCase::NrUeHandoverInterferenceTestCase(std::string name,
                                                                   bool withStaleSignal)
    : TestCase(std::move(name)),
      m_withStaleSignal(withStaleSignal)
{
}

void
NrUeHandoverInterferenceTestCase::SaveSnr(double snr)
{
    m_snr.push_back(snr);
}

void
NrUeHandoverInterferenceTestCase::DoRun()
{
    const double centerFrequency = 28e9;
    const double bandwidth = 10e6;
    const uint8_t numerology = 0;
    const double txPowerDbm = 30.0;
    const double noiseFigureDb = 9.0;

    const uint16_t servingCellId = 2; // current/target gNB (the UE is camped here)
    const uint16_t staleCellId = 1;   // previous/source gNB (handover just happened)
    const uint16_t ueRnti = 1;

    // ----- Receiver: a UE camped on the target cell -----
    Ptr<NrSpectrumPhy> rxPhy = CreateObject<NrSpectrumPhy>();
    rxPhy->SetMobility(CreateObject<ConstantPositionMobilityModel>());
    Ptr<MultiModelSpectrumChannel> channel = CreateObject<MultiModelSpectrumChannel>();
    channel->AddSpectrumPropagationLossModel(CreateObject<NoLossSpectrumPropagationLossModel>());
    rxPhy->SetChannel(channel);

    // An NrGnbPhy is installed only to provide GetCellId(); the receiver itself
    // is a UE (m_isGnb stays false) with an assigned RNTI.
    Ptr<NrGnbPhy> uePhyModel = CreateObject<NrGnbPhy>();
    uePhyModel->InstallSpectrumPhy(rxPhy);
    rxPhy->InstallPhy(uePhyModel);
    Ptr<UniformPlanarArray> rxAntenna = CreateObject<UniformPlanarArray>();
    rxPhy->SetAntenna(rxAntenna);
    Ptr<BeamManager> rxBeam = CreateObject<BeamManager>();
    rxBeam->Configure(rxAntenna);
    uePhyModel->DoSetCellId(servingCellId);
    rxPhy->SetIsGnb(false);
    rxPhy->SetRnti(ueRnti);

    // Spectrum model and PSDs.
    double scs = 15000 * static_cast<uint32_t>(std::pow(2, numerology));
    uint32_t rbNum = bandwidth / (12 * scs);
    Ptr<const SpectrumModel> sm =
        NrSpectrumValueHelper::GetSpectrumModel(rbNum, centerFrequency, scs);
    std::vector<int> activeRbs(sm->GetNumBands());
    std::iota(activeRbs.begin(), activeRbs.end(), 0);
    Ptr<const SpectrumValue> txPsd = NrSpectrumValueHelper::CreateTxPowerSpectralDensity(
        txPowerDbm,
        activeRbs,
        sm,
        NrSpectrumValueHelper::UNIFORM_POWER_ALLOCATION_BW);
    Ptr<const SpectrumValue> noisePsd =
        NrSpectrumValueHelper::CreateNoisePowerSpectralDensity(noiseFigureDb, sm);

    // ----- A shared transmitter PHY (provides mobility/antenna for the channel) -----
    Ptr<NrSpectrumPhy> txPhy = CreateObject<NrSpectrumPhy>();
    txPhy->SetMobility(CreateObject<ConstantPositionMobilityModel>());
    Ptr<NrGnbPhy> txPhyModel = CreateObject<NrGnbPhy>();
    txPhyModel->InstallSpectrumPhy(txPhy);
    txPhy->InstallPhy(txPhyModel);
    Ptr<UniformPlanarArray> txAntenna = CreateObject<UniformPlanarArray>();
    txPhy->SetAntenna(txAntenna);
    Ptr<BeamManager> txBeam = CreateObject<BeamManager>();
    txBeam->Configure(txAntenna);
    txPhyModel->DoSetCellId(servingCellId);

    // Signal from the current (target) gNB: intended for this UE.
    Ptr<NrSpectrumSignalParametersDataFrame> current =
        Create<NrSpectrumSignalParametersDataFrame>();
    current->duration = MilliSeconds(1);
    current->psd = Copy(txPsd);
    current->cellId = servingCellId;
    current->rnti = ueRnti;
    current->txPhy = txPhy;

    // Stale signal from the previous (source) gNB on the same RBs (non-orthogonal).
    Ptr<NrSpectrumSignalParametersDataFrame> stale = Create<NrSpectrumSignalParametersDataFrame>();
    stale->duration = MilliSeconds(1);
    stale->psd = Copy(txPsd);
    stale->cellId = staleCellId; // different cell -> routed to interference
    stale->rnti = ueRnti + 4;    // irrelevant: filtered out by the cellId gate
    stale->txPhy = txPhy;

    // Capture the SNR (signal/noise, interference-excluded) per processed chunk.
    rxPhy->GetNrInterference()->TraceConnectWithoutContext(
        "SnrPerProcessedChunk",
        MakeCallback(&NrUeHandoverInterferenceTestCase::SaveSnr, this));

    // Capture the per-RB SINR (signal/(interference+noise)) of the reception.
    Ptr<NrChunkProcessor> sinrChunk = Create<NrChunkProcessor>();
    sinrChunk->AddCallback(MakeCallback(&NrSpectrumValueCatcher::ReportValue, &m_sinrCatcher));
    rxPhy->GetNrInterference()->AddSinrChunkProcessor(sinrChunk);

    Simulator::Schedule(MilliSeconds(0),
                        &NrSpectrumPhy::SetNoisePowerSpectralDensity,
                        rxPhy,
                        noisePsd);
    Simulator::Schedule(MilliSeconds(0), &MultiModelSpectrumChannel::AddRx, channel, rxPhy);

    // Both signals arrive at the same time, overlapping in frequency.
    Simulator::Schedule(MilliSeconds(1), &MultiModelSpectrumChannel::StartTx, channel, current);
    if (m_withStaleSignal)
    {
        Simulator::Schedule(MilliSeconds(1), &MultiModelSpectrumChannel::StartTx, channel, stale);
    }

    Simulator::Run();

    // The UE must have received exactly one chunk: the current cell's signal.
    // (If the stale signal had wrongly entered the receive path, the overlap
    // handling would have rejected one of them or the run would have aborted.)
    NS_TEST_ASSERT_MSG_EQ(m_snr.size(),
                          1,
                          "UE should receive exactly one (the current cell's) data chunk");

    Ptr<SpectrumValue> sinr = m_sinrCatcher.GetValue();
    NS_TEST_ASSERT_MSG_EQ(sinr != nullptr, true, "a SINR chunk should have been produced");

    double nBands = sinr->GetSpectrumModel()->GetNumBands();
    double avgSinr = Sum(*sinr) / nBands;
    double avgSnr = m_snr.at(0);

    NS_TEST_ASSERT_MSG_GT(avgSinr, 0.0, "UE must still receive the current cell (positive SINR)");

    if (m_withStaleSignal)
    {
        // The stale previous-cell signal is interference, so SINR < SNR.
        NS_TEST_ASSERT_MSG_LT(
            avgSinr,
            avgSnr,
            "the stale previous-cell signal must lower SINR below the interference-free SNR");
    }
    else
    {
        // No interferer: SINR equals SNR (interference == noise only).
        NS_TEST_ASSERT_MSG_EQ_TOL(avgSinr,
                                  avgSnr,
                                  avgSnr * 0.01,
                                  "with no interferer, SINR should match SNR");
    }

    rxPhy->Dispose();
    uePhyModel->Dispose();
    txPhy->Dispose();
    txPhyModel->Dispose();
    Simulator::Destroy();
}

/**
 * @ingroup tests
 *
 * @brief Test suite for UE reception during handover with a stale previous-cell signal.
 */
class NrUeHandoverInterferenceTestSuite : public TestSuite
{
  public:
    NrUeHandoverInterferenceTestSuite();
};

NrUeHandoverInterferenceTestSuite::NrUeHandoverInterferenceTestSuite()
    : TestSuite("nr-ue-handover-interference", Type::SYSTEM)
{
    // Baseline: only the current gNB transmits. SINR equals SNR.
    AddTestCase(new NrUeHandoverInterferenceTestCase("current cell only (baseline)", false),
                Duration::QUICK);
    // Handover: previous gNB still transmits on the same RBs. The UE receives
    // the current cell, interfered by the stale signal (SINR < SNR).
    AddTestCase(
        new NrUeHandoverInterferenceTestCase("current cell with stale previous-cell signal", true),
        Duration::QUICK);
}

static NrUeHandoverInterferenceTestSuite
    g_nrUeHandoverInterferenceTestSuite; ///< test suite instance

} // namespace ns3
