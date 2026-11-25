// Copyright (c) 2026 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only

#include "ns3/nr-chunk-processor.h"
#include "ns3/nr-interference-base.h"
#include "ns3/nr-interference.h"
#include "ns3/nr-mimo-chunk-processor.h"
#include "ns3/nr-spectrum-signal-parameters.h"
#include "ns3/simulator.h"
#include "ns3/spectrum-model.h"
#include "ns3/spectrum-value.h"
#include "ns3/test.h"

#include <cmath>
#include <limits>
#include <string>
#include <utility>
#include <vector>

/**
 * @file nr-interference-saturating-signal-test.cc
 * @ingroup test
 *
 * @brief Regression test for the corruption of the running sum of received
 * signals in NrInterferenceBase by a saturating signal.
 *
 * NrInterferenceBase keeps the total received power as a running sum: every
 * perceived PSD is added on arrival and subtracted when it expires. The
 * pairing is exact in real arithmetic but not in floating point: a signal of
 * sufficiently large power absorbs co-resident ordinary signals when added
 * (S + s rounds to S), so the paired subtractions leave the sum permanently
 * biased by the absorbed amount. An infinite signal (overflowed linear rx
 * power) is worse: its subtraction computes inf - inf = NaN. In both cases
 * the corruption used to outlive the saturating signal for the rest of the
 * simulation, poisoning every subsequent SISO SINR chunk at that receiver.
 *
 * The test perceives a saturating signal (finite or infinite) together with
 * an ordinary co-resident signal at an idle receiver, lets the saturating
 * signal expire, and then performs an ordinary reception, asserting the
 * expected SINR. It is run against both the SISO path (NrInterferenceBase)
 * and the MIMO path (NrInterference via AddSignalMimo/StartRxMimo), since
 * both feed the same running sum.
 */
namespace ns3
{

/**
 * @ingroup tests
 *
 * @brief Drives NrInterferenceBase with a saturating signal followed by an
 * ordinary reception and checks the SINR of the ordinary reception.
 */
class NrInterferenceSaturatingSignalTestCase : public TestCase
{
  public:
    /**
     * @brief Constructor.
     *
     * @param name human-readable test case name
     * @param saturatingPower power per RB of the saturating signal; 0 disables
     *        it (control case)
     * @param interfererOutlives whether the ordinary co-resident signal is
     *        still present during the final reception
     */
    NrInterferenceSaturatingSignalTestCase(std::string name,
                                           double saturatingPower,
                                           bool interfererOutlives);

  private:
    void DoRun() override;

    double m_saturatingPower;  ///< power per RB of the saturating signal (0 = none)
    bool m_interfererOutlives; ///< whether the ordinary signal covers the final reception
};

NrInterferenceSaturatingSignalTestCase::NrInterferenceSaturatingSignalTestCase(
    std::string name,
    double saturatingPower,
    bool interfererOutlives)
    : TestCase(std::move(name)),
      m_saturatingPower(saturatingPower),
      m_interfererOutlives(interfererOutlives)
{
}

void
NrInterferenceSaturatingSignalTestCase::DoRun()
{
    const uint32_t numRbs = 4;
    const double desiredPower = 4.0;
    const double interfererPower = 3.0;
    const double noisePower = 1.0;

    std::vector<double> centerFreqs(numRbs);
    for (uint32_t i = 0; i < numRbs; ++i)
    {
        centerFreqs[i] = 2.1e9 + i * 180e3;
    }
    Ptr<SpectrumModel> sm = Create<SpectrumModel>(centerFreqs);

    auto makePsd = [&sm](double power) {
        Ptr<SpectrumValue> psd = Create<SpectrumValue>(sm);
        for (uint32_t i = 0; i < numRbs; ++i)
        {
            (*psd)[i] = power;
        }
        return psd;
    };

    Ptr<SpectrumValue> noisePsd = makePsd(noisePower);
    Ptr<SpectrumValue> desiredPsd = makePsd(desiredPower);
    Ptr<SpectrumValue> interfererPsd = makePsd(interfererPower);
    Ptr<SpectrumValue> saturatingPsd = makePsd(m_saturatingPower);

    Ptr<NrInterferenceBase> interf = CreateObject<NrInterferenceBase>();
    Ptr<NrChunkProcessor> chunk = Create<NrChunkProcessor>();
    NrSpectrumValueCatcher sinrCatcher;
    chunk->AddCallback(MakeCallback(&NrSpectrumValueCatcher::ReportValue, &sinrCatcher));
    interf->AddSinrChunkProcessor(chunk);
    interf->SetNoisePowerSpectralDensity(noisePsd);

    const Time satDuration = MilliSeconds(1);
    const Time gap = MilliSeconds(1);
    const Time rxDuration = MilliSeconds(1);

    // The saturating signal and the ordinary co-resident signal are perceived
    // while the receiver is idle; the saturating signal expires after 1 ms.
    if (m_saturatingPower != 0.0)
    {
        interf->AddSignal(saturatingPsd, satDuration);
    }
    // When the interferer outlives the saturating signal, it stays present
    // during the final reception and must appear in its interference.
    interf->AddSignal(interfererPsd,
                      m_interfererOutlives ? satDuration + gap + rxDuration : satDuration);

    // Ordinary reception after the corruption window.
    Simulator::Schedule(satDuration + gap, &NrInterferenceBase::StartRx, interf, desiredPsd);
    Simulator::Schedule(satDuration + gap,
                        &NrInterferenceBase::AddSignal,
                        interf,
                        desiredPsd,
                        rxDuration);
    Simulator::Schedule(satDuration + gap + rxDuration, &NrInterferenceBase::EndRx, interf);
    Simulator::Run();

    Ptr<SpectrumValue> sinr = sinrCatcher.GetValue();
    NS_TEST_ASSERT_MSG_EQ(sinr != nullptr, true, "no SINR chunk was evaluated");
    double expectedInterf = noisePower + (m_interfererOutlives ? interfererPower : 0.0);
    double expectedSinr = desiredPower / expectedInterf;
    for (uint32_t i = 0; i < numRbs; ++i)
    {
        NS_TEST_ASSERT_MSG_EQ_TOL((*sinr)[i], expectedSinr, 1e-9, "unexpected SINR on RB " << i);
    }

    Simulator::Destroy();
}

/**
 * @ingroup tests
 *
 * @brief Same scenario as NrInterferenceSaturatingSignalTestCase, but the
 * signals enter through the MIMO path (NrInterference::AddSignalMimo), which
 * shares the base class running sum. Both the SISO SINR chunk and the MIMO
 * SINR chunk of the final reception are checked.
 */
class NrInterferenceSaturatingSignalMimoTestCase : public TestCase
{
  public:
    /**
     * @brief Constructor.
     *
     * @param name human-readable test case name
     * @param saturatingPower power per RB of the saturating signal; 0 disables
     *        it (control case)
     * @param interfererOutlives whether the ordinary co-resident signal is
     *        still present during the final reception
     */
    NrInterferenceSaturatingSignalMimoTestCase(std::string name,
                                               double saturatingPower,
                                               bool interfererOutlives);

  private:
    void DoRun() override;
    /**
     * @brief Store the SINR of the last processed MIMO chunk.
     * @param chunks the processed MIMO SINR chunks
     */
    void SaveMimoSinr(const std::vector<MimoSinrChunk>& chunks);

    double m_saturatingPower;       ///< power per RB of the saturating signal (0 = none)
    bool m_interfererOutlives;      ///< whether the ordinary signal covers the final reception
    std::vector<double> m_mimoSinr; ///< per-RB SINR of the last MIMO chunk
};

NrInterferenceSaturatingSignalMimoTestCase::NrInterferenceSaturatingSignalMimoTestCase(
    std::string name,
    double saturatingPower,
    bool interfererOutlives)
    : TestCase(std::move(name)),
      m_saturatingPower(saturatingPower),
      m_interfererOutlives(interfererOutlives)
{
}

void
NrInterferenceSaturatingSignalMimoTestCase::SaveMimoSinr(const std::vector<MimoSinrChunk>& chunks)
{
    if (!chunks.empty())
    {
        const auto& sinr = chunks.back().mimoSinr;
        m_mimoSinr.clear();
        for (size_t i = 0; i < sinr.GetNumRbs(); ++i)
        {
            m_mimoSinr.push_back(sinr(0, i));
        }
    }
}

void
NrInterferenceSaturatingSignalMimoTestCase::DoRun()
{
    const uint32_t numRbs = 4;
    const double desiredPower = 4.0;
    const double interfererPower = 3.0;
    const double noisePower = 1.0;

    std::vector<double> centerFreqs(numRbs);
    for (uint32_t i = 0; i < numRbs; ++i)
    {
        centerFreqs[i] = 2.1e9 + i * 180e3;
    }
    Ptr<SpectrumModel> sm = Create<SpectrumModel>(centerFreqs);

    // Build a 1x1 (SISO-shaped) data-frame signal whose channel matrix entries
    // are sqrt(power), so that |h|^2 reproduces the PSD and the MIMO SINR is
    // directly comparable to the SISO one.
    auto makeParams = [&sm, numRbs](double power) {
        Ptr<SpectrumValue> psd = Create<SpectrumValue>(sm);
        auto chan = Create<ComplexMatrixArray>(1, 1, numRbs);
        for (uint32_t i = 0; i < numRbs; ++i)
        {
            (*psd)[i] = power;
            chan->Elem(0, 0, i) = std::sqrt(power);
        }
        Ptr<NrSpectrumSignalParametersDataFrame> params =
            Create<NrSpectrumSignalParametersDataFrame>();
        params->psd = psd;
        params->spectrumChannelMatrix = chan;
        params->cellId = 1;
        params->rnti = 1;
        return params;
    };

    Ptr<SpectrumValue> noisePsd = Create<SpectrumValue>(sm);
    for (uint32_t i = 0; i < numRbs; ++i)
    {
        (*noisePsd)[i] = noisePower;
    }

    auto desired = makeParams(desiredPower);
    auto interferer = makeParams(interfererPower);
    auto saturating = makeParams(m_saturatingPower);

    Ptr<NrInterference> interf = CreateObject<NrInterference>();
    Ptr<NrChunkProcessor> chunk = Create<NrChunkProcessor>();
    NrSpectrumValueCatcher sinrCatcher;
    chunk->AddCallback(MakeCallback(&NrSpectrumValueCatcher::ReportValue, &sinrCatcher));
    interf->AddSinrChunkProcessor(chunk);
    Ptr<NrMimoChunkProcessor> mimoChunk = Create<NrMimoChunkProcessor>();
    mimoChunk->AddCallback(
        MakeCallback(&NrInterferenceSaturatingSignalMimoTestCase::SaveMimoSinr, this));
    interf->AddMimoChunkProcessor(mimoChunk);
    interf->SetNoisePowerSpectralDensity(noisePsd);

    const Time satDuration = MilliSeconds(1);
    const Time gap = MilliSeconds(1);
    const Time rxDuration = MilliSeconds(1);

    if (m_saturatingPower != 0.0)
    {
        interf->AddSignalMimo(saturating, satDuration);
    }
    interf->AddSignalMimo(interferer,
                          m_interfererOutlives ? satDuration + gap + rxDuration : satDuration);

    Simulator::Schedule(satDuration + gap,
                        &NrInterference::AddSignalMimo,
                        interf,
                        desired,
                        rxDuration);
    Simulator::Schedule(satDuration + gap, &NrInterference::StartRxMimo, interf, desired);
    Simulator::Schedule(satDuration + gap + rxDuration, &NrInterference::EndRx, interf);
    Simulator::Run();

    double expectedInterf = noisePower + (m_interfererOutlives ? interfererPower : 0.0);
    double expectedSinr = desiredPower / expectedInterf;

    Ptr<SpectrumValue> sinr = sinrCatcher.GetValue();
    NS_TEST_ASSERT_MSG_EQ(sinr != nullptr, true, "no SINR chunk was evaluated");
    for (uint32_t i = 0; i < numRbs; ++i)
    {
        NS_TEST_ASSERT_MSG_EQ_TOL((*sinr)[i],
                                  expectedSinr,
                                  1e-9,
                                  "unexpected SISO SINR on RB " << i);
    }

    NS_TEST_ASSERT_MSG_EQ(m_mimoSinr.size(), numRbs, "no MIMO SINR chunk was evaluated");
    for (uint32_t i = 0; i < numRbs; ++i)
    {
        NS_TEST_ASSERT_MSG_EQ_TOL(m_mimoSinr[i],
                                  expectedSinr,
                                  1e-9,
                                  "unexpected MIMO SINR on RB " << i);
    }

    Simulator::Destroy();
}

/**
 * @ingroup tests
 *
 * @brief Test suite for the saturating-signal corruption of the interference
 * running sum.
 */
class NrInterferenceSaturatingSignalTestSuite : public TestSuite
{
  public:
    NrInterferenceSaturatingSignalTestSuite();
};

NrInterferenceSaturatingSignalTestSuite::NrInterferenceSaturatingSignalTestSuite()
    : TestSuite("nr-interference-saturating-signal", Type::UNIT)
{
    const double finite = 1e30;
    const double infinite = std::numeric_limits<double>::infinity();

    // Control: no saturating signal, ordinary reception after an ordinary
    // co-resident signal expired. SINR = 4/1.
    AddTestCase(
        new NrInterferenceSaturatingSignalTestCase("control, expired interferer", 0.0, false),
        Duration::QUICK);
    // Finite saturating signal: without the fix the absorbed ordinary signal
    // leaves a negative bias, making the next SINR negative.
    AddTestCase(
        new NrInterferenceSaturatingSignalTestCase("finite saturating signal", finite, false),
        Duration::QUICK);
    // Infinite saturating signal: without the fix the sum turns NaN forever.
    AddTestCase(
        new NrInterferenceSaturatingSignalTestCase("infinite saturating signal", infinite, false),
        Duration::QUICK);
    // The absorbed ordinary signal is still present during the final
    // reception: it must still appear in the interference. SINR = 4/(3+1).
    AddTestCase(new NrInterferenceSaturatingSignalTestCase("finite saturating, live interferer",
                                                           finite,
                                                           true),
                Duration::QUICK);

    // Same scenarios through the MIMO signal path (AddSignalMimo), which feeds
    // the same running sum.
    AddTestCase(new NrInterferenceSaturatingSignalMimoTestCase("mimo control, expired interferer",
                                                               0.0,
                                                               false),
                Duration::QUICK);
    AddTestCase(new NrInterferenceSaturatingSignalMimoTestCase("mimo finite saturating signal",
                                                               finite,
                                                               false),
                Duration::QUICK);
    AddTestCase(new NrInterferenceSaturatingSignalMimoTestCase("mimo infinite saturating signal",
                                                               infinite,
                                                               false),
                Duration::QUICK);
    AddTestCase(new NrInterferenceSaturatingSignalMimoTestCase("mimo finite saturating, live "
                                                               "interferer",
                                                               finite,
                                                               true),
                Duration::QUICK);
}

static NrInterferenceSaturatingSignalTestSuite
    g_nrInterferenceSaturatingSignalTestSuite; ///< the test suite instance

} // namespace ns3
