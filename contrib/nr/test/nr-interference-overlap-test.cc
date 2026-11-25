// Copyright (c) 2026 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only

#include "ns3/nr-chunk-processor.h"
#include "ns3/nr-interference-base.h"
#include "ns3/simulator.h"
#include "ns3/spectrum-model.h"
#include "ns3/spectrum-value.h"
#include "ns3/test.h"

#include <utility>
#include <vector>

/**
 * @file nr-interference-overlap-test.cc
 * @ingroup test
 *
 * @brief Unit test for the simultaneous-signal handling in
 * NrInterferenceBase::StartRx.
 *
 * When a PHY is already receiving and a second simultaneous signal arrives,
 * NrInterferenceBase::StartRx keeps the first signal and only accepts the
 * second one if it is orthogonal in frequency (disjoint resource blocks). A
 * non-orthogonal (overlapping) second signal is rejected from the desired
 * reception but, because it remains in the all-signals set, it is still
 * accounted for as interference. This test confirms both branches by checking
 * the return value of StartRx and the resulting per-RB SINR.
 */
namespace ns3
{

/**
 * @ingroup tests
 *
 * @brief Drives NrInterferenceBase with a desired signal plus one simultaneous
 * signal and checks acceptance and the resulting SINR.
 *
 * The desired signal occupies RBs {0,1}. A second signal of configurable RB
 * occupancy arrives at the same time. The test asserts whether the second
 * signal is accepted into the desired reception and verifies the per-RB SINR
 * computed by the interference model.
 */
class NrInterferenceOverlapTestCase : public TestCase
{
  public:
    /**
     * @brief Constructor.
     *
     * @param name human-readable test case name
     * @param secondRbs resource blocks occupied by the second simultaneous signal
     * @param expectedSecondAccepted expected return value of StartRx for the second signal
     * @param expectedSinr expected per-RB SINR after the reception
     */
    NrInterferenceOverlapTestCase(std::string name,
                                  std::vector<uint32_t> secondRbs,
                                  bool expectedSecondAccepted,
                                  std::vector<double> expectedSinr);

  private:
    void DoRun() override;

    std::vector<uint32_t> m_secondRbs;  ///< RBs of the second simultaneous signal
    bool m_expectedSecondAccepted;      ///< expected acceptance of the second signal
    std::vector<double> m_expectedSinr; ///< expected per-RB SINR
};

NrInterferenceOverlapTestCase::NrInterferenceOverlapTestCase(std::string name,
                                                             std::vector<uint32_t> secondRbs,
                                                             bool expectedSecondAccepted,
                                                             std::vector<double> expectedSinr)
    : TestCase(std::move(name)),
      m_secondRbs(std::move(secondRbs)),
      m_expectedSecondAccepted(expectedSecondAccepted),
      m_expectedSinr(std::move(expectedSinr))
{
}

void
NrInterferenceOverlapTestCase::DoRun()
{
    // Linear power units; the model is linear so realistic dBm values are not needed.
    const uint32_t numRbs = 4;
    const double desiredPower = 4.0; // desired signal power per occupied RB
    const double secondPower = 3.0;  // second signal power per occupied RB
    const double noisePower = 1.0;   // noise power per RB

    // Build a flat spectrum model with numRbs resource blocks.
    std::vector<double> centerFreqs(numRbs);
    for (uint32_t i = 0; i < numRbs; ++i)
    {
        centerFreqs[i] = 2.1e9 + i * 180e3;
    }
    Ptr<SpectrumModel> sm = Create<SpectrumModel>(centerFreqs);

    auto makePsd = [&sm](const std::vector<uint32_t>& rbs, double power) {
        Ptr<SpectrumValue> psd = Create<SpectrumValue>(sm);
        for (auto rb : rbs)
        {
            (*psd)[rb] = power;
        }
        return psd;
    };

    Ptr<SpectrumValue> noisePsd = makePsd({0, 1, 2, 3}, noisePower);
    Ptr<SpectrumValue> desiredPsd = makePsd({0, 1}, desiredPower);
    Ptr<SpectrumValue> secondPsd = makePsd(m_secondRbs, secondPower);

    Ptr<NrInterferenceBase> interf = CreateObject<NrInterferenceBase>();
    Ptr<NrChunkProcessor> chunk = Create<NrChunkProcessor>();
    NrSpectrumValueCatcher sinrCatcher;
    chunk->AddCallback(MakeCallback(&NrSpectrumValueCatcher::ReportValue, &sinrCatcher));
    interf->AddSinrChunkProcessor(chunk);
    interf->SetNoisePowerSpectralDensity(noisePsd);

    const Time duration = MilliSeconds(1);

    // Every perceived signal enters the all-signals (interference) set, mirroring
    // NrSpectrumPhy::StartRx which calls AddSignal for every incoming signal
    // regardless of whether it ends up being part of the desired reception.
    interf->AddSignal(desiredPsd, duration);
    interf->AddSignal(secondPsd, duration);

    // The desired signal arrives first and is always accepted.
    bool firstAccepted = interf->StartRx(desiredPsd);
    NS_TEST_ASSERT_MSG_EQ(firstAccepted, true, "the first signal must always be accepted");

    // The second simultaneous signal is accepted into the desired reception only
    // if it is orthogonal; an overlapping one is rejected (and stays as interference).
    bool secondAccepted = interf->StartRx(secondPsd);
    NS_TEST_ASSERT_MSG_EQ(secondAccepted,
                          m_expectedSecondAccepted,
                          "second signal acceptance does not match the orthogonality of its RBs");

    Simulator::Schedule(duration, &NrInterferenceBase::EndRx, interf);
    Simulator::Run();

    Ptr<SpectrumValue> sinr = sinrCatcher.GetValue();
    NS_TEST_ASSERT_MSG_EQ(sinr != nullptr, true, "no SINR chunk was evaluated");
    for (uint32_t i = 0; i < numRbs; ++i)
    {
        NS_TEST_ASSERT_MSG_EQ_TOL((*sinr)[i],
                                  m_expectedSinr[i],
                                  1e-9,
                                  "unexpected SINR on RB " << i);
    }

    Simulator::Destroy();
}

/**
 * @ingroup tests
 *
 * @brief Test suite for NrInterferenceBase::StartRx simultaneous-signal handling.
 */
class NrInterferenceOverlapTestSuite : public TestSuite
{
  public:
    NrInterferenceOverlapTestSuite();
};

NrInterferenceOverlapTestSuite::NrInterferenceOverlapTestSuite()
    : TestSuite("nr-interference-overlap", Type::UNIT)
{
    // Orthogonal second signal on RBs {2,3}: accepted and accumulated into the
    // desired reception, so each RB carries its own signal over noise and there
    // is no self-interference. SINR = power/noise on each occupied RB.
    AddTestCase(new NrInterferenceOverlapTestCase("orthogonal second signal accepted",
                                                  {2, 3},
                                                  true,
                                                  {4.0, 4.0, 3.0, 3.0}),
                Duration::QUICK);

    // Overlapping second signal on RBs {1,2} (shares RB 1 with the desired
    // signal): rejected from the desired reception but kept as interference.
    // RB 0: desired only, SINR = 4/1 = 4.
    // RB 1: desired vs interferer, SINR = 4/(3+1) = 1 (degraded by interference).
    // RB 2: no desired signal (interferer was rejected, not received), SINR = 0.
    // RB 3: empty, SINR = 0.
    AddTestCase(new NrInterferenceOverlapTestCase("overlapping second signal rejected",
                                                  {1, 2},
                                                  false,
                                                  {4.0, 1.0, 0.0, 0.0}),
                Duration::QUICK);
}

static NrInterferenceOverlapTestSuite g_nrInterferenceOverlapTestSuite; ///< the test suite instance

} // namespace ns3
