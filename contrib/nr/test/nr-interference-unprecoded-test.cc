// Copyright (c) 2026 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only

#include "ns3/nr-interference.h"
#include "ns3/nr-mimo-chunk-processor.h"
#include "ns3/nr-spectrum-signal-parameters.h"
#include "ns3/simulator.h"
#include "ns3/spectrum-model.h"
#include "ns3/spectrum-value.h"
#include "ns3/test.h"

#include <cmath>
#include <string>
#include <utility>
#include <vector>

/**
 * @file nr-interference-unprecoded-test.cc
 * @ingroup test
 *
 * @brief Test of the power of unprecoded multi-port interferers in the MIMO SINR.
 *
 * Every transmit port of a spectrum channel matrix carries the full transmit power
 * spectral density, and precoders are normalized to the total transmit power. The
 * interference of a signal without precoding matrix, like the DL control or the
 * CSI-RS of a neighbor cell, used to add the channel of every port at full power,
 * counting the interferer as many times as it has ports (16 times, or 12 dB, for a
 * dual-polarized gNB with 8 horizontal ports), while the desired unprecoded signal
 * is evaluated with all its power on the first port. The test checks the MIMO SINR of
 * a reception with an unprecoded or a precoded interferer of several ports.
 */
namespace ns3
{

/**
 * @ingroup tests
 *
 * @brief Checks the MIMO SINR of a reception with an interferer of several transmit
 * ports, with or without precoding.
 */
class NrInterferenceUnprecodedTestCase : public TestCase
{
  public:
    /**
     * @brief Constructor.
     *
     * @param nTxPorts number of transmit ports of the desired and interfering signals
     * @param precodedInterferer whether the interferer has a precoding matrix
     */
    NrInterferenceUnprecodedTestCase(size_t nTxPorts, bool precodedInterferer);

  private:
    void DoRun() override;
    /**
     * @brief Store the SINR of the last processed MIMO chunk.
     * @param chunks the processed MIMO SINR chunks
     */
    void SaveMimoSinr(const std::vector<MimoSinrChunk>& chunks);

    size_t m_nTxPorts;              ///< number of transmit ports
    bool m_precodedInterferer;      ///< whether the interferer has a precoding matrix
    std::vector<double> m_mimoSinr; ///< per-RB SINR of the last MIMO chunk
};

NrInterferenceUnprecodedTestCase::NrInterferenceUnprecodedTestCase(size_t nTxPorts,
                                                                   bool precodedInterferer)
    : TestCase(std::to_string(nTxPorts) + " transmit ports, " +
               (precodedInterferer ? "precoded" : "unprecoded") + " interferer"),
      m_nTxPorts(nTxPorts),
      m_precodedInterferer(precodedInterferer)
{
}

void
NrInterferenceUnprecodedTestCase::SaveMimoSinr(const std::vector<MimoSinrChunk>& chunks)
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
NrInterferenceUnprecodedTestCase::DoRun()
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

    // One receive port; every transmit port sees the channel sqrt(power), the scaling
    // of the spectrum channel matrix of a signal of that power spectral density
    auto makeParams = [&sm, numRbs, this](double power, uint16_t cellId) {
        Ptr<SpectrumValue> psd = Create<SpectrumValue>(sm);
        auto chan = Create<ComplexMatrixArray>(1, m_nTxPorts, numRbs);
        for (uint32_t i = 0; i < numRbs; ++i)
        {
            (*psd)[i] = power;
            for (size_t s = 0; s < m_nTxPorts; ++s)
            {
                chan->Elem(0, s, i) = std::sqrt(power);
            }
        }
        Ptr<NrSpectrumSignalParametersDataFrame> params =
            Create<NrSpectrumSignalParametersDataFrame>();
        params->psd = psd;
        params->spectrumChannelMatrix = chan;
        params->cellId = cellId;
        params->rnti = 1;
        return params;
    };

    Ptr<SpectrumValue> noisePsd = Create<SpectrumValue>(sm);
    for (uint32_t i = 0; i < numRbs; ++i)
    {
        (*noisePsd)[i] = noisePower;
    }

    auto desired = makeParams(desiredPower, 1);
    auto interferer = makeParams(interfererPower, 2);
    // Rank-1 precoder with unit norm, coherent over the ports: the interference then
    // gets the array gain of the ports
    double expectedInterference = interfererPower;
    if (m_precodedInterferer)
    {
        auto precMats = Create<ComplexMatrixArray>(m_nTxPorts, 1, numRbs);
        for (uint32_t i = 0; i < numRbs; ++i)
        {
            for (size_t s = 0; s < m_nTxPorts; ++s)
            {
                precMats->Elem(s, 0, i) = 1.0 / std::sqrt(m_nTxPorts);
            }
        }
        interferer->precodingMatrix = precMats;
        expectedInterference = interfererPower * m_nTxPorts;
    }

    Ptr<NrInterference> interf = CreateObject<NrInterference>();
    Ptr<NrMimoChunkProcessor> mimoChunk = Create<NrMimoChunkProcessor>();
    mimoChunk->AddCallback(MakeCallback(&NrInterferenceUnprecodedTestCase::SaveMimoSinr, this));
    interf->AddMimoChunkProcessor(mimoChunk);
    interf->SetNoisePowerSpectralDensity(noisePsd);

    const Time rxDuration = MilliSeconds(1);
    interf->AddSignalMimo(interferer, rxDuration);
    interf->AddSignalMimo(desired, rxDuration);
    interf->StartRxMimo(desired);
    Simulator::Schedule(rxDuration, &NrInterference::EndRx, interf);
    Simulator::Run();

    // The desired signal has no precoding matrix either: all its power is on the first port
    const double expectedSinr = desiredPower / (noisePower + expectedInterference);
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
 * @brief Test suite for the power of unprecoded multi-port interferers.
 */
class NrInterferenceUnprecodedTestSuite : public TestSuite
{
  public:
    NrInterferenceUnprecodedTestSuite();
};

NrInterferenceUnprecodedTestSuite::NrInterferenceUnprecodedTestSuite()
    : TestSuite("nr-interference-unprecoded", Type::UNIT)
{
    for (size_t nTxPorts : {1, 2, 16})
    {
        AddTestCase(new NrInterferenceUnprecodedTestCase(nTxPorts, false), Duration::QUICK);
        AddTestCase(new NrInterferenceUnprecodedTestCase(nTxPorts, true), Duration::QUICK);
    }
}

static NrInterferenceUnprecodedTestSuite
    g_nrInterferenceUnprecodedTestSuite; ///< the test suite instance

} // namespace ns3
