/*
 * SPDX-License-Identifier: GPL-2.0-only
 * Copyright (c) 2026 Muhammad Uzair (ns3-ntn-toolkit)
 *
 * The tapped-delay-line must reach a spectrum channel and vary ACROSS the band.
 *
 * NtnTdlSpectrumLossModel was correct, complete and unreachable: it is a plain
 * Object, not a SpectrumPropagationLossModel, so nothing could attach it, and
 * its ApplyTo() had two callers in the tree, both inside its own unit test.
 * NtnTdlSpectrumPropagationLossModel is the adapter that closes that. This
 * asserts the adapter actually shapes a PSD, that the shaping is frequency
 * SELECTIVE rather than a flat scale, and that it does not invent energy.
 */
#include "ns3/constant-position-mobility-model.h"
#include "ns3/ntn-tdl-spectrum-propagation-loss-model.h"
#include "ns3/spectrum-model.h"
#include "ns3/spectrum-signal-parameters.h"
#include "ns3/spectrum-value.h"
#include "ns3/test.h"

#include <cmath>
#include <vector>

using namespace ns3;

class NtnTdlIsFrequencySelectiveTestCase : public TestCase
{
  public:
    NtnTdlIsFrequencySelectiveTestCase()
        : TestCase("TDL adapter shapes a PSD across the band and conserves mean power")
    {
    }

  private:
    void DoRun() override
    {
        // 100 subcarriers over 20 MHz at S band. The delay spread of the
        // stand-in profile sets a coherence bandwidth well inside this, which is
        // the whole point: a flat model cannot produce variation here.
        const double fc = 2.0e9;
        const double bw = 20e6;
        const uint32_t n = 100;
        Bands bands;
        for (uint32_t i = 0; i < n; ++i)
        {
            BandInfo bi;
            bi.fc = fc - bw / 2 + (bw / n) * (i + 0.5);
            bi.fl = bi.fc - bw / (2 * n);
            bi.fh = bi.fc + bw / (2 * n);
            bands.push_back(bi);
        }
        Ptr<SpectrumModel> sm = Create<SpectrumModel>(bands);
        Ptr<SpectrumValue> psd = Create<SpectrumValue>(sm);
        for (uint32_t i = 0; i < n; ++i)
        {
            (*psd)[i] = 1.0;
        }

        Ptr<SpectrumSignalParameters> params = Create<SpectrumSignalParameters>();
        params->psd = psd;

        // Ground terminal on the globe, satellite 600 km overhead, so the
        // adapter takes the geocentric-up branch as a real scenario would.
        Ptr<ConstantPositionMobilityModel> ue = CreateObject<ConstantPositionMobilityModel>();
        Ptr<ConstantPositionMobilityModel> sat = CreateObject<ConstantPositionMobilityModel>();
        ue->SetPosition(Vector(6371000.0, 0.0, 0.0));
        sat->SetPosition(Vector(6371000.0 + 600e3, 0.0, 0.0));

        Ptr<NtnTdlSpectrumPropagationLossModel> tdl =
            CreateObject<NtnTdlSpectrumPropagationLossModel>();

        Ptr<SpectrumValue> rx = tdl->CalcRxPowerSpectralDensity(params, ue, sat);

        NS_TEST_ASSERT_MSG_EQ(tdl->GetAppliedCount(), 1u,
                              "the adapter must have shaped the PSD, not skipped it");
        NS_TEST_ASSERT_MSG_EQ(tdl->GetSkippedCount(), 0u, "nothing should have been skipped");

        double minG = 1e300;
        double maxG = -1e300;
        double sum = 0.0;
        for (uint32_t i = 0; i < n; ++i)
        {
            const double g = (*rx)[i];
            NS_TEST_ASSERT_MSG_EQ(std::isfinite(g), true, "subcarrier " << i << " is not finite");
            NS_TEST_ASSERT_MSG_GT_OR_EQ(g, 0.0, "a power gain cannot be negative");
            minG = std::min(minG, g);
            maxG = std::max(maxG, g);
            sum += g;
        }

        // Frequency selectivity is the claim. A flat model would give
        // max == min; a tapped-delay-line must not. Measured on the stand-in
        // profile: 2.34 dB of spread across 20 MHz, min 0.738, max 1.265, mean
        // 1.024. Replacing ApplyTo with a flat scale drops the spread to 0 dB
        // and fails this case, so the bound is doing work.
        const double spreadDb = 10.0 * std::log10(maxG / std::max(minG, 1e-12));
        NS_TEST_ASSERT_MSG_GT(spreadDb, 1.0,
                              "the transfer function varies only " << spreadDb
                                  << " dB across 20 MHz, which is a flat channel wearing a "
                                     "tapped-delay-line's name");

        // And it must not invent energy: the profile is normalised to unit mean.
        const double meanG = sum / n;
        NS_TEST_ASSERT_MSG_LT(std::abs(10.0 * std::log10(std::max(meanG, 1e-12))), 3.0,
                              "mean gain " << meanG
                                  << " is more than 3 dB off unity, so the fading model is "
                                     "adding or removing average power");
    }
};

class NtnTdlSpectrumTestSuite : public TestSuite
{
  public:
    NtnTdlSpectrumTestSuite()
        : TestSuite("ntn-tdl-spectrum", Type::UNIT)
    {
        AddTestCase(new NtnTdlIsFrequencySelectiveTestCase, Duration::QUICK);
    }
};

static NtnTdlSpectrumTestSuite g_ntnTdlSpectrumTestSuite;
