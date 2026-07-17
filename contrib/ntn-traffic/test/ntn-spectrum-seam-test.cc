/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only
//
// CI gate 4 (audit gap S1): the NtnSpectrumSeamModel must COMPOSE a module
// plugin ON TOP OF the 3GPP phased-array channel, not replace it. Before the
// fix, installing a plain SpectrumPropagationLossModel switched the whole 3GPP
// spatial channel (array gain, fading, the MIMO channel matrix) OFF, because
// MultiModelSpectrumChannel applies spectrum-loss ELSE-IF phased-array-loss.
// This test drives the seam directly with a known inner model and a known
// plugin and asserts the PSD is (inner then plugin), proving composition.

#include "ns3/constant-position-mobility-model.h"
#include "ns3/ntn-spectrum-seam-model.h"
#include "ns3/spectrum-signal-parameters.h"
#include "ns3/spectrum-value.h"
#include "ns3/test.h"
#include "ns3/uniform-planar-array.h"

using namespace ns3;

namespace
{

/// A plugin that scales the received PSD by a fixed factor (stands in for a
/// THz / Sionna / RIS per-RB transfer function).
class ScalingPlugin : public SpectrumPropagationLossModel
{
  public:
    explicit ScalingPlugin(double factor)
        : m_factor(factor)
    {
    }

    static TypeId GetTypeId()
    {
        static TypeId tid = TypeId("ns3::ScalingPlugin")
                                .SetParent<SpectrumPropagationLossModel>()
                                .SetGroupName("Test");
        return tid;
    }

  private:
    Ptr<SpectrumValue> DoCalcRxPowerSpectralDensity(Ptr<const SpectrumSignalParameters> params,
                                                    Ptr<const MobilityModel>,
                                                    Ptr<const MobilityModel>) const override
    {
        Ptr<SpectrumValue> out = params->psd->Copy();
        *out *= m_factor;
        return out;
    }

    int64_t DoAssignStreams(int64_t) override { return 0; }

    double m_factor;
};

/// A phased-array inner model that scales the PSD by a fixed factor — stands in
/// for the 3GPP channel's array gain, so we can prove the seam runs it FIRST.
class ScalingPhasedInner : public PhasedArraySpectrumPropagationLossModel
{
  public:
    explicit ScalingPhasedInner(double factor)
        : m_factor(factor)
    {
    }

    static TypeId GetTypeId()
    {
        static TypeId tid = TypeId("ns3::ScalingPhasedInner")
                                .SetParent<PhasedArraySpectrumPropagationLossModel>()
                                .SetGroupName("Test");
        return tid;
    }

  private:
    Ptr<SpectrumSignalParameters> DoCalcRxPowerSpectralDensity(
        Ptr<const SpectrumSignalParameters> params,
        Ptr<const MobilityModel>,
        Ptr<const MobilityModel>,
        Ptr<const PhasedArrayModel>,
        Ptr<const PhasedArrayModel>) const override
    {
        Ptr<SpectrumSignalParameters> rx = params->Copy();
        Ptr<SpectrumValue> psd = params->psd->Copy();
        *psd *= m_factor;
        rx->psd = psd;
        return rx;
    }

    int64_t DoAssignStreams(int64_t) override { return 0; }

    double m_factor;
};

Ptr<SpectrumSignalParameters>
MakeParams(double psdValue)
{
    std::vector<double> freqs = {2.0e9, 2.1e9};
    Ptr<SpectrumModel> model = Create<SpectrumModel>(freqs);
    Ptr<SpectrumValue> psd = Create<SpectrumValue>(model);
    (*psd)[0] = psdValue;
    (*psd)[1] = psdValue;
    Ptr<SpectrumSignalParameters> p = Create<SpectrumSignalParameters>();
    p->psd = psd;
    return p;
}

} // namespace

/// Gate 4: a plugin installed on the seam is applied, and when an inner phased-
/// array model is present the seam runs BOTH (inner first, plugin second) — it
/// composes, it does not replace.
class NtnSpectrumSeamComposeTest : public TestCase
{
  public:
    NtnSpectrumSeamComposeTest()
        : TestCase("NtnSpectrumSeamModel composes plugin ON TOP of the 3GPP channel (S1/gate 4)")
    {
    }

  private:
    void DoRun() override
    {
        Ptr<ConstantPositionMobilityModel> a = CreateObject<ConstantPositionMobilityModel>();
        Ptr<ConstantPositionMobilityModel> b = CreateObject<ConstantPositionMobilityModel>();
        b->SetPosition(Vector(1000.0, 0.0, 0.0));
        Ptr<UniformPlanarArray> arr = CreateObject<UniformPlanarArray>();

        // Case 1: no inner, one halving plugin -> PSD halved.
        {
            Ptr<NtnSpectrumSeamModel> seam = CreateObject<NtnSpectrumSeamModel>();
            seam->AddPlugin(Create<ScalingPlugin>(0.5));
            NS_TEST_ASSERT_MSG_EQ(seam->GetNumPlugins(), 1u, "plugin registered");
            auto rx = seam->CalcRxPowerSpectralDensity(MakeParams(100.0), a, b, arr, arr);
            NS_TEST_ASSERT_MSG_EQ_TOL((*rx->psd)[0], 50.0, 1e-9,
                                      "plugin must be applied to the PSD");
        }

        // Case 2: inner doubles, plugin halves -> composed = 100 * 2 * 0.5 = 100.
        // (If the seam REPLACED the inner with the plugin, the result would be
        // 50 -- the inner's array gain would be lost, which is exactly the S1
        // bug this test guards.)
        {
            Ptr<NtnSpectrumSeamModel> seam = CreateObject<NtnSpectrumSeamModel>();
            seam->SetInnerModel(Create<ScalingPhasedInner>(2.0));
            seam->AddPlugin(Create<ScalingPlugin>(0.5));
            auto rx = seam->CalcRxPowerSpectralDensity(MakeParams(100.0), a, b, arr, arr);
            NS_TEST_ASSERT_MSG_EQ_TOL(
                (*rx->psd)[0], 100.0, 1e-9,
                "seam must run inner (x2) THEN plugin (x0.5) = x1, not replace inner with plugin");
        }

        // Case 3: inner alone (no plugin) passes through unchanged (x2).
        {
            Ptr<NtnSpectrumSeamModel> seam = CreateObject<NtnSpectrumSeamModel>();
            seam->SetInnerModel(Create<ScalingPhasedInner>(2.0));
            auto rx = seam->CalcRxPowerSpectralDensity(MakeParams(100.0), a, b, arr, arr);
            NS_TEST_ASSERT_MSG_EQ_TOL((*rx->psd)[0], 200.0, 1e-9,
                                      "with no plugin the inner 3GPP channel is untouched");
        }
    }
};

class NtnSpectrumSeamTestSuite : public TestSuite
{
  public:
    NtnSpectrumSeamTestSuite()
        : TestSuite("ntn-spectrum-seam", Type::UNIT)
    {
        AddTestCase(new NtnSpectrumSeamComposeTest, Duration::QUICK);
    }
};

static NtnSpectrumSeamTestSuite g_ntnSpectrumSeamTestSuite;
