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
#include "ns3/matrix-array.h"
#include "ns3/multi-model-spectrum-channel.h"
#include "ns3/node-container.h"
#include "ns3/ntn-real-stack-helper.h"
#include "ns3/ntn-spectrum-seam-model.h"
#include "ns3/ntn-tdl-spectrum-propagation-loss-model.h"
#include "ns3/random-variable-stream.h"
#include "ns3/rng-seed-manager.h"
#include "ns3/simulator.h"
#include "ns3/spectrum-phy.h"
#include "ns3/spectrum-signal-parameters.h"
#include "ns3/spectrum-value.h"
#include "ns3/test.h"
#include "ns3/uniform-planar-array.h"

#include <cmath>
#include <complex>
#include <iostream>
#include <vector>

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

/// A phased-array inner model that scales the PSD by a fixed factor; it stands in
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
/// array model is present the seam runs BOTH (inner first, plugin second): it
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

// ---------------------------------------------------------------------------
// In-depth cases (Nov 2026 test campaign). The live cases install the seam on
// a MultiModelSpectrumChannel exactly as NtnRealStackHelper does (phased-array
// slot), between two probe PHYs that carry a UniformPlanarArray, and record
// every PSD and channel matrix the channel delivers across Simulator::Run().
// ---------------------------------------------------------------------------

namespace
{

/// out = psd x a + b on every bin: affine, so the order of two plugins shows.
class AffinePlugin : public SpectrumPropagationLossModel
{
  public:
    AffinePlugin(double a, double b)
        : m_a(a),
          m_b(b)
    {
    }

    static TypeId GetTypeId()
    {
        static TypeId tid = TypeId("ns3::NtnSeamTestAffinePlugin")
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
        *out *= m_a;
        *out += m_b;
        return out;
    }

    int64_t DoAssignStreams(int64_t) override
    {
        return 0;
    }

    double m_a;
    double m_b;
};

/// Multiplies bin k by gains[k] (a per-RB transfer function).
class PerRbPlugin : public SpectrumPropagationLossModel
{
  public:
    explicit PerRbPlugin(std::vector<double> g)
        : m_g(std::move(g))
    {
    }

    static TypeId GetTypeId()
    {
        static TypeId tid = TypeId("ns3::NtnSeamTestPerRbPlugin")
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
        for (std::size_t k = 0; k < out->GetValuesN() && k < m_g.size(); ++k)
        {
            (*out)[k] *= m_g[k];
        }
        return out;
    }

    int64_t DoAssignStreams(int64_t) override
    {
        return 0;
    }

    std::vector<double> m_g;
};

/// A plugin that fails: returns no PSD.
class NullPlugin : public SpectrumPropagationLossModel
{
  public:
    static TypeId GetTypeId()
    {
        static TypeId tid = TypeId("ns3::NtnSeamTestNullPlugin")
                                .SetParent<SpectrumPropagationLossModel>()
                                .SetGroupName("Test");
        return tid;
    }

  private:
    Ptr<SpectrumValue> DoCalcRxPowerSpectralDensity(Ptr<const SpectrumSignalParameters>,
                                                    Ptr<const MobilityModel>,
                                                    Ptr<const MobilityModel>) const override
    {
        return nullptr;
    }

    int64_t DoAssignStreams(int64_t) override
    {
        return 0;
    }
};

/// Inner phased-array model that scales the PSD and attaches a known 2x2
/// channel matrix per bin, H(r, c, k) = (1 + r + 2c) + j k, standing in for the
/// 3GPP channel's spectrumChannelMatrix. It optionally carries more matrix
/// pages than PSD bins.
class MatrixInner : public PhasedArraySpectrumPropagationLossModel
{
  public:
    MatrixInner(double factor, std::size_t extraPages)
        : m_factor(factor),
          m_extra(extraPages)
    {
    }

    static TypeId GetTypeId()
    {
        static TypeId tid = TypeId("ns3::NtnSeamTestMatrixInner")
                                .SetParent<PhasedArraySpectrumPropagationLossModel>()
                                .SetGroupName("Test");
        return tid;
    }

    static std::complex<double> H(std::size_t r, std::size_t c, std::size_t k)
    {
        return {1.0 + r + 2.0 * c, static_cast<double>(k)};
    }

  private:
    Ptr<SpectrumSignalParameters> DoCalcRxPowerSpectralDensity(Ptr<const SpectrumSignalParameters> params,
                                                               Ptr<const MobilityModel>,
                                                               Ptr<const MobilityModel>,
                                                               Ptr<const PhasedArrayModel>,
                                                               Ptr<const PhasedArrayModel>) const override
    {
        Ptr<SpectrumSignalParameters> rx = params->Copy();
        Ptr<SpectrumValue> psd = params->psd->Copy();
        *psd *= m_factor;
        rx->psd = psd;
        const std::size_t pages = psd->GetValuesN() + m_extra;
        auto m = Create<ComplexMatrixArray>(2, 2, pages);
        for (std::size_t k = 0; k < pages; ++k)
        {
            for (std::size_t r = 0; r < 2; ++r)
            {
                for (std::size_t c = 0; c < 2; ++c)
                {
                    m->Elem(r, c, k) = H(r, c, k);
                }
            }
        }
        rx->spectrumChannelMatrix = m;
        return rx;
    }

    int64_t DoAssignStreams(int64_t) override
    {
        return 0;
    }

    double m_factor;
    std::size_t m_extra;
};

/// Probe PHY with a phased array, so MultiModelSpectrumChannel takes the
/// phased-array branch and calls the seam.
class ArrayProbePhy : public SpectrumPhy
{
  public:
    static TypeId GetTypeId()
    {
        static TypeId tid =
            TypeId("ns3::NtnSeamTestArrayProbePhy").SetParent<SpectrumPhy>().SetGroupName("Test");
        return tid;
    }

    ArrayProbePhy()
        : m_array(CreateObject<UniformPlanarArray>())
    {
    }

    void SetDevice(Ptr<NetDevice>) override
    {
    }

    Ptr<NetDevice> GetDevice() const override
    {
        return nullptr;
    }

    void SetMobility(Ptr<MobilityModel> m) override
    {
        m_mobility = m;
    }

    Ptr<MobilityModel> GetMobility() const override
    {
        return m_mobility;
    }

    void SetChannel(Ptr<SpectrumChannel> c) override
    {
        m_channel = c;
    }

    Ptr<const SpectrumModel> GetRxSpectrumModel() const override
    {
        return m_model;
    }

    Ptr<Object> GetAntenna() const override
    {
        return m_array;
    }

    void StartRx(Ptr<SpectrumSignalParameters> params) override
    {
        m_rx.push_back(params);
    }

    void Send(Ptr<SpectrumValue> psd)
    {
        Ptr<SpectrumSignalParameters> p = Create<SpectrumSignalParameters>();
        p->psd = psd->Copy();
        p->duration = MicroSeconds(100);
        p->txPhy = this;
        m_channel->StartTx(p);
    }

    void SetModel(Ptr<const SpectrumModel> m)
    {
        m_model = m;
    }

    std::vector<Ptr<SpectrumSignalParameters>> m_rx;

  private:
    Ptr<UniformPlanarArray> m_array;
    Ptr<MobilityModel> m_mobility;
    Ptr<SpectrumChannel> m_channel;
    Ptr<const SpectrumModel> m_model;
};

/// Run \p n transmissions of a flat PSD of value \p value over a
/// MultiModelSpectrumChannel carrying \p seam in its phased-array slot.
/// Returns what the receiver got. \p txPsd receives the transmitted PSD so a
/// case can check it was not modified.
std::vector<Ptr<SpectrumSignalParameters>>
LiveSeam(Ptr<NtnSpectrumSeamModel> seam, uint32_t nBins, double value, uint32_t n, Ptr<SpectrumValue>* txPsd = nullptr)
{
    std::vector<double> freqs;
    for (uint32_t k = 0; k < nBins; ++k)
    {
        freqs.push_back(2.0e9 + 180e3 * k);
    }
    Ptr<SpectrumModel> sm = Create<SpectrumModel>(freqs);
    Ptr<MultiModelSpectrumChannel> ch = CreateObject<MultiModelSpectrumChannel>();
    ch->AddPhasedArraySpectrumPropagationLossModel(seam);
    seam->SetNext(nullptr);
    Ptr<ArrayProbePhy> tx = CreateObject<ArrayProbePhy>();
    Ptr<ArrayProbePhy> rx = CreateObject<ArrayProbePhy>();
    auto mt = CreateObject<ConstantPositionMobilityModel>();
    auto mr = CreateObject<ConstantPositionMobilityModel>();
    mt->SetPosition(Vector(6371e3 + 600e3, 0, 0));
    mr->SetPosition(Vector(6371e3, 0, 0));
    tx->SetMobility(mt);
    rx->SetMobility(mr);
    tx->SetModel(sm);
    rx->SetModel(sm);
    tx->SetChannel(ch);
    rx->SetChannel(ch);
    ch->AddRx(tx);
    ch->AddRx(rx);
    Ptr<SpectrumValue> psd = Create<SpectrumValue>(sm);
    *psd = value;
    if (txPsd)
    {
        *txPsd = psd;
    }
    for (uint32_t k = 0; k < n; ++k)
    {
        Simulator::Schedule(MilliSeconds(1 + k), &ArrayProbePhy::Send, tx, psd);
    }
    Simulator::Stop(MilliSeconds(n + 20));
    Simulator::Run();
    std::vector<Ptr<SpectrumSignalParameters>> out = rx->m_rx;
    Simulator::Destroy();
    return out;
}

} // namespace

/// Plugins run in insertion order on the inner model's output, on a live
/// channel. Inner x2, then affine (x2, +0), then (x1, +10): 1 -> 2 -> 4 -> 14.
/// The reverse insertion gives (2 + 10) x 2 = 24. A seam that applied plugins
/// to the original PSD (the ns-3.43 chain walk) or in another order would give
/// a different number. The transmitted PSD is left untouched.
class NtnSpectrumSeamOrderTest : public TestCase
{
  public:
    NtnSpectrumSeamOrderTest()
        : TestCase("Seam on a live channel: plugins applied in insertion order to the inner output")
    {
    }

  private:
    void DoRun() override
    {
        Ptr<NtnSpectrumSeamModel> a = CreateObject<NtnSpectrumSeamModel>();
        a->SetInnerModel(Create<ScalingPhasedInner>(2.0));
        a->AddPlugin(Create<AffinePlugin>(2.0, 0.0));
        a->AddPlugin(Create<AffinePlugin>(1.0, 10.0));
        Ptr<SpectrumValue> txPsd;
        const auto ra = LiveSeam(a, 6, 1.0, 20, &txPsd);
        NS_TEST_ASSERT_MSG_EQ(ra.size(), 20u, "every transmission delivered through the phased-array slot");
        for (const auto& p : ra)
        {
            for (std::size_t k = 0; k < p->psd->GetValuesN(); ++k)
            {
                NS_TEST_ASSERT_MSG_EQ_TOL((*p->psd)[k], 14.0, 1e-12, "inner, then plugin 1, then plugin 2");
            }
        }
        NS_TEST_ASSERT_MSG_EQ((*txPsd)[0], 1.0, "the transmitted PSD is not modified");

        Ptr<NtnSpectrumSeamModel> b = CreateObject<NtnSpectrumSeamModel>();
        b->SetInnerModel(Create<ScalingPhasedInner>(2.0));
        b->AddPlugin(Create<AffinePlugin>(1.0, 10.0));
        b->AddPlugin(Create<AffinePlugin>(2.0, 0.0));
        const auto rb = LiveSeam(b, 6, 1.0, 5);
        NS_TEST_ASSERT_MSG_EQ(rb.size(), 5u, "delivered");
        if (!rb.empty())
        {
            NS_TEST_ASSERT_MSG_EQ_TOL((*rb[0]->psd)[0], 24.0, 1e-12, "the reverse order composes in reverse");
        }
    }
};

/// The MIMO channel matrix follows the PSD (step 3 of the seam): psd ~ |H|^2,
/// so a per-RB plugin gain g scales every entry of that RB's matrix page by
/// sqrt(g), and NR's rank/PMI search (NrPmSearchFull, TS 38.214 5.2.2.2.1
/// Type-I codebook) sees the channel the PSD describes. Two plugins compose as
/// sqrt(g1 g2). Checked on the matrices the receiver gets on a live channel,
/// with gains 0.25, 4, 1 and 0 (a notched RB).
class NtnSpectrumSeamMatrixTest : public TestCase
{
  public:
    NtnSpectrumSeamMatrixTest()
        : TestCase("Seam rescales the MIMO channel matrix by sqrt(per-RB gain), cumulatively")
    {
    }

  private:
    void DoRun() override
    {
        const std::vector<double> g1 = {0.25, 4.0, 1.0, 0.0};
        const std::vector<double> g2 = {4.0, 0.25, 9.0, 5.0};
        Ptr<NtnSpectrumSeamModel> seam = CreateObject<NtnSpectrumSeamModel>();
        seam->SetInnerModel(Create<MatrixInner>(3.0, 0));
        seam->AddPlugin(Create<PerRbPlugin>(g1));
        const auto r1 = LiveSeam(seam, 4, 1.0, 3);
        NS_TEST_ASSERT_MSG_EQ(r1.size(), 3u, "delivered");
        for (const auto& p : r1)
        {
            NS_TEST_ASSERT_MSG_NE(p->spectrumChannelMatrix, nullptr, "the matrix reaches the receiver");
            for (std::size_t k = 0; k < 4; ++k)
            {
                NS_TEST_ASSERT_MSG_EQ_TOL((*p->psd)[k], 3.0 * g1[k], 1e-12, "PSD = inner x gain");
                for (std::size_t r = 0; r < 2; ++r)
                {
                    for (std::size_t c = 0; c < 2; ++c)
                    {
                        const auto h = p->spectrumChannelMatrix->Elem(r, c, k);
                        const auto ref = MatrixInner::H(r, c, k) * std::sqrt(g1[k]);
                        NS_TEST_ASSERT_MSG_EQ_TOL(std::abs(h - ref), 0.0, 1e-12, "H scaled by sqrt(g)");
                        NS_TEST_ASSERT_MSG_EQ_TOL(std::norm(h),
                                                  std::norm(MatrixInner::H(r, c, k)) * (*p->psd)[k] / 3.0,
                                                  1e-9,
                                                  "|H|^2 tracks the PSD");
                    }
                }
            }
        }

        Ptr<NtnSpectrumSeamModel> two = CreateObject<NtnSpectrumSeamModel>();
        two->SetInnerModel(Create<MatrixInner>(1.0, 0));
        two->AddPlugin(Create<PerRbPlugin>(g1));
        two->AddPlugin(Create<PerRbPlugin>(g2));
        const auto r2 = LiveSeam(two, 4, 1.0, 1);
        NS_TEST_ASSERT_MSG_EQ(r2.size(), 1u, "delivered");
        if (!r2.empty())
        {
            const auto& p = r2[0];
            for (std::size_t k = 0; k < 3; ++k)
            {
                const auto ref = MatrixInner::H(1, 1, k) * std::sqrt(g1[k] * g2[k]);
                NS_TEST_ASSERT_MSG_EQ_TOL(std::abs(p->spectrumChannelMatrix->Elem(1, 1, k) - ref), 0.0, 1e-12,
                                          "two plugins: sqrt(g1 g2)");
            }
            // RB 3 was notched to zero by the first plugin; the second plugin
            // has nothing to scale (pre = 0), so the page stays zero, no NaN.
            const auto h3 = p->spectrumChannelMatrix->Elem(0, 0, 3);
            NS_TEST_ASSERT_MSG_EQ(std::isfinite(h3.real()) && std::isfinite(h3.imag()), true, "no NaN");
            NS_TEST_ASSERT_MSG_EQ_TOL(std::abs(h3), 0.0, 1e-12, "a notched RB stays notched");
        }
    }
};

/// Failure paths on a live channel: AddPlugin(nullptr) is ignored; a plugin
/// that returns no PSD is skipped and the inner output survives; a seam with no
/// inner model and no plugin is a pass-through; matrix pages beyond the PSD
/// bins keep their inner value (gain 1) instead of being read out of range.
class NtnSpectrumSeamFailurePathsTest : public TestCase
{
  public:
    NtnSpectrumSeamFailurePathsTest()
        : TestCase("Seam failure paths: null plugin, plugin without output, empty seam, extra matrix pages")
    {
    }

  private:
    void DoRun() override
    {
        {
            Ptr<NtnSpectrumSeamModel> seam = CreateObject<NtnSpectrumSeamModel>();
            seam->AddPlugin(nullptr);
            NS_TEST_ASSERT_MSG_EQ(seam->GetNumPlugins(), 0u, "a null plugin is not registered");
            const auto r = LiveSeam(seam, 3, 7.0, 3);
            NS_TEST_ASSERT_MSG_EQ(r.size(), 3u, "an empty seam still delivers");
            for (const auto& p : r)
            {
                NS_TEST_ASSERT_MSG_EQ((*p->psd)[1], 7.0, "empty seam: pass-through");
            }
        }
        {
            Ptr<NtnSpectrumSeamModel> seam = CreateObject<NtnSpectrumSeamModel>();
            seam->SetInnerModel(Create<ScalingPhasedInner>(2.0));
            seam->AddPlugin(Create<NullPlugin>());
            seam->AddPlugin(Create<AffinePlugin>(3.0, 0.0));
            const auto r = LiveSeam(seam, 3, 1.0, 2);
            NS_TEST_ASSERT_MSG_EQ(r.size(), 2u, "delivered");
            for (const auto& p : r)
            {
                NS_TEST_ASSERT_MSG_EQ_TOL((*p->psd)[0], 6.0, 1e-12,
                                          "the failing plugin is skipped, the next one still applies");
            }
        }
        {
            Ptr<NtnSpectrumSeamModel> seam = CreateObject<NtnSpectrumSeamModel>();
            seam->SetInnerModel(Create<MatrixInner>(1.0, 2));
            seam->AddPlugin(Create<PerRbPlugin>(std::vector<double>{0.5, 0.5, 0.5}));
            const auto r = LiveSeam(seam, 3, 1.0, 1);
            NS_TEST_ASSERT_MSG_EQ(r.size(), 1u, "delivered");
            if (!r.empty())
            {
                const auto& m = r[0]->spectrumChannelMatrix;
                NS_TEST_ASSERT_MSG_EQ(m->GetNumPages(), 5u, "all inner pages kept");
                for (std::size_t k = 3; k < 5; ++k)
                {
                    NS_TEST_ASSERT_MSG_EQ_TOL(std::abs(m->Elem(0, 1, k) - MatrixInner::H(0, 1, k)), 0.0, 1e-12,
                                              "pages beyond the PSD keep gain 1");
                }
                NS_TEST_ASSERT_MSG_EQ_TOL(std::abs(m->Elem(0, 1, 0) - MatrixInner::H(0, 1, 0) * std::sqrt(0.5)),
                                          0.0, 1e-12, "pages inside the PSD scaled");
            }
        }
    }
};

/// Streams and determinism: AssignStreams reaches the inner model and every
/// plugin (the count is the sum), and a stochastic plugin (the TDL) composed
/// on the seam gives the identical received PSD sequence for the same stream
/// and a different one for another stream, on a live channel.
class NtnSpectrumSeamDeterminismTest : public TestCase
{
  public:
    NtnSpectrumSeamDeterminismTest()
        : TestCase("Seam AssignStreams reaches every plugin; a stochastic plugin is reproducible")
    {
    }

  private:
    std::vector<double> Run(int64_t stream, int64_t* used)
    {
        Ptr<NtnSpectrumSeamModel> seam = CreateObject<NtnSpectrumSeamModel>();
        seam->SetInnerModel(Create<ScalingPhasedInner>(2.0));
        seam->AddPlugin(CreateObject<NtnTdlSpectrumPropagationLossModel>());
        seam->AddPlugin(Create<ScalingPlugin>(0.5));
        *used = seam->AssignStreams(stream);
        const auto r = LiveSeam(seam, 12, 1.0, 20);
        std::vector<double> v;
        for (const auto& p : r)
        {
            for (std::size_t k = 0; k < p->psd->GetValuesN(); ++k)
            {
                v.push_back((*p->psd)[k]);
            }
        }
        return v;
    }

    void DoRun() override
    {
        int64_t used = 0;
        const auto a = Run(40, &used);
        NS_TEST_ASSERT_MSG_EQ(used, 2, "inner 0 + TDL 2 + scaling 0 streams");
        CreateObject<UniformRandomVariable>()->GetValue();
        const auto b = Run(40, &used);
        const auto c = Run(50, &used);
        NS_TEST_ASSERT_MSG_EQ(a.size(), 240u, "20 receptions x 12 bins");
        NS_TEST_ASSERT_MSG_EQ((a == b), true, "same stream: identical received PSDs");
        NS_TEST_ASSERT_MSG_EQ((a == c), false, "another stream: another realization");
        double mean = 0.0;
        for (double x : a)
        {
            mean += x / a.size();
        }
        NS_TEST_ASSERT_MSG_EQ_TOL(mean, 1.0, 0.15, "x2 inner, unit-power TDL, x0.5 plugin: mean near 1");
    }
};

namespace
{

/// Set-1 zenith NR-NTN link with \p plugins installed through
/// AddSpectrumChannelLoss; returns the measured mean DL SINR (dB).
double
NrSinrWithPlugins(const std::vector<Ptr<SpectrumPropagationLossModel>>& plugins, bool fading, const std::string& dir)
{
    RngSeedManager::ResetNextStreamIndex();
    NodeContainer sats;
    NodeContainer ues;
    sats.Create(1);
    ues.Create(1);
    auto sm = CreateObject<ConstantPositionMobilityModel>();
    sm->SetPosition(NtnRealStackHelper::GeodeticToEcef(40.0, -3.0, 600e3));
    sats.Get(0)->AggregateObject(sm);
    auto um = CreateObject<ConstantPositionMobilityModel>();
    um->SetPosition(NtnRealStackHelper::GeodeticToEcef(40.0, -3.0, 0.0));
    ues.Get(0)->AggregateObject(um);
    NtnRealStackHelper rs;
    rs.SetRadioBackend(NtnRealStackHelper::RadioBackend::Nr);
    rs.SetNumerology(1);
    rs.SetSimTime(Seconds(2.0));
    rs.SetOutputDir(dir);
    rs.SetSatEirpDensityDbwMhz(NtnRealStackHelper::kTr38821Set1SBandEirpDensityDbwMhz);
    rs.SetTr38811ExcessLoss(false);
    rs.SetSmallScaleFading(fading);
    rs.Build(sats, ues);
    for (const auto& p : plugins)
    {
        rs.AddSpectrumChannelLoss(p);
    }
    rs.InstallTraffic(NtnRealStackHelper::TrafficProfile::EmbbStreaming, Seconds(0.5), Seconds(1.9));
    Simulator::Stop(Seconds(2.0));
    Simulator::Run();
    rs.Collect();
    const double sinr = rs.GetMeanDlSinrDb();
    Simulator::Destroy();
    return sinr;
}

Ptr<SpectrumPropagationLossModel>
GainDb(double db)
{
    return Create<ScalingPlugin>(std::pow(10.0, db / 10.0));
}

} // namespace

/// Cross-module (nr + NtnRealStackHelper): two plugins added one after the
/// other through AddSpectrumChannelLoss share one seam per BWP channel and
/// compose on top of the 3GPP phased-array channel. -4 dB and -6 dB lower the
/// measured mean DL SINR of the noise-limited Set-1 zenith link by 10 dB
/// (within 1.5 dB); if the second replaced the first it would be 6 dB. A null
/// plugin changes nothing (identical SINR, same streams).
class NtnSpectrumSeamTwoPluginsOverNrTest : public TestCase
{
  public:
    NtnSpectrumSeamTwoPluginsOverNrTest()
        : TestCase("Cross-module: two AddSpectrumChannelLoss plugins compose on the NR channel (-10 dB)")
    {
    }

  private:
    void DoRun() override
    {
        const double base = NrSinrWithPlugins({}, false, CreateTempDirFilename("seam-base"));
        const double none = NrSinrWithPlugins({nullptr}, false, CreateTempDirFilename("seam-null"));
        const double two = NrSinrWithPlugins({GainDb(-4.0), GainDb(-6.0)}, false, CreateTempDirFilename("seam-two"));
        std::cout << "[seam-nr] base " << base << " dB, null plugin " << none << " dB, -4 and -6 dB plugins " << two
                  << " dB" << std::endl;
        NS_TEST_ASSERT_MSG_EQ(std::isfinite(base) && std::isfinite(two), true, "TBs measured");
        NS_TEST_ASSERT_MSG_EQ_TOL(none, base, 1e-9, "a null plugin is ignored");
        NS_TEST_ASSERT_MSG_EQ_TOL(base - two, 10.0, 1.5, "-4 dB and -6 dB compose to -10 dB");
    }
};

/// Cross-module with the 3GPP small-scale fading of the inner model switched
/// on: the seam keeps that fading (it runs the inner model first) and still
/// shifts the measured SINR by the plugin's -10 dB within 2 dB, i.e. the
/// plugin composes with a stochastic 3GPP channel, not only with the LOS
/// plane wave.
class NtnSpectrumSeamWithFadingOverNrTest : public TestCase
{
  public:
    NtnSpectrumSeamWithFadingOverNrTest()
        : TestCase("Cross-module: a -10 dB plugin composes with the 3GPP fading channel on NR")
    {
    }

  private:
    void DoRun() override
    {
        const double base = NrSinrWithPlugins({}, true, CreateTempDirFilename("seam-fad-base"));
        const double ten = NrSinrWithPlugins({GainDb(-10.0)}, true, CreateTempDirFilename("seam-fad-10"));
        std::cout << "[seam-nr-fading] base " << base << " dB, -10 dB plugin " << ten << " dB" << std::endl;
        NS_TEST_ASSERT_MSG_EQ(std::isfinite(base) && std::isfinite(ten), true, "TBs measured");
        NS_TEST_ASSERT_MSG_EQ_TOL(base - ten, 10.0, 2.0, "the plugin shifts the faded SINR by 10 dB");
    }
};

class NtnSpectrumSeamTestSuite : public TestSuite
{
  public:
    NtnSpectrumSeamTestSuite()
        : TestSuite("ntn-spectrum-seam", Type::UNIT)
    {
        AddTestCase(new NtnSpectrumSeamComposeTest, Duration::QUICK);
        AddTestCase(new NtnSpectrumSeamOrderTest, Duration::QUICK);
        AddTestCase(new NtnSpectrumSeamMatrixTest, Duration::QUICK);
        AddTestCase(new NtnSpectrumSeamFailurePathsTest, Duration::QUICK);
        AddTestCase(new NtnSpectrumSeamDeterminismTest, Duration::QUICK);
        AddTestCase(new NtnSpectrumSeamTwoPluginsOverNrTest, Duration::QUICK);
        AddTestCase(new NtnSpectrumSeamWithFadingOverNrTest, Duration::QUICK);
    }
};

static NtnSpectrumSeamTestSuite g_ntnSpectrumSeamTestSuite;
