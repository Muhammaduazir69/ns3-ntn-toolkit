/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only
//
// Wave 2 M3 unit tests: per-RB atmospheric attenuation (P.676 / P.838 /
// P.618), the rain-cell field, TR 38.901 blockage model B screens and the
// geometry-driven RIS spectrum plugin.

#include "ns3/constant-position-mobility-model.h"
#include "ns3/constant-velocity-mobility-model.h"
#include "ns3/ntn-screen-blockage-model.h"
#include "ns3/simulator.h"
#include "ns3/spectrum-model.h"
#include "ns3/spectrum-signal-parameters.h"
#include "ns3/spectrum-value.h"
#include "ns3/test.h"
#include "ns3/thz-ntn-itu-recommendations.h"
#include "ns3/thz-ntn-link-geometry.h"
#include "ns3/thz-ntn-molecular-absorption.h"
#include "ns3/thz-ntn-rain-cell-field.h"
#include "ns3/thz-ntn-ris-spectrum-model.h"
#include "ns3/thz-ntn-ris.h"
#include "ns3/thz-ntn-spectrum-atmosphere-model.h"
#include "ns3/uinteger.h"
#include "ns3/double.h"

#include <cmath>
#include <vector>

using namespace ns3;

namespace
{

Ptr<SpectrumSignalParameters>
FlatParams(double f0, double rbHz, uint32_t nRb)
{
    std::vector<double> fcs;
    for (uint32_t i = 0; i < nRb; ++i)
    {
        fcs.push_back(f0 + (i + 0.5) * rbHz);
    }
    Ptr<SpectrumModel> sm = Create<SpectrumModel>(fcs);
    Ptr<SpectrumValue> psd = Create<SpectrumValue>(sm);
    *psd = 1.0;
    Ptr<SpectrumSignalParameters> p = Create<SpectrumSignalParameters>();
    p->psd = psd;
    return p;
}

Ptr<ConstantPositionMobilityModel>
At(double x, double y, double z)
{
    Ptr<ConstantPositionMobilityModel> m = CreateObject<ConstantPositionMobilityModel>();
    m->SetPosition(Vector(x, y, z));
    return m;
}

std::vector<double>
LossDb(Ptr<const SpectrumValue> before, Ptr<const SpectrumValue> after)
{
    std::vector<double> out;
    for (size_t i = 0; i < after->GetValuesN(); ++i)
    {
        out.push_back(-10.0 * std::log10((*after)[i] / (*before)[i]));
    }
    return out;
}

} // namespace

/// Per-RB gaseous loss equals P.676 line-by-line at each RB frequency, varies
/// across a band near the 118.75 GHz O2 line, and vanishes on an ISL.
class ThzSpectrumAtmosphereGasTest : public TestCase
{
  public:
    ThzSpectrumAtmosphereGasTest()
        : TestCase("M3: per-RB P.676 absorption at each RB frequency, flat ISL")
    {
    }

  private:
    void DoRun() override
    {
        Ptr<ThzNtnSpectrumAtmosphereModel> m = CreateObject<ThzNtnSpectrumAtmosphereModel>();
        m->SetAttribute("ElevationQuantum", DoubleValue(1e-4));
        Ptr<ThzNtnMolecularAbsorption> ref = CreateObject<ThzNtnMolecularAbsorption>();
        // 30 deg elevation, ground at 50 m, satellite 550 km, flat ENU frame.
        const double el = 30.0;
        const double h = 550e3;
        auto gnd = At(0, 0, 50);
        auto sat = At((h - 50) / std::tan(el * M_PI / 180.0), 0, h);
        const ThzNtnLinkGeometry g = ThzNtnLinkGeometry::Compute(PeekPointer(sat), PeekPointer(gnd));
        NS_TEST_ASSERT_MSG_EQ_TOL(g.elevationDeg, el, 1e-6, "geometry elevation");

        // 100 GHz carrier, 66 RBs of 1.44 MHz (120 kHz SCS).
        auto p = FlatParams(100e9 - 33 * 1.44e6, 1.44e6, 66);
        Ptr<SpectrumValue> out = m->CalcRxPowerSpectralDensity(p, sat, gnd);
        auto loss = LossDb(p->psd, out);
        auto band = p->psd->ConstBandsBegin();
        for (size_t i = 0; i < loss.size(); i += 13, band += 13)
        {
            const double r =
                ref->ComputeSlantPathAbsorption(band->fc, g.elevationDeg, 0.05, 100.0);
            NS_TEST_ASSERT_MSG_EQ_TOL(loss[i], r, 1e-6, "RB loss = P.676 at RB fc");
        }
        // Near the 118.75 GHz O2 line the per-RB loss is not flat over 400 MHz.
        auto p2 = FlatParams(118.2e9, 1.44e6 * 4, 70);
        auto out2 = m->CalcRxPowerSpectralDensity(p2, sat, gnd);
        auto l2 = LossDb(p2->psd, out2);
        const double spread = *std::max_element(l2.begin(), l2.end()) -
                              *std::min_element(l2.begin(), l2.end());
        NS_TEST_ASSERT_MSG_GT(spread, 1.0, "frequency-selective absorption near 118.75 GHz");

        // ISL: both ends above 100 km -> pass through.
        auto s1 = At(0, 0, 550e3);
        auto s2 = At(600e3, 0, 550e3);
        auto out3 = m->CalcRxPowerSpectralDensity(p, s1, s2);
        auto l3 = LossDb(p->psd, out3);
        NS_TEST_ASSERT_MSG_EQ_TOL(l3[10], 0.0, 1e-12, "no atmosphere on an ISL");
    }
};

/// Rain: uniform P.618 mode per RB, and a rain cell field whose path integral
/// equals k R^alpha L_s for a uniform cell and vanishes when it moves away.
class ThzRainFieldTest : public TestCase
{
  public:
    ThzRainFieldTest()
        : TestCase("M3: rain-cell field path integral (P.838 x P.618 L_s) and motion")
    {
    }

  private:
    void DoRun() override
    {
        const double f = 100e9;
        const double el = 40.0;
        auto gnd = At(0, 0, 0);
        auto sat = At(550e3 / std::tan(el * M_PI / 180.0), 0, 550e3);
        const ThzNtnLinkGeometry g = ThzNtnLinkGeometry::Compute(PeekPointer(gnd), PeekPointer(sat));

        Ptr<ThzNtnRainCellField> field = CreateObject<ThzNtnRainCellField>();
        field->SetRainHeightKm(3.5);
        ThzNtnRainCellField::Cell c;
        c.peakRate = 25.0;
        c.r0 = 1e9; // effectively uniform
        field->AddCell(c);
        const auto [k, alpha] = itu::Itu838RainModel::GetKAlpha(f, itu::Polarization::circular);
        const double ls = ThzNtnRainCellField::SlantLengthKm(el, 0.0, 3.5);
        const double expect = k * std::pow(25.0, alpha) * ls;
        NS_TEST_ASSERT_MSG_EQ_TOL(field->PathAttenuationDb(g, f), expect, 1e-3 * expect,
                                  "uniform cell = k R^a L_s");

        // A 2 km cell moving east at 20 m/s from 30 km west: dry at t=0,
        // overhead at 1500 s.
        Ptr<ThzNtnRainCellField> f2 = CreateObject<ThzNtnRainCellField>();
        ThzNtnRainCellField::Cell m;
        m.x0 = -30e3;
        m.vx = 20.0;
        m.peakRate = 50.0;
        m.r0 = 2000.0;
        f2->AddCell(m);
        double ds = 0.0;
        const double a0 = f2->IntegrateDb(f2->SamplePath(g, Seconds(0), ds), ds, f);
        const double a1 = f2->IntegrateDb(f2->SamplePath(g, Seconds(1500), ds), ds, f);
        NS_TEST_ASSERT_MSG_EQ_TOL(a0, 0.0, 1e-9, "cell far away: dry path");
        NS_TEST_ASSERT_MSG_GT(a1, 5.0, "cell overhead: rain fade");

        // Spectrum model in field mode: centre-RB loss = field integral + gas.
        Ptr<ThzNtnSpectrumAtmosphereModel> sm = CreateObject<ThzNtnSpectrumAtmosphereModel>();
        sm->SetGasEnabled(false);
        sm->SetRainField(field);
        auto p = FlatParams(f - 10 * 1.44e6, 1.44e6, 20);
        auto out = sm->CalcRxPowerSpectralDensity(p, sat, gnd);
        auto l = LossDb(p->psd, out);
        NS_TEST_ASSERT_MSG_EQ_TOL(l[10], field->PathAttenuationDb(g, (p->psd->ConstBandsBegin() + 10)->fc),
                                  1e-6, "spectrum field mode");

        // Uniform P.618 mode: equals Itu618LossModel at the RB frequency.
        Ptr<ThzNtnSpectrumAtmosphereModel> um = CreateObject<ThzNtnSpectrumAtmosphereModel>();
        um->SetGasEnabled(false);
        um->SetUniformRainRate(25.0);
        Ptr<itu::Itu618LossModel> p618 = CreateObject<itu::Itu618LossModel>();
        auto out2 = um->CalcRxPowerSpectralDensity(p, sat, gnd);
        auto l2 = LossDb(p->psd, out2);
        const double ref = p618->SlantPathRainAttenuationDb((p->psd->ConstBandsBegin() + 3)->fc,
                                                            g.elevationDeg, 25.0, 0.0,
                                                            itu::Polarization::circular);
        NS_TEST_ASSERT_MSG_EQ_TOL(l2[3], ref, 1e-6, "uniform P.618 per RB");
    }
};

/// TR 38.901 model B: deep shadow loss grows with frequency, zero when the
/// screen is not between the ends or the ray passes above it.
class ThzScreenBlockageTest : public TestCase
{
  public:
    ThzScreenBlockageTest()
        : TestCase("M3: TR 38.901 7.6.4.2 blockage model B screens")
    {
    }

  private:
    void DoRun() override
    {
        Ptr<NtnScreenBlockageModel> b = CreateObject<NtnScreenBlockageModel>();
        auto truck = At(6, 0, 0);
        b->AddScreen(truck, 2.5, 4.0, "truck");
        // Satellite at 15 deg elevation due east; terminal at 1.5 m: the ray
        // is 3.1 m up at the truck, 0.9 m below its top.
        const double el = 15.0;
        const Vector ue(0, 0, 1.5);
        const Vector sat(550e3 / std::tan(el * M_PI / 180.0), 0, 550e3);
        const double l2 = b->LossDb(sat, ue, 2e9);
        const double l100 = b->LossDb(sat, ue, 100e9);
        NS_TEST_ASSERT_MSG_EQ(b->IsBlocked(sat, ue), true, "ray crosses the truck");
        NS_TEST_ASSERT_MSG_GT(l100, l2, "higher frequency, deeper shadow");
        NS_TEST_ASSERT_MSG_GT(l100, 15.0, "deep shadow at 100 GHz");
        // Satellite at 60 deg: the ray is 11.9 m up at the truck: clear.
        const Vector satHigh(550e3 / std::tan(60.0 * M_PI / 180.0), 0, 550e3);
        NS_TEST_ASSERT_MSG_EQ(b->IsBlocked(satHigh, ue), false, "ray above the truck");
        NS_TEST_ASSERT_MSG_LT(b->LossDb(satHigh, ue, 100e9), 0.5, "clear path ~0 dB");
        // Satellite on the other side (west): screen not between.
        const Vector satWest(-550e3 / std::tan(el * M_PI / 180.0), 0, 550e3);
        NS_TEST_ASSERT_MSG_EQ_TOL(b->LossDb(satWest, ue, 100e9), 0.0, 1e-12, "screen behind");
    }
};

/// RIS plugin: aperture law (no 20 log10 N over the direct link), coherent
/// per-RB sum with the excess delay, and recovery of a blocked direct ray.
class ThzRisSpectrumTest : public TestCase
{
  public:
    ThzRisSpectrumTest()
        : TestCase("M3: RIS spectrum plugin follows the cascaded aperture law")
    {
    }

  private:
    void DoRun() override
    {
        const double f = 100e9;
        Ptr<ThzNtnRis> ris = CreateObject<ThzNtnRis>();
        ris->SetAttribute("NumElementsX", UintegerValue(40));
        ris->SetAttribute("NumElementsY", UintegerValue(40));
        ris->SetAttribute("Frequency", DoubleValue(f));
        ris->SetAttribute("ReflectionEfficiency", DoubleValue(0.9));

        const Vector sat(0, -1500e3, 550e3); // 20 deg elevation, due south
        const Vector risPos(0, 10, 5);
        const Vector ue(0, 0, 1.5);
        // Unblocked: the RIS adds well under 1 dB (cascaded ray ~20+ dB down).
        const double gOpen = ris->ComputeLinkSnrGain_dB(sat, risPos, ue);
        NS_TEST_ASSERT_MSG_LT(gOpen, 1.0, "no 20 log10 N over an open direct link");
        NS_TEST_ASSERT_MSG_GT(gOpen, 0.0, "but a positive coherent contribution");
        // Old law would have claimed >= 60 dB.
        NS_TEST_ASSERT_MSG_GT(ris->ComputeArrayFactorGain_dB(true), 60.0, "array factor vs one element");

        // Spectrum plugin, direct ray blocked by a screen between UE and sat.
        Ptr<NtnScreenBlockageModel> blk = CreateObject<NtnScreenBlockageModel>();
        blk->AddScreen(At(0, -6, 0), 6.0, 10.0, "wall");
        Ptr<ThzNtnRisSpectrumModel> m = CreateObject<ThzNtnRisSpectrumModel>();
        m->SetRis(ris);
        auto risMob = At(risPos.x, risPos.y, risPos.z);
        m->SetRisMobility(risMob);
        m->SetPanelNormal(Vector(0, -1, 0.3)); // facing the street and the sky
        m->SetBlockage(blk);
        auto ueMob = At(ue.x, ue.y, ue.z);
        auto satMob = At(sat.x, sat.y, sat.z);
        m->AddTerminal(ueMob);
        auto p = FlatParams(f - 33 * 1.44e6, 1.44e6, 66);

        auto off = LossDb(p->psd, m->CalcRxPowerSpectralDensity(p, satMob, ueMob));
        const double blocked = off[33];
        NS_TEST_ASSERT_MSG_GT(blocked, 15.0, "direct ray blocked by the wall");

        m->SetEngaged(true);
        auto on = LossDb(p->psd, m->CalcRxPowerSpectralDensity(p, satMob, ueMob));
        const auto st = m->GetLastState();
        NS_TEST_ASSERT_MSG_EQ(st.inFront, true, "both ends in front of the panel");
        // Expected centre gain: |b_d + a_r|^2 (co-phased at the carrier).
        const double bD = std::pow(10.0, -st.directBlockDb / 20.0);
        const double aR = std::pow(10.0, st.risRelDb / 20.0);
        const double expectLoss = -20.0 * std::log10(bD + aR);
        NS_TEST_ASSERT_MSG_EQ_TOL(on[33], expectLoss, 0.5, "coherent sum at f_c");
        NS_TEST_ASSERT_MSG_LT(on[33], blocked, "RIS recovers part of the blockage");
        // RIS ray vs direct follows the aperture law computed independently.
        const double fsplD = 20.0 * std::log10(4.0 * M_PI * CalculateDistance(sat, ue) * f / 299792458.0);
        const double rel = fsplD - ris->ComputeCascadedPathLoss_dB(sat, risPos, ue);
        NS_TEST_ASSERT_MSG_EQ_TOL(st.risRelDb, rel, 0.5, "cascaded law, no misalignment at update");
        Simulator::Destroy();
    }
};

class ThzNtnSpectrumPluginsTestSuite : public TestSuite
{
  public:
    ThzNtnSpectrumPluginsTestSuite()
        : TestSuite("thz-ntn-spectrum-plugins", Type::UNIT)
    {
        AddTestCase(new ThzSpectrumAtmosphereGasTest, TestCase::Duration::QUICK);
        AddTestCase(new ThzRainFieldTest, TestCase::Duration::QUICK);
        AddTestCase(new ThzScreenBlockageTest, TestCase::Duration::QUICK);
        AddTestCase(new ThzRisSpectrumTest, TestCase::Duration::QUICK);
    }
};

static ThzNtnSpectrumPluginsTestSuite g_thzNtnSpectrumPluginsTestSuite;
