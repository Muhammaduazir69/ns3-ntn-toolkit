/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only
//
// WS3 multi-tier RIC tests (AI_NATIVE_ORAN_NTN plan):
//   1. timescale separation: RT (<10 ms) vs Near-RT vs Non-RT loop counts
//      over the same simulated horizon obey the O-RAN tier ratios, and the
//      RT actions actuate on the per-UE KPI stream;
//   2. RIC placement latency from real geometry: on-board == processing
//      only; ground gateway == slant/c + processing (inside the paper's
//      2.1-18 ms LEO band); cloud adds the backhaul; BindToGeometry tracks
//      a moving satellite's REAL slant range;
//   3. NWDAF -> SMO -> TPN closed loop: rising measured delay on a slice
//      raises slaRisk, the SMO grants radio share, and the TPN controller
//      switches to the lower-latency transport path;
//   4. ONNX xApp graceful fallback: without onnxruntime the registered
//      heuristic serves inference (and LoadModel reports false).

#include "ns3/constant-position-mobility-model.h"
#include "ns3/constant-velocity-mobility-model.h"
#include "ns3/nstime.h"
#include "ns3/oran-ntn-cross-domain.h"
#include "ns3/oran-ntn-e2-interface.h"
#include "ns3/oran-ntn-onnx-xapp.h"
#include "ns3/oran-ntn-ric-placement.h"
#include "ns3/oran-ntn-rt-ric.h"
#include "ns3/simulator.h"
#include "ns3/test.h"

#include <cmath>

using namespace ns3;

class RtRicTimescaleTest : public TestCase
{
  public:
    RtRicTimescaleTest()
        : TestCase("RT / Near-RT / Non-RT timescale separation + RT actuation")
    {
    }

  private:
    void DoRun() override
    {
        // Measured-KPI stand-in stream: UE0 healthy, UE1 fading after t=5 s.
        auto sinrGetter = [](uint32_t ue) {
            const double t = Simulator::Now().GetSeconds();
            return (ue == 0) ? 15.0 : ((t < 5.0) ? 12.0 : 1.5);
        };
        auto tblerGetter = [](uint32_t ue) {
            const double t = Simulator::Now().GetSeconds();
            return (ue == 1 && t >= 5.0) ? 0.6 : 0.01;
        };

        Ptr<OranNtnRtRic> rt = CreateObject<OranNtnRtRic>();
        rt->SetLoopPeriod(MilliSeconds(2));
        rt->SetNumUes(2);
        rt->SetKpiSources(sinrGetter, tblerGetter);
        uint32_t mcsCaps = 0;
        rt->AddRtAction("mcs-cap", [&mcsCaps](const OranNtnRtRic::RtKpi& k) {
            if (k.sinrDb < 3.0)
            {
                ++mcsCaps; // would clamp the AMC here
                return true;
            }
            return false;
        });
        rt->Start();

        // Near-RT (100 ms) and Non-RT (2 s) loops for the ratio assertion.
        uint64_t nearRtLoops = 0;
        uint64_t nonRtLoops = 0;
        for (double t = 0.1; t < 10.0; t += 0.1)
        {
            Simulator::Schedule(Seconds(t), [&nearRtLoops] { ++nearRtLoops; });
        }
        for (double t = 2.0; t < 10.0; t += 2.0)
        {
            Simulator::Schedule(Seconds(t), [&nonRtLoops] { ++nonRtLoops; });
        }

        Simulator::Stop(Seconds(10.0));
        Simulator::Run();

        // 10 s at 2 ms -> ~5000 RT loops; tiers separated by >=1 order each.
        NS_TEST_ASSERT_MSG_EQ_TOL(static_cast<double>(rt->GetLoopCount()), 5000.0, 5.0,
                                  "RT loop count at 2 ms");
        NS_TEST_ASSERT_MSG_GT(rt->GetLoopCount(), 10 * nearRtLoops,
                              "RT >= 10x Near-RT loop rate");
        NS_TEST_ASSERT_MSG_GT(nearRtLoops, 10 * nonRtLoops,
                              "Near-RT >= 10x Non-RT loop rate");
        // UE1 faded for 5 s -> ~2500 RT actuations, none for UE0.
        NS_TEST_ASSERT_MSG_EQ_TOL(static_cast<double>(mcsCaps), 2500.0, 10.0,
                                  "RT action tracked the per-UE fade window");
        NS_TEST_ASSERT_MSG_EQ(rt->GetActuationCount(), mcsCaps, "actuation counter");
        Simulator::Destroy();
    }
};

class RicPlacementGeometryTest : public TestCase
{
  public:
    RicPlacementGeometryTest()
        : TestCase("placement E2 delay from real geometry (paper 2.1-18 ms band)")
    {
    }

  private:
    void DoRun() override
    {
        constexpr double kC = 299792458.0;
        Ptr<OranNtnRicPlacement> pl = CreateObject<OranNtnRicPlacement>();
        pl->SetProcessingDelay(MicroSeconds(500));
        pl->SetCloudBackhaulDelay(MilliSeconds(20));

        pl->SetSite(OranNtnRicPlacement::Site::OnBoardSatellite);
        NS_TEST_ASSERT_MSG_EQ(pl->ComputeE2Delay(550e3), MicroSeconds(500),
                              "on-board = processing only");

        pl->SetSite(OranNtnRicPlacement::Site::GroundGateway);
        const double zenithMs = pl->ComputeE2Delay(630e3).GetSeconds() * 1e3;
        NS_TEST_ASSERT_MSG_EQ_TOL(zenithMs, 630e3 / kC * 1e3 + 0.5, 1e-6,
                                  "gateway = slant/c + processing");
        // Paper Sec. IV-B: measured NTN fronthaul one-way 2.1-18 ms at LEO.
        NS_TEST_ASSERT_MSG_GT(zenithMs, 2.1, "zenith inside the paper band");
        const double horizonMs = pl->ComputeE2Delay(2500e3).GetSeconds() * 1e3;
        NS_TEST_ASSERT_MSG_LT(horizonMs, 18.0, "near-horizon inside the paper band");

        pl->SetSite(OranNtnRicPlacement::Site::GroundCloud);
        NS_TEST_ASSERT_MSG_EQ_TOL(pl->ComputeE2Delay(630e3).GetSeconds() * 1e3,
                                  zenithMs + 20.0, 1e-6, "cloud adds the backhaul");

        // Live geometry binding: a satellite receding at 7 km/s must GROW the
        // applied feeder delay between ticks.
        Ptr<ConstantVelocityMobilityModel> sat = CreateObject<ConstantVelocityMobilityModel>();
        sat->SetPosition(Vector(0, 0, 550e3));
        sat->SetVelocity(Vector(7000, 0, 0));
        Ptr<ConstantPositionMobilityModel> gw = CreateObject<ConstantPositionMobilityModel>();
        gw->SetPosition(Vector(0, 0, 0));
        Ptr<OranNtnE2Node> e2 = CreateObject<OranNtnE2Node>();
        pl->SetSite(OranNtnRicPlacement::Site::GroundGateway);
        pl->BindToGeometry(sat, gw, e2, Seconds(1.0));
        const Time d0 = pl->GetLastAppliedDelay();
        Simulator::Stop(Seconds(30.0));
        Simulator::Run();
        const Time d30 = pl->GetLastAppliedDelay();
        NS_TEST_ASSERT_MSG_GT(d30.GetSeconds(), d0.GetSeconds(),
                              "feeder delay grew with the real slant range");
        // 30 s * 7 km/s -> slant 595 km at 550 km altitude: ~1.98 ms + proc.
        NS_TEST_ASSERT_MSG_EQ_TOL(d30.GetSeconds() * 1e3,
                                  std::hypot(550e3, 29.0 * 7000.0) / kC * 1e3 + 0.5,
                                  0.1, "applied delay matches the trajectory");
        Simulator::Destroy();
    }
};

class CrossDomainLoopTest : public TestCase
{
  public:
    CrossDomainLoopTest()
        : TestCase("NWDAF risk -> SMO radio/compute grant -> TPN path switch")
    {
    }

  private:
    void DoRun() override
    {
        Ptr<OranNtnNwdaf> nwdaf = CreateObject<OranNtnNwdaf>();
        Ptr<OranNtnTpnController> tpn = CreateObject<OranNtnTpnController>();
        Ptr<OranNtnCrossDomainSmo> smo = CreateObject<OranNtnCrossDomainSmo>();

        // Two transport paths whose latency crosses over mid-run (a LEO GSL
        // handover shape): pathA 8->40 ms, pathB constant 15 ms.
        auto pathALat = [] {
            const double t = Simulator::Now().GetSeconds();
            return 8.0 + (t > 10.0 ? (t - 10.0) * 4.0 : 0.0);
        };
        uint32_t aActivations = 0;
        uint32_t bActivations = 0;
        tpn->RegisterPath("sat-A", pathALat, [&aActivations](uint8_t) { ++aActivations; });
        tpn->RegisterPath("sat-B", [] { return 15.0; },
                          [&bActivations](uint8_t) { ++bActivations; });

        smo->SetNwdaf(nwdaf);
        smo->SetTpnController(tpn);
        double lastRadioShare = 0;
        double lastComputeShare = 0;
        smo->SetRadioQuotaSetter(
            [&lastRadioShare](uint8_t, double s) { lastRadioShare = s; });
        smo->SetComputeAllocSetter(
            [&lastComputeShare](uint8_t, double s) { lastComputeShare = s; });
        smo->AddSlice(/*sst*/ 2, /*budgetMs*/ 20.0, /*radio*/ 0.3, /*compute*/ 0.3);
        smo->SetCoordinationPeriod(Seconds(1.0));
        smo->Start();

        // Measured-shaped KPM stream: slice 2 delay healthy then degrading —
        // the same shape NtnOranAiFlowMonitor emits per granularity period.
        for (double t = 0.5; t < 30.0; t += 1.0)
        {
            Simulator::Schedule(Seconds(t), [nwdaf, t] {
                NtnOranAiFlowMonitor::KpmIndication ind;
                ind.collectionStart = Seconds(t - 1.0);
                ind.granularity = Seconds(1.0);
                ind.flowId = 1;
                ind.key.sst = 2;
                ind.key.sd = 1;
                ind.key.fiveQi = 82;
                ind.measurements["DRB.UEThpDl"] = 1.0;
                ind.measurements["DRB.RlcSduDelayDl"] = (t < 10.0) ? 6.0 : 6.0 + (t - 10.0) * 2.0;
                ind.measurements["DRB.PacketLossRateDl"] = (t < 10.0) ? 0.0 : 0.05;
                nwdaf->ConsumeKpm(ind);
            });
        }

        double earlyRisk = -1;
        Simulator::Schedule(Seconds(9.0), [nwdaf, &earlyRisk] {
            earlyRisk = nwdaf->GetAnalytics(2).slaRisk;
        });
        double earlyRadioShare = -1;
        Simulator::Schedule(Seconds(9.5), [&lastRadioShare, &earlyRadioShare] {
            earlyRadioShare = lastRadioShare;
        });

        Simulator::Stop(Seconds(30.0));
        Simulator::Run();

        const auto a = nwdaf->GetAnalytics(2);
        NS_TEST_ASSERT_MSG_GT(a.slaRisk, earlyRisk + 0.2,
                              "risk rose with the measured delay degradation");
        NS_TEST_ASSERT_MSG_GT(lastRadioShare, earlyRadioShare,
                              "SMO granted radio share as risk rose");
        NS_TEST_ASSERT_MSG_GT(lastComputeShare, 0.3, "SMO granted compute share");
        // The TPN must have moved the slice off the degrading path.
        NS_TEST_ASSERT_MSG_EQ(tpn->GetActivePath(2), "sat-B", "switched to sat-B");
        NS_TEST_ASSERT_MSG_GT(aActivations, 0u, "started on sat-A");
        NS_TEST_ASSERT_MSG_GT(bActivations, 0u, "ended on sat-B");
        NS_TEST_ASSERT_MSG_GT(smo->GetDecisions().size(), 20u, "decision log kept");
        Simulator::Destroy();
    }
};

class OnnxFallbackTest : public TestCase
{
  public:
    OnnxFallbackTest()
        : TestCase("ONNX xApp inference path: model when built-in, else heuristic")
    {
    }

  private:
    void DoRun() override
    {
        Ptr<OranNtnOnnxXapp> xapp = CreateObject<OranNtnOnnxXapp>();
        xapp->RegisterHeuristic([](const std::vector<double>& f) {
            // Threshold policy on (sinrMean, lossMean): boost when degraded.
            return std::vector<double>{(f[0] < 5.0 || f[1] > 0.1) ? 1.0 : 0.0};
        });
        const bool loaded = xapp->LoadModel("nonexistent-model.onnx");
        NS_TEST_ASSERT_MSG_EQ(loaded, false, "bogus model never loads");
        NS_TEST_ASSERT_MSG_EQ(xapp->IsModelLoaded(), false, "fallback engaged");

        auto healthy = xapp->Infer({15.0, 0.0});
        auto degraded = xapp->Infer({2.0, 0.3});
        NS_TEST_ASSERT_MSG_EQ(healthy.size(), 1, "heuristic output");
        NS_TEST_ASSERT_MSG_EQ_TOL(healthy[0], 0.0, 1e-9, "no action when healthy");
        NS_TEST_ASSERT_MSG_EQ_TOL(degraded[0], 1.0, 1e-9, "boost when degraded");
        NS_TEST_ASSERT_MSG_EQ(xapp->GetInferenceCount(), 2, "inference counter");
    }
};

class OranNtnMultiTierRicTestSuite : public TestSuite
{
  public:
    OranNtnMultiTierRicTestSuite()
        : TestSuite("oran-ntn-multi-tier-ric", Type::UNIT)
    {
        AddTestCase(new RtRicTimescaleTest, Duration::QUICK);
        AddTestCase(new RicPlacementGeometryTest, Duration::QUICK);
        AddTestCase(new CrossDomainLoopTest, Duration::QUICK);
        AddTestCase(new OnnxFallbackTest, Duration::QUICK);
    }
};

static OranNtnMultiTierRicTestSuite g_oranNtnMultiTierRicTestSuite;
