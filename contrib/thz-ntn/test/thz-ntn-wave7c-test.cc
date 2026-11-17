/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only
//
// Wave 7C tests: ITU-R P.840-9 cloud attenuation and P.834-9 apparent
// elevation on the thz-ntn packet path, and the ThzNtnPacketLink data plane
// (live delay, DVB-S2 ACM, BLER decode, sky noise). Reference values for
// P.840-9 come from ITU-Rpy (master, itu840 version 9) run on the full
// 0.25 degree maps, at sites that are 1 degree nodes.

#include "ns3/applications-module.h"
#include "ns3/constant-position-mobility-model.h"
#include "ns3/constant-velocity-mobility-model.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/test.h"
#include "ns3/thz-ntn-cloud-attenuation.h"
#include "ns3/thz-ntn-link-geometry.h"
#include "ns3/thz-ntn-molecular-absorption.h"
#include "ns3/thz-ntn-packet-link.h"
#include "ns3/thz-ntn-propagation-loss-model.h"

#include <cmath>
#include <vector>

using namespace ns3;
using namespace ns3::thzntn;

namespace
{
constexpr double kDegRad = M_PI / 180.0;

/// WGS-84 geodetic to ECEF (m).
Vector
GeodeticToEcef(double latDeg, double lonDeg, double altM)
{
    const double a = 6378137.0;
    const double e2 = 6.69437999014e-3;
    const double la = latDeg * kDegRad;
    const double lo = lonDeg * kDegRad;
    const double n = a / std::sqrt(1.0 - e2 * std::sin(la) * std::sin(la));
    return Vector((n + altM) * std::cos(la) * std::cos(lo),
                  (n + altM) * std::cos(la) * std::sin(lo),
                  (n * (1.0 - e2) + altM) * std::sin(la));
}

/// A ground node at a site and a satellite moving east over it at 550 km,
/// both in ECEF. The satellite starts \p startKm west of the zenith point.
void
MakePass(double latDeg,
         double lonDeg,
         double startKm,
         Ptr<ConstantPositionMobilityModel>& gnd,
         Ptr<ConstantVelocityMobilityModel>& sat)
{
    gnd = CreateObject<ConstantPositionMobilityModel>();
    gnd->SetPosition(GeodeticToEcef(latDeg, lonDeg, 0.0));
    const Vector top = GeodeticToEcef(latDeg, lonDeg, 550e3);
    const double lo = lonDeg * kDegRad;
    const Vector east(-std::sin(lo), std::cos(lo), 0.0);
    sat = CreateObject<ConstantVelocityMobilityModel>();
    sat->SetPosition(Vector(top.x - startKm * 1e3 * east.x,
                            top.y - startKm * 1e3 * east.y,
                            top.z - startKm * 1e3 * east.z));
    sat->SetVelocity(Vector(7.5e3 * east.x, 7.5e3 * east.y, 7.5e3 * east.z));
}
} // namespace

/// P.840-9 coefficient, site statistics and L(p) against ITU-Rpy.
class ThzNtnP840CloudModelTest : public TestCase
{
  public:
    ThzNtnP840CloudModelTest()
        : TestCase("Wave 7C: P.840-9 K_L(f), site log-normal statistics and L(p)")
    {
    }

  private:
    void DoRun() override
    {
        const double f[] = {10, 30, 50, 100, 140, 200, 300};
        const double kl[] = {0.08577094, 0.70785396, 1.71800037, 4.69065780,
                             6.94932426, 10.01439283, 14.25617960};
        for (size_t i = 0; i < 7; ++i)
        {
            NS_TEST_ASSERT_MSG_EQ_TOL(itu::P840CloudMassAbsorptionCoefficient(f[i]),
                                      kl[i],
                                      1e-4 * kl[i],
                                      "K_L(f) of P.840-9 eq. (13)");
        }
        // London node (51 N, 0 E): exact map node.
        const auto s = itu::P840SiteStatistics(51.0, 0.0);
        NS_TEST_ASSERT_MSG_EQ(s.cloudy, true, "cloudy site");
        NS_TEST_ASSERT_MSG_EQ_TOL(s.mL, -2.534, 1e-3, "m_L");
        NS_TEST_ASSERT_MSG_EQ_TOL(s.sigmaL, 0.910, 1e-3, "sigma_L");
        NS_TEST_ASSERT_MSG_EQ_TOL(s.pLPercent, 63.71, 0.01, "P_L");
        NS_TEST_ASSERT_MSG_EQ_TOL(s.LwcExceededKgM2(1.0), 0.56237, 0.002, "L(1 %)");
        NS_TEST_ASSERT_MSG_EQ_TOL(s.LwcExceededKgM2(10.0), 0.19838, 0.001, "L(10 %)");
        // Eastern US node (40 N, 280 E) and its negative-longitude alias.
        const auto us = itu::P840SiteStatistics(40.0, -80.0);
        NS_TEST_ASSERT_MSG_EQ_TOL(us.LwcExceededKgM2(5.0), 0.31625, 0.002, "L(5 %) at 40N 80W");
        // Gulf node (25 N, 55 E): P_L 12.9 %, so L(20 %) = 0.
        const auto gulf = itu::P840SiteStatistics(25.0, 55.0);
        NS_TEST_ASSERT_MSG_EQ_TOL(gulf.LwcExceededKgM2(1.0), 0.11651, 0.001, "L(1 %) Gulf");
        NS_TEST_ASSERT_MSG_EQ(gulf.LwcExceededKgM2(20.0), 0.0, "p above P_L: no cloud");
        // Failure path: a map node with no log-normal fit (Antarctic plateau).
        const auto ant = itu::P840SiteStatistics(-81.0, 77.0);
        NS_TEST_ASSERT_MSG_EQ(ant.cloudy, false, "no fit, no cloud");
        NS_TEST_ASSERT_MSG_EQ(ant.LwcExceededKgM2(0.01), 0.0, "L = 0 there");
        // Between nodes the value is bracketed by the neighbors.
        const auto mid = itu::P840SiteStatistics(51.5, 0.5);
        NS_TEST_ASSERT_MSG_GT(mid.pLPercent, 0.0, "interpolated P_L");

        // Model object: Exceedance mode gives K_L L / sin(theta).
        auto cm = CreateObject<ThzNtnCloudAttenuation>();
        cm->SetMode(ThzNtnCloudAttenuation::EXCEEDANCE);
        cm->SetAttribute("ExceedancePercent", DoubleValue(1.0));
        const double a = cm->ComputeAttenuationDb(140e9, 30.0, 51.0, 0.0);
        NS_TEST_ASSERT_MSG_EQ_TOL(a, 7.81616, 0.03, "A_C(140 GHz, 30 deg, 1 %) London");
        // Below 5 deg the path factor is held and counted.
        const double low = cm->ComputeAttenuationDb(140e9, 2.0, 51.0, 0.0);
        NS_TEST_ASSERT_MSG_EQ_TOL(low, 6.94932 * 0.56237 / std::sin(5.0 * kDegRad), 0.1,
                                  "held at 5 deg");
        NS_TEST_ASSERT_MSG_EQ(cm->GetLowElevationCount(), 1u, "low elevation counted");
        cm->ComputeAttenuationDb(300e9, 30.0, 51.0, 0.0);
        NS_TEST_ASSERT_MSG_EQ(cm->GetExtrapolatedFrequencyCount(), 1u, "300 GHz flagged");
        // Failure path: no site and no default site.
        const double ns = cm->ComputeAttenuationDb(140e9, 30.0, NAN, NAN);
        NS_TEST_ASSERT_MSG_EQ(ns, 0.0, "no site, no term");
        NS_TEST_ASSERT_MSG_EQ(cm->GetNoSiteCount(), 1u, "counted");
        // Off.
        auto off = CreateObject<ThzNtnCloudAttenuation>();
        off->SetMode(ThzNtnCloudAttenuation::OFF);
        NS_TEST_ASSERT_MSG_EQ(off->ComputeAttenuationDb(140e9, 30.0, 51.0, 0.0), 0.0, "off");
        // SampledPerRun: one p per run, reproducible for a stream.
        auto r1 = CreateObject<ThzNtnCloudAttenuation>();
        auto r2 = CreateObject<ThzNtnCloudAttenuation>();
        r1->AssignStreams(77);
        r2->AssignStreams(77);
        const double p1 = r1->GetRunExceedancePercent();
        NS_TEST_ASSERT_MSG_EQ(p1, r2->GetRunExceedancePercent(), "same stream, same p");
        NS_TEST_ASSERT_MSG_EQ((p1 > 0.0 && p1 < 100.0), true, "p in (0, 100)");
        NS_TEST_ASSERT_MSG_EQ(r1->GetRunExceedancePercent(), p1, "drawn once per run");
    }
};

/// P.834-9 eqs. (8)-(14).
class ThzNtnP834ApparentElevationTest : public TestCase
{
  public:
    ThzNtnP834ApparentElevationTest()
        : TestCase("Wave 7C: P.834-9 free-space to apparent elevation and back")
    {
    }

  private:
    void DoRun() override
    {
        using itu::P834;
        NS_TEST_ASSERT_MSG_EQ_TOL(P834::CorrectionFromFreeSpaceDeg(0.0, 0.0), 1.0 / 1.728, 1e-9,
                                  "tau_s(0, 0) = 1/1.728 deg");
        NS_TEST_ASSERT_MSG_EQ_TOL(P834::CorrectionFromFreeSpaceDeg(5.0, 0.0),
                                  1.0 / (1.728 + 0.5411 * 5 + 0.03723 * 25), 1e-9,
                                  "tau_s(0, 5 deg)");
        const double h = 1.0;
        const double t0 = 3.0;
        const double den = 1.728 + 0.5411 * t0 + 0.03723 * t0 * t0 +
                           h * (0.1815 + 0.06272 * t0 + 0.01138 * t0 * t0) +
                           h * h * (0.01727 + 0.008288 * t0);
        NS_TEST_ASSERT_MSG_EQ_TOL(P834::ApparentElevationDeg(t0, h), t0 + 1.0 / den, 1e-9,
                                  "eq. (14) with the station height");
        for (double e : {0.5, 2.0, 5.0, 10.0, 30.0})
        {
            const double app = P834::ApparentElevationDeg(e, 0.0);
            NS_TEST_ASSERT_MSG_GT(app, e, "refraction raises the apparent elevation");
            NS_TEST_ASSERT_MSG_EQ_TOL(P834::FreeSpaceElevationDeg(app, 0.0), e, 0.01,
                                      "eq. (9) inverts eq. (14) to 0.01 deg");
        }
        NS_TEST_ASSERT_MSG_LT(P834::CorrectionFromFreeSpaceDeg(90.0, 0.0), 0.005,
                              "negligible at the zenith");
        NS_TEST_ASSERT_MSG_EQ_TOL(P834::MinimumElevationDeg(0.0), 0.0, 1e-9, "theta_m(0) = 0");
        NS_TEST_ASSERT_MSG_LT(P834::MinimumElevationDeg(3.0), -1.0, "elevated station sees below 0");
        // Failure path: below the radio horizon.
        NS_TEST_ASSERT_MSG_EQ(P834::IsVisible(-3.0, 0.0), false, "not visible at -3 deg");
        NS_TEST_ASSERT_MSG_EQ(P834::IsVisible(-0.3, 0.0), true, "refraction lifts -0.3 deg");
        NS_TEST_ASSERT_MSG_EQ(P834::ApparentElevationDeg(2.0, 9.0),
                              P834::ApparentElevationDeg(2.0, 3.0), "height clamped at 3 km");
    }
};

/// Live ECEF pass through ThzNtnPropagationLossModel: apparent elevation into
/// the Annex 1 ray trace, P.840-9 cloud at the ground site, Off reproduces.
class ThzNtnCloudRefractionPacketPathSimTest : public TestCase
{
  public:
    ThzNtnCloudRefractionPacketPathSimTest()
        : TestCase("Wave 7C: packet-path loss carries P.834 elevation and P.840-9 cloud (Run)")
    {
    }

  private:
    struct Sample
    {
        double trueEl;
        double appEl;
        double gas;
        double cloud;
        double total;
        double totalOff;
    };

    static void Probe(Ptr<ThzNtnPropagationLossModel> pl,
                      Ptr<ThzNtnPropagationLossModel> legacy,
                      Ptr<MobilityModel> g,
                      Ptr<MobilityModel> s,
                      std::vector<Sample>* out)
    {
        pl->CalcRxPower(0.0, s, g);
        legacy->CalcRxPower(0.0, s, g);
        out->push_back({pl->GetLastTrueElevationDeg(), pl->GetLastApparentElevationDeg(),
                        pl->GetLastGasDb(), pl->GetLastCloudDb(), pl->GetLastLossDb(),
                        legacy->GetLastLossDb()});
    }

    void DoRun() override
    {
        Ptr<ConstantPositionMobilityModel> gnd;
        Ptr<ConstantVelocityMobilityModel> sat;
        MakePass(51.0, 0.0, 2000.0, gnd, sat);
        auto pl = CreateObject<ThzNtnPropagationLossModel>();
        pl->SetFrequency(140e9);
        pl->GetCloudModel()->SetMode(ThzNtnCloudAttenuation::EXCEEDANCE);
        pl->GetCloudModel()->SetAttribute("ExceedancePercent", DoubleValue(1.0));
        auto legacy = CreateObject<ThzNtnPropagationLossModel>();
        legacy->SetFrequency(140e9);
        legacy->SetAttribute("ApparentElevation", BooleanValue(false));
        legacy->GetCloudModel()->SetMode(ThzNtnCloudAttenuation::OFF);

        std::vector<Sample> v;
        for (int t = 0; t <= 260; t += 20)
        {
            Simulator::Schedule(Seconds(t), &Probe, pl, legacy, gnd, sat, &v);
        }
        Simulator::Run();
        Simulator::Destroy();

        NS_TEST_ASSERT_MSG_EQ(v.size(), 14u, "fourteen samples");
        auto ref = CreateObject<ThzNtnMolecularAbsorption>();
        const double kl = itu::P840CloudMassAbsorptionCoefficient(140.0);
        const double l1 = itu::P840SiteStatistics(51.0, 0.0).LwcExceededKgM2(1.0);
        for (const auto& s : v)
        {
            NS_TEST_ASSERT_MSG_EQ_TOL(s.appEl,
                                      itu::P834::ApparentElevationDeg(s.trueEl, 0.0),
                                      1e-6,
                                      "apparent elevation from eq. (14)");
            NS_TEST_ASSERT_MSG_EQ_TOL(s.gas,
                                      ref->ComputeSlantPathAbsorption(140e9, s.appEl, 0.0, 550.0),
                                      1e-6 * s.gas + 1e-9,
                                      "gas is the ray trace at the apparent elevation");
            const double el = std::max(5.0, s.appEl);
            NS_TEST_ASSERT_MSG_EQ_TOL(s.cloud, kl * l1 / std::sin(el * kDegRad), 0.03 * s.cloud,
                                      "cloud = K_L L(1 %) / sin(theta)");
            NS_TEST_ASSERT_MSG_GT(s.total, s.totalOff, "cloud adds loss over the legacy path");
        }
        // The pass rises to the zenith and the low-elevation end costs more.
        NS_TEST_ASSERT_MSG_GT(v.front().gas, v.back().gas, "gas falls as the satellite rises");
        NS_TEST_ASSERT_MSG_LT(v.front().trueEl, 20.0, "starts low");
        NS_TEST_ASSERT_MSG_GT(v.back().trueEl, 70.0, "ends high");
        // Refraction is largest at the lowest elevation and the legacy path
        // (no P.834) reads less gas there.
        NS_TEST_ASSERT_MSG_GT(v.front().appEl - v.front().trueEl,
                              v.back().appEl - v.back().trueEl,
                              "refraction shrinks with elevation");

        // Failure path: a flat ENU frame with no site carries no cloud and
        // says so in the run report.
        auto flatG = CreateObject<ConstantPositionMobilityModel>();
        flatG->SetPosition(Vector(0, 0, 0));
        auto flatS = CreateObject<ConstantPositionMobilityModel>();
        flatS->SetPosition(Vector(100e3, 0, 550e3));
        auto flat = CreateObject<ThzNtnPropagationLossModel>();
        flat->SetFrequency(140e9);
        flat->CalcRxPower(0.0, flatS, flatG);
        NS_TEST_ASSERT_MSG_EQ(flat->GetLastCloudDb(), 0.0, "no site, no cloud");
        NS_TEST_ASSERT_MSG_EQ(flat->GetCloudModel()->GetNoSiteCount(), 1u, "counted");
        const bool reported =
            flat->GetCloudModel()->GetRunReport().find("no geodetic site") != std::string::npos;
        NS_TEST_ASSERT_MSG_EQ(reported, true, "reported");
        // With a declared site the flat frame gets the site's cloud.
        auto sited = CreateObject<ThzNtnPropagationLossModel>();
        sited->SetFrequency(140e9);
        sited->GetCloudModel()->SetDefaultSite(51.0, 0.0);
        sited->GetCloudModel()->SetMode(ThzNtnCloudAttenuation::EXCEEDANCE);
        sited->CalcRxPower(0.0, flatS, flatG);
        NS_TEST_ASSERT_MSG_GT(sited->GetLastCloudDb(), 0.0, "declared site carries cloud");
    }
};

/// ThzNtnPacketLink over a 550 km pass: live delay, ACM, BLER decode, and the
/// failure paths (outage on a starved link, legacy threshold decode).
class ThzNtnPacketLinkPassSimTest : public TestCase
{
  public:
    ThzNtnPacketLinkPassSimTest()
        : TestCase("Wave 7C: THz packet link tracks delay, adapts MODCOD, decodes by BLER (Run)")
    {
    }

  private:
    struct Result
    {
        uint64_t tx{0};
        uint64_t rx{0};
        double minDelayMs{0};
        double maxDelayMs{0};
        uint64_t changes{0};
        double outageS{0};
        double meanRate{0};
        uint64_t corrupted{0};
        double minSnr{0};
        double maxSnr{0};
    };

    static Result RunPass(double groundGainDbi, bool thresholdDecode, bool acm)
    {
        Ptr<ConstantPositionMobilityModel> gnd;
        Ptr<ConstantVelocityMobilityModel> sat;
        MakePass(51.0, 0.0, 600.0, gnd, sat);
        NodeContainer nodes;
        nodes.Create(2);
        nodes.Get(0)->AggregateObject(sat);
        nodes.Get(1)->AggregateObject(gnd);

        auto link = CreateObject<ThzNtnPacketLink>();
        ThzNtnPacketLink::Config c;
        c.freqHz = 140e9;
        c.txPowerDbm = 30.0;
        c.spaceGainDbi = 50.0;
        c.groundGainDbi = groundGainDbi;
        c.bandwidthHz = 400e6;
        c.acm = acm;
        c.thresholdDecode = thresholdDecode;
        c.fixedModcod = SatEnums::SAT_MODCOD_8PSK_3_TO_4;
        link->SetConfig(c);
        NetDeviceContainer devs = link->Install(nodes.Get(0), nodes.Get(1), sat, gnd);
        link->GetAtmosphere()->GetCloudModel()->SetMode(ThzNtnCloudAttenuation::EXCEEDANCE);
        link->GetAtmosphere()->GetCloudModel()->SetAttribute("ExceedancePercent",
                                                             DoubleValue(5.0));
        link->AssignStreams(11);

        InternetStackHelper ip;
        ip.Install(nodes);
        Ipv4AddressHelper a;
        a.SetBase("10.77.0.0", "255.255.255.0");
        Ipv4InterfaceContainer ifs = a.Assign(devs);
        PacketSinkHelper sink("ns3::UdpSocketFactory", InetSocketAddress(Ipv4Address::GetAny(), 9));
        ApplicationContainer sk = sink.Install(nodes.Get(1));
        OnOffHelper src("ns3::UdpSocketFactory", InetSocketAddress(ifs.GetAddress(1), 9));
        src.SetConstantRate(DataRate("20Mbps"), 1400);
        ApplicationContainer sa = src.Install(nodes.Get(0));
        sa.Start(Seconds(0.5));
        sa.Stop(Seconds(95.0));
        uint64_t tx = 0;
        sa.Get(0)->TraceConnectWithoutContext(
            "Tx", MakeBoundCallback(+[](uint64_t* n, Ptr<const Packet>) { ++*n; }, &tx));
        Simulator::Stop(Seconds(96.0));
        Simulator::Run();
        Result r;
        r.tx = tx;
        r.rx = DynamicCast<PacketSink>(sk.Get(0))->GetTotalRx() / 1400;
        r.minDelayMs = link->GetMinDelayMs();
        r.maxDelayMs = link->GetMaxDelayMs();
        r.changes = link->GetModcodChanges();
        r.outageS = link->GetOutageSeconds();
        r.meanRate = link->GetMeanRateMbps();
        r.corrupted = link->GetDownlinkErrorModel()->GetPacketsCorrupted();
        r.minSnr = link->GetMinSnrDb();
        r.maxSnr = link->GetMaxSnrDb();
        Simulator::Destroy();
        return r;
    }

    void DoRun() override
    {
        // Normal: 0.4 GHz at 140 GHz, 50 + 50 dBi, a pass from ~42 deg through the zenith.
        const Result ok = RunPass(50.0, false, true);
        NS_TEST_ASSERT_MSG_GT(ok.tx, 160000u, "traffic offered");
        const double pdr = static_cast<double>(ok.rx) / ok.tx;
        NS_TEST_ASSERT_MSG_GT(pdr, 0.98, "ACM keeps the link closed");
        NS_TEST_ASSERT_MSG_EQ_TOL(ok.minDelayMs, 550e3 / 299792458.0 * 1e3, 0.01,
                                  "zenith delay = 550 km / c");
        NS_TEST_ASSERT_MSG_GT(ok.maxDelayMs, ok.minDelayMs + 0.8,
                              "delay follows the slant range (not a constant)");
        NS_TEST_ASSERT_MSG_GT(ok.changes, 0u, "MODCOD adapts over the pass");
        NS_TEST_ASSERT_MSG_GT(ok.maxSnr, ok.minSnr + 3.0, "SNR varies over the pass");
        NS_TEST_ASSERT_MSG_EQ(ok.outageS, 0.0, "no outage");
        NS_TEST_ASSERT_MSG_GT(ok.meanRate, 20.0, "line rate above the offered load");

        // Failure path: a 20 dBi ground aperture cannot close the link.
        const Result starved = RunPass(20.0, false, true);
        NS_TEST_ASSERT_MSG_GT(starved.outageS, 60.0, "outage most of the pass");
        NS_TEST_ASSERT_MSG_LT(static_cast<double>(starved.rx) / std::max<uint64_t>(1, starved.tx),
                              0.2,
                              "few packets decoded");

        // Fixed MODCOD with the legacy step decode: delivery is all or nothing
        // per update, so the BLER decode (above) is what produces gradual loss.
        const Result step = RunPass(50.0, true, false);
        NS_TEST_ASSERT_MSG_GT(step.tx, 0u, "offered");
        NS_TEST_ASSERT_MSG_EQ(step.changes, 0u, "fixed MODCOD never changes");
    }
};

std::vector<TestCase*>
MakeThzNtnWave7cTests()
{
    return {new ThzNtnP840CloudModelTest,
            new ThzNtnP834ApparentElevationTest,
            new ThzNtnCloudRefractionPacketPathSimTest,
            new ThzNtnPacketLinkPassSimTest};
}
