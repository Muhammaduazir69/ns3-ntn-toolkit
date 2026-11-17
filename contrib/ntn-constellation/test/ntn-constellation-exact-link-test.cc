/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only
//
// Wave 7B tests: exact per-packet propagation delay (NtnRangeDelayChannel),
// the ISL margin skip of ContactGraphScheduler, and the gated TR 38.821 Ka
// ground-link error model. Every case runs packets or contacts through
// Simulator::Run() and has a failure path.

#include "ns3/boolean.h"
#include "ns3/constant-position-mobility-model.h"
#include "ns3/contact-graph-scheduler.h"
#include "ns3/double.h"
#include "ns3/enum.h"
#include "ns3/ntn-geocentric-mobility-adapter.h"
#include "ns3/ntn-range-delay-channel.h"
#include "ns3/ntn-sat-link-error-model.h"
#include "ns3/orbital-elements.h"
#include "ns3/node.h"
#include "ns3/packet.h"
#include "ns3/pointer.h"
#include "ns3/point-to-point-net-device.h"
#include "ns3/sgp4-mobility-model.h"
#include "ns3/simulator.h"
#include "ns3/test.h"
#include "ns3/walker-constellation.h"

#include <cmath>
#include <string>
#include <tuple>
#include <vector>

using namespace ns3;
using namespace ns3::ntncon;

namespace
{
constexpr double kC = 299792458.0;

struct Rx
{
    std::vector<double> t;
    bool OnRx(Ptr<NetDevice>, Ptr<const Packet>, uint16_t, const Address&)
    {
        t.push_back(Simulator::Now().GetSeconds());
        return true;
    }
};

Ptr<Sgp4MobilityModel>
MakeSat(uint32_t idx = 0)
{
    WalkerConfig cfg;
    cfg.num_planes = 1;
    cfg.total_sats = 20;
    cfg.altitude_km = 550.0;
    cfg.inclination_deg = 53.0;
    cfg.epoch_unix_s = 1735689600.0;
    auto els = WalkerConstellation::BuildDelta(cfg);
    auto m = CreateObject<Sgp4MobilityModel>();
    m->SetElements(els[idx]);
    return m;
}

/// Reference light-time, written independently of the channel: exact SGP4
/// receiver state at t_e + tau (no linear extrapolation), Sagnac rotation,
/// iterated to convergence.
double
RefDelay(Ptr<MobilityModel> tx, Ptr<Sgp4MobilityModel> rxSat, double emitOffset)
{
    const Vector pt = tx->GetPosition();
    double tau = 0.0;
    for (int it = 0; it < 6; ++it)
    {
        Vector pr;
        Vector vr;
        rxSat->GetEcefStateAt(Seconds(emitOffset + tau), pr, vr);
        const double a = kEarthRotationRadS * tau;
        const Vector q(std::cos(a) * pr.x - std::sin(a) * pr.y,
                       std::sin(a) * pr.x + std::cos(a) * pr.y,
                       pr.z);
        tau = CalculateDistance(pt, q) / kC;
    }
    return tau;
}

void
SendAt(Ptr<PointToPointNetDevice> dev, std::vector<double>* sent)
{
    sent->push_back(Simulator::Now().GetSeconds());
    dev->Send(Create<Packet>(100), dev->GetBroadcast(), 0x0800);
}
} // namespace

/// The channel applies the light-time delay on the exact orbit to every
/// packet; a cached-position staircase would miss it by up to a microsecond.
class RangeDelayChannelExactTest : public TestCase
{
  public:
    RangeDelayChannelExactTest()
        : TestCase("NtnRangeDelayChannel: per-packet light-time delay on the exact SGP4 state")
    {
    }

  private:
    void DoRun() override
    {
        Ptr<Sgp4MobilityModel> sat = MakeSat();
        double lat;
        double lon;
        double alt;
        sat->GetGeodetic(lat, lon, alt);
        // Ground station 6 deg north of the t = 0 sub-point: a large range rate.
        auto gs = CreateObject<ConstantPositionMobilityModel>();
        gs->SetPosition(NtnGeocentricMobilityAdapter::GeodeticToEcef(Vector(lat + 6.0, lon, 0.0)));
        Ptr<Node> a = CreateObject<Node>();
        Ptr<Node> b = CreateObject<Node>();
        a->AggregateObject(gs);
        b->AggregateObject(sat);
        Ptr<NtnRangeDelayChannel> ch;
        const DataRate rate("1Gbps");
        NetDeviceContainer d = NtnRangeDelayChannel::Connect(a, b, rate, "100p", ch);
        Rx rx;
        d.Get(1)->SetReceiveCallback(MakeCallback(&Rx::OnRx, &rx));
        std::vector<double> sent;
        Ptr<PointToPointNetDevice> tx = DynamicCast<PointToPointNetDevice>(d.Get(0));
        // One packet every 37 ms (off the 100 ms cache grain) for 3 s.
        for (int i = 0; i < 80; ++i)
        {
            Simulator::Schedule(Seconds(0.011 + 0.037 * i), &SendAt, tx, &sent);
        }
        // Reference delays evaluated at the same emission instants.
        std::vector<double> ref;
        const double txTime = rate.CalculateBytesTxTime(102).GetSeconds(); // + PPP header
        for (int i = 0; i < 80; ++i)
        {
            ref.push_back(RefDelay(gs, sat, 0.011 + 0.037 * i + txTime));
        }
        Simulator::Stop(Seconds(4.0));
        Simulator::Run();

        NS_TEST_ASSERT_MSG_EQ(rx.t.size(), 80u, "every packet delivered");
        NS_TEST_ASSERT_MSG_EQ(ch->GetPacketCount(), 80u, "every packet timed by the channel");
        double maxErr = 0.0;
        double maxCacheErr = 0.0;
        for (size_t i = 0; i < rx.t.size(); ++i)
        {
            const double meas = rx.t[i] - sent[i] - txTime;
            maxErr = std::max(maxErr, std::abs(meas - ref[i]));
            // What a delay held from the 100 ms position cache would have been:
            // the range at the start of the grain.
            const double grain = std::floor((sent[i] + 1e-12) / 0.1) * 0.1;
            Vector p;
            Vector v;
            sat->GetEcefStateAt(Seconds(grain - Simulator::Now().GetSeconds()), p, v);
            maxCacheErr = std::max(maxCacheErr,
                                   std::abs(CalculateDistance(p, gs->GetPosition()) / kC - ref[i]));
        }
        // ns time resolution: 1 ns.
        NS_TEST_ASSERT_MSG_LT(maxErr, 2e-9, "measured delay equals the light-time solution");
        NS_TEST_ASSERT_MSG_GT(maxCacheErr, 3e-7,
                              "the 100 ms cached delay is off by more than 0.3 us somewhere");
        // The delay moves monotonically within the pass segment: no repeats
        // (a staircase repeats the same delay for a whole grain).
        uint32_t repeats = 0;
        for (size_t i = 1; i < rx.t.size(); ++i)
        {
            const double d0 = rx.t[i - 1] - sent[i - 1];
            const double d1 = rx.t[i] - sent[i];
            repeats += std::abs(d1 - d0) < 1e-12 ? 1 : 0;
        }
        NS_TEST_ASSERT_MSG_EQ(repeats, 0u, "no two packets see the same delay");
        NS_TEST_ASSERT_MSG_GT(ch->GetMaxDelayS(), ch->GetMinDelayS(), "delay spread recorded");
        Simulator::Destroy();
    }
};

/// Failure paths: without mobility the channel falls back to the Delay
/// attribute; FIXED mode ignores geometry.
class RangeDelayChannelFallbackTest : public TestCase
{
  public:
    RangeDelayChannelFallbackTest()
        : TestCase("NtnRangeDelayChannel: no mobility or FIXED mode use the Delay attribute")
    {
    }

  private:
    void DoRun() override
    {
        for (int mode = 0; mode < 2; ++mode)
        {
            Ptr<Node> a = CreateObject<Node>();
            Ptr<Node> b = CreateObject<Node>();
            if (mode == 1)
            {
                a->AggregateObject(CreateObject<ConstantPositionMobilityModel>());
                auto far = CreateObject<ConstantPositionMobilityModel>();
                far->SetPosition(Vector(3.0e6, 0, 0)); // 10 ms of geometry, ignored
                b->AggregateObject(far);
            }
            Ptr<NtnRangeDelayChannel> ch;
            const DataRate rate("1Gbps");
            NetDeviceContainer d = NtnRangeDelayChannel::Connect(a, b, rate, "10p", ch);
            ch->SetAttribute("Delay", TimeValue(MilliSeconds(7)));
            if (mode == 1)
            {
                ch->SetAttribute("DelayMode", EnumValue(NtnRangeDelayChannel::FIXED));
            }
            Rx rx;
            d.Get(1)->SetReceiveCallback(MakeCallback(&Rx::OnRx, &rx));
            std::vector<double> sent;
            Simulator::Schedule(Seconds(1.0), &SendAt, DynamicCast<PointToPointNetDevice>(d.Get(0)),
                                &sent);
            Simulator::Run();
            NS_TEST_ASSERT_MSG_EQ(rx.t.size(), 1u, "delivered");
            const double txTime = rate.CalculateBytesTxTime(102).GetSeconds();
            NS_TEST_ASSERT_MSG_EQ_TOL(rx.t[0] - 1.0 - txTime, 7e-3, 2e-9, "Delay attribute used");
            Simulator::Destroy();
        }
    }
};

namespace
{
using Ev = std::tuple<int64_t, uint32_t, uint32_t, bool>;
void
Rec(std::vector<Ev>* out, ContactEvent ev)
{
    out->emplace_back(Simulator::Now().GetNanoSeconds(), ev.node_a, ev.node_b, ev.up);
}

/// Six-plane +Grid shell whose cross-plane links cross a 2600 km cap.
std::vector<Ev>
RunShell(bool skip, double crossLat, uint64_t& checks, uint64_t& skipped)
{
    WalkerConfig cfg;
    cfg.num_planes = 6;
    cfg.total_sats = 36;
    cfg.phasing_f = 1;
    cfg.altitude_km = 550.0;
    cfg.inclination_deg = 53.0;
    cfg.epoch_unix_s = 1735689600.0;
    auto els = WalkerConstellation::BuildDelta(cfg);
    Ptr<ContactGraphScheduler> s = CreateObject<ContactGraphScheduler>();
    s->SetAttribute("IslMarginSkip", BooleanValue(skip));
    s->SetSamplingInterval(MilliSeconds(100));
    s->SetMaxIslRangeM(2.6e6);
    s->SetCrossPlaneLatCutoffDeg(crossLat);
    for (uint32_t k = 0; k < els.size(); ++k)
    {
        auto m = CreateObject<Sgp4MobilityModel>();
        m->SetElements(els[k]);
        s->RegisterSatellite(k, m);
    }
    for (uint32_t p = 0; p < 6; ++p)
    {
        for (uint32_t k = 0; k < 6; ++k)
        {
            const uint32_t a = p * 6 + k;
            s->RegisterIslCandidate(a, p * 6 + (k + 1) % 6, false);
            s->RegisterIslCandidate(a, ((p + 1) % 6) * 6 + k, true);
        }
    }
    std::vector<Ev> ev;
    s->m_contactUp.ConnectWithoutContext(MakeBoundCallback(&Rec, &ev));
    s->m_contactDown.ConnectWithoutContext(MakeBoundCallback(&Rec, &ev));
    s->Start();
    Simulator::Stop(Seconds(1500.0));
    Simulator::Run();
    s->Stop();
    checks = s->IslChecks();
    skipped = s->IslChecksSkipped();
    Simulator::Destroy();
    return ev;
}
} // namespace

/// The margin skip is exact: the same ISL transitions at the same nanosecond,
/// with most look-ahead evaluations skipped.
class SchedulerIslMarginSkipTest : public TestCase
{
  public:
    SchedulerIslMarginSkipTest()
        : TestCase("ContactGraphScheduler: IslMarginSkip gives identical ISL transitions")
    {
    }

  private:
    void DoRun() override
    {
        uint64_t cOn;
        uint64_t sOn;
        uint64_t cOff;
        uint64_t sOff;
        const auto on = RunShell(true, 0.0, cOn, sOn);
        const auto off = RunShell(false, 0.0, cOff, sOff);
        uint32_t downs = 0;
        for (const auto& e : off)
        {
            downs += std::get<3>(e) ? 0 : 1;
        }
        NS_TEST_ASSERT_MSG_GT(downs, 0u, "the shell has ISL transitions to reproduce");
        NS_TEST_ASSERT_MSG_EQ(on.size(), off.size(), "same number of ISL events");
        NS_TEST_ASSERT_MSG_EQ((on == off), true, "same events at the same nanosecond");
        NS_TEST_ASSERT_MSG_EQ(sOff, 0u, "no skip when disabled");
        NS_TEST_ASSERT_MSG_GT(sOn, 2 * cOn, "most evaluations skipped");
        NS_TEST_ASSERT_MSG_EQ(cOn + sOn, cOff, "every pair accounted for on every refresh");

        // Failure path: with a cross-plane latitude cutoff no bound is kept for
        // cross-plane pairs, so they are evaluated on every refresh; results
        // still match the unskipped run.
        uint64_t cLatOn;
        uint64_t sLatOn;
        uint64_t cLatOff;
        uint64_t sLatOff;
        const auto latOn = RunShell(true, 50.0, cLatOn, sLatOn);
        const auto latOff = RunShell(false, 50.0, cLatOff, sLatOff);
        NS_TEST_ASSERT_MSG_EQ((latOn == latOff), true, "cutoff run identical");
        NS_TEST_ASSERT_MSG_GT(cLatOn, cLatOff / 2 - 1, "cross-plane half never skipped");
    }
};

/// The gated ground-link model: gate down drops everything (counted as gate
/// drops), clear sky closes the link at the designed MODCOD, a heavy rain
/// fade at 0.01 % drops packets through the DVB-S2 curve.
class SatLinkGateAndModcodTest : public TestCase
{
  public:
    SatLinkGateAndModcodTest()
        : TestCase("NtnSatLinkErrorModel: contact gate, TR 38.821 Ka budget, DVB-S2 MODCOD design")
    {
    }

  private:
    uint64_t RunLink(bool gateUp, double rain, double designElev, double rateBps,
                     Ptr<NtnSatLinkErrorModel>& emOut)
    {
        Ptr<Sgp4MobilityModel> sat = MakeSat();
        double lat;
        double lon;
        double alt;
        sat->GetGeodetic(lat, lon, alt);
        auto gs = CreateObject<ConstantPositionMobilityModel>();
        gs->SetPosition(NtnGeocentricMobilityAdapter::GeodeticToEcef(Vector(lat, lon, 0.0)));
        Ptr<Node> a = CreateObject<Node>();
        Ptr<Node> b = CreateObject<Node>();
        a->AggregateObject(sat);
        b->AggregateObject(gs);
        Ptr<NtnRangeDelayChannel> ch;
        NetDeviceContainer d = NtnRangeDelayChannel::Connect(a, b, DataRate(rateBps), "200p", ch);
        emOut = CreateObject<NtnSatLinkErrorModel>();
        emOut->ConfigureTr38821Set1KaGsl(true, rateBps, lat, designElev, 550.0);
        emOut->SetAttribute("RainRateMmH", DoubleValue(rain));
        emOut->SetEndpoints(sat, gs);
        emOut->SetContactUp(gateUp);
        d.Get(1)->SetAttribute("ReceiveErrorModel", PointerValue(emOut));
        Rx rx;
        d.Get(1)->SetReceiveCallback(MakeCallback(&Rx::OnRx, &rx));
        std::vector<double> sent;
        for (int i = 0; i < 100; ++i)
        {
            Simulator::Schedule(Seconds(0.01 * i), &SendAt,
                                DynamicCast<PointToPointNetDevice>(d.Get(0)), &sent);
        }
        Simulator::Run();
        Simulator::Destroy();
        return rx.t.size();
    }

    void DoRun() override
    {
        Ptr<NtnSatLinkErrorModel> em;
        // Gate down: nothing gets through, all counted as gate drops.
        NS_TEST_ASSERT_MSG_EQ(RunLink(false, 0.0, 25.0, 100e6, em), 0u, "gate down drops all");
        NS_TEST_ASSERT_MSG_EQ(em->GetGateDrops(), 100u, "gate drops counted");
        NS_TEST_ASSERT_MSG_EQ(em->GetBlerDrops(), 0u, "no BLER draw while gated");

        // Clear sky, satellite overhead, link designed for 25 deg: closes.
        NS_TEST_ASSERT_MSG_EQ(RunLink(true, 0.0, 25.0, 100e6, em), 100u, "clear sky delivers all");
        NS_TEST_ASSERT_MSG_EQ(em->GetModcodName(), std::string("8PSK 2/3"),
                              "25 deg design picks 8PSK 2/3");
        NS_TEST_ASSERT_MSG_GT(em->LastEsNoDb(), em->GetDesignEsNoDb(), "zenith beats the design point");
        NS_TEST_ASSERT_MSG_LT(em->GetMeanBler(), 1e-6, "DVB-S2 curve: no loss with margin");

        // MODCOD ladder: zenith design picks the top QPSK/8PSK MODCOD, 10 deg a
        // QPSK one whose QEF + 0.5 dB margin is met.
        Ptr<NtnSatLinkErrorModel> z = CreateObject<NtnSatLinkErrorModel>();
        z->ConfigureTr38821Set1KaGsl(true, 100e6, 45.0, 90.0, 550.0);
        NS_TEST_ASSERT_MSG_EQ(z->GetModcodName(), std::string("8PSK 9/10"), "zenith design");
        Ptr<NtnSatLinkErrorModel> lo = CreateObject<NtnSatLinkErrorModel>();
        lo->ConfigureTr38821Set1KaGsl(true, 100e6, 45.0, 10.0, 550.0);
        NS_TEST_ASSERT_MSG_EQ(lo->GetModcodName().substr(0, 4), std::string("QPSK"), "10 deg: QPSK");
        NS_TEST_ASSERT_MSG_GT(lo->GetDesignEsNoDb(), 0.0, "10 deg design closes");

        // Failure path: an ITU-R P.618 R0.01 = 60 mm/h fade (about 20 dB at
        // 20 GHz near zenith) on the 25 deg design: the DVB-S2 curve drops packets.
        const uint64_t got = RunLink(true, 60.0, 25.0, 100e6, em);
        NS_TEST_ASSERT_MSG_LT(got, 100u, "rain fade loses packets");
        NS_TEST_ASSERT_MSG_EQ(em->GetBlerDrops(), 100u - got, "losses counted as BLER drops");
        NS_TEST_ASSERT_MSG_GT(em->LastRainDb(), 5.0, "P.618 rain term applied");
    }
};

class NtnConstellationExactLinkTestSuite : public TestSuite
{
  public:
    NtnConstellationExactLinkTestSuite()
        : TestSuite("ntn-constellation-exact-link", Type::UNIT)
    {
        AddTestCase(new RangeDelayChannelExactTest, Duration::QUICK);
        AddTestCase(new RangeDelayChannelFallbackTest, Duration::QUICK);
        AddTestCase(new SchedulerIslMarginSkipTest, Duration::QUICK);
        AddTestCase(new SatLinkGateAndModcodTest, Duration::QUICK);
    }
};

static NtnConstellationExactLinkTestSuite g_ntnConstellationExactLinkTestSuite;
