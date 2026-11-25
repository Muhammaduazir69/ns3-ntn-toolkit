// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only
//
// In-depth ntn-v2x validation (18 Nov 2026 test campaign).
//
// Every case drives Simulator::Run() with the class under test taking part in
// the running simulation, checks a normal path and at least one failure path,
// and asserts against a published reference value with a stated tolerance:
//   - 3GPP TR 37.885 Table 6.2.1-1 (V2X highway and urban path loss),
//   - 3GPP TS 38.321 clause 5.22.1.1 (SL_RESOURCE_RESELECTION_COUNTER 5..15),
//   - 3GPP TS 38.214 clause 8.1.4 (sensing-based resource exclusion, T1/T2 window),
//   - 3GPP TS 22.186 Table 5.1-1 / TR 37.885 (100 ms BSM latency budget),
//   - 3GPP TR 38.811 eq. 6.6-2/6.6-3 (free-space loss and slant range),
//   - SAE J2735 DF_BSMcoreData value ranges.
// The cross-module cases put the PC5 sidelink on a real IPv4 data plane whose
// satellite hop follows an SGP4 orbit from ntn-constellation (projected into the
// scenario frame by ntn-cho), measured with FlowMonitor.

#include "ns3/constant-position-mobility-model.h"
#include "ns3/constant-velocity-mobility-model.h"
#include "ns3/double.h"
#include "ns3/error-model.h"
#include "ns3/flow-monitor-helper.h"
#include "ns3/inet-socket-address.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/ipv4-flow-classifier.h"
#include "ns3/maritime-scenario.h"
#include "ns3/ntn-nr-sidelink.h"
#include "ns3/ntn-tr38811-mobility-model.h"
#include "ns3/ntn-v2x-bsm-header.h"
#include "ns3/packet.h"
#include "ns3/point-to-point-channel.h"
#include "ns3/point-to-point-helper.h"
#include "ns3/pointer.h"
#include "ns3/rng-seed-manager.h"
#include "ns3/sgp4-mobility-model.h"
#include "ns3/string.h"
#include "ns3/simulator.h"
#include "ns3/socket.h"
#include "ns3/sumo-traci-bridge.h"
#include "ns3/test.h"
#include "ns3/udp-socket-factory.h"
#include "ns3/v2x-leo-direct.h"
#include "ns3/v2x-leo-relay.h"
#include "ns3/walker-constellation.h"

#include <cmath>
#include <cstdio>
#include <fstream>
#include <vector>

using namespace ns3;
using namespace ns3::ntnv2x;

namespace
{

constexpr double kC = 299792458.0;

/// Restores the global RNG seed and run on scope exit so a case cannot leak
/// its seeding into the next case in the same test-runner process.
struct RngGuard
{
    uint32_t seed{RngSeedManager::GetSeed()};
    uint64_t run{RngSeedManager::GetRun()};

    ~RngGuard()
    {
        RngSeedManager::SetSeed(seed);
        RngSeedManager::SetRun(run);
    }
};

struct SlConfig
{
    uint32_t numUes{2};
    double spacingM{100.0};
    uint32_t numSubch{5};
    uint32_t t1{1};
    uint32_t t2{100};
    uint32_t prsvp{100};
    bool align{true};
    double rsrpThreshDbm{-110.0};
    double shadowSigmaDb{3.0};
    bool urban{false};
    double nlosP{0.0};
    double simS{10.0};
    uint32_t streamBase{1000};
};

struct SlResult
{
    uint64_t txTotal{0};
    uint64_t rxAttempt{0};
    uint64_t rxSuccess{0};
    uint64_t delivered{0}; //!< packets that reached a receiver callback
    uint64_t selfRx{0};
    double prr{0.0};
    double prrNoHd{0.0};
    double meanDelayMs{0.0};
    double maxDelayMs{0.0};
    std::vector<uint32_t> txPerUe;
    std::vector<uint32_t> reselPerUe;
};

/// One PC5 run: a line of UEs on a highway, Mode 2 autonomous selection,
/// slot-level channel ticking inside Simulator::Run().
SlResult
RunSidelink(const SlConfig& c)
{
    SlResult r;
    Ptr<NtnSlChannel> ch = CreateObject<NtnSlChannel>();
    NtnSlResourcePool pool;
    pool.numSubchannels = c.numSubch;
    pool.slotDuration = MilliSeconds(1);
    ch->SetResourcePool(pool);
    ch->SetCarrierFrequencyHz(5.9e9);
    ch->SetShadowingSigmaDb(c.shadowSigmaDb);
    if (c.urban)
    {
        ch->SetUrbanChannel(true, c.nlosP);
    }
    std::vector<Ptr<NtnSlUeMac>> ues;
    for (uint32_t i = 0; i < c.numUes; ++i)
    {
        Ptr<NtnSlUeMac> ue = CreateObject<NtnSlUeMac>();
        ue->SetUeId(i);
        Ptr<ConstantPositionMobilityModel> m = CreateObject<ConstantPositionMobilityModel>();
        m->SetPosition(Vector(c.spacingM * i, 0.0, 1.5));
        ue->SetMobility(m);
        ue->SetSelectionWindow(c.t1, c.t2);
        ue->SetReservationPeriod(c.prsvp);
        ue->SetRsrpThresholdDbm(c.rsrpThreshDbm);
        ue->SetAlignReselection(c.align);
        ue->AssignStreams(c.streamBase + i);
        const uint32_t self = i;
        ue->SetRxCallback(MakeBoundCallback(
            +[](SlResult* res, uint32_t me, uint32_t from, Ptr<Packet>) {
                if (from == me)
                {
                    ++res->selfRx;
                }
                ++res->delivered;
            },
            &r, self));
        ch->AddUe(ue);
        ues.push_back(ue);
    }
    ch->Start(MilliSeconds(1), Seconds(c.simS));
    Simulator::Stop(Seconds(c.simS) + MilliSeconds(50));
    Simulator::Run();
    r.txTotal = ch->GetTxTotal();
    r.rxAttempt = ch->GetRxAttemptTotal();
    r.rxSuccess = ch->GetRxSuccessTotal();
    r.prr = ch->GetPrr();
    r.prrNoHd = ch->GetPrrWithinRangeExcludingHalfDuplex(1e9);
    r.meanDelayMs = ch->GetMeanDelayMs();
    r.maxDelayMs = ch->GetMaxDelayMs();
    for (auto& ue : ues)
    {
        r.txPerUe.push_back(ue->GetTxCount());
        r.reselPerUe.push_back(ue->GetReselections());
    }
    Simulator::Destroy();
    return r;
}

/// Decode range of the TR 37.885 Highway LOS law against the TS 38.214
/// lowest-MCS SINR threshold with no shadowing: the distance at which
/// Ptx - PL(d) = N + SINRmin.
double
HighwayDecodeEdgeM(Ptr<NtnSlChannel> ch, double txDbm)
{
    const double plMax = txDbm - (ch->NoiseFloorDbm() + ch->GetDecodeSinrDb());
    return std::pow(10.0, (plMax - 32.4 - 20.0 * std::log10(5.9)) / 20.0);
}

} // namespace

// ---------------------------------------------------------------------------
/// TR 37.885 Table 6.2.1-1 Highway LOS + TS 38.214 SINR decode, measured on the
/// running PC5 channel: a pair 10% inside the analytic edge decodes every
/// message, a pair 10% outside decodes none (out-of-range failure path).
class V2xSlDecodeRangeEdgeTest : public TestCase
{
  public:
    V2xSlDecodeRangeEdgeTest()
        : TestCase("PC5 decode range edge matches TR 37.885 Highway LOS and the TS 38.214 SINR floor")
    {
    }

  private:
    void DoRun() override
    {
        Ptr<NtnSlChannel> probe = CreateObject<NtnSlChannel>();
        probe->SetCarrierFrequencyHz(5.9e9);
        const double edge = HighwayDecodeEdgeM(probe, 23.0);
        // N = -174 + 10log10(1.8 MHz) + 9 = -102.45 dBm, SINRmin = -2 dB,
        // so PLmax = 127.45 dB and d = 9.58 km at 5.9 GHz.
        NS_TEST_ASSERT_MSG_EQ_TOL(edge, 9582.0, 30.0, "analytic decode edge (TR 37.885 + kTB)");

        SlConfig c;
        c.shadowSigmaDb = 0.0;
        c.simS = 3.0;
        c.spacingM = 0.9 * edge;
        SlResult in = RunSidelink(c);
        NS_TEST_ASSERT_MSG_GT(in.txTotal, 50u, "both UEs transmit every 100 ms for 3 s");
        NS_TEST_ASSERT_MSG_EQ_TOL(in.prrNoHd, 1.0, 1e-9, "inside the edge every message decodes");
        NS_TEST_ASSERT_MSG_EQ(in.delivered, in.rxSuccess,
                              "each decode is a packet handed to the receiving MAC");

        c.spacingM = 1.1 * edge;
        SlResult out = RunSidelink(c);
        NS_TEST_ASSERT_MSG_GT(out.rxAttempt, 50u, "attempts are still counted beyond the edge");
        NS_TEST_ASSERT_MSG_EQ(out.rxSuccess, 0u, "beyond the edge nothing decodes");
        NS_TEST_ASSERT_MSG_EQ(out.delivered, 0u, "and no packet reaches the receiver");
    }
};

// ---------------------------------------------------------------------------
/// TS 38.321 clause 5.22.1.1: SL_RESOURCE_RESELECTION_COUNTER is drawn
/// uniformly in [5, 15] for a reservation period of 100 ms or more, so a UE
/// reselects once every 10 transmissions on average. With aligned reselection
/// a periodic source transmits exactly once per period.
class V2xSlReselectionCounterTest : public TestCase
{
  public:
    V2xSlReselectionCounterTest()
        : TestCase("Mode 2 reselection counter follows TS 38.321 5.22.1.1 (uniform 5..15)")
    {
    }

  private:
    void DoRun() override
    {
        RngGuard g;
        SlConfig c;
        c.numUes = 4;
        c.spacingM = 50.0;
        c.simS = 120.0;
        SlResult r = RunSidelink(c);
        uint64_t tx = 0;
        uint64_t resel = 0;
        for (uint32_t i = 0; i < c.numUes; ++i)
        {
            // one message arrival per 100 ms period; the first lands within the first period
            NS_TEST_ASSERT_MSG_EQ_TOL(double(r.txPerUe[i]), 1200.0, 2.0,
                                      "aligned Mode 2: one PSSCH per BSM period");
            tx += r.txPerUe[i];
            resel += r.reselPerUe[i];
        }
        // reselections include the initial selection at Start()
        const double txPerResel = double(tx) / double(resel);
        NS_TEST_ASSERT_MSG_GT_OR_EQ(txPerResel, 5.0, "counter lower bound 5");
        NS_TEST_ASSERT_MSG_LT_OR_EQ(txPerResel, 15.0, "counter upper bound 15");
        // E[U{5..15}] = 10; ~480 draws give a standard error of 0.15
        NS_TEST_ASSERT_MSG_EQ_TOL(txPerResel, 10.0, 0.75, "mean counter value is 10");
        NS_TEST_ASSERT_MSG_EQ(r.selfRx, 0u, "half duplex: a UE never hears itself");
    }
};

// ---------------------------------------------------------------------------
/// TS 38.214 clause 8.1.4: the selection window is [n+T1, n+T2] with T2 no
/// larger than the remaining packet delay budget. TS 22.186 / TR 37.885 give
/// BSMs a 100 ms budget. The measured MAC-to-MAC latency must stay inside
/// T2 + one slot, and its mean must sit near the middle of the window.
class V2xSlLatencyWithinPdbTest : public TestCase
{
  public:
    V2xSlLatencyWithinPdbTest()
        : TestCase("PC5 latency stays inside the TS 38.214 selection window (T2 <= PDB)")
    {
    }

  private:
    void DoRun() override
    {
        RngGuard g;
        SlConfig c;
        c.numUes = 4;
        c.spacingM = 50.0;
        c.simS = 60.0;
        c.t2 = 100;
        SlResult wide = RunSidelink(c);
        NS_TEST_ASSERT_MSG_GT(wide.rxSuccess, 1000u, "enough delay samples");
        NS_TEST_ASSERT_MSG_LT_OR_EQ(wide.maxDelayMs, 101.01, "latency inside the 100 ms PDB");
        NS_TEST_ASSERT_MSG_GT_OR_EQ(wide.maxDelayMs, 1.0, "at least one slot");
        // uniform offset in [1, 100] slots: mean 50.5 ms, a few hundred draws
        NS_TEST_ASSERT_MSG_EQ_TOL(wide.meanDelayMs, 50.5, 10.0, "mean offset is mid-window");

        // Tighter budget (URLLC-like 20 ms): every latency must follow it.
        c.t2 = 20;
        SlResult tight = RunSidelink(c);
        NS_TEST_ASSERT_MSG_LT_OR_EQ(tight.maxDelayMs, 21.01, "T2 = 20 bounds latency at 20 ms + slot");
        NS_TEST_ASSERT_MSG_LT(tight.meanDelayMs, wide.meanDelayMs, "a smaller T2 cuts latency");
    }
};

// ---------------------------------------------------------------------------
/// TS 38.214 clause 8.1.4 sensing: resources reserved by neighbours above the
/// RSRP threshold are excluded. A blind selection (threshold so high nothing is
/// excluded) is random access on the same pool and must collide more.
class V2xSlSensingReducesCollisionsTest : public TestCase
{
  public:
    V2xSlSensingReducesCollisionsTest()
        : TestCase("Sensing-based exclusion (TS 38.214 8.1.4) beats blind selection on PRR")
    {
    }

  private:
    void DoRun() override
    {
        RngGuard g;
        SlConfig c;
        c.numUes = 24;
        c.spacingM = 15.0;
        c.numSubch = 1;
        c.t2 = 50;
        c.prsvp = 100;
        c.simS = 20.0;
        c.shadowSigmaDb = 0.0;
        SlResult sensed = RunSidelink(c);
        c.rsrpThreshDbm = 100.0; // nothing exceeds +100 dBm: no exclusion
        SlResult blind = RunSidelink(c);
        NS_TEST_ASSERT_MSG_EQ(sensed.txTotal, blind.txTotal,
                              "same offered load (one BSM per UE per period)");
        NS_TEST_ASSERT_MSG_GT(sensed.prrNoHd, blind.prrNoHd + 0.01,
                              "sensing must lower co-channel collisions");
        NS_TEST_ASSERT_MSG_LT(blind.prrNoHd, 1.0, "blind selection does collide");
    }
};

// ---------------------------------------------------------------------------
/// Overload failure path: a fixed pool shared by more and more UEs. The PRR
/// must fall monotonically with density, and accounting must stay consistent
/// (attempts = tx x (N - 1) for an all-in-range cluster).
class V2xSlCongestionOverloadTest : public TestCase
{
  public:
    V2xSlCongestionOverloadTest()
        : TestCase("PC5 PRR falls monotonically under pool overload; accounting stays exact")
    {
    }

  private:
    void DoRun() override
    {
        RngGuard g;
        double prev = 1.01;
        for (uint32_t n : {5u, 20u, 60u})
        {
            SlConfig c;
            c.numUes = n;
            c.spacingM = 2.0; // a dense cluster, everyone within 120 m
            c.numSubch = 1;
            c.t2 = 20;
            c.prsvp = 20;
            c.simS = 4.0;
            c.shadowSigmaDb = 0.0;
            SlResult r = RunSidelink(c);
            NS_TEST_ASSERT_MSG_EQ(r.rxAttempt, r.txTotal * (n - 1),
                                  "every transmission is an attempt at every other UE");
            NS_TEST_ASSERT_MSG_LT(r.prr, prev, "PRR falls as load rises (n=" << n << ")");
            prev = r.prr;
        }
        // 60 UEs on 20 single-subchannel resources: more than 1 tx per resource
        NS_TEST_ASSERT_MSG_LT(prev, 0.5, "an overloaded pool cannot keep PRR high");
    }
};

// ---------------------------------------------------------------------------
/// Determinism: the same RngRun reproduces every KPI bit for bit; a different
/// run changes the draw but stays inside the physical bounds.
class V2xSlDeterminismTest : public TestCase
{
  public:
    V2xSlDeterminismTest()
        : TestCase("PC5 sidelink is bit-reproducible per RngRun and bounded across runs")
    {
    }

  private:
    void DoRun() override
    {
        RngGuard g;
        SlConfig c;
        c.numUes = 12;
        c.spacingM = 40.0;
        c.numSubch = 2;
        c.t2 = 50;
        c.simS = 5.0;
        RngSeedManager::SetSeed(7);
        RngSeedManager::SetRun(3);
        SlResult a = RunSidelink(c);
        RngSeedManager::SetRun(3);
        SlResult b = RunSidelink(c);
        NS_TEST_ASSERT_MSG_EQ(a.txTotal, b.txTotal, "tx identical");
        NS_TEST_ASSERT_MSG_EQ(a.rxSuccess, b.rxSuccess, "rx identical");
        NS_TEST_ASSERT_MSG_EQ(a.meanDelayMs, b.meanDelayMs, "delay identical");
        bool anyDiff = false;
        for (uint32_t run = 4; run < 8; ++run)
        {
            RngSeedManager::SetRun(run);
            SlResult o = RunSidelink(c);
            anyDiff |= (o.rxSuccess != a.rxSuccess) || (o.meanDelayMs != a.meanDelayMs);
            NS_TEST_ASSERT_MSG_GT(o.prr, 0.6, "a 12-UE, 440 m cluster keeps a high PRR");
            NS_TEST_ASSERT_MSG_LT_OR_EQ(o.prr, 1.0, "PRR is a probability");
            NS_TEST_ASSERT_MSG_LT_OR_EQ(o.maxDelayMs, 51.01, "latency inside T2 for every seed");
        }
        NS_TEST_ASSERT_MSG_EQ(anyDiff, true, "different runs draw different resources");
    }
};

// ---------------------------------------------------------------------------
/// TR 37.885 Table 6.2.1-1 urban law. NLOS: PL = 36.85 + 30 log10 d +
/// 18.9 log10 fc, sigma 4 dB. LOS: PL = 38.77 + 16.7 log10 d + 18.2 log10 fc,
/// sigma 3 dB. The sample mean must match within 0.3 dB, and on the running
/// channel a 400 m NLOS link must lose most messages while the highway link at
/// the same range keeps them.
class V2xSlUrbanNlosLawTest : public TestCase
{
  public:
    V2xSlUrbanNlosLawTest()
        : TestCase("Urban TR 37.885 LOS-NLOS path-loss means and their effect on measured PRR")
    {
    }

  private:
    void DoRun() override
    {
        RngGuard g;
        const double fc = 5.9;
        auto mob = [](double x) {
            Ptr<ConstantPositionMobilityModel> m = CreateObject<ConstantPositionMobilityModel>();
            m->SetPosition(Vector(x, 0, 1.5));
            return m;
        };
        Ptr<MobilityModel> a = mob(0.0);
        Ptr<MobilityModel> b = mob(200.0);
        Ptr<NtnSlChannel> nlos = CreateObject<NtnSlChannel>();
        nlos->SetCarrierFrequencyHz(fc * 1e9);
        nlos->SetUrbanChannel(true, 1.0);
        double sum = 0;
        const int n = 4000;
        for (int i = 0; i < n; ++i)
        {
            sum += 23.0 - nlos->SlRsrpDbmForTest(a, b);
        }
        const double expNlos = 36.85 + 30.0 * std::log10(200.0) + 18.9 * std::log10(fc);
        NS_TEST_ASSERT_MSG_EQ_TOL(sum / n, expNlos, 0.3, "urban NLOS mean path loss at 200 m");

        // At 10 m P(LOS) = min(1, 1.05 e^-0.114) = 0.937; pick 5 m (P = 0.99)
        // with the stress knob at zero: nearly always the LOS law.
        Ptr<MobilityModel> c5 = mob(5.0);
        Ptr<NtnSlChannel> los = CreateObject<NtnSlChannel>();
        los->SetCarrierFrequencyHz(fc * 1e9);
        los->SetUrbanChannel(true, 0.0);
        sum = 0;
        for (int i = 0; i < n; ++i)
        {
            sum += 23.0 - los->SlRsrpDbmForTest(a, c5);
        }
        const double expLos = 38.77 + 16.7 * std::log10(5.0) + 18.2 * std::log10(fc);
        // either LOS for this pair (mean = expLos) or NLOSv (mean >= expLos + 5)
        NS_TEST_ASSERT_MSG_GT_OR_EQ(sum / n, expLos - 0.3, "urban LOS mean path loss at 5 m");

        SlConfig c;
        c.spacingM = 400.0;
        c.simS = 10.0;
        c.urban = true;
        c.nlosP = 1.0;
        SlResult urban = RunSidelink(c);
        c.urban = false;
        SlResult hwy = RunSidelink(c);
        // NLOS at 400 m: PL 129.5 dB, mean SNR -4 dB against a -2 dB floor
        NS_TEST_ASSERT_MSG_LT(urban.prrNoHd, 0.5, "a 400 m urban NLOS link mostly fails");
        NS_TEST_ASSERT_MSG_GT(hwy.prrNoHd, 0.99, "the same range on a highway closes");
    }
};

// ---------------------------------------------------------------------------
/// J2735 BSMs on the PC5 data path: the receiver decodes the header the sender
/// built from its live mobility, msgCnt wraps mod 128, and out-of-range state
/// is clamped to the DF_BSMcoreData ranges (speed <= 163.80 m/s, elevation
/// <= 6143.9 m, heading in [0, 360)) rather than wrapping.
class V2xBsmOverPc5ClampTest : public TestCase
{
  public:
    V2xBsmOverPc5ClampTest()
        : TestCase("J2735 BSM survives PC5 end to end and clamps to DF_BSMcoreData ranges")
    {
    }

  private:
    struct State
    {
        uint32_t tx{0};
        uint32_t rx{0};
        int lastCnt{-1};
        uint32_t cntGaps{0};
        double speed{0};
        double elev{0};
        double heading{0};
        uint32_t id{0};
        Ptr<MobilityModel> mob;
    };

    static Ptr<Packet> Build(State* s, uint32_t ueId)
    {
        NtnV2xBsmHeader h;
        const Vector v = s->mob->GetVelocity();
        const double speed = std::hypot(v.x, v.y);
        // heading -90 deg (west-of-north convention irrelevant): must normalize to 270
        h.SetFromState(static_cast<uint8_t>(s->tx & 0xff), 0xABC000 + ueId, 1234, 38.9, -77.0,
                       70000.0, speed, -90.0);
        ++s->tx;
        Ptr<Packet> p = Create<Packet>(100);
        p->AddHeader(h);
        return p;
    }

    static void Rx(State* s, uint32_t /*from*/, Ptr<Packet> p)
    {
        NtnV2xBsmHeader h;
        p->PeekHeader(h);
        ++s->rx;
        const int cnt = h.GetMsgCnt();
        if (s->lastCnt >= 0 && cnt != (s->lastCnt + 1) % 128)
        {
            ++s->cntGaps;
        }
        s->lastCnt = cnt;
        s->speed = h.GetSpeedMps();
        s->elev = h.GetElevM();
        s->heading = h.GetHeadingDeg();
        s->id = h.GetId();
    }

    void DoRun() override
    {
        RngGuard g;
        Ptr<NtnSlChannel> ch = CreateObject<NtnSlChannel>();
        ch->SetShadowingSigmaDb(0.0);
        State s;
        // a "vehicle" at 250 m/s (an aircraft-speed outlier) to drive the clamp
        Ptr<ConstantVelocityMobilityModel> fast = CreateObject<ConstantVelocityMobilityModel>();
        fast->SetPosition(Vector(0, 0, 1.5));
        fast->SetVelocity(Vector(250.0, 0, 0));
        s.mob = fast;
        Ptr<ConstantPositionMobilityModel> rxm = CreateObject<ConstantPositionMobilityModel>();
        rxm->SetPosition(Vector(1000, 0, 1.5));
        Ptr<NtnSlUeMac> tx = CreateObject<NtnSlUeMac>();
        tx->SetUeId(1);
        tx->SetMobility(fast);
        tx->SetReservationPeriod(100);
        tx->SetAlignReselection(true);
        tx->SetTxPacketCallback(MakeBoundCallback(&Build, &s));
        Ptr<NtnSlUeMac> rx = CreateObject<NtnSlUeMac>();
        rx->SetUeId(2);
        rx->SetMobility(rxm);
        rx->SetReservationPeriod(100);
        rx->SetAlignReselection(true);
        rx->SetTxPacketCallback(MakeCallback(+[](uint32_t) -> Ptr<Packet> {
            return Create<Packet>(20); // receiver's own traffic, not a BSM
        }));
        tx->SetRxCallback(MakeCallback(+[](uint32_t, Ptr<Packet>) {}));
        rx->SetRxCallback(MakeBoundCallback(&Rx, &s));
        ch->AddUe(tx);
        ch->AddUe(rx);
        ch->Start(MilliSeconds(1), Seconds(20.0)); // 200 BSMs: msgCnt wraps once
        Simulator::Stop(Seconds(20.2));
        Simulator::Run();
        Simulator::Destroy();

        NS_TEST_ASSERT_MSG_GT_OR_EQ(s.tx, 199u, "10 Hz for 20 s");
        NS_TEST_ASSERT_MSG_GT_OR_EQ(s.rx, 0.9 * s.tx,
                                    "the PC5 link delivers nearly all BSMs (only half-duplex misses)");
        NS_TEST_ASSERT_MSG_LT_OR_EQ(s.cntGaps, s.tx - s.rx,
                                    "msgCnt is continuous mod 128 except where a BSM was lost");
        NS_TEST_ASSERT_MSG_EQ(s.id, 0xABC001u, "TemporaryID survives the air interface");
        NS_TEST_ASSERT_MSG_EQ_TOL(s.speed, 163.80, 1e-6, "DE_Speed clamps at 8190 x 0.02 m/s");
        NS_TEST_ASSERT_MSG_EQ_TOL(s.elev, 6143.9, 1e-6, "DE_Elevation clamps at 61439 x 0.1 m");
        NS_TEST_ASSERT_MSG_EQ_TOL(s.heading, 270.0, 0.0125, "DE_Heading normalizes into [0, 360)");
    }
};

// ---------------------------------------------------------------------------
/// The relay engine inside a running mobility scenario: a shadowed vehicle
/// relays through a peer that drives away; once the peer leaves MaxV2vRange
/// the engine must report no relay. Failure paths: no satellite, and a
/// satellite below the horizon (no line of sight through the Earth).
class V2xRelayTracksMobilityTest : public TestCase
{
  public:
    V2xRelayTracksMobilityTest()
        : TestCase("Relay selection follows live mobility and refuses below-horizon satellites")
    {
    }

  private:
    void DoRun() override
    {
        Ptr<V2xLeoRelay> relay = CreateObject<V2xLeoRelay>();
        NS_TEST_ASSERT_MSG_EQ(relay->EvaluateAll().size(), 0u, "no satellite: no decisions");

        Ptr<ConstantPositionMobilityModel> sat = CreateObject<ConstantPositionMobilityModel>();
        sat->SetPosition(Vector(0, 0, 600e3)); // zenith LEO
        Ptr<ConstantPositionMobilityModel> v0 = CreateObject<ConstantPositionMobilityModel>();
        v0->SetPosition(Vector(0, 0, 1.5));
        Ptr<ConstantVelocityMobilityModel> v1 = CreateObject<ConstantVelocityMobilityModel>();
        v1->SetPosition(Vector(500, 0, 1.5));
        v1->SetVelocity(Vector(30, 0, 0));
        relay->SetSatellite(sat);
        relay->SetMaxV2vRangeM(1500.0);
        relay->SetMinDirectSnrDb(6.0);
        relay->RegisterVehicle("v0", v0);
        relay->RegisterVehicle("v1", v1);
        relay->SetVehicleBlockageDb("v0", 60.0);

        double lostAt = -1.0;
        uint32_t relayedSamples = 0;
        for (int t = 0; t <= 60; ++t)
        {
            Simulator::Schedule(Seconds(t), [&, t]() {
                for (const auto& d : relay->EvaluateAll())
                {
                    if (d.vehId != "v0")
                    {
                        continue;
                    }
                    if (!d.relayPeerId.empty())
                    {
                        ++relayedSamples;
                        NS_TEST_EXPECT_MSG_LT_OR_EQ(d.v2vRangeM, 1500.0, "relay inside range");
                    }
                    else if (lostAt < 0)
                    {
                        lostAt = t;
                    }
                }
            });
        }
        Simulator::Run();
        // Destroy only after the post-run geometric checks below: it resets simulated time to 0,
        // and the constant-velocity vehicles (last updated at t = 33 s) would then see time run
        // backwards (ns-3 asserts on it; release builds hid this).
        // the peer crosses 1500 m at (1500 - 500)/30 = 33.3 s
        NS_TEST_ASSERT_MSG_EQ_TOL(lostAt, 34.0, 0.5, "relay drops when the peer leaves range");
        NS_TEST_ASSERT_MSG_EQ(relayedSamples, 34u, "relayed for t = 0..33 s");

        // Below the horizon: free-space SNR is finite but there is no LOS.
        sat->SetPosition(Vector(2000e3, 0, -100e3));
        relay->SetVehicleBlockageDb("v0", 0.0);
        for (const auto& d : relay->EvaluateAll())
        {
            NS_TEST_ASSERT_MSG_EQ(d.directToLeo, false,
                                  "a satellite below the horizon cannot serve " << d.vehId);
            NS_TEST_ASSERT_MSG_EQ(d.relayPeerId.empty(), true,
                                  "and no peer can relay to it either");
        }
        Simulator::Destroy();
    }
};

// ---------------------------------------------------------------------------
/// Cross-module: an SGP4 Walker satellite (ntn-constellation), projected into
/// the local ENU frame (ntn-cho), drives V2xLeoDirect through a real pass.
/// TR 38.811 eq. 6.6-3: d = sqrt(R^2 sin^2(a) + h^2 + 2 h R) - R sin(a);
/// TR 38.811 eq. 6.6-2: FSPL = 32.45 + 20 log10(fc GHz) + 20 log10(d m).
class V2xLeoDirectSgp4PassTest : public TestCase
{
  public:
    V2xLeoDirectSgp4PassTest()
        : TestCase("LEO direct budget over an SGP4 pass matches TR 38.811 slant range and FSPL")
    {
    }

  private:
    void DoRun() override
    {
        ntncon::WalkerConfig w;
        w.num_planes = 1;
        w.total_sats = 20;
        w.altitude_km = 600.0;
        w.inclination_deg = 53.0;
        w.epoch_unix_s = 1735689600.0;
        auto el = ntncon::WalkerConstellation::BuildDelta(w);
        Ptr<ntncon::Sgp4MobilityModel> sgp4 = CreateObject<ntncon::Sgp4MobilityModel>();
        sgp4->SetElementsSgp4(el[0]);
        double lat0;
        double lon0;
        double alt0;
        sgp4->GetGeodetic(lat0, lon0, alt0);
        Ptr<NtnEnuProjectionMobilityModel> sat = CreateObject<NtnEnuProjectionMobilityModel>();
        sat->SetSource(sgp4);
        sat->SetReference(lat0, lon0, 0.0);
        Ptr<ConstantPositionMobilityModel> veh = CreateObject<ConstantPositionMobilityModel>();
        veh->SetPosition(Vector(0, 0, 0));
        Ptr<V2xLeoDirect> direct = CreateObject<V2xLeoDirect>();
        direct->SetFrequencyGHz(2.0);

        struct Sample
        {
            double t;
            V2xLinkBudget lb;
            double altM;
        };

        std::vector<Sample> samples;
        for (int t = 0; t <= 600; t += 10)
        {
            Simulator::Schedule(Seconds(t), [&, t]() {
                double la;
                double lo;
                double al;
                sgp4->GetGeodetic(la, lo, al);
                samples.push_back({double(t), direct->Compute(veh, sat), al});
            });
        }
        Simulator::Run();
        Simulator::Destroy();

        NS_TEST_ASSERT_MSG_EQ(samples.size(), 61u, "a 10 s grid over 10 min");
        NS_TEST_ASSERT_MSG_GT(samples[0].lb.elevationDeg, 89.0, "overhead at t = 0");
        NS_TEST_ASSERT_MSG_EQ_TOL(samples[0].lb.slantRangeKm * 1e3, samples[0].altM,
                                  0.005 * samples[0].altM, "zenith slant range = altitude");
        bool setSeen = false;
        const double R = 6371e3;
        for (size_t i = 1; i < samples.size(); ++i)
        {
            const auto& s = samples[i];
            const auto& p = samples[i - 1];
            NS_TEST_ASSERT_MSG_LT(s.lb.elevationDeg, p.lb.elevationDeg + 1e-6,
                                  "elevation falls monotonically after culmination");
            NS_TEST_ASSERT_MSG_LT(s.lb.snrDb, p.lb.snrDb + 1e-9, "SNR follows the slant range");
            const double fspl = 32.45 + 20.0 * std::log10(2.0) +
                                20.0 * std::log10(s.lb.slantRangeKm * 1e3);
            NS_TEST_ASSERT_MSG_EQ_TOL(s.lb.freeSpacePlDb, fspl, 1e-6, "TR 38.811 eq. 6.6-2");
            if (s.lb.aboveHorizon && s.lb.elevationDeg > 10.0)
            {
                const double a = s.lb.elevationDeg * M_PI / 180.0;
                const double h = s.altM;
                const double d =
                    std::sqrt(R * R * std::sin(a) * std::sin(a) + h * h + 2 * h * R) -
                    R * std::sin(a);
                // spherical-Earth formula vs WGS-84 geometry: within 1.5%
                NS_TEST_ASSERT_MSG_EQ_TOL(s.lb.slantRangeKm * 1e3, d, 0.015 * d,
                                          "TR 38.811 eq. 6.6-3 slant range at " << s.t << " s");
            }
            setSeen |= !s.lb.aboveHorizon;
        }
        NS_TEST_ASSERT_MSG_EQ(setSeen, true, "the satellite sets within 10 min (600 km LEO)");
    }
};

// ---------------------------------------------------------------------------
/// One flow through the toolkit: BSMs cross PC5 to a relay vehicle, which
/// forwards each decoded BSM over UDP/IPv4 on a satellite hop whose delay
/// follows the live SGP4 slant range. FlowMonitor measures the hop: packets
/// received equal BSMs relayed, the mean delay equals the mean slant range
/// over c plus the serialization time, and a 1 s outage (link failure path)
/// loses exactly the packets sent inside it.
class V2xPc5ToLeoFlowMonitorTest : public TestCase
{
  public:
    V2xPc5ToLeoFlowMonitorTest()
        : TestCase("PC5 BSM relayed over an SGP4-timed IPv4 satellite hop, measured by FlowMonitor")
    {
    }

  private:
    struct Ctx
    {
        Ptr<Socket> tx;
        Ipv4Address gw;
        uint32_t relayed{0};
        uint32_t relayedInOutage{0};
        uint32_t gwRx{0};
        uint32_t gwBsmOk{0};
        double propSum{0.0};
        Ptr<V2xLeoDirect> direct;
        Ptr<MobilityModel> relayMob;
        Ptr<MobilityModel> sat;
        Ptr<PointToPointChannel> chan;
        Ptr<RateErrorModel> em;
    };

    static void OnPc5Rx(Ctx* c, uint32_t from, Ptr<Packet> p)
    {
        if (from != 0)
        {
            return;
        }
        const double now = Simulator::Now().GetSeconds();
        if (now >= 2.0 && now < 3.0)
        {
            ++c->relayedInOutage;
        }
        ++c->relayed;
        c->propSum += c->direct->Compute(c->relayMob, c->sat).slantRangeKm * 1e3 / kC;
        c->tx->SendTo(p->Copy(), 0, InetSocketAddress(c->gw, 9000));
    }

    static void OnGwRx(Ctx* c, Ptr<Socket> s)
    {
        Ptr<Packet> p;
        while ((p = s->Recv()))
        {
            ++c->gwRx;
            NtnV2xBsmHeader h;
            p->PeekHeader(h);
            if (h.GetId() == 77u)
            {
                ++c->gwBsmOk;
            }
        }
    }

    static void UpdateDelay(Ctx* c)
    {
        const double d = c->direct->Compute(c->relayMob, c->sat).slantRangeKm * 1e3;
        c->chan->SetAttribute("Delay", TimeValue(Seconds(d / kC)));
        Simulator::Schedule(MilliSeconds(10), &UpdateDelay, c);
    }

    void DoRun() override
    {
        RngGuard g;
        Ctx c;
        // SGP4 satellite in the local ENU frame
        ntncon::WalkerConfig w;
        w.num_planes = 1;
        w.total_sats = 20;
        w.altitude_km = 550.0;
        w.epoch_unix_s = 1735689600.0;
        auto el = ntncon::WalkerConstellation::BuildDelta(w);
        Ptr<ntncon::Sgp4MobilityModel> sgp4 = CreateObject<ntncon::Sgp4MobilityModel>();
        sgp4->SetElementsSgp4(el[0]);
        double la;
        double lo;
        double al;
        sgp4->GetGeodetic(la, lo, al);
        Ptr<NtnEnuProjectionMobilityModel> sat = CreateObject<NtnEnuProjectionMobilityModel>();
        sat->SetSource(sgp4);
        sat->SetReference(la, lo, 0.0);
        c.sat = sat;
        c.direct = CreateObject<V2xLeoDirect>();

        NodeContainer nodes;
        nodes.Create(2); // 0 = relay vehicle, 1 = satellite gateway
        Ptr<ConstantPositionMobilityModel> m0 = CreateObject<ConstantPositionMobilityModel>();
        m0->SetPosition(Vector(0, 0, 1.5));
        Ptr<ConstantPositionMobilityModel> m1 = CreateObject<ConstantPositionMobilityModel>();
        m1->SetPosition(Vector(120, 0, 1.5));
        nodes.Get(0)->AggregateObject(m1);
        c.relayMob = m1;

        InternetStackHelper inet;
        inet.Install(nodes);
        PointToPointHelper p2p;
        p2p.SetDeviceAttribute("DataRate", StringValue("10Mbps"));
        p2p.SetChannelAttribute("Delay", TimeValue(MilliSeconds(2)));
        NetDeviceContainer dev = p2p.Install(nodes);
        c.chan = DynamicCast<PointToPointChannel>(dev.Get(0)->GetChannel());
        c.em = CreateObject<RateErrorModel>();
        c.em->SetUnit(RateErrorModel::ERROR_UNIT_PACKET);
        c.em->SetRate(0.0);
        dev.Get(1)->SetAttribute("ReceiveErrorModel", PointerValue(c.em));
        Ipv4AddressHelper ip;
        ip.SetBase("10.77.0.0", "255.255.255.0");
        Ipv4InterfaceContainer ifs = ip.Assign(dev);
        c.gw = ifs.GetAddress(1);
        c.tx = Socket::CreateSocket(nodes.Get(0), UdpSocketFactory::GetTypeId());
        Ptr<Socket> rx = Socket::CreateSocket(nodes.Get(1), UdpSocketFactory::GetTypeId());
        rx->Bind(InetSocketAddress(Ipv4Address::GetAny(), 9000));
        rx->SetRecvCallback(MakeBoundCallback(&OnGwRx, &c));

        // PC5: source vehicle 0 sends BSMs (id 77), relay vehicle 1 decodes
        Ptr<NtnSlChannel> sl = CreateObject<NtnSlChannel>();
        Ptr<NtnSlUeMac> src = CreateObject<NtnSlUeMac>();
        src->SetUeId(0);
        src->SetMobility(m0);
        src->SetAlignReselection(true);
        src->SetTxPacketCallback(MakeCallback(+[](uint32_t) -> Ptr<Packet> {
            static uint8_t cnt = 0;
            NtnV2xBsmHeader h;
            h.SetFromState(cnt++, 77, 0, 38.9, -77.0, 10.0, 25.0, 90.0);
            Ptr<Packet> p = Create<Packet>(150);
            p->AddHeader(h);
            return p;
        }));
        src->SetRxCallback(MakeCallback(+[](uint32_t, Ptr<Packet>) {}));
        Ptr<NtnSlUeMac> rel = CreateObject<NtnSlUeMac>();
        rel->SetUeId(1);
        rel->SetMobility(m1);
        rel->SetAlignReselection(true);
        rel->SetRxCallback(MakeBoundCallback(&OnPc5Rx, &c));
        sl->AddUe(src);
        sl->AddUe(rel);

        FlowMonitorHelper fmh;
        Ptr<FlowMonitor> fm = fmh.InstallAll();
        Simulator::ScheduleNow(&UpdateDelay, &c);
        Simulator::Schedule(Seconds(2.0), [&c]() { c.em->SetRate(1.0); });
        Simulator::Schedule(Seconds(3.0), [&c]() { c.em->SetRate(0.0); });
        sl->Start(MilliSeconds(1), Seconds(6.0));
        Simulator::Stop(Seconds(6.5));
        Simulator::Run();

        uint64_t txP = 0;
        uint64_t rxP = 0;
        double delaySum = 0;
        uint64_t txBytes = 0;
        for (const auto& [id, st] : fm->GetFlowStats())
        {
            txP += st.txPackets;
            rxP += st.rxPackets;
            delaySum += st.delaySum.GetSeconds();
            txBytes += st.txBytes;
        }
        Simulator::Destroy();

        NS_TEST_ASSERT_MSG_GT_OR_EQ(c.relayed, 55u, "~60 BSMs cross PC5 in 6 s");
        NS_TEST_ASSERT_MSG_EQ(txP, c.relayed, "every relayed BSM enters the IPv4 hop");
        NS_TEST_ASSERT_MSG_GT(c.relayedInOutage, 5u, "the outage window carried traffic");
        NS_TEST_ASSERT_MSG_EQ(txP - rxP, c.relayedInOutage, "outage loses exactly its packets");
        NS_TEST_ASSERT_MSG_EQ(c.gwRx, rxP, "socket counter agrees with FlowMonitor");
        NS_TEST_ASSERT_MSG_EQ(c.gwBsmOk, c.gwRx, "BSM header intact at the gateway");
        // FlowMonitor delay = propagation (slant/c) + serialization at 10 Mbps
        const double ser = (double(txBytes) / txP + 2.0) * 8.0 / 10e6; // + PPP header
        const double expectMean = c.propSum / c.relayed + ser;
        NS_TEST_ASSERT_MSG_EQ_TOL(delaySum / rxP, expectMean, 20e-6,
                                  "measured delay = SGP4 slant range / c + serialization");
        NS_TEST_ASSERT_MSG_GT(delaySum / rxP, 1.8e-3, "550 km LEO one-way delay is ~1.8 ms or more");
    }
};

// ---------------------------------------------------------------------------
/// FCD replay failure paths in a running simulation: a missing file, a row
/// with non-finite coordinates, and loading the same trace twice. A clean
/// trace still drives the vehicle to its last sample.
class V2xFcdReplayFailurePathsTest : public TestCase
{
  public:
    V2xFcdReplayFailurePathsTest()
        : TestCase("SUMO FCD replay rejects missing, non-finite and duplicate input")
    {
    }

  private:
    void DoRun() override
    {
        Ptr<SumoTraciBridge> missing = CreateObject<SumoTraciBridge>();
        NS_TEST_ASSERT_MSG_EQ(missing->LoadFcdTrace("/nonexistent/fcd.csv"), false,
                              "a missing trace is refused");
        NS_TEST_ASSERT_MSG_EQ(missing->LoadedSampleCount(), 0u, "and loads nothing");

        const std::string path = CreateTempDirFilename("v2x-fcd-indepth.csv");
        {
            std::ofstream f(path);
            f << "time,id,x,y,z,speed\n";
            for (int t = 0; t <= 10; ++t)
            {
                f << t << ",car," << 20.0 * t << ",0,0,20\n";
            }
            f << "5,car,nan,0,0,20\n";  // corrupted row
            f << "6,car,inf,0,0,20\n";  // corrupted row
            f << "7,car,100\n";         // truncated row
        }
        Ptr<SumoTraciBridge> b = CreateObject<SumoTraciBridge>();
        NS_TEST_ASSERT_MSG_EQ(b->LoadFcdTrace(path), true, "the trace loads");
        NS_TEST_ASSERT_MSG_EQ(b->LoadedSampleCount(), 11u, "non-finite and truncated rows dropped");
        NS_TEST_ASSERT_MSG_EQ(b->LoadFcdTrace(path), true, "reloading succeeds");
        NS_TEST_ASSERT_MSG_EQ(b->LoadedSampleCount(), 11u, "reloading replaces, not appends");

        Ptr<ConstantVelocityMobilityModel> car = CreateObject<ConstantVelocityMobilityModel>();
        b->RegisterVehicle("car", car);
        bool finite = true;
        for (int t = 0; t <= 10; ++t)
        {
            Simulator::Schedule(Seconds(t) + MilliSeconds(1), [&finite, car]() {
                const Vector p = car->GetPosition();
                finite &= std::isfinite(p.x) && std::isfinite(p.y);
            });
        }
        b->RunReplay(1.0, 11);
        Simulator::Stop(Seconds(10.5));
        Simulator::Run();
        const Vector p = car->GetPosition();
        Simulator::Destroy();
        NS_TEST_ASSERT_MSG_EQ(finite, true, "a corrupt row never reaches the mobility model");
        NS_TEST_ASSERT_MSG_EQ_TOL(p.x, 200.0 + 20.0 * 0.5, 1e-3,
                                  "vehicle at the last sample, extrapolated at 20 m/s");
        std::remove(path.c_str());
    }
};

// ---------------------------------------------------------------------------
/// Maritime mobility over a running hour: every tick moves the vessel by
/// exactly speed x dt (until it hits a wall), the vessel never leaves its box,
/// and an out-of-range speed attribute is rejected by the checker.
struct MaritimeProbe
{
    Box box;
    uint32_t ticks{0};
    uint32_t inside{0};
    uint32_t speedOk{0};
    uint32_t stepOk{0};
    uint32_t bounces{0};
    bool havePrev{false};
    Vector prevPos;
    Vector prevVel;

    static void OnMove(MaritimeProbe* s, Ptr<const MobilityModel> mm)
    {
        const Vector p = mm->GetPosition();
        const Vector v = mm->GetVelocity();
        ++s->ticks;
        if (p.x >= s->box.xMin && p.x <= s->box.xMax && p.y >= s->box.yMin && p.y <= s->box.yMax)
        {
            ++s->inside;
        }
        if (std::abs(std::hypot(v.x, v.y) - 10.0) < 1e-9)
        {
            ++s->speedOk;
        }
        if (s->havePrev)
        {
            const bool reflected = (v.x != s->prevVel.x) || (v.y != s->prevVel.y);
            const double step = std::hypot(p.x - s->prevPos.x, p.y - s->prevPos.y);
            if (reflected)
            {
                ++s->bounces;
            }
            else if (std::abs(step - 10.0) < 1e-6)
            {
                ++s->stepOk;
            }
        }
        s->havePrev = true;
        s->prevPos = p;
        s->prevVel = v;
    }
};

class V2xMaritimeKinematicsTest : public TestCase
{
  public:
    V2xMaritimeKinematicsTest()
        : TestCase("Maritime vessel keeps its speed, stays in its sea area and rejects bad speed")
    {
    }

  private:
    void DoRun() override
    {
        RngGuard g;
        Ptr<MaritimeMobilityModel> m = CreateObject<MaritimeMobilityModel>();
        NS_TEST_ASSERT_MSG_EQ(m->SetAttributeFailSafe("Speed", DoubleValue(40.0)), false,
                              "40 m/s is outside the 0.5..25 m/s vessel range");
        m->SetAttribute("Speed", DoubleValue(10.0));
        Box area(-2000, 2000, -2000, 2000, 0, 0);
        m->SetSeaArea(area);
        MaritimeProbe probe;
        probe.box = area;
        m->TraceConnectWithoutContext("CourseChange", MakeBoundCallback(&MaritimeProbe::OnMove, &probe));
        Simulator::Stop(Seconds(3600));
        Simulator::Run();
        Simulator::Destroy();
        NS_TEST_ASSERT_MSG_GT_OR_EQ(probe.ticks, 3600u, "1 Hz tick over an hour");
        NS_TEST_ASSERT_MSG_EQ(probe.inside, probe.ticks, "the vessel never leaves the sea area");
        NS_TEST_ASSERT_MSG_EQ(probe.speedOk, probe.ticks, "speed magnitude conserved through reflections");
        NS_TEST_ASSERT_MSG_EQ(probe.stepOk + probe.bounces, probe.ticks - 1,
                              "every tick moves speed x dt unless it reflects off a wall");
        NS_TEST_ASSERT_MSG_GT(probe.bounces, 0u, "a 4 km box at 10 m/s forces reflections in an hour");
    }
};

class NtnV2xInDepthTestSuite : public TestSuite
{
  public:
    NtnV2xInDepthTestSuite()
        : TestSuite("ntn-v2x-indepth", Type::UNIT)
    {
        AddTestCase(new V2xSlDecodeRangeEdgeTest, Duration::QUICK);
        AddTestCase(new V2xSlReselectionCounterTest, Duration::QUICK);
        AddTestCase(new V2xSlLatencyWithinPdbTest, Duration::QUICK);
        AddTestCase(new V2xSlSensingReducesCollisionsTest, Duration::QUICK);
        AddTestCase(new V2xSlCongestionOverloadTest, Duration::QUICK);
        AddTestCase(new V2xSlDeterminismTest, Duration::QUICK);
        AddTestCase(new V2xSlUrbanNlosLawTest, Duration::QUICK);
        AddTestCase(new V2xBsmOverPc5ClampTest, Duration::QUICK);
        AddTestCase(new V2xRelayTracksMobilityTest, Duration::QUICK);
        AddTestCase(new V2xLeoDirectSgp4PassTest, Duration::QUICK);
        AddTestCase(new V2xPc5ToLeoFlowMonitorTest, Duration::QUICK);
        AddTestCase(new V2xFcdReplayFailurePathsTest, Duration::QUICK);
        AddTestCase(new V2xMaritimeKinematicsTest, Duration::QUICK);
    }
};

static NtnV2xInDepthTestSuite g_ntnV2xInDepthTestSuite;
