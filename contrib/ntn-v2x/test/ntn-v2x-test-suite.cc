/*
 * SPDX-License-Identifier: GPL-2.0-only
 * Copyright (c) 2026 Muhammad Uzair (ns3-ntn-toolkit, W7)
 */
#include "ns3/box.h"
#include "ns3/constant-position-mobility-model.h"
#include "ns3/constant-velocity-mobility-model.h"
#include "ns3/double.h"
#include "ns3/maritime-scenario.h"
#include "ns3/ntn-v2x-bsm-header.h"
#include "ns3/ntn-v2x-helper.h"
#include "ns3/packet.h"
#include "ns3/simulator.h"
#include "ns3/sumo-traci-bridge.h"
#include "ns3/test.h"
#include "ns3/v2x-leo-direct.h"
#include "ns3/v2x-leo-relay.h"
#include "ns3/ntn-nr-sidelink.h"
#include "ns3/constant-position-mobility-model.h"

#include <cmath>
#include <fstream>
#include <map>
#include <cstdio>

using namespace ns3;
using namespace ns3::ntnv2x;

namespace
{

class FcdTraceReplayJitterTest : public TestCase
{
  public:
    FcdTraceReplayJitterTest()
        : TestCase("Trace replay MEASURED (not injected) sync offset stays under 100 ms")
    {
    }

    void DoRun() override
    {
        const std::string trace = "/tmp/ntn-v2x-test-fcd.csv";
        const std::size_t nVeh = 10;
        const double simSec = 30.0;
        const double dt = 1.0;
        NS_TEST_ASSERT_MSG_EQ(NtnV2xHelper::WriteDeterministicTestFcdCsv(trace, nVeh,
                                                                         1000.0, simSec, dt),
                              true, "deterministic FCD fixture generation failed");

        Ptr<SumoTraciBridge> br = CreateObject<SumoTraciBridge>();
        NS_TEST_ASSERT_MSG_EQ(br->LoadFcdTrace(trace), true, "trace load failed");

        std::vector<Ptr<ConstantPositionMobilityModel>> mobs(nVeh);
        for (std::size_t i = 0; i < nVeh; ++i)
        {
            mobs[i] = CreateObject<ConstantPositionMobilityModel>();
            mobs[i]->SetPosition(Vector{0, 0, 0});
            br->RegisterVehicle("veh" + std::to_string(i), mobs[i]);
        }

        int nSteps = static_cast<int>(simSec / dt);
        for (int s = 0; s <= nSteps; ++s)
        {
            Simulator::Schedule(Seconds(s * dt), [br]() { br->Step(); });
        }
        Simulator::Stop(Seconds(simSec + 1));
        Simulator::Run();
        Simulator::Destroy();

        // The reported offset is now MEASURED from the real ns-3 scheduler vs
        // the trace timestamp (no injected term): in locked replay it is ~0,
        // comfortably under the 100 ms W7 gate.
        double maxJitterMs = br->GetMaxJitterSec() * 1000.0;
        NS_TEST_ASSERT_MSG_LT(maxJitterMs, 100.0,
                              "TraCI replay measured offset " << maxJitterMs
                              << " ms exceeded 100 ms gate");
        std::remove(trace.c_str());
    }
};

class V2xLeoDirectFreeSpaceTest : public TestCase
{
  public:
    V2xLeoDirectFreeSpaceTest()
        : TestCase("V2X-LEO direct free-space PL matches closed form within 0.1 dB")
    {
    }

    void DoRun() override
    {
        // 1000 km slant, 2 GHz: PL = 20*log10(1e6) + 20*log10(2) + 32.45
        //                          = 120 + 6.02 + 32.45 = 158.47 dB
        Vector v{0, 0, 0};
        Vector s{0, 0, 1.0e6};
        auto lb = V2xLeoDirect::ComputeStatic(v, s, /*fcGHz=*/2.0,
                                              /*eirpDbm=*/50.0,
                                              /*noiseDbm=*/-110.0);
        NS_TEST_ASSERT_MSG_LT(std::abs(lb.freeSpacePlDb - 158.47), 0.1,
                              "free-space PL out of spec: " << lb.freeSpacePlDb);
        NS_TEST_ASSERT_MSG_EQ(lb.aboveHorizon, true, "satellite must be above horizon");
        NS_TEST_ASSERT_MSG_LT(std::abs(lb.elevationDeg - 90.0), 0.01, "zenith elevation");
    }
};

class V2xLeoRelayDirectVsRelayTest : public TestCase
{
  public:
    V2xLeoRelayDirectVsRelayTest()
        : TestCase("Relay falls back to peer when direct SNR below threshold")
    {
    }

    void DoRun() override
    {
        // Use a low-altitude pseudo-satellite so vehicle separation is measurable.
        // Real LEO is too far (550 km) for ground-level meters to make a SNR diff —
        // the relay logic itself is identical regardless of magnitudes.
        Ptr<ConstantVelocityMobilityModel> sat = CreateObject<ConstantVelocityMobilityModel>();
        sat->SetPosition(Vector{0, 0, 100.0});  // 100 m AGL test fixture
        Ptr<V2xLeoRelay> relay = CreateObject<V2xLeoRelay>();
        relay->SetSatellite(sat);

        Ptr<ConstantPositionMobilityModel> a = CreateObject<ConstantPositionMobilityModel>();
        a->SetPosition(Vector{0, 0, 0});  // slant = 100 m
        Ptr<ConstantPositionMobilityModel> b = CreateObject<ConstantPositionMobilityModel>();
        b->SetPosition(Vector{300, 0, 0});  // slant ≈ 316 m, V2V range 300 m

        relay->RegisterVehicle("A", a);
        relay->RegisterVehicle("B", b);

        // V2X-8: the threshold must NOT be derived from the model's own output.
        //
        // This used to call ComputeStatic for both vehicles and set the
        // threshold to the mean of the two SNRs it got back. The expected
        // direct/relay split was therefore a consequence of whatever the model
        // returned, and held for ANY monotone SNR function, correct or not: a
        // model off by 30 dB, or with the wrong frequency term, or with the
        // path-loss exponent inverted, produced two numbers whose mean still
        // sat between them and the test still passed.
        //
        // The threshold is now an INDEPENDENT link-budget calculation from the
        // geometry, so a model that disagrees with the budget fails.
        //
        // Vehicle A is at the sub-satellite point, slant 100 m. Vehicle B is
        // 300 m away, slant sqrt(100^2 + 300^2) = 316.23 m. With EIRP 50 dBm,
        // noise -110 dBm and 2 GHz:
        //   FSPL = 20log10(d) + 20log10(f) - 147.55
        //   SNR  = EIRP - FSPL - noise
        // so the two differ by 20log10(316.23/100) = 10.0 dB, and the midpoint
        // is A's budget SNR minus 5 dB.
        constexpr double kC = 299792458.0;
        auto budgetSnrDb = [](double slantM) {
            const double fspl = 20.0 * std::log10(slantM) + 20.0 * std::log10(2.0e9) - 147.55;
            return 50.0 - fspl - (-110.0);
        };
        (void)kC;
        const double snrA = budgetSnrDb(100.0);
        const double snrB = budgetSnrDb(std::sqrt(100.0 * 100.0 + 300.0 * 300.0));
        NS_TEST_ASSERT_MSG_GT(snrA, snrB + 9.0,
                              "the two geometries must be far enough apart for a threshold "
                              "between them to be meaningful (got " << (snrA - snrB) << " dB)");
        relay->SetMinDirectSnrDb((snrA + snrB) / 2.0);

        // And the model must AGREE with that independent budget, which is the
        // assertion the old form could never make.
        auto lbA = V2xLeoDirect::ComputeStatic(a->GetPosition(), sat->GetPosition(),
                                               2.0, 50.0, -110.0);
        auto lbB = V2xLeoDirect::ComputeStatic(b->GetPosition(), sat->GetPosition(),
                                               2.0, 50.0, -110.0);
        NS_TEST_ASSERT_MSG_EQ_TOL(lbA.snrDb, snrA, 1.0,
                                  "the model's SNR at 100 m must match the free-space budget ("
                                      << snrA << " dB), not merely be larger than B's");
        NS_TEST_ASSERT_MSG_EQ_TOL(lbB.snrDb, snrB, 1.0,
                                  "and likewise at 316 m (" << snrB << " dB)");
        auto out = relay->EvaluateAll();
        bool sawDirect = false, sawRelay = false;
        for (auto& d : out)
        {
            if (d.vehId == "A")
                sawDirect = d.directToLeo;
            if (d.vehId == "B")
                sawRelay = !d.directToLeo && d.relayPeerId == "A";
        }
        NS_TEST_ASSERT_MSG_EQ(sawDirect, true, "vehicle A should be direct");
        NS_TEST_ASSERT_MSG_EQ(sawRelay, true, "vehicle B should relay via A");
    }
};

class MaritimeBouncesInBoxTest : public TestCase
{
  public:
    MaritimeBouncesInBoxTest()
        : TestCase("Maritime mobility stays inside its sea-area bounding box")
    {
    }

    void DoRun() override
    {
        Ptr<MaritimeMobilityModel> m = CreateObject<MaritimeMobilityModel>();
        Box area(-10000, 10000, -10000, 10000, 0, 0);
        m->SetSeaArea(area);

        double minX = 1e9, maxX = -1e9, minY = 1e9, maxY = -1e9;
        for (double t = 0; t <= 1800.0; t += 5.0)
        {
            Simulator::Schedule(Seconds(t), [&minX, &maxX, &minY, &maxY, m]() {
                Vector p = m->GetPosition();
                minX = std::min(minX, p.x);
                maxX = std::max(maxX, p.x);
                minY = std::min(minY, p.y);
                maxY = std::max(maxY, p.y);
            });
        }
        Simulator::Stop(Seconds(1801));
        Simulator::Run();
        Simulator::Destroy();

        NS_TEST_ASSERT_MSG_GT_OR_EQ(minX, area.xMin - 100.0, "drifted west");
        NS_TEST_ASSERT_MSG_LT_OR_EQ(maxX, area.xMax + 100.0, "drifted east");
        NS_TEST_ASSERT_MSG_GT_OR_EQ(minY, area.yMin - 100.0, "drifted south");
        NS_TEST_ASSERT_MSG_LT_OR_EQ(maxY, area.yMax + 100.0, "drifted north");
    }
};

class HundredVehicleSmokeTest : public TestCase
{
  public:
    HundredVehicleSmokeTest()
        : TestCase("100 vehicle 5-min replay completes inside test budget")
    {
    }

    void DoRun() override
    {
        const std::string trace = "/tmp/ntn-v2x-test-100veh.csv";
        const std::size_t n = 100;
        NS_TEST_ASSERT_MSG_EQ(
            NtnV2xHelper::WriteDeterministicTestFcdCsv(trace, n, 30000.0, 300.0, 1.0),
            true, "deterministic fixture generation failed");

        Ptr<SumoTraciBridge> br = CreateObject<SumoTraciBridge>();
        NS_TEST_ASSERT_MSG_EQ(br->LoadFcdTrace(trace), true, "trace load failed");

        std::vector<Ptr<ConstantPositionMobilityModel>> mobs(n);
        for (std::size_t i = 0; i < n; ++i)
        {
            mobs[i] = CreateObject<ConstantPositionMobilityModel>();
            mobs[i]->SetPosition(Vector{-1.0e9, 0, 0});
            br->RegisterVehicle("veh" + std::to_string(i), mobs[i]);
        }
        for (int s = 0; s <= 300; ++s)
        {
            Simulator::Schedule(Seconds(s), [br]() { br->Step(); });
        }
        Simulator::Stop(Seconds(301));
        Simulator::Run();
        Simulator::Destroy();
        // 100 veh × 301 ticks = 30 100 samples expected.
        NS_TEST_ASSERT_MSG_EQ(br->LoadedSampleCount(), 100u * 301u,
                              "trace sample count mismatch");
        std::remove(trace.c_str());
    }
};

/**
 * \brief The J2735 BSM header serialises its kinematic state and reads it back
 *        within each field's encoding resolution — proving the relay packets
 *        carry a real BSM, not opaque padding.
 */
class J2735BsmHeaderRoundTripTest : public TestCase
{
  public:
    J2735BsmHeaderRoundTripTest()
        : TestCase("SAE J2735 BSM header round-trips through a packet")
    {
    }

  private:
    void DoRun() override
    {
        ntnv2x::NtnV2xBsmHeader tx;
        tx.SetFromState(/*msgCnt=*/42, /*id=*/0x0A0B0C0Du, /*secMark=*/12345,
                        /*lat=*/48.137154, /*lon=*/11.576124, /*elev=*/542.3,
                        /*speed=*/27.4, /*heading=*/93.75);

        Ptr<Packet> p = Create<Packet>(0);
        p->AddHeader(tx);
        ntnv2x::NtnV2xBsmHeader rx;
        p->RemoveHeader(rx);

        NS_TEST_ASSERT_MSG_EQ(rx.GetMsgCnt(), 42, "msgCnt");
        NS_TEST_ASSERT_MSG_EQ(rx.GetId(), 0x0A0B0C0Du, "station id");
        NS_TEST_ASSERT_MSG_EQ(rx.GetSecMark(), 12345, "secMark");
        // 1/10 micro-degree resolution -> ~1e-7 deg.
        NS_TEST_ASSERT_MSG_EQ_TOL(rx.GetLatDeg(), 48.137154, 1e-6, "latitude");
        NS_TEST_ASSERT_MSG_EQ_TOL(rx.GetLonDeg(), 11.576124, 1e-6, "longitude");
        NS_TEST_ASSERT_MSG_EQ_TOL(rx.GetElevM(), 542.3, 0.05, "elevation (1 dm)");
        NS_TEST_ASSERT_MSG_EQ_TOL(rx.GetSpeedMps(), 27.4, 0.02, "speed (0.02 m/s)");
        NS_TEST_ASSERT_MSG_EQ_TOL(rx.GetHeadingDeg(), 93.75, 0.0125, "heading (0.0125 deg)");

        // V2X-8/V2X-7: a mutual-inverse check passes for ANY self-consistent
        // scale factor, so it could not see either out-of-spec encoding. These
        // assert the SAE J2735 data-element definitions instead.

        // DE_Elevation is INTEGER (-4096..61439) in units of 0.1 m: two octets.
        // The header used to write it with WriteHtonU32, making the encoded
        // message two bytes longer than the standard it is named for.
        // 1 + 4 + 2 + 4 + 4 + 2 + 2 + 2 = 21.
        NS_TEST_ASSERT_MSG_EQ(tx.GetSerializedSize(), 21u,
                              "the J2735 core fields encode to 21 octets; 23 means elevation is "
                              "still being written as four octets instead of two");

        // DE_Speed is INTEGER (0..8191) in units of 0.02 m/s with 8191 reserved
        // for "unavailable", so the largest representable speed is
        // 8190 * 0.02 = 163.80 m/s. The header used to clamp at 65534 units =
        // 1310.68 m/s, which no conforming decoder can express.
        {
            ntnv2x::NtnV2xBsmHeader fast;
            fast.SetFromState(1, 1, 0, 0.0, 0.0, 0.0, 500.0 /* m/s, far past the ceiling */, 0.0);
            NS_TEST_ASSERT_MSG_EQ_TOL(fast.GetSpeedMps(), 163.80, 0.02,
                                      "speed must clamp to the J2735 ceiling of 163.80 m/s, not "
                                      "to the width of a uint16 (got " << fast.GetSpeedMps()
                                          << " m/s)");
        }

        // And elevation must clamp to the DE_Elevation range rather than to the
        // width of an int32: 61439 units = 6143.9 m.
        {
            ntnv2x::NtnV2xBsmHeader high;
            high.SetFromState(1, 1, 0, 0.0, 0.0, 100000.0 /* m */, 0.0, 0.0);
            NS_TEST_ASSERT_MSG_EQ_TOL(high.GetElevM(), 6143.9, 0.1,
                                      "elevation must clamp to the J2735 maximum of 6143.9 m "
                                      "(got " << high.GetElevM() << " m)");
            ntnv2x::NtnV2xBsmHeader low;
            low.SetFromState(1, 1, 0, 0.0, 0.0, -100000.0, 0.0, 0.0);
            NS_TEST_ASSERT_MSG_EQ_TOL(low.GetElevM(), -409.6, 0.1,
                                      "and to the minimum of -409.6 m");
        }

        // A clamped value must still survive the wire, or the clamp merely
        // moves the corruption into the encoder.
        {
            ntnv2x::NtnV2xBsmHeader hi;
            hi.SetFromState(7, 9, 1, 1.0, 2.0, 6000.0, 160.0, 10.0);
            Ptr<Packet> q = Create<Packet>(0);
            q->AddHeader(hi);
            ntnv2x::NtnV2xBsmHeader back;
            q->RemoveHeader(back);
            NS_TEST_ASSERT_MSG_EQ_TOL(back.GetElevM(), 6000.0, 0.05,
                                      "a legal high elevation must round-trip through 16 bits");
            NS_TEST_ASSERT_MSG_EQ_TOL(back.GetSpeedMps(), 160.0, 0.02,
                                      "and a legal high speed");
        }
        // Negative elevation must round-trip too: a 16-bit field read unsigned
        // would turn -100 m into a large positive one.
        {
            ntnv2x::NtnV2xBsmHeader neg;
            neg.SetFromState(7, 9, 1, 1.0, 2.0, -100.0, 5.0, 10.0);
            Ptr<Packet> q = Create<Packet>(0);
            q->AddHeader(neg);
            ntnv2x::NtnV2xBsmHeader back;
            q->RemoveHeader(back);
            NS_TEST_ASSERT_MSG_EQ_TOL(back.GetElevM(), -100.0, 0.05,
                                      "a negative elevation must survive the narrowed field; "
                                      "reading it unsigned turns it into +6453.6 m");
        }
    }
};

// ============================================================================
//  WS-D / V1: NR PC5 sidelink Mode 2 (TS 38.321 §5.22). Vehicles exchange BSMs
//  directly over PC5 — no gNB. This asserts: (1) every UE autonomously selects a
//  resource and transmits; (2) with a pool wide enough for sensing to spread the
//  UEs, in-range PRR is high and the half-duplex rule holds (no self-reception);
//  (3) forcing all UEs onto a single subchannel (pool=1) causes measurable
//  co-channel collisions, i.e. PRR drops versus the spread case — proving the
//  collision + sensing model is real, not a pass-through.
// ============================================================================
/// V2X-3: a real SAE J2735 basic safety message must cross PC5.
///
/// The sidelink used to deliver an (id, byte count) pair. There was no
/// Ptr<Packet> anywhere in ntn-nr-sidelink.cc, the PC5 example did not even
/// include the BSM header, and NtnV2xBsmHeader appeared only in this test suite
/// and on the Uu relay path. The README's claim that vehicles broadcast J2735
/// BSMs over PC5 described something the code did not do, the content of a BSM
/// influenced nothing, and nothing at PDCP, RLC or IP could ever attach to the
/// sidelink.
///
/// This drives two vehicles through the real Mode-2 channel and asserts that
/// what arrives is the header that was sent, with the sender's identity, its
/// message count and its position intact. Reverting the sidelink to a byte
/// count fails at compile time, which is the strongest form this check can
/// take; with a packet that carried no header it fails at the first assertion.
/// V2X-5: PC5 reception must take time.
///
/// The whole delivery loop ran synchronously inside SlotTick, so DeliverRx
/// fired in the same simulation event as the transmission and the one-way delay
/// of every sidelink message was identically zero. The class exposed no delay
/// accessor at all, so nothing could notice. A V2X safety study whose latency
/// is zero by construction cannot say anything about latency, which is most of
/// what such a study is for.
///
/// A transmission occupies its slot, so the floor is one slot duration plus
/// propagation. This asserts the delay is present, is at least a slot, and is
/// not wildly more.
class NtnSidelinkDelayIsRealTest : public TestCase
{
  public:
    NtnSidelinkDelayIsRealTest()
        : TestCase("V2X-5: a PC5 message takes at least one slot to arrive")
    {
    }

  private:
    void DoRun() override
    {
        auto ch = CreateObject<NtnSlChannel>();
        NtnSlResourcePool pool;
        pool.numSubchannels = 4;
        pool.slotDuration = MilliSeconds(1); // mu = 0
        ch->SetResourcePool(pool);
        ch->SetTxPowerDbm(23.0);
        ch->SetDecodeThresholdDbm(-115.0);

        for (uint32_t i = 0; i < 3; ++i)
        {
            auto ue = CreateObject<NtnSlUeMac>();
            ue->SetUeId(i);
            auto mob = CreateObject<ConstantVelocityMobilityModel>();
            mob->SetPosition(Vector(30.0 * i, 0.0, 0.0));
            mob->SetVelocity(Vector(20.0, 0.0, 0.0));
            ue->SetMobility(mob);
            ue->SetSelectionWindow(1, 20);
            ue->SetReservationPeriod(20);
            ue->SetPacketBytes(190);
            ue->AssignStreams(700 + i);
            ch->AddUe(ue);
        }

        ch->Start(MilliSeconds(1), Seconds(1.0));
        Simulator::Stop(Seconds(1.0) + MilliSeconds(10));
        Simulator::Run();
        Simulator::Destroy();

        NS_TEST_ASSERT_MSG_GT(ch->GetDelaySampleCount(), 10u,
                              "the run must deliver enough messages to have a delay to report");

        const double mean = ch->GetMeanDelayMs();
        NS_TEST_ASSERT_MSG_GT(mean, 0.999,
                              "a PC5 message cannot arrive before the slot carrying it ends; a "
                              "mean at or near zero means reception is still happening in the "
                              "same event as transmission");
        // One slot plus propagation over tens of metres: the propagation term is
        // well under a microsecond, so anything far above one slot means the
        // delay is being accumulated wrongly rather than measured.
        NS_TEST_ASSERT_MSG_LT(mean, 1.1,
                              "the delay must be one slot plus a sub-microsecond propagation "
                              "term, not a multiple of the slot");
        NS_TEST_ASSERT_MSG_GT(ch->GetMaxDelayMs(), mean - 1e-9,
                              "the maximum cannot be below the mean");
    }
};

class NtnSidelinkCarriesBsmTest : public TestCase
{
  public:
    NtnSidelinkCarriesBsmTest()
        : TestCase("V2X-3: a J2735 BSM crosses PC5 and is read back")
    {
    }

  private:
    void DoRun() override
    {
        auto ch = CreateObject<NtnSlChannel>();
        NtnSlResourcePool pool;
        pool.numSubchannels = 4;
        pool.slotDuration = MilliSeconds(1);
        ch->SetResourcePool(pool);
        ch->SetTxPowerDbm(23.0);
        ch->SetDecodeThresholdDbm(-115.0);

        constexpr uint32_t kN = 2;
        std::vector<Ptr<MobilityModel>> mobs(kN);
        std::vector<uint8_t> cnt(kN, 0);
        uint32_t decoded = 0;
        uint32_t idMismatch = 0;
        double maxPosErr = 0.0;
        // Keyed by SENDER: each vehicle runs its own msgCnt sequence, and
        // folding them into one counter would make interleaved traffic look
        // discontinuous when it is not.
        std::map<uint32_t, int> lastSeen;
        uint32_t nonConsecutive = 0;

        for (uint32_t i = 0; i < kN; ++i)
        {
            auto ue = CreateObject<NtnSlUeMac>();
            ue->SetUeId(i);
            auto mob = CreateObject<ConstantVelocityMobilityModel>();
            // 40 m apart, well inside decode range, moving so the position
            // carried in each BSM actually changes between transmissions.
            mob->SetPosition(Vector(40.0 * i, 0.0, 0.0));
            mob->SetVelocity(Vector(20.0, 0.0, 0.0));
            ue->SetMobility(mob);
            mobs[i] = mob;
            ue->SetSelectionWindow(1, 20);
            ue->SetReservationPeriod(20);
            ue->SetPacketBytes(190);
            ue->AssignStreams(500 + i);

            NtnSlUeMac::SlTxPacketCallback txCb =
                [&mobs, &cnt](uint32_t id) -> Ptr<Packet> {
                const Vector p = mobs[id]->GetPosition();
                const Vector v = mobs[id]->GetVelocity();
                ntnv2x::NtnV2xBsmHeader h;
                h.SetFromState(cnt[id], id, 1234,
                               p.y / 111320.0, p.x / 111320.0, p.z,
                               v.GetLength(), 90.0);
                cnt[id] = static_cast<uint8_t>((cnt[id] + 1) % 128);
                Ptr<Packet> pkt = Create<Packet>(190 - h.GetSerializedSize());
                pkt->AddHeader(h);
                return pkt;
            };
            ue->SetTxPacketCallback(txCb);

            NtnSlUeMac::SlRxCallback rxCb =
                [&](uint32_t from, Ptr<Packet> pkt) {
                    ntnv2x::NtnV2xBsmHeader h;
                    if (pkt->GetSize() < h.GetSerializedSize())
                    {
                        return;
                    }
                    pkt->RemoveHeader(h);
                    ++decoded;
                    if (h.GetId() != from)
                    {
                        ++idMismatch;
                    }
                    const double x = h.GetLonDeg() * 111320.0;
                    maxPosErr = std::max(maxPosErr, std::abs(x - mobs[from]->GetPosition().x));
                    auto it = lastSeen.find(from);
                    if (it != lastSeen.end() &&
                        static_cast<int>(h.GetMsgCnt()) != (it->second + 1) % 128)
                    {
                        ++nonConsecutive;
                    }
                    lastSeen[from] = static_cast<int>(h.GetMsgCnt());
                };
            ue->SetRxCallback(rxCb);
            ch->AddUe(ue);
        }

        ch->Start(MilliSeconds(1), Seconds(1.0));
        Simulator::Stop(Seconds(1.0) + MilliSeconds(1));
        Simulator::Run();

        NS_TEST_ASSERT_MSG_GT(decoded, 10u,
                              "a real header must arrive over PC5; zero decodes means the "
                              "sidelink is carrying an opaque byte count again and the BSM "
                              "content influences nothing");
        NS_TEST_ASSERT_MSG_EQ(idMismatch, 0u,
                              "the sender identity inside the BSM must match the sender the MAC "
                              "reports; a mismatch means the payload is not the sender's");
        // The J2735 latitude/longitude fields are 1/10 microdegree, which is
        // about 11 mm, so the round-trip error is quantisation and nothing else.
        NS_TEST_ASSERT_MSG_LT(maxPosErr, 0.05,
                              "the decoded position must be the sender's true position to within "
                              "the J2735 field quantisation");
        // With two vehicles in decode range and almost no collisions, a
        // sender's sequence should arrive essentially unbroken. Allowing a
        // tenth leaves room for the half-duplex slots the Mode-2 scheduler
        // legitimately creates.
        NS_TEST_ASSERT_MSG_LT(nonConsecutive, decoded / 10 + 2,
                              "each sender's message counts must arrive consecutive; wholesale "
                              "breaks mean the payload is being rebuilt per receiver rather "
                              "than once per transmission");

        Simulator::Destroy();
    }
};

class NtnSidelinkMode2Test : public TestCase
{
  public:
    NtnSidelinkMode2Test()
        : TestCase("WS-D V1 - NR PC5 sidelink Mode 2 selection, half-duplex, and PRR")
    {
    }

  private:
    // Build a line of UEs 20 m apart, run the SL channel, return the channel so
    // the caller can read KPIs. selfRx is set true if any UE ever received its
    // own packet (a half-duplex violation).
    Ptr<NtnSlChannel> RunScenario(uint32_t numUes, uint32_t numSubch, bool& selfRx,
                                  std::vector<uint32_t>& rxPerUe)
    {
        auto ch = CreateObject<NtnSlChannel>();
        NtnSlResourcePool pool;
        pool.numSubchannels = numSubch;
        pool.slotDuration = MilliSeconds(1);
        ch->SetResourcePool(pool);
        ch->SetTxPowerDbm(23.0);
        ch->SetDecodeThresholdDbm(-115.0);

        rxPerUe.assign(numUes, 0);
        selfRx = false;
        std::vector<Ptr<NtnSlUeMac>> ues;
        for (uint32_t i = 0; i < numUes; ++i)
        {
            auto ue = CreateObject<NtnSlUeMac>();
            ue->SetUeId(i);
            auto mob = CreateObject<ConstantPositionMobilityModel>();
            mob->SetPosition(Vector(20.0 * i, 0.0, 0.0)); // 20 m spacing
            ue->SetMobility(mob);
            ue->SetSelectionWindow(1, 20);
            ue->SetReservationPeriod(20); // 20 ms BSM period (@ mu=0)
            ue->SetPacketBytes(190);
            ue->AssignStreams(100 + i);
            uint32_t self = i;
            NtnSlUeMac::SlRxCallback cb =
                [&, self](uint32_t from, Ptr<Packet> /*pkt*/) {
                    if (from == self)
                    {
                        selfRx = true;
                    }
                    else
                    {
                        rxPerUe[self]++;
                    }
                };
            ue->SetRxCallback(cb);
            ch->AddUe(ue);
            ues.push_back(ue);
        }
        ch->Start(MilliSeconds(1), MilliSeconds(600));
        Simulator::Run();

        selfRx = selfRx; // captured by reference
        // stash tx counts via KPIs before destroy
        Ptr<NtnSlChannel> ret = ch;
        // Keep ues alive until after Run via the channel's internal vector.
        Simulator::Destroy();
        return ret;
    }

    void DoRun() override
    {
        // Case A: wide pool (5 subchannels) — sensing spreads the 4 UEs.
        bool selfRxA = false;
        std::vector<uint32_t> rxA;
        auto chA = RunScenario(4, 5, selfRxA, rxA);

        NS_TEST_ASSERT_MSG_EQ(selfRxA, false, "half-duplex: a UE must never receive its own TX");
        NS_TEST_ASSERT_MSG_GT(chA->GetTxTotal(), 0u, "every UE must autonomously transmit");
        // Neighbours 20/40 m away are well within range -> high short-range PRR.
        double prrNearA = chA->GetPrrWithinRange(45.0);
        NS_TEST_ASSERT_MSG_GT(prrNearA, 0.9,
                              "with a wide pool, in-range PRR must be high (sensing avoids collisions)");

        // Case B: degenerate pool (1 subchannel) — all UEs forced to contend for
        // the same subchannel -> co-channel collisions -> PRR drops.
        bool selfRxB = false;
        std::vector<uint32_t> rxB;
        auto chB = RunScenario(4, 1, selfRxB, rxB);
        double prrNearB = chB->GetPrrWithinRange(45.0);

        NS_TEST_ASSERT_MSG_EQ(selfRxB, false, "half-duplex holds under contention too");
        NS_TEST_ASSERT_MSG_LT(prrNearB, prrNearA,
                              "a 1-subchannel pool must collide more than a 5-subchannel pool");
    }
};


/// V2X-6: SUMO's speed must reach the mobility model.
///
/// SumoTraciBridge::EmitSample called SetPosition and nothing else, so the
/// speedMps parsed out of every FCD row was read and discarded. Any consumer
/// asking a SUMO-driven vehicle how fast it was going got zero - which meant
/// the SAE J2735 BSM Part I speed and heading fields, computed from
/// GetVelocity(), were 0 and atan2(0,0)=0 on every packet the module put on the
/// wire, and any Doppler or velocity-dependent channel saw a stationary car.
class SumoSpeedReachesTheMobilityModelTest : public TestCase
{
  public:
    SumoSpeedReachesTheMobilityModelTest()
        : TestCase("V2X-6: FCD speed and heading reach the mobility model")
    {
    }

    void DoRun() override
    {
        // A diagonal path, so a wrong heading cannot hide behind an axis.
        // Columns are time, id, x, y, z, speed.
        const std::string trace = "/tmp/ntn-v2x-v2x6-fcd.csv";
        {
            std::ofstream f(trace);
            f << "time,id,x,y,z,speed\n";
            for (int k = 0; k <= 10; ++k)
            {
                const double t = k * 1.0;
                // veh0: 3-4-5 triangle each second, 15 m east and 20 m north,
                // and a speed column that DISAGREES with the displacement.
                // Displacement over a 1 s cadence gives 25 m/s; SUMO reports
                // the instantaneous 31 m/s. Real FCD behaves this way - the
                // trace cadence is coarser than the vehicle's dynamics - and
                // the disagreement is what makes it testable which source is
                // used. SUMO's column is authoritative.
                f << t << ",veh0," << (15.0 * k) << "," << (20.0 * k) << ",1.5,31.0\n";
            }
        }

        Ptr<SumoTraciBridge> br = CreateObject<SumoTraciBridge>();
        NS_TEST_ASSERT_MSG_EQ(br->LoadFcdTrace(trace), true, "trace load failed");

        Ptr<ConstantVelocityMobilityModel> mob = CreateObject<ConstantVelocityMobilityModel>();
        mob->SetPosition(Vector{0, 0, 0});
        br->RegisterVehicle("veh0", mob);

        for (int k = 0; k <= 10; ++k)
        {
            Simulator::Schedule(Seconds(k * 1.0), [br]() { br->Step(); });
        }
        Simulator::Stop(Seconds(12.0));
        Simulator::Run();

        const Vector v = mob->GetVelocity();
        const double speed = std::sqrt(v.x * v.x + v.y * v.y);
        NS_TEST_ASSERT_MSG_GT(br->VelocitySamplesApplied(), 0u,
                              "at least one FCD sample must have produced a velocity; zero here "
                              "means the speed column is being parsed and thrown away again");
        NS_TEST_ASSERT_MSG_EQ(br->VelocitySamplesDropped(), 0u,
                              "and none dropped, since this model can hold a velocity");
        NS_TEST_ASSERT_MSG_EQ_TOL(speed, 31.0, 0.5,
                                  "the magnitude must come from SUMO's own speed column (31 m/s), "
                                  "not from differencing positions across the trace cadence "
                                  "(which would give 25 m/s here). SUMO is authoritative about "
                                  "speed; a coarse FCD cadence is not");

        // Heading from the displacement: atan2(20, 15) = 53.13 degrees.
        const double headingDeg = std::atan2(v.y, v.x) * 180.0 / M_PI;
        NS_TEST_ASSERT_MSG_EQ_TOL(headingDeg, 53.130102, 0.5,
                                  "the heading comes from the displacement between consecutive "
                                  "samples; 0 here is the old behaviour, and an axis-aligned "
                                  "path would not have distinguished the two");
        Simulator::Destroy();

        // A model that CANNOT hold a velocity must be counted, not silently
        // ignored - that is how the defect stayed invisible.
        Ptr<SumoTraciBridge> br2 = CreateObject<SumoTraciBridge>();
        NS_TEST_ASSERT_MSG_EQ(br2->LoadFcdTrace(trace), true, "trace load failed");
        Ptr<ConstantPositionMobilityModel> fixed = CreateObject<ConstantPositionMobilityModel>();
        fixed->SetPosition(Vector{0, 0, 0});
        br2->RegisterVehicle("veh0", fixed);
        for (int k = 0; k <= 10; ++k)
        {
            Simulator::Schedule(Seconds(k * 1.0), [br2]() { br2->Step(); });
        }
        Simulator::Stop(Seconds(12.0));
        Simulator::Run();
        NS_TEST_ASSERT_MSG_GT(br2->VelocitySamplesDropped(), 0u,
                              "registering a ConstantPositionMobilityModel discards every speed "
                              "SUMO reported, and that must be COUNTED");
        NS_TEST_ASSERT_MSG_EQ(br2->VelocitySamplesApplied(), 0u, "and none applied");
        Simulator::Destroy();
    }
};


/// V2X-4: the sidelink PHY must respond to distance, blockage and noise.
///
/// It did not. Path loss was `40 + 22*log10(d)` and decode was a bare RSRP
/// threshold: no noise term, no SINR, no shadowing, no vehicle blockage. At the
/// example's 23 dBm and -115 dBm threshold that solves to a 28 km decode range,
/// so every vehicle in a 475 m platoon decoded everything it was not
/// half-duplex-blocked from, and PRR-vs-distance - the single KPI this module
/// advertises - had an inert distance axis.
class SidelinkTr37885PhysicsTest : public TestCase
{
  public:
    SidelinkTr37885PhysicsTest()
        : TestCase("V2X-4: TR 37.885 Highway path loss and an SINR-based decode")
    {
    }

    void DoRun() override
    {
        Ptr<NtnSlChannel> ch = CreateObject<NtnSlChannel>();
        ch->SetShadowingSigmaDb(0.0); // deterministic for the closed-form check
        ch->SetCarrierFrequencyHz(5.9e9);

        auto mob = [](double x) {
            Ptr<ConstantPositionMobilityModel> m = CreateObject<ConstantPositionMobilityModel>();
            m->SetPosition(Vector{x, 0.0, 1.5});
            return m;
        };

        // ---- TR 37.885 Table 6.2.1-1 Highway LOS, checked against the formula
        //      PL = 32.4 + 20 log10(d) + 20 log10(fc[GHz]) ----
        Ptr<MobilityModel> a = mob(0.0);
        Ptr<MobilityModel> b = mob(100.0);
        const double expectPl =
            32.4 + 20.0 * std::log10(100.0) + 20.0 * std::log10(5.9);
        const double rsrp100 = ch->SlRsrpDbmForTest(a, b);
        NS_TEST_ASSERT_MSG_EQ_TOL(23.0 - rsrp100, expectPl, 0.1,
                                  "path loss at 100 m must match the TR 37.885 Highway LOS "
                                  "closed form; the old model was a 40 + 22log10(d) fit with no "
                                  "standards basis");

        // Free-space slope: doubling the distance costs 6.02 dB. The old
        // exponent 2.2 gives 6.62, so this distinguishes the two.
        Ptr<MobilityModel> c = mob(200.0);
        const double rsrp200 = ch->SlRsrpDbmForTest(a, c);
        NS_TEST_ASSERT_MSG_EQ_TOL(rsrp100 - rsrp200, 6.0206, 0.05,
                                  "the TR 37.885 exponent is 2 (20log10), so doubling range "
                                  "costs 6.02 dB; the previous 2.2 exponent cost 6.62");

        // ---- A noise floor exists, and decode is an SINR test against it ----
        const double nf = ch->NoiseFloorDbm();
        NS_TEST_ASSERT_MSG_EQ_TOL(nf, -174.0 + 10.0 * std::log10(10.0 * 12.0 * 15e3) + 9.0, 0.01,
                                  "kTB over one subchannel plus the receiver noise figure");
        NS_TEST_ASSERT_MSG_LT(nf, -95.0, "a 1.8 MHz subchannel floor is around -102 dBm");

        // The decode range is now bounded by the SINR test rather than by an
        // RSRP threshold picked independently of the noise. Beyond it, decode
        // must fail - which is the property the distance axis needs.
        Ptr<MobilityModel> far = mob(50e3);
        const double rsrpFar = ch->SlRsrpDbmForTest(a, far);
        NS_TEST_ASSERT_MSG_LT(rsrpFar - nf, ch->GetDecodeSinrDb(),
                              "at 50 km the received power is below the noise floor plus the "
                              "decode threshold, so the link must fail; under the old model "
                              "23 - (40 + 22log10(50000)) = -63 dBm still cleared a -115 dBm "
                              "threshold and decoded");
        Ptr<MobilityModel> near = mob(200.0);
        NS_TEST_ASSERT_MSG_GT(ch->SlRsrpDbmForTest(a, near) - nf, ch->GetDecodeSinrDb(),
                              "and a 200 m highway link must still close comfortably");

        // ---- NLOSv: a vehicle between the two ends costs additional loss ----
        // Registering a blocker on the line must reduce received power. Without
        // this there is no blockage model at all, which is the TR 37.885 term
        // that actually shapes a highway PRR curve.
        Ptr<NtnSlChannel> blocked = CreateObject<NtnSlChannel>();
        blocked->SetShadowingSigmaDb(0.0);
        blocked->SetCarrierFrequencyHz(5.9e9);
        blocked->SetNlosvBlockage(20.0, 0.0); // deterministic 20 dB for the check
        NS_TEST_ASSERT_MSG_EQ(blocked->AddBlockerForTest(mob(100.0)), true, "blocker registered");
        Ptr<MobilityModel> t0 = mob(0.0);
        Ptr<MobilityModel> r0 = mob(200.0);
        const double blockedRsrp = blocked->SlRsrpDbmForTest(t0, r0);
        const double clearRsrp = ch->SlRsrpDbmForTest(t0, r0);
        NS_TEST_ASSERT_MSG_EQ_TOL(clearRsrp - blockedRsrp, 20.0, 0.1,
                                  "a vehicle on the line costs the NLOSv additional loss");

        // A vehicle well off the line must NOT block. Otherwise 'blockage'
        // would just be 'another vehicle exists'.
        Ptr<NtnSlChannel> offLine = CreateObject<NtnSlChannel>();
        offLine->SetShadowingSigmaDb(0.0);
        offLine->SetCarrierFrequencyHz(5.9e9);
        offLine->SetNlosvBlockage(20.0, 0.0);
        Ptr<ConstantPositionMobilityModel> side = CreateObject<ConstantPositionMobilityModel>();
        side->SetPosition(Vector{100.0, 50.0, 1.5}); // 50 m lateral
        offLine->AddBlockerForTest(side);
        NS_TEST_ASSERT_MSG_EQ_TOL(offLine->SlRsrpDbmForTest(t0, r0), clearRsrp, 0.1,
                                  "a vehicle 50 m off the path does not obstruct it");
    }
};

class NtnV2xTestSuite : public TestSuite
{
  public:
    NtnV2xTestSuite()
        : TestSuite("ntn-v2x", Type::UNIT)
    {
        AddTestCase(new FcdTraceReplayJitterTest, TestCase::Duration::QUICK);
        AddTestCase(new SumoSpeedReachesTheMobilityModelTest, TestCase::Duration::QUICK);
        AddTestCase(new SidelinkTr37885PhysicsTest, TestCase::Duration::QUICK);
        AddTestCase(new V2xLeoDirectFreeSpaceTest, TestCase::Duration::QUICK);
        AddTestCase(new V2xLeoRelayDirectVsRelayTest, TestCase::Duration::QUICK);
        AddTestCase(new MaritimeBouncesInBoxTest, TestCase::Duration::QUICK);
        AddTestCase(new J2735BsmHeaderRoundTripTest, TestCase::Duration::QUICK);
        AddTestCase(new HundredVehicleSmokeTest, TestCase::Duration::EXTENSIVE);
        AddTestCase(new NtnSidelinkMode2Test, TestCase::Duration::QUICK);
        AddTestCase(new NtnSidelinkCarriesBsmTest, TestCase::Duration::QUICK);
        AddTestCase(new NtnSidelinkDelayIsRealTest, TestCase::Duration::QUICK);
    }
};

static NtnV2xTestSuite g_ntnV2xTestSuite;

} // namespace
