// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only
//
// In-depth ntn-sagin validation (18 Nov 2026 test campaign).
//
// Every case runs inside Simulator::Run():
//   - residual bit errors on a real IPv4/UDP link (binomial statistics, header
//     protection, UDP checksum per RFC 768 as the failure path);
//   - HAPS station keeping (ITU-R F.1500: 20 km, +-50 m vertical envelope) at
//     the configured track speed;
//   - UAV waypoint and boustrophedon search kinematics;
//   - the multi-layer router over live SGP4 satellites from ntn-constellation,
//     projected by ntn-cho into the scenario frame, gating a FlowMonitor-
//     measured data plane, plus its congestion rejection fed by real P2P
//     queue occupancy.

#include "ns3/boolean.h"
#include "ns3/box.h"
#include "ns3/constant-position-mobility-model.h"
#include "ns3/double.h"
#include "ns3/error-model.h"
#include "ns3/flow-monitor-helper.h"
#include "ns3/global-value.h"
#include "ns3/haps-mobility-model.h"
#include "ns3/inet-socket-address.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/ipv4-flow-classifier.h"
#include "ns3/multi-layer-router.h"
#include "ns3/node-container.h"
#include "ns3/ntn-tr38811-mobility-model.h"
#include "ns3/on-off-helper.h"
#include "ns3/packet-sink-helper.h"
#include "ns3/pointer.h"
#include "ns3/point-to-point-helper.h"
#include "ns3/point-to-point-net-device.h"
#include "ns3/queue.h"
#include "ns3/sagin-bit-error-model.h"
#include "ns3/sgp4-mobility-model.h"
#include "ns3/simulator.h"
#include "ns3/socket.h"
#include "ns3/string.h"
#include "ns3/test.h"
#include "ns3/udp-socket-factory.h"
#include "ns3/uav-mobility-models.h"
#include "ns3/walker-constellation.h"

#include <cmath>
#include <map>
#include <vector>

using namespace ns3;
using namespace ns3::ntnsagin;

namespace
{

/// A UDP flow over a 10 Mb/s P2P link with a SaginBitErrorModel on the
/// receiver. Payloads carry a known pattern (byte k = k mod 251) so the sink
/// can count every flipped bit it received.
struct BerLink
{
    Ptr<SaginBitErrorModel> em;
    uint64_t rxPackets{0};
    uint64_t rxBitErrors{0};
    uint64_t rxCorruptPackets{0};
    uint64_t rxOddErrorPackets{0}; //!< delivered with an odd number of flipped bits
    std::map<uint16_t, uint64_t> rxPerPort;

    static void Rx(BerLink* l, uint16_t port, Ptr<Socket> s)
    {
        Ptr<Packet> p;
        while ((p = s->Recv()))
        {
            std::vector<uint8_t> b(p->GetSize());
            p->CopyData(b.data(), b.size());
            uint64_t errs = 0;
            for (size_t k = 0; k < b.size(); ++k)
            {
                errs += __builtin_popcount(static_cast<uint8_t>(b[k] ^ (k % 251)));
            }
            ++l->rxPackets;
            ++l->rxPerPort[port];
            l->rxBitErrors += errs;
            l->rxCorruptPackets += errs > 0;
            l->rxOddErrorPackets += errs % 2;
        }
    }

    static void Send(Ptr<Socket> s, Address to, uint32_t n)
    {
        std::vector<uint8_t> b(n);
        for (uint32_t k = 0; k < n; ++k)
        {
            b[k] = k % 251;
        }
        s->SendTo(Create<Packet>(b.data(), n), 0, to);
    }

    /// \p count packets of \p size bytes to each of the given ports.
    void Run(double ber, int64_t stream, std::vector<uint16_t> ports, uint32_t count,
             uint32_t size, uint16_t filterPort = 0, Ptr<ErrorModel> inner = nullptr)
    {
        NodeContainer n;
        n.Create(2);
        PointToPointHelper p2p;
        p2p.SetDeviceAttribute("DataRate", StringValue("10Mbps"));
        p2p.SetChannelAttribute("Delay", StringValue("2ms"));
        NetDeviceContainer d = p2p.Install(n);
        em = CreateObject<SaginBitErrorModel>();
        em->SetBitErrorRate(ber);
        em->SetAttribute("UdpDstPort", UintegerValue(filterPort));
        if (inner)
        {
            em->SetInner(inner);
        }
        em->AssignStreams(stream);
        d.Get(1)->SetAttribute("ReceiveErrorModel", PointerValue(em));
        InternetStackHelper inet;
        inet.Install(n);
        Ipv4AddressHelper ip;
        ip.SetBase("10.5.0.0", "255.255.255.252");
        Ipv4InterfaceContainer ifs = ip.Assign(d);
        Ptr<Socket> tx = Socket::CreateSocket(n.Get(0), UdpSocketFactory::GetTypeId());
        for (uint16_t port : ports)
        {
            Ptr<Socket> rx = Socket::CreateSocket(n.Get(1), UdpSocketFactory::GetTypeId());
            rx->Bind(InetSocketAddress(Ipv4Address::GetAny(), port));
            rx->SetRecvCallback(MakeBoundCallback(&Rx, this, port));
            for (uint32_t k = 0; k < count; ++k)
            {
                Simulator::Schedule(MilliSeconds(2 * k + 1), &Send, tx,
                                    Address(InetSocketAddress(ifs.GetAddress(1), port)), size);
            }
        }
        Simulator::Run();
        Simulator::Destroy();
    }
};

} // namespace

// ---------------------------------------------------------------------------
/// Residual BER 1e-4 on 1000 x 1000-byte UDP payloads (8 Mbit): the bits the
/// sink finds flipped equal the model's own count, the count is binomial
/// (800 expected, 3 sigma = 85), the packet-corruption share matches
/// 1 - (1 - p)^8000 = 0.551, and no header is ever hit (every packet reaches
/// its port).
class SgnBerStatisticsTest : public TestCase
{
  public:
    SgnBerStatisticsTest()
        : TestCase("Residual BER on a real UDP link: exact accounting and binomial statistics")
    {
    }

  private:
    void DoRun() override
    {
        BerLink l;
        l.Run(1e-4, 7, {9}, 1000, 1000);
        NS_TEST_ASSERT_MSG_EQ(l.rxPackets, 1000u, "headers protected: every packet delivered");
        NS_TEST_ASSERT_MSG_EQ(l.rxBitErrors, l.em->GetBitsFlipped(),
                              "the sink sees exactly the bits the model flipped");
        NS_TEST_ASSERT_MSG_EQ(l.rxCorruptPackets, l.em->GetPacketsCorrupted(), "per-packet agreement");
        NS_TEST_ASSERT_MSG_EQ_TOL(double(l.rxBitErrors), 800.0, 85.0, "binomial mean n p, 3 sigma");
        NS_TEST_ASSERT_MSG_EQ_TOL(double(l.rxCorruptPackets) / 1000.0, 0.551, 0.047,
                                  "P(packet hit) = 1 - (1 - p)^8000, 3 sigma");
        NS_TEST_ASSERT_MSG_EQ(l.em->GetPacketsSeen(), 1000u, "every packet evaluated");

        BerLink clean;
        clean.Run(0.0, 7, {9}, 200, 1000);
        NS_TEST_ASSERT_MSG_EQ(clean.rxBitErrors, 0u, "BER 0 flips nothing");
    }
};

// ---------------------------------------------------------------------------
/// Failure paths: only the configured UDP port is corrupted; an inner error
/// model's losses are drops, not flips; with UDP checksums on (RFC 768, the
/// RFC 1071 one's-complement sum) the receiving stack discards corrupted
/// datagrams. The sum always catches an odd number of flipped bits; two flips
/// in the same bit column of different 16-bit words with opposite polarity
/// cancel, so a few even-error datagrams slip through, as RFC 1071 allows.
class SgnBerFilterInnerChecksumTest : public TestCase
{
  public:
    SgnBerFilterInnerChecksumTest()
        : TestCase("BER port filter, inner-model drops and UDP checksum discard")
    {
    }

  private:
    void DoRun() override
    {
        BerLink f;
        f.Run(1e-3, 3, {9, 10}, 300, 500, 9);
        NS_TEST_ASSERT_MSG_EQ(f.rxPerPort[9], 300u, "port 9 delivered");
        NS_TEST_ASSERT_MSG_EQ(f.rxPerPort[10], 300u, "port 10 delivered");
        NS_TEST_ASSERT_MSG_GT(f.rxBitErrors, 0u, "port 9 is corrupted");
        NS_TEST_ASSERT_MSG_EQ(f.rxBitErrors, f.em->GetBitsFlipped(), "and only port 9 carries the flips");

        Ptr<RateErrorModel> inner = CreateObject<RateErrorModel>();
        inner->SetUnit(RateErrorModel::ERROR_UNIT_PACKET);
        inner->SetRate(0.2);
        inner->AssignStreams(55);
        BerLink in;
        in.Run(1e-4, 4, {9}, 500, 500, 0, inner);
        NS_TEST_ASSERT_MSG_EQ(in.rxPackets + in.em->GetPacketsDropped(), 500u, "drops + deliveries = sent");
        NS_TEST_ASSERT_MSG_EQ_TOL(double(in.em->GetPacketsDropped()), 100.0, 27.0, "20% inner loss, 3 sigma");

        BooleanValue saved;
        GlobalValue::GetValueByName("ChecksumEnabled", saved);
        GlobalValue::Bind("ChecksumEnabled", BooleanValue(true));
        BerLink ck;
        ck.Run(2e-4, 5, {9}, 500, 1000);
        GlobalValue::Bind("ChecksumEnabled", saved);
        NS_TEST_ASSERT_MSG_GT(ck.em->GetPacketsCorrupted(), 100u, "many datagrams are hit");
        const uint64_t discarded = 500u - ck.rxPackets;
        NS_TEST_ASSERT_MSG_EQ(discarded + ck.rxCorruptPackets, ck.em->GetPacketsCorrupted(),
                              "every corrupted datagram is either discarded or slips through");
        NS_TEST_ASSERT_MSG_GT(discarded, 0.9 * ck.em->GetPacketsCorrupted(),
                              "the UDP checksum discards the large majority");
        NS_TEST_ASSERT_MSG_EQ(ck.rxOddErrorPackets, 0u,
                              "no datagram with an odd number of bit errors passes the checksum");
    }
};

// ---------------------------------------------------------------------------
/// Determinism of the BER draw: the same stream flips the same bits, other
/// streams differ but stay within the binomial 4-sigma band.
class SgnBerDeterminismTest : public TestCase
{
  public:
    SgnBerDeterminismTest()
        : TestCase("BER flips are reproducible per stream and bounded across streams")
    {
    }

  private:
    void DoRun() override
    {
        BerLink a;
        a.Run(5e-4, 21, {9}, 300, 500);
        BerLink b;
        b.Run(5e-4, 21, {9}, 300, 500);
        NS_TEST_ASSERT_MSG_EQ(a.rxBitErrors, b.rxBitErrors, "same stream, same flips");
        NS_TEST_ASSERT_MSG_EQ(a.rxCorruptPackets, b.rxCorruptPackets, "same packets hit");
        bool differ = false;
        for (int64_t s : {22, 23, 24})
        {
            BerLink o;
            o.Run(5e-4, s, {9}, 300, 500);
            differ |= o.rxBitErrors != a.rxBitErrors;
            // n p = 600, sigma = 24.5
            NS_TEST_ASSERT_MSG_EQ_TOL(double(o.rxBitErrors), 600.0, 98.0, "4 sigma band");
        }
        NS_TEST_ASSERT_MSG_EQ(differ, true, "other streams draw other bits");
    }
};

// ---------------------------------------------------------------------------
/// HAPS station keeping over two hours: the platform flies its figure-eight
/// at the configured HorizontalSpeed (each tick covers v dt of track), stays
/// inside the station-keeping radius and the ITU-R F.1500 +-50 m vertical
/// envelope around 20 km, and laps the track in L / v with L = 5.2441 R (the
/// lemniscate perimeter).
class SgnHapsStationKeepingTest : public TestCase
{
  public:
    SgnHapsStationKeepingTest()
        : TestCase("HAPS flies its station-keeping track at the configured speed inside F.1500 bounds")
    {
    }

  private:
    struct Probe
    {
        uint32_t n{0};
        double maxSpeedErr{0};
        double maxR{0};
        double minZ{1e9};
        double maxZ{-1e9};
        double sumSpeed{0};
        std::vector<double> crossings; //!< times of passing the eastern tip
        Vector last;
        bool have{false};
    };

    static void OnMove(Probe* p, Ptr<HapsMobilityModel> m, Ptr<const MobilityModel>)
    {
        const Vector c = m->GetCenter();
        const Vector x = m->GetPosition();
        if (p->have)
        {
            const double h = std::hypot(x.x - p->last.x, x.y - p->last.y);
            p->maxSpeedErr = std::max(p->maxSpeedErr, std::abs(h - 15.0));
            p->sumSpeed += h;
            ++p->n;
            // a lap ends at the eastern tip (phi = 0), where y crosses zero upward
            if (p->last.y - c.y < 0 && x.y - c.y >= 0 && x.x - c.x > 0)
            {
                p->crossings.push_back(Simulator::Now().GetSeconds());
            }
        }
        p->maxR = std::max(p->maxR, std::hypot(x.x - c.x, x.y - c.y));
        p->minZ = std::min(p->minZ, x.z - c.z);
        p->maxZ = std::max(p->maxZ, x.z - c.z);
        p->last = x;
        p->have = true;
    }

    void DoRun() override
    {
        Ptr<HapsMobilityModel> h = CreateObject<HapsMobilityModel>();
        h->SetAttribute("HorizontalSpeed", DoubleValue(15.0));
        h->SetAttribute("StationKeepRadius", DoubleValue(5000.0));
        h->SetCenter(Vector(0, 0, 0));
        Probe p;
        h->TraceConnectWithoutContext("CourseChange", MakeBoundCallback(&OnMove, &p, h));
        Simulator::Stop(Seconds(7200));
        Simulator::Run();
        Simulator::Destroy();
        NS_TEST_ASSERT_MSG_GT(p.n, 7000u, "1 Hz updates for two hours");
        NS_TEST_ASSERT_MSG_LT(p.maxSpeedErr, 0.15, "every tick covers 15 m of track (1%)");
        NS_TEST_ASSERT_MSG_EQ_TOL(p.sumSpeed / p.n, 15.0, 0.02, "mean horizontal speed");
        NS_TEST_ASSERT_MSG_LT_OR_EQ(p.maxR, 5000.0 + 1e-6, "inside the station-keeping radius");
        NS_TEST_ASSERT_MSG_GT_OR_EQ(p.minZ, 20000.0 - 50.0, "F.1500 lower envelope");
        NS_TEST_ASSERT_MSG_LT_OR_EQ(p.maxZ, 20000.0 + 50.0, "F.1500 upper envelope");
        NS_TEST_ASSERT_MSG_GT_OR_EQ(p.crossings.size(), 2u, "at least two laps");
        if (p.crossings.size() >= 2)
        {
            const double lap = p.crossings[1] - p.crossings[0];
            NS_TEST_ASSERT_MSG_EQ_TOL(lap, 5.2441 * 5000.0 / 15.0, 3.0, "lap time = perimeter / speed");
        }
    }
};

// ---------------------------------------------------------------------------
/// UAV kinematics: the random-waypoint UAV never exceeds its speed on any
/// tick (including the arrival tick), stays in its box and hovers PauseTime
/// at each waypoint; the boustrophedon search flies exactly
/// n x leg + (n - 1) x strip of track and ends at the last waypoint.
class SgnUavKinematicsTest : public TestCase
{
  public:
    SgnUavKinematicsTest()
        : TestCase("UAV waypoint and search-pattern flights respect speed, box and coverage geometry")
    {
    }

  private:
    struct Probe
    {
        Vector last;
        bool have{false};
        double maxStep{0};
        double track{0};
        uint32_t outside{0};
        Box box;
    };

    static void OnMove(Probe* p, Ptr<const MobilityModel> m)
    {
        const Vector x = m->GetPosition();
        if (p->have)
        {
            const double s = CalculateDistance(x, p->last);
            p->maxStep = std::max(p->maxStep, s);
            p->track += s;
        }
        p->outside += !(x.x >= p->box.xMin - 1e-9 && x.x <= p->box.xMax + 1e-9 &&
                        x.y >= p->box.yMin - 1e-9 && x.y <= p->box.yMax + 1e-9 &&
                        x.z >= p->box.zMin - 1e-9 && x.z <= p->box.zMax + 1e-9);
        p->last = x;
        p->have = true;
    }

    void DoRun() override
    {
        Ptr<UavWaypointMobilityModel> w = CreateObject<UavWaypointMobilityModel>();
        Box box(-1000, 1000, -1000, 1000, 50, 200);
        Probe pw;
        pw.box = box;
        w->TraceConnectWithoutContext("CourseChange", MakeBoundCallback(&OnMove, &pw));
        w->SetPosition(Vector(0, 0, 100));
        w->SetBox(box);
        Simulator::Stop(Seconds(1200));
        Simulator::Run();
        Simulator::Destroy();
        // 20 m/s x 0.5 s tick
        NS_TEST_ASSERT_MSG_LT_OR_EQ(pw.maxStep, 10.0 + 1e-9, "no tick moves more than v dt");
        NS_TEST_ASSERT_MSG_EQ(pw.outside, 0u, "the UAV stays in its box");
        NS_TEST_ASSERT_MSG_GT(pw.track, 15000.0, "it actually flies (legs plus 2 s hovers)");
        NS_TEST_ASSERT_MSG_LT(pw.track, 24000.0 + 1e-6, "never faster than 20 m/s on average");

        Ptr<UavSearchPatternMobilityModel> s = CreateObject<UavSearchPatternMobilityModel>();
        s->SetAttribute("Speed", DoubleValue(12.0));
        Probe ps;
        ps.box = Box(-1, 501, -1, 201, 0, 200);
        s->TraceConnectWithoutContext("CourseChange", MakeBoundCallback(&OnMove, &ps));
        s->SetPosition(Vector(0, 0, 120));
        s->SetSearchArea(Vector(0, 0, 0), 500.0, 50.0, 5, 120.0);
        Simulator::Stop(Seconds(600));
        Simulator::Run();
        const Vector end = s->GetPosition();
        Simulator::Destroy();
        NS_TEST_ASSERT_MSG_EQ_TOL(ps.track, 5 * 500.0 + 4 * 50.0, 1e-6, "boustrophedon track length");
        NS_TEST_ASSERT_MSG_EQ_TOL(CalculateDistance(end, Vector(500, 200, 120)), 0.0, 1e-9,
                                  "ends at the last waypoint (odd strip count ends east)");
        NS_TEST_ASSERT_MSG_EQ(ps.outside, 0u, "search stays inside its area");
    }
};

// ---------------------------------------------------------------------------
/// Cross-module: the router's LEO layer is two SGP4 satellites of one plane
/// (ntn-constellation), projected into the scenario ENU frame (ntn-cho). Over
/// ten minutes the leading satellite sets and the trailing one rises. Every
/// second the router picks the higher satellite, its elevation equals the
/// propagator's WGS-84 look angle, and 10 UDP packets go over the P2P link to
/// the chosen satellite. FlowMonitor then counts, per satellite, exactly 10
/// packets per second it was chosen; below a 30 deg mask no LEO hop exists
/// (outage seconds send nothing).
class SgnRouterSgp4HandoffTest : public TestCase
{
  public:
    SgnRouterSgp4HandoffTest()
        : TestCase("Multi-layer router hands off between live SGP4 satellites and gates the data plane")
    {
    }

  private:
    void DoRun() override
    {
        ntncon::WalkerConfig w;
        w.total_sats = 10;
        w.num_planes = 1;
        w.altitude_km = 550.0;
        w.epoch_unix_s = 1767225600.0;
        auto el = ntncon::WalkerConstellation::BuildDelta(w);
        std::vector<Ptr<ntncon::Sgp4MobilityModel>> sg;
        for (size_t i : {size_t(0), size_t(9)})
        {
            Ptr<ntncon::Sgp4MobilityModel> m = CreateObject<ntncon::Sgp4MobilityModel>();
            m->SetElementsSgp4(el[i]);
            sg.push_back(m);
        }
        double lat0;
        double lon0;
        double alt0;
        sg[0]->GetGeodetic(lat0, lon0, alt0);
        NodeContainer n;
        n.Create(3); // 0 = ground, 1 = sat A (setting), 2 = sat B (rising)
        Ptr<ConstantPositionMobilityModel> gm = CreateObject<ConstantPositionMobilityModel>();
        n.Get(0)->AggregateObject(gm);
        std::vector<Ptr<NtnEnuProjectionMobilityModel>> enu;
        for (uint32_t i = 0; i < 2; ++i)
        {
            Ptr<NtnEnuProjectionMobilityModel> e = CreateObject<NtnEnuProjectionMobilityModel>();
            e->SetSource(sg[i]);
            e->SetReference(lat0, lon0, 0.0);
            n.Get(i + 1)->AggregateObject(e);
            enu.push_back(e);
        }
        Ptr<MultiLayerRouter> r = CreateObject<MultiLayerRouter>();
        r->AddNode(SaginLayer::Leo, enu[0]);
        r->AddNode(SaginLayer::Leo, enu[1]);
        r->SetMinElevationDeg(30.0);

        InternetStackHelper inet;
        inet.Install(n);
        PointToPointHelper p2p;
        p2p.SetDeviceAttribute("DataRate", StringValue("10Mbps"));
        p2p.SetChannelAttribute("Delay", StringValue("3ms"));
        Ipv4AddressHelper ip;
        std::vector<Ipv4Address> satAddr;
        for (uint32_t i = 1; i <= 2; ++i)
        {
            NetDeviceContainer d = p2p.Install(n.Get(0), n.Get(i));
            ip.SetBase(("10.80." + std::to_string(i) + ".0").c_str(), "255.255.255.252");
            satAddr.push_back(ip.Assign(d).GetAddress(1));
            Ptr<Socket> rx = Socket::CreateSocket(n.Get(i), UdpSocketFactory::GetTypeId());
            rx->Bind(InetSocketAddress(Ipv4Address::GetAny(), 7000));
        }
        Ptr<Socket> tx = Socket::CreateSocket(n.Get(0), UdpSocketFactory::GetTypeId());
        FlowMonitorHelper fmh;
        Ptr<FlowMonitor> fm = fmh.InstallAll();

        std::vector<int> chosen; // -1 outage, 0 = A, 1 = B
        double maxElErr = 0;
        uint32_t wrongPick = 0;
        for (int t = 0; t < 600; ++t)
        {
            Simulator::Schedule(Seconds(t), [&, t]() {
                const auto path = r->Route(gm);
                const double eA = sg[0]->GetElevationDeg(lat0, lon0);
                const double eB = sg[1]->GetElevationDeg(lat0, lon0);
                int pick = -1;
                if (path.size() >= 2)
                {
                    pick = path[1].node == Ptr<MobilityModel>(enu[0]) ? 0 : 1;
                    maxElErr = std::max(maxElErr, std::abs(path[1].elevationDeg - (pick ? eB : eA)));
                    wrongPick += (pick == 0) != (eA >= eB);
                }
                else
                {
                    wrongPick += std::max(eA, eB) >= 30.5; // a usable satellite was ignored
                }
                chosen.push_back(pick);
                if (pick >= 0)
                {
                    for (int k = 0; k < 10; ++k)
                    {
                        Simulator::Schedule(MilliSeconds(50 * k), [tx, pick, &satAddr]() {
                            tx->SendTo(Create<Packet>(200), 0, InetSocketAddress(satAddr[pick], 7000));
                        });
                    }
                }
            });
        }
        Simulator::Stop(Seconds(601));
        Simulator::Run();
        std::map<Ipv4Address, uint64_t> rxBy;
        Ptr<Ipv4FlowClassifier> cls = DynamicCast<Ipv4FlowClassifier>(fmh.GetClassifier());
        for (const auto& [id, st] : fm->GetFlowStats())
        {
            rxBy[cls->FindFlow(id).destinationAddress] += st.rxPackets;
        }
        Simulator::Destroy();

        uint32_t secA = 0;
        uint32_t secB = 0;
        uint32_t outage = 0;
        uint32_t switches = 0;
        for (size_t i = 0; i < chosen.size(); ++i)
        {
            secA += chosen[i] == 0;
            secB += chosen[i] == 1;
            outage += chosen[i] < 0;
            switches += i > 0 && chosen[i] != chosen[i - 1];
        }
        NS_TEST_ASSERT_MSG_EQ(wrongPick, 0u, "always the higher satellite above the mask");
        NS_TEST_ASSERT_MSG_LT(maxElErr, 0.2, "router elevation = SGP4 WGS-84 look angle");
        NS_TEST_ASSERT_MSG_GT(secA, 60u, "A serves first");
        NS_TEST_ASSERT_MSG_GT(secB, 60u, "B serves later");
        NS_TEST_ASSERT_MSG_GT(outage, 0u, "a 30 deg mask leaves a gap between the two");
        NS_TEST_ASSERT_MSG_EQ(switches, 2u, "A -> outage -> B");
        NS_TEST_ASSERT_MSG_EQ(rxBy[satAddr[0]], 10u * secA, "FlowMonitor: A carried its seconds");
        NS_TEST_ASSERT_MSG_EQ(rxBy[satAddr[1]], 10u * secB, "FlowMonitor: B carried its seconds");
    }
};

// ---------------------------------------------------------------------------
/// Overload failure path: the congestion-aware router reads the REAL transmit
/// queue of the P2P link toward each candidate. When a burst fills the queue
/// toward the higher HAPS above the 95% rejection mark, the router routes to
/// the other HAPS and counts the rejection; once the queue drains it returns.
class SgnRouterQueueRejectionTest : public TestCase
{
  public:
    SgnRouterQueueRejectionTest()
        : TestCase("Congestion-aware routing rejects a hop whose real P2P queue is full")
    {
    }

  private:
    void DoRun() override
    {
        NodeContainer n;
        n.Create(3);
        Ptr<ConstantPositionMobilityModel> g = CreateObject<ConstantPositionMobilityModel>();
        Ptr<ConstantPositionMobilityModel> hi = CreateObject<ConstantPositionMobilityModel>();
        hi->SetPosition(Vector(1000, 0, 20000)); // higher elevation
        Ptr<ConstantPositionMobilityModel> lo = CreateObject<ConstantPositionMobilityModel>();
        lo->SetPosition(Vector(30000, 0, 20000));
        n.Get(0)->AggregateObject(g);
        n.Get(1)->AggregateObject(hi);
        n.Get(2)->AggregateObject(lo);
        InternetStackHelper inet;
        inet.Install(n);
        PointToPointHelper p2p;
        p2p.SetDeviceAttribute("DataRate", StringValue("1Mbps"));
        p2p.SetQueue("ns3::DropTailQueue", "MaxSize", StringValue("100p"));
        NetDeviceContainer dHi = p2p.Install(n.Get(0), n.Get(1));
        NetDeviceContainer dLo = p2p.Install(n.Get(0), n.Get(2));
        Ipv4AddressHelper ip;
        ip.SetBase("10.90.1.0", "255.255.255.252");
        Ipv4Address aHi = ip.Assign(dHi).GetAddress(1);
        ip.SetBase("10.90.2.0", "255.255.255.252");
        ip.Assign(dLo);
        Ptr<Queue<Packet>> qHi = DynamicCast<PointToPointNetDevice>(dHi.Get(0))->GetQueue();
        Ptr<Queue<Packet>> qLo = DynamicCast<PointToPointNetDevice>(dLo.Get(0))->GetQueue();

        Ptr<MultiLayerRouter> r = CreateObject<MultiLayerRouter>();
        r->AddNode(SaginLayer::Haps, hi);
        r->AddNode(SaginLayer::Haps, lo);
        r->SetCongestionAware(true, 1.0, 0.95);
        r->SetLinkLoadSource([=](Ptr<MobilityModel>, Ptr<MobilityModel> node, double& cap, double& occ) {
            Ptr<Queue<Packet>> q = node == Ptr<MobilityModel>(hi) ? qHi : qLo;
            cap = 1e6;
            occ = double(q->GetNPackets()) / 100.0;
            return true;
        });
        // burst of 300 x 1000 B into the 1 Mb/s link toward 'hi' at t = 1 s: the
        // 100-packet device queue fills (the traffic-control queue holds the
        // rest) and drains at 125 packets/s
        Ptr<Socket> tx = Socket::CreateSocket(n.Get(0), UdpSocketFactory::GetTypeId());
        Simulator::Schedule(Seconds(1), [tx, aHi]() {
            for (int k = 0; k < 300; ++k)
            {
                tx->SendTo(Create<Packet>(1000), 0, InetSocketAddress(aHi, 9));
            }
        });
        std::vector<std::pair<double, bool>> picks; // (time, picked hi)
        for (double t : {0.5, 1.001, 1.3, 4.0})
        {
            Simulator::Schedule(Seconds(t), [&picks, r, g, hi]() {
                const auto p = r->Route(g);
                picks.emplace_back(Simulator::Now().GetSeconds(),
                                   p.size() >= 2 && p[1].node == Ptr<MobilityModel>(hi));
            });
        }
        Simulator::Run();
        const uint64_t rej = r->GetCongestionRejections();
        const uint32_t drops = qHi->GetTotalDroppedPackets();
        Simulator::Destroy();
        NS_TEST_ASSERT_MSG_EQ(picks.size(), 4u, "four routing decisions");
        NS_TEST_ASSERT_MSG_EQ(picks[0].second, true, "idle: the higher HAPS");
        NS_TEST_ASSERT_MSG_EQ(picks[1].second, false, "full queue: rejected, the other HAPS");
        NS_TEST_ASSERT_MSG_EQ(picks[2].second, false, "still above 95% while draining");
        NS_TEST_ASSERT_MSG_EQ(picks[3].second, true, "drained: back to the higher HAPS");
        NS_TEST_ASSERT_MSG_GT_OR_EQ(rej, 2u, "rejections counted");
        // the traffic-control layer above the device absorbs the rest of the
        // burst, so the device queue fills without dropping
        NS_TEST_ASSERT_MSG_EQ(drops, 0u, "the device queue is filled, not overrun");
    }
};

class NtnSaginInDepthTestSuite : public TestSuite
{
  public:
    NtnSaginInDepthTestSuite()
        : TestSuite("ntn-sagin-indepth", Type::UNIT)
    {
        AddTestCase(new SgnBerStatisticsTest, Duration::QUICK);
        AddTestCase(new SgnBerFilterInnerChecksumTest, Duration::QUICK);
        AddTestCase(new SgnBerDeterminismTest, Duration::QUICK);
        AddTestCase(new SgnHapsStationKeepingTest, Duration::QUICK);
        AddTestCase(new SgnUavKinematicsTest, Duration::QUICK);
        AddTestCase(new SgnRouterSgp4HandoffTest, Duration::QUICK);
        AddTestCase(new SgnRouterQueueRejectionTest, Duration::QUICK);
    }
};

static NtnSaginInDepthTestSuite g_ntnSaginInDepthTestSuite;
