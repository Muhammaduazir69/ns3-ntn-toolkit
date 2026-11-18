/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only
//
// In-depth cases (Nov 2026 test campaign): TR 38.821 maximum round-trip
// delays, timing advance and drift over a circular-orbit pass, SIB19
// carried as UDP packets over a satellite link whose delay follows the slant
// range, consumed
// by the UE pre-compensator with a link outage in the middle, measurement
// report codec fuzz and quantization properties, the location reporter under
// real motion, RACH window verdicts along a pass, and determinism.

#include "ns3/constant-position-mobility-model.h"
#include "ns3/constant-velocity-mobility-model.h"
#include "ns3/error-model.h"
#include "ns3/inet-socket-address.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/ntn-meas-report.h"
#include "ns3/ntn-rach-window.h"
#include "ns3/ntn-rrc-helper.h"
#include "ns3/ntn-sib19.h"
#include "ns3/ntn-timing-advance.h"
#include "ns3/ntn-ue-location-report.h"
#include "ns3/ntn-ue-sib19-consumer.h"
#include "ns3/ntn-ul-timing-precompensator.h"
#include "ns3/packet.h"
#include "ns3/point-to-point-net-device.h"
#include "ns3/rng-seed-manager.h"
#include "ns3/point-to-point-channel.h"
#include "ns3/point-to-point-helper.h"
#include "ns3/simulator.h"
#include "ns3/socket.h"
#include "ns3/string.h"
#include "ns3/test.h"
#include "ns3/udp-socket-factory.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <map>
#include <vector>

using namespace ns3;
using namespace ns3::ntnrrc;

namespace
{

constexpr double kC = 299792458.0;
constexpr double kRe = 6371e3; // TR 38.821 Section 4.2 uses a 6371 km spherical Earth

Ptr<MobilityModel>
Static(const Vector& p)
{
    auto m = CreateObject<ConstantPositionMobilityModel>();
    m->SetPosition(p);
    return m;
}

/// Point on the sphere seeing a satellite at (0, 0, R + h) under elevation
/// `elDeg`, on side `sign` of the sub-satellite point (TR 38.811 6.6.2 geometry).
Vector
PointAtElevation(double hM, double elDeg, double sign)
{
    const double el = elDeg * M_PI / 180.0;
    const double gamma = std::acos(kRe * std::cos(el) / (kRe + hM)) - el;
    return Vector(sign * kRe * std::sin(gamma), 0.0, kRe * std::cos(gamma));
}

/// A satellite on a circular Keplerian orbit (two-body, mu = 3.986004418e14,
/// TS 38.821 reference shells), expressed in an Earth-centered frame. ntn-rrc
/// cannot link ntn-constellation (SGP4) without a library cycle
/// (ntn-constellation -> satellite -> ntn-traffic -> ntn-rrc), so the depth
/// tests carry their own exact orbit: position and velocity are analytic, so
/// the TA and its drift have closed-form truths to compare against.
class CircularOrbitMobility : public MobilityModel
{
  public:
    static TypeId GetTypeId()
    {
        static TypeId tid = TypeId("ns3::NtnRrcDepthCircularOrbitMobility")
                                .SetParent<MobilityModel>()
                                .SetGroupName("NtnRrc");
        return tid;
    }

    void Set(double altM, double inclDeg, double raanDeg, double u0Deg)
    {
        m_r = kRe + altM;
        m_w = std::sqrt(3.986004418e14 / (m_r * m_r * m_r));
        const double i = inclDeg * M_PI / 180.0;
        const double o = raanDeg * M_PI / 180.0;
        m_u0 = u0Deg * M_PI / 180.0;
        m_e1 = Vector(std::cos(o), std::sin(o), 0.0);
        m_e2 = Vector(-std::sin(o) * std::cos(i), std::cos(o) * std::cos(i), std::sin(i));
    }

    Vector At(double t) const
    {
        const double u = m_u0 + m_w * t;
        return Vector(m_r * (std::cos(u) * m_e1.x + std::sin(u) * m_e2.x),
                      m_r * (std::cos(u) * m_e1.y + std::sin(u) * m_e2.y),
                      m_r * (std::cos(u) * m_e1.z + std::sin(u) * m_e2.z));
    }

    double Speed() const { return m_r * m_w; }

  private:
    Vector DoGetPosition() const override { return At(Simulator::Now().GetSeconds()); }

    void DoSetPosition(const Vector&) override {}

    Vector DoGetVelocity() const override
    {
        const double u = m_u0 + m_w * Simulator::Now().GetSeconds();
        const double v = m_r * m_w;
        return Vector(v * (-std::sin(u) * m_e1.x + std::cos(u) * m_e2.x),
                      v * (-std::sin(u) * m_e1.y + std::cos(u) * m_e2.y),
                      v * (-std::sin(u) * m_e1.z + std::cos(u) * m_e2.z));
    }

    double m_r{kRe + 600e3};
    double m_w{0.0};
    double m_u0{0.0};
    Vector m_e1{1, 0, 0};
    Vector m_e2{0, 1, 0};
};

Ptr<CircularOrbitMobility>
LeoSat(double altKm)
{
    auto m = CreateObject<CircularOrbitMobility>();
    m->Set(altKm * 1e3, 53.0, 30.0, 0.0);
    return m;
}

/// Ground point (6371 km sphere) under the satellite at time t, nudged by
/// (dLat, dLon) degrees.
Vector
GroundUnder(Ptr<CircularOrbitMobility> sat, double t, double dLatDeg = 0.0, double dLonDeg = 0.0)
{
    const Vector p = sat->At(t);
    const double r = p.GetLength();
    const double lat = std::asin(p.z / r) + dLatDeg * M_PI / 180.0;
    const double lon = std::atan2(p.y, p.x) + dLonDeg * M_PI / 180.0;
    return Vector(kRe * std::cos(lat) * std::cos(lon),
                  kRe * std::cos(lat) * std::sin(lon),
                  kRe * std::sin(lat));
}

} // namespace

/// TR 38.821 Table 5.3.2.1-1 (maximum round-trip delay, 10 deg minimum
/// elevation for UE and gateway): LEO-600 transparent 25.77 ms, regenerative
/// 12.89 ms; GEO transparent 541.46 ms, regenerative 270.73 ms. The total TA
/// for a UE and a gateway both at 10 deg must reproduce the table within
/// 0.05 ms, and the common TA (reference point at the UE) leaves zero
/// UE-specific part.
class TaMaxRoundTripTr38821Test : public TestCase
{
  public:
    TaMaxRoundTripTr38821Test()
        : TestCase("TR 38.821 Table 5.3.2.1-1: max RTD 25.77/12.89 ms LEO-600, 541.46/270.73 ms GEO")
    {
    }

  private:
    static Time Ta(double hM, PayloadMode mode, Time* ueSpecific = nullptr)
    {
        auto sat = Static(Vector(0, 0, kRe + hM));
        const Vector uePos = PointAtElevation(hM, 10.0, +1.0);
        NtnRrcHelper h;
        h.SetPayloadMode(mode);
        h.SetGatewayMobility(Static(PointAtElevation(hM, 10.0, -1.0)));
        h.SetReferencePosition(uePos);
        auto ta = h.InstallTimingAdvance(Static(uePos), sat);
        if (ueSpecific)
        {
            *ueSpecific = ta->ComputeUeSpecificTa();
        }
        return ta->ComputeTotalTa();
    }

    void DoRun() override
    {
        Time ueSpec;
        NS_TEST_ASSERT_MSG_EQ_TOL(Ta(600e3, PayloadMode::Transparent, &ueSpec).GetSeconds() * 1e3, 25.77, 0.05,
                                  "LEO-600 transparent 25.77 ms");
        NS_TEST_ASSERT_MSG_EQ_TOL(Ta(600e3, PayloadMode::RegenerativeLite).GetSeconds() * 1e3, 12.89, 0.05,
                                  "LEO-600 regenerative 12.89 ms");
        NS_TEST_ASSERT_MSG_EQ_TOL(Ta(35786e3, PayloadMode::Transparent).GetSeconds() * 1e3, 541.46, 0.05,
                                  "GEO transparent 541.46 ms");
        NS_TEST_ASSERT_MSG_EQ_TOL(Ta(35786e3, PayloadMode::RegenerativeLite).GetSeconds() * 1e3, 270.73, 0.05,
                                  "GEO regenerative 270.73 ms");
        NS_TEST_ASSERT_MSG_EQ_TOL(ueSpec.GetSeconds(), 0.0, 1e-9,
                                  "reference point at the UE: the whole TA is common");
        // Failure path: transparent without a gateway is flagged, not guessed.
        NtnTimingAdvance bare;
        bare.SetPayloadMode(PayloadMode::Transparent);
        NS_TEST_ASSERT_MSG_EQ(bare.FeederGeometryMissing(), true, "missing feeder flagged");
    }
};

/// Timing advance along a LEO-600 pass (circular orbit, 53 deg). At every
/// sample of the pass the regenerative TA is exactly 2 |r_sat - r_ue| / c, the
/// drift equals 2 (range rate) / c with the range rate from the analytic
/// orbital velocity, |drift| never exceeds 2 v_sat / c,
/// and the drift changes sign once (approach, then recession).
class TaAlongLeoPassTest : public TestCase
{
  public:
    TaAlongLeoPassTest()
        : TestCase("TA along a LEO-600 pass: 2d/c, drift = 2 rdot/c, |drift| <= 2v/c, one sign change")
    {
    }

  private:
    void DoRun() override
    {
        auto sat2 = LeoSat(600.0);
        // The UE sits 1 deg off the ground track under the satellite's t = 240 s point.
        auto ue = Static(GroundUnder(sat2, 240.0, 1.0, 0.0));
        NtnRrcHelper h;
        h.SetPayloadMode(PayloadMode::RegenerativeLite);
        h.SetReferencePosition(ue->GetPosition());
        auto ta = h.InstallTimingAdvance(ue, sat2);
        uint32_t samples = 0;
        uint32_t badTa = 0;
        uint32_t badDrift = 0;
        uint32_t overBound = 0;
        int signChanges = 0;
        double prevDrift = 0.0;
        double maxAbs = 0.0;
        for (int t = 0; t <= 480; t += 2)
        {
            Simulator::Schedule(Seconds(t), [&]() {
                const Vector s = sat2->GetPosition();
                const Vector v = sat2->GetVelocity();
                const Vector u = ue->GetPosition();
                const Vector d(s.x - u.x, s.y - u.y, s.z - u.z);
                const double range = d.GetLength();
                ++samples;
                badTa += std::abs(ta->ComputeTotalTa().GetSeconds() - 2.0 * range / kC) > 2e-9;
                const double rdot = (d.x * v.x + d.y * v.y + d.z * v.z) / range;
                const double drift = ta->ComputeTaDriftRate(MilliSeconds(10));
                badDrift += std::abs(drift - 2.0 * rdot / kC) > 0.2e-6;
                overBound += std::abs(drift) > 2.0 * v.GetLength() / kC + 1e-9;
                if (samples > 1 && (drift > 0) != (prevDrift > 0))
                {
                    ++signChanges;
                }
                prevDrift = drift;
                maxAbs = std::max(maxAbs, std::abs(drift));
            });
        }
        Simulator::Stop(Seconds(481));
        Simulator::Run();
        Simulator::Destroy();
        NS_TEST_ASSERT_MSG_EQ(samples, 241u, "0..480 s every 2 s");
        NS_TEST_ASSERT_MSG_EQ(badTa, 0u, "TA = 2d/c to 2 ns at every sample");
        NS_TEST_ASSERT_MSG_EQ(badDrift, 0u, "drift = 2 rdot / c to 0.2 us/s");
        NS_TEST_ASSERT_MSG_EQ(overBound, 0u, "|drift| <= 2 v / c (about 50 us/s)");
        NS_TEST_ASSERT_MSG_EQ(signChanges, 1, "approach then recession");
        NS_TEST_ASSERT_MSG_GT(maxAbs, 20e-6, "LEO-600 drifts by tens of us per second");
        NS_TEST_ASSERT_MSG_EQ_TOL(sat2->Speed(), 7557.0, 5.0, "a 600 km circular orbit: 7.56 km/s");
    }
};

/// SIB19 over a packet link (packet level, with the internet and
/// point-to-point modules). A gNB node riding a LEO-600 satellite sends the
/// serialized SIB19 every 160 ms as a UDP datagram over a point-to-point link
/// whose delay is the live slant range / c, to a ground UE. The UE
/// consumer decodes every datagram, and:
///  - the measured one-way packet delay equals slant range / c (light-time) to
///    within 1 us plus serialization;
///  - while synced, the pre-compensated TA error stays within Te_NTN = 716.15 ns
///    (TS 38.133 Table 7.1C.2-1, FR1 30 kHz);
///  - a 2 s outage (every packet dropped by a receive error model) stops
///    decoding, the information age grows past 1.9 s, and decoding resumes after.
class Sib19OverRangeDelayLinkTest : public TestCase
{
  public:
    Sib19OverRangeDelayLinkTest()
        : TestCase("SIB19 as UDP over a slant-range-delay link: delay = d/c, error < Te_NTN, outage")
    {
    }

  private:
    struct Out
    {
        uint32_t sent{0};
        uint32_t received{0};
        uint32_t decodedBeforeOutage{0};
        uint32_t decodedAfterOutage{0};
        uint32_t decodedEnd{0};
        double maxDelayErrS{0.0};
        double maxSyncedErrNs{0.0};
        double maxAgeMs{0.0};
        uint32_t syncedSamples{0};
    };

    static Out Run(double gnssSigmaM)
    {
        RngSeedManager::ResetNextStreamIndex();
        Out o;
        Ptr<Node> gnb = CreateObject<Node>();
        Ptr<Node> ueNode = CreateObject<Node>();
        auto sat = LeoSat(600.0);
        gnb->AggregateObject(sat);
        auto ue = CreateObject<ConstantPositionMobilityModel>();
        ue->SetPosition(GroundUnder(sat, 4.0, 0.5, 0.5));
        ueNode->AggregateObject(ue);
        auto gw = Static(GroundUnder(sat, 4.0, -2.5, 0.0));

        // A point-to-point link whose propagation delay is re-set to the
        // live slant range / c before every transmission.
        PointToPointHelper p2p;
        p2p.SetDeviceAttribute("DataRate", StringValue("50Mbps"));
        p2p.SetChannelAttribute("Delay", TimeValue(MilliSeconds(2)));
        NetDeviceContainer devs = p2p.Install(gnb, ueNode);
        Ptr<PointToPointChannel> ch =
            DynamicCast<PointToPointChannel>(devs.Get(0)->GetChannel());
        InternetStackHelper ip;
        ip.Install(gnb);
        ip.Install(ueNode);
        Ipv4AddressHelper addr("10.9.0.0", "255.255.255.0");
        auto ifs = addr.Assign(devs);

        NtnRrcHelper h;
        h.SetPayloadMode(PayloadMode::Transparent);
        h.SetGatewayMobility(gw);
        h.SetReferencePosition(ue->GetPosition());
        auto ta = h.InstallTimingAdvance(ue, sat);
        auto bc = h.InstallSib19Broadcaster(sat, 7, ta, MilliSeconds(160));
        auto consumer = CreateObject<NtnUeSib19Consumer>();
        consumer->Setup(ue, ta, gnssSigmaM, Seconds(1));
        // Fixed streams: within one process ns-3 hands out stream numbers in
        // creation order, so repeatability needs them pinned (ns-3 manual,
        // "Random Variables": AssignStreams).
        consumer->GetPrecompensator()->AssignStreams(1000);

        Ptr<Socket> rx = Socket::CreateSocket(ueNode, UdpSocketFactory::GetTypeId());
        rx->Bind(InetSocketAddress(Ipv4Address::GetAny(), 9019));
        Ptr<Socket> tx = Socket::CreateSocket(gnb, UdpSocketFactory::GetTypeId());
        tx->Connect(InetSocketAddress(ifs.GetAddress(1), 9019));
        rx->SetRecvCallback([&o, consumer](Ptr<Socket> s) {
            Ptr<Packet> p;
            while ((p = s->Recv()))
            {
                std::vector<uint8_t> b(p->GetSize());
                p->CopyData(b.data(), b.size());
                ++o.received;
                consumer->ReceiveWire(b);
            }
        });
        for (int k = 0; k < 50; ++k) // 8 s of 160 ms broadcasts
        {
            Simulator::Schedule(MilliSeconds(160 * k + 1), [&o, bc, tx, ch, sat, ue]() {
                ch->SetAttribute("Delay",
                                 TimeValue(Seconds(CalculateDistance(sat->GetPosition(),
                                                                     ue->GetPosition()) /
                                                   kC)));
                bc->RefreshNow();
                const auto& bytes = bc->GetLatestSerialised();
                tx->Send(Create<Packet>(bytes.data(), bytes.size()));
                ++o.sent;
            });
        }
        // Outage 3..5 s: the UE receiver drops every packet.
        auto em = CreateObject<RateErrorModel>();
        em->SetUnit(RateErrorModel::ERROR_UNIT_PACKET);
        em->SetRate(1.0);
        Ptr<PointToPointNetDevice> ueDev = DynamicCast<PointToPointNetDevice>(devs.Get(1));
        Simulator::Schedule(Seconds(3.0), [ueDev, em, &o, consumer]() {
            o.decodedBeforeOutage = consumer->GetDecodedCount();
            ueDev->SetReceiveErrorModel(em);
        });
        Simulator::Schedule(Seconds(5.0), [ueDev, &o, consumer]() {
            o.decodedAfterOutage = consumer->GetDecodedCount();
            ueDev->SetReceiveErrorModel(nullptr);
        });
        for (int k = 1; k < 80; ++k)
        {
            Simulator::Schedule(MilliSeconds(100 * k + 50), [&o, consumer]() {
                const auto r = consumer->Sample();
                if (!r.valid)
                {
                    return;
                }
                o.maxAgeMs = std::max(o.maxAgeMs, r.ageMs);
                if (r.synced && r.ageMs < 200.0)
                {
                    ++o.syncedSamples;
                    o.maxSyncedErrNs = std::max(o.maxSyncedErrNs, std::abs(r.errNs));
                }
            });
        }
        // Independent delay check: the delay on the wire equals the slant range
        // over c at the instant of the last transmission (within 1 us).
        double chanVsGeom = 0.0;
        for (int k = 1; k < 8; ++k)
        {
            Simulator::Schedule(MilliSeconds(160 * (6 * k) + 1) + MicroSeconds(1),
                                [&chanVsGeom, ch, sat, ue]() {
                                    TimeValue d;
                                    ch->GetAttribute("Delay", d);
                                    const double geo =
                                        CalculateDistance(sat->GetPosition(), ue->GetPosition()) / kC;
                                    chanVsGeom = std::max(chanVsGeom, std::abs(d.Get().GetSeconds() - geo));
                                });
        }
        Simulator::Stop(Seconds(8.0));
        Simulator::Run();
        o.decodedEnd = consumer->GetDecodedCount();
        o.maxDelayErrS = chanVsGeom;
        Simulator::Destroy();
        return o;
    }

    void DoRun() override
    {
        const Out o = Run(0.0);
        NS_TEST_ASSERT_MSG_EQ(o.sent, 50u, "50 broadcasts");
        NS_TEST_ASSERT_MSG_GT(o.received, 30u, "datagrams delivered outside the outage");
        NS_TEST_ASSERT_MSG_LT(o.received, 45u, "and about 12 lost inside it");
        NS_TEST_ASSERT_MSG_LT(o.maxDelayErrS, 1e-6, "link delay = slant range / c within 1 us");
        NS_TEST_ASSERT_MSG_EQ(o.decodedAfterOutage, o.decodedBeforeOutage,
                              "nothing decoded during the outage");
        NS_TEST_ASSERT_MSG_GT(o.decodedEnd, o.decodedAfterOutage + 10, "decoding resumes");
        NS_TEST_ASSERT_MSG_GT(o.maxAgeMs, 1900.0, "the information ages through the outage");
        NS_TEST_ASSERT_MSG_GT(o.syncedSamples, 30u, "fresh, synced samples");
        NS_TEST_ASSERT_MSG_LT(o.maxSyncedErrNs, 716.15,
                              "UE TA error within Te_NTN (TS 38.133 Table 7.1C.2-1)");

        // Determinism with GNSS noise (3 m): same run, same worst error.
        RngSeedManager::SetRun(3);
        const Out a = Run(3.0);
        RngSeedManager::SetRun(3);
        const Out b = Run(3.0);
        RngSeedManager::SetRun(1);
        NS_TEST_ASSERT_MSG_EQ(a.maxSyncedErrNs, b.maxSyncedErrNs, "same RngRun, same error");
        NS_TEST_ASSERT_MSG_EQ(a.decodedEnd, b.decodedEnd, "same decodes");
        NS_TEST_ASSERT_MSG_LT(a.maxSyncedErrNs, 716.15 + 2.0 * 3.0 * 3.0 / kC * 1e9,
                              "3 m GNSS noise adds at most 2 x 3 sigma / c");
    }
};

/// Measurement report codec under hostile input and the RSRP/RSRQ/SINR
/// quantizers as properties over their whole range (TS 38.133 Tables
/// 10.1.6.1-1, 10.1.11.1-1, 10.1.16.1-1): monotone non-decreasing, the inverse
/// lands within one step below the input, saturating at both ends. The codec
/// rejects every truncation, more than 8 neighbors, and survives 3000 seeded
/// byte mutations.
class MeasReportPropertiesTest : public TestCase
{
  public:
    MeasReportPropertiesTest()
        : TestCase("TS 38.133 quantizers monotone over range; MeasurementReport truncation and fuzz")
    {
    }

  private:
    void DoRun() override
    {
        uint32_t bad = 0;
        uint8_t prev = 0;
        for (double x = -170.0; x <= -20.0; x += 0.05)
        {
            const uint8_t l = NtnMeasQuantity::RsrpToLevel(x);
            bad += l < prev;
            prev = l;
            if (x >= -156.0 && x < -30.0)
            {
                const double back = NtnMeasQuantity::LevelToRsrpDbm(l);
                bad += !(back <= x + 1e-9 && x < back + 1.0 + 1e-9);
            }
        }
        NS_TEST_ASSERT_MSG_EQ(bad, 0u, "RSRP: monotone, 1 dB steps, inverse is the bin floor");
        prev = 0;
        for (double x = -50.0; x <= 50.0; x += 0.05)
        {
            const uint8_t l = NtnMeasQuantity::SinrToLevel(x);
            bad += l < prev;
            prev = l;
        }
        prev = 0;
        for (double x = -50.0; x <= 30.0; x += 0.05)
        {
            const uint8_t l = NtnMeasQuantity::RsrqToLevel(x);
            bad += l < prev;
            prev = l;
        }
        NS_TEST_ASSERT_MSG_EQ(bad, 0u, "SINR and RSRQ monotone");

        NtnMeasurementReport rep;
        rep.measId = 3;
        rep.servingCell = {101, 60, 40, 70, true, true, true};
        for (uint16_t i = 0; i < 8; ++i)
        {
            rep.neighbours.push_back({static_cast<uint16_t>(200 + i), static_cast<uint8_t>(50 + i),
                                      0, 0, true, false, false});
        }
        rep.servingTimingAdvance = MicroSeconds(12890);
        rep.servingSlantRangeM = 1932e3;
        std::vector<uint8_t> buf(NtnMeasReportCodec::SerialisedBytes(8));
        const std::size_t n = NtnMeasReportCodec::Serialise(rep, buf.data(), buf.size());
        NS_TEST_ASSERT_MSG_EQ(n, buf.size(), "serialized");
        uint32_t accepted = 0;
        for (std::size_t k = 0; k < n; ++k)
        {
            NtnMeasurementReport r;
            accepted += NtnMeasReportCodec::Parse(buf.data(), k, r);
        }
        NS_TEST_ASSERT_MSG_EQ(accepted, 0u, "no truncation parses");
        NtnMeasurementReport back;
        NS_TEST_ASSERT_MSG_EQ(NtnMeasReportCodec::Parse(buf.data(), n, back), true, "full parses");
        NS_TEST_ASSERT_MSG_EQ(back.neighbours.size(), 8u, "8 neighbors");
        NS_TEST_ASSERT_MSG_EQ(back.servingTimingAdvance, rep.servingTimingAdvance, "TA intact");
        rep.neighbours.push_back(rep.neighbours.back());
        std::vector<uint8_t> big(NtnMeasReportCodec::SerialisedBytes(9) + 16);
        NS_TEST_ASSERT_MSG_EQ(NtnMeasReportCodec::Serialise(rep, big.data(), big.size()), 0u,
                              "9 neighbors exceed maxReportCells (8) and are refused");
        uint64_t lcg = 99;
        for (int k = 0; k < 3000; ++k)
        {
            std::vector<uint8_t> m = buf;
            lcg = lcg * 6364136223846793005ULL + 1442695040888963407ULL;
            m[(lcg >> 33) % m.size()] ^= static_cast<uint8_t>(1u << ((lcg >> 20) % 8));
            NtnMeasurementReport r;
            if (NtnMeasReportCodec::Parse(m.data(), m.size(), r))
            {
                bad += r.neighbours.size() > NtnMeasurementReport::kMaxNeighbours;
            }
        }
        NS_TEST_ASSERT_MSG_EQ(bad, 0u, "mutated reports never exceed 8 neighbors");
    }
};

/// The UE location reporter under real motion (TS 38.331 Rel-17 location
/// reporting). A UE driving at 30 m/s for 60 s with a 100 m event trigger
/// produces floor(1800 / 100) reports (plus the first), each at least 100 m
/// from the previous; periodic reporting at 2 s produces 30; with 5 m GNSS
/// noise the scatter of the reported positions around the truth has a
/// standard deviation near 5 m, and two identical runs report identically.
class LocationReporterUnderMotionTest : public TestCase
{
  public:
    LocationReporterUnderMotionTest()
        : TestCase("Location reporter: event trigger at 100 m on a 30 m/s UE, periodic count, noise")
    {
    }

  private:
    struct Out
    {
        uint32_t reports{0};
        double minStepM{1e9};
        double rmsErrM{0.0};
        uint32_t lastSeq{0};
    };

    static Out Run(LocationReportMode mode, double accuracy)
    {
        RngSeedManager::ResetNextStreamIndex();
        Out o;
        auto ue = CreateObject<ConstantVelocityMobilityModel>();
        const Vector start = GeodeticWgs84ToEcef(33.68, 73.05, 500.0);
        ue->SetPosition(start);
        // 30 m/s due east: the local east unit vector at the start point.
        const double lon = 73.05 * M_PI / 180.0;
        ue->SetVelocity(Vector(-30.0 * std::sin(lon), 30.0 * std::cos(lon), 0.0));
        auto rep = CreateObject<NtnUeLocationReporter>();
        rep->SetUeMobility(ue);
        rep->SetReportingMode(mode);
        rep->SetReportingPeriod(mode == LocationReportMode::Periodic ? Seconds(2) : MilliSeconds(100));
        rep->SetEventTriggerDistanceMetres(100.0);
        rep->SetAccuracyMetres(accuracy);
        Vector last;
        bool have = false;
        double sumSq = 0.0;
        uint32_t lastSeq = 0;
        std::function<void()> poll = [&]() {
            if (rep->HasReport() && rep->GetLatestReport().reportSequence != lastSeq)
            {
                const auto& r = rep->GetLatestReport();
                lastSeq = r.reportSequence;
                const Vector p = GeodeticWgs84ToEcef(r.latDeg, r.lonDeg, r.altMetres);
                ++o.reports;
                if (have)
                {
                    o.minStepM = std::min(o.minStepM, CalculateDistance(p, last));
                }
                last = p;
                have = true;
                const double e = CalculateDistance(p, ue->GetPosition());
                sumSq += e * e;
            }
            Simulator::Schedule(MilliSeconds(50), poll);
        };
        rep->Start();
        Simulator::Schedule(MilliSeconds(25), poll);
        Simulator::Stop(Seconds(60.01));
        Simulator::Run();
        o.rmsErrM = o.reports ? std::sqrt(sumSq / o.reports) : 0.0;
        o.lastSeq = lastSeq;
        Simulator::Destroy();
        return o;
    }

    void DoRun() override
    {
        const Out ev = Run(LocationReportMode::EventTriggered, 0.0);
        NS_TEST_ASSERT_MSG_EQ_TOL(ev.reports * 1.0, 19.0, 1.0, "1800 m / 100 m + the first report");
        NS_TEST_ASSERT_MSG_GT(ev.minStepM, 99.0, "no report closer than the trigger distance");
        NS_TEST_ASSERT_MSG_LT(ev.rmsErrM, 3.1, "noise-free reports within one 100 ms tick of motion");
        const Out per = Run(LocationReportMode::Periodic, 0.0);
        NS_TEST_ASSERT_MSG_EQ_TOL(per.reports * 1.0, 30.0, 1.0, "60 s / 2 s");
        RngSeedManager::SetRun(4);
        const Out n1 = Run(LocationReportMode::Periodic, 5.0);
        RngSeedManager::SetRun(4);
        const Out n2 = Run(LocationReportMode::Periodic, 5.0);
        RngSeedManager::SetRun(1);
        NS_TEST_ASSERT_MSG_EQ(n1.rmsErrM, n2.rmsErrM, "same RngRun, same reports");
        NS_TEST_ASSERT_MSG_GT(n1.rmsErrM, 2.0, "noise is applied");
        NS_TEST_ASSERT_MSG_LT(n1.rmsErrM, 12.0, "3D scatter of a 5 m sigma stays near 5 to 9 m");
    }
};

/// RACH window verdict along the SGP4 pass (TS 38.321 section 5.1.4): with the
/// Rel-17 window start offset equal to the UE-gNB round trip the window always
/// fits; without it the window must cover the whole round trip, which a
/// low-elevation LEO round trip (> 8 ms at 30 kHz SCS) and any GEO one exceed.
class RachWindowAlongPassTest : public TestCase
{
  public:
    RachWindowAlongPassTest()
        : TestCase("TS 38.321 5.1.4: RAR window along a LEO pass, with and without the Rel-17 offset")
    {
    }

  private:
    void DoRun() override
    {
        auto sat = LeoSat(600.0);
        auto ue = Static(GroundUnder(sat, 60.0, 1.0, 1.0));
        uint32_t withFits = 0;
        uint32_t withoutFits = 0;
        uint32_t mismatch = 0;
        Time maxRtt;
        uint32_t samples = 0;
        const Time slot = NtnRachWindow::SlotPeriodForNumerology(1);
        for (int t = 0; t <= 600; t += 10)
        {
            Simulator::Schedule(Seconds(t), [&]() {
                const double d = CalculateDistance(sat->GetPosition(), ue->GetPosition());
                const Time rtt = NtnRachWindow::RoundTripForSlantRange(d);
                withFits += NtnRachWindow::Evaluate(rtt, slot, Time(), rtt).fits;
                const bool fitsWithout = NtnRachWindow::Evaluate(rtt, slot).fits;
                withoutFits += fitsWithout;
                // Without the offset the window, at most (6 + 10) slots = 8 ms at
                // 30 kHz, must cover the whole round trip plus one slot.
                mismatch += fitsWithout != (rtt + slot <= slot * 16);
                maxRtt = std::max(maxRtt, rtt);
                ++samples;
            });
        }
        Simulator::Stop(Seconds(601));
        Simulator::Run();
        Simulator::Destroy();
        NS_TEST_ASSERT_MSG_EQ(slot, MicroSeconds(500), "30 kHz slot is 0.5 ms (TS 38.211 4.3.2)");
        NS_TEST_ASSERT_MSG_EQ(withFits, samples, "with the offset the window always fits");
        NS_TEST_ASSERT_MSG_EQ(mismatch, 0u, "without it, fits iff RTT + 1 slot <= 8 ms");
        NS_TEST_ASSERT_MSG_GT(maxRtt, MilliSeconds(8), "the pass reaches low elevations");
        NS_TEST_ASSERT_MSG_LT(withoutFits, samples, "where the window no longer covers the RTT");
        const Time geo = NtnRachWindow::RoundTripForSlantRange(35786e3);
        NS_TEST_ASSERT_MSG_EQ_TOL(geo.GetSeconds(), 2 * 35786e3 / kC, 1e-9, "GEO nadir RTT");
        NS_TEST_ASSERT_MSG_EQ(NtnRachWindow::Evaluate(geo, slot).fits, false, "GEO without offset");
        NS_TEST_ASSERT_MSG_EQ(NtnRachWindow::Evaluate(geo, slot, Time(), geo + MilliSeconds(5)).fits,
                              false, "an offset past the RAR arrival misses it");
    }
};

/// SIB19 codec against corruption (failure path). The wire format has no CRC,
/// so Parse must refuse what TS 38.331 SIB19-r17 cannot carry: a K_offset of 0
/// or above 1023, kMac above 512, an unknown payload mode or TA frame, a
/// non-finite ephemeris, a validity of 0 or above 900 s. A broadcaster's own
/// block parses; 3000 seeded single-bit flips either fail to parse or decode
/// to a block inside every range.
class Sib19CodecCorruptionTest : public TestCase
{
  public:
    Sib19CodecCorruptionTest()
        : TestCase("SIB19 codec: out-of-range and corrupted blocks refused (TS 38.331 SIB19-r17)")
    {
    }

  private:
    void DoRun() override
    {
        auto sat = LeoSat(600.0);
        auto ue = Static(GroundUnder(sat, 0.0, 0.3, 0.3));
        NtnRrcHelper h;
        h.SetPayloadMode(PayloadMode::Transparent);
        h.SetGatewayMobility(Static(GroundUnder(sat, 0.0, -2.0, 0.0)));
        h.SetReferencePosition(ue->GetPosition());
        auto ta = h.InstallTimingAdvance(ue, sat);
        auto bc = h.InstallSib19Broadcaster(sat, 9, ta, MilliSeconds(160));
        bc->RefreshNow();
        const std::vector<uint8_t> good = bc->GetLatestSerialised();
        Sib19Content c;
        NS_TEST_ASSERT_MSG_EQ(Sib19Codec::Parse(good.data(), good.size(), c), true,
                              "the broadcaster's block parses");
        auto reject = [&](std::function<void(Sib19Content&)> spoil) {
            Sib19Content x = bc->GetLatest();
            spoil(x);
            std::vector<uint8_t> b(Sib19Codec::kSerialisedBytes);
            Sib19Codec::Serialise(x, b.data(), b.size());
            Sib19Content y;
            return !Sib19Codec::Parse(b.data(), b.size(), y);
        };
        NS_TEST_ASSERT_MSG_EQ(reject([](Sib19Content& x) { x.cellSpecificKoffset = 0; }), true,
                              "K_offset 0 (range 1..1023)");
        NS_TEST_ASSERT_MSG_EQ(reject([](Sib19Content& x) { x.cellSpecificKoffset = 1024; }), true,
                              "K_offset 1024");
        NS_TEST_ASSERT_MSG_EQ(reject([](Sib19Content& x) { x.kMac = 513; }), true, "kMac 513");
        NS_TEST_ASSERT_MSG_EQ(reject([](Sib19Content& x) { x.payloadMode = static_cast<PayloadMode>(7); }),
                              true, "unknown payload mode");
        NS_TEST_ASSERT_MSG_EQ(reject([](Sib19Content& x) { x.ephemeris.positionEcefM.x = std::nan(""); }),
                              true, "NaN ephemeris");
        NS_TEST_ASSERT_MSG_EQ(reject([](Sib19Content& x) { x.ulSyncValidity = Seconds(901); }), true,
                              "validity beyond s900");
        NS_TEST_ASSERT_MSG_EQ(reject([](Sib19Content& x) { x.taCommon = Seconds(-1); }), true,
                              "negative common TA");
        uint64_t lcg = 7;
        uint32_t bad = 0;
        for (int k = 0; k < 3000; ++k)
        {
            std::vector<uint8_t> m = good;
            lcg = lcg * 6364136223846793005ULL + 1442695040888963407ULL;
            m[(lcg >> 33) % m.size()] ^= static_cast<uint8_t>(1u << ((lcg >> 20) % 8));
            Sib19Content y;
            if (Sib19Codec::Parse(m.data(), m.size(), y))
            {
                bad += !(y.cellSpecificKoffset >= 1 && y.cellSpecificKoffset <= 1023 && y.kMac <= 512 &&
                         std::isfinite(y.ephemeris.positionEcefM.x) &&
                         y.ulSyncValidity <= Seconds(900));
            }
        }
        NS_TEST_ASSERT_MSG_EQ(bad, 0u, "a decoded block is always inside the TS 38.331 ranges");
        NS_TEST_ASSERT_MSG_EQ(Sib19Codec::Parse(good.data(), good.size() - 1, c), false, "truncated");
    }
};

/// ntn-UlSyncValidityDuration (TS 38.331 SIB19-r17, TS 38.213 4.2): after the
/// last SIB19 the UE may keep pre-compensating for the validity duration and
/// must stop being uplink-synchronized after it. One SIB19 at 1 s, then the
/// link is silent: synced just before 1 s + validity, not synced just after.
class UlSyncValidityExpiryTest : public TestCase
{
  public:
    UlSyncValidityExpiryTest()
        : TestCase("TS 38.331 ntn-UlSyncValidityDuration: the UE is out of UL sync once it lapses")
    {
    }

  private:
    void DoRun() override
    {
        auto sat = LeoSat(600.0);
        auto ue = Static(GroundUnder(sat, 1.0, 0.3, 0.3));
        NtnRrcHelper h;
        h.SetPayloadMode(PayloadMode::Transparent);
        h.SetGatewayMobility(Static(GroundUnder(sat, 1.0, -2.0, 0.0)));
        h.SetReferencePosition(ue->GetPosition());
        auto ta = h.InstallTimingAdvance(ue, sat);
        auto bc = h.InstallSib19Broadcaster(sat, 9, ta, MilliSeconds(160));
        // The test orbit is two-body in a non-rotating frame, so the UE
        // propagates with two-body gravity and no frame terms (matching truth),
        // and the broadcaster sizes the validity for that model.
        bc->SetAssumedUeModel(UeEphemerisModel::GravityEcef);
        // ErrorBudget: the largest enumerated validity whose predicted timing
        // error still fits Te_NTN (the legacy SlotOverDrift rule allows a slot).
        bc->SetValidityRule(Sib19ValidityRule::ErrorBudget);
        auto c = CreateObject<NtnUeSib19Consumer>();
        c->Setup(ue, ta);
        c->GetPrecompensator()->SetEphemerisModel(UeEphemerisModel::GravityEcef);
        Time validity;
        bool before = false;
        bool after = true;
        double errBefore = 0.0;
        Simulator::Schedule(Seconds(1.0), [&]() {
            bc->RefreshNow();
            c->ReceiveWire(bc->GetLatestSerialised());
            validity = c->GetHeld().ulSyncValidity;
            Simulator::Schedule(validity - MilliSeconds(100), [&]() {
                const auto r = c->Sample();
                before = r.synced;
                errBefore = r.errNs;
            });
            Simulator::Schedule(validity + MilliSeconds(100), [&]() { after = c->Sample().synced; });
        });
        Simulator::Stop(Seconds(1000));
        Simulator::Run();
        Simulator::Destroy();
        NS_TEST_ASSERT_MSG_GT(validity, Seconds(0), "the broadcaster set a validity");
        NS_TEST_ASSERT_MSG_LT(validity, Seconds(900) + NanoSeconds(1), "within s900");
        const std::vector<int> allowed = {5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 120, 180, 240, 900};
        const bool enumerated =
            std::count(allowed.begin(), allowed.end(),
                       static_cast<int>(std::lround(validity.GetSeconds()))) == 1 &&
            std::abs(validity.GetSeconds() - std::lround(validity.GetSeconds())) < 1e-9;
        NS_TEST_ASSERT_MSG_EQ(enumerated, true, "an ntn-UlSyncValidityDuration-r17 enumerated value");
        NS_TEST_ASSERT_MSG_EQ(before, true, "synced until the validity lapses");
        NS_TEST_ASSERT_MSG_EQ(after, false, "out of UL sync after it");
        // Measured: 719 ns at validity - 100 ms, i.e. the budget is spent to the
        // last nanoseconds; the TA values are quantized to the 1 ns time
        // resolution on both sides, hence the 5 ns allowance.
        NS_TEST_ASSERT_MSG_LT(std::abs(errBefore), 716.15 + 5.0,
                              "the ErrorBudget validity keeps the error within Te_NTN to the end");
        NS_TEST_ASSERT_MSG_GT(std::abs(errBefore), 0.5 * 716.15,
                              "and uses most of it (the validity is not needlessly short)");
    }
};

class NtnRrcDepthTestSuite : public TestSuite
{
  public:
    NtnRrcDepthTestSuite()
        : TestSuite("ntn-rrc-depth", Type::UNIT)
    {
        AddTestCase(new TaMaxRoundTripTr38821Test, TestCase::Duration::QUICK);
        AddTestCase(new TaAlongLeoPassTest, TestCase::Duration::QUICK);
        AddTestCase(new Sib19OverRangeDelayLinkTest, TestCase::Duration::QUICK);
        AddTestCase(new MeasReportPropertiesTest, TestCase::Duration::QUICK);
        AddTestCase(new LocationReporterUnderMotionTest, TestCase::Duration::QUICK);
        AddTestCase(new RachWindowAlongPassTest, TestCase::Duration::QUICK);
        AddTestCase(new Sib19CodecCorruptionTest, TestCase::Duration::QUICK);
        AddTestCase(new UlSyncValidityExpiryTest, TestCase::Duration::QUICK);
    }
};

static NtnRrcDepthTestSuite g_ntnRrcDepthTestSuite;
