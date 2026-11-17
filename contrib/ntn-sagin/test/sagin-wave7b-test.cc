/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only
//
// Wave 7B tests:
//  - RFC 9171 4.2.1 block CRCs: CRC-16 X.25 and CRC-32C check values, the
//    wire format, VerifyBundle on clean and corrupted bundles;
//  - corruption over a real point-to-point link (SaginBitErrorModel flips
//    bits in the UDP payload), every corrupted bundle detected, and the
//    failure case without CRCs where corruption goes through undetected;
//  - the custody-queue integrity check;
//  - CCSDS 734.3-B-1 SABR priority levels (3.2.6.8), applicable backlog per
//    priority (3.2.6.2) and critical bundles (3.2.6.9.2, 3.2.8.1.3).

#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/network-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/sagin-bit-error-model.h"
#include "ns3/sagin-bundle-protocol.h"
#include "ns3/sagin-custody-queue.h"
#include "ns3/sagin-sabr-router.h"
#include "ns3/test.h"

#include <cstring>
#include <map>

using namespace ns3;
using namespace ns3::ntnsagin;

namespace
{
constexpr uint32_t kPayload = 200;

uint8_t
Pattern(uint32_t i)
{
    return static_cast<uint8_t>((i * 37 + 11) & 0xFF);
}

Ptr<Packet>
MakeCrcBundle(uint8_t crcType, uint64_t seq, bool age = false, bool cos = false)
{
    std::vector<uint8_t> pl(kPayload);
    for (uint32_t i = 0; i < kPayload; ++i)
    {
        pl[i] = Pattern(i + seq);
    }
    Ptr<Packet> p = Create<Packet>(pl.data(), kPayload);
    SaginBundleHeader h;
    h.crcType = crcType;
    h.srcNode = 5;
    h.destNode = 9000;
    h.destService = 1;
    h.reportNode = 5;
    h.creationMs = 820000000000ULL + seq;
    h.creationSeq = seq;
    h.lifetimeMs = 3600000;
    h.payloadLength = kPayload;
    if (age)
    {
        h.hasAgeBlock = true;
        h.ageMs = 1234;
    }
    if (cos)
    {
        h.hasCosBlock = true;
        h.priority = kBpPriorityExpedited;
        h.critical = true;
    }
    p->AddHeader(h);
    p->AddTrailer(h.MakeTrailer());
    return p;
}

Ptr<Packet>
FlipBit(Ptr<const Packet> p, uint32_t bit)
{
    std::vector<uint8_t> b(p->GetSize());
    p->CopyData(b.data(), b.size());
    b[bit / 8] ^= static_cast<uint8_t>(0x80 >> (bit % 8));
    return Create<Packet>(b.data(), b.size());
}
} // namespace

// ---------------------------------------------------------------------------
class SaginBpv7CrcWireTest : public TestCase
{
  public:
    SaginBpv7CrcWireTest()
        : TestCase("7B: RFC 9171 CRC-16 X.25 / CRC-32C check values, wire format, VerifyBundle")
    {
    }

  private:
    void DoRun() override
    {
        const uint8_t check[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
        NS_TEST_ASSERT_MSG_EQ(Crc16X25(check, 9), 0x906E, "CRC-16 X.25 check value");
        NS_TEST_ASSERT_MSG_EQ(Crc32c(check, 9), 0xE3069283u, "CRC-32C check value");

        for (uint8_t ct : {kBpCrc16, kBpCrc32c})
        {
            Ptr<Packet> p = MakeCrcBundle(ct, 7, true, true);
            uint8_t head[2];
            p->CopyData(head, 2);
            NS_TEST_ASSERT_MSG_EQ(+head[1], 0x89, "primary block array(9) when it carries a CRC");
            auto r = VerifyBundle(p);
            NS_TEST_ASSERT_MSG_EQ(BundleCheckName(r.status), std::string("ok"), "clean bundle");
            NS_TEST_ASSERT_MSG_EQ(r.blocksChecked, 4u, "primary, age, class of service, payload");
            // The primary CRC is the last 2/4 bytes of the primary block, computed
            // with the field zeroed: recompute it from the wire bytes.
            SaginBundleHeader h;
            Ptr<Packet> q = p->Copy();
            q->RemoveHeader(h);
            NS_TEST_ASSERT_MSG_EQ(+h.crcType, +ct, "CRC type read back");
            NS_TEST_ASSERT_MSG_EQ(h.ageMs, 1234u, "age block read back");
            NS_TEST_ASSERT_MSG_EQ(+h.priority, +kBpPriorityExpedited, "priority read back");
            NS_TEST_ASSERT_MSG_EQ(h.critical, true, "critical flag read back");
            SaginBundleTrailer t = h.MakeTrailer();
            q->RemoveTrailer(t);
            NS_TEST_ASSERT_MSG_EQ(q->GetSize(), kPayload, "only the payload remains");
            uint8_t pl[kPayload];
            q->CopyData(pl, kPayload);
            NS_TEST_ASSERT_MSG_EQ(+pl[3], +Pattern(3 + 7), "payload intact");
            // Payload-block CRC: CRC over [array head .. payload .. crc bstr zeroed].
            std::vector<uint8_t> all(p->GetSize());
            p->CopyData(all.data(), all.size());
            const uint32_t n = ct == kBpCrc16 ? 2 : 4;
            const size_t crcAt = all.size() - 1 - n;
            const size_t blockStart = crcAt - 1 - kPayload - 2 /*bstr head 0x58 0xC8*/ - 5;
            std::vector<uint8_t> blk(all.begin() + blockStart, all.begin() + crcAt);
            blk.insert(blk.end(), n, 0);
            const uint32_t want = ct == kBpCrc16 ? Crc16X25(blk.data(), blk.size())
                                                 : Crc32c(blk.data(), blk.size());
            uint32_t got = 0;
            for (uint32_t i = 0; i < n; ++i)
            {
                got = (got << 8) | all[crcAt + i];
            }
            NS_TEST_ASSERT_MSG_EQ(got, want, "payload CRC big-endian over the zero-filled block");
            NS_TEST_ASSERT_MSG_EQ(t.crcValue, want, "trailer reads the CRC back");

            // Failure paths: a bit flipped in each block is caught by that block's CRC.
            auto rp = VerifyBundle(FlipBit(p, 8 * 20 + 3)); // inside the primary block
            NS_TEST_ASSERT_MSG_EQ(BundleCheckName(rp.status), std::string("primary-crc"),
                                  "primary corruption detected");
            auto rl = VerifyBundle(FlipBit(p, 8 * (all.size() - 1 - n - 50)));
            NS_TEST_ASSERT_MSG_EQ(BundleCheckName(rl.status), std::string("canonical-crc"),
                                  "payload corruption detected");
            NS_TEST_ASSERT_MSG_EQ(rl.failedBlockType, 1u, "the payload block failed");
            auto rb = VerifyBundle(FlipBit(p, 8 * (all.size() - 1)));
            NS_TEST_ASSERT_MSG_EQ(BundleCheckName(rb.status), std::string("malformed"),
                                  "a broken CBOR break is malformed");
            Ptr<Packet> trunc = p->CreateFragment(0, p->GetSize() - 4);
            NS_TEST_ASSERT_MSG_EQ(BundleCheckName(VerifyBundle(trunc).status),
                                  std::string("malformed"), "truncated bundle");
        }
        // Age block corruption: locate the age value byte after the primary block.
        {
            Ptr<Packet> p = MakeCrcBundle(kBpCrc32c, 1, true, false);
            Ptr<Packet> clean = MakeCrcBundle(kBpCrc32c, 1, false, false);
            // The age block sits right after the primary block; the primary is the
            // same length in both, so the first differing byte starts the age block.
            std::vector<uint8_t> a(p->GetSize()), b(clean->GetSize());
            p->CopyData(a.data(), a.size());
            clean->CopyData(b.data(), b.size());
            size_t k = 0;
            while (a[k] == b[k])
            {
                ++k;
            }
            auto r = VerifyBundle(FlipBit(p, 8 * (k + 7)));
            NS_TEST_ASSERT_MSG_EQ(BundleCheckName(r.status), std::string("canonical-crc"),
                                  "age block corruption detected");
            NS_TEST_ASSERT_MSG_EQ(r.failedBlockType, 7u, "the Bundle Age block failed");
        }
        // CRC type 0: parses, but the primary block is flagged unprotected and a
        // payload bit error goes through.
        {
            Ptr<Packet> p = MakeCrcBundle(kBpCrcNone, 2);
            uint8_t head[2];
            p->CopyData(head, 2);
            NS_TEST_ASSERT_MSG_EQ(+head[1], 0x88, "array(8) without a CRC (pre-7B format)");
            auto r = VerifyBundle(FlipBit(p, 8 * (p->GetSize() - 20)));
            NS_TEST_ASSERT_MSG_EQ(BundleCheckName(r.status), std::string("ok"),
                                  "no CRC, so corruption is invisible");
            NS_TEST_ASSERT_MSG_EQ(r.primaryUnprotected, true, "flagged as unprotected");
        }
    }
};

// ---------------------------------------------------------------------------
class SaginCrcOverLinkTest : public TestCase
{
  public:
    SaginCrcOverLinkTest(uint8_t crcType)
        : TestCase(std::string("7B: bit errors on a P2P link, bundles verified on receive, CRC type ") +
                   std::to_string(crcType)),
          m_crcType(crcType)
    {
    }

  private:
    uint8_t m_crcType;
    uint32_t m_rx{0}, m_ok{0}, m_bad{0}, m_undetected{0};

    void Receive(Ptr<Socket> s)
    {
        Ptr<Packet> p;
        while ((p = s->Recv()))
        {
            ++m_rx;
            if (VerifyBundle(p).status != BundleCheck::Ok)
            {
                ++m_bad;
                continue;
            }
            ++m_ok;
            SaginBundleHeader h;
            p->RemoveHeader(h);
            SaginBundleTrailer t = h.MakeTrailer();
            p->RemoveTrailer(t);
            uint8_t pl[kPayload];
            p->CopyData(pl, kPayload);
            for (uint32_t i = 0; i < kPayload; ++i)
            {
                if (pl[i] != Pattern(i + h.creationSeq))
                {
                    ++m_undetected;
                    break;
                }
            }
        }
    }

    void DoRun() override
    {
        RngSeedManager::SetSeed(7);
        NodeContainer n;
        n.Create(2);
        PointToPointHelper p2p;
        p2p.SetDeviceAttribute("DataRate", StringValue("1Mbps"));
        p2p.SetChannelAttribute("Delay", StringValue("4ms"));
        NetDeviceContainer d = p2p.Install(n);
        InternetStackHelper inet;
        inet.Install(n);
        Ipv4AddressHelper ip;
        ip.SetBase("10.1.1.0", "255.255.255.0");
        Ipv4InterfaceContainer ic = ip.Assign(d);
        Ptr<SaginBitErrorModel> em = CreateObject<SaginBitErrorModel>();
        em->SetAttribute("BitErrorRate", DoubleValue(2e-4));
        em->SetAttribute("UdpDstPort", UintegerValue(9000));
        em->AssignStreams(3);
        d.Get(1)->SetAttribute("ReceiveErrorModel", PointerValue(em));

        Ptr<Socket> rx = Socket::CreateSocket(n.Get(1), UdpSocketFactory::GetTypeId());
        rx->Bind(InetSocketAddress(Ipv4Address::GetAny(), 9000));
        rx->SetRecvCallback(MakeCallback(&SaginCrcOverLinkTest::Receive, this));
        Ptr<Socket> tx = Socket::CreateSocket(n.Get(0), UdpSocketFactory::GetTypeId());
        tx->Bind();
        const uint32_t N = 400;
        for (uint32_t i = 0; i < N; ++i)
        {
            Simulator::Schedule(MilliSeconds(10 * i), [this, tx, ic, i]() {
                tx->SendTo(MakeCrcBundle(m_crcType, i), 0, InetSocketAddress(ic.GetAddress(1), 9000));
            });
        }
        Simulator::Stop(Seconds(10));
        Simulator::Run();
        Simulator::Destroy();

        NS_TEST_ASSERT_MSG_EQ(m_rx, N, "bit errors corrupt, they do not drop");
        NS_TEST_ASSERT_MSG_GT(em->GetPacketsCorrupted(), 20u, "the link did corrupt bundles");
        if (m_crcType != kBpCrcNone)
        {
            NS_TEST_ASSERT_MSG_EQ(m_bad, em->GetPacketsCorrupted(), "every corrupted bundle discarded");
            NS_TEST_ASSERT_MSG_EQ(m_undetected, 0u, "no corrupted payload accepted");
            NS_TEST_ASSERT_MSG_EQ(m_ok + m_bad, N, "accounting closes");
        }
        else
        {
            // Failure path: without CRCs corrupted payloads are delivered as good.
            NS_TEST_ASSERT_MSG_GT(m_undetected, 0u, "corruption goes through undetected");
        }
    }
};

// ---------------------------------------------------------------------------
class SaginCustodyIntegrityTest : public TestCase
{
  public:
    SaginCustodyIntegrityTest()
        : TestCase("7B: custody queue discards a bundle whose CRC fails and counts it")
    {
    }

  private:
    void DoRun() override
    {
        Ptr<SaginCustodyQueue> q = CreateObject<SaginCustodyQueue>();
        q->SetIntegrityCheck(SaginCustodyQueue::Bpv7CrcCheck());
        uint32_t fwd = 0;
        q->SetForwardCallback([&fwd](Ptr<Packet>) {
            ++fwd;
            return true;
        });
        bool good = false, bad = true;
        Simulator::Schedule(Seconds(1), [&]() {
            q->SetContactUp(true);
            good = q->Offer(MakeCrcBundle(kBpCrc16, 1));
            bad = q->Offer(FlipBit(MakeCrcBundle(kBpCrc16, 2), 8 * 60));
        });
        Simulator::Run();
        Simulator::Destroy();
        NS_TEST_ASSERT_MSG_EQ(good, true, "clean bundle forwarded");
        NS_TEST_ASSERT_MSG_EQ(bad, false, "corrupt bundle refused");
        NS_TEST_ASSERT_MSG_EQ(fwd, 1u, "only the clean one went on");
        NS_TEST_ASSERT_MSG_EQ(q->GetCorrupt(), 1u, "counted");
        NS_TEST_ASSERT_MSG_EQ(q->GetInCustody(), 0u, "not stored");
    }
};

// ---------------------------------------------------------------------------
namespace
{
// X=1, A=2, B=3, D=9. Via A from t=0, via B from t=10 s (so A is preferred).
Ptr<SaginSabrRouter>
MakeRouter()
{
    Ptr<SaginSabrRouter> r = CreateObject<SaginSabrRouter>();
    r->SetLocalNode(1);
    r->SetContactPlan({{1, 2, 0.0, 100.0, 1000.0, 0.0},
                       {2, 9, 0.0, 1000.0, 1e6, 0.0},
                       {1, 3, 10.0, 400.0, 1000.0, 0.0},
                       {3, 9, 0.0, 1000.0, 1e6, 0.0}});
    return r;
}
} // namespace

class SaginSabrPriorityTest : public TestCase
{
  public:
    SaginSabrPriorityTest()
        : TestCase("7B: SABR MTV per priority (CCSDS 734.3-B-1 3.2.6.8)")
    {
    }

  private:
    void DoRun() override
    {
        Ptr<SaginSabrRouter> r = MakeRouter();
        std::vector<uint32_t> entry;
        std::vector<double> mtv;
        Simulator::Schedule(Seconds(0), [&]() {
            auto go = [&](double evc, uint8_t prio) {
                auto d = r->Route(9, Simulator::Now().GetSeconds(), evc, 1e9, {}, true, true, prio);
                entry.push_back(d.found ? d.entryNode : 0);
            };
            go(60000, SABR_BULK);      // via A; bulk MTV of A 40000
            mtv.push_back(r->GetMtvBits(0, SABR_BULK));
            mtv.push_back(r->GetMtvBits(0, SABR_NORMAL));
            go(60000, SABR_BULK);      // A has 40000 bulk left: via B
            go(60000, SABR_NORMAL);    // normal still sees 100000: via A
            go(90000, SABR_EXPEDITED); // expedited sees 100000: via A
            mtv.push_back(r->GetMtvBits(0, SABR_BULK));
            mtv.push_back(r->GetMtvBits(0, SABR_NORMAL));
            mtv.push_back(r->GetMtvBits(0, SABR_EXPEDITED));
            go(1000, SABR_NORMAL);     // A depleted for normal: via B
            go(5000, SABR_EXPEDITED);  // 10000 expedited left on A
            go(5000, 7);               // out of range: treated as expedited, 5000 left
            go(6000, SABR_EXPEDITED);  // failure: A has 0, B still open
        });
        Simulator::Run();
        Simulator::Destroy();
        const std::vector<uint32_t> want{2, 3, 2, 2, 3, 2, 2, 3};
        NS_TEST_ASSERT_MSG_EQ(entry.size(), want.size(), "decisions");
        for (size_t i = 0; i < want.size(); ++i)
        {
            NS_TEST_ASSERT_MSG_EQ(entry[i], want[i], "entry node of decision " << i);
        }
        NS_TEST_ASSERT_MSG_EQ_TOL(mtv[0], 40000.0, 1e-6, "bulk consumed bulk");
        NS_TEST_ASSERT_MSG_EQ_TOL(mtv[1], 100000.0, 1e-6, "bulk does not consume normal");
        NS_TEST_ASSERT_MSG_EQ_TOL(mtv[2], 0.0, 1e-6, "bulk floored at 0");
        NS_TEST_ASSERT_MSG_EQ_TOL(mtv[3], 0.0, 1e-6, "normal: 100000 - 60000 - 90000 floored");
        NS_TEST_ASSERT_MSG_EQ_TOL(mtv[4], 10000.0, 1e-6, "expedited: 100000 - 90000");
    }
};

class SaginSabrBacklogTest : public TestCase
{
  public:
    SaginSabrBacklogTest()
        : TestCase("7B: SABR applicable backlog by priority (3.2.6.2)")
    {
    }

  private:
    void DoRun() override
    {
        Ptr<SaginSabrRouter> r = MakeRouter();
        // queued bits per neighbor per priority
        std::map<uint32_t, std::array<double, 3>> q;
        r->SetPriorityBacklogFunction([&q](uint32_t nb, uint8_t prio) {
            double s = 0;
            for (uint8_t p = prio; p <= 2; ++p)
            {
                s += q[nb][p];
            }
            return s;
        });
        std::vector<SabrDecision> d;
        Simulator::Schedule(Seconds(0), [&]() {
            q[2] = {95000, 0, 0};
            d.push_back(r->Route(9, 0, 1000, 1e9, {}, true, false, SABR_NORMAL)); // bulk ignored
            // bulk sees its 95000 bits: ETO 95 s, only 5000 bits left on A
            d.push_back(r->Route(9, 0, 6000, 1e9, {}, true, false, SABR_BULK));
            q[2] = {0, 0, 95000};
            d.push_back(r->Route(9, 0, 10000, 1e9, {}, true, false, SABR_NORMAL)); // EVL 5000
            q[2] = {0, 0, 2e5};
            q[3] = {0, 0, 2e6};
            d.push_back(r->Route(9, 0, 1000, 1e9, {}, true, false, SABR_BULK)); // nothing fits
        });
        Simulator::Run();
        Simulator::Destroy();
        NS_TEST_ASSERT_MSG_EQ(d[0].entryNode, 2u, "lower-priority backlog not applicable");
        NS_TEST_ASSERT_MSG_EQ_TOL(d[0].etoS, 0.0, 1e-9, "no applicable backlog");
        NS_TEST_ASSERT_MSG_EQ(d[1].entryNode, 3u, "bulk waits behind its own backlog: B");
        NS_TEST_ASSERT_MSG_EQ_TOL(d[1].pbatS, 16.006, 1e-9, "B: start 10 s + 6 s + 6 ms");
        NS_TEST_ASSERT_MSG_EQ(d[2].entryNode, 3u, "expedited backlog leaves 5000 bits on A");
        NS_TEST_ASSERT_MSG_EQ_TOL(d[2].pbatS, 20.01, 1e-9, "B: 10 s + 10000/1000 + 10 ms");
        NS_TEST_ASSERT_MSG_EQ(d[3].found, false, "failure: backlog beyond every contact end");
    }
};

class SaginSabrCriticalTest : public TestCase
{
  public:
    SaginSabrCriticalTest()
        : TestCase("7B: SABR critical bundles go to every candidate entry node (3.2.8.1.3)")
    {
    }

  private:
    void DoRun() override
    {
        Ptr<SaginSabrRouter> r = MakeRouter();
        Ptr<SaginSabrRouter> r2 = MakeRouter();
        std::vector<SabrDecision> crit, late;
        SabrDecision normal;
        Simulator::Schedule(Seconds(0), [&]() {
            crit = r->RouteCritical(9, 0, 1000, 1e9);
            normal = r2->Route(9, 0, 1000, 1e9);
            late = r->RouteCritical(9, 0, 1000, 0.5); // expires before any arrival
        });
        Simulator::Run();
        Simulator::Destroy();
        NS_TEST_ASSERT_MSG_EQ(crit.size(), 2u, "one copy per entry node");
        NS_TEST_ASSERT_MSG_EQ(crit[0].entryNode, 2u, "best first");
        NS_TEST_ASSERT_MSG_EQ(crit[1].entryNode, 3u, "second neighbor");
        NS_TEST_ASSERT_MSG_EQ(normal.entryNode, 2u, "a normal bundle takes one route");
        NS_TEST_ASSERT_MSG_EQ_TOL(r->GetMtvBits(0), 99000.0, 1e-6, "A route reserved");
        NS_TEST_ASSERT_MSG_EQ_TOL(r->GetMtvBits(2), 389000.0, 1e-6, "B route reserved");
        NS_TEST_ASSERT_MSG_EQ_TOL(r2->GetMtvBits(2), 390000.0, 1e-6, "normal leaves B untouched");
        NS_TEST_ASSERT_MSG_EQ(late.empty(), true, "failure: no candidate before expiry");
    }
};

class SaginWave7bTestSuite : public TestSuite
{
  public:
    SaginWave7bTestSuite()
        : TestSuite("ntn-sagin-wave7b", Type::UNIT)
    {
        AddTestCase(new SaginBpv7CrcWireTest, TestCase::Duration::QUICK);
        AddTestCase(new SaginCrcOverLinkTest(kBpCrc16), TestCase::Duration::QUICK);
        AddTestCase(new SaginCrcOverLinkTest(kBpCrc32c), TestCase::Duration::QUICK);
        AddTestCase(new SaginCrcOverLinkTest(kBpCrcNone), TestCase::Duration::QUICK);
        AddTestCase(new SaginCustodyIntegrityTest, TestCase::Duration::QUICK);
        AddTestCase(new SaginSabrPriorityTest, TestCase::Duration::QUICK);
        AddTestCase(new SaginSabrBacklogTest, TestCase::Duration::QUICK);
        AddTestCase(new SaginSabrCriticalTest, TestCase::Duration::QUICK);
    }
};

static SaginWave7bTestSuite g_saginWave7bTestSuite;
