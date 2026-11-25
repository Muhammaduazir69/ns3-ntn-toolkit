/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only
//
// Wave 2 M3: BPv7 primary block (RFC 9171 4.3.1) CBOR round trip, custody
// signal (RFC 5050 6.3) round trip, and custody-transfer accounting of
// SaginCustodyQueue (accept, refuse, timeout retransmission, lifetime
// deletion with status-report counting) under Simulator::Run().

#include "ns3/packet.h"
#include "ns3/sagin-bundle-protocol.h"
#include "ns3/sagin-custody-queue.h"
#include "ns3/sagin-sabr-router.h"
#include "ns3/simulator.h"
#include "ns3/test.h"

using namespace ns3;
using namespace ns3::ntnsagin;

namespace
{
Ptr<Packet>
MakeBundle(uint64_t src, uint64_t ms, uint64_t seq, uint32_t payload = 52)
{
    Ptr<Packet> p = Create<Packet>(payload);
    SaginBundleHeader h;
    h.srcNode = src;
    h.destNode = 9000;
    h.reportNode = src;
    h.creationMs = ms;
    h.creationSeq = seq;
    h.lifetimeMs = 7200000;
    h.payloadLength = payload;
    p->AddHeader(h);
    p->AddTrailer(SaginBundleTrailer());
    return p;
}

uint64_t
IdOf(Ptr<const Packet> p)
{
    SaginBundleHeader h;
    p->PeekHeader(h);
    return h.BundleId();
}
} // namespace

class SaginBpv7WireTest : public TestCase
{
  public:
    SaginBpv7WireTest()
        : TestCase("M3: BPv7 primary block and custody signal CBOR round trip")
    {
    }

  private:
    void DoRun() override
    {
        // DTN time of 2026-10-30 00:00 UTC needs a 64-bit CBOR head.
        const uint64_t ms = static_cast<uint64_t>((1793318400.0 - kDtnEpochUnixS) * 1000.0);
        Ptr<Packet> p = MakeBundle(17, ms, 300);
        SaginBundleHeader h;
        p->RemoveHeader(h);
        NS_TEST_ASSERT_MSG_EQ(h.srcNode, 17u, "source node");
        NS_TEST_ASSERT_MSG_EQ(h.creationMs, ms, "creation DTN time");
        NS_TEST_ASSERT_MSG_EQ(h.creationSeq, 300u, "creation sequence");
        NS_TEST_ASSERT_MSG_EQ(h.lifetimeMs, 7200000u, "lifetime");
        NS_TEST_ASSERT_MSG_EQ(h.payloadLength, 52u, "payload bstr length");
        NS_TEST_ASSERT_MSG_EQ(((h.flags & kBpFlagReportDeletion) != 0), true, "deletion reports requested");
        SaginBundleTrailer t;
        p->RemoveTrailer(t);
        NS_TEST_ASSERT_MSG_EQ(p->GetSize(), 52u, "only the payload remains");
        // Primary-block first byte after 0x9F is array(9) = 0x89: since Wave 7B
        // the default bundle carries a CRC-16 (RFC 9171 4.3.1).
        Ptr<Packet> q = MakeBundle(1, 5, 0, 0);
        uint8_t buf[4];
        q->CopyData(buf, 4);
        NS_TEST_ASSERT_MSG_EQ(+buf[0], 0x9F, "indefinite array");
        NS_TEST_ASSERT_MSG_EQ(+buf[1], 0x89, "primary block array(9) with CRC");
        NS_TEST_ASSERT_MSG_EQ(+buf[2], 0x07, "version 7");

        SaginCustodySignalHeader s;
        s.succeeded = false;
        s.reason = kCustodyReasonDepletedStorage;
        s.srcNode = 17;
        s.creationMs = ms;
        s.creationSeq = 300;
        Ptr<Packet> sp = Create<Packet>();
        sp->AddHeader(s);
        NS_TEST_ASSERT_MSG_EQ(sp->GetSize(), s.GetSerializedSize(), "signal size");
        SaginCustodySignalHeader r;
        sp->RemoveHeader(r);
        NS_TEST_ASSERT_MSG_EQ(r.succeeded, false, "refused");
        NS_TEST_ASSERT_MSG_EQ(+r.reason, +kCustodyReasonDepletedStorage, "reason 4");
        NS_TEST_ASSERT_MSG_EQ(r.BundleId(), h.BundleId(), "signal names the same bundle");
    }
};

class SaginCustodyTransferTest : public TestCase
{
  public:
    SaginCustodyTransferTest()
        : TestCase("M3: custody transfer accept-refuse-timeout-lifetime accounting")
    {
    }

  private:
    void DoRun() override
    {
        Ptr<SaginCustodyQueue> q = CreateObject<SaginCustodyQueue>();
        q->SetCapacityBytes(1 << 20);
        q->SetLifetime(Seconds(100));
        q->SetCustodyTransfer(true, Seconds(10), &IdOf);
        std::vector<uint64_t> sent;
        q->SetForwardCallback([&sent](Ptr<Packet> p) {
            sent.push_back(IdOf(p));
            return true;
        });
        q->SetContactUp(true);
        Ptr<Packet> a = MakeBundle(1, 1000, 0);
        Ptr<Packet> b = MakeBundle(1, 1000, 1);
        Ptr<Packet> c = MakeBundle(2, 1000, 0);
        const uint64_t ia = IdOf(a);
        const uint64_t ib = IdOf(b);
        const uint64_t ic = IdOf(c);
        q->Offer(a);
        q->Offer(b);
        q->Offer(c);
        NS_TEST_ASSERT_MSG_EQ(q->GetAwaitingCustody(), 3u, "all retained after forwarding");
        NS_TEST_ASSERT_MSG_GT(q->GetBytesInCustody(), 0u, "retained bundles occupy storage");
        // a accepted, b refused (back in the queue, contact down so it waits).
        Simulator::Schedule(Seconds(1), [&] {
            q->SetContactUp(false);
            NS_TEST_EXPECT_MSG_EQ(q->AcceptCustody(ia), true, "accept a");
            NS_TEST_EXPECT_MSG_EQ(q->RefuseCustody(ib), true, "refuse b");
            NS_TEST_EXPECT_MSG_EQ(q->AcceptCustody(ia), false, "a already released");
            NS_TEST_EXPECT_MSG_EQ(q->GetInCustody(), 1u, "b queued again");
        });
        // c's signal never comes: retransmitted after the 10 s timer.
        Simulator::Schedule(Seconds(12), [&] {
            q->CheckCustodyTimeouts();
            q->SetContactUp(true); // drains b and c again
        });
        // Nothing acknowledged after that: everything expires at 100 s.
        Simulator::Schedule(Seconds(150), [&] { q->ExpireStale(); });
        Simulator::Run();
        NS_TEST_ASSERT_MSG_EQ(q->GetCustodyAccepted(), 1u, "accepted");
        NS_TEST_ASSERT_MSG_EQ(q->GetCustodyRefused(), 1u, "refused");
        NS_TEST_ASSERT_MSG_EQ(q->GetCustodyRetransmissions(), 1u, "one timer expiry");
        NS_TEST_ASSERT_MSG_EQ(sent.size(), 5u, "a, b, c, then b and c again");
        NS_TEST_ASSERT_MSG_EQ((sent[3] == ib || sent[4] == ib), true, "b resent");
        NS_TEST_ASSERT_MSG_EQ((sent[3] == ic || sent[4] == ic), true, "c resent");
        NS_TEST_ASSERT_MSG_EQ(q->GetExpired(), 2u, "b and c deleted at lifetime end");
        NS_TEST_ASSERT_MSG_EQ(q->GetDeletionReports(), 2u, "one deletion report each");
        NS_TEST_ASSERT_MSG_EQ(q->GetBytesInCustody(), 0u, "storage freed");
        Simulator::Destroy();
    }
};


/// Wave 6B: RFC 9171 5.5 per-bundle expiry on a live queue (Simulator::Run).
///
/// Simulation time 0 is DTN time kEpochDtnMs. Normal path: a bundle is
/// deleted from storage AT its own deadline by the expiry timer, with no
/// contact change; a clockless bundle (creation time 0) expires from its
/// Bundle Age block and leaves with its age advanced by the dwell time.
/// Failure paths: an already expired bundle is deleted on arrival; a bundle
/// with creation time 0 and no age block is malformed; a deletion report is
/// counted only when the bundle requested one.
class SaginBpv7ExpiryTest : public TestCase
{
  public:
    SaginBpv7ExpiryTest()
        : TestCase("Wave 6B: BPv7 lifetime enforced per bundle (RFC 9171 5.5, 4.4.2)")
    {
    }

  private:
    static constexpr double kEpochUnix = 1793318400.0;

    static Ptr<Packet> Bundle(uint64_t seq, int64_t createdSimMs, uint64_t lifetimeMs, bool report,
                              bool clockless = false, uint64_t ageMs = 0, bool ageBlock = true)
    {
        Ptr<Packet> p = Create<Packet>(40);
        SaginBundleHeader h;
        h.srcNode = 7;
        h.destNode = 9000;
        h.creationSeq = seq;
        h.creationMs = clockless ? 0
                                 : static_cast<uint64_t>((kEpochUnix - kDtnEpochUnixS) * 1000.0) +
                                       createdSimMs;
        h.lifetimeMs = lifetimeMs;
        h.flags = report ? kBpFlagReportDeletion : 0;
        h.hasAgeBlock = clockless && ageBlock;
        h.ageMs = ageMs;
        h.payloadLength = 40;
        p->AddHeader(h);
        p->AddTrailer(SaginBundleTrailer());
        return p;
    }

    void DoRun() override
    {
        Ptr<SaginCustodyQueue> q = CreateObject<SaginCustodyQueue>();
        q->SetBundleExpiry(SaginCustodyQueue::Bpv7PrimaryBlockExpiry(kEpochUnix));
        q->SetForwardTransform(SaginCustodyQueue::Bpv7AgeUpdate());
        std::vector<SaginBundleHeader> sent;
        q->SetForwardCallback([&sent](Ptr<Packet> p) {
            SaginBundleHeader h;
            p->PeekHeader(h);
            sent.push_back(h);
            return true;
        });

        uint64_t bytesAt5 = 0, bytesAt11 = 0, expAt11 = 0, reportsAt11 = 0;
        bool offerExpired = true, offerMalformed = true;
        // t=0: A (10 s, report requested) and B (100 s, no report) into custody.
        Simulator::Schedule(Seconds(0), [&]() {
            q->Offer(Bundle(1, 0, 10000, true));
            q->Offer(Bundle(2, 0, 100000, false));
        });
        Simulator::Schedule(Seconds(5), [&]() { bytesAt5 = q->GetBytesInCustody(); });
        Simulator::Schedule(Seconds(11), [&]() {
            bytesAt11 = q->GetBytesInCustody();
            expAt11 = q->GetExpiredInStorage();
            reportsAt11 = q->GetDeletionReports();
        });
        // t=20: C was created at t=0 with a 1 s lifetime: expired on arrival.
        Simulator::Schedule(Seconds(20), [&]() { offerExpired = q->Offer(Bundle(3, 0, 1000, true)); });
        // t=30: D, clockless, 5 s old, 8 s lifetime: expires at t=33 unless forwarded.
        Simulator::Schedule(Seconds(30), [&]() { q->Offer(Bundle(4, 0, 8000, false, true, 5000)); });
        // t=30: E, creation time 0 and no age block: malformed.
        Simulator::Schedule(Seconds(30), [&]() {
            offerMalformed = q->Offer(Bundle(5, 0, 8000, false, true, 0, false));
        });
        // t=31: the contact opens; B and D leave, D with age 6 s.
        Simulator::Schedule(Seconds(31), [&]() { q->SetContactUp(true); });
        Simulator::Stop(Seconds(200));
        Simulator::Run();

        NS_TEST_ASSERT_MSG_GT(bytesAt5, bytesAt11, "A left storage at its deadline, no contact change");
        NS_TEST_ASSERT_MSG_EQ(expAt11, 1u, "one bundle expired in storage by t = 11 s");
        NS_TEST_ASSERT_MSG_EQ(reportsAt11, 1u, "A asked for a deletion report");
        NS_TEST_ASSERT_MSG_EQ(offerExpired, false, "an expired bundle is not accepted");
        NS_TEST_ASSERT_MSG_EQ(q->GetExpiredOnArrival(), 1u, "and is counted as expired on arrival");
        NS_TEST_ASSERT_MSG_EQ(offerMalformed, false, "creation time 0 without an age block");
        NS_TEST_ASSERT_MSG_EQ(q->GetMalformed(), 1u, "is malformed");
        NS_TEST_ASSERT_MSG_EQ(sent.size(), 2u, "B and D are forwarded when the contact opens");
        bool sawD = false;
        for (const auto& h : sent)
        {
            if (h.creationSeq == 4)
            {
                sawD = true;
                NS_TEST_ASSERT_MSG_EQ(h.hasAgeBlock, true, "D keeps its Bundle Age block");
                NS_TEST_ASSERT_MSG_EQ(h.ageMs, 6000u, "and its age grew by the 1 s dwell");
            }
            NS_TEST_ASSERT_MSG_NE(h.creationSeq, 1u, "the expired A is never forwarded");
        }
        NS_TEST_ASSERT_MSG_EQ(sawD, true, "D was forwarded before its deadline");
        NS_TEST_ASSERT_MSG_EQ(q->GetDeletionReports(), 2u, "reports for A and C only (bit 18)");
        NS_TEST_ASSERT_MSG_EQ(q->GetInCustody(), 0u, "nothing left");
        Simulator::Destroy();

        // Wire: the age block round-trips and changes the encoded size.
        SaginBundleHeader h1;
        h1.hasAgeBlock = true;
        h1.ageMs = 123456;
        h1.payloadLength = 10;
        Ptr<Packet> p = Create<Packet>(10);
        p->AddHeader(h1);
        SaginBundleHeader h2;
        p->RemoveHeader(h2);
        NS_TEST_ASSERT_MSG_EQ(h2.hasAgeBlock, true, "age block decoded");
        NS_TEST_ASSERT_MSG_EQ(h2.ageMs, 123456u, "age value decoded");
        SaginBundleHeader h3 = h1;
        h3.hasAgeBlock = false;
        NS_TEST_ASSERT_MSG_GT(h1.GetSerializedSize(), h3.GetSerializedSize(), "the block costs bytes");
    }
};

/// Wave 6B: CCSDS 734.3-B-1 SABR on the Blue Book's own example (Figures
/// 3-1 to 3-4: nodes A=1, B=2, C=3, D=4; contact 3 has no range in Figure
/// 3-3 and is given 1 light second here). Decisions are taken inside
/// Simulator::Run at the simulation time they refer to.
class SaginSabrRouterTest : public TestCase
{
  public:
    SaginSabrRouterTest()
        : TestCase("Wave 6B: SABR route computation, candidate filters and volume (CCSDS 734.3-B-1)")
    {
    }

  private:
    void DoRun() override
    {
        // Figure 3-2 contacts with Figure 3-3 ranges (rate 1000 bit/s).
        std::vector<SabrContact> plan = {
            {1, 2, 1000, 1100, 1000, 1},   // 1  A->B
            {2, 1, 1000, 1100, 1000, 1},   // 2  B->A
            {2, 4, 1100, 1200, 1000, 1},   // 3  B->D
            {4, 2, 1100, 1200, 1000, 1},   // 4  D->B
            {1, 3, 1100, 1200, 1000, 30},  // 5  A->C
            {3, 1, 1100, 1200, 1000, 30},  // 6  C->A
            {1, 2, 1300, 1400, 1000, 1},   // 7  A->B
            {2, 1, 1300, 1400, 1000, 1},   // 8  B->A
            {2, 4, 1400, 1500, 1000, 120}, // 9  B->D
            {4, 2, 1400, 1500, 1000, 120}, // 10 D->B
            {3, 4, 1500, 1600, 1000, 90},  // 11 C->D
            {4, 4, 1500, 1600, 1000, 0},   // 12 D->D
        };
        Ptr<SaginSabrRouter> a = CreateObject<SaginSabrRouter>();
        a->SetLocalNode(1);
        a->SetContactPlan(plan);

        std::vector<SabrDecision> d(6);
        size_t listAfter = 0;
        double bcd[4] = {0, 0, 0, 0};
        Simulator::Schedule(Seconds(0), [&]() {
            // 1st bundle, 60 kbit: route AB(1) -> BD(3). PBAT = 1161.
            d[0] = a->Route(4, Simulator::Now().GetSeconds(), 60000, 2000);
            // 2nd: contacts 1 and 3 have 40 kbit left (3.2.8.1.2), route 1 fails
            // filter g); Yen gives AB(7) -> BD(9), PBAT 1580.
            d[1] = a->Route(4, Simulator::Now().GetSeconds(), 60000, 2000);
            // 3rd: AB(7) is now short too, and so is the third path Yen finds,
            // AB(1) -> wait at B -> BD(9) (best case 1520, contact 1 short);
            // the fourth, AC(5) -> CD(11), carries it: PBAT 1650.
            d[2] = a->Route(4, Simulator::Now().GetSeconds(), 60000, 2000);
            listAfter = a->GetRouteList(4).size();
            for (size_t i = 0; i < 4 && i < listAfter; ++i)
            {
                bcd[i] = a->GetRouteList(4)[i].bestCaseDeliveryS;
            }
            // 4th: every route projects arrival after 1600: no candidate (3.2.7).
            d[3] = a->Route(4, Simulator::Now().GetSeconds(), 60000, 1600);
            // An excluded entry node (3.2.5.2): a small bundle that must avoid B.
            d[4] = a->Route(4, Simulator::Now().GetSeconds(), 8000, 2000, {2}, true, false);
        });
        // At t = 1150 contact 1 has ended: pruned (3.2.3.2), route lists rebuilt.
        Simulator::Schedule(Seconds(1150), [&]() {
            d[5] = a->Route(4, Simulator::Now().GetSeconds(), 8000, 2000, {}, true, false);
        });
        Simulator::Run();
        Simulator::Destroy();

        NS_TEST_ASSERT_MSG_EQ(d[0].found, true, "first bundle routed");
        NS_TEST_ASSERT_MSG_EQ(d[0].entryNode, 2u, "via B");
        NS_TEST_ASSERT_MSG_EQ(d[0].route.contacts.size(), 2u, "two contacts");
        NS_TEST_ASSERT_MSG_EQ_TOL(d[0].pbatS, 1161.0, 1e-9, "PBAT 3.2.6.3-7");
        NS_TEST_ASSERT_MSG_EQ_TOL(d[0].route.bestCaseDeliveryS, 1101.0, 1e-9, "best case 3.2.4.2");
        NS_TEST_ASSERT_MSG_EQ(d[1].found, true, "second bundle routed");
        NS_TEST_ASSERT_MSG_EQ(d[1].entryNode, 2u, "via B again, on the later contacts");
        NS_TEST_ASSERT_MSG_EQ_TOL(d[1].pbatS, 1580.0, 1e-9, "PBAT of AB(7)-BD(9)");
        NS_TEST_ASSERT_MSG_EQ(d[2].found, true, "third bundle routed");
        NS_TEST_ASSERT_MSG_EQ(d[2].entryNode, 3u, "via C");
        NS_TEST_ASSERT_MSG_EQ_TOL(d[2].pbatS, 1650.0, 1e-9, "PBAT of AC(5)-CD(11)");
        NS_TEST_ASSERT_MSG_EQ(listAfter, 4u, "four routes, Yen order");
        NS_TEST_ASSERT_MSG_EQ_TOL(bcd[0], 1101.0, 1e-9, "route 1 best case");
        NS_TEST_ASSERT_MSG_EQ_TOL(bcd[1], 1520.0, 1e-9, "route 2 best case");
        NS_TEST_ASSERT_MSG_EQ_TOL(bcd[2], 1520.0, 1e-9, "route 3 best case");
        NS_TEST_ASSERT_MSG_EQ_TOL(bcd[3], 1590.0, 1e-9, "route 4 best case");
        NS_TEST_ASSERT_MSG_EQ(d[3].found, false, "no route delivers before expiry: CGR not used");
        NS_TEST_ASSERT_MSG_EQ(d[4].found, true, "with B excluded");
        NS_TEST_ASSERT_MSG_EQ(d[4].entryNode, 3u, "the bundle goes via C");
        NS_TEST_ASSERT_MSG_EQ(d[5].found, true, "after pruning");
        NS_TEST_ASSERT_MSG_NE(d[5].route.contacts.front(), 0u, "contact 1 is gone");
        NS_TEST_ASSERT_MSG_EQ(a->GetNoRouteDecisions(), 1u, "one no-route decision");
    }
};


/// Wave 6B: per-bundle drain. A bundle whose route is not open is skipped and
/// the bundles behind it still leave (normal path); with the default FIFO
/// drain the refused head blocks the queue (the pre-Wave-6B behavior).
class SaginPerBundleDrainTest : public TestCase
{
  public:
    SaginPerBundleDrainTest()
        : TestCase("Wave 6B: per-bundle drain removes head-of-line blocking")
    {
    }

  private:
    void DoRun() override
    {
        for (bool perBundle : {true, false})
        {
            Ptr<SaginCustodyQueue> q = CreateObject<SaginCustodyQueue>();
            q->SetPerBundleDrain(perBundle);
            std::vector<uint64_t> out;
            q->SetForwardCallback([&out](Ptr<Packet> p) {
                SaginBundleHeader h;
                p->PeekHeader(h);
                if (h.creationSeq == 1)
                {
                    return false; // its route opens later
                }
                out.push_back(h.creationSeq);
                return true;
            });
            Simulator::Schedule(Seconds(1), [&]() {
                for (uint64_t k = 1; k <= 3; ++k)
                {
                    q->Offer(MakeBundle(5, 1000, k));
                }
            });
            Simulator::Schedule(Seconds(2), [&]() { q->SetContactUp(true); });
            Simulator::Run();
            Simulator::Destroy();
            if (perBundle)
            {
                NS_TEST_ASSERT_MSG_EQ(out.size(), 2u, "bundles 2 and 3 leave");
                NS_TEST_ASSERT_MSG_EQ(out.front(), 2u, "in FIFO order");
                NS_TEST_ASSERT_MSG_EQ(q->GetInCustody(), 1u, "bundle 1 waits for its contact");
                NS_TEST_ASSERT_MSG_EQ(q->GetSkipped(), 1u, "skipped once");
            }
            else
            {
                NS_TEST_ASSERT_MSG_EQ(out.size(), 0u, "the refused head blocks the FIFO drain");
                NS_TEST_ASSERT_MSG_EQ(q->GetInCustody(), 3u, "everything stays");
            }
        }
    }
};

class SaginBpv7TestSuite : public TestSuite
{
  public:
    SaginBpv7TestSuite()
        : TestSuite("ntn-sagin-bpv7", Type::UNIT)
    {
        AddTestCase(new SaginBpv7WireTest, TestCase::Duration::QUICK);
        AddTestCase(new SaginCustodyTransferTest, TestCase::Duration::QUICK);
        AddTestCase(new SaginBpv7ExpiryTest, TestCase::Duration::QUICK);
        AddTestCase(new SaginSabrRouterTest, TestCase::Duration::QUICK);
        AddTestCase(new SaginPerBundleDrainTest, TestCase::Duration::QUICK);
    }
};

static SaginBpv7TestSuite g_saginBpv7TestSuite;
