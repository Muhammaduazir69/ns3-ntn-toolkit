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
        // Primary-block first byte after 0x9F is array(8) = 0x88.
        Ptr<Packet> q = MakeBundle(1, 5, 0, 0);
        uint8_t buf[4];
        q->CopyData(buf, 4);
        NS_TEST_ASSERT_MSG_EQ(+buf[0], 0x9F, "indefinite array");
        NS_TEST_ASSERT_MSG_EQ(+buf[1], 0x88, "primary block array(8)");
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
        : TestCase("M3: custody transfer accept/refuse/timeout/lifetime accounting")
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

class SaginBpv7TestSuite : public TestSuite
{
  public:
    SaginBpv7TestSuite()
        : TestSuite("ntn-sagin-bpv7", Type::UNIT)
    {
        AddTestCase(new SaginBpv7WireTest, TestCase::Duration::QUICK);
        AddTestCase(new SaginCustodyTransferTest, TestCase::Duration::QUICK);
    }
};

static SaginBpv7TestSuite g_saginBpv7TestSuite;
