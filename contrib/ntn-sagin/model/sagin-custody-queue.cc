/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#include "sagin-custody-queue.h"

#include "sagin-bundle-protocol.h"

#include "ns3/log.h"
#include "ns3/simulator.h"
#include "ns3/uinteger.h"

#include <algorithm>

namespace ns3
{
namespace ntnsagin
{

NS_LOG_COMPONENT_DEFINE("SaginCustodyQueue");
NS_OBJECT_ENSURE_REGISTERED(SaginCustodyQueue);

TypeId
SaginCustodyQueue::GetTypeId()
{
    static TypeId tid = TypeId("ns3::ntnsagin::SaginCustodyQueue")
                            .SetParent<Object>()
                            .SetGroupName("NtnSagin")
                            .AddConstructor<SaginCustodyQueue>();
    return tid;
}

bool
SaginCustodyQueue::Offer(Ptr<Packet> p)
{
    if (!p)
    {
        return false;
    }
    if (m_integrity && !m_integrity(p))
    {
        // Wave 7B, RFC 9171 4.2.1: a block CRC failed (or the bundle does not
        // parse); the bundle is unintelligible and is discarded, not stored.
        ++m_corrupt;
        return false;
    }
    ExpireStale();

    Item fresh{p, Simulator::Now()};
    if (m_expiryFn)
    {
        // RFC 9171 5.5: expiry from the bundle's own primary block.
        const BundleExpiry e = m_expiryFn(p, fresh.enqueued);
        if (!e.wellFormed)
        {
            ++m_malformed; // creation time 0 and no Bundle Age block (4.2.7)
            return false;
        }
        fresh.deadline = e.deadline;
        fresh.reportDel = e.reportDeletion;
        if (fresh.enqueued >= fresh.deadline)
        {
            CountExpired(fresh, false); // expired on arrival: deleted, not stored
            return false;
        }
    }

    if (m_contactUp && m_queue.empty() && m_forward &&
        m_forward(m_transform ? m_transform(p->Copy(), Time()) : p))
    {
        ++m_fwdNow;
        if (m_custody)
        {
            m_bytes += p->GetSize();
            Retain(fresh);
            ScheduleExpiry();
        }
        return true;
    }

    // Either the contact is down, or there is already a backlog and this packet
    // must queue behind it - forwarding it now would reorder the stream.
    if (m_key)
    {
        const uint64_t k = m_key(p);
        for (auto it = m_queue.begin(); it != m_queue.end();)
        {
            if (m_key(it->pkt) == k)
            {
                m_bytes -= it->pkt->GetSize();
                it = m_queue.erase(it);
                ++m_replaced;
            }
            else
            {
                ++it;
            }
        }
    }
    const uint64_t sz = p->GetSize();
    while (m_bytes + sz > m_capacityBytes && !m_queue.empty())
    {
        // Drop the OLDEST: in a custody queue it is the closest to expiry, and
        // dropping the newest would discard the freshest data first.
        m_bytes -= m_queue.front().pkt->GetSize();
        m_queue.pop_front();
        ++m_droppedSpace;
    }
    if (m_bytes + sz > m_capacityBytes)
    {
        ++m_droppedSpace; // does not fit even in an empty queue
        return false;
    }
    m_queue.push_back(fresh);
    m_bytes += sz;
    ScheduleExpiry();

    if (m_contactUp)
    {
        Drain();
    }
    return true;
}

void
SaginCustodyQueue::SetContactUp(bool up)
{
    const bool was = m_contactUp;
    m_contactUp = up;
    ExpireStale();
    if (up && !was)
    {
        Drain();
    }
}

void
SaginCustodyQueue::CountExpired(const Item& it, bool inStorage)
{
    ++m_expired;
    ++(inStorage ? m_expiredInStorage : m_expiredOnArrival);
    if (it.reportDel)
    {
        ++m_deletionReports; // RFC 9171 6.1.1 reason code 1, requested by bit 18
    }
}

void
SaginCustodyQueue::ScheduleExpiry()
{
    if (!m_expiryFn)
    {
        return;
    }
    Time next = Time::Max();
    for (const auto& it : m_queue)
    {
        next = std::min(next, it.deadline);
    }
    for (const auto& kv : m_pending)
    {
        next = std::min(next, kv.second.item.deadline);
    }
    if (next == Time::Max())
    {
        return;
    }
    if (m_expiryEvent.IsPending() && m_expiryAt <= next)
    {
        return;
    }
    m_expiryEvent.Cancel();
    m_expiryAt = next;
    m_expiryEvent = Simulator::Schedule(std::max(Time(), next - Simulator::Now()),
                                        &SaginCustodyQueue::OnExpiryTimer,
                                        this);
}

void
SaginCustodyQueue::OnExpiryTimer()
{
    m_expiryAt = Time::Max();
    ExpireStale();
    ScheduleExpiry();
}

void
SaginCustodyQueue::DoDispose()
{
    m_expiryEvent.Cancel();
    m_forward = nullptr;
    m_expiryFn = nullptr;
    m_transform = nullptr;
    Object::DoDispose();
}

void
SaginCustodyQueue::ExpireStale()
{
    const Time now = Simulator::Now();
    if (m_expiryFn)
    {
        // Wave 6B: every bundle at its own RFC 9171 deadline.
        for (auto it = m_pending.begin(); it != m_pending.end();)
        {
            if (now >= it->second.item.deadline)
            {
                m_bytes -= it->second.item.pkt->GetSize();
                CountExpired(it->second.item, true);
                it = m_pending.erase(it);
            }
            else
            {
                ++it;
            }
        }
        for (auto it = m_queue.begin(); it != m_queue.end();)
        {
            if (now >= it->deadline)
            {
                m_bytes -= it->pkt->GetSize();
                CountExpired(*it, true);
                it = m_queue.erase(it);
            }
            else
            {
                ++it;
            }
        }
        return;
    }
    if (m_lifetime <= Time())
    {
        return;
    }
    if (m_custody)
    {
        // RFC 9171 5.4.1: a retained bundle past its lifetime is deleted too.
        for (auto it = m_pending.begin(); it != m_pending.end();)
        {
            const Time origin = m_origin ? m_origin(it->second.item.pkt) : it->second.item.enqueued;
            if (now - origin > m_lifetime)
            {
                m_bytes -= it->second.item.pkt->GetSize();
                it = m_pending.erase(it);
                ++m_expired;
                ++m_deletionReports;
            }
            else
            {
                ++it;
            }
        }
    }
    const uint64_t expiredBefore = m_expired;
    if (m_origin)
    {
        // Origin ages need not follow queue order (keep-newest and LCFS), so
        // every item is checked.
        for (auto it = m_queue.begin(); it != m_queue.end();)
        {
            if (now - m_origin(it->pkt) > m_lifetime)
            {
                m_bytes -= it->pkt->GetSize();
                it = m_queue.erase(it);
                ++m_expired;
            }
            else
            {
                ++it;
            }
        }
        if (m_custody)
        {
            m_deletionReports += m_expired - expiredBefore;
        }
        return;
    }
    while (!m_queue.empty() && (now - m_queue.front().enqueued) > m_lifetime)
    {
        m_bytes -= m_queue.front().pkt->GetSize();
        m_queue.pop_front();
        ++m_expired;
    }
    if (m_custody)
    {
        m_deletionReports += m_expired - expiredBefore;
    }
}

void
SaginCustodyQueue::Drain()
{
    if (!m_contactUp || !m_forward)
    {
        return;
    }
    const Time now = Simulator::Now();
    if (m_perBundleDrain && !m_lcfs)
    {
        // Wave 6B: per-bundle forwarding decisions (contact graph routing).
        // A bundle whose route is not open now is skipped, not a barrier for
        // the bundles behind it, which may have different routes or deadlines.
        for (auto it = m_queue.begin(); it != m_queue.end();)
        {
            if (m_expiryFn && now >= it->deadline)
            {
                m_bytes -= it->pkt->GetSize();
                CountExpired(*it, true);
                it = m_queue.erase(it);
                continue;
            }
            Ptr<Packet> out =
                m_transform ? m_transform(it->pkt->Copy(), now - it->enqueued) : it->pkt;
            if (!m_forward(out))
            {
                ++m_skipped;
                ++it;
                continue;
            }
            m_maxCustody = std::max(m_maxCustody, now - it->enqueued);
            if (m_custody)
            {
                Retain(*it);
            }
            else
            {
                m_bytes -= it->pkt->GetSize();
            }
            it = m_queue.erase(it);
            ++m_fwdCustody;
        }
        return;
    }
    while (!m_queue.empty())
    {
        Item& front = m_lcfs ? m_queue.back() : m_queue.front();
        if (m_expiryFn && now >= front.deadline)
        {
            // RFC 9171 5.5: an expired bundle is deleted, never forwarded.
            m_bytes -= front.pkt->GetSize();
            CountExpired(front, true);
            if (m_lcfs)
            {
                m_queue.pop_back();
            }
            else
            {
                m_queue.pop_front();
            }
            continue;
        }
        Ptr<Packet> out = m_transform ? m_transform(front.pkt->Copy(), now - front.enqueued)
                                      : front.pkt;
        if (!m_forward(out))
        {
            // The forwarder refused. Custody is retained and the drain stops:
            // a hop that could not take the packet has not taken it, and
            // dropping it here would be the silent-success pattern.
            return;
        }
        const Time held = now - front.enqueued;
        if (held > m_maxCustody)
        {
            m_maxCustody = held;
        }
        if (m_custody)
        {
            Retain(front); // keeps the bytes: retained bundles occupy storage
        }
        else
        {
            m_bytes -= front.pkt->GetSize();
        }
        if (m_lcfs)
        {
            m_queue.pop_back();
        }
        else
        {
            m_queue.pop_front();
        }
        ++m_fwdCustody;
    }
}

SaginCustodyQueue::ExpiryFunction
SaginCustodyQueue::Bpv7PrimaryBlockExpiry(double simEpochUnixS)
{
    const int64_t epochDtnMs = static_cast<int64_t>((simEpochUnixS - kDtnEpochUnixS) * 1000.0);
    return [epochDtnMs](Ptr<const Packet> p, Time arrival) {
        BundleExpiry e;
        SaginBundleHeader h;
        p->PeekHeader(h);
        e.reportDeletion = (h.flags & kBpFlagReportDeletion) != 0;
        if (h.creationMs != 0)
        {
            // Age = current time - creation time (5.5 a): the deadline is fixed.
            e.deadline = MilliSeconds(static_cast<int64_t>(h.creationMs) - epochDtnMs +
                                      static_cast<int64_t>(h.lifetimeMs));
        }
        else if (h.hasAgeBlock)
        {
            // No accurate clock at the source: age from the Bundle Age block.
            e.deadline = arrival + MilliSeconds(static_cast<int64_t>(h.lifetimeMs) -
                                                static_cast<int64_t>(h.ageMs));
        }
        else
        {
            e.wellFormed = false;
        }
        return e;
    };
}

SaginCustodyQueue::ForwardTransform
SaginCustodyQueue::Bpv7AgeUpdate()
{
    return [](Ptr<Packet> p, Time dwell) {
        SaginBundleHeader h;
        p->PeekHeader(h);
        if (h.hasAgeBlock && dwell > Time())
        {
            p->RemoveHeader(h);
            h.ageMs += static_cast<uint64_t>(dwell.GetMilliSeconds());
            p->AddHeader(h);
        }
        return p;
    };
}

void
SaginCustodyQueue::SetCustodyTransfer(bool on, Time retransmitTimeout, KeyFunction id)
{
    NS_ABORT_MSG_IF(on && !id, "custody transfer needs a bundle-id function");
    m_custody = on;
    m_custodyTimeout = retransmitTimeout;
    m_id = std::move(id);
}

void
SaginCustodyQueue::Retain(Item& it)
{
    const uint64_t id = m_id(it.pkt);
    auto old = m_pending.find(id);
    if (old != m_pending.end())
    {
        // Same bundle sent again before its signal: one retained copy.
        m_bytes -= old->second.item.pkt->GetSize();
        m_pending.erase(old);
    }
    m_pending[id] = Pending{it, Simulator::Now()};
}

bool
SaginCustodyQueue::AcceptCustody(uint64_t id)
{
    auto it = m_pending.find(id);
    if (it == m_pending.end())
    {
        return false;
    }
    m_bytes -= it->second.item.pkt->GetSize();
    m_pending.erase(it);
    ++m_custAccepted;
    return true;
}

bool
SaginCustodyQueue::RefuseCustody(uint64_t id)
{
    auto it = m_pending.find(id);
    if (it == m_pending.end())
    {
        return false;
    }
    ++m_custRefused;
    // Back into the queue in its original order slot (oldest first); the bytes
    // stay counted, the custody is still ours.
    Item item = it->second.item;
    m_pending.erase(it);
    auto pos = m_queue.begin();
    while (pos != m_queue.end() && pos->enqueued <= item.enqueued)
    {
        ++pos;
    }
    m_queue.insert(pos, item);
    return true;
}

void
SaginCustodyQueue::CheckCustodyTimeouts()
{
    if (!m_custody)
    {
        return;
    }
    const Time now = Simulator::Now();
    bool any = false;
    for (auto it = m_pending.begin(); it != m_pending.end();)
    {
        if (now - it->second.sent >= m_custodyTimeout)
        {
            Item item = it->second.item;
            it = m_pending.erase(it);
            auto pos = m_queue.begin();
            while (pos != m_queue.end() && pos->enqueued <= item.enqueued)
            {
                ++pos;
            }
            m_queue.insert(pos, item);
            ++m_custRetx;
            any = true;
        }
        else
        {
            ++it;
        }
    }
    if (any && m_contactUp)
    {
        Drain();
    }
}

} // namespace ntnsagin
} // namespace ns3

namespace ns3
{
namespace ntnsagin
{
void
SaginCustodyQueue::VisitQueued(std::function<void(Ptr<const Packet>)> f) const
{
    for (const auto& it : m_queue)
    {
        f(it.pkt);
    }
}

SaginCustodyQueue::IntegrityFunction
SaginCustodyQueue::Bpv7CrcCheck()
{
    return [](Ptr<const Packet> p) { return VerifyBundle(p).status == BundleCheck::Ok; };
}
} // namespace ntnsagin
} // namespace ns3
