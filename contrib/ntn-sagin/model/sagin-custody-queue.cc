/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#include "sagin-custody-queue.h"

#include "ns3/log.h"
#include "ns3/simulator.h"
#include "ns3/uinteger.h"

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
    ExpireStale();

    if (m_contactUp && m_queue.empty() && m_forward && m_forward(p))
    {
        ++m_fwdNow;
        return true;
    }

    // Either the contact is down, or there is already a backlog and this packet
    // must queue behind it - forwarding it now would reorder the stream.
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
    m_queue.push_back({p, Simulator::Now()});
    m_bytes += sz;

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
SaginCustodyQueue::ExpireStale()
{
    if (m_lifetime <= Time())
    {
        return;
    }
    const Time now = Simulator::Now();
    while (!m_queue.empty() && (now - m_queue.front().enqueued) > m_lifetime)
    {
        m_bytes -= m_queue.front().pkt->GetSize();
        m_queue.pop_front();
        ++m_expired;
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
    while (!m_queue.empty())
    {
        Item& front = m_queue.front();
        if (!m_forward(front.pkt))
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
        m_bytes -= front.pkt->GetSize();
        m_queue.pop_front();
        ++m_fwdCustody;
    }
}

} // namespace ntnsagin
} // namespace ns3
