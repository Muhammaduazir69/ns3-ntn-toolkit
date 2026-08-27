/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only
//
// sagin-custody-queue - store-and-forward for disconnected operation.
//
// Why this exists (audit SAGIN-2). A grep for queue, congestion, buffer,
// store-and-forward, DTN or bundle across ntn-sagin and ntn-constellation
// returned nothing relevant. Contact-graph routing exists in the space
// community precisely BECAUSE contacts are intermittent: a node takes custody
// of traffic while the next hop is out of contact and forwards it when the
// contact opens. Without that, a disconnected-operation scenario - the main
// reason for contact-graph routing at all - cannot be run, and a router that
// finds no path simply drops.
//
// This is deliberately a custody queue and not a full DTN stack. It models the
// part the toolkit was missing: hold on contact-down, drain in order on
// contact-up, expire what outlives its lifetime, and report delivered against
// expired against dropped-for-space so a scenario can say what the store-and-
// forward actually bought. It does not implement RFC 5050 bundle formats,
// custody signalling or fragmentation, and does not claim to.

#ifndef SAGIN_CUSTODY_QUEUE_H
#define SAGIN_CUSTODY_QUEUE_H

#include "ns3/nstime.h"
#include "ns3/object.h"
#include "ns3/packet.h"

#include <cstdint>
#include <deque>
#include <functional>

namespace ns3
{
namespace ntnsagin
{

/**
 * \ingroup ntn-sagin
 * \brief Hold traffic while the next hop is out of contact; drain when it opens.
 */
class SaginCustodyQueue : public Object
{
  public:
    static TypeId GetTypeId();
    SaginCustodyQueue() = default;

    /// Where a drained packet goes. Return false to refuse it, in which case
    /// the packet stays in custody and the drain stops for this contact - a
    /// forwarder that cannot take the packet has not taken it.
    using ForwardCallback = std::function<bool(Ptr<Packet>)>;
    void SetForwardCallback(ForwardCallback cb) { m_forward = std::move(cb); }

    /// Maximum bytes held. Beyond this the OLDEST item is dropped, because in a
    /// store-and-forward queue the oldest is the closest to expiry anyway.
    void SetCapacityBytes(uint64_t bytes) { m_capacityBytes = bytes; }
    uint64_t GetCapacityBytes() const { return m_capacityBytes; }

    /// How long an item may stay in custody before it is discarded as stale.
    void SetLifetime(Time t) { m_lifetime = t; }
    Time GetLifetime() const { return m_lifetime; }

    /// Offer a packet. Accepted into custody when the contact is DOWN, or
    /// forwarded immediately when it is up and the forwarder takes it.
    /// \return true if the packet was forwarded or taken into custody.
    bool Offer(Ptr<Packet> p);

    /// Open or close the contact to the next hop. Opening drains the queue in
    /// arrival order; closing simply stops forwarding.
    void SetContactUp(bool up);
    bool IsContactUp() const { return m_contactUp; }

    /// Discard everything older than the lifetime. Called on every contact
    /// change and callable directly by a scenario's tick.
    void ExpireStale();

    // ---- KPIs: what the store-and-forward actually bought ----
    uint64_t GetForwardedImmediately() const { return m_fwdNow; }
    uint64_t GetForwardedFromCustody() const { return m_fwdCustody; }
    uint64_t GetExpired() const { return m_expired; }
    uint64_t GetDroppedForSpace() const { return m_droppedSpace; }
    uint64_t GetInCustody() const { return m_queue.size(); }
    uint64_t GetBytesInCustody() const { return m_bytes; }
    /// Longest time any delivered item spent in custody.
    Time GetMaxCustodyDelay() const { return m_maxCustody; }

  private:
    struct Item
    {
        Ptr<Packet> pkt;
        Time enqueued;
    };

    void Drain();

    ForwardCallback m_forward;
    std::deque<Item> m_queue;
    uint64_t m_capacityBytes{1024 * 1024};
    uint64_t m_bytes{0};
    Time m_lifetime{Seconds(3600)};
    bool m_contactUp{false};

    uint64_t m_fwdNow{0};
    uint64_t m_fwdCustody{0};
    uint64_t m_expired{0};
    uint64_t m_droppedSpace{0};
    Time m_maxCustody{};
};

} // namespace ntnsagin
} // namespace ns3

#endif // SAGIN_CUSTODY_QUEUE_H
