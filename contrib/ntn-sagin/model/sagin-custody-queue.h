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
// forward actually bought.
//
// Custody transfer (opt-in, SetCustodyTransfer): RFC 5050 clause 5.10 custody
// semantics, which BPv7 (RFC 9171) removed from the base protocol and left to
// an extension. With it on, a forwarded bundle is NOT released: it stays in
// custody (and keeps occupying storage) until the next hop's custody signal
// arrives. AcceptCustody() releases it ("custody accepted", RFC 5050 6.3
// succeeded = 1); RefuseCustody() returns it to the queue (refused, the sender
// keeps custody); no signal within the retransmission timeout returns it to
// the queue for another attempt (RFC 5050 5.10.1 custody transfer timer).
// Lifetime applies to retained bundles too (RFC 9171 4.2.2 / 5.5: a bundle
// whose age exceeds its lifetime is deleted, status report reason code 1
// "lifetime expired", RFC 9171 6.1.1), and the deletion is counted as a
// generated bundle-deletion status report.
//
// Wave 6B, BPv7 expiry (SetBundleExpiry). The queue-level lifetime above is a
// per-hop or per-origin timer configured on the QUEUE. RFC 9171 5.5 makes
// expiry a property of the BUNDLE: it expires when its age exceeds the
// lifetime field of its own primary block, age being current time minus the
// creation timestamp, or, when the creation time is 0, the Bundle Age block
// (4.4.2). With an expiry function set, every bundle carries its own deadline
// from its own header; it is deleted on arrival if already expired, never
// forwarded once expired (checked at the moment of forwarding), and deleted
// from storage AT its deadline by a timer, not at the next contact change, so
// expired bundles stop occupying storage when they expire. A deletion status
// report is counted only when the bundle asked for one (RFC 9171 4.2.3 bit
// 18). A bundle with creation time 0 and no Bundle Age block is malformed
// (RFC 9171 4.2.7) and is deleted. The bundle format itself (CBOR
// primary block) lives in sagin-bundle-protocol.h; fragmentation,
// convergence-layer (LTP, RFC 5326) and security (BPSec, RFC 9172) are not
// implemented.

#ifndef SAGIN_CUSTODY_QUEUE_H
#define SAGIN_CUSTODY_QUEUE_H

#include "ns3/event-id.h"
#include "ns3/nstime.h"
#include "ns3/object.h"
#include "ns3/packet.h"

#include <cstdint>
#include <deque>
#include <functional>
#include <map>

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
    /// Keep only the newest packet of each source: when set, Offer() first removes any queued packet
    /// whose key equals the new packet's key (a status update supersedes the older one).
    using KeyFunction = std::function<uint64_t(Ptr<const Packet>)>;
    void SetReplaceSameSource(KeyFunction key) { m_key = std::move(key); }
    uint64_t GetReplaced() const { return m_replaced; }
    /// Drain the newest item first (last come, first served) instead of in
    /// arrival order. With contacts long enough to empty the queue the order
    /// does not matter; when a contact ends mid-drain, LCFS has delivered the
    /// freshest updates and FCFS the stalest.
    void SetLastComeFirstServed(bool on) { m_lcfs = on; }
    /// Wave 6B: decide per bundle when draining. A refused bundle is skipped
    /// and the drain continues with the next one (FIFO order kept), instead
    /// of stopping at the first refusal. Contact graph routing needs this:
    /// each bundle has its own route and deadline, so one bundle waiting for
    /// a later contact must not block the others (head-of-line blocking).
    void SetPerBundleDrain(bool on) { m_perBundleDrain = on; }
    /// Bundles skipped by a per-bundle drain (refused by the forwarder).
    uint64_t GetSkipped() const { return m_skipped; }
    /// Measure the lifetime end to end: when set, an item's age is Now() minus
    /// the origin time this function reads from the packet (for instance the
    /// generation timestamp of a status update), not the time it entered this
    /// queue. A custody lifetime meant for the whole delivery then means the
    /// same thing at every hop.
    using OriginFunction = std::function<Time(Ptr<const Packet>)>;
    void SetOriginFunction(OriginFunction f) { m_origin = std::move(f); }
    Time GetLifetime() const { return m_lifetime; }

    /// Wave 6B: per-bundle expiry (RFC 9171 5.5). \p arrival is when the
    /// bundle reached this node.
    struct BundleExpiry
    {
        Time deadline{Time::Max()}; //!< simulation time at which the bundle expires
        bool reportDeletion{false}; //!< RFC 9171 4.2.3 bit 18 set
        bool wellFormed{true};      //!< false: no way to know its age (4.2.7)
    };
    using ExpiryFunction = std::function<BundleExpiry(Ptr<const Packet>, Time arrival)>;
    void SetBundleExpiry(ExpiryFunction f) { m_expiryFn = std::move(f); }
    /// The RFC 9171 expiry of a packet that starts with a SaginBundleHeader:
    /// creation timestamp + lifetime, or arrival + (lifetime - Bundle Age)
    /// when the creation time is 0. \p simEpochUnixS is the Unix time of
    /// simulation time 0, which maps DTN time onto simulation time.
    static ExpiryFunction Bpv7PrimaryBlockExpiry(double simEpochUnixS);

    /// Wave 7B: integrity check run on every Offer() before anything else
    /// reads the packet. A bundle that fails it is discarded and counted
    /// (GetCorrupt). Bpv7CrcCheck() verifies the RFC 9171 block CRCs.
    using IntegrityFunction = std::function<bool(Ptr<const Packet>)>;
    void SetIntegrityCheck(IntegrityFunction f) { m_integrity = std::move(f); }
    static IntegrityFunction Bpv7CrcCheck();
    uint64_t GetCorrupt() const { return m_corrupt; }
    /// Wave 7B: visit every packet held in storage (not those awaiting a
    /// custody signal), in queue order. Used for per-neighbor backlog.
    void VisitQueued(std::function<void(Ptr<const Packet>)> f) const;

    /// Applied to a packet as it is forwarded, with the time it spent here.
    using ForwardTransform = std::function<Ptr<Packet>(Ptr<Packet>, Time dwell)>;
    void SetForwardTransform(ForwardTransform f) { m_transform = std::move(f); }
    /// RFC 9171 4.4.2: add the dwell time at this node to the Bundle Age
    /// block, if the bundle carries one, on every forwarding.
    static ForwardTransform Bpv7AgeUpdate();

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
    /// Wave 6B: expired bundles deleted on arrival, deleted from storage by
    /// the expiry timer or at forwarding time, and malformed ones deleted.
    uint64_t GetExpiredOnArrival() const { return m_expiredOnArrival; }
    uint64_t GetExpiredInStorage() const { return m_expiredInStorage; }
    uint64_t GetMalformed() const { return m_malformed; }
    uint64_t GetDroppedForSpace() const { return m_droppedSpace; }
    uint64_t GetInCustody() const { return m_queue.size(); }
    uint64_t GetBytesInCustody() const { return m_bytes; }
    /// Longest time any delivered item spent in custody.
    Time GetMaxCustodyDelay() const { return m_maxCustody; }

    // ---- RFC 5050 5.10 custody transfer (opt-in) ----
    /// Keep forwarded items until AcceptCustody(); \p id maps a packet to its
    /// bundle identifier (source EID + creation timestamp, RFC 9171 4.2.7).
    void SetCustodyTransfer(bool on, Time retransmitTimeout, KeyFunction id);
    bool GetCustodyTransfer() const { return m_custody; }
    /// Custody signal "succeeded" from the next hop: release the bundle.
    /// \return true if a retained bundle with this id was released.
    bool AcceptCustody(uint64_t id);
    /// Custody signal "failed": the sender keeps custody; the bundle returns
    /// to the queue for another attempt.
    bool RefuseCustody(uint64_t id);
    /// Return bundles whose custody signal is overdue to the queue.
    void CheckCustodyTimeouts();
    uint64_t GetCustodyAccepted() const { return m_custAccepted; }
    uint64_t GetCustodyRefused() const { return m_custRefused; }
    uint64_t GetCustodyRetransmissions() const { return m_custRetx; }
    uint64_t GetAwaitingCustody() const { return m_pending.size(); }
    /// Bundle-deletion status reports generated (RFC 9171 6.1.1, reason 1
    /// lifetime expired), one per expired bundle while custody transfer is on.
    uint64_t GetDeletionReports() const { return m_deletionReports; }

  protected:
    void DoDispose() override;

  private:
    struct Item
    {
        Ptr<Packet> pkt;
        Time enqueued;
        Time deadline{Time::Max()}; //!< Wave 6B: RFC 9171 5.5 expiry
        bool reportDel{false};
    };
    /// Delete \p it as expired; counts the deletion report when requested.
    void CountExpired(const Item& it, bool inStorage);
    /// Arm the expiry timer at the earliest deadline held.
    void ScheduleExpiry();
    void OnExpiryTimer();
    ExpiryFunction m_expiryFn;
    IntegrityFunction m_integrity;
    uint64_t m_corrupt{0};
    ForwardTransform m_transform;
    EventId m_expiryEvent;
    Time m_expiryAt{Time::Max()};
    uint64_t m_expiredOnArrival{0};
    uint64_t m_expiredInStorage{0};
    uint64_t m_malformed{0};
    bool m_perBundleDrain{false};
    uint64_t m_skipped{0};

    void Drain();
    /// Forwarded: release (no custody transfer) or retain awaiting a signal.
    void Retain(Item& it);

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
    KeyFunction m_key;
    uint64_t m_replaced{0};
    bool m_lcfs{false};
    OriginFunction m_origin;

    bool m_custody{false};
    Time m_custodyTimeout{Seconds(30)};
    KeyFunction m_id;
    struct Pending
    {
        Item item;
        Time sent;
    };
    std::map<uint64_t, Pending> m_pending;
    uint64_t m_custAccepted{0};
    uint64_t m_custRefused{0};
    uint64_t m_custRetx{0};
    uint64_t m_deletionReports{0};
};

} // namespace ntnsagin
} // namespace ns3

#endif // SAGIN_CUSTODY_QUEUE_H
