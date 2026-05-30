/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#ifndef NTN_SIONNA_BATCH_CLIENT_H
#define NTN_SIONNA_BATCH_CLIENT_H

// Async batched query API on top of `SionnaTransport` (Roadmap §4.2.5).
//
// Callers ask for many path-loss queries at once. Today's underlying
// SionnaTransport API is synchronous (`Query()` blocks until the
// server responds), so this client wraps it with a thin scheduling
// layer that dispatches the requests through `Simulator::ScheduleNow`,
// accumulates the responses in order, and fires a completion callback
// when the whole batch is settled.
//
// Three call patterns:
//
//   1. `RequestBatch(reqs, cb)` — fully async; returns handle, fires
//      `cb` with the response vector when complete.
//   2. `RequestBatchSync(reqs)` — drains the batch in the caller's
//      context. Useful outside Simulator::Run().
//   3. `CancelBatch(handle)` — cancels a pending async batch.
//
// Responses are always returned in the same order as the original
// `reqs` vector so the caller can index by position.

#include "sionna-transport.h"

#include <ns3/event-id.h>
#include <ns3/nstime.h>
#include <ns3/object.h>
#include <ns3/ptr.h>

#include <cstdint>
#include <functional>
#include <map>
#include <vector>

namespace ns3
{

/**
 * \ingroup ntn-sionna
 * \brief Async batched Sionna-RT query client (Roadmap §4.2.5).
 */
class SionnaBatchClient : public Object
{
  public:
    using Request = SionnaTransport::Request;
    using Response = SionnaTransport::Response;
    using BatchCallback =
        std::function<void(const std::vector<Response>&)>;

    static TypeId GetTypeId();
    SionnaBatchClient();
    ~SionnaBatchClient() override;

    /// Adopt the underlying transport. Must be Attached before any
    /// Request*() call.
    void Attach(Ptr<SionnaTransport> transport);
    bool IsAttached() const { return m_transport != nullptr; }

    /// Optional per-request stagger; defaults to 0. Useful in tests
    /// that need to see batches resolve over simulated time.
    void SetPerRequestStagger(Time t) { m_stagger = t; }
    Time GetPerRequestStagger() const { return m_stagger; }

    /// Dispatch a batch asynchronously. Returns a handle (>0) on
    /// success, 0 on failure (no transport attached or empty input).
    uint64_t RequestBatch(const std::vector<Request>& reqs,
                            BatchCallback cb);

    /// Dispatch a batch synchronously; returns responses inline.
    /// Counter-bumps are identical to RequestBatch.
    std::vector<Response>
        RequestBatchSync(const std::vector<Request>& reqs);

    /// Cancel a pending batch. Returns true iff the handle was active.
    bool CancelBatch(uint64_t handle);

    /// Number of batches still in flight.
    size_t PendingBatchCount() const { return m_batches.size(); }

    uint64_t BatchesSubmitted() const { return m_submitted; }
    uint64_t BatchesCompleted() const { return m_completed; }
    uint64_t BatchesCancelled() const { return m_cancelled; }
    uint64_t TotalQueries() const { return m_totalQueries; }

  protected:
    void DoDispose() override;

  private:
    struct BatchState
    {
        std::vector<Response> responses;
        std::vector<bool> filled;
        size_t remaining{0};
        BatchCallback cb;
        std::vector<EventId> events;
        bool cancelled{false};
    };

    void DispatchOne(uint64_t handle, size_t idx, Request req);

    Ptr<SionnaTransport> m_transport;
    Time m_stagger;
    std::map<uint64_t, BatchState> m_batches;
    uint64_t m_nextHandle{1};
    uint64_t m_submitted{0};
    uint64_t m_completed{0};
    uint64_t m_cancelled{0};
    uint64_t m_totalQueries{0};
};

} // namespace ns3

#endif // NTN_SIONNA_BATCH_CLIENT_H
