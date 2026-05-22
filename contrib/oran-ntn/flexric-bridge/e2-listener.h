/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#ifndef ORAN_NTN_E2_LISTENER_H
#define ORAN_NTN_E2_LISTENER_H

// E2AP listener (Roadmap §3 T3).
//
// Accepts inbound SCTP (or TCP fallback) connections from RAN nodes
// (e.g. a FlexRIC `nearRT-RIC` test client, or another ns-3 instance
// acting as gNB). Implements the toolkit-side state machine:
//
//   1. Accept connection (Listen + AcceptOne)
//   2. Receive E2 Setup Request, respond with E2 Setup Response
//   3. Receive subsequent RIC Subscription / Control messages, route
//      them through the registered handlers, send Response / Ack
//   4. Forward RIC Indication frames received from RAN-side endpoints
//      to the indication trace
//
// Listener does not own any background thread; the user pumps it via
// `Poll(timeout_ms)` from a Simulator::Schedule callback. This avoids
// thread-safety issues with the ns-3 event loop.

#include "e2-transport.h"

#include <ns3/object.h>
#include <ns3/traced-callback.h>

#include <atomic>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace ns3
{
namespace oranntn
{
namespace flexric
{

struct E2SetupSummary
{
    uint32_t global_e2_node_id{0};
    std::vector<uint16_t> ran_function_ids;
};

class OranNtnE2Listener : public Object
{
  public:
    static TypeId GetTypeId();
    OranNtnE2Listener();
    ~OranNtnE2Listener() override;

    /// Choose the wire protocol. Default is TCP for tests; production
    /// builds should call SetTransportKind(Protocol::sctp).
    void SetTransportKind(E2Transport::Protocol p);
    E2Transport::Protocol GetTransportKind() const { return m_kind; }

    /// Start the listener on `host:port`. Returns false on bind failure
    /// (typical when libsctp's kernel module is missing — caller can
    /// retry with TCP).
    bool Start(const std::string& host, uint16_t port);

    /// Stop accepting + close all client sessions.
    void Stop();

    /// One Poll() iteration: accept pending connections + drain RX from
    /// each open client by `max_msgs_per_client`. Returns the number of
    /// messages handled.
    size_t Poll(uint32_t timeout_ms, uint32_t max_msgs_per_client = 16);

    /// Active client session count.
    size_t NumClients() const;

    /// Counters (test asserts).
    uint64_t SetupRequestsHandled() const { return m_setupReq; }
    uint64_t SubscriptionRequestsHandled() const { return m_subReq; }
    uint64_t IndicationsForwarded() const { return m_indFwd; }
    uint64_t ControlRequestsHandled() const { return m_ctrlReq; }

    /// Indication trace — fired for every RIC Indication received.
    TracedCallback<std::vector<uint8_t>> m_indicationTrace;

  protected:
    void DoDispose() override;

  private:
    struct ClientSession
    {
        std::unique_ptr<E2Transport> transport;
        bool e2_setup_done{false};
    };

    void HandleMessage(ClientSession& cs, const E2Message& msg);
    void SendSimpleResponse(ClientSession& cs,
                              E2MessageType replyType);

    E2Transport::Protocol m_kind{E2Transport::Protocol::tcp};
    std::unique_ptr<E2Transport> m_listener;
    std::vector<std::unique_ptr<ClientSession>> m_clients;

    std::atomic<uint64_t> m_setupReq{0};
    std::atomic<uint64_t> m_subReq{0};
    std::atomic<uint64_t> m_indFwd{0};
    std::atomic<uint64_t> m_ctrlReq{0};
};

} // namespace flexric
} // namespace oranntn
} // namespace ns3

#endif // ORAN_NTN_E2_LISTENER_H
