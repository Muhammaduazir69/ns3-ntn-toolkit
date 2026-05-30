/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#ifndef ORAN_NTN_E2_TRANSPORT_H
#define ORAN_NTN_E2_TRANSPORT_H

// E2AP transport abstraction (Roadmap §3 T3).
//
// The toolkit ships two backends behind the same interface:
//   - TCP: always available, uses standard SOCK_STREAM. Used in tests
//     and in environments where SCTP kernel modules are unavailable
//     (most CI containers).
//   - SCTP: SOCK_STREAM SCTP via libsctp on Linux. Wire-compatible with
//     real FlexRIC `nearRT-RIC` binary. Compiled in unconditionally
//     since libsctp-dev is widely available; a runtime probe at Open()
//     time confirms kernel support.
//
// Both backends frame messages as
//     uint16 type | uint32 len | payload
// per `e2-message-types.h`, so application-layer dispatch is identical.

#include "e2-message-types.h"

#include <cstdint>
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

struct E2Message
{
    E2MessageType type;
    std::vector<uint8_t> payload;
};

class E2Transport
{
  public:
    enum class Protocol : uint8_t
    {
        tcp = 0,
        sctp = 1,
    };

    virtual ~E2Transport() = default;

    /// Listener-side bind + listen. Returns false on failure.
    virtual bool Listen(const std::string& host, uint16_t port) = 0;

    /// Client-side connect. Returns false on failure.
    virtual bool Connect(const std::string& host, uint16_t port) = 0;

    /// Accept one inbound connection on a listener socket. Returns a
    /// new E2Transport for the accepted peer, or nullptr on error.
    /// `timeout_ms = 0` means non-blocking poll.
    virtual std::unique_ptr<E2Transport>
        AcceptOne(uint32_t timeout_ms) = 0;

    /// Send a framed E2 message. Returns the number of payload bytes
    /// sent or -1 on error.
    virtual int Send(const E2Message& msg) = 0;

    /// Blocking read with a wall-clock timeout. Returns false on
    /// timeout / connection close / framing error.
    virtual bool Recv(E2Message& out, uint32_t timeout_ms) = 0;

    /// Closes the socket.
    virtual void Close() = 0;

    virtual Protocol Kind() const = 0;
    virtual bool IsOpen() const = 0;
};

/// Factory — returns the requested transport. Both backends are always
/// linkable; SCTP availability is checked at Open() time.
std::unique_ptr<E2Transport> MakeE2Transport(E2Transport::Protocol p);

} // namespace flexric
} // namespace oranntn
} // namespace ns3

#endif // ORAN_NTN_E2_TRANSPORT_H
