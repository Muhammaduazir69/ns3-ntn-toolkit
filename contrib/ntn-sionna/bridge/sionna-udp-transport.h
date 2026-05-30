/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#ifndef NTN_SIONNA_UDP_TRANSPORT_H
#define NTN_SIONNA_UDP_TRANSPORT_H

// UDP / JSON transport (Roadmap §4.2.1).
//
// Wire protocol matches contrib/ntn-sionna/bridge/sionna-server.py:
//   request  : {"tx":[x,y,z],"rx":[x,y,z],"freq_hz":<f>,"id":<n>}
//   response : {"id":<n>,"path_loss_db":<d>,"n_paths":<k>,"compute_ms":<m>}
//
// SO_RCVTIMEO bounds Query() latency. Failure (timeout, send/recv error,
// malformed response) returns +inf so the caller can fall back to FSPL.

#include "sionna-transport.h"

#include <cstdint>
#include <string>

namespace ns3
{

class SionnaUdpTransport : public SionnaTransport
{
  public:
    static TypeId GetTypeId();
    SionnaUdpTransport();
    ~SionnaUdpTransport() override;

    /// Set the UDP destination. Subsequent Query() calls use it.
    void SetServer(const std::string& host, uint16_t port);
    /// Set the per-query receive timeout in milliseconds.
    void SetTimeoutMs(uint32_t timeoutMs);
    /// Inspect current config (test/diagnostic helpers).
    const std::string& Host() const { return m_host; }
    uint16_t Port() const { return m_port; }
    uint32_t TimeoutMs() const { return m_timeoutMs; }

    Response Query(const Request& req) const override;
    std::string Name() const override { return "udp"; }
    /// UDP is always "available" in the sense that the socket layer
    /// is — actual server reachability is verified per-query.
    bool IsAvailable() const override { return true; }

  protected:
    bool EnsureSocket() const;

  private:
    std::string m_host;
    uint16_t m_port;
    uint32_t m_timeoutMs;

    mutable int m_sock; //!< -1 when not yet opened
};

} // namespace ns3

#endif // NTN_SIONNA_UDP_TRANSPORT_H
