/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only
//
// sagin-bit-error-model (Wave 7B) - residual bit errors delivered to the
// bundle layer.
//
// ns-3 error models drop a packet they mark as corrupt, which models a link
// layer whose frame check sequence catches every error. Bundles can still
// arrive corrupted when the convergence layer does not check its payload
// (UDP with the checksum off, as the UDP convergence layer of RFC 7122
// permits) or when a frame check misses an error. The RFC 9171 4.2.1 block
// CRCs exist for that case, and they can only be exercised if corrupted
// bytes actually reach the bundle agent.
//
// This model flips bits instead of dropping. Bits in the region starting at
// PayloadOffsetBytes (default 30 = PPP 2 + IPv4 20 + UDP 8, i.e. the UDP
// payload on a point-to-point link) are flipped independently with
// probability BitErrorRate (error positions drawn by geometric gaps, so the
// count per packet is binomial). The IPv4 and UDP headers are left intact so
// the packet is still delivered to the socket. With UdpDstPort set, only
// datagrams to that port are touched (bundles, not custody signals).
// An optional Inner model is evaluated first: if it marks the packet
// corrupt, the packet is dropped as before (e.g. an elevation gate or the
// NtnSatLinkErrorModel packet-error decision).
//
// The residual BER is a declared input, not derived from a modulation and
// coding curve here.

#ifndef SAGIN_BIT_ERROR_MODEL_H
#define SAGIN_BIT_ERROR_MODEL_H

#include "ns3/error-model.h"
#include "ns3/random-variable-stream.h"

#include <cstdint>

namespace ns3
{
namespace ntnsagin
{

class SaginBitErrorModel : public ErrorModel
{
  public:
    static TypeId GetTypeId();
    SaginBitErrorModel();

    void SetBitErrorRate(double ber) { m_ber = ber; }
    double GetBitErrorRate() const { return m_ber; }
    void SetInner(Ptr<ErrorModel> inner) { m_inner = inner; }
    int64_t AssignStreams(int64_t stream);

    uint64_t GetPacketsSeen() const { return m_seen; }
    uint64_t GetPacketsDropped() const { return m_dropped; }
    uint64_t GetPacketsCorrupted() const { return m_corrupted; }
    uint64_t GetBitsFlipped() const { return m_flipped; }

  private:
    bool DoCorrupt(Ptr<Packet> p) override;
    void DoReset() override {}

    double m_ber{0.0};
    uint32_t m_offset{30};
    uint16_t m_port{0};
    Ptr<ErrorModel> m_inner;
    Ptr<UniformRandomVariable> m_uni;
    uint64_t m_seen{0};
    uint64_t m_dropped{0};
    uint64_t m_corrupted{0};
    uint64_t m_flipped{0};
};

} // namespace ntnsagin
} // namespace ns3

#endif // SAGIN_BIT_ERROR_MODEL_H
