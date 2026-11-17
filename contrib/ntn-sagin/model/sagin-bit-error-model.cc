/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#include "sagin-bit-error-model.h"

#include "ns3/double.h"
#include "ns3/log.h"
#include "ns3/packet.h"
#include "ns3/pointer.h"
#include "ns3/uinteger.h"

#include <cmath>
#include <vector>

namespace ns3
{
namespace ntnsagin
{

NS_LOG_COMPONENT_DEFINE("SaginBitErrorModel");
NS_OBJECT_ENSURE_REGISTERED(SaginBitErrorModel);

TypeId
SaginBitErrorModel::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::ntnsagin::SaginBitErrorModel")
            .SetParent<ErrorModel>()
            .SetGroupName("NtnSagin")
            .AddConstructor<SaginBitErrorModel>()
            .AddAttribute("BitErrorRate",
                          "Residual bit error rate applied to the protected region.",
                          DoubleValue(0.0),
                          MakeDoubleAccessor(&SaginBitErrorModel::m_ber),
                          MakeDoubleChecker<double>(0.0, 0.5))
            .AddAttribute("PayloadOffsetBytes",
                          "First byte that may be flipped (30 = PPP + IPv4 + UDP headers).",
                          UintegerValue(30),
                          MakeUintegerAccessor(&SaginBitErrorModel::m_offset),
                          MakeUintegerChecker<uint32_t>())
            .AddAttribute("UdpDstPort",
                          "Only corrupt UDP datagrams to this port (0 = any packet).",
                          UintegerValue(0),
                          MakeUintegerAccessor(&SaginBitErrorModel::m_port),
                          MakeUintegerChecker<uint16_t>())
            .AddAttribute("Inner",
                          "Error model evaluated first; a packet it marks corrupt is dropped.",
                          PointerValue(),
                          MakePointerAccessor(&SaginBitErrorModel::m_inner),
                          MakePointerChecker<ErrorModel>());
    return tid;
}

SaginBitErrorModel::SaginBitErrorModel()
{
    m_uni = CreateObject<UniformRandomVariable>();
}

int64_t
SaginBitErrorModel::AssignStreams(int64_t stream)
{
    m_uni->SetStream(stream);
    return 1;
}

bool
SaginBitErrorModel::DoCorrupt(Ptr<Packet> p)
{
    ++m_seen;
    if (m_inner && m_inner->IsCorrupt(p))
    {
        ++m_dropped;
        return true;
    }
    const uint32_t size = p->GetSize();
    if (m_ber <= 0.0 || size <= m_offset)
    {
        return false;
    }
    std::vector<uint8_t> b(size);
    p->CopyData(b.data(), size);
    if (m_port != 0)
    {
        // UDP destination port: bytes 2..3 of the UDP header, which ends at the offset.
        if (m_offset < 8)
        {
            return false;
        }
        const uint32_t udp = m_offset - 8;
        const uint16_t dst = static_cast<uint16_t>((b[udp + 2] << 8) | b[udp + 3]);
        if (dst != m_port)
        {
            return false;
        }
    }
    // Geometric gaps between independent bit errors over the region.
    const uint64_t nbits = static_cast<uint64_t>(size - m_offset) * 8;
    const double l1p = std::log1p(-m_ber);
    uint64_t flips = 0;
    double pos = -1.0;
    while (true)
    {
        double u = m_uni->GetValue();
        u = u <= 0.0 ? 1e-300 : u;
        pos += 1.0 + std::floor(std::log(u) / l1p);
        if (pos >= static_cast<double>(nbits))
        {
            break;
        }
        const uint64_t bit = static_cast<uint64_t>(pos);
        b[m_offset + bit / 8] ^= static_cast<uint8_t>(0x80 >> (bit % 8));
        ++flips;
    }
    if (flips == 0)
    {
        return false;
    }
    // Replace the packet's bytes with the corrupted ones; the packet object
    // (and its packet tags) is kept, so it continues up the stack.
    p->RemoveAtEnd(size);
    p->AddAtEnd(Create<Packet>(b.data(), size));
    ++m_corrupted;
    m_flipped += flips;
    return false;
}

} // namespace ntnsagin
} // namespace ns3
