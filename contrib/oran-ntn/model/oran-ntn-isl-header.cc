/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-only
 * Author: Muhammad Uzair
 *
 * ISL Protocol Header Implementation
 */

#include "oran-ntn-isl-header.h"

#include <ns3/log.h>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("OranNtnIslHeader");
NS_OBJECT_ENSURE_REGISTERED(OranNtnIslHeader);

TypeId
OranNtnIslHeader::GetTypeId()
{
    static TypeId tid = TypeId("ns3::OranNtnIslHeader")
                            .SetParent<Header>()
                            .SetGroupName("OranNtn")
                            .AddConstructor<OranNtnIslHeader>();
    return tid;
}

OranNtnIslHeader::OranNtnIslHeader()
    : m_messageType(static_cast<uint8_t>(IslMessageType::HEARTBEAT)),
      m_srcSatId(0),
      m_dstSatId(0),
      m_payloadSize(0),
      m_timestamp(0.0),
      m_sequenceNumber(0),
      m_priority(0),
      m_ttl(8)
{
    NS_LOG_FUNCTION(this);
}

OranNtnIslHeader::~OranNtnIslHeader()
{
    NS_LOG_FUNCTION(this);
}

// ---- Field accessors ----

void
OranNtnIslHeader::SetMessageType(IslMessageType type)
{
    m_messageType = static_cast<uint8_t>(type);
}

IslMessageType
OranNtnIslHeader::GetMessageType() const
{
    return static_cast<IslMessageType>(m_messageType);
}

void
OranNtnIslHeader::SetSourceSatId(uint32_t satId)
{
    m_srcSatId = satId;
}

uint32_t
OranNtnIslHeader::GetSourceSatId() const
{
    return m_srcSatId;
}

void
OranNtnIslHeader::SetDestSatId(uint32_t satId)
{
    m_dstSatId = satId;
}

uint32_t
OranNtnIslHeader::GetDestSatId() const
{
    return m_dstSatId;
}

void
OranNtnIslHeader::SetPayloadSize(uint32_t size)
{
    m_payloadSize = size;
}

uint32_t
OranNtnIslHeader::GetPayloadSize() const
{
    return m_payloadSize;
}

void
OranNtnIslHeader::SetTimestamp(double timestamp)
{
    m_timestamp = timestamp;
}

double
OranNtnIslHeader::GetTimestamp() const
{
    return m_timestamp;
}

void
OranNtnIslHeader::SetSequenceNumber(uint32_t seq)
{
    m_sequenceNumber = seq;
}

uint32_t
OranNtnIslHeader::GetSequenceNumber() const
{
    return m_sequenceNumber;
}

void
OranNtnIslHeader::SetPriority(uint8_t priority)
{
    m_priority = priority;
}

uint8_t
OranNtnIslHeader::GetPriority() const
{
    return m_priority;
}

void
OranNtnIslHeader::SetTtl(uint8_t ttl)
{
    m_ttl = ttl;
}

uint8_t
OranNtnIslHeader::GetTtl() const
{
    return m_ttl;
}

// ---- Header interface ----

TypeId
OranNtnIslHeader::GetInstanceTypeId() const
{
    return GetTypeId();
}

void
OranNtnIslHeader::Print(std::ostream& os) const
{
    os << "ISL Header ["
       << "type=" << static_cast<uint32_t>(m_messageType)
       << " src=" << m_srcSatId
       << " dst=" << m_dstSatId
       << " payload=" << m_payloadSize
       << " ts=" << m_timestamp
       << " seq=" << m_sequenceNumber
       << " pri=" << static_cast<uint32_t>(m_priority)
       << " ttl=" << static_cast<uint32_t>(m_ttl)
       << "]";
}

uint32_t
OranNtnIslHeader::GetSerializedSize() const
{
    // 1 (type) + 4 (src) + 4 (dst) + 4 (payload) + 8 (timestamp) +
    // 4 (seq) + 1 (priority) + 1 (ttl) = 27 bytes
    return 27;
}

void
OranNtnIslHeader::Serialize(Buffer::Iterator start) const
{
    NS_LOG_FUNCTION(this);

    start.WriteU8(m_messageType);
    start.WriteU32(m_srcSatId);
    start.WriteU32(m_dstSatId);
    start.WriteU32(m_payloadSize);

    // Serialize double timestamp as uint64_t via memcpy for portability
    uint64_t tsRaw;
    std::memcpy(&tsRaw, &m_timestamp, sizeof(uint64_t));
    start.WriteU64(tsRaw);

    start.WriteU32(m_sequenceNumber);
    start.WriteU8(m_priority);
    start.WriteU8(m_ttl);
}

uint32_t
OranNtnIslHeader::Deserialize(Buffer::Iterator start)
{
    NS_LOG_FUNCTION(this);

    m_messageType = start.ReadU8();
    m_srcSatId = start.ReadU32();
    m_dstSatId = start.ReadU32();
    m_payloadSize = start.ReadU32();

    uint64_t tsRaw = start.ReadU64();
    std::memcpy(&m_timestamp, &tsRaw, sizeof(double));

    m_sequenceNumber = start.ReadU32();
    m_priority = start.ReadU8();
    m_ttl = start.ReadU8();

    return GetSerializedSize();
}

} // namespace ns3
