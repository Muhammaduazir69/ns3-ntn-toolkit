/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2017 Magister Solutions Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: Frans Laakso <frans.laakso@magister.fi>
 */

#include "time-tag.h"

#include <ostream>

namespace ns3
{

/*
 * There are 3 classes defined here: PhyTimeTag, MacTimeTag and
 * DevTimeTag. Except of the name difference, they share
 * exactly the same definitions.
 */

// PHY ////////////////////////////////////////////////////////////////////////

NS_OBJECT_ENSURE_REGISTERED(PhyTimeTag);

PhyTimeTag::PhyTimeTag()
    : m_senderTimestamp(Seconds(0))
{
    // Nothing to do here
}

PhyTimeTag::PhyTimeTag(Time senderTimestamp)
    : m_senderTimestamp(senderTimestamp)
{
    // Nothing to do here
}

TypeId
PhyTimeTag::GetTypeId(void)
{
    static TypeId tid = TypeId("ns3::PhyTimeTag").SetParent<Tag>().AddConstructor<PhyTimeTag>();
    return tid;
}

TypeId
PhyTimeTag::GetInstanceTypeId(void) const
{
    return GetTypeId();
}

uint32_t
PhyTimeTag::GetSerializedSize(void) const
{
    return sizeof(Time);
}

void
PhyTimeTag::Serialize(TagBuffer i) const
{
    int64_t senderTimestamp = m_senderTimestamp.GetNanoSeconds();
    i.Write((const uint8_t*)&senderTimestamp, sizeof(int64_t));
}

void
PhyTimeTag::Deserialize(TagBuffer i)
{
    int64_t senderTimestamp;
    i.Read((uint8_t*)&senderTimestamp, 8);
    m_senderTimestamp = NanoSeconds(senderTimestamp);
}

void
PhyTimeTag::Print(std::ostream& os) const
{
    os << m_senderTimestamp;
}

Time
PhyTimeTag::GetSenderTimestamp(void) const
{
    return m_senderTimestamp;
}

void
PhyTimeTag::SetSenderTimestamp(Time senderTimestamp)
{
    this->m_senderTimestamp = senderTimestamp;
}

// MAC ////////////////////////////////////////////////////////////////////////

NS_OBJECT_ENSURE_REGISTERED(MacTimeTag);

MacTimeTag::MacTimeTag()
    : m_senderTimestamp(Seconds(0))
{
    // Nothing to do here
}

MacTimeTag::MacTimeTag(Time senderTimestamp)
    : m_senderTimestamp(senderTimestamp)
{
    // Nothing to do here
}

TypeId
MacTimeTag::GetTypeId(void)
{
    static TypeId tid = TypeId("ns3::MacTimeTag").SetParent<Tag>().AddConstructor<MacTimeTag>();
    return tid;
}

TypeId
MacTimeTag::GetInstanceTypeId(void) const
{
    return GetTypeId();
}

uint32_t
MacTimeTag::GetSerializedSize(void) const
{
    return sizeof(Time);
}

void
MacTimeTag::Serialize(TagBuffer i) const
{
    int64_t senderTimestamp = m_senderTimestamp.GetNanoSeconds();
    i.Write((const uint8_t*)&senderTimestamp, sizeof(int64_t));
}

void
MacTimeTag::Deserialize(TagBuffer i)
{
    int64_t senderTimestamp;
    i.Read((uint8_t*)&senderTimestamp, 8);
    m_senderTimestamp = NanoSeconds(senderTimestamp);
}

void
MacTimeTag::Print(std::ostream& os) const
{
    os << m_senderTimestamp;
}

Time
MacTimeTag::GetSenderTimestamp(void) const
{
    return m_senderTimestamp;
}

void
MacTimeTag::SetSenderTimestamp(Time senderTimestamp)
{
    this->m_senderTimestamp = senderTimestamp;
}

// DEV ////////////////////////////////////////////////////////////////////////

NS_OBJECT_ENSURE_REGISTERED(DevTimeTag);

DevTimeTag::DevTimeTag()
    : m_senderTimestamp(Seconds(0))
{
    // Nothing to do here
}

DevTimeTag::DevTimeTag(Time senderTimestamp)
    : m_senderTimestamp(senderTimestamp)
{
    // Nothing to do here
}

TypeId
DevTimeTag::GetTypeId(void)
{
    static TypeId tid = TypeId("ns3::DevTimeTag").SetParent<Tag>().AddConstructor<DevTimeTag>();
    return tid;
}

TypeId
DevTimeTag::GetInstanceTypeId(void) const
{
    return GetTypeId();
}

uint32_t
DevTimeTag::GetSerializedSize(void) const
{
    return sizeof(Time);
}

void
DevTimeTag::Serialize(TagBuffer i) const
{
    int64_t senderTimestamp = m_senderTimestamp.GetNanoSeconds();
    i.Write((const uint8_t*)&senderTimestamp, sizeof(int64_t));
}

void
DevTimeTag::Deserialize(TagBuffer i)
{
    int64_t senderTimestamp;
    i.Read((uint8_t*)&senderTimestamp, 8);
    m_senderTimestamp = NanoSeconds(senderTimestamp);
}

void
DevTimeTag::Print(std::ostream& os) const
{
    os << m_senderTimestamp;
}

Time
DevTimeTag::GetSenderTimestamp(void) const
{
    return m_senderTimestamp;
}

void
DevTimeTag::SetSenderTimestamp(Time senderTimestamp)
{
    this->m_senderTimestamp = senderTimestamp;
}

} // namespace ns3
