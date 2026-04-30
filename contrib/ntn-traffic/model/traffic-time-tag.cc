/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2014 Magister Solutions Ltd.
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
 * Author: Jani Puttonen <jani.puttonen@magister.fi>
 */

#include "traffic-time-tag.h"

#include <ostream>

namespace ns3
{

NS_OBJECT_ENSURE_REGISTERED(TrafficTimeTag);

TrafficTimeTag::TrafficTimeTag()
    : m_senderTimestamp(Seconds(0))
{
    // Nothing to do here
}

TrafficTimeTag::TrafficTimeTag(Time senderTimestamp)
    : m_senderTimestamp(senderTimestamp)
{
    // Nothing to do here
}

TypeId
TrafficTimeTag::GetTypeId(void)
{
    static TypeId tid =
        TypeId("ns3::TrafficTimeTag").SetParent<Tag>().AddConstructor<TrafficTimeTag>();
    return tid;
}

TypeId
TrafficTimeTag::GetInstanceTypeId(void) const
{
    return GetTypeId();
}

uint32_t
TrafficTimeTag::GetSerializedSize(void) const
{
    return sizeof(Time);
}

void
TrafficTimeTag::Serialize(TagBuffer i) const
{
    int64_t senderTimestamp = m_senderTimestamp.GetNanoSeconds();
    i.Write((const uint8_t*)&senderTimestamp, sizeof(int64_t));
}

void
TrafficTimeTag::Deserialize(TagBuffer i)
{
    int64_t senderTimestamp;
    i.Read((uint8_t*)&senderTimestamp, 8);
    m_senderTimestamp = NanoSeconds(senderTimestamp);
}

void
TrafficTimeTag::Print(std::ostream& os) const
{
    os << m_senderTimestamp;
}

Time
TrafficTimeTag::GetSenderTimestamp(void) const
{
    return m_senderTimestamp;
}

void
TrafficTimeTag::SetSenderTimestamp(Time senderTimestamp)
{
    this->m_senderTimestamp = senderTimestamp;
}

} // namespace ns3
