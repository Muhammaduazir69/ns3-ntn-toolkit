/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2017 Magister Solutions
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

#include "address-tag.h"

#include "ns3/log.h"

#include <ostream>

NS_LOG_COMPONENT_DEFINE("AddressTag");

namespace ns3
{

NS_OBJECT_ENSURE_REGISTERED(AddressTag);

AddressTag::AddressTag()
    : Tag()
{
    NS_LOG_FUNCTION(this);
}

AddressTag::AddressTag(Address addr)
    : Tag(),
      m_sourceAddress(addr)
{
    NS_LOG_FUNCTION(this);
}

TypeId
AddressTag::GetTypeId(void)
{
    static TypeId tid = TypeId("ns3::AddressTag").SetParent<Tag>().AddConstructor<AddressTag>();
    return tid;
}

TypeId
AddressTag::GetInstanceTypeId() const
{
    return GetTypeId();
}

uint32_t
AddressTag::GetSerializedSize() const
{
    return m_sourceAddress.GetSerializedSize();
}

void
AddressTag::Serialize(TagBuffer i) const
{
    NS_LOG_FUNCTION(this << &i);

    m_sourceAddress.Serialize(i);
}

void
AddressTag::Deserialize(TagBuffer i)
{
    NS_LOG_FUNCTION(this << &i);

    m_sourceAddress.Deserialize(i);
}

void
AddressTag::Print(std::ostream& os) const
{
    NS_LOG_FUNCTION(this << &os);
    os << "(SourceAddress=" << m_sourceAddress << ")";
}

void
AddressTag::SetSourceAddress(Address addr)
{
    NS_LOG_FUNCTION(this << addr);
    m_sourceAddress = addr;
}

Address
AddressTag::GetSourceAddress() const
{
    return m_sourceAddress;
}

} // namespace ns3
