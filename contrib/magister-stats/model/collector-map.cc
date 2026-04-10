/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2014 Magister Solutions
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
 * Author: Budiarto Herman <budiarto.herman@magister.fi>
 *
 */

#include "collector-map.h"

#include "ns3/fatal-error.h"
#include "ns3/log.h"

#include <string>

NS_LOG_COMPONENT_DEFINE("CollectorMap");

namespace ns3
{

CollectorMap::CollectorMap()
{
    NS_LOG_FUNCTION(this);
}

void
CollectorMap::SetType(std::string type)
{
    NS_LOG_FUNCTION(this << type);

    TypeId tid;
    const bool isValidType = TypeId::LookupByNameFailSafe(type, &tid);

    if (!isValidType)
    {
        NS_FATAL_ERROR("Invalid type " << type);
    }
    else
    {
        TypeId baseTid = TypeId::LookupByName("ns3::DataCollectionObject");
        const bool isCollector = tid.IsChildOf(baseTid);

        if (!isCollector)
        {
            NS_FATAL_ERROR("Type " << type << " is not a child of" << " ns3::DataCollectionObject");
        }
        else
        {
            m_factory.SetTypeId(tid);
        }
    }
}

TypeId
CollectorMap::GetType() const
{
    return m_factory.GetTypeId();
}

void
CollectorMap::SetAttribute(std::string n, const AttributeValue& v)
{
    NS_LOG_FUNCTION(this << n);
    m_factory.Set(n, v);
}

void
CollectorMap::Create(uint32_t identifier)
{
    NS_LOG_FUNCTION(this << identifier);
    m_map[identifier] = m_factory.Create()->GetObject<DataCollectionObject>();
}

void
CollectorMap::Insert(uint32_t identifier, Ptr<DataCollectionObject> dataCollectionObject)
{
    NS_LOG_FUNCTION(this << identifier);
    m_map[identifier] = dataCollectionObject;
}

bool
CollectorMap::IsEmpty() const
{
    return m_map.empty();
}

bool
CollectorMap::IsExists(uint32_t identifier) const
{
    CollectorMap::Iterator it = m_map.find(identifier);
    return it != m_map.end();
}

uint32_t
CollectorMap::GetN() const
{
    return m_map.size();
}

CollectorMap::Iterator
CollectorMap::Begin() const
{
    return m_map.begin();
}

CollectorMap::Iterator
CollectorMap::End() const
{
    return m_map.end();
}

Ptr<DataCollectionObject>
CollectorMap::Get(uint32_t identifier) const
{
    CollectorMap::Iterator it = m_map.find(identifier);

    if (it == m_map.end())
    {
        NS_LOG_WARN(this << " cannot find collector with identifier " << identifier);
        return 0;
    }
    else
    {
        return it->second;
    }
}

} // end of namespace ns3
