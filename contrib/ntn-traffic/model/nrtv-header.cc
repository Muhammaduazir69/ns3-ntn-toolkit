/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2013 Magister Solutions
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

#include "nrtv-header.h"

#include "ns3/log.h"
#include "ns3/simulator.h"

#include <ostream>

NS_LOG_COMPONENT_DEFINE("NrtvHeader");

namespace ns3
{

NS_OBJECT_ENSURE_REGISTERED(NrtvHeader);

NrtvHeader::NrtvHeader()
    : m_frameNumber(0),
      m_numOfFrames(0),
      m_sliceNumber(0),
      m_numOfSlices(0),
      m_sliceSize(0),
      m_arrivalTime(Simulator::Now())
{
    NS_LOG_FUNCTION(this);
}

TypeId
NrtvHeader::GetTypeId(void)
{
    static TypeId tid = TypeId("ns3::NrtvHeader").SetParent<Header>().AddConstructor<NrtvHeader>();
    return tid;
}

void
NrtvHeader::SetFrameNumber(uint32_t frameNumber)
{
    NS_LOG_FUNCTION(this << frameNumber);
    m_frameNumber = frameNumber;
}

uint32_t
NrtvHeader::GetFrameNumber() const
{
    return m_frameNumber;
}

void
NrtvHeader::SetNumOfFrames(uint32_t numOfFrames)
{
    NS_LOG_FUNCTION(this << numOfFrames);
    m_numOfFrames = numOfFrames;
}

uint32_t
NrtvHeader::GetNumOfFrames() const
{
    return m_numOfFrames;
}

void
NrtvHeader::SetSliceNumber(uint16_t sliceNumber)
{
    NS_LOG_FUNCTION(this << sliceNumber);
    m_sliceNumber = sliceNumber;
}

uint16_t
NrtvHeader::GetSliceNumber() const
{
    return m_sliceNumber;
}

void
NrtvHeader::SetNumOfSlices(uint16_t numOfSlices)
{
    NS_LOG_FUNCTION(this << numOfSlices);
    m_numOfSlices = numOfSlices;
}

uint16_t
NrtvHeader::GetNumOfSlices() const
{
    return m_numOfSlices;
}

void
NrtvHeader::SetSliceSize(uint32_t sliceSize)
{
    NS_LOG_FUNCTION(this << sliceSize);
    m_sliceSize = sliceSize;
}

uint32_t
NrtvHeader::GetSliceSize() const
{
    return m_sliceSize;
}

Time
NrtvHeader::GetArrivalTime() const
{
    return m_arrivalTime;
}

uint32_t
NrtvHeader::GetSerializedSize() const
{
    return 24;
}

void
NrtvHeader::Print(std::ostream& os) const
{
    os << "(frameNumber: " << m_frameNumber << " numOfFrames: " << m_numOfFrames
       << " sliceNumber: " << m_sliceNumber << " numOfSlices: " << m_numOfSlices
       << " sliceSize: " << m_sliceSize << " arrivalTime: " << m_arrivalTime << ")";
}

void
NrtvHeader::Serialize(Buffer::Iterator start) const
{
    NS_LOG_FUNCTION(this << &start);
    Buffer::Iterator i = start;
    i.WriteHtonU32(m_frameNumber);
    i.WriteHtonU32(m_numOfFrames);
    i.WriteHtonU16(m_sliceNumber);
    i.WriteHtonU16(m_numOfSlices);
    i.WriteHtonU32(m_sliceSize);
    i.WriteHtonU64(m_arrivalTime.GetNanoSeconds());
}

uint32_t
NrtvHeader::Deserialize(Buffer::Iterator start)
{
    NS_LOG_FUNCTION(this << &start);
    Buffer::Iterator i = start;
    m_frameNumber = i.ReadNtohU32();
    m_numOfFrames = i.ReadNtohU32();
    m_sliceNumber = i.ReadNtohU16();
    m_numOfSlices = i.ReadNtohU16();
    m_sliceSize = i.ReadNtohU32();
    m_arrivalTime = NanoSeconds(i.ReadNtohU64());
    return GetSerializedSize();
}

TypeId
NrtvHeader::GetInstanceTypeId() const
{
    return GetTypeId();
}

} // namespace ns3
