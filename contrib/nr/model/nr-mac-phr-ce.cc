// Copyright (c) 2026 Muhammad Uzair
//
// SPDX-License-Identifier: GPL-2.0-only

#include "nr-mac-phr-ce.h"

#include "ns3/log.h"

#include <cmath>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("NrMacPhrCe");

NS_OBJECT_ENSURE_REGISTERED(NrMacSingleEntryPhrCe);
NS_OBJECT_ENSURE_REGISTERED(NrPuschAllocationTag);

TypeId
NrMacSingleEntryPhrCe::GetTypeId()
{
    static TypeId tid = TypeId("ns3::NrMacSingleEntryPhrCe")
                            .SetParent<Header>()
                            .SetGroupName("Nr")
                            .AddConstructor<NrMacSingleEntryPhrCe>();
    return tid;
}

TypeId
NrMacSingleEntryPhrCe::GetInstanceTypeId() const
{
    return GetTypeId();
}

NrMacSingleEntryPhrCe::NrMacSingleEntryPhrCe()
{
    m_header.SetLcId(NrMacHeaderFsUl::SINGLE_ENTRY_PHR);
}

void
NrMacSingleEntryPhrCe::Serialize(Buffer::Iterator start) const
{
    NS_ASSERT(m_phIndex <= 63);
    NS_ASSERT(m_pcmaxIndex <= 63);
    m_header.Serialize(start);
    start.Next(m_header.GetSerializedSize());
    start.WriteU8(m_phIndex & 0x3F);    // R R PH
    start.WriteU8(m_pcmaxIndex & 0x3F); // R R P_CMAX,f,c
}

uint32_t
NrMacSingleEntryPhrCe::Deserialize(Buffer::Iterator start)
{
    auto readBytes = m_header.Deserialize(start);
    start.Next(readBytes);
    NS_ASSERT(m_header.GetLcId() == NrMacHeaderFsUl::SINGLE_ENTRY_PHR);
    m_phIndex = start.ReadU8() & 0x3F;
    m_pcmaxIndex = start.ReadU8() & 0x3F;
    return GetSerializedSize();
}

uint32_t
NrMacSingleEntryPhrCe::GetSerializedSize() const
{
    return m_header.GetSerializedSize() + 2;
}

void
NrMacSingleEntryPhrCe::Print(std::ostream& os) const
{
    os << "PH index " << +m_phIndex << " (" << PhIndexToDb(m_phIndex) << " dB), PCMAX index "
       << +m_pcmaxIndex << " (" << PcmaxIndexToDbm(m_pcmaxIndex) << " dBm)";
}

uint8_t
NrMacSingleEntryPhrCe::PhDbToIndex(double phDb)
{
    // TS 38.133 Table 10.1.17.1-1: 1 dB steps from -32 dB to 22 dB, then
    // 2 dB steps up to 38 dB.
    if (!(phDb >= -32.0))
    {
        return 0;
    }
    if (phDb >= 38.0)
    {
        return 63;
    }
    if (phDb < 22.0)
    {
        return static_cast<uint8_t>(std::floor(phDb) + 33.0);
    }
    return static_cast<uint8_t>(55.0 + std::floor((phDb - 22.0) / 2.0));
}

double
NrMacSingleEntryPhrCe::PhIndexToDb(uint8_t index)
{
    if (index == 0)
    {
        return -33.0;
    }
    if (index <= 54)
    {
        return static_cast<double>(index) - 33.0;
    }
    if (index <= 62)
    {
        return 22.0 + 2.0 * (index - 55);
    }
    return 38.0;
}

uint8_t
NrMacSingleEntryPhrCe::PcmaxDbmToIndex(double pcmaxDbm)
{
    // TS 38.133 Table 10.1.18.1-1: 1 dB steps from -29 dBm to 33 dBm.
    if (!(pcmaxDbm >= -29.0))
    {
        return 0;
    }
    if (pcmaxDbm >= 33.0)
    {
        return 63;
    }
    return static_cast<uint8_t>(std::floor(pcmaxDbm) + 30.0);
}

double
NrMacSingleEntryPhrCe::PcmaxIndexToDbm(uint8_t index)
{
    return static_cast<double>(std::min<uint8_t>(index, 63)) - 30.0;
}

TypeId
NrPuschAllocationTag::GetTypeId()
{
    static TypeId tid = TypeId("ns3::NrPuschAllocationTag")
                            .SetParent<Tag>()
                            .SetGroupName("Nr")
                            .AddConstructor<NrPuschAllocationTag>();
    return tid;
}

TypeId
NrPuschAllocationTag::GetInstanceTypeId() const
{
    return GetTypeId();
}

NrPuschAllocationTag::NrPuschAllocationTag(uint16_t numRb)
    : m_numRb(numRb)
{
}

uint32_t
NrPuschAllocationTag::GetSerializedSize() const
{
    return 2;
}

void
NrPuschAllocationTag::Serialize(TagBuffer i) const
{
    i.WriteU16(m_numRb);
}

void
NrPuschAllocationTag::Deserialize(TagBuffer i)
{
    m_numRb = i.ReadU16();
}

void
NrPuschAllocationTag::Print(std::ostream& os) const
{
    os << "numRb=" << m_numRb;
}

uint16_t
NrPuschAllocationTag::GetNumRb() const
{
    return m_numRb;
}

} // namespace ns3
