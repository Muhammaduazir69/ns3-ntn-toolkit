// SPDX-License-Identifier: GPL-2.0-only
#include "ntn-oran-payload-header.h"

#include "ns3/log.h"

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("NtnOranPayloadHeader");
NS_OBJECT_ENSURE_REGISTERED(NtnOranPayloadHeader);

TypeId
NtnOranPayloadHeader::GetTypeId()
{
    static TypeId tid = TypeId("ns3::NtnOranPayloadHeader")
                            .SetParent<Header>()
                            .SetGroupName("NtnTraffic")
                            .AddConstructor<NtnOranPayloadHeader>();
    return tid;
}

TypeId
NtnOranPayloadHeader::GetInstanceTypeId() const
{
    return GetTypeId();
}

void
NtnOranPayloadHeader::Print(std::ostream& os) const
{
    os << "v" << +m_version << " type=" << +m_payloadType << " seq=" << m_seq
       << " txNs=" << m_txTimestampNs << " 5qi=" << +m_fiveQi << " snssai=" << +m_sst << "/"
       << m_sd << " qfi=" << +m_qfi << " src=" << m_srcId << " dst=" << m_dstId;
}

uint32_t
NtnOranPayloadHeader::GetSerializedSize() const
{
    return SERIALIZED_SIZE;
}

void
NtnOranPayloadHeader::Serialize(Buffer::Iterator start) const
{
    Buffer::Iterator i = start;
    i.WriteU8(m_version);
    i.WriteU8(m_payloadType);
    i.WriteHtonU32(m_seq);
    i.WriteHtonU64(m_txTimestampNs);
    i.WriteU8(m_fiveQi);
    i.WriteU8(m_sst);
    // S-NSSAI SD is 24 bits on the wire (TS 23.003).
    i.WriteU8((m_sd >> 16) & 0xFF);
    i.WriteU8((m_sd >> 8) & 0xFF);
    i.WriteU8(m_sd & 0xFF);
    i.WriteU8(m_qfi & 0x3F);
    i.WriteHtonU16(m_srcId);
    i.WriteHtonU16(m_dstId);
}

uint32_t
NtnOranPayloadHeader::Deserialize(Buffer::Iterator start)
{
    Buffer::Iterator i = start;
    m_version = i.ReadU8();
    m_payloadType = i.ReadU8();
    m_seq = i.ReadNtohU32();
    m_txTimestampNs = i.ReadNtohU64();
    m_fiveQi = i.ReadU8();
    m_sst = i.ReadU8();
    uint32_t sd = i.ReadU8();
    sd = (sd << 8) | i.ReadU8();
    sd = (sd << 8) | i.ReadU8();
    m_sd = sd;
    m_qfi = i.ReadU8() & 0x3F;
    m_srcId = i.ReadNtohU16();
    m_dstId = i.ReadNtohU16();
    return SERIALIZED_SIZE;
}

} // namespace ns3
