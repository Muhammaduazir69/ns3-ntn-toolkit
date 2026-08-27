/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#include "ntn-xnap-messages.h"

namespace ns3
{
namespace ntncon
{

const char*
XnapMessageTypeName(XnapMessageType t)
{
    switch (t)
    {
    case XnapMessageType::HandoverRequest:
        return "HANDOVER REQUEST";
    case XnapMessageType::HandoverRequestAcknowledge:
        return "HANDOVER REQUEST ACKNOWLEDGE";
    case XnapMessageType::HandoverPreparationFailure:
        return "HANDOVER PREPARATION FAILURE";
    case XnapMessageType::SnStatusTransfer:
        return "SN STATUS TRANSFER";
    case XnapMessageType::UeContextRelease:
        return "UE CONTEXT RELEASE";
    case XnapMessageType::HandoverCancel:
        return "HANDOVER CANCEL";
    }
    return "UNKNOWN";
}

NS_OBJECT_ENSURE_REGISTERED(NtnXnapHeader);

TypeId
NtnXnapHeader::GetTypeId()
{
    static TypeId tid = TypeId("ns3::ntncon::NtnXnapHeader")
                            .SetParent<Header>()
                            .SetGroupName("NtnConstellation")
                            .AddConstructor<NtnXnapHeader>();
    return tid;
}

TypeId
NtnXnapHeader::GetInstanceTypeId() const
{
    return GetTypeId();
}

uint32_t
NtnXnapHeader::GetSerializedSize() const
{
    // type(1) + srcUe(4) + tgtUe(4) + srcGnb(4) + tgtGnb(4) + nrcgi(8)
    // + dlSn(4) + ulSn(4) + hfn(4) + cause(1)
    return 1 + 4 + 4 + 4 + 4 + 8 + 4 + 4 + 4 + 1;
}

void
NtnXnapHeader::Serialize(Buffer::Iterator i) const
{
    i.WriteU8(static_cast<uint8_t>(m_type));
    i.WriteHtonU32(m_srcUeId);
    i.WriteHtonU32(m_tgtUeId);
    i.WriteHtonU32(m_srcGnb);
    i.WriteHtonU32(m_tgtGnb);
    i.WriteHtonU64(m_targetNrCgi);
    i.WriteHtonU32(m_dlPdcpSn);
    i.WriteHtonU32(m_ulPdcpSn);
    i.WriteHtonU32(m_hfn);
    i.WriteU8(m_cause);
}

uint32_t
NtnXnapHeader::Deserialize(Buffer::Iterator i)
{
    m_type = static_cast<XnapMessageType>(i.ReadU8());
    m_srcUeId = i.ReadNtohU32();
    m_tgtUeId = i.ReadNtohU32();
    m_srcGnb = i.ReadNtohU32();
    m_tgtGnb = i.ReadNtohU32();
    m_targetNrCgi = i.ReadNtohU64();
    m_dlPdcpSn = i.ReadNtohU32();
    m_ulPdcpSn = i.ReadNtohU32();
    m_hfn = i.ReadNtohU32();
    m_cause = i.ReadU8();
    return GetSerializedSize();
}

void
NtnXnapHeader::Print(std::ostream& os) const
{
    os << XnapMessageTypeName(m_type) << " srcUeXnapId=" << m_srcUeId
       << " tgtUeXnapId=" << m_tgtUeId << " gnb " << m_srcGnb << "->" << m_tgtGnb
       << " nrcgi=" << m_targetNrCgi;
}

} // namespace ntncon
} // namespace ns3
