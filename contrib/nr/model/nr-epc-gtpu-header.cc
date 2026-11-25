// Copyright (c) 2011 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only
//
// Author: Jaume Nin <jnin@cttc.cat>

#include "nr-epc-gtpu-header.h"

#include "ns3/log.h"
#include "ns3/packet.h"

#include <algorithm>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("NrGtpuHeader");

/********************************************************
 *        GTP-U-v1 Header
 ********************************************************/

NS_OBJECT_ENSURE_REGISTERED(NrGtpuHeader);

TypeId
NrGtpuHeader::GetTypeId()
{
    static TypeId tid = TypeId("ns3::NrGtpuHeader")
                            .SetParent<Header>()
                            .SetGroupName("Nr")
                            .AddConstructor<NrGtpuHeader>();
    return tid;
}

NrGtpuHeader::NrGtpuHeader()
    : m_version(1),
      m_protocolType(true),
      m_sequenceNumberFlag(true),
      m_nPduNumberFlag(true),
      m_messageType(255),
      m_length(0),
      m_teid(0),
      m_sequenceNumber(0),
      m_nPduNumber(0)
{
}

NrGtpuHeader::~NrGtpuHeader()
{
}

TypeId
NrGtpuHeader::GetInstanceTypeId() const
{
    return GetTypeId();
}

bool
NrGtpuHeader::HasOptionalFields() const
{
    return !m_extensionHeaders.empty() || m_sequenceNumberFlag || m_nPduNumberFlag;
}

uint32_t
NrGtpuHeader::GetSerializedSize() const
{
    // mandatory part, then the optional fields and the extension headers (TS 29.281, 5.1)
    uint32_t size = HasOptionalFields() ? 12 : 8;
    for (const auto& extension : m_extensionHeaders)
    {
        size += extension.content.size() + 2;
    }
    return size;
}

void
NrGtpuHeader::Serialize(Buffer::Iterator start) const
{
    Buffer::Iterator i = start;
    uint8_t firstByte = m_version << 5 | m_protocolType << 4;
    firstByte |= GetExtensionHeaderFlag() << 2 | m_sequenceNumberFlag << 1 | m_nPduNumberFlag;
    i.WriteU8(firstByte);
    i.WriteU8(m_messageType);
    i.WriteHtonU16(m_length);
    i.WriteHtonU32(m_teid);
    if (!HasOptionalFields())
    {
        return;
    }
    i.WriteHtonU16(m_sequenceNumber);
    i.WriteU8(m_nPduNumber);
    i.WriteU8(GetNextExtensionType());
    for (size_t e = 0; e < m_extensionHeaders.size(); e++)
    {
        const auto& content = m_extensionHeaders[e].content;
        // the length is in 4-octet units, including the length and next type octets
        i.WriteU8(static_cast<uint8_t>((content.size() + 2) / 4));
        for (auto octet : content)
        {
            i.WriteU8(octet);
        }
        i.WriteU8(e + 1 < m_extensionHeaders.size() ? m_extensionHeaders[e + 1].type : 0);
    }
}

uint32_t
NrGtpuHeader::Deserialize(Buffer::Iterator start)
{
    Buffer::Iterator i = start;
    uint8_t firstByte = i.ReadU8();
    m_version = firstByte >> 5 & 0x7;
    m_protocolType = firstByte >> 4 & 0x1;
    const bool extensionHeaderFlag = firstByte >> 2 & 0x1;
    m_sequenceNumberFlag = firstByte >> 1 & 0x1;
    m_nPduNumberFlag = firstByte & 0x1;
    m_messageType = i.ReadU8();
    m_length = i.ReadNtohU16();
    m_teid = i.ReadNtohU32();
    m_extensionHeaders.clear();
    m_sequenceNumber = 0;
    m_nPduNumber = 0;
    if (!extensionHeaderFlag && !m_sequenceNumberFlag && !m_nPduNumberFlag)
    {
        return 8;
    }
    // the fields are present if any flag is set, and only interpreted if their flag is set
    const uint16_t sequenceNumber = i.ReadNtohU16();
    const uint8_t nPduNumber = i.ReadU8();
    uint8_t nextExtensionType = i.ReadU8();
    m_sequenceNumber = m_sequenceNumberFlag ? sequenceNumber : 0;
    m_nPduNumber = m_nPduNumberFlag ? nPduNumber : 0;
    uint32_t size = 12;
    while (extensionHeaderFlag && nextExtensionType != 0)
    {
        const uint32_t length = i.ReadU8() * 4;
        NS_ABORT_MSG_IF(length == 0, "Malformed GTP-U extension header");
        ExtensionHeader extension{nextExtensionType, std::vector<uint8_t>(length - 2)};
        for (auto& octet : extension.content)
        {
            octet = i.ReadU8();
        }
        m_extensionHeaders.push_back(std::move(extension));
        nextExtensionType = i.ReadU8();
        size += length;
    }
    return size;
}

void
NrGtpuHeader::Print(std::ostream& os) const
{
    os << " version=" << (uint32_t)m_version << " [";
    if (m_protocolType)
    {
        os << " PT ";
    }
    if (GetExtensionHeaderFlag())
    {
        os << " E ";
    }
    if (m_sequenceNumberFlag)
    {
        os << " S ";
    }
    if (m_nPduNumberFlag)
    {
        os << " PN ";
    }
    os << "], messageType=" << (uint32_t)m_messageType << ", length=" << (uint32_t)m_length;
    os << ", teid=" << (uint32_t)m_teid << ", sequenceNumber=" << (uint32_t)m_sequenceNumber;
    os << ", nPduNumber=" << (uint32_t)m_nPduNumber
       << ", nextExtensionType=" << +GetNextExtensionType();
    for (const auto& extension : m_extensionHeaders)
    {
        os << ", extension type=" << +extension.type << " length=" << extension.content.size() + 2;
    }
    if (HasPduSessionContainer())
    {
        os << ", pduType=" << +GetPduType() << ", qfi=" << +GetQfi();
    }
}

bool
NrGtpuHeader::GetExtensionHeaderFlag() const
{
    return !m_extensionHeaders.empty();
}

uint16_t
NrGtpuHeader::GetLength() const
{
    return m_length;
}

uint8_t
NrGtpuHeader::GetMessageType() const
{
    return m_messageType;
}

uint8_t
NrGtpuHeader::GetNPduNumber() const
{
    return m_nPduNumber;
}

bool
NrGtpuHeader::GetNPduNumberFlag() const
{
    return m_nPduNumberFlag;
}

uint8_t
NrGtpuHeader::GetNextExtensionType() const
{
    return m_extensionHeaders.empty() ? 0 : m_extensionHeaders.front().type;
}

bool
NrGtpuHeader::GetProtocolType() const
{
    return m_protocolType;
}

uint16_t
NrGtpuHeader::GetSequenceNumber() const
{
    return m_sequenceNumber;
}

bool
NrGtpuHeader::GetSequenceNumberFlag() const
{
    return m_sequenceNumberFlag;
}

uint32_t
NrGtpuHeader::GetTeid() const
{
    return m_teid;
}

uint8_t
NrGtpuHeader::GetVersion() const
{
    return m_version;
}

void
NrGtpuHeader::SetLength(uint16_t m_length)
{
    this->m_length = m_length;
}

void
NrGtpuHeader::SetMessageType(uint8_t m_messageType)
{
    this->m_messageType = m_messageType;
}

void
NrGtpuHeader::SetNPduNumber(uint8_t m_nPduNumber)
{
    this->m_nPduNumber = m_nPduNumber;
}

void
NrGtpuHeader::SetNPduNumberFlag(bool m_nPduNumberFlag)
{
    this->m_nPduNumberFlag = m_nPduNumberFlag;
}

void
NrGtpuHeader::SetProtocolType(bool m_protocolType)
{
    this->m_protocolType = m_protocolType;
}

void
NrGtpuHeader::SetSequenceNumber(uint16_t m_sequenceNumber)
{
    this->m_sequenceNumber = m_sequenceNumber;
}

void
NrGtpuHeader::SetSequenceNumberFlag(bool m_sequenceNumberFlag)
{
    this->m_sequenceNumberFlag = m_sequenceNumberFlag;
}

void
NrGtpuHeader::SetTeid(uint32_t m_teid)
{
    this->m_teid = m_teid;
}

void
NrGtpuHeader::SetVersion(uint8_t m_version)
{
    // m_version is a uint3_t
    this->m_version = m_version & 0x7;
}

bool
NrGtpuHeader::operator==(const NrGtpuHeader& b) const
{
    return m_version == b.m_version && m_protocolType == b.m_protocolType &&
           m_sequenceNumberFlag == b.m_sequenceNumberFlag &&
           m_nPduNumberFlag == b.m_nPduNumberFlag && m_messageType == b.m_messageType &&
           m_length == b.m_length && m_teid == b.m_teid && m_sequenceNumber == b.m_sequenceNumber &&
           m_nPduNumber == b.m_nPduNumber && m_extensionHeaders == b.m_extensionHeaders;
}

void
NrGtpuHeader::AddExtensionHeader(uint8_t type, const std::vector<uint8_t>& content)
{
    NS_ASSERT_MSG(type != 0, "Type 0 means no more extension headers");
    ExtensionHeader extension{type, content};
    // padding to a multiple of 4 octets, with the length and next type octets
    extension.content.resize((content.size() + 2 + 3) / 4 * 4 - 2, 0);
    NS_ASSERT_MSG(extension.content.size() + 2 <= 255 * 4, "Extension header too long");
    m_extensionHeaders.push_back(std::move(extension));
}

const std::vector<NrGtpuHeader::ExtensionHeader>&
NrGtpuHeader::GetExtensionHeaders() const
{
    return m_extensionHeaders;
}

const NrGtpuHeader::ExtensionHeader*
NrGtpuHeader::FindPduSessionContainer() const
{
    auto it = std::find_if(m_extensionHeaders.begin(),
                           m_extensionHeaders.end(),
                           [](const auto& extension) {
                               return extension.type == PDU_SESSION_CONTAINER &&
                                      extension.content.size() >= 2;
                           });
    return it == m_extensionHeaders.end() ? nullptr : &*it;
}

void
NrGtpuHeader::SetPduSessionContainer(PduType pduType, uint8_t qfi)
{
    std::erase_if(m_extensionHeaders,
                  [](const auto& extension) { return extension.type == PDU_SESSION_CONTAINER; });
    // PDU type in the first octet, QFI in the 6 least significant bits of the second one, in
    // both directions (TS 38.415, sections 5.5.2.1 and 5.5.2.2)
    AddExtensionHeader(PDU_SESSION_CONTAINER,
                       {static_cast<uint8_t>(pduType << 4), static_cast<uint8_t>(qfi & 0x3f)});
}

bool
NrGtpuHeader::HasPduSessionContainer() const
{
    return FindPduSessionContainer() != nullptr;
}

NrGtpuHeader::PduType
NrGtpuHeader::GetPduType() const
{
    const auto* container = FindPduSessionContainer();
    NS_ASSERT_MSG(container, "No PDU Session Container");
    return static_cast<PduType>(container->content[0] >> 4);
}

uint8_t
NrGtpuHeader::GetQfi() const
{
    const auto* container = FindPduSessionContainer();
    NS_ASSERT_MSG(container, "No PDU Session Container");
    return container->content[1] & 0x3f;
}

} // namespace ns3
