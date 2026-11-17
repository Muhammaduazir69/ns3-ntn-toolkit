/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#include "sagin-bundle-protocol.h"

#include "ns3/abort.h"
#include "ns3/log.h"

namespace ns3
{
namespace ntnsagin
{

NS_LOG_COMPONENT_DEFINE("SaginBundleProtocol");
NS_OBJECT_ENSURE_REGISTERED(SaginBundleHeader);
NS_OBJECT_ENSURE_REGISTERED(SaginBundleTrailer);
NS_OBJECT_ENSURE_REGISTERED(SaginCustodySignalHeader);

namespace cbor
{
// RFC 8949 3: major types used here.
constexpr uint8_t kUint = 0;
constexpr uint8_t kBytes = 2;
constexpr uint8_t kArray = 4;
constexpr uint8_t kSimpleFalse = 0xF4;
constexpr uint8_t kSimpleTrue = 0xF5;

uint32_t
HeadSize(uint64_t v)
{
    if (v < 24)
    {
        return 1;
    }
    if (v <= 0xFF)
    {
        return 2;
    }
    if (v <= 0xFFFF)
    {
        return 3;
    }
    if (v <= 0xFFFFFFFFULL)
    {
        return 5;
    }
    return 9;
}

void
WriteHead(Buffer::Iterator& i, uint8_t major, uint64_t v)
{
    const uint8_t mt = static_cast<uint8_t>(major << 5);
    if (v < 24)
    {
        i.WriteU8(mt | static_cast<uint8_t>(v));
    }
    else if (v <= 0xFF)
    {
        i.WriteU8(mt | 24);
        i.WriteU8(static_cast<uint8_t>(v));
    }
    else if (v <= 0xFFFF)
    {
        i.WriteU8(mt | 25);
        i.WriteHtonU16(static_cast<uint16_t>(v));
    }
    else if (v <= 0xFFFFFFFFULL)
    {
        i.WriteU8(mt | 26);
        i.WriteHtonU32(static_cast<uint32_t>(v));
    }
    else
    {
        i.WriteU8(mt | 27);
        i.WriteHtonU64(v);
    }
}

uint64_t
ReadHead(Buffer::Iterator& i, uint8_t expectMajor)
{
    const uint8_t b = i.ReadU8();
    NS_ABORT_MSG_IF((b >> 5) != expectMajor,
                    "CBOR: major type " << +(b >> 5) << " where " << +expectMajor << " expected");
    const uint8_t ai = b & 0x1F;
    if (ai < 24)
    {
        return ai;
    }
    switch (ai)
    {
    case 24:
        return i.ReadU8();
    case 25:
        return i.ReadNtohU16();
    case 26:
        return i.ReadNtohU32();
    case 27:
        return i.ReadNtohU64();
    default:
        NS_ABORT_MSG("CBOR: unsupported additional information " << +ai);
    }
    return 0;
}

uint32_t
EidSize(uint64_t node, uint64_t service)
{
    // [2, [node, service]] : array(2), uint 2, array(2), node, service
    return 1 + 1 + 1 + HeadSize(node) + HeadSize(service);
}

void
WriteEid(Buffer::Iterator& i, uint64_t node, uint64_t service)
{
    WriteHead(i, kArray, 2);
    WriteHead(i, kUint, 2); // URI scheme code 2 = ipn (RFC 9171 4.2.5.1)
    WriteHead(i, kArray, 2);
    WriteHead(i, kUint, node);
    WriteHead(i, kUint, service);
}

void
ReadEid(Buffer::Iterator& i, uint64_t& node, uint64_t& service)
{
    NS_ABORT_MSG_IF(ReadHead(i, kArray) != 2, "EID: not a 2-array");
    NS_ABORT_MSG_IF(ReadHead(i, kUint) != 2, "EID: only the ipn scheme is supported");
    NS_ABORT_MSG_IF(ReadHead(i, kArray) != 2, "ipn SSP: not a 2-array");
    node = ReadHead(i, kUint);
    service = ReadHead(i, kUint);
}
} // namespace cbor

// ---------------------------------------------------------------------------
TypeId
SaginBundleHeader::GetTypeId()
{
    static TypeId tid = TypeId("ns3::ntnsagin::SaginBundleHeader")
                            .SetParent<Header>()
                            .SetGroupName("NtnSagin")
                            .AddConstructor<SaginBundleHeader>();
    return tid;
}

TypeId
SaginBundleHeader::GetInstanceTypeId() const
{
    return GetTypeId();
}

uint64_t
SaginBundleHeader::BundleId(uint64_t srcNode, uint64_t creationMs, uint64_t seq)
{
    // RFC 9171 4.2.7: (source, creation timestamp) identifies the bundle.
    uint64_t h = srcNode * 0x9E3779B97F4A7C15ULL;
    h ^= creationMs + 0x7F4A7C159E3779B9ULL + (h << 6) + (h >> 2);
    h ^= seq + 0x9E3779B97F4A7C15ULL + (h << 6) + (h >> 2);
    return h;
}

uint64_t
SaginBundleHeader::BundleId() const
{
    return BundleId(srcNode, creationMs, creationSeq);
}

uint32_t
SaginBundleHeader::GetSerializedSize() const
{
    using namespace cbor;
    uint32_t primary = 1 /*array(8)*/ + HeadSize(7) + HeadSize(flags) + HeadSize(0) +
                       EidSize(destNode, destService) + EidSize(srcNode, srcService) +
                       EidSize(reportNode, reportService) + 1 /*array(2)*/ +
                       HeadSize(creationMs) + HeadSize(creationSeq) + HeadSize(lifetimeMs);
    uint32_t payloadHead = 1 /*array(5)*/ + HeadSize(1) + HeadSize(1) + HeadSize(0) + HeadSize(0) +
                           HeadSize(payloadLength) /*bstr head*/;
    uint32_t age = 0;
    if (hasAgeBlock)
    {
        // [7, 2, 0, 0, bstr(uint age)] (RFC 9171 4.3.2 canonical block, 4.4.2)
        age = 1 + HeadSize(7) + HeadSize(2) + HeadSize(0) + HeadSize(0) +
              HeadSize(HeadSize(ageMs)) + HeadSize(ageMs);
    }
    return 1 /*0x9F*/ + primary + age + payloadHead;
}

void
SaginBundleHeader::Serialize(Buffer::Iterator i) const
{
    using namespace cbor;
    i.WriteU8(0x9F); // indefinite-length array (RFC 9171 4.1)
    WriteHead(i, kArray, 8);
    WriteHead(i, kUint, 7); // version
    WriteHead(i, kUint, flags);
    WriteHead(i, kUint, 0); // CRC type: none
    WriteEid(i, destNode, destService);
    WriteEid(i, srcNode, srcService);
    WriteEid(i, reportNode, reportService);
    WriteHead(i, kArray, 2);
    WriteHead(i, kUint, creationMs);
    WriteHead(i, kUint, creationSeq);
    WriteHead(i, kUint, lifetimeMs);
    if (hasAgeBlock)
    {
        // Bundle Age block (RFC 9171 4.4.2): type 7, block number 2, flags 0,
        // CRC 0, block-type-specific data = bstr holding the CBOR uint age.
        WriteHead(i, kArray, 5);
        WriteHead(i, kUint, 7);
        WriteHead(i, kUint, 2);
        WriteHead(i, kUint, 0);
        WriteHead(i, kUint, 0);
        WriteHead(i, kBytes, HeadSize(ageMs));
        WriteHead(i, kUint, ageMs);
    }
    // Payload block (RFC 9171 4.3.2): [type 1, number 1, flags 0, CRC 0, bstr]
    WriteHead(i, kArray, 5);
    WriteHead(i, kUint, 1);
    WriteHead(i, kUint, 1);
    WriteHead(i, kUint, 0);
    WriteHead(i, kUint, 0);
    WriteHead(i, kBytes, payloadLength); // the ns-3 payload follows
}

uint32_t
SaginBundleHeader::Deserialize(Buffer::Iterator start)
{
    using namespace cbor;
    Buffer::Iterator i = start;
    NS_ABORT_MSG_IF(i.ReadU8() != 0x9F, "bundle: not an indefinite-length array");
    NS_ABORT_MSG_IF(ReadHead(i, kArray) != 8, "primary block: expected 8 items (CRC type 0)");
    NS_ABORT_MSG_IF(ReadHead(i, kUint) != 7, "primary block: version is not 7");
    flags = ReadHead(i, kUint);
    NS_ABORT_MSG_IF(ReadHead(i, kUint) != 0, "primary block: only CRC type 0 supported");
    ReadEid(i, destNode, destService);
    ReadEid(i, srcNode, srcService);
    ReadEid(i, reportNode, reportService);
    NS_ABORT_MSG_IF(ReadHead(i, kArray) != 2, "creation timestamp: not a 2-array");
    creationMs = ReadHead(i, kUint);
    creationSeq = ReadHead(i, kUint);
    lifetimeMs = ReadHead(i, kUint);
    hasAgeBlock = false;
    ageMs = 0;
    NS_ABORT_MSG_IF(ReadHead(i, kArray) != 5, "canonical block: expected 5 items");
    uint64_t type = ReadHead(i, kUint);
    if (type == 7)
    {
        ReadHead(i, kUint); // block number
        ReadHead(i, kUint); // flags
        NS_ABORT_MSG_IF(ReadHead(i, kUint) != 0, "bundle age block: only CRC type 0 supported");
        ReadHead(i, kBytes);
        ageMs = ReadHead(i, kUint);
        hasAgeBlock = true;
        NS_ABORT_MSG_IF(ReadHead(i, kArray) != 5, "payload block: expected 5 items");
        type = ReadHead(i, kUint);
    }
    NS_ABORT_MSG_IF(type != 1, "payload block: type is not 1");
    ReadHead(i, kUint);
    ReadHead(i, kUint);
    ReadHead(i, kUint);
    payloadLength = ReadHead(i, kBytes);
    return i.GetDistanceFrom(start);
}

void
SaginBundleHeader::Print(std::ostream& os) const
{
    os << "bpv7 ipn:" << srcNode << "." << srcService << " -> ipn:" << destNode << "."
       << destService << " ts=[" << creationMs << "," << creationSeq << "] life=" << lifetimeMs
       << "ms flags=0x" << std::hex << flags << std::dec;
    if (hasAgeBlock)
    {
        os << " age=" << ageMs << "ms";
    }
}

// ---------------------------------------------------------------------------
TypeId
SaginBundleTrailer::GetTypeId()
{
    static TypeId tid = TypeId("ns3::ntnsagin::SaginBundleTrailer")
                            .SetParent<Trailer>()
                            .SetGroupName("NtnSagin")
                            .AddConstructor<SaginBundleTrailer>();
    return tid;
}

TypeId
SaginBundleTrailer::GetInstanceTypeId() const
{
    return GetTypeId();
}

void
SaginBundleTrailer::Serialize(Buffer::Iterator start) const
{
    start.Prev(1);
    start.WriteU8(0xFF);
}

uint32_t
SaginBundleTrailer::Deserialize(Buffer::Iterator start)
{
    start.Prev(1);
    NS_ABORT_MSG_IF(start.ReadU8() != 0xFF, "bundle: missing CBOR break");
    return 1;
}

// ---------------------------------------------------------------------------
TypeId
SaginCustodySignalHeader::GetTypeId()
{
    static TypeId tid = TypeId("ns3::ntnsagin::SaginCustodySignalHeader")
                            .SetParent<Header>()
                            .SetGroupName("NtnSagin")
                            .AddConstructor<SaginCustodySignalHeader>();
    return tid;
}

TypeId
SaginCustodySignalHeader::GetInstanceTypeId() const
{
    return GetTypeId();
}

uint32_t
SaginCustodySignalHeader::GetSerializedSize() const
{
    using namespace cbor;
    return 1 /*array(2)*/ + HeadSize(2) + 1 /*array(4)*/ + 1 /*bool*/ + HeadSize(reason) +
           EidSize(srcNode, srcService) + 1 + HeadSize(creationMs) + HeadSize(creationSeq);
}

void
SaginCustodySignalHeader::Serialize(Buffer::Iterator i) const
{
    using namespace cbor;
    WriteHead(i, kArray, 2);
    WriteHead(i, kUint, 2); // RFC 5050 6.1 record type 2: custody signal
    WriteHead(i, kArray, 4);
    i.WriteU8(succeeded ? kSimpleTrue : kSimpleFalse);
    WriteHead(i, kUint, reason);
    WriteEid(i, srcNode, srcService);
    WriteHead(i, kArray, 2);
    WriteHead(i, kUint, creationMs);
    WriteHead(i, kUint, creationSeq);
}

uint32_t
SaginCustodySignalHeader::Deserialize(Buffer::Iterator start)
{
    using namespace cbor;
    Buffer::Iterator i = start;
    NS_ABORT_MSG_IF(ReadHead(i, kArray) != 2, "admin record: not a 2-array");
    NS_ABORT_MSG_IF(ReadHead(i, kUint) != 2, "admin record: not a custody signal");
    NS_ABORT_MSG_IF(ReadHead(i, kArray) != 4, "custody signal: expected 4 items");
    const uint8_t b = i.ReadU8();
    NS_ABORT_MSG_IF(b != kSimpleTrue && b != kSimpleFalse, "custody signal: bad boolean");
    succeeded = (b == kSimpleTrue);
    reason = static_cast<uint8_t>(ReadHead(i, kUint));
    ReadEid(i, srcNode, srcService);
    NS_ABORT_MSG_IF(ReadHead(i, kArray) != 2, "custody signal: bad timestamp");
    creationMs = ReadHead(i, kUint);
    creationSeq = ReadHead(i, kUint);
    return i.GetDistanceFrom(start);
}

void
SaginCustodySignalHeader::Print(std::ostream& os) const
{
    os << "custody-signal " << (succeeded ? "accepted" : "refused") << " reason=" << +reason
       << " bundle ipn:" << srcNode << " ts=[" << creationMs << "," << creationSeq << "]";
}

} // namespace ntnsagin
} // namespace ns3
