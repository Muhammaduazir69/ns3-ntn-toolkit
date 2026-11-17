/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#include "sagin-bundle-protocol.h"

#include "ns3/abort.h"
#include "ns3/log.h"

#include <vector>

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

namespace
{
// ---- Wave 7B: block encoder into a byte vector, so each block's CRC can be
// computed over its own bytes (RFC 9171 4.2.1) before it goes on the wire.
void
PutHead(std::vector<uint8_t>& v, uint8_t major, uint64_t x)
{
    const uint8_t mt = static_cast<uint8_t>(major << 5);
    if (x < 24)
    {
        v.push_back(mt | static_cast<uint8_t>(x));
        return;
    }
    int n;
    if (x <= 0xFF)
    {
        v.push_back(mt | 24);
        n = 1;
    }
    else if (x <= 0xFFFF)
    {
        v.push_back(mt | 25);
        n = 2;
    }
    else if (x <= 0xFFFFFFFFULL)
    {
        v.push_back(mt | 26);
        n = 4;
    }
    else
    {
        v.push_back(mt | 27);
        n = 8;
    }
    for (int i = n - 1; i >= 0; --i)
    {
        v.push_back(static_cast<uint8_t>(x >> (8 * i)));
    }
}

void
PutEid(std::vector<uint8_t>& v, uint64_t node, uint64_t service)
{
    PutHead(v, 4, 2);
    PutHead(v, 0, 2);
    PutHead(v, 4, 2);
    PutHead(v, 0, node);
    PutHead(v, 0, service);
}

uint32_t
CrcLen(uint8_t crcType)
{
    return crcType == kBpCrc16 ? 2 : (crcType == kBpCrc32c ? 4 : 0);
}

/// Append the CRC field (zero-filled), compute the CRC over [blockStart, end)
/// and fill it in, big-endian (RFC 9171 4.2.1).
void
SealBlock(std::vector<uint8_t>& v, size_t blockStart, uint8_t crcType)
{
    const uint32_t n = CrcLen(crcType);
    if (n == 0)
    {
        return;
    }
    PutHead(v, 2, n);
    const size_t at = v.size();
    v.insert(v.end(), n, 0);
    const uint32_t crc = crcType == kBpCrc16 ? Crc16X25(v.data() + blockStart, v.size() - blockStart)
                                             : Crc32c(v.data() + blockStart, v.size() - blockStart);
    for (uint32_t i = 0; i < n; ++i)
    {
        v[at + i] = static_cast<uint8_t>(crc >> (8 * (n - 1 - i)));
    }
}

/// Bounds-checked CBOR reader; never aborts.
struct SafeReader
{
    const uint8_t* d;
    size_t n;
    size_t pos{0};
    bool ok{true};

    bool Head(uint8_t major, uint64_t& out)
    {
        if (!ok || pos >= n)
        {
            ok = false;
            return false;
        }
        const uint8_t b = d[pos++];
        if ((b >> 5) != major)
        {
            ok = false;
            return false;
        }
        const uint8_t ai = b & 0x1F;
        if (ai < 24)
        {
            out = ai;
            return true;
        }
        int len = ai == 24 ? 1 : ai == 25 ? 2 : ai == 26 ? 4 : ai == 27 ? 8 : -1;
        if (len < 0 || pos + len > n)
        {
            ok = false;
            return false;
        }
        out = 0;
        for (int i = 0; i < len; ++i)
        {
            out = (out << 8) | d[pos++];
        }
        return true;
    }

    bool Uint(uint64_t& out) { return Head(0, out); }

    bool Eid()
    {
        uint64_t a, scheme, x;
        if (!Head(4, a) || a != 2 || !Uint(scheme))
        {
            return ok = false;
        }
        if (scheme == 2) // ipn: [node, service]
        {
            return Head(4, a) && a == 2 && Uint(x) && Uint(x);
        }
        if (scheme == 1) // dtn: uint 0 (dtn:none) or a text string
        {
            if (pos < n && (d[pos] >> 5) == 0)
            {
                return Uint(x) && x == 0;
            }
            if (!Head(3, x) || pos + x > n)
            {
                return ok = false;
            }
            pos += x;
            return true;
        }
        return ok = false;
    }

    /// Skip a CRC field of \p crcType; records where its value bytes start.
    bool Crc(uint8_t crcType, size_t& valueAt)
    {
        uint64_t len;
        if (!Head(2, len) || len != CrcLen(crcType) || pos + len > n)
        {
            return ok = false;
        }
        valueAt = pos;
        pos += len;
        return true;
    }
};

struct ParsedBlock
{
    uint64_t type{0};
    size_t start{0};   //!< first byte of the block's array head
    size_t end{0};     //!< one past the block's last byte
    uint8_t crcType{0};
    size_t crcAt{0};   //!< first byte of the CRC value
};

/// Parse a bundle; \p stopAtPayloadData stops right after the payload bytes
/// (the trailer has not been written yet).
bool
ParseBundle(const uint8_t* d, size_t n, std::vector<ParsedBlock>& blocks, bool stopAtPayloadData)
{
    SafeReader r{d, n};
    blocks.clear();
    if (n < 2 || d[0] != 0x9F)
    {
        return false;
    }
    r.pos = 1;
    // Primary block (RFC 9171 4.3.1).
    ParsedBlock pb;
    pb.start = r.pos;
    uint64_t items, version, flags, crcType, x;
    if (!r.Head(4, items) || items < 8 || items > 11 || !r.Uint(version) || version != 7 ||
        !r.Uint(flags) || !r.Uint(crcType) || crcType > 2)
    {
        return false;
    }
    const bool frag = (flags & 0x1) != 0;
    const uint64_t expect = 8 + (frag ? 2 : 0) + (crcType ? 1 : 0);
    if (items != expect || !r.Eid() || !r.Eid() || !r.Eid())
    {
        return false;
    }
    if (!r.Head(4, x) || x != 2 || !r.Uint(x) || !r.Uint(x) || !r.Uint(x))
    {
        return false;
    }
    if (frag && (!r.Uint(x) || !r.Uint(x)))
    {
        return false;
    }
    pb.crcType = static_cast<uint8_t>(crcType);
    if (crcType && !r.Crc(pb.crcType, pb.crcAt))
    {
        return false;
    }
    pb.end = r.pos;
    blocks.push_back(pb);
    // Canonical blocks (4.3.2) until the payload block, which is last (4.1).
    while (r.ok)
    {
        ParsedBlock cb;
        cb.start = r.pos;
        uint64_t num, bflags, ctype, len;
        if (!r.Head(4, items) || (items != 5 && items != 6) || !r.Uint(cb.type) || !r.Uint(num) ||
            !r.Uint(bflags) || !r.Uint(ctype) || ctype > 2 || items != 5 + (ctype ? 1 : 0))
        {
            return false;
        }
        cb.crcType = static_cast<uint8_t>(ctype);
        if (!r.Head(2, len) || r.pos + len > n)
        {
            return false;
        }
        r.pos += len;
        if (cb.type == 1 && stopAtPayloadData)
        {
            cb.end = r.pos;
            blocks.push_back(cb);
            return true;
        }
        if (ctype && !r.Crc(cb.crcType, cb.crcAt))
        {
            return false;
        }
        cb.end = r.pos;
        blocks.push_back(cb);
        if (cb.type == 1)
        {
            // The CBOR break closes the bundle and nothing follows it.
            return r.pos + 1 == n && d[r.pos] == 0xFF;
        }
    }
    return false;
}

bool
BlockCrcOk(const std::vector<uint8_t>& b, const ParsedBlock& pb)
{
    const uint32_t n = CrcLen(pb.crcType);
    std::vector<uint8_t> tmp(b.begin() + pb.start, b.begin() + pb.end);
    uint32_t got = 0;
    for (uint32_t i = 0; i < n; ++i)
    {
        got = (got << 8) | tmp[pb.crcAt - pb.start + i];
        tmp[pb.crcAt - pb.start + i] = 0;
    }
    const uint32_t want =
        pb.crcType == kBpCrc16 ? Crc16X25(tmp.data(), tmp.size()) : Crc32c(tmp.data(), tmp.size());
    return got == want;
}
} // namespace

uint16_t
Crc16X25(const uint8_t* data, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i)
    {
        crc ^= data[i];
        for (int k = 0; k < 8; ++k)
        {
            crc = (crc & 1) ? static_cast<uint16_t>((crc >> 1) ^ 0x8408) : static_cast<uint16_t>(crc >> 1);
        }
    }
    return static_cast<uint16_t>(crc ^ 0xFFFF);
}

uint32_t
Crc32c(const uint8_t* data, size_t len)
{
    uint32_t crc = 0xFFFFFFFFu;
    for (size_t i = 0; i < len; ++i)
    {
        crc ^= data[i];
        for (int k = 0; k < 8; ++k)
        {
            crc = (crc & 1) ? (crc >> 1) ^ 0x82F63B78u : crc >> 1;
        }
    }
    return crc ^ 0xFFFFFFFFu;
}

const char*
BundleCheckName(BundleCheck c)
{
    switch (c)
    {
    case BundleCheck::Ok:
        return "ok";
    case BundleCheck::Malformed:
        return "malformed";
    case BundleCheck::PrimaryCrc:
        return "primary-crc";
    case BundleCheck::CanonicalCrc:
        return "canonical-crc";
    }
    return "?";
}

BundleCheckResult
VerifyBundle(Ptr<const Packet> p)
{
    BundleCheckResult res;
    std::vector<uint8_t> b(p->GetSize());
    if (!b.empty())
    {
        p->CopyData(b.data(), b.size());
    }
    std::vector<ParsedBlock> blocks;
    if (!ParseBundle(b.data(), b.size(), blocks, false))
    {
        res.status = BundleCheck::Malformed;
        return res;
    }
    res.primaryUnprotected = blocks.front().crcType == kBpCrcNone;
    for (size_t i = 0; i < blocks.size(); ++i)
    {
        if (blocks[i].crcType == kBpCrcNone)
        {
            continue;
        }
        ++res.blocksChecked;
        if (!BlockCrcOk(b, blocks[i]))
        {
            res.status = i == 0 ? BundleCheck::PrimaryCrc : BundleCheck::CanonicalCrc;
            res.failedBlockType = i == 0 ? 0 : blocks[i].type;
            return res;
        }
    }
    res.status = BundleCheck::Ok;
    return res;
}

namespace
{
/// The header bytes: 0x9F, primary block, extension blocks, payload block
/// head up to and including the payload bstr head (RFC 9171 4.1, 4.3).
std::vector<uint8_t>
EncodeHead(const SaginBundleHeader& h)
{
    std::vector<uint8_t> v;
    v.reserve(96);
    v.push_back(0x9F);
    size_t s = v.size();
    PutHead(v, 4, h.crcType ? 9 : 8);
    PutHead(v, 0, 7);
    PutHead(v, 0, h.flags);
    PutHead(v, 0, h.crcType);
    PutEid(v, h.destNode, h.destService);
    PutEid(v, h.srcNode, h.srcService);
    PutEid(v, h.reportNode, h.reportService);
    PutHead(v, 4, 2);
    PutHead(v, 0, h.creationMs);
    PutHead(v, 0, h.creationSeq);
    PutHead(v, 0, h.lifetimeMs);
    SealBlock(v, s, h.crcType);
    const uint64_t blockItems = h.crcType ? 6 : 5;
    if (h.hasAgeBlock)
    {
        // Bundle Age block (4.4.2): type 7, block number 2, data = bstr(uint age).
        s = v.size();
        PutHead(v, 4, blockItems);
        PutHead(v, 0, 7);
        PutHead(v, 0, 2);
        PutHead(v, 0, 0);
        PutHead(v, 0, h.crcType);
        PutHead(v, 2, cbor::HeadSize(h.ageMs));
        PutHead(v, 0, h.ageMs);
        SealBlock(v, s, h.crcType);
    }
    if (h.hasCosBlock)
    {
        // Class of service (private type 193, RFC 6258 semantics), block number 3,
        // data = bstr([priority, ordinal, flags]).
        std::vector<uint8_t> data;
        PutHead(data, 4, 3);
        PutHead(data, 0, h.priority);
        PutHead(data, 0, h.ordinal);
        PutHead(data, 0, h.critical ? kBpCosFlagCritical : 0);
        s = v.size();
        PutHead(v, 4, blockItems);
        PutHead(v, 0, kBpBlockTypeCos);
        PutHead(v, 0, 3);
        PutHead(v, 0, 0);
        PutHead(v, 0, h.crcType);
        PutHead(v, 2, data.size());
        v.insert(v.end(), data.begin(), data.end());
        SealBlock(v, s, h.crcType);
    }
    // Payload block (4.3.2): [1, 1, 0, crcType, bstr(payload), crc?]; the
    // payload bytes and the CRC field follow in the packet (SaginBundleTrailer).
    PutHead(v, 4, blockItems);
    PutHead(v, 0, 1);
    PutHead(v, 0, 1);
    PutHead(v, 0, 0);
    PutHead(v, 0, h.crcType);
    PutHead(v, 2, h.payloadLength);
    return v;
}
} // namespace

SaginBundleTrailer
SaginBundleHeader::MakeTrailer() const
{
    SaginBundleTrailer t;
    t.crcType = crcType;
    return t;
}

uint32_t
SaginBundleHeader::GetSerializedSize() const
{
    return static_cast<uint32_t>(EncodeHead(*this).size());
}

void
SaginBundleHeader::Serialize(Buffer::Iterator i) const
{
    const auto v = EncodeHead(*this);
    i.Write(v.data(), v.size());
}

uint32_t
SaginBundleHeader::Deserialize(Buffer::Iterator start)
{
    // Callers on a receive path run VerifyBundle() first: this parser aborts
    // on malformed input, which a corrupted bundle would be.
    using namespace cbor;
    Buffer::Iterator i = start;
    NS_ABORT_MSG_IF(i.ReadU8() != 0x9F, "bundle: not an indefinite-length array");
    const uint64_t items = ReadHead(i, kArray);
    NS_ABORT_MSG_IF(items != 8 && items != 9, "primary block: expected 8 or 9 items");
    NS_ABORT_MSG_IF(ReadHead(i, kUint) != 7, "primary block: version is not 7");
    flags = ReadHead(i, kUint);
    crcType = static_cast<uint8_t>(ReadHead(i, kUint));
    NS_ABORT_MSG_IF(crcType > 2 || (items == 9) != (crcType != 0), "primary block: bad CRC type");
    ReadEid(i, destNode, destService);
    ReadEid(i, srcNode, srcService);
    ReadEid(i, reportNode, reportService);
    NS_ABORT_MSG_IF(ReadHead(i, kArray) != 2, "creation timestamp: not a 2-array");
    creationMs = ReadHead(i, kUint);
    creationSeq = ReadHead(i, kUint);
    lifetimeMs = ReadHead(i, kUint);
    auto skipCrc = [&i](uint64_t ct) {
        if (ct != 0)
        {
            const uint64_t n = ReadHead(i, kBytes);
            i.Next(static_cast<uint32_t>(n));
        }
    };
    skipCrc(crcType);
    hasAgeBlock = false;
    ageMs = 0;
    hasCosBlock = false;
    priority = kBpPriorityNormal;
    ordinal = 0;
    critical = false;
    while (true)
    {
        const uint64_t bi = ReadHead(i, kArray);
        NS_ABORT_MSG_IF(bi != 5 && bi != 6, "canonical block: expected 5 or 6 items");
        const uint64_t type = ReadHead(i, kUint);
        ReadHead(i, kUint); // block number
        ReadHead(i, kUint); // flags
        const uint64_t ct = ReadHead(i, kUint);
        if (type == 1)
        {
            payloadLength = ReadHead(i, kBytes);
            break; // the payload bytes and its CRC follow (trailer)
        }
        const uint64_t len = ReadHead(i, kBytes);
        if (type == 7)
        {
            ageMs = ReadHead(i, kUint);
            hasAgeBlock = true;
        }
        else if (type == kBpBlockTypeCos)
        {
            NS_ABORT_MSG_IF(ReadHead(i, kArray) != 3, "class-of-service block: expected 3 items");
            priority = static_cast<uint8_t>(ReadHead(i, kUint));
            ordinal = static_cast<uint8_t>(ReadHead(i, kUint));
            critical = (ReadHead(i, kUint) & kBpCosFlagCritical) != 0;
            hasCosBlock = true;
        }
        else
        {
            i.Next(static_cast<uint32_t>(len)); // unknown extension block
        }
        skipCrc(ct);
    }
    return i.GetDistanceFrom(start);
}

void
SaginBundleHeader::Print(std::ostream& os) const
{
    os << "bpv7 ipn:" << srcNode << "." << srcService << " -> ipn:" << destNode << "."
       << destService << " ts=[" << creationMs << "," << creationSeq << "] life=" << lifetimeMs
       << "ms flags=0x" << std::hex << flags << std::dec << " crc=" << +crcType;
    if (hasAgeBlock)
    {
        os << " age=" << ageMs << "ms";
    }
    if (hasCosBlock)
    {
        os << " prio=" << +priority << (critical ? " critical" : "");
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

uint32_t
SaginBundleTrailer::GetSerializedSize() const
{
    const uint32_t n = CrcLen(crcType);
    return 1 + (n ? 1 + n : 0);
}

void
SaginBundleTrailer::Serialize(Buffer::Iterator end) const
{
    const uint32_t size = GetSerializedSize();
    // Read the bundle from the start of the buffer up to this trailer.
    Buffer::Iterator b = end;
    const uint32_t total = b.GetSize() - b.GetRemainingSize();
    b.Prev(total);
    std::vector<uint8_t> v(total - size);
    if (!v.empty())
    {
        b.Read(v.data(), v.size());
    }
    if (size > 1)
    {
        std::vector<ParsedBlock> blocks;
        NS_ABORT_MSG_UNLESS(ParseBundle(v.data(), v.size(), blocks, true) &&
                                blocks.back().type == 1 && blocks.back().end == v.size(),
                            "bundle trailer: the bundle must start the packet buffer and end "
                            "with the payload bytes");
        NS_ABORT_MSG_UNLESS(blocks.back().crcType == crcType,
                            "bundle trailer: CRC type differs from the payload block's "
                            "(use SaginBundleHeader::MakeTrailer())");
        const size_t s = blocks.back().start;
        PutHead(v, 2, CrcLen(crcType));
        const size_t at = v.size();
        v.insert(v.end(), CrcLen(crcType), 0);
        const uint32_t crc = crcType == kBpCrc16 ? Crc16X25(v.data() + s, v.size() - s)
                                                 : Crc32c(v.data() + s, v.size() - s);
        for (uint32_t k = 0; k < CrcLen(crcType); ++k)
        {
            v[at + k] = static_cast<uint8_t>(crc >> (8 * (CrcLen(crcType) - 1 - k)));
        }
        Buffer::Iterator w = end;
        w.Prev(size);
        w.Write(v.data() + total - size, size - 1);
    }
    Buffer::Iterator w = end;
    w.Prev(1);
    w.WriteU8(0xFF);
}

uint32_t
SaginBundleTrailer::Deserialize(Buffer::Iterator end)
{
    const uint32_t size = GetSerializedSize();
    Buffer::Iterator i = end;
    i.Prev(size);
    crcValue = 0;
    if (size > 1)
    {
        i.ReadU8(); // bstr head
        for (uint32_t k = 0; k < CrcLen(crcType); ++k)
        {
            crcValue = (crcValue << 8) | i.ReadU8();
        }
    }
    NS_ABORT_MSG_IF(i.ReadU8() != 0xFF, "bundle: missing CBOR break");
    return size;
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
