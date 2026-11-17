/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only
//
// sagin-bundle-protocol - the wire format of a BPv7 bundle (RFC 9171) and of a
// custody signal (RFC 5050 6.3 semantics), CBOR encoded (RFC 8949), carried as
// ns-3 headers so a bundle on a link has its real size and fields.
//
// What IS implemented:
//  - the bundle as a CBOR indefinite-length array (RFC 9171 4.1): 0x9F,
//    primary block, payload block, 0xFF break (SaginBundleTrailer);
//  - the primary block (RFC 9171 4.3.1) as a CBOR array of 8 items (CRC
//    type 0) or 9 items (CRC type 1 or 2, the default): version 7,
//    bundle processing control flags, CRC type, destination EID,
//    source node ID, report-to EID (all "ipn" scheme, [2, [node, service]],
//    RFC 9171 4.2.5.1.2), creation timestamp [DTN time ms, sequence number]
//    (4.2.7, DTN time = ms since 2000-01-01 00:00:00 UTC, 4.2.6), lifetime in
//    ms (4.2.2), CRC;
//  - the payload block header (4.3.2, block type 1, block number 1, flags 0,
//    CRC type, block-type-specific data as a CBOR byte string whose content
//    is the ns-3 payload that follows the header);
//  - the custody signal administrative record of RFC 5050 6.1 / 6.3 (record
//    type 2: succeeded flag, reason code, subject bundle source EID and
//    creation timestamp), CBOR encoded. BPv7 removed custody transfer from the
//    base protocol; this keeps the RFC 5050 semantics as an extension.
//
//  - the Bundle Age extension block (RFC 9171 4.4.2, block type 7), Wave 6B,
//    carried when the creation time is 0 and updated at every forwarding by
//    SaginCustodyQueue (the dwell time at the node is added).
//
//  - Wave 7B, block CRCs (RFC 9171 4.2.1, 4.3.1, 4.3.2): CRC type 1 (CRC-16
//    X.25) or 2 (CRC-32C, Castagnoli) on the primary block and on every
//    canonical block, computed over the CBOR bytes of the whole block with
//    the CRC field present and zero-filled, carried big-endian in a CBOR
//    byte string. The payload block's CRC follows the payload bytes, so it
//    lives in SaginBundleTrailer. VerifyBundle() checks every CRC on receive
//    with a bounds-checked parser; a node that fails it discards the bundle
//    (RFC 9171 4.2.1: the CRC lets the receiver detect corruption; a block
//    that fails it is unintelligible, status report reason 8).
//  - Wave 7B, class of service: an optional extension block of private-use
//    type 193 (RFC 9171 9.1 reserves 192-255 for private and experimental
//    use) carrying [priority, ordinal, flags] with the semantics of the
//    BPv6 Extended Class of Service block (RFC 6258): priority bulk 0,
//    normal 1, expedited 2; flag 0x01 = critical. BPv7 has no standard
//    priority field; CCSDS 734.3-B-1 SABR needs one (3.2.6.8, 3.2.8.1.3).
//
// What is NOT implemented: the previous node and hop count extension blocks,
// fragmentation
// (4.2.3 flag never set), BPSec (RFC 9172), administrative records sent as
// bundles (the custody signal is a bare record over UDP), bundle status
// reports on the wire (they are counted, see SaginCustodyQueue), a
// convergence-layer protocol (LTP RFC 5326, TCPCL RFC 9174: UDP datagrams
// carry one bundle each). CCSDS 734.3-B-1 SABR contact graph routing is in
// sagin-sabr-router.h (Wave 6B).

#ifndef SAGIN_BUNDLE_PROTOCOL_H
#define SAGIN_BUNDLE_PROTOCOL_H

#include "ns3/header.h"
#include "ns3/packet.h"
#include "ns3/trailer.h"

#include <cstddef>
#include <cstdint>

namespace ns3
{
namespace ntnsagin
{

/// RFC 9171 4.2.3 bundle processing control flags used here.
constexpr uint64_t kBpFlagCustodyRequested = 0x0008;   //!< RFC 5050 5.10 (bit 3)
constexpr uint64_t kBpFlagReportDeletion = 0x40000;    //!< RFC 9171 4.2.3 bit 18
constexpr uint64_t kBpFlagReportDelivery = 0x20000;    //!< RFC 9171 4.2.3 bit 17

/// RFC 9171 6.1.1 status report reason code: lifetime expired.
constexpr uint8_t kBpReasonLifetimeExpired = 1;
/// RFC 5050 6.3 custody signal reason codes.
constexpr uint8_t kCustodyReasonNone = 0;
constexpr uint8_t kCustodyReasonRedundant = 3;
constexpr uint8_t kCustodyReasonDepletedStorage = 4;

/// RFC 9171 4.2.1 CRC types.
constexpr uint8_t kBpCrcNone = 0;
constexpr uint8_t kBpCrc16 = 1;  //!< CRC-16 X.25
constexpr uint8_t kBpCrc32c = 2; //!< CRC-32C (Castagnoli)
/// Private-use block type (RFC 9171 9.1) for the class-of-service block.
constexpr uint64_t kBpBlockTypeCos = 193;
/// Class-of-service priorities (RFC 6258 / RFC 5050 4.2 semantics).
constexpr uint8_t kBpPriorityBulk = 0;
constexpr uint8_t kBpPriorityNormal = 1;
constexpr uint8_t kBpPriorityExpedited = 2;
constexpr uint8_t kBpCosFlagCritical = 0x01;

/// CRC-16 X.25 (RFC 9171 4.2.1; poly 0x1021 reflected, init and xorout 0xFFFF).
uint16_t Crc16X25(const uint8_t* data, size_t len);
/// CRC-32C (RFC 9171 4.2.1; Castagnoli poly 0x1EDC6F41 reflected, init and xorout 0xFFFFFFFF).
uint32_t Crc32c(const uint8_t* data, size_t len);

/// Outcome of VerifyBundle().
enum class BundleCheck : uint8_t
{
    Ok,           //!< well formed, every CRC present matches
    Malformed,    //!< not a parseable BPv7 bundle (truncated, bad CBOR, bad version)
    PrimaryCrc,   //!< the primary block CRC does not match
    CanonicalCrc, //!< a canonical (extension or payload) block CRC does not match
};
struct BundleCheckResult
{
    BundleCheck status{BundleCheck::Malformed};
    uint64_t failedBlockType{0}; //!< block type of the failed canonical block
    bool primaryUnprotected{false}; //!< CRC type 0 on the primary block (4.3.1 deviation)
    uint32_t blocksChecked{0};   //!< blocks whose CRC was verified
};
/// Bounds-checked parse of the whole bundle (the packet must start with the
/// bundle and end with its CBOR break) and verification of every block CRC.
/// It never aborts, so it is safe on bytes corrupted in transit; run it
/// before PeekHeader/RemoveHeader on anything that came off a link.
BundleCheckResult VerifyBundle(Ptr<const Packet> p);
const char* BundleCheckName(BundleCheck c);

/// Unix time (s) of the DTN epoch 2000-01-01 00:00:00 UTC (RFC 9171 4.2.6).
constexpr double kDtnEpochUnixS = 946684800.0;

/// CBOR (RFC 8949) helpers for unsigned integers and heads.
namespace cbor
{
/// Bytes needed for a head of major type \p major with argument \p v.
uint32_t HeadSize(uint64_t v);
}

class SaginBundleTrailer;

/**
 * \ingroup ntn-sagin
 * \brief BPv7 bundle head: 0x9F + primary block + payload block header.
 */
class SaginBundleHeader : public Header
{
  public:
    static TypeId GetTypeId();
    TypeId GetInstanceTypeId() const override;
    uint32_t GetSerializedSize() const override;
    void Serialize(Buffer::Iterator start) const override;
    uint32_t Deserialize(Buffer::Iterator start) override;
    void Print(std::ostream& os) const override;

    uint64_t flags{kBpFlagCustodyRequested | kBpFlagReportDeletion};
    uint64_t destNode{0};
    uint64_t destService{0};
    uint64_t srcNode{0};
    uint64_t srcService{0};
    uint64_t reportNode{0};
    uint64_t reportService{0};
    uint64_t creationMs{0};  //!< DTN time (ms since 2000-01-01)
    uint64_t creationSeq{0}; //!< sequence number within the ms
    uint64_t lifetimeMs{0};
    uint64_t payloadLength{0}; //!< bytes of the ns-3 payload after this header
    /// Bundle Age extension block (RFC 9171 4.4.2, block type 7, block
    /// number 2), placed before the payload block. RFC 9171 4.2.7: a bundle
    /// whose creation time is 0 (no accurate clock at the source) MUST carry
    /// it, and its age is then the only basis for expiry (5.5).
    bool hasAgeBlock{false};
    uint64_t ageMs{0}; //!< elapsed ms since creation, as of the latest forwarding
    /// Wave 7B: CRC type of every block of this bundle (RFC 9171 4.2.1). The
    /// default is CRC-16 X.25: 4.3.1 requires a CRC on the primary block
    /// when no BIB protects it. kBpCrcNone reproduces the pre-7B wire format.
    uint8_t crcType{kBpCrc16};
    /// Wave 7B: class-of-service block (type 193), present when set.
    bool hasCosBlock{false};
    uint8_t priority{kBpPriorityNormal};
    uint8_t ordinal{0};
    bool critical{false};

    /// The trailer that matches this header's payload-block CRC type.
    SaginBundleTrailer MakeTrailer() const;

    /// Bundle identifier (source node, creation timestamp) folded to 64 bits.
    uint64_t BundleId() const;
    static uint64_t BundleId(uint64_t srcNode, uint64_t creationMs, uint64_t seq);
};

/// The payload block's CRC field (RFC 9171 4.3.2, it follows the payload
/// bytes) and the 0xFF break that closes the bundle's indefinite-length array.
/// Serialize() locates the payload block by parsing the bundle from the start
/// of the packet buffer, so the bundle header must be at the buffer start
/// when the trailer is added. Set crcType to the header's (MakeTrailer()).
class SaginBundleTrailer : public Trailer
{
  public:
    static TypeId GetTypeId();
    TypeId GetInstanceTypeId() const override;
    uint32_t GetSerializedSize() const override;
    void Serialize(Buffer::Iterator start) const override;
    uint32_t Deserialize(Buffer::Iterator start) override;
    void Print(std::ostream& os) const override { os << "crc" << +crcType << " break"; }

    uint8_t crcType{kBpCrc16};
    uint32_t crcValue{0}; //!< read back by Deserialize (not verified there)
};

/**
 * \ingroup ntn-sagin
 * \brief RFC 5050 6.3 custody signal, CBOR encoded:
 *        [2, [succeeded, reason, [2, [srcNode, srcService]], [creationMs, seq]]]
 */
class SaginCustodySignalHeader : public Header
{
  public:
    static TypeId GetTypeId();
    TypeId GetInstanceTypeId() const override;
    uint32_t GetSerializedSize() const override;
    void Serialize(Buffer::Iterator start) const override;
    uint32_t Deserialize(Buffer::Iterator start) override;
    void Print(std::ostream& os) const override;

    bool succeeded{true};
    uint8_t reason{kCustodyReasonNone};
    uint64_t srcNode{0};
    uint64_t srcService{0};
    uint64_t creationMs{0};
    uint64_t creationSeq{0};

    uint64_t BundleId() const
    {
        return SaginBundleHeader::BundleId(srcNode, creationMs, creationSeq);
    }
};

} // namespace ntnsagin
} // namespace ns3

#endif // SAGIN_BUNDLE_PROTOCOL_H
