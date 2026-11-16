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
//  - the primary block (RFC 9171 4.3.1) as an 8-item CBOR array: version 7,
//    bundle processing control flags, CRC type 0 (none), destination EID,
//    source node ID, report-to EID (all "ipn" scheme, [2, [node, service]],
//    RFC 9171 4.2.5.1.2), creation timestamp [DTN time ms, sequence number]
//    (4.2.7, DTN time = ms since 2000-01-01 00:00:00 UTC, 4.2.6), lifetime in
//    ms (4.2.2);
//  - the payload block header (4.3.2, block type 1, block number 1, flags 0,
//    CRC type 0, block-type-specific data as a CBOR byte string whose content
//    is the ns-3 payload that follows the header);
//  - the custody signal administrative record of RFC 5050 6.1 / 6.3 (record
//    type 2: succeeded flag, reason code, subject bundle source EID and
//    creation timestamp), CBOR encoded. BPv7 removed custody transfer from the
//    base protocol; this keeps the RFC 5050 semantics as an extension.
//
// What is NOT implemented: CRC-16/CRC-32C block CRCs (CRC type 0 is legal),
// extension blocks (previous node, bundle age, hop count), fragmentation
// (4.2.3 flag never set), BPSec (RFC 9172), administrative records sent as
// bundles (the custody signal is a bare record over UDP), bundle status
// reports on the wire (they are counted, see SaginCustodyQueue), a
// convergence-layer protocol (LTP RFC 5326, TCPCL RFC 9174: UDP datagrams
// carry one bundle each), and CCSDS 734.3-B-1 SABR contact-graph routing.

#ifndef SAGIN_BUNDLE_PROTOCOL_H
#define SAGIN_BUNDLE_PROTOCOL_H

#include "ns3/header.h"
#include "ns3/trailer.h"

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

/// Unix time (s) of the DTN epoch 2000-01-01 00:00:00 UTC (RFC 9171 4.2.6).
constexpr double kDtnEpochUnixS = 946684800.0;

/// CBOR (RFC 8949) helpers for unsigned integers and heads.
namespace cbor
{
/// Bytes needed for a head of major type \p major with argument \p v.
uint32_t HeadSize(uint64_t v);
}

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

    /// Bundle identifier (source node, creation timestamp) folded to 64 bits.
    uint64_t BundleId() const;
    static uint64_t BundleId(uint64_t srcNode, uint64_t creationMs, uint64_t seq);
};

/// The 0xFF break that closes the bundle's indefinite-length array.
class SaginBundleTrailer : public Trailer
{
  public:
    static TypeId GetTypeId();
    TypeId GetInstanceTypeId() const override;
    uint32_t GetSerializedSize() const override { return 1; }
    void Serialize(Buffer::Iterator start) const override;
    uint32_t Deserialize(Buffer::Iterator start) override;
    void Print(std::ostream& os) const override { os << "break"; }
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
