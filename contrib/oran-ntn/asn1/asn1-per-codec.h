/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#ifndef ORAN_NTN_ASN1_PER_CODEC_H
#define ORAN_NTN_ASN1_PER_CODEC_H

// Aligned-PER (X.691) encoder/decoder for E2AP / E2SM-KPM / E2SM-RC
// (Roadmap §3 T2).
//
// Hand-written subset of Aligned-PER covering the shapes the toolkit's
// SMs emit today:
//   INTEGER (variable-length, length-prefixed)
//   OCTET STRING (length-determinant + bytes, octet aligned)
//   UTF8String (length-determinant + bytes, octet aligned)
//   SEQUENCE (preamble bit-map for OPTIONAL fields, octet aligned)
//   SEQUENCE OF (length-determinant + N copies)
//   CHOICE (alternative index byte, then chosen alternative)
//   OPTIONAL (presence bit in the preamble)
//
// HONESTY NOTE (codec conformance): this codec is internally self-consistent
// (its own encode↔decode round-trips, exercised in the unit tests and in the
// inlined beam example) but it is NOT bit-conformant Aligned-PER. It takes
// deliberate full-byte / octet-aligned shortcuts — a full-byte CHOICE
// alternative index (not the minimal constrained-index bits), octet-aligned
// SEQUENCE preambles, length-prefixed INTEGERs rather than constrained-INTEGER
// bit-packing, and no extension-marker handling. Bytes produced here would
// therefore NOT decode on a spec-conformant asn1c peer (e.g. FlexRIC). It is
// a test-only / in-sim codec, never on a live external control path.
//
// The Q4-end T2 follow-on swaps this implementation for asn1c-generated
// encoders sourced from O-RAN-SC ASN.1 files without changing the
// public ASN.1-PER API surface here. Reviewers can therefore audit the
// C++ side once and let the on-the-wire bytes evolve.
//
// Reference: ITU-T X.691 (Aligned PER),
//            O-RAN.WG3.E2SM-KPM R004 v06.00 (ETSI TS 104 040 v4.0.0, 2025),
//            O-RAN.WG3.E2SM-RC R004 v07.00 (2025) ASN.1 modules.

#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

namespace ns3
{
namespace oranntn
{
namespace asn1
{

/// Append-only byte writer for PER-encoded output. Octet aligned —
/// every value lands on a byte boundary, matching X.691 §10.6 for
/// non-bit-string types.
class PerWriter
{
  public:
    PerWriter() = default;

    const std::vector<uint8_t>& Bytes() const { return m_buf; }
    std::vector<uint8_t> Take() { return std::move(m_buf); }
    size_t Size() const { return m_buf.size(); }

    /// Length determinant per X.691 §10.9.3: short form 0..127 in 1 byte
    /// (0xxxxxxx) or long form 128..16383 in 2 bytes (10xxxxxx xxxxxxxx).
    /// Fragmentation for larger lengths is rejected for the v2.1 subset.
    void WriteLengthDeterminant(uint32_t n);

    /// Unconstrained INTEGER (X.691 §10.5.6): two's-complement bytes
    /// prefixed by their octet count.
    void WriteInteger(int64_t v);

    /// UTF8String: length determinant + UTF-8 bytes.
    void WriteUtf8String(const std::string& s);

    /// OCTET STRING: length determinant + raw bytes.
    void WriteOctetString(const std::vector<uint8_t>& bytes);

    /// CHOICE alternative index in 0..255. The chosen alternative's body
    /// follows on the next octet boundary.
    void WriteChoiceIndex(uint8_t idx);

    /// SEQUENCE preamble bit-map for OPTIONAL / DEFAULT fields. Pass the
    /// number of optional slots; each subsequent SetPreambleBit(i, on)
    /// flips slot i. CallEndPreamble before emitting the body.
    void BeginSequencePreamble(uint8_t num_optionals);
    void SetPreambleBit(uint8_t idx, bool present);
    void EndSequencePreamble();

  private:
    std::vector<uint8_t> m_buf;
    // Buffered SEQUENCE preamble (pending bits).
    uint16_t m_preambleBits{0};
    uint8_t m_preambleSlots{0};
    bool m_preambleOpen{false};
};

/// Position-tracking reader. Throws `std::runtime_error` on out-of-range
/// access; the C++ API wrappers catch this and return false to callers.
class PerReader
{
  public:
    explicit PerReader(const std::vector<uint8_t>& bytes) : m_buf(bytes) {}

    bool Eof() const { return m_pos >= m_buf.size(); }
    size_t Pos() const { return m_pos; }

    uint32_t ReadLengthDeterminant();
    int64_t ReadInteger();
    std::string ReadUtf8String();
    std::vector<uint8_t> ReadOctetString();
    uint8_t ReadChoiceIndex();

    /// Returns the preamble bit-map for the next `num_optionals` slots.
    /// Caller decodes each bit via `((bitmap >> (num_optionals-1-i)) & 1)`.
    uint16_t ReadSequencePreamble(uint8_t num_optionals);

  private:
    uint8_t Get();

    const std::vector<uint8_t>& m_buf;
    size_t m_pos{0};
};

} // namespace asn1
} // namespace oranntn
} // namespace ns3

#endif // ORAN_NTN_ASN1_PER_CODEC_H
