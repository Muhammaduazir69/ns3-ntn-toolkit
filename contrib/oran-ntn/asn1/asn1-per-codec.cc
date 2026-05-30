/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#include "asn1-per-codec.h"

#include <stdexcept>

namespace ns3
{
namespace oranntn
{
namespace asn1
{

// ----------------------------------------------------------------------------
//  PerWriter
// ----------------------------------------------------------------------------

void
PerWriter::WriteLengthDeterminant(uint32_t n)
{
    if (n <= 127)
    {
        m_buf.push_back(static_cast<uint8_t>(n));
    }
    else if (n <= 16383)
    {
        m_buf.push_back(static_cast<uint8_t>(0x80 | (n >> 8)));
        m_buf.push_back(static_cast<uint8_t>(n & 0xFF));
    }
    else
    {
        throw std::runtime_error(
            "PerWriter::WriteLengthDeterminant: > 16383 not supported");
    }
}

void
PerWriter::WriteInteger(int64_t v)
{
    // Two's complement octet count (X.691 §10.5.6 unconstrained INTEGER).
    uint8_t bytes[9];
    int n = 0;
    // Always emit at least 1 byte. Trim leading 0x00 / 0xFF when the
    // sign bit of the next byte already matches.
    if (v >= 0)
    {
        if (v == 0)
        {
            bytes[0] = 0;
            n = 1;
        }
        else
        {
            uint64_t u = static_cast<uint64_t>(v);
            int idx = 0;
            uint8_t tmp[9];
            while (u > 0)
            {
                tmp[idx++] = static_cast<uint8_t>(u & 0xFF);
                u >>= 8;
            }
            // If the top bit of the highest byte is set, push a 0x00.
            if (tmp[idx - 1] & 0x80)
            {
                bytes[n++] = 0x00;
            }
            for (int i = idx - 1; i >= 0; --i)
            {
                bytes[n++] = tmp[i];
            }
        }
    }
    else
    {
        // Negative — two's complement, prefix 0xFF if needed.
        uint64_t u = static_cast<uint64_t>(v);
        int idx = 0;
        uint8_t tmp[9];
        while (idx < 8)
        {
            tmp[idx++] = static_cast<uint8_t>(u & 0xFF);
            u >>= 8;
            // Stop when redundant 0xFF sign-extension begins.
            if (idx >= 1 && u == 0xFFFFFFFFFFFFFFFFULL &&
                (tmp[idx - 1] & 0x80))
            {
                break;
            }
        }
        for (int i = idx - 1; i >= 0; --i)
        {
            bytes[n++] = tmp[i];
        }
    }
    WriteLengthDeterminant(static_cast<uint32_t>(n));
    for (int i = 0; i < n; ++i)
    {
        m_buf.push_back(bytes[i]);
    }
}

void
PerWriter::WriteUtf8String(const std::string& s)
{
    WriteLengthDeterminant(static_cast<uint32_t>(s.size()));
    for (char c : s)
    {
        m_buf.push_back(static_cast<uint8_t>(c));
    }
}

void
PerWriter::WriteOctetString(const std::vector<uint8_t>& bytes)
{
    WriteLengthDeterminant(static_cast<uint32_t>(bytes.size()));
    for (uint8_t b : bytes)
    {
        m_buf.push_back(b);
    }
}

void
PerWriter::WriteChoiceIndex(uint8_t idx)
{
    m_buf.push_back(idx);
}

void
PerWriter::BeginSequencePreamble(uint8_t num_optionals)
{
    if (m_preambleOpen)
    {
        throw std::runtime_error(
            "PerWriter::BeginSequencePreamble: previous preamble not closed");
    }
    if (num_optionals > 16)
    {
        throw std::runtime_error(
            "PerWriter::BeginSequencePreamble: >16 OPTIONALs unsupported");
    }
    m_preambleSlots = num_optionals;
    m_preambleBits = 0;
    m_preambleOpen = true;
}

void
PerWriter::SetPreambleBit(uint8_t idx, bool present)
{
    if (!m_preambleOpen)
    {
        throw std::runtime_error(
            "PerWriter::SetPreambleBit: no preamble open");
    }
    if (idx >= m_preambleSlots)
    {
        throw std::runtime_error(
            "PerWriter::SetPreambleBit: idx out of range");
    }
    const uint8_t shift =
        static_cast<uint8_t>(m_preambleSlots - 1 - idx);
    if (present)
    {
        m_preambleBits |= static_cast<uint16_t>(1u << shift);
    }
    else
    {
        m_preambleBits &= static_cast<uint16_t>(~(1u << shift));
    }
}

void
PerWriter::EndSequencePreamble()
{
    if (!m_preambleOpen)
    {
        return;
    }
    if (m_preambleSlots == 0)
    {
        // No optionals -> no preamble bytes.
    }
    else if (m_preambleSlots <= 8)
    {
        const uint8_t shift =
            static_cast<uint8_t>(8 - m_preambleSlots);
        m_buf.push_back(static_cast<uint8_t>(m_preambleBits << shift));
    }
    else
    {
        const uint8_t shift =
            static_cast<uint8_t>(16 - m_preambleSlots);
        const uint16_t aligned =
            static_cast<uint16_t>(m_preambleBits << shift);
        m_buf.push_back(static_cast<uint8_t>(aligned >> 8));
        m_buf.push_back(static_cast<uint8_t>(aligned & 0xFF));
    }
    m_preambleOpen = false;
}

// ----------------------------------------------------------------------------
//  PerReader
// ----------------------------------------------------------------------------

uint8_t
PerReader::Get()
{
    if (m_pos >= m_buf.size())
    {
        throw std::runtime_error("PerReader: EOF");
    }
    return m_buf[m_pos++];
}

uint32_t
PerReader::ReadLengthDeterminant()
{
    const uint8_t b0 = Get();
    if ((b0 & 0x80) == 0)
    {
        return b0;
    }
    const uint8_t b1 = Get();
    return static_cast<uint32_t>(((b0 & 0x3F) << 8) | b1);
}

int64_t
PerReader::ReadInteger()
{
    const uint32_t n = ReadLengthDeterminant();
    if (n == 0 || n > 9)
    {
        throw std::runtime_error("PerReader::ReadInteger: bad length");
    }
    const uint8_t first = Get();
    int64_t v = (first & 0x80) ? -1 : 0;
    v = (v & ~int64_t{0xFF}) | first;
    for (uint32_t i = 1; i < n; ++i)
    {
        v = (v << 8) | Get();
    }
    return v;
}

std::string
PerReader::ReadUtf8String()
{
    const uint32_t n = ReadLengthDeterminant();
    std::string s;
    s.reserve(n);
    for (uint32_t i = 0; i < n; ++i)
    {
        s.push_back(static_cast<char>(Get()));
    }
    return s;
}

std::vector<uint8_t>
PerReader::ReadOctetString()
{
    const uint32_t n = ReadLengthDeterminant();
    std::vector<uint8_t> out;
    out.reserve(n);
    for (uint32_t i = 0; i < n; ++i)
    {
        out.push_back(Get());
    }
    return out;
}

uint8_t
PerReader::ReadChoiceIndex()
{
    return Get();
}

uint16_t
PerReader::ReadSequencePreamble(uint8_t num_optionals)
{
    if (num_optionals == 0)
    {
        return 0;
    }
    if (num_optionals <= 8)
    {
        const uint8_t b = Get();
        const uint8_t shift = static_cast<uint8_t>(8 - num_optionals);
        return static_cast<uint16_t>(b >> shift);
    }
    if (num_optionals > 16)
    {
        throw std::runtime_error(
            "PerReader::ReadSequencePreamble: >16 OPTIONALs unsupported");
    }
    const uint8_t hi = Get();
    const uint8_t lo = Get();
    const uint16_t raw = static_cast<uint16_t>((hi << 8) | lo);
    const uint8_t shift = static_cast<uint8_t>(16 - num_optionals);
    return static_cast<uint16_t>(raw >> shift);
}

} // namespace asn1
} // namespace oranntn
} // namespace ns3
