/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#include "ntn-meas-report.h"

#include "ns3/log.h"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("NtnMeasReport");

namespace
{

/// Saturating conversion from a real-valued quantity to a TS 38.133 level.
/// Saturation, not wraparound: a UE seeing -180 dBm reports level 0, which is
/// the standard's "below the reporting range", and a wrap to 127 would turn
/// the worst possible signal into the best.
uint8_t
Quantize(double value, double minValue, double step)
{
    if (std::isnan(value))
    {
        return 0;
    }
    const double raw = (value - minValue) / step;
    if (raw <= 0.0)
    {
        return 0;
    }
    if (raw >= static_cast<double>(NtnMeasQuantity::kLevelMax))
    {
        return NtnMeasQuantity::kLevelMax;
    }
    return static_cast<uint8_t>(std::lround(raw));
}

void
PutU16(uint8_t* p, uint16_t v)
{
    p[0] = static_cast<uint8_t>(v & 0xFF);
    p[1] = static_cast<uint8_t>((v >> 8) & 0xFF);
}

uint16_t
GetU16(const uint8_t* p)
{
    return static_cast<uint16_t>(p[0]) | static_cast<uint16_t>(p[1] << 8);
}

void
PutI64(uint8_t* p, int64_t v)
{
    for (int i = 0; i < 8; ++i)
    {
        p[i] = static_cast<uint8_t>((v >> (8 * i)) & 0xFF);
    }
}

int64_t
GetI64(const uint8_t* p)
{
    uint64_t v = 0;
    for (int i = 0; i < 8; ++i)
    {
        v |= static_cast<uint64_t>(p[i]) << (8 * i);
    }
    return static_cast<int64_t>(v);
}

void
PutF64(uint8_t* p, double v)
{
    uint64_t bits;
    std::memcpy(&bits, &v, sizeof(bits));
    PutI64(p, static_cast<int64_t>(bits));
}

double
GetF64(const uint8_t* p)
{
    const uint64_t bits = static_cast<uint64_t>(GetI64(p));
    double v;
    std::memcpy(&v, &bits, sizeof(v));
    return v;
}

/// Per-cell record: pcid(2) + 3 levels(3) + presence bitmap(1).
constexpr std::size_t kCellBytes = 6;
/// measId(1) + neighbour count(1) + serving cell + TA(8) + slant(8).
constexpr std::size_t kFixedBytes = 1 + 1 + kCellBytes + 8 + 8;

void
PutCell(uint8_t* p, const NtnMeasResultNr& c)
{
    PutU16(p, c.physCellId);
    p[2] = c.rsrpLevel;
    p[3] = c.rsrqLevel;
    p[4] = c.sinrLevel;
    p[5] = static_cast<uint8_t>((c.haveRsrp ? 0x1 : 0) | (c.haveRsrq ? 0x2 : 0) |
                                (c.haveSinr ? 0x4 : 0));
}

void
GetCell(const uint8_t* p, NtnMeasResultNr& c)
{
    c.physCellId = GetU16(p);
    c.rsrpLevel = p[2];
    c.rsrqLevel = p[3];
    c.sinrLevel = p[4];
    c.haveRsrp = (p[5] & 0x1) != 0;
    c.haveRsrq = (p[5] & 0x2) != 0;
    c.haveSinr = (p[5] & 0x4) != 0;
}

} // namespace

uint8_t
NtnMeasQuantity::RsrpToLevel(double rsrpDbm)
{
    // TS 38.133 Table 10.1.6.1-1: 1 dB steps from -156 dBm.
    return Quantize(rsrpDbm, static_cast<double>(kRsrpMinDbm), 1.0);
}

double
NtnMeasQuantity::LevelToRsrpDbm(uint8_t level)
{
    return static_cast<double>(kRsrpMinDbm) +
           static_cast<double>(std::min<uint8_t>(level, kLevelMax));
}

uint8_t
NtnMeasQuantity::RsrqToLevel(double rsrqDb)
{
    // TS 38.133 Table 10.1.11.1-1: 0.5 dB steps from -43 dB.
    return Quantize(rsrqDb, kRsrqMinDb, 0.5);
}

double
NtnMeasQuantity::LevelToRsrqDb(uint8_t level)
{
    return kRsrqMinDb + 0.5 * static_cast<double>(std::min<uint8_t>(level, kLevelMax));
}

uint8_t
NtnMeasQuantity::SinrToLevel(double sinrDb)
{
    // TS 38.133 Table 10.1.16.1-1: 0.5 dB steps from -23 dB.
    return Quantize(sinrDb, kSinrMinDb, 0.5);
}

double
NtnMeasQuantity::LevelToSinrDb(uint8_t level)
{
    return kSinrMinDb + 0.5 * static_cast<double>(std::min<uint8_t>(level, kLevelMax));
}

std::size_t
NtnMeasReportCodec::SerialisedBytes(std::size_t neighbours)
{
    return kFixedBytes + neighbours * kCellBytes;
}

std::size_t
NtnMeasReportCodec::Serialise(const NtnMeasurementReport& rep, uint8_t* out, std::size_t len)
{
    if (out == nullptr || rep.neighbours.size() > NtnMeasurementReport::kMaxNeighbours)
    {
        return 0;
    }
    const std::size_t need = SerialisedBytes(rep.neighbours.size());
    if (len < need)
    {
        return 0; // write nothing rather than a truncated report
    }

    std::size_t o = 0;
    out[o++] = rep.measId;
    out[o++] = static_cast<uint8_t>(rep.neighbours.size());
    PutCell(out + o, rep.servingCell);
    o += kCellBytes;
    PutI64(out + o, rep.servingTimingAdvance.GetNanoSeconds());
    o += 8;
    PutF64(out + o, rep.servingSlantRangeM);
    o += 8;
    for (const auto& n : rep.neighbours)
    {
        PutCell(out + o, n);
        o += kCellBytes;
    }
    NS_ASSERT(o == need);
    return o;
}

bool
NtnMeasReportCodec::Parse(const uint8_t* in, std::size_t len, NtnMeasurementReport& rep)
{
    if (in == nullptr || len < kFixedBytes)
    {
        return false;
    }
    std::size_t o = 0;
    const uint8_t measId = in[o++];
    const uint8_t nNeigh = in[o++];
    if (nNeigh > NtnMeasurementReport::kMaxNeighbours)
    {
        return false; // TS 38.331 caps measResultNeighCells at 8
    }
    if (len < SerialisedBytes(nNeigh))
    {
        return false; // truncated: refuse rather than return a partial report
    }

    rep = NtnMeasurementReport{};
    rep.measId = measId;
    GetCell(in + o, rep.servingCell);
    o += kCellBytes;
    rep.servingTimingAdvance = NanoSeconds(GetI64(in + o));
    o += 8;
    rep.servingSlantRangeM = GetF64(in + o);
    o += 8;
    rep.neighbours.resize(nNeigh);
    for (uint8_t i = 0; i < nNeigh; ++i)
    {
        GetCell(in + o, rep.neighbours[i]);
        o += kCellBytes;
    }
    return true;
}

} // namespace ns3
