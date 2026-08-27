/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only
//
// ntn-meas-report - the TS 38.331 MeasurementReport this module was missing.
//
// Why this exists (audit RRC-5). The module is called ntn-rrc and its README
// once promised "Rel-17 NR-NTN RRC procedures", but a grep for RrcSetup,
// Reconfiguration, Reestablishment or MeasurementReport found nothing. What
// passed for a measurement report was a printf and a counter increment when a
// measured SINR crossed a fixed threshold. That is a log line. It has no
// measId, no quantization, no neighbour list, and nothing receives it - and in
// a results table it reads as TS 38.331 signalling.
//
// This supplies the message itself. It deliberately does NOT add another event
// evaluator: NtnChoAlgorithm already implements the TS 38.331 section 5.5.4
// trigger set (A3 offset, hysteresis, time-to-trigger, Rel-18 D2), and a
// second one would be a copy that could drift. What was missing was the
// report, and in particular the QUANTIZATION - a report carries TS 38.133
// level indices, not the floating-point dB a simulator happens to hold. A
// consumer that reads back -95.7 dBm from a report is reading a number the air
// interface cannot express.
//
// Standards: TS 38.331 section 5.5.5 and section 6.2.2 (MeasurementReport,
// MeasResults, MeasResultServMO, MeasResultNR, ResultsPerCSI-RS-Index),
// TS 38.133 section 10.1.6 (RSRP), section 10.1.11 (RSRQ), section 10.1.16
// (SINR) for the reporting ranges.

#ifndef NTN_MEAS_REPORT_H
#define NTN_MEAS_REPORT_H

#include "ns3/nstime.h"

#include <cstddef>
#include <cstdint>
#include <vector>

namespace ns3
{

/**
 * \brief TS 38.133 measurement-quantity quantization.
 *
 * A MeasurementReport does not carry dB. It carries level indices with a fixed
 * mapping and a fixed range, and values outside the range saturate rather than
 * wrap. Getting this wrong is not cosmetic: it is the difference between a
 * report a real gNB could have sent and one it could not.
 */
class NtnMeasQuantity
{
  public:
    // TS 38.133 Table 10.1.6.1-1: RSRP_LEV 0..127, dBm = index - 156.
    static constexpr int kRsrpMinDbm = -156;
    static constexpr uint8_t kLevelMax = 127;
    /// TS 38.133 Table 10.1.11.1-1: RSRQ_LEV 0..127, dB = index/2 - 43.
    static constexpr double kRsrqMinDb = -43.0;
    /// TS 38.133 Table 10.1.16.1-1: SINR_LEV 0..127, dB = index/2 - 23.
    static constexpr double kSinrMinDb = -23.0;

    static uint8_t RsrpToLevel(double rsrpDbm);
    static double LevelToRsrpDbm(uint8_t level);

    static uint8_t RsrqToLevel(double rsrqDb);
    static double LevelToRsrqDb(uint8_t level);

    static uint8_t SinrToLevel(double sinrDb);
    static double LevelToSinrDb(uint8_t level);
};

/// One cell's results, TS 38.331 MeasResultNR.
struct NtnMeasResultNr
{
    uint16_t physCellId{0};
    uint8_t rsrpLevel{0}; //!< TS 38.133 RSRP_LEV
    uint8_t rsrqLevel{0}; //!< TS 38.133 RSRQ_LEV
    uint8_t sinrLevel{0}; //!< TS 38.133 SINR_LEV
    /// Whether the three quantities are present. TS 38.331 makes each optional,
    /// and a report that omits RSRQ is not the same as one reporting level 0.
    bool haveRsrp{false};
    bool haveRsrq{false};
    bool haveSinr{false};
};

/// TS 38.331 MeasurementReport / MeasResults.
struct NtnMeasurementReport
{
    /// TS 38.331 MeasId, 1..64. Zero means unset, which is not a legal measId
    /// and is how a default-constructed report is told from a real one.
    uint8_t measId{0};
    NtnMeasResultNr servingCell{};
    /// TS 38.331 caps measResultNeighCells at 8 entries per report.
    std::vector<NtnMeasResultNr> neighbours;

    /// NTN assistance carried alongside, so a report from this toolkit says
    /// which geometry produced it. Not part of MeasurementReport in 38.331;
    /// kept explicit so it is never mistaken for standard signalling.
    Time servingTimingAdvance{};
    double servingSlantRangeM{0.0};

    static constexpr std::size_t kMaxNeighbours = 8;
};

/**
 * \brief Serialise/parse NtnMeasurementReport, mirroring Sib19Codec.
 *
 * Fixed-layout little-endian, not ASN.1 PER (this toolkit has no asn1c
 * dependency). Variable length because the neighbour list is variable.
 */
class NtnMeasReportCodec
{
  public:
    /// Bytes for a report with n neighbours.
    static std::size_t SerialisedBytes(std::size_t neighbours);
    /// Returns bytes written, or 0 if `out` is null, too small, or the report
    /// carries more than kMaxNeighbours entries.
    static std::size_t Serialise(const NtnMeasurementReport& rep, uint8_t* out, std::size_t len);
    /// Parse into `rep`. Returns false on a truncated or malformed buffer.
    static bool Parse(const uint8_t* in, std::size_t len, NtnMeasurementReport& rep);
};

} // namespace ns3

#endif // NTN_MEAS_REPORT_H
