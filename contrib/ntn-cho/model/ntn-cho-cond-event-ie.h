/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only
//
// TS 38.331 Rel-17 CondTriggerConfig encodings for the NTN conditional
// handover events condEventT1-r17 and condEventD1-r17 (Wave 6A).
//
// The CHO algorithm used to evaluate both events on unquantized doubles in
// simulation time: T1 against a sim-time epoch with a free-form duration, D1
// against a metre threshold that no RRC message could carry. These structs are
// the fields as RRC signals them, so whatever the algorithm evaluates is
// exactly what a network could have configured:
//
//   condEventT1-r17   t1-Threshold-r17  INTEGER (0..549755813887), units of
//                     10 ms since 00:00:00 on 1 January 1900 (UTC);
//                     duration-r17      INTEGER (1..6000), units of 100 ms.
//                     Entering Mt > Thresh1, leaving Mt > Thresh1 + Duration
//                     (TS 38.331 5.5.4.16), Mt the UE's UTC time in ms.
//   condEventD1-r17   distanceThreshFromReference1/2-r17 INTEGER (0..65525),
//                     units of 50 m; hysteresisLocation-r17 INTEGER
//                     (0..32768), units of 10 m. Entering Ml1 - Hys > Thresh1
//                     AND Ml2 + Hys < Thresh2 (TS 38.331 5.5.4.15).

#ifndef NTN_CHO_COND_EVENT_IE_H
#define NTN_CHO_COND_EVENT_IE_H

#include "ns3/nstime.h"

#include <cstdint>

namespace ns3
{

/// Seconds from 1900-01-01T00:00:00Z (the t1-Threshold epoch) to the Unix epoch.
constexpr double kNtpToUnixOffsetS = 2208988800.0;

/// condEventT1-r17 as signalled (TS 38.331 CondTriggerConfig-r16).
struct CondEventT1Ie
{
    static constexpr uint64_t kThresholdMax = 549755813887ULL; //!< 2^39 - 1
    static constexpr uint16_t kDurationMin = 1;
    static constexpr uint16_t kDurationMax = 6000;

    uint64_t t1Threshold{0}; //!< 10 ms units since 1900-01-01 UTC
    uint16_t duration{1};    //!< 100 ms units, 1..6000

    /// Encode an absolute UTC instant (Unix seconds) and a window duration.
    /// The threshold rounds to the nearest 10 ms and the duration to the
    /// nearest 100 ms; values outside the ASN.1 ranges are clamped and \p
    /// clamped (if given) is set.
    static CondEventT1Ie Encode(double utcUnixSeconds, Time duration, bool* clamped = nullptr);
    /// t1-Threshold as Unix seconds.
    double ThresholdUnixSeconds() const;
    /// duration-r17 as a Time.
    Time DurationTime() const;
    /// Entering condition Mt > Thresh1 and not yet leaving (Mt > Thresh1 +
    /// Duration), with Mt the UE's UTC time; evaluated in integer ms as the
    /// spec defines it.
    bool IsInside(double utcUnixSeconds) const;
};

/// condEventD1-r17 distance fields as signalled.
struct CondEventD1Ie
{
    static constexpr uint32_t kThreshMax = 65525;   //!< x 50 m
    static constexpr uint32_t kHysMax = 32768;      //!< x 10 m

    uint16_t distanceThreshFromReference1{0}; //!< 50 m units (serving reference)
    uint16_t distanceThreshFromReference2{0}; //!< 50 m units (candidate reference)
    uint16_t hysteresisLocation{0};           //!< 10 m units

    /// Distance threshold in metres -> 50 m steps (nearest), clamped.
    static uint16_t EncodeThreshold(double metres, bool* clamped = nullptr);
    /// Hysteresis in metres -> 10 m steps (nearest), clamped.
    static uint16_t EncodeHysteresis(double metres, bool* clamped = nullptr);
    static double ThresholdMetres(uint16_t steps) { return 50.0 * steps; }
    static double HysteresisMetres(uint16_t steps) { return 10.0 * steps; }

    /// TS 38.331 5.5.4.15 entering condition for the serving leg (Ml1).
    bool ServingLegMet(double ml1Metres) const;
    /// TS 38.331 5.5.4.15 entering condition for the candidate leg (Ml2).
    bool CandidateLegMet(double ml2Metres) const;
};

} // namespace ns3

#endif // NTN_CHO_COND_EVENT_IE_H
