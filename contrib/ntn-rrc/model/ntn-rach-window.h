/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only
//
// ntn-rach-window - sizing the random-access response window for an NTN cell.
//
// Why this exists (audit RRC-4). NtnTimingAdvance says in its own header that
// "without pre-compensation, RACH preambles arrive far outside their reception
// window", and the toolkit had nothing that checked the claim: no PRACH, no
// RAR, no ra-ResponseWindow anywhere. The TA value could be shown to exist but
// never to be sufficient, which leaves the SIB19 -> TA -> RACH chain open at
// exactly the end that matters.
//
// The window is where the geometry bites. A preamble leaves the UE, flies the
// slant range, is detected, and the response flies back; the UE gives up when
// its ra-ResponseWindow expires. Over a terrestrial cell the flight is
// microseconds and the window is generous. Over LEO-600 the round trip is
// about 4 ms and over GEO about 240 ms, so whether access completes at all is
// decided by arithmetic on the window length.
//
// This class is deliberately free of any radio-stack dependency: it takes a
// round trip and a slot period and returns the window a stack would need. The
// application to a live NrGnbMac lives in NtnRealStackHelper, which already
// links nr.
//
// Standards: TS 38.213 section 8.1 (ra-ResponseWindow), TS 38.331
// RACH-ConfigGeneric, TR 38.821 section 7.3 (NTN PRACH and RAR-window
// extension), TS 38.321 section 5.1 (the four-step procedure).

#ifndef NTN_RACH_WINDOW_H
#define NTN_RACH_WINDOW_H

#include "ns3/nstime.h"

#include <cstdint>

namespace ns3
{

/// What a given geometry demands of the RAR window, and whether the stack can
/// express it.
struct NtnRachWindowVerdict
{
    Time roundTrip{};        //!< two-way service-link propagation
    Time processing{};       //!< gNB preamble detection + RAR assembly allowance
    Time slotPeriod{};       //!< numerology-dependent slot length
    Time requiredWindow{};   //!< the window length the geometry demands
    Time configuredWindow{}; //!< what the chosen responseWindowSize actually buys
    /// ra-ResponseWindow in the units nr's NrGnbMac attribute uses. nr computes
    /// the deadline as slotPeriod * (6 + responseWindowSize), so this is that N.
    uint32_t requiredWindowSize{0};
    /// The same value clamped into the attribute's [2, 10] range.
    uint8_t appliedWindowSize{0};
    /// True when requiredWindowSize survived the clamp, i.e. the stack can
    /// actually express a window this long.
    bool fits{false};
    /// How much window the geometry needs beyond what the stack can express.
    /// Zero when fits is true.
    Time shortfall{};
};

/**
 * \brief Size the RAR window for an NTN cell from its geometry.
 *
 * The nr UE MAC arms its timeout at slotPeriod * (6 + raResponseWindowSize)
 * from the instant the preamble is transmitted (NrUeMac::SendRaPreamble, TS
 * 38.321 section 5.1.4). The response cannot exist before the preamble has
 * flown up, been detected, and the RAR has flown back, so the window must
 * cover the whole round trip plus the gNB's own turnaround or the UE declares
 * a timeout on a message that is still in flight.
 *
 * The 6 is nr's own fixed head start and is not ours to change; only N is
 * configurable, and only within [2, 10]. That cap is the interesting part: it
 * bounds the longest window the stack can express, and for a GEO cell the
 * bound is exceeded by a wide margin. Reporting that honestly is the point of
 * the verdict struct - a silent clamp would turn "this constellation cannot
 * complete random access on this stack" into "access failed for some reason".
 */
class NtnRachWindow
{
  public:
    /// nr's fixed offset in slotPeriod * (6 + N). TS 38.321 section 5.1.4.
    static constexpr uint32_t kNrWindowBaseSlots = 6;
    /// Bounds of NrGnbMac's RaResponseWindowSize attribute checker.
    static constexpr uint8_t kNrWindowSizeMin = 2;
    static constexpr uint8_t kNrWindowSizeMax = 10;

    /**
     * \param roundTrip two-way propagation on the service link.
     * \param slotPeriod slot length for the configured numerology.
     * \param processing allowance for preamble detection and RAR assembly at
     *        the gNB. Defaults to one slot, which is the smallest value that
     *        is not a claim that detection is free.
     */
    static NtnRachWindowVerdict Evaluate(Time roundTrip,
                                         Time slotPeriod,
                                         Time processing = Time());

    /// Round-trip propagation for a straight-line slant range, in metres.
    static Time RoundTripForSlantRange(double slantRangeMeters);

    /// Slot length for a 3GPP numerology mu (TS 38.211 Table 4.2-1):
    /// 1 ms / 2^mu.
    static Time SlotPeriodForNumerology(uint8_t mu);
};

} // namespace ns3

#endif // NTN_RACH_WINDOW_H
