/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#include "ntn-rach-window.h"

#include "ns3/log.h"

#include <algorithm>
#include <cmath>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("NtnRachWindow");

Time
NtnRachWindow::RoundTripForSlantRange(double slantRangeMeters)
{
    if (slantRangeMeters <= 0.0)
    {
        return Time();
    }
    const double oneWaySec = slantRangeMeters / 299792458.0;
    return Seconds(2.0 * oneWaySec);
}

Time
NtnRachWindow::SlotPeriodForNumerology(uint8_t mu)
{
    // TS 38.211 Table 4.2-1: SCS = 15 * 2^mu kHz, slot = 1 ms / 2^mu.
    const uint32_t div = 1u << std::min<uint8_t>(mu, 6);
    return NanoSeconds(1000000 / div);
}

NtnRachWindowVerdict
NtnRachWindow::Evaluate(Time roundTrip, Time slotPeriod, Time processing)
{
    NtnRachWindowVerdict v;
    v.roundTrip = roundTrip;
    v.slotPeriod = slotPeriod;
    // A zero processing allowance would assert the gNB detects a preamble and
    // assembles a RAR in no time, so default to one slot rather than nothing.
    v.processing = (processing == Time()) ? slotPeriod : processing;

    if (slotPeriod <= Time())
    {
        return v; // nothing sensible to say without a numerology
    }

    v.requiredWindow = roundTrip + v.processing;

    // Invert slotPeriod * (6 + N) >= requiredWindow for N, rounding up: a
    // window one slot short is a window that times out.
    const double slots = static_cast<double>(v.requiredWindow.GetNanoSeconds()) /
                         static_cast<double>(slotPeriod.GetNanoSeconds());
    const double needed = std::ceil(slots) - static_cast<double>(kNrWindowBaseSlots);
    v.requiredWindowSize = needed <= 0.0 ? 0u : static_cast<uint32_t>(needed);

    v.appliedWindowSize = static_cast<uint8_t>(
        std::min<uint32_t>(std::max<uint32_t>(v.requiredWindowSize, kNrWindowSizeMin),
                           kNrWindowSizeMax));
    v.fits = v.requiredWindowSize <= kNrWindowSizeMax;

    v.configuredWindow = slotPeriod * (kNrWindowBaseSlots + v.appliedWindowSize);
    v.shortfall = v.configuredWindow >= v.requiredWindow ? Time()
                                                         : v.requiredWindow - v.configuredWindow;

    NS_LOG_INFO("RTT " << roundTrip.GetMilliSeconds() << " ms, slot "
                       << slotPeriod.GetMicroSeconds() << " us -> need N="
                       << v.requiredWindowSize << " (applied " << +v.appliedWindowSize
                       << (v.fits ? ", fits" : ", DOES NOT FIT") << ", shortfall "
                       << v.shortfall.GetMilliSeconds() << " ms)");
    return v;
}

} // namespace ns3
