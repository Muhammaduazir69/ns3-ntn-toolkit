/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#include "ntn-cho-cond-event-ie.h"

#include <algorithm>
#include <cmath>

namespace ns3
{

CondEventT1Ie
CondEventT1Ie::Encode(double utcUnixSeconds, Time duration, bool* clamped)
{
    bool c = false;
    CondEventT1Ie ie;
    // 10 ms units since 1900: (unix + 2208988800) * 100. Rounded in long
    // double so a present-day epoch (about 3.9e11 units) keeps its 10 ms digit.
    const long long units =
        std::llround((static_cast<long double>(utcUnixSeconds) + kNtpToUnixOffsetS) * 100.0L);
    if (units < 0)
    {
        ie.t1Threshold = 0;
        c = true;
    }
    else if (static_cast<unsigned long long>(units) > kThresholdMax)
    {
        ie.t1Threshold = kThresholdMax;
        c = true;
    }
    else
    {
        ie.t1Threshold = static_cast<uint64_t>(units);
    }
    const long long d = std::llround(duration.GetSeconds() * 10.0);
    if (d < kDurationMin || d > kDurationMax)
    {
        c = true;
    }
    ie.duration = static_cast<uint16_t>(
        std::clamp<long long>(d, kDurationMin, kDurationMax));
    if (clamped)
    {
        *clamped = c;
    }
    return ie;
}

double
CondEventT1Ie::ThresholdUnixSeconds() const
{
    return static_cast<double>(t1Threshold) / 100.0 - kNtpToUnixOffsetS;
}

Time
CondEventT1Ie::DurationTime() const
{
    return MilliSeconds(100 * static_cast<int64_t>(duration));
}

bool
CondEventT1Ie::IsInside(double utcUnixSeconds) const
{
    // Mt in ms since 1900; Thresh1 = 10 x t1-Threshold ms; Duration = 100 x duration ms.
    const long double mtMs =
        (static_cast<long double>(utcUnixSeconds) + kNtpToUnixOffsetS) * 1000.0L;
    const long double thrMs = 10.0L * static_cast<long double>(t1Threshold);
    const long double durMs = 100.0L * static_cast<long double>(duration);
    return mtMs > thrMs && !(mtMs > thrMs + durMs);
}

uint16_t
CondEventD1Ie::EncodeThreshold(double metres, bool* clamped)
{
    const long long s = std::llround(metres / 50.0);
    if (clamped)
    {
        *clamped = (s < 0 || s > static_cast<long long>(kThreshMax));
    }
    return static_cast<uint16_t>(std::clamp<long long>(s, 0, kThreshMax));
}

uint16_t
CondEventD1Ie::EncodeHysteresis(double metres, bool* clamped)
{
    const long long s = std::llround(metres / 10.0);
    if (clamped)
    {
        *clamped = (s < 0 || s > static_cast<long long>(kHysMax));
    }
    return static_cast<uint16_t>(std::clamp<long long>(s, 0, kHysMax));
}

bool
CondEventD1Ie::ServingLegMet(double ml1Metres) const
{
    // Ml1 - Hys > Thresh1
    return ml1Metres - HysteresisMetres(hysteresisLocation) >
           ThresholdMetres(distanceThreshFromReference1);
}

bool
CondEventD1Ie::CandidateLegMet(double ml2Metres) const
{
    // Ml2 + Hys < Thresh2
    return ml2Metres + HysteresisMetres(hysteresisLocation) <
           ThresholdMetres(distanceThreshFromReference2);
}

} // namespace ns3
