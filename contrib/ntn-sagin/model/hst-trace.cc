/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#include "hst-trace.h"

#include <cmath>

namespace ns3
{
namespace sagin
{

namespace
{
constexpr double C_LIGHT_MPS = 2.99792458e8;
} // namespace

double
HstTrace::DopplerHzAt(double t_s, double gnb_x_m, double carrierHz) const
{
    const double v = SpeedMps();
    const double x_t = startPos_m + v * t_s;
    // Vector from train to gNB:
    //   dx = gnb_x_m - x_t  (along-track)
    //   dy = dmin_m         (perpendicular)
    const double dx = gnb_x_m - x_t;
    const double dy = dmin_m;
    const double r = std::sqrt(dx * dx + dy * dy);
    if (r < 1e-9)
    {
        return 0.0;
    }
    // Unit vector r̂ pointing from train to gNB.
    const double rxhat = dx / r;
    // Train velocity vector = (v, 0). Radial component along r̂:
    const double vRadial = v * rxhat;
    // Positive Doppler when v · r̂ > 0  (train approaching gNB).
    return vRadial / C_LIGHT_MPS * carrierHz;
}

HstTrace
HstTraceGenerator::Generate(double speed_kmh,
                             double dmin_m,
                             double duration_s,
                             uint32_t numSamples)
{
    HstTrace tr;
    tr.speed_kmh = speed_kmh;
    tr.dmin_m = dmin_m;
    tr.cellSpacing_m = 2.0 * dmin_m;
    tr.startPos_m = 0.0;
    const double v = speed_kmh / 3.6;
    if (numSamples < 2)
        numSamples = 2;
    const double dt = duration_s / static_cast<double>(numSamples - 1);
    tr.samples.reserve(numSamples);
    for (uint32_t i = 0; i < numSamples; ++i)
    {
        const double t = i * dt;
        HstSample s{};
        s.time_s = t;
        s.x_m = v * t;
        s.y_m = 0.0;
        s.z_m = 0.0;
        s.speed_mps = v;
        tr.samples.push_back(s);
    }
    return tr;
}

HstTrace
HstTraceGenerator::PresetTR38901_A(double duration_s, uint32_t numSamples)
{
    return Generate(500.0, 150.0, duration_s, numSamples);
}

HstTrace
HstTraceGenerator::PresetTR38901_B(double duration_s, uint32_t numSamples)
{
    return Generate(300.0, 10.0, duration_s, numSamples);
}

HstTrace
HstTraceGenerator::PresetTR38901_C(double duration_s, uint32_t numSamples)
{
    return Generate(350.0, 100.0, duration_s, numSamples);
}

} // namespace sagin
} // namespace ns3
