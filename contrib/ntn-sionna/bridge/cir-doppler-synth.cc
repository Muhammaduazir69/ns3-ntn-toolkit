/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#include "cir-doppler-synth.h"

#include <cmath>

namespace ns3
{

namespace
{

double
DotProduct(const Vector& a, const Vector& b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

} // namespace

CirEvolution
CirDopplerSynthesizer::Synthesize(const CirSnapshot& snap,
                                    const Vector& tx_velocity_mps,
                                    const Vector& rx_velocity_mps,
                                    double dt_s)
{
    CirEvolution out;
    out.amplitudes.reserve(snap.taps.size());
    out.doppler_hz.reserve(snap.taps.size());
    const double inv_lambda =
        (snap.wavelength_m > 0.0) ? 1.0 / snap.wavelength_m : 0.0;
    for (const auto& tap : snap.taps)
    {
        const double f_d =
            (DotProduct(tx_velocity_mps, tap.tx_dir) +
              DotProduct(rx_velocity_mps, tap.rx_dir)) *
            inv_lambda;
        const double phi = 2.0 * M_PI * f_d * dt_s;
        const std::complex<double> rot(std::cos(phi), std::sin(phi));
        out.amplitudes.push_back(tap.amplitude * rot);
        out.doppler_hz.push_back(f_d);
    }
    return out;
}

std::vector<CirEvolution>
CirDopplerSynthesizer::SynthesizeSeries(const CirSnapshot& snap,
                                         const Vector& tx_velocity_mps,
                                         const Vector& rx_velocity_mps,
                                         double fs_hz,
                                         uint32_t num_steps)
{
    std::vector<CirEvolution> series;
    if (num_steps == 0 || fs_hz <= 0.0)
    {
        return series;
    }
    series.reserve(num_steps);
    const double dt = 1.0 / fs_hz;
    for (uint32_t k = 0; k < num_steps; ++k)
    {
        const double t = dt * static_cast<double>(k);
        series.push_back(Synthesize(snap,
                                       tx_velocity_mps,
                                       rx_velocity_mps,
                                       t));
    }
    return series;
}

} // namespace ns3
