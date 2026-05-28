/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#ifndef NTN_SIONNA_CIR_DOPPLER_SYNTH_H
#define NTN_SIONNA_CIR_DOPPLER_SYNTH_H

// C++ equivalent of Sionna's `paths.apply_doppler()` (Roadmap §4.2.11).
//
// Sionna RT v2 exposes `Paths.apply_doppler(sampling_frequency,
// num_time_steps, tx_velocities, rx_velocities)` to synthesise the
// time-varying CIR between two ray-tracer snapshots without re-running
// the (expensive) RT solver. For NTN simulations where the satellite
// moves several km between RT queries, this lets the caller interpolate
// each tap's phase rotation analytically while caching the snapshot.
//
// This C++ helper mirrors the Sionna formula for a single complex
// tap p:
//
//     a_p(t) = a_p(0) · exp(j 2π f_d,p t)
//
// where the per-tap Doppler is:
//
//     f_d,p = (v_tx · k_tx,p + v_rx · k_rx,p) / λ
//
// and k_{*,p} is the unit direction vector at the {tx,rx} side for
// path p.

#include <ns3/object.h>
#include <ns3/vector.h>

#include <complex>
#include <cstdint>
#include <vector>

namespace ns3
{

/// One ray-tracer path tap.
struct CirPathTap
{
    /// Complex baseband amplitude at snapshot time.
    std::complex<double> amplitude;
    /// Propagation delay (seconds).
    double delay_s;
    /// Unit direction vector at the tx side (path leaves the tx).
    Vector tx_dir;
    /// Unit direction vector at the rx side (path arrives at rx).
    Vector rx_dir;
};

/// Snapshot of the CIR + reference geometry at time `t_ref`.
struct CirSnapshot
{
    std::vector<CirPathTap> taps;
    /// Carrier wavelength (m) for the doppler computation.
    double wavelength_m{0.01};
    /// Reference snapshot time (s). The synthesiser uses this as
    /// the integration anchor; subsequent CIRs are produced at
    /// `t_ref + dt`.
    double t_ref_s{0.0};
};

/// Synthesised CIR at time `t_ref + dt`.
struct CirEvolution
{
    /// Tap amplitudes after Doppler rotation. Same shape + delays as
    /// the snapshot.
    std::vector<std::complex<double>> amplitudes;
    /// Per-tap Doppler in Hz (signed).
    std::vector<double> doppler_hz;
};

class CirDopplerSynthesizer
{
  public:
    /// Synthesise the CIR at offset `dt` relative to the snapshot
    /// reference time, given constant tx/rx velocities. Velocity
    /// vectors are (m/s) in the same ECEF frame as the direction
    /// vectors stored on the taps.
    static CirEvolution
        Synthesize(const CirSnapshot& snap,
                    const Vector& tx_velocity_mps,
                    const Vector& rx_velocity_mps,
                    double dt_s);

    /// Convenience: produce a time series of `num_steps` evolutions
    /// at sampling frequency `fs_hz` starting from the snapshot.
    static std::vector<CirEvolution>
        SynthesizeSeries(const CirSnapshot& snap,
                          const Vector& tx_velocity_mps,
                          const Vector& rx_velocity_mps,
                          double fs_hz,
                          uint32_t num_steps);
};

} // namespace ns3

#endif // NTN_SIONNA_CIR_DOPPLER_SYNTH_H
