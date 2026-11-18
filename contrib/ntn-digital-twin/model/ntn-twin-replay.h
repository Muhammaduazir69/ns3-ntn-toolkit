/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only
//
// ntn-twin-replay - replay the twin's scenario inside a running ns-3 simulation
// under the SAME handover policy the twin applies.
//
// Why this exists. The TWIN-02 gate replayed the twin's orbits in ns-3 but
// decided the serving satellite by a bare argmax over elevation, while the twin
// exporter (twin_loop.emit_predictions_file) runs the TWIN-04 A3 guard: a 3 dB
// margin on a link-budget SNR. Two different policies over identical orbits
// produced zero matched handovers on the committed 66-satellite run (5 twin
// events, 11 argmax events, 0 matched), so the gate measured the policy gap and
// said nothing about whether ns-3 and the twin agree on mobility. It also put
// the observer on a sphere of radius 6378137 m at geocentric latitude, while
// the twin observes from the WGS-84 geodetic point, and ranked satellites by
// geocentric elevation while the twin uses the ellipsoid normal.
//
// This file ports the A3 guard (ntn_digital_twin/a3_guard.py) to C++ term for
// term and replays the scenario with it from the geodetic observer, so a
// disagreement now isolates mobility: ns-3 SGP4 + TEME->ECEF against the twin's
// Skyfield SGP4.

#ifndef NTN_TWIN_REPLAY_H
#define NTN_TWIN_REPLAY_H

#include "ntn-twin-agreement.h"

#include "ns3/nstime.h"
#include "ns3/ptr.h"
#include "ns3/vector.h"

#include <cstdint>
#include <limits>
#include <vector>

namespace ns3
{
namespace ntncon
{
class Sgp4MobilityModel;
}

namespace ntntwin
{

/// The A3 guard parameters (a3_guard.A3Params with margin_quantity = "db").
/// Defaults are the twin loop's: 3 dB offset, no time-to-trigger, no minimum
/// service time, and the TR 38.821 Table 6.1.1.1-1/-2 Set-1 S-band LEO-600
/// payload (34 dBW/MHz over 20 MHz = 77 dBm) into the Set-1 handheld
/// (G/T -31.6 dB/K).
struct TwinA3Params
{
    double a3OffsetDb{3.0};
    double timeToTriggerS{0.0};
    double minServiceS{0.0};
    double minElevationDeg{0.0};
    double eirpDbm{77.0};
    double gtDbPerK{-31.6};
    double bandwidthHz{20.0e6};
    double frequencyHz{2.0e9};
};

/// TR 38.811 Section 6.6.2 slant range on a spherical Earth (R = 6371 km):
/// d = -R sin(el) + sqrt((R sin el)^2 + 2 R h + h^2). Elevation clamped to [0, 90].
double TwinSlantRangeM(double elevDeg, double altKm, double earthRadiusM = 6371e3);

/// Link-budget SNR (dB): C/N = EIRP - FSPL + G/T - k - 10 log10(B), with
/// FSPL = 20 log10(d) + 20 log10(f) - 147.55 (TR 38.811 Section 6.6.2). -999 dB
/// below the horizon.
double TwinSnrDb(const TwinA3Params& p, double elevDeg, double altKm);

/// Geocentric altitude (km) above the 6378135 m radius the twin uses.
double TwinAltitudeKmFromEcef(const Vector& ecef);

/// One satellite seen from the observer at one instant.
struct TwinCandidate
{
    uint32_t key{0};
    double elevDeg{-90.0};
    double altKm{600.0};
};

/// A handover the guard admitted.
struct TwinA3Event
{
    double tSec{0.0};
    uint32_t keyIn{0};
    uint32_t keyOut{0};
    double marginDb{0.0};
};

/**
 * \brief The TWIN-04 A3 evaluator, ported from a3_guard.A3Evaluator.
 *
 * Semantics (identical to the Python): the ranking quantity is the comparison
 * quantity; candidates below the minimum elevation cannot serve; initial
 * acquisition is NOT a handover; the time-to-trigger window tracks one target
 * and restarts when the target changes or the condition lapses; a serving cell
 * that drops from view keeps its last geometry.
 */
class TwinA3Evaluator
{
  public:
    explicit TwinA3Evaluator(const TwinA3Params& p = TwinA3Params());
    /// Advance to `tSec`. Returns true and fills `ev` when a handover fires.
    bool Step(double tSec, const std::vector<TwinCandidate>& cands, TwinA3Event& ev);
    bool HasServing() const { return m_hasServing; }
    uint32_t GetServing() const { return m_serving.key; }

  private:
    TwinA3Params m_p;
    bool m_hasServing{false};
    TwinCandidate m_serving;
    bool m_timing{false};
    double m_since{0.0};
    uint32_t m_target{0};
    bool m_hadHo{false};
    double m_lastHoT{0.0};
};

/// Which serving policy the replay applies.
enum class TwinReplayPolicy
{
    ArgmaxElevation, //!< geodetic elevation argmax, no guard
    A3Guard,         //!< the twin's A3 guard on link-budget SNR
};

/**
 * \brief Replays a twin scenario inside a running ns-3 simulation.
 *
 * Loads the twin's TLEs into ntncon::Sgp4MobilityModel (full Vallado SGP4 and
 * TEME->ECEF), places the observer at the WGS-84 geodetic point named in the
 * prediction header, and schedules one sample per step on the simulator clock.
 * Each sample measures geodetic elevations, runs the policy, and appends a
 * TwinHandover (1-based cell id, as the twin numbers cells) on every handover.
 * Initial acquisition is reported separately and is not a handover.
 */
class TwinReplay
{
  public:
    TwinReplay();
    ~TwinReplay();

    /// Load the satellites. Returns how many TLEs initialized SGP4.
    std::size_t SetTles(const std::vector<TwinTle>& tles);
    /// Observer and sampling window (normally from LoadTwinRunMeta).
    void SetRunMeta(const TwinRunMeta& meta);
    void SetPolicy(TwinReplayPolicy p) { m_policy = p; }
    void SetA3Params(const TwinA3Params& p) { m_a3 = p; }
    /// Shift the simulation epoch by `dt` seconds against the TLE epoch, i.e. the
    /// replay runs with a clock that disagrees with the twin's shared epoch.
    void SetEpochSkewS(double dt) { m_epochSkewS = dt; }

    /// Schedule the sampling events. Call before Simulator::Run(); returns
    /// false when there is nothing to replay (no satellites or invalid meta).
    bool Schedule();

    const std::vector<TwinHandover>& GetHandovers() const { return m_handovers; }
    /// 1-based cell acquired at the first sample with a visible satellite, 0 if none.
    uint32_t GetInitialCell() const { return m_initialCell; }
    double GetInitialAcquisitionS() const { return m_initialT; }
    std::size_t GetSampleCount() const { return m_samples; }
    std::size_t GetSamplesWithoutCoverage() const { return m_noCoverage; }
    /// Geodetic elevation (deg) of 1-based cell `cell` at the current sim time.
    double ElevationOfCellDeg(uint32_t cell) const;
    /// SGP4 propagation failures summed over all satellites.
    uint64_t GetSgp4Fallbacks() const;
    std::size_t GetNumSatellites() const { return m_sats.size(); }

  private:
    void Sample();

    std::vector<Ptr<ntncon::Sgp4MobilityModel>> m_sats;
    TwinRunMeta m_meta;
    TwinReplayPolicy m_policy{TwinReplayPolicy::A3Guard};
    TwinA3Params m_a3;
    TwinA3Evaluator m_eval;
    double m_epochSkewS{0.0};
    std::vector<TwinHandover> m_handovers;
    uint32_t m_initialCell{0};
    double m_initialT{-1.0};
    int64_t m_argmaxPrev{-1};
    std::size_t m_samples{0};
    std::size_t m_noCoverage{0};
};

} // namespace ntntwin
} // namespace ns3

#endif // NTN_TWIN_REPLAY_H
