/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only
//
// ntn-twin-agreement - compare twin-predicted handovers against ns-3's.
//
// Why this exists (audit TWIN-02). The "twin/sim agreement" CI gate contained
// no simulator. test_gate9_twin_sim_agreement.py compares
// api_server._elevation_deg_ecef against Skyfield's Satellite.elevation_deg -
// both Python, both over the SAME Python-propagated constellation - and states
// in its own docstring that "with no ns-3 runtime available here, the reference
// is the geometric ground truth over identical orbits". The plan document
// nonetheless records it as "Gate 9 (twin/sim HO agreement): PINNED, agreement
// 100%". A 100% score between two elevation formulas over one orbit
// propagation is close to a tautology; it says nothing about whether the twin
// agrees with the simulator.
//
// The number that would matter - twin-predicted handover instants against ns-3
// handover instants - was computed nowhere. This computes it.
//
// It is only possible because of TWIN-01. The twin propagates synthetic Walker
// TLEs with real SGP4; Sgp4MobilityModel used to fall back to Kepler + J2 even
// when handed a TLE, so feeding it the twin's own elements still produced a
// different orbit and any disagreement would have been dominated by geometry.
// With SetTle() now selecting SGP4, both sides run the same propagator on the
// same elements, and a disagreement in handover instants is a disagreement
// about MOBILITY DECISIONS - which is the thing worth measuring.

#ifndef NTN_TWIN_AGREEMENT_H
#define NTN_TWIN_AGREEMENT_H

#include "ns3/nstime.h"
#include "ns3/vector.h"

#include <string>
#include <vector>

namespace ns3
{
namespace ntntwin
{

/// One handover instant: at `t`, the terminal is recommended cell `gnbId`.
struct TwinHandover
{
    double tSec{0.0};
    uint32_t ueId{0};
    uint32_t gnbId{0};
    double confidence{0.0};
};

/// How well two handover sequences line up.
struct TwinAgreement
{
    std::size_t twinCount{0};
    std::size_t simCount{0};
    std::size_t matched{0};      //!< twin events with a sim event inside the window
    double meanAbsOffsetS{0.0};  //!< mean |t_twin - t_sim| over matched pairs
    double maxAbsOffsetS{0.0};
    /// matched / max(twinCount, simCount): 1.0 only when both sequences have
    /// the same length AND every event pairs up.
    double agreement{0.0};
};

/// Parse the prediction file the twin emits and the C++ consumer reads:
/// `t_s,ueId,recommendedGnbId,confidence`, '#' comments ignored.
std::vector<TwinHandover> LoadTwinPredictions(const std::string& path);

/// The geometry the twin used, carried in the prediction file's own header so
/// an ns-3 gate replays the identical scenario instead of being told it out of
/// band and silently diverging.
struct TwinRunMeta
{
    double epochUnix{0.0};
    double observerLatDeg{0.0};
    double observerLonDeg{0.0};
    double observerAltM{0.0};
    double horizonS{0.0};
    double stepS{0.0};
    bool valid{false};
};
TwinRunMeta LoadTwinRunMeta(const std::string& path);

/// Load a 3LE/TLE dump (name, line1, line2 triples).
struct TwinTle
{
    std::string name;
    std::string line1;
    std::string line2;
};
std::vector<TwinTle> LoadTleDump(const std::string& path);

/**
 * \brief Compare two handover-instant sequences.
 *
 * A twin event matches a sim event when they name the SAME cell and their
 * instants are within `windowS`. Matching on the instant alone would score a
 * handover to the wrong satellite as agreement, which is the failure the gate
 * exists to catch.
 */
TwinAgreement CompareHandovers(const std::vector<TwinHandover>& twin,
                               const std::vector<TwinHandover>& sim,
                               double windowS);

} // namespace ntntwin
} // namespace ns3

#endif // NTN_TWIN_AGREEMENT_H
