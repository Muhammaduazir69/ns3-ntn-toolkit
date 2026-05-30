/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#ifndef NTN_CONSTELLATION_WALKER_CONSTELLATION_H
#define NTN_CONSTELLATION_WALKER_CONSTELLATION_H

// Walker-Delta / Walker-Star constellation generator (Roadmap §3 T5).
//
// Walker notation: i:T/P/F where
//   i  = inclination (deg)
//   T  = total number of satellites
//   P  = number of planes
//   F  = phasing parameter (0..P-1; controls satellite spacing between
//        adjacent planes)
// Satellites per plane S = T / P. Plane n RAAN = n * 360/P (Delta) or
// n * 180/P (Star). Within plane n, satellite m has mean anomaly
// m * 360/S + n * F * 360/T.

#include "orbital-elements.h"

#include <vector>

namespace ns3
{
namespace ntncon
{

struct WalkerConfig
{
    double inclination_deg{53.0};
    uint32_t total_sats{66};
    uint32_t num_planes{6};
    uint32_t phasing_f{1};
    double altitude_km{550.0};
    double eccentricity{0.0};
    double arg_perigee_deg{0.0};
    /// Reference epoch (Unix seconds) at which the constellation is
    /// instantiated. Each satellite inherits this epoch.
    double epoch_unix_s{0.0};
};

class WalkerConstellation
{
  public:
    /// Build a Walker-Delta constellation (T/P/F notation).
    static std::vector<KeplerianElements> BuildDelta(const WalkerConfig& cfg);

    /// Build a Walker-Star constellation (RAAN spread 0..180°; polar
    /// inclinations).
    static std::vector<KeplerianElements> BuildStar(const WalkerConfig& cfg);
};

} // namespace ntncon
} // namespace ns3

#endif // NTN_CONSTELLATION_WALKER_CONSTELLATION_H
