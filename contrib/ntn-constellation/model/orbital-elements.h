/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#ifndef NTN_CONSTELLATION_ORBITAL_ELEMENTS_H
#define NTN_CONSTELLATION_ORBITAL_ELEMENTS_H

// Keplerian orbital elements + TLE parser (Roadmap §3 T5).
//
// `KeplerianElements` is the canonical input to Sgp4MobilityModel. A
// TLE record can be parsed into these elements via TleRecord::ToKeplerian.
// All angles are in radians, distances in metres, time in seconds.
//
// The v2.1 toolkit uses a Kepler + J2 secular propagator that is accurate
// to within a few km over hours of propagation for the typical Walker
// constellation geometries this module ships (Starlink-A, OneWeb, etc.).
// Full Vallado SGP4 is a Q4 2026 follow-on inside T5; the TleRecord
// parser already extracts the SGP4-specific drag (Bstar) field so the
// upgrade is a backend swap.

#include <cstdint>
#include <string>
#include <vector>

namespace ns3
{
namespace ntncon
{

/// Earth constants — WGS-84.
inline constexpr double kEarthRadiusM = 6378137.0;
inline constexpr double kEarthMuM3S2 = 3.986004418e14; //!< GM (m^3/s^2)
inline constexpr double kEarthJ2 = 1.08262668e-3;
inline constexpr double kEarthRotationRadS = 7.2921159e-5; //!< rad/s

/// Classical orbital elements.
struct KeplerianElements
{
    double semi_major_axis_m{0.0};   //!< a
    double eccentricity{0.0};        //!< e (0 = circular)
    double inclination_rad{0.0};     //!< i
    double raan_rad{0.0};            //!< Right Ascension of Ascending Node
    double arg_perigee_rad{0.0};     //!< omega
    double mean_anomaly_rad{0.0};    //!< M at epoch
    double epoch_unix_s{0.0};        //!< UTC seconds since Unix epoch
    double bstar{0.0};               //!< SGP4 drag term (read-only in T5 v1)
    uint32_t norad_id{0};            //!< satellite catalog number
    std::string name;

    /// Mean motion in rad/s = sqrt(GM / a^3).
    double MeanMotionRadS() const;

    /// Orbital period in seconds = 2*pi / n.
    double PeriodSeconds() const;
};

/// Raw two-line element record (lines 1 and 2 of a TLE, plus the optional
/// 0-line satellite name). All fields are exactly as they appear on the
/// wire so reviewers can grep the columns.
struct TleRecord
{
    std::string name;   //!< from optional 0-line
    std::string line1;  //!< 69-char TLE line 1
    std::string line2;  //!< 69-char TLE line 2

    /// Parse the orbital elements out of `line1` and `line2`.
    /// Returns false on any column/format error.
    bool ToKeplerian(KeplerianElements& out) const;

    /// Verify the modulo-10 line checksum that appears in TLE column 69.
    /// Returns true when line1 and line2 both pass.
    bool ChecksumOk() const;
};

/// Convenience parser that consumes a multi-line string containing zero
/// or more 0/1/2-line groups and returns the parsed records.
/// Lines starting with '#' are treated as comments.
std::size_t
ParseTleStream(const std::string& body, std::vector<TleRecord>& out);

} // namespace ntncon
} // namespace ns3

#endif // NTN_CONSTELLATION_ORBITAL_ELEMENTS_H
