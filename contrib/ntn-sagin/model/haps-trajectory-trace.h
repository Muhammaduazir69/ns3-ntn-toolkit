/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#ifndef NTN_SAGIN_HAPS_TRAJECTORY_TRACE_H
#define NTN_SAGIN_HAPS_TRAJECTORY_TRACE_H

// HAPS trajectory CSV ingest (Roadmap §4.4.8).
//
// Reads HAPS station-keeping / waypoint traces in the format published
// by Kea Atmos Mk1 mission ops dumps and similar Aero/Aerostat-style
// platforms. Each row is a sampled state-vector:
//
//   time_s,lat_deg,lon_deg,alt_m[,heading_deg][,speed_mps]
//
// `heading_deg` and `speed_mps` are optional. Header row is mandatory
// and column lookup is by name (header order can vary). Rows with
// blank lat/lon are skipped.
//
// `HapsTrajectoryTrace::InterpolateAt(t_s)` linearly interpolates
// each field between adjacent samples (no extrapolation: at boundary
// times it returns the boundary sample). Speed/heading are
// interpolated if present on both flanks; otherwise default 0.
//
// A companion `HapsTrajectoryMobilityModel` (see haps-trajectory-
// mobility-model.h) wraps an ns-3 MobilityModel that consumes the
// trace and exposes position + velocity at every Simulator::Now().

#include <cstdint>
#include <map>
#include <string>
#include <vector>

namespace ns3
{
namespace sagin
{

struct HapsTrajectorySample
{
    double time_s{0.0};   //!< wall-clock seconds (mission-relative or Unix)
    double lat_deg{0.0};
    double lon_deg{0.0};
    double alt_m{0.0};
    double heading_deg{0.0}; //!< 0 = true north, increasing clockwise
    double speed_mps{0.0};
    bool has_heading{false};
    bool has_speed{false};
};

struct HapsTrajectoryTrace
{
    std::string platform_id;
    std::vector<HapsTrajectorySample> samples;

    double TStart() const;
    double TEnd() const;

    /// Interpolate fields at `time_s`. If the trace is non-empty and
    /// `time_s` is outside [TStart, TEnd] the boundary sample is
    /// returned verbatim.
    HapsTrajectorySample InterpolateAt(double time_s) const;
};

class HapsTrajectoryImporter
{
  public:
    /// Load a CSV file. If the file has a `platform_id` column, traces
    /// are keyed by it; otherwise everything goes under "haps".
    /// `tStart_s` / `tEnd_s` clamp the wall-clock window.
    std::map<std::string, HapsTrajectoryTrace>
        LoadCsv(const std::string& path,
                 double tStart_s = 0.0,
                 double tEnd_s = 1e18);

    size_t LastRowsRead() const { return m_lastRowsRead; }
    size_t LastRowsSkipped() const { return m_lastRowsSkipped; }

  private:
    size_t m_lastRowsRead{0};
    size_t m_lastRowsSkipped{0};
};

} // namespace sagin
} // namespace ns3

#endif // NTN_SAGIN_HAPS_TRAJECTORY_TRACE_H
