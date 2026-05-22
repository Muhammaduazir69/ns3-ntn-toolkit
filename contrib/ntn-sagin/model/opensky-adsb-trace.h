/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#ifndef NTN_SAGIN_OPENSKY_ADSB_TRACE_H
#define NTN_SAGIN_OPENSKY_ADSB_TRACE_H

// OpenSky Network ADS-B bulk-dump trace importer (Roadmap §4.4.1).
//
// Reads the publicly-redistributable OpenSky bulk-dump CSV format
//   https://opensky-network.org/datasets/states/
// into a per-icao24 map of sampled aircraft states. Each sample carries
// the wall-clock time (s since epoch), WGS-84 latitude/longitude, altitude
// (barometric in metres preferred, geometric as fallback), ground-speed
// (m/s) and heading (degrees from true north).
//
// The importer expects the header row from the OpenSky bulk dumps:
//   time,icao24,lat,lon,velocity,heading,vertrate,callsign,onground,
//   alert,spi,squawk,baroaltitude,geoaltitude,lastposupdate,lastcontact
// but tolerates re-orderings (columns are looked up by header name) and
// skips rows with empty lat/lon. Aircraft that are `onground=true` are
// imported with altitude clamped to 0 m.
//
// The companion `OpenSkyMobilityModel` consumes a trace and replays the
// aircraft trajectory in ns-3 simulation time, anchored to a user-supplied
// reference (lat_ref, lon_ref, alt_ref) origin via local ENU coordinates.

#include <cstdint>
#include <map>
#include <string>
#include <vector>

namespace ns3
{
namespace sagin
{

/// One ADS-B state-vector sample.
struct OpenSkySample
{
    double time_s;     //!< wall-clock seconds (Unix epoch in the bulk dumps)
    double lat_deg;
    double lon_deg;
    double alt_m;      //!< barometric altitude (or 0 when onground)
    double velocity_mps; //!< ground speed
    double heading_deg;  //!< 0 = true north, increasing clockwise
};

/// One aircraft's trace: ordered samples, possibly with gaps.
struct OpenSkyAdsbTrace
{
    std::string icao24;
    std::vector<OpenSkySample> samples; //!< sorted ascending by time_s

    /// Earliest / latest sample times. Returns NaN if empty.
    double TStart() const;
    double TEnd() const;

    /// Interpolated sample at `time_s`. Outside the trace's [TStart, TEnd]
    /// the boundary sample is returned (no extrapolation). The trace must
    /// be non-empty.
    OpenSkySample InterpolateAt(double time_s) const;
};

/// Parse an OpenSky bulk-dump CSV file. Returns one trace per unique
/// icao24 (sorted in ascending time order). Returns an empty map on I/O
/// or parse error.
class OpenSkyAdsbImporter
{
  public:
    /// `tStartAbs_s` and `tEndAbs_s` clamp the wall-clock window; samples
    /// outside this range are skipped. Pass `0` and `1e18` for no filter.
    std::map<std::string, OpenSkyAdsbTrace> LoadCsv(const std::string& path,
                                                      double tStartAbs_s = 0.0,
                                                      double tEndAbs_s = 1e18);

    /// Diagnostics — last parse stats.
    size_t LastRowsRead() const { return m_lastRowsRead; }
    size_t LastRowsSkipped() const { return m_lastRowsSkipped; }

  private:
    size_t m_lastRowsRead{0};
    size_t m_lastRowsSkipped{0};
};

} // namespace sagin
} // namespace ns3

#endif // NTN_SAGIN_OPENSKY_ADSB_TRACE_H
