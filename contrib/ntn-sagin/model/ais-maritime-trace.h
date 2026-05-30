/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#ifndef NTN_SAGIN_AIS_MARITIME_TRACE_H
#define NTN_SAGIN_AIS_MARITIME_TRACE_H

// AIS maritime bulk-dump trace importer (Roadmap §4.4.2).
//
// Parses the Danish Maritime Authority bulk AIS exports
//   https://web.ais.dk/aisdata/
// into per-MMSI vessel state vectors. The Piraeus port dataset (244 M
// records) uses a compatible column set so the same importer ingests
// both. Each AIS sample carries: timestamp, lat, lon, SOG (Speed Over
// Ground, knots), COG (Course Over Ground, deg), heading, ship type.
//
// Companion `AisMobilityModel` replays a vessel trajectory under ns-3
// simulation time, anchored to a user-supplied (lat_ref, lon_ref) origin
// via local ENU coordinates. Altitude is fixed at 0 m (sea level).
//
// CSV header expected (Danish Maritime canonical layout, comma-separated):
//   # Timestamp,Type of mobile,MMSI,Latitude,Longitude,Navigational status,
//   ROT,SOG,COG,Heading,IMO,Callsign,Name,Ship type,Cargo type,Width,Length,
//   Type of position fixing device,Draught,Destination,ETA,Data source type,
//   A,B,C,D

#include <cstdint>
#include <map>
#include <string>
#include <vector>

namespace ns3
{
namespace sagin
{

/// One AIS state-vector sample.
struct AisSample
{
    double time_s;        //!< absolute time, seconds since Unix epoch
    double lat_deg;
    double lon_deg;
    double sog_knots;     //!< Speed Over Ground (knots; 1 knot = 0.5144 m/s)
    double cog_deg;       //!< Course Over Ground, 0..360 deg, true north
    double heading_deg;   //!< Hull heading, 0..360 deg (511 = not available)
    std::string ship_type;
    std::string callsign;
};

struct AisMaritimeTrace
{
    uint32_t mmsi{0};
    std::string ship_type;
    std::vector<AisSample> samples; //!< sorted ascending by time_s

    double TStart() const;
    double TEnd() const;
    AisSample InterpolateAt(double time_s) const;
};

class AisDanishImporter
{
  public:
    /// Parse a Danish Maritime bulk-dump CSV. `tStartAbs_s` / `tEndAbs_s`
    /// filter the wall-clock window. Returns a map keyed by MMSI.
    std::map<uint32_t, AisMaritimeTrace>
        LoadCsv(const std::string& path,
                 double tStartAbs_s = 0.0,
                 double tEndAbs_s = 1e18);

    size_t LastRowsRead() const { return m_lastRowsRead; }
    size_t LastRowsSkipped() const { return m_lastRowsSkipped; }

  private:
    size_t m_lastRowsRead{0};
    size_t m_lastRowsSkipped{0};
};

} // namespace sagin
} // namespace ns3

#endif // NTN_SAGIN_AIS_MARITIME_TRACE_H
