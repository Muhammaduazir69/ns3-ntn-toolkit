/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#ifndef NTN_SAGIN_OPENSKY_MOBILITY_MODEL_H
#define NTN_SAGIN_OPENSKY_MOBILITY_MODEL_H

// Mobility model that replays an OpenSky ADS-B trace (Roadmap §4.4.1).
//
// Anchors the trace at a user-supplied (lat_ref, lon_ref, alt_ref) origin
// and projects each sample's (lat, lon, alt) into local ENU metres:
//   x_m = (lon - lon_ref) * deg2rad * R_earth * cos(lat_ref * deg2rad)
//   y_m = (lat - lat_ref) * deg2rad * R_earth
//   z_m = alt_m - alt_ref
// Valid for traces spanning < ~500 km from the reference (typical NTN cell
// footprint). Longer traces should use the full ECEF transform; the v2.1
// baseline keeps the equirectangular approximation since the closed-form
// error stays under 1 km for cell-scale geometries.
//
// `t_offset_s` lets the user align trace wall-clock time with simulator
// time zero, e.g. SetTraceTimeOffsetSeconds(trace.TStart()) makes the
// trace start at Simulator::Now() == 0.

#include "opensky-adsb-trace.h"

#include "ns3/mobility-model.h"

namespace ns3
{
namespace sagin
{

class OpenSkyMobilityModel : public MobilityModel
{
  public:
    static TypeId GetTypeId();
    OpenSkyMobilityModel();
    ~OpenSkyMobilityModel() override = default;

    /// Install a trace. The model retains a copy.
    void SetTrace(const OpenSkyAdsbTrace& trace);

    /// Reference origin for the ENU projection.
    void SetReference(double lat_deg, double lon_deg, double alt_m);

    /// Wall-clock time at which the trace samples are anchored. Default is
    /// the trace's TStart() so Simulator::Now()=0 aligns with the first
    /// sample.
    void SetTraceTimeOffsetSeconds(double t_offset_s);

    /// Current interpolated trace sample in geographic coordinates.
    OpenSkySample CurrentSample() const;

    /// The ENU x-axis distance in metres from the reference at sim time t.
    double CurrentEastM() const;
    double CurrentNorthM() const;
    double CurrentUpM() const;

  private:
    Vector DoGetPosition() const override;
    void DoSetPosition(const Vector& position) override;
    Vector DoGetVelocity() const override;

    OpenSkySample SampleAtSimulatorNow() const;
    Vector ToEnu(const OpenSkySample& s) const;

    OpenSkyAdsbTrace m_trace;
    double m_latRefDeg{0.0};
    double m_lonRefDeg{0.0};
    double m_altRefM{0.0};
    double m_traceTimeOffset_s{0.0};
};

} // namespace sagin
} // namespace ns3

#endif // NTN_SAGIN_OPENSKY_MOBILITY_MODEL_H
