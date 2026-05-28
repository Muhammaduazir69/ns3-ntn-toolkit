/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#ifndef NTN_SAGIN_HAPS_TRAJECTORY_MOBILITY_MODEL_H
#define NTN_SAGIN_HAPS_TRAJECTORY_MOBILITY_MODEL_H

// HAPS replay mobility model (Roadmap §4.4.8).
//
// Reads a `HapsTrajectoryTrace` and exposes the interpolated position
// + velocity at every Simulator::Now() through ns-3's standard
// `MobilityModel` API. Coordinates are projected to local ENU around
// a user-supplied reference (lat_ref, lon_ref, alt_ref).

#include "haps-trajectory-trace.h"

#include "ns3/mobility-model.h"

namespace ns3
{
namespace sagin
{

class HapsTrajectoryMobilityModel : public MobilityModel
{
  public:
    static TypeId GetTypeId();
    HapsTrajectoryMobilityModel();
    ~HapsTrajectoryMobilityModel() override = default;

    void SetTrace(const HapsTrajectoryTrace& trace);
    void SetReference(double lat_deg, double lon_deg, double alt_m);
    void SetTraceTimeOffsetSeconds(double t_offset_s);

    HapsTrajectorySample CurrentSample() const;

  private:
    Vector DoGetPosition() const override;
    void DoSetPosition(const Vector& position) override;
    Vector DoGetVelocity() const override;

    HapsTrajectorySample SampleAtSimulatorNow() const;
    Vector ToEnu(const HapsTrajectorySample& s) const;

    HapsTrajectoryTrace m_trace;
    double m_latRefDeg{0.0};
    double m_lonRefDeg{0.0};
    double m_altRefM{0.0};
    double m_traceTimeOffset_s{0.0};
};

} // namespace sagin
} // namespace ns3

#endif // NTN_SAGIN_HAPS_TRAJECTORY_MOBILITY_MODEL_H
