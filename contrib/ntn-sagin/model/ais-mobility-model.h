/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#ifndef NTN_SAGIN_AIS_MOBILITY_MODEL_H
#define NTN_SAGIN_AIS_MOBILITY_MODEL_H

// Mobility model replaying an AIS maritime trace (Roadmap §4.4.2).
//
// Same ENU projection as OpenSkyMobilityModel; altitude clamped to 0 m
// (sea level). Velocity is derived from SOG (Speed Over Ground in knots
// -> m/s) and COG (Course Over Ground, 0=N going clockwise).
//
// Use case: drive a maritime UE through a NTN beam footprint along a
// real-world AIS-recorded route (Danish Maritime / Piraeus port).

#include "ais-maritime-trace.h"

#include "ns3/mobility-model.h"

namespace ns3
{
namespace sagin
{

class AisMobilityModel : public MobilityModel
{
  public:
    static constexpr double kKnotsToMps = 0.5144444;

    static TypeId GetTypeId();
    AisMobilityModel();
    ~AisMobilityModel() override = default;

    void SetTrace(const AisMaritimeTrace& trace);
    void SetReference(double lat_deg, double lon_deg);
    void SetTraceTimeOffsetSeconds(double t_offset_s);

    AisSample CurrentSample() const;

  private:
    Vector DoGetPosition() const override;
    void DoSetPosition(const Vector& position) override;
    Vector DoGetVelocity() const override;

    AisSample SampleAtSimulatorNow() const;
    Vector ToEnu(const AisSample& s) const;

    AisMaritimeTrace m_trace;
    double m_latRefDeg{0.0};
    double m_lonRefDeg{0.0};
    double m_traceTimeOffset_s{0.0};
};

} // namespace sagin
} // namespace ns3

#endif // NTN_SAGIN_AIS_MOBILITY_MODEL_H
