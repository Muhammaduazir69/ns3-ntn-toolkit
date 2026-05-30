/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#ifndef NTN_CONSTELLATION_SGP4_MOBILITY_MODEL_H
#define NTN_CONSTELLATION_SGP4_MOBILITY_MODEL_H

// SGP4 / Kepler+J2 mobility model (Roadmap §3 T5).
//
// The v2.1 baseline uses an analytic Kepler propagator with secular J2
// corrections to RAAN and argument-of-perigee. This is accurate to within
// a few km over hours of propagation for the typical near-circular LEO
// constellations this module targets (Walker-Delta Starlink shell 1,
// OneWeb, Kuiper). Full Vallado SGP4 is a Q4 2026 T5 follow-on; the
// classical-element interface is identical so the backend swap is
// transparent to consumers.
//
// Position is cached at 100 ms grain per roadmap §3 T5 to avoid
// re-evaluating Kepler's equation on every DoGetPosition() call from
// densely scheduled events.
//
// Two coordinate-system surfaces:
//   GetPosition() / DoGetPosition() returns ECEF metres (rotates with
//     Earth at omega = 7.2921159e-5 rad/s). This is what ns-3 mobility
//     consumers (link budgets, mobility-aware xApps) expect.
//   GetEciPosition() returns the inertial ECI position for ground-track
//     tooling.

#include "orbital-elements.h"

#include "ns3/mobility-model.h"
#include "ns3/nstime.h"

namespace ns3
{
namespace ntncon
{

class Sgp4MobilityModel : public MobilityModel
{
  public:
    static TypeId GetTypeId();
    Sgp4MobilityModel();
    ~Sgp4MobilityModel() override = default;

    /// Install classical elements directly.
    void SetElements(const KeplerianElements& elements);

    /// Convenience: parse a TLE record and install it.
    /// Returns false on parse failure.
    bool SetTle(const TleRecord& tle);

    /// Read-only access to the installed elements.
    const KeplerianElements& GetElements() const { return m_elements; }

    /// Cache granularity for the position lookup. Default 100 ms.
    void SetCacheGranularity(Time grain) { m_cacheGrain = grain; }

    /// Inertial-frame position (ECI metres) at sim time.
    Vector GetEciPosition() const;

    /// Earth-fixed position (ECEF metres) at sim time — what ns-3 link
    /// budget consumers expect.
    Vector GetEcefPosition() const;

    /// ECEF velocity (m/s).
    Vector GetEcefVelocity() const;

    /// Geodetic latitude / longitude / altitude (deg, deg, m above WGS-84
    /// reference ellipsoid).
    void GetGeodetic(double& lat_deg, double& lon_deg, double& alt_m) const;

    /// Elevation angle (deg) from a ground station at (lat_deg, lon_deg)
    /// at the current sim time. Returns a value < 0 when the satellite is
    /// below the horizon.
    double GetElevationDeg(double gs_lat_deg, double gs_lon_deg) const;

  private:
    Vector DoGetPosition() const override;
    void DoSetPosition(const Vector& position) override;
    Vector DoGetVelocity() const override;

    /// Propagate to absolute time `unix_s` and fill ECI position/velocity.
    void Propagate(double unix_s, Vector& pos_eci, Vector& vel_eci) const;

    /// Convert ECI to ECEF for the given absolute time.
    static Vector EciToEcef(const Vector& eci, double unix_s);

    /// Cache lookup keyed on the rounded-down sim time.
    void EnsureCache() const;

    KeplerianElements m_elements;
    Time m_cacheGrain{MilliSeconds(100)};

    // Cache.
    mutable bool m_cacheValid{false};
    mutable Time m_cacheTime;
    mutable Vector m_cachedEci;
    mutable Vector m_cachedEciVel;
};

} // namespace ntncon
} // namespace ns3

#endif // NTN_CONSTELLATION_SGP4_MOBILITY_MODEL_H
