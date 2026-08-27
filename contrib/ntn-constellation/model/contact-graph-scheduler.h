/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#ifndef NTN_CONSTELLATION_CONTACT_GRAPH_SCHEDULER_H
#define NTN_CONSTELLATION_CONTACT_GRAPH_SCHEDULER_H

// Contact-graph scheduler (Roadmap §3 T5).
//
// Periodically samples link geometry between registered satellites and
// ground stations and emits up / down events when a link's visibility
// state changes:
//   GSL (Ground-Satellite Link): comes up when sat elevation reaches
//                                MinElevationDeg from the ground station's
//                                local horizon, and goes down only when it
//                                falls below MinElevationDeg -
//                                GateHysteresisDeg (default 2 deg) so the
//                                contact does not flap while the elevation
//                                hovers at the threshold.
//   ISL (Inter-Satellite Link):  up while the Euclidean distance is
//                                <= MaxIslRangeM (default 5000 km, the
//                                roadmap's LEO-LEO cap).
//
// Consumers subscribe to the `m_contactUp` / `m_contactDown` traces and
// react (route changes, packet drops, beam reconfig). The scheduler does
// not modify ns3::Channel state itself — that's the consumer's job, kept
// orthogonal so different xApps can react differently.

#include "sgp4-mobility-model.h"

#include <ns3/event-id.h>
#include <ns3/nstime.h>
#include <ns3/object.h>
#include <ns3/traced-callback.h>

#include <map>
#include <utility>
#include <vector>

namespace ns3
{
namespace ntncon
{

struct ContactEvent
{
    double timestamp_s;
    uint32_t node_a;   //!< first endpoint ID
    uint32_t node_b;   //!< second endpoint ID (0 reserved for GS-side)
    bool is_isl;       //!< true = ISL, false = GSL
    bool up;           //!< true = contact just became visible, false = lost
    double range_m;    //!< Euclidean range at the event time
    double elevation_deg; //!< GSL only; meaningless for ISL
};

class ContactGraphScheduler : public Object
{
  public:
    static TypeId GetTypeId();
    ContactGraphScheduler();
    ~ContactGraphScheduler() override = default;

    /// Sampling interval for visibility checks. Default 1 s.
    void SetSamplingInterval(Time dt) { m_dt = dt; }

    /// Per-link visibility gates.
    void SetMinElevationDeg(double e) { m_minElevDeg = e; }
    void SetMaxIslRangeM(double r) { m_maxIslRangeM = r; }
    double GetMinElevationDeg() const { return m_minElevDeg; }
    double GetMaxIslRangeM() const { return m_maxIslRangeM; }
    void SetIslMinTangentAltM(double m) { m_islMinTangentAltM = m; }
    double GetIslMinTangentAltM() const { return m_islMinTangentAltM; }

    /**
     * \brief CON-3: is the straight segment a-b clear of the atmospheric limb?
     *
     * True when the segment's closest approach to geocentre clears the
     * ellipsoid by at least \p minTangentAltM. Extracted from the contact loop
     * so the predicate can be exercised at a chosen tangent height directly:
     * inline in the loop it was only reachable through SGP4 ephemeris, which is
     * why a grazing acceptance went unnoticed.
     *
     * Endpoints are satellites, above the surface, so a blocked link occurs
     * only when the closest point lies strictly between them.
     */
    static bool IsLimbClear(const Vector& a, const Vector& b, double minTangentAltM);

    /// GSL gate hysteresis (deg): a contact comes UP at MinElevationDeg and
    /// goes DOWN at MinElevationDeg - hysteresis. Default 2.0 (also exposed
    /// as the `GateHysteresisDeg` attribute). Set 0 for the legacy
    /// single-threshold gate.
    void SetGateHysteresisDeg(double h) { m_gateHysteresisDeg = h; }
    double GetGateHysteresisDeg() const { return m_gateHysteresisDeg; }

    /// Register a satellite. `id` must be unique. The scheduler retains
    /// the pointer; the caller owns the object.
    void RegisterSatellite(uint32_t id, Ptr<Sgp4MobilityModel> sat);

    /// Register a ground station at (lat, lon, alt=0 m MSL).
    void RegisterGroundStation(uint32_t id, double lat_deg, double lon_deg);

    /// Start the periodic visibility sampler. Schedules through the
    /// Simulator::Schedule path; the user only needs to call Stop() when
    /// the scenario is over (the scheduler itself does not stop the
    /// simulator).
    void Start();
    void Stop();

    /// Read-only counters (test asserts).
    uint64_t GslEventsUp() const { return m_gslUp; }
    uint64_t GslEventsDown() const { return m_gslDown; }
    uint64_t IslEventsUp() const { return m_islUp; }
    uint64_t IslEventsDown() const { return m_islDown; }
    size_t NumActiveGsl() const;
    size_t NumActiveIsl() const;

    /// Traced callbacks fired on every contact transition.
    TracedCallback<ContactEvent> m_contactUp;
    TracedCallback<ContactEvent> m_contactDown;
    /// SAGIN-1: fires once per tick for every contact that is already up,
    /// carrying the CURRENT range and elevation.
    ///
    /// m_contactUp and m_contactDown fire on visibility TRANSITIONS only, so a
    /// consumer that took its geometry from the up event kept that value for
    /// the whole pass. At 550 km with a 20 degree elevation floor the GSL slant
    /// sweeps from about 550 km at zenith to about 1075 km at the floor, so a
    /// pinned delay is wrong by up to ~1.8 ms one way and a link budget
    /// evaluated once at zenith never degrades on the way down.
    ///
    /// Consumers that only care about topology can keep ignoring this; those
    /// that model delay, link budget or path cost should subscribe to it in
    /// addition to the transition traces.
    TracedCallback<ContactEvent> m_contactUpdate;

  private:
    void Tick();

    struct GsRecord
    {
        double lat_deg;
        double lon_deg;
    };

    Time m_dt{Seconds(1.0)};
    double m_minElevDeg{25.0};
    double m_gateHysteresisDeg{2.0};
    double m_maxIslRangeM{5'000'000.0};
    /// CON-3: minimum clearance above the ellipsoid an ISL must keep, in metres.
    ///
    /// The limb test accepted a link whose closest approach was exactly one
    /// Earth radius, i.e. a ray grazing the surface. A grazing path is not a
    /// usable optical or RF crosslink: it crosses the full depth of the
    /// atmosphere twice, with refraction, absorption and scintillation that
    /// rise without bound as the tangent height falls to zero. Real
    /// constellations budget a tangent-height margin and drop the link below it.
    ///
    /// 80 km is the default, which clears the mesosphere and is the usual
    /// engineering floor for treating a crosslink as effectively in vacuum.
    double m_islMinTangentAltM{80.0e3};

    std::map<uint32_t, Ptr<Sgp4MobilityModel>> m_sats;
    std::map<uint32_t, GsRecord> m_gss;

    /// Visibility state: keyed by (sat_id, gs_id) for GSL or (sat_a, sat_b)
    /// for ISL (with sat_a < sat_b).
    std::map<std::pair<uint32_t, uint32_t>, bool> m_gslState;
    std::map<std::pair<uint32_t, uint32_t>, bool> m_islState;

    uint64_t m_gslUp{0};
    uint64_t m_gslDown{0};
    uint64_t m_islUp{0};
    uint64_t m_islDown{0};

    bool m_running{false};
    EventId m_tickEvent;
};

} // namespace ntncon
} // namespace ns3

#endif // NTN_CONSTELLATION_CONTACT_GRAPH_SCHEDULER_H
