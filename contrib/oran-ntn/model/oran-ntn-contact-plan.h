/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only
//
// OranNtnContactPlan: an ephemeris-driven feeder contact plan.
//
// For every satellite that may host or serve an E2 node, the plan holds the
// time windows in which the satellite sees at least one gateway above that
// gateway's elevation mask. Windows are found by stepping a caller-supplied
// ECEF position function (in practice the Sgp4MobilityModel of the satellite,
// optionally perturbed to model ephemeris error) and are refined at each edge
// by bisection, so an edge is accurate to the requested resolution and not to
// the coarse step.
//
// The plan is what a management function can know IN ADVANCE about feeder
// availability. It knows nothing about unpredicted outages (rain fade, gateway
// faults); a consumer must still detect those at run time.

#ifndef ORAN_NTN_CONTACT_PLAN_H
#define ORAN_NTN_CONTACT_PLAN_H

#include <ns3/nstime.h>
#include <ns3/object.h>
#include <ns3/vector.h>

#include <cstdint>
#include <functional>
#include <map>
#include <vector>

namespace ns3
{

class OranNtnContactPlan : public Object
{
  public:
    /// One feeder contact: [start, end) in simulation time.
    struct Window
    {
        Time start;
        Time end;
    };

    /// Absolute simulation time -> ECEF position in metres.
    using EcefFn = std::function<Vector(Time)>;

    static TypeId GetTypeId();
    OranNtnContactPlan() = default;

    void AddGateway(uint32_t gwId, double latDeg, double lonDeg, double altM, double minElevDeg);
    void AddSatellite(uint32_t satId, EcefFn ecef);

    /// Compute the windows of every satellite over [from, to]. \p step is the
    /// coarse scan step, \p resolution the edge accuracy after bisection.
    void Build(Time from, Time to, Time step, Time resolution);

    /// Merged feeder windows of \p satId (union over all gateways).
    const std::vector<Window>& GetWindows(uint32_t satId) const;
    /// Windows of one (satellite, gateway) pair.
    const std::vector<Window>& GetPairWindows(uint32_t satId, uint32_t gwId) const;

    /// True when the plan says \p satId sees some gateway at \p t.
    bool IsVisible(uint32_t satId, Time t) const;
    /// First predicted loss of all feeders at or after \p t (Time::Max() if the
    /// satellite stays connected to the horizon, \p t if it is not connected).
    Time NextLoss(uint32_t satId, Time t) const;
    /// First predicted regain of a feeder at or after \p t (\p t if connected
    /// now, Time::Max() if never within the horizon).
    Time NextGain(uint32_t satId, Time t) const;

    /// Fraction of [from, to] covered by the satellite's merged windows.
    double Availability(uint32_t satId, Time from, Time to) const;

    /// Elevation (deg) of a satellite at ECEF \p sat seen from ECEF \p gs.
    static double ElevationDeg(const Vector& sat, const Vector& gs);
    /// WGS-84 geodetic to ECEF (metres).
    static Vector GeodeticToEcef(double latDeg, double lonDeg, double altM);

    uint32_t GetGatewayCount() const { return static_cast<uint32_t>(m_gw.size()); }
    Vector GetGatewayEcef(uint32_t gwId) const;
    double GetGatewayMaskDeg(uint32_t gwId) const;

  private:
    struct Gateway
    {
        uint32_t id;
        Vector ecef;
        double maskDeg;
    };

    bool SeesGateway(const EcefFn& f, const Gateway& g, Time t) const;
    std::vector<Window> ScanPair(const EcefFn& f,
                                 const Gateway& g,
                                 Time from,
                                 Time to,
                                 Time step,
                                 Time res) const;
    static std::vector<Window> Union(std::vector<Window> w);

    std::vector<Gateway> m_gw;
    std::map<uint32_t, EcefFn> m_sat;
    std::map<uint32_t, std::vector<Window>> m_merged;
    std::map<std::pair<uint32_t, uint32_t>, std::vector<Window>> m_pair;
    std::vector<Window> m_empty;
};

} // namespace ns3

#endif // ORAN_NTN_CONTACT_PLAN_H
