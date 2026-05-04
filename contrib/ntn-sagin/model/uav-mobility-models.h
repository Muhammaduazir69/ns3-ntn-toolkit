/*
 * SPDX-License-Identifier: GPL-2.0-only
 * Copyright (c) 2026 Muhammad Uzair (ns3-ntn-toolkit, Workstream W5)
 */
#ifndef NTN_SAGIN_UAV_MOBILITY_H
#define NTN_SAGIN_UAV_MOBILITY_H

#include "ns3/box.h"
#include "ns3/mobility-model.h"
#include "ns3/nstime.h"
#include "ns3/random-variable-stream.h"
#include "ns3/vector.h"

#include <vector>

namespace ns3
{

/**
 * @brief Random-waypoint UAV mobility, bounded inside a 3D box.
 *
 * Picks a uniformly-random destination inside @c Box, flies straight to it
 * at @c Speed, pauses for @c PauseTime, then repeats. Altitude is sampled
 * inside the box's vertical range so a single instance can cover both
 * low-altitude (50 m) and 3GPP TR 36.777 ceiling (300 m) UAVs.
 */
class UavWaypointMobilityModel : public MobilityModel
{
  public:
    static TypeId GetTypeId();
    UavWaypointMobilityModel();
    ~UavWaypointMobilityModel() override;

    void SetBox(const Box& box);

  private:
    Vector DoGetPosition() const override;
    void DoSetPosition(const Vector& position) override;
    Vector DoGetVelocity() const override;

    void NewLeg();
    void Tick();

    Box m_box{-1000.0, 1000.0, -1000.0, 1000.0, 50.0, 200.0};
    double m_speedMps{20.0};
    Time m_pause{Seconds(2.0)};
    Time m_tickInterval{MilliSeconds(500)};

    Vector m_position;
    Vector m_target;
    Vector m_velocity;
    bool m_paused{false};
    Time m_pauseUntil;

    Ptr<UniformRandomVariable> m_rand;
    EventId m_tickEvent;
};

/**
 * @brief Round-trip patrol between two waypoints (e.g. perimeter watch).
 *
 * Flies from @c PointA to @c PointB at @c Speed, pauses, flies back, repeat.
 * Deterministic — no RNG state. Useful as a baseline against
 * `UavWaypointMobilityModel`.
 */
class UavPatrolMobilityModel : public MobilityModel
{
  public:
    static TypeId GetTypeId();
    UavPatrolMobilityModel();
    ~UavPatrolMobilityModel() override;

    void SetEndpoints(const Vector& a, const Vector& b);
    /// Distance (m) the UAV has flown along its current leg, useful for tests.
    double GetLegProgressM() const;

  private:
    Vector DoGetPosition() const override;
    void DoSetPosition(const Vector& position) override;
    Vector DoGetVelocity() const override;

    void Tick();

    Vector m_a;
    Vector m_b;
    double m_speedMps{15.0};
    Time m_pause{Seconds(1.0)};
    Time m_tickInterval{MilliSeconds(200)};

    Vector m_current;
    Vector m_velocity;
    bool m_forward{true};
    bool m_paused{false};
    Time m_pauseUntil;
    EventId m_tickEvent;
};

/**
 * @brief Lawnmower / boustrophedon search-pattern mobility (S-curve).
 *
 * Sweeps a rectangular search area at constant altitude, flying straight
 * legs of length @c LegLength along the X axis, then advancing @c StripWidth
 * in Y, alternating direction. Used for ISR / search-and-rescue.
 */
class UavSearchPatternMobilityModel : public MobilityModel
{
  public:
    static TypeId GetTypeId();
    UavSearchPatternMobilityModel();
    ~UavSearchPatternMobilityModel() override;

    void SetSearchArea(const Vector& origin, double legLengthM, double stripWidthM,
                       int nStrips, double altitudeM);

  private:
    Vector DoGetPosition() const override;
    void DoSetPosition(const Vector& position) override;
    Vector DoGetVelocity() const override;

    void Tick();
    void AdvanceWaypoint();

    Vector m_origin;
    double m_legLengthM{500.0};
    double m_stripWidthM{50.0};
    int m_nStrips{6};
    double m_altitudeM{100.0};
    double m_speedMps{12.0};
    Time m_tickInterval{MilliSeconds(200)};

    std::vector<Vector> m_waypoints;
    std::size_t m_wpIndex{0};
    Vector m_current;
    Vector m_velocity;
    EventId m_tickEvent;
};

} // namespace ns3

#endif // NTN_SAGIN_UAV_MOBILITY_H
