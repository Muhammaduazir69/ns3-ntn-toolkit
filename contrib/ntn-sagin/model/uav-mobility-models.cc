/*
 * SPDX-License-Identifier: GPL-2.0-only
 * Copyright (c) 2026 Muhammad Uzair (ns3-ntn-toolkit, Workstream W5)
 */
#include "uav-mobility-models.h"

#include "ns3/box.h"
#include "ns3/double.h"
#include "ns3/log.h"
#include "ns3/simulator.h"
#include "ns3/uinteger.h"

#include <cmath>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("UavMobilityModels");

// ============================================================================
// UavWaypointMobilityModel
// ============================================================================

NS_OBJECT_ENSURE_REGISTERED(UavWaypointMobilityModel);

TypeId
UavWaypointMobilityModel::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::UavWaypointMobilityModel")
            .SetParent<MobilityModel>()
            .SetGroupName("NtnSagin")
            .AddConstructor<UavWaypointMobilityModel>()
            .AddAttribute("Speed",
                          "UAV airspeed (m/s); typical 20 m/s for small fixed-wing",
                          DoubleValue(20.0),
                          MakeDoubleAccessor(&UavWaypointMobilityModel::m_speedMps),
                          MakeDoubleChecker<double>(0.5, 100.0))
            .AddAttribute("PauseTime",
                          "Hover duration on reaching a waypoint",
                          TimeValue(Seconds(2.0)),
                          MakeTimeAccessor(&UavWaypointMobilityModel::m_pause),
                          MakeTimeChecker())
            .AddAttribute("TickInterval",
                          "Internal position-update period",
                          TimeValue(MilliSeconds(500)),
                          MakeTimeAccessor(&UavWaypointMobilityModel::m_tickInterval),
                          MakeTimeChecker(MilliSeconds(10)));
    return tid;
}

UavWaypointMobilityModel::UavWaypointMobilityModel()
    : m_rand(CreateObject<UniformRandomVariable>())
{
}

UavWaypointMobilityModel::~UavWaypointMobilityModel()
{
    if (m_tickEvent.IsPending())
    {
        Simulator::Cancel(m_tickEvent);
    }
}

void
UavWaypointMobilityModel::SetBox(const Box& box)
{
    m_box = box;
    if (m_position == Vector{})
    {
        m_position = Vector((box.xMin + box.xMax) / 2,
                            (box.yMin + box.yMax) / 2,
                            (box.zMin + box.zMax) / 2);
    }
    NewLeg();
    if (!m_tickEvent.IsPending())
    {
        m_tickEvent = Simulator::ScheduleNow(&UavWaypointMobilityModel::Tick, this);
    }
}

void
UavWaypointMobilityModel::NewLeg()
{
    m_target = Vector(m_rand->GetValue(m_box.xMin, m_box.xMax),
                      m_rand->GetValue(m_box.yMin, m_box.yMax),
                      m_rand->GetValue(m_box.zMin, m_box.zMax));
    Vector d(m_target.x - m_position.x, m_target.y - m_position.y,
             m_target.z - m_position.z);
    double len = std::sqrt(d.x * d.x + d.y * d.y + d.z * d.z);
    if (len < 1e-3)
    {
        m_velocity = Vector{};
        return;
    }
    double scale = m_speedMps / len;
    m_velocity = Vector(d.x * scale, d.y * scale, d.z * scale);
    m_paused = false;
}

Vector
UavWaypointMobilityModel::DoGetPosition() const
{
    return m_position;
}

void
UavWaypointMobilityModel::DoSetPosition(const Vector& position)
{
    m_position = position;
    NotifyCourseChange();
}

Vector
UavWaypointMobilityModel::DoGetVelocity() const
{
    return m_paused ? Vector{} : m_velocity;
}

void
UavWaypointMobilityModel::Tick()
{
    Time now = Simulator::Now();
    double dt = m_tickInterval.GetSeconds();
    if (m_paused)
    {
        if (now >= m_pauseUntil)
        {
            NewLeg();
        }
    }
    else
    {
        m_position.x += m_velocity.x * dt;
        m_position.y += m_velocity.y * dt;
        m_position.z += m_velocity.z * dt;
        Vector d(m_target.x - m_position.x, m_target.y - m_position.y,
                 m_target.z - m_position.z);
        double remaining = std::sqrt(d.x * d.x + d.y * d.y + d.z * d.z);
        if (remaining < m_speedMps * dt)
        {
            m_position = m_target;
            m_paused = true;
            m_pauseUntil = now + m_pause;
            m_velocity = Vector{};
        }
        NotifyCourseChange();
    }
    m_tickEvent = Simulator::Schedule(m_tickInterval,
                                      &UavWaypointMobilityModel::Tick, this);
}

// ============================================================================
// UavPatrolMobilityModel
// ============================================================================

NS_OBJECT_ENSURE_REGISTERED(UavPatrolMobilityModel);

TypeId
UavPatrolMobilityModel::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::UavPatrolMobilityModel")
            .SetParent<MobilityModel>()
            .SetGroupName("NtnSagin")
            .AddConstructor<UavPatrolMobilityModel>()
            .AddAttribute("Speed",
                          "Ground speed (m/s)",
                          DoubleValue(15.0),
                          MakeDoubleAccessor(&UavPatrolMobilityModel::m_speedMps),
                          MakeDoubleChecker<double>(0.1, 100.0))
            .AddAttribute("PauseTime",
                          "Hover at endpoints",
                          TimeValue(Seconds(1.0)),
                          MakeTimeAccessor(&UavPatrolMobilityModel::m_pause),
                          MakeTimeChecker())
            .AddAttribute("TickInterval",
                          "Internal update period",
                          TimeValue(MilliSeconds(200)),
                          MakeTimeAccessor(&UavPatrolMobilityModel::m_tickInterval),
                          MakeTimeChecker(MilliSeconds(10)));
    return tid;
}

UavPatrolMobilityModel::UavPatrolMobilityModel() = default;

UavPatrolMobilityModel::~UavPatrolMobilityModel()
{
    if (m_tickEvent.IsPending())
    {
        Simulator::Cancel(m_tickEvent);
    }
}

void
UavPatrolMobilityModel::SetEndpoints(const Vector& a, const Vector& b)
{
    m_a = a;
    m_b = b;
    m_current = a;
    m_forward = true;
    Vector d(b.x - a.x, b.y - a.y, b.z - a.z);
    double len = std::sqrt(d.x * d.x + d.y * d.y + d.z * d.z);
    double scale = (len > 1e-3) ? m_speedMps / len : 0.0;
    m_velocity = Vector(d.x * scale, d.y * scale, d.z * scale);
    if (!m_tickEvent.IsPending())
    {
        m_tickEvent = Simulator::ScheduleNow(&UavPatrolMobilityModel::Tick, this);
    }
}

double
UavPatrolMobilityModel::GetLegProgressM() const
{
    Vector& start = const_cast<Vector&>(m_forward ? m_a : m_b);
    Vector d(m_current.x - start.x, m_current.y - start.y, m_current.z - start.z);
    return std::sqrt(d.x * d.x + d.y * d.y + d.z * d.z);
}

Vector
UavPatrolMobilityModel::DoGetPosition() const
{
    return m_current;
}

void
UavPatrolMobilityModel::DoSetPosition(const Vector& position)
{
    m_current = position;
    NotifyCourseChange();
}

Vector
UavPatrolMobilityModel::DoGetVelocity() const
{
    return m_paused ? Vector{} : m_velocity;
}

void
UavPatrolMobilityModel::Tick()
{
    Time now = Simulator::Now();
    double dt = m_tickInterval.GetSeconds();
    Vector goal = m_forward ? m_b : m_a;

    if (m_paused)
    {
        if (now >= m_pauseUntil)
        {
            m_paused = false;
            m_forward = !m_forward;
            Vector start = m_forward ? m_a : m_b;
            Vector end = m_forward ? m_b : m_a;
            m_current = start;
            Vector d(end.x - start.x, end.y - start.y, end.z - start.z);
            double len = std::sqrt(d.x * d.x + d.y * d.y + d.z * d.z);
            double scale = (len > 1e-3) ? m_speedMps / len : 0.0;
            m_velocity = Vector(d.x * scale, d.y * scale, d.z * scale);
        }
    }
    else
    {
        m_current.x += m_velocity.x * dt;
        m_current.y += m_velocity.y * dt;
        m_current.z += m_velocity.z * dt;
        Vector d(goal.x - m_current.x, goal.y - m_current.y, goal.z - m_current.z);
        double remaining = std::sqrt(d.x * d.x + d.y * d.y + d.z * d.z);
        if (remaining < m_speedMps * dt)
        {
            m_current = goal;
            m_paused = true;
            m_pauseUntil = now + m_pause;
            m_velocity = Vector{};
        }
        NotifyCourseChange();
    }
    m_tickEvent = Simulator::Schedule(m_tickInterval,
                                      &UavPatrolMobilityModel::Tick, this);
}

// ============================================================================
// UavSearchPatternMobilityModel
// ============================================================================

NS_OBJECT_ENSURE_REGISTERED(UavSearchPatternMobilityModel);

TypeId
UavSearchPatternMobilityModel::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::UavSearchPatternMobilityModel")
            .SetParent<MobilityModel>()
            .SetGroupName("NtnSagin")
            .AddConstructor<UavSearchPatternMobilityModel>()
            .AddAttribute("Speed",
                          "Sweep speed (m/s)",
                          DoubleValue(12.0),
                          MakeDoubleAccessor(&UavSearchPatternMobilityModel::m_speedMps),
                          MakeDoubleChecker<double>(0.1, 100.0))
            .AddAttribute("TickInterval",
                          "Internal update period",
                          TimeValue(MilliSeconds(200)),
                          MakeTimeAccessor(&UavSearchPatternMobilityModel::m_tickInterval),
                          MakeTimeChecker(MilliSeconds(10)));
    return tid;
}

UavSearchPatternMobilityModel::UavSearchPatternMobilityModel() = default;

UavSearchPatternMobilityModel::~UavSearchPatternMobilityModel()
{
    if (m_tickEvent.IsPending())
    {
        Simulator::Cancel(m_tickEvent);
    }
}

void
UavSearchPatternMobilityModel::SetSearchArea(const Vector& origin, double legLengthM,
                                             double stripWidthM, int nStrips,
                                             double altitudeM)
{
    m_origin = origin;
    m_legLengthM = legLengthM;
    m_stripWidthM = stripWidthM;
    m_nStrips = std::max(1, nStrips);
    m_altitudeM = altitudeM;

    m_waypoints.clear();
    m_waypoints.reserve(static_cast<std::size_t>(2 * m_nStrips));
    bool eastbound = true;
    for (int i = 0; i < m_nStrips; ++i)
    {
        double y = origin.y + i * stripWidthM;
        if (eastbound)
        {
            m_waypoints.emplace_back(origin.x, y, altitudeM);
            m_waypoints.emplace_back(origin.x + legLengthM, y, altitudeM);
        }
        else
        {
            m_waypoints.emplace_back(origin.x + legLengthM, y, altitudeM);
            m_waypoints.emplace_back(origin.x, y, altitudeM);
        }
        eastbound = !eastbound;
    }

    m_current = m_waypoints.front();
    m_wpIndex = 1;
    AdvanceWaypoint();
    if (!m_tickEvent.IsPending())
    {
        m_tickEvent = Simulator::ScheduleNow(&UavSearchPatternMobilityModel::Tick, this);
    }
}

void
UavSearchPatternMobilityModel::AdvanceWaypoint()
{
    if (m_wpIndex >= m_waypoints.size())
    {
        m_velocity = Vector{};
        return;
    }
    Vector tgt = m_waypoints[m_wpIndex];
    Vector d(tgt.x - m_current.x, tgt.y - m_current.y, tgt.z - m_current.z);
    double len = std::sqrt(d.x * d.x + d.y * d.y + d.z * d.z);
    double scale = (len > 1e-3) ? m_speedMps / len : 0.0;
    m_velocity = Vector(d.x * scale, d.y * scale, d.z * scale);
}

Vector
UavSearchPatternMobilityModel::DoGetPosition() const
{
    return m_current;
}

void
UavSearchPatternMobilityModel::DoSetPosition(const Vector& position)
{
    m_current = position;
    NotifyCourseChange();
}

Vector
UavSearchPatternMobilityModel::DoGetVelocity() const
{
    return m_velocity;
}

void
UavSearchPatternMobilityModel::Tick()
{
    if (m_wpIndex >= m_waypoints.size())
    {
        m_velocity = Vector{};
        return;  // pattern complete
    }
    double dt = m_tickInterval.GetSeconds();
    m_current.x += m_velocity.x * dt;
    m_current.y += m_velocity.y * dt;
    m_current.z += m_velocity.z * dt;
    Vector tgt = m_waypoints[m_wpIndex];
    Vector d(tgt.x - m_current.x, tgt.y - m_current.y, tgt.z - m_current.z);
    double remaining = std::sqrt(d.x * d.x + d.y * d.y + d.z * d.z);
    if (remaining < m_speedMps * dt)
    {
        m_current = tgt;
        m_wpIndex += 1;
        AdvanceWaypoint();
    }
    NotifyCourseChange();
    m_tickEvent = Simulator::Schedule(m_tickInterval,
                                      &UavSearchPatternMobilityModel::Tick, this);
}

} // namespace ns3
