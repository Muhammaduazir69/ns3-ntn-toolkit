/*
 * SPDX-License-Identifier: GPL-2.0-only
 * Copyright (c) 2026 Muhammad Uzair (ns3-ntn-toolkit, Workstream W7)
 */
#include "maritime-scenario.h"

#include "ns3/double.h"
#include "ns3/log.h"
#include "ns3/simulator.h"

#include <algorithm>
#include <cmath>

namespace ns3
{
namespace ntnv2x
{

NS_LOG_COMPONENT_DEFINE("MaritimeMobilityModel");
NS_OBJECT_ENSURE_REGISTERED(MaritimeMobilityModel);

TypeId
MaritimeMobilityModel::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::ntnv2x::MaritimeMobilityModel")
            .SetParent<MobilityModel>()
            .SetGroupName("NtnV2x")
            .AddConstructor<MaritimeMobilityModel>()
            .AddAttribute("Speed",
                          "Vessel speed (m/s); merchant 5-13 m/s",
                          DoubleValue(8.0),
                          MakeDoubleAccessor(&MaritimeMobilityModel::m_speedMps),
                          MakeDoubleChecker<double>(0.5, 25.0))
            .AddAttribute("TickInterval",
                          "Position update period",
                          TimeValue(Seconds(1.0)),
                          MakeTimeAccessor(&MaritimeMobilityModel::m_tickInterval),
                          MakeTimeChecker(MilliSeconds(10)));
    return tid;
}

MaritimeMobilityModel::MaritimeMobilityModel()
    : m_rand(CreateObject<UniformRandomVariable>())
{
}

MaritimeMobilityModel::~MaritimeMobilityModel()
{
    if (m_tickEvent.IsPending())
    {
        Simulator::Cancel(m_tickEvent);
    }
}

void
MaritimeMobilityModel::SetSeaArea(const Box& area)
{
    m_area = area;
    m_position = Vector(m_rand->GetValue(area.xMin, area.xMax),
                        m_rand->GetValue(area.yMin, area.yMax),
                        0.0);
    double heading = m_rand->GetValue(0.0, 2.0 * M_PI);
    m_velocity = Vector(m_speedMps * std::cos(heading),
                        m_speedMps * std::sin(heading), 0.0);
    if (!m_tickEvent.IsPending())
    {
        m_tickEvent = Simulator::ScheduleNow(&MaritimeMobilityModel::Tick, this);
    }
}

Vector
MaritimeMobilityModel::DoGetPosition() const
{
    return m_position;
}

void
MaritimeMobilityModel::DoSetPosition(const Vector& position)
{
    m_position = position;
    NotifyCourseChange();
}

Vector
MaritimeMobilityModel::DoGetVelocity() const
{
    return m_velocity;
}

void
MaritimeMobilityModel::Tick()
{
    double dt = m_tickInterval.GetSeconds();
    m_position.x += m_velocity.x * dt;
    m_position.y += m_velocity.y * dt;
    if (m_position.x < m_area.xMin || m_position.x > m_area.xMax)
    {
        m_velocity.x = -m_velocity.x;
        m_position.x = std::clamp(m_position.x, m_area.xMin, m_area.xMax);
    }
    if (m_position.y < m_area.yMin || m_position.y > m_area.yMax)
    {
        m_velocity.y = -m_velocity.y;
        m_position.y = std::clamp(m_position.y, m_area.yMin, m_area.yMax);
    }
    NotifyCourseChange();
    m_tickEvent = Simulator::Schedule(m_tickInterval,
                                      &MaritimeMobilityModel::Tick, this);
}

} // namespace ntnv2x
} // namespace ns3
