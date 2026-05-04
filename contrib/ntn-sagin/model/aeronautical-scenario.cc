/*
 * SPDX-License-Identifier: GPL-2.0-only
 * Copyright (c) 2026 Muhammad Uzair (ns3-ntn-toolkit, Workstream W5)
 */
#include "aeronautical-scenario.h"

#include "ns3/log.h"
#include "ns3/simulator.h"

#include <cmath>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("AeronauticalMobilityModel");
NS_OBJECT_ENSURE_REGISTERED(AeronauticalMobilityModel);

TypeId
AeronauticalMobilityModel::GetTypeId()
{
    static TypeId tid = TypeId("ns3::AeronauticalMobilityModel")
                           .SetParent<MobilityModel>()
                           .SetGroupName("NtnSagin")
                           .AddConstructor<AeronauticalMobilityModel>();
    return tid;
}

AeronauticalMobilityModel::AeronauticalMobilityModel() = default;

AeronauticalMobilityModel::~AeronauticalMobilityModel()
{
    if (m_tickEvent.IsPending())
    {
        Simulator::Cancel(m_tickEvent);
    }
}

void
AeronauticalMobilityModel::SetFlightPlan(const Vector& departure,
                                         const Vector& arrival,
                                         double cruiseAltitudeM,
                                         double cruiseSpeedMps)
{
    m_dep = Vector(departure.x, departure.y, cruiseAltitudeM);
    m_arr = Vector(arrival.x, arrival.y, cruiseAltitudeM);
    m_cruiseAltM = cruiseAltitudeM;
    m_speedMps = cruiseSpeedMps;
    Vector d(m_arr.x - m_dep.x, m_arr.y - m_dep.y, 0.0);
    m_legLengthM = std::sqrt(d.x * d.x + d.y * d.y);
    if (m_legLengthM < 1e-3)
    {
        m_legLengthM = 1e-3;
    }
    double scale = m_speedMps / m_legLengthM;
    m_velocity = Vector(d.x * scale, d.y * scale, 0.0);
    m_current = m_dep;
    m_progressM = 0.0;
    if (!m_tickEvent.IsPending())
    {
        m_tickEvent = Simulator::ScheduleNow(&AeronauticalMobilityModel::Tick, this);
    }
}

Vector
AeronauticalMobilityModel::DoGetPosition() const
{
    return m_current;
}

void
AeronauticalMobilityModel::DoSetPosition(const Vector& position)
{
    m_current = position;
    NotifyCourseChange();
}

Vector
AeronauticalMobilityModel::DoGetVelocity() const
{
    return m_velocity;
}

void
AeronauticalMobilityModel::Tick()
{
    double dt = m_tickInterval.GetSeconds();
    m_progressM += m_speedMps * dt;
    if (m_progressM >= m_legLengthM)
    {
        m_current = m_arr;
        m_velocity = Vector{};
        NotifyCourseChange();
        return; // arrived
    }
    double frac = m_progressM / m_legLengthM;
    m_current = Vector(m_dep.x + (m_arr.x - m_dep.x) * frac,
                       m_dep.y + (m_arr.y - m_dep.y) * frac,
                       m_cruiseAltM);
    NotifyCourseChange();
    m_tickEvent = Simulator::Schedule(m_tickInterval,
                                      &AeronauticalMobilityModel::Tick, this);
}

} // namespace ns3
