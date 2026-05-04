/*
 * SPDX-License-Identifier: GPL-2.0-only
 * Copyright (c) 2026 Muhammad Uzair (ns3-ntn-toolkit, Workstream W5)
 */
#include "haps-mobility-model.h"

#include "ns3/double.h"
#include "ns3/log.h"
#include "ns3/simulator.h"

#include <cmath>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("HapsMobilityModel");
NS_OBJECT_ENSURE_REGISTERED(HapsMobilityModel);

TypeId
HapsMobilityModel::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::HapsMobilityModel")
            .SetParent<MobilityModel>()
            .SetGroupName("NtnSagin")
            .AddConstructor<HapsMobilityModel>()
            .AddAttribute("Altitude",
                          "Nominal HAPS altitude (m); ITU-R F.1500 typical 20 km",
                          DoubleValue(20000.0),
                          MakeDoubleAccessor(&HapsMobilityModel::m_altitudeM),
                          MakeDoubleChecker<double>(15000.0, 50000.0))
            .AddAttribute("StationKeepRadius",
                          "Horizontal station-keeping radius (m)",
                          DoubleValue(5000.0),
                          MakeDoubleAccessor(&HapsMobilityModel::m_radiusM),
                          MakeDoubleChecker<double>(100.0, 50000.0))
            .AddAttribute("HorizontalSpeed",
                          "Linear speed along the racetrack (m/s)",
                          DoubleValue(15.0),
                          MakeDoubleAccessor(&HapsMobilityModel::m_speedMps),
                          MakeDoubleChecker<double>(0.0, 100.0))
            .AddAttribute("MaxVerticalDeviation",
                          "Vertical envelope (m); ITU-R F.1500 typical ±50 m",
                          DoubleValue(50.0),
                          MakeDoubleAccessor(&HapsMobilityModel::m_maxVerticalDeviationM),
                          MakeDoubleChecker<double>(1.0, 1000.0))
            .AddAttribute("TickInterval",
                          "How often to re-evaluate position (s)",
                          TimeValue(Seconds(1.0)),
                          MakeTimeAccessor(&HapsMobilityModel::m_tickInterval),
                          MakeTimeChecker(MilliSeconds(10)));
    return tid;
}

HapsMobilityModel::HapsMobilityModel()
    : m_verticalNoise(CreateObject<NormalRandomVariable>())
{
    m_verticalNoise->SetAttribute("Mean", DoubleValue(0.0));
    m_verticalNoise->SetAttribute("Variance", DoubleValue(1.0));
}

HapsMobilityModel::~HapsMobilityModel()
{
    if (m_tickEvent.IsPending())
    {
        Simulator::Cancel(m_tickEvent);
    }
}

void
HapsMobilityModel::SetCenter(const Vector& centerXyz)
{
    m_center = centerXyz;
    m_currentPosition = Vector(centerXyz.x + m_radiusM,
                               centerXyz.y,
                               centerXyz.z + m_altitudeM);
    if (!m_tickEvent.IsPending())
    {
        m_tickEvent = Simulator::ScheduleNow(&HapsMobilityModel::Tick, this);
    }
}

Vector
HapsMobilityModel::GetCenter() const
{
    return m_center;
}

Vector
HapsMobilityModel::DoGetPosition() const
{
    return m_currentPosition;
}

void
HapsMobilityModel::DoSetPosition(const Vector& position)
{
    m_currentPosition = position;
    m_center = Vector(position.x, position.y, position.z - m_altitudeM);
    NotifyCourseChange();
    if (!m_tickEvent.IsPending())
    {
        m_tickEvent = Simulator::ScheduleNow(&HapsMobilityModel::Tick, this);
    }
}

Vector
HapsMobilityModel::DoGetVelocity() const
{
    return m_currentVelocity;
}

void
HapsMobilityModel::Tick()
{
    // Lemniscate of Bernoulli (figure-8) parameterised by phase t:
    //   x(t) = a · cos(t) / (1 + sin²(t))
    //   y(t) = a · sin(t)·cos(t) / (1 + sin²(t))
    // The arc length per phase step is non-trivial; we approximate by
    // advancing phase at a rate that keeps |v_horiz| ≈ m_speedMps near the
    // longest portion of the curve.
    double dt = m_tickInterval.GetSeconds();
    double dphi = m_speedMps * dt / m_radiusM;
    m_phase += dphi;
    if (m_phase > 2.0 * M_PI)
    {
        m_phase -= 2.0 * M_PI;
    }

    double s = std::sin(m_phase);
    double c = std::cos(m_phase);
    double denom = 1.0 + s * s;
    double xOff = m_radiusM * c / denom;
    double yOff = m_radiusM * s * c / denom;

    // Vertical envelope: bounded random walk via tanh-clamped accumulator.
    double rawZ = m_verticalOffset + 0.05 * m_verticalNoise->GetValue() * dt;
    m_verticalOffset = m_maxVerticalDeviationM *
                       std::tanh(rawZ / m_maxVerticalDeviationM);
    double zOff = m_altitudeM + m_verticalOffset;

    Vector prev = m_currentPosition;
    m_currentPosition = Vector(m_center.x + xOff,
                               m_center.y + yOff,
                               m_center.z + zOff);
    if (dt > 0.0)
    {
        m_currentVelocity = Vector((m_currentPosition.x - prev.x) / dt,
                                   (m_currentPosition.y - prev.y) / dt,
                                   (m_currentPosition.z - prev.z) / dt);
    }
    NotifyCourseChange();
    m_tickEvent = Simulator::Schedule(m_tickInterval,
                                      &HapsMobilityModel::Tick, this);
}

} // namespace ns3
