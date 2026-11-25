// Copyright (c) 2026 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only

#include "random-direction-disc-2d-mobility-model.h"

#include "ns3/double.h"
#include "ns3/log.h"
#include "ns3/pointer.h"
#include "ns3/simulator.h"
#include "ns3/string.h"

#include <algorithm>
#include <cmath>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("RandomDirectionDisc2dMobilityModel");
NS_OBJECT_ENSURE_REGISTERED(RandomDirectionDisc2dMobilityModel);

TypeId
RandomDirectionDisc2dMobilityModel::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::RandomDirectionDisc2dMobilityModel")
            .SetParent<MobilityModel>()
            .SetGroupName("Mobility")
            .AddConstructor<RandomDirectionDisc2dMobilityModel>()
            .AddAttribute("CenterX",
                          "X coordinate of the disc centre (m)",
                          DoubleValue(0.0),
                          MakeDoubleAccessor(&RandomDirectionDisc2dMobilityModel::m_centerX),
                          MakeDoubleChecker<double>())
            .AddAttribute("CenterY",
                          "Y coordinate of the disc centre (m)",
                          DoubleValue(0.0),
                          MakeDoubleAccessor(&RandomDirectionDisc2dMobilityModel::m_centerY),
                          MakeDoubleChecker<double>())
            .AddAttribute("Radius",
                          "Disc radius (m). Must be strictly positive.",
                          DoubleValue(100.0),
                          MakeDoubleAccessor(&RandomDirectionDisc2dMobilityModel::m_radius),
                          MakeDoubleChecker<double>(std::numeric_limits<double>::min()))
            .AddAttribute("Speed",
                          "A random variable to control the speed (m/s).",
                          StringValue("ns3::UniformRandomVariable[Min=1.0|Max=2.0]"),
                          MakePointerAccessor(&RandomDirectionDisc2dMobilityModel::m_speed),
                          MakePointerChecker<RandomVariableStream>())
            .AddAttribute("Pause",
                          "A random variable to control the pause (s).",
                          StringValue("ns3::ConstantRandomVariable[Constant=2.0]"),
                          MakePointerAccessor(&RandomDirectionDisc2dMobilityModel::m_pause),
                          MakePointerChecker<RandomVariableStream>());
    return tid;
}

RandomDirectionDisc2dMobilityModel::RandomDirectionDisc2dMobilityModel()
{
    m_direction = CreateObject<UniformRandomVariable>();
}

RandomDirectionDisc2dMobilityModel::~RandomDirectionDisc2dMobilityModel()
{
    m_event.Cancel();
}

void
RandomDirectionDisc2dMobilityModel::DoDispose()
{
    MobilityModel::DoDispose();
}

void
RandomDirectionDisc2dMobilityModel::DoInitialize()
{
    StartLeg();
    MobilityModel::DoInitialize();
}

double
RandomDirectionDisc2dMobilityModel::TimeToBoundary(const Vector& position,
                                                   const Vector& velocity) const
{
    // Quadratic |q + t*v|^2 = R^2, with q = position - centre.
    const double qx = position.x - m_centerX;
    const double qy = position.y - m_centerY;
    const double vx = velocity.x;
    const double vy = velocity.y;
    const double a = vx * vx + vy * vy;
    if (a <= 0.0)
    {
        return 0.0;
    }
    const double b = 2.0 * (qx * vx + qy * vy);
    const double c = qx * qx + qy * qy - m_radius * m_radius;
    const double disc = b * b - 4.0 * a * c;
    if (disc < 0.0)
    {
        // Should not happen if the position is inside the disc and the velocity is finite.
        return 0.0;
    }
    const double sqrtDisc = std::sqrt(disc);
    // Take the positive root (forward in time).
    double t = (-b + sqrtDisc) / (2.0 * a);
    if (t <= 0.0)
    {
        // Numerical drift can leave us just outside; the other root would be the
        // backward intersection. Fall back to 0 so we immediately restart inward.
        t = 0.0;
    }
    return t;
}

void
RandomDirectionDisc2dMobilityModel::RunLeg(double direction)
{
    m_helper.Update();
    const double speed = m_speed->GetValue();
    const Vector velocity(std::cos(direction) * speed, std::sin(direction) * speed, 0.0);
    m_helper.SetVelocity(velocity);
    m_helper.Unpause();
    const Vector position = m_helper.GetCurrentPosition();
    const double t = TimeToBoundary(position, velocity);
    m_event.Cancel();
    m_event =
        Simulator::Schedule(Seconds(t), &RandomDirectionDisc2dMobilityModel::BeginPause, this);
    NotifyCourseChange();
}

void
RandomDirectionDisc2dMobilityModel::StartLeg()
{
    // First leg uses a fully uniform direction; subsequent legs after boundary hits use
    // RestartInwardLeg() to ensure the velocity points back into the disc.
    const double direction = m_direction->GetValue(0.0, 2.0 * M_PI);
    RunLeg(direction);
}

void
RandomDirectionDisc2dMobilityModel::BeginPause()
{
    m_helper.Update();
    m_helper.Pause();
    Time pause = Seconds(m_pause->GetValue());
    m_event.Cancel();
    m_event =
        Simulator::Schedule(pause, &RandomDirectionDisc2dMobilityModel::RestartInwardLeg, this);
    NotifyCourseChange();
}

void
RandomDirectionDisc2dMobilityModel::RestartInwardLeg()
{
    m_helper.Update();
    const Vector position = m_helper.GetCurrentPosition();
    // Outward normal angle at the boundary point.
    const double thetaN = std::atan2(position.y - m_centerY, position.x - m_centerX);
    // Inward half-plane: direction in (thetaN + pi/2, thetaN + 3*pi/2) so cos(d - thetaN) < 0.
    const double direction = m_direction->GetValue(thetaN + M_PI_2, thetaN + M_PI + M_PI_2);
    RunLeg(direction);
}

Vector
RandomDirectionDisc2dMobilityModel::DoGetPosition() const
{
    m_helper.Update();
    return m_helper.GetCurrentPosition();
}

void
RandomDirectionDisc2dMobilityModel::DoSetPosition(const Vector& position)
{
    m_helper.SetPosition(position);
    m_event.Cancel();
    m_event = Simulator::ScheduleNow(&RandomDirectionDisc2dMobilityModel::StartLeg, this);
}

Vector
RandomDirectionDisc2dMobilityModel::DoGetVelocity() const
{
    return m_helper.GetVelocity();
}

int64_t
RandomDirectionDisc2dMobilityModel::DoAssignStreams(int64_t stream)
{
    m_direction->SetStream(stream);
    m_speed->SetStream(stream + 1);
    m_pause->SetStream(stream + 2);
    return 3;
}

} // namespace ns3
