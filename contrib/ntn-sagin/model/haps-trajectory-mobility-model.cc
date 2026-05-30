/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#include "haps-trajectory-mobility-model.h"

#include "ns3/log.h"
#include "ns3/simulator.h"

#include <cmath>

namespace ns3
{
namespace sagin
{

namespace
{
constexpr double R_EARTH_M = 6371000.0;
constexpr double DEG_TO_RAD = M_PI / 180.0;
} // namespace

NS_LOG_COMPONENT_DEFINE("HapsTrajectoryMobilityModel");
NS_OBJECT_ENSURE_REGISTERED(HapsTrajectoryMobilityModel);

TypeId
HapsTrajectoryMobilityModel::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::sagin::HapsTrajectoryMobilityModel")
            .SetParent<MobilityModel>()
            .SetGroupName("NtnSagin")
            .AddConstructor<HapsTrajectoryMobilityModel>();
    return tid;
}

HapsTrajectoryMobilityModel::HapsTrajectoryMobilityModel() = default;

void
HapsTrajectoryMobilityModel::SetTrace(const HapsTrajectoryTrace& trace)
{
    m_trace = trace;
    if (!m_trace.samples.empty())
    {
        m_traceTimeOffset_s = m_trace.TStart();
    }
}

void
HapsTrajectoryMobilityModel::SetReference(double lat_deg,
                                            double lon_deg,
                                            double alt_m)
{
    m_latRefDeg = lat_deg;
    m_lonRefDeg = lon_deg;
    m_altRefM = alt_m;
}

void
HapsTrajectoryMobilityModel::SetTraceTimeOffsetSeconds(double t_offset_s)
{
    m_traceTimeOffset_s = t_offset_s;
}

HapsTrajectorySample
HapsTrajectoryMobilityModel::SampleAtSimulatorNow() const
{
    if (m_trace.samples.empty())
    {
        return HapsTrajectorySample{};
    }
    const double t = m_traceTimeOffset_s +
                       Simulator::Now().GetSeconds();
    return m_trace.InterpolateAt(t);
}

HapsTrajectorySample
HapsTrajectoryMobilityModel::CurrentSample() const
{
    return SampleAtSimulatorNow();
}

Vector
HapsTrajectoryMobilityModel::ToEnu(const HapsTrajectorySample& s) const
{
    const double cosLatRef = std::cos(m_latRefDeg * DEG_TO_RAD);
    const double east = (s.lon_deg - m_lonRefDeg) * DEG_TO_RAD *
                          R_EARTH_M * cosLatRef;
    const double north =
        (s.lat_deg - m_latRefDeg) * DEG_TO_RAD * R_EARTH_M;
    const double up = s.alt_m - m_altRefM;
    return Vector(east, north, up);
}

Vector
HapsTrajectoryMobilityModel::DoGetPosition() const
{
    return ToEnu(SampleAtSimulatorNow());
}

void
HapsTrajectoryMobilityModel::DoSetPosition(const Vector&)
{
    NS_LOG_WARN(
        "HapsTrajectoryMobilityModel::DoSetPosition ignored — trace "
        "drives position; use SetTrace + SetReference instead");
}

Vector
HapsTrajectoryMobilityModel::DoGetVelocity() const
{
    if (m_trace.samples.size() < 2)
    {
        return Vector(0, 0, 0);
    }
    constexpr double dt = 0.5;
    const double now_s = Simulator::Now().GetSeconds();
    const double t1 = m_traceTimeOffset_s + now_s;
    const double t0 = std::max(m_trace.TStart(), t1 - dt);
    const double t2 = std::min(m_trace.TEnd(), t1 + dt);
    const auto p0 = ToEnu(m_trace.InterpolateAt(t0));
    const auto p1 = ToEnu(m_trace.InterpolateAt(t2));
    const double span = t2 - t0;
    if (span <= 0.0)
    {
        return Vector(0, 0, 0);
    }
    return Vector((p1.x - p0.x) / span,
                  (p1.y - p0.y) / span,
                  (p1.z - p0.z) / span);
}

} // namespace sagin
} // namespace ns3
