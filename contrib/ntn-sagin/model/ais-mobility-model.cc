/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#include "ais-mobility-model.h"

#include "ns3/double.h"
#include "ns3/log.h"
#include "ns3/simulator.h"

#include <cmath>

namespace ns3
{
namespace sagin
{

NS_LOG_COMPONENT_DEFINE("AisMobilityModel");
NS_OBJECT_ENSURE_REGISTERED(AisMobilityModel);

namespace
{

constexpr double R_EARTH_M = 6371000.0;
constexpr double DEG_TO_RAD = M_PI / 180.0;

} // namespace

TypeId
AisMobilityModel::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::sagin::AisMobilityModel")
            .SetParent<MobilityModel>()
            .SetGroupName("NtnSagin")
            .AddConstructor<AisMobilityModel>()
            .AddAttribute("ReferenceLatitudeDeg",
                          "ENU origin latitude (deg)",
                          DoubleValue(0.0),
                          MakeDoubleAccessor(&AisMobilityModel::m_latRefDeg),
                          MakeDoubleChecker<double>(-90.0, 90.0))
            .AddAttribute("ReferenceLongitudeDeg",
                          "ENU origin longitude (deg)",
                          DoubleValue(0.0),
                          MakeDoubleAccessor(&AisMobilityModel::m_lonRefDeg),
                          MakeDoubleChecker<double>(-180.0, 180.0));
    return tid;
}

AisMobilityModel::AisMobilityModel() = default;

void
AisMobilityModel::SetTrace(const AisMaritimeTrace& trace)
{
    m_trace = trace;
    if (!m_trace.samples.empty())
    {
        m_traceTimeOffset_s = m_trace.TStart();
    }
}

void
AisMobilityModel::SetReference(double lat_deg, double lon_deg)
{
    m_latRefDeg = lat_deg;
    m_lonRefDeg = lon_deg;
}

void
AisMobilityModel::SetTraceTimeOffsetSeconds(double t_offset_s)
{
    m_traceTimeOffset_s = t_offset_s;
}

AisSample
AisMobilityModel::SampleAtSimulatorNow() const
{
    if (m_trace.samples.empty())
    {
        return {};
    }
    return m_trace.InterpolateAt(m_traceTimeOffset_s +
                                  Simulator::Now().GetSeconds());
}

AisSample
AisMobilityModel::CurrentSample() const
{
    return SampleAtSimulatorNow();
}

Vector
AisMobilityModel::ToEnu(const AisSample& s) const
{
    const double cosLatRef = std::cos(m_latRefDeg * DEG_TO_RAD);
    const double east = (s.lon_deg - m_lonRefDeg) * DEG_TO_RAD *
                          R_EARTH_M * cosLatRef;
    const double north = (s.lat_deg - m_latRefDeg) * DEG_TO_RAD * R_EARTH_M;
    return Vector(east, north, 0.0); // sea level
}

Vector
AisMobilityModel::DoGetPosition() const
{
    return ToEnu(SampleAtSimulatorNow());
}

void
AisMobilityModel::DoSetPosition(const Vector& /*position*/)
{
    NS_LOG_WARN("AisMobilityModel::DoSetPosition ignored — trace drives "
                "position; use SetTrace + SetReference instead");
}

Vector
AisMobilityModel::DoGetVelocity() const
{
    if (m_trace.samples.empty())
    {
        return Vector(0, 0, 0);
    }
    AisSample s = SampleAtSimulatorNow();
    const double speedMps = s.sog_knots * kKnotsToMps;
    // COG: 0 = true north, increasing clockwise.
    //   east = v * sin(cog), north = v * cos(cog)
    const double cogRad = s.cog_deg * DEG_TO_RAD;
    return Vector(speedMps * std::sin(cogRad),
                   speedMps * std::cos(cogRad),
                   0.0);
}

} // namespace sagin
} // namespace ns3
