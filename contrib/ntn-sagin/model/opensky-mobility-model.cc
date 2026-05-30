/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#include "opensky-mobility-model.h"

#include "ns3/double.h"
#include "ns3/log.h"
#include "ns3/simulator.h"

#include <algorithm>
#include <cmath>

namespace ns3
{
namespace sagin
{

NS_LOG_COMPONENT_DEFINE("OpenSkyMobilityModel");
NS_OBJECT_ENSURE_REGISTERED(OpenSkyMobilityModel);

namespace
{

constexpr double R_EARTH_M = 6371000.0;
constexpr double DEG_TO_RAD = M_PI / 180.0;

} // namespace

TypeId
OpenSkyMobilityModel::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::sagin::OpenSkyMobilityModel")
            .SetParent<MobilityModel>()
            .SetGroupName("NtnSagin")
            .AddConstructor<OpenSkyMobilityModel>()
            .AddAttribute("ReferenceLatitudeDeg",
                          "ENU origin latitude (deg)",
                          DoubleValue(0.0),
                          MakeDoubleAccessor(&OpenSkyMobilityModel::m_latRefDeg),
                          MakeDoubleChecker<double>(-90.0, 90.0))
            .AddAttribute("ReferenceLongitudeDeg",
                          "ENU origin longitude (deg)",
                          DoubleValue(0.0),
                          MakeDoubleAccessor(&OpenSkyMobilityModel::m_lonRefDeg),
                          MakeDoubleChecker<double>(-180.0, 180.0))
            .AddAttribute("ReferenceAltitudeM",
                          "ENU origin altitude (m, MSL)",
                          DoubleValue(0.0),
                          MakeDoubleAccessor(&OpenSkyMobilityModel::m_altRefM),
                          MakeDoubleChecker<double>());
    return tid;
}

OpenSkyMobilityModel::OpenSkyMobilityModel() = default;

void
OpenSkyMobilityModel::SetTrace(const OpenSkyAdsbTrace& trace)
{
    m_trace = trace;
    if (!m_trace.samples.empty())
    {
        m_traceTimeOffset_s = m_trace.TStart();
    }
}

void
OpenSkyMobilityModel::SetReference(double lat_deg, double lon_deg, double alt_m)
{
    m_latRefDeg = lat_deg;
    m_lonRefDeg = lon_deg;
    m_altRefM = alt_m;
}

void
OpenSkyMobilityModel::SetTraceTimeOffsetSeconds(double t_offset_s)
{
    m_traceTimeOffset_s = t_offset_s;
}

OpenSkySample
OpenSkyMobilityModel::SampleAtSimulatorNow() const
{
    if (m_trace.samples.empty())
    {
        return {};
    }
    return m_trace.InterpolateAt(m_traceTimeOffset_s +
                                  Simulator::Now().GetSeconds());
}

OpenSkySample
OpenSkyMobilityModel::CurrentSample() const
{
    return SampleAtSimulatorNow();
}

Vector
OpenSkyMobilityModel::ToEnu(const OpenSkySample& s) const
{
    const double cosLatRef = std::cos(m_latRefDeg * DEG_TO_RAD);
    const double east = (s.lon_deg - m_lonRefDeg) * DEG_TO_RAD *
                          R_EARTH_M * cosLatRef;
    const double north = (s.lat_deg - m_latRefDeg) * DEG_TO_RAD * R_EARTH_M;
    const double up = s.alt_m - m_altRefM;
    return Vector(east, north, up);
}

double
OpenSkyMobilityModel::CurrentEastM() const
{
    return ToEnu(SampleAtSimulatorNow()).x;
}

double
OpenSkyMobilityModel::CurrentNorthM() const
{
    return ToEnu(SampleAtSimulatorNow()).y;
}

double
OpenSkyMobilityModel::CurrentUpM() const
{
    return ToEnu(SampleAtSimulatorNow()).z;
}

Vector
OpenSkyMobilityModel::DoGetPosition() const
{
    return ToEnu(SampleAtSimulatorNow());
}

void
OpenSkyMobilityModel::DoSetPosition(const Vector& /*position*/)
{
    // No-op: position is determined by the trace + sim time. Direct
    // SetPosition() calls from helpers (e.g. MobilityHelper::Install)
    // should not corrupt the trace.
    NS_LOG_WARN("OpenSkyMobilityModel::DoSetPosition ignored — trace "
                "drives position; use SetTrace + SetReference instead");
}

Vector
OpenSkyMobilityModel::DoGetVelocity() const
{
    if (m_trace.samples.size() < 2)
    {
        return Vector(0, 0, 0);
    }
    OpenSkySample s = SampleAtSimulatorNow();
    // Heading-based ground-plane velocity from the latest sample's
    // velocity_mps + heading. Vertical rate is computed from the trace
    // slope between adjacent samples bracketing `s.time_s`.
    const double hdgRad = s.heading_deg * DEG_TO_RAD;
    // ADS-B "heading" is bearing from true north, clockwise -> ENU:
    //   east = velocity * sin(heading), north = velocity * cos(heading)
    const double east = s.velocity_mps * std::sin(hdgRad);
    const double north = s.velocity_mps * std::cos(hdgRad);

    double vz = 0.0;
    // Vertical rate: numerical derivative of alt between bracketing samples.
    if (m_trace.samples.size() >= 2)
    {
        auto it = std::upper_bound(m_trace.samples.begin(),
                                     m_trace.samples.end(),
                                     s.time_s,
                                     [](double t, const OpenSkySample& a) {
                                         return t < a.time_s;
                                     });
        if (it != m_trace.samples.begin() && it != m_trace.samples.end())
        {
            const OpenSkySample& hi = *it;
            const OpenSkySample& lo = *(it - 1);
            const double dt = hi.time_s - lo.time_s;
            if (dt > 0.0)
            {
                vz = (hi.alt_m - lo.alt_m) / dt;
            }
        }
    }
    return Vector(east, north, vz);
}

} // namespace sagin
} // namespace ns3
