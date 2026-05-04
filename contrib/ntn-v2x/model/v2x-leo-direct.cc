/*
 * SPDX-License-Identifier: GPL-2.0-only
 * Copyright (c) 2026 Muhammad Uzair (ns3-ntn-toolkit, Workstream W7)
 */
#include "v2x-leo-direct.h"

#include "ns3/double.h"
#include "ns3/log.h"

#include <cmath>

namespace ns3
{
namespace ntnv2x
{

NS_LOG_COMPONENT_DEFINE("V2xLeoDirect");
NS_OBJECT_ENSURE_REGISTERED(V2xLeoDirect);

TypeId
V2xLeoDirect::GetTypeId()
{
    static TypeId tid = TypeId("ns3::ntnv2x::V2xLeoDirect")
                           .SetParent<Object>()
                           .SetGroupName("NtnV2x")
                           .AddConstructor<V2xLeoDirect>();
    return tid;
}

V2xLeoDirect::V2xLeoDirect() = default;

void
V2xLeoDirect::SetFrequencyGHz(double fc)
{
    m_fcGHz = fc;
}

void
V2xLeoDirect::SetSatEirpDbm(double dbm)
{
    m_satEirpDbm = dbm;
}

void
V2xLeoDirect::SetNoiseFloorDbm(double dbm)
{
    m_noiseFloorDbm = dbm;
}

V2xLinkBudget
V2xLeoDirect::Compute(Ptr<MobilityModel> vehicle, Ptr<MobilityModel> sat) const
{
    return ComputeStatic(vehicle->GetPosition(), sat->GetPosition(),
                         m_fcGHz, m_satEirpDbm, m_noiseFloorDbm);
}

V2xLinkBudget
V2xLeoDirect::ComputeStatic(const Vector& v, const Vector& s,
                            double fcGHz, double eirpDbm, double noiseDbm)
{
    Vector d{s.x - v.x, s.y - v.y, s.z - v.z};
    double slant = std::sqrt(d.x * d.x + d.y * d.y + d.z * d.z);
    if (slant < 1.0)
        slant = 1.0;

    V2xLinkBudget out;
    out.slantRangeKm = slant / 1000.0;
    // Free-space PL (dB) = 20·log10(d_m) + 20·log10(fc_GHz) + 32.45
    out.freeSpacePlDb = 20.0 * std::log10(slant) + 20.0 * std::log10(fcGHz) + 32.45;
    out.rxPowerDbm = eirpDbm - out.freeSpacePlDb;
    out.snrDb = out.rxPowerDbm - noiseDbm;
    double horiz = std::sqrt(d.x * d.x + d.y * d.y);
    out.elevationDeg = std::atan2(d.z, horiz) * 180.0 / M_PI;
    out.aboveHorizon = out.elevationDeg > 0.0;
    return out;
}

} // namespace ntnv2x
} // namespace ns3
