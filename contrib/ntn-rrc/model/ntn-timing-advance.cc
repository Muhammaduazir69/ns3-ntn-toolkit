/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#include "ntn-timing-advance.h"

#include <ns3/log.h>
#include <ns3/mobility-model.h>
#include <ns3/simulator.h>

#include <cmath>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("NtnTimingAdvance");

namespace ntnrrc
{

NS_OBJECT_ENSURE_REGISTERED(NtnTimingAdvance);

TypeId
NtnTimingAdvance::GetTypeId()
{
    static TypeId tid = TypeId("ns3::ntnrrc::NtnTimingAdvance")
                            .SetParent<Object>()
                            .SetGroupName("NtnRrc")
                            .AddConstructor<NtnTimingAdvance>()
                            .AddTraceSource("Ta",
                                            "TA samples (total, common, ue-specific)",
                                            MakeTraceSourceAccessor(&NtnTimingAdvance::m_taTrace),
                                            "ns3::ntnrrc::NtnTimingAdvance::TaTracedCallback");
    return tid;
}

NtnTimingAdvance::NtnTimingAdvance() = default;
NtnTimingAdvance::~NtnTimingAdvance() = default;

void
NtnTimingAdvance::SetUeMobility(Ptr<MobilityModel> ue)
{
    m_ue = ue;
}

void
NtnTimingAdvance::SetSatelliteMobility(Ptr<MobilityModel> sat)
{
    m_sat = sat;
}

void
NtnTimingAdvance::SetReferencePosition(const Vector& earthFixedRefPosition)
{
    m_referencePos = earthFixedRefPosition;
}

void
NtnTimingAdvance::SetPayloadMode(PayloadMode mode)
{
    m_payloadMode = mode;
}

double
NtnTimingAdvance::GetSlantRangeMetres() const
{
    NS_ASSERT_MSG(m_ue && m_sat, "UE and satellite mobility must be set before computing TA");
    return CalculateDistance(m_ue->GetPosition(), m_sat->GetPosition());
}

double
NtnTimingAdvance::GetReferenceRangeMetres() const
{
    NS_ASSERT_MSG(m_sat, "Satellite mobility must be set before computing reference range");
    return CalculateDistance(m_referencePos, m_sat->GetPosition());
}

// Timing Advance compensates the ROUND-TRIP propagation delay so the UE's
// uplink arrives time-aligned at the receiver. The service link (UE<->satellite)
// round trip is 2*d/c, and this holds for BOTH payload modes — the earlier
// one-way value for the regenerative case under-compensated the uplink by 2x.
//
// NOTE (documented omission, not a bug): in Transparent (bent-pipe) mode the gNB
// sits on the ground, so the full uplink additionally traverses the
// satellite<->gateway feeder link; that feeder round-trip term (2*d_feeder/c) is
// not added here because this class is not given the gateway geometry. Callers
// that need the transparent-mode feeder delay must add it from the feeder pair.
Time
NtnTimingAdvance::ComputeTotalTa() const
{
    const double d = GetSlantRangeMetres();
    return Seconds(2.0 * d / kSpeedOfLight);
}

Time
NtnTimingAdvance::ComputeCommonTa() const
{
    const double d = GetReferenceRangeMetres();
    return Seconds(2.0 * d / kSpeedOfLight);
}

Time
NtnTimingAdvance::ComputeUeSpecificTa() const
{
    return ComputeTotalTa() - ComputeCommonTa();
}

double
NtnTimingAdvance::ComputeTaDriftRate(Time eps) const
{
    NS_ASSERT_MSG(m_ue && m_sat, "Mobility models required for drift rate");
    const Time t0 = ComputeTotalTa();
    const Vector ue0 = m_ue->GetPosition();
    const Vector sat0 = m_sat->GetPosition();
    const Vector ueV = m_ue->GetVelocity();
    const Vector satV = m_sat->GetVelocity();

    // Project positions forward by `eps` using current velocities (closed form
    // — does not advance the global Simulator clock).
    const double dt = eps.GetSeconds();
    const Vector ue1{ue0.x + ueV.x * dt, ue0.y + ueV.y * dt, ue0.z + ueV.z * dt};
    const Vector sat1{sat0.x + satV.x * dt, sat0.y + satV.y * dt, sat0.z + satV.z * dt};
    const double d1 = CalculateDistance(ue1, sat1);
    // TA_total is the round-trip service-link delay (2*d/c) for BOTH payload
    // modes — see ComputeTotalTa(). t0 above already uses 2*d/c, so t1 must too;
    // a mode-dependent multiplier here made regenerative drift ~= (d/c - 2d/c)/dt
    // ~= -0.18 s/s at LEO regardless of geometry (poisoning SIB19
    // taCommonDriftRate). Use 2.0 unconditionally to match ComputeTotalTa().
    const Time t1 = Seconds(2.0 * d1 / kSpeedOfLight);
    const double dTa = (t1 - t0).GetSeconds();
    return (dt > 0.0) ? (dTa / dt) : 0.0;
}

} // namespace ntnrrc
} // namespace ns3
