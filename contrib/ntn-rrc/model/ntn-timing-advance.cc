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

void
NtnTimingAdvance::SetGatewayMobility(Ptr<MobilityModel> gw)
{
    m_gw = gw;
}

double
NtnTimingAdvance::GetFeederRangeMetres() const
{
    // Zero when the payload terminates on board (the gNB is the satellite, so
    // there is no feeder leg in the timing loop) or when no gateway was given.
    if (m_payloadMode != PayloadMode::Transparent || !m_gw || !m_sat)
    {
        return 0.0;
    }
    return CalculateDistance(m_gw->GetPosition(), m_sat->GetPosition());
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
// RRC-2 FIX (2026-08-24): the feeder leg is no longer omitted. In Transparent
// (bent-pipe) mode the gNB sits on the ground, so the uplink traverses the
// satellite<->gateway feeder link as well and the advance must cover both legs.
// Previously this class had no gateway geometry, returned the service-link round
// trip for every payload mode, and m_payloadMode was stored but never read: a
// transparent LEO-600 cell was compensated for ~4 ms when TR 38.821 Table 4.2-2
// gives ~41.77 ms. SetGatewayMobility() supplies the missing geometry; when the
// mode needs it and it is absent, FeederGeometryMissing() reports so rather than
// the shortfall passing silently.
Time
NtnTimingAdvance::ComputeTotalTa() const
{
    const double d = GetSlantRangeMetres() + GetFeederRangeMetres();
    return Seconds(2.0 * d / kSpeedOfLight);
}

Time
NtnTimingAdvance::ComputeCommonTa() const
{
    const double d = GetReferenceRangeMetres() + GetFeederRangeMetres();
    return Seconds(2.0 * d / kSpeedOfLight);
}

Time
NtnTimingAdvance::ComputeUeSpecificTa() const
{
    return ComputeTotalTa() - ComputeCommonTa();
}

double
NtnTimingAdvance::ComputeTaDriftVariation(Time eps) const
{
    NS_ASSERT_MSG(m_ue && m_sat, "Mobility models required for drift variation");
    const double dt = eps.GetSeconds();
    if (dt <= 0.0)
    {
        return 0.0;
    }
    // RRC-6: central difference of the drift rate, i.e. the second derivative
    // of TA_common. SIB19's ta-CommonDriftVariant-r17 is the rate of change of
    // the drift rate; without it a UE extrapolating its timing between
    // broadcasts follows a straight line, which is worst exactly at the closest
    // approach of a LEO pass where the drift reverses sign fastest.
    //
    // Both samples are taken from projected positions, closed form, without
    // advancing the global clock: forward over [t, t+eps] and backward over
    // [t-eps, t].
    const Vector ue0 = m_ue->GetPosition();
    const Vector sat0 = m_sat->GetPosition();
    const Vector ueV = m_ue->GetVelocity();
    const Vector satV = m_sat->GetVelocity();

    auto taAt = [&](double step) {
        const Vector u{ue0.x + ueV.x * step, ue0.y + ueV.y * step, ue0.z + ueV.z * step};
        const Vector v{sat0.x + satV.x * step, sat0.y + satV.y * step, sat0.z + satV.z * step};
        return 2.0 * CalculateDistance(u, v) / kSpeedOfLight;
    };

    const double taMinus = taAt(-dt);
    const double taZero = taAt(0.0);
    const double taPlus = taAt(dt);

    const double driftFwd = (taPlus - taZero) / dt;   // over [t, t+eps]
    const double driftBack = (taZero - taMinus) / dt; // over [t-eps, t]
    return (driftFwd - driftBack) / dt;
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
