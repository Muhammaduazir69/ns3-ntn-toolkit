/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#include "ntn-timing-advance.h"

#include <iostream>

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
    //
    // RRC-2b: the second case used to be silent, and it is not the same thing as
    // the first. A Transparent payload genuinely HAS a feeder leg; returning
    // zero because nobody supplied the gateway makes the advance identical to
    // regenerative and the payload-mode distinction inert. SetGatewayMobility
    // had no caller outside the unit tests, so every shipped scenario in
    // Transparent mode was quietly under-compensating: measured on
    // ntn-rrc-real-stack, 3789 us either way, against 7717 us once the gateway
    // is supplied. Warn once rather than let it pass for a result.
    if (m_payloadMode == PayloadMode::Transparent && m_sat && !m_gw)
    {
        static bool warned = false;
        if (!warned)
        {
            warned = true;
            // std::cerr, not NS_LOG_WARN: the logging macros are compiled out
            // of ns-3 optimized builds, which is how every example in this tree
            // is run, so a warning behind NS_LOG would never have been seen by
            // the people it is for.
            std::cerr << "[ntn-rrc/RRC-2b] Transparent payload with no gateway mobility. The "
                         "feeder leg is omitted, so this timing advance covers the service link "
                         "only and reports what a REGENERATIVE payload would. Call "
                         "NtnRrcHelper::SetGatewayMobility() to include it.\n";
        }
    }
    if (m_payloadMode != PayloadMode::Transparent || !m_gw || !m_sat)
    {
        return 0.0;
    }
    return CalculateDistance(m_gw->GetPosition(), m_sat->GetPosition());
}

void
NtnTimingAdvance::SetSatelliteStateProvider(SatStateProvider provider)
{
    m_satState = provider;
}

Vector
NtnTimingAdvance::GetSatellitePositionAt(Time offset) const
{
    NS_ASSERT_MSG(m_sat, "Satellite mobility must be set");
    if (!m_satState.IsNull())
    {
        Vector p;
        Vector v;
        m_satState(offset, p, v);
        return p;
    }
    const Vector p = m_sat->GetPosition();
    const Vector v = m_sat->GetVelocity();
    const double t = offset.GetSeconds();
    return Vector(p.x + v.x * t, p.y + v.y * t, p.z + v.z * t);
}

NtnTimingAdvance::DelayDerivatives
NtnTimingAdvance::ComputeDelayDerivatives(const Vector& point, Time offset, Time h) const
{
    // Five-point stencils: O(h^4) for the first two derivatives, O(h^2) for the third.
    const double s = h.GetSeconds();
    double f[5];
    for (int k = -2; k <= 2; ++k)
    {
        const Vector p = GetSatellitePositionAt(offset + Seconds(k * s));
        f[k + 2] = 2.0 * CalculateDistance(p, point) / kSpeedOfLight;
    }
    DelayDerivatives d;
    d.ta = f[2];
    d.d1 = (f[0] - 8.0 * f[1] + 8.0 * f[3] - f[4]) / (12.0 * s);
    d.d2 = (-f[0] + 16.0 * f[1] - 30.0 * f[2] + 16.0 * f[3] - f[4]) / (12.0 * s * s);
    d.d3 = (-f[0] + 2.0 * f[1] - 2.0 * f[3] + f[4]) / (2.0 * s * s * s);
    return d;
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
// gives up to 25.77 ms. SetGatewayMobility() supplies the missing geometry; when the
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
    // Both ends in double seconds. The difference used to be taken between
    // two ns3::Time values, each rounded to the 1 ns time resolution, so a
    // 10 ms step quantized the drift to 0.1 us/s steps (0.25 % of a LEO
    // drift, and the whole of a GEO one at 1 ns / 10 ms).
    const double t0 = 2.0 * (GetSlantRangeMetres() + GetFeederRangeMetres()) / kSpeedOfLight;
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

    // RRC-8 (2026-09-18): t0 and t1 must cover the SAME path.
    //
    // ComputeTotalTa() spans the service link plus, in Transparent mode, the
    // gateway feeder leg. This projection used only the service slant, so with
    // a gateway attached the difference carried a spurious -2*feeder/c term and
    // the reported drift was wrong by a constant far larger than the drift
    // itself. The feeder endpoint is fixed on the ground while the satellite
    // moves, so the feeder leg is projected with the satellite as well.
    double feeder1 = 0.0;
    if (m_payloadMode == PayloadMode::Transparent && m_gw)
    {
        feeder1 = CalculateDistance(sat1, m_gw->GetPosition());
    }
    const double t1 = 2.0 * (d1 + feeder1) / kSpeedOfLight;
    const double dTa = t1 - t0;
    return (dt > 0.0) ? (dTa / dt) : 0.0;
}

} // namespace ntnrrc
} // namespace ns3
