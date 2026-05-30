/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#include "hst-mobility-model.h"

#include "ns3/double.h"
#include "ns3/log.h"
#include "ns3/simulator.h"

namespace ns3
{
namespace sagin
{

NS_LOG_COMPONENT_DEFINE("HstMobilityModel");
NS_OBJECT_ENSURE_REGISTERED(HstMobilityModel);

TypeId
HstMobilityModel::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::sagin::HstMobilityModel")
            .SetParent<MobilityModel>()
            .SetGroupName("NtnSagin")
            .AddConstructor<HstMobilityModel>()
            .AddAttribute("CarrierFrequencyHz",
                          "Carrier frequency for GetDopplerHz()",
                          DoubleValue(30e9),
                          MakeDoubleAccessor(&HstMobilityModel::m_carrierHz),
                          MakeDoubleChecker<double>(1e6, 1e12));
    return tid;
}

HstMobilityModel::HstMobilityModel()
{
    // Default to a TR 38.901 HST-C trace (350 km/h, Dmin=100 m).
    m_trace = HstTraceGenerator::PresetTR38901_C(/*duration_s=*/3600.0,
                                                  /*numSamples=*/3601);
}

void
HstMobilityModel::SetTrace(const HstTrace& trace)
{
    m_trace = trace;
}

void
HstMobilityModel::SetCarrierFrequencyHz(double freqHz)
{
    m_carrierHz = freqHz;
}

double
HstMobilityModel::GetDopplerHz(double gnb_x_m) const
{
    return m_trace.DopplerHzAt(Simulator::Now().GetSeconds(),
                                gnb_x_m,
                                m_carrierHz);
}

Vector
HstMobilityModel::DoGetPosition() const
{
    // Analytic linear motion — no interpolation needed.
    const double t = Simulator::Now().GetSeconds();
    const double v = m_trace.SpeedMps();
    return Vector(m_trace.startPos_m + v * t, 0.0, 0.0);
}

void
HstMobilityModel::DoSetPosition(const Vector& position)
{
    // Shift the trace origin so the train is at `position.x` at the
    // current sim time. Lateral / vertical components are ignored — the
    // train is constrained to the rail corridor.
    const double t = Simulator::Now().GetSeconds();
    m_trace.startPos_m = position.x - m_trace.SpeedMps() * t;
}

Vector
HstMobilityModel::DoGetVelocity() const
{
    return Vector(m_trace.SpeedMps(), 0.0, 0.0);
}

} // namespace sagin
} // namespace ns3
