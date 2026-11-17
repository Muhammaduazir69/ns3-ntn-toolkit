/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#include "ntn-range-delay-channel.h"

#include "orbital-elements.h"
#include "sgp4-mobility-model.h"

#include "ns3/boolean.h"
#include "ns3/drop-tail-queue.h"
#include "ns3/enum.h"
#include "ns3/log.h"
#include "ns3/mac48-address.h"
#include "ns3/net-device-queue-interface.h"
#include "ns3/point-to-point-net-device.h"
#include "ns3/queue-size.h"
#include "ns3/simulator.h"
#include "ns3/string.h"

#include <cmath>

namespace ns3
{
namespace ntncon
{

NS_LOG_COMPONENT_DEFINE("NtnRangeDelayChannel");
NS_OBJECT_ENSURE_REGISTERED(NtnRangeDelayChannel);

namespace
{
constexpr double kC = 299792458.0;
}

TypeId
NtnRangeDelayChannel::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::ntncon::NtnRangeDelayChannel")
            .SetParent<PointToPointChannel>()
            .SetGroupName("NtnConstellation")
            .AddConstructor<NtnRangeDelayChannel>()
            .AddAttribute("DelayMode",
                          "EXACT: per-packet light-time delay on the exact endpoint state. "
                          "FIXED: the base class Delay attribute.",
                          EnumValue(NtnRangeDelayChannel::EXACT),
                          MakeEnumAccessor<DelayMode>(&NtnRangeDelayChannel::m_mode),
                          MakeEnumChecker(NtnRangeDelayChannel::EXACT,
                                          "Exact",
                                          NtnRangeDelayChannel::FIXED,
                                          "Fixed"))
            .AddAttribute("SagnacCorrection",
                          "Rotate the receiver by omega_E tau during the flight (ECEF endpoints).",
                          BooleanValue(true),
                          MakeBooleanAccessor(&NtnRangeDelayChannel::m_sagnac),
                          MakeBooleanChecker())
            .AddTraceSource("PropagationDelay",
                            "Per-packet propagation delay applied by the channel.",
                            MakeTraceSourceAccessor(&NtnRangeDelayChannel::m_delayTrace),
                            "ns3::ntncon::NtnRangeDelayChannel::DelayTracedCallback");
    return tid;
}

NtnRangeDelayChannel::NtnRangeDelayChannel() = default;

void
NtnRangeDelayChannel::SetEndpointMobility(Ptr<MobilityModel> m0, Ptr<MobilityModel> m1)
{
    m_mob[0] = m0;
    m_mob[1] = m1;
}

Ptr<MobilityModel>
NtnRangeDelayChannel::Endpoint(uint32_t i) const
{
    if (m_mob[i])
    {
        return m_mob[i];
    }
    Ptr<PointToPointNetDevice> d = GetPointToPointDevice(i);
    return d ? d->GetNode()->GetObject<MobilityModel>() : nullptr;
}

void
NtnRangeDelayChannel::StateAt(Ptr<MobilityModel> m, Time offset, Vector& p, Vector& v)
{
    if (auto s = DynamicCast<Sgp4MobilityModel>(m))
    {
        s->GetEcefStateAt(offset, p, v); // exact, bypasses the 100 ms cache
        return;
    }
    const Vector p0 = m->GetPosition();
    v = m->GetVelocity();
    const double dt = offset.GetSeconds();
    p = Vector(p0.x + v.x * dt, p0.y + v.y * dt, p0.z + v.z * dt);
}

double
NtnRangeDelayChannel::ComputeDelayS(uint32_t from, Time emitOffset) const
{
    Ptr<MobilityModel> tx = Endpoint(from);
    Ptr<MobilityModel> rx = Endpoint(1 - from);
    if (!tx || !rx)
    {
        return GetDelay().GetSeconds();
    }
    Vector pt;
    Vector vt;
    Vector pr;
    Vector vr;
    StateAt(tx, emitOffset, pt, vt);
    StateAt(rx, emitOffset, pr, vr);
    double tau = CalculateDistance(pt, pr) / kC;
    for (int it = 0; it < 2; ++it)
    {
        Vector q(pr.x + vr.x * tau, pr.y + vr.y * tau, pr.z + vr.z * tau);
        if (m_sagnac)
        {
            // Inertial frame aligned with ECEF at t_e: a point fixed in ECEF at q
            // has rotated eastward by omega_E tau when the signal arrives.
            const double a = kEarthRotationRadS * tau;
            const double c = std::cos(a);
            const double s = std::sin(a);
            q = Vector(c * q.x - s * q.y, s * q.x + c * q.y, q.z);
        }
        tau = CalculateDistance(pt, q) / kC;
    }
    return tau;
}

bool
NtnRangeDelayChannel::TransmitStart(Ptr<const Packet> p,
                                    Ptr<PointToPointNetDevice> src,
                                    Time txTime)
{
    if (m_mode == FIXED)
    {
        return PointToPointChannel::TransmitStart(p, src, txTime);
    }
    NS_ASSERT(IsInitialized());
    const uint32_t wire = (src == GetSource(0)) ? 0 : 1;
    const double d = ComputeDelayS(wire, txTime);
    const Time delay = Seconds(d);
    ++m_packets;
    m_lastDelayS = d;
    m_minDelayS = std::min(m_minDelayS, d);
    m_maxDelayS = std::max(m_maxDelayS, d);
    m_delayTrace(p, delay);
    Ptr<PointToPointNetDevice> dst = GetDestination(wire);
    Simulator::ScheduleWithContext(dst->GetNode()->GetId(),
                                   txTime + delay,
                                   &PointToPointNetDevice::Receive,
                                   dst,
                                   p->Copy());
    return true;
}

NetDeviceContainer
NtnRangeDelayChannel::Connect(Ptr<Node> a,
                              Ptr<Node> b,
                              DataRate rate,
                              const std::string& queueSize,
                              Ptr<NtnRangeDelayChannel>& chOut)
{
    NetDeviceContainer c;
    chOut = CreateObject<NtnRangeDelayChannel>();
    for (Ptr<Node> n : {a, b})
    {
        Ptr<PointToPointNetDevice> dev = CreateObject<PointToPointNetDevice>();
        dev->SetAddress(Mac48Address::Allocate());
        dev->SetDataRate(rate);
        n->AddDevice(dev);
        Ptr<DropTailQueue<Packet>> q = CreateObject<DropTailQueue<Packet>>();
        q->SetMaxSize(QueueSize(queueSize));
        dev->SetQueue(q);
        Ptr<NetDeviceQueueInterface> ndqi = CreateObject<NetDeviceQueueInterface>();
        ndqi->GetTxQueue(0)->ConnectQueueTraces(q);
        dev->AggregateObject(ndqi);
        dev->Attach(chOut);
        c.Add(dev);
    }
    return c;
}

} // namespace ntncon
} // namespace ns3
