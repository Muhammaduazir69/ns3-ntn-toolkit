/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#include "oran-ntn-ofh-interface.h"

#include "ns3/log.h"
#include "ns3/simulator.h"

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("OranNtnOfhInterface");

TypeId
OranNtnOfhInterface::GetTypeId()
{
    static TypeId tid = TypeId("ns3::OranNtnOfhInterface")
                            .SetParent<Object>()
                            .SetGroupName("OranNtn")
                            .AddConstructor<OranNtnOfhInterface>();
    return tid;
}

OranNtnOfhInterface::OranNtnOfhInterface()
{
    m_planeDelays[static_cast<int>(OfhPlane::c_plane)] = MicroSeconds(100);
    m_planeDelays[static_cast<int>(OfhPlane::u_plane)] = MicroSeconds(50);
    m_planeDelays[static_cast<int>(OfhPlane::s_plane)] = MicroSeconds(1);
    m_planeDelays[static_cast<int>(OfhPlane::m_plane)] = MilliSeconds(5);
}

OranNtnOfhInterface::~OranNtnOfhInterface() = default;

void
OranNtnOfhInterface::DoDispose()
{
    for (auto& ev : m_pending)
    {
        Simulator::Cancel(ev);
    }
    m_pending.clear();
    m_du = nullptr;
    m_ru = nullptr;
    m_duRecv = nullptr;
    m_ruRecv = nullptr;
    Object::DoDispose();
}

void
OranNtnOfhInterface::Attach(Ptr<OranNtnSplitGnbEntity> du,
                              Ptr<OranNtnSplitGnbEntity> ru)
{
    m_du = du;
    m_ru = ru;
}

void
OranNtnOfhInterface::SetPlaneDelay(OfhPlane p, Time t)
{
    m_planeDelays[static_cast<int>(p)] = t;
}

Time
OranNtnOfhInterface::GetPlaneDelay(OfhPlane p) const
{
    return m_planeDelays[static_cast<int>(p)];
}

void
OranNtnOfhInterface::SetPlaneUp(OfhPlane p, bool up)
{
    m_planeUp[static_cast<int>(p)] = up;
}

bool
OranNtnOfhInterface::IsPlaneUp(OfhPlane p) const
{
    return m_planeUp[static_cast<int>(p)];
}

uint64_t
OranNtnOfhInterface::MessagesDroppedPerPlane(OfhPlane p) const
{
    return m_planeDroppedCounters[static_cast<int>(p)];
}

bool
OranNtnOfhInterface::SendFromDu(const OfhMessage& msg)
{
    const int pi = static_cast<int>(msg.plane);
    if (!m_planeUp[pi] || !m_du || !m_ru)
    {
        ++m_dropped;
        ++m_planeDroppedCounters[pi];
        return false;
    }
    ++m_duToRu;
    m_planeUsed(msg.plane);
    OfhMessage copy = msg;
    auto ev = Simulator::Schedule(m_planeDelays[pi],
                                    &OranNtnOfhInterface::DispatchToRu,
                                    this,
                                    copy);
    m_pending.push_back(ev);
    return true;
}

bool
OranNtnOfhInterface::SendFromRu(const OfhMessage& msg)
{
    const int pi = static_cast<int>(msg.plane);
    if (!m_planeUp[pi] || !m_du || !m_ru)
    {
        ++m_dropped;
        ++m_planeDroppedCounters[pi];
        return false;
    }
    ++m_ruToDu;
    m_planeUsed(msg.plane);
    OfhMessage copy = msg;
    auto ev = Simulator::Schedule(m_planeDelays[pi],
                                    &OranNtnOfhInterface::DispatchToDu,
                                    this,
                                    copy);
    m_pending.push_back(ev);
    return true;
}

bool
OranNtnOfhInterface::DispatchToRu(OfhMessage msg)
{
    if (m_ruRecv)
    {
        m_ruRecv(msg);
    }
    return true;
}

bool
OranNtnOfhInterface::DispatchToDu(OfhMessage msg)
{
    if (m_duRecv)
    {
        m_duRecv(msg);
    }
    return true;
}

} // namespace ns3
