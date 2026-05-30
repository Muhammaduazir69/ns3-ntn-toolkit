/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#include "oran-ntn-f1-interface.h"

#include "ns3/log.h"
#include "ns3/simulator.h"

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("OranNtnF1Interface");

TypeId
OranNtnF1Interface::GetTypeId()
{
    static TypeId tid = TypeId("ns3::OranNtnF1Interface")
                            .SetParent<Object>()
                            .SetGroupName("OranNtn")
                            .AddConstructor<OranNtnF1Interface>();
    return tid;
}

OranNtnF1Interface::OranNtnF1Interface()
    : m_delay(MilliSeconds(0) + MicroSeconds(500))
{
}

OranNtnF1Interface::~OranNtnF1Interface() = default;

void
OranNtnF1Interface::DoDispose()
{
    for (auto& ev : m_pending)
    {
        Simulator::Cancel(ev);
    }
    m_pending.clear();
    m_cuCp = nullptr;
    m_cuUp = nullptr;
    m_du = nullptr;
    m_cuCpRecv = nullptr;
    m_cuUpRecv = nullptr;
    m_duRecv = nullptr;
    Object::DoDispose();
}

void
OranNtnF1Interface::Attach(Ptr<OranNtnSplitGnbEntity> cu_cp,
                             Ptr<OranNtnSplitGnbEntity> du)
{
    m_cuCp = cu_cp;
    m_du = du;
}

void
OranNtnF1Interface::AttachCuUp(Ptr<OranNtnSplitGnbEntity> cu_up)
{
    m_cuUp = cu_up;
}

void
OranNtnF1Interface::SetLinkUp(bool up)
{
    if (!up)
    {
        for (auto& ev : m_pending)
        {
            Simulator::Cancel(ev);
        }
        m_pending.clear();
    }
    m_up = up;
}

bool
OranNtnF1Interface::SendFromCuCp(const F1apMessage& msg)
{
    if (!m_up || !m_cuCp || !m_du)
    {
        ++m_dropped;
        return false;
    }
    ++m_cuToDu;
    m_msgDispatched(msg.kind);
    F1apMessage copy = msg;
    auto ev = Simulator::Schedule(m_delay,
                                    &OranNtnF1Interface::DispatchToDu,
                                    this,
                                    copy);
    m_pending.push_back(ev);
    return true;
}

bool
OranNtnF1Interface::SendFromCuUp(const F1apMessage& msg)
{
    if (!m_up || !m_cuUp || !m_du)
    {
        ++m_dropped;
        return false;
    }
    ++m_cuToDu;
    m_msgDispatched(msg.kind);
    F1apMessage copy = msg;
    auto ev = Simulator::Schedule(m_delay,
                                    &OranNtnF1Interface::DispatchToDu,
                                    this,
                                    copy);
    m_pending.push_back(ev);
    return true;
}

bool
OranNtnF1Interface::SendFromDu(const F1apMessage& msg)
{
    if (!m_up || !m_du)
    {
        ++m_dropped;
        return false;
    }
    ++m_duToCu;
    m_msgDispatched(msg.kind);
    F1apMessage copy = msg;
    // CU-UP-bound vs CU-CP-bound is selected by message kind: UE-context
    // signalling goes CP, RRC transfers go CP, anything else defaults CP.
    auto ev = Simulator::Schedule(m_delay,
                                    &OranNtnF1Interface::DispatchToCuCp,
                                    this,
                                    copy);
    m_pending.push_back(ev);
    return true;
}

bool
OranNtnF1Interface::DispatchToDu(F1apMessage msg)
{
    if (m_duRecv)
    {
        m_duRecv(msg);
    }
    return true;
}

bool
OranNtnF1Interface::DispatchToCuCp(F1apMessage msg)
{
    if (m_cuCpRecv)
    {
        m_cuCpRecv(msg);
    }
    return true;
}

bool
OranNtnF1Interface::DispatchToCuUp(F1apMessage msg)
{
    if (m_cuUpRecv)
    {
        m_cuUpRecv(msg);
    }
    return true;
}

} // namespace ns3
