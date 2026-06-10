/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only
#include "oran-ntn-rt-ric.h"

#include <ns3/abort.h>
#include <ns3/log.h>
#include <ns3/simulator.h>

#include <cmath>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("OranNtnRtRic");
NS_OBJECT_ENSURE_REGISTERED(OranNtnRtRic);

TypeId
OranNtnRtRic::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::OranNtnRtRic")
            .SetParent<Object>()
            .SetGroupName("OranNtn")
            .AddConstructor<OranNtnRtRic>()
            .AddTraceSource("Actuation",
                            "An RT action actuated (name, ueIndex, sinrDb).",
                            MakeTraceSourceAccessor(&OranNtnRtRic::m_actuationTrace),
                            "ns3::OranNtnRtRic::ActuationTracedCallback");
    return tid;
}

OranNtnRtRic::OranNtnRtRic() = default;

void
OranNtnRtRic::SetKpiSources(KpiGetter sinrDb, KpiGetter tbler)
{
    m_sinrGetter = std::move(sinrDb);
    m_tblerGetter = std::move(tbler);
}

void
OranNtnRtRic::AddRtAction(std::string name, RtAction action)
{
    m_actions.push_back({std::move(name), std::move(action)});
}

void
OranNtnRtRic::Start()
{
    NS_ABORT_MSG_IF(m_period >= MilliSeconds(10),
                    "OranNtnRtRic: loop period " << m_period.GetMilliSeconds()
                                                 << " ms violates the O-RAN RT bound (<10 ms); "
                                                    "use the Near-RT RIC for slower loops");
    NS_ABORT_MSG_IF(!m_sinrGetter || !m_tblerGetter,
                    "OranNtnRtRic: KPI sources not wired (SetKpiSources)");
    if (m_running)
    {
        return;
    }
    m_running = true;
    m_loopEvent = Simulator::Schedule(m_period, &OranNtnRtRic::Loop, this);
}

void
OranNtnRtRic::Stop()
{
    m_running = false;
    m_loopEvent.Cancel();
}

void
OranNtnRtRic::Loop()
{
    if (!m_running)
    {
        return;
    }
    ++m_loops;
    for (uint32_t u = 0; u < m_numUes; ++u)
    {
        RtKpi kpi;
        kpi.ueIndex = u;
        kpi.sinrDb = m_sinrGetter(u);
        kpi.tbler = m_tblerGetter(u);
        if (std::isnan(kpi.sinrDb)) // no PHY samples yet for this UE
        {
            continue;
        }
        for (const auto& a : m_actions)
        {
            if (a.fn(kpi))
            {
                ++m_actuations;
                m_actuationTrace(a.name, u, kpi.sinrDb);
            }
        }
    }
    m_loopEvent = Simulator::Schedule(m_period, &OranNtnRtRic::Loop, this);
}

} // namespace ns3
