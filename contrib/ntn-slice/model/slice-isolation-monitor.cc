/*
 * SPDX-License-Identifier: GPL-2.0-only
 * Copyright (c) 2026 Muhammad Uzair (ns3-ntn-toolkit, Workstream W6)
 */
#include "slice-isolation-monitor.h"

#include "ns3/log.h"
#include "ns3/uinteger.h"

#include <algorithm>

namespace ns3
{
namespace ntnslice
{

NS_LOG_COMPONENT_DEFINE("SliceIsolationMonitor");
NS_OBJECT_ENSURE_REGISTERED(SliceIsolationMonitor);

TypeId
SliceIsolationMonitor::GetTypeId()
{
    static TypeId tid = TypeId("ns3::ntnslice::SliceIsolationMonitor")
                           .SetParent<Object>()
                           .SetGroupName("NtnSlice")
                           .AddConstructor<SliceIsolationMonitor>()
                           .AddTraceSource("Breach",
                                           "Slice SLA breach event",
                                           MakeTraceSourceAccessor(&SliceIsolationMonitor::m_traceBreach),
                                           "ns3::ntnslice::SliceIsolationMonitor::BreachTrace");
    return tid;
}

SliceIsolationMonitor::SliceIsolationMonitor() = default;

void
SliceIsolationMonitor::RegisterSlice(const SliceProfile& profile)
{
    PerSlice ps;
    ps.profile = profile;
    m_slices[profile.snssai] = std::move(ps);
}

void
SliceIsolationMonitor::SetWindow(uint32_t window)
{
    m_window = std::max<uint32_t>(window, 10);
}

void
SliceIsolationMonitor::RecordPacket(const Snssai& s, double latencyMs, bool delivered)
{
    auto it = m_slices.find(s);
    if (it == m_slices.end())
    {
        return;
    }
    PerSlice& ps = it->second;
    ps.latencyMsWindow.push_back(latencyMs);
    ps.deliveredWindow.push_back(delivered);
    while (ps.latencyMsWindow.size() > m_window)
    {
        ps.latencyMsWindow.pop_front();
    }
    while (ps.deliveredWindow.size() > m_window)
    {
        ps.deliveredWindow.pop_front();
    }
}

BreachEvent
SliceIsolationMonitor::EvaluateSlice(const PerSlice& ps) const
{
    BreachEvent ev{};
    ev.snssai = ps.profile.snssai;
    ev.latencyBudgetMs = ps.profile.latencyBudgetMs;
    ev.reliabilityTarget = ps.profile.reliability;
    ev.windowSamples = ps.latencyMsWindow.size();

    // p99 latency
    if (!ps.latencyMsWindow.empty())
    {
        std::vector<double> lat(ps.latencyMsWindow.begin(), ps.latencyMsWindow.end());
        std::sort(lat.begin(), lat.end());
        std::size_t idx = static_cast<std::size_t>(0.99 * (lat.size() - 1));
        ev.observedLatencyP99Ms = lat[idx];
    }
    // loss rate
    if (!ps.deliveredWindow.empty())
    {
        std::size_t lost = 0;
        for (bool d : ps.deliveredWindow)
        {
            if (!d)
                lost += 1;
        }
        ev.observedLossRate = static_cast<double>(lost) / ps.deliveredWindow.size();
    }
    ev.latencyBreach = ev.observedLatencyP99Ms > ev.latencyBudgetMs;
    ev.reliabilityBreach = (1.0 - ev.observedLossRate) < ev.reliabilityTarget;
    return ev;
}

std::vector<BreachEvent>
SliceIsolationMonitor::EvaluateAll() const
{
    std::vector<BreachEvent> out;
    out.reserve(m_slices.size());
    for (const auto& [k, ps] : m_slices)
    {
        BreachEvent ev = EvaluateSlice(ps);
        if (ev.latencyBreach || ev.reliabilityBreach)
        {
            const_cast<BreachTrace&>(m_traceBreach)(ev);
        }
        out.push_back(ev);
    }
    return out;
}

std::size_t
SliceIsolationMonitor::BreachCount() const
{
    std::size_t n = 0;
    for (const auto& [k, ps] : m_slices)
    {
        BreachEvent ev = EvaluateSlice(ps);
        if (ev.latencyBreach || ev.reliabilityBreach)
            n += 1;
    }
    return n;
}

} // namespace ntnslice
} // namespace ns3
