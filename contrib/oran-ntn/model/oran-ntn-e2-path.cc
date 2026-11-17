/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#include "oran-ntn-e2-path.h"

#include <ns3/log.h>
#include <ns3/simulator.h>

#include <algorithm>
#include <cmath>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("OranNtnE2Path");
NS_OBJECT_ENSURE_REGISTERED(OranNtnE2Path);

namespace
{
constexpr double kC = 299792458.0;
}

TypeId
OranNtnE2Path::GetTypeId()
{
    static TypeId tid = TypeId("ns3::OranNtnE2Path")
                            .SetParent<Object>()
                            .SetGroupName("OranNtn")
                            .AddConstructor<OranNtnE2Path>();
    return tid;
}

const char*
OranNtnE2Path::KindName(HopKind k)
{
    switch (k)
    {
    case HopKind::SERVICE_LINK:
        return "service";
    case HopKind::FEEDER_LINK:
        return "feeder";
    case HopKind::ISL:
        return "isl";
    case HopKind::TERRESTRIAL:
        return "terrestrial";
    case HopKind::PROCESSING:
        return "processing";
    }
    return "unknown";
}

void
OranNtnE2Path::AddGeometricHop(const std::string& name,
                               HopKind kind,
                               Ptr<MobilityModel> a,
                               Ptr<MobilityModel> b)
{
    NS_ABORT_MSG_IF(!a || !b, "a geometric E2 hop needs both mobility models");
    m_hops.push_back({name, kind, a, b, Time()});
    m_stats.push_back({name, kind});
}

void
OranNtnE2Path::AddFixedHop(const std::string& name, HopKind kind, Time delay)
{
    NS_ABORT_MSG_IF(delay < Time(), "negative hop delay");
    m_hops.push_back({name, kind, nullptr, nullptr, delay});
    m_stats.push_back({name, kind});
}

Time
OranNtnE2Path::ComputeOneWay(std::vector<HopSample>* out) const
{
    Time total;
    if (out)
    {
        out->clear();
    }
    for (const auto& h : m_hops)
    {
        double rangeKm = std::nan("");
        Time d = h.fixed;
        if (h.a)
        {
            const double m = h.a->GetDistanceFrom(h.b);
            rangeKm = m / 1e3;
            d = Seconds(m / kC);
        }
        total += d;
        if (out)
        {
            out->push_back({h.name, h.kind, rangeKm, d});
        }
    }
    return total;
}

void
OranNtnE2Path::BindToE2Node(Ptr<OranNtnE2Node> e2, Time refresh)
{
    NS_ABORT_MSG_IF(!e2, "null E2 node");
    NS_ABORT_MSG_IF(m_hops.empty(), "an E2 path needs at least one hop");
    NS_ABORT_MSG_IF(refresh <= Time(), "refresh must be positive");
    m_e2 = e2;
    m_refresh = refresh;
    m_event.Cancel();
    Refresh();
}

void
OranNtnE2Path::Unbind()
{
    m_event.Cancel();
    m_e2 = nullptr;
}

void
OranNtnE2Path::Refresh()
{
    std::vector<HopSample> s;
    const Time total = ComputeOneWay(&s);
    for (size_t i = 0; i < s.size(); ++i)
    {
        HopStats& st = m_stats[i];
        const double ms = s[i].delay.GetSeconds() * 1e3;
        st.minMs = st.n ? std::min(st.minMs, ms) : ms;
        st.maxMs = st.n ? std::max(st.maxMs, ms) : ms;
        st.sumMs += ms;
        st.sumRangeKm += std::isfinite(s[i].rangeKm) ? s[i].rangeKm : 0.0;
        ++st.n;
    }
    m_sumTotalMs += total.GetSeconds() * 1e3;
    ++m_nTotal;
    if (m_e2)
    {
        m_e2->SetFeederLinkDelay(total);
        m_event = Simulator::Schedule(m_refresh, &OranNtnE2Path::Refresh, this);
    }
}

double
OranNtnE2Path::GetMeanOneWayMs() const
{
    return m_nTotal ? m_sumTotalMs / m_nTotal : 0.0;
}

void
OranNtnE2Path::WriteLoopBudgetCsv(std::ostream& os, double ricWaitMeanMs) const
{
    os << "hop,kind,range_km_mean,one_way_ms_mean,one_way_ms_min,one_way_ms_max,loop_ms_mean\n";
    double loop = 0.0;
    for (const auto& st : m_stats)
    {
        const bool geo = st.kind != HopKind::TERRESTRIAL && st.kind != HopKind::PROCESSING;
        os << st.name << "," << KindName(st.kind) << ",";
        if (geo)
        {
            os << st.MeanRangeKm();
        }
        os << "," << st.MeanMs() << "," << st.minMs << "," << st.maxMs << ","
           << 2.0 * st.MeanMs() << "\n";
        loop += 2.0 * st.MeanMs();
    }
    os << "ric_tick_wait,ric,," << ricWaitMeanMs << ",,," << ricWaitMeanMs << "\n";
    os << "loop_total,total,,,,," << loop + ricWaitMeanMs << "\n";
}

void
OranNtnE2Path::DoDispose()
{
    m_event.Cancel();
    m_e2 = nullptr;
    m_hops.clear();
    Object::DoDispose();
}

} // namespace ns3
