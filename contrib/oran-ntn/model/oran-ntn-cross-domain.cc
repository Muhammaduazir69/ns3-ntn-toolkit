/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only
#include "oran-ntn-cross-domain.h"

#include <ns3/log.h>
#include <ns3/simulator.h>

#include <algorithm>
#include <cmath>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("OranNtnCrossDomain");
NS_OBJECT_ENSURE_REGISTERED(OranNtnNwdaf);
NS_OBJECT_ENSURE_REGISTERED(OranNtnTpnController);
NS_OBJECT_ENSURE_REGISTERED(OranNtnCrossDomainSmo);

// ------------------------------------------------------------------- NWDAF

TypeId
OranNtnNwdaf::GetTypeId()
{
    static TypeId tid = TypeId("ns3::OranNtnNwdaf")
                            .SetParent<Object>()
                            .SetGroupName("OranNtn")
                            .AddConstructor<OranNtnNwdaf>();
    return tid;
}

void
OranNtnNwdaf::ConsumeKpm(const NtnOranAiFlowMonitor::KpmIndication& ind)
{
    SliceState& st = m_slices[ind.key.sst];
    SliceAnalytics& a = st.a;
    a.sst = ind.key.sst;
    a.sd = ind.key.sd;

    auto metric = [&ind](const char* name, double fallback) {
        auto it = ind.measurements.find(name);
        return (it != ind.measurements.end()) ? it->second : fallback;
    };
    const double thp = metric("DRB.UEThpDl", 0.0);
    const double delay = metric("DRB.RlcSduDelayDl", a.meanDelayMs);
    const double loss = metric("DRB.PacketLossRateDl", a.lossRate);

    const double alpha = 0.3;
    a.loadMbps += alpha * (thp - a.loadMbps);
    a.meanDelayMs += alpha * (delay - a.meanDelayMs);
    a.lossRate += alpha * (loss - a.lossRate);

    const Time now = Simulator::Now();
    if (a.samples > 0 && now > st.prevTime)
    {
        const double slope =
            (delay - st.prevDelayMs) / (now - st.prevTime).GetSeconds();
        a.delaySlopeMsPerS += alpha * (slope - a.delaySlopeMsPerS);
    }
    st.prevDelayMs = delay;
    st.prevTime = now;
    ++a.samples;

    // SLA risk: how much of the delay budget is consumed now, plus where the
    // measured trend puts us one horizon (5 s) ahead. Loss adds directly.
    auto budgetIt = m_budgetMs.find(a.sst);
    const double budget = (budgetIt != m_budgetMs.end()) ? budgetIt->second : 100.0;
    const double projected = a.meanDelayMs + 5.0 * std::max(0.0, a.delaySlopeMsPerS);
    a.slaRisk = std::clamp(projected / std::max(budget, 1e-6) + a.lossRate, 0.0, 1.0);
}

OranNtnNwdaf::SliceAnalytics
OranNtnNwdaf::GetAnalytics(uint8_t sst) const
{
    auto it = m_slices.find(sst);
    return (it != m_slices.end()) ? it->second.a : SliceAnalytics{};
}

std::vector<OranNtnNwdaf::SliceAnalytics>
OranNtnNwdaf::GetAllAnalytics() const
{
    std::vector<SliceAnalytics> out;
    out.reserve(m_slices.size());
    for (const auto& [sst, st] : m_slices)
    {
        out.push_back(st.a);
    }
    return out;
}

// ---------------------------------------------------------- TPN controller

TypeId
OranNtnTpnController::GetTypeId()
{
    static TypeId tid = TypeId("ns3::OranNtnTpnController")
                            .SetParent<Object>()
                            .SetGroupName("OranNtn")
                            .AddConstructor<OranNtnTpnController>();
    return tid;
}

void
OranNtnTpnController::RegisterPath(std::string name,
                                   LatencyProvider latency,
                                   PathSwitcher activate)
{
    m_paths.push_back({std::move(name), std::move(latency), std::move(activate)});
}

std::string
OranNtnTpnController::SelectPathForSlice(uint8_t sst, double budgetMs)
{
    if (m_paths.empty())
    {
        return "";
    }
    const PathEntry* best = nullptr;
    double bestLat = 0.0;
    for (const auto& p : m_paths)
    {
        const double lat = p.latency();
        if (!best || lat < bestLat)
        {
            best = &p;
            bestLat = lat;
        }
    }
    const bool feasible = bestLat <= budgetMs;
    if (m_active[sst] != best->name)
    {
        best->activate(sst);
        m_active[sst] = best->name;
        ++m_switches;
    }
    return feasible ? best->name : "";
}

std::string
OranNtnTpnController::GetActivePath(uint8_t sst) const
{
    auto it = m_active.find(sst);
    return (it != m_active.end()) ? it->second : "";
}

// -------------------------------------------------------- cross-domain SMO

TypeId
OranNtnCrossDomainSmo::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::OranNtnCrossDomainSmo")
            .SetParent<Object>()
            .SetGroupName("OranNtn")
            .AddConstructor<OranNtnCrossDomainSmo>()
            .AddTraceSource("Decision",
                            "Cross-domain decision (sst, slaRisk, radioShare).",
                            MakeTraceSourceAccessor(&OranNtnCrossDomainSmo::m_decisionTrace),
                            "ns3::TracedValueCallback::Double");
    return tid;
}

void
OranNtnCrossDomainSmo::AddSlice(uint8_t sst,
                                double delayBudgetMs,
                                double baseRadio,
                                double baseCompute)
{
    m_slices.push_back({sst, delayBudgetMs, baseRadio, baseCompute});
    if (m_nwdaf)
    {
        m_nwdaf->SetSliceDelayBudget(sst, delayBudgetMs);
    }
}

void
OranNtnCrossDomainSmo::Start()
{
    if (m_running)
    {
        return;
    }
    m_running = true;
    Simulator::Schedule(m_period, &OranNtnCrossDomainSmo::Coordinate, this);
}

void
OranNtnCrossDomainSmo::Coordinate()
{
    if (!m_running)
    {
        return;
    }
    for (auto& s : m_slices)
    {
        const auto a = m_nwdaf ? m_nwdaf->GetAnalytics(s.sst)
                               : OranNtnNwdaf::SliceAnalytics{};
        // Risk-proportional adaptation around the baseline: a slice at risk
        // gets up to +50% radio and compute share; a relaxed slice decays
        // back toward its baseline. All inputs are measured analytics.
        const double target = std::clamp(s.radioShare * (1.0 + a.slaRisk * 0.5),
                                         0.05, 0.95);
        const double computeTarget =
            std::clamp(s.computeShare * (1.0 + a.slaRisk * 0.5), 0.05, 0.95);

        Decision d;
        d.time = Simulator::Now();
        d.sst = s.sst;
        d.slaRisk = a.slaRisk;
        d.radioShare = target;
        d.computeShare = computeTarget;
        if (m_radioCb)
        {
            m_radioCb(s.sst, target);
        }
        if (m_computeCb)
        {
            m_computeCb(s.sst, computeTarget);
        }
        if (m_tpn)
        {
            d.tpnPath = m_tpn->SelectPathForSlice(s.sst, s.budgetMs);
        }
        m_decisions.push_back(d);
        m_decisionTrace(s.sst, a.slaRisk, target);
    }
    Simulator::Schedule(m_period, &OranNtnCrossDomainSmo::Coordinate, this);
}

} // namespace ns3
