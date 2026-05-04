/*
 * SPDX-License-Identifier: GPL-2.0-only
 * Copyright (c) 2026 Muhammad Uzair (ns3-ntn-toolkit, Workstream W6)
 */
#include "slice-orchestrator-xapp.h"

#include "ns3/log.h"
#include "ns3/uinteger.h"

#include <algorithm>
#include <cmath>

namespace ns3
{
namespace ntnslice
{

NS_LOG_COMPONENT_DEFINE("SliceOrchestratorXapp");
NS_OBJECT_ENSURE_REGISTERED(SliceOrchestratorXapp);

TypeId
SliceOrchestratorXapp::GetTypeId()
{
    static TypeId tid = TypeId("ns3::ntnslice::SliceOrchestratorXapp")
                           .SetParent<Object>()
                           .SetGroupName("NtnSlice")
                           .AddConstructor<SliceOrchestratorXapp>()
                           .AddTraceSource("Tick",
                                           "Per-slice allocation tick",
                                           MakeTraceSourceAccessor(&SliceOrchestratorXapp::m_traceTick),
                                           "ns3::ntnslice::SliceOrchestratorXapp::SliceTickTrace");
    return tid;
}

SliceOrchestratorXapp::SliceOrchestratorXapp() = default;

void
SliceOrchestratorXapp::RegisterSlice(const SliceProfile& profile)
{
    m_slices.push_back(profile);
}

void
SliceOrchestratorXapp::SetTotalPrb(uint32_t prb)
{
    m_totalPrb = prb;
}

uint32_t
SliceOrchestratorXapp::GetTotalPrb() const
{
    return m_totalPrb;
}

void
SliceOrchestratorXapp::SetPrbCapacityMbps(double mbpsPerPrb)
{
    m_mbpsPerPrb = mbpsPerPrb;
}

void
SliceOrchestratorXapp::SetDemand(const Snssai& s, double mbps)
{
    m_demand[s] = std::max(0.0, mbps);
}

double
SliceOrchestratorXapp::GetDemand(const Snssai& s) const
{
    auto it = m_demand.find(s);
    return (it != m_demand.end()) ? it->second : 0.0;
}

std::vector<SliceTick>
SliceOrchestratorXapp::Step()
{
    return Allocate({}, false);
}

std::vector<SliceTick>
SliceOrchestratorXapp::StepWithShares(const std::map<Snssai, double>& shares)
{
    return Allocate(shares, true);
}

double
SliceOrchestratorXapp::GetAggregateSatisfaction() const
{
    return m_lastAggregateSat;
}

std::size_t
SliceOrchestratorXapp::SliceCount() const
{
    return m_slices.size();
}

std::vector<SliceTick>
SliceOrchestratorXapp::Allocate(const std::map<Snssai, double>& sharesOpt,
                                bool useShares)
{
    std::vector<SliceTick> ticks;
    ticks.reserve(m_slices.size());
    if (m_slices.empty() || m_totalPrb == 0)
    {
        m_lastAggregateSat = 0.0;
        return ticks;
    }

    const double totalCapacity = m_totalPrb * m_mbpsPerPrb;

    // Step 1: reserve PRBs to satisfy each slice's minThroughputMbps.
    std::vector<double> reservedMbps(m_slices.size(), 0.0);
    double reservedSum = 0.0;
    for (std::size_t i = 0; i < m_slices.size(); ++i)
    {
        // Reserve enough to meet min, but never more than demand or capacity.
        double demand = GetDemand(m_slices[i].snssai);
        double minRes = std::min({m_slices[i].minThroughputMbps,
                                  demand,
                                  totalCapacity - reservedSum});
        minRes = std::max(0.0, minRes);
        reservedMbps[i] = minRes;
        reservedSum += minRes;
    }

    // Step 2: distribute the remainder. Either by external shares or by
    // priority-weighted unmet demand.
    double remainder = std::max(0.0, totalCapacity - reservedSum);
    std::vector<double> extraMbps(m_slices.size(), 0.0);

    if (useShares && !sharesOpt.empty())
    {
        // Renormalise external shares.
        double sum = 0.0;
        for (auto& [k, v] : sharesOpt)
        {
            sum += std::max(0.0, v);
        }
        if (sum > 1e-9)
        {
            for (std::size_t i = 0; i < m_slices.size(); ++i)
            {
                auto it = sharesOpt.find(m_slices[i].snssai);
                if (it != sharesOpt.end())
                {
                    double frac = std::max(0.0, it->second) / sum;
                    extraMbps[i] = remainder * frac;
                }
            }
        }
    }
    else
    {
        // Built-in policy: priority × unmet-demand weighted.
        std::vector<double> w(m_slices.size(), 0.0);
        double totalW = 0.0;
        for (std::size_t i = 0; i < m_slices.size(); ++i)
        {
            double demand = GetDemand(m_slices[i].snssai);
            double unmet = std::max(0.0, demand - reservedMbps[i]);
            w[i] = unmet * (1.0 + m_slices[i].priority);
            totalW += w[i];
        }
        if (totalW > 1e-9)
        {
            for (std::size_t i = 0; i < m_slices.size(); ++i)
            {
                extraMbps[i] = remainder * (w[i] / totalW);
            }
        }
    }

    // Step 3: clamp by demand and by maxThroughputMbps cap.
    double satSum = 0.0;
    int satCount = 0;
    for (std::size_t i = 0; i < m_slices.size(); ++i)
    {
        SliceTick t{};
        t.snssai = m_slices[i].snssai;
        t.demandMbps = GetDemand(m_slices[i].snssai);
        double servedMbps = reservedMbps[i] + extraMbps[i];
        // demand cap
        servedMbps = std::min(servedMbps, t.demandMbps);
        // max-throughput cap (0 means uncapped)
        if (m_slices[i].maxThroughputMbps > 0.0)
        {
            servedMbps = std::min(servedMbps, m_slices[i].maxThroughputMbps);
        }
        t.servedMbps = servedMbps;
        t.prbAllocated = static_cast<uint32_t>(
            std::min(static_cast<double>(m_totalPrb),
                     std::ceil(servedMbps / std::max(m_mbpsPerPrb, 1e-9))));
        t.satisfactionRatio = (t.demandMbps > 1e-9)
                                  ? std::min(1.0, t.servedMbps / t.demandMbps)
                                  : 1.0;
        t.minThroughputMet = t.servedMbps + 1e-6 >= m_slices[i].minThroughputMbps;
        m_traceTick(t);
        ticks.push_back(t);
        satSum += t.satisfactionRatio;
        satCount += 1;
    }
    m_lastAggregateSat = (satCount > 0) ? satSum / satCount : 0.0;
    return ticks;
}

} // namespace ntnslice
} // namespace ns3
