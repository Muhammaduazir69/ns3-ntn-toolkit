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
    const std::size_t n = m_slices.size();
    // What a slice can usefully take: its demand, under its throughput cap.
    std::vector<double> ceilMbps(n, 0.0);
    for (std::size_t i = 0; i < n; ++i)
    {
        ceilMbps[i] = GetDemand(m_slices[i].snssai);
        if (m_slices[i].maxThroughputMbps > 0.0)
        {
            ceilMbps[i] = std::min(ceilMbps[i], m_slices[i].maxThroughputMbps);
        }
    }

    // Step 1: reserve each slice's minimum, HIGHEST PRIORITY FIRST. The
    // reservation used to run in registration order, so when the minimums did
    // not all fit, whichever slice was registered first kept its guarantee and
    // a priority-9 URLLC slice registered after eMBB could lose its own.
    std::vector<std::size_t> order(n);
    for (std::size_t i = 0; i < n; ++i)
    {
        order[i] = i;
    }
    std::stable_sort(order.begin(), order.end(), [this](std::size_t a, std::size_t b) {
        return m_slices[a].priority > m_slices[b].priority;
    });
    std::vector<double> alloc(n, 0.0);
    double reservedSum = 0.0;
    for (std::size_t i : order)
    {
        const double minRes = std::max(
            0.0,
            std::min({m_slices[i].minThroughputMbps, ceilMbps[i], totalCapacity - reservedSum}));
        alloc[i] = minRes;
        reservedSum += minRes;
    }

    // Step 2: distribute the remainder by external shares or by priority x
    // unmet demand, as water-filling: capacity a slice cannot use (its demand
    // or cap is reached) is handed to the others in the next round instead of
    // being dropped. The single pass this replaces discarded it, so a capped
    // eMBB slice left PRBs idle while URLLC still had unmet demand.
    double remainder = std::max(0.0, totalCapacity - reservedSum);
    const bool shares = useShares && !sharesOpt.empty();
    for (int round = 0; round < 16 && remainder > 1e-9; ++round)
    {
        std::vector<double> w(n, 0.0);
        double totalW = 0.0;
        for (std::size_t i = 0; i < n; ++i)
        {
            const double head = ceilMbps[i] - alloc[i];
            if (head <= 1e-12)
            {
                continue;
            }
            if (shares)
            {
                auto it = sharesOpt.find(m_slices[i].snssai);
                w[i] = (it != sharesOpt.end()) ? std::max(0.0, it->second) : 0.0;
            }
            else
            {
                w[i] = head * (1.0 + m_slices[i].priority);
            }
            totalW += w[i];
        }
        if (totalW <= 1e-12)
        {
            break;
        }
        double given = 0.0;
        for (std::size_t i = 0; i < n; ++i)
        {
            if (w[i] <= 0.0)
            {
                continue;
            }
            const double add = std::min(remainder * (w[i] / totalW), ceilMbps[i] - alloc[i]);
            alloc[i] += add;
            given += add;
        }
        remainder -= given;
        if (given <= 1e-12)
        {
            break;
        }
    }

    // Step 3: PRBs. Each slice's PRB need was rounded up on its own, so the
    // slices' PRBs could add up to more than the cell has. Largest-remainder
    // rounding keeps the sum at or below m_totalPrb.
    std::vector<uint32_t> prb(n, 0);
    std::vector<std::pair<double, std::size_t>> frac;
    uint32_t prbSum = 0;
    for (std::size_t i = 0; i < n; ++i)
    {
        const double exact = alloc[i] / std::max(m_mbpsPerPrb, 1e-9);
        prb[i] = static_cast<uint32_t>(std::floor(exact + 1e-9));
        prbSum += prb[i];
        frac.emplace_back(exact - prb[i], i);
    }
    std::stable_sort(frac.begin(), frac.end(), [](const auto& a, const auto& b) {
        return a.first > b.first;
    });
    for (const auto& [f, i] : frac)
    {
        if (f > 1e-9 && prbSum < m_totalPrb)
        {
            ++prb[i];
            ++prbSum;
        }
    }

    double satSum = 0.0;
    int satCount = 0;
    for (std::size_t i = 0; i < n; ++i)
    {
        SliceTick t{};
        t.snssai = m_slices[i].snssai;
        t.demandMbps = GetDemand(m_slices[i].snssai);
        t.servedMbps = alloc[i];
        t.prbAllocated = prb[i];
        t.satisfactionRatio = (t.demandMbps > 1e-9)
                                  ? std::min(1.0, t.servedMbps / t.demandMbps)
                                  : 1.0;
        // The guarantee is min(minimum, demand, cap): a slice asking for less
        // than its minimum and getting all of it has had its guarantee met. It
        // was reported as a violation.
        t.minThroughputMet =
            t.servedMbps + 1e-6 >= std::min(m_slices[i].minThroughputMbps, ceilMbps[i]);
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
