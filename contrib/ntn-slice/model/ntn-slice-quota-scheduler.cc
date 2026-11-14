/*
 * SPDX-License-Identifier: GPL-2.0-only
 * Copyright (c) 2026 Muhammad Uzair (ns3-ntn-toolkit)
 */
#include "ntn-slice-quota-scheduler.h"

#include "ns3/boolean.h"
#include "ns3/log.h"
#include "ns3/nr-mac-scheduler-ue-info.h"

#include <algorithm>
#include <cmath>
#include <numeric>

namespace ns3
{
namespace ntnslice
{

NS_LOG_COMPONENT_DEFINE("NtnSliceQuotaScheduler");
NS_OBJECT_ENSURE_REGISTERED(NtnSliceQuotaScheduler);

TypeId
NtnSliceQuotaScheduler::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::ntnslice::NtnSliceQuotaScheduler")
            .SetParent<NrMacSchedulerOfdmaPF>()
            .AddConstructor<NtnSliceQuotaScheduler>()
            .AddAttribute("WorkConserving",
                          "Let a slice exceed its quota when no slice below quota has demand",
                          BooleanValue(true),
                          MakeBooleanAccessor(&NtnSliceQuotaScheduler::m_workConserving),
                          MakeBooleanChecker())
            .AddTraceSource("SliceAllocation",
                            "Per slot and slice: slice index, RBGs given, RBGs in the slot",
                            MakeTraceSourceAccessor(&NtnSliceQuotaScheduler::m_sliceAlloc),
                            "ns3::TracedCallback::Uint32Uint32Uint32");
    return tid;
}

NtnSliceQuotaScheduler::NtnSliceQuotaScheduler()
    : NrMacSchedulerOfdmaPF()
{
}

void
NtnSliceQuotaScheduler::SetSliceOfRnti(uint16_t rnti, uint32_t slice)
{
    m_sliceOfRnti[rnti] = slice;
}

uint32_t
NtnSliceQuotaScheduler::GetSliceOfRnti(uint16_t rnti) const
{
    auto it = m_sliceOfRnti.find(rnti);
    return it == m_sliceOfRnti.end() ? 0 : it->second;
}

void
NtnSliceQuotaScheduler::SetQuotas(const std::vector<double>& shares)
{
    const double sum = std::accumulate(shares.begin(), shares.end(), 0.0);
    NS_ABORT_MSG_IF(sum > 1.0 + 1e-9, "slice quotas sum to " << sum << " > 1");
    for (double s : shares)
    {
        NS_ABORT_MSG_IF(s < 0, "negative slice quota");
    }
    m_quota = shares;
    if (m_rbgCount.size() < shares.size())
    {
        m_rbgCount.resize(shares.size(), 0);
    }
}

std::vector<double>
NtnSliceQuotaScheduler::GetQuotas() const
{
    return m_quota;
}

std::vector<uint64_t>
NtnSliceQuotaScheduler::TakeRbgCounts(uint64_t* slots)
{
    std::vector<uint64_t> out = m_rbgCount;
    if (slots)
    {
        *slots = m_slots;
    }
    std::fill(m_rbgCount.begin(), m_rbgCount.end(), 0);
    m_slots = 0;
    return out;
}

std::vector<double>
NtnSliceQuotaScheduler::TakeBacklog()
{
    std::vector<double> out(m_backlogSum.size(), 0.0);
    for (size_t s = 0; s < out.size(); ++s)
    {
        out[s] = m_backlogSlots ? m_backlogSum[s] / m_backlogSlots : 0.0;
    }
    std::fill(m_backlogSum.begin(), m_backlogSum.end(), 0.0);
    m_backlogSlots = 0;
    return out;
}

std::vector<double>
NtnSliceQuotaScheduler::TakeBytesPerRbg()
{
    std::vector<double> out(m_tbBytes.size(), 0.0);
    for (size_t s = 0; s < out.size(); ++s)
    {
        out[s] = (s < m_rbgCount.size() && m_rbgCount[s]) ? m_tbBytes[s] / m_rbgCount[s] : 0.0;
    }
    std::fill(m_tbBytes.begin(), m_tbBytes.end(), 0.0);
    return out;
}

NrMacSchedulerNs3::BeamSymbolMap
NtnSliceQuotaScheduler::AssignDLRBG(uint32_t symAvail, const ActiveUeMap& activeDl) const
{
    NS_LOG_FUNCTION(this);
    NS_ABORT_MSG_IF(m_nrFhSchedSapProvider, "fronthaul control is not supported by the slice-quota scheduler");
    GetFirst GetBeamId;
    GetSecond GetUeVector;
    GetFirst GetUe;
    BeamSymbolMap symPerBeam = GetSymPerBeam(symAvail, activeDl);

    for (const auto& el : activeDl)
    {
        const uint32_t beamSym = symPerBeam.at(GetBeamId(el));
        const uint32_t rbgAssignable = beamSym;
        FTResources assigned(0, 0);
        const std::vector<uint8_t> mask = GetDlNotchedRbgMask();
        uint32_t resources = !mask.empty() ? std::count(mask.begin(), mask.end(), 1) : GetBandwidthInRbg();
        const uint32_t total = resources;
        std::vector<UePtrAndBufferReq> ueVector(GetUeVector(el).begin(), GetUeVector(el).end());
        for (auto& ue : ueVector)
        {
            BeforeDlSched(ue, FTResources(rbgAssignable, beamSym));
        }
        const size_t nSlices = std::max<size_t>(m_quota.size(), 1);
        std::vector<uint32_t> limit(nSlices, total), used(nSlices, 0);
        if (!m_quota.empty())
        {
            for (size_t s = 0; s < nSlices; ++s)
            {
                limit[s] = static_cast<uint32_t>(std::floor(m_quota[s] * total + 1e-9));
            }
        }
        auto sliceOf = [this](const UePtrAndBufferReq& u) {
            const uint32_t s = GetSliceOfRnti(u.first->m_rnti);
            return s < m_quota.size() ? s : 0;
        };
        if (m_backlogSum.size() < nSlices)
        {
            m_backlogSum.resize(nSlices, 0.0);
            m_tbBytes.resize(nSlices, 0.0);
        }
        for (const auto& u : ueVector)
        {
            m_backlogSum[sliceOf(u)] += u.second;
        }
        ++m_backlogSlots;
        auto needs = [&GetUe](const UePtrAndBufferReq& u) {
            return GetUe(u)->m_dlTbSize < std::max(u.second, 10U);
        };

        while (resources > 0)
        {
            std::stable_sort(ueVector.begin(), ueVector.end(), GetUeCompareDlFn());
            auto it = std::find_if(ueVector.begin(), ueVector.end(), [&](const UePtrAndBufferReq& u) {
                return needs(u) && used[sliceOf(u)] < limit[sliceOf(u)];
            });
            if (it == ueVector.end() && m_workConserving)
            {
                it = std::find_if(ueVector.begin(), ueVector.end(), needs);
            }
            if (it == ueVector.end())
            {
                break;
            }
            const uint32_t s = sliceOf(*it);
            do
            {
                GetUe(*it)->m_dlRBG += rbgAssignable;
                assigned.m_rbg += rbgAssignable;
                GetUe(*it)->m_dlSym = beamSym;
                assigned.m_sym = beamSym;
                resources -= 1;
                used[s] += 1;
                AssignedDlResources(*it, FTResources(rbgAssignable, beamSym), assigned);
            } while (GetUe(*it)->m_dlTbSize < 10 && resources > 0); // the minimum TB, as the base scheduler
            for (auto& ue : ueVector)
            {
                if (GetUe(ue)->m_rnti != GetUe(*it)->m_rnti)
                {
                    NotAssignedDlResources(ue, FTResources(rbgAssignable, beamSym), assigned);
                }
            }
        }
        if (m_rbgCount.size() < nSlices)
        {
            m_rbgCount.resize(nSlices, 0);
        }
        for (const auto& u : ueVector)
        {
            m_tbBytes[sliceOf(u)] += GetUe(u)->m_dlTbSize;
        }
        for (size_t s = 0; s < nSlices; ++s)
        {
            m_rbgCount[s] += used[s];
            m_sliceAlloc(static_cast<uint32_t>(s), used[s], total);
        }
        ++m_slots;
    }
    return symPerBeam;
}

} // namespace ntnslice
} // namespace ns3
