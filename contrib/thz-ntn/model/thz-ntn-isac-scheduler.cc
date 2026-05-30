/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#include "thz-ntn-isac-scheduler.h"

#include "ns3/log.h"
#include "ns3/uinteger.h"

#include <algorithm>
#include <cmath>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("ThzNtnIsacScheduler");
NS_OBJECT_ENSURE_REGISTERED(ThzNtnIsacScheduler);

TypeId
ThzNtnIsacScheduler::GetTypeId()
{
    static TypeId tid = TypeId("ns3::ThzNtnIsacScheduler")
                            .SetParent<Object>()
                            .SetGroupName("ThzNtn")
                            .AddConstructor<ThzNtnIsacScheduler>()
                            .AddAttribute("JointSubbands",
                                          "Number of subbands carrying both "
                                          "comm + sense pilots simultaneously.",
                                          UintegerValue(0),
                                          MakeUintegerAccessor(
                                              &ThzNtnIsacScheduler::m_jointSubbands),
                                          MakeUintegerChecker<uint32_t>());
    return tid;
}

ThzNtnIsacScheduler::ThzNtnIsacScheduler() = default;
ThzNtnIsacScheduler::~ThzNtnIsacScheduler() = default;

double
ThzNtnIsacScheduler::ResolveSensingRatio(IsacMode mode)
{
    switch (mode)
    {
    case IsacMode::COMMUNICATION_ONLY:
        return 0.0;
    case IsacMode::COMMUNICATION_CENTRIC:
        return 0.2;
    case IsacMode::JOINT_ISAC:
        return 0.5;
    case IsacMode::SENSING_CENTRIC:
        return 0.8;
    case IsacMode::SENSING_ONLY:
        return 1.0;
    }
    return 0.5;
}

std::vector<uint32_t>
ThzNtnIsacScheduler::PickFront(std::vector<ThzNtnSubBand>& pool,
                                uint32_t count)
{
    std::vector<uint32_t> out;
    out.reserve(count);
    const uint32_t n = std::min<uint32_t>(count, pool.size());
    for (uint32_t i = 0; i < n; ++i)
    {
        out.push_back(pool[i].index);
    }
    pool.erase(pool.begin(), pool.begin() + n);
    return out;
}

ThzNtnIsacSchedulingDecision
ThzNtnIsacScheduler::Schedule(
    const std::vector<ThzNtnUeContext>& ues,
    const std::vector<ThzNtnSubBand>& availableSubBands)
{
    ThzNtnIsacSchedulingDecision out;
    out.numSubBandsTotal = static_cast<uint32_t>(availableSubBands.size());
    out.mode = m_isac ? m_isac->GetIsacMode() : IsacMode::JOINT_ISAC;
    out.sensingTimeShareRatio = ResolveSensingRatio(out.mode);

    if (availableSubBands.empty())
    {
        return out;
    }

    // Working copy — PickFront() consumes from the head.
    std::vector<ThzNtnSubBand> pool = availableSubBands;

    // Decide how many subbands go to sensing vs comm.
    const uint32_t total = static_cast<uint32_t>(pool.size());
    const uint32_t nSensing = static_cast<uint32_t>(
        std::round(out.sensingTimeShareRatio * total));
    const uint32_t nComm = total - nSensing;

    // Pull sensing share first so they come from the high-frequency end
    // (the existing scheduler typically prefers low-absorption subbands;
    // we leave those for comm).
    if (nSensing > 0)
    {
        // From the tail.
        for (uint32_t i = 0; i < nSensing; ++i)
        {
            out.sensingSubBandIndices.push_back(pool.back().index);
            pool.pop_back();
        }
        std::reverse(out.sensingSubBandIndices.begin(),
                      out.sensingSubBandIndices.end());
    }

    // Shared-overlay subbands: shave J from the comm pool (front-most).
    const uint32_t nShared = std::min(m_jointSubbands, nComm);
    if (nShared > 0)
    {
        out.sharedSubBandIndices = PickFront(pool, nShared);
        // Reinsert these into a temporary so they appear in both the
        // comm pool (so they get an MCS / TB) and the shared list.
        // We rebuild a SubBand entry for the scheduler's input.
    }
    // Remaining pool = comm-only subbands.
    out.commSubBandIndices.reserve(pool.size());
    for (const auto& s : pool)
    {
        out.commSubBandIndices.push_back(s.index);
    }

    // Reconstitute the comm subband pool (comm-only + shared overlay) and
    // hand it to the underlying scheduler.
    std::vector<ThzNtnSubBand> commPool;
    commPool.reserve(pool.size() + nShared);
    // Re-add the shared subbands (we lost the original SubBand structs;
    // synthesise lightweight ones whose index matches the original).
    for (uint32_t idx : out.sharedSubBandIndices)
    {
        for (const auto& src : availableSubBands)
        {
            if (src.index == idx)
            {
                commPool.push_back(src);
                break;
            }
        }
    }
    for (const auto& s : pool)
    {
        commPool.push_back(s);
    }

    if (m_comm && !commPool.empty() && !ues.empty() && nComm > 0)
    {
        out.commDecisions = m_comm->Schedule(ues, commPool);
    }
    return out;
}

} // namespace ns3
