/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only
#include "oran-ntn-authority-lease.h"

#include <ns3/log.h>

#include <algorithm>
#include <cmath>
#include <set>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("OranNtnAuthorityLease");
NS_OBJECT_ENSURE_REGISTERED(OranNtnAuthorityLease);

TypeId
OranNtnAuthorityLease::GetTypeId()
{
    static TypeId tid = TypeId("ns3::OranNtnAuthorityLease")
                            .SetParent<Object>()
                            .SetGroupName("OranNtn")
                            .AddConstructor<OranNtnAuthorityLease>();
    return tid;
}

OranNtnAuthorityLease::Host
OranNtnAuthorityLease::Place(Time predictedGroundLoop,
                             Time deadline,
                             double slackFrac,
                             Time remainingContact,
                             Time minContact)
{
    const double budget = deadline.GetSeconds() * (1.0 - std::clamp(slackFrac, 0.0, 1.0));
    const bool fits = predictedGroundLoop.GetSeconds() <= budget;
    const bool lastsLongEnough = remainingContact >= minContact;
    return (fits && lastsLongEnough) ? Host::Ground : Host::Onboard;
}

Time
OranNtnAuthorityLease::LeaseHorizon(Time meanUpdate, double eps)
{
    if (eps <= 0.0)
    {
        return Time(0);
    }
    if (eps >= 1.0)
    {
        return Time::Max();
    }
    return Seconds(-meanUpdate.GetSeconds() * std::log1p(-eps));
}

Time
OranNtnAuthorityLease::LeaseExpiry(Time now, Time predictedRestore, Time margin, Time maxLease)
{
    Time cap = now + maxLease;
    if (predictedRestore == Time::Max())
    {
        return cap;
    }
    Time e = predictedRestore + margin;
    return std::max(now, std::min(e, cap));
}

OranNtnAuthorityLease::MergeResult
OranNtnAuthorityLease::Merge(MergeRule rule,
                             const std::vector<Write>& ground,
                             const std::vector<Write>& onboard,
                             Time leaseStart,
                             Time leaseEnd,
                             uint32_t currentVersion,
                             double tol)
{
    MergeResult r;
    std::set<uint32_t> params;
    for (const auto& w : ground)
    {
        params.insert(w.paramId);
    }
    for (const auto& w : onboard)
    {
        params.insert(w.paramId);
    }

    auto latest = [](const std::vector<Write>& log, uint32_t p, auto pred) -> const Write* {
        const Write* best = nullptr;
        for (const auto& w : log)
        {
            if (w.paramId == p && pred(w) && (!best || w.at >= best->at))
            {
                best = &w;
            }
        }
        return best;
    };
    auto any = [](const Write&) { return true; };
    auto inLease = [&](const Write& w) { return w.at >= leaseStart && w.at < leaseEnd; };

    for (uint32_t p : params)
    {
        const Write* g = latest(ground, p, [&](const Write& w) { return w.at >= leaseStart; });
        const Write* o = latest(onboard, p, inLease);
        // A direct conflict is a disagreement under the SAME intent version:
        // two writes made under different versions are expected to differ and
        // are handled as policy-stale writes instead.
        if (g && o && g->policyVersion == o->policyVersion &&
            std::fabs(g->value - o->value) > tol)
        {
            ++r.directConflicts;
        }

        const Write* win = nullptr;
        if (rule == MergeRule::GroundWins)
        {
            win = latest(ground, p, any);
            if (!win)
            {
                win = latest(onboard, p, any);
            }
        }
        else
        {
            // Authoritative = written by the side that held the lease then.
            const Write* ga =
                latest(ground, p, [&](const Write& w) { return !inLease(w); });
            const Write* oa = latest(onboard, p, inLease);
            if (ga && oa)
            {
                win = (ga->at >= oa->at) ? ga : oa;
            }
            else
            {
                win = ga ? ga : oa;
            }
            if (!win)
            {
                // Nothing authoritative: fall back to the latest of anything.
                const Write* gl = latest(ground, p, any);
                const Write* ol = latest(onboard, p, any);
                win = (gl && ol) ? (gl->at >= ol->at ? gl : ol) : (gl ? gl : ol);
            }
        }
        if (!win)
        {
            continue;
        }
        r.state[p] = win->value;
        r.winner[p] = win->writer;
        if (win->policyVersion < currentVersion)
        {
            // A write under the safe intent (version 0) is a deliberate
            // fallback after a lapsed lease, not a superseded intent; it is
            // counted apart but still recomputed under the current intent.
            if (win->policyVersion == 0)
            {
                ++r.fallbackWrites;
            }
            else
            {
                ++r.policyStaleWrites;
            }
            if (rule == MergeRule::LeaseVersioned)
            {
                r.recompute.push_back(p);
            }
        }
    }
    return r;
}

} // namespace ns3
