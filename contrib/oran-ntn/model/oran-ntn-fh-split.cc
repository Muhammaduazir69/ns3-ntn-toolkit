/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only
#include "oran-ntn-fh-split.h"

#include <ns3/abort.h>
#include <ns3/log.h>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("NtnFhSplitModel");
NS_OBJECT_ENSURE_REGISTERED(NtnFhSplitModel);

TypeId
NtnFhSplitModel::GetTypeId()
{
    static TypeId tid = TypeId("ns3::NtnFhSplitModel")
                            .SetParent<Object>()
                            .SetGroupName("OranNtn")
                            .AddConstructor<NtnFhSplitModel>();
    return tid;
}

const std::vector<NtnFhSplitModel::SplitSpec>&
NtnFhSplitModel::GetAllSpecs()
{
    // TR 38.801 Annex A latency classes; O-RAN WG4 CUS-plane 0.25 ms for
    // lower-PHY splits; rate multipliers per the split-option literature.
    static const std::vector<SplitSpec> specs = {
        {Split::Opt2, "Option-2 (F1, PDCP/RLC)", MilliSeconds(10), 1.05},
        {Split::Opt7_2a, "Option-7.2a (low-PHY IQ)", MicroSeconds(250), 10.0},
        {Split::Opt7_2b, "Option-7.2b (beamformed IQ)", MicroSeconds(250), 7.0},
        {Split::Opt8, "Option-8 (time-domain IQ)", MicroSeconds(250), 16.0},
    };
    return specs;
}

const NtnFhSplitModel::SplitSpec&
NtnFhSplitModel::GetSpec(Split s)
{
    for (const auto& spec : GetAllSpecs())
    {
        if (spec.split == s)
        {
            return spec;
        }
    }
    NS_ABORT_MSG("unknown split option");
}

bool
NtnFhSplitModel::IsFeasible(Split s,
                            Time measuredFhOneWay,
                            double userPlaneMbps,
                            double fhCapacityMbps)
{
    const SplitSpec& spec = GetSpec(s);
    return measuredFhOneWay <= spec.maxOneWayLatency &&
           userPlaneMbps * spec.fhRateMultiplier <= fhCapacityMbps;
}

NtnFhSplitModel::Split
NtnFhSplitModel::ChooseBestSplit(Time measuredFhOneWay,
                                 double userPlaneMbps,
                                 double fhCapacityMbps,
                                 bool& ok)
{
    // Centralization preference: the more functions pulled off the platform
    // the better (paper Sec. IV-B), so try Opt8 first.
    static const Split order[] = {Split::Opt8, Split::Opt7_2b, Split::Opt7_2a, Split::Opt2};
    for (Split s : order)
    {
        if (IsFeasible(s, measuredFhOneWay, userPlaneMbps, fhCapacityMbps))
        {
            ok = true;
            return s;
        }
    }
    ok = false;
    return Split::Opt2;
}

} // namespace ns3
