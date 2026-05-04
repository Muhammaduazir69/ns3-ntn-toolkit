/*
 * SPDX-License-Identifier: GPL-2.0-only
 * Copyright (c) 2026 Muhammad Uzair (ns3-ntn-toolkit, Workstream W6)
 */
#include "ntn-slice-selector.h"

namespace ns3
{
namespace ntnslice
{

NS_OBJECT_ENSURE_REGISTERED(NtnSliceSelector);

TypeId
NtnSliceSelector::GetTypeId()
{
    static TypeId tid = TypeId("ns3::ntnslice::NtnSliceSelector")
                           .SetParent<Object>()
                           .SetGroupName("NtnSlice")
                           .AddConstructor<NtnSliceSelector>();
    return tid;
}

NtnSliceSelector::NtnSliceSelector() = default;

void
NtnSliceSelector::SetDefault(const Snssai& s)
{
    m_default = s;
}

Snssai
NtnSliceSelector::GetDefault() const
{
    return m_default;
}

void
NtnSliceSelector::AddRule(const Rule& rule)
{
    m_rules.push_back(rule);
}

std::size_t
NtnSliceSelector::RuleCount() const
{
    return m_rules.size();
}

void
NtnSliceSelector::AddDscpRule(uint8_t dscp, const Snssai& s)
{
    m_rules.push_back(Rule{RuleType::ByDscp, dscp, 0, 0, "", s});
}

void
NtnSliceSelector::AddPortRangeRule(uint16_t lo, uint16_t hi, const Snssai& s)
{
    m_rules.push_back(Rule{RuleType::ByDstPortRange, 0, lo, hi, "", s});
}

void
NtnSliceSelector::AddAppLabelRule(const std::string& label, const Snssai& s)
{
    m_rules.push_back(Rule{RuleType::ByAppLabel, 0, 0, 0, label, s});
}

Snssai
NtnSliceSelector::Match(const FlowDescriptor& f) const
{
    for (const auto& r : m_rules)
    {
        switch (r.type)
        {
        case RuleType::ByDscp:
            if (f.dscp == r.dscp)
                return r.snssai;
            break;
        case RuleType::ByDstPortRange:
            if (f.dstPort >= r.portLo && f.dstPort <= r.portHi)
                return r.snssai;
            break;
        case RuleType::ByAppLabel:
            if (!r.appLabel.empty() && r.appLabel == f.appLabel)
                return r.snssai;
            break;
        }
    }
    return m_default;
}

} // namespace ntnslice
} // namespace ns3
