/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#include "oran-ntn-a1-policy-schema.h"

namespace ns3
{
namespace oranntn
{
namespace a1
{

const char*
PolicyTypeIdName(PolicyTypeId id)
{
    switch (id)
    {
    case PolicyTypeId::OscTrafficSteering:     return "traffic-steering";
    case PolicyTypeId::OscQualityOfExperience: return "quality-of-experience";
    case PolicyTypeId::OscHandoverControl:     return "handover-control";
    case PolicyTypeId::OscMlModelSelection:    return "ml-model-selection";
    case PolicyTypeId::NtnBeamPriority:        return "ntn-beam-priority";
    }
    return "unknown";
}

const std::vector<A1PolicySchema>&
Registry()
{
    static const std::vector<A1PolicySchema> registry = {
        {PolicyTypeId::OscTrafficSteering,
         "traffic-steering",
         "contrib/oran-ntn/a1-policies/20000-traffic-steering.json",
         true},
        {PolicyTypeId::OscQualityOfExperience,
         "quality-of-experience",
         "contrib/oran-ntn/a1-policies/20001-quality-of-experience.json",
         true},
        {PolicyTypeId::OscHandoverControl,
         "handover-control",
         "contrib/oran-ntn/a1-policies/20008-handover-control.json",
         true},
        {PolicyTypeId::OscMlModelSelection,
         "ml-model-selection",
         "contrib/oran-ntn/a1-policies/20020-ml-model-selection.json",
         true},
        {PolicyTypeId::NtnBeamPriority,
         "ntn-beam-priority",
         "contrib/oran-ntn/a1-policies/20050-ntn-beam-priority.json",
         false},
    };
    return registry;
}

const A1PolicySchema*
Lookup(PolicyTypeId id)
{
    for (const auto& s : Registry())
    {
        if (s.type_id == id)
        {
            return &s;
        }
    }
    return nullptr;
}

uint32_t
OscIdForLocalType(A1PolicyType local)
{
    switch (local)
    {
    case A1PolicyType::HO_THRESHOLD:
        return static_cast<uint32_t>(PolicyTypeId::OscHandoverControl);
    case A1PolicyType::TN_NTN_STEERING:
        return static_cast<uint32_t>(PolicyTypeId::OscTrafficSteering);
    case A1PolicyType::SLICE_SLA:
        return static_cast<uint32_t>(PolicyTypeId::OscQualityOfExperience);
    case A1PolicyType::BEAM_PRIORITY:
        return static_cast<uint32_t>(PolicyTypeId::NtnBeamPriority);
    // The remaining toolkit-local types do not yet have an OSC mapping;
    // 4.1.11 follow-up work will either register more OSC IDs or add
    // toolkit-reserved (20050+) entries.
    case A1PolicyType::LOAD_BALANCE:
    case A1PolicyType::ENERGY_SAVING:
    case A1PolicyType::INTERFERENCE_LIMIT:
    case A1PolicyType::DC_PREFERENCE:
    case A1PolicyType::FL_PARTICIPATION:
    case A1PolicyType::MODCOD_RANGE:
    case A1PolicyType::PREDICTIVE_ALLOC:
    default:
        return 0;
    }
}

} // namespace a1
} // namespace oranntn
} // namespace ns3
