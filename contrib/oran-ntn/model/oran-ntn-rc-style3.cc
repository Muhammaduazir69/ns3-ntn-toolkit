/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#include "oran-ntn-rc-style3.h"

namespace ns3
{
namespace oranntn
{
namespace rc_v103
{
namespace style3
{

namespace
{

/// Pack a gNB ID (upper 28 bits) with a default cell ID (0) into the 36-bit
/// NR Cell Identity. The mmwave gNB IDs used in the toolkit fit in 28 bits.
uint64_t
NrCellIdentityFromGnb(uint32_t gnbId)
{
    return (static_cast<uint64_t>(gnbId) & 0x0FFFFFFFULL) << 8;
}

} // namespace

std::optional<ControlAction>
ConvertE2RcToStyle3(const E2RcAction& action, const std::string& plmn_id)
{
    NrCellGlobalId target{};
    target.plmn_id = plmn_id;
    target.nr_cell_identity = NrCellIdentityFromGnb(action.targetGnbId);

    switch (action.actionType)
    {
    case E2RcActionType::HANDOVER_TRIGGER: {
        HandoverControl h{};
        h.target_primary_cell_id = target;
        h.handover_type = HandoverType::intra5gs;
        return ControlAction{h};
    }
    case E2RcActionType::HANDOVER_CANCEL: {
        ConditionalHandoverControl c{};
        // reconfiguration_id 0 = "cancel all pending CHO configs for this UE"
        c.conditional_reconfiguration_id = 0;
        // empty candidate list -> cancellation
        return ControlAction{c};
    }
    default:
        return std::nullopt;
    }
}

} // namespace style3
} // namespace rc_v103
} // namespace oranntn
} // namespace ns3
