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

} // namespace

uint64_t
NrCellIdentityFrom(uint32_t gnbId, uint16_t cellId, uint8_t gnbIdLengthBits)
{
    // TS 38.413: NR Cell Identity is 36 bits, of which the leading
    // gnbIdLengthBits carry the gNB ID and the remainder the cell ID. The old
    // helper fixed that split at 28/8.
    if (gnbIdLengthBits < kMinGnbIdLengthBits || gnbIdLengthBits > kMaxGnbIdLengthBits)
    {
        return 0;
    }
    const uint8_t cellBits = static_cast<uint8_t>(36 - gnbIdLengthBits);
    const uint64_t gnbMask = (gnbIdLengthBits >= 64) ? ~0ULL : ((1ULL << gnbIdLengthBits) - 1);
    const uint64_t cellMask = (1ULL << cellBits) - 1;
    return ((static_cast<uint64_t>(gnbId) & gnbMask) << cellBits) |
           (static_cast<uint64_t>(cellId) & cellMask);
}

std::optional<ControlAction>
ConvertE2RcToStyle3(const E2RcAction& action, const std::string& plmn_id,
                    uint8_t gnbIdLengthBits)
{
    NrCellGlobalId target{};
    target.plmn_id = plmn_id;
    target.nr_cell_identity = NrCellIdentityFrom(action.targetGnbId, 0, gnbIdLengthBits);

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
        // ORAN-10: cancel the way TS 38.331 section 5.3.5.13 cancels, by naming
        // the conditional reconfigurations to remove. parameter1 carries the
        // CondReconfigId when the caller targets one; otherwise the request is
        // an explicit release-all, which is stated as such rather than encoded
        // as "the configuration whose id happens to be 0".
        const uint32_t reconfigId = static_cast<uint32_t>(action.parameter1);
        if (reconfigId != 0)
        {
            c.conditional_reconfiguration_id = reconfigId;
            c.conditional_reconfiguration_to_remove_list.push_back(reconfigId);
        }
        else
        {
            c.remove_all_conditional_reconfigurations = true;
        }
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
