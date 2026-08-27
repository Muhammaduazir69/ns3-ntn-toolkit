/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#ifndef ORAN_NTN_RC_STYLE3_H
#define ORAN_NTN_RC_STYLE3_H

// E2SM-RC R004 v07.00 (2025) — Style 3 Connected-Mode Mobility ControlAction (Roadmap
// §4.1.3). The WG3 spec maps mobility-related xApp commands to one of three
// Style 3 actions:
//
//   Action 1: Handover Control          (legacy intra-5GS HO)
//   Action 2: Conditional Handover      (CHO; xApp configures candidates +
//                                        triggers ahead of time)
//   Action 3: Dual Active Protocol Stack HO (DAPS-HO)
//
// References:
//   O-RAN.WG3.E2SM-RC R004 v07.00 (2025) §8.2.3.3 (Connected-Mode Mobility action shapes)
//   3GPP TS 38.331 §5.3.5.13.2        (CHO configuration)
//   3GPP TS 38.331 §5.3.5.13.4        (DAPS-HO)
//   First 5G-Advanced NR-NTN CHO demo, ESA/Eutelsat/Airbus/MediaTek Nov 2025
//     https://connectivity.esa.int/news/esa-eutelsat-airbus-mediatek-and-partners-successfully-test-5gadvanced-nrntn-connection-over-oneweb-leo-satellites-conditional-handover
//
// NOTE: these Style 3 shapes are currently exercised by unit tests only
// (converter + shape round-trips); no example issues Style 3 controls yet.

#include "oran-ntn-types.h"

#include <cstdint>
#include <optional>
#include <string>
#include <variant>
#include <vector>

namespace ns3
{
namespace oranntn
{
namespace rc_v103
{
namespace style3
{

/// 5G NR Cell Global Identity. PLMN ID + 36-bit NR Cell Identity.
struct NrCellGlobalId
{
    std::string plmn_id;        //!< 5- or 6-digit string, e.g. "00101"
    uint64_t nr_cell_identity;  //!< 36-bit; upper 28 bits = gNB-ID, lower 8 = cell-ID
};

/// WG3 E2SM-RC R004 v07.00 (2025) §8.2.3.3 HandoverType.
enum class HandoverType : uint8_t
{
    intra5gs       = 0,
    fivegsToEpc    = 1,
    epcTo5gs       = 2,
    fivegsToUtran  = 3,
};

/// WG3 cause family for DAPS-HO termination.
enum class DapsTerminationCause : uint8_t
{
    target_radio_problem        = 0,
    target_radio_link_recovery  = 1,
    condition_release           = 2,
};

/// Style 3 — Action 1: Handover Control.
struct HandoverControl
{
    static constexpr uint8_t kStyleId = 3;
    static constexpr uint8_t kActionId = 1;

    NrCellGlobalId target_primary_cell_id;
    HandoverType handover_type;
    std::optional<NrCellGlobalId> new_secondary_cell_id;
};

/// Style 3 — Action 2: Conditional Handover Control. Pre-configures one or
/// more candidate target cells; the UE executes the HO on its own when the
/// per-candidate trigger condition fires. CHO is the action the
/// ESA/Eutelsat/Airbus/MediaTek OneWeb 5G-Advanced demo (Nov 2025) used.
struct ConditionalHandoverControl
{
    static constexpr uint8_t kStyleId = 3;
    static constexpr uint8_t kActionId = 2;

    /// Identifier shared by all per-UE conditional reconfigurations.
    uint32_t conditional_reconfiguration_id;

    /// ORAN-10: cancellation, expressed the way TS 38.331 expresses it.
    ///
    /// The previous mapping encoded "cancel everything" as
    /// conditional_reconfiguration_id = 0 with an empty candidate list. That is
    /// an invented semantic: CondReconfigId 0 is a perfectly valid identifier,
    /// not a wildcard, and TS 38.331 section 5.3.5.13 removes conditional
    /// reconfigurations by naming them in condReconfigToRemoveList. A receiver
    /// implementing the standard would have read the old encoding as "cancel
    /// the configuration whose id is 0" and left the rest in place.
    ///
    /// This mirrors condReconfigToRemoveList directly.
    std::vector<uint32_t> conditional_reconfiguration_to_remove_list;

    /// Explicit "release all pending conditional reconfigurations for this UE".
    /// TS 38.331 achieves this by listing every configured CondReconfigId; the
    /// flag keeps the intent unambiguous on the wire without pretending that a
    /// particular id means "all".
    bool remove_all_conditional_reconfigurations{false};

    struct CandidateCell
    {
        NrCellGlobalId target_primary_cell_id;
        /// ASN.1-PER blob carrying CondReconfigurationToAddMod-r16; T2
        /// (Roadmap §3) supplies the encoder/decoder.
        std::vector<uint8_t> trigger_condition;
    };
    std::vector<CandidateCell> candidate_cell_list;
};

/// Style 3 — Action 3: DAPS-HO Control.
struct DapsHandoverControl
{
    static constexpr uint8_t kStyleId = 3;
    static constexpr uint8_t kActionId = 3;

    NrCellGlobalId target_primary_cell_id;
    std::optional<DapsTerminationCause> daps_termination_policy;
};

/// Type-erased Style 3 ControlAction. The variant's index is the WG3
/// Action ID minus one (so .index() ∈ {0,1,2} matches Action {1,2,3}).
using ControlAction =
    std::variant<HandoverControl, ConditionalHandoverControl, DapsHandoverControl>;

/// Returns the WG3 Action ID (1, 2 or 3) for a given ControlAction.
inline uint8_t
ActionId(const ControlAction& a)
{
    return static_cast<uint8_t>(a.index() + 1);
}

/// E2SM-RC ControlMessage wire frame — the bytes carried inside
/// ric_control_request_t::ric_control_message. T2 (ASN.1-PER) will fill in
/// the encoder; for now this struct is the in-memory contract so call-sites
/// can produce / inspect actions without depending on T2.
struct ControlMessage
{
    uint8_t style_id = HandoverControl::kStyleId; //!< always 3 for Style 3
    ControlAction action;
};

/// Convert an `E2RcAction` to a Style 3 `ControlAction` where applicable:
///
///   HANDOVER_TRIGGER -> HandoverControl (intra5gs, NRCGI from targetGnbId)
///   HANDOVER_CANCEL  -> ConditionalHandoverControl with empty candidate list
///                       (cancellation = clearing all pending CHO configs)
///
/// All other action types return std::nullopt. PLMN defaults to "00101"
/// (MCC=001 MNC=01) which matches the toolkit's default test PLMN.
/// gNB-ID length in bits within the 36-bit NR Cell Identity.
///
/// ORAN-10: this was hardwired to a 28/8 split. TS 38.413 makes the gNB-ID
/// length configurable from 22 to 32 bits, with the remaining bits carrying the
/// cell identity, so a deployment using a 24-bit gNB-ID would have had its cell
/// identities silently mangled by the fixed shift.
inline constexpr uint8_t kDefaultGnbIdLengthBits = 28;
inline constexpr uint8_t kMinGnbIdLengthBits = 22;
inline constexpr uint8_t kMaxGnbIdLengthBits = 32;

/// Pack a gNB ID and a cell ID into the 36-bit NR Cell Identity, honouring the
/// configured gNB-ID length. Returns 0 if gnbIdLengthBits is out of range.
uint64_t NrCellIdentityFrom(uint32_t gnbId,
                            uint16_t cellId = 0,
                            uint8_t gnbIdLengthBits = kDefaultGnbIdLengthBits);

std::optional<ControlAction>
ConvertE2RcToStyle3(const E2RcAction& action,
                    const std::string& plmn_id = "00101",
                    uint8_t gnbIdLengthBits = kDefaultGnbIdLengthBits);

} // namespace style3
} // namespace rc_v103
} // namespace oranntn
} // namespace ns3

#endif // ORAN_NTN_RC_STYLE3_H
