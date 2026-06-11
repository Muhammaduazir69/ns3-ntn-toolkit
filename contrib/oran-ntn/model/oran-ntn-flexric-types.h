/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#ifndef ORAN_NTN_FLEXRIC_TYPES_H
#define ORAN_NTN_FLEXRIC_TYPES_H

// FlexRIC field-name parity layer (2026 realism roadmap §4.1.1).
//
// This header mirrors the verbatim field names and struct shapes of
// FlexRIC's E2AP v2.03 / v3.01 + E2SM-KPM v3.00 C structs so that the
// ASN.1-PER encoder (Roadmap §3 T2) and the SCTP listener (§3 T3) can be
// dropped in without touching call-sites. C++ types replace FlexRIC's
// custom `seq_arr_t` / `byte_array_t` with std::vector / std::string /
// std::optional, but every field NAME is preserved exactly as FlexRIC
// uses it — that is the integration contract the roadmap requires.
//
// Reference: FlexRIC e2ap_msg.h + sm/kpm_sm/v3_00/ie/kpm_ind_msg_form_1.h
//   https://gitlab.eurecom.fr/mosaic5g/flexric
//   https://github.com/openaicellular/flexric

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <variant>
#include <vector>

namespace ns3
{
namespace oranntn
{
namespace flexric
{

// ----------------------------------------------------------------------------
//  E2AP common
// ----------------------------------------------------------------------------

/// FlexRIC e2ap.h `ric_request_id_t`. Two 16-bit IDs identifying a RIC
/// subscription within a RAN function.
struct ric_request_id_t
{
    uint16_t ric_id;
    uint16_t ric_instance_id;
};

/// FlexRIC e2ap.h `ric_action_type_t`. Values are the wire codes.
enum class ric_action_type_t : uint8_t
{
    report = 0,
    insert = 1,
    policy = 2,
};

/// FlexRIC e2ap.h `ric_subsequent_action_type_t`.
enum class ric_subsequent_action_type_t : uint8_t
{
    continue_action = 0,
    wait_action = 1,
};

/// FlexRIC e2ap.h `ric_action_to_be_setup_t`. The `action_definition` blob
/// is the ASN.1-PER body that T2 emits.
struct ric_action_to_be_setup_t
{
    uint16_t ric_action_id;
    ric_action_type_t ric_action_type;
    std::vector<uint8_t> action_definition;
    std::optional<ric_subsequent_action_type_t> subsequent_action;
};

// ----------------------------------------------------------------------------
//  E2AP top-level messages (the subset the v2.1 sprint depends on)
// ----------------------------------------------------------------------------

/// FlexRIC e2ap.h `ric_subscription_request_t`. Sent xApp -> RIC -> RAN.
struct ric_subscription_request_t
{
    ric_request_id_t ric_id;
    uint16_t ran_function_id;
    std::vector<uint8_t> event_trigger;
    std::vector<ric_action_to_be_setup_t> action_to_be_setup;
};

/// FlexRIC e2ap.h `ric_subscription_response_t`. Sent RAN -> RIC -> xApp.
struct ric_subscription_response_t
{
    ric_request_id_t ric_id;
    uint16_t ran_function_id;
    std::vector<uint16_t> action_admitted_list;
    std::vector<uint16_t> action_not_admitted_list;
};

/// FlexRIC e2ap.h `ric_subscription_delete_request_t`.
struct ric_subscription_delete_request_t
{
    ric_request_id_t ric_id;
    uint16_t ran_function_id;
};

/// FlexRIC e2ap.h `ric_indication_t`. Carried over SCTP from RAN -> RIC.
struct ric_indication_t
{
    ric_request_id_t ric_id;
    uint16_t ran_function_id;
    uint16_t ric_action_id;
    uint32_t ric_indication_sn;
    uint8_t ric_indication_type;                       //!< 0 = report, 1 = insert
    std::vector<uint8_t> ric_indication_header;
    std::vector<uint8_t> ric_indication_message;
    std::optional<std::vector<uint8_t>> ric_call_process_id;
};

/// FlexRIC e2ap.h `ric_control_request_t`. xApp -> RIC -> RAN. Carries an
/// E2SM-RC ControlAction (e.g. Style 3 CHO command).
struct ric_control_request_t
{
    ric_request_id_t ric_id;
    uint16_t ran_function_id;
    std::optional<std::vector<uint8_t>> ric_call_process_id;
    std::vector<uint8_t> ric_control_header;
    std::vector<uint8_t> ric_control_message;
    std::optional<uint8_t> ric_control_ack_request;
};

/// FlexRIC e2ap_msg.h `e2ap_pdu_type_t`.
enum class e2ap_pdu_type_t : uint8_t
{
    initiating_message = 0,
    successful_outcome = 1,
    unsuccessful_outcome = 2,
};

/// FlexRIC e2ap_msg.h `e2ap_msg_t`. Top-level tagged-union E2AP PDU.
/// Only the bodies the v2.1 sprint exercises are wired in; extension to
/// the full set is part of the T3 SCTP listener (Q4 2026).
struct e2ap_msg_t
{
    e2ap_pdu_type_t type;
    std::variant<std::monostate,
                 ric_subscription_request_t,
                 ric_subscription_response_t,
                 ric_subscription_delete_request_t,
                 ric_indication_t,
                 ric_control_request_t>
        u;
};

// ----------------------------------------------------------------------------
//  E2SM-KPM v3.00
// ----------------------------------------------------------------------------

namespace kpm_v3
{

/// FlexRIC sm/kpm_sm/v3_00/ie/meas_type.h `meas_type_form_t`.
enum class meas_type_form_t : uint8_t
{
    name = 0,
    id = 1,
};

/// FlexRIC `meas_type_t`.
struct meas_type_t
{
    meas_type_form_t form;
    /// Populated when form == name. Carries one of the canonical IDs (e.g.
    /// "DRB.UEThpDl") from oran-ntn-kpm-canonical-ids.h (Roadmap T8).
    std::string meas_name;
    /// Populated when form == id.
    uint32_t meas_id;
};

/// FlexRIC `meas_value_form_t`.
enum class meas_value_form_t : uint8_t
{
    integer = 0,
    real = 1,
    no_value = 2,
};

/// FlexRIC `meas_record_item_t`.
struct meas_record_item_t
{
    meas_value_form_t form;
    int64_t int_val;
    double real_val;
};

/// FlexRIC `label_info_t`. Optional WG3 label dimensions.
struct label_info_t
{
    std::optional<uint8_t> five_qi;
    std::optional<std::string> s_nssai;
    std::optional<std::string> plmn_id;
};

/// FlexRIC `meas_info_format_1_lst_t`. One canonical metric across N
/// timestamped samples with M label sets.
struct meas_info_format_1_lst_t
{
    meas_type_t meas_type;
    std::vector<meas_record_item_t> meas_record_lst;
    std::vector<label_info_t> label_info_lst;
};

/// FlexRIC `kpm_ind_msg_format_1_t`. Body of an E2SM-KPM Indication
/// Format 1 carried inside ric_indication_t.ric_indication_message.
struct kpm_ind_msg_format_1_t
{
    std::vector<meas_info_format_1_lst_t> meas_info_lst;
    uint32_t gran_period_ms;
};

} // namespace kpm_v3

} // namespace flexric
} // namespace oranntn
} // namespace ns3

#endif // ORAN_NTN_FLEXRIC_TYPES_H
