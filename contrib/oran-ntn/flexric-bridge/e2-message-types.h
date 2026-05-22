/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#ifndef ORAN_NTN_E2_MESSAGE_TYPES_H
#define ORAN_NTN_E2_MESSAGE_TYPES_H

// E2AP / SCTP framing types for the flexric-bridge (Roadmap §3 T3).
//
// Wire frame on the SCTP / TCP stream:
//   uint16  message_type (network byte order, see E2MessageType)
//   uint32  payload_len  (network byte order)
//   <payload_len bytes>  ASN.1-PER body produced by asn1-per-codec.cc
//
// One frame = one E2AP PDU. The framing layer is independent of the
// chosen transport (SCTP vs TCP); the listener simply pulls bytes off
// the socket and dispatches by `message_type`.

#include <cstdint>

namespace ns3
{
namespace oranntn
{
namespace flexric
{

/// Wire-level E2AP message types. Values are toolkit-internal — they
/// don't map onto an official E2AP codepoint registry (the codepoints
/// in the official ASN.1 are part of the InitiatingMessage / Successful
/// Outcome tag, not the SCTP framing). The T3 follow-on switches to
/// SCTP_SNDINFO + ppid signalling per real E2AP.
enum class E2MessageType : uint16_t
{
    e2_setup_request          = 1,
    e2_setup_response         = 2,
    e2_setup_failure          = 3,
    ric_subscription_request  = 10,
    ric_subscription_response = 11,
    ric_subscription_failure  = 12,
    ric_subscription_delete_request = 13,
    ric_indication            = 20,
    ric_control_request       = 30,
    ric_control_acknowledge   = 31,
    ric_control_failure       = 32,
    e2_connection_update      = 40,
    keepalive_ping            = 90,
    keepalive_pong            = 91,
};

inline const char*
E2MessageTypeName(E2MessageType t)
{
    switch (t)
    {
    case E2MessageType::e2_setup_request:          return "E2 Setup Request";
    case E2MessageType::e2_setup_response:         return "E2 Setup Response";
    case E2MessageType::e2_setup_failure:          return "E2 Setup Failure";
    case E2MessageType::ric_subscription_request:  return "RIC Subscription Request";
    case E2MessageType::ric_subscription_response: return "RIC Subscription Response";
    case E2MessageType::ric_subscription_failure:  return "RIC Subscription Failure";
    case E2MessageType::ric_subscription_delete_request:
                                                    return "RIC Subscription Delete";
    case E2MessageType::ric_indication:            return "RIC Indication";
    case E2MessageType::ric_control_request:       return "RIC Control Request";
    case E2MessageType::ric_control_acknowledge:   return "RIC Control Ack";
    case E2MessageType::ric_control_failure:       return "RIC Control Failure";
    case E2MessageType::e2_connection_update:      return "E2 Connection Update";
    case E2MessageType::keepalive_ping:            return "Keepalive Ping";
    case E2MessageType::keepalive_pong:            return "Keepalive Pong";
    }
    return "Unknown";
}

/// Endpoint role: the listener side acts as the RIC (accepts inbound
/// SCTP from RAN nodes) by default. xApp-driven test clients act as
/// the RAN side and emit RIC Indications.
enum class E2Role : uint8_t
{
    ric = 0,
    ran = 1,
};

} // namespace flexric
} // namespace oranntn
} // namespace ns3

#endif // ORAN_NTN_E2_MESSAGE_TYPES_H
