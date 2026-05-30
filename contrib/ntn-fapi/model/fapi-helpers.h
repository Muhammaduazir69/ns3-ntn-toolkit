/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#ifndef NTN_FAPI_HELPERS_H
#define NTN_FAPI_HELPERS_H

#include "fapi-messages.h"

#include <array>
#include <cstdint>
#include <vector>

namespace ns3
{
namespace fapi
{

/// Expand the FAPI dmrsSymbPos bitmap into the OFDM symbol indices that
/// carry DMRS. The bitmap is 14 bits wide (one bit per symbol in a normal-
/// CP slot); bit 0 is symbol 0 (LSB first). This is the "FAPI-to-bit-array"
/// helper named explicitly in Roadmap §3 T1.
std::vector<uint8_t>
DmrsFapiToBitArray(uint16_t dmrsSymbPos);

/// Inverse of DmrsFapiToBitArray: pack symbol indices back to the 14-bit
/// bitmap form FAPI expects. Indices >= 14 are silently dropped; the helper
/// is intentionally lenient so call-sites can pass the raw scheduler output.
uint16_t
DmrsBitArrayToFapi(const std::vector<uint8_t>& symbolIndices);

/// Returns the message-level ID for any of the typed-message structs.
template <typename Msg>
constexpr MessageId
GetMessageId()
{
    return Msg::kId;
}

} // namespace fapi
} // namespace ns3

#endif // NTN_FAPI_HELPERS_H
