/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#ifndef NTN_RRC_TYPES_H
#define NTN_RRC_TYPES_H

#include <cstdint>

namespace ns3
{
namespace ntnrrc
{

/// 3GPP Rel-17 NTN payload architecture (TR 38.821 §4.2).
enum class PayloadMode : uint8_t
{
    Transparent = 0,        //!< Bent-pipe: gNB on ground, satellite is RF relay only.
    RegenerativeLite = 1,   //!< Partial gNB on sat (e.g. RLC + PDCP).
    RegenerativeFull = 2,   //!< Full gNB on sat (Rel-19 target).
};

/// Whether the UE has a GNSS receiver and is expected to assist its own RRC procedures
/// (TS 38.331 §5.7.4 NTN UE capabilities).
enum class GnssCapability : uint8_t
{
    None = 0,
    GnssCapable = 1,
};

/// Reference frame for TA broadcast in SIB19 (TS 38.331 NTN-Config IE).
enum class TaReferenceFrame : uint8_t
{
    BeamCenter = 0,        //!< common TA referenced to beam centre on Earth surface
    SatelliteSubpoint = 1, //!< referenced to nadir
};

} // namespace ntnrrc
} // namespace ns3

#endif // NTN_RRC_TYPES_H
