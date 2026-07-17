/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#ifndef NTN_FAPI_COMMON_H
#define NTN_FAPI_COMMON_H

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace ns3
{
namespace fapi
{

/// SCF FAPI version this module's typedefs track.
/// (Roadmap §3 T1: SCF FAPI 222.10.02 + 222.10.04.)
inline constexpr const char* kScfFapiBase = "SCF FAPI 222.10.02";
inline constexpr const char* kScfFapiAddendum = "SCF FAPI 222.10.04";

/// FAPI message IDs (subset implemented in v2.1). Values are the message-type
/// codes used on the SCF FAPI wire. (2026 realism roadmap §3 T1.)
enum MessageId : uint16_t
{
    // P5 / P7 — TTI-level scheduling and indication.
    kDlTtiRequest        = 0x0080,
    kUlTtiRequest        = 0x0081,
    kSlotIndication      = 0x0082,
    kUlDciRequest        = 0x0083,
    kTxDataRequest       = 0x0084,
    kRxDataIndication    = 0x0085,
    kCrcIndication       = 0x0086,
    kUciIndication       = 0x0087,
    kSrsIndication       = 0x0088,
    kRachIndication      = 0x0089,
};

inline const char*
MessageIdName(MessageId id)
{
    switch (id)
    {
    case kDlTtiRequest:     return "DL_TTI.request";
    case kUlTtiRequest:     return "UL_TTI.request";
    case kSlotIndication:   return "SLOT.indication";
    case kUlDciRequest:     return "UL_DCI.request";
    case kTxDataRequest:    return "TX_DATA.request";
    case kRxDataIndication: return "RX_DATA.indication";
    case kCrcIndication:    return "CRC.indication";
    case kUciIndication:    return "UCI.indication";
    case kSrsIndication:    return "SRS.indication";
    case kRachIndication:   return "RACH.indication";
    }
    return "unknown";
}

/// Maximum number of DL/UL HARQ processes. SCF FAPI 222.10.02 sized the HARQ
/// process id at 0..15 (4 bits); 3GPP Rel-17 (TS 38.214 §5.1) raised the cap to
/// 32 processes (0..31) to cover the long NTN RTT. The harqProcessId fields in
/// PdschPdu/PuschPdu are uint8_t and already hold the full Rel-17 range; use
/// this constant when validating a scheduler-supplied HARQ id.
inline constexpr uint8_t kMaxHarqProcessesRel17 = 32;

/// Cyclic prefix per TS 38.211.
enum class CyclicPrefix : uint8_t
{
    kNormal   = 0,
    kExtended = 1,
};

/// Sub-carrier spacing index per TS 38.211 Table 4.2-1 (mu = log2(SCS / 15kHz)).
enum class Numerology : uint8_t
{
    kMu0_15kHz   = 0,
    kMu1_30kHz   = 1,
    kMu2_60kHz   = 2,
    kMu3_120kHz  = 3,
    kMu4_240kHz  = 4,
};

inline double
SubCarrierSpacingKhz(Numerology mu)
{
    return 15.0 * (1U << static_cast<unsigned>(mu));
}

/// Carrier configuration. Aligned to SCF FAPI 222.10.02 §3.3.1 Carrier_config
/// TLV with the field names FlexRIC / Aerial PHY also use verbatim.
struct CarrierConfig
{
    uint32_t dlBandwidth;         //!< MHz
    uint32_t dlFrequency;         //!< kHz
    uint16_t dlK0[5];             //!< per-numerology DL K0
    uint16_t dlGridSize[5];       //!< per-numerology PRB grid size
    uint16_t numRxAnt;
    uint32_t ulBandwidth;         //!< MHz
    uint32_t ulFrequency;         //!< kHz
    uint16_t ulK0[5];             //!< per-numerology UL K0
    uint16_t ulGridSize[5];       //!< per-numerology PRB grid size
    uint16_t numTxAnt;
    Numerology frequencyShift7p5khz; //!< present per SCF FAPI
};

/// Cell-level configuration. SCF FAPI 222.10.02 §3.3.1 Cell_config.
struct CellConfig
{
    uint16_t phyCellId;           //!< 0–1007
    uint8_t frameDuplexType;      //!< 0=FDD, 1=TDD
    uint8_t pdschCpType;          //!< 0=normal CP, 1=extended CP
    uint8_t pdschResourceAllocation;
    uint8_t pdschVrbToPrbMapping;
};

/// Bandwidth-part info carried inside scheduling PDUs.
struct BwpInfo
{
    uint16_t bwpSize;             //!< number of PRBs
    uint16_t bwpStart;            //!< starting PRB
    uint8_t subcarrierSpacing;    //!< matches Numerology enum
    CyclicPrefix cyclicPrefix;
};

/// CCE-to-REG mapping used by PDCCH PDUs. Aligned to FAPI 222.10.02 §3.4.2.6.
struct CceRegMapping
{
    uint8_t cceRegMappingType;    //!< 0=interleaved, 1=non-interleaved
    uint8_t regBundleSize;
    uint8_t interleaverSize;
    uint16_t shiftIndex;
    uint8_t coresetType;          //!< 0=coreset0, 1=other
    uint8_t precoderGranularity;
};

/// DMRS configuration carried inside PDSCH/PUSCH PDUs. The `dmrsSymbPos`
/// bitmap is the field that needs converting via DmrsFapiToBitArray() for
/// downstream PHY consumers (ns-3 mmwave, Aerial cuPHY).
struct DmrsConfig
{
    uint16_t dmrsSymbPos;         //!< Bitmap of OFDM symbols carrying DMRS
    uint8_t dmrsConfigType;       //!< 0=type1, 1=type2
    uint16_t dlDmrsScramblingId;
    uint16_t ulDmrsScramblingId;
    uint8_t scid;                 //!< 0 or 1
    uint8_t numDmrsCdmGrpsNoData;
    uint16_t dmrsPorts;           //!< bitmap of antenna ports carrying DMRS
};

} // namespace fapi
} // namespace ns3

#endif // NTN_FAPI_COMMON_H
