/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#ifndef NTN_FAPI_MESSAGES_H
#define NTN_FAPI_MESSAGES_H

// SCF FAPI 222.10.02 / 222.10.04 message-level typedefs.
// (2026 realism roadmap §3 T1.)
//
// These structs are intentionally header-only and free of algorithmic logic;
// the role of T1 is to provide an ABI shape against which mmwave / oran-ntn
// scheduler code, NVIDIA cuPHY, and OAI's nfapi can all link without
// touching scheduler-side code.

#include "fapi-common.h"
#include "fapi-pdu-types.h"

#include <cstdint>
#include <variant>
#include <vector>

namespace ns3
{
namespace fapi
{

/// A single DL_TTI PDU is one of the four DL PDU descriptors. The order in
/// `pduList` is the order PHY consumes for that slot.
struct DlTtiPdu
{
    enum class Type : uint8_t
    {
        kPdcch = 0,
        kPdsch = 1,
        kCsiRs = 2,
        kSsb = 3,                   //!< SS/PBCH (not modelled in v2.1 typedefs)
    };

    Type type;
    std::variant<PdcchPdu, PdschPdu, CsiRsPdu> pdu; //!< PdcchPdu when type=kPdcch, etc.
};

/// DL_TTI.request — SCF FAPI 222.10.02 §3.4.2.1.
struct DlTtiRequest
{
    static constexpr MessageId kId = kDlTtiRequest;

    uint16_t sfn;                   //!< 0–1023
    uint16_t slot;                  //!< 0–319 (per numerology)
    uint8_t nPdusOfEachType[4];     //!< [PDCCH, PDSCH, CSI-RS, SSB]
    uint8_t numGroups;              //!< Tx beamforming groups
    std::vector<DlTtiPdu> pduList;
};

/// UL_TTI PDU descriptor wrapper.
struct UlTtiPdu
{
    enum class Type : uint8_t
    {
        kPrach = 0,
        kPusch = 1,
        kPucch = 2,
        kSrs = 3,
    };

    Type type;
    std::variant<PrachPdu, PuschPdu, PucchPdu, SrsPdu> pdu;
};

/// UL_TTI.request — SCF FAPI 222.10.02 §3.4.3.1.
struct UlTtiRequest
{
    static constexpr MessageId kId = kUlTtiRequest;

    uint16_t sfn;
    uint16_t slot;
    uint8_t nPdusOfEachType[4];     //!< [PRACH, PUSCH, PUCCH, SRS]
    std::vector<UlTtiPdu> pduList;
};

/// SLOT.indication — SCF FAPI 222.10.02 §3.4.4.1.
struct SlotIndication
{
    static constexpr MessageId kId = kSlotIndication;

    uint16_t sfn;
    uint16_t slot;
};

/// UL_DCI.request — SCF FAPI 222.10.02 §3.4.2.2 (carries DCI for the UL
/// grant only, separate from DL_TTI to ease split-PHY decoding).
struct UlDciRequest
{
    static constexpr MessageId kId = kUlDciRequest;

    uint16_t sfn;
    uint16_t slot;
    std::vector<PdcchPdu> pdcchList;
};

/// TX_DATA.request — SCF FAPI 222.10.02 §3.4.2.3. Carries the actual TBs
/// referenced by PDSCH PDUs in the same slot.
struct TxDataRequest
{
    static constexpr MessageId kId = kTxDataRequest;

    uint16_t sfn;
    uint16_t slot;
    struct PduPayload
    {
        uint16_t pduIndex;
        uint16_t cwIndex;            //!< codeword index (0 or 1)
        std::vector<uint8_t> tbBytes;
    };
    std::vector<PduPayload> pdus;
};

/// RX_DATA.indication — SCF FAPI 222.10.02 §3.4.4.2.
struct RxDataIndication
{
    static constexpr MessageId kId = kRxDataIndication;

    uint16_t sfn;
    uint16_t slot;
    struct PduRx
    {
        uint32_t handle;
        uint16_t rnti;
        uint16_t harqId;
        uint16_t pduLength;
        std::vector<uint8_t> tbBytes;
    };
    std::vector<PduRx> pdus;
};

/// CRC.indication — SCF FAPI 222.10.02 §3.4.4.3.
struct CrcIndication
{
    static constexpr MessageId kId = kCrcIndication;

    uint16_t sfn;
    uint16_t slot;
    struct CrcReport
    {
        uint32_t handle;
        uint16_t rnti;
        uint16_t harqId;
        bool tbCrcStatusOk;
        std::vector<bool> cbCrcStatusOk; //!< per CBG
        int16_t ul_cqi;                  //!< signed dB (FAPI 222.10.04 addendum)
        uint16_t timingAdvance;
        uint16_t rssi;
    };
    std::vector<CrcReport> crcList;
};

/// SRS.indication — SCF FAPI 222.10.02 §3.4.4.5.
struct SrsIndication
{
    static constexpr MessageId kId = kSrsIndication;

    uint16_t sfn;
    uint16_t slot;
    struct Report
    {
        uint32_t handle;
        uint16_t rnti;
        uint16_t timingAdvance;
        uint8_t numSymbols;
        uint8_t wideband_snr;
        uint16_t numRbs;
        std::vector<int8_t> wbSnrPerRbDb; //!< per-RB wideband SNR
    };
    std::vector<Report> reports;
};

/// RACH.indication — SCF FAPI 222.10.02 §3.4.4.6.
struct RachIndication
{
    static constexpr MessageId kId = kRachIndication;

    uint16_t sfn;
    uint16_t slot;
    struct Preamble
    {
        uint8_t symbolIndex;
        uint8_t slotIndex;
        uint8_t freqIndex;
        uint8_t preambleIndex;
        uint16_t timingAdvance;
        uint16_t preamblePower;
    };
    std::vector<Preamble> preambles;
};

} // namespace fapi
} // namespace ns3

#endif // NTN_FAPI_MESSAGES_H
