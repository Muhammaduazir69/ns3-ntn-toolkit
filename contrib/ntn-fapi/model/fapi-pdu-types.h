/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#ifndef NTN_FAPI_PDU_TYPES_H
#define NTN_FAPI_PDU_TYPES_H

// SCF FAPI 222.10.02 / 222.10.04 — per-PDU descriptors carried inside the
// scheduling messages (DL_TTI.request / UL_TTI.request / UL_DCI.request).
// Each struct mirrors the SCF wire-format field names so a translator to
// NVIDIA Aerial cuPHY or OAI's FAPI binding is a pure rename.
//
// References:
//   https://docs.nvidia.com/aerial/cuda-accelerated-ran/latest/cubb/cubb_integration_guide/scf_fapi/message_details.html
//   https://github.com/OPENAIRINTERFACE/openairinterface5g/tree/develop/nfapi
//
// (2026 realism roadmap §3 T1.)

#include "fapi-common.h"

#include <cstdint>
#include <vector>

namespace ns3
{
namespace fapi
{

/// PDCCH PDU — SCF FAPI 222.10.02 §3.4.2.6 / Aerial cuPHY same fields.
struct PdcchPdu
{
    BwpInfo bwp;
    uint16_t coresetBwpSize;
    uint16_t coresetBwpStart;
    uint8_t subcarrierSpacing;
    CyclicPrefix cyclicPrefix;
    uint8_t startSymbolIndex;
    uint8_t durationSymbols;
    std::array<uint8_t, 6> freqDomainResource; //!< 45-bit bitmap, packed
    CceRegMapping cceRegMapping;

    struct Dci
    {
        uint16_t rnti;
        uint16_t scramblingId;
        uint16_t scramblingRnti;
        uint8_t cceIndex;
        uint8_t aggregationLevel;
        uint16_t beta_pdcch_1_0;
        uint8_t powerControlOffsetSs;
        std::vector<uint8_t> payload;          //!< DCI bit-string, MSB-first
    };

    std::vector<Dci> dciList;
};

/// PDSCH PDU — SCF FAPI 222.10.02 §3.4.2.7.
struct PdschPdu
{
    uint16_t pduBitmap;
    uint16_t rnti;
    uint16_t pduIndex;
    BwpInfo bwp;
    uint8_t numCodewords;
    struct Codeword
    {
        uint16_t targetCodeRate;
        uint8_t qamModOrder;
        uint8_t mcsIndex;
        uint8_t mcsTable;
        uint8_t rvIndex;
        uint32_t tbSize;          //!< bytes
    };
    std::array<Codeword, 2> codewords;
    uint8_t harqProcessId;        //!< HARQ process ID (0..31, Rel-17 32-process cap; SCF 222.10.02 §3.4.2.7 sized 0..15). See kMaxHarqProcessesRel17.
    uint16_t dataScramblingId;
    uint8_t nrOfLayers;
    uint8_t transmissionScheme;
    uint8_t refPoint;
    DmrsConfig dmrs;
    uint8_t resourceAlloc;
    std::array<uint8_t, 36> rbBitmap; //!< 273-bit bitmap (FR1) / 275-bit (FR2)
    uint16_t rbStart;
    uint16_t rbSize;
    uint8_t vrbToPrbMapping;
    uint8_t startSymbolIndex;
    uint8_t nrOfSymbols;
    // PTRS:
    uint8_t ptrsPortIndex;
    uint8_t ptrsTimeDensity;
    uint8_t ptrsFreqDensity;
    uint8_t ptrsReOffset;
    uint8_t nepre_ratio_of_pdsch_to_ssb;
    // CDM / interleaving (NTN-relevant).
    uint8_t cbgRetransmissionFlag;
};

/// CSI-RS PDU — SCF FAPI 222.10.02 §3.4.2.8.
struct CsiRsPdu
{
    BwpInfo bwp;
    uint16_t startRb;
    uint16_t nrOfRbs;
    uint8_t csiType;              //!< 0=tracking, 1=NZP CSI-RS, 2=ZP CSI-RS
    uint8_t row;                  //!< TS 38.211 Table 7.4.1.5.3-1
    uint16_t freqDomain;          //!< 12-bit bitmap, MSB-first
    uint8_t symbolL0;
    uint8_t symbolL1;
    uint8_t cdmType;
    uint8_t freqDensity;
    uint16_t scrambId;
    uint8_t powerControlOffset;
    uint8_t powerControlOffsetSs;
};

/// PUSCH PDU — SCF FAPI 222.10.02 §3.4.3.1.
struct PuschPdu
{
    uint16_t pduBitmap;
    uint16_t rnti;
    uint32_t handle;
    BwpInfo bwp;
    uint16_t targetCodeRate;
    uint8_t qamModOrder;
    uint8_t mcsIndex;
    uint8_t mcsTable;
    uint8_t transformPrecoding;
    uint16_t dataScramblingId;
    uint8_t nrOfLayers;
    DmrsConfig dmrs;
    uint8_t resourceAlloc;
    std::array<uint8_t, 36> rbBitmap;
    uint16_t rbStart;
    uint16_t rbSize;
    uint8_t vrbToPrbMapping;
    uint8_t startSymbolIndex;
    uint8_t nrOfSymbols;
    uint16_t puschIdentity;       //!< nID per TS 38.211 6.3.1.1
    uint8_t rvIndex;
    uint8_t harqProcessId;        //!< HARQ process ID (0..31, Rel-17 32-process cap). See kMaxHarqProcessesRel17.
    uint32_t tbSize;
    // UCI on PUSCH (FAPI 222.10.04 addendum):
    uint16_t harqAckBitLength;
    uint16_t csiPart1BitLength;
    uint16_t csiPart2BitLength;
};

/// PUCCH PDU — SCF FAPI 222.10.02 §3.4.3.2.
struct PucchPdu
{
    uint16_t rnti;
    uint32_t handle;
    BwpInfo bwp;
    uint8_t formatType;           //!< 0/1/2/3/4
    uint16_t multiSlotTxIndicator;
    uint8_t pi2Bpsk;
    uint16_t prbStart;
    uint16_t prbSize;
    uint8_t startSymbolIndex;
    uint8_t nrOfSymbols;
    uint8_t freqHopFlag;
    uint16_t secondHopPrb;
    uint8_t groupHopFlag;
    uint8_t sequenceHopFlag;
    uint16_t hoppingId;
    uint16_t initialCyclicShift;
    uint16_t dataScramblingId;
    uint8_t timeDomainOccIdx;
    uint8_t preDftOccIdx;
    uint8_t preDftOccLen;
    uint8_t addDmrsFlag;
    uint8_t dmrsScramblingId;
    uint8_t dmrsCyclicShift;
    uint8_t srFlag;
    uint8_t bitLenHarq;
    uint16_t bitLenCsiPart1;
    uint16_t bitLenCsiPart2;
};

/// SRS PDU — SCF FAPI 222.10.02 §3.4.3.3.
struct SrsPdu
{
    uint16_t rnti;
    uint32_t handle;
    BwpInfo bwp;
    uint8_t numAntPorts;
    uint8_t numSymbols;
    uint8_t numRepetitions;
    uint8_t timeStartPosition;
    uint8_t configIndex;
    uint16_t sequenceId;
    uint8_t bandwidthConfig;
    uint8_t bandwidthIndex;
    uint8_t cyclicShift;
    uint8_t freqPosition;
    uint8_t freqShift;
    uint8_t freqHopping;
    uint8_t groupOrSequenceHopping;
    uint8_t resourceType;
    uint16_t tSrs;
    uint16_t tOffset;
};

/// PRACH PDU — SCF FAPI 222.10.02 §3.4.3.4.
struct PrachPdu
{
    uint16_t physCellId;
    uint8_t numPrachOcas;
    uint8_t prachFormat;
    uint8_t indexFdRa;
    uint8_t prachStartSymbol;
    uint16_t numCs;
    BwpInfo bwp;
};

} // namespace fapi
} // namespace ns3

#endif // NTN_FAPI_PDU_TYPES_H
