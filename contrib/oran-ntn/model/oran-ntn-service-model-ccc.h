/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#ifndef ORAN_NTN_SERVICE_MODEL_CCC_H
#define ORAN_NTN_SERVICE_MODEL_CCC_H

// E2SM-CCC v1.00 Service Model plugin (Roadmap §4.1.6).
//
// E2SM-CCC (Cell Configuration and Control) is the SM added to the WG3
// July 2025 spec train alongside KPM and RC. It carries:
//   - per-cell configuration structures (DTX, DRX, RF channel, output
//     power, antenna mask, PRB pool)
//   - performance-objective declarations (target SE, latency, BLER)
//   - configuration-update notifications when a cell config changes
//   - ControlActions to write configuration values from an xApp
//
// The toolkit uses RIC Function ID 1000 for CCC (not yet in the
// official O-RAN-SC registry — toolkit-reserved 1000+ range), so an
// xApp written against this SM picks 1000 via the T4 SM Registry.
//
// The on-the-wire ASN.1 PER encoding mirrors the FlexRIC sm/ccc_sm
// surface where it exists; absent fields default to "not present" via
// the SEQUENCE preamble bitmap.

#include "oran-ntn-service-model.h"

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace ns3
{

namespace oranntn
{
namespace ccc
{

/// One cell-level configuration record. Fields use the WG3-aligned names
/// so reviewers can grep them in any CCC-aware xApp.
struct CellConfigRecord
{
    uint64_t nr_cell_global_id{0};   //!< NCGI (36-bit) packed as uint64_t
    uint8_t dtx_us_log2{0};          //!< DTX cycle = 2^dtx_us_log2 microseconds
    uint8_t drx_us_log2{0};          //!< DRX cycle = 2^drx_us_log2 microseconds
    int16_t output_power_dbm{40};    //!< gNB output power (-30 to +50 dBm)
    uint16_t prb_pool_total{273};    //!< total PRBs in the cell (FR2 100 MHz default)
    uint16_t prb_pool_reserved{0};   //!< PRBs reserved for control / overhead
    uint64_t antenna_mask{0xFFFFFFFFFFFFFFFFULL}; //!< bitmask of active antenna elements
    std::optional<uint32_t> arfcn_dl; //!< NR-ARFCN (DL, 0..3279165 per TS 38.104)
    std::optional<uint32_t> arfcn_ul; //!< NR-ARFCN (UL)
};

/// Performance objective for a cell or slice.
struct PerformanceObjective
{
    enum class Metric : uint8_t
    {
        spectral_efficiency = 0,    //!< target b/s/Hz
        latency_ms = 1,             //!< target one-way latency
        bler = 2,                   //!< target block error rate
        prb_utilisation = 3,        //!< target avg PRB utilisation
        ue_throughput_mbps = 4,
    };
    Metric metric;
    double target_value;
    double tolerance;               //!< +/- around target
    uint64_t scope_nr_cgi{0};       //!< 0 = global, else NCGI
    uint8_t scope_slice_id{0};      //!< 0 = unscoped
};

/// E2SM-CCC Indication Format 1 — current configuration snapshot for a
/// list of cells.
struct CccIndMsgFormat1
{
    std::vector<CellConfigRecord> cells;
    /// Performance objectives currently in force.
    std::vector<PerformanceObjective> perf_objectives;
    uint32_t snapshot_seq{0};       //!< monotonic sequence number
};

/// E2SM-CCC Control Action — set / clear config on the listed cells.
struct CccControlAction
{
    enum class Op : uint8_t
    {
        set_config = 0,
        clear_config = 1,
        set_perf_objective = 2,
        clear_perf_objective = 3,
    };
    Op op;
    std::vector<CellConfigRecord> cell_updates;
    std::vector<PerformanceObjective> objective_updates;
};

} // namespace ccc
} // namespace oranntn

/**
 * \ingroup oran-ntn
 * \brief E2SM-CCC v1.00 Service Model plugin (Roadmap §4.1.6).
 *
 * RIC Function ID 1000. Encodes/decodes CccIndMsgFormat1 and
 * CccControlAction via the T2 Aligned-PER codec.
 */
class OranNtnServiceModelCcc : public OranNtnServiceModel
{
  public:
    static constexpr uint16_t kRicFunctionId = 1000;

    static TypeId GetTypeId();
    OranNtnServiceModelCcc() = default;
    ~OranNtnServiceModelCcc() override = default;

    uint16_t RicFunctionId() const override { return kRicFunctionId; }
    std::string Name() const override { return "CCC"; }
    std::string Version() const override { return "v1.00"; }

    /// `body` must point at an oranntn::ccc::CccIndMsgFormat1.
    std::vector<uint8_t> EncodeIndication(const void* body) const override;

    /// Decodes a wire-side CCC ControlAction. `out` -> oranntn::ccc::CccControlAction.
    bool DecodeControl(const std::vector<uint8_t>& msg,
                       void* out) const override;

    /// Convenience: encode a ControlAction into the wire form (writer
    /// path; tests use both directions).
    std::vector<uint8_t>
        EncodeControl(const oranntn::ccc::CccControlAction& action) const;

    /// Convenience: decode a CCC Indication body back into the struct.
    bool DecodeIndication(const std::vector<uint8_t>& msg,
                          oranntn::ccc::CccIndMsgFormat1& out) const;
};

} // namespace ns3

#endif // ORAN_NTN_SERVICE_MODEL_CCC_H
