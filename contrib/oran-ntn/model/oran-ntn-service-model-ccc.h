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

// ----------------------------------------------------------------------------
//  4.1.8 NTN extensions to E2SM-CCC
//
//  Three NTN-specific ControlAction shapes layered on top of the base
//  CCC ops added in 4.1.6:
//    leo_pass_toggle — enable / disable a beam (or whole cell) on a
//                      satellite for the duration of a GSL pass.
//                      Driven by T5 ContactGraphScheduler GSL events.
//    beam_reconfig   — update a beam's steering angles + complex
//                      weights in the cell's beamforming codebook.
//    doppler_retune  — adjust DL / UL ARFCN to compensate the
//                      LOS-component Doppler shift.
// ----------------------------------------------------------------------------

/// `set_leo_pass_on/off` payload.
struct LeoPassToggleIe
{
    uint64_t nr_cell_global_id{0};
    uint16_t beam_id{0};         //!< 0xFFFF = whole cell
    bool enable{false};
    double t_event_s{0.0};       //!< event time (wall-clock seconds)
};

/// `beam_reconfig` payload.
struct BeamReconfigIe
{
    uint64_t nr_cell_global_id{0};
    uint16_t beam_id{0};
    double steering_az_deg{0.0};
    double steering_el_deg{0.0};
    /// Complex codebook weights, one pair per antenna element.
    /// Encoded as 2*N real numbers (re_0, im_0, re_1, im_1, ...).
    std::vector<double> codebook_weights;
};

/// `doppler_retune` payload.
struct DopplerRetuneIe
{
    uint64_t nr_cell_global_id{0};
    /// New DL / UL ARFCN values per TS 38.104 (0..3279165).
    uint32_t arfcn_dl{0};
    uint32_t arfcn_ul{0};
    /// Reported LOS Doppler offset (Hz). Sign convention: positive when
    /// the satellite approaches the cell's coverage centre.
    double doppler_offset_hz{0.0};
};

/// E2SM-CCC Control Action — set / clear config on the listed cells.
/// 4.1.8 adds three NTN-extension ops with their own per-op vectors.
struct CccControlAction
{
    enum class Op : uint8_t
    {
        set_config = 0,
        clear_config = 1,
        set_perf_objective = 2,
        clear_perf_objective = 3,
        // 4.1.8 NTN extensions:
        set_leo_pass_on = 4,
        set_leo_pass_off = 5,
        beam_reconfig = 6,
        doppler_retune = 7,
    };
    Op op;
    std::vector<CellConfigRecord> cell_updates;
    std::vector<PerformanceObjective> objective_updates;
    // 4.1.8 NTN-extension payloads. Each is populated only when `op`
    // matches the corresponding kind; other ops leave the vector empty.
    std::vector<LeoPassToggleIe> leo_pass_updates;
    std::vector<BeamReconfigIe> beam_reconfigs;
    std::vector<DopplerRetuneIe> doppler_retunes;
};

} // namespace ccc
} // namespace oranntn

/**
 * \ingroup oran-ntn
 * \brief E2SM-CCC v1.00 Service Model plugin (Roadmap §4.1.6).
 *
 * RIC Function ID 1000. Encodes/decodes CccIndMsgFormat1 and
 * CccControlAction via the T2 Aligned-PER codec.
 *
 * \note Currently exercised by unit tests only.
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
