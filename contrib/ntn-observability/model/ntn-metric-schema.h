/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#ifndef NTN_METRIC_SCHEMA_H
#define NTN_METRIC_SCHEMA_H

namespace ns3
{
namespace ntnobs
{

/// Canonical InfluxDB measurement names + Grafana dashboard panels reference
/// them. Keep stable across versions — adding new metrics is fine, renaming
/// existing ones breaks every shipped dashboard. All names are snake_case.
namespace measurement
{

inline constexpr const char* kRadio = "ntn_radio";          //!< RSRP, SINR, BLER, MCS per UE x cell
inline constexpr const char* kHandover = "ntn_handover";    //!< CHO trigger, exec, fail counters
inline constexpr const char* kIsl = "ntn_isl";              //!< ISL link load, latency, neighbours
inline constexpr const char* kTa = "ntn_timing_advance";    //!< total / common / drift TA per UE
inline constexpr const char* kSib = "ntn_sib19";            //!< SIB19 broadcast counters
inline constexpr const char* kDrx = "ntn_drx";              //!< per-state cumulative time
inline constexpr const char* kSatPosition = "ntn_sat_pos";  //!< sat ECEF position + velocity
inline constexpr const char* kUeReport = "ntn_ue_report";   //!< UE GNSS lat/lon/alt
inline constexpr const char* kSlice = "ntn_slice";          //!< per-slice throughput / latency
inline constexpr const char* kBeam = "ntn_beam";            //!< beam selection / gain

} // namespace measurement

namespace tag
{

inline constexpr const char* kCellId = "cell_id";
inline constexpr const char* kUeImsi = "ue_imsi";
inline constexpr const char* kSatNorad = "sat_norad";
inline constexpr const char* kBeamId = "beam_id";
inline constexpr const char* kSliceSst = "slice_sst";
inline constexpr const char* kPayloadMode = "payload_mode";
inline constexpr const char* kRunId = "run_id";

} // namespace tag

namespace field
{

inline constexpr const char* kRsrpDbm = "rsrp_dbm";
inline constexpr const char* kSinrDb = "sinr_db";
inline constexpr const char* kBler = "bler";
inline constexpr const char* kMcs = "mcs";
inline constexpr const char* kTaTotalUs = "ta_total_us";
inline constexpr const char* kTaCommonUs = "ta_common_us";
inline constexpr const char* kTaDriftUsPerS = "ta_drift_us_per_s";
inline constexpr const char* kSatXM = "sat_x_m";
inline constexpr const char* kSatYM = "sat_y_m";
inline constexpr const char* kSatZM = "sat_z_m";
inline constexpr const char* kSatVxMps = "sat_vx_mps";
inline constexpr const char* kSatVyMps = "sat_vy_mps";
inline constexpr const char* kSatVzMps = "sat_vz_mps";
inline constexpr const char* kLatDeg = "lat_deg";
inline constexpr const char* kLonDeg = "lon_deg";
inline constexpr const char* kAltM = "alt_m";
inline constexpr const char* kThroughputMbps = "throughput_mbps";
inline constexpr const char* kLatencyMs = "latency_ms";
inline constexpr const char* kBroadcastSeq = "broadcast_seq";
inline constexpr const char* kReportSeq = "report_seq";
inline constexpr const char* kHoExecCount = "ho_exec_count";
inline constexpr const char* kHoTriggerCount = "ho_trigger_count";
inline constexpr const char* kHoFailCount = "ho_fail_count";
inline constexpr const char* kIslRangeKm = "isl_range_km";
inline constexpr const char* kIslLoadMbps = "isl_load_mbps";
inline constexpr const char* kDrxState = "drx_state";
inline constexpr const char* kDrxAwakeMs = "drx_awake_ms";
// W6 — slice orchestration KPIs.
inline constexpr const char* kSlicePrbAllocated = "slice_prb_allocated";
inline constexpr const char* kSliceServedMbps = "slice_served_mbps";
inline constexpr const char* kSliceDemandMbps = "slice_demand_mbps";
inline constexpr const char* kSliceSatisfaction = "slice_satisfaction";
inline constexpr const char* kSliceLatencyP99Ms = "slice_latency_p99_ms";
inline constexpr const char* kSliceLossRate = "slice_loss_rate";
inline constexpr const char* kSliceLatencyBreach = "slice_latency_breach";
inline constexpr const char* kSliceReliabilityBreach = "slice_reliability_breach";

} // namespace field

} // namespace ntnobs
} // namespace ns3

#endif // NTN_METRIC_SCHEMA_H
