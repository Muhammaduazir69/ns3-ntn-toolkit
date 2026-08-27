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
/// OBS-13: the module's own traffic example emitted "ntn_downlink" and five
/// field names that appear nowhere in this header and in none of the four
/// shipped dashboards. A schema whose own module bypasses it is advisory, and
/// an advisory schema is a naming convention nobody has to follow. The
/// measurement is legitimate, so it is adopted here rather than the example
/// being made to fake one of the existing ones.
inline constexpr const char* kDownlink = "ntn_downlink"; //!< per-UE DL goodput + link geometry

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

// OBS-08: provenance is part of the schema.
//
// It was not, so every exporter invented its own key as a bare string literal.
// Two names were in use for the same concept - "provenance" and
// "rsrp_provenance" - plus "band" and "link", none of which any consumer could
// discover from the schema. A dashboard or a lint that wants to reject a series
// whose numbers are derived has to know what the key is called.
//
// The convention, stated here so it stops being reinvented:
//   kProvenance          applies to the whole point
//   <field>_provenance   overrides it for one field, where a point genuinely
//                        mixes origins (a measured SINR beside a reconstructed
//                        RSRP is the case that forced this)
//
// Values in use across the toolkit: "measured", "phy-trace", "app-trace",
// "inband-timestamp", "inband-seq", "rrc-meas-report", "derived",
// "derived-per-re", "geometry-budget", "config", "bler-errormodel".
inline constexpr const char* kProvenance = "provenance";
inline constexpr const char* kRsrpProvenance = "rsrp_provenance";
inline constexpr const char* kBand = "band";
inline constexpr const char* kLink = "link";

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
// OBS-13: fields the traffic example was emitting off-schema.
inline constexpr const char* kGoodputMbps = "goodput_mbps";
inline constexpr const char* kTbler = "tbler";
inline constexpr const char* kElevationDeg = "elevation_deg";
inline constexpr const char* kSlantRangeKm = "slant_range_km";
inline constexpr const char* kRxBytesTotal = "rx_bytes_total";
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
