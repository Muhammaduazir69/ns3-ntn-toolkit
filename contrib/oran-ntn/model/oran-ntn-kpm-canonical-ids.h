/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#ifndef ORAN_NTN_KPM_CANONICAL_IDS_H
#define ORAN_NTN_KPM_CANONICAL_IDS_H

#include "oran-ntn-types.h"

#include <map>
#include <ostream>
#include <string>
#include <vector>

namespace ns3
{
namespace oranntn
{

/**
 * Canonical KPM metric IDs aligned to O-RAN WG3 E2SM-KPM R004 v06.00
 * (ETSI TS 104 040 v4.0.0, 2025) and the srsRAN / OAI naming used by
 * FlexRIC's KPM SM. (2026 realism roadmap §3 T8.)
 *
 * Provenance note: the throughput/volume/PRB/TB IDs (DRB.*, RRU.*, TB.*)
 * are 3GPP TS 28.552 PM counters. The two SINR IDs (CARR.AverageSINR and
 * L1M.RS-SINR.Mean) are NOT TS 28.552 PM counters — RS-SINR is a PHY
 * quantity defined in 3GPP TS 38.215. They are kept as legitimate vendor
 * L1 metrics (as emitted by srsRAN / OAI L1) and reported in dB.
 *
 * Reviewers can grep these strings in the KPM CSV output and in any KPM
 * Indication-Message blob emitted on the wire. Adding a new ID is fine;
 * renaming an existing one breaks any xApp written against FlexRIC's
 * KPM SM. The full T4 plugin ABI (Q4 2026) will move this list inside
 * `OranNtnServiceModelKpm`; for now the strings live as toolkit-wide
 * constants so the rest of the v2.1 sprint can rely on them.
 *
 * Reference: <https://github.com/srsran/srsRAN_Project/blob/main/lib/e2/
 *             e2sm/e2sm_kpm/e2sm_kpm_metric_defs.h>
 */
namespace kpm
{

inline constexpr const char* kDrbUeThpDl      = "DRB.UEThpDl";
inline constexpr const char* kDrbUeThpUl      = "DRB.UEThpUl";
inline constexpr const char* kDrbPdcpVolumeDl = "DRB.PdcpSduVolumeDL";
inline constexpr const char* kDrbPdcpVolumeUl = "DRB.PdcpSduVolumeUL";
inline constexpr const char* kRruPrbAvailDl   = "RRU.PrbAvailDl";
inline constexpr const char* kRruPrbAvailUl   = "RRU.PrbAvailUl";
inline constexpr const char* kRruPrbUsedDl    = "RRU.PrbUsedDl";
inline constexpr const char* kRruPrbUsedUl    = "RRU.PrbUsedUl";
// TB.ErrTotNbrDl / TB.TotNbrDl — canonical 3GPP TS 28.552 DL transport-block
// counters. The DL TB error rate is the ratio ErrTotNbrDl / TotNbrDl; it is
// already measured upstream as the HARQ BLER (see BuildCanonicalKpmMeasurements).
inline constexpr const char* kTbErrTotNbrDl   = "TB.ErrTotNbrDl";
inline constexpr const char* kTbTotNbrDl      = "TB.TotNbrDl";
// CARR.AverageSINR / L1M.RS-SINR.Mean — L1/PHY SINR quantities per 3GPP
// TS 38.215 (RS-SINR), NOT TS 28.552 PM counters. Legitimate vendor L1
// metrics (srsRAN / OAI L1), reported in dB.
inline constexpr const char* kCarrAvgSinr     = "CARR.AverageSINR";
inline constexpr const char* kL1mRsSinrMean   = "L1M.RS-SINR.Mean";

/// Canonical metric IDs in stable order. Reviewers can pin this list
/// with a snapshot test.
const std::vector<std::string>& CanonicalMetricIds();

} // namespace kpm

/// Canonical KPM label dimensions per O-RAN WG3 E2SM-KPM R004 v06.00
/// (ETSI TS 104 040 v4.0.0, 2025). Every
/// canonical measurement must carry these label keys (value can be empty).
namespace label
{

inline constexpr const char* kFiveQi = "FIVE_QI";
inline constexpr const char* kSnssai = "S-NSSAI";
inline constexpr const char* kPlmn   = "PLMN";

/// Sentinel label written when the source field is not populated by the
/// current toolkit revision. Reviewers can distinguish unpopulated from
/// genuinely-zero measurements without ambiguity.
inline constexpr const char* kPresent = "present";

} // namespace label

/// Provenance of a canonical KPM value, so a consumer never mistakes an
/// estimate/fabrication for a ground-truth measurement (TS 28.552 mandate):
///   - "measured"    : taken from the real data plane (FlowMonitor RX bytes,
///                     measured HARQ NACKs, RS-SINR from the PHY).
///   - "derived"     : computed from a measured quantity or a fixed cell-config
///                     constant (e.g. link-budget throughput, PRB-avail = 273).
///   - "synthesized" : no measurement basis; the value is NaN / present=false.
namespace provenance
{
inline constexpr const char* kMeasured    = "measured";
inline constexpr const char* kDerived     = "derived";
inline constexpr const char* kSynthesized = "synthesized";
} // namespace provenance

/// A single canonical KPM measurement. `value` is in WG3-canonical units
/// (kbps for throughput, bytes for volume, integer PRB/TB counts, dB for SINR).
struct KpmMeasurement
{
    std::string metricId;
    double value;
    std::map<std::string, std::string> labels;
    std::string provenance; //!< measured | derived | synthesized (see above)
};

/// Map an E2KpmReport to the full canonical 12-metric measurement vector,
/// applying `baseLabels` to every measurement. Where a metric's source
/// field is not populated by the current report (e.g. UL throughput is not
/// yet plumbed end-to-end), the measurement is still emitted with value 0
/// and an extra `present=false` label, so reviewers can confirm ID/label
/// alignment even before backfilling all source fields.
std::vector<KpmMeasurement>
BuildCanonicalKpmMeasurements(const E2KpmReport& report,
                              const std::map<std::string, std::string>& baseLabels);

/// Emit the WG3-canonical long-format KPM CSV for a vector of reports
/// (Roadmap §4.1.2). Header columns:
///   timestamp, gnb_id, is_ntn, ue_id, metric_id, value, present, provenance,
///   FIVE_QI, S-NSSAI, PLMN
/// The `provenance` column (measured|derived|synthesized) lets a consumer
/// reject non-ground-truth values without guessing.
///
/// Each report produces 12 rows (one per canonical metric ID). Rows with
/// not-yet-plumbed source fields carry `present = 0`.
/// `baseLabels` must contain entries for kFiveQi, kSnssai, kPlmn.
void
WriteCanonicalKpmCsv(const std::vector<E2KpmReport>& reports,
                     const std::map<std::string, std::string>& baseLabels,
                     std::ostream& os);

} // namespace oranntn
} // namespace ns3

#endif // ORAN_NTN_KPM_CANONICAL_IDS_H
