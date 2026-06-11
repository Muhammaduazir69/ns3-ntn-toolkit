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
 * Canonical KPM metric IDs aligned to O-RAN WG3 E2SM-KPM v03.00 and the
 * srsRAN / OAI naming used by FlexRIC's KPM SM.
 * (2026 realism roadmap §3 T8.)
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
inline constexpr const char* kCarrAvgSinr     = "CARR.AverageSINR";
inline constexpr const char* kL1mRsSinrMean   = "L1M.RS-SINR.Mean";

/// Canonical metric IDs in stable order. Reviewers can pin this list
/// with a snapshot test.
const std::vector<std::string>& CanonicalMetricIds();

} // namespace kpm

/// Canonical KPM label dimensions per O-RAN WG3 E2SM-KPM v03.00. Every
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

/// A single canonical KPM measurement. `value` is in WG3-canonical units
/// (kbps for throughput, bytes for volume, integer PRB counts, dB for SINR).
struct KpmMeasurement
{
    std::string metricId;
    double value;
    std::map<std::string, std::string> labels;
};

/// Map an E2KpmReport to the full canonical 10-metric measurement vector,
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
///   timestamp, gnb_id, is_ntn, ue_id, metric_id, value, present,
///   FIVE_QI, S-NSSAI, PLMN
///
/// Each report produces 10 rows (one per canonical metric ID). Rows with
/// not-yet-plumbed source fields carry `present = 0`.
/// `baseLabels` must contain entries for kFiveQi, kSnssai, kPlmn.
void
WriteCanonicalKpmCsv(const std::vector<E2KpmReport>& reports,
                     const std::map<std::string, std::string>& baseLabels,
                     std::ostream& os);

} // namespace oranntn
} // namespace ns3

#endif // ORAN_NTN_KPM_CANONICAL_IDS_H
