/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#include "oran-ntn-kpm-canonical-ids.h"

namespace ns3
{
namespace oranntn
{

namespace kpm
{

const std::vector<std::string>&
CanonicalMetricIds()
{
    static const std::vector<std::string> ids = {
        kDrbUeThpDl,
        kDrbUeThpUl,
        kDrbPdcpVolumeDl,
        kDrbPdcpVolumeUl,
        kRruPrbAvailDl,
        kRruPrbAvailUl,
        kRruPrbUsedDl,
        kRruPrbUsedUl,
        kCarrAvgSinr,
        kL1mRsSinrMean,
    };
    return ids;
}

} // namespace kpm

namespace
{

// Number of PRBs in a single 100 MHz NR carrier at 30 kHz SCS. Used as the
// PRB-availability divisor in the absence of a dedicated cell-config field.
constexpr uint32_t kPrbsPerCarrierFr2_100mhz_30khz = 273;

KpmMeasurement
MakeMeasurement(const char* id,
                double value,
                bool present,
                const std::map<std::string, std::string>& base)
{
    KpmMeasurement m;
    m.metricId = id;
    m.value = value;
    m.labels = base;
    if (!present)
    {
        m.labels[label::kPresent] = "false";
    }
    return m;
}

} // namespace

std::vector<KpmMeasurement>
BuildCanonicalKpmMeasurements(const E2KpmReport& r,
                              const std::map<std::string, std::string>& base)
{
    std::vector<KpmMeasurement> out;
    out.reserve(10);

    // DRB.UEThpDl — DL throughput, kbps (WG3 unit).
    const double thpDlKbps = r.throughput_Mbps * 1e3;
    out.push_back(
        MakeMeasurement(kpm::kDrbUeThpDl, thpDlKbps, /*present=*/true, base));

    // DRB.UEThpUl — UL throughput. Not yet populated in E2KpmReport at the
    // v2.1 baseline; the Q4 2026 ContactGraphScheduler + CU/DU/RU split work
    // will fill this. Emit as not-present so the canonical vector stays
    // shape-stable for downstream consumers (FlexRIC xApps).
    out.push_back(
        MakeMeasurement(kpm::kDrbUeThpUl, 0.0, /*present=*/false, base));

    // DRB.PdcpSduVolumeDL — bytes since last report. Stand-in derived from
    // instantaneous throughput at the report cadence; will be replaced by a
    // proper integrating counter when the OranNtnDataRepository (§4.1.4)
    // lands.
    const double volDlBytes = (r.throughput_Mbps * 1e6 / 8.0);
    out.push_back(
        MakeMeasurement(kpm::kDrbPdcpVolumeDl, volDlBytes, /*present=*/true, base));

    out.push_back(
        MakeMeasurement(kpm::kDrbPdcpVolumeUl, 0.0, /*present=*/false, base));

    // RRU.PrbAvailDl/Ul — total PRBs configured for the cell. v2.1 assumes a
    // single 100 MHz FR2 carrier at 30 kHz SCS = 273 PRBs.
    out.push_back(MakeMeasurement(kpm::kRruPrbAvailDl,
                                  kPrbsPerCarrierFr2_100mhz_30khz,
                                  /*present=*/true,
                                  base));
    out.push_back(MakeMeasurement(kpm::kRruPrbAvailUl,
                                  kPrbsPerCarrierFr2_100mhz_30khz,
                                  /*present=*/true,
                                  base));

    // RRU.PrbUsedDl/Ul — used PRBs derived from cell-level prbUtilization.
    const double prbUsedDl =
        r.prbUtilization * static_cast<double>(kPrbsPerCarrierFr2_100mhz_30khz);
    out.push_back(MakeMeasurement(kpm::kRruPrbUsedDl,
                                  prbUsedDl,
                                  /*present=*/true,
                                  base));
    out.push_back(MakeMeasurement(kpm::kRruPrbUsedUl,
                                  0.0,
                                  /*present=*/false,
                                  base));

    // CARR.AverageSINR — cell-level mean SINR in dB. Until per-cell averaging
    // is wired the per-UE sinr is forwarded; reviewers should treat the
    // FIVE_QI label as the disambiguator.
    out.push_back(
        MakeMeasurement(kpm::kCarrAvgSinr, r.sinr_dB, /*present=*/true, base));

    // L1M.RS-SINR.Mean — L1-measured RS-SINR mean. The mmwave PHY emits the
    // same source today; once srsRAN-style per-symbol L1 SINR averaging is in
    // place the two metrics will diverge.
    out.push_back(MakeMeasurement(kpm::kL1mRsSinrMean,
                                  r.sinr_dB,
                                  /*present=*/true,
                                  base));

    return out;
}

void
WriteCanonicalKpmCsv(const std::vector<E2KpmReport>& reports,
                     const std::map<std::string, std::string>& baseLabels,
                     std::ostream& os)
{
    os << "timestamp,gnb_id,is_ntn,ue_id,metric_id,value,present,"
          "FIVE_QI,S-NSSAI,PLMN\n";
    for (const auto& r : reports)
    {
        // Per-slice 5QI so the canonical stream differentiates the slices:
        // eMBB(0)->9 (default non-GBR), URLLC(1)->82 (delay-critical GBR),
        // mMTC(2)->79. Overrides the base FIVE_QI label per record.
        std::map<std::string, std::string> labels = baseLabels;
        labels[label::kFiveQi] =
            (r.sliceId == 1) ? "82" : (r.sliceId == 2) ? "79" : "9";
        const auto measurements = BuildCanonicalKpmMeasurements(r, labels);
        for (const auto& m : measurements)
        {
            const auto presentIt = m.labels.find(label::kPresent);
            const bool present =
                (presentIt == m.labels.end()) || presentIt->second != "false";
            os << r.timestamp << "," << r.gnbId << ","
               << (r.isNtn ? 1 : 0) << "," << r.ueId << "," << m.metricId
               << "," << m.value << "," << (present ? 1 : 0) << ","
               << m.labels.at(label::kFiveQi) << ","
               << m.labels.at(label::kSnssai) << ","
               << m.labels.at(label::kPlmn) << "\n";
        }
    }
}

} // namespace oranntn
} // namespace ns3
