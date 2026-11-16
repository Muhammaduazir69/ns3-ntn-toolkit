/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#include "oran-ntn-kpm-canonical-ids.h"

#include <sstream>
#include <cmath>
#include <string>

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
        kTbTotNbrDl,
        kTbErrTotNbrDl,
    };
    return ids;
}

} // namespace kpm

namespace
{

// Number of PRBs in a single 100 MHz NR carrier at 30 kHz SCS (TS 38.101-1
// Table 5.3.2-1, an FR1 configuration; the old name said FR2, where 100 MHz at
// 120 kHz SCS is 66 PRBs). Used as the PRB-availability divisor in the absence
// of a dedicated cell-config field, so RRU.PrbAvailDl/RRU.PrbUsedDl are only
// right for a 100 MHz / 30 kHz cell and carry provenance "derived".
constexpr uint32_t kPrbsPerCarrierFr2_100mhz_30khz = 273;

KpmMeasurement
MakeMeasurement(const char* id,
                double value,
                bool present,
                const std::map<std::string, std::string>& base,
                const char* prov = provenance::kSynthesized)
{
    KpmMeasurement m;
    m.metricId = id;
    m.value = value;
    m.labels = base;
    // A not-present or NaN value has no measurement basis: force provenance to
    // "synthesized" so a caller-declared measured/derived can never mislabel an
    // absent value as ground truth.
    if (!present || std::isnan(value))
    {
        m.labels[label::kPresent] = "false";
        m.provenance = provenance::kSynthesized;
    }
    else
    {
        m.provenance = prov;
    }
    return m;
}

} // namespace

std::vector<KpmMeasurement>
BuildCanonicalKpmMeasurements(const E2KpmReport& r,
                              const std::map<std::string, std::string>& base)
{
    std::vector<KpmMeasurement> out;
    out.reserve(12);

    // Per-KPI provenance from the report's measured-plane flags: measured when
    // the value came from the real data plane, derived otherwise (link-budget /
    // cell-config estimate). MakeMeasurement downgrades any NaN/absent value to
    // "synthesized" regardless.
    const char* thpProv =
        r.throughputMeasured ? provenance::kMeasured : provenance::kDerived;
    const char* prbProv =
        r.prbMeasured ? provenance::kMeasured : provenance::kDerived;
    // ORAN-03: SINR gets the same treatment as throughput and PRB. Hardcoding
    // kMeasured here labelled every closed-form candidate SINR as a
    // measurement.
    const char* sinrProv =
        r.sinrMeasured ? provenance::kMeasured : provenance::kDerived;

    // DRB.UEThpDl — DL throughput, kbps (WG3 unit).
    const double thpDlKbps = r.throughput_Mbps * 1e3;
    out.push_back(
        MakeMeasurement(kpm::kDrbUeThpDl, thpDlKbps, /*present=*/true, base, thpProv));

    // DRB.UEThpUl — UL throughput. Not yet populated in E2KpmReport at the
    // v2.1 baseline; the Q4 2026 ContactGraphScheduler + CU/DU/RU split work
    // will fill this. Emit as not-present so the canonical vector stays
    // shape-stable for downstream consumers (FlexRIC xApps).
    out.push_back(
        MakeMeasurement(kpm::kDrbUeThpUl, 0.0, /*present=*/false, base));

    // DRB.PdcpSduVolumeDL: TS 28.552 §5.1.2.1.1.1 reports the DL PDCP SDU
    // data volume in Mbit over the granularity period. This used to emit
    // BYTES (throughput*1e6/8), a factor of 8e6 off the standard's unit.
    // Stand-in derived from the throughput over a 1 s granularity period
    // (E2KpmReport carries no period); will be replaced by a proper
    // integrating counter when the OranNtnDataRepository (§4.1.4) lands.
    // Provenance tracks the throughput it is derived from.
    const double volDlMbit = r.throughput_Mbps * 1.0;
    out.push_back(
        MakeMeasurement(kpm::kDrbPdcpVolumeDl, volDlMbit, /*present=*/true, base, thpProv));

    out.push_back(
        MakeMeasurement(kpm::kDrbPdcpVolumeUl, 0.0, /*present=*/false, base));

    // RRU.PrbAvailDl/Ul — total PRBs configured for the cell. v2.1 assumes a
    // single 100 MHz FR2 carrier at 30 kHz SCS = 273 PRBs (cell-config constant).
    out.push_back(MakeMeasurement(kpm::kRruPrbAvailDl,
                                  kPrbsPerCarrierFr2_100mhz_30khz,
                                  /*present=*/true,
                                  base,
                                  provenance::kDerived));
    out.push_back(MakeMeasurement(kpm::kRruPrbAvailUl,
                                  kPrbsPerCarrierFr2_100mhz_30khz,
                                  /*present=*/true,
                                  base,
                                  provenance::kDerived));

    // RRU.PrbUsedDl/Ul — used PRBs from cell-level prbUtilization. If
    // prbUtilization is NaN (not measured) this is NaN -> synthesized.
    const double prbUsedDl =
        r.prbUtilization * static_cast<double>(kPrbsPerCarrierFr2_100mhz_30khz);
    out.push_back(MakeMeasurement(kpm::kRruPrbUsedDl,
                                  prbUsedDl,
                                  /*present=*/true,
                                  base,
                                  prbProv));
    out.push_back(MakeMeasurement(kpm::kRruPrbUsedUl,
                                  0.0,
                                  /*present=*/false,
                                  base));

    // CARR.AverageSINR — cell-level mean SINR in dB. This is a PHY SINR
    // quantity per 3GPP TS 38.215 (RS-SINR), NOT a TS 28.552 PM counter; it
    // is a legitimate vendor L1 metric (srsRAN / OAI L1). Until per-cell
    // averaging is wired the per-UE sinr is forwarded; reviewers should treat
    // the FIVE_QI label as the disambiguator.
    out.push_back(
        MakeMeasurement(kpm::kCarrAvgSinr, r.sinr_dB, /*present=*/true, base, sinrProv));

    // L1M.RS-SINR.Mean — L1-measured RS-SINR mean (dB), per 3GPP TS 38.215
    // (RS-SINR PHY quantity), NOT a TS 28.552 PM counter. The mmwave PHY emits
    // the same source today; once srsRAN-style per-symbol L1 SINR averaging is
    // in place the two metrics will diverge.
    out.push_back(MakeMeasurement(kpm::kL1mRsSinrMean, r.sinr_dB, /*present=*/true, base,
                                  sinrProv));

    // TB.TotNbrDl / TB.ErrTotNbrDl — canonical TS 28.552 DL transport-block
    // counters; the DL TB error rate is TB.ErrTotNbrDl / TB.TotNbrDl. That
    // error fraction is ALREADY measured upstream as E2KpmReport::harqBler
    // (HARQ NACK ratio from OranNtnPhyKpmExtractor::GetHarqBler()). No
    // integrating absolute-TB counter is plumbed into E2KpmReport at the v2.1
    // baseline, so both counters are emitted as shape-stable present=false
    // placeholders (mirroring DRB.UEThpUl). To avoid discarding the measured
    // information, TB.ErrTotNbrDl carries the already-measured DL HARQ BLER
    // fraction (0..1) so it is greppable; once the integrating counter lands
    // (OranNtnDataRepository §4.1.4) the canonical relation
    // TB.ErrTotNbrDl / TB.TotNbrDl == harqBler holds with absolute counts.
    // NO new physics is introduced here — the BLER is reused as measured.
    out.push_back(
        MakeMeasurement(kpm::kTbTotNbrDl, 0.0, /*present=*/false, base));
    out.push_back(
        MakeMeasurement(kpm::kTbErrTotNbrDl, r.harqBler, /*present=*/false, base));

    return out;
}

void
WriteCanonicalKpmCsv(const std::vector<E2KpmReport>& reports,
                     const std::map<std::string, std::string>& baseLabels,
                     std::ostream& os)
{
    os << "timestamp,gnb_id,is_ntn,ue_id,metric_id,value,present,provenance,"
          "FIVE_QI,S-NSSAI,PLMN\n";
    for (const auto& r : reports)
    {
        // Per-slice 5QI so the canonical stream differentiates the slices:
        // eMBB(0)->9 (default non-GBR), URLLC(1)->82 (delay-critical GBR),
        // mMTC(2)->9. Overrides the base FIVE_QI label per record. mMTC was
        // labelled 79, which TS 23.501 Table 5.7.4-1 assigns to V2X messages,
        // not to MIoT; the toolkit's own mMTC traffic (ntn-traffic
        // NbIotPeriodic) is marked 5QI 9.
        std::map<std::string, std::string> labels = baseLabels;
        labels[label::kFiveQi] =
            (r.sliceId == 1) ? "82" : (r.sliceId == 2) ? "9" : "9";
        // S-NSSAI must follow the slice too: TS 23.501 Table 5.15.2.2-1 SST
        // 1 = eMBB, 2 = URLLC, 3 = MIoT. A URLLC record used to carry the
        // caller's eMBB S-NSSAI ("1-..."). The caller's SD is kept.
        {
            const std::string& sn = labels[label::kSnssai];
            const auto dash = sn.find('-');
            const std::string sd = (dash == std::string::npos) ? std::string() : sn.substr(dash);
            labels[label::kSnssai] =
                std::string((r.sliceId == 1) ? "2" : (r.sliceId == 2) ? "3" : "1") + sd;
        }
        const auto measurements = BuildCanonicalKpmMeasurements(r, labels);
        for (const auto& m : measurements)
        {
            const auto presentIt = m.labels.find(label::kPresent);
            const bool present =
                (presentIt == m.labels.end()) || presentIt->second != "false";
            os << r.timestamp << "," << r.gnbId << ","
               << (r.isNtn ? 1 : 0) << "," << r.ueId << "," << m.metricId
               << ","
               << [&] {
                      // Not measured (NaN) is an empty field, never the text "nan".
                      std::ostringstream v;
                      if (std::isfinite(m.value))
                      {
                          v << m.value;
                      }
                      return v.str();
                  }()
               << "," << (present ? 1 : 0) << ","
               << m.provenance << ","
               << m.labels.at(label::kFiveQi) << ","
               << m.labels.at(label::kSnssai) << ","
               << m.labels.at(label::kPlmn) << "\n";
        }
    }
}

} // namespace oranntn
} // namespace ns3
