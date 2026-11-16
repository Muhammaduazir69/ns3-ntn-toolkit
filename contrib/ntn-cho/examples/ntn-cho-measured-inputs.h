/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026 Muhammad Uzair
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Shared by the ntn-cho real-stack examples (Wave 2 M1, P0-2): the MEASURED
 * inputs a CHO decision is allowed to use.
 *
 * 1. Serving DL SINR measured at the UE PHY: NtnRealStackHelper::
 *    TakeUeSinrWindowMeanDb(ue), the linear mean of the UE's per-TB SINR over
 *    the decision window (TS 38.215 5.1.6 defines SINR as a linear average),
 *    keyed by the UE index so it is continuous across handovers (nr
 *    ntn-patch 10 labels the UE trace with the serving cell). It replaced the
 *    example-local UeSinrProbe of Wave 2 M1 (same numbers).
 * 2. Candidate quality from the UE's own measurement reports (TS 38.331 5.5.5),
 *    already L3-filtered at the UE (5.5.3.2): serving SINR plus the reported
 *    RSRP difference Mn - Mp. Nothing geometric enters.
 */

#ifndef NTN_CHO_MEASURED_INPUTS_H
#define NTN_CHO_MEASURED_INPUTS_H

#include "ns3/ntn-real-stack-helper.h"

#include <cmath>
#include <limits>
#include <string>

namespace ns3
{
namespace ntnchoex
{

/// Candidate quality from the UE's measurement reports.
struct ReportedCandidate
{
    double servRsrpDbm{std::numeric_limits<double>::quiet_NaN()};
    double candRsrpDbm{std::numeric_limits<double>::quiet_NaN()};
    double ageS{-1.0};
    double candSinrDb{std::numeric_limits<double>::quiet_NaN()};
    bool fresh{false};
};

/// Serving SINR plus the reported (Mn - Mp); fresh = both reported within maxAge.
inline ReportedCandidate
ReadReportedCandidate(const NtnRealStackHelper& rs,
                      uint32_t ueIndex,
                      uint16_t candCell,
                      double servSinrDb,
                      Time maxAge = Seconds(2.0))
{
    ReportedCandidate r;
    r.servRsrpDbm = rs.GetUeServingRsrpDbm(ueIndex);
    r.candRsrpDbm = rs.GetUeNeighbourRsrpDbm(ueIndex, candCell);
    const Time age = rs.GetUeMeasReportAge(ueIndex, candCell);
    r.ageS = age.GetSeconds();
    r.fresh = !std::isnan(r.servRsrpDbm) && !std::isnan(r.candRsrpDbm) && age >= Time() &&
              age <= maxAge && !std::isnan(servSinrDb);
    if (r.fresh)
    {
        r.candSinrDb = servSinrDb + (r.candRsrpDbm - r.servRsrpDbm);
    }
    return r;
}

} // namespace ntnchoex
} // namespace ns3

#endif // NTN_CHO_MEASURED_INPUTS_H
