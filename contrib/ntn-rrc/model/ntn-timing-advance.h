/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#ifndef NTN_TIMING_ADVANCE_H
#define NTN_TIMING_ADVANCE_H

#include "ntn-rrc-types.h"

#include <ns3/nstime.h>
#include <ns3/object.h>
#include <ns3/traced-callback.h>
#include <ns3/vector.h>

namespace ns3
{

class MobilityModel;

namespace ntnrrc
{

/**
 * \ingroup ntn-rrc
 *
 * Ephemeris-driven Timing Advance pre-compensation for NR-NTN
 * (3GPP TS 38.213 §4.2.2 + TR 38.821 §6.3.3).
 *
 * In NTN the round-trip propagation delay (1–13 ms LEO, ~125 ms GEO) far
 * exceeds NR's native maximum TA (~667 µs at 15 kHz SCS). Without
 * pre-compensation, RACH preambles arrive far outside their reception window.
 *
 * This class computes:
 *   TA_total      = 2 * d_path / c           (round-trip; both payload modes)
 *   TA_common     = 2 * d_to_reference / c   (broadcast in SIB19)
 *   TA_UE-specific = TA_total - TA_common
 *   TA_drift      = d/dt(TA_total)           (used to schedule TA refresh)
 *
 * Distances are taken from the attached MobilityModel pair (UE + satellite),
 * which in this toolkit are populated by SatelliteSGP4MobilityModel (SNS3)
 * fed from `ntn-constellation`.
 */
class NtnTimingAdvance : public Object
{
  public:
    static TypeId GetTypeId();

    NtnTimingAdvance();
    ~NtnTimingAdvance() override;

    /// Configure the UE-side mobility (ground location).
    void SetUeMobility(Ptr<MobilityModel> ue);
    /// Configure the satellite mobility (the gNB anchor for transparent mode,
    /// the gNB itself for regenerative mode).
    void SetSatelliteMobility(Ptr<MobilityModel> sat);
    /// Reference point for TA_common (typically beam centre on the ground).
    void SetReferencePosition(const Vector& earthFixedRefPosition);
    void SetPayloadMode(PayloadMode mode);

    /// Total TA: round-trip service-link delay (2*d/c) for both payload modes.
    Time ComputeTotalTa() const;
    /// Common TA referenced to `m_referencePos` — the value broadcast in SIB19.
    Time ComputeCommonTa() const;
    /// UE-specific residual TA = total − common.
    Time ComputeUeSpecificTa() const;
    /// Instantaneous TA drift rate (s/s). Sign convention: positive when
    /// the satellite is receding from the UE.
    double ComputeTaDriftRate(Time eps = MilliSeconds(10)) const;

    /// Slant range in metres from `m_ue` to `m_sat` at the current sim time.
    double GetSlantRangeMetres() const;
    /// Same, but to the reference position (used for TA_common).
    double GetReferenceRangeMetres() const;

    /// Speed of light in vacuum (m/s) — exposed so tests can sanity-check.
    static constexpr double kSpeedOfLight = 299792458.0;

  private:
    Ptr<MobilityModel> m_ue;
    Ptr<MobilityModel> m_sat;
    Vector m_referencePos{0.0, 0.0, 0.0};
    PayloadMode m_payloadMode{PayloadMode::Transparent};

    TracedCallback<Time, Time, Time> m_taTrace; //!< (total, common, ue-specific)
};

} // namespace ntnrrc
} // namespace ns3

#endif // NTN_TIMING_ADVANCE_H
