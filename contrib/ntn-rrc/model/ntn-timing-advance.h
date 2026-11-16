/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#ifndef NTN_TIMING_ADVANCE_H
#define NTN_TIMING_ADVANCE_H

#include "ntn-rrc-types.h"

#include <ns3/nstime.h>
#include <ns3/callback.h>
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

    /// RRC-2: the satellite-to-gateway feeder endpoint.
    ///
    /// In Transparent (bent-pipe) mode the gNB is on the ground, so the uplink
    /// traverses the feeder link as well as the service link and the timing
    /// advance must cover both. Without this the class had no gateway geometry
    /// and returned the service-link round trip for every payload mode, which
    /// made m_payloadMode dead and under-compensated a transparent LEO cell by
    /// the feeder round trip (TR 38.821 Table 4.2-2 gives a 25.77 ms worst-case
    /// RTT for LEO-600 transparent, 41.77 ms for LEO-1200, against about 4 ms
    /// for the LEO-600 service link alone at zenith).
    void SetGatewayMobility(Ptr<MobilityModel> gw);

    /// True when the configured payload mode needs a feeder leg that has not
    /// been supplied, i.e. the returned TA is knowingly short.
    bool FeederGeometryMissing() const
    {
        return m_payloadMode == PayloadMode::Transparent && !m_gw;
    }

    /// Total TA: round-trip service-link delay (2*d/c) for both payload modes.
    Time ComputeTotalTa() const;
    /// Common TA referenced to `m_referencePos` — the value broadcast in SIB19.
    Time ComputeCommonTa() const;
    /// UE-specific residual TA = total − common.
    Time ComputeUeSpecificTa() const;
    /// Instantaneous TA drift rate (s/s). Sign convention: positive when
    /// the satellite is receding from the UE.
    double ComputeTaDriftRate(Time eps = MilliSeconds(10)) const;

    /**
     * \brief RRC-6: second derivative of TA_common, in (s/s)/s.
     *
     * SIB19's ta-CommonDriftVariant-r17 is the RATE OF CHANGE of the drift
     * rate. Without it a UE extrapolating its timing between broadcasts uses a
     * straight line, which is exactly wrong near the closest approach of a LEO
     * pass, where the drift rate reverses sign fastest. The field was declared
     * and never populated, so every broadcast carried 0.
     *
     * Central difference on ComputeTaDriftRate over +/- eps, computed against
     * projected positions without advancing the global clock.
     */
    double ComputeTaDriftVariation(Time eps = MilliSeconds(10)) const;

    /// Slant range in metres from `m_ue` to `m_sat` at the current sim time.
    /// Satellite ECEF position and velocity at Simulator::Now() + offset. When a provider is
    /// set (for example Sgp4MobilityModel::GetEcefStateAt), the derivatives below come from
    /// the orbit the satellite actually flies; otherwise from a straight-line projection,
    /// which omits the orbital acceleration (about 8 m/s^2 in LEO) and so mis-states the
    /// curvature of the delay by several percent and its rate of change by far more.
    typedef Callback<void, Time, Vector&, Vector&> SatStateProvider;
    void SetSatelliteStateProvider(SatStateProvider provider);
    bool HasSatelliteStateProvider() const { return !m_satState.IsNull(); }
    /// Satellite ECEF position at Now() + offset (provider, else straight-line projection).
    Vector GetSatellitePositionAt(Time offset) const;

    /// Two-way delay from the satellite to a fixed Earth-fixed point and its first three
    /// time derivatives (s, s/s, s/s^2, s/s^3), by five-point central differences of step h.
    struct DelayDerivatives
    {
        double ta{0.0};
        double d1{0.0};
        double d2{0.0};
        double d3{0.0};
    };
    DelayDerivatives ComputeDelayDerivatives(const Vector& earthFixedPoint,
                                             Time offset = Seconds(0),
                                             Time h = Seconds(0.5)) const;

    double GetSlantRangeMetres() const;
    /// Same, but to the reference position (used for TA_common).
    double GetReferenceRangeMetres() const;
    /// RRC-2: satellite-to-gateway range, zero unless Transparent with a gateway set.
    double GetFeederRangeMetres() const;

    /// Speed of light in vacuum (m/s) — exposed so tests can sanity-check.
    static constexpr double kSpeedOfLight = 299792458.0;

  private:
    Ptr<MobilityModel> m_ue;
    Ptr<MobilityModel> m_sat;
    Ptr<MobilityModel> m_gw; ///< RRC-2: feeder/gateway endpoint (Transparent mode)
    Vector m_referencePos{0.0, 0.0, 0.0};
    PayloadMode m_payloadMode{PayloadMode::Transparent};
    SatStateProvider m_satState; ///< true-orbit state at an offset, if available

    TracedCallback<Time, Time, Time> m_taTrace; //!< (total, common, ue-specific)
};

} // namespace ntnrrc
} // namespace ns3

#endif // NTN_TIMING_ADVANCE_H
