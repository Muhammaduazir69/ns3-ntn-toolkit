/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#ifndef NTN_SIB19_H
#define NTN_SIB19_H

#include "ntn-rrc-types.h"

#include <ns3/buffer.h>
#include <ns3/nstime.h>
#include <ns3/object.h>
#include <ns3/traced-callback.h>
#include <ns3/vector.h>

#include <cstdint>

namespace ns3
{

class MobilityModel;

namespace ntnrrc
{

class NtnTimingAdvance;

/// Where the uplink time synchronization reference point sits (TS 38.213 cl. 4.2): the
/// common delay ta-Common describes the satellite-to-reference-point leg.
enum class UlSyncReference : uint8_t
{
    BeamCenterLegacy = 0, ///< toolkit convention before Oct 2026: beam centre plus feeder
    Gateway = 1,          ///< reference at the gNB / gateway (transparent payload): feeder leg
    Satellite = 2         ///< reference at the satellite: no common delay is broadcast
};

/// How the network turns the delay derivatives at the epoch into the three coefficients.
enum class Sib19CoefficientFit : uint8_t
{
    Taylor = 0,    ///< T_C, T_D, T_V = TA, TA', TA''/2 at the epoch
    Economized = 1, ///< Chebyshev economization of the cubic Taylor term over the window
    Window = 2      ///< least squares of the predicted orbit at Chebyshev nodes of the window
};

/// How ul-SyncValidityDuration is chosen.
enum class Sib19ValidityRule : uint8_t
{
    SlotOverDrift = 0, ///< legacy: time for the common TA to drift by one slot
    ErrorBudget = 1,   ///< largest enumerated value whose predicted timing error fits Te
    Fixed = 2          ///< the fit window, unchanged (e.g. 5 s, TS 38.133 Table A.3.36.5-1)
};

/// Satellite propagator assumed at the UE when sizing the validity.
enum class UeEphemerisModel : uint8_t
{
    Linear = 0,      ///< straight line in ECEF
    GravityEcef = 1, ///< two-body gravity in the rotating frame, no frame terms
    Kepler = 2,      ///< two-body in an inertial frame
    KeplerJ2 = 3     ///< two-body plus J2
};

/**
 * Satellite ephemeris carried inside NTN-Config-r17 (TS 38.331 §6.3.2).
 *
 * The spec allows two equivalent representations: state vector (position +
 * velocity in ECEF) or Keplerian orbital elements. State vector is the
 * canonical one for simulation because it round-trips through SGP4 cleanly.
 * Units match the IE: positions in 1.25 m steps, velocities in 6.25e-4 m/s
 * steps; we store SI internally and only quantise on serialisation.
 */
struct EphemerisInfo
{
    Vector positionEcefM{0.0, 0.0, 0.0};         //!< satellite position (ECEF, m)
    Vector velocityEcefMps{0.0, 0.0, 0.0};       //!< satellite velocity (ECEF, m/s)
    Time epoch{Seconds(0)};                       //!< validity epoch (UTC)
};

/**
 * SIB19 NTN assistance information (TS 38.331 §6.3.2 SystemInformationBlockType19).
 *
 * The receiver-side decisions a UE makes from this block PER THE SPEC:
 *  - Pre-compensate uplink with `taCommon` (+ derivative drift over time).
 *  - Predict satellite position over `ulSyncValidity` to maintain TA.
 *  - Apply `cellSpecificKoffset` and `kMac` to NR scheduling.
 *  - Decide regenerative-vs-transparent expectations from `payloadMode`.
 *
 * NOTE (honest scope): in this toolkit no UE actually decodes this block, so the
 * above are the *intended* receiver actions, not modelled behaviour. The fields
 * are stored metadata: `cellSpecificKoffset` / `kMac` are now DERIVED from the
 * common TA (R3) and broadcast, but consuming them in the NR scheduler's K1/K2
 * slot timing is a separate follow-on (they are not yet applied to any
 * ns-3 scheduling decision (the mmwave MAC never reads them), and `taCommon` is
 * consumed only by the offline TA accounting, not by a UL timing loop.
 */
struct Sib19Content
{
    EphemerisInfo ephemeris;
    Vector referencePosEcefM{0.0, 0.0, 0.0};      //!< beam centre (ECEF, m)
    TaReferenceFrame taFrame{TaReferenceFrame::BeamCenter};
    Time taCommon{Seconds(0)};                     //!< broadcast TA_common
    double taCommonDriftRate{0.0};                 //!< s/s
    double taCommonDriftVariation{0.0};            //!< T_V (s/s^2): HALF the curvature, TS 38.213 cl. 4.2
    Time ulSyncValidity{Seconds(900)};             //!< how long UE may keep using this block
    uint32_t cellSpecificKoffset{0};               //!< NR scheduling offset (slots)
    uint32_t kMac{0};                              //!< MAC-side offset (slots)
    PayloadMode payloadMode{PayloadMode::Transparent};
    uint16_t cellId{0};
};

/**
 * Serialise/parse Sib19Content into a binary buffer.
 *
 * The wire format is fixed-layout little-endian and not ASN.1 PER (no asn1c
 * dependency in this toolkit). Field widths and ordering are stable across
 * versions; size is exactly `kSerialisedBytes` so receivers can sanity-check.
 */
class Sib19Codec
{
  public:
    static constexpr std::size_t kSerialisedBytes = 124;

    /// Returns the number of bytes written, or 0 if `out` is null or the buffer
    /// is smaller than `kSerialisedBytes` (no bytes are written in that case).
    static std::size_t Serialise(const Sib19Content& sib, uint8_t* out, std::size_t len);
    /// Parse `len` bytes starting at `in` into `sib`. Returns true on success.
    static bool Parse(const uint8_t* in, std::size_t len, Sib19Content& sib);
};

/**
 * \ingroup ntn-rrc
 *
 * Periodically snapshots SIB19 content for a satellite cell and emits it on a
 * `TracedCallback` (and into a private byte buffer via `Sib19Codec`). Re-derives
 * the content from the attached MobilityModel + NtnTimingAdvance every `period`.
 *
 * NOTE (honest scope): this is NOT a real over-the-air broadcast. The content is
 * never carried on BCCH/PDSCH and no UE decodes it; consumers (loggers / external
 * emulators) read `GetLatest()` or the `Broadcast` trace to observe the most
 * recent snapshot. The cadence is the only "broadcast" semantics modelled.
 *
 * Default period 160 ms matches the `si-Periodicity-r17` 16-frame value used in
 * the Rel-17 NTN reference scenarios.
 */
class NtnSib19Broadcaster : public Object
{
  public:
    static TypeId GetTypeId();

    NtnSib19Broadcaster();
    ~NtnSib19Broadcaster() override;

    void SetSatelliteMobility(Ptr<MobilityModel> sat);
    void SetTimingAdvance(Ptr<NtnTimingAdvance> ta);
    void SetReferencePosition(const Vector& earthFixedRefPosition);
    void SetCellId(uint16_t cellId);
    void SetPayloadMode(PayloadMode mode);
    void SetPeriod(Time period);
    /// R3: numerology used to convert the common TA (RTT) into the
    /// cellSpecificKoffset / kMac slot counts (TS 38.213 §4.2). Slot duration
    /// = 1 ms / 2^numerology. Default 1 (30 kHz SCS).
    void SetNumerology(uint8_t mu) { m_numerology = mu; }

    /// Uplink time synchronization reference point (TS 38.213 cl. 4.2). Gateway needs
    /// SetGatewayPosition(); Satellite broadcasts a zero common delay.
    void SetUlSyncReference(UlSyncReference ref) { m_ulRef = ref; }
    void SetGatewayPosition(const Vector& ecef) { m_gatewayPos = ecef; }
    void SetCoefficientFit(Sib19CoefficientFit fit) { m_fit = fit; }
    /// Window of the economized fit when the validity rule does not set it.
    void SetFitWindow(Time window) { m_fitWindow = window; }
    void SetValidityRule(Sib19ValidityRule rule) { m_validityRule = rule; }
    /// Uplink timing error limit Te (TS 38.133 cl. 7.1.2) and the share of it given to the
    /// common-delay polynomial; the rest goes to the UE's ephemeris propagation.
    void SetTimingErrorLimit(Time te, double commonShare = 0.5)
    {
        m_te = te;
        m_commonShare = commonShare;
    }
    void SetAssumedUeModel(UeEphemerisModel m) { m_ueModel = m; }
    /// One-sigma error of the network's own orbit prediction (position m, velocity m/s) and the
    /// multiple z of sigma at which its norm is budgeted (3: P(|eta| > 3 sigma) about 0.4 % for an
    /// along-track-dominated error); zero sigma means the network trusts its orbit exactly.
    void SetOrbitErrorBudget(double sigmaPosM, double sigmaVelMps, double z = 3.0)
    {
        m_odPos = sigmaPosM;
        m_odVel = sigmaVelMps;
        m_z = z;
    }
    /// Part of Te reserved for the UE's own position (GNSS error, motion since the fix).
    void SetUeAllowance(Time a) { m_ueAllowance = a; }
    /// Number of refreshes at which no enumerated validity met the budget (5 s was broadcast).
    uint64_t GetInfeasibleCount() const { return m_infeasible; }
    /// The three broadcast coefficients (s, s/s, s/s^2), already quantized.
    struct Coefficients
    {
        double tc{0.0};
        double td{0.0};
        double tv{0.0};
    };
    /// Coefficients the network would broadcast for a validity window of `window` seconds.
    Coefficients FitCoefficients(double window) const;
    /// Largest deviation of the coefficients from the predicted common delay over [0, window] (s).
    double CommonResidual(const Coefficients& c, double window) const;
    /// Residual of the last broadcast over its own validity window (s).
    double GetLastCommonResidual() const { return m_lastCommonResidual; }
    /// Predicted worst-case ephemeris-propagation error over [0, v] for the assumed UE model (s).
    double PredictEphemerisError(double v) const;
    /// Residual line-of-sight acceleration left by a UE propagator at radius r (m/s^2).
    static double ResidualAcceleration(UeEphemerisModel m, double radiusM);

    /// Force an immediate refresh of the broadcast content.
    void RefreshNow();
    /// Most recent broadcast content (by reference; do not mutate).
    const Sib19Content& GetLatest() const;
    /// Bytes written by the most recent broadcast (for downstream emulators
    /// that move bits across a real link).
    const std::vector<uint8_t>& GetLatestSerialised() const;

    /// Start the periodic broadcast loop. Idempotent.
    void Start();
    /// Stop the periodic loop without destroying the object.
    void Stop();


    /// RRC-1: register a consumer for cellSpecificKoffset.
    ///
    /// The field was populated on the wire and read by nothing: the NR
    /// scheduler independently re-derived its own K_offset from its own
    /// geometry and numerology. The two could disagree without any test
    /// noticing, leaving the network scheduling against a value it had never
    /// broadcast. Wire this to NtnRealStackHelper::SetBroadcastKOffsetSlots()
    /// to make SIB19 the single source of truth.
    void SetKOffsetSink(Callback<void, uint32_t> sink) { m_kOffsetSink = sink; }

  protected:
    void DoDispose() override;


  private:
    void DoBroadcast();

    Ptr<MobilityModel> m_sat;
    Ptr<NtnTimingAdvance> m_ta;
    Vector m_referencePos{0.0, 0.0, 0.0};
    uint16_t m_cellId{0};
    PayloadMode m_payloadMode{PayloadMode::Transparent};
    uint8_t m_numerology{1}; // R3: for K_offset slot conversion
    UlSyncReference m_ulRef{UlSyncReference::BeamCenterLegacy};
    Vector m_gatewayPos{0.0, 0.0, 0.0};
    Sib19CoefficientFit m_fit{Sib19CoefficientFit::Taylor};
    Time m_fitWindow{Seconds(10)};
    Sib19ValidityRule m_validityRule{Sib19ValidityRule::SlotOverDrift};
    Time m_te{NanoSeconds(716.15)}; ///< Te_NTN, FR1-NTN 30/30 kHz, TS 38.133 Table 7.1C.2-1
    double m_commonShare{0.5};
    UeEphemerisModel m_ueModel{UeEphemerisModel::KeplerJ2};
    double m_lastCommonResidual{0.0};
    double m_odPos{0.0};
    double m_odVel{0.0};
    double m_z{3.0};
    Time m_ueAllowance{Seconds(0)};
    uint64_t m_infeasible{0};
    static Coefficients Quantize(Coefficients c);
    double CommonDelayAt(double tau) const;

    Time m_period{MilliSeconds(160)};
    bool m_running{false};
    EventId m_event;

    Sib19Content m_latest;
    std::vector<uint8_t> m_latestBytes;

    TracedCallback<const Sib19Content&> m_broadcastTrace;
    /// RRC-1: consumer for the broadcast K_offset. Fired on every refresh so the
    /// scheduler can adopt the value the network actually advertised.
    Callback<void, uint32_t> m_kOffsetSink;
};

} // namespace ntnrrc
} // namespace ns3

#endif // NTN_SIB19_H
