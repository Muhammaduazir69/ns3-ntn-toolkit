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
 * are stored metadata: `cellSpecificKoffset` / `kMac` are NOT applied to any
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
    double taCommonDriftVariation{0.0};            //!< (s/s)/s
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

  protected:
    void DoDispose() override;

  private:
    void DoBroadcast();

    Ptr<MobilityModel> m_sat;
    Ptr<NtnTimingAdvance> m_ta;
    Vector m_referencePos{0.0, 0.0, 0.0};
    uint16_t m_cellId{0};
    PayloadMode m_payloadMode{PayloadMode::Transparent};
    Time m_period{MilliSeconds(160)};
    bool m_running{false};
    EventId m_event;

    Sib19Content m_latest;
    std::vector<uint8_t> m_latestBytes;

    TracedCallback<const Sib19Content&> m_broadcastTrace;
};

} // namespace ntnrrc
} // namespace ns3

#endif // NTN_SIB19_H
