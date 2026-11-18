/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#include "ntn-sib19.h"

#include <cmath>

#include "ntn-timing-advance.h"

#include <ns3/log.h>
#include <ns3/mobility-model.h>
#include <ns3/simulator.h>

#include <algorithm>
#include <cstring>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("NtnSib19");

namespace ntnrrc
{

namespace
{

inline void
PutU8(uint8_t*& p, uint8_t v)
{
    *p++ = v;
}

inline void
PutU16(uint8_t*& p, uint16_t v)
{
    *p++ = static_cast<uint8_t>(v & 0xff);
    *p++ = static_cast<uint8_t>((v >> 8) & 0xff);
}

inline void
PutU32(uint8_t*& p, uint32_t v)
{
    *p++ = static_cast<uint8_t>(v & 0xff);
    *p++ = static_cast<uint8_t>((v >> 8) & 0xff);
    *p++ = static_cast<uint8_t>((v >> 16) & 0xff);
    *p++ = static_cast<uint8_t>((v >> 24) & 0xff);
}

inline void
PutI64(uint8_t*& p, int64_t v)
{
    auto u = static_cast<uint64_t>(v);
    for (int i = 0; i < 8; ++i)
    {
        *p++ = static_cast<uint8_t>((u >> (8 * i)) & 0xff);
    }
}

inline void
PutF64(uint8_t*& p, double v)
{
    uint64_t u;
    std::memcpy(&u, &v, sizeof(u));
    for (int i = 0; i < 8; ++i)
    {
        *p++ = static_cast<uint8_t>((u >> (8 * i)) & 0xff);
    }
}

inline uint8_t
GetU8(const uint8_t*& p)
{
    return *p++;
}

inline uint16_t
GetU16(const uint8_t*& p)
{
    uint16_t v = static_cast<uint16_t>(p[0]) | (static_cast<uint16_t>(p[1]) << 8);
    p += 2;
    return v;
}

inline uint32_t
GetU32(const uint8_t*& p)
{
    uint32_t v = static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
                 (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24);
    p += 4;
    return v;
}

inline int64_t
GetI64(const uint8_t*& p)
{
    uint64_t u = 0;
    for (int i = 0; i < 8; ++i)
    {
        u |= static_cast<uint64_t>(p[i]) << (8 * i);
    }
    p += 8;
    return static_cast<int64_t>(u);
}

inline double
GetF64(const uint8_t*& p)
{
    uint64_t u = 0;
    for (int i = 0; i < 8; ++i)
    {
        u |= static_cast<uint64_t>(p[i]) << (8 * i);
    }
    p += 8;
    double v;
    std::memcpy(&v, &u, sizeof(v));
    return v;
}

} // namespace

std::size_t
Sib19Codec::Serialise(const Sib19Content& sib, uint8_t* out, std::size_t len)
{
    // Real runtime guard (NOT NS_ASSERT, which is compiled out in optimized
    // builds and would leave an OOB write). Refuse an under-size buffer.
    if (out == nullptr || len < kSerialisedBytes)
    {
        return 0;
    }
    uint8_t* p = out;

    PutU16(p, sib.cellId);
    PutU8(p, static_cast<uint8_t>(sib.payloadMode));
    PutU8(p, static_cast<uint8_t>(sib.taFrame));
    PutU32(p, sib.cellSpecificKoffset);
    PutU32(p, sib.kMac);
    PutI64(p, sib.taCommon.GetNanoSeconds());
    PutF64(p, sib.taCommonDriftRate);
    PutF64(p, sib.taCommonDriftVariation);
    PutI64(p, sib.ulSyncValidity.GetNanoSeconds());
    PutI64(p, sib.ephemeris.epoch.GetNanoSeconds());
    PutF64(p, sib.ephemeris.positionEcefM.x);
    PutF64(p, sib.ephemeris.positionEcefM.y);
    PutF64(p, sib.ephemeris.positionEcefM.z);
    PutF64(p, sib.ephemeris.velocityEcefMps.x);
    PutF64(p, sib.ephemeris.velocityEcefMps.y);
    PutF64(p, sib.ephemeris.velocityEcefMps.z);
    PutF64(p, sib.referencePosEcefM.x);
    PutF64(p, sib.referencePosEcefM.y);
    PutF64(p, sib.referencePosEcefM.z);

    return static_cast<std::size_t>(p - out);
}

bool
Sib19Codec::Parse(const uint8_t* in, std::size_t len, Sib19Content& sib)
{
    if (len < kSerialisedBytes)
    {
        return false;
    }
    const uint8_t* p = in;

    sib.cellId = GetU16(p);
    sib.payloadMode = static_cast<PayloadMode>(GetU8(p));
    sib.taFrame = static_cast<TaReferenceFrame>(GetU8(p));
    sib.cellSpecificKoffset = GetU32(p);
    sib.kMac = GetU32(p);
    sib.taCommon = NanoSeconds(GetI64(p));
    sib.taCommonDriftRate = GetF64(p);
    sib.taCommonDriftVariation = GetF64(p);
    sib.ulSyncValidity = NanoSeconds(GetI64(p));
    sib.ephemeris.epoch = NanoSeconds(GetI64(p));
    sib.ephemeris.positionEcefM.x = GetF64(p);
    sib.ephemeris.positionEcefM.y = GetF64(p);
    sib.ephemeris.positionEcefM.z = GetF64(p);
    sib.ephemeris.velocityEcefMps.x = GetF64(p);
    sib.ephemeris.velocityEcefMps.y = GetF64(p);
    sib.ephemeris.velocityEcefMps.z = GetF64(p);
    sib.referencePosEcefM.x = GetF64(p);
    sib.referencePosEcefM.y = GetF64(p);
    sib.referencePosEcefM.z = GetF64(p);

    // The wire format carries no CRC of its own, and nothing below this layer
    // in the toolkit's SIB19 path checks the fields either, so a corrupted or
    // out-of-range block used to be accepted and applied: any byte became a
    // PayloadMode, a NaN ephemeris propagated into the UE's TA. Refuse what
    // TS 38.331 cannot encode (SIB19-r17: cellSpecificKoffset-r17 1..1023,
    // kmac-r17 1..512 when present, ntn-UlSyncValidityDuration-r17 at most
    // 900 s, ta-Common-r17 at most about 270 ms, which a 1 s bound covers)
    // and any non-finite number.
    const bool finite = std::isfinite(sib.taCommonDriftRate) &&
                        std::isfinite(sib.taCommonDriftVariation) &&
                        std::isfinite(sib.ephemeris.positionEcefM.x) &&
                        std::isfinite(sib.ephemeris.positionEcefM.y) &&
                        std::isfinite(sib.ephemeris.positionEcefM.z) &&
                        std::isfinite(sib.ephemeris.velocityEcefMps.x) &&
                        std::isfinite(sib.ephemeris.velocityEcefMps.y) &&
                        std::isfinite(sib.ephemeris.velocityEcefMps.z) &&
                        std::isfinite(sib.referencePosEcefM.x) &&
                        std::isfinite(sib.referencePosEcefM.y) &&
                        std::isfinite(sib.referencePosEcefM.z);
    const bool enumsOk = static_cast<uint8_t>(sib.payloadMode) <= 2 &&
                         static_cast<uint8_t>(sib.taFrame) <= 1;
    const bool rangesOk = sib.cellSpecificKoffset >= 1 && sib.cellSpecificKoffset <= 1023 &&
                          sib.kMac <= 512 && sib.taCommon >= Time() &&
                          sib.taCommon <= Seconds(1) && sib.ulSyncValidity > Time() &&
                          sib.ulSyncValidity <= Seconds(900);
    return finite && enumsOk && rangesOk;
}

NS_OBJECT_ENSURE_REGISTERED(NtnSib19Broadcaster);

TypeId
NtnSib19Broadcaster::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::ntnrrc::NtnSib19Broadcaster")
            .SetParent<Object>()
            .SetGroupName("NtnRrc")
            .AddConstructor<NtnSib19Broadcaster>()
            .AddTraceSource(
                "Broadcast",
                "Fires every time a fresh SIB19 has been re-derived",
                MakeTraceSourceAccessor(&NtnSib19Broadcaster::m_broadcastTrace),
                "ns3::ntnrrc::NtnSib19Broadcaster::BroadcastTracedCallback");
    return tid;
}

NtnSib19Broadcaster::NtnSib19Broadcaster() = default;
NtnSib19Broadcaster::~NtnSib19Broadcaster() = default;

void
NtnSib19Broadcaster::DoDispose()
{
    Stop();
    m_sat = nullptr;
    m_ta = nullptr;
    Object::DoDispose();
}

void
NtnSib19Broadcaster::SetSatelliteMobility(Ptr<MobilityModel> sat)
{
    m_sat = sat;
}

void
NtnSib19Broadcaster::SetTimingAdvance(Ptr<NtnTimingAdvance> ta)
{
    m_ta = ta;
}

void
NtnSib19Broadcaster::SetReferencePosition(const Vector& earthFixedRefPosition)
{
    m_referencePos = earthFixedRefPosition;
}

void
NtnSib19Broadcaster::SetCellId(uint16_t cellId)
{
    m_cellId = cellId;
}

void
NtnSib19Broadcaster::SetPayloadMode(PayloadMode mode)
{
    m_payloadMode = mode;
}

void
NtnSib19Broadcaster::SetPeriod(Time period)
{
    NS_ASSERT_MSG(period > Time(0), "SIB19 period must be positive");
    m_period = period;
}

void
NtnSib19Broadcaster::RefreshNow()
{
    NS_ASSERT_MSG(m_sat, "satellite mobility must be set");

    m_latest.cellId = m_cellId;
    m_latest.payloadMode = m_payloadMode;
    m_latest.taFrame = TaReferenceFrame::BeamCenter;
    m_latest.referencePosEcefM = m_referencePos;
    m_latest.ephemeris.positionEcefM = m_sat->GetPosition();
    m_latest.ephemeris.velocityEcefMps = m_sat->GetVelocity();
    m_latest.ephemeris.epoch = Simulator::Now();
    if (m_ta)
    {
        m_latest.taCommon = m_ta->ComputeCommonTa();
        m_latest.taCommonDriftRate = m_ta->ComputeTaDriftRate();

        // GAP R3 FIX: populate cellSpecificKoffset (and kMac) from the common
        // TA instead of leaving them 0 on the wire. TS 38.213 §4.2: K_offset is
        // the scheduling offset that pushes the UL grant / feedback beyond the
        // cell round trip so a UE that has GNSS-pre-compensated its own TA still
        // lands its transmission in the slot the gNB expects. It must cover at
        // least the common (cell-wide) round-trip delay. ComputeCommonTa()
        // returns 2*d/c (the RTT), so K_offset = ceil(RTT / slot) + 1 margin
        // slot. kMac (TS 38.213 §4.2, the DL-config offset for MAC-CE timing) is
        // set to the same coverage; both are in slots of the configured
        // numerology.
        // RRC-7 (2026-09-18). cellSpecificKoffset-r17 is counted in slots of
        // the REFERENCE 15 kHz subcarrier spacing, not in slots of the
        // numerology the cell happens to run. TS 38.331 bounds the field at
        // 1..1023, and that ceiling only makes sense at 15 kHz: 1023 slots of
        // 1 ms covers the ~541 ms GEO round trip, whereas at numerology 1 it
        // would cap at 512 ms and could not express the geometry the field
        // exists for. kmac-r17 uses the same reference spacing in FR1.
        //
        // This previously divided by 1 ms / 2^mu, so a 600 km shell at the
        // default numerology broadcast 9 where the specified derivation gives
        // 5. The value that leaves the broadcaster is now numerology-
        // independent; converting it into the scheduler's own slots is the
        // consumer's job, and NtnRealStackHelper does that conversion.
        constexpr double kRefSlotS = 1.0e-3; // 15 kHz reference SCS
        const double rttS = m_ta->ComputeCommonTa().GetSeconds();
        uint32_t koff =
            static_cast<uint32_t>(std::ceil(rttS / kRefSlotS)) + 1;

        // RRC-6: TS 38.331 bounds these fields, and the code emitted values
        // outside them.
        //
        //   cellSpecificKoffset-r17  INTEGER (1..1023)
        //   kmac-r17                 INTEGER (1..512)
        //
        // A GEO geometry (about 541 ms round trip) at numerology 1 needs 1083
        // slots: past the K_offset ceiling and more than double the kMac one.
        // The old code assigned that number to both fields unclamped, so the
        // broadcast carried values that cannot be encoded in the ASN.1 the
        // struct is named for. Clamping is the honest behaviour, and saying so
        // is the other half: a clamped K_offset no longer covers the round trip,
        // which is a real limitation of the numerology at that geometry rather
        // than something to hide.
        constexpr uint32_t kMaxKoffset = 1023; // TS 38.331 cellSpecificKoffset-r17
        constexpr uint32_t kMaxKmac = 512;     // TS 38.331 kmac-r17
        if (koff < 1)
        {
            koff = 1;
        }
        if (koff > kMaxKoffset)
        {
            NS_LOG_WARN("SIB19 cell " << m_cellId << ": K_offset " << koff
                        << " reference slots exceeds the TS 38.331 ceiling of "
                        << kMaxKoffset << "; clamping. The broadcast offset no longer "
                           "covers the " << rttS * 1e3 << " ms round trip, which the "
                           "field cannot express at this geometry.");
            koff = kMaxKoffset;
        }
        m_latest.cellSpecificKoffset = koff;
        // kMac has a lower ceiling than K_offset, so it needs its own clamp
        // rather than inheriting the same number.
        m_latest.kMac = std::min(koff, kMaxKmac);
        if (koff > kMaxKmac)
        {
            NS_LOG_WARN("SIB19 cell " << m_cellId << ": kMac clamped to " << kMaxKmac
                        << " (K_offset is " << koff << "); TS 38.331 gives kMac the smaller "
                           "range and the two are not interchangeable.");
        }

        // RRC-6: these two were declared and never written, so every broadcast
        // carried 0.0 and the 900 s struct default.
        //
        // ta-CommonDriftVariant-r17 is the rate of change of the drift rate. A
        // UE extrapolating its timing between broadcasts uses a straight line
        // without it, which is worst at the closest approach of a LEO pass where
        // the drift reverses sign fastest.
        // TS 38.213 cl. 4.2 writes the two-way common delay as
        // T_C + T_D tau + T_V tau^2, so ta-CommonDriftVariant is HALF the second
        // derivative. Broadcasting the full curvature doubled the quadratic term
        // at any UE that follows the specification.
        m_latest.taCommonDriftVariation = 0.5 * m_ta->ComputeTaDriftVariation();

        // ul-SyncValidityDuration-r17 is how long the UE may keep using this
        // block. TS 38.331 gives it an enumerated set; 900 s is its MAXIMUM,
        // so leaving the struct default meant every cell, LEO included,
        // broadcast the longest validity the standard allows. Derive it from
        // how fast this geometry actually invalidates: the time for the common
        // TA to drift by one slot, capped to the enumerated set.
        {
            // RRC-7: this block asks how long the broadcast stays usable, which
            // is a question about the cell's own scheduling granularity, so it
            // keeps the configured-numerology slot. Only the K_offset field
            // above is counted in 15 kHz reference slots.
            const double schedSlotS =
                1.0e-3 / std::pow(2.0, static_cast<double>(m_numerology));
            const double drift = std::fabs(m_latest.taCommonDriftRate);
            double validS = 900.0;
            if (drift > 0.0)
            {
                validS = schedSlotS / drift;
            }
            static const double kEnum[] = {5, 10, 15, 20, 25, 30, 35, 40,
                                           45, 50, 55, 60, 120, 180, 240, 900};
            double chosen = kEnum[0];
            for (double v : kEnum)
            {
                if (v <= validS)
                {
                    chosen = v;
                }
            }
            m_latest.ulSyncValidity = Seconds(chosen);
        }
        // P0-6 (TS 38.213 4.2, TS 38.331 NTN-Config-r17 kmac-r17): kMac exists
        // for the case where the gNB's DL and UL frame timing are NOT aligned,
        // which happens when the uplink time synchronization reference point is
        // not at the gNB. With the reference at the gNB/gateway (transparent) or
        // with the gNB on board (regenerative, reference at the satellite) the
        // frames are aligned and kmac-r17 is absent (0 here). Only a transparent
        // payload with the reference at the satellite leaves the feeder round
        // trip between the gNB's DL and UL frames: kMac = ceil(2 d_feeder / c)
        // in 15 kHz reference slots (FR1), clamped to 512. The legacy beam-centre
        // reference keeps its earlier value (kMac = K_offset) for reproducibility.
        if (m_ulRef != UlSyncReference::BeamCenterLegacy)
        {
            uint32_t kmac = 0;
            if (m_ulRef == UlSyncReference::Satellite && m_payloadMode == PayloadMode::Transparent)
            {
                double feederM = m_ta->GetFeederRangeMetres();
                const bool haveGwPos =
                    m_gatewayPos.x != 0.0 || m_gatewayPos.y != 0.0 || m_gatewayPos.z != 0.0;
                if (feederM <= 0.0 && haveGwPos)
                {
                    feederM = CalculateDistance(m_sat->GetPosition(), m_gatewayPos);
                }
                if (feederM > 0.0)
                {
                    const double rttFeederS = 2.0 * feederM / NtnTimingAdvance::kSpeedOfLight;
                    kmac = static_cast<uint32_t>(std::ceil(rttFeederS / kRefSlotS));
                    kmac = std::min(std::max<uint32_t>(kmac, 1), kMaxKmac);
                }
            }
            m_latest.kMac = kmac;
        }
        // RRC-1: hand the broadcast value to whoever schedules with it.
        if (!m_kOffsetSink.IsNull())
        {
            m_kOffsetSink(koff);
        }
    }

    // Oct 2026: specification-conformant common delay (reference point at the gateway or
    // at the satellite), coefficient fit over the validity window, validity checked against
    // the network's own orbit prediction, and quantization to the TS 38.331 field steps.
    if (m_ta && m_ulRef != UlSyncReference::BeamCenterLegacy)
    {
        static const double kEnum[] = {5, 10, 15, 20, 25, 30, 35, 40,
                                       45, 50, 55, 60, 120, 180, 240, 900};
        if (m_ta->HasSatelliteStateProvider())
        {
            // the broadcast state vector is the network's own prediction, errors included
            m_latest.ephemeris.positionEcefM = m_ta->GetSatellitePositionAt(Seconds(0));
            const Vector p1 = m_ta->GetSatellitePositionAt(Seconds(0.5));
            const Vector p0 = m_ta->GetSatellitePositionAt(Seconds(-0.5));
            m_latest.ephemeris.velocityEcefMps = Vector(p1.x - p0.x, p1.y - p0.y, p1.z - p0.z);
        }
        Coefficients cf;
        double window = m_fitWindow.GetSeconds();
        if (m_validityRule == Sib19ValidityRule::ErrorBudget)
        {
            double chosen = 0.0;
            // With the reference point at the satellite there is no common term, so the UE side
            // gets the whole limit; otherwise it gets the share left after the common term.
            const double ueShare =
                (m_ulRef == UlSyncReference::Satellite ? 1.0 : 1.0 - m_commonShare) *
                    m_te.GetSeconds() -
                m_ueAllowance.GetSeconds();
            for (double v : kEnum)
            {
                const Coefficients c = FitCoefficients(v);
                if (CommonResidual(c, v) <= m_commonShare * m_te.GetSeconds() &&
                    PredictEphemerisError(v) <= ueShare)
                {
                    chosen = v;
                }
            }
            if (chosen == 0.0)
            {
                // no enumerated value meets the budget: broadcast the smallest and count it
                ++m_infeasible;
                chosen = kEnum[0];
            }
            window = chosen;
            m_latest.ulSyncValidity = Seconds(window);
        }
        cf = FitCoefficients(window);
        if (m_validityRule == Sib19ValidityRule::Fixed)
        {
            m_latest.ulSyncValidity = Seconds(window);
        }
        if (m_validityRule == Sib19ValidityRule::SlotOverDrift)
        {
            // legacy rule on the broadcast drift: time to drift by one scheduling slot
            const double slot = 1.0e-3 / std::pow(2.0, static_cast<double>(m_numerology));
            const double v = (std::fabs(cf.td) > 0.0) ? slot / std::fabs(cf.td) : 900.0;
            double chosen = kEnum[0];
            for (double e : kEnum)
            {
                if (e <= v)
                {
                    chosen = e;
                }
            }
            m_latest.ulSyncValidity = Seconds(chosen);
        }
        m_lastCommonResidual = CommonResidual(cf, window);
        m_latest.taCommon = Seconds(cf.tc);
        m_latest.taCommonDriftRate = cf.td;
        m_latest.taCommonDriftVariation = cf.tv;
        auto q = [](double x, double step) { return std::round(x / step) * step; };
        Vector& p = m_latest.ephemeris.positionEcefM;
        Vector& u = m_latest.ephemeris.velocityEcefMps;
        p = Vector(q(p.x, 1.3), q(p.y, 1.3), q(p.z, 1.3));
        u = Vector(q(u.x, 0.06), q(u.y, 0.06), q(u.z, 0.06));
    }

    m_latestBytes.assign(Sib19Codec::kSerialisedBytes, 0);
    const auto written =
        Sib19Codec::Serialise(m_latest, m_latestBytes.data(), m_latestBytes.size());
    NS_ASSERT(written == Sib19Codec::kSerialisedBytes);

    m_broadcastTrace(m_latest);
}

double
NtnSib19Broadcaster::ResidualAcceleration(UeEphemerisModel m, double r)
{
    constexpr double kMu = 3.986004418e14;
    constexpr double kRe = 6378137.0;
    constexpr double kJ2 = 1.08262668e-3;
    constexpr double kOmegaE = 7.2921150e-5;
    switch (m)
    {
    case UeEphemerisModel::Linear:
        // gravity (two-body plus the largest J2 term) and the frame terms of the rotating frame
        return kMu / (r * r) * (1.0 + 3.0 * kJ2 * (kRe / r) * (kRe / r)) +
               2.0 * kOmegaE * std::sqrt(kMu / r) + kOmegaE * kOmegaE * r;
    case UeEphemerisModel::GravityEcef:
        // what the rotating-frame two-body propagator leaves out: Coriolis, centrifugal and J2
        return 2.0 * kOmegaE * std::sqrt(kMu / r) + kOmegaE * kOmegaE * r +
               1.5 * kJ2 * kMu * kRe * kRe / std::pow(r, 4) * 2.0;
    case UeEphemerisModel::Kepler:
        // J2 acceleration, largest over latitude (factor 2 at the poles)
        return 1.5 * kJ2 * kMu * kRe * kRe / std::pow(r, 4) * 2.0;
    case UeEphemerisModel::KeplerJ2:
    default:
        // Higher zonal terms and drag left by a J2 propagator. 1.1e-3 m/s^2 is calibrated on the
        // effective residual against SGP4 on the 2026 Starlink and OneWeb element sets (0.68e-3 to
        // 0.85e-3 m/s^2 at the 99th percentile over the four shells), rounded up.
        (void)r;
        return 1.1e-3;
    }
}

NtnSib19Broadcaster::Coefficients
NtnSib19Broadcaster::Quantize(Coefficients c)
{
    // TS 38.331 NTN-Config-r17: ta-Common 0..66485757 x 4.072 ns, ta-CommonDrift
    // -257303..257303 x 0.2 ns/s, ta-CommonDriftVariant 0..28949 x 0.02 ns/s^2 (unsigned).
    auto q = [](double x, double step, double lo, double hi) {
        return std::min(hi, std::max(lo, std::round(x / step))) * step;
    };
    c.tc = q(c.tc, 4.072e-9, 0.0, 66485757.0);
    c.td = q(c.td, 0.2e-9, -257303.0, 257303.0);
    c.tv = q(c.tv, 0.2e-10, 0.0, 28949.0);
    return c;
}

double
NtnSib19Broadcaster::CommonDelayAt(double tau) const
{
    if (m_ulRef != UlSyncReference::Gateway)
    {
        return 0.0;
    }
    const Vector s = m_ta->GetSatellitePositionAt(Seconds(tau));
    return 2.0 * CalculateDistance(s, m_gatewayPos) / NtnTimingAdvance::kSpeedOfLight;
}

NtnSib19Broadcaster::Coefficients
NtnSib19Broadcaster::FitCoefficients(double window) const
{
    Coefficients c;
    if (m_ulRef != UlSyncReference::Gateway)
    {
        return c; // reference at the satellite: no common delay
    }
    const auto d = m_ta->ComputeDelayDerivatives(m_gatewayPos);
    c.tc = d.ta;
    c.td = d.d1;
    c.tv = 0.5 * d.d2;
    if (m_fit == Sib19CoefficientFit::Economized)
    {
        // Best uniform quadratic approximation of tau^3 over [0, V]:
        // (3/2) V tau^2 - (9/16) V^2 tau + V^3/32, error V^3/32 (Chebyshev T3).
        const double a = d.d3 / 6.0;
        const double tv = c.tv + 1.5 * a * window;
        if (tv >= 0.0)
        {
            c.tc += a * window * window * window / 32.0;
            c.td -= a * 9.0 / 16.0 * window * window;
            c.tv = tv;
            return Quantize(c);
        }
        // the unsigned field cannot carry the economized curvature: fall through to the
        // constrained window fit, which handles T_V = 0
    }
    if (m_fit != Sib19CoefficientFit::Taylor)
    {
        // Least squares of the predicted delay at 16 Chebyshev nodes of [0, V] in the scaled
        // variable u = tau / V, with the curvature held inside the unsigned field.
        constexpr int kN = 16;
        double S[5] = {0, 0, 0, 0, 0};
        double B[3] = {0, 0, 0};
        for (int i = 0; i < kN; ++i)
        {
            const double u = 0.5 * (1.0 - std::cos((2.0 * (i + 1) - 1.0) * M_PI / (2.0 * kN)));
            const double y = CommonDelayAt(u * window);
            double up = 1.0;
            for (int k = 0; k < 5; ++k)
            {
                S[k] += up;
                if (k < 3)
                {
                    B[k] += up * y;
                }
                up *= u;
            }
        }
        auto det3 = [](double a, double b, double c2, double d, double e, double f, double g,
                       double hh, double ii) {
            return a * (e * ii - f * hh) - b * (d * ii - f * g) + c2 * (d * hh - e * g);
        };
        const double D = det3(S[0], S[1], S[2], S[1], S[2], S[3], S[2], S[3], S[4]);
        double c0 = det3(B[0], S[1], S[2], B[1], S[2], S[3], B[2], S[3], S[4]) / D;
        double c1 = det3(S[0], B[0], S[2], S[1], B[1], S[3], S[2], B[2], S[4]) / D;
        double c2 = det3(S[0], S[1], B[0], S[1], S[2], B[1], S[2], S[3], B[2]) / D;
        if (c2 < 0.0)
        {
            const double d2 = S[0] * S[2] - S[1] * S[1];
            c0 = (B[0] * S[2] - S[1] * B[1]) / d2;
            c1 = (S[0] * B[1] - S[1] * B[0]) / d2;
            c2 = 0.0;
        }
        c.tc = c0;
        c.td = c1 / window;
        c.tv = c2 / (window * window);
    }
    return Quantize(c);
}

double
NtnSib19Broadcaster::CommonResidual(const Coefficients& c, double window) const
{
    if (m_ulRef != UlSyncReference::Gateway)
    {
        return 0.0;
    }
    double worst = 0.0;
    constexpr int kGrid = 41;
    for (int i = 0; i < kGrid; ++i)
    {
        const double tau = window * i / (kGrid - 1.0);
        const double e = c.tc + c.td * tau + c.tv * tau * tau - CommonDelayAt(tau);
        worst = std::max(worst, std::fabs(e));
    }
    // the feeder delay is predicted from an orbit that errs by up to z sigma
    return worst + 2.0 * m_z * (m_odPos + m_odVel * window) / NtnTimingAdvance::kSpeedOfLight;
}

double
NtnSib19Broadcaster::PredictEphemerisError(double v) const
{
    const Vector p = m_sat->GetPosition();
    const double r = std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
    // (2/c) [ (1/2) a v^2 + |dp| + |dv| v ]: quantization of the state vector, as vector norms
    // of half a step per axis (1.13 m, 0.052 m/s), and the network's own orbit error at z sigma
    const double qp = 0.5 * 1.3 * std::sqrt(3.0);
    const double qv = 0.5 * 0.06 * std::sqrt(3.0);
    return (ResidualAcceleration(m_ueModel, r) * v * v +
            2.0 * (qp + qv * v + m_z * (m_odPos + m_odVel * v))) /
           NtnTimingAdvance::kSpeedOfLight;
}

const Sib19Content&
NtnSib19Broadcaster::GetLatest() const
{
    return m_latest;
}

const std::vector<uint8_t>&
NtnSib19Broadcaster::GetLatestSerialised() const
{
    return m_latestBytes;
}

void
NtnSib19Broadcaster::Start()
{
    if (m_running)
    {
        return;
    }
    m_running = true;
    DoBroadcast();
}

void
NtnSib19Broadcaster::Stop()
{
    m_running = false;
    if (m_event.IsPending())
    {
        Simulator::Cancel(m_event);
    }
}

void
NtnSib19Broadcaster::DoBroadcast()
{
    if (!m_running)
    {
        return;
    }
    RefreshNow();
    m_event = Simulator::Schedule(m_period, &NtnSib19Broadcaster::DoBroadcast, this);
}

} // namespace ntnrrc
} // namespace ns3
