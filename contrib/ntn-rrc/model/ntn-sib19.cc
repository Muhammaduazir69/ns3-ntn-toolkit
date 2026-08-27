/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#include "ntn-sib19.h"

#include <cmath>

#include "ntn-timing-advance.h"

#include <ns3/log.h>
#include <ns3/mobility-model.h>
#include <ns3/simulator.h>

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

    return true;
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
        const double slotS = 1.0e-3 / std::pow(2.0, static_cast<double>(m_numerology));
        const double rttS = m_ta->ComputeCommonTa().GetSeconds();
        uint32_t koff =
            static_cast<uint32_t>(std::ceil(rttS / slotS)) + 1;

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
                        << " slots exceeds the TS 38.331 ceiling of " << kMaxKoffset
                        << " at numerology " << static_cast<uint32_t>(m_numerology)
                        << "; clamping. The broadcast offset no longer covers the "
                        << rttS * 1e3 << " ms round trip, which this numerology cannot "
                           "express at this geometry.");
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
        m_latest.taCommonDriftVariation = m_ta->ComputeTaDriftVariation();

        // ul-SyncValidityDuration-r17 is how long the UE may keep using this
        // block. TS 38.331 gives it an enumerated set; 900 s is its MAXIMUM,
        // so leaving the struct default meant every cell, LEO included,
        // broadcast the longest validity the standard allows. Derive it from
        // how fast this geometry actually invalidates: the time for the common
        // TA to drift by one slot, capped to the enumerated set.
        {
            const double drift = std::fabs(m_latest.taCommonDriftRate);
            double validS = 900.0;
            if (drift > 0.0)
            {
                validS = slotS / drift;
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
        // RRC-1: hand the broadcast value to whoever schedules with it.
        if (!m_kOffsetSink.IsNull())
        {
            m_kOffsetSink(koff);
        }
    }

    m_latestBytes.assign(Sib19Codec::kSerialisedBytes, 0);
    const auto written =
        Sib19Codec::Serialise(m_latest, m_latestBytes.data(), m_latestBytes.size());
    NS_ASSERT(written == Sib19Codec::kSerialisedBytes);

    m_broadcastTrace(m_latest);
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
