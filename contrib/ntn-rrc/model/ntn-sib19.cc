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
        const uint32_t koff =
            static_cast<uint32_t>(std::ceil(rttS / slotS)) + 1;
        m_latest.cellSpecificKoffset = koff;
        m_latest.kMac = koff;
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
