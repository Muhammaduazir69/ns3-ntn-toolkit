/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#include "ntn-v2x-bsm-header.h"

#include "ns3/log.h"

#include <algorithm>
#include <cmath>

namespace ns3
{
namespace ntnv2x
{

NS_LOG_COMPONENT_DEFINE("NtnV2xBsmHeader");
NS_OBJECT_ENSURE_REGISTERED(NtnV2xBsmHeader);

TypeId
NtnV2xBsmHeader::GetTypeId()
{
    static TypeId tid = TypeId("ns3::ntnv2x::NtnV2xBsmHeader")
                            .SetParent<Header>()
                            .SetGroupName("NtnV2x")
                            .AddConstructor<NtnV2xBsmHeader>();
    return tid;
}

namespace
{
// V2X-7: SAE J2735 data-element ranges, so the clamps are the standard's and
// not the C++ type's.
constexpr int32_t kElevMin = -4096;   //!< DE_Elevation lower bound (also "unavailable")
constexpr int32_t kElevMax = 61439;   //!< DE_Elevation upper bound
constexpr uint16_t kSpeedMax = 8190;  //!< DE_Speed: 8191 is reserved for "unavailable"
} // namespace

TypeId
NtnV2xBsmHeader::GetInstanceTypeId() const
{
    return GetTypeId();
}

uint32_t
NtnV2xBsmHeader::GetSerializedSize() const
{
    // V2X-7: elevation is TWO octets, not four.
    //
    // SAE J2735 DE_Elevation is INTEGER (-4096..61439) in units of 0.1 m, with
    // -4096 meaning unavailable: a 16-bit field. This wrote it with
    // WriteHtonU32, so the encoded message was two bytes longer than the
    // standard it is named for and could carry values no conforming decoder
    // accepts.
    //
    // 1 (msgCnt) + 4 (id) + 2 (secMark) + 4 + 4 (lat/lon) + 2 (elev)
    // + 2 (speed) + 2 (heading) = 21 bytes core.
    return 21;
}

void
NtnV2xBsmHeader::Serialize(Buffer::Iterator start) const
{
    start.WriteU8(m_msgCnt);
    start.WriteHtonU32(m_id);
    start.WriteHtonU16(m_secMark);
    start.WriteHtonU32(static_cast<uint32_t>(m_lat));
    start.WriteHtonU32(static_cast<uint32_t>(m_lon));
    // V2X-7: two octets, as the UPER encoding of a constrained INTEGER.
    //
    // DE_Elevation is INTEGER (-4096..61439). That span is 65536 values, so it
    // fits exactly two octets, but NOT as a signed 16-bit int: 61439 is past
    // INT16_MAX. UPER encodes a constrained integer as an unsigned offset from
    // the lower bound, which is what makes the full range fit. Writing it as a
    // signed cast turned a legal 6000 m into -553.6 m.
    start.WriteHtonU16(static_cast<uint16_t>(
        std::max<int32_t>(kElevMin, std::min<int32_t>(kElevMax, m_elev)) - kElevMin));
    start.WriteHtonU16(m_speed);
    start.WriteHtonU16(m_heading);
}

uint32_t
NtnV2xBsmHeader::Deserialize(Buffer::Iterator start)
{
    m_msgCnt = start.ReadU8();
    m_id = start.ReadNtohU32();
    m_secMark = start.ReadNtohU16();
    m_lat = static_cast<int32_t>(start.ReadNtohU32());
    m_lon = static_cast<int32_t>(start.ReadNtohU32());
    // V2X-7: undo the UPER offset from the lower bound.
    m_elev = static_cast<int32_t>(start.ReadNtohU16()) + kElevMin;
    m_speed = start.ReadNtohU16();
    m_heading = start.ReadNtohU16();
    return GetSerializedSize();
}

void
NtnV2xBsmHeader::Print(std::ostream& os) const
{
    os << "BSM msgCnt=" << static_cast<uint32_t>(m_msgCnt) << " id=" << m_id
       << " secMark=" << m_secMark << " lat=" << GetLatDeg() << " lon=" << GetLonDeg()
       << " elev=" << GetElevM() << "m speed=" << GetSpeedMps() << "m/s heading=" << GetHeadingDeg()
       << "deg";
}

void
NtnV2xBsmHeader::SetFromState(uint8_t msgCnt,
                              uint32_t id,
                              uint16_t secMark,
                              double latDeg,
                              double lonDeg,
                              double elevM,
                              double speedMps,
                              double headingDeg)
{
    m_msgCnt = msgCnt & 0x7f; // J2735 msgCnt is 0..127
    m_id = id;
    m_secMark = secMark;
    m_lat = static_cast<int32_t>(std::llround(latDeg * 1.0e7));
    m_lon = static_cast<int32_t>(std::llround(lonDeg * 1.0e7));
    // V2X-7: clamp into the J2735 ranges rather than into the width of the C++
    // type that happened to be used.
    //
    // DE_Elevation: INTEGER (-4096..61439), units 0.1 m, -4096 = unavailable.
    // DE_Speed:     INTEGER (0..8191),      units 0.02 m/s, 8191 = unavailable,
    //               so the largest REPRESENTABLE speed is 8190 * 0.02 =
    //               163.80 m/s. This used to clamp at 65534 units = 1310.68
    //               m/s, a value no conforming decoder can express and roughly
    //               eight times the standard's ceiling.
    {
        const int64_t e = std::llround(elevM * 10.0);
        m_elev = static_cast<int32_t>(std::max<int64_t>(kElevMin, std::min<int64_t>(kElevMax, e)));
        const int64_t sp = std::llround(speedMps / 0.02);
        m_speed = static_cast<uint16_t>(
            std::max<int64_t>(0, std::min<int64_t>(kSpeedMax, sp)));
    }
    double h = std::fmod(headingDeg, 360.0);
    if (h < 0.0)
    {
        h += 360.0;
    }
    m_heading = static_cast<uint16_t>(std::min<double>(28799.0, std::llround(h / 0.0125)));
}

} // namespace ntnv2x
} // namespace ns3
