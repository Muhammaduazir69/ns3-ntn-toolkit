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

TypeId
NtnV2xBsmHeader::GetInstanceTypeId() const
{
    return GetTypeId();
}

uint32_t
NtnV2xBsmHeader::GetSerializedSize() const
{
    // 1 (msgCnt) + 4 (id) + 2 (secMark) + 4 + 4 (lat/lon) + 4 (elev)
    // + 2 (speed) + 2 (heading) = 23 bytes core.
    return 23;
}

void
NtnV2xBsmHeader::Serialize(Buffer::Iterator start) const
{
    start.WriteU8(m_msgCnt);
    start.WriteHtonU32(m_id);
    start.WriteHtonU16(m_secMark);
    start.WriteHtonU32(static_cast<uint32_t>(m_lat));
    start.WriteHtonU32(static_cast<uint32_t>(m_lon));
    start.WriteHtonU32(static_cast<uint32_t>(m_elev));
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
    m_elev = static_cast<int32_t>(start.ReadNtohU32());
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
    m_elev = static_cast<int32_t>(std::llround(elevM * 10.0));
    m_speed = static_cast<uint16_t>(
        std::min<double>(65534.0, std::max<double>(0.0, std::llround(speedMps / 0.02))));
    double h = std::fmod(headingDeg, 360.0);
    if (h < 0.0)
    {
        h += 360.0;
    }
    m_heading = static_cast<uint16_t>(std::min<double>(28799.0, std::llround(h / 0.0125)));
}

} // namespace ntnv2x
} // namespace ns3
