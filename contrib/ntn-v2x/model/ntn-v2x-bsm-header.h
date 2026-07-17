/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#ifndef NTN_V2X_BSM_HEADER_H
#define NTN_V2X_BSM_HEADER_H

#include "ns3/header.h"

#include <cstdint>

namespace ns3
{
namespace ntnv2x
{

/**
 * \ingroup ntn-v2x
 * \brief SAE J2735 Basic Safety Message (BSM) Part I core, as an ns-3 Header.
 *
 * The relay examples previously sent a BSM as an opaque N-byte UDP payload with
 * no structure, so nothing downstream could read a vehicle's real state and the
 * "BSM" was a size label only. This header carries the J2735 BSM Part I
 * (BSMcoreData) fields, filled from the sending vehicle's mobility model, so a
 * receiver (or the relay) reads genuine position/kinematics — the packet is a
 * BSM, not just 300 bytes.
 *
 * Field encodings follow J2735 (2016) BSMcoreData resolutions:
 *   msgCnt    0..127 rolling message counter
 *   id        4-byte temporary station id
 *   secMark   ms of the current minute (0..65535, DE_DSecond)
 *   lat/long  1/10 micro-degree (int32, DE_Latitude/Longitude)
 *   elev      decimetre (int32; J2735 DE_Elevation is 1 dm)
 *   speed     0.02 m/s units (uint16, DE_Speed)
 *   heading   0.0125 degree units (uint16, DE_Heading)
 * The wire size of this core is 25 bytes; the example pads the datagram to its
 * configured BSM size to also account for the (unmodelled) Part II extensions.
 */
class NtnV2xBsmHeader : public Header
{
  public:
    NtnV2xBsmHeader() = default;

    static TypeId GetTypeId();
    TypeId GetInstanceTypeId() const override;
    uint32_t GetSerializedSize() const override;
    void Serialize(Buffer::Iterator start) const override;
    uint32_t Deserialize(Buffer::Iterator start) override;
    void Print(std::ostream& os) const override;

    /// Fill the kinematic fields from SI quantities. lat/lon in degrees,
    /// elevation in metres, speed in m/s, heading in degrees [0,360).
    void SetFromState(uint8_t msgCnt,
                      uint32_t id,
                      uint16_t secMark,
                      double latDeg,
                      double lonDeg,
                      double elevM,
                      double speedMps,
                      double headingDeg);

    uint8_t GetMsgCnt() const { return m_msgCnt; }
    uint32_t GetId() const { return m_id; }
    uint16_t GetSecMark() const { return m_secMark; }
    double GetLatDeg() const { return m_lat / 1.0e7; }
    double GetLonDeg() const { return m_lon / 1.0e7; }
    double GetElevM() const { return m_elev / 10.0; }
    double GetSpeedMps() const { return m_speed * 0.02; }
    double GetHeadingDeg() const { return m_heading * 0.0125; }

  private:
    uint8_t m_msgCnt{0};
    uint32_t m_id{0};
    uint16_t m_secMark{0};
    int32_t m_lat{0};   ///< 1/10 micro-degree
    int32_t m_lon{0};   ///< 1/10 micro-degree
    int32_t m_elev{0};  ///< decimetre
    uint16_t m_speed{0}; ///< 0.02 m/s units
    uint16_t m_heading{0}; ///< 0.0125 degree units
};

} // namespace ntnv2x
} // namespace ns3

#endif // NTN_V2X_BSM_HEADER_H
