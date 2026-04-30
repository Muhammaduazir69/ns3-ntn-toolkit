/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-only
 * Author: Muhammad Uzair
 *
 * ISL Protocol Header for Space RIC Coordination
 *
 * Lightweight fixed-format header for inter-satellite link messages
 * used by Space RICs for coordination. Avoids Protobuf dependency
 * while supporting all ISL message types.
 */

#ifndef ORAN_NTN_ISL_HEADER_H
#define ORAN_NTN_ISL_HEADER_H

#include <ns3/header.h>

namespace ns3
{

/**
 * \brief ISL message types for Space RIC coordination
 */
enum class IslMessageType : uint8_t
{
    KPM_EXCHANGE = 0,          //!< Beam load and KPM state exchange
    MODEL_GRADIENTS = 1,       //!< Federated learning gradient exchange
    HO_COORDINATION = 2,       //!< Inter-satellite handover preparation
    FL_ROUND = 3,              //!< Federated learning round coordination
    BEAM_COORDINATION = 4,     //!< Cross-satellite beam interference coordination
    ENERGY_STATE = 5,          //!< Energy state sharing for coordinated power mgmt
    ROUTING_UPDATE = 6,        //!< ISL routing table update
    HEARTBEAT = 7,             //!< Keep-alive / link status
};

/**
 * \ingroup oran-ntn
 * \brief Header for ISL messages between Space RICs
 */
class OranNtnIslHeader : public Header
{
  public:
    static TypeId GetTypeId();
    OranNtnIslHeader();
    ~OranNtnIslHeader() override;

    // ---- Field accessors ----
    void SetMessageType(IslMessageType type);
    IslMessageType GetMessageType() const;

    void SetSourceSatId(uint32_t satId);
    uint32_t GetSourceSatId() const;

    void SetDestSatId(uint32_t satId);
    uint32_t GetDestSatId() const;

    void SetPayloadSize(uint32_t size);
    uint32_t GetPayloadSize() const;

    void SetTimestamp(double timestamp);
    double GetTimestamp() const;

    void SetSequenceNumber(uint32_t seq);
    uint32_t GetSequenceNumber() const;

    void SetPriority(uint8_t priority);
    uint8_t GetPriority() const;

    void SetTtl(uint8_t ttl);
    uint8_t GetTtl() const;

    // ---- Header interface ----
    TypeId GetInstanceTypeId() const override;
    void Print(std::ostream& os) const override;
    uint32_t GetSerializedSize() const override;
    void Serialize(Buffer::Iterator start) const override;
    uint32_t Deserialize(Buffer::Iterator start) override;

  private:
    uint8_t m_messageType;
    uint32_t m_srcSatId;
    uint32_t m_dstSatId;
    uint32_t m_payloadSize;
    double m_timestamp;
    uint32_t m_sequenceNumber;
    uint8_t m_priority;
    uint8_t m_ttl;
};

} // namespace ns3

#endif // ORAN_NTN_ISL_HEADER_H
