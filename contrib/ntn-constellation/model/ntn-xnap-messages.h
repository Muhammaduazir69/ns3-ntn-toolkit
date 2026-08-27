/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only
//
// ntn-xnap-messages - inter-satellite Xn handover signalling.
//
// Why this exists (audit SAGIN-3). A repo-wide grep for 'Xn' or 'XnAP' across
// ntn-sagin and ntn-constellation returned ZERO hits. RegenMode{bent_pipe,
// regen_du, regen_cu, regen_full} existed but was used purely as a transit
// filter inside ShortestPath, and nothing outside contact-graph-router and its
// own unit test ever set it. So there was no inter-satellite mobility
// signalling of any kind: no Handover Request, no SN Status Transfer, no UE
// Context Release. A regenerative-payload handover study - the main Rel-19 NTN
// research topic - could not be run, while the enum implied it could.
//
// The real gNB half of that finding already existed: NtnRealStackHelper builds
// an NrGnbNetDevice on the satellite node under PayloadOption::FullGnb, with
// the X2/Xn delay derived from live inter-satellite geometry. What was absent
// is the PROCEDURE. This supplies it.
//
// Standards: 3GPP TS 38.423 (XnAP) section 8.2.1 Handover Preparation,
// section 8.2.2 SN Status Transfer, section 8.2.3 UE Context Release;
// TS 38.300 section 9.2.3 for the Xn-based handover sequence.
//
// ON THE WIRE FORMAT, PLAINLY. These are ns-3 Headers with a fixed
// little-endian layout, NOT ASN.1 APER, and the message-type values are
// toolkit-internal rather than TS 38.423 procedure codes. The same honesty
// applies here as to the E2 framing in oran-ntn: what is modelled is the
// PROCEDURE - which messages flow in which order, carrying which identifiers,
// over a link whose delay comes from real orbital geometry - not a
// bit-conformant XnAP encoding. An asn1c peer would not decode these.

#ifndef NTN_XNAP_MESSAGES_H
#define NTN_XNAP_MESSAGES_H

#include "ns3/header.h"
#include "ns3/nstime.h"

#include <cstdint>
#include <string>

namespace ns3
{
namespace ntncon
{

/// XnAP message types. Values are TOOLKIT-INTERNAL, not TS 38.423 procedure
/// codes: the standard's codes live in the ASN.1 InitiatingMessage tag, and
/// inventing values for them would be a standards claim this cannot support.
enum class XnapMessageType : uint8_t
{
    HandoverRequest = 1,          //!< TS 38.423 section 8.2.1.2
    HandoverRequestAcknowledge,   //!< section 8.2.1.3
    HandoverPreparationFailure,   //!< section 8.2.1.4
    SnStatusTransfer,             //!< section 8.2.2
    UeContextRelease,             //!< section 8.2.3
    HandoverCancel,               //!< section 8.2.4
};

const char* XnapMessageTypeName(XnapMessageType t);

/**
 * \ingroup ntn-constellation
 * \brief One XnAP message on the inter-satellite link.
 *
 * Carries the identifiers the procedure actually turns on: the two NG-RAN node
 * UE XnAP IDs that bind a UE context across the two gNBs, the target cell, and
 * the PDCP sequence-number state that SN Status Transfer exists to move.
 */
class NtnXnapHeader : public Header
{
  public:
    NtnXnapHeader() = default;
    static TypeId GetTypeId();
    TypeId GetInstanceTypeId() const override;
    uint32_t GetSerializedSize() const override;
    void Serialize(Buffer::Iterator start) const override;
    uint32_t Deserialize(Buffer::Iterator start) override;
    void Print(std::ostream& os) const override;

    void SetMessageType(XnapMessageType t) { m_type = t; }
    XnapMessageType GetMessageType() const { return m_type; }

    /// TS 38.423: Old NG-RAN node UE XnAP ID (allocated at the source).
    void SetSourceUeXnapId(uint32_t id) { m_srcUeId = id; }
    uint32_t GetSourceUeXnapId() const { return m_srcUeId; }
    /// New NG-RAN node UE XnAP ID (allocated at the target, returned in the
    /// acknowledge). Zero until the target has allocated it.
    void SetTargetUeXnapId(uint32_t id) { m_tgtUeId = id; }
    uint32_t GetTargetUeXnapId() const { return m_tgtUeId; }

    void SetSourceGnbId(uint32_t id) { m_srcGnb = id; }
    uint32_t GetSourceGnbId() const { return m_srcGnb; }
    void SetTargetGnbId(uint32_t id) { m_tgtGnb = id; }
    uint32_t GetTargetGnbId() const { return m_tgtGnb; }
    /// Target cell NR Cell Global Identity.
    void SetTargetNrCgi(uint64_t nrcgi) { m_targetNrCgi = nrcgi; }
    uint64_t GetTargetNrCgi() const { return m_targetNrCgi; }

    /// PDCP SN and HFN carried by SN Status Transfer, which is the whole point
    /// of that message: without it the target cannot resume the sequence and
    /// the handover is lossless in name only.
    void SetDlPdcpSn(uint32_t sn) { m_dlPdcpSn = sn; }
    uint32_t GetDlPdcpSn() const { return m_dlPdcpSn; }
    void SetUlPdcpSn(uint32_t sn) { m_ulPdcpSn = sn; }
    uint32_t GetUlPdcpSn() const { return m_ulPdcpSn; }
    void SetHfn(uint32_t hfn) { m_hfn = hfn; }
    uint32_t GetHfn() const { return m_hfn; }

    /// TS 38.423 cause value; 0 = unspecified.
    void SetCause(uint8_t cause) { m_cause = cause; }
    uint8_t GetCause() const { return m_cause; }

  private:
    XnapMessageType m_type{XnapMessageType::HandoverRequest};
    uint32_t m_srcUeId{0};
    uint32_t m_tgtUeId{0};
    uint32_t m_srcGnb{0};
    uint32_t m_tgtGnb{0};
    uint64_t m_targetNrCgi{0};
    uint32_t m_dlPdcpSn{0};
    uint32_t m_ulPdcpSn{0};
    uint32_t m_hfn{0};
    uint8_t m_cause{0};
};

} // namespace ntncon
} // namespace ns3

#endif // NTN_XNAP_MESSAGES_H
