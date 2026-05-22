/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#ifndef ORAN_NTN_F1_INTERFACE_H
#define ORAN_NTN_F1_INTERFACE_H

// F1AP-lite transport between CU-CP / CU-UP and DU (Roadmap §4.1.9).
//
// Real F1AP runs over SCTP between O-CU and O-DU per O-RAN.WG3.E1AP /
// O-RAN.WG5.MP. This module provides a typed message bus with a
// configurable delivery delay (default 0.5 ms intra-rack, 3 ms when
// CU and DU are on different sites) — enough to drive the simulator
// timeline without bringing in a full SCTP/TCP stack.
//
// Two CU peers attach (CP and UP) but most flows are CU-CP↔DU; the
// CU-UP path is reserved for bearer-related signalling.

#include "oran-ntn-split-gnb.h"

#include <ns3/event-id.h>
#include <ns3/nstime.h>
#include <ns3/object.h>
#include <ns3/ptr.h>
#include <ns3/traced-callback.h>

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace ns3
{

/// F1AP-lite message kinds (subset of TS 38.473).
enum class F1apMessageKind : uint16_t
{
    f1_setup_request = 1,
    f1_setup_response = 2,
    gnb_du_configuration_update = 3,
    ue_context_setup_request = 10,
    ue_context_setup_response = 11,
    ue_context_release_command = 12,
    ue_context_release_complete = 13,
    dl_rrc_message_transfer = 20,
    ul_rrc_message_transfer = 21,
    /// Toolkit-internal: forwarded RC ControlAction from CU-CP → DU.
    rc_control_forward = 30,
};

struct F1apMessage
{
    F1apMessageKind kind;
    uint32_t ue_id{0};
    uint32_t nr_cgi{0};
    /// Opaque payload — RRC PDU for *_rrc_message_transfer, or the
    /// serialised E2RcAction for `rc_control_forward`.
    std::vector<uint8_t> payload;
    /// Caller-assigned correlation id; the responder echoes it back.
    uint32_t transaction_id{0};
};

/**
 * \ingroup oran-ntn
 * \brief F1AP-lite interface between CU and DU.
 */
class OranNtnF1Interface : public Object
{
  public:
    static TypeId GetTypeId();
    OranNtnF1Interface();
    ~OranNtnF1Interface() override;

    void Attach(Ptr<OranNtnSplitGnbEntity> cu_cp,
                Ptr<OranNtnSplitGnbEntity> du);

    /// Optional CU-UP peer. Bearer-related flows route through this.
    void AttachCuUp(Ptr<OranNtnSplitGnbEntity> cu_up);

    /// Delay applied to every Send() (default 0.5 ms).
    void SetDeliveryDelay(Time t) { m_delay = t; }
    Time GetDeliveryDelay() const { return m_delay; }

    using F1Callback = std::function<void(const F1apMessage&)>;
    /// Set the inbound callback for the CU-CP side.
    void SetCuCpReceiveCallback(F1Callback cb) { m_cuCpRecv = std::move(cb); }
    /// Set the inbound callback for the CU-UP side.
    void SetCuUpReceiveCallback(F1Callback cb) { m_cuUpRecv = std::move(cb); }
    /// Set the inbound callback for the DU side.
    void SetDuReceiveCallback(F1Callback cb) { m_duRecv = std::move(cb); }

    /// Send a message from the CU-CP toward the DU.
    bool SendFromCuCp(const F1apMessage& msg);
    /// Send a message from the CU-UP toward the DU.
    bool SendFromCuUp(const F1apMessage& msg);
    /// Send a message from the DU toward the appropriate CU peer. The
    /// CU side is selected based on the message kind.
    bool SendFromDu(const F1apMessage& msg);

    uint64_t MessagesCuToDu() const { return m_cuToDu; }
    uint64_t MessagesDuToCu() const { return m_duToCu; }
    uint64_t MessagesDropped() const { return m_dropped; }

    /// Simulate a link outage. Pending events are dropped; new sends
    /// are dropped until SetLinkUp(true) is called.
    void SetLinkUp(bool up);
    bool IsLinkUp() const { return m_up; }

    TracedCallback<F1apMessageKind> m_msgDispatched;

  protected:
    void DoDispose() override;

  private:
    bool DispatchToDu(F1apMessage msg);
    bool DispatchToCuCp(F1apMessage msg);
    bool DispatchToCuUp(F1apMessage msg);

    Ptr<OranNtnSplitGnbEntity> m_cuCp;
    Ptr<OranNtnSplitGnbEntity> m_cuUp;
    Ptr<OranNtnSplitGnbEntity> m_du;

    Time m_delay;
    bool m_up{true};

    F1Callback m_cuCpRecv;
    F1Callback m_cuUpRecv;
    F1Callback m_duRecv;

    uint64_t m_cuToDu{0};
    uint64_t m_duToCu{0};
    uint64_t m_dropped{0};

    std::vector<EventId> m_pending;
};

} // namespace ns3

#endif // ORAN_NTN_F1_INTERFACE_H
