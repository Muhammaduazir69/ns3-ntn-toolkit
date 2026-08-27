/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only
//
// ntn-xn-handover - the TS 38.300 section 9.2.3 Xn handover procedure between
// two regenerative satellites, with the ISL delay taken from real geometry.

#ifndef NTN_XN_HANDOVER_H
#define NTN_XN_HANDOVER_H

#include "ntn-xnap-messages.h"

#include "ns3/callback.h"
#include "ns3/nstime.h"
#include "ns3/object.h"

#include <map>

namespace ns3
{
namespace ntncon
{

/**
 * \ingroup ntn-constellation
 * \brief One satellite's Xn endpoint: source and target roles of a handover.
 *
 * Models the message SEQUENCE of TS 38.300 section 9.2.3, which is what a
 * regenerative-payload handover study turns on and what this toolkit had none
 * of:
 *
 *   source -> target : HANDOVER REQUEST         (TS 38.423 section 8.2.1.2)
 *   target -> source : HANDOVER REQUEST ACK     (section 8.2.1.3)
 *   source -> target : SN STATUS TRANSFER       (section 8.2.2)
 *   target -> source : UE CONTEXT RELEASE       (section 8.2.3)
 *
 * Every leg crosses the inter-satellite link, so the procedure takes real time
 * derived from real orbital separation rather than completing instantly.
 */
class NtnXnHandover : public Object
{
  public:
    static TypeId GetTypeId();
    NtnXnHandover() = default;

    /// This node's gNB identity.
    void SetGnbId(uint32_t id) { m_gnbId = id; }
    uint32_t GetGnbId() const { return m_gnbId; }

    /// Deliver a message to the peer after one ISL one-way delay. A scenario
    /// wires this to the peer's Receive() through whatever transport it has;
    /// the delay is the caller's, because only the caller knows the geometry.
    using SendCallback = Callback<void, uint32_t /*peerGnb*/, NtnXnapHeader, Time /*delay*/>;
    void SetSendCallback(SendCallback cb) { m_send = cb; }

    /// One-way delay to a peer. Set from the live inter-satellite range.
    void SetPeerDelay(uint32_t peerGnb, Time oneWay) { m_peerDelay[peerGnb] = oneWay; }
    Time GetPeerDelay(uint32_t peerGnb) const;

    /// SAGIN-3 step 3: does this node's payload actually terminate Xn?
    ///
    /// RegenMode existed as a Dijkstra transit filter and nothing else, so a
    /// bent-pipe satellite was indistinguishable from a regenerative one
    /// outside routing. It is a real distinction: a transparent payload has no
    /// on-board gNB, so there is nothing there to originate or terminate an Xn
    /// procedure. Wire this from ContactGraphRouter::IsRegenerative() and the
    /// enum stops being decorative.
    ///
    /// A non-regenerative node refuses to START a handover and answers
    /// HANDOVER PREPARATION FAILURE to any it receives - it does not go silent,
    /// because silence would leave a peer believing a procedure is in flight.
    void SetRegenerative(bool regen) { m_regenerative = regen; }
    bool IsRegenerative() const { return m_regenerative; }

    /// Whether this node admits an incoming HANDOVER REQUEST. A target that
    /// cannot admit must answer HANDOVER PREPARATION FAILURE, not silence.
    void SetAdmitIncoming(bool admit) { m_admit = admit; }

    /// Fired when a handover completes at the SOURCE (UE CONTEXT RELEASE
    /// received) with the total procedure time.
    using CompleteCallback = Callback<void, uint32_t /*ueXnapId*/, Time /*elapsed*/, bool /*ok*/>;
    void SetCompleteCallback(CompleteCallback cb) { m_complete = cb; }

    /// SOURCE role: begin an Xn handover of `ueXnapId` to `targetGnb`.
    /// \return false if no send callback is wired, so a caller cannot believe
    ///         it started a procedure that went nowhere.
    bool StartHandover(uint32_t ueXnapId, uint32_t targetGnb, uint64_t targetNrCgi,
                       uint32_t dlPdcpSn, uint32_t ulPdcpSn, uint32_t hfn);

    /// Deliver an arriving XnAP message to this endpoint.
    void Receive(NtnXnapHeader msg);

    // ---- Counters, so a scenario can show the procedure ran ----
    uint32_t GetRequestsSent() const { return m_reqSent; }
    uint32_t GetRequestsReceived() const { return m_reqRecv; }
    uint32_t GetAcksSent() const { return m_ackSent; }
    uint32_t GetFailuresSent() const { return m_failSent; }
    uint32_t GetSnStatusTransfers() const { return m_snXfer; }
    uint32_t GetContextReleases() const { return m_ctxRel; }
    uint32_t GetCompleted() const { return m_completed; }
    /// PDCP DL SN the target most recently received in SN STATUS TRANSFER.
    uint32_t GetLastReceivedDlPdcpSn() const { return m_lastDlSn; }
    uint32_t GetLastReceivedHfn() const { return m_lastHfn; }

  private:
    struct Pending
    {
        uint32_t targetGnb{0};
        Time started{};
        uint32_t dlSn{0};
        uint32_t ulSn{0};
        uint32_t hfn{0};
    };

    void SendTo(uint32_t peer, const NtnXnapHeader& h);

    uint32_t m_gnbId{0};
    bool m_admit{true};
    bool m_regenerative{true};
    SendCallback m_send;
    CompleteCallback m_complete;
    std::map<uint32_t, Time> m_peerDelay;
    std::map<uint32_t, Pending> m_pending; //!< keyed by source UE XnAP ID
    uint32_t m_nextTargetUeId{1000};

    uint32_t m_reqSent{0};
    uint32_t m_reqRecv{0};
    uint32_t m_ackSent{0};
    uint32_t m_failSent{0};
    uint32_t m_snXfer{0};
    uint32_t m_ctxRel{0};
    uint32_t m_completed{0};
    uint32_t m_lastDlSn{0};
    uint32_t m_lastHfn{0};
};

} // namespace ntncon
} // namespace ns3

#endif // NTN_XN_HANDOVER_H
