/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#include "ntn-xn-handover.h"

#include "ns3/log.h"
#include "ns3/simulator.h"

namespace ns3
{
namespace ntncon
{

NS_LOG_COMPONENT_DEFINE("NtnXnHandover");
NS_OBJECT_ENSURE_REGISTERED(NtnXnHandover);

TypeId
NtnXnHandover::GetTypeId()
{
    static TypeId tid = TypeId("ns3::ntncon::NtnXnHandover")
                            .SetParent<Object>()
                            .SetGroupName("NtnConstellation")
                            .AddConstructor<NtnXnHandover>();
    return tid;
}

Time
NtnXnHandover::GetPeerDelay(uint32_t peerGnb) const
{
    auto it = m_peerDelay.find(peerGnb);
    return it == m_peerDelay.end() ? Time() : it->second;
}

void
NtnXnHandover::SendTo(uint32_t peer, const NtnXnapHeader& h)
{
    if (m_send.IsNull())
    {
        return;
    }
    m_send(peer, h, GetPeerDelay(peer));
}

bool
NtnXnHandover::StartHandover(uint32_t ueXnapId, uint32_t targetGnb, uint64_t targetNrCgi,
                             uint32_t dlPdcpSn, uint32_t ulPdcpSn, uint32_t hfn)
{
    if (m_send.IsNull())
    {
        // Refuse rather than pretend. A caller must not believe it started a
        // procedure whose messages go nowhere - that is the silent-success
        // pattern this campaign keeps finding.
        NS_LOG_WARN("StartHandover with no send callback wired; not started");
        return false;
    }
    if (!m_regenerative)
    {
        // SAGIN-3: a transparent payload has no on-board gNB, so there is
        // nothing here to originate an Xn procedure. Saying so is the point of
        // the RegenMode distinction.
        NS_LOG_WARN("StartHandover on a bent-pipe payload: no on-board gNB to originate Xn");
        return false;
    }
    Pending p;
    p.targetGnb = targetGnb;
    p.started = Simulator::Now();
    p.dlSn = dlPdcpSn;
    p.ulSn = ulPdcpSn;
    p.hfn = hfn;
    m_pending[ueXnapId] = p;

    NtnXnapHeader h;
    h.SetMessageType(XnapMessageType::HandoverRequest);
    h.SetSourceUeXnapId(ueXnapId);
    h.SetSourceGnbId(m_gnbId);
    h.SetTargetGnbId(targetGnb);
    h.SetTargetNrCgi(targetNrCgi);
    ++m_reqSent;
    SendTo(targetGnb, h);
    return true;
}

void
NtnXnHandover::Receive(NtnXnapHeader msg)
{
    switch (msg.GetMessageType())
    {
    case XnapMessageType::HandoverRequest: {
        // TARGET role. Admit or refuse - TS 38.423 has no "accept and ignore"
        // outcome, so a target that cannot admit answers PREPARATION FAILURE.
        ++m_reqRecv;
        NtnXnapHeader r;
        r.SetSourceUeXnapId(msg.GetSourceUeXnapId());
        r.SetSourceGnbId(msg.GetSourceGnbId());
        r.SetTargetGnbId(m_gnbId);
        r.SetTargetNrCgi(msg.GetTargetNrCgi());
        if (m_admit && m_regenerative)
        {
            r.SetMessageType(XnapMessageType::HandoverRequestAcknowledge);
            // The target allocates its own UE XnAP ID; the pair binds the
            // context across the two gNBs for the rest of the procedure.
            r.SetTargetUeXnapId(m_nextTargetUeId++);
            ++m_ackSent;
        }
        else
        {
            r.SetMessageType(XnapMessageType::HandoverPreparationFailure);
            // TS 38.423 cause: 1 = radio-network unspecified, 2 = transport
            // (used here for "no on-board gNB", which is a payload property
            // rather than a radio condition).
            r.SetCause(m_regenerative ? 1 : 2);
            ++m_failSent;
        }
        SendTo(msg.GetSourceGnbId(), r);
        break;
    }

    case XnapMessageType::HandoverRequestAcknowledge: {
        // SOURCE role: the target admitted, so move the PDCP state.
        auto it = m_pending.find(msg.GetSourceUeXnapId());
        if (it == m_pending.end())
        {
            NS_LOG_WARN("ACK for an unknown UE XnAP ID " << msg.GetSourceUeXnapId());
            return;
        }
        NtnXnapHeader sn;
        sn.SetMessageType(XnapMessageType::SnStatusTransfer);
        sn.SetSourceUeXnapId(msg.GetSourceUeXnapId());
        sn.SetTargetUeXnapId(msg.GetTargetUeXnapId());
        sn.SetSourceGnbId(m_gnbId);
        sn.SetTargetGnbId(msg.GetTargetGnbId());
        sn.SetDlPdcpSn(it->second.dlSn);
        sn.SetUlPdcpSn(it->second.ulSn);
        sn.SetHfn(it->second.hfn);
        ++m_snXfer;
        SendTo(msg.GetTargetGnbId(), sn);
        break;
    }

    case XnapMessageType::SnStatusTransfer: {
        // TARGET role: record the sequence state, then release the source's
        // context. Without the SN and HFN the target cannot resume the
        // sequence, and the handover is lossless in name only.
        m_lastDlSn = msg.GetDlPdcpSn();
        m_lastHfn = msg.GetHfn();
        NtnXnapHeader rel;
        rel.SetMessageType(XnapMessageType::UeContextRelease);
        rel.SetSourceUeXnapId(msg.GetSourceUeXnapId());
        rel.SetTargetUeXnapId(msg.GetTargetUeXnapId());
        rel.SetSourceGnbId(msg.GetSourceGnbId());
        rel.SetTargetGnbId(m_gnbId);
        ++m_ctxRel;
        SendTo(msg.GetSourceGnbId(), rel);
        break;
    }

    case XnapMessageType::UeContextRelease: {
        // SOURCE role: the procedure is complete.
        auto it = m_pending.find(msg.GetSourceUeXnapId());
        if (it == m_pending.end())
        {
            return;
        }
        const Time elapsed = Simulator::Now() - it->second.started;
        m_pending.erase(it);
        ++m_completed;
        if (!m_complete.IsNull())
        {
            m_complete(msg.GetSourceUeXnapId(), elapsed, true);
        }
        break;
    }

    case XnapMessageType::HandoverPreparationFailure: {
        auto it = m_pending.find(msg.GetSourceUeXnapId());
        if (it == m_pending.end())
        {
            return;
        }
        const Time elapsed = Simulator::Now() - it->second.started;
        m_pending.erase(it);
        if (!m_complete.IsNull())
        {
            m_complete(msg.GetSourceUeXnapId(), elapsed, false);
        }
        break;
    }

    case XnapMessageType::HandoverCancel:
        m_pending.erase(msg.GetSourceUeXnapId());
        break;
    }
}

} // namespace ntncon
} // namespace ns3
