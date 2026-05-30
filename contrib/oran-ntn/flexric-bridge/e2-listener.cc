/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#include "e2-listener.h"

#include "ns3/log.h"

namespace ns3
{
namespace oranntn
{
namespace flexric
{

NS_LOG_COMPONENT_DEFINE("OranNtnE2Listener");
NS_OBJECT_ENSURE_REGISTERED(OranNtnE2Listener);

TypeId
OranNtnE2Listener::GetTypeId()
{
    static TypeId tid = TypeId("ns3::oranntn::flexric::OranNtnE2Listener")
                            .SetParent<Object>()
                            .SetGroupName("OranNtn")
                            .AddConstructor<OranNtnE2Listener>();
    return tid;
}

OranNtnE2Listener::OranNtnE2Listener() = default;

OranNtnE2Listener::~OranNtnE2Listener()
{
    Stop();
}

void
OranNtnE2Listener::DoDispose()
{
    Stop();
    Object::DoDispose();
}

void
OranNtnE2Listener::SetTransportKind(E2Transport::Protocol p)
{
    m_kind = p;
}

bool
OranNtnE2Listener::Start(const std::string& host, uint16_t port)
{
    Stop();
    m_listener = MakeE2Transport(m_kind);
    if (!m_listener)
    {
        return false;
    }
    if (!m_listener->Listen(host, port))
    {
        m_listener.reset();
        return false;
    }
    NS_LOG_INFO("OranNtnE2Listener: listening on " << host << ":" << port
                                                     << " ("
                                                     << (m_kind == E2Transport::Protocol::tcp
                                                             ? "TCP"
                                                             : "SCTP")
                                                     << ")");
    return true;
}

void
OranNtnE2Listener::Stop()
{
    if (m_listener)
    {
        m_listener->Close();
        m_listener.reset();
    }
    for (auto& c : m_clients)
    {
        if (c->transport)
        {
            c->transport->Close();
        }
    }
    m_clients.clear();
}

size_t
OranNtnE2Listener::NumClients() const
{
    size_t n = 0;
    for (const auto& c : m_clients)
    {
        if (c->transport && c->transport->IsOpen())
        {
            ++n;
        }
    }
    return n;
}

void
OranNtnE2Listener::SendSimpleResponse(ClientSession& cs,
                                       E2MessageType replyType)
{
    E2Message reply;
    reply.type = replyType;
    reply.payload = {0x00, 0x01}; // minimal success payload
    cs.transport->Send(reply);
}

void
OranNtnE2Listener::HandleMessage(ClientSession& cs, const E2Message& msg)
{
    switch (msg.type)
    {
    case E2MessageType::e2_setup_request:
        ++m_setupReq;
        cs.e2_setup_done = true;
        SendSimpleResponse(cs, E2MessageType::e2_setup_response);
        break;
    case E2MessageType::ric_subscription_request:
        ++m_subReq;
        if (cs.e2_setup_done)
        {
            SendSimpleResponse(cs,
                                E2MessageType::ric_subscription_response);
        }
        else
        {
            SendSimpleResponse(cs,
                                E2MessageType::ric_subscription_failure);
        }
        break;
    case E2MessageType::ric_indication:
        ++m_indFwd;
        m_indicationTrace(msg.payload);
        break;
    case E2MessageType::ric_control_request:
        ++m_ctrlReq;
        if (cs.e2_setup_done)
        {
            SendSimpleResponse(cs, E2MessageType::ric_control_acknowledge);
        }
        else
        {
            SendSimpleResponse(cs, E2MessageType::ric_control_failure);
        }
        break;
    case E2MessageType::keepalive_ping:
        SendSimpleResponse(cs, E2MessageType::keepalive_pong);
        break;
    default:
        NS_LOG_DEBUG("OranNtnE2Listener: ignoring "
                      << E2MessageTypeName(msg.type));
        break;
    }
}

size_t
OranNtnE2Listener::Poll(uint32_t timeout_ms,
                          uint32_t max_msgs_per_client)
{
    size_t handled = 0;
    if (!m_listener || !m_listener->IsOpen())
    {
        return 0;
    }
    // Accept pending connections — non-blocking poll so we don't burn
    // the caller's whole `timeout_ms` budget waiting for clients that
    // are already attached.
    for (;;)
    {
        auto fresh = m_listener->AcceptOne(0);
        if (!fresh)
        {
            break;
        }
        auto cs = std::make_unique<ClientSession>();
        cs->transport = std::move(fresh);
        m_clients.push_back(std::move(cs));
        NS_LOG_INFO("OranNtnE2Listener: accepted client #"
                     << m_clients.size());
    }
    // Drain each open client. Use the full caller-provided timeout for
    // the FIRST Recv on each client; subsequent Recvs within the same
    // drain go non-blocking so we don't stall on an empty queue.
    for (auto it = m_clients.begin(); it != m_clients.end();)
    {
        ClientSession* cs = it->get();
        if (!cs->transport || !cs->transport->IsOpen())
        {
            it = m_clients.erase(it);
            continue;
        }
        for (uint32_t k = 0; k < max_msgs_per_client; ++k)
        {
            E2Message in;
            // Use the full caller-provided timeout for every iteration —
            // on a busy host even already-buffered frames can take a
            // few ms to surface via the poll-then-recv path.
            if (!cs->transport->Recv(in, timeout_ms))
            {
                break;
            }
            HandleMessage(*cs, in);
            ++handled;
        }
        ++it;
    }
    return handled;
}

} // namespace flexric
} // namespace oranntn
} // namespace ns3
