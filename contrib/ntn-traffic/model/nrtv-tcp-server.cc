/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2013 Magister Solutions
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: Budiarto Herman <budiarto.herman@magister.fi>
 *
 */

#include "nrtv-tcp-server.h"

#include "nrtv-video-worker.h"

#include "ns3/address-utils.h"
#include "ns3/inet-socket-address.h"
#include "ns3/inet6-socket-address.h"
#include "ns3/log.h"
#include "ns3/packet.h"
#include "ns3/pointer.h"
#include "ns3/simulator.h"
#include "ns3/socket.h"
#include "ns3/tcp-socket-factory.h"
#include "ns3/uinteger.h"

#include <map>
#include <string>

NS_LOG_COMPONENT_DEFINE("NrtvTcpServer");

namespace ns3
{

NS_OBJECT_ENSURE_REGISTERED(NrtvTcpServer);

NrtvTcpServer::NrtvTcpServer()
    : m_state(NOT_STARTED),
      m_initialSocket(0)
{
    NS_LOG_FUNCTION(this);
}

TypeId
NrtvTcpServer::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::NrtvTcpServer")
            .SetParent<Application>()
            .AddConstructor<NrtvTcpServer>()
            .AddAttribute("LocalAddress",
                          "The local address of the server, "
                          "i.e., the address on which to bind the Rx socket",
                          AddressValue(),
                          MakeAddressAccessor(&NrtvTcpServer::m_localAddress),
                          MakeAddressChecker())
            .AddAttribute("LocalPort",
                          "Port on which the application listen for incoming packets",
                          UintegerValue(1935), // the default port for Adobe Flash video
                          MakeUintegerAccessor(&NrtvTcpServer::m_localPort),
                          MakeUintegerChecker<uint16_t>())
            .AddTraceSource("Tx",
                            "A packet has been sent",
                            MakeTraceSourceAccessor(&NrtvTcpServer::m_txTrace),
                            "ns3::Packet::TracedCallback")
            .AddTraceSource("StateTransition",
                            "Trace fired upon every NRTV server state transition",
                            MakeTraceSourceAccessor(&NrtvTcpServer::m_stateTransitionTrace),
                            "ns3::NrtvVariables::StateTransitionCallback");
    return tid;
}

Address
NrtvTcpServer::GetLocalAddress() const
{
    return m_localAddress;
}

uint16_t
NrtvTcpServer::GetLocalPort() const
{
    return m_localPort;
}

NrtvTcpServer::State_t
NrtvTcpServer::GetState() const
{
    return m_state;
}

std::string
NrtvTcpServer::GetStateString() const
{
    return GetStateString(m_state);
}

std::string
NrtvTcpServer::GetStateString(NrtvTcpServer::State_t state)
{
    switch (state)
    {
    case NOT_STARTED:
        return "NOT_STARTED";
        break;
    case STARTED:
        return "STARTED";
        break;
    case STOPPED:
        return "STOPPED";
        break;
    default:
        NS_FATAL_ERROR("Unknown state");
        return "";
        break;
    }
}

void
NrtvTcpServer::DoDispose()
{
    NS_LOG_FUNCTION(this);

    if (!Simulator::IsFinished()) // guard against canceling out-of-bound events
    {
        StopApplication();
    }

    Application::DoDispose(); // chain up
}

void
NrtvTcpServer::StartApplication()
{
    NS_LOG_FUNCTION(this);

    if (m_state == NOT_STARTED)
    {
        if (m_initialSocket == nullptr)
        {
            m_initialSocket = Socket::CreateSocket(GetNode(), TcpSocketFactory::GetTypeId());
            NS_LOG_INFO(this << " created socket " << m_initialSocket);
            int ret;

            if (Ipv4Address::IsMatchingType(m_localAddress))
            {
                const Ipv4Address ipv4 = Ipv4Address::ConvertFrom(m_localAddress);
                const InetSocketAddress inetSocket = InetSocketAddress(ipv4, m_localPort);
                NS_LOG_INFO(this << " binding on " << ipv4 << " port " << m_localPort << " / "
                                 << inetSocket);
                ret = m_initialSocket->Bind(inetSocket);
                NS_LOG_DEBUG(this << " Bind() return value= " << ret
                                  << " GetErrNo= " << m_initialSocket->GetErrno());
            }
            else if (Ipv6Address::IsMatchingType(m_localAddress))
            {
                const Ipv6Address ipv6 = Ipv6Address::ConvertFrom(m_localAddress);
                const Inet6SocketAddress inet6Socket = Inet6SocketAddress(ipv6, m_localPort);
                NS_LOG_INFO(this << " binding on " << ipv6 << " port " << m_localPort << " / "
                                 << inet6Socket);
                ret = m_initialSocket->Bind(inet6Socket);
                NS_LOG_DEBUG(this << " Bind() return value= " << ret
                                  << " GetErrNo= " << m_initialSocket->GetErrno());
            }

            ret = m_initialSocket->Listen();
            NS_LOG_DEBUG(this << " Listen () return value= " << ret
                              << " GetErrNo= " << m_initialSocket->GetErrno());

        } // end of `if (m_initialSocket == nullptr)`

        NS_ASSERT_MSG(m_initialSocket != nullptr, "Failed creating socket");
        m_initialSocket->ShutdownRecv();
        m_initialSocket->SetAcceptCallback(
            MakeCallback(&NrtvTcpServer::ConnectionRequestCallback, this),
            MakeCallback(&NrtvTcpServer::NewConnectionCreatedCallback, this));
        m_initialSocket->SetCloseCallbacks(MakeCallback(&NrtvTcpServer::NormalCloseCallback, this),
                                           MakeCallback(&NrtvTcpServer::ErrorCloseCallback, this));

        SwitchToState(STARTED);
        for (auto w = m_workers.begin(); w != m_workers.end(); w++)
        {
            w->second->ChangeState(NrtvVideoWorker::READY);
        }

    } // end of `if (m_state == NOT_STARTED)`
    else
    {
        NS_LOG_WARN(this << " invalid state " << GetStateString() << " for StartApplication");
    }

} // end of `void StartApplication ()`

void
NrtvTcpServer::StopApplication()
{
    NS_LOG_FUNCTION(this);

    SwitchToState(STOPPED);

    // close all accepted sockets
    for (std::map<Ptr<Socket>, Ptr<NrtvVideoWorker>>::iterator it = m_workers.begin();
         it != m_workers.end();
         ++it)
    {
        it->first->Close();
        it->first->SetSendCallback(MakeNullCallback<void, Ptr<Socket>, uint32_t>());
    }

    // destroy all workers
    m_workers.clear();

    // stop listening
    if (m_initialSocket != nullptr)
    {
        m_initialSocket->Close();
        m_initialSocket->SetSendCallback(MakeNullCallback<void, Ptr<Socket>, uint32_t>());
    }
}

bool
NrtvTcpServer::ConnectionRequestCallback(Ptr<Socket> socket, const Address& address)
{
    NS_LOG_FUNCTION(this << socket << address);
    return true; // unconditionally accept the connection request
}

void
NrtvTcpServer::NewConnectionCreatedCallback(Ptr<Socket> socket, const Address& address)
{
    NS_LOG_FUNCTION(this << socket << address);

    Ptr<NrtvVideoWorker> worker = CreateObject<NrtvVideoWorker>(socket);
    worker->SetTxCallback(MakeCallback(&NrtvTcpServer::NotifyTxSlice, this));
    worker->SetVideoCompletedCallback(MakeCallback(&NrtvTcpServer::NotifyVideoCompleted, this));
    m_workers[socket] = worker;
    if (GetState() == STARTED)
    {
        worker->ChangeState(NrtvVideoWorker::READY);
    }
}

void
NrtvTcpServer::NormalCloseCallback(Ptr<Socket> socket)
{
    NS_LOG_FUNCTION(this << socket);

    if (socket == m_initialSocket)
    {
        if (m_state == STARTED)
        {
            NS_FATAL_ERROR(
                "Initial listener socket shall not be closed when server is still running");
        }
    }
}

void
NrtvTcpServer::ErrorCloseCallback(Ptr<Socket> socket)
{
    NS_LOG_FUNCTION(this << socket);

    if (socket == m_initialSocket)
    {
        if (m_state == STARTED)
        {
            NS_FATAL_ERROR(
                "Initial listener socket shall not be closed when server is still running");
        }
    }
}

void
NrtvTcpServer::NotifyTxSlice(Ptr<Socket> socket, Ptr<const Packet> packet)
{
    NS_LOG_FUNCTION(this << socket << packet << packet->GetSize());
    NS_LOG_INFO("NrtvTcpServer sent " << packet->GetSize() << " bytes.");
    m_txTrace(packet);
}

void
NrtvTcpServer::NotifyVideoCompleted(Ptr<Socket> socket)
{
    NS_LOG_FUNCTION(this << socket);

    // remove the worker entry
    std::map<Ptr<Socket>, Ptr<NrtvVideoWorker>>::iterator it = m_workers.find(socket);
    NS_ASSERT(it != m_workers.end());
    m_workers.erase(it); // This will destroy the worker
    socket->Close();     // Close the socket, client app will request reconnection
}

void
NrtvTcpServer::SwitchToState(NrtvTcpServer::State_t state)
{
    const std::string oldState = GetStateString();
    const std::string newState = GetStateString(state);
    NS_LOG_FUNCTION(this << oldState << newState);
    m_state = state;
    NS_LOG_INFO(this << " NrtvTcpServer " << oldState << " --> " << newState);
    m_stateTransitionTrace(oldState, newState);
}

} // namespace ns3
