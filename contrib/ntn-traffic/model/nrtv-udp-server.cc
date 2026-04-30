/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2015 Magister Solutions
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

#include "nrtv-udp-server.h"

#include "nrtv-variables.h"
#include "nrtv-video-worker.h"

#include "ns3/address-utils.h"
#include "ns3/boolean.h"
#include "ns3/inet-socket-address.h"
#include "ns3/inet6-socket-address.h"
#include "ns3/ipv4-address.h"
#include "ns3/ipv6-address.h"
#include "ns3/log.h"
#include "ns3/nstime.h"
#include "ns3/packet.h"
#include "ns3/pointer.h"
#include "ns3/simulator.h"
#include "ns3/socket.h"
#include "ns3/tcp-socket-factory.h"
#include "ns3/udp-socket-factory.h"
#include "ns3/uinteger.h"

#include <map>
#include <string>

NS_LOG_COMPONENT_DEFINE("NrtvUdpServer");

namespace ns3
{

NS_OBJECT_ENSURE_REGISTERED(NrtvUdpServer);

NrtvUdpServer::NrtvUdpServer()
    : m_state(NOT_STARTED),
      m_remotePort(0)
{
    NS_LOG_FUNCTION(this);
    m_nrtvVariables = CreateObject<NrtvVariables>();
}

TypeId
NrtvUdpServer::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::NrtvUdpServer")
            .SetParent<Application>()
            .AddConstructor<NrtvUdpServer>()
            .AddAttribute("Variables",
                          "Pointer to random number generator",
                          PointerValue(),
                          MakePointerAccessor(&NrtvUdpServer::m_nrtvVariables),
                          MakePointerChecker<NrtvVariables>())
            .AddAttribute("RemotePort",
                          "The destination port of the outbound packets",
                          UintegerValue(1935), // the default port for Adobe Flash video
                          MakeUintegerAccessor(&NrtvUdpServer::m_remotePort),
                          MakeUintegerChecker<uint16_t>())
            .AddTraceSource("Tx",
                            "A packet has been sent",
                            MakeTraceSourceAccessor(&NrtvUdpServer::m_txTrace),
                            "ns3::Packet::TracedCallback")
            .AddTraceSource("StateTransition",
                            "Trace fired upon every NRTV UDP server state transition",
                            MakeTraceSourceAccessor(&NrtvUdpServer::m_stateTransitionTrace),
                            "ns3::NrtvVariables::StateTransitionCallback");
    return tid;
}

uint16_t
NrtvUdpServer::GetRemotePort() const
{
    return m_remotePort;
}

NrtvUdpServer::State_t
NrtvUdpServer::GetState() const
{
    return m_state;
}

std::string
NrtvUdpServer::GetStateString() const
{
    return GetStateString(m_state);
}

std::string
NrtvUdpServer::GetStateString(NrtvUdpServer::State_t state)
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
NrtvUdpServer::DoDispose()
{
    NS_LOG_FUNCTION(this);

    if (!Simulator::IsFinished()) // guard against canceling out-of-bound events
    {
        StopApplication();
    }

    Application::DoDispose(); // chain up
}

void
NrtvUdpServer::StartApplication()
{
    NS_LOG_FUNCTION(this);

    if (m_state == NOT_STARTED)
    {
        SwitchToState(STARTED);
        NS_LOG_INFO(this << " NRTV UDP server was started - " << " Starting workers...");

        for (auto w = m_workers.begin(); w != m_workers.end(); w++)
        {
            Simulator::Schedule(m_nrtvVariables->GetConnectionOpenDelay(),
                                &NrtvVideoWorker::ChangeState,
                                w->second,
                                NrtvVideoWorker::READY);
        }
    }
    else
    {
        NS_LOG_WARN(this << " invalid state " << GetStateString() << " for StartApplication");
    }

} // end of `void StartApplication ()`

void
NrtvUdpServer::StopApplication()
{
    NS_LOG_FUNCTION(this);

    if (m_state == STOPPED)
    {
        NS_LOG_ERROR("Cannot stop stopped NrtvUdpServer application.");
        return;
    }

    SwitchToState(STOPPED);

    // close all accepted sockets
    for (std::map<Ptr<Socket>, Ptr<NrtvVideoWorker>>::iterator it = m_workers.begin();
         it != m_workers.end();
         ++it)
    {
        it->first->Close();
        it->first->SetSendCallback(MakeNullCallback<void, Ptr<Socket>, uint32_t>());
    }
    // Clear video workers
    m_workers.clear();
}

void
NrtvUdpServer::NotifyTxSlice(Ptr<Socket> socket, Ptr<const Packet> packet)
{
    NS_LOG_FUNCTION(this << socket << packet << packet->GetSize());
    NS_LOG_INFO("NrtvUdpServer sent " << packet->GetSize() << " bytes.");
    m_txTrace(packet);
}

void
NrtvUdpServer::NotifyVideoCompleted(Ptr<Socket> socket)
{
    NS_LOG_FUNCTION(this << socket);

    auto it = m_workers.find(socket);
    NS_ASSERT(it != m_workers.end());
    m_workers.erase(it); // this will destroy the worker
    videosLeft[socket]--;

    if (videosLeft[socket] == 0)
    {
        socket->Close();
        NS_LOG_LOGIC(this << " a video has just completed. "
                             " The client is now disconnected.");

        return;
    }

    // Socket was not closed, so we will re-use it for the next video.
    // This has to be done since there is no connection between UDP
    // client and server, and client (PacketSink) does not have
    // capability to request a new video.

    // Wait until the next video.
    const Time idleTime = m_nrtvVariables->GetIdleTime();
    NS_LOG_LOGIC(this << " a video has just completed, now waiting for " << idleTime.GetSeconds()
                      << " seconds before the next video");

    Simulator::Schedule(idleTime, &NrtvUdpServer::AddVideoWorker, this, socket);
}

void
NrtvUdpServer::AddClient(Address remoteAddress, uint32_t numberOfVideos)
{
    NS_LOG_FUNCTION(this << remoteAddress);

    Ptr<Socket> socket = Socket::CreateSocket(GetNode(), UdpSocketFactory::GetTypeId());
    int ret;

    if (Ipv4Address::IsMatchingType(remoteAddress))
    {
        ret = socket->Bind();
        NS_LOG_DEBUG(this << " Bind() return value= " << ret
                          << " GetErrNo= " << socket->GetErrno());
        Ipv4Address addr = Ipv4Address::ConvertFrom(remoteAddress);
        socket->Connect(InetSocketAddress(addr, m_remotePort));
    }
    else if (Ipv6Address::IsMatchingType(remoteAddress))
    {
        ret = socket->Bind6();
        NS_LOG_DEBUG(this << " Bind6() return value= " << ret
                          << " GetErrNo= " << socket->GetErrno());
        Ipv6Address addr = Ipv6Address::ConvertFrom(remoteAddress);
        socket->Connect(Inet6SocketAddress(addr, m_remotePort));
    }

    // NS_UNUSED (ret);

    // Create an entry of how many videos are to be streamed to this socket
    // before disconnecting.
    videosLeft[socket] = numberOfVideos;

    // Assign video worker for the socket
    AddVideoWorker(socket);

    NS_LOG_INFO("NrtvUdpServer will stream " << numberOfVideos << " videos to " << remoteAddress);
} // end of `void AddClient ()`

void
NrtvUdpServer::AddVideoWorker(Ptr<Socket> socket)
{
    Ptr<NrtvVideoWorker> worker = CreateObject<NrtvVideoWorker>(socket);
    m_workers[socket] = worker;
    worker->SetTxCallback(MakeCallback(&NrtvUdpServer::NotifyTxSlice, this));
    worker->SetVideoCompletedCallback(MakeCallback(&NrtvUdpServer::NotifyVideoCompleted, this));
    if (GetState() == STARTED)
    {
        worker->ChangeState(NrtvVideoWorker::READY);
    }
}

void
NrtvUdpServer::SwitchToState(NrtvUdpServer::State_t state)
{
    const std::string oldState = GetStateString();
    const std::string newState = GetStateString(state);
    NS_LOG_FUNCTION(this << oldState << newState);
    m_state = state;
    NS_LOG_INFO(this << " NrtvUdpServer " << oldState << " --> " << newState);
    m_stateTransitionTrace(oldState, newState);
}

} // namespace ns3
