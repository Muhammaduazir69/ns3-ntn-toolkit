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

#include "nrtv-tcp-client.h"

#include "nrtv-header.h"
#include "nrtv-variables.h"

#include "ns3/config.h"
#include "ns3/inet-socket-address.h"
#include "ns3/inet6-socket-address.h"
#include "ns3/log.h"
#include "ns3/pointer.h"
#include "ns3/simulator.h"
#include "ns3/socket.h"
#include "ns3/tcp-socket-factory.h"
#include "ns3/udp-socket-factory.h"
#include "ns3/uinteger.h"

#include <string>

NS_LOG_COMPONENT_DEFINE("NrtvTcpClient");

namespace ns3
{

// NRTV CLIENT ////////////////////////////////////////////////////////////////

NS_OBJECT_ENSURE_REGISTERED(NrtvTcpClient);

NrtvTcpClient::NrtvTcpClient()
    : m_state(NOT_STARTED),
      m_socket(0),
      m_rxBuffer(Create<NrtvTcpClientRxBuffer>()),
      m_nrtvVariables(CreateObject<NrtvVariables>()),
      m_lastDelay(0)
{
    NS_LOG_FUNCTION(this);

    m_dejitterBufferWindowSize = m_nrtvVariables->GetDejitterBufferWindowSize();
    NS_LOG_INFO(this << " this client application uses" << " a de-jitter buffer window size of "
                     << m_dejitterBufferWindowSize.GetSeconds() << " seconds");
}

TypeId
NrtvTcpClient::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::NrtvTcpClient")
            .SetParent<Application>()
            .AddConstructor<NrtvTcpClient>()
            .AddAttribute("NrtvConfigurationVariables",
                          "Pointer to random number generator",
                          PointerValue(),
                          MakePointerAccessor(&NrtvTcpClient::m_nrtvVariables),
                          MakePointerChecker<NrtvVariables>())
            .AddAttribute("RemoteServerAddress",
                          "The address of the destination server",
                          AddressValue(),
                          MakeAddressAccessor(&NrtvTcpClient::m_remoteServerAddress),
                          MakeAddressChecker())
            .AddAttribute("RemoteServerPort",
                          "The destination port of the outbound packets",
                          UintegerValue(1935), // the default port for Adobe Flash video
                          MakeUintegerAccessor(&NrtvTcpClient::m_remoteServerPort),
                          MakeUintegerChecker<uint16_t>())
            .AddTraceSource("Rx",
                            "One packet of has been received (not necessarily a "
                            "single video slice)",
                            MakeTraceSourceAccessor(&NrtvTcpClient::m_rxTrace),
                            "ns3::Packet::PacketAddressTracedCallback")
            .AddTraceSource("RxDelay",
                            "Received a whole slice with delay information",
                            MakeTraceSourceAccessor(&NrtvTcpClient::m_rxDelayTrace),
                            "ns3::ApplicationDelayProbe::PacketDelayAddressCallback")
            .AddTraceSource("RxJitter",
                            "Received a packet with jitter information",
                            MakeTraceSourceAccessor(&NrtvTcpClient::m_rxJitterTrace),
                            "ns3::ApplicationDelayProbe::PacketDelayAddressCallback")
            .AddTraceSource("RxSlice",
                            "Received a whole slice",
                            MakeTraceSourceAccessor(&NrtvTcpClient::m_rxSliceTrace),
                            "ns3::Packet::TracedCallback")
            .AddTraceSource("RxFrame",
                            "Received a whole frame",
                            MakeTraceSourceAccessor(&NrtvTcpClient::m_rxFrameTrace),
                            "ns3::NrtvTcpClient::RxFrameCallback")
            .AddTraceSource("StateTransition",
                            "Trace fired upon every NRTV client state transition",
                            MakeTraceSourceAccessor(&NrtvTcpClient::m_stateTransitionTrace),
                            "ns3::NrtvVariables::StateTransitionCallback");
    return tid;
}

Time
NrtvTcpClient::GetStartTime() const
{
    return m_startTime;
}

Time
NrtvTcpClient::GetStopTime() const
{
    return m_stopTime;
}

bool
NrtvTcpClient::IsScheduledToStop() const
{
    return (m_stopTime != TimeStep(0));
}

Address
NrtvTcpClient::GetRemoteServerAddress() const
{
    return m_remoteServerAddress;
}

uint16_t
NrtvTcpClient::GetRemoteServerPort() const
{
    return m_remoteServerPort;
}

NrtvTcpClient::State_t
NrtvTcpClient::GetState() const
{
    return m_state;
}

std::string
NrtvTcpClient::GetStateString() const
{
    return GetStateString(m_state);
}

std::string
NrtvTcpClient::GetStateString(NrtvTcpClient::State_t state)
{
    switch (state)
    {
    case NOT_STARTED:
        return "NOT_STARTED";
        break;
    case CONNECTING:
        return "CONNECTING";
        break;
    case RECEIVING:
        return "RECEIVING";
        break;
    case IDLE:
        return "IDLE";
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
NrtvTcpClient::DoDispose()
{
    NS_LOG_FUNCTION(this);

    if (!Simulator::IsFinished()) // guard against canceling out-of-bound events
    {
        StopApplication();
    }

    Application::DoDispose(); // chain up
}

void
NrtvTcpClient::StartApplication()
{
    NS_LOG_FUNCTION(this);

    if (m_state == NOT_STARTED)
    {
        const Time connectionOpenDelay = m_nrtvVariables->GetConnectionOpenDelay();
        NS_LOG_INFO(this << " NRTV TCP client started - " << connectionOpenDelay.GetSeconds()
                         << " seconds before opening connection.");
        Simulator::Schedule(connectionOpenDelay, &NrtvTcpClient::OpenConnection, this);
    }
    else
    {
        NS_LOG_WARN(this << " invalid state " << GetStateString() << " for StartApplication");
    }
}

void
NrtvTcpClient::StopApplication()
{
    NS_LOG_FUNCTION(this);

    SwitchToState(STOPPED);
    CancelAllPendingEvents();
    CloseConnection();
}

void
NrtvTcpClient::ConnectionSucceededCallback(Ptr<Socket> socket)
{
    NS_LOG_FUNCTION(this << socket);

    if (m_state == CONNECTING)
    {
        NS_ASSERT_MSG(m_socket == socket, "Invalid socket");
        socket->SetRecvCallback(MakeCallback(&NrtvTcpClient::ReceivedDataCallback, this));
        SwitchToState(RECEIVING);
    }
    else
    {
        NS_LOG_WARN(this << " invalid state " << GetStateString() << " for ConnectionSucceeded");
    }
}

void
NrtvTcpClient::ConnectionFailedCallback(Ptr<Socket> socket)
{
    NS_LOG_FUNCTION(this << socket);

    if (m_state == CONNECTING)
    {
        if (socket->GetErrno() != Socket::ERROR_NOTERROR)
        {
            m_eventRetryConnection = Simulator::ScheduleNow(&NrtvTcpClient::RetryConnection, this);
        }
    }
    else
    {
        NS_LOG_WARN(this << " invalid state " << GetStateString() << " for ConnectionFailed");
    }
}

void
NrtvTcpClient::NormalCloseCallback(Ptr<Socket> socket)
{
    NS_LOG_FUNCTION(this << socket);

    CancelAllPendingEvents();
    SwitchToState(IDLE);
    const Time idleTime = m_nrtvVariables->GetIdleTime();
    NS_LOG_INFO(this << " a video has just completed, now waiting for " << idleTime.GetSeconds()
                     << " seconds before the next video");
    Simulator::Schedule(idleTime, &NrtvTcpClient::OpenConnection, this);
}

void
NrtvTcpClient::ErrorCloseCallback(Ptr<Socket> socket)
{
    NS_LOG_FUNCTION(this << socket);

    CancelAllPendingEvents();
    m_eventRetryConnection = Simulator::ScheduleNow(&NrtvTcpClient::RetryConnection, this);
    /// @todo This won't work because the socket is already closed
}

void
NrtvTcpClient::ReceivedDataCallback(Ptr<Socket> socket)
{
    NS_LOG_FUNCTION(this << socket);

    if (m_state == RECEIVING)
    {
        Ptr<Packet> packet;
        Address from;

        while ((packet = socket->RecvFrom(from)))
        {
            if (packet->GetSize() == 0)
            {
                break; // EOF
            }

#ifdef NS3_LOG_ENABLE
            if (InetSocketAddress::IsMatchingType(from))
            {
                NS_LOG_INFO(this << " a packet of " << packet->GetSize() << " bytes"
                                 << " received from "
                                 << InetSocketAddress::ConvertFrom(from).GetIpv4() << " port "
                                 << InetSocketAddress::ConvertFrom(from).GetPort() << " / "
                                 << InetSocketAddress::ConvertFrom(from));
            }
            else if (Inet6SocketAddress::IsMatchingType(from))
            {
                NS_LOG_INFO(this << " a packet of " << packet->GetSize() << " bytes"
                                 << " received from "
                                 << Inet6SocketAddress::ConvertFrom(from).GetIpv6() << " port "
                                 << Inet6SocketAddress::ConvertFrom(from).GetPort() << " / "
                                 << InetSocketAddress::ConvertFrom(from));
            }
#endif /* NS3_LOG_ENABLE */

            m_rxBuffer->PushPacket(packet);
            m_rxTrace(packet, from);

            while (m_rxBuffer->HasVideoSlice())
            {
                ReceiveVideoSlice(from);
            }

        } // end of `while ((packet = socket->RecvFrom (from)))`

    } // end of  `if (m_state == RECEIVING)`
    else
    {
        NS_LOG_WARN(this << " invalid state " << GetStateString() << " for ReceivedData");
    }

} // end of `void ReceivedDataCallback (Ptr<Socket> socket)`

void
NrtvTcpClient::OpenConnection()
{
    NS_LOG_FUNCTION(this);

    if (m_state == NOT_STARTED || m_state == IDLE)
    {
        m_socket = Socket::CreateSocket(GetNode(), TcpSocketFactory::GetTypeId());
        NS_LOG_INFO(this << " created socket " << m_socket);
        int ret;

        if (Ipv4Address::IsMatchingType(m_remoteServerAddress))
        {
            ret = m_socket->Bind();
            NS_LOG_DEBUG(this << " Bind() return value= " << ret
                              << " GetErrNo= " << m_socket->GetErrno());

            const Ipv4Address ipv4 = Ipv4Address::ConvertFrom(m_remoteServerAddress);
            const InetSocketAddress inetSocket = InetSocketAddress(ipv4, m_remoteServerPort);
            NS_LOG_INFO(this << " connecting to " << ipv4 << " port " << m_remoteServerPort << " / "
                             << inetSocket);
            ret = m_socket->Connect(inetSocket);
            NS_LOG_DEBUG(this << " Connect() return value= " << ret
                              << " GetErrNo= " << m_socket->GetErrno());
        }
        else if (Ipv6Address::IsMatchingType(m_remoteServerAddress))
        {
            ret = m_socket->Bind6();
            NS_LOG_DEBUG(this << " Bind6() return value= " << ret
                              << " GetErrNo= " << m_socket->GetErrno());

            const Ipv6Address ipv6 = Ipv6Address::ConvertFrom(m_remoteServerAddress);
            const Inet6SocketAddress inet6Socket = Inet6SocketAddress(ipv6, m_remoteServerPort);
            NS_LOG_INFO(this << " connecting to " << ipv6 << " port " << m_remoteServerPort << " / "
                             << inet6Socket);
            ret = m_socket->Connect(inet6Socket);
            NS_LOG_DEBUG(this << " Connect() return value= " << ret
                              << " GetErrNo= " << m_socket->GetErrno());
        }

        // NS_UNUSED (ret); // mute compiler warning
        NS_ASSERT_MSG(m_socket != nullptr, "Failed creating socket");

        m_socket->ShutdownSend();
        m_socket->SetConnectCallback(
            MakeCallback(&NrtvTcpClient::ConnectionSucceededCallback, this),
            MakeCallback(&NrtvTcpClient::ConnectionFailedCallback, this));
        m_socket->SetCloseCallbacks(MakeCallback(&NrtvTcpClient::NormalCloseCallback, this),
                                    MakeCallback(&NrtvTcpClient::ErrorCloseCallback, this));
        m_socket->SetRecvCallback(MakeCallback(&NrtvTcpClient::ReceivedDataCallback, this));

        SwitchToState(CONNECTING);

    } // end of `if (m_state == NOT_STARTED)`
    else
    {
        NS_LOG_WARN(this << " invalid state " << GetStateString() << " for OpenConnection");
    }

} // end of `void OpenConnection ()`

void
NrtvTcpClient::RetryConnection()
{
    NS_LOG_FUNCTION(this);

    if (Ipv4Address::IsMatchingType(m_remoteServerAddress))
    {
        const Ipv4Address ipv4 = Ipv4Address::ConvertFrom(m_remoteServerAddress);
        const InetSocketAddress inetSocket = InetSocketAddress(ipv4, m_remoteServerPort);
        NS_LOG_INFO(this << " retrying connecting to " << ipv4 << " port " << m_remoteServerPort
                         << " / " << inetSocket);
        int ret = m_socket->Connect(inetSocket);
        NS_LOG_DEBUG(this << " Connect() return value= " << ret
                          << " GetErrNo= " << m_socket->GetErrno());
    }
    else if (Ipv6Address::IsMatchingType(m_remoteServerAddress))
    {
        const Ipv6Address ipv6 = Ipv6Address::ConvertFrom(m_remoteServerAddress);
        const Inet6SocketAddress inet6Socket = Inet6SocketAddress(ipv6, m_remoteServerPort);
        NS_LOG_INFO(this << " retrying connection to " << ipv6 << " port " << m_remoteServerPort
                         << " / " << inet6Socket);
        int ret = m_socket->Connect(inet6Socket);
        NS_LOG_DEBUG(this << " Connect() return value= " << ret
                          << " GetErrNo= " << m_socket->GetErrno());
    }

    SwitchToState(CONNECTING);

} // end of `void RetryConnection ()`

void
NrtvTcpClient::CloseConnection()
{
    NS_LOG_FUNCTION(this);

    if (m_socket != nullptr)
    {
        m_socket->Close();
        m_socket->SetRecvCallback(MakeNullCallback<void, Ptr<Socket>>());
    }
}

uint32_t
NrtvTcpClient::ReceiveVideoSlice(const Address& from)
{
    NS_LOG_FUNCTION(this << from);

    Ptr<Packet> slice = m_rxBuffer->PopVideoSlice();

    NrtvHeader nrtvHeader;

    NS_ASSERT_MSG(slice->GetSize() >= nrtvHeader.GetSerializedSize(),
                  "The video slice contains no NRTV header");

    slice->PeekHeader(nrtvHeader);
    const uint32_t frameNumber = nrtvHeader.GetFrameNumber();
    const uint32_t numOfFrames = nrtvHeader.GetNumOfFrames();
    const uint16_t sliceNumber = nrtvHeader.GetSliceNumber();
    const uint16_t numOfSlices = nrtvHeader.GetNumOfSlices();
    const uint32_t sliceSize = nrtvHeader.GetSliceSize();
    NS_ASSERT(sliceSize + nrtvHeader.GetSerializedSize() == slice->GetSize());

    const Time delay = Simulator::Now() - nrtvHeader.GetArrivalTime();
    NS_LOG_INFO(this << " received a " << sliceSize << "-byte video slice" << " for frame "
                     << frameNumber << " and slice " << sliceNumber
                     << " (delay= " << delay.GetSeconds() << ")");

    m_rxSliceTrace(slice);
    m_rxDelayTrace(delay, from);
    if (m_lastDelay.IsZero() == false)
    {
        Time jitter = Abs(delay - m_lastDelay);
        m_rxJitterTrace(jitter, from);
    }
    m_lastDelay = delay;

    if (sliceNumber == numOfSlices)
    {
        // this is the last slice of the frame
        m_rxFrameTrace(frameNumber, numOfFrames);
    }

    return sliceSize;
}

void
NrtvTcpClient::CancelAllPendingEvents()
{
    NS_LOG_FUNCTION(this);

    if (!Simulator::IsExpired(m_eventRetryConnection))
    {
        NS_LOG_INFO(this << " canceling RetryConnection which is due in "
                         << Simulator::GetDelayLeft(m_eventRetryConnection).GetSeconds()
                         << " seconds");
        Simulator::Cancel(m_eventRetryConnection);
    }
}

void
NrtvTcpClient::SwitchToState(NrtvTcpClient::State_t state)
{
    const std::string oldState = GetStateString();
    const std::string newState = GetStateString(state);
    NS_LOG_FUNCTION(this << oldState << newState);

    m_state = state;
    NS_LOG_INFO(this << " NrtvTcpClient " << oldState << " --> " << newState);
    m_stateTransitionTrace(oldState, newState);
}

// NRTV CLIENT TX BUFFER //////////////////////////////////////////////////////

NS_LOG_COMPONENT_DEFINE("NrtvTcpClientRxBuffer");

NrtvTcpClientRxBuffer::NrtvTcpClientRxBuffer()
    : m_totalBytes(0),
      m_sizeOfVideoSlice(0)
{
    NS_LOG_FUNCTION(this);
}

bool
NrtvTcpClientRxBuffer::IsEmpty() const
{
    if (m_totalBytes == 0)
    {
        NS_ASSERT(m_rxBuffer.size() == 0);
        return true;
    }
    else
    {
        NS_ASSERT(m_rxBuffer.size() > 0);
        return false;
    }
}

bool
NrtvTcpClientRxBuffer::HasVideoSlice() const
{
    NrtvHeader header;
    return m_totalBytes >= (m_sizeOfVideoSlice + header.GetSerializedSize());
}

void
NrtvTcpClientRxBuffer::PushPacket(Ptr<const Packet> packet)
{
    const uint32_t packetSize = packet->GetSize();
    NS_LOG_FUNCTION(this << packet << packetSize);

    if (m_sizeOfVideoSlice == 0)
    {
        // we don't know the size of slice yet
        NrtvHeader header;

        if (IsEmpty())
        {
            m_rxBuffer.push_back(packet->Copy());
        }
        else
        {
            /*
             * The existing packet in the buffer must be a very small packet that
             * contains part of the header. So we combine it together with this
             * packet.
             */
            NS_ASSERT(m_rxBuffer.size() == 1);
            const uint32_t priorRemain = m_rxBuffer.back()->GetSize();
            NS_ASSERT(priorRemain < header.GetSerializedSize());
            NS_LOG_LOGIC(this << " combining a " << priorRemain << "-byte"
                              << " left over from previous slice with " << packetSize
                              << " bytes of packet");
            m_rxBuffer.back()->AddAtEnd(packet);
        }

        NS_ASSERT(m_rxBuffer.size() == 1);
        if (m_rxBuffer.back()->GetSize() >= header.GetSerializedSize())
        {
            m_sizeOfVideoSlice = PeekSliceSize(m_rxBuffer.back());
            NS_LOG_INFO(this << " now expecting a video slice of " << m_sizeOfVideoSlice
                             << " bytes");
        }
        else
        {
            /*
             * Still not enough packets to constitute a 24-byte header (sigh).
             * m_sizeOfVideoSlice stays at zero.
             */
        }
    }
    else
    {
        m_rxBuffer.push_back(packet->Copy());
    }

    // increase the buffer size counter
    m_totalBytes += packetSize;
    NS_LOG_DEBUG(this << " Rx buffer now contains " << m_rxBuffer.size() << " packet(s)" << " ("
                      << m_totalBytes << " bytes)");
}

Ptr<Packet>
NrtvTcpClientRxBuffer::PopVideoSlice()
{
    NS_LOG_FUNCTION(this);
    NS_ASSERT_MSG(!IsEmpty(), "Unable to pop from an empty Rx buffer");
    NS_ASSERT_MSG(HasVideoSlice(), "Not enough packets to constitute a complete video slice");
    NS_ASSERT(PeekSliceSize(m_rxBuffer.front()) == m_sizeOfVideoSlice);

    Ptr<Packet> slice = Create<Packet>();
    NrtvHeader header;
    const uint32_t expectedPacketSize = m_sizeOfVideoSlice + header.GetSerializedSize();
    uint32_t bytesToFetch = expectedPacketSize;

    while (bytesToFetch > 0)
    {
        NS_ASSERT(!m_rxBuffer.empty()); // ensure that front() is not undefined
        const uint32_t packetSize = m_rxBuffer.front()->GetSize();
        NS_LOG_INFO(this << " using a " << packetSize << "-byte packet"
                         << " to compose a video slice" << " (" << bytesToFetch << " bytes to go)");

        if (packetSize <= bytesToFetch)
        {
            // absorb the whole packet
            slice->AddAtEnd(m_rxBuffer.front());
            bytesToFetch -= packetSize;
            m_rxBuffer.pop_front();
        }
        else
        {
            // absorb only the first part of the packet
            slice->AddAtEnd(m_rxBuffer.front()->CreateFragment(0, bytesToFetch));

            // leave the second part in the buffer
            const uint32_t residueBytes = packetSize - bytesToFetch;
            NS_LOG_LOGIC(this << " setting aside " << residueBytes << " bytes"
                              << " for the next video slice");
            m_rxBuffer.front()->RemoveAtStart(bytesToFetch);
            NS_ASSERT(m_rxBuffer.front()->GetSize() == residueBytes);
            bytesToFetch = 0; // this exits the loop
        }

    } // end of `while (bytesToFetch > 0)`

    const uint32_t packetSize = slice->GetSize();
    NS_ASSERT(packetSize == expectedPacketSize);

    // deplete the buffer size counter
    NS_ASSERT(m_totalBytes >= packetSize);
    m_totalBytes -= packetSize;
    NS_LOG_DEBUG(this << " Rx buffer now contains " << m_rxBuffer.size() << " packet(s)" << " ("
                      << m_totalBytes << " bytes)");

    // determine the size of next slice to receive
    if (m_rxBuffer.empty())
    {
        /*
         * The buffer is empty, so we can only tell about the next slice later
         * after the next packet is received.
         */
        m_sizeOfVideoSlice = 0;
    }
    else if (m_rxBuffer.front()->GetSize() >= header.GetSerializedSize())
    {
        // the buffer is not empty and we can read an NRTV header from it
        m_sizeOfVideoSlice = PeekSliceSize(m_rxBuffer.front());
        NS_LOG_INFO(this << " now expecting a video slice of " << m_sizeOfVideoSlice << " bytes");
    }
    else
    {
        // the buffer is not empty, but we cannot read an NRTV header
        m_sizeOfVideoSlice = 0;
        NS_LOG_INFO(this << " cannot read the header yet," << " it must have been split,"
                         << " so the rest will come in the next packet");
    }

    return slice;

} // end of `Ptr<Packet> PopVideoSlice ()`

uint32_t // static
NrtvTcpClientRxBuffer::PeekSliceSize(Ptr<const Packet> packet)
{
    NS_LOG_FUNCTION(packet << packet->GetSize());
    NrtvHeader nrtvHeader;
    NS_ASSERT_MSG(packet->GetSize() >= nrtvHeader.GetSerializedSize(),
                  "The packet contains no NRTV header");
    packet->PeekHeader(nrtvHeader);
    return nrtvHeader.GetSliceSize();
}

} // namespace ns3
