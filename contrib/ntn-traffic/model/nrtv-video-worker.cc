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

#include "nrtv-video-worker.h"

#include "nrtv-header.h"
#include "nrtv-variables.h"

#include "ns3/boolean.h"
#include "ns3/log.h"
#include "ns3/packet.h"
#include "ns3/pointer.h"
#include "ns3/simulator.h"
#include "ns3/socket.h"
#include "ns3/uinteger.h"

#include <algorithm>

NS_LOG_COMPONENT_DEFINE("NrtvVideoWorker");

namespace ns3
{

NS_OBJECT_ENSURE_REGISTERED(NrtvVideoWorker);

NrtvVideoWorker::NrtvVideoWorker()
    : m_socket(),
      m_numOfFrames(0),
      m_numOfFramesServed(0),
      m_numOfSlices(0),
      m_numOfSlicesServed(0)
{
    NS_LOG_FUNCTION(this);
    NS_FATAL_ERROR("Default constructor not supported.");
}

NrtvVideoWorker::NrtvVideoWorker(Ptr<Socket> socket)
    : m_socket(socket),
      m_state(NrtvVideoWorker::NOT_READY),
      m_numOfFramesServed(0),
      m_numOfSlicesServed(0)
{
    NS_LOG_FUNCTION(this << socket);

    m_nrtvVariables = CreateObject<NrtvVariables>();
    m_frameInterval = m_nrtvVariables->GetFrameInterval(); // frame rate
    m_numOfFrames = m_nrtvVariables->GetNumOfFrames();     // length of video
    NS_ASSERT(m_numOfFrames > 0);
    m_numOfSlices = m_nrtvVariables->GetNumOfSlices(); // slices per frame
    NS_ASSERT(m_numOfSlices > 0);
    NS_LOG_INFO(this << " this video is " << m_numOfFrames << " frames long" << " (each frame is "
                     << m_frameInterval.GetMilliSeconds() << " ms long and made of "
                     << m_numOfSlices << " slices)");

    socket->SetCloseCallbacks(MakeCallback(&NrtvVideoWorker::NormalCloseCallback, this),
                              MakeCallback(&NrtvVideoWorker::ErrorCloseCallback, this));
}

void
NrtvVideoWorker::DoDispose()
{
    NS_LOG_FUNCTION(this);

    // close the socket
    m_socket->SetCloseCallbacks(MakeNullCallback<void, Ptr<Socket>>(),
                                MakeNullCallback<void, Ptr<Socket>>());
    m_socket->SetSendCallback(MakeNullCallback<void, Ptr<Socket>, uint32_t>());
    // m_socket->Close (); // Do not close the socket, leave it for the application.

    CancelAllPendingEvents();
}

TypeId
NrtvVideoWorker::GetTypeId()
{
    static TypeId tid = TypeId("ns3::NrtvVideoWorker")
                            .SetParent<Object>()
                            .AddConstructor<NrtvVideoWorker>()
                            .AddAttribute("NrtvConfigurationVariables",
                                          "Pointer to random number generator",
                                          PointerValue(),
                                          MakePointerAccessor(&NrtvVideoWorker::m_nrtvVariables),
                                          MakePointerChecker<NrtvVariables>())
                            .AddAttribute("MaxSliceSize",
                                          "Maximum size of a slice",
                                          UintegerValue(536),
                                          MakeUintegerAccessor(&NrtvVideoWorker::m_maxSliceSize),
                                          MakeUintegerChecker<uint32_t>(200, 1500));
    return tid;
}

void
NrtvVideoWorker::ChangeState(SendState_t state)
{
    if (m_state == state)
    {
        return; // If state is not changed, do nothing
    }
    if (state == NrtvVideoWorker::READY)
    {
        // It is OK to start scheduling frames
        m_eventNewFrame = Simulator::ScheduleNow(&NrtvVideoWorker::NewFrame, this);
    }
    else
    {
        CancelAllPendingEvents(); // cancel any scheduled transmission
        m_videoCompletedCallback(m_socket);
    }
}

void
NrtvVideoWorker::SetTxCallback(Callback<void, Ptr<Socket>, Ptr<const Packet>> callback)
{
    m_txCallback = callback;
}

void
NrtvVideoWorker::SetVideoCompletedCallback(Callback<void, Ptr<Socket>> callback)
{
    m_videoCompletedCallback = callback;
}

void
NrtvVideoWorker::NormalCloseCallback(Ptr<Socket> socket)
{
    NS_LOG_FUNCTION(this << socket);
    NS_ASSERT_MSG(m_socket == socket,
                  "Socket " << m_socket << " is expected, " << "but socket " << socket
                            << " is received");
    m_socket->SetSendCallback(MakeNullCallback<void, Ptr<Socket>, uint32_t>());
    CancelAllPendingEvents(); // cancel any scheduled transmission
    m_videoCompletedCallback(m_socket);
}

void
NrtvVideoWorker::ErrorCloseCallback(Ptr<Socket> socket)
{
    NS_LOG_FUNCTION(this << socket);
    NS_ASSERT_MSG(m_socket == socket,
                  "Socket " << m_socket << " is expected, " << "but socket " << socket
                            << " is received");
    m_socket->SetSendCallback(MakeNullCallback<void, Ptr<Socket>, uint32_t>());
    CancelAllPendingEvents(); // cancel any scheduled transmission
    m_videoCompletedCallback(m_socket);
}

void
NrtvVideoWorker::SendCallback(Ptr<Socket> socket, uint32_t availableBufferSize)
{
    NS_LOG_FUNCTION(this << socket << availableBufferSize);
    NS_ASSERT_MSG(m_socket == socket,
                  "Socket " << m_socket << " is expected, " << "but socket " << socket
                            << " is received");
}

void
NrtvVideoWorker::ScheduleNewFrame()
{
    uint32_t frameNumber = m_numOfFramesServed + 1;
    NS_LOG_FUNCTION(this << frameNumber << m_numOfFrames);
    NS_ASSERT(frameNumber <= m_numOfFrames);

    m_eventNewFrame = Simulator::Schedule(m_frameInterval, &NrtvVideoWorker::NewFrame, this);
    NS_LOG_INFO(this << " video frame " << frameNumber << " will be generated in "
                     << m_frameInterval.GetSeconds() << " seconds");
}

void
NrtvVideoWorker::NewFrame()
{
    m_numOfFramesServed++;
    NS_LOG_FUNCTION(this << m_numOfFramesServed << m_numOfFrames);

    if (m_numOfFramesServed < m_numOfFrames)
    {
        ScheduleNewFrame(); // schedule the next frame
    }
    else
    {
        // inform the server instance
        NS_LOG_INFO(this << " no more frame after this");
        m_eventNewFrame = Simulator::Schedule(m_frameInterval, &NrtvVideoWorker::EndVideo, this);
    }

    m_numOfSlicesServed = 0;
    ScheduleNewSlice(); // the first slice of this frame
}

void
NrtvVideoWorker::ScheduleNewSlice()
{
    uint16_t sliceNumber = m_numOfSlicesServed + 1;
    NS_LOG_FUNCTION(this << sliceNumber << m_numOfSlices);
    NS_ASSERT(sliceNumber <= m_numOfSlices);

    const Time encodingDelay = m_nrtvVariables->GetSliceEncodingDelay();
    NS_LOG_DEBUG(this << " encoding the slice needs " << encodingDelay.GetMilliSeconds() << " ms,"
                      << " while new frame is coming in "
                      << Simulator::GetDelayLeft(m_eventNewFrame).GetMilliSeconds() << " ms");

    if (encodingDelay < Simulator::GetDelayLeft(m_eventNewFrame))
    {
        // still time for a new slice
        NS_LOG_INFO(this << " video slice " << sliceNumber << " will be generated in "
                         << encodingDelay.GetMilliSeconds() << " ms");
        m_eventNewSlice = Simulator::Schedule(encodingDelay, &NrtvVideoWorker::NewSlice, this);
    }
    else
    {
        // not enough time for another slice
        NS_LOG_LOGIC(this << " " << (m_numOfSlices - m_numOfSlicesServed) << " slices are skipped");
    }
}

void
NrtvVideoWorker::NewSlice()
{
    m_numOfSlicesServed++;
    NS_LOG_FUNCTION(this << m_numOfSlicesServed << m_numOfSlices);

    const uint32_t socketSize = m_socket->GetTxAvailable();
    NS_LOG_DEBUG(this << " socket has " << socketSize << " bytes available for Tx");

    const uint32_t sliceSize = m_nrtvVariables->GetSliceSize();
    NS_LOG_INFO(this << " video slice " << m_numOfSlicesServed << " is " << sliceSize << " bytes");

    NrtvHeader nrtvHeader;

    const uint32_t headerSize = nrtvHeader.GetSerializedSize();
    const uint32_t contentSize = std::min(sliceSize, socketSize - headerSize);
    /*
     * We simply assume that our packets are rather small and the socket will
     * always has space to fit these packets.
     */
    NS_ASSERT_MSG(contentSize == sliceSize, "Socket size is too small");

    nrtvHeader.SetFrameNumber(m_numOfFramesServed);
    nrtvHeader.SetNumOfFrames(m_numOfFrames);
    nrtvHeader.SetSliceNumber(m_numOfSlicesServed);
    nrtvHeader.SetNumOfSlices(m_numOfSlices);
    nrtvHeader.SetSliceSize(sliceSize);

    Ptr<Packet> packet = Create<Packet>(contentSize);
    packet->AddHeader(nrtvHeader);

    const uint32_t packetSize = packet->GetSize();
    NS_ASSERT(packetSize == (contentSize + headerSize));
    NS_ASSERT(packetSize <= socketSize);
    // NS_ASSERT_MSG (packetSize <= m_maxSliceSize, // hard-coded MTU size 536
    //                "Packet size shall not be larger than MTU size");

    NS_LOG_INFO(this << " created packet " << packet << " of " << packetSize << " bytes");

#ifdef NS3_LOG_ENABLE
    const int actualBytes = m_socket->Send(packet);
    NS_LOG_DEBUG(this << " Send() packet " << packet << " of " << packetSize << " bytes,"
                      << " return value= " << actualBytes);

    if ((unsigned)actualBytes == packetSize)
    {
        // nothing
    }
    else
    {
        /// @todo We don't do retry at the moment, so we just do nothing in this case
        NS_LOG_ERROR(this << " failure in sending packet");
    }
#else  /* NS3_LOG_ENABLE */
    m_socket->Send(packet);
#endif /* NS3_LOG_ENABLE */

    m_txCallback(m_socket, packet);

    // make way for the next slice
    if (m_numOfSlicesServed < m_numOfSlices)
    {
        ScheduleNewSlice();
    }

} // end of `void NewSlice ()`

void
NrtvVideoWorker::EndVideo()
{
    NS_LOG_FUNCTION(this);
    m_videoCompletedCallback(m_socket);
}

void
NrtvVideoWorker::CancelAllPendingEvents()
{
    NS_LOG_FUNCTION(this);

    if (!Simulator::IsExpired(m_eventNewFrame))
    {
        NS_LOG_INFO(this << " canceling NewFrame which is due in "
                         << Simulator::GetDelayLeft(m_eventNewFrame).GetSeconds() << " seconds");
        Simulator::Cancel(m_eventNewFrame);
    }

    if (!Simulator::IsExpired(m_eventNewSlice))
    {
        NS_LOG_INFO(this << " canceling NewSlice which is due in "
                         << Simulator::GetDelayLeft(m_eventNewSlice).GetSeconds() << " seconds");
        Simulator::Cancel(m_eventNewSlice);
    }
}

} // namespace ns3
