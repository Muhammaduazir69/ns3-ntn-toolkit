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

#ifndef NRTV_VIDEO_WORKER_H
#define NRTV_VIDEO_WORKER_H

#include "ns3/callback.h"
#include "ns3/event-id.h"
#include "ns3/nstime.h"
#include "ns3/object.h"
#include "ns3/ptr.h"

namespace ns3
{

class Socket;
class Packet;
class NrtvVariables;

/**
 * @internal
 * @ingroup nrtv
 * @brief Represent a single video session and its transmission.
 */
class NrtvVideoWorker : public Object
{
  public:
    /**
     * @brief Creates a new instance of worker and starts the transmission.
     *
     * @param socket pointer to the socket (must be already connected to a
     *               destination client) that will be utilized by the worker to
     *               send video packets
     *
     * The worker will determine the length of video using NrtvVariables class.
     * Other variables are also retrieved from this class, such as number of
     * frames per second (frame rate) and number of slices per frame.
     *
     * The first video frame starts once the server has given a permission to do so.
     * Each frame has a fixed number of slices, and each slice is preceded by a
     * random length of encoding delay. Each slice constitutes a single packet,
     * which size is also determined randomly. Each packet begins with a 24-byte
     * NrtvHeader.
     *
     * Each frame always abides to the given frame rate, i.e., the start of each
     * frame is always punctual according to the frame rate. If the transmission
     * of the slices takes longer than the length of a single frame, then the
     * remaining unsent slices would be discarded, without postponing the start
     * time of the next frame.
     *
     * Each slice sent will invoke the callback function specified using
     * SetTxCallback(). After all the frames have been transmitted, another
     * callback function, specified using SetVideoCompletedCallback(), will be
     * invoked.
     */
    NrtvVideoWorker();
    NrtvVideoWorker(Ptr<Socket> socket);

    enum SendState_t
    {
        NOT_READY = 0,
        READY
    };

    void ChangeState(SendState_t state);

    // inherited from ObjectBase base class
    static TypeId GetTypeId();

    /**
     * @param callback this function is invoked after transmitting a video slice
     *                 through the socket
     */
    void SetTxCallback(Callback<void, Ptr<Socket>, Ptr<const Packet>> callback);

    /**
     * @param callback this function is invoked after a whole video has been
     *                 transmitted
     *
     * After a video is completed, the worker will stay idle indefinitely.
     */
    void SetVideoCompletedCallback(Callback<void, Ptr<Socket>> callback);

  protected:
    /// Instance destructor, will close the socket.
    void DoDispose();

  private:
    // SOCKET CALLBACK METHODS

    /// Invoked if the client disconnects.
    void NormalCloseCallback(Ptr<Socket> socket);
    /// Invoked if the client disconnects abruptly.
    void ErrorCloseCallback(Ptr<Socket> socket);
    /// Invoked if the socket has space for transmission.
    void SendCallback(Ptr<Socket> socket, uint32_t availableBufferSize);

    void ScheduleNewFrame();
    void NewFrame();
    void ScheduleNewSlice();
    void NewSlice();
    void EndVideo();
    void CancelAllPendingEvents();

    // EVENTS

    EventId m_eventNewFrame;
    EventId m_eventNewSlice;

    Ptr<Socket> m_socket;               ///< Pointer to the socket for transmission.
    Ptr<NrtvVariables> m_nrtvVariables; ///< Pointer to a NRTV variable collection.
    uint32_t m_maxSliceSize;            ///< The maximum slice size in bytes.
    Callback<void, Ptr<Socket>, Ptr<const Packet>> m_txCallback;
    Callback<void, Ptr<Socket>> m_videoCompletedCallback;
    SendState_t m_state; ///< State for checking if the video worker can start sending packets

    /// Length of time between consecutive frames.
    Time m_frameInterval;
    /// Number of frames, i.e., indicating the length of the video.
    uint32_t m_numOfFrames;
    /// The number of frames that has been sent.
    uint32_t m_numOfFramesServed;
    /// Number of slices in one frame.
    uint16_t m_numOfSlices;
    /// The number of slices that has been sent, resets to 0 after completing a frame.
    uint16_t m_numOfSlicesServed;

}; // end of `class NrtvVideoWorker`

} // namespace ns3

#endif /* NRTV_VIDEO_WORKER_H */
