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
 * Original author: Budiarto Herman <budiarto.herman@magister.fi>
 * Modified by: Lauri Sormunen <lauri.sormunen@magister.fi>
 *
 */

#ifndef NRTV_TCP_CLIENT_H
#define NRTV_TCP_CLIENT_H

#include "ns3/address.h"
#include "ns3/application.h"
#include "ns3/nstime.h"
#include "ns3/packet.h"
#include "ns3/traced-callback.h"

#include <list>
#include <string>

namespace ns3
{

class Socket;
class NrtvVariables;
class NrtvTcpClientRxBuffer;

/**
 * @ingroup nrtv
 * @brief Model application which simulates the traffic of a client of a Near
 *        Real-Time Video (NRTV) service, i.e., a client accessing a video
 *        streaming service, over TCP.
 *
 * Upon start, the application sends a connection request to the destination
 * server. Once connected, the application waits for incoming video packets.
 *
 * When the server terminates the connection, the application regards it as the
 * end of a video session. At this point, the application enters the IDLE state,
 * which is a randomly determined delay that simulates the user "resting"
 * between videos (e.g., commenting or picking the next video). After the IDLE
 * timer expires, the application restarts again by sending another connection
 * request.
 */
class NrtvTcpClient : public Application
{
  public:
    /**
     * @brief Creates a new instance of NRTV TCP client application.
     *
     * After creation, the application must be further configured through
     * attributes. To avoid having to do this process manually, please use one of
     * the helper classes (either NrtvHelper or NrtvClientHelper).
     *
     * @warning At the moment, only TCP protocol and IPv4 is supported.
     */
    NrtvTcpClient();

    // inherited from ObjectBase base class
    static TypeId GetTypeId();

    /**
     * @return the time the application is scheduled to start
     */
    Time GetStartTime() const;

    /**
     * @return the time the application is scheduled to stop, or 0 if the stop has
     *         never been scheduled
     */
    Time GetStopTime() const;

    /**
     * @return true if the application has been scheduled to stop during the
     *         simulation
     */
    bool IsScheduledToStop() const;

    /**
     * @return the address of the destination server
     */
    Address GetRemoteServerAddress() const;

    /**
     * @return the destination port
     */
    uint16_t GetRemoteServerPort() const;

    /// The possible states of the application.
    enum State_t
    {
        /// Before the StartApplication() is executed.
        NOT_STARTED = 0,
        /// Sent the server a connection request and waiting for the server to be accept it.
        CONNECTING,
        /// Receiving incoming video packets.
        RECEIVING,
        /// Finished received a video and transitioning to the next video.
        IDLE,
        /// After StopApplication() is invoked.
        STOPPED
    };

    /**
     * @return the current state of the application
     */
    State_t GetState() const;

    /**
     * @return the current state of the application in string format
     */
    std::string GetStateString() const;

    /**
     * @param state an arbitrary state of an application
     * @return the state equivalently expressed in string format
     */
    static std::string GetStateString(State_t state);

  protected:
    // Inherited from Object base class
    virtual void DoDispose();

    // Inherited from Application base class
    virtual void StartApplication();
    virtual void StopApplication();

  private:
    // SOCKET CALLBACK METHODS

    /**
     * Callback for connection succeeding.
     * @param socket The socket bound to remote server address
     */
    void ConnectionSucceededCallback(Ptr<Socket> socket);

    /**
     * Callback for connection failing.
     * @param socket The socket bound to remote server address
     */
    void ConnectionFailedCallback(Ptr<Socket> socket);

    /**
     * Callback for connection closing normally.
     * @param socket The socket bound to remote server address
     */
    void NormalCloseCallback(Ptr<Socket> socket);

    /**
     * Callback for connection closing due to error.
     * @param socket The socket bound to remote server address
     */
    void ErrorCloseCallback(Ptr<Socket> socket);

    /**
     * Callback for receiving data from the socket.
     * @param socket The socket bound to remote server address
     */
    void ReceivedDataCallback(Ptr<Socket> socket);

    /**
     * Open connection to the server.
     */
    void OpenConnection();

    /**
     * Retry connecting to the server.
     */
    void RetryConnection();

    /**
     * Close connection to the server.
     */
    void CloseConnection();

    /**
     * Receive a video slice from the video buffer.
     * @param from Address of the sender.
     * @return Size of the slice in bytes.
     */
    uint32_t ReceiveVideoSlice(const Address& from);

    /**
     * Cancel reconnection event.
     */
    void CancelAllPendingEvents();

    /**
     * Switch state of the client.
     * @param state New state.
     */
    void SwitchToState(State_t state);

    State_t m_state;                 ///< State of the NRTV TCP client
    Time m_dejitterBufferWindowSize; ///< Dejitter buffer windows size
    Ptr<Socket> m_socket;            ///< Socket bound to the remote server

    /**
     * An Rx buffer for all received packets, which constructs video slices from received packets
     * and hands them over to the application.
     */
    Ptr<NrtvTcpClientRxBuffer> m_rxBuffer;

    // ATTRIBUTES

    /**
     * The random variable collection instance which is used as a source for
     * client behavior, e.g. idle time between videos.
     */
    Ptr<NrtvVariables> m_nrtvVariables;

    Address m_remoteServerAddress; ///!< Remote server address
    uint16_t m_remoteServerPort;   ///!< Remote server port

    Time m_lastDelay; /// Last delay measurement. Used to compute jitter.

    // TRACE SOURCES

    /**
     * @brief Trace source for packet being received.
     *
     * Example signature of callback function (with context):
     *
     *     void RxCallback (std::string context, Ptr<const Packet> packet,
     *                      const Address & from);
     */
    TracedCallback<Ptr<const Packet>, const Address&> m_rxTrace;

    /**
     * @brief Trace source for packet delay upon receiving of a packet.
     *
     * Example signature of callback function (with context):
     *
     *     void RxDelayCallback (std::string context, Time delay,
     *                           const Address & from);
     */
    TracedCallback<const Time&, const Address&> m_rxDelayTrace;

    /**
     * @brief Trace source for packet jitter upon receiving of a packet.
     *
     * Example signature of callback function (with context):
     *
     *     void RxJitterCallback (std::string context, Time jitter,
     *                           const Address & from);
     */
    TracedCallback<const Time&, const Address&> m_rxJitterTrace;

    /**
     * @brief Trace source for an entire slice being constructed from the buffer.
     *
     * Example signature of callback function (with context):
     *
     *     void RxSliceCallback (std::string context, Ptr<const Packet> slice);
     */
    TracedCallback<Ptr<const Packet>> m_rxSliceTrace;

    /**
     * @brief Trace source for all slices of a frame having been received.
     *
     * Example signature of callback function (with context):
     *
     *     void RxFrameCallback (std::string context,
     *                           uint32_t frameNumber, uint32_t numOfFrames);
     */
    TracedCallback<uint32_t, uint32_t> m_rxFrameTrace;

    /**
     * @brief Trace source for application state changing.
     *
     * Example signature of callback function (with context):
     *
     *     void RxFrameCallback (std::string context,
     *                           std::string oldState, std::string newState);
     */
    TracedCallback<std::string, std::string> m_stateTransitionTrace;

    // EVENTS

    EventId m_eventRetryConnection; ///<! Event for retrying connection

}; // end of `class NrtvTcpClient`

/**
 * @brief Receive (possibly) fragmented packets from NrtvServer and re-assemble
 *        them to the original video slices they were sent.
 */
class NrtvTcpClientRxBuffer : public SimpleRefCount<NrtvTcpClientRxBuffer>
{
  public:
    /// Create an empty instance of Rx buffer.
    NrtvTcpClientRxBuffer();

    /**
     * @brief Check if the buffer is empty.
     * @return true if the buffer is completely empty
     */
    bool IsEmpty() const;

    /**
     * @brief Check if the buffer contains at least one complete video slice.
     *        If at least one slice is found, PopVideoSlice() can be called.
     * @return true if the buffer contains at least a complete video slice.
     */
    bool HasVideoSlice() const;

    /**
     * @brief Insert a received packet into the buffer.
     * @param packet the packet data to be added
     *
     * @warning If the packet is the first packet of a video slice, it must
     *          contain an NrtvHeader.
     */
    void PushPacket(Ptr<const Packet> packet);

    /**
     * @brief Get and remove the next video slice from the buffer as a packet.
     * @return the next video slice, re-assembled from the packets which have been
     *         received (still including its NrtvHeader)
     *
     * @warning As pre-conditions, IsEmpty() must be false and HasVideoSlice()
     *          must be true before calling this method.
     */
    Ptr<Packet> PopVideoSlice();

  private:
    /**
     * @param packet the packet to be read
     * @return the slice size field of the NRTV header embedded in the packet
     *
     * @warning An NRTV header must be found in the beginning of the packet.
     */
    static uint32_t PeekSliceSize(Ptr<const Packet> packet);

    /// The buffer, containing copies of packets received.
    std::list<Ptr<Packet>> m_rxBuffer;
    /// Overall size of buffer in bytes (including header).
    uint32_t m_totalBytes;
    /// The expected size of the next video slice (zero if size is not yet known).
    uint32_t m_sizeOfVideoSlice;

}; // end of `class NrtvTcpClientRxBuffer`

} // namespace ns3

#endif /* NRTV_TCP_CLIENT_H */
