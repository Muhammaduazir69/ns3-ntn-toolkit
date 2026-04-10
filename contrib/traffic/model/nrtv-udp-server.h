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

#ifndef NRTV_UDP_SERVER_H
#define NRTV_UDP_SERVER_H

#include "ns3/address.h"
#include "ns3/application.h"
#include "ns3/callback.h"
#include "ns3/event-id.h"
#include "ns3/traced-callback.h"

#include <map>
#include <string>

namespace ns3
{

class Packet;
class Socket;
class NrtvVariables;
class NrtvVideoWorker;

/**
 * @ingroup nrtv
 * @brief Model application which simulates the traffic of a Near Real-Time
 *        Video (NRTV) service, i.e., a video streaming service, over UDP.
 *
 * The application provides unidirectional NRTV-like traffic to a multiple
 * targets. This target is specified by giving the address of the target node
 * to the NrtvUdpServer by using its AddClient method.
 *
 * When the transmission of a whole video is completed, the application becomes
 * idle for a random length of time, and then resumes with another video.
 */
class NrtvUdpServer : public Application
{
  public:
    /**
     * @brief Creates a new instance of NRTV server application which operates
     *        over UDP.
     *
     * After creation, the application must be further configured through
     * attributes. To avoid having to do this process manually, please use one of
     * the helper classes (either NrtvHelper or NrtvServerHelper).
     */
    NrtvUdpServer();

    // inherited from ObjectBase base class
    static TypeId GetTypeId();

    /**
     * @brief Adds a client in remote address to the server memory.
     *        a video worker instance will be created for the client.
     *        Once server is started (or if it has already been started),
     *        the video worker starts generating and sending packets.
     *
     * @param remoteAddress Address of the remote application.
     * @param numberOfVideos Number of videos streamed to the client.
     */
    void AddClient(Address remoteAddress, uint32_t numberOfVideos = 1);

    /**
     * @return the address of the destination client
     */
    Address GetRemoteAddress() const;

    /**
     * @return the destination port
     */
    uint16_t GetRemotePort() const;

    /// The possible states of the application.
    enum State_t
    {
        NOT_STARTED = 0, ///< Before StartApplication() is invoked.
        STARTED,         ///< Transmitting video stream to the remote host.
        STOPPED          ///< After StopApplication() is invoked.
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
    /// Invoked by NrtvVideoWorker instance after transmitting a video slice.
    void NotifyTxSlice(Ptr<Socket> socket, Ptr<const Packet> packet);

    /// Invoked by NrtvVideoWorker instance after completed a video.
    void NotifyVideoCompleted(Ptr<Socket> socket);

    /**
     * Add a video worker for the socket. Socket is assumed to be
     * bound to remote address.
     */
    void AddVideoWorker(Ptr<Socket> socket);

    /**
     * Switches the state of the application.
     */
    void SwitchToState(State_t state);

    State_t m_state;                            ///< Internal state of the application
    std::map<Ptr<Socket>, uint32_t> videosLeft; ///< Videos left to be streamed to the socket.
    std::map<Ptr<Socket>, Ptr<NrtvVideoWorker>> m_workers; ///< Worker memory
    Ptr<NrtvVariables> m_nrtvVariables; ///< Nrtv variable collection of this instance

    // ATTRIBUTES

    uint16_t m_remotePort;

    // TRACE SOURCES

    TracedCallback<Ptr<const Packet>> m_txTrace;
    TracedCallback<std::string, std::string> m_stateTransitionTrace;

}; // end of `class NrtvUdpServer`

} // namespace ns3

#endif /* NRTV_UDP_SERVER_H */
