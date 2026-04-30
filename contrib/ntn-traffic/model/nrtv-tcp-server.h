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

#ifndef NRTV_TCP_SERVER_H
#define NRTV_TCP_SERVER_H

#include "ns3/address.h"
#include "ns3/application.h"
#include "ns3/nstime.h"
#include "ns3/traced-callback.h"

#include <map>
#include <string>

namespace ns3
{

class Socket;
class NrtvVariables;
class NrtvVideoWorker;

/**
 * @ingroup nrtv
 * @brief Model application which simulates the traffic of a Near Real-Time
 *        Video (NRTV) service, i.e., a video streaming service, over TCP.
 *
 * In summary, the application works as follows. Upon start, the application
 * opens a socket and listens to connection requests from clients (NrtvTcpClient).
 * Once the request is accepted (always) and a connection is established, the
 * application begins to send a video (as a stream of packets) to the client.
 * When the transmission of the whole video is completed, the application
 * disconnects the client.
 *
 * The application maintains several workers (NrtvTcpServerVideoWorker). Each
 * worker is responsible for sending a single video for a single client.
 */
class NrtvTcpServer : public Application
{
  public:
    /**
     * @brief Creates a new instance of NRTV server application which operates
     *        over TCP.
     *
     * After creation, the application must be further configured through
     * attributes. To avoid having to do this process manually, please use one of
     * the helper classes (either NrtvHelper or NrtvTcpServerHelper).
     */
    NrtvTcpServer();

    // inherited from ObjectBase base class
    static TypeId GetTypeId();

    /**
     * @return the address bound to the server
     */
    Address GetLocalAddress() const;

    /**
     * @return the port the server listens to
     */
    uint16_t GetLocalPort() const;

    /// The possible states of the application.
    enum State_t
    {
        NOT_STARTED = 0, ///< Before StartApplication() is invoked.
        STARTED,         ///< Passively waiting for connections and/or actively sending videos.
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
    // LISTENER SOCKET CALLBACK METHODS
    bool ConnectionRequestCallback(Ptr<Socket> socket, const Address& address);
    void NewConnectionCreatedCallback(Ptr<Socket> socket, const Address& address);

    void NormalCloseCallback(Ptr<Socket> socket);
    void ErrorCloseCallback(Ptr<Socket> socket);

    /// Invoked by NrtvVideoWorker instance after transmitting a video slice.
    void NotifyTxSlice(Ptr<Socket> socket, Ptr<const Packet> packet);

    /// Invoked by NrtvVideoWorker instance after completed a video.
    void NotifyVideoCompleted(Ptr<Socket> socket);

    void SwitchToState(State_t state);

    State_t m_state;
    Ptr<Socket> m_initialSocket;

    /// Keeping all the active workers.
    std::map<Ptr<Socket>, Ptr<NrtvVideoWorker>> m_workers;

    // ATTRIBUTES

    Address m_localAddress;
    uint16_t m_localPort;

    // TRACE SOURCES

    TracedCallback<Ptr<const Packet>> m_txTrace;
    TracedCallback<std::string, std::string> m_stateTransitionTrace;

}; // end of `class NrtvTcpServer`

} // namespace ns3

#endif /* NRTV_TCP_SERVER_H */
