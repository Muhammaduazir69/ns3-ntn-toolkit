/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2008 INRIA
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
 * Original work author (from packet-sink-helper.cc):
 * - Mathieu Lacage <mathieu.lacage@sophia.inria.fr>
 *
 * Converted to NRTV traffic models by:
 * - Budiarto Herman <budiarto.herman@magister.fi>
 * - Lauri Sormunen <lauri.sormunen@magister.fi>
 *
 */

#ifndef NRTV_HELPER_H
#define NRTV_HELPER_H

#include "ns3/application-container.h"
#include "ns3/ipv4-address.h"
#include "ns3/node-container.h"
#include "ns3/nrtv-udp-server.h"
#include "ns3/nrtv-variables.h"
#include "ns3/object-factory.h"

#include <map>
#include <string>
#include <tuple>
#include <vector>

namespace ns3
{

/**
 * @ingroup nrtv
 * @brief Helper to make it easier to instantiate an NrtvClient on a set of
 *        nodes.
 */
class NrtvClientHelper
{
  public:
    /**
     * @brief Create a NrtvClientHelper to make it easier to work with
     *        NrtvClient applications.
     *
     * @param protocolTid The TypeId of the protocol used (either TcpSocketFactory
     *                    or UdpSocketFactory)
     * @param address In case of TCP: the address of the remote server
     *                node to send traffic to. In case of UDP: the local address.
     */
    NrtvClientHelper(TypeId protocolTid, Address address);

    /**
     * @brief Helper function used to set the underlying application attributes,
     *        but *not* the socket attributes.
     *
     * @param name the name of the application attribute to set
     * @param value the value of the application attribute to set
     */
    void SetAttribute(std::string name, const AttributeValue& value);

    /**
     * @brief Install an NrtvClient on each node of the input container
     *        configured with all the attributes set with SetAttribute().
     *
     * @param c NodeContainer of the set of nodes on which an NrtvClient will be
     *          installed
     * @return Container of Ptr to the applications installed
     */
    ApplicationContainer Install(NodeContainer c) const;

    /**
     * @brief Install an NrtvClient on each node of the input container
     *        configured with all the attributes set with SetAttribute().
     *
     * @param node The node on which an NrtvClient will be installed
     * @return Container of Ptr to the applications installed
     */
    ApplicationContainer Install(Ptr<Node> node) const;

    /**
     * @brief Install an NrtvClient on each node of the input container
     *        configured with all the attributes set with SetAttribute().
     *
     * @param nodeName The name of the node on which an NrtvClient will be
     *                 installed
     * @return Container of Ptr to the applications installed
     */
    ApplicationContainer Install(std::string nodeName) const;

  private:
    /**
     * @internal
     * @brief Install an NrtvClient on the node configured with all the
     *        attributes set with SetAttribute().
     *
     * @param node The node on which an NrtvClient will be installed
     * @return Ptr to the application installed
     */
    Ptr<Application> InstallPriv(Ptr<Node> node) const;

    ObjectFactory m_factory;

}; // end of `class NrtvClientHelper`

/**
 * @ingroup nrtv
 * @brief Helper to make it easier to instantiate an NrtvServer on a set of
 *        nodes.
 */
class NrtvServerHelper
{
  public:
    /**
     * @brief Create a NrtvServerHelper to make it easier to work with
     *        NrtvServer applications.
     *
     * @param protocolTid The TypeId of the protocol used (either TcpSocketFactory or
     * UdpSocketFactory) \param address In case of TCP the address of the server, in case of UDP the
     * remote address.
     */
    NrtvServerHelper(TypeId protocolTid, Address address);

    /**
     * @brief Helper function used to set the underlying application attributes,
     *        but *not* the socket attributes.
     *
     * @param name the name of the application attribute to set
     * @param value the value of the application attribute to set
     */
    void SetAttribute(std::string name, const AttributeValue& value);

    /**
     * @brief Install an NrtvServer on each node of the input container
     *        configured with all the attributes set with SetAttribute().
     *
     * @param c NodeContainer of the set of nodes on which an NrtvServer will be
     *          installed
     * @return Container of Ptr to the applications installed
     */
    ApplicationContainer Install(NodeContainer c) const;

    /**
     * @brief Install an NrtvServer on each node of the input container
     *        configured with all the attributes set with SetAttribute().
     *
     * @param node The node on which an NrtvServer will be installed
     * @return Container of Ptr to the applications installed
     */
    ApplicationContainer Install(Ptr<Node> node) const;

    /**
     * @brief Install an NrtvServer on each node of the input container
     *        configured with all the attributes set with SetAttribute().
     *
     * @param nodeName The name of the node on which an NrtvServer will be
     *                 installed
     * @return Container of Ptr to the applications installed
     */
    ApplicationContainer Install(std::string nodeName) const;

  private:
    /**
     * @internal
     * @brief Install an NrtvServer on the node configured with all the
     *        attributes set with SetAttribute().
     *
     * @param node The node on which an NrtvServer will be installed
     * @return Ptr to the application installed
     */
    Ptr<Application> InstallPriv(Ptr<Node> node) const;

    ObjectFactory m_factory;

}; // end of `class NrtvServerHelper`

/**
 * @ingroup nrtv
 * @brief Helper to make it easier to instantiate an NRTV server and a group of
 *        connected NRTV clients.
 */
class NrtvHelper
{
  public:
    /**
     * @brief Create a NrtvHelper to make it easier to work with
     *        NrtvTcpClient and NrtvTcpServer or alternatively PacketSink
     *        and NrtvUdpServer applications.
     *
     * @param protocolTid The TypeId of the protocol to be used by the server
     *                    and client. Either TypeId of TcpSocketFactory or UdpSocketFactory.
     */
    NrtvHelper(TypeId protocolTid = TypeId::LookupByName("ns3::TcpSocketFactory"));

    /// Instance destructor.
    virtual ~NrtvHelper();

    /**
     * @brief Helper function used to set the underlying NrtvTcpClient or PacketSink
     *        application attributes, but *not* the socket attributes.
     *
     * @param name the name of the application attribute to set
     * @param value the value of the application attribute to set
     */
    void SetClientAttribute(std::string name, const AttributeValue& value);

    /**
     * @brief Helper function used to set the underlying NrtvTcpServer
     *        or NrtvUdpServer application attributes, but *not* the
     *        socket attributes.
     *
     * @param name the name of the application attribute to set
     * @param value the value of the application attribute to set
     */
    void SetServerAttribute(std::string name, const AttributeValue& value);

    /**
     * @brief Helper function used to set the NrtvVariables attributes
     *        used by this helper instance. Currently, only number of videos
     *        specified in the Variables class is used.
     *
     * @param name the name of the application attribute to set
     * @param value the value of the application attribute to set
     */
    void SetVariablesAttribute(std::string name, const AttributeValue& value);

    /**
     * @brief Install an Nrtv Server application and several Nrtv client
     *        applications, in which each client is connected using IPv4 to the
     *        server. The server will be either an NrtvTcpServer or a NrtvUdpServer.
     *
     * @param serverNode the node on which an NrtvServer will be installed
     * @param clientNodes the set of nodes on which NrtvClient applications will
     *                    be installed
     * @return container of Ptr to the server and client applications installed
     *
     * @warning The given nodes must have Internet stack installed properly
     *          before this method can be called.
     *
     * The installed applications will be configured with all the attributes set
     * with SetClientAttribute() and SetServerAttribute(). Pointers to these
     * applications can be retrieved afterwards by calling GetClients() and
     * GetServer() methods separately.
     */
    ApplicationContainer InstallUsingIpv4(Ptr<Node> serverNode, NodeContainer clientNodes);

    /**
     * @brief Install an Nrtv Server application and an Nrtv client
     *        applications, in which each client is connected using IPv4 to the
     *        server. The server will be either an NrtvTcpServer or a NrtvUdpServer.
     *
     * @param serverNode the node on which an NrtvServer will be installed
     * @param clientNode the node on which an NrtvClient will be installed
     * @return container of Ptr to the server and client applications installed
     *
     * @warning The given nodes must have Internet stack installed properly
     *          before this method can be called.
     *
     * The installed applications will be configured with all the attributes set
     * with SetClientAttribute() and SetServerAttribute(). Pointers to these
     * applications can be retrieved afterwards by calling GetClients() and
     * GetServer() methods separately.
     */
    ApplicationContainer InstallUsingIpv4(Ptr<Node> serverNode, Ptr<Node> clientNode);

    /**
     * @brief Retrieve pointers to the NRTV clients which were installed by the
     *        previous call of Install().
     *
     * @return an application container containing NRTV clients, or an empty
     *         container if Install() has never been called before
     */
    ApplicationContainer GetClients() const;

    /**
     * @brief Retrieve a pointer to the NRTV server which was installed by the
     *        previous call of Install().
     *
     * @return an application container containing a single NRTV server, or an
     *         empty container if Install() has never been called before
     */
    ApplicationContainer GetServer() const;

  private:
    NrtvServerHelper* m_serverHelper;
    NrtvClientHelper* m_clientHelper;
    Ptr<NrtvVariables> m_nrtvVariables;
    ApplicationContainer m_lastInstalledClients;
    ApplicationContainer m_lastInstalledServer;

    /**
     * TypeId of the protocol to be used
     */
    TypeId m_protocolTid;

}; // end of `class NrtvHelper`

} // namespace ns3

#endif /* NRTV_HELPER_H */
