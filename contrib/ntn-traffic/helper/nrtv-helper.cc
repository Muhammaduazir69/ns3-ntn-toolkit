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
 *
 */

#include "nrtv-helper.h"

#include "ns3/inet-socket-address.h"
#include "ns3/ipv4.h"
#include "ns3/names.h"
#include "ns3/string.h"

#include <map>
#include <string>
#include <tuple>
#include <vector>

namespace ns3
{

// NRTV CLIENT HELPER /////////////////////////////////////////////////////////

NrtvClientHelper::NrtvClientHelper(TypeId protocolTid, Address address)
{
    NS_ASSERT_MSG(protocolTid == TypeId::LookupByName("ns3::TcpSocketFactory") ||
                      protocolTid == TypeId::LookupByName("ns3::UdpSocketFactory"),
                  "The protocol should be set either to TypeId of TcpSocketFactory "
                  "or TypeId of UdpSocketFactory!");

    if (protocolTid == TypeId::LookupByName("ns3::TcpSocketFactory"))
    {
        m_factory.SetTypeId("ns3::NrtvTcpClient");
        m_factory.Set("RemoteServerAddress", AddressValue(address));
    }
    else
    {
        m_factory.SetTypeId("ns3::PacketSink");
        m_factory.Set("Protocol", TypeIdValue(protocolTid));
        m_factory.Set("Local", AddressValue(address));
    }
}

void
NrtvClientHelper::SetAttribute(std::string name, const AttributeValue& value)
{
    m_factory.Set(name, value);
}

ApplicationContainer
NrtvClientHelper::Install(Ptr<Node> node) const
{
    return ApplicationContainer(InstallPriv(node));
}

ApplicationContainer
NrtvClientHelper::Install(std::string nodeName) const
{
    Ptr<Node> node = Names::Find<Node>(nodeName);
    return ApplicationContainer(InstallPriv(node));
}

ApplicationContainer
NrtvClientHelper::Install(NodeContainer c) const
{
    ApplicationContainer apps;
    for (NodeContainer::Iterator i = c.Begin(); i != c.End(); ++i)
    {
        apps.Add(InstallPriv(*i));
    }

    return apps;
}

Ptr<Application>
NrtvClientHelper::InstallPriv(Ptr<Node> node) const
{
    Ptr<Application> app = m_factory.Create<Application>();
    node->AddApplication(app);

    return app;
}

// NRTV SERVER HELPER /////////////////////////////////////////////////////////

NrtvServerHelper::NrtvServerHelper(TypeId protocolTid, Address address)
{
    if (protocolTid == TypeId::LookupByName("ns3::TcpSocketFactory"))
    {
        m_factory.SetTypeId("ns3::NrtvTcpServer");
        m_factory.Set("LocalAddress", AddressValue(address));
    }
    else if (protocolTid == TypeId::LookupByName("ns3::UdpSocketFactory"))
    {
        m_factory.SetTypeId("ns3::NrtvUdpServer");
    }
}

void
NrtvServerHelper::SetAttribute(std::string name, const AttributeValue& value)
{
    m_factory.Set(name, value);
}

ApplicationContainer
NrtvServerHelper::Install(Ptr<Node> node) const
{
    return ApplicationContainer(InstallPriv(node));
}

ApplicationContainer
NrtvServerHelper::Install(std::string nodeName) const
{
    Ptr<Node> node = Names::Find<Node>(nodeName);
    return ApplicationContainer(InstallPriv(node));
}

ApplicationContainer
NrtvServerHelper::Install(NodeContainer c) const
{
    ApplicationContainer apps;
    for (NodeContainer::Iterator i = c.Begin(); i != c.End(); ++i)
    {
        apps.Add(InstallPriv(*i));
    }

    return apps;
}

Ptr<Application>
NrtvServerHelper::InstallPriv(Ptr<Node> node) const
{
    Ptr<Application> app = m_factory.Create<Application>();
    node->AddApplication(app);

    return app;
}

// NRTV HELPER ////////////////////////////////////////////////////////////////

NrtvHelper::NrtvHelper(TypeId protocolTid)
    : m_protocolTid(protocolTid)
{
    NS_ASSERT_MSG(m_protocolTid == TypeId::LookupByName("ns3::TcpSocketFactory") ||
                      m_protocolTid == TypeId::LookupByName("ns3::UdpSocketFactory"),
                  "The protocol should be set either to TypeId of TcpSocketFactory "
                  "or TypeId of UdpSocketFactory!");

    Address invalidAddr;
    m_clientHelper = new NrtvClientHelper(m_protocolTid, invalidAddr);
    m_serverHelper = new NrtvServerHelper(m_protocolTid, invalidAddr);
    m_nrtvVariables = CreateObject<NrtvVariables>();
}

NrtvHelper::~NrtvHelper()
{
    delete m_clientHelper;
    delete m_serverHelper;
}

void
NrtvHelper::SetClientAttribute(std::string name, const AttributeValue& value)
{
    m_clientHelper->SetAttribute(name, value);
}

void
NrtvHelper::SetServerAttribute(std::string name, const AttributeValue& value)
{
    m_serverHelper->SetAttribute(name, value);
}

void
NrtvHelper::SetVariablesAttribute(std::string name, const AttributeValue& value)
{
    m_nrtvVariables->SetAttribute(name, value);
}

ApplicationContainer
NrtvHelper::InstallUsingIpv4(Ptr<Node> serverNode, NodeContainer clientNodes)
{
    ApplicationContainer ret; // the return value of the function
    bool tcpInUse = m_protocolTid == TypeId::LookupByName("ns3::TcpSocketFactory");

    Ptr<Ipv4> ipv4 = serverNode->GetObject<Ipv4>();
    if (ipv4 == nullptr)
    {
        NS_FATAL_ERROR("No IPv4 object is found within the server node " << serverNode);
    }
    else
    {
        /// Still unclear if the hard-coded indices below will work in any possible cases.
        const Ipv4InterfaceAddress interfaceAddress = ipv4->GetAddress(1, 0);
        const Ipv4Address serverAddress = interfaceAddress.GetLocal();

        if (tcpInUse)
        {
            m_serverHelper->SetAttribute("LocalAddress", AddressValue(serverAddress));
        }

        m_lastInstalledServer = m_serverHelper->Install(serverNode);
        ret.Add(m_lastInstalledServer);

        if (tcpInUse)
        {
            // If TCP server is used, installation is straightforward.
            m_clientHelper->SetAttribute("RemoteServerAddress", AddressValue(serverAddress));
            m_lastInstalledClients = m_clientHelper->Install(clientNodes);
        }
        else
        {
            // If UDP is used, we need to configure PacketSink "Local" attribute
            // for each client node after installation.
            m_lastInstalledClients = ApplicationContainer();
            Ptr<NrtvUdpServer> serverApp = m_lastInstalledServer.Get(0)->GetObject<NrtvUdpServer>();
            for (auto it = clientNodes.Begin(); it != clientNodes.End(); it++)
            {
                Ptr<Ipv4> clientIpv4 = (*it)->GetObject<Ipv4>();
                const Ipv4InterfaceAddress clientInterfaceAddress = clientIpv4->GetAddress(1, 0);
                const Ipv4Address clientAddress = clientInterfaceAddress.GetLocal();
                ApplicationContainer apps = m_clientHelper->Install((*it));
                for (auto it2 = apps.Begin(); it2 != apps.End(); it2++)
                {
                    (*it2)->SetAttribute(
                        "Local",
                        AddressValue(InetSocketAddress(clientAddress, serverApp->GetRemotePort())));
                }
                m_lastInstalledClients.Add(apps);
                serverApp->AddClient(clientAddress, m_nrtvVariables->GetNumOfVideos());
            }
        }
        ret.Add(m_lastInstalledClients);
    }

    return ret;
}

ApplicationContainer
NrtvHelper::InstallUsingIpv4(Ptr<Node> serverNode, Ptr<Node> clientNode)
{
    return InstallUsingIpv4(serverNode, NodeContainer(clientNode));
}

ApplicationContainer
NrtvHelper::GetClients() const
{
    return m_lastInstalledClients;
}

ApplicationContainer
NrtvHelper::GetServer() const
{
    return m_lastInstalledServer;
}

} // namespace ns3
