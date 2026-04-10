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
 * Converted to 3GPP HTTP web browsing traffic models by:
 * - Budiarto Herman <budiarto.herman@magister.fi>
 *
 * Adapted to ns-3.29 architecture by:
 * - Patrice Raveneau <patrice.raveneau@cnes.fr>
 *
 */

#include "three-gpp-http-satellite-helper.h"

#include "ns3/ipv4.h"
#include "ns3/names.h"

#include <string>

namespace ns3
{

// 3GPP HTTP SATELLITE CLIENT HELPER /////////////////////////////////////////////////////////

ThreeGppHttpSatelliteClientHelper::ThreeGppHttpSatelliteClientHelper(const Address& address)
{
    m_factory.SetTypeId("ns3::ThreeGppHttpSatelliteClient");
    m_factory.Set("RemoteServerAddress", AddressValue(address));
}

void
ThreeGppHttpSatelliteClientHelper::SetAttribute(const std::string& name,
                                                const AttributeValue& value)
{
    m_factory.Set(name, value);
}

ApplicationContainer
ThreeGppHttpSatelliteClientHelper::Install(Ptr<Node> node) const
{
    return ApplicationContainer(InstallPriv(node));
}

ApplicationContainer
ThreeGppHttpSatelliteClientHelper::Install(const std::string& nodeName) const
{
    Ptr<Node> node = Names::Find<Node>(nodeName);
    return ApplicationContainer(InstallPriv(node));
}

ApplicationContainer
ThreeGppHttpSatelliteClientHelper::Install(NodeContainer c) const
{
    ApplicationContainer apps;
    for (NodeContainer::Iterator i = c.Begin(); i != c.End(); ++i)
    {
        apps.Add(InstallPriv(*i));
    }

    return apps;
}

Ptr<Application>
ThreeGppHttpSatelliteClientHelper::InstallPriv(Ptr<Node> node) const
{
    Ptr<Application> app = m_factory.Create<Application>();
    node->AddApplication(app);

    return app;
}

// HTTP SERVER HELPER /////////////////////////////////////////////////////////

ThreeGppHttpSatelliteServerHelper::ThreeGppHttpSatelliteServerHelper(const Address& address)
{
    m_factory.SetTypeId("ns3::ThreeGppHttpServer");
    m_factory.Set("LocalAddress", AddressValue(address));
}

void
ThreeGppHttpSatelliteServerHelper::SetAttribute(const std::string& name,
                                                const AttributeValue& value)
{
    m_factory.Set(name, value);
}

ApplicationContainer
ThreeGppHttpSatelliteServerHelper::Install(Ptr<Node> node) const
{
    return ApplicationContainer(InstallPriv(node));
}

ApplicationContainer
ThreeGppHttpSatelliteServerHelper::Install(const std::string& nodeName) const
{
    Ptr<Node> node = Names::Find<Node>(nodeName);
    return ApplicationContainer(InstallPriv(node));
}

ApplicationContainer
ThreeGppHttpSatelliteServerHelper::Install(NodeContainer c) const
{
    ApplicationContainer apps;
    for (NodeContainer::Iterator i = c.Begin(); i != c.End(); ++i)
    {
        apps.Add(InstallPriv(*i));
    }

    return apps;
}

Ptr<Application>
ThreeGppHttpSatelliteServerHelper::InstallPriv(Ptr<Node> node) const
{
    Ptr<Application> app = m_factory.Create<Application>();
    node->AddApplication(app);

    return app;
}

// THREE GPP HTTP HELPER ////////////////////////////////////////////////////////////////

ThreeGppHttpHelper::ThreeGppHttpHelper()
{
    Address invalidAddr;
    m_clientHelper = new ThreeGppHttpSatelliteClientHelper(invalidAddr);
    m_serverHelper = new ThreeGppHttpSatelliteServerHelper(invalidAddr);
    m_httpVariables = CreateObject<ThreeGppHttpVariables>();
}

ThreeGppHttpHelper::~ThreeGppHttpHelper()
{
    delete m_clientHelper;
    delete m_serverHelper;
}

void
ThreeGppHttpHelper::SetClientAttribute(std::string name, const AttributeValue& value)
{
    m_clientHelper->SetAttribute(name, value);
}

void
ThreeGppHttpHelper::SetServerAttribute(std::string name, const AttributeValue& value)
{
    m_serverHelper->SetAttribute(name, value);
}

void
ThreeGppHttpHelper::SetVariablesAttribute(std::string name, const AttributeValue& value)
{
    m_httpVariables->SetAttribute(name, value);
}

ApplicationContainer
ThreeGppHttpHelper::InstallUsingIpv4(Ptr<Node> serverNode, NodeContainer clientNodes)
{
    ApplicationContainer ret; // the return value of the function

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

        m_serverHelper->SetAttribute("LocalAddress", AddressValue(serverAddress));

        m_lastInstalledServer = m_serverHelper->Install(serverNode);
        ret.Add(m_lastInstalledServer);

        m_clientHelper->SetAttribute("RemoteServerAddress", AddressValue(serverAddress));
        m_lastInstalledClients = m_clientHelper->Install(clientNodes);
        ret.Add(m_lastInstalledClients);
    }

    return ret;
}

ApplicationContainer
ThreeGppHttpHelper::InstallUsingIpv4(Ptr<Node> serverNode, Ptr<Node> clientNode)
{
    return InstallUsingIpv4(serverNode, NodeContainer(clientNode));
}

ApplicationContainer
ThreeGppHttpHelper::GetClients() const
{
    return m_lastInstalledClients;
}

ApplicationContainer
ThreeGppHttpHelper::GetServer() const
{
    return m_lastInstalledServer;
}

} // namespace ns3
