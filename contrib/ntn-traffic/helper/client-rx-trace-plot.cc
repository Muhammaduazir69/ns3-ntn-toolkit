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

#include "client-rx-trace-plot.h"

#include "ns3/address.h"
#include "ns3/log.h"
#include "ns3/nrtv-tcp-client.h"
#include "ns3/packet-sink.h"
#include "ns3/packet.h"
#include "ns3/simulator.h"

#include <fstream>
#include <string>

NS_LOG_COMPONENT_DEFINE("ClientTracePlot");

namespace ns3
{

NS_OBJECT_ENSURE_REGISTERED(ClientRxTracePlot);

ClientRxTracePlot::ClientRxTracePlot(Ptr<Application> clientApp, std::string outputName)
    : m_client(clientApp),
      m_outputName(outputName)
{
    NS_LOG_FUNCTION(this << m_client << m_outputName);

    Initialize();
}

ClientRxTracePlot::ClientRxTracePlot(Ptr<Application> clientApp)
    : m_client(clientApp),
      m_outputName("client-trace")
{
    NS_LOG_FUNCTION(this << m_client << m_outputName);

    Initialize();
}

void
ClientRxTracePlot::Initialize()
{
    if (m_client == nullptr)
    {
        NS_FATAL_ERROR("Invalid NrtvTcpClient object is given");
    }

    m_client->TraceConnectWithoutContext("Rx", MakeCallback(&ClientRxTracePlot::RxCallback, this));

    m_packet.SetTitle("Packet");
    m_packet.SetStyle(Gnuplot2dDataset::IMPULSES);
    m_counter = 0;
}

ClientRxTracePlot::~ClientRxTracePlot()
{
    NS_LOG_FUNCTION(this);

    Plot();
}

TypeId
ClientRxTracePlot::GetTypeId()
{
    static TypeId tid = TypeId("ns3::ClientTracePlot").SetParent<Object>()
        // for future attributes to configure how the plot is drawn?
        ;
    return tid;
}

void
ClientRxTracePlot::SetOutputName(std::string outputName)
{
    NS_LOG_FUNCTION(this << outputName);
    m_outputName = outputName;
}

std::string
ClientRxTracePlot::GetOutputName() const
{
    return m_outputName;
}

void
ClientRxTracePlot::Plot()
{
    NS_LOG_FUNCTION(this << m_outputName);

    Gnuplot plot(m_outputName + ".png");
    plot.SetTitle("NRTV Client Traffic Trace");
    plot.SetTerminal("png");
    plot.SetLegend("Time (in seconds)", "Bytes received");
    plot.AddDataset(m_packet);
    const std::string plotFileName = m_outputName + ".plt";
    std::ofstream plotFile(plotFileName.c_str());
    plot.GenerateOutput(plotFile);
    plotFile.close();
}

void
ClientRxTracePlot::RxCallback(Ptr<const Packet> packet, const Address& from)
{
    NS_LOG_FUNCTION(this << packet << from);
    m_counter++;
    m_packet.Add(Simulator::Now().GetSeconds(), static_cast<double>(packet->GetSize()));
}

} // namespace ns3
