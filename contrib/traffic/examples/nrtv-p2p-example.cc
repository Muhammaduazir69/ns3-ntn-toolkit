/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2016 Magister Solutions
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
 * Authors: Budiarto Herman <budiarto.herman@magister.fi>
 *          Lauri Sormunen <lauri.sormunen@magister.fi>
 */

/**
 * @file
 *
 * @ingroup nrtv
 * @brief Simple example of two nodes connected by a point-to-point link. One
 *        acts as a video streaming server, while the other one acts as the
 *        client.
 *
 * This example demonstrates the use of NrtvClientTracePlot helper class to
 * generate a plot to visualise the Rx traffic experienced by the client. After
 * the example ends, locate the file "nrtv-client-trace.plt" in the working
 * directory, and convert it to a PNG image file using the following command:
 *
 *     $ gnuplot nrtv-client-trace.plt
 *
 * After that, the plot can be seen in the "nrtv-client-trace.png", which can
 * also be found in the working directory.
 */

#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/network-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/traffic-module.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("NrtvP2pExample");

int
main(int argc, char* argv[])
{
    uint32_t simTime = 10;
    std::string protocol = "UDP";
    bool verbose = false;

    /// NRTV video configurations ///

    // We set the bitrate of video a little higher than specified in initial attributes.

    // How often a frame is displayed in milliseconds. Interval 42 ms -> 1000/42 =~ 24 fps
    Config::SetDefault("ns3::NrtvVariables::FrameInterval", TimeValue(MilliSeconds(42)));

    // Maximum size of slices
    Config::SetDefault("ns3::NrtvVideoWorker::MaxSliceSize", UintegerValue(1500));

    // Video variables: video length (in frames), slice sizes
    Config::SetDefault("ns3::NrtvVariables::NumOfFramesMean", UintegerValue(6000));
    Config::SetDefault("ns3::NrtvVariables::SliceEncodingDelayScale", DoubleValue(1));
    Config::SetDefault("ns3::NrtvVariables::SliceEncodingDelayShape", DoubleValue(50));
    Config::SetDefault("ns3::NrtvVariables::SliceEncodingDelayMax", TimeValue(MilliSeconds(2)));
    Config::SetDefault("ns3::NrtvVariables::NumOfSlices", UintegerValue(4));
    Config::SetDefault("ns3::NrtvVariables::SliceSizeScale", DoubleValue(1100));
    Config::SetDefault("ns3::NrtvVariables::SliceSizeMax", UintegerValue(1200));
    Config::SetDefault("ns3::NrtvVariables::SliceSizeShape", DoubleValue(50));

    /// End of NRTV video configurations ///

    /// Parse command line arguments ///

    CommandLine cmd;
    cmd.AddValue("time", "Simulation time in seconds", simTime);
    cmd.AddValue("protocol", "TCP or UDP protocol", protocol);
    cmd.AddValue("verbose", "Print trace information", verbose);
    cmd.Parse(argc, argv);

    /// End of command line arguments ///

    std::stringstream prss;
    prss << "ns3::";
    if (protocol == "TCP")
    {
        prss << "Tcp";
    }
    else if (protocol == "UDP")
    {
        prss << "Udp";
    }
    else
    {
        NS_FATAL_ERROR("Invalid protocol given, use either TCP or UDP in upper case.");
    }
    prss << "SocketFactory";

    /// Log components ///
    if (verbose)
    {
        LogComponentEnable("NrtvTcpClient", LOG_PREFIX_ALL);
        LogComponentEnable("NrtvTcpServer", LOG_PREFIX_ALL);
        LogComponentEnable("NrtvTcpClient", LOG_WARN);
        LogComponentEnable("NrtvTcpServer", LOG_WARN);
        LogComponentEnable("NrtvTcpClient", LOG_ERROR);
        LogComponentEnable("NrtvTcpServer", LOG_ERROR);
        LogComponentEnable("NrtvTcpClient", LOG_INFO);
        LogComponentEnable("NrtvTcpServer", LOG_INFO);
        LogComponentEnable("NrtvTcpClient", LOG_DEBUG);
        LogComponentEnable("NrtvTcpServer", LOG_DEBUG);

        LogComponentEnable("PacketSink", LOG_PREFIX_ALL);
        LogComponentEnable("NrtvUdpServer", LOG_PREFIX_ALL);
        LogComponentEnable("PacketSink", LOG_WARN);
        LogComponentEnable("NrtvUdpServer", LOG_WARN);
        LogComponentEnable("PacketSink", LOG_ERROR);
        LogComponentEnable("NrtvUdpServer", LOG_ERROR);
        LogComponentEnable("PacketSink", LOG_INFO);
        LogComponentEnable("NrtvUdpServer", LOG_INFO);
        LogComponentEnable("PacketSink", LOG_DEBUG);
        LogComponentEnable("NrtvUdpServer", LOG_DEBUG);
    }
    /// End of log components ///

    NodeContainer nodes;
    nodes.Create(2);

    PointToPointHelper pointToPoint;
    pointToPoint.SetDeviceAttribute("DataRate", StringValue("5Mbps"));
    pointToPoint.SetChannelAttribute("Delay", StringValue("2ms"));

    NetDeviceContainer devices;
    devices = pointToPoint.Install(nodes);

    InternetStackHelper stack;
    stack.Install(nodes);

    Ipv4AddressHelper address;
    address.SetBase("10.1.1.0", "255.255.255.0");
    Ipv4InterfaceContainer interfaces = address.Assign(devices);

    /// Server & Client setup using NrtvHelper ///

    NrtvHelper nrtvHelper(TypeId::LookupByName(prss.str()));
    nrtvHelper.SetVariablesAttribute("NumberOfVideos",
                                     StringValue("ns3::UniformRandomVariable[Min=2|Max=5]"));
    nrtvHelper.InstallUsingIpv4(nodes.Get(1), nodes.Get(0));
    nrtvHelper.GetServer().Start(Seconds(2.0));
    nrtvHelper.GetClients().Start(Seconds(1.0));

    /// End of Server & Client setup ///

    /// Plot of packets received by the client application ///

    std::stringstream plotName;
    plotName << "NRTV-" << protocol << "-client-trace";
    Ptr<Application> clientApp = nrtvHelper.GetClients().Get(0);
    Ptr<ClientRxTracePlot> plot = CreateObject<ClientRxTracePlot>(clientApp, plotName.str());
    ;

    /// End of plot configurations ///

    Simulator::Stop(Seconds(simTime));
    Simulator::Run();
    Simulator::Destroy();

    return 0;

} // end of `int main (int argc, char *argv[])`
