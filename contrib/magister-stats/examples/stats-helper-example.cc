/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2016 Magister Solutions Ltd
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
 * Author: Lauri Sormunen <lauri.sormunen@magister.fi>
 */

#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/magister-stats-module.h"
#include "ns3/network-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/stats-module.h"

#include <string>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("StatsHelperExample");

int
main(int argc, char* argv[])
{
    std::string outputPath = "output";
    uint32_t timeSec = 60;

    CommandLine cmd;
    cmd.AddValue("SimulationTime", "Simulation time in seconds.", timeSec);
    cmd.AddValue("OutputPath", "Output path for the statistics files.", outputPath);
    cmd.Parse(argc, argv);

    // Setup two nodes
    NodeContainer nodes;
    nodes.Create(2);

    Time::SetResolution(Time::NS);
    LogComponentEnableAll(LOG_PREFIX_TIME);
    LogComponentEnableAll(LOG_PREFIX_FUNC);
    LogComponentEnable("StatsHelperExample", LOG_LEVEL_ALL);
    LogComponentEnable("PacketSink", LOG_INFO);
    LogComponentEnable("OnOffApplication", LOG_INFO);

    PointToPointHelper pointToPoint;
    pointToPoint.SetDeviceAttribute("DataRate", StringValue("5Mbps"));
    pointToPoint.SetChannelAttribute("Delay", StringValue("15ms"));

    NetDeviceContainer devices;
    devices = pointToPoint.Install(nodes);

    InternetStackHelper stack;
    stack.Install(nodes);

    Ipv4AddressHelper address;
    address.SetBase("10.1.1.0", "255.255.255.0");

    Ipv4InterfaceContainer interfaces = address.Assign(devices);

    OnOffHelper senderHelper("ns3::TcpSocketFactory",
                             InetSocketAddress(interfaces.GetAddress(1), 9));
    PacketSinkHelper receiverHelper("ns3::TcpSocketFactory",
                                    InetSocketAddress(interfaces.GetAddress(1), 9));
    ApplicationContainer receiverApps = receiverHelper.Install(nodes.Get(1));
    ApplicationContainer senderApps = senderHelper.Install(nodes.Get(0));
    receiverApps.Start(Seconds(1));
    receiverApps.Stop(Seconds(600));
    senderApps.Start(Seconds(2));
    senderApps.Stop(Seconds(600));

    Simulator::Stop(Seconds(timeSec));

    // Create statistics helpers

    // By default, we save the statistics to directory given as argument
    Config::SetDefault("ns3::StatsHelper::OutputPath", StringValue(outputPath));

    Ptr<StatsAppDelayHelper> delayScatterByNode = CreateObject<StatsAppDelayHelper>();
    delayScatterByNode->SetName("stat-app-delay-scatter-node");
    delayScatterByNode->SetIdentifierType(StatsHelper::IDENTIFIER_NODE);
    delayScatterByNode->SetOutputType(StatsHelper::OUTPUT_SCATTER_FILE);
    delayScatterByNode->InstallNodes(nodes);
    delayScatterByNode->Install();

    Ptr<StatsAppDelayHelper> delayScalarByNode = CreateObject<StatsAppDelayHelper>();
    delayScalarByNode->SetName("stat-app-delay-scalar-node");
    delayScalarByNode->SetIdentifierType(StatsHelper::IDENTIFIER_NODE);
    delayScalarByNode->SetOutputType(StatsHelper::OUTPUT_SCALAR_FILE);
    delayScalarByNode->InstallNodes(nodes);
    delayScalarByNode->Install();

    Ptr<StatsAppDelayHelper> delayScalarGlobal = CreateObject<StatsAppDelayHelper>();
    delayScalarGlobal->SetName("stat-app-delay-scalar-global");
    delayScalarGlobal->SetIdentifierType(StatsHelper::IDENTIFIER_GLOBAL);
    delayScalarGlobal->SetOutputType(StatsHelper::OUTPUT_SCALAR_FILE);
    delayScalarGlobal->InstallNodes(nodes);
    delayScalarGlobal->Install();

    Ptr<StatsAppDelayHelper> delayScatterGlobalPlot = CreateObject<StatsAppDelayHelper>();
    delayScatterGlobalPlot->SetName("stat-app-delay-scatter-plot-global");
    delayScatterGlobalPlot->SetIdentifierType(StatsHelper::IDENTIFIER_GLOBAL);
    delayScatterGlobalPlot->SetOutputType(StatsHelper::OUTPUT_SCATTER_PLOT);
    delayScatterGlobalPlot->InstallNodes(nodes);
    delayScatterGlobalPlot->Install();

    Ptr<StatsAppDelayHelper> delayCdfByNodePlot = CreateObject<StatsAppDelayHelper>();
    delayCdfByNodePlot->SetName("stat-app-delay-cdf-plot-node");
    delayCdfByNodePlot->SetIdentifierType(StatsHelper::IDENTIFIER_NODE);
    delayCdfByNodePlot->SetOutputType(StatsHelper::OUTPUT_CDF_PLOT);
    delayCdfByNodePlot->InstallNodes(nodes);
    delayCdfByNodePlot->Install();

    Ptr<StatsAppThroughputHelper> throughputScatterByNode =
        CreateObject<StatsAppThroughputHelper>();
    throughputScatterByNode->SetName("stat-app-throughput-scatter-node");
    throughputScatterByNode->SetIdentifierType(StatsHelper::IDENTIFIER_NODE);
    throughputScatterByNode->SetOutputType(StatsHelper::OUTPUT_SCATTER_FILE);
    throughputScatterByNode->InstallNodes(nodes);
    throughputScatterByNode->Install();

    Ptr<StatsAppThroughputHelper> throughputScalarByNode = CreateObject<StatsAppThroughputHelper>();
    throughputScalarByNode->SetName("stat-app-throughput-scalar-node");
    throughputScalarByNode->SetIdentifierType(StatsHelper::IDENTIFIER_NODE);
    throughputScalarByNode->SetOutputType(StatsHelper::OUTPUT_SCALAR_FILE);
    throughputScalarByNode->InstallNodes(nodes);
    throughputScalarByNode->Install();

    Ptr<StatsAppThroughputHelper> throughputScalarGlobal = CreateObject<StatsAppThroughputHelper>();
    throughputScalarGlobal->SetName("stat-app-throughput-scalar-global");
    throughputScalarGlobal->SetIdentifierType(StatsHelper::IDENTIFIER_GLOBAL);
    throughputScalarGlobal->SetOutputType(StatsHelper::OUTPUT_SCALAR_FILE);
    throughputScalarGlobal->InstallNodes(nodes);
    throughputScalarGlobal->Install();

    Ptr<StatsAppThroughputHelper> throughputScatterGlobalPlot =
        CreateObject<StatsAppThroughputHelper>();
    throughputScatterGlobalPlot->SetName("stat-app-throughput-scatter-plot-global");
    throughputScatterGlobalPlot->SetIdentifierType(StatsHelper::IDENTIFIER_GLOBAL);
    throughputScatterGlobalPlot->SetOutputType(StatsHelper::OUTPUT_SCATTER_PLOT);
    throughputScatterGlobalPlot->InstallNodes(nodes);
    throughputScatterGlobalPlot->Install();

    Ptr<StatsAppThroughputHelper> throughputCdfByNodePlot =
        CreateObject<StatsAppThroughputHelper>();
    throughputCdfByNodePlot->SetAveragingMode(true);
    throughputCdfByNodePlot->SetName("stat-app-throughput-cdf-plot-node");
    throughputCdfByNodePlot->SetIdentifierType(StatsHelper::IDENTIFIER_NODE);
    throughputCdfByNodePlot->SetOutputType(StatsHelper::OUTPUT_CDF_PLOT);
    throughputCdfByNodePlot->InstallNodes(nodes);
    throughputCdfByNodePlot->Install();

    // Run the simulation

    Simulator::Run();
    Simulator::Destroy();

    // Dispose of each statistics helper.
    // Upon calling Dispose, the helpers will produce output files.

    delayScatterByNode->Dispose();
    delayScalarByNode->Dispose();
    delayScalarGlobal->Dispose();
    delayScatterGlobalPlot->Dispose();
    delayCdfByNodePlot->Dispose();

    throughputScatterByNode->Dispose();
    throughputScalarByNode->Dispose();
    throughputScalarGlobal->Dispose();
    throughputScatterGlobalPlot->Dispose();
    throughputCdfByNodePlot->Dispose();
    return 0;
}
