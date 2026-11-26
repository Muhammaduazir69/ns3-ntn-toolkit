/*
 * Copyright (c) 2011-2018 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
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
 * Authors: Jaume Nin <jaume.nin@cttc.cat>
 *          Manuel Requena <manuel.requena@cttc.es>
 * Modify:  Pengyu Liu <eic_lpy@hust.edu.cn>
 *          Hao Yin <haoyin@uw.edu>
 *          Muyuan Shen <muyuan_shen@hust.edu.cn>
 */

#include "ns3/ntn-dataset-writer.h"
#include "../../ntn-wifi-site-context.h"
#include <filesystem>
#include <algorithm>
#include <fstream>
#include "ns3/applications-module.h"
#include "ns3/config-store-module.h"
#include "ns3/core-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/internet-module.h"
#include "ns3/lte-module.h"
#include "ns3/mobility-module.h"
#include "ns3/point-to-point-module.h"

#include "my-rr-sched.h"

using namespace ns3;
using namespace std;

NS_LOG_COMPONENT_DEFINE("LenaSimpleEpc");

int
main(int argc, char* argv[])
{
    // LogComponentEnable ("MyRrMacScheduler", LOG_LEVEL_INFO);

    uint16_t m_nUser = 5;
    // although this speed is meaningless in reality, it has to be large to make CQI change
    double speed = 10000;

    string datarate = "20Mbps";
    uint32_t packetSize = 1200;

    // Set the simulation time
    double simTime = 3.0;

    // Command line arguments
    CommandLine cmd;
    cmd.AddValue("simTime", "Total duration of the simulation [s])", simTime);
    cmd.AddValue("datarate", "datarate", datarate);
    cmd.AddValue("packetSize", "packetSize", packetSize);
    cmd.AddValue("speed", "x-axis speed of moving UE", speed);
    cmd.Parse(argc, argv);

    ConfigStore inputConfig;
    inputConfig.ConfigureDefaults();

    // parse again so you can override default values from the command line
    cmd.Parse(argc, argv);

    RngSeedManager::SetSeed(6);
    RngSeedManager::SetRun(4);

    Ptr<LteHelper> lteHelper = CreateObject<LteHelper>();
    Ptr<PointToPointEpcHelper> epcHelper = CreateObject<PointToPointEpcHelper>();
    lteHelper->SetEpcHelper(epcHelper);
    lteHelper->SetSchedulerType("ns3::MyRrMacScheduler");
    lteHelper->SetAttribute("PathlossModel", StringValue("ns3::FriisSpectrumPropagationLossModel"));

    Ptr<Node> pgw = epcHelper->GetPgwNode();

    // Create a single RemoteHost
    NodeContainer remoteHostContainer;
    remoteHostContainer.Create(1);
    Ptr<Node> remoteHost = remoteHostContainer.Get(0);
    InternetStackHelper internet;
    internet.Install(remoteHostContainer);

    // Create the Internet
    PointToPointHelper p2ph;
    p2ph.SetDeviceAttribute("DataRate", DataRateValue(DataRate("100Gb/s")));
    p2ph.SetDeviceAttribute("Mtu", UintegerValue(1500));
    p2ph.SetChannelAttribute("Delay", TimeValue(MilliSeconds(10)));
    NetDeviceContainer internetDevices = p2ph.Install(pgw, remoteHost);
    Ipv4AddressHelper ipv4h;
    ipv4h.SetBase("1.0.0.0", "255.0.0.0");
    Ipv4InterfaceContainer internetIpIfaces = ipv4h.Assign(internetDevices);
    // interface 0 is localhost, 1 is the p2p device
    // Ipv4Address remoteHostAddr = internetIpIfaces.GetAddress (1);

    Ipv4StaticRoutingHelper ipv4RoutingHelper;
    Ptr<Ipv4StaticRouting> remoteHostStaticRouting =
        ipv4RoutingHelper.GetStaticRouting(remoteHost->GetObject<Ipv4>());
    remoteHostStaticRouting->AddNetworkRouteTo(Ipv4Address("7.0.0.0"), Ipv4Mask("255.0.0.0"), 1);

    NodeContainer ueNodes;
    NodeContainer enbNodes;
    enbNodes.Create(1);
    ueNodes.Create(m_nUser);

    // Install Mobility Model
    Ptr<ListPositionAllocator> enbPositionAlloc = CreateObject<ListPositionAllocator>();
    Ptr<ListPositionAllocator> uePositionAlloc = CreateObject<ListPositionAllocator>();
    enbPositionAlloc->Add(Vector(0, 0, 25));
    uePositionAlloc->Add(Vector(-15000, 8000, 1.5));   // the moving ue
    uePositionAlloc->Add(Vector(10000, 10000, 1.5));   // stable ue #1
    uePositionAlloc->Add(Vector(-10000, 10000, 1.5));  // stable ue #2
    uePositionAlloc->Add(Vector(-10000, -10000, 1.5)); // stable ue #3
    uePositionAlloc->Add(Vector(10000, -10000, 1.5));  // stable ue #4
    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.SetPositionAllocator(enbPositionAlloc);
    mobility.Install(enbNodes);
    mobility.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
    mobility.SetPositionAllocator(uePositionAlloc);
    mobility.Install(ueNodes);

    // set speed for the moving ue
    Vector sp(speed, 0, 0);
    ueNodes.Get(0)->GetObject<ConstantVelocityMobilityModel>()->SetVelocity(sp);

    // Install LTE Devices to the nodes
    NetDeviceContainer enbLteDevs = lteHelper->InstallEnbDevice(enbNodes);
    NetDeviceContainer ueLteDevs = lteHelper->InstallUeDevice(ueNodes);

    Ptr<LteEnbNetDevice> lteEnbDev = enbLteDevs.Get(0)->GetObject<LteEnbNetDevice>();
    Ptr<LteEnbPhy> enbPhy = lteEnbDev->GetPhy();
    enbPhy->SetAttribute("TxPower", DoubleValue(30.0));
    enbPhy->SetAttribute("NoiseFigure", DoubleValue(5.0));

    // Install the IP stack on the UEs
    internet.Install(ueNodes);
    Ipv4InterfaceContainer ueIpIface;
    ueIpIface = epcHelper->AssignUeIpv4Address(NetDeviceContainer(ueLteDevs));
    // Assign IP address to UEs, and install applications
    for (uint32_t u = 0; u < ueNodes.GetN(); ++u)
    {
        Ptr<Node> ueNode = ueNodes.Get(u);
        // Set the default gateway for the UE
        Ptr<Ipv4StaticRouting> ueStaticRouting =
            ipv4RoutingHelper.GetStaticRouting(ueNode->GetObject<Ipv4>());
        ueStaticRouting->SetDefaultRoute(epcHelper->GetUeDefaultGatewayAddress(), 1);
    }

    enum EpsBearer::Qci q = EpsBearer::GBR_CONV_VOICE;
    EpsBearer bearer(q);

    // Attach the UEs to the eNB
    for (uint16_t i = 0; i < m_nUser; i++)
    {
        lteHelper->Attach(ueLteDevs.Get(i), enbLteDevs.Get(0));
        // side effect: the default EPS bearer will be activated
    }

    // The agent learns the CQI of the moving UE (ueNodes 0). Its RNTI is assigned in
    // random-access order, so hand it to the scheduler once its RRC connection is up.
    Ptr<MyRrMacScheduler> sched =
        DynamicCast<MyRrMacScheduler>(lteEnbDev->GetCcMap().at(0)->GetFfMacScheduler());
    NS_ABORT_MSG_IF(!sched, "expected ns3::MyRrMacScheduler on the eNB");
    const uint64_t movingImsi = ueLteDevs.Get(0)->GetObject<LteUeNetDevice>()->GetImsi();
    uint16_t movingRnti = 0;
    ueLteDevs.Get(0)->GetObject<LteUeNetDevice>()->GetRrc()->TraceConnectWithoutContext(
        "ConnectionEstablished",
        MakeCallback(
            +[](Ptr<MyRrMacScheduler> s, uint64_t want, uint16_t* out, uint64_t imsi, uint16_t,
                uint16_t rnti) {
                if (imsi == want)
                {
                    *out = rnti;
                    s->SetAttribute("AgentRnti", UintegerValue(rnti));
                }
            })
            .Bind(sched, movingImsi, &movingRnti));

    Time udpInterval =
        Time::FromDouble((packetSize * 8) / static_cast<double>(DataRate(datarate).GetBitRate()),
                         Time::S);

    // Install and start applications on UEs and remote host
    uint16_t dlPort = 1234;
    ApplicationContainer clientApps;
    ApplicationContainer serverApps;
    for (uint32_t u = 0; u < ueNodes.GetN(); ++u)
    {
        dlPort++;
        PacketSinkHelper dlPacketSinkHelper("ns3::UdpSocketFactory",
                                            InetSocketAddress(Ipv4Address::GetAny(), dlPort));

        serverApps.Add(dlPacketSinkHelper.Install(ueNodes.Get(u)));

        UdpClientHelper dlClient(ueIpIface.GetAddress(u), dlPort);
        dlClient.SetAttribute("PacketSize", UintegerValue(packetSize));
        dlClient.SetAttribute("Interval", TimeValue(udpInterval));
        dlClient.SetAttribute("MaxPackets", UintegerValue(0xFFFFFFFF));

        clientApps.Add(dlClient.Install(remoteHost));
    }

    serverApps.Start(MilliSeconds(10));
    clientApps.Start(MilliSeconds(10));
    lteHelper->EnableTraces();
    // Uncomment to enable PCAP tracing
    // p2ph.EnablePcapAll("lena-simple-epc");

    FlowMonitorHelper flowmon;
    Ptr<FlowMonitor> monitor = flowmon.InstallAll();
    // flowmon.Install (enbNodes.Get (0));

    Ptr<RadioBearerStatsCalculator> rlcStats = lteHelper->GetRlcStats();
    rlcStats->SetAttribute("EpochDuration", TimeValue(Seconds(simTime)));

    Simulator::Stop(Seconds(simTime));
    // NTN context: the LTE cell is a satellite-backhauled rural eNB at a real ground site (the
    // EPC backhaul stands for the satellite link). The eNB and UEs keep their local positions,
    // drawn as east-north-up offsets about the site; one device-less node carries the
    // backhaul satellite of an SGP4 Walker shell, so the LTE results do not change.
    const ntnwifisite::SiteContext ctx = ntnwifisite::AddBackhaulSatelliteContext({});
    if (ctx.satNode)
    {
        std::cout << "Backhaul satellite context: elevation " << ctx.elevationDeg
                  << " deg, slant range " << ctx.slantRangeKm << " km (" << ctx.propagator
                  << ")" << std::endl;
    }
    ns3::ntnobs::StartSceneFromNodeList(".");
    Simulator::Run();

    /*GtkConfigStore config;
    config.ConfigureAttributes();*/

    cout << "Begin Simulation Results" << endl;

    monitor->CheckForLostPackets();

    Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier>(flowmon.GetClassifier());
    map<FlowId, FlowMonitor::FlowStats> stats = monitor->GetFlowStats();

    double Throughput = 0.0;
    // ntn-dataset@1: the per-flow results as a dataset, so they follow the parameters
    // (UE speed, data rate, packet size).
    std::ofstream flowCsv("lte-cqi-flows.csv");
    flowCsv << "flow_id,src,dst,tx_packets,rx_packets,lost_packets,throughput_kbps,mean_delay_ms\n";
    uint64_t txSum = 0;
    uint64_t rxSum = 0;
    double thpSum = 0.0;
    double delaySumS = 0.0;

    for (map<FlowId, FlowMonitor::FlowStats>::const_iterator i = stats.begin(); i != stats.end();
         ++i)
    {
        Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow(i->first);

        cout << "Flow ID: " << i->first << " Src Addr " << t.sourceAddress << " Dst Addr "
             << t.destinationAddress << endl;
        cout << "Tx Packets = " << i->second.txPackets << endl;
        cout << "Rx Packets = " << i->second.rxPackets << endl;
        Throughput =
            i->second.rxBytes * 8.0 /
            (i->second.timeLastRxPacket.GetSeconds() - i->second.timeFirstTxPacket.GetSeconds()) /
            1024;
        cout << "Throughput: " << Throughput << " Kbps" << endl;
        const double meanDelayMs =
            i->second.rxPackets > 0 ? 1e3 * i->second.delaySum.GetSeconds() / i->second.rxPackets
                                    : 0.0;
        flowCsv << i->first << "," << t.sourceAddress << "," << t.destinationAddress << ","
                << i->second.txPackets << "," << i->second.rxPackets << ","
                << (i->second.txPackets - std::min(i->second.txPackets, i->second.rxPackets))
                << "," << (i->second.rxPackets > 0 ? std::to_string(Throughput) : std::string("0"))
                << "," << (i->second.rxPackets > 0 ? std::to_string(meanDelayMs) : std::string(""))
                << "\n";
        txSum += i->second.txPackets;
        rxSum += i->second.rxPackets;
        thpSum += i->second.rxPackets > 0 ? Throughput : 0.0;
        delaySumS += i->second.delaySum.GetSeconds();
    }
    flowCsv.close();

    // Application-level cross-check of the FlowMonitor counts: bytes the UE PacketSinks got.
    uint64_t sinkRxBytes = 0;
    for (uint32_t u = 0; u < serverApps.GetN(); ++u)
    {
        sinkRxBytes += DynamicCast<PacketSink>(serverApps.Get(u))->GetTotalRx();
    }
    cout << "PacketSink total Rx: " << sinkRxBytes << " bytes" << endl;
    cout << "Agent UE: IMSI " << movingImsi << " RNTI " << movingRnti << ", "
         << sched->GetAgentCqiReports() << " CQI reports, CQI "
         << unsigned(sched->GetAgentCqiMin()) << " to " << unsigned(sched->GetAgentCqiMax())
         << endl;

    NS_LOG_UNCOND("Done");

    cout << "End Simulation Results" << endl;

    // ntn-dataset@1: standard manifest + sim_health (plan/MASTER_PLAN_GLOBE_DATASET_E2E_2026-11.md)
    ns3::ntnobs::WriteDatasetRecord(
        ".",
        std::filesystem::path(argv[0]).filename().string(),
        "measured",
        argc,
        argv,
        {{"flows", std::to_string(stats.size()), "measured"},
         {"agent_ue_rnti", std::to_string(movingRnti), "measured"},
         {"agent_cqi_reports", std::to_string(sched->GetAgentCqiReports()), "measured"},
         {"agent_cqi_min", std::to_string(sched->GetAgentCqiMin()), "measured"},
         {"agent_cqi_max", std::to_string(sched->GetAgentCqiMax()), "measured"},
         {"tx_packets", std::to_string(txSum), "measured"},
         {"rx_packets", std::to_string(rxSum), "measured"},
         {"delivery_ratio", std::to_string(txSum > 0 ? double(rxSum) / txSum : 0.0), "measured"},
         {"throughput_kbps_sum", std::to_string(thpSum), "measured"},
         {"mean_delay_ms", std::to_string(rxSum > 0 ? 1e3 * delaySumS / rxSum : 0.0), "measured"},
         {"sink_rx_bytes", std::to_string(sinkRxBytes), "measured"}});
    Simulator::Destroy();
    return 0;
}
