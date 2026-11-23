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
 *
 * Wave 7A: the two-node point-to-point link is now the `--radio=p2p`
 * reference. The default `--radio=nr` streams the same NRTV session from the
 * EPC remote host to a TR 38.811 UE (ntn-cho) through a 5G-LENA NR-NTN cell
 * whose gNB rides an SGP4-propagated LEO satellite (ntn-constellation), with
 * the TR 38.821 Set-1 S-band link budget, a WGS-84 feeder gateway, the EPC
 * (GTP-U), PDCP, RLC, MAC/HARQ and the SINR-driven PHY error model
 * (NtnRealStackHelper). Slice delay and jitter are measured in band from the
 * NrtvHeader send timestamp for both TCP (NrtvTcpClient traces) and UDP (the
 * header of every received datagram); UDP loss is server Tx minus client Rx.
 * ue_kpi.csv, cell_kpi.csv, sim_health.csv and nrtv-qoe.csv go to --outputDir.
 */

#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/network-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/ntn-traffic-module.h"
#include "ns3/mobility-module.h"
#include "ns3/ntn-tr38811-mobility-model.h"
#include "ns3/sgp4-mobility-model.h"
#include "ns3/walker-constellation.h"

#include <filesystem>
#include <fstream>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("NrtvP2pExample");

// ---- Application-layer QoE sinks for the NrtvTcpClient trace sources. ----
// These connect the previously-unconnected RxDelay/RxJitter/RxSlice/RxFrame
// traces to a CSV file and to run-level accumulators. All values are MEASURED
// by the live TCP NRTV session; no formulas are involved.
namespace
{
std::ofstream g_qoeCsv;
uint64_t g_sliceCount = 0;
uint64_t g_frameCount = 0;
uint64_t g_delaySamples = 0;
double g_delaySumMs = 0.0;
uint64_t g_jitterSamples = 0;
double g_jitterSumMs = 0.0;

void
QoeRxDelay(const Time& delay, const Address&)
{
    g_delaySumMs += delay.GetSeconds() * 1000.0;
    ++g_delaySamples;
    if (g_qoeCsv.is_open())
    {
        g_qoeCsv << Simulator::Now().GetSeconds() << ",delay," << delay.GetSeconds() * 1000.0
                 << ",\n"; // empty extra: four fields per row
    }
}

void
QoeRxJitter(const Time& jitter, const Address&)
{
    g_jitterSumMs += jitter.GetSeconds() * 1000.0;
    ++g_jitterSamples;
    if (g_qoeCsv.is_open())
    {
        g_qoeCsv << Simulator::Now().GetSeconds() << ",jitter," << jitter.GetSeconds() * 1000.0
                 << ",\n"; // empty extra: four fields per row
    }
}

void
QoeRxSlice(Ptr<const Packet> slice)
{
    ++g_sliceCount;
    if (g_qoeCsv.is_open())
    {
        g_qoeCsv << Simulator::Now().GetSeconds() << ",slice," << slice->GetSize() << ",\n"; // empty extra: four fields per row
    }
}

// Wave 7A, UDP path: every NRTV datagram starts with an NrtvHeader that
// carries the server-side send time, so delay and jitter are measured in band
// from the received bytes (the same quantities NrtvTcpClient reports for TCP).
uint64_t g_udpRxPackets = 0;
bool g_udpHaveLast = false;
Time g_udpLastDelay;

void
QoeUdpRx(Ptr<const Packet> p, const Address& from)
{
    ++g_udpRxPackets;
    NrtvHeader h;
    if (p->GetSize() < h.GetSerializedSize())
    {
        return;
    }
    p->PeekHeader(h);
    const Time delay = Simulator::Now() - h.GetArrivalTime();
    QoeRxDelay(delay, from);
    if (g_udpHaveLast)
    {
        QoeRxJitter(Abs(delay - g_udpLastDelay), from);
    }
    g_udpHaveLast = true;
    g_udpLastDelay = delay;
}

uint64_t g_serverTxPackets = 0;

void
CountServerTx(Ptr<const Packet>)
{
    ++g_serverTxPackets;
}

void
QoeRxFrame(uint32_t frameNumber, uint32_t numOfFrames)
{
    ++g_frameCount;
    if (g_qoeCsv.is_open())
    {
        g_qoeCsv << Simulator::Now().GetSeconds() << ",frame," << frameNumber << ","
                 << numOfFrames << std::endl;
    }
}
} // anonymous namespace

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
    std::string radio = "nr";
    double altitudeKm = 600.0;
    double ueNorthOffsetKm = 300.0;
    double gwDistanceKm = 500.0;
    std::string outputDir = "nrtv-p2p-example-output";
    cmd.AddValue("radio",
                 "nr (default: SGP4 satellite gNB, NR-NTN radio, EPC) or p2p (the original "
                 "5 Mb/s, 2 ms point-to-point link)",
                 radio);
    cmd.AddValue("altitudeKm", "nr: satellite altitude (km)", altitudeKm);
    cmd.AddValue("ueNorthOffsetKm",
                 "nr: UE ground offset north of the t=0 sub-satellite point (km)",
                 ueNorthOffsetKm);
    cmd.AddValue("gwDistanceKm", "nr: feeder gateway ground distance north of the UE (km)",
                 gwDistanceKm);
    cmd.AddValue("outputDir",
                 "nr: directory for ue_kpi.csv, cell_kpi.csv, sim_health.csv and nrtv-qoe.csv",
                 outputDir);
    cmd.Parse(argc, argv);
    NS_ABORT_MSG_IF(radio != "nr" && radio != "p2p", "--radio must be nr or p2p");
    const bool useNr = (radio == "nr");

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

    // node 1 = video server, node 0 = client (p2p); remote host / UE (nr).
    Ptr<Node> serverNode;
    Ptr<Node> clientNode;
    NodeContainer nodes;
    NtnRealStackHelper rs;
    NodeContainer gnbNodes;
    NodeContainer ueNodes;
    Ptr<ns3::ntncon::Sgp4MobilityModel> satMob;
    if (!useNr)
    {
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
        serverNode = nodes.Get(1);
        clientNode = nodes.Get(0);
    }
    else
    {
        std::error_code ec;
        std::filesystem::create_directories(outputDir, ec);
        // SGP4 LEO satellite (ntn-constellation), slot 0 of a 53 deg Walker plane.
        ns3::ntncon::WalkerConfig wcfg;
        wcfg.num_planes = 1;
        wcfg.total_sats = 80;
        wcfg.altitude_km = altitudeKm;
        wcfg.inclination_deg = 53.0;
        wcfg.epoch_unix_s = 1735689600.0; // 2025-01-01T00:00:00Z
        const auto elements = ns3::ntncon::WalkerConstellation::BuildDelta(wcfg);
        gnbNodes.Create(1);
        satMob = CreateObject<ns3::ntncon::Sgp4MobilityModel>();
        satMob->SetElementsSgp4(elements[0]);
        gnbNodes.Get(0)->AggregateObject(satMob);
        double subLat;
        double subLon;
        double subAlt;
        satMob->GetGeodetic(subLat, subLon, subAlt);
        // TR 38.811 UE class (ntn-cho).
        const double ueLat = subLat + ueNorthOffsetKm / 111.195;
        ueNodes.Create(1);
        NtnTr38811MobilityHelper ueMobility(1);
        ueMobility.Install(ueNodes,
                           NtnMobilityScenarios::MixedContinental(),
                           ueLat - 0.03,
                           ueLat + 0.03,
                           subLon - 0.03,
                           subLon + 0.03);
        // NR-NTN cell + EPC, TR 38.821 Set-1 S-band link budget (helper default).
        rs.SetRadioBackend(NtnRealStackHelper::RadioBackend::Nr);
        rs.SetNumerology(1);
        rs.SetSimTime(Seconds(simTime));
        rs.SetOutputDir(outputDir);
        rs.SetRunTag("nrtv-p2p-example");
        rs.Build(gnbNodes, ueNodes);
        if (gwDistanceKm > 0.0)
        {
            rs.SetFeederGatewayGeodetic(ueLat + gwDistanceKm / 111.195, subLon, 0.0);
        }
        serverNode = rs.GetRemoteHost();
        clientNode = ueNodes.Get(0);
    }

    /// Server & Client setup using NrtvHelper ///

    NrtvHelper nrtvHelper(TypeId::LookupByName(prss.str()));
    nrtvHelper.SetVariablesAttribute("NumberOfVideos",
                                     StringValue("ns3::UniformRandomVariable[Min=2|Max=5]"));
    nrtvHelper.InstallUsingIpv4(serverNode, clientNode);
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

    /// Connect the NrtvTcpClient QoE trace sources (TCP path only). For UDP the
    /// client is a PacketSink, not an NrtvTcpClient, so the DynamicCast returns
    /// nullptr and these traces never fire -> guard on a non-null cast. ///
    Ptr<NrtvTcpClient> tcpClient = DynamicCast<NrtvTcpClient>(clientApp);
    const std::string qoePath = useNr ? outputDir + "/nrtv-qoe.csv" : std::string("nrtv-qoe.csv");
    Ptr<Application> serverApp = nrtvHelper.GetServer().Get(0);
    serverApp->TraceConnectWithoutContext("Tx", MakeCallback(&CountServerTx));
    if (useNr)
    {
        // The helper accounts for the flow (ue_kpi.csv app columns, delivery
        // ratio for UDP where server Tx and client Rx count the same datagrams).
        rs.AttachExternalDlFlow(0,
                                tcpClient ? nullptr : serverApp,
                                clientApp,
                                Seconds(1.0),
                                Seconds(simTime));
        if (!tcpClient)
        {
            g_qoeCsv.open(qoePath, std::ios::out | std::ios::trunc);
            if (g_qoeCsv.is_open())
            {
                g_qoeCsv << "time_s,kind,value,extra" << std::endl;
            }
            clientApp->TraceConnectWithoutContext("Rx", MakeCallback(&QoeUdpRx));
        }
    }
    if (tcpClient)
    {
        g_qoeCsv.open(qoePath, std::ios::out | std::ios::trunc);
        if (g_qoeCsv.is_open())
        {
            g_qoeCsv << "time_s,kind,value,extra" << std::endl;
        }
        tcpClient->TraceConnectWithoutContext("RxDelay", MakeCallback(&QoeRxDelay));
        tcpClient->TraceConnectWithoutContext("RxJitter", MakeCallback(&QoeRxJitter));
        tcpClient->TraceConnectWithoutContext("RxSlice", MakeCallback(&QoeRxSlice));
        tcpClient->TraceConnectWithoutContext("RxFrame", MakeCallback(&QoeRxFrame));
    }

    Simulator::Stop(Seconds(simTime));
    Simulator::Run();

    if (useNr)
    {
        rs.Collect();
        rs.WriteHealthReport();
        const double meanDelayMs = (g_delaySamples > 0) ? g_delaySumMs / g_delaySamples : 0.0;
        const double meanJitterMs = (g_jitterSamples > 0) ? g_jitterSumMs / g_jitterSamples : 0.0;
        const Vector ue = clientNode->GetObject<MobilityModel>()->GetPosition();
        const Vector sat = satMob->GetPosition();
        std::cout << "==== nrtv-p2p-example MEASURED KPIs (radio=nr, " << protocol
                  << ", SGP4 + NR-NTN + EPC) ====" << std::endl;
        std::cout << "nrtv_server_tx_packets = " << g_serverTxPackets << std::endl;
        if (!tcpClient)
        {
            const double loss =
                g_serverTxPackets > 0
                    ? 1.0 - static_cast<double>(g_udpRxPackets) / g_serverTxPackets
                    : 0.0;
            std::cout << "nrtv_udp_rx_packets    = " << g_udpRxPackets << std::endl;
            std::cout << "nrtv_udp_loss_ratio    = " << loss << std::endl;
        }
        std::cout << "nrtv_mean_delay_ms     = " << meanDelayMs << std::endl;
        std::cout << "nrtv_mean_jitter_ms    = " << meanJitterMs << std::endl;
        std::cout << "dl_throughput_mbps     = " << rs.GetRxThroughputMbps() << std::endl;
        std::cout << "dl_sinr_db             = " << rs.GetMeanDlSinrDb() << std::endl;
        std::cout << "dl_tbler               = " << rs.GetMeanDlTbler() << std::endl;
        std::cout << "ue elevation end (deg) = " << ntngeo::ElevationDeg(ue, sat) << std::endl;
        std::cout << "outputs in " << outputDir << std::endl;
        std::cout << "=============================================" << std::endl;
        if (g_qoeCsv.is_open())
        {
            g_qoeCsv.close();
        }
    }

    if (tcpClient && !useNr)
    {
        if (g_qoeCsv.is_open())
        {
            g_qoeCsv.close();
        }
        double meanDelayMs = (g_delaySamples > 0) ? g_delaySumMs / g_delaySamples : 0.0;
        double meanJitterMs = (g_jitterSamples > 0) ? g_jitterSumMs / g_jitterSamples : 0.0;
        std::cout << "==== nrtv-p2p-example MEASURED QoE (TCP) ====" << std::endl;
        std::cout << "nrtv_qoe_slice_count   = " << g_sliceCount << std::endl;
        std::cout << "nrtv_qoe_frame_count   = " << g_frameCount << std::endl;
        std::cout << "nrtv_qoe_mean_delay_ms = " << meanDelayMs << std::endl;
        std::cout << "nrtv_qoe_mean_jitter_ms= " << meanJitterMs << std::endl;
        std::cout << "QoE trace written to nrtv-qoe.csv" << std::endl;
        std::cout << "=============================================" << std::endl;
    }

    Simulator::Destroy();

    return 0;

} // end of `int main (int argc, char *argv[])`
