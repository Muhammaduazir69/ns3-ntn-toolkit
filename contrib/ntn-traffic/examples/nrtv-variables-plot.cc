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

/**
 * @file
 *
 * @ingroup nrtv
 * @brief Example script for plotting histograms from some of the random
 *        variable distributions used in NRTV traffic model.
 *
 * The script repeatedly draws random samples from the distributions and then
 * plot a histogram for each distribution. By default, 100 000 samples are
 * taken, which can be modified through a command line argument, for example:
 *
 *     $ ./waf --run="nrtv-variables-plot --numOfSamples=1000000"
 *
 * The script generates the following files in the ns-3 project root directory:
 * - `nrtv-slice-size.plt`
 * - `nrtv-slice-encoding-delay.plt`
 *
 * These files are Gnuplot files. Each of these files can be converted to a PNG
 * file, for example by this command:
 *
 *     $ gnuplot nrtv-slice-size.plt
 *
 * which will produce `slice-size.png` file in the same directory. To
 * convert all the Gnuplot files in the directory, the command below can be
 * used:
 *
 *     $ gnuplot *.plt
 *
 */

#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/network-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/stats-module.h"
#include "ns3/ntn-traffic-module.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("NrtvVariablesPlot");

// ---- Live-plane QoE sinks. The histogram blocks below sample the 3GPP NRTV
// distributions (legitimate provenance). The real-plane section additionally
// runs a TCP NRTV session and connects these MEASURED QoE traces. ----
namespace
{
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
}

void
QoeRxJitter(const Time& jitter, const Address&)
{
    g_jitterSumMs += jitter.GetSeconds() * 1000.0;
    ++g_jitterSamples;
}

void
QoeRxSlice(Ptr<const Packet>)
{
    ++g_sliceCount;
}

void
QoeRxFrame(uint32_t, uint32_t)
{
    ++g_frameCount;
}
} // anonymous namespace

int
main(int argc, char* argv[])
{
    uint32_t numOfSamples = 100000;
    double simTime = 20.0;
    bool runPlane = true;

    // read command line arguments given by the user
    CommandLine cmd;
    cmd.AddValue("numOfSamples",
                 "Number of samples taken from each random number distribution",
                 numOfSamples);
    cmd.AddValue("simTime", "Real-plane NRTV session duration in seconds", simTime);
    cmd.AddValue("runPlane",
                 "If true, run a real TCP NRTV data plane and report measured QoE",
                 runPlane);
    cmd.Parse(argc, argv);

    Ptr<NrtvVariables> nrtvVariables = CreateObject<NrtvVariables>();
    // nrtvVariables->SetStream (99);

    HistogramPlotHelper::Plot<uint32_t>(MakeCallback(&NrtvVariables::GetNumOfFrames, nrtvVariables),
                                        "nrtv-num-of-frames",
                                        "Histogram of number of frames in NRTV traffic model",
                                        "Number of frames",
                                        numOfSamples,
                                        100, // bin width = 100 frames
                                        static_cast<double>(nrtvVariables->GetNumOfFramesMean()));

    HistogramPlotHelper::Plot<uint32_t>(MakeCallback(&NrtvVariables::GetSliceSize, nrtvVariables),
                                        "nrtv-slice-size",
                                        "Histogram of slice size in NRTV traffic model",
                                        "Slice size (in bytes)",
                                        numOfSamples,
                                        5, // bin width = 5 bytes
                                        nrtvVariables->GetSliceSizeMean(),
                                        nrtvVariables->GetSliceSizeMax());

    HistogramPlotHelper::Plot<uint64_t>(
        MakeCallback(&NrtvVariables::GetSliceEncodingDelayMilliSeconds, nrtvVariables),
        "nrtv-slice-encoding-delay",
        "Histogram of slice encoding delay in NRTV traffic model",
        "Slice encoding delay (in milliseconds)",
        numOfSamples,
        1, // bin width = 1 ms
        nrtvVariables->GetSliceEncodingDelayMean().GetMilliSeconds(),
        nrtvVariables->GetSliceEncodingDelayMax().GetMilliSeconds());

    HistogramPlotHelper::Plot<double>(
        MakeCallback(&NrtvVariables::GetIdleTimeSeconds, nrtvVariables),
        "nrtv-idle-time",
        "Histogram of client idle time in NRTV traffic model",
        "Idle time (in seconds)",
        numOfSamples,
        1, // bar width = 1 second
        nrtvVariables->GetIdleTimeMean().GetSeconds());

    // ---- Real-plane section: run a live TCP NRTV session over a P2P link and
    // report MEASURED QoE from the NrtvTcpClient trace sources, alongside the
    // distribution means already plotted above. ----
    if (runPlane)
    {
        NodeContainer nodes;
        nodes.Create(2);

        PointToPointHelper pointToPoint;
        pointToPoint.SetDeviceAttribute("DataRate", StringValue("5Mbps"));
        pointToPoint.SetChannelAttribute("Delay", StringValue("2ms"));

        NetDeviceContainer devices = pointToPoint.Install(nodes);

        InternetStackHelper stack;
        stack.Install(nodes);

        Ipv4AddressHelper address;
        address.SetBase("10.1.1.0", "255.255.255.0");
        Ipv4InterfaceContainer interfaces = address.Assign(devices);

        NrtvHelper nrtvHelper(TypeId::LookupByName("ns3::TcpSocketFactory"));
        nrtvHelper.SetVariablesAttribute("NumberOfVideos",
                                         StringValue("ns3::UniformRandomVariable[Min=2|Max=5]"));
        nrtvHelper.InstallUsingIpv4(nodes.Get(1), nodes.Get(0));
        nrtvHelper.GetServer().Start(Seconds(2.0));
        nrtvHelper.GetClients().Start(Seconds(1.0));

        Ptr<NrtvTcpClient> tcpClient =
            DynamicCast<NrtvTcpClient>(nrtvHelper.GetClients().Get(0));
        if (tcpClient)
        {
            tcpClient->TraceConnectWithoutContext("RxDelay", MakeCallback(&QoeRxDelay));
            tcpClient->TraceConnectWithoutContext("RxJitter", MakeCallback(&QoeRxJitter));
            tcpClient->TraceConnectWithoutContext("RxSlice", MakeCallback(&QoeRxSlice));
            tcpClient->TraceConnectWithoutContext("RxFrame", MakeCallback(&QoeRxFrame));
        }

        Simulator::Stop(Seconds(simTime));
        Simulator::Run();
        Simulator::Destroy();

        double meanDelayMs = (g_delaySamples > 0) ? g_delaySumMs / g_delaySamples : 0.0;
        double meanJitterMs = (g_jitterSamples > 0) ? g_jitterSumMs / g_jitterSamples : 0.0;

        std::cout << "==== nrtv-variables-plot real-plane MEASURED QoE (TCP) ====" << std::endl;
        std::cout << "distribution_num_of_frames_mean = "
                  << nrtvVariables->GetNumOfFramesMean() << std::endl;
        std::cout << "distribution_slice_size_mean    = "
                  << nrtvVariables->GetSliceSizeMean() << std::endl;
        std::cout << "nrtv_qoe_slice_count            = " << g_sliceCount << std::endl;
        std::cout << "nrtv_qoe_frame_count            = " << g_frameCount << std::endl;
        std::cout << "nrtv_qoe_mean_delay_ms          = " << meanDelayMs << std::endl;
        std::cout << "nrtv_qoe_mean_jitter_ms         = " << meanJitterMs << std::endl;
        std::cout << "===========================================================" << std::endl;
    }

    return 0;

} // end of `int main (int argc, char *argv[])`
