/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * Author: Muhammad Uzair, Independent Researcher
 */

/**
 * @file
 *
 * @ingroup nrtv
 * @brief Drives the module's own CbrApplication / CbrHelper over a
 *        satellite-representative point-to-point link (a LEO-600 gNB <-> ground
 *        UE feeder/access hop). The link carries a RateErrorModel on the
 *        receive NetDevice so that packet loss is produced by a real ns-3
 *        error model, not a formula. Every KPI reported below is MEASURED:
 *
 *        - cbr_sent_bytes     : CbrApplication::GetSent() on the sender
 *        - cbr_rx_bytes       : PacketSink::GetTotalRx() on the receiver
 *        - cbr_loss_ratio     : 1 - (rx_bytes / sent_bytes)
 *        - throughput / delay / jitter : from FlowMonitor over the live flow
 *
 *        The one-way propagation delay defaults to ~12.9 ms which gives an
 *        end-to-end RTD of ~25.77 ms, the LEO-600 round-trip-delay reference
 *        in 3GPP TR 38.821 (NTN reference scenarios).
 *
 *        Wave 7A: that fixed link is now the `--radio=p2p` reference only. The
 *        default (`--radio=nr`) carries the same CbrApplication flow end to end
 *        over the toolkit: an SGP4-propagated LEO satellite (ntn-constellation)
 *        is the gNB of a 5G-LENA NR-NTN cell (NtnRealStackHelper, TR 38.821
 *        Set-1 S-band link budget, n256 at 2.185 GHz, 30 kHz SCS), the UE is a
 *        TR 38.811 UE class (ntn-cho), the feeder runs to a WGS-84 gateway, and
 *        the CBR packets cross the EPC (GTP-U), PDCP, RLC, MAC/HARQ and the
 *        SINR-driven PHY error model. Delay, RFC 3393 IPDV and loss are measured
 *        in band from an ns-3 SeqTsSizeHeader inside every packet; FlowMonitor
 *        runs alongside; ue_kpi.csv / cell_kpi.csv and sim_health.csv go to
 *        --outputDir. --delayMs, --dataRateMbps and --errorRate configure the
 *        p2p reference link; in nr mode delay, rate and loss come from the
 *        orbit and the radio.
 */

#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/error-model.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/internet-module.h"
#include "ns3/network-module.h"
#include "ns3/ntn-traffic-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/mobility-module.h"
#include "ns3/ntn-tr38811-mobility-model.h"
#include "ns3/seq-ts-size-header.h"
#include "ns3/sgp4-mobility-model.h"
#include "ns3/walker-constellation.h"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("NtnCbrLeoLink");

namespace
{
/// In-band per-packet measurement from the SeqTsSizeHeader each CBR packet
/// carries: one-way delay, RFC 3393 IPDV between consecutive sequence numbers,
/// and sequence-gap loss.
struct InbandStats
{
    uint64_t rx{0};
    uint64_t rxBytes{0};
    uint64_t highestSeq{0};
    bool any{false};
    double sumDelayMs{0.0};
    double maxDelayMs{0.0};
    double minDelayMs{std::numeric_limits<double>::infinity()};
    uint64_t ipdvN{0};
    double ipdvSumAbsMs{0.0};
    uint64_t lastSeq{0};
    double lastDelayMs{0.0};
    std::ofstream* trace{nullptr};

    void Rx(Ptr<const Packet> p, const Address&, const Address&, const SeqTsSizeHeader& h)
    {
        const double d = (Simulator::Now() - h.GetTs()).GetSeconds() * 1e3;
        ++rx;
        rxBytes += p->GetSize() + h.GetSerializedSize();
        sumDelayMs += d;
        maxDelayMs = std::max(maxDelayMs, d);
        minDelayMs = std::min(minDelayMs, d);
        if (any && h.GetSeq() == lastSeq + 1)
        {
            ipdvSumAbsMs += std::fabs(d - lastDelayMs);
            ++ipdvN;
        }
        highestSeq = any ? std::max<uint64_t>(highestSeq, h.GetSeq()) : h.GetSeq();
        any = true;
        lastSeq = h.GetSeq();
        lastDelayMs = d;
        if (trace)
        {
            *trace << Simulator::Now().GetSeconds() << "," << h.GetSeq() << "," << d << "\n";
        }
    }
};

/// The p2p reference path (the example as it was before Wave 7A).
int RunP2p(double simTime,
           double intervalMs,
           uint32_t pktSize,
           double errorRate,
           double delayMs,
           double dataRateMbps);

/// Wave 7A: the same CBR flow over SGP4 orbit + NR-NTN radio + EPC.
int RunNr(double simTime,
          double intervalMs,
          uint32_t pktSize,
          uint32_t numUes,
          double altitudeKm,
          double ueNorthOffsetKm,
          double gwDistanceKm,
          const std::string& outputDir);
} // namespace

int
main(int argc, char* argv[])
{
    double simTime = 10.0;     // seconds
    double intervalMs = 5.0;   // CBR inter-packet interval (ms)
    uint32_t pktSize = 1024;   // CBR payload bytes
    double errorRate = 0.01;   // per-packet loss probability on the rx device
    double delayMs = 12.885;   // one-way prop delay -> ~25.77 ms RTD (LEO-600, TR 38.821)
    double dataRateMbps = 50.0;

    CommandLine cmd(__FILE__);
    cmd.AddValue("simTime", "Simulation time in seconds", simTime);
    cmd.AddValue("intervalMs", "CBR inter-packet interval in milliseconds", intervalMs);
    cmd.AddValue("pktSize", "CBR payload size in bytes", pktSize);
    cmd.AddValue("errorRate", "Per-packet error rate on the receive NetDevice", errorRate);
    cmd.AddValue("delayMs", "One-way link propagation delay in milliseconds", delayMs);
    cmd.AddValue("dataRateMbps", "Link data rate in Mbps", dataRateMbps);
    std::string radio = "nr";
    uint32_t numUes = 1;
    double altitudeKm = 600.0;
    double ueNorthOffsetKm = 300.0;
    double gwDistanceKm = 500.0;
    std::string outputDir = "ntn-cbr-leo-link-output";
    cmd.AddValue("radio",
                 "nr (default: SGP4 satellite gNB, NR-NTN radio, EPC) or p2p (the fixed "
                 "LEO-600 reference link that --delayMs/--dataRateMbps/--errorRate configure)",
                 radio);
    cmd.AddValue("numUes", "nr: TR 38.811 UEs, one CBR flow each", numUes);
    cmd.AddValue("altitudeKm", "nr: satellite altitude (km)", altitudeKm);
    cmd.AddValue("ueNorthOffsetKm",
                 "nr: UE cluster ground offset north of the t=0 sub-satellite point (km); 300 km "
                 "puts the UE near 60 deg elevation at 600 km",
                 ueNorthOffsetKm);
    cmd.AddValue("gwDistanceKm", "nr: feeder gateway ground distance north of the UEs (km)",
                 gwDistanceKm);
    cmd.AddValue("outputDir", "nr: directory for ue_kpi.csv, cell_kpi.csv, sim_health.csv, "
                 "cbr_kpi.csv and cbr_owd.csv", outputDir);
    cmd.Parse(argc, argv);

    if (radio == "nr")
    {
        return RunNr(simTime, intervalMs, pktSize, numUes, altitudeKm, ueNorthOffsetKm,
                     gwDistanceKm, outputDir);
    }
    NS_ABORT_MSG_IF(radio != "p2p", "--radio must be nr or p2p");
    return RunP2p(simTime, intervalMs, pktSize, errorRate, delayMs, dataRateMbps);
}

namespace
{
int
RunP2p(double simTime,
       double intervalMs,
       uint32_t pktSize,
       double errorRate,
       double delayMs,
       double dataRateMbps)
{

    // node 0 = satellite gNB (sender), node 1 = ground UE (receiver)
    NodeContainer nodes;
    nodes.Create(2);

    PointToPointHelper p2p;
    std::ostringstream rateStr;
    rateStr << dataRateMbps << "Mbps";
    p2p.SetDeviceAttribute("DataRate", StringValue(rateStr.str()));
    std::ostringstream delayStr;
    delayStr << delayMs << "ms";
    p2p.SetChannelAttribute("Delay", StringValue(delayStr.str()));

    NetDeviceContainer devices = p2p.Install(nodes);

    // Real ns-3 error model on the UE (receive) device -> measured packet loss.
    Ptr<RateErrorModel> em = CreateObject<RateErrorModel>();
    em->SetUnit(RateErrorModel::ERROR_UNIT_PACKET);
    em->SetRate(errorRate);
    em->SetRandomVariable(CreateObject<UniformRandomVariable>());
    devices.Get(1)->SetAttribute("ReceiveErrorModel", PointerValue(em));

    InternetStackHelper stack;
    stack.Install(nodes);

    Ipv4AddressHelper address;
    address.SetBase("10.1.1.0", "255.255.255.0");
    Ipv4InterfaceContainer interfaces = address.Assign(devices);

    uint16_t port = 4000;
    Ipv4Address ueAddr = interfaces.GetAddress(1);

    // Receiver: PacketSink on the ground UE.
    PacketSinkHelper sinkHelper("ns3::UdpSocketFactory",
                                InetSocketAddress(Ipv4Address::GetAny(), port));
    ApplicationContainer sinkApps = sinkHelper.Install(nodes.Get(1));
    sinkApps.Start(Seconds(0.5));
    sinkApps.Stop(Seconds(simTime));

    // Sender: the module's own CbrApplication via CbrHelper.
    CbrHelper cbrHelper("ns3::UdpSocketFactory", InetSocketAddress(ueAddr, port));
    cbrHelper.SetConstantTraffic(MilliSeconds(intervalMs), pktSize);
    cbrHelper.SetAttribute("EnableStatisticsTags", BooleanValue(true));
    ApplicationContainer cbrApps = cbrHelper.Install(nodes.Get(0));
    cbrApps.Start(Seconds(1.0));
    cbrApps.Stop(Seconds(simTime));

    // FlowMonitor over the live data plane.
    FlowMonitorHelper flowmonHelper;
    Ptr<FlowMonitor> monitor = flowmonHelper.InstallAll();

    Simulator::Stop(Seconds(simTime + 0.5));
    Simulator::Run();

    // ---- MEASURED KPIs ----
    Ptr<CbrApplication> sender = DynamicCast<CbrApplication>(cbrApps.Get(0));
    Ptr<PacketSink> sink = DynamicCast<PacketSink>(sinkApps.Get(0));

    // CbrApplication::GetSent() and PacketSink::GetTotalRx() both return BYTES
    // (this is the invariant asserted in test/cbr-test.cc).
    uint64_t sentBytes = sender ? sender->GetSent() : 0;
    uint64_t rxBytes = sink ? sink->GetTotalRx() : 0;
    uint64_t sentPkts = (pktSize > 0) ? (sentBytes / pktSize) : 0;
    uint64_t rxPkts = (pktSize > 0) ? (rxBytes / pktSize) : 0;
    double lossRatio = (sentBytes > 0)
                           ? (1.0 - static_cast<double>(rxBytes) / static_cast<double>(sentBytes))
                           : 0.0;

    monitor->CheckForLostPackets();
    Ptr<Ipv4FlowClassifier> classifier =
        DynamicCast<Ipv4FlowClassifier>(flowmonHelper.GetClassifier());
    double fmThroughputMbps = 0.0;
    double fmMeanDelayMs = 0.0;
    double fmJitterMs = 0.0;
    uint64_t fmRxPackets = 0;
    uint64_t fmJitterPairs = 0;
    for (auto& kv : monitor->GetFlowStats())
    {
        const FlowMonitor::FlowStats& st = kv.second;
        if (st.rxPackets > 0)
        {
            double durSec =
                (st.timeLastRxPacket - st.timeFirstTxPacket).GetSeconds();
            if (durSec > 0)
            {
                fmThroughputMbps += st.rxBytes * 8.0 / durSec / 1e6;
            }
            // Time::GetMilliSeconds() truncates to an integer, which zeroed
            // any sub-millisecond jitter sum; take the double value instead.
            fmMeanDelayMs += st.delaySum.GetSeconds() * 1e3;
            fmJitterMs += st.jitterSum.GetSeconds() * 1e3;
            fmRxPackets += st.rxPackets;
            // FlowMonitor accumulates |D(i) - D(i-1)| (RFC 3393 IPDV) over
            // consecutive pairs, so the mean is over rxPackets - 1 samples.
            fmJitterPairs += st.rxPackets - 1;
        }
    }
    if (fmRxPackets > 0)
    {
        fmMeanDelayMs /= fmRxPackets;
    }
    if (fmJitterPairs > 0)
    {
        fmJitterMs /= fmJitterPairs;
    }

    std::cout << "==== ntn-cbr-leo-link MEASURED KPIs ====" << std::endl;
    std::cout << "cbr_sent_bytes         = " << sentBytes << std::endl;
    std::cout << "cbr_sent_packets       = " << sentPkts << std::endl;
    std::cout << "cbr_rx_bytes           = " << rxBytes << std::endl;
    std::cout << "cbr_rx_packets         = " << rxPkts << std::endl;
    std::cout << "cbr_loss_ratio         = " << lossRatio << std::endl;
    std::cout << "flowmon_throughput_Mbps= " << fmThroughputMbps << std::endl;
    std::cout << "flowmon_mean_delay_ms  = " << fmMeanDelayMs << std::endl;
    std::cout << "flowmon_jitter_ms      = " << fmJitterMs << std::endl;
    std::cout << "========================================" << std::endl;

    Simulator::Destroy();
    return 0;
}

int
RunNr(double simTime,
      double intervalMs,
      uint32_t pktSize,
      uint32_t numUes,
      double altitudeKm,
      double ueNorthOffsetKm,
      double gwDistanceKm,
      const std::string& outputDir)
{
    NS_ABORT_MSG_IF(numUes < 1, "--numUes must be >= 1");
    NS_ABORT_MSG_IF(simTime <= 1.5, "--simTime must exceed 1.5 s (traffic starts at 1 s)");
    std::error_code ec;
    std::filesystem::create_directories(outputDir, ec);

    // ---- SGP4 satellite (ntn-constellation): slot 0 of a 53 deg Walker plane.
    ns3::ntncon::WalkerConfig wcfg;
    wcfg.num_planes = 1;
    wcfg.total_sats = 80;
    wcfg.altitude_km = altitudeKm;
    wcfg.inclination_deg = 53.0;
    wcfg.epoch_unix_s = 1735689600.0; // 2025-01-01T00:00:00Z
    const auto elements = ns3::ntncon::WalkerConstellation::BuildDelta(wcfg);
    NodeContainer gnbNodes;
    gnbNodes.Create(1);
    Ptr<ns3::ntncon::Sgp4MobilityModel> satMob = CreateObject<ns3::ntncon::Sgp4MobilityModel>();
    satMob->SetElementsSgp4(elements[0]);
    gnbNodes.Get(0)->AggregateObject(satMob);
    double subLat;
    double subLon;
    double subAlt;
    satMob->GetGeodetic(subLat, subLon, subAlt);

    // ---- TR 38.811 UE classes (ntn-cho) in a 0.06 deg box at the offset.
    const double ueLat = subLat + ueNorthOffsetKm / 111.195;
    NodeContainer ueNodes;
    ueNodes.Create(numUes);
    NtnTr38811MobilityHelper ueMobility(1);
    ueMobility.Install(ueNodes,
                       NtnMobilityScenarios::MixedContinental(),
                       ueLat - 0.03,
                       ueLat + 0.03,
                       subLon - 0.03,
                       subLon + 0.03);

    // ---- NR-NTN radio + EPC (ntn-traffic NtnRealStackHelper, nr backend). The
    // link budget is the helper default, TR 38.821 Set-1 S-band handheld
    // (--NtnRealStackLinkBudget=Legacy restores the old operating point).
    NtnRealStackHelper rs;
    rs.SetRadioBackend(NtnRealStackHelper::RadioBackend::Nr);
    rs.SetNumerology(1);
    rs.SetSimTime(Seconds(simTime + 0.5));
    rs.SetOutputDir(outputDir);
    rs.SetRunTag("ntn-cbr-leo-link");
    rs.Build(gnbNodes, ueNodes);
    if (gwDistanceKm > 0.0)
    {
        rs.SetFeederGatewayGeodetic(ueLat + gwDistanceKm / 111.195, subLon, 0.0);
    }

    // ---- The module's CbrApplication on the remote host, a PacketSink per UE.
    const uint16_t port = 4000;
    std::vector<InbandStats> stats(numUes);
    std::ofstream owdTrace(outputDir + "/cbr_owd.csv");
    owdTrace << "time_s,seq,owd_ms\n";
    ApplicationContainer cbrApps;
    ApplicationContainer sinkApps;
    for (uint32_t u = 0; u < numUes; ++u)
    {
        PacketSinkHelper sinkHelper("ns3::UdpSocketFactory",
                                    InetSocketAddress(Ipv4Address::GetAny(), port));
        sinkHelper.SetAttribute("EnableSeqTsSizeHeader", BooleanValue(true));
        ApplicationContainer sink = sinkHelper.Install(ueNodes.Get(u));
        sink.Start(Seconds(0.5));
        sink.Stop(Seconds(simTime + 0.5));
        if (u == 0)
        {
            stats[u].trace = &owdTrace;
        }
        sink.Get(0)->TraceConnectWithoutContext("RxWithSeqTsSize",
                                                MakeCallback(&InbandStats::Rx, &stats[u]));
        CbrHelper cbrHelper("ns3::UdpSocketFactory",
                            InetSocketAddress(rs.GetUeIpv4Address(u), port));
        cbrHelper.SetConstantTraffic(MilliSeconds(intervalMs), pktSize);
        cbrHelper.SetAttribute("EnableSeqTsSizeHeader", BooleanValue(true));
        ApplicationContainer cbr = cbrHelper.Install(rs.GetRemoteHost());
        cbr.Start(Seconds(1.0));
        cbr.Stop(Seconds(simTime));
        rs.AttachExternalDlFlow(u, cbr.Get(0), sink.Get(0), Seconds(1.0), Seconds(simTime));
        cbrApps.Add(cbr);
        sinkApps.Add(sink);
    }

    // FlowMonitor on the two end points only (remote host and UEs), as a
    // cross-check of the in-band numbers.
    FlowMonitorHelper flowmonHelper;
    NodeContainer ends;
    ends.Add(rs.GetRemoteHost());
    ends.Add(ueNodes);
    Ptr<FlowMonitor> monitor = flowmonHelper.Install(ends);

    Simulator::Stop(Seconds(simTime + 0.5));
    Simulator::Run();
    rs.Collect();
    rs.WriteHealthReport();

    uint64_t sentBytes = 0;
    for (uint32_t u = 0; u < numUes; ++u)
    {
        sentBytes += DynamicCast<CbrApplication>(cbrApps.Get(u))->GetSent();
    }
    uint64_t rxBytes = 0;
    uint64_t rxPkts = 0;
    uint64_t expected = 0;
    double sumDelay = 0.0;
    double maxDelay = 0.0;
    double minDelay = std::numeric_limits<double>::infinity();
    uint64_t ipdvN = 0;
    double ipdvSum = 0.0;
    for (uint32_t u = 0; u < numUes; ++u)
    {
        const InbandStats& st = stats[u];
        rxBytes += DynamicCast<PacketSink>(sinkApps.Get(u))->GetTotalRx();
        rxPkts += st.rx;
        expected += st.any ? st.highestSeq + 1 : 0;
        sumDelay += st.sumDelayMs;
        maxDelay = std::max(maxDelay, st.maxDelayMs);
        minDelay = std::min(minDelay, st.minDelayMs);
        ipdvN += st.ipdvN;
        ipdvSum += st.ipdvSumAbsMs;
    }
    const uint64_t sentPkts = (pktSize > 0) ? sentBytes / pktSize : 0;
    const double lossRatio =
        sentPkts > 0 ? 1.0 - static_cast<double>(rxPkts) / static_cast<double>(sentPkts) : 0.0;
    const double meanDelay = rxPkts > 0 ? sumDelay / rxPkts : 0.0;
    const double ipdv = ipdvN > 0 ? ipdvSum / ipdvN : 0.0;
    const double activeS = simTime - 1.0;
    const double thrMbps = rxBytes * 8.0 / activeS / 1e6;

    monitor->CheckForLostPackets();
    uint64_t fmRx = 0;
    double fmDelaySum = 0.0;
    for (const auto& kv : monitor->GetFlowStats())
    {
        fmRx += kv.second.rxPackets;
        fmDelaySum += kv.second.delaySum.GetSeconds() * 1e3;
    }

    // Geometry at the end of the run, UE 0.
    const Vector ue0 = ueNodes.Get(0)->GetObject<MobilityModel>()->GetPosition();
    const Vector sat = satMob->GetPosition();
    const double slantKm = CalculateDistance(ue0, sat) / 1e3;
    const double elevDeg = ntngeo::ElevationDeg(ue0, sat);

    std::ofstream csv(outputDir + "/cbr_kpi.csv");
    csv << "metric,value,unit,provenance\n"
        << "cbr_sent_packets," << sentPkts << ",count,CbrApplication::GetSent\n"
        << "cbr_rx_packets," << rxPkts << ",count,PacketSink+SeqTsSizeHeader\n"
        << "cbr_loss_ratio," << lossRatio << ",ratio,in-band\n"
        << "cbr_seq_expected," << expected << ",count,in-band\n"
        << "throughput_mbps," << thrMbps << ",Mbps,PacketSink\n"
        << "owd_mean_ms," << meanDelay << ",ms,in-band\n"
        << "owd_min_ms," << (rxPkts ? minDelay : 0.0) << ",ms,in-band\n"
        << "owd_max_ms," << maxDelay << ",ms,in-band\n"
        << "ipdv_mean_abs_ms," << ipdv << ",ms,in-band RFC 3393\n"
        << "dl_sinr_db," << rs.GetMeanDlSinrDb() << ",dB,RxPacketTraceUe\n"
        << "dl_tbler," << rs.GetMeanDlTbler() << ",ratio,RxPacketTraceUe\n"
        << "phy_rx_tb," << rs.GetPhyRxTb() << ",count,RxPacketTraceUe\n"
        << "flowmon_rx_packets," << fmRx << ",count,FlowMonitor\n"
        << "flowmon_mean_delay_ms," << (fmRx ? fmDelaySum / fmRx : 0.0) << ",ms,FlowMonitor\n"
        << "ue0_elevation_end_deg," << elevDeg << ",deg,SGP4+TR38.811\n"
        << "ue0_slant_end_km," << slantKm << ",km,SGP4+TR38.811\n";

    std::cout << "==== ntn-cbr-leo-link MEASURED KPIs (radio=nr, SGP4 + NR-NTN + EPC) ====\n"
              << "cbr_sent_packets       = " << sentPkts << "\n"
              << "cbr_rx_packets         = " << rxPkts << "\n"
              << "cbr_loss_ratio         = " << lossRatio << "\n"
              << "throughput_Mbps        = " << thrMbps << "\n"
              << "owd_mean_ms            = " << meanDelay << "  (min " << (rxPkts ? minDelay : 0.0)
              << ", max " << maxDelay << ")\n"
              << "ipdv_mean_abs_ms       = " << ipdv << "\n"
              << "dl_sinr_db             = " << rs.GetMeanDlSinrDb() << "\n"
              << "dl_tbler               = " << rs.GetMeanDlTbler() << "\n"
              << "ue0 elevation / slant  = " << elevDeg << " deg / " << slantKm << " km\n"
              << "flowmon_rx_packets     = " << fmRx << "\n"
              << "outputs in " << outputDir << "\n"
              << "========================================================================"
              << std::endl;
    Simulator::Destroy();
    return rs.GetPhyRxTb() > 0 ? 0 : 1;
}
} // namespace
