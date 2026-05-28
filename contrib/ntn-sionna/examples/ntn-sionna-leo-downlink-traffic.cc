/*
 * SPDX-License-Identifier: GPL-2.0-only
 * Copyright (c) 2026 Muhammad Uzair (ns3-ntn-toolkit)
 *
 * ntn-sionna-leo-downlink-traffic — END-TO-END packet transmission over a
 * LEO downlink whose physical link is gated by the NtnSionnaCascadeChannel
 * (Sionna RT multipath + ITU-R atmospheric cascade, with FSPL fall-back).
 *
 * This runs a REAL ns-3 data plane. The ground gateway and the LEO satellite
 * are joined by a point-to-point link carrying genuine UDP traffic; what makes
 * the link behave like a satellite link is that, every second of sim time, a
 * probe:
 *   1. reads the live geometry from the satellite's ConstantVelocityMobility,
 *   2. asks the NtnSionnaCascadeChannel for the Rx power at that geometry,
 *   3. converts it to an SNR (against a kTB noise floor) and a packet-error
 *      rate, and installs that PER on the receiver's RateErrorModel, and
 *   4. updates the channel propagation delay from the live slant range.
 *
 * A point-to-point link (rather than Wi-Fi) is used deliberately: Wi-Fi's MAC
 * ACK timeout is microseconds and breaks at the 2–9 ms one-way delays of a
 * LEO link, whereas a P2P link models exactly the propagation delay + a
 * physically-derived error rate. So delivered throughput RISES as the
 * satellite climbs toward zenith (path loss + delay drop) and FALLS / stops
 * as it sets — nothing is hardcoded; every number tracks sim time + CLI args.
 *
 * Run:
 *   ./ns3 run "ntn-sionna-leo-downlink-traffic --simSeconds=600 --altKm=550 \
 *              --rainMmH=25 --dataRateMbps=20 --freqHz=2.0e9"
 */
#include "ns3/applications-module.h"
#include "ns3/command-line.h"
#include "ns3/constant-position-mobility-model.h"
#include "ns3/constant-velocity-mobility-model.h"
#include "ns3/core-module.h"
#include "ns3/error-model.h"
#include "ns3/flow-monitor-helper.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/ipv4-flow-classifier.h"
#include "ns3/point-to-point-channel.h"
#include "ns3/point-to-point-helper.h"

#include "ns3/ntn-atmospheric-loss-chain.h"
#include "ns3/ntn-sionna-cascade-channel.h"
#include "ns3/sionna-udp-transport.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("NtnSionnaLeoDownlinkTraffic");

namespace
{

constexpr double kC = 299792458.0;

Ptr<NtnSionnaCascadeChannel> g_loss;
Ptr<MobilityModel> g_gnd;
Ptr<MobilityModel> g_sat;
Ptr<RateErrorModel> g_errorModel;
Ptr<PointToPointChannel> g_channel;
Ptr<PacketSink> g_sink;
uint64_t g_lastRx = 0;
double g_eirpDbm = 75.0;
double g_noiseFloorDbm = -98.0;
double g_minElevDeg = 5.0;

double
ElevDeg(const Vector& ue, const Vector& sat)
{
    const Vector d(sat.x - ue.x, sat.y - ue.y, sat.z - ue.z);
    const double horiz = std::sqrt(d.x * d.x + d.y * d.y);
    return std::atan2(d.z, std::max(horiz, 1e-3)) * 180.0 / M_PI;
}

double
Distance(const Vector& a, const Vector& b)
{
    const double dx = a.x - b.x, dy = a.y - b.y, dz = a.z - b.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

// Map SNR (dB) → packet error rate with a smooth waterfall around a 6 dB
// decoding threshold (representative of a robust NTN MODCOD). Below the
// threshold PER → 1, ~10 dB above it PER → 0.
double
SnrToPer(double snrDb)
{
    const double thresholdDb = 6.0;
    const double x = snrDb - thresholdDb;
    // Logistic waterfall: PER = 1 / (1 + exp(k*x)).
    const double k = 0.8;
    return 1.0 / (1.0 + std::exp(k * x));
}

// Per-second probe: recompute geometry → Rx power → SNR → PER, install it on
// the receiver, update propagation delay, and log the goodput delivered in
// the last second (real received bytes from the PacketSink counter).
void
LinkProbe()
{
    const Vector g = g_gnd->GetPosition();
    const Vector s = g_sat->GetPosition();
    const double elev = ElevDeg(g, s);
    const double range = Distance(g, s);
    const double rxDbm = g_loss->CalcRxPower(g_eirpDbm, g_sat, g_gnd);
    const double snrDb = rxDbm - g_noiseFloorDbm;

    double per = SnrToPer(snrDb);
    if (elev < g_minElevDeg)
    {
        per = 1.0; // below horizon / min elevation → link unusable
    }
    g_errorModel->SetRate(per);
    g_errorModel->SetUnit(RateErrorModel::ERROR_UNIT_PACKET);
    // Update propagation delay from live slant range.
    g_channel->SetAttribute("Delay", TimeValue(Seconds(range / kC)));

    const uint64_t totalRx = g_sink ? g_sink->GetTotalRx() : 0;
    const double lastSecBytes = static_cast<double>(totalRx - g_lastRx);
    g_lastRx = totalRx;
    const double mbps = lastSecBytes * 8.0 / 1e6;
    std::printf("  %6.1f  %8.2f  %10.2f  %8.2f  %7.3f  %10.3f\n",
                Simulator::Now().GetSeconds(), elev, rxDbm, snrDb, per, mbps);
    Simulator::Schedule(Seconds(1.0), &LinkProbe);
}

} // namespace

int
main(int argc, char* argv[])
{
    double simSeconds = 600.0;
    double altKm = 550.0;
    double satSpeed = 7500.0;
    double rainMmH = 0.0;
    double freqHz = 2.0e9;
    double dataRateMbps = 20.0;
    uint32_t packetBytes = 1200;
    double txPowerDbm = 33.0;
    double antennaGainDb = 42.0;
    double noiseFloorDbm = -98.0;
    double minElevDeg = 5.0;
    double linkCapacityMbps = 50.0;
    std::string sionnaHost = "";
    uint16_t sionnaPort = 8765;

    CommandLine cmd(__FILE__);
    cmd.AddValue("simSeconds", "Simulation duration (s)", simSeconds);
    cmd.AddValue("altKm", "Satellite altitude (km)", altKm);
    cmd.AddValue("satSpeed", "Satellite ground-track speed (m/s)", satSpeed);
    cmd.AddValue("rainMmH", "Rain rate (mm/h, 0 disables)", rainMmH);
    cmd.AddValue("freqHz", "Carrier frequency for the NTN loss model (Hz)", freqHz);
    cmd.AddValue("dataRateMbps", "Offered downlink load (Mbps)", dataRateMbps);
    cmd.AddValue("packetBytes", "UDP payload size (bytes)", packetBytes);
    cmd.AddValue("txPowerDbm", "Satellite HPA output power (dBm)", txPowerDbm);
    cmd.AddValue("antennaGainDb", "Combined Tx+Rx antenna gain (dB)", antennaGainDb);
    cmd.AddValue("noiseFloorDbm", "Receiver noise floor (dBm)", noiseFloorDbm);
    cmd.AddValue("minElevDeg", "Min elevation for a usable link (deg)", minElevDeg);
    cmd.AddValue("linkCapacityMbps", "P2P link capacity (Mbps)", linkCapacityMbps);
    cmd.AddValue("sionnaHost", "Sionna RT server host (empty = FSPL fallback)",
                  sionnaHost);
    cmd.AddValue("sionnaPort", "Sionna RT server port", sionnaPort);
    cmd.Parse(argc, argv);

    const double eirpDbm = txPowerDbm + antennaGainDb;
    g_eirpDbm = eirpDbm;
    g_noiseFloorDbm = noiseFloorDbm;
    g_minElevDeg = minElevDeg;

    // --- Nodes: ground gateway (0) + LEO satellite (1) ---
    NodeContainer nodes;
    nodes.Create(2);

    Ptr<ConstantPositionMobilityModel> gnd =
        CreateObject<ConstantPositionMobilityModel>();
    gnd->SetPosition(Vector(0, 0, 0));
    nodes.Get(0)->AggregateObject(gnd);

    Ptr<ConstantVelocityMobilityModel> sat =
        CreateObject<ConstantVelocityMobilityModel>();
    sat->SetPosition(Vector(-0.5 * satSpeed * simSeconds, 0, altKm * 1000.0));
    sat->SetVelocity(Vector(satSpeed, 0, 0));
    nodes.Get(1)->AggregateObject(sat);
    g_gnd = gnd;
    g_sat = sat;

    // --- NTN cascade channel (Sionna RT + ITU-R), FSPL fall-back ---
    Ptr<NtnSionnaChannel> base = CreateObject<NtnSionnaChannel>();
    base->SetFrequencyHz(freqHz);
    if (!sionnaHost.empty())
    {
        base->SetServer(sionnaHost, sionnaPort);
        base->SetTimeoutMs(50);
    }
    Ptr<NtnAtmosphericLossChain> chain = CreateObject<NtnAtmosphericLossChain>();
    chain->SetFrequencyHz(freqHz);
    chain->SetRainRateMmH(rainMmH);
    Ptr<NtnSionnaCascadeChannel> loss = CreateObject<NtnSionnaCascadeChannel>();
    loss->SetSionnaChannel(base);
    loss->SetAtmosphericChain(chain);
    g_loss = loss;

    // --- Point-to-point downlink: capacity-bounded, geometry-driven delay ---
    PointToPointHelper p2p;
    p2p.SetDeviceAttribute(
        "DataRate",
        DataRateValue(DataRate(static_cast<uint64_t>(linkCapacityMbps * 1e6))));
    p2p.SetChannelAttribute("Delay", TimeValue(Seconds(altKm * 1000.0 / kC)));
    NetDeviceContainer devices = p2p.Install(nodes);

    // Receiver-side error model driven by the live SNR each second.
    Ptr<RateErrorModel> em = CreateObject<RateErrorModel>();
    em->SetUnit(RateErrorModel::ERROR_UNIT_PACKET);
    em->SetRate(1.0); // start "down" until the first probe sets it
    devices.Get(0)->SetAttribute("ReceiveErrorModel", PointerValue(em));
    g_errorModel = em;
    g_channel = DynamicCast<PointToPointChannel>(devices.Get(0)->GetChannel());

    // --- Internet stack + addressing ---
    InternetStackHelper internet;
    internet.Install(nodes);
    Ipv4AddressHelper ipv4;
    ipv4.SetBase("10.1.1.0", "255.255.255.0");
    Ipv4InterfaceContainer ifaces = ipv4.Assign(devices);

    // --- Downlink traffic: satellite (1) → ground gateway (0) ---
    const uint16_t port = 9000;
    PacketSinkHelper sinkHelper(
        "ns3::UdpSocketFactory",
        InetSocketAddress(Ipv4Address::GetAny(), port));
    ApplicationContainer sinkApp = sinkHelper.Install(nodes.Get(0));
    sinkApp.Start(Seconds(0.0));
    sinkApp.Stop(Seconds(simSeconds));
    g_sink = DynamicCast<PacketSink>(sinkApp.Get(0));

    OnOffHelper onoff("ns3::UdpSocketFactory",
                      InetSocketAddress(ifaces.GetAddress(0), port));
    onoff.SetAttribute("DataRate",
                       DataRateValue(DataRate(static_cast<uint64_t>(
                           dataRateMbps * 1e6))));
    onoff.SetAttribute("PacketSize", UintegerValue(packetBytes));
    onoff.SetAttribute("OnTime",
                       StringValue("ns3::ConstantRandomVariable[Constant=1]"));
    onoff.SetAttribute("OffTime",
                       StringValue("ns3::ConstantRandomVariable[Constant=0]"));
    ApplicationContainer srcApp = onoff.Install(nodes.Get(1));
    srcApp.Start(Seconds(1.0));
    srcApp.Stop(Seconds(simSeconds));

    FlowMonitorHelper fmHelper;
    Ptr<FlowMonitor> monitor = fmHelper.InstallAll();

    std::printf("# ntn-sionna-leo-downlink-traffic\n");
    std::printf("#   sim=%.0fs alt=%.0fkm satSpeed=%.0fm/s rain=%.1fmm/h "
                "freq=%.2fGHz load=%.1fMbps link=%.1fMbps EIRP=%.1fdBm "
                "noise=%.1fdBm minElev=%.1f transport=%s\n",
                simSeconds, altKm, satSpeed, rainMmH, freqHz / 1e9,
                dataRateMbps, linkCapacityMbps, eirpDbm, noiseFloorDbm,
                minElevDeg,
                sionnaHost.empty() ? "FSPL-fallback" : sionnaHost.c_str());
    std::printf("# %5s  %8s  %10s  %8s  %7s  %10s\n",
                "t_s", "elev_deg", "rx_dBm", "snr_dB", "per", "goodput_Mbps");

    Simulator::Schedule(Seconds(2.0), &LinkProbe);
    Simulator::Stop(Seconds(simSeconds + 0.1));
    Simulator::Run();

    monitor->CheckForLostPackets();
    const auto stats = monitor->GetFlowStats();
    uint64_t txP = 0, rxP = 0, lostP = 0;
    double sumDelay = 0.0;
    uint64_t rxForDelay = 0;
    for (const auto& kv : stats)
    {
        const auto& s = kv.second;
        txP += s.txPackets;
        rxP += s.rxPackets;
        lostP += s.lostPackets;
        sumDelay += s.delaySum.GetSeconds();
        rxForDelay += s.rxPackets;
    }
    const double pdr = txP ? (100.0 * rxP / txP) : 0.0;
    const double meanDelayMs = rxForDelay ? (sumDelay / rxForDelay) * 1000.0 : 0.0;
    const uint64_t totalRx = g_sink ? g_sink->GetTotalRx() : 0;
    const double avgGoodput = totalRx * 8.0 / simSeconds / 1e6;

    std::printf("# === FlowMonitor summary ===\n");
    std::printf("#   txPackets=%lu rxPackets=%lu lostPackets=%lu PDR=%.2f%%\n",
                (unsigned long)txP, (unsigned long)rxP, (unsigned long)lostP,
                pdr);
    std::printf("#   meanDelay=%.3f ms  avgGoodput=%.3f Mbps  totalRxBytes=%lu\n",
                meanDelayMs, avgGoodput, (unsigned long)totalRx);

    Simulator::Destroy();
    return 0;
}
