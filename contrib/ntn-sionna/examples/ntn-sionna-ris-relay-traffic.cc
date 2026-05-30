/*
 * SPDX-License-Identifier: GPL-2.0-only
 * Copyright (c) 2026 Muhammad Uzair (ns3-ntn-toolkit)
 *
 * ntn-sionna-ris-relay-traffic — a LEO downlink whose direct path is blocked
 * (NLOS, e.g. urban canyon / terrain) is recovered by a Reconfigurable
 * Intelligent Surface that is switched ON mid-simulation. REAL UDP traffic
 * flows the whole time; goodput is near-zero while the link is blocked, then
 * jumps once the RIS provides a coherent specular path.
 *
 * The RIS gain is the standard perfect-CSI coherent-combining law
 *   G_ris(N) = 20*log10(N_elements)   [dB]
 * computed from the RisConfig rows*cols supplied on the channel (the same
 * descriptor the Sionna RT server consumes). With a live Sionna server the
 * gain comes from actual ray tracing of the surface; in the standalone
 * FSPL-fallback run shown here it is the closed-form array gain, so the
 * example runs without a GPU while still being parameter-driven (try
 * --risRows / --risCols / --blockageDb).
 *
 * Quick test:  --simSeconds=120 --dataRateMbps=5
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
#include "ns3/point-to-point-channel.h"
#include "ns3/point-to-point-helper.h"

#include "ns3/ntn-atmospheric-loss-chain.h"
#include "ns3/ntn-sionna-cascade-channel.h"
#include "ns3/sionna-transport.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("NtnSionnaRisRelayTraffic");

namespace
{
constexpr double kC = 299792458.0;
Ptr<NtnSionnaCascadeChannel> g_loss;
Ptr<MobilityModel> g_gnd;
Ptr<MobilityModel> g_sat;
Ptr<RateErrorModel> g_em;
Ptr<PointToPointChannel> g_channel;
Ptr<PacketSink> g_sink;
uint64_t g_lastRx = 0;
double g_baseEirpDbm = 88.0;
double g_blockageDb = 30.0; // NLOS blockage on the direct path
double g_risGainDb = 0.0;   // active RIS coherent gain (0 until engaged)
double g_noiseDbm = -98.0;
double g_minElev = 5.0;
bool g_risOn = false;

double
ElevDeg(const Vector& u, const Vector& s)
{
    const Vector d(s.x - u.x, s.y - u.y, s.z - u.z);
    return std::atan2(d.z, std::max(std::sqrt(d.x * d.x + d.y * d.y), 1e-3)) *
           180.0 / M_PI;
}

double
Dist(const Vector& a, const Vector& b)
{
    const double dx = a.x - b.x, dy = a.y - b.y, dz = a.z - b.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double
SnrToPer(double snrDb)
{
    return 1.0 / (1.0 + std::exp(0.8 * (snrDb - 6.0)));
}

void
EngageRis(double gainDb)
{
    g_risOn = true;
    g_risGainDb = gainDb;
}

void
LinkProbe()
{
    const Vector u = g_gnd->GetPosition();
    const Vector s = g_sat->GetPosition();
    const double elev = ElevDeg(u, s);
    const double range = Dist(u, s);
    // Effective EIRP = base EIRP - NLOS blockage + RIS coherent gain (if on).
    const double effEirp = g_baseEirpDbm - g_blockageDb + g_risGainDb;
    const double rxDbm = g_loss->CalcRxPower(effEirp, g_sat, g_gnd);
    const double snr = rxDbm - g_noiseDbm;
    double per = (elev < g_minElev) ? 1.0 : SnrToPer(snr);
    g_em->SetRate(per);
    g_channel->SetAttribute("Delay", TimeValue(Seconds(range / kC)));

    const uint64_t tot = g_sink ? g_sink->GetTotalRx() : 0;
    const double mbps = (tot - g_lastRx) * 8.0 / 1e6;
    g_lastRx = tot;
    std::printf("  %6.1f  %7.2f  %5s  %8.2f  %8.2f  %9.3f\n",
                Simulator::Now().GetSeconds(), elev,
                g_risOn ? "ON" : "off", g_risGainDb, snr, mbps);
    Simulator::Schedule(Seconds(1.0), &LinkProbe);
}
} // namespace

int
main(int argc, char* argv[])
{
    double simSeconds = 600.0;
    double altKm = 550.0;
    double satSpeed = 7500.0;
    double freqHz = 2.0e9;
    double dataRateMbps = 20.0;
    uint32_t packetBytes = 1200;
    double txPowerDbm = 33.0;
    double antennaGainDb = 55.0;
    double blockageDb = 30.0;
    uint32_t risRows = 32;
    uint32_t risCols = 32;
    double risOnFraction = 0.4; // engage RIS at 40% of the sim
    double linkCapacityMbps = 50.0;

    CommandLine cmd(__FILE__);
    cmd.AddValue("simSeconds", "Simulation duration (s)", simSeconds);
    cmd.AddValue("altKm", "Satellite altitude (km)", altKm);
    cmd.AddValue("satSpeed", "Satellite ground-track speed (m/s)", satSpeed);
    cmd.AddValue("freqHz", "Carrier frequency (Hz)", freqHz);
    cmd.AddValue("dataRateMbps", "Offered downlink load (Mbps)", dataRateMbps);
    cmd.AddValue("packetBytes", "UDP payload size (bytes)", packetBytes);
    cmd.AddValue("txPowerDbm", "Satellite HPA output power (dBm)", txPowerDbm);
    cmd.AddValue("antennaGainDb", "Combined antenna gain (dB)", antennaGainDb);
    cmd.AddValue("blockageDb", "NLOS blockage on the direct path (dB)", blockageDb);
    cmd.AddValue("risRows", "RIS element rows", risRows);
    cmd.AddValue("risCols", "RIS element cols", risCols);
    cmd.AddValue("risOnFraction", "Fraction of sim at which RIS engages",
                  risOnFraction);
    cmd.AddValue("linkCapacityMbps", "P2P link capacity (Mbps)", linkCapacityMbps);
    cmd.Parse(argc, argv);

    g_baseEirpDbm = txPowerDbm + antennaGainDb;
    g_blockageDb = blockageDb;
    const double risGainDb = 20.0 * std::log10(std::max(1u, risRows * risCols));

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

    Ptr<NtnSionnaChannel> base = CreateObject<NtnSionnaChannel>();
    base->SetFrequencyHz(freqHz);
    // Attach the RIS descriptor to the channel (consumed by a live Sionna
    // server; informational in FSPL-fallback runs).
    RisConfig ris;
    ris.rows = static_cast<uint16_t>(risRows);
    ris.cols = static_cast<uint16_t>(risCols);
    ris.phase_profile = "focus";
    base->SetRis(ris);
    Ptr<NtnAtmosphericLossChain> chain = CreateObject<NtnAtmosphericLossChain>();
    chain->SetFrequencyHz(freqHz);
    Ptr<NtnSionnaCascadeChannel> loss = CreateObject<NtnSionnaCascadeChannel>();
    loss->SetSionnaChannel(base);
    loss->SetAtmosphericChain(chain);
    g_loss = loss;

    PointToPointHelper p2p;
    p2p.SetDeviceAttribute(
        "DataRate",
        DataRateValue(DataRate(static_cast<uint64_t>(linkCapacityMbps * 1e6))));
    p2p.SetChannelAttribute("Delay", TimeValue(Seconds(altKm * 1000.0 / kC)));
    NetDeviceContainer devices = p2p.Install(nodes);
    Ptr<RateErrorModel> em = CreateObject<RateErrorModel>();
    em->SetUnit(RateErrorModel::ERROR_UNIT_PACKET);
    em->SetRate(1.0);
    devices.Get(0)->SetAttribute("ReceiveErrorModel", PointerValue(em));
    g_em = em;
    g_channel = DynamicCast<PointToPointChannel>(devices.Get(0)->GetChannel());

    InternetStackHelper internet;
    internet.Install(nodes);
    Ipv4AddressHelper ipv4;
    ipv4.SetBase("10.3.1.0", "255.255.255.0");
    Ipv4InterfaceContainer ifaces = ipv4.Assign(devices);

    const uint16_t port = 9200;
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

    Simulator::Schedule(Seconds(risOnFraction * simSeconds), &EngageRis,
                        risGainDb);

    FlowMonitorHelper fmHelper;
    Ptr<FlowMonitor> monitor = fmHelper.InstallAll();

    std::printf("# ntn-sionna-ris-relay-traffic\n");
    std::printf("#   sim=%.0fs alt=%.0fkm freq=%.1fGHz load=%.1fMbps "
                "baseEIRP=%.1fdBm blockage=%.0fdB RIS=%ux%u→%.1fdB @%.0fs\n",
                simSeconds, altKm, freqHz / 1e9, dataRateMbps, g_baseEirpDbm,
                blockageDb, risRows, risCols, risGainDb,
                risOnFraction * simSeconds);
    std::printf("# %5s  %7s  %5s  %8s  %8s  %9s\n",
                "t_s", "elev", "ris", "ris_dB", "snr_dB", "goodput");

    Simulator::Schedule(Seconds(2.0), &LinkProbe);
    Simulator::Stop(Seconds(simSeconds + 0.1));
    Simulator::Run();

    monitor->CheckForLostPackets();
    const auto stats = monitor->GetFlowStats();
    uint64_t txP = 0, rxP = 0;
    for (const auto& kv : stats)
    {
        txP += kv.second.txPackets;
        rxP += kv.second.rxPackets;
    }
    const uint64_t totalRx = g_sink ? g_sink->GetTotalRx() : 0;
    std::printf("# === summary ===  txPackets=%lu rxPackets=%lu PDR=%.2f%% "
                "avgGoodput=%.3f Mbps (RIS gain %.1f dB)\n",
                (unsigned long)txP, (unsigned long)rxP,
                txP ? 100.0 * rxP / txP : 0.0,
                totalRx * 8.0 / simSeconds / 1e6, risGainDb);
    Simulator::Destroy();
    return 0;
}
