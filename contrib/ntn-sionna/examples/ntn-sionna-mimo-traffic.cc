/*
 * SPDX-License-Identifier: GPL-2.0-only
 * Copyright (c) 2026 Muhammad Uzair (ns3-ntn-toolkit)
 *
 * ntn-sionna-mimo-traffic — two ground UEs served by the same passing LEO
 * satellite over the same geometry, one with a SISO terminal and one with an
 * N×N MIMO terminal. REAL UDP downlink traffic flows to BOTH; FlowMonitor
 * shows the MIMO UE sustaining higher delivered throughput across the pass
 * because its beamforming array gain (10*log10(N_elements), the rank-1 LOS
 * NTN MIMO gain) keeps the SNR above the decoding waterfall for more of the
 * pass than the SISO UE.
 *
 * The MIMO descriptor (rows×cols) is also attached to the channel via
 * MimoArrayConfig — the same field a live Sionna RT server consumes for true
 * spatial channel synthesis; here the standalone run uses the closed-form
 * array gain so no GPU is required, and the comparison is fully
 * parameter-driven (--rows / --cols).
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

NS_LOG_COMPONENT_DEFINE("NtnSionnaMimoTraffic");

namespace
{
constexpr double kC = 299792458.0;

struct Link
{
    Ptr<NtnSionnaCascadeChannel> loss;
    Ptr<MobilityModel> ue;
    Ptr<RateErrorModel> em;
    Ptr<PointToPointChannel> channel;
    Ptr<PacketSink> sink;
    double extraGainDb{0.0};
    uint64_t lastRx{0};
    const char* tag{"siso"};
};

Ptr<MobilityModel> g_sat;
Link g_siso;
Link g_mimo;
double g_eirpDbm = 88.0;
double g_noiseDbm = -98.0;
double g_minElev = 5.0;

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

double
UpdateLink(Link& L)
{
    const Vector u = L.ue->GetPosition();
    const Vector s = g_sat->GetPosition();
    const double elev = ElevDeg(u, s);
    const double range = Dist(u, s);
    const double rxDbm = L.loss->CalcRxPower(g_eirpDbm + L.extraGainDb, g_sat, L.ue);
    const double snr = rxDbm - g_noiseDbm;
    const double per = (elev < g_minElev) ? 1.0 : SnrToPer(snr);
    L.em->SetRate(per);
    L.channel->SetAttribute("Delay", TimeValue(Seconds(range / kC)));
    const uint64_t tot = L.sink ? L.sink->GetTotalRx() : 0;
    const double mbps = (tot - L.lastRx) * 8.0 / 1e6;
    L.lastRx = tot;
    return mbps;
}

void
MimoProbe()
{
    const Vector s = g_sat->GetPosition();
    const double elev = ElevDeg(g_siso.ue->GetPosition(), s);
    const double sisoMbps = UpdateLink(g_siso);
    const double mimoMbps = UpdateLink(g_mimo);
    std::printf("  %6.1f  %7.2f  %10.3f  %10.3f\n",
                Simulator::Now().GetSeconds(), elev, sisoMbps, mimoMbps);
    Simulator::Schedule(Seconds(1.0), &MimoProbe);
}

Link
BuildLink(Ptr<Node> gndNode, Ptr<Node> satNode, Ptr<MobilityModel> ueMob,
          double freqHz, double capMbps, double altKm, const char* base,
          const char* tag, double extraGainDb, const MimoArrayConfig* mimo,
          uint16_t port)
{
    Link L;
    L.ue = ueMob;
    L.tag = tag;
    L.extraGainDb = extraGainDb;

    Ptr<NtnSionnaChannel> b = CreateObject<NtnSionnaChannel>();
    b->SetFrequencyHz(freqHz);
    if (mimo)
    {
        b->SetTxArray(*mimo);
    }
    Ptr<NtnAtmosphericLossChain> ch = CreateObject<NtnAtmosphericLossChain>();
    ch->SetFrequencyHz(freqHz);
    Ptr<NtnSionnaCascadeChannel> loss = CreateObject<NtnSionnaCascadeChannel>();
    loss->SetSionnaChannel(b);
    loss->SetAtmosphericChain(ch);
    L.loss = loss;

    PointToPointHelper p2p;
    p2p.SetDeviceAttribute(
        "DataRate", DataRateValue(DataRate(static_cast<uint64_t>(capMbps * 1e6))));
    p2p.SetChannelAttribute("Delay", TimeValue(Seconds(altKm * 1000.0 / kC)));
    NetDeviceContainer dev = p2p.Install(NodeContainer(gndNode, satNode));
    Ptr<RateErrorModel> em = CreateObject<RateErrorModel>();
    em->SetUnit(RateErrorModel::ERROR_UNIT_PACKET);
    em->SetRate(1.0);
    dev.Get(0)->SetAttribute("ReceiveErrorModel", PointerValue(em));
    L.em = em;
    L.channel = DynamicCast<PointToPointChannel>(dev.Get(0)->GetChannel());

    Ipv4AddressHelper ipv4;
    ipv4.SetBase(base, "255.255.255.0");
    Ipv4InterfaceContainer ifaces = ipv4.Assign(dev);

    PacketSinkHelper sinkHelper(
        "ns3::UdpSocketFactory",
        InetSocketAddress(Ipv4Address::GetAny(), port));
    ApplicationContainer sinkApp = sinkHelper.Install(gndNode);
    sinkApp.Start(Seconds(0.0));
    L.sink = DynamicCast<PacketSink>(sinkApp.Get(0));

    return L;
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
    double txPowerDbm = 23.0;     // deliberately tight so the SISO link is
    double antennaGainDb = 33.0;  // SNR-limited and MIMO gain is decisive
    uint32_t rows = 8;
    uint32_t cols = 8;
    double linkCapacityMbps = 50.0;

    CommandLine cmd(__FILE__);
    cmd.AddValue("simSeconds", "Simulation duration (s)", simSeconds);
    cmd.AddValue("altKm", "Satellite altitude (km)", altKm);
    cmd.AddValue("satSpeed", "Satellite ground-track speed (m/s)", satSpeed);
    cmd.AddValue("freqHz", "Carrier frequency (Hz)", freqHz);
    cmd.AddValue("dataRateMbps", "Offered downlink load per UE (Mbps)", dataRateMbps);
    cmd.AddValue("packetBytes", "UDP payload size (bytes)", packetBytes);
    cmd.AddValue("txPowerDbm", "Satellite HPA output power (dBm)", txPowerDbm);
    cmd.AddValue("antennaGainDb", "Combined antenna gain, SISO baseline (dB)",
                  antennaGainDb);
    cmd.AddValue("rows", "MIMO array rows", rows);
    cmd.AddValue("cols", "MIMO array cols", cols);
    cmd.AddValue("linkCapacityMbps", "P2P link capacity (Mbps)", linkCapacityMbps);
    cmd.Parse(argc, argv);

    g_eirpDbm = txPowerDbm + antennaGainDb;
    const double mimoGainDb = 10.0 * std::log10(std::max(1u, rows * cols));

    // Ground node hosts both UE terminals; satellite is shared.
    NodeContainer gnd;
    gnd.Create(1);
    NodeContainer satC;
    satC.Create(1);
    InternetStackHelper internet;
    internet.Install(gnd);
    internet.Install(satC);

    Ptr<ConstantVelocityMobilityModel> sat =
        CreateObject<ConstantVelocityMobilityModel>();
    sat->SetPosition(Vector(-0.5 * satSpeed * simSeconds, 0, altKm * 1000.0));
    sat->SetVelocity(Vector(satSpeed, 0, 0));
    satC.Get(0)->AggregateObject(sat);
    g_sat = sat;

    Ptr<ConstantPositionMobilityModel> ueMob =
        CreateObject<ConstantPositionMobilityModel>();
    ueMob->SetPosition(Vector(0, 0, 0));

    MimoArrayConfig mimo;
    mimo.rows = static_cast<uint8_t>(rows);
    mimo.cols = static_cast<uint8_t>(cols);
    mimo.polarization = "VH";

    const uint16_t sisoPort = 9300;
    const uint16_t mimoPort = 9301;
    g_siso = BuildLink(gnd.Get(0), satC.Get(0), ueMob, freqHz, linkCapacityMbps,
                       altKm, "10.4.1.0", "siso", 0.0, nullptr, sisoPort);
    g_mimo = BuildLink(gnd.Get(0), satC.Get(0), ueMob, freqHz, linkCapacityMbps,
                       altKm, "10.4.2.0", "mimo", mimoGainDb, &mimo, mimoPort);
    g_siso.sink->SetStopTime(Seconds(simSeconds));
    g_mimo.sink->SetStopTime(Seconds(simSeconds));

    // One downlink flow per terminal (distinct destination subnets + ports).
    auto installFlow = [&](Ptr<Node> satNode, const char* dstAddr,
                           uint16_t port) {
        OnOffHelper onoff("ns3::UdpSocketFactory",
                          InetSocketAddress(Ipv4Address(dstAddr), port));
        onoff.SetAttribute("DataRate",
                           DataRateValue(DataRate(static_cast<uint64_t>(
                               dataRateMbps * 1e6))));
        onoff.SetAttribute("PacketSize", UintegerValue(packetBytes));
        onoff.SetAttribute(
            "OnTime", StringValue("ns3::ConstantRandomVariable[Constant=1]"));
        onoff.SetAttribute(
            "OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0]"));
        ApplicationContainer a = onoff.Install(satNode);
        a.Start(Seconds(1.0));
        a.Stop(Seconds(simSeconds));
    };
    installFlow(satC.Get(0), "10.4.1.1", sisoPort); // → SISO UE
    installFlow(satC.Get(0), "10.4.2.1", mimoPort); // → MIMO UE

    FlowMonitorHelper fmHelper;
    Ptr<FlowMonitor> monitor = fmHelper.InstallAll();

    std::printf("# ntn-sionna-mimo-traffic\n");
    std::printf("#   sim=%.0fs alt=%.0fkm freq=%.1fGHz load=%.1fMbps/UE "
                "sisoEIRP=%.1fdBm MIMO=%ux%u→+%.1fdB beamforming gain\n",
                simSeconds, altKm, freqHz / 1e9, dataRateMbps, g_eirpDbm, rows,
                cols, mimoGainDb);
    std::printf("# %5s  %7s  %10s  %10s\n",
                "t_s", "elev", "siso_Mbps", "mimo_Mbps");

    Simulator::Schedule(Seconds(2.0), &MimoProbe);
    Simulator::Stop(Seconds(simSeconds + 0.1));
    Simulator::Run();

    const uint64_t sisoRx = g_siso.sink ? g_siso.sink->GetTotalRx() : 0;
    const uint64_t mimoRx = g_mimo.sink ? g_mimo.sink->GetTotalRx() : 0;
    std::printf("# === summary ===  SISO avgGoodput=%.3f Mbps  "
                "MIMO avgGoodput=%.3f Mbps  MIMO/SISO=%.2fx\n",
                sisoRx * 8.0 / simSeconds / 1e6,
                mimoRx * 8.0 / simSeconds / 1e6,
                sisoRx ? static_cast<double>(mimoRx) / sisoRx : 0.0);
    Simulator::Destroy();
    return 0;
}
