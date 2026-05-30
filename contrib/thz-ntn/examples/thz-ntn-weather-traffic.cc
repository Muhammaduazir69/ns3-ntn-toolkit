/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only
//
// thz-ntn-weather-traffic — a weather front (fog → rain → wet snow → clear)
// passes over the ground station during a sub-THz LEO pass, exercising
// ThzNtnWeatherAttenuation while REAL UDP traffic flows. Each weather phase
// is computed by the module (ComputeFog/Rain/Snow Attenuation_dB at the live
// elevation + frequency), folded into the link budget alongside FSPL +
// molecular absorption, and mapped to a packet-error rate. So the delivered
// goodput dips by a phase-specific amount — sub-THz is far more sensitive to
// rain/wet-snow than to fog — and recovers afterward. Nothing hardcoded.
//
// Quick test:  --simSeconds=120 --dataRateMbps=10
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

#include "ns3/thz-ntn-molecular-absorption.h"
#include "ns3/thz-ntn-weather-attenuation.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("ThzNtnWeatherTraffic");

namespace
{
constexpr double kC = 299792458.0;

enum class Wx
{
    Clear,
    Fog,
    Rain,
    Snow
};

Ptr<ThzNtnMolecularAbsorption> g_abs;
Ptr<ThzNtnWeatherAttenuation> g_wx;
Ptr<MobilityModel> g_gnd;
Ptr<MobilityModel> g_sat;
Ptr<RateErrorModel> g_em;
Ptr<PointToPointChannel> g_channel;
Ptr<PacketSink> g_sink;
uint64_t g_lastRx = 0;
double g_eirpDbm = 118.0;
double g_freqHz = 140e9;
double g_noiseDbm = -90.0;
double g_satAltKm = 550.0;
Wx g_phase = Wx::Clear;
double g_rainMmH = 25.0;
double g_fogLwc = 0.5;
double g_snowMmH = 10.0;

const char*
PhaseName(Wx w)
{
    switch (w)
    {
    case Wx::Clear: return "clear";
    case Wx::Fog: return "fog";
    case Wx::Rain: return "rain";
    case Wx::Snow: return "wet-snow";
    }
    return "?";
}

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
FsplDb(double dM, double fHz)
{
    return 20.0 * std::log10(std::max(dM, 1.0)) +
           20.0 * std::log10(fHz / 1e9) + 32.45;
}

double
WeatherLossDb(double elev)
{
    switch (g_phase)
    {
    case Wx::Fog:
        return g_wx->ComputeFogAttenuation_dB(g_freqHz, elev, g_fogLwc);
    case Wx::Rain:
        return g_wx->ComputeRainAttenuation_dB(g_freqHz, elev, g_rainMmH);
    case Wx::Snow:
        return g_wx->ComputeSnowAttenuation_dB(g_freqHz, elev, g_snowMmH, true);
    case Wx::Clear:
    default:
        return 0.0;
    }
}

double
SnrToPer(double snrDb)
{
    return 1.0 / (1.0 + std::exp(0.8 * (snrDb - 6.0)));
}

void
SetPhase(Wx w)
{
    g_phase = w;
}

void
LinkProbe()
{
    const Vector u = g_gnd->GetPosition();
    const Vector s = g_sat->GetPosition();
    const double elev = ElevDeg(u, s);
    const double range = Dist(u, s);
    const double fspl = FsplDb(range, g_freqHz);
    const double mol =
        (elev > 0.0)
            ? g_abs->ComputeSlantPathAbsorption(g_freqHz, elev, 0.05, g_satAltKm)
            : 200.0;
    const double wxLoss = (elev > 0.0) ? WeatherLossDb(elev) : 0.0;
    const double rxDbm = g_eirpDbm - fspl - mol - wxLoss;
    const double snr = rxDbm - g_noiseDbm;
    const double per = (elev < 10.0) ? 1.0 : SnrToPer(snr);
    g_em->SetRate(per);
    g_channel->SetAttribute("Delay", TimeValue(Seconds(range / kC)));

    const uint64_t tot = g_sink ? g_sink->GetTotalRx() : 0;
    const double mbps = (tot - g_lastRx) * 8.0 / 1e6;
    g_lastRx = tot;
    std::printf("  %6.1f  %7.2f  %-9s  %8.2f  %8.2f  %9.3f\n",
                Simulator::Now().GetSeconds(), elev, PhaseName(g_phase), wxLoss,
                snr, mbps);
    Simulator::Schedule(Seconds(1.0), &LinkProbe);
}
} // namespace

int
main(int argc, char* argv[])
{
    double simSeconds = 600.0;
    double altKm = 550.0;
    double satSpeed = 7500.0;
    double freqGHz = 140.0;
    double dataRateMbps = 50.0;
    uint32_t packetBytes = 1200;
    double txPowerDbm = 30.0;
    double antennaGainDb = 92.0;
    double rainMmH = 25.0;
    double fogLwc = 0.5;
    double snowMmH = 10.0;
    double linkCapacityMbps = 200.0;

    CommandLine cmd(__FILE__);
    cmd.AddValue("simSeconds", "Simulation duration (s)", simSeconds);
    cmd.AddValue("altKm", "Satellite altitude (km)", altKm);
    cmd.AddValue("satSpeed", "Satellite ground-track speed (m/s)", satSpeed);
    cmd.AddValue("freqGHz", "Carrier frequency (GHz)", freqGHz);
    cmd.AddValue("dataRateMbps", "Offered downlink load (Mbps)", dataRateMbps);
    cmd.AddValue("packetBytes", "UDP payload size (bytes)", packetBytes);
    cmd.AddValue("txPowerDbm", "HPA output power (dBm)", txPowerDbm);
    cmd.AddValue("antennaGainDb", "Combined antenna gain (dB)", antennaGainDb);
    cmd.AddValue("rainMmH", "Rain rate during the rain phase (mm/h)", rainMmH);
    cmd.AddValue("fogLwc", "Fog liquid water content (g/m^3)", fogLwc);
    cmd.AddValue("snowMmH", "Snow rate during the snow phase (mm/h)", snowMmH);
    cmd.AddValue("linkCapacityMbps", "P2P link capacity (Mbps)", linkCapacityMbps);
    cmd.Parse(argc, argv);

    g_eirpDbm = txPowerDbm + antennaGainDb;
    g_freqHz = freqGHz * 1e9;
    g_satAltKm = altKm;
    g_rainMmH = rainMmH;
    g_fogLwc = fogLwc;
    g_snowMmH = snowMmH;
    g_abs = CreateObject<ThzNtnMolecularAbsorption>();
    g_wx = CreateObject<ThzNtnWeatherAttenuation>();

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
    ipv4.SetBase("10.14.1.0", "255.255.255.0");
    Ipv4InterfaceContainer ifaces = ipv4.Assign(devices);

    const uint16_t port = 9900;
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

    // Weather front: clear → fog → rain → wet snow → clear across the pass.
    Simulator::Schedule(Seconds(0.20 * simSeconds), &SetPhase, Wx::Fog);
    Simulator::Schedule(Seconds(0.35 * simSeconds), &SetPhase, Wx::Rain);
    Simulator::Schedule(Seconds(0.55 * simSeconds), &SetPhase, Wx::Snow);
    Simulator::Schedule(Seconds(0.75 * simSeconds), &SetPhase, Wx::Clear);

    FlowMonitorHelper fmHelper;
    Ptr<FlowMonitor> monitor = fmHelper.InstallAll();

    std::printf("# thz-ntn-weather-traffic\n");
    std::printf("#   sim=%.0fs alt=%.0fkm freq=%.0fGHz load=%.1fMbps "
                "EIRP=%.1fdBm rain=%.0fmm/h fogLwc=%.2f snow=%.0fmm/h\n",
                simSeconds, altKm, freqGHz, dataRateMbps, g_eirpDbm, rainMmH,
                fogLwc, snowMmH);
    std::printf("#   front: clear→fog@%.0fs→rain@%.0fs→snow@%.0fs→clear@%.0fs\n",
                0.20 * simSeconds, 0.35 * simSeconds, 0.55 * simSeconds,
                0.75 * simSeconds);
    std::printf("# %5s  %7s  %-9s  %8s  %8s  %9s\n",
                "t_s", "elev", "weather", "wxLoss", "snr_dB", "goodput");

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
                "avgGoodput=%.3f Mbps\n",
                (unsigned long)txP, (unsigned long)rxP,
                txP ? 100.0 * rxP / txP : 0.0,
                totalRx * 8.0 / simSeconds / 1e6);
    Simulator::Destroy();
    return 0;
}
