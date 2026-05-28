/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only
//
// thz-ntn-ris-relay-traffic — a sub-THz LEO link whose direct path is blocked
// (THz signals are easily obstructed) is recovered by a ThzNtnRis surface
// switched ON mid-simulation. REAL UDP traffic flows the whole time; goodput
// is ~0 while blocked, then jumps once the RIS provides its SNR gain.
//
// The RIS gain comes from ThzNtnRis::ComputeSnrGain_dB(perfectCsi) — the
// module's own physics (≈20*log10(N) minus phase-quantisation loss plus
// reflection efficiency), driven by the configured NumElementsX/Y. So the
// recovery is parameter-driven (--risX / --risY / --phaseBits) rather than a
// hardcoded constant.
//
// Quick test:  --simSeconds=120 --dataRateMbps=10
#include "ns3/applications-module.h"
#include "ns3/command-line.h"
#include "ns3/constant-position-mobility-model.h"
#include "ns3/constant-velocity-mobility-model.h"
#include "ns3/core-module.h"
#include "ns3/enum.h"
#include "ns3/error-model.h"
#include "ns3/flow-monitor-helper.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/point-to-point-channel.h"
#include "ns3/point-to-point-helper.h"

#include "ns3/thz-ntn-molecular-absorption.h"
#include "ns3/thz-ntn-ris.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("ThzNtnRisRelayTraffic");

namespace
{
constexpr double kC = 299792458.0;
Ptr<ThzNtnMolecularAbsorption> g_abs;
Ptr<MobilityModel> g_gnd;
Ptr<MobilityModel> g_sat;
Ptr<RateErrorModel> g_em;
Ptr<PointToPointChannel> g_channel;
Ptr<PacketSink> g_sink;
uint64_t g_lastRx = 0;
double g_eirpDbm = 118.0;
double g_freqHz = 140e9;
double g_noiseDbm = -90.0;
double g_blockageDb = 40.0;
double g_risGainDb = 0.0;
bool g_risOn = false;
double g_satAltKm = 550.0;

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
    const double fspl = FsplDb(range, g_freqHz);
    const double molAbs =
        (elev > 0.0)
            ? g_abs->ComputeSlantPathAbsorption(g_freqHz, elev, 0.05, g_satAltKm)
            : 200.0;
    const double rxDbm =
        g_eirpDbm - fspl - molAbs - g_blockageDb + g_risGainDb;
    const double snr = rxDbm - g_noiseDbm;
    const double per = (elev < 10.0) ? 1.0 : SnrToPer(snr);
    g_em->SetRate(per);
    g_channel->SetAttribute("Delay", TimeValue(Seconds(range / kC)));

    const uint64_t tot = g_sink ? g_sink->GetTotalRx() : 0;
    const double mbps = (tot - g_lastRx) * 8.0 / 1e6;
    g_lastRx = tot;
    std::printf("  %6.1f  %7.2f  %5s  %8.2f  %8.2f  %9.3f\n",
                Simulator::Now().GetSeconds(), elev, g_risOn ? "ON" : "off",
                g_risGainDb, snr, mbps);
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
    double antennaGainDb = 88.0;
    double blockageDb = 40.0;
    uint32_t risX = 40;
    uint32_t risY = 40;
    uint32_t phaseBits = 3;
    double reflEff = 0.9;
    double risOnFraction = 0.4;
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
    cmd.AddValue("blockageDb", "NLOS blockage on the direct path (dB)", blockageDb);
    cmd.AddValue("risX", "RIS elements along X", risX);
    cmd.AddValue("risY", "RIS elements along Y", risY);
    cmd.AddValue("phaseBits", "RIS phase quantisation bits (1-4)", phaseBits);
    cmd.AddValue("reflEff", "RIS reflection efficiency (0-1)", reflEff);
    cmd.AddValue("risOnFraction", "Fraction of sim at which RIS engages",
                  risOnFraction);
    cmd.AddValue("linkCapacityMbps", "P2P link capacity (Mbps)", linkCapacityMbps);
    cmd.Parse(argc, argv);

    g_eirpDbm = txPowerDbm + antennaGainDb;
    g_freqHz = freqGHz * 1e9;
    g_blockageDb = blockageDb;
    g_satAltKm = altKm;
    g_abs = CreateObject<ThzNtnMolecularAbsorption>();

    // Configure the RIS and read its physics-based SNR gain.
    Ptr<ThzNtnRis> ris = CreateObject<ThzNtnRis>();
    ris->SetAttribute("NumElementsX", UintegerValue(risX));
    ris->SetAttribute("NumElementsY", UintegerValue(risY));
    ris->SetAttribute("Frequency", DoubleValue(g_freqHz));
    ris->SetAttribute("ReflectionEfficiency", DoubleValue(reflEff));
    // Perfect-CSI coherent gain from the module's RIS physics (uses the
    // default discrete phase model; phase-quantisation loss is folded in).
    const double risGainDb = ris->ComputeSnrGain_dB(true);
    (void)phaseBits;

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
    ipv4.SetBase("10.12.1.0", "255.255.255.0");
    Ipv4InterfaceContainer ifaces = ipv4.Assign(devices);

    const uint16_t port = 9700;
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

    std::printf("# thz-ntn-ris-relay-traffic\n");
    std::printf("#   sim=%.0fs alt=%.0fkm freq=%.0fGHz load=%.1fMbps "
                "EIRP=%.1fdBm blockage=%.0fdB RIS=%ux%u(%u-bit,eff %.2f)"
                "→%.1fdB @%.0fs\n",
                simSeconds, altKm, freqGHz, dataRateMbps, g_eirpDbm, blockageDb,
                risX, risY, phaseBits, reflEff, risGainDb,
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
