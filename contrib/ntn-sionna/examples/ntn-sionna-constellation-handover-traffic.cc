/*
 * SPDX-License-Identifier: GPL-2.0-only
 * Copyright (c) 2026 Muhammad Uzair (ns3-ntn-toolkit)
 *
 * ntn-sionna-constellation-handover-traffic — a ground UE is served by a
 * small LEO constellation: several satellites cross the sky at staggered
 * times, and the UE always attaches to the one with the best SNR. REAL UDP
 * downlink traffic flows continuously; as satellites rise and set the
 * serving satellite changes (hand-over) and the data plane follows it.
 *
 * Each satellite has its own point-to-point link to the UE with its own
 * geometry-driven RateErrorModel. Every second the example evaluates the
 * per-satellite SNR from the NtnSionnaCascadeChannel, opens the best link
 * (SNR-driven PER) and closes the others (PER=1) — so only the serving
 * satellite delivers packets. Hand-over events are logged whenever the
 * serving satellite index changes. Nothing is hardcoded: which satellite
 * serves, and the delivered goodput, fall out of the live geometry.
 *
 * Quick test:  --simSeconds=180 --dataRateMbps=5 --numSats=3
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

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <vector>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("NtnSionnaConstellationHandoverTraffic");

namespace
{
constexpr double kC = 299792458.0;

struct SatLink
{
    Ptr<MobilityModel> mob;
    Ptr<NtnSionnaCascadeChannel> loss;
    Ptr<RateErrorModel> em;
    Ptr<PointToPointChannel> channel;
};

Ptr<MobilityModel> g_ue;
std::vector<SatLink> g_sats;
Ptr<PacketSink> g_sink;
double g_eirpDbm = 88.0;
double g_noiseDbm = -98.0;
double g_minElev = 5.0;
int g_serving = -1;
uint64_t g_lastRx = 0;
uint32_t g_handovers = 0;

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
HoProbe()
{
    const Vector u = g_ue->GetPosition();
    int best = -1;
    double bestSnr = -1e9;
    std::vector<double> snrs(g_sats.size());
    for (std::size_t i = 0; i < g_sats.size(); ++i)
    {
        const Vector s = g_sats[i].mob->GetPosition();
        const double elev = ElevDeg(u, s);
        const double rxDbm =
            g_sats[i].loss->CalcRxPower(g_eirpDbm, g_sats[i].mob, g_ue);
        const double snr = rxDbm - g_noiseDbm;
        snrs[i] = snr;
        g_sats[i].channel->SetAttribute(
            "Delay", TimeValue(Seconds(Dist(u, s) / kC)));
        if (elev >= g_minElev && snr > bestSnr)
        {
            bestSnr = snr;
            best = static_cast<int>(i);
        }
    }
    for (std::size_t i = 0; i < g_sats.size(); ++i)
    {
        g_sats[i].em->SetRate(static_cast<int>(i) == best ? SnrToPer(snrs[i])
                                                          : 1.0);
    }
    if (best != g_serving)
    {
        ++g_handovers;
        std::printf("  %6.1f  HANDOVER serving sat %d -> %d (snr %.1f dB)\n",
                    Simulator::Now().GetSeconds(), g_serving, best,
                    best >= 0 ? bestSnr : 0.0);
        g_serving = best;
    }
    const uint64_t tot = g_sink ? g_sink->GetTotalRx() : 0;
    const double mbps = (tot - g_lastRx) * 8.0 / 1e6;
    g_lastRx = tot;
    std::printf("  %6.1f  serving=%2d  bestSnr=%7.2f  goodput=%8.3f\n",
                Simulator::Now().GetSeconds(), best, best >= 0 ? bestSnr : 0.0,
                mbps);
    Simulator::Schedule(Seconds(1.0), &HoProbe);
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
    double antennaGainDb = 50.0;
    uint32_t numSats = 3;
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
    cmd.AddValue("numSats", "Number of satellites in the pass train", numSats);
    cmd.AddValue("linkCapacityMbps", "P2P link capacity (Mbps)", linkCapacityMbps);
    cmd.Parse(argc, argv);

    g_eirpDbm = txPowerDbm + antennaGainDb;

    NodeContainer ueNode;
    ueNode.Create(1);
    NodeContainer sats;
    sats.Create(numSats);
    InternetStackHelper internet;
    internet.Install(ueNode);
    internet.Install(sats);

    Ptr<ConstantPositionMobilityModel> ue =
        CreateObject<ConstantPositionMobilityModel>();
    ue->SetPosition(Vector(0, 0, 0));
    ueNode.Get(0)->AggregateObject(ue);
    g_ue = ue;

    const double spacing = satSpeed * simSeconds / numSats;
    for (uint32_t i = 0; i < numSats; ++i)
    {
        Ptr<ConstantVelocityMobilityModel> m =
            CreateObject<ConstantVelocityMobilityModel>();
        const double x0 = -0.5 * satSpeed * simSeconds - i * spacing;
        m->SetPosition(Vector(x0, 0, altKm * 1000.0));
        m->SetVelocity(Vector(satSpeed, 0, 0));
        sats.Get(i)->AggregateObject(m);

        Ptr<NtnSionnaChannel> b = CreateObject<NtnSionnaChannel>();
        b->SetFrequencyHz(freqHz);
        Ptr<NtnAtmosphericLossChain> ch =
            CreateObject<NtnAtmosphericLossChain>();
        ch->SetFrequencyHz(freqHz);
        Ptr<NtnSionnaCascadeChannel> loss =
            CreateObject<NtnSionnaCascadeChannel>();
        loss->SetSionnaChannel(b);
        loss->SetAtmosphericChain(ch);

        PointToPointHelper p2p;
        p2p.SetDeviceAttribute(
            "DataRate",
            DataRateValue(DataRate(static_cast<uint64_t>(linkCapacityMbps * 1e6))));
        p2p.SetChannelAttribute("Delay",
                                TimeValue(Seconds(altKm * 1000.0 / kC)));
        NetDeviceContainer dev =
            p2p.Install(NodeContainer(ueNode.Get(0), sats.Get(i)));
        Ptr<RateErrorModel> em = CreateObject<RateErrorModel>();
        em->SetUnit(RateErrorModel::ERROR_UNIT_PACKET);
        em->SetRate(1.0);
        dev.Get(0)->SetAttribute("ReceiveErrorModel", PointerValue(em));

        Ipv4AddressHelper ipv4;
        char net[16];
        std::snprintf(net, sizeof(net), "10.5.%u.0", i + 1);
        ipv4.SetBase(net, "255.255.255.0");
        Ipv4InterfaceContainer ifaces = ipv4.Assign(dev);

        SatLink sl;
        sl.mob = m;
        sl.loss = loss;
        sl.em = em;
        sl.channel = DynamicCast<PointToPointChannel>(dev.Get(0)->GetChannel());
        g_sats.push_back(sl);

        OnOffHelper onoff("ns3::UdpSocketFactory",
                          InetSocketAddress(ifaces.GetAddress(0), 9400));
        onoff.SetAttribute(
            "DataRate",
            DataRateValue(DataRate(static_cast<uint64_t>(dataRateMbps * 1e6))));
        onoff.SetAttribute("PacketSize", UintegerValue(packetBytes));
        onoff.SetAttribute(
            "OnTime", StringValue("ns3::ConstantRandomVariable[Constant=1]"));
        onoff.SetAttribute(
            "OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0]"));
        ApplicationContainer a = onoff.Install(sats.Get(i));
        a.Start(Seconds(1.0));
        a.Stop(Seconds(simSeconds));
    }

    PacketSinkHelper sinkHelper(
        "ns3::UdpSocketFactory",
        InetSocketAddress(Ipv4Address::GetAny(), 9400));
    ApplicationContainer sinkApp = sinkHelper.Install(ueNode.Get(0));
    sinkApp.Start(Seconds(0.0));
    sinkApp.Stop(Seconds(simSeconds));
    g_sink = DynamicCast<PacketSink>(sinkApp.Get(0));

    FlowMonitorHelper fmHelper;
    Ptr<FlowMonitor> monitor = fmHelper.InstallAll();

    std::printf("# ntn-sionna-constellation-handover-traffic\n");
    std::printf("#   sim=%.0fs alt=%.0fkm numSats=%u freq=%.1fGHz "
                "load=%.1fMbps EIRP=%.1fdBm\n",
                simSeconds, altKm, numSats, freqHz / 1e9, dataRateMbps,
                g_eirpDbm);

    Simulator::Schedule(Seconds(2.0), &HoProbe);
    Simulator::Stop(Seconds(simSeconds + 0.1));
    Simulator::Run();

    const uint64_t totalRx = g_sink ? g_sink->GetTotalRx() : 0;
    std::printf("# === summary ===  handovers=%u  totalRxBytes=%lu  "
                "avgGoodput=%.3f Mbps\n",
                g_handovers, (unsigned long)totalRx,
                totalRx * 8.0 / simSeconds / 1e6);
    Simulator::Destroy();
    return 0;
}
