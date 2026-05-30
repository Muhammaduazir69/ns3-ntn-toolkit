/*
 * SPDX-License-Identifier: GPL-2.0-only
 * Copyright (c) 2026 Muhammad Uzair (ns3-ntn-toolkit)
 *
 * ntn-cho-handover-traffic — REAL UDP downlink traffic to a ground UE that is
 * handed over between two passing LEO satellites by the actual NtnChoAlgorithm
 * (TTE-aware conditional handover). Unlike the analytical CHO examples, here
 * the CHO decision STEERS the data plane: each second the example computes the
 * per-satellite SINR from the live geometry, feeds it to the algorithm via
 * UpdateMeasurement(), asks SelectBestCandidate() which satellite should serve,
 * and opens that satellite's PointToPoint link (closing the other). When the
 * serving satellite changes, that is a real handover — the data plane follows
 * it, and FlowMonitor shows the UDP flow continuing across the handover with
 * only a brief dip. Nothing is hardcoded; the serving choice and the delivered
 * goodput fall out of the geometry + the CHO algorithm.
 *
 * Quick test:  --simSeconds=180 --dataRateMbps=5
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

#include "ns3/ntn-cho-algorithm.h"
#include "ns3/ntn-cho-helper.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <vector>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("NtnChoHandoverTraffic");

namespace
{
constexpr double kC = 299792458.0;

struct SatLink
{
    uint16_t cellId;
    Ptr<MobilityModel> mob;
    Ptr<RateErrorModel> em;
    Ptr<PointToPointChannel> ch;
};

Ptr<NtnChoAlgorithm> g_cho;
Ptr<MobilityModel> g_ue;
std::vector<SatLink> g_sats;
Ptr<PacketSink> g_sink;
double g_eirpDbm = 90.0;
double g_noiseDbm = -95.0;
double g_minElev = 5.0;
uint16_t g_serving = 0;
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

double g_freqHz = 2.0e9;

void
ChoTick()
{
    const Vector u = g_ue->GetPosition();
    // 1. Feed per-satellite geometry-derived SINR + gain to the CHO algorithm.
    std::vector<double> snr(g_sats.size());
    int bestByGeom = -1;
    double bestSnr = -1e9;
    for (std::size_t i = 0; i < g_sats.size(); ++i)
    {
        const Vector s = g_sats[i].mob->GetPosition();
        const double elev = ElevDeg(u, s);
        const double rx = g_eirpDbm - FsplDb(Dist(u, s), g_freqHz);
        snr[i] = rx - g_noiseDbm;
        // Beam gain proxy: scales with elevation (overhead = peak).
        const double gain = std::max(-20.0, (elev - 45.0) / 5.0);
        g_cho->UpdateMeasurement(g_sats[i].cellId, snr[i], gain);
        if (elev >= g_minElev && snr[i] > bestSnr)
        {
            bestSnr = snr[i];
            bestByGeom = static_cast<int>(i);
        }
    }
    g_cho->EvaluateConditions();

    // 2. Ask the CHO algorithm which cell should serve. Fall back to the
    //    best-visible-by-geometry cell if the algorithm has no admitted
    //    candidate yet (e.g. before the first TTE estimate).
    uint16_t chosen = g_cho->SelectBestCandidate();
    bool chosenVisible = false;
    for (std::size_t i = 0; i < g_sats.size(); ++i)
    {
        if (g_sats[i].cellId == chosen)
        {
            const double elev =
                ElevDeg(u, g_sats[i].mob->GetPosition());
            chosenVisible = (elev >= g_minElev);
        }
    }
    if (chosen == 0 || !chosenVisible)
    {
        chosen = (bestByGeom >= 0) ? g_sats[bestByGeom].cellId : 0;
    }

    // 3. Steer the data plane: open the serving cell's link, close others.
    for (std::size_t i = 0; i < g_sats.size(); ++i)
    {
        g_sats[i].ch->SetAttribute(
            "Delay",
            TimeValue(Seconds(Dist(u, g_sats[i].mob->GetPosition()) / kC)));
        if (g_sats[i].cellId == chosen)
        {
            g_sats[i].em->SetRate(SnrToPer(snr[i]));
        }
        else
        {
            g_sats[i].em->SetRate(1.0);
        }
    }

    if (chosen != g_serving)
    {
        ++g_handovers;
        g_cho->ExecuteHandover(chosen);
        std::printf("  %6.1f  HANDOVER  serving cell %u -> %u (snr %.1f dB)\n",
                    Simulator::Now().GetSeconds(), g_serving, chosen, bestSnr);
        g_serving = chosen;
    }

    const uint64_t tot = g_sink ? g_sink->GetTotalRx() : 0;
    const double mbps = (tot - g_lastRx) * 8.0 / 1e6;
    g_lastRx = tot;
    std::printf("  %6.1f  serving=%2u  bestSnr=%7.2f  goodput=%8.3f\n",
                Simulator::Now().GetSeconds(), g_serving, bestSnr, mbps);
    Simulator::Schedule(Seconds(1.0), &ChoTick);
}
} // namespace

int
main(int argc, char* argv[])
{
    double simSeconds = 180.0;
    double leoAltKm = 550.0;
    double satSpeed = 7500.0;
    double freqGHz = 2.0;
    double dataRateMbps = 5.0;
    uint32_t packetBytes = 1200;
    double txPowerDbm = 33.0;
    double antennaGainDb = 57.0;
    double tteMinSec = 5.0;
    double linkCapacityMbps = 50.0;

    CommandLine cmd(__FILE__);
    cmd.AddValue("simSeconds", "Simulation duration (s)", simSeconds);
    cmd.AddValue("leoAltKm", "LEO altitude (km)", leoAltKm);
    cmd.AddValue("satSpeed", "LEO ground-track speed (m/s)", satSpeed);
    cmd.AddValue("freqGHz", "Carrier frequency (GHz)", freqGHz);
    cmd.AddValue("dataRateMbps", "Offered downlink load (Mbps)", dataRateMbps);
    cmd.AddValue("packetBytes", "UDP payload size (bytes)", packetBytes);
    cmd.AddValue("txPowerDbm", "Satellite HPA output power (dBm)", txPowerDbm);
    cmd.AddValue("antennaGainDb", "Combined antenna gain (dB)", antennaGainDb);
    cmd.AddValue("tteMinSec", "Minimum TTE for CHO admission (s)", tteMinSec);
    cmd.AddValue("linkCapacityMbps", "P2P link capacity (Mbps)", linkCapacityMbps);
    cmd.Parse(argc, argv);

    g_eirpDbm = txPowerDbm + antennaGainDb;
    g_freqHz = freqGHz * 1e9;

    // --- CHO algorithm (TTE-aware) via the helper ---
    Ptr<NtnChoHelper> choHelper = CreateObject<NtnChoHelper>();
    choHelper->SetChoTriggerType(NtnChoAlgorithm::TRIGGER_TTE_AWARE);
    choHelper->SetCarrierFrequency(g_freqHz);
    choHelper->SetSatelliteTxPower(txPowerDbm);
    choHelper->SetTteMinimum(Seconds(tteMinSec));
    g_cho = choHelper->CreateChoAlgorithm();

    NodeContainer ueNode;
    ueNode.Create(1);
    NodeContainer sats;
    sats.Create(2);
    InternetStackHelper internet;
    internet.Install(ueNode);
    internet.Install(sats);

    Ptr<ConstantPositionMobilityModel> ue =
        CreateObject<ConstantPositionMobilityModel>();
    ue->SetPosition(Vector(0, 0, 0));
    ueNode.Get(0)->AggregateObject(ue);
    g_ue = ue;

    // Two satellites staggered so sat 1 sets while sat 2 rises → a handover.
    Ipv4AddressHelper ipv4;
    for (uint32_t i = 0; i < 2; ++i)
    {
        Ptr<ConstantVelocityMobilityModel> m =
            CreateObject<ConstantVelocityMobilityModel>();
        // Stagger so sat 1 crosses zenith at ~0.25*T and sat 2 at ~0.75*T;
        // their link-quality crossover near mid-sim forces a real handover.
        const double x0 = -(0.25 + 0.5 * i) * satSpeed * simSeconds;
        m->SetPosition(Vector(x0, 0, leoAltKm * 1000.0));
        m->SetVelocity(Vector(satSpeed, 0, 0));
        sats.Get(i)->AggregateObject(m);

        const uint16_t cellId = static_cast<uint16_t>(i + 1);
        g_cho->AddCandidateCell(cellId, i, 0);

        PointToPointHelper p2p;
        p2p.SetDeviceAttribute(
            "DataRate",
            DataRateValue(DataRate(static_cast<uint64_t>(linkCapacityMbps * 1e6))));
        p2p.SetChannelAttribute("Delay",
                                TimeValue(Seconds(leoAltKm * 1000.0 / kC)));
        NetDeviceContainer dev =
            p2p.Install(NodeContainer(ueNode.Get(0), sats.Get(i)));
        Ptr<RateErrorModel> em = CreateObject<RateErrorModel>();
        em->SetUnit(RateErrorModel::ERROR_UNIT_PACKET);
        em->SetRate(1.0);
        dev.Get(0)->SetAttribute("ReceiveErrorModel", PointerValue(em));

        char net[20];
        std::snprintf(net, sizeof(net), "10.50.%u.0", i + 1);
        ipv4.SetBase(net, "255.255.255.0");
        Ipv4InterfaceContainer ifc = ipv4.Assign(dev);

        SatLink sl;
        sl.cellId = cellId;
        sl.mob = m;
        sl.em = em;
        sl.ch = DynamicCast<PointToPointChannel>(dev.Get(0)->GetChannel());
        g_sats.push_back(sl);

        OnOffHelper onoff("ns3::UdpSocketFactory",
                          InetSocketAddress(ifc.GetAddress(0), 7500));
        onoff.SetAttribute(
            "DataRate", DataRateValue(DataRate(uint64_t(dataRateMbps * 1e6))));
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
        InetSocketAddress(Ipv4Address::GetAny(), 7500));
    ApplicationContainer sinkApp = sinkHelper.Install(ueNode.Get(0));
    sinkApp.Start(Seconds(0.0));
    sinkApp.Stop(Seconds(simSeconds));
    g_sink = DynamicCast<PacketSink>(sinkApp.Get(0));

    FlowMonitorHelper fmHelper;
    Ptr<FlowMonitor> monitor = fmHelper.InstallAll();

    std::printf("# ntn-cho-handover-traffic (TTE-aware CHO steering the data plane)\n");
    std::printf("#   sim=%.0fs leoAlt=%.0fkm freq=%.1fGHz load=%.1fMbps "
                "EIRP=%.1fdBm tteMin=%.0fs\n",
                simSeconds, leoAltKm, freqGHz, dataRateMbps, g_eirpDbm,
                tteMinSec);

    Simulator::Schedule(Seconds(2.0), &ChoTick);
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
    // Make-before-break: both candidate satellites carry a copy of the
    // stream, but only the CHO-selected serving link is open, so the
    // aggregate PDR is ~1/numCandidates by construction. The meaningful
    // metric is the SERVED-stream goodput, which stays ≈ the offered rate
    // across the hand-overs — that is the data-continuity result.
    std::printf("# === summary ===  handovers=%u (CHO-driven)  "
                "served-stream goodput=%.3f Mbps (offered %.1f)  "
                "aggregate over %u candidate copies: txPkts=%lu rxPkts=%lu "
                "PDR=%.1f%%\n",
                g_handovers, totalRx * 8.0 / simSeconds / 1e6, dataRateMbps,
                static_cast<unsigned>(g_sats.size()), (unsigned long)txP,
                (unsigned long)rxP, txP ? 100.0 * rxP / txP : 0.0);
    Simulator::Destroy();
    return 0;
}
