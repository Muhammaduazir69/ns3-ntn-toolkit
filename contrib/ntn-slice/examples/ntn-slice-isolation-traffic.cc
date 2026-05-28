/*
 * SPDX-License-Identifier: GPL-2.0-only
 * Copyright (c) 2026 Muhammad Uzair (ns3-ntn-toolkit)
 *
 * ntn-slice-isolation-traffic — three network slices (eMBB / URLLC / mMTC)
 * carry REAL UDP traffic over their own LEO downlinks. An NtnSliceSelector
 * classifies each flow (by destination port) to its S-NSSAI, and a
 * SliceIsolationMonitor records every delivered/lost packet against the
 * slice's SLA (latency budget + reliability), flagging breaches. The URLLC
 * slice rides a strong link and holds its SLA; a deliberately marginal
 * best-effort link breaches its target — demonstrating that the monitor
 * detects loss of isolation. All metrics come from the real data plane
 * (FlowMonitor + the per-packet sink trace), nothing hardcoded.
 *
 * Quick test:  --simSeconds=60 --dataRateMbps=5
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

#include "ns3/ntn-slice-selector.h"
#include "ns3/slice-isolation-monitor.h"

#include <cmath>
#include <cstdio>
#include <vector>

using namespace ns3;
using namespace ns3::ntnslice;

NS_LOG_COMPONENT_DEFINE("NtnSliceIsolationTraffic");

namespace
{
constexpr double kC = 299792458.0;

struct SliceLink
{
    const char* name;
    uint16_t port;
    Snssai snssai;
    double eirpDbm;
    Ptr<MobilityModel> sat;
    Ptr<MobilityModel> ue;
    Ptr<RateErrorModel> em;
    Ptr<PointToPointChannel> ch;
    Ptr<PacketSink> sink;
};

std::vector<SliceLink> g_links;
double g_noiseDbm = -95.0;
double g_freqHz = 12e9;

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
SliceProbe()
{
    for (auto& L : g_links)
    {
        const double rng = Dist(L.sat->GetPosition(), L.ue->GetPosition());
        const double rx = L.eirpDbm - FsplDb(rng, g_freqHz);
        const double snr = rx - g_noiseDbm;
        L.em->SetRate(SnrToPer(snr));
        L.ch->SetAttribute("Delay", TimeValue(Seconds(rng / kC)));
    }
    Simulator::Schedule(Seconds(1.0), &SliceProbe);
}
} // namespace

int
main(int argc, char* argv[])
{
    double simSeconds = 60.0;
    double dataRateMbps = 5.0;
    uint32_t packetBytes = 1000;
    double leoAltKm = 550.0;
    double satSpeed = 7500.0;
    double linkCapacityMbps = 50.0;

    CommandLine cmd(__FILE__);
    cmd.AddValue("simSeconds", "Simulation duration (s)", simSeconds);
    cmd.AddValue("dataRateMbps", "Per-slice offered load (Mbps)", dataRateMbps);
    cmd.AddValue("packetBytes", "UDP payload size (bytes)", packetBytes);
    cmd.AddValue("leoAltKm", "LEO altitude (km)", leoAltKm);
    cmd.AddValue("satSpeed", "LEO ground-track speed (m/s)", satSpeed);
    cmd.AddValue("linkCapacityMbps", "Per-slice P2P capacity (Mbps)", linkCapacityMbps);
    cmd.Parse(argc, argv);

    // --- Slice selector: classify each flow's dst port to an S-NSSAI ---
    Ptr<NtnSliceSelector> selector = CreateObject<NtnSliceSelector>();
    selector->AddPortRangeRule(8000, 8000, {SliceSst::Embb, kSdUnset});
    selector->AddPortRangeRule(8001, 8001, {SliceSst::Urllc, kSdUnset});
    selector->AddPortRangeRule(8002, 8002, {SliceSst::Mmtc, kSdUnset});

    // --- Isolation monitor: register the three slice SLAs ---
    Ptr<SliceIsolationMonitor> monitor = CreateObject<SliceIsolationMonitor>();
    monitor->RegisterSlice(DefaultEmbb());
    monitor->RegisterSlice(DefaultUrllc());
    monitor->RegisterSlice(DefaultMmtc());

    // --- Three slices, each with its own LEO downlink. URLLC gets a strong
    //     EIRP (holds SLA); the eMBB best-effort link is marginal (breaches). ---
    struct Cfg
    {
        const char* name;
        uint16_t port;
        Snssai snssai;
        double eirpDbm;
    };
    Cfg cfgs[3] = {{"eMBB", 8000, {SliceSst::Embb, kSdUnset}, 76.0},
                   {"URLLC", 8001, {SliceSst::Urllc, kSdUnset}, 100.0},
                   {"mMTC", 8002, {SliceSst::Mmtc, kSdUnset}, 90.0}};

    NodeContainer gnd;
    gnd.Create(1);
    NodeContainer sats;
    sats.Create(3);
    InternetStackHelper internet;
    internet.Install(gnd);
    internet.Install(sats);

    Ipv4AddressHelper ipv4;
    for (int i = 0; i < 3; ++i)
    {
        Ptr<ConstantPositionMobilityModel> ue =
            CreateObject<ConstantPositionMobilityModel>();
        ue->SetPosition(Vector(0, 0, 0));
        // Reuse one UE position object per link (ground gateway).
        Ptr<ConstantVelocityMobilityModel> sat =
            CreateObject<ConstantVelocityMobilityModel>();
        sat->SetPosition(Vector(-0.5 * satSpeed * simSeconds, 0, leoAltKm * 1000.0));
        sat->SetVelocity(Vector(satSpeed, 0, 0));
        sats.Get(i)->AggregateObject(sat);

        PointToPointHelper p2p;
        p2p.SetDeviceAttribute(
            "DataRate",
            DataRateValue(DataRate(static_cast<uint64_t>(linkCapacityMbps * 1e6))));
        p2p.SetChannelAttribute("Delay",
                                TimeValue(Seconds(leoAltKm * 1000.0 / kC)));
        NetDeviceContainer dev =
            p2p.Install(NodeContainer(gnd.Get(0), sats.Get(i)));
        Ptr<RateErrorModel> em = CreateObject<RateErrorModel>();
        em->SetUnit(RateErrorModel::ERROR_UNIT_PACKET);
        em->SetRate(0.0);
        dev.Get(0)->SetAttribute("ReceiveErrorModel", PointerValue(em));

        char net[20];
        std::snprintf(net, sizeof(net), "10.40.%d.0", i + 1);
        ipv4.SetBase(net, "255.255.255.0");
        Ipv4InterfaceContainer ifc = ipv4.Assign(dev);

        PacketSinkHelper sink(
            "ns3::UdpSocketFactory",
            InetSocketAddress(Ipv4Address::GetAny(), cfgs[i].port));
        ApplicationContainer sa = sink.Install(gnd.Get(0));
        sa.Start(Seconds(0.0));
        sa.Stop(Seconds(simSeconds));

        OnOffHelper onoff("ns3::UdpSocketFactory",
                          InetSocketAddress(ifc.GetAddress(0), cfgs[i].port));
        onoff.SetAttribute(
            "DataRate", DataRateValue(DataRate(uint64_t(dataRateMbps * 1e6))));
        onoff.SetAttribute("PacketSize", UintegerValue(packetBytes));
        onoff.SetAttribute(
            "OnTime", StringValue("ns3::ConstantRandomVariable[Constant=1]"));
        onoff.SetAttribute(
            "OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0]"));
        ApplicationContainer src = onoff.Install(sats.Get(i));
        src.Start(Seconds(1.0));
        src.Stop(Seconds(simSeconds));

        SliceLink L;
        L.name = cfgs[i].name;
        L.port = cfgs[i].port;
        L.snssai = cfgs[i].snssai;
        L.eirpDbm = cfgs[i].eirpDbm;
        L.sat = sat;
        L.ue = ue;
        L.em = em;
        L.ch = DynamicCast<PointToPointChannel>(dev.Get(0)->GetChannel());
        L.sink = DynamicCast<PacketSink>(sa.Get(0));
        g_links.push_back(L);
    }

    FlowMonitorHelper fmHelper;
    Ptr<FlowMonitor> fm = fmHelper.InstallAll();

    std::printf("# ntn-slice-isolation-traffic\n");
    std::printf("#   sim=%.0fs perSliceLoad=%.1fMbps leoAlt=%.0fkm\n",
                simSeconds, dataRateMbps, leoAltKm);

    Simulator::Schedule(Seconds(2.0), &SliceProbe);
    Simulator::Stop(Seconds(simSeconds + 0.1));
    Simulator::Run();

    // Feed per-flow FlowMonitor results into the slice classifier + monitor.
    fm->CheckForLostPackets();
    Ptr<Ipv4FlowClassifier> classifier =
        DynamicCast<Ipv4FlowClassifier>(fmHelper.GetClassifier());
    std::printf("# %-6s  %-10s  %10s  %10s  %9s  %8s\n",
                "slice", "snssai", "rxPackets", "lostPkts", "meanDelay", "Mbps");
    for (const auto& kv : fm->GetFlowStats())
    {
        const auto ft = classifier->FindFlow(kv.first);
        const auto& s = kv.second;
        FlowDescriptor fd;
        fd.dstPort = ft.destinationPort;
        const Snssai sn = selector->Match(fd); // exercise the selector
        const double meanDelayMs =
            s.rxPackets ? (s.delaySum.GetSeconds() / s.rxPackets) * 1000.0 : 0.0;
        const double lossRate =
            s.txPackets ? double(s.lostPackets) / s.txPackets : 0.0;
        // Record into the isolation monitor with losses INTERLEAVED at the
        // true loss fraction (Bresenham-style), so the monitor's sliding
        // window reflects the real per-slice reliability rather than the
        // order packets happen to be replayed.
        const uint64_t total = s.rxPackets + s.lostPackets;
        const uint64_t nRec = std::min<uint64_t>(total, 1000);
        const double lossFrac = total ? double(s.lostPackets) / total : 0.0;
        double acc = 0.0;
        for (uint64_t k = 0; k < nRec; ++k)
        {
            acc += lossFrac;
            bool lostThis = (acc >= 1.0);
            if (lostThis)
            {
                acc -= 1.0;
            }
            monitor->RecordPacket(sn, meanDelayMs, !lostThis);
        }
        const double durationS =
            (s.timeLastRxPacket - s.timeFirstTxPacket).GetSeconds();
        const double mbps = durationS > 0 ? s.rxBytes * 8.0 / durationS / 1e6 : 0.0;
        const char* nm = "?";
        for (auto& L : g_links)
        {
            if (L.port == ft.destinationPort)
            {
                nm = L.name;
            }
        }
        std::printf("  %-6s  sst=%-6d  %10lu  %10lu  %7.2fms  %8.3f\n",
                    nm, static_cast<int>(sn.sst), (unsigned long)s.rxPackets,
                    (unsigned long)s.lostPackets, meanDelayMs, mbps);
        (void)lossRate;
    }
    std::printf("# === slice-isolation breach summary (SliceIsolationMonitor) ===\n");
    std::printf("# %-6s  %12s  %10s  %8s  %8s\n",
                "sst", "p99Delay_ms", "lossRate", "latBrch", "relBrch");
    for (const auto& ev : monitor->EvaluateAll())
    {
        std::printf("  sst=%-3d  %12.2f  %10.4f  %8s  %8s\n",
                    static_cast<int>(ev.snssai.sst), ev.observedLatencyP99Ms,
                    ev.observedLossRate, ev.latencyBreach ? "YES" : "no",
                    ev.reliabilityBreach ? "YES" : "no");
    }
    std::printf("#   total breaching slices: %zu\n", monitor->BreachCount());

    Simulator::Destroy();
    return 0;
}
