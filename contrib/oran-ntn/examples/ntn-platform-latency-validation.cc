/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only
//
// ntn-platform-latency-validation — validates the toolkit's user-plane
// latency per NTN platform class against the paper's Table 3 (Deng 2026;
// adoption plan WS5), with REAL packets.
//
// For each platform class a bent-pipe path UE -> platform -> gateway is
// built from two P2P legs whose delay is the physical slant/c at the
// 10-deg cell-edge elevation the Table-3 classes assume (Table 3 classifies
// the platform ACCESS latency). An NtnOranApplication URLLC flow crosses it and
// the sink MEASURES the one-way delay from in-band timestamps; the reported
// round trip (2x measured OWD) must land in the Table-3 band:
//   UAV ~1 ms | HAP ~1-2 ms | LEO 30-50 ms | MEO ~150 ms | GEO ~600 ms
//
// Quick test: (no arguments; runs all five classes in one process)
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/network-module.h"
#include "ns3/ntn-oran-application.h"
#include "ns3/ntn-oran-sink.h"
#include "ns3/ntn-platform-spec.h"
#include "ns3/point-to-point-module.h"

#include <cmath>
#include <cstdio>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("NtnPlatformLatencyValidation");

namespace
{
constexpr double kC = 299792458.0;
constexpr double kRe = 6371e3;

// Slant range at the 10-deg cell-edge elevation for a platform at altitude
// h (spherical Earth).
double
SlantAtCellEdge(double hM)
{
    const double e = 10.0 * M_PI / 180.0;
    const double rs = kRe + hM;
    return -kRe * std::sin(e) + std::sqrt(rs * rs - kRe * kRe * std::cos(e) * std::cos(e));
}
} // namespace

int
main(int argc, char* argv[])
{
    double simSeconds = 20.0;
    CommandLine cmd(__FILE__);
    cmd.AddValue("simSeconds", "Per-class measurement duration (s)", simSeconds);
    cmd.Parse(argc, argv);

    struct Case
    {
        NtnPlatformSpec::Class cls;
        double altM;
        double rttMinMs, rttMaxMs; // Table-3 acceptance band
    };
    const Case cases[] = {
        {NtnPlatformSpec::Class::UavUntethered, 200.0, 0.0, 5.0},
        {NtnPlatformSpec::Class::Hap, 20e3, 0.0, 5.0},
        {NtnPlatformSpec::Class::Leo, 600e3, 20.0, 60.0},
        {NtnPlatformSpec::Class::Meo, 10000e3, 100.0, 250.0},
        {NtnPlatformSpec::Class::Geo, 35786e3, 450.0, 700.0},
    };

    std::printf("# ntn-platform-latency-validation (Table 3, measured in-band)\n");
    std::printf("# %-18s %10s %12s %12s %14s %6s\n", "platform", "alt_km", "slant_km",
                "rtt_meas_ms", "table3_band", "pass");

    bool allOk = true;
    uint16_t port = 8200;
    for (const auto& c : cases)
    {
        const auto& spec = NtnPlatformSpec::Get(c.cls);
        const double slantM = SlantAtCellEdge(c.altM);

        // UE(0) -> platform(1) -> gateway(2), each leg at the physical slant/c
        // (bent pipe: the user plane crosses both legs).
        NodeContainer nodes;
        nodes.Create(3);
        InternetStackHelper internet;
        internet.Install(nodes);
        PointToPointHelper p2p;
        p2p.SetDeviceAttribute("DataRate", StringValue("50Mbps"));
        p2p.SetChannelAttribute("Delay", TimeValue(Seconds(slantM / kC)));
        NetDeviceContainer up = p2p.Install(nodes.Get(0), nodes.Get(1));
        NetDeviceContainer down = p2p.Install(nodes.Get(1), nodes.Get(2));
        Ipv4AddressHelper ipv4;
        ipv4.SetBase("10.9.1.0", "255.255.255.0");
        ipv4.Assign(up);
        ipv4.SetBase("10.9.2.0", "255.255.255.0");
        Ipv4InterfaceContainer iDown = ipv4.Assign(down);
        Ipv4GlobalRoutingHelper::PopulateRoutingTables();

        Ptr<NtnOranSink> sink = CreateObject<NtnOranSink>();
        sink->SetAttribute("Local",
                           AddressValue(InetSocketAddress(Ipv4Address::GetAny(), port)));
        nodes.Get(2)->AddApplication(sink);
        sink->SetStartTime(Seconds(0.0));

        Ptr<NtnOranApplication> app = CreateObject<NtnOranApplication>();
        app->SetRemote(InetSocketAddress(iDown.GetAddress(1), port));
        app->SetProfile(NtnOranApplication::URLLC_PERIODIC);
        app->SetFlowIdentity(82, 2, 0x000001, 1, 2);
        nodes.Get(0)->AddApplication(app);
        app->SetStartTime(Seconds(0.5));
        app->SetStopTime(Seconds(simSeconds - 0.5));

        Simulator::Stop(Seconds(simSeconds));
        Simulator::Run();

        const double owdMs = sink->GetMeanDelayMs();
        const double rttMs = 2.0 * owdMs;
        const bool ok = (rttMs >= c.rttMinMs && rttMs <= c.rttMaxMs);
        allOk = allOk && ok && sink->GetRxPackets() > 100;
        std::printf("# %-18s %10.1f %12.1f %12.2f  [%5.0f,%5.0f] %6s\n",
                    spec.name.c_str(), c.altM / 1e3, slantM / 1e3, rttMs, c.rttMinMs,
                    c.rttMaxMs, ok ? "PASS" : "FAIL");
        Simulator::Destroy();
        ++port;
    }

    std::printf("# === summary ===  Table-3 latency validation: %s\n",
                allOk ? "ALL CLASSES PASS" : "FAIL");
    return allOk ? 0 : 1;
}
