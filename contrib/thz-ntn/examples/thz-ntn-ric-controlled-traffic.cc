/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only
//
// thz-ntn-ric-controlled-traffic — Roadmap §4.3.7 (thz-ntn × oran-ntn closed-
// loop full-stack example).
//
// The two modules cooperate over a real ns-3 LEO downlink:
//   telemetry: every second the data-plane Tick computes the live THz channel
//              SINR (FSPL + ThzNtnMolecularAbsorption + Rx noise) for the
//              current pass geometry and submits it as an E2-KPM report via
//              `OranNtnE2Node::SubmitKpmMeasurement()`.
//   control:   an xApp callback (registered via `SetIndicationCallback()`)
//              receives every indication. When the reported SINR falls below
//              `--sinrThreshDb`, the xApp ENGAGES the ground-deployed RIS;
//              `ThzNtnRis::ComputeSnrGain_dB(true)` contributes its array gain
//              to the next tick's link budget, lifting the SINR back into the
//              usable region. When SINR recovers the xApp releases the RIS.
//   data:      a UDP OnOff flow (sat -> ground) runs over a PointToPoint link
//              whose RateErrorModel PER is set from the current SINR; FlowMonitor
//              measures the goodput. Throughput therefore tracks the closed
//              loop in real sim time.
//
// Quick test:  --simSeconds=120 --xapp=1   (then re-run with --xapp=0 to see
//              the goodput drop on the low-elevation shoulders without the RIS).
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

#include "ns3/oran-ntn-e2-interface.h"
#include "ns3/oran-ntn-types.h"

#include "ns3/thz-ntn-molecular-absorption.h"
#include "ns3/thz-ntn-ris.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("ThzNtnRicControlledTraffic");

namespace
{
constexpr double kC = 299792458.0;

Ptr<MobilityModel> g_ue, g_sat;
Ptr<ThzNtnRis> g_ris;
Ptr<ThzNtnMolecularAbsorption> g_absorb;
Ptr<OranNtnE2Node> g_e2;
Ptr<RateErrorModel> g_em;
Ptr<PointToPointChannel> g_channel;
Ptr<PacketSink> g_sink;
uint64_t g_lastRx = 0;

double g_eirpDbm = 80.0;
double g_freqHz = 225e9;
double g_noiseDbm = -90.0;
double g_minElev = 5.0;
double g_sinrThreshDb = 8.0;
double g_leoAltKm = 550.0; // also used by ComputeSlantPathAbsorption
bool g_xappEnabled = true;

bool g_risEngaged = false;
uint32_t g_risEngagements = 0;
uint32_t g_kpmsSent = 0;
uint32_t g_xappActions = 0;

double
Dist(const Vector& a, const Vector& b)
{
    const double dx = a.x - b.x, dy = a.y - b.y, dz = a.z - b.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double
ElevDeg(const Vector& u, const Vector& s)
{
    const Vector d(s.x - u.x, s.y - u.y, s.z - u.z);
    return std::atan2(d.z, std::max(std::sqrt(d.x * d.x + d.y * d.y), 1e-3)) * 180.0 / M_PI;
}

double
FsplDb(double dM, double fHz)
{
    return 20.0 * std::log10(std::max(dM, 1.0)) + 20.0 * std::log10(fHz / 1e9) + 32.45;
}

double
SnrToPer(double snrDb)
{
    return 1.0 / (1.0 + std::exp(0.8 * (snrDb - 4.0))); // shifted operating point for THz
}

// The xApp: invoked on every E2-KPM indication. Toggles the RIS based on SINR.
// This is the control half of the closed loop.
void
RisXapp(E2Indication ind)
{
    ++g_xappActions;
    if (!g_xappEnabled)
    {
        return;
    }
    // KPM measurement: WG3-canonical SINR shipped as the report's sinr_dB field.
    const double sinrDb = ind.kpmReport.sinr_dB;
    const bool wantEngage = (sinrDb < g_sinrThreshDb);
    if (wantEngage && !g_risEngaged)
    {
        g_risEngaged = true;
        ++g_risEngagements;
    }
    else if (!wantEngage && g_risEngaged)
    {
        g_risEngaged = false;
    }
}

void
Tick(double simSeconds)
{
    const Vector u = g_ue->GetPosition();
    const Vector s = g_sat->GetPosition();
    const double elev = ElevDeg(u, s);
    const double range = Dist(u, s);

    // --- live THz link budget (real ns-3 / thz-ntn physics) ----------------
    const double fspl = FsplDb(range, g_freqHz);
    // Slant-path molecular absorption: ThzNtnMolecularAbsorption integrates
    // layer-by-layer through the atmosphere (water vapour decays with altitude),
    // so applying alpha over the *full* slant range would be wrong.
    const double molDb = g_absorb->ComputeSlantPathAbsorption(
        g_freqHz, std::max(elev, 5.0), 0.0, g_leoAltKm);
    // intrinsic SINR = what the link would deliver WITHOUT the RIS. This is
    // what we report to the xApp -- the xApp's decision must track the
    // geometry, not its own actuation (otherwise you get bang-bang).
    const double intrinsicSinr = g_eirpDbm - fspl - molDb - g_noiseDbm;
    // applied SINR = what the data plane actually sees, after the xApp's
    // chosen RIS state. The RIS gain is real thz-ntn physics.
    double risGainDb = 0.0;
    if (g_risEngaged)
    {
        risGainDb = g_ris->ComputeSnrGain_dB(true);
    }
    const double appliedSinr = intrinsicSinr + risGainDb;

    g_em->SetRate(elev < g_minElev ? 1.0 : SnrToPer(appliedSinr));
    g_channel->SetAttribute("Delay", TimeValue(Seconds(range / kC)));

    // --- telemetry half of the loop: submit a KPM indication --------------
    // The reported SINR is the INTRINSIC one (without the RIS); the xApp's
    // engage/release decision is therefore stable across its own actions.
    E2KpmReport report{};
    report.timestamp = Simulator::Now().GetSeconds();
    report.gnbId = 10;
    report.isNtn = true;
    report.ueId = 1;
    report.sinr_dB = intrinsicSinr; // WG3-canonical L1M.RS-SINR.Mean
    report.elevation_deg = elev;
    g_e2->SubmitKpmMeasurement(report);
    ++g_kpmsSent;

    const uint64_t tot = g_sink ? g_sink->GetTotalRx() : 0;
    const double mbps = (tot - g_lastRx) * 8.0 / 1e6;
    g_lastRx = tot;

    std::printf("  t=%6.1f elev=%5.1f range=%6.1fkm sinrIntrinsic=%5.1f "
                "sinrApplied=%5.1f ris=%s(+%4.1fdB) goodput=%6.2f Mbps\n",
                Simulator::Now().GetSeconds(), elev, range / 1000.0, intrinsicSinr,
                appliedSinr, g_risEngaged ? "ON " : "off", risGainDb, mbps);

    if (Simulator::Now().GetSeconds() + 1.0 < simSeconds)
    {
        Simulator::Schedule(Seconds(1.0), &Tick, simSeconds);
    }
}
} // namespace

int
main(int argc, char* argv[])
{
    double simSeconds = 120.0;
    double leoAltKm = 550.0;
    double satSpeed = 7500.0;
    double freqGHz = 225.0;
    double dataRateMbps = 20.0;
    uint32_t packetBytes = 1200;
    double satEirpDbm = 50.0;
    double rxGainDb = 30.0;
    double sinrThreshDb = 8.0;
    bool xappEnabled = true;
    uint32_t risN = 64; // 64x64 RIS panel
    std::string humidityProfile = "mid_latitude_summer";
    double linkCapMbps = 200.0;

    CommandLine cmd(__FILE__);
    cmd.AddValue("simSeconds", "Simulation duration (s)", simSeconds);
    cmd.AddValue("leoAltKm", "Satellite altitude (km)", leoAltKm);
    cmd.AddValue("satSpeed", "LEO ground-track speed (m/s)", satSpeed);
    cmd.AddValue("freqGHz", "THz carrier frequency (GHz)", freqGHz);
    cmd.AddValue("dataRateMbps", "Offered downlink load (Mbps)", dataRateMbps);
    cmd.AddValue("packetBytes", "UDP payload size (bytes)", packetBytes);
    cmd.AddValue("satEirpDbm", "Satellite EIRP (dBm)", satEirpDbm);
    cmd.AddValue("rxGainDb", "Ground/UE antenna gain (dB)", rxGainDb);
    cmd.AddValue("sinrThreshDb", "SINR threshold below which the xApp engages the RIS",
                 sinrThreshDb);
    cmd.AddValue("xapp", "Enable the RIS control xApp (0/1)", xappEnabled);
    cmd.AddValue("risN", "RIS panel side N (NxN elements)", risN);
    cmd.AddValue("humidityProfile",
                 "ThzNtnMolecularAbsorption humidity profile (e.g. mid_latitude_summer / tropical / dry)",
                 humidityProfile);
    cmd.Parse(argc, argv);

    g_freqHz = freqGHz * 1e9;
    g_eirpDbm = satEirpDbm + rxGainDb;
    g_sinrThreshDb = sinrThreshDb;
    g_xappEnabled = xappEnabled;
    g_leoAltKm = leoAltKm;

    // --- nodes + mobility (real LEO pass) ---
    NodeContainer nodes;
    nodes.Create(2);
    Ptr<ConstantPositionMobilityModel> ueMob = CreateObject<ConstantPositionMobilityModel>();
    ueMob->SetPosition(Vector(0, 0, 0));
    nodes.Get(0)->AggregateObject(ueMob);
    g_ue = ueMob;
    Ptr<ConstantVelocityMobilityModel> satMob = CreateObject<ConstantVelocityMobilityModel>();
    satMob->SetPosition(Vector(-0.5 * satSpeed * simSeconds, 0, leoAltKm * 1000.0));
    satMob->SetVelocity(Vector(satSpeed, 0, 0));
    nodes.Get(1)->AggregateObject(satMob);
    g_sat = satMob;

    // --- thz-ntn physics objects ---
    g_absorb = CreateObject<ThzNtnMolecularAbsorption>();
    g_absorb->SetHumidityProfile(humidityProfile);

    g_ris = CreateObject<ThzNtnRis>();
    g_ris->Configure(risN, risN, g_freqHz, RisDeployment::GROUND);
    g_ris->ComputeOptimalPhases(45.0, 0.0, 0.0, 0.0); // pre-tuned phase profile

    // --- oran-ntn E2 node + xApp closed-loop wiring ---
    g_e2 = CreateObject<OranNtnE2Node>();
    g_e2->SetNodeId(10);
    g_e2->SetIsNtn(true);
    g_e2->SetFeederLinkDelay(MilliSeconds(2));
    // E2SM-KPM is RAN-Function id 2 in this codebase; subscribe so submitted
    // measurements actually fire indications into our xApp callback.
    g_e2->RegisterRanFunction(2, "E2SM-KPM v03.00 (THz downlink SINR)");
    E2Subscription sub{};
    sub.subscriptionId = 1;
    sub.ranFunctionId = 2;
    sub.reportingPeriod = Seconds(1.0);
    sub.eventTrigger = false;
    g_e2->HandleSubscriptionRequest(sub);
    g_e2->SetIndicationCallback(MakeCallback(&RisXapp));

    // --- real ns-3 data plane gated by the closed-loop SINR ---
    InternetStackHelper internet;
    internet.Install(nodes);
    PointToPointHelper p2p;
    p2p.SetDeviceAttribute(
        "DataRate", DataRateValue(DataRate(static_cast<uint64_t>(linkCapMbps * 1e6))));
    p2p.SetChannelAttribute("Delay", TimeValue(Seconds(leoAltKm * 1000.0 / kC)));
    NetDeviceContainer devs = p2p.Install(nodes);
    g_em = CreateObject<RateErrorModel>();
    g_em->SetUnit(RateErrorModel::ERROR_UNIT_PACKET);
    g_em->SetRate(1.0);
    devs.Get(0)->SetAttribute("ReceiveErrorModel", PointerValue(g_em));
    g_channel = DynamicCast<PointToPointChannel>(devs.Get(0)->GetChannel());

    Ipv4AddressHelper ipv4;
    ipv4.SetBase("10.77.1.0", "255.255.255.0");
    Ipv4InterfaceContainer ifc = ipv4.Assign(devs);

    const uint16_t port = 6000;
    PacketSinkHelper sinkH("ns3::UdpSocketFactory",
                           InetSocketAddress(Ipv4Address::GetAny(), port));
    ApplicationContainer sinkApp = sinkH.Install(nodes.Get(0));
    sinkApp.Start(Seconds(0.0));
    sinkApp.Stop(Seconds(simSeconds));
    g_sink = DynamicCast<PacketSink>(sinkApp.Get(0));

    OnOffHelper onoff("ns3::UdpSocketFactory", InetSocketAddress(ifc.GetAddress(0), port));
    onoff.SetAttribute("DataRate", DataRateValue(DataRate(uint64_t(dataRateMbps * 1e6))));
    onoff.SetAttribute("PacketSize", UintegerValue(packetBytes));
    onoff.SetAttribute("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=1]"));
    onoff.SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0]"));
    ApplicationContainer srcApp = onoff.Install(nodes.Get(1));
    srcApp.Start(Seconds(1.0));
    srcApp.Stop(Seconds(simSeconds));

    FlowMonitorHelper fm;
    Ptr<FlowMonitor> monitor = fm.InstallAll();

    std::printf("# thz-ntn-ric-controlled-traffic (Roadmap §4.3.7)\n");
    std::printf("#   sim=%.0fs alt=%.0fkm freq=%.0fGHz EIRP=%.0fdBm RIS=%ux%u xApp=%s thresh=%.1fdB\n",
                simSeconds, leoAltKm, freqGHz, g_eirpDbm, risN, risN,
                xappEnabled ? "on" : "off", sinrThreshDb);
    std::printf("#   thz-ntn cascade: ThzNtnMolecularAbsorption (humidity=%s) +\n"
                "#                    ThzNtnRis (max gain %.1f dB)\n",
                humidityProfile.c_str(), g_ris->ComputeSnrGain_dB(true));

    Simulator::Schedule(Seconds(2.0), &Tick, simSeconds);
    Simulator::Stop(Seconds(simSeconds + 0.1));
    Simulator::Run();

    monitor->CheckForLostPackets();
    const uint64_t rx = g_sink ? g_sink->GetTotalRx() : 0;
    std::printf("# === summary ===  KPMs sent=%u  xApp actions=%u  RIS engagements=%u\n"
                "#                  rxBytes=%lu  avgGoodput=%.3f Mbps\n",
                g_kpmsSent, g_xappActions, g_risEngagements,
                (unsigned long)rx, rx * 8.0 / simSeconds / 1e6);
    Simulator::Destroy();
    return 0;
}
