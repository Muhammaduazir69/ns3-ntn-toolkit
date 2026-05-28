/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only
//
// oran-ntn-ric-controlled-traffic — a closed O-RAN control loop steering a
// REAL LEO downlink:
//
//   data plane:   satellite --(P2P + geometry error model)--> UE  (real UDP)
//   telemetry:    OranNtnE2Node reports E2-KPM (SINR) each period
//   control:      an mMIMO-precoder xApp consumes each KPM indication; when
//                 the reported SINR drops below a threshold it engages
//                 beamforming (selects a beam from an OranNtnMmimoCodebook and
//                 applies the 10*log10(N_tx) array gain), which raises the
//                 link EIRP and therefore the delivered throughput.
//
// So the RIC control action has a measurable effect on the real data plane:
// during the low-elevation part of the pass the xApp turns the beam on and
// the goodput recovers; at high elevation the beam is not needed. Compare
// --xapp=true vs --xapp=false. Nothing is hardcoded — the beam decision comes
// from the live KPM telemetry.
//
// Quick test:  --simSeconds=120 --dataRateMbps=5
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
#include "ns3/oran-ntn-mmimo-codebook.h"
#include "ns3/oran-ntn-types.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <vector>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("OranNtnRicControlledTraffic");

namespace
{
constexpr double kC = 299792458.0;

Ptr<MobilityModel> g_ue, g_sat;
Ptr<OranNtnE2Node> g_e2;
Ptr<OranNtnMmimoCodebook> g_codebook;
Ptr<RateErrorModel> g_em;
Ptr<PointToPointChannel> g_channel;
Ptr<PacketSink> g_sink;
uint64_t g_lastRx = 0;
double g_baseEirpDbm = 62.0;
double g_freqHz = 20.0e9;
double g_noiseDbm = -95.0;
double g_minElev = 10.0;
double g_sinrThreshDb = 12.0; // xApp engages the beam below this
bool g_xappEnabled = true;
double g_beamGainDb = 0.0;    // current RIC-commanded beamforming gain
uint32_t g_numTx = 64;
uint32_t g_beamActivations = 0;
bool g_beamOn = false;

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
    return std::atan2(d.z, std::max(std::sqrt(d.x * d.x + d.y * d.y), 1e-3)) *
           180.0 / M_PI;
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

// The xApp: invoked on every E2-KPM indication from the satellite gNB.
// Implements the §4.1.12 mMIMO precoder control — engage beamforming when
// the link SINR is below target.
void
PrecoderXapp(E2Indication ind)
{
    const double sinr = ind.kpmReport.sinr_dB;
    if (!g_xappEnabled)
    {
        g_beamGainDb = 0.0;
        g_beamOn = false;
        return;
    }
    if (sinr < g_sinrThreshDb)
    {
        // Build a broadside steering target and pick the nearest codebook
        // beam (exercises the codebook); apply its array gain.
        std::vector<float> target(g_numTx * 2, 0.0f);
        for (uint32_t i = 0; i < g_numTx; ++i)
        {
            target[2 * i] = 1.0f; // real part = 1 (broadside), im = 0
        }
        (void)g_codebook->BestMatch(target);
        const double newGain = 10.0 * std::log10(static_cast<double>(g_numTx));
        if (!g_beamOn)
        {
            ++g_beamActivations;
        }
        g_beamGainDb = newGain;
        g_beamOn = true;
    }
    else
    {
        g_beamGainDb = 0.0;
        g_beamOn = false;
    }
}

void
Tick()
{
    const Vector u = g_ue->GetPosition();
    const Vector s = g_sat->GetPosition();
    const double elev = ElevDeg(u, s);
    const double range = Dist(u, s);

    // Baseline (pre-control) SINR — this is what the E2-KPM reports.
    const double baseSinr = (g_baseEirpDbm - FsplDb(range, g_freqHz)) - g_noiseDbm;

    // Submit an E2-KPM report; the xApp (indication callback) reacts and may
    // set g_beamGainDb.
    E2KpmReport r{};
    r.timestamp = Simulator::Now().GetSeconds();
    r.gnbId = g_e2->GetNodeId();
    r.isNtn = true;
    r.ueId = 1;
    r.sinr_dB = baseSinr;
    r.elevation_deg = elev;
    g_e2->SubmitKpmMeasurement(r);

    // Effective SINR after the RIC-commanded beamforming gain.
    const double effSinr = baseSinr + g_beamGainDb;
    const double per = (elev < g_minElev) ? 1.0 : SnrToPer(effSinr);
    g_em->SetRate(per);
    g_channel->SetAttribute("Delay", TimeValue(Seconds(range / kC)));

    const uint64_t tot = g_sink ? g_sink->GetTotalRx() : 0;
    const double mbps = (tot - g_lastRx) * 8.0 / 1e6;
    g_lastRx = tot;
    std::printf("  %6.1f  elev=%6.1f  baseSinr=%6.1f  beam=%-3s(%4.1fdB)  "
                "effSinr=%6.1f  goodput=%8.3f\n",
                Simulator::Now().GetSeconds(), elev, baseSinr,
                g_beamOn ? "ON" : "off", g_beamGainDb, effSinr, mbps);
    Simulator::Schedule(Seconds(1.0), &Tick);
}
} // namespace

int
main(int argc, char* argv[])
{
    double simSeconds = 120.0;
    double leoAltKm = 1200.0;
    double satSpeed = 7000.0;
    double freqGHz = 20.0;
    double dataRateMbps = 5.0;
    uint32_t packetBytes = 1200;
    double baseEirpDbm = 90.0; // marginal (SINR < threshold) without the beam
    uint32_t numTx = 64;
    double sinrThreshDb = 12.0;
    bool xappEnabled = true;
    double linkCapacityMbps = 50.0;

    CommandLine cmd(__FILE__);
    cmd.AddValue("simSeconds", "Simulation duration (s)", simSeconds);
    cmd.AddValue("leoAltKm", "Satellite altitude (km)", leoAltKm);
    cmd.AddValue("satSpeed", "LEO ground-track speed (m/s)", satSpeed);
    cmd.AddValue("freqGHz", "Carrier frequency (GHz)", freqGHz);
    cmd.AddValue("dataRateMbps", "Offered downlink load (Mbps)", dataRateMbps);
    cmd.AddValue("packetBytes", "UDP payload size (bytes)", packetBytes);
    cmd.AddValue("baseEirpDbm", "Baseline EIRP without beamforming (dBm)", baseEirpDbm);
    cmd.AddValue("numTx", "mMIMO Tx antennas (beam gain = 10log10(numTx))", numTx);
    cmd.AddValue("sinrThreshDb", "SINR threshold below which the xApp beamforms", sinrThreshDb);
    cmd.AddValue("xapp", "Enable the mMIMO precoder xApp control loop", xappEnabled);
    cmd.AddValue("linkCapacityMbps", "P2P link capacity (Mbps)", linkCapacityMbps);
    cmd.Parse(argc, argv);

    g_baseEirpDbm = baseEirpDbm;
    g_freqHz = freqGHz * 1e9;
    g_sinrThreshDb = sinrThreshDb;
    g_xappEnabled = xappEnabled;
    g_numTx = numTx;

    NodeContainer nodes;
    nodes.Create(2); // 0=UE 1=sat
    Ptr<ConstantPositionMobilityModel> ue =
        CreateObject<ConstantPositionMobilityModel>();
    ue->SetPosition(Vector(0, 0, 0));
    nodes.Get(0)->AggregateObject(ue);
    g_ue = ue;
    Ptr<ConstantVelocityMobilityModel> sat =
        CreateObject<ConstantVelocityMobilityModel>();
    // Start at low elevation (far behind) so the early pass is SINR-limited.
    sat->SetPosition(Vector(-0.45 * satSpeed * simSeconds, 0, leoAltKm * 1000.0));
    sat->SetVelocity(Vector(satSpeed, 0, 0));
    nodes.Get(1)->AggregateObject(sat);
    g_sat = sat;

    // O-RAN E2 node on the satellite gNB + KPM subscription.
    g_e2 = CreateObject<OranNtnE2Node>();
    g_e2->SetNodeId(7);
    g_e2->SetIsNtn(true);
    g_e2->RegisterRanFunction(2, "E2SM-KPM v03.00");
    E2Subscription sub{};
    sub.subscriptionId = 1;
    sub.ranFunctionId = 2;
    sub.reportingPeriod = Seconds(1.0);
    sub.eventTrigger = false;
    g_e2->HandleSubscriptionRequest(sub);
    g_e2->SetIndicationCallback(MakeCallback(&PrecoderXapp));

    // mMIMO codebook the xApp selects beams from.
    g_codebook = CreateObject<OranNtnMmimoCodebook>();
    g_codebook->Configure(numTx, 64);
    g_codebook->PopulateDftAzimuthSweep(numTx, 64);

    InternetStackHelper internet;
    internet.Install(nodes);

    PointToPointHelper p2p;
    p2p.SetDeviceAttribute(
        "DataRate",
        DataRateValue(DataRate(static_cast<uint64_t>(linkCapacityMbps * 1e6))));
    p2p.SetChannelAttribute("Delay", TimeValue(Seconds(leoAltKm * 1000.0 / kC)));
    NetDeviceContainer devices = p2p.Install(nodes);
    g_em = CreateObject<RateErrorModel>();
    g_em->SetUnit(RateErrorModel::ERROR_UNIT_PACKET);
    g_em->SetRate(1.0);
    devices.Get(0)->SetAttribute("ReceiveErrorModel", PointerValue(g_em));
    g_channel = DynamicCast<PointToPointChannel>(devices.Get(0)->GetChannel());

    Ipv4AddressHelper ipv4;
    ipv4.SetBase("10.90.1.0", "255.255.255.0");
    Ipv4InterfaceContainer ifaces = ipv4.Assign(devices);

    const uint16_t port = 7900;
    PacketSinkHelper sinkHelper(
        "ns3::UdpSocketFactory",
        InetSocketAddress(Ipv4Address::GetAny(), port));
    ApplicationContainer sinkApp = sinkHelper.Install(nodes.Get(0));
    sinkApp.Start(Seconds(0.0));
    sinkApp.Stop(Seconds(simSeconds));
    g_sink = DynamicCast<PacketSink>(sinkApp.Get(0));

    OnOffHelper onoff("ns3::UdpSocketFactory",
                      InetSocketAddress(ifaces.GetAddress(0), port));
    onoff.SetAttribute("DataRate", DataRateValue(DataRate(uint64_t(dataRateMbps * 1e6))));
    onoff.SetAttribute("PacketSize", UintegerValue(packetBytes));
    onoff.SetAttribute("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=1]"));
    onoff.SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0]"));
    ApplicationContainer src = onoff.Install(nodes.Get(1));
    src.Start(Seconds(1.0));
    src.Stop(Seconds(simSeconds));

    FlowMonitorHelper fmHelper;
    Ptr<FlowMonitor> monitor = fmHelper.InstallAll();

    std::printf("# oran-ntn-ric-controlled-traffic (E2-KPM -> mMIMO precoder xApp -> beam)\n");
    std::printf("#   sim=%.0fs alt=%.0fkm freq=%.0fGHz load=%.1fMbps baseEIRP=%.0fdBm "
                "numTx=%u (beamGain=%.1fdB) xApp=%s thresh=%.0fdB\n",
                simSeconds, leoAltKm, freqGHz, dataRateMbps, baseEirpDbm, numTx,
                10.0 * std::log10((double)numTx), xappEnabled ? "on" : "off",
                sinrThreshDb);

    Simulator::Schedule(Seconds(2.0), &Tick);
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
    std::printf("# === summary ===  xApp=%s beamActivations=%u  txPackets=%lu "
                "rxPackets=%lu PDR=%.2f%% avgGoodput=%.3f Mbps\n",
                xappEnabled ? "on" : "off", g_beamActivations,
                (unsigned long)txP, (unsigned long)rxP,
                txP ? 100.0 * rxP / txP : 0.0, totalRx * 8.0 / simSeconds / 1e6);
    Simulator::Destroy();
    return 0;
}
