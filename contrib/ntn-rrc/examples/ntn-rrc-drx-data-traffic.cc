/*
 * SPDX-License-Identifier: GPL-2.0-only
 * Copyright (c) 2026 Muhammad Uzair (ns3-ntn-toolkit)
 *
 * ntn-rrc-drx-data-traffic — REAL UDP downlink traffic to a UE whose radio is
 * gated by the 3GPP NR-NTN RRC procedures of this module:
 *   - NtnSib19Broadcaster periodically broadcasts SIB19 (ephemeris + common TA)
 *   - NtnTimingAdvance computes the UE's total timing advance from the live
 *     UE<->satellite geometry (logged, drifting as the satellite moves)
 *   - NtnDrxStateMachine runs the connected-mode DRX cycle; on every state
 *     transition (its StateChange trace) the UE's receive link is opened
 *     (awake: Active / OnDuration) or closed (asleep: Short/Long sleep)
 *
 * Because the DRX duty cycle gates the receiver, the delivered goodput is
 * throttled to roughly the DRX on-fraction — the power-saving vs throughput
 * trade-off, measured on the real data plane. Compare --drxOn vs --drxOff.
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
#include "ns3/point-to-point-channel.h"
#include "ns3/point-to-point-helper.h"

#include "ns3/ntn-drx.h"
#include "ns3/ntn-sib19.h"
#include "ns3/ntn-timing-advance.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

using namespace ns3;
using namespace ns3::ntnrrc;

NS_LOG_COMPONENT_DEFINE("NtnRrcDrxDataTraffic");

namespace
{
constexpr double kC = 299792458.0;

Ptr<MobilityModel> g_ue, g_sat;
Ptr<NtnTimingAdvance> g_ta;
Ptr<NtnSib19Broadcaster> g_sib19;
Ptr<NtnDrxStateMachine> g_drx;
Ptr<RateErrorModel> g_em;
Ptr<PointToPointChannel> g_channel;
Ptr<PacketSink> g_sink;
uint64_t g_lastRx = 0;
double g_eirpDbm = 90.0;
double g_freqHz = 2.0e9;
double g_noiseDbm = -95.0;
double g_minElev = 10.0;
bool g_drxEnabled = true;
bool g_awake = true;     // updated by the DRX StateChange trace
double g_geomPer = 1.0;  // updated each second from geometry

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

void
ApplyEm()
{
    // Radio passes packets only when the geometry is usable AND (if DRX is
    // enabled) the UE is awake.
    const bool awake = g_drxEnabled ? g_awake : true;
    g_em->SetRate(awake ? g_geomPer : 1.0);
}

void
OnDrxStateChange(DrxState /*oldS*/, DrxState newS, Time /*at*/)
{
    g_awake = (newS == DrxState::Active || newS == DrxState::OnDuration);
    ApplyEm();
}

const char*
StateName(DrxState s)
{
    switch (s)
    {
    case DrxState::Active: return "ACTIVE";
    case DrxState::OnDuration: return "ON_DUR";
    case DrxState::ShortSleep: return "SHORT_SLP";
    case DrxState::LongSleep: return "LONG_SLP";
    case DrxState::AwaitingPass: return "AWAIT_PASS";
    }
    return "?";
}

void
GeoTick()
{
    const Vector u = g_ue->GetPosition();
    const Vector s = g_sat->GetPosition();
    const double elev = ElevDeg(u, s);
    const double range = Dist(u, s);
    const double snr = (g_eirpDbm - FsplDb(range, g_freqHz)) - g_noiseDbm;
    g_geomPer = (elev < g_minElev) ? 1.0 : SnrToPer(snr);
    g_channel->SetAttribute("Delay", TimeValue(Seconds(range / kC)));
    ApplyEm();

    // Exercise SIB19 + timing advance from the live geometry.
    g_sib19->RefreshNow();
    const Time ta = g_ta->ComputeTotalTa();

    const uint64_t tot = g_sink ? g_sink->GetTotalRx() : 0;
    const double mbps = (tot - g_lastRx) * 8.0 / 1e6;
    g_lastRx = tot;
    std::printf("  %6.1f  elev=%6.1f  slant=%7.0fkm  TA=%8.3fms  drx=%-10s  "
                "goodput=%8.3f\n",
                Simulator::Now().GetSeconds(), elev, range / 1000.0,
                ta.GetSeconds() * 1000.0,
                g_drxEnabled ? StateName(g_drx->GetState()) : "OFF", mbps);
    Simulator::Schedule(Seconds(1.0), &GeoTick);
}
} // namespace

int
main(int argc, char* argv[])
{
    double simSeconds = 60.0;
    double leoAltKm = 1200.0; // higher LEO so the pass lasts the whole sim
    double satSpeed = 7000.0;
    double freqGHz = 2.0;
    double dataRateMbps = 5.0;
    uint32_t packetBytes = 1200;
    double txPowerDbm = 33.0;
    double antennaGainDb = 57.0;
    bool drxEnabled = true;
    double drxLongCycleMs = 320.0;
    double drxOnDurationMs = 40.0;
    double linkCapacityMbps = 50.0;

    CommandLine cmd(__FILE__);
    cmd.AddValue("simSeconds", "Simulation duration (s)", simSeconds);
    cmd.AddValue("leoAltKm", "Satellite altitude (km)", leoAltKm);
    cmd.AddValue("satSpeed", "LEO ground-track speed (m/s)", satSpeed);
    cmd.AddValue("freqGHz", "Carrier frequency (GHz)", freqGHz);
    cmd.AddValue("dataRateMbps", "Offered downlink load (Mbps)", dataRateMbps);
    cmd.AddValue("packetBytes", "UDP payload size (bytes)", packetBytes);
    cmd.AddValue("txPowerDbm", "Satellite HPA output power (dBm)", txPowerDbm);
    cmd.AddValue("antennaGainDb", "Combined antenna gain (dB)", antennaGainDb);
    cmd.AddValue("drxOn", "Enable connected-mode DRX gating", drxEnabled);
    cmd.AddValue("drxLongCycleMs", "DRX long cycle (ms)", drxLongCycleMs);
    cmd.AddValue("drxOnDurationMs", "DRX on-duration (ms)", drxOnDurationMs);
    cmd.AddValue("linkCapacityMbps", "P2P link capacity (Mbps)", linkCapacityMbps);
    cmd.Parse(argc, argv);

    g_eirpDbm = txPowerDbm + antennaGainDb;
    g_freqHz = freqGHz * 1e9;
    g_drxEnabled = drxEnabled;

    NodeContainer nodes;
    nodes.Create(2); // 0=UE 1=satellite
    Ptr<ConstantPositionMobilityModel> ue =
        CreateObject<ConstantPositionMobilityModel>();
    ue->SetPosition(Vector(0, 0, 0));
    nodes.Get(0)->AggregateObject(ue);
    g_ue = ue;
    Ptr<ConstantVelocityMobilityModel> sat =
        CreateObject<ConstantVelocityMobilityModel>();
    sat->SetPosition(Vector(-0.3 * satSpeed * simSeconds, 0, leoAltKm * 1000.0));
    sat->SetVelocity(Vector(satSpeed, 0, 0));
    nodes.Get(1)->AggregateObject(sat);
    g_sat = sat;

    // RRC NTN procedures.
    g_ta = CreateObject<NtnTimingAdvance>();
    g_ta->SetUeMobility(ue);
    g_ta->SetSatelliteMobility(sat);
    g_ta->SetReferencePosition(Vector(0, 0, 0));

    g_sib19 = CreateObject<NtnSib19Broadcaster>();
    g_sib19->SetSatelliteMobility(sat);
    g_sib19->SetTimingAdvance(g_ta);
    g_sib19->SetReferencePosition(Vector(0, 0, 0));
    g_sib19->SetCellId(1);
    g_sib19->SetPeriod(MilliSeconds(160)); // SIB19 broadcast period

    g_drx = CreateObject<NtnDrxStateMachine>();
    NtnDrxConfig drxCfg;
    drxCfg.longCycle = MilliSeconds(drxLongCycleMs);
    drxCfg.onDuration = MilliSeconds(drxOnDurationMs);
    drxCfg.shortCycle = MilliSeconds(0); // disabled: clean long-cycle duty
    drxCfg.inactivityTimer = MilliSeconds(10);
    g_drx->SetConfig(drxCfg);
    g_drx->TraceConnectWithoutContext(
        "StateChange", MakeCallback(&OnDrxStateChange));

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
    ipv4.SetBase("10.80.1.0", "255.255.255.0");
    Ipv4InterfaceContainer ifaces = ipv4.Assign(devices);

    const uint16_t port = 7800;
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

    if (drxEnabled)
    {
        g_drx->Start();
    }

    FlowMonitorHelper fmHelper;
    Ptr<FlowMonitor> monitor = fmHelper.InstallAll();

    std::printf("# ntn-rrc-drx-data-traffic (SIB19 + timing-advance + DRX gating)\n");
    std::printf("#   sim=%.0fs alt=%.0fkm freq=%.1fGHz load=%.1fMbps EIRP=%.1fdBm "
                "DRX=%s (long=%.0fms on=%.0fms)\n",
                simSeconds, leoAltKm, freqGHz, dataRateMbps, g_eirpDbm,
                drxEnabled ? "on" : "off", drxLongCycleMs, drxOnDurationMs);

    Simulator::Schedule(Seconds(2.0), &GeoTick);
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
    const double dutyPct =
        drxEnabled ? 100.0 * drxOnDurationMs / drxLongCycleMs : 100.0;
    std::printf("# === summary ===  DRX=%s (on-duty≈%.0f%%)  txPackets=%lu "
                "rxPackets=%lu PDR=%.2f%% avgGoodput=%.3f Mbps\n",
                drxEnabled ? "on" : "off", dutyPct, (unsigned long)txP,
                (unsigned long)rxP, txP ? 100.0 * rxP / txP : 0.0,
                totalRx * 8.0 / simSeconds / 1e6);
    Simulator::Destroy();
    return 0;
}
