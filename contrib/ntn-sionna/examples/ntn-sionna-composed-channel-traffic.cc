/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only
//
// ntn-sionna-composed-channel-traffic — Roadmap §4.2.8 (wire the ntn-sionna
// channel into the ns-3 propagation pipeline so modules compose).
//
// `NtnSionnaCascadeChannel` (Sionna RT multipath × ITU-R P.618/676/838 cascade)
// is a first-class ns3::PropagationLossModel. This example proves it composes in
// the standard ns-3 way and governs a real data plane:
//
//   1. CORRECT COMPOSITION — chain it with the built-in ns-3 small-scale fading
//      model from src/propagation via PropagationLossModel::SetNext():
//
//          txPwr --[ NtnSionnaCascadeChannel ]--[ NakagamiPropagationLossModel ]--> rxPwr
//
//      (Chaining two *full* path-loss models would double-count FSPL — the ns-3
//      idiom is one propagation model + delta fading models, which is exactly
//      what we do here.) The composed result is itself a PropagationLossModel,
//      so it drops straight into a MultiModelSpectrumChannel or an mmwave PHY.
//
//   2. INTERCHANGEABILITY with oran-ntn — `OranNtnChannelModel` (oran-ntn's TR
//      38.811 model) is also a PropagationLossModel for the same link; we
//      evaluate it each second alongside the composed model so the two are shown
//      as drop-in alternatives for the oran-ntn / mmwave channel slot.
//
//   3. REAL DATA PLANE — every second we ask the composed model for received
//      power via the ns-3 CalcRxPower() API, turn it into an SINR → packet-error
//      rate on a PointToPoint link, and run a UDP flow whose measured FlowMonitor
//      goodput is therefore governed by the composed channel. Nakagami fading
//      makes the SINR vary realistically tick-to-tick.
//
// Quick test:  --simSeconds=120 --dataRateMbps=20 --rainRateMmH=15
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
#include "ns3/propagation-module.h" // NakagamiPropagationLossModel (src/propagation)

#include "ns3/ntn-sionna-cascade-channel.h"
#include "ns3/oran-ntn-channel-model.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("NtnSionnaComposedChannelTraffic");

namespace
{
constexpr double kC = 299792458.0;
Ptr<MobilityModel> g_ue, g_sat;
Ptr<NtnSionnaCascadeChannel> g_composed; // chain head; SetNext(Nakagami)
Ptr<OranNtnChannelModel> g_oran;         // interchangeable drop-in model
Ptr<RateErrorModel> g_em;
Ptr<PointToPointChannel> g_channel;
Ptr<PacketSink> g_sink;
uint64_t g_lastRx = 0;
double g_effTxDbm = 102.0; // sat EIRP + ground dish gain (folded into tx power)
double g_noiseDbm = -96.0;
double g_minElev = 10.0;

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
SnrToPer(double snrDb)
{
    return 1.0 / (1.0 + std::exp(0.8 * (snrDb - 6.0)));
}

void
Tick(double simSeconds)
{
    const Vector u = g_ue->GetPosition();
    const Vector s = g_sat->GetPosition();
    const double elev = ElevDeg(u, s);
    const double range = Dist(u, s);

    // Received power from the COMPOSED ns-3 propagation chain
    // (NtnSionnaCascadeChannel -> NakagamiPropagationLossModel).
    const double rxComposed = g_composed->CalcRxPower(g_effTxDbm, g_sat, g_ue);
    // The interchangeable oran-ntn model evaluated for the same link.
    const double rxOran = g_oran->CalcRxPower(g_effTxDbm, g_sat, g_ue);

    const double sinr = rxComposed - g_noiseDbm;
    g_em->SetRate(elev < g_minElev ? 1.0 : SnrToPer(sinr));
    g_channel->SetAttribute("Delay", TimeValue(Seconds(range / kC)));

    const uint64_t tot = g_sink ? g_sink->GetTotalRx() : 0;
    const double mbps = (tot - g_lastRx) * 8.0 / 1e6;
    g_lastRx = tot;

    std::printf("  t=%6.1f  elev=%5.1f  range=%6.1fkm  rxComposed=%6.1f  rxOran=%6.1f  "
                "sinr=%5.1f  goodput=%7.3f Mbps\n",
                Simulator::Now().GetSeconds(), elev, range / 1000.0, rxComposed, rxOran,
                sinr, mbps);

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
    double leoAltKm = 600.0;
    double satSpeed = 7000.0;
    double freqGHz = 20.0;
    double dataRateMbps = 20.0;
    uint32_t packetBytes = 1200;
    double satEirpDbm = 62.0; // Ka-band LEO downlink EIRP
    double rxGainDb = 40.0;   // ground/UE dish gain
    double rainRateMmH = 5.0;
    std::string env = "Urban";
    double nakagamiM = 3.0; // Rician-like LOS fading near zenith
    double linkCapacityMbps = 120.0;

    CommandLine cmd(__FILE__);
    cmd.AddValue("simSeconds", "Simulation duration (s)", simSeconds);
    cmd.AddValue("leoAltKm", "Satellite altitude (km)", leoAltKm);
    cmd.AddValue("satSpeed", "LEO ground-track speed (m/s)", satSpeed);
    cmd.AddValue("freqGHz", "Carrier frequency (GHz)", freqGHz);
    cmd.AddValue("dataRateMbps", "Offered downlink load (Mbps)", dataRateMbps);
    cmd.AddValue("packetBytes", "UDP payload size (bytes)", packetBytes);
    cmd.AddValue("satEirpDbm", "Satellite downlink EIRP (dBm)", satEirpDbm);
    cmd.AddValue("rxGainDb", "Ground/UE antenna gain (dB)", rxGainDb);
    cmd.AddValue("rainRateMmH", "Rain rate for the oran-ntn P.838 term (mm/h)", rainRateMmH);
    cmd.AddValue("env", "Propagation environment (Urban/Suburban/Rural)", env);
    cmd.AddValue("nakagamiM", "Nakagami m-factor (LOS sharpness)", nakagamiM);
    cmd.AddValue("linkCapacityMbps", "P2P link capacity (Mbps)", linkCapacityMbps);
    cmd.Parse(argc, argv);

    g_effTxDbm = satEirpDbm + rxGainDb;

    // --- nodes + mobility (real LEO pass) ---
    NodeContainer nodes;
    nodes.Create(2);
    Ptr<ConstantPositionMobilityModel> ue = CreateObject<ConstantPositionMobilityModel>();
    ue->SetPosition(Vector(0, 0, 0));
    nodes.Get(0)->AggregateObject(ue);
    g_ue = ue;
    Ptr<ConstantVelocityMobilityModel> sat = CreateObject<ConstantVelocityMobilityModel>();
    sat->SetPosition(Vector(-0.5 * satSpeed * simSeconds, 0, leoAltKm * 1000.0));
    sat->SetVelocity(Vector(satSpeed, 0, 0));
    nodes.Get(1)->AggregateObject(sat);
    g_sat = sat;

    // --- COMPOSE: ntn-sionna cascade + ns-3 built-in Nakagami fading ---------
    Ptr<NtnSionnaCascadeChannel> sionna = CreateObject<NtnSionnaCascadeChannel>();
    sionna->SetFrequencyHz(freqGHz * 1e9); // FSPL + Sionna multipath + ITU cascade

    Ptr<NakagamiPropagationLossModel> nakagami = CreateObject<NakagamiPropagationLossModel>();
    nakagami->SetAttribute("m0", DoubleValue(nakagamiM)); // small-scale fading delta
    nakagami->SetAttribute("m1", DoubleValue(nakagamiM));
    nakagami->SetAttribute("m2", DoubleValue(nakagamiM));

    // Standard ns-3 composition: full propagation model -> delta fading model.
    sionna->SetNext(nakagami);
    g_composed = sionna;

    // --- oran-ntn TR 38.811 model: an interchangeable drop-in for the slot ---
    g_oran = CreateObject<OranNtnChannelModel>();
    g_oran->SetBand("Ka");
    g_oran->SetEnvironment(env);
    g_oran->SetAtmosphericAttenuation(true);
    g_oran->SetMarkovFading(true);
    g_oran->SetRainRate(rainRateMmH);

    // --- real ns-3 data plane gated by the composed channel ---
    InternetStackHelper internet;
    internet.Install(nodes);
    PointToPointHelper p2p;
    p2p.SetDeviceAttribute(
        "DataRate", DataRateValue(DataRate(static_cast<uint64_t>(linkCapacityMbps * 1e6))));
    p2p.SetChannelAttribute("Delay", TimeValue(Seconds(leoAltKm * 1000.0 / kC)));
    NetDeviceContainer devices = p2p.Install(nodes);
    g_em = CreateObject<RateErrorModel>();
    g_em->SetUnit(RateErrorModel::ERROR_UNIT_PACKET);
    g_em->SetRate(1.0);
    devices.Get(0)->SetAttribute("ReceiveErrorModel", PointerValue(g_em));
    g_channel = DynamicCast<PointToPointChannel>(devices.Get(0)->GetChannel());

    Ipv4AddressHelper ipv4;
    ipv4.SetBase("10.50.1.0", "255.255.255.0");
    Ipv4InterfaceContainer ifaces = ipv4.Assign(devices);

    const uint16_t port = 8080;
    PacketSinkHelper sinkH("ns3::UdpSocketFactory",
                           InetSocketAddress(Ipv4Address::GetAny(), port));
    ApplicationContainer sinkApp = sinkH.Install(nodes.Get(0));
    sinkApp.Start(Seconds(0.0));
    sinkApp.Stop(Seconds(simSeconds));
    g_sink = DynamicCast<PacketSink>(sinkApp.Get(0));

    OnOffHelper onoff("ns3::UdpSocketFactory", InetSocketAddress(ifaces.GetAddress(0), port));
    onoff.SetAttribute("DataRate", DataRateValue(DataRate(uint64_t(dataRateMbps * 1e6))));
    onoff.SetAttribute("PacketSize", UintegerValue(packetBytes));
    onoff.SetAttribute("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=1]"));
    onoff.SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0]"));
    ApplicationContainer srcApp = onoff.Install(nodes.Get(1));
    srcApp.Start(Seconds(1.0));
    srcApp.Stop(Seconds(simSeconds));

    FlowMonitorHelper fm;
    Ptr<FlowMonitor> monitor = fm.InstallAll();

    std::printf("# ntn-sionna-composed-channel-traffic (Roadmap §4.2.8)\n");
    std::printf("#   composed: NtnSionnaCascadeChannel -> NakagamiPropagationLossModel (m=%.1f)\n",
                nakagamiM);
    std::printf("#   drop-in alt: OranNtnChannelModel (TR 38.811)\n");
    std::printf("#   sim=%.0fs alt=%.0fkm freq=%.0fGHz effTx=%.0fdBm rain=%.0fmm/h env=%s\n",
                simSeconds, leoAltKm, freqGHz, g_effTxDbm, rainRateMmH, env.c_str());

    Simulator::Schedule(Seconds(2.0), &Tick, simSeconds);
    Simulator::Stop(Seconds(simSeconds + 0.1));
    Simulator::Run();

    monitor->CheckForLostPackets();
    const uint64_t rx = g_sink ? g_sink->GetTotalRx() : 0;
    std::printf("# === summary ===  rxBytes=%lu  avgGoodput=%.3f Mbps  "
                "lastCascadeAtten=%.2f dB\n",
                (unsigned long)rx, rx * 8.0 / simSeconds / 1e6,
                sionna->GetLastTotalAttenuationDb());
    Simulator::Destroy();
    return 0;
}
