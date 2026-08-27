/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * SPDX-License-Identifier: GPL-2.0-only
 * Copyright (c) 2026 Muhammad Uzair
 *
 * thz-ntn-native-300ghz-traffic - packets at TRUE THz (audit THZ-07).
 *
 * Until this example, the module's headline 200-400 GHz band had no scenario in
 * which a packet was ever transmitted. The seven examples that ran at 140-300
 * GHz contained no InternetStackHelper, NetDeviceContainer, PointToPointHelper,
 * FlowMonitor, OnOffHelper or PacketSink at all - they computed link budgets
 * into CSV. Every example that DID carry packets forced freqGHz = 100.0,
 * because the in-tree 3GPP spectrum model asserts 500 MHz <= f <= 100 GHz
 * (three-gpp-propagation-loss-model.cc:358), so the NR/mmwave bridge cannot be
 * driven above it. Every measured-plane THz result in the repo was therefore a
 * W-band result.
 *
 * This carries real IP traffic at 300 GHz by not using the 3GPP model at all.
 * The propagation chain is FriisPropagationLossModel -> ThzNtnPropagationLossModel:
 * free-space loss plus this module's own gaseous, weather, scintillation and
 * pointing terms, neither of which has a frequency cap. ThzNtnLinkErrorModel
 * puts that chain in the packet path, asking it per packet what the received
 * power is for the CURRENT geometry.
 *
 * HONEST SCOPE. This is not a SpectrumPhy: no per-subcarrier processing, no
 * MAC, no HARQ, no scheduler. It is a data plane whose delivery depends on real
 * THz physics rather than on a hardcoded wire, at a frequency the 3GPP bridge
 * refuses to model. Read the throughput as a link-limited transport figure, not
 * as an NR-at-THz result.
 *
 * Quick test:  --freqGhz=300 --simSeconds=30 --bandwidthGhz=1
 */

#include "ns3/applications-module.h"
#include "ns3/command-line.h"
#include "ns3/constant-position-mobility-model.h"
#include "ns3/constant-velocity-mobility-model.h"
#include "ns3/core-module.h"
#include "ns3/flow-monitor-helper.h"
#include "ns3/internet-module.h"
#include "ns3/ipv4-flow-classifier.h"
#include "ns3/point-to-point-module.h"
#include "ns3/propagation-loss-model.h"
#include "ns3/thz-ntn-link-error-model.h"
#include "ns3/thz-ntn-propagation-loss-model.h"

#include <cmath>
#include <iomanip>
#include <iostream>

using namespace ns3;
using namespace ns3::thzntn;

int
main(int argc, char* argv[])
{
    double freqGhz = 300.0;
    double simSeconds = 30.0;
    double altitudeKm = 550.0;
    double txPowerDbm = 30.0;
    double txGainDbi = 55.0;
    double rxGainDbi = 55.0;
    double bandwidthGhz = 1.0;
    double noiseFigureDb = 8.0;
    double decodeSnrDb = 0.0;
    double rainRateMmH = 0.0;

    CommandLine cmd(__FILE__);
    cmd.AddValue("freqGhz", "Carrier frequency (GHz) - NOT capped at 100", freqGhz);
    cmd.AddValue("simSeconds", "Simulation time (s)", simSeconds);
    cmd.AddValue("altitudeKm", "Satellite altitude (km)", altitudeKm);
    cmd.AddValue("txPowerDbm", "Transmit power (dBm)", txPowerDbm);
    cmd.AddValue("txGainDbi", "Transmit array gain (dBi)", txGainDbi);
    cmd.AddValue("rxGainDbi", "Receive array gain (dBi)", rxGainDbi);
    cmd.AddValue("bandwidthGhz", "Noise bandwidth (GHz)", bandwidthGhz);
    cmd.AddValue("noiseFigureDb", "Receiver noise figure (dB)", noiseFigureDb);
    cmd.AddValue("decodeSnrDb", "Minimum SNR for a decode (dB)", decodeSnrDb);
    cmd.AddValue("rainRateMmH", "Rain rate (mm/h), 0 = clear sky", rainRateMmH);
    cmd.Parse(argc, argv);

    const double freqHz = freqGhz * 1e9;

    NodeContainer nodes;
    nodes.Create(2);

    // Ground terminal and a satellite descending through the pass.
    Ptr<ConstantPositionMobilityModel> gnd = CreateObject<ConstantPositionMobilityModel>();
    gnd->SetPosition(Vector(0.0, 0.0, 0.0));
    nodes.Get(0)->AggregateObject(gnd);

    Ptr<ConstantVelocityMobilityModel> sat = CreateObject<ConstantVelocityMobilityModel>();
    sat->SetPosition(Vector(0.0, 0.0, altitudeKm * 1000.0));
    sat->SetVelocity(Vector(7560.0, 0.0, 0.0)); // real along-track motion
    nodes.Get(1)->AggregateObject(sat);

    // THE POINT: a propagation chain with no 100 GHz assertion in it.
    Ptr<FriisPropagationLossModel> friis = CreateObject<FriisPropagationLossModel>();
    friis->SetFrequency(freqHz);
    Ptr<ThzNtnPropagationLossModel> thz = CreateObject<ThzNtnPropagationLossModel>();
    thz->SetFrequency(freqHz);
    thz->SetRainRate(rainRateMmH);
    friis->SetNext(thz);

    PointToPointHelper p2p;
    p2p.SetDeviceAttribute("DataRate", StringValue("10Gbps"));
    p2p.SetChannelAttribute("Delay",
                            TimeValue(Seconds(altitudeKm * 1000.0 / 299792458.0)));
    NetDeviceContainer devs = p2p.Install(nodes);

    Ptr<ThzNtnLinkErrorModel> em = CreateObject<ThzNtnLinkErrorModel>();
    em->SetEndpoints(sat, gnd);
    em->SetPropagationChain(friis);
    em->SetTxPowerDbm(txPowerDbm);
    em->SetAntennaGainsDb(txGainDbi, rxGainDbi);
    em->SetBandwidthHz(bandwidthGhz * 1e9);
    em->SetNoiseFigureDb(noiseFigureDb);
    em->SetDecodeSnrDb(decodeSnrDb);
    devs.Get(0)->SetAttribute("ReceiveErrorModel", PointerValue(em));

    InternetStackHelper stack;
    stack.Install(nodes);
    Ipv4AddressHelper addr;
    addr.SetBase("10.60.0.0", "255.255.255.0");
    Ipv4InterfaceContainer ifs = addr.Assign(devs);

    const uint16_t port = 9000;
    PacketSinkHelper sink("ns3::UdpSocketFactory",
                          InetSocketAddress(Ipv4Address::GetAny(), port));
    ApplicationContainer sinkApp = sink.Install(nodes.Get(0));
    sinkApp.Start(Seconds(0.0));
    sinkApp.Stop(Seconds(simSeconds));

    OnOffHelper onoff("ns3::UdpSocketFactory",
                      InetSocketAddress(ifs.GetAddress(0), port));
    onoff.SetAttribute("DataRate", StringValue("500Mbps"));
    onoff.SetAttribute("PacketSize", UintegerValue(1400));
    onoff.SetAttribute("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=1]"));
    onoff.SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0]"));
    ApplicationContainer src = onoff.Install(nodes.Get(1));
    src.Start(Seconds(1.0));
    src.Stop(Seconds(simSeconds - 0.5));

    FlowMonitorHelper fmh;
    Ptr<FlowMonitor> fm = fmh.InstallAll();

    std::cout << "thz-ntn-native-300ghz-traffic: REAL PACKETS at " << freqGhz << " GHz\n"
              << "  propagation: Friis + ThzNtnPropagationLossModel (no 3GPP 100 GHz cap)\n"
              << "  initial SNR: " << std::fixed << std::setprecision(2) << em->CurrentSnrDb()
              << " dB  (rx " << em->CurrentRxPowerDbm() << " dBm, noise floor "
              << em->NoiseFloorDbm() << " dBm)\n";

    Simulator::Stop(Seconds(simSeconds));
    Simulator::Run();

    fm->CheckForLostPackets();
    uint64_t tx = 0;
    uint64_t rx = 0;
    double rxBytes = 0.0;
    for (const auto& kv : fm->GetFlowStats())
    {
        tx += kv.second.txPackets;
        rx += kv.second.rxPackets;
        rxBytes += kv.second.rxBytes;
    }
    const double thrMbps = (rxBytes * 8.0) / (simSeconds * 1e6);

    std::cout << "\n--- measured at " << freqGhz << " GHz ---\n"
              << "  packets tx/rx:   " << tx << " / " << rx
              << "  (PDR " << (tx ? 100.0 * rx / tx : 0.0) << "%)\n"
              << "  goodput:         " << thrMbps << " Mbps\n"
              << "  error model:     " << em->GetPacketsChecked() << " checked, "
              << em->GetPacketsCorrupted() << " corrupted\n"
              << "  final SNR:       " << em->CurrentSnrDb() << " dB "
              << (em->LinkCloses() ? "(link closes)" : "(LINK DOES NOT CLOSE)") << "\n"
              << "  -> this is a THz data plane, not an NR-at-THz result: no SpectrumPhy,\n"
              << "     no MAC, no HARQ. What it establishes is that packets can be carried\n"
              << "     at " << freqGhz << " GHz on real physics, which the 3GPP bridge refuses.\n";

    Simulator::Destroy();
    return 0;
}
