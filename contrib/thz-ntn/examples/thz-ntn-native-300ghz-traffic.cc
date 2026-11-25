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
 * Wave 7C: the pass is no longer a ConstantVelocity straight line over a
 * ConstantPosition origin with a fixed 10 Gb/s wire. The satellite is an SGP4
 * member of an ntn-constellation Walker shell, the ground end a TR 38.811
 * static-class terminal under the pass (ntn-cho), the link a ThzNtnPacketLink
 * (live slant-range delay, P.676-13 Annex 1 gas + P.840-9 cloud at the P.834
 * apparent elevation, DVB-S2 ACM with per-packet BLER decode, sky noise), and
 * the traffic an ntn-traffic 5QI 2 video flow measured by NtnOranSink, the
 * NtnOranAiFlowMonitor KPM series and ns-3 FlowMonitor.
 *
 * Quick test:  --freqGhz=300 --simSeconds=30 --bandwidthGhz=1
 * Legacy decode (THZ-07 step at decodeSnrDb, fixed MODCOD): --decode=threshold
 */

#include "thz-ntn-pass-kit.h"

#include "ns3/ntn-dataset-writer.h"
#include "ns3/command-line.h"

#include <cmath>
#include <iostream>
#include <limits>

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
    // Wave 7C additions (defaults keep the old CLI meaning where one exists).
    std::string decode = "acm";
    double appRateMbps = 500.0;
    double groundOffsetKm = 150.0;
    std::string cloud = "sampled";
    std::string outputDir = "thz-ntn-native-300ghz-out";

    CommandLine cmd(__FILE__);
    cmd.AddValue("freqGhz", "Carrier frequency (GHz) - NOT capped at 100", freqGhz);
    cmd.AddValue("simSeconds", "Simulation time (s)", simSeconds);
    cmd.AddValue("altitudeKm", "Satellite altitude (km)", altitudeKm);
    cmd.AddValue("txPowerDbm", "Transmit power (dBm)", txPowerDbm);
    cmd.AddValue("txGainDbi", "Transmit array gain (dBi)", txGainDbi);
    cmd.AddValue("rxGainDbi", "Receive array gain (dBi)", rxGainDbi);
    cmd.AddValue("bandwidthGhz", "Occupied channel bandwidth (GHz)", bandwidthGhz);
    cmd.AddValue("noiseFigureDb", "Receiver noise figure (dB)", noiseFigureDb);
    cmd.AddValue("decodeSnrDb",
                 "Minimum SNR for a decode (dB), used with --decode=threshold", decodeSnrDb);
    cmd.AddValue("rainRateMmH", "Rain rate (mm/h), 0 = clear sky", rainRateMmH);
    cmd.AddValue("decode",
                 "acm (DVB-S2 ACM + BLER decode) or threshold (fixed QPSK 1/2, step at "
                 "decodeSnrDb)",
                 decode);
    cmd.AddValue("appRateMbps", "Offered 5QI 2 video load (Mb/s)", appRateMbps);
    cmd.AddValue("groundOffsetKm",
                 "Ground terminal offset from the sub-satellite track at mid-run (km)",
                 groundOffsetKm);
    cmd.AddValue("cloud", "P.840-9 cloud state: sampled | pXX | fixed:L | off", cloud);
    cmd.AddValue("outputDir", "Output directory", outputDir);
    cmd.Parse(argc, argv);

    thzkit::PassConfig cfg;
    cfg.runTag = "thz-ntn-native-300ghz-traffic";
    cfg.outputDir = outputDir;
    cfg.simSeconds = simSeconds;
    cfg.altitudeKm = altitudeKm;
    cfg.groundOffsetKm = groundOffsetKm;
    cfg.cloud = cloud;
    cfg.appRateMbps = appRateMbps;
    cfg.argc = argc;
    cfg.argv = argv;
    cfg.link.freqHz = freqGhz * 1e9;
    cfg.link.txPowerDbm = txPowerDbm;
    cfg.link.spaceGainDbi = txGainDbi;
    cfg.link.groundGainDbi = rxGainDbi;
    cfg.link.bandwidthHz = bandwidthGhz * 1e9;
    cfg.link.noiseFigureDb = noiseFigureDb;
    cfg.link.rainRateMmH = rainRateMmH;
    if (decode == "threshold")
    {
        cfg.link.acm = false;
        cfg.link.thresholdDecode = true;
        cfg.link.decodeSnrDb = decodeSnrDb;
    }

    std::cout << "thz-ntn-native-300ghz-traffic: REAL PACKETS at " << freqGhz << " GHz\n"
              << "  propagation: Friis + ThzNtnPropagationLossModel (no 3GPP 100 GHz cap)\n";
    // Globe scene of the measured pass (SGP4 satellite and the TR 38.811 terminal).
    const thzkit::PassResult r = thzkit::RunPass(
        cfg,
        [&outputDir](Ptr<thzntn::ThzNtnPacketLink>, Ptr<MobilityModel>, Ptr<MobilityModel>) {
            ns3::ntnobs::StartSceneFromNodeList(outputDir);
        });

    std::cout << "\n--- measured at " << freqGhz << " GHz ---\n"
              << "  packets tx/rx:   " << r.txPackets << " / " << r.rxPackets << "  (PDR "
              << 100.0 * r.pdr << "%)\n"
              << "  goodput:         " << r.goodputMbps << " Mbps (offered " << appRateMbps
              << ", mean ACM line rate " << r.meanLineRateMbps << ")\n"
              << "  one-way delay:   " << r.meanOwdMs << " ms mean, jitter " << r.jitterMs
              << " ms\n"
              << "  elevation:       " << r.minElevDeg << " .. " << r.maxElevDeg << " deg\n"
              << "  -> this is a THz data plane, not an NR-at-THz result: no SpectrumPhy,\n"
              << "     no HARQ. Packets are carried at " << freqGhz
              << " GHz on real physics, which the 3GPP bridge refuses.\n";
    // ntn-dataset@1: standard manifest + sim_health for this analytic example
    // (plan/MASTER_PLAN_GLOBE_DATASET_E2E_2026-11.md).
    ns3::ntnobs::WriteDatasetRecord(outputDir, "thz-ntn-native-300ghz-traffic", "analytic", argc, argv);
    return 0;
}
