/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only
//
// thz-ntn-isl-traffic — REAL packet transmission over a sub-THz inter-
// satellite link between two CROSS-PLANE neighbours of a Starlink-class
// Walker shell (72 planes x 22 sats, 53 deg, 550 km). Both satellites fly
// genuine SGP4 orbits, so the inter-satellite range oscillates as the planes
// converge toward the high-latitude crossings and diverge at the equator —
// real constellation geometry, not a scripted drift.
//
// Audit fix (2026-06 protocol-fidelity audit): the old version mapped
// ThzNtnIslChannel::ComputeIslSnr_dB() through a sigmoid SnrToPer() onto a
// P2P RateErrorModel — no packet crossed a radio, and the satellites were
// ConstantPosition/ConstantVelocity placeholders. Here the ISL is a REAL
// mmwave NR link (NtnRealStackHelper: SpectrumPhy + MAC + RLC/PDCP + RRC)
// between the two satellites: the SINR is MEASURED off the PHY trace and
// tracks the live SGP4 range through the stack's own Friis loss. The
// module's analytic ComputeIslSnr_dB() is printed alongside the measurement
// so the formula-vs-measured gap is visible. The ThzNtnPropagationLossModel
// plug-in stays in the packet path and correctly reports ~0 dB excess —
// an exo-atmospheric ISL sees no molecular absorption (vacuum).
//
// The carrier is capped at 100 GHz by the 3GPP spectrum model; the 300 GHz
// D-band study stays in the analytic thz-ntn-isl link-budget tool. The high
// EIRP reflects UM-MIMO array gains at sub-THz (~50+ dBi per end).
//
// Quick test:  --simSeconds=60
#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/ntn-real-stack-helper.h"
#include "ns3/ntn-tr38811-mobility-model.h"
#include "ns3/sgp4-mobility-model.h"
#include "ns3/thz-ntn-isl-channel.h"
#include "ns3/thz-ntn-propagation-loss-model.h"
#include "ns3/walker-constellation.h"

#include <cmath>
#include <cstdio>
#include <string>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("ThzNtnIslTraffic");

int
main(int argc, char* argv[])
{
    double simSeconds = 60.0;
    double freqGHz = 100.0;    // sub-THz (3GPP spectrum model upper bound)
    double islEirpDbm = 110.0; // UM-MIMO ISL beam (closes ~189 dB FSPL at ~700 km)
    uint32_t numPlanes = 72;
    uint32_t satsPerPlane = 22;
    std::string outputDir = "thz-ntn-isl-traffic-output";
    std::string radio = "nr"; // radio backend: nr (FR1) or mmwave

    CommandLine cmd(__FILE__);
    cmd.AddValue("simSeconds", "Simulation duration (s)", simSeconds);
    cmd.AddValue("freqGHz", "ISL carrier frequency (GHz), capped at 100", freqGHz);
    cmd.AddValue("islEirpDbm", "ISL EIRP / gNB Tx power (dBm)", islEirpDbm);
    cmd.AddValue("radio", "Radio backend: nr (FR1) or mmwave", radio);
    cmd.AddValue("numPlanes", "Walker shell planes", numPlanes);
    cmd.AddValue("satsPerPlane", "Satellites per plane", satsPerPlane);
    cmd.AddValue("outputDir", "Output directory", outputDir);
    cmd.Parse(argc, argv);

    if (freqGHz > 100.0)
    {
        std::printf("# NOTE: 3GPP spectrum model caps the carrier at 100 GHz; "
                    "clamping %.0f -> 100 GHz\n",
                    freqGHz);
        freqGHz = 100.0;
    }

    std::printf("# thz-ntn-isl-traffic (REAL radio between two SGP4 satellites)\n");
    std::printf("#   shell: %u planes x %u sats, 53 deg, 550 km; link: plane0/slot0 <-> "
                "plane1/slot0\n",
                numPlanes, satsPerPlane);
    std::printf("#   sim=%.0fs freq=%.0fGHz EIRP=%.1fdBm\n", simSeconds, freqGHz,
                islEirpDbm);

    // Starlink-class Walker delta shell; the two ISL endpoints are the first
    // satellites of two ADJACENT planes (RAAN 5 deg apart), whose separation
    // breathes with latitude over the orbit.
    ns3::ntncon::WalkerConfig wcfg;
    wcfg.num_planes = numPlanes;
    wcfg.total_sats = numPlanes * satsPerPlane;
    wcfg.altitude_km = 550.0;
    wcfg.inclination_deg = 53.0;
    wcfg.epoch_unix_s = 1735689600.0;
    const auto elements = ns3::ntncon::WalkerConstellation::BuildDelta(wcfg);

    NodeContainer satA;
    satA.Create(1);
    NodeContainer satB;
    satB.Create(1);

    Ptr<ns3::ntncon::Sgp4MobilityModel> sgpA = CreateObject<ns3::ntncon::Sgp4MobilityModel>();
    sgpA->SetElements(elements[0]); // plane 0, slot 0
    Ptr<ns3::ntncon::Sgp4MobilityModel> sgpB = CreateObject<ns3::ntncon::Sgp4MobilityModel>();
    sgpB->SetElements(elements[satsPerPlane]); // plane 1, slot 0

    // Project BOTH orbits into one common local ENU frame at sat A's initial
    // sub-point so the helper's Friis loss sees the true inter-satellite range.
    double refLat, refLon, refAlt;
    sgpA->GetGeodetic(refLat, refLon, refAlt);
    Ptr<NtnEnuProjectionMobilityModel> enuA = CreateObject<NtnEnuProjectionMobilityModel>();
    enuA->SetSource(sgpA);
    enuA->SetReference(refLat, refLon, 0.0);
    satA.Get(0)->AggregateObject(enuA);
    Ptr<NtnEnuProjectionMobilityModel> enuB = CreateObject<NtnEnuProjectionMobilityModel>();
    enuB->SetSource(sgpB);
    enuB->SetReference(refLat, refLon, 0.0);
    satB.Get(0)->AggregateObject(enuB);

    NtnRealStackHelper rs;
    rs.SetRadioBackend(radio == "mmwave" ? NtnRealStackHelper::RadioBackend::Mmwave
                                         : NtnRealStackHelper::RadioBackend::Nr);
    if (radio != "mmwave")
    {
        rs.SetNumerology(1); // FR1 30 kHz SCS
    }
    rs.SetSimTime(Seconds(simSeconds));
    rs.SetOutputDir(outputDir);
    rs.SetRunTag("thz-ntn-isl-traffic");
    rs.SetCarrierFrequencyHz(freqGHz * 1e9);
    // NT-02: declared as CONDUCTED power at the array input. This carrier has
    // no TR 38.821 Set-1 reference in the toolkit, so the EIRP health gate
    // reports "not asserted" rather than certifying an uncalibrated budget.
    rs.SetSatConductedPowerDbm(islEirpDbm);
    rs.SetBackhaulDelay(MilliSeconds(1)); // on-board switch, not a ground feeder
    rs.Build(satA, satB); // sat A is the gNB end, sat B the UE end of the ISL

    // The THz plug-in stays in the packet path: for an exo-atmospheric ISL it
    // correctly contributes ~0 dB (vacuum — no O2/H2O column at 550 km).
    Ptr<ThzNtnPropagationLossModel> thz = CreateObject<ThzNtnPropagationLossModel>();
    thz->SetFrequency(freqGHz * 1e9);
    rs.AddExtraPropagationLoss(thz);

    rs.InstallTraffic(NtnRealStackHelper::TrafficProfile::EmbbStreaming,
                      Seconds(1.0), Seconds(simSeconds - 0.5));
    rs.EnableAiFlowMonitor("thz-ntn-isl-traffic");

    // The module's analytic ISL budget, evaluated on the SAME live geometry,
    // printed beside the measured SINR (formula vs measurement).
    Ptr<ThzNtnIslChannel> isl = CreateObject<ThzNtnIslChannel>();
    isl->SetAttribute("Frequency", DoubleValue(freqGHz * 1e9));
    isl->SetAttribute("TxPower", DoubleValue(30.0));
    isl->SetAttribute("TxGain", DoubleValue((islEirpDbm - 30.0) / 2.0));
    isl->SetAttribute("RxGain", DoubleValue((islEirpDbm - 30.0) / 2.0));
    isl->SetAttribute("Bandwidth", DoubleValue(50.0e6));

    std::printf("# %5s  %10s  %10s  %8s  %8s  %8s  %9s\n",
                "t_s", "sep_km", "formula", "molAbs", "sinr_dB", "tbler", "goodput");

    uint64_t lastRx = 0;
    rs.RegisterPeriodicCallback(
        Seconds(1.0),
        [&rs, thz, isl, enuA, enuB, &lastRx](Time now) {
            const double sepKm = enuA->GetDistanceFrom(enuB) / 1000.0;
            const double formulaSnr = isl->ComputeIslSnr_dB(enuA, enuB);
            const double sinr = rs.GetUeRecentSinrDb(0);
            const double tbler = rs.GetUeRecentTbler(0);
            const uint64_t rx = rs.GetUeRxBytes(0);
            const double mbps = (rx - lastRx) * 8.0 / 1e6;
            lastRx = rx;
            std::printf("  %5.1f  %10.1f  %10.2f  %8.2f  %8.2f  %8.3f  %9.3f\n",
                        now.GetSeconds(), sepKm, formulaSnr, thz->GetLastLossDb(),
                        sinr, tbler, mbps);
        });

    Simulator::Stop(Seconds(simSeconds));
    Simulator::Run();
    rs.Collect();
    rs.WriteHealthReport();

    std::printf("# === summary ===  measured ISL SINR=%.2f dB TBLER=%.4f "
                "throughput=%.3f Mbps (real packets between two SGP4 satellites)\n",
                rs.GetMeanDlSinrDb(), rs.GetMeanDlTbler(), rs.GetRxThroughputMbps());

    Simulator::Destroy();
    return 0;
}
