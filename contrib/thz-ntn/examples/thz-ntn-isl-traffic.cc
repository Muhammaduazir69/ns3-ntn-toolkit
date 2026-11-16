/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only
//
// thz-ntn-isl-traffic: REAL packet transmission over a sub-THz inter-
// satellite link between two CROSS-PLANE neighbours of a Starlink-class
// Walker shell (72 planes x 22 sats, 53 deg, 550 km). Both satellites fly
// genuine SGP4 orbits, so the inter-satellite range oscillates as the planes
// converge toward the high-latitude crossings and diverge at the equator , 
// real constellation geometry, not a scripted drift.
//
// Audit fix (2026-06 protocol-fidelity audit): the old version mapped
// ThzNtnIslChannel::ComputeIslSnr_dB() through a sigmoid SnrToPer() onto a
// P2P RateErrorModel: no packet crossed a radio, and the satellites were
// ConstantPosition/ConstantVelocity placeholders. Here the ISL is a REAL
// mmwave NR link (NtnRealStackHelper: SpectrumPhy + MAC + RLC/PDCP + RRC)
// between the two satellites: the SINR is MEASURED off the PHY trace and
// tracks the live SGP4 range through the stack's own Friis loss. The
// module's analytic ComputeIslSnr_dB() is printed alongside the measurement
// so the formula-vs-measured gap is visible. The ThzNtnPropagationLossModel
// plug-in stays in the packet path and correctly reports ~0 dB excess , 
// an exo-atmospheric ISL sees no molecular absorption (vacuum).
//
// The carrier is capped at 100 GHz by the 3GPP spectrum model; the 300 GHz
// D-band study stays in the analytic thz-ntn-isl link-budget tool.
//
// Wave 2 M3 (16 Nov 2026), root-cause fixes:
//  - Declared terminals instead of a 110 dBm (100 kW) "conducted" knob: each
//    satellite carries a --islTxW transmitter (default 1 W) behind a --dishM
//    reflector (default 0.3 m, eta 0.6: 47.7 dBi at 100 GHz). EIRP = Pt + G
//    goes through SetSatEirpTotalDbm, the receive dish through
//    SetUeRxAntennaGain (split over the default 1x2 panel).
//  - No terrestrial fading between satellites: SetSmallScaleFading(false)
//    (deterministic LOS plane wave; TR 38.821 treats ISLs as free-space LOS).
//    The run used TR 38.901 UMi cluster fading before (SINR sd 34 dB at a
//    constant 621 km).
//  - No Earth-space excess terms on an ISL: SetTr38811ExcessLoss(false). The
//    legacy Ntn38811ExcessLossModel treated one satellite as a ground end and
//    drew its "P.618" scintillation, sigma = 0.5 sqrt(f/2) / sin(el)^(11/12),
//    at a near-zero "elevation", redrawn every second: that, not fading, is
//    what moved the measured SINR between -26 and +96 dB at a constant
//    621 km (measured with fading off: same 34.4 dB sd as with UMi fading).
//  - Numerology mu 3 (120 kHz) over 100 MHz, labeled as an extrapolation of
//    TS 38.211 Table 4.2-1 beyond the 71 GHz FR2-2 limit (TS 38.101-2); no 3GPP
//    air interface exists for a 100 GHz ISL. mu 1 at 100 GHz (the old value) is
//    an FR1 numerology.
// --legacyConductedDbm=110 restores the old operating point.
//
// Quick test:  --simSeconds=60
#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/nr-phy-mac-common.h"
#include "ns3/ntn-real-stack-helper.h"
#include "ns3/ntn-tr38811-mobility-model.h"
#include "ns3/sgp4-mobility-model.h"
#include "ns3/thz-ntn-isl-channel.h"
#include "ns3/thz-ntn-propagation-loss-model.h"
#include "ns3/walker-constellation.h"

#include "ns3/thz-ntn-aperture.h"

#include <cmath>
#include <cstdio>
#include <limits>
#include <string>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("ThzNtnIslTraffic");

int
main(int argc, char* argv[])
{
    double simSeconds = 60.0;
    double freqGHz = 100.0;    // sub-THz (3GPP spectrum model upper bound)
    double islTxW = 1.0;   // ISL transmitter output (W)
    double dishM = 0.3;    // ISL reflector diameter, both ends (m)
    double eta = 0.6;      // aperture efficiency
    double nfDb = 7.0;     // receiver noise figure (dB)
    double bandwidthMHz = 100.0;
    uint32_t numerology = 3;
    bool fading = false;
    double legacyConductedDbm = std::numeric_limits<double>::quiet_NaN();
    uint32_t numPlanes = 72;
    uint32_t satsPerPlane = 22;
    std::string outputDir = "thz-ntn-isl-traffic-output";
    std::string tbTrace;
    std::string radio = "nr"; // radio backend: nr (FR1) or mmwave

    CommandLine cmd(__FILE__);
    cmd.AddValue("simSeconds", "Simulation duration (s)", simSeconds);
    cmd.AddValue("freqGHz", "ISL carrier frequency (GHz), capped at 100", freqGHz);
    cmd.AddValue("islTxW", "ISL transmitter output power (W)", islTxW);
    cmd.AddValue("dishM", "ISL reflector diameter at both ends (m)", dishM);
    cmd.AddValue("eta", "Aperture efficiency", eta);
    cmd.AddValue("nfDb", "Receiver noise figure (dB)", nfDb);
    cmd.AddValue("bandwidthMHz", "Channel bandwidth (MHz)", bandwidthMHz);
    cmd.AddValue("numerology", "NR numerology (3 = 120 kHz, extrapolated)", numerology);
    cmd.AddValue("fading", "1 = 3GPP cluster fading (pre-Wave-2), 0 = free-space LOS", fading);
    cmd.AddValue("legacyConductedDbm",
                 "Reproduce the pre-Wave-2 point: this conducted power, mu 1, 20 MHz, fading",
                 legacyConductedDbm);
    cmd.AddValue("radio", "Radio backend: nr (FR1) or mmwave", radio);
    cmd.AddValue("numPlanes", "Walker shell planes", numPlanes);
    cmd.AddValue("satsPerPlane", "Satellites per plane", satsPerPlane);
    cmd.AddValue("outputDir", "Output directory", outputDir);
    cmd.AddValue("tbTrace", "Write every measured TB (time, SINR avg/min, RBs, MCS) to this CSV",
                 tbTrace);
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
    const bool legacy = std::isfinite(legacyConductedDbm);
    if (legacy)
    {
        numerology = 1;
        bandwidthMHz = 20.0;
        fading = true;
    }
    const double fHz = freqGHz * 1e9;
    const double gDish = ThzNtnAperture::GainDbi(dishM, fHz, eta);
    const double ptDbm = 10.0 * std::log10(islTxW * 1e3);
    const double eirpDbm = ptDbm + gDish;
    if (legacy)
    {
        std::printf("#   LEGACY: %.1f dBm conducted, mu 1, 20 MHz, UMi fading\n",
                    legacyConductedDbm);
    }
    else
    {
        std::printf("#   terminals: Pt %.2f W (%.2f dBm), dish %.2f m -> %.2f dBi each end (eta "
                    "%.2f), EIRP %.2f dBm, NF %.1f dB; %s\n",
                    islTxW, ptDbm, dishM, gDish, eta, eirpDbm, nfDb,
                    fading ? "3GPP cluster fading" : "free-space LOS (no small-scale fading)");
    }
    std::printf("#   air interface: %.0f GHz, mu %u over %.0f MHz: NR numerology EXTRAPOLATED "
                "above 71 GHz (TS 38.211 Table 4.2-1, TS 38.101-2); no 3GPP ISL air interface\n",
                freqGHz, numerology, bandwidthMHz);

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
    sgpA->SetElementsSgp4(elements[0]); // plane 0, slot 0
    Ptr<ns3::ntncon::Sgp4MobilityModel> sgpB = CreateObject<ns3::ntncon::Sgp4MobilityModel>();
    sgpB->SetElementsSgp4(elements[satsPerPlane]); // plane 1, slot 0

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
        rs.SetNumerology(numerology);
    }
    rs.SetSimTime(Seconds(simSeconds));
    rs.SetOutputDir(outputDir);
    rs.SetRunTag("thz-ntn-isl-traffic");
    rs.SetCarrierFrequencyHz(freqGHz * 1e9);
    if (legacy)
    {
        rs.SetSatConductedPowerDbm(legacyConductedDbm);
    }
    else
    {
        rs.SetBandwidthHz(bandwidthMHz * 1e6);
        rs.SetSatEirpTotalDbm(eirpDbm);
        rs.SetEirpBudgetDbm(eirpDbm, 0.5);
        rs.SetUeRxAntennaGain(gDish - 10.0 * std::log10(2.0));
        rs.SetUeNoiseFigureDb(nfDb);
        rs.SetSmallScaleFading(fading);
        rs.SetTr38811ExcessLoss(false);
    }
    rs.SetBackhaulDelay(MilliSeconds(1)); // on-board switch, not a ground feeder
    rs.Build(satA, satB); // sat A is the gNB end, sat B the UE end of the ISL

    // The THz plug-in stays in the packet path: for an exo-atmospheric ISL it
    // correctly contributes ~0 dB (vacuum: no O2/H2O column at 550 km).
    Ptr<ThzNtnPropagationLossModel> thz = CreateObject<ThzNtnPropagationLossModel>();
    thz->SetFrequency(freqGHz * 1e9);
    rs.AddExtraPropagationLoss(thz);

    FILE* tbf = nullptr;
    if (!tbTrace.empty() && radio != "mmwave")
    {
        tbf = std::fopen(tbTrace.c_str(), "w");
        if (tbf)
        {
            std::fprintf(tbf, "t_s,sinr_avg_db,sinr_min_db,rbs,mcs,tb_bytes,corrupt\n");
            Config::ConnectWithoutContextFailSafe(
                "/NodeList/*/DeviceList/*/$ns3::NrUeNetDevice/ComponentCarrierMapUe/*/NrUePhy/"
                "SpectrumPhy/RxPacketTraceUe",
                MakeBoundCallback(
                    +[](FILE* f, RxPacketTraceParams p) {
                        if (p.m_tbSize == 0)
                        {
                            return;
                        }
                        std::fprintf(f, "%.6f,%.3f,%.3f,%u,%u,%u,%d\n",
                                     Simulator::Now().GetSeconds(),
                                     10.0 * std::log10(std::max(p.m_sinr, 1e-12)),
                                     10.0 * std::log10(std::max(p.m_sinrMin, 1e-12)),
                                     p.m_rbAssignedNum, static_cast<unsigned>(p.m_mcs),
                                     p.m_tbSize, p.m_corrupt ? 1 : 0);
                    },
                    tbf));
        }
    }

    rs.InstallTraffic(NtnRealStackHelper::TrafficProfile::EmbbStreaming,
                      Seconds(1.0), Seconds(simSeconds - 0.5));
    rs.EnableAiFlowMonitor("thz-ntn-isl-traffic");

    // The module's analytic ISL budget, evaluated on the SAME live geometry,
    // printed beside the measured SINR (formula vs measurement).
    Ptr<ThzNtnIslChannel> isl = CreateObject<ThzNtnIslChannel>();
    isl->SetAttribute("Frequency", DoubleValue(freqGHz * 1e9));
    const double occHz = rs.GetBwpPrbs() * 12.0 * 15e3 * (1u << numerology);
    isl->SetAttribute("TxPower", DoubleValue(legacy ? 30.0 : ptDbm));
    isl->SetAttribute("TxGain", DoubleValue(legacy ? (legacyConductedDbm - 30.0) / 2.0 : gDish));
    isl->SetAttribute("RxGain", DoubleValue(legacy ? (legacyConductedDbm - 30.0) / 2.0 : gDish));
    isl->SetAttribute("Bandwidth", DoubleValue(legacy ? 50.0e6 : occHz));
    // Independent free-space budget on the same live range.
    const double noiseDbm = ThzNtnAperture::NoiseDbm(occHz, nfDb);

    std::printf("# %5s  %10s  %10s  %8s  %8s  %8s  %8s  %9s\n", "t_s", "sep_km", "formula",
                "budget", "molAbs", "sinr_dB", "tbler", "goodput");

    uint64_t lastRx = 0;
    rs.RegisterPeriodicCallback(
        Seconds(1.0),
        [&, thz, isl, enuA, enuB](Time now) {
            const double sepKm = enuA->GetDistanceFrom(enuB) / 1000.0;
            const double formulaSnr = isl->ComputeIslSnr_dB(enuA, enuB);
            const double budget = legacy ? std::numeric_limits<double>::quiet_NaN()
                                         : eirpDbm - ThzNtnAperture::FsplDb(sepKm * 1e3, fHz) +
                                               gDish - noiseDbm;
            const double sinr = rs.GetUeRecentSinrDb(0);
            const double tbler = rs.GetUeRecentTbler(0);
            const uint64_t rx = rs.GetUeRxBytes(0);
            const double mbps = (rx - lastRx) * 8.0 / 1e6;
            lastRx = rx;
            std::printf("  %5.1f  %10.1f  %10.2f  %8.2f  %8.2f  %8.2f  %8.3f  %9.3f\n",
                        now.GetSeconds(), sepKm, formulaSnr, budget, thz->GetLastLossDb(), sinr,
                        tbler, mbps);
        });

    Simulator::Stop(Seconds(simSeconds));
    Simulator::Run();
    rs.Collect();
    rs.WriteHealthReport();

    std::printf("# === summary ===  measured ISL SINR=%.2f dB TBLER=%.4f "
                "throughput=%.3f Mbps (real packets between two SGP4 satellites)\n",
                rs.GetMeanDlSinrDb(), rs.GetMeanDlTbler(), rs.GetRxThroughputMbps());

    Simulator::Destroy();
    if (tbf)
    {
        std::fclose(tbf);
    }
    return 0;
}
