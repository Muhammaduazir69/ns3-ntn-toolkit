/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only
//
// thz-ntn-leo-ground-downlink-traffic: END-TO-END packet transmission over a
// sub-THz LEO downlink on the measured 5G-LENA NR stack (NtnRealStackHelper:
// SpectrumPhy + MAC + HARQ + RLC/PDCP + RRC + EPC), SGP4 satellite.
//
// Wave 2 M3 (16 Nov 2026), root-cause fixes:
//  - Declared payload instead of a power knob. The link used to close with
//    115 dBm (316 kW) "conducted" power. It now closes because of a stated
//    satellite transmitter (--satTxW, default 10 W) behind a stated reflector
//    (--satDishM, 0.5 m) and a stated ground dish (--gndDishM, 0.6 m), with the
//    aperture law G = 10 log10(eta (pi D / lambda)^2), eta 0.6:
//    EIRP = Pt + G_sat (SetSatEirpTotalDbm), G_rx via SetUeRxAntennaGain (the
//    default 1x2 UE panel adds 10 log10 2, so each element carries
//    G_gnd - 3.01 dB and the panel totals G_gnd). Pointing loss uses the
//    ground dish's own 70 lambda / D beamwidth.
//  - Per-RB atmosphere: ThzNtnSpectrumAtmosphereModel through
//    AddSpectrumChannelLoss applies ITU-R P.676-13 line-by-line O2/H2O
//    absorption at EACH RB centre frequency of the live slant path, instead
//    of one scalar dB per link (--scalarAtmosphere=1 restores the scalar).
//  - The legacy Ntn38811ExcessLossModel is NOT chained (SetTr38811ExcessLoss
//    false): its gas term would double count P.676, and its "P.618"
//    scintillation draws sigma = 0.5 sqrt(f/2) / sin(el)^(11/12) dB, i.e.
//    3.5 dB rms at 100 GHz and zenith, redrawn independently every second
//    (measured: per-TB SINR stepping by up to 8 dB at each 1 s boundary on a
//    deterministic LOS link). Scintillation comes instead from
//    ThzNtnScintillation (ITU-R P.618-13 2.4.1 sigma with the aperture
//    averaging of the declared ground dish, AR(1) in simulated time) inside
//    the per-RB atmosphere model (--scintillation=0 turns it off).
//  - Fixed gateway dish, LOS: by default the spatial channel is the
//    deterministic LOS plane wave (SetSmallScaleFading(false)), as for a
//    TR 38.821 link budget; --fading=1 restores the 3GPP cluster model.
//  - Numerology: TS 38.211 Table 4.2-1 defines mu = 3 (120 kHz) for FR2,
//    which TS 38.101-2 stops at 71 GHz (FR2-2). No NR numerology exists at
//    100 GHz; mu = 3 with a 100 MHz channel is used as a labeled
//    extrapolation (printed in the banner).
//  - An independent budget column (EIRP - FSPL - gas - pointing + G_rx - N)
//    is printed beside the measured SINR every second.
//
// --legacyConductedDbm=115 reproduces the pre-Wave-2 operating point
// (conducted power, mu = 1 at the helper's 20 MHz, scalar absorption, fading).
//
// The 3GPP spectrum model caps the carrier at 100 GHz; the 183 GHz study stays
// in the analytic thz-ntn-leo-ground link-budget tool.
//
// Quick test:  --simSeconds=20
#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/nr-phy-mac-common.h"
#include "ns3/ntn-oran-application.h"
#include "ns3/ntn-real-stack-helper.h"
#include "ns3/ntn-tr38811-mobility-model.h"
#include "ns3/sgp4-mobility-model.h"
#include "ns3/thz-ntn-aperture.h"
#include "ns3/thz-ntn-pointing-loss-model.h"
#include "ns3/thz-ntn-propagation-loss-model.h"
#include "ns3/thz-ntn-spectrum-atmosphere-model.h"
#include "ns3/walker-constellation.h"

#include <cmath>
#include <cstdio>
#include <limits>
#include <string>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("ThzNtnLeoGroundDownlinkTraffic");

namespace
{

double
ElevDegEnu(const Vector& gnd, const Vector& sat)
{
    const double dx = sat.x - gnd.x;
    const double dy = sat.y - gnd.y;
    const double dz = sat.z - gnd.z;
    const double horiz = std::max(std::sqrt(dx * dx + dy * dy), 1e-3);
    return std::atan2(dz, horiz) * 180.0 / M_PI;
}

double
DistM(const Vector& a, const Vector& b)
{
    const double dx = a.x - b.x, dy = a.y - b.y, dz = a.z - b.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

} // namespace

int
main(int argc, char* argv[])
{
    double simSeconds = 60.0;
    double freqGHz = 100.0; // sub-THz / W-band (3GPP spectrum model upper bound)
    double satTxW = 10.0;   // satellite power amplifier output (W)
    double satDishM = 0.5;  // satellite reflector diameter (m)
    double gndDishM = 0.6;  // ground-station reflector diameter (m)
    double eta = 0.6;       // aperture efficiency
    double nfDb = 7.0;      // ground receiver noise figure (dB)
    double bandwidthMHz = 100.0;
    uint32_t numerology = 3;
    bool fading = false;
    bool scalarAtmosphere = false;
    bool pointing = true;
    bool scintillation = true;
    double offeredMbps = 5.0;
    double legacyConductedDbm = std::numeric_limits<double>::quiet_NaN();
    std::string outputDir = "thz-ntn-leo-ground-downlink-output";
    std::string tbTrace;
    std::string radio = "nr";

    CommandLine cmd(__FILE__);
    cmd.AddValue("simSeconds", "Simulation duration (s)", simSeconds);
    cmd.AddValue("freqGHz", "Carrier frequency (GHz), capped at 100", freqGHz);
    cmd.AddValue("satTxW", "Satellite transmitter output power (W)", satTxW);
    cmd.AddValue("satDishM", "Satellite reflector diameter (m)", satDishM);
    cmd.AddValue("gndDishM", "Ground-station reflector diameter (m)", gndDishM);
    cmd.AddValue("eta", "Aperture efficiency of both reflectors", eta);
    cmd.AddValue("nfDb", "Ground receiver noise figure (dB)", nfDb);
    cmd.AddValue("bandwidthMHz", "Channel bandwidth (MHz)", bandwidthMHz);
    cmd.AddValue("numerology", "NR numerology mu (3 = 120 kHz, extrapolated above 71 GHz)",
                 numerology);
    cmd.AddValue("fading", "1 = 3GPP cluster fading, 0 = LOS plane wave (default)", fading);
    cmd.AddValue("scalarAtmosphere", "1 = one scalar P.676 dB per link (pre-Wave-2)",
                 scalarAtmosphere);
    cmd.AddValue("scintillation", "1 = P.618 amplitude scintillation (default), 0 = off",
                 scintillation);
    cmd.AddValue("pointing", "1 = THz pointing-error loss in the path (default), 0 = off",
                 pointing);
    cmd.AddValue("offeredMbps", "Offered downlink CBR (Mb/s); above capacity = saturate",
                 offeredMbps);
    cmd.AddValue("legacyConductedDbm",
                 "Reproduce the pre-Wave-2 point: this conducted power, mu 1, 20 MHz, "
                 "scalar absorption, fading on",
                 legacyConductedDbm);
    cmd.AddValue("radio", "Radio backend: nr or mmwave", radio);
    cmd.AddValue("tbTrace", "Write every measured DL TB (time, SINR avg/min, RBs, MCS) to this CSV",
                 tbTrace);
    cmd.AddValue("outputDir", "Output directory", outputDir);
    cmd.Parse(argc, argv);

    const bool legacy = std::isfinite(legacyConductedDbm);
    if (legacy)
    {
        numerology = 1;
        bandwidthMHz = 20.0;
        fading = true;
        scalarAtmosphere = true;
    }
    if (radio == "mmwave")
    {
        scalarAtmosphere = true; // the spectrum seam is nr-only
    }
    if (freqGHz > 100.0)
    {
        std::printf("# NOTE: 3GPP spectrum model caps the carrier at 100 GHz; "
                    "clamping %.0f -> 100 GHz\n",
                    freqGHz);
        freqGHz = 100.0;
    }
    const double fHz = freqGHz * 1e9;
    const double gSat = ThzNtnAperture::GainDbi(satDishM, fHz, eta);
    const double gGnd = ThzNtnAperture::GainDbi(gndDishM, fHz, eta);
    const double ptDbm = 10.0 * std::log10(satTxW * 1e3);
    const double eirpDbm = ptDbm + gSat;
    const double bw3dB = ThzNtnAperture::Beamwidth3dBDeg(gndDishM, fHz);

    std::printf("# thz-ntn-leo-ground-downlink-traffic (measured NR, per-RB P.676 absorption)\n");
    if (legacy)
    {
        std::printf("#   LEGACY operating point: %.1f dBm conducted, mu 1, 20 MHz, scalar "
                    "absorption, fading on\n",
                    legacyConductedDbm);
    }
    else
    {
        std::printf("#   payload: Pt %.1f W (%.2f dBm), sat dish %.2f m -> %.2f dBi, EIRP %.2f dBm; "
                    "ground dish %.2f m -> G_rx %.2f dBi (eta %.2f), NF %.1f dB, "
                    "pointing beamwidth %.3f deg\n",
                    satTxW, ptDbm, satDishM, gSat, eirpDbm, gndDishM, gGnd, eta, nfDb, bw3dB);
    }
    std::printf("#   air interface: %.0f GHz, mu %u (%u kHz SCS) over %.0f MHz: TS 38.211 "
                "Table 4.2-1 numerology EXTRAPOLATED above the 71 GHz FR2-2 limit "
                "(TS 38.101-2); no NR numerology is defined at this carrier\n",
                freqGHz, numerology, 15u << numerology, bandwidthMHz);
    std::printf("#   channel: %s spatial channel, %s atmosphere\n",
                fading ? "3GPP cluster (fading)" : "LOS plane wave (no small-scale fading)",
                scalarAtmosphere ? "scalar" : "per-RB");

    NodeContainer satNodes;
    satNodes.Create(1);
    NodeContainer gndNodes;
    gndNodes.Create(1);

    // Real SGP4 orbit projected into the local ENU frame: the serving Walker
    // element is at zenith at t=0 and recedes with genuine orbital dynamics.
    ns3::ntncon::WalkerConfig wcfg;
    wcfg.num_planes = 1;
    wcfg.total_sats = 80;
    wcfg.altitude_km = 550.0;
    wcfg.inclination_deg = 53.0;
    wcfg.epoch_unix_s = 1735689600.0;
    const auto elements = ns3::ntncon::WalkerConstellation::BuildDelta(wcfg);
    Ptr<ns3::ntncon::Sgp4MobilityModel> satSgp4 =
        CreateObject<ns3::ntncon::Sgp4MobilityModel>();
    satSgp4->SetElementsSgp4(elements[0]);
    double subLat, subLon, subAlt;
    satSgp4->GetGeodetic(subLat, subLon, subAlt);
    Ptr<NtnEnuProjectionMobilityModel> satEnu = CreateObject<NtnEnuProjectionMobilityModel>();
    satEnu->SetSource(satSgp4);
    satEnu->SetReference(subLat, subLon, 0.0);
    satNodes.Get(0)->AggregateObject(satEnu);

    // Fixed gateway-class ground station at the sub-satellite point (a fixed
    // site is the right model for a gateway).
    MobilityHelper mob;
    mob.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    Ptr<ListPositionAllocator> gndPos = CreateObject<ListPositionAllocator>();
    gndPos->Add(Vector(0.0, 0.0, 50.0));
    mob.SetPositionAllocator(gndPos);
    mob.Install(gndNodes);

    Config::SetDefault("ns3::NtnOranApplication::DataRate",
                       DataRateValue(DataRate(static_cast<uint64_t>(offeredMbps * 1e6))));

    NtnRealStackHelper rs;
    rs.SetRadioBackend(radio == "mmwave" ? NtnRealStackHelper::RadioBackend::Mmwave
                                         : NtnRealStackHelper::RadioBackend::Nr);
    if (radio != "mmwave")
    {
        rs.SetNumerology(numerology);
    }
    rs.SetSimTime(Seconds(simSeconds));
    rs.SetOutputDir(outputDir);
    rs.SetRunTag("thz-ntn-leo-ground-downlink-traffic");
    rs.SetCarrierFrequencyHz(fHz);
    if (!legacy)
    {
        rs.SetBandwidthHz(bandwidthMHz * 1e6);
        rs.SetSatEirpTotalDbm(eirpDbm);
        rs.SetEirpBudgetDbm(eirpDbm, 0.5);
        rs.SetUeRxAntennaGain(gGnd - 10.0 * std::log10(2.0));
        rs.SetUeNoiseFigureDb(nfDb);
        rs.SetSmallScaleFading(fading);
        rs.SetTr38811ExcessLoss(false);
    }
    else
    {
        rs.SetSatConductedPowerDbm(legacyConductedDbm);
    }
    rs.Build(satNodes, gndNodes);

    Ptr<ThzNtnPropagationLossModel> thz;
    Ptr<ThzNtnSpectrumAtmosphereModel> atm;
    if (scalarAtmosphere)
    {
        thz = CreateObject<ThzNtnPropagationLossModel>();
        thz->SetFrequency(fHz);
        rs.AddExtraPropagationLoss(thz);
    }
    else
    {
        atm = CreateObject<ThzNtnSpectrumAtmosphereModel>();
        if (scintillation)
        {
            Ptr<ThzNtnScintillation> sc = CreateObject<ThzNtnScintillation>();
            sc->SetAntennaDiameter(gndDishM);
            atm->SetScintillation(sc);
        }
        rs.AddSpectrumChannelLoss(atm);
    }

    // THz beam pointing impairment in the measured packet path, with the
    // ground dish's own beamwidth.
    Ptr<ThzNtnPointingLossModel> ptg = CreateObject<ThzNtnPointingLossModel>();
    if (!legacy)
    {
        ptg->SetAttribute("Beamwidth3dBDeg", DoubleValue(bw3dB));
    }
    if (pointing)
    {
        rs.AddExtraPropagationLoss(ptg);
    }

    // Per-TB evidence: the SINR the nr PHY reports for every decoded TB.
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
    // The helper installs its eMBB flows at a fixed 5 Mb/s; apply the offered
    // load (the DataRate attribute takes effect on a running client).
    for (uint32_t k = 0; k < rs.GetRemoteHost()->GetNApplications(); ++k)
    {
        if (auto app = DynamicCast<NtnOranApplication>(rs.GetRemoteHost()->GetApplication(k)))
        {
            app->SetAttribute("DataRate",
                              DataRateValue(DataRate(static_cast<uint64_t>(offeredMbps * 1e6))));
        }
    }
    rs.EnableAiFlowMonitor("thz-ntn-leo-ground-downlink-traffic");

    // Occupied bandwidth the PHY integrates noise over.
    const double occHz = rs.GetBwpPrbs() * 12.0 * 15e3 * (1u << numerology);
    const double noiseDbm = ThzNtnAperture::NoiseDbm(occHz, nfDb);
    std::printf("#   occupied %u PRB = %.2f MHz, noise %.2f dBm\n", rs.GetBwpPrbs(), occHz / 1e6,
                noiseDbm);
    std::printf("# %5s  %7s  %9s  %8s  %8s  %8s  %8s  %8s  %8s  %8s  %9s\n", "t_s", "elev",
                "slant_km", "gas_dB", "gasSprd", "scint", "ptg_dB", "budget", "sinr_dB", "tbler",
                "goodput");

    Ptr<MobilityModel> gndMob = gndNodes.Get(0)->GetObject<MobilityModel>();
    uint64_t lastRx = 0;
    rs.RegisterPeriodicCallback(
        Seconds(1.0),
        [&, gndMob, satEnu](Time now) {
            const Vector g = gndMob->GetPosition();
            const Vector s = satEnu->GetPosition();
            const double elev = ElevDegEnu(g, s);
            const double slant = DistM(g, s);
            double gas = 0.0;
            double spread = 0.0;
            double scint = 0.0;
            if (atm)
            {
                gas = atm->GetLastGasCentreDb();
                spread = atm->GetLastMaxLossDb() - atm->GetLastMinLossDb();
                scint = atm->GetLastScintillationDb();
            }
            else if (thz)
            {
                gas = thz->GetLastLossDb();
            }
            const double ptgDb = pointing ? ptg->GetLastLossDb() : 0.0;
            const double budget = legacy ? std::numeric_limits<double>::quiet_NaN()
                                         : eirpDbm - ThzNtnAperture::FsplDb(slant, fHz) - gas +
                                               scint - ptgDb + gGnd - noiseDbm;
            const double sinr = rs.GetUeRecentSinrDb(0);
            const double tbler = rs.GetUeRecentTbler(0);
            const uint64_t rx = rs.GetUeRxBytes(0);
            const double mbps = (rx - lastRx) * 8.0 / 1e6;
            lastRx = rx;
            std::printf(
                "  %5.1f  %7.2f  %9.1f  %8.3f  %8.4f  %8.3f  %8.3f  %8.2f  %8.2f  %8.3f  %9.3f\n",
                now.GetSeconds(), elev, slant / 1000.0, gas, spread, scint, ptgDb, budget, sinr,
                tbler, mbps);
        });

    Simulator::Stop(Seconds(simSeconds));
    Simulator::Run();
    rs.Collect();
    rs.WriteHealthReport();

    std::printf("# === summary ===  measured cell SINR=%.2f dB TBLER=%.4f throughput=%.3f Mbps "
                "(offered %.1f Mb/s), effective EIRP %.2f dBm; atmosphere %s (%llu evaluations, "
                "%llu below the 5 deg P.676 floor)\n",
                rs.GetMeanDlSinrDb(), rs.GetMeanDlTbler(), rs.GetRxThroughputMbps(), offeredMbps,
                rs.GetEffectiveEirpDbm(), atm ? "per-RB" : "scalar",
                static_cast<unsigned long long>(atm ? atm->GetEvaluations() : 0),
                static_cast<unsigned long long>(atm ? atm->GetClampedEvaluations() : 0));

    Simulator::Destroy();
    if (tbf)
    {
        std::fclose(tbf);
    }
    return 0;
}
