/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only
//
// thz-ntn-ris-relay-traffic: a sub-THz LEO downlink whose direct path is
// blocked (THz signals are easily obstructed) is recovered by a ThzNtnRis
// surface switched ON mid-simulation, while REAL traffic flows on a REAL
// mmwave NR NTN cell (NtnRealStackHelper).
//
// Audit fix (2026-06 protocol-fidelity audit, channel-plugin recipe):
// the old version folded blockage and RIS gain into a closed-form SNR and
// drove a P2P RateErrorModel through a sigmoid SnrToPer(): packets never
// felt the blockage. Here the blockage onset and the RIS engagement are LIVE
// channel reconfigurations (NtnStaticExtraLossModel chained onto the real
// spectrum channel): the MEASURED SINR collapses when the canyon blocks the
// path and recovers when the RIS engages. The recovery magnitude is the
// module's own physics: the far-field cascaded RIS path loss
// (ThzNtnRis::ComputeCascadedPathLoss_dB, Tang et al. 2021) relative to the
// direct path, power-summed with the residual blocked direct ray. It used to
// be ThzNtnRis::ComputeSnrGain_dB(true) = 20 log10(N), the gain over a single
// element, which is not a gain over the direct link: it ignored the second
// hop's spreading loss and let a 40x40 panel at 200 m "recover" 40 dB when the
// physical RIS ray sits ~45 dB below the direct path.
//
// Molecular absorption (ThzNtnPropagationLossModel) also stays in the packet
// path the whole time. Mobility is real: SGP4 satellite, fixed ground site.
//
// Quick test:  --simSeconds=40 --risX=40 --risY=40
#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/ntn-real-stack-helper.h"
#include "ns3/ntn-static-extra-loss-model.h"
#include "ns3/ntn-tr38811-mobility-model.h"
#include "ns3/sgp4-mobility-model.h"
#include "ns3/thz-ntn-pointing-loss-model.h"
#include "ns3/thz-ntn-propagation-loss-model.h"
#include "ns3/thz-ntn-ris.h"
#include "ns3/walker-constellation.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <functional>
#include <limits>
#include <string>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("ThzNtnRisRelayTraffic");

static int
RunScripted(int argc, char* argv[])
{
    double simSeconds = 40.0;
    double freqGHz = 100.0;    // sub-THz (3GPP spectrum model upper bound)
    double satEirpDbm = 115.0;
    double blockageDb = 40.0;
    uint32_t risX = 40;
    uint32_t risY = 40;
    double reflEff = 0.9;
    // RIS-to-terminal distance. Far field of a 40x40 half-wavelength panel at
    // 100 GHz starts at 2 D^2 / lambda ~ 5 m (D = panel diagonal).
    double risDistM = 10.0;
    double blockFraction = 0.25; // canyon blocks the direct path here
    double risOnFraction = 0.55; // RIS engages here
    std::string outputDir = "thz-ntn-ris-relay-output";
    std::string radio = "nr"; // radio backend: nr (FR1) or mmwave

    CommandLine cmd(__FILE__);
    cmd.AddValue("simSeconds", "Simulation duration (s)", simSeconds);
    cmd.AddValue("freqGHz", "Carrier frequency (GHz), capped at 100", freqGHz);
    cmd.AddValue("satEirpDbm", "Satellite EIRP / gNB Tx power (dBm)", satEirpDbm);
    cmd.AddValue("radio", "Radio backend: nr (FR1) or mmwave", radio);
    cmd.AddValue("blockageDb", "NLOS blockage on the direct path (dB)", blockageDb);
    cmd.AddValue("risX", "RIS elements along X", risX);
    cmd.AddValue("risY", "RIS elements along Y", risY);
    cmd.AddValue("reflEff", "RIS reflection efficiency (0-1)", reflEff);
    cmd.AddValue("risDistM", "RIS-to-terminal distance in m (far field)", risDistM);
    cmd.AddValue("blockFraction", "Fraction of sim at which the blockage starts",
                 blockFraction);
    cmd.AddValue("risOnFraction", "Fraction of sim at which the RIS engages",
                 risOnFraction);
    cmd.AddValue("outputDir", "Output directory", outputDir);
    cmd.Parse(argc, argv);

    if (freqGHz > 100.0)
    {
        std::printf("# NOTE: 3GPP spectrum model caps the carrier at 100 GHz; "
                    "clamping %.0f -> 100 GHz\n",
                    freqGHz);
        freqGHz = 100.0;
    }

    // The RIS recovery magnitude is the module's own array physics.
    Ptr<ThzNtnRis> ris = CreateObject<ThzNtnRis>();
    ris->SetAttribute("NumElementsX", UintegerValue(risX));
    ris->SetAttribute("NumElementsY", UintegerValue(risY));
    ris->SetAttribute("Frequency", DoubleValue(freqGHz * 1e9));
    ris->SetAttribute("ReflectionEfficiency", DoubleValue(reflEff));
    const double risGainDb = ris->ComputeArrayFactorGain_dB(true); // vs one element, printed only
    // RIS ray relative to the direct ray: FSPL(d1) - PL_cascaded(d1, d2). The
    // satellite range cancels (d1 ~ direct range), so a nominal 550 km is used.
    double risRelDb;
    {
        Ptr<ConstantPositionMobilityModel> tx = CreateObject<ConstantPositionMobilityModel>();
        Ptr<ConstantPositionMobilityModel> rp = CreateObject<ConstantPositionMobilityModel>();
        Ptr<ConstantPositionMobilityModel> rx = CreateObject<ConstantPositionMobilityModel>();
        const double d1 = 550.0e3;
        tx->SetPosition(Vector(0.0, 0.0, d1));
        rp->SetPosition(Vector(0.0, 0.0, 0.0));
        rx->SetPosition(Vector(risDistM, 0.0, 0.0));
        const double lambda = 299792458.0 / (freqGHz * 1e9);
        const double fsplDirect = 20.0 * std::log10(4.0 * M_PI * d1 / lambda);
        risRelDb = fsplDirect - ris->ComputeCascadedPathLoss_dB(tx, rp, rx);
    }
    // Extra loss once the RIS engages: blocked direct ray plus RIS ray, summed
    // in power. Recovery = blockageDb - that.
    const double engagedLossDb =
        -10.0 * std::log10(std::pow(10.0, -blockageDb / 10.0) + std::pow(10.0, risRelDb / 10.0));
    const double risCompDb = std::max(0.0, blockageDb - engagedLossDb);

    std::printf("# thz-ntn-ris-relay-traffic (REAL radio, blockage + RIS in the packet path)\n");
    std::printf("#   sim=%.0fs freq=%.0fGHz EIRP=%.1fdBm blockage=%.0fdB "
                "RIS=%ux%u(eff %.2f) at %.1f m: N^2 gain %.1f dB, RIS ray %.1f dB vs direct, "
                "%.1f dB recovered\n",
                simSeconds, freqGHz, satEirpDbm, blockageDb, risX, risY, reflEff, risDistM,
                risGainDb, risRelDb, risCompDb);

    NodeContainer satNodes;
    satNodes.Create(1);
    NodeContainer gndNodes;
    gndNodes.Create(1);

    // Real SGP4 orbit projected into the local ENU frame.
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

    // Fixed ground terminal in the urban canyon.
    MobilityHelper mob;
    mob.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    Ptr<ListPositionAllocator> gndPos = CreateObject<ListPositionAllocator>();
    gndPos->Add(Vector(0.0, 0.0, 1.5));
    mob.SetPositionAllocator(gndPos);
    mob.Install(gndNodes);

    NtnRealStackHelper rs;
    rs.SetRadioBackend(radio == "mmwave" ? NtnRealStackHelper::RadioBackend::Mmwave
                                         : NtnRealStackHelper::RadioBackend::Nr);
    if (radio != "mmwave")
    {
        rs.SetNumerology(1); // FR1 30 kHz SCS
    }
    rs.SetSimTime(Seconds(simSeconds));
    rs.SetOutputDir(outputDir);
    rs.SetRunTag("thz-ntn-ris-relay-traffic");
    rs.SetCarrierFrequencyHz(freqGHz * 1e9);
    // NT-02: declared as CONDUCTED power at the array input. This carrier has
    // no TR 38.821 Set-1 reference in the toolkit, so the EIRP health gate
    // reports "not asserted" rather than certifying an uncalibrated budget.
    rs.SetSatConductedPowerDbm(satEirpDbm);
    rs.Build(satNodes, gndNodes);

    // THz molecular absorption stays in the packet path the whole run.
    Ptr<ThzNtnPropagationLossModel> thz = CreateObject<ThzNtnPropagationLossModel>();
    thz->SetFrequency(freqGHz * 1e9);
    rs.AddExtraPropagationLoss(thz);

    // gap A2: THz beam pointing impairment now in the measured packet path.
    Ptr<ThzNtnPointingLossModel> ptg = CreateObject<ThzNtnPointingLossModel>();
    rs.AddExtraPropagationLoss(ptg);

    // Blockage / RIS as a LIVE channel reconfiguration in the real path.
    Ptr<NtnStaticExtraLossModel> nlos = CreateObject<NtnStaticExtraLossModel>();
    nlos->SetLossDb(0.0); // LOS until the canyon blocks the path
    rs.AddExtraPropagationLoss(nlos);

    rs.InstallTraffic(NtnRealStackHelper::TrafficProfile::EmbbStreaming,
                      Seconds(1.0), Seconds(simSeconds - 0.5));
    rs.EnableAiFlowMonitor("thz-ntn-ris-relay-traffic");

    const double tBlock = blockFraction * simSeconds;
    const double tRis = risOnFraction * simSeconds;
    bool risOn = false;
    Simulator::Schedule(Seconds(tBlock), [nlos, blockageDb] {
        nlos->SetLossDb(blockageDb);
    });
    Simulator::Schedule(Seconds(tRis), [nlos, blockageDb, risCompDb, &risOn] {
        risOn = true;
        nlos->SetLossDb(blockageDb - risCompDb);
    });
    std::printf("#   timeline: LOS -> blocked@%.0fs -> RIS ON@%.0fs\n", tBlock, tRis);
    std::printf("# %5s  %5s  %8s  %8s  %8s  %9s\n",
                "t_s", "ris", "extra_dB", "sinr_dB", "tbler", "goodput");

    uint64_t lastRx = 0;
    rs.RegisterPeriodicCallback(
        Seconds(1.0),
        [&rs, nlos, &risOn, &lastRx](Time now) {
            const double sinr = rs.GetUeRecentSinrDb(0);
            const double tbler = rs.GetUeRecentTbler(0);
            const uint64_t rx = rs.GetUeRxBytes(0);
            const double mbps = (rx - lastRx) * 8.0 / 1e6;
            lastRx = rx;
            std::printf("  %5.1f  %5s  %8.2f  %8.2f  %8.3f  %9.3f\n",
                        now.GetSeconds(), risOn ? "ON" : "off", nlos->GetLossDb(),
                        sinr, tbler, mbps);
        });

    Simulator::Stop(Seconds(simSeconds));
    Simulator::Run();
    rs.Collect();
    rs.WriteHealthReport();

    std::printf("# === summary ===  measured cell SINR=%.2f dB TBLER=%.4f "
                "throughput=%.3f Mbps (blockage + RIS applied to real packets, "
                "RIS recovers %.1f dB of the %.0f dB blockage, far-field cascaded "
                "ThzNtnRis physics)\n",
                rs.GetMeanDlSinrDb(), rs.GetMeanDlTbler(), rs.GetRxThroughputMbps(),
                risCompDb, blockageDb);

    Simulator::Destroy();
    return 0;
}

// ===========================================================================
// Wave 2 M3 (16 Nov 2026): geometry-caused blockage and a geometry-driven RIS
// (default). The function above is the pre-Wave-2 scripted run, kept behind
// --scripted=1.
//
//  - The satellite is an SGP4 element seen from the street at about 31 deg
//    elevation to the south. A 12 m truck (4 m tall) drives east along the
//    lane 3 m south of the terminal; whenever it sits across the line of
//    sight, TR 38.901 7.6.4.2 blockage model B (NtnScreenBlockageModel)
//    attenuates the direct ray per RB on the radio.
//  - A RIS panel (ThzNtnRis, half-wavelength elements) on the facade 15 m
//    north of the terminal adds a second ray: ThzNtnRisSpectrumModel combines
//    direct and RIS rays coherently per RB from the live positions, with the
//    far-field Tang et al. 2021 cascaded law (no 20 log10 N over the direct
//    link) and the RIS excess delay.
//  - The RIS is engaged by a controller that reads the MEASURED SINR only:
//    after the SINR stays 6 dB below its LOS reference for --tttS, the
//    terminal beam is switched to the RIS (direct ray then in a -20 dB
//    sidelobe) and the RIS is configured; every 2 s a 100 ms probe returns the
//    beam to the satellite (beam sweep, TS 38.214 5.2.1.4 L1-RSRP reporting)
//    and keeps the direct path if it has recovered.
//  - Declared payload: Pt 20 W behind a 0.5 m reflector; terminal with a
//    steerable 30 dBi array (--ueGainDbi). Offered load above the blocked
//    capacity (--offeredMbps 100) so goodput can move.
// ===========================================================================
#include "ns3/ntn-oran-application.h"
#include "ns3/ntn-screen-blockage-model.h"
#include "ns3/thz-ntn-aperture.h"
#include "ns3/thz-ntn-ris-spectrum-model.h"
#include "ns3/thz-ntn-spectrum-atmosphere-model.h"

#include <set>
#include <string>
#include <vector>

int
main(int argc, char* argv[])
{
    for (int i = 1; i < argc; ++i)
    {
        if (std::string(argv[i]) == "--scripted=1" || std::string(argv[i]) == "--scripted=true")
        {
            // The scripted timeline has its own (older) option set. Options
            // that exist only in the geometry-driven run (truck, RIS
            // placement, controller) are dropped with a note instead of
            // aborting the run as unknown arguments.
            static const std::set<std::string> kScriptedOpts = {
                "simSeconds", "freqGHz", "satEirpDbm", "radio",         "blockageDb",
                "risX",       "risY",    "reflEff",    "risDistM",      "blockFraction",
                "risOnFraction", "outputDir"};
            std::vector<char*> rest;
            std::string ignored;
            for (int j = 0; j < argc; ++j)
            {
                if (j == i)
                {
                    continue;
                }
                const std::string a(argv[j]);
                if (j > 0 && a.rfind("--", 0) == 0 && a.rfind("--Print", 0) != 0 &&
                    a.rfind("--ns3::", 0) != 0)
                {
                    const std::string name = a.substr(2, a.find('=') - 2);
                    if (kScriptedOpts.count(name) == 0)
                    {
                        ignored += (ignored.empty() ? "" : " ") + a;
                        continue;
                    }
                }
                rest.push_back(argv[j]);
            }
            if (!ignored.empty())
            {
                std::printf("#   --scripted=1: options of the geometry-driven run ignored: %s\n",
                            ignored.c_str());
            }
            return RunScripted(static_cast<int>(rest.size()), rest.data());
        }
    }

    double simSeconds = 30.0;
    double freqGHz = 100.0;
    double satTxW = 20.0;
    double satDishM = 0.5;
    double ueGainDbi = 30.0;
    double nfDb = 7.0;
    double bandwidthMHz = 100.0;
    uint32_t numerology = 3;
    uint32_t risX = 64;
    uint32_t risY = 64;
    double reflEff = 0.9;
    double risNorthM = 15.0;
    double risHeightM = 6.0;
    double truckSpeed = 1.5;
    double truckStartX = -15.0;
    double offeredMbps = 100.0;
    double tttS = 1.0;
    double sidelobeDb = -20.0;
    bool ris = true;
    bool scripted = false;
    std::string outputDir = "thz-ntn-ris-relay-output";

    CommandLine cmd(__FILE__);
    cmd.AddValue("simSeconds", "Simulation duration (s)", simSeconds);
    cmd.AddValue("satTxW", "Satellite transmitter output (W)", satTxW);
    cmd.AddValue("satDishM", "Satellite reflector diameter (m)", satDishM);
    cmd.AddValue("ueGainDbi", "Terminal steerable-array gain (dBi)", ueGainDbi);
    cmd.AddValue("nfDb", "Terminal noise figure (dB)", nfDb);
    cmd.AddValue("bandwidthMHz", "Channel bandwidth (MHz)", bandwidthMHz);
    cmd.AddValue("numerology", "NR numerology (3 = 120 kHz, extrapolated)", numerology);
    cmd.AddValue("risX", "RIS elements along X", risX);
    cmd.AddValue("risY", "RIS elements along Y", risY);
    cmd.AddValue("reflEff", "RIS reflection efficiency", reflEff);
    cmd.AddValue("risNorthM", "RIS distance north of the terminal (m)", risNorthM);
    cmd.AddValue("risHeightM", "RIS height (m)", risHeightM);
    cmd.AddValue("truckSpeed", "Truck speed toward east (m/s)", truckSpeed);
    cmd.AddValue("truckStartX", "Truck centre east offset at t=0 (m)", truckStartX);
    cmd.AddValue("offeredMbps", "Offered downlink CBR (Mb/s)", offeredMbps);
    cmd.AddValue("tttS", "Controller time-to-trigger (s)", tttS);
    cmd.AddValue("sidelobeDb", "Direct-ray terminal gain when the beam is on the RIS (dB)",
                 sidelobeDb);
    cmd.AddValue("ris", "1 = RIS deployed (default), 0 = no RIS", ris);
    cmd.AddValue("scripted", "1 = pre-Wave-2 scripted timeline", scripted);
    cmd.AddValue("outputDir", "Output directory", outputDir);
    cmd.Parse(argc, argv);

    const double fHz = freqGHz * 1e9;
    const double gSat = ThzNtnAperture::GainDbi(satDishM, fHz, 0.6);
    const double eirpDbm = 10.0 * std::log10(satTxW * 1e3) + gSat;

    NodeContainer satNodes;
    satNodes.Create(1);
    NodeContainer gndNodes;
    gndNodes.Create(1);
    ns3::ntncon::WalkerConfig wcfg;
    wcfg.num_planes = 1;
    wcfg.total_sats = 80;
    wcfg.altitude_km = 550.0;
    wcfg.inclination_deg = 53.0;
    wcfg.epoch_unix_s = 1735689600.0;
    const auto elements = ns3::ntncon::WalkerConstellation::BuildDelta(wcfg);
    Ptr<ns3::ntncon::Sgp4MobilityModel> satSgp4 = CreateObject<ns3::ntncon::Sgp4MobilityModel>();
    satSgp4->SetElementsSgp4(elements[0]);
    double subLat, subLon, subAlt;
    satSgp4->GetGeodetic(subLat, subLon, subAlt);
    // Street 7 deg of latitude north of the sub-satellite point: the satellite
    // is low in the southern sky (about 31 deg) and rising.
    Ptr<NtnEnuProjectionMobilityModel> satEnu = CreateObject<NtnEnuProjectionMobilityModel>();
    satEnu->SetSource(satSgp4);
    satEnu->SetReference(subLat + 7.0, subLon, 0.0);
    satNodes.Get(0)->AggregateObject(satEnu);
    Ptr<ConstantPositionMobilityModel> ueMob = CreateObject<ConstantPositionMobilityModel>();
    ueMob->SetPosition(Vector(0.0, 0.0, 1.5));
    gndNodes.Get(0)->AggregateObject(ueMob);

    // The truck: a 12 m x 4 m screen moving east 3 m south of the terminal.
    Ptr<ConstantVelocityMobilityModel> truck = CreateObject<ConstantVelocityMobilityModel>();
    truck->SetPosition(Vector(truckStartX, -3.0, 0.0));
    truck->SetVelocity(Vector(truckSpeed, 0.0, 0.0));
    Ptr<NtnScreenBlockageModel> blk = CreateObject<NtnScreenBlockageModel>();
    blk->AddScreen(truck, 12.0, 4.0, "truck");

    Ptr<ThzNtnRis> panel = CreateObject<ThzNtnRis>();
    panel->SetAttribute("NumElementsX", UintegerValue(risX));
    panel->SetAttribute("NumElementsY", UintegerValue(risY));
    panel->SetAttribute("Frequency", DoubleValue(fHz));
    panel->SetAttribute("ReflectionEfficiency", DoubleValue(reflEff));
    Ptr<ConstantPositionMobilityModel> risMob = CreateObject<ConstantPositionMobilityModel>();
    risMob->SetPosition(Vector(0.0, risNorthM, risHeightM));
    const double risDist = CalculateDistance(risMob->GetPosition(), ueMob->GetPosition());

    std::printf("# thz-ntn-ris-relay-traffic (measured NR; truck blockage from geometry, "
                "geometry-driven RIS, measured-SINR controller)\n");
    std::printf("#   payload: Pt %.0f W, dish %.2f m (%.2f dBi), EIRP %.2f dBm; terminal %.1f dBi, "
                "NF %.1f dB; %.0f GHz mu %u over %.0f MHz (NR numerology EXTRAPOLATED above "
                "71 GHz)\n",
                satTxW, satDishM, gSat, eirpDbm, ueGainDbi, nfDb, freqGHz, numerology,
                bandwidthMHz);
    std::printf("#   RIS %ux%u (%.1f cm) at %.1f m from the terminal: %s field (Fraunhofer "
                "2D^2/lambda); %s\n",
                risX, risY, risX * panel->GetWavelength() / 2.0 * 100.0, risDist,
                panel->IsInNearField(risDist) ? "NEAR (far-field law not valid)" : "far",
                ris ? "deployed" : "NOT deployed (--ris=0)");

    Config::SetDefault("ns3::NtnOranApplication::DataRate",
                       DataRateValue(DataRate(static_cast<uint64_t>(offeredMbps * 1e6))));
    NtnRealStackHelper rs;
    rs.SetRadioBackend(NtnRealStackHelper::RadioBackend::Nr);
    rs.SetNumerology(numerology);
    rs.SetSimTime(Seconds(simSeconds));
    rs.SetOutputDir(outputDir);
    rs.SetRunTag("thz-ntn-ris-relay-traffic");
    rs.SetCarrierFrequencyHz(fHz);
    rs.SetBandwidthHz(bandwidthMHz * 1e6);
    rs.SetSatEirpTotalDbm(eirpDbm);
    rs.SetEirpBudgetDbm(eirpDbm, 0.5);
    rs.SetUeRxAntennaGain(ueGainDbi - 10.0 * std::log10(2.0));
    rs.SetUeNoiseFigureDb(nfDb);
    rs.SetSmallScaleFading(false);
    rs.SetTr38811ExcessLoss(false);
    rs.Build(satNodes, gndNodes);

    Ptr<ThzNtnSpectrumAtmosphereModel> atm = CreateObject<ThzNtnSpectrumAtmosphereModel>();
    rs.AddSpectrumChannelLoss(atm);
    Ptr<ThzNtnRisSpectrumModel> rsm = CreateObject<ThzNtnRisSpectrumModel>();
    rsm->SetRis(panel);
    rsm->SetRisMobility(risMob);
    rsm->SetPanelNormal(Vector(0.0, -1.0, 0.5)); // facade facing the street and the sky
    rsm->SetBlockage(blk);
    rsm->AddTerminal(ueMob);
    rsm->SetAttribute("DirectRayGainWhenEngagedDb", DoubleValue(sidelobeDb));
    rs.AddSpectrumChannelLoss(rsm);

    rs.InstallTraffic(NtnRealStackHelper::TrafficProfile::EmbbStreaming,
                      Seconds(1.0),
                      Seconds(simSeconds - 0.5));
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
    rs.EnableAiFlowMonitor("thz-ntn-ris-relay-traffic");

    // ---- controller on measured SINR only --------------------------------
    double refSinr = std::numeric_limits<double>::quiet_NaN();
    double lowSince = -1.0;
    double lastProbe = 0.0;
    bool probing = false;
    uint32_t engagements = 0;
    Simulator::Schedule(Seconds(1.5), [] {}); // keep ordering explicit
    std::function<void()> ctl;
    ctl = [&]() {
        const double t = Simulator::Now().GetSeconds();
        const double s = rs.GetUeRecentSinrDb(0);
        if (!std::isnan(s))
        {
            if (!rsm->IsEngaged())
            {
                if (probing)
                {
                    probing = false;
                    if (s < refSinr - 3.0)
                    {
                        rsm->SetEngaged(true); // direct path still blocked
                        lastProbe = t;
                    }
                    else
                    {
                        lowSince = -1.0; // direct path back
                    }
                }
                else if (std::isnan(refSinr) || s > refSinr - 6.0)
                {
                    refSinr = std::isnan(refSinr) ? s : 0.9 * refSinr + 0.1 * s;
                    lowSince = -1.0;
                }
                else
                {
                    if (lowSince < 0.0)
                    {
                        lowSince = t;
                    }
                    if (ris && t - lowSince >= tttS)
                    {
                        rsm->SetEngaged(true);
                        ++engagements;
                        lastProbe = t;
                    }
                }
            }
            else if (t - lastProbe >= 2.0)
            {
                rsm->SetEngaged(false); // 100 ms beam probe toward the satellite
                probing = true;
            }
        }
        if (t + 0.1 < simSeconds)
        {
            Simulator::Schedule(MilliSeconds(100), ctl);
        }
    };
    Simulator::Schedule(Seconds(1.2), ctl);

    std::printf("# %5s  %6s  %7s  %7s  %8s  %8s  %8s  %8s  %8s  %9s\n", "t_s", "elev", "blocked",
                "ris", "dirBlk", "risRel", "excNs", "sinr_dB", "tbler", "goodput");
    uint64_t lastRx = 0;
    double gp[3] = {0, 0, 0};
    uint32_t np[3] = {0, 0, 0};
    rs.RegisterPeriodicCallback(Seconds(1.0), [&](Time now) {
        const Vector u = ueMob->GetPosition();
        const Vector sp = satEnu->GetPosition();
        const double elev =
            std::atan2(sp.z - u.z, std::hypot(sp.x - u.x, sp.y - u.y)) * 180.0 / M_PI;
        const bool blocked = blk->IsBlocked(sp, u);
        const auto st = rsm->GetLastState();
        const double sinr = rs.GetUeRecentSinrDb(0);
        const double tbler = rs.GetUeRecentTbler(0);
        const uint64_t rx = rs.GetUeRxBytes(0);
        const double mbps = (rx - lastRx) * 8.0 / 1e6;
        lastRx = rx;
        if (now.GetSeconds() >= 2.0)
        {
            const int k = !blocked ? 0 : (rsm->IsEngaged() ? 2 : 1);
            gp[k] += mbps;
            ++np[k];
        }
        std::printf("  %5.1f  %6.2f  %7s  %7s  %8.2f  %8.2f  %8.1f  %8.2f  %8.3f  %9.3f\n",
                    now.GetSeconds(), elev, blocked ? "yes" : "no",
                    rsm->IsEngaged() ? "ON" : "off", st.directBlockDb,
                    st.risRelDb < -500 ? std::nan("") : st.risRelDb, st.excessDelayNs, sinr,
                    tbler, mbps);
    });

    Simulator::Stop(Seconds(simSeconds));
    Simulator::Run();
    rs.Collect();
    rs.WriteHealthReport();
    auto avg = [&](int k) { return np[k] ? gp[k] / np[k] : std::nan(""); };
    std::printf("# === goodput by geometric state (1 s bins from t=2 s) === LOS: %.3f Mb/s (%u s); "
                "blocked, RIS off: %.3f Mb/s (%u s); blocked, RIS on: %.3f Mb/s (%u s); RIS "
                "engagements %u, controller updates %llu\n",
                avg(0), np[0], avg(1), np[1], avg(2), np[2], engagements,
                static_cast<unsigned long long>(rsm->GetControllerUpdates()));
    std::printf("# === summary ===  measured cell SINR=%.2f dB TBLER=%.4f throughput=%.3f Mbps "
                "(offered %.0f)\n",
                rs.GetMeanDlSinrDb(), rs.GetMeanDlTbler(), rs.GetRxThroughputMbps(), offeredMbps);
    Simulator::Destroy();
    return 0;
}
