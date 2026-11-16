/*
 * SPDX-License-Identifier: GPL-2.0-only
 * Copyright (c) 2026 Muhammad Uzair (ns3-ntn-toolkit)
 *
 * ntn-sionna-ris-relay-traffic — a LEO downlink whose direct path is blocked
 * (NLOS, e.g. urban canyon / terrain) is recovered by a Reconfigurable
 * Intelligent Surface switched ON mid-simulation, while REAL traffic flows on
 * a REAL mmwave NR NTN cell (NtnRealStackHelper).
 *
 * Audit fix (2026-06 protocol-fidelity audit, channel-plugin recipe):
 * the old version folded the blockage and the RIS array gain into a
 * closed-form SNR and drove a P2P RateErrorModel through a sigmoid
 * SnrToPer() — packets never felt the blockage. Here the blockage onset and
 * the RIS engagement are LIVE channel reconfigurations
 * (NtnStaticExtraLossModel chained onto the real spectrum channel): the
 * MEASURED SINR collapses when the path is blocked and recovers when the RIS
 * engages. The ITU-R atmospheric cascade also stays in the packet path.
 *
 * SIONNA-07: the RIS path is the far-field plate-scattering law, not
 * 20*log10(N). With half-wavelength elements, unit reflection and perfect
 * phase control, the RIS-reflected power relative to the UNOBSTRUCTED direct
 * path (satellite range d1 ~ direct range) is
 *   P_ris / P_los = (N * lambda / (4 * d2))^2
 * (Tang et al., IEEE TWC 20(1), 2021, eq. (7) with G = pi, d_x = d_y = lambda/2;
 * Ellingson, IEEE OJ-COMS 2021), d2 = RIS-to-UE distance (--risDistM), capped
 * at 0 dB, the specular-mirror limit inside the RIS near field. The RIS path
 * adds coherently (optimized phase) to the blocked direct path. The old
 * 20*log10(N) = 60 dB "gain" omitted the element aperture and the second
 * path-loss leg, so any 32x32 surface erased any blockage up to 60 dB.
 *
 * Mobility is real: SGP4 satellite (ENU-projected), fixed ground terminal.
 *
 * Wave 2 M3 (default since 16 Nov 2026; the old run is
 * `--scripted=1 --legacyConductedDbm=75`):
 *  - geometry instead of a schedule: the satellite is past zenith and
 *    descending; a building facade (--facadeDistM, --facadeHeightM) on the
 *    satellite side of the street blocks the direct ray once the elevation
 *    falls below atan((H - h_UE)/d). The facade is a TR 38.901 clause 7.6.4.2
 *    blockage-model-B screen (NtnScreenBlockageModel), evaluated per RB from
 *    the live positions;
 *  - the RIS hangs on the opposite facade at a fixed ENU point and the direct
 *    plus RIS rays are combined coherently per RB (ThzNtnRisSpectrumModel:
 *    Tang et al. 2021 cascaded law on live geometry, excess-delay phase per
 *    RB, controller re-optimization every 1 ms) through
 *    NtnRealStackHelper::AddSpectrumChannelLoss;
 *  - the RIS is engaged by a controller that reads the MEASURED SINR
 *    (--risMode=auto, threshold --risEngageSinrDb), not at a scripted time;
 *  - declared payload: satellite Pt and aperture (EIRP = Pt + G_tx) and a
 *    terminal aperture G_rx = 10 log10(eta (pi D / lambda)^2), eta 0.6.
 *
 * Quick test:  --simSeconds=40
 */
#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/ntn-atmospheric-loss-chain.h"
#include "ns3/ntn-atmospheric-propagation-loss-model.h"
#include "ns3/nr-spectrum-phy.h"
#include "ns3/nr-ue-net-device.h"
#include "ns3/nr-ue-phy.h"
#include "ns3/ntn-spectrum-seam-model.h"
#include "ns3/ntn-real-stack-helper.h"
#include "ns3/ntn-static-extra-loss-model.h"
#include "ns3/ntn-tr38811-mobility-model.h"
#include "ns3/sgp4-mobility-model.h"
#include "ns3/ntn-screen-blockage-model.h"
#include "ns3/thz-ntn-link-geometry.h"
#include "ns3/thz-ntn-ris-spectrum-model.h"
#include "ns3/thz-ntn-ris.h"
#include "ns3/walker-constellation.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <memory>
#include <string>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("NtnSionnaRisRelayTraffic");

int
main(int argc, char* argv[])
{
    double simSeconds = 40.0;
    double freqGHz = 12.0;
    double legacyConductedDbm = -1.0; // > 0: old conducted operating point (75)
    bool scripted = false;
    double satTxW = 10.0;     // declared satellite RF power per beam
    double satDishM = 0.7;    // satellite aperture diameter (36.7 dBi at 12 GHz)
    double ueDishM = 0.10;    // terminal aperture diameter (19.8 dBi at 12 GHz)
    double ueNfDb = 2.0;      // Ku LNB-class noise figure
    double eta = 0.6;         // aperture efficiency
    double pastZenithS = 25.0; // satellite passed zenith this long before t=0
    double facadeDistM = 10.0;
    double facadeHeightM = 22.0;
    double facadeWidthM = 80.0;
    std::string risMode = "auto";
    double risEngageSinrDb = 10.0;
    bool fading = true;
    double blockageDb = 30.0;
    uint32_t risRows = 32;
    uint32_t risCols = 32;
    double blockFraction = 0.25;
    double risOnFraction = 0.55;
    double risDistM = 30.0; // RIS-to-UE range; beyond 2D^2/lambda = 26 m (32x32, 12 GHz)
    std::string radio = "nr"; // radio spine: "nr" (5G-LENA FR1) | "mmwave" (FR2)
    std::string outputDir = "ntn-sionna-ris-relay-output";

    CommandLine cmd(__FILE__);
    cmd.AddValue("simSeconds", "Simulation duration (s)", simSeconds);
    cmd.AddValue("freqGHz", "Carrier frequency (GHz)", freqGHz);
    cmd.AddValue("legacyConductedDbm", "> 0: CONDUCTED gNB power instead of the declared payload (old 75)",
                 legacyConductedDbm);
    cmd.AddValue("scripted", "1 = old scripted blockage/RIS timeline (scalar losses)", scripted);
    cmd.AddValue("satTxW", "Satellite RF power per beam (W)", satTxW);
    cmd.AddValue("satDishM", "Satellite aperture diameter (m)", satDishM);
    cmd.AddValue("ueDishM", "Terminal aperture diameter (m)", ueDishM);
    cmd.AddValue("ueNfDb", "Terminal noise figure (dB)", ueNfDb);
    cmd.AddValue("pastZenithS", "Satellite passed zenith this many seconds before t=0", pastZenithS);
    cmd.AddValue("facadeDistM", "Horizontal distance UE -> facade on the satellite side (m)", facadeDistM);
    cmd.AddValue("facadeHeightM", "Facade height (m)", facadeHeightM);
    cmd.AddValue("facadeWidthM", "Facade width (m)", facadeWidthM);
    cmd.AddValue("risMode", "auto (engage on measured SINR), on, off", risMode);
    cmd.AddValue("risEngageSinrDb", "auto mode: engage when the measured SINR falls below (dB)",
                 risEngageSinrDb);
    cmd.AddValue("fading", "3GPP small-scale fading (0 = LOS plane wave)", fading);
    cmd.AddValue("radio", "Radio backend: nr (FR1) or mmwave", radio);
    cmd.AddValue("blockageDb", "NLOS blockage on the direct path (dB)", blockageDb);
    cmd.AddValue("risRows", "RIS element rows", risRows);
    cmd.AddValue("risCols", "RIS element cols", risCols);
    cmd.AddValue("blockFraction", "Fraction of sim at which the blockage starts",
                 blockFraction);
    cmd.AddValue("risOnFraction", "Fraction of sim at which the RIS engages",
                 risOnFraction);
    cmd.AddValue("risDistM", "RIS-to-UE distance (m) for the far-field RIS path", risDistM);
    cmd.AddValue("outputDir", "Output directory", outputDir);
    cmd.Parse(argc, argv);

    // SIONNA-07: RIS path relative to the unobstructed LOS, then coherent sum
    // with the blocked direct path. risCompDb is the loss the RIS removes.
    const double lambdaM = 299792458.0 / (freqGHz * 1e9);
    const double nEl = static_cast<double>(std::max(1u, risRows * risCols));
    const double risGainDb =
        std::min(0.0, 20.0 * std::log10(nEl * lambdaM / (4.0 * std::max(risDistM, 1e-3))));
    const double withRisLossDb = std::max(
        0.0,
        -20.0 * std::log10(std::pow(10.0, -blockageDb / 20.0) + std::pow(10.0, risGainDb / 20.0)));
    const double risCompDb = blockageDb - withRisLossDb;

    std::printf("# ntn-sionna-ris-relay-traffic (REAL radio, blockage + RIS in the "
                "packet path)\n");
    if (scripted)
    {
        std::printf("#   [scripted legacy] sim=%.0fs freq=%.1fGHz blockage=%.0fdB "
                    "RIS=%ux%u at %.0fm -> RIS path %.1f dB vs LOS (%.1f dB recovered)\n",
                    simSeconds, freqGHz, blockageDb, risRows, risCols, risDistM, risGainDb,
                    risCompDb);
    }
    NS_ABORT_MSG_IF(!scripted && radio == "mmwave",
                    "geometric blockage + RIS are per-RB plugins (nr backend); use --scripted=1 "
                    "with --radio=mmwave");

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
    Ptr<ns3::ntncon::Sgp4MobilityModel> satSgp4 =
        CreateObject<ns3::ntncon::Sgp4MobilityModel>();
    satSgp4->SetElementsSgp4(elements[0]);
    double subLat, subLon, subAlt;
    satSgp4->GetGeodetic(subLat, subLon, subAlt);
    if (!scripted)
    {
        // The UE site is the sub-satellite point of pastZenithS ago, so the
        // satellite is past zenith and descending over the whole run.
        Vector pe;
        Vector ve;
        satSgp4->GetEcefStateAt(Seconds(-pastZenithS), pe, ve);
        const Vector g = ThzNtnLinkGeometry::EcefToGeodetic(pe);
        subLat = g.x;
        subLon = g.y;
    }
    Ptr<NtnEnuProjectionMobilityModel> satEnu = CreateObject<NtnEnuProjectionMobilityModel>();
    satEnu->SetSource(satSgp4);
    satEnu->SetReference(subLat, subLon, 0.0);
    satNodes.Get(0)->AggregateObject(satEnu);

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
    rs.SetRunTag("ntn-sionna-ris-relay-traffic");
    rs.SetCarrierFrequencyHz(freqGHz * 1e9);
    const double lambdaDecl = 299792458.0 / (freqGHz * 1e9);
    auto apertureGainDbi = [&](double d) {
        return 10.0 * std::log10(eta * std::pow(M_PI * d / lambdaDecl, 2.0));
    };
    const double gTx = apertureGainDbi(satDishM);
    const double gRx = apertureGainDbi(ueDishM);
    const double eirpDecl = 10.0 * std::log10(satTxW * 1e3) + gTx;
    if (legacyConductedDbm > 0.0 || scripted)
    {
        // Old operating point: CONDUCTED power at the array input (NT-02).
        rs.SetSatConductedPowerDbm(legacyConductedDbm > 0.0 ? legacyConductedDbm : 75.0);
    }
    else
    {
        // Declared payload: total EIRP (the helper back-computes the conducted
        // power of its 8x8 array) and the terminal aperture. The default UE
        // panel is 1x2 and beamforms toward the satellite (+3.01 dB), so each
        // element carries G_rx - 10 log10(2) and the panel totals G_rx.
        rs.SetSatEirpTotalDbm(eirpDecl);
        rs.SetUeRxAntennaGain(gRx - 10.0 * std::log10(2.0));
        rs.SetUeNoiseFigureDb(ueNfDb);
        rs.SetSmallScaleFading(fading);
        // The legacy Ntn38811ExcessLossModel carries its own P.676 gas term
        // and an iid-per-second "P.618" scintillation draw (about 1.2 dB rms at
        // 12 GHz, 1.6 dB at 20 GHz): gas would be counted twice next to this
        // example's atmospheric model, and the draw is not P.618-13 2.4.1.
        rs.SetTr38811ExcessLoss(false);
    }
    rs.Build(satNodes, gndNodes);
    Ptr<MobilityModel> ueMob = gndNodes.Get(0)->GetObject<MobilityModel>();
    if (!scripted && legacyConductedDbm <= 0.0)
    {
        const double slant = CalculateDistance(satEnu->GetPosition(), ueMob->GetPosition());
        const double fspl = 20.0 * std::log10(4.0 * M_PI * slant / lambdaDecl);
        const double occHz = rs.GetBwpPrbs() * 12.0 * rs.GetScsKhz() * 1e3;
        const double noise = -174.0 + 10.0 * std::log10(occHz) + ueNfDb;
        std::printf("#   payload: Pt %.1f W, sat aperture %.2f m (%.1f dBi) -> EIRP %.2f dBm "
                    "(effective %.2f); terminal %.2f m (%.1f dBi), NF %.1f dB\n",
                    satTxW, satDishM, gTx, eirpDecl, rs.GetEffectiveEirpDbm(), ueDishM, gRx,
                    ueNfDb);
        std::printf("#   independent budget t=0: FSPL %.2f dB at %.1f km, noise %.2f dBm over "
                    "%.2f MHz -> SNR %.2f dB (no gas, no blockage, no fading)\n",
                    fspl, slant / 1e3, noise, occHz / 1e6, eirpDecl - fspl + gRx - noise);
    }

    // ITU-R atmospheric excess stays in the packet path the whole run.
    Ptr<NtnAtmosphericLossChain> chain = CreateObject<NtnAtmosphericLossChain>();
    chain->SetFrequencyHz(freqGHz * 1e9);
    Ptr<NtnAtmosphericPropagationLossModel> atmo =
        CreateObject<NtnAtmosphericPropagationLossModel>();
    atmo->SetChain(chain);
    rs.AddExtraPropagationLoss(atmo);

    uint64_t lastRx = 0;
    bool risOn = false; // scripted mode state (outlives the block: lambdas hold it)
    if (scripted)
    {
        // Blockage / RIS as a LIVE channel reconfiguration in the real path.
        Ptr<NtnStaticExtraLossModel> nlos = CreateObject<NtnStaticExtraLossModel>();
        nlos->SetLossDb(0.0);
        rs.AddExtraPropagationLoss(nlos);

        rs.InstallTraffic(NtnRealStackHelper::TrafficProfile::EmbbStreaming,
                          Seconds(1.0), Seconds(simSeconds - 0.5));
        rs.EnableAiFlowMonitor("ntn-sionna-ris-relay-traffic"); // WS2 KPM series (TS 28.552 names)

        const double tBlock = blockFraction * simSeconds;
        const double tRis = risOnFraction * simSeconds;
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
    }
    else
    {
        // Street geometry in the UE's ENU frame: u = horizontal direction of
        // the satellite now, v = along the street, z up.
        const Vector sp = satEnu->GetPosition();
        const double sh = std::max(std::hypot(sp.x, sp.y), 1e-9);
        const Vector u(sp.x / sh, sp.y / sh, 0.0);
        const Vector v(-u.y, u.x, 0.0);
        Ptr<NtnScreenBlockageModel> blk = CreateObject<NtnScreenBlockageModel>();
        Ptr<ConstantPositionMobilityModel> facade = CreateObject<ConstantPositionMobilityModel>();
        facade->SetPosition(Vector(facadeDistM * u.x, facadeDistM * u.y, 0.0));
        blk->AddScreen(facade, facadeWidthM, facadeHeightM, "facade");

        // RIS on the opposite facade, 25 m along the street, 12 m up (far
        // field of a 32x32 panel at 12 GHz starts at 2 D^2 / lambda = 26 m).
        Ptr<ConstantPositionMobilityModel> risMob = CreateObject<ConstantPositionMobilityModel>();
        const Vector rp(-facadeDistM * u.x + 25.0 * v.x, -facadeDistM * u.y + 25.0 * v.y, 12.0);
        risMob->SetPosition(rp);
        Ptr<ThzNtnRis> risObj = CreateObject<ThzNtnRis>();
        // Configure() (not the "Frequency" attribute, whose checker starts at
        // 100 GHz): the Tang far-field law itself is frequency generic.
        risObj->Configure(risRows, risCols, freqGHz * 1e9, RisDeployment::GROUND);
        Ptr<ThzNtnRisSpectrumModel> risPlug = CreateObject<ThzNtnRisSpectrumModel>();
        risPlug->SetRis(risObj);
        risPlug->SetRisMobility(risMob);
        risPlug->SetPanelNormal(Vector(u.x - 0.8 * v.x, u.y - 0.8 * v.y, 0.6));
        risPlug->SetBlockage(blk); // direct ray and both RIS legs see the facade
        risPlug->AddTerminal(ueMob);
        risPlug->SetEngaged(risMode == "on");
        rs.AddSpectrumChannelLoss(risPlug);
        std::printf("#   geometry: facade %.0f m wide, %.0f m tall, %.0f m from the UE toward the "
                    "satellite (blocks below %.1f deg elevation); RIS %ux%u at %.1f m from the UE; "
                    "risMode=%s\n",
                    facadeWidthM, facadeHeightM, facadeDistM,
                    std::atan2(facadeHeightM - 1.5, facadeDistM) * 180.0 / M_PI, risRows, risCols,
                    CalculateDistance(rp, ueMob->GetPosition()), risMode.c_str());

        rs.InstallTraffic(NtnRealStackHelper::TrafficProfile::EmbbStreaming,
                          Seconds(1.0), Seconds(simSeconds - 0.5));
        rs.EnableAiFlowMonitor("ntn-sionna-ris-relay-traffic");

        // Controller: engages the RIS from the MEASURED SINR (200 ms period).
        auto engagedAt = std::make_shared<double>(-1.0);
        if (risMode == "auto")
        {
            rs.RegisterPeriodicCallback(MilliSeconds(200),
                                        [&rs, risPlug, risEngageSinrDb, engagedAt](Time now) {
                                            const double sinr = rs.GetUeRecentSinrDb(0);
                                            if (!risPlug->IsEngaged() && !std::isnan(sinr) &&
                                                now > Seconds(1.5) && sinr < risEngageSinrDb)
                                            {
                                                risPlug->SetEngaged(true);
                                                *engagedAt = now.GetSeconds();
                                            }
                                        });
        }
        std::printf("# %5s  %6s  %8s  %5s  %8s  %8s  %8s  %8s  %9s\n", "t_s", "elev",
                    "blk_dB", "ris", "risRel", "plugG", "sinr_dB", "tbler", "goodput");
        rs.RegisterPeriodicCallback(
            Seconds(1.0),
            [&rs, risPlug, blk, satEnu, ueMob, freqGHz, &lastRx](Time now) {
                const Vector s = satEnu->GetPosition();
                const Vector g = ueMob->GetPosition();
                const double elev =
                    std::atan2(s.z - g.z, std::hypot(s.x - g.x, s.y - g.y)) * 180.0 / M_PI;
                const double blkDb = blk->LossDb(s, g, freqGHz * 1e9);
                const auto st = risPlug->GetLastState();
                const double sinr = rs.GetUeRecentSinrDb(0);
                const double tbler = rs.GetUeRecentTbler(0);
                const uint64_t rx = rs.GetUeRxBytes(0);
                const double mbps = (rx - lastRx) * 8.0 / 1e6;
                lastRx = rx;
                std::printf("  %5.1f  %6.2f  %8.2f  %5s  %8.2f  %8.2f  %8.2f  %8.3f  %9.3f\n",
                            now.GetSeconds(), elev, blkDb, risPlug->IsEngaged() ? "ON" : "off",
                            risPlug->IsEngaged() ? st.risRelDb : 0.0, -st.meanGainDb, sinr, tbler,
                            mbps);
            });
        Simulator::Schedule(Seconds(simSeconds - 1e-3), [engagedAt, risPlug] {
            std::printf("#   RIS engaged at %s (controller updates %lu)\n",
                        *engagedAt < 0 ? "never" : (std::to_string(*engagedAt) + " s").c_str(),
                        static_cast<unsigned long>(risPlug->GetControllerUpdates()));
        });
    }

    Simulator::Stop(Seconds(simSeconds));
    Simulator::Run();
    rs.Collect();
    rs.WriteHealthReport();

    std::printf("# === summary ===  measured cell SINR=%.2f dB TBLER=%.4f "
                "throughput=%.3f Mbps (blockage + RIS applied to real packets)\n",
                rs.GetMeanDlSinrDb(), rs.GetMeanDlTbler(), rs.GetRxThroughputMbps());

    Simulator::Destroy();
    return 0;
}
