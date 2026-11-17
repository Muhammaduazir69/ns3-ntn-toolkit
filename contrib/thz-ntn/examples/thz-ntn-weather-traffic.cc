/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only
//
// thz-ntn-weather-traffic: a weather front (fog -> rain -> wet snow -> clear)
// passes over the ground station during a sub-THz LEO pass, while REAL traffic
// flows on a REAL mmwave NR NTN cell (NtnRealStackHelper: SpectrumPhy + MAC +
// HARQ + RLC/PDCP + RRC + EPC).
//
// Audit fix (2026-06 protocol-fidelity audit, channel-plugin recipe):
// the old version folded fog/rain/snow attenuation into a closed-form SNR and
// drove a P2P RateErrorModel through a sigmoid SnrToPer(): no packet ever saw
// the weather. Here the SAME module physics (ThzNtnWeatherAttenuation +
// ThzNtnMolecularAbsorption, re-homed as ThzNtnPropagationLossModel) is chained
// onto the real spectrum channel via AddExtraPropagationLoss(), so each weather
// phase attenuates the transmitted packets and the dip shows up in the
// MEASURED SINR / TBLER / goodput: sub-THz is visibly far more sensitive to
// rain and wet snow than to fog. Nothing closed-form in the packet path.
//
// Mobility is real: the satellite is an SGP4 Walker element projected into the
// scenario's local ENU frame (genuine pass dynamics); the ground station is a
// fixed gateway site, as real THz ground stations are.
//
// Wave 2 M3 (16 Nov 2026): weather caused by geometry and time (default).
//  - A convective EXCELL rain cell (ThzNtnRainCellField: R = R_peak exp(-r/r0))
//    is advected over the station by the wind; the attenuation of every RB is
//    the ITU-R P.838-3 specific attenuation k(f) R^alpha(f) integrated along
//    the LIVE slant path to the satellite below the P.839 rain height
//    (ThzNtnSpectrumAtmosphereModel via AddSpectrumChannelLoss), plus P.676
//    line-by-line gas per RB, an optional P.840 cloud layer, and P.618 2.4.1
//    scintillation of the declared dish. Nothing is switched on a timer: the
//    fade grows as the cell arrives and the path tilts with the pass.
//  - Declared payload (Pt, reflector diameters, aperture law) as in
//    thz-ntn-leo-ground-downlink-traffic; the legacy TR 38.811 excess model is
//    not chained (it double counts gas and over-draws scintillation).
//  - --offeredMbps above capacity makes goodput follow the measured MCS.
// --scripted=1 restores the pre-Wave-2 timed front (fog -> rain -> wet snow,
// scalar ThzNtnPropagationLossModel, 115 dBm conducted, mu 1).
//
// Quick test:  --simSeconds=40
#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/ntn-real-stack-helper.h"
#include "ns3/ntn-tr38811-mobility-model.h"
#include "ns3/ntn-oran-application.h"
#include "ns3/sgp4-mobility-model.h"
#include "ns3/thz-ntn-aperture.h"
#include "ns3/thz-ntn-pointing-loss-model.h"
#include "ns3/thz-ntn-propagation-loss-model.h"
#include "ns3/thz-ntn-spectrum-atmosphere-model.h"
#include "ns3/walker-constellation.h"

#include <cmath>
#include <cstdio>
#include <map>
#include <string>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("ThzNtnWeatherTraffic");

namespace
{

enum class Wx
{
    Clear,
    Fog,
    Rain,
    Snow
};

const char*
PhaseName(Wx w)
{
    switch (w)
    {
    case Wx::Clear:
        return "clear";
    case Wx::Fog:
        return "fog";
    case Wx::Rain:
        return "rain";
    case Wx::Snow:
        return "wet-snow";
    }
    return "?";
}

/// Per-phase MEASURED accumulators (SINR samples + delivered bytes).
struct PhaseStats
{
    double sumSinr = 0.0;
    uint64_t nSinr = 0;
    uint64_t rxBytes = 0;
    double seconds = 0.0;
};

Wx g_phase = Wx::Clear;
std::map<Wx, PhaseStats> g_stats;

double
ElevDegEnu(const Vector& gnd, const Vector& sat)
{
    const double dx = sat.x - gnd.x;
    const double dy = sat.y - gnd.y;
    const double dz = sat.z - gnd.z;
    const double horiz = std::max(std::sqrt(dx * dx + dy * dy), 1e-3);
    return std::atan2(dz, horiz) * 180.0 / M_PI;
}

} // namespace

int
main(int argc, char* argv[])
{
    double simSeconds = 40.0;
    double freqGHz = 100.0;    // sub-THz / W-band (3GPP spectrum model upper bound)
    double satEirpDbm = 115.0; // --scripted=1 only: conducted power of the old run
    double rainMmH = 25.0;
    double fogLwc = 0.5;
    double snowMmH = 10.0;
    bool scripted = false;
    // Geometric weather (default).
    double satTxW = 10.0;
    double satDishM = 0.5;
    double gndDishM = 0.6;
    double eta = 0.6;
    double nfDb = 7.0;
    double bandwidthMHz = 100.0;
    uint32_t numerology = 3;
    double cellPeakMmH = 30.0; // convective cell peak rate (mm/h)
    double cellR0Km = 1.5;     // EXCELL e-folding radius (km)
    double windMps = 20.0;     // advection speed toward east (m/s)
    double cellStartKm = -1.2; // cell centre east of the station at t = 0 (km)
    double cloudLwc = 0.0;     // P.840 cloud liquid water content (g/m^3)
    double offeredMbps = 5.0;
    std::string outputDir = "thz-ntn-weather-traffic-output";
    std::string radio = "nr"; // radio backend: nr (FR1) or mmwave

    CommandLine cmd(__FILE__);
    cmd.AddValue("simSeconds", "Simulation duration (s)", simSeconds);
    cmd.AddValue("freqGHz", "Carrier frequency (GHz)", freqGHz);
    cmd.AddValue("satEirpDbm", "Satellite EIRP / gNB Tx power (dBm)", satEirpDbm);
    cmd.AddValue("radio", "Radio backend: nr (FR1) or mmwave", radio);
    cmd.AddValue("rainMmH", "Rain rate during the rain phase (mm/h)", rainMmH);
    cmd.AddValue("fogLwc", "Fog liquid water content (g/m^3)", fogLwc);
    cmd.AddValue("snowMmH", "Snow rate during the snow phase (mm/h)", snowMmH);
    cmd.AddValue("outputDir", "Output directory", outputDir);
    cmd.AddValue("scripted", "1 = pre-Wave-2 timed weather front (legacy)", scripted);
    cmd.AddValue("satTxW", "Satellite transmitter output power (W)", satTxW);
    cmd.AddValue("satDishM", "Satellite reflector diameter (m)", satDishM);
    cmd.AddValue("gndDishM", "Ground reflector diameter (m)", gndDishM);
    cmd.AddValue("nfDb", "Ground receiver noise figure (dB)", nfDb);
    cmd.AddValue("bandwidthMHz", "Channel bandwidth (MHz)", bandwidthMHz);
    cmd.AddValue("numerology", "NR numerology (3 = 120 kHz, extrapolated)", numerology);
    cmd.AddValue("cellPeakMmH", "Rain-cell peak rate (mm/h)", cellPeakMmH);
    cmd.AddValue("cellR0Km", "Rain-cell e-folding radius (km)", cellR0Km);
    cmd.AddValue("windMps", "Cell advection speed toward east (m/s)", windMps);
    cmd.AddValue("cellStartKm", "Cell centre east offset at t=0 (km, negative = west)",
                 cellStartKm);
    cmd.AddValue("cloudLwc", "Cloud liquid water content (g/m^3)", cloudLwc);
    cmd.AddValue("offeredMbps", "Offered downlink CBR (Mb/s)", offeredMbps);
    cmd.Parse(argc, argv);

    if (freqGHz > 100.0)
    {
        std::printf("# NOTE: 3GPP spectrum model caps the carrier at 100 GHz; "
                    "clamping %.0f -> 100 GHz\n",
                    freqGHz);
        freqGHz = 100.0;
    }

    if (!scripted)
    {
        const double fHz = freqGHz * 1e9;
        const double gSat = ThzNtnAperture::GainDbi(satDishM, fHz, eta);
        const double gGnd = ThzNtnAperture::GainDbi(gndDishM, fHz, eta);
        const double ptDbm = 10.0 * std::log10(satTxW * 1e3);
        const double eirpDbm = ptDbm + gSat;
        std::printf("# thz-ntn-weather-traffic (measured NR; rain cell advected over the "
                    "station, per-RB P.838 path integral)\n");
        std::printf("#   payload: Pt %.1f W, sat dish %.2f m (%.2f dBi), EIRP %.2f dBm; ground "
                    "dish %.2f m (%.2f dBi), NF %.1f dB\n",
                    satTxW, satDishM, gSat, eirpDbm, gndDishM, gGnd, nfDb);
        std::printf("#   air interface: %.0f GHz, mu %u over %.0f MHz (NR numerology "
                    "EXTRAPOLATED above 71 GHz, TS 38.211 Table 4.2-1 / TS 38.101-2)\n",
                    freqGHz, numerology, bandwidthMHz);
        std::printf("#   rain cell: EXCELL peak %.1f mm/h, r0 %.2f km, starts %.2f km east of "
                    "the station, moving east at %.1f m/s; rain height %.2f km (P.839 "
                    "mid-latitude summer table); cloud LWC %.2f g/m^3\n",
                    cellPeakMmH, cellR0Km, cellStartKm, windMps, 3.5, cloudLwc);

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
        Ptr<NtnEnuProjectionMobilityModel> satEnu =
            CreateObject<NtnEnuProjectionMobilityModel>();
        satEnu->SetSource(satSgp4);
        satEnu->SetReference(subLat, subLon, 0.0);
        satNodes.Get(0)->AggregateObject(satEnu);
        MobilityHelper mob;
        mob.SetMobilityModel("ns3::ConstantPositionMobilityModel");
        Ptr<ListPositionAllocator> gndPos = CreateObject<ListPositionAllocator>();
        gndPos->Add(Vector(0.0, 0.0, 5.0));
        mob.SetPositionAllocator(gndPos);
        mob.Install(gndNodes);

        Config::SetDefault("ns3::NtnOranApplication::DataRate",
                           DataRateValue(DataRate(static_cast<uint64_t>(offeredMbps * 1e6))));
        NtnRealStackHelper rs;
        rs.SetRadioBackend(NtnRealStackHelper::RadioBackend::Nr);
        rs.SetNumerology(numerology);
        rs.SetSimTime(Seconds(simSeconds));
        rs.SetOutputDir(outputDir);
        rs.SetRunTag("thz-ntn-weather-traffic");
        rs.SetCarrierFrequencyHz(fHz);
        rs.SetBandwidthHz(bandwidthMHz * 1e6);
        rs.SetSatEirpTotalDbm(eirpDbm);
        rs.SetEirpBudgetDbm(eirpDbm, 0.5);
        rs.SetUeRxAntennaGain(gGnd - 10.0 * std::log10(2.0));
        rs.SetUeNoiseFigureDb(nfDb);
        rs.SetSmallScaleFading(false);
        rs.SetTr38811ExcessLoss(false);
        rs.Build(satNodes, gndNodes);

        Ptr<ThzNtnRainCellField> field = CreateObject<ThzNtnRainCellField>();
        field->SetRainHeightFromClimate(itu::Itu618LossModel::ClimateRegion::midlat_summer);
        ThzNtnRainCellField::Cell cell;
        cell.x0 = cellStartKm * 1e3;
        cell.y0 = 0.0;
        cell.vx = windMps;
        cell.peakRate = cellPeakMmH;
        cell.r0 = cellR0Km * 1e3;
        field->AddCell(cell);
        Ptr<ThzNtnSpectrumAtmosphereModel> atm = CreateObject<ThzNtnSpectrumAtmosphereModel>();
        atm->SetRainField(field);
        atm->SetCloudLwc(cloudLwc);
        Ptr<ThzNtnScintillation> sc = CreateObject<ThzNtnScintillation>();
        sc->SetAntennaDiameter(gndDishM);
        atm->SetScintillation(sc);
        rs.AddSpectrumChannelLoss(atm);
        Ptr<ThzNtnPointingLossModel> ptg = CreateObject<ThzNtnPointingLossModel>();
        ptg->SetAttribute("Beamwidth3dBDeg",
                          DoubleValue(ThzNtnAperture::Beamwidth3dBDeg(gndDishM, fHz)));
        rs.AddExtraPropagationLoss(ptg);

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
        rs.EnableAiFlowMonitor("thz-ntn-weather-traffic");

        const double occHz = rs.GetBwpPrbs() * 12.0 * 15e3 * (1u << numerology);
        const double noiseDbm = ThzNtnAperture::NoiseDbm(occHz, nfDb);
        // rainX = 1 when rain was applied above 55 GHz, outside the validated
        // range of ITU-R P.618-14 2.2.1.1 (Wave 6B dataset flag).
        std::printf("# %5s  %7s  %8s  %8s  %8s  %8s  %8s  %8s  %8s  %8s  %9s  %5s\n", "t_s",
                    "elev", "R_stn", "R_pathMx", "rain_dB", "rainSprd", "gas_dB", "budget",
                    "sinr_dB", "tbler", "goodput", "rainX");
        Ptr<MobilityModel> gndMob = gndNodes.Get(0)->GetObject<MobilityModel>();
        uint64_t lastRx = 0;
        double sumDry = 0.0;
        double sumWet = 0.0;
        uint32_t nDry = 0;
        uint32_t nWet = 0;
        rs.RegisterPeriodicCallback(Seconds(1.0), [&, gndMob, satEnu, atm, field](Time now) {
            const Vector g = gndMob->GetPosition();
            const Vector sp = satEnu->GetPosition();
            const double elev = ElevDegEnu(g, sp);
            const double slant = std::sqrt((sp.x - g.x) * (sp.x - g.x) +
                                           (sp.y - g.y) * (sp.y - g.y) +
                                           (sp.z - g.z) * (sp.z - g.z));
            const double rStn = field->RainRateAt(g.x, g.y, now);
            const double rain = atm->GetLastRainCentreDb();
            const double gas = atm->GetLastGasCentreDb();
            const double sprd = atm->GetLastMaxLossDb() - atm->GetLastMinLossDb();
            const double budget = eirpDbm - ThzNtnAperture::FsplDb(slant, fHz) - gas - rain +
                                  atm->GetLastScintillationDb() - ptg->GetLastLossDb() + gGnd -
                                  noiseDbm;
            const double sinr = rs.GetUeRecentSinrDb(0);
            const double tbler = rs.GetUeRecentTbler(0);
            const uint64_t rx = rs.GetUeRxBytes(0);
            const double mbps = (rx - lastRx) * 8.0 / 1e6;
            lastRx = rx;
            if (now.GetSeconds() >= 2.0)
            {
                if (rain < 1.0)
                {
                    sumDry += mbps;
                    ++nDry;
                }
                else
                {
                    sumWet += mbps;
                    ++nWet;
                }
            }
            std::printf("  %5.1f  %7.2f  %8.2f  %8.2f  %8.2f  %8.3f  %8.3f  %8.2f  %8.2f  %8.3f  "
                        "%9.3f  %5d\n",
                        now.GetSeconds(), elev, rStn, atm->GetLastPathRainRate(), rain, sprd,
                        gas, budget, sinr, tbler, mbps, atm->WasLastRainExtrapolated() ? 1 : 0);
        });

        Simulator::Stop(Seconds(simSeconds));
        Simulator::Run();
        rs.Collect();
        rs.WriteHealthReport();
        std::printf("# === per-condition MEASURED goodput === rain fade < 1 dB: %u s, %.3f Mb/s; "
                    "rain fade >= 1 dB: %u s, %.3f Mb/s (offered %.1f Mb/s)\n",
                    nDry, nDry ? sumDry / nDry : 0.0, nWet, nWet ? sumWet / nWet : 0.0,
                    offeredMbps);
        std::printf("# === summary ===  measured cell SINR=%.2f dB TBLER=%.4f "
                    "throughput=%.3f Mbps (rain cell field on real packets, per-RB P.838/P.676)\n",
                    rs.GetMeanDlSinrDb(), rs.GetMeanDlTbler(), rs.GetRxThroughputMbps());
        std::printf("# rain validity: %llu link evaluations applied rain above 55 GHz "
                    "(ITU-R P.618-14 2.2.1.1 validated to 55 GHz; P.838-3 to 1000 GHz)\n",
                    static_cast<unsigned long long>(atm->GetExtrapolatedRainCount()));
        Simulator::Destroy();
        return 0;
    }

    std::printf("# thz-ntn-weather-traffic --scripted=1 (pre-Wave-2 timed front, legacy power)\n");
    std::printf("#   sim=%.0fs freq=%.0fGHz EIRP=%.1fdBm rain=%.0fmm/h fogLwc=%.2f "
                "snow=%.0fmm/h\n",
                simSeconds, freqGHz, satEirpDbm, rainMmH, fogLwc, snowMmH);

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

    // The ground station is a fixed gateway site at the sub-point (real THz
    // ground stations are static high-gain dishes).
    MobilityHelper mob;
    mob.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    Ptr<ListPositionAllocator> gndPos = CreateObject<ListPositionAllocator>();
    gndPos->Add(Vector(0.0, 0.0, 5.0));
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
    rs.SetRunTag("thz-ntn-weather-traffic");
    rs.SetCarrierFrequencyHz(freqGHz * 1e9);
    // NT-02: declared as CONDUCTED power at the array input. This carrier has
    // no TR 38.821 Set-1 reference in the toolkit, so the EIRP health gate
    // reports "not asserted" rather than certifying an uncalibrated budget.
    rs.SetSatConductedPowerDbm(satEirpDbm);
    rs.Build(satNodes, gndNodes);

    // Channel plug-in: the module's weather + molecular-absorption physics as a
    // real PropagationLossModel chained AFTER the built-in Friis loss. The
    // model returns pure atmospheric EXCESS (no FSPL inside), so nothing is
    // double-counted.
    Ptr<ThzNtnPropagationLossModel> wx = CreateObject<ThzNtnPropagationLossModel>();
    wx->SetFrequency(freqGHz * 1e9);
    rs.AddExtraPropagationLoss(wx);

    // gap A2: THz beam pointing impairment now in the measured packet path.
    Ptr<ThzNtnPointingLossModel> ptg = CreateObject<ThzNtnPointingLossModel>();
    rs.AddExtraPropagationLoss(ptg);

    rs.InstallTraffic(NtnRealStackHelper::TrafficProfile::EmbbStreaming,
                      Seconds(1.0), Seconds(simSeconds - 0.5));
    rs.EnableAiFlowMonitor("thz-ntn-weather-traffic");

    // Weather front: clear -> fog -> rain -> wet snow -> clear across the pass.
    // Each phase reconfigures the LIVE channel plug-in (packets feel it).
    const double tFog = 0.20 * simSeconds;
    const double tRain = 0.35 * simSeconds;
    const double tSnow = 0.55 * simSeconds;
    const double tClear = 0.75 * simSeconds;
    Simulator::Schedule(Seconds(tFog), [wx, fogLwc] {
        g_phase = Wx::Fog;
        wx->SetFogLwc(fogLwc);
    });
    Simulator::Schedule(Seconds(tRain), [wx, rainMmH] {
        g_phase = Wx::Rain;
        wx->SetFogLwc(0.0);
        wx->SetRainRate(rainMmH);
    });
    Simulator::Schedule(Seconds(tSnow), [wx, snowMmH] {
        g_phase = Wx::Snow;
        wx->SetRainRate(0.0);
        wx->SetSnowRate(snowMmH, true);
    });
    Simulator::Schedule(Seconds(tClear), [wx] {
        g_phase = Wx::Clear;
        wx->SetSnowRate(0.0);
    });
    std::printf("#   front: clear→fog@%.0fs→rain@%.0fs→snow@%.0fs→clear@%.0fs\n",
                tFog, tRain, tSnow, tClear);
    std::printf("# %5s  %7s  %-9s  %8s  %8s  %8s  %9s\n",
                "t_s", "elev", "weather", "wxLoss", "sinr_dB", "tbler", "goodput");

    // 1 Hz probe: MEASURED SINR/TBLER off the PHY trace, goodput off the UE
    // PacketSink, weather loss off the live plug-in.
    Ptr<MobilityModel> gndMob = gndNodes.Get(0)->GetObject<MobilityModel>();
    uint64_t lastRx = 0;
    rs.RegisterPeriodicCallback(
        Seconds(1.0),
        [&rs, wx, gndMob, satEnu, &lastRx](Time now) {
            const double elev = ElevDegEnu(gndMob->GetPosition(), satEnu->GetPosition());
            const double sinr = rs.GetUeRecentSinrDb(0);
            const double tbler = rs.GetUeRecentTbler(0);
            const uint64_t rx = rs.GetUeRxBytes(0);
            const double mbps = (rx - lastRx) * 8.0 / 1e6;
            lastRx = rx;
            PhaseStats& ps = g_stats[g_phase];
            if (!std::isnan(sinr))
            {
                ps.sumSinr += sinr;
                ps.nSinr++;
            }
            ps.rxBytes += static_cast<uint64_t>(mbps * 1e6 / 8.0);
            ps.seconds += 1.0;
            std::printf("  %5.1f  %7.2f  %-9s  %8.2f  %8.2f  %8.3f  %9.3f\n",
                        now.GetSeconds(), elev, PhaseName(g_phase), wx->GetLastLossDb(),
                        sinr, tbler, mbps);
        });

    Simulator::Stop(Seconds(simSeconds));
    Simulator::Run();
    rs.Collect();
    rs.WriteHealthReport();

    std::printf("# === per-phase MEASURED summary ===\n");
    std::printf("# %-9s  %8s  %12s\n", "phase", "sinr_dB", "goodput_Mbps");
    for (Wx w : {Wx::Clear, Wx::Fog, Wx::Rain, Wx::Snow})
    {
        const PhaseStats& ps = g_stats[w];
        const double meanSinr = ps.nSinr ? ps.sumSinr / ps.nSinr : std::nan("");
        const double mbps = ps.seconds > 0.0 ? ps.rxBytes * 8.0 / ps.seconds / 1e6 : 0.0;
        std::printf("# %-9s  %8.2f  %12.3f\n", PhaseName(w), meanSinr, mbps);
    }
    std::printf("# === summary ===  measured cell SINR=%.2f dB TBLER=%.4f "
                "throughput=%.3f Mbps (weather applied to real packets)\n",
                rs.GetMeanDlSinrDb(), rs.GetMeanDlTbler(), rs.GetRxThroughputMbps());

    Simulator::Destroy();
    return 0;
}
