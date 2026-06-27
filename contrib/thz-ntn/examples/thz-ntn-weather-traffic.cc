/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only
//
// thz-ntn-weather-traffic — a weather front (fog → rain → wet snow → clear)
// passes over the ground station during a sub-THz LEO pass, while REAL traffic
// flows on a REAL mmwave NR NTN cell (NtnRealStackHelper: SpectrumPhy + MAC +
// HARQ + RLC/PDCP + RRC + EPC).
//
// Audit fix (2026-06 protocol-fidelity audit, channel-plugin recipe):
// the old version folded fog/rain/snow attenuation into a closed-form SNR and
// drove a P2P RateErrorModel through a sigmoid SnrToPer() — no packet ever saw
// the weather. Here the SAME module physics (ThzNtnWeatherAttenuation +
// ThzNtnMolecularAbsorption, re-homed as ThzNtnPropagationLossModel) is chained
// onto the real spectrum channel via AddExtraPropagationLoss(), so each weather
// phase attenuates the transmitted packets and the dip shows up in the
// MEASURED SINR / TBLER / goodput — sub-THz is visibly far more sensitive to
// rain and wet snow than to fog. Nothing closed-form in the packet path.
//
// Mobility is real: the satellite is an SGP4 Walker element projected into the
// scenario's local ENU frame (genuine pass dynamics); the ground station is a
// fixed gateway site, as real THz ground stations are.
//
// Quick test:  --simSeconds=40 --rainMmH=25
#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/ntn-real-stack-helper.h"
#include "ns3/ntn-tr38811-mobility-model.h"
#include "ns3/sgp4-mobility-model.h"
#include "ns3/thz-ntn-pointing-loss-model.h"
#include "ns3/thz-ntn-propagation-loss-model.h"
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
    double satEirpDbm = 115.0; // high-gain sub-THz feeder beam (closes ~187 dB FSPL)
    double rainMmH = 25.0;
    double fogLwc = 0.5;
    double snowMmH = 10.0;
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
    cmd.Parse(argc, argv);

    if (freqGHz > 100.0)
    {
        std::printf("# NOTE: 3GPP spectrum model caps the carrier at 100 GHz; "
                    "clamping %.0f -> 100 GHz\n",
                    freqGHz);
        freqGHz = 100.0;
    }

    std::printf("# thz-ntn-weather-traffic (REAL radio, weather in the packet path)\n");
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
    satSgp4->SetElements(elements[0]);
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
    rs.SetSatEirpDbm(satEirpDbm);
    rs.Build(satNodes, gndNodes);

    // Channel plug-in: the module's weather + molecular-absorption physics as a
    // real PropagationLossModel chained AFTER the built-in Friis loss. The
    // model returns pure atmospheric EXCESS (no FSPL inside), so nothing is
    // double-counted.
    Ptr<ThzNtnPropagationLossModel> wx = CreateObject<ThzNtnPropagationLossModel>();
    wx->SetFrequency(freqGHz * 1e9);
    rs.AddExtraPropagationLoss(wx);

    // gap A2 — THz beam pointing impairment now in the measured packet path.
    Ptr<ThzNtnPointingLossModel> ptg = CreateObject<ThzNtnPointingLossModel>();
    rs.AddExtraPropagationLoss(ptg);

    rs.InstallTraffic(NtnRealStackHelper::TrafficProfile::EmbbStreaming,
                      Seconds(1.0), Seconds(simSeconds - 0.5));
    rs.EnableAiFlowMonitor("thz-ntn-weather-traffic");

    // Weather front: clear → fog → rain → wet snow → clear across the pass.
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
