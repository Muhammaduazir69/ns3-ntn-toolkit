/*
 * SPDX-License-Identifier: GPL-2.0-only
 * Copyright (c) 2026 Muhammad Uzair (ns3-ntn-toolkit)
 *
 * ntn-sionna-rain-event-traffic — a convective rain cell sweeps over the
 * gateway mid-pass, exercising the NtnAtmosphericLossChain dynamically while
 * REAL traffic flows on a REAL mmwave NR NTN cell (NtnRealStackHelper:
 * SpectrumPhy + MAC + HARQ + RLC/PDCP + RRC + EPC).
 *
 * Audit fix (2026-06 protocol-fidelity audit, channel-plugin recipe):
 * the old version queried the cascade in a probe loop and drove a P2P
 * RateErrorModel through a sigmoid SnrToPer() — packets never saw the rain.
 * Here the SAME ITU-R physics (P.676 gaseous + P.618/P.838 rain), re-homed as
 * NtnAtmosphericPropagationLossModel (pure atmospheric EXCESS, no FSPL
 * double-count), is chained onto the real spectrum channel via
 * AddExtraPropagationLoss(). The rain-rate schedule reconfigures the LIVE
 * chain, so Ka-band attenuation rises and falls in the MEASURED SINR / TBLER
 * and the delivered goodput dips during the cell and recovers — driven
 * entirely by the channel, nothing closed-form in the packet path.
 *
 * Mobility is real: the satellite is an SGP4 Walker element projected into
 * the scenario's local ENU frame; the gateway is a fixed Ka-band ground site.
 *
 * SIONNA-07 note: the chain applies the ITU-R P.618-13 Section 2.2.1.1
 * effective-path method to the instantaneous rain rate of each schedule step.
 * P.618 defines that method for the 0.01% exceedance rate R0.01, so the fade
 * here is the long-term-statistics attenuation a station with R0.01 equal to
 * the step's rate would see, not a time-series rain-cell model (P.1853). The
 * station latitude used by the vertical factor is the Itu618 default (45 deg).
 *
 * Wave 2 M3 (default since 16 Nov 2026; the old run is `--scripted=1
 * --legacyConductedDbm=95`):
 *  - declared payload instead of a 95 dBm "conducted" knob: TR 38.821
 *    Table 6.1.1.1-1/-2 Set-1 Ka band (satellite EIRP density 4 dBW/MHz, VSAT
 *    39.7 dBi, G/T 15.9 dB/K) via ApplyTr38821Set1Defaults();
 *  - rain caused by geometry and time: an EXCELL rain cell
 *    (ThzNtnRainCellField) advected at --cellSpeedMps sits --cellOffsetM
 *    along the satellite ground track; the P.838-3 specific attenuation of
 *    EACH RB is integrated along the live slant path below the P.839 rain
 *    height (ThzNtnSpectrumAtmosphereModel through
 *    NtnRealStackHelper::AddSpectrumChannelLoss). As the satellite recedes
 *    the slant path below 3.5 km swings into the cell: the fade is an outcome
 *    of the pass, not of a schedule. P.676-13 gas is in the same plugin.
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
#include "ns3/ntn-tr38811-mobility-model.h"
#include "ns3/sgp4-mobility-model.h"
#include "ns3/thz-ntn-rain-cell-field.h"
#include "ns3/thz-ntn-spectrum-atmosphere-model.h"
#include "ns3/walker-constellation.h"

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("NtnSionnaRainEventTraffic");

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

} // namespace

int
main(int argc, char* argv[])
{
    double simSeconds = 40.0;
    double freqGHz = 20.0;     // Ka-band — rain-sensitive
    double legacyConductedDbm = -1.0; // >0: old 95 dBm conducted operating point
    double peakRainMmH = 50.0;
    bool scripted = false;
    double cellOffsetM = 1600.0; // cell centre along the ground track at t=0
    double cellR0M = 800.0;      // EXCELL e-folding radius
    double cellSpeedMps = 10.0;  // advection toward the station
    bool fading = true;
    std::string radio = "nr"; // radio spine: "nr" (5G-LENA FR1) | "mmwave" (FR2)
    std::string outputDir = "ntn-sionna-rain-event-output";

    CommandLine cmd(__FILE__);
    cmd.AddValue("simSeconds", "Simulation duration (s)", simSeconds);
    cmd.AddValue("freqGHz", "Carrier frequency (GHz)", freqGHz);
    cmd.AddValue("legacyConductedDbm",
                 "> 0: replace the Set-1 Ka payload by this CONDUCTED gNB power (old default 95)",
                 legacyConductedDbm);
    cmd.AddValue("scripted", "1 = old scripted rain-rate time series (P.618 scalar chain)", scripted);
    cmd.AddValue("cellOffsetM", "Rain cell centre along the ground track at t=0 (m)", cellOffsetM);
    cmd.AddValue("cellR0M", "EXCELL e-folding radius r0 (m)", cellR0M);
    cmd.AddValue("cellSpeedMps", "Cell advection speed toward the station (m/s)", cellSpeedMps);
    cmd.AddValue("fading", "3GPP small-scale fading on the spatial channel (0 = LOS plane wave)",
                 fading);
    cmd.AddValue("radio", "Radio backend: nr (FR1) or mmwave", radio);
    cmd.AddValue("peakRainMmH", "Peak rain rate of the cell (mm/h)", peakRainMmH);
    cmd.AddValue("outputDir", "Output directory", outputDir);
    cmd.Parse(argc, argv);

    std::printf("# ntn-sionna-rain-event-traffic (REAL radio, ITU-R rain in the packet path)\n");
    std::printf("#   sim=%.0fs freq=%.1fGHz peakRain=%.0fmm/h mode=%s payload=%s\n",
                simSeconds, freqGHz, peakRainMmH,
                scripted ? "scripted rain series (legacy)" : "rain-cell field on the slant path",
                legacyConductedDbm > 0 ? "legacy conducted" : "TR 38.821 Set-1 Ka");
    NS_ABORT_MSG_IF(!scripted && radio == "mmwave",
                    "the per-RB rain field needs the nr backend (AddSpectrumChannelLoss); "
                    "use --scripted=1 with --radio=mmwave");

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

    // The gateway is a fixed Ka-band ground site at the sub-point.
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
        rs.SetNumerology(3); // 120 kHz SCS: 20 GHz is Ka-band NTN (TS 38.101-5 FR2-NTN, 60/120 kHz)
    }
    rs.SetSimTime(Seconds(simSeconds));
    rs.SetOutputDir(outputDir);
    rs.SetRunTag("ntn-sionna-rain-event-traffic");
    rs.SetCarrierFrequencyHz(freqGHz * 1e9);
    if (legacyConductedDbm > 0.0)
    {
        // Old operating point: CONDUCTED power at the array input (NT-02).
        rs.SetSatConductedPowerDbm(legacyConductedDbm);
    }
    else
    {
        // TR 38.821 Table 6.1.1.1-1 Set-1 LEO-600 Ka: 4 dBW/MHz; Table
        // 6.1.1.1-2 VSAT 39.7 dBi, G/T 15.9 dB/K (equivalent NF), 1x1 panel.
        rs.ApplyTr38821Set1Defaults();
    }
    if (!scripted)
    {
        // The legacy Ntn38811ExcessLossModel carries its own P.676 gas term
        // and an iid-per-second "P.618" scintillation draw (about 1.2 dB rms at
        // 12 GHz, 1.6 dB at 20 GHz): gas would be counted twice next to this
        // example's atmospheric model, and the draw is not P.618-13 2.4.1.
        rs.SetTr38811ExcessLoss(false);
    }
    rs.SetSmallScaleFading(fading);
    rs.Build(satNodes, gndNodes);
    {
        // Independent budget at t=0 (zenith, slant 550 km), no fading, no rain:
        // EIRP - FSPL + G/T - k - 10 log10(B_occ) (TR 38.821 6.1.3 CNR form).
        const double lambda = 299792458.0 / (freqGHz * 1e9);
        const double slant = satEnu->GetPosition().z - 5.0;
        const double fspl = 20.0 * std::log10(4.0 * M_PI * slant / lambda);
        const double eirp = rs.GetEffectiveEirpDbm();
        const double occHz = rs.GetBwpPrbs() * 12.0 * rs.GetScsKhz() * 1e3;
        if (legacyConductedDbm <= 0.0)
        {
            const double cnr = (eirp - 30.0) - fspl + 15.9 + 228.6 - 10.0 * std::log10(occHz);
            std::printf("#   payload: EIRP %.2f dBm over %.2f MHz occupied, VSAT 39.7 dBi, "
                        "G/T 15.9 dB/K; FSPL %.2f dB at %.0f km -> budget CNR %.2f dB "
                        "(before gas and rain)\n",
                        eirp, occHz / 1e6, fspl, slant / 1e3, cnr);
        }
        else
        {
            std::printf("#   payload: legacy conducted %.1f dBm (effective EIRP %.2f dBm)\n",
                        legacyConductedDbm, eirp);
        }
    }

    Ptr<MobilityModel> gndMob = gndNodes.Get(0)->GetObject<MobilityModel>();
    uint64_t lastRx = 0;
    // Per-tick record for the summary (measured SINR vs rain on the path).
    struct Tick
    {
        double t, rainDb, sinr, mbps;
    };
    std::vector<Tick> ticks;

    Ptr<NtnAtmosphericLossChain> chain;
    Ptr<NtnAtmosphericPropagationLossModel> atmo;
    Ptr<ThzNtnSpectrumAtmosphereModel> spec;
    if (scripted)
    {
        // Legacy: ITU-R P.676 gaseous + P.618/P.838 rain as a scalar EXCESS
        // chained after Friis; the rain schedule reconfigures it.
        chain = CreateObject<NtnAtmosphericLossChain>();
        chain->SetFrequencyHz(freqGHz * 1e9);
        chain->SetRainRateMmH(0.0);
        atmo = CreateObject<NtnAtmosphericPropagationLossModel>();
        atmo->SetChain(chain);
        rs.AddExtraPropagationLoss(atmo);
        const double tBuild = 0.30 * simSeconds;
        const double tPeak = 0.40 * simSeconds;
        const double tEase = 0.55 * simSeconds;
        const double tClear = 0.65 * simSeconds;
        Simulator::Schedule(Seconds(tBuild),
                            [chain, peakRainMmH] { chain->SetRainRateMmH(0.25 * peakRainMmH); });
        Simulator::Schedule(Seconds(tPeak),
                            [chain, peakRainMmH] { chain->SetRainRateMmH(peakRainMmH); });
        Simulator::Schedule(Seconds(tEase),
                            [chain, peakRainMmH] { chain->SetRainRateMmH(0.50 * peakRainMmH); });
        Simulator::Schedule(Seconds(tClear), [chain] { chain->SetRainRateMmH(0.0); });
        std::printf("#   scripted rain: clear->25%%@%.0fs ->peak@%.0fs ->50%%@%.0fs ->clear@%.0fs\n",
                    tBuild, tPeak, tEase, tClear);
    }
    else
    {
        // Geometry: the cell sits on the satellite ground track (direction of
        // the satellite's horizontal velocity at t=0) and drifts toward the
        // station. EXCELL profile, P.839 rain height (mid-latitude table).
        Vector v = satEnu->GetVelocity();
        const double vh = std::max(std::hypot(v.x, v.y), 1e-9);
        const double ux = v.x / vh;
        const double uy = v.y / vh;
        Ptr<ThzNtnRainCellField> field = CreateObject<ThzNtnRainCellField>();
        field->SetRainHeightFromClimate(itu::Itu618LossModel::ClimateRegion::midlat_summer);
        ThzNtnRainCellField::Cell c;
        c.x0 = cellOffsetM * ux;
        c.y0 = cellOffsetM * uy;
        c.vx = -cellSpeedMps * ux;
        c.vy = -cellSpeedMps * uy;
        c.peakRate = peakRainMmH;
        c.r0 = cellR0M;
        field->AddCell(c);
        spec = CreateObject<ThzNtnSpectrumAtmosphereModel>();
        spec->SetRainField(field);
        rs.AddSpectrumChannelLoss(spec);
        std::printf("#   rain cell: EXCELL R_peak %.0f mm/h, r0 %.0f m, centre %.0f m along the "
                    "ground track at t=0, drifting %.1f m/s toward the station; rain height "
                    "%.2f km (P.839 table); per-RB P.838-3 path integral + P.676-13 gas\n",
                    peakRainMmH, cellR0M, cellOffsetM, cellSpeedMps, field->GetRainHeightKm());
    }

    rs.InstallTraffic(NtnRealStackHelper::TrafficProfile::EmbbStreaming,
                      Seconds(1.0), Seconds(simSeconds - 0.5));
    rs.EnableAiFlowMonitor("ntn-sionna-rain-event-traffic"); // WS2 KPM series (TS 28.552 names)

    std::printf("# %5s  %7s  %7s  %8s  %8s  %8s  %8s  %8s  %9s\n",
                "t_s", "elev", "rainR", "rain_dB", "gas_dB", "rbSpread", "sinr_dB", "tbler",
                "goodput");
    rs.RegisterPeriodicCallback(
        Seconds(1.0),
        [&rs, atmo, chain, spec, gndMob, satEnu, &lastRx, &ticks](Time now) {
            const double elev = ElevDegEnu(gndMob->GetPosition(), satEnu->GetPosition());
            const double sinr = rs.GetUeRecentSinrDb(0);
            const double tbler = rs.GetUeRecentTbler(0);
            const uint64_t rx = rs.GetUeRxBytes(0);
            const double mbps = (rx - lastRx) * 8.0 / 1e6;
            lastRx = rx;
            double rate;
            double rainDb;
            double gasDb;
            double spread = 0.0;
            if (spec)
            {
                rate = spec->GetLastPathRainRate();
                rainDb = spec->GetLastRainCentreDb();
                gasDb = spec->GetLastGasCentreDb();
                spread = spec->GetLastMaxLossDb() - spec->GetLastMinLossDb();
            }
            else
            {
                rate = chain->GetRainRateMmH();
                rainDb = atmo->GetLastLossDb();
                gasDb = 0.0;
            }
            ticks.push_back({now.GetSeconds(), rainDb, sinr, mbps});
            std::printf("  %5.1f  %7.2f  %7.2f  %8.2f  %8.3f  %8.3f  %8.2f  %8.3f  %9.3f\n",
                        now.GetSeconds(), elev, rate, rainDb, gasDb, spread, sinr, tbler, mbps);
        });

    Simulator::Stop(Seconds(simSeconds));
    Simulator::Run();
    rs.Collect();
    rs.WriteHealthReport();

    // Measured SINR at the least and the most rain-attenuated ticks (t >= 2 s,
    // a PHY sample exists).
    const Tick* lo = nullptr;
    const Tick* hi = nullptr;
    for (const auto& t : ticks)
    {
        if (t.t < 2.0 || std::isnan(t.sinr))
        {
            continue;
        }
        if (!lo || t.rainDb < lo->rainDb)
        {
            lo = &t;
        }
        if (!hi || t.rainDb > hi->rainDb)
        {
            hi = &t;
        }
    }
    if (lo && hi)
    {
        std::printf("# least rain t=%.0fs (%.2f dB): SINR %.2f dB, %.3f Mb/s | most rain t=%.0fs "
                    "(%.2f dB): SINR %.2f dB, %.3f Mb/s | measured fade %.2f dB vs path "
                    "attenuation difference %.2f dB\n",
                    lo->t, lo->rainDb, lo->sinr, lo->mbps, hi->t, hi->rainDb, hi->sinr, hi->mbps,
                    lo->sinr - hi->sinr, hi->rainDb - lo->rainDb);
    }
    std::printf("# === summary ===  cell SINR=%.2f dB TBLER=%.4f throughput=%.3f Mbps "
                "(%s)\n",
                rs.GetMeanDlSinrDb(), rs.GetMeanDlTbler(), rs.GetRxThroughputMbps(),
                scripted ? "scripted rain series" : "rain-cell field, per-RB P.838");

    Simulator::Destroy();
    return 0;
}
