/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only
//
// ntn-observability-traffic — instruments a REAL LEO downlink with the
// toolkit's observability stack. The downlink is a REAL mmwave NR NTN cell
// (NtnRealStackHelper: SpectrumPhy + MAC + RLC/PDCP + RRC + EPC); every second
// an NtnInfluxSink exports the MEASURED KPIs — delivered goodput (UE
// PacketSink), SINR/TBLER (PHY trace RxPacketTraceUe), elevation and slant
// range (live SGP4 geometry) — as InfluxDB line protocol (to a file by
// default, or UDP to a Telegraf/InfluxDB endpoint). The exported time series
// is a faithful telemetry trace of the pass: nothing closed-form anywhere.
//
// Audit fix (AI-Native ORAN-NTN adoption WS0): the previous version
// drove a P2P RateErrorModel from a sigmoid SnrToPer() and exported THAT
// formula value as "sinr_db" — the dashboard observed a synthetic link.
//
// Quick test:  --simSeconds=40 --out=/tmp/ntn-obs.lp
#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/ntn-influx-sink.h"
#include "ns3/ntn-metric-schema.h"
#include "ns3/ntn-real-stack-helper.h"
#include "ns3/ntn-tr38811-mobility-model.h"
#include "ns3/sgp4-mobility-model.h"
#include "ns3/walker-constellation.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>

using namespace ns3;
using ns3::ntnobs::NtnInfluxSink;
using ns3::ntnobs::Point;

NS_LOG_COMPONENT_DEFINE("NtnObservabilityTraffic");

namespace
{

double
DistM(const Vector& a, const Vector& b)
{
    const double dx = a.x - b.x, dy = a.y - b.y, dz = a.z - b.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double
ElevDegEnu(const Vector& gnd, const Vector& sat)
{
    const double dx = sat.x - gnd.x;
    const double dy = sat.y - gnd.y;
    const double dz = sat.z - gnd.z;
    const double horiz = std::max(std::sqrt(dx * dx + dy * dy), 1e-3);
    return std::atan2(dz, horiz) * 180.0 / M_PI;
}

/// OBS-16: IEEE Std 521-2019 letter band of the carrier, so the `band` tag
/// follows --freqGHz instead of the fixed "Ku" it used to carry.
std::string
Ieee521Band(double fGHz)
{
    if (fGHz < 1.0)
        return "UHF";
    if (fGHz < 2.0)
        return "L";
    if (fGHz < 4.0)
        return "S";
    if (fGHz < 8.0)
        return "C";
    if (fGHz < 12.0)
        return "X";
    if (fGHz < 18.0)
        return "Ku";
    if (fGHz < 27.0)
        return "K";
    if (fGHz < 40.0)
        return "Ka";
    return "V";
}

} // namespace

int
main(int argc, char* argv[])
{
    double simSeconds = 40.0;
    double leoAltKm = 550.0;
    double freqGHz = 12.0; // Ku-band
    // Conducted power at the array input (NT-02); the radiated EIRP that
    // sim_health.csv reports adds the array gain (75 dBm -> ~93 dBm EIRP).
    double satEirpDbm = 75.0;
    std::string radio = "nr"; // radio backend: "nr" (5G-LENA FR1) | "mmwave" (FR2)
    std::string outPath = "ntn-observability-traffic.lp";
    std::string influxHost = ""; // empty = file transport
    uint16_t influxPort = 8089;
    std::string outputDir = "ntn-observability-traffic-output";

    CommandLine cmd(__FILE__);
    cmd.AddValue("simSeconds", "Simulation duration (s)", simSeconds);
    cmd.AddValue("leoAltKm", "Satellite altitude (km)", leoAltKm);
    cmd.AddValue("freqGHz", "Carrier frequency (GHz)", freqGHz);
    cmd.AddValue("satEirpDbm",
                 "Satellite CONDUCTED Tx power at the array input (dBm); the "
                 "radiated EIRP adds the array gain (see effective_eirp_dbm)",
                 satEirpDbm);
    cmd.AddValue("radio", "Radio backend: nr (FR1) or mmwave", radio);
    cmd.AddValue("out", "InfluxDB line-protocol output file", outPath);
    cmd.AddValue("influxHost", "InfluxDB/Telegraf UDP host (empty=file)", influxHost);
    cmd.AddValue("influxPort", "InfluxDB/Telegraf UDP port", influxPort);
    cmd.AddValue("outputDir", "Output directory for sim_health.csv", outputDir);
    cmd.Parse(argc, argv);
    // A bare --out name goes with the rest of the run (sim_health.csv,
    // _kpm_series.*) instead of the launch directory; a path still wins.
    if (!outputDir.empty() && outPath.find('/') == std::string::npos)
    {
        outPath = outputDir + (outputDir.back() == '/' ? "" : "/") + outPath;
        SystemPath::MakeDirectories(outputDir); // the sink flushes from t=5 s
    }

    std::printf("# ntn-observability-traffic (REAL radio + InfluxDB KPI export)\n");
    std::printf("#   sim=%.0fs alt=%.0fkm freq=%.0fGHz EIRP=%.1fdBm export=%s\n",
                simSeconds, leoAltKm, freqGHz, satEirpDbm,
                influxHost.empty() ? outPath.c_str() : "UDP");

    NodeContainer satNodes;
    satNodes.Create(1);
    NodeContainer ueNodes;
    ueNodes.Create(1);

    // Real SGP4 orbit projected into the local ENU frame: the serving Walker
    // element is at zenith at t=0 and recedes with genuine orbital dynamics.
    ns3::ntncon::WalkerConfig wcfg;
    wcfg.num_planes = 1;
    wcfg.total_sats = 80;
    wcfg.altitude_km = leoAltKm;
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

    // Fixed terminal (TR 38.821 Sec. 6.1 VSAT class, the Ku-band reference
    // terminal) at the ENU origin under the sub-satellite point at t=0. A
    // fixed class is a constant position by definition, not a placeholder.
    MobilityHelper mob;
    mob.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    Ptr<ListPositionAllocator> uePos = CreateObject<ListPositionAllocator>();
    uePos->Add(Vector(0.0, 0.0, 1.5));
    mob.SetPositionAllocator(uePos);
    mob.Install(ueNodes);

    NtnRealStackHelper rs;
    rs.SetRadioBackend(radio == "mmwave" ? NtnRealStackHelper::RadioBackend::Mmwave
                                         : NtnRealStackHelper::RadioBackend::Nr);
    if (radio != "mmwave")
    {
        rs.SetNumerology(1); // FR1 30 kHz SCS
    }
    rs.SetSimTime(Seconds(simSeconds));
    rs.SetOutputDir(outputDir);
    rs.SetRunTag("ntn-observability-traffic");
    rs.SetCarrierFrequencyHz(freqGHz * 1e9);
    // NT-02: declared as CONDUCTED power at the array input. This carrier has
    // no TR 38.821 Set-1 reference in the toolkit, so the EIRP health gate
    // reports "not asserted" rather than certifying an uncalibrated budget.
    rs.SetSatConductedPowerDbm(satEirpDbm);
    // Wave 2 M2: TS 28.552 KPM from the RAN counters (DRB.RlcSduDelayDl from
    // RLC, TB.TotNbrDl / TB.ErrTotNbrDl as integer counts, RRU.PrbUsedDl);
    // TB.ErrTotNbrDl used to carry a BLER fraction (e.g. 1.56e-11).
    rs.SetKpmFromRanCounters(true);
    rs.Build(satNodes, ueNodes);
    rs.InstallTraffic(NtnRealStackHelper::TrafficProfile::EmbbStreaming,
                      Seconds(1.0), Seconds(simSeconds - 0.5));
    // Canonical KPM wiring: per-flow TS 28.552 series auto-exported to
    // ntn-observability-traffic_kpm_series.{csv,lp} at end of simulation.
    rs.EnableAiFlowMonitor(outputDir.empty() ? std::string("ntn-observability-traffic")
                                             : outputDir + "/ntn-observability-traffic");

    // Observability sink.
    Ptr<NtnInfluxSink> influx = CreateObject<NtnInfluxSink>();
    influx->SetUseSimulationTime(true);
    if (influxHost.empty())
    {
        influx->SetTransport(NtnInfluxSink::Transport::File);
        influx->SetFilePath(outPath);
    }
    else
    {
        influx->SetTransport(NtnInfluxSink::Transport::Udp);
        influx->SetUdpEndpoint(influxHost, influxPort);
    }
    influx->SetFlushPeriod(Seconds(5.0));

    std::printf("# %5s  %7s  %9s  %8s  %8s  %9s\n",
                "t_s", "elev", "slant_km", "sinr_dB", "tbler", "goodput");

    // 1 Hz export tick: every value is MEASURED (PHY trace / PacketSink) or
    // real ephemeris geometry.
    Ptr<MobilityModel> ueMob = ueNodes.Get(0)->GetObject<MobilityModel>();
    uint64_t lastRx = 0;
    rs.RegisterPeriodicCallback(
        Seconds(1.0),
        [&rs, influx, ueMob, satEnu, &lastRx, freqGHz](Time now) {
            const Vector u = ueMob->GetPosition();
            const Vector s = satEnu->GetPosition();
            const double elev = ElevDegEnu(u, s);
            const double rangeM = DistM(u, s);
            const double sinr = rs.GetUeRecentSinrDb(0);
            const double tbler = rs.GetUeRecentTbler(0);
            const uint64_t tot = rs.GetUeRxBytes(0);
            const double mbps = (tot - lastRx) * 8.0 / 1e6;
            lastRx = tot;

            Point p;
            // OBS-13: schema constants, not bare literals. This example invented
            // its measurement and five field names, none of which appeared in
            // ntn-metric-schema.h or in any shipped dashboard.
            p.measurement = ntnobs::measurement::kDownlink;
            p.tags[ntnobs::tag::kLink] = "leo-gnd";
            p.tags[ntnobs::tag::kBand] = Ieee521Band(freqGHz);
            p.tags[ntnobs::tag::kProvenance] = "phy-trace";
            p.fieldsFloat[ntnobs::field::kGoodputMbps] = mbps;
            if (!std::isnan(sinr))
            {
                p.fieldsFloat[ntnobs::field::kSinrDb] = sinr;
            }
            if (!std::isnan(tbler))
            {
                p.fieldsFloat[ntnobs::field::kTbler] = tbler;
            }
            p.fieldsFloat[ntnobs::field::kElevationDeg] = elev;
            p.fieldsFloat[ntnobs::field::kSlantRangeKm] = rangeM / 1000.0;
            p.fieldsInt[ntnobs::field::kRxBytesTotal] = static_cast<long long>(tot);
            p.timestamp = now;
            influx->Push(p);

            // Before the first PHY sample (t=1 s, traffic starts at 1 s) there
            // is no SINR/TBLER; print "-" rather than "nan" (the export above
            // already omits the field).
            char sinrTxt[16] = "       -";
            char tblerTxt[16] = "       -";
            if (!std::isnan(sinr))
            {
                std::snprintf(sinrTxt, sizeof(sinrTxt), "%8.2f", sinr);
            }
            if (!std::isnan(tbler))
            {
                std::snprintf(tblerTxt, sizeof(tblerTxt), "%8.3f", tbler);
            }
            std::printf("  %5.1f  %7.2f  %9.1f  %s  %s  %9.3f\n",
                        now.GetSeconds(), elev, rangeM / 1000.0, sinrTxt, tblerTxt, mbps);
        });

    influx->Start();
    Simulator::Stop(Seconds(simSeconds));
    Simulator::Run();
    influx->Flush();
    influx->Stop();
    rs.Collect();
    rs.WriteHealthReport();

    std::printf("# === summary ===  measured cell SINR=%.2f dB TBLER=%.4f "
                "throughput=%.3f Mbps  KPIs exported to %s\n",
                rs.GetMeanDlSinrDb(), rs.GetMeanDlTbler(), rs.GetRxThroughputMbps(),
                influxHost.empty() ? outPath.c_str() : "UDP endpoint");
    Simulator::Destroy();
    return 0;
}
