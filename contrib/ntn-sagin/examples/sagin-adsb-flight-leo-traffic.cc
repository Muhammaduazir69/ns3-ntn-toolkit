/*
 * SPDX-License-Identifier: GPL-2.0-only
 * Copyright (c) 2026 Muhammad Uzair (ns3-ntn-toolkit)
 *
 * sagin-adsb-flight-leo-traffic — an in-flight-connectivity terminal aboard a
 * commercial aircraft whose motion is replayed from an ADS-B trace
 * (OpenSkyMobilityModel) is served by a passing LEO satellite over a REAL mmwave
 * NR NTN cell (NtnRealStackHelper: SpectrumPhy + MAC + HARQ + RLC/PDCP + RRC +
 * EPC). The aircraft cruises along the ADS-B trace while the satellite crosses
 * overhead; the DL SINR/TBLER/throughput are MEASURED off the mmwave PHY trace
 * as the geometry evolves — no closed-form FSPL->SINR, no sigmoid error model.
 *
 * Covers the OpenSkyMobilityModel / OpenSkyAdsbTrace classes with a real
 * measured data plane. Quick test:  --simSeconds=30 --numUes=1
 */
#include "ns3/constant-velocity-mobility-model.h"
#include "ns3/ntn-tr38811-mobility-model.h"
#include "ns3/sgp4-mobility-model.h"
#include "ns3/walker-constellation.h"
#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/ntn-real-stack-helper.h"

#include "ns3/opensky-adsb-trace.h"
#include "ns3/opensky-mobility-model.h"

#include <cmath>
#include <cstdio>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("SaginAdsbFlightLeoTraffic");

namespace
{
NtnRealStackHelper* g_rs = nullptr;
Ptr<MobilityModel> g_ac;
double g_simTime = 30.0;

void
LinkProbe()
{
    if (Simulator::Now().GetSeconds() >= g_simTime)
    {
        return;
    }
    const Vector u = g_ac->GetPosition();
    const double sinr = g_rs->GetUeRecentSinrDb(0);
    std::printf("  %6.1f  ac=(%9.0f,%6.0f)  measSINR=%7.2f dB\n",
                Simulator::Now().GetSeconds(), u.x, u.z, std::isnan(sinr) ? 0.0 : sinr);
    Simulator::Schedule(Seconds(2.0), &LinkProbe);
}
} // namespace

int
main(int argc, char* argv[])
{
    double simSeconds = 30.0;
    double leoAltKm = 550.0;
    double satSpeed = 7500.0;
    double freqGHz = 12.0;
    double satEirpDbm = 70.0; // Ku-band budget so the measured SINR lands realistically
    double cruiseAltM = 11000.0;
    double cruiseSpeedMps = 250.0;
    std::string outputDir = "sagin-adsb-flight-leo-output";

    CommandLine cmd(__FILE__);
    cmd.AddValue("simSeconds", "Simulation duration (s)", simSeconds);
    cmd.AddValue("leoAltKm", "LEO altitude (km)", leoAltKm);
    cmd.AddValue("satSpeed", "LEO ground-track speed (m/s)", satSpeed);
    cmd.AddValue("freqGHz", "Carrier frequency (GHz)", freqGHz);
    cmd.AddValue("satEirpDbm", "Satellite EIRP / gNB Tx power (dBm)", satEirpDbm);
    cmd.AddValue("cruiseAltM", "Aircraft cruise altitude (m)", cruiseAltM);
    cmd.AddValue("cruiseSpeedMps", "Aircraft cruise speed (m/s)", cruiseSpeedMps);
    cmd.AddValue("outputDir", "Output directory", outputDir);
    cmd.Parse(argc, argv);
    g_simTime = simSeconds;

    // Build an ADS-B trace inline: a great-circle cruise leg heading east.
    sagin::OpenSkyAdsbTrace trace;
    trace.icao24 = "abcdef";
    const double refLat = 50.0, refLon = 8.0;
    for (double t = 0.0; t <= simSeconds + 20.0; t += 10.0)
    {
        sagin::OpenSkySample smp;
        smp.time_s = t;
        const double mEast = cruiseSpeedMps * t;
        smp.lat_deg = refLat;
        smp.lon_deg = refLon + mEast / (111320.0 * std::cos(refLat * M_PI / 180.0));
        smp.alt_m = cruiseAltM;
        smp.velocity_mps = cruiseSpeedMps;
        smp.heading_deg = 90.0;
        trace.samples.push_back(smp);
    }

    // Aircraft terminal = UE (real ADS-B mobility); LEO satellite = mmwave gNB.
    NodeContainer ueNodes;
    ueNodes.Create(1);
    Ptr<sagin::OpenSkyMobilityModel> ac = CreateObject<sagin::OpenSkyMobilityModel>();
    ac->SetReference(refLat, refLon, 0.0);
    ac->SetTrace(trace);
    ueNodes.Get(0)->AggregateObject(ac);
    g_ac = ac;

    NodeContainer satNodes;
    satNodes.Create(1);
    // Real SGP4 orbit projected into the scenario's local ENU frame: the
    // satellite passes overhead near t=0 and recedes with genuine orbital
    // dynamics (no straight-line placeholder).
    ns3::ntncon::WalkerConfig wcfgSat;
    wcfgSat.num_planes = 1;
    wcfgSat.total_sats = 80;
    wcfgSat.altitude_km = leoAltKm;
    wcfgSat.inclination_deg = 53.0;
    wcfgSat.epoch_unix_s = 1735689600.0;
    const auto satElements = ns3::ntncon::WalkerConstellation::BuildDelta(wcfgSat);
    Ptr<ns3::ntncon::Sgp4MobilityModel> satSgp4 =
        CreateObject<ns3::ntncon::Sgp4MobilityModel>();
    satSgp4->SetElements(satElements[0]);
    double satSubLat, satSubLon, satSubAlt;
    satSgp4->GetGeodetic(satSubLat, satSubLon, satSubAlt);
    Ptr<NtnEnuProjectionMobilityModel> sat = CreateObject<NtnEnuProjectionMobilityModel>();
    sat->SetSource(satSgp4);
    sat->SetReference(satSubLat, satSubLon, 0.0);
    satNodes.Get(0)->AggregateObject(sat);

    NtnRealStackHelper rs;
    rs.SetSimTime(Seconds(simSeconds));
    rs.SetOutputDir(outputDir);
    rs.SetRunTag("sagin-adsb-flight-leo-traffic");
    rs.SetCarrierFrequencyHz(freqGHz * 1e9);
    rs.SetSatEirpDbm(satEirpDbm);
    rs.Build(satNodes, ueNodes);
    rs.InstallTraffic(NtnRealStackHelper::TrafficProfile::EmbbStreaming,
                      Seconds(1.0), Seconds(simSeconds - 0.5));
    rs.EnableAiFlowMonitor("sagin-adsb-flight-leo-traffic"); // WS2 KPM series (TS 28.552 names)
    g_rs = &rs;

    std::printf("# sagin-adsb-flight-leo-traffic (ADS-B aircraft terminal on a real mmwave NR cell)\n"
                "#   sim=%.0fs leoAlt=%.0fkm freq=%.0fGHz EIRP=%.1fdBm cruise=%.0fm/%.0fm/s\n",
                simSeconds, leoAltKm, freqGHz, satEirpDbm, cruiseAltM, cruiseSpeedMps);

    Simulator::Schedule(Seconds(2.0), &LinkProbe);
    Simulator::Stop(Seconds(simSeconds));
    Simulator::Run();
    rs.Collect();
    rs.WriteHealthReport();

    std::printf("# === summary ===  measured SINR=%.2f dB  measured TBLER=%.4f  "
                "measured throughput=%.3f Mbps\n",
                rs.GetMeanDlSinrDb(), rs.GetMeanDlTbler(), rs.GetRxThroughputMbps());
    Simulator::Destroy();
    return 0;
}
