/*
 * SPDX-License-Identifier: GPL-2.0-only
 * Copyright (c) 2026 Muhammad Uzair (ns3-ntn-toolkit)
 *
 * sagin-hst-leo-traffic — a high-speed-train terminal (HstMobilityModel, a
 * TR 38.901-style HST track) is served by a passing LEO satellite over a REAL
 * mmwave NR NTN cell (NtnRealStackHelper: SpectrumPhy + MAC + HARQ + RLC/PDCP +
 * RRC + EPC). The DL SINR/TBLER/throughput are MEASURED off the mmwave PHY trace
 * as the train races along the track and the satellite crosses overhead — no
 * closed-form FSPL->SINR, no sigmoid error model.
 *
 * Covers the HstMobilityModel / HstTraceGenerator classes with a real measured
 * data plane. Quick test:  --simSeconds=30 --trainSpeedKmh=500
 */
#include "ns3/constant-velocity-mobility-model.h"
#include "ns3/ntn-tr38811-mobility-model.h"
#include "ns3/sgp4-mobility-model.h"
#include "ns3/walker-constellation.h"
#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/ntn-real-stack-helper.h"

#include "ns3/hst-mobility-model.h"
#include "ns3/hst-trace.h"

#include <cmath>
#include <cstdio>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("SaginHstLeoTraffic");

namespace
{
NtnRealStackHelper* g_rs = nullptr;
Ptr<MobilityModel> g_train;
double g_simTime = 30.0;

void
LinkProbe()
{
    if (Simulator::Now().GetSeconds() >= g_simTime)
    {
        return;
    }
    const Vector u = g_train->GetPosition();
    const double sinr = g_rs->GetUeRecentSinrDb(0);
    std::printf("  %6.1f  train=(%9.0f,%6.0f)  measSINR=%7.2f dB\n",
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
    double freqGHz = 2.0;
    double satEirpDbm = -1.0; // sentinel: backend-appropriate default chosen below
    double trainSpeedKmh = 500.0;
    std::string outputDir = "sagin-hst-leo-output";
    std::string radio = "nr"; // radio backend: "nr" (5G-LENA FR1) | "mmwave" (FR2)

    CommandLine cmd(__FILE__);
    cmd.AddValue("simSeconds", "Simulation duration (s)", simSeconds);
    cmd.AddValue("leoAltKm", "LEO altitude (km)", leoAltKm);
    cmd.AddValue("satSpeed", "LEO ground-track speed (m/s)", satSpeed);
    cmd.AddValue("freqGHz", "Carrier frequency (GHz)", freqGHz);
    cmd.AddValue("satEirpDbm", "Satellite EIRP / gNB Tx power (dBm); -1 = backend default",
                 satEirpDbm);
    cmd.AddValue("radio", "Radio backend: nr (FR1) or mmwave", radio);
    cmd.AddValue("trainSpeedKmh", "Train speed (km/h)", trainSpeedKmh);
    cmd.AddValue("outputDir", "Output directory", outputDir);
    cmd.Parse(argc, argv);

    // Backend-appropriate EIRP default (honoured only if the user did not set it):
    // nr's Friis LEO link needs ~70 dBm for a healthy SINR; mmwave keeps 55 dBm.
    if (satEirpDbm < 0.0)
    {
        satEirpDbm = (radio == "mmwave") ? 55.0 : 70.0;
    }
    g_simTime = simSeconds;

    // Generate a TR 38.901-style HST track at the requested speed.
    sagin::HstTrace trace =
        sagin::HstTraceGenerator::Generate(trainSpeedKmh, 150.0, simSeconds + 60.0, 200);

    // Train terminal = UE (real HST mobility); LEO satellite = mmwave gNB.
    NodeContainer ueNodes;
    ueNodes.Create(1);
    Ptr<sagin::HstMobilityModel> train = CreateObject<sagin::HstMobilityModel>();
    train->SetCarrierFrequencyHz(freqGHz * 1e9);
    train->SetTrace(trace);
    ueNodes.Get(0)->AggregateObject(train);
    g_train = train;

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
    rs.SetRunTag("sagin-hst-leo-traffic");
    rs.SetCarrierFrequencyHz(freqGHz * 1e9);
    rs.SetSatEirpDbm(satEirpDbm);
    rs.SetRadioBackend(radio == "mmwave" ? NtnRealStackHelper::RadioBackend::Mmwave
                                         : NtnRealStackHelper::RadioBackend::Nr);
    if (radio != "mmwave")
    {
        rs.SetNumerology(1); // FR1 30 kHz SCS
    }
    rs.Build(satNodes, ueNodes);
    rs.InstallTraffic(NtnRealStackHelper::TrafficProfile::EmbbStreaming,
                      Seconds(1.0), Seconds(simSeconds - 0.5));
    rs.EnableAiFlowMonitor("sagin-hst-leo-traffic"); // WS2 KPM series (TS 28.552 names)
    g_rs = &rs;

    std::printf("# sagin-hst-leo-traffic (HST terminal on a real mmwave NR cell)\n"
                "#   sim=%.0fs leoAlt=%.0fkm freq=%.1fGHz EIRP=%.1fdBm trainSpeed=%.0fkm/h\n",
                simSeconds, leoAltKm, freqGHz, satEirpDbm, trainSpeedKmh);

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
