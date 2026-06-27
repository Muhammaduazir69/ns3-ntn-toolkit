/*
 * SPDX-License-Identifier: GPL-2.0-only
 * Copyright (c) 2026 Muhammad Uzair (ns3-ntn-toolkit, Workstream W7)
 *
 * ntn-v2x-maritime-ais — real maritime NTN access.
 *
 * Audit finding (ntn-v2x §2): MaritimeMobilityModel (billiard-ball box-bounce
 * motion) was never instantiated by any example, so the maritime scenario was
 * dead, and its motion was synthetic. This example replaces it with the EXISTING
 * ntn-sagin AIS pipeline: AisDanishImporter parses a real Danish Maritime
 * Authority AIS CSV export, and AisMobilityModel replays one vessel's recorded
 * lat/lon/SOG/COG track under ns-3 simulation time. The vessel terminal is a UE
 * on a REAL mmwave NR NTN cell (NtnRealStackHelper: SpectrumPhy + MAC + HARQ +
 * RLC/PDCP + RRC + EPC), so its LEO link SINR / TBLER / throughput are MEASURED
 * off the mmwave PHY trace as the satellite crosses overhead — no closed-form
 * FSPL->SINR and no box-bounce mobility.
 *
 * Reuses ntn-sagin classes verbatim (AisDanishImporter, AisMaritimeTrace,
 * AisMobilityModel) and the real sample trace shipped at
 * contrib/ntn-sagin/data/ais-sample-trace.csv.
 *
 * Usage:
 *   ./ns3 run "ntn-v2x-maritime-ais --simSeconds=30"
 *   ./ns3 run "ntn-v2x-maritime-ais --aisTrace=<path/to/aisdk.csv>"
 */
#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/ntn-real-stack-helper.h"
#include "ns3/ntn-tr38811-mobility-model.h"
#include "ns3/sgp4-mobility-model.h"
#include "ns3/walker-constellation.h"

#include "ns3/ais-maritime-trace.h"
#include "ns3/ais-mobility-model.h"

#include <cmath>
#include <cstdio>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("NtnV2xMaritimeAis");

namespace
{
NtnRealStackHelper* g_rs = nullptr;
Ptr<MobilityModel> g_vessel;
double g_simTime = 30.0;

void
LinkProbe()
{
    if (Simulator::Now().GetSeconds() >= g_simTime)
    {
        return;
    }
    const Vector u = g_vessel->GetPosition();
    const double sinr = g_rs->GetUeRecentSinrDb(0);
    std::printf("  %6.1f  vessel=(%9.0f,%8.0f)  measSINR=%7.2f dB\n",
                Simulator::Now().GetSeconds(), u.x, u.y, std::isnan(sinr) ? 0.0 : sinr);
    Simulator::Schedule(Seconds(2.0), &LinkProbe);
}
} // namespace

int
main(int argc, char* argv[])
{
    double simSeconds = 30.0;
    double leoAltKm = 550.0;
    double freqGHz = 2.0;
    double satEirpDbm = -1.0; // sentinel: backend-appropriate default chosen below
    std::string radio = "nr"; // radio backend: "nr" (5G-LENA FR1) | "mmwave" (FR2)
    std::string aisTrace = "contrib/ntn-sagin/data/ais-sample-trace.csv";
    std::string outputDir = "ntn-v2x-maritime-ais-output";

    CommandLine cmd(__FILE__);
    cmd.AddValue("simSeconds", "Simulation duration (s)", simSeconds);
    cmd.AddValue("leoAltKm", "LEO altitude (km)", leoAltKm);
    cmd.AddValue("freqGHz", "Carrier frequency (GHz)", freqGHz);
    cmd.AddValue("satEirpDbm", "Satellite EIRP / gNB Tx power (dBm); -1 = backend default", satEirpDbm);
    cmd.AddValue("radio", "Radio backend: nr (FR1) or mmwave", radio);
    cmd.AddValue("aisTrace",
                 "Real Danish Maritime AIS CSV export (no synthetic fallback). "
                 "Defaults to the shipped ntn-sagin sample trace.",
                 aisTrace);
    cmd.AddValue("outputDir", "Output directory", outputDir);
    cmd.Parse(argc, argv);
    g_simTime = simSeconds;

    // Backend-appropriate EIRP default (honoured only if the user did not set it):
    // nr's Friis LEO link needs ~70 dBm for a healthy SINR; mmwave keeps 55 dBm.
    if (satEirpDbm < 0.0)
    {
        satEirpDbm = (radio == "mmwave") ? 55.0 : 70.0;
    }

    // Parse a REAL Danish Maritime AIS export and take the first vessel's track.
    sagin::AisDanishImporter importer;
    auto traces = importer.LoadCsv(aisTrace);
    if (traces.empty())
    {
        NS_FATAL_ERROR("no AIS vessel tracks parsed from '"
                       << aisTrace
                       << "'. Supply a real Danish Maritime AIS CSV export "
                          "(https://web.ais.dk/aisdata/).");
    }
    // Pick the vessel with the richest (longest) track so we replay genuine
    // motion rather than an anchored single-fix target.
    const sagin::AisMaritimeTrace* best = nullptr;
    for (const auto& [mmsi, tr] : traces)
    {
        if (!best || tr.samples.size() > best->samples.size())
        {
            best = &tr;
        }
    }
    const sagin::AisMaritimeTrace& trace = *best;
    std::printf("# ntn-v2x-maritime-ais: loaded %zu AIS rows, %zu vessels; "
                "replaying MMSI %u (%zu samples, the longest track)\n",
                importer.LastRowsRead(), traces.size(), trace.mmsi,
                trace.samples.size());

    // ENU origin = the vessel's first recorded position (real lat/lon).
    const double refLat = trace.samples.front().lat_deg;
    const double refLon = trace.samples.front().lon_deg;

    // Vessel terminal = UE driven by the real AIS replay mobility.
    NodeContainer ueNodes;
    ueNodes.Create(1);
    Ptr<sagin::AisMobilityModel> vessel = CreateObject<sagin::AisMobilityModel>();
    vessel->SetReference(refLat, refLon);
    vessel->SetTrace(trace);
    ueNodes.Get(0)->AggregateObject(vessel);
    g_vessel = vessel;

    // Real SGP4 LEO orbit projected into the scenario's local ENU frame.
    NodeContainer satNodes;
    satNodes.Create(1);
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

    // Real NR NTN cell -> MEASURED SINR/TBLER/throughput.
    NtnRealStackHelper rs;
    rs.SetRadioBackend(radio == "mmwave" ? NtnRealStackHelper::RadioBackend::Mmwave
                                         : NtnRealStackHelper::RadioBackend::Nr);
    if (radio != "mmwave")
    {
        rs.SetNumerology(1); // FR1 30 kHz SCS
    }
    rs.SetSimTime(Seconds(simSeconds));
    rs.SetOutputDir(outputDir);
    rs.SetRunTag("ntn-v2x-maritime-ais");
    rs.SetCarrierFrequencyHz(freqGHz * 1e9);
    rs.SetSatEirpDbm(satEirpDbm);
    rs.Build(satNodes, ueNodes);
    rs.InstallTraffic(NtnRealStackHelper::TrafficProfile::EmbbStreaming,
                      Seconds(1.0), Seconds(simSeconds - 0.5));
    rs.EnableAiFlowMonitor("ntn-v2x-maritime-ais"); // WS2 KPM series (TS 28.552 names)
    g_rs = &rs;

    std::printf("# ntn-v2x-maritime-ais (AIS-replayed vessel UE on a real %s NTN cell)\n"
                "#   sim=%.0fs leoAlt=%.0fkm freq=%.1fGHz EIRP=%.1fdBm refLatLon=(%.4f,%.4f)\n",
                radio.c_str(), simSeconds, leoAltKm, freqGHz, satEirpDbm, refLat, refLon);

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
