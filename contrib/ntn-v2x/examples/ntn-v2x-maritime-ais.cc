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
 * AisMobilityModel). The default file contrib/ntn-sagin/data/ais-sample-trace.csv
 * is a small hand-made fixture in the Danish Maritime Authority CSV format (11
 * rows, round coordinates), NOT a recorded track; pass --aisTrace with a real
 * aisdk export (https://web.ais.dk/aisdata/) for recorded vessel motion.
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
    double freqGHz = 2.185; // TS 38.101-5 n256 DOWNLINK block (2170-2200 MHz).
    double satEirpDbm = -1.0; // sentinel: backend-appropriate default chosen below
    std::string radio = "nr"; // radio backend: "nr" (5G-LENA FR1) | "mmwave" (FR2)
    std::string aisTrace = "contrib/ntn-sagin/data/ais-sample-trace.csv";
    std::string outputDir = "ntn-v2x-maritime-ais-output";

    CommandLine cmd(__FILE__);
    cmd.AddValue("simSeconds", "Simulation duration (s)", simSeconds);
    cmd.AddValue("leoAltKm", "LEO altitude (km)", leoAltKm);
    cmd.AddValue("freqGHz", "Carrier frequency (GHz)", freqGHz);
    cmd.AddValue("satEirpDbm",
                 "Satellite total EIRP (dBm, antenna gain included); -1 = TR 38.821 Set-1 density",
                 satEirpDbm);
    cmd.AddValue("radio", "Radio backend: nr (FR1) or mmwave", radio);
    cmd.AddValue("aisTrace",
                 "Danish Maritime Authority AIS CSV export. Defaults to the shipped "
                 "ntn-sagin fixture (DMA format, hand-made rows, not a recorded track).",
                 aisTrace);
    cmd.AddValue("outputDir", "Output directory", outputDir);
    cmd.Parse(argc, argv);
    g_simTime = simSeconds;

    // --satEirpDbm used to be parsed, defaulted to 70 dBm, printed as the EIRP
    // and then ignored: the cell always ran the TR 38.821 Set-1 density. Now
    // the TR 38.821 density is the default and an explicit value is applied as
    // the TOTAL EIRP (antenna gain included), which is what the flag says.
    const bool eirpExplicit = (satEirpDbm >= 0.0);

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
    // NT-02: TR 38.821 Table 6.1.1.1-1 Set-1 downlink EIRP density for the
    // S-band LEO reference payload. Declared as a DENSITY so the helper
    // back-computes conducted power against the array gain instead of the
    // antenna being counted twice.
    if (eirpExplicit)
    {
        rs.SetSatEirpTotalDbm(satEirpDbm);
    }
    else
    {
        rs.SetSatEirpDensityDbwMhz(NtnRealStackHelper::kTr38821Set1SBandEirpDensityDbwMhz);
    }
    rs.Build(satNodes, ueNodes);
    // The EIRP actually radiated, for the banner below.
    const double eirpPrintedDbm =
        eirpExplicit ? satEirpDbm
                     : NtnRealStackHelper::kTr38821Set1SBandEirpDensityDbwMhz +
                           10.0 * std::log10(rs.GetBandwidthHz() / 1e6) + 30.0;
    rs.InstallTraffic(NtnRealStackHelper::TrafficProfile::EmbbStreaming,
                      Seconds(1.0), Seconds(simSeconds - 0.5));
    rs.EnableAiFlowMonitor("ntn-v2x-maritime-ais"); // WS2 KPM series (TS 28.552 names)
    g_rs = &rs;

    std::printf("# ntn-v2x-maritime-ais (AIS-replayed vessel UE on a real %s NTN cell)\n"
                "#   sim=%.0fs leoAlt=%.0fkm freq=%.3fGHz EIRP=%.1fdBm (%s) refLatLon=(%.4f,%.4f)\n",
                radio.c_str(), simSeconds, leoAltKm, freqGHz, eirpPrintedDbm,
                eirpExplicit ? "--satEirpDbm" : "TR 38.821 Set-1 34 dBW/MHz", refLat, refLon);

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
