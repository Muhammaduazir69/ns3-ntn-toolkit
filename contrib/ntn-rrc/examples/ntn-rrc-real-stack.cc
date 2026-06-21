/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026  Muhammad Uzair
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * ntn-rrc-real-stack — real-stack flagship for ntn-rrc.
 *
 * The real RRC NTN machinery (NtnTimingAdvance + NtnSib19Broadcaster, TS 38.331
 * NTN-Config) sits on a REAL mmwave NR NTN cell (NtnRealStackHelper: SpectrumPhy
 * + MAC + HARQ + RLC/PDCP + RRC + EPC). The serving satellite flies a LEO pass,
 * so the TA total/common/UE-specific and the TA drift come from the real
 * slant-range geometry (the classic NTN "smile" curve), and the RRC
 * connection-quality measurement report is the MEASURED DL SINR off the mmwave
 * PHY trace (GetUeRecentSinrDb) — not a formula. SIB19 is re-broadcast every
 * si-period from the live ephemeris while packets actually traverse the radio.
 *
 * Usage:
 *   ./ns3 run "ntn-rrc-real-stack --duration=20 --numUes=4"
 */

#include "ns3/core-module.h"
#include "ns3/mmwave-enb-net-device.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/ntn-real-stack-helper.h"
#include "ns3/ntn-tr38811-mobility-model.h"
#include "ns3/sgp4-mobility-model.h"
#include "ns3/walker-constellation.h"

#include "ns3/ntn-rrc-helper.h"
#include "ns3/ntn-sib19.h"
#include "ns3/ntn-timing-advance.h"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>

#include "ns3/ntn-scene-helper.h"

using namespace ns3;
using namespace ns3::ntnrrc;

NS_LOG_COMPONENT_DEFINE("NtnRrcRealStack");

namespace
{
NtnRealStackHelper* g_rs = nullptr;
Ptr<NtnTimingAdvance> g_ta;
Ptr<NtnSib19Broadcaster> g_sib;
std::ofstream g_csv;
double g_simTime = 20.0;
double g_measThreshDb = 14.0;
uint32_t g_measReports = 0;
uint32_t g_sib19Refresh = 0;
bool g_belowThresh = false;

void
Sample()
{
    const double t = Simulator::Now().GetSeconds();
    if (t >= g_simTime)
    {
        return;
    }
    const Time total = g_ta->ComputeTotalTa();
    const Time common = g_ta->ComputeCommonTa();
    const Time residual = g_ta->ComputeUeSpecificTa();
    const double drift = g_ta->ComputeTaDriftRate(MilliSeconds(10));
    const double slantKm = g_ta->GetSlantRangeMetres() / 1000.0;

    g_sib->RefreshNow();
    ++g_sib19Refresh;

    const double sinr = g_rs->GetUeRecentSinrDb(0);
    if (!std::isnan(sinr))
    {
        const bool below = sinr < g_measThreshDb;
        if (below && !g_belowThresh)
        {
            ++g_measReports;
            std::printf("  %6.1fs  RRC measurement report: serving SINR meas=%.1f dB "
                        "< %.1f dB (slant %.0f km, TA=%lld us)\n",
                        t, sinr, g_measThreshDb, slantKm,
                        static_cast<long long>(total.GetMicroSeconds()));
        }
        g_belowThresh = below;
    }

    if (g_csv.is_open())
    {
        g_csv << std::fixed << std::setprecision(3) << t << "," << slantKm << ","
              << total.GetMicroSeconds() << "," << common.GetMicroSeconds() << ","
              << residual.GetMicroSeconds() << "," << std::scientific << std::setprecision(3)
              << (drift * 1e6) << "," << std::fixed << std::setprecision(2)
              << (std::isnan(sinr) ? 0.0 : sinr) << "\n";
    }
    Simulator::Schedule(MilliSeconds(500), &Sample);
}
} // namespace

int
main(int argc, char* argv[])
{
    double duration = 20.0;
    uint32_t numUes = 4;
    double altitudeKm = 550.0;
    double satEirpDbm = 55.0;
    double freqGhz = 2.0;
    bool transparent = true;
    std::string outputDir = "ntn-rrc-real-stack-output";

    CommandLine cmd(__FILE__);
    cmd.AddValue("duration", "Simulation duration (s)", duration);
    cmd.AddValue("numUes", "Number of UEs on the serving cell", numUes);
    cmd.AddValue("altitude", "Satellite altitude (km)", altitudeKm);
    cmd.AddValue("satEirpDbm", "Satellite EIRP / gNB Tx power (dBm)", satEirpDbm);
    cmd.AddValue("freqGhz", "Carrier frequency (GHz)", freqGhz);
    cmd.AddValue("transparent", "Transparent (true) vs regenerative (false) payload", transparent);
    cmd.AddValue("outputDir", "Output directory", outputDir);
    std::string netSimOut;
    std::string czmlOut;
    cmd.AddValue("netSim", "NetSimulyzer 3D JSON output (empty=off)", netSimOut);
    cmd.AddValue("czml", "Cesium CZML 3D output (empty=off)", czmlOut);
    cmd.Parse(argc, argv);
    g_simTime = duration;

    std::cout << "\n=== ntn-rrc REAL-STACK (SIB19 + TA on a real mmwave NR NTN cell) ===\n"
              << "  serving cell: real mmwave NR link, " << numUes << " UEs\n"
              << "  TA + SIB19 ephemeris: from live LEO-pass geometry (TS 38.331 NTN-Config)\n"
              << "  RRC measurement trigger: MEASURED DL SINR (not a formula)\n"
              << "  duration: " << duration << " s\n\n";

    NodeContainer satNodes;
    satNodes.Create(1);
    NodeContainer ueNodes;
    ueNodes.Create(numUes);

    // Real SGP4 Walker orbit for the serving satellite (genuine LEO pass).
    ns3::ntncon::WalkerConfig wcfg;
    wcfg.num_planes = 1;
    wcfg.total_sats = 80;
    wcfg.altitude_km = altitudeKm;
    wcfg.inclination_deg = 53.0;
    wcfg.epoch_unix_s = 1735689600.0;
    const auto wElements = ns3::ntncon::WalkerConstellation::BuildDelta(wcfg);
    Ptr<ns3::ntncon::Sgp4MobilityModel> satSgp4 =
        CreateObject<ns3::ntncon::Sgp4MobilityModel>();
    satSgp4->SetElements(wElements[0]);
    satNodes.Get(0)->AggregateObject(satSgp4);
    Ptr<MobilityModel> satMob = satSgp4;
    double subLat, subLon, subAlt;
    satSgp4->GetGeodetic(subLat, subLon, subAlt);

    // TR 38.811 class UEs (real MobilityModel) under the t=0 sub-point.
    NtnTr38811MobilityHelper ueMobility(1);
    auto mobProfile = NtnMobilityScenarios::MixedContinental();
    auto ueModels = ueMobility.Install(ueNodes, mobProfile, subLat - 0.03, subLat + 0.03,
                                       subLon - 0.03, subLon + 0.03);
    Ptr<MobilityModel> ueMob = ueModels[0];

    NtnRealStackHelper rs;
    rs.SetSimTime(Seconds(duration));
    rs.SetOutputDir(outputDir);
    rs.SetRunTag("ntn-rrc-real-stack");
    rs.SetCarrierFrequencyHz(freqGhz * 1e9);
    rs.SetSatEirpDbm(satEirpDbm);
    rs.Build(satNodes, ueNodes);
    rs.InstallTraffic(NtnRealStackHelper::TrafficProfile::EmbbStreaming,
                      Seconds(1.0), Seconds(duration - 0.5));
    rs.EnableAiFlowMonitor("ntn-rrc-real-stack"); // WS2 KPM series (TS 28.552 names)
    g_rs = &rs;

    NtnRrcHelper rrc;
    rrc.SetPayloadMode(transparent ? PayloadMode::Transparent : PayloadMode::RegenerativeFull);
    // TA reference = beam centre at the sub-point (ECEF), per TS 38.331.
    rrc.SetReferencePosition(ntngeo::GeodeticToEcef(subLat, subLon, 0.0));
    g_ta = rrc.InstallTimingAdvance(ueMob, satMob);

    Ptr<mmwave::MmWaveEnbNetDevice> enb =
        DynamicCast<mmwave::MmWaveEnbNetDevice>(rs.GetEnbDevices().Get(0));
    uint16_t cellId = enb ? enb->GetCellId() : 1;
    g_sib = rrc.InstallSib19Broadcaster(satMob, cellId, g_ta, MilliSeconds(160));
    g_sib->Start();

    std::filesystem::create_directories(outputDir);
    g_csv.open(outputDir + "/ntn-rrc-real-stack-ta.csv");
    g_csv << "time_s,slant_km,ta_total_us,ta_common_us,ta_ue_us,ta_drift_us_per_s,"
             "measured_sinr_db\n";

    Simulator::Schedule(Seconds(1.0), &Sample);

    Simulator::Stop(Seconds(duration));
    ns3::ntnobs::NtnSceneHelper ntnScene;
    if (!netSimOut.empty()) ntnScene.SetNetSimulyzer(netSimOut);
    if (!czmlOut.empty()) ntnScene.SetCzml(czmlOut);
    Ptr<ns3::ntnobs::NtnSceneRecorder> ntnSceneRec = ntnScene.Build(satNodes, ueNodes);

    Simulator::Run();
    if (ntnSceneRec) ntnSceneRec->Stop();
    rs.Collect();
    rs.WriteHealthReport();
    if (g_csv.is_open())
    {
        g_csv.close();
    }

    const Time finalTa = g_ta->ComputeTotalTa();
    std::cout << "\n--- RRC Summary (real SIB19/TA on MEASURED radio) ---\n"
              << "  measured serving SINR (mean): " << rs.GetMeanDlSinrDb() << " dB\n"
              << "  measured DL throughput:       " << rs.GetRxThroughputMbps() << " Mbps\n"
              << "  final slant range:            " << g_ta->GetSlantRangeMetres() / 1000.0
              << " km\n"
              << "  final total TA:               " << finalTa.GetMicroSeconds() << " us\n"
              << "  SIB19 ephemeris refreshes:    " << g_sib19Refresh << "\n"
              << "  RRC measurement reports:      " << g_measReports
              << "  (triggered on MEASURED SINR < " << g_measThreshDb << " dB)\n"
              << "  TA/SINR trace:                " << outputDir << "/ntn-rrc-real-stack-ta.csv\n";

    Simulator::Destroy();
    return 0;
}
