/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only
//
// ntn-rrc-leo-pass — ephemeris-driven Timing Advance + SIB19 (TS 38.331
// NTN-Config) on a REAL mmwave NR NTN cell (NtnRealStackHelper: SpectrumPhy +
// MAC + HARQ + RLC/PDCP + RRC + EPC). The TA total/common/UE-specific values and
// the TA drift rate are taken from the live LEO-pass geometry (the classic NTN
// "smile" curve), SIB19 is re-broadcast from the live ephemeris, and the RRC
// connection-quality measurement report fires on the MEASURED DL SINR off the
// mmwave RxPacketTraceUe trace — no closed-form SINR anywhere. Writes a TA/SINR
// trace CSV and an honest sim_health.csv (phy-trace provenance).

#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/ntn-real-stack-helper.h"
#include "ns3/ntn-tr38811-mobility-model.h"
#include "ns3/sgp4-mobility-model.h"
#include "ns3/walker-constellation.h"

#include "ns3/ntn-rrc-helper.h"
#include "ns3/ntn-sib19.h"
#include "ns3/ntn-meas-report.h"
#include "ns3/ntn-timing-advance.h"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>

using namespace ns3;
using namespace ns3::ntnrrc;

NS_LOG_COMPONENT_DEFINE("NtnRrcLeoPass");

namespace
{
NtnRealStackHelper* g_rs = nullptr;
Ptr<NtnTimingAdvance> g_ta;
Ptr<NtnSib19Broadcaster> g_sib;
std::ofstream g_csv;
double g_simTime = 20.0;
double g_measThreshDb = 14.0;
uint32_t g_measReports = 0;
uint64_t g_measReportBytes = 0;
uint16_t g_servingPci = 1;
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

    // RRC-5: a real TS 38.331 MeasurementReport, not a printf.
    //
    // What used to be here incremented a counter and logged a line when the
    // measured SINR crossed a threshold. It had no measId, no quantization and
    // no recipient, yet it was labelled "RRC measurement report" and counted in
    // the summary - which in a results table reads as TS 38.331 signalling.
    //
    // The report below is built, quantized to the TS 38.133 reporting levels,
    // serialised through NtnMeasReportCodec and parsed back, so what the
    // example counts is a message that survived a wire round trip. The
    // threshold crossing stays as the reporting trigger; the event machinery
    // proper (A3 offset, hysteresis, time-to-trigger) lives in NtnChoAlgorithm
    // and is not duplicated here.
    const double sinr = g_rs->GetUeRecentSinrDb(0);
    if (!std::isnan(sinr))
    {
        const bool below = sinr < g_measThreshDb;
        if (below && !g_belowThresh)
        {
            NtnMeasurementReport rep;
            rep.measId = 1; // TS 38.331 MeasId, 1..64
            rep.servingCell.physCellId = g_servingPci;
            rep.servingCell.sinrLevel = NtnMeasQuantity::SinrToLevel(sinr);
            rep.servingCell.haveSinr = true;
            const double rsrp = g_rs->GetServingRsrpDbm();
            if (!std::isnan(rsrp))
            {
                rep.servingCell.rsrpLevel = NtnMeasQuantity::RsrpToLevel(rsrp);
                rep.servingCell.haveRsrp = true;
            }
            rep.servingTimingAdvance = total;
            rep.servingSlantRangeM = slantKm * 1000.0;

            uint8_t buf[NtnMeasurementReport::kMaxNeighbours * 8 + 32];
            const std::size_t n = NtnMeasReportCodec::Serialise(rep, buf, sizeof(buf));
            NtnMeasurementReport decoded;
            const bool ok = n > 0 && NtnMeasReportCodec::Parse(buf, n, decoded);
            if (ok)
            {
                ++g_measReports;
                g_measReportBytes += n;
                // Print RSRP only when the report actually carries it. TS 38.331
                // makes each quantity optional, and printing a level for an
                // absent one would show -156 dBm - the bottom of the reporting
                // range - as though it had been measured.
                char rsrpTxt[48];
                if (decoded.servingCell.haveRsrp)
                {
                    std::snprintf(rsrpTxt, sizeof(rsrpTxt), "RSRP_LEV=%u (%.0f dBm)",
                                  decoded.servingCell.rsrpLevel,
                                  NtnMeasQuantity::LevelToRsrpDbm(decoded.servingCell.rsrpLevel));
                }
                else
                {
                    std::snprintf(rsrpTxt, sizeof(rsrpTxt), "RSRP absent");
                }
                std::printf("  %6.1fs  TS 38.331 MeasurementReport measId=%u pci=%u "
                            "SINR_LEV=%u (%.1f dB) %s %zu B (slant %.0f km, TA=%lld us)\n",
                            t, decoded.measId, decoded.servingCell.physCellId,
                            decoded.servingCell.sinrLevel,
                            NtnMeasQuantity::LevelToSinrDb(decoded.servingCell.sinrLevel),
                            rsrpTxt, n, slantKm,
                            static_cast<long long>(total.GetMicroSeconds()));
            }
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
    double simTime = 20.0;
    uint32_t numUes = 4;
    double altitudeKm = 550.0;
    double satEirpDbm = -1.0; // sentinel: backend-appropriate default chosen below
    std::string radio = "nr"; // radio spine: "nr" (5G-LENA FR1) | "mmwave" (FR2)
    double freqGhz = 2.0;
    bool transparent = true;
    std::string outputDir = "ntn-rrc-leo-pass-output";

    CommandLine cmd(__FILE__);
    cmd.AddValue("simTime", "Simulation duration (s)", simTime);
    cmd.AddValue("numUes", "Number of UEs on the serving cell", numUes);
    cmd.AddValue("altitude", "Satellite altitude (km)", altitudeKm);
    cmd.AddValue("satEirpDbm", "Satellite EIRP / gNB Tx power (dBm); -1 = backend default", satEirpDbm);
    cmd.AddValue("radio", "Radio backend: nr (FR1) or mmwave", radio);
    cmd.AddValue("freqGhz", "Carrier frequency (GHz)", freqGhz);
    cmd.AddValue("transparent", "Transparent (true) vs regenerative (false) payload", transparent);
    cmd.AddValue("outputDir", "Output directory", outputDir);
    cmd.Parse(argc, argv);
    g_simTime = simTime;

    // Backend-appropriate EIRP default: nr's Friis LEO link needs ~+15 dB vs
    // mmwave, so honour the historical 55 dBm for mmwave but give nr 70 dBm.
    const bool useNr = (radio == "nr");
    if (satEirpDbm < 0.0)
    {
        satEirpDbm = useNr ? 70.0 : 55.0;
    }
    const char* radioName = useNr ? "5G-LENA nr FR1" : "mmwave FR2";

    std::cout << "\n=== ntn-rrc-leo-pass (SIB19 + TA on a real " << radioName << " NTN cell) ===\n"
              << "  serving cell: real " << radioName << " link, " << numUes << " UEs\n"
              << "  TA + SIB19 ephemeris: live LEO-pass geometry (TS 38.331 NTN-Config)\n"
              << "  RRC measurement trigger: MEASURED DL SINR (not a formula)\n"
              << "  duration: " << simTime << " s\n\n";

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
    rs.SetRadioBackend(radio == "mmwave" ? NtnRealStackHelper::RadioBackend::Mmwave
                                         : NtnRealStackHelper::RadioBackend::Nr);
    if (radio != "mmwave")
    {
        rs.SetNumerology(1); // FR1 30 kHz SCS (nr backend only)
    }
    rs.SetSimTime(Seconds(simTime));
    rs.SetOutputDir(outputDir);
    rs.SetRunTag("ntn-rrc-leo-pass");
    rs.SetCarrierFrequencyHz(freqGhz * 1e9);
    // NT-02: declared as CONDUCTED power at the array input. This carrier has
    // no TR 38.821 Set-1 reference in the toolkit, so the EIRP health gate
    // reports "not asserted" rather than certifying an uncalibrated budget.
    rs.SetSatConductedPowerDbm(satEirpDbm);
    rs.Build(satNodes, ueNodes);
    rs.InstallTraffic(NtnRealStackHelper::TrafficProfile::EmbbStreaming,
                      Seconds(1.0), Seconds(simTime - 0.5));
    rs.EnableAiFlowMonitor("ntn-rrc-leo-pass"); // WS2 KPM series (TS 28.552 names)
    g_rs = &rs;

    NtnRrcHelper rrc;
    rrc.SetPayloadMode(transparent ? PayloadMode::Transparent : PayloadMode::RegenerativeFull);
    rrc.SetReferencePosition(ntngeo::GeodeticToEcef(subLat, subLon, 0.0));
    g_ta = rrc.InstallTimingAdvance(ueMob, satMob);

    const uint16_t cellId = rs.GetServingCellId(); // radio-agnostic (mmwave or nr gNB)
    g_sib = rrc.InstallSib19Broadcaster(satMob, cellId, g_ta, MilliSeconds(160));
    g_sib->Start();

    std::filesystem::create_directories(outputDir);
    g_csv.open(outputDir + "/ntn-rrc-leo-pass-ta.csv");
    g_csv << "time_s,slant_km,ta_total_us,ta_common_us,ta_ue_us,ta_drift_us_per_s,"
             "measured_sinr_db\n";

    // The serving satellite flies its REAL SGP4 pass: the slant range, TA and
    // measured SINR all evolve with genuine orbital dynamics (no teleports).
    Simulator::Schedule(Seconds(1.0), &Sample);

    Simulator::Stop(Seconds(simTime));
    Simulator::Run();
    rs.Collect();
    rs.WriteHealthReport();
    if (g_csv.is_open())
    {
        g_csv.close();
    }

    std::cout << "\n--- RRC Summary (real SIB19/TA on MEASURED radio) ---\n"
              << "  measured serving SINR (mean): " << rs.GetMeanDlSinrDb() << " dB\n"
              << "  measured DL throughput:       " << rs.GetRxThroughputMbps() << " Mbps\n"
              << "  final slant range:            " << g_ta->GetSlantRangeMetres() / 1000.0
              << " km\n"
              << "  final total TA:               " << g_ta->ComputeTotalTa().GetMicroSeconds()
              << " us\n"
              << "  SIB19 ephemeris refreshes:    " << g_sib19Refresh << "\n"
              << "  RRC measurement reports:      " << g_measReports
              << "  (on MEASURED SINR < " << g_measThreshDb << " dB)\n"
              << "  TA/SINR trace:                " << outputDir << "/ntn-rrc-leo-pass-ta.csv\n";

    Simulator::Destroy();
    return 0;
}
