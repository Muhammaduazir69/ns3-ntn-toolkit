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
#include "ns3/ntn-ue-sib19-consumer.h"

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
Ptr<NtnUeSib19Consumer> g_ueSib; // P0-7: UE-side SIB19 consumer (in-process wire image)
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

    // P0-7: the UE's TA from the SIB19 it holds (decoded 500 ms ago), then it
    // decodes the fresh wire image. In-process delivery until SIB19 goes on
    // the PHY (P1-B); the UE still only sees the parsed bytes.
    const NtnUeSib19Consumer::SampleResult ueTa =
        g_ueSib ? g_ueSib->Sample() : NtnUeSib19Consumer::SampleResult{};
    g_sib->RefreshNow();
    ++g_sib19Refresh;
    if (g_ueSib)
    {
        g_ueSib->ReceiveWire(g_sib->GetLatestSerialised());
    }

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
              << (std::isnan(sinr) ? 0.0 : sinr) << "," << std::setprecision(3)
              << (ueTa.valid ? ueTa.estUs : -1.0) << "," << (ueTa.valid ? ueTa.errNs : 0.0) << ","
              << (ueTa.valid ? ueTa.ageMs : -1.0) << "," << (ueTa.valid ? int(ueTa.synced) : -1)
              << "\n";
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
    double freqGhz = 2.185; // TS 38.101-5 n256 downlink block (2170-2200 MHz). Was 2.0,
                              // which is n256 UPLINK and never conformant as a DL carrier.
    bool transparent = true;
    std::string outputDir = "ntn-rrc-leo-pass-output";

    CommandLine cmd(__FILE__);
    cmd.AddValue("simTime", "Simulation duration (s)", simTime);
    cmd.AddValue("numUes", "Number of UEs on the serving cell", numUes);
    cmd.AddValue("altitude", "Satellite altitude (km)", altitudeKm);
    cmd.AddValue("satEirpDbm", "-1 = TR 38.821 Set-1 S-band EIRP density (34 dBW/MHz); otherwise conducted power at the array input (dBm)", satEirpDbm);
    cmd.AddValue("radio", "Radio backend: nr (FR1) or mmwave", radio);
    cmd.AddValue("freqGhz", "Carrier frequency (GHz)", freqGhz);
    cmd.AddValue("transparent", "Transparent (true) vs regenerative (false) payload", transparent);
    cmd.AddValue("outputDir", "Output directory", outputDir);
    cmd.Parse(argc, argv);
    g_simTime = simTime;

    const bool useNr = (radio == "nr");
    // EIRP default (NT-02 calibration). The nr default used to be 70 dBm of
    // CONDUCTED power, about 88 dBm EIRP once the ~18 dB array gain is added,
    // ~11 dB above the TR 38.821 Table 6.1.1.1-1 Set-1 S-band payload
    // (34 dBW/MHz, 77 dBm over 20 MHz). On an S-band FR1 carrier the nr
    // default is now that Set-1 density, as ntn-v2x-real-stack does; an
    // explicit --satEirpDbm keeps meaning conducted power, and mmwave keeps
    // its historical 55.0 dBm.
    // The probe mirrors the cell's carrier and bandwidth and returns the
    // equivalent CONDUCTED power from the helper's own formula
    // (density + 10log10(BW_MHz) + 30 - ArrayGainDb()), so every other
    // consumer of satEirpDbm stays consistent with the radio.
    NtnRealStackHelper eirpProbe;
    eirpProbe.SetCarrierFrequencyHz(freqGhz * 1e9);
    const bool eirpFromTr38821 = satEirpDbm < 0.0 && useNr &&
                                 eirpProbe.GetCarrierFrequencyHz() > 1.0e9 &&
                                 eirpProbe.GetCarrierFrequencyHz() <= 7.125e9;
    if (eirpFromTr38821)
    {
        eirpProbe.SetSatEirpDensityDbwMhz(NtnRealStackHelper::kTr38821Set1SBandEirpDensityDbwMhz);
        satEirpDbm = eirpProbe.GetSatEirpDbm();
    }
    else if (satEirpDbm < 0.0)
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
    satSgp4->SetElementsSgp4(wElements[0]);
    // UE TA study: a 100 ms SGP4 cache is up to 0.76 km stale (microseconds).
    satSgp4->SetCacheGranularity(MilliSeconds(1));
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
    if (eirpFromTr38821)
    {
        // NT-02: TR 38.821 Table 6.1.1.1-1 Set-1 S-band EIRP density. The
        // helper back-computes the conducted power from the bandwidth and
        // the array gain, and again in Build().
        rs.SetSatEirpDensityDbwMhz(NtnRealStackHelper::kTr38821Set1SBandEirpDensityDbwMhz);
    }
    else
    {
        // NT-02: an explicit value (or a default on a carrier outside the
        // Set-1 S-band reference) is CONDUCTED power at the array input.
        rs.SetSatConductedPowerDbm(satEirpDbm);
    }
    if (radio != "mmwave")
    {
        // P0-7: the nr scheduler consumes K_offset (N2Delay, TS 38.213 4.2);
        // the broadcast value replaces the local derivation once SIB19 runs.
        rs.SetKOffsetConsumption(true);
    }
    rs.Build(satNodes, ueNodes);
    // satEirpDbm was resolved before Build() and handed to the rest of the
    // example; make sure the radio landed on the same conducted power.
    NS_ABORT_MSG_IF(eirpFromTr38821 && std::abs(rs.GetSatEirpDbm() - satEirpDbm) > 1e-6,
                    "Set-1 conducted power mismatch: radio "
                        << rs.GetSatEirpDbm() << " dBm vs " << satEirpDbm << " dBm");
    rs.InstallTraffic(NtnRealStackHelper::TrafficProfile::EmbbStreaming,
                      Seconds(1.0), Seconds(simTime - 0.5));
    rs.EnableAiFlowMonitor("ntn-rrc-leo-pass"); // WS2 KPM series (TS 28.552 names)
    g_rs = &rs;

    NtnRrcHelper rrc;
    rrc.SetPayloadMode(transparent ? PayloadMode::Transparent : PayloadMode::RegenerativeFull);
    rrc.SetReferencePosition(ntngeo::GeodeticToEcef(subLat, subLon, 0.0));
    // RRC-2c: give the transparent payload its feeder leg, as ntn-rrc-real-stack
    // does. Without a gateway NtnTimingAdvance returns a zero feeder range, so
    // --transparent=1 and --transparent=0 printed the same TA and the payload
    // flag was inert. TS 38.213 section 4.2: the common TA the network signals
    // covers the feeder (gateway to satellite) round trip; TR 38.821 Table
    // 4.2-2 lists the transparent RTT as service plus feeder. The gateway is a
    // fixed ground station 300 km from the t=0 sub-point, in view for the pass.
    if (transparent)
    {
        Ptr<ConstantPositionMobilityModel> gwMob = CreateObject<ConstantPositionMobilityModel>();
        gwMob->SetPosition(ntngeo::GeodeticToEcef(subLat + 2.7, subLon, 0.0));
        rrc.SetGatewayMobility(gwMob);
    }
    g_ta = rrc.InstallTimingAdvance(ueMob, satMob);

    const uint16_t cellId = rs.GetServingCellId(); // radio-agnostic (mmwave or nr gNB)
    g_sib = rrc.InstallSib19Broadcaster(satMob, cellId, g_ta, MilliSeconds(160));
    if (radio != "mmwave")
    {
        // 15 kHz reference slots on the wire; the helper converts them to
        // slots of the cell numerology (x 2^mu) itself, before and after Build.
        g_sib->SetKOffsetSink(
            Callback<void, uint32_t>([&rs](uint32_t k) { rs.SetBroadcastKOffsetSlots(k); }));
    }
    g_sib->Start();
    g_ueSib = CreateObject<NtnUeSib19Consumer>();
    g_ueSib->Setup(ueMob, g_ta);

    std::filesystem::create_directories(outputDir);
    g_csv.open(outputDir + "/ntn-rrc-leo-pass-ta.csv");
    g_csv << "time_s,slant_km,ta_total_us,ta_common_us,ta_ue_us,ta_drift_us_per_s,"
             "measured_sinr_db,ta_ue_est_us,ta_err_ns,sib19_age_ms,ul_synced\n";

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
              << "  TA/SINR trace:                " << outputDir << "/ntn-rrc-leo-pass-ta.csv\n"
              << "  UE TA from decoded SIB19:     " << g_ueSib->GetSampleCount()
              << " samples, max |err| " << g_ueSib->GetMaxAbsErrorNs() << " ns (Te_NTN 716.15 ns, "
              << "violations " << g_ueSib->GetViolationCount() << "), UL synced "
              << g_ueSib->GetSyncedCount() << "; ta-Common " << g_ueSib->GetHeld().taCommon.GetMicroSeconds()
              << " us, kMac " << g_ueSib->GetHeld().kMac << "\n"
              << "  K_offset consumed:            " << rs.GetConsumedKOffsetSlots()
              << " nr slots (broadcast " << g_ueSib->GetHeld().cellSpecificKoffset
              << " x 15 kHz slots)\n";

    Simulator::Destroy();
    return 0;
}
