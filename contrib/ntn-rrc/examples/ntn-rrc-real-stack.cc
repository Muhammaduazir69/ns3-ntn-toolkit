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
 * Wave 2 M1 (P0-6, P0-7):
 *  - The UL sync reference point follows the payload (TS 38.213 4.2): at the
 *    gateway for this transparent payload, so ta-Common is the feeder leg and
 *    kmac is absent; `--sib19Ref=legacy` restores the beam-centre reference.
 *  - UE 0 decodes the SIB19 bytes that crossed the radio and hands them to its
 *    own NtnUlTimingPrecompensator (GNSS fix + broadcast ephemeris + ta-Common,
 *    TS 38.213 4.2). The sampler writes the UE's estimate, its error against
 *    the true UE-gNB round trip, the SIB19 age and the UL sync state (T430
 *    analogue: ul-SyncValidityDuration, TS 38.331 5.2.2.6).
 *  - The broadcast cellSpecificKoffset is consumed by the nr scheduler
 *    (N2Delay, TS 38.213 4.2 / 8.3 / 9.2.3), converted from 15 kHz reference
 *    slots to the cell's numerology.
 *
 * Usage:
 *   ./ns3 run "ntn-rrc-real-stack --duration=20 --numUes=4"
 */

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
#include "ns3/ntn-ul-timing-precompensator.h"

#include "ns3/ipv4.h"
#include "ns3/socket.h"
#include "ns3/udp-socket-factory.h"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>

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
uint64_t g_measReportBytes = 0;
uint16_t g_servingPci = 1;
uint32_t g_sib19Refresh = 0;
bool g_belowThresh = false;

// Real downlink delivery of the serialized SIB19 wire image across the radio.
Ptr<Socket> g_sib19TxSock;     // on the remote host (EPC PGW side)
InetSocketAddress g_sib19Dst = InetSocketAddress(Ipv4Address::GetAny(), 0);
uint32_t g_sib19Delivered = 0; // count of SIB19 packets parsed on the UE side
// P0-7: UE-side consumer of the decoded SIB19.
Ptr<NtnUlTimingPrecompensator> g_uePc;
double g_maxAbsTaErrNs = 0.0;
uint32_t g_taSamples = 0;
uint32_t g_taViol = 0;
uint32_t g_ulSyncedSamples = 0;
constexpr double kTeNtnNs = 716.15; // Te_NTN FR1 30/30 kHz, TS 38.133 Table 7.1C.2-1

void
Sib19UeRecv(Ptr<Socket> sock)
{
    Ptr<Packet> pkt;
    Address from;
    while ((pkt = sock->RecvFrom(from)))
    {
        const uint32_t n = pkt->GetSize();
        std::vector<uint8_t> buf(n);
        pkt->CopyData(buf.data(), n);
        Sib19Content parsed;
        if (Sib19Codec::Parse(buf.data(), n, parsed))
        {
            // A parsed-but-mismatched cellId (corrupt / reordered / foreign SIB)
            // must not abort the run — count only well-matched deliveries.
            if (parsed.cellId == g_sib->GetLatest().cellId)
            {
                ++g_sib19Delivered;
                // P0-7: the UE keeps what it decoded (TS 38.331 5.2.2.4) and
                // computes its own TA from it (TS 38.213 4.2).
                if (g_uePc)
                {
                    g_uePc->ReceiveSib19(parsed);
                }
            }
        }
    }
}

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

    // Transmit the freshly built 124-byte SIB19 wire image over the REAL radio
    // path (UDP -> GTP/EPC -> mmwave DL) so it actually crosses the link and is
    // parsed on the UE side (Sib19Codec::Parse) rather than only round-tripping
    // in an in-memory unit test.
    if (g_sib19TxSock)
    {
        const std::vector<uint8_t>& wire = g_sib->GetLatestSerialised();
        if (!wire.empty())
        {
            Ptr<Packet> pkt = Create<Packet>(wire.data(), wire.size());
            g_sib19TxSock->SendTo(pkt, 0, g_sib19Dst);
        }
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

    // P0-7: UE 0's own TA from the decoded SIB19, against the true UE-gNB
    // round trip (service link + feeder for a transparent payload).
    double taEstUs = std::numeric_limits<double>::quiet_NaN();
    double taErrNs = std::numeric_limits<double>::quiet_NaN();
    double sibAgeMs = std::numeric_limits<double>::quiet_NaN();
    int ulSynced = -1;
    if (g_uePc && g_uePc->HasSib19())
    {
        const double est = g_uePc->ComputeTa().GetSeconds();
        taEstUs = est * 1e6;
        taErrNs = (est - total.GetSeconds()) * 1e9;
        sibAgeMs = g_uePc->GetInformationAge().GetSeconds() * 1e3;
        ulSynced = g_uePc->IsUplinkSynchronized() ? 1 : 0;
        g_maxAbsTaErrNs = std::max(g_maxAbsTaErrNs, std::fabs(taErrNs));
        g_taViol += (std::fabs(taErrNs) > kTeNtnNs) ? 1 : 0;
        g_ulSyncedSamples += ulSynced;
        ++g_taSamples;
    }
    // No measurement yet (no TB decoded, no SIB19 decoded): empty CSV fields,
    // never a placeholder number.
    auto opt = [](double v, int prec) {
        if (std::isnan(v))
        {
            return std::string();
        }
        std::ostringstream o;
        o << std::fixed << std::setprecision(prec) << v;
        return o.str();
    };
    if (g_csv.is_open())
    {
        g_csv << std::fixed << std::setprecision(3) << t << "," << slantKm << ","
              << total.GetMicroSeconds() << "," << common.GetMicroSeconds() << ","
              << residual.GetMicroSeconds() << "," << std::scientific << std::setprecision(3)
              << (drift * 1e6) << "," << std::fixed << std::setprecision(2)
              << opt(sinr, 2) << "," << opt(taEstUs, 3) << "," << opt(taErrNs, 3) << ","
              << opt(sibAgeMs, 3) << "," << ulSynced << "," << std::setprecision(3)
              << g_sib->GetLatest().taCommon.GetNanoSeconds() / 1e3 << ","
              << g_sib->GetLatest().kMac << "\n";
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
    double satEirpDbm = -1.0; // sentinel: backend-appropriate default chosen below
    std::string radio = "nr"; // radio spine: "nr" (5G-LENA FR1) | "mmwave" (FR2)
    double freqGhz = 2.185; // TS 38.101-5 n256 downlink block (2170-2200 MHz). Was 2.0,
                              // which is n256 UPLINK and never conformant as a DL carrier.
    bool transparent = true;
    std::string outputDir = "ntn-rrc-real-stack-output";
    bool ueTa = true;
    std::string sib19Ref = "auto";
    bool kOffset = true;
    double gnssSigma = 0.0;

    CommandLine cmd(__FILE__);
    cmd.AddValue("duration", "Simulation duration (s)", duration);
    cmd.AddValue("numUes", "Number of UEs on the serving cell", numUes);
    cmd.AddValue("altitude", "Satellite altitude (km)", altitudeKm);
    cmd.AddValue("satEirpDbm", "-1 = TR 38.821 Set-1 S-band EIRP density (34 dBW/MHz); otherwise conducted power at the array input (dBm)", satEirpDbm);
    cmd.AddValue("radio", "Radio backend: nr (FR1) or mmwave", radio);
    cmd.AddValue("freqGhz", "Carrier frequency (GHz)", freqGhz);
    cmd.AddValue("transparent", "Transparent (true) vs regenerative (false) payload", transparent);
    cmd.AddValue("outputDir", "Output directory", outputDir);
    cmd.AddValue("ueTa", "UE 0 computes its TA from the decoded SIB19 (TS 38.213 4.2)", ueTa);
    cmd.AddValue("sib19Ref",
                 "UL sync reference: auto (gateway if transparent, satellite if regenerative) | "
                 "legacy (beam centre, double counts the service link) | gateway | satellite",
                 sib19Ref);
    cmd.AddValue("kOffset", "nr scheduler consumes the broadcast K_offset (N2Delay)", kOffset);
    cmd.AddValue("gnssSigma", "UE GNSS horizontal 1-sigma error (m)", gnssSigma);
    std::string netSimOut;
    std::string czmlOut;
    cmd.AddValue("netSim", "NetSimulyzer 3D JSON output (empty=off)", netSimOut);
    cmd.AddValue("czml", "Cesium CZML 3D output (empty=off)", czmlOut);
    cmd.Parse(argc, argv);
    g_simTime = duration;
    NS_ABORT_MSG_IF(sib19Ref != "auto" && sib19Ref != "legacy" && sib19Ref != "gateway" &&
                        sib19Ref != "satellite",
                    "--sib19Ref=auto|legacy|gateway|satellite");

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

    std::cout << "\n=== ntn-rrc REAL-STACK (SIB19 + TA on a real " << radioName << " NTN cell) ===\n"
              << "  serving cell: real " << radioName << " link, " << numUes << " UEs\n"
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
    satSgp4->SetElementsSgp4(wElements[0]);
    if (ueTa)
    {
        // The 100 ms position cache is up to 0.76 km stale at LEO speed, several
        // microseconds of delay: a timing study must not use it (as in
        // ntn-rrc-ul-timing). 1 ms keeps the stale range near 8 m (50 ns).
        satSgp4->SetCacheGranularity(MilliSeconds(1));
    }
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
    rs.SetSimTime(Seconds(duration));
    rs.SetOutputDir(outputDir);
    rs.SetRunTag("ntn-rrc-real-stack");
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
    NtnRrcHelper rrc;
    rrc.SetPayloadMode(transparent ? PayloadMode::Transparent : PayloadMode::RegenerativeFull);
    // TA reference = beam centre at the sub-point (ECEF), per TS 38.331.
    rrc.SetReferencePosition(ntngeo::GeodeticToEcef(subLat, subLon, 0.0));
    // RRC-2b: give the transparent payload its feeder leg.
    //
    // NtnTimingAdvance only adds the satellite-to-gateway hop when a gateway
    // mobility model is supplied; without one it returns a zero feeder range for
    // EVERY payload mode. SetGatewayMobility had no caller outside the unit
    // tests, so --transparent=1 and --transparent=0 produced an identical
    // 3789 us and the documented difference between the two payloads was inert
    // in every shipped run.
    //
    // The gateway sits 300 km from the sub-point, a realistic ground-station
    // offset for a LEO feeder link, so the transparent advance now covers both
    // legs as TS 38.213 requires.
    {
        Ptr<ConstantPositionMobilityModel> gwMob = CreateObject<ConstantPositionMobilityModel>();
        gwMob->SetPosition(ntngeo::GeodeticToEcef(subLat + 2.7, subLon, 0.0));
        rrc.SetGatewayMobility(gwMob);
    }
    g_ta = rrc.InstallTimingAdvance(ueMob, satMob);

    if (sib19Ref != "auto")
    {
        rrc.SetUlSyncReference(sib19Ref == "legacy"    ? UlSyncReference::BeamCenterLegacy
                               : sib19Ref == "gateway" ? UlSyncReference::Gateway
                                                       : UlSyncReference::Satellite);
    }
    // Cell id assigned after Build(); the K_offset the scheduler consumes must
    // be the broadcast one, so the broadcaster exists before the radio.
    g_sib = rrc.InstallSib19Broadcaster(satMob, 0, g_ta, MilliSeconds(160));
    g_sib->RefreshNow();
    const uint32_t koffRef = g_sib->GetLatest().cellSpecificKoffset; // 15 kHz reference slots
    if (kOffset && radio != "mmwave")
    {
        // RRC-1 / P0-7: the scheduler uses the value the network broadcast.
        // Before Build() the helper converts reference slots to the cell's
        // numerology itself.
        rs.SetKOffsetConsumption(true);
        rs.SetBroadcastKOffsetSlots(koffRef);
    }
    rs.Build(satNodes, ueNodes);
    const uint16_t cellId = rs.GetServingCellId(); // radio-agnostic (mmwave or nr gNB)
    g_sib->SetCellId(cellId);
    if (kOffset && radio != "mmwave")
    {
        // The sink passes the broadcast 15 kHz reference slots unchanged; the
        // helper converts them to cell-numerology slots (x 2^mu) after Build()
        // exactly as it does before it.
        g_sib->SetKOffsetSink(Callback<void, uint32_t>([&rs](uint32_t k) {
            rs.SetBroadcastKOffsetSlots(k);
        }));
    }
    g_sib->Start();
    if (ueTa)
    {
        // P0-7: UE 0's precompensator, fed only by the SIB19 it decodes.
        g_uePc = CreateObject<NtnUlTimingPrecompensator>();
        g_uePc->SetUeMobility(ueMob);
        g_uePc->SetGnss(Seconds(1), gnssSigma);
        g_uePc->AssignStreams(77);
    }
    // satEirpDbm was resolved before Build() and handed to the rest of the
    // example; make sure the radio landed on the same conducted power.
    NS_ABORT_MSG_IF(eirpFromTr38821 && std::abs(rs.GetSatEirpDbm() - satEirpDbm) > 1e-6,
                    "Set-1 conducted power mismatch: radio "
                        << rs.GetSatEirpDbm() << " dBm vs " << satEirpDbm << " dBm");
    rs.InstallTraffic(NtnRealStackHelper::TrafficProfile::EmbbStreaming,
                      Seconds(1.0), Seconds(duration - 0.5));
    rs.EnableAiFlowMonitor("ntn-rrc-real-stack"); // WS2 KPM series (TS 28.552 names)
    g_rs = &rs;


    // ---- real downlink delivery of the serialized SIB19 over the EPC+mmwave DL ----
    // UE-0 IP (interface 1 = the mmwave UE NetDevice assigned by the EPC).
    const uint16_t sib19Port = 9876; // outside the helper's DL (1234+) / UL (2000+) ranges
    Ipv4Address ueAddr = rs.GetUeDevices().Get(0)->GetNode()->GetObject<Ipv4>()
                             ->GetAddress(1, 0).GetLocal();
    g_sib19Dst = InetSocketAddress(ueAddr, sib19Port);
    g_sib19TxSock = Socket::CreateSocket(rs.GetRemoteHost(), UdpSocketFactory::GetTypeId());
    g_sib19TxSock->Bind();
    Ptr<Socket> sib19RxSock =
        Socket::CreateSocket(rs.GetUeDevices().Get(0)->GetNode(), UdpSocketFactory::GetTypeId());
    sib19RxSock->Bind(InetSocketAddress(Ipv4Address::GetAny(), sib19Port));
    sib19RxSock->SetRecvCallback(MakeCallback(&Sib19UeRecv));

    std::filesystem::create_directories(outputDir);
    g_csv.open(outputDir + "/ntn-rrc-real-stack-ta.csv");
    g_csv << "time_s,slant_km,ta_total_us,ta_common_us,ta_ue_us,ta_drift_us_per_s,"
             "measured_sinr_db,ta_ue_est_us,ta_err_ns,sib19_age_ms,ul_synced,"
             "sib19_ta_common_us,sib19_kmac\n";

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
              << "  SIB19 wire pkts delivered+parsed: " << g_sib19Delivered
              << "  (124-byte image crossed EPC+mmwave DL, Sib19Codec::Parse OK)\n"
              << "  RRC measurement reports:      " << g_measReports
              << "  (triggered on MEASURED SINR < " << g_measThreshDb << " dB)\n"
              << "  TA/SINR trace:                " << outputDir << "/ntn-rrc-real-stack-ta.csv\n";
    std::cout << "\n--- UE-side SIB19 consumption and K_offset (P0-6, P0-7) ---\n"
              << "  UL sync reference:            "
              << (rrc.GetEffectiveUlSyncReference() == UlSyncReference::Gateway     ? "gateway"
                  : rrc.GetEffectiveUlSyncReference() == UlSyncReference::Satellite ? "satellite"
                                                                                     : "legacy beam centre")
              << " (ta-Common " << g_sib->GetLatest().taCommon.GetMicroSeconds()
              << " us, kMac " << g_sib->GetLatest().kMac << ")\n";
    if (g_uePc)
    {
        std::cout << "  UE TA from decoded SIB19:     " << g_taSamples << " samples, max |err| "
                  << g_maxAbsTaErrNs << " ns (Te_NTN " << kTeNtnNs << " ns, violations "
                  << g_taViol << "), UL synced " << g_ulSyncedSamples << "/" << g_taSamples
                  << "\n";
    }
    if (kOffset && radio != "mmwave")
    {
        std::cout << "  K_offset broadcast (15 kHz):  " << koffRef << " slots at t=0; consumed "
                  << rs.GetConsumedKOffsetSlots() << " nr slots (mu=1), gNB N2Delay "
                  << rs.GetGnbN2Delay() << "\n";
    }

    Simulator::Destroy();
    return 0;
}
