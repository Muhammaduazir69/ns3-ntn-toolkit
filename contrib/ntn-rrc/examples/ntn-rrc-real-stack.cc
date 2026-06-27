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
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/ntn-real-stack-helper.h"
#include "ns3/ntn-tr38811-mobility-model.h"
#include "ns3/sgp4-mobility-model.h"
#include "ns3/walker-constellation.h"

#include "ns3/ntn-rrc-helper.h"
#include "ns3/ntn-sib19.h"
#include "ns3/ntn-timing-advance.h"

#include "ns3/ipv4.h"
#include "ns3/socket.h"
#include "ns3/udp-socket-factory.h"

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

// Real downlink delivery of the serialized SIB19 wire image across the radio.
Ptr<Socket> g_sib19TxSock;     // on the remote host (EPC PGW side)
InetSocketAddress g_sib19Dst = InetSocketAddress(Ipv4Address::GetAny(), 0);
uint32_t g_sib19Delivered = 0; // count of SIB19 packets parsed on the UE side

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
            NS_ASSERT_MSG(parsed.cellId == g_sib->GetLatest().cellId,
                          "SIB19 cellId mismatch across the radio link");
            ++g_sib19Delivered;
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
    double satEirpDbm = -1.0; // sentinel: backend-appropriate default chosen below
    std::string radio = "nr"; // radio spine: "nr" (5G-LENA FR1) | "mmwave" (FR2)
    double freqGhz = 2.0;
    bool transparent = true;
    std::string outputDir = "ntn-rrc-real-stack-output";

    CommandLine cmd(__FILE__);
    cmd.AddValue("duration", "Simulation duration (s)", duration);
    cmd.AddValue("numUes", "Number of UEs on the serving cell", numUes);
    cmd.AddValue("altitude", "Satellite altitude (km)", altitudeKm);
    cmd.AddValue("satEirpDbm", "Satellite EIRP / gNB Tx power (dBm); -1 = backend default", satEirpDbm);
    cmd.AddValue("radio", "Radio backend: nr (FR1) or mmwave", radio);
    cmd.AddValue("freqGhz", "Carrier frequency (GHz)", freqGhz);
    cmd.AddValue("transparent", "Transparent (true) vs regenerative (false) payload", transparent);
    cmd.AddValue("outputDir", "Output directory", outputDir);
    std::string netSimOut;
    std::string czmlOut;
    cmd.AddValue("netSim", "NetSimulyzer 3D JSON output (empty=off)", netSimOut);
    cmd.AddValue("czml", "Cesium CZML 3D output (empty=off)", czmlOut);
    cmd.Parse(argc, argv);
    g_simTime = duration;

    // Backend-appropriate EIRP default: nr's Friis LEO link needs ~+15 dB vs
    // mmwave, so honour the historical 55 dBm for mmwave but give nr 70 dBm.
    const bool useNr = (radio == "nr");
    if (satEirpDbm < 0.0)
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

    const uint16_t cellId = rs.GetServingCellId(); // radio-agnostic (mmwave or nr gNB)
    g_sib = rrc.InstallSib19Broadcaster(satMob, cellId, g_ta, MilliSeconds(160));
    g_sib->Start();

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
              << "  SIB19 wire pkts delivered+parsed: " << g_sib19Delivered
              << "  (124-byte image crossed EPC+mmwave DL, Sib19Codec::Parse OK)\n"
              << "  RRC measurement reports:      " << g_measReports
              << "  (triggered on MEASURED SINR < " << g_measThreshDb << " dB)\n"
              << "  TA/SINR trace:                " << outputDir << "/ntn-rrc-real-stack-ta.csv\n";

    Simulator::Destroy();
    return 0;
}
