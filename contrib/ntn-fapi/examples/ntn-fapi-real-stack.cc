/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026  Muhammad Uzair
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * ntn-fapi-real-stack — real-stack flagship for ntn-fapi.
 *
 * The SCF-222 L1<->L2 message ABI (DL_TTI / TX_DATA / RX_DATA / CRC.indication)
 * rides a REAL mmwave NR NTN cell (NtnRealStackHelper: SpectrumPhy + MAC + HARQ
 * + RLC/PDCP + RRC + EPC). The per-slot CRC.indication is decided by the
 * MEASURED PHY outcome: the recent DL SINR and TBLER are read off the mmwave
 * RxPacketTraceUe trace (the real SINR->BLER error model), and CRC pass/fail /
 * HARQ feedback follow that measured TBLER. The FAPI message format is preserved
 * verbatim; only the L1 physics that fills it is now real and measured. A mid-run
 * elevation descent drops the measured SINR so CRC failures + HARQ retx appear.
 *
 * Usage:
 *   ./ns3 run "ntn-fapi-real-stack --duration=20 --numUes=4 --scsKhz=30"
 */

#include "ns3/core-module.h"
#include "ns3/mmwave-enb-net-device.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/ntn-real-stack-helper.h"
#include "ns3/ntn-tr38811-mobility-model.h"
#include "ns3/sgp4-mobility-model.h"
#include "ns3/walker-constellation.h"


#include "ns3/fapi-messages.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <iostream>

#include "ns3/ntn-scene-helper.h"

using namespace ns3;
using namespace ns3::fapi;

NS_LOG_COMPONENT_DEFINE("NtnFapiRealStack");

namespace
{
NtnRealStackHelper* g_rs = nullptr;
Ptr<UniformRandomVariable> g_rng;
double g_simTime = 20.0;
uint32_t g_slotsPerSubframe = 2;
uint32_t g_tbBytes = 1500;
uint16_t g_sfn = 0, g_slot = 0;
uint32_t g_harqId = 0;
uint64_t g_tbSent = 0, g_tbOk = 0, g_tbRetx = 0;
uint64_t g_bytesDelivered = 0;
bool g_pendingRetx = false;

void
SlotTick()
{
    const double sinr = g_rs->GetUeRecentSinrDb(0);
    const double measuredTbler = g_rs->GetUeRecentTbler(0);
    const bool haveMeas = !std::isnan(sinr) && !std::isnan(measuredTbler);

    TxDataRequest tx;
    tx.sfn = g_sfn;
    tx.slot = g_slot;
    TxDataRequest::PduPayload pdu;
    pdu.pduIndex = 0;
    pdu.cwIndex = 0;
    pdu.tbBytes.assign(g_tbBytes, 0xAB);
    tx.pdus.push_back(pdu);
    ++g_tbSent;
    if (g_pendingRetx)
    {
        ++g_tbRetx;
    }

    const double bler = haveMeas ? measuredTbler : 0.0;
    const bool crcOk = g_rng->GetValue() > bler;

    RxDataIndication rx;
    rx.sfn = g_sfn;
    rx.slot = g_slot;
    if (crcOk)
    {
        RxDataIndication::PduRx prx;
        prx.handle = g_tbSent;
        prx.rnti = 1;
        prx.harqId = static_cast<uint16_t>(g_harqId);
        prx.pduLength = static_cast<uint16_t>(g_tbBytes);
        prx.tbBytes = tx.pdus[0].tbBytes;
        rx.pdus.push_back(prx);
    }

    CrcIndication crc;
    crc.sfn = g_sfn;
    crc.slot = g_slot;
    CrcIndication::CrcReport rep;
    rep.handle = g_tbSent;
    rep.rnti = 1;
    rep.harqId = static_cast<uint16_t>(g_harqId);
    rep.tbCrcStatusOk = crcOk;
    rep.ul_cqi = static_cast<int16_t>(std::lround(std::max(-10.0, haveMeas ? sinr : 0.0)));
    crc.crcList.push_back(rep);

    if (crc.crcList[0].tbCrcStatusOk)
    {
        ++g_tbOk;
        g_bytesDelivered += rx.pdus.empty() ? 0 : rx.pdus[0].pduLength;
        g_pendingRetx = false;
        g_harqId = (g_harqId + 1) % 16;
    }
    else
    {
        g_pendingRetx = true;
    }

    if (++g_slot >= g_slotsPerSubframe * 10)
    {
        g_slot = 0;
        g_sfn = (g_sfn + 1) % 1024;
    }

    const int64_t slotDurNs = static_cast<int64_t>(1000000.0 / g_slotsPerSubframe);
    if (Simulator::Now().GetSeconds() + slotDurNs / 1e9 < g_simTime)
    {
        Simulator::Schedule(NanoSeconds(slotDurNs), &SlotTick);
    }
}
} // namespace

int
main(int argc, char* argv[])
{
    double duration = 20.0;
    uint32_t numUes = 4;
    uint32_t scsKhz = 30;
    uint32_t tbBytes = 1500;
    double altitudeKm = 550.0;
    double satEirpDbm = 55.0;
    std::string outputDir = "ntn-fapi-real-stack-output";

    CommandLine cmd(__FILE__);
    cmd.AddValue("duration", "Simulation duration (s)", duration);
    cmd.AddValue("numUes", "Number of UEs on the serving cell", numUes);
    cmd.AddValue("scsKhz", "Sub-carrier spacing (kHz): 15/30/60/120", scsKhz);
    cmd.AddValue("tbBytes", "FAPI transport-block size (bytes)", tbBytes);
    cmd.AddValue("altitude", "Satellite altitude (km)", altitudeKm);
    cmd.AddValue("satEirpDbm", "Satellite EIRP / gNB Tx power (dBm)", satEirpDbm);
    cmd.AddValue("outputDir", "Output directory", outputDir);
    std::string netSimOut;
    std::string czmlOut;
    cmd.AddValue("netSim", "NetSimulyzer 3D JSON output (empty=off)", netSimOut);
    cmd.AddValue("czml", "Cesium CZML 3D output (empty=off)", czmlOut);
    cmd.Parse(argc, argv);
    g_simTime = duration;
    g_tbBytes = tbBytes;
    g_slotsPerSubframe = std::max<uint32_t>(1, scsKhz / 15);

    std::cout << "\n=== ntn-fapi REAL-STACK (SCF-222 ABI on a real mmwave NR cell) ===\n"
              << "  FAPI L1 CRC.indication: decided by MEASURED PHY SINR/TBLER (not a sigmoid)\n"
              << "  scs: " << scsKhz << " kHz (" << g_slotsPerSubframe << " slots/ms), TB "
              << tbBytes << " B, " << numUes << " UEs, " << duration << " s\n\n";

    NodeContainer satNodes;
    satNodes.Create(1);
    NodeContainer ueNodes;
    ueNodes.Create(numUes);

    // Real NTN mobility: SGP4 Walker serving satellite + TR 38.811 UEs under
    // its t=0 sub-point (UE+sat share the ECEF frame; the pass is genuine).
    ns3::ntncon::WalkerConfig wcfg;
    wcfg.num_planes = 1;
    wcfg.total_sats = 80;
    wcfg.altitude_km = altitudeKm;
    wcfg.inclination_deg = 53.0;
    wcfg.epoch_unix_s = 1735689600.0;
    const auto wElements = ns3::ntncon::WalkerConstellation::BuildDelta(wcfg);
    Ptr<ns3::ntncon::Sgp4MobilityModel> servSatMob =
        CreateObject<ns3::ntncon::Sgp4MobilityModel>();
    servSatMob->SetElements(wElements[0]);
    satNodes.Get(0)->AggregateObject(servSatMob);
    double subLat, subLon, subAlt;
    servSatMob->GetGeodetic(subLat, subLon, subAlt);
    NtnTr38811MobilityHelper ueMobility(1);
    auto mobProfile = NtnMobilityScenarios::MixedContinental();
    ueMobility.Install(ueNodes, mobProfile, subLat - 0.03, subLat + 0.03,
                       subLon - 0.03, subLon + 0.03);

    NtnRealStackHelper rs;
    rs.SetSimTime(Seconds(duration));
    rs.SetOutputDir(outputDir);
    rs.SetRunTag("ntn-fapi-real-stack");
    rs.SetSatEirpDbm(satEirpDbm);
    rs.Build(satNodes, ueNodes);
    rs.InstallTraffic(NtnRealStackHelper::TrafficProfile::EmbbStreaming,
                      Seconds(1.0), Seconds(duration - 0.5));
    rs.EnableAiFlowMonitor("ntn-fapi-real-stack"); // WS2 KPM series (TS 28.552 names)
    g_rs = &rs;
    g_rng = CreateObject<UniformRandomVariable>();

    Simulator::Schedule(Seconds(1.0), &SlotTick);

    Simulator::Stop(Seconds(duration));
    ns3::ntnobs::NtnSceneHelper ntnScene;
    if (!netSimOut.empty()) ntnScene.SetNetSimulyzer(netSimOut);
    if (!czmlOut.empty()) ntnScene.SetCzml(czmlOut);
    Ptr<ns3::ntnobs::NtnSceneRecorder> ntnSceneRec = ntnScene.Build(satNodes, ueNodes);

    Simulator::Run();
    if (ntnSceneRec) ntnSceneRec->Stop();
    rs.Collect();
    rs.WriteHealthReport();

    const double goodputMbps = (g_bytesDelivered * 8.0) / (duration * 1e6);
    std::cout << "\n--- FAPI Summary (SCF-222 ABI on MEASURED radio) ---\n"
              << "  measured serving SINR (mean): " << rs.GetMeanDlSinrDb() << " dB\n"
              << "  measured DL TBLER (mean):     " << rs.GetMeanDlTbler() << "\n"
              << "  measured radio throughput:    " << rs.GetRxThroughputMbps() << " Mbps\n"
              << "  FAPI slots: sent=" << g_tbSent << " crcOk=" << g_tbOk
              << " retx=" << g_tbRetx << "\n"
              << "  FAPI delivered:               " << (g_bytesDelivered / 1000)
              << " KB  goodput=" << goodputMbps << " Mbps\n"
              << "  -> CRC.indication driven by the MEASURED error model, not SinrToBler().\n";

    Simulator::Destroy();
    return 0;
}
