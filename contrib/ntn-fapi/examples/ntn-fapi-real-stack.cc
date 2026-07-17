/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026  Muhammad Uzair
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * ntn-fapi-real-stack — real FAPI L1/L2 boundary over a live mmwave NR NTN cell.
 *
 * Closes audit gap F1/F3 + CI gate 15. The SCF-222.10 L1<->L2 message ABI is no
 * longer assembled into local variables that fall out of scope: a
 * NtnFapiSapBridge DECORATES the real mmwave MAC<->PHY SAP
 * (contrib/mmwave/model/mmwave-phy-sap.h) on a NtnRealStackHelper cell:
 *
 *   * the real PHY's MmWaveEnbPhySapUser::SlotIndication(SfnSf) fires the FAPI
 *     SLOT.indication (0x82) with the SfnSf-derived frame/slot;
 *   * the real MAC's MmWavePhySapProvider::SetSlotAllocInfo(SlotAllocInfo) is
 *     translated into a FAPI DL_TTI.request (0x80) built from the actual DL DCIs
 *     (rnti/mcs/tbSize and the REAL mmwave HARQ process id);
 *   * the real per-TB decode (mmwave RxPacketTraceUe, m_corrupt) drives the FAPI
 *     RX_DATA.indication (0x85) / CRC.indication (0x86).
 *
 * Every FAPI message thus has a REAL producer and a REAL consumer tied to actual
 * mmwave slot timing. The example prints the request->indication latency measured
 * across the SAP (DL_TTI.request -> CRC.indication, aligned by SFN/slot) and the
 * scheduling-pipeline latency (DL_TTI.request -> SLOT.indication) — that latency
 * IS CI gate 15. A P5 CONFIG.request is consumed at bring-up from the real PHY
 * config, so "FAPI configures the PHY" is genuinely true.
 *
 * Usage:
 *   ./ns3 run "ntn-fapi-real-stack --duration=20 --numUes=4 --scsKhz=30"
 */

#include "ns3/config.h"
#include "ns3/core-module.h"
#include "ns3/mmwave-enb-net-device.h"
#include "ns3/mmwave-phy-mac-common.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/ntn-real-stack-helper.h"
#include "ns3/ntn-tr38811-mobility-model.h"
#include "ns3/sgp4-mobility-model.h"
#include "ns3/walker-constellation.h"

#include "ns3/fapi-helpers.h"
#include "ns3/fapi-messages.h"
#include "ns3/fapi-pdu-types.h"
#include "ns3/ntn-fapi-sap-bridge.h"

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
Ptr<NtnFapiSapBridge> g_bridge = nullptr;
uint16_t g_ueRnti = 0;

// Reverse FAPI path: feed the real per-TB decode into the bridge, which emits
// RX_DATA.indication / CRC.indication and closes the SAP latency measurement.
// Same RxPacketTraceUe trace the helper uses, lazily filtered to UE-0's RNTI.
void
FapiPhyRxTrace(mmwave::RxPacketTraceParams p)
{
    if (g_ueRnti == 0 && g_rs)
    {
        g_ueRnti = g_rs->GetUeRnti(0); // RNTI assigned once RRC connects at runtime
        if (g_ueRnti != 0 && g_bridge)
        {
            g_bridge->SetUeRnti(g_ueRnti);
        }
    }
    if (g_bridge)
    {
        g_bridge->OnPhyRx(p);
    }
}
} // namespace

int
main(int argc, char* argv[])
{
    double duration = 20.0;
    uint32_t numUes = 4;
    uint32_t scsKhz = 30;
    double altitudeKm = 550.0;
    double satEirpDbm = -1.0; // sentinel: backend-appropriate default chosen below
    // The FAPI SAP bridge decorates the mmwave MAC<->PHY SAP, so the DL_TTI /
    // SLOT / CRC.indication timing is only measured on the mmwave backend.
    // --radio=nr still selects the nr air interface (measured SINR/TBLER stay
    // valid) but the SAP bridge is not installed (nr has a different SAP).
    std::string radio = "mmwave";
    std::string outputDir = "ntn-fapi-real-stack-output";

    CommandLine cmd(__FILE__);
    cmd.AddValue("duration", "Simulation duration (s)", duration);
    cmd.AddValue("numUes", "Number of UEs on the serving cell", numUes);
    cmd.AddValue("scsKhz", "Sub-carrier spacing (kHz): 15/30/60/120", scsKhz);
    cmd.AddValue("altitude", "Satellite altitude (km)", altitudeKm);
    cmd.AddValue("satEirpDbm", "Satellite EIRP / gNB Tx power (dBm); -1 = backend default", satEirpDbm);
    cmd.AddValue("radio", "Radio backend: mmwave (FR2, FAPI SAP bridge) or nr (FR1)", radio);
    cmd.AddValue("outputDir", "Output directory", outputDir);
    std::string netSimOut;
    std::string czmlOut;
    cmd.AddValue("netSim", "NetSimulyzer 3D JSON output (empty=off)", netSimOut);
    cmd.AddValue("czml", "Cesium CZML 3D output (empty=off)", czmlOut);
    cmd.Parse(argc, argv);

    // Backend-appropriate EIRP default: nr's Friis LEO link needs ~+15 dB vs
    // mmwave, so honour the historical 55 dBm for mmwave but give nr 70 dBm.
    const bool useNr = (radio == "nr");
    if (satEirpDbm < 0.0)
    {
        satEirpDbm = useNr ? 70.0 : 55.0;
    }

    std::cout << "\n=== ntn-fapi REAL-STACK (real SCF-222 L1/L2 SAP on a live mmwave NR cell) ===\n"
              << "  FAPI SLOT.indication  <- real MmWaveEnbPhySapUser::SlotIndication(SfnSf)\n"
              << "  FAPI DL_TTI.request   <- real MmWavePhySapProvider::SetSlotAllocInfo(SlotAllocInfo)\n"
              << "  FAPI CRC.indication   <- real per-TB decode (mmwave m_corrupt)\n"
              << "  scs: " << scsKhz << " kHz, " << numUes << " UEs, " << duration << " s\n\n";

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
    rs.SetRadioBackend(radio == "mmwave" ? NtnRealStackHelper::RadioBackend::Mmwave
                                         : NtnRealStackHelper::RadioBackend::Nr);
    if (radio != "mmwave")
    {
        rs.SetNumerology(1); // FR1 30 kHz SCS (nr backend only)
    }
    rs.SetSimTime(Seconds(duration));
    rs.SetOutputDir(outputDir);
    rs.SetRunTag("ntn-fapi-real-stack");
    rs.SetSatEirpDbm(satEirpDbm);
    rs.Build(satNodes, ueNodes);
    rs.InstallTraffic(NtnRealStackHelper::TrafficProfile::EmbbStreaming,
                      Seconds(1.0), Seconds(duration - 0.5));
    rs.EnableAiFlowMonitor("ntn-fapi-real-stack"); // WS2 KPM series (TS 28.552 names)
    g_rs = &rs;

    // ---- Install the real FAPI SAP bridge on the live mmwave enb cell -------
    if (radio == "mmwave")
    {
        Ptr<mmwave::MmWaveEnbNetDevice> enb =
            DynamicCast<mmwave::MmWaveEnbNetDevice>(rs.GetEnbDevices().Get(0));
        NS_ABORT_MSG_IF(!enb, "expected an MmWaveEnbNetDevice on the mmwave backend");
        g_bridge = CreateObject<NtnFapiSapBridge>();
        g_bridge->InstallEnb(enb); // interposes decorators + consumes P5 CONFIG.request
        const auto& cc = g_bridge->GetCarrierConfig();
        std::cout << "  P5 CONFIG.request consumed: cellId=" << g_bridge->GetCellConfig().phyCellId
                  << " dlFreq=" << cc.dlFrequency / 1e3 << " MHz dlBW=" << cc.dlBandwidth
                  << " MHz numRb=" << cc.dlGridSize[0] << "\n\n";
    }

    // Reverse FAPI path: same real per-TB decode trace the helper connects.
    Config::ConnectWithoutContextFailSafe(
        "/NodeList/*/DeviceList/*/ComponentCarrierMap/*/MmWaveUePhy/DlSpectrumPhy/RxPacketTraceUe",
        MakeCallback(&FapiPhyRxTrace));

    Simulator::Stop(Seconds(duration));
    ns3::ntnobs::NtnSceneHelper ntnScene;
    if (!netSimOut.empty()) ntnScene.SetNetSimulyzer(netSimOut);
    if (!czmlOut.empty()) ntnScene.SetCzml(czmlOut);
    Ptr<ns3::ntnobs::NtnSceneRecorder> ntnSceneRec = ntnScene.Build(satNodes, ueNodes);

    Simulator::Run();
    if (ntnSceneRec) ntnSceneRec->Stop();
    rs.Collect();
    rs.WriteHealthReport();

    const uint64_t realRxBytes = rs.GetUeRxBytes(0);
    const double goodputMbps = (realRxBytes * 8.0) / (duration * 1e6);

    std::cout << "\n--- FAPI Summary (real SCF-222 SAP on MEASURED radio) ---\n"
              << "  measured serving SINR (mean): " << rs.GetMeanDlSinrDb() << " dB\n"
              << "  measured DL TBLER (mean):     " << rs.GetMeanDlTbler() << "\n"
              << "  measured radio throughput:    " << rs.GetRxThroughputMbps() << " Mbps\n";

    if (g_bridge)
    {
        const double meanLatUs = g_bridge->GetMeanSapLatencySec() * 1e6;
        const double minLatUs = g_bridge->GetMinSapLatencySec() * 1e6;
        const double maxLatUs = g_bridge->GetMaxSapLatencySec() * 1e6;
        const double pipeUs = g_bridge->GetMeanSchedPipelineSec() * 1e6;
        std::cout << "\n--- FAPI SAP (CI gate 15: real request->indication latency) ---\n"
                  << "  SLOT.indication fired:        " << g_bridge->GetSlotIndicationCount()
                  << " (real PHY slot cadence)\n"
                  << "  DL_TTI.request emitted:       " << g_bridge->GetDlTtiRequestCount()
                  << " (with DL data: " << g_bridge->GetDlTtiWithDataCount() << ")\n"
                  << "  RX_DATA.indication emitted:   " << g_bridge->GetRxDataIndicationCount() << "\n"
                  << "  CRC.indication emitted:       " << g_bridge->GetCrcIndicationCount() << "\n"
                  << "  matched (SFN/slot-aligned):   " << g_bridge->GetMatchedLatencyCount() << "\n"
                  << "  sched-pipeline latency (mean): DL_TTI.request -> SLOT.indication = "
                  << pipeUs << " us\n"
                  << "  SAP latency  DL_TTI.request -> CRC.indication (SFN/slot aligned):\n"
                  << "    mean=" << meanLatUs << " us  min=" << minLatUs
                  << " us  max=" << maxLatUs << " us  last=" << g_bridge->GetLastSapLatencySec() * 1e6
                  << " us\n"
                  << "  -> every FAPI message had a REAL mmwave producer + consumer at real slot timing.\n";
    }
    std::cout << "  FAPI delivered:               " << (realRxBytes / 1000)
              << " KB  goodput=" << goodputMbps << " Mbps\n";

    Simulator::Destroy();
    return 0;
}
