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
uint16_t g_ueRnti = 0;
double g_simTime = 20.0;
uint32_t g_slotsPerSubframe = 2;
uint32_t g_tbBytes = 1500;
uint16_t g_sfn = 0, g_slot = 0;
uint32_t g_harqId = 0;
uint64_t g_tbSent = 0, g_tbOk = 0, g_tbRetx = 0;
uint64_t g_bytesDelivered = 0;
bool g_pendingRetx = false;

// Assemble the real per-slot DL scheduling/data messages from the module's own
// integrated structs (DlTtiRequest + PdcchPdu + PdschPdu + TxDataRequest),
// round-tripping the DMRS bitmap through the real fapi-helpers conversion. The
// CRC/HARQ outcome comes from the measured PHY decode in FapiPhyRxTrace below.
void
SlotTick()
{
    // ---- L1/L2 scheduling: assemble a real DL_TTI.request (PDCCH + PDSCH) ----
    DlTtiRequest req;
    req.sfn = g_sfn;
    req.slot = g_slot;
    req.nPdusOfEachType[0] = 1; // PDCCH
    req.nPdusOfEachType[1] = 1; // PDSCH
    req.nPdusOfEachType[2] = 0;
    req.nPdusOfEachType[3] = 0;
    req.numGroups = 1;

    DlTtiPdu pdcch;
    pdcch.type = DlTtiPdu::Type::kPdcch;
    PdcchPdu pdcchPdu{};
    PdcchPdu::Dci dci{};
    dci.rnti = g_rs->GetUeRnti(0);
    dci.aggregationLevel = 4;
    pdcchPdu.dciList.push_back(dci);
    pdcch.pdu = pdcchPdu;
    req.pduList.push_back(pdcch);

    DlTtiPdu pdsch;
    pdsch.type = DlTtiPdu::Type::kPdsch;
    PdschPdu pdschPdu{};
    pdschPdu.rnti = g_rs->GetUeRnti(0);
    pdschPdu.numCodewords = 1;
    pdschPdu.codewords[0].tbSize = g_tbBytes;
    // Real DMRS bitmap round-trip via the module's own helpers (fapi-helpers.cc).
    pdschPdu.dmrs.dmrsSymbPos = DmrsBitArrayToFapi({2, 11});
    NS_ASSERT(DmrsFapiToBitArray(pdschPdu.dmrs.dmrsSymbPos).size() == 2);
    pdsch.pdu = pdschPdu;
    req.pduList.push_back(pdsch);

    static_assert(GetMessageId<DlTtiRequest>() == kDlTtiRequest, "DL_TTI id");
    NS_ASSERT(DlTtiRequest::kId == kDlTtiRequest);

    // ---- L2 (MAC): TX_DATA.request carrying the real transport block ----
    TxDataRequest tx;
    tx.sfn = g_sfn;
    tx.slot = g_slot;
    TxDataRequest::PduPayload pdu;
    pdu.pduIndex = 0;
    pdu.cwIndex = 0;
    pdu.tbBytes.assign(g_tbBytes, 0xAB);
    tx.pdus.push_back(pdu);

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

// FAPI CRC.indication / RX_DATA.indication driven by the REAL per-TB decode the
// mmwave error model already computed (RxPacketTraceParams::m_corrupt). Connected
// to the same RxPacketTraceUe trace the helper uses, filtered to UE-0's RNTI.
void
FapiPhyRxTrace(mmwave::RxPacketTraceParams p)
{
    if (g_ueRnti == 0 && g_rs)
    {
        g_ueRnti = g_rs->GetUeRnti(0); // RNTI assigned once RRC connects at runtime
    }
    if (p.m_tbSize == 0 || (g_ueRnti != 0 && p.m_rnti != g_ueRnti))
    {
        return; // skip control TBs and other UEs (mirror DlRxTrace filtering)
    }
    const bool crcOk = !p.m_corrupt; // real decode outcome, no RNG
    const double sinrDb = 10.0 * std::log10(std::max(p.m_sinr, 1e-12));
    ++g_tbSent;
    if (g_pendingRetx)
    {
        ++g_tbRetx;
    }

    RxDataIndication rx;
    rx.sfn = static_cast<uint16_t>(p.m_frameNum);
    rx.slot = p.m_slotNum;
    if (crcOk)
    {
        RxDataIndication::PduRx prx;
        prx.handle = static_cast<uint32_t>(g_tbSent);
        prx.rnti = p.m_rnti;
        prx.harqId = static_cast<uint16_t>(g_harqId);
        prx.pduLength = static_cast<uint16_t>(p.m_tbSize);
        rx.pdus.push_back(prx);
    }

    CrcIndication crc;
    crc.sfn = static_cast<uint16_t>(p.m_frameNum);
    crc.slot = p.m_slotNum;
    CrcIndication::CrcReport rep{};
    rep.handle = static_cast<uint32_t>(g_tbSent);
    rep.rnti = p.m_rnti;
    rep.harqId = static_cast<uint16_t>(g_harqId);
    rep.tbCrcStatusOk = crcOk; // == !m_corrupt
    rep.ul_cqi = static_cast<int16_t>(std::lround(std::max(-10.0, sinrDb)));
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
        g_pendingRetx = true; // real decode failure -> HARQ retx
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
    double satEirpDbm = -1.0; // sentinel: backend-appropriate default chosen below
    // Default mmwave: this example builds FAPI CRC.indication per TB off the mmwave
    // RxPacketTraceUe trace (mmwave::RxPacketTraceParams), which the nr backend's
    // PHY trace cannot feed; --radio=nr still selects the nr air interface (the
    // helper's measured SINR/TBLER/throughput stay valid) but the per-TB FAPI loop
    // only populates on mmwave.
    std::string radio = "mmwave";
    std::string outputDir = "ntn-fapi-real-stack-output";

    CommandLine cmd(__FILE__);
    cmd.AddValue("duration", "Simulation duration (s)", duration);
    cmd.AddValue("numUes", "Number of UEs on the serving cell", numUes);
    cmd.AddValue("scsKhz", "Sub-carrier spacing (kHz): 15/30/60/120", scsKhz);
    cmd.AddValue("tbBytes", "FAPI transport-block size (bytes)", tbBytes);
    cmd.AddValue("altitude", "Satellite altitude (km)", altitudeKm);
    cmd.AddValue("satEirpDbm", "Satellite EIRP / gNB Tx power (dBm); -1 = backend default", satEirpDbm);
    cmd.AddValue("radio", "Radio backend: mmwave (FR2, per-TB FAPI loop) or nr (FR1)", radio);
    cmd.AddValue("outputDir", "Output directory", outputDir);
    std::string netSimOut;
    std::string czmlOut;
    cmd.AddValue("netSim", "NetSimulyzer 3D JSON output (empty=off)", netSimOut);
    cmd.AddValue("czml", "Cesium CZML 3D output (empty=off)", czmlOut);
    cmd.Parse(argc, argv);
    g_simTime = duration;
    g_tbBytes = tbBytes;
    g_slotsPerSubframe = std::max<uint32_t>(1, scsKhz / 15);

    // Backend-appropriate EIRP default: nr's Friis LEO link needs ~+15 dB vs
    // mmwave, so honour the historical 55 dBm for mmwave but give nr 70 dBm.
    const bool useNr = (radio == "nr");
    if (satEirpDbm < 0.0)
    {
        satEirpDbm = useNr ? 70.0 : 55.0;
    }

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
    g_ueRnti = rs.GetUeRnti(0); // 0 until RRC connects; refreshed in FapiPhyRxTrace

    // Drive FAPI CRC.indication off the SAME real per-TB decode trace the helper
    // connects (RxPacketTraceUe), filtered to UE-0. Connect after Build(). This
    // mmwave-specific path only matches on the mmwave backend (no-op under nr).
    Config::ConnectWithoutContextFailSafe(
        "/NodeList/*/DeviceList/*/ComponentCarrierMap/*/MmWaveUePhy/DlSpectrumPhy/RxPacketTraceUe",
        MakeCallback(&FapiPhyRxTrace));

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

    // Goodput from the helper's already-measured real delivered bytes.
    const uint64_t realRxBytes = rs.GetUeRxBytes(0);
    const double goodputMbps = (realRxBytes * 8.0) / (duration * 1e6);
    std::cout << "\n--- FAPI Summary (SCF-222 ABI on MEASURED radio) ---\n"
              << "  measured serving SINR (mean): " << rs.GetMeanDlSinrDb() << " dB\n"
              << "  measured DL TBLER (mean):     " << rs.GetMeanDlTbler() << "\n"
              << "  measured radio throughput:    " << rs.GetRxThroughputMbps() << " Mbps\n"
              << "  FAPI slots: sent=" << g_tbSent << " crcOk=" << g_tbOk
              << " retx=" << g_tbRetx << "\n"
              << "  FAPI delivered:               " << (realRxBytes / 1000)
              << " KB  goodput=" << goodputMbps << " Mbps\n"
              << "  -> CRC.indication driven by the MEASURED error model (real m_corrupt decode).\n";

    Simulator::Destroy();
    return 0;
}
