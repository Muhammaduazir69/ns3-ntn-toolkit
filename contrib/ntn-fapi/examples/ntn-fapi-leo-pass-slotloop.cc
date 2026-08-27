/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only
//
// ntn-fapi-leo-pass-slotloop — the SCF-222 FAPI L1<->L2 data ABI driven by a
// REAL SGP4 LEO pass over a REAL mmwave NR NTN cell. A satellite is propagated
// by ntn-constellation's Sgp4MobilityModel; the ground station is auto-placed at
// its t=0 sub-point so a real rise->zenith->set pass occurs, and a real mmwave
// NR cell (NtnRealStackHelper) carries traffic over the pass. The per-slot FAPI
// CRC.indication is decided by the MEASURED PHY outcome (recent DL TBLER off the
// mmwave RxPacketTraceUe trace), so the delivered FAPI goodput tracks the
// genuine orbital geometry — no closed-form ElevationToSinrDb / SinrToBler.
//
// Quick test:  --simSeconds=20 --scsKhz=30 --tle=contrib/ntn-rrc/data/iss-zarya.tle

#include "ns3/command-line.h"
#include "ns3/config.h"
#include "ns3/core-module.h"
#include "ns3/mmwave-enb-net-device.h"
#include "ns3/mmwave-phy-mac-common.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/ntn-real-stack-helper.h"
#include "ns3/ntn-tr38811-mobility-model.h"

#include "ns3/fapi-helpers.h"
#include "ns3/ntn-fapi-sap-bridge.h"
#include "ns3/fapi-messages.h"
#include "ns3/fapi-pdu-types.h"
#include "ns3/sgp4-mobility-model.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>

using namespace ns3;
using namespace ns3::fapi;
using ns3::ntncon::Sgp4MobilityModel;
using ns3::ntncon::TleRecord;

NS_LOG_COMPONENT_DEFINE("NtnFapiLeoPassSlotLoop");

namespace
{
NtnRealStackHelper* g_rs = nullptr;
uint16_t g_ueRnti = 0;
double g_simSeconds = 20.0;
uint32_t g_slotsPerSubframe = 2;
uint32_t g_tbBytes = 1500;
uint16_t g_sfn = 0, g_slot = 0;
uint32_t g_harqId = 0;
uint64_t g_tbSent = 0, g_tbOk = 0, g_tbRetx = 0;
uint64_t g_bytesDelivered = 0;
Ptr<fapi::NtnFapiSapBridge> g_bridge;
bool g_pendingRetx = false;

Vector
GeodeticToEcef(double latDeg, double lonDeg, double altM)
{
    constexpr double kA = 6378137.0;
    constexpr double kF = 1.0 / 298.257223563;
    constexpr double kE2 = kF * (2.0 - kF);
    const double latR = latDeg * M_PI / 180.0;
    const double lonR = lonDeg * M_PI / 180.0;
    const double s = std::sin(latR), c = std::cos(latR);
    const double N = kA / std::sqrt(1.0 - kE2 * s * s);
    return Vector((N + altM) * c * std::cos(lonR),
                  (N + altM) * c * std::sin(lonR),
                  (N * (1.0 - kE2) + altM) * s);
}

// FAPI-3: the DL_TTI.request / TX_DATA.request assembly that used to live here
// was a free-running loop. It built the structs into locals that fell out of
// scope, stepped its own g_sfn/g_slot counters on its own timer, and was
// therefore uncorrelated with the SFN/slot the radio was actually running -
// while printing its output next to genuinely measured SINR and TBLER, which
// made it read as driven. NtnFapiSapBridge now supplies those messages off the
// real MAC-PHY SAP calls, so the slot numbers are the radio's own.


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
    if (g_bridge)
    {
        // Same real per-TB decode, into the bridge's own reverse path, so its
        // RX_DATA/CRC counters and the example's accounting see one event.
        g_bridge->OnPhyRx(p);
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

bool
ReadThreeLineTle(const std::string& path, TleRecord& tle)
{
    std::ifstream f(path);
    if (!f)
    {
        return false;
    }
    if (!std::getline(f, tle.name) || !std::getline(f, tle.line1) || !std::getline(f, tle.line2))
    {
        return false;
    }
    return tle.line1.size() >= 60 && tle.line2.size() >= 60;
}
} // namespace

int
main(int argc, char* argv[])
{
    double simSeconds = 20.0;
    uint32_t scsKhz = 30;
    uint32_t tbBytes = 1500;
    uint32_t numUes = 4;
    double satEirpDbm = -1.0; // sentinel: backend-appropriate default chosen below
    // Default mmwave: this example builds FAPI CRC.indication per TB off the mmwave
    // RxPacketTraceUe trace (mmwave::RxPacketTraceParams), which the nr backend's
    // PHY trace cannot feed; --radio=nr still selects the nr air interface (the
    // helper's measured SINR/TBLER/throughput stay valid) but the per-TB FAPI loop
    // only populates on mmwave.
    std::string radio = "mmwave";
    std::string tlePath;
    std::string outputDir = "ntn-fapi-leo-pass-output";

    CommandLine cmd(__FILE__);
    cmd.AddValue("simSeconds", "Simulation duration (s)", simSeconds);
    cmd.AddValue("scsKhz", "Sub-carrier spacing (kHz): 15/30/60/120", scsKhz);
    cmd.AddValue("tbBytes", "Transport-block size (bytes)", tbBytes);
    cmd.AddValue("numUes", "Number of ground UEs", numUes);
    cmd.AddValue("satEirpDbm", "Satellite EIRP / gNB Tx power (dBm); -1 = backend default", satEirpDbm);
    cmd.AddValue("radio", "Radio backend: mmwave (FR2, per-TB FAPI loop) or nr (FR1)", radio);
    cmd.AddValue("tle", "Path to 3-line TLE (default: contrib/ntn-rrc/data/iss-zarya.tle)", tlePath);
    cmd.AddValue("outputDir", "Output directory", outputDir);
    cmd.Parse(argc, argv);
    g_simSeconds = simSeconds;
    g_tbBytes = tbBytes;
    g_slotsPerSubframe = std::max<uint32_t>(1, scsKhz / 15);

    // Backend-appropriate EIRP default: nr's Friis LEO link needs ~+15 dB vs
    // mmwave, so honour the historical 58 dBm for mmwave but give nr 70 dBm.
    const bool useNr = (radio == "nr");
    if (satEirpDbm < 0.0)
    {
        satEirpDbm = useNr ? 70.0 : 58.0;
    }

    if (tlePath.empty())
    {
        for (const std::string& c : {
                 std::string("contrib/ntn-rrc/data/iss-zarya.tle"),
                 std::string("../contrib/ntn-rrc/data/iss-zarya.tle"),
                 std::string("../../contrib/ntn-rrc/data/iss-zarya.tle"),
             })
        {
            std::ifstream p(c);
            if (p)
            {
                tlePath = c;
                break;
            }
        }
    }
    TleRecord tle;
    if (!ReadThreeLineTle(tlePath, tle))
    {
        std::fprintf(stderr, "error: could not read 3-line TLE from %s\n", tlePath.c_str());
        return 2;
    }

    auto sat = CreateObject<Sgp4MobilityModel>();
    if (!sat->SetTle(tle))
    {
        std::fprintf(stderr, "error: TLE parse failed\n");
        return 2;
    }
    double subLat, subLon, subAlt;
    sat->GetGeodetic(subLat, subLon, subAlt);
    const Vector gsEcef = GeodeticToEcef(subLat, subLon, 540.0);

    NodeContainer satNodes;
    satNodes.Create(1);
    satNodes.Get(0)->AggregateObject(sat);
    NodeContainer ueNodes;
    ueNodes.Create(numUes);
    // TR 38.811 class UEs around the sub-point ground station (real mobility).
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
    rs.SetSimTime(Seconds(simSeconds));
    rs.SetOutputDir(outputDir);
    rs.SetRunTag("ntn-fapi-leo-pass-slotloop");
    // NT-02: declared as CONDUCTED power at the array input. This carrier has
    // no TR 38.821 Set-1 reference in the toolkit, so the EIRP health gate
    // reports "not asserted" rather than certifying an uncalibrated budget.
    rs.SetSatConductedPowerDbm(satEirpDbm);
    rs.Build(satNodes, ueNodes);
    rs.InstallTraffic(NtnRealStackHelper::TrafficProfile::EmbbStreaming,
                      Seconds(1.0), Seconds(simSeconds - 0.5));
    rs.EnableAiFlowMonitor("ntn-fapi-leo-pass-slotloop"); // WS2 KPM series (TS 28.552 names)
    g_rs = &rs;
    g_ueRnti = rs.GetUeRnti(0); // 0 until RRC connects; refreshed in FapiPhyRxTrace

    // Drive FAPI CRC.indication off the SAME real per-TB decode trace the helper
    // connects (RxPacketTraceUe), filtered to UE-0. Connect after Build() so the
    // UE devices exist, matching the helper's own ordering. This mmwave-specific
    // path only matches on the mmwave backend (no-op under nr).
    Config::ConnectWithoutContextFailSafe(
        "/NodeList/*/DeviceList/*/ComponentCarrierMap/*/MmWaveUePhy/DlSpectrumPhy/RxPacketTraceUe",
        MakeCallback(&FapiPhyRxTrace));

    std::printf("# ntn-fapi-leo-pass-slotloop (FAPI ABI on a real mmwave cell over a real SGP4 pass)\n"
                "#   TLE=%s  GS sub-point=(%.3f,%.3f)  sim=%.0fs scs=%ukHz TB=%uB\n",
                tlePath.c_str(), subLat, subLon, simSeconds, scsKhz, tbBytes);

    // Install the FAPI SAP bridge on the live mmwave cell. From here every FAPI
    // message this example reports has a real producer: SLOT.indication from
    // the PHY's own slot tick, DL_TTI/UL_TTI from the scheduler's allocation,
    // TX_DATA from the MAC PDU actually handed down.
    Ptr<mmwave::MmWaveEnbNetDevice> fapiEnb =
        DynamicCast<mmwave::MmWaveEnbNetDevice>(rs.GetEnbDevices().Get(0));
    if (fapiEnb)
    {
        g_bridge = CreateObject<fapi::NtnFapiSapBridge>();
        g_bridge->InstallEnb(fapiEnb);
        g_bridge->SetUeRnti(rs.GetUeRnti(0));
    }
    else
    {
        std::printf("#   NOTE: no mmwave gNB device, FAPI SAP bridge not installed\n");
    }
    Simulator::Stop(Seconds(simSeconds));
    Simulator::Run();
    rs.Collect();
    rs.WriteHealthReport();

    // Goodput is the helper's already-measured real delivered bytes, not the
    // FAPI-loop tally (which is itself now sourced from the real m_corrupt decode).
    const uint64_t realRxBytes = rs.GetUeRxBytes(0);
    const double goodputMbps = (realRxBytes * 8.0) / (simSeconds * 1e6);
    std::printf("# === ntn-fapi-leo-pass-slotloop summary ===\n"
                "#   measured SINR=%.2f dB  measured TBLER=%.4f  measured throughput=%.3f Mbps\n"
                "#   FAPI slots: sent=%lu crcOk=%lu retx=%lu  delivered=%lu KB  goodput=%.3f Mbps\n",
                rs.GetMeanDlSinrDb(), rs.GetMeanDlTbler(), rs.GetRxThroughputMbps(),
                (unsigned long)g_tbSent, (unsigned long)g_tbOk, (unsigned long)g_tbRetx,
                (unsigned long)(realRxBytes / 1000), goodputMbps);

    if (g_bridge)
    {
        std::printf("#   FAPI off the REAL MAC<->PHY SAP (no shadow loop): SLOT.ind=%lu "
                    "DL_TTI=%lu(%lu with data) UL_TTI=%lu(%lu granted) TX_DATA=%lu(%lu KB) "
                    "RACH.ind=%lu\n"
                    "#   SAP latency mean=%.1f us (%s)\n",
                    (unsigned long)g_bridge->GetSlotIndicationCount(),
                    (unsigned long)g_bridge->GetDlTtiRequestCount(),
                    (unsigned long)g_bridge->GetDlTtiWithDataCount(),
                    (unsigned long)g_bridge->GetUlTtiRequestCount(),
                    (unsigned long)g_bridge->GetUlTtiWithGrantCount(),
                    (unsigned long)g_bridge->GetTxDataRequestCount(),
                    (unsigned long)(g_bridge->GetTxDataBytes() / 1000),
                    (unsigned long)g_bridge->GetRachIndicationCount(),
                    g_bridge->GetMeanSapLatencySec() * 1e6,
                    g_bridge->GetSapLatencyProvenance());
    }

    Simulator::Destroy();
    return 0;
}
