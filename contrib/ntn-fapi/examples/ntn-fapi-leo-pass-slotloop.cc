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
#include "ns3/core-module.h"
#include "ns3/mmwave-enb-net-device.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/ntn-real-stack-helper.h"
#include "ns3/ntn-tr38811-mobility-model.h"

#include "ns3/fapi-messages.h"
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
Ptr<UniformRandomVariable> g_rng;
double g_simSeconds = 20.0;
uint32_t g_slotsPerSubframe = 2;
uint32_t g_tbBytes = 1500;
uint16_t g_sfn = 0, g_slot = 0;
uint32_t g_harqId = 0;
uint64_t g_tbSent = 0, g_tbOk = 0, g_tbRetx = 0;
uint64_t g_bytesDelivered = 0;
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

void
SlotTick()
{
    const double measuredTbler = g_rs->GetUeRecentTbler(0);
    const double sinr = g_rs->GetUeRecentSinrDb(0);
    const bool haveMeas = !std::isnan(measuredTbler);

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
    rep.ul_cqi = static_cast<int16_t>(std::lround(std::max(-10.0, std::isnan(sinr) ? 0.0 : sinr)));
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
    if (Simulator::Now().GetSeconds() + slotDurNs / 1e9 < g_simSeconds)
    {
        Simulator::Schedule(NanoSeconds(slotDurNs), &SlotTick);
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
    double satEirpDbm = 58.0;
    std::string tlePath;
    std::string outputDir = "ntn-fapi-leo-pass-output";

    CommandLine cmd(__FILE__);
    cmd.AddValue("simSeconds", "Simulation duration (s)", simSeconds);
    cmd.AddValue("scsKhz", "Sub-carrier spacing (kHz): 15/30/60/120", scsKhz);
    cmd.AddValue("tbBytes", "Transport-block size (bytes)", tbBytes);
    cmd.AddValue("numUes", "Number of ground UEs", numUes);
    cmd.AddValue("satEirpDbm", "Satellite EIRP / gNB Tx power (dBm)", satEirpDbm);
    cmd.AddValue("tle", "Path to 3-line TLE (default: contrib/ntn-rrc/data/iss-zarya.tle)", tlePath);
    cmd.AddValue("outputDir", "Output directory", outputDir);
    cmd.Parse(argc, argv);
    g_simSeconds = simSeconds;
    g_tbBytes = tbBytes;
    g_slotsPerSubframe = std::max<uint32_t>(1, scsKhz / 15);

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
    rs.SetSimTime(Seconds(simSeconds));
    rs.SetOutputDir(outputDir);
    rs.SetRunTag("ntn-fapi-leo-pass-slotloop");
    rs.SetSatEirpDbm(satEirpDbm);
    rs.Build(satNodes, ueNodes);
    rs.InstallTraffic(NtnRealStackHelper::TrafficProfile::EmbbStreaming,
                      Seconds(1.0), Seconds(simSeconds - 0.5));
    g_rs = &rs;
    g_rng = CreateObject<UniformRandomVariable>();

    std::printf("# ntn-fapi-leo-pass-slotloop (FAPI ABI on a real mmwave cell over a real SGP4 pass)\n"
                "#   TLE=%s  GS sub-point=(%.3f,%.3f)  sim=%.0fs scs=%ukHz TB=%uB\n",
                tlePath.c_str(), subLat, subLon, simSeconds, scsKhz, tbBytes);

    Simulator::Schedule(Seconds(1.0), &SlotTick);
    Simulator::Stop(Seconds(simSeconds));
    Simulator::Run();
    rs.Collect();
    rs.WriteHealthReport();

    const double goodputMbps = (g_bytesDelivered * 8.0) / (simSeconds * 1e6);
    std::printf("# === ntn-fapi-leo-pass-slotloop summary ===\n"
                "#   measured SINR=%.2f dB  measured TBLER=%.4f  measured throughput=%.3f Mbps\n"
                "#   FAPI slots: sent=%lu crcOk=%lu retx=%lu  delivered=%lu KB  goodput=%.3f Mbps\n",
                rs.GetMeanDlSinrDb(), rs.GetMeanDlTbler(), rs.GetRxThroughputMbps(),
                (unsigned long)g_tbSent, (unsigned long)g_tbOk, (unsigned long)g_tbRetx,
                (unsigned long)(g_bytesDelivered / 1000), goodputMbps);

    Simulator::Destroy();
    return 0;
}
