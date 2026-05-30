/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only
//
// ntn-fapi-dl-data-slotloop — exercises the SCF-222 FAPI L1<->L2 data ABI as a
// real, sim-time-driven downlink transfer. ntn-fapi is the message ABI between
// the MAC (L2) and PHY (L1); this example runs the actual data path slot by
// slot over NR-NTN timing:
//
//   L2 (MAC):  builds a DL_TTI.request + TX_DATA.request carrying a real
//              transport block (byte buffer) each scheduled slot
//   L1 (PHY):  "transmits" the TB over a satellite link whose SINR follows the
//              live pass geometry, derives a BLER, decides CRC pass/fail, and
//              returns RX_DATA.indication (received bytes) + CRC.indication
//   L2 (MAC):  on CRC NACK schedules a HARQ retransmission; counts delivered
//              transport-block bytes -> goodput
//
// So real data (TB bytes) moves across the FAPI with genuine CRC/HARQ feedback,
// and the delivered goodput tracks the geometry-driven SINR. Everything is
// sim-time scheduled (one event per NR slot) and parameter-dynamic.
//
// Quick test:  --simSeconds=10 --scsKhz=30
#include "ns3/command-line.h"
#include "ns3/core-module.h"

#include "ns3/fapi-messages.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <vector>

using namespace ns3;
using namespace ns3::fapi;

NS_LOG_COMPONENT_DEFINE("NtnFapiDlDataSlotLoop");

namespace
{
// Geometry: a single LEO pass parameterised analytically (elevation rises to
// zenith then sets), driving the per-slot SINR the PHY sees.
double g_simSeconds = 10.0;
double g_minSinrDb = -2.0;
double g_maxSinrDb = 18.0;

// FAPI/MAC state.
uint16_t g_sfn = 0, g_slot = 0;
uint32_t g_slotsPerSubframe = 2; // 30 kHz SCS -> 2 slots / 1 ms subframe
uint32_t g_tbBytes = 1500;       // transport-block size
uint32_t g_harqId = 0;
uint64_t g_tbSent = 0, g_tbOk = 0, g_tbRetx = 0;
uint64_t g_bytesDelivered = 0;
bool g_pendingRetx = false;
Ptr<UniformRandomVariable> g_rng;

// Elevation/SINR model for a symmetric pass over g_simSeconds.
double
SlotSinrDb()
{
    const double t = Simulator::Now().GetSeconds();
    const double frac = std::clamp(t / g_simSeconds, 0.0, 1.0);
    // Triangle: 0 at edges, 1 at mid-pass.
    const double shape = 1.0 - std::abs(2.0 * frac - 1.0);
    return g_minSinrDb + shape * (g_maxSinrDb - g_minSinrDb);
}

// Simple AWGN-ish BLER waterfall around a 2 dB operating point.
double
SinrToBler(double sinrDb)
{
    return 1.0 / (1.0 + std::exp(1.1 * (sinrDb - 2.0)));
}

// One NR slot: L2 builds TX_DATA.request, L1 returns RX_DATA + CRC.indication.
void
SlotTick()
{
    const double sinr = SlotSinrDb();
    const double bler = SinrToBler(sinr);

    // --- L2 (MAC): assemble the DL transport block + TX_DATA.request ---
    TxDataRequest tx;
    tx.sfn = g_sfn;
    tx.slot = g_slot;
    TxDataRequest::PduPayload pdu;
    pdu.pduIndex = 0;
    pdu.cwIndex = 0;
    pdu.tbBytes.assign(g_tbBytes, 0xAB); // real transport-block bytes
    tx.pdus.push_back(pdu);
    ++g_tbSent;
    if (g_pendingRetx)
    {
        ++g_tbRetx;
    }

    // --- L1 (PHY): transmit over the satellite link, decide CRC ---
    const bool crcOk = (g_rng->GetValue() > bler);

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
        prx.tbBytes = tx.pdus[0].tbBytes; // delivered payload
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
    rep.ul_cqi = static_cast<int16_t>(std::lround(sinr));
    crc.crcList.push_back(rep);

    // --- L2 (MAC): consume CRC.indication, drive HARQ ---
    if (crc.crcList[0].tbCrcStatusOk)
    {
        ++g_tbOk;
        g_bytesDelivered += rx.pdus.empty() ? 0 : rx.pdus[0].pduLength;
        g_pendingRetx = false;
        g_harqId = (g_harqId + 1) % 16;
    }
    else
    {
        g_pendingRetx = true; // retransmit this TB next slot (HARQ)
    }

    // Advance NR slot/SFN counters.
    if (++g_slot >= g_slotsPerSubframe * 10) // 10 subframes / frame
    {
        g_slot = 0;
        g_sfn = (g_sfn + 1) % 1024;
    }

    // Periodic log (~ every 1000 slots).
    if (g_tbSent % 1000 == 0)
    {
        std::printf("  t=%6.2f  sfn=%4u slot=%3u  sinr=%5.1f  bler=%5.3f  "
                    "tbOk=%lu/%lu  deliveredKB=%lu\n",
                    Simulator::Now().GetSeconds(), g_sfn, g_slot, sinr, bler,
                    (unsigned long)g_tbOk, (unsigned long)g_tbSent,
                    (unsigned long)(g_bytesDelivered / 1000));
    }

    // NR slot duration = 1 ms / slotsPerSubframe; use ns precision so a
    // 0.5 ms (30 kHz) slot does not truncate to a zero-delay event.
    const int64_t slotDurNs =
        static_cast<int64_t>(1000000.0 / g_slotsPerSubframe);
    if (Simulator::Now().GetSeconds() + slotDurNs / 1e9 < g_simSeconds)
    {
        Simulator::Schedule(NanoSeconds(slotDurNs), &SlotTick);
    }
}
} // namespace

int
main(int argc, char* argv[])
{
    double simSeconds = 10.0;
    uint32_t scsKhz = 30;
    uint32_t tbBytes = 1500;
    double minSinrDb = -2.0;
    double maxSinrDb = 18.0;
    uint32_t rngSeed = 1;

    CommandLine cmd(__FILE__);
    cmd.AddValue("simSeconds", "Simulation duration (s)", simSeconds);
    cmd.AddValue("scsKhz", "Sub-carrier spacing (kHz): 15/30/60/120", scsKhz);
    cmd.AddValue("tbBytes", "Transport-block size (bytes)", tbBytes);
    cmd.AddValue("minSinrDb", "SINR at the pass edges (dB)", minSinrDb);
    cmd.AddValue("maxSinrDb", "SINR at mid-pass / zenith (dB)", maxSinrDb);
    cmd.AddValue("rngSeed", "RNG run number", rngSeed);
    cmd.Parse(argc, argv);

    RngSeedManager::SetRun(rngSeed);
    g_simSeconds = simSeconds;
    g_tbBytes = tbBytes;
    g_minSinrDb = minSinrDb;
    g_maxSinrDb = maxSinrDb;
    g_slotsPerSubframe = std::max<uint32_t>(1, scsKhz / 15); // 15kHz=1, 30=2, ...
    g_rng = CreateObject<UniformRandomVariable>();

    std::printf("# ntn-fapi-dl-data-slotloop (SCF-222 L1<->L2 data ABI)\n");
    std::printf("#   sim=%.1fs scs=%ukHz (%u slots/ms) TB=%uB sinr=%.0f..%.0fdB\n",
                simSeconds, scsKhz, g_slotsPerSubframe, tbBytes, minSinrDb,
                maxSinrDb);
    std::printf("# %6s  %14s  %5s  %5s  %14s  %s\n",
                "t_s", "sfn/slot", "sinr", "bler", "tbOk/tbSent", "deliveredKB");

    Simulator::Schedule(Seconds(0.0), &SlotTick);
    Simulator::Stop(Seconds(simSeconds));
    Simulator::Run();

    const double secs = simSeconds;
    const double goodputMbps = g_bytesDelivered * 8.0 / secs / 1e6;
    const double bler = g_tbSent ? 1.0 - double(g_tbOk) / g_tbSent : 0.0;
    std::printf("# === summary ===  TBs sent=%lu ok=%lu HARQ-retx=%lu "
                "avgBLER=%.3f  deliveredMB=%.2f  goodput=%.3f Mbps\n",
                (unsigned long)g_tbSent, (unsigned long)g_tbOk,
                (unsigned long)g_tbRetx, bler, g_bytesDelivered / 1e6,
                goodputMbps);
    Simulator::Destroy();
    return 0;
}
