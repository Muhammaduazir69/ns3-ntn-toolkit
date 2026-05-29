/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only
//
// ntn-fapi-leo-pass-slotloop — the SCF-222 FAPI L1<->L2 data ABI driven by a
// REAL SGP4 LEO pass instead of an analytical SINR triangle. This is the
// cross-module sibling of ntn-fapi-dl-data-slotloop: the per-slot SINR the
// PHY sees is derived from the live elevation of a satellite propagated by
// ntn-constellation's Sgp4MobilityModel over a ground station, so the
// delivered FAPI goodput tracks genuine orbital geometry.
//
// Cross-module composition:
//   * `ntn-constellation` : Sgp4MobilityModel propagates the orbit; the GS is
//                           auto-placed under the t=0 sub-point so a real
//                           rise->zenith->set pass occurs.
//   * `ntn-fapi`          : the L1<->L2 message ABI (DL_TTI / TX_DATA /
//                           RX_DATA / CRC.indication) carries real TB bytes
//                           slot by slot with CRC + HARQ feedback.
//
// Data path per NR slot:
//   L2 (MAC):  TX_DATA.request with a real transport block (byte buffer)
//   L1 (PHY):  maps the current elevation -> slant-range path loss -> SINR,
//              derives BLER, decides CRC pass/fail, returns RX_DATA.indication
//   L2 (MAC):  on CRC NACK schedules a HARQ retx; counts delivered TB bytes
//
// Quick test:  --simSeconds=600 --scsKhz=30 --tle=contrib/ntn-rrc/data/iss-zarya.tle

#include "ns3/command-line.h"
#include "ns3/core-module.h"

#include "ns3/fapi-messages.h"

#include "ns3/sgp4-mobility-model.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <vector>

using namespace ns3;
using namespace ns3::fapi;
using ns3::ntncon::Sgp4MobilityModel;
using ns3::ntncon::TleRecord;

NS_LOG_COMPONENT_DEFINE("NtnFapiLeoPassSlotLoop");

namespace
{

// --- Geometry (ntn-constellation) -------------------------------------------
Ptr<Sgp4MobilityModel> g_sat;
double g_gsLatDeg = 0.0;
double g_gsLonDeg = 0.0;

// --- FAPI/MAC state ----------------------------------------------------------
double g_simSeconds = 600.0;
uint16_t g_sfn = 0, g_slot = 0;
uint32_t g_slotsPerSubframe = 2;
uint32_t g_tbBytes = 1500;
uint32_t g_harqId = 0;
uint64_t g_tbSent = 0, g_tbOk = 0, g_tbRetx = 0;
uint64_t g_bytesDelivered = 0;
uint64_t g_slotsInContact = 0;
double g_maxElevDeg = 0.0;
bool g_pendingRetx = false;
Ptr<UniformRandomVariable> g_rng;

// Map the live elevation to an SINR. Below the horizon (elev <= 0) the link is
// fully obstructed (no signal). Above it, SINR grows with elevation because the
// slant range — and therefore the free-space path loss — shrinks as the
// satellite climbs toward zenith. Anchored so a 10 deg elevation sits near the
// decoding cliff and zenith is comfortably in the green.
double
ElevationToSinrDb(double elevDeg)
{
    if (elevDeg <= 0.0)
    {
        return -100.0; // below horizon: no link
    }
    // Slant range to a 550 km shell as a function of elevation (spherical
    // Earth law of sines); normalise loss to the zenith case.
    const double Re = 6'371'000.0;
    const double h = 550'000.0;
    const double el = elevDeg * M_PI / 180.0;
    const double slant =
        std::sqrt(Re * Re * std::sin(el) * std::sin(el) + h * h + 2 * Re * h) -
        Re * std::sin(el);
    const double slantZenith = h;
    const double extraLossDb = 20.0 * std::log10(slant / slantZenith);
    const double zenithSinrDb = 18.0;
    return zenithSinrDb - extraLossDb;
}

double
SinrToBler(double sinrDb)
{
    return 1.0 / (1.0 + std::exp(1.1 * (sinrDb - 2.0)));
}

void
SlotTick()
{
    const double elev = g_sat->GetElevationDeg(g_gsLatDeg, g_gsLonDeg);
    g_maxElevDeg = std::max(g_maxElevDeg, elev);
    const double sinr = ElevationToSinrDb(elev);
    const bool inContact = elev > 0.0;
    if (inContact)
    {
        ++g_slotsInContact;
    }
    const double bler = inContact ? SinrToBler(sinr) : 1.0;

    // L2 (MAC): assemble TX_DATA.request with a real transport block.
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

    // L1 (PHY): out of contact => guaranteed CRC failure (no link).
    const bool crcOk = inContact && (g_rng->GetValue() > bler);

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
    rep.ul_cqi = static_cast<int16_t>(std::lround(std::max(-10.0, sinr)));
    crc.crcList.push_back(rep);

    // L2 (MAC): consume CRC, drive HARQ.
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

    if (g_tbSent % 5000 == 0)
    {
        std::printf("  t=%6.1f  elev=%5.1f  sinr=%6.1f  bler=%5.3f  "
                    "tbOk=%lu/%lu  deliveredKB=%lu\n",
                    Simulator::Now().GetSeconds(), elev, sinr, bler,
                    (unsigned long)g_tbOk, (unsigned long)g_tbSent,
                    (unsigned long)(g_bytesDelivered / 1000));
    }

    const int64_t slotDurNs =
        static_cast<int64_t>(1000000.0 / g_slotsPerSubframe);
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
    if (!std::getline(f, tle.name) || !std::getline(f, tle.line1) ||
        !std::getline(f, tle.line2))
    {
        return false;
    }
    return tle.line1.size() >= 60 && tle.line2.size() >= 60;
}

} // namespace

int
main(int argc, char* argv[])
{
    double simSeconds = 600.0;
    uint32_t scsKhz = 30;
    uint32_t tbBytes = 1500;
    uint32_t rngSeed = 1;
    std::string tlePath;
    double gsLatDeg = std::nan("");
    double gsLonDeg = std::nan("");

    CommandLine cmd(__FILE__);
    cmd.AddValue("simSeconds", "Simulation duration (s)", simSeconds);
    cmd.AddValue("scsKhz", "Sub-carrier spacing (kHz): 15/30/60/120", scsKhz);
    cmd.AddValue("tbBytes", "Transport-block size (bytes)", tbBytes);
    cmd.AddValue("rngSeed", "RNG run number", rngSeed);
    cmd.AddValue("tle",
                   "Path to 3-line TLE (default: contrib/ntn-rrc/data/iss-zarya.tle)",
                   tlePath);
    cmd.AddValue("gsLat",
                   "Ground-station latitude (deg; default = sat sub-point)",
                   gsLatDeg);
    cmd.AddValue("gsLon",
                   "Ground-station longitude (deg; default = sat sub-point)",
                   gsLonDeg);
    cmd.Parse(argc, argv);

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
        std::fprintf(stderr, "error: could not read 3-line TLE from %s\n",
                       tlePath.c_str());
        return 2;
    }

    g_sat = CreateObject<Sgp4MobilityModel>();
    if (!g_sat->SetTle(tle))
    {
        std::fprintf(stderr, "error: TLE parse failed\n");
        return 2;
    }
    if (std::isnan(gsLatDeg) || std::isnan(gsLonDeg))
    {
        double subAlt;
        g_sat->GetGeodetic(gsLatDeg, gsLonDeg, subAlt);
    }
    g_gsLatDeg = gsLatDeg;
    g_gsLonDeg = gsLonDeg;

    RngSeedManager::SetRun(rngSeed);
    g_simSeconds = simSeconds;
    g_tbBytes = tbBytes;
    g_slotsPerSubframe = std::max<uint32_t>(1, scsKhz / 15);
    g_rng = CreateObject<UniformRandomVariable>();

    std::printf("# ntn-fapi-leo-pass-slotloop "
                "(FAPI L1<->L2 ABI driven by a real SGP4 pass)\n");
    std::printf("#   TLE=%s  GS(lat,lon)=(%.3f,%.3f)\n", tlePath.c_str(),
                g_gsLatDeg, g_gsLonDeg);
    std::printf("#   sim=%.0fs scs=%ukHz (%u slots/ms) TB=%uB\n", simSeconds,
                scsKhz, g_slotsPerSubframe, tbBytes);

    Simulator::Schedule(Seconds(0.0), &SlotTick);
    Simulator::Stop(Seconds(simSeconds));
    Simulator::Run();
    Simulator::Destroy();

    const double goodputMbps =
        (g_bytesDelivered * 8.0) / (simSeconds * 1e6);
    std::printf("# === ntn-fapi-leo-pass-slotloop summary ===\n"
                "#   slots: sent=%lu ok=%lu retx=%lu  inContact=%lu\n"
                "#   maxElev=%.1f deg  deliveredKB=%lu  goodput=%.3f Mbps\n",
                (unsigned long)g_tbSent, (unsigned long)g_tbOk,
                (unsigned long)g_tbRetx, (unsigned long)g_slotsInContact,
                g_maxElevDeg, (unsigned long)(g_bytesDelivered / 1000),
                goodputMbps);
    return 0;
}
