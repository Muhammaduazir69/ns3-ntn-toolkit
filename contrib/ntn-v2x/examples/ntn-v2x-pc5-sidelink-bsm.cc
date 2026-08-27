/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only
//
// NR PC5 sidelink (V1) — vehicles broadcast SAE J2735 BSMs directly over PC5
// with autonomous Mode-2 resource selection (TS 38.321 §5.22), NO gNB in the
// loop. Reports the TS 38.885 KPI: Packet Reception Ratio (PRR) vs distance.
//
// Contrast with ntn-v2x-real-stack (Uu relay via a satellite gNB): here the V2V
// safety layer is genuine sidelink — sensing-based selection, half-duplex, and
// co-channel collisions all shape the delivered PRR.

#include "ns3/command-line.h"
#include "ns3/constant-velocity-mobility-model.h"
#include "ns3/core-module.h"
#include "ns3/ntn-nr-sidelink.h"
#include "ns3/ntn-v2x-bsm-header.h"
#include "ns3/packet.h"
#include "ns3/simulator.h"

#include <filesystem>
#include <fstream>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <cstdio>
#include <map>
#include <vector>

using namespace ns3;
using ns3::ntnv2x::NtnV2xBsmHeader;

int
main(int argc, char* argv[])
{
    uint32_t numVehicles = 20;
    uint32_t numSubchannels = 5;
    double spacingM = 25.0;     // inter-vehicle spacing along the highway
    double speedMps = 27.0;     // ~100 km/h
    double bsmPeriodMs = 100.0; // 10 Hz BSM cadence (SAE J2735 typical)
    double durationS = 5.0;
    double txPowerDbm = 23.0;
    std::string outputDir = "ntn-v2x-pc5-sidelink-out";

    CommandLine cmd;
    cmd.AddValue("numVehicles", "Vehicles on the highway", numVehicles);
    cmd.AddValue("numSubchannels", "SL-ResourcePool subchannels", numSubchannels);
    cmd.AddValue("spacingM", "Inter-vehicle spacing (m)", spacingM);
    cmd.AddValue("bsmPeriodMs", "BSM period (ms)", bsmPeriodMs);
    cmd.AddValue("duration", "Sim duration (s)", durationS);
    cmd.AddValue("txPowerDbm", "PC5 Tx power (dBm)", txPowerDbm);
    cmd.AddValue("outputDir",
                 "WF-15: directory for the persisted result. This example printed its PRR to "
                 "stdout and wrote nothing, the only one of the toolkit's examples with no "
                 "persisted result at all, so a sweep over it had nothing to collect.",
                 outputDir);
    cmd.Parse(argc, argv);

    auto ch = CreateObject<NtnSlChannel>();
    NtnSlResourcePool pool;
    pool.numSubchannels = numSubchannels;
    pool.slotDuration = MilliSeconds(1); // mu=0
    ch->SetResourcePool(pool);
    ch->SetTxPowerDbm(txPowerDbm);
    ch->SetDecodeThresholdDbm(-115.0);

    const uint32_t prsvpSlots = static_cast<uint32_t>(bsmPeriodMs); // @ 1 ms slots

    // V2X-3. Every transmission now carries a real SAE J2735 basic safety
    // message as packet bytes, and the receiver deserialises it. Before this
    // the sidelink delivered an (id, byte count) pair: NtnV2xBsmHeader existed
    // but only ever crossed the Uu path, so the README's claim that vehicles
    // broadcast BSMs over PC5 described something the code did not do.
    std::vector<uint32_t> rxPerUe(numVehicles, 0);
    std::vector<Ptr<MobilityModel>> mobs(numVehicles);
    std::vector<uint8_t> msgCnt(numVehicles, 0);
    // Per (receiver, sender) continuity bookkeeping, which is only checkable
    // because a real header now arrives.
    std::vector<std::map<uint32_t, int>> lastCnt(numVehicles);
    uint64_t bsmDecoded = 0;
    uint64_t bsmSeqBreaks = 0;
    double maxPosErrM = 0.0;

    for (uint32_t i = 0; i < numVehicles; ++i)
    {
        auto ue = CreateObject<NtnSlUeMac>();
        ue->SetUeId(i);
        auto mob = CreateObject<ConstantVelocityMobilityModel>();
        mob->SetPosition(Vector(spacingM * i, 0.0, 0.0));
        mob->SetVelocity(Vector(speedMps, 0.0, 0.0));
        ue->SetMobility(mob);
        mobs[i] = mob;
        // Selection window within the packet delay budget (one BSM period).
        ue->SetSelectionWindow(1, prsvpSlots);
        ue->SetReservationPeriod(prsvpSlots);
        ue->SetPacketBytes(190);
        ue->AssignStreams(1 + i);
        uint32_t self = i;

        // Build one BSM per transmission opportunity, stamped with this
        // vehicle's CURRENT kinematics and an incrementing J2735 msgCnt. The
        // highway runs along +x, so a metre of x maps to a degree of longitude
        // through a fixed scale; the point is that the receiver reads back the
        // sender's real position, not that the projection is a survey.
        NtnSlUeMac::SlTxPacketCallback txCb =
            [&mobs, &msgCnt, spacingM](uint32_t id) -> Ptr<Packet> {
            const Vector p = mobs[id]->GetPosition();
            const Vector v = mobs[id]->GetVelocity();
            NtnV2xBsmHeader h;
            h.SetFromState(msgCnt[id],
                           id,
                           static_cast<uint16_t>(Simulator::Now().GetMilliSeconds() % 60000),
                           p.y / 111320.0,  // metres -> degrees latitude
                           p.x / 111320.0,  // metres -> degrees longitude at the equator
                           p.z,
                           v.GetLength(),
                           (v.x >= 0.0) ? 90.0 : 270.0);
            msgCnt[id] = static_cast<uint8_t>((msgCnt[id] + 1) % 128); // J2735 wraps at 127
            Ptr<Packet> pkt = Create<Packet>(190 - h.GetSerializedSize());
            pkt->AddHeader(h);
            return pkt;
        };
        ue->SetTxPacketCallback(txCb);

        NtnSlUeMac::SlRxCallback cb =
            [&rxPerUe, &lastCnt, &mobs, &bsmDecoded, &bsmSeqBreaks, &maxPosErrM, self](
                uint32_t from, Ptr<Packet> pkt) {
                rxPerUe[self]++;
                NtnV2xBsmHeader h;
                if (pkt->GetSize() < h.GetSerializedSize())
                {
                    return;
                }
                pkt->RemoveHeader(h);
                ++bsmDecoded;
                // The sender's identity must survive the air interface.
                if (h.GetId() != from)
                {
                    ++bsmSeqBreaks;
                    return;
                }
                // msgCnt continuity: consecutive BSMs from one sender differ by
                // one modulo 128. A gap is a real loss, which is exactly the
                // signal a safety application would act on.
                auto it = lastCnt[self].find(from);
                if (it != lastCnt[self].end())
                {
                    const int expected = (it->second + 1) % 128;
                    if (static_cast<int>(h.GetMsgCnt()) != expected)
                    {
                        ++bsmSeqBreaks;
                    }
                }
                lastCnt[self][from] = static_cast<int>(h.GetMsgCnt());
                // The decoded position must be the sender's actual position.
                const double decodedX = h.GetLonDeg() * 111320.0;
                maxPosErrM = std::max(maxPosErrM, std::abs(decodedX - mobs[from]->GetPosition().x));
            };
        ue->SetRxCallback(cb);
        ch->AddUe(ue);
    }

    ch->Start(MilliSeconds(1), Seconds(durationS));
    Simulator::Stop(Seconds(durationS) + MilliSeconds(1));
    Simulator::Run();

    std::printf("# NR PC5 sidelink Mode 2 — %u vehicles, %u subchannels, "
                "BSM %.0f ms, %.0f s\n",
                numVehicles, numSubchannels, bsmPeriodMs, durationS);
    std::printf("#   total TX BSMs         : %llu\n",
                static_cast<unsigned long long>(ch->GetTxTotal()));
    std::printf("#   RX attempts / success : %llu / %llu\n",
                static_cast<unsigned long long>(ch->GetRxAttemptTotal()),
                static_cast<unsigned long long>(ch->GetRxSuccessTotal()));
    std::printf("#   BSMs decoded          : %llu (J2735 header read back from PC5 bytes)\n",
                static_cast<unsigned long long>(bsmDecoded));
    std::printf("#   msgCnt discontinuities: %llu (gaps = genuine losses)\n",
                static_cast<unsigned long long>(bsmSeqBreaks));
    std::printf("#   max position error    : %.3f m (decode vs sender truth)\n", maxPosErrM);
    std::printf("#   PC5 one-way delay    : mean %.3f ms, max %.3f ms over %llu receptions\n",
                ch->GetMeanDelayMs(), ch->GetMaxDelayMs(),
                static_cast<unsigned long long>(ch->GetDelaySampleCount()));
    std::printf("#   PRR (TS 38.885) by distance:\n");
    for (double r : {50.0, 100.0, 200.0, 300.0, 500.0})
    {
        std::printf("#     within %4.0f m : PRR = %.3f\n", r, ch->GetPrrWithinRange(r));
    }

    // WF-15: persist the result.
    //
    // This example printed its PRR to stdout and wrote no CSV or JSON, the only
    // one of the toolkit's examples with nothing persisted. A sweep, a
    // regression check or a figure script had nothing to read, and the numbers
    // existed only in whatever terminal happened to run it.
    {
        std::error_code ec;
        std::filesystem::create_directories(outputDir, ec);
        const std::string path = outputDir + "/pc5_sidelink_prr.csv";
        std::ofstream csv(path);
        if (!csv)
        {
            std::printf("#   WARNING: could not write %s\n", path.c_str());
        }
        else
        {
            csv << "metric,value,unit,provenance\n";
            csv << "num_vehicles," << numVehicles << ",count,config\n";
            csv << "num_subchannels," << numSubchannels << ",count,config\n";
            csv << "spacing_m," << spacingM << ",m,config\n";
            csv << "bsm_period_ms," << bsmPeriodMs << ",ms,config\n";
            csv << "tx_power_dbm," << txPowerDbm << ",dBm,config\n";
            csv << "duration_s," << durationS << ",s,config\n";
            csv << "tx_bsms," << ch->GetTxTotal() << ",count,sidelink-mac\n";
            csv << "rx_attempts," << ch->GetRxAttemptTotal() << ",count,sidelink-mac\n";
            csv << "rx_success," << ch->GetRxSuccessTotal() << ",count,sidelink-mac\n";
            csv << "bsms_decoded," << bsmDecoded << ",count,j2735-header\n";
            csv << "msgcnt_discontinuities," << bsmSeqBreaks << ",count,j2735-header\n";
            csv << "max_position_error_m," << maxPosErrM << ",m,decode-vs-truth\n";
            csv << "pc5_owd_mean_ms," << ch->GetMeanDelayMs() << ",ms,sidelink-mac\n";
            csv << "pc5_owd_max_ms," << ch->GetMaxDelayMs() << ",ms,sidelink-mac\n";
            csv << "pc5_delay_samples," << ch->GetDelaySampleCount() << ",count,sidelink-mac\n";
            for (double r : {50.0, 100.0, 200.0, 300.0, 500.0})
            {
                csv << "prr_within_" << static_cast<int>(r) << "m,"
                    << ch->GetPrrWithinRange(r) << ",ratio,ts38885\n";
            }
            std::printf("#   wrote %s\n", path.c_str());
        }
    }

    Simulator::Destroy();
    return 0;
}
