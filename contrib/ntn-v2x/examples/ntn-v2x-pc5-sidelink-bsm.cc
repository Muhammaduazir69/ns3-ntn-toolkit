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

#include <cstdio>
#include <vector>

using namespace ns3;

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

    CommandLine cmd;
    cmd.AddValue("numVehicles", "Vehicles on the highway", numVehicles);
    cmd.AddValue("numSubchannels", "SL-ResourcePool subchannels", numSubchannels);
    cmd.AddValue("spacingM", "Inter-vehicle spacing (m)", spacingM);
    cmd.AddValue("bsmPeriodMs", "BSM period (ms)", bsmPeriodMs);
    cmd.AddValue("duration", "Sim duration (s)", durationS);
    cmd.AddValue("txPowerDbm", "PC5 Tx power (dBm)", txPowerDbm);
    cmd.Parse(argc, argv);

    auto ch = CreateObject<NtnSlChannel>();
    NtnSlResourcePool pool;
    pool.numSubchannels = numSubchannels;
    pool.slotDuration = MilliSeconds(1); // mu=0
    ch->SetResourcePool(pool);
    ch->SetTxPowerDbm(txPowerDbm);
    ch->SetDecodeThresholdDbm(-115.0);

    const uint32_t prsvpSlots = static_cast<uint32_t>(bsmPeriodMs); // @ 1 ms slots

    std::vector<uint32_t> rxPerUe(numVehicles, 0);
    for (uint32_t i = 0; i < numVehicles; ++i)
    {
        auto ue = CreateObject<NtnSlUeMac>();
        ue->SetUeId(i);
        auto mob = CreateObject<ConstantVelocityMobilityModel>();
        mob->SetPosition(Vector(spacingM * i, 0.0, 0.0));
        mob->SetVelocity(Vector(speedMps, 0.0, 0.0));
        ue->SetMobility(mob);
        // Selection window within the packet delay budget (one BSM period).
        ue->SetSelectionWindow(1, prsvpSlots);
        ue->SetReservationPeriod(prsvpSlots);
        ue->SetPacketBytes(190);
        ue->AssignStreams(1 + i);
        uint32_t self = i;
        NtnSlUeMac::SlRxCallback cb = [&rxPerUe, self](uint32_t, uint32_t) { rxPerUe[self]++; };
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
    std::printf("#   PRR (TS 38.885) by distance:\n");
    for (double r : {50.0, 100.0, 200.0, 300.0, 500.0})
    {
        std::printf("#     within %4.0f m : PRR = %.3f\n", r, ch->GetPrrWithinRange(r));
    }

    Simulator::Destroy();
    return 0;
}
