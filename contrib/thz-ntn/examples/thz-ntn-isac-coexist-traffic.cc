/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only
//
// thz-ntn-isac-coexist-traffic — joint communication + sensing coexistence on
// a REAL mmwave NR NTN cell (NtnRealStackHelper). A ThzNtnIsacScheduler
// partitions the THz resource grid between comm and sensing per the active
// ThzNtnIsac mode; the comm share drives the TIME-DOMAIN duty cycle of the
// real downlink — during sensing slots the shared-aperture beam is steered to
// the radar target, so the comm path is gated OFF in the live channel (a real
// reconfiguration packets feel), exactly how a monostatic ISAC payload
// time-shares its array.
//
// Audit fix (2026-06 protocol-fidelity audit): the data plane was a
// P2P link whose CAPACITY attribute was throttled — no radio, placeholder
// nodes. Here the satellite flies a genuine SGP4 orbit; as the ISAC mode is
// stepped (COMM_ONLY → COMM_CENTRIC → JOINT → SENSING_CENTRIC → SENSING_ONLY)
// the MEASURED goodput drops in proportion to the comm share decided by the
// module's real scheduler over a real sub-band grid — nothing hardcoded.
//
// Quick test:  --simSeconds=50 --numSubBands=20
#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/ntn-real-stack-helper.h"
#include "ns3/ntn-static-extra-loss-model.h"
#include "ns3/ntn-tr38811-mobility-model.h"
#include "ns3/sgp4-mobility-model.h"
#include "ns3/thz-ntn-isac-scheduler.h"
#include "ns3/thz-ntn-isac.h"
#include "ns3/thz-ntn-mac-scheduler.h"
#include "ns3/walker-constellation.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("ThzNtnIsacCoexistTraffic");

namespace
{

Ptr<ThzNtnIsac> g_isac;
Ptr<ThzNtnIsacScheduler> g_sched;
Ptr<NtnStaticExtraLossModel> g_gate; // 0 dB in comm slots, blocked in sensing slots
std::vector<ThzNtnSubBand> g_subBands;
std::vector<ThzNtnUeContext> g_ues;
uint32_t g_commSlotsPerFrame = 10; // out of 10 x 100 ms slots per 1 s frame
uint32_t g_slotIndex = 0;
const char* g_modeName = "?";

const char*
ModeName(IsacMode m)
{
    switch (m)
    {
    case COMMUNICATION_ONLY:
        return "COMM_ONLY";
    case COMMUNICATION_CENTRIC:
        return "COMM_CENTRIC";
    case JOINT_ISAC:
        return "JOINT_ISAC";
    case SENSING_CENTRIC:
        return "SENSING_CENTRIC";
    case SENSING_ONLY:
        return "SENSING_ONLY";
    }
    return "?";
}

void
SetMode(IsacMode m)
{
    g_isac->SetIsacMode(m);
    g_modeName = ModeName(m);
    // The module's REAL scheduler decides the comm share over the grid.
    const auto decision = g_sched->Schedule(g_ues, g_subBands);
    const double share =
        decision.numSubBandsTotal
            ? static_cast<double>(decision.commSubBandIndices.size()) /
                  decision.numSubBandsTotal
            : 0.0;
    g_commSlotsPerFrame = static_cast<uint32_t>(std::lround(share * 10.0));
}

// 100 ms slot clock: gate the live channel by the scheduler's comm share.
// During sensing slots the shared aperture is on the radar target -> the comm
// path is blocked in the REAL channel chain.
void
SlotTick()
{
    const bool commSlot = (g_slotIndex % 10) < g_commSlotsPerFrame;
    g_gate->SetLossDb(commSlot ? 0.0 : 200.0);
    ++g_slotIndex;
    Simulator::Schedule(MilliSeconds(100), &SlotTick);
}

} // namespace

int
main(int argc, char* argv[])
{
    double simSeconds = 50.0;
    double freqGHz = 100.0; // sub-THz (3GPP spectrum model upper bound)
    double satEirpDbm = 115.0;
    uint32_t numSubBands = 20;
    uint32_t numUes = 4;
    std::string outputDir = "thz-ntn-isac-coexist-output";

    CommandLine cmd(__FILE__);
    cmd.AddValue("simSeconds", "Simulation duration (s)", simSeconds);
    cmd.AddValue("freqGHz", "Carrier frequency (GHz), capped at 100", freqGHz);
    cmd.AddValue("satEirpDbm", "Satellite EIRP / gNB Tx power (dBm)", satEirpDbm);
    cmd.AddValue("numSubBands", "Number of THz sub-bands in the grid", numSubBands);
    cmd.AddValue("numUes", "Number of comm UE contexts for the scheduler", numUes);
    cmd.AddValue("outputDir", "Output directory", outputDir);
    cmd.Parse(argc, argv);

    if (freqGHz > 100.0)
    {
        freqGHz = 100.0;
    }

    // --- the module's REAL ISAC scheduler over a real sub-band grid ---
    g_isac = CreateObject<ThzNtnIsac>();
    Ptr<ThzNtnMacScheduler> comm = CreateObject<ThzNtnMacScheduler>();
    g_sched = CreateObject<ThzNtnIsacScheduler>();
    g_sched->SetIsac(g_isac);
    g_sched->SetCommScheduler(comm);
    for (uint32_t i = 0; i < numSubBands; ++i)
    {
        ThzNtnSubBand sb;
        sb.index = i;
        sb.centerFreqHz = 100e9 + i * 1e9;
        sb.bandwidthHz = 1e9;
        sb.isAvailable = true;
        sb.absorptionLoss_dB = 0.0;
        sb.assignedUeId = 0;
        sb.carrierIndex = i;
        g_subBands.push_back(sb);
    }
    for (uint32_t i = 0; i < numUes; ++i)
    {
        ThzNtnUeContext u;
        u.ueId = 100 + i;
        u.rnti = 1 + static_cast<uint16_t>(i);
        u.qosClass = ThzNtnQosClass::EMBB;
        u.sinr_dB = 15.0;
        u.throughput_Mbps = 100.0;
        u.dopplerHz = 0.0;
        u.mcsIndex = 10;
        u.bufferSize_bytes = 200000;
        g_ues.push_back(u);
    }

    // --- REAL mobility + REAL radio ---
    NodeContainer satNodes;
    satNodes.Create(1);
    NodeContainer ueNodes;
    ueNodes.Create(1);

    ns3::ntncon::WalkerConfig wcfg;
    wcfg.num_planes = 1;
    wcfg.total_sats = 80;
    wcfg.altitude_km = 550.0;
    wcfg.inclination_deg = 53.0;
    wcfg.epoch_unix_s = 1735689600.0;
    const auto elements = ns3::ntncon::WalkerConstellation::BuildDelta(wcfg);
    Ptr<ns3::ntncon::Sgp4MobilityModel> satSgp4 =
        CreateObject<ns3::ntncon::Sgp4MobilityModel>();
    satSgp4->SetElements(elements[0]);
    double subLat, subLon, subAlt;
    satSgp4->GetGeodetic(subLat, subLon, subAlt);
    Ptr<NtnEnuProjectionMobilityModel> satEnu = CreateObject<NtnEnuProjectionMobilityModel>();
    satEnu->SetSource(satSgp4);
    satEnu->SetReference(subLat, subLon, 0.0);
    satNodes.Get(0)->AggregateObject(satEnu);

    MobilityHelper mob;
    mob.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    Ptr<ListPositionAllocator> uePos = CreateObject<ListPositionAllocator>();
    uePos->Add(Vector(0.0, 0.0, 1.5));
    mob.SetPositionAllocator(uePos);
    mob.Install(ueNodes);

    NtnRealStackHelper rs;
    rs.SetSimTime(Seconds(simSeconds));
    rs.SetOutputDir(outputDir);
    rs.SetRunTag("thz-ntn-isac-coexist-traffic");
    rs.SetCarrierFrequencyHz(freqGHz * 1e9);
    rs.SetSatEirpDbm(satEirpDbm);
    rs.Build(satNodes, ueNodes);

    g_gate = CreateObject<NtnStaticExtraLossModel>();
    g_gate->SetLossDb(0.0);
    rs.AddExtraPropagationLoss(g_gate);

    rs.InstallTraffic(NtnRealStackHelper::TrafficProfile::EmbbStreaming,
                      Seconds(1.0), Seconds(simSeconds - 0.5));
    rs.EnableAiFlowMonitor("thz-ntn-isac-coexist-traffic");

    // Step the ISAC mode across the sim: comm share shrinks over time.
    const IsacMode seq[] = {COMMUNICATION_ONLY, COMMUNICATION_CENTRIC, JOINT_ISAC,
                            SENSING_CENTRIC, SENSING_ONLY};
    for (int i = 0; i < 5; ++i)
    {
        Simulator::Schedule(Seconds(0.5 + i * (simSeconds / 5.0)), &SetMode, seq[i]);
    }
    Simulator::Schedule(Seconds(0.6), &SlotTick);

    std::printf("# thz-ntn-isac-coexist-traffic (REAL radio, TDM aperture sharing)\n");
    std::printf("#   sim=%.0fs freq=%.0fGHz EIRP=%.1fdBm subBands=%u\n", simSeconds,
                freqGHz, satEirpDbm, numSubBands);
    std::printf("#   ISAC mode steps every %.0fs: COMM_ONLY->COMM_CENTRIC->JOINT->"
                "SENSING_CENTRIC->SENSING_ONLY\n",
                simSeconds / 5.0);
    std::printf("# %5s  %-16s  %9s  %8s  %8s  %9s\n",
                "t_s", "isac_mode", "commShare", "sinr_dB", "tbler", "goodput");

    uint64_t lastRx = 0;
    rs.RegisterPeriodicCallback(
        Seconds(1.0),
        [&rs, &lastRx](Time now) {
            const uint64_t rx = rs.GetUeRxBytes(0);
            const double mbps = (rx - lastRx) * 8.0 / 1e6;
            lastRx = rx;
            std::printf("  %5.1f  %-16s  %4u/10    %8.2f  %8.3f  %9.3f\n",
                        now.GetSeconds(), g_modeName, g_commSlotsPerFrame,
                        rs.GetUeRecentSinrDb(0), rs.GetUeRecentTbler(0), mbps);
        });

    Simulator::Stop(Seconds(simSeconds));
    Simulator::Run();
    rs.Collect();
    rs.WriteHealthReport();

    std::printf("# === summary ===  measured throughput=%.3f Mbps, cell SINR=%.2f dB "
                "(goodput tracks the REAL scheduler's comm share via TDM aperture "
                "gating in the live channel)\n",
                rs.GetRxThroughputMbps(), rs.GetMeanDlSinrDb());

    Simulator::Destroy();
    return 0;
}
