/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only
//
// oran-ntn-emergency-communication — the paper's post-disaster use case
// (Deng 2026 Sec. V-A; adoption plan WS6), end to end on a REAL LEO cell.
//
// Timeline (everything measured, in-band):
//   t<20 s   normal operation: four ground UEs carry eMBB on S-NSSAI 1.
//   t=20 s   DISASTER: the terrestrial gateway fails. OranNtnRoleSwitch
//            elevates the satellite to a full on-board gNB (failure-injected
//            switch with a real 500 ms service-interruption window) and the
//            feeder re-points to a surviving gateway (payload option applied
//            to the live backhaul).
//   t=22 s   emergency responders arrive: two URLLC flows on the DEDICATED
//            emergency slice (S-NSSAI SST=5) join via InstallOranFlow; the
//            SMO protects them RIC-style by throttling the eMBB quota (live
//            offered-rate actuation, as in the cross-domain example).
// Measured recovery timeline: disaster -> first emergency byte delivered,
// per-slice OWD/loss/throughput before and after, all from NtnOranSink and
// the KPM monitor on the real radio.
//
// Quick test:  --simSeconds=60
#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/ntn-oran-ai-flow-monitor.h"
#include "ns3/ntn-oran-application.h"
#include "ns3/ntn-oran-sink.h"
#include "ns3/ntn-platform-spec.h"
#include "ns3/ntn-real-stack-helper.h"
#include "ns3/ntn-tr38811-mobility-model.h"
#include "ns3/sgp4-mobility-model.h"
#include "ns3/walker-constellation.h"

#include <cstdio>
#include <string>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("OranNtnEmergencyCommunication");

int
main(int argc, char* argv[])
{
    double simSeconds = 60.0;
    double disasterAt = 20.0;
    std::string radio = "nr"; // radio backend: nr (FR1) or mmwave
    std::string outputDir = "oran-ntn-emergency-output";

    CommandLine cmd(__FILE__);
    cmd.AddValue("simSeconds", "Simulation duration (s)", simSeconds);
    cmd.AddValue("disasterAt", "Disaster time (s)", disasterAt);
    cmd.AddValue("radio", "Radio backend: nr (FR1) or mmwave", radio);
    cmd.AddValue("outputDir", "Output directory", outputDir);
    cmd.Parse(argc, argv);

    // nr's Friis LEO link needs ~70 dBm for a healthy SINR; mmwave keeps 60.

    std::printf("# oran-ntn-emergency-communication (REAL cell, disaster at t=%.0fs)\n",
                disasterAt);

    NodeContainer satNodes;
    satNodes.Create(1);
    NodeContainer ueNodes;
    ueNodes.Create(4);

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
    Ptr<ListPositionAllocator> pos = CreateObject<ListPositionAllocator>();
    pos->Add(Vector(0.0, 0.0, 1.5));
    pos->Add(Vector(1500.0, 0.0, 1.5));
    pos->Add(Vector(0.0, 1500.0, 1.5)); // responder 1 (joins after disaster)
    pos->Add(Vector(-1500.0, 0.0, 1.5)); // responder 2
    mob.SetPositionAllocator(pos);
    mob.Install(ueNodes);
    // Surviving feeder gateway 200 km away.
    NodeContainer gwNodes;
    gwNodes.Create(1);
    Ptr<ListPositionAllocator> gwPos = CreateObject<ListPositionAllocator>();
    gwPos->Add(Vector(200e3, 0.0, 10.0));
    mob.SetPositionAllocator(gwPos);
    mob.Install(gwNodes);

    NtnRealStackHelper rs;
    rs.SetRadioBackend(radio == "mmwave" ? NtnRealStackHelper::RadioBackend::Mmwave
                                         : NtnRealStackHelper::RadioBackend::Nr);
    if (radio != "mmwave")
    {
        rs.SetNumerology(1); // FR1 30 kHz SCS
    }
    rs.SetSimTime(Seconds(simSeconds));
    rs.SetOutputDir(outputDir);
    rs.SetRunTag("oran-ntn-emergency");
    rs.SetCarrierFrequencyHz(2.0e9);
    // NT-02: TR 38.821 Table 6.1.1.1-1 Set-1 downlink EIRP density for the
    // S-band LEO reference payload. Declared as a DENSITY so the helper
    // back-computes conducted power against the array gain instead of the
    // antenna being counted twice.
    rs.SetSatEirpDensityDbwMhz(
        NtnRealStackHelper::kTr38821Set1SBandEirpDensityDbwMhz);
    rs.SetPayloadOption(NtnRealStackHelper::PayloadOption::RegenerativeRu);
    rs.Build(satNodes, ueNodes);

    // Normal operation: two eMBB UEs on the public slice (SST 1).
    const Time stop = Seconds(simSeconds - 0.5);
    ApplicationContainer embb0 = rs.InstallOranFlow(
        0, 2, 1, 0x000001, NtnOranApplication::CBR_SATURATING, Seconds(1.0), stop);
    ApplicationContainer embb1 = rs.InstallOranFlow(
        1, 2, 1, 0x000001, NtnOranApplication::CBR_SATURATING, Seconds(1.0), stop);
    // Emergency slice (SST 5): responders join AFTER the disaster.
    ApplicationContainer emrg0 = rs.InstallOranFlow(
        2, 82, 5, 0x000001, NtnOranApplication::URLLC_PERIODIC,
        Seconds(disasterAt + 2.0), stop);
    ApplicationContainer emrg1 = rs.InstallOranFlow(
        3, 82, 5, 0x000001, NtnOranApplication::URLLC_PERIODIC,
        Seconds(disasterAt + 2.0), stop);

    Ptr<NtnOranAiFlowMonitor> kpm = rs.EnableOranFlowMonitor();

    // Role switch: the satellite elevates to full gNB when the ground O-DU
    // dies; the applier re-points the feeder to the surviving gateway.
    Ptr<OranNtnRoleSwitch> rsw = CreateObject<OranNtnRoleSwitch>();
    rsw->SetInterruptionWindow(MilliSeconds(500));
    rsw->RegisterPlatform("leo-sat", OranNtnRoleSwitch::Role::Ru,
                          [&rs, &satEnu, &gwNodes](OranNtnRoleSwitch::Role r) {
                              if (r == OranNtnRoleSwitch::Role::FullGnb)
                              {
                                  rs.SetPayloadOption(
                                      NtnRealStackHelper::PayloadOption::FullGnb);
                                  rs.SetFeederGeometry(
                                      satEnu, gwNodes.Get(0)->GetObject<MobilityModel>());
                              }
                          });

    Ptr<NtnOranApplication> embbApp0 = DynamicCast<NtnOranApplication>(embb0.Get(0));
    Ptr<NtnOranApplication> embbApp1 = DynamicCast<NtnOranApplication>(embb1.Get(0));
    Simulator::Schedule(Seconds(disasterAt), [&rsw, embbApp0, embbApp1] {
        std::printf("  [%.1fs] DISASTER: ground O-DU lost -> elevating satellite "
                    "to full gNB (500 ms interruption)\n",
                    Simulator::Now().GetSeconds());
        rsw->TriggerSwitch("leo-sat", OranNtnRoleSwitch::Role::FullGnb, "o-du-lost");
        // SMO emergency policy: throttle the public eMBB slice to protect
        // the emergency slice (live offered-rate actuation).
        embbApp0->SetAttribute("DataRate", DataRateValue(DataRate("1Mbps")));
        embbApp1->SetAttribute("DataRate", DataRateValue(DataRate("1Mbps")));
    });

    // Recovery timeline: first emergency byte delivered after the disaster.
    Ptr<NtnOranSink> emrgSink0 = DynamicCast<NtnOranSink>(emrg0.Get(1));
    Ptr<NtnOranSink> emrgSink1 = DynamicCast<NtnOranSink>(emrg1.Get(1));
    double firstEmergencyByteAt = -1.0;
    rs.RegisterPeriodicCallback(Seconds(0.1), [&](Time now) {
        if (firstEmergencyByteAt < 0.0 &&
            (emrgSink0->GetTotalRx() > 0 || emrgSink1->GetTotalRx() > 0))
        {
            firstEmergencyByteAt = now.GetSeconds();
        }
    });
    rs.RegisterPeriodicCallback(Seconds(5.0), [&](Time now) {
        std::printf("  t=%5.1f  embb0=%.2f Mbps  emergency rx=%lu B  role=%s\n",
                    now.GetSeconds(), rs.GetUeRxBytes(0) * 8.0 / now.GetSeconds() / 1e6,
                    static_cast<unsigned long>(emrgSink0->GetTotalRx() +
                                               emrgSink1->GetTotalRx()),
                    rsw->GetRole("leo-sat") == OranNtnRoleSwitch::Role::FullGnb
                        ? "full-gnb"
                        : "ru");
    });

    Simulator::Stop(Seconds(simSeconds));
    Simulator::Run();
    rs.Collect();
    rs.WriteHealthReport();
    kpm->WriteCsv(outputDir + "/kpm_series.csv");

    std::printf("# === recovery timeline (measured) ===\n");
    std::printf("#   disaster t=%.1f s -> first emergency byte t=%.2f s "
                "(recovery %.2f s incl. 0.5 s switch window)\n",
                disasterAt, firstEmergencyByteAt,
                firstEmergencyByteAt - disasterAt);
    for (int i = 0; i < 2; ++i)
    {
        Ptr<NtnOranSink> s = (i == 0) ? emrgSink0 : emrgSink1;
        for (const auto& [k, fs] : s->GetFlowStats())
        {
            std::printf("#   emergency-%d 5QI=%u SST=%u owd=%.2f ms loss=%.4f "
                        "thr=%.3f Mbps\n",
                        i, fs.fiveQi, fs.sst, fs.MeanDelayMs(), fs.LossRatio(),
                        fs.ThroughputMbps());
        }
    }
    std::printf("# === summary ===  roleSwitches=%zu cellSINR=%.2f dB thr=%.3f Mbps "
                "owd=%.2f ms\n",
                rsw->GetEvents().size(), rs.GetMeanDlSinrDb(), rs.GetRxThroughputMbps(),
                rs.GetMeanDelayMs());
    Simulator::Destroy();
    return 0;
}
