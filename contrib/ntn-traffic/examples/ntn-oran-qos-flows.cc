/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only
//
// ntn-oran-qos-flows — the WS1 flagship of the AI-Native ORAN-NTN adoption
// plan. One REAL LEO cell (NtnRealStackHelper: mmwave SpectrumPhy + MAC +
// RLC/PDCP + RRC + EPC, SGP4 satellite) carries four 3GPP QoS flows side by
// side, each an NtnOranApplication with its own 5QI / S-NSSAI in REAL packet
// bytes:
//   UE0  5QI 1   conversational voice    (S-NSSAI 1/0x000001)
//   UE1  5QI 2   eMBB video frame bursts (S-NSSAI 1/0x000002)
//   UE2  5QI 82  URLLC periodic commands (S-NSSAI 2/0x000001)
//   UE3  5QI 9   mMTC NB-IoT reports     (S-NSSAI 3/0x000001)
// plus the satellite's own C&C telemetry (5QI 69) flowing uplink to an SMO
// endpoint as real packets. Per-flow one-way delay / jitter / loss are
// measured by NtnOranSink from the in-band header — through the GTP tunnel,
// across the real radio, with real orbital geometry. Nothing closed-form.
//
// Quick test:  --simSeconds=40

#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/ntn-command-and-control-app.h"
#include "ns3/ntn-oran-ai-flow-monitor.h"
#include "ns3/ntn-oran-application.h"
#include "ns3/ntn-oran-sink.h"
#include "ns3/ntn-real-stack-helper.h"
#include "ns3/ntn-tr38811-mobility-model.h"
#include "ns3/point-to-point-module.h"
#include "ns3/sgp4-mobility-model.h"
#include "ns3/walker-constellation.h"

#include <cstdio>
#include <string>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("NtnOranQosFlows");

int
main(int argc, char* argv[])
{
    double simSeconds = 40.0;
    double leoAltKm = 550.0;
    double freqGHz = 2.0; // S-band NR-NTN FR1
    double satEirpDbm = 60.0;
    std::string outputDir = "ntn-oran-qos-flows-output";

    CommandLine cmd(__FILE__);
    cmd.AddValue("simSeconds", "Simulation duration (s)", simSeconds);
    cmd.AddValue("leoAltKm", "Satellite altitude (km)", leoAltKm);
    cmd.AddValue("freqGHz", "Carrier frequency (GHz)", freqGHz);
    cmd.AddValue("satEirpDbm", "Satellite EIRP (dBm)", satEirpDbm);
    cmd.AddValue("outputDir", "Output directory", outputDir);
    cmd.Parse(argc, argv);

    std::printf("# ntn-oran-qos-flows (REAL radio, per-5QI ORAN-NTN flows)\n");
    std::printf("#   sim=%.0fs alt=%.0fkm fc=%.1fGHz EIRP=%.1fdBm\n",
                simSeconds, leoAltKm, freqGHz, satEirpDbm);

    NodeContainer satNodes;
    satNodes.Create(1);
    NodeContainer ueNodes;
    ueNodes.Create(4);

    // Real NTN mobility (mobility mandate, audit issue 11): SGP4 Walker
    // serving satellite + TR 38.811 UE classes under its t=0 sub-point
    // (UE+sat share the ECEF frame; the pass is genuine) — the exact pattern
    // proven in ntn-real-stack-smoke.
    ns3::ntncon::WalkerConfig wcfg;
    wcfg.num_planes = 1;
    wcfg.total_sats = 80;
    wcfg.altitude_km = leoAltKm;
    wcfg.inclination_deg = 53.0;
    wcfg.epoch_unix_s = 1735689600.0;
    const auto elements = ns3::ntncon::WalkerConstellation::BuildDelta(wcfg);
    Ptr<ns3::ntncon::Sgp4MobilityModel> satSgp4 =
        CreateObject<ns3::ntncon::Sgp4MobilityModel>();
    satSgp4->SetElements(elements[0]);
    satNodes.Get(0)->AggregateObject(satSgp4);
    double subLat, subLon, subAlt;
    satSgp4->GetGeodetic(subLat, subLon, subAlt);
    NtnTr38811MobilityHelper ueMobility(1);
    auto mobProfile = NtnMobilityScenarios::MixedContinental();
    ueMobility.Install(ueNodes, mobProfile, subLat - 0.03, subLat + 0.03,
                       subLon - 0.03, subLon + 0.03);

    NtnRealStackHelper rs;
    rs.SetSimTime(Seconds(simSeconds));
    rs.SetOutputDir(outputDir);
    rs.SetRunTag("ntn-oran-qos-flows");
    rs.SetCarrierFrequencyHz(freqGHz * 1e9);
    rs.SetSatEirpDbm(satEirpDbm);
    rs.Build(satNodes, ueNodes);

    // Four QoS flows with distinct 5QI / S-NSSAI identity in real bytes.
    const Time start = Seconds(1.0);
    const Time stop = Seconds(simSeconds - 0.5);
    ApplicationContainer voice = rs.InstallOranFlow(
        0, 1, 1, 0x000001, NtnOranApplication::CONVERSATIONAL_VOICE, start, stop);
    ApplicationContainer video = rs.InstallOranFlow(
        1, 2, 1, 0x000002, NtnOranApplication::EMBB_VIDEO, start, stop);
    ApplicationContainer urllc = rs.InstallOranFlow(
        2, 82, 2, 0x000001, NtnOranApplication::URLLC_PERIODIC, start, stop);
    ApplicationContainer mmtc = rs.InstallOranFlow(
        3, 9, 3, 0x000001, NtnOranApplication::MMTC_PERIODIC, start, stop);

    // Platform C&C telemetry: the app runs ON the satellite node (so it reads
    // the satellite's REAL SGP4 mobility) and reports to the SMO on the remote
    // host over a dedicated feeder P2P link with the physical zenith delay.
    InternetStackHelper internet;
    internet.Install(satNodes);
    PointToPointHelper feeder;
    feeder.SetDeviceAttribute("DataRate", StringValue("10Mbps"));
    // One-way feeder delay at zenith: alt/c.
    feeder.SetChannelAttribute("Delay",
                               TimeValue(Seconds(leoAltKm * 1000.0 / 299792458.0)));
    NetDeviceContainer feederDevs = feeder.Install(satNodes.Get(0), rs.GetRemoteHost());
    Ipv4AddressHelper feederIp;
    feederIp.SetBase("11.0.0.0", "255.255.255.0");
    Ipv4InterfaceContainer feederIf = feederIp.Assign(feederDevs);

    const uint16_t smoPort = 9100;
    Ptr<NtnOranSink> smo = CreateObject<NtnOranSink>();
    smo->SetAttribute("Local",
                      AddressValue(InetSocketAddress(Ipv4Address::GetAny(), smoPort)));
    rs.GetRemoteHost()->AddApplication(smo);
    smo->SetStartTime(Seconds(0.0));

    Ptr<NtnCommandAndControlApp> cnc = CreateObject<NtnCommandAndControlApp>();
    cnc->SetRemote(InetSocketAddress(feederIf.GetAddress(1), smoPort));
    cnc->SetAttribute("Period", TimeValue(Seconds(1.0)));
    cnc->SetAttribute("SrcId", UintegerValue(1));
    cnc->SetAttribute("BatteryCapacityWh", DoubleValue(2500.0)); // smallsat bus
    cnc->SetAttribute("PowerDrawW", DoubleValue(1200.0));
    satNodes.Get(0)->AddApplication(cnc);
    cnc->SetStartTime(Seconds(0.5));
    cnc->SetStopTime(Seconds(simSeconds - 0.5));

    // WS2: AI-native KPM measurement layer over all four flows — TS 28.552
    // metric names, AI feature windows, EWMA anomaly events. The canonical
    // one-call wiring also auto-exports
    // ntn-oran-qos-flows_kpm_series.{csv,lp} at end of simulation.
    rs.EnableAiFlowMonitor("ntn-oran-qos-flows");
    Ptr<NtnOranAiFlowMonitor> kpm = rs.GetAiFlowMonitor();
    kpm->RegisterAnomalyCallback([](const NtnOranAiFlowMonitor::AnomalyEvent& ev) {
        std::printf("  [anomaly] t=%.1f flow=%u 5qi=%u %s=%.4f z=%.1f\n",
                    ev.time.GetSeconds(), ev.flowId, ev.key.fiveQi,
                    ev.metric.c_str(), ev.value, ev.zScore);
    });

    std::printf("# %5s  %10s  %10s  %10s  %10s  %8s\n",
                "t_s", "voice_ms", "video_ms", "urllc_ms", "mmtc_ms", "battery");
    Ptr<NtnOranSink> sinks[4] = {DynamicCast<NtnOranSink>(voice.Get(1)),
                                 DynamicCast<NtnOranSink>(video.Get(1)),
                                 DynamicCast<NtnOranSink>(urllc.Get(1)),
                                 DynamicCast<NtnOranSink>(mmtc.Get(1))};
    rs.RegisterPeriodicCallback(Seconds(5.0), [&](Time now) {
        NtnCncTelemetry t;
        const bool haveTel = smo->GetLatestTelemetry(1, t);
        std::printf("  %5.1f  %10.2f  %10.2f  %10.2f  %10.2f  %7.1f%%\n",
                    now.GetSeconds(), sinks[0]->GetMeanDelayMs(),
                    sinks[1]->GetMeanDelayMs(), sinks[2]->GetMeanDelayMs(),
                    sinks[3]->GetMeanDelayMs(),
                    haveTel ? t.batteryFraction * 100.0 : 100.0);
    });

    Simulator::Stop(Seconds(simSeconds));
    Simulator::Run();
    rs.Collect();
    rs.WriteHealthReport();
    // CSV + Influx LP series export automatically at Simulator::Destroy()
    // (EnableAiFlowMonitor); only the FlowMonitor XML is written manually.
    kpm->SerializeToXmlFile(outputDir + "/oran_flow_monitor.xml");
    std::printf("# KPM: %zu flows, anomalies=%zu, series -> "
                "ntn-oran-qos-flows_kpm_series.{csv,lp}\n",
                kpm->GetKpmSeries().size(), kpm->GetAnomalies().size());

    std::printf("# === per-flow measured KPIs (in-band, through GTP + radio) ===\n");
    std::printf("# %-8s %5s %9s %10s %10s %9s %9s %10s\n",
                "flow", "5QI", "snssai", "rx_pkts", "owd_ms", "jit_ms", "loss",
                "thr_mbps");
    const char* names[4] = {"voice", "video", "urllc", "mmtc"};
    for (int i = 0; i < 4; ++i)
    {
        for (const auto& [key, fs] : sinks[i]->GetFlowStats())
        {
            std::printf("# %-8s %5u %3u/0x%06X %10lu %10.2f %9.3f %9.4f %10.3f\n",
                        names[i], fs.fiveQi, fs.sst, fs.sd,
                        static_cast<unsigned long>(fs.rxPackets), fs.MeanDelayMs(),
                        fs.jitterMs, fs.LossRatio(), fs.ThroughputMbps());
        }
    }
    NtnCncTelemetry t;
    if (smo->GetLatestTelemetry(1, t))
    {
        std::printf("# C&C: last telemetry pos=(%.0f,%.0f,%.0f)km vel=%.2fkm/s "
                    "yaw=%.1fdeg battery=%.1f%% (REAL mobility, real packets)\n",
                    t.posX / 1e3, t.posY / 1e3, t.posZ / 1e3,
                    std::sqrt(t.velX * t.velX + t.velY * t.velY + t.velZ * t.velZ) / 1e3,
                    t.yawDeg, t.batteryFraction * 100.0);
    }
    std::printf("# === summary ===  cell SINR=%.2f dB TBLER=%.4f thr=%.3f Mbps "
                "owd=%.2f ms jitter=%.3f ms loss=%.4f\n",
                rs.GetMeanDlSinrDb(), rs.GetMeanDlTbler(), rs.GetRxThroughputMbps(),
                rs.GetMeanDelayMs(), rs.GetMeanJitterMs(), rs.GetAppLossRatio());
    Simulator::Destroy();
    return 0;
}
