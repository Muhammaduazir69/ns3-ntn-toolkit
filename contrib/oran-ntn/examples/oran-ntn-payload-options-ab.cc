/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only
//
// oran-ntn-payload-options-ab — the four satellite payload architectures of
// Deng 2026 Sec. II-B2 measured on ONE real scenario (adoption plan WS4).
//
// A REAL mmwave NR NTN cell serves a ground UE from an SGP4 LEO satellite;
// the EPC backhaul (the feeder/transport leg behind the satellite) is driven
// LIVE from the real feeder slant range per the selected payload option
// (NtnRealStackHelper::SetFeederGeometry):
//   transparent: user plane rides the RF feeder leg too (2 x slant/c)
//   ru:          O-RU on sat, Open-FH over the feeder (slant/c + 0.25 ms)
//   rudu:        O-DU on sat, F1 midhaul over the feeder (slant/c + 0.15 ms)
//   fullgnb:     full gNB on sat, GTP backhaul (slant/c + 0.05 ms)
// The one-way delay is MEASURED by NtnOranSink from in-band timestamps that
// crossed radio + GTP + backhaul; the FH-split feasibility of the measured
// feeder delay is checked against NtnFhSplitModel (the paper's argument that
// LEO latencies rule out lower-PHY splits).
//
// Run all four:  for p in transparent ru rudu fullgnb; do
//                  ./ns3 run "oran-ntn-payload-options-ab --payload=$p"; done
#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/ntn-real-stack-helper.h"
#include "ns3/ntn-tr38811-mobility-model.h"
#include "ns3/oran-ntn-fh-split.h"
#include "ns3/sgp4-mobility-model.h"
#include "ns3/walker-constellation.h"

#include <cstdio>
#include <string>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("OranNtnPayloadOptionsAb");

int
main(int argc, char* argv[])
{
    double simSeconds = 30.0;
    double leoAltKm = 550.0;
    std::string payload = "fullgnb";
    std::string outputDir = "oran-ntn-payload-options-ab-output";

    CommandLine cmd(__FILE__);
    cmd.AddValue("simSeconds", "Simulation duration (s)", simSeconds);
    cmd.AddValue("payload", "Payload: transparent|ru|rudu|fullgnb", payload);
    cmd.AddValue("outputDir", "Output directory", outputDir);
    cmd.Parse(argc, argv);

    using PO = NtnRealStackHelper::PayloadOption;
    PO opt = PO::FullGnb;
    if (payload == "transparent")
    {
        opt = PO::Transparent;
    }
    else if (payload == "ru")
    {
        opt = PO::RegenerativeRu;
    }
    else if (payload == "rudu")
    {
        opt = PO::RegenerativeRuDu;
    }

    std::printf("# oran-ntn-payload-options-ab (REAL cell, payload=%s)\n",
                payload.c_str());

    NodeContainer satNodes;
    satNodes.Create(1);
    NodeContainer ueNodes;
    ueNodes.Create(1);

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
    mob.SetPositionAllocator(pos);
    mob.Install(ueNodes);
    // The feeder gateway sits 100 km from the UE (same ENU frame).
    NodeContainer gwNodes;
    gwNodes.Create(1);
    Ptr<ListPositionAllocator> gwPos = CreateObject<ListPositionAllocator>();
    gwPos->Add(Vector(100e3, 0.0, 10.0));
    mob.SetPositionAllocator(gwPos);
    mob.Install(gwNodes);

    NtnRealStackHelper rs;
    rs.SetSimTime(Seconds(simSeconds));
    rs.SetOutputDir(outputDir);
    rs.SetRunTag("oran-ntn-payload-options-ab-" + payload);
    rs.SetCarrierFrequencyHz(2.0e9);
    rs.SetSatEirpDbm(60.0);
    rs.SetPayloadOption(opt);
    rs.Build(satNodes, ueNodes);
    rs.SetFeederGeometry(satEnu, gwNodes.Get(0)->GetObject<MobilityModel>());
    rs.InstallTraffic(NtnRealStackHelper::TrafficProfile::EmbbStreaming,
                      Seconds(1.0), Seconds(simSeconds - 0.5));

    rs.RegisterPeriodicCallback(Seconds(5.0), [&rs, &satEnu, &gwNodes](Time now) {
        const double slantM =
            satEnu->GetDistanceFrom(gwNodes.Get(0)->GetObject<MobilityModel>());
        std::printf("  t=%5.1f  feeder_slant=%7.1f km  payload_extra=%6.3f ms  "
                    "owd_so_far=%6.2f ms\n",
                    now.GetSeconds(), slantM / 1e3,
                    rs.ComputePayloadExtraDelay(slantM).GetSeconds() * 1e3,
                    rs.GetMeanDelayMs());
    });

    Simulator::Stop(Seconds(simSeconds));
    Simulator::Run();
    rs.Collect();
    rs.WriteHealthReport();

    // FH-split feasibility at the MEASURED feeder one-way delay.
    const double endSlantM =
        satEnu->GetDistanceFrom(gwNodes.Get(0)->GetObject<MobilityModel>());
    const Time fhOneWay = Seconds(endSlantM / 299792458.0);
    bool ok = false;
    const auto best = NtnFhSplitModel::ChooseBestSplit(fhOneWay, 5.0, 100.0, ok);
    std::printf("# FH check: feeder one-way %.2f ms -> deepest feasible split: %s%s\n",
                fhOneWay.GetSeconds() * 1e3, NtnFhSplitModel::GetSpec(best).name.c_str(),
                ok ? "" : " (NONE feasible)");

    std::printf("# === summary ===  payload=%s  MEASURED owd=%.2f ms jitter=%.3f ms "
                "loss=%.4f thr=%.3f Mbps SINR=%.2f dB\n",
                payload.c_str(), rs.GetMeanDelayMs(), rs.GetMeanJitterMs(),
                rs.GetAppLossRatio(), rs.GetRxThroughputMbps(), rs.GetMeanDlSinrDb());
    Simulator::Destroy();
    return 0;
}
