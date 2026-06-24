/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026  Muhammad Uzair
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * ntn-netsimulyzer-official-demo — Path B of the 3D-visualization plan.
 *
 * The toolkit's primary 3D path is the home-grown NtnSceneRecorder (emits a
 * NetSimulyzer-1.0 JSON + Cesium CZML from the same tap). This example shows the
 * SECOND, schema-exact path: driving the OFFICIAL usnistgov NetSimulyzer ns-3
 * module (vendored at contrib/netsimulyzer, v1.0.13) from the toolkit's REAL
 * mobility models — SGP4 satellites + TR 38.811 UEs. The official Orchestrator
 * auto-polls each node's MobilityModel, so once NodeConfiguration is installed
 * the trace updates from genuine orbital dynamics.
 *
 * Coordinate note: the toolkit's satellites report ECEF metres. NetSimulyzer is
 * a local-Cartesian player with no globe, so positions render at planetary scale
 * about an Earth sphere placed at the origin — the established community pattern
 * (satellites are given a large display Scale so they remain visible). Use the
 * NtnSceneRecorder + Cesium path for a true geographic globe.
 *
 * Usage:
 *   ./ns3 run "ntn-netsimulyzer-official-demo --sats=6 --ues=4 --duration=30 \
 *              --out=ntn-official.json"
 */

#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include "ns3/netsimulyzer-module.h"
#include "ns3/network-module.h"
#include "ns3/ntn-real-stack-helper.h"
#include "ns3/ntn-repro-manifest.h"
#include "ns3/ntn-scene-recorder.h"
#include "ns3/ntn-tr38811-mobility-model.h"
#include "ns3/sgp4-mobility-model.h"
#include "ns3/walker-constellation.h"

#include <cmath>
#include <iostream>
#include <limits>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("NtnNetSimulyzerOfficialDemo");

int
main(int argc, char* argv[])
{
    uint32_t sats = 6;
    uint32_t ues = 4;
    double duration = 30.0;
    double altitudeKm = 550.0;
    std::string out = "ntn-official.json";

    CommandLine cmd(__FILE__);
    cmd.AddValue("sats", "Satellites to render", sats);
    cmd.AddValue("ues", "UEs to render", ues);
    cmd.AddValue("duration", "Simulation duration (s)", duration);
    cmd.AddValue("altitude", "Constellation altitude (km)", altitudeKm);
    cmd.AddValue("out", "Official NetSimulyzer JSON output path", out);
    cmd.Parse(argc, argv);

    // ---- REAL SGP4 satellites (ECEF) ----
    ns3::ntncon::WalkerConfig wcfg;
    wcfg.num_planes = 1;
    wcfg.total_sats = std::max<uint32_t>(sats, 1);
    wcfg.altitude_km = altitudeKm;
    wcfg.inclination_deg = 53.0;
    wcfg.epoch_unix_s = 1735689600.0;
    const auto elements = ns3::ntncon::WalkerConstellation::BuildDelta(wcfg);

    NodeContainer satNodes;
    satNodes.Create(sats);
    for (uint32_t s = 0; s < sats; ++s)
    {
        Ptr<ns3::ntncon::Sgp4MobilityModel> m = CreateObject<ns3::ntncon::Sgp4MobilityModel>();
        m->SetElements(elements[s]);
        satNodes.Get(s)->AggregateObject(m);
    }

    // ---- REAL TR 38.811 UEs (ECEF) under the first satellite's sub-point ----
    double subLat, subLon, subAlt;
    satNodes.Get(0)->GetObject<ns3::ntncon::Sgp4MobilityModel>()->GetGeodetic(subLat, subLon, subAlt);
    NodeContainer ueNodes;
    ueNodes.Create(ues);
    NtnTr38811MobilityHelper ueMob(/*seed=*/1);
    auto profile = NtnMobilityScenarios::MixedContinental();
    ueMob.Install(ueNodes, profile, subLat - 0.05, subLat + 0.05, subLon - 0.05, subLon + 0.05);

    // ---- REAL radio (mmwave NR-NTN) so the scene spine is fed MEASURED KPIs ----
    // Same proven backbone as ntn-observability-traffic.cc: SpectrumPhy + MAC +
    // RLC/PDCP + RRC + EPC. Headline KPIs (SINR/TBLER/goodput) come from the PHY
    // RxPacketTraceUe + PacketSink, never closed-form. Build() needs >=1 of each.
    NS_ABORT_MSG_IF(sats < 1 || ues < 1, "real stack needs at least one sat and one UE");
    NtnRealStackHelper rs;
    rs.SetSimTime(Seconds(duration));
    rs.SetRunTag("ntn-netsimulyzer-official-demo");
    rs.SetSatEirpDbm(55.0);
    rs.Build(satNodes, ueNodes);
    rs.InstallTraffic(NtnRealStackHelper::TrafficProfile::EmbbStreaming,
                      Seconds(1.0),
                      Seconds(duration - 0.5));
    rs.EnableAiFlowMonitor("ntn-netsimulyzer-official-demo");

    // ---- Home-grown scene spine (NtnSceneRecorder) as the SECOND sink ----
    // Instantiated directly (not via NtnSceneHelper) because TrackKpiSeries must
    // precede Start(), and Helper::Build() calls Start() internally. Sats carry
    // raw SGP4 ECEF here, so the frame is EcefGlobal (no ENU wrapper).
    Ptr<ntnobs::NtnSceneRecorder> scene = CreateObject<ntnobs::NtnSceneRecorder>();
    scene->SetFrame(ntnobs::NtnSceneRecorder::EcefGlobal);
    std::vector<uint32_t> satSceneIds(sats);
    for (uint32_t s = 0; s < sats; ++s)
    {
        satSceneIds[s] =
            scene->TrackNode(satNodes.Get(s), ntnobs::NtnSceneRecorder::Sat, "sat" + std::to_string(s));
    }
    uint32_t ueSceneId0 = 0;
    for (uint32_t u = 0; u < ues; ++u)
    {
        uint32_t id =
            scene->TrackNode(ueNodes.Get(u), ntnobs::NtnSceneRecorder::Ue, "ue" + std::to_string(u));
        if (u == 0)
        {
            ueSceneId0 = id;
        }
    }
    // MEASURED KPI series for UE-0 (driven from NtnRealStackHelper accessors).
    const uint32_t sinrSeries =
        scene->TrackKpiSeries(ueSceneId0, ntnobs::NtnSceneRecorder::Sinr, "SINR-dl");
    const uint32_t thrSeries =
        scene->TrackKpiSeries(ueSceneId0, ntnobs::NtnSceneRecorder::Throughput, "goodput");
    const uint32_t tblerSeries =
        scene->TrackKpiSeries(ueSceneId0, ntnobs::NtnSceneRecorder::Tbler, "tbler");
    scene->EnableNetSimulyzer(out + ".scene.json");
    scene->EnableCzml(out + ".czml");
    scene->Start();

    // 1 Hz tick: push MEASURED values into the scene + drive a GENUINE handover
    // (serving-satellite reselection by real SGP4 slant range, not np.random).
    uint64_t lastRx = 0;
    int prevBest = -1;
    rs.RegisterPeriodicCallback(
        Seconds(1.0),
        [&rs, scene, &satNodes, sats, ueNodes, sinrSeries, thrSeries, tblerSeries, ueSceneId0,
         &satSceneIds, &lastRx, &prevBest](Time /*now*/) {
            const double sinr = rs.GetUeRecentSinrDb(0);
            if (!std::isnan(sinr))
            {
                scene->RecordKpi(sinrSeries, sinr);
            }
            const uint64_t tot = rs.GetUeRxBytes(0);
            scene->RecordKpi(thrSeries, (tot - lastRx) * 8.0 / 1e6);
            lastRx = tot;
            const double tbler = rs.GetUeRecentTbler(0);
            if (!std::isnan(tbler))
            {
                scene->RecordKpi(tblerSeries, tbler);
            }

            // Best-satellite reselection by measured slant range. Each sat owns a
            // real Sgp4MobilityModel; pick the argmin Euclidean distance to UE-0.
            const Vector u = ueNodes.Get(0)->GetObject<MobilityModel>()->GetPosition();
            int best = 0;
            double bestD = std::numeric_limits<double>::max();
            for (uint32_t s = 0; s < sats; ++s)
            {
                const Vector p = satNodes.Get(s)->GetObject<MobilityModel>()->GetPosition();
                const double dx = p.x - u.x, dy = p.y - u.y, dz = p.z - u.z;
                const double d = std::sqrt(dx * dx + dy * dy + dz * dz);
                if (d < bestD)
                {
                    bestD = d;
                    best = static_cast<int>(s);
                }
            }
            if (prevBest >= 0 && best != prevBest)
            {
                scene->OnHandover(ueSceneId0,
                                  satSceneIds[static_cast<uint32_t>(prevBest)],
                                  satSceneIds[static_cast<uint32_t>(best)]);
            }
            prevBest = best;
        });

    // ---- Earth reference sphere at the ECEF origin (NetSimulyzer has no globe) ----
    NodeContainer earth;
    earth.Create(1);
    MobilityHelper em;
    em.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    em.Install(earth);
    earth.Get(0)->GetObject<MobilityModel>()->SetPosition(Vector(0, 0, 0));

    // ---- Official NetSimulyzer orchestrator + node configurations ----
    auto orchestrator = CreateObject<netsimulyzer::Orchestrator>(out);

    netsimulyzer::NodeConfigurationHelper nc{orchestrator};

    // Earth sphere sized to the real radius so the ECEF nodes sit on its surface.
    nc.Set("Model", netsimulyzer::models::SPHERE_VALUE);
    nc.Set("Scale", DoubleValue(6371000.0));
    nc.Install(earth.Get(0));

    // Satellites: diamonds, enlarged so they read at planetary scale.
    nc.Set("Model", netsimulyzer::models::DIAMOND_VALUE);
    nc.Set("Scale", DoubleValue(150000.0));
    nc.Install(satNodes);

    // UEs: smartphones (also enlarged to be visible from orbit-scale views).
    nc.Set("Model", netsimulyzer::models::SMARTPHONE_VALUE);
    nc.Set("Scale", DoubleValue(80000.0));
    nc.Install(ueNodes);

    std::cout << "\n=== ntn-netsimulyzer-official-demo (Path B: official module) ===\n"
              << "  " << sats << " SGP4 sats + " << ues << " TR 38.811 UEs -> " << out << "\n"
              << "  open with the NetSimulyzer desktop app (Qt6).\n\n";

    Simulator::Stop(Seconds(duration));
    Simulator::Run();
    scene->Stop();
    rs.Collect();

    // ---- Reproducibility manifest (roadmap T9): exercise BOTH WriteJson + LoadJson ----
    const std::string manifestPath = out + ".manifest.json";
    ntnobs::NtnReproManifest manifest;
    manifest.SetScenarioName("ntn-netsimulyzer-official-demo")
        .SetScenarioDuration(duration)
        .SetRng(RngSeedManager::GetSeed(), RngSeedManager::GetRun())
        .SetConstellation("walker-delta", wcfg.num_planes, wcfg.total_sats, altitudeKm, 53.0)
        .SetCliArgv(argc, argv)
        .AddExtra("mean_dl_sinr_db", std::to_string(rs.GetMeanDlSinrDb()));
    const bool wrote = manifest.WriteJson(manifestPath);

    ntnobs::NtnReproManifest roundtrip;
    const bool loaded = ntnobs::NtnReproManifest::LoadJson(manifestPath, roundtrip);
    const bool manifestOk = wrote && loaded &&
                            roundtrip.GetScenarioName() == "ntn-netsimulyzer-official-demo" &&
                            roundtrip.GetScenarioDuration() == duration;

    std::cout << "\n  === measured KPI summary (real plane) ===\n"
              << "    mean DL SINR : " << rs.GetMeanDlSinrDb() << " dB\n"
              << "    mean DL TBLER: " << rs.GetMeanDlTbler() << "\n"
              << "    DL goodput   : " << rs.GetRxThroughputMbps() << " Mbps\n"
              << "    scene events : " << scene->GetEventCount() << "\n"
              << "    manifest round-trip: " << (manifestOk ? "PASS" : "FAIL") << "\n";
    std::cout << "  wrote official NetSimulyzer trace: " << out << "\n"
              << "  wrote scene spine: " << out << ".scene.json + " << out << ".czml\n"
              << "  wrote manifest:    " << manifestPath << "\n";

    Simulator::Destroy();
    return 0;
}
