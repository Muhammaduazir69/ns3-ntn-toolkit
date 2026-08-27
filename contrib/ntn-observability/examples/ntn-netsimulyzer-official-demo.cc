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
#include <ctime>
#include <iostream>
#include <limits>
#include <map>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("NtnNetSimulyzerOfficialDemo");

int
main(int argc, char* argv[])
{
    uint32_t sats = 6;
    uint32_t ues = 4;
    double duration = 30.0;
    double altitudeKm = 550.0;
    std::string radio = "nr"; // radio backend: "nr" (5G-LENA FR1) | "mmwave" (FR2)
    std::string out = "ntn-official.json";

    CommandLine cmd(__FILE__);
    cmd.AddValue("sats", "Satellites to render", sats);
    cmd.AddValue("ues", "UEs to render", ues);
    cmd.AddValue("duration", "Simulation duration (s)", duration);
    cmd.AddValue("altitude", "Constellation altitude (km)", altitudeKm);
    cmd.AddValue("radio", "Radio backend: nr (FR1) or mmwave", radio);
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

    // ---- REAL radio (NR-NTN) so the scene spine is fed MEASURED KPIs ----
    // Same proven backbone as ntn-observability-traffic.cc: SpectrumPhy + MAC +
    // RLC/PDCP + RRC + EPC. Headline KPIs (SINR/TBLER/goodput) come from the PHY
    // RxPacketTraceUe + PacketSink, never closed-form. Build() needs >=1 of each.
    NS_ABORT_MSG_IF(sats < 1 || ues < 1, "real stack needs at least one sat and one UE");
    NtnRealStackHelper rs;
    rs.SetRadioBackend(radio == "mmwave" ? NtnRealStackHelper::RadioBackend::Mmwave
                                         : NtnRealStackHelper::RadioBackend::Nr);
    if (radio != "mmwave")
    {
        rs.SetNumerology(1); // FR1 30 kHz SCS
    }
    rs.SetSimTime(Seconds(duration));
    rs.SetRunTag("ntn-netsimulyzer-official-demo");
    // nr's Friis LEO link needs ~70 dBm for a healthy SINR; mmwave keeps 55 dBm.
    // NT-02: declared as CONDUCTED power at the array input. This carrier has
    // no TR 38.821 Set-1 reference in the toolkit, so the EIRP health gate
    // reports "not asserted" rather than certifying an uncalibrated budget.
    rs.SetSatConductedPowerDbm(radio == "mmwave" ? 55.0 : 70.0);
    // Enable the REAL NR inter-cell handover path (A3-RSRP + X2) so the scene's
    // handover events, if any, come from a genuine RRC serving-cell change
    // (TS 38.300: reconfiguration-with-sync) — not a geometric guess. Requires
    // the nr backend and >= 2 gNB satellites; with a single sat there is nothing
    // to hand over TO, so we leave it off and no handover will (or should) show.
    const bool realHandoverPossible = (radio != "mmwave") && (sats >= 2);
    if (realHandoverPossible)
    {
        rs.SetHandover(true);
    }
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
    // Anchor the CZML clock/lighting to the constellation's TLE epoch (not the
    // hardcoded 2026-01-01 default) so Cesium's sun position matches the orbit.
    scene->SetSceneEpochUnix(wcfg.epoch_unix_s);
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
    // The Euclidean nearest-satellite argmin is a GEOMETRIC candidate only — it
    // is NOT what the RRC layer serves. Record it as its own clearly-named
    // series so it is never mistaken for an executed handover.
    const uint32_t geoCandSeries =
        scene->TrackKpiSeries(ueSceneId0, ntnobs::NtnSceneRecorder::Tbler, "best-geo-candidate-idx");
    scene->EnableNetSimulyzer(out + ".scene.json");
    scene->EnableCzml(out + ".czml");

    // Map each gNB's real cell id -> scene sat id, so a real serving-cell change
    // can be rendered against the right satellites.
    std::map<uint16_t, uint32_t> cellToScene;
    for (uint32_t s = 0; s < sats; ++s)
    {
        const uint16_t cid = rs.GetGnbCellId(s);
        if (cid != 0)
        {
            cellToScene[cid] = satSceneIds[s];
        }
    }

    // OBS-06: declare the serving beam BEFORE Start(), so the scene has a
    // communication link in it at all.
    //
    // This example tracked seven nodes and four KPI series and never called
    // TrackBeam, so its CZML sink held node positions and handover arcs and not
    // one link-<a>-<b> polyline: a scene of satellites and a terminal with
    // nothing drawn between them. Everything needed was already here, the
    // cellToScene map and the real GetUeServingCellId below.
    //
    // Declared from the REAL serving cell, not from the nearest satellite. A
    // geometric argmin would draw a link the stack is not using, which is the
    // fabricated-beam habit OBS-01 exists to stop.
    const uint16_t servingAtStart = rs.GetUeServingCellId(0);
    auto itStart = cellToScene.find(servingAtStart);
    if (itStart != cellToScene.end())
    {
        scene->TrackBeam(itStart->second, ueSceneId0);
    }
    else
    {
        NS_LOG_UNCOND("[obs-06] no RRC serving cell at t0 (cell id "
                      << servingAtStart << "); the serving beam will be declared at the first "
                      << "handover instead, and until then the scene correctly shows no link");
    }

    scene->Start();

    // 1 Hz tick: push MEASURED values into the scene. A handover is emitted ONLY
    // when the UE's REAL RRC serving cell changes (GetUeServingCellId tracks the
    // actual A3/X2 handover, unlike a geometric nearest-sat argmin). If the stack
    // never hands over — the common case for a single short pass — no handover is
    // shown, because none executed. The geometric nearest-sat is recorded
    // separately as "best-geo-candidate-idx", never as a handover.
    uint64_t lastRx = 0;
    uint16_t prevCell = servingAtStart;
    rs.RegisterPeriodicCallback(
        Seconds(1.0),
        [&rs, scene, &satNodes, sats, ueNodes, sinrSeries, thrSeries, tblerSeries, geoCandSeries,
         ueSceneId0, &cellToScene, &lastRx, &prevCell](Time /*now*/) {
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

            // GEOMETRIC candidate only: argmin Euclidean distance to UE-0. This
            // is decision-support telemetry, NOT a handover — it is recorded as
            // its own series and never triggers scene->OnHandover.
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
            scene->RecordKpi(geoCandSeries, static_cast<double>(best));

            // REAL handover: reflect an actual RRC serving-cell change only.
            const uint16_t cell = rs.GetUeServingCellId(0);
            if (cell != 0 && prevCell != 0 && cell != prevCell)
            {
                auto itFrom = cellToScene.find(prevCell);
                auto itTo = cellToScene.find(cell);
                if (itFrom != cellToScene.end() && itTo != cellToScene.end())
                {
                    scene->OnHandover(ueSceneId0, itFrom->second, itTo->second);
                }
            }
            if (cell != 0)
            {
                prevCell = cell;
            }
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
    // OBS-07: SetConstellation's third parameter is satsPerPlane, and this
    // passed wcfg.total_sats, which is only the same number because num_planes
    // is 1. Divide so the field means what it is named.
    const uint32_t satsPerPlane =
        (wcfg.num_planes > 0) ? (wcfg.total_sats / wcfg.num_planes) : wcfg.total_sats;
    // OBS-07: the epoch was sitting in wcfg all along and the manifest recorded
    // an empty string for it.
    const std::time_t epochT = static_cast<std::time_t>(wcfg.epoch_unix_s);
    std::tm epochTm{};
    gmtime_r(&epochT, &epochTm);
    char epochBuf[32];
    std::strftime(epochBuf, sizeof(epochBuf), "%Y-%m-%dT%H:%M:%SZ", &epochTm);

    manifest.SetScenarioName("ntn-netsimulyzer-official-demo")
        .SetScenarioDuration(duration)
        .SetRng(RngSeedManager::GetSeed(), RngSeedManager::GetRun())
        .SetTleEpoch(epochBuf)
        .SetConstellation("walker-delta", wcfg.num_planes, satsPerPlane, altitudeKm, 53.0)
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
