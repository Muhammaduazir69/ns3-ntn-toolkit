/*
 * ntn-dataset@1 writer for examples that are not built on NtnRealStackHelper (sweeps,
 * calibration tools, custom forwarding planes). It writes the same repro-manifest.json and a
 * long-format sim_health.csv (metric,value,floor,pass,provenance) that helper-built examples
 * write, so every example's run directory carries the dataset standard. The provenance class
 * says how the headline numbers were obtained: "measured" (packets through a simulated data
 * plane) or "analytic" (closed form, no packets). Plan: plan/MASTER_PLAN_GLOBE_DATASET_E2E_2026-11.md.
 */
#ifndef NTN_DATASET_WRITER_H
#define NTN_DATASET_WRITER_H

#include "ntn-repro-manifest.h"
#include "ntn-scene-recorder.h"

#include "ns3/double.h"
#include "ns3/mobility-model.h"
#include "ns3/node-list.h"
#include "ns3/node-container.h"
#include "ns3/node.h"

#include "ns3/rng-seed-manager.h"
#include "ns3/simulator.h"

#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <string>
#include <vector>

namespace ns3
{
namespace ntnobs
{

struct DatasetMetric
{
    std::string metric;
    std::string value;
    std::string provenance; ///< measured | config | geometry | standard-constant | analytic
};

/// Writes `<outDir>/repro-manifest.json` and `<outDir>/sim_health.csv`.
/// `provenanceClass` is "measured" or "analytic". Returns false if a file could not be written.
inline bool
WriteDatasetRecord(const std::string& outDir,
                   const std::string& scenario,
                   const std::string& provenanceClass,
                   int argc,
                   char** argv,
                   const std::vector<DatasetMetric>& metrics = {},
                   const std::map<std::string, std::string>& extras = {})
{
    std::error_code ec;
    const std::string dir = outDir.empty() ? std::string(".") : outDir;
    std::filesystem::create_directories(dir, ec);
    NtnReproManifest m;
    m.SetScenarioName(scenario)
        .SetScenarioDuration(Simulator::Now().GetSeconds())
        .SetRng(RngSeedManager::GetSeed(), RngSeedManager::GetRun())
        .SetCliArgv(argc, argv)
        .AddExtra("dataset_schema", "ntn-dataset@1")
        .AddExtra("provenance_class", provenanceClass)
        .AddExtra("built_on", "custom data plane or analytic tool (no NtnRealStackHelper)");
    for (const auto& [k, v] : extras)
    {
        m.AddExtra(k, v);
    }
    bool ok = m.WriteJson(dir + "/repro-manifest.json");
    std::ofstream h(dir + "/sim_health.csv");
    if (!h)
    {
        return false;
    }
    h << "metric,value,floor,pass,provenance\n";
    h << "sim_time_s," << Simulator::Now().GetSeconds() << ",-,-,config\n";
    h << "provenance_class," << provenanceClass << ",-,-,config\n";
    h << "rng_run," << RngSeedManager::GetRun() << ",-,-,config\n";
    for (const auto& x : metrics)
    {
        h << x.metric << "," << x.value << ",-,-," << x.provenance << "\n";
    }
    return ok && h.good();
}

/**
 * Give every satellite mobility model that is not attached to a node its own device-less node, so
 * a recorded scene can draw it (the auto scene of NtnRealStackHelper draws orbiting nodes the radio
 * stack does not own as context satellites, "ctx-sat-<id>"). For constellation members that only
 * feed a contact-graph scheduler, an O-RAN candidate list or a TTE predictor. The nodes carry no
 * devices, so the network and its measurements are unchanged; call it after the scenario created
 * its own nodes so their ids do not shift. Models already aggregated to a node are skipped.
 * Returns the new nodes.
 */
template <typename MobPtr>
inline NodeContainer
AddContextSatelliteNodes(const std::vector<MobPtr>& mobs)
{
    NodeContainer out;
    for (const auto& m : mobs)
    {
        if (!m || m->template GetObject<Node>())
        {
            continue;
        }
        Ptr<Node> n = CreateObject<Node>();
        n->AggregateObject(m);
        out.Add(n);
    }
    return out;
}

/**
 * Frame of a scene: a scenario that projects SGP4 into a local east-north-up frame
 * (NtnEnuProjectionMobilityModel, read through its read-only Reference* attributes) records an
 * ENU scene about that model's origin; otherwise node positions are taken as Earth-fixed (ECEF).
 * Returns true and the origin when the scene is ENU.
 */
inline bool
ConfigureSceneFrame(Ptr<NtnSceneRecorder> rec, double& lat0, double& lon0, double& alt0)
{
    for (uint32_t i = 0; i < NodeList::GetNNodes(); ++i)
    {
        Ptr<MobilityModel> mm = NodeList::GetNode(i)->GetObject<MobilityModel>();
        DoubleValue la, lo, al;
        if (mm && mm->GetAttributeFailSafe("ReferenceLatitude", la) &&
            mm->GetAttributeFailSafe("ReferenceLongitude", lo) &&
            mm->GetAttributeFailSafe("ReferenceAltitude", al))
        {
            lat0 = la.Get();
            lon0 = lo.Get();
            alt0 = al.Get();
            rec->SetFrame(NtnSceneRecorder::LocalEnu);
            rec->SetEnuReference(lat0, lon0, alt0);
            return true;
        }
    }
    rec->SetFrame(NtnSceneRecorder::EcefGlobal);
    return false;
}

/**
 * Globe scene for examples not built on NtnRealStackHelper: at t = 0, track every node whose
 * mobility model reports an Earth-fixed (ECEF) position, classify it by geodetic altitude
 * (satellite above 160 km, HAPS 15-60 km, aircraft 1-15 km, ground terminal below), write
 * `<outDir>/scene.czml` when the simulator is destroyed and, when NTN_LIVE_SCENE=1, stream
 * ##NTNSCENE## frames. Nodes in a local Cartesian frame (|p| below 6,200 km) are skipped, so
 * a scenario without real geography records nothing. Call once, before Simulator::Run().
 */
inline void
StartSceneFromNodeList(const std::string& outDirIn)
{
    const std::string outDir = outDirIn.empty() ? std::string(".") : outDirIn;
    Simulator::ScheduleNow([outDir]() {
        if (std::getenv("NTN_SCENE_DEBUG"))
        {
            std::cerr << "[scene] t=0 nodes=" << NodeList::GetNNodes()
                      << " active=" << NtnSceneRecorder::ActiveCount() << "\n";
        }
        if (std::getenv("NTN_SCENE_OFF") || NtnSceneRecorder::ActiveCount() > 0)
        {
            return;
        }
        Ptr<NtnSceneRecorder> rec = CreateObject<NtnSceneRecorder>();
        rec->SetSampleInterval(Seconds(1.0));
        rec->SetFrameBudget(150);
        double lat0 = 0, lon0 = 0, alt0 = 0;
        const bool enu = ConfigureSceneFrame(rec, lat0, lon0, alt0);
        uint32_t tracked = 0;
        for (uint32_t i = 0; i < NodeList::GetNNodes(); ++i)
        {
            Ptr<Node> n = NodeList::GetNode(i);
            Ptr<MobilityModel> mm = n->GetObject<MobilityModel>();
            if (!mm)
            {
                continue;
            }
            const Vector p = mm->GetPosition();
            const double r = std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
            if (std::getenv("NTN_SCENE_DEBUG"))
            {
                std::cerr << "[scene] node " << n->GetId() << " " << mm->GetInstanceTypeId().GetName()
                          << " |p|=" << r << " m" << (enu ? " (enu)" : "") << "\n";
            }
            double h = 0;
            if (enu)
            {
                if (r > 5.0e7)
                {
                    continue;
                }
                h = alt0 + p.z; // up component; curvature is small at LEO slant ranges here
            }
            else
            {
                if (r < 6.2e6 || r > 5.0e7)
                {
                    continue; // local frame or not on/near the Earth
                }
                // WGS84 geodetic height (Bowring single iteration, < 1 m at these altitudes)
                const double a = 6378137.0, e2 = 6.69437999014e-3, b = a * std::sqrt(1 - e2);
                const double pxy = std::sqrt(p.x * p.x + p.y * p.y);
                const double th = std::atan2(p.z * a, pxy * b);
                const double lat = std::atan2(p.z + (e2 * a * a / (b * b) - e2) * b * std::pow(std::sin(th), 3),
                                              pxy - e2 * a * std::pow(std::cos(th), 3));
                const double nrad = a / std::sqrt(1 - e2 * std::sin(lat) * std::sin(lat));
                h = (std::abs(std::cos(lat)) > 1e-9) ? pxy / std::cos(lat) - nrad : std::abs(p.z) - b;
            }
            NtnSceneRecorder::NodeKind k = h > 160e3   ? NtnSceneRecorder::Sat
                                           : h > 15e3  ? NtnSceneRecorder::Haps
                                           : h > 1e3   ? NtnSceneRecorder::Aircraft
                                                       : NtnSceneRecorder::Ue;
            const char* tag = k == NtnSceneRecorder::Sat        ? "sat-"
                              : k == NtnSceneRecorder::Haps     ? "haps-"
                              : k == NtnSceneRecorder::Aircraft ? "air-"
                                                                : "ground-";
            rec->TrackNode(n, k, tag + std::to_string(n->GetId()));
            ++tracked;
        }
        if (tracked == 0)
        {
            return;
        }
        std::error_code ec;
        std::filesystem::create_directories(outDir, ec);
        rec->EnableCzml(outDir + "/scene.czml");
        const char* live = std::getenv("NTN_LIVE_SCENE");
        if (live && std::string(live) == "1")
        {
            rec->EnableLiveStdout(true);
        }
        rec->Start();
        Simulator::ScheduleDestroy([rec]() { rec->Stop(); });
    });
}

} // namespace ntnobs
} // namespace ns3

#endif // NTN_DATASET_WRITER_H
