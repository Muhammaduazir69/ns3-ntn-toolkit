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

#include "ns3/rng-seed-manager.h"
#include "ns3/simulator.h"

#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <string>

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
 * Globe scene for examples not built on NtnRealStackHelper: at t = 0, track every node whose
 * mobility model reports an Earth-fixed (ECEF) position, classify it by geodetic altitude
 * (satellite above 160 km, HAPS 15-60 km, aircraft 1-15 km, ground terminal below), write
 * `<outDir>/scene.czml` when the simulator is destroyed and, when NTN_LIVE_SCENE=1, stream
 * ##NTNSCENE## frames. Nodes in a local Cartesian frame (|p| below 6,200 km) are skipped, so
 * a scenario without real geography records nothing. Call once, before Simulator::Run().
 */
inline void
StartSceneFromNodeList(const std::string& outDir)
{
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
        // Frame: a scenario that projects SGP4 into a local east-north-up frame
        // (NtnEnuProjectionMobilityModel) records an ENU scene about that model's origin;
        // otherwise positions must be Earth-fixed (ECEF).
        bool enu = false;
        double lat0 = 0, lon0 = 0, alt0 = 0;
        for (uint32_t i = 0; i < NodeList::GetNNodes() && !enu; ++i)
        {
            Ptr<MobilityModel> mm = NodeList::GetNode(i)->GetObject<MobilityModel>();
            DoubleValue la, lo, al;
            if (mm && mm->GetAttributeFailSafe("ReferenceLatitude", la) &&
                mm->GetAttributeFailSafe("ReferenceLongitude", lo) &&
                mm->GetAttributeFailSafe("ReferenceAltitude", al))
            {
                enu = true;
                lat0 = la.Get();
                lon0 = lo.Get();
                alt0 = al.Get();
            }
        }
        if (enu)
        {
            rec->SetEnuReference(lat0, lon0, alt0);
        }
        else
        {
            rec->SetFrame(NtnSceneRecorder::EcefGlobal);
        }
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
