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

#include "ns3/rng-seed-manager.h"
#include "ns3/simulator.h"

#include <filesystem>
#include <fstream>
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

} // namespace ntnobs
} // namespace ns3

#endif // NTN_DATASET_WRITER_H
