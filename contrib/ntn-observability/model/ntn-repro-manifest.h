/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#ifndef NTN_REPRO_MANIFEST_H
#define NTN_REPRO_MANIFEST_H

#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace ns3
{
namespace ntnobs
{

/**
 * Reproducibility manifest — 2026 realism roadmap §3 T9.
 *
 * Every example writes one of these JSON files alongside its CSV/Parquet
 * outputs. A subsequent run can pass `--replay-manifest=path.json` to
 * deterministically replay the scenario: scenario name, duration, RNG
 * seed/run, TLE epoch + NORAD IDs, constellation generator parameters,
 * and CLI argv are restored from the manifest before Simulator::Run().
 *
 * Provenance fields (git SHA, ns-3 version, Sionna RT version, Sionna
 * scene SHA-256, HITRAN release tag, ITU-Rpy version, RIC service-model
 * versions) are recorded but not used by the replay path.
 *
 * The JSON schema is "ns3-ntn-toolkit/manifest" version 1; reviewers can
 * pin the schema in their citations. Unknown top-level keys are ignored
 * by the parser so newer manifests remain readable by older toolkits.
 */
class NtnReproManifest
{
  public:
    static constexpr const char* kSchemaName = "ns3-ntn-toolkit/manifest";
    static constexpr uint32_t kSchemaVersion = 1;

    NtnReproManifest();

    // --- builders (chained) ---
    NtnReproManifest& SetToolkitGitSha(const std::string& sha);
    NtnReproManifest& SetNs3Version(const std::string& v);
    NtnReproManifest& SetScenarioName(const std::string& name);
    NtnReproManifest& SetScenarioDuration(double seconds);
    NtnReproManifest& SetRng(uint32_t seed, uint32_t run);
    NtnReproManifest& SetTleEpoch(const std::string& utc);
    NtnReproManifest& AddNoradId(uint32_t id);
    NtnReproManifest& SetConstellation(const std::string& name,
                                       uint32_t planes,
                                       uint32_t satsPerPlane,
                                       double altKm,
                                       double incDeg);
    NtnReproManifest& SetSionna(const std::string& rtVersion,
                                const std::string& sceneSha256);
    NtnReproManifest& SetHitranRelease(const std::string& tag);
    NtnReproManifest& SetItuRpyVersion(const std::string& v);
    NtnReproManifest& AddServiceModel(const std::string& name,
                                      const std::string& version);
    NtnReproManifest& SetCliArgv(int argc, char** argv);
    NtnReproManifest& AddExtra(const std::string& key, const std::string& value);

    // --- I/O ---

    /// Atomically writes a JSON manifest to `path` (writes to .tmp, renames).
    /// Returns true on success.
    bool WriteJson(const std::string& path) const;
    /// Returns the JSON serialisation as a string.
    std::string ToJson() const;
    /// Reads a manifest from `path`. Returns false on I/O or parse error;
    /// on failure `out` is in a partially-populated state.
    static bool LoadJson(const std::string& path, NtnReproManifest& out);
    /// Parse a JSON manifest from a string. Same error semantics as LoadJson.
    static bool ParseJson(const std::string& body, NtnReproManifest& out);

    // --- accessors (replay path) ---
    const std::string& GetToolkitGitSha() const { return m_gitSha; }
    const std::string& GetNs3Version() const { return m_ns3Version; }
    const std::string& GetCreatedUtc() const { return m_createdUtc; }
    const std::string& GetScenarioName() const { return m_scenarioName; }
    double GetScenarioDuration() const { return m_scenarioDuration; }
    uint32_t GetRngSeed() const { return m_rngSeed; }
    uint32_t GetRngRun() const { return m_rngRun; }
    const std::string& GetTleEpoch() const { return m_tleEpoch; }
    const std::vector<uint32_t>& GetNoradIds() const { return m_noradIds; }
    bool HasConstellation() const { return m_hasConstellation; }
    const std::string& GetConstellationName() const { return m_constName; }
    uint32_t GetPlanes() const { return m_planes; }
    uint32_t GetSatsPerPlane() const { return m_satsPerPlane; }
    double GetAltitudeKm() const { return m_altKm; }
    double GetInclinationDeg() const { return m_incDeg; }
    const std::string& GetSionnaRtVersion() const { return m_sionnaRt; }
    const std::string& GetSionnaSceneSha256() const { return m_sionnaSceneSha; }
    const std::string& GetHitranRelease() const { return m_hitran; }
    const std::string& GetItuRpyVersion() const { return m_ituRpy; }
    const std::vector<std::pair<std::string, std::string>>& GetServiceModels() const
    {
        return m_serviceModels;
    }
    const std::vector<std::string>& GetCliArgv() const { return m_cliArgv; }
    const std::vector<std::pair<std::string, std::string>>& GetExtras() const
    {
        return m_extras;
    }

  private:
    std::string m_gitSha;
    std::string m_ns3Version{"ns-3.43"};
    std::string m_createdUtc;
    std::string m_scenarioName;
    double m_scenarioDuration{0.0};
    uint32_t m_rngSeed{1};
    uint32_t m_rngRun{1};
    std::string m_tleEpoch;
    std::vector<uint32_t> m_noradIds;
    bool m_hasConstellation{false};
    std::string m_constName;
    uint32_t m_planes{0};
    uint32_t m_satsPerPlane{0};
    double m_altKm{0.0};
    double m_incDeg{0.0};
    std::string m_sionnaRt;
    std::string m_sionnaSceneSha;
    std::string m_hitran;
    std::string m_ituRpy;
    std::vector<std::pair<std::string, std::string>> m_serviceModels;
    std::vector<std::string> m_cliArgv;
    std::vector<std::pair<std::string, std::string>> m_extras;
};

} // namespace ntnobs
} // namespace ns3

#endif // NTN_REPRO_MANIFEST_H
