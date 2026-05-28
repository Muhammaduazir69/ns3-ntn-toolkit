/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#ifndef NTN_CONSTELLATION_TR38821_CORPUS_H
#define NTN_CONSTELLATION_TR38821_CORPUS_H

// 3GPP TR 38.821 + Starlink EU calibration corpus (Roadmap §4.4.11).
//
// Loads the toolkit's bundled reference CSVs from
//   contrib/ntn-constellation/data/tr38821/   (scenarios + link budgets)
//   contrib/ntn-constellation/data/starlink_eu/ (latency + PoP locations)
//
// and exposes a `CalibrationHarness` that compares a toolkit prediction
// against the corpus, returning per-row residuals.

#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <vector>

namespace ns3
{

struct Tr38821Scenario
{
    std::string scenario_id;
    std::string orbit;       // "GEO" / "LEO_600" / "LEO_1200"
    std::string freq_band;   // "S" / "Ka_DL" / "Ka_UL"
    double freq_ghz{0.0};
    double alt_km{0.0};
    double max_eirp_dbw{0.0};
    double ue_g_t_db_per_k{0.0};
    std::string ue_class;
    double bandwidth_mhz{0.0};
    double target_cnr_db{0.0};
    double link_pathloss_db_min{0.0};
    double link_pathloss_db_max{0.0};
};

struct Tr38821LinkBudget
{
    std::string scenario_id;
    double elevation_deg{0.0};
    double pathloss_db{0.0};
    double shadowing_db{0.0};
    double atmos_loss_db{0.0};
    double antenna_gain_dbi{0.0};
    double cnr_db{0.0};
};

struct StarlinkLatencySample
{
    std::string station_id;
    int utc_hour{0};
    double rtt_p50_ms{0.0};
    double rtt_p95_ms{0.0};
    double pkt_loss_pct{0.0};
    double jitter_ms{0.0};
};

struct StarlinkStation
{
    std::string station_id;
    std::string city;
    std::string country_iso2;
    double lat_deg{0.0};
    double lon_deg{0.0};
    double pop_lat_deg{0.0};
    double pop_lon_deg{0.0};
};

/// One pass / fail residual record.
struct CalibrationResidual
{
    std::string scenario_id;
    std::string metric;         // "pathloss" / "atmos_loss" / "cnr" / "rtt_p50"
    double reference{0.0};
    double toolkit{0.0};
    double residual{0.0};       // toolkit - reference
    bool within_gate{false};
};

class Tr38821CorpusReader
{
  public:
    bool LoadScenarios(const std::string& path);
    bool LoadLinkBudgets(const std::string& path);
    bool LoadStarlinkLatency(const std::string& path);
    bool LoadStarlinkStations(const std::string& path);

    const std::vector<Tr38821Scenario>& Scenarios() const { return m_scenarios; }
    const std::vector<Tr38821LinkBudget>& LinkBudgets() const { return m_linkBudgets; }
    const std::vector<StarlinkLatencySample>& StarlinkLatency() const { return m_latency; }
    const std::vector<StarlinkStation>& StarlinkStations() const { return m_stations; }

    std::optional<Tr38821Scenario> FindScenario(const std::string& id) const;
    std::optional<StarlinkStation> FindStation(const std::string& id) const;

  private:
    std::vector<Tr38821Scenario> m_scenarios;
    std::vector<Tr38821LinkBudget> m_linkBudgets;
    std::vector<StarlinkLatencySample> m_latency;
    std::vector<StarlinkStation> m_stations;
};

/**
 * \ingroup ntn-constellation
 * \brief Calibration harness: compares toolkit predictions vs corpus
 *        reference values, enforces ≤ gate residuals (Roadmap §4.4.11).
 *
 * Default gates: ≤ 1 dB on pathloss / atmos / CNR; ≤ 5 ms on RTT p50.
 * Users can tighten or loosen via SetGates().
 */
class CalibrationHarness
{
  public:
    struct Gates
    {
        double pathloss_db{1.0};
        double atmos_db{1.0};
        double cnr_db{1.0};
        double rtt_ms{5.0};
    };

    CalibrationHarness() = default;

    void SetGates(const Gates& g) { m_gates = g; }
    const Gates& GetGates() const { return m_gates; }

    /// Record one observation. Returns the residual record.
    CalibrationResidual Compare(const std::string& scenario_id,
                                  const std::string& metric,
                                  double reference,
                                  double toolkit);

    const std::vector<CalibrationResidual>& Residuals() const
    {
        return m_residuals;
    }

    /// `true` iff every recorded residual is within its gate.
    bool AllWithinGate() const;

    /// Counts by metric, useful for end-of-run reporting.
    std::map<std::string, uint32_t> CountsByMetric() const;
    std::map<std::string, uint32_t> FailuresByMetric() const;

    void Reset() { m_residuals.clear(); }

  private:
    double GateFor(const std::string& metric) const;

    Gates m_gates;
    std::vector<CalibrationResidual> m_residuals;
};

} // namespace ns3

#endif // NTN_CONSTELLATION_TR38821_CORPUS_H
