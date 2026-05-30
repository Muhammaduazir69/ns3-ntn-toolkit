/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#include "tr38821-corpus.h"

#include "ns3/log.h"

#include <cmath>
#include <fstream>
#include <sstream>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("Tr38821CorpusReader");

namespace
{

std::vector<std::string>
SplitCsv(const std::string& line)
{
    std::vector<std::string> out;
    std::string cur;
    for (char c : line)
    {
        if (c == ',')
        {
            out.push_back(cur);
            cur.clear();
        }
        else if (c == '\r')
        {
            // skip
        }
        else
        {
            cur.push_back(c);
        }
    }
    out.push_back(cur);
    return out;
}

bool
ReadCsv(const std::string& path,
         std::vector<std::vector<std::string>>& rows)
{
    std::ifstream in(path);
    if (!in)
    {
        return false;
    }
    std::string line;
    bool header_seen = false;
    while (std::getline(in, line))
    {
        if (line.empty() || line[0] == '#')
        {
            continue;
        }
        if (!header_seen)
        {
            // first non-comment, non-blank line is the header
            header_seen = true;
            continue;
        }
        rows.push_back(SplitCsv(line));
    }
    return true;
}

double
ParseDouble(const std::string& s, double def = 0.0)
{
    if (s.empty())
    {
        return def;
    }
    try
    {
        return std::stod(s);
    }
    catch (...)
    {
        return def;
    }
}

int
ParseInt(const std::string& s, int def = 0)
{
    if (s.empty())
    {
        return def;
    }
    try
    {
        return std::stoi(s);
    }
    catch (...)
    {
        return def;
    }
}

} // namespace

bool
Tr38821CorpusReader::LoadScenarios(const std::string& path)
{
    m_scenarios.clear();
    std::vector<std::vector<std::string>> rows;
    if (!ReadCsv(path, rows))
    {
        return false;
    }
    for (const auto& r : rows)
    {
        if (r.size() < 12)
        {
            continue;
        }
        Tr38821Scenario s;
        s.scenario_id = r[0];
        s.orbit = r[1];
        s.freq_band = r[2];
        s.freq_ghz = ParseDouble(r[3]);
        s.alt_km = ParseDouble(r[4]);
        s.max_eirp_dbw = ParseDouble(r[5]);
        s.ue_g_t_db_per_k = ParseDouble(r[6]);
        s.ue_class = r[7];
        s.bandwidth_mhz = ParseDouble(r[8]);
        s.target_cnr_db = ParseDouble(r[9]);
        s.link_pathloss_db_min = ParseDouble(r[10]);
        s.link_pathloss_db_max = ParseDouble(r[11]);
        m_scenarios.push_back(s);
    }
    return !m_scenarios.empty();
}

bool
Tr38821CorpusReader::LoadLinkBudgets(const std::string& path)
{
    m_linkBudgets.clear();
    std::vector<std::vector<std::string>> rows;
    if (!ReadCsv(path, rows))
    {
        return false;
    }
    for (const auto& r : rows)
    {
        if (r.size() < 7)
        {
            continue;
        }
        Tr38821LinkBudget b;
        b.scenario_id = r[0];
        b.elevation_deg = ParseDouble(r[1]);
        b.pathloss_db = ParseDouble(r[2]);
        b.shadowing_db = ParseDouble(r[3]);
        b.atmos_loss_db = ParseDouble(r[4]);
        b.antenna_gain_dbi = ParseDouble(r[5]);
        b.cnr_db = ParseDouble(r[6]);
        m_linkBudgets.push_back(b);
    }
    return !m_linkBudgets.empty();
}

bool
Tr38821CorpusReader::LoadStarlinkLatency(const std::string& path)
{
    m_latency.clear();
    std::vector<std::vector<std::string>> rows;
    if (!ReadCsv(path, rows))
    {
        return false;
    }
    for (const auto& r : rows)
    {
        if (r.size() < 6)
        {
            continue;
        }
        StarlinkLatencySample s;
        s.station_id = r[0];
        s.utc_hour = ParseInt(r[1]);
        s.rtt_p50_ms = ParseDouble(r[2]);
        s.rtt_p95_ms = ParseDouble(r[3]);
        s.pkt_loss_pct = ParseDouble(r[4]);
        s.jitter_ms = ParseDouble(r[5]);
        m_latency.push_back(s);
    }
    return !m_latency.empty();
}

bool
Tr38821CorpusReader::LoadStarlinkStations(const std::string& path)
{
    m_stations.clear();
    std::vector<std::vector<std::string>> rows;
    if (!ReadCsv(path, rows))
    {
        return false;
    }
    for (const auto& r : rows)
    {
        if (r.size() < 7)
        {
            continue;
        }
        StarlinkStation s;
        s.station_id = r[0];
        s.city = r[1];
        s.country_iso2 = r[2];
        s.lat_deg = ParseDouble(r[3]);
        s.lon_deg = ParseDouble(r[4]);
        s.pop_lat_deg = ParseDouble(r[5]);
        s.pop_lon_deg = ParseDouble(r[6]);
        m_stations.push_back(s);
    }
    return !m_stations.empty();
}

std::optional<Tr38821Scenario>
Tr38821CorpusReader::FindScenario(const std::string& id) const
{
    for (const auto& s : m_scenarios)
    {
        if (s.scenario_id == id)
        {
            return s;
        }
    }
    return std::nullopt;
}

std::optional<StarlinkStation>
Tr38821CorpusReader::FindStation(const std::string& id) const
{
    for (const auto& s : m_stations)
    {
        if (s.station_id == id)
        {
            return s;
        }
    }
    return std::nullopt;
}

// ----------------------------------------------------------------------------
// CalibrationHarness
// ----------------------------------------------------------------------------

double
CalibrationHarness::GateFor(const std::string& metric) const
{
    if (metric == "pathloss")
    {
        return m_gates.pathloss_db;
    }
    if (metric == "atmos_loss")
    {
        return m_gates.atmos_db;
    }
    if (metric == "cnr")
    {
        return m_gates.cnr_db;
    }
    if (metric == "rtt_p50" || metric == "rtt_p95")
    {
        return m_gates.rtt_ms;
    }
    return 1.0;
}

CalibrationResidual
CalibrationHarness::Compare(const std::string& scenario_id,
                              const std::string& metric,
                              double reference,
                              double toolkit)
{
    CalibrationResidual r;
    r.scenario_id = scenario_id;
    r.metric = metric;
    r.reference = reference;
    r.toolkit = toolkit;
    r.residual = toolkit - reference;
    r.within_gate = std::fabs(r.residual) <= GateFor(metric);
    m_residuals.push_back(r);
    return r;
}

bool
CalibrationHarness::AllWithinGate() const
{
    for (const auto& r : m_residuals)
    {
        if (!r.within_gate)
        {
            return false;
        }
    }
    return true;
}

std::map<std::string, uint32_t>
CalibrationHarness::CountsByMetric() const
{
    std::map<std::string, uint32_t> out;
    for (const auto& r : m_residuals)
    {
        ++out[r.metric];
    }
    return out;
}

std::map<std::string, uint32_t>
CalibrationHarness::FailuresByMetric() const
{
    std::map<std::string, uint32_t> out;
    for (const auto& r : m_residuals)
    {
        if (!r.within_gate)
        {
            ++out[r.metric];
        }
    }
    return out;
}

} // namespace ns3
