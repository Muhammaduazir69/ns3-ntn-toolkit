/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#include "thz-ntn-nyusim-reference.h"

#include "ns3/log.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <sstream>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("ThzNtnNyusimReference");
NS_OBJECT_ENSURE_REGISTERED(ThzNtnNyusimReference);

namespace
{

constexpr const char* kDefaultRelativePath =
    "contrib/thz-ntn/data/nyusim-140-reference.csv";

std::string
Trim(const std::string& s)
{
    auto begin = s.find_first_not_of(" \t\r\n");
    auto end = s.find_last_not_of(" \t\r\n");
    if (begin == std::string::npos)
    {
        return "";
    }
    return s.substr(begin, end - begin + 1);
}

bool
ToBool(const std::string& s)
{
    return Trim(s) == "los";
}

/// CI-model (n, FSPL_1m_dB) coefficients per Rappaport 2019 + NYUSIM-140.
/// Picks values that match the bundled CSV; calibration code can also use
/// this surface to construct expected residual envelopes for an arbitrary
/// (env, los, dist).
void
CiCoefficients(const std::string& environment,
                bool los,
                double& n,
                double& sigma_dB)
{
    if (environment == "suburban")
    {
        n = los ? 2.0 : 2.8;
        sigma_dB = los ? 3.0 : 8.0;
    }
    else if (environment == "indoor")
    {
        n = los ? 1.7 : 3.0;
        sigma_dB = los ? 3.0 : 8.0;
    }
    else
    {
        // urban default
        n = los ? 2.0 : 2.9;
        sigma_dB = los ? 4.0 : 9.0;
    }
}

} // namespace

TypeId
ThzNtnNyusimReference::GetTypeId()
{
    static TypeId tid = TypeId("ns3::ThzNtnNyusimReference")
                            .SetParent<Object>()
                            .SetGroupName("ThzNtn")
                            .AddConstructor<ThzNtnNyusimReference>();
    return tid;
}

ThzNtnNyusimReference::ThzNtnNyusimReference()
{
    const char* envPath = std::getenv("THZ_NTN_NYUSIM_CSV");
    if (envPath && *envPath)
    {
        m_csvPath = envPath;
    }
    else
    {
        // Use the toolkit-relative path; callers running from the repo root
        // (ns-3-dev) will resolve it directly. Tests + examples may
        // override via SetCsvPath().
        m_csvPath = kDefaultRelativePath;
    }
}

ThzNtnNyusimReference::~ThzNtnNyusimReference() = default;

std::size_t
ThzNtnNyusimReference::Load()
{
    m_entries.clear();
    m_lastError.clear();

    std::ifstream f(m_csvPath);
    if (!f.is_open())
    {
        // Also try the absolute toolkit path so unit tests run from an
        // arbitrary working directory still locate the bundled CSV.
        const std::string fallback = std::string("/home/uzair/6g_ntn_ns3/ns-3-dev/")
                                      + kDefaultRelativePath;
        f.open(fallback);
        if (!f.is_open())
        {
            m_lastError = "cannot open " + m_csvPath +
                            " (tried fallback " + fallback + ")";
            return 0;
        }
    }
    std::string line;
    std::size_t lineNo = 0;
    while (std::getline(f, line))
    {
        ++lineNo;
        const std::string trimmed = Trim(line);
        if (trimmed.empty() || trimmed[0] == '#')
        {
            continue;
        }
        Entry e{};
        std::string err;
        if (!ParseLine(trimmed, e, err))
        {
            m_lastError = "line " + std::to_string(lineNo) + ": " + err;
            return 0;
        }
        m_entries.push_back(e);
    }
    return m_entries.size();
}

bool
ThzNtnNyusimReference::ParseLine(const std::string& line,
                                  Entry& out,
                                  std::string& err)
{
    std::istringstream is(line);
    std::string scenario, freqStr, distStr, plStr, stdStr, env;
    if (!std::getline(is, scenario, ',') ||
        !std::getline(is, freqStr, ',') ||
        !std::getline(is, distStr, ',') ||
        !std::getline(is, plStr, ',') ||
        !std::getline(is, stdStr, ',') ||
        !std::getline(is, env))
    {
        err = "expected 6 comma-separated fields";
        return false;
    }
    out.los = ToBool(scenario);
    try
    {
        out.freq_ghz = std::stod(freqStr);
        out.dist_m = std::stod(distStr);
        out.pl_db = std::stod(plStr);
        out.std_db = std::stod(stdStr);
    }
    catch (const std::exception& ex)
    {
        err = std::string("number parse: ") + ex.what();
        return false;
    }
    out.environment = Trim(env);
    return true;
}

std::vector<ThzNtnNyusimReference::Entry>
ThzNtnNyusimReference::Select(const std::string& environment, bool los) const
{
    std::vector<Entry> out;
    out.reserve(m_entries.size());
    for (const auto& e : m_entries)
    {
        if (e.environment == environment && e.los == los)
        {
            out.push_back(e);
        }
    }
    return out;
}

double
ThzNtnNyusimReference::ReferenceCiPathLossDb(const std::string& environment,
                                              bool los,
                                              double dist_m,
                                              double freq_hz)
{
    if (dist_m <= 0.0 || freq_hz <= 0.0)
    {
        return 0.0;
    }
    double n = 0.0;
    double sigma = 0.0;
    CiCoefficients(environment, los, n, sigma);
    const double fGhz = freq_hz / 1e9;
    const double fspl_1m =
        20.0 * std::log10(1.0) + 20.0 * std::log10(fGhz) + 32.45;
    return fspl_1m + 10.0 * n * std::log10(std::max(dist_m, 1e-3));
}

} // namespace ns3
