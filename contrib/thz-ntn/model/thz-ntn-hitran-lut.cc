/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#include "thz-ntn-hitran-lut.h"

#include "ns3/log.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <set>
#include <sstream>

namespace ns3
{
namespace thzntn
{

NS_LOG_COMPONENT_DEFINE("ThzNtnHitranLut");

namespace
{

bool
ParseDouble(const std::string& s, double& out)
{
    try
    {
        out = std::stod(s);
        return true;
    }
    catch (...)
    {
        return false;
    }
}

} // namespace

bool
HitranLut::LoadCsv(const std::string& path)
{
    std::ifstream f(path);
    if (!f)
    {
        NS_LOG_WARN("HitranLut: cannot open " << path);
        return false;
    }
    m_freqGhz.clear();
    m_altKm.clear();
    m_attDbKm.clear();
    m_releaseTag.clear();

    // Pre-pass: collect unique freq + alt grids so we can size the table.
    std::set<double> freqSet;
    std::set<double> altSet;
    std::vector<std::tuple<double, double, double>> raw; // (f, h, att)
    std::string line;
    size_t lineNo = 0;
    while (std::getline(f, line))
    {
        ++lineNo;
        if (line.empty())
        {
            continue;
        }
        // Comments / metadata lines start with '#'. A `# release: HITRAN-2024`
        // line, if present, sets the release tag.
        if (line[0] == '#')
        {
            auto pos = line.find("release:");
            if (pos != std::string::npos)
            {
                m_releaseTag = line.substr(pos + 8);
                // strip leading whitespace
                while (!m_releaseTag.empty() &&
                       (m_releaseTag.front() == ' ' ||
                        m_releaseTag.front() == '\t'))
                {
                    m_releaseTag.erase(m_releaseTag.begin());
                }
            }
            continue;
        }
        // Skip header.
        if (line.find("freq_ghz") != std::string::npos)
        {
            continue;
        }
        std::istringstream is(line);
        std::string fStr;
        std::string hStr;
        std::string aStr;
        if (!std::getline(is, fStr, ',') ||
            !std::getline(is, hStr, ',') ||
            !std::getline(is, aStr, ','))
        {
            NS_LOG_WARN("HitranLut: malformed row at line " << lineNo);
            return false;
        }
        double freq;
        double alt;
        double att;
        if (!ParseDouble(fStr, freq) || !ParseDouble(hStr, alt) ||
            !ParseDouble(aStr, att))
        {
            NS_LOG_WARN("HitranLut: bad numeric at line " << lineNo);
            return false;
        }
        freqSet.insert(freq);
        altSet.insert(alt);
        raw.emplace_back(freq, alt, att);
    }
    if (raw.empty())
    {
        NS_LOG_WARN("HitranLut: empty body at " << path);
        return false;
    }
    m_freqGhz.assign(freqSet.begin(), freqSet.end());
    m_altKm.assign(altSet.begin(), altSet.end());
    m_attDbKm.assign(m_freqGhz.size() * m_altKm.size(),
                      std::numeric_limits<double>::quiet_NaN());

    auto findIdx = [](const std::vector<double>& v, double x) {
        return static_cast<size_t>(std::lower_bound(v.begin(), v.end(), x) -
                                    v.begin());
    };
    for (const auto& [freq, alt, att] : raw)
    {
        size_t fi = findIdx(m_freqGhz, freq);
        size_t ai = findIdx(m_altKm, alt);
        m_attDbKm[fi * m_altKm.size() + ai] = att;
    }
    // Verify the table is fully populated: a missing (f, h) cell would
    // surface as NaN at interpolation time, so we reject the file outright.
    for (double v : m_attDbKm)
    {
        if (std::isnan(v))
        {
            NS_LOG_WARN("HitranLut: sparse grid in " << path
                                                      << ", refusing to load");
            m_freqGhz.clear();
            m_altKm.clear();
            m_attDbKm.clear();
            return false;
        }
    }
    NS_LOG_INFO("HitranLut: loaded " << raw.size() << " rows from " << path
                                       << " (" << m_freqGhz.size() << " freqs × "
                                       << m_altKm.size() << " alts), release="
                                       << (m_releaseTag.empty() ? "?"
                                                                : m_releaseTag));
    return true;
}

size_t
HitranLut::LowerIndex(const std::vector<double>& v, double x) const
{
    if (v.empty())
    {
        return 0;
    }
    if (x <= v.front())
    {
        return 0;
    }
    if (x >= v.back())
    {
        return v.size() - 1;
    }
    auto it = std::upper_bound(v.begin(), v.end(), x);
    return static_cast<size_t>((it - v.begin()) - 1);
}

double
HitranLut::Get(double freqHz, double altKm) const
{
    if (!IsLoaded())
    {
        return std::numeric_limits<double>::quiet_NaN();
    }
    // Return NaN for queries outside the tabulated grid. The LUT covers a
    // finite (frequency x altitude) domain (e.g. 100-500 GHz x 0-30 km);
    // edge-clamping an out-of-domain query would misapply the boundary cell
    // — e.g. reading the 500 GHz column for a 600 GHz link, or integrating
    // the 30 km rate up to 100 km, both of which fabricate absorption where
    // reality is ~0. Callers (ComputeSlantPathAbsorption / GetTransmittance)
    // treat NaN as "not covered" and fall back to the in-process P.676-13
    // kernel, which is valid over the full domain.
    const double freqGhzRaw = freqHz / 1e9;
    constexpr double kEps = 1e-9;   // tolerance for exact-edge queries
    if (freqGhzRaw < m_freqGhz.front() - kEps ||
        freqGhzRaw > m_freqGhz.back() + kEps ||
        altKm < m_altKm.front() - kEps ||
        altKm > m_altKm.back() + kEps)
    {
        return std::numeric_limits<double>::quiet_NaN();
    }
    // Clamp exact-edge / rounding queries into range (grid interior only).
    const double freqGhz =
        std::min(std::max(freqGhzRaw, m_freqGhz.front()), m_freqGhz.back());
    const double altKmCl =
        std::min(std::max(altKm, m_altKm.front()), m_altKm.back());
    const size_t fi = LowerIndex(m_freqGhz, freqGhz);
    const size_t ai = LowerIndex(m_altKm, altKmCl);
    const size_t fiNext = std::min(fi + 1, m_freqGhz.size() - 1);
    const size_t aiNext = std::min(ai + 1, m_altKm.size() - 1);
    const double f0 = m_freqGhz[fi];
    const double f1 = m_freqGhz[fiNext];
    const double h0 = m_altKm[ai];
    const double h1 = m_altKm[aiNext];
    const double tf = (f1 == f0) ? 0.0 : (freqGhz - f0) / (f1 - f0);
    const double th = (h1 == h0) ? 0.0 : (altKmCl - h0) / (h1 - h0);
    const auto idx = [&](size_t f, size_t a) {
        return f * m_altKm.size() + a;
    };
    const double a00 = m_attDbKm[idx(fi, ai)];
    const double a01 = m_attDbKm[idx(fi, aiNext)];
    const double a10 = m_attDbKm[idx(fiNext, ai)];
    const double a11 = m_attDbKm[idx(fiNext, aiNext)];
    return (1.0 - tf) * ((1.0 - th) * a00 + th * a01) +
           tf * ((1.0 - th) * a10 + th * a11);
}

} // namespace thzntn
} // namespace ns3
