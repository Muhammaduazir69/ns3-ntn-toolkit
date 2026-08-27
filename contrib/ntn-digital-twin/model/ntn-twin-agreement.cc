/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#include "ntn-twin-agreement.h"

#include "ns3/log.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>

namespace ns3
{
namespace ntntwin
{

NS_LOG_COMPONENT_DEFINE("NtnTwinAgreement");

std::vector<TwinHandover>
LoadTwinPredictions(const std::string& path)
{
    std::vector<TwinHandover> out;
    std::ifstream f(path);
    if (!f)
    {
        return out;
    }
    std::string line;
    while (std::getline(f, line))
    {
        if (line.empty() || line[0] == '#')
        {
            continue;
        }
        std::istringstream ss(line);
        std::string tok;
        TwinHandover h;
        try
        {
            if (!std::getline(ss, tok, ','))
            {
                continue;
            }
            h.tSec = std::stod(tok);
            if (!std::getline(ss, tok, ','))
            {
                continue;
            }
            h.ueId = static_cast<uint32_t>(std::stoul(tok));
            if (!std::getline(ss, tok, ','))
            {
                continue;
            }
            h.gnbId = static_cast<uint32_t>(std::stoul(tok));
            if (std::getline(ss, tok, ','))
            {
                h.confidence = std::stod(tok);
            }
        }
        catch (const std::exception&)
        {
            continue; // malformed row: skip rather than abort the comparison
        }
        out.push_back(h);
    }
    std::sort(out.begin(), out.end(),
              [](const TwinHandover& a, const TwinHandover& b) { return a.tSec < b.tSec; });
    return out;
}

TwinRunMeta
LoadTwinRunMeta(const std::string& path)
{
    TwinRunMeta m;
    std::ifstream f(path);
    if (!f)
    {
        return m;
    }
    auto grab = [](const std::string& line, const std::string& key, double& out) {
        const auto p = line.find(key + "=");
        if (p == std::string::npos)
        {
            return false;
        }
        try
        {
            out = std::stod(line.substr(p + key.size() + 1));
            return true;
        }
        catch (const std::exception&)
        {
            return false;
        }
    };
    std::string line;
    bool sawObs = false;
    while (std::getline(f, line) && !line.empty() && line[0] == '#')
    {
        grab(line, "epoch_unix", m.epochUnix);
        if (grab(line, "observer_lat_deg", m.observerLatDeg))
        {
            grab(line, "observer_lon_deg", m.observerLonDeg);
            grab(line, "observer_alt_m", m.observerAltM);
            sawObs = true;
        }
        grab(line, "horizon_s", m.horizonS);
        grab(line, "step_s", m.stepS);
    }
    m.valid = sawObs && m.horizonS > 0.0 && m.stepS > 0.0;
    return m;
}

std::vector<TwinTle>
LoadTleDump(const std::string& path)
{
    std::vector<TwinTle> out;
    std::ifstream f(path);
    if (!f)
    {
        return out;
    }
    std::string a;
    std::string b;
    std::string c;
    while (std::getline(f, a) && std::getline(f, b) && std::getline(f, c))
    {
        if (b.size() < 69 || c.size() < 69 || b[0] != '1' || c[0] != '2')
        {
            continue; // not a TLE triple
        }
        out.push_back({a, b, c});
    }
    return out;
}

TwinAgreement
CompareHandovers(const std::vector<TwinHandover>& twin,
                 const std::vector<TwinHandover>& sim,
                 double windowS)
{
    TwinAgreement r;
    r.twinCount = twin.size();
    r.simCount = sim.size();
    if (twin.empty() || sim.empty())
    {
        return r;
    }

    std::vector<bool> used(sim.size(), false);
    double sumAbs = 0.0;
    for (const auto& t : twin)
    {
        // Nearest UNUSED sim event naming the SAME cell. Matching on the
        // instant alone would score a handover to the wrong satellite as
        // agreement, which is the failure this gate exists to catch.
        std::size_t best = sim.size();
        double bestAbs = windowS;
        for (std::size_t i = 0; i < sim.size(); ++i)
        {
            if (used[i] || sim[i].gnbId != t.gnbId)
            {
                continue;
            }
            const double d = std::abs(sim[i].tSec - t.tSec);
            if (d <= bestAbs)
            {
                bestAbs = d;
                best = i;
            }
        }
        if (best < sim.size())
        {
            used[best] = true;
            ++r.matched;
            sumAbs += bestAbs;
            r.maxAbsOffsetS = std::max(r.maxAbsOffsetS, bestAbs);
        }
    }
    r.meanAbsOffsetS = r.matched ? sumAbs / r.matched : 0.0;
    // Denominator is the LONGER sequence, so a twin that predicts two of the
    // simulator's twenty handovers cannot score 100%.
    r.agreement = static_cast<double>(r.matched) /
                  static_cast<double>(std::max(r.twinCount, r.simCount));
    return r;
}

} // namespace ntntwin
} // namespace ns3
