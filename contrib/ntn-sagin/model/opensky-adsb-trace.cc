/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#include "opensky-adsb-trace.h"

#include "ns3/log.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>

namespace ns3
{
namespace sagin
{

NS_LOG_COMPONENT_DEFINE("OpenSkyAdsbTrace");

double
OpenSkyAdsbTrace::TStart() const
{
    return samples.empty() ? std::numeric_limits<double>::quiet_NaN()
                             : samples.front().time_s;
}

double
OpenSkyAdsbTrace::TEnd() const
{
    return samples.empty() ? std::numeric_limits<double>::quiet_NaN()
                             : samples.back().time_s;
}

OpenSkySample
OpenSkyAdsbTrace::InterpolateAt(double time_s) const
{
    if (samples.empty())
    {
        return {};
    }
    if (time_s <= samples.front().time_s)
    {
        return samples.front();
    }
    if (time_s >= samples.back().time_s)
    {
        return samples.back();
    }
    auto it = std::upper_bound(samples.begin(),
                                 samples.end(),
                                 time_s,
                                 [](double t, const OpenSkySample& s) {
                                     return t < s.time_s;
                                 });
    const OpenSkySample& hi = *it;
    const OpenSkySample& lo = *(it - 1);
    const double dt = hi.time_s - lo.time_s;
    if (dt <= 0.0)
    {
        return lo;
    }
    const double f = (time_s - lo.time_s) / dt;
    OpenSkySample out{};
    out.time_s = time_s;
    out.lat_deg = lo.lat_deg + f * (hi.lat_deg - lo.lat_deg);
    out.lon_deg = lo.lon_deg + f * (hi.lon_deg - lo.lon_deg);
    out.alt_m = lo.alt_m + f * (hi.alt_m - lo.alt_m);
    out.velocity_mps = lo.velocity_mps + f * (hi.velocity_mps - lo.velocity_mps);
    // Heading: short-arc interpolation (handles wrap around 360 deg).
    double dh = hi.heading_deg - lo.heading_deg;
    if (dh > 180.0)
    {
        dh -= 360.0;
    }
    else if (dh < -180.0)
    {
        dh += 360.0;
    }
    out.heading_deg = lo.heading_deg + f * dh;
    if (out.heading_deg < 0.0)
    {
        out.heading_deg += 360.0;
    }
    else if (out.heading_deg >= 360.0)
    {
        out.heading_deg -= 360.0;
    }
    return out;
}

namespace
{

bool
ParseDouble(const std::string& s, double& out)
{
    if (s.empty())
    {
        return false;
    }
    try
    {
        out = std::stod(s);
        return std::isfinite(out);
    }
    catch (...)
    {
        return false;
    }
}

std::vector<std::string>
SplitCsv(const std::string& line)
{
    std::vector<std::string> out;
    std::string cur;
    bool inQuote = false;
    for (char c : line)
    {
        if (c == '"')
        {
            inQuote = !inQuote;
            continue;
        }
        if (c == ',' && !inQuote)
        {
            out.push_back(cur);
            cur.clear();
        }
        else
        {
            cur.push_back(c);
        }
    }
    out.push_back(cur);
    return out;
}

} // namespace

std::map<std::string, OpenSkyAdsbTrace>
OpenSkyAdsbImporter::LoadCsv(const std::string& path,
                              double tStartAbs_s,
                              double tEndAbs_s)
{
    std::map<std::string, OpenSkyAdsbTrace> out;
    m_lastRowsRead = 0;
    m_lastRowsSkipped = 0;

    std::ifstream f(path);
    if (!f)
    {
        NS_LOG_WARN("OpenSkyAdsbImporter: cannot open " << path);
        return out;
    }

    std::string header;
    if (!std::getline(f, header))
    {
        return out;
    }
    std::vector<std::string> cols = SplitCsv(header);
    auto idx = [&](const std::string& name) -> int {
        for (size_t i = 0; i < cols.size(); ++i)
        {
            if (cols[i] == name)
                return static_cast<int>(i);
        }
        return -1;
    };
    const int iTime = idx("time");
    const int iIcao = idx("icao24");
    const int iLat = idx("lat");
    const int iLon = idx("lon");
    const int iVel = idx("velocity");
    const int iHdg = idx("heading");
    const int iBaro = idx("baroaltitude");
    const int iGeo = idx("geoaltitude");
    const int iOnGr = idx("onground");
    if (iTime < 0 || iIcao < 0 || iLat < 0 || iLon < 0)
    {
        NS_LOG_WARN("OpenSkyAdsbImporter: required column missing");
        return out;
    }

    std::string line;
    while (std::getline(f, line))
    {
        if (line.empty())
        {
            continue;
        }
        ++m_lastRowsRead;
        std::vector<std::string> v = SplitCsv(line);
        if (static_cast<int>(v.size()) <= std::max({iTime, iIcao, iLat, iLon}))
        {
            ++m_lastRowsSkipped;
            continue;
        }
        double t;
        double lat;
        double lon;
        if (!ParseDouble(v[iTime], t) || !ParseDouble(v[iLat], lat) ||
            !ParseDouble(v[iLon], lon))
        {
            ++m_lastRowsSkipped;
            continue;
        }
        if (t < tStartAbs_s || t > tEndAbs_s)
        {
            ++m_lastRowsSkipped;
            continue;
        }
        const std::string& icao = v[iIcao];
        if (icao.empty())
        {
            ++m_lastRowsSkipped;
            continue;
        }
        bool onground = false;
        if (iOnGr >= 0 && static_cast<int>(v.size()) > iOnGr)
        {
            const std::string& og = v[iOnGr];
            onground = (og == "true" || og == "True" || og == "TRUE" ||
                         og == "1");
        }
        OpenSkySample s{};
        s.time_s = t;
        s.lat_deg = lat;
        s.lon_deg = lon;
        double alt_baro = std::numeric_limits<double>::quiet_NaN();
        double alt_geo = std::numeric_limits<double>::quiet_NaN();
        if (iBaro >= 0 && static_cast<int>(v.size()) > iBaro)
        {
            ParseDouble(v[iBaro], alt_baro);
        }
        if (iGeo >= 0 && static_cast<int>(v.size()) > iGeo)
        {
            ParseDouble(v[iGeo], alt_geo);
        }
        if (onground)
        {
            s.alt_m = 0.0;
        }
        else if (std::isfinite(alt_baro))
        {
            s.alt_m = alt_baro;
        }
        else if (std::isfinite(alt_geo))
        {
            s.alt_m = alt_geo;
        }
        else
        {
            s.alt_m = 0.0;
        }
        s.velocity_mps = 0.0;
        if (iVel >= 0 && static_cast<int>(v.size()) > iVel)
        {
            ParseDouble(v[iVel], s.velocity_mps);
        }
        s.heading_deg = 0.0;
        if (iHdg >= 0 && static_cast<int>(v.size()) > iHdg)
        {
            ParseDouble(v[iHdg], s.heading_deg);
        }
        auto& tr = out[icao];
        tr.icao24 = icao;
        tr.samples.push_back(s);
    }
    for (auto& kv : out)
    {
        std::sort(kv.second.samples.begin(),
                   kv.second.samples.end(),
                   [](const OpenSkySample& a, const OpenSkySample& b) {
                       return a.time_s < b.time_s;
                   });
    }
    NS_LOG_INFO("OpenSkyAdsbImporter: " << path << " => " << out.size()
                                          << " aircraft, " << m_lastRowsRead
                                          << " rows read, "
                                          << m_lastRowsSkipped
                                          << " skipped");
    return out;
}

} // namespace sagin
} // namespace ns3
